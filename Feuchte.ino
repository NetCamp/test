#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include "WEBset.h"  // Hier ist die HTML-Webseite als Array abgelegt

#define LEDpin 2    		// LED zum Blinken. Bei ESP-07 Pin 2. Bei ESP-01 Pin 1
#define Sensor_PIN    13 	// Pinn mit Vorwiderstand von 330 Ohm
#define Load_PIN    12   	// Pinn mit Vorwiderstand von 1 KOhm 
#define RTC_Wert 3600e6  	//    1Stunde

#define SchlafZyklen 6 		// Nach 6 Stunden wird gemessen

//#define seriell; 			// Einschalter für serielle Ausgabe
IPAddress apIP(192, 168, 178, 1);  // IP-Adresse für Setup

ADC_MODE (ADC_VCC);			// ADC an das Stromversorgung Der ADC-Pinn soll nicht angeschlossen sein
int z = 0; 					//Aktuelle EEPROM-Adresse zum lesen
String nachricht = "Testnachricht";
String apiKey = "";
int timout = 0;
unsigned int Messwert[2];

char ssid[32] = "fhgdhfghd\0";
char passwort[64] = "gghdfgh\0";
ESP8266WebServer server;
EspClass ESPm;
HTTPClient http;
#include "Setup.h"
extern "C" {
#include "user_interface.h"
}

void setup()
{
  //------------------------------------ Prüfen ob Tx und Rx verbunden sind --------------
  pinMode(1, OUTPUT); // Tx
  pinMode(3, INPUT_PULLUP); // Rx
  digitalWrite(1, 1);
  delayMicroseconds(2);
  if (!digitalRead(3))goto weiter;  // Prüfen ob Tx und Rx verbunden sind

  digitalWrite(1, 0);
  delayMicroseconds(2);
  if (digitalRead(3))goto weiter;
  Einstellen();                      // TxD ist mit RxD verbunden Setupmodus (Setup.h)
weiter:

  Serial.begin(115200);

  unsigned int Ziklus[2];              // Zykluszehler
  const rst_info * resetInfo = system_get_rst_info();  // Resetgrund einlesen
  ESPm.rtcUserMemoryRead(0, Ziklus, sizeof(Ziklus));  // Zyklus Nummer einlesen
  if (((resetInfo->reason) == 6) || ((resetInfo->reason) == 0))
  {
    Ziklus[0] = 0;         // Wenn Spannung OFF
    Ziklus[1] = 0;
    ESPm.rtcUserMemoryWrite(4, 0, 2);
  }
  Ziklus[1]++;
  ESPm.rtcUserMemoryWrite(0, Ziklus, sizeof(Ziklus)); // Zyklus Nummer speichern
# if defined  seriell
  Serial.println("");
  Serial.print("Ziklys");
  Serial.println(Ziklus[1] - 1);
# endif

  if (Ziklus[1] > SchlafZyklen + 1)
  {
    Ziklus[1] = 1;
    Ziklus[0]++;    	// Anzahl Messungen seit dem anlegen der Spannung
    ESPm.rtcUserMemoryWrite(0, Ziklus, sizeof(Ziklus)); // Zyklus Nummer speichern
    goto Wecken;
  }

  if (Ziklus[1] != SchlafZyklen + 1) ESPm.deepSleep(RTC_Wert, WAKE_RF_DISABLED);
  else
  {
    FeuchtMessung ();
    ESPm.deepSleep(100000, WAKE_RF_DEFAULT); // Nach 100mS Wach und Radio ON
  }
  delay(100);

Wecken:
  uint ADCValue = 0;						// Batteriespannung messen
  ADCValue = ESPm.getVcc();					//  hier Korrektur Wert eintragen
  float ADCfloat = float(ADCValue);
  String Ubatt = "";
  Ubatt = String(ADCfloat / 1000, 2) + "V";

  EEPROM.begin(250);  				// EEPROM initialisieren
  z = 0;                            // Einstellungen einlesen
  LeseEeprom(ssid, sizeof(ssid));
  LeseEeprom(passwort, sizeof(passwort));
  LeseEepromStr(&apiKey, 100);
  LeseEepromStr(&nachricht, 100);

  WiFi.mode(WIFI_STA);
# if defined  seriell
  Serial.println();
  Serial.print("Ziklus: ");
  Serial.println(Ziklus[0] - 1);
  Serial.print("U Batt:  ");
  Serial.println(Ubatt);
  Serial.print("Connecting to ");
  Serial.println (ssid);
  // Serial.print(" Pass: ");
  // Serial.println (passwort);
  Serial.println (apiKey);
  Serial.println (nachricht);
#endif

  WiFi.begin(ssid, passwort);

#if not defined seriell       // Vorbereiten das LED
  pinMode(LEDpin, OUTPUT);
#endif

  while (WiFi.status() != WL_CONNECTED)
  {
    timout++;
    if  (timout > 60) // Wenn Anmeldung nicht möglich
    {
#if defined seriell
      Serial.print("Netzwerk nicht gefunden");
#endif

      ESPm.deepSleep(RTC_Wert, WAKE_RF_DISABLED);
      delay(100);
    }
    // End timeout
#if defined seriell
    delay(1000);
    Serial.print("O");
#else
    digitalWrite(LEDpin, 0);
    delay(100);
    digitalWrite(LEDpin, 1);

    delay(100);
#endif
  }                         // Verbunden

  //Serial.println(Ziklus[0]);
  ESPm.rtcUserMemoryRead(4, Messwert, sizeof(Messwert));  // Zyklus Nummer einlesen
  //Serial.print("Sende Messwert: ");
  //Serial.println(Messwert[0]);

  int32_t rssi = wifi_station_get_rssi(); // Signalstärke einlesen
  String postStr = "key=" + apiKey;
  postStr += "&field1=" + String(Messwert[0]);
  postStr += "&field2=" + Ubatt;
  postStr += "&field3=" + String(Ziklus[0] - 1);
  postStr += "&field4=" + String(rssi);

#if defined  seriell
  Serial.begin(115200);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Signal strength: ");
  Serial.print(rssi);
  Serial.println("dBm");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("connecting to http://api.thingspeak.com/update");

#endif

  http.begin("http://api.thingspeak.com/update"); //HTTP
  int httpCode = http.POST(postStr);

  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
# if defined  seriell
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);
#endif
    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
# if defined  seriell
      Serial.println(payload);
#endif
    }
  } else {
# if defined  seriell
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
#else
    http.end();

    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, 0);
    delay(5000);
    digitalWrite(LEDpin, 1);
    ESPm.deepSleep(RTC_Wert, WAKE_RF_DISABLED); //gehe schlafen
#endif
  }
  http.end();

# if defined  seriell
  Serial.println();
  Serial.println("closing connection");
  delay(100);
#endif
  ESPm.deepSleep(RTC_Wert, WAKE_RF_DISABLED);
  delay(100);
}

void loop()
{
  yield();
}

void FeuchtMessung ()
{
  unsigned int feuchte = 0;
  Messwert[0] = 0;
  int i = 0;
  for (i = 0; i < 10; i++ )
  {
    pinMode(Sensor_PIN , OUTPUT);
    pinMode(Load_PIN , OUTPUT);
    digitalWrite(Load_PIN , 0);
    digitalWrite(Sensor_PIN , 0);
    delay(500);
    pinMode(Sensor_PIN , INPUT);
    digitalWrite(Load_PIN , 1);
    feuchte = 0;
    while (!digitalRead(Sensor_PIN))
    {
      feuchte++;
      if (feuchte > 500000)break;
    }
    Messwert[0] += feuchte;
    if (feuchte > 500000)break;
  }
  pinMode(Load_PIN , INPUT);
  if (feuchte < 500000)Messwert[0] = Messwert[0] / 10;
  else Messwert[0] = 500000;
# if defined  seriell
  Serial.print("Gemessen: ");
  Serial.println(Messwert[0]);
# endif
  ESPm.rtcUserMemoryWrite(4, Messwert, sizeof(Messwert));
}
