#include <SoftwareSerial.h>
#include <FuGPS.h>
#include "SdsDustSensor.h"
#include "secrets.h"    // use key generator to produce this file
#include <ESP8266WiFi.h>
#include <Ed25519.h>

#ifndef STASSID
#define STASSID $$STASSID$$
#define STAPSK  $$STAPSK$$
#endif

#define rxPinSensor $$RXPIN$$     // D2 on ESP TX of the sensor is connected to RX of the board
#define txPinSensor $$TXPIN$$     // D3 on ESP and vice versa sensor's rx is connected to boards's tx

#define rxPinGPS $$RXPINGPS$$
#define txPinGPS $$TXPINGPS$$

const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = $$HOST$$;
const uint16_t port = $$PORT$$;

const short model = 2;
const byte work_period = $$WORKPERIOD$$;     // minutes

float geo_lat = 0;
float geo_lon = 0;

uint8_t* privateKey = signing_key;
uint8_t* publicKey = verifying_key;

uint8_t buffer[16] = {0};
uint8_t signature[64] = {0};

SoftwareSerial in(rxPinGPS, txPinGPS);
FuGPS fuGPS(in);
bool gpsAlive = false;

SdsDustSensor sds(rxPinSensor, txPinSensor);
WiFiClient client;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  in.begin(9600);
  fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  sds.begin(9600);
  sds.setActiveReportingMode();
  sds.setCustomWorkingPeriod(work_period);

  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    delay(5000);
    return;
  }

  if (client.connected()) {
    client.write(verifying_key, sizeof(verifying_key));
    client.write((byte*)&model, sizeof(model));
    client.flush();
  }
}

void loop() {
  PmResult pm = sds.readPm();

  if (!client.connected()) {
    client.connect(host, port);
    client.write(verifying_key, sizeof(verifying_key));
    client.write((byte*)&model, sizeof(model));
    client.flush();
  }

  if (fuGPS.read()) {
    // We don't know, which message was came first (GGA or RMC).
    // Thats why some fields may be empty.

    gpsAlive = true;
    if (fuGPS.hasFix() == true) {
      geo_lat = fuGPS.Latitude;
      geo_lon = fuGPS.Longitude;
    }
  }

  if (fuGPS.isAlive() == false) {
    if (gpsAlive == true) {
        gpsAlive = false;
        Serial.println("GPS module not responding with valid data.");
        Serial.println("Check wiring or restart.");
    }
  }

  if(pm.isOk()) {
    Serial.print("Got data: ");
    Serial.print(pm.pm25);
    Serial.print(", ");
    Serial.println(pm.pm10);
    memcpy(buffer, (byte*) &pm.pm25, 4);
    memcpy(buffer+4, (byte*) &pm.pm10, 4);
    memcpy(buffer+8, (byte*) &geo_lat, 4);
    memcpy(buffer+12, (byte*) &geo_lon, 4);

    Ed25519::sign(signature, privateKey, publicKey, buffer, 16);
    client.write(buffer, sizeof(buffer));
    client.write(signature, sizeof(signature));
    client.flush();
  }

  delay(500);
}

