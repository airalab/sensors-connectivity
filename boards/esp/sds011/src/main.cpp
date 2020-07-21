#include "SdsDustSensor.h"
#include "secrets.h"    // use key generator to produce this file

#define rxPin $$RXPIN$$     // D2 on ESP TX of the sensor is connected to RX of the board
#define txPin $$TXPIN$$     // D3 on ESP and vice versa sensor's rx is connected to boards's tx

const float GEO_LAT = $$GEOLAT$$;
const float GEO_LON = $$GEOLON$$;

const short model = 2;
const byte work_period = $$WORKPERIOD$$;     // minutes

SdsDustSensor sds(rxPin, txPin);

void print_bytes(uint8_t *buffer, int len) {
  for(int i = 0; i < len; ++i) {
    Serial.print(buffer[i], HEX);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  sds.begin(9600);
  sds.setActiveReportingMode();
  sds.setCustomWorkingPeriod(work_period);
}

void loop() {
  PmResult pm = sds.readPm();

  if(pm.isOk()) {
    Serial.print("{\"");
    print_bytes(verifying_key, sizeof(verifying_key));
    Serial.print("\": {\"pm25\": ");
    Serial.print(pm.pm25);
    Serial.print(", \"pm10\": ");
    Serial.print(pm.pm10);
    Serial.print(", \"geo\": \"");
    Serial.print(GEO_LAT, 6);
    Serial.print(",");
    Serial.print(GEO_LON, 6);
    Serial.println("\"}}");
    Serial.flush();
   }

  delay(500);
}

