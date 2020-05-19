#include "SdsDustSensor.h"

#define rxPin 2     // TX of the sensor is connected to RX of Arduino
#define txPin 3     // and vice versa sensor's rx is connected to arduino's tx

const byte work_period = 5;         // minutes

SdsDustSensor sds(rxPin, txPin);

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
    Serial.print("{\"pm25\":");
    Serial.print(pm.pm25);
    Serial.print(",\"pm10\":");
    Serial.print(pm.pm10);
    Serial.println("}");
  }

  delay(500);
}
