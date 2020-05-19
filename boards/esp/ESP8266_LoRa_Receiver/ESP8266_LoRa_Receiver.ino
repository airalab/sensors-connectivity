#include <SoftwareSerial.h>

#define loraRxPin D5
#define loraTxPin D6

SoftwareSerial lora(loraRxPin, loraTxPin);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(loraRxPin, INPUT);
  pinMode(loraTxPin, OUTPUT);
  lora.begin(9600);
}

void loop() {
  if (lora.available() > 1) {
    String data = lora.readString();
    Serial.println(data);
  }

  delay(50);
}
