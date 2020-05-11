#include <HardwareSerial.h>
#include <ESP8266WiFi.h>

#ifndef STASSID
#define STASSID "your-ssid"
#define STAPSK  "your-password"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "HOST";
const uint16_t port = 31313;

#define rxPin 2     // TX of the sensor is connected to RX of the board
#define txPin 3     // and vice versa sensor's rx is connected to boards's tx

#define HEAD 0xAA
#define TAIL 0xAB
#define CMD_ID 0xB4
#define WRITE 0x01
#define REPORT_MODE_CMD 0x02
#define WORK_PERIOD_CMD 0x08

const byte work_period = 5;   // 0 <= work_period <= 30 minutes

HardwareSerial mySensor(1);

void print_cmd(byte *cmd) {
  Serial.print("[ ");
  for(int i = 0; i < 19; i++) {
    Serial.print((int)cmd[i]);
    Serial.print(" ");
  }
  Serial.println("]");
}

void execute(byte *buf) {
  //Serial.println("Execute");
  //print_cmd(buf);
  mySensor.write(buf, 19);
}

int get_reply(float *p25, float *p10) {
  byte buffer;
  int value;
  int len = 0;
  int pm10_serial = 0;
  int pm25_serial = 0;
  int checksum_is;
  int checksum_ok = 0;
  int error = 1;
  while (mySensor.available() > 0) {
    buffer = mySensor.read();
    value = int(buffer);
    switch (len) {
      case (0): if (value != 170) { len = -1; }; break;
      case (1): if (value != 192) { len = -1; }; break;
      case (2): pm25_serial = value; checksum_is = value; break;
      case (3): pm25_serial += (value << 8); checksum_is += value; break;
      case (4): pm10_serial = value; checksum_is += value; break;
      case (5): pm10_serial += (value << 8); checksum_is += value; break;
      case (6): checksum_is += value; break;
      case (7): checksum_is += value; break;
      case (8): if (value == (checksum_is & 255)) { checksum_ok = 1; } else { len = -1; }; break;
      case (9): if (value != 171) { len = -1; }; break;
    }
    //Serial.println(len);
    len++;
    if (len == 10 && checksum_ok == 1) {
      //Serial.println("Returning values");
      *p10 = (float)pm10_serial/10.0;
      *p25 = (float)pm25_serial/10.0;
      len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
      error = 0;
    }
  }
  return error;
}

byte checksum(byte *cmd) {
  int c = 0;
  for(int i = 2; i < 17; i++) {
    c += (int) cmd[i];
  }

  return (byte)(c % 256);
}

void set_active_mode() {
  byte cmd[] = {
    HEAD, CMD_ID, REPORT_MODE_CMD, WRITE,
    0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0xFF, 0xFF, 0x0, TAIL
  };

  cmd[17] = checksum(cmd);
  execute(cmd);
}

void set_work_period() {
  byte cmd[] = {
    HEAD, CMD_ID, WORK_PERIOD_CMD, WRITE,
    work_period, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0xFF, 0xFF, 0x0, TAIL
  };

  cmd[17] = checksum(cmd);
  execute(cmd);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySensor.begin(9600, SERIAL_8N1, rxPin, txPin);

  set_active_mode();
  set_work_period();
}

void loop() {
  float pm25, pm10;
  int error = get_reply(&pm25, &pm10);

  if(error == 0) {
    String buff = "{\"pm2.5\":";
    buff += String(pm25);
    buff += ",\"pm10\":";
    buff += String(pm10);
    buff += "}";
    Serial.println(buff);

    WiFiClient client;
    if (!client.connect(host, port)) {
      Serial.println("connection failed");
      delay(5000);
      return;
    }

    if (client.connected()) {
      client.println(buff);
    }

    client.stop();
  }

  delay(500);
}
