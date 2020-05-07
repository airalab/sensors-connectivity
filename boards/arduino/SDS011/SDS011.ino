#include <SoftwareSerial.h>

#define rxPin 2
#define txPin 3

#define HEAD 0xAA
#define TAIL 0xAB
#define CMD_ID 0xB4
#define REPORT_MODE_CMD 0x02
#define WRITE 0x01
#define PASSIVE 0x01
#define WORK_PERIOD_CMD 0x08
#define QUERY_CMD 0x04

const int timeout = 1 * 15 * 1000;  // miliseconds
const byte work_period = 1;   // minutes

SoftwareSerial mySensor(rxPin, txPin);  // RX, TX of the sensor

void print_cmd(byte *cmd) {
  Serial.print("[ ");
  for(int i = 0; i < 19; i++) {
    Serial.print((int)cmd[i]);
    Serial.print(" ");
  }
  Serial.println("]");
}

void execute(byte *buf) {
  Serial.println("Execute");
  print_cmd(buf);
  mySensor.write(buf, 19);
}

int get_reply(float *p25, float *p10) {
  //Serial.println("get reply");
  byte buffer;
  int value;
  int len = 0;
  int pm10_serial = 0;
  int pm25_serial = 0;
  int checksum_is;
  int checksum_ok = 0;
  int error = 1;
  while (mySensor.available() > 0) {
  // while(len < 10) {
    //Serial.println("Start while");
    buffer = mySensor.read();
    value = int(buffer);
    //Serial.println(value);
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
  Serial.println(error);
  return error;
}

byte checksum(byte *cmd) {
  int c = 0;
  for(int i = 2; i < 17; i++) {
    c += (int) cmd[i];
  }

  //Serial.println(c % 256);
  return (byte)(c % 256);
}

void set_report_mode() {
  //Serial.println("set report mode");
  /* byte cmd[] = {
    HEAD, CMD_ID, REPORT_MODE_CMD, WRITE,
    PASSIVE, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0xFF, 0xFF, 0x0, TAIL
  }; */
  byte cmd[] = {
    HEAD, CMD_ID, REPORT_MODE_CMD, 0x0,
    0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0xFF, 0xFF, 0x0, TAIL
  };

  cmd[17] = checksum(cmd);

  execute(cmd);
  float *pm25, *pm10;
  get_reply(pm25, pm10);
}

void set_work_period() {
  //Serial.println("Set work period");
  
  byte cmd[] = {
    HEAD, CMD_ID, WORK_PERIOD_CMD, WRITE,
    work_period, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0xFF, 0xFF, 0x0, TAIL
  };

  cmd[17] = checksum(cmd);

  execute(cmd);
  float *pm25, *pm10;
  get_reply(pm25, pm10);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySensor.begin(9600);

  set_report_mode();
  set_work_period();
}

void loop() {

  byte cmd[] = {
    HEAD, CMD_ID, QUERY_CMD, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0xFF, 0xFF, 0x0, TAIL
  };

  cmd[17] = checksum(cmd);

  execute(cmd);
  float pm25, pm10;
  int error = get_reply(&pm25, &pm10);

  if(error == 0) {
    Serial.print("{\"pm2.5\":");
    Serial.print(pm25);
    Serial.print(",\"pm10\":");
    Serial.print(pm10);
    Serial.println("}");
  }

  delay(timeout);
}
