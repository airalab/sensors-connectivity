# Firmwares

This folder contains firmwares for Arduino and ESP8266 (possibly ESP32) boards.

## Pre-requirements
Every board is meant to be shipped with `Ed25519::sign()` method. For this needs it requires `signing` and `verifying` keys.

In order to create ones there is the utility script in `utils` folder.

```
# in the package's root after building the package do
source result/setup.bash
./utils/generate_secrets.py -o boards/esp/ESP_TCP/
```

After that open `boards/esp/ESP_TCP/ESP_TCP.ino` file in Arduino IDE

## Install Dependencies

The firmware dependencies are:

* SDS011 Library - [link](https://github.com/lewapek/sds-dust-sensors-arduino-library) - via Library Manager of Arduino IDE
* Ed25519 - [link](https://github.com/rweather/arduinolibs) - download and extract `libraries/Crypto` to `~/Arduino/libraries`

Possibly you would be missing ESP boards in Arduino IDE. If so install it as well via Boards Manager

## Upload Firmware

Don't forget to set your WiFi's name and password!

Also set `HOST` of your connectivity instance.

You might want to set geo position as well. To summarize these are the options you need to set:

```
#ifndef STASSID
#define STASSID ""
#define STAPSK  ""
#endif

#define rxPin 2     // D2 on ESP TX of the sensor is connected to RX of the board
#define txPin 3     // D3 on ESP and vice versa sensor's rx is connected to boards's tx

const char* host = "HOST";
const uint16_t port = 31313;

const float GEO_LAT = 0.0;
const float GEO_LON = 0.0;

const byte work_period = 5;     // minutes
```

Upload the firmware as usual via Arduino IDE

