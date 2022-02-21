#!/usr/bin/env python3
import argparse
from os import name
import random
import requests
import json
import time
import logging.config

from connectivity.config.logging import LOGGING_CONFIG
# from config.logging import LOGGING_CONFIG


logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger(name="__name__")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Virtual Weather Station for Robonomics Connectivity Service"
    )
    parser.add_argument(
        "--remote",
        help="set server's URL (default: http://127.0.0.1:8001/)",
        default="http://127.0.0.1:8001/",
    )
    parser.add_argument(
        "--geo",
        default="59.944843, 30.294372",
        help="GPS position in lat,lon format (default: 59.944843, 30.294372)",
    )
    parser.add_argument(
        "--period",
        default=60,
        help="measurement period (default: 60 seconds)",
        type=int,
    )

    args = parser.parse_args()

    header = {"Content-type": "application/json"}

    geo = args.geo.split(",")
    lat = float(geo[0])
    lon = float(geo[1])
    period = args.period
    id = 843120
    url = args.remote

    while True:

        pm10 = round(random.uniform(0.1, 100.99), 3)
        pm25 = round(random.uniform(0.1, 100.99), 3)
        temp = round(random.uniform(-30, 30), 2)
        humidity = round(random.uniform(0, 100), 2)
        pressure = round(random.uniform(100, 300), 3)

        body = {
            "esp8266id": id,
            "software_version": "NRZ-2020-129",
            "sensordatavalues": [
                {"value_type": "SDS_P1", "value": pm10},
                {"value_type": "SDS_P2", "value": pm25},
                {"value_type": "temperature", "value": temp},
                {"value_type": "humidity", "value": humidity},
                {"value_type": "pressure", "value": pressure},
                {"value_type": "samples", "value": "890618"},
                {"value_type": "min_micro", "value": "43"},
                {"value_type": "max_micro", "value": "21069"},
                {"value_type": "GPS_lat", "value": lat},
                {"value_type": "GPS_lon", "value": lon},
                {"value_type": "signal", "value": "-46"},
            ],
        }
        try:
            response = requests.post(url, data=json.dumps(body), headers=header)
        except Exception as e:
            logger.warning(e)
            pass
        time.sleep(period)


if __name__ == "__main__":
    main()
