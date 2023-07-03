import json
import logging.config
import random
import time

import requests

from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger(__name__)

header = {"Content-type": "application/json"}


def main():
    pm10 = round(random.uniform(0.1, 100.99), 3)
    pm25 = round(random.uniform(0.1, 100.99), 3)
    humidity = round(random.uniform(0, 100))
    temperature = round(random.uniform(-50, 50), 3)
    lat = round(random.uniform(0.000000, 60.000000), 6)
    lon = round(random.uniform(0.000000, 49.999999), 6)
    id = round(random.randrange(0, 100000))
    body = {
        "esp8266id": id,
        "software_version": "NRZ-2020-129",
        "donated_by": "Robonomics",
        "sensordatavalues": [
            {"value_type": "SDS_P1", "value": pm10},
            {"value_type": "SDS_P2", "value": pm25},
            {"value_type": "temperature", "value": temperature},
            {"value_type": "humidity", "value": humidity},
            {"value_type": "samples", "value": "890618"},
            {"value_type": "min_micro", "value": "43"},
            {"value_type": "max_micro", "value": "21069"},
            {"value_type": "GPS_lat", "value": lat},
            {"value_type": "GPS_lon", "value": lon},
            {"value_type": "signal", "value": "-46"},
        ],
    }

    try:
        response = requests.post("http://127.0.0.1:31112/", data=json.dumps(body), headers=header)
    except Exception as e:
        logger.warning(e)
        pass
