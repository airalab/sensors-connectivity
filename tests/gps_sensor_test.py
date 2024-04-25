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
    lat = round(random.uniform(0.000000, 60.000000), 6)
    lon = round(random.uniform(0.000000, 49.999999), 6)
    humidity = round(random.uniform(0, 100))
    temperature = round(random.uniform(-50, 50), 3)
    id = 666
    body = {
        "esp8266id": id,
        "software_version": "NRZ-2020-129",
        "donated_by": "Robonomics",
        "model": 3,
        "sensordatavalues": [
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
