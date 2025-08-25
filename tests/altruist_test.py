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
    co1 = round(random.uniform(0.1, 100.99), 3)
    pm25 = round(random.uniform(0.1, 100.99), 3)
    humidity = round(random.uniform(0, 100))
    temperature = round(random.uniform(-50, 50), 3)
    lat = round(random.uniform(0.000000, 60.000000), 6)
    lon = round(random.uniform(0.000000, 49.999999), 6)
    body = {
        "robonomics_address": "",
        "owner": "",
        "signature": "",
        "software_version": "NRZ-2020-129",
        "donated_by": "Robonomics",
        "GPS_lat": lat,
        "GPS_lon": lon,
        "sensordatavalues": f"nm:0.00,na:0.00,t:{temperature},p:360,h:{humidity},p1:{pm10},p2:{pm25},co1:{co1}"
    }   

    try:
        response = requests.post("http://127.0.0.1:31112/", data=json.dumps(body), headers=header)
    except Exception as e:
        logger.warning(e)
        pass
