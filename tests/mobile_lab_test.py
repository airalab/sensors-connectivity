import random
import requests, json
import time
import logging.config
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger(__name__)

header = {"Content-type": "application/json"}


def main():
    humidity = round(random.uniform(0, 100))
    temperature = round(random.uniform(-50, 50), 3)
    lat = round(random.uniform(0.000000, 60.000000), 6)
    lon = round(random.uniform(0.000000, 49.999999), 6)
    body = {
        "ID": "A4B9274B",
        "temperature": temperature,
        "humidity": humidity,
        "pressure": "10001",
        "CO": "45.0006",
        "NO2": "0.0027",
        "NH3": "0.00013",
        "GPS_lat": lat,
        "GPS_lon": lon,
    }

    try:
        response = requests.post(
            "http://127.0.0.1:31113/", data=json.dumps(body), headers=header
        )
    except Exception as e:
        logger.warning(e)
        pass
