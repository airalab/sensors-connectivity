"""
This script allows to test HTTP Station on several servers. Defaults servers are localhost on port 31133
and 31112.
"""
from asyncio import tasks
from cmath import log
import random
import requests, json
import time
import logging.config
from connectivity.config.logging import LOGGING_CONFIG
import asyncio

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")
print(logger)


class Sensor:
    def __init__(self, id: int, coords: tuple[float]):
        self.id: int = id
        self.lat: float = coords[0]
        self.lon: float = coords[1]
        self.list_of_servers = ["localhost:31113", "localhost:31112"]

    def get_server(self) -> str:
        header = {"Sensor-id": f"{self.id}"}
        min_sensors = 255
        for server in self.list_of_servers:
            response = requests.get(f"http://{server}/", headers=header)
            logger.info(f"Sensor {self.id}. Responce from server {server} is {response.headers}")
            sensors_count = response.headers["sensors-count"]
            on_server = response.headers["on-server"]
            if on_server == "True":
                return server
            if int(sensors_count) < min_sensors:
                min_sensors = int(sensors_count)
                robonomics_host = server
        return robonomics_host

    def post_data(self, server_with_port: str) -> None:
        header = {"Content-type": "application/json"}
        pm10: float = round(random.uniform(0.1, 100.99), 3)
        pm25: float = round(random.uniform(0.1, 100.99), 3)
        humidity: float = round(random.uniform(0, 100))
        temperature: float = round(random.uniform(-50, 50), 3)
        body = {
            "esp8266id": self.id,
            "software_version": "NRZ-2020-129",
            "sensordatavalues": [
                {"value_type": "SDS_P1", "value": pm10},
                {"value_type": "SDS_P2", "value": pm25},
                {"value_type": "temperature", "value": temperature},
                {"value_type": "humidity", "value": humidity},
                {"value_type": "samples", "value": "890618"},
                {"value_type": "min_micro", "value": "43"},
                {"value_type": "max_micro", "value": "21069"},
                {"value_type": "GPS_lat", "value": self.lat},
                {"value_type": "GPS_lon", "value": self.lon},
                {"value_type": "signal", "value": "-46"},
            ],
        }

        try:
            response = requests.post(f"http://{server_with_port}/", data=json.dumps(body), headers=header)
            logger.info(f"Sensor {self.id}. Response for post request is {response}")
        except Exception as e:
            logger.warning(f"Sensor {self.id}. Exception in post request: {e}")
        pass

    async def spin(self, interval: int) -> None:
        while True:
            server = self.get_server()
            logger.info(f"Sensor {self.id}. Chosen server {server}")
            self.post_data(server)
            await asyncio.sleep(interval)
            logger.info("\n")


async def run_sensors():
    ids = ["12345", "22345", "32345", "42345", "52345"]
    coords = [
        (round(random.uniform(0.000000, 60.000000), 6), round(random.uniform(0.000000, 49.999999), 6))
        for i in range(len(ids))
    ]
    futures = [Sensor(ids[i], coords[i]).spin(10) for i in range(len(ids))]
    for i, future in enumerate(asyncio.as_completed(futures)):
        result = await future
        logger.info(result)


def main():
    ioloop = asyncio.get_event_loop()
    ioloop.run_until_complete(run_sensors())
    ioloop.close()
