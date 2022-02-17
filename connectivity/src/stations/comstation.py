import threading
import typing
import nacl.signing
import time
import typing as tp
import logging.config

from .istation import IStation, StationData, STATION_VERSION, Measurement
from ..drivers.sds011 import SDS011_MODEL, SDS011
from collections import deque
from connectivity.config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


def _read_data_thread(sensor: SDS011, q: deque, timeout: int) -> None:
    while True:
        meas = sensor.query()
        timestamp = int(time.time())
        q.append((meas, timestamp))
        time.sleep(timeout)


class COMStation(IStation):
    """
    Reads data from a serial port
    """

    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self.version: str = f"airalab-com-{STATION_VERSION}"

        self.sensor: SDS011 = SDS011(config["comstation"]["port"])

        work_period: int = int(config["comstation"]["work_period"])
        self.sensor.set_work_period(work_time=int(work_period / 60))

        self.geo: tp.List[float, float] = [0, 0]
        if config["comstation"]["geo"]:
            self.geo = config["comstation"]["geo"].split(",")

        if "public_key" in config["comstation"] and config["comstation"]["public_key"]:
            self.public = config["comstation"]["public_key"]
        else:
            signing_key = nacl.signing.SigningKey.generate()
            verify_key = signing_key.verify_key

            self.public = bytes(verify_key).hex()
        logger.info(f"COMStation public key: {self.public}")

        self.meas_data = {"pm25": 0, "pm10": 0, "timestamp": 0}
        self.q = deque(maxlen=1)
        threading.Thread(
            target=_read_data_thread, args=(self.sensor, self.q, work_period)
        ).start()

    def get_data(self) -> tp.List[StationData]:
        meas = Measurement(self.public, SDS011_MODEL, 0, 0, self.meas_data)
        if self.q:
            values = self.q[0]
            pm = values[0]
            self.meas_data.update(
                {"pm25": pm[0], "pm10": pm[1], "timestamp": values[1]}
            )
            meas = Measurement(
                self.public,
                SDS011_MODEL,
                float(self.geo[0]),
                float(self.geo[1]),
                self.meas_data,
            )

        return [
            StationData(
                self.version, self.mac_address, time.time() - self.start_time, meas
            )
        ]
