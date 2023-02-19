import logging.config
import threading
import time
import typing as tp
from collections import deque

import nacl.signing

from connectivity.config.logging import LOGGING_CONFIG

from ...constants import STATION_VERSION
from ..drivers.sds011 import SDS011
from ..sensors import SensorSDS011
from .istation import IStation

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


def _read_data_thread(sensor: SDS011, q: deque, timeout: int) -> None:
    while True:
        meas = sensor.query()
        q.append((meas))
        time.sleep(timeout)


class COMStation(IStation):
    """Reads data from a serial port."""

    def __init__(self, config: dict) -> None:
        """Safe the last data from a serial port.
        
        :param config: Dict with configuration file.
        """

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

        self.initial_data = [0, 0]
        self.q = deque(maxlen=1)
        threading.Thread(target=_read_data_thread, args=(self.sensor, self.q, work_period)).start()

    def get_data(self) -> tp.List[dict]:
        """Main function of the stations.
        
        :return: Formatetd data.
        """
        
        if self.q:
            values = self.q[0]
            pm = values[0]
            meas = SensorSDS011(public_key=self.public, data=pm, geo=self.geo)
        else:
            meas = SensorSDS011(public_key=self.public, data=self.initial_data, geo=self.geo)

        return [meas]
