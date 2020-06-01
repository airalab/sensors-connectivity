import threading
import rospy

from stations import *
import nacl.signing
import time
from drivers.sds011 import SDS011, SDS011_MODEL
from collections import deque


def _read_data_thread(sensor: SDS011, q: deque, timeout: int):
    while True:
        meas = sensor.query()
        timestamp = int(time.time())
        q.append((meas, timestamp))

        time.sleep(timeout)


class COMStation(IStation):
    """
    Reads data from a serial port
    """

    def __init__(self, config: dict):
        super().__init__(config)
        self.version = f"airalab-com-{STATION_VERSION}"

        self.sensor = SDS011(config["comstation"]["port"])

        work_period = int(config["comstation"]["work_period"])
        self.sensor.set_work_period(work_time=int(work_period / 60))

        self.geo = [0, 0]
        if config["comstation"]["geo"]:
            self.geo = config["comstation"]["geo"].split(",")

        if "public_key" in config["comstation"] and config["comstation"]["public_key"]:
            self.public = config["comstation"]["public_key"]
        else:
            signing_key = nacl.signing.SigningKey.generate()
            verify_key = signing_key.verify_key

            self.public = bytes(verify_key).hex()

        rospy.loginfo(f"COMStation public key: {self.public}")

        self.q = deque(maxlen=1)
        threading.Thread(target=_read_data_thread, args=(self.sensor, self.q, work_period)).start()

    def get_data(self) -> [StationData]:
        meas = Measurement()
        if self.q:
            values = self.q[0]
            pm = values[0]

            meas = Measurement(self.public,
                               SDS011_MODEL,
                               pm[0],
                               pm[1],
                               float(self.geo[0]),
                               float(self.geo[1]),
                               values[1])

        return [StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            meas
        )]

