import threading
import rospy

from stations import *
import time
from drivers import SDS011
from collections import deque

BROADCASTER_VERSION = "v0.1.0"


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
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"

        self.sensor = SDS011(self.config["comstation"]["port"])

        work_period = int(self.config["comstation"]["work_period"])
        self.sensor.set_work_period(work_time=int(work_period / 60))

        self.q = deque(maxlen=1)
        threading.Thread(target=_read_data_thread, args=(self.sensor, self.q, work_period)).start()

    def get_data(self) -> StationData:
        if self.q:
            values = self.q[-1]
            # rospy.loginfo(values)
            pm = values[0]
            meas = Measurement(pm[0], pm[1], values[1])
        else:
            meas = Measurement()

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            meas
        )
