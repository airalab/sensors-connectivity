import time
import rospy
from datetime import timedelta
import netifaces
from sds011.sds011 import SDS011

BROADCASTER_VERSION = "v0.1.0"


class Measurement:
    def __init__(self, pm25, pm10):
        self.pm25 = pm25
        self.pm10 = pm10

    def __str__(self):
        return f"{{PM2.5: {self.pm25}, PM10: {self.pm10}}}"


class StationData:
    def __init__(self, ver: str, mac: str, uptime: float, meas: Measurement):
        self.version = ver
        self.mac = mac
        self.uptime = uptime
        self.meas = meas

    def to_json(self) -> dict:
        ret = {
            "software_version": self.version,
            "sensordatavalues": [
                {"value_type": "P1", "value": self.meas.pm10},
                {"value_type": "P2", "value": self.meas.pm25}
            ]
        }

        rospy.logdebug(ret)
        return ret

    def __str__(self):
        uptime = str(timedelta(seconds=self.uptime))
        return f"{{MAC: {self.mac}, Uptime: {uptime}, M: {self.meas}}}"


def _get_mac() -> str:
    for interface in netifaces.interfaces():
        if interface != 'lo':
            if 17 in netifaces.ifaddresses(interface):
                _i = netifaces.ifaddresses(interface)
                _i = _i[17][0]['addr']
                break

    mac = _i.replace(':', '')
    return mac


class Station:
    def __init__(self, sds_sensor_port: str = "/dev/ttyUSB0", work_time: int = 300):
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"
        self.start_time = time.time()
        self.mac_address = _get_mac()

        self.sensor = SDS011(sds_sensor_port)
        self.sensor.set_work_period(work_time=int(work_time / 60))

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        meas = self.sensor.query()

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            Measurement(meas[0], meas[1])
        )
