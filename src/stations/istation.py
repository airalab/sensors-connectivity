import time
from dataclasses import dataclass
import netifaces
from datetime import timedelta


@dataclass
class Measurement:
    pm25: float = 0
    pm10: float = 0

    def __str__(self):
        return f"{{PM2.5: {self.pm25}, PM10: {self.pm10}}}"


@dataclass
class StationData:
    version: str
    mac: str
    uptime: float
    measurement: Measurement

    def __str__(self):
        uptime = str(timedelta(seconds=self.uptime))
        return f"{{MAC: {self.mac}, Uptime: {uptime}, M: {self.measurement}}}"


def _get_mac() -> str:
    for interface in netifaces.interfaces():
        if interface != "lo":
            if 17 in netifaces.ifaddresses(interface):
                _i = netifaces.ifaddresses(interface)
                _i = _i[17][0]["addr"]
                break

    mac = _i.replace(":", "")
    return mac


class IStation:
    def __init__(self, config: dict):
        self.config = config
        self.version = "0.1.0"
        self.start_time = time.time()
        self.mac_address = _get_mac()

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            Measurement()
        )


__all__ = ["IStation", "Measurement", "StationData"]
