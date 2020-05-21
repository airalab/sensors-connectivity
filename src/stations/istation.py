# This is an interface for a station
import time
from dataclasses import dataclass
import netifaces
from datetime import timedelta


@dataclass(frozen=True)
class Measurement:
    """
    Represents a single measurement
    """

    public: str     = ""
    model: int      = 0
    pm25: float     = 0
    pm10: float     = 0
    geo_lat: float  = 0
    geo_lon: float  = 0
    timestamp: int  = 0

    def __str__(self):
        return f"{{Public: {self.public}, PM2.5: {self.pm25}, PM10: {self.pm10}, geo: ({self.geo_lat},{self.geo_lon}), timestamp: {self.timestamp}}}"


@dataclass
class StationData:
    """
    It's a wrapper for a measurement with some additional information
    """

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
    """
    Station is an input/source of data

    station1 \                        / output1
    station2 -  sensors-connectivity  - output2
    station3 /                        \ output3

    Every station must implement `get_data()` method.

    Keep in mind `get_data()` can be called more often than actual data arrives.
    A good practice is to have a thread for data reading and a variable that keeps last record.
    Have a look at COMStation and TCPStation implementation.
    """

    def __init__(self, config: dict):
        """
        The station is responsible for its own settings

        :param config: configuration dictionary
        """

        self.config = config
        self.version = "0.1.0"
        self.start_time = time.time()
        self.mac_address = _get_mac()

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        """
        Must return a new record of data or last measured data

        Depending on a configuration file this method could be called
        more often than new data is received

        :return: StationData object
        """

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            Measurement()
        )


__all__ = ["IStation", "Measurement", "StationData"]
