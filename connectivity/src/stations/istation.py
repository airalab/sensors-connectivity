# This is an interface for a station
import time
import netifaces
from datetime import timedelta
import json
from dataclasses import dataclass
import threading
import copy
import typing as tp

STATION_VERSION = "v0.8.0"
thlock = threading.RLock()

class Measurement():

    """
    Represents a single measurement
    """
    
    def __init__(self, public: str, model: int, geo_lat: float, geo_lon: float, measurement: dict) -> None:
        self.public: str = public
        self.model: int = model
        self.geo_lat: float = geo_lat
        self.geo_lon: float = geo_lon
        self.measurement: dict = measurement
         

    def measurement_check(self) -> dict:
        with thlock:
            data_copy = copy.deepcopy(self.measurement)
            for key, value in data_copy.items():
                if value is None:
                    del self.measurement[key]
        return self.measurement

    def __str__(self) -> str:

        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), measurements: {self.measurement_check()}}}"


@dataclass
class StationData:
    """
    It's a wrapper for a measurement with some additional information
    """

    version: str
    mac: str
    uptime: float
    measurement: Measurement

    def __str__(self) -> str:
        uptime = str(timedelta(seconds=self.uptime))
        return f"{{MAC: {self.mac}, Uptime: {uptime}, M: {self.measurement}}}"

    def __repr__(self) -> str:
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
    Have a look at COMStation and HTTPStation implementation.
    """

    def __init__(self, config: dict) -> None:
        """
        The station is responsible for its own settings
        :param config: configuration dictionary
        """

        self.config: dict = config
        self.version: tp.Literal["v0.8.0"] = STATION_VERSION
        self.start_time: float = time.time()
        self.mac_address: str = _get_mac()

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> tp.List[StationData]:
        """
        Must return a new record of data or last measured data
        Depending on a configuration file this method could be called
        more often than new data is received
        :return: StationData object
        """

        return [StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            Measurement()
        )]

