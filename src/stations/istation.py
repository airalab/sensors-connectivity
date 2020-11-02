# This is an interface for a station
import time
import netifaces
from datetime import timedelta
import json
from dataclasses import dataclass
import jsonpickle
from json import JSONEncoder
import rospy
import threading
import copy

STATION_VERSION = "v0.3.0"
thlock = threading.RLock()

class Measurement():
    """
    Represents a single measurement
    """

    # public: str     = ""
    # model: int      = 0
    # pm25: float     = 0
    # pm10: float     = 0
    # geo_lat: float  = 0
    # geo_lon: float  = 0
    # timestamp: int  = 0
    # temperature: float = None
    # pressure: float = None
    # humidity: float = None
    # ph: float = 0
    # conductivity: float = 0

    def __init__(self, public, model, geo_lat, geo_lon, measurement):
        self.public = public
        self.model = model
        self.geo_lat = geo_lat
        self.geo_lon = geo_lon
        self.measurement = measurement
         

    def to_json(self):
        with thlock:
            data_copy = copy.deepcopy(self.measurement)
            for key, value in data_copy.items():
                rospy.loginfo(f"inside to_json {value}")
                if value is None:
                    del self.measurement[key]
        return jsonpickle.encode(self.measurement)

    def __str__(self):

        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), measurements: {self.measurement}}}"


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

    def __repr__(self):
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
        self.version = STATION_VERSION
        self.start_time = time.time()
        self.mac_address = _get_mac()

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> [StationData]:
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


__all__ = ["IStation", "Measurement", "StationData"]
