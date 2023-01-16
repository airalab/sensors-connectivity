# This is an interface for a station
import time
from datetime import timedelta
import json
from dataclasses import dataclass, field
import threading
import copy
import typing as tp
import netifaces

STATION_VERSION = "v0.8.0"
thlock = threading.RLock()

@dataclass
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

    # measurement: dict = field(init=False)
    # """
    # The station is responsible for its own settings
    # :param config: configuration dictionary
    # """

    # def __str__(self):
    #     return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> tp.List[dict]:
        """
        Must return a new record of data or last measured data
        Depending on a configuration file this method could be called
        more often than new data is received
        :return: List of measurements
        """
        raise NotImplementedError("Subclass must implement get_data()!")
