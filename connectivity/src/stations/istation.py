# This is an interface for a station
import copy
import threading
import time
import typing as tp
from dataclasses import dataclass, field

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

    DEAD_SENSOR_TIME: int = field(init=False)

    def drop_dead_sensors(self, sessions: dict) -> dict:
        """Drop sensors which have not sent measurements for more than
        `DEAD_SENSOR_TIME` interval. Interval is setted for each station.
        """

        stripped = dict()
        current_time = int(time.time())
        sessions_copy = copy.deepcopy(sessions)
        for k, v in sessions_copy.items():
            if (current_time - v.timestamp) < self.DEAD_SENSOR_TIME:
                stripped[k] = v
            else:
                del sessions[k]
        return stripped

    def get_data(self) -> tp.List[dict]:
        """
        Must return a new record of data or last measured data
        Depending on a configuration file this method could be called
        more often than new data is received
        :return: List of measurements
        """

        raise NotImplementedError("Subclass must implement get_data()!")
