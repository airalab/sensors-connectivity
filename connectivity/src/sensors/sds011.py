from dataclasses import dataclass, field
from functools import reduce
import time

from ..drivers.sds011 import SDS011_MODEL
from .base import Device


@dataclass(repr=False, eq=False)
class EnvironmentalBox(Device):
    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        self.id = int(self.data["esp8266id"])
        self.model = SDS011_MODEL
        self.public = self.generate_pubkey(str(self.id))
        sensors_data = self.data["sensordatavalues"]
        for d in sensors_data:
            if d["value_type"] == "GPS_lat":
                self.geo_lat = d["value"]
            if d["value_type"] == "GPS_lon":
                self.geo_lon = d["value"]
        self.measurement = reduce(self._SDS011_values_saver, sensors_data, {})
        self.timestamp = time.time()
        self.measurement.update({"timestamp": self.timestamp})
        self.measurement.update({"model": self.model})


    def _SDS011_values_saver(self, meas: dict, value: dict) -> dict:
        "Reducer callback for SDS011 sensors"
        extra_data = [
            "GPS",
            "micro",
            "signal",
            "samples",
            "interval",
        ]  # values which shouldn't be stored in meas dict
        paskal = 133.32
        if any(x in value["value_type"] for x in extra_data):
            return meas
        if "_" in value["value_type"] and not "CCS" in value["value_type"]:
            if "pressure" in value["value_type"]:
                meas[value["value_type"].split("_")[1]] = float(value["value"]) / paskal
            else:
                meas[value["value_type"].split("_")[1]] = value["value"]
        else:
            meas[value["value_type"]] = value["value"]
        return meas
