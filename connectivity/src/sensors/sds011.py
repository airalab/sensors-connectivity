from dataclasses import dataclass, field
from functools import reduce
import time

from ..drivers.sds011 import SDS011_MODEL
from ...utils.generate_pubkey import generate_pubkey


@dataclass
class SDS011:
    id: int = field(init=False)
    public: str = field(init=False)
    geo_lat: float = field(init=False)
    geo_lon: float = field(init=False)
    measurement: dict = field(init=False)
    data: dict = field(repr=False)
    model: int = SDS011_MODEL

    def __post_init__(self):
        self.id = int(self.data["esp8266id"])
        self.public = generate_pubkey(str(self.id))
        sensors_data = self.data["sensordatavalues"]
        for d in sensors_data:
            if d["value_type"] == "GPS_lat":
                self.geo_lat = d["value"]
            if d["value_type"] == "GPS_lon":
                self.geo_lon = d["value"]
        self.measurement = reduce(self._SDS011_values_saver, sensors_data, {})
        self.measurement.update({"timestamp": time.time()})

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

    def __str__(self) -> str:
        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), measurements: {self.measurement}}}"
