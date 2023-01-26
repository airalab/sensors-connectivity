from dataclasses import dataclass, field
from functools import reduce
import time

from ...constants import MOBILE_GPS, PASKAL2MMHG
from .base import Device


@dataclass(repr=False, eq=False)
class MobileLab(Device):
    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        self.id = self.data["ID"]
        self.model = MOBILE_GPS
        self.public = self.generate_pubkey(str(self.id))
        self.geo_lat = float(self.data.get("GPS_lat"))
        self.geo_lon = float(self.data.get("GPS_lon"))
        self.measurement = reduce(self._mobile_sensor_data_saver, self.data.items(), {})
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})
        self.measurement.update({"model": self.model})

    def _mobile_sensor_data_saver(self, meas: dict, value: str) -> dict:
        "Reducer callback for mobile GPS sensor's data"
        key, item = value
        if "GPS" in key or "ID" in key:
            return meas
        if "pressure" in key:
            meas[key] = float(item) / PASKAL2MMHG
        meas[key] = item
        return meas
