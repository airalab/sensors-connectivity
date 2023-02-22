import time
from dataclasses import dataclass, field
from functools import reduce

from ...constants import MOBILE_GPS, PASKAL2MMHG
from .base import Device


@dataclass(repr=False, eq=False)
class MobileLab(Device):
    """Represents a sensors from mobile lab.

    :param data: Unparsed data from the sensor.
    """

    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        self.id = self.data["ID"]
        self.model = MOBILE_GPS
        self.public = self.generate_pubkey(str(self.id))
        self.geo_lat = float(self.data.get("GPS_lat"))
        self.geo_lon = float(self.data.get("GPS_lon"))
        self.measurement = reduce(self._mobile_sensor_data_saver, self.data.items(), {})
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})

    def _mobile_sensor_data_saver(self, meas: dict, value: tuple) -> dict:
        """Reducer callback for mobile GPS sensor's data
        
        :param meas: Unparsed data from the sensor.
        :param value: Current element (tuple) of the meas dict.
        :return: Formatted data with the measurements.
        """

        key, item = value
        if "GPS" in key or "ID" in key:
            return meas
        if "pressure" in key:
            meas[key] = float(item) / PASKAL2MMHG
        meas[key] = item
        return meas
