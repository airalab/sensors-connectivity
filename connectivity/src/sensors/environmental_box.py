import time
from dataclasses import dataclass, field
from functools import reduce

from ...constants import PASKAL2MMHG, SDS011_MODEL
from .base import Device


@dataclass(repr=False, eq=False)
class EnvironmentalBox(Device):
    """Represents a Robonomics Environmental Box.

    :param data: Unparsed data from the sensor.
    """

    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        self.id = str(self.data["esp8266id"])
        self.model = SDS011_MODEL
        self.public = self.generate_pubkey(str(self.id))
        sensors_data = self.data["sensordatavalues"]
        for d in sensors_data:
            if d["value_type"] == "GPS_lat":
                self.geo_lat = d["value"]
            if d["value_type"] == "GPS_lon":
                self.geo_lon = d["value"]
        self.measurement = reduce(self._SDS011_values_saver, sensors_data, {})
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})

    def _SDS011_values_saver(self, meas: dict, value: dict) -> dict:
        """Reducer callback for SDS011 sensors.
        :param meas: Unparsed data from the sensor.
        :param value: Current element (dict) of the meas dict.
        :return: Formatted data with the measurements.
        """

        extra_data = [
            "GPS",
            "micro",
            "signal",
            "samples",
            "interval",
        ]  # values which shouldn't be stored in meas dict
        if any(x in value["value_type"] for x in extra_data):
            return meas
        if "_" in value["value_type"] and not "CCS" in value["value_type"]:
            if "pressure" in value["value_type"]:
                meas[value["value_type"].split("_")[1]] = float(value["value"]) / PASKAL2MMHG
            else:
                meas[value["value_type"].split("_")[1]] = value["value"]
        else:
            meas[value["value_type"]] = value["value"]
        return meas
