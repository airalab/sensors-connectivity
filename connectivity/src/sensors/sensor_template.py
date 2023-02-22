import time
from dataclasses import dataclass, field

from .base import Device


@dataclass(repr=False, eq=False)
class SensorName(Device):
    """
    Template for new sensor. You need to initialise all variables from base class.
    Have a look at the example below.

    :param data: Unparsed data from the sensor.
    """

    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        self.id = ""
        self.model = ""
        self.public = self.generate_pubkey(str(self.id))
        self.geo_lat = ""
        self.geo_lon = ""
        self.measurement = {}
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})
