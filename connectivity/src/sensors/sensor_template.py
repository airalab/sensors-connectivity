from dataclasses import dataclass, field
import time

from .base import Device


@dataclass(repr=False, eq=False)
class SensorName(Device):
    """
    Template for new sensor. You need to initialise all variables from base class.
    Have a look at the example below.
    """

    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        self.id = ""
        self.model = ""
        self.public = self.generate_pubkey(str(self.id))
        self.geo_lat = ""
        self.geo_lon = ""
        self.measurement = {}
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})
        self.measurement.update({"model": self.model})
