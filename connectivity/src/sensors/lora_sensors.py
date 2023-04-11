import time
from dataclasses import dataclass, field

from ...constants import SDS011_MODEL
from .base import Device


@dataclass(repr=False, eq=False)
class LoraSensor(Device):
    data: dict = field(repr=False)
    id: str

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""
        self.model = SDS011_MODEL
        self.public = self.generate_pubkey(str(self.id))
        self.geo_lat = 59.944502
        self.geo_lon = 30.295037
        self.measurement = self.data
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})
