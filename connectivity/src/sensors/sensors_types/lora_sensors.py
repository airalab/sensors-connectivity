import time
import re
from dataclasses import dataclass, field

from connectivity.constants import SDS011_MODEL
from .base import Device


@dataclass(repr=False, eq=False)
class LoraSensor(Device):
    data: dict = field(repr=False)
    id: str

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""
        
        super().__post_init__()
        self.model = SDS011_MODEL
        self.public = self.generate_pubkey(str(self.id))
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})
        self._paprse_data()

    def _paprse_data(self) -> None:
        for sensor_name, value in self.data["decoded_payload"].items():
            entity_name = sensor_name.split("_")[1]
            self.measurement.update({entity_name: str(round(value, 2))})
        self.geo_lat = self.data["locations"]["user"]["latitude"]
        self.geo_lon = self.data["locations"]["user"]["longitude"]
