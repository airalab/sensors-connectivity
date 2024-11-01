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
        self._paprse_data()
        if self.measurement:
            self.model = SDS011_MODEL
            self.public = self.generate_pubkey(str(self.id))
            self.geo_lat = 59.944502
            self.geo_lon = 30.295037
            self.timestamp = int(time.time())
            self.measurement.update({"timestamp": self.timestamp})
        else:
            raise LoraValidationException("Message doesn't contain required entities")

    def _paprse_data(self) -> None:
        mesaurement_entity = ["Temperature", "Humidity", "Pressure", "PM10", "PM2.5"]
        str_data = self.data["payload"]
        entity_name_pattern = r'(?P<entity_name>[A-Za-z0-9.]+)'
        entity_value_pattern = r'(?P<entity_value>[0-9.]+)'
        pattern = fr'\.{entity_name_pattern}:\s{entity_value_pattern}'
        if any(entity in str_data for entity in mesaurement_entity):
            match = re.search(pattern, str_data)
            entity_name = match.group("entity_name").replace('.', '').lower()
            entity_value = match.group("entity_value")
            self.measurement.update({entity_name: entity_value})

class LoraValidationException(Exception):
    pass