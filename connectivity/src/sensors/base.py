import hashlib
import time
from dataclasses import dataclass, field


@dataclass()
class Device:
    """Base interface for a sensor. Initialize all variables and formate message with measurements."""

    id: str = field(init=False)
    public: str = field(init=False)
    geo_lat: float = field(init=False)
    geo_lon: float = field(init=False)
    model: int = field(init=False)
    timestamp: float = field(init=False)
    measurement: dict = field(init=False)

    def __post_init__(self) -> None:
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})

    def generate_pubkey(self, id: str) -> str:
        """Generate public key for a sensor based on its id.
        
        :param id: Unique id of a sensor.
        :return: Public key of a sensor.
        """

        verify_key = hashlib.sha256(id.encode("utf-8"))
        verify_key_hex = verify_key.hexdigest()
        return str(verify_key_hex)

    def __str__(self) -> str:
        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), model: {self.model}, measurements: {self.measurement}}}"

    def __repr__(self) -> str:
        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), model: {self.model}, measurements: {self.measurement}}}"

    def __hash__(self) -> int:
        return hash((self.id, self.timestamp))
