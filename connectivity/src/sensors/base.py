from dataclasses import dataclass, field
import hashlib


@dataclass
class Device:
    id: int = field(init=False)
    public: str = field(init=False)
    geo_lat: float = field(init=False)
    geo_lon: float = field(init=False)
    model: int = field(init=False)
    measurement: dict = field(init=False)

    def __post_init__(self) -> None:
        pass

    def generate_pubkey(self, id: str) -> str:
        verify_key = hashlib.sha256(id.encode("utf-8"))
        verify_key_hex = verify_key.hexdigest()
        return str(verify_key_hex)

    def __str__(self) -> str:
        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), model: {self.model}, measurements: {self.measurement}}}"

    def __repr__(self) -> str:
        return f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), model: {self.model}, measurements: {self.measurement}}}"
