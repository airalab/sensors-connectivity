from abc import ABC, abstractmethod
from dataclasses import dataclass
import typing as tp


@dataclass
class PinArgs:
    file_path: str
    hash: tp.Optional[str] = None
    file_size: tp.Optional[int] = None


class PinningGateway(ABC):
    """Base class for custom pinning services."""

    @abstractmethod
    def pin(self, args: PinArgs) -> tp.Optional[tp.Tuple[str, int]]:
        raise NotImplementedError("Gateway must implement pin method")
