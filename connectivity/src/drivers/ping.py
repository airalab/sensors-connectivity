import struct
from ..stations.istation import Measurement

PING_MODEL = 1  # unique model for the driver


def ping_codec(data: bytes, pk: str, timestamp: int) -> Measurement:
    unpacked = struct.unpack("<ff", data)

    return Measurement(
        pk, PING_MODEL, 0, 0, round(unpacked[0], 6), round(unpacked[1], 6), timestamp
    )
