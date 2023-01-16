import struct
from ..sensors import SensorSDS011
from ...constants import PING_MODEL 


def ping_codec(data: bytes, pk: str) -> str:
    unpacked = struct.unpack("<ff", data)
    meas = SensorSDS011(public_key=pk, model=PING_MODEL, data=unpacked)
    return meas
