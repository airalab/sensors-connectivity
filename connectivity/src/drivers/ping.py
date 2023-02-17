import struct

from ...constants import PING_MODEL
from ..sensors import SensorSDS011


def ping_codec(data: bytes, pk: str) -> str:
    """Creates a measurement message for SDS011 sensor
    connected with USB.

    :param data: Data from sensor.
    :param: pk: Public key of the sensor.
    :return: Formatted string with measurement.
    """

    unpacked = struct.unpack("<ff", data)
    meas = SensorSDS011(public_key=pk, model=PING_MODEL, data=unpacked)
    return meas
