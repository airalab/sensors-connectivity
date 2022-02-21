"""This module provides an abstraction for the SDS011 air partuclate densiry sensor.
The code is borrowed from here https://github.com/ikalchev/py-sds011
Thanks @ikalchev for work
"""
import struct
import typing as tp
import serial
from ..stations.istation import Measurement


SDS011_MODEL = 2  # unique model for the driver
MOBILE_GPS = 3


def sds011_codec(data: bytes, pk: str, timestamp: int) -> Measurement:
    unpacked = struct.unpack("<ffff", data)

    return Measurement(
        pk,
        SDS011_MODEL,
        round(unpacked[0], 2),
        round(unpacked[1], 2),
        round(unpacked[2], 6),
        round(unpacked[3], 6),
        timestamp,
    )


def sds011_gps_codec(data: bytes, pk: str, timestamp: int) -> Measurement:
    unpacked = struct.unpack("<ffff", data)

    return Measurement(
        pk,
        MOBILE_GPS,
        round(unpacked[0], 2),
        round(unpacked[1], 2),
        round(unpacked[2], 6),
        round(unpacked[3], 6),
        timestamp,
    )


# TODO: Commands against the sensor should read the reply and return success status.
class SDS011(object):
    """Provides method to read from a SDS011 air particlate density sensor
    using UART.
    """

    HEAD: bytes = b"\xaa"
    TAIL: bytes = b"\xab"
    CMD_ID: bytes = b"\xb4"

    # The sent command is a read or a write
    READ: bytes = b"\x00"
    WRITE: bytes = b"\x01"

    REPORT_MODE_CMD: bytes = b"\x02"
    ACTIVE: bytes = b"\x00"
    PASSIVE: bytes = b"\x01"

    QUERY_CMD: bytes = b"\x04"

    # The sleep command ID
    SLEEP_CMD: bytes = b"\x06"
    # Sleep and work byte
    SLEEP: bytes = b"\x00"
    WORK: bytes = b"\x01"

    # The work period command ID
    WORK_PERIOD_CMD: bytes = b"\x08"

    def __init__(
        self,
        serial_port: str,
        baudrate: int = 9600,
        timeout: int = 2,
        use_query_mode: bool = True,
    ) -> None:
        """Initialise and open serial port."""
        self.ser = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
        self.ser.flush()
        self.set_report_mode(active=not use_query_mode)

    def _execute(self, cmd_bytes) -> None:
        """Writes a byte sequence to the serial."""
        self.ser.write(cmd_bytes)

    def _get_reply(self) -> tp.Optional[tp.Any]:
        """Read reply from device."""
        raw = self.ser.read(size=10)
        data = raw[2:8]
        if len(data) == 0:
            return None
        if (sum(d for d in data) & 255) != raw[8]:
            return None  # TODO: also check cmd id
        return raw

    def cmd_begin(self) -> bytes:
        """Get command header and command ID bytes.
        @rtype: list
        """
        return self.HEAD + self.CMD_ID

    def set_report_mode(self, read=False, active=False) -> None:
        """Get sleep command. Does not contain checksum and tail.
        @rtype: list
        """
        cmd = self.cmd_begin()
        cmd += (
            self.REPORT_MODE_CMD
            + (self.READ if read else self.WRITE)
            + (self.ACTIVE if active else self.PASSIVE)
            + b"\x00" * 10
        )
        cmd = self._finish_cmd(cmd)
        self._execute(cmd)
        self._get_reply()

    def query(self) -> tp.Optional[tp.Tuple[float, float]]:
        """Query the device and read the data.
        @return: Air particulate density in micrograms per cubic meter.
        @rtype: tuple(float, float) -> (PM2.5, PM10)
        """
        cmd = self.cmd_begin()
        cmd += self.QUERY_CMD + b"\x00" * 12
        cmd = self._finish_cmd(cmd)
        self._execute(cmd)

        raw = self._get_reply()
        if raw is None:
            return None  # TODO:
        data = struct.unpack("<HH", raw[2:6])
        pm25 = data[0] / 10.0
        pm10 = data[1] / 10.0
        return (pm25, pm10)

    def sleep(self, read=False, sleep=True) -> None:
        """Sleep/Wake up the sensor.
        @param sleep: Whether the device should sleep or work.
        @type sleep: bool
        @param read
        @type read: bool
        """
        cmd = self.cmd_begin()
        cmd += (
            self.SLEEP_CMD
            + (self.READ if read else self.WRITE)
            + (self.SLEEP if sleep else self.WORK)
            + b"\x00" * 10
        )
        cmd = self._finish_cmd(cmd)
        self._execute(cmd)
        self._get_reply()

    def set_work_period(self, read=False, work_time=0) -> None:
        """Get work period command. Does not contain checksum and tail.
        @rtype: list
        """
        assert 0 <= work_time <= 30
        cmd = self.cmd_begin()
        cmd += (
            self.WORK_PERIOD_CMD
            + (self.READ if read else self.WRITE)
            + bytes([work_time])
            + b"\x00" * 10
        )
        cmd = self._finish_cmd(cmd)
        self._execute(cmd)
        self._get_reply()

    def _finish_cmd(self, cmd, id1=b"\xff", id2=b"\xff") -> bytes:
        """Add device ID, checksum and tail bytes.
        @rtype: list
        """
        cmd += id1 + id2
        checksum = sum(d for d in cmd[2:]) % 256
        cmd += bytes([checksum]) + self.TAIL
        return cmd

    def _process_frame(self, data) -> tp.Optional[tp.Tuple[float, float]]:
        """Process a SDS011 data frame.
        Byte positions:
            0 - Header
            1 - Command No.
            2,3 - PM2.5 low/high byte
            4,5 - PM10 low/high
            6,7 - ID bytes
            8 - Checksum - sum of bytes 2-7
            9 - Tail
        """
        raw = struct.unpack("<HHxxBBB", data[2:])
        checksum = sum(v for v in data[2:8]) % 256
        if checksum != data[8]:
            return None
        pm25 = raw[0] / 10.0
        pm10 = raw[1] / 10.0
        return pm25, pm10

    def read(self) -> tp.Tuple[float, float]:
        """Read sensor data.
        @return: PM2.5 and PM10 concetration in micrograms per cude meter.
        @rtype: tuple(float, float) - first is PM2.5.
        """
        byte = 0
        while byte != self.HEAD:
            byte = self.ser.read(size=1)
            d = self.ser.read(size=10)
            if d[0:1] == b"\xc0":
                data = self._process_frame(byte + d)
                return data
