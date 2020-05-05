import json
import select
import socket
import threading
import time
import rospy
import netifaces
from datetime import timedelta
from sds011.sds011 import SDS011
from queue import LifoQueue

BROADCASTER_VERSION = "v0.1.0"


class Measurement:
    def __init__(self, pm25: float = 0, pm10: float = 0):
        self.pm25 = pm25
        self.pm10 = pm10

    def __str__(self):
        return f"{{PM2.5: {self.pm25}, PM10: {self.pm10}}}"


class StationData:
    def __init__(self, ver: str, mac: str, uptime: float, meas: Measurement):
        self.version = ver
        self.mac = mac
        self.uptime = uptime
        self.meas = meas

    def to_json(self) -> dict:
        ret = {
            "software_version": self.version,
            "sensordatavalues": [
                {"value_type": "P1", "value": self.meas.pm10},
                {"value_type": "P2", "value": self.meas.pm25}
            ]
        }

        rospy.logdebug(ret)
        return ret

    def __str__(self):
        uptime = str(timedelta(seconds=self.uptime))
        return f"{{MAC: {self.mac}, Uptime: {uptime}, M: {self.meas}}}"


def _get_mac() -> str:
    for interface in netifaces.interfaces():
        if interface != "lo":
            if 17 in netifaces.ifaddresses(interface):
                _i = netifaces.ifaddresses(interface)
                _i = _i[17][0]["addr"]
                break

    mac = _i.replace(":", "")
    return mac


class Station:
    def __init__(self, sds_sensor_port: str = "/dev/ttyUSB0", work_time: int = 300):
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"
        self.start_time = time.time()
        self.mac_address = _get_mac()

        self.sensor = SDS011(sds_sensor_port)
        self.sensor.set_work_period(work_time=int(work_time / 60))

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        meas = self.sensor.query()

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            Measurement(meas[0], meas[1])
        )


class ReadingThread(threading.Thread):
    MAX_CONNECTIONS = 10
    INPUTS = list()
    OUTPUTS = list()

    def __init__(self, address: str, q: LifoQueue):
        super().__init__()

        self.buffer = bytearray()
        self.q = q
        self.server_address = self._extract_ip_and_port(address)

    def _extract_ip_and_port(self, address: str) -> tuple:
        # assume the address looks like IP:PORT where IP is xx.xx.xx.xx
        splitted = address.split(":")
        return splitted[0], splitted[1]

    def _get_non_blocking_server_socket(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setblocking(False)
        server.bind(self.server_address)
        server.listen(self.MAX_CONNECTIONS)

        return server

    def handle_readables(self, readables, server):
        for resource in readables:
            if resource is server:
                connection, client_address = resource.accept()
                connection.setblocking(0)
                self.INPUTS.append(connection)
                rospy.loginfo("new connection from {address}".format(address=client_address))
            else:
                data = ""
                try:
                    data = resource.recv(1024)
                except ConnectionResetError:
                    pass

                if data:
                    rospy.loginfo("getting data: {data}".format(data=str(data)))
                    self.buffer.extend(data)

                    while len(self.buffer) > 1:
                        if b'\n' in self.buffer:
                            index = self.buffer.find(b'\n')
                            line = self.buffer[:index]
                            self.q.put(line.decode("utf-8", "backslashreplace"))
                            self.buffer = self.buffer[index + 1:]

                    if resource not in self.OUTPUTS:
                        self.OUTPUTS.append(resource)
                else:
                    self.clear_resource(resource)

    def clear_resource(self, resource):
        if resource in self.OUTPUTS:
            self.OUTPUTS.remove(resource)
        if resource in self.INPUTS:
            self.INPUTS.remove(resource)
        resource.close()

        rospy.loginfo('closing connection ' + str(resource))

    def run(self):
        server_socket = self._get_non_blocking_server_socket()
        self.INPUTS.append(server_socket)

        rospy.loginfo("Server is running...")

        try:
            while self.INPUTS:
                readables, writables, exceptional = select.select(self.INPUTS, self.OUTPUTS, self.INPUTS)
                self.handle_readables(readables, server_socket)
        except KeyboardInterrupt:
            self.clear_resource(server_socket)
            rospy.loginfo("Server stopped!")


class TCPStation:
    def __init__(self, address: str):
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"
        self.start_time = time.time()
        self.mac_address = _get_mac()

        self.q = LifoQueue()
        self.server = ReadingThread(address, self.q)
        self.server.start()

    def __str__(self):
        return f"{{Version: {self.version}, Start: {self.start_time}, MAC: {self.mac_address}}}"

    def get_data(self) -> StationData:
        if self.q.empty():
            meas = Measurement()
        else:
            values = json.loads(self.q.get(timeout=3))
            meas = Measurement(values["pm25"], values["pm10"])

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            meas
        )
