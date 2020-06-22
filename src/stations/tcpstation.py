import select
import threading
import socket
import struct
import signal
import time
import nacl.signing

import rospy
from collections import deque
from stations import IStation, StationData, Measurement, STATION_VERSION
from drivers.sds011 import SDS011_MODEL, MOBILE_GPS, sds011_codec
from drivers.ping import PING_MODEL, ping_codec


def _extract_ip_and_port(address: str) -> tuple:
    # assume the address looks like IP:PORT where IP is xx.xx.xx.xx
    splitted = address.split(":")
    return splitted[0], int(splitted[1])

def _get_non_blocking_server_socket(addr: tuple, max_connections: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    server.setblocking(False)
    server.bind(addr)
    server.listen(max_connections)

    return server

def parse_header(data: bytes) -> tuple:
    public_key = data[:32].hex()
    model = struct.unpack("<h", data[32:34])
    return public_key, model[0]


def _get_codec(model: int) -> int:
    models = {
        PING_MODEL: (8 + 64, ping_codec),
        SDS011_MODEL: (16 + 64, sds011_codec)
        MOBILE_GPS: (16 + 64, sds011_codec)
    }

    return models[model]


class ReadingThread(threading.Thread):
    MAX_CONNECTIONS = 10
    INPUTS = list()
    OUTPUTS = list()

    """
    SESSIONS[peer] = {
        "public": public_key,
        "model": model,
        "buffer": bytearray()
    }
    """
    SESSIONS = dict()

    def __init__(self, config: dict, q: deque):
        super().__init__()

        self.q = q
        self.running = True
        self.server_address = _extract_ip_and_port(config["address"])
        self.acl = config["acl"]
        rospy.loginfo(self.acl)

    def cancel(self):
        self.running = False

    def parse_frame(self, peer) -> tuple:
        try:
            data_length, parser = _get_codec(self.SESSIONS[peer]["model"])
        except Exception as e:
            rospy.logerr(e)
            return False, Measurement()

        if data_length > len(self.SESSIONS[peer]["buffer"]):
            return False, Measurement()

        signed = bytes(self.SESSIONS[peer]["buffer"][:data_length])
        self.SESSIONS[peer]["buffer"] = self.SESSIONS[peer]["buffer"][data_length:]
        pk = self.SESSIONS[peer]["public"]

        public_key = nacl.signing.VerifyKey(pk, encoder=nacl.encoding.HexEncoder)

        try:
            public_key.verify(signed[:-64], signed[-64:])
            m = parser(signed[:-64], pk, int(time.time()))
            return (True, m)
        except nacl.exceptions.BadSignatureError as e:
            rospy.logerr(e)
            return (False, Measurement())

    def handle_readables(self, readables, server):
        for resource in readables:
            if resource is server:
                connection, client_address = resource.accept()
                connection.setblocking(0)
                self.INPUTS.append(connection)
                rospy.loginfo("new connection from {address}".format(address=client_address))
            else:
                data = ""

                rospy.loginfo("Peer name: " + str(resource.getpeername()))

                peer = resource.getpeername()
                if peer not in self.SESSIONS:
                    rospy.loginfo("Unknown peer yet")
                    data = resource.recv(34)

                    public_key, model = parse_header(data)

                    if (not self.acl) or (public_key in self.acl):
                        self.SESSIONS[peer] = {
                            "public": public_key,
                            "model": model,
                            "buffer": bytearray()
                        }
                        rospy.loginfo(f"Welcome to the party: ({public_key},{model})")
                    else:
                        self.clear_resource(resource)
                else:
                    rospy.loginfo("I know you buddy!")
                    data = resource.recv(128)
                    self.SESSIONS[peer]["buffer"].extend(data)

                    if data:
                        status, measurement = self.parse_frame(peer)

                        if status:
                            rospy.loginfo(measurement)
                            self.q[0][measurement.public] = measurement
                            # self.q.append(measurement)

                        if resource not in self.OUTPUTS:
                            self.OUTPUTS.append(resource)
                    else:
                        self.clear_resource(resource)

    def clear_resource(self, resource):
        if resource in self.OUTPUTS:
            self.OUTPUTS.remove(resource)
        if resource in self.INPUTS:
            self.INPUTS.remove(resource)
        try:
            if resource.getpeername() in self.SESSIONS:
                del self.q[0][self.SESSIONS[resource.getpeername()]["public"]]
                del self.SESSIONS[resource.getpeername()]
        except:
            rospy.loginfo("Error cleaning connections")

        resource.close()

        rospy.loginfo("closing connection " + str(resource))

    def run(self):
        server_socket = _get_non_blocking_server_socket(self.server_address, self.MAX_CONNECTIONS)
        self.INPUTS.append(server_socket)

        rospy.loginfo("Server is running...")

        while self.INPUTS and self.running:
            readables, writables, exceptional = select.select(self.INPUTS, self.OUTPUTS, self.INPUTS)
            self.handle_readables(readables, server_socket)
            time.sleep(2)   # had to limit cpu usage

        self.clear_resource(server_socket)
        rospy.loginfo("Server stopped!")


class TCPStation(IStation):
    """
    Reads data from a TCP port

    """

    TIMEOUT = 1 * 60 * 60   # Timeout to drop dead sensors in seconds

    def __init__(self, config: dict):
        super().__init__(config)
        self.version = f"airalab-tcp-{STATION_VERSION}"

        """
        q[0] = {
          "public": Measurement,
          ...
          "public2": Measurement
        }
        """
        self.q = deque(maxlen=1)
        self.q.append({})   # Need to find a better way of interthread data exchange
        self.server = ReadingThread(self.config["tcpstation"], self.q)
        self.server.start()

        signal.signal(signal.SIGINT, self.handle_sigint)

    def handle_sigint(self, signal, frame):
        self.server.cancel()

    def get_data(self) -> StationData:
        self._drop_dead_sensors()

        result = []
        for k, v in self.q[0].items():
            result.append(StationData(
                self.version,
                self.mac_address,
                time.time() - self.start_time,
                self.q[0][k]
            ))

        return result

    def _drop_dead_sensors(self):
        stripped = {}
        current_time = int(time.time())
        for k, v in self.q[0].items():
            if (current_time - self.q[0][k].timestamp) < self.TIMEOUT:
                stripped[k] = v

        self.q[0] = stripped

