import json
import select
import threading
import socket
import struct
import time
import nacl.signing

import rospy
from collections import deque
from stations import IStation, StationData, Measurement
from .comstation import BROADCASTER_VERSION
from drivers.sds011 import SDS011_MODEL
#from drivers.payload_pb2 import Header, Body


def _extract_ip_and_port(address: str) -> tuple:
    # assume the address looks like IP:PORT where IP is xx.xx.xx.xx
    splitted = address.split(":")
    return splitted[0], int(splitted[1])

def _get_non_blocking_server_socket(addr: tuple, max_connections: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setblocking(False)
    server.bind(addr)
    server.listen(max_connections)

    return server

def parse_header(data: bytes) -> tuple:
    public_key = data[:32].hex()
    model = struct.unpack("<h", data[32:34])
    return public_key, model[0]

def parse_sds011(data: bytes, pk: str, model: int, timestamp: int) -> Measurement:
    unpacked = struct.unpack("<ffff", data)

    return Measurement(pk,
                       model,
                       round(unpacked[0], 2),
                       round(unpacked[1], 2),
                       round(unpacked[2], 6),
                       round(unpacked[3], 6),
                       timestamp)

def _get_codec(model: int) -> int:
    models = {
        SDS011_MODEL: (16 + 64, parse_sds011)
    }

    return models[model]


class ReadingThread(threading.Thread):
    MAX_CONNECTIONS = 10
    INPUTS = list()
    OUTPUTS = list()
    SESSIONS = dict()

    def __init__(self, config: dict, q: deque):
        super().__init__()

        self.q = q
        self.server_address = _extract_ip_and_port(config["address"])
        self.acl = config["acl"]
        rospy.loginfo(self.acl)

    def parse_frame(self, peer) -> tuple:
        try:
            data_length, parser = _get_codec(self.SESSIONS[peer]["model"])
        except:
            return False, Measurement()

        if data_length > len(self.SESSIONS[peer]["buffer"]):
            return False, Measurement()

        signed = bytes(self.SESSIONS[peer]["buffer"][:data_length])
        self.SESSIONS[peer]["buffer"] = self.SESSIONS[peer]["buffer"][data_length:]
        pk = self.SESSIONS[peer]["public"]

        public_key = nacl.signing.VerifyKey(pk, encoder=nacl.encoding.HexEncoder)

        try:
            public_key.verify(signed[:-64], signed[-64:])
            m = parser(signed[:-64], pk, self.SESSIONS[peer]["model"], int(time.time()))
            return (True, m)
        except nacl.exceptions.BadSignatureError:
            return (False, Measurement())

    def handle_readables(self, readables, server):
        for resource in readables:
            if resource is server:
                connection, client_address = resource.accept()
                connection.setblocking(0)
                self.INPUTS.append(connection)
                print("new connection from {address}".format(address=client_address))
            else:
                data = ""

                print("Peer name: " + str(resource.getpeername()))

                peer = resource.getpeername()
                if peer not in self.SESSIONS:
                    print("Unknown peer yet")
                    data = resource.recv(34)

                    public_key, model = parse_header(data)

                    if public_key in self.acl:
                        self.SESSIONS[peer] = {
                            "public": public_key,
                            "model": model,
                            "buffer": bytearray()
                        }
                        print(f"Welcome to the party: ({public_key},{model})")
                    else:
                        self.clear_resource(resource)
                else:
                    print("I know you buddy!")
                    data = resource.recv(128)
                    self.SESSIONS[peer]["buffer"].extend(data)

                    if data:
                        status, measurement = self.parse_frame(peer)

                        if status:
                            rospy.loginfo(measurement)
                            self.q.append(measurement)

                        if resource not in self.OUTPUTS:
                            self.OUTPUTS.append(resource)
                    else:
                        self.clear_resource(resource)

    def clear_resource(self, resource):
        if resource in self.OUTPUTS:
            self.OUTPUTS.remove(resource)
        if resource in self.INPUTS:
            self.INPUTS.remove(resource)
        if resource.getpeername() in self.SESSIONS:
            del self.SESSIONS[resource.getpeername()]

        resource.close()

        rospy.loginfo("closing connection " + str(resource))

    def run(self):
        server_socket = _get_non_blocking_server_socket(self.server_address, self.MAX_CONNECTIONS)
        self.INPUTS.append(server_socket)

        rospy.loginfo("Server is running...")

        try:
            while self.INPUTS:
                readables, writables, exceptional = select.select(self.INPUTS, self.OUTPUTS, self.INPUTS)
                self.handle_readables(readables, server_socket)
                time.sleep(2)   # had to limit cpu usage
        except KeyboardInterrupt:
            self.clear_resource(server_socket)
            rospy.loginfo("Server stopped!")


class TCPStation(IStation):
    """
    Reads data from a TCP port

    """

    def __init__(self, config: dict):
        super().__init__(config)
        self.version = f"airalab-rpi-broadcaster-{BROADCASTER_VERSION}"

        self.q = deque(maxlen=1)
        self.server = ReadingThread(self.config["tcpstation"], self.q)
        self.server.start()

    def get_data(self) -> StationData:
        meas = self.q[-1] if self.q else Measurement()

        return StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            meas
        )

