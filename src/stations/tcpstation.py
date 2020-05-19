import json
import select
import threading
import socket
import time
import nacl.signing

import rospy
from collections import deque
from stations import IStation, StationData, Measurement
from .comstation import BROADCASTER_VERSION
from drivers.payload_pb2 import Header, Body


class ReadingThread(threading.Thread):
    MAX_CONNECTIONS = 10
    INPUTS = list()
    OUTPUTS = list()
    session = dict()

    def __init__(self, config: dict, q: deque):
        super().__init__()

        self.buffer = bytearray()
        self.q = q
        self.server_address = self._extract_ip_and_port(config["address"])
        self.acl = config["acl"]
        rospy.loginfo(self.acl)

    def _extract_ip_and_port(self, address: str) -> tuple:
        # assume the address looks like IP:PORT where IP is xx.xx.xx.xx
        splitted = address.split(":")
        return splitted[0], int(splitted[1])

    def _get_non_blocking_server_socket(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setblocking(False)
        server.bind(self.server_address)
        server.listen(self.MAX_CONNECTIONS)

        return server

    def check_acl(self, data: Header) -> (bytes, int):
        public = data.public_key
        codec = data.model

        pk = public.hex()

        print(f"Public: {pk}, codec: {codec}")
        if pk in self.acl:
            return public, codec

        return "", 0

    def parse_data(self, data: bytes, signature: bytes, pk: bytes) -> str:
        print("Parsing data...")
        print(f"Data: {data}")
        print(f"Signature: {signature}")
        print(f"Public key: {pk}")


        public_key = nacl.signing.VerifyKey(pk)

        try:
            public_key.verify(data, signature)
            return "Cool"
        except:
            return "Broken"

    def frame_length(self, peer: tuple) -> int:
        if self.session[peer][1] == 2:
            return 32

        return 0

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
                if peer not in self.session:
                    print("Unknown peer yet")
                    data = resource.recv(36)

                    header = Header()
                    header.ParseFromString(data)

                    known, codec = self.check_acl(header)

                    if known:
                        self.session[peer] = (known, codec)
                        print("Welcome to the party: " + str(self.session))
                else:
                    print("I know you buddy!")
                    msg_len = self.frame_length(peer)
                    data = resource.recv(msg_len)

                    if data:

                        print("getting data: {data}".format(data=str(data)))
                        body = Body()
                        body.ParseFromString(data)

                        public_id = self.session[peer][0]
                        signature = resource.recv(64)

                        response = self.parse_data(data, signature, public_id)
                        print(f"Response: {response}")

                        if response == "Cool":
                            m = Measurement(
                                public_id.hex(),
                                self.session[peer][1],
                                body.pm25,
                                body.pm10,
                                body.geo,
                                int(time.time())
                            )
                            rospy.loginfo(m)
                            self.q.append(m)
                        else:
                            print("Something bad happend")

                        if resource not in self.OUTPUTS:
                            self.OUTPUTS.append(resource)
                    else:
                        self.clear_resource(resource)

    def clear_resource(self, resource):
        if resource in self.OUTPUTS:
            self.OUTPUTS.remove(resource)
        if resource in self.INPUTS:
            self.INPUTS.remove(resource)
        if resource.getpeername() in self.session:
            del self.session[resource.getpeername()]

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

        self.q = deque(maxlen=1)  # LifoQueue(maxsize=1)
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

