import asyncio
from asyncio import StreamReader, StreamWriter
import threading
import struct
import time
import nacl.signing

import rospy
from stations import IStation, StationData, Measurement, STATION_VERSION
from drivers.sds011 import SDS011_MODEL, MOBILE_GPS, sds011_codec, sds011_gps_codec
from drivers.ping import PING_MODEL, ping_codec


def _extract_ip_and_port(address: str) -> tuple:
    # assume the address looks like IP:PORT where IP is xx.xx.xx.xx
    splitted = address.split(":")
    return splitted[0], int(splitted[1])

def _parse_header(data: bytes) -> tuple:
    public_key = data[:32].hex()
    model = struct.unpack("<h", data[32:34])
    return public_key, model[0]

def _get_codec(model: int) -> int:
    models = {
        PING_MODEL: (8 + 64, ping_codec),
        SDS011_MODEL: (16 + 64, sds011_codec),
        MOBILE_GPS: (16 + 64, sds011_gps_codec)
    }

    return models[model]


class TCPStation(IStation):
    """
    Reads data from a TCP port

    """

    TIMEOUT = 1 * 60 * 60   # Timeout to drop dead sensors in seconds

    """
    {
      task: (client_reader, client_writer),
      ...
    }
    """
    clients = {}

    """
    sessions[peer] = {
        "public": public_key,
        "model": model,
        "buffer": bytearray(),
        "measurement": Measurement()
    }
    """
    sessions = {}

    def __init__(self, config: dict):
        super().__init__(config)
        rospy.loginfo("Starting tcp station...")
        self.version = f"airalab-tcp-{STATION_VERSION}"

        server_address = _extract_ip_and_port(config["tcpstation"]["address"])

        # Starting a dedicated thread to serve TCP server
        threading.Thread(target=self._start_server_thread, args=(server_address,)).start()
        rospy.loginfo("TCP Station is launched!")

    # Placing asyncio stuff to a dedicated thread
    def _start_server_thread(self, server_address: tuple):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        f = asyncio.start_server(self._accept_client, host=server_address[0], port=server_address[1])
        loop.run_until_complete(f)
        loop.run_forever()

    def _accept_client(self, client_reader: StreamReader, client_writer: StreamWriter):
        # creating a new task for every connection
        task = asyncio.Task(self._handle_client(client_reader, client_writer))
        self.clients[task] = (client_reader, client_writer)

        def client_done(task):
            try:
                peer = self.clients[task][1].get_extra_info('peername')
                del self.sessions[peer]
                del self.clients[task]
                client_writer.close()
                rospy.loginfo("End Connection")
            except Exception as e:
                rospy.logerr(e)

        task.add_done_callback(client_done)

    async def _handle_client(self, client_reader: StreamReader, client_writer: StreamWriter):
        peer = client_writer.get_extra_info('peername')
        rospy.loginfo(f"Client at {peer}")

        data = await asyncio.wait_for(client_reader.readexactly(34), timeout=30)
        public_key, model = _parse_header(data)

        self.sessions[peer] = {
            "public": public_key,
            "model": model,
            "buffer": bytearray(),
            "measurement": Measurement()
        }

        rospy.loginfo(f"Welcome to the party: ({public_key},{model})")

        try:
            while True:
                try:
                    # Timeout is kinda big but it covers the maximum timeout for SDS011 sensor
                    # and gets rid of dropped connections
                    data = await asyncio.wait_for(client_reader.read(128), timeout=2000.0)
                    self.sessions[peer]["buffer"].extend(data)
                except:
                    rospy.logwarn("Timeout")
                    return

                if data:
                    status, measurement = self._parse_frame(peer)

                    if status:
                        self.sessions[peer]["measurement"] = measurement
                        rospy.logdebug(measurement)
                else:
                    rospy.logwarn("Received no data")
                    # exit echo loop and disconnect
                    return
        except Exception as e:
            rospy.logwarn(e)

    def _parse_frame(self, peer: tuple) -> tuple:
        try:
            data_length, parser = _get_codec(self.sessions[peer]["model"])
        except Exception as e:
            rospy.logerr(e)
            return False, Measurement()

        if data_length > len(self.sessions[peer]["buffer"]):
            return False, Measurement()

        signed = bytes(self.sessions[peer]["buffer"][:data_length])
        self.sessions[peer]["buffer"] = self.sessions[peer]["buffer"][data_length:]
        pk = self.sessions[peer]["public"]

        public_key = nacl.signing.VerifyKey(pk, encoder=nacl.encoding.HexEncoder)

        try:
            # checking signature
            public_key.verify(signed[:-64], signed[-64:])
            m = parser(signed[:-64], pk, int(time.time()))
            return (True, m)
        except nacl.exceptions.BadSignatureError as e:
            rospy.logerr(e)
            return (False, Measurement())

    def get_data(self) -> StationData:
        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(StationData(
                self.version,
                self.mac_address,
                time.time() - self.start_time,
                v
            ))

        return result

    def _drop_dead_sensors(self) -> dict:
        stripped = {}
        current_time = int(time.time())
        for k, v in self.sessions.items():
            if (current_time - v["measurement"].timestamp) < self.TIMEOUT:
                stripped[v["public"]] = v["measurement"]

        return stripped

