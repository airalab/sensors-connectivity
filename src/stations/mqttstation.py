import paho.mqtt.client as mqtt
from drivers.sds011 import SDS011_MODEL, MOBILE_GPS
from stations import *
import nacl.signing
import threading
import time
import rospy
import json
import copy
import hashlib


thlock = threading.RLock()
sessions = dict()


def _generate_pubkey(id) -> str:
    verify_key = hashlib.sha256(id.encode('utf-8'))
    verify_key_hex = verify_key.hexdigest()
    return str(verify_key_hex)


class MQTTHandler(mqtt.Client):
    def __init__(self, host: str, port: int):
        mqtt.Client.__init__(self)
        self.host = host
        self.port = port

    def on_connect(self, client, obj, flags, rc):
        rospy.loginfo(f"Connected to mqtt with result code {str(rc)}")
        self.subscribe("/freertos_mqtt_robonomics_example/#", 0)

    def _parser(self, data: dict) -> Measurement: 
        global sessions
        global thlock
        try:
            if "esp32mac" in data.keys():
                    self.client_id = data["esp32mac"]
                    temperature = None
                    pressure = None
                    humidity = None
                    for d in data["sensordatavalues"]:
                        if d["value_type"] == "SDS_P1":
                            pm10 = float(d["value"])
                        if d["value_type"] == "SDS_P2":
                            pm25 = float(d["value"])
                        if d["value_type"] == "GPS_lat":
                            geo_lat = d["value"]
                        if d["value_type"] == "GPS_lon":
                            geo_lon = d["value"]

                        if "temperature" in d["value_type"]:
                            temperature = float(d["value"])
                        if "pressure" in d["value_type"]:
                            pressure = float(d["value"])
                        if "humidity" in d["value_type"]:
                            humidity = float(d["value"])

                    meas = {}
                    meas.update({'pm10': pm10, 'pm25': pm25, 'temperature': temperature, 'pressure': pressure, 'humidity': humidity})

            elif 'ID' in data.keys():
                self.client_id = data["ID"]
                temperature = None
                pressure = None
                humidity = None
                CO = None
                NH3 = None
                NO2 = None
                speed = None
                vane = None

                if "temperature" in data.keys():
                    temperature = float(data["temperature"])
                if "humidity" in data.keys():
                    humidity = float(data["humidity"])
                if "pressure" in data.keys():
                    pressure = float(data["pressure"])
                if "CO" in data.keys():
                    CO = float(data["CO"])
                if "NH3" in data.keys():
                    NH3 = float(data["NH3"])
                if "NO2" in data.keys():
                    NO2 = float(data["NO2"])
                if "speed" in data.keys():
                    speed = float(data["speed"])
                if "vane" in data.keys():
                    vane = data["vane"]

                geo_lat = float(data["GPS_lat"])
                geo_lon = float(data["GPS_lon"])

                meas = {}
                meas.update({"temperature": temperature, "humidity": humidity, "pressure": pressure, "CO": CO, "NH3": NH3, "NO2": NO2, "speed": speed, "vane": vane})

            timestamp = int(time.time())
            meas.update({'timestamp': timestamp})
            with thlock:
                if self.client_id not in sessions:
                    public = _generate_pubkey()
                else:
                    public = sessions[self.client_id].public
            
            measurement = Measurement(public,
                                     SDS011_MODEL,
                                     geo_lat,
                                     geo_lon,
                                     meas)

            rospy.loginfo(f"measurment: {measurement}")
        except Exception as e:
            rospy.logerr(e)
            return
            
        return measurement

    def on_message(self, client, userdata, msg: dict):
        global thlock
        global sessions
        #rospy.loginfo(f'on msg {msg.payload}')
        data = json.loads(msg.payload.decode())
        parse_data = self._parser(data)
        print(f"parse {parse_data}")

        with thlock:
            if parse_data:
                sessions[self.client_id] = parse_data

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print(f"Subscribed {str(mid)}, client {client}")
        rospy.loginfo(f"Subscribed {str(mid)}, client {client}")

    def run(self):
        self.connect_async(self.host, self.port, 60)
        #self.subscribe("$SYS/#")
        self.loop_start()
        
    
class MQTTStation(IStation):
    def __init__(self, config: dict):
        super().__init__(config)
        self.version = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME = 60*60 # 1 hour
        host = config["mqttstation"]["host"]
        port = int(config["mqttstation"]["port"])

        client = MQTTHandler(host, port)
        rc = client.run()
    
    
    def get_data(self) -> StationData:
        global sessions
        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(StationData(
                self.version,
                self.mac_address,
                time.time() - self.start_time,
                v
            ))
        print(f'result {result}')
        rospy.loginfo(f'result {result}')
        return result


    def _drop_dead_sensors(self) -> dict:
        global thlock
        global sessions
        stripped = dict()
        current_time = int(time.time())
        with thlock:
            sessions_copy = copy.deepcopy(sessions)
            for k, v in sessions_copy.items():
                if (current_time - v.measurement["timestamp"]) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v
                else:
                    del sessions[k]

        return stripped

