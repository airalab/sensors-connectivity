import paho.mqtt.client as mqtt
import random
import json

broker = "localhost"
port = 1883
topic = ""
username = ""
password = ""


def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code: " + str(rc))


def on_publish(client, userdata, mid):
    print("Message published successfully")


def main():
    id = round(random.randrange(0, 100000))
    lat = round(random.uniform(29.972, 17.051), 6)
    lon = round(random.uniform(-109.202, -94.823), 6)
    temperature = round(random.uniform(-50, 50), 3)
    altitude = round(random.uniform(0, 1000), 3)
    pressure = round(random.randrange(0, 100), 2)
    humidity = round(random.uniform(0, 100))
    pm10 = round(random.uniform(0.1, 100.99), 3)
    pm25 = round(random.uniform(0.1, 100.99), 3)

    msg = {
        "end_device_ids": {
            "device_id": "eui-a7c405d10ed8b370",
        },
        "received_at": "2023-07-03T14:12:12.317322686Z",
        "uplink_message": {
            "f_port": 10,
            "f_cnt": 82,
            "decoded_payload": {
                "altitude": altitude,
                "humidity": humidity,
                "pm10": pm10,
                "pm2.5": pm25,
                "pressure": pressure,
                "temperature": temperature,
            },
        },
    }

    client = mqtt.Client()
    client.username_pw_set(username, password)

    client.on_connect = on_connect
    client.on_publish = on_publish

    client.connect(broker, port)

    client.loop_start()
    client.publish(topic, json.dumps(msg))
    client.loop_stop()
