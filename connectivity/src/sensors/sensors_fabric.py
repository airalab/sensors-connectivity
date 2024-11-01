from .sensors_types import EnvironmentalBox, LoraSensor, MobileLab, Device

class SensorsFabcric():

    @staticmethod
    def get_sensor(data: dict) -> Device:
        meas = None
        if "esp8266id" in data.keys():
            meas = EnvironmentalBox(data)
        elif "ID" in data.keys():
            meas = MobileLab(data)
        elif "uplink_message" in data.keys():
            if "decoded_payload" in data["uplink_message"]:
                id = data["end_device_ids"]["device_id"]
                meas = LoraSensor(id=id, data=data["uplink_message"]["decoded_payload"])
        return meas