import json

from connectivity.src.sensors.sensors_types import Device

def to_pubsub_message(data: Device) -> str:
    """Prepare JSON formatted string with measurements.

    :param data: Dict with the last measurement from one sensor.
    :return: JSON formatted string for pubsub.
    """

    message = {}
    message[data.public] = {
        "model": data.model,
        "geo": "{},{}".format(data.geo_lat, data.geo_lon),
        "donated_by": data.donated_by,
        "measurement": data.measurement,
    }
    return json.dumps(message)


def to_ping_message(data: Device) -> str:
    """Prepare JSON formatted string with base info about sensor.
    No measurements.

    :param data: Dict with the base info from one sensor.
    :return: JSON formatted string for pubsub.
    """

    message = {}
    message[data.public] = {
        "model": data.model,
        "timestamp": data.measurement.timestamp,
        "measurement": {"geo": "{},{}".format(data.geo_lat, data.geo_lon)},
    }

    return json.dumps(message)