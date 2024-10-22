import logging.config
import typing as tp
from tempfile import NamedTemporaryFile
import json

from connectivity.config.logging import LOGGING_CONFIG
from connectivity.constants import MOBILE_GPS

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("sensors-connectivity")


def create_payload(buf: set) -> dict:
    """Format measurements to payload.

    :param buf: Set of measurements from all sensors.
    :return: Payload
    """
    payload = {}
    for m in buf:
        try:
            if m.public in payload:
                payload[m.public]["measurements"].append(m.measurement)
            else:
                if m.model == MOBILE_GPS:
                    payload[m.public] = {
                        "model": m.model,
                        "donated_by": m.donated_by,
                        "measurements": [m.measurement],
                    }
                else:
                    payload[m.public] = {
                        "model": m.model,
                        "geo": "{},{}".format(m.geo_lat, m.geo_lon),
                        "donated_by": m.donated_by,
                        "measurements": [m.measurement],
                    }
        except Exception as e:
            logger.warning(f"Create datalog payload: Couldn't store data: {e}")

    return payload


def sort_payload(payload: dict) -> dict:
    """Sort measurements dict with timestamp.

    :param payload: Measurements dict.
    :return: Sorted measurements dict.
    """

    ordered = {}
    for k, v in payload.items():
        try:
            meas = sorted(v["measurements"], key=lambda x: x["timestamp"])
            if v.get("geo"):
                ordered[k] = {
                    "model": v["model"],
                    "geo": v["geo"],
                    "donated_by": v["donated_by"],
                    "measurements": meas,
                }
            else:
                ordered[k] = {
                    "model": v["model"],
                    "donated_by": v["donated_by"],
                    "measurements": meas,
                }
        except Exception as e:
            logger.warning(f"Sort datalog payload: Couldn't sort data: {e}")
    return ordered


def create_tmp_file(payload: dict) -> str:
    """Save payload to a named tmp file.

    :param payload: Payload which will be stored.

    :return: Path to the tmp file
    """
    temp = NamedTemporaryFile(mode="w", delete=False)
    logger.debug(f"Created temp file: {temp.name}")
    temp.write(json.dumps(payload))
    temp.close()

    return temp.name
