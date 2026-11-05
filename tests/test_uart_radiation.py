import base64
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from substrateinterface import Keypair, KeypairType

from connectivity.constants import PASKAL2MMHG
from connectivity.src.sensors import SensorsFabcric
from connectivity.src.sensors.sensors_types.altruist import Altruist
from connectivity.utils.datalog_payload import create_payload, create_tmp_file, sort_payload
from connectivity.utils.format_robonomics_feeder_msg import to_pubsub_message


class UartRadiationTests(unittest.TestCase):
    timestamp = 1800000000
    radiation = {
        "dose_rate_nsv_h": 86.8,
        "count_rate_cps": 7.57,
        "dose_rate_error_pct": 6.5,
        "count_rate_error_pct": 3.5,
    }

    def make_sensor(self, sensor_data):
        keypair = Keypair.create_from_seed("0x" + "01" * 32, crypto_type=KeypairType.ED25519)
        signed_data = f"{sensor_data},time:{str(self.timestamp)[:-2]}"
        data = {
            "robonomics_address": keypair.ss58_address,
            "owner": keypair.ss58_address,
            "GPS_lat": 34.7,
            "GPS_lon": 33.0,
            "sensordatavalues": sensor_data,
            "signature": keypair.sign(signed_data).hex(),
        }
        # Keep signature verification real; subscription membership requires external RPC.
        with patch.object(Altruist, "_is_address_in_subscription", return_value=True), patch(
            "connectivity.src.sensors.sensors_types.altruist.time.time", return_value=self.timestamp
        ):
            return SensorsFabcric.get_sensor(data)

    def test_plaintext_and_existing_measurements(self):
        sensor = self.make_sensor(
            "dose_rate_nsv_h:86.80,count_rate_cps:7.57,"
            "dose_rate_error_pct:6.50,count_rate_error_pct:3.50,"
            "t:21.5,h:45,p:100000,gc:12.3"
        )
        self.assertEqual(sensor.measurement, {
            **self.radiation,
            "temperature": 21.5,
            "humidity": 45.0,
            "pressure": round(100000 / PASKAL2MMHG, 2),
            "radiation": 12.3,
            "timestamp": self.timestamp,
        })

    def test_zero_values(self):
        sensor = self.make_sensor(",".join(f"{key}:0.00" for key in self.radiation))
        self.assertEqual(sensor.measurement, {
            **dict.fromkeys(self.radiation, 0.0), "timestamp": self.timestamp,
        })

    def test_missing_fields_are_not_filled(self):
        for payload, expected in (
            ("t:21.5,gc:12.3", {"temperature": 21.5, "radiation": 12.3}),
            ("dose_rate_nsv_h:86.80", {"dose_rate_nsv_h": 86.8}),
        ):
            with self.subTest(payload=payload):
                self.assertEqual(self.make_sensor(payload).measurement, {
                    **expected, "timestamp": self.timestamp,
                })

    def test_fields_survive_pubsub_and_datalog_serialization(self):
        encrypted = {
            key: "e." + base64.b64encode(f"opaque-{index}".encode()).decode()
            for index, key in enumerate(self.radiation)
        }
        mixed = {**self.radiation, "count_rate_cps": encrypted["count_rate_cps"]}
        for values in (self.radiation, encrypted, mixed):
            with self.subTest(values=values):
                sensor = self.make_sensor(",".join(f"{key}:{value}" for key, value in values.items()))
                expected = {**values, "timestamp": self.timestamp}
                self.assertEqual(sensor.measurement, expected)
                realtime = json.loads(to_pubsub_message(sensor, "test-signature"))
                self.assertEqual(realtime[sensor.public]["measurement"], expected)
                payload = sort_payload(create_payload({sensor}))
                with TemporaryDirectory() as directory, patch(
                    "tempfile.tempdir", directory
                ):
                    path = create_tmp_file(payload)
                    stored = json.loads(Path(path).read_text())
                self.assertEqual(stored[sensor.public]["measurements"], [expected])
                self.assertEqual(stored[sensor.public]["geo"], "34.7,33.0")


if __name__ == "__main__":
    unittest.main()
