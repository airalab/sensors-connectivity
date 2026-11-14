import threading
import time
from dataclasses import dataclass, field
from substrateinterface import Keypair, KeypairType
from robonomicsinterface import RWS, Account

from connectivity.constants import SDS011_MODEL, POLKA_REMOTE_WS, KSM_REMOTE_WS, PASKAL2MMHG
from .base import Device

# Legacy Insight firmware sent SCD4x readings as "co:" instead of "co2:".
# Old builds do not send device_model — use SS58 allowlist only (no heuristics).
# Signature is verified on the original sensordatavalues string; alias is parse-time only.
LEGACY_CO2_AS_CO_ADDRESSES = frozenset({
    "4GS6AHES49wdTpo1p3BvGG616SAn8cDPokjXQxs5KB8dbG8J",  # cyprus
    "4ErRoB79d9fp2o9YCvTkHYAooTDp49PeVtamGW6pwo2jDEQf",  # moscow
    "4E8onUwyvQUCxSD56MYpyaGKnPraKTWJZ9ruMj1aLLRk7Vk8",  # ulan-ude
})

# Subscription lookups open a websocket to the parachain (~1.5 s each) and ran
# on every POST. Cache the answer per (owner, address): a sensor posts every
# ~30 s, membership changes rarely.
SUB_CACHE_TTL_POSITIVE = 30 * 60
SUB_CACHE_TTL_NEGATIVE = 10 * 60
_sub_cache: dict = {}
_sub_cache_lock = threading.Lock()


def _sub_cache_get(key: tuple):
    with _sub_cache_lock:
        entry = _sub_cache.get(key)
    if entry is None:
        return None
    value, expires = entry
    if time.time() > expires:
        return None
    return value


def _sub_cache_put(key: tuple, value: bool) -> None:
    ttl = SUB_CACHE_TTL_POSITIVE if value else SUB_CACHE_TTL_NEGATIVE
    with _sub_cache_lock:
        _sub_cache[key] = (value, time.time() + ttl)


@dataclass(repr=False, eq=False)
class Altruist(Device):
    """Represents a Robonomics Altruist Sensor.

    :param data: Unparsed data from the sensor.
    """

    data: dict = field(repr=False)

    def __post_init__(self) -> None:
        """Parse data from sensor and store into the corresponding variables."""

        super().__post_init__()
        self.id = str(self.data["robonomics_address"])
        self.model = int(self.data.get("model", SDS011_MODEL))
        self.public = self.id
        self.donated_by = str(self.data.get("donated_by", ""))
        self.owner = str(self.data.get("owner", ""))
        self.device_model = str(self.data.get("device_model", ""))
        sensors_data: str = self.data["sensordatavalues"]
        self.measurement = {}
        if not self._check_signature(sensors_data):
            print(f"Altruist sensor: Signature is not verified")
            return
        elif not self._is_address_in_subscription():
            print(f"Altruist sensor: Address is not in subscription")
            return
        self.geo_lat = self.data["GPS_lat"]
        self.geo_lon = self.data["GPS_lon"]

        self._measurements_formatter(sensors_data)
        self.timestamp = int(time.time())
        self.measurement.update({"timestamp": self.timestamp})

    def _check_signature(self, sensor_data: str) -> bool:
        """Verifies data with specified signature.
        :param sensor_data: Unparsed data from the sensor.
        :return: True if data is signed with this Keypair, otherwise False
        """

        sender_keypair = Keypair(
            ss58_address=self.public, crypto_type=KeypairType.ED25519
        )
        timestamp = str(int(time.time()))[:-2]
        print(f'singature: {self.data["signature"]}')
        sensor_data = f"{sensor_data},time:{timestamp}"
        return sender_keypair.verify(sensor_data, f"0x{self.data['signature']}")

    def _is_address_in_subscription(self) -> bool:
        key = (str(self.data.get("owner", "")), self.public)
        cached = _sub_cache_get(key)
        if cached is not None:
            return cached
        result, lookup_failed = self._lookup_subscription()
        # A negative answer caused by an RPC error is not cached: the next
        # POST retries instead of dropping a registered sensor for 10 minutes.
        if result or not lookup_failed:
            _sub_cache_put(key, result)
        return result

    def _lookup_subscription(self) -> tuple:
        """:return: (in subscription, any lookup raised)"""
        lookup_failed = False
        try:
            account = Account(remote_ws=POLKA_REMOTE_WS)
            rws = RWS(account)
            if rws.is_in_sub(sub_owner_addr=self.data["owner"], addr=self.public):
                return True, False
        except Exception as e:
            lookup_failed = True
            print(f"Altruist polkadot: error: {e}")

        try:
            account_ksm = Account(remote_ws=KSM_REMOTE_WS)
            rws_ksm = RWS(account_ksm)
            if rws_ksm.is_in_sub(sub_owner_addr=self.data["owner"], addr=self.public):
                return True, False
        except Exception as e:
            lookup_failed = True
            print(f"Altruist kusama: error: {e}")

        print(f"Altruist owner: {self.data['owner']}")
        print(f"Altruist public: {self.public}")

        return False, lookup_failed

    def _apply_legacy_co2_alias(self, sensor_data_dict: dict) -> None:
        """Map legacy key co: -> co2: for allowlisted SS58 (old firmware, no device_model)."""
        if "co2" in sensor_data_dict or "co" not in sensor_data_dict:
            return
        if self.id not in LEGACY_CO2_AS_CO_ADDRESSES:
            return
        sensor_data_dict["co2"] = sensor_data_dict.pop("co")

    def _measurements_formatter(self, sensor_data: str) -> dict:
        mapping = {
            "temperature": "t",
            "pressure": "p",
            "humidity": "h",
            "pm10": "p1",
            "pm25": "p2",
            "noiseMax": "nm",
            "noiseAvg": "na",
            "radiation": "gc",
            "dose_rate_nsv_h": "dose_rate_nsv_h",
            "count_rate_cps": "count_rate_cps",
            "dose_rate_error_pct": "dose_rate_error_pct",
            "count_rate_error_pct": "count_rate_error_pct",
            "CO2": "co2",
            "TVOC": "vc",
            "CO": "co",
            "o3": "o3",
            "no2": "no2"
        }
        sensor_data_dict = dict(item.split(":", 1) for item in sensor_data.split(","))

        def _parse_value(value: str):
            # Encrypted values: "e.<base64(IV+ciphertext)>" — pass through as string.
            if isinstance(value, str) and value.startswith("e."):
                return value
            return float(value)

        sensor_data_dict = {key: _parse_value(value) for key, value in sensor_data_dict.items()}
        self._apply_legacy_co2_alias(sensor_data_dict)

        self.measurement.update({
            key: (
                round(sensor_data_dict[value] / PASKAL2MMHG, 2)
                if key == "pressure" and isinstance(sensor_data_dict[value], (int, float))
                else sensor_data_dict[value]
            )
            for key, value in mapping.items()
            if value in sensor_data_dict
        })

    def __str__(self) -> str:
        if self.model == SDS011_MODEL:
            return (
                f"{{Public: {self.public}, geo: ({self.geo_lat},{self.geo_lon}), "
                f"model: {self.model}, donated_by: {self.donated_by}, owner: {self.owner}, "
                f"device_model: {self.device_model}, measurements: {self.measurement}}}"
            )

        self.measurement.update({"geo": f"{self.geo_lat},{self.geo_lon}"})

        return (
            f"{{Public: {self.public}, model: {self.model}, donated_by: {self.donated_by}, "
            f"owner: {self.owner}, device_model: {self.device_model}, "
            f"measurements: {self.measurement}}}"
        )
