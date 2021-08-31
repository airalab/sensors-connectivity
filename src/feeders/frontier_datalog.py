from substrateinterface import SubstrateInterface, Keypair
from substrateinterface.exceptions import SubstrateRequestException
import rospy

from feeders import IFeeder
from stations import StationData, Measurement
from drivers.ping import PING_MODEL

class FrontierFeeder(IFeeder):
    def __init__(self, config: dict):
        super().__init__(config)
        self.substrate = SubstrateInterface(
            url="wss://main.frontier.rpc.robonomics.network",
            ss58_format=32,
            type_registry_preset="substrate-node-template",
            type_registry={
                "types": {
                    "Record": "Vec<u8>",
                    "<T as frame_system::Config>::AccountId": "AccountId",
                    "RingBufferItem": {
                        "type": "struct",
                        "type_mapping": [
                            ["timestamp", "Compact<u64>"],
                            ["payload", "Vec<u8>"],
                        ],
                    },
                }
            }
        )
        self.keypair = Keypair.create_from_mnemonic(self.config["frontier"]["suri"]) 

    def feed(self, data: [StationData]):
        if self.config["frontier"]["enable"]:
            for d in data:
                call = self.substrate.compose_call(
                    call_module = "Datalog",
                    call_function = "record",
                    call_params = {
                        "record": f"{d.measurement}"
                    }
                )
                extrinsic = self.substrate.create_signed_extrinsic(call=call, keypair=self.keypair)
                rospy.loginfo(f"Extrincsic is: {extrinsic}")
                try:
                    receipt = self.substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
                    rospy.loginfo(f'Data sent to Robonomics datalog and included in block {receipt.block_hash}')
                except SubstrateRequestException as e:
                    rospy.loginfo(f'Something went wrong during extrinsic submission to Robonomics: {e}')


