from substrateinterface import Keypair, KeypairType

def sign_data(seed: str, data: str) -> str:
    keypair = Keypair.create_from_mnemonic(seed, crypto_type=KeypairType.SR25519)
    return keypair.sign(data).hex()
    