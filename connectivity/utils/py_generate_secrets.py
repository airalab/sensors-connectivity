#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import nacl.signing
import logging.config
from config.logging import LOGGING_CONFIG

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger(__name__)


def main() -> None:
    signing_key = nacl.signing.SigningKey.generate()
    signing_key_hex = bytes(signing_key).hex()

    print(f"Signing key: {signing_key_hex}")

    verify_key = signing_key.verify_key
    verify_key_hex = bytes(verify_key).hex()

    print(f"Verifying key: {verify_key_hex}")


if __name__ == "__main__":
    main()
