#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import nacl.signing


def main():
    signing_key = nacl.signing.SigningKey.generate()
    signing_key_hex = bytes(signing_key).hex()

    print(f"Signing key: {signing_key_hex}")

    verify_key = signing_key.verify_key
    verify_key_hex = bytes(verify_key).hex()

    print(f"Verifying key: {verify_key_hex}")

if __name__ == "__main__":
    main()

