#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import nacl.signing


def write_array(arr: []) -> str:
    buff = ""
    i = 0
    while i < 32:
        buff += arr[i]
        if i != 31:
            buff += ", "

        i += 1
        if i % 8 == 0:
            buff += "\n  "
    return buff

def main():
    parser = argparse.ArgumentParser(description="Generate signing and public key for Arduino/ESP firmware")

    parser.add_argument("-o", "--output", metavar="path", type=str, default="", help="folder to store secret.h file")

    args = parser.parse_args()

    folder_path = args.output if args.output else "."
    if not folder_path.endswith("/"):
        folder_path += "/"

    signing_key = nacl.signing.SigningKey.generate()
    signing_array = [int(x) for x in bytes(signing_key)]
    signing_letters = [ "0x{0:02X}".format(x) for x in signing_array]

    verify_key = signing_key.verify_key
    verify_array = [int(x) for x in bytes(verify_key)]
    verify_letters = [ "0x{0:02X}".format(x) for x in verify_array]
    verify_key_hex = bytes(verify_key).hex()

    print("Put the following key to connectivities' ACL:")
    print(verify_key_hex)

    save_path = folder_path + "secrets.h"
    with open(save_path, "w") as f:
        f.write("uint8_t signing_key[32] = {\n  ")
        f.write(write_array(signing_letters))
        f.write(f"}};\n\n// {verify_key_hex}\nuint8_t verifying_key[32] = {{\n  ")
        f.write(write_array(verify_letters))
        f.write("};")

    print(f"\nSaved to '{save_path}'")


if __name__ == "__main__":
    main()

