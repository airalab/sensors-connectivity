#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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


print("1. Generating signing key...")
signing_key = nacl.signing.SigningKey.generate()
signing_array = [int(x) for x in bytes(signing_key)]
signing_letters = [hex(x).upper().replace('X', 'x') for x in signing_array]

print(signing_key)
print(bytes(signing_key).hex())

print("2. Deriving verify key...")
verify_key = signing_key.verify_key
verify_array = [int(x) for x in bytes(verify_key)]
verify_letters = [hex(x).upper().replace('X', 'x') for x in verify_array]

print(verify_key)
print(bytes(verify_key).hex())

print("3. Writing keys to 'secrets.h' file...")
with open("secrets.h", "w") as f:
    f.write("uint8_t signing_key[32] = {\n  ")
    f.write(write_array(signing_letters))
    f.write("};\n\nuint8_t verifying_key[32] = {\n  ")
    f.write(write_array(verify_letters))
    f.write("};")

