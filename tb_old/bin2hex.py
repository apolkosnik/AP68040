#!/usr/bin/env python3
"""Convert a flat binary into one 16-bit hex word per line for $readmemh."""
import sys

def main():
    if len(sys.argv) != 3:
        print("usage: bin2hex.py <in.bin> <out.hex>")
        return 1
    data = open(sys.argv[1], "rb").read()
    if len(data) % 2:
        data += b"\x00"
    with open(sys.argv[2], "w") as out:
        for i in range(0, len(data), 2):
            out.write("%02x%02x\n" % (data[i], data[i + 1]))
    return 0

if __name__ == "__main__":
    sys.exit(main())
