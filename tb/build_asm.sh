#!/bin/sh
# Assemble tb/asm/<name>.s into build/<name>.hex (one 16-bit word per line)
set -e
cd "$(dirname "$0")"
VASM=${VASM:-/opt/amiga-cc/vbcc/bin/vasmm68k_mot}
mkdir -p build
for t in "$@"; do
	$VASM -quiet -Fbin -m68040 -no-opt -o build/$t.bin asm/$t.s
	python3 bin2hex.py build/$t.bin build/$t.hex
done
