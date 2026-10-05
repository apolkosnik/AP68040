#!/bin/sh
# Compile tb/c/<name>.c with vbcc for the program bench: build/<name>.hex
# (crt0.s first, linked at 0).  VBCC=<vbcc root>, OPT=<-O level>.
set -e
cd "$(dirname "$0")"
VBCC=${VBCC:-/opt/amiga-cc/vbcc}
OPT=${OPT:-1023}
mkdir -p build
$VBCC/bin/vasmm68k_mot -quiet -Fvobj -m68040 $ASFLAGS -o build/crt0.o c/crt0.s
for t in "$@"; do
	$VBCC/bin/vbccm68k -quiet -c99 -O=$OPT -cpu=68040 -fpu=68040 c/$t.c -o=build/$t.s
	$VBCC/bin/vasmm68k_mot -quiet -Fvobj -m68040 -o build/$t.o build/$t.s
	$VBCC/bin/vlink -brawbin1 -Ttext 0 -nostdlib -o build/$t.bin build/crt0.o build/$t.o
	python3 bin2hex.py build/$t.bin build/$t.hex
done
