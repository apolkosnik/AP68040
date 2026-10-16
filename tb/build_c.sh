#!/bin/sh
# Compile tb/c/<name>.c with vbcc for the program bench: build/<name>.hex
# (crt0.s first, linked at 0).  VBCC=<vbcc root>, OPT=<-O level>,
# CDEFS=<-D options>, SUFFIX=<appended to the image name>, ASFLAGS for crt0
# (-DCOPYBACK: user mode, copyback data).  Headers also come from build/.
set -e
cd "$(dirname "$0")"
VBCC=${VBCC:-/opt/amiga-cc/vbcc}
OPT=${OPT:-1023}
mkdir -p build
$VBCC/bin/vasmm68k_mot -quiet -Fvobj -m68060 $ASFLAGS -o build/crt0.o c/crt0.s
for t in "$@"; do
	o=build/$t$SUFFIX
	$VBCC/bin/vbccm68k -quiet -c99 -O=$OPT -cpu=68060 -fpu=68060 -Ibuild $CDEFS c/$t.c -o=$o.s
	$VBCC/bin/vasmm68k_mot -quiet -Fvobj -m68060 -o $o.o $o.s
	$VBCC/bin/vlink -brawbin1 -Ttext 0 -nostdlib -o $o.bin build/crt0.o $o.o
	python3 bin2hex.py $o.bin $o.hex
done
