#!/bin/sh
# Verilate the program bench.  Object tree under $WORK (default
# /home/adam/ap68040-60/obj, NOT /tmp: it is a small tmpfs on this host).
set -e
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-$(cd .. && pwd)/obj}
mkdir -p "$WORK"
python3 ../tools/ucode.py
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_ap68040 --Mdir "$WORK/obj_prog" -o tb_ap68040 \
	-I"$RTL" -CFLAGS "-O2 -march=native" \
	$RTL/ap68040_pkg.sv $RTL/gen/ap68040_upkg.sv $RTL/ap68040_alu.sv $RTL/ap68040_muldiv.sv \
	$RTL/ap68040_biu.sv $RTL/ap68040_dmu.sv $RTL/ap68040_fetch.sv \
	$RTL/ap68040_decode.sv $RTL/ap68040_useq.sv $RTL/ap68040_backend.sv \
	$RTL/ap68040.sv m68040_bus_slave.sv tb_ap68040.sv
echo "built $WORK/obj_prog/tb_ap68040"
