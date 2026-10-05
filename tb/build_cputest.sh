#!/bin/sh
# Verilate the cputest replay bench (tb_cputest.sv) and assemble its monitor.
set -e
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-$(cd .. && pwd)/obj}
mkdir -p "$WORK"
python3 ../tools/ucode.py
./build_asm.sh cputest_mon
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Werror-PINMISSING -Werror-IMPLICIT -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_cputest --Mdir "$WORK/obj_cputest" -o tb_cputest \
	-I"$RTL" -I"$RTL/fpu" -CFLAGS "-O2 -march=native" \
	$RTL/ap68040_pkg.sv $RTL/gen/ap68040_upkg.sv $RTL/ap68040_alu.sv $RTL/ap68040_alu_slow.sv $RTL/ap68040_muldiv.sv \
	$RTL/ap68040_biu.sv $RTL/ap68040_ram.sv $RTL/ap68040_atc.sv $RTL/ap68040_dmu.sv $RTL/ap68040_predec.sv $RTL/ap68040_fetch.sv \
	$RTL/ap68040_decode.sv $RTL/ap68040_useq.sv $RTL/fpu/ap040_fpu.v $RTL/ap68040_fpif.sv $RTL/ap68040_backend.sv \
	$RTL/ap68040.sv m68040_bus_slave.sv tb_cputest.sv
echo "built $WORK/obj_cputest/tb_cputest"
