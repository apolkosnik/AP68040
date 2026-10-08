#!/bin/sh
# Verilate the pipelined-FPU differential bench (tb_fpu_pipe.sv).
set -e
cd "$(dirname "$0")"
WORK=${WORK:-$(cd .. && pwd)/obj}
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_fpu_pipe --Mdir "$WORK/obj_fpu_pipe" -o tb_fpu_pipe \
	-I../rtl/fpu -CFLAGS "-O2 -march=native" \
	../rtl/fpu/ap040_fpu.v tb_fpu_pipe.sv
echo "built $WORK/obj_fpu_pipe/tb_fpu_pipe"
