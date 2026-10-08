#!/bin/sh
# Verilate the pipelined-FPU differential bench (tb_fpu_pipe.sv).
set -e
cd "$(dirname "$0")"
WORK=${WORK:-$(cd .. && pwd)/obj}
# REF=old: compare against the committed FPU (git HEAD), renamed
EXTRA=""
if [ "$REF" = old ]; then
	git show HEAD:rtl/fpu/ap040_fpu.v | sed 's/^module ap040_fpu$/module ap040_fpu_ref/' > "$WORK/ap040_fpu_ref.v"
	EXTRA="+define+REF_OLD $WORK/ap040_fpu_ref.v"
fi
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_fpu_pipe --Mdir "$WORK/obj_fpu_pipe${REF:+_$REF}" -o tb_fpu_pipe \
	-I../rtl/fpu -CFLAGS "-O2 -march=native" \
	../rtl/fpu/ap040_fpu.v $EXTRA tb_fpu_pipe.sv
echo "built $WORK/obj_fpu_pipe${REF:+_$REF}/tb_fpu_pipe"
