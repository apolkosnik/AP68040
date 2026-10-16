#!/bin/sh
# Verilate and run the exhaustive decoder bench (tb_decode.sv): every
# operation word through the pattern decoder against tools/ucode.py's
# model, and the block RAM predecoder against the pattern decoder.
set -e
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-$(cd .. && pwd)/obj}
mkdir -p "$WORK"
python3 ../tools/ucode.py > /dev/null
verilator --binary --timing -j 8 --build-jobs 8 -O2 \
	-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
	--top-module tb_decode --Mdir "$WORK/obj_dec" -o tb_decode \
	-I"$RTL" $RTL/ap68040_pkg.sv $RTL/gen/ap68040_upkg.sv $RTL/ap68040_predec.sv tb_decode.sv > "$WORK/obj_dec_build.log" 2>&1 || { tail -20 "$WORK/obj_dec_build.log"; exit 1; }
"$WORK/obj_dec/tb_decode" | grep -E "DECODE|op "
