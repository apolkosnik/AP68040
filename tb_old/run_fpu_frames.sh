#!/bin/sh
# Standalone wire-format regressions for both supported FPU revisions.
set -eu
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-build/fpu_frames}
VASM=${VASM:-vasmm68k_mot}
mkdir -p "$WORK"
for revision in 64 65; do
    define=
    if [ "$revision" = 64 ]; then define=-DREV40=1; fi
    iverilog -g2012 -I "$RTL" -s tb_ap040_program \
        -P tb_ap040_program.FPU_REVISION="$revision" \
        -o "$WORK/frames_$revision.vvp" tb_ap040_program.v "$RTL"/*.v "$RTL/primitives/dpram.v"
    for test in frames resume; do
        "$VASM" -m68040 -Fbin -no-opt -quiet $define \
            -o "$WORK/${test}_$revision.bin" "asm/t_fpu_${test}.s"
        python3 bin2hex.py "$WORK/${test}_$revision.bin" "$WORK/${test}_$revision.hex"
        vvp "$WORK/frames_$revision.vvp" +prog="$WORK/${test}_$revision.hex" \
            > "$WORK/${test}_$revision.log" 2>&1
        if ! grep -q 'ALL TESTS PASSED' "$WORK/${test}_$revision.log"; then
            echo "FAIL $test revision=$revision: see $WORK/${test}_$revision.log" >&2
            exit 1
        fi
        echo "PASS $test revision=$revision (three bus phases)"
    done
done
