#!/bin/sh
# Standalone wire-format regressions for both supported FPU revisions.
# Verilator, as run_tests.sh and for the same reason.
set -eu
cd "$(dirname "$0")"
RTL=../rtl_old
WORK=${WORK:-build/fpu_frames}
VASM=${VASM:-vasmm68k_mot}
JOBS=${JOBS:-8}
mkdir -p "$WORK"
for revision in 64 65; do
    define=
    if [ "$revision" = 64 ]; then define=-DREV40=1; fi
    if ! verilator --binary --timing -Wno-fatal -j "$JOBS" -I"$RTL" \
        --top-module tb_ap040_program -GFPU_REVISION="$revision" \
        --Mdir "$WORK/obj-frames_$revision" \
        tb_ap040_program.v "$RTL"/*.v "$RTL/primitives/dpram.v" \
        > "$WORK/frames_$revision.build.log" 2>&1; then
        echo "FAIL build revision=$revision: see $WORK/frames_$revision.build.log" >&2
        exit 1
    fi
    for test in frames resume; do
        "$VASM" -m68040 -Fbin -no-opt -quiet $define \
            -o "$WORK/${test}_$revision.bin" "asm/t_fpu_${test}.s"
        python3 bin2hex.py "$WORK/${test}_$revision.bin" "$WORK/${test}_$revision.hex"
        "$WORK/obj-frames_$revision/Vtb_ap040_program" +prog="$WORK/${test}_$revision.hex" \
            > "$WORK/${test}_$revision.log" 2>&1 || true
        if ! grep -q 'ALL TESTS PASSED' "$WORK/${test}_$revision.log"; then
            echo "FAIL $test revision=$revision: see $WORK/${test}_$revision.log" >&2
            exit 1
        fi
        echo "PASS $test revision=$revision (three bus phases)"
    done
done
