#!/bin/sh
# Assemble the AP040 self-test programs into $readmemh images.
# Requires vasmm68k_mot (vbcc toolchain) -- set VASM if it isn't on PATH
# under the plain name (the previous hardcoded path only existed on one
# machine).

set -e
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
mkdir -p build

# bench_loop is a measurement program, not a regression leg: it is built
# here so it cannot rot, and run by hand under +prof to compare cache
# configurations (see AUDIT_20260816.md, cache re-enable trial).
for t in t_integer t_exceptions t_mmu t_cache t_fpu bench_loop; do
	$VASM -Fbin -m68040 -no-opt -o build/$t.bin asm/$t.s
	python3 bin2hex.py build/$t.bin build/$t.hex
	echo "built build/$t.hex"
done
