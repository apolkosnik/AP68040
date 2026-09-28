#!/bin/sh
# Assemble the AP040 self-test programs into $readmemh images.
# Requires vasmm68k_mot (vbcc toolchain).

set -e
cd "$(dirname "$0")"

VASM=${VASM:-/opt/amiga-cc/vbcc/bin/vasmm68k_mot}
mkdir -p build

# bench_loop is a measurement program, not a regression leg: it is built
# here so it cannot rot, and run by hand under +prof to compare cache
# configurations (see AUDIT_20260816.md, cache re-enable trial).
for t in t_integer t_exceptions t_mmu t_bitfield_mmu t_bitfield_cache t_moves_fc t_movem_restart t_atcprobe t_fpu_frames t_fpu_resume t_cache t_fpu t_branch_early t_loops_irq bench_loop pipe_bench branch_bench; do
	$VASM -Fbin -m68040 -no-opt -o build/$t.bin asm/$t.s
	python3 bin2hex.py build/$t.bin build/$t.hex
	echo "built build/$t.hex"
done
