#!/bin/sh
# Assemble the AP040 self-test programs into $readmemh images.
# Requires vasmm68k_mot (vbcc toolchain).

set -e
cd "$(dirname "$0")"

VASM=${VASM:-/opt/amiga-cc/vbcc/bin/vasmm68k_mot}
mkdir -p build

# bench_loop and bench_alu are measurement programs, not regression legs:
# they are built here so they cannot rot, and run by hand under +prof to
# compare cache and dispatch configurations (see AUDIT_20260816.md and
# PERFORMANCE.md).  bench_alu was previously built only by run_verilator.py;
# its image was simply absent, and a +prog pointing at the
# missing file produced a full, plausible profile of the core executing
# zeros -- see the $fopen guard in tb_ap040_program.v.
for t in t_integer t_exceptions t_mmu t_bitfield_mmu t_bitfield_cache t_moves_fc t_cache t_fpu bench_loop bench_alu; do
	$VASM -Fbin -m68040 -no-opt -o build/$t.bin asm/$t.s
	python3 bin2hex.py build/$t.bin build/$t.hex
	echo "built build/$t.hex"
done
# Compiled C measurement programs (c/*.c): vbcc for the 68040, linked flat
# behind c/start.s (vectors at 0, code from $400) by vlink.  dhry is
# Dhrystone 2.1, self-checking against the published final values.
VC=${VC:-/opt/amiga-cc/vbcc/bin/vc}
VLINK=${VLINK:-/opt/amiga-cc/vbcc/bin/vlink}
export VBCC=${VBCC:-/opt/amiga-cc/vbcc}
$VASM -quiet -Fhunk -m68040 -o build/start.o c/start.s
for t in dhry; do
	$VC +aos68k -c -O2 -speed -cpu=68040 -fpu=68040 -c99 -o build/$t.o c/$t.c
	$VLINK -brawbin1 -o build/$t.bin build/start.o build/$t.o
	python3 bin2hex.py build/$t.bin build/$t.hex
	echo "built build/$t.hex"
done
