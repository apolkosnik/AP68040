#!/bin/sh
# AP68040 self-test suite -- the WORKING sequential core (rtl_old/), kept as
# the golden reference while rtl/ap040_pipe_* is built up to replace it (see
# tb/run_pipe_tests.sh). Do not repoint RTL below at rtl/ -- that directory
# holds the pipelined core under construction and does not implement this
# module set; see AP040_IMPLEMENTATION_PLAN.md.
#
# Needs iverilog and vasmm68k_mot (vbcc).  Everything here runs against the
# core alone -- no host-project sources -- so a failure is the CPU's, not an
# integration artifact.  The Minimig-AGA tree this core is developed in adds
# further benches that co-simulate it with a real chipset, SDRAM and DDR3
# controller; those live there because they need those modules.
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
RTL=../rtl_old
WORK=build
mkdir -p "$WORK"

SRC="$RTL/ap040_tg68k_compat.v $RTL/ap040_core.v $RTL/ap040_bus16_adapter.v \
     $RTL/ap040_bus_timeout.v $RTL/ap040_regfile.v $RTL/ap040_alu.v \
     $RTL/ap040_muldiv.v $RTL/ap040_mmu.v $RTL/ap040_cache.v $RTL/ap040_fpu.v \
     $RTL/ap040_walker_cdc.v $RTL/primitives/dpram.v"

echo "== assembling test programs =="
for t in t_integer t_exceptions t_mmu t_cache t_fpu bench_loop; do
	$VASM -Fbin -m68040 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

echo "== compiling benches =="
iverilog -g2012 -I "$RTL" -o "$WORK/tb_prog.vvp"      tb_ap040_program.v $SRC
iverilog -g2012 -I "$RTL" -o "$WORK/tb_reset.vvp"     tb_ap040_reset.v $SRC
iverilog -g2012 -I "$RTL" -o "$WORK/tb_dblflt.vvp"    tb_ap040_double_fault.v $SRC
iverilog -g2012 -I "$RTL" -o "$WORK/tb_walker.vvp"    tb_ap040_walker_cdc.v $RTL/ap040_walker_cdc.v
iverilog -g2012 -I "$RTL" -o "$WORK/tb_bus16.vvp"     tb_ap040_bus16_gap.v $RTL/ap040_bus16_adapter.v
iverilog -g2012 -I "$RTL" -o "$WORK/tb_timeout.vvp"   tb_ap040_bus_timeout.v $RTL/ap040_bus_timeout.v
iverilog -g2012 -I "$RTL" -s tb_ap040_cache_snoop -o "$WORK/tb_snoop.vvp" \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v

echo "== running =="
fail=0
run() {
	name=$1; shift
	if vvp "$@" 2>&1 | tee "$WORK/$name.log" | grep -q "ALL TESTS PASSED"; then
		echo "  pass  $name"
	else
		echo "  FAIL  $name  (see $WORK/$name.log)"
		fail=1
	fi
}
run reset        "$WORK/tb_reset.vvp"
run double_fault "$WORK/tb_dblflt.vvp"
run walker_cdc   "$WORK/tb_walker.vvp"
run bus16_gap    "$WORK/tb_bus16.vvp"
run bus_timeout  "$WORK/tb_timeout.vvp"
run cache_snoop  "$WORK/tb_snoop.vvp"
for t in integer exceptions mmu cache fpu; do
	run "$t" "$WORK/tb_prog.vvp" "+prog=$WORK/t_$t.hex"
done

if [ $fail -eq 0 ]; then echo "AP68040: ALL TESTS PASSED"; else echo "AP68040: FAILURES"; exit 1; fi
