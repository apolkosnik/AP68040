#!/bin/sh
# AP68040 self-test suite.
#
# Needs iverilog and vasmm68k_mot (vbcc).  Everything here runs against the
# core alone -- no host-project sources -- so a failure is the CPU's, not an
# integration artifact.  The Minimig-AGA tree this core is developed in adds
# further benches that co-simulate it with a real chipset, SDRAM and DDR3
# controller; those live there because they need those modules.
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
RTL=../rtl
WORK=${CPU_TEST_WORK:-build}
OPT_FLAGS=
if [ "${CPU_TEST_LEA:-0}" = 1 ]; then OPT_FLAGS="$OPT_FLAGS -DAP040_EXPERIMENTAL_LEA"; fi
if [ "${CPU_TEST_XSTORE:-0}" = 1 ]; then OPT_FLAGS="$OPT_FLAGS -DAP040_EXPERIMENTAL_XSTORE"; fi
mkdir -p "$WORK"

SRC="$RTL/ap040_tg68k_compat.v $RTL/ap040_core.v $RTL/ap040_bus16_adapter.v \
     $RTL/ap040_bus_timeout.v $RTL/ap040_regfile.v $RTL/ap040_alu.v \
     $RTL/ap040_muldiv.v $RTL/ap040_mmu.v $RTL/ap040_cache.v $RTL/ap040_fpu.v \
     $RTL/ap040_walker_cdc.v $RTL/primitives/dpram.v"

echo "== assembling test programs =="
for t in t_integer t_exceptions t_mmu t_bitfield_mmu t_bitfield_cache t_moves_fc t_movem_restart t_atcprobe t_fpu_frames t_fpu_resume t_cache t_fpu t_branch_early t_loops_irq t_refill_load t_lea_d16 t_lea_fault bench_loop pipe_bench branch_bench; do
	$VASM -Fbin -m68040 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

echo "== compiling benches =="
# unit benches for the shared datapaths (Adam Polkosnik 7431dcb): exhaustive
# byte ADD/ADDX/SUB/SUBX/CMP against an oracle, and the three FPU
# normalization states against a serial-shift reference
iverilog -g2012 $OPT_FLAGS -I "$RTL" -s tb_ap040_regfile -o "$WORK/tb_regfile.vvp"  tb_ap040_regfile.v $RTL/ap040_regfile.v
iverilog -g2012 $OPT_FLAGS -I "$RTL" -s tb_ap040_alu_arithmetic -o "$WORK/tb_alu_arithmetic.vvp"  tb_ap040_alu_arithmetic.v $RTL/ap040_alu.v
iverilog -g2012 $OPT_FLAGS -I "$RTL" -s tb_ap040_fpu_normalize -o "$WORK/tb_fpu_normalize.vvp"  tb_ap040_fpu_normalize.v $RTL/ap040_fpu.v $RTL/ap040_regfile.v $RTL/primitives/dpram.v
iverilog -g2012 $OPT_FLAGS -I "$RTL" -o "$WORK/tb_prog.vvp"      tb_ap040_program.v $SRC
iverilog -g2012 $OPT_FLAGS -I "$RTL" -o "$WORK/tb_reset.vvp"     tb_ap040_reset.v $SRC
iverilog -g2012 $OPT_FLAGS -I "$RTL" -o "$WORK/tb_dblflt.vvp"    tb_ap040_double_fault.v $SRC
iverilog -g2012 $OPT_FLAGS -I "$RTL" -o "$WORK/tb_walker.vvp"    tb_ap040_walker_cdc.v $RTL/ap040_walker_cdc.v
iverilog -g2012 $OPT_FLAGS -I "$RTL" -o "$WORK/tb_bus16.vvp"     tb_ap040_bus16_gap.v $RTL/ap040_bus16_adapter.v
iverilog -g2012 $OPT_FLAGS -I "$RTL" -o "$WORK/tb_timeout.vvp"   tb_ap040_bus_timeout.v $RTL/ap040_bus_timeout.v
iverilog -g2012 $OPT_FLAGS -I "$RTL" -s tb_ap040_cache_snoop -o "$WORK/tb_snoop.vvp" \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v

iverilog -g2012 $OPT_FLAGS -DAP040_EXPERIMENTAL_XSTORE -I "$RTL" -s tb_ap040_cache_xstore -o "$WORK/tb_xstore.vvp" \
	tb_ap040_cache_xstore.sv $RTL/ap040_cache.v $RTL/primitives/dpram.v

echo "== running =="
fail=0
run() {
	name=$1; shift
	# Wait for the simulator to finish: grep -q in a pipeline can accept a
	# pass banner before a final assertion fails, and hides vvp's status.
	sim_rc=0
	vvp "$@" > "$WORK/$name.log" 2>&1 || sim_rc=$?
	if [ "$sim_rc" -eq 0 ] && grep -q "ALL TESTS PASSED" "$WORK/$name.log" &&
	   ! grep -Eq 'FATAL:|TEST FAILED' "$WORK/$name.log"; then
		echo "  pass  $name"
	else
		echo "  FAIL  $name  (see $WORK/$name.log)"
		fail=1
	fi
}
# A negative leg passes only when the bench FAILS: the register file's
# pending-write bypass must be shown to matter, so the poisoned RAM word with
# the bypass disabled has to be caught.
negrun() {
	name=$1; shift
	sim_rc=0
	vvp "$@" > "$WORK/$name.log" 2>&1 || sim_rc=$?
	if [ "$sim_rc" -ne 0 ] && grep -q "TEST FAILED" "$WORK/$name.log"; then
		echo "  pass  $name  (control: failed as required)"
	else
		echo "  FAIL  $name  (control did NOT fail; see $WORK/$name.log)"
		fail=1
	fi
}
run reset        "$WORK/tb_reset.vvp"
run regfile       "$WORK/tb_regfile.vvp"
run regfile_poison "$WORK/tb_regfile.vvp" +poison
negrun regfile_bypass_control "$WORK/tb_regfile.vvp" +poison +disable_bypass
negrun regfile_extra_bypass_control "$WORK/tb_regfile.vvp" +poison +disable_extra_bypass
negrun regfile_fifth_bypass_control "$WORK/tb_regfile.vvp" +poison +disable_fifth_bypass
run alu_arithmetic "$WORK/tb_alu_arithmetic.vvp"
run fpu_normalize "$WORK/tb_fpu_normalize.vvp"
run double_fault "$WORK/tb_dblflt.vvp"
run walker_cdc   "$WORK/tb_walker.vvp"
run bus16_gap    "$WORK/tb_bus16.vvp"
run bus_timeout  "$WORK/tb_timeout.vvp"
run cache_snoop  "$WORK/tb_snoop.vvp"
run cache_xstore "$WORK/tb_xstore.vvp"
for t in integer exceptions mmu bitfield_mmu bitfield_cache moves_fc movem_restart atcprobe fpu_frames fpu_resume cache fpu branch_early loops_irq refill_load lea_d16 lea_fault; do
	run "$t" "$WORK/tb_prog.vvp" "+prog=$WORK/t_$t.hex"
done

if [ $fail -eq 0 ]; then echo "AP68040: ALL TESTS PASSED"; else echo "AP68040: FAILURES"; exit 1; fi
