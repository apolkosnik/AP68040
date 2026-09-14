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
WORK=build
mkdir -p "$WORK"

SRC="$RTL/ap040_tg68k_compat.v $RTL/ap040_core.v $RTL/ap040_bus16_adapter.v \
     $RTL/ap040_bus_timeout.v $RTL/ap040_regfile.v $RTL/ap040_alu.v \
     $RTL/ap040_muldiv.v $RTL/ap040_mmu.v $RTL/ap040_cache.v $RTL/ap040_fpu.v \
     $RTL/ap040_walker_cdc.v $RTL/primitives/dpram.v"

echo "== assembling test programs =="
for t in t_integer t_exceptions t_mmu t_bitfield_mmu t_bitfield_cache t_moves_fc t_cache t_fpu bench_loop; do
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
# the same bench at the 4:1 clock enable, and both with the tag row's
# mixed-port read-during-write modelled, which is what makes the lookup
# guard load-bearing at all
iverilog -g2012 -I "$RTL" -s tb_ap040_cache_snoop -P tb_ap040_cache_snoop.CE_DIV=4 \
	-o "$WORK/tb_snoop_ce4.vvp" \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v
iverilog -g2012 -I "$RTL" -s tb_ap040_cache_snoop -DSNOOP_MIXED_X \
	-o "$WORK/tb_snoop_x.vvp" \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v
iverilog -g2012 -I "$RTL" -s tb_ap040_cache_snoop -DSNOOP_MIXED_X \
	-P tb_ap040_cache_snoop.CE_DIV=4 -o "$WORK/tb_snoop_x_ce4.vvp" \
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
# A negative leg passes only when the bench FAILS.  Each lookup-guard term is
# load-bearing in one clock-enable regime and redundant in the other, so
# blinding it must break the bench at its own divide -- a guard that cannot be
# shown to matter is not being tested.
negrun() {
	name=$1; shift
	if vvp "$@" 2>&1 | tee "$WORK/$name.log" | grep -q "TEST FAILED"; then
		echo "  pass  $name  (control: failed as required)"
	else
		echo "  FAIL  $name  (control did NOT fail; see $WORK/$name.log)"
		fail=1
	fi
}
run reset        "$WORK/tb_reset.vvp"
run double_fault "$WORK/tb_dblflt.vvp"
run walker_cdc   "$WORK/tb_walker.vvp"
run bus16_gap    "$WORK/tb_bus16.vvp"
run bus_timeout  "$WORK/tb_timeout.vvp"
run cache_snoop  "$WORK/tb_snoop.vvp"
run cache_snoop_ce4       "$WORK/tb_snoop_ce4.vvp"
run cache_snoop_x         "$WORK/tb_snoop_x.vvp"
run cache_snoop_x_lkw     "$WORK/tb_snoop_x.vvp"     +inj_look_whole
run cache_snoop_x_ce4     "$WORK/tb_snoop_x_ce4.vvp"
run cache_snoop_x_ce4_accw "$WORK/tb_snoop_x_ce4.vvp" +inj_acc_whole
run cache_snoop_x_ce4_accs "$WORK/tb_snoop_x_ce4.vvp" +inj_acc_settle
negrun cache_snoop_x_neg_accw   "$WORK/tb_snoop_x.vvp"     +inj_acc_whole
negrun cache_snoop_x_ce4_neg_lkw "$WORK/tb_snoop_x_ce4.vvp" +inj_look_whole
for t in integer exceptions mmu bitfield_mmu bitfield_cache moves_fc cache fpu; do
	run "$t" "$WORK/tb_prog.vvp" "+prog=$WORK/t_$t.hex"
done

if [ $fail -eq 0 ]; then echo "AP68040: ALL TESTS PASSED"; else echo "AP68040: FAILURES"; exit 1; fi
