#!/bin/sh
# AP68040 self-test suite -- the WORKING sequential core (rtl_old/), kept as
# the golden reference while the pipelined core in rtl/ is built up to
# replace it (see tb/run_pipe_verilator.py). Do not repoint RTL below at
# rtl/ -- that directory holds the pipelined core and does not implement
# this module set; see AP040_IMPLEMENTATION_PLAN.md.
#
# Needs Verilator (5.x: --binary --timing) and vasmm68k_mot (vbcc).  The core
# names signals before it declares them, which Verilator accepts and Icarus
# (13.0 included) rejects at elaboration, so this suite no longer runs
# iverilog; the Minimig-AGA tree this core is developed in runs Verilator
# only.  Everything here runs against the core alone -- no host-project
# sources -- so a failure is the CPU's, not an integration artifact.  That
# tree adds further benches that co-simulate it with a real chipset, SDRAM
# and DDR3 controller; those live there because they need those modules.
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
JOBS=${JOBS:-8}
RTL=../rtl_old
WORK=build
mkdir -p "$WORK"

SRC="$RTL/ap040_tg68k_compat.v $RTL/ap040_core.v $RTL/ap040_bus16_adapter.v \
     $RTL/ap040_bus_timeout.v $RTL/ap040_regfile.v $RTL/ap040_alu.v \
     $RTL/ap040_muldiv.v $RTL/ap040_mmu.v $RTL/ap040_cache.v $RTL/ap040_fpu.v \
     $RTL/ap040_walker_cdc.v $RTL/primitives/dpram.v"

# the regression programs the Minimig tree runs on this core
# (tests/ap040/run_verilator.py); bench_loop and bench_alu are measurement
# programs and are only assembled here so they cannot rot
PROGRAMS="t_integer t_fastpaths t_exceptions t_mmu t_movem_restart t_atcprobe t_bitfield_mmu \
          t_bitfield_cache t_moves_fc t_cinv_moves t_fault_edges t_agu t_walk_order \
          t_moves_alt t_smc_mmu t_cache t_fpu"

echo "== assembling test programs =="
for t in $PROGRAMS bench_loop bench_alu; do
	$VASM -Fbin -m68040 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

fail=0
# build NAME TOP [verilator arguments and sources...] -> $WORK/obj-NAME/VTOP
build() {
	name=$1; top=$2; shift 2
	if verilator --binary --timing -Wno-fatal -j "$JOBS" -I"$RTL" --top-module "$top" \
		--Mdir "$WORK/obj-$name" "$@" > "$WORK/$name.build.log" 2>&1; then
		echo "  built $name"
	else
		echo "  FAIL  build $name  (see $WORK/$name.build.log)"
		fail=1
	fi
}

echo "== compiling benches =="
build regfile tb_ap040_regfile tb_ap040_regfile.v $RTL/ap040_regfile.v
build alu_arithmetic tb_ap040_alu_arithmetic tb_ap040_alu_arithmetic.v $RTL/ap040_alu.v
build fpu_normalize tb_ap040_fpu_normalize \
	tb_ap040_fpu_normalize.v $RTL/ap040_fpu.v $RTL/ap040_regfile.v $RTL/primitives/dpram.v
build prog   tb_ap040_program      tb_ap040_program.v $SRC
build reset  tb_ap040_reset        tb_ap040_reset.v $SRC
build dblflt tb_ap040_double_fault tb_ap040_double_fault.v $SRC
build walker tb_ap040_walker_cdc   tb_ap040_walker_cdc.v $RTL/ap040_walker_cdc.v
build bus16  tb_ap040_bus16_gap    tb_ap040_bus16_gap.v $RTL/ap040_bus16_adapter.v
build timeout tb_ap040_bus_timeout tb_ap040_bus_timeout.v $RTL/ap040_bus_timeout.v
build snoop  tb_ap040_cache_snoop \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v
# the same bench at the 4:1 clock enable, and both with the tag row's
# mixed-port read-during-write modelled, which is what makes the lookup
# guard load-bearing at all.  dpram answers that don't-care read with a
# pseudo-random word rather than X, so the controls below fail on concrete
# wrong data -- see tb_ap040_cache_snoop.v
build snoop_ce4 tb_ap040_cache_snoop -GCE_DIV=4 \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v
build snoop_x tb_ap040_cache_snoop -DSNOOP_MIXED_X \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v
build snoop_x_ce4 tb_ap040_cache_snoop -DSNOOP_MIXED_X -GCE_DIV=4 \
	tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v

echo "== running =="
run() {
	name=$1; shift
	if "$@" 2>&1 | tee "$WORK/$name.log" | grep -q "ALL TESTS PASSED"; then
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
	if "$@" 2>&1 | tee "$WORK/$name.log" | grep -q "TEST FAILED"; then
		echo "  pass  $name  (control: failed as required)"
	else
		echo "  FAIL  $name  (control did NOT fail; see $WORK/$name.log)"
		fail=1
	fi
}
bin() { echo "$WORK/obj-$1/V$2"; }
run reset        "$(bin reset tb_ap040_reset)"
run regfile       "$(bin regfile tb_ap040_regfile)"
run regfile_poison "$(bin regfile tb_ap040_regfile)" +poison
negrun regfile_bypass_control "$(bin regfile tb_ap040_regfile)" +poison +disable_bypass
run alu_arithmetic "$(bin alu_arithmetic tb_ap040_alu_arithmetic)"
run fpu_normalize "$(bin fpu_normalize tb_ap040_fpu_normalize)"
run double_fault "$(bin dblflt tb_ap040_double_fault)"
run walker_cdc   "$(bin walker tb_ap040_walker_cdc)"
run bus16_gap    "$(bin bus16 tb_ap040_bus16_gap)"
run bus_timeout  "$(bin timeout tb_ap040_bus_timeout)"
run cache_snoop  "$(bin snoop tb_ap040_cache_snoop)"
run cache_snoop_ce4       "$(bin snoop_ce4 tb_ap040_cache_snoop)"
run cache_snoop_x         "$(bin snoop_x tb_ap040_cache_snoop)"
run cache_snoop_x_lkw     "$(bin snoop_x tb_ap040_cache_snoop)"     +inj_look_whole
run cache_snoop_x_ce4     "$(bin snoop_x_ce4 tb_ap040_cache_snoop)"
run cache_snoop_x_ce4_accw "$(bin snoop_x_ce4 tb_ap040_cache_snoop)" +inj_acc_whole
run cache_snoop_x_ce4_accs "$(bin snoop_x_ce4 tb_ap040_cache_snoop)" +inj_acc_settle
negrun cache_snoop_x_neg_accw   "$(bin snoop_x tb_ap040_cache_snoop)"     +inj_acc_whole
negrun cache_snoop_x_ce4_neg_lkw "$(bin snoop_x_ce4 tb_ap040_cache_snoop)" +inj_look_whole
for t in $PROGRAMS; do
	run "${t#t_}" "$(bin prog tb_ap040_program)" "+prog=$WORK/$t.hex"
done

if ! sh ./run_fpu_frames.sh; then fail=1; fi

if [ $fail -eq 0 ]; then echo "AP68040: ALL TESTS PASSED"; else echo "AP68040: FAILURES"; exit 1; fi
