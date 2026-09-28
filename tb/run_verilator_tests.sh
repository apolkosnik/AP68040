#!/bin/sh
# AP68040 self-test suite under Verilator: the benches, programs and pass
# rules of run_tests.sh, compiled with --binary --timing instead of iverilog.
# Icarus 13 rejects the core (signals used before their declaration) and
# Icarus 12 is an order of magnitude slower on the program benches.
#
# Needs verilator (5.x) and vasmm68k_mot (vbcc).
# Usage: sh run_verilator_tests.sh [workdir]
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
RTL=../rtl
WORK=${1:-${CPU_TEST_WORK:-build/verilator}}
JOBS=${VERILATOR_JOBS:-$(nproc 2>/dev/null || echo 8)}
OPT_FLAGS=
if [ "${CPU_TEST_LEA:-0}" = 1 ]; then OPT_FLAGS="$OPT_FLAGS -DAP040_EXPERIMENTAL_LEA"; fi
if [ "${CPU_TEST_XSTORE:-0}" = 1 ]; then OPT_FLAGS="$OPT_FLAGS -DAP040_EXPERIMENTAL_XSTORE"; fi
mkdir -p "$WORK"

SRC="$RTL/ap040_tg68k_compat.v $RTL/ap040_core.v $RTL/ap040_bus16_adapter.v \
     $RTL/ap040_bus_timeout.v $RTL/ap040_regfile.v $RTL/ap040_alu.v \
     $RTL/ap040_muldiv.v $RTL/ap040_mmu.v $RTL/ap040_cache.v $RTL/ap040_fpu.v \
     $RTL/ap040_walker_cdc.v $RTL/primitives/dpram.v"

PROGS="t_integer t_exceptions t_mmu t_bitfield_mmu t_bitfield_cache t_moves_fc t_movem_restart t_atcprobe t_fpu_frames t_fpu_resume t_cache t_fpu t_branch_early t_loops_irq t_refill_load t_lea_d16 t_lea_fault"

echo "== assembling test programs =="
for t in $PROGS bench_loop pipe_bench branch_bench; do
	$VASM -Fbin -m68040 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

VFLAGS="--binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-MULTIDRIVEN \
        -Wno-PINMISSING -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-TIMESCALEMOD \
        --output-split 20000 --output-split-cfuncs 500 -CFLAGS -O2"

# build <name> <top> <files...>: one binary per bench, built in parallel
pids=
build() {
	name=$1; top=$2; shift 2
	# shellcheck disable=SC2086
	( verilator $VFLAGS $OPT_FLAGS -I"$RTL" --top-module "$top" \
	    --Mdir "$WORK/obj_$name" -o "tb_$name" "$@" \
	    > "$WORK/build_$name.log" 2>&1 || { echo "  BUILD FAIL  $name  (see $WORK/build_$name.log)"; exit 1; } ) &
	pids="$pids $!"
}

echo "== compiling benches =="
build reset          tb_ap040_reset           tb_ap040_reset.v $SRC
build regfile        tb_ap040_regfile         tb_ap040_regfile.v $RTL/ap040_regfile.v
build alu_arithmetic tb_ap040_alu_arithmetic  tb_ap040_alu_arithmetic.v $RTL/ap040_alu.v
build fpu_normalize  tb_ap040_fpu_normalize   tb_ap040_fpu_normalize.v $RTL/ap040_fpu.v $RTL/ap040_regfile.v $RTL/primitives/dpram.v
build dblflt         tb_ap040_double_fault    tb_ap040_double_fault.v $SRC
build walker         tb_ap040_walker_cdc      tb_ap040_walker_cdc.v $RTL/ap040_walker_cdc.v
build bus16          tb_ap040_bus16_gap       tb_ap040_bus16_gap.v $RTL/ap040_bus16_adapter.v
build timeout        tb_ap040_bus_timeout     tb_ap040_bus_timeout.v $RTL/ap040_bus_timeout.v
build snoop          tb_ap040_cache_snoop     tb_ap040_cache_snoop.v $RTL/ap040_cache.v $RTL/primitives/dpram.v
build xstore         tb_ap040_cache_xstore    -DAP040_EXPERIMENTAL_XSTORE tb_ap040_cache_xstore.sv $RTL/ap040_cache.v $RTL/primitives/dpram.v
build prog           tb_ap040_program         tb_ap040_program.v $SRC
bfail=0
for p in $pids; do wait "$p" || bfail=1; done
if [ $bfail -ne 0 ]; then echo "AP68040: BUILD FAILURES"; exit 1; fi

echo "== running =="
fail=0
bin() { echo "$WORK/obj_$1/tb_$1"; }
run() {
	name=$1; shift
	sim_rc=0
	"$@" > "$WORK/$name.log" 2>&1 || sim_rc=$?
	if [ "$sim_rc" -eq 0 ] && grep -q "ALL TESTS PASSED" "$WORK/$name.log" &&
	   ! grep -Eq 'FATAL:|TEST FAILED' "$WORK/$name.log"; then
		echo "  pass  $name"
	else
		echo "  FAIL  $name  (see $WORK/$name.log)"
		fail=1
	fi
}
# A negative leg passes only when the bench FAILS (see run_tests.sh).
negrun() {
	name=$1; shift
	sim_rc=0
	"$@" > "$WORK/$name.log" 2>&1 || sim_rc=$?
	if [ "$sim_rc" -ne 0 ] && grep -q "TEST FAILED" "$WORK/$name.log"; then
		echo "  pass  $name  (control: failed as required)"
	else
		echo "  FAIL  $name  (control did NOT fail; see $WORK/$name.log)"
		fail=1
	fi
}
run reset          "$(bin reset)"
run regfile        "$(bin regfile)"
run regfile_poison "$(bin regfile)" +poison
negrun regfile_bypass_control       "$(bin regfile)" +poison +disable_bypass
negrun regfile_extra_bypass_control "$(bin regfile)" +poison +disable_extra_bypass
negrun regfile_fifth_bypass_control "$(bin regfile)" +poison +disable_fifth_bypass
run alu_arithmetic "$(bin alu_arithmetic)"
run fpu_normalize  "$(bin fpu_normalize)"
run double_fault   "$(bin dblflt)"
run walker_cdc     "$(bin walker)"
run bus16_gap      "$(bin bus16)"
run bus_timeout    "$(bin timeout)"
run cache_snoop    "$(bin snoop)"
run cache_xstore   "$(bin xstore)"
for t in $PROGS; do
	run "${t#t_}" "$(bin prog)" "+prog=$WORK/$t.hex"
done

if [ $fail -eq 0 ]; then echo "AP68040: ALL TESTS PASSED"; else echo "AP68040: FAILURES"; exit 1; fi
