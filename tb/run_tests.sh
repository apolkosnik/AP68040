#!/bin/sh
# AP68040-60 regression: every program under five bus configurations.
# Usage: run_tests.sh [program ...]   (default: the passing set)
set -e
cd "$(dirname "$0")"
PROGS=${*:-"smoke t_integer t_exceptions t_mmu t_cache t_atcprobe t_bitfield_cache t_bitfield_mmu t_movem_restart t_moves_fc t_fpu t_fpu_frames t_fpu_resume"}
./build_asm.sh $PROGS
./build_sim.sh > build/sim_build.log 2>&1 || { tail -30 build/sim_build.log; exit 1; }
SIM=$(cd .. && pwd)/obj/obj_prog/tb_ap68040
fail=0
for p in $PROGS; do
	for cfg in "" "+waits" "+waits +tbi=2 +retry=10" "+bclk2" "+bclk2 +waits +tbi=1 +retry=20"; do
		out=$($SIM +prog=build/$p.hex $cfg 2>&1 | grep -E "^(PASS|FAIL)" | tail -1)
		printf '%-14s %-34s %s\n' "$p" "${cfg:-(zero wait)}" "$out"
		case "$out" in PASS*) ;; *) fail=1 ;; esac
	done
done
[ $fail = 0 ] && echo "ALL PASSED" || { echo "FAILURES"; exit 1; }
