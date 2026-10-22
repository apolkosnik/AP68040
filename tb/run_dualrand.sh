#!/bin/bash
# Dual-issue differential test: random programs (tools/dualrand/gen.py) on
# the core with dual issue and without, under the bus configurations of
# run_tests.sh; every checkpoint's registers, CCR and the final buffer
# checksum must agree.
# Usage: run_dualrand.sh [first seed] [count]   JOBS=<n>, BUILD=0 to reuse
# the simulators, DEFS for both builds (e.g. +define+AP_EARLY_RES=0)
set -e
cd "$(dirname "$0")"
WORK=${WORK:-$(cd .. && pwd)/obj}
JOBS=${JOBS:-$(( $(nproc) / 2 ))}
S0=${1:-1}; N=${2:-50}
VASM=${VASM:-/opt/amiga-cc/vbcc/bin/vasmm68k_mot}
if [ "${BUILD:-1}" = 1 ]; then
	DEFS="$DEFS +define+AP_DUAL=1" WORK=$WORK/dr1 ./build_sim.sh > build/dr1_build.log 2>&1 || { tail -20 build/dr1_build.log; exit 1; }
	DEFS="$DEFS +define+AP_DUAL=0" WORK=$WORK/dr0 ./build_sim.sh > build/dr0_build.log 2>&1 || { tail -20 build/dr0_build.log; exit 1; }
fi
mkdir -p build/dualrand
CFGS=("" "+waits" "+waits +tbi=2 +retry=10" "+bclk2" "+bclk2 +waits +tbi=1 +retry=20")
export WORK VASM
seq $S0 $((S0 + N - 1)) | xargs -P "$JOBS" -I{} bash -c '
	s={}; p=build/dualrand/r$s
	python3 ../tools/dualrand/gen.py $s $p.s 40
	$VASM -quiet -Fbin -m68040 -no-opt -o $p.bin $p.s 2>/dev/null
	python3 bin2hex.py $p.bin $p.hex
	c=$(( s % 5 ))
	cfgs=("" "+waits" "+waits +tbi=2 +retry=10" "+bclk2" "+bclk2 +waits +tbi=1 +retry=20")
	cfg=${cfgs[$c]}
	o1=$($WORK/dr1/obj_prog/tb_ap68040 +prog=$p.hex $cfg +timeout=8000000 2>&1)
	o0=$($WORK/dr0/obj_prog/tb_ap68040 +prog=$p.hex $cfg +timeout=8000000 2>&1)
	r1=$(echo "$o1" | grep -E "^(PASS|FAIL)" | tail -1); r0=$(echo "$o0" | grep -E "^(PASS|FAIL)" | tail -1)
	if [ "$(echo "$o1" | grep ^STAMP | awk "{print \$2}")" != "$(echo "$o0" | grep ^STAMP | awk "{print \$2}")" ] ||
	   [ "${r1%% *}" != PASS ] || [ "${r0%% *}" != PASS ]; then
		echo "seed $s ${cfg:-(zero wait)}: DIFFER ($r1 | $r0)"
	else
		c1=${r1##*cycles=}; c0=${r0##*cycles=}
		echo "seed $s ${cfg:-(zero wait)}: same, cycles $c1 vs $c0"
	fi' | sort -n -k2 | tee build/dualrand/summary.txt
grep -q DIFFER build/dualrand/summary.txt && { echo "DUALRAND FAILURES"; exit 1; }
awk '/same/ {a += $(NF-2); b += $NF} END {printf "ALL SAME: %d cycles dual, %d single (%.1f%%)\n", a, b, 100*(b-a)/b}' build/dualrand/summary.txt
