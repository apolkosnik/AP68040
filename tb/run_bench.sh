#!/bin/bash
# Dhrystone 2.1 (c/dhry.c) and Whetstone (c/whet.c) on the core, built by
# build_c.sh with the Amiga-like start-up (crt0.s -DCOPYBACK: user mode,
# copyback data, both caches
# on).  Each runs at two counts; two stamps bracket the timed loop, and the
# difference of the two counts gives the steady-state cost without start-up
# or cold caches.  Whetstone's reference results come from the same source
# compiled on the host (cc -DHOST).  Scores at MHZ (default 60).
# Usage: run_bench.sh [plusargs...]     SIM=<simulator>, or WORK as for
# build_sim.sh (default ../obj)
set -e
cd "$(dirname "$0")"
WORK=${WORK:-$(cd .. && pwd)/obj}
SIM=${SIM:-$WORK/obj_prog/tb_ap68040}
MHZ=${MHZ:-60}
export ASFLAGS=-DCOPYBACK
for n in 2000 4000; do
	CDEFS=-DNUMBER_OF_RUNS=$n SUFFIX=_$n ./build_c.sh dhry > /dev/null
done
for n in 2 4; do
	cc -O0 -DHOST -DLOOPS=$n -o build/whet_host c/whet.c -lm
	./build/whet_host > build/whet_ref.h
	CDEFS=-DLOOPS=$n SUFFIX=_$n ./build_c.sh whet > /dev/null
done
cyc() {   # image -> the timed loop's cycles (the second stamp); fails on a bad run
	local out
	out=$($SIM +prog=build/$1.hex +timeout=60000000 "${@:2}" 2>&1)
	echo "$out" | grep -q "^PASS" || { echo "$1: $(echo "$out" | grep FAIL | head -2)" >&2; exit 1; }
	echo "$out" | grep "^STAMP" | sed -n 2p | sed 's/.*cycles=\([0-9]*\).*/\1/'
}
d1=$(cyc dhry_2000 "$@"); d2=$(cyc dhry_4000 "$@")
w1=$(cyc whet_2 "$@");    w2=$(cyc whet_4 "$@")
python3 - "$d1" "$d2" "$w1" "$w2" "$MHZ" <<'EOF'
import sys
d1, d2, w1, w2, mhz = map(float, sys.argv[1:])
dc = (d2 - d1) / 2000          # cycles per Dhrystone
wc = (w2 - w1) / 2             # cycles per Whetstone loop (100,000 Whetstone instructions)
dps = mhz * 1e6 / dc
print(f"Dhrystone 2.1: {dc:.1f} cycles/iteration, {dps:,.0f} Dhrystones/s, "
      f"{dps/1757:.2f} DMIPS at {mhz:g} MHz ({1e6/(dc*1757):.3f} DMIPS/MHz)")
print(f"Whetstone:     {wc:,.0f} cycles/loop, {0.1*mhz*1e6/wc:.2f} MWIPS at {mhz:g} MHz "
      f"({0.1e6/wc:.4f} MWIPS/MHz)")
EOF
