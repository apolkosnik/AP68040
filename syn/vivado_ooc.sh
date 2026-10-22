#!/bin/sh
# syn/vivado_ooc.sh <out dir name> [period_ns]: Vivado OOC fit (vivado_ooc.tcl)
# of R=<rtl dir> (default ../rtl); prints WNS and the LUT count.
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../rtl}
O=$here/$1
mkdir -p "$O"
${VIVADO:-/opt/Xilinx/Vivado/2023.2/bin/vivado} -mode batch -nojournal -log "$O/vivado.log" \
	-source "$here/vivado_ooc.tcl" -tclargs "$(cd "$R" && pwd)" "$O" $2 > "$O/run.out" 2>&1
awk '/WNS\(ns\)/ {getline; getline; print "WNS " $1 " ns, failing endpoints " $3; exit}' "$O/route_timing.rpt"
grep -m1 -E '^\| Slice LUTs' "$O/route_util.rpt"
