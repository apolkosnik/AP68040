#!/bin/sh
# trymap.sh <name>: snapshot ../rtl to the scratchpad and print the synthesis
# estimate (ALMs needed) and per-entity combinational ALUTs
S=/tmp/claude-1000/-home-adam-AP68040/80c3ea1a-a635-4e98-8200-1d32096f0561/scratchpad
n=$1
rm -rf $S/x_$n && cp -r /home/adam/AP68040/rtl $S/x_$n
cd /home/adam/AP68040/syn && shift; R=$S/x_$n ./mapvar.sh map_$n "$@" > /home/adam/AP68040/obj/syn/map_$n.log 2>&1
grep "ALMs needed" /home/adam/AP68040/obj/syn/map_$n.log | head -1
awk '/Analysis & Synthesis Resource Utilization by Entity/{f=1} f && /Compilation Hierarchy Node/ {n=$0; sub(/.*: /,"",n)} f && /^Combinational ALUTs/ {v=$0; sub(/.*: /,"",v)} f && /^Dedicated Logic Registers/ {print v"\t"n} f && /Partition Status/ {exit}' map_$n/ap68040.map.rpt | grep -v altsyncram | awk -F'\t' '{split($1,a," "); if (a[1]+0>=250) print}'
