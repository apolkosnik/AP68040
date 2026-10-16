#!/bin/sh
# Synthesis-only area estimate of the core with parameter overrides:
# syn/mapvar.sh <name> [PARAM=value ...]   (results: <name>/ap68040.map.rpt)
set -e
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../rtl}
name=$1; shift
d=$here/$name
rm -rf "$d"; mkdir -p "$d"
{
echo "set_global_assignment -name FAMILY \"Cyclone V\""
echo "set_global_assignment -name DEVICE 5CSEBA6U23I7"
echo "set_global_assignment -name TOP_LEVEL_ENTITY ap68040"
echo "set_global_assignment -name NUM_PARALLEL_PROCESSORS 4"
echo "set_instance_assignment -name VIRTUAL_PIN ON -to *"
for p in "$@"; do echo "set_parameter -name ${p%%=*} ${p#*=}"; done
for f in $R/ap68040_pkg.sv $R/gen/ap68040_upkg.sv $R/ap68040_alu.sv $R/ap68040_alu_slow.sv $R/ap68040_muldiv.sv $R/ap68040_biu.sv $R/ap68040_ram.sv $R/ap68040_atc.sv $R/ap68040_dmu.sv $R/ap68040_predec.sv $R/ap68040_fetch.sv $R/ap68040_decode.sv $R/ap68040_useq.sv $R/fpu/ap040_fpu.v $R/ap68040_fpif.sv $R/ap68040_backend.sv $R/ap68040_snoop.sv $R/ap68040.sv; do
  case "$f" in *.sv) echo "set_global_assignment -name SYSTEMVERILOG_FILE $(readlink -f $f)";; *) echo "set_global_assignment -name VERILOG_FILE $(readlink -f $f)";; esac
  echo "set_global_assignment -name SEARCH_PATH $(dirname $(readlink -f $f))"
done
} > "$d/ap68040.qsf"
echo 'PROJECT_REVISION = "ap68040"' > "$d/ap68040.qpf"
cd "$d"
quartus_map ap68040 > map.log 2>&1 || { echo "$name: MAP FAILED"; tail -20 map.log; exit 1; }
grep -E "Estimate of Logic utilization|Total registers" ap68040.map.rpt | head -3 | sed "s/^/$name /"
