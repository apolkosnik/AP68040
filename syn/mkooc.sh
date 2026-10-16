#!/bin/sh
# usage: mkooc.sh <name> <top> <period_ns> <srcfile>...
# Out-of-context Quartus fit on the MiSTer part: every port is a virtual
# pin, so the result is register-to-register timing of the module alone.
set -e
name=$1; top=$2; per=$3; shift 3
d=$(pwd)/$name
rm -rf "$d"; mkdir -p "$d"
{
echo "set_global_assignment -name FAMILY \"Cyclone V\""
echo "set_global_assignment -name DEVICE 5CSEBA6U23I7"
echo "set_global_assignment -name TOP_LEVEL_ENTITY $top"
echo "set_global_assignment -name SDC_FILE ooc.sdc"
echo "set_global_assignment -name OPTIMIZATION_MODE \"${OPTMODE:-HIGH PERFORMANCE EFFORT}\""
echo "set_global_assignment -name NUM_PARALLEL_PROCESSORS 4"
echo "set_global_assignment -name OPTIMIZE_HOLD_TIMING \"ALL PATHS\""
# register retiming off: with it on, the fitter adds ~6,000 registers
# (13,000 retiming moves); the core is closed at 60 MHz without it
echo "set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_RETIMING ${RETIME:-OFF}"
# PHYS=ON: physical synthesis of combinational logic and register
# duplication (a measurement of what the fitter can add)
if [ "$PHYS" = ON ]; then
  echo "set_global_assignment -name PHYSICAL_SYNTHESIS_COMBO_LOGIC ON"
  echo "set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION ON"
fi
echo "set_instance_assignment -name VIRTUAL_PIN ON -to *"
[ -n "$SEED" ] && echo "set_global_assignment -name SEED $SEED"
for f in "$@"; do
  case "$f" in
    *.sv|*.svh) echo "set_global_assignment -name SYSTEMVERILOG_FILE $(readlink -f $f)";;
    *.vhd) echo "set_global_assignment -name VHDL_FILE $(readlink -f $f)";;
    *) echo "set_global_assignment -name VERILOG_FILE $(readlink -f $f)";;
  esac
  echo "set_global_assignment -name SEARCH_PATH $(dirname $(readlink -f $f))"
done
} > "$d/$top.qsf"
echo "PROJECT_REVISION = \"$top\"" > "$d/$top.qpf"
cat > "$d/ooc.sdc" <<S
create_clock -name clk -period $per [get_ports clk]
derive_clock_uncertainty
S
cd "$d"
quartus_sh --flow compile $top > flow.log 2>&1 || { echo "$name: FLOW FAILED"; tail -20 flow.log; exit 1; }
quartus_sta -t ../sta_report.tcl $top > sta.log 2>&1 || true
grep -E 'Fmax|slack|WORST' sta.log | head -20
