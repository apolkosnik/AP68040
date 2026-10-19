#!/bin/sh
# Out-of-context fit of the whole core on the MiSTer part (5CSEBA6U23I7) at
# 65 MHz: syn/fitcore.sh <work dir name> [mkooc script]
#   R=<rtl dir>  (default: the rtl next to this script)   SEED=<n>
#   PERIOD=<ns>  (default 15.385, 65 MHz)
# Every port is a virtual pin, so the result is the core's register-to-
# register timing alone.  Results: <name>/ap68040.sta.summary (all corners),
# <name>/paths.txt (worst setup paths).  Judge a change on three seeds.
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../rtl}
S=${2:-$here/mkooc.sh}
cd "$here"
exec $S $1 ap68040 ${PERIOD:-15.385} $R/ap68040_pkg.sv $R/gen/ap68040_upkg.sv $R/ap68040_alu.sv $R/ap68040_alu_slow.sv $R/ap68040_muldiv.sv $R/ap68040_biu.sv $R/ap68040_ram.sv $R/ap68040_atc.sv $R/ap68040_dmu.sv $R/ap68040_predec.sv $R/ap68040_fetch.sv $R/ap68040_decode.sv $R/ap68040_useq.sv $R/fpu/ap040_fpu.v $R/ap68040_fpif.sv $R/ap68040_backend.sv $R/ap68040_snoop.sv $R/ap68040.sv
