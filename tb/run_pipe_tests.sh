#!/bin/sh
# AP040_PIPE self-test suite -- the pipelined core under active development
# (rtl/ap040_pipe_core.v and its stage files). Needs only iverilog: every
# tb_ap040_pipe_*.v bench pokes its own tiny program directly into
# ap040_inst_fetch.v's ROM (dut.u_if.rom[...]) at time 0, so there is no
# assembler dependency here the way run_tests.sh has for the rtl_old suite.
#
# rtl/ap040_pipe_core.v is deliberately self-contained (see its header) --
# this is the complete source list, no other rtl/ or rtl_old/ file is ever
# needed to build it.
set -eu
cd "$(dirname "$0")"

RTL=../rtl
WORK=build
mkdir -p "$WORK"

SRC="$RTL/ap040_pipe_core.v $RTL/ap040_inst_fetch.v $RTL/ap040_decode.v \
     $RTL/ap040_ea_calc.v $RTL/ap040_ea_fetch.v $RTL/ap040_execute.v \
     $RTL/ap040_writeback.v $RTL/ap040_pipe_alu.v $RTL/ap040_pipe_regfile.v \
     $RTL/ap040_pipe_l1.v"

# tb_ap040_pipe_l1_wbuf.v tests ap040_pipe_l1.v standalone, not through
# ap040_pipe_core.v (no pipeline instruction drives wren_b yet) -- give it
# just the one file it needs. Compiling it against the full SRC above would
# put two top-level-instantiable modules (this tb AND ap040_pipe_core.v
# itself) in one compilation unit, which iverilog treats as two independent
# simulation roots -- not wrong exactly, just pointless and confusing.
L1_SRC="$RTL/ap040_pipe_l1.v"

TESTS="nop moveq add bra bcc bccw bccl scc dbcc move_mem move_disp jmp bsr jsr exc sup rts_rte addrerr"

echo "== compiling pipe benches =="
for t in $TESTS; do
	iverilog -g2012 -I "$RTL" -o "$WORK/tb_pipe_$t.vvp" "tb_ap040_pipe_$t.v" $SRC
done
iverilog -g2012 -I "$RTL" -o "$WORK/tb_pipe_l1_wbuf.vvp" tb_ap040_pipe_l1_wbuf.v $L1_SRC

echo "== running =="
fail=0
for t in $TESTS; do
	if vvp "$WORK/tb_pipe_$t.vvp" 2>&1 | tee "$WORK/pipe_$t.log" | grep -q "ALL TESTS PASSED"; then
		echo "  pass  $t"
	else
		echo "  FAIL  $t  (see $WORK/pipe_$t.log)"
		fail=1
	fi
done
if vvp "$WORK/tb_pipe_l1_wbuf.vvp" 2>&1 | tee "$WORK/pipe_l1_wbuf.log" | grep -q "ALL TESTS PASSED"; then
	echo "  pass  l1_wbuf"
else
	echo "  FAIL  l1_wbuf  (see $WORK/pipe_l1_wbuf.log)"
	fail=1
fi

if [ $fail -eq 0 ]; then echo "AP040_PIPE: ALL TESTS PASSED"; else echo "AP040_PIPE: FAILURES"; exit 1; fi
