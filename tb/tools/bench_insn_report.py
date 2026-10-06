#!/usr/bin/env python3
"""Cycles per copy for each form of tb/asm/bench_insn.s, from the bench's
STAMP lines (stdin).  Each form: 8 copies x 100 passes; the per-pass DBRA
and setup are included (about 1-2 cycles a pass, /8 per copy)."""
import re, sys, importlib.util, os
here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("g", os.path.join(here, "gen_bench_insn.py"))
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
cyc = {}
for l in sys.stdin:
    m = re.match(r"STAMP tag=([0-9a-f]+) cycles=(\d+)", l.strip())
    if m:
        cyc[int(m.group(1), 16)] = int(m.group(2))
for i, (name, body, setup) in enumerate(g.FORMS):
    c = cyc.get(2 * i + 2)
    n = body.count("\n") + 1
    if c is not None:
        print("%-26s %6.2f cycles/copy  (%d instr/copy)" % (name, c / 800.0, n))
