#!/usr/bin/env python3
"""Exhaustive check of isa.py against WinUAE's 68040 opcode table."""
import sys
from isa import lookup, T
ORACLE = sys.argv[1] if len(sys.argv) > 1 else '/home/adam/ap68040-60/tools-oracle/op040.txt'
SZ = {0: 'B', 1: 'W', 2: 'L'}
bad = 0
names = {}
for line in open(ORACLE):
    f = line.split()
    op = int(f[0], 16)
    valid = f[1] == '1'
    name = f[2]
    size = int(f[3]); plev = int(f[8])
    e = lookup(op)
    mine_valid = e is not None and e.name != 'ILLG'
    if valid != mine_valid or (valid and e.name != name):
        bad += 1
        if bad <= 40:
            print('%04x oracle %s %s  mine %s' % (op, 'valid' if valid else 'ILLG', name,
                  (e.name + '/' + e.rt) if e else 'none'))
        continue
    if valid:
        if plev == 2 and not e.priv:
            bad += 1; print('%04x %s: oracle privileged' % (op, name))
        # MOVEC: table68k leaves privilege to the handler; the 040 traps
        if plev in (0, 1) and e.priv and name not in ('MVSR2', 'MOVEC2', 'MOVE2C'):
            bad += 1; print('%04x %s: oracle not privileged' % (op, name))
print('mismatches:', bad)
sys.exit(1 if bad else 0)
