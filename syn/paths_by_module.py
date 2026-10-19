#!/usr/bin/env python3
"""Group a fit's failing setup paths (failing.txt from sta_report.tcl) by
source and destination module: syn/paths_by_module.py <fit dir> [n]"""
import re, sys, collections
d = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
txt = open(f"{d}/failing.txt").read()
recs = re.findall(r"Slack\s*:\s*(-?[\d.]+).*?From Node\s*:\s*(\S+).*?To Node\s*:\s*(\S+)", txt, re.S)
def mod(node):
    parts = node.split("|")
    inst = [p.split(":")[-1] for p in parts[:-1]]
    reg = re.sub(r"\[\d+\]", "", parts[-1])
    return "/".join(inst[:2]) or "top", reg
grp = collections.defaultdict(list)
for sl, f, t in recs:
    fm, fr = mod(f); tm, tr = mod(t)
    grp[(fm, fr, tm, tr)].append(float(sl))
print(f"{len(recs)} failing paths; total negative slack {sum(float(r[0]) for r in recs):.1f} ns")
for k, v in sorted(grp.items(), key=lambda kv: min(kv[1]))[:n]:
    print(f"{min(v):7.3f} {len(v):5d}  {k[0]}.{k[1]} -> {k[2]}.{k[3]}")
