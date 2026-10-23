#!/usr/bin/env python3
"""Random instruction streams for the dual-issue differential test.

  gen.py <seed> <out.s> [blocks]

The program is rich in the second slot's instructions (one-word data and
address register operations) mixed with loads, stores, read-modify-writes,
(An)+/-(An), indexed modes, shifts, multiplies, Scc, forward branches over
data code and counted DBRA loops.  At every checkpoint it writes the CCR,
D0-D7 and A0-A4 to the bench's stamp register ($F108, one line per word),
and at the end a checksum of the data buffer; tb/run_dualrand.sh runs it on
the core with and without dual issue and compares the lines.

The address registers A0-A4 are tracked exactly (only data-independent
updates touch them) and kept inside the buffer at $40000-$4FFFF.
"""
import random, sys

# (the bench decodes A15-A0 alone for its registers at $F100-$F2FF: no
# address used has those low bits)
BUF_LO, BUF_HI, CENTER = 0x40000, 0x4E000, 0x47000
SAFE_LO, SAFE_HI = 0x41000, 0x4D000
STAMP = "$F108"
SAVE = "$30000"
M32 = 0xFFFFFFFF

rnd = random.Random(int(sys.argv[1]))

def dr(no=None):
    while True:
        r = "d%d" % rnd.randrange(8)
        if r != no: return r

def ar():
    return rnd.randrange(5)

def sz():
    return rnd.choice(".b .w .l".split())

def szn(s):
    return {".b": 1, ".w": 2, ".l": 4}[s]

def sx16(v):
    v &= 0xFFFF
    return (v - 0x10000 if v & 0x8000 else v) & M32

def nofn(A):
    pass

# an instruction is (text, model update)
def data_op(no=None):
    k = rnd.randrange(22)
    x, y = dr(no), dr()
    s = sz()
    q = rnd.randint(1, 8)
    if k == 0: t = "moveq\t#%d,%s" % (rnd.randint(-128, 127), x)
    elif k == 1: t = "move%s\t%s,%s" % (s, y, x)
    elif k == 2: t = "move.l\ta%d,%s" % (ar(), x)
    elif k == 3: t = "%s%s\t%s,%s" % (rnd.choice("add sub and or cmp".split()), s, y, x)
    elif k == 4: t = "eor%s\t%s,%s" % (s, y, x)
    elif k == 5: t = "%s%s\t#%d,%s" % (rnd.choice(["addq", "subq"]), s, q, x)
    elif k == 6: t = "tst%s\t%s" % (s, x)
    elif k == 7: t = "%s%s\t%s" % (rnd.choice("clr not neg".split()), s, x)
    elif k == 8: t = rnd.choice(["ext.w", "ext.l", "extb.l", "swap"]) + "\t" + x
    elif k == 9: t = "%s%s\t#%d,%s" % (rnd.choice("add sub and or eor cmp".split()), s,
                                       rnd.randint(0, 0xFFFF if s != ".b" else 0xFF), x)
    elif k == 10: t = "%s%s\t#%d,%s" % (rnd.choice("lsl lsr asl asr rol ror roxl roxr".split()), s, q, x)
    elif k == 11: t = "%s%s\t%s,%s" % (rnd.choice("lsl lsr asr ror".split()), s, y, x)
    elif k == 12: t = "%s.w\t%s,%s" % (rnd.choice(["mulu", "muls"]), y, x)
    elif k == 13: t = "%s%s\t%s,%s" % (rnd.choice(["addx", "subx"]), s, y, x)
    elif k == 14: t = "s%s\t%s" % (rnd.choice("eq ne cs cc mi pl vs vc hi ls ge lt gt le".split()), x)
    elif k == 15: t = "%s.l\ta%d,%s" % (rnd.choice(["add", "sub", "cmp"]), ar(), x)
    elif k == 16: t = "%s.w\ta%d,%s" % (rnd.choice(["add", "sub", "cmp", "move"]), ar(), x)
    elif k == 17:
        if no in (x, y): t = "nop"
        else: t = "exg\t%s,%s" % (y, x)
    elif k == 18: t = "btst\t%s,%s" % (y, x)
    elif k == 19: t = "bset\t#%d,%s" % (rnd.randrange(32), x)
    elif k == 20: t = "move.w\t%s,ccr" % y
    else: t = "negx%s\t%s" % (s, x)
    return (t, nofn)

def mem_ea(an, s, upd):
    n = szn(s)
    k = rnd.randrange(6) if upd else rnd.choice([0, 3, 4, 5])
    if k == 0: return "(a%d)" % an, nofn
    if k == 1:
        def f(A, an=an, n=n): A[an] = (A[an] + n) & M32
        return "(a%d)+" % an, f
    if k == 2:
        def f(A, an=an, n=n): A[an] = (A[an] - n) & M32
        return "-(a%d)" % an, f
    if k == 3: return "%d(a%d)" % (rnd.randint(-256, 256), an), nofn
    # indexed by another pointer's low word: the address must stay in the
    # buffer (checked against the model when it runs)
    xm, d = ar(), rnd.randint(-64, 64)
    def f(A, an=an, xm=xm, d=d):
        a = (A[an] + sx16(A[xm]) + d) & M32
        if not (BUF_LO <= a < BUF_HI): A["bad"] = 1
    return "%d(a%d,a%d.w)" % (d, an, xm), f

def both(f, g):
    def h(A):
        f(A); g(A)
    return h

def mem_op(no=None):
    an, s, x = ar(), sz(), dr(no)
    k = rnd.randrange(8)
    ea, f = mem_ea(an, s, True)
    if k == 0: t = "move%s\t%s,%s" % (s, ea, x)
    elif k == 1: t = "move%s\t%s,%s" % (s, x, ea)
    elif k == 2: t = "%s%s\t%s,%s" % (rnd.choice("add sub and or cmp".split()), s, ea, x)
    elif k == 3: t = "%s%s\t%s,%s" % (rnd.choice("add sub and or eor".split()), s, x, ea)
    elif k == 4: t = "%s%s\t%s" % (rnd.choice("clr not neg tst".split()), s, ea)
    elif k == 5: t = "%s%s\t#%d,%s" % (rnd.choice(["addq", "subq"]), s, rnd.randint(1, 8), ea)
    elif k == 6:
        bn = ar()
        eb, g = mem_ea(bn, s, True)
        if "," in eb: eb, g = "(a%d)" % bn, nofn
        t, f = "move%s\t%s,%s" % (s, ea, eb), both(f, g)
    else: t = "move.l\t%s,%s" % (ea, x) if s == ".l" else "move%s\t%s,%s" % (s, ea, x)
    return (t, f)

# a second slot's load: (An) or d16(An) operand of a MOVE, MOVEA.L, ALU op
# or TST, after a uop with no memory access
def ld_op(no=None):
    an, x, s = ar(), dr(no), sz()
    k = rnd.randrange(6)
    e = rnd.choice(["(a%d)" % an, "%d(a%d)" % (rnd.randint(-256, 256), an)])
    if k == 0: t = ["move%s\t%s,%s" % (s, e, x)]
    elif k == 1: t = ["%s%s\t%s,%s" % (rnd.choice("add sub and or cmp".split()), s, e, x)]
    elif k == 2: t = ["tst%s\t%s" % (s, e)]
    elif k == 3: t = ["movea.l\t%s,a5" % e, "move.l\ta5,%s" % x]
    elif k == 4: t = ["move.l\t%s,%s" % (rnd.choice(["(a7)", "%d(a7)" % rnd.randrange(-64, 64, 2)]), x)]
    else: t = ["cmp%s\t%s,%s" % (s, e, x), "s%s\t%s" % (rnd.choice("eq ne cs hi lt".split()), dr(no))]
    first = rnd.choice([data_op(no), data_op(no), addr_op()])
    return [first] + [(i, nofn) for i in t]

def addr_op():
    x, y = ar(), ar()
    k = rnd.randrange(7)
    q = rnd.randint(1, 8)
    if k == 0:
        def f(A, x=x, q=q): A[x] = (A[x] + q) & M32
        return ("addq.%s\t#%d,a%d" % (rnd.choice("wl"), q, x), f)
    if k == 1:
        def f(A, x=x, q=q): A[x] = (A[x] - q) & M32
        return ("subq.%s\t#%d,a%d" % (rnd.choice("wl"), q, x), f)
    if k == 2:
        def f(A, x=x, y=y): A[x] = A[y]
        return ("movea.l\ta%d,a%d" % (y, x), f)
    if k == 3:
        def f(A, x=x, y=y): A[x] = sx16(A[y])
        return ("movea.w\ta%d,a%d" % (y, x), f)
    if k == 4:
        d = rnd.randint(-32, 32)
        def f(A, x=x, y=y, d=d): A[x] = (A[y] + d) & M32
        return ("lea\t%d(a%d),a%d" % (d, y, x), f)
    if k == 5:
        d = rnd.randint(-100, 100)
        def f(A, x=x, d=d): A[x] = (A[x] + d) & M32
        return ("adda.l\t#%d,a%d" % (d, x), f)
    def f(A, x=x): A[x] = CENTER
    return ("lea\t($%x).l,a%d" % (CENTER, x), f)

def center(i):
    def f(A, i=i): A[i] = CENTER
    return ("lea\t($%x).l,a%d" % (CENTER, i), f)

def ok(A):
    return all(SAFE_LO <= A[i] < SAFE_HI for i in range(5)) and "bad" not in A

def gen_seq(n, A, no=None):
    """n instructions run once on model A (kept in range)"""
    seq = []
    for _ in range(n):
        r = rnd.random()
        if r < 0.42: ins = [data_op(no)]
        elif r < 0.66: ins = [addr_op()]
        elif r < 0.8: ins = [mem_op(no)]
        elif r < 0.9: ins = ld_op(no)
        elif r < 0.97:
            lab = "f%d" % rnd.getrandbits(40)
            ins = [("b%s.s\t%s" % (rnd.choice("eq ne cs cc mi pl hi ls ge lt gt le".split()), lab), nofn)]
            ins += [data_op(no) for _ in range(rnd.randint(1, 3))]
            ins += [(lab + ":", nofn)]
        else: ins = [("nop", nofn)]
        B = dict(A)
        for t, f in ins: f(B)
        if not ok(B):
            # pull the pointers back first
            ins = [center(i) for i in range(5) if not (SAFE_LO <= B[i] < SAFE_HI)] + ins
            if "bad" in B: continue
            B = dict(A)
            for t, f in ins: f(B)
            if not ok(B): continue
        for t, f in ins: f(A)
        seq += ins
    return seq

A = {i: CENTER for i in range(5)}
prog = []
def emit(t): prog.append(t if t.endswith(":") else "\t" + t)

blocks = int(sys.argv[3]) if len(sys.argv) > 3 else 40
prog.append("; generated by tools/dualrand/gen.py, seed %s" % sys.argv[1])
prog += ["\torg\t0", "\tdc.l\t$3400", "\tdc.l\tstart", "\trept\t254", "\tdc.l\tunexp",
         "\tendr", "\torg\t$400", "start:"]
emit("move.l\t#$80008000,d0")
emit("movec\td0,cacr")
for i in range(8):
    emit("move.l\t#$%08x,d%d" % (rnd.getrandbits(32), i))
for i in range(5):
    emit("lea\t($%x).l,a%d" % (CENTER, i))
emit("move.w\t#0,ccr")
for b in range(blocks):
    if rnd.random() < 0.4:
        cnt = rnd.randint(1, 6)
        for _ in range(50):
            B = dict(A)
            body = gen_seq(rnd.randint(3, 12), dict(A), no="d7")
            good = True
            for p in range(cnt + 1):
                for t, f in body:
                    f(B)
                    if not ok(B): good = False
            if good: break
        else:
            body, B = [], dict(A)
        emit("moveq\t#%d,d7" % cnt)
        emit("l%d:" % b)
        for t, f in body: emit(t)
        emit("dbra\td7,l%d" % b)
        A = B
    else:
        for t, f in gen_seq(rnd.randint(5, 25), A): emit(t)
    # checkpoint
    emit("move.w\tccr,(%s).l" % SAVE)
    emit("move.w\t(%s).l,(%s).l" % (SAVE, STAMP))
    for r in ["d%d" % i for i in range(8)] + ["a%d" % i for i in range(5)]:
        emit("move.l\t%s,(%s).l" % (r, STAMP))
        emit("move.w\t%s,(%s).l" % (r, STAMP))
    emit("move.w\t(%s).l,ccr" % SAVE)
emit("lea\t($%x).l,a0" % BUF_LO)
emit("moveq\t#0,d0")
emit("move.l\t#$%x,d1" % ((BUF_HI - BUF_LO) // 4 - 1))
emit("ck:")
emit("add.l\t(a0)+,d0")
emit("rol.l\t#1,d0")
emit("subq.l\t#1,d1")
emit("bpl.s\tck")
emit("move.l\td0,(%s).l" % STAMP)
emit("move.w\td0,(%s).l" % STAMP)
emit("move.w\t#$600D,($F102).l")
emit("stop:")
emit("bra.s\tstop")
emit("unexp:")
emit("move.w\t#99,($F100).l")
emit("move.w\t#$BAD0,($F102).l")
emit("hang:")
emit("bra.s\thang")
open(sys.argv[2], "w").write("\n".join(prog) + "\n")
