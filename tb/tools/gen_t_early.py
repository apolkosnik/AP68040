#!/usr/bin/env python3
"""Generate tb/asm/t_early.s: AG's early results as index registers.

AG computes simple register operations (MOVE, ADD, SUB, AND, OR, EOR,
LSL/ASL #n) as well as EX, and hands the result to the index of a younger
uop instead of waiting for EX.  The test runs random chains of such
operations, mixed with operations AG cannot do early (MULU, a load, a
shift by a register, DIVU, EXT, SWAP, NEG, CLR), then masks one register
into a table index and loads through it.  The table holds its own offsets
(the long word at offset k is k), so the loaded value must equal the index
register as the program sees it: a stale or wrong forwarded value reads
another entry.  Between producer and load: 0 to 3 other instructions, a
taken or not-taken branch (some mispredicted, flushing early results in
flight), or a TRAP.  The run is checked once more under a level 2
interrupt 150 clocks after each one returns.  Deterministic (seeded)."""
import random
import sys

R = ["d1", "d2", "d3", "d4", "d5"]
SZ = ["b", "w", "l"]


def early_op(rnd):
    k = rnd.randrange(9)
    d = rnd.choice(R)
    s = rnd.choice(R)
    z = rnd.choice(SZ)
    if k == 0:
        return "moveq\t#%d,%s" % (rnd.randrange(-128, 128), d)
    if k == 1:
        return "move.%s\t%s,%s" % (z, s, d)
    if k == 2:
        return "%s.%s\t%s,%s" % (rnd.choice(["add", "sub"]), z, s, d)
    if k == 3:
        return "%s.%s\t%s,%s" % (rnd.choice(["and", "or", "eor"]), z, s, d)
    if k == 4:
        return "%s.%s\t#%d,%s" % (rnd.choice(["addq", "subq"]), z, rnd.randrange(1, 9), d)
    if k == 5:
        return "%s.%s\t#%d,%s" % (rnd.choice(["lsl", "asl"]), z, rnd.randrange(1, 9), d)
    if k == 6:
        imm = rnd.randrange(1 << 16) if z != "b" else rnd.randrange(256)
        if z == "l":
            imm = rnd.randrange(1 << 32)
        return "%s.%s\t#$%X,%s" % (rnd.choice(["addi", "subi", "andi", "ori", "eori"]), z, imm, d)
    if k == 7:
        return "eor.%s\t%s,%s" % (z, s, d)
    return "add.l\t%s,%s" % (s, d)


def late_op(rnd):
    k = rnd.randrange(8)
    d = rnd.choice(R)
    s = rnd.choice(R)
    if k == 0:
        return "mulu.w\t%s,%s" % (s, d)
    if k == 1:
        return "move.l\t(a1),%s" % d          # a load (a1 -> a varying scratch long)
    if k == 2:
        return "lsl.l\t%s,%s" % (s, d)        # register count: not early
    if k == 3:
        return "swap\t%s" % d
    if k == 4:
        return "ext.l\t%s" % d
    if k == 5:
        return "neg.w\t%s" % d
    if k == 6:
        return "clr.b\t%s" % d
    return "move.l\t%s,(a1)" % s             # a store the next load reads


def block(rnd, n, w):
    # a chain of operations
    for _ in range(rnd.randrange(1, 5)):
        w("\t" + (late_op(rnd) if rnd.random() < 0.25 else early_op(rnd)))
    x = rnd.choice(R)
    form = rnd.randrange(4)
    if form == 0:      # long index
        w("\tandi.l\t#$3FC,%s" % x)
        gap(rnd, x, w)
        w("\tmove.l\t(a0,%s.l),d6" % x)
        w("\tcmp.l\t%s,d6" % x)
    elif form == 1:    # word index: the low word (upper bits whatever they are)
        w("\tandi.w\t#$3FC,%s" % x)
        gap(rnd, x, w)
        w("\tmove.l\t(a0,%s.w),d6" % x)
        w("\tmoveq\t#0,d7")
        w("\tmove.w\t%s,d7" % x)
        w("\tcmp.l\td7,d6")
    elif form == 2:    # scaled by 4
        w("\tandi.l\t#$FF,%s" % x)
        gap(rnd, x, w)
        w("\tmove.l\t(a0,%s.l*4),d6" % x)
        w("\tmove.l\t%s,d7" % x)
        w("\tlsl.l\t#2,d7")
        w("\tcmp.l\td7,d6")
    else:              # an index computed by a shift after the mask
        w("\tandi.l\t#$FF,%s" % x)
        w("\tlsl.l\t#2,%s" % x)
        gap(rnd, x, w)
        w("\tmove.l\t(4,a0,%s.l),d6" % x)
        w("\tmove.l\t%s,d7" % x)
        w("\taddq.l\t#4,d7")
        w("\tcmp.l\td7,d6")
    w("\tbeq.s\t.k%d" % n)
    w("\tmove.w\t#%d,d7" % (n + 1))
    w("\ttrap\t#2")
    w(".k%d:" % n)
    w("\taddq.l\t#4,a1")                     # the scratch long moves on
    w("\tcmpa.l\ta2,a1")
    w("\tbne.s\t.m%d" % n)
    w("\tmove.l\ta3,a1")
    w(".m%d:" % n)


def gap(rnd, x, w):
    g = rnd.randrange(8)
    if g < 4:
        # 0-3 independent instructions (registers other than x)
        others = [r for r in R if r != x]
        for _ in range(g):
            w("\t%s" % early_op_on(rnd, rnd.choice(others)))
    elif g == 4:
        # a data-dependent forward branch over a write of x: x keeps its
        # value either way (the write is the same mask); taken half the time
        w("\tbtst\t#2,%s" % x)
        w("\tbeq.s\t*+8")
        w("\tandi.l\t#$FFFFFFFF,%s" % x)
    elif g == 5:
        w("\ttrap\t#0")
    elif g == 6:
        # a write of x by an op AG cannot do early, between mask and use
        w("\tswap\t%s" % x)
        w("\tswap\t%s" % x)
    # g == 7: nothing


def early_op_on(rnd, d):
    s = rnd.choice(R)
    z = rnd.choice(SZ)
    k = rnd.randrange(3)
    if k == 0:
        return "add.%s\t%s,%s" % (z, s, d)
    if k == 1:
        return "lsl.%s\t#%d,%s" % (z, rnd.randrange(1, 9), d)
    return "moveq\t#%d,%s" % (rnd.randrange(-128, 128), d)


def main(path, nblk=900):
    rnd = random.Random(0x40E)
    o = []
    w = o.append
    for l in __doc__.split("\n"):
        w(";" + (" " + l if l else ""))
    w(";")
    w("; generated by tb/tools/gen_t_early.py")
    w("")
    w("FAILREG\tequ\t$F100")
    w("DONEREG\tequ\t$F102")
    w("IPLREG\tequ\t$F110")
    w("IPLDLY\tequ\t$F148\t\t; level 2 after this many clocks")
    w("TBL\tequ\t$40000\t\t; 1 KB: the long word at offset k holds k")
    w("SCR\tequ\t$41000\t\t; scratch long words the chains load and store")
    w("ntrap\tequ\t$E100")
    w("npass\tequ\t$E104")
    w("nirq\tequ\t$E108")
    w("rearm\tequ\t$E10C\t\t; the handler re-arms the delay")
    w("")
    w("\torg\t0")
    w("\tdc.l\t$E000\t\t; above the code")
    w("\tdc.l\tstart")
    w("\trept\t24\n\tdc.l\tunexp\n\tendr")
    w("\tdc.l\th_irq2\t\t; 26 level 2 autovector")
    w("\trept\t5\n\tdc.l\tunexp\n\tendr")
    w("\tdc.l\th_trap0\t\t; 32")
    w("\tdc.l\th_trap1\t\t; 33: the pass is done")
    w("\tdc.l\th_trap2\t\t; 34: failure, d7 the block")
    w("\trept\t221\n\tdc.l\tunexp\n\tendr")
    w("")
    w("\torg\t$400")
    w("start:")
    w("\tmove.l\t#$0000A040,d0\t; DTT1: supervisor data noncachable")
    w("\tmovec\td0,dtt1")
    w("\tmove.l\t#$00008020,d0\t; DTT0: user data copyback")
    w("\tmovec\td0,dtt0")
    w("\tmove.l\t#$80008000,d0")
    w("\tmovec\td0,cacr")
    w("\tlea\t(TBL).l,a0\t\t; the table")
    w("\tmoveq\t#0,d0")
    w(".f:\tmove.l\td0,(a0,d0.l)")
    w("\taddq.l\t#4,d0")
    w("\tcmp.l\t#$404,d0")
    w("\tbne.s\t.f")
    w("\tclr.l\t(ntrap).l")
    w("\tclr.l\t(npass).l")
    w("\tclr.l\t(nirq).l")
    w("\tclr.l\t(rearm).l")
    w("pass:\tlea\t(TBL).l,a0")
    w("\tlea\t(SCR).l,a1")
    w("\tlea\t(SCR+64).l,a2")
    w("\tlea\t(SCR).l,a3")
    w("\tlea\t($30000).l,a4")
    w("\tmove.l\ta4,usp")
    w("\tmove.l\t#$12345678,d1")
    w("\tmove.l\t#$9ABCDEF0,d2")
    w("\tmoveq\t#3,d3")
    w("\tmove.l\t#$00FF00FF,d4")
    w("\tmoveq\t#-1,d5")
    w("\tcmp.l\t#1,(npass).l\t; second pass: interrupts every 37 clocks")
    w("\tbne.s\t.u")
    w("\tmove.l\t#1,(rearm).l")
    w("\tmove.w\t#150,(IPLDLY).l")
    w(".u:\tmove.w\t#$0000,sr\t\t; user mode, interrupts on")
    for n in range(nblk):
        block(rnd, n, w)
    w("\ttrap\t#1")
    w("")
    w("h_trap1:")
    w("\tclr.l\t(rearm).l")
    w("\tclr.w\t(IPLDLY).l")
    w("\tclr.w\t(IPLREG).l")
    w("\taddq.l\t#1,(npass).l")
    w("\tcmp.l\t#2,(npass).l")
    w("\tbeq.s\t.d")
    w("\tjmp\tpass")
    w(".d:")
    w("\ttst.l\t(nirq).l\t\t; the second pass was interrupted")
    w("\tbne.s\t.ok")
    w("\tmove.w\t#9001,d7")
    w("\tbra.s\tfail_all")
    w(".ok:\tmove.w\t#$600D,(DONEREG).l")
    w("\tbra.s\t*")
    w("")
    w("h_trap2:")
    w("fail_all:")
    w("\tmove.w\t#$2700,sr")
    w("\tmove.w\td7,(FAILREG).l")
    w("\tmove.w\t#$BAD0,(DONEREG).l")
    w("\tbra.s\t*")
    w("")
    w("h_trap0:")
    w("\taddq.l\t#1,(ntrap).l")
    w("\trte")
    w("")
    w("h_irq2:")
    w("\tclr.w\t(IPLREG).l")
    w("\taddq.l\t#1,(nirq).l")
    w("\ttst.l\t(rearm).l")
    w("\tbeq.s\t.r")
    w("\tmove.w\t#150,(IPLDLY).l")
    w(".r:\trte")
    w("")
    w("unexp:")
    w("\tmove.w\t#9999,d7")
    w("\tbra.s\tfail_all")
    open(path, "w").write("\n".join(o) + "\n")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "tb/asm/t_early.s")
