#!/usr/bin/env python3
"""Materialize one WinUAE cputest v20 data slice for real-RTL replay.

The native runner is the specification for the delta codec and round state
machine.  This generator deliberately works one NNNN.dat slice at a time:
the complete 68040 corpus contains more than 31 million execution rounds,
so a single expanded job would be both enormous and impossible to resume.

APR2 is a private, big-endian streaming format consumed by tb_dat_replay.v.
It contains complete integer/FPU input and oracle state, interrupt level,
memory setup/restore actions, expected writes, exception-frame bytes/masks,
and the v20 trace/odd-vector metadata needed by the testbench.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
from dataclasses import dataclass
from typing import BinaryIO, Iterable, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dat_parser import (  # noqa: E402
    Header, DataFile, Stream, new_regs,
    CT_END, CT_END_INIT, CT_END_FINISH, CT_END_SKIP, CT_OVERRIDE_REG,
    CT_SR, CT_PC, CT_CYCLES, CT_FPIAR, CT_FPSR, CT_FPCR,
    CT_SRCADDR, CT_DSTADDR, CT_ENDPC, CT_BRANCHTARGET,
    CT_MEMWRITE, CT_MEMWRITES, SIZE_FPU, PC_BYTES, CT_EMPTY,
)

# cputest's "NOP" is MOVEA.L A0,A0: a real 68040 NOP is a T0 trace point.
NOP = 0x2048
ILLG = 0x4AFC
OPCODE_AREA = 48

JOB_MAGIC = b"APR2"
JOB_VERSION = 3
ROUND_MAGIC = 0x524E4432  # RND2

FLAG_FPU = 1 << 0
FLAG_IGNORE_EXCEPTION = 1 << 1       # v20 exception marker 1
FLAG_TRACE_STACKED = 1 << 2          # trace at primary handler entry
FLAG_TRACE_STANDALONE = 1 << 3       # trace before primary exception
FLAG_ODD_VECTOR = 1 << 4             # header exception_vectors != 0
FLAG_CHECK_FPIAR = 1 << 5            # result stream explicitly names FPIAR


def clone_regs(r):
    n = dict(r)
    n["regs"] = list(r["regs"])
    n["fpu"] = list(r["fpu"])
    n["mem"] = list(r.get("mem", []))
    n["opbytes"] = list(r.get("opbytes", []))
    return n


def resolve_addr(hdr: Header, base: str, addr: int) -> int:
    bases = {
        "low": 0,
        "abs": 0,
        "opcode_rel": hdr.opcode_memory_addr,
        "high": 0xFFFF8000,
    }
    return (bases[base] + addr) & 0xFFFFFFFF


def value_bytes(value: int, size: int) -> bytes:
    width = (1, 2, 4)[size]
    return int(value & ((1 << (8 * width)) - 1)).to_bytes(width, "big")


@dataclass
class Patch:
    addr: int
    data: bytes


@dataclass
class MemCheck:
    addr: int
    size: int
    expected: int
    restore: int


@dataclass
class ExceptionSpec:
    frame: bytes = b""
    mask: bytes = b""
    trace_mode: int = 0
    trace_sr: int = 0
    trace_pc: int = 0
    group2: int = 0


class Payload:
    """Small cursor for the relative fields inside an exception payload."""

    def __init__(self, data: bytes):
        self.data = data
        self.p = 0

    def u8(self):
        if self.p >= len(self.data):
            raise ValueError("short exception payload")
        v = self.data[self.p]
        self.p += 1
        return v

    def u16(self):
        return (self.u8() << 8) | self.u8()

    def rel_ordered(self, old):
        if self.data[self.p] == CT_EMPTY:
            self.p += 1
            return old
        tag = self.u8()
        sz = (tag >> 5) & 3
        if sz == 3:
            d = self.u8()
            if d & 0x80:
                d -= 0x100
            return (old + d) & 0xFFFFFFFF
        if sz == 0:
            d = self.u16()
            if d & 0x8000:
                d -= 0x10000
            return (old + d) & 0xFFFFFFFF
        if sz == 1:
            v = self.u16()
            if v & 0x8000:
                v -= 0x10000
            return v & 0xFFFFFFFF
        return ((self.u16() << 16) | self.u16()) & 0xFFFFFFFF


def decode_exception(hdr: Header, exc: int, payload: Optional[bytes],
                     expected_sr: int, expected_pc: int,
                     previous: ExceptionSpec) -> ExceptionSpec:
    """Decode validate_exception() data for an MC68040 frame (v20 and v24).

    The one format change between the two: v24 encodes the STACKED PC as a
    rel_ordered oracle value ahead of the frame word, where v20 rebuilt
    those four bytes from the processor's own captured PC and therefore
    never checked them.  Under v24 they are real corpus data, so they are
    decoded and compared.  Verified against WinUAE cputest/main.c's
    validate_exception, not inferred from the data.
    """
    if payload is None:              # length $ff: vector/count only
        # validate_exception() resets last_exception_len at entry.  The $ff
        # marker therefore skips trace/frame decoding and returns with a zero
        # comparison length; inheriting the prior record invents an oracle.
        return ExceptionSpec()
    if not payload:                  # length zero: vector only
        return ExceptionSpec()

    q = Payload(payload)
    extra = q.u8()
    group2 = 0
    if extra & 0x40:
        group2 = q.u8()
        extra &= ~0x40

    trace_mode = 0
    trace_sr = trace_pc = 0
    if (extra & 0x3F) == 9:
        if extra & 0x80:
            trace_mode = 2
            trace_sr = q.u16()
            trace_pc = q.rel_ordered(hdr.opcode_memory_addr)
        else:
            trace_mode = 1
    elif extra:
        raise ValueError("unsupported v20 exception-extra byte %02x" % extra)

    if exc == 1:
        if q.p != len(payload):
            raise ValueError("exception-1 payload has %d trailing bytes" %
                             (len(payload) - q.p))
        return ExceptionSpec(trace_mode=trace_mode, trace_sr=trace_sr,
                             trace_pc=trace_pc, group2=group2)

    if hdr.data_version >= 24:
        stacked_pc = q.rel_ordered(hdr.opcode_memory_addr)
    else:
        stacked_pc = expected_pc & 0xFFFFFFFF
    frame_word = q.u16()
    fmt = frame_word >> 12
    frame = bytearray(struct.pack(">HIH", expected_sr & 0xFFFF,
                                  stacked_pc, frame_word))
    mask = bytearray(b"\xff" * 8)
    if hdr.data_version < 24:
        # v20 validate_exception() constructs bytes 2..5 from test_regs.pc --
        # the PC already captured from the processor -- rather than decoding
        # an oracle value from the payload.  Those four bytes are therefore
        # not corpus-checked by the native runner; preserve that contract.
        mask[2:6] = b"\x00" * 4

    if fmt == 0:
        pass
    elif fmt in (2, 3):
        ea = q.rel_ordered(hdr.opcode_memory_addr)
        frame += struct.pack(">I", ea)
        mask += b"\xff" * 4
        if fmt == 3 or exc in (11, 55):
            fpeaset = q.u8() & 1
            if not fpeaset:
                mask[8:12] = b"\x00" * 4
    elif fmt == 4:
        ea = q.rel_ordered(hdr.opcode_memory_addr)
        oldpc = q.rel_ordered(hdr.opcode_memory_addr)
        frame += struct.pack(">II", ea, oldpc)
        mask += b"\xff" * 8
        if not (q.u8() & 1):
            mask[8:12] = b"\x00" * 4
    elif fmt in (0xA, 0xB):
        # v20 accepts these as an eight-byte frame on 010+; retain the exact
        # header.  They are included for structural completeness even though
        # a 68040 corpus should not normally generate them.
        pass
    else:
        raise ValueError("unsupported 68040 exception frame %x" % fmt)

    if q.p != len(payload):
        raise ValueError("exception %d payload length mismatch: %d/%d" %
                         (exc, q.p, len(payload)))
    return ExceptionSpec(bytes(frame), bytes(mask), trace_mode,
                         trace_sr, trace_pc, group2)


def read_override(st: Stream, test, cur):
    while st.peek() == CT_OVERRIDE_REG:
        st.u8()
        tag = st.u8()
        reg = tag & 31
        size = tag & 0x60
        if reg == CT_SR:
            if size == 0:
                val = st.u8()
                test["sr"] = (test["sr"] & 0xFF00) | val
            else:
                test["sr"] = struct.unpack(">H", st.take(2))[0]
            cur["sr"] = test["sr"]
        elif reg in (CT_FPSR, CT_FPCR, CT_FPIAR):
            val = struct.unpack(">I", st.take(4))[0]
            key = {CT_FPSR: "fpsr", CT_FPCR: "fpcr", CT_FPIAR: "fpiar"}[reg]
            test[key] = cur[key] = val
        elif reg < 16:
            if size == 0x60:
                raw = st.take(12)
                test["fpu"][reg] = (struct.unpack(">I", raw[:4])[0] >> 16,
                                     struct.unpack(">I", raw[4:8])[0],
                                     struct.unpack(">I", raw[8:12])[0])
            else:
                # This apparently surprising assignment is exact v20
                # process_test behavior: the current round was copied before
                # the override, so a GPR override takes effect next round.
                cur["regs"][reg] = struct.unpack(">I", st.take(4))[0]
        else:
            raise ValueError("unknown override register %02x @%x" % (tag, st.p))


def irq_level(bit_number: int) -> int:
    """Amiga INTREQ bit to IPL level, matching the corpus' hardware runner."""
    # The hardware corpus serializes custom-chip INTREQ bit numbers.  Paula's
    # level groups differ from WinUAE's host-side synthetic interrupt table:
    # bits 0..2/3/4..6/7..10/11..12/13..14 map to levels 1..6.
    if bit_number <= 2:
        return 1
    if bit_number == 3:
        return 2
    if bit_number <= 6:
        return 3
    if bit_number <= 10:
        return 4
    if bit_number <= 12:
        return 5
    if bit_number <= 14:
        return 6
    return 0


def write_fp(f: BinaryIO, values):
    for exp, hi, lo in values:
        f.write(struct.pack(">III", exp & 0xFFFF, hi & 0xFFFFFFFF,
                            lo & 0xFFFFFFFF))


def write_patches(f: BinaryIO, patches: Iterable[Patch]):
    patches = list(patches)
    if len(patches) > 0xFFFF:
        raise ValueError("too many patches in one round")
    f.write(struct.pack(">H", len(patches)))
    for p in patches:
        if len(p.data) > 0xFFFF:
            raise ValueError("patch too long")
        f.write(struct.pack(">IH", p.addr & 0xFFFFFFFF, len(p.data)))
        f.write(p.data)


def write_round(f: BinaryIO, testno: int, roundno: int, flags: int,
                initial, ssp: int, msp: int, level: int,
                pre: list[Patch], toggles: list[tuple[int, int]],
                expected, sr_mask: int, actual_exc: int, expected_pc: int,
                exspec: ExceptionSpec, trace_sr_mask: int,
                memchecks: list[MemCheck],
                post: list[Patch], cleanup: list[Patch]):
    if f is None:
        return
    f.write(struct.pack(">III", ROUND_MAGIC, testno, roundno))
    f.write(struct.pack(">I", flags))
    f.write(struct.pack(">16I", *[v & 0xFFFFFFFF for v in initial["regs"]]))
    f.write(struct.pack(">IIII", initial["sr"] & 0xFFFF,
                        initial["pc"] & 0xFFFFFFFF, ssp, msp))
    write_fp(f, initial["fpu"])
    f.write(struct.pack(">III", initial["fpcr"] & 0xFFFFFFFF,
                        initial["fpsr"] & 0xFFFFFFFF,
                        initial["fpiar"] & 0xFFFFFFFF))
    f.write(struct.pack(">B", level & 7))
    write_patches(f, pre)
    f.write(struct.pack(">H", len(toggles)))
    for addr, kind in toggles:
        f.write(struct.pack(">IB", addr & 0xFFFFFFFF, kind))

    f.write(struct.pack(">16I", *[v & 0xFFFFFFFF for v in expected["regs"]]))
    f.write(struct.pack(">II", expected["sr"] & 0xFFFF, sr_mask & 0xFFFF))
    write_fp(f, expected["fpu"])
    f.write(struct.pack(">III", expected["fpcr"] & 0xFFFFFFFF,
                        expected["fpsr"] & 0xFFFFFFFF,
                        expected["fpiar"] & 0xFFFFFFFF))
    f.write(struct.pack(">BI", actual_exc & 0xFF, expected_pc & 0xFFFFFFFF))
    f.write(struct.pack(">BBHHI", exspec.trace_mode, exspec.group2,
                        exspec.trace_sr & 0xFFFF, trace_sr_mask & 0xFFFF,
                        exspec.trace_pc & 0xFFFFFFFF))
    f.write(struct.pack(">H", len(exspec.frame)))
    f.write(exspec.frame)
    f.write(exspec.mask)
    f.write(struct.pack(">H", len(memchecks)))
    for m in memchecks:
        f.write(struct.pack(">IBII", m.addr, m.size, m.expected, m.restore))
    write_patches(f, post)
    write_patches(f, cleanup)


def generate(header_path: str, dat_path: str, out_path: str,
             max_rounds: Optional[int] = None,
             audit_only: bool = False) -> dict:
    hdr = Header(open(header_path, "rb").read())
    df = DataFile.load(dat_path, header=hdr)
    st = Stream(df.body, hdr)
    cur = new_regs()
    cur["sr"] = hdr.interrupt_mask << 8
    startpc = endpc = hdr.opcode_memory_addr

    # Last decoded exception image is global to process_test(), including
    # across tests in this one data slice.
    last_exception = ExceptionSpec()
    # WinUAE resets this once per slice, then updates it when a CT_SR result
    # is consumed.  Exception validation follows those result fields, so a
    # standalone trace uses the current persistent value.
    ccr_ignore_mask = 0xFFFF
    count = execution_rounds = tests = skipped = 0

    with open(out_path, "wb+") as out:
        out.write(JOB_MAGIC)
        out.write(struct.pack(">IIIII", JOB_VERSION, 0,
                              hdr.test_memory_addr, hdr.test_memory_size,
                              hdr.exception_vectors))

        while True:
            cur["endpc"] = endpc
            cur["pc"] = startpc
            # Setup patches are applied in STREAM ORDER, not grouped by kind.
            # They overlap: v24 setups fill the opcode area with $F2 bytes
            # through a CT_PC_BYTES record and then write the instruction
            # words over it with CT_MEMWRITE records.  Emitting all memory
            # writes before all opcode bytes lets the fill erase the
            # instruction, and the CPU then executes the fill pattern.
            setup_patches = []
            setup_cleanup = []
            while st.peek() not in (CT_END_INIT, CT_END_FINISH):
                cur["mem"] = []
                cur["opbytes"] = []
                st.restore_data(cur)
                for base, addr, old, new, size in cur["mem"]:
                    a = resolve_addr(hdr, base, addr)
                    setup_patches.append(Patch(a, value_bytes(new, size)))
                    # CT_MEMWRITE setup records enter WinUAE's access
                    # history and are restored at the end of this test.
                    setup_cleanup.append(Patch(a, value_bytes(old, size)))
                for off, data in cur["opbytes"]:
                    setup_patches.append(Patch(hdr.opcode_memory_addr + off,
                                               data))
                cur["mem"] = []
                cur["opbytes"] = []
            if st.peek() == CT_END_FINISH:
                break
            st.u8()
            testno = tests
            tests += 1
            startpc = cur["pc"]
            endpc = cur["endpc"]
            fpumode = bool(hdr.fpu_model)
            # main.c: interrupt tests at level 2+ keep a fixed opcode tail
            doopcodeswap = not (hdr.interrupttest >= 2)
            last = clone_regs(cur)

            extraccr = 0
            roundno = 0
            # Mirror cputest exactly (main.c): the tail starts as
            # NOP:ILLG, is swapped BEFORE each round only when opcode
            # swapping is enabled, and the PC adjustment is DERIVED from
            # the current tail rather than toggled alongside it.  The old
            # code started from the opposite value and toggled
            # unconditionally, which put every round's opcode image one
            # round out of phase.  v20 could not see it -- it masked the
            # stacked PC out of the frame comparison -- but v24 encodes
            # that PC as a real oracle, and it shows up immediately.
            # The 2020-era v20 generator started this tail as ILLG:NOP;
            # current cputest (main.c originalopcodeend) starts it as
            # NOP:ILLG.  Measured, not assumed: using the v24 phase on v20
            # data breaks nine slices (FBcc, FDBcc, FMOVEM.X, FSMUL.W) and
            # using the v20 phase on v24 data puts every round's opcode
            # image one round out of step.  The derived opcodeextra rule
            # below is identical for both.
            opcodeend = ((NOP << 16) | ILLG) if hdr.data_version >= 24 \
                else ((ILLG << 16) | NOP)
            opcodeextra = 0
            first_round = True
            deferred_toggles = []

            while True:
                ccrmode = st.u8()
                maxccr = ccrmode & 0x3F
                sr_bits = ((0x2000 if extraccr & 1 else 0) |
                           (0x4000 if extraccr & 2 else 0) |
                           (0x8000 if extraccr & 4 else 0) |
                           (0x1000 if extraccr & 8 else 0))

                for ccr in range(maxccr):
                    if doopcodeswap:
                        opcodeend = ((opcodeend >> 16) |
                                     (opcodeend << 16)) & 0xFFFFFFFF
                    opcodeextra = 2 if (opcodeend >> 16) == NOP else 0

                    level = irq_level(st.u8()) if hdr.interrupttest else 0
                    test = clone_regs(cur)
                    test["pc"] = startpc
                    # The native cputest runner resets FPIAR to all ones before
                    # every CCR/round (main.c: cur_regs.fpiar = 0xffffffff)
                    # and lets only instructions with an architectural FPIAR
                    # side effect replace it with their instruction address.
                    # Seeding it with startpc made FBcc/FScc/FMOVEM preservation
                    # checks look like RTL failures and hid missing updates.
                    test["fpiar"] = 0xFFFFFFFF
                    test["sr"] = ((ccr & 0xFF) if maxccr >= 32
                                  else (0x1F if ccr & 1 else 0))
                    test["sr"] |= sr_bits | (hdr.interrupt_mask << 8)
                    if fpumode:
                        test["fpcr"] = 0
                        test["fpsr"] = 0
                        if maxccr < 16:
                            test["fpsr"] = (15 if ccr & 1 else 0) << 24
                            test["fpcr"] = (15 if ccr & 1 else 0) << 4
                        elif ccrmode & 0x40:
                            test["fpsr"] = (ccr & 15) << 24
                        else:
                            test["fpcr"] = (ccr & 15) << 4
                    read_override(st, test, cur)

                    cur["sr"] = test["sr"]
                    cur["fpsr"] = test["fpsr"]
                    cur["fpcr"] = test["fpcr"]
                    if not sr_bits and ccr == 0:
                        last["sr"] = test["sr"]
                        last["fpsr"] = test["fpsr"]
                        last["fpcr"] = test["fpcr"]

                    valid = st.peek() != CT_END_SKIP
                    round_toggles = []
                    bt = cur.get("branchtarget", 0xFFFFFFFF)
                    if bt != 0xFFFFFFFF and not (bt & 1):
                        if cur.get("branchtarget_mode") == 1:
                            round_toggles.append((bt, 1))
                        elif cur.get("branchtarget_mode") == 2:
                            round_toggles.append((bt, 2))
                    if not valid:
                        st.u8()
                        skipped += 1
                        deferred_toggles.extend(round_toggles)
                    else:
                        expected = clone_regs(last)
                        sr_mask = ccr_ignore_mask
                        explicit_fpiar = False
                        memchecks = []
                        post = []
                        while True:
                            tag = st.peek()
                            if tag & CT_END:
                                marker = st.u8()
                                exp_exc = marker & 63
                                payload = b""
                                if exp_exc:
                                    n = st.u8()
                                    if n == 0xFF:
                                        payload = None
                                    else:
                                        payload = bytes(st.take(n))
                                break
                            mode = tag & 31
                            if mode < 16 and ((tag >> 5) & 3) == SIZE_FPU:
                                expected["fpu"][mode] = st.restore_fpvalue(expected["fpu"][mode])
                            elif mode < 16:
                                expected["regs"][mode], _ = st.restore_value(expected["regs"][mode])
                            elif mode == CT_SR:
                                expected["sr"], _ = st.restore_value(expected["sr"])
                                sr_mask = (~(expected["sr"] >> 16)) & 0xFFFF
                                ccr_ignore_mask = sr_mask
                            elif mode == CT_PC:
                                expected["pc"] = st.restore_rel(expected["pc"])
                            elif mode in (CT_FPCR, CT_FPSR, CT_FPIAR, CT_CYCLES,
                                          CT_SRCADDR, CT_DSTADDR, CT_ENDPC):
                                key = {CT_FPCR: "fpcr", CT_FPSR: "fpsr",
                                       CT_FPIAR: "fpiar", CT_CYCLES: "cycles",
                                       CT_SRCADDR: "srcaddr", CT_DSTADDR: "dstaddr",
                                       CT_ENDPC: "endpc"}[mode]
                                expected[key], _ = st.restore_value(expected.get(key, 0))
                                if mode == CT_FPIAR:
                                    explicit_fpiar = True
                            elif mode == CT_MEMWRITE:
                                base, addr, old, new, size = st.restore_memory()
                                memchecks.append(MemCheck(resolve_addr(hdr, base, addr),
                                                          size, new, old))
                            elif mode == CT_MEMWRITES:
                                if ((tag >> 5) & 3) == PC_BYTES:
                                    st.u8()
                                    off, data = st.restore_bytes()
                                    post.append(Patch(hdr.opcode_memory_addr + off, data))
                                else:
                                    base, addr, old, new, size = st.restore_memory()
                                    post.append(Patch(resolve_addr(hdr, base, addr),
                                                      value_bytes(new, size)))
                            elif mode == CT_BRANCHTARGET:
                                expected["branchtarget"], _ = st.restore_value(expected["branchtarget"])
                                expected["branchtarget_mode"] = st.u8()
                            else:
                                raise ValueError("expected mode %02x @%x" % (tag, st.p))

                        last = clone_regs(expected)
                        actual_exc = 4 if exp_exc == 0 else exp_exc
                        expected_pc = (expected["pc"] + opcodeextra) & 0xFFFFFFFF \
                            if exp_exc == 0 else expected["pc"] & 0xFFFFFFFF
                        exspec = decode_exception(hdr, exp_exc, payload,
                                                  expected["sr"], expected_pc,
                                                  last_exception) if exp_exc else ExceptionSpec()
                        if exp_exc and payload is not None and payload:
                            last_exception = exspec

                        flags = FLAG_FPU if fpumode else 0
                        if exp_exc == 1:
                            flags |= FLAG_IGNORE_EXCEPTION
                        if exspec.trace_mode == 1:
                            flags |= FLAG_TRACE_STACKED
                        elif exspec.trace_mode == 2:
                            flags |= FLAG_TRACE_STANDALONE
                        if hdr.exception_vectors:
                            flags |= FLAG_ODD_VECTOR
                        if explicit_fpiar:
                            flags |= FLAG_CHECK_FPIAR

                        trace_sr_mask = sr_mask

                        pre = []
                        if first_round:
                            pre.extend(setup_patches)
                        pre.append(Patch(endpc, struct.pack(">I", opcodeend)))
                        # A CT_MEMWRITE result carries both the expected value
                        # and the value restored by WinUAE after validation.
                        # The latter is therefore the authoritative operand
                        # image at instruction entry.  Materialize it here so
                        # every APR record is independently replayable and a
                        # first use cannot inherit the static/random tmem byte
                        # or residue from an earlier test.
                        pre.extend(Patch(m.addr, value_bytes(m.restore, m.size))
                                   for m in memchecks)
                        toggles = deferred_toggles + round_toggles
                        deferred_toggles = []

                        cleanup = []
                        pending = (testno, roundno, flags, test,
                                   hdr.super_stack_memory - 0x80,
                                   hdr.super_stack_memory, level, pre, toggles,
                                   expected, sr_mask, actual_exc, expected_pc,
                                   exspec, trace_sr_mask, memchecks, post, cleanup)
                        if max_rounds is None or count < max_rounds:
                            write_round(None if audit_only else out, *pending)
                            count += 1
                            execution_rounds += 1
                        first_round = False
                    roundno += 1

                if st.peek() == CT_END:
                    st.u8()
                    break
                extraccr = st.u8()

            # Preserve setup/toggle state even when every execution round was
            # skipped, then reproduce process_test()'s end-of-test
            # restoreahist().  CT_MEMWRITES opcode patches are not part of
            # that history and remain resident.  Reverse order is required
            # when temporary setup writes overlap.
            if (not audit_only and
                    (first_round or deferred_toggles or setup_cleanup) and
                    (max_rounds is None or count < max_rounds)):
                maintenance = clone_regs(cur)
                maintenance["pc"] = startpc
                write_round(out, testno, 0xFFFFFFFF,
                            FLAG_IGNORE_EXCEPTION, maintenance,
                            hdr.super_stack_memory - 0x80,
                            hdr.super_stack_memory, 0,
                            (setup_mem + setup_op) if first_round else [],
                            deferred_toggles, maintenance,
                            0xFFFF, 0xFF, 0, ExceptionSpec(), 0xFFFF,
                            [], [], list(reversed(setup_cleanup)))
                count += 1

            if max_rounds is not None and count >= max_rounds:
                break

        if not audit_only:
            out.seek(8)
            out.write(struct.pack(">I", count))

    return {
        "instruction": hdr.inst_name,
        "tests": tests,
        "rounds": execution_rounds,
        "records": count,
        "skipped": skipped,
        "slice_last": df.last,
        "instruction_size": df.instruction_size,
        "consumed": st.p,
        "stream_size": len(st.d),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("header")
    ap.add_argument("data")
    ap.add_argument("output")
    ap.add_argument("--limit", type=int, help="maximum emitted records")
    ap.add_argument("--audit", action="store_true",
                    help="decode every round but do not materialize records")
    args = ap.parse_args(argv)
    result = generate(args.header, args.data, args.output, args.limit,
                      audit_only=args.audit)
    print("{instruction}: {rounds} rounds, {records} records from {tests} tests -> {out}".format(
        out=args.output, **result))


if __name__ == "__main__":
    main()
