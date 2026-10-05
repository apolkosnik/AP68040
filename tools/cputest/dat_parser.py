#!/usr/bin/env python3
# WinUAE cputest .dat parser for the AP040 dat-replay harness.
#
# Format authority: WinUAE cputest/main.c (restore_value ~987, restore_rel
# ~1023, restore_rel_ordered ~1079, get_memory_addr ~1100, restore_memory
# ~1247, test_mnemo header parse ~3629) and cputest/cputest_defines.h.
# Source extracts preserved under the session scratchpad winuae_extracts/
# and re-derivable from /home/adam/WinUAE/cputest.
#
# Layout: <INSTR>/0000.dat is a header file (DATA_VERSION 20); NNNN.dat(.gz)
# have a 16-byte envelope followed by a delta-encoded test stream.  Values
# are big-endian.  Register/memory
# records are tagged bytes: low 5 bits = id (CT_DREG 0-7, CT_AREG 8-15,
# CT_SSP 16, CT_MSP 17, CT_SR 18, CT_PC 19, CT_FPIAR 20, CT_FPSR 21,
# CT_FPCR 22, CT_EDATA 23, CT_CYCLES 25, CT_ENDPC 26, CT_BRANCHTARGET 27,
# CT_SRCADDR 28, CT_DSTADDR 29, CT_MEMWRITE 30, CT_MEMWRITES 31), bits 6:5
# = size (byte/word/long, or FPU=3 meaning the id names an FP register with
# a 12-byte extended image), bit 7 = END-class markers.

import gzip
import os
import struct
import sys

DATA_VERSION = 20
SUPPORTED_VERSIONS = (20, 24)  # the downloaded 040 sets; header layout per main.c @f0cf9536~1

CT_SSP, CT_MSP, CT_SR, CT_PC = 16, 17, 18, 19
CT_FPIAR, CT_FPSR, CT_FPCR = 20, 21, 22
CT_CYCLES, CT_ENDPC, CT_BRANCHTARGET = 25, 26, 27
CT_SRCADDR, CT_DSTADDR, CT_MEMWRITE, CT_MEMWRITES = 28, 29, 30, 31

CT_END = 0x80
CT_END_FINISH = 0xFF
CT_END_INIT = 0x80 | 0x40
CT_END_SKIP = 0x80 | 0x40 | 0x01
CT_SKIP_REGS = 0x80 | 0x40 | 0x02
CT_EMPTY = CT_END_INIT
CT_OVERRIDE_REG = 0x80 | 0x40 | 0x10
CT_BRANCHED = 0x40

SIZE_BYTE, SIZE_WORD, SIZE_LONG, SIZE_FPU = 0, 1, 2, 3
REL_WORD, ABS_WORD, ABS_LONG, PC_BYTES = 0, 1, 2, 3
REL_BYTE = 3


class Header:
    # v20 layout: version, starttimeid, hmem/lmem, tmem addr/size,
    # opcode addr, lvl_mask, fpu_model, low start/end, high start/end,
    # safe start/end, ustack, sstack, vectors, 3 spare, name
    #
    # v24 is v20 with three longwords inserted after the lvl/mask word
    # (initial interrupt state + fpu_max_precision, then two reserved),
    # so everything from fpu_model on shifts up by three and the
    # instruction name moves from +80 to +92.  Field order verified
    # against WinUAE cputest/main.c's header reader, not inferred.
    def __init__(self, data):
        if len(data) < 96:
            raise ValueError("short instruction header (%d bytes)" % len(data))
        ver = struct.unpack(">I", data[0:4])[0]
        if ver not in SUPPORTED_VERSIONS:
            raise ValueError("bad DATA_VERSION %d" % ver)
        self.data_version = ver
        shift = 3 if ver >= 24 else 0
        name_off = 92 if ver >= 24 else 80
        f = struct.unpack(">21I", data[0:84])
        f = f[0:7] + f[7 + shift:]
        self.starttimeid = f[1]
        self.hmem_rom = struct.unpack(">h", data[8:10])[0]
        self.lmem_rom = struct.unpack(">h", data[10:12])[0]
        self.test_memory_addr = f[3]
        self.test_memory_size = f[4]
        self.opcode_memory_addr = f[5]
        v = f[6]
        self.cpu_lvl = (v >> 16) & 15
        self.interrupt_mask = (v >> 20) & 7
        self.addressing_mask = 0xFFFFFFFF if (v & 0x80000000) else 0x00FFFFFF
        self.interrupttest = (v >> 26) & 3
        self.sr_undefined_mask = v & 0xFFFF
        self.safe_memory_mode = (v >> 23) & 7
        self.fpu_model = f[7]
        self.test_low_memory_start = f[8]
        self.test_low_memory_end = f[9]
        self.test_high_memory_start = f[10]
        self.test_high_memory_end = f[11]
        self.safe_memory_start = f[12]
        self.safe_memory_end = f[13]
        self.user_stack_memory = f[14]
        self.super_stack_memory = f[15]
        self.exception_vectors = f[16]
        self.inst_name = data[name_off:name_off + 16].split(b"\0")[0].decode(
            "ascii", "replace")


class DataFile:
    """Validated NNNN.dat envelope and its process_test() payload.

    The envelope (version, starttimeid, flags, spare; 16 bytes; footer
    CT_END_FINISH) is byte-identical in v20 and v24 -- verified against
    WinUAE cputest/main.c -- so only the accepted version number differs.
    """

    HEADER_SIZE = 16

    def __init__(self, data, header=None, path=None):
        if len(data) < self.HEADER_SIZE + 2:
            raise ValueError("short data file%s (%d bytes)" %
                             ((" " + path) if path else "", len(data)))
        self.path = path
        self.version, self.starttimeid, self.flags, self.spare = \
            struct.unpack(">4I", data[:self.HEADER_SIZE])
        if self.version not in SUPPORTED_VERSIONS:
            raise ValueError("bad data-file version %d%s" %
                             (self.version, (" in " + path) if path else ""))
        if header is not None and self.version != header.data_version:
            raise ValueError("data v%d under header v%d%s" %
                             (self.version, header.data_version,
                              (" in " + path) if path else ""))
        if header is not None and self.starttimeid != header.starttimeid:
            raise ValueError("data/header starttime mismatch %08x != %08x%s" %
                             (self.starttimeid, header.starttimeid,
                              (" in " + path) if path else ""))
        if data[-2] != CT_END_FINISH:
            raise ValueError("bad data-file footer%s" %
                             ((" in " + path) if path else ""))
        self.instruction_size = self.flags & 3
        self.last = data[-1] == CT_END_FINISH
        # process_test() receives exactly this pointer.  The first of the
        # two footer bytes terminates its loop; the second is the slice-list
        # last flag and deliberately remains outside the parsed stream.
        self.body = data[self.HEADER_SIZE:]
        self.raw_size = len(data)

    @classmethod
    def load(cls, path, header=None):
        if path.endswith(".gz"):
            with gzip.open(path, "rb") as f:
                data = f.read()
        else:
            with open(path, "rb") as f:
                data = f.read()
        return cls(data, header=header, path=path)


class Stream:
    """Delta-decoded walk of one NNNN.dat test stream."""

    def __init__(self, data, header):
        self.d = data
        self.p = 0
        self.h = header

    def u8(self):
        if self.p >= len(self.d):
            raise ValueError("unexpected end of stream @%x" % self.p)
        v = self.d[self.p]
        self.p += 1
        return v

    def peek(self):
        if self.p >= len(self.d):
            raise ValueError("unexpected end of stream @%x" % self.p)
        return self.d[self.p]

    def take(self, n):
        if n < 0 or self.p + n > len(self.d):
            raise ValueError("short stream read @%x: need %d, have %d" %
                             (self.p, n, len(self.d) - self.p))
        v = self.d[self.p:self.p + n]
        self.p += n
        return v

    def restore_value(self, old):
        v = self.u8()
        sz = (v >> 5) & 3
        if sz == SIZE_BYTE:
            return (old & 0xFFFFFF00) | self.u8(), 0
        if sz == SIZE_WORD:
            hi, lo = self.u8(), self.u8()
            return (old & 0xFFFF0000) | (hi << 8) | lo, 1
        if sz == SIZE_LONG:
            b = self.take(4)
            return struct.unpack(">I", b)[0], 2
        raise ValueError("CT_SIZE_FPU in restore_value @%x" % (self.p - 1))

    def restore_rel(self, old):
        v = self.u8()
        sz = (v >> 5) & 3
        if sz == REL_BYTE:
            return (old + struct.unpack(">b", self.take(1))[0]) & 0xFFFFFFFF
        if sz == REL_WORD:
            return (old + struct.unpack(">h", self.take(2))[0]) & 0xFFFFFFFF
        if sz == ABS_WORD:
            return struct.unpack(">h", self.take(2))[0] & 0xFFFFFFFF
        if sz == ABS_LONG:
            return struct.unpack(">I", self.take(4))[0]

    def restore_rel_ordered(self, old):
        if self.peek() == CT_EMPTY:
            self.p += 1
            return old
        return self.restore_rel(old)

    def get_memory_addr(self):
        v = self.u8()
        sz = (v >> 5) & 3
        if sz == ABS_WORD:
            off = struct.unpack(">h", self.take(2))[0]
            base = "high" if off < 0 else "low"
            return base, off if off >= 0 else 32768 + off
        if sz == ABS_LONG:
            val = struct.unpack(">I", self.take(4))[0]
            return "abs", val
        if sz == REL_WORD:
            off = struct.unpack(">h", self.take(2))[0]
            return "opcode_rel", off
        raise ValueError("get_memory_addr size %d @%x" % (sz, self.p - 1))

    def restore_memory(self):
        base, addr = self.get_memory_addr()
        old, _ = self.restore_value(0)
        new, size = self.restore_value(0)
        return (base, addr, old, new, size)

    # ---- full record layer -------------------------------------------------

    def restore_fpvalue(self, old):
        """old = (exp16, m0, m1); delta encoding per main.c restore_fpvalue."""
        v = self.u8()
        if (v >> 5) & 3 != SIZE_FPU:
            raise ValueError("expected CT_SIZE_FPU @%x" % (self.p - 1))
        size = self.u8()
        if size == 0x00:
            return (0, 0, 0)
        if size == 0xFF:
            e = struct.unpack(">H", self.take(2))[0]
            m0, m1 = struct.unpack(">II", self.take(8))
            return (e, m0, m1)
        f = bytearray(struct.pack(">HII", old[0], old[1], old[2]))
        n_head = (size >> 4) & 15
        n_tail = size & 15
        for i in range(n_head):
            f[i] = self.u8()
        for i in range(n_tail):
            f[9 - i] = self.u8()
        e, m0, m1 = struct.unpack(">HII", bytes(f))
        return (e, m0, m1)

    def restore_bytes(self):
        """CT_MEMWRITES/CT_PC_BYTES: opcode bytes at opcode_memory+offset.

        The escape encoding differs between corpus versions (main.c
        restore_bytes): v20 keys it on the packed length field being 31 and
        follows with a length byte only, keeping the packed 3-bit offset.
        v24 keys it on the whole byte being $FF and follows with a FULL
        offset byte and then the length.  Getting this wrong shifts the
        entire opcode image by one byte, which shows up as the CPU
        fetching garbage rather than as a decode error.
        """
        if getattr(self.h, "data_version", 20) >= 24:
            if self.peek() == 0xFF:
                self.u8()
                off = self.u8()
                n = self.u8()
                if n == 0:
                    n = 256
            else:
                v = self.u8()
                off = v >> 5
                n = v & 31
                if n == 0:
                    n = 32
            return off, self.take(n)
        v = self.u8()
        off = v >> 5
        n = v & 31
        if n == 31:
            n = self.u8()
            if n == 0:
                n = 256
        elif n == 0:
            n = 32
        return off, self.take(n)

    def restore_data(self, regs):
        """One setup record into regs dict; returns record kind."""
        v = self.peek()
        if v & CT_END:
            raise ValueError("END bit in setup @%x" % self.p)
        mode = v & 31
        if mode == CT_SRCADDR:
            regs["srcaddr"], _ = self.restore_value(regs["srcaddr"])
        elif mode == CT_DSTADDR:
            regs["dstaddr"], _ = self.restore_value(regs["dstaddr"])
        elif mode == CT_ENDPC:
            regs["endpc"], _ = self.restore_value(regs["endpc"])
        elif mode == CT_PC:
            regs["pc"], _ = self.restore_value(regs["pc"])
        elif mode == CT_BRANCHTARGET:
            regs["branchtarget"], _ = self.restore_value(regs["branchtarget"])
            regs["branchtarget_mode"] = self.u8()
        elif mode < 16:
            if (v >> 5) & 3 == SIZE_FPU:
                regs["fpu"][mode] = self.restore_fpvalue(regs["fpu"][mode])
            else:
                regs["regs"][mode], _ = self.restore_value(regs["regs"][mode])
        elif mode == CT_SR:
            regs["sr"], _ = self.restore_value(regs["sr"])
        elif mode == CT_CYCLES:
            regs["cycles"], _ = self.restore_value(regs.get("cycles", 0))
        elif mode == CT_FPIAR:
            regs["fpiar"], _ = self.restore_value(regs["fpiar"])
        elif mode == CT_FPCR:
            regs["fpcr"], _ = self.restore_value(regs["fpcr"])
        elif mode == CT_FPSR:
            regs["fpsr"], _ = self.restore_value(regs["fpsr"])
        elif mode == CT_MEMWRITE:
            regs.setdefault("mem", []).append(self.restore_memory())
        elif mode == CT_MEMWRITES:
            sz = (v >> 5) & 3
            if sz == PC_BYTES:
                self.u8()
                off, data = self.restore_bytes()
                regs.setdefault("opbytes", []).append((off, bytes(data)))
            else:
                regs.setdefault("mem", []).append(self.restore_memory())
        else:
            raise ValueError("mode %02x @%x" % (v, self.p))


def new_regs():
    return {
        "regs": [0] * 16, "fpu": [(0, 0, 0)] * 8,
        "sr": 0, "pc": 0, "endpc": 0, "fpiar": 0, "fpsr": 0, "fpcr": 0,
        "srcaddr": 0xFFFFFFFF, "dstaddr": 0xFFFFFFFF,
        "branchtarget": 0xFFFFFFFF, "branchtarget_mode": 0,
    }


def data_body(data, hdr=None):
    """Return (body, DataFile-or-None).

    Accepting an already stripped body keeps this module useful to callers
    that obtained process_test() bytes by another route.  A v20 prefix is
    always treated as an envelope and validated, never as stream data.
    """
    if isinstance(data, DataFile):
        return data.body, data
    if len(data) >= DataFile.HEADER_SIZE and data[:4] == struct.pack(">I", DATA_VERSION):
        df = DataFile(data, header=hdr)
        return df.body, df
    return data, None


def walk_stream(hdr, data, interrupttest=False, verbose=False):
    """Structural walk of one NNNN.dat: yields per-test summaries and
    validates that the stream never desyncs (hard error otherwise)."""
    body, df = data_body(data, hdr)
    st = Stream(body, hdr)
    cur = new_regs()
    cur["sr"] = (hdr.interrupt_mask << 8)
    tests = 0
    rounds = 0
    excs = {}
    while True:
        cur["mem"] = []
        cur["opbytes"] = []
        while st.peek() not in (CT_END_INIT, CT_END_FINISH):
            st.restore_data(cur)
        if st.peek() == CT_END_FINISH:
            break
        st.u8()  # CT_END_INIT
        tests += 1
        # ccr/extraccr rounds
        while True:
            ccrmode = st.u8()
            maxccr = ccrmode & 0x3F
            for ccr in range(maxccr):
                if interrupttest:
                    st.u8()  # interrupt_count
                while st.peek() == CT_OVERRIDE_REG:
                    st.u8()
                    v = st.u8()
                    r = v & 31
                    sz = v & 0x60
                    if r == CT_SR:
                        st.take(1 if sz == 0 else 2)
                    elif r in (CT_FPSR, CT_FPCR, CT_FPIAR):
                        st.take(4)
                    elif r < 16:
                        st.take(12 if sz == 0x60 else 4)
                    else:
                        raise ValueError("override %02x" % v)
                if st.peek() == CT_END_SKIP:
                    st.u8()
                    continue
                # expected-side records until END-flagged byte
                while True:
                    v = st.peek()
                    if v & CT_END:
                        st.u8()
                        exc = v & 63
                        excs[exc] = excs.get(exc, 0) + 1
                        if exc:
                            excdatalen = st.u8()
                            if excdatalen not in (0, 0xFF):
                                payload = bytes(st.take(excdatalen))
                                # First payload byte is the extra-exception
                                # descriptor.  Keep a compact inventory here;
                                # replay_gen performs the semantic decode.
                                extra = payload[0] if payload else 0
                                key = "extra:%02x" % extra
                                excs[key] = excs.get(key, 0) + 1
                            elif excdatalen == 0xFF:
                                excs["frame-reuse"] = excs.get("frame-reuse", 0) + 1
                        break
                    mode = v & 31
                    if mode < 16 and (v >> 5) & 3 == SIZE_FPU:
                        st.restore_fpvalue((0, 0, 0))
                    elif mode in (CT_MEMWRITE, CT_MEMWRITES):
                        if mode == CT_MEMWRITES and (v >> 5) & 3 == PC_BYTES:
                            st.u8()
                            st.restore_bytes()
                        else:
                            st.restore_memory()
                    elif mode == CT_SR:
                        st.restore_value(0)
                    elif mode == CT_BRANCHTARGET:
                        st.restore_value(0)
                        st.u8()
                    elif mode == CT_PC:
                        st.restore_rel(0)  # expected PC is RELATIVE
                    elif mode in (CT_CYCLES, CT_FPIAR, CT_FPSR, CT_FPCR,
                                  CT_SRCADDR, CT_DSTADDR, CT_ENDPC) or mode < 16:
                        st.restore_value(0)
                    else:
                        raise ValueError("expected-mode %02x @%x" % (v, st.p))
                rounds += 1
            if st.peek() == CT_END:
                st.u8()
                break
            st.u8()  # next extraccr value
    return {"tests": tests, "rounds": rounds, "exceptions": excs,
            "consumed": st.p, "size": len(st.d),
            "enveloped": df is not None,
            "instruction_size": df.instruction_size if df else None,
            "last": df.last if df else None}


if __name__ == "__main__":
    hdr = Header(open(sys.argv[1], "rb").read())
    f = sys.argv[2]
    df = DataFile.load(f, header=hdr)
    r = walk_stream(hdr, df, interrupttest=hdr.interrupttest != 0)
    print("%-12s tests=%d rounds=%d consumed=%d/%d exceptions=%s" %
          (hdr.inst_name, r["tests"], r["rounds"], r["consumed"], r["size"],
           sorted(r["exceptions"].items(), key=lambda kv: str(kv[0]))))
