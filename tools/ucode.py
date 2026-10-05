#!/usr/bin/env python3
"""AP68040-60 microcode and decoder generator.

Writes
  rtl/gen/ap68040_ucode_defs.svh   selector encodings shared with D2
  rtl/gen/ap68040_dec_pla.svh      opword -> routine/format (priority casez)
  rtl/gen/ap68040_ucode_rom.svh    microcode ROM (function over the address)

A routine is a list of micro-instructions (U).  Each one becomes one uop,
except that the D2 sequencer expands an EA operand that needs memory
indirection into a pointer load first, and repeats a MOVEM micro-instruction
once per register in the mask.

Operand selectors (a, b, d): symbolic names below, or a physical register
'D0'..'D7', 'A0'..'A6', 'USP', 'ISP', 'MSP', 'T0'..'T13'.

  EA0/EA1   the instruction's effective address: a register, the immediate,
            or memory (a load when used as a or b, a store when used as d;
            b and d both EA0 on memory make a read-modify-write)
  EA1R      EA1 if it is a register, else nothing (MOVE's merge operand)
  DX DY     D register in opword bits 11:9 / 2:0
  AX AY     A register in opword bits 11:9 / 2:0 (A7 mapped by S/M)
  IMM       the immediate field;  QUICK  opword 11:9 (0 -> 8)
  MOVEQ     sign-extended opword 7:0;  SHCNT  shift count (quick or Dx)
  SP SSP    A7 / the supervisor stack of the exception
  NPC       address of the next instruction;  PC  of this one
  ZERO      0;  CONST  the micro-instruction's constant
  X1R X1DL X1DH X1DU X2DC X2DU X2R   extension-word registers
  MVR       MOVEM: the register of this iteration
"""

import os
import sys
from isa import T, MODES, T0_RT

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'rtl', 'gen')

# ----------------------------------------------------------------- encodings
OPS = {n: i for i, n in enumerate("""MOV ADD ADDX SUB SUBX CMP AND OR EOR NOT NEG
NEGX CLR EXT EXTB SWAP TAS ABCD SBCD NBCD PACK UNPK ASL ASR LSL LSR ROL ROR
ROXL ROXR BTST BCHG BCLR BSET SCC EA BCC DBCC TRAPCC CHK CHK2A CHK2B CCRLOG
SRLOG SPR SPW LATCH CAS MUL DIV MDHI MDRES BF BFSET MISC FPU CHKSR CAS2C
CAS2W CAS2R RTEF RTE IACKV""".split())}

SYM = ['NONE', 'EA0', 'EA1', 'EA1R', 'DX', 'DY', 'AX', 'AY', 'IMM', 'QUICK',
       'MOVEQ', 'SHCNT', 'SP', 'SSP', 'NPC', 'PC', 'ZERO', 'CONST', 'X1R',
       'X1DL', 'X1DH', 'X1DU', 'X2DC', 'X2DU', 'X2R', 'MVR', 'EA0R', 'LD',
       'CREG', 'CREGR', 'BFO', 'BFW']
SYMI = {n: i for i, n in enumerate(SYM)}
assert len(SYM) <= 32

PHYS = {}
for i in range(8):
    PHYS['D%d' % i] = i
for i in range(7):
    PHYS['A%d' % i] = 8 + i
PHYS.update(USP=15, ISP=16, MSP=17)
for i in range(14):
    PHYS['T%d' % i] = 18 + i


def sel(x):
    if x is None:
        return 0
    if x in SYMI:
        return SYMI[x]
    return 32 + PHYS[x]


# AG modes
#   EA0/EA1  the instruction's EA (with its (An)+/-(An) update)
#   PUSH/POP SP pre-decrement / post-increment by msz
#   BASED    agb + disp; BASEDU also writes agb (or agw) = EA
#   POPR     access at agb, agw = agb + msz (UNLK)
#   VAL0     agw = value of a register/immediate EA0 (MOVEA)
#   ADDV     agw = agb + value of EA0 (ADDA)
#   ADDC     agw = agb + disp
#   LEA0     agw = address of EA0, no access
#   MOVEM    MOVEM transfer (offset from the sequencer)
#   ADDT0    agw = agb + T0 (bit field byte offset)
AGM = {n: i for i, n in enumerate(
    ['NONE', 'EA0', 'EA1', 'PUSH', 'POP', 'BASED', 'BASEDU', 'VAL0', 'ADDV',
     'ADDC', 'LEA0', 'MOVEM', 'POPR', 'ADDT0'])}
DSEL = {n: i for i, n in enumerate(['CONST', 'IMM', 'NIMM', 'QUICK', 'NQUICK',
                                    'IMMC', 'SZB'])}
MEM = {None: 0, 'LD': 1, 'ST': 2, 'RMW': 3}
MFC = {None: 0, 'SFC': 1, 'DFC': 2, 'SUP': 3, 'IACK': 4}
BR = {None: 0, 'COND': 1, 'IMM': 2, 'EA': 3, 'A': 4, 'B': 5}
# condition sources: opword cc (11:8), the opcode entry's, the extension
# word's ({ext[10], ext[11]}: 64-bit, signed), or a constant
CSRC = {'CC': 1, 'EC': 2, 'X1': 3, 'BFR': 4, 'BFM': 5, 'CHK2': 6}
SZ = {'S': 0, 'B': 1, 'W': 2, 'L': 3, 'Q': 4}
SXW = {None: 0, 1: 1, 'SW': 2}
# static sequencer conditions (evaluated in D2 on the decoded instruction)
JC = {n: i for i, n in enumerate(
    ['NEVER', 'ALWAYS', 'EA0_MEM', 'EA1_MEM', 'EA0_REG', 'EA0_DN', 'EA0_AN',
     'EA0_IMM', 'NOT_EA0_MEM', 'EXT11', 'SZ_L', 'AY7', 'BOTH_MEM',
     'MASK0', 'SUPER', 'EXT10', 'CREG_RF', 'X1A', 'SZ_B'])}
CCR = {'XNZVC': 0x1F, 'NZVC': 0x0F, 'Z': 0x04, 'ZC': 0x05, 'NONE': 0x00}


class U:
    FIELDS = dict(op=None, sz='S', msz='S', cond=None, ccr=None,
                  a=None, b=None, d=None, sxw=None,
                  ag=None, agb=None, agw=None, dsel='CONST', const=0,
                  mem=None, mfc=None, lock=0, locke=0, br=None, last=0, ser=0,
                  noupd=0, upd2=0, jc='NEVER', jt=None, loop=0)

    def __init__(self, **kw):
        for k in kw:
            assert k in self.FIELDS, k
        self.f = dict(self.FIELDS)
        self.f.update(kw)
        self.label = kw.get('label')


ROUTINES = {}
ORDER = []


def R(name, *uops, labels=None):
    assert name not in ROUTINES, name
    ROUTINES[name] = list(uops)
    ORDER.append(name)


# label support: a U with jt='name.k' jumps to micro-instruction k of name
def J(cond, target):
    return dict(jc=cond, jt=target)


# ---------------------------------------------------------------- routines
# Conventions: 'EOP' as op takes the opcode entry's execution op (and its
# condition); 'CCR' as ccr takes the entry's flag mask.

R('ILLEGAL', U(last=1))          # exception set by D1 (vector in the record)
R('DEC_EXC', U(last=1))          # decode-time exception carrier

# ALU <ea>,Dn / Dn,<ea> / #imm,<ea>
R('OP_EA_DN',  U(op='EOP', a='EA0', b='DX', d='DX', ccr='CCR', last=1))
R('OP_DN_EA',  U(op='EOP', a='DX', b='EA0', d='EA0', ccr='CCR', last=1))
R('IMM_OP',    U(op='EOP', a='IMM', b='EA0', d='EA0', ccr='CCR', last=1))
R('IMM_CMP',   U(op='CMP', a='IMM', b='EA0', ccr='NZVC', last=1))
R('CMP_EA_DN', U(op='CMP', a='EA0', b='DX', ccr='NZVC', last=1))
R('CMPA',      U(op='CMP', sz='L', a='EA0', sxw='SW', b='AX', ccr='NZVC', last=1))
R('QUICK',     U(op='EOP', a='QUICK', b='EA0', d='EA0', ccr='CCR', last=1))
R('MOVEQ',     U(op='MOV', sz='L', a='MOVEQ', d='DX', ccr='NZVC', last=1))
R('UNARY',     U(op='EOP', b='EA0', d='EA0', ccr='CCR', last=1))
R('TST',       U(op='MOV', a='EA0', ccr='NZVC', last=1))
R('SCC',       U(op='SCC', cond='CC', b='EA0R', d='EA0', last=1))
R('SWAP',      U(op='SWAP', b='DY', d='DY', ccr='NZVC', last=1))
R('EXT',       U(op='EXT', b='DY', d='DY', ccr='NZVC', last=1))
R('EXTB',      U(op='EXTB', b='DY', d='DY', ccr='NZVC', last=1))
R('TAS',       U(op='TAS', b='EA0', d='EA0', lock=1, locke=1, ccr='NZVC', last=1))

# address-register arithmetic: register/immediate sources run in AG
R('ADDA',
  U(jc='EA0_MEM', jt='ADDA.2'),
  U(ag='ADDV', agb='AX', agw='AX', last=1),
  U(op='ADD', sz='L', a='EA0', sxw='SW', b='AX', d='AX', last=1))
R('SUBA',
  U(jc='EA0_IMM', jt='SUBA.2'),
  U(op='SUB', sz='L', a='EA0', sxw='SW', b='AX', d='AX', last=1),
  U(ag='ADDC', agb='AX', agw='AX', dsel='NIMM', last=1))
R('QUICK_A',   U(ag='ADDC', agb='AY', agw='AY', dsel='QUICK', last=1))
R('QUICK_SA',  U(ag='ADDC', agb='AY', agw='AY', dsel='NQUICK', last=1))
R('LEA',       U(ag='LEA0', agw='AX', last=1))
R('PEA',       U(ag='LEA0', agw='T0'),
               U(op='MOV', sz='L', a='T0', ag='PUSH', msz='L', mem='ST', last=1))
R('MOVEA',
  U(jc='EA0_MEM', jt='MOVEA.2'),
  U(ag='VAL0', agw='AX', last=1),
  U(op='MOV', sz='L', a='EA0', sxw='SW', d='AX', last=1))

# MOVE: one uop unless both operands are in memory; then the source
# address update is deferred into the store uop (restartable)
R('MOVE',
  U(jc='BOTH_MEM', jt='MOVE.2'),
  U(op='MOV', a='EA0', b='EA1R', d='EA1', ccr='NZVC', last=1),
  U(op='MOV', a='EA0', d='T0', noupd=1),
  U(op='MOV', a='T0', d='EA1', upd2=1, ccr='NZVC', last=1))

# bit operations: the bit number is the immediate or Dx; a register
# destination is a long operation (mod 32), memory a byte (mod 8)
R('BIT_IMM',
  U(jc='EA0_DN', jt='BIT_IMM.2'),
  U(op='EOP', a='IMM', b='EA0', d='EA0', ccr='Z', last=1),
  U(op='EOP', sz='L', a='IMM', b='EA0', d='EA0', ccr='Z', last=1))
R('BIT_DYN',
  U(jc='EA0_DN', jt='BIT_DYN.2'),
  U(op='EOP', a='DX', b='EA0', d='EA0', ccr='Z', last=1),
  U(op='EOP', sz='L', a='DX', b='EA0', d='EA0', ccr='Z', last=1))
R('BTST_IMM',
  U(jc='EA0_DN', jt='BTST_IMM.2'),
  U(op='BTST', a='IMM', b='EA0', ccr='Z', last=1),
  U(op='BTST', sz='L', a='IMM', b='EA0', ccr='Z', last=1))
R('BTST_DYN',
  U(jc='EA0_DN', jt='BTST_DYN.2'),
  U(op='BTST', a='DX', b='EA0', ccr='Z', last=1),
  U(op='BTST', sz='L', a='DX', b='EA0', ccr='Z', last=1))

# shifts: register forms take the count from Dx or the quick field
R('SHIFT_R', U(op='EOP', a='SHCNT', b='DY', d='DY', ccr='CCR', last=1))
R('SHIFT_M', U(op='EOP', a='CONST', const=1, b='EA0', d='EA0', ccr='CCR', last=1))

# multi-precision: register and -(Ay),-(Ax) forms
R('ADDX_R', U(op='EOP', a='DY', b='DX', d='DX', ccr='CCR', last=1))
R('ADDX_M',
  U(op='MOV', a='EA0', d='T0', noupd=1),
  U(op='EOP', a='T0', b='EA1', d='EA1', upd2=1, ccr='CCR', last=1))
R('CMPM',
  U(op='MOV', a='EA0', d='T0', noupd=1),
  U(op='CMP', a='T0', b='EA1', upd2=1, ccr='NZVC', last=1))
# PACK/UNPK: the ALU result is the converted field; a second uop merges
# it into Dx or stores it
R('PACK_R',
  U(op='PACK', a='IMM', b='DY', d='T0'),
  U(op='MOV', sz='B', a='T0', b='DX', d='DX', last=1))
R('PACK_M',
  U(op='MOV', sz='W', msz='W', a='EA0', d='T0', noupd=1),
  U(op='PACK', a='IMM', b='T0', d='T0'),
  U(op='MOV', sz='B', msz='B', a='T0', d='EA1', upd2=1, last=1))
R('UNPK_R',
  U(op='UNPK', a='IMM', b='DY', d='T0'),
  U(op='MOV', sz='W', a='T0', b='DX', d='DX', last=1))
R('UNPK_M',
  U(op='MOV', sz='B', msz='B', a='EA0', d='T0', noupd=1),
  U(op='UNPK', a='IMM', b='T0', d='T0'),
  U(op='MOV', sz='W', msz='W', a='T0', d='EA1', upd2=1, last=1))

# multiply / divide (word forms); the signed variant is the entry cond
R('MULW', U(op='EOP', cond='EC', a='EA0', b='DX', d='DX', ccr='NZVC', last=1))
R('DIVW', U(op='EOP', cond='EC', a='EA0', b='DX', d='DX', ccr='NZVC', last=1))
# long forms: ext word Dl/Dq = 14:12, Dh/Dr = 2:0, bit 11 signed, bit 10
# 64-bit.  Write order follows the 68040: MULx.L 64 writes Dh then Dl,
# DIVx.L writes Dr then Dq (a shared register keeps low / quotient).
R('MULL',
  U(jc='EXT10', jt='MULL.2'),
  U(op='MUL', cond='X1', sz='L', a='EA0', b='X1DL', d='X1DL', ccr='NZVC', last=1),
  U(op='MUL', cond='X1', sz='L', a='EA0', b='X1DL', ccr='NZVC'),
  U(op='MDRES', cond=0, d='X1DH'),
  U(op='MDRES', cond=1, d='X1DL', last=1))
R('DIVL',
  U(op='MDHI', a='X1DH'),
  U(op='DIV', cond='X1', sz='L', a='EA0', b='X1DL', ccr='NZVC'),
  U(op='MDRES', cond=0, d='X1DH'),
  U(op='MDRES', cond=1, d='X1DL', last=1))

# CHK
R('CHK', U(op='CHK', a='EA0', b='DX', ccr='NZVC', last=1))

# exchange
R('EXG_DD', U(op='MOV', sz='L', a='DX', d='T0'),
            U(op='MOV', sz='L', a='DY', d='DX'),
            U(op='MOV', sz='L', a='T0', d='DY', last=1))
R('EXG_AA', U(op='MOV', sz='L', a='AX', d='T0'),
            U(op='MOV', sz='L', a='AY', d='AX'),
            U(op='MOV', sz='L', a='T0', d='AY', last=1))
R('EXG_DA', U(op='MOV', sz='L', a='DX', d='T0'),
            U(op='MOV', sz='L', a='AY', d='DX'),
            U(op='MOV', sz='L', a='T0', d='AY', last=1))

# condition / flow
R('BCC',    U(op='BCC', cond='CC', br='COND', last=1))
R('BSR',    U(op='MOV', sz='L', a='NPC', ag='PUSH', msz='L', mem='ST', br='IMM', last=1))
R('DBCC',   U(op='DBCC', cond='CC', b='DY', d='DY', br='COND', last=1))
R('JMP',    U(ag='LEA0', br='EA', last=1))
R('JSR',    U(ag='LEA0', agw='T0'),
            U(op='MOV', sz='L', a='NPC', b='T0', ag='PUSH', msz='L', mem='ST', br='B', last=1))
R('RTS',    U(op='MOV', sz='L', a='LD', ag='POP', msz='L', mem='LD', br='A', last=1))
R('RTD',    U(op='MOV', sz='L', a='LD', ag='BASED', agb='SP', msz='L', mem='LD', d='T0'),
            U(ag='ADDC', agb='SP', agw='SP', dsel='IMMC', const=4, a='T0', br='A', last=1))
R('TRAPCC', U(op='TRAPCC', cond='CC', last=1))
R('TRAPV',  U(op='TRAPCC', cond=9, last=1))

# stack frames.  LINK A7 pushes the decremented A7 (68040).
R('LINK',   U(jc='AY7', jt='LINK.4'),
            U(op='MOV', sz='L', a='AY', ag='PUSH', msz='L', mem='ST'),
            U(ag='BASED', agb='SP', agw='AY'),
            U(ag='ADDC', agb='SP', agw='SP', dsel='IMM', last=1),
            U(op='SUB', sz='L', a='CONST', const=4, b='SP', ag='PUSH', msz='L', mem='ST'),
            U(ag='ADDC', agb='SP', agw='SP', dsel='IMM', last=1))
# UNLK: SP = An + 4 and An = (An) in one uop
R('UNLK',   U(op='MOV', sz='L', a='LD', ag='POPR', agb='AY', agw='SP', msz='L',
              mem='LD', d='AY', last=1))

# status register
R('MOVE_FROM_SR',  U(op='SPR', sz='W', const=0x10, b='EA0R', d='EA0', last=1))
R('MOVE_FROM_CCR', U(op='SPR', sz='W', const=0x11, b='EA0R', d='EA0', last=1))
R('MOVE_TO_CCR',   U(op='CCRLOG', cond=3, a='EA0', ccr='XNZVC', last=1))
R('MOVE_TO_SR',    U(op='SRLOG', cond=3, a='EA0', ccr='XNZVC', last=1))
R('CCR_LOG',       U(op='CCRLOG', cond='EC', a='IMM', ccr='XNZVC', last=1))
R('SR_LOG',        U(op='SRLOG', cond='EC', a='IMM', ccr='XNZVC', last=1))
R('MOVE_TO_USP',   U(op='MOV', sz='L', a='AY', d='USP', last=1))
R('MOVE_FROM_USP', U(op='MOV', sz='L', a='USP', d='AY', last=1))

# TRAP #n, ILLEGAL and BKPT raise their exception from D1
R('TRAP',  U(last=1))
R('BKPT',  U(last=1))
R('NOP',   U(op='MISC', cond=0, ser=1, last=1))
R('RESET', U(op='MISC', cond=1, ser=1, last=1))
R('STOP',  U(op='MISC', cond=2, a='IMM', ser=1, last=1))

# not yet implemented in this revision: they raise F-line/illegal through
# D1 until their routines land (see doc/STATUS.md)
# MOVEC: USP/MSP/ISP live in the register file, the rest are special
# registers written at WB (serializing)
R('MOVEC_RD',
  U(jc='CREG_RF', jt='MOVEC_RD.2'),
  U(op='SPR', sz='L', a='CREG', d='X1R', last=1),
  U(op='MOV', sz='L', a='CREGR', d='X1R', last=1))
R('MOVEC_WR',
  U(jc='CREG_RF', jt='MOVEC_WR.2'),
  U(op='SPW', sz='L', a='X1R', b='CREG', ser=1, last=1),
  U(op='MOV', sz='L', a='X1R', d='CREGR', ser=1, last=1))

# MOVEM: one micro-instruction repeated per register (D2 loop); an empty
# mask transfers nothing and leaves An alone
R('MOVEM_RM',
  U(jc='MASK0', jt='MOVEM_RM.2'),
  U(op='MOV', a='MVR', d='EA0', loop=1, last=1),
  U(last=1))
R('MOVEM_MR',
  U(jc='MASK0', jt='MOVEM_MR.2'),
  U(op='MOV', sz='L', msz='S', sxw='SW', a='EA0', d='MVR', loop=1, last=1),
  U(last=1))

# MOVE16: a line read into the DMU's line buffer, then a line write; the
# source update is deferred into the write
R('MOVE16',
  U(op='MOV', sz='L', msz='Q', a='EA0', noupd=1),
  U(op='MOV', sz='L', msz='Q', d='EA1', upd2=1, last=1))

# MOVEP: bytes at d16(Ay) + 0, 2 (, 4, 6), most significant first
def _mpb(k, **kw):
    return U(op='MOV', sz='B', msz='B', ag='BASED', agb='AY', dsel='IMMC',
             const=2 * k, **kw)
R('MOVEP_MR',
  _mpb(0, a='LD', b='ZERO', d='T0'),
  U(op='LSL', sz='L', a='CONST', const=8, b='T0', d='T0'),
  _mpb(1, a='LD', b='T0', d='T0'),
  U(jc='SZ_L', jt='MOVEP_MR.5'),
  U(op='MOV', sz='W', a='T0', b='DX', d='DX', last=1),
  U(op='LSL', sz='L', a='CONST', const=8, b='T0', d='T0'),
  _mpb(2, a='LD', b='T0', d='T0'),
  U(op='LSL', sz='L', a='CONST', const=8, b='T0', d='T0'),
  _mpb(3, a='LD', b='T0', d='T0'),
  U(op='MOV', sz='L', a='T0', d='DX', last=1))
R('MOVEP_RM',
  U(jc='SZ_L', jt='MOVEP_RM.4'),
  U(op='LSR', sz='L', a='CONST', const=8, b='DX', d='T0'),
  _mpb(0, a='T0', mem='ST'),
  _mpb(1, a='DX', mem='ST', last=1),
  U(op='LSR', sz='L', a='CONST', const=24, b='DX', d='T0'),
  _mpb(0, a='T0', mem='ST'),
  U(op='LSR', sz='L', a='CONST', const=16, b='DX', d='T0'),
  _mpb(1, a='T0', mem='ST'),
  U(op='LSR', sz='L', a='CONST', const=8, b='DX', d='T0'),
  _mpb(2, a='T0', mem='ST'),
  _mpb(3, a='DX', mem='ST', last=1))

# bit fields.  BFSET latches offset (BFO) and width (BFW) in EX and returns
# the signed byte offset of the field (offset >> 3).  The memory form works
# on a five-byte window: T2 = long at EA + offset/8, T3 = the next byte.
# cond 'BFR'/'BFM' = opword 10:8 with the memory flag clear/set.
_BFSETUP = U(op='BFSET', cond=0, sz='L', a='BFO', b='BFW', d='T0')
def _bf_mem_load():
    return [U(ag='LEA0', agw='T1'),
            U(ag='ADDT0', agb='T1', agw='T1'),
            U(op='MOV', sz='L', msz='L', a='LD', d='T2', ag='BASED', agb='T1', const=0),
            U(op='MOV', sz='B', msz='B', a='LD', b='ZERO', d='T3', ag='BASED', agb='T1', const=4),
            U(op='LATCH', sz='L', a='T3')]
R('BF_TST',
  U(jc='EA0_DN', jt='BF_TST.8'),
  _BFSETUP, *_bf_mem_load(),
  U(op='BF', cond='BFM', a='X1DL', b='T2', ccr='NZVC', last=1),
  _BFSETUP,
  U(op='BF', cond='BFR', a='X1DL', b='EA0', ccr='NZVC', last=1))
R('BF_EXT',
  U(jc='EA0_DN', jt='BF_EXT.8'),
  _BFSETUP, *_bf_mem_load(),
  U(op='BF', cond='BFM', a='X1DL', b='T2', d='X1DL', ccr='NZVC', last=1),
  _BFSETUP,
  U(op='BF', cond='BFR', a='X1DL', b='EA0', d='X1DL', ccr='NZVC', last=1))
R('BF_MOD',
  U(jc='EA0_DN', jt='BF_MOD.11'),
  _BFSETUP, *_bf_mem_load(),
  U(op='BF', cond='BFM', a='X1DL', b='T2', d='T2', ccr='NZVC'),
  U(op='BFSET', cond=1, sz='L', d='T3'),
  U(op='MOV', sz='L', msz='L', a='T2', mem='ST', ag='BASED', agb='T1', const=0),
  U(op='MOV', sz='B', msz='B', a='T3', mem='ST', ag='BASED', agb='T1', const=4, last=1),
  _BFSETUP,
  U(op='BF', cond='BFR', a='X1DL', b='EA0', d='EA0', ccr='NZVC', last=1))

# CAS Dc,Du,<ea>: a locked read-modify-write.  The 68040 always writes:
# Du on a match, the value read on a mismatch (then Dc = that value).
R('CAS',
  U(op='LATCH', sz='L', a='X1DH'),
  U(op='CAS', a='X1DU', b='EA0', d='X1DH', mem='RMW', ag='EA0', lock=1, locke=1,
    ccr='NZVC', last=1))
# CAS2: two locked reads, the compare leaves eq in EX; the first write
# only on a match, the second always (Du2 or the second value read, with
# LOCKE); on a mismatch Dc1/Dc2 take the values read
R('CAS2',
  U(op='MOV', a='LD', d='T0', ag='BASED', agb='X1R', lock=1),
  U(op='MOV', a='LD', d='T1', ag='BASED', agb='X2R', lock=1),
  U(op='CAS2C', cond=0, a='X1DH', b='T0', ccr='NZVC'),
  U(op='CAS2C', cond=1, a='X2DC', b='T1', ccr='NZVC'),
  U(op='CAS2W', cond=0, a='X1DU', mem='ST', ag='BASED', agb='X1R', lock=1),
  U(op='CAS2W', cond=1, a='X2DU', b='T1', mem='ST', ag='BASED', agb='X2R', lock=1, locke=1),
  U(op='CAS2R', a='T0', b='X1DH', d='X1DH'),
  U(op='CAS2R', a='T1', b='X2DC', d='X2DC', last=1))

# CHK2/CMP2 <ea>,Rn: bounds at EA and EA + size; cond 'CHK2' carries
# {Rn is a data register, CHK2}
R('CHK2',
  U(ag='LEA0', agw='T1'),
  U(op='MOV', sz='L', a='LD', d='T0', ag='BASED', agb='T1', const=0),
  U(op='MOV', sz='L', a='LD', d='T2', ag='BASED', agb='T1', dsel='SZB'),
  U(op='LATCH', sz='L', a='T0'),
  U(op='CHK2B', cond='CHK2', a='T2', b='X1R', ccr='ZC', last=1))

# RTE: SR, PC and the format word; RTEF turns the format into the frame
# length (exception 14 on an unknown format; format $1 marks a throwaway
# frame), SP += length, then RTE loads SR and jumps -- for a throwaway
# frame back to this RTE, which then runs on the stack the new SR selects
R('RTE',
  U(op='MOV', sz='W', msz='W', a='LD', d='T1', ag='BASED', agb='SP', const=0),
  U(op='MOV', sz='L', msz='L', a='LD', d='T2', ag='BASED', agb='SP', const=2),
  U(op='MOV', sz='W', msz='W', a='LD', d='T3', ag='BASED', agb='SP', const=6),
  U(op='RTEF', sz='W', a='T3', d='T0'),
  U(ag='ADDT0', agb='SP', agw='SP'),
  U(op='RTE', sz='W', a='T1', b='T2', last=1))
# RTR: CCR = (SP)+ word, PC = (SP)+ long
R('RTR',
  U(op='MOV', sz='W', msz='W', a='LD', d='T1', ag='BASED', agb='SP', const=0),
  U(op='MOV', sz='L', msz='L', a='LD', d='T2', ag='BASED', agb='SP', const=2),
  U(op='CCRLOG', cond=3, a='T1', b='T2', ag='ADDC', agb='SP', agw='SP', const=6,
    ccr='XNZVC', br='B', last=1))

# FSAVE/FRESTORE with the FPU in its reset state: a four-byte NULL frame
# (replaced by the FPU's frames when the FPU is attached)
R('FSAVE',    U(op='MOV', sz='L', msz='L', a='ZERO', d='EA0', last=1))
R('FRESTORE', U(op='MOV', sz='L', msz='L', a='EA0', d='T0', last=1))

# MOVES: ext bit 11 = register to memory (DFC), else memory to register
# (SFC); an address register takes the operand sign-extended to 32 bits
R('MOVES',
  U(jc='EXT11', jt='MOVES.9'),
  U(jc='X1A', jt='MOVES.3'),
  U(op='MOV', a='EA0', b='X1R', d='X1R', mfc='SFC', last=1),
  U(op='MOV', a='EA0', d='T0', mfc='SFC'),
  U(jc='SZ_L', jt='MOVES.8'),
  U(jc='SZ_B', jt='MOVES.7'),
  U(op='EXT', sz='L', b='T0', d='X1R', last=1),
  U(op='EXTB', sz='L', b='T0', d='X1R', last=1),
  U(op='MOV', sz='L', a='T0', d='X1R', last=1),
  U(op='MOV', a='X1R', d='EA0', mfc='DFC', last=1))

for n in ['FPU_GEN',
          'FSCC', 'FDBCC', 'FTRAPCC', 'FBCC',
          'CACHE_OP', 'PFLUSH', 'PTEST']:
    R(n, U(last=1))

# exception routines.  At entry the back end has set S, cleared T, and
# written: T8 vector address, T9 PC to stack, T10 old SR, T11 address
# (format $2) / fault address, T12 format-vector word, T13 SSW
R('EXC_FMT0',
  U(op='MOV', sz='W', a='T12', ag='BASED', agb='SSP', const=-2, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', a='T9',  ag='BASED', agb='SSP', const=-6, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T10', ag='BASEDU', agb='SSP', const=-8, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', ag='BASED', agb='T8', const=0, msz='L', mem='LD', mfc='SUP',
    a='LD', br='A', last=1))
R('EXC_FMT2',
  U(op='MOV', sz='L', a='T11', ag='BASED', agb='SSP', const=-4, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T12', ag='BASED', agb='SSP', const=-6, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', a='T9',  ag='BASED', agb='SSP', const=-10, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T10', ag='BASEDU', agb='SSP', const=-12, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', ag='BASED', agb='T8', const=0, msz='L', mem='LD', mfc='SUP',
    a='LD', br='A', last=1))
_f7 = [U(op='MOV', sz='L', a='ZERO', ag='BASED', agb='SSP', const=-4 * (k + 1),
         msz='L', mem='ST', mfc='SUP') for k in range(10)]
R('EXC_FMT7',
  *_f7,                                                     # PD3..WB3A area
  U(op='MOV', sz='L', a='T11', ag='BASED', agb='SSP', const=-40, msz='L', mem='ST', mfc='SUP'),  # FA
  U(op='MOV', sz='L', a='ZERO', ag='BASED', agb='SSP', const=-44, msz='L', mem='ST', mfc='SUP'), # WB2S WB1S
  U(op='MOV', sz='W', a='ZERO', ag='BASED', agb='SSP', const=-46, msz='W', mem='ST', mfc='SUP'), # WB3S
  U(op='MOV', sz='W', a='T13', ag='BASED', agb='SSP', const=-48, msz='W', mem='ST', mfc='SUP'),  # SSW
  U(op='MOV', sz='L', a='T11', ag='BASED', agb='SSP', const=-52, msz='L', mem='ST', mfc='SUP'),  # EA
  U(op='MOV', sz='W', a='T12', ag='BASED', agb='SSP', const=-54, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', a='T9',  ag='BASED', agb='SSP', const=-58, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T10', ag='BASEDU', agb='SSP', const=-60, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', ag='BASED', agb='T8', const=0, msz='L', mem='LD', mfc='SUP',
    a='LD', br='A', last=1))
R('EXC_RESET',
  U(op='MOV', sz='L', ag='BASED', agb='ZERO', const=0, msz='L', mem='LD', mfc='SUP',
    a='LD', d='ISP'),
  U(op='MOV', sz='L', ag='BASED', agb='ZERO', const=4, msz='L', mem='LD', mfc='SUP',
    a='LD', br='A', last=1))
# interrupts.  At entry the back end has set S, cleared T, raised the mask,
# and written T9 PC, T10 old SR; the IACK cycle (TT=3, TM=level) returns
# the vector (AVEC: autovector, TEA: spurious).  IACKV turns it into the
# format/vector word (cond 0) and the vector address (cond 1).
R('EXC_IRQ',
  U(op='IACKV', cond=0, sz='L', msz='B', a='LD', d='T12', mem='LD', mfc='IACK',
    ag='BASED', agb='ZERO', const=-1),
  U(op='IACKV', cond=1, sz='L', d='T8'),
  U(op='MOV', sz='W', a='T12', ag='BASED', agb='ISP', const=-2, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', a='T9',  ag='BASED', agb='ISP', const=-6, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T10', ag='BASEDU', agb='ISP', const=-8, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', ag='BASED', agb='T8', const=0, msz='L', mem='LD', mfc='SUP',
    a='LD', br='A', last=1))
# with M set (68020-68040): format $0 on the master stack, then a format $1
# throwaway frame on the interrupt stack with S set in its SR
R('EXC_IRQM',
  U(op='IACKV', cond=0, sz='L', msz='B', a='LD', d='T12', mem='LD', mfc='IACK',
    ag='BASED', agb='ZERO', const=-1),
  U(op='IACKV', cond=1, sz='L', d='T8'),
  U(op='MOV', sz='W', a='T12', ag='BASED', agb='MSP', const=-2, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', a='T9',  ag='BASED', agb='MSP', const=-6, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T10', ag='BASEDU', agb='MSP', const=-8, msz='W', mem='ST', mfc='SUP'),
  U(op='OR', sz='W', a='CONST', const=0x1000, b='T12', d='T7'),
  U(op='OR', sz='W', a='CONST', const=0x2000, b='T10', d='T6'),
  U(op='MOV', sz='W', a='T7',  ag='BASED', agb='ISP', const=-2, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', a='T9',  ag='BASED', agb='ISP', const=-6, msz='L', mem='ST', mfc='SUP'),
  U(op='MOV', sz='W', a='T6',  ag='BASEDU', agb='ISP', const=-8, msz='W', mem='ST', mfc='SUP'),
  U(op='MOV', sz='L', ag='BASED', agb='T8', const=0, msz='L', mem='LD', mfc='SUP',
    a='LD', br='A', last=1))

# ------------------------------------------------------------ entry params
EOP = {
    'OR': 'OR', 'AND': 'AND', 'SUB': 'SUB', 'ADD': 'ADD', 'EOR': 'EOR',
    'CMP': 'CMP', 'NEGX': 'NEGX', 'CLR': 'CLR', 'NEG': 'NEG', 'NOT': 'NOT',
    'NBCD': 'NBCD', 'ADDX': 'ADDX', 'SUBX': 'SUBX', 'ABCD': 'ABCD',
    'SBCD': 'SBCD', 'BTST': 'BTST', 'BCHG': 'BCHG', 'BCLR': 'BCLR',
    'BSET': 'BSET', 'PACK': 'PACK', 'UNPK': 'UNPK', 'ADDA': 'ADD',
    'SUBA': 'SUB', 'MULU': 'MUL', 'MULS': 'MUL', 'DIVU': 'DIV', 'DIVS': 'DIV',
    'ASL': 'ASL', 'ASR': 'ASR', 'LSL': 'LSL', 'LSR': 'LSR', 'ROL': 'ROL',
    'ROR': 'ROR', 'ROXL': 'ROXL', 'ROXR': 'ROXR', 'ASLW': 'ASL',
    'ASRW': 'ASR', 'LSLW': 'LSL', 'LSRW': 'LSR', 'ROLW': 'ROL', 'RORW': 'ROR',
    'ROXLW': 'ROXL', 'ROXRW': 'ROXR', 'ORSR': 'CCRLOG', 'ANDSR': 'CCRLOG',
    'EORSR': 'CCRLOG',
}
ECCR = {
    'ADD': 'XNZVC', 'SUB': 'XNZVC', 'ADDX': 'XNZVC', 'SUBX': 'XNZVC',
    'NEG': 'XNZVC', 'NEGX': 'XNZVC', 'ABCD': 'XNZVC', 'SBCD': 'XNZVC',
    'NBCD': 'XNZVC', 'ASL': 'XNZVC', 'ASR': 'XNZVC', 'LSL': 'XNZVC',
    'LSR': 'XNZVC', 'ROXL': 'XNZVC', 'ROXR': 'XNZVC', 'ASLW': 'XNZVC',
    'ASRW': 'XNZVC', 'LSLW': 'XNZVC', 'LSRW': 'XNZVC', 'ROXLW': 'XNZVC',
    'ROXRW': 'XNZVC',
    'ROL': 'NZVC', 'ROR': 'NZVC', 'ROLW': 'NZVC', 'RORW': 'NZVC',
    'OR': 'NZVC', 'AND': 'NZVC', 'EOR': 'NZVC', 'NOT': 'NZVC', 'CLR': 'NZVC',
    'CMP': 'NZVC', 'MULU': 'NZVC', 'MULS': 'NZVC', 'DIVU': 'NZVC',
    'DIVS': 'NZVC', 'ADDA': 'NONE', 'SUBA': 'NONE',
}
# signed multiply/divide and the CCR/SR logic variant ride in the cond field
ECOND = {'MULS': 1, 'DIVS': 1, 'ORSR': 1, 'ANDSR': 0, 'EORSR': 2}

# imm kind encoding for D1
IMMK = {None: 0, 'z': 1, 'B': 2, 'W': 3, 'L': 4, 'bcc': 5, 'trapcc': 6,
        'fbcc': 7}
SZC = {'B': 0, 'W': 1, 'L': 2, 'U': 2, 'z': 3, 'Z': 3}

# ---------------------------------------------------------------- assemble
ROM = []
ENTRY = {}


def resolve_jumps():
    for name in ORDER:
        ENTRY[name] = len(ROM)
        ROM.extend((name, k, u) for k, u in enumerate(ROUTINES[name]))


def val(u, name):
    f = u.f
    jt = 0
    if f['jt'] is not None:
        rn, k = f['jt'].split('.')
        jt = ENTRY[rn] + int(k)
    op = f['op']
    op_inst = 1 if op == 'EOP' else 0
    opc = 0 if op in (None, 'EOP') else OPS[op]
    cond = f['cond']
    cond_inst = CSRC.get(cond, 0) if not isinstance(cond, int) else 0
    condc = cond if isinstance(cond, int) else 0
    ccr = f['ccr']
    ccr_inst = 1 if ccr == 'CCR' else 0
    ccrc = 0 if ccr in (None, 'CCR') else CCR[ccr]
    v = dict(
        op_inst=op_inst, op=opc, sz=SZ[f['sz']], msz=SZ[f['msz']],
        cond_inst=cond_inst, cond=condc, ccr_inst=ccr_inst, ccr=ccrc,
        a=sel(f['a']), b=sel(f['b']), d=sel(f['d']), sxw=SXW[f['sxw']],
        ag=AGM[f['ag'] or 'NONE'], agb=sel(f['agb']), agw=sel(f['agw']),
        dsel=DSEL[f['dsel']], cval=f['const'] & 0xFFFF,
        mem=MEM[f['mem']], mfc=MFC[f['mfc']], lock=f['lock'], locke=f['locke'],
        br=BR[f['br']],
        last=f['last'], ser=f['ser'], noupd=f['noupd'], upd2=f['upd2'],
        jc=JC[f['jc']], jt=jt, loop=f['loop'])
    return v


LAYOUT = [('op_inst', 1), ('op', 7), ('sz', 2), ('msz', 3), ('cond_inst', 3),
          ('cond', 4), ('ccr_inst', 1), ('ccr', 5), ('a', 6), ('b', 6),
          ('d', 6), ('sxw', 2), ('ag', 4), ('agb', 6), ('agw', 6),
          ('dsel', 3), ('cval', 16), ('mem', 2), ('mfc', 3), ('lock', 1),
          ('locke', 1),
          ('br', 3), ('last', 1), ('ser', 1), ('noupd', 1), ('upd2', 1),
          ('jc', 5), ('jt', 9), ('loop', 1)]
WIDTH = sum(w for _, w in LAYOUT)


def pack(v):
    x = 0
    for n, w in LAYOUT:
        assert 0 <= v[n] < (1 << w), (n, v[n])
        x = (x << w) | v[n]
    return x


def emit():
    os.makedirs(OUT, exist_ok=True)
    resolve_jumps()
    assert len(ROM) <= 512, len(ROM)

    # shared definitions
    with open(os.path.join(OUT, 'ap68040_upkg.sv'), 'w') as f:
        f.write('// generated by tools/ucode.py - do not edit\n')
        f.write('package ap68040_upkg;\n')
        f.write('localparam int UW = %d;\n' % WIDTH)
        f.write('typedef struct packed {\n')
        for n, w in LAYOUT:
            f.write('\tlogic [%d:0] %s;\n' % (w - 1, n))
        f.write('} uword_t;\n')
        for n, i in SYMI.items():
            f.write('localparam logic [5:0] S_%s = 6\'d%d;\n' % (n, i))
        for n, i in AGM.items():
            f.write('localparam logic [3:0] AGM_%s = 4\'d%d;\n' % (n, i))
        for n, i in DSEL.items():
            f.write('localparam logic [2:0] DS_%s = 3\'d%d;\n' % (n, i))
        for n, i in JC.items():
            f.write('localparam logic [4:0] JC_%s = 5\'d%d;\n' % (n, i))
        for n in ['EXC_FMT0', 'EXC_FMT2', 'EXC_FMT7', 'EXC_RESET', 'EXC_IRQ', 'EXC_IRQM',
                  'DEC_EXC', 'BCC', 'BSR', 'DBCC', 'FBCC', 'FDBCC', 'FPU_GEN',
                  'TRAP', 'BKPT', 'ILLEGAL', 'MOVEM_RM', 'MOVEM_MR',
                  'MOVEC_RD', 'MOVEC_WR']:
            f.write('localparam logic [8:0] UA_%s = 9\'d%d;\n' % (n, ENTRY[n]))
        f.write('endpackage\n')

    # ROM
    with open(os.path.join(OUT, 'ap68040_ucode_rom.svh'), 'w') as f:
        f.write('// generated by tools/ucode.py - do not edit\n')
        f.write('function automatic uword_t ucode_rom(input logic [8:0] a);\n')
        f.write('\tcase (a)\n')
        for i, (name, k, u) in enumerate(ROM):
            f.write("\t\t9'd%d: ucode_rom = %d'h%x;  // %s.%d\n" %
                    (i, WIDTH, pack(val(u, name)), name, k))
        f.write("\t\tdefault: ucode_rom = '0;\n\tendcase\nendfunction\n")

    # decoder: parallel match of every pattern, each excluded by the more
    # specific patterns that overlap it (one-hot), then OR trees
    ents = list(T)
    def fixed(e):
        return sum(c in '01' for c in e.pat)
    def inter(x, y):
        return all(not (a in '01' and b in '01' and a != b) for a, b in zip(x.pat, y.pat))
    def mv(e):
        m = v = 0
        for i, c in enumerate(e.pat):
            bit = 15 - i
            if c in '01':
                m |= 1 << bit
                if c == '1':
                    v |= 1 << bit
        return m, v
    with open(os.path.join(OUT, 'ap68040_dec_pla.svh'), 'w') as f:
        f.write('// generated by tools/ucode.py - do not edit\n')
        f.write('// Operation word decoder: every pattern is matched in parallel and\n')
        f.write('// excluded by the more specific patterns overlapping it, so exactly\n')
        f.write('// one entry (or none) is selected; attributes are OR trees.\n')
        f.write('localparam int NENT = %d;\n' % len(ents))
        f.write('typedef struct packed {\n\tlogic match;\n\tlogic [8:0] ent;\n\tlogic [8:0] rt;\n'
                '\tlogic [6:0] eop;\n\tlogic [3:0] econd;\n\tlogic econd_v;\n'
                '\tlogic [4:0] ccr;\n'
                '\tlogic [1:0] szc;\n\tlogic [11:0] ea0m;\n\tlogic [11:0] ea1m;\n'
                '\tlogic [1:0] nfix;\n\tlogic [2:0] immk;\n\tlogic priv;\n'
                '\tlogic [4:0] jc0;\n\tlogic [8:0] jt0;\n\tlogic ea0v;\n\tlogic ea1v;\n\tlogic [1:0] fea;\n\tlogic t0;\n'
                '} pla_t;\n')
        # entry selection
        f.write('function automatic logic [NENT-1:0] dec_sel(input logic [15:0] op);\n')
        f.write('\tlogic [NENT-1:0] m, s;\n')
        for i, e in enumerate(ents):
            m, v = mv(e)
            zc = " && (op[7:6] != 2'b11)" if e.sz == 'z' else ''
            f.write("\tm[%d] = ((op & 16'h%04x) == 16'h%04x)%s;  // %s %s\n" % (i, m, v, zc, e.name, e.rt))
        for i, e in enumerate(ents):
            ex = [j for j, x in enumerate(ents) if x is not e and fixed(x) > fixed(e) and inter(e, x)]
            if ex:
                f.write('\ts[%d] = m[%d] & ~(%s);\n' % (i, i, ' | '.join('m[%d]' % j for j in ex)))
            else:
                f.write('\ts[%d] = m[%d];\n' % (i, i))
        f.write('\tdec_sel = s;\nendfunction\n\n')
        # attributes of an entry index
        f.write('function automatic pla_t ent_attr(input logic [8:0] ent, input logic hit);\n')
        f.write("\tent_attr = '0;\n\tcase (ent)\n")
        for i, e in enumerate(ents):
            mask0 = 0 if e.ea0 is None else sum(1 << k for k, mm in enumerate(MODES) if mm in e.ea0)
            mask1 = 0 if e.ea1 is None else sum(1 << k for k, mm in enumerate(MODES) if mm in e.ea1)
            eop = OPS[EOP[e.name]] if e.name in EOP else 0
            ec = ECOND.get(e.name)
            ccr = CCR[ECCR.get(e.name, 'NONE')]
            u0 = ROUTINES[e.rt][0].f
            jc0 = JC[u0['jc']]
            jt0 = 0
            if u0['jt'] is not None:
                rn, k = u0['jt'].split('.')
                jt0 = ENTRY[rn] + int(k)
            f.write("\t\t9'd%d: ent_attr = '{1'b1, 9'd%d, 9'd%d, 7'd%d, 4'd%d, 1'b%d, 5'h%x, 2'd%d, 12'h%03x, 12'h%03x, 2'd%d, 3'd%d, 1'b%d, 5'd%d, 9'd%d, 1'b%d, 1'b%d, 2'd%d, 1'b%d};  // %s %s\n"
                    % (i, i, ENTRY[e.rt], eop, ec or 0, 1 if ec is not None else 0,
                       ccr, SZC[e.sz], mask0, mask1, e.nfix, IMMK[e.imm],
                       1 if e.priv else 0, jc0, jt0, e.ea0 is not None,
                       e.ea1 is not None, e.fea, e.rt in T0_RT, e.name, e.rt))
        f.write("\t\tdefault: ;\n\tendcase\n\tif (!hit) ent_attr = '0;\nendfunction\n\n")
        # one-hot -> index
        f.write('function automatic logic [8:0] sel_index(input logic [NENT-1:0] s);\n')
        f.write("\tlogic [8:0] x;\n\tx = '0;\n")
        f.write('\tfor (int i = 0; i < NENT; i++) if (s[i]) x = x | 9\'(i);\n')
        f.write('\tsel_index = x;\nendfunction\n')
        # the decode as a whole, for code that wants the old interface
        f.write('function automatic pla_t dec_pla(input logic [15:0] op);\n')
        f.write('\tlogic [NENT-1:0] s;\n\ts = dec_sel(op);\n')
        f.write('\tdec_pla = ent_attr(sel_index(s), |s);\nendfunction\n')
    # expected decode of every operation word for the exhaustive bench:
    # entry index + 1 (0 = illegal) of the hardware's rule (pattern, then EA)
    from isa import bits_match, ea_legal
    idx = {id(e): i for i, e in enumerate(ents)}
    with open(os.path.join(HERE, '..', 'tb', 'build', 'dec_expect.hex'), 'w') as f:
        for op in range(65536):
            e = bits_match(op)
            if e is None:
                f.write('000\n')
            else:
                f.write('%03x\n' % ((idx[id(e)] + 1) | (0x400 if ea_legal(e, op) else 0)))
    print('ucode: %d words of %d bits, %d routines' % (len(ROM), WIDTH, len(ORDER)))


if __name__ == '__main__':
    emit()
