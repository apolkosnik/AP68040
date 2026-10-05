#!/usr/bin/env python3
"""MC68040 instruction table for the AP68040-60 decoder generator.

Every entry is one opcode pattern:

  pat    16 characters, MSB first: '0'/'1' fixed, anything else free
  name   WinUAE mnemonic (readcpu lookuptab), used for the oracle check
  rt     microcode routine name (see ucode.py)
  sz     operation size: 'B' 'W' 'L', 'z' (bits 7:6, 11 excluded),
         'Z' (MOVE, bits 13:12: 01 B, 11 W, 10 L), 'U' unsized
  ea0    allowed modes of the EA in bits 5:0, or None
  ea1    allowed modes of the MOVE destination EA (bits 11:6), or None
  nfix   fixed extension words that follow the operation word
  imm    immediate field kind that follows the fixed words:
           None, 'z' (op size), 'B' (1 word), 'W', 'L',
           'bcc' (8-bit displacement, $00 -> word, $FF -> long),
           'trapcc' (opword bits 2:0: 010 word, 011 long, 100 none),
           'fbcc' (bit 6: 0 word, 1 long)
  priv   privileged (vector 8 in user mode)
  esz    size of an immediate EA (#<data>) if it differs from sz
  fea    fixed EAs: 1 = (Ay)+,(Ax)+ (CMPM), 2 = -(Ay),-(Ax) (ADDX etc.),
         3 = MOVE16 operands

The decoder generator checks this table exhaustively against WinUAE's
per-opcode table (tools-oracle/op040.txt, built from readcpu.cpp and
table68k for cpu_level 4).
"""

# addressing mode index order
MODES = ['Dn', 'An', 'Ai', 'Aip', 'Apd', 'Ad16', 'Ax', 'absW', 'absL',
         'pc16', 'pcx', 'imm']
ALL   = set(MODES)
DATA  = ALL - {'An'}
MEM   = ALL - {'Dn', 'An'}
ALT   = {'Dn', 'An', 'Ai', 'Aip', 'Apd', 'Ad16', 'Ax', 'absW', 'absL'}
DALT  = ALT - {'An'}
MALT  = ALT - {'Dn', 'An'}
CTL   = {'Ai', 'Ad16', 'Ax', 'absW', 'absL', 'pc16', 'pcx'}
CALT  = {'Ai', 'Ad16', 'Ax', 'absW', 'absL'}
DN    = {'Dn'}
AN    = {'An'}


class I:
    def __init__(self, pat, name, rt, sz='U', ea0=None, ea1=None, nfix=0,
                 imm=None, priv=False, esz=None, fea=0):
        pat = pat.replace(' ', '')
        assert len(pat) == 16, pat
        self.pat, self.name, self.rt, self.sz = pat, name, rt, sz
        self.ea0, self.ea1, self.nfix, self.imm = ea0, ea1, nfix, imm
        self.priv, self.esz = priv, esz
        # fixed EAs: 1 (Ay)+,(Ax)+  2 -(Ay),-(Ax)  (registers in 2:0 and 11:9)
        self.fea = fea

    def match(self, op):
        for i, c in enumerate(self.pat):
            b = (op >> (15 - i)) & 1
            if c == '0' and b:
                return False
            if c == '1' and not b:
                return False
        return True


def ea_index(mode, reg):
    if mode < 7:
        return mode
    if reg <= 4:
        return 7 + reg
    return None


def ea_name(mode, reg):
    i = ea_index(mode, reg)
    return None if i is None else MODES[i]


T = []
def add(*a, **k):
    T.append(I(*a, **k))


# ---------------------------------------------------------------- line 0
for z in ('00', '01', '10'):
    # immediate group
    add('0000 0000 %s.. ....' % z, 'OR',   'IMM_OP',  'z', ea0=DALT, imm='z')
    add('0000 0010 %s.. ....' % z, 'AND',  'IMM_OP',  'z', ea0=DALT, imm='z')
    add('0000 0100 %s.. ....' % z, 'SUB',  'IMM_OP',  'z', ea0=DALT, imm='z')
    add('0000 0110 %s.. ....' % z, 'ADD',  'IMM_OP',  'z', ea0=DALT, imm='z')
    add('0000 1010 %s.. ....' % z, 'EOR',  'IMM_OP',  'z', ea0=DALT, imm='z')
    add('0000 1100 %s.. ....' % z, 'CMP',  'IMM_CMP', 'z', ea0=DATA - {'imm'}, imm='z')
    add('0000 1110 %s.. ....' % z, 'MOVES', 'MOVES',  'z', ea0=MALT, nfix=1, priv=True)
add('0000 0000 0011 1100', 'ORSR',  'CCR_LOG', 'B', imm='B')
add('0000 0000 0111 1100', 'ORSR',  'SR_LOG',  'W', imm='W', priv=True)
add('0000 0010 0011 1100', 'ANDSR', 'CCR_LOG', 'B', imm='B')
add('0000 0010 0111 1100', 'ANDSR', 'SR_LOG',  'W', imm='W', priv=True)
add('0000 1010 0011 1100', 'EORSR', 'CCR_LOG', 'B', imm='B')
add('0000 1010 0111 1100', 'EORSR', 'SR_LOG',  'W', imm='W', priv=True)
# CHK2/CMP2 (size in bits 10:9)
add('0000 0000 11.. ....', 'CHK2', 'CHK2', 'B', ea0=CTL, nfix=1)
add('0000 0010 11.. ....', 'CHK2', 'CHK2', 'W', ea0=CTL, nfix=1)
add('0000 0100 11.. ....', 'CHK2', 'CHK2', 'L', ea0=CTL, nfix=1)
# CAS / CAS2
add('0000 1010 11.. ....', 'CAS', 'CAS', 'B', ea0=MALT, nfix=1)
add('0000 1100 11.. ....', 'CAS', 'CAS', 'W', ea0=MALT, nfix=1)
add('0000 1110 11.. ....', 'CAS', 'CAS', 'L', ea0=MALT, nfix=1)
add('0000 1100 1111 1100', 'CAS2', 'CAS2', 'W', nfix=2)
add('0000 1110 1111 1100', 'CAS2', 'CAS2', 'L', nfix=2)
# static bit operations: bit number word, then the EA
add('0000 1000 00.. ....', 'BTST', 'BTST_IMM', 'B', ea0=DATA - {'imm'}, imm='B')
add('0000 1000 01.. ....', 'BCHG', 'BIT_IMM', 'B', ea0=DALT, imm='B')
add('0000 1000 10.. ....', 'BCLR', 'BIT_IMM', 'B', ea0=DALT, imm='B')
add('0000 1000 11.. ....', 'BSET', 'BIT_IMM', 'B', ea0=DALT, imm='B')
# dynamic bit operations and MOVEP (MOVEP is the An mode of this group)
add('0000 ...1 00.. ....', 'BTST', 'BTST_DYN', 'B', ea0=DATA)
add('0000 ...1 01.. ....', 'BCHG', 'BIT_DYN', 'B', ea0=DALT)
add('0000 ...1 10.. ....', 'BCLR', 'BIT_DYN', 'B', ea0=DALT)
add('0000 ...1 11.. ....', 'BSET', 'BIT_DYN', 'B', ea0=DALT)
add('0000 ...1 0000 1...', 'MVPMR', 'MOVEP_MR', 'W', imm='W')
add('0000 ...1 0100 1...', 'MVPMR', 'MOVEP_MR', 'L', imm='W')
add('0000 ...1 1000 1...', 'MVPRM', 'MOVEP_RM', 'W', imm='W')
add('0000 ...1 1100 1...', 'MVPRM', 'MOVEP_RM', 'L', imm='W')

# ---------------------------------------------------------------- MOVE
add('0001 .... .... ....', 'MOVE',  'MOVE',  'B', ea0=ALL - {'An'}, ea1=DALT)
add('0011 .... .... ....', 'MOVE',  'MOVE',  'W', ea0=ALL, ea1=DALT)
add('0010 .... .... ....', 'MOVE',  'MOVE',  'L', ea0=ALL, ea1=DALT)
add('0011 ...0 01.. ....', 'MOVEA', 'MOVEA', 'W', ea0=ALL)
add('0010 ...0 01.. ....', 'MOVEA', 'MOVEA', 'L', ea0=ALL)

# ---------------------------------------------------------------- line 4
for z in ('00', '01', '10'):
    add('0100 0000 %s.. ....' % z, 'NEGX', 'UNARY', 'z', ea0=DALT)
    add('0100 0010 %s.. ....' % z, 'CLR',  'UNARY', 'z', ea0=DALT)
    add('0100 0100 %s.. ....' % z, 'NEG',  'UNARY', 'z', ea0=DALT)
    add('0100 0110 %s.. ....' % z, 'NOT',  'UNARY', 'z', ea0=DALT)
    add('0100 1010 %s.. ....' % z, 'TST',  'TST',   'z', ea0=ALL - ({'An'} if z == '00' else set()))
add('0100 0000 11.. ....', 'MVSR2', 'MOVE_FROM_SR', 'W', ea0=DALT, priv=True)
add('0100 0010 11.. ....', 'MVSR2', 'MOVE_FROM_CCR', 'W', ea0=DALT)
add('0100 0100 11.. ....', 'MV2SR', 'MOVE_TO_CCR', 'W', ea0=DATA)
add('0100 0110 11.. ....', 'MV2SR', 'MOVE_TO_SR', 'W', ea0=DATA, priv=True)
add('0100 1000 0000 1...', 'LINK', 'LINK', 'L', imm='L')
add('0100 1000 00.. ....', 'NBCD', 'UNARY', 'B', ea0=DALT)
add('0100 1000 0100 1...', 'BKPT', 'BKPT', 'U')
add('0100 1000 0100 0...', 'SWAP', 'SWAP', 'W')
add('0100 1000 01.. ....', 'PEA',  'PEA',  'L', ea0=CTL)
add('0100 1000 1000 0...', 'EXT',  'EXT',  'W')
add('0100 1000 1100 0...', 'EXT',  'EXT',  'L')
add('0100 1001 1100 0...', 'EXT',  'EXTB', 'L')
add('0100 1000 10.. ....', 'MVMLE', 'MOVEM_RM', 'W', ea0=CALT | {'Apd'}, nfix=1)
add('0100 1000 11.. ....', 'MVMLE', 'MOVEM_RM', 'L', ea0=CALT | {'Apd'}, nfix=1)
add('0100 1100 10.. ....', 'MVMEL', 'MOVEM_MR', 'W', ea0=CTL | {'Aip'}, nfix=1)
add('0100 1100 11.. ....', 'MVMEL', 'MOVEM_MR', 'L', ea0=CTL | {'Aip'}, nfix=1)
add('0100 1010 11.. ....', 'TAS',  'TAS', 'B', ea0=DALT)
add('0100 1010 1111 1100', 'ILLG', 'ILLEGAL', 'U')
add('0100 1100 00.. ....', 'MULL', 'MULL', 'L', ea0=DATA, nfix=1)
add('0100 1100 01.. ....', 'DIVL', 'DIVL', 'L', ea0=DATA, nfix=1)
add('0100 1110 0100 ....', 'TRAP', 'TRAP', 'U')
add('0100 1110 0101 0...', 'LINK', 'LINK', 'W', imm='W')
add('0100 1110 0101 1...', 'UNLK', 'UNLK', 'L')
add('0100 1110 0110 0...', 'MVR2USP', 'MOVE_TO_USP', 'L', priv=True)
add('0100 1110 0110 1...', 'MVUSP2R', 'MOVE_FROM_USP', 'L', priv=True)
add('0100 1110 0111 0000', 'RESET', 'RESET', 'U', priv=True)
add('0100 1110 0111 0001', 'NOP',   'NOP',   'U')
add('0100 1110 0111 0010', 'STOP',  'STOP',  'U', imm='W', priv=True)
add('0100 1110 0111 0011', 'RTE',   'RTE',   'U', priv=True)
add('0100 1110 0111 0100', 'RTD',   'RTD',   'U', imm='W')
add('0100 1110 0111 0101', 'RTS',   'RTS',   'U')
add('0100 1110 0111 0110', 'TRAPV', 'TRAPV', 'U')
add('0100 1110 0111 0111', 'RTR',   'RTR',   'U')
add('0100 1110 0111 1010', 'MOVEC2', 'MOVEC_RD', 'L', nfix=1, priv=True)
add('0100 1110 0111 1011', 'MOVE2C', 'MOVEC_WR', 'L', nfix=1, priv=True)
add('0100 1110 10.. ....', 'JSR', 'JSR', 'U', ea0=CTL)
add('0100 1110 11.. ....', 'JMP', 'JMP', 'U', ea0=CTL)
add('0100 ...1 00.. ....', 'CHK', 'CHK', 'L', ea0=DATA)
add('0100 ...1 10.. ....', 'CHK', 'CHK', 'W', ea0=DATA)
add('0100 ...1 11.. ....', 'LEA', 'LEA', 'L', ea0=CTL)

# ---------------------------------------------------------------- line 5
for z in ('00', '01', '10'):
    add('0101 ...0 %s.. ....' % z, 'ADD', 'QUICK', 'z', ea0=DALT)
    add('0101 ...1 %s.. ....' % z, 'SUB', 'QUICK', 'z', ea0=DALT)
add('0101 ...0 0100 1...', 'ADDA', 'QUICK_A', 'W')
add('0101 ...0 1000 1...', 'ADDA', 'QUICK_A', 'L')
add('0101 ...1 0100 1...', 'SUBA', 'QUICK_SA', 'W')
add('0101 ...1 1000 1...', 'SUBA', 'QUICK_SA', 'L')
add('0101 .... 11.. ....', 'Scc', 'SCC', 'B', ea0=DALT)
add('0101 .... 1100 1...', 'DBcc', 'DBCC', 'W', imm='W')
add('0101 .... 1111 1010', 'TRAPcc', 'TRAPCC', 'U', imm='trapcc')
add('0101 .... 1111 1011', 'TRAPcc', 'TRAPCC', 'U', imm='trapcc')
add('0101 .... 1111 1100', 'TRAPcc', 'TRAPCC', 'U', imm='trapcc')

# ---------------------------------------------------------------- line 6
add('0110 0001 .... ....', 'BSR', 'BSR', 'U', imm='bcc')
add('0110 .... .... ....', 'Bcc', 'BCC', 'U', imm='bcc')

# ---------------------------------------------------------------- MOVEQ
add('0111 ...0 .... ....', 'MOVE', 'MOVEQ', 'L')

# ---------------------------------------------------------------- line 8, 9, B, C, D
for z in ('00', '01', '10'):
    add('1000 ...0 %s.. ....' % z, 'OR',  'OP_EA_DN', 'z', ea0=DATA)
    add('1000 ...1 %s.. ....' % z, 'OR',  'OP_DN_EA', 'z', ea0=MALT)
    add('1100 ...0 %s.. ....' % z, 'AND', 'OP_EA_DN', 'z', ea0=DATA)
    add('1100 ...1 %s.. ....' % z, 'AND', 'OP_DN_EA', 'z', ea0=MALT)
    add('1001 ...0 %s.. ....' % z, 'SUB', 'OP_EA_DN', 'z', ea0=ALL - ({'An'} if z == '00' else set()))
    add('1001 ...1 %s.. ....' % z, 'SUB', 'OP_DN_EA', 'z', ea0=MALT)
    add('1101 ...0 %s.. ....' % z, 'ADD', 'OP_EA_DN', 'z', ea0=ALL - ({'An'} if z == '00' else set()))
    add('1101 ...1 %s.. ....' % z, 'ADD', 'OP_DN_EA', 'z', ea0=MALT)
    add('1011 ...0 %s.. ....' % z, 'CMP', 'CMP_EA_DN', 'z', ea0=ALL - ({'An'} if z == '00' else set()))
    add('1011 ...1 %s.. ....' % z, 'EOR', 'OP_DN_EA', 'z', ea0=DALT)
    add('1011 ...1 %s00 1...' % z, 'CMPM', 'CMPM', 'z', fea=1)
    add('1001 ...1 %s00 0...' % z, 'SUBX', 'ADDX_R', 'z')
    add('1001 ...1 %s00 1...' % z, 'SUBX', 'ADDX_M', 'z', fea=2)
    add('1101 ...1 %s00 0...' % z, 'ADDX', 'ADDX_R', 'z')
    add('1101 ...1 %s00 1...' % z, 'ADDX', 'ADDX_M', 'z', fea=2)
add('1000 ...0 11.. ....', 'DIVU', 'DIVW', 'W', ea0=DATA)
add('1000 ...1 11.. ....', 'DIVS', 'DIVW', 'W', ea0=DATA)
add('1100 ...0 11.. ....', 'MULU', 'MULW', 'W', ea0=DATA)
add('1100 ...1 11.. ....', 'MULS', 'MULW', 'W', ea0=DATA)
add('1000 ...1 0000 0...', 'SBCD', 'ADDX_R', 'B')
add('1000 ...1 0000 1...', 'SBCD', 'ADDX_M', 'B', fea=2)
add('1100 ...1 0000 0...', 'ABCD', 'ADDX_R', 'B')
add('1100 ...1 0000 1...', 'ABCD', 'ADDX_M', 'B', fea=2)
add('1000 ...1 0100 0...', 'PACK', 'PACK_R', 'U', imm='W')
add('1000 ...1 0100 1...', 'PACK', 'PACK_M', 'U', imm='W', fea=2)
add('1000 ...1 1000 0...', 'UNPK', 'UNPK_R', 'U', imm='W')
add('1000 ...1 1000 1...', 'UNPK', 'UNPK_M', 'U', imm='W', fea=2)
add('1001 ...0 11.. ....', 'SUBA', 'SUBA', 'W', ea0=ALL)
add('1001 ...1 11.. ....', 'SUBA', 'SUBA', 'L', ea0=ALL)
add('1101 ...0 11.. ....', 'ADDA', 'ADDA', 'W', ea0=ALL)
add('1101 ...1 11.. ....', 'ADDA', 'ADDA', 'L', ea0=ALL)
add('1011 ...0 11.. ....', 'CMPA', 'CMPA', 'W', ea0=ALL)
add('1011 ...1 11.. ....', 'CMPA', 'CMPA', 'L', ea0=ALL)
add('1100 ...1 0100 0...', 'EXG', 'EXG_DD', 'L')
add('1100 ...1 0100 1...', 'EXG', 'EXG_AA', 'L')
add('1100 ...1 1000 1...', 'EXG', 'EXG_DA', 'L')

# ---------------------------------------------------------------- line E
SH = {'00': 'AS', '01': 'LS', '10': 'ROX', '11': 'RO'}
# register shifts: 1110 ccc d zz i tt rrr (type tt straddles bits 4:3)
for z in ('00', '01', '10'):
    for k, n in SH.items():
        for d, dn in (('0', 'R'), ('1', 'L')):
            add('1110...%s%s.%s%s...' % (d, z, k[0], k[1]), n + dn, 'SHIFT_R', 'z')
for k, n in SH.items():
    add('1110 0%s0 11.. ....' % k, n + 'RW', 'SHIFT_M', 'W', ea0=MALT)
    add('1110 0%s1 11.. ....' % k, n + 'LW', 'SHIFT_M', 'W', ea0=MALT)
BFR = DN | CTL
BFW = DN | CALT
add('1110 1000 11.. ....', 'BFTST',  'BF_TST', 'U', ea0=BFR, nfix=1)
add('1110 1001 11.. ....', 'BFEXTU', 'BF_EXT', 'U', ea0=BFR, nfix=1)
add('1110 1010 11.. ....', 'BFCHG',  'BF_MOD', 'U', ea0=BFW, nfix=1)
add('1110 1011 11.. ....', 'BFEXTS', 'BF_EXT', 'U', ea0=BFR, nfix=1)
add('1110 1100 11.. ....', 'BFCLR',  'BF_MOD', 'U', ea0=BFW, nfix=1)
add('1110 1101 11.. ....', 'BFFFO',  'BF_EXT', 'U', ea0=BFR, nfix=1)
add('1110 1110 11.. ....', 'BFSET',  'BF_MOD', 'U', ea0=BFW, nfix=1)
add('1110 1111 11.. ....', 'BFINS',  'BF_MOD', 'U', ea0=BFW, nfix=1)

# ---------------------------------------------------------------- line F
add('1111 0010 00.. ....', 'FPP',     'FPU_GEN', 'U', ea0=ALL, nfix=1)
add('1111 0010 01.. ....', 'FScc',    'FSCC',    'B', ea0=DALT, nfix=1)
add('1111 0010 0100 1...', 'FDBcc',   'FDBCC',   'W', nfix=1, imm='W')
add('1111 0010 0111 1010', 'FTRAPcc', 'FTRAPCC', 'U', nfix=1, imm='trapcc')
add('1111 0010 0111 1011', 'FTRAPcc', 'FTRAPCC', 'U', nfix=1, imm='trapcc')
add('1111 0010 0111 1100', 'FTRAPcc', 'FTRAPCC', 'U', nfix=1, imm='trapcc')
add('1111 0010 10.. ....', 'FBcc',    'FBCC',    'U', imm='fbcc')
add('1111 0010 11.. ....', 'FBcc',    'FBCC',    'U', imm='fbcc')
add('1111 0011 00.. ....', 'FSAVE',    'FSAVE',    'U', ea0=CALT | {'Apd'}, priv=True)
add('1111 0011 01.. ....', 'FRESTORE', 'FRESTORE', 'U', ea0=CTL | {'Aip'}, priv=True)
for p in ('00', '01', '10', '11'):
    add('1111 0100 %s00 1...' % p, 'CINVL',  'CACHE_OP', 'U', priv=True)
    add('1111 0100 %s01 0...' % p, 'CINVP',  'CACHE_OP', 'U', priv=True)
    add('1111 0100 %s01 1...' % p, 'CINVA',  'CACHE_OP', 'U', priv=True)
    add('1111 0100 %s10 1...' % p, 'CPUSHL', 'CACHE_OP', 'U', priv=True)
    add('1111 0100 %s11 0...' % p, 'CPUSHP', 'CACHE_OP', 'U', priv=True)
    add('1111 0100 %s11 1...' % p, 'CPUSHA', 'CACHE_OP', 'U', priv=True)
add('1111 0101 0000 0...', 'PFLUSHN',  'PFLUSH', 'U', priv=True)
add('1111 0101 0000 1...', 'PFLUSH',   'PFLUSH', 'U', priv=True)
add('1111 0101 0001 0...', 'PFLUSHAN', 'PFLUSH', 'U', priv=True)
add('1111 0101 0001 1...', 'PFLUSHA',  'PFLUSH', 'U', priv=True)
add('1111 0101 0100 1...', 'PTESTW',   'PTEST',  'U', priv=True)
add('1111 0101 0110 1...', 'PTESTR',   'PTEST',  'U', priv=True)
# MOVE16: fea=3, D1 builds (Ay)+/(Ay)/abs.L operands; one routine
add('1111 0110 0000 0...', 'MOVE16', 'MOVE16', 'U', imm='L', fea=3)
add('1111 0110 0000 1...', 'MOVE16', 'MOVE16', 'U', imm='L', fea=3)
add('1111 0110 0001 0...', 'MOVE16', 'MOVE16', 'U', imm='L', fea=3)
add('1111 0110 0001 1...', 'MOVE16', 'MOVE16', 'U', imm='L', fea=3)
add('1111 0110 0010 0...', 'MOVE16', 'MOVE16', 'U', nfix=1, fea=3)


def bits_match(op):
    """The entry whose bit pattern is the most specific match (this is the
    hardware's priority casez), ignoring EA legality."""
    best = None
    for e in T:
        if not e.match(op):
            continue
        if e.sz == 'z' and ((op >> 6) & 3) == 3:
            continue
        fixed = sum(c in '01' for c in e.pat)
        assert best is None or fixed != best[0] or best[1] is e, \
            'equal-specificity overlap %04x' % op
        if best is None or fixed > best[0]:
            best = (fixed, e)
    return None if best is None else best[1]


def ea_legal(e, op):
    if e.ea0 is not None:
        n = ea_name((op >> 3) & 7, op & 7)
        if n is None or n not in e.ea0:
            return False
    if e.ea1 is not None:
        n = ea_name((op >> 6) & 7, (op >> 9) & 7)
        if n is None or n not in e.ea1:
            return False
    return True


def lookup(op):
    """Decode exactly as the hardware does: most specific bit pattern, then
    that entry's EA legality; None is an illegal/unimplemented opcode."""
    e = bits_match(op)
    if e is None or not ea_legal(e, op):
        return None
    return e
