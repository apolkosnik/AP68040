# AP68040-60: a pipelined MC68040 for 60 MHz

This branch (`ap68040-60mhz`) holds a new implementation of the MC68040,
written for a 60 MHz processor clock on Cyclone V (MiSTer, 5CSEBA6U23I7,
speed grade I7), validated under Verilator.  The previous sequential core
is kept unchanged in `rtl_old/` (with its benches in `tb_old/`) as a
reference model for differential testing.

Architectural ground truth, in order of authority:
1. Real hardware behaviour where it is known (recorded in tests).
2. WinUAE (`newcpu*.cpp`, `cpummu.cpp`, `gencpu.cpp`, `table68k`) and its
   cputest 68040 corpus.
3. MC68040 User's Manual (MC68040UM), sections 3 (MMU), 4 (caches),
   7 (bus), 8 (exceptions), 9 (FPU).

## Clocking

One clock, `clk`, is the 68040's PCLK.  The bus clock is expressed as a
clock enable, `bclk_en`: every bus output changes, and every bus input is
sampled, only in a `clk` cycle with `bclk_en` high.  `bclk_en = 1` runs the
bus at the processor clock; a 1-0-1-0 pattern gives the real 68040's 2:1
PCLK:BCLK ratio.

## External bus (MC68040UM section 7)

The top level, `ap68040`, presents the 68040 bus as synchronous signals with
separate in/out/output-enable where the real pin is bidirectional:

| group | signals |
|---|---|
| address/data | `a_o[31:0]`, `d_i[31:0]`, `d_o[31:0]`, `d_oe` |
| attributes | `rw_n`, `siz[1:0]`, `tt[1:0]`, `tm[2:0]`, `tln[1:0]`, `upa[1:0]`, `ciout_n`, `lock_n`, `locke_n` |
| control | `ts_n`, `tip_n`, `ta_n`, `tea_n`, `tci_n`, `tbi_n` |
| snoop | `sc[1:0]`, `mi_n`, plus the alternate master's `a_i`, `siz_i`, `rw_n_i`, `ts_n_i` |
| arbitration | `br_n`, `bg_n`, `bb_n_i`, `bb_n_o` |
| interrupts | `ipl_n[2:0]`, `avec_n`, `ipend_n` |
| status | `pst[3:0]`, `rsti_n`, `rsto_n` |

Supported transfers: byte/word/long and misaligned splits (Table 7-3), line
burst read/write with wrap-around addressing, burst-inhibit (TBI) fallback to
three long-word transfers, cache-inhibit-on-fill (TCI), bus error (TEA),
retry (TA+TEA), locked read-modify-write (`LOCK`/`LOCKE`) for TAS, CAS,
CAS2 and descriptor U/M updates, MOVE16 line transfers (TT=1), alternate
space (TT=2, MOVES), interrupt acknowledge (TT=3, AVEC autovector), table
search (TM=3/4), push (TM=0) and snooping of alternate masters.

## Pipeline

```
 F0  F1  F2 | D1  D2 | AG  DC1  DC2  EX  WB
 PC  I$  Q  | dec seq| RR+EA D$  D$   ALU commit
```

* **F0** selects the fetch PC (sequential, decode redirect, branch-target
  buffer hit, execute redirect, exception vector).  **F1** reads the
  instruction cache (4 KB, 4-way, 64 sets of 16-byte lines) and the I-ATC.
  **F2** selects the way and writes 8 bytes into the instruction queue.
* **D1** parses one instruction from the queue head: operation word, the
  fixed second word if the class has one, immediates and the extension
  words of each effective address (brief and full formats, base and outer
  displacements).  Common instructions take one cycle; long ones take one
  cycle per extension group.  D1 resolves unconditional and predicted-taken
  PC-relative branches and redirects fetch.
* **D2** is the micro-sequencer.  Each 68040 instruction maps (through a
  PLA generated from the instruction table) to a microcode routine of one
  or more micro-operations (uops).  Most instructions are one uop: a fused
  `load-op-store` pass through the back end.  Multi-uop routines cover
  memory-to-memory forms, memory-indirect addressing, MOVEM, bit fields,
  CAS/CAS2, exceptions, RTE, MOVE16, FPU transfers and MMU instructions.
* **AG** reads the register file (a future file, see below), forwards, and
  computes the effective address `base + index*scale + disp` and the
  postincrement/predecrement update of the base register.
* **DC1/DC2** look up the D-ATC and the data cache (4 KB, 4-way, copyback),
  select the way and align the operand.
* **EX** runs the ALU, shifter, bit-field unit, BCD, condition codes and
  branch resolution; multiply/divide and the FPU are multi-cycle units
  that hold EX.
* **WB** commits: architectural register file, CCR/SR, store into the data
  cache or the bus, exceptions, interrupts and trace.

### Timing rules

1. Every stall and flush is a function of at most two levels of logic over
   registered state.  Data-dependent conditions (cache miss, ATC miss,
   misaligned split, store/load overlap) are registered in the stage that
   detects them and act in the next cycle.
2. Block RAMs (M10K) have registered addresses; a stalled stage keeps its
   address so the RAM re-reads it every cycle.
3. Long arithmetic (divide, FPU divide/square root, 64-bit multiply) is
   iterative or pipelined, never a single-cycle cone.

### Register files

Integer registers are D0-D7, A0-A6, USP, ISP, MSP and eight microcode
temporaries.  `A7` is mapped to USP/ISP/MSP at decode time from S and M;
every instruction that changes S or M serializes the pipeline.

* The **front file** holds the newest value of every register.  AG writes
  postincrement/predecrement updates into it; EX writes results into it.
  AG reads it (with bypass of the values being written in the same cycle).
* The **back file** is written by WB in program order and is the
  architectural state.  A flush (mispredict, exception, serialization)
  copies the back file into the front file in one cycle.
* A scoreboard marks registers with a pending EX write.  AG stalls when the
  base or index register is pending, and when it would write a register an
  older instruction will still write in EX.  EX operands do not stall:
  instructions in AG/DC1/DC2 capture an EX result for their source
  registers when it is broadcast.

### Memory ordering

Loads read the data cache in DC1.  Stores write it in WB.  A load whose
physical long word overlaps an older store still in EX or WB, or written by
WB in the cycle the load read the RAM, is replayed: it and everything
younger are flushed and refetched.  Noncachable accesses go to the bus in
program order; serialized noncachable reads wait for older writes.

## Caches (section 4)

Both caches are 4 KB, 4-way set associative, 64 sets, 16-byte lines,
physically tagged and indexed by address bits 9-4 (inside the page, so no
aliasing).  Valid and dirty bits are flip-flops; tags and data are M10K.

* Data cache modes from the ATC/TTR CM field: copyback, write-through,
  cache-inhibited serialized, cache-inhibited non-serialized.
* Read miss: line fill by burst (critical long word first, wrapping), with
  the requested operand forwarded from the bus.  A dirty victim is pushed
  (line write, TM=0) through the push buffer.
* Write miss in copyback mode allocates (read line, then write); in
  write-through it does not allocate.
* `CINV`/`CPUSH` (line, page, all; data, instruction, both).
* Snooping: an alternate master's write invalidates matching lines (SC=01)
  or is sunk into a dirty line (SC=10); a read of a dirty line is sourced
  with MI asserted.

## MMU (section 3)

* I-ATC and D-ATC: 64 entries each, 4-way, 16 sets, indexed by logical page
  number bits, tagged with the logical page, S, and the global bit.
* 4 KB and 8 KB pages, three-level tables (root, pointer, page; indirect
  page descriptors), U and M history updates with locked read-modify-write,
  write protection, supervisor-only pages, U1/U0 (UPA pins), CM.
* ITT0/1 and DTT0/1 transparent translation with FC matching and S field.
* Following WinUAE, the 68040 creates an ATC entry for an invalid or
  bus-errored descriptor (an R-clear entry); a later access through it
  faults without a search until PFLUSH.
* PTEST (R/W) writes MMUSR and installs the entry; PFLUSH (page, page with
  no global, all, all non-global).
* Access faults produce a format $7 frame with the 68040's SSW, effective
  address, and fault address; instruction restart (not continuation).

## Exceptions (section 8)

Detected per uop, taken when the uop reaches WB (precise).  The back end
flushes, restores the front file and starts the exception microroutine,
which builds format $0/$1/$2/$3/$4/$7 frames with ordinary store uops
sourcing the exception-information registers (vector, old SR, PC, faulted
address, SSW).  Interrupts and trace are taken at instruction boundaries.

## FPU (section 9)

The 68040 hardware subset, as in `rtl_old/ap040_fpu.v` (verified against
the cputest corpus) with its divide/square-root step retimed for 60 MHz.
Unimplemented instructions and data types trap with the 68040 frames an
FPSP expects.

## Source layout

```
rtl/        new core (SystemVerilog)
rtl/gen/    generated tables (decoder PLA, microcode ROM) - do not edit
tools/      generators for rtl/gen (instruction table, microcode assembler)
tb/         Verilator benches, 68040 bus model, test runners
tb/asm/     self-checking 68040 test programs (shared with tb_old)
rtl_old/    previous sequential core (reference only)
tb_old/     its benches
doc/        this document, STATUS.md
```
