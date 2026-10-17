# AP68040 `faster`: a pipelined MC68040 for 60 MHz under 20,000 ALMs

This branch holds an implementation of the MC68040 written for a 60 MHz
processor clock on Cyclone V (MiSTer, 5CSEBA6U23I7), validated under
Verilator, with the ALM count of the whole core as a design goal.  It was
made from the AP68060 tree (a 68060 built on this repository's
`ap68040-60mhz` branch and tuned there for area and timing), converted back
to the 68040's programming model, caches, MMU and bus, keeping the pipeline
work of that round.  The previous sequential core is kept in `rtl_old/`
(benches in `tb_old/`) as a reference.

Architectural ground truth, in order of authority:
1. Real hardware behaviour where it is known (recorded in tests).
2. WinUAE (`newcpu*.cpp`, `cpummu.cpp`, `gencpu.cpp`, `softfloat/`) and its
   cputest 68040 corpus, replayed by `tb/tb_cputest.sv`.
3. The MC68040 User's Manual (MC68040UM): section 3 (MMU), 4 (caches),
   5 (signals), 7 (bus), 8 (exceptions), 9 (FPU).

Where WinUAE and the manual disagree and a test pins WinUAE, WinUAE wins
(MOVES through the FC2-selected root for every function code; INEX2 on an
overflowing store only for lost bits); where no test pins anything, the
manual was followed (STATUS.md lists the decisions).

## Configuration

`ap68040`'s parameters (tb/tb_ap68040.sv sets them from `AP_*` defines;
bit 0 of the bench's $F2E8 tells a program whether it has the loop
predictor):

| parameter | default | what it selects |
|---|---|---|
| `FPU_REVISION` | $41 | the FSAVE frame revision ($40 for the other mask set) |
| `FPU_PIPE` | 0 | 1: the arithmetic pipeline beside the FPU state machine |
| `LOOP_PRED` | 0 | 1: the loop-exit predictor (the history table and the BTB remain) |
| `EARLY_RES` | 0 | 1: early results in AG (a base or index need not wait for EX) |
| `DC1_ATC` | 1 | DC1's own ATC read port and parallel tag compare (timing) |
| `SKID` | 0 | 1: a skid register between D2 and AG |
| `DIV_STEPS` | 2 | quotient bits per clock of the integer divider (2 or 4) |

What each costs in ALMs and in cycles is measured in STATUS.md.  The
defaults are the configuration the 20,000-ALM goal is measured on.

## Clocking

One clock, `clk`, is the 68040's PCLK.  The bus clock is a clock enable,
`bclk_en`: every bus output changes, and every bus input is sampled, only in a
`clk` cycle with `bclk_en` high.  `bclk_en = 1` runs the bus at the processor
clock; 1-0-1-0 gives the real 68040's 2:1 PCLK:BCLK ratio.

## External bus (MC68040UM sections 5 and 7)

| group | signals |
|---|---|
| address, data | `a_o`, `a_oe`, `d_i`, `d_o`, `d_oe` |
| attributes | `rw_n`, `siz`, `tt`, `tm`, `tln`, `upa`, `ciout_n`, `lock_n`, `locke_n` |
| transfer control | `ts_n`, `tip_n`, `ta_n`, `tea_n`, `tci_n`, `tbi_n` |
| snooping | `sc`, `mi_n`; the other master's `a_i`, `ts_n_i`, `rw_n_i`, `siz_i`, `tt_i`; the 68040 as a slave: `ta_n_o`, `ta_oe` (and `d_o`/`d_oe`) |
| arbitration | `br_n`, `bg_n`, `bb_n_i`, `bb_n_o`, `bb_oe` |
| interrupts | `ipl_n`, `avec_n`, `ipend_n` |
| control, status | `rsti_n`, `rsto_n`, `cdis_n`, `mdis_n`, `pst` |

`ap68040_biu.sv` runs one transaction at a time for its clients (the DMU
first, then the instruction fetch): byte/word/long transfers (the requester
splits misaligned operands, table 7-3), line bursts with A3:A2 wrapping,
TBI fallback to long-word transfers, TCI on fills (the first beat decides,
4.5.1), TEA bus errors, TA+TEA retry (a retry after the first beat of a
line is a bus error; a retry with BG negated gives the bus up and reruns
the transfer), locked sequences (LOCK/LOCKE) that keep the bus, MOVE16
(TT=1), MOVES alternate spaces (TT=2, TM the function code), interrupt
and breakpoint acknowledge (TT=3: TM the level, or TM=0 at address 0 for
BKPT, 7.5), table searches (TM=3/4), pushes (TM=0), and RSTO for 512 bus
clocks once the transfer in progress has ended.  Arbitration (7.8.1): BR
is asserted while a transfer is wanted, the bus is taken when BG is
asserted and BB is free; a negated BG gives it up after the transfer in
progress unless a locked sequence is open.  Reset: RSTI is synchronized
over two bus clocks and holds the processor in reset for 128 bus clocks
after it negates (figure C-2); any bus cycle in progress ends as if
terminated.  IPL2-IPL0 are taken only when two consecutive bus-clock
samples agree (7.8); IPEND negates when the request is recognized
internally.  A double bus fault (an access or address error during the
processing of an access error, an address error or a reset) halts the
processor: PST $5, BR negated, no bus cycle, until RSTI (7.6.3).

Not provided: the multiplexed bus mode CDIS selects at reset, DLE, JTAG.

## Pipeline

```
 F0  F1  F2  F3 | D1    D2   | AG   DC1  DC2  EX   WB
 PC  I$  tbl pre| parse useq | EA   D$   D$   ALU  commit
```

* **F0** issues one long-word fetch per cycle: I-ATC/ITT lookup, the cache
  RAM read, and the branch target buffer lookup.  **F1** compares the tags;
  a hit goes on, a miss starts the miss engine (table walk through the DMU,
  line fill with critical word first, or a cache-inhibited read) and F0
  rolls back to the missed address.  **F2** reads the predecoder table for
  both words of the chunk (each could start an instruction): a block RAM
  of 8192 words, one per operation word bits 15:3, with two read ports
  (`ap68040_pdrom`, 32 M10Ks, generated with the microcode).  **F3** forms
  each word's predecode from its table word and the word itself
  (`ap68040_predec`: entry, size, length assuming brief index extensions,
  EA legality; where the register field decides the entry too -- ORI/ANDI/
  EORI to CCR/SR, CAS2, ILLEGAL, TRAPcc, FTRAPcc, the $4E7x group -- the
  table word names an alternate from a 26-entry table in logic) and
  appends them to the 8-word queue: the words shift; the predecode sits in
  a ring of two LUT RAM banks (even and odd slots) whose head D1 reads from
  a register, loaded a cycle ahead as D1 consumes.  The parallel pattern
  decoder the table replaced (`ap68040_predec_pla`, `rtl/gen/
  ap68040_dec_pla.svh`) stays as the simulation reference: every predecode
  is checked against it, and `tb/build_decode.sh` runs the exhaustive
  bench over all 65,536 operation words.
* **D1** parses one instruction from the queue head per cycle (the fast
  path, from the predecode: operation word, fixed words, immediate, brief
  extensions; the operation word's decode attributes come from a 512-entry
  ROM, M10K, addressed by the entry number the predecode gives the word
  that heads the queue next cycle, so D1 starts from a register); a
  full-format extension, an FPU immediate or more than seven words take
  one cycle per part.  D1 adds PC-relative displacements and branch
  targets, predicts branches and redirects fetch (see below), and turns
  illegal/privileged/TRAP/odd-PC/fetch-fault cases into records that carry
  their exception vector.  Records go to D2 through a FIFO of two, which
  has room in the clock the sequencer pops a record (`rq_pop_e`, the early
  form of the pop: a plain last microcode word with no memory indirection)
  -- D1 would otherwise wait a clock behind every pop.
* **D2** is the micro-sequencer (`ap68040_useq.sv`).  Each instruction maps
  to a microcode routine (652 words of 120 bits, M10K, generated by
  `tools/ucode.py`) of one or more uops, one uop per cycle, into a FIFO of
  two uops that the back end pops (both outcomes of the pop are formed and
  the pop selects last: the back end's stall stays off the sequencer's
  step).  Most instructions are one uop: a `load-op-store` pass through the
  back end.  Multi-uop routines cover memory-to-memory forms, memory
  indirection (a pointer load into T12/T13 first), MOVEM, bit fields,
  CAS/CAS2, CHK2/CMP2, MOVEP, 64-bit multiply and divide, RTE (format $7
  included), MOVE16, FPU transfers, MMU and cache instructions and the
  exception routines (format $0, $2 and $7 frames, interrupts with and
  without the master stack, BKPT's acknowledge cycle).  A7 maps to USP,
  ISP or MSP by S and M.  A source (An)+/-(An) can be deferred to a later
  uop so an instruction that faults on its second access restarts cleanly.
  D2 chains from one routine's last word straight into the next record's
  first, so single-uop instructions issue one a cycle.  JSR to an absolute
  or PC-relative address is one uop, as BSR (D1 computed the target).
  While the back end prepares an exception entry (`exc_pend`) the
  sequencer issues nothing.
* **AG** reads the front register file and forms `base + index*scale +
  disp`, the address register updates (into the front file) and the
  operands.  Base, index and update registers interlock on an older uop
  that will write them in EX.  A plain load (long MOVE, MOVEA.W, a
  memory-indirect pointer) hands its data to a waiting base or index as it
  leaves DC2, a cycle before EX would write it; a register AG itself
  updates still waits for every older writer.
* **Early results** (`EARLY_RES`, off by default).  A register MOVE, ADD,
  SUB, AND, OR, EOR or LSL/ASL by an immediate count with operands AG has
  is also computed in AG; a younger base or index whose youngest older
  writer carries one takes it instead of waiting for EX.  EX still computes
  and writes every result; simulation checks that the two agree.
* **DC1/DC2**: the D-ATC/DTT lookup and tag compare (DC1 reads an ATC port
  of its own, a copy of the entries with the set registered with DC1's
  address, and compares the cache tags with all four ways' pages in
  parallel: no address multiplexer before the hit) and the operand from
  DC2's copy of the way it hit (see the DMU).
* **EX**: ALU, shifter (one 66-bit right funnel for every right shift and
  the rotates, one left funnel for LSL/ASL; the carry and X come from the
  operand bit by index, off the funnel, and N and Z from the registered
  result in the op's second cycle; ROL shifts the doubled operand by the
  count's complement, ROXL's amount and the count modulo size+1 are
  registered in a cycle of their own), bit-field unit, BCD, condition
  codes, branch resolution (every prediction is verified here),
  multiply/divide (holds EX; `DIV_STEPS` quotient bits a clock), CHK2/CMP2,
  the FPU interface.  EX results go to the front file and are captured by
  younger uops in AG/DC1/DC2 that name the register.  The uops that can
  suppress their register write (FPU, DIV, DBcc, CHK2, CAS2) are flagged
  as they enter EX, so the kill is an AND-OR of its sources.
* **WB**: the back (architectural) register file, CCR/SR, stores, and every
  redirect: exceptions, interrupts, trace, serialization, and the flush
  behind a misprediction.  A misprediction that WB would redirect for no
  other reason turns the front end round as it leaves EX (a cycle early);
  the back-end stages behind it go when it completes at WB, and AG takes
  nothing in between.

### Register files

D0-D7, A0-A6, USP, ISP, MSP and the microcode temporaries T0-T13.  Two
copies, both MLAB (`ap68040_rf`, write ports and five read ports through a
live-value table): the front file (newest values; three write ports: AG's
two address updates and EX's result) and the back file (architectural; two
write ports: WB's update and result, or the exception entry's pairs).
`fv[r]` marks registers whose front copy is current; a redirect or an
exception clears `fv`, so the back file is read again with no copy.
Exception entry writes the routine's operands (vector address, PC, SR,
address, format/vector word, SSW, EA, WB3A, WB3D) into the back file over
up to five cycles before the routine starts.

## Branch prediction

* **D1** predicts: Bcc by the static rule (backward taken) overruled by a
  256-entry table of 2-bit counters indexed by PC[8:1] (a counter counts
  disagreement with the static rule; its training copy in block RAM);
  BRA/BSR always; JSR/JMP to absolute and PC-relative addresses; RTS
  through an 8-entry return stack (LUT RAM, the top entry read) pushed by
  BSR/JSR.  A taken prediction redirects fetch (four bubbles).
* Training counts from the table's current value, one cycle after the
  branch leaves EX, and not if that cycle flushes.
* **Loop exits** (`LOOP_PRED`, off by default): an 8-entry table learns the
  trip of a backward Bcc/DBcc and predicts the fall-through at that count.
* Each record carries the return-stack pointer D1 found and what the
  instruction did to it; a back-end redirect restores it.
* **The fetch BTB** (256 entries, a slot for each word of a long word;
  block RAM with a registered output, read with F0's address and compared
  with F1's) remembers D1's taken predictions: branches, calls and returns.
  A hit at F0 ends the chunk at the branch's last word, flags that word,
  and sends F0 to the target from F1 (one bubble): the entry's target, or
  for a return the top of the fetch's own return stack, resynchronized from
  D1's on every redirect.  D1 checks every flagged word and rewrites or
  drops a wrong entry.
* EX verifies direction and target of every prediction; a misprediction
  redirects the front end as the branch leaves EX.

## Memory ordering and coherency

* Loads read the cache at DC1/DC2; stores write it at WB.  DC2 keeps the two
  long words of the line it reads and merges every byte write to them, so
  a load after a store to its line stays on the fast path; a load behind an
  older store still in EX/WB waits for it in DC2 only if it reads the
  store's bytes.
* A later read may complete before an earlier write (MC68040UM 7.7), except
  that a cache-inhibited or locked read waits for older stores, and a read
  from a serialized page (or a locked read) is performed only once every
  older instruction has completed.
* Self-modifying code: a store into the 128 bytes after its own
  instruction refetches what follows.  Otherwise the program keeps the
  instruction stream coherent, as the 68040 requires (MC68040UM 4.5, 7.7).
* A snoop that removes a valid instruction-cache line refetches the next
  instruction to reach AG and everything after it.

## Data memory unit (`ap68040_dmu.sv`)

* Data cache: 4 KB, 4 ways of 64 sets of 16-byte lines, physically tagged
  (PA31-10), indexed by PA9-4 (= LA9-4).  Tags and data in M10K; valid and
  dirty bits in LUT RAM, packed per set.  Modes from the ATC/TTR CM field:
  copyback, write-through, cache-inhibited serialized and nonserialized;
  an untranslated access with no TTR match is cachable write-through.
  Reset leaves the caches and ATCs alone (8.2.1); CINVA clears a cache in
  a sweep of its 64 sets.
* The fast path: a load that hits, within one line, answers from DC2's copy;
  a copyback store that hits writes the RAM in its WB cycle.
* An operand across a line whose first part hits: one stolen lookup of the
  next set gives the second part's translation, tags and data; a copyback
  store writes its two parts in consecutive WB cycles without the engine.
* The engine runs everything else, one job at a time: misses (line fill,
  critical word first; a dirty victim goes to the push buffer and is pushed
  after the fill; a bus error on a later beat of a fill takes no exception
  unless the operand needs that long word, and the line is not allocated,
  7.6.1), copyback write-allocate, cache-inhibited and locked accesses (a
  matching dirty line is pushed and invalidated first, 4.6.2), MOVE16 (no
  allocation; a write hit invalidates), CINV/CPUSH (line, page, all),
  PFLUSH/PTEST, and table walks for both ATCs.  Its push, walk and snoop
  readers share one line multiplexer; its operand extractor selects its
  way from the job's registered way.  With the data cache disabled (CACR
  or CDIS) accesses bypass it and leave its lines alone.
* Table walks (MC68040UM 3.2): root, pointer and page descriptors,
  indirect descriptors, 4 KB/8 KB pages, accumulated write protection, U
  and M updates.  Descriptor reads see the data cache (cachable
  write-through, no allocate, 3.2.5); an update is a noncachable write, or
  a locked read-modify-write when U must be set (table 3-1), with a dirty
  copy of the descriptor's line pushed first (with LOCK inside a locked
  update, 4.6.2) and invalidated.  A failed search still creates an ATC
  entry with R clear (B set for a bus error), as on the 68040; exception
  stack accesses and vector fetches do not allocate either (4.5, 7.5).
* ATCs (`ap68040_atc.sv`): 64 entries, 4-way, 16 sets indexed by LA15-12
  (4 KB) or LA16-13 (8 KB), tagged with FC2 and the logical page.
* Snoop port: see below.

## Bus snooping (`ap68040_snoop.sv`; MC68040UM 4.4, 4.7, 7.9)

While another master owns the bus, each of its transfers with TT = normal
or MOVE16 and SC = 01 or 10 is looked up in the data cache, in the push
buffer, and in a push queued for the bus.  SC 01: a read of a dirty line is
supplied by the 68040 (MI held, TA and D31-D0 from it, a line one long word
per bus clock); a byte/word/long write into a dirty line is sunk into it;
other write hits invalidate.  SC 10: a read of a dirty line is supplied and
the line invalidated; any other hit is invalidated.  The instruction cache
drops a line on any snooped write and on SC 10 reads.  MI is asserted
whenever another master owns the bus, except while memory may answer the
current transfer.  The DMU freezes its engine and holds DC1 while the
snooper uses the cache; an invalidated line loses its hit in every
in-flight record, so no store lands in a line that left.

## Exceptions (MC68040UM section 8)

Detected per uop, taken when the uop reaches WB (precise).  The back end
flushes, and the micro-sequencer runs the exception routine, which builds
the frame with ordinary store uops from the exception information.
Interrupts and trace are taken at instruction boundaries; an interrupt
pending as an exception's processing ends is taken before the handler's
first instruction (8.2.9).  Level 7 is taken on its edge and by level
comparison.  Access errors use the 68040's restart model with the format $7
frame (SSW, FA, EA, WB3A/WB3D with WB3S clear); a bus error on a line push
is reported as an access error with the push's SSW (figure 8-6); MOVEM
continues from the stacked EA (SSW CM); RTE's reads do not allocate.  T0
(change of flow) and T1 trace, with a trace deferred over an interrupt.
Fetch faults on speculative prefetch are refetched once on demand before
they become access errors.  A double bus fault halts the processor (7.6.3).

## FPU

The 68040's hardware subset (`rtl/fpu/ap040_fpu.v`, verified against the
cputest corpus), behind `ap68040_fpif.sv`, which runs the FPU beside the
integer pipeline: arithmetic continues in the background while integer
instructions proceed, and exceptions are reported the 68040 way
(pre-instruction at the next FPU instruction).  Divide and square root
take two bits per clock; the 64x64 multiply runs in DSP blocks.  The FPCR
keeps all of bits 15:0 (WinUAE's 040 mask); an FMOVE.X of a signaling NaN
stores it as it is; underflow is decided on the intermediate result before
rounding (9.7.5) and the underflow rounding (F_UNFL) forms the denormalized
result; an overflowing single or double store sets INEX2 only for bits
actually lost.  Unimplemented instructions and data types trap with the
frames an FPSP expects: the deciding state leaves the operands in the
shadow registers as the frame wants them and one state (F_CAP, the cycle
after) captures the frame.  The normalizer takes two cycles: its leading-
zero count is registered (F_NLZ) and the state it returns to shifts.
`FPU_REVISION` selects the FSAVE frame revision ($41 or $40).

An FPm-to-FPn operation is one uop (FPU_RRF: CHK and DISP fused); others
are CHK then DISP, DISP allowed to act behind its own CHK at WB.  The
common memory forms (FPU_K_*) fuse CHK with EAL, the uop that records the
operand's address for the frames.  The next FPU instruction is released
in the cycle the previous one writes back without an enabled exception.
Inside the FPU a binary operation on a non-NaN source goes from dispatch
straight to F_BIN; the alignment shift takes one cycle; an effective
subtraction normalizes its difference (F_NLZ, F_NORM2).  An FADD or FMUL
takes 6 cycles back to back, an effective subtraction 7.  A move that goes
straight to F_ROUND records its exception shadow there; F_BIN and F_EXEC
record theirs as they start.  The accepted flag the core's holds start
from is a register kept with every state assignment.

`FPU_PIPE=1` adds the arithmetic pipeline of the ap68040-60mhz round
(FMOVE/FABS/FNEG/FADD/FSUB/FMUL on ordinary operands with no exception
enabled, bit-identical to the state machine; tb/build_fpu_pipe.sh checks
them side by side).  It is off here: its cost is in STATUS.md.

## Pins beyond the bus

* IPEND: a request exceeds the mask (on bus clock edges), negated once the
  interrupt is recognized internally.
* PST3-PST0 (table 5-6): each instruction's end (branch taken / not taken /
  other) for one bus clock, else the status: table search, halted ($5),
  stopped, RTE executing, exception stacking.
* CDIS disables both caches without flushing them (snooping continues);
  MDIS disables page translation (the TTRs, PFLUSH and PTEST are not
  affected).

## Timing rules

1. Block RAMs have registered addresses; a held stage re-presents its
   address.  The D-cache set index takes AG's address through one LUT
   (the other sources are merged from registers) and is duplicated per way.
2. Long arithmetic is iterative or pipelined: divide and square root two
   bits per clock, the 64x64 multiply in DSP blocks, the FPU normalizer's
   count registered before its shift.
3. Every redirect comes from registered state (D1's records, WB).
4. RAMs are written the way Quartus infers them: a LUT RAM has one read
   address and one write statement; a block RAM has a registered output
   (`ap68040_sdpr`) or is an initialized array (the ROMs, generated; a
   loop of at most 5,000 iterations initializes one); a table read at two
   addresses stays two LUT RAM copies.
5. A change is judged on three fitter seeds (syn/fitcore.sh, SEED=n).

## Source layout

```
rtl/            the core (SystemVerilog)
rtl/fpu/        the FPU datapath
rtl/gen/        generated tables (decoder, attribute ROM, predecoder table, microcode ROM) - do not edit
tools/          ucode.py (microcode, decoder and predecoder tables), cputest replay generator
syn/            Quartus out-of-context fit of the core (fitcore.sh: ALMs and timing per seed;
                mapvar.sh: synthesis-only estimates with parameter overrides; MISTER=1 fits
                with the Minimig project's fitter settings)
tb/             Verilator benches, 68040 bus slave, alternate master
tb/asm/         self-checking 68040 test programs
tb/asm060/      AP68060 programs not applicable to the 68040 (kept for reference)
tb/c/           C programs (vbcc), Dhrystone and Whetstone
rtl_old/        the previous sequential core (reference only)
tb_old/         its benches
doc/            this document, STATUS.md
```
