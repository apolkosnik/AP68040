# AP68040 `faster` status

## Milestones

| | milestone | state |
|---|---|---|
| M0 | the AP68060 tree converted back to the 68040 (631f440): MSP/ISP, format $7 frames with the restart model, MOVEM continuation, T0/T1 trace, the 040 MOVEC set, CACR/TC, PTEST/MMUSR, nonresident ATC entries, table walks through the data cache, 4 KB caches, the 040 snooper, BIU and bus protocol, 128-clock reset, IPL sampling, PST codes, FPU revision $41 frames; the 060 round's pipeline work kept | done |
| M1 | regression: 29 programs x 5 bus configurations, the exhaustive decode bench | done |
| M2 | cputest corpus replay (tb/tb_cputest.sv): the sampled corpus | done: 3,778 of 3,801 slices, the rest the known generator defects |
| M3 | 60 MHz on 5CSEBA6U23I7, out of context (Quartus 17.0, HIGH PERFORMANCE, hold optimized on all paths), judged on three seeds | done: +0.53 / +0.09 / +0.44 ns at the 100C slow corner after the area round (the timing section) |
| M4 | area: under 20,000 ALMs with every feature | done: 19,481 - 19,525 ALMs on three seeds (21,436 at the conversion); with the Minimig project's fitter settings about 20,100 (the area section) |
| M5 | the AP68060's tests under the 68040's rules: level 7 interrupts, IPL skew, an interrupt among snoop invalidations, nested exceptions and double faults, RESET and the halted bus, the cache cases (t_irq7, t_iplglitch, t_snirq, t_xnest, t_halt, t_cache2) | done |
| M6 | performance | Dhrystone 2.1: 964 cycles/iteration, 35.4 DMIPS at 60 MHz (0.590 DMIPS/MHz); Whetstone 469,922 cycles/loop, 12.77 MWIPS (948 cycles/iteration with DIV_STEPS 4) |

## Regression

```
tb/run_tests.sh                 # the 29 programs, five bus configurations each, and the decode bench
tb/build_cputest.sh             # the corpus replay bench
tb/run_cputest.py ~/Downloads/data040.zip                    # smoke slices
tb/run_cputest.py ~/Downloads/data040.zip --full --group BasicFPU --group intFPU --group PackedFPU --sample 300:24  # the FPU groups
tb/run_cputest.py ~/Downloads/data040.zip --full --sample 300:24 --jobs 12  # every slice, sampled
tb/run_bench.sh                 # Dhrystone and Whetstone (vbcc), scores at 60 MHz
```

Testing tiers: run_tests.sh for every change (about 10 minutes at eight
jobs); the corpus slices of the area touched (FPU: the three FPU groups,
sampled, 562 slices; shifts: `--instruction 'AS*' ...`); before a push, the
sampled corpus -- every round of each slice up to 300, then every 24th,
about 40 minutes at eight jobs -- which must fail exactly the known
slices; the full corpus (hours) for milestones.

Bus configurations: zero wait, random waits, waits with TBI and TA/TEA
retries, BCLK at half PCLK, and all of those together.

Programs: smoke, t_integer, t_exceptions, t_mmu, t_cache, t_atcprobe,
t_bitfield_cache, t_bitfield_mmu, t_movem_restart, t_moves_fc, t_fpu,
t_fpu_frames, t_fpu_resume, t_snoop, t_btb, t_stld, t_pins, t_snstress,
t_eredir, t_cbsplit, t_cmodes, t_loops, t_early, and from the AP68060
under the 68040's rules: t_irq7 (level 7 on its edge and by level
comparison, 8.2.9), t_iplglitch (IPL pins settling at different clocks:
a request counts after two consecutive bus-clock samples agree, 7.8),
t_snirq (an interrupt taken while snoops invalidate the instruction cache
and the front end refetches), t_xnest (a handler whose fetch fails is an
ordinary access error; an access or address error during the processing
of an access error, an address error or a reset, the frame's writes and
the handler's fetch included, halts the processor: PST $5, the bus quiet,
until a reset, 7.6.3), t_halt (RESET with a fetch in flight: RSTO waits
for it and no bus cycle overlaps RSTO; the reset's SR; an odd address
error vector halts; the caches survive a reset), t_cache2 (line fill
errors on a later beat, locked accesses, CPUSHP with 8K pages, RTE's
reads not allocating, a misaligned operand's fill error, inhibited writes
hitting a dirty line, a disabled cache, TCI lines, hits behind an
invalidation, MOVE16 with the cache disabled).

The exhaustive decode bench (tb/build_decode.sh, part of run_tests.sh)
runs every operation word through the pattern decoder against
tools/ucode.py's model and through the block RAM predecoder against the
pattern decoder.

The bench (tb/tb_ap68040.sv) provides, besides the 68040 bus slave and
the alternate master: one-shot and standing TEAs on data reads, fetches
and pushes ($F142, $F154, $F156, $F15E), TCI on an instruction line and a
rewrite of its second long word ($F16A, $F16C), IPL timing and skew
($F110-$F150, $F16E), a reset the bench raises itself ($F168), a halted
processor answered with a reset and counted ($F158, $F15A), the watch
window of bus beats ($F2D0-$F2DC), CDIS/MDIS ($F2C0), and checks on every
run: no bus cycle starting or ending under RSTO, none inside the 128-clock
internal reset, PST never reserved, interrupts never phantom, IPEND held
through the acknowledge.

## Area

The goal is the core's ALM count in the Minimig build.  Measured out of
context (syn/fitcore.sh, PERIOD=16.667, retiming off, seeds 1-3; a seed
moves the count by about 100 ALMs and the worst path by about half a
nanosecond, so a change is judged on three):

| state | ALMs (three seeds) | setup slack at 100C | what changed |
|---|---:|---|---|
| 631f440 | 21,436 (seed 1) | +0.08 | the converted core, FPU_PIPE/LOOP_PRED/EARLY_RES off, DIV_STEPS 4 |
| 1de8e9b | 21,575 - 21,687 | -0.05 / -0.32 / -0.41 | the AP68060's FPU timing/area work re-applied with 68040 semantics (accepted register, tree leading-zero count, staged sticky shift, shared incrementer; -288 ALUTs); DIV_STEPS 2 (-130 ALUTs); the worst paths the engine's stolen ATC lookup into DC1's hit and the slow ALU's funnel into the flags |
| 3cb76fc | | | the shifts' carry and X from the operand bit by index, N and Z from the registered result: the funnel off the flags |
| (DC1_ATC=1) | 21,809 - 21,834 | +0.45 / +0.30 / +0.25 | DC1's own ATC port and parallel tag compare (+200 ALMs): 60 MHz on every seed |
| 058958e | 21,298 - 21,336 | -0.17 / +0.30 / +0.38 | the predecoder from a block RAM table (-963 ALUTs, +32 M10Ks); the remaining path the rotate amount's subtraction into the funnel |
| 604e98e | | | F_UNFL through the shared incrementer, a move's shadow recorded once (-354 ALUTs); ROL by the complemented count, ROXL's amount registered |
| 93721e3 | 20,768 - 20,895 | -0.48 / -1.44 / -0.34 | one data RAM way select for the engine's readers (-500 ALUTs): the walk's tag compare and the state decode on the operand path |
| 0f80c32 | 20,901 - 20,916 | -0.03 / -0.37 / +0.21 | the operand path's way from a register again; push, walk and snoop readers share one multiplexer; the FPU normalizer (count, shift and the a_m multiplexer in one cycle) is the worst path |
| 1a48b50 | 20,728 - 20,763 | -0.22 / +0.03 / -0.34 | datatype and unimplemented-instruction frames captured in one state from the shadow registers (-531 ALUTs by measurement of the capture paths); imm_b a signed word; the normalizer's select into a_m and the walk's compare from the valid-copy RAM into the hit update are the failing paths |
| 1434241 | 20,733 - 20,781 | +0.49 / +0.18 / +0.05 | the normalizer's count registered before its shift: 60 MHz on every seed |
| 92ea705 | 20,761 - 20,822 | +0.37 / +0.22 / +0.65 | the table walk's line compare registered throughout the walk: the walk's update state off the hit update; the remaining worst paths are the TC register into the engine's hit decision, the fetch queue's word into D1's record, and the ATC's address into the engine's physical address |
| area round | 19,443 - 19,519 | +0.33 / +0.05 / +0.16 | below |
| MOVE16 fix | 19,481 - 19,525 | +0.53 / +0.09 / +0.44 | a MOVE16 from a cached source line copied the line of the way f_way named, not the hit way's (t_cache2 28-29); the engine sets f_way to the hit way before its line reads, and its operand extractor shares the line multiplexer |

The area round (synthesis estimates, ALMs needed, each change against the
one before; the fit lands about 800 above the estimate):

| change | ALMs |
|---|---:|
| DIV_STEPS 2 at the top level as well (the top's default had stayed 4: every earlier fit had the four-step divider) | -78 |
| FPU: the add, multiply, divide and square-root results through one window select into a_m and grs | -36 |
| FPU: one incrementer for F_ROUND and the single/double register stores | -43 |
| DMU: the ATC's lookup set (later its page and FC2) registered, the steal lagging a cycle; DC1_ATC 0 (no second ATC copy, no 16-way compare) | -222 |
| D1: one adder for every PC-relative branch target; MOVE's destination EA without a PC base | -79 |
| FPU: single and double stores (and F_PACKS) in one form, the single's fields the double's upper bits | -286 |
| bit fields: one right rotator for BFINS's source, the extracted field and the register write-back | -116 |
| fetch: the instruction cache's data RAM by long word, fills written beat by beat | -91 |
| fetch: no line buffer -- the tag written as the fill starts, the fetched long word kept for TCI and failed fills | -198 |
| timing: the shift kind registered with the EX uop; DC1's tag compare from the raw tag RAM output, its transparency registered; the register file's write classified per source; the fetch walk's registers loaded in E_IDLE | +31 |

Measured and not taken in the round: the fetch BTB off (-205 ALMs, but
Dhrystone 964 -> 1,345 cycles); D1's return stack (-37) and history table
(-34) off; the snooper's line copy replaced by a word port (no ALMs, and
the DMU frozen longer); the integer stores in one form (-1).  What remains
measurable: CHK2/CMP2's bound compares (117 ALMs if removed), the FPU's
gradual underflow (64), DC2's store merges (54), the MOVE16 line buffer
(up to 127).

What the options cost, synthesis estimates (syn/mapvar.sh, ALMs needed
and combinational ALUTs) of the same RTL with one option on; their
cycle gains were measured in the ap68040-60mhz round (Dhrystone: early
results -1.9 %, the loop predictor -4.4 %; FP workloads with the pipeline
-20 to -82 %):

| option on | ALMs (estimate) | ALUTs |
|---|---:|---:|
| none (the default, before the area round) | 19,869 | 28,088 |
| FPU_PIPE | +714 | +1,417 |
| LOOP_PRED | +427 | +753 |
| EARLY_RES | +519 | +783 |

Measured and not taken: the integer register files in M10K (the reads a
cycle ahead need a cycle of write bypass that costs what the MLAB banks
cost); a decoded-record FIFO of one (D1 would wait a clock behind every
pop: 28 % on Dhrystone); a uop FIFO of one (the back end's stall would
reach the sequencer's step); the upd2 register-file port (about 200 ALMs
for a cycle on every instruction with two address updates).  Left: the FPU
result output's sources (about 150 ALUTs), the fetch queue's length, the
table walk's locked update sequencing.

With the Minimig project's fitter settings (physical synthesis with
retiming, sparse register packing; `MISTER=1 syn/fitcore.sh`) the same
RTL fitted about 100 ALMs larger before the area round, and about 600 larger after it (20,073 / 20,117 / 20,181 ALMs on seeds 1-3 at the MOVE16 fix): physical synthesis adds about 1,000 ALUTs and retiming 1,300 registers, mostly in fetch, D1 and the FPU, and the retiming moves the worst paths:
that build's closure is its own matter.

## Timing

The paths that limited seeds in this round, and what cut them: the engine's
stolen ATC lookup address into DC1's fast-path decision (DC1_ATC: DC1 reads
its own ATC port and compares tags with the four ways' pages in parallel);
the slow ALU's funnel into the first-cycle flags (carry and X by index off
the funnel, N and Z from the registered result); the rotate amounts'
subtractions before the funnel (ROL by the complemented count, ROXL's
amount registered in its pre-cycle); the walk's tag compare and the state
decode on the engine's operand path (the operand path keeps a registered
way); the FPU normalizer's count, shift and result multiplexer in one
cycle (the count registered); the walk's update state into the data
cache's hit update through the valid-copy read (the walk's compare
registered every cycle the walk runs).  At 92ea705 every seed has at
least +0.22 ns at the 100C slow corner; the paths nearest the limit are
the TC register into the miss engine's hit decision, the fetch queue's
word into D1's record, the ATC's address into the engine's physical
address, the EX operand into the result register, and the valid bit into
the register file's live-value table.

In the area round, with DC1 reading the shared ATC port: the steal's
address multiplexer in front of the entries (the set, then the page and
FC2, registered with a cycle's lag: the engine reads the port a cycle or
more after taking it, and DC1 is held the cycle after a steal); the TTR
match into DC1's tag compare (the registered transparency selects); the
tag RAM's write bypass into DC1's hit (DC1 compares the raw output: tags
are written only by the engine, and DC1's lookup is stale then); the EX
opcode decode into the slow ALU's amount select (the shift kind
registered); the FPU state into the register file's class flags (each
source classified, then selected); the WB kill into the fetch walk's
registers (loaded in E_IDLE).  After it the paths nearest the limit are
the engine's bus state into the EX-stage record's hit, DC2's address into the engine's
hit, and the EX opcode into DC1's operand forwarding.

## Known corpus failures

WinUAE generator defects (a real 68040 fails them too): BasicFPU
FADD.L/0001, FNEG.B/0002, FSNEG.S/0002, FSNEG.X/0007; ODD_EXC
CHK/DIV*/TRAPV and ODD_IRQ EXT/SWAP slices (21).  The sampled corpus hits
23 of them.  Any other failing slice is a regression.

## Running one program

```
tb/build_asm.sh smoke t_integer      # assemble tb/asm/*.s
tb/build_sim.sh                      # verilate (objects in ./obj)
obj/obj_prog/tb_ap68040 +prog=tb/build/smoke.hex [+trace] [+ptrace] [+amtrace] [+prof] [+waits] [+tbi=2] [+retry=5] [+bclk2] [+iplskew=<mask>]
tb/build_decode.sh                   # the exhaustive decoder check
```

## Behaviour notes (decided by tests, the oracle or the manual)

* Authority: real hardware, then WinUAE and its corpus, then the manual.
  Where WinUAE and the manual conflict and a test pins WinUAE, WinUAE
  wins: MOVES with any function code is translated through the root FC2
  selects (t_mmu 55, 177-180; the manual's alternate address spaces are
  not treated as physical); an overflowing FMOVE.S/.D store sets INEX2
  only for lost bits (softfloat's 68K rounding); the FPCR keeps bits 15:0.
  Where no test pinned anything the manual was followed: reset leaves the
  caches and ATCs alone (8.2.1); a cache-inhibited or locked access
  hitting a dirty line pushes and invalidates it first (4.6.2); a locked
  table update carries LOCK/LOCKE and pushes a dirty descriptor line
  first; table searches read descriptors through the data cache without
  allocating (3.2.5), as do exception stack accesses and vector fetches
  (4.5, 7.5); an untranslated access with no TTR match is cachable
  write-through; IPEND negates at internal recognition; interrupt
  processing follows other exception processing at once; BKPT runs an
  acknowledge cycle (TT 3, TM 0, address 0) and then takes the illegal
  instruction; RTE's stack reads and the frame's writes are supervisor
  data; the internal reset lasts 128 bus clocks after RSTI negates and
  RSTI is sampled over two bus clocks (C.4); an IPL level counts once two
  consecutive bus-clock samples agree (7.8); a TA+TEA retry with BG negated
  gives the bus up and reruns the transfer (7.8.1).
* Interrupts: the mask an SR-writing instruction sets decides at its own
  boundary; IPEND keeps a request that beat the old mask; an interrupt
  beats a simultaneous trace; an interrupt pending as an exception's
  processing ends is taken before the handler's first instruction (8.2.9,
  t_exceptions 93-105 -- the 68060 differs); level 7 is taken on its edge
  at any mask and again by level comparison when the mask drops below 7
  (figure 8-3, t_irq7); a MOVEM continuation (SSW CM) runs before a
  pending interrupt.
* Double bus faults (7.6.3, 8.2.1, 8.2.2): an access or address error
  during the processing of an access error, an address error or a reset
  -- the vector fetch, the frame's writes, the fetch of the handler's first
  instruction (an odd handler address included) -- halts the processor:
  PST $5, BR negated, no bus cycle, until RSTI.  A handler fetch that fails
  during any other exception's processing is an ordinary access error with
  FA and the stacked PC the handler's address and the SSW a supervisor
  code read (t_xnest).  RESET drives RSTO for 512 bus clocks once the bus
  cycle in progress has ended (t_halt).
* Access error frames (format $7): SSW SIZE is encoded as the SIZ pins (00
  long, 01 byte, 10 word, 11 line); a misaligned operand within one line
  whose fill fails on the long word it needs takes the access error at
  once with FA the operand (7.6.1, t_cache2 12-14); a bus error on a later
  beat of a fill the operand does not need takes no exception and the line
  is not allocated; a bus error on a line push is reported as an access
  error with the push's SSW (a write, SIZE line, TT normal, TM 0; figure
  8-6).  Fetch faults on the lookahead are refetched once on demand before
  they become access errors; FA is the faulting word's address.
* Odd change-of-flow targets: address error at the instruction, frame PC
  per gencpu (JMP +2/+6, JSR the target, handler fetch the vector offset),
  Bcc/DBcc validate the target taken or not, RTE commits its SR first.
* Memory bit fields touch only the bytes of the field (WinUAE).
* MOVEM: a loaded base/index register commits after the last transfer;
  indexed and PC-relative modes stack CM and the EA, RTE continues.
* Caches: with the data cache disabled by CACR or CDIS, accesses bypass it
  and leave its lines alone, MOVE16 included (4.4.1); a TCI instruction
  line serves only the burst it came with (4.5.1); MOVE16 allocates no
  line, a write hit invalidates (4.5); CPUSHP with TC.P pushes the 8K
  page (t_cache2).
* Bus snooping (MC68040UM tables 4-3/4-4, 7.9): SC 01/10 transfers of
  another master are looked up in the data cache, the push buffer and a
  push queued for the bus; dirty data is supplied (MI held, the 68040
  answers with TA), SC 01 byte/word/long writes into a dirty line are
  sunk, other write hits and SC 10 read hits invalidate; the I-cache
  drops a line on any snooped write and on an SC 10 read.  MI is
  asserted whenever another master owns the bus except while memory may
  answer.  The bench's alternate master (tb/m68040_alt_master.sv) is
  also the arbiter; t_snoop and t_snirq check the cases.
* Bus arbitration: a negated BG gives the bus up after the transfer in
  progress (a locked sequence keeps it).
* Reads may pass earlier writes (7.7) except: a cache-inhibited or locked
  read waits for older stores, and a serialized-page or locked read is
  performed only once every older instruction has completed.
* FPU: FDIV/FSQRT two quotient bits per clock; FADD/FMUL register to
  register 6 cycles back to back, an effective subtraction 7 (the
  normalizer's count cycle); a datatype or unimplemented-instruction trap
  is raised a cycle after the decision (the frame captured from the
  shadow registers); underflow is decided on the intermediate result
  before rounding (9.7.5, softfloat roundAndPackFloatx80); the underflow
  rounding forms the denormalized result (the 68040 keeps nothing else);
  an FMOVE.X of a signaling NaN stores it as it is.
* Branch prediction is invisible to programs except for self-modified
  code: like the 68040 the core may fetch a branch target before an older
  store to it; CPUSHA must precede modified code (MC68040UM 4.5).  A store
  into the 128 bytes after its own instruction still refetches what
  follows.  A snoop that drops valid instruction-cache data refetches from
  the next instruction.
* CDIS's reset-time selection of the multiplexed bus mode is not provided.
