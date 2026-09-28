# AP040 pipeline restructuring plan

Status: planned; implementation has not started. Created 2026-09-24 from the
instruction restructuring review of `dbccaa6f` plus its working-tree changes.

Improve instruction throughput by removing unnecessary dependencies and memory
accesses, distributing work across pipeline stages, and allowing independent
operations to overlap. Preserve architectural results, exception/restart
behavior, clock-enable handling, and the 40 MHz implementation target.

This is the current performance work plan; the original
[implementation plan](doc_AP040_PIPELINE_PLAN.md) remains the feature history.
Use `rtl/ap040_pipe/` and `tests/ap040/` as the active paths. The sequential
reference lives in `rtl/ap040/`; retain its identity when making comparisons.

## Baseline and measurement rules

The review ran 60 primary sequences and 12 dependency/follow-up sequences in
two memory configurations: 144 focused simulations. Each sampled 96 steady
instruction/block intervals. These establish throughput, not complete ISA
correctness. Local memory means the behavioral array; bus means the shared
32-bit interface with zero added waits. Neither is a production cache-hit or
16-bit board measurement.

| Workload | Local CPI | Bus CPI |
|---|---:|---:|
| Register ADD, EXG, LSL, MULU.W | 1 | 1.5 |
| Repeated MOVE.L (A0),D1 | 3 | 6 |
| Loads alternating D1/D3 destinations | 2 | 5.5 |
| Passing CHK.W checking the same register | 2 | 2 |
| Passing CHK.W alternating checked registers | 1 | 1.5 |
| MOVE.L D1,(A0) | 2 | 5 |
| CLR.L (A0), ST (A0) | 4 | 8.5 |
| Register read-only / modifying bitfields | 9 / 10 | 9 / 10 |
| Eight-register MOVEM.L load / store | 19 / 18 | 43 / 34 |
| MOVE16 (A0)+,(A1)+ | 18 | 35 |
| Register FMOVE.X | 9 | 9 |
| Two-register FMOVEM.X load / store | 36 / 30 | 54 / 34 |
| MULU.L / DIVU.L | 3 / 11 | 3 / 11 |
| MOVE.L immediate / LEA displacement | 3 / 2 | 4.5 / 3 |

Starting evidence is in `/tmp/ap040-pipeline-structure/` (review, harness,
assembly, JSON results, logs and source hashes). Preserve the useful evidence
in the repository during phase 0 so this plan does not depend on temporary
files surviving. The current microbenchmarks use simple operand values;
extend dependency and operand coverage before setting workload expectations.

The September 24 archived standalone fit reports 16,271 CPU-hierarchy ALMs,
7,440 registers and 17 DSPs; its complete test top uses 17,475 ALMs and reaches
40.43 MHz. It predates the current working changes. Establish a fresh baseline
before attributing area or timing changes to this work. Compare like-for-like
tops, parameters, constraints, tool versions and fitter seeds.

## Delivery order

| Phase | Deliverable | Depends on |
|---|---|---|
| 0 | Reproducible baseline and fault/handshake coverage | Current tree |
| 1 | Operand-specific hazards and actual write dependencies | 0 baseline |
| 2 | Write-only memory CLR and Scc | 1; relevant write-handshake fixes |
| 3 | Shorter bitfield sequencing | 1 |
| 4 | Explicit instruction metadata and a real operand/address stage | 1–3 |
| 5 | Ordered memory request/response engine and streamed transfers | 4; relevant fault fixes |
| 6 | Staged bitfields, thinner FPU dispatch, staged long multiply | 4; 5 for memory forms |
| 7 | Optional divide overlap with ordered completion | 6; measured workload benefit |
| 8 | Wider instruction assembly and integration qualification | Explicit hazards from 4; final unit interfaces |

Land each independently testable change separately. Phase 6's execution units
can be delivered individually. Investigate instruction-word delivery earlier,
but do not remove gather spacing until its hidden hazard assumptions are gone.

## Phase 0 — Make the baseline reproducible

- [x] Record HEAD, working diff and RTL hashes; preserve existing user changes.
  Done: `run_pipe_perf.py` records HEAD and whether `rtl/` is dirty in every result; the restructuring review's own tree was preserved as bundle 10's WIP commits.
- [x] Turn the temporary performance probes into a maintained runner, assembly
  cases and machine-readable results under `tests/ap040/`. Include dependencies,
  bus transaction counts and architectural checks alongside timing. Reject
  missing images, missing measurements and unexpected exceptions.
  Done: `tests/ap040/run_pipe_perf.py` + `perf/tb_ap040_pipe_perf.v`, 77 cases (the review's 60, its probes, the write-only shapes); retirement, port-B read/write and bus-transaction counts; a missing image, missing measurement, exception or wrong retirement count fails the case; `perf/baseline.json` with `--check`.
- [ ] Measure instruction latency, initiation interval, instruction words,
  accepted reads/writes and stall reasons separately. Treat paired sequences as
  blocks and normalize their CPI explicitly.
  Partly: initiation interval, instruction words, reads/writes and the stall holds are recorded; dependent latency is measured only by the paired probes (`data_to_store`, `address_to_load`, `mul_four_alu`, ...), not as its own column.
- [x] Reproduce the review-16 write acceptance loss during `ce=0` and FPU operand
  fault/exception-port ownership defects. Add focused regressions and fix them
  before changing the affected handshake or adding concurrency. Temporary
  experiments in `/tmp/ap040-review16/` are evidence, not qualified fixes.
  Done: both fixed in bundle 10 (86f69b53): the write receipt and the FPU sequencer stopped by an access fault; `tb_ap040_pipe_wrreceipt_bus16.v` and `_fpufault_bus16.v` fail without them.
- [x] Record current `t_mmu` and `t_bitfield_mmu` failures by case/phase. The
  program runner's overall PASS currently excludes failures in its OPEN list.
  Keep these visible and require resolution before final integration sign-off;
  unrelated gaps need not block phase 1's local improvements.
  Done: none remain: bundle 10 made all fifteen programs required, passing all three phases on both suites.
- [x] Capture a fresh standalone fit with the existing 25 ns constraints.
  Done: 476cc5d3: 17,595 ALMs, 40.41 MHz, +0.251 ns at 25 ns.

Exit: repeatable results for the actual working sources, explicit known gaps,
and durable regressions for acceptance/fault ownership. No performance claim
may rely on reducing the amount of architectural work completed.

## Phase 1 — Describe the operands each instruction actually consumes

Primary files: `ap040_decode.v`, `ap040_ea_calc.v`, `ap040_ea_fetch.v`,
`ap040_execute.v`, `ap040_pipe_cpu.v` in `rtl/ap040_pipe/`.

- [ ] Classify operand roles: EA base/index, ALU source, old destination,
  store data, CCR, early trap operand and auxiliary result destinations.
  Implement the minimum explicit metadata needed; do not introduce another
  broad instruction-family approximation.
  Partly: the address views' consumers are enumerated in `ap040_ea_fetch.v` (`addr_use_a`/`addr_use_b`); no general operand-role metadata yet.
- [ ] Qualify forwarding/hazard producer matches with actual write enables,
  including conditional, second-port and banked-stack writes.
  Partly: CHK's match is qualified by `eaf_writes_reg`; the other producer matches are not yet reviewed.
- [x] Restrict `addr_hz` to inputs consumed by address/early-verdict logic.
  An overwritten load destination is not an address input. Handle store-data
  forwarding independently from base/index readiness.
  Done: `addr_hz` holds only a port whose address view feeds an address or a verdict: a load's destination (port B) and a plain store's data (port A) no longer wait. Repeated MOVE.L (A0),D1: 3 -> 2 local cycles.
- [x] Correct CHK's destination match so a preceding CHK that writes no GPR
  cannot create a false GPR dependency.
  Done: repeated passing CHK: 2 -> 1 local cycle.
- [x] Preserve real change/use stalls where a long EX result would otherwise
  enter the AGU or exception decision in the same cycle.
  Done: the base, index, push, memory-to-memory destination and every verdict operand still hold on a long forward (`address_to_load` unchanged at 4).

Validation: repeated and alternating load destinations; ALU-to-store-data
versus ALU-to-address; passing and trapping CHK; EX/WB forwarding; dual writes;
A7 banking; a producer stalled by memory/divide; random CE and flush.

Exit targets: repeated ordinary loads reach **2 local CPI**; repeated passing
CHK reaches **1 local CPI**; no extra bubble for a store-data-only dependency
when that value can be forwarded safely. Register ADD/EXG/shift/MUL.W remain
at 1 local CPI. These are implementation targets, not completed improvements.

## Phase 2 — Give memory CLR and Scc write-only paths

Primary files: decode, EA-fetch, execute and CPU memory-port arbitration.

- [x] Decode CLR memory as a sized zero store with CLR's CCR result.
  Done: `id_st_only`: CLR leaves EA-fetch with no read and EX stores it as it stores a read-modify-write; flags from EX's CLR.
- [x] Decode Scc memory as a sized predicate store using the correct CCR
  producer; preserve CCR itself.
  Done: same path; EX's condition reads the forwarded CCR as before.
- [x] Remove destination reads in direct and extension-word addressing forms.
  Preserve the genuine read-modify-write paths used by other instructions.
  Done: direct, displacement, indexed and absolute forms (decode's `held_st_only` for the gathered ones); 0 reads in every write-only case of the perf runner.
- [x] Carry the store address, privilege/function code, An update and fault
  context explicitly. Do not commit flags/address changes too early on faults.
  Done: the EX store's address, size, An step and fault path (EX abandons and refetches, EA-fetch takes the owed fault) are the read-modify-write's; `t_fault_edges.s` 30-37 refuses and restarts CLR and ST (A0)+ on both cores; `tb_ap040_pipe_wronly_bus16.v` checks zero reads, one store per operation, every byte of a read-sensitive window, flags, A7's byte step and a wrong-path CLR.

The Motorola programmer's reference manual identifies preliminary reads for
CLR and Scc as MC68000/MC68008 behavior (CLR p. 4-74; Scc p. 4-173). The
sequential implementation is useful for architectural comparison but must not
force an unnecessary read into this 68040 path.

Validation: byte/word/long CLR; all Scc conditions; immediate flag producers;
all supported EAs; A7 byte steps; adjacent-byte preservation; read-sensitive
device model; write faults, MMU rejection, CE pauses and wrong-path squash.
Count operand reads separately from instruction fetch and page-table walks:
there must be **zero destination reads and one accepted logical store**.
Physical beat counts must match access size/alignment on the 16-bit adapter.

Exit target: simple stable-condition cases reach ordinary-store throughput,
initially **at most 2 local CPI**, versus the measured 4. Confirm the reduction
in real bus transactions as well as CPI.

## Phase 3 — Remove redundant bitfield preparation cycles

Primary files: decode and EA-fetch; use existing `bitfield` and `bitmem` benches.

- [x] Supply immediate offset/width together, retaining width-zero-as-32 rules.
  Done: both are taken at the sequencer's start and the port-C phases are skipped; width 0 still means 32.
- [ ] Gather dynamic operands according to actual register-port availability;
  preserve overlaps among field operand, offset, width and BFINS source.
- [x] Rotate register operands directly into the aligned window, bypassing the
  register form's zero-shift `BF_S1` copy.
  Done: the rotation loads the S2 input directly; register forms 9/10 -> 6/7 local cycles.
- [ ] Separate read-only result generation from modifying merge/writeback work.
  Retain timing stages around variable rotate/shift/mask/leading-zero logic.
- [ ] Reuse preparation for memory forms without widening their access spans.

Validation: all eight opcodes; immediate/dynamic offset and width; widths 1/32;
register wrap; negative memory offsets; one-to-five-byte spans; source/destination
aliasing; flags; byte preservation; fault on each transfer; CE pauses/flush.

Exit targets for immediate register forms: read-only **at most 7 local CPI**
(baseline 9), modifying **at most 8** (baseline 10). Dynamic and memory forms
must not regress. This phase shortens the existing sequencer; it does not yet
claim an overlapping bitfield execution unit.

## Phase 4 — Establish stage and completion ownership

Approach taken (2026-09-24), each step behaviour-neutral until its check has
run over every suite and the corpus:

1. A conservative write set per instruction (`ap040_ea_fetch.v` `wr_mask`,
   from the instruction's fields) and a Verilator check at every register
   write port -- WB's two, EX's early An step, EA-fetch's MOVEM port, the
   stack-pointer banks -- that the set covered it. (Found at once: DBcc's
   counter is decided in EX and was in no field; MOVEM's last load lands
   after the MOVEM has left EA-fetch.)
2. EA-calculate forms the address of the simple loads, read-modify-writes
   and plain stores ((An), (An)+, -(An), (d16,An), absolute) from a fourth
   register-file port and EX's An step, when neither instruction ahead may
   still change the base; EA-fetch asserts every address it forms equals it.
3. EA-calculate holds such an instruction until its base is resolvable (EX's
   forward admitted, into a register), and EA-fetch uses the registered
   address for it alone.
4. The same for pushes, RTS/RTE pops and the indexed forms. Only when every
   L1 address source is a register does the adder leave the L1 address path
   -- the bus16 top's timing (33.43 -> 35.84 MHz so far) and phase 5's base.

Steps 1 and 2 are in: both suites, fast and with a random clock enable and
the slow L1, ran with neither check firing.

Step 3 is in, and step 4 in part. EA-calculate takes the base from the
youngest producer: the instruction ahead's (An)+/-(An) step (formed there a
cycle earlier, a register, and its only write to that register), EX's
result (taken only in a cycle EX advances, so it is final, and only into a
register), EX's An step, or the register file. It waits for anything else:
another write from the instruction ahead, a write EX makes that is not
forwarded, a MOVEM load still landing. EA-fetch uses the registered address
and An value for (An), (An)+, -(An), (d16,An), (d16,PC) and absolute loads,
read-modify-writes, CLR/Scc and stores, displacement stores included; its
own views remain only to check them (`agu:`), and an invariant checks that
no such instruction reaches EA-fetch without its address. The waits moved
rather than grew: the 77 perf cases are unchanged, cycle for cycle. The
core top fits at 43.33 MHz (+1.921 ns at 25 ns, 18,355 ALMs, +654 for the
fourth port and the stage) with EA-calculate in none of the 40 worst paths.
`t_agu.s` puts an access straight behind the producer of its base from each
source, on both cores.

Two things the checks could not see, found by a mutation that should have
cost cycles and did not. EA-fetch still forms, correctly, any address it is
not given, so an instruction the stage wrongly passes over fails nothing:
the classification took MOVE from SR/CCR's CCR bit -- opcode bit 9, decoded
for every instruction -- as a MOVE from SR, and every access with bit 9 set
(any load into D1, D3, D5 or D7, among others) had been going the old way.
The perf harness now counts the instructions that leave EA-fetch with an
address from EA-calculate, and `--check` fails a case that has fewer than
its baseline. And a MOVEM load whose beat faulted left its read marked
outstanding through the exception: since `l1_rvalid_b` is a level, every
read after it -- the vector, then the handler's own loads -- was written
into the faulted beat's register until the next flush, which a handler
could see, though the restart then reloaded it. `t_fault_edges.s` 38-45
check the handler's view on both cores; the MOVEM port now only ever
writes for the MOVEM in EA-fetch, which the write-set check holds, and so
EA-calculate needs no wait of its own for a MOVEM's last load.

Left for later, on the evidence of the fits: pushes, pops, the indexed and
full-format forms and the sequencers' starting addresses. The L1 address
adders are not what limits either top -- the core's worst path is EX's
result into EA-fetch's operand registers, and bus16's is EA-fetch's
sequencer selects into membus's prefetch-window snoop, which phase 5's
request storage removes -- so moving them buys structure, not time. They
move when phase 5 wants every request's address from a register.

Step 4, the indexed forms: EA-calculate forms (d8,An,Xn) and (d8,PC,Xn)
too, reading the index through a fifth register-file port and resolving it
from the same places as a base (the instruction ahead's An step, EX's
result, EX's An step, the register file). It was meant to take the index
register, its scale and two adders off the L1 address -- the bus16 top's
worst path once pipelined loads were in (-0.169 ns at 25 ns). It did take
them off, and the slack did not improve (-0.590): the path came back
through EA-fetch's other address views, the same shape as before, which
is what the membus change below addresses. The full format stays with
EA-fetch. Indexed loads and stores now count in the perf harness
(load_index, store_index), all their addresses from EA-calculate.

- "the index hazard ignored" and "the step ahead not taken for the index"
  survive, and are equivalent while decode gathers: every writer EX does not
  forward -- MULL/DIVL's second result, a MOVEM load, MOVEC to a stack
  pointer -- is itself a multi-word instruction, and so is every indexed
  consumer, so the consumer is delivered only once its producer is past EX
  (t_agu.s 88-89 put MULL's high word and DIVL's remainder straight in as
  the index, and pass with and without the term). Both stay for phase 8,
  whose wider decode takes that spacing away.

Primary files: CPU, decode, EA-calculate, EA-fetch, execute and register file.

- [ ] Define a documented instruction metadata bundle compatible with the
  existing Verilog/Quartus flow: PC/next PC, operation, operand roles, size,
  destinations, CCR effects, privilege/function code, An updates, memory
  attributes and exception/restart context.
- [ ] Specify valid/ready, acceptance, response, completion, squash and CE
  behavior. Retain acknowledgements until consumed; accepted transactions
  must complete or fault exactly once.
- [ ] Move operand read and address arithmetic into EA-calculate incrementally:
  simple EAs and LEA, then indexed/full-format forms and secondary addresses.
  Register addresses before the memory stage. Adjust forwarding/read ports as
  needed rather than merely moving the adder into an earlier file.
- [ ] Preserve MOVE source-update-before-destination-EA semantics and genuine
  memory-indirect pointer dependencies.
- [ ] Give faults and completions a single instruction owner. Track partial
  progress for operations such as MOVEM; do not assume every instruction can
  delay all architectural effects until its final beat.
- [x] Replace hazards that currently rely on gather bubbles, including port C
  and control-register dependencies, with explicit checks (phase 8's first
  step: port C's address uses, MOVES and MOVEC).

Exit: architectural and bus-event equivalence for the migrated paths, except
the intentional removed CLR/Scc reads; no new combinational
EX-result-to-AGU-to-memory path; no simple-ALU throughput regression. Keep
single-issue, ordered architectural completion as the initial design.

## Phase 5 — Stream ordered memory work

Progress (2026-09-24): the local targets below are met -- eight-register
MOVEM load 19 -> 12 cycles, store 18 -> 11, MOVE16 18 -> 12 -- with no case
slower. A run of loads now issues each next read in the cycle the last one's
data arrives (MOVEM, MOVEP, CHK2/CMP2's two bounds, CAS2's two operands);
the L1 model's write buffer takes a new write in the cycle it drains the
last one, so a run of stores is one per cycle; and MOVE16 reads its line
into a four-longword buffer back to back and then writes it, the sequential
core's own order. FMOVEM's reads are the FPU wrapper's microcoded read
subroutine and wait for 6B.

The bus16 top meets 40 MHz: 41.05 MHz, +0.642 ns at 25 ns (from 35.84 MHz,
-2.900 at d1373fc6, and 38.52, -0.959 with phase 4's step 3). Two changes.
membus's write snoop no longer works out which of two longwords a write
reaches: it takes both -- a rare extra refetch -- which took the write's
size and an adder off the worst path (EX's SR forward, the stack bank, the
address arithmetic, the snoop, pf_base). And a combinational loop that had
always been in the RTL -- EX's read-modify-write waits on wr_busy, which
membus formed from wren_b, which EA-fetch formed from its stalls, which EX
formed; Verilator's UNOPTFLAT on stall_self -- was synthesized as one for
the first time (22 nodes, a 17.7 ns LOOP element, 26 MHz). The CPU only
reads wr_busy while it presents a write, so membus now gives it wr_busy_w,
the same with wren_b taken as set; the loop is gone from both tools.


A plain load now leaves EA-fetch in the cycle its read goes out (lx): EX
takes the data from the port as it arrives, holds until then, and owns its
fault, which is abandoned and owed like a refused store's; port B is the
load's until its data is in, so nothing younger reaches memory first. A
plain load is one cycle rather than two locally (5.5 -> 5.0 on the zero-wait
bus, 8.5 -> 8.0 with two wait states, 13.0 -> 12.5 with five), and two
independent loads in a row are 2 rather than 4. The same for a read-modify-
write was built and measured and is not in: locally 4 -> 3, but on the bus
the store is then accepted in the cycle the read returns, which is the cycle
membus would have used for a prefetch -- it takes none in a write's accept
cycle, so a fetch can never overtake the write (d1373fc6) -- and RMW-then-ALU
lost a cycle at every wait count (9 -> 10, 15 -> 16, 24 -> 25). It waits for
a membus that can decide the prefetch against a registered write.

The corpus found the one thing no bench had: MOVE <memory>,SR and
MOVE <memory>,CCR, which decode marks as the ORI/ANDI-to-SR kind
(is_immsr) and EX writes from eaf_operand_a -- sent on this way, they
wrote the status register from a stale operand (MV2SR.B/.W in Basic,
Default and IRQ). They wait for their data in EA-fetch as before, and
t_agu.s 76-80 now have them.

The surviving mutations of the pipelined load, each equivalent:

- "the port is not held for the load" survives, and is equivalent: while EX
  waits for its load (ld_hold) it stalls EA-fetch, and every EA-fetch request
  is gated by that stall (stall_self / !stall_in); in the cycle a faulted read
  arrives the flush takes EA-fetch's instruction (live) and nothing of it has
  started. The term, and the CPU's port gates, stay as the port's own
  guarantee.
- "EA-fetch also takes the pipelined load's fault" (rd_out set for it)
  survives, and is equivalent: the read's fault is EX's ex_aerr, whose
  flush drops EA-fetch's live in the same cycle, so aerr_rd (live-gated
  through aerr_now) never fires for it; the flush clears rd_out too.
- "the load data is not kept while EX is held" survived: the latch was
  unreachable (no instruction sent on this way holds EX after its data), so
  it was taken out and an assertion holds the reason.
- "a load becoming an exception is sent on" survives, and is equivalent:
  every entry an ordinary load can meet -- an owed trace, an interrupt --
  raises trc_hold/irq_hold before and while it is taken, and mem_issue
  requires !trace_hold; an access error's owe raises ae_busy. The traced
  run of loads (t_fault_edges.s 55-60) was added to be sure, and passes
  with and without the term.

The bus16 top's worst path kept the same shape through every change above:
some port-B address source in EA-fetch, through the address views, into
membus's write snoop and on into pf_base, q_a or a fill -- each fit found
whichever source was left (the indexed views; EX's SR forward into the
stack bank; a MOVEM's register). So the register file's stack bank now
comes from sr_base, WB's commit and the register, not EX's forward: every
change of S or M empties what is behind it while it is in EX (asserted), so
no register is read in its commit cycle. And membus snoops a write a cycle
after accepting it, from w_addr, and in the accept cycle answers no fetch
at all -- a request, or a pending fetch whose fill arrives then, is held
(a_dfr) and answered next cycle from the window if the write missed it, or
fetched again after the write. The CPU's port-B address reaches only
membus's registers. No perf case changed. Fits: bus16 41.04 MHz, +0.635 ns
(its worst path now EX's shifter into eaf_operand_b); the core top +1.570.
The stack bank from the raw SR register, WB's commit not written through,
survives mutation and is equivalent, for the same reason: the first
refetched instruction reads after the commit.

Primary files: EA-fetch, `ap040_pipe_membus.v`, `ap040_pipe_l1.v`, bus16 and CPU;
include the FPU wrapper when moving its memory operations.

- [ ] Introduce bounded request/response storage with explicit per-beat owner,
  address, size, register destination, final-beat and restart information.
  Support response consumption and next-request acceptance without an empty
  bookkeeping cycle when the downstream memory can sustain it.
- [x] Allow store-buffer consumption and replacement in the same clock when
  legal (the L1 model; membus's bus is one transaction at a time). Preserve byte overlap, ordering, function codes and fault attribution.
- [ ] Feed MOVEM, paired memory operations, CHK2/CMP2 and FMOVEM through this
  engine. Do not expand all their transfer helpers into separate global stalls.
- [x] Give MOVE16 a small line buffer and consecutive-transfer path. Only use
  bursts where the downstream interface and memory attributes permit them.
- [ ] Keep device/strongly ordered accesses conservative. Preserve MOVEP's
  spaced byte transactions, bitfield boundaries, and CAS/CAS2 atomic ownership.

Validation: faults on every beat, aliased base/destination registers, identical
MOVE16 lines, cache-inhibited/device accesses, page crossings, pending writes,
snoops/self-modifying code, interrupts, CE pauses and reset/flush while busy.
Compare partial progress and physical transaction logs, not just final RAM.

Provisional local-memory targets with a one-beat-per-cycle-capable interface:
eight-register MOVEM load **at most 12 cycles**, store **at most 11**, and
MOVE16 **at most 12**. Confirm these budgets against the finalized interface
before implementation. Report bus-limited throughput separately; additional
queue entries cannot increase a serial external bus's transfer capacity.

## Phase 6 — Make execution units accept prepared operations

Deliver these as separate changes, preserving ordered completion.

### 6A. Register bitfields

6A, first step (2026-09-24): the register forms 6 -> 4 cycles (a modifying
one 7 -> 5), memory forms one fewer, a dynamic offset 8 -> 7. A register
field with an immediate offset and width is rotated in the sequencer's
first cycle -- port A already reads the register then, and port C, with no
EA to index, is pointed at the extension word's register (BFINS's source)
-- and the extract stage and the flags/result stage, two registered steps
that only fed each other, are one. The staged EX datapath the plan
describes is still to come. Its mutations: the register field's source not
pointed at from the start, the merged stage's stale field, BFFFO's stale
mask, and the start merge undone (a slowdown) are caught; a rotation
started with a register offset is caught by t_agu.s 98, added for it.

- [ ] Move the phase-3 field datapath into staged execution with valid bits,
  operand/result metadata, forwarding and explicit dependency handling.
- [ ] Share it with memory bitfields through phase 5's operand-window path.
- [ ] Measure dependent latency separately from independent initiation rate.
  Aim to accept independent prepared register operations every cycle; the
  current word-at-a-time decoder may still limit instruction-level throughput.

### 6B. FPU dispatch and transfers

6B, first step (2026-09-24): register FMOVE 9 -> 7 cycles, FADD 11 -> 9,
and one cycle off every F-line instruction. The wrapper's outcome was
complete as it entered S_FIN -- exc, w1/w2 and the redirect are registered
on the way in -- but fin and a forced T0 flow were set in S_FIN itself, a
cycle later; they are seen on entry now. And a general operation whose EA
is not An's own skips DISPATCH, which only captured An and passed it on to
S_FPU_DEC. The engine's own cycles and the wait for EX to empty remain.
Its mutations: DISPATCH skipped for (An), (An)+ or -(An) is caught (fpudual,
the programs); fin or the DISPATCH skip undone is caught as a slowdown; a
forced T0 flow not seen on entry is caught by t_fault_edges.s 64-67, FSAVE
and FRESTORE traced under T0, added for it. DISPATCH skipped for a general
operation with EA mode 7, register 5-7, survives: decode turns those into
F-line exceptions itself (fp_bad7) before a word is fetched, so the wrapper
never sees them -- 61-63 check the exception.

- [ ] Decode static FPU command class and operand requirements once. Launch a
  prepared command when operands, engine acceptance and deferred-exception
  ordering permit, without repeating general dispatch/decode helper states.
- [ ] Preserve the existing `fpu_bg` release of register-destination arithmetic.
  Keep deferred exceptions, FPSR/FPIAR updates and save/restore semantics.
- [ ] Route FMOVE/FMOVEM memory operands through phase 5; retain a dedicated
  serial path for architectural state save/restore and complex exceptions.

Target: register FMOVE falls below 9 CPI, initially aiming for **at most 7**;
FMOVEM loses transfer-helper bubbles. Validate `fpudual`, exception frames,
resume behavior and faulted operands before claiming improved FP throughput.
Operand-sensitive arithmetic timing needs representative finite/special values.

### 6C. Long multiply

6C, first step (2026-09-25): a 32-bit MULU.L/MULS.L no longer holds EX.
Its product and flags go straight into WB's registers as it leaves, from
the product itself, so register MUL.L 3 -> 2 cycles, the decoder's supply
limit. The product is not on EX's forward path (that would put the
multiplier in front of the forward mux), so a reader of its register waits
one cycle (ex_res_slow: EA-fetch's hold, EA-calculate's base and index
hazards) and takes it from WB's commit. The 64-bit forms and the divides
keep the hold, for their second register and An step. t_agu.s 90-104 add
MUL.L followed by readers of its register as data, base, index and
bitfield operand, and the register bitfields' offset forms. Fits: core
+1.228 ns, bus16 +1.790.
Its mutations: EA-fetch not waiting for a staged multiply, V from the low
half only and the result from EX's path are caught (the programs, mull);
the hold put back is caught as a slowdown. A staged multiply marking the
multiplier busy survives, equivalent while decode gathers: a multiply held
for its product right behind a staged one would take the stale product,
but every MUL.L is two words, so the next reaches EX two cycles on, when
mul_busy has cleared. EA-calculate taking a staged multiply as a base
survives and is equivalent: MUL.L writes only data registers, and a base
is An; the term stays beside the index's, where a data register can be
the index. Taking it as an index survives, equivalent while decode
gathers: traced on t_agu.s 91, the indexed load behind a MUL.L is judged
with the multiply already past EX. Phase 8 takes that spacing away, so
both terms stay.

- [ ] Replace `mul_wait` as a global EX hold with a staged multiplier request
  and result path; carry both possible destinations and CCR metadata.
- [ ] Preserve signed/unsigned, 32/64-bit result, register-alias and An-update
  behavior, including memory forms that produce three register effects.

Target: independent prepared operations can enter the multiplier each cycle;
register MUL.L instruction throughput approaches the current decoder's
**2 local CPI** supply limit. Keep MUL.W's existing 1-CPI path. Fit the design
before accepting additional DSPs, registers or bypass muxes.

## Phase 7 — Consider overlapping iterative division

Proceed when workload measurements justify the completion machinery and area.

- [ ] Retain an iterative divider initially; give it captured operands,
  destination/flag metadata and a tagged completion slot.
- [ ] Allow independent younger arithmetic to execute only with explicit
  dependency tracking and bounded ordered result storage. A scoreboard alone
  cannot preserve register/CCR order or exception precision.
- [ ] Prevent younger stores and irreversible state changes from passing an
  unresolved older operation. Specify interrupt, trace, flush and multi-result
  behavior before enabling overlap.

Validation: signed/unsigned word and long division; zero/overflow; dependent
consumers; CCR readers/writers; remainder aliases; memory-source faults; CE
pauses; younger exceptions; queue-full backpressure.

Exit: DIVU.L plus four independent-GPR ADDs takes **fewer than the baseline
15 local cycles**, with architectural CCR and completion order unchanged.
Do not assume fully pipelining/unrolling the divider is the best area tradeoff.

## Phase 8 — Improve instruction assembly and qualify integration

Phase 8, first step (2026-09-25): two words a cycle into decode.

- Port A answers with the word asked for and, when the address is a
  longword's first half, the word after it (l1_rdata_a2: the L1 array's
  next word, membus's longword low half). ap040_inst_fetch.v keeps what
  decode has not taken in a four-word queue and shows decode the first two
  words of the stream -- the queue, then the fetch returning now, so a
  redirect's target reaches decode as soon as before.
  The queue fills behind an ordinary stall; `hold` stops it for the
  reset-vector read and STOP. Every word keeps its own fetch-fault flags.
- When the fetch asks decides the bus tops' schedule. Asking whenever the
  queue had room for two more words (judged before decode's take) kept the
  queue full and asked for the next longword just as decode took an
  instruction -- in front of that instruction's own first data access, in
  every iteration of a memory-bound loop: ten bus cases a cycle slower at
  two wait states (movem_load2 17 -> 18), with the same transactions. A
  hint that held membus's prefetch back for a coming read changed nothing:
  the fetch's own request then went out as a demand in the same place. The
  fetch now asks when at most one word will be left after the take -- the
  one-word fetch's timing, and still two words a cycle into decode on the
  core top.
- membus kept a read on the bus that a request wanted only when it
  answered the request at once; one held over a write's accept cycle
  (a_dfr) killed it and fetched the longword again. The fetch's queue asks
  in such cycles often: behind a read-modify-write every instruction
  longword was fetched twice (rmw_add 13.5 -> 15 cycles at two wait
  states). The held request keeps it too now.
- PROG_WORDS counts the words decode takes. Counting fetches charged a
  bench's budget with every word a redirect discards from the queue, up to
  six a time where it used to be one, and cut chkmem, dbcc and exctrace
  short. The first fit summed issued + (words taken) and added the result
  into the fetch address: the words taken come from the pipeline's stall,
  the latest signal the core has, and all 40 worst paths ran through that
  sum (core -4.497 ns, bus16 -3.432). Every wide value is now formed from
  registers for none, one and two words taken, and the count only picks;
  the queue's next contents are chosen the same way.
- A bus error on a speculative fetch still never surfaces (X2.2,
  t_exceptions 139): the old fetch asked only for the word decode needed
  next, so membus saw every request as a demand; the queue asks ahead. A
  fetch that faults is dropped unseen if it was fetched ahead of need and
  fetched again once the stream is empty, so a lasting fault is taken where
  decode needs the word and a one-shot one is gone.
- Decode completes a gathered instruction whose first extension word is in
  the second slot in the opcode's cycle (`fast`), and a longer one a cycle
  sooner. The gather start's loads are wires g_* (a task, start_gather,
  runs them), and the completing logic reads h_* = fast ? g_* : held_*, the
  extension word from xw = fast ? if_word2 : if_opcode; the rewrite was done
  by a script that checks declaration order, and the completion's reads
  were substituted, never its targets.
- The branch redirect reaches port A's address, and in the fast case the
  gather-start classification ran into it (the core top's worst path once
  the budget was fixed: queue -> gstart -> h_* -> the negative list -> the
  L1's read). Of the kinds that can complete in the opcode's cycle only
  Bcc.W, BSR.W and DBcc redirect, so the fast redirect decodes those three
  from the opcode and the held one reads the held registers; a Verilator
  check holds the split to the one whole expression.
- The CPU's store snoop covers every word fetched and not yet taken,
  if_pc up to if_end, the fetch in flight included.

What the gather's bubble had been doing, now explicit (phase 4's last item):
- Port C's address uses -- an index while the port names it, a
  memory-to-memory MOVE's destination index -- hold for a long forward
  (addr_use_c). dhry's LEA (0,A3,D0.L),A2 straight behind the LSL making D0
  took the stale index.
- MOVES waits for a control-register write in flight (SFC/DFC), and MOVEC
  reading USP/ISP/MSP waits while the instruction in EX may write A7.
- The "never happens" checks for CHK2/CAS verdicts, displacement-store
  bases, the index, MOVES and MOVEC now say "never used without its hold".

New benches: tb_ap040_pipe_program_local.v runs t_integer, t_fastpaths,
t_agu, t_fpu_frames, t_fpu_resume, dhry, bench_alu and bench_loop on the
core top, whose one-cycle array feeds two words a cycle; the bus16 program
bench's 16-bit bus rarely does. It found the dhry index at once, and one
defect that was there before phase 8: the L1 array let a fetch overtake a
buffered write, so a store into the next instruction refetched the old one
(t_integer 192); port A now waits for a write that covers its words
(a3e99024, committed ahead of this step).
t_agu.s 105-115 put each pair the gather kept apart straight together.

Fits: core 41.97 MHz, +1.173 ns at 25 ns, 19,389 ALMs (6C: 42.07 MHz,
+1.228, 19,045); bus16 42.04 MHz, +1.212 (6C: 43.08, +1.790, 19,823 ->
20,169) -- about 345 ALMs for the queue, the second port-A word and the
fast path, and neither worst path is in them.

Measured (cycles, before -> after; the programs as tb_ap040_pipe_program_local
and tb_ap040_pipe_program run them): on the core top dhry 247,018 ->
225,966 (-8.5%), t_integer -16.5%, t_fastpaths -14.3%, t_fpu_frames
-13.5%, bench_loop -24.2%; every gathered perf case a cycle a block
faster on the core top (466 -> 428 cycles over all 79). On bus16, whose
16-bit bus supplies under a word a cycle, the programs are level --
dhry +0.1%, t_bitfield_cache -11.8%, t_mmu -1.7%, the rest within 1%.
The perf harness's zero-wait bus top is 769.3 -> 751.9 over all cases,
and at two wait states 1,042 -> 1,039 with no case slower; at zero wait
five store loops are a third to a half cycle a block slower (movem_store2
and 8, movep_store, clr_disp, scc_disp) with the same transactions, fetch
by fetch -- where a fetch falls in a bus the loop keeps full.

Its mutations: the fetch ignoring take2, a redirect keeping the queue, the
queue filling past four words, the budget counting fetched words again,
the fetch running through the reset-vector read, a speculative fetch fault
surfacing, every fetch counted a demand, and a dropped speculative fault
never fetched again are caught (the branch benches, chkmem, dbcc,
exctrace, gxlen, the programs); asking before the take again is caught as
a slowdown at two wait states. membus killing the wanted read in a
deferred request survives the perf check: under the final fetch timing
none of its loops meets that case. The programs do -- 387 times on the
bus16 program bench, 49 in smcdual -- and without the fix t_bitfield_cache
takes 2.1% longer (0.05% over all the programs); measured, not caught.
Decode taking a faulted second word survives, and is equivalent: the
fetch presents a faulted word only at the head of an empty stream -- a
speculative fetch's fault is dropped, a demand fetch goes out only when
the stream will be empty, and one fetch is ever outstanding -- so the word
after it is the same fetch's, with the same flag, and !if_flt already
refuses it. !if_flt2 stays, for any later fetch that asks further ahead.
Turning the fast path off is caught as a slowdown on the core top. Also
caught: the fast path reading the opcode as its extension word, the fast
redirect taking a forward Bcc or leaving DBcc out (the branch benches and
the split's check), the store snoop covering only the word presented
(t_agu.s 115, smcdual), a longword fetch at an odd word taking both
halves and membus handing the high half as the second word (prefetch,
the programs), dropping the port C hold (jmpmodes, dhry, t_agu.s 105-108),
the MOVES hold (t_agu.s 113) and the MOVEC one (t_agu.s 114), and either
L1 fetch overtaking a write (t_integer 192) -- 21 of 23.

- [ ] Add wider word delivery/buffering, length/predecode information and
  assembled instruction packets. Start with common two-word forms, then long
  immediates and full-format/FPU extensions. A queue alone cannot exceed its
  sustained input-word bandwidth.
- [x] Test producer/consumer adjacency previously hidden by gather cycles:
  indexed EAs, dynamic bitfields, MOVES function codes, MOVEC and stack banks
  (t_agu.s 105-115, on both tops and the sequential core).
- [ ] Preserve instruction-fetch faults, PC-relative bases, branch prediction,
  discarded extensions, page boundaries and self-modifying-code invalidation.
- [ ] Profile calls/returns and CAS2 before later target/return prediction or
  operand-gather optimization. Keep these outside the initial implementation.
- [ ] Run representative programs on the actual bus16/MMU/cache integration,
  resolve outstanding integration failures, and complete a full-system fit.

Exit: common multiword instructions improve when instruction supply is
available; 1-CPI register operations remain unaffected; real-system workloads
improve without correctness or timing regressions. Preserve explicit drains
for translation changes, exceptions/RTE and cache-control ordering.

## Validation and fit gates

For each focused change, run affected instruction benches in ordinary mode and
with random CE/slow memory. Include mixed sequences that exercise forwarding,
faults and interactions rather than only isolated instructions. Broaden to the
full suite/corpus at stage-interface and milestone boundaries.

Existing runner examples (select names relevant to the phase):

```sh
python3 tests/ap040/run_pipe_verilator.py --only move_mem,chk,bitfield,bitmem --work tests/ap040/build/restructure-directed
python3 tests/ap040/run_pipe_verilator.py --only cepause,storeonce,rmwsup,rmwfc,fpudual,smcdual,fxdual,irqdual --ce-random --slow-l1 --work tests/ap040/build/restructure-interactions
python3 tests/ap040/run_pipe_verilator.py --only program --ce-random --work tests/ap040/build/restructure-programs
```

Inspect every OPEN program result independently of the aggregate exit status.
Use `tests/ap040/run_cputest.py --core pipe` with the relevant instruction
groups and the installed corpus; compare architectural results against a
recorded reference revision. Bus semantics with deliberate corrections, such
as CLR/Scc, also require manual-grounded transaction expectations.

Run the standalone `tests/ap040/pipe_synth/run.sh` with a **new, dedicated work
directory for each checkpoint**: the existing script deletes its argument
directory before building. Fit after changes to forwarding, AGU placement,
bitfield logic, functional units or memory interfaces. Record CPU hierarchy
and whole-top ALMs, registers, RAM, DSPs, worst paths and all-corner timing.
Do not relax the 25 ns constraint to accept a performance optimization.

Each implementation change must report:

- Architectural/transaction checks and remaining known failures.
- Before/after CPI, dependent latency and independent initiation interval.
- Changes to instruction/data bus traffic and stall causes.
- Area/timing deltas when the datapath/interface changed; explain any increase
  against its measured benefit. No arbitrary full-system area estimate from a
  standalone CPU fit.

Final completion requires the targeted throughput gains, resolved relevant
fault/restart gaps, normal and stressed regressions, affected corpus coverage,
and timing closure within the FPGA's full-system capacity. Hardware speedup
remains unclaimed until measured on a qualified image.
