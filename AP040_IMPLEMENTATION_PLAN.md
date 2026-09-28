# AP040 Implementation Plan

For the instruction-performance restructuring work identified on 2026-09-24,
see [the pipeline restructuring plan](doc_AP040_PIPELINE_RESTRUCTURING_PLAN.md).
It uses the current repository paths and defines implementation order,
correctness checks, performance targets, and FPGA fit gates.

This repo holds two CPU cores, not one:

- **`rtl_old/`** -- the working, feature-complete sequential-FSM MC68040 core.
  Boots NetBSD/amiga and AmigaOS 3.x on real hardware, passes 3,776/3,801
  slices of the WinUAE `cputest` corpus (the rest are documented generator
  defects, not core bugs -- see `doc/CPUTEST_UPSTREAM_REPORT.md`). This is the
  **reference**: correct, but a multi-cycle-per-instruction FSM, not a
  pipeline. Its own test suite is `tb/run_tests.sh`.
- **`rtl/`** -- a from-scratch six-stage pipelined replacement
  (`ap040_pipe_core.v` and its stage files), built incrementally, one
  instruction/mechanism at a time, verified bit-for-bit against `rtl_old`'s
  decode/execute logic at every step. This is the active work. Its test suite
  is `tb/run_pipe_tests.sh`, and it needs nothing from `rtl_old/` or vice
  versa -- deleting either directory cannot affect the other (see
  `ap040_pipe_core.v`'s header).

The brute-force plan is exactly that: read what a stage does in `rtl_old`,
port its *intent* (not its FSM structure) into the matching pipeline stage,
add a testbench that would fail if the port were wrong, and move on. Sections
5-6 below are the actual TODO list; sections 1-4 are the parts of the
original architecture spec still worth keeping as reference.

## 1. Integration contract (unchanged from `rtl_old`)

Both cores present the same TG68K-shaped port set so either can drop into a
host that already speaks that interface (this is `rtl_old/ap040_tg68k_compat.v`
today; the pipeline will grow the same adapter once it's far enough along to
run real programs -- see section 6):

- clock/reset: `clk`, active-low `nreset`, `clkena_in` as the only advance
  enable
- external bus: `addr_out[31:0]`, `data_in[15:0]`, `data_write[15:0]`,
  `nWr/nUDS/nLDS`, `busstate[1:0]` (fetch/idle/read/write), `longword`, `FC[2:0]`
- MMU/cache sideband: logical/physical address, cache-inhibit, walker
  request/ack channel, cache req/ack/burst
- parameters: `AP040_HAS_MMU`, `AP040_HAS_FPU`, `AP040_ENABLE_CACHE`

`rtl_old`'s internal file split (one `ap040_core.v` FSM containing decode/EA/
exceptions/sequencing, plus standalone `ap040_alu.v`/`ap040_mmu.v`/
`ap040_cache.v`/`ap040_fpu.v`/`ap040_regfile.v`) is what actually got built and
proven, not the finer-grained module list an earlier draft of this plan
proposed -- if you're looking for `ap040_ea.v` or `ap040_sequencer.v`, they
don't exist; that logic lives inside `ap040_core.v`.

## 2. Non-goals (still true)

- cycle-accurate MC68040 external pin timing
- a physical 32-bit data bus exposed to a 16-bit host fabric
- multiprocessor bus-snoop intervention beyond what a specific host needs
- exact six-stage timing at the cycle level -- "architecturally-equivalent,
  not cycle-exact" (this is why the pipeline's branch/DBcc misprediction
  recovery, for example, doesn't replicate the real chip's shadow-register
  timing exactly -- see `ap040_pipe_core.v`'s milestone-4/5 notes)
- full hardware FPU before the pipeline reaches LC040-class MMU/cache parity
- replacing `rtl_old` before `rtl/` is independently proven

## 3. `rtl_old`: what's done, for reference

Feature-complete: full integer ISA, 68010/020/030/040 addressing modes,
68040 exception model (formats `$0/$1/$2/$3/$7`), 040 MMU (TTRs, 64-entry
4-way ATCs, three-level table walker, history bits), split I/D cache with
copyback, and the FPU (extended precision + FPSP trap path). Its own
regression suite (`tb/run_tests.sh`) covers all of this; treat any pipeline
milestone as unfinished until it matches `rtl_old`'s behavior for the same
instruction, not just "looks plausible."

`rtl_old`'s test suite needs `vasmm68k_mot` (vbcc) to assemble the `.s` test
programs in `tb/asm/`, in addition to `iverilog`. Neither is preinstalled on
a fresh machine; `iverilog` is a one-line `brew install icarus-verilog`,
`vasmm68k_mot` is not in Homebrew and needs building from the vbcc/vasm
sources separately. The non-assembly legs of that suite (reset, double
fault, walker CDC, bus16 gap, bus timeout, cache snoop) need only `iverilog`
and can be run standalone as a partial check.

## 4. `rtl/`: pipeline status

Six real stages (IF/ID/EA-calc/EA-fetch/EX/WB), synchronous stall/flush
chain, register forwarding (EX-forward mux + regfile write-through),
architectural CCR with its own write-through forward, and a "guess branches
taken, recover on misprediction" front end matching the real 68040's
documented policy (no BTB, always-assume-taken, one-cycle recovery --
see `ap040_pipe_core.v`'s milestone-1 note on the "shadow register" research).

Completed milestones (`tb/run_pipe_tests.sh`, one file per milestone):

| # | Adds | Test |
|---|------|------|
| 1 | Six-stage skeleton, NOP drains cleanly | `tb_ap040_pipe_nop.v` |
| 2 | MOVEQ, register-direct MOVE.L, register forwarding | `tb_ap040_pipe_moveq.v` |
| 3 | ADD.L Dn,Dm, dual-operand forwarding | `tb_ap040_pipe_add.v` |
| 4 | BRA.B, zero-bubble redirect | `tb_ap040_pipe_bra.v` |
| 5 | Bcc.B, misprediction recovery, CCR forwarding | `tb_ap040_pipe_bcc.v` |
| 6 | Scc.B Dn, folder independence (`rtl/` no longer touches `rtl_old/`) | `tb_ap040_pipe_scc.v` |
| 7 | Bcc.W/Bcc.L, multi-word decode gather, `id_next_pc` | `tb_ap040_pipe_bccw.v`, `tb_ap040_pipe_bccl.v` |
| 8 | DBcc Dn,\<label\>, dynamic (runtime-conditioned) register write | `tb_ap040_pipe_dbcc.v` |
| 9a | Unified L1 (`ap040_pipe_l1.v`) replaces IF's inline ROM; port B reserved for data | all of the above, unchanged behavior |
| 9b | MOVE.L (An),Dn -- first memory read, first genuine pipeline stall | `tb_ap040_pipe_move_mem.v` |
| 10 | MOVE.L (d16,An),Dn -- first real EA displacement arithmetic; found and fixed a real L1/stall bug (see below) | `tb_ap040_pipe_move_disp.v` |
| 11 | JMP (An), JMP (d16,An) -- EA-computed PC redirect, reusing EX's mispredict/recovery mechanism unconditionally | `tb_ap040_pipe_jmp.v` |
| 12 | `ap040_pipe_l1.v` posted write buffer (port B write side, ahead of BSR/JSR) | `tb_ap040_pipe_l1_wbuf.v` |
| 13 | BSR (all 3 widths), JSR (An)/(d16,An) -- first real stack push, reusing the write buffer | `tb_ap040_pipe_bsr.v`, `tb_ap040_pipe_jsr.v` |
| 14 | Exception entry: illegal instruction (vector 4), TRAP #n (vectors 32-47), format $0 (4-word) frame -- first exception-entry sequencer | `tb_ap040_pipe_exc.v` |
| 15 | Supervisor state: real 16-bit SR, VBR/SFC/DFC/CACR, MOVEC, MOVE to SR, dynamic privilege violation (vector 8), USP/ISP/MSP banking | `tb_ap040_pipe_sup.v` |
| 16 | RTS (reuses mem_issue/mem_complete verbatim), RTE (new 2-beat pop sequencer, format $0 only) -- the return half of BSR/JSR and illegal/TRAP/priv | `tb_ap040_pipe_rts_rte.v` |
| 17 | Address error (vector 3) on odd JMP/JSR targets -- first format $2 (6-word, 12-byte) frame, dynamic (EA-fetch-time) detection like priv violation, JSR's push suppressed entirely for the faulting case | `tb_ap040_pipe_addrerr.v` |

Register-direct/immediate/register-indirect(+displacement) only -- no
indexed/absolute/PC-relative EA modes, no byte/word sizes yet (BSR/JSR now
done -- see section 6). See section 6.

## 5a. The unified-L1 decision (2026-08-27)

Went with a single dual-port memory shared between instruction fetch and
(eventually) data, instead of separate I/D storage or a private flat-memory
stub per testbench, on the reasoning below -- see `ap040_pipe_l1.v`'s header
for the implementation-level detail.

- **Coherency for free.** A write through the data port is visible to a
  later instruction-fetch read with no explicit invalidation, just from both
  ports addressing the same array -- unlike a split Harvard I/D cache (what
  `rtl_old/ap040_cache.v` and the real 68040 both are), which needs CINV/
  CPUSH to move data from the D side to the I side. This is a real,
  intentional deviation from the real chip's cache architecture, not an
  oversight: `rtl_old` already IS the split-cache reference if bit-exact
  cache behavior is ever needed, and this repo has no backing store of its
  own to make a real miss/fill/replacement policy meaningful yet (no SDRAM/
  DDR model -- unlike the Minimig-AGA tree this core was extracted from).
- **Consequence, not free:** CINV/CPUSH can only ever be decoded/accepted as
  no-ops on this substrate -- there is no staleness for them to fix, so
  their actual observable effect (stale-until-flush) is untestable here.
  `rtl_old`'s `t_cache` suite is where that semantic is actually exercised;
  don't expect the pipeline to ever reproduce it against this L1.
- **CACR's independent I/D enable bits** (the real 68040 lets software
  disable the data cache while leaving the instruction cache on, or vice
  versa) are not a blocker: they become an access-side gate ("does this
  read consult the array or bypass it") over the one shared array, not a
  reason to keep two cache instances. Not implemented yet -- there's no
  CACR in the pipeline at all -- but the port shape doesn't foreclose it.
- **Timing: this was a drop-in, not a stall-infrastructure change**, and
  that was verified, not assumed: `ap040_inst_fetch.v`'s old inline ROM read
  was already registered (1-cycle address-to-data latency, same as a real
  BRAM), so lifting the array out to `ap040_pipe_l1.v` and having IF drive
  its address combinationally reproduces the exact same timing. All 9
  existing pipe tests pass unmodified (only their hierarchical preload path
  changed, `dut.u_if.rom[N]` -> `dut.u_l1.mem[N]`, since the storage moved).
- **Decided (2026-08-27): registered + stalled**, not combinational. Port
  B's read matches `ap040_pipe_l1.v`'s own `q_b <= mem[address_b]` (and
  IF's port A precedent), not `ap040_pipe_regfile.v`'s zero-added-latency
  ports. Chosen over combinational specifically to avoid building a second
  timing model that would need redoing once a real cache-miss path exists
  later.

**DONE (milestone 9b, same day): MOVE.L (An),Dn.** Port B is 32-bit
(read/write, both adjacent 16-bit words in one access -- see
`ap040_pipe_l1.v`'s header for why: this core's internal memory interface
is meant to be 32-bit-native per section 1, with 16-bit-bus splitting left
to a future host adapter, not built into the core). `ap040_ea_fetch.v`
gained a two-state `mem_issue`/`mem_complete` FSM that needed no new
register-forwarding logic at all -- An's value already resolves through the
EXACT SAME `operand_a` priority mux (regfile read + EX-forward) every
register-direct source operand has used since milestone 2, because decode
sets `eac_src_reg` to An's own unified regfile index (8+n) rather than
inventing a separate address-register field. The one-cycle stall reuses the
existing freeze-on-`stall_in` behavior every stage has had since milestone 1
(asserting `eaf_stall` for exactly the issuing cycle freezes EA-calc's
output, so it still describes the same instruction on the completing cycle)
-- no separate pending-instruction latch needed in EA-fetch.

Two real bugs surfaced during development, worth recording:
1. **A genuine RTL bug**, caught by the test before it shipped: the first
   draft had EA-fetch address L1 port B as an ABSOLUTE byte address
   (`operand_a >> 1`), while `ap040_inst_fetch.v` addresses port A
   PC_RESET-RELATIVE (`(fetch_pc - PC_RESET) >> 1`) -- two different
   conventions into the SAME shared array silently breaks the entire point
   of unifying it. Fixed by giving `ap040_ea_fetch.v` the same `PC_RESET`
   parameter and using the identical convention; confirmed the mismatch is
   real and now caught by mutation-testing it back in (reproduces the exact
   `4e714e71` symptom the real bug produced).
2. **A test-methodology bug, not RTL**: the test seeds `A0` via a
   hierarchical poke (`dut.u_regfile.areg[0] = ...`, no MOVEA/LEA yet to
   load it architecturally) placed immediately after `nreset = 1`. A classic
   blocking/non-blocking race: `ap040_pipe_regfile.v`'s own reset block
   samples `nreset` in the SAME edge's active region (still 0) and schedules
   `areg[i] <= 0` as a non-blocking update, which then commits AFTER the
   poke's blocking assignment in the same timestep, silently overwriting it.
   `ap040_pipe_l1.v`'s ROM pokes never hit this because `mem[]` has no reset
   logic at all. Fixed by waiting one more clock edge past `nreset = 1`
   before poking. General lesson for any FUTURE test that pokes a resettable
   register (not memory): the reset edge itself is not a safe time to poke
   across.

Three independent mutations confirmed the mechanism is actually exercised,
not passing by coincidence: dropping `mem_issue` from `eaf_stall` (EA-calc
advances early -> the memory value lands in the WRONG register, D1 instead
of D0, not just a wrong value in the right one), swapping the L1 word order
(byte-swapped-at-the-word-level result), and the PC_RESET-omission mutation
above.

Not exercised by this test, explicitly deferred rather than assumed: EX-
forwarding INTO the address computation (a producer writing An immediately
before this instruction reads it) -- mechanically covered by the reused
`operand_a` mux, but nothing tests it yet since MOVEA/LEA don't exist to
generate that producer. Revisit once they do.

**DONE (milestone 10, 2026-08-27): MOVE.L (d16,An),Dn**, and with it a real,
previously-shipped bug in the unified L1 itself -- found because this was
the first test where a stall from one instruction's own memory access
happened to overlap with a SECOND instruction's multi-word gather still in
flight, a combination milestone 9b's test never produced.

- **The EA arithmetic itself needed almost no new machinery.** `id_imm`
  (previously only MOVEQ's immediate) now carries the sign-extended
  displacement, reusing `ap040_decode.v`'s existing gather machinery (a
  third trigger alongside Bcc.W and DBcc) and the same `gather_disp` wire
  those two already compute. `ap040_ea_fetch.v`'s address became
  `operand_a + eac_imm` unconditionally -- 0 for plain `(An)`, the real
  displacement for `(d16,An)` -- one formula, not a per-mode branch.
- **A real, needed guard**: every prior gather user (Bcc.W/L, DBcc) wants
  IF's speculative redirect; `(d16,An)`'s "displacement" is a memory offset,
  not a branch target, and would have hijacked IF into jumping to a garbage
  address had `redirect_from_gather` not been explicitly gated
  `&& !held_is_move_disp`. Confirmed load-bearing by mutation (dropping the
  guard breaks the test, not just theoretically).
- **A real, load-bearing companion fix**: `id_imm` was previously set to the
  sign-extended opcode low byte for EVERY instruction (harmless when nothing
  consumed it for non-MOVEQ cases). Once `ap040_ea_fetch.v` started actually
  ADDING `eac_imm` to every memory address, the plain `(An)` case would have
  silently added its own opcode's low byte as a phantom displacement.
  Zeroed explicitly for `is_move_mem_l`; confirmed load-bearing by mutation
  (breaks `tb_ap040_pipe_move_mem.v`, the milestone 9b test, when reverted).

**The real find: `ap040_pipe_l1.v` port A had no enable, and would silently
desynchronize from `if_opcode` across any stall longer than the instant it
started.** `ap040_inst_fetch.v`'s internal `pc` register is *always* one
step ahead of `if_pc`/`if_opcode` by design (normal single-stage-prefetch
shape: `pc` is the next fetch address, `if_pc`/`if_opcode` is what's
currently presented, one cycle younger). A stall correctly freezes both
registers together -- but `pc` freezes at its own, already-more-advanced
value. Port A's `q_a` had no idea any of this happened and kept
`<= mem[address_a]` on every single clock edge regardless, and since
`address_a` is a combinational function of `pc`, it kept reading the
address `pc` had *already* moved to -- one word past what `if_opcode` was
supposed to keep presenting -- silently overwriting it after just one
stalled cycle. `tb_ap040_pipe_move_mem.v` never surfaced this because its
own stall's collateral damage always landed on a harmless NOP; this
milestone's case B (a second gather needing its extension word held steady
across an EARLIER, unrelated instruction's memory stall) is what finally
landed the overwrite on a real, non-inert word (`MOVEQ #9,D2`'s own
opcode), turning silent corruption into a wrong register value. Fixed with
a new `en_a` port on `ap040_pipe_l1.v`, wired from `ap040_pipe_core.v` as
`ce && !id_stall` -- the EXACT condition already gating
`ap040_inst_fetch.v`'s own `if_pc`/`pc` registers, so port A now freezes in
lockstep with what it's supposed to represent instead of free-running past
it. Confirmed load-bearing by mutation, reproducing the exact `4e714e71`
garbage-NOP symptom the original bug produced. Port B needed no equivalent
fix -- its address is a direct combinational function of `ap040_ea_calc.v`'s
own already-stall-frozen output, not an independently-advancing register
the way IF's `pc` is; see `ap040_pipe_l1.v`'s header for the full argument
and the flag for revisiting it if that ever changes.

General lesson, worth carrying forward: **a stalled pipeline register
freezing correctly is not sufficient -- everything that register's value
combinationally FEEDS must also stop advancing, including free-running
memories with no natural concept of "stall."** This class of bug is
specifically invisible to single-instruction tests and to tests whose
stalls never overlap with another in-flight multi-cycle operation; it took
a second gather colliding with a first instruction's own memory stall to
surface it. Worth deliberately constructing scenarios like this again once
more stall sources exist (a real L1 miss, a store's write-then-read
hazard), not just testing one mechanism at a time in isolation.

**DONE (milestone 11, 2026-08-27): JMP (An), JMP (d16,An).** The first
control-flow instruction whose target isn't known until a register is
read, and it turned out to need almost no new mechanism at all -- just a
new way to USE two that already existed:

- **EA resolution**: identical to `MOVE.L (An)/(d16,An),Dn` -- same
  `id_src_reg`/`id_imm` fields, same gather machinery for the displacement
  form. What differs is the CONSUMER: a memory-source MOVE dereferences
  the computed address (`ap040_ea_fetch.v`'s `mem_issue`/`mem_complete`
  FSM); JMP never sets `id_is_mem_src`, so it takes the plain,
  non-stalling path, and the computed address itself (`operand_a +
  eac_imm`) is routed straight into `eaf_operand_a` as the result.
- **The redirect**: decode has no literal displacement to speculate a
  target with (unlike Bcc/DBcc), so it doesn't try -- `id_is_jmp` never
  participates in `id_redirect_valid`. Instead `ap040_execute.v` treats an
  unresolved JMP as an UNCONDITIONAL misprediction against IF's implicit
  "keep going sequentially" non-guess, reusing `ex_mispredict`/
  `ex_recovery_pc`/`flush` completely unchanged -- the exact wiring DBcc
  reused from Bcc in milestone 8, now reused a second time for a genuinely
  different kind of instruction. `ex_recovery_pc` becomes `eaf_operand_a`
  (the computed target) instead of `eaf_next_pc` when `eaf_is_jmp`.

Four mutations were tried; three caught cleanly (dropping `eaf_is_jmp`
from `ex_mispredict`, reverting `ex_recovery_pc` to always `eaf_next_pc`,
and routing plain `operand_a` instead of `operand_a + eac_imm` for JMP --
each breaks the test in the way predicted). **The fourth did NOT catch
anything, and that's a real, worth-recording finding, not a gap papered
over**: dropping the `!held_is_jmp` guard on `redirect_from_gather` (the
same guard shape `held_is_move_disp` needed in milestone 10) left the test
passing. Unlike move-disp -- which has NO EX-side correction mechanism at
all, so a wrong decode-time redirect for it is a permanent, unrecoverable
bug -- JMP's target is ALWAYS corrected by EX's unconditional
misprediction regardless of what decode guessed, so a stray extra
speculative redirect only wastes a few cycles of now-discarded fetching;
it cannot survive to be observed. The guard is kept anyway (consistent
shape with move-disp, avoids pointless mis-speculation), but it is
correctly understood as defensive, not load-bearing, for JMP specifically
-- don't claim mutation coverage a test didn't actually demonstrate.

Also surfaced a real test-construction issue, not an RTL bug:
`ap040_inst_fetch.v`'s `issued` counter (the `have_more`/`PROG_WORDS`
budget) counts every word FETCHED, including ones a later misprediction
flush discards. JMP never lets decode speculate a real target, so IF
races several words past EVERY JMP before EX's unconditional-misprediction
correction arrives and redirects it back -- and with TWO chained JMPs in
one test program, `PROG_WORDS=10` (the size every earlier test needed)
silently ran out before the second JMP's real, post-redirect target ever
got to execute. Not a hang -- caught cleanly as a wrong final register
value, but worth remembering: any test chaining multiple JMP/Bcc
mispredictions needs a substantially larger budget than instruction count
alone would suggest.

**DONE (milestone 12, 2026-08-27): posted write buffer on `ap040_pipe_l1.v`
port B**, ahead of actually wiring BSR/JSR's stack push, per the user's
explicit request: get the write PATH right first, not as an afterthought
bolted onto the first store instruction.

- **Why a buffer at all.** Port B is a single read/write port -- one
  address bus serving both `q_b`'s read request and `wren_b`'s write
  request. Without buffering, a store landing the same cycle port B is
  needed for something else (a concurrent read, or an earlier store still
  landing) would force the WHOLE PIPELINE to stall until the port frees
  up. A 1-entry buffer (`wbuf_valid`/`wbuf_addr`/`wbuf_data` -- one 32-bit
  longword; per the user, nothing wider is needed, this core never posts
  more than one store's worth at a time) decouples that: `wren_b` POSTS a
  write, accepted immediately whenever the buffer is empty, and the
  requester can move on without waiting for the physical `mem[]` write to
  land -- the same "commit, don't wait for the backing store" contract a
  real store buffer gives a pipeline.
- **Bounded, not indefinite, wait.** Draining an already-posted write and
  accepting a genuinely new one are mutually exclusive per edge (they're
  sequenced, not simultaneous), so a request arriving while the buffer is
  already busy needs up to two edges from when it FIRST starts waiting:
  one for the old entry to drain, one more to actually accept the new
  one. From the moment `wr_busy` is observed to have already dropped,
  though, a held request is accepted on the very next edge. A first draft
  of both the RTL's own comment and the test's cycle-accounting claimed a
  flat "one cycle" bound, which was wrong by exactly one edge for the
  back-to-back case -- caught by the test itself, not asserted past it.
- **Read-after-write forwarding.** A read whose address matches an
  undrained buffered write returns the buffered value, not stale `mem[]`
  content. Not reachable by any instruction implemented yet (nothing
  reads memory in the same window a store's write might still be
  buffered -- `RTS`, a stack pop, will be the first), built anyway since
  it's the module genuinely responsible for the guarantee and it's cheap;
  don't let it go untested indefinitely once something actually depends
  on it.
- **`wren_b` unconditionally accepted for now.** In this behavioral model
  there is no real port contention beyond the buffer's own single entry
  (draining and reading a DIFFERENT address coexist fine -- Verilog lets
  a behavioral array be written and read at different indices in the same
  block; a real BRAM's actual port count is exactly the kind of thing the
  future BCU has to arbitrate for real, not this module). `wr_busy` is
  real and correctly computed, but nothing in this repo can currently
  drive it to reject a write for longer than the two-edge bound above.
- **A real bug, caught immediately by the existing suite going X the
  moment this milestone's code was added**: `ap040_pipe_l1.v` never had
  an `nreset` before (`q_a`/`q_b` are pure data outputs with no meaningful
  reset value while nothing downstream trusts them yet -- matching real
  BRAM read ports). `wbuf_valid` is different: it's genuine CONTROL
  state, and Verilog gives an un-reset reg `X` at time 0 -- which
  propagated through the read-forwarding ternary and poisoned `q_b` on
  reads that had nothing to do with any write, breaking
  `tb_ap040_pipe_move_mem.v`/`tb_ap040_pipe_move_disp.v` immediately.
  Fixed by giving the module the same `nreset` every other stateful pipe
  module already has, gating `wbuf_valid` specifically (`mem[]`/`q_a`/
  `q_b` keep their original no-reset treatment -- they're still pure
  data).
- **Two testbench-only bugs, not RTL, both worth remembering**: (1) a
  classic active-region-vs-NBA-region race reading `wr_busy` (or setting
  new stimulus) in the SAME simulation instant a testbench process
  resumes from `@(posedge clk)` -- the fix applied everywhere in
  `tb_ap040_pipe_l1_wbuf.v` is `#1` after every edge before touching
  anything DUT-driven, not just where a failure happened to surface (an
  early version passed one check by scheduling luck while an
  IDENTICALLY-shaped later check failed). (2) `check1`/`check32`'s
  message parameter was declared `[255:0]` (32 characters) and silently
  truncated every longer message to its tail, actively misleading the
  first round of debugging by showing a garbled, wrong-looking failure
  reason; fixed by switching to SystemVerilog's unbounded `string` type
  (already available, `-g2012` is required for this whole suite anyway).

All three real mechanisms (read-after-write forwarding, drain-before-
accept priority, and the `nreset` fix) were mutation-tested independently
and caught cleanly; the `nreset` mutation reproduces the exact original
`X`-propagation symptom. Full 13-file `run_pipe_tests.sh` green.

**DONE (milestone 13, 2026-08-27): BSR (all three displacement widths),
JSR (An)/(d16,An)** -- the first instructions to actually drive
`ap040_pipe_l1.v` port B's write side (milestone 12's buffer was built
ahead of a real user, now it has one), and the first stack push. Both
reuse existing machinery almost entirely; very little of this milestone
is genuinely new logic.

- **BSR reuses Bcc's speculative decode-time redirect verbatim.** It's
  the SAME opcode class as Bcc (`0110 cccc dddddddd`), condition `0001`,
  which `is_branch_opcode` has excluded since milestone 4 specifically
  because it needs a stack push. BSR is unconditionally taken (like
  BRA's `cond_true(0)==1` special case), so decode's "assume taken" guess
  is always correct and EX has nothing left to correct -- confirmed by
  `ex_mispredict` deliberately excluding `eaf_is_bsr`. Kept as its own
  `id_is_bsr` flag rather than folded into `id_is_branch`, though: BSR's
  condition-code field bits (`0001`, the literal "F"/always-false
  encoding) would make `ap040_execute.v`'s `cond_true()` check always
  evaluate false and wrongly fire a misprediction if BSR were
  misclassified as a plain branch. Verified against `ap040_core.v`'s
  `S_BCC_EXT` (`ir[11:8]==4'h1` arm) and its byte-form counterpart.
- **JSR reuses JMP's EA resolution and gather completely unchanged**
  (`id_src_reg = An`, `id_imm` = displacement or 0) for the redirect
  target -- one bit different from JMP's own opcode (`0100 1110 10 mmm
  rrr` vs. JMP's `...11...`, verified against `ap040_core.v`'s
  `d_op8_6 == 3'b010` JSR arm vs. `3'b011` JMP arm). Because JSR's target
  isn't known at decode time, it does NOT get BSR/Bcc's speculative
  redirect -- `held_is_jsr` is excluded from `redirect_from_gather`,
  same shape as `held_is_jmp` -- and instead rides `id_is_jsr` through to
  EX exactly like JMP does, reusing `ex_mispredict`/`ex_recovery_pc`
  unconditionally.
- **The push itself needed no new regfile port.** Decode sets
  `id_dest_reg = A7`'s unified index (`4'd15`) for BOTH BSR and JSR --
  the register the push+decrement actually write -- so
  `ap040_ea_fetch.v`'s EXISTING `operand_b` mux (port B, driven by
  `dest_reg`, already correctly EX-forwarded) resolves A7's current
  value for free. `push_addr = operand_b - 4` is computed ONCE in
  EA-fetch and reused both as the L1 write address and as the value
  eventually written back to A7 -- avoiding a second subtraction in EX,
  which already has `eaf_operand_a` carrying JSR's redirect target and
  can't also carry an ALU computation for the same instruction.
- **The write-side stall (`wr_stall`) is simpler than the read-side
  `mem_issue`/`mem_complete` FSM.** A write needs no "wait for a
  response" phase -- just "wait for the port to be free" -- so a single
  `eac_valid && eac_is_push && l1_wr_busy` term bubbles EX for exactly as
  long as the buffer stays busy, re-evaluating the SAME request
  combinationally each cycle with no pending-instruction latch needed
  (mirroring milestone 9b's mem_issue reasoning, but for the write
  direction).
- **No real RTL bugs surfaced this milestone** -- unusual for this
  project's track record, and attributed directly to milestone 12 having
  front-loaded the write buffer's own bugs (the missing `nreset`, the
  drain-priority off-by-one) before any instruction depended on them.
  BSR's testbench passed on its first real compile/run; JSR's did too.
- **Mutation testing, BSR** (`tb_ap040_pipe_bsr.v`, `ap040_decode.v` /
  `ap040_ea_fetch.v`): removing `is_bsr_byte` from `redirect_from_byte`
  -- caught (poison ran). Adding `&& !held_is_bsr` to
  `redirect_from_gather` -- caught (poison ran). Changing
  `l1_data_b` from `eac_next_pc` to `eac_pc` (pushing the BSR's OWN
  address instead of the return address) -- caught (both pushed values
  wrong). All three restored and diff-confirmed clean.
- **Mutation testing, JSR** (`tb_ap040_pipe_jsr.v`, `ap040_decode.v` /
  `ap040_execute.v`): removing `eaf_is_jsr` from `ex_mispredict` --
  caught (both poisons ran, JSR never redirected at all). Removing
  `eaf_is_jsr` from `ex_recovery_pc`'s target-select ternary (falls back
  to `eaf_next_pc`) -- caught (both poisons ran: the "recovery" silently
  redirected PC to the address IF was already fetching anyway, so
  nothing actually changed). Removing `held_is_jsr` from
  `redirect_from_gather` -- **not caught**, and correctly so: exactly
  the same non-finding already recorded for JMP in milestone 11 (see
  above). JSR's `ex_mispredict` fires unconditionally regardless of
  decode's guess, so a bogus decode-time redirect for case B gets
  corrected by EX before anything observable could latch onto the wrong
  path -- the guard is defensive (avoids wasted speculative fetches down
  a memory-offset-as-if-it-were-an-address), not load-bearing for
  correctness, and it would take an instruction-count/cycle-count
  assertion (not built) to actually demonstrate the waste. Documented
  rather than overclaimed, same discipline as JMP's.
- Both testbenches chain two sub-cases end-to-end (case A's target falls
  straight through into case B's BSR/JSR) so normal execution resuming
  cleanly after the push/redirect is proven, not just that the mechanism
  fires once. Both also directly verify A7's post-decrement value and
  both pushed longwords by reading `dut.u_l1.mem[]` hierarchically --
  the same style every earlier test already uses to preload programs,
  so no new memory-model or port-driving scaffolding was needed.

Full 15-file `run_pipe_tests.sh` green (13 pipe-core tests + BSR + JSR +
the standalone `l1_wbuf` test).

**DONE (milestone 14, 2026-08-27): exception entry -- illegal instruction
(vector 4), TRAP #n (vectors 32-47), format $0 (4-word, 8-byte) frame
only.** The user's framing for this milestone: "also jump related but
with more writes" -- accurate in hindsight, since it reuses almost every
mechanism BSR/JSR/JMP already built, just with more of them at once.
Format $2 (address error on an odd JMP/JSR target -- the two vectors
`rtl_old` has real, mode-dependent bit-exact quirks for) is deliberately
its own follow-up milestone, not guessed at here -- see section 6 item 2b.

- **What changed semantically, not just additively**: every opcode this
  decoder doesn't recognize used to ride through as a harmless bubble
  (`id_writes_reg` stays 0, nothing downstream ever consumed `id_unimpl`).
  That bit is now `id_is_illegal`, threaded all the way to a real
  exception. No existing test's program uses an unrecognized opcode
  (confirmed by inspection before relying on it -- none start with the
  0xA/0xF top nibble either, so the still-deferred A-line/F-line split
  has no coverage gap to hide), so this was a safe rename-and-wire, not a
  risky one, but it IS a real behavior change worth flagging plainly.
- **Vector fetch has no real VBR.** This substrate has no control
  registers at all yet (no MOVEC), so vector*4 is used as an ABSOLUTE
  address, fed through the exact same `address - PC_RESET) >> 1`
  conversion every other L1 access already uses -- not a special case.
  This works out cleanly rather than by luck: `PC_RESET` has been `$400`
  in every testbench since milestone 1, which happens to be exactly the
  byte size of a full 256-entry 68k vector table, so the unsigned
  wraparound in that subtraction lands a low vector number in the
  mathematically-correct modular slot of the SAME `ap040_pipe_l1.v`
  array, verified arithmetically (Python, mirroring Verilog's 32-bit
  wraparound then 12-bit truncation) before relying on it, not assumed.
  New test programs need to keep their real code and vector-table pokes
  out of each other's word-index ranges -- worth remembering for any
  future exception test.
- **The exception-entry sequencer lives in `ap040_ea_fetch.v`**, this
  pipeline's first multi-beat memory operation: two posted writes (the
  frame, reusing milestone 12's write buffer exactly like BSR/JSR's
  single write already did, just twice) followed by a plain read (the
  vector table, reusing the SAME `mem_issue`/`mem_complete` one-cycle-
  latency shape MOVE.L's read already established). A 3-state register
  (`exc_ph`: BEAT0/BEAT1/VECRD) sequences them one at a time; each beat
  independently reuses the accept-when-`!l1_wr_busy` retry idiom BSR/
  JSR's `wr_stall` already established, not a new handshake shape.
- **No new regfile port, no new forwarding mux, again.** Decode points
  `id_dest_reg` at A7 for both illegal and TRAP, exactly like BSR/JSR --
  `operand_b` (port B) resolves A7's current, correctly-forwarded value
  for free, and the frame's SR/PC/format-vector words are all computed
  combinationally off already-available fields (`eac_pc`/`eac_next_pc`/
  `eac_imm`/a newly-threaded `ccr_in`) every cycle of the sequence, not
  latched once -- safe because `eac_*` is frozen by the sequence's own
  stall the whole time anyway, so a latch would just be a redundant copy.
- **The illegal-vs-TRAP PC-field distinction is real and load-bearing,
  not a stylistic choice**: illegal instruction stacks its OWN address
  (`eac_pc` -- you can't "return past" an illegal opcode), TRAP stacks
  the FOLLOWING instruction's address (`eac_next_pc` -- TRAP is
  architecturally a subroutine call). Verified against `ap040_core.v`'s
  `go_illegal` (`spc=pc_i`) vs. its TRAP dispatch (`spc=pc`) -- confirmed
  by mutation-testing this exact ternary (see below), not assumed from
  the doc comment alone.
- **SR word is synthesized, not read from a real register**: this
  pipeline has no T1/T0/M/IPL state at all yet (`sr_s`/`sr_m` are still
  hardwired in `ap040_pipe_core.v`), so the system byte is a fixed
  `8'b0010_0000` (S=1, everything else 0, matching `AP040_SR_RESET`'s
  S=1/M=0 but omitting its IPL=111 reset default -- a known, documented
  simplification) with the real, live `ccr_in` (write-through forwarded,
  same source `ap040_execute.v`'s Bcc/Scc condition check already uses)
  supplying the low byte.
- **No real RTL bugs surfaced this milestone** -- the second milestone in
  a row with that track record (see milestone 13's writeup), again
  attributed to reusing thoroughly-debugged machinery (the write buffer,
  the mem_issue/mem_complete read shape, the operand_b-as-A7 trick)
  rather than inventing new mechanisms from scratch. The testbench passed
  on its first real compile/run, including the exact frame word values
  computed by hand/script beforehand.
- **The test deliberately chains through a real JMP, not just
  back-to-back like BSR/JSR's two cases**: the illegal handler executes
  `MOVEQ #7,D2` (marker) then `JMP (A2)` to resume the mainline program at
  the TRAP instruction, proving the exception's flush/redirect composes
  correctly with an ORDINARY, unrelated misprediction recovery
  immediately afterward -- not just that the exception mechanism fires in
  isolation. A7 is also deliberately NOT reseeded between the illegal and
  TRAP cases, so TRAP's frame push is verified from a different base than
  $600, the same "chain through the same register" property BSR's two
  cases already established.
- **Mutation testing**: removing `is_illegal` from `id_dest_reg`'s A7
  selection -- caught, with an instructive cascade (the illegal push
  silently wrote through D0 instead of A7, using D0's untouched value of
  0 as a "stack pointer," landing the frame at a wildly different array
  index; A7 itself never moved, so the SUBSEQUENT TRAP's push then landed
  exactly where the test expected the ILLEGAL frame to be, and the real
  illegal frame's contents were nowhere the test looked -- a clean
  demonstration of why this field matters, not just that it does).
  Flipping illegal's PC-field ternary to also use `eac_next_pc` (matching
  TRAP's) -- caught, and cleanly isolated: only the illegal frame's PC
  word broke, TRAP's stayed correct, confirming the two vectors' PC-field
  semantics are independently exercised, not accidentally coupled.
  Removing `eaf_is_illegal` (only) from `ex_mispredict`'s OR-chain,
  leaving `eaf_is_trap` untouched -- caught (the poison after the illegal
  instruction ran, its handler never did), while TRAP's own case B stayed
  fully passing throughout, confirming illegal's redirect is independently
  load-bearing rather than incidentally covered by TRAP's own working path.
  All three restored and diff-confirmed clean.

Full 16-file `run_pipe_tests.sh` green (14 pipe-core tests + the new
exception test + the standalone `l1_wbuf` test).

**DONE (milestone 15, 2026-08-27): supervisor state -- real 16-bit SR,
VBR/SFC/DFC/CACR, MOVEC, MOVE to SR, dynamic privilege violation (vector
8), and (for the first time) genuine USP/ISP/MSP banking.** The user's
own framing: audit what supervisor state exists, fill the gaps (VBR, the
three stack pointers, SFC, DFC, CACR), and prove mode switching and
privileged-instruction faulting actually work, not just that the
registers exist.

- **`ccr` (5 bits) became `sr` (16 bits)**, matching `AP040_SR_RESET`'s
  exact layout (T1/T0/S/M/-/IPL/-/-/-/CCR). `sr_s`/`sr_m` feeding
  `ap040_pipe_regfile.v`'s A7 bank -- hardwired `1'b1`/`1'b0` since that
  module was first written -- are finally real bits of live state.
  `ap040_pipe_regfile.v`'s `aux_we`/`aux_sel`/`aux_wdata` port -- present
  since that file's first version, never driven until now -- got its
  first real user, for MOVEC's USP/ISP/MSP targets.
- **Four new control registers** (VBR/SFC/DFC/CACR) live directly in
  `ap040_pipe_core.v`, MOVEC-writable/readable. The MMU registers
  (TC/ITT0/ITT1/DTT0/DTT1/URP/SRP/MMUSR) are explicitly out of scope per
  the user's own framing -- `ap040_decode.v`'s MOVEC selector validation
  rejects them as illegal (vector 4), the same treatment any other
  unrecognized construct gets, not silently accepted or dropped. CACR is
  a plain, behaviorally inert register (masked `& 32'h8000_8000` for
  bit-exact readback) -- this substrate's unified L1 has no per-way
  enable/disable concept to actually gate, the same "diminished capacity"
  already flagged for CINV/CPUSH in section 5a.
- **MOVEC's read direction needed no new commit path at all** -- the
  selected control register's current value is fed straight from
  `ap040_pipe_core.v` into `ap040_execute.v` (bypassing
  `ap040_decode.v`/`ap040_ea_calc.v`/`ap040_ea_fetch.v` entirely, since
  these are live architectural state, not per-instruction pipeline data)
  and routed through the SAME `combined_result`/`commit_reg` machinery
  every other GPR-writing instruction already uses.
- **The privilege check is the pipeline's first genuinely DYNAMIC
  exception trigger.** Illegal/TRAP are decode-time facts; whether MOVEC/
  MOVE-to-SR actually FAULTS depends on the live, forwarded S bit, which
  doesn't exist until `ap040_ea_fetch.v` -- `eac_is_priv` is computed
  there and folds into the exact same `eac_is_exc`/`exc_pc_field`/
  `exc_vec_num` machinery illegal/TRAP already built (vector 8, format
  $0, own-address PC field, go_priv's exact convention).
- **A real architectural bug found and fixed BEFORE it could hide in an
  untested corner**: an exception's own frame push must ALWAYS land on a
  SUPERVISOR stack (ISP, or MSP if M=1), never on whichever bank happens
  to be currently active -- a privilege violation taken WHILE ALREADY IN
  USER MODE would otherwise silently push its own exception frame onto
  USP. Confirmed wrong against `ap040_core.v`'s own S_EXC0/S_EXC1
  ordering (`sr[13]<=1` commits a full cycle before `dbg_a7` -- itself
  bank-selected off the now-already-supervisor `sr` -- is read for the
  stack pointer) before fixing, not assumed. Fixed by having
  `ap040_ea_fetch.v` read `isp_in`/`msp_in` DIRECTLY (bypassing port B/A7
  entirely for this one purpose) rather than through whatever bank is
  currently active -- which, as a side effect, made the earlier
  `eff_dest_reg` port-B-rerouting mechanism unnecessary and it was
  removed, a net simplification alongside the fix.
- **A second real hazard, found by the test actually failing, not by
  inspection**: `sr_resolved`'s write-through forward (mirroring the
  regfile's own bypass and CCR's prior version of the same mux) covered
  "a producer committing THIS cycle," but not "a producer still IN EX,
  one stage ahead" -- exactly the gap `ex_fwd_*` already exists to close
  for GPRs, just never needed for SR before now because nothing upstream
  of EX ever consumed live SR until `ap040_ea_fetch.v`'s privilege check
  did. A BSR one instruction behind a `MOVE to SR` that had just dropped
  to user mode read A7 through the STALE, pre-switch (supervisor) bank
  for exactly one cycle, banking its push onto ISP instead of USP.
  Diagnosed with a cycle-by-cycle trace (not guessed), fixed by adding
  `ex_sr_fwd_valid`/`ex_sr_fwd_data` -- SR's own EX-live forward,
  mirroring `ex_fwd_*`'s shape exactly, now the highest-priority term in
  `sr_resolved`. Fixing this closed off a genuine combinational-loop risk
  too: `ap040_execute.v`'s own exception-masking arithmetic used to read
  a live `sr_in` port fed by `sr_resolved` -- which, once `sr_resolved`
  itself depends on EX's own forward output, would have closed a loop
  through EX's own input. Resolved by having `ap040_ea_fetch.v` thread
  its OWN already-correct, one-cycle-earlier read down as
  `eaf_sr_snapshot` instead, removing `ap040_execute.v`'s `sr_in` port
  entirely rather than papering over the loop.
- **`tb_ap040_pipe_sup.v` chains four phases** (MOVEC read/write for all
  seven control registers -> MOVE-to-SR drops to user mode -> an
  ordinary BSR in user mode proves USP banking -> a privileged MOVEC in
  user mode faults, proving the frame still lands on ISP, untouched USP)
  through ONE continuous program, deliberately reusing each register's
  phase-1 source value as an implicit "poison never overwrote me" proof
  later on (only eight registers exist for four phases' worth of
  independent checks) rather than needing fresh ones. Two honest
  testbench-only bugs surfaced and were fixed, not the RTL: a `MOVEQ
  #$99,D5` marker asserted against zero-extended `$99` instead of the
  correct sign-extended `$FFFFFF99` (real MOVEQ semantics), and an
  "SR.S==0" check asserted against the FINAL simulation snapshot even
  though phase 4's exception legitimately forces S back to 1 by
  then -- removed in favor of the indirect proof phase 3's successful USP
  banking already provides.
- **Mutation testing**: removing `ex_sr_fwd_valid`'s priority term from
  `sr_resolved` -- caught, reproducing the EXACT original bug (USP off by
  the ISP-vs-USP arithmetic difference) byte-for-byte. Forcing
  `exc_sp_bank` to the naive "currently active bank" (`operand_b`)
  instead of `isp_in`/`msp_in` -- caught (the privilege-violation frame
  landed on the wrong stack). Removing the `!sr_in[13]` gate from
  `eac_is_priv` (privilege-violating unconditionally, even in
  supervisor mode) -- caught with a total cascade, every MOVEC in phase 1
  faulting instead of succeeding. Removing VBR's selector from
  `movec_sel_valid` -- caught with the same total-cascade signature,
  confirming an invalid MOVEC selector's illegal-instruction path is
  real, not just decoded-and-ignored. All four restored and
  diff-confirmed clean.
- **Deliberately NOT built**: `MOVE from SR` and `MOVE An,USP`/`MOVE
  USP,An` (MOVEC's own selector set already gives read/write access to
  USP/ISP/MSP, making the dedicated USP opcode redundant for this
  milestone's goals; SR's value is fully observable via the new `dbg_sr`
  tap without needing an ISA instruction for it). VBR is now a REAL,
  settable/readable register, but the exception-entry sequencer still
  does NOT consult it -- vector fetch still uses the PC_RESET-relative
  convention milestone 14 established; wiring VBR in is explicitly
  deferred, not forgotten.

Full 17-file `run_pipe_tests.sh` green (15 pipe-core tests + the new
supervisor-state test + the standalone `l1_wbuf` test).

**DONE (milestone 16, 2026-08-27): RTS, RTE -- the return half of BSR/JSR's
push and illegal/TRAP/priv's exception-entry push.** This pipeline can now
actually return, not just enter -- closing the biggest gap flagged when
milestone 15 shipped.

- **RTS is deliberately not a new mechanism.** It reuses
  `mem_issue`/`mem_complete` (MOVE.L (An),Dn's own FSM) verbatim -- a pop
  is structurally just a 32-bit read from (A7) with the register hardcoded
  instead of decoded from opcode bits. The only real addition is
  `mem_complete`'s `eaf_operand_b`, which now also carries A7's NEW
  (post-pop) value forward for the commit -- one ternary, not a new state
  machine.
- **RTE needed genuinely new machinery**: a privilege check (reusing
  milestone 15's `eac_is_priv` mechanism -- RTE joins MOVE-to-SR/MOVEC as a
  third dynamic-fault source) gates a real supervisor RTE into its own
  2-beat READ sequencer, the mirror image of the exception-entry
  sequencer's own WRITE beats, reading back the exact two dwords a
  format-$0 push wrote. **Format $0 is assumed unconditionally** once the
  pop completes -- this pipeline has no mechanism that could ever have
  pushed anything else yet, so a real FMTERR fallback for an unrecognized
  frame format is deliberately deferred, not overlooked (see section 6
  item 2b).
- **A real architectural race, found by the test failing, not by
  inspection**: RTE's SR restore and its A7 restore commit on the exact
  SAME cycle. Routing the A7 write through the normal `commit_reg`/A7-
  bank-selected path (same as every other A7-touching instruction) meant
  `ap040_pipe_core.v`'s `sr_resolved` -- correctly, per milestone 15's own
  fix -- made the SR restore visible to THAT SAME cycle's bank selection,
  so RTE's own A7 write got banked through the NEW (post-restore) S bit
  instead of the OLD one active while the frame was actually being popped
  -- silently landing the restored ISP/MSP value in USP whenever RTE
  returned to a different mode than it ran in. Fixed by routing RTE's A7
  restore through the SAME direct-to-ISP/MSP path (`exe_writes_creg`, the
  aux port) the exception entry's own push already uses, bypassing the
  live bank entirely -- not through `commit_reg` at all.
- `tb_ap040_pipe_rts_rte.v` chains two round trips end to end: plain
  BSR/RTS in supervisor mode (verified three ways -- the subroutine ran,
  execution resumed at the exact correct return address, and ISP is
  bit-for-bit back to its pre-call value), then TRAP taken FROM user mode
  with RTE popping the frame back out (verified: the frame landed on ISP
  not USP even though the fault occurred in user mode, exactly the
  scenario milestone 15's own fix targeted but had never been exercised
  with a REAL RTE consuming the frame afterward; PC and the full SR both
  restored exactly; a third, post-RTE BSR proves USP banking still works
  correctly afterward, and ISP ends up exactly back at its starting value
  once both round trips are complete).
- **Two real testbench-construction bugs, not RTL, both worth
  remembering**: (1) an early draft placed the BSR/RTS return point
  immediately before the subroutine's own body in memory -- RTS's return
  fell straight through and re-entered the subroutine a second time,
  caught by tracing cycle-by-cycle, not by inspection. (2) the program had
  no proper termination after its interesting part finished: plain NOP
  padding let execution free-run past it, off the end into uninitialized
  (illegal-instruction) memory, tripping an UNRELATED exception that
  corrupted the final ISP/USP/register snapshot before the test ever
  checked it. Fixed with a tight `BRA.B <-2>` self-loop instead of NOP
  padding -- the established fix is durable regardless of how generous a
  future test's cycle budget gets, where a fixed NOP count is not.
- **Mutation testing**: removing RTE's own commit path from
  `exe_writes_creg_c` (caught -- ISP never restored). Removing RTS's `+4`
  new-A7 arithmetic (caught, with an instructive cascade through the
  following TRAP push). Removing RTE's SR-restore data source from
  `exe_sr_data_c` -- **not caught on the first pass**: the wrong fallback
  value (the popped PC's low 16 bits) happened to also read S=0 for this
  program's specific addresses, and the test was only checking `dbg_sr[13]`,
  not the full register. Strengthened to assert the exact known SR value
  (`16'h0000`) instead of one bit, which then caught it cleanly -- a
  genuine improvement to the test, not a workaround, and now a permanent
  part of the suite. Forcing RTE's restore to always target USP instead of
  ISP/MSP -- caught with a cascade through the rest of the program. All
  four restored and diff-confirmed clean.

Full 18-file `run_pipe_tests.sh` green (16 pipe-core tests + the new
RTS/RTE test + the standalone `l1_wbuf` test).

**DONE (milestone 17, 2026-08-27): address error (vector 3) on odd JMP/JSR
targets -- this pipeline's first format $2 (6-word, 12-byte) exception
frame.** Every earlier exception (illegal, TRAP, priv) is format $0 (4-word,
8-byte); format $2 adds one extra 32-bit "faulting address" longword.

- **Detection is entirely dynamic, at EA-fetch time, against the live,
  forwarded EA target** (`ea_target[0]`) -- exactly like milestone 15's
  privilege-violation check, and for the same reason: whether a JMP/JSR
  target is odd isn't known until the address itself is resolved (register
  value + displacement), which doesn't exist until `ap040_ea_fetch.v`.
  `ap040_decode.v` needs ZERO changes for this milestone (its banner was
  updated, its logic wasn't) -- the same "thread the flag through unchanged"
  shape every dynamic-fault milestone since 15 has followed.
- **JMP and JSR are genuinely different, bit-exact-verified against
  `rtl_old/ap040_core.v`'s `S_JMP1`/`S_JSR1`, not assumed to be the same
  shape just because both are "odd target, format $2":**
  - JMP's stacked PC field is the JMP instruction's OWN address + 2 (the
    faulting instruction fetch never even started forming a new PC).
  - JSR's stacked PC field is the odd TARGET itself, raw and unrounded --
    the fault is on the instruction fetch AT the target, so the frame names
    that address, not the call site.
  - JSR's stack push (the return address that would let the callee RTS
    back) is skipped ENTIRELY for the odd-target case -- there is no return
    address to protect if the call itself never completes. `eac_is_push`
    now excludes `eac_is_jsr_odd` explicitly; ISP moves by exactly one
    format-$2 frame (12 bytes), never 12+4 (a frame plus an orphaned push).
  - Both cases' extra address-field longword IS rounded down
    (`{ea_target[31:1], 1'b0}`) -- this is NOT the same value as JSR's PC
    field, a real, easy-to-get-backwards distinction the test isolates
    separately.
- **Mechanically, the exception-entry sequencer just grew a third write
  beat** (`EXC_BEAT2`, format-$2-only, selected by a new `eac_is_fmt2`),
  reusing the exact same beat/vec-read/finalize shape illegal/TRAP/priv/
  RTE's own push already established -- no new state machine, no new
  control-flow shape, just one more localparam value and one more `case`
  arm.
- **A real testbench-construction bug, not RTL, caught the same way every
  prior one in this project has been: tracing, not inspection.** The first
  draft of `tb_ap040_pipe_addrerr.v` gave its JMP-odd handler an
  unconditional JMP back to the (also odd) JSR instruction, expecting a
  SEPARATE handler at a different address to catch the JSR fault -- but
  address error is ONE vector (3) for every odd-target fault, JMP or JSR
  alike; there is no way to route the two cases to different handlers via
  the vector table. The JSR re-faulted through the SAME vector back into
  the SAME handler, which unconditionally jumped back to the JSR again --
  an infinite loop, silently draining the stack by one 12-byte frame per
  pass forever. Visible in the failing run only as "D6 never became $22"
  and "ISP landed at a value that didn't match the two-frames-total
  arithmetic" -- not obviously an infinite loop from the failure message
  alone. A cycle-by-cycle trace of `eac_pc`/`eaf_pc`/`isp` (watching ISP
  decrement by 12 every ~10 cycles while PC kept bouncing back to the same
  handler address) made it unambiguous. Fixed by merging the two handlers
  into one, using an otherwise-unused data register as a one-shot entry
  counter (`ADD.L D7,D7` to test-and-double it, `BNE.B` to branch on the
  second entry) -- a real fix to the test's control flow, not a change to
  what it verifies.
- **Mutation testing**: removing `eac_is_jsr_odd`'s exclusion from
  `eac_is_push` (caught -- the poisoned push corrupts everything
  downstream: D4/D5 poison instructions run, the JSR frame's own beat data
  goes wrong, CCR ends up nonzero). Forcing `exc_frame_size` to always be 8
  (format $0's size) regardless of `eac_is_fmt2` (caught -- every
  format-$2 field lands at the wrong beat offset, ISP arithmetic wrong).
  Swapping JMP's `eac_pc+2` and JSR's `ea_target` PC-field formulas (caught
  -- both frames' PC field wrong, isolated exactly to the swapped fields,
  nothing else affected, confirming the test's granularity). Removing
  `exc_addr_field`'s LSB-clearing (caught -- both address fields wrong by
  exactly 1, the raw odd target instead of the rounded one). Breaking
  `EXC_BEAT2`'s case-statement advance so format-$2 exceptions skip straight
  from `EXC_BEAT1` to `EXC_VECRD` (caught -- the address-field beat never
  gets written, both frames' third word reads back as whatever was already
  in memory). All five restored and diff-confirmed clean against the
  pre-mutation file.

Full 19-file `run_pipe_tests.sh` green (17 pipe-core tests + the new
address-error test + the standalone `l1_wbuf` test).

## 5. Verification discipline (keep doing this)

This is what has actually kept the pipeline correct across twelve milestones
of rewrites; don't relax it for speed:

- **Bit-exact opcode/semantics verification against `rtl_old`.** Every
  decode pattern and execute-stage behavior added to `rtl/` cites the
  `rtl_old/ap040_core.v` line range it was checked against, not just "looks
  like the manual." When the two disagree, `rtl_old` wins (it's the
  cputest-validated one) unless there's a documented reason (see DBcc's
  deferred odd-target-parity check for the shape of that reason).
- **Mutation testing.** After a testbench passes, break the mechanism it's
  supposed to prove (force a mux to always miss, remove a term from a
  condition, etc.) and confirm the test actually fails. Several real test
  bugs (wrong immediates, poison instructions reachable via the CORRECT
  path because they sat next in memory, mutations that happen to converge
  on the same value as the correct path) were only caught this way -- see
  the milestone notes inside `ap040_decode.v`/`ap040_execute.v` and
  `tb_ap040_pipe_dbcc.v` for worked examples. A test that has never been
  seen to fail on broken RTL proves nothing.
- **Folder independence.** `rtl/ap040_pipe_*` never reaches into `rtl_old/`
  and vice versa (milestone 6). If a shared constant or helper looks
  tempting to factor out, don't -- fork it, the same way `ap040_pipe_alu.v`/
  `ap040_pipe_regfile.v`/`ap040_pipe_defs.svh` already do.
- **Scope discipline.** Defer, explicitly and in a comment, anything that
  needs a mechanism that doesn't exist yet (e.g. TRAPcc and DBcc's
  odd-target fault both need exception delivery -- neither is silently
  half-implemented to avoid admitting that).
- **Test stall OVERLAP, not just stalls in isolation.** Milestone 10 found a
  real, already-shipped bug (`ap040_pipe_l1.v` port A free-running past a
  stall -- see section 5a) that no single-mechanism test could have caught:
  it only manifests when a stall from one in-flight operation collides with
  a SECOND, independent multi-cycle operation (a gather) still assembling.
  When adding a new stall source, deliberately construct a test where it
  overlaps something else already in flight, not just a test of the new
  stall by itself.
- **`#1` after every `@(posedge clk)` before touching anything DUT-driven,
  no exceptions.** Milestone 12's write-buffer test lost real debugging
  time to an active-region-vs-NBA-region race: reading a signal (or
  setting new stimulus) in the same simulation instant a testbench
  process resumes from an edge can see pre-edge state, and WHICH checks
  are affected is simulator-scheduling-order-dependent -- one check can
  pass by luck while a structurally identical one fails. Apply `#1`
  uniformly, not just where a failure happened to show up.
- **Give test-helper task message parameters an unbounded `string` type,
  not a fixed-width `[N:0]` vector.** The SAME milestone's `check1`/
  `check32` tasks used `[255:0]` and silently truncated every message
  over 32 characters to its TAIL, actively misleading debugging (a
  garbled, plausible-looking wrong message, not an obvious truncation
  error). `-g2012` is already required for this whole suite; `string`
  costs nothing extra.

## 5b. Milestones 18-29 (2026-09-19): decode reach was the whole constraint

Twelve milestones landed in one run on branch `ap040-pipelined`, taking the
decoder from **2 of `ap040_pipe_alu.v`'s 33 operations to 31**, plus
immediate operands in two encodings and absolute addressing. 31/31 benches,
each one failing on the RTL immediately before it.

| ms | opened | cost |
|---|---|---|
| 18 | byte and word sizes | a size field through all four stages |
| 19 | OR, SUB, CMP, AND | predicate + op map |
| 20 | NEGX, CLR, NEG, NOT, TST | predicate + op map |
| 21 | SWAP, EXT.W, EXT.L, EXTB.L | predicate + op map |
| 22 | ADDX, SUBX | predicate + op map |
| 23 | 8 shifts/rotates, immediate count | a count field |
| 24 | BTST, BCHG, BCLR, BSET | first `src_reg` override |
| 25 | ABCD, SBCD, NBCD, TAS | predicate + op map |
| 26 | ORI, ANDI, SUBI, ADDI, EORI, CMPI | first non-register source |
| 27 | ADDQ, SUBQ | predicate only |
| 28 | `MOVE #imm,Dn` | reuses 26's gather |
| 29 | `MOVE.L (xxx).W/.L,Dn` | first address with no register term |

**The ALU needed nothing.** `ap040_pipe_alu.v` differs from
`rtl/ap040/ap040_alu.v` only in comments, module name and include path --
the logic is byte-for-byte identical, and that file passes 3,797/3,801
cputest slices through the FSM core. So the flag semantics of every
operation above are already validated upstream; what these milestones
changed is decode, which is exactly what each bench tests by producing a
wrong value when the op, size or operand is wrong.

**Area tracks distinct ALU operations reached, not instructions decoded.**
Standalone Quartus fits with `L1_AW=4` (see the warning in section 7 about
the L1 model):

| | ms 17 | ms 20 | ms 24 | ms 27 |
|---|---:|---:|---:|---:|
| ALMs | 1,940 | 2,366 | 3,404 | 3,391 |
| ALU ALUTs | 36 | 526 | 2,035 | 2,034 |
| ops reachable | 2 | 10 | 27 | 31 |

The ALU grew 56x with its RTL unchanged. Milestones 25-27 then added four
instruction families and the fit went slightly DOWN, because they all map
onto operations already instantiated. Any area comparison against `rtl_old`
is meaningless except at equal ISA coverage.

Three traps worth not rediscovering. A completing gather speculatively
redirects to `held_pc + 2 + gather_disp`, which is right for a branch
displacement and wrong for an operand -- immediates are the fifth class to
need excluding there. A new gathering class must be added to the guard that
STARTS a gather, not only to the branch that completes one. And
`tb_ap040_pipe_exc.v` used `0000` as its "matches nothing" opcode, which is
`ORI.B #imm,D0` and started decoding as one at milestone 26; it now uses
`4AFC`, illegal by definition rather than by omission.

## 6a. THE NEXT DECISION: a second GPR write port

`(An)+` and `-(An)` are where this run stops, and not for decode reasons.
`MOVE.L (An)+,Dn` writes TWO registers -- the data to Dn and the updated
address to An -- and the pipeline has exactly one commit path,
`commit_reg`/`exe_dest_reg`. The regfile's `aux_we`/`aux_sel` port is not a
way out: it reaches only USP/ISP/MSP, not a GPR.

So the work is a second write port on `ap040_pipe_regfile.v` AND a second
forwarding path, because `ex_fwd_dest`/`ex_fwd_data` forward one register
and an instruction immediately after `MOVE.L (A0)+,D0` may read A0.

The fault-ordering question section 6 raises ("when does the An update
commit relative to a fault") is currently MOOT and should be recorded as
such: `ap040_pipe_l1.v` is a flat array that cannot fault, so nothing in
this pipeline can fault a memory access at all. The An update can ride the
ordinary commit path with no undo log. That stops being true the moment a
real MMU or bus-error path arrives, which is the same boundary
`ap040_writeback.v`'s header already names.

## 6. Remaining scope, roughly in dependency order

1. **DONE (milestones 9b/10): `MOVE.L (An),Dn` and `MOVE.L (d16,An),Dn`** --
   see section 5a. Next EA-mode candidates, in increasing order of new
   machinery needed: `(An)+`/`-(An)` (An update -- needs a real WRITE to the
   regfile from EA-calc/EA-fetch, not just a read, and a decision on WHEN
   the update commits relative to a fault), then indexed/absolute/
   PC-relative.
2. **DONE (milestone 11): `JMP (An)`, `JMP (d16,An)`** -- see the writeup
   above section 5. **DONE (milestone 12): the write PATH itself** --
   `ap040_pipe_l1.v` port B now has a real posted write buffer
   (`wren_b`/`data_b`/`wr_busy`, 1 entry, drain-then-accept, read-after-
   write forwarding) -- see the writeup above section 5. **DONE
   (milestone 13): `BSR` (all 3 widths), `JSR (An)`/`JSR (d16,An)`** --
   the actual push (address = `A7-4`, data = the return address) plus
   A7's decrement, both landing on the same instruction as the memory
   access; turned out to need no second, non-ALU register-write source
   after all -- decode already routes A7 through the existing `dest_reg`
   port, and `operand_b - 4` is written back through the SAME
   `commit_reg` path every other instruction uses. See the writeup above
   section 5 for the mutation-testing results, including an honest
   non-finding on JSR's `redirect_from_gather` guard (mirrors JMP's).
   **`RTS`** (a read from `(A7)+`, register-indirect-postincrement into
   the PC) is next -- the natural close-out now that a push exists to
   test pop symmetry against, and the first instruction to actually
   exercise the write buffer's read-after-write forwarding path (built
   in milestone 12, never yet reachable by any implemented instruction).
2b. **DONE (milestone 14): illegal-instruction / exception delivery**,
   format $0 (4-word frame) only -- vector fetch (`VBR` conceptually
   equals `PC_RESET`, no real control register yet) + supervisor stack
   frame push, for vector 4 (illegal instruction -- what used to be a
   silent bubble now genuinely traps) and TRAP #n (vectors 32-47, new
   opcode). See the writeup above section 5. **DONE (milestone 15):
   supervisor state** -- real 16-bit SR, VBR/SFC/DFC/CACR, MOVEC, MOVE to
   SR, and privilege violation (vector 8, format $0, genuinely DYNAMIC --
   the first exception trigger that isn't a decode-time fact) are all
   real now, with USP/ISP/MSP banking proven end-to-end (an ordinary BSR
   in user mode correctly targets USP; an exception taken from user mode
   still correctly lands on ISP/MSP, never USP -- a real bug caught and
   fixed before it could hide, see the writeup above section 5). **DONE
   (milestone 17): format $2** (the 6-word frame, one extra "instruction
   address" longword) for **address error on an odd JMP/JSR target** --
   `rtl_old`'s S_JMP1/S_JSR1 real, mode-dependent bit-exact quirks (JMP's
   stacked PC is `pc_i+2` regardless of gather width; JSR's is the target
   itself, and JSR skips its push entirely rather than attempting one at
   an odd address) are now built and verified here, see the writeup above
   section 5. **`MOVE from SR`, `MOVE An,USP`/`MOVE
   USP,An`** deliberately not built (MOVEC's own selector set already
   covers USP/ISP/MSP; SR's value is observable via the new `dbg_sr` tap
   without needing an ISA instruction for it) -- add them if/when
   something in this repo actually needs the opcodes, not preemptively.
   **VBR exists but isn't consulted yet** -- vector fetch still uses the
   PC_RESET-relative convention milestone 14 established; wiring a
   settable VBR into the actual vector-fetch address is separate,
   deferred work. **DONE (milestone 16): `RTS`, `RTE`** -- the return path
   now exists (RTS reuses `mem_issue`/`mem_complete` verbatim; RTE gets its
   own 2-beat pop sequencer, format $0 only), proven with a real, chained
   round trip (BSR/RTS, then TRAP-from-user-mode/RTE) rather than just
   inspecting pushed frame contents -- see the writeup above section 5,
   including a real same-cycle SR/A7-restore race it caught and fixed.
   **RESOLVED (milestone 54): RTE now sizes its pop by the format nibble.**
   It had read that nibble off the stack from the beginning and ignored it.
   Only the frame SIZE differs between $0 and $2 -- the SR, PC and format
   word sit at the same offsets, and format $2's extra longword is the
   faulting address, which this core has no use for on return. So the defect
   was never a failed return; it was A7 coming back four bytes low, once per
   return, with the stack walking downward. `tb_ap040_pipe_rte_fmt2.v`
   therefore checks A7 rather than the resumption, since a bench that only
   confirmed "we got back" would have passed against this for as long as it
   existed.

   **FMTERR: DONE (milestone 76).** A format nibble that is neither $0 nor
   $2/$3 raises vector 14 from inside the pop, stacking the RTE's own
   address with A7 untouched; `tb_ap040_pipe_fmterr.v` repairs the frame
   from the handler and re-executes the RTE. The note that stood here until
   then, kept for the reasoning: raising it means starting an exception
   from a branch that is already mid-pop, and nothing in this core pushes
   any other format -- both still true; the first turned out to be one
   wire (`fmterr_now`) because `exc_writing` already outranks `ret_done`
   in EA-fetch's output chain.

   The original note, kept for the reasoning:

   RTE's own format-$0-only assumption is now a REAL gap, not a moot one:
   since milestone 17, this pipeline CAN push a format-$2 frame (address
   error), but RTE still only knows how to pop format $0 -- an RTE
   returning from an address-error handler would misread the frame (and
   leave the extra address-field longword on the stack). No FMTERR
   fallback either. Deliberately still deferred, now genuinely reachable
   rather than hypothetical -- next in line if anything in this repo
   needs to return from an address-error handler.
   Also still deferred: **TRAPcc**, **DBcc's branch-target parity check**,
   **CHK**/**zero-divide** (no such instructions exist yet), **trace**,
   and **bus/access-fault format $7** (needs a real BCU/MMU, explicitly
   deferred with that milestone per section 5's write-buffer note).
3. **DONE (milestones 18, 35-38): byte and word sizes throughout.** The ALU
   port was already there; decode, a size field across all four stages, a
   read-side lane select, a size-dependent auto-increment step (with the
   68000's A7 byte exception), a held size for the gathering modes, and byte
   enables on the L1's write port. Every load and store mode is now sized.

   One judgement recorded here was wrong and is corrected rather than
   quietly dropped: milestone 35 deferred sized STORES on the grounds that
   byte lanes in a placeholder L1 would be throwaway work. They are not. A
   behavioural array writes a half-word as easily as a whole one, and a real
   block RAM has byte enables anyway, so the change is both small and
   representative of whatever replaces the model.

   The subtle part was not the write but the READ-AFTER-WRITE forward, which
   returned the buffered longword whole -- correct when all four lanes are
   written, and fabricating three of them when one is. It merges now.

   ORIGINAL TEXT: **Byte/word-sized ALU ops and MOVE** (current pipeline is Long-only
   throughout `ap040_pipe_alu.v`'s size port is already there and unused).

   **Milestone 39 (memory-source ALU)** finished the other half of that
   sentence. Sizes were complete, but every binary ALU operation still
   demanded both operands in registers: `ADD.L (A0),D0` -- the form compiled
   code actually emits -- did not decode. Extending the family's shape from
   ea mode `000` to `010` reached OR/SUB/CMP/AND/ADD against memory with no
   new operand plumbing at all, because `ap040_ea_fetch.v` already replaces
   `eaf_operand_a` with the loaded word and the ALU computes `b op a`. The
   only real work was pointing `id_src_reg` at the ADDRESS register.

   It also hit, for the third time, the trap recorded under milestone 9b:
   `id_imm` defaults to the sign-extended low opcode BYTE (MOVEQ needs
   that), and `ap040_ea_fetch.v` adds `eac_imm` to every memory address. Any
   new mode with no displacement must be added to that zero list or it reads
   its own opcode as an offset -- `ADD.L (A0),D0` went to `A0 + $FFFFFF90`,
   landing back in the instruction stream, which is why its bench checked a
   COMPOSITION of two operations rather than one result: a wrong address
   that still decodes produces a plausible number.

   **Milestone 40 (displacement mode)** carried the same family to ea mode
   101, `(d16,An)` -- the mode compiled code leans on hardest, since every
   struct field and every stack-frame local is a displacement off a base
   register. Unlike mode 010 this could not be a wire change, because the
   displacement is an extension word, so it became the EIGHTH kind on the
   shared gather state machine rather than a parallel mechanism.

   It cost almost no new state, which is the payoff for having kept that
   machine shared: `held_reg` is already An, `held_dest_reg` already Dn,
   `held_mv_size` already the size, and `gather_disp` is already the
   sign-extended displacement `MOVE.L (d16,An),Dn` has fed into `id_imm`
   since milestone 10. The one genuinely new field is `held_alu_op` -- every
   earlier gather kind had a FIXED operation and could pick it from the kind
   flags alone, and this is the first that cannot. The size wire had to be
   chosen rather than shared: the ALU family sizes from `ir[7:6]`, MOVE from
   `ir[13:12]` with a different encoding.

   Like `MOVE.L (d16,An),Dn` it must be excluded from
   `redirect_from_gather`, which fires for every gather kind that MIGHT
   branch. That is now five exclusions against three branch kinds, and the
   gate is written as a list of negations -- the next non-branching gather
   kind that forgets to add itself will jump to `held_pc + 2 + disp`.

   **Milestone 41 (autoincrement)** added modes 011 and 100, `(An)+` and
   `-(An)`, for one shape term and two flags -- the address-register update
   already existed, driven off `eac_is_postinc`/`eac_is_predec` through
   milestone 30's second write port. `ADD.L (A0)+,D0` walking an array is
   what a compiler emits for a summation, and it is one instruction where
   the FSM core needs several.

   It exposed something that had been true but untested since milestone 30:
   `ap040_ea_fetch.v`'s `an_write` is independent of `writes_reg`. Every
   earlier user of it also wrote a data register, so the two were
   indistinguishable. `CMP.L (A0)+,D0` writes NO data register and must
   still advance A0 -- an implementation that gated the address update on
   "does this instruction write a register" would have passed every bench
   written before this one.

   The register-indirect source modes are now complete for this family:
   `Dn`, `(An)`, `(An)+`, `-(An)`, `(d16,An)`. Still absent: `An` direct
   (mode 001), indexed `(d8,An,Xn)` (110, needs a third register read port),
   the absolute and PC-relative modes (111), and the whole `ir[8]=1`
   direction -- ALU-to-memory, which is a read-modify-write and the first
   instruction that would need both a load and a store.

   **Milestone 42 (LEA)** is in every function prologue and every array
   index, and was the cheapest useful instruction left: it produces an
   address and writes it to An, reading no memory and setting no condition
   codes. Its entire datapath cost is one term on one ternary --
   `ap040_ea_fetch.v` already routes `ea_target` into `eaf_operand_a` for
   JMP and JSR, and with `id_alu_op = MOVE` the address then lands in An
   through the ordinary writeback. The rest is decode plus a flag threaded
   through EA-calc, and mode 101 is the NINTH kind on the shared gather.

   The absolute forms were deliberately not added: `LEA (xxx).L,An` is
   `MOVEA.L #imm,An`, which `held_is_imm` has assembled since milestone 26.

   Writing its bench turned up a real hole rather than a bug: **`MOVE.L
   An,Dn` does not decode.** `is_move_rr` requires source mode 000, so
   source mode 001 -- an address register as the source of anything -- has
   no entry anywhere in this decoder. That affects the ALU family from
   milestones 39-41 as well (`ADD.L A0,D0` is legal 68k for .W and .L) and
   is the smallest remaining gap in the integer core.

   ### Next, in rough order of value

   1. ~~Source mode 001~~ **DONE (milestone 43).** Pure decode, as
      predicted: An lives in the same 4-bit unified register space the
      decode stage already uses, so naming it in `id_src_reg` was the whole
      change. The care went into NOT overreaching -- Byte is never allowed
      with an address register (and the exclusion has to be written twice,
      since MOVE encodes Byte at `ir[13:12]==01` and the ALU family at
      `ir[7:6]==00`), and AND and OR take no address register at any size.

      That produced the first bench in this series whose job is to prove
      instructions are ILLEGAL, `tb_ap040_pipe_ansrc_illegal.v`, running
      each excluded form for real against the illegal vector and counting
      traps. It is not a milestone control -- it passes on milestone 42's
      RTL too, where every An-source form was illegal -- so it was instead
      validated by breaking the decoder on purpose: widening the mode field
      and dropping both restrictions makes it report 0 traps. A restriction
      nobody tests is a comment.
   2. ~~ADDA/SUBA/CMPA~~ **DONE (milestone 44)**, for the five source modes
      that need no extension word: `Dn`, `An`, `(An)`, `(An)+`, `-(An)`.
      This is the `ir[7:6]==11` case every ALU shape had been carrying a
      `!= 2'b11` term to exclude.

      The new thing was the one predicted: `id_size` had always meant ONE
      width for both the memory access and the ALU, and `ADDA.W` is the
      first instruction where they differ -- Word read, sign-extended, Long
      operation, full 32-bit write to An. `id_sxt_w` carries that, and
      `ap040_ea_fetch.v` grew an `eff_size` that every size-driven decision
      on the MEMORY side now reads: both the lane select and the
      auto-increment step, since `(A0)+` under `ADDA.W` must advance by two.

      Its bench uses two Word cases on purpose, because they fail
      differently: A1 borrows out of its low word, so it catches a 16-bit
      writeback, and A4 subtracts a negative, so it catches zero extension.
      Verified by mutating the extension to zero-fill, which produces
      exactly the two values the header names.

      **Milestone 45** added the `#imm` source, which is the form that
      mattered: `ADDA.L #n,A7` opens and closes every compiled frame. It
      needed no new gather kind and no sign-extension flag -- `held_is_imm`
      already assembles immediates and routes them through
      `id_src_a_is_imm`, and `gather_disp` already sign-extends its
      single-word form, so the Word immediate arrives 32 bits wide and
      correct without `id_sxt_w` being involved.

      One rule had to stop being inferred. `id_writes_ccr` derived "sets no
      condition codes" from `held_imm_areg` -- from the destination being an
      address register -- which held for every immediate form until CMPA,
      whose destination IS an address register and which DOES set condition
      codes. `held_imm_ccr` now carries it directly and reproduces the old
      behaviour exactly for the older forms.

      **Milestone 46** finished the family with `(d16,An)`. It rides
      milestone 40's gather kind rather than adding a tenth -- same one
      extension word, same `held_reg` base, same `gather_disp`. What
      differs is what the instruction DOES with the loaded value, so the
      kind grew three properties that used to be constants of it:
      `held_alu_areg` (destination is An, ALU width Long), `held_alu_ccr`
      (CMPA sets flags, ADDA/SUBA do not) and `held_alu_sxt`.

      That is the third time a "property of the kind" has had to become a
      carried bit rather than an inference -- `held_alu_op` in milestone 40,
      `held_imm_ccr` in 45 -- and the pattern is identical each time: a new
      instruction shares a gather's SHAPE but not its semantics. Worth
      expecting rather than rediscovering for the tenth kind.

      Each property was verified by pinning it to a constant and confirming
      which checks fail. Pinning `sxt` corrected a wrong prediction in the
      bench header: the flag drives `eff_size` as well as the extension, so
      losing it reads a whole LONGWORD ($00200EEF) rather than reading a
      word and zero-filling it ($FFFF2020, which is what milestone 44's
      mutation of the extension function itself produces). Two bugs, two
      values.
   2b. ~~`EOR Dn,Dm`~~ **DONE (milestone 47).** `ap040_decode.v` had
      carried a comment naming this hole since the binary family was
      written: `ir[8]=1` is the `Dn -> <ea>` direction, and for nibble 1011
      that is EOR rather than a second CMP. The ALU has implemented
      `AP040_ALU_EOR` since the fork; only decode was missing.

      Its operand roles are REVERSED from the `ir[8]=0` family -- `ir[11:9]`
      is the source and `ir[2:0]` the destination -- and because EOR's
      result is symmetric, getting that backwards puts the RIGHT VALUE IN
      THE WRONG REGISTER. Verified by swapping the two fields on purpose.
      Mode 001 stays out: that is CMPM.

   3. ~~ALU-to-memory~~ **DONE (milestone 48)** for `(An)`, `(An)+` and
      `-(An)`, all sizes, OR/SUB/EOR/AND/ADD. The first real structural
      addition since milestone 30, and the first instruction here that both
      loads and stores.

      `ap040_execute.v` is now a second producer on L1 port B, because an
      RMW's store data is the ALU result and does not exist a stage
      earlier. EX wins the port unconditionally -- it is the OLDER
      instruction, so making EA-fetch wait is correct and deadlock-free,
      where the reverse could starve an RMW behind a run of loads.

      **EA-fetch answers `port_taken` with a BUBBLE, not a freeze**, and
      getting that wrong deadlocked the pipeline on the first run: freezing
      EA-fetch holds `eaf_valid`, `eaf_valid` is what holds `ex_st_req`, and
      `ex_st_req` is what froze EA-fetch. The bubble is the same mechanism
      `mem_issue` has always used. It sits AFTER the `mem_complete` branch
      on purpose -- a completing read consumes `l1_q_b`, registered from the
      address driven before EX took the port, so that data is still ours.

      Operands cross over in EA-fetch: the ALU computes `b op a`, so the
      LOADED value must be `b`, the opposite of every other memory-source
      form, or `SUB.L D1,(A1)` computes D1 minus memory. Verified by
      removing the crossover, which gives `FFFFFFE3` where `0000001D` is
      right.

      **One guard here is deliberately untested.** EX also gained a local
      stall for a store the L1 cannot accept, and its output-register gate
      moved from `stall_in` to `ex_stall` to match. That path is
      unreachable against the current `ap040_pipe_l1.v`: `wr_busy` is high
      for exactly one cycle after a write, and EA-fetch bubbles whenever EX
      holds the port, so nothing lets EX observe a busy port. Confirmed by
      instrumenting `rmw_wait` and running all 52 benches -- it never fires,
      and reverting the gate still passes everything. It stays as defence
      for the real cache in item 4, and the bench header says so rather
      than implying it is checked.

      **Milestone 55** added `(d16,An)` as an RMW destination -- the
      struct-field update, `s->field += x`, and the most-used RMW mode. The
      DATAPATH needed nothing: milestone 48 already loads from `ea_target`
      and stores back to `eaf_ea_target`, and for mode 101 `ea_target` is
      already `operand_a` plus the displacement. It is decode alone, riding
      milestone 40's gather kind with three more carried properties.

      That is the FIFTH instance of the carried-property pattern, and the
      first where the carried thing is the OP MAP rather than a flag: in the
      `ir[8]=1` direction nibble 1011 is EOR, not CMP, so the kind can no
      longer take `held_alu_op` from `alu_nib_op` unconditionally. Verified
      by reverting the map, which leaves the EOR's target word untouched and
      nothing else wrong anywhere -- a silent no-op, since CMP writes
      nothing.

      Still not reached: the absolute modes as RMW destinations.

      **Milestone 56 (indexed addressing, `(d8,An,Xn)`)** closed the largest
      addressing-mode gap: `MOVE.L (0,A0,D1.L*4),D2` is `a[i]`. It gathers
      ONE extension word like `(d16,An)`, so it rides the same gather kinds
      rather than adding more -- what differs is what the word MEANS. For
      this mode `id_imm` carries the brief format VERBATIM and
      `ap040_ea_fetch.v` unpacks it, which is why no new per-stage fields
      were needed: the extension word already IS the packed form.

      It needed a THIRD register READ port. An is on port A and the
      destination operand is on port B for everything except a plain load,
      so two genuinely do not reach. It forwards like the other two.

      **Milestone 57 (PC-relative)** added `(d16,PC)` and `(d8,PC,Xn)` for
      MOVE, the binary ALU family and LEA. Amiga code is position-
      independent throughout and `LEA msg(pc),A0` is its signature idiom.

      Both modes gather the SAME extension word the register-based modes do
      -- a displacement for 010, a brief format for 011 -- so they ride the
      same gather kinds with one more carried property. All that differs is
      the BASE: the PC of the EXTENSION WORD, which is the opcode's PC plus
      two. `eac_pc` was already threaded for the exception frames, so
      nothing new reaches EA-fetch, and `(d8,PC,Xn)` came along for free on
      top of milestone 56's index arithmetic.

      Keeping the resolution in EA-fetch rather than folding it into
      `id_imm` at decode -- where the PC is also known -- is what makes that
      sharing possible. The decode-time shortcut would have forced
      `(d8,PC,Xn)` to grow its own path.

      The base being PC+2 rather than the opcode's PC is the easy thing to
      get wrong, so every check in its bench sits two bytes from the wrong
      answer. Verified by making the base `eac_pc`: all five fail, with A1
      at `$047E`, A2 at `$03FE` and both loads straddling their longwords.

      **Milestone 58 (unary ops on memory)** reached CLR, NOT, NEG, NEGX
      and TST against `(An)`, `(An)+` and `-(An)`. CLR and TST especially
      are everywhere -- zeroing a field, polling a flag -- and until then
      none of the five could name anything but a data register.

      They split across two paths that already existed, and the split
      follows from the ALU rather than being chosen: `ap040_pipe_alu.v`
      computes NOT, NEG, NEGX and CLR from operand B and TST from operand
      A. Milestone 48's RMW crossover puts the loaded value in B; the
      ordinary memory-source path puts it in A. So TST is a plain load with
      no store and the other four are RMWs, with no new datapath either way.

      CLR still performs a read its result does not need -- 68000/68010
      behaviour; the 68040 suppresses it. Noted rather than silently
      divergent.

      **Milestone 59 (absolute EAs)** reached the ALU family and the unary
      ops on `$xxx.W` and `$xxx.L`, the last modes whose extension words are
      the ADDRESS itself. They ride `held_is_abs`, which already gathers one
      word or two and already sets `id_is_abs`; what it did not carry was an
      OPERATION, since every absolute form until then was a MOVE.

      The absolute READ-MODIFY-WRITE composed without new datapath:
      `id_is_abs` makes `ea_target` equal `eac_imm`, and milestone 48's
      store half writes to `eaf_ea_target`, which is that same value.
      Nothing had to learn that an absolute address could also be a
      destination.

      **Milestone 60 (PEA)** is LEA with a different destination: the same
      effective address, sent to the stack instead of to An. It rides
      `held_is_lea` with one carried property, and the push is the
      BSR/JSR/LINK path, whose address is already `operand_b - 4` once
      `eac_dest_reg` names A7. The only new wiring is the DATA -- BSR pushes
      a return address, LINK pushes the old An, PEA pushes the EA itself.

      Its bench uses four different modes on purpose, and a mutation showed
      why: pushing `operand_a` instead of `ea_target` is invisible for
      `PEA (An)`, where the displacement is zero and the two are the same
      value. Three of the four modes catch it; the simplest one cannot.

      **Milestone 61** closed that, and found the same hole in LEA.
      `LEA $xxx.L,An` is a real encoding a compiler may emit; milestone 42
      noted that `MOVEA.L #imm,An` has the same EFFECT and then left the
      opcode undecoded, which is a gap rather than a redundancy. Both ride
      `held_is_abs` with one property: the instruction delivers the ADDRESS
      rather than the contents, so it reads no memory and sets no flags --
      a distinction milestone 59 never needed, because every absolute form
      it reached did read memory.

      Verified by leaving them reading, as every other absolute form does:
      A1 comes back `DEADBEEF` and A2 `CAFEBABE`, the seeded contents of the
      addresses they should merely have named.

      ### When the right answer and the wrong one look the same

      Mutating TST onto the RMW path did NOT fail the first version of its
      bench, and the reason generalises. A unary memory op names no data
      register, so decode leaves `id_dest_reg` at zero; the mis-routed TST
      therefore computed **D0**, which was also zero, set Z anyway, and
      stored a zero byte over a byte that was already zero. Every check
      passed against RTL that was wrong.

      The fix was a `MOVEQ #-1,D0` at the top of the bench, purely to make
      the wrong path produce something distinctive -- the mis-routing now
      sets N instead of Z and writes `FF` into the tested byte.

      The general rule, which applies to TST, CLR and anything else whose
      correct result equals what it read: **when the right answer and the
      wrong answer coincide, the bench has to poison the wrong path**,
      because the right one produces no evidence at all.

      **It happened again in milestone 59, which is why it is a rule and not
      an anecdote.** Mutating the absolute forms to lose their operation --
      defaulting to MOVE -- was caught only by the binary ADD. CLR and NOT
      passed, because a unary form names no source register, so decode
      leaves the destination field pointing at whatever `ir[11:9]` holds
      (D1 for that CLR, D3 for that NOT). Both were zero, so the defaulted
      MOVE stored ZERO -- exactly what CLR should store, and exactly what
      `NOT.W` of `FFFF` should leave. Two MOVEQs fixed it.

      The pattern to watch for: **an instruction whose unused register
      fields read as zero will hide a defaulted operation whenever zero is
      also the right answer.** Set them to something else before testing.

      ### A testability limit worth knowing

      **A Word index's sign extension cannot be observed through a LOAD in
      this core.** `ap040_pipe_l1.v` is 4096 words, so addresses wrap mod
      8192, and `65536 * scale` is always a multiple of 8192 -- a
      zero-extended index therefore lands on exactly the same word as a
      sign-extended one, for every scale. The first version of
      `tb_ap040_pipe_index.v` checked it through a load and passed happily
      against RTL with the sign extension removed.

      The fix was to add `LEA (d8,An,Xn),Am` and check An: LEA puts the full
      32-bit address in a register, where `$00000484` and `$00040484` are
      plainly different. Any future address arithmetic whose result exceeds
      the L1's span needs the same treatment -- a load will not see it.
   4. ~~LINK/UNLK~~ **DONE (milestone 49)**, then **MOVEM**, then
      **MULU/DIVU**.

      **MULU.W/MULS.W (milestone 51)** occupy nibble 1100 with
      `ir[7:6]==11` -- the slot every ALU shape had been excluding, since
      the 68k gives AND's opmode 011/111 to multiply. A 16x16 multiply is
      one DSP block and fits a 40 MHz cycle, so it is decode plus two ALU
      cases with no sequencer.

      The result is 32 bits into the whole of Dn while the source is a word,
      so `id_size` is Long and `id_sxt_w` forces the memory read and the
      autoincrement step to Word, as for `ADDA.W`. The sign extension that
      implies is irrelevant rather than wrong: the ALU reads only bits
      [15:0] of each operand, so MULU is not made signed by arriving
      sign-extended. Its flags are read from bit 31 explicitly instead of
      through `res_msb`, which follows `size`.

      The bench settles signed against unsigned with one pair: identical
      source bits and the same multiplier, which must give `0005FFFA` one
      way and `FFFFFFFA` the other. Verified by making MULS unsigned, which
      collapses the two to the same value.

      **DIVU.W/DIVS.W (milestone 52)** are an iterative restoring divider in
      `ap040_execute.v` -- thirty-two steps, computing a 32-bit quotient and
      then checking it fits in 16, which is the same test as the 68k's
      "upper word of the dividend >= divisor" precondition without needing
      separate reasoning. It lives in EX rather than the ALU because it has
      state, and keeping `ap040_pipe_alu.v` purely combinational is worth
      more than uniformity.

      It reuses milestone 48's local-stall machinery wholesale, including
      the output-register gate that milestone moved from `stall_in` to
      `ex_stall`. **That gate was recorded there as unreachable and
      deliberately untested; every divide exercises it now.** Without it the
      instruction would retire while the divider was still running.

      Overflow makes DIVU/DIVS the first instructions here that complete,
      set a flag and write NOTHING, via DBcc's `writes_reg_resolved` hook.
      Zero divide is a real vector-5 exception, detected in
      `ap040_ea_fetch.v` where the operand already is, so the divider never
      sees a zero divisor.

      Two things the exception needed beyond a trigger wire, both found by
      the bench rather than by reading:

      - `exc_reaching_ex` in `ap040_execute.v` is a list of `eaf_is_*`
        flags, so a new exception with no flag pushes its frame and reads
        its vector correctly and then **does not redirect at all**. The
        file's own comment had predicted that adding to the aggregate was
        all address error needed; it is all divide by zero needs too, but
        it is not nothing.
      - The dynamic-exception destination override listed only priv and
        address error, so the new supervisor SP committed to the divide's
        own destination register.

      **Milestone 53** added the `#imm` source to all four, which is the
      form compiled code uses most (`MULU #10,D0`, `DIVU #10,D0`). No new
      gather kind -- `held_is_imm` already assembles a Word immediate and
      routes it through `id_src_a_is_imm`.

      It hit the carried-property pattern for the fourth time, and this one
      is qualitatively different from the first three: every earlier
      instance carried a variant of something the gather already had (an ALU
      op, a CCR rule, a destination bank). A divide is not an ALU op at all
      -- it is a sequencer flag `ap040_execute.v` keys off -- so
      `held_imm_div`/`held_imm_divs` carry a property from a different
      mechanism entirely. Verified by not carrying it: `DIVU #7,D0` then
      moves the immediate into D0 and leaves `00000007`.

      **And one latent bug of its own.** The exception's VECTOR READ went
      through `mem_lane`, which selects a lane from `eff_size`. A vector is
      always a longword, but divide by zero is the first instruction that
      both sets `eac_sxt_w` and can fault -- so the vector came back
      sign-extended from a half-word, i.e. `$00000000`, and the redirect
      went to zero. It now reads `l1_q_b` directly. Any future faulting
      instruction with a Byte or Word size would have hit this.

      **MOVEM.L (milestone 50)** covers the two autoincrement forms, which
      are the prologue/epilogue idiom. One instruction, up to sixteen memory
      accesses, so it is a sequencer in `ap040_ea_fetch.v` alongside the
      exception-frame and RTE ones -- and a THIRD register write port, since
      one instruction writing sixteen registers cannot use a writeback path
      that carries one result per instruction. That port needs no
      arbitration: a MOVEM holds EA-fetch and the pipeline ahead of it has
      drained, so ports 1 and 2 are idle.

      The mask is numbered differently in the two directions -- bit 0 is A7
      for the predecrementing store and D0 for the load -- and walking from
      bit 0 upward is correct for BOTH only because the register index is
      read as `15 - bit` in one case and `bit` in the other. That symmetry
      is why one sequencer covers both, and it was verified by mutating the
      store to use the load's numbering.

      Stores take one cycle per register when the write buffer is free and
      retry while it is not, as an exception frame beat does. Loads take two
      -- drive the address, capture `l1_q_b` the cycle after. Pipelining the
      load beats is left for when MOVEM is on a path that cares.

      `raddr_a` is redirected to the register each store beat reads, and the
      two forwarding comparators now compare against `raddr_a` rather than
      `eac_src_reg` -- a no-op when they are equal, and what keeps a
      register written by the instruction just ahead of the MOVEM correctly
      forwarded.

      LINK and UNLK fit because they write TWO registers and milestone 30's
      second write port is free for them -- neither autoincrements. LINK's
      memory write reuses `eac_is_push`, the BSR/JSR path, whose address is
      already `operand_b - 4`, so pointing `eac_dest_reg` at A7 makes it
      come out right with no new arithmetic; only the pushed DATA is new.
      UNLK needed no new memory path at all. LINK is the TENTH gather kind,
      carrying its own properties as predicted.

      The three `an_write` expressions were hoisted out of the four branches
      that repeated them into `an_wr_any`/`an_wr_reg`/`an_wr_data`, since
      the selection now has three cases rather than one.

   **Milestone 62 (ORI/ANDI/EORI to CCR and SR)** is what interrupt masking
   is made of -- `ORI #$0700,SR` to mask, `ANDI #$F8FF,SR` to restore. They
   share `immop_shape`'s nibble but not its mode field, so the two families
   are disjoint by construction, and only ORI, ANDI and EORI are legal.

   The result does not go through the ALU: the operand is the status
   register, so `ap040_execute.v` computes it from `eaf_sr_snapshot` and
   commits on the same path MOVE-to-SR and RTE use. The CCR form is computed
   on EIGHT bits rather than masked afterwards -- verified by doing it the
   obvious way, which ANDs `$0007` into the whole SR and drops the core out
   of supervisor mode.

   ### Milestone 63: a defect in milestone 52, and what hid it

   **Memory-source divide by zero did not trap.** `eac_is_divzero` tested
   `operand_a`, which is the divisor only for a REGISTER source; for a
   memory source `operand_a` is the ADDRESS, so `DIVU.W (A1),D0` with a zero
   divisor in memory compared the address against zero, never faulted, and
   ran the divider on a zero divisor -- writing garbage to D0 and executing
   the following instruction normally.

   It was found while designing CHK, which has the same operand shape, not
   by a test. `tb_ap040_pipe_divzero.v` covers only the register source and
   **passes on the defective RTL**, which is exactly why the defect survived
   eleven milestones.

   Fixing it took three things, and the second two are the interesting ones:

   1. The divisor is `mem_lane` for a memory source, valid only when
      `mem_pending` says `l1_q_b` holds it.
   2. `mem_complete` has to YIELD to an active exception. Both are true in
      the same cycle -- the value that causes the fault is the one the load
      just returned -- and the branch chain reaches `mem_complete` first,
      retiring the instruction before the frame push can start. Without
      this, no memory-source exception can exist at all.
   3. The fault condition must be LATCHED. `mem_lane` is `l1_q_b`, which
      lives one cycle: the frame push this exception starts drives a new
      address on port B, so the condition evaporates, the exception
      unasserts itself mid-sequence, and `mem_complete` -- freed again --
      retires the instruction. The first two fixes alone left the bench
      still failing, with the exception visibly starting and then giving up.

   The general lesson: **a fault derived from loaded data is a one-cycle
   condition in a multi-cycle sequence.** Any future memory-sourced
   exception (CHK is next) needs the same latch.

   ### Milestone 64: CHK

   Bounds checking, vector 6, for `Dn`, `(An)`, `(An)+`, `-(An)` and
   `#imm`. Its operand shape is the one that exposed milestone 63's defect,
   so the latch was built in from the start rather than discovered.

   N is defined on the two trapping paths only -- SET when the value is
   negative, CLEARED when it merely exceeds the bound -- and is written into
   the STACKED SR, which is what a handler reads and what RTE restores. The
   non-trapping case leaves N, Z, V and C unchanged; the 68040 calls them
   undefined, so that is one legal reading and the decoder says so.

   Two bench corrections, both instructive:

   - The immediate form read the WRONG REGISTER. `held_imm_dest9` selects
     `ir[11:9]`, and without it the destination falls back to `ir[2:0]`,
     which for mode 111/100 is the constant 4 -- so `CHK #10,D0` checked D4,
     found zero, and never trapped.
   - **The N checks did not discriminate.** Each CHK is preceded by a MOVE
     that loads Dn, and that MOVE sets the live N from the very value CHK is
     about to judge -- so stacking the UNMODIFIED SR gives the right N by
     coincidence, and the bench passed against RTL that never wrote N into
     the frame. A `MOVEQ` before each CHK now sets the live N to the
     OPPOSITE of what CHK must stack.

   That is the fourth time the "poison the wrong path" rule has been needed,
   and the first where the coinciding value came from a neighbouring
   instruction rather than from an unused register field.

   ### Milestone 65: covering the mode, not just the instruction

   Milestone 64's CHK bench used only `#imm`. That is precisely the gap
   that let milestone 52's memory-source divide by zero survive eleven
   milestones -- a bench written for the INSTRUCTION rather than for the
   addressing modes it supports -- so `tb_ap040_pipe_chkmem.v` exercises
   the path the latch was actually built for.

   The result is worth recording as evidence rather than assertion. Two
   mutations were tried:

   - remove the `exc_pend_chk` latch
   - read `operand_a` unconditionally instead of `mem_lane`, which is
     milestone 52's defect exactly

   **Each breaks the memory-source bench and leaves the immediate one
   passing.** The immediate-only bench would have shipped both.

   The bound word is 10 with `$FFFF` in the half-word below it, so a Long
   read would see a low word of -1 and reject every value -- the in-range
   case therefore doubles as a check that the bound is read Word-sized.

   **Standing rule:** when an instruction supports memory sources, the
   bench covers a memory source. A register-only or immediate-only bench
   does not test the operand routing at all.

   ### Milestone 70: a mispredict on top of every stall

   `tb_ap040_pipe_integration3.v` places a not-taken BNE -- which decode
   predicts taken, so each one is a recovery -- immediately after a divide,
   a MOVEM restore, an RMW followed by a load of the same address, and a
   nested BSR/RTS pair. D2 counts checkpoints: 4 if every recovery landed,
   -1 if a branch went the wrong way, fewer if the pipeline hung.

   **The first version was vacuous, and only the control run said so.** It
   passed on milestone-68 RTL, which has the lost-redirect bug. The reason
   is a precise statement of that bug's trigger: the redirect is lost only
   when the branch's PREDICTED-TAKEN TARGET is a memory instruction, because
   that instruction is fetched speculatively and its `mem_issue` is what
   stalls IF in the recovery cycle. `fail:` was a MOVEQ, so nothing ever
   stalled. It now opens with `MOVE.L (A0),D3`, and on the old RTL the bench
   hangs at the first recovery with D2 = 0 -- the failure mode milestone 69
   actually saw, reproduced on purpose.

   The rule that follows: **a bench for a control-flow bug has to reproduce
   the exact timing, not just the instruction sequence**, and the control
   run is the only thing that tells you whether it did.

   ### Bundle 10 (2026-09-24): the sequential core's programs, and the MMU

   WIP 817ba673, 0debfa4d, 48f9c864 (the programs); 969fa7ef (the MMU),
   86f69b53 (review 16), 476cc5d3 (double faults); benches and docs
   closing it in the commit after. Verified on 476cc5d3 (v16): corpus all
   eleven groups, wrong 0 (BasicFPU 362/366, the reference's four); both
   suites green but for the two fetch-budget benches the closing commit
   fixes. Fit (ap040_pipe_core): 17,595 ALMs, 40.41 MHz, +0.251 ns at
   25 ns.

   **`tb_ap040_pipe_program.v`** runs `tests/ap040/asm`'s self-checking
   programs -- `tb_ap040_program.v`'s 64 KB map, its three bus phases and its
   protocol registers around `ap040_pipe_bus16.v` -- and
   `run_pipe_verilator.py` runs it once per program. All fifteen are
   REQUIRED and pass all three phases, on both suites (random ce, slow L1):
   t_integer, t_fastpaths, t_fpu, t_fpu_frames, t_fpu_resume, t_cinv_moves,
   Dhrystone, t_exceptions, t_moves_fc, t_mmu, t_bitfield_mmu,
   t_bitfield_cache, t_atcprobe, t_movem_restart and t_fault_edges (new,
   passing on both cores). t_cache and bench_* are excluded as
   `run_verilator.py` excludes them for a core without caches.

   What the programs found before the MMU, all fixed:
   - No reset exception processing: the ISP and PC now come from $0/$4
     (`RESET_VECTORS`, default 0 for the milestone benches).
   - CINV/CPUSH were F-line exceptions. Now privileged instructions that
     retire with a refetch (`eaf_refetch`) and empty the prefetch stream.
   - A store onto an instruction already fetched behind it ran the old
     instruction (t_integer 192). Stores from EA-fetch (judged a cycle late,
     off the port-B address path) and EX's store beat are snooped against
     EA-calc, decode's gather and the fetch word; a hit refetches.
     `tb_ap040_pipe_smcdual.v` rewrites an instruction 0-3 ahead through all
     four store routes on both cores (480 cases) and found membus handing
     over a fill that landed in the very cycle a write to it was accepted.
   - MOVES went out under the S bit's function code; port B carried only
     that bit. SFC/DFC now reach the bus.
   - TRAPcc judged the forwarded CCR, putting EX's shifter flags on the
     path into the output chain (ae1116d8's worst path).

   **The MMU.** `rtl/ap040/ap040_mmu.v`, unmodified, sits between membus and
   the 16-bit adapter (its request port is the external memory port membus
   drives), with the walker port and a physical bus error.
   - Access error, format $7, the restart model (ap040_core.v's): 30 words,
     WB3S clear, the SSW's CM/MA/ATC/LK/RW/SIZE/TT/TM, FA, the EA field.
     From EA-fetch's own reads and writes; from an instruction fetch
     (membus hands over ILLEGAL marked faulted, decode gives up a gather at
     a faulted extension word, EA-fetch takes it where the ILLEGAL would
     have been; only a fetch issued FOR a request is a demand fault, and a
     prefetch the fetch unit joined is issued again); and from EX's store
     (EX abandons the instruction, it is refetched, and EA-fetch takes the
     fault it is owed when it arrives, the trace it armed withdrawn).
   - With translation able to refuse a write (TC.E, or a data TTR), a
     write is tentative until the MMU forwards it. A refusal is held until
     the CPU withdraws its write in a cycle it runs; a pass is held as a
     receipt until the CPU next presents that write (review 16: a pass
     during ce low was lost and the store went out again -- and a schedule
     pausing at each pass re-issued one store for ever). A refused write
     is withdrawn at once, and an instruction with a latched access error
     does nothing more on the port (a MOVES store re-posted inside its own
     frame held the entry for ever).
   - PTEST/PFLUSH through the MMU's sidebands, MMUSR. MOVEC to TC, URP,
     SRP or a TTR waits for quiet and refetches; fetches are held off while
     it or a PTEST/PFLUSH changes what the MMU sees (a walk had started
     with the bus still running the access it had just passed). Every SR
     write refetches: the words behind it were fetched under its privilege.
   - CM: a MOVEM transfer fault stacks CM and the MOVEM's first address;
     a format $7 RTE reads the popped frame's SSW and EA and the MOVEM
     resumes from that address, ahead of a pending interrupt; a fault
     before it can start keeps the outer CM/EA.
   - A transfer crossing a page goes out a byte at a time, each through
     its own page, MA on a fault past the boundary; a crossing write is
     first probed on both pages (the PTEST sideband as an access check),
     so a refused one has written nothing.
   - An FPU transfer's access fault ends the FPU sequencer (review 16: its
     address outranked the frame's, and the frame went partly to the
     operand).
   - A fault during exception processing -- a refused frame write, a
     faulted vector read -- and an odd vector 2/3 halt until reset. The
     odd-vector halt had been a STOP, which since bundle 9 an interrupt
     woke.

   New benches: `tb_ap040_pipe_wrreceipt_bus16.v` (every store route on
   both tops, four ce schedules, writes counted), `_fpufault_bus16.v` (FPU
   and integer operand faults and refusals, every longword, three ce
   schedules including a pause at each refusal, every write checked
   against the frame), `_dblfault_bus16.v` (the three double faults, then
   IPL 7). Mutations: 26 against the MMU work, run on the programs (25
   caught -- the refusal-clearing one by `_fpufault_bus16.v`'s pause at each
   refusal, which exists for it; the redirect gate on a faulted gather word
   is not observable, below); 7 against review 16 and the double faults on
   the benches (6 caught; gating the frame's own refused beat is
   equivalent -- the entry leaves for its vector read the next cycle).
   Two benches' fetch budgets (`PROG_WORDS`, which counts every issued word)
   were raised: `immsr` and `exctrace` ran out once SR writes refetched.

   Known gaps: a physical bus error on a write already accepted (posted, or
   passed by the MMU) is dropped -- a 68040 would report it through the
   write-back fields; a fault on the CM check's own stack read is ignored;
   the redirect gate on a faulted gather word is not observable (without it
   the stray fetch it stops is flushed with the fault). The fit top is
   still `ap040_pipe_core` without the MMU.

   Dhrystone, zero wait, phase 0: 1,442,504 cycles before the MMU, CPI
   about 11 against the sequential core's 7.7 with its caches. Performance
   is next: [the restructuring plan](doc_AP040_PIPELINE_RESTRUCTURING_PLAN.md).

   ### Bundle 9 (2026-09-24): full-format EA, interrupts, and 40 MHz

   WIP fd399c09, 710c0692, 63f63af4, 82ad8e7e, ae1116d8; benches 2cedfe9d,
   da63f47b, 30325472.

   **The full-format extension** (68020 BS/IS/bd/od, pre- and post-indexed
   memory indirection, the reserved encodings): FFEXT_SRC 1055/1055 and
   FFEXT_DST 794/794, from 0; `tb_ap040_pipe_fxdual.v` puts the format on
   every other memory-EA instruction on both cores, 15/15 mutations caught.

   **Interrupts** (`ap040_pipe_irq.v`, the reference's input chain): the
   boundary instruction is held and becomes a format-$0 entry at 24 + level;
   STOP wakes; an interrupt with M set pushes the format-$1 throwaway and
   clears M, and RTE continues through a throwaway on the stack its SR
   names. IRQ 115/115 and ODD_IRQ 8/8. At a boundary owing both, the
   interrupt goes first and the trace is carried into its handler, as the
   reference's texc_pend does. `tb_ap040_pipe_irqdual.v` takes interrupts
   INSIDE programs on both cores -- after MOVE/ANDI to SR, RTE, STOP
   wake-ups, level 7, from user mode and with M set, RTEs through built
   throwaways, traced events, nested interrupts, trapping instructions
   held -- and found what no IRQ round can see, since a round ends when
   the entry retires: the level just taken re-qualified against the old
   mask and was taken twice; the arm was sampled before an SR write's mask
   was forwarded; STOP flushed without redirecting and the word after it
   was lost on a bus; a verdict latched in the arm's re-sample cycle kept
   writing its frame after the interrupt took the instruction over; and an
   interrupt entry stacked the held CHK/zero divide's flags and format.

   **ODD_EXC 33/33**: odd-vector rounds run on to the nested address error,
   and the generator artifacts are skipped by `tb_dat_replay.v`'s own rule
   (36,752 rounds, counted).

   **Timing: 25 ns met.** Every fit had the same worst path, EX's ALU,
   multiplier or divider result forwarded into an address base or index.
   Address arithmetic and the in-stage trap verdicts now read operand views
   without that forward and hold a cycle behind it (the 68040's change/use
   stall); EX reads `sr_base` rather than its own SR forward. 710c0692
   37.32 MHz (-1.794); 63f63af4 38.63 (-0.885); **82ad8e7e 42.2 MHz,
   +1.304, 16,816 ALMs**; ae1116d8 39.94 (-0.038, TRAPcc on the forwarded
   CCR, moved off in 817ba673).

   Corpus at ae1116d8: Basic 658/658, Default 532/532, AE 38/38, ODD_STK
   6/6, ODD_EXC 33/33, ODD_IRQ 8/8, IRQ 115/115, FFEXT_SRC 1055/1055,
   FFEXT_DST 794/794, BasicFPU 362/366 (the reference's four), PackedFPU
   196/196; wrong 0 outside BasicFPU's four. Suites: 159/161 -- irqdual
   could not build (its `berr` tie was committed ahead of the port, fixed
   in 817ba673) and rmwsup, whose memory model pulsed port A's valid for one
   cycle against the L1's hold-until-next-request contract (48f9c864).
   Mutations: interrupts 16/16, trace carry 4/4 (one only after the nesting
   extension), address views and verdicts all caught, port C's hold term
   removed as unreachable (gathered instructions are never straight behind
   their producer; a Verilator check asserts it).

   Rules: a differential bench has to run the feature INSIDE programs, not
   one instruction at a time, or everything after the entry goes unjudged;
   a hold cycle must not latch a verdict; and a bench's memory model is
   held to the same port contract the RTL is.

   ### Bundle 8 (2026-09-24): fetch bandwidth, timing, review 15, the FPU

   WIP commits a0bdc57e, 9d94eada, 9c73aa5d, e818618b; benches 4e8b71c1 and
   26e264b6.

   **Performance** (a user report measured 4 CPI on a zero-wait bus against
   the sequential core's 2). `ap040_pipe_membus.v` fetches aligned longwords
   into a four-entry stream buffer; decode predicts forward branches not
   taken and backward ones taken; the divider does four restoring steps a
   cycle. Zero-wait bus, CPI before -> after: NOP 4 -> 1.5, dependent ADD 4
   -> 1.5, LEA 8 -> 3, BCS not taken 7 -> 1.5, MULU.L 8 -> 3, DIVU.L 40 ->
   11, DIVU.W 36 -> 10 (review 15 measured the same).

   **Review 15**, both confirmed with the full CPU: the stream fetched $0
   out of reset before the first request (a $0 that never acknowledges hung
   the reset fetch behind it), and its write-snoop compares were ordered, so
   a window across $FFFFFFFF kept stale words. `pf_live` gates the stream;
   the snoop uses modular distances of both longwords a write touches. The
   `busredirect` bench still expected word fetches -- a0bdc57e failed it in
   both suites -- and was rewritten for the stream. The prefetch bench's
   patch case had gone VACUOUS when the divider got faster (the store went
   out before its target was fetched): it now uses one-word divides, whose
   issue rate lets the window gain on the fetch unit, and asserts the target
   was fetched before its store.

   **Timing**, standalone at 25 ns: a0bdc57e -2.540 ns. All 40 worst paths
   ran EX's forward into the two-load forms' second address on the L1 path
   (bundle 7c); `xm_daddr`/`xm_dan` are latched at the first load's issue
   (9d94eada, -0.429). Then all 40 ran EX's forward into CHK's trap decision
   and `exc_active`; CHK now waits one bubble when EX may write its register
   (9c73aa5d, 39.89 MHz, -0.068, 9,261 ALMs).

   **The FPU** (e818618b). `ap040_pipe_fpu.v` runs `rtl/ap040/ap040_fpu.v`,
   the engine the sequential core uses, unchanged, inside a transliteration
   of that core's `S_FPU_*`/`S_FBCC`/`S_FSCC*`/`S_FDBCC`/`S_FSAVE*`/`S_FREST*`
   states under the same names. An F-line instruction starts once EX is
   empty and holds EA-fetch; it retires through the default branch or as an
   exception with its own vector, format ($0/$2/$3 -- format $3 is new to
   the frame builder), PC and address. Decode gathers the command word and
   then as many words as it asks for. Register-destination arithmetic is
   released to the background and the next F-line instruction delivers its
   exception, as the reference does. Corpus: BasicFPU 362/366 over
   2,080,064 rounds -- the four slices the sequential core fails, with
   identical mismatch counts (176/640/128/192) -- and PackedFPU 196/196,
   544,364 rounds, 0 mismatches. This archive's 4_FINT has no slices, so
   `tb_ap040_pipe_fpudual.v` runs generated FPU programs on both cores and
   compares registers, FP state, memory and a log of every exception frame:
   24 programs equal, 454 frames logged, 18/18 mutations of the FPU glue
   caught (two only after the generator learned stores through (An)+ and
   FMOVEM lists against their EA's direction). Fit: 15,944 ALMs, 38.94 MHz,
   -0.681 at 25 ns; the worst path is integer -- EX's forward as a LEA/JMP
   index register into `ea_target` -- not the FPU.

   **The pipe driver** judges FPU rounds (state injected and checked as the
   sequential driver does), holds the bus while a released FPU operation
   finishes so the integer side cannot move, and judges a round that runs on
   to the terminal entry on the SR it stacked (every user-mode FPU round
   read the handler's $2000: cputest follows each FPU instruction with FNOP).

   State: Basic 658/658 0 wrong; Default 532/532 and AE 38/38, 0 wrong (run
   for the first time); ODD_STK 6/6. Not yet: FFEXT_SRC/FFEXT_DST (the
   full-format extension -- the pipeline read it as the brief format, silently
   wrong, found by the FPU differential), IRQ/ODD_IRQ (no interrupt input),
   ODD_EXC (the driver skips odd vectors), and the MMU. 158 benches, both
   builds.

   Rules: a bench whose case depends on timing must assert the case
   happened, or it goes vacuous when the timing moves; a differential
   generator needs its own mutations before it is believed; and a forward
   into any DECISION (a trap, an address) is the same timing problem as a
   forward onto the L1 address.

   ### Bundle 7 (2026-09-24): the Basic integer ISA complete, every round judged

   Bundles replaced one-milestone-per-commit on 2026-09-23 at the user's
   direction: several families per commit, one full corpus run per bundle,
   `wrong` held at zero. Bundle 7 (WIP commits eaa3e14a, 79b5c69f, 905425f3,
   9b70f21b) finished the integer instruction set the Basic group tests:
   EXG, MOVEP, MOVEM with an index, MULL/DIVL (three-register forms
   included), the bitfields, CHK2/CMP2, CAS/CAS2, MOVE16, TRAPV, MOVE USP,
   RESET, RTR, the two-memory-operand arithmetic and PACK/UNPK forms, and
   `#imm.L` to `(xxx).L`. Exception vectors now come from VBR.

   The corpus driver had skipped every trace round -- 4,509,254 of Basic,
   the blind spot where reviews 13 (MOVES not a T0 change of flow) and 14
   (every exception entry armed a trace, the 68000/68020 rule) found their
   defects. It now judges them: the stacked trace SR and PC, a round whose
   result is the trace, a traced program run through each trace to its
   terminal ILLEGAL, and any trace armed where the oracle has none.

   State: Basic 658/658 over all 20,875,528 rounds -- the sequential core's
   count -- with 0 wrong and 0 undecoded; only the 454,455 vector-only
   rounds are unjudged, as in the sequential driver. 156 benches in both
   builds.

   Rules this bundle paid for: a sequencer that can raise an exception must
   release the L1 port muxes once finished; an EA used after port C changes
   meaning must be latched at issue; a hazard only adjacency can trigger is
   untestable when decode's gather freezes with every stall, so it is left
   out rather than kept as dead logic; and a driver snapshot taken at an
   exception's verdict can predate an older instruction still in EX -- read
   what the frame stacks.

   ### Milestone 107: the sequential core's EA classification, and Scc to memory

   The user's observation, and it was the right one: `ap040_core.v` is in this
   same worktree as the corpus oracle, it passes all 658 Basic slices, and it
   already knows everything this decoder keeps re-deriving per family. Its
   `ea_start`/`S_EA_DISP` (ap040_core.v:1764, :2648) is one `case` over the
   mode field that every instruction enters with a return state, and
   `dst_not_alt`/`src_not_data` (ap040_core.v:940) are the two 68k mode
   CLASSES that say which modes an instruction may use.

   The pipelined decoder had none of that. 29 `*_shape` wires testing
   `if_opcode[5:3]` in 92 places, each family hand-enumerating its own subset,
   and a SECOND enumeration listing every instruction that gathers extension
   words at all. What transfers is the classification, not the sequencing --
   a six-stage pipeline cannot call an EA subroutine and return -- so
   `ea_is_*`, `ea_not_alt`, `ea_not_data` and `ea_ext_words` are now
   combinational wires, lifted and cited rather than re-derived.

   **Scc is the first family written against them**, and it was the largest
   single gap in the core: 2,561,872 corpus rounds, 33.4% of everything the
   decoder could not read. It existed only as `Scc Dn`. It is now "the
   0101 cccc 11 space, not Dn, and alterable" -- one line, where `ea_not_alt`
   excludes DBcc's mode 001 and TRAPcc's three mode-7 encodings by
   construction. The family says what it takes instead of listing what it
   must avoid.

   The memory form rides the read-modify-write store path, because its byte
   is not known until EX where the condition is evaluated. Like CLR it
   performs a read its result does not need; the 68040 does not, and that is
   noted rather than left silent.

   **Four things the corpus found on the way, none of them about Scc:**

   1. `id_imm` defaults to the sign-extended opcode low byte. For a family
      with no displacement that default becomes one, and the store landed at
      `An + sign_extend(opcode[7:0])`.
   2. The gather block's `id_is_branch` is a NEGATIVE list: anything gathered
      and not named there IS a branch, and its `id_imm` is read as a branch
      displacement. An Scc left off it had its destination checked for an odd
      branch target and raised an address error on every gathered form
      landing on an odd byte.
   3. `redirect_from_gather` is the same shape, and redirected the fetch to
      the Scc's own displacement.
   4. The extension-word count lives in BOTH `held_is_long` and
      `ext_pending`. Adding Scc to one and not the other is milestone 89
      exactly, and it happened again: `(xxx).L` gathered one word, so the
      second half of the address executed as the next instruction. `ext_pending`
      now asks `ea_ext_words` instead of carrying another name.

   Three of those four are the shape-by-shape structure charging rent, which
   is the argument for the classifier rather than an aside about it.

   | run | result |
   |---|---|
   | control: `tb_ap040_pipe_sccmem` | FAILS 14 checks on the prior RTL |
   | mutation: `ea_not_alt` dropped | caught -- the reserved mode-7 encoding stopped being illegal |
   | mutation: `ea_ext_words` term dropped | caught -- `(xxx).L` short by a word, D3 never runs |
   | mutation: byte size -> word | caught by the neighbour checks, all five |
   | mutation: condition ignored, always true | caught by SF and SNE, all three |
   | corpus: Basic/Scc.B | 83/83 slices, 0 undecoded, 0 wrong (was 0/83) |
   | corpus: all 658 Basic slices | **260/658 slices** (was 177); undecoded 7,674,801 -> **5,112,929**; wrong unchanged at 5,164 |
   | bench suite, four build modes | 123/123 each |
   | fit at 25 ns | +0.724 ns, 41.19 MHz, 6,048.5 ALMs |

   On the fit: the last three milestones measured +0.704, +1.344 and
   +0.724 ns, all inside the ~0.6 ns placement band of each other. There is
   no trend in that, and reading the middle one as a gain or this one as a
   loss would be reading noise. +26 ALMs is small for a whole instruction
   family, which is the number worth quoting.

   The wrong column not moving is the point of quoting it: 2.56M rounds
   changed from "cannot decode" to "decoded and correct" without a single
   round changing from correct to wrong.

   **A mutation worth recording for what it did NOT catch.** Dropping
   `ea_not_alt` entirely left `tb_ap040_pipe_dbcc` and `tb_ap040_pipe_trapcc`
   passing, because their own decode wires win the ternary chains before the
   Scc arm is reached. The class predicate earns its place here only on the
   reserved mode-7 values, which must stay illegal, and nothing tested those
   until the bench grew an `ST` with a reserved destination. A mode class is
   worth having for what it excludes silently, and that is exactly what a
   bench does not see unless it looks.

   And a bench lesson: `PROG_WORDS` is an instruction ISSUE budget, not an
   address bound. At 40 the fetch ran out before the handler could run, and
   the symptom -- a correct frame pushed, a correct vector table, a correct
   handler, and a PC frozen on the instruction after the trap -- reads
   exactly like a broken exception redirect.

   ### Milestone 106: a bench that passed, and the defect it found anyway

   The worklist milestone 105 produced put "executed and wrong" ahead of
   "not decoded" -- a missing instruction faults loudly, a wrong answer
   corrupts silently -- and its largest entry was CHK.W: 5,668 rounds, every
   one an opcode with A7 as the EA register (459f, 45a7, 479f, 47a7), every
   one reading the ORIGINAL A7 back where the oracle expects the
   postincrement. `tb_ap040_pipe_faultpi.v` already proves that shape with
   DIVU.W and passes, so the difference had to be that CHK's fault is
   decided later: `chk_now` needs the loaded BOUND, so it is qualified by
   `mem_pending && l1_rvalid_b` and deferred through `exc_pend_chk`.

   `tb_ap040_pipe_chkpia7.v` was written to reproduce that. **It passed on
   the RTL it was written to fail on**, which is the whole milestone. The
   core was right and the corpus driver was wrong, for the third time in two
   milestones and in the same family as the other two.

   The driver ended an exception round one cycle after `exc_go`. That is the
   cycle the fault's VERDICT registers, not the cycle its consequences land.
   Traced on Basic/CHK.W/0002, the entry needs 35 more: the frame is pushed
   over the 16-bit bus, the vector read, A7 given the new stack pointer, and
   the faulting instruction's own (A7)+ written back. At +1 none of it has
   happened and the register file still holds the round's input; at +35
   `usp=43800402` and `isp=438007f4`, both exactly what the oracle asked
   for. The sequential driver waits two qualified cycles for this same
   hazard and says so in a comment; one cycle is that comment's translation
   to a machine where the entry is itself a multi-cycle instruction.

   The entry retires under the FAULTING INSTRUCTION'S OWN PC, so the
   boundary already used for completed rounds covers both kinds and the
   early exit is simply deleted. The entry-time register snapshot goes with
   it -- registers are read live, since nothing younger has committed -- and
   the vector, stacked SR and new SP are still sampled at entry, those being
   the frame's own latched fields.

   That also broke the census, which keyed "never decoded" off "nothing
   committed": once rounds ran long enough, the illegal-instruction ENTRY
   commits like any other instruction. It now reads the core's own
   `eac_is_illegal` while the tested instruction is in the EA stage, which
   is the fact itself rather than a proxy for it.

   **The core defect this milestone does fix is JMP.** The absolute EA forms
   all ride `held_is_abs`, and four sub-kinds say what becomes of the
   gathered address: `held_abs_lea` delivers it to An, `held_abs_push`
   pushes it, `held_abs_jsr` pushes a return address, `held_abs_jmp` only
   branches. `id_writes_ccr` excludes all four. `id_writes_reg` excluded
   three, and a JMP is not an ALU operation, so `!held_abs_alu` made the
   term true for free. The register it wrote is not arbitrary: the absolute
   path takes its destination from the opcode's bits [11:9], and JMP fixes
   those at 111, so both forms wrote the target address into D7.

   | run | result |
   |---|---|
   | control: `tb_ap040_pipe_chkpia7` | PASSES on the prior RTL -- a regression guard, not a control, and the reason the CHK failure was traced to the driver |
   | control: `tb_ap040_pipe_jmpabs` | FAILS on the prior RTL, `D7 = 00000600` |
   | mutation: `!held_abs_jmp` removed | caught, `D7 = 00000600` |
   | mutation: absolute JMP stops branching | caught, D3/D4 zero and the poison ran |
   | mutation: exclusion moved to `held_abs_push` | **NOT caught at first** -- see below; caught after the bench grew |
   | corpus: Basic/JMP, both slices | 1,280 wrong rounds -> 0, 4,080 judged |
   | corpus: Basic/CHK.W | 5,668 wrong rounds -> 1,652 |
   | corpus: all 658 Basic slices | 177/658 slices, wrong rounds 10,460 -> 5,164 |
   | bench suite, four build modes | 122/122 each |
   | fit at 25 ns | +1.344 ns, Fmax 42.27 MHz, 6,022 ALMs (was +0.704 ns; 0.64 ns is the placement band, so: no regression) |

   **The third mutation is the one worth keeping.** Moving the exclusion
   from `held_abs_jmp` to `held_abs_push` -- an over-reaching version of
   exactly this fix -- left `tb_ap040_pipe_pea`, `_jsr` and `_lea` all
   passing. Nothing in the suite covered the absolute PEA's A7 write, so a
   fix that broke PEA instead of fixing JMP would have gone in green.
   `tb_ap040_pipe_jmpabs.v` now runs PEA (xxx).L and JSR (xxx).L beside the
   two JMPs and does catch it. The rule: **when a fix is an EXCLUSION from a
   shared term, mutate the exclusion onto its siblings, because that is the
   way this kind of fix fails.**

   What is left of the executed-and-wrong list, 5,164 rounds in 9 families:
   CHK.W 1,652 in its `-(A7)` and immediate forms (`43bc`, `41a7` -- the
   `(A7)+` cases are gone), ADDA/SUBA .L/.W 624 each where the EA register
   is also the destination (`dfdf` is ADDA.L (A7)+,A7), NEGX.L/W/B 992 in
   their memory forms, MOVEC2 24. The not-decoded column is untouched at
   7,674,801 rounds and remains the larger job.

   ### Milestone 105: the corpus at scale, and two driver bugs between it and the truth

   The stack-pointer pattern milestone 104 left open was the driver's, not
   the core's. `final_a7()` picked the bank the FINAL SR selects. But the
   corpus' `regs[15]` is not "whichever A7 is live" -- it is the serialized
   A7 image the native runner entered with, and `inject_state` puts it in
   the USP, because a supervisor round runs on a RELOCATED ISP at `i_ssp`
   holding a copy of the same 32 bytes. So a supervisor round modifies the
   ISP while the slot the oracle serializes back is still the USP, and
   selecting on the final SR reports every such round as off by exactly
   `i_ssp - i_regs[15]` -- the `$400` that showed up in every mismatch.
   `tb_dat_replay.v` has read the USP unconditionally all along, which is
   why the sequential core never saw this. In user mode the USP *is* the
   live A7, so this is not a special case; it is the only reading right in
   both modes.

   **Then running more than one slice found the second one.** A store leaves
   this pipeline long before its bytes reach memory: the CPU posts it to the
   membus, which may be midway through an instruction fetch and only starts
   the write cycles afterwards. The round was being compared in between.
   `+buswr` follows one BSR through all three: the CPU posts
   `L1WR addr=438003fc data=43900002`, correct address and correct return
   address, and the `MEMRQ ... write=1` that carries it to the bus does not
   appear until after `$finish`. Waiting for the 16-bit bus to go idle does
   NOT fix it -- the bus is already idle while the write sits queued, which
   is why the first attempt changed nothing. What has to go quiet is
   `l1_wr_busy`, the membus' own "a write is outstanding", the same signal
   the core stalls on. That alone took the executed-but-wrong rounds from
   273,940 to 10,460.

   Both are the milestone 104 lesson again, which is now worth stating as a
   rule: **a testing assumption imported from the sequential driver has to
   be re-derived on a machine that retires out from under its own stores.**
   Completion, stack-pointer identity and store visibility were all three
   true there and false here.

   The driver now also keeps a two-axis opcode census, because counting
   printed mismatch lines counts the report cap rather than the population
   -- sampling the capped lines put "never decoded" at 93% of mismatches
   where the exact count is 61%. Every round that disagrees is now charged
   to one of two columns, exactly, by opcode: **not decoded** (vector 4 with
   nothing committed -- the illegal-instruction handler ran in place of the
   instruction) and **executed and wrong**. They are different milestones
   and want different benches.

   | run | result |
   |---|---|
   | control: ABCD.B/0001, A7 fix | 48 judged both ways, 24 mismatches -> 0, skipped unchanged at 16 |
   | control: BSR.B, drain fix | 11,110 judged both ways, all mismatching -> 0 |
   | mutation: USP writes dropped | BSR.B 0 -> 11,110 mismatches, both slices |
   | mutation: `eac_is_push` forced low | BSR.B 0 -> 22,220 mismatches; the longer drain cannot pass a missing store |
   | smoke set, six slices | 150 judged, 0 mismatches; the three FPU/IRQ/odd-vector slices judge nothing and report FAILED, as they should |
   | **sequential core, 658 Basic slices** | **658/658 slices, 20,875,528 rounds, 0 mismatches** |
   | **pipelined core, same 658 slices** | 175/658 slices, 16,366,274 judged, 4,963,709 skipped |
   | of which not decoded | 7,609,521 rounds, 14,259 opcodes, 128 families |
   | of which executed and wrong | 10,460 rounds, 105 opcodes, 10 families |
   | milestone bench suite | 120/120 under Verilator, on the clean re-run |

   The sequential baseline is the point of that table: same corpus, same
   oracle, same driver structure, zero mismatches. Nothing left to blame but
   this core.

   No RTL changed in this milestone -- `git diff` touches
   `tests/ap040/tb_dat_replay_pipe.v` and nothing else -- so there is no fit
   to quote. The +0.704 ns at 25 ns from the last milestone that moved RTL
   still stands, and the bench suite was re-run only to prove the worktree
   was left as it was found.

   **The worklist, ranked, and it is short at the wrong end.** Not decoded,
   by weight: `Scc.B` to memory (2,561,872 rounds -- `50f9` is ST (xxx).L),
   `STOP` (1,048,576, the single opcode `4e72`), `ILLEGAL` (240,544),
   MOVE.W/L/B in the EA forms the decoder has never had (641,772 together),
   CHK.L/W, MOVEA.W, the MUL/DIV group. The shapes are missing, not the
   operations: `unary_mem_shape` admits modes 010/011/100 only, so
   `4a2e` -- TST.B (d16,A6) -- is an illegal instruction to this core.

   Executed and wrong, the whole list: CHK.W 5,668 (`459f`, `45a7`: the
   `(A7)+` and `-(A7)` forms), JMP 1,280 (`4ef9`/`4ef8`, absolute),
   ADDA/SUBA .L/.W 624 each (`dfdf` is ADDA.L (A7)+,A7 -- the EA register is
   also the destination), NEGX.L/W/B 992 together (memory forms), MOVEC2 24.
   Two clusters and two strays: predecrement/postincrement where the address
   register is A7 or is the destination, and NEGX to memory.

   **A rule this milestone paid for: never mutate the worktree while a suite
   is running.** The first suite run here reported 119/120 with
   `tb_ap040_pipe_bsr` failing, and the cause was not the RTL -- it was that
   the `eac_is_push` mutation above was applied while that bench was being
   compiled, so it built against mutated sources. The same worktree cannot
   serve a mutation and a regression run at once. Either finish the
   mutations first, or give the mutation its own copy. The suite result
   below is the clean re-run.

   The full ranked inventories are evidence, not repo content:
   `/home/adam/ap040-audit4/cputest-pipe17/undecoded_by_opcode.txt` and
   `wrong_by_opcode.txt` beside it, each opcode under its family with a
   round count. Regenerate them from any run's logs with the `UNDECODED`
   and `WRONG` lines the driver now prints.

   Correction to milestone 104's table: the driver as committed there judges
   **150** of the 323 smoke rounds, not 89. The 89 was measured mid-milestone
   and the entry recorded it after the driver had moved on; the control run
   here reproduces 150 on that exact commit.

   ### Milestone 104: the corpus judges rounds, and names its own bug first

   Following an exception was the thing that made the driver worth running,
   so it now does: `exc_go` is this core's `S_EXC0`, and the snapshot is
   taken the cycle after it rises, once the frame's latched fields have
   settled. **89 of 323 smoke rounds judged**, against none at milestone
   103.

   **And the first thing those rounds found was the driver's own
   completion boundary.** 143 mismatches, every one of them the injected
   input state read straight back, with zero commits. The sequential driver
   stops a round when the core asks memory for the instruction the oracle
   names next -- which on a machine that executes one instruction at a time
   means the previous one has finished. On a six-stage pipeline that fetch
   happens five stages ahead of the tested instruction retiring, so every
   round was being read with the instruction still in flight. It is the
   cleanest example this campaign has produced of a testing assumption that
   is true of one microarchitecture and false of another.

   A round completes now when the writeback stage names the tested
   instruction's own address, or when an exception entry is captured. The
   Basic slice went from 143 mismatches to 24, and what is left is one
   repeated pattern rather than a scattering: a single address register,
   expected `$43800400` and read as `$43800800`, exactly `$400` apart in
   every round. That is which of the three stack pointers the readback
   selects, and it is the next thing to chase.

   | run | result |
   |---|---|
   | smoke set, six slices | 89 rounds judged, 24 mismatches, 0 unreached, 234 skipped |
   | what is still skipped | trace, interrupt and FPU oracles |

   ### Milestone 103: the corpus driver runs, and measures its own scope

   `tb_dat_replay_pipe.v` replays the cputest corpus against this core, and
   `run_cputest.py --core pipe` drives it. Everything about the corpus is
   shared verbatim with the sequential driver -- the APR2 parsing, the
   memory images, the patch and toggle streams, the value helpers -- and
   `ap040_pipe_bus16.v` presents the same sixteen-bit port, so the memory
   model came over too. What is new is the round: this core has no reset
   vector, so a round writes the slice's start address into the fetch stage
   and its registers into `dreg`/`areg`/`usp`/`isp`/`msp` while `ce` holds
   the core, then releases and runs to the address the oracle names next.

   **It runs end to end on the first try, and judged nothing.** Across the
   harness's six smoke slices, **0 of 323 rounds** fell inside the scope it
   started with: no expected exception, no trace, no interrupt, no FPU
   state. That is not a tuning problem. A cputest round ends in an
   exception by construction -- the generator closes every test with a
   terminal ILLEGAL -- so "no expected exception" excludes nearly the whole
   corpus. Following an exception to its frame is not a refinement to add
   later; it is the thing that makes the driver worth running, and it is
   the next milestone.

   **And it reported ALL TESTS PASSED on zero rounds.** Six slices, six
   passes, nothing judged. That is the third vacuous green this campaign
   has produced, after milestone 89's branch marker that ran either way and
   milestone 92's mutation that memory could not see -- and the first where
   the HARNESS was the thing at fault rather than a bench. A slice with
   nothing judged now reports FAILED, which is how the number above came to
   be measured at all.

   The rule, which is the same one three times over: **a green result is
   worth exactly as much as the count of things it actually checked, and a
   harness that does not report that count can hide the difference.**

   ### Milestone 102: groundwork for running the real corpus

   Eight rounds of external review have found more than twenty defects,
   and nearly all of them are what the WinUAE cputest corpus tests: frame
   contents, stacked program counters, condition codes after a fault,
   which stack a frame lands on. `tests/ap040/run_cputest.py` already
   replays that corpus against the SEQUENTIAL core, which passes 3,797 of
   3,801 slices. Running it against this one is worth more than any
   further bench written by hand.

   **What is already shared.** The corpus tooling takes a slice and writes
   three files -- a job, a low-memory image and a test-memory image --
   which the replay driver reads through plusargs. None of that cares
   which core is underneath. `ap040_pipe_bus16.v` drives the same
   sixteen-bit port the sequential core's compatibility top does, so the
   driver's memory model needs no changes either. 4,494 slices are
   unpacked under `tests/ap040/build/cputest/unpacked`, and the group
   naming makes them v24.

   **What is not.** `tb_dat_replay.v` reaches into the sequential core in
   about sixty places. It writes each slice's registers into
   `regfile.bank_a`/`bank_b`/`rf_written`, captures results through
   `pend_we`/`pend_wdata`, and watches `core.state` to know an instruction
   retired. This core has `dreg`/`areg`/`usp`/`isp`/`msp` and a writeback
   stage, so the poke and the capture have to be rewritten. It also has no
   floating-point unit, no memory management unit and no caches, which
   puts the `4_F*` groups, the transparent translation registers and the
   address translation cache out of scope, and no interrupt path, which
   rules out `4_IRQ`. The integer groups are the target: Default, BASIC,
   EXTSRC, EXTDST, AE, ODDEXC, ODDSTK.

   **The one thing that could have sunk it.** The sequential core fetches
   a reset vector and can therefore be started anywhere. This one begins
   at `PC_RESET`, a parameter fixed at elaboration, and every slice picks
   its own start address. `tb_ap040_pipe_inject.v` settles it: the core is
   elaborated at `$1000`, `$2000` is written into the fetch stage's own
   program counter with a register value alongside, and the instruction at
   `$2000` runs while the register survives the release. Both halves of
   what a slice needs are therefore available at run time.

   That bench is the milestone. The replay driver itself is the next one,
   and it starts from a known-good injection rather than from a guess.

   ### Milestones 100-101: the RTE's fault, and what a load inherits

   **An RTE to an odd address faults after it returns.** It restores the
   status register and pops its frame first, so the error frame carries
   the RESTORED register and sits below the popped one, and a restored M
   bit chooses the stack it lands on. That makes it an exception owed by a
   COMPLETED instruction, which is what the trace machinery already is, so
   it is built the same way: armed as the RTE departs with its address and
   the odd target latched, held over the instruction behind it, taken once
   EX and WB have drained.

   The first attempt, backed out at milestone 99, had all of that and
   still never fired. `ae_take` fires while the instruction behind the RTE
   is held, and `own_exc` exists precisely to keep THAT instruction's
   faults from being taken -- it masked the debt along with them. The
   trace sits outside that mask for the same reason, and now so does this.
   Starting the second attempt from a bench rather than from the RTL is
   what made the difference: the bench carried the reviewer's exact
   values, so every wrong answer named itself.

   One mutation had no witness at first. Excluding the held instruction's
   own faults passed, because the word at the odd address happened to be
   harmless. It is a TRAP now, so the held instruction has an exception of
   its own to wrongly take.

   **A load inherited the previous instruction's flags.** EA-fetch retires
   down two paths -- the general one, and `mem_complete` for anything that
   waited on a load -- and a field one of them leaves alone keeps the
   previous instruction's value. A load behind a LINK was retired as a
   LINK and wrote no register; a load behind an ORI to SR rewrote the
   status register with whatever it had loaded, which is tracing turned on
   by a data word.

   A review found three stale fields. Diffing the two branches'
   assignments found EIGHT, and that diff is also the only way to know
   there is not a ninth. Four of them have no program that reaches them --
   their consumers in EX are gated on flags this stage also clears -- and
   are fixed alongside rather than left for the next review.

   The rule: **two retire paths for one stage is two lists that have to
   agree, and the way to know they do is to diff them.** That is the third
   time in this campaign a defect has been two lists drifting apart, after
   milestone 89's extension-word count and milestone 97's branch
   displacement.

   | run | result |
   |---|---|
   | control, both benches on the RTL each was written against | both fail with the reported values |
   | full suite, all four run modes | 119/119 |
   | standalone fit | +0.779 ns at 25 ns, 6,030 ALMs |

   ### Milestone 99: two more of the odd-target work's own defects

   **A fault verdict needs a live instruction.** None of the five
   odd-target terms checked `eac_valid` on its own -- `eac_is_jmp_odd` is
   `eac_is_jmp` AND an address bit -- so whatever the stage held after a
   flush could re-arm the pending latch, and the next instruction to take
   ANY exception inherited the dead one's address. A TRAP stacked the
   wrong return address, and returning from it would have run the TRAP
   again. That is the second defect this latch has produced since
   milestone 97 introduced it, and both were about scope: who a held
   verdict belongs to, and whether there is anyone to belong to at all.

   **An indexed JMP stacks pc + 6.** It has resolved its extension word
   against the program counter before it faults, so the counter has moved
   past that word. `rtl/ap040/ap040_core.v` gives ea mode 110 and the
   PC-indexed form `pc_i + 6` and everything else `pc_i + 2`, and says
   those values are what its own corpus group records.

   **The milestone-98 regression was the bench.** `tb_ap040_pipe_oddtarget`
   ended in a tail of NOPs, and by then the stack had walked down through
   seven frames: the fetcher ran off the end of the program straight into
   them, which is an illegal instruction inside the frames and an eighth
   exception. It ends in a terminal loop now. The reviewer who found it
   diagnosed it, which is worth recording -- the failure was committed
   visible and unexplained, and being visible is what got it explained.

   | run | result |
   |---|---|
   | control, the indexed-JMP bench on the previous RTL | stacks $0416, needs $041A |
   | full suite, normal build | 117/117 |
   | full suite, random enable and slow build | 117/117 |
   | standalone fit | +0.704 ns at 25 ns, 5,988 ALMs |

   **Two remain, and they are one shape.** Both are exceptions raised by
   the COMPLETION of something else rather than by a stage refusing to let
   an instruction retire, which is how every dynamic fault here works
   today.

   An RTE with an odd restored PC must restore its SR and finish popping
   BEFORE it faults, so the error frame carries the RESTORED status
   register and sits below the popped frame rather than twelve bytes below
   where it started. A restored M bit selects the stack it lands on.

   An odd EXCEPTION VECTOR is the same thing one level further in. The
   reference's rule, extracted and recorded here so the next attempt does
   not have to find it again: an odd handler address for vector 2 or 3 is
   a double fault and halts; any other odd handler address becomes an
   address error whose frame's PC field is `4 * vector` WITHOUT the vector
   base register -- "offset, not vbr + offset" -- and whose address field
   is the handler address with bit 0 cleared. This core has no halt, so
   the first of those needs a decision before the second can be written.

   **A first attempt at the RTE case, reverted, and what it established.**
   The design is the trace machinery's: `rte_odd_now` (the old detection,
   renamed) ARMS a flag as the RTE departs instead of faulting it, with
   the RTE's own address and the odd target latched beside it; `ae_hold`
   holds the instruction behind the RTE and `own_exc` excludes that
   instruction's faults exactly as `trace_hold` does; `ae_take` fires once
   EX and WB have drained, by which time the restore has committed; and
   the address error is then raised with the latched fields. The arm must
   NOT clear on a flush -- the RTE's own redirect is one -- and clearing
   it on `exc_vec_done` hung the pipeline, so it clears the moment the
   exception is taken. With all of that in place six of
   `tb_ap040_pipe_oddtarget`'s seven exceptions happened and the seventh
   did not, and the discrepancy was not diagnosed before the change was
   backed out: that bench's phases 4 and 7 encode the OLD placement of the
   RTE's frame, twelve bytes below where the RTE started rather than below
   where it finished, and need rewriting for the new one. The next attempt
   needs a dedicated bench first, with the reviewer's values -- ISP `$1000`,
   a format-0 frame restoring SR `$0015` and PC `$0601`, an error frame at
   `$0FFC` stacking `$0015` -- and the frame fields unified into one
   `addrerr_pc_live` select, which the attempt also did and which is worth
   keeping when it is redone.

   ### Milestones 97-98: odd targets, and what the enable found in them

   Milestone 17 made an odd JMP or JSR target take an address error, and
   the check it added reads `ea_target` -- the only target those two
   instructions have, and the only one no other instruction does. So BRA,
   Bcc, BSR, DBcc, RTS and RTE walked into odd addresses and executed
   whatever sat at the even one below.

   Four sources, four places to look. A branch's target is
   `eac_pc + 2 + eac_imm`, the same sum decode turns into its redirect, and
   it is checked whether or not the branch is taken -- the reference core's
   `finish_bcc` raises the error before it decides, and this stage could
   not consult the condition anyway. RTS takes its target from the loaded
   longword, RTE from the frame's own PC field. An odd BSR no longer
   pushes, for the reason an odd JSR has not since milestone 17.

   **The gathered forms needed decode changed too.** A word or long form
   branch gathers its displacement and then redirects, so nothing
   downstream had ever needed it and `id_imm` was handing EA-fetch a zero.
   The target checked was therefore the instruction's own address, which is
   never odd, and `BRA.W` to an odd target sailed through. That is the
   third time a field defaulting to zero has been the whole defect.

   **Three things the random clock enable found, in this milestone's own
   work.** RTS and RTE see their target for exactly one cycle, so the
   verdict has to be latched -- and latching it let it outlive its owner:
   the handler's first instruction inherited the verdict and took the same
   address error again with the same stale target. `exc_go` is what says
   which instruction a verdict belongs to, so the held flag only applies
   while it is set. The other four fault latches had the same defect
   waiting in them and now clear on the same condition `exc_go` does.

   **What is NOT fixed, and it is not a loose end.**

   An RTE with an odd restored PC must restore its SR and finish popping
   BEFORE it faults -- the reference says so in as many words, because the
   error frame then carries the RESTORED status register and sits below the
   popped one. This core faults first, so the frame lands twelve bytes low
   and stacks the pre-RTE SR, and a restored M bit selects the wrong stack
   besides. Faulting after the fact is a different shape from every other
   dynamic fault here: the instruction has to complete and the error be
   raised by what it produced.

   And `tb_ap040_pipe_oddtarget.v` FAILS in the random-enable build, on a
   defect this milestone introduced and has not explained. The seven
   exceptions it expects all happen, in order and with the right vectors --
   the probe confirms six vector 3 and one vector 14 -- and then an eighth
   arrives: an illegal instruction at `$05A2`, which is inside the stack
   region the frames have been walking down. Execution reaches memory it
   was never sent to, after the format-error handler returns. The bench is
   committed failing rather than trimmed to pass, because a bench that is
   quiet about a defect is worse than one that is loud about it.

   ### Milestones 94-96: the enable, the fourth review, and the fifth

   **Milestone 94 is the systematic version of the lesson.** Every bench
   in this suite tied `ce` high, and eight of the thirteen defects the
   first three reviews found had been hiding behind that. The harness
   gains `--ce-random` and every bench a conditional pseudo-random enable,
   so the suite now runs in four combinations of that and `--slow-l1`.
   `tb_ap040_pipe_nop.v` needed its lockstep snapshot gated on `ce`: taken
   on every edge it advanced on cycles the core did not, which is the
   bench making the same mistake the core had been making. Nothing else
   changed, and the suite passes in all four modes.

   **Milestone 95, the fourth review's four.** All confirmed.

   `a7_busy` listed MOVEC's auxiliary write at its COMMIT and not the
   cycle before, while the MOVEC is still in EX -- so an exception one
   instruction behind a MOVEC to the active stack pointer built its frame
   on the stack MOVEC had just replaced.

   The CHK comparison and the divisor test judged whatever the iterative
   divider was showing: `100/7` tripped divide-by-zero on the quotient it
   was still building, and a CHK against a bound its operand was within
   trapped. TRAPcc's detector had carried the guard since milestone 74 and
   the other two never got it. They wait for the producer now -- for the
   REGISTER source only, because the memory source's window is one cycle
   wide and requiring `!stall_in` there would drop the fault rather than
   delay it.

   A trace entry belongs to the instruction that just finished, not the
   one held behind it. Milestone 93 taught the frame's base to take the
   instruction's own A7 update into account, which is right for a fault
   and wrong for a trace: a held `MOVE.L (A7)+` moved the frame four
   bytes, exactly the step it had not taken.

   `LINK An,#d` decrements the stack pointer before it pushes An, so when
   An IS A7 what reaches memory is the decremented value. The core pushed
   the entry value.

   **Milestone 96, four of the fifth review's five.** These are
   architecture rather than pipeline: each is a place where this core and
   `rtl/ap040/ap040_core.v` -- which passes the cputest corpus -- disagree.

   A predecrement MOVEM whose list contains its own base register stores
   the initial value minus one operation size on the 68020 through 68040.
   This core stored it unchanged, which is the 68000's answer.

   CHK's N and a divide by zero's cleared C are architectural results of
   the fault. CHK's N was applied to the stacked word alone, so a negative
   operand stacked `$2708` and entered its handler with `$2700`; a handler
   that branches on its own flags and one that reads the frame would take
   different paths. The divide cleared C in neither. The faulting status
   register is resolved once now, and both readers take it from there.

   MOVE to SR and the ORI/ANDI/EORI immediates did not apply the
   architectural mask: `$2FFF` written to SR stayed `$2FFF` where the
   register has only `$271F` of bits. RTE already masked.

   | run | result |
   |---|---|
   | control, all eight benches on the RTL each was written against | all eight fail |
   | full suite, normal build | 115/115 |
   | full suite, slow build | 115/115 |
   | full suite, random enable | 115/115 |
   | full suite, random enable and slow build | 115/115 |
   | standalone fit | +1.668 ns at 25 ns, 5,864 ALMs |

   **What is NOT fixed.** The fifth review's remaining item is its only
   P1: odd targets fault for JMP and JSR and for nothing else. RTS and RTE
   returning to an odd address execute the instruction at the even one
   below it, and BRA and BSR to an odd target do the same, with BSR
   pushing a return address on the way. The check that exists reads
   `ea_target`, which only those two instructions compute; RTS takes its
   target from `mem_lane`, RTE from its own pop sequencer, and BRA and BSR
   from a redirect decode issues before EA-fetch sees them at all. Four
   different sources, and the frame each one stacks has its own bit-exact
   PC convention -- the existing JMP/JSR case records that those were
   "verified against ap040_core.v's own S_JMP1/S_JSR1, not guessed", and
   the same is owed here. It is the next milestone, not a loose end.

   ### Milestone 93: five more, and four of them are milestone 92's

   A third round of external review. Five defects, and the honest summary
   is that four were milestone 92's own fixes not carried far enough. Each
   had a bench that passed, and each of those benches was checking the
   wrong thing.

   **The hazard that held the wrong stages.** Milestone 92 gave EA-fetch a
   hazard for a reader of A7 behind a MOVEC to a stack pointer, and put it
   in `eaf_stall` -- which tells the stages BEHIND this one to wait. It
   never stopped EA-fetch. The held instruction retired, and re-issued its
   request, once per cycle of the hazard. The register-read check written
   with it passed because the instruction ran TWICE and the last pass wrote
   the right answer over the earlier ones. A push does not forgive that: it
   pushed twice, through the stale pointer, into a word nothing had asked
   it to touch.

   It emits a bubble now, and that branch has to come FIRST in the retire
   chain. Placed below the memory issue it set `mem_pending` for a read the
   gate had already suppressed, and the stage then waited for a return
   nobody had asked for -- bookkeeping without its request, which is the
   defect milestone 92 was about, reintroduced by the fix for it.

   **One frame, two stacks.** The exception sequencer read the stack
   pointer out of the register file live, once per beat. `MOVEA.L
   #$1200,A7` followed by `TRAP #0` put beat 0 at `$0FF8` and beat 1 at
   `$11FC`: half a frame at each, 512 bytes apart, and the RTE reads
   neither. The base is resolved once with the verdict now, and the verdict
   waits for any older A7 write to reach the register file.

   **Separate banks were not enough.** Milestone 92 gave the second write
   port its own stack bank, which fixed the user-mode fault: there the
   postincrement means USP and the exception's pointer means ISP. In
   SUPERVISOR mode they are the same register and two banks achieve
   nothing. Two things were wrong at once, and the reverts show both are
   load-bearing: the frame's base has to be the pointer AFTER this
   instruction's own increment when the two land on the same stack, and the
   register file's second write port must not outrank the first on the same
   physical register -- which is what the comment beside it always claimed
   and the code never did.

   **Two privileges, both read from the wrong place.** The first fetch of a
   handler went out as a USER program access: the privilege came from the
   committed status register, and the exception's switch to supervisor had
   not reached the commit point when that fetch was issued. And EX, which
   takes the memory's write port for address, size and data, did not take
   it for privilege -- so a user-mode read-modify-write posted as
   supervisor because the exception behind it was forcing supervisor for
   its own frame.

   **A defect the repository's memories cannot reach.** That last one
   needs EX's store to WAIT, and neither memory here can make it: both
   drain the write buffer before answering the read-modify-write's own
   load, so the store is accepted in the cycle it appears -- one cycle
   before the exception behind it has decided anything. Two benches were
   written before this was understood and neither reproduced it. The one
   that does, `tb_ap040_pipe_rmwsup.v`, drives `ap040_pipe_cpu.v` directly
   with a memory that holds the port busy for seven cycles, which is what
   a cache with a deeper queue does. The same unreachability is on record
   against milestone 48's `rmw_wait`, and it is the second time a real
   defect has been invisible to every memory model in the tree.

   **A regression the suite caught, from the fix itself.** The frame's bank
   select read `exc_m_r` in the very cycle `exc_m_r` was being latched, so
   it saw the value left by the PREVIOUS exception. A TRAP taken with M set
   built its frame from ISP and left MSP alone. Milestone 88's master-stack
   phase failed on it, which is the only reason it was caught before the
   milestone closed.

   | revert | caught by |
   |---|---|
   | the bubble removed | `creghold`, six checks |
   | the requests back on `stall_in` | `creghold`, the write count |
   | the verdict no longer waiting | `excbase`, four checks |
   | the base ignoring this instruction's increment | `faultpisup` |
   | the second write port outranking the first | `faultpisup` |
   | EX's store not carrying its privilege | `rmwsup` |
   | the fetch privilege back on the committed register | `excfc` |
   | the frame base latched rather than re-read | **nothing** |

   The last is real but redundant: with the verdict waiting for older A7
   writes, the base cannot move between beats anyway. It stays, because
   "resolve once" is the property that was wrong and the wait is a reason
   it happens to hold rather than a guarantee of it. That is the third such
   guard on record here, and they are listed rather than quietly kept.

   | run | result |
   |---|---|
   | control, all five benches on milestone 92's RTL | all five fail |
   | full suite, normal build | 107/107 |
   | full suite, slow build | 107/107 |
   | standalone fit | +1.932 ns at 25 ns, 5,829 ALMs |

   The fit is 0.49 ns below milestone 92's +2.426, which is a real cost
   rather than placement noise: the exception's verdict now waits on a
   condition computed from four forwarding comparators.

   The rule this leaves: **a stall signal names who waits, and a fix has to
   say which stage that is.** Three of these five are one stage's signal
   used as though it were another's -- a stall that held the wrong stages,
   a privilege read at the wrong end of the pipe, and a write port whose
   owner changed for three of its four fields.

   ### Milestone 92: eight defects found from outside

   An external review of the core produced two rounds of four
   reproducible defects each. All eight were confirmed, all eight are
   fixed, and every one of them had gone unnoticed through ninety-one
   milestones of benches for a single reason: **every bench in this suite
   ran the core with `ce` tied high and memory answering in one cycle.**
   That is not how the core will run.

   **Round one.**

   A store held behind a stalled EX was accepted again every time the
   write buffer drained. The request is combinational off `eac_valid` and
   says nothing about whether the instruction has had its turn -- right
   while its own `wr_stall` holds it, wrong while anything else does. One
   `MOVE.L D0,(A0)` behind a `DIVU.W` posted eighteen times. Every repeat
   writes the same frozen operands to the same frozen address, so RAM ends
   up correct and no value check can see it; a device register does not
   work that way.

   A commit pending when `ce` went low was dropped. `exe_fresh` was
   computed as `ce && !ex_stall`, so it cleared during a disabled cycle
   while EX's output registers, correctly gated on `ce`, held their value.
   With `ce` alternating every cycle, a four-write program committed
   **nothing at all**.

   A write to A7 took its bank from the forwarded SR, which belongs to a
   younger instruction. `MOVEA.L #$12345678,A7` followed by `MOVE D0,SR`
   put the value in USP and left ISP at zero -- the exact pair a
   supervisor uses to hand control to user code.

   A redirect accepted in the cycle a fetch was acknowledged published the
   abandoned word as valid and cleared the request for the new address.
   The guard compared against `a_addr`, which is assigned non-blocking
   earlier in the same block and still reads as the OLD address there, so
   it never fired on a same-cycle redirect.

   **Round two, and three of the four are the first round not finished.**

   Gating the ordinary store on `!stall_in` fixed the reported symptom
   rather than the defect. Every piece of bookkeeping that records a
   request as having happened -- `mem_pending`, the exception sequencer's
   phase, MOVEM's beat counter -- lives in the same `!stall_in` block, so
   reads and frame beats repeated for exactly the same reason:
   **thirty-four reads for one load, eighteen beats for one TRAP.** The
   comment that exempted the frame beats because they carry a sequencer
   was wrong, and said so in the source. Both request outputs now share
   that one enable.

   Giving the register file one write-side bank fixed the reported pair
   and not the general case. One instruction can write A7 TWICE, and when
   it faults the two writes mean different stacks: `DIVU.W (A7)+,D0` in
   user mode with a zero divisor must update USP through the
   autoincrement port and ISP through the exception's own result. With one
   select they both went to ISP, and the postincrement landed on top of
   the frame pointer. Port 2 now carries the bank the instruction itself
   was running in.

   MOVEC to a stack pointer commits through the register file's auxiliary
   port, which no forward reaches: a reader of A7 one instruction behind
   got the old value and caught up two instructions later. Two things were
   needed, and the mutations show both are load-bearing -- a bypass on the
   auxiliary write, and a one-cycle interlock so the read lands in the
   commit cycle where that bypass answers it.

   An exception's frame writes and vector read went out with USER function
   codes when the faulting instruction was in user mode. They are
   supervisor accesses whatever mode faulted. The privilege was read off
   the committed SR at the far end of the bridge -- which still says user
   while the frame is being pushed -- and read when the transaction was
   SENT rather than when the request was accepted.

   | revert | caught by | and not by |
   |---|---|---|
   | the store's `!stall_in` gate | `storeonce` | the other three |
   | `exe_fresh`'s hold | `cepause` | the other three |
   | the write-side bank | `a7bank` | the other three |
   | the redirect guard | `busredirect` | the other three |
   | port 2's own bank | `faultpi` | the other three |
   | the read request's gate | `storeonce` (37 reads) | the other three |
   | the frame beats' gate | `storeonce` (21 writes) | the other three |
   | the auxiliary bypass | `movecsp` | the other three |
   | the MOVEC interlock | `movecsp` | the other three |
   | the exception's supervisor override | `excfc` | the other three |
   | the A7 bypass's bank guard | **nothing** | all of them |
   | capturing privilege with the request | **nothing** | all of them |

   The last two are defence with no reachable trigger in the core as it
   stands, and are kept rather than removed: both protect a condition that
   is real and cheap to state, and the plan already records one such guard
   under milestone 48. They are listed here so nobody mistakes them for
   tested behaviour.

   **The fit moved, and the cause was not isolated.** 5,822 ALMs, Fmax
   44.3 MHz, slack **+2.426 ns** against milestone 91's +0.996, worst path
   21.95 ns. Only four of the forty worst paths still end at the array,
   where all of them used to, and the worst is now register-to-register
   inside EA-fetch. Two fits of this RTL agree to the picosecond, so the
   1.43 ns is real; which of the eight changes bought it was not measured,
   and it is recorded as an observation rather than a claim.

   **A second false regression, from a different disk.** Ten benches
   failed at once with `g++` exiting and no diagnostic. `/tmp` is a 31 GB
   tmpfs shared with everything else on this machine and it was at 100%.
   Milestone 90 had the same failure from `/home`, and the rule it left --
   check the machine before the RTL -- is what found it in one step this
   time. Both the bench harness and the fit script now put their
   temporaries in their own work directory.

   | run | result |
   |---|---|
   | control, all eight benches on the RTL each was written against | all eight fail |
   | full suite, normal build | 102/102 |
   | full suite, slow build | 102/102 |
   | standalone fit, twice | +2.426 ns at 25 ns, 5,822 ALMs, identical |

   The rule this leaves: **a bench that only ever runs the core with the
   clock enable high and memory answering instantly is not testing the
   core that will be built.** Eight defects lived behind that one
   assumption, and four of them are in the two signals every access
   crosses.

   ### Milestone 91: a store with a displacement

   `MOVE.sz Dn,(d16,An)`. Destination mode 101 was the last one the MOVE
   family could not reach; loads have had it since milestone 10 and the
   ALU family since milestone 40, and it is how every compiled function
   writes a local. The eleventh gather kind, and one that does not branch,
   so it joins `redirect_from_gather`'s exclusion list.

   **Three shapes were fitted, because the first one cost a nanosecond.**
   A store's address had been `an_base`, with the predecrement mode
   subtracting a step from it. The obvious change was to make that a
   general offset -- one adder where there was already one -- and it
   measured **+0.530 ns**, against milestone 90's +1.502. Quartus is
   deterministic here (milestone 90 was fitted twice, identical to the
   picosecond), so that is a real 0.97 ns, not noise.

   The second shape shared the LOAD's adder by making `ea_base` select
   `operand_b` for a store. One adder, a smaller address mux, and one more
   mux level on the load spine: **+0.849 ns**. Better, and still 0.65 ns
   short.

   The third shape keeps the load spine untouched. Decode points
   `eac_src_reg` at An for this form instead of at Dn, so `operand_a` is
   already the base and `ea_target` is already base plus displacement --
   the address falls through to the bottom of the address mux with no new
   adder and no new mux input, just `!eac_st_disp` ANDed onto a select
   that existed. The data and the flags then come from port B, both on
   registered assignments rather than the address path. **+0.996 ns**,
   5,849 ALMs.

   That is still 0.5 ns below milestone 90 and it is the honest number:
   reaching a new addressing mode from the store side costs something, and
   three shapes is where the search stopped rather than where it
   converged. The ordering is the useful part -- an adder on the address
   branch cost 0.97 ns, sharing an adder through a mux on `ea_base` cost
   0.65, and reusing the adder that was already there cost 0.51.

   **A mutation the memory model cannot see, and the check that replaced
   it.** Zero-extending the store displacement instead of sign-extending
   it passed every value check and both cores of the differential.
   `ap040_pipe_l1.v` indexes with `address[12:0]`, so the model wraps every
   8 KB -- and a 16-bit displacement that is zero-extended is off by
   exactly 65536, a multiple of 8 KB. It lands on the SAME word whatever
   address is chosen, so no sentinel anywhere could catch it, and the
   differential's own 64 KB model wraps the same way. The bench now
   records the address the core DRIVES at each of its six write posts and
   checks all six. That is the quantity in question and it does not depend
   on the memory model at all.

   | mutation | `stdisp` | `dual` |
   |---|---|---|
   | the displacement not applied | 7 checks | fails, round 0 |
   | the displacement zero-extended | 1 check (store 1's address) -- only after the address capture | passes: the model memory wraps at exactly the error |
   | not excluded from `redirect_from_gather` | 8 checks | the core never finishes |
   | `id_is_store` not set | 8 checks | fails, round 0 |
   | the address register read as Dn | 8 checks | fails, round 0 |
   | the data register not selected | 6 checks | fails, round 0 |
   | the store not writing condition codes | 1 check (D6) | fails, round 12 |
   | the size not taken from the MOVE field | 2 checks | fails, round 0 |

   The rule: **a bench that reads only memory contents cannot see an
   address error the memory model aliases away.** Where the address itself
   is the claim, check the address the core drives.

   | run | result |
   |---|---|
   | control, both benches on milestone 90's RTL | both fail; the differential reports vector 4 on `1b46 001e` |
   | full suite, normal build | 95/95 |
   | full suite, slow build | 95/95 |
   | standalone fit | +0.996 ns at 25 ns, 5,849 ALMs |

   ### Milestone 90: the quick forms' other two destinations

   `0101 qqq d SS mmm rrr` has reached only `mmm=000` since milestone 27.
   Both destinations it lacked are ones compiled code leans on: `SUBQ.L
   #8,A7` is how a small stack frame opens, and `ADDQ.L #1,(A0)` is a
   counter that lives in memory.

   **They are not one feature, and the decode says so.** The memory forms
   are milestone 89's read-modify-write with the immediate coming out of
   the opcode instead of a gathered extension word: `id_immrmw`, condition
   codes written, no register written. The An form writes a register,
   writes no condition codes, and is 32 bits wide whatever the size field
   says -- the ADDA/SUBA rule from milestone 44 -- so `id_size` is forced
   Long and Byte is excluded from the shape, because `ADDQ.B #n,An` does
   not exist. A decode that treats the two alike is wrong in both
   directions, and the bench is built to say which.

   **Fit:** 5,812 ALMs (5,802 after milestone 89), Fmax 42.56 MHz (42.79),
   slack **+1.502 ns** (+1.632), worst path 23.20 ns (23.12). No `quick`
   term appears on any of the forty worst paths. The 0.13 ns is inside the
   0.6 ns placement band. Two fits were run and agree to the picosecond,
   for the reason below.

   **A false regression, and what caused it.** The first full-suite run
   reported 34 failures across both builds, including
   `tb_ap040_pipe_nop.v` -- a bench that cannot fail for a decode reason.
   They were not simulation failures. They were `g++` exiting with
   `Error 1` and no diagnostic, because `/home` was at 100% with 64 KB
   free. The suite marks a bench failed when its log lacks a pass line,
   which a build failure also produces, so a full disk reads as a
   suite-wide RTL regression.

   The cause was this harness. Every bench leaves a Verilator build
   directory holding two precompiled headers of about 100 MB each; a full
   suite leaves roughly 10 GB, and ninety milestones of suite runs had
   accumulated **197 GB across 2,099 of them**, against 52 MB of logs --
   which are the actual evidence. `run_pipe_verilator.py` now deletes a
   bench's build directory as soon as that bench PASSES, and keeps it when
   the bench fails, which is when the binary is worth having. `--keep-obj`
   restores the old behaviour. Verified all three ways: a passing bench
   leaves 20 KB where it left 200 MB, a bench failed on purpose keeps its
   directory, and `--keep-obj` keeps it too.

   The rule: **a build failure and a test failure are not distinguishable
   from the pass line alone.** When a bench that has no way to fail fails,
   check the machine before the RTL.

   | mutation | `quickdst` | `dual` |
   |---|---|---|
   | the An form's width not forced Long | 2 checks (A1 and A2) | passes, as the bench's own comment predicts |
   | the An form writing condition codes | 1 check (D6) | fails, round 1 |
   | the memory form not writing condition codes | 1 check (D7) | fails, round 2 |
   | the memory form's register write leaked | 1 check (D0) | fails, rounds 0 and 1 |
   | `id_immrmw` not set for the memory form | 5 checks | fails, round 0 |
   | `id_src_a_is_imm` set for the memory form | 5 checks | fails, round 0 |
   | the memory form not marked an RMW | 5 checks | fails, round 0 |
   | Byte not excluded from the An destination | `ansrc_illegal`: 3 traps not 4, A0 = $AB | -- |

   The differential cannot see the width mutation, and that is by
   construction rather than by oversight. Every address register in the
   generator is a pointer the rest of the program dereferences, so an An
   destination left to drift would walk out of its scratch lane and
   eventually into the program. The add and its matching subtract
   therefore go in one slot and the pointer ends where it started -- which
   proves the form decodes in both cores, writes the register the opcode
   names, and (through the Scc capture milestone 89 added) does not write
   condition codes, but cancels a symmetric width bug. That is the
   dedicated bench's A1 and A2, and the two benches are complementary here
   rather than redundant.

   The Byte restriction went to `tb_ap040_pipe_ansrc_illegal.v`, which
   exists for exactly this and now runs four excluded forms rather than
   three. Its own rule still holds: it passes on RTL predating the
   feature, so it is validated by breaking the decoder rather than by a
   control run.

   | run | result |
   |---|---|
   | control, both benches on milestone 89's RTL | both fail; the differential reports vector 4 on `568c 578c` |
   | full suite, normal build | 94/94 |
   | full suite, slow build | 94/94 |
   | standalone fit, twice | +1.502 ns at 25 ns, 5,812 ALMs, identical both times |

   ### Milestone 89: an immediate straight into memory

   The second of the two decode gaps the differential found at milestone
   83, and the last one. ORI/ANDI/SUBI/ADDI/EORI/CMPI with a memory
   destination, modes `(An)`, `(An)+` and `-(An)` -- the same three
   `alu_dst_shape` already admits for the register-source direction.

   No new datapath. The read-modify-write path is milestone 48's: load
   through `mem_issue`/`mem_complete`, compute, store from EX through
   `ex_st_req`. What is new is where the ALU's second operand comes from.
   That path already crosses its operands over, because `SUB.L D0,(A0)` is
   memory MINUS D0 and the ALU computes `b op a`, so the loaded value has
   to be b. Here a is the gathered immediate rather than a register, which
   is one more mux on a registered assignment.

   **The field that collides is `eac_imm`.** For every other gathered form
   it is a displacement EA-fetch ADDS to the base register; here it is the
   operand. `id_immrmw` masks it at the adder's input -- `eac_imm &
   {32{~eac_immrmw}}` rather than a fifth way on the address mux, because
   this is the L1 address path and an AND folds into the LUT that already
   feeds the carry chain where a mux way is another level after it. The
   other half of the same collision is `id_src_a_is_imm`, which every
   register-destination immediate form sets and this one must NOT: here
   `operand_a` is the ADDRESS.

   **What the fit says about that choice:** 5,802 ALMs (5,787 after
   milestone 88), Fmax 42.79 MHz (42.78), slack at 25 ns **+1.632 ns**
   (+1.622), worst path 23.12 ns (23.06). Neither `immrmw` nor `ea_disp`
   appears on any of the forty worst paths. The 0.010 ns is inside the
   0.6 ns placement band and means nothing on its own; what it does mean
   is that the mask did not join the spine.

   **A bug found in development, by the bench, before any claim was
   written.** `ext_pending` keeps its own list of the two-extension-word
   forms, separate from `held_is_long`, and the first version updated only
   the second. The Long forms then gathered one word instead of two and
   the whole instruction stream desynchronised behind them. Two lists that
   have to agree, one of them updated: the same shape as milestone 87's
   own near-miss.

   **Two mutations passed, and both were coverage gaps rather than dead
   code.**

   The first was CMPI's operand crossover. Reverting it made the compare
   read D0 instead of the loaded value, and both benches passed. The
   differential's own blind spot is structural: it compares REGISTERS and
   MEMORY, and a compare's only product is condition codes, so it can see
   a reversed compare only when a branch happens to land right behind one.
   `tb_ap040_pipe_dual.v` now emits an `Scc Dn` in the slot after every
   compare, which puts the flags into a register the epilogue's MOVEM
   dumps. With that, the mutation fails two of the sixteen rounds. The
   dedicated bench had a matching flaw of its own: its poison was on the
   branch's fall-through path and its marker after it, so the marker ran
   whichever way the branch went. The poison value is now loaded BEFORE
   the compare and the marker is reached by NOT branching. Equal operands
   also cannot see a reversed compare, since a-b and b-a are both zero, so
   a second comparison is off by one and reads the borrow.

   The second was CMPI's `nowrite` flag. Clearing it makes CMPI request a
   store, and memory did not change -- because the ALU returns the
   DESTINATION unchanged for a compare, so the spurious store writes the
   same bytes back. It is still a real bus write, and on the bus-attached
   top it is a write cycle to an address the program only read. Memory
   contents cannot see it; the write PORT can. The bench now counts the
   posts the core makes to the L1's write buffer, which is exactly one per
   store, and asserts the program's six.

   | mutation | `immmem` | `dual` |
   |---|---|---|
   | the displacement mask removed | 7 checks | fails, round 0 |
   | the crossover's immediate source removed | 7 checks | fails, round 0 |
   | CMPI's loaded value not crossed over | 2 checks (both markers) | fails, rounds 3 and 5 -- only after the Scc capture |
   | `id_src_a_is_imm` not cleared | 8 checks | fails, round 0 |
   | `id_writes_reg` not suppressed | 1 check (D0) | fails, round 0 |
   | CMPI not marked `nowrite` | 1 check (the write count) -- only after the counter | passes: the store is invisible in memory |
   | the Long form's extension count | 7 checks | passes: a slot holds one extension word, so no Long immediate is generated |

   | run | result |
   |---|---|
   | control, both benches on milestone 88's RTL | both fail; the differential names the opcodes ($0c53, $0215, $0651, $0a50, $0452, $0c50, $0251, $0c54) as vector 4 |
   | `tb_ap040_pipe_immmem.v`, both builds | passes |
   | full suite, normal build | 93/93 |
   | full suite, slow build | 93/93 |
   | standalone fit | +1.632 ns at 25 ns, 5,802 ALMs |

   The rule this leaves: **a bench that reads only architectural state
   cannot see an access that writes the value already there.** Where an
   instruction's contract is that it does NOT touch memory, the check is
   on the port, not on the contents.

   ### Milestone 88: the exception frame starts a cycle after the fault

   Milestone 87's fit left the spine with no margin and named where 5.7 ns
   of it was: the CHK compare on a forwarded ALU result, `eac_is_chk_trap`,
   `exc_active`, and the exception's claim on the L1 address mux. The
   frame's first beat went out in the very cycle the fault was detected --
   a choice made at milestone 76, when the question was correctness -- and
   that put the whole fault-detection cone (the CHK compare, the divisor
   test on loaded data, the privilege check on a forwarded S bit, the
   odd-target test on the EA adder) on EVERY load's address path, exception
   or not.

   `exc_go` is the cone, registered. The sequencer's own signals -- the
   beats, the vector read, the completion -- key off it, so the address,
   size, write enable and read strobe the L1 sees are a register away from
   the cone. The stall and the retirement block still use the cone
   directly: the faulting instruction has to be held and not retired in the
   cycle it faults, and neither of those is on the L1's path. A wait branch
   covers the fault cycle so nothing lower in the chain runs for the
   instruction -- in particular not `ret_done`, which would otherwise let an
   RTE with a bad format word complete its pop in the cycle `fmterr_now`
   says otherwise. One cycle per exception entry; no bench changed.

   **First fit:** 5,787 ALMs (5,763), Fmax 41.85 MHz (39.85), setup slack
   at 25 ns **+1.107 ns** (-0.097), worst path 23.68 ns (24.46). The CHK
   cone is off the spine -- `chk` and `exc_` appear on none of the 40
   worst paths, where they were on all of them.

   **And the new worst path is the same problem's last trace.** ALU Z flag
   -> the milestone-74 CCR forward (`sr_resolved_ea[2]`) -> `trapcc_now` ->
   `eac_is_fmt2` -> the 8-vs-12 frame-size select -> `exc_beat_addr` ->
   `l1_addr_b`. The address mux's SELECT had left the cone; the frame's
   ADDRESS still depended on the fault type through the format nibble.
   One more register -- `exc_fmt2_r`, latched with `exc_go` -- and the
   frame shape is fixed at the same moment the verdict is.

   **Second fit, and it went backwards:** 5,802 ALMs, Fmax 39.97 MHz,
   slack **-0.021 ns**, worst path 24.38 ns. The path: Z flag -> the CCR
   forward -> `eac_is_trapcc_trap` -> `l1_addr_b`. Not through the frame
   size this time -- through the VECTOR. The vector fetch's address is
   `{exc_vec_num, 2'b00}`, and `exc_vec_num` is the fault-priority mux over
   the whole cone. I had latched the frame's shape and left its vector
   live: the same select-versus-data mistake, one mux to the right, and the
   fitter's placement happened to expose it as the worst path this time
   where the first fit had hidden it under the CHK cone. The stack-bank
   select is the last such term -- `sr_in[12]`, the M bit, is a forward
   from EX. Both are now captured with `exc_go`: `exc_vec_r` and `exc_m_r`.
   The frame's address is then `bank(exc_m_r) - (exc_fmt2_r ? 12 : 8) +
   phase`, and the vector's is `{exc_vec_r, 00}`: registers, constants, and
   muxes selected by registers, which is the rule stated below.

   **Third fit:** 5,787 ALMs, Fmax 42.78 MHz, slack **+1.622 ns**, worst
   path 23.06 ns. Seven of the forty worst paths still end at the L1's
   `q_b`, and they are the spine proper: a forwarded operand, the EA adder,
   `l1_addr_b`. The other thirty-three end at `eaf_operand_b`, and that
   path is the cone's new home: ALU -> `ex_fwd_data` -> `operand_a` -> the
   CHK compare -> `exc_active` -> the priority select of the stage's own
   next-value chain (the fault branch sits above the load and store
   branches, so every register the chain writes has the cone in its
   select). It is a register-to-register path inside the stage, with 1.6 ns
   to spare, and it is where the cone belongs: off the array, on a flop.
   The fitter packed that flop into the multiplier's input register, which
   is why the endpoint reads as `Mult1~mac|ax`.

   **The gap the mutations found.** Three registers were added and each
   was mutated to a constant before the claims below were written. The
   second one exposed a hole eighteen milestones old: `exc_m_r` forced to
   zero passed every exception bench, because no bench in the suite had
   ever taken an exception with M set. Every frame since milestone 15 had
   gone to ISP, so a sequencer that ignored M entirely was
   indistinguishable from one that honoured it. `tb_ap040_pipe_sup.v` --
   the bench that owns ISP/MSP/USP -- gains a fifth phase inside its
   privilege handler: `MOVE #$3000,SR` sets M, `TRAP #1` has to land its
   frame at MSP-8 with ISP untouched, the frame's SR word has to read
   `$3000`, and the handler's live SR has to keep M (only interrupts clear
   it). The third register's mutation found a smaller version of the same
   thing: `exc_fmt2_r` forced to zero passes `rte_fmt2`, `chkmem` AND
   `trapcc`, none of which looks at its own frame's shape -- a format-$0
   frame plus a format-$0 RTE is self-consistent. `addrerr`, `trace` and
   `integration4` do look, and fail it. The TRAPcc bench's blind spot
   stands as recorded: it proves the trap is taken, not that the frame is
   six words.

   | mutation | benches | result |
   |---|---|---|
   | `exc_vec_r` latched as constant 4 | trapcc, exc, sup | all three fail: handlers not reached, frame vector word `$0010` |
   | `exc_m_r` latched as 0 | sup, exc, trapcc | sup fails (MSP stays `$60`, frame words never written at `$58`); exc and trapcc pass, as neither sets M |
   | `exc_fmt2_r` latched as 0 | rte_fmt2, chkmem, trapcc | **all three pass** -- none checks the frame's shape |
   | `exc_fmt2_r` latched as 0, again | addrerr, trace, integration4, trapcc | addrerr fails (no word2, ISP -8 not -12), trace fails (fmt/vec `$0024` not `$2024`), integration4 fails; trapcc passes |
   | the fault-cycle wait branch removed | fmterr, exc, rte_fmt2 | all three fail (5, 6 and 2 checks) |

   | run | result |
   |---|---|
   | exception benches, after each of the three changes | 10/10 after `exc_go`, 10/10 after `exc_fmt2_r`, 11/11 (sup included) after `exc_vec_r`/`exc_m_r` |
   | extended sup bench, both builds | passes on the final RTL |
   | full suite, normal build | 92/92 after `exc_go`, 92/92 after `exc_fmt2_r`, 92/92 on the final RTL |
   | full suite, slow build | 92/92, 92/92, 92/92, same three points |

   The rule this leaves: **nothing combinational from a fault, a forward or
   a compare belongs on the L1 address path.** Every address the L1 sees
   should be a register, a register plus a constant, or a mux of those
   selected by a register. The fit reports enough to check it, and
   `paths40.txt` says when it stops being true.

   ### Milestone 87: shifts and rotates counted by a register

   The first of the two decode gaps the differential found in milestone 83.
   Bit 5 of `1110 ccc d ss i tt rrr` says where the count comes from, and
   only the immediate form decoded; the register form was an illegal
   instruction. `shift_shape` now accepts both, and decode points
   `id_src_reg` at the count register, so the count reads through port A
   and forwards from EX and WB like any other source operand -- no new
   path, which is the whole datapath cost.

   **Two cases come with it that the immediate encoding cannot express.** A
   count of more than 32, which the closed-form barrel already handled --
   it takes 1..63 and composes the one-bit steps. And a count of ZERO,
   which it did not: 0 in the immediate field means EIGHT, so the barrel
   had never been asked, and it computes a carry out of a shift that never
   happened and writes it to X. `ap040_pipe_alu.v` special-cases it now:
   the operand is unchanged, V and C are cleared, C takes X for ROXL/ROXR
   rather than 0, and X is left alone.

   `tb_ap040_pipe_shiftreg.v` checks a plain register count, a count of 33
   taken modulo 32, and a zero-count ROXL whose `SCS` result says C took X
   -- which makes that check as much about X as about C, since the wrong
   answer writes the phantom carry to both.

   | run | result |
   |---|---|
   | control: milestone-86 RTL | FAIL -- 3 of 4: the shifts are illegal instructions there, so nothing happens |
   | full suite, normal build | 92/92 |
   | full suite, slow build | 92/92 |

   **Two fits, because the first one was a lesson.** With the zero-count
   guard as first written -- `r = bm` forced alongside the flags -- the fit
   came back at 5,780 ALMs, Fmax 37.40 MHz, slack **-1.736 ns**, worst
   path 26.22 ns: a 32-bit mux had landed on the ALU's output path, on the
   spine, for a result the closed forms already produce at n = 0. Every
   form reduces to the identity there; only the four shifts' X and the two
   plain rotates' C were wrong. Two flag bits, then:

   | | ms 86 | ms 87, first | ms 87, two-bit guard |
   |---|---:|---:|---:|
   | ALMs | 5,598 | 5,780 | 5,763 |
   | ALU ALMs | 1,642 | 1,802 | 1,793 |
   | Fmax, slow 1100 mV 100 C | 40.38 MHz | 37.40 MHz | 39.85 MHz |
   | setup slack at 25 ns | +0.235 | -1.736 | **-0.097** |
   | worst path data delay | 24.59 ns | 26.22 ns | 24.46 ns |

   Two things remain from that, and they are different in kind.

   The +150 ALMs in the ALU are real and stay. Decode's immediate count
   never exceeds 8, so `eaf_shcnt[5:4]` had been constant zero all the way
   into the barrel and Quartus had folded two of its six levels away. A
   register count makes all six bits live. That is the cost of counts 0..63
   and there is no cheaper way to have them.

   The -0.097 ns is not a longer path: the data delay is 24.46 ns against
   24.59 ns on the SAME spine a milestone earlier, and the slack moved
   0.33 ns with the endpoint's placement, inside the spread every fit
   since milestone 75 has shown. What it says is that the spine has no
   margin left at 25 ns, and the next milestone that touches it will need
   to shorten it rather than hope. The path names where: from
   `ex_fwd_data` at 18.4 ns to `l1_addr_b` at 24.1 ns is 5.7 ns of CHK
   compare, `eac_is_chk_trap`, `exc_active` and the exception's claim on
   the L1 address mux -- the frame's first beat goes out in the very cycle
   the fault is detected, which was a choice (milestone 76), and it puts
   the whole fault-detection cone on every LOAD's address path. Starting
   the frame one cycle later takes it off. That is milestone 88.

   The differential generates register-count shifts with whatever the count
   register holds -- 0 to 63 after the modulo -- so the FSM core is the
   oracle for the rest of the flag semantics, which is exactly the part I
   would otherwise have had to derive from the manual by hand.

   One gap left from milestone 83: the immediate-to-memory forms
   (`ORI.B #x,(An)` and its family).

   ### Milestone 86: a sized data port, and Longwords at odd addresses

   The bytes of a Long at an odd address span THREE words. No lane select
   out of one aligned longword can reach them, which is why milestone 85
   fixed Words and stopped. The answer was not a wider mux but the port.

   **Port B carries a size now**, and the memory does all placement:
   right-aligned on the way out, placed from the address on the way in, at
   any alignment. That moved logic OUT of the CPU, which is the sign it was
   in the wrong place: `ap040_ea_fetch.v`'s `mem_raw` is now just
   `l1_q_b`, its `st_be`/`st_dat` are gone, and a MOVEM word beat and a
   sized store became the same expression; `ap040_execute.v`'s
   `ex_st_be`/`ex_st_data` became `ex_st_size` and `alu_result`.
   `ap040_pipe_l1.v` assembles a read across as many words as the size and
   alignment need, and `ap040_pipe_membus.v` hands size and address
   straight to the bus, where `ap040_bus16_adapter.v` already split any
   alignment -- for the third milestone running, the bus side needed
   nothing.

   The feature then fell out of the refactor: `tb_ap040_pipe_unaligned_long.v`
   passed the first time it was run.

   **One behaviour changed with it.** The L1's write buffer no longer
   forwards into a read; it drains first. A forward can only answer an
   exactly-matching access, and once size and alignment are in play
   "matching" stops being a comparison -- an overlap can be partial at
   either end. Draining gives the same answer for every overlap, and is the
   ordering `ap040_pipe_membus.v` has always had, so the array and the bus
   now order accesses alike. `tb_ap040_pipe_l1_wbuf`'s case C checks the
   same outcome by the new route, and says so.

   | run | result |
   |---|---|
   | control: milestone-85 RTL | FAIL -- 6 of 9: the load returns the ALIGNED longword ($12345678 for $3456789A) and the stores land a byte early |
   | full suite, normal build | 91/91 |
   | full suite, slow build | 91/91 |

   **Fit, and it is the most expensive milestone so far:** 5,598 ALMs
   (5,259 after milestone 85), Fmax 40.38 MHz (41.37), setup slack at
   25 ns +0.235 ns (+0.825), worst path 24.59 ns and ending, as ever, at
   `q_b`.

   Where it went is the point. `ap040_pipe_l1.v` went 572 -> 930 ALMs,
   `ap040_ea_fetch.v` 1,422 -> 1,349, and `ap040_pipe_cpu.v` as a whole
   4,432 -> 4,412 -- slightly SMALLER. The +339 is all memory-side: the
   read assembler and the write placer, which now mux bytes out of and
   into three words at four alignments. Two things to keep in view. The
   L1 is a stand-in for a cache, and a real cache needs that datapath
   anyway -- it is not overhead the design can avoid by leaving unaligned
   access unimplemented, only overhead it can MOVE. And the assembly sits
   at the tail of the critical spine, which is where the 0.6 ns went; if
   the spine needs room later, registering the assembly one cycle deeper
   is the obvious trade, at a cycle per load.

   The differential now starts A4 odd as well as A5, so every round
   exercises unaligned Longs and Words, through the bus, where they become
   byte/word/byte cycles. Both cores still agree on 15 registers and 8,192
   scratch words.

   With this the core's data path is alignment-agnostic for every size,
   which is what the 68040 promises software. The gaps the differential
   found are down to two, both decode reach rather than wrong behaviour:
   register-count shifts and the immediate-to-memory forms.

   ### Milestone 85: Word accesses at odd addresses

   Milestone 84's finding, fixed. A Word at an odd address is still inside
   the longword port B returns -- bytes 1 and 2 of it -- so the load is a
   lane select and the store a byte-enable pattern, with no extra access
   either way. Taking the high half whatever the address said read the byte
   BEFORE the one asked for.

   Three sites had the same assumption, which is why the differential saw
   it in loads, stores and read-modify-writes alike: `ap040_ea_fetch.v`'s
   `mem_raw`, its `st_be`/`st_dat`, and `ap040_execute.v`'s
   `ex_st_be`/`ex_st_data` for the RMW store. `ap040_pipe_membus.v` turns
   the new `0110` mask into a Word transaction at the odd address, which
   `ap040_bus16_adapter.v` already splits into two byte cycles -- so the
   bus side needed nothing, again.

   A Byte is never misaligned, and was always right. **A LONG at an odd
   address is still not implemented**: it spans three words, so it needs
   two accesses and a merge on the read side and two posted writes on the
   store side, which is a sequencer change in EA-fetch rather than a lane
   select. That is the next milestone, and the differential keeps A0-A4
   even so the rest of it can run meanwhile.

   `tb_ap040_pipe_unaligned.v` checks eight bytes the old code got wrong.
   The control is exact: on milestone-84 RTL seven of the eight fail, each
   by one byte -- $1122 for $2233, a store landing at $0900 instead of
   $0901 -- and the eighth, A0 after a postincrement, passes, because the
   pointer arithmetic was never what was wrong.

   The differential now generates these deliberately: A5 starts ODD in the
   prologue and is the pointer the new `MOVE.W`/`ADD.W`/`MOVE.B` forms use,
   so every round contains unaligned Word accesses, through the bus path
   where they become byte cycles. 16 programs x 96 slots still agree on 15
   registers and 8,192 scratch words.

   | run | result |
   |---|---|
   | control: milestone-84 RTL | FAIL -- 7 of 8, each one byte off |
   | full suite, normal build | 90/90 |
   | full suite, slow build | 90/90 |

   **Fit, same flow:** 5,259 ALMs (5,250 after milestone 81), ALU 1,532,
   Fmax 41.37 MHz (40.47), setup slack at 25 ns +0.825 ns (+0.288), worst
   path 23.58 ns. Nine ALMs for the lane selects, and the spine did not
   move -- the new muxes sit on the data return, not on the address path
   that the worst paths run through.

   ### Milestone 84: memory operands in the differential, and what they found

   The generated programs now touch memory. A prologue points A0-A6 into a
   16 KB scratch region seeded with random data, and seven forms join the
   register-only set: `MOVE.L` both ways through `(An)` and `(An)+`,
   `ADD.L` from and to `(An)` -- the read-modify-write path -- and
   `CMP.L (An),Dn`. The comparison grew with them: 15 registers and all
   8,192 scratch words. **16 programs x 96 slots, both cores agree on all
   of it.**

   Operands are Long and go through a pointer the prologue set, so `(An)+`
   keeps them even and nothing reaches below $4000, where the program, the
   vectors, the dump and the stack live.

   **`MOVEA.L Dm,An` is deliberately not generated, and that is the
   milestone's finding.** The first run with memory operands generated it,
   which puts an arbitrary value in a pointer, and the two cores then
   diverged by exactly one byte: $6FE3FEF0 on the pipelined core where the
   FSM core had $8D6FE3FE, and a stored longword landing one byte apart in
   the two memories. That is an UNALIGNED longword access. A 68040
   performs one in hardware -- `ap040_bus16_adapter.v` even splits it into
   byte/word/byte, which is why the FSM core gets it right through the same
   adapter -- and this core silently accesses the aligned longword instead.

   Not a decode gap: the instruction decodes and executes, and produces the
   wrong value. It is the first outright WRONG behaviour the differential
   has found, as opposed to a missing instruction, and it needs its own
   milestone: port B does one aligned 32-bit access, and an access spanning
   two of them changes EA-fetch's `mem_issue`/`mem_complete` sequencer, the
   store path's lane mask, and `ap040_pipe_membus.v`'s sizing.

   | mutation | result |
   |---|---|
   | a Long store writes only its high half | FAIL -- every round: the pipelined core never finishes, since the done flag is a Long store too |
   | the read-modify-write operand crossover removed | FAIL -- round 0, four registers |
   | full suite, normal build | 89/89 |
   | full suite, slow build | 89/89 |

   Still not covered: no displacement or absolute addressing in the
   generator, no byte or word memory operands (they would make a pointer
   odd, which is the gap above), no MOVEM, no traps, no supervisor state.

   ### Milestone 83: checked against the FSM core, not against me

   Every bench before this one compares the pipelined core to values I
   worked out by hand. `tb_ap040_pipe_dual.v` compares it to
   `rtl/ap040/ap040_core.v`, which passes 3,797 of 3,801 cputest slices --
   recorded 68040 hardware behaviour. Where the two disagree, the
   pipelined core is wrong.

   Both cores run the same generated program, each behind its own 16-bit
   bus (milestone 82 is what makes that a comparison of CPUs rather than of
   bus models) and its own copy of the same memory. Neither core's
   internals are read: the PROGRAM dumps its own state with one `MOVEM.L
   D0-D7/A0-A6,$1000` and then writes a done flag, so what is compared is
   architectural state in memory. Any exception on either side lands on a
   shared handler that records the vector and the stacked PC, so a trap
   names the instruction instead of hanging.

   Programs are generated from a seeded xorshift in FOUR-BYTE SLOTS: a
   one-word instruction is padded with a NOP, so every slot boundary is an
   instruction boundary and a `Bcc` displacement of 4k-2 always lands on
   one. That is the whole trick that makes random 68k generation tractable
   without an assembler. Register-to-register forms only -- no memory
   operand can wander -- A7 is never a destination, and divides are left
   out because a divide by zero is a trap.

   **16 programs x 96 slots: both cores agree on all 15 registers.** 1,536
   generated instructions, the first validation of this core against
   something other than my own arithmetic.

   **Two bench bugs on the way, both of which looked exactly like core
   findings**, which is the hazard of this technique:

   - `rbits()` returns 32 bits. Dropped into a concatenation it contributes
     all 32 and pushes the opcode out of the low 16, so `MOVEQ #$17,D1`
     ($7217) was generated as $0017 -- `ORI.B #x,(A7)`, which the FSM core
     implements and this one does not. Vector 4, and it looked like a
     missing instruction until the generator was re-run in Python.
   - The shift encoding is `1110 ccc d ss i tt rrr`, and I had the
     count-source bit and the type field the wrong way round, generating
     register-count shifts. Milestone 23 implemented immediate counts only,
     so that one IS a real gap -- recorded below -- but it is not what the
     generator meant.

   | mutation of the pipelined core | result |
   |---|---|
   | `ADDQ/SUBQ #0` no longer means 8 | FAIL -- rounds 2, 4, 5: A3, A6, D4, D5, A2 |
   | MOVEQ zero-extends instead of sign-extending | FAIL -- round 0, five registers |
   | ASR loses its sign fill | FAIL -- rounds 2, 10, 14: A0 = $1FFFFFFF for $FFFFFFFF |
   | EXT.L and EXTB.L swapped | **passes** -- the generator never emits EXTB.L, so the bench cannot see it. A differential tests what it generates, and this one's instruction set is the honest limit of the result above |
   | full suite, normal build | 89/89 |
   | full suite, slow build | 89/89 |

   **What this does not cover yet**, in the order it should grow: no memory
   operands (so no addressing modes, no stores, no MOVEM beyond the dump),
   no divides or other traps, no supervisor state, and the comparison is
   15 registers -- flags only insofar as the conditional branches in the
   program act on them. Each of those is a generator change, not a harness
   change, which is the point of building it this way.

   **Gaps this found in the core itself:** register-count shifts
   (`ASR.L D1,D2`) are not decoded -- milestone 23 built the immediate-count
   forms and the register-count ones were never added. `ORI.B #x,(An)` and
   the other immediate-to-memory forms are not decoded either. Neither is a
   defect in what exists; both are decode reach, and the differential will
   keep finding this class as the generator widens.

   ### Milestone 82: the 16-bit Minimig bus, with the FSM core's own adapter

   `ap040_pipe_bus16.v` is the third pairing of `ap040_pipe_cpu.v`: CPU ->
   `ap040_pipe_membus.v` -> `rtl/ap040/ap040_bus16_adapter.v`, the latter
   instantiated **unmodified**. That was the whole point of milestone 81
   emitting `rtl/ap040/ap040_core.v`'s external port rather than inventing
   one: the adapter needed nothing, and the pipelined core now presents the
   same interface to a host that the FSM core does -- `addr_out`/`data_in`/
   `data_write`, `nwr`/`nuds`/`nlds`, `busstate`, `longword`, `fc`, and one
   qualified `clkena_in` pulse per 16-bit sub-cycle.

   **Two enables, and they are not the same thing.** `ce` advances the CPU;
   `clkena_in` advances the bus. `ap040_tg68k_compat.v` hands the FSM core
   one enable for both, because that core has nothing to keep doing while a
   transfer is outstanding. This one does -- five stages of it -- and
   collapsing the two would throw that away. The port keeps them separate.

   `tb_ap040_pipe_bus16.v` runs tb_ap040_pipe_bus.v's program, so the two
   benches differ only in what is under the CPU, and drives
   `tests/ap040/tb_dat_replay.v`'s memory model: `data_in` is the whole word
   at `addr_out`, the lanes decide which half a write lands in, and the
   answer takes 1 to 8 cycles. It came up on the first run: **78 sub-cycles
   -- 64 fetch, 8 read, 6 write** -- which is exactly one Word cycle per
   fetch and two per Long access, four Long reads (the load, the vector,
   two RTE pops) and three Long writes (the store and two frame beats).

   What it checks beyond the program's result is the splitting, because
   that is all this layer does: the Long store must appear as two word
   sub-cycles, $1234 to $0800 then $5678 to $0802, both with `longword`
   asserted and both lanes enabled; a fetch must take one sub-cycle; no
   sub-cycle may select neither lane; function codes must be right. A
   bridge that emitted the halves in the wrong order leaves memory wrong
   and the program catches it. One that emitted them as four byte cycles
   leaves memory RIGHT, and only the sub-cycle counts catch it.

   The adapter itself is shared code with its own bench
   (`tests/ap040/tb_ap040_bus16_gap.v`, the sampled idle cycle between
   sub-cycles); nothing here re-tests it, and `pipe_mutate.sh` cannot reach
   it -- it only edits `rtl/ap040_pipe/`, which is the right boundary.

   | mutation | result |
   |---|---|
   | writes issued as Word | FAIL -- one store sub-cycle instead of two, carrying $5678 to $0800: `[$0800] = $5678BEEF`, and D1 reads it back |
   | fetches not marked as instruction | FAIL -- 0 fetch sub-cycles, 64 with the wrong function code: they went out as data reads |
   | write no longer beats a waiting read | FAIL -- D1 = $DEADBEF0 again, and the store never reaches the bus before the program derails |
   | full suite, normal build | 88/88 |
   | full suite, slow build | 88/88 |

   **No new fit.** This milestone adds a module the synthesised top does
   not reach -- `tests/ap040/pipe_synth/` fits `ap040_pipe_core`, and
   nothing in `ap040_pipe_cpu.v` or below changed -- so milestone 81's
   numbers stand unchanged: 5,250 ALMs, 40.47 MHz, +0.288 ns at 25 ns.
   Fitting `ap040_pipe_bus16` instead would need `ap040_bus16_adapter.v`
   added to the project as well, which is why it is not in the file list:
   the adapter is `rtl/ap040`'s, and this project deliberately builds only
   `rtl/ap040_pipe`.

   ### Milestone 81: the core on a bus

   Second step of the bus axis, and the one that makes the CPU a component
   rather than a thing wrapped around its own memory. Three changes, each
   needed by the next.

   **Byte addresses.** `ap040_inst_fetch.v` and `ap040_ea_fetch.v` used to
   emit `(addr - PC_RESET) >> 1` -- an index into `ap040_pipe_l1.v`'s array
   layout. Nothing else could ever have been attached to a port like that.
   They now emit the 32-bit byte address and the L1 does its own mapping
   from a `PC_RESET` parameter of its own, so the array's contents, and
   every bench's `dut.u_l1.mem[N]`, are unchanged.

   **The split.** `ap040_pipe_cpu.v` is the pipeline, with the L1 protocol
   at its boundary. `ap040_pipe_core.v` is now a thin wrapper pairing it
   with the array -- the same module name the 86 existing benches
   instantiate, with `u_l1` still at the top of it, so only the paths into
   the CPU's own state moved (`dut.u_regfile` -> `dut.u_cpu.u_regfile`,
   and `sr`/`vbr`/`sfc`/`dfc`/`cacr` likewise; 213 references, mechanically
   rewritten). `ap040_pipe_sys.v` is the other pairing: the same CPU with
   `ap040_pipe_membus.v` under it and no array at all.

   **The bridge.** `ap040_pipe_membus.v` turns the two CPU ports into one
   transaction at a time on `rtl/ap040/ap040_core.v`'s own external port --
   `mem_req` held until `mem_ack`, a single-cycle ack with `mem_rdata`
   valid, `mem_size`/`mem_instr`/`mem_fc` alongside -- which is the port
   `ap040_bus16_adapter.v` already converts to the 16-bit Minimig bus. So
   the next step needs no reshaping on either side. Its rules:

   - a posted write goes out before any waiting read, which is how the
     bridge gets the array's write-buffer forwarding for free: drain first
     and memory gives the same answer;
   - a data read beats a fetch, because a fetch can be re-issued and a load
     cannot;
   - a fetch whose address changed while it was on the bus (a redirect) has
     its result discarded and is re-issued, matching the array's port-A
     restart;
   - sizes come from the lane mask: `1111` Long, `1100` Word, `1000` Byte
     at the even address, `0100` Byte at the odd one, which are the only
     four patterns the CPU produces.

   `tb_ap040_pipe_bus.v` runs a program through a memory model that answers
   in 1 to 8 cycles: an immediate, a Long store, a Long load of the address
   just stored (the ordering case -- the store is still posted when the
   load issues), a TRAP with its frame push and vector fetch, and an RTE.
   It checks the registers and memory, and also the bus itself: every fetch
   a Word with a supervisor-program function code, every data access
   supervisor-data, and exactly one Long write to $0800 -- a bridge that
   split the store into bytes would leave the right memory contents and
   fail here. Vector 33 sits at byte $84, its architectural address, rather
   than the array benches' aliased word index 3650: a consequence of the
   CPU emitting byte addresses that the array's wrapping had hidden.

   **What the bus bench found.** 347 fetch transactions for a program that
   issues 64. Once the fetcher has nothing left to fetch, `l1_req_a` kept
   firing at the same address every cycle -- an array answers that for
   free, a bus does not. The request is now gated on `have_more` while the
   ADVANCE that clears `if_pend` is not, because that advance is what lets
   the pipeline drain; separating the two took one wire. 64 transactions
   for 64 fetches after it, and the bench asserts the bound.

   | mutation | result |
   |---|---|
   | write no longer beats a waiting read | FAIL -- D1 = $DEADBEF0, the pre-store value plus one: the load overtook the store it should have seen, and the program never recovers |
   | fetch request no longer gated on `have_more` | FAIL -- 347 fetches for a 64-word budget, the storm this milestone found |
   | data reads issued as Word | FAIL -- D1 = $00001235: half the longword, and the RTE pops come back short (ISP $05F8) |
   | every transaction carries the program function code | FAIL -- 7 transactions with the wrong code |
   | a redirected fetch returns its stale word | FAIL -- D2, D3 zero: the TRAP handler never runs |
   | L1 drops its PC_RESET mapping | FAIL -- integration3 and movem, on the array side: the mapping the CPU no longer does is live in the L1 |
   | full suite, normal build | 87/87 |
   | full suite, slow build | 87/87 |

   The first row is the one worth keeping in mind. The array answers a read
   from its write buffer, so ordering was never a question on that side;
   the bridge has to make it one, and a bench that only checked memory
   contents at the end would have passed both ways.

   **The fit caught a real regression, and the path named it.** First fit
   after the three changes: 5,272 ALMs, Fmax 39.12 MHz, setup slack at
   25 ns **-0.562 ns** -- 1.05 ns worse than milestone 80's +0.488 and
   outside the +0.34..+0.96 spread four previous fits had shown for
   changes that did not touch the spine. This one did touch it: the spine
   ends at the L1's address.

   The worst path read `eaf_operand_b` -> the ALU -> `ex_fwd_data` ->
   `an_base` -> the EA adder -> `ea_target` -> `mem_raw` -> `divzero_now`
   -> `eac_is_divzero` -> `eac_is_fmt2` -> **`Add9`** -> `l1_addr_b` ->
   `q_b`. `Add9` is `exc_sp_bank - exc_frame_size`, and
   `exc_frame_size` is `eac_is_fmt2 ? 12 : 8` -- so the format select,
   which since milestone 77 depends on the loaded divisor, was driving a
   32-bit subtract at the very end of the longest path in the design.
   Computing both `bank - 8` and `bank - 12` in parallel and letting the
   format pick one leaves the subtracts off the path:

   | | ms 80 | ms 81, first fit | ms 81, carry-select |
   |---|---:|---:|---:|
   | ALMs | 5,275 | 5,272 | 5,250 |
   | Fmax, slow 1100 mV 100 C | 40.80 MHz | 39.12 MHz | 40.47 MHz |
   | setup slack at 25 ns | +0.488 | **-0.562** | +0.288 |
   | worst path data delay | 23.90 ns | 24.99 ns | 24.13 ns |

   Two things worth keeping. A constant-offset subtract that a SELECT
   feeds is a carry-select waiting to happen, and there are more of them in
   `ap040_ea_fetch.v`'s address tail if the spine needs more room later.
   And `tests/ap040/pipe_synth/` now carries `paths40.tcl` and run.sh runs
   it, printing the worst path's delay and leaving the 40 worst in the
   workdir -- this is the second time that script had to be rewritten from
   memory after a workdir was cleaned up.

   ### Milestone 80: the pipeline waits for memory

   First step of the bus axis. Until now every L1 access completed in one
   cycle by construction, and every requester was built on that: the
   instruction fetcher registered `if_valid` at the request, decode's
   multi-word gather counted cycles, and EA-fetch's four readers (operand
   load, vector fetch, RTE pop, MOVEM) each took `l1_q_b` the cycle after
   driving the address. Nothing outside the L1 model can be attached to a
   pipeline like that, so this milestone makes memory latency a variable
   before anything is behind it.

   **The L1 grows a request/return handshake on both ports**: `en_a`/
   `rvalid_a`, `rd_b`/`rvalid_b`, data and valid held until the next
   accepted request on that port. Port A restarts on a new request (a
   redirect abandons the fetch in flight); port B allows one outstanding
   read. In the normal build every read still returns the cycle after its
   request and the write buffer drains the cycle after a post -- the timing
   every bench was written against, and the fast suite is unchanged. With
   `AP040_PIPE_L1_SLOW` a deterministic xorshift adds 0-3 cycles to every
   read and every drain, and the SAME 86 benches run again
   (`run_pipe_verilator.py --slow-l1`; `PIPE_HARNESS_ARGS=--slow-l1` for the
   control and mutation helpers). Every bench's end-of-program wait is now
   `repeat (N * AP040_PIPE_WAIT_SCALE)`, 1 normally and 4 in the slow
   build, because a program takes about three times as long there.

   **What the slow build found, in order, on its first seven benches:**

   1. EA-fetch's chain had no state for "read in flight": with the return a
      cycle late the chain fell through to the default branch and DEPARTED
      the instruction with no data. Three wait branches now hold a bubble
      (operand load, vector fetch, RTE pop); MOVEM's `rd_pend` waits for the
      return before `rf3_we`. Two orderings I got wrong on the first pass:
      the vector wait sat before `exc_vec_done` in the chain and won forever
      once the vector returned, and the load wait pre-empted an active
      exception on a faulting memory-source load (its data HAS returned;
      `mem_pending` stays set through the entry).
   2. A misdiagnosis, kept because the mutation table caught it. Seeing
      opcode words land in data registers, I first blamed a lost decode
      redirect (a one-cycle pulse arriving while a fetch was in flight) and
      added `|| redirect_valid` to the fetch request. The real cause was
      item 3. The mutation that removed the term passed the slow build,
      and the reason is a two-line argument: decode redirects only on a
      word it can see, `if_valid` implies `rvalid_a` implies `can_issue`,
      so the redirected fetch always issued. The term is gone
      (`60235da0b`), and the comment where it stood says why.
   3. Decode's gather consumed a word every unstalled cycle whether or not
      one had arrived: `MOVE.L #$64,D0` executed as `#$203C203C`, the held
      opcode taken twice as its own immediate. The whole decode step is now
      gated on `if_valid`, and `redirect_from_gather` too, so a bubble
      mid-gather neither counts nor redirects on a stale word.
   4. A store from EX into a write buffer still draining was fine after
      all -- EX already holds `ex_st_req` under `rmw_wait` until the buffer
      accepts -- but the one-cycle drain had never once exercised that wait.

   None of these is visible in the normal build, which is the thesis of the
   milestone and of the mutation table below.

   Every mutation below passes the normal build's benches. That is the
   finding of the milestone as much as the fixes are: a one-cycle memory
   cannot see any of them.

   | mutation (slow build) | result |
   |---|---|
   | operand-load wait branch removed | FAIL -- aluax, 3 checks: a `(An)+` load departs twice and increments An twice; benches without a postincrement load pass, the stale departure's register write being overwritten by the right one |
   | vector-fetch wait branch removed | FAIL -- integration4, 2 checks |
   | RTE-pop wait branch removed | FAIL -- integration4, USP $05F4 for $0500: the RTE departs twice; fmterr, rte_fmt2 pass |
   | `mem_complete` ignores `rvalid_b` | FAIL -- 2 of 4 benches, 3 checks each |
   | decode consumes on a fetch bubble | FAIL -- every bench in the set |
   | MOVEM `rf3_we` ignores `rvalid_b` | **passes** -- unobservable: the pending-clear is still gated, so the stale write is followed by the right one to the same register before anything reads it; kept because a forwarded read of that register in between would see the stale word once a real bus makes the wait long |
   | `\|\| redirect_valid` in the fetch request | **passes** -- redundant, see item 2; removed |
   | full suite, normal build | 86/86 |
   | full suite, slow build | 86/86 |

   The rule this adds: **a milestone that changes timing needs a build in
   which the timing is different**, or its benches test the old timing
   twice. The slow build stays; every later memory-side change runs the
   suite in both.

   **Fit, same flow (of `094f97546`):** 5,275 ALMs (5,220 after milestone
   79), 7,197 combinational ALUTs (7,115), 2,250 registers (2,211), Fmax
   40.80 MHz (40.55), setup slack at 25 ns +0.488 ns (+0.342), TNS 0. The
   40 worst paths are the same spine; neither port's valid nor `if_pend`
   is on any of them. The handshake cost 55 ALMs and 39 registers.

   ### Milestone 79: T0, trace on change of flow

   With T1T0 = 01 the trace arm is taken only by the instructions the
   68040 defines as changes of flow: taken Bcc/DBcc, BSR/JMP/JSR, RTS/RTE,
   every exception entry, and the non-branch ones that resynchronise the
   pipeline -- MOVE to SR, ORI/ANDI/EORI to SR, MOVEC to a control
   register, NOP. That list is `ap040_core.v`'s `t0_special`, which cputest
   confirmed on hardware (its comments record BSET D5,(A6) once tracing by
   mistake under T0 until CAS was told apart from it); MOVE An,USP and
   MOVES/CAS/CINV/CPUSH/FSAVE are not decoded in this core and so do not
   arise. NOP gained a decode flag for it. T1 still traces everything, and
   T1T0 = 11 behaves as T1.

   The one design point: a conditional branch's taken-ness is known in EX,
   not where the arm is written. So Bcc/DBcc arm provisionally
   (`trace_arm_cond`), and EX exports its verdict -- `ex_br_resolve` and
   `ex_br_taken`, from the `cond_result`/`dbcc_branch_taken` it already
   computes -- which confirms or cancels the arm the cycle after the branch
   departs. The next instruction is held for at least that long, so no
   departure write can race the verdict. This is independent of decode's
   prediction policy; a flush is not used as a proxy for "not taken".

   `tb_ap040_pipe_trace_t0.v`: a not-taken BEQ and a not-taken DBF leave
   nothing; a taken BNE, NOP, JSR, RTS, ORI to SR, a taken DBF, a TRAP and
   the MOVE to SR that clears T0 leave eight frames, logged and checked
   field by field. Eight and not nine or ten is the point.

   **The bench was wrong twice before the RTL was right once.** Its first
   listing was one word off from K12 on -- the DBF's displacement word --
   so the taken DBF landed in the poison slot, and it had put the
   subroutine at $0440, where the end of the program fell through into its
   RTS and popped garbage (ISP $0604). The RTL had traced exactly what ran;
   the cycle monitor from milestone 78 said so within one run. Subroutines
   go past $0800, as integration3 already knew.

   | run | result |
   |---|---|
   | control: milestone-78 RTL | FAIL -- T0 ignored, D7 = 0 |
   | NOP off the list | FAIL -- 7 entries, entry 2 gone |
   | no verdict (not-taken branches keep the arm) | FAIL -- 10 entries, the BEQ and the second DBF traced |
   | verdict inverted | FAIL -- entries 1 and 6 are the not-taken pair instead |
   | exception entry not a T0 flow change | FAIL -- 7 entries, the TRAP's gone |
   | MOVE to SR off the list | FAIL -- 7 entries, the last one gone |
   | RTS off the list | FAIL -- 7 entries, entry 4 gone |
   | full suite on milestone-79 RTL | 86/86 |

   **Fit, same flow:** 5,220 ALMs (5,225 after milestone 78), 7,115
   combinational ALUTs (7,088), Fmax 40.55 MHz (41.24), setup slack at 25 ns
   +0.342 ns (+0.752), TNS 0. The 40 worst paths are the same spine, ALU ->
   `ex_fwd_data` -> EA adder -> L1 address -> `q_b`; neither the arm nor
   the verdict is on any of them. Four fits since milestone 75 have put
   the slack between +0.34 and +0.96 ns for changes that never touched the
   spine -- that spread is the fitter, and the spine's real margin at
   25 ns is the low end of it.

   This closes trace. With it, every exception the 68040 raises without a
   bus, an MMU, an FPU or an interrupt controller is implemented and has a
   bench: illegal, privilege, TRAP, TRAPcc, CHK, zero divide, address error
   (odd JMP/JSR), format error, and trace in both modes. What is left in
   the exception model is the hardware-facing part -- interrupts and the
   throwaway frame, access error -- and that belongs with the bus.

   ### Milestone 78: T1 instruction trace, and the flush cycle it exposed

   **Trace.** The traced instruction is the one that leaves EA-fetch with
   T1 set in its start SR; it arms `trace_arm`/`trace_pc` on its way out.
   The exception is delivered on the instruction that FOLLOWS it: that
   instruction is held in EA-fetch until EX and WB have drained (`wb_busy`
   is the core's `exe_valid`), and is then turned into a format-$2 vector-9
   entry whose PC field is its own address and whose address field is the
   traced instruction's -- with the stacked SR and the stack pointer read
   from the real registers, after the traced instruction's own writes.
   None of the held instruction's semantics happen: `own_exc` keeps its own
   fault out, and `!trace_hold` gates its read, its push/store write, its
   MOVEM and its RTE pop. The frame's PC brings it back after the handler's
   RTE. So a traced TRAP is traced on its handler's first instruction,
   after the exception processing; a traced MOVE to SR that clears T1 is
   still traced, with T1 clear in the frame, so the handler's RTE returns
   with tracing off; a traced RTE stacks the SR it restored and the PC it
   went to. `trace_arm` is not cleared by a flush -- the flush after a
   traced branch or a traced TRAP kills the instruction in EA-fetch, and
   the trace is still owed to whichever instruction arrives next -- and
   clears only when the trace entry itself departs. T0 (change of flow)
   is not implemented; T1T0 = 11 behaves as T1.

   `tb_ap040_pipe_trace.v` sets T1 and logs every trace frame's four
   fields at (A5)+: fifteen entries across a taken branch, a store, a
   TRAP, an RTE from a hand-built frame, and the MOVE to SR that turns
   tracing off. The SR column carries each instruction's own CCR result,
   so it also says the stacked SR is the one AFTER the traced instruction.

   **The bench's first run found a defect older than trace.** The third
   entry never completed: the trace frame's second longword came back as
   `{2700, 0000}`. A cycle monitor showed why. The instruction behind an
   exception entry waits in EA-calc through the frame push and moves into
   EA-fetch the cycle the entry departs -- which is the cycle EX raises
   `flush` for it. The output block ignores that cycle (flush has priority
   there), but the stage's combinational side effects did not: for one
   cycle that instruction was live. Here it was the TRAP after the traced
   store, and its own beat 0 went out at ISP-8 before the trace entry's A7
   had committed -- on top of the trace frame. Without trace the same
   cycle exists after EVERY exception entry: a store behind a TRAP wrote
   to the pre-handler A0, and a TRAP behind a CHK wrote its `{SR, PC_hi}`
   over the CHK frame's `{PC_lo, fmt/vec}`. `tb_ap040_pipe_excexc.v`
   shows both without trace; on milestone-77 RTL the old A0 receives $11
   and the CHK handler's RTE goes to $2700. The fix is one predicate,
   `live = eac_valid && !flush`, on everything this stage sends to the L1
   on the instruction's behalf: `mem_issue`, `wr_stall`, `exc_active`,
   `ret_active`, `l1_wren_b`.

   A second thing the bench forced: `eac_is_store` outranks `exc_writing`
   in `l1_addr_word` and sets `st_be` to the store's size, so the trace
   frame for a held STORE went to the store's address with the store's
   lanes. Stores and exceptions had never coincided before. `store_now`
   (the store predicate gated by the hold) now feeds every functional use.

   And one thing the bench got wrong: its handler used D0 as scratch, and
   the traced program's `MOVE D0,SR` then loaded the handler's leftover --
   the fmt/vec word, $2024 -- into SR, dropping to user mode with IPL 0.
   The RTL was right and the stacked SR said so. The handler uses D4.

   | run | result |
   |---|---|
   | control: trace bench on milestone-77 RTL | FAIL -- D7 = 0, no entries |
   | control: excexc bench on milestone-77 RTL | FAIL -- [$0B00] = $11; D5, D6 unset; ISP $05FC |
   | `live` without `!flush` | FAIL -- excexc as the control; trace as its own first run (entry 3's PC field $2700) |
   | trace entry re-arms | FAIL -- 17 entries, the handler traced (entry 1 = {$040C, $0800, $2700}) |
   | no drain wait | FAIL -- entries 12-14: SR $2700 for $A708, PC field `a708042c`: the frame written under in-flight pushes |
   | held store still selects the L1 | FAIL -- D7 = 2, entry 3 missing: the frame went to (A6) |
   | held instruction's own fault taken | FAIL -- 13 entries; 8, 13, 14 wrong |
   | RTE never arms | FAIL -- 14 entries; entry 13, the RTE, missing |
   | flush clears the arm | FAIL -- 13 entries; entry 5, the TRAP's, missing |
   | full suite on milestone-78 RTL | 85/85 |

   **Fit, same flow:** 5,225 ALMs (5,199 after milestone 77), 7,088
   combinational ALUTs (6,999), EA-fetch 1,365 ALMs (1,320), Fmax 41.24 MHz
   (41.59), setup slack at 25 ns +0.752 ns (+0.957), TNS 0. The 40 worst
   paths are the same spine, ALU -> `ex_fwd_data` -> EA adder -> L1 address
   -> `q_b`; neither `live` (which puts `flush` on the L1 write enable) nor
   the trace hold is on any of them.

   Defect six, then, and it is the same shape as the other five: nothing
   any unit bench for TRAP, CHK or the store path could see, because each
   checked its own instruction's result, and the defect is in the cycle
   between one instruction and the next.

   ### Milestone 77: the six-word frame for CHK, TRAPcc and zero divide -- found by integration4

   **The bench first.** `tb_ap040_pipe_integration4.v` is the third
   integration bench, and like the two before it, it was written across the
   seams the unit benches do not cross: supervisor to user with `MOVE D0,SR`,
   a MOVEM round trip on the USER stack, then from user mode TRAP #1, CHK,
   TRAPEQ, DIVU #0 and a privileged ORI to SR, and TRAP #0 to finish in
   supervisor. Every handler bumps D7 and ORs a bit into D6 per fact about
   its own frame -- format/vector word, stacked PC, the address field of a
   six-word frame, S clear in the stacked SR -- the way an OS reads a frame.
   The privilege handler advances the stacked PC past the ORI; the last one
   discards its frame with `LEA 8(A7),A7`. Ten checks: the thirteen frame
   facts as one mask, six handler entries, both stacks balanced (USP $0500,
   ISP $0600), the user code's and the finisher's markers, S set and T clear
   at the end.

   **On milestone-76 RTL it failed one check, and that check named the
   seam:** D6 = $1E13, bits 2, 3, 5, 6, 7 and 8 -- CHK, TRAPcc and zero
   divide pushed format $0 with no address field. The 68040 pushes the
   six-word format-$2 frame for those three (and trace), with the faulting
   instruction's address in the extra longword and the NEXT instruction in
   the PC field; `rtl/ap040/ap040_core.v` does exactly that
   (`exc(vector, 4'd2, pc, pc_i)`) and passes cputest with it. The pipe
   core's RTE pops whatever the format word says, so the wrong format was
   self-consistent: every unit bench for these exceptions returned
   correctly and balanced A7, and none of them looked at the frame.

   The fix is two lines in `ap040_ea_fetch.v`: `eac_is_fmt2` covers
   `eac_is_divzero`, `eac_is_chk_trap` and `eac_is_trapcc_trap` beside
   address error, and `exc_addr_field` is `eac_pc` for them (the odd
   JMP/JSR target, LSB cleared, stays for address error). Frame size, the
   format nibble, the third write beat and RTE's twelve-byte pop were all
   already keyed on `eac_is_fmt2` since milestones 17 and 54.

   | run | result |
   |---|---|
   | control: bench against milestone-76 RTL | FAIL -- D6 = $1E13 (bits 2, 3, 5, 6, 7, 8) |
   | address field = `eac_next_pc` | FAIL -- D6 = $1EB7: bits 3, 6, 8, the three address-field checks and nothing else |
   | TRAPcc dropped from the six-word list | FAIL -- D6 = $1F9F: bits 5, 6 |
   | zero divide dropped from the six-word list | FAIL -- D6 = $1E7F: bits 7, 8 |
   | full suite on milestone-77 RTL | 83/83 |

   Each mutation removes exactly the bits its check owns, so the mask is
   diagnostic rather than a pass/fail: the failing bit says which frame and
   which field.

   **Fit, same flow:** 5,199 ALMs (5,185 after milestone 75), 6,999
   combinational ALUTs (7,064), Fmax 41.59 MHz (40.68), setup slack at 25 ns
   +0.957 ns (+0.417), TNS 0. The 40 worst paths are the same spine as
   before -- ALU -> `ex_fwd_data` -> EA adder -> L1 address -> `q_b` -- and
   `eac_is_fmt2` now sits on all forty, since it selects the frame's beat
   address. The slack moved by half a nanosecond for a two-line change,
   which is the fitter's placement varying between runs, not a result; what
   the fit says is that the change cost nothing at 25 ns.

   Three integration benches have now found five pipeline defects that the
   unit benches around them did not: the lost redirect, the WB re-commit,
   the memory-source divide by zero, the CCR blind spot, and this frame
   format. The pattern in all five is the same -- a unit bench checks the
   instruction's own result, and the defect is in what the instruction
   leaves for the NEXT thing to read.

   ### Milestone 76: RTE format error

   RTE judges the format nibble when the pop's second dword arrives: $0
   pops eight bytes, $2 and $3 twelve (the FSM core's `S_RTE_FIN` accepts
   both), and anything else is a format error -- vector 14, a format-$0
   frame whose PC is the RTE INSTRUCTION ITSELF, stacked below the bad
   frame with A7 otherwise untouched. That PC convention is the point of
   the exception: the handler repairs the frame, returns, and the RTE runs
   again. `fmterr_now` turns the `ret_done` cycle into an exception entry
   -- `exc_writing` sits above `ret_done` in EA-fetch's output chain and
   wins the L1 address mux, so beat 0 of the frame goes out in that same
   cycle -- and is latched (`exc_pend_fmterr`) like the other data-derived
   faults. `eaf_is_fmterr` joins execute's `exc_reaching_ex`.

   Deliberate deviations, deferred with the mechanisms they need: $1
   (throwaway: load the SR, restart the pop on the next frame) and $7
   (access error: needs the BCU/MMU that would push it) take the format
   error rather than being popped as $0. This closes the original ISA list;
   what remains is the throwaway frame, trace, and the things that need
   hardware this core does not have.

   **The bench does the repair.** `tb_ap040_pipe_fmterr.v` builds a
   format-$B frame by hand, takes the error, and the handler reads its own
   frame and the bad one into D2..D6, patches the format word to $0 and
   returns -- so the RTE runs a second time and lands. Then a twelve-byte
   format-$3 frame is popped whole. Nine values are checked; the header
   says what each one proves.

   | run | result |
   |---|---|
   | control: milestone-75 RTL | FAIL -- D2..D6 zero, A7 $05FC: the $B frame popped as $0, and so did the $3 frame |
   | stack `next_pc` instead of the RTE's own address | FAIL -- D3 = $0420, A7 = $05F8: the repaired frame was never re-popped |
   | vector 14 -> 10 | FAIL -- D0..D6 and A7 ($05F0): handler never found |
   | latch removed | FAIL -- D0..D6 and SR: the push dies once the L1 address moves off the frame and the nibble reads as $0 |
   | $3 treated as short | FAIL -- D3 = $0442, D2 = $2700, A7 = $05FC: a second format error, from the format-$3 RTE |
   | parked-pop reset in `exc_vec_done` removed | **passes** -- the flush EX raises for every exception resets `ret_ph`/`ret_pending` a cycle later, before anything can consume them; kept as the sequencer's own exit and noted as unobservable in the RTL |
   | full suite on milestone-76 RTL | 82/82 |

   **A claim in the bench header was wrong, and the control corrected it.**
   The first draft said part 2 would pass on milestone-75 RTL. It does
   not: that RTL knew $2 alone as the long frame, so the $3 frame popped as
   eight bytes and A7 ended $05FC. The header now says so. Same lesson as
   milestones 73 and 74: the control is written down after it is run, not
   before.

   ### Milestone 75: the ROX count without a divider

   `nx = n % (nbits + 6'd1)` -- the ROX rotates take their count mod
   size+1 -- became `mod_np1(n)`: at most three conditional subtracts of a
   constant (36/18/9, 34/17, or 33), selected by size. Nothing else in the
   ALU changed. It is the first divergence between `ap040_pipe_alu.v` and
   the FSM core's `rtl/ap040/ap040_alu.v` beyond MULU/MULS; the FSM copy
   keeps its `%`, because that core's critical path is the SDRAM clock and
   a change there costs a Minimig fit and a cputest run to prove nothing.

   **The bench is an equivalence check, so its control passes.**
   `tb_ap040_pipe_alu_equiv.v` instantiates both ALUs and compares result
   and all five flags: every shift/rotate op x 3 sizes x all 64 counts x
   both X-in values x 16 operand patterns (49,152 checks), then 512 random
   vectors for every shared op and size (67,584 more). The reference is the
   copy that passes 3,797/3,801 cputest slices. On the previous commit the
   two files are the same logic, so the bench passes there as well -- that
   run shows the harness compares the right two modules rather than a
   shared default. `run_pipe_verilator.py` gives this bench the two ALU
   files and both include directories; the shared macros have identical
   values in both defs files, so nothing is redefined.

   | run | result |
   |---|---|
   | control: milestone-74 RTL | passes, 116,736 checks (identical ALUs) |
   | byte: drop the `>= 36` step | FAIL, first at ROXL.B count 37 |
   | word: `>= 17` -> `> 17` | **passes** -- an equivalent mutant, see below |
   | long: `n & 31` for `n mod 33` | FAIL, first at ROXL.L count 32 |
   | full suite on milestone-75 RTL | 81/81 |

   The word mutation leaves `nx = 17` where the reference has 0, and the
   bench is right to accept it: rotating a (size+1)-bit container by
   size+1 is the identity, so the two counts give the same result and the
   same flags. The same identity is why the byte mutation first fails at
   count 37, not 36 -- 36 reduces to 9 under the mutant, and a 9-bit
   rotate by 9 is a no-op. A mutation that changes a value without
   changing behaviour is not a bench blind spot; it is a fact about the
   operation, and recorded as one.

   **The fit, same flow as below:**

   | | ms 74 | ms 75 |
   |---|---:|---:|
   | ALMs | 5,190 | 5,185 |
   | ALU ALUTs | 2,429 | 2,256 |
   | combinational ALUTs, total | 7,213 | 7,064 |
   | `lpm_divide` entities | 1 | 0 |
   | Fmax, slow 1100 mV 100 C | 37.48 MHz | 40.68 MHz |
   | setup slack at 25 ns | -1.680 ns, TNS -84.0 | +0.417 ns, TNS 0 |

   The 40 worst paths are still one shape, the same one minus the divider:
   `eaf_shcnt`/`eaf_size` -> the ALU's count arithmetic and barrel -> the
   result mux -> `ex_fwd_data` -> `an_base` -> `Add2` -> `ea_target` -> the
   compare that feeds `eac_is_exc` -> `l1_addr_b` -> `wbuf_hits_read` ->
   `q_b`. 36 of the 40 cross the EX->EA-fetch forward and 39 the
   write-buffer merge. Worst data delay 23.956 ns against 26.56 before; the
   ALU segment is 8.9 ns where it was 12.4.

   So the standalone core closes 40 MHz with 0.4 ns to spare, on this die,
   with virtual pins and the L1 as flops at `L1_AW=4`. That is not the
   Minimig fit -- the CPU clock there is 35.234 ns, so the margin that
   matters is ten times larger -- and it is not the shape the L1 would
   have integrated. What the milestone settles is narrower: the divider was
   the whole shortfall, and the forward into address generation (item 2
   below) is now the critical shape with about 0.4 ns of headroom at
   25 ns. Any further ALU or EA-fetch depth lands on it first.

   ### Fit after milestone 74: 5,190 ALMs, 37.5 MHz, and one path shape

   The last standalone fit was at milestone 27 (section 5b). Forty-seven
   milestones later, `tests/ap040/pipe_synth/run.sh` reproduces that flow
   from the repo -- Cyclone V 5CSEBA6U23I7, Quartus 17.0, `L1_AW=4`, all
   ports virtual, one 25 ns clock, which is the 40 MHz target (the Minimig
   CPU clock is 35.234 ns) -- and prints the four numbers the plan tracks.
   The fit was run twice, by hand and by the script, with identical
   results.

   | | ms 27 | ms 74 |
   |---|---:|---:|
   | ALMs | 3,391 | 5,190 |
   | ALU ALUTs | 2,034 | 2,429 |
   | combinational ALUTs, total | -- | 7,213 |
   | registers | -- | 2,211 |
   | DSP blocks | -- | 2 |
   | Fmax, slow 1100 mV 100 C | -- | 37.48 MHz |
   | setup slack at 25 ns | -- | -1.680 ns, TNS -84.0 |

   Per entity, ALMs: execute 1,870 (the ALU 1,616 of it), EA-fetch 1,310,
   L1 604, register file 459, decode 378, instruction fetch 81, EA-calc 52.
   Every `AP040_ALU_*` code in the defs file is now selected somewhere in
   `ap040_decode.v` (35, MULU/MULS included). The core grew by half while
   the ALU grew by a fifth: the growth since milestone 27 is sequencers
   and addressing, not operations.

   **The 40 worst setup paths are one path.** `report_timing -npaths 40`
   returns forty paths, and every one of them crosses, in order: an
   EA-fetch output register (`eaf_size`, `eaf_operand_*`) -> the ALU's
   shift/rotate logic, including the `lpm_divide` that
   `nx = n % (nbits + 6'd1)` at `ap040_pipe_alu.v:308` infers -> the result
   mux -> `ex_fwd_data` -> EA-fetch's forwarded-An base (`an_base`) -> the
   EA adder (`Add2`) -> `l1_addr_b` -> the L1 read mux -> `q_b`. Twenty of
   the forty also pass through the compare that feeds `eac_is_chk_trap`,
   nine through `eac_is_chk_trap` -> `exc_vec_addr` on the way to the L1
   address, fourteen through the write-buffer merge in the L1 read mux.
   The worst is 26.56 ns of data delay against 25 ns, split roughly:
   12.4 ns in the ALU (4.7 of it inside the divider cells), 10.5 ns from
   `ex_fwd_data` to the L1 address, 3.4 ns in the L1 read.

   So the shape is: EX's forwarding output is the ALU result, unregistered,
   and EA-fetch consumes it in the same cycle -- as an address base, as a
   CHK operand, and through the exception-vector select -- ahead of the L1
   read that the address selects. Three stages of logic in one period. Two
   things sit on it that need not:

   1. The `%`. `nbits + 1` is 9, 17 or 33 and `n` is six bits, so the
      reduction is a few constant compares, not a divider. The same line
      is `rtl/ap040/ap040_alu.v:271` in the FSM core (the two files are
      identical logic); it never showed there because that core's critical
      path is the SDRAM clock, not the CPU's.
   2. The forward itself. Forwarding EX's combinational result into
      EA-fetch's address adder is what lets `ADDQ #4,A0` / `MOVE.L (A0),D0`
      run back to back without a stall; the price is ALU + EA adder + L1
      read in one cycle. Registering that forward, or taking it from WB
      only the way `sr_resolved` takes the CCR, costs a cycle on every
      An-after-ALU dependency and buys the whole ALU depth back.

   Neither is done here. Item 1 is a local change whose test is the
   existing suite plus a re-fit; item 2 changes CPI and is a decision, not
   a fix. What the fit settles is the question it was run for: the core
   does not close 40 MHz as it stands, by 1.7 ns, on a single path shape
   with a named divider on it.

   ### Milestone 74: TRAPcc, and the CCR one stage too early

   TRAPcc traps to vector 7 if its condition holds; the `.W`/`.L` forms
   carry an immediate the hardware ignores. Mode 111 in the Scc/DBcc space
   was unclaimed. The condition is judged in EA-fetch -- where a frame can
   still be pushed -- by a mirror of EX's `cond_true`, decided once and
   latched.

   **The bench's first run found a bug older than TRAPcc.** A `TRAPNE`
   directly after a `CMPI` trapped with Z set: its frame push is logged
   before the `CMPI` retires. `sr_resolved` forwarded flags only from WB.
   The instruction in EX had computed new flags nobody one stage upstream
   could read. Bcc/Scc/DBcc never saw it -- they evaluate in EX. TRAPcc is
   the first EA-fetch consumer of the CCR, and **the exception frame's
   stacked SR has had the same one-cycle blind spot since it existed** (the
   same `sr_in` feeds `exc_sr_word`; it is now current too, though not
   separately tested).

   EX now exports its committing flags live (`ex_ccr_fwd_*`) and the core
   folds them into a SECOND SR view, `sr_resolved_ea`, that only EA-fetch
   reads. Folding it into the view EX itself reads (`ccr_in`) made
   `alu_flags` feed the ALU that produced them -- a loop that broke ADDX,
   the one op whose result depends on an input flag. TRAPcc's decision is
   also gated on `!stall_in`, so a divide in EX finishes before the
   condition is judged.

   **What the bench is sensitive to, measured** (helper mutations, run
   before this text was written):

   | mutation | result |
   |---|---|
   | control: milestone-73 RTL | 4 failures, D2 = 0 |
   | CCR forward removed from EA-fetch's view | D2 = 4: the spurious `TRAPNE` returns |
   | `!stall_in` gate removed | D2 = 5, A7 unbalanced: `TRAPVS` judged the stale V mid-divide |
   | vector 7 -> 6 | 4 failures: handler never found |
   | `.L` `next_pc` counted as one word | D2 = 2, D1 = 0: RTE returned into the immediate |
   | latch removed | **passes** -- unobservable here; the CCR is stable through every frame push in this program. Correct by argument, same standing as milestone 48's stall gate |

   Two of those rows exist only because a first attempt was wrong. The stall
   gate was unobservable until the bench gained a divide before a TRAPcc.
   And my first length mutation removed `is_trapcc_l` from `ext_pending`
   but not `held_is_long`, so `next_pc` stayed right and the stray word was
   flushed by the trap itself -- a mutation that changes half of a
   two-place invariant tests nothing. The half RTE actually consumes is
   `held_is_long`.

   ### Milestone 73: MOVEM $xxx.L -- the three-word gather, and MOVEM complete

   Mask plus a 32-bit address is the first THREE-word gather in this
   decoder. It cost less than expected: `disp_acc` shifts every gathered
   word in, so after the mask and the high address word it holds
   `{mask, addr_hi}`, the completing word is `addr_lo`, `gather_disp` is
   the whole address for free and the mask is `disp_acc[31:16]`.
   `ext_pending` was already two bits wide; 3 fits. `held_is_xlong` carries
   the length for `id_next_pc`.

   That is also why the mask now has its OWN field, `id_movem_mask`, for
   every MOVEM mode -- one word: the completing word itself; two: the word
   shifted in; three: two words back. Milestone 68's `{mask, displacement}`
   packing in `id_imm` only worked while no mode needed all 32 bits of
   `id_imm` for an address, and the five MOVEM benches guarded the change.

   **A claim in milestone 73's commit was wrong and is retracted here.** It
   said the `MOVEQ #$2A,D2` after the store checked the three-word length
   through `next_pc`. Mutating the xlong term of `id_next_pc` left the bench
   passing. A plain MOVEM's `id_next_pc` is never consumed: IF fetches
   linearly and the gather stalls it for exactly `ext_pending` cycles, so
   fetch lands after the third word whatever `next_pc` says; only an
   exception or a return would read it, and neither applies. The term is
   correct and unobservable -- the same standing as milestone 48's stall
   gate -- and is recorded as such.

   What guards the three-word gather was then established by mutation:
   shortening it to two words, or reading the mask from the wrong slot,
   each fails the address and round-trip checks (three failures), while D2
   stays `2A` under both. The lesson is the one from milestone 70 again,
   now for a length rather than a timing: **the mutation has to be run
   before the claim is written, not after.**

   **MOVEM is now complete**: `.W` and `.L`; `-(An)`, `(An)+`, `(An)`,
   `(d16,An)`, `(d16,PC)`, `$xxx.W`, `$xxx.L`.

   ### Milestone 72: MOVEM (d16,PC) and $xxx.W

   Both are control modes and both gather a second word after the mask, so
   they ride milestone 68's two-word MOVEM gather with two more carried
   properties, `held_movem_pcrel` and `held_movem_abs`. Decode plus a
   three-way base select at the sequencer's start.

   **The PC-relative base is PC+4, not PC+2.** Every other PC-relative mode
   takes the address of its extension word, PC+2. For MOVEM the mask word
   sits between the opcode and the displacement, so the displacement word
   -- and the base -- is at PC+4. It is the one place MOVEM's extra word
   changes an ADDRESS rather than just a gather length, and the bench puts
   the table where a PC+2 base reads one word early and gives `4E711111`.

   **`MOVEM $xxx.L` is not reached**: mask plus a 32-bit address is a
   THREE-word gather, and `ext_pending` is two bits wide with two as its
   maximum. It is the last MOVEM mode missing, and the only one that needs
   the gather machinery itself to grow.

   ### Milestone 71: JMP/JSR with indexed, PC-relative and absolute targets

   `JSR (d16,PC)`, `JSR $xxx.L`, `JMP (d8,PC,Xn)` and their counterparts.
   JMP/JSR reached only `(An)` and `(d16,An)` before. Every EA path they
   need already existed from LEA and PEA, so the indexed and PC-relative
   forms ride `held_is_jmp`/`held_is_jsr` with the `held_ea_*` properties,
   and the absolute forms ride `held_is_abs` with `held_abs_jmp` and
   `held_abs_jsr` beside `held_abs_lea` and `held_abs_push`. Decode only.
   `tb_ap040_pipe_jmpmodes.v` covers all six; on milestone-70 RTL none
   decode and D2 stays 0.

   **The control procedure failed again, one milestone after the rule was
   written.** The command was labelled "via checkout" and then ran
   `git checkout HEAD -- rtl/ap040_pipe/` FIRST -- discarding the
   uncommitted decode edit -- followed by a `stash push` on the now-clean
   path, which saved nothing, and a `stash pop` that pulled the August entry
   in for a second time. The RTL edit was recovered by re-running the same
   deterministic script; the suite had already passed on it before the
   checkout. Same recovery as before: `reset --hard`, remove the eight
   untracked files the pop drops, verify the stash list is unchanged.

   The rule, restated so it cannot be half-followed: **commit first, then
   `git checkout <rev> -- rtl/ap040_pipe/`, run, `git checkout HEAD --
   rtl/ap040_pipe/`.** `stash` does not appear in a control at all, and
   `checkout HEAD` is only ever the LAST step, after the run.

   ### How to run a control, and how not to

   For thirty milestones the control was `git stash push rtl/ap040_pipe/`,
   run, `git stash pop`. It worked only because there were always
   uncommitted RTL changes to stash. Run on a CLEAN tree -- as after
   milestone 69's commit -- `stash push` with a pathspec saves nothing, and
   `stash pop` then pops whatever is on top of the stash: here an August
   entry from the other worktree, into this one, with merge conflicts across
   `Minimig.sv`, `rtl/ap040/` and `tests/ap040/build/`. Recovered with
   `git reset --hard HEAD` (all work was committed) plus removal of the
   eight untracked files the pop dropped in; the stale entry itself is
   untouched, since a conflicting pop keeps it.

   **Use `git checkout <rev> -- rtl/ap040_pipe/` to run a control and
   `git checkout HEAD -- rtl/ap040_pipe/` to come back.** It works whether
   or not the tree is clean, and it touches nothing outside the path.

   ### Milestone 69: an integration bench, and the two things it found

   `tb_ap040_pipe_integration2.v` runs a whole subroutine the way compiled
   code would -- BSR, LINK, MOVEM save, a SUBQ/BNE loop over `(A0)+`, DIVU,
   MOVEM restore, UNLK, RTS -- with sentinels carried across the call. The
   first integration bench dated from milestone 33; thirty milestones had
   gone in since, each proved in isolation. On its first run this one failed,
   and neither cause was visible from any unit bench.

   **A lost redirect in IF.** EX's mispredict recovery is a ONE-cycle
   redirect, and IF only advanced `pc` when not stalled. A not-taken
   loop-closing BNE had a memory load as its speculatively-fetched
   successor; that load's `mem_issue` stalled the front end for exactly the
   recovery cycle. `l1_addr_a` saw the recovery address, `pc` did not, and
   the branch re-executed from stale state forever. IF now advances on a
   flushing redirect regardless of stall -- and the L1's port-A enable had
   to follow, or `if_pc` and `if_opcode` skew apart: the first attempt had
   decode seeing the BNE's own opcode at the recovery PC.

   **WB re-committing during an EX stall.** The trace showed one MOVE
   retiring five times behind the divide. `commit_reg` was `exe_valid &&
   exe_writes_reg`, ungated: EX's outputs hold correctly while EX stalls,
   but WB committed them every cycle. It was HARMLESS, because every commit
   writes a value already registered in EX and rewriting it changes nothing
   -- which is also why no value check could ever have seen it. It is gated
   anyway (`exe_fresh`, high the cycle after EX writes), because that
   idempotence belongs to what is committed today, not to the commit path.

   Verifying the gate needed a COUNT, so the core grew `dbg_commits`. With
   the gate removed `tb_ap040_pipe_divstall.v` reports 38 commits for 5
   register-writing instructions, and `integration2` still passes -- the
   two together being the evidence. The stash-based control for that bench
   is vacuous (`PINNOTFOUND`: the port did not exist before), which is why
   the gate was controlled by mutation on the current RTL instead.

   **Bench layout rule, twice over:** `PROG_WORDS` is an issue budget, not
   an address bound. With the subroutine adjacent to main, trailing NOPs
   walked into a second LINK; with the array adjacent, they EXECUTED it --
   `0000 000A` is `ORI.B #$0A,D0`, and four data words OR'd `$3F` into D0.
   Anything a program does not jump over goes where a program cannot walk
   into it.

   ### Milestone 68: MOVEM's control modes

   `(An)` and `(d16,An)`, both directions. They differ from the
   autoincrement modes in THREE ways at once, and all three belong to the
   MODE rather than to the direction:

   - no register writeback at all
   - the address walks UPWARD, even for a store
   - the mask is numbered bit 0 = D0, even for a store

   Only the predecrement store reverses the numbering and walks downward.
   **Milestone 50 tied both behaviours to "is a store"**, which was
   indistinguishable from the truth while `-(An)` was the only store mode
   that existed -- a correct implementation of an incomplete rule. The
   sequencer now carries `mvm_down` and `mvm_wb` as properties of the mode.

   Reverting either rule breaks the control-mode bench and leaves the
   autoincrement one passing, so the old bench could not have caught it.

   `(d16,An)` also gathers a second word after the mask, so `id_imm` now
   carries `{mask, displacement}` for every MOVEM, with the displacement
   zero where the mode has none.

   Still not reached: the absolute and PC-relative MOVEM modes.

   ### Milestone 67: MOVEM.W

   `ir[6]` is the SIZE, so the shapes had pinned it and only the Long forms
   decoded. Widening them is most of the decode work; the behaviour worth
   testing is on the LOAD side, where MOVEM.W **sign-extends** each word
   into the whole 32-bit register rather than preserving the upper half.

   `$8001` is negative as a word and the destination starts as `AAAA8001`,
   so the three candidate answers are all distinct -- `FFFF8001`
   sign-extended, `00008001` zero-extended, `AAAA8001` upper half kept.
   Verified by zero-extending, which gives exactly `00008001`.

   Two things the bench had to be careful about:

   - A7 returning to where it started does NOT prove the step was two: two
     registers at four bytes each also balance. What proves it is the two
     words landing in ADJACENT slots.
   - The MOVEM.**L** bench is untouched by the sign-extension mutation, so
     the Word form needed its own bench -- the same coverage-per-mode point
     milestone 65 turned into a rule.

   ### Milestone 66: auditing every operand_a use

   Three milestones in a row turned on one root cause, so it was worth
   stating precisely and checking exhaustively rather than waiting to trip
   over it again:

   > In `ap040_ea_fetch.v`, `operand_a` is the ADDRESS for a memory source,
   > not the data. Any logic there that treats it as a VALUE is wrong for
   > every memory-mode form of its instruction.

   All twenty-odd uses were classified. The result is that **only the two
   already fixed were wrong** -- the divide's divisor and CHK's bound. Every
   other use wants the address or belongs to an instruction that is not a
   memory source:

   - `an_base`, `ea_base`, `mvm_addr`, `ret_addr` and UNLK's `operand_a + 4`
     all want the address, which is what they get.
   - `st_dat` and LINK's pushed value belong to instructions with
     `is_mem_src = 0`, so `operand_a` is a register value there. There is no
     memory-to-memory MOVE to complicate it.
   - MOVEM's use is under a `raddr_a` override, so it reads the register the
     beat is storing.
   - The plain branch's `sxt_w_of(operand_a)` is only reached by
     non-memory-source instructions; the memory path sign-extends inside
     `mem_lane` instead.

   The audit raised one SUSPECTED deviation, which was then measured rather
   than argued: does a trapping `CHK (A0)+` keep its postincrement? The
   68040 completes the EA before the comparison, so it must. **It does** --
   the second write port survives the exception path, `eaf_writes_an` being
   set from `an_wr_any` in the exception branch as well as the ordinary
   one. `tb_ap040_pipe_chkpi.v` keeps it as a regression check, because
   nothing else in the suite pins that interaction down.

   ### Adding a gather kind: the lists it must join

   Milestone 62 needed two debug cycles, both from the same cause, and
   between them they enumerate the trap. A new kind on the shared gather
   must be added to **every list that is written as a set of negations or an
   enumeration**, not just to the ones its own feature seems to touch:

   - `id_is_branch` -- a list of `!held_is_*`. A kind missing from it is
     decoded as a BRANCH. The first `ANDI #x,SR` redirected the pipeline.
   - `redirect_from_gather` -- the same list again, separately maintained.
   - `id_imm`'s `gather_disp` enumeration. A kind missing from it gets
     `id_imm = 0`, so `ANDI #$F8FF,SR` computed `SR & 0`, cleared the
     supervisor bit and took a privilege violation.

   Both symptoms were plausible wrong behaviour rather than crashes, and
   neither was visible from the feature's own code. Check the three lists
   when adding the twelfth kind.

   ### A hazard worth naming

   `ap040_ea_fetch.v`'s output block is one `always` with several branches,
   each assigning most of the `eaf_*` set. Adding a flag to it by blanket
   string-replace -- appending `<flag> <= 1'b0;` next to an existing
   `<= 1'b0;` line -- silently lands a SECOND assignment inside the branch
   that also sets the flag for real, and Verilog's last-write-wins makes the
   flag permanently zero. It cost a debug cycle in milestone 48
   (`eaf_is_rmw`, no store ever issued) and again in 49 (`eaf_is_link`, A7
   took `operand_a`), and milestone 34 had the same shape.

   Both times the symptom was a plausible wrong VALUE rather than a crash,
   because the fallback path is a real one. When adding the next flag here,
   assign it in each branch explicitly and grep the branch order before
   running anything.
4. **MMU and cache integration**, once enough of the integer ISA exists that
   testing them against real address translation is meaningful. Reuse the
   architectural requirements from `rtl_old/ap040_mmu.v`/`ap040_cache.v`
   directly -- don't re-derive the 040 table-walk/ATC/TTR rules from the
   manual a second time.
5. **FPU integration**, gated on the above the same way `rtl_old` staged it
   (LC040 trap-only mode before any hardware arithmetic).
7. **Superscalar/dual-issue** is an explicit stretch goal, not a
   prerequisite for anything above -- 68060-class pairing rules, do it last,
   and only if single-issue CPI is still the bottleneck once the ISA is
   complete.

Performance-tuning work that depends on a specific FPGA target (clock
domain, ALM/DSP budget, Quartus fit) is out of scope for this repo entirely
-- it belongs in whichever downstream project integrates this core, the same
way `rtl_old`'s own Minimig-specific timing/fit history lived in that
project's tree, not here.

## 7. Test infrastructure

- `tb/run_pipe_tests.sh` -- the active suite. Needs only `iverilog`
  (`brew install icarus-verilog`; nothing else). Every `tb_ap040_pipe_*.v`
  bench pokes its program directly into `ap040_inst_fetch.v`'s ROM at time 0
  (`dut.u_if.rom[N] = ...`) -- no assembler, no host project.
- `tb/run_tests.sh` -- the `rtl_old` reference suite. Needs `iverilog` +
  `vasmm68k_mot`; see section 3 for the toolchain gap.
- Both suites are self-contained: no host-project sources, so a failure is
  the CPU's, not an integration artifact (a downstream project like the one
  this core is designed to be pulled into adds its own co-simulation
  benches against real chipset/memory-controller RTL; those don't belong
  here).
