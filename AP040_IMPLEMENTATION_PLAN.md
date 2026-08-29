# AP040 Implementation Plan

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
3. **Byte/word-sized ALU ops and MOVE** (current pipeline is Long-only
   throughout `ap040_pipe_alu.v`'s size port is already there and unused).
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
