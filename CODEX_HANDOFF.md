# Handoff: AP68040 pipeline integration

This file's previous contents were a hardware-debugging log (sdram_ctrl,
ram_cs_guard, MiSTer board access) for the Minimig-AGA_MiSTer monorepo this
core was extracted from -- none of those files (`Minimig.sv`, `sdram_ctrl.v`,
`ram_cs_guard.v`, ...) exist in this repo, and that investigation has no
bearing on the standalone core. That history is still real and still lives
in the original project's tree/git history if it's ever needed again; it
just doesn't belong in this repo's handoff doc. Replaced with a handoff for
the actual work here.

For the full architecture spec, current milestone status, and remaining
scope, see `AP040_IMPLEMENTATION_PLAN.md` -- that's the document to keep
current going forward. This file is for short, dated session-to-session
continuity notes.

## 2026-08-26: repo extraction cleanup + milestone 8 (DBcc)

The pipeline branch was moved into this new standalone repo
(`rtl_old/` = the working sequential core, `rtl/` = the in-progress pipeline,
same layout `ap040_pipe_core.v`'s header already documented). Two things
were actually broken, both environmental, not RTL:

- **`iverilog` wasn't installed on this machine at all** -- every
  `tb_ap040_pipe_*.v` bench was correctly written and would have passed the
  whole time; there was nothing to fix in the RTL or the tests themselves.
  Fixed with `brew install icarus-verilog` (13.0). No test file changed.
- **`tb/run_tests.sh` (the `rtl_old` reference suite) still pointed
  `RTL=../rtl`**, which is now the pipeline directory and doesn't contain
  `ap040_core.v`/`ap040_tg68k_compat.v`/etc. Repointed to `RTL=../rtl_old`.
  That suite's assembly-based legs also need `vasmm68k_mot` (vbcc), which
  isn't in Homebrew and wasn't installed here -- confirmed the script now
  fails at the (expected, documented) assembly step rather than the wrong
  place, and confirmed the six non-assembly benches (reset, double fault,
  walker CDC, bus16 gap, bus timeout, cache snoop) pass standalone against
  `rtl_old` with just `iverilog`.
- Added `tb/run_pipe_tests.sh` so the pipeline suite has a real runner
  instead of being invoked by hand per-file (which is how the milestone-1
  through 7 testbenches had been getting run/verified up to this point).

Then continued the actual integration work: milestone 8, **DBcc
Dn,\<label\>**, following the plan's own note that it was the natural next
target (its prerequisite, the multi-word decode gather, already existed
from milestone 7). Verified bit-for-bit against `rtl_old/ap040_core.v`'s
`S_DBCC1`/`S_DBCC2` (lines ~3040-3059) and its `0x5` ADDQ/SUBQ/Scc/DBcc
decode group (lines ~5415-5421). Summary (full reasoning is in
`ap040_execute.v`/`ap040_decode.v`'s header comments, cited by line number
the same way every prior milestone's has been):

- Reuses the Bcc.W gather mechanism as-is (DBcc is always exactly one
  extension word) and the existing `ex_mispredict`/`ex_recovery_pc` wiring
  completely unchanged -- both of DBcc's "don't branch" outcomes (condition
  true, or the decremented counter hit `$FFFF`) are, from the front end's
  point of view, identical to a not-taken Bcc.
- New mechanism: DBcc is the first instruction whose register write depends
  on a **runtime** value, not a static decode-time bit (`writes_reg_resolved`
  in `ap040_execute.v`, gated on `!cond_result`). Three independent
  mutations (disable the dynamic gate entirely, drop DBcc from the
  mispredict condition, remove the cond_result gate but keep the branch
  logic) were each confirmed to break `tb_ap040_pipe_dbcc.v` in a
  distinguishable way before trusting the test.
- Explicitly deferred, matching the same discipline TRAPcc's deferral used
  in milestone 6: the real 68040 checks the branch target's parity *before*
  the condition (an odd target always faults, even on an iteration that
  wouldn't have branched -- `rtl_old`'s own S_DBCC1 comment cites cputest
  68040_ae DBcc.W for this). That needs exception delivery, which the
  pipeline doesn't have yet; not implemented rather than guessed at.
- Compressed `AP040_IMPLEMENTATION_PLAN.md` from a 1380-line narrative log
  (much of it Minimig-fitting/performance-tuning detail that doesn't apply
  to this standalone repo) into a ~200-line current-state-plus-roadmap
  document; the detailed per-milestone reasoning it used to duplicate is
  already anchored in the RTL file header comments with line citations, so
  nothing was actually lost, just de-duplicated.

Next candidate per the plan: the first memory-referencing instruction
(suggest `MOVE.L (An),Dn`) -- see `AP040_IMPLEMENTATION_PLAN.md` section 6.
That's the actual biggest remaining gap; everything implemented through
milestone 8 is register-direct/immediate only.

## 2026-08-27: CRITIQUE.md review + milestone 9a (unified L1)

The user pasted an external LLM-generated `CRITIQUE.md` (68040-dev-review
style) and asked about two things it raised: CCR timing, and whether to
build a unified Von Neumann L1 (dual-port BRAM, shared I/D) ahead of the
first memory instruction, prefilled directly by testbenches the way IF's ROM
already was.

**CRITIQUE.md accuracy notes** (for whoever reads it next -- it's still in
the repo as the user's own artifact, not corrected in place):
- Its "SCCR, DCCR: two separate CCRs for supervisor/data modes" row is
  fabricated -- no such registers exist on the MC68040. There is exactly one
  5-bit CCR (X/N/Z/V/C), the low byte of the 16-bit SR.
- Its CCR-forwarding recommendation ("shift the write-back of ccr to the ALU
  stage instead of WB") is already effectively true: `ccr_resolved` in
  `ap040_pipe_core.v` write-through-forwards the EX-computed flags to a
  same-cycle consumer, the identical mechanism the regfile itself uses for
  GPRs. This was mutation-tested in milestone 5 (the "immediately adjacent
  flag-setter" case in `tb_ap040_pipe_bcc.v`) specifically to prove the
  tightest-gap case works. No change was needed.
- Its "SR register... not modeled" row is a real, correctly-identified gap
  (S/M/T0/T1/IPL don't exist, only the 5-bit CCR does), but it's not urgent
  in isolation -- it's naturally sequenced with exception delivery (section
  6 item 3), since privilege/trace bits are meaningless without anywhere to
  trap to.
- Everything else in it is a reasonable, correctly-scoped observation about
  a work-in-progress core (single-word fetch, no addressing modes yet, no
  cache/stall logic) -- it's describing this pipeline accurately at the
  stage it caught it at, not raising alarms about actual bugs.

**Unified L1 decision:** agreed with the user's proposal and built the first
slice -- see `AP040_IMPLEMENTATION_PLAN.md` section 5a for the full
reasoning (coherency-for-free vs. the CINV/CPUSH semantic gap it creates,
why CACR's independent I/D enables aren't a blocker, why this was a
zero-risk drop-in for IF's timing). Concretely: `ap040_pipe_l1.v` (a fork of
`rtl_old/primitives/dpram.v`'s port convention, true dual-port, NOP-filled
by default), `ap040_inst_fetch.v` now drives its address combinationally
instead of owning storage, port B sits reserved and tied off at
`ap040_pipe_core.v` waiting for the first memory instruction. All 9 existing
pipe tests pass with zero behavior change (only their hierarchical preload
path moved, `dut.u_if.rom[N]` -> `dut.u_l1.mem[N]`).

Left open, deliberately: whether port B's read (once milestone 9b actually
consumes it) is combinational or a genuine registered/stalled read -- this
is the first time any of this pipeline's `*_stall` wiring would do
something real, so it's worth a deliberate answer rather than a default.

## 2026-08-27 (later): milestone 9b, MOVE.L (An),Dn

User picked registered+stalled for L1 port B. Built it same session --
`ap040_ea_fetch.v` gained a two-state `mem_issue`/`mem_complete` FSM; needed
no new forwarding logic since An's value already flows through the existing
`operand_a` mux (decode just sets `eac_src_reg` to An's unified index, 8+n,
same 4-bit space every register-direct source already used). Full reasoning
and the two real bugs caught during development (a genuine RTL addressing-
convention mismatch between IF's and EA-fetch's L1 ports, and a testbench
NBA-race poking a register too early) are in
`AP040_IMPLEMENTATION_PLAN.md` section 5a -- worth reading in full before
touching `ap040_ea_fetch.v` again, especially the PC_RESET-relative
addressing convention, which is easy to silently break a second way.

`tb_ap040_pipe_move_mem.v` mutation-tested three ways (drop the stall,
swap the L1 word order, drop the PC_RESET base) -- all caught distinctly.
Full 10-file `run_pipe_tests.sh` green.

Next: an EA mode that needs real EA-calc arithmetic (`(d16,An)` is the
natural candidate -- reuses the existing multi-word decode gather) or
`(An)+`/`-(An)` (needs the pipeline's first register WRITE originating from
EA-calc/EA-fetch, not just EX) -- see `AP040_IMPLEMENTATION_PLAN.md`
section 6 item 1.

## 2026-08-27 (later still): milestone 10, MOVE.L (d16,An),Dn -- and a real bug

Did `(d16,An)` next, per the note above. The EA arithmetic itself was easy
(reused the existing gather machinery + `id_imm` as a general-purpose
offset field). What actually mattered: this milestone's test was the FIRST
to have a stall from one instruction's memory access overlap with a SECOND
instruction's gather still assembling its extension word -- a combination
milestone 9b's test never produced -- and that overlap exposed a real,
already-shipped bug: `ap040_pipe_l1.v` port A had no stall-awareness at
all, and would silently read one word PAST what `if_opcode` was supposed to
keep presenting, the moment any stall outlasted a single cycle. Fixed with
a proper `en_a` enable, matching `ap040_inst_fetch.v`'s own freeze
condition exactly. Full writeup, including WHY milestone 9b's own test
never caught it, is in `AP040_IMPLEMENTATION_PLAN.md` section 5a -- read it
before touching `ap040_pipe_l1.v` or adding a new stall source; the general
lesson (stalls must propagate to everything the frozen register
combinationally feeds, not just the register itself) will matter again.

All three new mechanisms this milestone touched (the redirect-suppression
guard, the `id_imm` zero-fix, the `en_a` fix) were mutation-tested
independently; all caught cleanly, and the `en_a` mutation reproduces the
exact original bug's symptom. Full 11-file `run_pipe_tests.sh` green.

Next: `(An)+`/`-(An)` (needs the pipeline's first EA-side register write)
or start on indexed/absolute/PC-relative modes -- see
`AP040_IMPLEMENTATION_PLAN.md` section 6 item 1.

## 2026-08-27 (later still): milestone 11, JMP (An) / JMP (d16,An)

User asked to pick up BSR/JSR/JMP. Sequenced JMP first: it reuses the EA
resolution `MOVE.L (An)/(d16,An),Dn` already built, wholesale, and needs
ZERO new memory-write machinery (unlike BSR/JSR, which both need this
pipeline's first actual write -- the stack push -- and a register side
effect, A7's decrement, that isn't the primary ALU destination). JMP's
only new idea: since decode has no literal displacement to speculate a
target with, `ap040_execute.v` treats JMP as an UNCONDITIONAL
misprediction once it resolves, reusing `ex_mispredict`/`ex_recovery_pc`/
`flush` completely unchanged -- the same wiring DBcc reused from Bcc back
in milestone 8. Zero new top-level redirect plumbing needed.

Two real, worth-remembering findings from this one -- full writeup in
`AP040_IMPLEMENTATION_PLAN.md`'s milestone-11 section, right after 5a:

1. **A genuine test-budget bug** (not RTL): `ap040_inst_fetch.v`'s
   `issued` counter counts every word fetched, INCLUDING ones a later
   misprediction flush discards. JMP never lets decode guess a real
   target, so IF races several words past every JMP before EX's
   unconditional correction arrives -- with two chained JMPs in one test,
   the usual `PROG_WORDS=10` silently ran out before the second JMP's
   real post-redirect target ever executed. Caught as a wrong register
   value, not a hang; fixed with `PROG_WORDS=30`.
2. **An honestly-reported non-finding**: of four mutations tried, only
   three caught anything. Dropping the `!held_is_jmp` redirect-suppression
   guard (same shape move-disp needed in milestone 10) left the test
   passing -- and that's correct, not a coverage gap to chase: unlike
   move-disp, which has NO EX-side correction at all (so a bad decode-time
   guess for it is permanent and unrecoverable), JMP's target is ALWAYS
   fixed by EX's unconditional misprediction regardless of what decode
   guessed, so an extra stray speculative redirect just wastes a few
   cycles of now-discarded fetching. Kept the guard anyway (consistent
   shape, avoids pointless work), but documented it as defensive rather
   than claiming mutation coverage that doesn't actually exist for it.

Full 12-file `run_pipe_tests.sh` green.

Next: `BSR`/`JSR` -- the actual next real lift, since both need a genuine
first for this pipeline (a memory write, plus a non-ALU register side
effect on the same instruction). See `AP040_IMPLEMENTATION_PLAN.md`
section 6 item 2 for the concrete shape (an `ap040_pipe_l1.v` port-B write
path, `wren_b`/`data_b` already reserved but undriven; a second write
source into the regfile alongside `commit_reg`). JSR becomes close to free
once BSR's write mechanism exists -- it's JMP's EA resolution (already
built) plus BSR's push (about to be built).

## 2026-08-27 (later still): milestone 12, L1 posted write buffer

User asked, before wiring BSR's push, to make sure the write PATH itself
was right first: a real buffer (1 entry, nothing wider than 32-bit is
needed), so a store doesn't force the pipeline to stall hard if port B is
otherwise busy. Built it into `ap040_pipe_l1.v` -- `wren_b`/`data_b` now
POST into a real 1-entry buffer (`wbuf_valid`/`wbuf_addr`/`wbuf_data`),
drained into `mem[]` with priority over accepting a new post, plus
read-after-write forwarding so a load can't see stale data behind an
undrained store. `wr_busy` exposes backpressure; bounded to two edges
worst-case (drain, then accept), one edge once already observed clear --
full reasoning, including why the "one cycle" claim in an early draft was
wrong by exactly one edge, is in `AP040_IMPLEMENTATION_PLAN.md`'s
milestone-12 writeup, right after 11's.

No pipeline instruction drives `wren_b` yet (BSR still needs to be wired
up to actually USE this), so this got its own standalone testbench,
`tb_ap040_pipe_l1_wbuf.v`, instantiating `ap040_pipe_l1.v` directly rather
than the full `ap040_pipe_core.v` -- the first test in this suite to do
that; `tb/run_pipe_tests.sh` special-cases its (much smaller) source list
accordingly.

Three real bugs surfaced, all documented in detail in the plan doc since
the next session needs the full reasoning, not just the summary:
1. **A genuine RTL bug**: `wbuf_valid` had no reset (this module never
   needed one before), so it read `X` at time 0 and stayed `X` forever
   with nothing to write it -- which poisoned `q_b` via the forwarding
   ternary on reads that had NOTHING to do with any write, breaking two
   already-passing tests the moment this milestone's code landed. Fixed
   by giving the module a real `nreset`, gating `wbuf_valid` only (`q_a`/
   `q_b`/`mem[]` keep the original no-reset treatment).
2. **A testbench race** (active-region vs. NBA-region, reading `wr_busy`
   or asserting new stimulus in the same instant a process resumes from
   `@(posedge clk)`) that passed one check by scheduling luck while an
   identically-shaped later check failed -- fixed with `#1` after every
   edge, applied uniformly rather than only where a failure surfaced.
3. **A testbench message-truncation bug**: `check1`/`check32` used
   `[255:0]` for the failure message and silently truncated anything over
   32 characters to its TAIL, which actively misled the first round of
   debugging (garbled but plausible-looking wrong text, not an obvious
   truncation). Fixed with SystemVerilog's `string` type.

All three real RTL mechanisms (forwarding, drain-priority, the reset fix)
mutation-tested independently and caught cleanly. Full 13-file
`run_pipe_tests.sh` green.

Next: actually wire BSR's push through this new path -- EA-fetch/EX needs
to drive `wren_b`/`address_b`(the L1 word index for `A7-4`)/`data_b`(the
return address) respecting `wr_busy`, plus a second, non-ALU register-write
source into the regfile for A7's decrement (alongside `commit_reg`'s
existing one). See `AP040_IMPLEMENTATION_PLAN.md` section 6 item 2.

## 2026-08-27 (later still): milestone 13, BSR + JSR

User: "Okie dokie -- let's do BSR/JSR!" Both landed together, same set of
edits across `ap040_decode.v`/`ap040_ea_calc.v`/`ap040_ea_fetch.v`/
`ap040_execute.v`/`ap040_pipe_core.v` -- turned out to need far less new
machinery than the "next: a second non-ALU register-write source" note
above predicted:

- **BSR reuses Bcc's speculative decode-time redirect verbatim** --
  unconditionally taken, so it's always correct, and EX has nothing left
  to correct (`ex_mispredict` explicitly excludes `eaf_is_bsr`). Kept as
  its own `id_is_bsr` flag rather than folded into `id_is_branch`: BSR's
  condition-code field bits literally encode "F" (always false), which
  would make `cond_true()` wrongly fire a misprediction if misclassified.
- **JSR reuses JMP's EA resolution and gather unchanged** -- one opcode
  bit different from JMP, excluded from the speculative redirect the same
  way `held_is_jmp` is (its target isn't known until EA resolves), rides
  `ex_mispredict`/`ex_recovery_pc` unconditionally like JMP already does.
- **No second regfile write port needed.** Decode sets `id_dest_reg = A7`
  for both BSR and JSR, so EA-fetch's EXISTING `operand_b` mux (port B,
  already EX-forwarded) resolves A7's current value for free.
  `push_addr = operand_b - 4` is computed once in EA-fetch, reused both
  as the L1 write address and as the eventual `commit_reg` value -- the
  same write port every other instruction already uses, just with a
  non-ALU value substituted into `combined_result` for these two ops.
- **The write-side stall is simpler than the read side's `mem_issue`/
  `mem_complete` FSM** -- no "wait for response" phase, just "wait for
  the port," so one term (`wr_stall`) bubbles EX with no pending latch.
- **No real RTL bugs surfaced this milestone** -- milestone 12 having
  front-loaded the write buffer's own bugs before anything depended on
  them gets the credit. Both testbenches (`tb_ap040_pipe_bsr.v`,
  `tb_ap040_pipe_jsr.v`) passed on first real compile/run.
- Mutation testing: BSR's redirect-byte guard, redirect-gather guard, and
  push-data-source (`eac_next_pc` vs `eac_pc`) all caught cleanly. JSR's
  `ex_mispredict`/`ex_recovery_pc` participation both caught cleanly.
  JSR's `redirect_from_gather` guard mutation was **not caught** -- an
  honest non-finding, same shape and same reason as JMP's in milestone
  11 (EX's unconditional correction masks a bad decode-time guess before
  it's observable). Full detail, including exact mutation diffs and
  reasoning, is in `AP040_IMPLEMENTATION_PLAN.md`'s milestone-13 writeup.

Full 15-file `run_pipe_tests.sh` green (13 pipe-core tests + BSR + JSR +
the standalone `l1_wbuf` test).

Next: `RTS` -- `(A7)+` into the PC, the natural pop to BSR/JSR's push, and
the first instruction to actually exercise the write buffer's
read-after-write forwarding path (built in milestone 12, unreachable by
anything until now). See `AP040_IMPLEMENTATION_PLAN.md` section 6 item 2.

## 2026-08-27 (later still): milestone 14, exception entry (illegal, TRAP #n)

User: exception handling is next -- check different frame sizes, verify the
stack data is right, cover the common vector types. Scoped to what the
current ISA and rtl_old's own (verified, not guessed) exception table
actually support cleanly: **illegal instruction (vector 4)** and **TRAP #n
(vectors 32-47)**, both format $0 (4-word frame). Format $2 (address error
on odd JMP/JSR targets -- real, mode-dependent bit-exact quirks in
`ap040_core.v`'s S_JMP1/S_JSR1) is split into its own follow-up milestone
rather than guessed at alongside this one.

- Reuses almost everything BSR/JSR/JMP already built: decode points
  `id_dest_reg` at A7 for both new vectors (same trick, no new regfile
  port), the frame's contents are all computed off fields already
  threaded (`eac_pc`/`eac_next_pc`/`eac_imm`, plus a newly-threaded
  `ccr_in` for the SR word), and the eventual handler-address redirect
  reuses `ex_mispredict`/`ex_recovery_pc` exactly like JMP/JSR's
  unconditional misprediction path.
- What's genuinely new: `ap040_ea_fetch.v` gained this pipeline's first
  multi-beat memory operation -- two posted writes (the frame) then a
  read (the vector table), sequenced by a 3-state register reusing the
  SAME accept-when-`!l1_wr_busy` retry idiom BSR/JSR's single write
  already established, one beat at a time.
- No real VBR register exists (no control registers/MOVEC yet) -- vector
  fetch treats vector*4 as an ABSOLUTE address fed through the same
  PC_RESET-relative conversion every other L1 access uses. This works
  out cleanly, not by luck: PC_RESET has been $400 in every testbench
  since milestone 1, exactly the byte size of a full 256-entry vector
  table, so the unsigned address wraparound lands correctly in the same
  L1 array -- verified arithmetically (Python) before relying on it.
- Illegal-vs-TRAP PC-field distinction verified against `ap040_core.v`:
  illegal stacks its OWN address (`go_illegal`'s `spc=pc_i`), TRAP stacks
  the FOLLOWING instruction's (its dispatch's `spc=pc`) -- a subroutine-
  call vs. can't-return-past-a-fault distinction, confirmed load-bearing
  by mutation testing, not assumed from the doc comment.
- A real semantic change, not just additive: every previously-
  unrecognized opcode used to be a harmless bubble (`id_unimpl`, never
  consumed downstream); it's now `id_is_illegal`, a real trap. Confirmed
  safe by inspection first -- no existing test's program uses an
  unrecognized opcode.
- No real RTL bugs surfaced (second milestone in a row with that
  record -- reusing thoroughly-debugged machinery, not inventing new
  mechanisms). Test passed on first real compile/run.
- The test deliberately chains through a real JMP (not just back-to-back
  like BSR/JSR): the illegal handler marks a register then JMPs back to
  resume the mainline program at the TRAP instruction, proving the
  exception's flush/redirect composes correctly with an ordinary,
  unrelated misprediction recovery right afterward. A7 is not reseeded
  between the two cases either, so TRAP's push is verified from a
  different base than the illegal case's.
- Mutation testing: removing `is_illegal` from A7's dest-reg selection
  (caught, with an instructive cascade through both frames); flipping
  illegal's PC-field ternary to TRAP's (caught, cleanly isolated to just
  the illegal frame); removing `eaf_is_illegal` from `ex_mispredict`
  while leaving `eaf_is_trap` untouched (caught, confirming illegal's
  redirect is independently load-bearing). All three restored and
  diff-confirmed clean. Full detail in `AP040_IMPLEMENTATION_PLAN.md`'s
  milestone-14 writeup.

Full 16-file `run_pipe_tests.sh` green (14 pipe-core tests + the new
exception test + the standalone `l1_wbuf` test).

Next: format $2 (address error on odd JMP/JSR targets) -- see
`AP040_IMPLEMENTATION_PLAN.md` section 6 item 2b for the exact
`ap040_core.v` call sites and their mode-dependent quirks (JMP's stacked
PC is `pc_i+2` regardless of gather width; JSR's is the target itself,
and JSR skips its push entirely on an odd target rather than attempting
one). After that: `RTS` (see the milestone-13 entry above), TRAPcc, and
the DBcc branch-target parity check are all queued behind this
exception-entry machinery now actually existing.

## 2026-08-27 (later still): milestone 15, supervisor state

User: audit supervisor-state completeness (VBR, the three stack pointers,
SFC, DFC, CACR should all be present) and prove mode switching plus
privileged-instruction faulting actually work, not just that the
registers exist.

- `ccr` (5 bits) became `sr` (16 bits), matching `AP040_SR_RESET`'s
  layout. `sr_s`/`sr_m` feeding `ap040_pipe_regfile.v`'s A7 bank --
  hardwired since that module was first written -- are finally real.
  Its `aux_we`/`aux_sel`/`aux_wdata` port, unused since that file's first
  version, got its first real driver, for MOVEC's USP/ISP/MSP targets.
- Four new control registers (VBR/SFC/DFC/CACR) live in
  `ap040_pipe_core.v`, MOVEC-writable/readable. The MMU registers are
  explicitly out of scope (per the user's own framing) -- an invalid
  MOVEC selector becomes illegal (vector 4), not silently
  accepted/dropped. CACR is a plain, behaviorally inert register (masked
  `& 32'h8000_8000` for bit-exact readback) -- matches CINV/CPUSH's
  already-documented "diminished capacity" on this unified-L1 substrate.
- MOVEC's read direction needed no new commit path: the selected control
  register's value is fed straight from `ap040_pipe_core.v` into
  `ap040_execute.v` (bypassing every pipeline stage in between -- live
  architectural state, not per-instruction data) and rides the SAME
  `combined_result`/`commit_reg` machinery every GPR-writer already uses.
- Privilege violation is this pipeline's first genuinely DYNAMIC
  exception trigger: illegal/TRAP are decode-time facts, but whether
  MOVEC/MOVE-to-SR faults depends on the live, forwarded S bit, which
  doesn't exist until `ap040_ea_fetch.v`. Folds into the exact same
  exception machinery illegal/TRAP built (vector 8, format $0).
- **Two real bugs found and fixed, not just decorated with tests**:
  (1) an exception's own frame push must ALWAYS land on a supervisor
  stack (ISP/MSP), never whichever bank is currently active -- a
  privilege violation taken WHILE ALREADY IN USER MODE would otherwise
  push its own frame onto USP. Confirmed wrong against `ap040_core.v`'s
  S_EXC0/S_EXC1 ordering before fixing. Fixed by reading `isp_in`/
  `msp_in` directly, bypassing port B/A7 for this one purpose -- which
  also let an earlier `eff_dest_reg` port-B-rerouting mechanism be
  removed as unnecessary, a net simplification. (2) `sr_resolved`'s
  write-through forward covered "committing this cycle" but not "still
  in EX, one stage ahead" -- exactly the gap `ex_fwd_*` exists to close
  for GPRs, newly relevant now that `ap040_ea_fetch.v` reads live SR. A
  BSR one instruction behind a MOVE-to-SR read A7 through the stale,
  pre-switch bank for one cycle. Found via a cycle-by-cycle trace (not
  guessed), fixed with `ex_sr_fwd_valid`/`ex_sr_fwd_data` (SR's own
  EX-live forward). Fixing it surfaced a genuine combinational-loop risk
  too -- resolved by having `ap040_ea_fetch.v` thread its own
  already-correct read down as `eaf_sr_snapshot` instead of
  `ap040_execute.v` re-reading a live value that would have depended on
  its own output.
- `tb_ap040_pipe_sup.v` chains four phases (MOVEC read/write for all
  seven control registers -> MOVE-to-SR drops to user mode -> an
  ordinary BSR in user mode proves USP banking -> a privileged MOVEC in
  user mode faults, proving the frame still lands on ISP, USP
  untouched) through one continuous program. Two testbench-only bugs
  fixed along the way (a MOVEQ sign-extension expectation, and a check
  asserted against the wrong point in program order) -- not RTL bugs.
- Mutation testing: removing `ex_sr_fwd_valid`'s priority term (caught,
  reproducing the original bug byte-for-byte); forcing `exc_sp_bank` to
  the naive active-bank read (caught); removing the S-bit gate from
  `eac_is_priv` (caught, total cascade); removing VBR from
  `movec_sel_valid` (caught, same cascade signature). All four restored
  and diff-confirmed clean.
- Deliberately not built: `MOVE from SR`, `MOVE An,USP`/`MOVE USP,An`
  (redundant given MOVEC + the new `dbg_sr` tap), and VBR is not yet
  consulted by the exception-entry sequencer itself (settable/readable,
  just not wired into the vector-fetch address yet).

Full 17-file `run_pipe_tests.sh` green (15 pipe-core tests + the new
supervisor-state test + the standalone `l1_wbuf` test).

## 2026-08-27 (later still): milestone 16, RTS, RTE

User: RTS/RTE next -- closes the biggest gap milestone 15 left open (entry
without a return path).

- RTS is deliberately not a new mechanism: it reuses `mem_issue`/
  `mem_complete` (MOVE.L (An),Dn's own FSM) verbatim -- a pop is
  structurally just a 32-bit read from (A7) with the register hardcoded
  instead of decoded from opcode bits. Only `mem_complete`'s
  `eaf_operand_b` needed a new ternary, carrying A7's post-pop value
  forward for the commit.
- RTE needed real new machinery: a privilege check (RTE joins MOVE-to-SR/
  MOVEC as a third dynamic-fault source, reusing `eac_is_priv`) gates a
  genuinely supervisor RTE into its own 2-beat READ sequencer -- the
  mirror image of the exception-entry sequencer's WRITE beats, reading
  back the exact two dwords a format-$0 push wrote. Format $0 is assumed
  unconditionally (this pipeline has never pushed anything else) -- a
  real FMTERR fallback is deliberately deferred, not overlooked.
- **A real architectural race, found by the test failing**: RTE's SR
  restore and its A7 restore commit on the exact same cycle. Routing the
  A7 write through the normal commit_reg/A7-bank-selected path meant
  milestone 15's own `sr_resolved` fix made the SR restore visible to
  that SAME cycle's bank selection -- RTE's own A7 write got banked
  through the NEW (post-restore) S bit instead of the OLD one active
  while the frame was actually being popped, silently landing the
  restored ISP/MSP value in USP whenever RTE returned to a different mode
  than it ran in. Fixed by routing RTE's A7 restore through the same
  direct-to-ISP/MSP path (`exe_writes_creg`, the aux port) the exception
  entry's own push already uses, bypassing the live bank entirely.
- `tb_ap040_pipe_rts_rte.v` chains two round trips: plain BSR/RTS in
  supervisor mode, then TRAP taken FROM user mode with RTE popping the
  frame back out -- proving the frame lands on ISP not USP even though
  the fault occurred in user mode (the exact scenario milestone 15's fix
  targeted but had never been exercised with a real RTE consuming the
  frame), PC and the full SR both restored exactly, and a third, post-RTE
  BSR proving USP banking still works afterward.
- Two testbench-construction bugs, not RTL: an early draft placed the
  BSR/RTS return point immediately before the subroutine's own body,
  so RTS's return fell straight through and re-entered the subroutine a
  second time (caught by tracing, not inspection); the program had no
  proper termination, so execution free-ran past the interesting part
  into uninitialized memory and tripped an unrelated exception that
  corrupted the final snapshot -- fixed with a `BRA.B <-2>` self-loop
  instead of NOP padding.
- Mutation testing: removing RTE's own commit path (caught); removing
  RTS's `+4` new-A7 arithmetic (caught, cascade); removing RTE's SR-
  restore data source -- **not caught on the first pass** (the wrong
  fallback value coincidentally also read S=0 for this program's
  addresses, and the test only checked bit 13) -- strengthened to assert
  the full, exactly-known SR value instead, which then caught it cleanly;
  forcing RTE's restore to always target USP instead of ISP/MSP (caught,
  cascade). All four restored and diff-confirmed clean.

Full 18-file `run_pipe_tests.sh` green (16 pipe-core tests + the new
RTS/RTE test + the standalone `l1_wbuf` test).

## 2026-08-27 (later still): milestone 17, address error (format $2)

User: "full steam ahead" -- continuing autonomously to the next item this
repo's own plan/handoff had already flagged as next: format $2, address
error on odd JMP/JSR targets.

- This pipeline's first format-$2 (6-word, 12-byte) frame -- every earlier
  exception (illegal, TRAP, priv) is format $0. Detection is entirely
  dynamic, at EA-fetch time against the live, forwarded `ea_target[0]`,
  same shape as milestone 15's privilege-violation check -- `ap040_decode.v`
  needed zero functional changes (banner updated, nothing else).
- JMP and JSR are genuinely different, verified bit-exact against
  `rtl_old`'s `S_JMP1`/`S_JSR1`, not assumed to share a shape: JMP's
  stacked PC field is the JMP instruction's own address + 2; JSR's is the
  odd target itself, raw and unrounded, because the fault is on the
  instruction fetch AT the target, not the call site. JSR's own stack
  push is skipped entirely for the odd-target case (`eac_is_push` now
  excludes `eac_is_jsr_odd`) -- no return address to protect for a call
  that never completes; ISP moves by exactly one 12-byte frame, never
  12+4. Both cases' extra address-field longword IS rounded down
  (`{ea_target[31:1],1'b0}`) -- deliberately NOT the same value as JSR's
  PC field.
- Mechanically just a third exception-entry write beat (`EXC_BEAT2`,
  format-$2-only, gated by new `eac_is_fmt2`) reusing the exact
  beat/vec-read/finalize shape illegal/TRAP/priv/RTE's push already
  established -- no new state-machine shape.
- **A real testbench-construction bug, caught by tracing, not
  inspection**: the first draft of `tb_ap040_pipe_addrerr.v` gave its
  JMP-odd handler an unconditional JMP back to the (also odd) JSR
  instruction, expecting a SEPARATE handler to catch the JSR fault --
  but address error is ONE vector (3) for both cases, so the JSR
  re-faulted through the SAME vector into the SAME handler, which jumped
  back to the JSR again: an infinite loop silently draining the stack 12
  bytes/pass. The failure symptom (`D6` never set, `ISP` at an
  unexplained value) gave no direct hint of a loop; a cycle-by-cycle
  trace of `eac_pc`/`eaf_pc`/`isp` (ISP decrementing by 12 every ~10
  cycles while PC kept bouncing to the same handler) made it unambiguous.
  Fixed by merging the two handlers into one, using D7 as a one-shot
  entry counter (`ADD.L D7,D7` to test-and-double it, `BNE.B` on the
  second entry) -- a real fix to the test's control flow.
- Mutation testing: removing `eac_is_jsr_odd`'s exclusion from
  `eac_is_push` (caught, cascade through D4/D5/frame/CCR); forcing
  `exc_frame_size` to always be format $0's 8 (caught, every format-$2
  field lands at the wrong offset); swapping JMP's/JSR's PC-field
  formulas (caught, isolated exactly to both frames' PC field); removing
  `exc_addr_field`'s LSB-clearing (caught, both address fields off by 1);
  breaking `EXC_BEAT2`'s case-statement advance so it's skipped entirely
  for format-$2 (caught, address-field beat never written). All five
  restored and diff-confirmed clean.
- Still deferred: RTE remains format-$0-only, and this is now a REAL gap
  (not hypothetical) since this pipeline can push format $2 as of this
  milestone -- an RTE returning from an address-error handler would
  misread the frame. No FMTERR fallback either. VBR still not consulted
  by vector fetch.

Full 19-file `run_pipe_tests.sh` green (17 pipe-core tests + the new
address-error test + the standalone `l1_wbuf` test).

Next: TRAPcc, the DBcc branch-target parity check, or RTE's format-$2 pop
support (now a real gap, not a moot one -- see above).
