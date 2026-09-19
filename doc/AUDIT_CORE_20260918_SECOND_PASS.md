# Second CPU audit: write-fault restart

Audited current RTL at `ec25690cd` on 2026-09-18, including the NOP fix
and the maintenance wait moved inside its priority branch. Two additional
high-priority correctness defects reproduce. They are independent of
posted stores and of enabling the caches. No production RTL was edited,
no commits were made by this audit, and no FPGA build or flash was run.

This pass reviewed integer execution/writeback, address-register rollback,
access-error frames, RTE, split page accesses, CAS2, fetch-queue handshakes,
and the previous synchronization fixes. It is not an exhaustive ISA audit.
The new tests require a recoverable write-protection fault. They do not
establish the cause of Elysium or Deformations failing without such faults.

## 1. P1: retrying an instruction consumes its own modified X flag

[ap040_core.v:2753](rtl/ap040/ap040_core.v#L2753) updates the CCR before
issuing an ALU result's memory write. The shift path does the same at
[line 2792](rtl/ap040/ap040_core.v#L2792), before `S_SHIFT_WB`.
When that write faults, [line 2938](rtl/ap040/ap040_core.v#L2938) saves the
already modified SR. The format-$7 frame's PC identifies the original
instruction ([line 1749](rtl/ap040/ap040_core.v#L1749)). RTE restores that
SR and re-executes the instruction; address-register rollback does not
restore its original CCR inputs.

Reproductions, each with input X=0, a readable but write-protected page,
and a handler that makes the page writable, executes PFLUSHA, and RTEs:

| Instruction | Initial operand | Correct result | After repaired fault |
|---|---:|---:|---:|
| `negx.l ($8000).l` | `$00000001` | `$FFFFFFFF` | **`$FFFFFFFE`** |
| `roxl.w ($8000).l` | `$8000` | `$0000` | **`$0001`** |

The handler observes original, unmodified operand memory, but stacked SR
is `$2719` / `$2715`: X is already 1. The retried instruction consumes it.
The no-fault execution gives the correct result. A diagnostic handler
that restores this probe's known input CCR (`andi.w #$FFE0,(sp)`) also
gives the correct result after one fault. That control is not a proposed
general software workaround or an assertion that real 68040 write-fault
frames must always contain pre-instruction flags. It isolates the
inconsistency in this core's whole-instruction retry model.

Other X-dependent memory operations merit the same checks, but only NEGX
and ROXL are claimed reproduced here.

## 2. P1: retry repeats committed portions of multi-write instructions

[ap040_core.v:2304](rtl/ap040/ap040_core.v#L2304) writes a page-crossing
operand one byte at a time. Earlier bytes can reach memory before a later
byte fails translation/protection. The error path restarts the entire
instruction without recovering the original operand or continuing the
remaining write. Re-reading partially modified memory changes the result.

With page `$7000` writable and page `$8000` readable/write-protected:

| Operation, initially zero | Memory seen by handler | Correct final value | Actual final value |
|---|---:|---:|---:|
| `add.l #$01000000,($7FFF).l` | `$01000000` | `$01000000` | **`$02000000`** |
| `bfchg ($7FFE).l{0:32}` | `$FFFF0000` | `$FFFFFFFF` | **`$0000FFFF`** |

The ADD applies its high-byte change twice. BFCHG flips the first two
bytes twice, undoing them. Restoring the initial CCR in a preliminary
diagnostic control does not fix either case; this defect is independent
of finding 1.

**CAS2 has the same partial-commit failure with aligned operands**, so this
is not confined to misaligned accesses. At
[ap040_core.v:3832](rtl/ap040/ap040_core.v#L3832), the two matching operands
are written in sequence. Starting with `[$7000,$8000] = [$1,$2]`, a
`cas2.l` comparing `$1:$2` and replacing them with `$11:$22` first writes
`$11` to `$7000`, then faults on `$8000`. The handler observes `$11` in
the first location. After repair, the retried first comparison fails and
the second location remains **`$2`, rather than `$22`**.

The code intentionally clears WB3's valid status and returns to the
instruction ([line 1758](rtl/ap040/ap040_core.v#L1758)); it therefore cannot
rely on the handler to complete the saved write instead of recomputing it.
Simply advertising a valid writeback while keeping whole-instruction
retry would reintroduce the already documented double-writeback problem.

The architectural distinction matters: the MC68040 saves pending writes
for handler completion; its locked RMW accesses also check write permission
before accessing operands. See [MC68040 User's Manual, sections 8.4.6.5 and
8.4.6.7](https://www.nxp.com/docs/en/reference-manual/MC68040UM.pdf#page=240).
The numerical failures above are established by executable no-fault
controls and inspection of memory at fault entry, not solely by comparing
the implementation with prose in the manual.

## Reproduction and controls

New runner: [tests/ap040/audit_restart.py](tests/ap040/audit_restart.py).

```sh
python3 tests/ap040/audit_restart.py \
  --work /tmp/ap040-audit2-restart-final --jobs 4 --expect-defects
```

It generates the assembly programs and a copy of the existing core bench
with result logging added. Real MMU write-protection faults drive the
test; no RTL state is forced. Source SHA-256 hashes are recorded alongside
the results and were checked against `ec25690cd` after the runs.
PFLUSHA is explicit at setup because the ATC intentionally survives the
bench's warm resets between handshake phases.

| Cases | Number | Result |
|---|---:|---|
| Five instructions without a fault, caches on/off, posting on/off | 20 | All pass |
| Same combinations with one repaired write fault | 20 | All fail with the values above |
| NEGX/ROXL with fault and diagnostic input-CCR restoration | 8 | All pass |

Each case runs in three handshake phases: 144 phase executions total.
Every faulting phase records exactly one access error. `before_repair`
records operand memory at handler entry (the first operand for CAS2).
`--expect-defects` validates this observed matrix; without that option,
the runner exits nonzero on the current production failures. It does not
silently classify failing CPU tests as passing regressions.

Logs/results: `/tmp/ap040-audit2-restart-final/`. Preliminary diagnostics,
including CCR restoration on the two page-crossing operations, are under
`/tmp/ap040-audit2-restart/`.

## Existing checks and limits

- The previous posted-ordering gate passes: posted and unposted production
  each pass 15/15; removing both fixes restores the expected 7/15 result;
  the old shared-enable control masks the failures again (15/15).
- All 12 selected core programs pass with production-style split enables
  and posting: integer, exceptions, MMU, bitfield/MMU, bitfield/cache,
  MOVES/FC, ATC, MOVEM restart, FPU frames, FPU resume, cache, and FPU.
- Posted snoop/DMA, divide-1/divide-4 collision checks and their negative
  controls, bus-gap, and bus-timeout checks pass 9/9 legs.
- No complete cputest corpus rerun, hardware test, or demo trace was taken.

The passing MMU tests cover successful restart of ordinary stores and ADD
to an aligned operand, but not X-dependent operations or non-idempotent
partial writes. Those successes cannot validate whole-instruction retry
for the cases above.

The next correctness change needs an explicit fault-recovery contract:
preserve the input state required for a true retry, or complete pending
results without re-executing their calculations. Handling CCR alone does
not solve partial writes. Any change must retain the existing MOVEM,
address-register rollback, and no-double-writeback regressions.

---

## Fix status (2026-09-18, in response to this audit)

### Finding 1: FIXED, `afb93236f`

The CCR of an instruction whose destination is memory is held in `fl_pend`
and committed at the write's acknowledge, ahead of `fetch_next` so a trace
or interrupt at that boundary still stacks the completed instruction's
flags. Exception entry discards it, so the frame carries the
pre-instruction CCR the retry needs. Both shift paths get the same
treatment as the ALU path -- ROXL/ROXR consume X whether the count came
through the barrel or the multi-cycle loop.

This audit's matrix goes 20 failing -> 12: all eight X-flag cases pass with
caches and posting independently enabled. Suite 54/54, corpus 3797/3801.

### Finding 2: MMU permission-fault cases fixed in `f86b3980f`

The original proposal below is retained for context. The implemented
follow-up after it describes the final design and its remaining limits.

The fix does not need writeback frames, and does not need the fault path to
learn how to raise an error from a probe. The MMU already answers the only
question that matters, through the port PTEST uses (`pt_req`/`pt_write`/
`pt_addr` -> `pt_done`/`pt_mmusr`, where a probe reports R in bit 0, W in
bit 2 and B in bit 11), and it answers it WITHOUT performing an access.

So: probe the LATER access before committing anything, and reorder only
when the probe predicts a fault.

* `CAS2` -- probe the second operand before writing the first. If it would
  fault, write it FIRST: it faults immediately and the first operand is
  never written. This is what section 8.4.6.7 describes the 68040 doing
  for locked RMW, and it is self-contained in the CAS2 states.
* Page-crossing writes (`S_MWR_B`) -- probe the second page. If it would
  fault, start the byte loop at that page's first byte, which faults on the
  first access with nothing committed.

Every case then faults before any commit, so whole-instruction retry is
sound and no frame construction changes:

| case | outcome |
|---|---|
| both pages/operands writable | normal order, no fault |
| the first faults | faults on the first access, nothing committed |
| the second faults | probe reorders, faults on the first access |
| both fault | reordered; the handler repairs, the retry then faults on the first and converges |

The last row diverges from a real 68040 in WHICH address the first frame
reports, and converges to the same final state over the repairs.

Risk, and why it was not done in the same pass as finding 1: `S_MWR` is on
the path of every write in the core, and the byte-loop reordering needs its
own tests before it can be trusted. The regressions this audit names --
MOVEM restart, address-register rollback, no double writeback -- are the
ones to hold. CAS2 is the safe half and can land first.

### Implemented follow-up: permission checks before partial writes

The RTL now prevents the reproduced MMU permission faults from occurring
after a committed prefix. CAS2 checks both operands, including both pages
of a crossing operand, before reading either operand. Split writes check
both pages before byte zero. Three/five-byte bitfield writes also check
their independent trailing byte before committing the first transfer.

This refines the proposal above: a failed probe raises a format-7 access
error directly from the saved transfer context. Writes retain their normal
order; correctness does not depend on a subsequent reordered write faulting.
The internal check uses the operand's privilege context (including MOVES),
preserves architectural MMUSR/DFC, and honors TC-disabled transparent
translation. Explicit PTEST retains its existing TC-disabled table walk.
An outstanding instruction fetch retires before the probe takes the port.

The guarantee concerns translation/protection faults in stable mappings.
It does not solve physical operand bus errors after a partial write, or
add external bus locking. For a three/five-byte bitfield with both pages
write protected, the trailing-byte check can report that fault first.

Validation against the implemented source:

- Original restart audit: 48/48 cases pass, three phases each.
- New `test_partial_restart.py`: 688/688 cases pass, three phases each.
  The unchanged `b3da46b6a` baseline fails 232 of the same cases, with
  456 passing controls. Tests check untouched operands at handler entry,
  final data/CCR, fault count, FA/SSW, MMUSR/DFC, 4K/8K pages, distinct
  user/supervisor roots, caches/posting, CAS2 aliases and compare failure,
  MOVES, and TC-disabled transparent protection.
- Deferred-flag audit: 44/44 cases pass, three phases each.
- Existing Verilator suite: 54/54 legs pass. The new `partial_restart`
  suite leg also passes separately, giving 55 passing legs in total.
- Corpus: all 3,801 slices pass **with a 64-round limit per slice**. This
  bounded run does not replace the earlier uncapped 3,797/3,801 result or
  establish that its four failures were fixed.

Logs: `/tmp/ap040-restart-fixed`, `/tmp/ap040-restart-suite`,
`/tmp/ap040-partial-negative`, `/tmp/ap040-restart-flags`, and
`/tmp/ap040-restart-corpus/verilator`. The new runner is part of the normal
suite and records its source hashes; `--rtl-dir` selects a baseline for a
negative control. No demo recovery or board performance is inferred from
these simulations. FPGA timing and hardware validation remain separate.

### Physical bus errors: fault sources and the remaining platform assumption

The probe cannot predict a physical bus error, so the partial-commit
window it closes for translation and protection faults stays open for one
below the MMU. Closing that properly needs the writeback frames the
MC68040 uses (WB1/WB2/WB3 valid, handler completion), which would replace
whole-instruction retry and the `no double writeback` regression with it.
That is a much larger change than the probe and is not proposed here.

The direct operand-bus `berr` input has one driver: the wrapper's bus
watchdog. That is narrower than saying every physical/table-walk fault
comes from the two watchdogs below. The complete inspected paths include:

| Source | Path into the core | Timeout / trigger |
|---|---|---:|
| [cpu_wrapper.v:542](rtl/cpu_wrapper.v#L542) `bus_timeout` | `bus_berr` -> `berr` | 2^20 clk_sys cycles |
| [ap040_tg68k_compat.v:134](rtl/ap040/ap040_tg68k_compat.v#L134) `core_stall_watchdog` | `core_stall_flt` -> `mem_flt` | 2^21 clk_sys cycles |
| [Minimig.sv:369](Minimig.sv#L369) `walker_timeout` | walker CDC -> MMU walk error -> `mem_flt` or probe result | 2^16 clk_114 cycles |
| [ap040_walker_cdc.v:144](rtl/ap040/ap040_walker_cdc.v#L144) rejected descriptor address | `s_bad_hold` -> walker bus error -> MMU fault/probe result | no timeout required |

No target module directly asserts operand `bus_berr`. The comment at
[cpu_wrapper.v:41](rtl/cpu_wrapper.v#L41) assumes a write timeout is fatal
when justifying posting. That is an integration assumption, not a property
enforced by the timeout or exception logic. A timeout says a particular
request did not complete within the budget; it does not prove the target
can never answer another request, or that earlier writes did not commit.

The watchdog clears when the request drops
([ap040_bus_timeout.v:20](rtl/ap040/ap040_bus_timeout.v#L20)); the chip-bus
state machine releases its transaction on `bus_berr`
([cpu_wrapper.v:819](rtl/cpu_wrapper.v#L819)); and an ordinary operand fault
enters `aerr_start`, with `fatal_halt` reserved for a fault during exception
entry ([ap040_core.v:2499](rtl/ap040/ap040_core.v#L2499)). Consequently the
RTL does have a handler-entry path. Whether a particular target timeout
is recoverable on the board needs evidence; these connections alone do
not rule out partial-write replay. Walker errors above are reported through
the MMU and are distinct from operand-bus errors after a committed prefix.

One encoding detail follows from the table. `core_stall_flt` joins
`mem_flt` rather than `berr`, and `aer_bus <= berr && !mem_flt`
([ap040_core.v:1725](rtl/ap040/ap040_core.v#L1725)) therefore clears
`aer_bus` for it, so a core-stall timeout stacks with the SSW ATC bit set
and is reported as an MMU fault. That classification can affect a handler's
diagnosis; calling it cosmetic depends on the unproven fatality assumption.
No RTL change is proposed here.

This section is RTL inspection of the fault sources. No board timeout
recovery test or partial-write timeout injection was performed for this
review, and it makes no claim about either demo. Physical bus-error restart
remains an open correctness limit, rather than a proven unreachable case.

### Follow-up review: corpus evidence, CAS2 frame data, and timing

The retained `pw_corpus.log` contains 3,801 distinct slice records:
3,797 pass, four fail, no timeouts or errors, in 957.27 seconds. Its four
failures are the documented Issue 1 slices: `BasicFPU/FADD.L/0001`,
`BasicFPU/FNEG.B/0002`, `BasicFPU/FSNEG.S/0002`, and
`BasicFPU/FSNEG.X/0007`. Claude reported this as an unbounded run on
`f86b3980f`. The retained log corroborates the counts and failure set;
the referenced `/tmp/pw_corpus/verilator` JSON/XML reports are not present
in this workspace, so source hashes and per-slice round totals were not
independently checked here. This review did not rerun the corpus.

Retained log:
`/tmp/claude-1000/-home-adam-ap040x2/a9d14bf8-a66d-4135-b333-55b6c7dd6a8d/scratchpad/pw_corpus.log`.
SHA-256: `8130c5329e07ec7bf9db549f8b9c8c6f480ab84797dca43e6e6af74d18e96fb5`.

CAS2's pre-read checks pass zero write data to `check_write`; a failed
probe therefore captures zero in `aer_wd` and stacks it in WB3D. WB3S is
zero, so this is not an advertised pending writeback. The probes run
before comparisons and update-register reads. The zero is diagnostic
placeholder data under the current whole-instruction retry model; it must
be revisited if valid writeback frames are implemented. Hardware-exact
contents of an invalid WB3D slot were not established by this review.

The posted-store ordering observation is supported by
`walker_req = walker_req_mmu & ~post_drain` in the compatibility wrapper.

The timing build already completed. The named `ec25690cd` build from
12:11:23 has +0.132 ns CPU-domain setup. The `f86b3980f` build from
15:24:11 completed at 15:42:16 and fails at -1.253 ns setup / +0.068 ns
hold. The rejected artifact and SDRAM critical path are recorded in
`tests/ap040/PERFORMANCE.md`. An idle Quartus process list now cannot
establish that this earlier build never ran. Nothing was flashed.
