# Third CPU audit: deferred condition-code lifetime

Audited on 2026-09-18, starting at `8df2ea6be` (including `afb93236f`).
Two additional defects reproduce in the deferred-CCR change. One affects
successful instructions with no exception; the other changes the live CCR
while constructing an access-error frame. The previous partial-write
restart defect also remains reproducible.

These findings do **not** identify the cause of Sanity's Elysium or
Deform's Deformations failing. The two new defects were introduced after
the earlier board images `33e173e22` and `ec25690cd`. A demo trace and the
exact failing image would be needed to connect a finding to either demo.

This pass reviewed deferred flags, split transfers, access-error and normal
exception entry, instruction completion/dispatch, and the interrupt and
posted-store contracts. It is not an exhaustive ISA or chipset audit.
This audit changed no production RTL, ran no Quartus builds, and flashed
nothing. Concurrent edits to the core and `t_mmu.s` were preserved; all
simulation builds used frozen source copies and recorded SHA-256 hashes.

Final checkout check: HEAD remains `8df2ea6be`, with the concurrent
split-write fix present as an uncommitted core edit. Its core hash matches
the second snapshot below. Finding 1 is resolved in that edit; finding 2
and the earlier partial-write restart issue remain open.

## 1. P1: successful page-crossing writes lose their condition codes

At [ap040_core.v:2348](rtl/ap040/ap040_core.v#L2348), `S_MWR_B` returns
after the final byte without consuming `fl_pend`. The new deferred-flag
commit exists only at the ordinary `S_MWR` acknowledge
([line 2462](rtl/ap040/ap040_core.v#L2462)). A page-crossing write bypasses
that acknowledge, so its flags neither become architectural nor retire.

With translation enabled, both pages writable, and initial CCR zero:

| Instruction at address `$7FFF` | Input operand | Result | Expected CCR | Actual CCR |
|---|---:|---:|---:|---:|
| `addq.w #1,($7FFF).l` | `$FFFF` | `$0000` | `$15` (X/Z/C) | **`$00`** |
| `roxl.w ($7FFF).l` | `$8000` | `$0000` | `$15` (X/Z/C) | **`$00`** |

There is no access error in either case. Branches and subsequent
X-dependent operations can therefore consume stale flags after an otherwise
successful instruction. CCR is captured directly into a data register
before any result-logging store can mask the failure.

The leftover pending flag also leaks into later instructions. After the
crossing ADD, execute `move.w #0,ccr`, then `st ($3502).l`. ST should leave
CCR at zero; its ordinary memory-write acknowledge instead commits the
old ADD's pending flags, giving **CCR `$15`**.

Controls: the same instructions at `$7FFE` pass, and the same unaligned
address with translation disabled passes. The split path is selected by
TC and the page boundary, not merely by unaligned bus access. Each result
repeats with caches independently on/off, posting independently on/off,
and all three bench handshake phases.

A diagnostic core copy that commits and clears pending flags at the final
successful byte fixes all these cases. A concurrent working-tree patch
implementing that same change was separately captured and tested: all 36
normal-execution cases pass. This does not resolve finding 2 or the known
partial-write restart defect. See the snapshot identities below for the
precise tested versions, rather than assuming a changing checkout matches
one particular run.

## 2. P2: access-error stacking commits an aborted instruction's flags

[ap040_core.v:1686](rtl/ap040/ap040_core.v#L1686), `aerr_start`, does not
clear `fl_pend`. It jumps directly to `S_AERR0`, bypassing the `e_go` block
where the new discard was added. `S_AERR0` correctly snapshots the
pre-instruction SR, but the first exception-frame write then uses `S_MWR`:
its acknowledge consumes the faulting instruction's pending flags and
changes the handler's live CCR.

Repro: readable but write-protected page `$8000`, initial CCR zero. The
handler's **first instruction** is `move.w ccr,d7`; it then captures the
stacked SR, repairs the page, flushes the ATC, and executes RTE.

| Faulting instruction | Input | Stacked SR | Handler's initial CCR | Expected handler CCR |
|---|---:|---:|---:|---:|
| `negx.l ($8000).l` | `$00000001` | `$2700` | **`$19`** | `$00` |
| `roxl.w ($8000).l` | `$8000` | `$2700` | **`$15`** | `$00` |

The architectural issue here is the disagreement between the captured SR
and live condition codes created by stack writes, not an assertion that
every real 68040 write-fault frame must carry pre-instruction flags.
Access-error entry copies SR and changes supervisor/trace state; it does
not apply an aborted arithmetic result to CCR during stacking. See
[MC68040 User's Manual, sections 8.1 and 8.2.1](https://www.nxp.com/docs/en/reference-manual/MC68040UM.pdf#page=220).

The final retried arithmetic result is correct in these probes: the frame
still has the input X flag, so the previous audit's arithmetic-result test
passes. A handler that tests or saves live flags before changing them sees
the wrong state. This narrower exception-entry impact is why this finding
is P2 rather than the successful-instruction corruption in finding 1.

This repeats across both cache settings, both posting settings, and all
three handshake phases. The split-write-only diagnostic leaves it failing.
Adding `fl_pend <= 0` to `aerr_start` in a temporary core copy fixes it,
while preserving the correct arithmetic result after repair and RTE.
Those temporary edits are causal controls, not a timing-validated patch.

## Reproduction and results

Runner: [tests/ap040/audit_deferred_flags.py](tests/ap040/audit_deferred_flags.py).

```sh
python3 tests/ap040/audit_deferred_flags.py \
  --work /tmp/ap040-deferred-flags-audit --jobs 4
```

The ordinary command exits nonzero if any CPU test fails. It snapshots the
RTL and bench before compiling, adds result logging to a bench copy, and
generates real 68040 programs using the existing MMU and memory model.
No RTL state is forced. Every run explicitly flushes the ATC at setup,
because the bench's warm resets intentionally preserve translations.

`--diagnostic-controls` also builds temporary core copies with split-write
commit alone, and then with access-error discard as well. Already-present
fixes are retained. `--expect-defects` is specifically for asserting the
original `8df2ea6be` failure matrix; it deliberately rejects a changed
matrix after a fix. It must not be used to classify failing CPU tests as
passing production regressions.

Original frozen-source run: `/tmp/ap040-audit3-flags/`.

| Core variant | Pass | Fail | Phase executions |
|---|---:|---:|---:|
| Production `8df2ea6be` | 24 | 20 | 132 |
| Diagnostic split-write commit only | 36 | 8 | 132 |
| Diagnostic split-write commit + access-error discard | 44 | 0 | 132 |

The 44 cases are ADD, rotate and stale-flag leakage in three modes
(crossing with MMU on, non-crossing with MMU on, unaligned with MMU off),
plus two fault-entry probes, each with caches and posting on/off. Every
case runs three phases. Fault-entry cases record exactly one access error;
all other cases record zero. Every operand result is also checked.

The original core SHA-256 is
`73faa53031c014dc9d52e094ddd46152c11795cc0db5bf65cdb62536a3a5e62d`.

A second source capture includes the concurrent split-completion patch:
`/tmp/ap040-audit3-current/`. It reproduces **36 passing / 8 failing**
cases, confirming that finding 2 survives that patch. Its core SHA-256 is
`c5294dbc7c33ffb3ace915cf8af57e1400a24789765806099dc8a911b78ddd51`.
Both directories retain source copies, generated programs, build logs,
per-run logs, `source_sha256.json`, and `results.json`.

## Existing checks and the still-open restart issue

Against the second frozen source, seven existing programs pass with
posting enabled and all three handshake phases: `t_integer`,
`t_exceptions`, `t_mmu` (including the concurrent page-crossing CCR
regression), `t_movem_restart`, `t_bitfield_mmu`, `t_cache`, and
`t_moves_fc`. Results and the exact assembly copies are under
`/tmp/ap040-audit3-current/existing/`.

The previous partial-write restart finding was retested, not counted as
a new discovery. All 12 faulting combinations (three operations, cache
on/off, posting on/off) still fail identically in all three phases:

| Operation after a repaired second-page/operand fault | Expected | Actual |
|---|---:|---:|
| Crossing `ADD.L #$01000000` | `$01000000` | `$02000000` |
| Crossing `BFCHG {0:32}` | `$FFFFFFFF` | `$0000FFFF` |
| CAS2 second destination | `$00000022` | `$00000002` |

Programs and results: `/tmp/ap040-audit3-current/known-partial/`, generated
from [audit_restart.py](tests/ap040/audit_restart.py). The complete
explanation remains in the [second audit](AUDIT_CORE_20260918_SECOND_PASS.md).

No complete corpus rerun, hardware demo run, or new timing measurement
was performed. The passing existing programs do not cover live handler
CCR or establish that either demo is fixed.
