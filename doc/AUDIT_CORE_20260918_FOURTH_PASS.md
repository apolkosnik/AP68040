# Fourth CPU audit: an unsnooped cache window, deferred flags, pending traces

This file contains the six-finding MMU/trace/cache audit. The independent
RESET finding and 780-case normal-execution results are restored in
[AUDIT_EXECUTION_RESET_20260918.md](AUDIT_EXECUTION_RESET_20260918.md).
The subsequent Fast RAM integration reproduction is in
[AUDIT_FAST_RAM_DMA_20260918.md](AUDIT_FAST_RAM_DMA_20260918.md).

Audited on 2026-09-18 at `28d1101b2`. Six reported findings. One is a coherence hole
in the SoC wiring that can return stale data to the CPU; the rest are in
the core, three of them pre-existing and two low-priority regressions from
the write-probe fix `f86b3980f`. Four predate that fix; two appear with it.
The user subsequently confirmed that the loaded image is
`Minimig-ap040-40mhz-f86b3980f-20260918_152411-TIMING-FAIL-DO-NOT-FLASH.rbf`,
so the earlier card/source distinction does not exclude the two newer
findings. This image also fails timing.

This pass deliberately avoided what the first three covered. It targeted
what the cputest corpus structurally cannot reach, because that corpus runs
with translation disabled: the deferred-flag mechanism in the paths
`afb93236f` did not touch, the trace machinery interacting with faults, MMU
history bits under an enabled TC, and the new probe's side effects.

The same tests confirm `f86b3980f` doing its job. A CAS2 that faults on its
second operand now leaves the first operand unchanged, where the previous
commit committed it before faulting.

No production RTL was edited, no Quartus build was started, and nothing was
flashed. Diagnostic RTL copies are causal controls, not timing-validated
patches. Every run recorded SHA-256 hashes of its sources, and all three
bench handshake phases had to agree or the runner aborted.

These findings have **not been connected** to Elysium or Deformations.
The Fast RAM finding needs an external write to a resident cacheable line;
the core findings need MMU/trace conditions. No failing demo trace has
established those conditions. The demo question remains where the
[first audit](AUDIT_DEMOS_20260918.md) left it.

## 1. P1 by class, narrow by reach: Fast RAM is cached but never snooped

This is the only finding in this pass that can hand the CPU wrong data, and
it is not in the core. It is a missing wire at the top level.

The chain, each link read directly rather than inferred:

| Step | Where | What it establishes |
|---|---|---|
| Z2 and both Z3 windows are cacheable | [ap040_tg68k_compat.v:369](rtl/ap040/ap040_tg68k_compat.v#L369) | `cache_win` admits Fast RAM |
| gated by the autoconfig enables | [cpu_wrapper.v:469](rtl/cpu_wrapper.v#L469) | live whenever Fast RAM is configured |
| Fast RAM selects `ram2` | [Minimig.sv:335](Minimig.sv#L335) | `zram_sel = \|ram_addr[28:26]` |
| `ram2` is the DDR controller | [Minimig.sv:804](Minimig.sv#L804) | Fast RAM lives in DDR |
| the CPU's snoop comes from `ram1` | [Minimig.sv:744](Minimig.sv#L744) | `chip_snoop_tgl` is the SDRAM controller's |
| the SDRAM controller exports one | [sdram_ctrl.v:73](rtl/sdram_ctrl.v#L73) | `output snoop_tgl` off `chipWE \| walker_snoop` |
| **the DDR controller exports none** | [ddram_ctrl.v:141](rtl/ddram_ctrl.v#L141) | its snoop feeds only its **own** `cpu_cache_new` |

So a master writing Fast RAM updates the outer cache and leaves the AP040's
internal D-cache holding the pre-write line. The two levels disagree and the
inner one is the stale one. Eviction, CINV, or an unrelated SDRAM/walker
snoop invalidating the same D-cache set can remove the stale copy.

Such a master exists. [chipdma_arb.v:233](rtl/chipdma_arb.v#L233) sets
`ak_is_ddr <= router_zram_sel`, so Akiko and CDTV CD-ROM DMA whose target
falls in Fast RAM is driven out on `ddr_out_*` into exactly that unsnooped
port. Akiko's address is 24 bits (Z2 reach); CDTV's is 32 bits and can
also reach the configured Z3 windows.

The asymmetry looks like an oversight rather than a decision. The comment
above `cache_win` reasons carefully about which windows are safe and
discusses only chip RAM, which genuinely is snooped. The **walker's**
version of this same hole in the DDR path was found and fixed before, via
`wsnp_pend` in the compatibility wrapper; the DMA version was not.

What source inspection cannot settle is how often it fires. Exposure
depends on DMA targeting a resident cacheable Fast RAM line and software
not invalidating it before use. Workbench/demo exposure requires evidence
of their actual DMA configuration and cache maintenance.

**Independent follow-up now reproduces it dynamically:** the real cache,
bus adapter, router, DMA arbiter and DDR controller return stale CPU data
in 16/16 wiring-equivalent cases. Cache bypass and a diagnostic correct-set
snoop each pass 16/16 controls; a wrong-set snoop leaves all 16 cases stale.
This focused integration bench does not instantiate the complete SoC.
See [the executable reproduction and limits](AUDIT_FAST_RAM_DMA_20260918.md).

Dropping the Z2 and Z3 terms from `cache_win` contains the inner-cache
failure but also disables Fast RAM instruction caching through the shared
predicate. A snoop export/merge must preserve independently arriving
events; **OR-ing independent toggle signals is unsafe**. Current AP040
invalidation uses only set bits `[9:4]`, which DDR remapping preserves, so
a full reverse address mapping is not required for this whole-set policy.

## 2. P2: an aborted bitfield instruction still stacks its own flags

`afb93236f` deferred an instruction's condition codes into `fl_pend` so a
faulting memory write stacks the pre-instruction CCR, and `b3da46b6a` made
`aerr_start` discard them. Both act on `fl_pend`. The memory bitfield path
never joined that mechanism: `S_BF_M3` assigns `sr[3:0]` directly at
[ap040_core.v:5200](rtl/ap040/ap040_core.v#L5200), before the write is even
attempted at `S_BF_WR1`.

So a BFCHG, BFCLR, BFSET or BFINS whose write faults stacks flags it
computed but never committed. This is exactly the defect those two commits
fixed for the ALU and shift paths, in the one writing path they missed.

Readable but write-protected page, initial CCR zero, and a handler whose
first instruction reads its own CCR:

| Instruction | Stacked SR | Handler's live CCR | Memory |
|---|---:|---:|---|
| `bfchg ($8000).l{0:32}` | **`$2704`** | **`$04`** | unchanged |
| `bfclr ($8000).l{0:32}` | **`$2704`** | **`$04`** | unchanged |
| `bfset ($8000).l{0:32}` | **`$2708`** | **`$08`** | unchanged |
| `add.l #1,($8000).l` | `$2700` | `$00` | unchanged |

ADD is the positive control and it is the point: it takes the same fault on
the same page and stacks a clean CCR, because it goes through `fl_pend`.
The three bitfield forms stack their own N or Z on an instruction that
wrote nothing. Every no-fault run leaves the correct flags, so the values
above are genuinely the aborted instruction's.

A diagnostic core copy that routes only the four writing forms of the
**memory** path through `fl_pend`, leaving the register path `S_BF_X3`
committing immediately because it has no write to defer to, fixes all three
rows and leaves the no-fault flags correct. With it, `t_bitfield_mmu`,
`t_bitfield_cache`, `t_integer`, `t_exceptions`, `t_mmu`, `t_cache` and
`t_movem_restart` all still pass. That is a causal control, not a
timing-validated patch.

Reachability is the same as the defects already fixed: a recoverable write
fault under an enabled MMU. It does not arise on an ordinary AmigaOS boot.

## 3. P2: an access error leaves a T0 change-of-flow trace pending

A taken branch under T0 does not raise its trace immediately. `go_pc` parks
it in `flow_t0_pend` and `S_FETCH` raises it once the target's opcode word
arrives. Every exception that runs through the `e_go` carrier clears that
flag at [ap040_core.v:6537](rtl/ap040/ap040_core.v#L6537), because, as the
core's own comment at [ap040_core.v:2296](rtl/ap040/ap040_core.v#L2296)
puts it, no exception leaves a T0 trace pending on the 040.

`aerr_start` does not run through `e_go`. It assigns `state <= S_AERR0`
directly, and it clears `fl_pend` but not `flow_t0_pend`
([ap040_core.v:1724](rtl/ap040/ap040_core.v#L1724)). The pending trace
survives exception entry, and the `flow_t0_pend` branch at
[ap040_core.v:2288](rtl/ap040/ap040_core.v#L2288), which has no `in_exc`
guard, raises vector 9 **in place of the fault handler's first
instruction**.

This is the same structural gap `b3da46b6a` closed for the deferred flags,
in the same task, left open for the other pending event.

| Case | Access errors | Traces | Trace stacked PC |
|---|---:|---:|---:|
| T0 set, branch target page not resident | **0** | **1** | **`$00000480`** |
| T0 set, target resident | 0 | 1 | `$00005000` |
| T0 clear, target page not resident | 1 | 0 | none |

`$00000480` is the access-error handler's own entry address and
`$00005000` is the branch target. The two controls are what make the first
row mean something: with T0 clear the fault is taken normally and the
handler runs, and with the target resident the ordinary T0 trace fires and
correctly stacks the branch target. Only the combination loses the
handler's first instruction to a spurious vector 9, and leaves SSP 12 bytes
low because a format-$2 frame now sits on top of the format-$7 one.

Adding `flow_t0_pend <= 0` to `aerr_start` in a diagnostic copy fixes the
first row while leaving both controls unchanged, so no legitimate trace is
lost. With that one line, `t_exceptions`, `t_mmu`, `t_integer`,
`t_movem_restart` and `t_cache` all still pass.

**Verified in simulation against named commits.** The matrix reproduces
byte-identically on `ec25690cd` and `33e173e22`, the latter associated
with the validated 8,553 Dhrystone image. That is a source/simulation
comparison, not a reproduction of the defect on the current card.
It predates all of this session's work.

T0 tracing is what a debugger or single-stepper turns on, not something
AmigaOS runs with, so this cannot affect an ordinary boot or either demo.
It matters to a demand-paged system, where it turns a page fault on any
branch, `jmp` or `rts` target into a supervisor trace trap taken inside the
page-fault handler.

## 4. P3: a change-of-flow trace and an interrupt nest the wrong way round

[ap040_core.v:1866](rtl/ap040/ap040_core.v#L1866) states the rule this core
follows: an interrupt sampled at the completing instruction's boundary wins
over a simultaneous **T1 or T0** trace, and the trace is redelivered at the
interrupt handler's entry through `texc_pend`. `fetch_next` and `go_pc`'s
T1 branch implement it. `go_pc`'s T0 branch never samples `irq_pend` at
all; it parks the trace and `S_FETCH` raises it unconditionally, after
which `S_EXC_JMP` stacks the interrupt on top.

Sweeping the delayed level-2 source across the boundary, with each handler
recording the shared sequence value it saw:

| Mode | Handler that runs first | Trace frame stacked PC | Interrupt frame stacked PC |
|---|---|---:|---:|
| T1 | trace | `$0470`, the interrupt handler's entry | `$0430`, the branch target |
| **T0** | **interrupt** | **`$0430`, the branch target** | **`$044A`, the trace handler's entry** |

The two are exact mirrors. Under T1 the interrupt frame is the outer one
and the trace preempts the interrupt handler's first instruction; under T0
the trace frame is outer and the interrupt preempts the trace handler's
first instruction. T0 shows this at every delay swept, T1 only inside the
simultaneous window, which is what a genuinely deferred trace should look
like.

This needs T0 tracing; it does not require the write fault in finding 2.

## 5. P3: a failed CAS2 marks both operand pages modified

`f86b3980f` probes MMU write permission before committing a partial
operand. The probe reuses the PTEST port, and a PTESTW-style table search
sets U and M when the probed write is permitted
([ap040_mmu.v:332](rtl/ap040/ap040_mmu.v#L332), `w_hist_m`). Both CAS2
operands are probed at [ap040_core.v:3936](rtl/ap040/ap040_core.v#L3936)
**before either operand is read**, so the probe runs whether or not the
comparison will succeed. A CAS2 whose comparison fails writes nothing, yet
leaves both operand pages marked modified.

| Case | Page `$7000` | Page `$8000` | Memory changed |
|---|---:|---:|---|
| CAS2, comparison succeeds | `$0000701B` | `$0000801B` | yes, correctly |
| **CAS2, comparison fails** | **`$0000701B`** | **`$0000801B`** | **no** |
| CAS, comparison fails | `$0000700B` | untouched | no |
| Plain reads | `$0000700B` | `$0000800B` | no |

`$1B` carries M (bit 4); `$0B` does not. Single CAS is the control that
isolates the cause: it is not probed, and its failed comparison correctly
leaves M clear.

All six cases were rerun against a frozen copy of `b3da46b6a`, the commit
immediately before the probe. Every case is identical between the two
versions **except** the failed CAS2, which leaves M clear there.

A spuriously set M bit tells an operating system to write back a page that
was never modified. The data is correct either way, so this costs
unnecessary writebacks rather than integrity, and AmigaOS does not page.

The page-crossing probe sets M the same way but is not a divergence: before
the fix that case reached the same state by actually performing the partial
write the fix exists to prevent.

## 6. P3: a probe-detected fault reports no write data in its frame

The two CAS2 probes are issued before `rr_a`/`rr_b` are pointed at the
update registers Du1 and Du2 in `S_CAS2_6`, so the update data is not yet
readable and both call sites pass a placeholder of `32'd0`. It reaches
`m_wdat`, is captured into `aer_wd`, and lands in the frame's WB3D slot
([ap040_core.v:1814](rtl/ap040/ap040_core.v#L1814)).

| Frame field | Current | `b3da46b6a` |
|---|---:|---:|
| Fault address | `$00008000` | `$00008000` |
| WB3A | `$00008000` | `$00008000` |
| **WB3D** | **`$00000000`** | **`$00000022`** |
| WB3S | `$0000` | `$0000` |
| SSW | `$0605` | `$0605` |
| **First operand at fault** | **`$00000001`** | **`$00000011`** |

The last row is the fix working: the first operand is untouched now and was
committed before. Only WB3D regressed.

P3 because WB3S valid stays clear by design, so no conforming handler acts
on WB3D. [ap040_core.v:1812](rtl/ap040/ap040_core.v#L1812) nevertheless
states the slot is kept "for diagnostics", and a probe-detected fault
defeats that. It is not cheap to fix, because the update operands reach the
register file read ports only two states later. Against a diagnostic-only
field, accepting the divergence and correcting the comment is the better
trade.

## 7. Observations

**STOP does not consult the store buffer.** The NOP fix from the first
audit gave the core a `store_busy` input and an `S_NOP_SYNC` state, applied
to NOP alone. [ap040_core.v:6461](rtl/ap040/ap040_core.v#L6461) samples
`irq_pend` with no such interlock. A monitor on
`state == S_STOPPED && post_drain` confirms the core enters the stopped
state with an acknowledged store still in flight: 3 cycles in the first
bench handshake phase, 8 or more in the second. No wrong interrupt decision
was produced from it, and unlike NOP, whose synchronizing behavior
[MC68040 User's Manual section 7.7](https://www.nxp.com/docs/en/reference-manual/MC68040UM.pdf#page=186)
specifies, STOP carries no architectural guarantee to drain. Recorded
because AmigaOS executes STOP in its idle loop after clearing an interrupt
source, the idiom that made the NOP defect reachable, and because draining
there costs nothing.

**Three small cache items, none producing wrong data.** `inv_we`
([ap040_cache.v:131](rtl/ap040/ap040_cache.v#L131)) is assigned and never
consumed, since the tag RAM takes `inv_wren`; a reader checking port B
arbitration will follow the wrong wire. `rd_accept`, `store_inv` and the
`store_inv_lost` recorder omit guards that the FSM branches consuming them
do have, which costs over-invalidation and a phantom lookup rather than
correctness. And the comment at
[ap040_cache.v:438](rtl/ap040/ap040_cache.v#L438) saying every accepted
store clears its row went stale when update-on-hit landed: `store_inv` now
requires `!fits_long`, so a fitting store merges instead. That leaves the
read and write paths disagreeing about a cache-inhibited hit, which only
becomes reachable through finding 1.

**The MMU header documents the opposite of the RTL.**
[ap040_mmu.v:20](rtl/ap040/ap040_mmu.v#L20) says invalid translations are
not cached, so a descriptor fixed by a handler takes effect without a
PFLUSH. `W_FLT` asserts `fill_we` and installs a valid, nonresident entry,
and `atc_fault` then faults every later access without re-walking. The RTL
is the architecturally correct one and software must PFLUSH; the comment is
a load-bearing claim about the ATC contract that a future change could be
built on. `t_mmu`'s handler always issues `pflusha`, so nothing exercises
the contract the comment promises.

## 8. Coverage gaps worth closing

No defect was found behind these, but nothing would catch one. Neither the
WinUAE differential nor `t_mmu.s` exercises indirect page descriptors
(PDT=10), the W bit set in a **root or pointer** descriptor, or invalid
root/pointer descriptors. Write-protect accumulation down the table levels
is implemented and looks correct, but every generator emits
`TBL_PTR|3`/`TBL_PAGE|3`, so no test sets W above the page level. Extending
the differential's operand generator to emit indirect descriptors and
non-zero upper-level W bits would cover the only walker paths with no
oracle behind them.

## 9. Candidates examined and disproved

Recorded so a later pass does not spend time on them again.

- **BFTST faulting on a read-only page.** The trailing-byte probe is
  reached only through `S_BF_M4`, and
  [ap040_core.v:5206](rtl/ap040/ap040_core.v#L5206) dispatches BFTST,
  BFEXTU, BFEXTS and BFFFO to `fetch_next` instead.
- **The probe walking with the wrong function code.**
  [ap040_core.v:180](rtl/ap040/ap040_core.v#L180) drives `pt_fc` from the
  operand's supervisor bit and the MOVES override, selecting data space,
  and falls back to DFC only for a real PTEST.
- **The probe clobbering the architectural MMUSR, hanging, leaking its
  sideband, corrupting caller state at any of the four call sites, or
  mis-terminating its two-page loop.** All traced and correct; the fault
  condition is bit-for-bit equivalent to the real access path's.
- **A T1 trace surviving TRAP, CHK, TRAPV or a zero divide.** Guarded to
  processors below the 68040 in the reference implementation; dropping it
  is correct here and `t_exceptions` pins it.
- **`irq_hold_lvl` keeping a request alive across a mask raise.**
  Deliberate, hardware-motivated, covered by `t_exceptions` test 136.
- **Format-$1 throwaway frames and M-bit stack switching**, **A7 rollback
  against the S-bit change in `S_AERR_U`**, **frame layouts and SSW
  packing**, **RTE format validation**, **autovector numbering**.
- **In the cache, the whole store-buffer class.** A load bypassing a posted
  store to the same address, partial overlaps in either direction, a fill
  installing over a pending store, a cache-inhibited read overtaking one,
  and the walker reading a descriptor the buffer still holds: all traced at
  both full and divided clock enable and all correct. The merge is a full
  read-modify-write of the resident longword, so partial overlap is exact.
  Also checked and correct: snoop against fill and against the acceptance
  tick, sweep-versus-snoop suppression, every port A/B collision, the snoop
  clock crossing, maintenance against in-flight lookups, cache-inhibited
  reads staying out, self-modifying code, and the cache-disabled path.
- **In the MMU:** TTRs active with TC.E clear, TTR field positions,
  write-protect accumulation, indirect descriptor resolution, U/M on denied
  accesses, descriptor type decoding, 4K/8K layout, MMUSR bit assembly, all
  four PFLUSH variants, the duplicate-way hazard, ATC read-during-write,
  and MOVEC to TC/URP/SRP not flushing.

## Reproduction

```sh
python3 tests/ap040/audit_bitfield_flags.py   --tag current   # finding 2
python3 tests/ap040/audit_flow_trace.py       --tag current   # finding 3
python3 tests/ap040/audit_trace_irq_order.py  --tag current   # finding 4
python3 tests/ap040/audit_probe_hist.py       --tag current   # finding 5
python3 tests/ap040/audit_probe_frame.py      --tag current   # finding 6
```

Each takes `--rtl-dir` to run the identical cases against another
checkout's `rtl/ap040`, which is how the `b3da46b6a`, `ec25690cd` and
`33e173e22` comparisons were produced and how both diagnostic controls were
checked.

All five generate real 68040 programs, build their page tables in the
program itself, and read results back through ordinary loads. No RTL state
is forced and no production file is modified; the bench copies add result
logging only. Every run flushes the ATC explicitly at setup, because the
bench's warm resets between handshake phases intentionally preserve
translations.

Each exits nonzero on its defect and each passes on a version that does not
have it, so the failures are meaningful rather than expectations written to
match. As with the earlier runners, `--expect-defects` asserts today's
matrix and deliberately rejects a changed one after a fix. The frame runner
checks both sides, so current RTL fails it only on WB3D while `b3da46b6a`
fails it only on the committed partial write.

## Limits

This pass did not rerun the cputest corpus, run the full Verilator suite,
take any timing measurement, or touch hardware. It did not audit the FPU,
MOVEM, MOVE16, or the bus adapter. Findings 5 and 6 concern descriptor
accounting and a diagnostic frame field. Finding 1 was initially established
by inspection; the independent integration follow-up now demonstrates
stale CPU read data, as documented in
[AUDIT_FAST_RAM_DMA_20260918.md](AUDIT_FAST_RAM_DMA_20260918.md).
