# CPU audit prompted by Elysium and Deformations

Audited on 2026-09-18. Two posted-write synchronization defects reproduced
in simulation. Neither has yet been tied to an instruction trace from the
reported demos. The running image, failing parts, cold-boot reproducibility,
and cache/MMU settings were not supplied, so this is not a diagnosis of the
two demos or a claim that their graphics corruption is explained.

Scope: CPU interrupt entry/return, posted stores, cache maintenance, MMU
cache attributes, bus adapter/enables, and DMA snoop paths, supported by
the existing ISA/MMU/FPU tests. This is not an exhaustive ISA or chipset
audit. No production RTL was edited, no Quartus build was started, and no
image was flashed.

The checkout is `dc982258b` with pre-existing staged changes in
`rtl/ap040/ap040_cache.v` and `tests/ap040/tb_ap040_cache_snoop.v`. Those
changes were preserved. `git diff 33e173e22 -- rtl` is empty: the audited
working-tree RTL is the deployed baseline, not the parked four-entry queue.

## 1. High priority: NOP does not synchronize the posted write buffer

At [ap040_core.v:5666](rtl/ap040/ap040_core.v#L5666), NOP immediately calls
`fetch_next`. The core has no input telling it that the cache is still
draining a write. Production [cpu_wrapper.v:372](rtl/cpu_wrapper.v#L372)
lets the core execute during that drain.

Repro: enable I/D caches; run code and a word-aligned stack in configured
Fast RAM; request two level-2 interrupts. The handler increments a register,
clears a synthetic interrupt device, executes NOP, then RTE. The clear is
delayed on the external bus. The first invocation warms the handler and
stack. With SP = 2 modulo 4, RTE's stacked PC is longword-aligned and its
frame reads can hit in cache. On the second invocation, NOP and RTE execute
before the clear lands; RTE accepts the old asserted interrupt again.
The program observes **three entries for two requests**.

This reproduces in all three handshake phases with delays of 32, 64, 128,
and 200 bench clocks. Zero added delay passes. Disabling posting passes.
A diagnostic-only core copy that waits for the drain at NOP also passes.
SP = 0 modulo 4 masked the issue in the preliminary probe: the unaligned
PC read bypassed the cache and implicitly waited for the drain.

The NOP behavior violates the bus synchronization requirement in
[MC68040 User's Manual, section 7.7, printed page 7-44](https://www.nxp.com/docs/en/reference-manual/MC68040UM.pdf#page=186).
The no-NOP version is not used as evidence of an architectural defect.
Cache-inhibited serialized mode does not by itself make every write wait
for completion; section 7.7 describes its ordering restriction on reads.

An extra interrupt can disturb frame counters and demo part sequencing.
That is a plausible connection, not an observed trace of either demo.

## 2. High priority: cache maintenance can complete before an earlier write

[ap040_cache.v:653](rtl/ap040/ap040_cache.v#L653) enters the invalidate sweep
without checking `sb_v`. At [line 828](rtl/ap040/ap040_cache.v#L828), it raises
`cinv_done` after the sweep regardless of whether the posted write landed.
The core consumes that completion at
[ap040_core.v:3700](rtl/ap040/ap040_core.v#L3700). CINV and CPUSH share this
path because the cache is write-through.

Replacing the probe's NOP with either `cinva dc` or `cpusha dc` reproduces
`cinv_done && post_drain` at delays 128 and 200, in all three handshake
phases. A monitor stops each run on that condition; these runs do not
claim to observe subsequent graphics corruption. Delays 0, 32 and 64
pass. Disabling posting, or making a diagnostic cache copy wait for the
drain before starting maintenance, passes every tested delay.

The maintenance timing/ordering rules account for pending writes and
prefetches in [MC68040 User's Manual, section 10.3, printed page 10-8](https://www.nxp.com/docs/en/reference-manual/MC68040UM.pdf#page=299).
Write-through removes dirty-line writebacks, but an acknowledged store
still sitting in this buffer has not reached memory. Maintenance cannot
treat that store as already globally complete.

## 3. Coverage gap: core_post does not model production drain overlap

[tb_ap040_program.v:65](tests/ap040/tb_ap040_program.v#L65) stops its enable
during a bus wait, and [line 153](tests/ap040/tb_ap040_program.v#L153) connects
that same enable to both core and adapter. The suite's `core_post` entry
changes only `POST_STORES=1`. It therefore does not test the production
split where the core continues while a posted write waits on the bus.

Restoring the original shared enable in the new probe makes **all 15
cases pass on the defective RTL**. This directly establishes the blind
spot. The wrapper-based benches do have the real split; their passing
results do not cover this combination of warmed Fast RAM code/stack,
stack alignment, delayed interrupt-clear write, and synchronization.

## Reproduction and results

Run:

```sh
python3 tests/ap040/audit_posted_ordering.py --work /tmp/ap040-posted-order-audit --jobs 4
```

The runner creates temporary bench/RTL copies; production files remain
untouched. It uses [t_posted_irq_audit.s](tests/ap040/asm/t_posted_irq_audit.s).
The device at `$00dff110` is a synthetic interrupt latch, **not** a model
of the actual Amiga register at that address. Fast RAM is backed by the
bench's flat memory through an explicit address alias. The diagnostic
NOP control uses a hierarchical reference and is not a production fix.

Each case runs three handshake phases. There are five delays per instruction.

| Configuration | NOP cases | CINVA cases | CPUSHA cases |
|---|---:|---:|---:|
| Baseline, production-style split enables | 1/5 pass | 3/5 pass | 3/5 pass |
| Posting disabled | 5/5 pass | 5/5 pass | 5/5 pass |
| Temporary drain-wait controls | 5/5 pass | 5/5 pass | 5/5 pass |
| Original core bench shared enable | 5/5 pass | 5/5 pass | 5/5 pass |

Logs and structured results: `/tmp/ap040-posted-order-audit/`.
The runner records observed failures; it is an audit tool, not yet a
regression-suite entry that requires all production cases to pass.

Existing verification also ran:

- Posted snoop/real-memory DMA checks, divide-1/divide-4 collision checks,
  their must-fail controls, bus-gap and bus-timeout tests: 9/9 legs passed.
- Core tests: all 14 assembly programs passed both with and without
  posting. Dhrystone initially failed to compile because `vbccm68k` was
  absent from PATH; rerunning Dhrystone with the toolchain directory on
  PATH passed in both configurations. This was an environment failure,
  not a CPU test failure.
- Fast-clock divide-4 wrapper: all seven selected programs passed
  (`t_integer`, `t_exceptions`, `t_mmu`, `t_fpu`, `t_moves_fc`,
  `t_movem_restart`, `t_atcprobe`).

No new DMA stale-data failure was found in these checks. They do not run
the demos, the complete chipset, or the real snoop CDC with every DMA
arbitration schedule. The full cputest corpus was not rerun for this audit.

Recommended next change: implement a proper core-visible drain completion
contract for NOP, and interlock cache maintenance with pending stores;
turn these repros into regression gates using production enable behavior.
Then validate on the named demos with a timing-clean image, recording the
exact image, failing part, cache/MMU setup, and cold-boot behavior. No
timing or performance claim can be made for the temporary controls.

---

## Status (2026-09-18): which of these ever ran on the board

Both defects above were fixed in the previously reported card image.
The NOP synchronization and maintenance interlock landed in `ec25690cd`,
which measured 8,559 Dhrystones. The latest user report confirms the card
now runs the timing-failing `f86b3980f` artifact: Deformations improves,
with mouse-pointer-related artifacting remaining. Cold-boot behavior is
awaiting confirmation; no new Elysium result was supplied.

Distinguish when a fix landed from when its defect existed. Two later
commits repaired bugs introduced by the deferred-flag change; the other
two repaired defects already present in the flashed source:

| Fix commit | Defect | Fix in `ec25690cd`? | Defect in `ec25690cd`? |
|---|---|---|---|
| `ec25690cd` | NOP sync, maintenance interlock | yes | fixed |
| `afb93236f` | retry consumed its own X flag | no | yes, if a fault/retry exercises it |
| `8473f4c68` | split write lost its deferred flags | no | no; introduced by `afb93236f` |
| `b3da46b6a` | aborted deferred flags reached the handler CCR | no | no; introduced by `afb93236f` |
| `f86b3980f` | partial writes replayed after a permission fault | no | yes, if a fault/retry exercises it |

The two deferred-flag defects were created after the card was flashed and
fixed before the tip, so they were never in an image where a demo ran. The
X-flag and partial-write failures reproduced by the audits require a fault
and retry; the demonstrated cases use recoverable MMU permission faults.
The `ec25690cd` source commits flags before the store and writes CAS2's
first operand before attempting its second. The installed memory tools,
actual page permissions, and presence or absence of faults in the failing
demos have not been established. Assuming an ordinary unprotected AmigaOS
configuration can lower their priority, but does not rule them out. Physical
bus-error restart also remains a separate limit, as the second-pass audit
now explains.

The earlier recommendation was to run both demos on the then-current
`ec25690cd` image. The user has since identified a different image on the
card. Returning to `ec25690cd` for comparison requires loading its existing
named artifact and a cold boot, but no new build. That image passes the CPU
timing gate; its HDMI domain does not close (see below).
If they still fail, the two shipped ordering fixes are insufficient to
resolve those failures; that does not exclude a contribution from those
bugs, the unfixed restart paths, or an additional cause. If they pass, a
repeatable comparison against the earlier image under the same cold-boot
and machine configuration would strengthen the connection. One successful
run alone does not identify which change caused recovery.

Recording the image, the failing part, the cache and MMU settings, and
cold-boot versus reset behavior is what would make either outcome usable.

The later `f86b3980f` image has already been built and fails setup at
-1.253 ns. It must not be used for the demo comparison. See
`tests/ap040/PERFORMANCE.md` for the completed build and preserved artifact.

### Board follow-up: Deformations recovered, pointer artifacts remain

The user reports that the Deformations issue is gone and suspects timing
closure for remaining artifacts associated with the mouse pointer. They
confirmed the running image as
`Minimig-ap040-40mhz-f86b3980f-20260918_152411-TIMING-FAIL-DO-NOT-FLASH.rbf`.
This records an observed improvement without attributing it to a particular
fix or counting it as validation on a timing-passing image. Power-cycle
repeatability, video output/mode, and whether the pointer itself or the
background is corrupted still need confirmation.

The current image fails emu-domain setup at -1.253 ns and HDMI setup at
-0.667 ns. Its saved TimeQuest report also has a -0.315 ns path from
`ram1|sdata_reg[11]` to `ram1|sdata_chip[11]`. In `rtl/sdram_ctrl.v`,
`sdata_chip` feeds `chipRD` and `chip48_*`, so there is a failing chipset
read-data path upstream of HDMI as well. That makes timing a concrete
candidate for sprite/display corruption; it does not prove the observed
pointer artifacts originate on that path.

For comparison, the `ec25690cd` build at `20260918_121123` passes the emu clock-domain gate
at +0.132 ns setup / +0.090 ns hold, but **fails HDMI setup at -0.345 ns**
(HDMI hold +0.123 ns). `build.sh` deliberately reports but tolerates
`pll_hdmi` violations. Earlier references to this image as simply
"timing-clean" meant only the CPU-domain gate and were too broad for
diagnosing graphical artifacts. The retained build log establishes the
HDMI-domain violation, not its exact failing endpoints. The current fit
database belongs to `f86b3980f`, so its critical paths cannot substitute
for those of `ec25690cd`.

After returning to a CPU-timing-passing image, the RTL provides a useful
output comparison without changing the CPU:
normal scaled HDMI uses `clk_hdmi` for the scaler output, while native
analog output (VGA scaler/framebuffer disabled) takes the core video path.
Direct Video also selects the core video clock/path when `vga_fb` is clear
(`sys/sys_top.v`, `hdmi_clk_sw` and the HDMI output mux). A reproducible
artifact confined to scaled HDMI would favor that output branch; an
artifact shared with native output would shift attention toward sprite
DMA/rendering, memory, or software updates upstream. Neither result alone
proves which timing or logic defect is responsible.

No build, RTL change, or flash was performed for this follow-up.
