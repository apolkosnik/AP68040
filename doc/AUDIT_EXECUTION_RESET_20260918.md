# CPU audit: normal execution and RESET, 2026-09-18

This is the independent audit previously linked as
`AUDIT_CORE_20260918_FOURTH_PASS.md`. That shared filename was subsequently
used for the separate six-finding MMU/trace/cache audit. This file restores
the RESET finding and the 780-case execution results under a distinct name.
It does not replace or merge the six reported findings.

Related reports:

- [Six-finding MMU/trace/cache audit](AUDIT_CORE_20260918_FOURTH_PASS.md).
- [Fast RAM DMA integration reproduction](AUDIT_FAST_RAM_DMA_20260918.md).
- [Timing diagnosis for the rejected f86b3980f image](TIMING_F86B3980F_20260918.md).

Audited source: `28d1101b21f0c274d6ece6482c307eebe25a086e`, with production
RTL unchanged from `f86b3980f`. No production RTL was edited, no Quartus
build was launched, and nothing was flashed by this audit. The results
below were recovered from the retained logs and JSON records, not rerun
when restoring this document.

## RESET pulse is too short

[ap040_core.v:5838](rtl/ap040/ap040_core.v#L5838) loads the eight-bit
`rst_cnt` with 127. `nresetout` is low throughout `S_RESET_HOLD`, which
decrements the counter once per enabled core cycle and exits at zero.
The measured pulse is **128 enabled core cycles**.

The [68040 manual, section 7.10](https://www.nxp.com/docs/en/reference-manual/MC68040UM.pdf#page=209)
specifies **512 BCLK cycles** for the RESET instruction's output pulse.
The initial audit qualified this comparison unnecessarily: production
instantiates `cpu_wrapper` without a FAST_CLOCK override, whose default is
zero. Consequently `core_tick = !FAST_CLOCK || (core_phase == 0)` is
always high. There is no divide-by-four interpretation that makes the
128 count correct. Bus waits can extend the pulse; they do not ensure the
required duration when the bus is idle.

The diagnostic measured 128 enabled cycles in all **36 executions**.
Elapsed bench clocks were 128, 136 or 158 depending on fetch stalls. This
behavior already exists in `33e173e22` and was not introduced by the recent
write-permission probe.

The minimum count correction requires widening `rst_cnt` to **nine bits**
and loading **511**, with consistent compare/decrement widths. Retaining
the existing enable gating would still allow bus waits to lengthen the
pulse. No RESET RTL correction is included here; timing recovery is the
current priority.

The reset output reaches fastchip directly in `Minimig.sv:898` and is ORed
into chipset reset in `rtl/minimig.v:482`. The frame-counted system-reset
generator is not triggered by every CPU RESET instruction, so it cannot
be assumed to stretch this pulse. No failed peripheral reset or connection
to the pointer artifacts has been demonstrated.

## Separate observation: RESET can overlap a posted write

The test warms its code, writes a word to `$3000`, then executes RESET.
With the word's bus completion delayed by 32 or 200 clocks, RESET asserts
while `post_drain=1`, busstate is `11`, and backing memory still contains
the old value. This repeats in all three handshake phases. Either a NOP
before RESET or disabling posting makes the write reach memory first.

This demonstrates overlap, not a lost write. The flat memory model keeps
servicing writes during reset, and every program eventually reads the
correct value. The RESET description does not establish NOP's explicit
bus-synchronization contract. An integration test showing harm, or an
architectural requirement, is needed before calling this a second defect.

Runner: [audit_reset_boundary.py](tests/ap040/audit_reset_boundary.py).
The 12 configurations cover posting on/off, RESET with/without preceding
NOP, and three write delays. Each executes three handshake phases.

## Normal-execution results

Runner: [audit_execution_sequences.py](tests/ap040/audit_execution_sequences.py).
It assembles real instructions and checks results against Python bit-list
and integer calculations. It does not force core state.

| Sequence family | Cases passed |
|---|---:|
| Register bitfields: all eight operations, wrapping and register aliases | 192/192 |
| Memory bitfields: negative offsets and cache-line/page crossings | 192/192 |
| Four consecutive register ADDX/SUBX operations, X and sticky Z | 192/192 |
| Four consecutive predecrement memory ADDX/SUBX operations | 192/192 |
| Modified target/adjacent instruction followed by cache maintenance | 12/12 |
| **Total** | **780/780** |

The matrix uses 16 deterministic seeds per arithmetic/bitfield family,
caches on/off, posting on/off, and translation off/4 KB/8 KB. Each case
runs three handshake phases: **2,340 phase executions**. Condition codes
are captured before comparisons change them. No MMU fault is injected;
these passes do not contradict the separate fault/trace findings.

## Cache/DMA unit results

The existing unit tests passed **7/7 expected outcomes**: four positive
legs (`cache_snoop_post`, its CE4 version, and both mixed-port variants)
plus three must-fail controls. These exercise actual backing-memory DMA
writes and supplied cache snoops. They do not verify that each SoC DMA
writer supplies a snoop. The subsequently reproduced Fast RAM wiring hole
is outside their scope; its results are in the separate DMA report above.

## Reproduction and evidence

```sh
python3 tests/ap040/audit_execution_sequences.py --work /tmp/ap040-execution-audit --jobs 4
python3 tests/ap040/audit_reset_boundary.py --work /tmp/ap040-reset-audit --jobs 4
python3 tests/ap040/run_verilator_suite.py --only cache_snoop_post,cache_snoop_post_ce4,cache_snoop_post_x,cache_snoop_post_x_ce4 --work /tmp/ap040-execution-dma --jobs 4
```

The work directories contain generated programs, frozen source copies and
logs. A compact [evidence record](tests/ap040/audit_evidence/execution_reset_summary_20260918.json)
retains the case counts, input hashes, RESET observations and seven-leg
DMA log in the repository. The tested core and program bench hashes still
matched the checkout when this report was restored.

Core SHA-256:
`1f14edfbabdef4f68cabeff2e7df1d7246d4d557bab4a8e9c94a40a3b446a425`.

This is bounded normal-execution and RESET evidence, not a full ISA/FPU
audit or board validation. The full corpus was not rerun for this pass.
The loaded timing-failing image is not a valid measurement platform for
attributing graphics symptoms or comparing performance.
