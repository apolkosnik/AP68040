# AP68040 `faster`

A pipelined MC68040 in SystemVerilog for a 60 MHz processor clock on
Cyclone V (MiSTer, 5CSEBA6U23I7), validated under Verilator, with the
core's ALM count as a design goal.  It presents the real 68040's external
bus (MC68040UM sections 5 and 7), including bus snooping, arbitration,
reset timing, and the IPEND, PST, CDIS and MDIS pins.  The MMU, caches,
exceptions and FPU state frames behave as the 68040's do.

This branch was made from the AP68060 tree -- a 68060 built on this
repository's `ap68040-60mhz` branch and tuned there for area and timing --
converted back to the 68040, keeping that round's pipeline work.  The
previous sequential core of this repository is kept in `rtl_old/`, with
its benches in `tb_old/`, as a reference.

## Numbers

| | |
|---|---|
| clock | 60 MHz (16.667 ns), out of context on three fitter seeds (Quartus 17.0, HIGH PERFORMANCE EFFORT, hold optimized on all paths), at least +0.22 ns at the 100C slow corner; doc/STATUS.md has each seed |
| area | 20,761 - 20,822 ALMs on the three seeds (50 % of the 5CSEBA6), 12,500 registers, 140 M10Ks (the caches, the ATCs' tags, the microcode, the decoder tables, the predecoder table), 15 DSPs -- with every feature of the core; the arithmetic pipeline, the loop predictor and the early results are parameters, off by default |
| Dhrystone 2.1 | 948 cycles/iteration, 36.0 DMIPS at 60 MHz, 0.600 DMIPS/MHz (user mode, copyback caches, in simulation) |
| Whetstone | 469,922 cycles/loop, 12.77 MWIPS at 60 MHz |
| cputest 68040 corpus | 3,778 of 3,801 slices (sampled); the rest are known WinUAE generator defects (doc/STATUS.md) |

## Design

```
 F0  F1  F2  F3 | D1    D2   | AG   DC1  DC2  EX   WB
 PC  I$  tbl pre| parse useq | EA   D$   D$   ALU  commit
```

* In-order pipeline that completes one instruction per cycle at best.
  Complex instructions are run by a microcode sequencer (652 microwords
  of 120 bits in a 1024-entry M10K ROM).  The predecoder reads a table in block RAM (8192 x 32, two
  read ports) instead of a pattern decoder in logic; the pattern decoder
  remains as the simulation reference, checked on every word.
* Branch prediction:
  * In D1: a 256-entry table of 2-bit counters over the static rule, JSR/JMP
    targets, and an 8-entry return stack.
  * In fetch: a 256-entry branch target buffer and a return stack of its
    own, for one-bubble taken branches, calls and returns.
* Caches: 4 KB instruction and 4 KB data, 4-way, 16-byte lines, physically
  tagged.
  * The data cache supports copyback and write-through, a push buffer, and
    store-to-load forwarding; a bus error on a later beat of a fill takes no
    exception unless the operand needs it (7.6.1); a cache-inhibited or
    locked access hitting a dirty line pushes it first (4.6.2); a disabled
    cache is bypassed and left alone; reset leaves the caches alone.
  * Both caches snoop SC 01/10 transfers of another master as in
    MC68040UM tables 4-3/4-4.
* MMU:
  * 64-entry 4-way instruction and data ATCs.
  * A hardware table walk with 4 KB and 8 KB pages and indirect
    descriptors; descriptors read through the data cache without
    allocating (3.2.5), updated with locked read-modify-write.
  * The TTRs, PTEST/PFLUSH, and nonresident ATC entries.
* Exceptions follow the 68040's restart model: access-error frames are
  format $7, MOVEM continues after a fault (SSW CM), a double bus fault
  halts the processor (7.6.3), BKPT runs its acknowledge cycle, T0/T1
  trace.
* FPU:
  * The 68040's hardware subset; unimplemented instructions and data
    types trap with the frames an FPSP expects.
  * Divide and square root produce two bits per clock; the 64x64 multiply
    runs in DSP blocks.
  * FSAVE/FRESTORE frames use revision $41, or $40 when `FPU_REVISION`
    selects it.

[doc/ARCHITECTURE.md](doc/ARCHITECTURE.md) describes the implementation.
[doc/STATUS.md](doc/STATUS.md) lists the milestones, the regression suite,
the area and timing rounds with their measurements, and the behaviour
decisions with the evidence for each.

Not provided: the multiplexed bus mode that CDIS selects at reset, DLE, and
JTAG.

## Top level

`rtl/ap68040.sv`, module `ap68040`:

* `clk` is PCLK.  `bclk_en` marks the processor clocks that end a bus
  clock: tie it high for a 1:1 bus, or toggle it for the 68040's 2:1.
* Bidirectional pins are split into `_i`/`_o`/`_oe`.
* `a_i`, `ts_n_i`, `rw_n_i`, `siz_i` and `tt_i` are the bus as another
  master drives it, for snooping.
* Parameters: `FPU_REVISION` ($41), `FPU_PIPE` (0), `LOOP_PRED` (0),
  `EARLY_RES` (0), `DC1_ATC` (1), `SKID` (0), `DIV_STEPS` (2); their
  costs are in doc/STATUS.md.

## Building and testing

The tools are Verilator 5, vasm/vbcc (`vasmm68k_mot`, `vbccm68k`) and, for
the fits, Quartus 17.0.

```
tb/run_tests.sh                 # 29 programs x 5 bus configurations, and the decode bench
tb/build_cputest.sh             # the cputest replay bench
tb/run_cputest.py data040.zip --full --sample 300:24 --jobs 12   # the corpus, sampled
tb/run_bench.sh                 # Dhrystone and Whetstone (vbcc)
syn/fitcore.sh <name>           # an out-of-context fit (PERIOD=16.667 SEED=<n> R=<rtl snapshot>)
```

Each program runs under these five bus configurations:

* zero wait states;
* random wait states;
* TBI with TA/TEA retries;
* BCLK at half of PCLK;
* all of these at once.

The bench (`tb/tb_ap68040.sv`) also provides:

* a 68040 bus slave;
* an alternate master that drives snooped traffic;
* bus error, TCI, IPL, reset and halt injections (doc/STATUS.md lists the
  registers);
* checks of PST, of the reset timing, of the halted processor's bus;
* `+prof`, a report of where the cycles go.

The microcode ROM, the decoder tables and the predecoder table in
`rtl/gen/` are generated by `tools/ucode.py`.  `tools/check_isa.py` checks
the decoder against WinUAE for all 65536 operation words.

## References

The references, in order of authority:

1. Real 68040 behaviour where it is known.
2. WinUAE: its CPU emulation and its cputest 68040 corpus.
3. The MC68040 User's Manual.

## License

GPL v2 or later; see [LICENSE](LICENSE).

Copyright (C) 2026 Adam Polkosnik
