# AP68040

A from-scratch MC68040-compatible CPU core in Verilog, with MMU, FPU and
split instruction/data caches. Developed inside the
[Minimig-AGA_MiSTer](https://github.com/apolkosnik/Minimig-AGA_MiSTer) fork,
extracted here so it can be pulled into other projects as a submodule.

It boots NetBSD/amiga and AmigaOS 3.x on real hardware (DE10-Nano), runs the
Amiga demo and application software that exercises the 040's cache, MMU and
FPSP paths, and passes 3,776 of the 3,801 slices of the WinUAE `cputest`
68040 corpus. The remaining 25 are documented generator defects, not core
failures — the corpus records expectations no real MC68040 can satisfy; see
`doc/CPUTEST_UPSTREAM_REPORT.md`.

## What is here

This tree currently holds **two** cores, mid-transition from one to the
other -- see `AP040_IMPLEMENTATION_PLAN.md` for the full status and roadmap.

```
rtl_old/                the WORKING reference core (what "Integrating it"
                        below describes) -- a sequential multi-cycle FSM
  ap040.qip             Quartus file list, in dependency order
  ap040_defs.svh        shared defines (every .v includes this)
  ap040_tg68k_compat.v  top level: TG68K-shaped port set
  ap040_core.v          sequencer, decode, EA engine, exception logic
  ap040_alu.v           ALU and shifter
  ap040_muldiv.v        multiply / divide
  ap040_regfile.v       register file
  ap040_fpu.v           FPU (extended precision, FPSP trap path)
  ap040_mmu.v           MMU: ATCs, TTRs, hardware table walker
  ap040_cache.v         4-way split I/D cache
  ap040_bus16_adapter.v 32-bit core to 16-bit host bus
  ap040_bus_timeout.v   bus watchdog
  ap040_walker_cdc.v    table-walk port clock crossing
  primitives/dpram.v    inferred true-dual-port RAM -- substitutable
rtl/                    the IN-PROGRESS pipelined replacement (not yet wired
                        into anything -- built and tested standalone)
  ap040_pipe_core.v      top level: six-stage IF/ID/EA-calc/EA-fetch/EX/WB
  ap040_inst_fetch.v     IF
  ap040_decode.v         ID
  ap040_ea_calc.v        EA-calc
  ap040_ea_fetch.v       EA-fetch (register forwarding lives here)
  ap040_execute.v        EX (condition checks, misprediction recovery)
  ap040_writeback.v      WB
  ap040_pipe_alu.v       ALU (forked from rtl_old/ap040_alu.v)
  ap040_pipe_regfile.v   register file (forked from rtl_old/ap040_regfile.v)
  ap040_pipe_defs.svh    shared defines, independent copy of ap040_defs.svh
tb/                     self-contained test suites for BOTH cores (see Testing)
doc/
```

`rtl_old/` and `rtl/` are deliberately independent -- deleting either cannot
affect the other (no shared includes, no shared module names).

## Integrating it

This section describes `rtl_old/` -- the working core. (`rtl/`'s pipelined
replacement isn't ready to integrate anywhere yet; see
`AP040_IMPLEMENTATION_PLAN.md`.)

Add `rtl_old/ap040.qip` to a Quartus project, or hand the eleven
`rtl_old/*.v` files plus `rtl_old/primitives/dpram.v` to any other flow —
`ap040_defs.svh` must be on the include path (`-I rtl_old` for iverilog).

The top level is `ap040_tg68k_compat`, which presents a TG68K-shaped port set
so it can drop into a host that already speaks that interface:

```verilog
ap040_tg68k_compat #(
    .AP040_HAS_MMU     (1),
    .AP040_HAS_FPU     (1),
    .AP040_ENABLE_CACHE(1)
) cpu (
    .clk        (clk),
    .nreset     (nreset),
    .clkena_in  (cpu_enable),      // stall the core by holding this low

    .data_in    (cpu_din),         // 16-bit host bus
    .data_write (cpu_dout),
    .addr_out   (cpu_addr),        // 32-bit
    .nwr (nwr), .nuds (nuds), .nlds (nlds),
    .busstate   (busstate),        // 0 fetch, 1 idle, 2 read, 3 write
    .longword   (longword),
    .fc         (fc),
    .ipl (ipl), .ipl_autovector (1'b1), .berr (berr),
    .nresetout  (nresetout),       // the RESET instruction
    ...
);
```

Three groups of ports are optional and can be tied off:

- **`walker_*`** — the hardware table walker's own memory port. Give it a
  path to RAM; a walk that never gets `walker_ack` is turned into a bus
  error by the watchdog rather than hanging.
- **`cache_snoop_stb` / `cache_snoop_addr`** — DMA write snoop, already in
  the core's clock domain. Without it the data cache cannot see writes made
  by other bus masters. The `cache_z2_*`/`cache_z3_*` inputs describe which
  physical windows are cacheable at all; `cache_allow_all` bypasses them for
  flat simulation environments.
- **`mmu_*`, `cacr_out`, `vbr_out`, `debug_*`** — observation only.

`dpram` is a plain inferred true-dual-port RAM. Replace it with a vendor
macro (altsyncram, XPM) if your flow needs one; the ports are
`clock, address_a, data_a, wren_a, q_a, address_b, data_b, wren_b, q_b` with
`AW`/`DW` parameters.

### One thing to get right

The core is a **restart-model** 68040: on a write fault the handler repairs
the mapping and the faulting instruction re-executes. It deliberately never
advertises a valid WB3 in its access-error frame. A host OS that completes
valid writeback slots itself — NetBSD's `trap.c` does — would otherwise
double-apply the store of an RMW instruction.

## Testing

```
cd tb && ./run_tests.sh          # rtl_old (reference core): needs iverilog and vasmm68k_mot (vbcc)
cd tb && ./run_pipe_tests.sh     # rtl (pipeline, in progress): needs only iverilog
```

Everything under `tb/` runs against a core alone, with no host-project
sources, so a failure is the CPU's rather than an integration artifact.
`run_tests.sh` covers the integer ISA, the exception and trace model, the MMU
(translation, TTRs, page-table walks, access faults, 4K and 8K pages), the
caches, and the FPU -- against `rtl_old`. `run_pipe_tests.sh` covers whatever
the pipeline has reached so far (see `AP040_IMPLEMENTATION_PLAN.md` for the
milestone list); each `tb_ap040_pipe_*.v` bench pokes a tiny program directly
into the fetch stage's ROM, so it needs no assembler.

The Minimig-AGA tree adds further benches that co-simulate the core against a
real Amiga chipset, an SDRAM controller and a DDR3 controller, and drives the
WinUAE corpus and per-instruction differentials against WinUAE's own softfloat
and `cpummu`. Those live there because they need those modules.

## Status and provenance

Developed against three references, in this order of authority: real hardware
first, then WinUAE as the executable oracle, then the Motorola manuals. Where
the manual and WinUAE disagree the reference wins — several documented
"deviations" in this core turned out to be manual misreadings, and the
comments record which oracle settled each one and where.

## License

GPL v2 or later — see [LICENSE](LICENSE). The core was written as part of
Minimig-AGA_MiSTer, which is distributed under the same terms.

Copyright © 2026 Adam Polkosnik
