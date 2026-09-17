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

```
rtl/                    the core, and nothing else
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
tb/                     self-contained test suite (see Testing)
doc/
```

## Integrating it

Add `rtl/ap040.qip` to a Quartus project, or hand the eleven `rtl/*.v` files
plus `rtl/primitives/dpram.v` to any other flow — `ap040_defs.svh` must be on
the include path (`-I rtl` for iverilog).

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

`AP040_FPU_REVISION` selects the FPU state-frame ABI at elaboration time:
`8'h41` (default) or `8'h40` for older non-Turbo NeXT software. Revision
`0x40` uses a 44-byte unimplemented-instruction frame; `0x41` uses 52 bytes.
NULL frames remain four zero bytes; IDLE and 100-byte BUSY frames carry the
selected revision. FRESTORE rejects non-null frames from another revision.
This selects serialization/layout, not an alternative arithmetic datapath.

BUSY FRESTORE with `CU_SAVEPC=0xfe` resumes supported arithmetic commands
in opclass 0/2 using the frame's ETEMP and FPTEMP, including their extended
exponent bits. Completion uses the existing background-FPU interlock and
deferred arithmetic-exception handling. Other resume opclasses and
software-only opcodes are not newly implemented by this path.

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

MOVEM operand faults set SSW.CM and stack the original effective address.
RTE uses that address for indexed/PC-relative modes and replays the transfer
list without rereading a memory-indirect pointer that MOVEM may have changed.
Base/index load deferral remains in place. CT and WB2/WB1 are not implemented.

Failed MMU searches install nonresident ATC entries, as on a 68040. Repairing
a descriptor alone does not make it accessible: software must invalidate the
old entry (PFLUSH or PTEST), or wait for replacement. PTEST reports a table
bus error with MMUSR.B and also caches the failed translation.

## Testing

```
cd tb && ./run_tests.sh          # needs iverilog and vasmm68k_mot (vbcc)
```

`sh tb/run_fpu_frames.sh` independently checks both FPU revisions' headers,
payloads, pointer adjustments, frame round-trips, invalid-frame rejection
and BUSY-command resumption under all three bus-handshake phases.

The main runner also checks the shared arithmetic datapaths directly:
`tb_ap040_alu_arithmetic.v` exhausts byte ADD/ADDX/SUB/SUBX/CMP operands and
X/Z combinations, then checks word/long boundaries and seeded random inputs
against an independent arithmetic/CCR oracle. `tb_ap040_fpu_normalize.v`
checks all three normalization states, every leading-zero count, GRS bits,
operand tags, exponent wrap and clock-enable holding against a serial-shift
reference. These tests do not require guest software.

`tb_ap040_regfile.v` compares the integer register file with a flip-flop
reference through consecutive writes, clock-enable stalls, reset and all
three stack-pointer banks. A second leg poisons the pending RAM word to
check bypass isolation; disabling that bypass must fail the negative control.
This does not replace MLAB timing analysis or hardware boot testing.

Everything under `tb/` runs against the core alone, with no host-project
sources, so a failure is the CPU's rather than an integration artifact. The
suite covers the integer ISA, the exception and trace model, the MMU
(translation, TTRs, page-table walks, access faults, 4K and 8K pages), the
caches, and the FPU.

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
