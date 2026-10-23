# AP68040 on Xilinx

Measurements and findings from the first Vivado run of the `faster` core,
2026-10-18.  Vivado 2023.2, `xc7a100tfgg676-2`, out of context unless stated.

Everything here is out of context at 45 % LUT and 53 % slice occupancy, where
the tool had the whole die to spread over and no area or timing pressure.  The
shipping MinimigAGA_TC64 image, for contrast, sits at 48,471 LUTs (76 %) and
**92.79 % slice occupancy**, `clk_38` +0.444 ns, `clk_114` +0.205 ns, 70 of 135
BRAM tiles, 28 of 240 DSPs.  OOC numbers rank candidates; only an in-context fit
accepts one.

## 1. The fit

Default directives, all core parameters at their defaults, 16.667 ns,
synth -> opt -> place -> phys_opt -> route.

| | |
|---|---:|
| Slice LUTs | 28,721 (45.3 %) -- 27,833 logic + 888 memory |
| Slice registers | 11,498 (9.1 %) |
| Slices | 8,422 (53.1 %), 3.41 LUTs per slice |
| BRAM tiles | 29 (21.5 %) -- 17 RAMB36 + 24 RAMB18 |
| DSP48E1 | 20 (8.3 %) |
| Setup WNS | **+1.314 ns** |

Worst routed path `be/wb_v_reg` -> `dmu/e_ret_reg[1]/CE`: 15 logic levels,
**2.494 ns logic + 12.616 ns routing** -- 83 % route.  Scalar OOC Fmax =
1/(16.667 - 1.314) = **65.1 MHz**.

Hold: the report says constraints are not met, but every violation (-0.367 ns)
is an input-port path `a_i[*]` -> `snoop/dc_pa_reg[*]`, an artifact of the 0 ns
input delay in the probe XDC.  Register-to-register hold has no failing
endpoints.

Reproduce with an OOC synth of the `syn/fitcore.sh` file list,
`-mode out_of_context`, one `create_clock -period 16.667`, and
`report_utilization -hierarchical`.

## 2. The three memory-template failures

Every memory in the core is behavioural and every multiplier is an inferred
`*` -- no vendor primitive is instantiated anywhere.  What does not port are the
*attributes*: `ramstyle`, `romstyle` and `altera_attribute` are Quartus-only, so
Vivado maps on its own defaults and gets three arrays wrong.

| variant | LUTs | FFs | BRAM tiles |
|---|---:|---:|---:|
| as delivered | 57,405 | 65,297 | 5.5 |
| `ap68040_tdp` split + `rom_style` added | 47,698 | 59,365 | 10.5 |
| all three templates fixed | **28,721** | **11,498** | **29** |

The register count is the tell -- 65,297 flip-flops against 11,498, and 12,500
in the Cyclone V fit.  The first two rows are synthesis estimates only; neither
was implemented, because they put 57,405 and 47,698 LUTs on a 63,400-LUT part.

| array | shape | Vivado as written | cause |
|---|---|---|---|
| `ap68040_tdp` -- D-cache tags | 64x22, x4 ways | flip-flops | both writes in one `always_ff`; Vivado needs one write per process for a true dual port template |
| `ap68040_sdp_be` -- I$ data and tags, D$ data | 256x32, 64x24, 64x128, x4 ways each | flip-flops | one `always_ff` **per byte lane**; all writes and the read must share one process |
| `ap68040_pdrom` -- predecoder table | 8192x30, two read ports | LUT logic | two reads in one process; `rom_style = "block"` alone does not fix it |
| `urom`, `attr_rom` | 1024x120, 512x90 | LUT until `rom_style` added | inferred correctly once the attribute is present |

Fixes, all template shape and no semantic change:

* `ap68040_tdp` (`rtl/ap68040_ram.sv:57-100`) -- one write per `always_ff`.
  Read-first per port preserved; the explicit write bypass
  (`q_a`/`qa_bv`/`qa_bd`) untouched.  Vivado then reports "recognized as a true
  dual port RAM template".
* `ap68040_sdp_be` (`rtl/ap68040_ram.sv:14-55`) -- all byte writes **and** the
  read in one process; the packed `[SL-1:0][7:0]` flattened to `[SL*8-1:0]`.
* `ap68040_pdrom` (`rtl/ap68040_ram.sv:192-209`) -- one read per process, plus
  `rom_style = "block"`; the same attribute on `urom`
  (`rtl/ap68040_useq.sv:844`) and `attr_rom` (`rtl/ap68040_decode.sv:245`).

Keep the Quartus attributes alongside the Vivado ones -- each tool ignores the
other's.

What maps cleanly with no work: MLAB -> distributed RAM (`RAM32M`, `RAM64M`,
`RAM128X1D`), the three-write-port register file's live-value-table trick
included; the BTB's 256x56 to one RAMB36 each; all five 32x32 multiplies and the
FPU's 64x64 into **20 DSP48E1**, the cascaded `(PCIN>>17)+A2*B2` chain included.

One Quartus-only attribute remains:
`(* altera_attribute = "-name MAX_FANOUT 14" *)` on `atc_set_q`
(`rtl/ap68040_dmu.sv:507`).  Vivado ignores it; not a problem in this fit.

## 3. Per-module area

`report_utilization -hierarchical` on the routed checkpoint.

| module | LUTs | FFs | BRAM | DSP |
|---|---:|---:|---|---:|
| `be` total | 15,956 | 5,809 | -- | 20 |
| -- `be` proper (AG/EX/WB/interlocks) | 3,918 | 3,129 | -- | -- |
| -- `fpif` + `fpu` | 8,653 | 2,345 | -- | 16 |
| -- `md` muldiv | 908 | 239 | -- | 4 |
| -- `rff` front RF (3W x 5R) | 1,666 | 64 | -- | -- |
| -- `rfb` back RF (2W x 5R) | 833 | 32 | -- | -- |
| `dmu` | 5,622 | 2,336 | 4x36K + 16x18K | -- |
| -- `datc` | 812 | 130 | -- | -- |
| `fetch` | 3,387 | 919 | 9x36K + 7x18K | -- |
| -- `iatc` | 119 | 130 | -- | -- |
| `useq` | 1,848 | 675 | 3x36K + 1x18K | -- |
| `dec` | 1,288 | 1,208 | 1x36K | -- |
| `biu` | 436 | 201 | -- | -- |
| `snoop` | 194 | 318 | -- | -- |

`fpif` + `fpu` is **30.1 % of the core**.  The `iatc`/`datc` asymmetry (119
against 812) is Vivado pruning the I-ATC's second read port, which is tied off
at `rtl/ap68040_fetch.sv:271-281` -- worth remembering, because a refactor that
merges ATC read ports can destroy that pruning and make the I-ATC grow by about
290 LUTs.

## 4. Standalone probe fits

Same part and flow, each module synthesized alone.

| module | LUTs | FFs |
|---|---:|---:|
| `ap68040_alu` | 609 | 0 (combinational) |
| `ap68040_alu_slow` | 818 | 0 |
| `ap68040_rf` 2W x 5R | 551 | 32 |
| `ap68040_rf` 3W x 5R | 784 | 64 |
| `ap68040_rf` 2W x 7R | 731 | 32 |
| `ap68040_rf` 3W x 7R | 1,028 | 64 |

Derived: **+233 to +297 LUTs per write port**, **+180 to +244 per read-port
pair** -- about 41 to 47 LUTs per added bank.  Cost tracks the NW x NR bank
count exactly.  Note `be` already instantiates three `ap68040_alu` (`alu`,
`cas_alu`, `cmp_alu`), so a further one is a known 609 LUTs.

## 5. Cyclone V and Artix-7 are the same size

Comparing 19,637 ALMs to 28,721 Slice LUTs makes the Xilinx build look 40 %
larger when it is not.  STATUS.md's own option table gives both Altera figures:
**19,869 ALMs = 28,088 ALUTs**, a ratio of 1.414.

| metric | Cyclone V | Artix-7 |
|---|---:|---:|
| logic elements (ALUTs / LUTs) | 28,088 | 27,833 |
| occupied sites (ALMs / LUT6 sites) | 19,869 | 27,833 |
| registers | 12,500 | 11,498 |
| device fraction | 47 % of 41,910 ALMs | 45 % of 63,400 LUTs |

An ALM is a *pair* site holding one 6-input function or two independent
5-input-or-smaller functions; a Slice LUT is a *single* LUT6 site, which pairs a
second function only through O5/O6 and only when it shares all its inputs.
Vivado achieved that on **4,002 of 27,833** sites, so in equation terms Xilinx
has 23,829 + 2x4,002 = about **31,835** against Altera's 28,088 -- a **13 %
residual**, attributable to the looser fracturing rules.  Not to adders (equal
in element count) and not to wide muxes: only 459 MUXF7 and 12 MUXF8 are used of
31,700 and 15,850, so the dedicated mux resources are headroom rather than a
penalty.

**Track ALUTs against LUT-as-Logic across the two tools, never ALMs against
Slice LUTs.**

## 6. Three negative results

Each kills an idea that looks good until measured.

**`RAM32M` cannot share stored bits across read addresses.**  It is 32x8 of
storage arranged as four independent 32x2 memories sharing one write address and
one `WE` -- separate `DIA`/`DIB`/`DIC`/`DID` and separate
`ADDRA`/`ADDRB`/`ADDRC`/`ADDRD`, with `ADDRD` doubling as the write address.  A
LUT6 in 32x2 mode presents two bits at one address.  Serving R read addresses
over W bits costs W x R / 2 LUT6 however it is expressed; UG474 gives
`32 x 2 quad-port = 4 LUTs` and `32 x 6 SDP = 4 LUTs`, the same density.  So the
register file's NW x NR bank replication (`rtl/ap68040_ram.sv:126-156`) is
**not** an Altera artifact -- it is the floor on both vendors, and `RAM32M x6`
per bank in Vivado's report is the optimum, not an opportunity.  The probe fits
in section 4 are the local proof: 5R -> 7R at fixed NW cost +180 and +244, where
sharing would have cost about zero.  Residual packing waste across `rff`, `rfb`
and `datc` is about **76 LUTs, 0.26 %** -- not worth touching the only module
with subtle write-port priority and read-during-write semantics.

**No SRL32 candidates (zero used in the fit).**  A property of the design, not
an oversight.  The 8-word fetch queue consumes 0 to 5 words per cycle
(`rtl/ap68040_decode.sv:836`) with all eight slots read in parallel
(`rtl/ap68040_fetch.sv:184-192`); the predecode ring does not shift at all,
being two independent mod-8 pointers moving by different variable amounts
(`rtl/ap68040_fetch.sv:149-156,178,580-587`); both return stacks write at an
arbitrary depth (`ras[ras_tp + 3'd1]`, `rtl/ap68040_decode.sv:892`) and pop
without destroying.  The back end's `upd`/`upd2` chains
(`rtl/ap68040_backend.sv:1705,1729,1754,1791`) are structurally ideal 3-clock
delays but each hop has its own enable under bubble collapsing.  The only clean
untapped delay of depth 3 or more is `p1_op` -> `p5_op` in the FPU
(`rtl/fpu/ap040_fpu.v:647`) -- 28 FFs, and absent at the default
`FPU_PIPE = 0`.  Flip-flops are at 9 % of the part anyway, so this would trade
an abundant resource for the contended SLICEM sites the 888 LUTRAM bits need.

**DSP48E1 does not fit the shifter or the adders (20 of 240 used).**  The funnel
is a single **66-bit** variable right shifter with a clean 7-bit amount
(`rtl/ap68040_alu_slow.sv:180`).  DSP48E1's 25x18 multiplier and 48-bit ALU are
the wrong shape: roughly eight cascaded DSPs plus a 67-bit one-hot decoder, with
more cascade delay than the LUT form.  It is also already off the critical path
by design -- C and X come off the operand by bit index and N/Z from the
registered result (`rtl/ap68040_alu_slow.sv:201-230`, `:266`;
`rtl/ap68040_backend.sv:1169-1176`).  AG's adders appear nowhere in STATUS.md's
timing section, are protected by a design rule (ARCHITECTURE.md:375-377), and
have the index scale applied a stage earlier at the register load
(`rtl/ap68040_backend.sv:409-411`).

*Doc correction found while investigating:* ARCHITECTURE.md:169-170 and the
`ap68040_alu_slow.sv` header both claim two funnels -- one right, one left.  The
RTL has **one** 66-bit right shifter; LSL/ASL go through it with the operand
pre-padded by 31 zeros and the amount complemented, and `lo = ro` is a plain
alias.

## 7. Dual issue, and the Apollo 68080

### What a second integer pipe would cost

Costed from the measurements above rather than estimated.  68060-style
restricted pairing -- slot 1 register-to-register only, no memory operand, no
shifter, no branch, no microcode, no FP -- leaves `dmu`, `fetch`, `biu`,
`snoop`, `fpif`/`fpu` and `md` (19,200 LUTs, two-thirds of the core) untouched.

| item | LUTs | basis |
|---|---:|---|
| second simple ALU | +609 | measured |
| front RF 3W5R -> 4W7R | +1,150 | measured deltas, scaled in-context |
| back RF 2W5R -> 3W7R | +720 | same |
| D1 dual decode: ring select at head+len0, slot-1 attribute table back **in logic** (the ROM is read a cycle ahead and cannot serve slot 1), second PC-relative adder, pairing legality | +820 to +1,450 | |
| D2 direct uop formatter + 2-wide FIFO | +700 to +1,200 | `uop_t` is 292 bits; a serial microcode ROM cannot issue two |
| slot-1 AG/DC1/DC2/EX registers and muxes | +600 to +1,000 | about +1,500 FFs |
| hazard matrix | +500 to +900 | |
| dual retire, age-ordered CCR merge | +400 to +700 | |
| **total** | **+5,000 to +8,000** | 28,721 -> about 34,000 to 37,000 LUTs |

A **full** dual issue -- both pipes memory-capable -- adds **+2,700 to +4,100
LUTs, +4 RAMB36, +16 RAMB18** on top, and requires the ATC and data cache to be
designed for two lookups per cycle from the outset.

Against the incumbent: the Minimig's AP68040-pipelined core is **30,677 LUTs**
OOC at 37.81 MHz (WNS +0.451).  This core is already 1,956 LUTs *smaller* at
1.59x the clock -- but a dual-issue version lands about 10 % *larger* in the
expected case, and only ties in the most optimistic corner.

### The published FPGA comparison

Gokulan, Muraleedharan and Varghese, "Design of a 32-bit, dual pipeline
superscalar RISC-V processor on FPGA", DSD 2020: a 5-stage, two-way in-order
RV32IMAFD core on a **Virtex-7 XC7VX485T-2** -- faster fabric than Artix-7 -2 --
reaching **40 MHz**, 30,125 LUTs for the pipeline alone plus 10,631 (D-cache),
7,739 (I-cache) and 2,862 (register file).  Their pairing rules are already the
aggressive ones: the second instruction issues only with **no dependency** on
the first; two memory or two branch instructions never pair; load, branch,
multiply/divide and CSR go only through pipe 0.

Wider consensus from the same literature: 4-way superscalar Fmax runs about
33 % below single-issue; bypass and interlock logic is routinely *the* critical
path; out-of-order soft cores land at 25 to 50 MHz on older families.

### The Apollo 68080

Its clock is real -- **92 to 100 MHz** on Vampire V4 (Cyclone V 5CEFA7; special
editions about 107 MHz), 78 to 85 MHz on V2 (Cyclone III) -- and it is the
**full** dual-issue shape: two address-calculation engines and a dual MMU in the
data cache doing **two translations per cycle**, so both pipes do memory.

It bought that by removing what this core exists to provide:

| | AP68040 `faster` | Apollo 68080 |
|---|---|---|
| MMU | full 68040: 64-entry 4-way ATCs, hardware table walk, 4K/8K pages, indirect descriptors, TTRs, PTEST/PFLUSH, locked U/M updates | deliberately incompatible with 020/030/040/060; MMU instructions **not supported** |
| FPU | full 80-bit extended, FSAVE/FRESTORE per table 9-16 | 52-bit precision on Cyclone III boards; 64-bit on V4 |
| Bus | real 68040 pins: TS/TA/TEA/TBI/TCI, bursts, LOCK/LOCKE, arbitration, snooping with dirty-data supply, PST, IPEND | none; integrated with its own controller |
| Exception fidelity | format $7 restart model, MOVEM continuation, double bus fault halt, 3,778/3,801 cputest slices | no such obligation |
| Source | GPL | closed -- techniques not inspectable, claims self-reported |

The area is the proof: the entire Vampire V2 -- CPU *plus* AGA chipset, SAGA and
memory controller -- fits a Cyclone III EP3C40, about 40,000 LEs, roughly 16,000
ALM-equivalents, against 19,637 ALMs for this core alone.  A superscalar core
smaller than a scalar one is not doing the same work.

**The transferable lesson is architectural, not a trick.**  If memory operations
are ever to go in both pipes, the ATC and data cache must be dual-ported from
the start; it cannot be retrofitted onto a 68040-compatible ATC.  The existing
`DC1_ATC` parameter is already the early sign of that tension.  Keeping 68040
MMU compatibility and getting two translations per cycle are close to mutually
exclusive on this budget.

## 8. Fmax tests

### The question

Can a dual-issue version reach **56.72 MHz** -- the clock the Minimig CPU island
lands on (MMCM VCO 1134.375 / 20, ratio 2 against `clk_114`)?  Ratio 2 divides
the 16-phase SDRAM round, which is the property the Minimig's own hardware
measurements tie to clean chip RAM.

### The arithmetic

Dual issue puts a mux layer on all three paths STATUS.md names as nearest the
limit: EX flags -> `fw_ccr` -> `br_cc` -> `tr_go` -> `fin` -> ID consume -> IF
fetch PC; EX result forwarded into AG operands; and the register file's
live-value-table read.

| Fmax loss | dual-issue result | period needed | vs 17.637 ns available |
|---|---:|---:|---:|
| 15 % | 55.3 MHz | 18.07 ns | **short 0.44 ns** |
| 20 % | 52.1 MHz | 19.20 ns | **short 1.57 ns** |
| 25 % | 48.8 MHz | 20.49 ns | **short 2.86 ns** |

Inverted: **the scalar core must reach 66.7 MHz** (15 % loss) or **70.9 MHz**
(20 %) for dual issue to land on 56.72.  It is at 65.1.

### Two unmeasured levers

The baseline fit had **both of the repo's timing options off**, and neither has
ever been measured on Xilinx.

| option | default | what it does | measured effect (Quartus, STATUS.md) |
|---|---|---|---|
| `DC1_ATC` | 0 | DC1 gets its own ATC read port and compares tags against all four ATC ways' pages in parallel | three seeds moved from -0.17 / +0.30 / +0.38 to **+0.45 / +0.30 / +0.25**, for about 200 ALMs |
| `SKID` | 0 | a skid register between D2 and AG, aimed at the stall chain | not separately tabulated |

### Test 1 -- scalar Fmax headroom (the decisive gate)

OOC fits of `ap68040`, full flow to routed, one `create_clock`.  Sweep the
period down until WNS crosses zero rather than fitting once at 16.667 ns -- the
WNS at a loose constraint understates what the tool can do when pushed.

| run | `DC1_ATC` | `SKID` | periods |
|---|---|---|---|
| A (baseline) | 0 | 0 | 16.667, 15.4, 14.3 ns |
| B | 1 | 0 | same |
| C | 0 | 1 | same |
| D | 1 | 1 | same |

Record per run: Fmax at WNS = 0, total LUTs, slices, the worst path's
logic/route split and logic-level count, and the hierarchical LUT table.

* **70.9 MHz or above** -- survives a 20 % loss.  Proceed to Test 2.
* **66.7 to 70.9 MHz** -- survives only 15 %.  Marginal; proceed, but the
  in-context result is binding.
* **below 66.7 MHz** -- **stop.**  Dual issue cannot reach 56.72 MHz on this
  part.

Judge register-to-register only; input-port hold violations are XDC artifacts.

### Test 2 -- in-context confirmation

Drop the winning configuration into the real MinimigAGA_TC64 design as an
unconnected island on a 56.72 MHz `clk_cpu` -- ports tied off, outputs to a
`DONT_TOUCH` sink -- and route the whole design.  Report slice occupancy and
`clk_cpu` WNS.  If `clk_cpu` does not close with the **scalar** core in context,
dual issue is moot and the question closes for the price of one fit.

### Re-pipelining, if Test 1 falls short

Each changes cycle counts, so each needs the full regression.

1. **Take `br_cc` out of the redirect path** -- resolve the branch a cycle later
   and pay one more misprediction cycle.  The repo already did the special case
   (commit 9025b70, "apply TRAPcc's decision after stepf").
2. **Stop routing a global stall.**  Replace the `fin` -> ID consume -> IF
   room-check chain with credit-based flow control: a registered free-slot
   counter, so "may I send" is a register compare rather than a combinational
   chain back from the consumer.  Deepen the fetch queue so IF never sees ID's
   `consume` in the same cycle.  (Tseng and Asanovic: "Stalling in particular is
   usually prohibited on high frequency pipeline designs due to the difficulty
   of generating and routing a global stall signal to a large number of pipeline
   registers.")
3. **Compute Z and N in parallel with the EX result mux**, not after it -- an
   ALU restructure already named in STATUS.md.
4. **Conservative bypass bits** -- mark operands that will arrive from the
   bypass network so they never contend for a register-file read port.
   Read-port count is the direct lever: +180 to +244 LUTs per pair, measured.

### If the tests fail

Take the scalar core at 56.72 MHz: **1.50x** the shipping 37.8125 MHz for no
microarchitecture work, in a core already 1,956 LUTs smaller than the incumbent.
Dual issue, even if it reached 56.72, buys only IPC -- **1.10 to 1.25x** on
register-poor, memory-heavy 68k code -- for +5,000 to +8,000 LUTs on a die at
92.79 % slice occupancy.

## 9. Dual issue as built

Measured on the shipping configuration (`FPU_PIPE`, `LOOP_PRED`,
`EARLY_RES` 0), Dhrystone 2.1 and Whetstone by `tb/run_bench.sh` at 56.72
MHz, area by `syn/vivado_ooc.sh` (Vivado 2023.2, xc7a100t, 17.630 ns):

| step | Dhrystone cycles | DMIPS | LUTs | slices | BRAM tiles | WNS |
|---|---|---|---|---|---|---|
| single issue | 1,007 | 32.06 | | | | |
| + dual issue (`DUAL`) | 912 | 35.40 | 31,560 | 9,168 | 29 | +1.97 |
| + second-slot (Ay) loads | 892 | 36.19 | 31,396 | 8,878 | 29 | +2.39 |
| + BTB target words (`BTIC`) | 878 | 36.77 | 31,663 | 9,024 | 31 | +1.38 |
| + 8 KB caches (`CSB` 7) | 878 | 36.77 | 31,747 | 8,876 | 31 | +2.05 |

* In total 12.8 % fewer Dhrystone cycles (1.15x per clock); 2.06x the
  incumbent pipelined core's 17.82 DMIPS in the Minimig.  Whetstone is
  flat (about 581,600 cycles a loop): FPU work issues alone.
* Random generated programs (`tb/run_dualrand.sh`, 400 seeds): 6.4 %
  fewer cycles with dual issue, every checkpoint identical to the
  single-issue core.
* Dhrystone and Whetstone fit 4 KB caches, so the 8 KB caches show
  nothing there; they cost no block RAM (the 4 KB arrays used half of
  theirs) and about 84 LUTs.  CINVA/CPUSHA scan twice the sets.
* Tried and dropped: d16(An) loads in the second slot (+369 LUTs, +227
  slices, no Dhrystone gain: those loads follow another memory access);
  branch folding.  A second data cache port is the next lever for
  loads (estimated +1.5 to 2 % Dhrystone for 2,000 to 3,000 LUTs).
* The BTB target words' first version kept their valid bits in flip-
  flops: +1,654 LUTs.  In LUT RAM with a clearing sweep: +267.

Verification: `tb/run_tests.sh` 175/175 (with `t_ld1`, `t_btic`);
`tb/run_dualrand.sh` 400 seeds; the WinUAE cputest corpus fails only
the 21 slices the scalar core fails too (13 ODD_EXC trace, 8 ODD_IRQ
EXT/SWAP), checked through the second-slot loads; the BTIC and 8 KB
builds' corpus runs were not complete at the time of writing.

## Verification status

**One item is unverified and must not be glossed.**  The memory-template
rewrites in section 2 were not proven behaviour-neutral: `tb/build_asm.sh:5`
wants `/opt/amiga-cc/vbcc/bin/vasmm68k_mot`, absent on the measuring machine.  A
substituted older `vasmm68k_mot` produced binaries that never reach their first
check -- every program in `tb/run_tests.sh` failed identically at
`last test 0`.

A differential against **pristine** RTL with the same assembler failed
identically, same programs and same PCs (`t_mmu` at `pc=00001f4a`, `t_snoop` at
`pc=00000fba`), which exonerates the rewrites but is **not** a positive
correctness proof.

Before relying on them, install the correct vbcc and run:

```
tb/run_tests.sh                   # 30 programs x 5 bus configs + exhaustive decode
tb/run_cputest.py data040.zip --full --sample 300:24 --jobs 12
syn/fitcore.sh <name>             # Quartus, 3 seeds: the Altera fit must not regress
```

The corpus must fail exactly the 23 known WinUAE generator defects listed in
STATUS.md; any other failing slice is a regression.  Note that
`tb/build_sim.sh:9` runs `tools/ucode.py`, which regenerates `rtl/gen/*` --
confirm `git status rtl/gen` is clean after a build.

## Sources

* Gokulan, Muraleedharan and Varghese, "Design of a 32-bit, dual pipeline
  superscalar RISC-V processor on FPGA", DSD 2020.
* Tseng and Asanovic, "Banked Multiported Register Files for High-Frequency
  Superscalar Microprocessors", ISCA 2003.
* Palacharla, Jouppi and Smith, "Complexity-Effective Superscalar Processors".
* LaForest and Steffan, "Efficient Multi-Ported Memories for FPGAs" (FPGA 2010)
  and "Multi-Ported Memories for FPGAs via XOR" (FPGA 2012).
* Xilinx UG474, 7 Series CLB User Guide -- the distributed-RAM tables.
* Apollo Team wiki and apollo-core.com, for the 68080 figures (self-reported).
