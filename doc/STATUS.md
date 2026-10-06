# AP68040-60 status

## Milestones

| | milestone | state |
|---|---|---|
| M0 | bus unit, 68040 bus slave model, Verilator bench | done |
| M1 | pipeline skeleton: fetch, D1, D2, AG/DC/EX/WB, bus loads/stores | done |
| M2 | integer ISA (t_integer) | done |
| M3 | exceptions, interrupts, trace (t_exceptions) | done |
| M4 | instruction and data caches, copyback, bus snooping (t_cache, t_snoop) | done |
| M5 | MMU: ATCs, table walk, TTRs, PTEST/PFLUSH (t_mmu, t_atcprobe) | done |
| M6 | FPU (t_fpu, t_fpu_frames, t_fpu_resume) | done |
| M7 | cputest corpus replay (tb_cputest.sv) | done: only the 25 known generator artifacts fail |
| M8 | 60 MHz timing closure on 5CSEBA6U23I7 | done: out of context (Quartus 17.0, HIGH PERFORMANCE), the default seed gives worst setup slack +0.547 ns at 16.667 ns, hold and pulse width met in all four corners; 26,399 ALMs, 18,195 registers.  Seed 2 also closes (+0.093 ns); seed 3 misses the slow -40C corner by 0.062 ns |
| M9 | performance: prediction, BTB, return stacks, store forwarding, early redirect | Dhrystone 2.1: CPI 1.50, about 35.0 DMIPS at 60 MHz |
| M10 | the remaining 68040 pins: IPEND, PST, CDIS, MDIS (t_pins) | done |

## Regression

```
tb/run_tests.sh                 # the nineteen programs, five bus configurations each
tb/build_cputest.sh             # the corpus replay bench
tb/run_cputest.py ~/Downloads/data040.zip                    # smoke slices
tb/run_cputest.py ~/Downloads/data040.zip --full --group AE  # a group, every slice
```

Bus configurations: zero wait, random waits, waits with TBI and TA/TEA
retries, BCLK at half PCLK, and all of those together.

Programs: smoke, t_integer, t_exceptions, t_mmu, t_cache, t_atcprobe,
t_bitfield_cache, t_bitfield_mmu, t_movem_restart, t_moves_fc, t_fpu,
t_fpu_frames, t_fpu_resume, t_snoop, t_btb, t_stld, t_pins, t_snstress,
t_eredir.

Performance: `tb/build_c.sh dhry` (vbcc; `ASFLAGS=-DCOPYBACK=1` runs it in
user mode with copyback caches), then `obj/obj_prog/tb_ap68040
+prog=tb/build/dhry.hex +prof`: cycles and instructions between the
program's stamps, and where the cycles went (stage holds, redirects by
cause, BTB checks, load fast-path misses, DMU engine jobs).  `+pcprof`
adds, by instruction address, the cycles since the previous instruction
completed, the uops and the mispredictions, and the AG interlock cycles by
producer (stage, load or not, op).  `+ftrace=<cycle>` prints the pipeline
by instruction address, one line a cycle (`+ftto=<cycle>` ends it).

Dhrystone history (cycles for the 2000 runs between the stamps):
2,475,417 with the two-entry record FIFO (D2 idled one cycle in three);
2,015,434 with three entries and the return stack and BHT repaired;
1,997,442 with load data handed to AG at DC2; 1,985,428 with mispredicts
redirecting from EX; 1,951,441 with one-uop JSR.  What is left: about
12,000 mispredictions (loop exits), load- and ALU-to-address interlocks,
two-uop memory-to-memory MOVE, MOVEM one register a cycle.  A 512-entry
BTB removes 8,000 D1 redirects (2 %), but failed timing by 0.44 ns
(placement of the D1 consume path); not adopted.

Timing: seeds move the worst path by about +-0.5 ns, so a change is judged
on three seeds.  The paths that limited the seeds in this round, and what
cut them: the FPU normalizer's input select (registered with the state),
the fetch F1 ITT match (formed in F0), the D1 BTB target check (compares
the displacement with bt_tgt - qpc - 2, no adder), JSR_K chosen as the
record enters the FIFO (off the return stack path), and the ALU's taken
kept off EX's operand forwarding.

Known corpus failures (WinUAE generator defects, a real 68040 fails them
too): BasicFPU FADD.L/0001, FNEG.B/0002, FSNEG.S/0002, FSNEG.X/0007;
ODD_EXC CHK/DIV*/TRAPV and ODD_IRQ EXT/SWAP slices (21).  Any other
failing slice is a regression.

## Running one program

```
tb/build_asm.sh smoke t_integer      # assemble tb/asm/*.s
tb/build_sim.sh                      # verilate (objects in ./obj)
obj/obj_prog/tb_ap68040 +prog=tb/build/smoke.hex [+trace] [+ptrace] [+amtrace] [+prof] [+waits] [+tbi=2] [+retry=5] [+bclk2]
tb/where.sh t_mmu 172                # the source of a failing test number
```

The decoder table is checked against WinUAE for all 65536 operation words:
`python3 tools/check_isa.py <op040.txt>` (the oracle dump is produced by a
small program linked against WinUAE's readcpu.cpp; see tools/README).

## Behaviour notes (decided by tests or the oracle)

* Odd change-of-flow targets: address error at the instruction, frame PC
  per gencpu (JMP +2/+6, JSR the target, handler fetch the vector offset),
  Bcc/DBcc validate the target taken or not, RTE commits its SR first.
* Interrupts: the mask an SR-writing instruction sets decides at its own
  boundary; IPEND keeps a request that beat the old mask; an interrupt
  beats a simultaneous trace (delivered at the handler entry); a MOVEM
  continuation (SSW CM) runs before a pending interrupt.
* Fetch faults: a fault on the lookahead is refetched once on demand
  before it becomes an access error; FA is the faulting word's address.
* Memory bit fields touch only the bytes of the field (WinUAE).
* MOVEM: a loaded base/index register commits after the last transfer;
  indexed and PC-relative modes stack CM and the EA, RTE continues.
* MOVES translates through its SFC/DFC space; IACK is never translated.
* Bus snooping (MC68040UM tables 4-3/4-4, 7.9): SC 01/10 transfers of
  another master are looked up in the data cache, the push buffer and a
  push queued for the bus; dirty data is supplied (MI held, the 68040
  answers with TA), SC 01 byte/word/long writes into a dirty line are
  sunk, other write hits and SC 10 read hits invalidate; the I-cache
  drops a line on any snooped write and on an SC 10 read.  MI is
  asserted whenever another master owns the bus except while memory may
  answer.  The bench's alternate master (tb/m68040_alt_master.sv) is
  also the arbiter; t_snoop checks every case.
* Bus arbitration: a negated BG gives the bus up after the transfer in
  progress (a locked sequence keeps it).
* Reads may pass earlier writes (7.7) except: a cache-inhibited or locked
  read waits for older stores, and a serialized-page or locked read is
  performed only once every older instruction has completed, so nothing
  can restart it after the read.
* FDIV/FSQRT: two quotient bits (root digits) per clock, 33 iterations.
* Branch prediction is invisible to programs except for self-modified
  code: like the 68040 (which prefetches both paths of a branch) the core
  may fetch a branch target before an older store to it; CPUSHA must
  precede modified code (MC68040UM 4.5).  A store into the 128 bytes after
  its own instruction still refetches what follows.  A snoop that drops
  valid instruction-cache data refetches from the next instruction.
* CDIS's reset-time selection of the multiplexed bus mode is not provided.
