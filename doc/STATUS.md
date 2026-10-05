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
| M7 | cputest corpus replay (tb_cputest.sv) | FPU groups: only the 4 known generator artifacts fail; integer groups running |
| M8 | 60 MHz timing closure on 5CSEBA6U23I7 | in progress: 26.1k ALMs, -3.7 ns (FPU divide) before the radix change |
| M9 | performance: BTB, early restart, posted writes | |

## Regression

```
tb/run_tests.sh                 # the fourteen programs, five bus configurations each
tb/build_cputest.sh             # the corpus replay bench
tb/run_cputest.py ~/Downloads/data040.zip                    # smoke slices
tb/run_cputest.py ~/Downloads/data040.zip --full --group AE  # a group, every slice
```

Bus configurations: zero wait, random waits, waits with TBI and TA/TEA
retries, BCLK at half PCLK, and all of those together.

Programs: smoke, t_integer, t_exceptions, t_mmu, t_cache, t_atcprobe,
t_bitfield_cache, t_bitfield_mmu, t_movem_restart, t_moves_fc, t_fpu,
t_fpu_frames, t_fpu_resume, t_snoop.

Known corpus failures (WinUAE generator defects, a real 68040 fails them
too): BasicFPU FADD.L/0001, FNEG.B/0002, FSNEG.S/0002, FSNEG.X/0007;
ODD_EXC CHK/DIV*/TRAPV and ODD_IRQ EXT/SWAP slices (21).  Any other
failing slice is a regression.

## Running one program

```
tb/build_asm.sh smoke t_integer      # assemble tb/asm/*.s
tb/build_sim.sh                      # verilate (objects in ./obj)
obj/obj_prog/tb_ap68040 +prog=tb/build/smoke.hex [+trace] [+ptrace] [+amtrace] [+waits] [+tbi=2] [+retry=5] [+bclk2]
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
