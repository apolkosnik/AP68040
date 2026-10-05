# AP68040-60 status

## Milestones

| | milestone | state |
|---|---|---|
| M0 | bus unit, 68040 bus slave model, Verilator bench | done |
| M1 | pipeline skeleton: fetch, D1, D2, AG/DC/EX/WB, bus loads/stores | smoke test passes |
| M2 | integer ISA (t_integer) | in progress |
| M3 | exceptions, interrupts, trace (t_exceptions) | |
| M4 | instruction and data caches, copyback, snooping (t_cache) | |
| M5 | MMU: ATCs, table walk, TTRs, PTEST/PFLUSH (t_mmu) | |
| M6 | FPU (t_fpu, t_fpu_frames) | |
| M7 | cputest corpus replay | |
| M8 | 60 MHz timing closure on 5CSEBA6U23I7 | |
| M9 | performance: BTB, early restart, posted writes | |

## Running

```
tb/build_asm.sh smoke t_integer      # assemble tb/asm/*.s
tb/build_sim.sh                      # verilate (objects in ./obj)
obj/obj_prog/tb_ap68040 +prog=tb/build/smoke.hex [+trace] [+ptrace] [+waits] [+tbi=2] [+retry=5] [+bclk2]
```

The decoder table is checked against WinUAE for all 65536 operation words:
`python3 tools/check_isa.py <op040.txt>` (the oracle dump is produced by a
small program linked against WinUAE's readcpu.cpp; see tools/README).
