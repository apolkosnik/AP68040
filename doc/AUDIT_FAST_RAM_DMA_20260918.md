# Fast RAM DMA: independent verification and executable reproduction

Reviewed at `28d1101b21f0c274d6ece6482c307eebe25a086e` on 2026-09-18.
The missing hardware-coherence path reported in the fourth audit is real,
and now has an executable integration reproducer. No production RTL or
top-level wiring was changed. No Quartus build or hardware operation ran.

## Result

1. An ordinary read fills AP040's D-cache with `$11223344` from Fast RAM.
   A second read hits without accessing the lower cache.
2. The real `chipdma_arb` routes two byte writes through the real DDR
   controller: `$AA` to the even byte and `$BB` to the next odd byte.
   Both complete, and DDR backing memory contains `$AABB3344`.
3. The next cacheable CPU-side read returns **`$11223344`**, without any
   lower-cache access.
4. Invalidating AP040's D-cache makes the read return **`$AABB3344`**.
   This needs no new DDR read, confirming the controller's outer cache
   already contains the DMA update.

| Wiring/control | Correct result | Stale result |
|---|---:|---:|
| Reviewed wiring, no DDR-to-AP040 snoop | 0/16 | **16/16** |
| Bypass AP040's cache | **16/16** | 0/16 |
| Diagnostic invalidate of the correct set after DMA | **16/16** | 0/16 |
| Diagnostic invalidate of a different set | 0/16 | **16/16** |

Each group covers Z2 via Akiko, Z2 via CDTV, Z3_0 via CDTV, and Z3_1 via
CDTV, with clock enables 1 and 4 and DDR waitrequest either clear or
alternating. Both byte lanes are written in every case. Akiko's request
address is 24 bits; CDTV's is 32 bits, including its writable ACR register.
The Z3 tests deliberately use bases different from their backing-memory
addresses. Each configuration enables only the RAM window under test.

The positive controls isolate the missing inner-cache invalidation. The
wrong-set control confirms that an arbitrary snoop pulse is insufficient.
The runs do not measure the performance cost of disabling Fast RAM caching.

## Reproduction

```sh
python3 tests/ap040/audit_fast_dma.py --work /tmp/ap040-fast-dma-audit --jobs 4
```

This exits nonzero on the current stale-data result. To verify the complete
current defect/control matrix with a successful diagnostic exit:

```sh
python3 tests/ap040/audit_fast_dma.py --expect-defect --work /tmp/ap040-fast-dma-audit --jobs 4
```

Runner: [audit_fast_dma.py](tests/ap040/audit_fast_dma.py).
Bench: [tb_ap040_fast_dma_audit.v](tests/ap040/tb_ap040_fast_dma_audit.v).
Logs, JSON results and source hashes are in `/tmp/ap040-fast-dma-audit/`.

This is a focused integration bench, not the entire `Minimig.sv`. It
instantiates the actual AP040 cache, bus16 adapter, memory router, CD DMA
arbiter, DDR controller, outer cache and DDR arbiter. The compat wrapper's
cache-window predicate is extracted verbatim. The CPU request source and
DDR memory are models. No internal cache state is forced or preloaded;
only backing memory is initialized. CPU/cache and DDR share the bench
clock, with divided CPU enables; the DMA arbiter has its own slower clock.
The CPU wrapper's real clock crossing and full chipset are outside scope.

The bench represents the inspected missing DDR snoop connection by driving
AP040's snoop input inactive. The diagnostic snoop is injected after the
DMA operation settles; it is not a production CDC/merger implementation.
Once a real fix exists, this bench must instantiate that producer and
merger before it can serve as an integration regression gate. The runner
rejects a newly added DDR snoop interface rather than silently treating
the old wiring model as the fixed design.

## Fix constraints and limits of the claim

- **Do not OR independent snoop toggles.** One source can mask transitions
  from the other. Independently capture events and retain both addresses
  when different sets need invalidation. Account for the existing walker
  pending-snoop path as well.
- **Current invalidation uses only `[9:4]`.** AP040 clears a whole D-cache
  set, and the memory router preserves these bits. A 32-bit reverse address
  mapping is therefore not required for this policy, despite the existing
  25-bit chipset address port. A later line/tag-selective policy would need
  the correct CPU physical address and alias handling. Earlier concern that
  the narrow address alone blocks a fix would be overstated.
- **Invalidate at a safe point relative to memory visibility.** A cache
  miss/refill after invalidation must not reinstall pre-DMA data. DDR
  backpressure, simultaneous SDRAM/DDR/walker events, cache fills, and
  divided enables need tests of the actual future implementation.
- **The stale line does not necessarily wait for eviction or CINV.** An
  unrelated SDRAM or walker snoop to the same set can invalidate it. This
  incidental invalidation can mask the defect; it supplies no reliable
  coherence guarantee for the DMA write.
- **Reachability is conditional on the workload.** The hardware path needs
  a resident cacheable Fast RAM line, an external write to it, and a later
  CPU read before effective cache maintenance or incidental invalidation.
  It does not need an MMU fault or tracing. Driver cache-maintenance behavior
  and the actual CD/DMA configuration determine exposure. Neither a generic
  Workbench boot nor the named demos can be categorically included or
  excluded from source inspection alone. No connection to their symptoms
  has been established.

Removing the Fast RAM terms from `cache_win` prevents this inner-cache
failure for those windows, as the bypass control demonstrates. It would
also disable instruction caching there if done literally to the shared
predicate. That broad performance change is unnecessary to reproduce or
review the issue and was not applied.

## Corrections to the surrounding report

The confirmed loaded image is
`Minimig-ap040-40mhz-f86b3980f-20260918_152411-TIMING-FAIL-DO-NOT-FLASH.rbf`,
not `ec25690cd`. Statements counting only pre-`f86b3980f` findings as
present in the loaded source are stale. This verification independently
confirms the Fast RAM finding; it does not revalidate every architectural
classification in the other five reported findings.

The [separate execution/RESET audit](AUDIT_EXECUTION_RESET_20260918.md)
contains the earlier 780 passing normal-execution cases and seven
cache/DMA unit legs. They do not contradict this failure. Those cases do not
inject the new MMU/trace conditions, and a unit bench that supplies snoops
cannot prove that every SoC DMA writer actually supplies them.

The running bitstream also retains its measured setup failures: -1.253 ns
in the emu domain, including a -0.315 ns chipset-read path, and -0.667 ns
in HDMI. The independent simulated coherence failure neither establishes
the cause of the mouse artifacts nor removes that timing confounder.
The failed image is not a valid measurement platform; timing recovery
precedes further board-level symptom attribution. See the
[path and clock diagnosis](TIMING_F86B3980F_20260918.md).
