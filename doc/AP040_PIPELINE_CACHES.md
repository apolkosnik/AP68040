# AP040 pipelined core: instruction and data memory units

Status: design, revision 2 (2026-09-25), after review; stages A-E and R
built, F in part (below, "Stage A as built" through "Stage F as built"). Goal: the MC68040's
integrated caches in the pipelined core -- a 4 KB instruction cache and a
4 KB data cache, each beside its own ATC at one of the pipeline's two memory
ports, the data cache write-through or copyback page by page through the
internal MMU, as MC68040UM section 4 describes. Where this platform departs
from the 68040 it says so below; nothing else is intended to differ.

## Where they sit (MC68040UM 4, Figure 4-1)

The instruction memory unit (IMU) serves instruction prefetch and the data
memory unit (DMU) operand accesses. Each translates the logical address in
its ATC while its cache reads the set selected by the untranslated bits
PA9-PA4; the ATC's PA31-PA12 and the untranslated PA11-PA10 are compared
with the four 22-bit tags. Misses, write-through writes and pushes go to the
bus controller below both units.

Until stage A the pipelined core translated below its bus controller:
CPU -> ap040_pipe_membus.v -> ap040_mmu.v -> bus16 adapter. Since stage A
the operand port translates in the DMU (ap040_pipe_dmu.v) and the fetch
stream is translated before each of its reads -- by the bus controller in
stage A, by the IMU (ap040_pipe_imu.v) since B1 -- both through
ap040_pipe_mmu.v; nothing below the bus controller translates. The
instruction cache joins the IMU in stage B, the data cache the DMU in
stage C. The target:

    port A (IF)  -> IMU: I-ATC lookup + I-cache  \
                                                   > membus (bus controller, physical) -> adapter
    port B (EAF) -> DMU: D-ATC lookup + D-cache  /
                    table walker behind both ATCs; its descriptor reads and
                    writes consult the data cache (below)

## Cache behaviour

Both caches: 64 sets x 4 ways x 16-byte lines, 22-bit physical tags;
replacement takes the first invalid line, else a 2-bit counter advanced as
4.1 describes. A line is loaded only by a complete four-longword line read
(4.1, 4.6.1), the requested longword first and the rest wrapping; the data
is available to the requester as each longword arrives (the line read
buffer), and hits on other lines are served meanwhile.

Instruction cache: one valid bit per line; CACR IE enables it. CPU data
writes do not reach it (4.5): self-modifying code needs CPUSH/CINV.
External snooped writes invalidate matching lines (Table 4-3, V6).

Data cache: a valid bit and four dirty bits per line; CACR DE enables it.
Caching mode per access from the matching TTR or the page descriptor's CM;
translation disabled and no TTR match: write-through (4.3).

| Access | Write-through | Copyback | Cache-inhibited (serialized or not) |
|---|---|---|---|
| read hit | from the cache | from the cache | push the line if dirty, invalidate, bus read |
| read miss | line read, allocate | line read, allocate | bus read |
| write hit | update the line, Dn unchanged, bus write | update the line, set Dn | push if dirty, invalidate, bus write |
| write miss | bus write, no allocation | line read, allocate, update, set Dn | bus write |

A write-through write to a dirty line keeps its Dn bits (Table 4-4, D6, a
programming error the manual still defines). Replacing a dirty line: the
line goes to the push buffer, the new line is read, then the buffer is
written back -- a longword push if one longword is dirty, a line push
otherwise; if the replacement line read is cache-inhibited or errors, the
dirty line returns to its place and a valid line stays invalid (4.6.1, 4.6.2).

Special accesses, each carried to the DMU as request metadata (below):
- exception stacking, vector fetches and table searches never allocate;
  hits are served by the access's mode (4.3.3);
- MOVE16 never allocates; a read hit is served from the cache; a write hit
  invalidates the line and the write goes to the bus (4.3.3);
- locked accesses (TAS, CAS, CAS2, descriptor updates) are noncachable: a
  matching dirty line is pushed and a valid one invalidated before the
  locked transfer (7.4.5);
- MOVES to SFC/DFC $2/$6 becomes a data access in $1/$5 and is handled as
  any data access, cache included; $0, $3, $4 and $7 are alternate-space
  accesses, used untranslated as physical addresses (3.2, Table 3-2), and
  treated here as cache-inhibited (both cores since stage A2).

CINV (line, page, all; IC, DC, BC) invalidates, discarding dirty data;
CPUSH pushes dirty data lines, then invalidates. Neither depends on CACR.
The line and page forms take a physical address in An.

Table searches: the 68040 serves descriptor reads that hit in the data
cache from the cache (4.3.3), so a descriptor the CPU wrote into a copyback
page is seen by the next search. Here the walker keeps its physical port
for misses, but every descriptor read first looks up the data cache and
uses a hit, and every descriptor history write (U, M) updates a present
line's longword as well as memory, clearing that longword's dirty bit since
memory then matches. The walker uses the DMU's lookup port only while the
DMU is itself waiting on the walk, or by holding the DMU's next request.

## Platform boundary: snooping and DMA

The Amiga chipset cannot drive the 68040's snoop controls: it can neither
source dirty data to a DMA read nor sink a DMA write into a dirty line. What
this platform provides is the "invalidate line" operation (SC = 10) on
every chipset or walker write into memory, applied to both caches as Tables
4-3 and 4-4 define. Consequently:
- a DMA read cannot see dirty data: memory that DMA reads must be
  write-through or inhibited, or pushed by software (CPUSH) first;
- a DMA write into a dirty line loses the line's dirty data;
- so chip RAM and any other DMA-shared memory must be write-through or
  cache-inhibited through the MMU or a TTR -- as 68040.library configures a
  real A4000 -- and copyback is for memory no other master touches.
These are requirements on system software, stated here as the boundary of
"as on the 68040". The FSM wrapper's cacheable windows and its instruction
bypass for chip RAM with the MMU off remain integration options (stage F).

## Storage

Cyclone V M10K: true dual-port at most 20 bits a port; simple dual-port
(one read, one write) up to 40. The M10K column is stage 0's fit of the
arrays alone (tests/ap040/pipe_synth/ap040_pipe_cache_fit.v).

| Array | Organization | Mode | M10Ks |
|---|---|---|---|
| I data | 4 ways x 128 x 64 (a half-line a read) | simple dual-port: read lookups, write fills | 8 |
| I tags | 64 x 88 (a row of four 22-bit tags) | simple dual-port | 3 |
| D data | 4 ways x 256 x 32, byte enables | true dual-port: A lookups and store hits, B fills and push reads | 8 |
| D tags | 64 x 88 | true dual-port: A lookups, B fills and maintenance reads | 5 |
| snoop tags | a copy of each cache's tag rows | simple dual-port, written with the tags | 6 |
| ATCs | 32 rows {bank, set} x 4 ways x 46 | true dual-port, as ap040_mmu.v: A I-lookups, B D-lookups and fills | 8 |

38 of the 5CSEBA6's 553; the FSM core's caches and ATC, which this core
replaces on the card, free a similar number. A tag row holds all four ways,
so a fill reads its row first and writes it back with the new way's tag:
the I side through its lookup port, idle while the unit waits on its own
miss, the D side through port B. Valid, dirty and replacement state in flops
(1,280 bits for the D-cache) so invalidation and dirty updates need no RAM
port -- but tag reads still do: an addressed snoop reads the snoop-tag copy,
line and page maintenance and pushes use port B, a store hit writes port A.

rtl/bram.vhd's dpram has no byte enables. Stage 0 adds a wrapper exposing
altsyncram's byteena on both ports (and its Verilator model); partial stores
write their bytes directly rather than by read-modify-write.

Port use and collisions, per data cache cycle: port A takes a lookup, or a
store-hit write (the store's lookup was two cycles earlier; a lookup of the
same line that cycle is served from the store buffer); port B takes a fill
write, a push read, or a maintenance tag read, in that priority, with the
walker's descriptor lookups taking port A while the DMU waits for the walk.
A snoop reads the snoop-tag copy and clears valid flops; a snoop that hits
the line being filled marks the fill "do not validate"; a snoop that hits
the push buffer's line is recorded against the buffer (the push still
writes -- the DMA-write loss above).

## Performance model (fixed in stage 0, before stage A)

- Hit latency: two cycles on both ports (request, RAM read, tag compare
  into a registered result), one new request per port per cycle. Stage 0's
  fit of the arrays and the ATC alone: the slowest case -- translation on,
  no recent-hit copy: ATC row, ATC tag compare, physical page, the four
  cache tag compares, way select, into the result register -- is the
  worst path, 14.3 ns, 62.9 MHz, +9.1 ns at 25 ns; 874 ALMs of compare
  and select logic.
- Instruction fetch: a half-line (four words) per hit, so the fetch queue
  (ap040_inst_fetch.v) takes up to four words a return and sustains two
  words a cycle with one request outstanding.
- Operand port: one outstanding read in stages B-D (port B's present
  protocol), so a load every two cycles at best; pipelining it is stage E.
- Simultaneous instruction and data hits: yes, the caches are separate.
- Stores: posted into the DMU's store buffer once translated; a load that
  matches a buffered store is served from it (byte-accurate) or waits.
- Hit under miss: the DMU serves hits while a line read completes (4.6.1);
  the IMU serves hits during a data-side miss and vice versa.

## Request metadata and handshakes (defined in stage 0)

From EA-fetch to the DMU, with each port-B request: size and alignment,
read/write, function code (MOVES override, with the $2/$6 conversion and the
alternate-space flag), locked (read and write halves of TAS/CAS/CAS2),
no-allocate (exception stacking, vector fetch), MOVE16 (read or write,
line-sized), and cache maintenance (CINV/CPUSH: IC/DC/BC, line/page/all,
push or invalidate, the physical address from An) with a completion
handshake EX waits on. From the MMU with each translation: the full CM
(2 bits), W, S and M, not ap040_mmu.v's CM[1] alone. Decode carries CINV/
CPUSH's push bit (opcode bit 5), scope (bits 4-3) and register (bits 2-0),
which it currently drops.

## Faults and write-back recovery

A fill that errors on the requested longword's beat faults that access as
today's bus errors do. An error on a later beat aborts the line read and
leaves the line unloaded; it faults only an access actually waiting on that
beat (the second half of a misaligned operand), otherwise the line is read
again when next needed (4.6.1). A failed replacement read restores a dirty
victim from the push buffer.

A push belongs to no instruction: the dirty data came from earlier stores.
The 68040 reports a push bus error as an access error with SSW RW = 0,
TT = 0, TM = 0, a PHYSICAL fault address, the line's four longwords in the
frame's push data fields (PD0-PD3), WB1S invalid and WB2S/WB3S for any
further pending write-backs (8.4.6, Table 8-6), and calls it normally fatal.
The pipe's format-$7 frame builder (ap040_ea_fetch.v) clears every
write-back field today; stage R extends it to carry push data and pending
write-backs, and defines which instruction the exception is taken at. The
same stage defines the posted-write buffer's fault ownership once copyback
exists.

## Stages, each verified before the next

0. Storage and interfaces: the RAM wrappers (rtl/ap040_pipe/ap040_pipe_ram.vhd,
   true dual-port with byte enables and simple dual-port, and their
   Verilator models, tests/ap040/sim_pipe_ram.v, which return junk where
   the silicon is DONT_CARE); the arrays (rtl/ap040_pipe/ap040_pipe_cache_arr.v)
   fitted alone with the ATC -- done, figures above; the request metadata
   and handshakes specified above. Each is implemented with the stage that
   first consumes it, so none is a port nothing drives or reads: the MMU's
   full attributes in A, CINV/CPUSH decode in B, the access attributes and
   the maintenance handshake in C.
A. Translation at the ports: I and D lookup ports on the one ATC RAM, the
   walker behind them; membus physical, reporting only bus errors; faults,
   page-crossing accesses, MOVES spaces, PTEST and PFLUSH at the ports.
   Behaviour-neutral: every suite, the corpus, every MMU program -- built in
   two steps: A1, the translation moved (done, "Stage A as built": the DMU
   at port B; the fetch stream translated by membus until the IMU comes in
   B), and A2, alternate-space MOVES ($0/$3/$4/$7) untranslated and $2/$6
   on the bus as data, the one intended change of behaviour -- made in
   both cores, as the user decided: ap040_pipe_dmu.v and membus for the
   pipelined core, ap040_mmu.v (and ap040_core.v's page split) for the
   sequential one. t_mmu.s tests 55-56 had expected the MMU's access fault
   on a MOVES to FC 0 of a write-protected page (TT=10, TM=0, after
   WinUAE's mmu_bus_error); under 3.2 that write is untranslated and lands,
   and they now say so. t_moves_alt.s tests each space on both cores, the
   bus function code of $2, and the TT/TM an alternate-space bus error
   reports.
B. The instruction cache: half-line fills and reads, the four-word fetch
   queue, CACR IE, CINV/CPUSH on IC, snoop invalidation.
C. The data cache, write-through only: fills with the read buffer, write-hit
   update with byte enables, the store buffer, the special accesses, locked
   accesses, MOVES, table searches through the cache, snoop invalidation,
   CINV/CPUSH on DC.
R. Write-back recovery: fill-beat error rules, push faults and their frame,
   victim restore, posted-write fault ownership -- exercised with forced
   errors before copyback is enabled.
D. Copyback: dirty bits, write allocation, the push buffer, CPUSH pushes,
   pushes before inhibited and locked accesses.
E. Throughput: the pipelined operand port, back-to-back line transfers.
F. Integration: the cacheable windows and snoop inputs on the bus16 top,
   a full-system fit.

## Stage A as built (A1)

Units: ap040_pipe_dmu.v between the CPU's port B and ap040_pipe_membus.v,
which it hands physical addresses; ap040_pipe_mmu.v (rtl/ap040/ap040_mmu.v's
rules, ATC rows and walker, with an instruction and a data translation port)
beside it; membus drives the 16-bit adapter directly. The CPU sees the same
port protocols it saw from membus.

- Translation from registers. The first draft translated each request in
  its own cycle, at both ports, in front of membus's one-cycle fetch window.
  It fitted at -5.3 ns (32.98 MHz): both ports' addresses settle about 17 ns
  into the cycle, from the CPU's stalls, and an ATC compare and a physical
  address after them do not fit. The old design translated from membus's
  registered transaction. So every translation now starts from a register:
  the DMU latches a request as it arrives and translates from the latch;
  membus translates its stream's next read from its own register. The
  caches' two-cycle lookups (stages B and C) start from registers anyway.
- Port B. With translation able to refuse (TC.E or a data TTR), a read is
  translated the cycle after it is asked for and goes to membus through a
  translated-read input that puts it on the bus in that cycle -- the old
  timing, membus having taken a read and put it on the bus the next cycle.
  A tentative write is accepted the cycle after its translation passes --
  a cycle after it is taken on the MMU's most recent hit, two after a
  lookup, as membus accepted it -- so wr_busy is a register. Untranslated,
  reads and writes pass straight through, as they did.
- Port A. membus keeps its window, logical as before. With TC.E set, the
  stream's next read is translated through the MMU's instruction port from
  membus's register before it takes the bus -- starting at once, whatever
  the bus is doing -- and never while holding the bus. Holding it would
  deadlock: the walker may be waiting for a write membus has still to send.
  A prefetch in the page the instruction port translated last goes out in
  its own cycle through a peek at that translation (ip_*: combinational from
  the MMU's registers, no search, no fault); a demand miss to another page
  takes one cycle more than before. All of this moved to the IMU in B1.
- Port ownership. A requester holds its MMU port, at the same address, until
  the translation passes or faults -- a walk cannot be recalled and its fault
  belongs to the access it was for. After a fault the port is down for a
  cycle, which is what the walker's W_DROP waits for; in the DMU, whose slot
  and read share the data port, that takes an explicit gap.
- Ordering. A read waits while the DMU's write slot holds anything, and
  membus sends the writes it holds before reads. The walker's next access
  waits while any write the DMU has accepted has not reached memory
  (walk_hold): the sequential MMU's walks ran only at the head of the bus,
  after every older write, and ap040_tg68k_compat.v holds its walker behind
  the posted-store drain for the same reason. Reads and fetches run beside
  a walk.
- Page crossings (TC.E): the DMU checks a crossing write on both pages (the
  MMU's access check, no M), then translates both for real (setting M), and
  only then accepts it and posts its bytes; a refused one has written and
  marked nothing, MA if its second page refused it. A crossing read
  translates both pages, then reads its bytes.
- membus reports only physical bus errors: mem_flt is the bus error alone,
  wr_sync low, no crossing split, no probes.
- The window's snoop (A3). The window is logical, and with the DMU above
  it membus receives each write translated; the snoop compared the one with
  the other, so with code mapped away from its physical address a store into
  the window left the replaced words there. Programs never showed it -- the
  CPU's own store snoop refetches what it has already taken, and that
  refetch empties the window -- but nothing stood in front of the window
  itself. The DMU now hands membus the write's logical address as well
  (m_la), and the snoop compares that.

Tests changed: tb_ap040_pipe_program.v's "walker and 16-bit bus active
together" assertion -- true of the old structure, not a requirement -- is
now the ordering rule itself: the bytes of every write the memory side
commits to are counted until they land on the 16-bit bus, and the walker may
start an access only with none outstanding (and every committed byte must
land, unless a bus error aborted it). New: t_walk_order.s (a descriptor
stored and at once searched through, read, pointer and write forms; passes
on both cores), t_fault_edges.s test 68 (a refused crossing write leaves
the first page's M clear), and tb_ap040_pipe_dmuport.v (the DMU, MMU and
membus driven at the CPU's ports: a write presented during a read's failing
search, a read behind a write waiting in the DMU, a refused fetch, a search
behind a committed write). tb_ap040_pipe_wrreceipt_bus16.v times its
clock-enable holes from the DMU's acceptance; tb_dat_replay_pipe.v drains
the DMU's writes. A3: t_smc_mmu.s (stores rewriting prefetched code run
through an alias of it, both cores) and tb_ap040_pipe_dmuport.v test 7 (a
write into the window through a page mapped elsewhere, no CPU in front).

## Stage B as built

B1, the IMU. membus's port A -- the prefetch window, its reads' translation
through the MMU's instruction port and peek, and its snoop -- moved unchanged
into ap040_pipe_imu.v, which asks membus for each read through a fetch port:
f_req with f_addr and f_sup, taken when the bus is free for it (f_free:
nothing on it, no write or read waiting -- the order membus's chain always
had), answered with f_ack (mem_rdata) or f_flt; and membus tells it of every
write it takes (w_accept, with w_sla, the write's logical longword) for the
snoop. Every bench was cycle-identical to A3 after the move.

Moving it showed a flaw from stage A: a write whose snoop empties the window
in a cycle with no read in flight. The next longword was pf_base + pf_cnt
before the snoop and pf_base after it, and a stream read for the first
landed as the second -- the window then answered its base with another
longword's word. The bus controller's own read cannot go in that cycle (the
write is waiting there, so the bus is not free), but since A1 a translation
could start then, and did. Nothing new starts in a cycle whose snoop hits
the window (tb_ap040_pipe_dmuport.v test 8: the write's arrival swept
across the window's refill on a slow memory, reaching that cycle four
times; without the fix, fetches returned the longword two or three past the
one asked for).

B2, CINV and CPUSH. Decode keeps the instruction's whole field -- caches
(IC, DC), push, scope -- and reads An as its source. EA-fetch runs it at the
caches as it runs PFLUSH at the MMU: once EX, WB and the memory side are
quiet, the request goes out with An and is held until done (cm_*), fetches
are quiesced meanwhile, and the instruction retires with a refetch -- the
68040's interlock (10.3: previous writes and pending prefetches complete,
and nothing behind it reaches the caches until it is done). Line and page
name a physical address; a page is 4 KB whatever TC.P says (the MC68EC040
notes of the manual). The IMU does the instruction cache's part once it is
idle; the data cache's is the DMU's from stage C, and until then there is
nothing to do. The sequential core keeps widening every form to all lines.

B3, the instruction cache, in the IMU:
- The window stays the fetch unit's source; with CACR IE set the cache is
  the source of the window's reads. A read is held in a register (lk_*):
  its set is read in the next cycle and compared in the one after. The read
  cannot address the RAM in its own cycle: it forms from port A's address,
  which on the bus16 top arrives about 19.5 ns into the 25 ns cycle.
- A hit answers with the half-line (PA3), which is the longword asked for
  plus the next one when both lie in it. The window may send its next read
  in the cycle an answer arrives. Hits therefore bring two longwords every
  two cycles, and the fetch unit takes a longword a cycle, the rate it
  always could; the plan's four-word fetch queue is not needed for it.
  Eight cached longwords reach the fetch unit in ten cycles.
- A miss reads the whole line, the longword asked for first and the rest
  wrapping (4.1, 4.6.1). Each longword is one bus controller read, since
  the 16-bit adapter has no burst, and the first goes in the cycle the miss
  is seen.
- The longwords gather in the line read buffer, and each answers the
  window's read for it as it arrives. The stream takes a line in the order
  the fill reads it, from wherever it came in.
- Each half-line is written once both its longwords are in. The tag goes
  in with the last, in the same cycle that makes the line valid; any
  lookup after that sees it.
- A read for another line waits until the fill ends.
- A bus error on any beat abandons the line, and its way stays invalid.
  Only a read waiting for that very longword is faulted. A read waiting
  for another longword is looked up again, and fills again from its own
  longword.
- Replacement takes the first invalid way, else the way a 2-bit counter
  names. The counter advances on every half-line looked up, and once more
  after it names a way (4.1).
- Reads with IE clear, and reads cache-inhibited by a TTR's or a page's CM
  (1x), go to the bus controller one longword each, allocate nothing and
  leave the cache as it was. The MMU's peek now reports its page's CM with
  the page. With IE clear and the cache idle, the window's reads take B1's
  path unchanged; a read already out when IE changes is still answered.
- CPU writes do not reach the cache (4.5). The window's own snoop remains:
  it empties the window, and the refetch then comes from the cache.
- A read the window abandons is answered at once, unless its bus read is
  already out, and the miss it would have made is not filled. A fill
  already under way completes: the line is wanted, and abandoning a line
  at a loop's branch would keep that line from ever being cached.
- CINV/CPUSH on the instruction cache: all ways at once (valid bits in
  flops); a line reads its set's tag row and invalidates the matching way;
  a page reads all 64 rows, one a cycle, comparing PA31-PA12. That is 66
  cycles, where the 68040's CINVP takes 266 (10.3). There is no dirty data
  here, so CPUSH is CINV.
- Snoops (sn_*) follow Table 4-3, V5/V6. They read the arrays' copy of the
  tag rows, never the lookup's port.
  - A snoop of the line being filled keeps it from being made valid.
  - A copy row read in the cycle a fill writes that set's tags is
    undefined in the silicon, and the snoop then takes the whole set.
  - The valid bits are updated for a snoop's set and one other change
    (a miss's victim, a fill's end, a maintenance row) in the same cycle,
    merged when the two are the same set.
  - Nothing on the tops writes memory behind the CPU yet; the card's
    chipset will drive the snoop in stage F.
- The price is on code run once. Each line's first longword waits two
  cycles longer than a bus read would, and a line entered partway reads
  longwords the stream never wants. t_integer, which runs its whole battery
  with the caches on, takes 11,941 cycles on the program bench's first
  phase against 10,961 with IE clear. Loops gain: dhry takes 652,982
  against 1,424,493. Burst line reads (the DDR3 line interface) are what
  would remove the cost.
- Fit at 25 ns, bus16 top: 40.81 MHz, +0.497 ns, 21,243 ALMs, 21 RAM blocks
  (B1: +0.310, 20,533 ALMs, 10 blocks) -- the eleven blocks the data and
  tag rows take; the snoop copy is not in it while the tops tie the snoop
  off. The worst paths are EA-fetch's own; the IMU's are the window's.

Tests:
- t_icache.s (pipelined core only). It runs translated, with the code's
  pages inhibited, so that only the stubs it probes are ever cached. It
  covers:
  - stale lines after stores (4.5);
  - CINV and CPUSH on a line, a page and everything, each leaving the
    rest, with the data cache's forms leaving the instruction cache;
  - IE clear, which bypasses the cache and keeps it;
  - inhibited TTRs and pages, which allocate nothing;
  - four ways of a set held, and a fifth line replacing one;
  - an error on a beat nobody asked for, which faults nothing and leaves
    the line invalid;
  - one on a beat the program needs;
  - a physical An through an alias.
- tb_ap040_pipe_icache.v, which drives the IMU and membus at their ports.
  It checks timing and order:
  - the fill order;
  - the hit latency and a hit's silence on the bus;
  - the streaming rate;
  - 4.5 against a port-B write;
  - the replacement order against a model of the counter;
  - CINV's scopes, its duration and its wait for a fill;
  - beat errors;
  - abandoned reads;
  - IE changing under a read and under a fill;
  - inhibited reads;
  - the line read buffer;
  - snoops: back to back, swept across a fill and across its tag write,
    and meeting a CINVP row and a miss's victim in the same cycle.
- tb_ap040_pipe_dmuport.v test 8 is run with IE clear and with IE set.
- The programs that turn the caches on now run through the cache: t_integer,
  t_fastpaths, t_fpu, t_exceptions (fetch bus errors), t_moves_fc, t_mmu,
  t_bitfield_cache, t_cinv_moves and dhry.

## Stage C as built

The data cache, write-through, in the DMU (ap040_pipe_dmu.v). It uses
stage 0's arrays: four ways of 256 longwords with byte enables, one tag row
per set, and the snoop copy. The valid bits and the counter are in flops.
CACR DE turns it on; with DE clear every path is stage B's.

- Reads. With DE set, every read is latched and translated. An
  untranslated read passes the MMU at once, with a data TTR's caching mode
  or write-through; DE thus costs that path its straight-through cycle.
  The read's set is read in the cycle its translation passes, since PA9-PA2
  are the latched logical bits, and compared the cycle after. A hit is
  answered three cycles after the request, right-aligned by size, for any
  size at any offset inside a longword.
- A read spanning two longwords, or crossing a page, goes to the bus as it
  did. Memory holds what the cache does, since the cache is write-through.
- Misses read the line, the longword asked for first and the rest wrapping.
  - Each longword is one bus controller read through the translated-read
    port; the first goes in the miss cycle.
  - Each longword is written to its way as it arrives; the tag and the
    valid bit go in with the last.
  - The read is answered from the first longword. Later reads of the line
    are answered from the line read buffer as their longwords arrive,
    unless a write's update is waiting.
  - A read for another line waits until the fill ends.
  - An error on any beat abandons the line, faulting only a read waiting
    for that longword. A read waiting for another longword is looked up
    again. The way's old tag is still in the row, so the way stays invalid.
- Writes go to the bus controller as they did.
  - Each updates a line holding it, through the byte enables (both, for a
    write spanning two longwords or lines), in the order sent, before any
    later read's lookup (4.3.1.1, Table 4-4).
  - A write sent while a line is being read keeps its update until the
    line is in, then applies it, so the line never loses a write, whichever
    of its reads the write overtook.
  - While an update waits, the next write is held.
- Not cached. Accesses made inhibited by a TTR's or a page's CM, MOVES to
  an alternate space, and locked accesses (TAS, CAS and CAS2: EA-fetch's
  reads and EX's store beat) each invalidate a line holding them, then go
  to the bus (4.3.2, 7.4.5).
- Allocating nothing. Exception frame writes, the vector fetch (and the
  reset vectors) and MOVE16 allocate nothing (4.3.3): a miss is a single
  bus read and a hit is served. A MOVE16 write that hits invalidates the
  line.
- Replacement takes the first invalid way, else the way the counter names.
  The counter counts every read looked up and every write sent, and once
  more after it names a way (4.1).
- CINV/CPUSH on DC: all ways, a line, or a page (64 rows through the tag
  port B). There is no dirty data, so CPUSH is CINV here.
  - Each cache takes a request once; the bus16 top's done is both units'.
  - The CPU starts the instruction only once the data side is idle, so
    the data cache begins at once; every later access waits for it.
- Table searches. The walker writes U and M through its own port, behind
  the cache, and every write it lands is snooped into the data cache: the
  line holding the descriptor is invalidated. 4.3.3 has a table search's
  write hit update the line; with write-through the two agree (t_mmu.s
  tests 148-149). The walker's reads come from memory, which a write-
  through cache matches once the writes ahead have landed (walk_hold).
  Copyback (stage D) makes both go through the cache.
- Snoops work as the instruction cache's; the walker drives the data
  cache's on the bus16 top.
- Cost and gain, on the program bench's first phase with the caches on
  (stage B's figures first):
  - dhry: 652,982 -> 437,103 cycles;
  - t_fpu: 107,313 -> 100,022;
  - t_bitfield_cache: 36,609 -> 34,941;
  - t_integer: 11,941 -> 12,128 (straight-line code, every read latched
    and looked up).
- Fit at 25 ns, bus16 top: 40.31 MHz, +0.190 ns, 22,561 ALMs, 38 RAM blocks
  (B: +0.497, 21,243, 21). The worst paths are EA-fetch's own, none through
  the data cache.

Tests:
- t_dcache.s (pipelined core only), which sees the cache through pokes:
  memory changed behind the CPU (tb_ap040_pipe_program.v's new $F134/$F136
  registers). It covers:
  - a read miss allocating, a write miss not;
  - write hits updating, including byte, word at an odd address, a
    longword across longwords and across lines;
  - CINV/CPUSH DC on a line, a page and everything, and the instruction
    cache's forms (all, line, page) leaving it;
  - DE clear, which bypasses the cache and keeps it;
  - a DTT's CM 10 inhibiting reads and writes, invalidating a line hit;
  - MOVES to FC 3;
  - TAS and CAS locked;
  - MOVE16 hits served from the line, its writes invalidating, and its
    misses allocating nothing;
  - the vector fetch allocating nothing;
  - a page's CM and a physical An through an alias.
- t_cache.s, the sequential core's cache test, now runs on the pipe too.
- tb_ap040_pipe_dcache.v, at the DMU's ports. It checks:
  - the fill order;
  - the hit latency, and every size at every offset;
  - the byte enables, and spanning writes;
  - a write meeting a line being read, swept across the fill;
  - inhibited, locked, MOVES, no-allocate and MOVE16 accesses;
  - CINV's scopes and its done;
  - beat errors, including a way whose old tag outlives an abandoned fill;
  - snoops, swept across a fill and its tag write, with memory written as
    they are raised;
  - replacement against the counter model;
  - DE cleared under a fill, with a read at once;
  - a page's CM.

## Stage D as built

Copyback, in the DMU; dirty bits a longword each, in flops.

- Where it applies. A page or data TTR with CM 01. Only the translating
  write slot reaches it (TC.E or a data TTR enabled). With neither, every
  access is write-through by the manual's default (4.3), and writes keep
  their straight path and stage C's ordering.
- Copyback writes. The slot's aligned write -- not MOVE16, locked or to an
  alternate space, and spanning neither longwords nor pages -- goes to the
  cache instead of the bus.
  - A hit updates the line and marks its longword dirty.
  - A miss reads the line, the written longword first, and then does the
    same.
  - A miss that allocates nothing (an exception frame) goes to the bus
    alone, and so does one whose line read errs, whichever longword errs.
  - A copyback write spanning longwords or pages is written through: lines
    it hits are updated and keep their dirty bits.
- Write-through writes update a line holding them and leave its dirty bits
  as they were (Table 4-4).
- Replacement of a dirty line.
  - Its four longwords are read out of the way through the lookup port,
    from the longword the fill reads first, one a cycle, while the fill
    lands on the other port. Each is out before the fill's longword lands
    on it (tb_ap040_pipe_dcache.v test 13 sweeps memory latency and the
    first longword).
  - Its dirty longwords are then written, ahead of any CPU write. Only the
    dirty longwords are written; memory ends as after the 68040's
    four-longword push.
  - No line is read and no read goes to the bus until they are written:
    memory has the line first.
  - A fill that errs pushes the victim all the same. The 68040 returns it to
    its place (4.6.2); either way no data is lost.
- Not cached (inhibited, locked, alternate space, MOVE16). A hit on a dirty
  line pushes it and then invalidates it (4.3.2, 7.4.5). For a write, the
  write is merged into the line first, so the push carries it after the
  write itself is in memory.
- CPUSH pushes each dirty line in its scope (a line, a page, or all), one
  at a time, then invalidates. CINV drops dirty data.
- Table searches go through the cache. The walker's port passes through the
  DMU (4.3.3):
  - a read uses a hit;
  - a write (U, M) updates a line holding the descriptor and goes to memory,
    leaving that longword clean;
  - the walker still waits for writes on their way to memory, and now for
    pushes: with DE clear its reads go straight to memory, and a CPUSHA
    after clearing DE may be pushing the line holding a descriptor.
  t_copyback.s's search reads a descriptor that is dirty in the cache,
  where memory still says invalid.
- Snoops invalidate, and a dirty line snooped loses its data. This is the
  platform boundary: the chipset cannot take dirty data.
- Push bus errors are dropped with the push, as the bus controller drops
  any posted write's. The 68040's access error with the push data (8.4.6)
  is stage R's.
- The counter also counts copyback write lookups.
- Fit at 25 ns, bus16 top: 42.14 MHz, +1.271 ns, 23,840 ALMs, 35 RAM
  blocks (C: 40.31 MHz, +0.190, 22,561, 38). The three fewer blocks are the
  data cache's snoop tag copy: stage C drove it from the walker's writes;
  with the walker through the cache nothing on this top snoops, so it is
  pruned until stage F wires the snoop inputs. The worst paths are
  EA-fetch's (cm_resume, the destination register) into the CPU's sq_a and,
  through the DMU's straight path, the bus controller's read address; none
  goes through the cache's logic.

Tests:
- t_copyback.s (pipelined core only). It sees memory through the program
  bench's new peek registers ($F138/$F13A) and the cache through reads. It
  covers:
  - copyback write misses and hits staying in the cache, and a byte miss
    landing in the longword the line read brought;
  - CPUSHL, and CINVL dropping dirty data;
  - an inhibited read and an inhibited write pushing a dirty line first,
    the write merged;
  - a write-through alias leaving the other dirty longwords;
  - a dirty victim pushed;
  - CPUSHP and CPUSHA;
  - TAS pushing first;
  - MOVE16 writing past a cached line to memory and invalidating it;
  - the table walker reading a dirty descriptor and its U update reaching
    both the line and memory.
- tb_ap040_pipe_dcache.v tests 12-19 cover:
  - the push's longwords exactly;
  - the victim readout race, swept over memory latency and the first
    longword, with the line read back at once;
  - CPUSH over mixed dirty and clean ways and pages;
  - the walker's read hits, misses and write hits;
  - a no-allocate copyback miss as one bus write;
  - the push before an inhibited read;
  - a copyback write meeting its line's fill;
  - a copyback write whose line read errs, on either longword;
  - a write-through write racing the push of its longword, swept;
  - with DE clear, CPUSHA pushing a dirty page-table line while an
    instruction-side translation walks: the walk waits for the push
    (DE clear, then CPUSHA, is how a system turns the cache off).
- tb_ap040_pipe_program.v's write-ordering monitor counts a copyback
  write as landed when the cache takes it, and a push when handed to the
  bus controller.

## Stage R as built

Write-back recovery (8.4.6). The plan put it before copyback; it came after,
and is exercised with copyback on.

- Fill beats (4.6.1): stage C's rules stand. An error on the beat a read
  waits for faults that read; one on another beat abandons the line. A
  copyback write whose line read errs goes to the bus alone (stage D).
- A write's bus error. A write is accepted, and its instruction completes,
  long before the bus answers it, and the bus controller used to drop a
  bus error then. It now tells the DMU (wr_berr). The DMU records every
  write it hands over and holds the fault, with its access error frame's
  fields, until EA-fetch takes it.
  - The record is the whole write:
    - its logical address (a push's is physical), size and data;
    - the SSW's TT and TM, MOVES reported as the precise path reports it;
    - LK;
    - whether it was MOVE16's or an exception frame's write.
    A write crossing a page goes out a byte at a time but is reported
    whole: FA is its first byte (8.4.6.4).
  - A write's frame (Table 8-6, case 3):
    - SSW with RW 0 and ATC 0;
    - FA, and EA = FA;
    - WB1S valid; WB1A = FA;
    - WB1D in the byte lanes written (Table 8-5);
    - WB2S and WB3S clear.
  - A push (case 2):
    - SSW 0: TT 0, TM 0, a longword;
    - FA and WB1A the longword's physical address;
    - WB1S invalid;
    - PD0-PD3 the line.
    The push engine now keeps its line until its last write is done on the
    bus, not only handed over, so the line is still in its registers. The
    other dirty longwords are written all the same.
  - MOVE16 (case 4): TT 1, SIZE line, WB1S valid, PD0-PD3 its four
    longwords, captured as they go.
  - EA-fetch takes it at an instruction boundary, as it takes an
    interrupt. An instruction arriving while the fault is held is itself
    held, and becomes the entry: format $7, vector 2, with its own address
    as the PC, the instruction RTE comes back to.
  - Unlike an interrupt's entry, it does not wait for EX and WB to drain.
    It is an access error's entry, which a precise fault already takes with
    older instructions still in EX: the SR it stacks is forwarded from EX,
    and the frame waits on the registers it needs.
  - Order at a boundary. The fault goes before an interrupt. It goes after
    a trace already owed, and is then held in the trace handler's first
    instruction. The 68040 would set CT instead; this core's RTE does not
    continue it.
- The writes behind it. One fault is held at a time.
  - The writes accepted after the one that erred complete as they would
    have. The 68040 would stop and hand them to the handler in WB2/WB3
    (Table 8-6).
  - A further bus error while one is held is dropped, unless it is an
    exception frame's write.
- A bus error on an exception frame's write is a double fault. The write
  goes out before its entry's vector read, so the DMU holds it by then, and
  the entry departs as the halt (tb_ap040_pipe_dblfault_bus16.v case D).
  The same holds for a frame's write that errs while another fault is held.
- A replaced dirty line (4.6.1, 4.6.2).
  - It is written once the new line is in, as the 68040 orders it; stage D
    wrote it while the new line was being read.
  - If the new line's read errs, the line goes back to its place. Its tag
    is still in the row, since the fill writes the tag last. The longwords
    the fill overwrote are written back from the push engine through port
    B, the fill's own port (the same way and set), and its valid and dirty
    bits are set again. Nothing is written to memory.
  - A snoop that hits the line while it is out means it is not put back,
    as it would have been lost in the cache.
- A fix. A copyback write that goes to the bus alone was handed to the bus
  controller with its physical address in place of the logical one, which
  the prefetch window snoops by (m_la). It now carries its logical address
  (su_la).
- Fit at 25 ns, bus16 top: 40.01 MHz, +0.008 ns, 24,247 ALMs, 35 RAM
  blocks (D: 42.14 MHz, +1.271, 23,840). The worst paths are the core's own
  chain: a pipelined load's forward, EA-fetch's CHK and DIV bound checks,
  its stall, then the IMU's fetch issue. None runs through stage R's logic.
  - Two choices keep them that way. The take is registers only (above),
    since it feeds every exception decision in EA-fetch. MOVE16's capture
    after its fault counts to the line's last longword rather than
    comparing the CPU's address.
  - The restore first wrote back through port A. Quartus 17.0's placer
    aborted on that netlist, twice, on an internal assertion
    (apl_dp.cpp hpwl_cost, `!net._external_pins`). Through port B the
    lookup path gains no muxes.

Tests:
- t_wberr.s (pipelined core only). It uses the program bench's new
  registers: $F156 and $F158, a one-shot bus error on the next data write
  or read to a longword; $F15A, how many cycles that write is held before
  it errs. It covers:
  - a longword write: every frame field, the PC within the run, each
    instruction run once, the stacked condition codes those of the
    instruction before the PC, the write never landed;
  - a word at offset 1, a byte at offset 3 and a longword at offset 2
    (WB1D per Table 8-5);
  - MOVES to FC 1 (TM 1) and to FC 3 (TT 2);
  - a trace owed at the same boundary goes first, and an interrupt at the
    same boundary second (the write's error held 40 cycles, so the MOVE to
    SR lowering the mask arrives before it);
  - a push through a logical alias: FA physical, PD0-PD3 the line, the
    fault taken at the instruction after the CPUSH, the other dirty
    longword in memory;
  - MOVE16's write: TT 1, SIZE line, PD0-PD3, the rest of its line written;
  - a traced CPUSH whose push errs, so that the next instruction arrives
    owing the trace with the fault held: the trace first (test 11);
  - a dirty line going back when its new line's read errs, through the
    bench's new $F158 (a one-shot bus error on a data read): nothing in
    memory, the data in the cache, and CPUSHA then writing all four lines
    (test 12).
- tb_ap040_pipe_dcache.v test 20, at the DMU's ports. It covers:
  - each field, for each size and offset;
  - a write crossing a page;
  - MOVES;
  - the push's line, with CPUSH done only after the push's last write;
  - MOVE16's line, captured before and after its fault;
  - an exception frame's write in a copyback page, sent alone: reported by
    its logical address and marked a double fault;
  - a frame's write erring behind a fault already held;
  - a second fault dropped, and one taken in the cycle the first is let go.
- tb_ap040_pipe_dcache.v test 21. A replaced dirty line goes back, swept
  over the longword asked for and the beat that errs: nothing written, the
  data intact, and every dirty bit back (CPUSHA writes all sixteen). After
  a full line read it is written after the four reads. A line a snoop hit
  while it was out is not put back.
- tb_ap040_pipe_dblfault_bus16.v case D.

## Stage E as built

Throughput, measured first. The program bench's Dhrystone, on the bus16
top with both caches on, spent its 437,103 cycles (stage R) like this:
- 88,333 cycles with a write waiting for the bus: every write is written
  through the 16-bit bus, and the DMU held one;
- about 110,000 with EA-fetch stalled on an outstanding read;
- about 90,000 with EA-calculate empty.
A build of Dhrystone with a copyback data TTR (not in the repository)
removed nearly all the bus's writes (181k busy cycles to 4k) but only 8%
of the time. Its writes still waited about two cycles each for their
translation to pass. Its reads were unchanged.

Built: a store buffer in the DMU.
- Every write for the bus waits there in order for the bus controller: a
  write sent, a push, a copyback write going alone.
  - Four entries. The bus controller holds one more.
  - An entry is the bus write, and for stage R the whole write it belongs
    to (a write crossing a page goes out a byte at a time).
  - A write is accepted when there is room, not when the bus is free.
- The data cache takes a write's update as the write enters, in order, as
  before. So a read that hits never waits for the buffer.
- What waits until the buffer is empty:
  - a read that goes to the bus (straight through, translated, or a byte
    of a crossing read);
  - a line read. A read or copyback write that misses meanwhile is looked
    up again once the buffer is empty, since a buffered write may be to
    its line;
  - on the bus16 top, a fetch from memory (the bus controller's f_req and
    the IMU's f_free both);
  - the table walker (wr_pend);
  - CINV, CPUSH, PFLUSH, PTEST, and a MOVEC to an MMU register, which start
    only with the memory side idle, the buffer included. The 68040 lets
    every earlier write complete first (10.3); software pushes the cache
    before a DMA reads memory;
  - the push engine's end: a push is done when its last write is done on
    the bus, not in the buffer.
- The prefetch window's snoop sees each write as the buffer takes it
  (sb_accept, sb_sla), not as the bus controller does, which is now later.
- Stage R's record is the buffer's head as the bus controller takes it.
- Result, on the program bench (bus16 top, both caches on), stage R to E,
  first phase and the phase with the bench's memory latency:
  - dhry: 437,103 to 374,647 (-14.3%); 538,939 to 441,672 (-18.0%);
  - t_fpu: -1.1%; -1.9%;
  - every other program within 0.3% either way. The small increases come
    from a write reaching the bus controller a cycle later, through the
    buffer. Handing it straight on when the buffer is empty would put the
    CPU's late address back onto the bus controller's registers.
- Fit at 25 ns, bus16 top: 41.89 MHz, +1.127 ns, 24,135 ALMs, 36 RAM
  blocks (R: 40.01 MHz, +0.008, 24,247, 35). The extra block is the buffer's
  address array, which Quartus put in an M10K, as it did the line read
  buffer. The worst paths are a load's data returning into EA-fetch's
  operand (the DMU's answer select), with no buffer logic on them. The
  bus controller's write address now comes from the buffer's registers, not
  the CPU's late address, which was the worst path of an earlier stage R
  fit.

Measured but not built:
- A translated write waits two cycles for its translation to pass. It
  could be accepted in the cycle the translation passes, but that puts the
  MMU's pass logic onto EA-fetch's stall, the core's critical chain. It
  could be accepted before translation, but then a refusal is imprecise,
  and this core restarts the faulting instruction.
- Two reads in flight at the operand port. That needs the CPU's port-B
  protocol and EX's load return reworked, which is the restructuring
  plan's domain (its phase 5 pipelined loads already send a load on as its
  read goes out).
- Back-to-back line transfers. The 16-bit adapter's contract is one stable
  request at a time, with an idle cycle between transactions. A line
  arrives as four transactions, and Dhrystone has 36 data misses. The line
  interface belongs with the native 32-bit/16-byte-line DDR3 interface
  queued after the caches.

Tests:
- tb_ap040_pipe_dcache.v test 22, on a slow bus. It covers:
  - write-through writes taken without waiting for the bus, four behind the
    bus controller's, the next waiting, reaching the bus in order;
  - a hit answered while they wait;
  - a miss, and a read with DE clear, going to the bus only after them,
    returning the buffered write's data;
  - a bus error on a buffered write reporting that write, the writes around
    it landing;
  - the window's snoop at the buffer;
  - a miss behind buffered writes (a read's, and a copyback write's) looked
    up once more when they are out, not over and over. The way it replaces
    is the counter's after exactly those lookups, since each lookup counts
    (4.1).
- tb_ap040_pipe_program.v asserts 10.3 where a maintenance instruction
  starts: no write in the store buffer, none with the bus controller. The
  program cannot see it, as its own reads wait for the buffer. Its $F144
  mode 1 raises the level on vector 32's frame write reaching the bus: a
  frame beat leaving EA-fetch can now come before $F144's own write, still
  buffered, arms it.
- tb_ap040_pipe_dcache.v test 20: the window's snoop names a crossing
  write's last byte's longword.
- The benches that assumed one write held: tb_ap040_pipe_dmuport.v test 2
  (the second write now waits in the buffer), and tb_ap040_pipe_program.v's
  write-ordering monitor (a buffered write has not landed).

## Stage F as built

The platform's side, on the bus16 top (F1).

- Cacheable windows, as the FSM core's wrapper has them
  (rtl/ap040/ap040_tg68k_compat.v): cache_allow_all, cache_z2_ena and two
  Zorro III bases with enables.
  - Applied in ap040_pipe_mmu.v, to the caching mode of every translation,
    the data side's, the instruction side's and its peek's.
  - Outside the windows the mode is inhibited, whatever a TTR or a page
    says: IO, ROM and unconfigured space are never cached.
  - Chip RAM ($000000-$1FFFFF) and configured Fast RAM may be cached.
  - The FSM core's wrapper keeps instruction fetches from chip RAM out of
    its instruction cache, because that cache is not snooped (Phenomena's
    Enigma). This core's is, so chip RAM fetches are cached here.
- Fix, in the MMU's instruction peek (ip_*), there since stage B: the IMU
  takes the peek's caching mode for every prefetch it sends untranslated,
  but with TC.E clear and no TTR match the peek gave the mode and address
  of whatever entry its ATC register last held, not write-through and the
  address itself (MC68040UM 4.3), as the instruction port does. Checked
  against that address, the windows would have cached ROM and IO fetches
  while translation is off -- the card at boot.
- Snoops: snoop_stb/snoop_addr, one a cycle (cpu_wrapper.v's arbiter
  gives exactly that), to both caches' snoop ports.
  - A line holding the address is invalidated (Table 4-3). A dirty data
    line loses its data, the platform boundary above.
  - The data cache's snoop tag copy, pruned since stage D, is back.
- Fit at 25 ns, bus16 top: 40.49 MHz, +0.303 ns, 25,621 ALMs, 42 RAM
  blocks (E: 41.89 MHz, +1.127, 24,135, 36). The six new blocks are both
  caches' snoop tag copies, which nothing drove before; the ALMs are their
  compare paths and the window checks. The worst paths are the fetch
  queue's count into the bus controller's request; none goes through a
  window, a snoop or the peek.

Tests:
- tb_ap040_pipe_dcache.v test 23, with the windows on. It covers:
  - chip RAM cached; ROM space not, even through a copyback TTR, whose
    write then goes to the bus;
  - the Zorro II and both Zorro III windows following their enables;
  - the instruction side's caching mode, and its peek's: untranslated, the
    address itself, write-through or the TTR's mode, then the windows;
  - a dirty line snooped losing its data.
- t_snoop.s (pipelined core only), with the program bench's new $F15C/$F15E
  (a chipset write: memory changed behind the CPU, and snooped). It covers:
  - a stale read after an unsnooped poke, showing the line cached;
  - a chipset write then seen, from the data cache and the instruction
    cache alike.
  The program waits for each chipset write to have happened before reading,
  as a program starting DMA would: the write that starts it may still be in
  the store buffer while a read that hits is answered.
- The program bench now runs with production's windows (chip RAM
  cacheable, the rest not). The other benches that instantiate the bus16
  top tie the windows open.

The card's wrapper (F2): rtl/ap040_pipe/ap040_pipe_tg68k_compat.v, the
pipelined core behind the port set rtl/ap040/ap040_tg68k_compat.v presents
to cpu_wrapper.v -- the same 57 ports, parameters and meaning, so
cpu_wrapper.v takes either core (`AP040_PIPE_CORE`; the instance keeps its
name, which Minimig.sdc's paths start from).

- The core's enable is cpu_wrapper.v's core tick (tick_in): every clock on
  the card (FAST_CLOCK 0), the divided rate under FAST_CLOCK. clkena_in is
  not used: cpu_wrapper.v holds it low while a bus cycle runs, which the
  FSM core needs, and this core stalls on its own. The memory side runs
  every clock.
- The interrupt level: active low, synchronized, taken once stable over two
  clocks, as ap040_core.v takes it; always autovectored.
- RESET drives the reset line (nresetout) through EA-fetch's maintenance
  sequencer: earlier writes and prefetches complete first, the line is up
  for 128 of the core's cycles (ap040_core.v's S_RESET_HOLD), and the
  instruction then retires. The done is taken on a ce edge, since the count
  advances under ce; taken on any clock it was 127 under a divided enable.
- One nmi_ack_toggle flip per level 7 interrupt taken; CACR and VBR as they
  stand; cache_maint_req while CINV/CPUSH runs; debug_halted on a double
  fault.
- mmu_cache_inhibit is held high: this core's caches are the 68040's two,
  and a cache below them would only add a coherency question. post_drain is
  low: the store buffer is this core's own.
- The bus adapter's acknowledge. cpu_wrapper.v's bus enable is not every
  clock, and the adapter's acknowledge is a register that enable updates:
  it stayed up until the next enabled edge, and the bus controller, which
  runs every clock, took it again for the transaction it had started
  meanwhile -- with the old data. The benches' enable is every clock, so
  none saw it; the boot bench did, at its first write. The acknowledge is
  now taken once, in the clock after the enabled edge that set it.
- rtl/ap040_pipe/ap040_pipe.qip lists every file the wrapper is built
  from, and files.qip takes it.
- Fit at 25 ns, bus16 top: 40.49 MHz, +0.304 ns, 25,727 ALMs, 42 RAM
  blocks (F1: +0.303, 25,621, 42). The worst paths are a load's answer
  into EA-fetch's operand and the store buffer's data; none goes through
  RESET's sequencer, the NMI acknowledge or the adapter's acknowledge.
- The full Minimig project with AP040_PIPE_CORE synthesizes but does not
  fit: the fitter needs 4,370 LABs of the device's 4,191 (43,149 ALMs,
  103%), against 39,400 (94%) for the card's last full fit with the FSM
  core. The caches took the bus16 top from 20,401 ALMs (stage A3) to
  25,727; the core has to shed that much, or the card something else,
  before it can run there.

Tests:
- tests/ap040/tb_cpu_wrapper_boot_bridge.v, the sequential suite's boot
  bench (reset to the CIA and serial writes through the production clock
  and bus bridge), built with the pipelined core. run_pipe_verilator.py
  runs it over the phases and DMA modes that suite sweeps, twice: at its
  default (FAST_CLOCK, CORE_DIV 4) and as the card runs today (FAST_CLOCK
  0, the _28 build).
- The boot bench's program starts with RESET, as Kickstart does. For both
  cores, the CPU's reset output must go low once, for 128 of the core's
  cycles (core_enable's), with the CIA writes after its release. The FSM
  core runs an instruction prefetch while its line is low, which the
  manual does not forbid; the pipelined core runs none.
- t_reset.s (pipelined core only): a write, then RESET; the bench checks the
  line rose with no write pending, the bus stayed idle while it was up, it
  was up for 128 of the core's cycles ($F166 counts it), and the
  instruction after it ran once; in user mode a privilege violation and
  nothing driven. RESET leaves the ATC alone.
- The program bench counts level 7 entries against nmi_ack pulses.

## Tests

- Existing programs, corrected: t_cache.s test 6 expects DMA to leave stale
  data, which holds only with snooping suppressed -- run it on a bench mode
  with the snoop off and add the snooped expectation; t_bitfield_cache.s
  writes $00000808 to CACR as "CINV both", which on a 68040 only disables
  the caches (CACR has DE and IE alone) -- use CINVA BC.
- New programs: nonidentity and discontiguous mappings (the set index from
  untranslated bits, the tag from translation); partial byte/word stores
  into present lines; write-through writes to dirty lines; copyback dirty
  data absent from memory until CPUSH or replacement and present after, the
  bench checking memory at points the program names; CINV discarding dirty
  data; locked accesses and MOVE16 on copyback lines; exception stacking
  not allocating; MOVES in each space; dirty page descriptors read and
  updated by the walker; snoops colliding with fills and with the push
  buffer; bus errors on each fill beat and each push beat.
- Differential: the FSM core's cache is write-through only; copyback
  programs are judged on architectural results and on the bench's memory
  checks, not against it.
