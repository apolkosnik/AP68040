//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25)                   //
//                                                                          //
// ap040_pipe_imu.v - the instruction memory unit                           //
//                                                                          //
// Between the fetch stage's port A and the bus controller                  //
// (ap040_pipe_membus.v), as the MC68040's IMU sits between its instruction //
// fetch and its bus controller (MC68040UM Figure 4-1): the prefetch window //
// and, behind it, the instruction cache (stage B of                        //
// doc_AP040_PIPELINE_CACHES.md).                                           //
//                                                                          //
// CPU side: ap040_pipe_l1.v's port-A protocol exactly -- a request, a      //
// return with a valid that holds as a level until the next request, and a  //
// new request abandoning one in flight.                                    //
//                                                                          //
// The prefetch stream (2026-09-24). Fetching one word per transaction, and //
// only once the previous word had been handed over, held the pipeline to 4 //
// cycles per instruction on a zero-wait bus where the sequential core ran  //
// at 2. Fetches are aligned longwords into a four-entry window that runs   //
// ahead of the fetch unit whenever the bus has nothing else to do. A       //
// request inside the window is answered the next cycle -- the timing the   //
// L1 array's port A gives, so ap040_inst_fetch.v issues back to back --    //
// and the window slides forward to it. A request for the longword in       //
// flight waits for it; anything else is a redirect: the window is emptied, //
// the read in flight is discarded when it returns, and the stream restarts //
// at the new address. A CPU write that touches the window or the read in   //
// flight empties it, and the bus controller sends writes before the        //
// refetch, so a store into the instruction stream is fetched -- from       //
// memory; the instruction cache, as the 68040's, does not see CPU writes   //
// (4.5), and with it on a store into cached code needs CINV or CPUSH. A    //
// change of privilege empties the window too, so every word carries its    //
// own function code.                                                       //
//                                                                          //
// Restarts: a redirect can change the request while a read is on the bus.  //
// That transaction cannot be recalled, so its result is discarded and the  //
// new address read -- the fetch unit never sees the stale word.            //
//                                                                          //
// Translation (caches stage A). The window is logical. With pages mapped   //
// (pf_xlat) each read the stream wants is translated through the MMU's     //
// instruction port (x_*) from this unit's own register before it goes on   //
// -- never while holding the bus: the table walker may be waiting for a    //
// write the bus controller has still to send -- and a prefetch in the page //
// the port translated last goes out in its own cycle through the MMU's     //
// peek (pk_*). The translation of a read the window no longer wants is     //
// seen through and then discarded. Each translation carries the page's    //
// caching mode (x_cm, pk_cm); untranslated, the mode is the instruction    //
// TTRs' if one matches, else write-through (4.3).                          //
//                                                                          //
// The window's reads (caches stage B). With CACR IE clear, and the cache   //
// idle, each goes to the bus controller in the cycle the window asks, if   //
// its bus is free (f_free) -- as before the cache: writes and reads beat   //
// fetches, since a fetch can always be re-issued and a load cannot -- and  //
// is answered with f_ack or f_flt. With IE set it goes to the cache, which //
// holds it (lk_*) until it answers:                                        //
//   - a cache-inhibited page's read goes to the bus controller as above;   //
//   - otherwise the set is read the cycle after the request -- from a      //
//     register: the request forms late in the cycle, after port A's        //
//     address -- and compared the cycle after that. A hit answers with     //
//     the half-line (PA3) the longword is in: the longword, and the next   //
//     one when that is in the same half-line, both into the window;        //
//   - a miss reads the line, the longword asked for first and the rest     //
//     wrapping (4.1, 4.6.1), each longword a bus controller read. They are //
//     gathered in the line read buffer (fb), each answers the window's     //
//     read for it as it arrives -- the stream reads a line in the order    //
//     the fill does, from the longword it came in at -- and each half-line //
//     is written to its way once both its longwords are in. The line is    //
//     valid only when all four are: an error on any beat abandons the      //
//     line, faulting only a read waiting for that beat; a read waiting for //
//     another is looked up again, and fills again from its own longword.   //
//   - replacement: the first invalid way of the set, else the way a 2-bit  //
//     counter names -- it counts every half-line looked up, and once more  //
//     after it names a way to replace (4.1).                               //
// A read the window has abandoned (pf_kill) is answered at once, unless    //
// its bus read is out: a miss it would have made is not filled. A fill     //
// already under way is completed, for the line is wanted.                  //
//                                                                          //
// CINV/CPUSH on the instruction cache (cm_*), whatever CACR says: once the //
// cache is idle -- no read held, no fill -- all ways at once, or the line  //
// or 4 KB page at the physical address An gave, each set's tag row read    //
// and the matching ways invalidated; a page is every set, 64 rows. There   //
// is no dirty data here, so CPUSH is CINV. The data cache is the DMU's     //
// (stage C); until then it has nothing to do.                              //
//                                                                          //
// Snoops (sn_*): a write by another master invalidates any line holding    //
// its address, found through the arrays' copy of the tag rows, never the   //
// lookup's port -- captured, its set's row read, compared. A snoop of the  //
// line being read keeps it from being made valid. The copy's row read in   //
// the cycle a fill writes that set's tags is undefined, and the snoop then //
// takes the whole set.                                                     //
//                                                                          //
// The bus controller also tells this unit of every write it takes          //
// (w_accept) and where the program wrote (w_sla), for the window's snoop.  //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_imu
(
	input             clk,
	input             nreset,

	// ---- the fetch stage (port A) ----
	input      [31:0] address_a,
	input             en_a,
	output reg [15:0] q_a,
	output reg [15:0] q_a2,     // the word after q_a, when address_a[1] was 0 (phase 8)
	output reg        rvalid_a,
	// with rvalid_a: the fetch faulted. The word handed over is then $4AFC,
	// ILLEGAL -- one word, no redirect, no gather -- so decode needs nothing
	// of the faulted fetch's stale data; rflt_a_bus says the fault was a
	// physical bus error rather than the MMU's.
	output reg        rflt_a,
	output reg        rflt_a_bus,
	// The supervisor bit, for the function code alone.
	input             sup,
	// CINV/CPUSH: empty the window. The refetch that follows arrives in the
	// same cycle and must go to memory, not to what the window held.
	input             pf_inval,
	// Start no fetch: the MMU's registers or ATC are about to change, or the
	// caches are to be maintained, and a fetch must be made wholly before or
	// wholly after.
	input             quiesce,

	// ---- the instruction cache's controls ----
	input             ic_en,        // CACR IE
	input      [31:0] itt0,         // the caching mode of an untranslated fetch
	input      [31:0] itt1,
	// CINV/CPUSH (ap040_ea_fetch.v): held until cm_done, a one-clock pulse
	input             cm_req,
	input             cm_ic,
	input       [1:0] cm_scope,     // 01 line, 10 page, 11 all
	input      [31:0] cm_addr,      // physical
	output reg        cm_done,
	// Another master wrote memory: any line holding the physical address is
	// invalidated (MC68040UM Table 4-3, V5/V6). Nothing on this core's tops
	// writes memory behind the CPU yet; the card's chipset will (stage F).
	input             sn_req,
	input      [31:0] sn_addr,

	// ---- translation ----
	input             pf_xlat,      // translate each stream read first (TC.E)
	output            x_req,        // ap040_pipe_mmu.v's instruction port
	output     [31:0] x_addr,
	output            x_sup,
	input             x_pass,
	input             x_flt,
	input      [31:0] x_pa,
	input       [1:0] x_cm,
	// ...and the MMU's peek (ip_*) at the stream's next longword, so a
	// prefetch in the page translated last goes on in its own cycle
	output     [31:0] pk_addr,
	output            pk_sup,
	input             pk_hit,
	input      [31:0] pk_pa,
	input       [1:0] pk_cm,

	// ---- the bus controller ----
	output            f_req,        // a fetch, now, if the bus is free for it
	output     [31:0] f_addr,       // physical
	output            f_sup,
	input             f_free,       // the bus is free for a fetch this cycle
	input             f_ack,        // the fetch on the bus is answered...
	input      [31:0] f_rdata,
	input             f_flt,        // ...or faulted
	input             f_flt_bus,    // ...with a physical bus error
	input             w_accept,     // a write is taken this cycle
	input      [29:0] w_sla         // the logical longword of the write taken last
);

function ttr_match;   // ap040_pipe_mmu.v's
	input [31:0] ttr;
	input [31:0] la;
	input        s;
	begin
		ttr_match = ttr[15] &&
		            (&((la[31:24] ~^ ttr[31:24]) | ttr[23:16])) &&
		            (ttr[14] || (ttr[13] == s));
	end
endfunction

// A request held over a write's accept cycle. Writes are snooped a cycle
// after they are accepted, from w_sla -- the write's LOGICAL address, as the
// window is -- never from the address the CPU is presenting (2026-09-25):
// that address is EA-fetch's latest signal, and every compare the window
// made against it ran on into pf_base, q_a and the fill -- the bus16 top's
// worst path, reached from each of EA-fetch's address sources in turn as the
// others were taken off it. In the accept cycle itself no fetch is answered,
// neither from the window nor by a fill arriving: the request is kept
// (a_dfr) and answered in the snoop cycle, from the window if the write
// missed it, or fetched again after the write if it did not. A fill that
// arrives meanwhile joins the window, and the snoop empties it if it was
// stale.
reg        a_pend;      // a fetch is wanted and has not been returned
reg [31:0] a_addr;
reg        a_sup;
reg        a_dfr;       // a request held over a write's accept cycle
reg        w_snoop;     // a write was accepted last cycle: snoop it now

// The prefetch stream. Entry i holds the longword whose address bits [3:2]
// are i; the window is pf_cnt longwords from pf_base, never more than four,
// so no two of them share an entry and nothing is ever shifted. The next
// longword to fetch -- and the one in flight, if pf_out -- is always
// pf_base + 4 * pf_cnt: sliding the window forward adds to the one what it
// takes from the other.
localparam [2:0] PF_N = 3'd4;
reg [31:0] pf_q [0:3];
reg [29:0] pf_base;     // longword address
reg  [2:0] pf_cnt;
reg        pf_out;      // a prefetch read is in flight
reg        pf_kill;     // ...for a window since emptied: drop it when it returns
reg        pf_sup;      // the privilege the stream was fetched under
reg        pf_live;     // the fetch unit has asked for something since reset
reg        pf_stop;     // a speculative prefetch faulted: no more until a new request
// The read in flight went out FOR a request -- the fetch unit's miss --
// rather than ahead of one. Only such a read's fault is the fetch unit's:
// a request that finds its longword already in flight as a prefetch has
// it re-issued if that prefetch faults, as the 68040 re-runs a faulted
// prefetch at the point the word is needed (t_exceptions.s tests 138-141).
reg        pf_dem;
// The stream's next read, being translated (pf_xlat): pf_out is set, nothing
// has gone out. It holds the MMU's instruction port at the same address
// until the translation passes or faults, as the MMU requires. After a
// fault the port is down for at least the next cycle, which is what the
// walker waits for (W_DROP): a new translation needs pf_out clear, and that
// clears only at the fault's own edge.
reg        xl_on, xl_sp;
reg [29:0] xl_lw;
assign     x_req  = xl_on;
assign     x_addr = {xl_lw, 2'b00};
assign     x_sup  = xl_sp;

reg         dr_out;     // the window's read is on the bus controller's bus (see dir)

// The window's read, answered this cycle (s_*, below): by the bus
// controller, or by the cache -- which may answer with two longwords, the
// one asked for and the next, when both are in one half-line.
wire        s_ack, s_two, s_flt, s_flt_bus;
wire [31:0] s_rd0, s_rd1;
// This cycle's view of the window, with what arrives now already in it -- a
// request in the same cycle must see it.
wire        pf_ack     = s_ack;
// A fill landing in a write's accept cycle joins the window like any other;
// it is not handed to the fetch waiting for it then (see a_dfr), and the
// snoop a cycle later empties the window if the write reached it -- the old
// instruction behind a store that rewrote it (tb_ap040_pipe_smcdual.v).
wire        pf_app     = pf_ack && !pf_kill;
// A second longword joins where it fits: the window held at most three when
// its read went out (pf_cnt_aft < PF_N), and has only shrunk since.
wire        pf_app2    = pf_app && s_two && (pf_cnt != 3'd3);
wire [29:0] pf_next    = pf_base + {27'd0, pf_cnt};
wire  [1:0] pf_next2   = pf_next[1:0] + 2'd1;
wire  [2:0] pf_cnt1    = pf_cnt + {2'd0, pf_app} + {2'd0, pf_app2};
function [31:0] pf_word1;   // entry i, including what arrives now
	input [1:0] i;
	begin
		pf_word1 = (pf_app  && (i == pf_next[1:0])) ? s_rd0 :
		           (pf_app2 && (i == pf_next2))     ? s_rd1 : pf_q[i];
	end
endfunction
wire [29:0] req_lw     = address_a[31:2];
wire [29:0] req_k      = req_lw - pf_base;
// Not in a write's accept cycle (the request is held, a_dfr), nor from a
// window the snoop is emptying this cycle.
wire        req_hit    = !pf_inval && !w_accept && !w_hits_pf && (req_k < {27'd0, pf_cnt1}) && (sup == pf_sup);
wire [31:0] req_long   = pf_word1(req_lw[1:0]);
// ...or the one still in flight, which it will wait for.
wire        req_onbus  = pf_out && !pf_ack && !pf_kill && (req_lw == pf_next) && (sup == pf_sup);
// A pending request whose longword arrives now. It always is the one: a
// miss restarts the window AT the requested longword with nothing in it, so
// the first answer to join -- after any write empties it again -- is that
// longword. (An address compare here could never be false.)
wire        pend_fill  = a_pend && !a_dfr && pf_app && !w_accept && !w_hits_pf;
// The held request, in the snoop cycle: from the window, or a miss.
wire [29:0] a_lw       = a_addr[31:2];
wire [29:0] dfr_k      = a_lw - pf_base;
wire        dfr_hit    = !pf_inval && !w_hits_pf && (dfr_k < {27'd0, pf_cnt1}) && (a_sup == pf_sup);
wire [31:0] dfr_long   = pf_word1(a_lw[1:0]);
wire        dfr_onbus  = pf_out && !pf_ack && !pf_kill && (a_lw == pf_next) && (a_sup == pf_sup);
// The write accepted last cycle, snooped now from its registered address.
// Its bytes lie in its own longword and at most the next, and it is taken
// to touch both, whatever its size and alignment (restructuring plan, phase
// 5): too much only empties the window once more than it had to, and a
// write next to the instruction stream is rare.
wire [29:0] w_lo       = w_sla;
// Distances into the window, modular like req_k: a window can span the top
// of the address space, and ordered compares against pf_base + 4 missed
// every write into one that did (review 15: a stream from $FFFFFFF8 kept
// stale words at $FFFFFFFC and at $00000000). Either longword a write
// touches can be the one inside -- w_lo + 1 is inside exactly when w_lo is
// the longword below pf_base. The read in flight is always at pf_base +
// pf_cnt with pf_cnt at most three -- one goes out only while pf_cnt_aft <
// PF_N -- so the window's four longwords are the whole range. (A cache
// hit's second longword can lie one past it; it comes from the cache,
// which CPU writes do not reach anyway.)
wire [29:0] w_klo      = w_lo - pf_base;
wire        w_hits_pf  = w_snoop && ((w_klo < {27'd0, PF_N}) || (w_klo == 30'h3FFF_FFFF));
// What the next prefetch would be once this cycle's request is applied. A
// hit leaves base + count where it was; a miss starts the new stream, which
// can go out in the same cycle. Keeping prefetch out of every request cycle
// instead refilled the window only once the fetch unit had drained it --
// 2.5 cycles per instruction on the zero-wait bus rather than the bus's own
// rate. The next read can go in the cycle the cache answers the last
// (c_ans), so it counts what arrives now; a bus controller's answer comes
// in a cycle its bus is not free, and nothing goes then.
wire        c_ans       = pf_ack && !dr_out;
wire [29:0] pf_nx1      = pf_base + {27'd0, pf_cnt1};
wire [29:0] pf_issue_lw = (en_a && !req_hit) ? req_lw : pf_nx1;
wire  [2:0] pf_cnt_aft  = !en_a ? pf_cnt1 : req_hit ? (pf_cnt1 - req_k[2:0]) : 3'd0;
wire        pf_issue_sp = (en_a && !req_hit) ? sup : pf_sup;
// The stream's read translated and still wanted: it goes on when nothing
// comes first.
wire        xl_go       = xl_on && x_pass && !pf_kill;
// The next longword of the stream, as the window stands after this cycle's
// request. Not in a cycle a write is accepted: whether that write lands in
// the window is a thirty-bit compare on an address the CPU has only just
// formed, and deciding the bus on it was the bus16 top's worst path (-4.913
// ns at 25 ns); the write goes first next cycle anyway. Not before the first
// request (review 15): pf_base is zero out of reset, and a read of $0 the
// fetch unit never asked for went out while ce held the core -- one the
// reset PC's fetch then queued behind, for ever if $0 never acknowledges.
// Untranslated it goes on when nothing comes first; translated (pf_xlat)
// its translation starts at once.
// Nor in the cycle a write's snoop empties the window: the next longword
// was pf_base + pf_cnt before it and is pf_base after, and one read for the
// first landed as the second (2026-09-25). The bus controller's own read
// never went -- the write is still waiting there, so the bus is not free --
// but a translation started for it (from caches stage A), and a cache
// lookup would (stage B).
wire        pf_new      = (pf_live || en_a) && (!pf_out || c_ans) && (pf_cnt_aft < PF_N) && !w_accept && !a_dfr &&
                          !pf_inval && !w_hits_pf && (!pf_stop || en_a) && !quiesce;
// Translated, a prefetch -- the stream's next longword, not a request that
// missed -- in the page the MMU's instruction port translated last needs no
// translation of its own: it goes on now, like an untranslated one.
assign      pk_addr     = {pf_nx1, 2'b00};
assign      pk_sup      = pf_sup;
wire        pf_direct   = !pf_xlat || (pk_hit && !(en_a && !req_hit));

// A read going now is the fetch unit's demand if a request of its is still
// waiting after this cycle: a new one the window misses, or one pending
// that nothing arriving now answers.
wire        pf_dem_nx   = en_a ? !req_hit : (a_pend && !pend_fill);

// The window's read this cycle, if it can go: the stream's read once
// translated, or its next read now. (Never both: a read being translated
// holds pf_out, and a new one needs it clear.)
wire        rd_go       = xl_go || (pf_new && pf_direct);
wire [29:0] rd_pa       = xl_go ? x_pa[31:2] : pf_xlat ? pk_pa[31:2] : pf_issue_lw;
wire        rd_sp       = xl_go ? xl_sp : pf_issue_sp;
wire  [1:0] rd_cm       = xl_go ? x_cm : pk_cm;

//---------------------------------------------------------------------------
// the instruction cache
//---------------------------------------------------------------------------

// The read the window has in the cache, held until answered.
reg        lk_v;
reg [29:0] lk_pa;       // physical longword: [29:8] tag, [7:2] set, [1] half, [0] longword
reg        lk_sp;
reg        lk_xl;       // translated: lk_cm is the MMU's
reg  [1:0] lk_cm;
reg        lk_res;      // its set's rows are on the arrays' outputs this cycle
reg        by_out;      // its bypass read (cache-inhibited) is on the bus
// The line being read (a miss's), and its buffer.
reg        fl_act;
reg [27:0] fl_line;     // PA31-PA4
reg        fl_sp;
reg  [1:0] fl_st;       // the longword asked for: read first
reg  [2:0] fl_iss;      // reads sent
reg  [1:0] fl_arr;      // longwords arrived
reg  [3:0] fl_have;
reg [31:0] fb [0:3];
reg  [1:0] fl_way;
reg [87:0] fl_tags;     // the set's tag row, for writing back with the new tag
reg        fl_out;      // a read of the line is on the bus
// Valid bits and the replacement counter, in flops (see the plan, Storage).
reg  [3:0] vld [0:63];
reg  [1:0] rep;
// CINV/CPUSH
localparam [1:0] CM_IDLE = 2'd0, CM_LINE = 2'd1, CM_PAGE = 2'd2;
reg  [1:0] cm_st;
reg  [5:0] cm_set;      // the set to read next...
reg        cm_more;     // ...if any is left
reg  [5:0] cm_rset;     // the set whose row is on the outputs...
reg        cm_rdv;      // ...this cycle
reg        cm_seen;     // done with this request: its done may wait on the data cache's
// snoops: captured (1), their row on the copy's outputs (2)
reg        sn_v1, sn_v2;
reg [27:0] sn_l1, sn_l2;
reg        sn_junk2;    // ...a row read as a fill wrote that set's tags
reg        fl_nov;      // the line being read was snooped: not to be made valid

// With IE clear and nothing left in the cache, the window's reads go to the
// bus controller themselves, exactly as before the cache. Such a read is
// answered to the window whatever IE is by then (dr_out).
wire        dir     = !ic_en && !lk_v && !fl_act;

// the arrays: read by lookups and maintenance, written by the fill -- in
// the cycle each longword arrives, so the last half-line and the tag row
// are in by the edge the line turns valid and the fill lets lookups go on
wire  [5:0] a_set   = (cm_st != CM_IDLE) ? cm_set : lk_pa[7:2];
wire [255:0] a_data;
wire  [87:0] a_tags, s_tags;
wire        aw_dwe, aw_twe;
wire [63:0] aw_data;
wire [87:0] aw_tags;
ap040_pipe_icache_arr u_arr
(
	.clk       (clk),
	.l_set     (a_set),        .l_half (lk_pa[1]),
	.l_data    (a_data),       .l_tags (a_tags),
	.f_set     (fl_line[5:0]), .f_half (fl_aidx[1]),  .f_way (fl_way),
	.f_data    (aw_data),      .f_data_we (aw_dwe),
	.f_tags    (aw_tags),      .f_tags_we (aw_twe),
	.s_set     (sn_l1[5:0]),   .s_tags (s_tags)
);

// The held read's caching mode: the MMU's with its translation, else an
// instruction TTR's, else write-through. Inhibited modes are 10 and 11.
wire        lk_ta   = ttr_match(itt0, {lk_pa, 2'b00}, lk_sp);
wire        lk_tb   = ttr_match(itt1, {lk_pa, 2'b00}, lk_sp);
wire  [1:0] lk_mode = lk_xl ? lk_cm : lk_ta ? itt0[6:5] : lk_tb ? itt1[6:5] : 2'b00;
wire        lk_ci   = lk_mode[1];
// abandoned by the window: answered at once, unless its bus read is out
wire        lk_drop = lk_v && pf_kill && !by_out;

// The lookup: the set is read in the cycle after the request, compared the
// cycle after that.
wire        lk_go   = lk_v && !lk_res && !fl_act && !lk_ci && !lk_drop;
wire [21:0] lk_tag  = lk_pa[29:8];
wire  [3:0] lk_vrow = vld[lk_pa[7:2]];
wire  [3:0] lk_hw;
genvar gw;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : hw
	assign lk_hw[gw] = lk_vrow[gw] && (a_tags[gw*22 +: 22] == lk_tag);
end
endgenerate
wire        lk_hit  = lk_res && (|lk_hw);
wire        lk_miss = lk_res && !(|lk_hw);
wire [63:0] lk_half = lk_hw[0] ? a_data[63:0]    : lk_hw[1] ? a_data[127:64] :
                      lk_hw[2] ? a_data[191:128] : a_data[255:192];
// The way a miss replaces: the first invalid, else the counter's.
wire  [1:0] lk_vict = !lk_vrow[0] ? 2'd0 : !lk_vrow[1] ? 2'd1 : !lk_vrow[2] ? 2'd2 :
                      !lk_vrow[3] ? 2'd3 : rep;

// The line read buffer: the held read's longword, if it is the fill's and in.
wire  [1:0] fl_aidx = fl_st + fl_arr;              // the longword arriving next
wire        fl_ack  = fl_out && f_ack;
wire        fl_flt  = fl_out && f_flt;
wire        lk_infl = lk_v && fl_act && (lk_pa[29:2] == fl_line) && !lk_ci;
wire        lk_fb   = lk_infl && fl_have[lk_pa[1:0]];
wire        lk_fa   = lk_infl && fl_ack && (fl_aidx == lk_pa[1:0]);
wire        lk_ff   = lk_infl && fl_flt && (fl_aidx == lk_pa[1:0]);
// A half-line is written once both its longwords are in; the last longword
// writes the tag.
assign aw_dwe  = fl_ack && fl_have[fl_aidx ^ 2'd1];
assign aw_data = fl_aidx[0] ? {fb[fl_aidx ^ 2'd1], f_rdata} : {f_rdata, fb[fl_aidx ^ 2'd1]};
assign aw_twe  = fl_ack && (fl_arr == 2'd3);
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : twr
	assign aw_tags[gw*22 +: 22] = (fl_way == gw) ? fl_line[27:6] : fl_tags[gw*22 +: 22];
end
endgenerate

// The cache's own bus reads: the line's, or an inhibited read's.
wire        fl_rq   = fl_act && !fl_out && (fl_iss != 3'd4);
wire        by_rq   = lk_v && lk_ci && !fl_act && !by_out && !lk_drop;
// A miss's first read goes in the cycle the miss is seen.
wire        fl_rq0  = lk_miss && !lk_drop;
wire        c_rq    = fl_rq || fl_rq0 || by_rq;
wire [31:0] c_addr  = fl_act ? {fl_line, fl_st + fl_iss[1:0], 2'b00} : {lk_pa, 2'b00};
wire        c_sp    = fl_act ? fl_sp : lk_sp;
wire        by_ack  = by_out && f_ack;
wire        by_flt  = by_out && f_flt;

// the window's read, answered
wire        c_ack   = lk_drop || lk_hit || lk_fb || lk_fa || by_ack;
wire        c_flt   = !lk_drop && (lk_ff || by_flt);
// The window's next read comes in when none is held, or in the cycle the
// held one is answered.
wire        rd_free = dir ? f_free : (ic_en && (!lk_v || c_ack) && !cm_req);
assign s_ack     = (dr_out && f_ack) || c_ack;
assign s_flt     = (dr_out && f_flt) || c_flt;
assign s_flt_bus = f_flt_bus;
assign s_rd0     = dr_out ? f_rdata : lk_hit ? (lk_pa[0] ? lk_half[31:0] : lk_half[63:32]) :
                   lk_fb ? fb[lk_pa[1:0]] : f_rdata;
assign s_rd1     = lk_half[31:0];
assign s_two     = !dr_out && lk_hit && !lk_pa[0];

// to the bus controller
assign f_req  = dir ? rd_go : c_rq;
assign f_addr = dir ? {rd_pa, 2'b00} : c_addr;
assign f_sup  = dir ? rd_sp : c_sp;

// CINV/CPUSH: once the cache is idle.
wire        cm_go   = cm_req && !cm_seen && (cm_st == CM_IDLE) && !lk_v && !fl_act;
wire        cm_rd   = (cm_st != CM_IDLE) && cm_more;
wire  [3:0] cm_hw;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : cmw
	// a line: the whole tag; a page: PA31-PA12 of it
	assign cm_hw[gw] = (cm_st == CM_LINE) ? (a_tags[gw*22 +: 22] == cm_addr[31:10])
	                                      : (a_tags[gw*22 + 2 +: 20] == cm_addr[31:12]);
end
endgenerate

// A snoop's ways, and the line being read.
wire  [3:0] sn_hw;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : snw
	assign sn_hw[gw] = (s_tags[gw*22 +: 22] == sn_l2[27:6]);
end
endgenerate
wire  [3:0] sn_clr  = sn_junk2 ? 4'hF : sn_hw;
wire        sn_fill = sn_v2 && fl_act && (sn_l2 == fl_line);

// The valid bits: at most two sets change in a cycle -- a snoop's, and one
// of a miss's victim, a fill's end and a maintenance row, which never come
// together -- merged when they are the same set, so neither is lost.
wire        fl_done = fl_ack && (fl_arr == 2'd3);
wire        lk_fill = lk_miss && !lk_drop;       // a fill starts (fl_rq0's condition)
wire        o_v     = lk_fill || fl_done || cm_rdv;
wire  [5:0] o_set   = lk_fill ? lk_pa[7:2] : fl_done ? fl_line[5:0] : cm_rset;
wire  [3:0] o_and   = lk_fill ? ~(4'd1 << lk_vict) : fl_done ? 4'hF : ~cm_hw;
wire  [3:0] o_or    = (fl_done && !fl_nov && !sn_fill) ? (4'd1 << fl_way) : 4'd0;
wire        cm_all  = cm_go && cm_ic && (cm_scope == 2'b11);

integer i;
always @(posedge clk) begin
	if (!nreset) begin
		pf_base <= 30'd0; pf_cnt <= 3'd0; pf_out <= 1'b0; pf_kill <= 1'b0; pf_sup <= 1'b1;
		pf_live <= 1'b0; pf_stop <= 1'b0; pf_dem <= 1'b0;
		xl_on <= 1'b0; xl_sp <= 1'b1; xl_lw <= 30'd0;
		rflt_a <= 1'b0; rflt_a_bus <= 1'b0;
		pf_q[0] <= 32'd0; pf_q[1] <= 32'd0; pf_q[2] <= 32'd0; pf_q[3] <= 32'd0;
		a_pend <= 1'b0; a_dfr <= 1'b0; w_snoop <= 1'b0;
		a_addr <= 32'd0; a_sup <= 1'b1;
		rvalid_a <= 1'b0;
		q_a <= 16'd0; q_a2 <= 16'd0;
	end else begin
		w_snoop <= w_accept;
		// ---- the prefetch stream ----
		// What arrives for the live window joins it.
		if (pf_ack) begin
			pf_out <= 1'b0;
			if (pf_kill) pf_kill <= 1'b0;
			else begin
				pf_q[pf_next[1:0]] <= s_rd0;
				if (pf_app2) pf_q[pf_next2] <= s_rd1;
				pf_cnt <= pf_cnt1;
			end
		end
		if (en_a) begin
			pf_live <= 1'b1;
			pf_stop <= 1'b0;
			a_addr <= {address_a[31:1], 1'b0};
			a_sup  <= sup;
			a_dfr  <= w_accept;
			if (w_accept) begin
				// Held over the write's accept cycle; the window is untouched.
				rvalid_a <= 1'b0;
				a_pend   <= 1'b1;
			end else if (req_hit) begin
				// Buffered: answered next cycle, and the window starts here.
				q_a      <= address_a[1] ? req_long[15:0] : req_long[31:16];
				q_a2     <= req_long[15:0];
				rvalid_a <= 1'b1;
				rflt_a   <= 1'b0;
				a_pend   <= 1'b0;
				pf_base  <= req_lw;
				pf_cnt   <= pf_cnt1 - req_k[2:0];
			end else begin
				rvalid_a <= 1'b0;
				a_pend   <= 1'b1;
				pf_base  <= req_lw;
				pf_cnt   <= 3'd0;
				pf_sup   <= sup;
				// In flight and still wanted: it lands at the new base. Any
				// other read in flight belongs to the old window.
				if (pf_out && !pf_ack && !req_onbus) pf_kill <= 1'b1;
			end
		end else if (a_dfr) begin
			// The snoop cycle: the held request, from the window or missed.
			a_dfr <= 1'b0;
			if (dfr_hit) begin
				q_a      <= a_addr[1] ? dfr_long[15:0] : dfr_long[31:16];
				q_a2     <= dfr_long[15:0];
				rvalid_a <= 1'b1;
				rflt_a   <= 1'b0;
				a_pend   <= 1'b0;
				pf_base  <= a_lw;
				pf_cnt   <= pf_cnt1 - dfr_k[2:0];
			end else begin
				pf_base  <= a_lw;
				pf_cnt   <= 3'd0;
				pf_sup   <= a_sup;
				// As for a request answered at once: the read in flight is
				// kept when it is the longword asked for (phase 8 -- the
				// fetch's queue asks in a write's accept cycle often, and
				// killing it here fetched every instruction longword twice
				// behind a read-modify-write).
				if (pf_out && !pf_ack && !dfr_onbus) pf_kill <= 1'b1;
			end
		end else if (a_pend && pf_app && w_accept) begin
			// A pending fetch's answer in a write's accept cycle: it joins the
			// window, and the fetch is answered from there next cycle.
			a_dfr <= 1'b1;
		end else if (pend_fill) begin
			q_a      <= a_addr[1] ? s_rd0[15:0] : s_rd0[31:16];
			q_a2     <= s_rd0[15:0];
			rvalid_a <= 1'b1;
			rflt_a   <= 1'b0;
			a_pend   <= 1'b0;
		end
		// A write into the window, or into the read in flight, empties it;
		// what was pending is fetched again, after the write.
		if (w_hits_pf || pf_inval) begin
			pf_cnt <= 3'd0;
			if (pf_out && !pf_ack) pf_kill <= 1'b1;
		end

		// ---- the window's read ----
		if (s_flt) begin
			// Refused, or a physical bus error. A demand fetch gets the fault
			// with its valid; a speculative prefetch stops the stream -- the
			// fetch unit asking for that longword later restarts it, and
			// faults then, so a fault is only ever raised on a word the
			// program needs. A request that joined a prefetch already in
			// flight is not a demand yet: it stays pending, and the read goes
			// out again for it. Nothing is left in flight to kill, whoever
			// wanted it; a request arriving now supersedes the stream the
			// fault belonged to.
			pf_out  <= 1'b0;
			pf_kill <= 1'b0;
			if (!pf_kill && !en_a) begin
				if (a_pend && pf_dem) begin
					q_a        <= 16'h4AFC;
					q_a2       <= 16'h4AFC;
					rvalid_a   <= 1'b1;
					rflt_a     <= 1'b1;
					rflt_a_bus <= s_flt_bus;
					a_pend     <= 1'b0;
					pf_stop    <= 1'b1;
				end else if (!a_pend) pf_stop <= 1'b1;
			end
		end else if (rd_free && xl_go) begin
			// the stream's read, translated, gone on
			xl_on <= 1'b0;
		end else if (rd_free && pf_new && pf_direct) begin
			pf_out <= 1'b1;
			pf_dem <= pf_dem_nx;
		end
		// Otherwise translated first, from this register (see the header) --
		// whatever the bus is doing, so the translation overlaps it.
		if (pf_new && pf_xlat && !(pf_direct && rd_free)) begin
			pf_out <= 1'b1;
			pf_dem <= pf_dem_nx;
			xl_on  <= 1'b1;  xl_lw <= pf_issue_lw;  xl_sp <= pf_issue_sp;
		end
		// The stream's translation ends. Passed but no longer wanted (the
		// window moved on while it ran): dropped, nothing having gone out.
		// Refused: as a fetch the MMU refused on the bus was -- the fetch
		// unit's demand gets $4AFC and the fault, a speculative read stops
		// the stream.
		if (xl_on && x_pass && pf_kill) begin
			xl_on   <= 1'b0;
			pf_out  <= 1'b0;
			pf_kill <= 1'b0;
		end
		if (xl_on && x_flt) begin
			xl_on   <= 1'b0;
			pf_out  <= 1'b0;
			pf_kill <= 1'b0;
			if (!pf_kill && !en_a) begin
				if (a_pend && pf_dem) begin
					q_a        <= 16'h4AFC;
					q_a2       <= 16'h4AFC;
					rvalid_a   <= 1'b1;
					rflt_a     <= 1'b1;
					rflt_a_bus <= 1'b0;
					a_pend     <= 1'b0;
					pf_stop    <= 1'b1;
				end else if (!a_pend) pf_stop <= 1'b1;
			end
		end
	end
end

// the cache's registers
always @(posedge clk) begin
	cm_done <= 1'b0;
	if (!nreset) begin
		lk_v <= 1'b0; lk_pa <= 30'd0; lk_sp <= 1'b1; lk_xl <= 1'b0; lk_cm <= 2'b00; lk_res <= 1'b0;
		by_out <= 1'b0; dr_out <= 1'b0;
		fl_act <= 1'b0; fl_line <= 28'd0; fl_sp <= 1'b1; fl_st <= 2'd0; fl_iss <= 3'd0; fl_arr <= 2'd0;
		fl_have <= 4'd0; fl_way <= 2'd0; fl_tags <= 88'd0; fl_out <= 1'b0;
		rep <= 2'd0;
		cm_st <= CM_IDLE; cm_set <= 6'd0; cm_more <= 1'b0; cm_rset <= 6'd0; cm_rdv <= 1'b0; cm_seen <= 1'b0;
		sn_v1 <= 1'b0; sn_v2 <= 1'b0; sn_l1 <= 28'd0; sn_l2 <= 28'd0; sn_junk2 <= 1'b0; fl_nov <= 1'b0;
		for (i = 0; i < 64; i = i + 1) vld[i] <= 4'd0;
	end else begin
		// ---- the window's read goes to the bus controller... ----
		if (dir && rd_go && f_free) dr_out <= 1'b1;
		else if (f_ack || f_flt) dr_out <= 1'b0;
		// ---- ...or is answered here, or looked up ----
		lk_res <= lk_go;
		if (lk_go) rep <= rep + 2'd1;           // a half-line looked up
		if (c_ack || c_flt) lk_v <= 1'b0;
		else if (lk_miss) begin
			// the line is read, the longword asked for first
			fl_act  <= 1'b1;
			fl_line <= lk_pa[29:2];
			fl_sp   <= lk_sp;
			fl_st   <= lk_pa[1:0];
			fl_iss  <= 3'd0;
			fl_arr  <= 2'd0;
			fl_have <= 4'd0;
			fl_way  <= lk_vict;
			fl_tags <= a_tags;
			fl_nov  <= 1'b0;
			if (&lk_vrow) rep <= rep + 2'd1;    // after naming the way it replaces
		end
		// ---- ...and the next comes in, in the cycle the last is answered ----
		if (!dir && rd_free && rd_go) begin
			lk_v   <= 1'b1;
			lk_pa  <= rd_pa;
			lk_sp  <= rd_sp;
			lk_xl  <= pf_xlat;
			lk_cm  <= rd_cm;
			lk_res <= 1'b0;
		end

		// ---- the cache's bus reads ----
		if (c_rq && f_free) begin
			if (fl_act)      begin fl_out <= 1'b1; fl_iss <= fl_iss + 3'd1; end
			else if (fl_rq0) begin fl_out <= 1'b1; fl_iss <= 3'd1; end
			else by_out <= 1'b1;
		end
		if (by_ack || by_flt) by_out <= 1'b0;
		if (fl_ack) begin
			fl_out           <= 1'b0;
			fl_arr           <= fl_arr + 2'd1;
			fl_have[fl_aidx] <= 1'b1;
			fb[fl_aidx]      <= f_rdata;
			// the last: the line is valid (its tag and half-line written now)
			if (fl_arr == 2'd3) fl_act <= 1'b0;
		end
		if (fl_flt) begin
			// the line is abandoned; its way stays invalid
			fl_out <= 1'b0;
			fl_act <= 1'b0;
		end

		// ---- CINV/CPUSH ----
		// A row is read each cycle (cm_set, while cm_more) and its ways
		// compared the next (cm_rset): one for a line, sets 0-63 for a page.
		if (!cm_req) cm_seen <= 1'b0;
		cm_rdv  <= cm_rd;
		cm_rset <= cm_set;
		if (cm_go) begin
			if (!cm_ic) begin cm_done <= 1'b1; cm_seen <= 1'b1; end
			else case (cm_scope)
			2'b01: begin cm_st <= CM_LINE; cm_set <= cm_addr[9:4]; cm_more <= 1'b1; end
			2'b10: begin cm_st <= CM_PAGE; cm_set <= 6'd0;         cm_more <= 1'b1; end
			default: begin cm_done <= 1'b1; cm_seen <= 1'b1; end   // all ways: cm_all, below
			endcase
		end
		if (cm_rd) begin
			if (cm_st == CM_PAGE && cm_set != 6'd63) cm_set <= cm_set + 6'd1;
			else cm_more <= 1'b0;
		end
		if (cm_rdv) begin
			if (cm_st == CM_LINE || cm_rset == 6'd63) begin cm_st <= CM_IDLE; cm_done <= 1'b1; cm_seen <= 1'b1; end
		end

		// ---- snoops ----
		sn_v1    <= sn_req;
		sn_l1    <= sn_addr[31:4];
		sn_v2    <= sn_v1;
		sn_l2    <= sn_l1;
		sn_junk2 <= sn_v1 && aw_twe && (fl_line[5:0] == sn_l1[5:0]);
		if (sn_fill) fl_nov <= 1'b1;

		// ---- the valid bits ----
		if (cm_all) for (i = 0; i < 64; i = i + 1) vld[i] <= 4'd0;
		else if (o_v && sn_v2 && (o_set == sn_l2[5:0]))
			vld[o_set] <= ((vld[o_set] & o_and) | o_or) & ~sn_clr;
		else begin
			if (o_v)   vld[o_set]       <= (vld[o_set] & o_and) | o_or;
			if (sn_v2) vld[sn_l2[5:0]] <= vld[sn_l2[5:0]] & ~sn_clr;
		end
	end
end

endmodule
