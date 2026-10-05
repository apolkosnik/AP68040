//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_cache.v - internal instruction and data caches (milestone G)       //
//                                                                          //
// 4KB per side: 64 sets x 4 ways x 16 byte lines, physically tagged        //
// (sits between the MMU and the 16-bit bus adapter). Write-through with    //
// update-on-hit: writes always go to memory, and a store that fits inside  //
// one aligned longword is merged into the resident line (no allocation on  //
// a miss), so no dirty state ever exists and CPUSH degenerates to CINV.    //
// A store that crosses a line clears both data sets it touches instead.    //
// Cacheable reads must fit inside one aligned longword; misaligned and     //
// line-crossing accesses, walker cycles and cache-inhibited pages bypass   //
// the cache entirely.                                                      //
//                                                                          //
// Invalidate-on-write was the first policy here, and it cost a fifth of   //
// Dhrystone: every store cleared its whole set, so the loads that follow   //
// a struct assignment, a string copy or a stack push all missed again.     //
//                                                                          //
// The instruction cache is not snooped by CPU writes (as on the real       //
// 68040): self-modifying code must execute CINV, which invalidates the     //
// whole selected cache (over-invalidation is architecturally safe).        //
//                                                                          //
// Storage is block RAM throughout, instantiated rather than inferred       //
// (rtl/bram.vhd dpram -> altsyncram): one 512x32 row RAM per way for the   //
// line data, and one wide row per {bank, set} carrying all four tags,      //
// their valid bits and the round-robin victim pointer together.            //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_cache
(
	input             clk,
	input             nreset,
	input             ce,

	input             ie,          // CACR instruction cache enable
	input             de,          // CACR data cache enable

	input             cinv_req,
	input             cinv_ic,
	input             cinv_dc,
	output reg        cinv_done,

	// slave side (from the MMU)
	input             c_req,
	input             c_write,
	input             c_instr,
	input       [1:0] c_size,
	input      [31:0] c_addr,
	input      [31:0] c_wdata,
	input       [2:0] c_fc,
	input             c_nocache,
	// The platform accepts a write to this address into a store queue and
	// will never report a bus error for it, so the store may be acknowledged
	// as soon as it is captured and drained afterwards.  Tie low to disable
	// posting entirely, which is the behaviour without it.
	input             c_post_ok,
	output            c_ack,
	output     [31:0] c_rdata,
	// A posted store is draining: the master side is its own until the
	// acknowledge.  The wrapper uses this to keep the core ticking through
	// the drain (it is waiting on nothing the bus is doing) and to hold the
	// table walker behind it.
	output            sb_busy,

	// master side (to the bus adapter)
	output            m_req,
	output            m_write,
	output            m_instr,
	output      [1:0] m_size,
	output     [31:0] m_addr,
	output     [31:0] m_wdata,
	output      [2:0] m_fc,
	input             m_ack,
	input      [31:0] m_rdata,
	// A physical bus error on the transfer this cache issued.  The core
	// samples the same signal and builds its format-$7 frame; the cache
	// must abandon the transfer rather than re-issue it forever.
	input             m_err,

	// Snoop: an external master (chipset DMA, or the MMU table walker)
	// wrote memory behind the CPU's back.  s_stb is a single CLOCK
	// pulse in THIS clock domain, ce-independent, with s_addr held
	// alongside it; the matching data-cache set is invalidated on that
	// clock.  (A ce-gated snoop port was the 5.1 loss: chipset writes
	// landing while clkena is frozen simply vanished.)
	input             s_stb,
	input      [31:0] s_addr
);

//---------------------------------------------------------------------------
// storage
//---------------------------------------------------------------------------

// The tag row holds everything the lookup needs: the four way tags, their
// valid bits and the round-robin victim pointer.  Keeping validity and LRU
// here rather than in flop arrays puts them in M10K with the tags instead
// of in LABs.  The row is carried by the project's true-dual-port dpram
// (rtl/bram.vhd -> altsyncram), so port B can invalidate on a store while
// port A serves lookups and fills; inferring a second write port from a
// bare array does NOT map to M10K and costs ~3000 ALMs instead.
//
//   row = { rr[1:0], valid[3:0], tag3, tag2, tag1, tag0 }   (94 bits)
localparam TAGW = 22;
localparam ROWW = 2 + 4 + 4*TAGW;

// One data RAM per way, each {bank, set, word}: reading all four at once
// lets the hit be served in the same cycle the tag compare resolves, so a
// hit costs two cycles instead of three.  Same total bits as the single
// {bank, set, way, word} array it replaces.
//
// These are the same instantiated dpram as the tag row above, not
// inferred arrays.  Inference did reach M10K here, but only as a
// synthesis judgement renewed on every recompile, and the failure mode
// is silent: 16K bits of cache line data landing in LABs is ~2000 ALMs
// and a fit that no longer closes, reported as nothing louder than a
// changed resource count.  Instantiating altsyncram puts the block RAM
// in the source instead.  Port A reads, port B fills; the two never
// share a cycle (rd_accept is C_IDLE-only, cd_we is C_FILL-only), so
// the mixed-port read-during-write case cannot arise.
wire [ROWW-1:0] tag_q;
wire [31:0] data_q0, data_q1, data_q2, data_q3;

// RAM control, driven combinationally from the FSM state: the RAMs take
// no resets and their writes are the only ce-gated inputs
wire        tag_we;
wire  [6:0] tag_ridx, tag_widx;
wire [ROWW-1:0] tag_wdat;
wire        inv_we;              // port B: store invalidation
wire        inv_wren;            // port B write strobe (snoops free-run)
wire  [6:0] inv_idx;
wire  [8:0] cd_ridx, cd_widx;
wire  [3:0] cd_we;               // one per way
wire [31:0] cd_wdat;

// Reads free-run: the address is held for the whole request, so a stalled
// ce simply re-reads the same row.  Only the writes are ce-gated.
dpram #(7, ROWW) ctag_ram
(
	.clock     (clk),
	.address_a (tag_we ? tag_widx : tag_ridx),
	.data_a    (tag_wdat),
	.wren_a    (ce & tag_we),
	.q_a       (tag_q),
	.address_b (inv_idx),
	.data_b    ({ROWW{1'b0}}),
	.wren_b    (inv_wren),
	.q_b       ()
);

// The read address is held for the whole request (the MMU may not move
// c_addr before c_ack, which is what the tag row above already relies
// on), so letting port A free-run reproduces the held read register the
// inferred array had: a stalled ce simply re-reads the same word.
dpram #(9, 32) cdata_way0
(
	.clock     (clk),
	.address_a (cd_ridx),
	.data_a    (32'd0),
	.wren_a    (1'b0),
	.q_a       (data_q0),
	.address_b (cd_widx),
	.data_b    (cd_wdat),
	.wren_b    (ce & cd_we[0]),
	.q_b       ()
);

dpram #(9, 32) cdata_way1
(
	.clock     (clk),
	.address_a (cd_ridx),
	.data_a    (32'd0),
	.wren_a    (1'b0),
	.q_a       (data_q1),
	.address_b (cd_widx),
	.data_b    (cd_wdat),
	.wren_b    (ce & cd_we[1]),
	.q_b       ()
);

dpram #(9, 32) cdata_way2
(
	.clock     (clk),
	.address_a (cd_ridx),
	.data_a    (32'd0),
	.wren_a    (1'b0),
	.q_a       (data_q2),
	.address_b (cd_widx),
	.data_b    (cd_wdat),
	.wren_b    (ce & cd_we[2]),
	.q_b       ()
);

dpram #(9, 32) cdata_way3
(
	.clock     (clk),
	.address_a (cd_ridx),
	.data_a    (32'd0),
	.wren_a    (1'b0),
	.q_a       (data_q3),
	.address_b (cd_widx),
	.data_b    (cd_wdat),
	.wren_b    (ce & cd_we[3]),
	.q_b       ()
);

//---------------------------------------------------------------------------
// request classification
//---------------------------------------------------------------------------

wire        ena       = c_instr ? ie : de;
// the access must sit inside one aligned longword to be served
wire        fits_long = (c_size == `AP040_SZ_B) ||
                        (c_size == `AP040_SZ_W && !c_addr[0]) ||
                        (c_size == `AP040_SZ_L && c_addr[1:0] == 2'b00);
wire        bypass    = c_nocache || !ena || c_write || !fits_long;

// Number of bytes following the first byte.  Use a five-bit sum so a
// transfer ending beyond offset 15 cannot wrap before the comparison.
wire  [2:0] write_tail = (c_size == `AP040_SZ_B) ? 3'd0 :
                          (c_size == `AP040_SZ_W) ? 3'd1 : 3'd3;
wire        write_cross_line = ({1'b0, c_addr[3:0]} +
                                 {2'd0, write_tail}) > 5'd15;

wire  [5:0] a_set  = c_addr[9:4];
wire [21:0] a_tag  = c_addr[31:10];
wire  [6:0] a_row  = {c_instr, a_set};

// cacheable read acceptance out of idle (shared with the tag RAM read)
wire rd_accept;
// acceptance of a store that fits in one aligned longword: its lookup
// shares the same tag row and data reads
wire st_accept;

//---------------------------------------------------------------------------
// FSM
//---------------------------------------------------------------------------

localparam C_IDLE  = 3'd0;
localparam C_LOOK  = 3'd1;
localparam C_FERR  = 3'd2;   // aborted fill: invalidate the corrupted row
localparam C_WINV  = 3'd3;   // second-line invalidate owed by a store
localparam C_FILL  = 3'd4;
localparam C_TAGW  = 3'd5;
localparam C_PASS  = 3'd6;
localparam C_SWEEP = 3'd7;   // reset / CINV: walk the rows clearing them

reg   [2:0] cst;
reg   [6:0] sweep_cnt;
reg         sweep_all;   // reset sweep clears both banks
reg         winv_pend;   // a store still owes its second-line invalidate
reg   [5:0] winv_set2;
reg         store_inv_lost;  // a store invalidate that a snoop displaced
reg   [5:0] store_inv_set;
reg         st_chk;          // a fitting store's update-on-hit lookup is live in C_PASS
reg         pass_first;      // the first cycle of C_PASS: the RAM output still
                             // belongs to the acceptance-cycle address
reg   [6:0] r_row;
reg  [21:0] r_tag;
reg   [3:0] r_word;              // {word[1:0]} of the request, plus bank/way
reg   [1:0] r_way;
reg   [1:0] way_fallback;        // victim when the snoop guard forced the miss
reg         r_bank;
reg   [1:0] r_beat;
reg         r_issued;
reg  [31:0] r_addr;
reg   [1:0] r_size;
reg   [1:0] r_off;
reg  [31:0] fill_hold;           // requested longword captured during fill
reg         ack_r;
reg  [31:0] rdata_r;

wire [21:0] t_w0 = tag_q[21:0];
wire [21:0] t_w1 = tag_q[43:22];
wire [21:0] t_w2 = tag_q[65:44];
wire [21:0] t_w3 = tag_q[87:66];
wire v_w0 = tag_q[88];
wire v_w1 = tag_q[89];
wire v_w2 = tag_q[90];
wire v_w3 = tag_q[91];
wire h0 = v_w0 && (t_w0 == r_tag);
wire h1 = v_w1 && (t_w1 == r_tag);
wire h2 = v_w2 && (t_w2 == r_tag);
wire h3 = v_w3 && (t_w3 == r_tag);
wire      look_hit = h0 | h1 | h2 | h3;
wire [1:0] hit_way = h0 ? 2'd0 : h1 ? 2'd1 : h2 ? 2'd2 : 2'd3;

// size extraction from a cached longword (big endian lanes)
function [31:0] lw_extract;
	input [31:0] lw;
	input [1:0] size;
	input [1:0] off;
	begin
		case (size)
			`AP040_SZ_B:
				case (off)
					2'd0: lw_extract = {24'd0, lw[31:24]};
					2'd1: lw_extract = {24'd0, lw[23:16]};
					2'd2: lw_extract = {24'd0, lw[15:8]};
					default: lw_extract = {24'd0, lw[7:0]};
				endcase
			`AP040_SZ_W:
				lw_extract = off[1] ? {16'd0, lw[15:0]} : {16'd0, lw[31:16]};
			default: lw_extract = lw;
		endcase
	end
endfunction
// a store's bytes merged into the cached longword (big endian lanes; the
// data is right-aligned by size, as the bus adapter takes it)
function [31:0] lw_merge;
	input [31:0] lw;
	input [31:0] nw;
	input [1:0] size;
	input [1:0] off;
	begin
		case (size)
			`AP040_SZ_B:
				case (off)
					2'd0: lw_merge = {nw[7:0], lw[23:0]};
					2'd1: lw_merge = {lw[31:24], nw[7:0], lw[15:0]};
					2'd2: lw_merge = {lw[31:16], nw[7:0], lw[7:0]};
					default: lw_merge = {lw[31:8], nw[7:0]};
				endcase
			`AP040_SZ_W:
				lw_merge = off[1] ? {lw[31:16], nw[15:0]} : {nw[15:0], lw[15:0]};
			default: lw_merge = nw;
		endcase
	end
endfunction

// Snoop-vs-fill and snoop-vs-lookup collisions (5.2).  A snoop hitting
// the row of an in-flight fill poisons it: the fill's data may predate
// the snooped write, and the tag writeback would compose valid bits
// from a row image the snoop is concurrently changing.  The fill still
// delivers its data to the CPU (read from memory) but the line is not
// validated.  A snoop hitting the row of a lookup in its acceptance or
// compare cycle forces a miss: the row image under the compare is
// mid-change (mixed-port read-during-write is DONT_CARE on silicon),
// and the refill is always safe.  Both flags are set free-running --
// the snoop is -- and consumed/cleared in the ce domain.
wire snoop_fill_row = s_stb && !r_bank && (s_addr[9:4] == r_row[5:0]);
// The lookup guard has two windows with two different address sources.
// In the ACCEPTANCE cycle only the live address exists, so that compare
// runs on a_set: the MMU's combinational translation of the core's
// request, a long cone (core|mem_addr -> look_snooped, -5.9 ns at
// 114 MHz).  From the tick after acceptance the cache holds r_row,
// captured under ce, and r_row[5:0] IS a_set for the rest of the
// lookup -- so the COMPARE-cycle window uses that, exactly as
// snoop_fill_row already does.  The split moves the window that matters
// (a snoop landing after the tag read is issued but before the compare)
// onto a cache-local, tick-gated register.  The acceptance term keeps
// the long cone, and under a divided clock enable its only exposure is
// the fast cycle in which translation is still settling: a snoop there
// lands its port-B invalidate before the lookup's tag read is issued
// (that happens on the tick), so the read sees the cleared row and
// misses by itself.  tb_ap040_cache_snoop at CE_DIV 4 with +inject_acc
// forces this term blind in exactly that cycle and must still pass;
// +inject_look does the same to the compare-cycle term and must fail.
wire snoop_look_row_acc  = s_stb && !c_instr && (s_addr[9:4] == a_set);
wire snoop_look_row_look = s_stb && !r_bank  && (s_addr[9:4] == r_row[5:0]);
reg  fill_snooped, look_snooped;
always @(posedge clk) begin
	if (!nreset) begin
		fill_snooped <= 0;
		look_snooped <= 0;
	end
	else begin
		// Cleared on every lookup tick, hit or miss: the flag is consumed
		// only by tag_we at C_TAGW, which only a miss reaches, so the hit
		// case is a no-op -- and without look_hit the clear is cst alone,
		// keeping the tag compare (r_tag -> fill_snooped, -1.7 ns at
		// 114 MHz) out of an endpoint whose SET term must stay
		// single-cycle for a snoop on any fast edge.
		if (ce && cst == C_LOOK) fill_snooped <= 0;
		if ((cst == C_FILL || cst == C_TAGW) && snoop_fill_row)
			fill_snooped <= 1;
		// A fitting store's lookup runs through the same two windows: its
		// acceptance cycle and then the whole of C_PASS, since the merge
		// waits for the memory acknowledge.  A snoop on the set anywhere
		// in there suppresses the update; that snoop cleared the row, so
		// nothing stale can remain.
		if (ce && (rd_accept || st_accept)) look_snooped <= 0;
		if (((rd_accept || st_accept) && snoop_look_row_acc) ||
		    (cst == C_LOOK && snoop_look_row_look) ||
		    (cst == C_PASS && st_chk && snoop_look_row_look))
			look_snooped <= 1;
	end
end
//---------------------------------------------------------------------------
// forwarding
//---------------------------------------------------------------------------

// A passed access is forwarded from C_PASS ONLY.  Accepting and acking
// one in C_IDLE used to save a cycle, but it made c_ack combinational in
// c_req -- and c_req carries the ATC compare, so the core's mem_ack (and
// with it the whole 47-level exception-format mux it gates) hung off the
// ATC block RAM output in the same cycle.  That single path cost 5.9 ns
// and was the entire reason the internal caches could not be enabled.
// ap040_mmu already refuses the same shortcut for the same reason -- see
// its c_ack comment.  The cost is one cycle per BYPASSED access (I/O,
// misaligned, cache-inhibited); cacheable traffic goes through C_LOOK
// and is untouched.
wire pass_active = (cst == C_PASS);
wire fill_active = (cst == C_FILL);

// Set when a transfer this cache issued took a bus error; cleared when
// the core withdraws the faulted request.  Without it the level-held
// request would be re-accepted on the very next cycle and re-issued to
// the address that just faulted.
// Posted store buffer.  A passed access otherwise forwards c_addr/c_wdata
// LIVE to the master side, and the core withdraws them the cycle it is
// acknowledged -- so posting needs the request CAPTURED, not just an earlier
// ack.  sb_v also steers the update-on-hit merge, which lands on the memory
// acknowledge and would otherwise merge a c_wdata the core has dropped.
//
// THE DRAIN IS NOT A STATE.  sb_v owns the master side on its own, from the
// capture until the memory acknowledge, and the FSM leaves C_PASS the cycle
// after the merge.  C_IDLE and C_LOOK keep serving cacheable HITS meanwhile
// -- an instruction fetch or a load that finds its line needs nothing the
// drain is using -- and only what does need the master side waits for
// !sb_v: a miss (C_LOOK holds before C_FILL), a bypassed access and the next
// store (both held in C_IDLE).  That ordering is also what keeps the buffer
// architecturally invisible: a read that misses the line a store just
// touched refills only after the store has landed in memory.  The first
// version kept the FSM in C_PASS for the whole drain, and every fetch during
// it waited: on real memory that gave back the entire gain (PERFORMANCE.md,
// "Posted stores, built and measured").
reg         sb_v;
reg  [31:0] sb_addr, sb_wdata;
reg   [1:0] sb_size;
reg   [2:0] sb_fc;

reg  err_hold;
// A cache-inhibited READ that hits a resident line must invalidate it
// while it bypasses (WinUAE dcache040: a hit under CACHE_DISABLE_MMU is
// pushed and invalidated before the uncached access; the icache path
// invalidates likewise).  Leaving the line valid let stale data hit
// again when the mapping turned cacheable.  Stores need nothing extra:
// every accepted store already clears its row (store_inv).  The
// invalidate is recorded here and served through port B whenever the
// port is free; new cacheable reads are held off until it lands, so the
// stale line cannot be re-hit in the window.
reg        pass_ci_chk;   // first C_PASS cycle of a CI read: tags valid
reg        ci_inv_pend;   // a CI hit awaits its row invalidate
reg  [6:0] ci_inv_row;

assign m_req   = fill_active ? 1'b1 : sb_v ? 1'b1 : (pass_active ? c_req : 1'b0);
assign m_write = fill_active ? 1'b0 : (sb_v ? 1'b1 : c_write);
assign m_instr = sb_v ? 1'b0 : c_instr;
assign m_size  = fill_active ? `AP040_SZ_L : (sb_v ? sb_size : c_size);
assign m_addr  = fill_active ? {r_addr[31:4], r_beat, 2'b00}
                             : (sb_v ? sb_addr : c_addr);
assign m_wdata = sb_v ? sb_wdata : c_wdata;
assign m_fc    = sb_v ? sb_fc : c_fc;

// A posted store was acknowledged when it was captured, so C_PASS must not
// hand the core the drain's acknowledge as well.
// A hit is acknowledged IN the compare cycle.  The tag row and all four
// data words are on the RAM outputs in C_LOOK, so the word is ready the
// same cycle the compare resolves; registering it into ack_r/rdata_r cost
// every cached load and fetch a third cycle.  The path is RAM output ->
// compare -> the core's acknowledge, a cone that had to be registered at
// 114 MHz and fits at 40 (the 5.9 ns c_req->c_ack shortcut that was removed
// went through the ATC; this one does not touch c_req).  Under a divided
// enable cst holds C_LOOK until the tick, so the level is stable when the
// core samples it.  Fills still acknowledge from C_TAGW through ack_r.
wire look_ack = (cst == C_LOOK) && look_hit && !look_snooped && !snoop_look_row_look;
assign c_ack   = pass_active ? (sb_v ? ack_r : m_ack) : (ack_r | look_ack);
assign sb_busy = sb_v;
assign c_rdata = pass_active ? m_rdata
                             : (look_ack ? lw_extract(data_hit, r_size, r_off) : rdata_r);

assign rd_accept = (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                   c_req && !ack_r && !c_write && !bypass &&
                   !ci_inv_pend && !store_inv_lost;
assign st_accept = (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                   c_req && !ack_r && !err_hold && c_write && fits_long &&
                   !store_inv_lost && !sb_v;

assign tag_ridx  = a_row;
wire [87:0] tags_next = (r_way == 2'd0) ? {tag_q[87:22], r_tag} :
                        (r_way == 2'd1) ? {tag_q[87:44], r_tag, tag_q[21:0]} :
                        (r_way == 2'd2) ? {tag_q[87:66], r_tag, tag_q[43:0]} :
                                          {r_tag, tag_q[65:0]};
wire  [3:0] val_next  = tag_q[91:88] | (4'd1 << r_way);
wire        sweep_hit = sweep_all || (sweep_cnt[6] ? cinv_ic : cinv_dc);
assign tag_we    = ((cst == C_TAGW) && !fill_snooped && !snoop_fill_row) ||
                   ((cst == C_SWEEP) && sweep_hit);
assign tag_widx  = (cst == C_SWEEP) ? sweep_cnt : r_row;
assign tag_wdat  = (cst == C_SWEEP) ? {ROWW{1'b0}}
                                    : {tag_q[93:92] + 2'd1, val_next, tags_next};

// Port B: a store invalidates the data-bank set it touches, and the next
// set when the transfer crosses the line.  A cleared row needs no
// read-modify-write -- the tags left behind are never consulted without
// their valid bit.  The 68040 leaves the instruction cache alone here.
// Port B invalidates: a snoop takes priority over a store's own
// invalidate, because a missed snoop leaves stale data while a delayed
// store invalidate is picked up again from snoop_pend below.
// Only a store that does not fit in one longword clears rows; a fitting
// one is merged into its line on a hit instead (st_upd below).
wire store_inv = ((cst == C_IDLE) && c_req && c_write && !ack_r &&
                  !store_inv_lost && !sb_v && !fits_long) ||
                 ((cst == C_PASS) && winv_pend) ||
                 (cst == C_WINV);
// Snoop invalidates are FREE-RUNNING (5.1): a chipset write must land
// even while clkena is frozen.  The store-side invalidates stay in the
// ce domain with the FSM that generates them.  The only suppression is
// a sweep zeroing the same row in the same cycle (both write zero; the
// double write is avoided, the effect is identical).
// Port B is the one write port a snoop reaches on ANY fast edge, and the
// sweep suppression below is the only tick-gated state in its select:
// under a divided enable the live term may still be settling in the fast
// cycle after a tick.  A port-A/port-B collision is only possible on a
// tick edge (port A writes under ce), so the LIVE term is used exactly
// there and a registered copy of the same state everywhere else.  Between
// ticks the tick-gated state is constant, so the copy is exact from the
// second fast cycle on; in the first it still shows the row the sweep
// cleared on the tick, and a snoop dropped on a row that has just been
// cleared changes nothing.  With ce high every cycle (the legacy clocking)
// the live term is always selected and behaviour is identical.  This is
// what lets Minimig.sdc give port B's tick-gated sources their four
// cycles (atc_ram/l_row/mem_addr -> ctag_ram~portb_*, -2.5 ns, 60a42def).
reg        snoop_sweep_on_r;
reg  [6:0] snoop_sweep_row_r;
always @(posedge clk) begin
	if (!nreset) begin
		snoop_sweep_on_r  <= 1'b0;
		snoop_sweep_row_r <= 7'd0;
	end
	else begin
		snoop_sweep_on_r  <= (cst == C_SWEEP) && sweep_hit;
		snoop_sweep_row_r <= sweep_cnt;
	end
end
wire snoop_sweep_live = (cst == C_SWEEP) && sweep_hit &&
                        (sweep_cnt == {1'b0, s_addr[9:4]});
wire snoop_sweep_held = snoop_sweep_on_r &&
                        (snoop_sweep_row_r == {1'b0, s_addr[9:4]});
wire snoop_wr  = s_stb && !(ce ? snoop_sweep_live : snoop_sweep_held);
// An aborted refill has already written its beats into the victim way's
// data RAM while that way still carries its PREVIOUS tag and valid bit.
// Only C_TAGW validates a line, so the incoming line stays unreachable --
// but the line it was evicting does NOT: it keeps hitting on its old tag
// over data the dead fill overwrote.  A user-side miss that bus-errors
// mid-fill therefore hands the next SUPERVISOR hit on that row a mixture
// of kernel tag and user data.  Clear the whole row through port B, which
// writes a constant zero and so cannot race the live tag read the way a
// port A read-modify-write would.  Over-invalidation is correctness-safe.
wire fill_err_inv = (cst == C_FERR) && !snoop_wr;
// lowest priority: the zero-row write is idempotent, so waiting is safe
wire ci_inv = ci_inv_pend && !snoop_wr && !store_inv && !store_inv_lost &&
              !fill_err_inv;

assign inv_we   = snoop_wr || store_inv || store_inv_lost || fill_err_inv ||
                  ci_inv;
assign inv_wren = snoop_wr | (ce & (store_inv | store_inv_lost | fill_err_inv |
                                    ci_inv));
assign inv_idx  = snoop_wr        ? {1'b0, s_addr[9:4]} :
                  fill_err_inv    ? r_row :   // the fill's own bank and row
                  ci_inv          ? ci_inv_row :
                  store_inv_lost ? {1'b0, store_inv_set} :
                  (cst == C_IDLE) ? {1'b0, c_addr[9:4]} : {1'b0, winv_set2};
assign cd_ridx   = {c_instr, a_set, c_addr[3:2]};
// the four ways arrive together; the tag compare picks one
wire [31:0] data_hit = (hit_way == 2'd0) ? data_q0 :
                       (hit_way == 2'd1) ? data_q1 :
                       (hit_way == 2'd2) ? data_q2 : data_q3;
// Update-on-hit: a fitting store's tag row and data words were read at
// acceptance and are still on the RAM outputs (the request is level-held
// through C_PASS), so on the memory acknowledge the hit way's word is
// rewritten with the store merged in.  The merge waits for the ack so a
// write that bus-errors leaves the line as it was: memory was not
// written either.  Data port B is the fill's write port; a store never
// runs during a fill, so the two select cleanly.  The guard terms are
// the lookup's own: a snoop on this set in the acceptance cycle or
// during the pass forces the update off, and that snoop cleared the row.
// A POSTED store merges in the first C_PASS cycle.  The core has been
// released, its next request will move the RAM read address, and only in
// this cycle do tag_q/data_q still belong to the store's own set -- so the
// merge cannot wait for the memory acknowledge the way an unposted one does.
// It also does not need to: c_post_ok says the write cannot fault, which is
// the only reason the unposted merge waits.
wire st_upd = (cst == C_PASS) && st_chk &&
              look_hit && !look_snooped && !snoop_look_row_look &&
              (sb_v ? pass_first : (m_ack && !m_err));
assign cd_we     = st_upd ? (4'd1 << hit_way) :
                   ((cst == C_FILL) && r_issued && m_ack) ? (4'd1 << r_way) : 4'd0;
assign cd_widx   = st_upd ? {r_bank, r_row[5:0], r_word[1:0]}
                          : {r_bank, r_row[5:0], r_beat};
assign cd_wdat   = st_upd ? lw_merge(data_hit, sb_v ? sb_wdata : c_wdata,
                                     r_size, r_off) : m_rdata;



always @(posedge clk) begin
	if (!nreset) begin
		// the tag RAM has no reset, so sweep it clear before serving
		// anything: a garbage row would otherwise read back as a hit
		cst <= C_SWEEP;
		sweep_cnt <= 0;
		sweep_all <= 1;
		winv_pend <= 0;
		winv_set2 <= 0;
		err_hold <= 0;
		pass_ci_chk <= 0;
		ci_inv_pend <= 0;
		ci_inv_row <= 0;
		store_inv_lost <= 0;
		store_inv_set <= 0;
		st_chk <= 0;
		cinv_done <= 0;
		r_row <= 0; r_tag <= 0; r_word <= 0; r_way <= 0; r_bank <= 0; way_fallback <= 0;
		r_beat <= 0; r_issued <= 0; r_addr <= 0; r_size <= 0; r_off <= 0;
		fill_hold <= 0; ack_r <= 0; rdata_r <= 0;
		sb_v <= 0; sb_addr <= 0; sb_wdata <= 0; sb_size <= 0; sb_fc <= 0;
		pass_first <= 0;
	end
	else if (ce) begin
		ack_r <= 0;
		pass_first <= 0;
		cinv_done <= 0;
		if (ci_inv) ci_inv_pend <= 0;
		// The drain owns the master side while sb_v is set, so this
		// acknowledge (or error) is its own whatever state the FSM is
		// in.  A fill or a pass is never issued while it is set, so
		// nothing else can be on the bus to claim it.  A drain that
		// bus-errors is dropped here: the core has been released and
		// samples m_err itself, which is the imprecision c_post_ok
		// promises the platform cannot produce.
		if (sb_v && (m_ack || m_err)) sb_v <= 0;

		// A snoop displaced a store's first-set invalidate in its
		// acceptance cycle: remember it and issue it as soon as port B
		// is free.  The second-set (winv) invalidate needs no recording:
		// winv_pend persists until port B actually serves it.  A NEW
		// store is stalled one cycle while a recorded invalidate waits
		// (store_inv's !store_inv_lost term), so the single slot cannot
		// be overwritten.
		if (snoop_wr && (cst == C_IDLE) && c_req && c_write && !ack_r &&
		    !store_inv_lost && !sb_v && !fits_long) begin
			store_inv_lost <= 1;
			store_inv_set  <= c_addr[9:4];
		end
		else if (store_inv_lost && !s_stb)
			store_inv_lost <= 0;

		case (cst)
			C_IDLE: begin
				if (!c_req) err_hold <= 0;
				// Cache maintenance may not report completion while a
				// store it precedes is still in this buffer: an
				// acknowledged store that has not reached memory is not
				// globally complete, and CINV/CPUSH exist to make memory
				// and the caches agree (M68040UM 10.3).  Software pushes,
				// then starts a DMA that would read stale memory.  Waiting
				// for the drain BEFORE the sweep is enough -- the sweep is
				// 128 cycles, so completion is long after the write lands
				// -- and nothing new can enter the buffer meanwhile,
				// because rd_accept and st_accept are already held off
				// while cinv_req is up.
				if (cinv_req && !cinv_done) begin
					// The wait goes INSIDE this branch, not into its
					// condition.  rd_accept and st_accept are gated on
					// !(cinv_req && !cinv_done), so "maintenance pending"
					// must keep the FSM out of the accept branch below as
					// it always did: a condition that falls through while
					// the drain finishes lets a lookup start that those
					// wires say is not happening, and look_snooped -- which
					// is cleared and set on rd_accept/st_accept -- then
					// belongs to a different access than the one in flight.
					// With chipset DMA snooping continuously that desyncs
					// the guard on cached chip RAM.  Hold here instead.
					if (!sb_v) begin
						sweep_cnt <= 0;
						sweep_all <= 0;   // honour the cinv_ic/cinv_dc selects
						cst <= C_SWEEP;
					end
				end
				// A cache-inhibited hit owes a row invalidate.  Accept
				// NOTHING until it lands.  The data RAMs read every
				// cycle, so on paper the FSM could still enter C_LOOK
				// and compare against the not-yet-invalidated tag row
				// using stale data_q.
				//
				// WRITES ARE EXEMPT, and must be.  store_inv asserts
				// combinationally while a store waits in C_IDLE and it
				// blocks ci_inv; holding the store as well made the two
				// block each other with no way out -- a hard wedge, the
				// worst possible failure for a cache.  A store needs no
				// exemption from the guarantee anyway: it never reads
				// data_q, and it clears its own row on acceptance.
				// Exempting it also lets ci_inv fire the moment the FSM
				// leaves C_IDLE.
				//
				// HONEST NOTE: that window could not be demonstrated.
				// ci_inv_pend is raised on the FIRST cycle of C_PASS while
				// the access itself runs to m_ack, so the invalidate lands
				// during the memory latency -- before the FSM can accept
				// anything.  A back-to-back request pair with a port-B
				// stealing snoop swept across the completion (T8) passes
				// with and without this guard.  It is kept as
				// defence-in-depth: it makes the module enforce its own
				// contract instead of depending on the caller inserting a
				// request-low cycle, and it keeps ci_inv_row single-slot
				// so a second CI hit cannot overwrite a pending row and
				// lose its invalidate.  The cost is nil in practice.
				// A store's first-row invalidate can also remain owed
				// after its memory ack if snoops kept port B occupied.
				// Hold reads until it lands.  Accepting on the replay
				// edge reads the old (or undefined) tag row and can
				// return pre-store data even though RAM is up to date.
				else if (c_req && !ack_r && !err_hold &&
				         (c_write || (!ci_inv_pend && !store_inv_lost))) begin
					if (c_write) begin
						if (store_inv_lost || sb_v) begin
							// port B owes a recorded invalidate: hold the
							// store one cycle so its own invalidate cannot
							// be skipped (the request is level-held).
							// Or the single-entry buffer is still
							// draining: hold until it has landed, which
							// keeps stores in order on the bus.
						end
						else begin
						// write-through.  Port B clears the set this store
						// touches in this acceptance cycle; a store crossing
						// the line owes a second one, taken during the pass
						// wait or in C_WINV.  The transfer itself is issued
						// from C_PASS (see pass_active), so no ack can land
						// in this cycle.
						// A store that fits in one longword takes the
						// update-on-hit path: the tag row and data words
						// of its set are read in this cycle (the RAM
						// addresses follow the live request) and compared
						// in C_PASS, where the merge lands on the ack.
						// Only a store that does not fit clears its set
						// through port B here (store_inv).
						st_chk <= fits_long;
						r_row  <= a_row;
						r_tag  <= a_tag;
						r_bank <= c_instr;
						r_word <= {2'd0, c_addr[3:2]};
						r_size <= c_size;
						r_off  <= c_addr[1:0];
						winv_set2 <= c_addr[9:4] + 6'd1;
						winv_pend <= write_cross_line;
						// Post it when the platform guarantees the write
						// cannot fault: capture the request and release the
						// core now, and drain from the buffer in C_PASS.
						pass_first <= 1;
						if (c_post_ok) begin
							sb_v     <= 1;
							sb_addr  <= c_addr;
							sb_wdata <= c_wdata;
							sb_size  <= c_size;
							sb_fc    <= c_fc;
							ack_r    <= 1;
						end
						cst <= C_PASS;
						end
					end
					else if (bypass) begin
						// the tag row read runs in parallel here too, so
						// a cache-inhibited read can detect and kill a
						// resident line while it bypasses.  Held while a
						// store drains: the pass needs the master side,
						// and an uncached read must not overtake a write.
						if (!sb_v) begin
							r_row <= a_row;
							r_tag <= a_tag;
							pass_ci_chk <= c_nocache && !c_write;
							cst <= C_PASS;
						end
					end
					else begin
						// cacheable read: the tag row read runs in parallel
						r_row <= a_row;
						r_tag <= a_tag;
						r_bank <= c_instr;
						r_addr <= c_addr;
						r_size <= c_size;
						r_off <= c_addr[1:0];
						r_word <= {2'd0, c_addr[3:2]};
						cst <= C_LOOK;
					end
				end
			end

			C_PASS: begin
				// the second-set invalidate clears only when port B truly
				// served it; a snoop or a recorded first-set replay owns
				// the port this cycle and winv stays pending
				if (!s_stb && !store_inv_lost) winv_pend <= 0;
				if (pass_ci_chk) begin
					pass_ci_chk <= 0;
					if (look_hit) begin
						ci_inv_pend <= 1;
						ci_inv_row  <= r_row;
					end
				end
				// the store lookup is over with the pass, merged or not
				if (m_err || m_ack || sb_v) st_chk <= 0;
				if (sb_v) begin
					// A posted store: this is its merge cycle
					// (pass_first), the core was released at capture,
					// and the drain now holds the master side by
					// itself.  Back to C_IDLE so hits can be served
					// under it; a still-owed second-line invalidate
					// goes through C_WINV as for any store.
					cst <= (winv_pend && (s_stb || store_inv_lost))
					       ? C_WINV : C_IDLE;
				end
				else if (m_err) begin
					// a passed access faulted: release the bus, but a
					// still-owed invalidate is honoured (invalidating
					// more is always safe under write-through).  A POSTED
					// store that faults here is reported after the core has
					// moved on, which is why c_post_ok must only be asserted
					// where the platform cannot fault a write.
					err_hold <= 1;
					cst <= (winv_pend && (s_stb || store_inv_lost))
					       ? C_WINV : C_IDLE;
				end
				else if (m_ack) cst <= (winv_pend && (s_stb || store_inv_lost))
				                  ? C_WINV : C_IDLE;
			end

			C_FERR: begin
				// The core withdraws the faulting request while this
				// state runs, and only C_IDLE used to watch for that.
				// Release the hold here too, or a request raised again
				// before C_IDLE is reached is blocked forever.
				if (!c_req) err_hold <= 0;
				// a free-running snoop owns port B when it fires; retry
				// until this row's invalidate is the one that lands
				if (!snoop_wr) cst <= C_IDLE;
			end

			C_WINV: begin
				if (!s_stb && !store_inv_lost) begin
					winv_pend <= 0;
					cst <= C_IDLE;
				end
			end

			C_SWEEP: begin
				// one row per cycle; port A writes it (see sweep_hit)
				sweep_cnt <= sweep_cnt + 7'd1;
				if (sweep_cnt == 7'd127) begin
					if (!sweep_all) cinv_done <= 1;
					sweep_all <= 0;
					cst <= C_IDLE;
				end
			end

			C_LOOK: begin
				if (look_hit && !look_snooped && !snoop_look_row_look) begin
					// all four ways were read alongside the tags, so the
					// hit completes here, acknowledged combinationally
					// (look_ack above): one cycle request-to-ack after
					// acceptance
					cst <= C_IDLE;
				end
				else begin
					// Round-robin victim -- unless the guard is what forced
					// this miss.  Then the row image under the compare is
					// the one a snoop is concurrently rewriting (mixed-port
					// read-during-write, DONT_CARE on silicon), and its rr
					// bits are as unreliable as its tags: on silicon that
					// picked SOME way, which is legal, so nothing was ever
					// corrupted; in a faithful sim it picked X, and C_TAGW
					// then composed an X row.  A rotating fallback keeps
					// fairness and removes the dependence on garbage.
					// A miss needs the master side; while a store drains
					// it holds here.  The row cannot turn into a hit
					// meanwhile (nothing but a fill validates a line,
					// and a snoop that lands is latched in look_snooped),
					// so the decision stands and the refill is issued
					// the cycle after the drain's acknowledge -- which
					// also orders it behind the write it may depend on.
					if (!sb_v) begin
						r_way <= (look_snooped || snoop_look_row_look)
						         ? way_fallback : tag_q[93:92];
						if (look_snooped || snoop_look_row_look)
							way_fallback <= way_fallback + 2'd1;
						r_beat <= 0;
						r_issued <= 0;
						cst <= C_FILL;
					end
				end
			end

			C_FILL: begin
				if (m_err) begin
					// 5.4: abandon the fill.  The incoming line is
					// never validated (only C_TAGW validates it), but
					// the beats already written landed in the VICTIM
					// way, whose old tag and valid bit are still live
					// -- so the evicted line would keep hitting over
					// corrupted data.  C_FERR invalidates the row
					// before anything can look at it.  err_hold keeps
					// the still-asserted request from being
					// re-accepted before the core withdraws it.
					r_issued <= 0;
					err_hold <= 1;
					cst <= C_FERR;
				end
				else if (!r_issued) r_issued <= 1;
				else if (m_ack) begin
					// the data RAM write runs in parallel (cd_we)
					if (r_beat == r_addr[3:2]) fill_hold <= m_rdata;
					r_issued <= 0;
					if (r_beat == 2'd3) cst <= C_TAGW;
					else r_beat <= r_beat + 2'd1;
				end
			end

			C_TAGW: begin
				// the tag row write runs in parallel (tag_we): new tag,
				// its valid bit, and the advanced round robin
				rdata_r <= lw_extract(fill_hold, r_size, r_off);
				ack_r <= 1;
				cst <= C_IDLE;
			end

			default: cst <= C_IDLE;
		endcase
	end
end

endmodule
