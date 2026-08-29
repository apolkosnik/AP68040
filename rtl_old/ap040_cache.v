//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_cache.v - internal instruction and data caches (milestone G)       //
//                                                                          //
// 4KB per side: 64 sets x 4 ways x 16 byte lines, physically tagged        //
// (sits between the MMU and the 16-bit bus adapter). Write-through with    //
// invalidate-on-write: writes always go to memory and clear any matching   //
// data cache set, so no dirty state ever exists and CPUSH degenerates to   //
// CINV. Cacheable reads must fit inside one aligned longword; misaligned   //
// and line-crossing accesses, walker cycles and cache-inhibited pages      //
// bypass the cache entirely.                                               //
//                                                                          //
// The instruction cache is not snooped by CPU writes (as on the real       //
// 68040): self-modifying code must execute CINV, which invalidates the     //
// whole selected cache (over-invalidation is architecturally safe).        //
//                                                                          //
// Storage: line data in one synchronous RAM (2 banks x 1024 longwords),    //
// tags in one wide synchronous RAM row per {bank, set} (4 ways of 22       //
// bits), valid bits in flip-flops for single-cycle invalidation.           //
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
	output            c_ack,
	output     [31:0] c_rdata,

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
(* ramstyle = "no_rw_check" *) reg [31:0] cdata0 [0:511];
(* ramstyle = "no_rw_check" *) reg [31:0] cdata1 [0:511];
(* ramstyle = "no_rw_check" *) reg [31:0] cdata2 [0:511];
(* ramstyle = "no_rw_check" *) reg [31:0] cdata3 [0:511];

wire [ROWW-1:0] tag_q;
reg  [31:0] data_q0, data_q1, data_q2, data_q3;

// RAM control (driven combinationally from the FSM state so the arrays
// infer as block RAM: no resets, enable-gated synchronous reads)
wire        tag_we;
wire  [6:0] tag_ridx, tag_widx;
wire [ROWW-1:0] tag_wdat;
wire        inv_we;              // port B: store invalidation
wire        inv_wren;            // port B write strobe (snoops free-run)
wire  [6:0] inv_idx;
wire        cd_rd_en;
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

always @(posedge clk) begin
	if (ce & cd_we[0]) cdata0[cd_widx] <= cd_wdat;
	if (ce & cd_we[1]) cdata1[cd_widx] <= cd_wdat;
	if (ce & cd_we[2]) cdata2[cd_widx] <= cd_wdat;
	if (ce & cd_we[3]) cdata3[cd_widx] <= cd_wdat;
	if (ce & cd_rd_en) begin
		data_q0 <= cdata0[cd_ridx];
		data_q1 <= cdata1[cd_ridx];
		data_q2 <= cdata2[cd_ridx];
		data_q3 <= cdata3[cd_ridx];
	end
end

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
reg   [6:0] r_row;
reg  [21:0] r_tag;
reg   [3:0] r_word;              // {word[1:0]} of the request, plus bank/way
reg   [1:0] r_way;
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
wire snoop_look_row = s_stb && !c_instr && (s_addr[9:4] == a_set);
reg  fill_snooped, look_snooped;
always @(posedge clk) begin
	if (!nreset) begin
		fill_snooped <= 0;
		look_snooped <= 0;
	end
	else begin
		if (ce && cst == C_LOOK && !look_hit) fill_snooped <= 0;
		if ((cst == C_FILL || cst == C_TAGW) && snoop_fill_row)
			fill_snooped <= 1;
		if (ce && rd_accept) look_snooped <= 0;
		if ((rd_accept || cst == C_LOOK) && snoop_look_row)
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

assign m_req   = fill_active ? 1'b1 : (pass_active ? c_req : 1'b0);
assign m_write = fill_active ? 1'b0 : c_write;
assign m_instr = c_instr;
assign m_size  = fill_active ? `AP040_SZ_L : c_size;
assign m_addr  = fill_active ? {r_addr[31:4], r_beat, 2'b00} : c_addr;
assign m_wdata = c_wdata;
assign m_fc    = c_fc;

assign c_ack   = pass_active ? m_ack : ack_r;
assign c_rdata = pass_active ? m_rdata : rdata_r;

assign rd_accept = (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                   c_req && !ack_r && !c_write && !bypass && !ci_inv_pend;

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
wire store_inv = ((cst == C_IDLE) && c_req && c_write && !ack_r &&
                  !store_inv_lost) ||
                 ((cst == C_PASS) && winv_pend) ||
                 (cst == C_WINV);
// Snoop invalidates are FREE-RUNNING (5.1): a chipset write must land
// even while clkena is frozen.  The store-side invalidates stay in the
// ce domain with the FSM that generates them.  The only suppression is
// a sweep zeroing the same row in the same cycle (both write zero; the
// double write is avoided, the effect is identical).
wire snoop_wr  = s_stb && !((cst == C_SWEEP) && sweep_hit &&
                            (sweep_cnt == {1'b0, s_addr[9:4]}));
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
assign cd_rd_en  = rd_accept;                       // issued with the tag read
assign cd_ridx   = {c_instr, a_set, c_addr[3:2]};
assign cd_we     = ((cst == C_FILL) && r_issued && m_ack)
                   ? (4'd1 << r_way) : 4'd0;
assign cd_widx   = {r_bank, r_row[5:0], r_beat};
assign cd_wdat   = m_rdata;

// the four ways arrive together; the tag compare picks one
wire [31:0] data_hit = (hit_way == 2'd0) ? data_q0 :
                       (hit_way == 2'd1) ? data_q1 :
                       (hit_way == 2'd2) ? data_q2 : data_q3;



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
		cinv_done <= 0;
		r_row <= 0; r_tag <= 0; r_word <= 0; r_way <= 0; r_bank <= 0;
		r_beat <= 0; r_issued <= 0; r_addr <= 0; r_size <= 0; r_off <= 0;
		fill_hold <= 0; ack_r <= 0; rdata_r <= 0;
	end
	else if (ce) begin
		ack_r <= 0;
		cinv_done <= 0;
		if (ci_inv) ci_inv_pend <= 0;

		// A snoop displaced a store's first-set invalidate in its
		// acceptance cycle: remember it and issue it as soon as port B
		// is free.  The second-set (winv) invalidate needs no recording:
		// winv_pend persists until port B actually serves it.  A NEW
		// store is stalled one cycle while a recorded invalidate waits
		// (store_inv's !store_inv_lost term), so the single slot cannot
		// be overwritten.
		if (snoop_wr && (cst == C_IDLE) && c_req && c_write && !ack_r &&
		    !store_inv_lost) begin
			store_inv_lost <= 1;
			store_inv_set  <= c_addr[9:4];
		end
		else if (store_inv_lost && !s_stb)
			store_inv_lost <= 0;

		case (cst)
			C_IDLE: begin
				if (!c_req) err_hold <= 0;
				if (cinv_req && !cinv_done) begin
					sweep_cnt <= 0;
					sweep_all <= 0;   // honour the cinv_ic/cinv_dc selects
					cst <= C_SWEEP;
				end
				// A cache-inhibited hit owes a row invalidate.  Accept
				// NOTHING until it lands.  rd_accept alone gated only the
				// data-RAM read enable, so on paper the FSM could still
				// enter C_LOOK and compare against the not-yet-invalidated
				// tag row using stale data_q.
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
				else if (c_req && !ack_r && !err_hold &&
				         (c_write || !ci_inv_pend)) begin
					if (c_write) begin
						if (store_inv_lost) begin
							// port B owes a recorded invalidate: hold the
							// store one cycle so its own invalidate cannot
							// be skipped (the request is level-held)
						end
						else begin
						// write-through.  Port B clears the set this store
						// touches in this acceptance cycle; a store crossing
						// the line owes a second one, taken during the pass
						// wait or in C_WINV.  The transfer itself is issued
						// from C_PASS (see pass_active), so no ack can land
						// in this cycle.
						winv_set2 <= c_addr[9:4] + 6'd1;
						winv_pend <= write_cross_line;
						cst <= C_PASS;
						end
					end
					else if (bypass) begin
						// the tag row read runs in parallel here too, so
						// a cache-inhibited read can detect and kill a
						// resident line while it bypasses
						r_row <= a_row;
						r_tag <= a_tag;
						pass_ci_chk <= c_nocache && !c_write;
						cst <= C_PASS;
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
				if (m_err) begin
					// a passed access faulted: release the bus, but a
					// still-owed invalidate is honoured (invalidating
					// more is always safe under write-through)
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
				if (look_hit && !look_snooped && !snoop_look_row) begin
					// all four ways were read alongside the tags, so the
					// hit completes here: two cycles request-to-ack
					rdata_r <= lw_extract(data_hit, r_size, r_off);
					ack_r <= 1;
					cst <= C_IDLE;
				end
				else begin
					r_way <= tag_q[93:92];   // round-robin victim
					r_beat <= 0;
					r_issued <= 0;
					cst <= C_FILL;
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
