//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_cache.v - internal instruction and data caches (milestone G)       //
//                                                                          //
// 4KB per side: 64 sets x 4 ways x 16 byte lines, physically tagged        //
// (sits between the MMU and the 16-bit bus adapter). Write-through: an      //
// aligned cacheable store updates a resident data-cache word while still   //
// going to memory. Complex or cache-inhibited writes invalidate their      //
// touched sets, so no dirty state ever exists and CPUSH degenerates to      //
// CINV. Cacheable reads must fit inside one aligned longword; misaligned    //
// and line-crossing accesses, walker cycles and cache-inhibited pages       //
// bypass the cache entirely.                                                //
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
	input      [31:0] c_hint_addr,   // next access, one cycle early
	input             c_hint_instr,
	input      [21:0] c_hint_ptag,   // its physical tag, registered by the MMU
	input             c_hint_match,  // the request is that hint
	input             c_hint_wmatch, // ... and the MMU vouches for writing its page now
	input             c_hint_away,   // this clock's hint is not the presented request (P182)
	output            c_fast_ready,  // registered: a hinted data read can hit in one clock now (P182)
	output            c_ack_q,       // registered: this clock's acknowledge is ack_r's (P204)
	input      [31:0] c_ihint_addr,  // the instruction side's hint (P175)
	input      [21:0] c_ihint_ptag,  // its physical tag, registered by the MMU
	input             c_ihint_match, // the request is that hint
	input             c_ihold,       // an instruction request is presented (the arbiter's registered choice)
	input      [31:0] c_wdata,
	input       [2:0] c_fc,
	input             c_nocache,
	// The platform posts writes to this address (it will accept them into
	// its store queue and never report a bus error for them), so the cache
	// may acknowledge such a store on admission and drain it afterwards.
	input             c_post_ok,
	// The same predicate evaluated on the HINT's physical address (the
	// platform derives it from c_hint_ptag, which the MMU registered), so
	// the one-clock posted store below hangs off registers only; a store
	// it acknowledges is posted whatever the live c_post_ok says (the two
	// agree whenever the request is the hint, except when the store queue
	// filled in between, and then the drain simply waits for it).
	input             c_post_ok_hint,
	output            c_ack,
	output            c_posting,     // a posted store is draining (bench attribution)
	output     [31:0] c_rdata,
	// Instruction line sideband: one cycle after an instruction hit is
	// acknowledged, the whole 16-byte physical line it came from (word 0 in
	// [127:96]) with its address.  The core may take any of its words that
	// continue its prefetch stream; it must never treat this as an
	// acknowledge or as a translation of anything.
	output            c_line_stb,
	output     [31:4] c_line_tag,
	output    [127:0] c_line_data,
	// A fill or a posted store is still on the master side after the
	// requester was released; hosts that share one bus with the table
	// walker hold the walker off while this is high.
	output            c_busy,
	// The master-side write being presented was already acknowledged to
	// the requester (posted): the store queue may acknowledge it in the
	// same cycle it captures it, there is no requester path behind it.
	output            m_posted,

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
	// Optional completed-line sideband.  A host that already retained the
	// whole 16-byte physical line may let a fill copy its remaining words
	// locally instead of issuing redundant bus transactions.  The CPU is
	// still acknowledged only after all four words and the tag are committed.
	input             m_line_valid,
	input      [31:4] m_line_tag,
	input     [127:0] m_line_data,
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
// SETW index bits per bank: 6 = 4 KB (the MC68040 geometry), 8 = 16 KB.
// Sets, rows ({bank, set}) and data words ({bank, set, way}) derive from it;
// the tag is everything above the index.
// Interim peripheral build: 8 KB per bank leaves M10Ks for Mac devices.
localparam SETW = 7;
localparam ROWIW = SETW + 1;
localparam DIDXW = SETW + 3;
localparam TAGW = 28 - SETW;
localparam ROWW = 2 + 4 + 4*TAGW;

// Four data RAMs, word-interleaved across the ways: array k holds, at index
// {bank, set, way}, the word w of that way for which (way + w) mod 4 == k.
// A word-wise read (word w of all four ways, for the tag compare to pick
// one) addresses array k at way (k - w) mod 4; a line-wise read of one way
// addresses every array at that way and returns the whole 16-byte line in a
// single cycle.  Same total bits as the plain per-way layout.
(* ramstyle = "no_rw_check" *) reg [7:0] cdata0_b3 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata0_b2 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata0_b1 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata0_b0 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata1_b3 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata1_b2 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata1_b1 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata1_b0 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata2_b3 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata2_b2 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata2_b1 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata2_b0 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata3_b3 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata3_b2 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata3_b1 [0:(1<<DIDXW)-1];
(* ramstyle = "no_rw_check" *) reg [7:0] cdata3_b0 [0:(1<<DIDXW)-1];

// Mirrored data banks read the NEXT word of all four ways on every idle
// read (P170, from P136 without its live-request look paths): a longword
// at 2 mod 4, or a word at 3 mod 4, inside its line is acknowledged in its
// request cycle from the registered hint, as fast_hit does for an aligned
// one.  Speedometer's Pascal pushes word arguments, so every callee's stack
// longwords sit at 2 mod 4 and took the registered path (MOVEM 45k, RTS 12k,
// UNLK 16k acknowledges per Permute(7)).
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata0_b3 [0:(1<<(DIDXW-1))-1];   // data rows only (P175b)
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata0_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata0_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata0_b0 [0:(1<<(DIDXW-1))-1];
reg [31:0] pair_q0;
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata1_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata1_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata1_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata1_b0 [0:(1<<(DIDXW-1))-1];
reg [31:0] pair_q1;
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata2_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata2_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata2_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata2_b0 [0:(1<<(DIDXW-1))-1];
reg [31:0] pair_q2;
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata3_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata3_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata3_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] pairdata3_b0 [0:(1<<(DIDXW-1))-1];
reg [31:0] pair_q3;

wire [ROWW-1:0] tag_q;
wire [ROWW-1:0] hint_tag_q;
// P175: the instruction side's own idle read.  A mirror of the tag RAM and
// of the data banks, written in step with them, is read every clock at the
// instruction hint's row and word, so the fetch's one-clock hit
// (fast_ihit) never competes with the data hint for the architectural
// arrays.  The mirror banks for the pair hit (pairdata) stay as they are.
wire [ROWW-1:0] ihint_tag_q;
// P178: the data hint's NEXT row (data bank), for the one-clock hit of a
// longword that spans two lines: the mirror tag RAM's port B reads its
// tags and the pair banks read its word 0 when the hint sits at word 3.
wire [ROWW-1:0] xhint_tag_q;
wire [31:0] c_ihint_addr_x = c_ihint_addr;
wire  [SETW-1:0] xi_set = c_ihint_addr[SETW+3:4];
wire  [ROWIW-1:0] xi_row = {1'b1, xi_set};
wire  [1:0] xi_w = c_ihint_addr[3:2];
// The mirrors hold only their own bank's rows (the index drops the bank
// bit): a full-depth mirror is 28 M10K and took the device over its RAM
// budget, at which point Quartus quietly turned the framework's OSD and
// palette buffers into 80,000 registers (P175b, first fit).
(* ramstyle = "no_rw_check" *) reg [7:0] idata0_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata0_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata0_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata0_b0 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata1_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata1_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata1_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata1_b0 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata2_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata2_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata2_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata2_b0 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata3_b3 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata3_b2 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata3_b1 [0:(1<<(DIDXW-1))-1];
(* ramstyle = "no_rw_check" *) reg [7:0] idata3_b0 [0:(1<<(DIDXW-1))-1];
reg [31:0] idata_q0, idata_q1, idata_q2, idata_q3;
reg  [31:0] data_q0, data_q1, data_q2, data_q3;

// RAM control (driven combinationally from the FSM state so the arrays
// infer as block RAM: no resets, enable-gated synchronous reads)
wire        tag_we;
wire  [ROWIW-1:0] tag_ridx, tag_widx;
wire [ROWW-1:0] tag_wdat;
wire        inv_we;              // port B: store invalidation
wire        inv_wren;            // port B write strobe (snoops free-run)
wire  [ROWIW-1:0] inv_idx;
wire        cd_rd_en;
wire  [DIDXW-1:0] cd_ridx, cd_widx;
wire  [3:0] cd_we;               // one per way
wire [31:0] cd_wdat;             // single-word writes (fills, aligned merges)
wire [31:0] cd_wdat0, cd_wdat1, cd_wdat2, cd_wdat3;   // per array
wire  [3:0] cd_be0, cd_be1, cd_be2, cd_be3;           // their byte enables (P195)

// Reads free-run: the address is held for the whole request, so a stalled
// ce simply re-reads the same row.  Only the writes are ce-gated.
dpram #(ROWIW, ROWW) ctag_ram
(
	.clock     (clk),
	.address_a (tag_we ? tag_widx : tag_ridx),
	.data_a    (tag_wdat),
	.wren_a    (ce & tag_we),
	.q_a       (tag_q),
	.address_b (inv_wren ? inv_idx : x_row),
	.data_b    ({ROWW{1'b0}}),
	.wren_b    (inv_wren),
	.q_b       (hint_tag_q)
);
// P175: the mirror tag RAM -- port A takes the same writes and otherwise
// reads the instruction hint's row; port B takes the same invalidates.
dpram #(ROWIW, ROWW) ctag_ram_i
(
	.clock     (clk),
	.address_a (tag_we ? tag_widx : xi_row),
	.data_a    (tag_wdat),
	.wren_a    (ce & tag_we),
	.q_a       (ihint_tag_q),
	.address_b (inv_wren ? inv_idx : x_rowp1),
	.data_b    ({ROWW{1'b0}}),
	.wren_b    (inv_wren),
	.q_b       (xhint_tag_q)
);
// In the clock a fast instruction hit acknowledges, the mirror reads that
// way's whole line (every array at {set, way}) for the offer a clock later
// (P174, moved here from the pair banks); the instruction hint's own idle
// read is skipped in that clock (the offer feeds the queue next).
always @(posedge clk) begin
	if (ce & cd_we[0] & cd_widx[DIDXW-1]) begin
		if (cd_be0[3]) idata0_b3[cd_widx[DIDXW-2:0]] <= cd_wdat0[31:24];
		if (cd_be0[2]) idata0_b2[cd_widx[DIDXW-2:0]] <= cd_wdat0[23:16];
		if (cd_be0[1]) idata0_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat0[15:8];
		if (cd_be0[0]) idata0_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat0[7:0];
	end
	if (ce & cd_we[1] & cd_widx[DIDXW-1]) begin
		if (cd_be1[3]) idata1_b3[cd_widx[DIDXW-2:0]] <= cd_wdat1[31:24];
		if (cd_be1[2]) idata1_b2[cd_widx[DIDXW-2:0]] <= cd_wdat1[23:16];
		if (cd_be1[1]) idata1_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat1[15:8];
		if (cd_be1[0]) idata1_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat1[7:0];
	end
	if (ce & cd_we[2] & cd_widx[DIDXW-1]) begin
		if (cd_be2[3]) idata2_b3[cd_widx[DIDXW-2:0]] <= cd_wdat2[31:24];
		if (cd_be2[2]) idata2_b2[cd_widx[DIDXW-2:0]] <= cd_wdat2[23:16];
		if (cd_be2[1]) idata2_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat2[15:8];
		if (cd_be2[0]) idata2_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat2[7:0];
	end
	if (ce & cd_we[3] & cd_widx[DIDXW-1]) begin
		if (cd_be3[3]) idata3_b3[cd_widx[DIDXW-2:0]] <= cd_wdat3[31:24];
		if (cd_be3[2]) idata3_b2[cd_widx[DIDXW-2:0]] <= cd_wdat3[23:16];
		if (cd_be3[1]) idata3_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat3[15:8];
		if (cd_be3[0]) idata3_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat3[7:0];
	end
	if (ce) begin
		idata_q0 <= {idata0_b3[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd0 - xi_w}], idata0_b2[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd0 - xi_w}], idata0_b1[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd0 - xi_w}], idata0_b0[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd0 - xi_w}]};
		idata_q1 <= {idata1_b3[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd1 - xi_w}], idata1_b2[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd1 - xi_w}], idata1_b1[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd1 - xi_w}], idata1_b0[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd1 - xi_w}]};
		idata_q2 <= {idata2_b3[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd2 - xi_w}], idata2_b2[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd2 - xi_w}], idata2_b1[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd2 - xi_w}], idata2_b0[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd2 - xi_w}]};
		idata_q3 <= {idata3_b3[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd3 - xi_w}], idata3_b2[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd3 - xi_w}], idata3_b1[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd3 - xi_w}], idata3_b0[fast_ihit ? {hqi_lo[SETW+3:4], hint_way} : {xi_set, 2'd3 - xi_w}]};
	end
end

wire  [DIDXW-1:0] cd_ridx0, cd_ridx1, cd_ridx2, cd_ridx3;
always @(posedge clk) begin
	if (ce & cd_we[0]) begin
		if (cd_be0[3]) cdata0_b3[cd_widx] <= cd_wdat0[31:24];
		if (cd_be0[2]) cdata0_b2[cd_widx] <= cd_wdat0[23:16];
		if (cd_be0[1]) cdata0_b1[cd_widx]  <= cd_wdat0[15:8];
		if (cd_be0[0]) cdata0_b0[cd_widx]   <= cd_wdat0[7:0];
	end
	if (ce & cd_we[1]) begin
		if (cd_be1[3]) cdata1_b3[cd_widx] <= cd_wdat1[31:24];
		if (cd_be1[2]) cdata1_b2[cd_widx] <= cd_wdat1[23:16];
		if (cd_be1[1]) cdata1_b1[cd_widx]  <= cd_wdat1[15:8];
		if (cd_be1[0]) cdata1_b0[cd_widx]   <= cd_wdat1[7:0];
	end
	if (ce & cd_we[2]) begin
		if (cd_be2[3]) cdata2_b3[cd_widx] <= cd_wdat2[31:24];
		if (cd_be2[2]) cdata2_b2[cd_widx] <= cd_wdat2[23:16];
		if (cd_be2[1]) cdata2_b1[cd_widx]  <= cd_wdat2[15:8];
		if (cd_be2[0]) cdata2_b0[cd_widx]   <= cd_wdat2[7:0];
	end
	if (ce & cd_we[3]) begin
		if (cd_be3[3]) cdata3_b3[cd_widx] <= cd_wdat3[31:24];
		if (cd_be3[2]) cdata3_b2[cd_widx] <= cd_wdat3[23:16];
		if (cd_be3[1]) cdata3_b1[cd_widx]  <= cd_wdat3[15:8];
		if (cd_be3[0]) cdata3_b0[cd_widx]   <= cd_wdat3[7:0];
	end
	if (ce & cd_rd_en) begin
		data_q0 <= {cdata0_b3[cd_ridx0], cdata0_b2[cd_ridx0], cdata0_b1[cd_ridx0], cdata0_b0[cd_ridx0]};
		data_q1 <= {cdata1_b3[cd_ridx1], cdata1_b2[cd_ridx1], cdata1_b1[cd_ridx1], cdata1_b0[cd_ridx1]};
		data_q2 <= {cdata2_b3[cd_ridx2], cdata2_b2[cd_ridx2], cdata2_b1[cd_ridx2], cdata2_b0[cd_ridx2]};
		data_q3 <= {cdata3_b3[cd_ridx3], cdata3_b2[cd_ridx3], cdata3_b1[cd_ridx3], cdata3_b0[cd_ridx3]};
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
// A read that straddles two longwords of the same line (a longword at
// offset 1, 2 or 3, or a word at offset 3, in words 0 to 2) is served from
// the cache too: on a hit the whole line of the hit way is read one cycle
// later and the pair assembled; on a miss the fill captures both words.
wire        span2     = !c_instr && (c_addr[3:2] != 2'd3) &&
                        ((c_size == `AP040_SZ_L && c_addr[1:0] != 2'b00) ||
                         (c_size == `AP040_SZ_W && c_addr[1:0] == 2'b11));
// A read that straddles two LINES (word 3 of one and word 0 of the next)
// is served from the cache when both lines hit: the first lookup takes
// word 3 of the hit way, a second lookup of the next row (next set, the
// next tag when the set wraps) takes word 0, and the pair is assembled
// as for a spanning read.  Either line missing, or any port-B write or
// snoop around the second lookup, falls back to the bypass.  The core
// splits page-crossing accesses itself, so both halves of what arrives
// here translated identically.  (These were 3.8 M reads at 10.5 clocks
// in the Speedometer bracket, 4 % of its cycles.)
// While no request is presented the idle RAM reads index by the hint bus;
// the request bus is registered state only (see the core's mem_hint_addr).
// The core repeats a presented request on the hint bus (mem_hint_addr is
// mem_addr_q while mem_req), so the RAM address inputs index by the hint
// bus alone: no request/hint mux in front of fifty RAM address bits.
// P175: while an instruction request is presented the architectural
// banks follow the instruction hint (which repeats that request), so its
// registered paths (idle_hit, C_LOOK, the line read) index its own row.
wire [31:0] x_addr  = c_ihold ? c_ihint_addr : c_hint_addr;
wire        x_instr = c_ihold;
wire  [SETW-1:0] x_set = x_addr[SETW+3:4];
wire  [SETW:0]   x_row = {x_instr, x_set};
wire  [SETW:0]   x_rowp1 = {1'b0, x_set + {{(SETW-1){1'b0}}, 1'b1}};   // the data hint's next row (P178)
wire  [1:0] x_w     = x_addr[3:2];
wire        xline     = !c_instr && (c_addr[3:2] == 2'd3) &&
                        ((c_size == `AP040_SZ_L && c_addr[1:0] != 2'b00) ||
                         (c_size == `AP040_SZ_W && c_addr[1:0] == 2'b11));
// a word at offset 1 sits inside one longword: extracted, not spanned
wire        fits_lane = fits_long ||
                        (!c_instr && c_size == `AP040_SZ_W && c_addr[1:0] == 2'b01);
wire        bypass    = c_nocache || !ena || c_write ||
                        !(fits_lane || span2 || xline);

// Number of bytes following the first byte.  Use a five-bit sum so a
// transfer ending beyond offset 15 cannot wrap before the comparison.
wire  [2:0] write_tail = (c_size == `AP040_SZ_B) ? 3'd0 :
                          (c_size == `AP040_SZ_W) ? 3'd1 : 3'd3;
wire        write_cross_line = ({1'b0, c_addr[3:0]} +
                                 {2'd0, write_tail}) > 5'd15;

wire  [SETW-1:0] a_set  = c_addr[SETW+3:4];
wire  [TAGW-1:0] a_tag  = c_addr[31:SETW+4];
wire [ROWIW-1:0] a_row  = {c_instr, a_set};

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

`ifdef AP040_EXPERIMENTAL_XSTORE
localparam C_XSTORE_LOOK = 4'd8, C_XSTORE_WRITE = 4'd9;
reg [3:0] cst;
reg cross_store, cross_fault, cross_second_lost;
wire cross_lookup = cst == C_XSTORE_LOOK;
wire cross_second = cst == C_XSTORE_WRITE;
// Only host-qualified non-faulting posted memory uses delayed hit merges.
// Uncacheable and potentially faulting writes retain both-set invalidation.
wire cross_accept = c_write && !c_instr && de && !c_nocache &&
                    write_cross_line && c_post_ok;
`else
reg [2:0] cst;
wire cross_store = 1'b0, cross_fault = 1'b0, cross_second_lost = 1'b0;
wire cross_lookup = 1'b0, cross_second = 1'b0, cross_accept = 1'b0;
`endif
reg   [ROWIW-1:0] sweep_cnt;
reg         sweep_all;   // reset sweep clears both banks
reg         winv_pend;   // a store still owes its second-line invalidate
reg   [SETW-1:0] winv_set2;
reg         store_inv_lost;  // a store invalidate that a snoop displaced
reg   [SETW-1:0] store_inv_set;
reg   [ROWIW-1:0] r_row;
reg  [TAGW-1:0] r_tag;
reg   [3:0] r_word;              // {word[1:0]} of the request, plus bank/way
reg   [1:0] r_way;
reg         r_bank;
reg   [1:0] r_beat;
reg         r_issued;
reg  [31:0] r_addr;
reg   [1:0] r_size;
reg   [1:0] r_off;
reg  [31:0] fill_hold;           // requested longword captured during fill
reg  [31:0] fill_hold2;          // ... and its successor, for a spanning read
reg         r_span2;             // the request straddles two longwords
reg         look2;               // C_LOOK's second cycle: the line read is in
reg         r_xline;             // the request straddles two lines
reg         xlook;               // C_LOOK's second lookup (the next line) is in
reg         xsnooped;            // a snoop touched the next line's row during its read
reg  [SETW-1:0] r_setB;          // the next line's set ...
reg  [TAGW-1:0] r_tagB;          // ... and tag
reg   [1:0] r_hway;              // the way the first C_LOOK cycle found
reg   [1:0] fill_cnt;            // beats completed in this fill (requested word first)
reg   [2:0] r_fc;                // the fill's own function code (the requester may be gone)
reg         sline_ready;         // a spanning store's line read has completed
reg         sp_held;             // ... and its two merge words are held (P187)
reg  [31:0] sp_h0, sp_h1;
reg         fill_acked;          // the requester already has its data
reg  [31:0] r_wdata;             // captured aligned store data for hit update
reg         ack_r;
reg  [31:0] rdata_r;
reg         pass_store_chk;      // C_PASS owns an aligned store tag lookup
reg         post_active;         // C_PASS drains a store already acknowledged
reg  [31:0] p_addr, p_wdata;     // ... from these copies, not the pins
reg   [1:0] p_size;
reg   [2:0] p_fc;

// One-longword sequential instruction lookahead.  A normal I-cache hit has
// already identified the resident way and leaves the data RAM otherwise idle
// while its registered acknowledge is returned.  Use that cycle to read the
// next longword in the same 16-byte line, then retain it in this small buffer.
// If the very next accepted cache request is for that physical longword it can
// be acknowledged directly from C_IDLE, without another synchronous tag/data
// lookup.  The buffer is strictly a cache-hit latency optimization: it neither
// advances architectural PC nor creates a new memory request.
// Instruction line buffer.  A normal I-cache hit has identified the
// resident way; the following cycle reads that way's whole line (all four
// arrays at index {1, set, way}) into this private copy.  Any later
// instruction request inside the line is acknowledged from C_IDLE without a
// tag or data lookup, and the line is also offered to the core as a
// sideband so its prefetch queue can take the remaining words at once.
reg          iline_pending;
reg          iline_valid;
reg    [1:0] iline_way;
reg   [27:0] iline_tag;          // physical address [31:4]
reg  [127:0] iline_data;         // word 0 in [127:96]
reg          iline_stb;          // offer the line to the core this cycle
reg          iline_stb_pend;     // a buffer hit acknowledged: offer next cycle

wire [TAGW-1:0] t_w0 = tag_q[TAGW-1:0];
wire [TAGW-1:0] t_w1 = tag_q[2*TAGW-1:TAGW];
wire [TAGW-1:0] t_w2 = tag_q[3*TAGW-1:2*TAGW];
wire [TAGW-1:0] t_w3 = tag_q[4*TAGW-1:3*TAGW];
wire v_w0 = tag_q[4*TAGW];
wire v_w1 = tag_q[4*TAGW+1];
wire v_w2 = tag_q[4*TAGW+2];
wire v_w3 = tag_q[4*TAGW+3];
// In idle, a qualified physical request may reuse an already-settled RAM
// read. The same comparator/mux serves this and the ordinary registered
// lookup; no CPU acknowledgement is combinational in the MMU request.
wire [TAGW-1:0] compare_tag = (cst == C_IDLE) ? a_tag : (xlook ? r_tagB : r_tag);
wire h0 = v_w0 && (t_w0 == compare_tag);
wire h1 = v_w1 && (t_w1 == compare_tag);
wire h2 = v_w2 && (t_w2 == compare_tag);
wire h3 = v_w3 && (t_w3 == compare_tag);
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
				case (off)
					2'd0: lw_extract = {16'd0, lw[31:16]};
					2'd1: lw_extract = {16'd0, lw[23:8]};
					2'd2: lw_extract = {16'd0, lw[15:0]};
					default: lw_extract = {16'd0, lw[7:0], 8'd0};   // never: spanned
				endcase
			default: lw_extract = lw;
		endcase
	end
endfunction

// Extract a spanning access from two consecutive big-endian longwords.
function [31:0] span_extract;
	input [63:0] pair;      // {word w, word w+1}
	input  [1:0] size;
	input  [1:0] off;
	reg   [63:0] sh;
	begin
		sh = pair << (8 * off);
		span_extract = (size == `AP040_SZ_W) ? {16'd0, sh[63:48]} : sh[63:32];
	end
endfunction

// Merge right-aligned store data into a cached big-endian longword.  This is
// used only for fits_long stores, so a word is even and no operand crosses a
// longword boundary.
function [31:0] lw_merge;
	input [31:0] old_lw;
	input [31:0] new_data;
	input  [1:0] size;
	input  [1:0] off;
	begin
		case (size)
			`AP040_SZ_B:
				case (off)
					2'd0: lw_merge = {new_data[7:0], old_lw[23:0]};
					2'd1: lw_merge = {old_lw[31:24], new_data[7:0], old_lw[15:0]};
					2'd2: lw_merge = {old_lw[31:16], new_data[7:0], old_lw[7:0]};
					default: lw_merge = {old_lw[31:8], new_data[7:0]};
				endcase
			`AP040_SZ_W:
				case (off)
					2'd0: lw_merge = {new_data[15:0], old_lw[15:0]};
					2'd1: lw_merge = {old_lw[31:24], new_data[15:0], old_lw[7:0]};
					2'd2: lw_merge = {old_lw[31:16], new_data[15:0]};
					default: lw_merge = old_lw;   // never: spanned
				endcase
			default: lw_merge = new_data;
		endcase
	end
endfunction

// Merge a spanning store into two consecutive big-endian longwords.
function [63:0] span_merge;
	input [63:0] pair;      // {word w, word w+1}
	input [31:0] new_data;
	input  [1:0] size;
	input  [1:0] off;
	reg   [63:0] mask, val;
	begin
		mask = (size == `AP040_SZ_W) ? 64'h0000_0000_0000_FFFF : 64'h0000_0000_FFFF_FFFF;
		val  = (size == `AP040_SZ_W) ? {48'd0, new_data[15:0]} : {32'd0, new_data};
		// byte k of the pair sits at [63-8k]; the operand's last byte is
		// off+3 (long) or off+1 (word)
		mask = (size == `AP040_SZ_W) ? (mask << (48 - 8 * off)) : (mask << (32 - 8 * off));
		val  = (size == `AP040_SZ_W) ? (val  << (48 - 8 * off)) : (val  << (32 - 8 * off));
		span_merge = (pair & ~mask) | val;
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
wire snoop_fill_row = s_stb && !r_bank && (s_addr[SETW+3:4] == r_row[SETW-1:0]);
wire snoop_look_row = s_stb && !c_instr && (s_addr[SETW+3:4] == a_set);
wire snoop_xrow     = s_stb && (s_addr[SETW+3:4] == r_setB);
// A next-line snoop may arrive while CE is stopped. Retain it until
// the read resumes so the second-line result cannot use stale RAM data.
reg xline_snoop_pending;
always @(posedge clk) begin
    if (!nreset) xline_snoop_pending <= 0;
    else begin
        if (ce && rd_accept) xline_snoop_pending <= 0;
        if (cst == C_LOOK && xlook && snoop_xrow) xline_snoop_pending <= 1;
    end
end
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

// Aligned writes to an enabled, cacheable data line may update a matching
// cached word.  They still pass through to memory; only the needless
// invalidate/refill cycle is removed.  Every other write keeps the existing
// conservative set-invalidate path.
wire store_lane      = fits_long ||
                       (c_size == `AP040_SZ_W && c_addr[1:0] == 2'b01);
wire store_update_ok = c_write && !c_instr && de && !c_nocache && store_lane;
// A store that straddles two longwords of one line updates both words of
// a matching line (MC68040UM 4.3.1.1: write-through stores update matching
// lines) instead of invalidating its whole set.
wire store_update2   = c_write && !c_instr && de && !c_nocache && span2;
wire store_any_update = store_update_ok || store_update2 || cross_accept;

// Set when a transfer this cache issued took a bus error; cleared when
// the core withdraws the faulted request.  Without it the level-held
// request would be re-accepted on the very next cycle and re-issued to
// the address that just faulted.
reg  err_hold;
wire store_lookup_accept = (cst == C_IDLE) && c_req && !ack_r &&
	                         !err_hold && !store_inv_lost && store_any_update;
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
reg  [ROWIW-1:0] ci_inv_row;

// A line can become valid while the first beat is already in flight.  Never
// abandon an issued transfer; after it completes, copy later beats directly
// from the retained line one word per cache clock.  Exact physical-tag
// matching makes the sideband harmless for hosts that retain some other line.
wire fill_line_match = fill_active && !r_issued && m_line_valid &&
                       (m_line_tag == r_addr[31:4]);
wire [31:0] fill_line_word = (r_beat == 2'd0) ? m_line_data[127:96] :
                             (r_beat == 2'd1) ? m_line_data[95:64]  :
                             (r_beat == 2'd2) ? m_line_data[63:32]  :
                                                        m_line_data[31:0];
wire fill_line_write = fill_line_match;

assign m_req   = fill_active ? !fill_line_match :
                 (pass_active ? (post_active | c_req) : 1'b0);
assign m_write = fill_active ? 1'b0 : (post_active | c_write);
assign m_instr = fill_active ? r_bank : (post_active ? 1'b0 : c_instr);
assign m_size  = fill_active ? `AP040_SZ_L : (post_active ? p_size : c_size);
assign m_addr  = fill_active ? {r_addr[31:4], r_beat, 2'b00} :
                 (post_active ? p_addr : c_addr);
assign m_wdata = post_active ? p_wdata : c_wdata;
assign m_fc    = fill_active ? r_fc : (post_active ? p_fc : c_fc);

// A data read whose idle read (set up by the core's hint) holds the
// matching word is acknowledged in its request cycle: the tag compare
// runs against the hint's registered physical tag (c_hint_ptag) and the
// MMU vouches that the request is that hint (c_hint_match), so nothing
// in the acknowledge path starts at the live translation.  Instruction
// fetches keep the registered acknowledge (their consumer is the fetch
// queue's ring write, the tightest path in the core).
wire fast_hit;
wire [31:0] fast_data;
wire fast_ihit;
wire [1:0] hint_way;
wire [ROWIW-1:0] hq_irow;
reg  pair_idle_valid;
reg  iline_pair_pending;
// Keep writes identical to the architectural cache banks. Only the
// independent synchronous read address differs; cache capacity is unchanged.
wire [1:0] pair_read_word = x_w + 2'd1;
wire [SETW:0] pair_row = (x_w == 2'd3 && !x_instr) ? x_rowp1 : x_row;   // P178
always @(posedge clk) begin
    if (ce & cd_we[0] & !cd_widx[DIDXW-1]) begin
		if (cd_be0[3]) pairdata0_b3[cd_widx[DIDXW-2:0]] <= cd_wdat0[31:24];
		if (cd_be0[2]) pairdata0_b2[cd_widx[DIDXW-2:0]] <= cd_wdat0[23:16];
		if (cd_be0[1]) pairdata0_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat0[15:8];
		if (cd_be0[0]) pairdata0_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat0[7:0];
	end
    if (ce & cd_we[1] & !cd_widx[DIDXW-1]) begin
		if (cd_be1[3]) pairdata1_b3[cd_widx[DIDXW-2:0]] <= cd_wdat1[31:24];
		if (cd_be1[2]) pairdata1_b2[cd_widx[DIDXW-2:0]] <= cd_wdat1[23:16];
		if (cd_be1[1]) pairdata1_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat1[15:8];
		if (cd_be1[0]) pairdata1_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat1[7:0];
	end
    if (ce & cd_we[2] & !cd_widx[DIDXW-1]) begin
		if (cd_be2[3]) pairdata2_b3[cd_widx[DIDXW-2:0]] <= cd_wdat2[31:24];
		if (cd_be2[2]) pairdata2_b2[cd_widx[DIDXW-2:0]] <= cd_wdat2[23:16];
		if (cd_be2[1]) pairdata2_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat2[15:8];
		if (cd_be2[0]) pairdata2_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat2[7:0];
	end
    if (ce & cd_we[3] & !cd_widx[DIDXW-1]) begin
		if (cd_be3[3]) pairdata3_b3[cd_widx[DIDXW-2:0]] <= cd_wdat3[31:24];
		if (cd_be3[2]) pairdata3_b2[cd_widx[DIDXW-2:0]] <= cd_wdat3[23:16];
		if (cd_be3[1]) pairdata3_b1[cd_widx[DIDXW-2:0]]  <= cd_wdat3[15:8];
		if (cd_be3[0]) pairdata3_b0[cd_widx[DIDXW-2:0]]   <= cd_wdat3[7:0];
	end
    if (ce & cd_rd_en) begin
        pair_q0 <= {pairdata0_b3[{pair_row[SETW-1:0], 2'd0 - pair_read_word}], pairdata0_b2[{pair_row[SETW-1:0], 2'd0 - pair_read_word}], pairdata0_b1[{pair_row[SETW-1:0], 2'd0 - pair_read_word}], pairdata0_b0[{pair_row[SETW-1:0], 2'd0 - pair_read_word}]};
        pair_q1 <= {pairdata1_b3[{pair_row[SETW-1:0], 2'd1 - pair_read_word}], pairdata1_b2[{pair_row[SETW-1:0], 2'd1 - pair_read_word}], pairdata1_b1[{pair_row[SETW-1:0], 2'd1 - pair_read_word}], pairdata1_b0[{pair_row[SETW-1:0], 2'd1 - pair_read_word}]};
        pair_q2 <= {pairdata2_b3[{pair_row[SETW-1:0], 2'd2 - pair_read_word}], pairdata2_b2[{pair_row[SETW-1:0], 2'd2 - pair_read_word}], pairdata2_b1[{pair_row[SETW-1:0], 2'd2 - pair_read_word}], pairdata2_b0[{pair_row[SETW-1:0], 2'd2 - pair_read_word}]};
        pair_q3 <= {pairdata3_b3[{pair_row[SETW-1:0], 2'd3 - pair_read_word}], pairdata3_b2[{pair_row[SETW-1:0], 2'd3 - pair_read_word}], pairdata3_b1[{pair_row[SETW-1:0], 2'd3 - pair_read_word}], pairdata3_b0[{pair_row[SETW-1:0], 2'd3 - pair_read_word}]};
        pair_idle_valid <= 1'b1;
    end
end

assign c_ack   = (pass_active && !post_active) ? m_ack : (ack_r | fast_hit | fast_ihit | fast_store | fast_span_ack | fast_pair_idle | fast_xline_idle);
// The offer is a level, not a pulse: the core refuses a line while a
// queue fetch is outstanding or a data access acknowledges in the same
// cycle, and a pulse lost to that refusal cost explicit fetches for the
// rest of the line (Sieve, 2026-09-14).  The core's accept is idempotent.
assign c_line_stb  = iline_valid && !iline_pending;
assign c_busy      = fill_active || (cst == C_TAGW) || post_active || cross_lookup || cross_second;
assign c_posting   = post_active;
// A spanning store merges from its line read, which completes one cycle
// after admission; a capture-cycle acknowledge would arrive first and
// force the invalidate fallback (13 % more data fills in the Speedometer
// profile), so only non-spanning stores are reported posted at once.
assign m_posted    = post_active;   // P195: a spanning store no longer reads its line first
assign c_line_tag  = iline_tag;
assign c_line_data = iline_data;
assign c_rdata = (pass_active && !post_active) ? m_rdata : fast_pair_idle ? hint_pair_data : fast_xline_idle ? hint_xline_data : fast_span_ack ? span_extract({sp_w0, sp_w1}, r_size, r_off) : (fast_hit ? fast_data : fast_ihit ? fast_idata : rdata_r);

assign rd_accept = (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                   c_req && !ack_r && !c_write && !bypass &&
                   !ci_inv_pend && !store_inv_lost;

// While a fill or a posted store drains, the requester may already have
// been released and c_addr has moved on to its next request or hint: the
// tag row that C_TAGW rewrites and that the posted store's merge consults
// must be the transaction's own row, not the live address's.
// the line-crossing read's second lookup reads the next row (data and tag
// alike) while the first lookup's hit is being decided
wire xlook_read = (cst == C_LOOK) && r_xline && !xlook && look_hit &&
                  !look_snooped && !snoop_look_row && !inv_wren;
assign tag_ridx  = (fill_active || (cst == C_TAGW) || post_active || cross_lookup || cross_second) ? r_row :
                   // P196: a store accepted while the core hints its next access
                   // looks its tag up at its own row
                   (c_hint_away && c_write) ? a_row :
                   idle_xline_hit ? {1'b0, a_set + {{(SETW-1){1'b0}},1'b1}} :
                   xlook_read ? {1'b0, r_setB} : x_row;
wire [4*TAGW-1:0] tags_next = (r_way == 2'd0) ? {tag_q[4*TAGW-1:TAGW], r_tag} :
                        (r_way == 2'd1) ? {tag_q[4*TAGW-1:2*TAGW], r_tag, tag_q[TAGW-1:0]} :
                        (r_way == 2'd2) ? {tag_q[4*TAGW-1:3*TAGW], r_tag, tag_q[2*TAGW-1:0]} :
                                          {r_tag, tag_q[3*TAGW-1:0]};
wire  [3:0] val_next  = tag_q[4*TAGW+3:4*TAGW] | (4'd1 << r_way);
wire        sweep_hit = sweep_all || (sweep_cnt[ROWIW-1] ? cinv_ic : cinv_dc);
assign tag_we    = ((cst == C_TAGW) && !fill_snooped && !snoop_fill_row) ||
                   ((cst == C_SWEEP) && sweep_hit);
assign tag_widx  = (cst == C_SWEEP) ? sweep_cnt : r_row;
assign tag_wdat  = (cst == C_SWEEP) ? {ROWW{1'b0}}
                                    : {tag_q[ROWW-1:ROWW-2] + 2'd1, val_next, tags_next};

// Port B: a complex or uncacheable store invalidates the data-bank set it
// touches, and the next set when the transfer crosses the line.  A cleared
// row needs no read-modify-write -- the tags left behind are never consulted
// without their valid bit.  Aligned cacheable stores use the hit-update path
// below instead.  The 68040 leaves the instruction cache alone here.
// Port B invalidates: a snoop takes priority over a store's own
// invalidate, because a missed snoop leaves stale data while a delayed
// store invalidate is picked up again from snoop_pend below.
wire store_inv = ((cst == C_IDLE) && c_req && c_write && !ack_r &&
	                  !store_inv_lost && !store_any_update) ||
                 ((cst == C_PASS) && winv_pend) ||
                 (cst == C_WINV);
// Snoop invalidates are FREE-RUNNING (5.1): a chipset write must land
// even while clkena is frozen.  The store-side invalidates stay in the
// ce domain with the FSM that generates them.  The only suppression is
// a sweep zeroing the same row in the same cycle (both write zero; the
// double write is avoided, the effect is identical).
wire snoop_wr  = s_stb && !((cst == C_SWEEP) && sweep_hit &&
                            (sweep_cnt[ROWIW-1:1] == {1'b0, s_addr[SETW+3:5]}));
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
// The sweep clears two rows per cycle: port A the even row (sweep_cnt,
// bit 0 held at zero) and port B the odd one whenever port B is not
// serving a snoop or an owed invalidate; the step repeats until it is,
// so no odd row is skipped.  The walk covers only the selected bank(s):
// rows {0, *} for the data cache, {1, *} for the instruction cache, both
// at reset.  With SETW = 8 a one-bank CINV takes 128 cycles, as the
// 4 KB geometry's one-row-per-cycle walk did.
wire sweep_b    = (cst == C_SWEEP) && sweep_hit && !snoop_wr &&
                  !store_inv_lost && !ci_inv_pend;
wire sweep_last = (&sweep_cnt[SETW-1:1]) &&
                  (sweep_cnt[ROWIW-1] || !(sweep_all || cinv_ic));

assign inv_we   = snoop_wr || store_inv || store_inv_lost || fill_err_inv ||
                  ci_inv || sweep_b;
assign inv_wren = snoop_wr | (ce & (store_inv | store_inv_lost | fill_err_inv |
                                    ci_inv | sweep_b));
assign inv_idx  = snoop_wr        ? {1'b0, s_addr[SETW+3:4]} :
                  sweep_b         ? {sweep_cnt[ROWIW-1:1], 1'b1} :
                  fill_err_inv    ? r_row :   // the fill's own bank and row
                  ci_inv          ? ci_inv_row :
                  store_inv_lost ? {1'b0, store_inv_set} :
                  (cst == C_IDLE) ? {1'b0, c_addr[SETW+3:4]} : {1'b0, winv_set2};

// The four ways' copies of the requested word arrive together; the tag
// compare picks one.  Word w of way v sits in array (v + w) mod 4.
wire  [1:0] q_word = xlook ? 2'd0 :
                     (cst == C_IDLE) ? c_addr[3:2] :
                     (cst == C_PASS) ? r_beat : r_addr[3:2];
wire  [1:0] q_arr  = hit_way + q_word;
wire [31:0] data_hit = (q_arr == 2'd0) ? data_q0 :
                       (q_arr == 2'd1) ? data_q1 :
                       (q_arr == 2'd2) ? data_q2 : data_q3;

// An intervening DATA request cannot displace an instruction-cache line
// (the banks have separate tag rows), and the buffer is a private copy, so
// loads/stores may pass without killing it.  A nonmatching instruction
// request clears it before it can miss and refill, which proves that the
// buffered line cannot have been displaced while the buffer was valid.
// CINV/reset clear it separately below.
// (ipred_hit keeps its historical name: the simulator harness probes it.)
wire ipred_hit = (cst == C_IDLE) && c_req && !ack_r && !c_write && c_instr &&
                 ie && !c_nocache && fits_long && !ci_inv_pend &&
                 !(cinv_req && !cinv_done) && iline_valid &&
                 (c_addr[31:4] == iline_tag);
wire iline_hit = ipred_hit && !fast_ihit;
wire [31:0] iline_lw = (c_addr[3:2] == 2'd0) ? iline_data[127:96] :
                       (c_addr[3:2] == 2'd1) ? iline_data[95:64]  :
                       (c_addr[3:2] == 2'd2) ? iline_data[63:32]  :
                                               iline_data[31:0];

// Read only the page-offset-indexed arrays while idle. This is not a
// speculative physical hit: c_req/rd_accept must still come from the MMU
// AFTER translation, permissions and cacheability have been resolved.
// Remember which synchronous read actually produced the RAM outputs. The
// tag read free-runs even with ce low, whereas data reads are ce-gated.
// Port B prepares the hinted tag row while port A serves a posted store.
// A read is still admitted only after the store has left C_PASS.
reg hint_tag_valid;
reg [ROWIW-1:0] hint_tag_idx;
// P175: the instruction mirrors' read of the previous clock
reg ihint_tag_valid, idle_idata_valid;
reg [ROWIW-1:0] ihint_tag_idx;
reg xhint_tag_valid;
reg [ROWIW-1:0] xhint_tag_idx;
reg [DIDXW-1:0] idle_idata_idx;
reg [SETW+3:0] hqi_lo;
always @(posedge clk) begin
	hqi_lo <= c_ihint_addr[SETW+3:0];
	if (!nreset) begin ihint_tag_valid <= 0; ihint_tag_idx <= 0; idle_idata_valid <= 0; idle_idata_idx <= 0; xhint_tag_valid <= 0; xhint_tag_idx <= 0; end
	else begin
		ihint_tag_idx <= xi_row;
		ihint_tag_valid <= !inv_wren && !tag_we;
		xhint_tag_idx <= x_rowp1;
		xhint_tag_valid <= !inv_wren && !tag_we;
		if (ce) begin
			idle_idata_idx <= {xi_row, xi_w};
			idle_idata_valid <= !fast_ihit && !inv_wren && (!(|cd_we) || (cd_widx[DIDXW-1:2] != xi_row));
		end
		else if (inv_wren || (|cd_we)) idle_idata_valid <= 0;
	end
end
always @(posedge clk) begin
    if (!nreset) begin hint_tag_valid <= 0; hint_tag_idx <= 0; end
    else begin
        hint_tag_idx <= x_row;
        hint_tag_valid <= !inv_wren && !tag_we;
    end
end
// Save the original word before speculative reads replace the RAM outputs.
// Partial stores must merge against this word when their bus ack arrives.
reg posted_word_valid;
reg [31:0] posted_word;
always @(posedge clk) begin
    if (!nreset) begin posted_word_valid <= 0; posted_word <= 0; end
    else if (ce) begin
        if (!post_active) posted_word_valid <= 0;
        else if (!posted_word_valid) begin
            posted_word <= data_hit;
            posted_word_valid <= 1;
        end
    end
end
wire [31:0] store_merge_word = (post_active && posted_word_valid) ? posted_word : data_hit;
// P187: a spanning store's merge words are captured (sp_h0/sp_h1) in the
// clock its line read lands, so from then on the data banks are free for
// the next access's idle read as for any other posted store -- Pascal's
// 2-mod-4 stack longwords made every such store cost the following read
// its one-clock hit.
wire posted_hint_read = (cst == C_PASS) && post_active &&
    !cross_store && !pass_ci_chk && !winv_pend &&
    !ci_inv_pend && !store_inv_lost;
reg idle_data_valid, idle_tag_valid;
// P178 fix: the pair banks' next-row read (x_rowp1, for the cross-line
// hit) needs its own same-clock write check -- idle_data_valid only
// rejects a write to x_row, and a posted store's C_PASS write to the next
// line lands exactly in the clocks posted_hint_read keeps the idle read up.
reg idle_next_valid;
reg [DIDXW-1:0] idle_data_idx;
wire rd_redirect;   // P220: the arrays read another row/word than the hint's
reg [ROWIW-1:0] idle_tag_idx;
always @(posedge clk) begin
	if (!nreset) begin
		idle_data_valid <= 0;
		idle_tag_valid <= 0;
		idle_next_valid <= 0;
		idle_data_idx <= 0;
		idle_tag_idx <= 0;
	end else begin
		idle_tag_idx <= tag_we ? tag_widx : tag_ridx;
		// Reject all mixed-port writes, conservatively even to another row.
		idle_tag_valid <= !tag_we && !inv_wren;
		if (inv_wren || (ce && |cd_we)) begin idle_data_valid <= 0; idle_next_valid <= 0; end
		if (ce && cd_rd_en) begin
			idle_data_idx <= {x_instr, x_set, x_addr[3:2]};
			// P220: and only when the arrays really read the hint's row and
			// word -- a cross-line idle hit (its registered twin fires in the
			// same clock as fast_xline_idle), the crossing lookups and the
			// second-line reads steer rd_row/rd_w elsewhere, and a read issued
			// in place on the next edge used to hit on that other row's data
			idle_data_valid <= ((cst == C_IDLE) || posted_hint_read) && !iline_read && !rd_redirect &&
                               !inv_wren && (!(|cd_we) ||
                               (cd_widx[DIDXW-1:2] != x_row));
			idle_next_valid <= ((cst == C_IDLE) || posted_hint_read) && !iline_read && !rd_redirect &&
                               !inv_wren && (!(|cd_we) ||
                               ((cd_widx[DIDXW-1:2] != x_row) && (cd_widx[DIDXW-1:2] != x_rowp1)));
		end
	end
end
wire idle_hit = rd_accept && !ipred_hit && !err_hold && !m_err && fits_lane &&
                idle_data_valid && idle_tag_valid &&
                (idle_data_idx == {c_instr, c_addr[SETW+3:2]}) &&
                (idle_tag_idx == a_row) && look_hit &&
                !tag_we && !inv_wren && !look_snooped && !snoop_look_row;
// Start a settled within-line pair read at admission.
// P217: not when the one-clock pair hit is predicted from its registered
// terms: the line read it starts replaced the next clock's idle read, so a
// run of 2-mod-4 longwords (MOVEM on the stack, FPU operands, Pascal stack
// temporaries) could not hit two in a row.  A wrong prediction only sends
// the read to the registered lookup.
wire pair_pred;   // assigned after hint_pair_lane
wire idle_span_hit = rd_accept && !ipred_hit && !err_hold && !m_err && span2 && !pair_pred &&
                idle_data_valid && idle_tag_valid &&
                (idle_data_idx == {c_instr, c_addr[SETW+3:2]}) &&
                (idle_tag_idx == a_row) && look_hit &&
                !tag_we && !inv_wren && !look_snooped && !snoop_look_row;
// Start a settled crossing read's second lookup at admission.
wire idle_xline_hit = rd_accept && !ipred_hit && !err_hold && !m_err && xline &&
                idle_data_valid && idle_tag_valid &&
                (idle_data_idx == {c_instr, c_addr[SETW+3:2]}) &&
                (idle_tag_idx == a_row) && look_hit &&
                !tag_we && !inv_wren && !look_snooped && !snoop_look_row;
wire hh0 = hint_tag_q[4*TAGW+0] && (hint_tag_q[0*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire hh1 = hint_tag_q[4*TAGW+1] && (hint_tag_q[1*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire hh2 = hint_tag_q[4*TAGW+2] && (hint_tag_q[2*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire hh3 = hint_tag_q[4*TAGW+3] && (hint_tag_q[3*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire       hint_look_hit = hh0 | hh1 | hh2 | hh3;
// The fast hit's own view of the request: the hint bus repeats the
// registered request address while it is presented, so the offset bits
// come from there and not from the translated address (whose low bits
// pass through the MMU's physical-address mux).  They are taken from
// the cache's own registered copy of the hint (hq_lo), not from the live
// bus: c_hint_match already requires the request to equal the hint the
// MMU registered a cycle earlier, so the two agree whenever fast_hit can
// be true, and the timing analyzer no longer follows the core's
// combinational hint mux (state -> hint select) into the word select,
// the ALU and the branch lookahead (a 33 ns false path that missed the
// 33 MHz clock by 3.8 ns, 2026-09-17).
reg  [SETW+3:0] hq_lo;
always @(posedge clk) hq_lo <= c_hint_addr[SETW+3:0];
wire        fast_lane = (c_size == `AP040_SZ_L && hq_lo[1:0] == 2'b00) ||
                        (c_size == `AP040_SZ_W && hq_lo[1:0] != 2'b11) ||
                        (c_size == `AP040_SZ_B);
// Admission for the one-clock acknowledge without the request's live
// translation: no c_req (the MMU asserts it only after its translation
// passes), no bypass (cacheability is in the hint's vouch), no ipred_hit
// (instruction only); c_hint_match carries "a data request is presented,
// it is the registered hint, it translated, it is cacheable and readable".
wire        fast_accept = (cst == C_IDLE) && !(cinv_req && !cinv_done) && !ack_r &&
                          !c_write && !c_instr && de && !ci_inv_pend && !store_inv_lost;
// P182: fast_accept's and fast_hit's registered terms, for the core's
// acknowledge prediction (no request, hint compare or acknowledge in it)
// P204: nothing is admitted in an ack_r clock (every admission requires
// !ack_r), so the core may hint its next request in it
assign c_ack_q = ack_r;
// P210: or a posted store's C_PASS with fast_accept_pp's registered terms
// (declared below): the read after a store hits there, and the MOVE store
// behind it can be hinted in that clock (the posted store's C_PASS lasts one
// clock, so the cache is back in C_IDLE when the MOVE store arrives)
wire        c_fast_ready_pp;
assign c_fast_ready = ((cst == C_IDLE) || c_fast_ready_pp) && !(cinv_req && !cinv_done) && !ack_r && de &&
                      !ci_inv_pend && !store_inv_lost && idle_data_valid && hint_tag_valid;
// Rotate words using the registered offset before the late tag selection.
// Preserve the original way priority, including the no-hit default.
wire [31:0] hint_word0 = (hq_lo[3:2] == 0) ? data_q0 :
    (hq_lo[3:2] == 1) ? data_q1 :
    (hq_lo[3:2] == 2) ? data_q2 : data_q3;
wire [31:0] hint_word1 = (hq_lo[3:2] == 0) ? data_q1 :
    (hq_lo[3:2] == 1) ? data_q2 :
    (hq_lo[3:2] == 2) ? data_q3 : data_q0;
wire [31:0] hint_word2 = (hq_lo[3:2] == 0) ? data_q2 :
    (hq_lo[3:2] == 1) ? data_q3 :
    (hq_lo[3:2] == 2) ? data_q0 : data_q1;
wire [31:0] hint_word3 = (hq_lo[3:2] == 0) ? data_q3 :
    (hq_lo[3:2] == 1) ? data_q0 :
    (hq_lo[3:2] == 2) ? data_q1 : data_q2;
wire [31:0] hint_data_hit = (hint_word0 & {32{hh0}}) |
    (hint_word1 & {32{!hh0 && hh1}}) |
    (hint_word2 & {32{!hh0 && !hh1 && hh2}}) |
    (hint_word3 & {32{!hh0 && !hh1 && !hh2}});
// idle_hit without look_hit (the live translation's tag compare)
// P196: a data read may also hit in its request cycle while a posted store
// finishes in C_PASS -- its row differing from the store's (whose RAM write
// may land this clock), the store not line-crossing (the second line is
// written after C_PASS), no invalidate owed.
// The store writes only its own bytes (P195), so the read's idle-read data
// stays right unless a word it takes is one the store writes: the store's
// word (and the next, spanning) against the read's word (and the next, for
// the pair and cross-line hits; the cross-line one's second word is word 0
// of the next row).
wire [SETW+1:0] pp_sw0 = {r_row[SETW-1:0], r_addr[3:2]};
wire [SETW+1:0] pp_sw1 = pp_sw0 + {{SETW{1'b0}}, 2'd1};
wire [SETW+1:0] pp_rw0 = hq_lo[SETW+3:2];
wire [SETW+1:0] pp_rw1 = pp_rw0 + {{SETW{1'b0}}, 2'd1};
wire        pp_clash = (pp_rw0 == pp_sw0) || (r_span2 && (pp_rw0 == pp_sw1)) ||
                       (pp_rw1 == pp_sw0) || (r_span2 && (pp_rw1 == pp_sw1));
assign c_fast_ready_pp = (cst == C_PASS) && post_active && !cross_store && !pass_ci_chk &&
                         !winv_pend && !pp_clash;
wire        fast_accept_pp = (cst == C_PASS) && post_active && !cross_store && !pass_ci_chk &&
                             !winv_pend && !ci_inv_pend && !store_inv_lost &&
                             !(cinv_req && !cinv_done) && !ack_r && !c_write && !c_instr && de &&
                             !pp_clash;
assign fast_hit  = (fast_accept || fast_accept_pp) && !err_hold && !m_err && fast_lane &&
                   idle_data_valid && hint_tag_valid &&
                   (idle_data_idx == {1'b0, hq_lo[SETW+3:2]}) &&
                   (hint_tag_idx == {1'b0, hq_lo[SETW+3:4]}) && hint_look_hit &&
                   // !snoop_wr, not !inv_wren: with cst == C_IDLE, !c_write,
                   // !ci_inv_pend and !store_inv_lost already required, every
                   // other term of inv_wren is zero, and inv_wren's store term
                   // carries c_req -- the live translation (ATC RAM -> hit ->
                   // walk decision), which put 8 ns in front of the fast
                   // acknowledge and its data (build 5, -1.04 ns, 2026-09-17).
                   // the snoop-row collision compares against the registered
                   // hint's set as well: snoop_look_row uses a_set from c_addr,
                   // whose low bits reach the cache through the MMU's
                   // physical-address mux, i.e. another false dependency on
                   // the live translation for the analyzer
                   !tag_we && !snoop_wr && !look_snooped &&
                   !(s_stb && (s_addr[SETW+3:4] == hq_lo[SETW+3:4])) &&
                   !c_instr && c_hint_match;
assign fast_data = lw_extract(hint_data_hit, c_size, hq_lo[1:0]);
// P174/P175: the one-clock instruction hit -- fast_hit's terms on the
// instruction side's own idle read (the mirror tag row and the mirror
// data word at the instruction hint, the hint's registered physical tag,
// the MMU's vouch that the request is that hint).  An instruction fetch
// is an aligned longword or a word, so the lane always fits.  The
// buffered-line hit (ipred_hit, a registered acknowledge) yields to it.
wire ihh0 = ihint_tag_q[4*TAGW+0] && (ihint_tag_q[0*TAGW +: TAGW] == c_ihint_ptag[21:22-TAGW]);
wire ihh1 = ihint_tag_q[4*TAGW+1] && (ihint_tag_q[1*TAGW +: TAGW] == c_ihint_ptag[21:22-TAGW]);
wire ihh2 = ihint_tag_q[4*TAGW+2] && (ihint_tag_q[2*TAGW +: TAGW] == c_ihint_ptag[21:22-TAGW]);
wire ihh3 = ihint_tag_q[4*TAGW+3] && (ihint_tag_q[3*TAGW +: TAGW] == c_ihint_ptag[21:22-TAGW]);
wire ihint_look_hit = ihh0 | ihh1 | ihh2 | ihh3;
assign hint_way = ihh0 ? 2'd0 : ihh1 ? 2'd1 : ihh2 ? 2'd2 : 2'd3;
assign hq_irow  = {1'b1, hqi_lo[SETW+3:4]};
wire [31:0] ihint_word0 = (hqi_lo[3:2] == 0) ? idata_q0 : (hqi_lo[3:2] == 1) ? idata_q1 : (hqi_lo[3:2] == 2) ? idata_q2 : idata_q3;
wire [31:0] ihint_word1 = (hqi_lo[3:2] == 0) ? idata_q1 : (hqi_lo[3:2] == 1) ? idata_q2 : (hqi_lo[3:2] == 2) ? idata_q3 : idata_q0;
wire [31:0] ihint_word2 = (hqi_lo[3:2] == 0) ? idata_q2 : (hqi_lo[3:2] == 1) ? idata_q3 : (hqi_lo[3:2] == 2) ? idata_q0 : idata_q1;
wire [31:0] ihint_word3 = (hqi_lo[3:2] == 0) ? idata_q3 : (hqi_lo[3:2] == 1) ? idata_q0 : (hqi_lo[3:2] == 2) ? idata_q1 : idata_q2;
wire [31:0] ihint_data_hit = (ihint_word0 & {32{ihh0}}) |
    (ihint_word1 & {32{!ihh0 && ihh1}}) |
    (ihint_word2 & {32{!ihh0 && !ihh1 && ihh2}}) |
    (ihint_word3 & {32{!ihh0 && !ihh1 && !ihh2}});
wire        ifast_lane = (c_size == `AP040_SZ_L && hqi_lo[1:0] == 2'b00) ||
                         (c_size == `AP040_SZ_W && hqi_lo[1:0] != 2'b11);
// P201: and during a posted store's C_PASS (fast_accept_pp's terms): the
// hit reads only the instruction mirrors, which no store writes, so the
// store's own row needs no clash check.
wire        fast_iaccept = ((cst == C_IDLE) ||
                            ((cst == C_PASS) && post_active && !cross_store && !pass_ci_chk && !winv_pend)) &&
                           !(cinv_req && !cinv_done) && !ack_r &&
                           !c_write && c_instr && ie && !ci_inv_pend && !store_inv_lost;
assign fast_ihit = fast_iaccept && !err_hold && !m_err && ifast_lane &&
                   idle_idata_valid && ihint_tag_valid &&
                   (idle_idata_idx == {1'b1, hqi_lo[SETW+3:2]}) &&
                   (ihint_tag_idx == hq_irow) && ihint_look_hit &&
                   !tag_we && !snoop_wr && !look_snooped &&
                   !(s_stb && (s_addr[SETW+3:4] == hqi_lo[SETW+3:4])) &&
                   !iline_pending && !iline_pair_pending && c_ihint_match;
wire [31:0] fast_idata = lw_extract(ihint_data_hit, c_size, hqi_lo[1:0]);
// The hinted pair hit (P170): fast_hit's terms with the pair lane instead of the aligned one.
wire hint_pair_lane = hq_lo[3:2] != 3 &&
    ((c_size == `AP040_SZ_L && hq_lo[1:0] != 0) ||
     (c_size == `AP040_SZ_W && hq_lo[1:0] == 3));
assign pair_pred = c_fast_ready && hint_pair_lane && pair_idle_valid &&
                   (idle_data_idx == {1'b0, hq_lo[SETW+3:2]}) &&
                   (hint_tag_idx == {1'b0, hq_lo[SETW+3:4]});
wire fast_pair_idle = (fast_accept || fast_accept_pp) && !err_hold && !m_err && hint_pair_lane &&
                   idle_data_valid && pair_idle_valid && hint_tag_valid &&
                   (idle_data_idx == {1'b0, hq_lo[SETW+3:2]}) &&
                   (hint_tag_idx == {1'b0, hq_lo[SETW+3:4]}) && hint_look_hit &&
                   // !snoop_wr, not !inv_wren: with cst == C_IDLE, !c_write,
                   // !ci_inv_pend and !store_inv_lost already required, every
                   // other term of inv_wren is zero, and inv_wren's store term
                   // carries c_req -- the live translation (ATC RAM -> hit ->
                   // walk decision), which put 8 ns in front of the fast
                   // acknowledge and its data (build 5, -1.04 ns, 2026-09-17).
                   // the snoop-row collision compares against the registered
                   // hint's set as well: snoop_look_row uses a_set from c_addr,
                   // whose low bits reach the cache through the MMU's
                   // physical-address mux, i.e. another false dependency on
                   // the live translation for the analyzer
                   !tag_we && !snoop_wr && !look_snooped &&
                   !(s_stb && (s_addr[SETW+3:4] == hq_lo[SETW+3:4])) &&
                   !c_instr && c_hint_match;
wire [31:0] hint_pair_word0 = hq_lo[3:2] == 0 ? pair_q1 : hq_lo[3:2] == 1 ? pair_q2 : hq_lo[3:2] == 2 ? pair_q3 : pair_q0;
wire [31:0] hint_pair_word1 = hq_lo[3:2] == 0 ? pair_q2 : hq_lo[3:2] == 1 ? pair_q3 : hq_lo[3:2] == 2 ? pair_q0 : pair_q1;
wire [31:0] hint_pair_word2 = hq_lo[3:2] == 0 ? pair_q3 : hq_lo[3:2] == 1 ? pair_q0 : hq_lo[3:2] == 2 ? pair_q1 : pair_q2;
wire [31:0] hint_pair_word3 = hq_lo[3:2] == 0 ? pair_q0 : hq_lo[3:2] == 1 ? pair_q1 : hq_lo[3:2] == 2 ? pair_q2 : pair_q3;
wire [31:0] hint_pair_next = (hint_pair_word0 & {32{hh0}}) |
    (hint_pair_word1 & {32{!hh0 && hh1}}) |
    (hint_pair_word2 & {32{!hh0 && !hh1 && hh2}}) |
    (hint_pair_word3 & {32{!hh0 && !hh1 && !hh2}});
wire [31:0] hint_pair_data = span_extract({hint_data_hit, hint_pair_next}, c_size, hq_lo[1:0]);
// P178: the one-clock hit of a longword (or a word at 3 mod 4) that spans
// two lines: this line's word 3 from the hinted idle read, the next line's
// word 0 from the pair banks (read at the next row when the hint sits at
// word 3), the next row's tags from the mirror tag RAM's port B.  The two
// lines share the page unless the set wraps, which is excluded.
wire xhh0 = xhint_tag_q[4*TAGW+0] && (xhint_tag_q[0*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire xhh1 = xhint_tag_q[4*TAGW+1] && (xhint_tag_q[1*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire xhh2 = xhint_tag_q[4*TAGW+2] && (xhint_tag_q[2*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire xhh3 = xhint_tag_q[4*TAGW+3] && (xhint_tag_q[3*TAGW +: TAGW] == c_hint_ptag[21:22-TAGW]);
wire xhint_look_hit = xhh0 | xhh1 | xhh2 | xhh3;
// word 0 of way k sits in array k
wire [31:0] xhint_next = (pair_q0 & {32{xhh0}}) |
    (pair_q1 & {32{!xhh0 && xhh1}}) |
    (pair_q2 & {32{!xhh0 && !xhh1 && xhh2}}) |
    (pair_q3 & {32{!xhh0 && !xhh1 && !xhh2}});
wire [SETW-1:0] hq_setp1 = hq_lo[SETW+3:4] + {{(SETW-1){1'b0}}, 1'b1};
wire hint_xline_lane = hq_lo[3:2] == 2'd3 && !(&hq_lo[SETW+3:4]) &&
    ((c_size == `AP040_SZ_L && hq_lo[1:0] != 0) ||
     (c_size == `AP040_SZ_W && hq_lo[1:0] == 3));
wire fast_xline_idle = (fast_accept || fast_accept_pp) && !err_hold && !m_err && hint_xline_lane &&
                   idle_next_valid && pair_idle_valid && hint_tag_valid && xhint_tag_valid &&
                   (idle_data_idx == {1'b0, hq_lo[SETW+3:2]}) &&
                   (hint_tag_idx == {1'b0, hq_lo[SETW+3:4]}) && hint_look_hit &&
                   (xhint_tag_idx == {1'b0, hq_setp1}) && xhint_look_hit &&
                   !tag_we && !snoop_wr && !look_snooped &&
                   !(s_stb && ((s_addr[SETW+3:4] == hq_lo[SETW+3:4]) || (s_addr[SETW+3:4] == hq_setp1))) &&
                   !c_instr && c_hint_match;
wire [31:0] hint_xline_data = span_extract({hint_data_hit, xhint_next}, c_size, hq_lo[1:0]);

// The one-clock posted store: a data write that the platform posts
// (c_post_ok), whose request is the registered hint and whose page the
// MMU vouches for writing (c_hint_wmatch: translated, cacheable, not
// write-protected, modified bit set), admitted in C_IDLE with nothing
// owed.  The admission below does everything it did -- captures the
// copy it drains from, sets post_active, books the hit-update -- on this
// same edge; only the acknowledge moves from the registered ack_r a
// cycle later to now.  Every term is a register or the MMU's registered
// verdict, as for fast_hit.  (2026-09-19)
wire        fast_store = (cst == C_IDLE) && !(cinv_req && !cinv_done) && !ack_r &&
                         !err_hold && !m_err && c_write && !c_instr &&
                         !store_inv_lost && c_post_ok_hint && c_hint_wmatch;

// Any instruction hit that identified its way (a C_LOOK hit, or an idle
// admission) reads that way's whole line on the same edge it acknowledges.
wire iline_seed_read = (cst == C_LOOK) && look_hit && r_bank && !look2;
wire iline_idle_read = idle_hit && c_instr && !fast_ihit;
wire dline_read = (cst == C_LOOK) && look_hit && !r_bank && r_span2 && !look2;
wire iline_tagw_read = (cst == C_TAGW) && r_bank && !fill_snooped && !snoop_fill_row;
wire sline_read = 1'b0;   // P195: byte-enable writes need no line read
wire iline_read = iline_seed_read || iline_idle_read || dline_read || iline_tagw_read || sline_read || idle_span_hit;
wire [ROWIW-1:0] line_read_row = (iline_idle_read || idle_span_hit) ? {c_instr, c_addr[SETW+3:4]} : r_row;
wire [1:0] line_read_way = iline_tagw_read ? r_way : hit_way;

assign cd_rd_en  = (cst == C_IDLE) || posted_hint_read || rd_accept || store_lookup_accept || iline_read ||
                   xlook_read || cross_lookup;
// word-wise: array k at way (k - w); line-wise: every array at the hit way;
// the crossing read's second lookup: word 0 of the next row
assign rd_redirect = idle_xline_hit || xlook_read || cross_lookup || cross_second;
wire  [1:0] rd_w = (idle_xline_hit || xlook_read || cross_lookup || cross_second) ? 2'd0 : x_w;
wire  [SETW:0] rd_row = (cross_lookup || cross_second) ? r_row :
                        idle_xline_hit ? {1'b0, a_set + {{(SETW-1){1'b0}},1'b1}} :
                   xlook_read ? {1'b0, r_setB} : x_row;
assign cd_ridx0  = iline_read ? {line_read_row, line_read_way}
                              : {rd_row, 2'd0 - rd_w};
assign cd_ridx1  = iline_read ? {line_read_row, line_read_way}
                              : {rd_row, 2'd1 - rd_w};
assign cd_ridx2  = iline_read ? {line_read_row, line_read_way}
                              : {rd_row, 2'd2 - rd_w};
assign cd_ridx3  = iline_read ? {line_read_row, line_read_way}
                              : {rd_row, 2'd3 - rd_w};
// the spanning pair, from the line read of way r_hway: word w in array
// (way + w) mod 4, its successor in the next array
wire  [1:0] sp_a0 = r_hway + r_addr[3:2];
wire  [1:0] sp_a1 = sp_a0 + 2'd1;
wire [31:0] sp_w0 = (sp_a0 == 2'd0) ? data_q0 : (sp_a0 == 2'd1) ? data_q1 :
                    (sp_a0 == 2'd2) ? data_q2 : data_q3;
wire [31:0] sp_w1 = (sp_a1 == 2'd0) ? data_q0 : (sp_a1 == 2'd1) ? data_q1 :
                    (sp_a1 == 2'd2) ? data_q2 : data_q3;
wire store_hit_write = look_hit &&
                       (((cst == C_PASS) && pass_store_chk && m_ack && (!cross_store || !m_err) &&
                         1'b1) ||
                        (cross_second && !cross_second_lost));
wire store_pair_write = store_hit_write && r_span2;
wire fill_beat_write = ((cst == C_FILL) && r_issued && m_ack) || fill_line_write;
wire  [1:0] wr_way = store_hit_write ? hit_way : r_way;
wire  [1:0] wr_arr = wr_way + r_beat;
wire  [1:0] wr_arr1 = wr_arr + 2'd1;
// The registered whole-line read already supplies both words in look2.
// Acknowledge a qualified spanning data hit here instead of registering it.
// No c_req here (P176): the request was accepted into C_LOOK and is
// level-held until its acknowledge, and c_req is the MMU's pass_ok -- the
// live ATC lookup (ATC RAM -> hit -> need_walk), which this term put in
// front of c_rdata and c_ack, i.e. in front of the operand that the
// in-place compare-and-branch dispatch consumes in the same clock (the
// CPU clock's worst path, -1.2 to -1.6 ns across seeds).  C_LOOK only
// ever holds a read (stores post through C_PASS); c_write and c_instr are
// the core's own registers through the arbiter mux and stay.
wire fast_span_ack = (cst == C_LOOK) && look2 && r_span2 && !r_bank &&
    !c_write && !c_instr && !look_snooped && !snoop_look_row &&
    !err_hold && !m_err;
wire [63:0] pair_new = span_merge(sp_held ? {sp_h0, sp_h1} : {sp_w0, sp_w1}, r_wdata, r_size, r_off);
assign cd_we     = store_pair_write ? ((4'd1 << wr_arr) | (4'd1 << wr_arr1)) :
                   (store_hit_write || fill_beat_write) ? (4'd1 << wr_arr) : 4'd0;
// P195: a store hit writes only its own bytes, so no store reads the old
// word first.  st_val/st_be place the operand over the pair {word w, word
// w+1} (w = the word its first byte is in); a store inside one longword
// uses the high half, a spanning store both, a line-crossing store the high
// half in its first line (word 3) and the low half in the next (word 0).
wire  [7:0] st_be  = ((r_size == `AP040_SZ_B) ? 8'b1000_0000 :
                      (r_size == `AP040_SZ_W) ? 8'b1100_0000 : 8'b1111_0000) >> r_off;
wire [63:0] st_val = ((r_size == `AP040_SZ_B) ? {r_wdata[7:0], 56'd0} :
                      (r_size == `AP040_SZ_W) ? {r_wdata[15:0], 48'd0} : {r_wdata, 32'd0}) >> {r_off, 3'd0};
wire        st_lo  = cross_second;          // the second line of a crossing store: its low half
wire [31:0] st_w   = st_lo ? st_val[31:0] : st_val[63:32];
wire  [3:0] st_b   = st_lo ? st_be[3:0]   : st_be[7:4];
// array k: a spanning store's second word goes to wr_arr1, everything
// else (its first word, a single-word store, a fill beat) to wr_arr
assign {cd_be0, cd_wdat0} = store_pair_write ? ((wr_arr1 == 2'd0) ? {st_be[3:0], st_val[31:0]} : {st_be[7:4], st_val[63:32]}) :
                            store_hit_write ? {st_b, st_w} : {4'hF, cd_wdat};
assign {cd_be1, cd_wdat1} = store_pair_write ? ((wr_arr1 == 2'd1) ? {st_be[3:0], st_val[31:0]} : {st_be[7:4], st_val[63:32]}) :
                            store_hit_write ? {st_b, st_w} : {4'hF, cd_wdat};
assign {cd_be2, cd_wdat2} = store_pair_write ? ((wr_arr1 == 2'd2) ? {st_be[3:0], st_val[31:0]} : {st_be[7:4], st_val[63:32]}) :
                            store_hit_write ? {st_b, st_w} : {4'hF, cd_wdat};
assign {cd_be3, cd_wdat3} = store_pair_write ? ((wr_arr1 == 2'd3) ? {st_be[3:0], st_val[31:0]} : {st_be[7:4], st_val[63:32]}) :
                            store_hit_write ? {st_b, st_w} : {4'hF, cd_wdat};
assign cd_widx   = {r_bank, r_row[SETW-1:0], wr_way};
wire [63:0] cross_first_merge = span_merge({data_hit,32'd0}, r_wdata, r_size, r_off);
wire [63:0] cross_last_merge = span_merge({32'd0,data_hit}, r_wdata, r_size, r_off);
// fills only: a store hit's bytes come from st_val/st_be (P195)
assign cd_wdat   = fill_line_write ? fill_line_word : m_rdata;


`ifdef AP040_EXPERIMENTAL_XSTORE
// Port-B invalidations are free-running even when CE is low. Remember a
// second-row invalidation through the lookup so undefined read-during-write
// tag data cannot select a live cache way for the delayed merge.
wire [SETW-1:0] cross_row_b = (cst == C_IDLE) ? a_set + 1'b1 : r_setB;
always @(posedge clk) begin
    if (!nreset) cross_second_lost <= 0;
    else begin
        if (ce && cst == C_IDLE && cross_accept) cross_second_lost <= 0;
        if ((cross_store || (cst == C_IDLE && cross_accept)) &&
            inv_wren && inv_idx == {1'b0,cross_row_b}) cross_second_lost <= 1;
    end
end
`endif

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
		fill_hold <= 0; r_wdata <= 0; pass_store_chk <= 0;
		fill_hold2 <= 0; r_span2 <= 0; look2 <= 0; r_hway <= 0;
		r_xline <= 0; xlook <= 0; xsnooped <= 0; r_setB <= 0; r_tagB <= 0;
		fill_cnt <= 0; fill_acked <= 0; r_fc <= 0; sline_ready <= 0; sp_held <= 0;
		post_active <= 0; p_addr <= 0; p_wdata <= 0; p_size <= 0; p_fc <= 0;
		iline_pending <= 0; iline_valid <= 0; iline_way <= 0;
		iline_tag <= 0; iline_data <= 0; iline_stb <= 0; iline_stb_pend <= 0;
		iline_pair_pending <= 0;
		ack_r <= 0; rdata_r <= 0;
`ifdef AP040_EXPERIMENTAL_XSTORE
        cross_store <= 0; cross_fault <= 0;
`endif
	end
	else if (ce) begin
		ack_r <= 0;
		cinv_done <= 0;
		if (ci_inv) ci_inv_pend <= 0;
		// The offer must follow the acknowledge by one cycle: the core takes
		// its acknowledged words that cycle and cannot append more then.
		iline_stb <= iline_stb_pend;
		iline_stb_pend <= 0;

		// The line read completed on the preceding edge: array (way + w)
		// holds word w.  Capture before any new request reuses the port.
		if (iline_pending) begin
			case (iline_way)
				2'd0: iline_data <= {data_q0, data_q1, data_q2, data_q3};
				2'd1: iline_data <= {data_q1, data_q2, data_q3, data_q0};
				2'd2: iline_data <= {data_q2, data_q3, data_q0, data_q1};
				default: iline_data <= {data_q3, data_q0, data_q1, data_q2};
			endcase
			iline_pending <= 0;
			iline_valid <= 1;
			iline_stb <= 1;
		end
		// P174: the line read on the mirror banks completed on the preceding edge
		if (iline_pair_pending) begin
			case (iline_way)
				2'd0: iline_data <= {idata_q0, idata_q1, idata_q2, idata_q3};
				2'd1: iline_data <= {idata_q1, idata_q2, idata_q3, idata_q0};
				2'd2: iline_data <= {idata_q2, idata_q3, idata_q0, idata_q1};
				default: iline_data <= {idata_q3, idata_q0, idata_q1, idata_q2};
			endcase
			iline_pair_pending <= 0;
			iline_valid <= 1;
			iline_stb <= 1;
		end

		// A snoop displaced a store's first-set invalidate in its
		// acceptance cycle: remember it and issue it as soon as port B
		// is free.  The second-set (winv) invalidate needs no recording:
		// winv_pend persists until port B actually serves it.  A NEW
		// store is stalled one cycle while a recorded invalidate waits
		// (store_inv's !store_inv_lost term), so the single slot cannot
		// be overwritten.
		if (snoop_wr && (cst == C_IDLE) && c_req && c_write && !ack_r &&
		    !store_inv_lost && !store_any_update) begin
			store_inv_lost <= 1;
			store_inv_set  <= c_addr[SETW+3:4];
		end
		else if (store_inv_lost && !s_stb)
			store_inv_lost <= 0;

		case (cst)
			C_IDLE: begin
				if (!c_req) err_hold <= 0;
				if (cinv_req && !cinv_done) begin
					iline_pending <= 0;
					iline_valid <= 0;
					// start at the selected bank (the data bank first
					// when both are selected)
					sweep_cnt <= (cinv_ic && !cinv_dc) ? {1'b1, {SETW{1'b0}}}
					                                   : {ROWIW{1'b0}};
					sweep_all <= 0;   // honour the cinv_ic/cinv_dc selects
					cst <= C_SWEEP;
				end
				else if (fast_ihit) begin
					// P174: acknowledged in the request cycle from the
					// hinted idle read; the way's line is read on the mirror
					// banks on this edge and offered next cycle.
					iline_pair_pending <= 1;
					iline_valid <= 0;
					iline_way <= hint_way;
					iline_tag <= c_addr[31:4];
				end
				else if (iline_hit) begin
					// Registered completion just like C_LOOK, from the
					// buffered line; the buffer stays valid for the rest of
					// the line (a loop inside one line keeps hitting here).
					rdata_r <= lw_extract(iline_lw, c_size, c_addr[1:0]);
					ack_r <= 1;
					iline_stb_pend <= 1;
				end
				else if (fast_pair_idle) begin
					// completed in the request cycle from the pair banks (P170)
					rdata_r <= hint_pair_data;
				end
				else if (fast_xline_idle) begin
					// completed in the request cycle across two lines (P178)
					rdata_r <= hint_xline_data;
				end
				else if (idle_hit || fast_hit) begin
					// Identical registered response to C_LOOK, using the prior
					// matching read rather than issuing a redundant RAM read.
					// A data hit the hint vouched for was acknowledged
					// combinationally (fast_hit).
					if (!fast_hit && !fast_pair_idle) begin
						rdata_r <= lw_extract(data_hit, c_size, c_addr[1:0]);
						ack_r <= 1;
					end
					if (c_instr) begin
						// the line read runs in parallel (iline_idle_read)
						iline_pending <= 1;
						iline_valid <= 0;
						iline_way <= hit_way;
						iline_tag <= c_addr[31:4];
					end
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
				// exemption from the guarantee anyway: an aligned store
				// lookup can only update a still-valid matching way, while
				// every other store clears its own row on acceptance.
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
					// A nonmatching instruction access makes the one-shot
					// prediction unreachable.  Data traffic uses the independent
					// D-cache bank and may safely pass between sequential fills.
					if (c_instr) begin
						iline_pending <= 0;
						iline_valid <= 0;
					end
					if (c_write) begin
						if (store_inv_lost) begin
							// port B owes a recorded invalidate: hold the
							// store one cycle so its own invalidate cannot
							// be skipped (the request is level-held)
						end
						else begin
							// A simple cacheable store reads its tag and all four
							// data ways here, then merges into a matching word only
							// when memory acknowledges it in C_PASS.  A fault can
							// therefore never leave uncommitted data in the cache.
							pass_store_chk <= store_any_update;
`ifdef AP040_EXPERIMENTAL_XSTORE
                            cross_store <= cross_accept;
                            r_setB <= a_set + 1'b1;
                            r_tagB <= (&a_set) ? a_tag + 1'b1 : a_tag;
`endif
							r_span2 <= store_update2;
							sline_ready <= 0;
							sp_held <= 0;
							// A posted store is acknowledged now and drained
							// from its captured copy while the core moves on.
							if (c_post_ok || fast_store) begin
								// acknowledged now by fast_store, or next
								// cycle by ack_r
								if (!fast_store) ack_r <= 1;
								post_active <= 1;
								p_addr <= c_addr; p_wdata <= c_wdata;
								p_size <= c_size; p_fc <= c_fc;
							end
							if (store_any_update) begin
								r_row   <= {1'b0, a_set};
								r_tag   <= a_tag;
								r_bank  <= 0;
								r_beat  <= c_addr[3:2];
								r_addr  <= c_addr;
								r_size  <= c_size;
								r_off   <= c_addr[1:0];
								r_wdata <= c_wdata;
								winv_pend <= 0;
							end
							else begin
								// Complex or uncacheable write: Port B clears the
								// touched set now and the next set, if crossed,
								// during the pass wait or in C_WINV.
								winv_set2 <= c_addr[SETW+3:4] + {{(SETW-1){1'b0}}, 1'b1};
								winv_pend <= write_cross_line;
							end
							cst <= C_PASS;
						end
					end
					else if (c_instr && !c_ihold) begin
						// P184: a hinted fetch's first clock -- the banks read
						// the data hint's row; hold it (fast_ihit would have
						// taken it), the banks follow it from the next clock.
					end
					else if (c_hint_away && !c_instr) begin
						// P182: the core is hinting its next request, not
						// this read, so the RAMs are reading another row:
						// hold the read one clock (it is level-held, and the
						// hint repeats it from the next clock).
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
						r_span2 <= span2;
						r_fc <= c_fc;
						look2 <= idle_span_hit;
                        if (idle_span_hit) r_hway <= hit_way;
						r_xline <= xline;
                        xlook <= idle_xline_hit;
                        if (idle_xline_hit) begin
                            fill_hold2 <= data_hit;
                            xsnooped <= 0;
                        end
						r_setB <= a_set + {{(SETW-1){1'b0}}, 1'b1};
						r_tagB <= (&a_set) ? a_tag + {{(TAGW-1){1'b0}}, 1'b1} : a_tag;
						cst <= C_LOOK;
					end
				end
			end

			C_PASS: begin
				if (fast_ihit) begin
					// P201: the one-clock instruction hit under a posted store
					// (as in C_IDLE: the way's line is offered next cycle)
					iline_pair_pending <= 1;
					iline_valid <= 0;
					iline_way <= hint_way;
					iline_tag <= c_addr[31:4];
				end
				// the second-set invalidate clears only when port B truly
				// served it; a snoop or a recorded first-set replay owns
				// the port this cycle and winv stays pending
				if (!s_stb && !store_inv_lost) winv_pend <= 0;
				// a spanning store's line read (sline_read) completes next
				// cycle; an acknowledge that arrives first cannot merge and
				// the matching line is invalidated instead
				if (sline_read) begin
					r_hway <= hit_way;
					sline_ready <= 1;
				end
				// P187: the line read has landed in data_q this clock; hold the
				// merge words before an idle read replaces it
				if (sline_ready && !sp_held) begin
					sp_h0   <= sp_w0;
					sp_h1   <= sp_w1;
					sp_held <= 1;
				end
				if (pass_ci_chk) begin
					pass_ci_chk <= 0;
					if (look_hit) begin
						ci_inv_pend <= 1;
						ci_inv_row  <= r_row;
					end
				end
				if (m_err) begin
					pass_store_chk <= 0;
					// a passed access faulted: release the bus, but a
					// still-owed invalidate is honoured (invalidating
					// more is always safe under write-through).  A posted
					// store was promised not to fault; nothing to hold.
					err_hold <= !post_active;
					post_active <= 0;
					cst <= (winv_pend && (s_stb || store_inv_lost))
					       ? C_WINV : C_IDLE;
`ifdef AP040_EXPERIMENTAL_XSTORE
                    if (cross_store) begin
                        cross_store <= 0; cross_fault <= 1;
                        winv_pend <= 1; winv_set2 <= r_setB;
                        cst <= C_FERR;
                    end
`endif
				end
				else if (m_ack) begin
					pass_store_chk <= 0;
					post_active <= 0;
					cst <= (winv_pend && (s_stb || store_inv_lost))
					       ? C_WINV : C_IDLE;
`ifdef AP040_EXPERIMENTAL_XSTORE
                    if (cross_store) begin
                        r_row <= {1'b0,r_setB}; r_tag <= r_tagB;
                        r_addr <= {r_addr[31:4]+28'd1,4'd0}; r_beat <= 0;
                        cst <= C_XSTORE_LOOK;
                    end
`endif
                end
			end

`ifdef AP040_EXPERIMENTAL_XSTORE
            C_XSTORE_LOOK: cst <= C_XSTORE_WRITE;
            C_XSTORE_WRITE: begin
                cross_store <= 0;
                cst <= C_IDLE;
            end
`endif
			C_FERR: begin
				// The core withdraws the faulting request while this
				// state runs, and only C_IDLE used to watch for that.
				// Release the hold here too, or a request raised again
				// before C_IDLE is reached is blocked forever.
				if (!c_req) err_hold <= 0;
				// a free-running snoop owns port B when it fires; retry
				// until this row's invalidate is the one that lands
				if (!snoop_wr) begin
                    cst <= cross_fault ? C_WINV : C_IDLE;
`ifdef AP040_EXPERIMENTAL_XSTORE
                    cross_fault <= 0;
`endif
                end
			end

			C_WINV: begin
				if (!s_stb && !store_inv_lost) begin
					winv_pend <= 0;
					cst <= C_IDLE;
				end
			end

			C_SWEEP: begin
				// two rows per cycle: port A the even row (tag_we /
				// sweep_hit), port B the odd one (sweep_b); the step
				// holds while port B is busy elsewhere
				if (sweep_b) begin
					sweep_cnt <= sweep_cnt + {{(ROWIW-2){1'b0}}, 2'b10};
					if (sweep_last) begin
						if (!sweep_all) cinv_done <= 1;
						sweep_all <= 0;
						cst <= C_IDLE;
					end
				end
			end

			C_LOOK: begin
				if (look2) begin
					// the line read of the hit way completed: assemble the
					// spanning pair (a snoop meanwhile forces the refill)
					look2 <= 0;
					if (!look_snooped && !snoop_look_row) begin
						rdata_r <= span_extract({sp_w0, sp_w1}, r_size, r_off);
						if (!fast_span_ack) ack_r <= 1;
						cst <= C_IDLE;
					end
					else begin
						r_way <= !v_w0 ? 2'd0 : !v_w1 ? 2'd1 : !v_w2 ? 2'd2 :
						         !v_w3 ? 2'd3 : tag_q[ROWW-1:ROWW-2];
						r_beat <= r_addr[3:2];
						fill_cnt <= 0;
						fill_acked <= 0;
						r_issued <= 0;
						cst <= C_FILL;
					end
				end
				else if (xlook) begin
					// the next line's tags and word 0 are in: assemble the
					// crossing pair on a hit; on a clean miss fill the next
					// line (word 0 first, the pair acknowledged on that
					// beat: the line is the one a sequential walk needs
					// next); a snoop that touched the row around the read
					// falls back to the bypass
					xlook <= 0;
					if (look_hit && !xsnooped && !xline_snoop_pending && !snoop_xrow) begin
						rdata_r <= span_extract({fill_hold2, data_hit}, r_size, r_off);
						ack_r <= 1;
						cst <= C_IDLE;
					end
					else if (!xsnooped && !xline_snoop_pending && !snoop_xrow) begin
						r_row  <= {1'b0, r_setB};
						r_tag  <= r_tagB;
						r_addr <= {r_addr[31:4] + 28'd1, 4'd0};
						r_way <= !v_w0 ? 2'd0 : !v_w1 ? 2'd1 : !v_w2 ? 2'd2 :
						         !v_w3 ? 2'd3 : tag_q[ROWW-1:ROWW-2];
						r_beat <= 2'd0;
						fill_cnt <= 0;
						fill_acked <= 0;
						r_issued <= 0;
						cst <= C_FILL;
					end
					else begin
						pass_ci_chk <= 0;
						cst <= C_PASS;
					end
				end
				else if (r_xline) begin
					// first line: keep its word 3 and look the next line
					// up (xlook_read runs the reads); a miss bypasses
					if (xlook_read) begin
						fill_hold2 <= data_hit;
						xsnooped <= snoop_xrow;
						xlook <= 1;
					end
					else begin
						pass_ci_chk <= 0;
						cst <= C_PASS;
					end
				end
				else if (look_hit && !look_snooped && !snoop_look_row && r_span2) begin
					// the line read runs in parallel (dline_read)
					r_hway <= hit_way;
					look2 <= 1;
				end
				else if (look_hit && !look_snooped && !snoop_look_row) begin
					// all four ways were read alongside the tags, so the
					// hit completes here: two cycles request-to-ack
					rdata_r <= lw_extract(data_hit, r_size, r_off);
					ack_r <= 1;
					if (r_bank) begin
						// the line read runs in parallel (iline_seed_read)
						iline_pending <= 1;
						iline_valid <= 0;
						iline_way <= hit_way;
						iline_tag <= r_addr[31:4];
					end
					cst <= C_IDLE;
				end
				else begin
					// MC68040UM 4.1: use the first invalid line of the set,
					// and only otherwise the round-robin victim
					r_way <= !v_w0 ? 2'd0 : !v_w1 ? 2'd1 : !v_w2 ? 2'd2 :
					         !v_w3 ? 2'd3 : tag_q[ROWW-1:ROWW-2];
					// MC68040UM 4.6.1: the requested long word is fetched
					// first and handed to the requester as soon as it
					// arrives; the remaining beats follow in wrap order
					r_beat <= r_addr[3:2];
					fill_cnt <= 0;
					fill_acked <= 0;
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
					// once the requester has its data the request is gone;
					// holding would block the next one forever
					err_hold <= !fill_acked;
					cst <= C_FERR;
				end
				else if (fill_line_match || (r_issued && m_ack)) begin : fill_beat
					// The data RAM write runs in parallel (cd_we), just as it
					// does for a returned bus beat.  The requester is
					// acknowledged as soon as its word (or word pair) is in;
					// C_TAGW then validates the line for everyone else.
					reg [31:0] w;
					w = fill_line_match ? fill_line_word : m_rdata;
					if (fill_cnt == 2'd0) fill_hold <= w;
					if (fill_cnt == 2'd1) fill_hold2 <= w;
					if (!fill_acked &&
					    ((fill_cnt == 2'd0 && !r_span2) || (fill_cnt == 2'd1 && r_span2))) begin
						rdata_r <= r_span2 ? span_extract({fill_hold, w}, r_size, r_off) :
						           r_xline ? span_extract({fill_hold2, w}, r_size, r_off)
						                   : lw_extract(w, r_size, r_off);
						ack_r <= 1;
						fill_acked <= 1;
					end
					r_issued <= 0;
					if (fill_cnt == 2'd3) cst <= C_TAGW;
					else begin
						r_beat <= r_beat + 2'd1;
						fill_cnt <= fill_cnt + 2'd1;
					end
				end
				else if (!r_issued) r_issued <= 1;
			end

			C_TAGW: begin
				// the tag row write runs in parallel (tag_we): new tag,
				// its valid bit, and the advanced round robin
				if (!fill_acked) begin
					rdata_r <= r_span2 ? span_extract({fill_hold, fill_hold2}, r_size, r_off) :
					           r_xline ? span_extract({fill_hold2, fill_hold}, r_size, r_off)
					                   : lw_extract(fill_hold, r_size, r_off);
					ack_r <= 1;
				end
				// a filled instruction line seeds the line buffer (the line
				// read runs in parallel, iline_tagw_read), so the following
				// sequential fetches hit it without another lookup
				if (r_bank) begin
					iline_pending <= 1;
					iline_valid <= 0;
					iline_way <= r_way;
					iline_tag <= r_addr[31:4];
				end
				cst <= C_IDLE;
			end

			default: cst <= C_IDLE;
		endcase
	end
end

endmodule
