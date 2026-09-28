//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_mmu.v - MC68040 memory management unit (milestone E)               //
//                                                                          //
// Sits between the core memory port and the 16-bit bus adapter:            //
//  - ITT0/1 and DTT0/1 transparent translation matching                    //
//  - split instruction/data ATCs, 16 sets x 4 ways each                    //
//  - three-level table walk with indirect page descriptors, U/M history    //
//    bit updates and accumulated write protection                          //
//  - 4K (TC.P=0) and 8K (TC.P=1) page sizes                                //
//  - protection/translation faults reported with a one-cycle c_flt pulse;  //
//    the faulted request is consumed and the core builds the format $7     //
//    access error frame                                                    //
//  - PTEST performs a real table search and returns MMUSR contents;        //
//    PFLUSH implements the page/global variants over both ATCs             //
//                                                                          //
// The whole unit advances only when ce (clkena) is high. Table searches    //
// are plain read/write cycles (not bus locked): this fabric has a single   //
// CPU master. A failed search installs a valid but NONRESIDENT entry, as   //
// the 68040 does (MC68040UM 3-14): later accesses fault from the ATC       //
// without walking again, and a descriptor fixed by a handler takes effect  //
// only after a PFLUSH/PTEST of that entry or its replacement.              //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_mmu
(
	input             clk,
	input             nreset,
	input             ce,

	// control registers (from the core MOVEC set)
	input      [31:0] tc,          // bit15 E, bit14 P
	input      [31:0] urp,
	input      [31:0] srp,
	input      [31:0] itt0,
	input      [31:0] itt1,
	input      [31:0] dtt0,
	input      [31:0] dtt1,

	// core side
	input             c_req,
	input             c_write,
	input             c_instr,
	input       [1:0] c_size,
	input      [31:0] c_addr,
	input      [31:0] c_hint_addr,  // next access, one cycle early (logical)
	input      [31:0] c_ihint_addr, // the instruction side's hint (P175), always instruction space
	input             c_hint_instr,
	input      [31:0] c_wdata,
	input       [2:0] c_fc,
	output            c_ack,
	output     [31:0] c_rdata,
	output            c_line_stb,   // cache instruction line sideband, passed up
	output     [31:4] c_line_tag,
	output    [127:0] c_line_data,
	output reg        c_flt,       // one ce cycle; request is consumed

	// PTEST/PFLUSH sideband
	input             pt_req,
	input             pt_write,
	input      [31:0] pt_addr,
	input       [2:0] pt_fc,
	output reg        pt_done,
	output reg [31:0] pt_mmusr,

	input             pf_req,
	input       [1:0] pf_mode,     // 00 (An) nonglobal, 01 (An), 10 all nonglobal, 11 all
	input      [31:0] pf_addr,
	input       [2:0] pf_fc,
	output reg        pf_done,

	// bus adapter side
	output            m_req,
	output            m_write,
	output            m_instr,
	output      [1:0] m_size,
	output     [31:0] m_addr,
	output     [31:0] m_hint_addr,  // hint bus, passed down (page offset only matters)
	output            m_hint_instr,
	output     [21:0] m_hint_ptag,  // the hint's physical tag, registered
	output            m_hint_match, // the request is the registered hint
	output            m_hint_wmatch,// ... and its page may be written now (see hq_wok)
	output     [31:0] m_ihint_addr, // the instruction hint, passed down (P175)
	output     [21:0] m_ihint_ptag, // its physical tag, registered
	output            m_ihint_match,// the request is the registered instruction hint
	output     [31:0] m_wdata,
	output      [2:0] m_fc,
	input             m_ack,
	input      [31:0] m_rdata,
	input             m_line_stb,
	input      [31:4] m_line_tag,
	input     [127:0] m_line_data,

	// Dedicated physical longword port used only for table searches.  Keeping
	// descriptor traffic off m_* avoids serialising every descriptor through
	// the 16-bit CPU bus and prevents it from polluting either CPU cache.
	output            walker_req,
	output            walker_we,
	output     [31:0] walker_addr,
	output     [31:0] walker_wdat,
	input             walker_ack,
	input      [31:0] walker_data,
	input             walker_berr,

	output     [31:0] phys_addr,
	output            cache_inhibit,
	output            m_nocache
);

wire tc_e = tc[15];
wire tc_p = tc[14];

//---------------------------------------------------------------------------
// helper functions
//---------------------------------------------------------------------------

function ttr_match;
	input [31:0] ttr;
	input [31:0] la;
	input        sup;
	begin
		ttr_match = ttr[15] &&
		            (&((la[31:24] ~^ ttr[31:24]) | ttr[23:16])) &&
		            (ttr[14] || (ttr[13] == sup));
	end
endfunction

function [31:0] pgtbl_addr;
	input [31:0] desc;
	begin
		pgtbl_addr = tc_p ? {desc[31:7], 7'd0} : {desc[31:8], 8'd0};
	end
endfunction

//---------------------------------------------------------------------------
// address translation cache: index {bank, set, way}, bank 0=data 1=instr
//---------------------------------------------------------------------------

// The entry payload (tag, PA, attributes) lives in ONE bram.vhd dpram:
// 32 rows of {bank, set}, 4 ways of 46 bits per row.  A flop array here
// costs ~5.7K registers plus the 4-way mux fabric (the single largest
// ALM sink in the design); the M10K row costs a one-clock lookup pipe
// on ENABLED translation only -- TC.E=0 and TTR hits stay combinational
// and pay nothing, which is the common Amiga configuration.  Validity
// and the round-robin pointers stay in flops so PFLUSHA, warm-reset
// preservation and the lookup guard remain single-cycle.
localparam EW   = 46;            // {resident, tag[16:0], pa[19:0], attr[7:0]}
localparam ROWW = 4*EW;

reg         atc_v    [0:127];
reg   [1:0] atc_rr   [0:31];    // round robin per {bank, set}
// ATC entries survive RSTI.  Clear them only at FPGA configuration/cold
// start; the initialized flag makes later nreset assertions architectural
// warm resets.
reg         atc_reset_seen = 1'b0;

wire        a_super = c_fc[2];
wire  [3:0] a_set   = tc_p ? c_addr[16:13] : c_addr[15:12];
wire [16:0] a_tag   = tc_p ? {a_super, c_addr[31:17], 1'b0}
                           : {a_super, c_addr[31:16]};
wire  [4:0] a_row   = {c_instr, a_set};

// port A: lookup reads, and the PFLUSH/PTEST sweep while the core is
// stalled behind pf_req/pt_req.  port B: the walker's fill row -- its
// address holds {f_bank, f_set} for the whole walk, so q_b carries the
// row to compose the read-modify-write fill from.
reg         sweep_on;
reg   [5:0] sweep_cnt;
wire        fill_we;             // assigned below the walker state decls
wire  [4:0] sweep_row = sweep_cnt[4:0];
wire [ROWW-1:0] row_q, frow_q;

// free-running one-clock lookup pipe: the request is level-held, so the
// piped row is judged fresh when it still describes the live request.
// Sweeps and fill writes poison the pipe for one clock.
reg   [4:0] l_row;
reg  [16:0] l_tag;
reg         l_ld;
always @(posedge clk) begin
	l_row <= a_row;
	l_tag <= a_tag;
	l_ld  <= c_req && !sweep_on && !fill_we;
end
wire lk_fresh = l_ld && (l_row == a_row) && (l_tag == a_tag);

wire [EW-1:0] a_w0 = row_q[0*EW +: EW];
wire [EW-1:0] a_w1 = row_q[1*EW +: EW];
wire [EW-1:0] a_w2 = row_q[2*EW +: EW];
wire [EW-1:0] a_w3 = row_q[3*EW +: EW];

wire hit0 = lk_fresh && atc_v[{l_row, 2'd0}] && (a_w0[44:28] == l_tag);
wire hit1 = lk_fresh && atc_v[{l_row, 2'd1}] && (a_w1[44:28] == l_tag);
wire hit2 = lk_fresh && atc_v[{l_row, 2'd2}] && (a_w2[44:28] == l_tag);
wire hit3 = lk_fresh && atc_v[{l_row, 2'd3}] && (a_w3[44:28] == l_tag);
wire pipe_hit = hit0 | hit1 | hit2 | hit3;
wire [EW-1:0] pipe_ent = hit0 ? a_w0 : hit1 ? a_w1 : hit2 ? a_w2 : a_w3;

// Per-space copies of the most recent piped ATC hit.  A request whose
// {row, tag} equals it translates in its own request cycle instead of
// waiting for the one-clock lookup pipe; sequential fetches and most data
// traffic stay in one page for long stretches.  The copy is dropped on
// every ATC write (walker fill, PFLUSH clear or sweep), on any TC change
// and on reset, so it can never disagree with the array it mirrors.  It
// carries no state of its own: permissions, cache mode and the M bit come
// from the copied entry and go through the same fault/walk decisions.
// P200: four instruction copies too (a call from user code into the ROM's
// FPU glue and its return kept replacing the single instruction copy, so
// the return target's hint never translated)
reg          u_valid [0:7];   // four data copies and four instruction copies, indexed by page set
reg    [4:0] u_row   [0:7];
reg   [16:0] u_tag   [0:7];
reg [EW-1:0] u_ent   [0:7];
reg   [31:0] u_tc;
integer      ui;
always @(posedge clk) begin
	u_tc <= tc;
	if (!nreset || fill_we || sweep_on || pf_req || (tc != u_tc)) begin
		for (ui = 0; ui < 8; ui = ui + 1) u_valid[ui] <= 0;
	end
	else if (pipe_hit) begin
		u_valid[{l_row[4],l_row[1:0]}] <= 1;
		u_row[{l_row[4],l_row[1:0]}]   <= l_row;
		u_tag[{l_row[4],l_row[1:0]}]   <= l_tag;
		u_ent[{l_row[4],l_row[1:0]}]   <= pipe_ent;
	end
end
wire u_hit = u_valid[{c_instr,a_row[1:0]}] && (u_row[{c_instr,a_row[1:0]}] == a_row) &&
             (u_tag[{c_instr,a_row[1:0]}] == a_tag);
wire [EW-1:0] u_sel = u_ent[{c_instr,a_row[1:0]}];

wire atc_hit = u_hit | pipe_hit;

wire [EW-1:0] h_ent = u_hit ? u_sel : pipe_ent;
wire        h_r    = h_ent[45];
wire [19:0] h_pa   = h_ent[27:8];
wire  [7:0] h_attr = h_ent[7:0];
wire        h_s    = h_attr[4];
wire  [1:0] h_cm   = h_attr[3:2];
wire        h_m    = h_attr[1];
wire        h_w    = h_attr[0];

//---------------------------------------------------------------------------
// transparent translation
//---------------------------------------------------------------------------

wire [31:0] ttra = c_instr ? itt0 : dtt0;
wire [31:0] ttrb = c_instr ? itt1 : dtt1;
wire ttr_hit_a = ttr_match(ttra, c_addr, a_super);
wire ttr_hit_b = ttr_match(ttrb, c_addr, a_super);
wire ttr_hit   = ttr_hit_a | ttr_hit_b;
wire ttr_w     = ttr_hit_a ? ttra[2]   : ttrb[2];
wire [1:0] ttr_cm = ttr_hit_a ? ttra[6:5] : ttrb[6:5];

// PTEST uses DFC to select supervisor/user and instruction/data space.
wire        pt_instr = (pt_fc[1:0] == 2'b10);
wire [31:0] pt_ttra  = pt_instr ? itt0 : dtt0;
wire [31:0] pt_ttrb  = pt_instr ? itt1 : dtt1;
wire        pt_ttr_a = ttr_match(pt_ttra, pt_addr, pt_fc[2]);
wire        pt_ttr_b = ttr_match(pt_ttrb, pt_addr, pt_fc[2]);
wire        pt_ttr_hit = pt_ttr_a | pt_ttr_b;
wire        pt_ttr_w = pt_ttr_a ? pt_ttra[2] : pt_ttrb[2];

//---------------------------------------------------------------------------
// translation decision
//---------------------------------------------------------------------------

wire ttr_fault = ttr_hit && c_write && ttr_w;
wire atc_fault = tc_e && !ttr_hit && atc_hit &&
                 (!h_r || (c_write && h_w) || (!a_super && h_s));
// write to a clean page runs a table search to set the M bit
wire atc_mmiss = atc_hit && h_r && c_write && !h_m && !h_w;

// lk_fresh gates atc_hit, so a walk is only started once the piped row
// has been judged against the live request
wire need_walk = tc_e && !ttr_hit && (lk_fresh || u_hit) &&
                 (!atc_hit || atc_mmiss) && !atc_fault;

wire [31:0] pa_out =
	ttr_hit ? c_addr :
	(tc_e && atc_hit) ? (tc_p ? {h_pa[19:1], c_addr[12], c_addr[11:0]}
	                          : {h_pa, c_addr[11:0]})
	: c_addr;

//---------------------------------------------------------------------------
// walker state
//---------------------------------------------------------------------------

localparam W_IDLE = 4'd0;
localparam W_RA   = 4'd1;
localparam W_UA   = 4'd2;
localparam W_RB   = 4'd3;
localparam W_UB   = 4'd4;
localparam W_RC   = 4'd5;
localparam W_RI   = 4'd6;
localparam W_UC   = 4'd7;
localparam W_FILL = 4'd8;
localparam W_SWEEP = 4'd12;
localparam W_PTGO  = 4'd13;
localparam W_FLT  = 4'd9;
localparam W_DFLT = 4'd10;
localparam W_DROP = 4'd11;

reg  [3:0] wst;
reg        w_issued;
reg        w_pt;
reg [31:0] w_la;
reg        w_super, w_write, w_user;
reg [31:0] w_desc_addr;
reg [31:0] w_desc;
reg        w_wp;
reg        w_buserr;            // failed-search MMUSR B status (PTEST only)
reg [31:0] w_req_addr, w_req_wdat;
reg        w_req_wr;
reg        w_active;
reg        sw_pt;               // the running sweep is a PTEST pre-flush

wire  [6:0] w_pi  = w_la[24:18];
wire  [5:0] w_pgi = tc_p ? {1'b0, w_la[17:13]} : w_la[17:12];

wire walk_ack = w_active && w_issued && walker_ack && !walker_berr;
wire walk_err = w_active && w_issued && walker_berr;

// fill way selection: overwrite an existing mapping of the same page.
// q_b has been holding the fill row since the walk started (address_b is
// stable), so the compare and the read-modify-write composition are
// combinational over it.
wire  [3:0] f_set = tc_p ? w_la[16:13] : w_la[15:12];
wire [16:0] f_tag = tc_p ? {w_super, w_la[31:17], 1'b0}
                         : {w_super, w_la[31:16]};
reg         f_bank;
wire  [4:0] fill_row = {f_bank, f_set};
wire [EW-1:0] f_w0 = frow_q[0*EW +: EW];
wire [EW-1:0] f_w1 = frow_q[1*EW +: EW];
wire [EW-1:0] f_w2 = frow_q[2*EW +: EW];
wire [EW-1:0] f_w3 = frow_q[3*EW +: EW];
wire fhit0 = atc_v[{fill_row, 2'd0}] && (f_w0[44:28] == f_tag);
wire fhit1 = atc_v[{fill_row, 2'd1}] && (f_w1[44:28] == f_tag);
wire fhit2 = atc_v[{fill_row, 2'd2}] && (f_w2[44:28] == f_tag);
wire fhit3 = atc_v[{fill_row, 2'd3}] && (f_w3[44:28] == f_tag);
wire       f_way_hit = fhit0 | fhit1 | fhit2 | fhit3;
wire [1:0] f_way = fhit0 ? 2'd0 : fhit1 ? 2'd1 : fhit2 ? 2'd2 : fhit3 ? 2'd3
                 : atc_rr[fill_row];

// the freshly walked entry and the composed fill row
wire [19:0] f_pa_new   = tc_p ? {w_desc[31:13], 1'b0} : w_desc[31:12];
wire  [7:0] f_attr_new = {w_desc[10], w_desc[9:8], w_desc[7], w_desc[6:5],
                          w_desc[4], (w_wp | w_desc[2])};
// A failed 68040 search installs a VALID but nonresident entry. Future
// accesses fault on that entry without walking again until it is flushed.
wire [EW-1:0] f_ent_new = (wst == W_FLT) ? {1'b0, f_tag, 28'd0} :
                                        {1'b1, f_tag, f_pa_new, f_attr_new};
wire [ROWW-1:0] fill_wrow = {
	(f_way == 2'd3) ? f_ent_new : f_w3,
	(f_way == 2'd2) ? f_ent_new : f_w2,
	(f_way == 2'd1) ? f_ent_new : f_w1,
	(f_way == 2'd0) ? f_ent_new : f_w0 };
// the fill writes in the SAME cycle W_FILL commits the way choice:
// a registered strobe would land one cycle later, after the round
// robin pointer has already advanced under f_way's feet
assign fill_we = (((wst == W_FILL || wst == W_DFLT) && !w_active) ||
                  wst == W_FLT) && ce && nreset;

dpram #(5, ROWW) atc_ram
(
	.clock     (clk),
	.address_a (sweep_on ? sweep_row : a_row),
	.data_a    ({ROWW{1'b0}}),
	.wren_a    (1'b0),
	.q_a       (row_q),
	.address_b (fill_row),
	.data_b    (fill_wrow),
	.wren_b    (fill_we),
	.q_b       (frow_q)
);


// PTESTW has ordinary table-search history side effects only when the
// probed write is permitted.  A failed probe still reports W/S in MMUSR.
wire w_hist_m = w_write &&
                  (!w_pt || (!(w_wp || w_desc[2]) &&
                             !(w_user && w_desc[7])));
wire w_denied = !w_pt && ((w_user && w_desc[7]) ||
                          (w_write && (w_wp || w_desc[2])));

//---------------------------------------------------------------------------
// request forwarding
//---------------------------------------------------------------------------

// An enabled non-TTR translation may only forward once the lookup pipe
// is fresh: with a stale pipe need_walk/atc_fault are still low and the
// request would otherwise pass untranslated.
wire pass_ok = c_req && !c_flt && !need_walk && !ttr_fault && !atc_fault &&
               (!tc_e || ttr_hit || lk_fresh || u_hit) &&
               (wst == W_IDLE) && !w_active && !pf_req && !pt_req;

assign m_req   = pass_ok;
assign m_write = c_write;
assign m_instr = c_instr;
assign m_size  = c_size;
assign m_addr  = pa_out;
assign m_wdata = c_wdata;
assign m_fc    = c_fc;

// w_issued inserts a request-low cycle before each descriptor transaction.
// Besides making the interface unambiguous for a level-handshake backend,
// this prevents a held ack from completing the following descriptor.
assign walker_req  = w_active && w_issued;
assign walker_we   = w_req_wr;
assign walker_addr = w_req_addr;
assign walker_wdat = w_req_wdat;

// A downstream ack can only be generated for a request which m_req already
// admitted.  Re-evaluating pass_ok on the response creates a needless
// mem_addr -> ATC lookup -> core-ack critical path (over 50 logic levels in
// TimeQuest) and cannot reject any legitimate stale response because the
// core holds the request stable until ack.
assign c_ack   = m_ack;
assign c_rdata = m_rdata;
assign c_line_stb  = m_line_stb;
assign c_line_tag  = m_line_tag;
assign c_line_data = m_line_data;

assign phys_addr     = pa_out;
assign cache_inhibit = ttr_hit ? ttr_cm[1]
                     : (tc_e && atc_hit) ? h_cm[1] : 1'b0;
assign m_nocache     = cache_inhibit;

//---------------------------------------------------------------------------
// walker FSM (single always block: owns atc arrays and w_* state)
//---------------------------------------------------------------------------

task wrd;
	input [31:0] a;
	begin
		w_req_addr <= a;
		w_req_wr   <= 0;
		w_active   <= 1;
		w_issued   <= 0;
	end
endtask

task wwr;
	input [31:0] a;
	input [31:0] d;
	begin
		w_req_addr <= a;
		w_req_wdat <= d;
		w_req_wr   <= 1;
		w_active   <= 1;
		w_issued   <= 0;
	end
endtask

integer k;

always @(posedge clk) begin
	if (!nreset) begin
		wst <= W_IDLE;
		w_issued <= 0; w_pt <= 0;
		w_la <= 0; w_super <= 0; w_write <= 0; w_user <= 0;
		w_desc_addr <= 0; w_desc <= 0; w_wp <= 0; w_buserr <= 0;
		w_req_addr <= 0; w_req_wdat <= 0; w_req_wr <= 0;
		w_active <= 0; f_bank <= 0;
		c_flt <= 0;
		pt_done <= 0; pt_mmusr <= 0;
		pf_done <= 0;
		sweep_on <= 0; sweep_cnt <= 0; sw_pt <= 0;
		if (!atc_reset_seen) begin
			for (k = 0; k < 128; k = k + 1) atc_v[k] <= 0;
			for (k = 0; k < 32; k = k + 1) atc_rr[k] <= 0;
		end
		atc_reset_seen <= 1;
	end
	else if (ce) begin
		c_flt <= 0;
		pt_done <= 0;
		pf_done <= 0;
		if (w_active && !w_issued) w_issued <= 1;
		if (fill_we) begin
			atc_v[{fill_row, f_way}] <= 1;
			if (!f_way_hit) atc_rr[fill_row] <= atc_rr[fill_row] + 2'd1;
		end

		if (walk_err) begin
			// A physical bus error while fetching or updating a descriptor is
			// reported as an unsuccessful table search and cached as
			// nonresident. A probing PTEST also reports its MMUSR B bit.
			w_active <= 0;
			w_buserr <= 1;
			wst <= W_FLT;
		end
		else case (wst)
			W_IDLE: begin
				if (pf_req && !pf_done) begin
					// PFLUSHA (mode 11) needs no tags: clear every valid
					// flop in one cycle.  Every other variant reads tags/G
					// from the BRAM rows, so it sweeps them (W_SWEEP); the
					// core is stalled behind pf_req the whole time and
					// PFLUSH is rare, so the ~34 cycles are free.
					if (pf_mode == 2'b11) begin
						for (k = 0; k < 128; k = k + 1) atc_v[k] <= 0;
						pf_done <= 1;
					end
					else begin
						sweep_on  <= 1;
						sweep_cnt <= 0;
						sw_pt     <= 0;
						wst       <= W_SWEEP;
					end
				end
				else if (pt_req && !pt_done) begin
					// A PTEST first discards the matching entry in BOTH ATCs,
					// via the same sweep; the search itself starts from
					// W_SWEEP's completion.
					//
					// VERIFIED 2026-08-24 against an audit claiming Motorola
					// selects ONE ATC by DFC here.  It does not: PTEST calls
					// mmu_flush_atc(addr, super, true) (cpummu.cpp:1427) and
					// that walks both arrays --
					//   for (type=0; type<ATC_TYPE; type++)
					//     for (way=0; way<ATC_WAYS; way++) ...
					// (cpummu.cpp:1493).  DFC selects the array the probe
					// RESULT is installed in, which is pt_instr below, not
					// the array the pre-flush clears.
					sweep_on  <= 1;
					sweep_cnt <= 0;
					sw_pt     <= 1;
					wst       <= W_SWEEP;
				end
				else if (c_req && !c_flt && (ttr_fault || atc_fault)) begin
					c_flt <= 1;
				end
				else if (c_req && !c_flt && need_walk) begin
					w_pt    <= 0;
					w_la    <= c_addr;
					w_super <= a_super;
					w_user  <= !a_super;
					w_write <= c_write;
					w_wp    <= 0;
					w_buserr <= 0;
					f_bank  <= c_instr;
					wrd({(a_super ? srp[31:9] : urp[31:9]), 9'd0} +
					    {23'd0, c_addr[31:25], 2'b00});
					wst <= W_RA;
				end
			end

			// Tag sweep for PFLUSH page/nonglobal variants and the PTEST
			// pre-flush.  Two-cycle pipeline over the 32 rows: the row
			// addressed at count N is judged at count N+1 from q_a.
			W_SWEEP: begin : sweep
				reg [16:0] sw_tag;
				reg  [3:0] sw_set;
				reg        sw_match;
				integer    w;
				sw_tag = sw_pt ? (tc_p ? {pt_fc[2], pt_addr[31:17], 1'b0}
				                       : {pt_fc[2], pt_addr[31:16]})
				               : (tc_p ? {pf_fc[2], pf_addr[31:17], 1'b0}
				                       : {pf_fc[2], pf_addr[31:16]});
				sw_set = sw_pt ? (tc_p ? pt_addr[16:13] : pt_addr[15:12])
				               : (tc_p ? pf_addr[16:13] : pf_addr[15:12]);
				if (sweep_cnt != 0) begin : sweep_act
					reg [4:0] pr;
					pr = sweep_cnt[4:0] - 5'd1;
					for (w = 0; w < 4; w = w + 1) begin : sweep_way
						reg [EW-1:0] e;
						e = row_q[w*EW +: EW];
						if (sw_pt) begin
							if (pr[3:0] == sw_set && e[44:28] == sw_tag)
								atc_v[{pr, w[1:0]}] <= 0;
						end
						else begin
							sw_match = pf_mode[1] ||
							           (pr[3:0] == sw_set && e[44:28] == sw_tag);
							if (sw_match && (pf_mode[0] || !e[7]))
								atc_v[{pr, w[1:0]}] <= 0;
						end
					end
				end
				if (sweep_cnt == 6'd32) begin
					sweep_on  <= 0;
					sweep_cnt <= 0;
					if (!sw_pt) begin
						pf_done <= 1;
						wst <= W_IDLE;
					end
					else wst <= W_PTGO;
				end
				else sweep_cnt <= sweep_cnt + 1'd1;
			end

			// PTEST proper, after its pre-flush sweep
			W_PTGO: begin
					w_buserr <= 0;
					w_pt    <= 1;
					w_la    <= pt_addr;
					w_super <= pt_fc[2];
					w_user  <= !pt_fc[2];
					w_write <= pt_write;
					w_wp    <= 0;
					f_bank  <= pt_instr;
					if (pt_ttr_hit) begin
						// A TTR match reports T and R only -- the physical
						// address field stays clear -- and a write probe
						// against a write-protected TTR reports B.
						//
						// VERIFIED against the oracle 2026-08-24, because an
						// audit claimed Motorola returns T|R here and that
						// this is wrong.  cpummu.cpp:1429-1435 is explicit:
						//   if (ttr_match == TTR_NO_WRITE && write)
						//       regs.mmusr = MMU_MMUSR_B;
						//   else
						//       regs.mmusr = MMU_MMUSR_T | MMU_MMUSR_R;
						// with MMU_MMUSR_B = 1<<11 (include/cpummu.h:103),
						// T|R = 3, and mmu_match_ttr returning TTR_NO_WRITE
						// exactly on MMU_TTR_BIT_WRITE_PROTECT
						// (cpummu.cpp:573).  Manual-vs-reference goes to the
						// reference; t_mmu 38 pins it.
						pt_mmusr <= (pt_write && pt_ttr_w) ? 32'h0000_0800
						                                   : 32'h0000_0003;
						pt_done <= 1;
						w_pt <= 0;
						wst <= W_IDLE;
					end
					else begin
						// PTEST runs the table search even when translation
						// is disabled: TC.E gates ordinary accesses only,
						// the probe always walks URP/SRP (WinUAE has no
						// tc_e test in its PTEST path).
						wrd({(pt_fc[2] ? srp[31:9] : urp[31:9]), 9'd0} +
						    {23'd0, pt_addr[31:25], 2'b00});
						wst <= W_RA;
					end
			end

			W_RA: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				if (!walker_data[1]) wst <= W_FLT;   // UDT invalid
				else begin
					w_wp <= w_wp | walker_data[2];
					if (!walker_data[3]) begin
						wwr(w_req_addr, walker_data | 32'h8);
						wst <= W_UA;
					end
					else begin
						wrd({walker_data[31:9], 9'd0} + {23'd0, w_pi, 2'b00});
						wst <= W_RB;
					end
				end
			end

			W_UA: if (walk_ack) begin
				w_active <= 0;
				wrd({w_desc[31:9], 9'd0} + {23'd0, w_pi, 2'b00});
				wst <= W_RB;
			end

			W_RB: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				if (!walker_data[1]) wst <= W_FLT;
				else begin
					w_wp <= w_wp | walker_data[2];
					if (!walker_data[3]) begin
						wwr(w_req_addr, walker_data | 32'h8);
						wst <= W_UB;
					end
					else begin
						wrd(pgtbl_addr(walker_data) + {24'd0, w_pgi, 2'b00});
						wst <= W_RC;
					end
				end
			end

			W_UB: if (walk_ack) begin
				w_active <= 0;
				wrd(pgtbl_addr(w_desc) + {24'd0, w_pgi, 2'b00});
				wst <= W_RC;
			end

			W_RC: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				case (walker_data[1:0])
					2'b00: wst <= W_FLT;
					2'b10: begin
						wrd(walker_data & 32'hFFFF_FFFC);
						wst <= W_RI;
					end
					default: wst <= W_UC;
				endcase
			end

			W_RI: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				// an indirect descriptor must resolve to a resident page
				if (walker_data[1:0] == 2'b00 || walker_data[1:0] == 2'b10) wst <= W_FLT;
				else wst <= W_UC;
			end

			W_UC: begin
				// A valid page descriptor is used by the table search even when
				// its protection attributes deny the access.  The 68040 therefore
				// sets U before reporting the access error, but must not set M for
				// a denied write.  Defer c_flt until the descriptor writeback has
				// completed, then keep the walker quiescent until the core drops
				// the held request; otherwise it can immediately begin a second
				// walk before the access-error state has consumed the fault pulse.
				if (w_denied) begin
					if (!w_desc[3]) begin
						wwr(w_desc_addr, w_desc | 32'h8);
						w_desc <= w_desc | 32'h8;
					end
					wst <= W_DFLT;
				end
				else if (!w_desc[3] || (w_hist_m && !w_desc[4])) begin
					wwr(w_desc_addr, w_desc | 32'h8 |
					    (w_hist_m ? 32'h10 : 32'h0));
					w_desc <= w_desc | 32'h8 | (w_hist_m ? 32'h10 : 32'h0);
					wst <= W_FILL;
				end
				else wst <= W_FILL;
			end

			// Protection fault after the optional Used-bit writeback.
			W_DFLT: begin
				if (w_active) begin
					if (walk_ack) w_active <= 0;
				end
				else begin
					c_flt <= 1;
					wst <= W_DROP;
				end
			end

			// c_flt is sampled by the core one ce edge after it is registered.
			// Do not return to W_IDLE until that edge has made c_req fall.
			W_DROP: begin
				if (!c_req) wst <= W_IDLE;
			end

			W_FILL: begin
				if (w_active) begin
					if (walk_ack) w_active <= 0;
				end
				else if (w_pt) begin
					// PTEST reports the PAGE FRAME, not the translated
					// address of the probed LA.  In 8K mode that means
					// bit 12 is CLEAR: the frame is 8K-aligned, and the
					// LA's bit 12 belongs to the page offset.  This used
					// to substitute w_la[12] on the reasoning that the
					// "true PA" is what matters -- but WinUAE keeps the
					// two separate and so does the architecture:
					// mmu_translate returns the full PA while PTEST
					// returns `desc & mmu_pagemaski` (~0x1FFF at 8K), so
					// its MMUSR frame has bit 12 clear.  The differential
					// comparator had been masking this bit, which is the
					// only reason an "all seeds match" ever held here.
					pt_mmusr <= (tc_p ? {w_desc[31:13], 13'd0}
					                  : {w_desc[31:12], 12'd0}) |
					            {21'd0, w_desc[10], w_desc[9:8], w_desc[7],
					             w_desc[6:5], w_desc[4], 1'b0,
					             (w_wp | w_desc[2]), 1'b0, 1'b1};
					pt_done <= 1;
					w_pt <= 0;
					wst <= W_IDLE;
				end
				else begin
					wst <= W_IDLE;   // the held request now hits and forwards
				end
			end

			W_FLT: begin
				if (w_pt) begin
					pt_mmusr <= w_buserr ? 32'h0000_0800 : 32'd0;
					pt_done <= 1;
					w_pt <= 0;
					wst <= W_IDLE;
				end
				else begin
					c_flt <= 1;
					wst <= W_DROP;
				end
			end

			default: wst <= W_IDLE;
		endcase
	end
end

//---------------------------------------------------------------------------
// hint-side translation
//---------------------------------------------------------------------------
// The core's address hint (one cycle ahead of the request) is translated
// here through the same hit copy and TTRs and registered, so that the
// cache can qualify a one-clock hit in the request cycle against a
// registered physical tag instead of the live translation.  Registered
// every clock from the live hint bus; m_hint_match asserts when the
// request now presented is the address, space and supervisor state the
// copy was made from and nothing that decides a translation has moved
// since (the copy itself, TC, the TTRs, a walk, sweep or fill).
wire [3:0]    hn_set = tc_p ? c_hint_addr[16:13] : c_hint_addr[15:12];
wire [4:0]    hn_row = {c_hint_instr, hn_set};
wire [16:0]   hn_tag = tc_p ? {a_super, c_hint_addr[31:17], 1'b0}
                           : {a_super, c_hint_addr[31:16]};
wire          uh_hit = u_valid[{c_hint_instr,hn_row[1:0]}] && (u_row[{c_hint_instr,hn_row[1:0]}] == hn_row) &&
                       (u_tag[{c_hint_instr,hn_row[1:0]}] == hn_tag);
// A held request repeats itself on the hint bus, so when the lookup pipe
// resolves the request this cycle (the copy is being refilled from it)
// the same entry translates the hint: without this every first access
// to a page after a copy miss lost the one-clock hit (3 % of boot
// dispatches).
wire          hn_pipe = pipe_hit && (c_hint_addr == c_addr) && (c_hint_instr == c_instr);
wire [EW-1:0] uh_ent = uh_hit ? u_ent[{c_hint_instr,hn_row[1:0]}] : pipe_ent;
wire [19:0]   uh_pa  = uh_ent[27:8];
wire [31:0]   hn_ttra = c_hint_instr ? itt0 : dtt0;
wire [31:0]   hn_ttrb = c_hint_instr ? itt1 : dtt1;
wire          hn_ttr_a = ttr_match(hn_ttra, c_hint_addr, a_super);
wire          hn_ttr_b = ttr_match(hn_ttrb, c_hint_addr, a_super);
wire          hn_ttr  = hn_ttr_a | hn_ttr_b;
// The hint's cacheability and read protection, registered with its
// translation, so the cache's one-clock acknowledge needs nothing from
// the request's live translation (the ATC read, hit compare and pa mux
// were 7 ns in front of the acknowledge, 2026-09-15): a cache-inhibited
// or supervisor-only page never vouches, and the request then takes the
// ordinary path (bypass, or the access error).
wire  [1:0]   hn_ttr_cm = hn_ttr_a ? hn_ttra[6:5] : hn_ttrb[6:5];
wire          hn_ci    = hn_ttr ? hn_ttr_cm[1] :
                         (tc_e && (uh_hit || hn_pipe)) ? uh_ent[3] : 1'b0;
wire          hn_sprot = tc_e && !hn_ttr && (uh_hit || hn_pipe) && !a_super && uh_ent[4];
wire [31:0]   hn_pa   = hn_ttr ? c_hint_addr :
                        (tc_e && (uh_hit || hn_pipe)) ? (tc_p ? {uh_pa[19:1], c_hint_addr[12], c_hint_addr[11:0]}
                                                 : {uh_pa, c_hint_addr[11:0]})
                        : c_hint_addr;
wire          mmu_quiet = (wst == W_IDLE) && !w_active && !pf_req && !pt_req &&
                          !sweep_on && !fill_we;
wire          hn_ok    = (hn_ttr || !tc_e || uh_hit || hn_pipe) && mmu_quiet &&
                         !hn_ci && !hn_sprot;
// The hint's write side, registered with the rest: the page carries no
// accumulated write protection (entry bit 0, or the TTR's W bit for a
// transparent hint) and its modified bit is already set (entry bit 1: the
// first write to a page walks to set M, and must).  A store whose request
// is such a hint is acknowledged by the cache in its request cycle
// (the one-clock posted store, 2026-09-19); every other store takes the
// registered path, faults included.
wire          hn_wp    = hn_ttr ? (hn_ttr_a ? hn_ttra[2] : hn_ttrb[2]) :
                         (tc_e && (uh_hit || hn_pipe)) ? (uh_ent[0] || !uh_ent[1]) : 1'b0;
wire          hn_wok   = hn_ok && !hn_wp;
reg  [31:0]   hq_addr;
reg           hq_instr, hq_super, hq_ok, hq_wok;
reg  [21:0]   hq_ptag;
reg [127:0]   ttr_q;
always @(posedge clk) begin
	hq_addr  <= c_hint_addr;
	hq_instr <= c_hint_instr;
	hq_super <= a_super;
	hq_ptag  <= hn_pa[31:10];
	hq_ok    <= nreset && hn_ok;
	hq_wok   <= nreset && hn_wok;
	ttr_q    <= {itt0, itt1, dtt0, dtt1};
end
assign m_hint_addr  = c_hint_addr;
assign m_hint_instr = c_hint_instr;
assign m_hint_ptag  = hq_ptag;
assign m_hint_match = c_req && hq_ok && (hq_addr == c_addr) && (hq_instr == c_instr) &&
                      (hq_super == a_super) && (tc == u_tc) &&
                      (ttr_q == {itt0, itt1, dtt0, dtt1}) && mmu_quiet;
assign m_hint_wmatch = m_hint_match && hq_wok;

// P175: the instruction hint, translated the same way (instruction space,
// the per-space copy's instruction entry, the ITTs) and registered.
wire [3:0]    hi_set = tc_p ? c_ihint_addr[16:13] : c_ihint_addr[15:12];
wire [4:0]    hi_row = {1'b1, hi_set};
wire [16:0]   hi_tag = tc_p ? {a_super, c_ihint_addr[31:17], 1'b0}
                           : {a_super, c_ihint_addr[31:16]};
wire          ui_hit = u_valid[{1'b1,hi_row[1:0]}] && (u_row[{1'b1,hi_row[1:0]}] == hi_row) && (u_tag[{1'b1,hi_row[1:0]}] == hi_tag);
wire          hi_pipe = pipe_hit && (c_ihint_addr == c_addr) && c_instr;
wire [EW-1:0] ui_ent = ui_hit ? u_ent[{1'b1,hi_row[1:0]}] : pipe_ent;
wire [19:0]   ui_pa  = ui_ent[27:8];
wire          hi_ttr_a = ttr_match(itt0, c_ihint_addr, a_super);
wire          hi_ttr_b = ttr_match(itt1, c_ihint_addr, a_super);
wire          hi_ttr  = hi_ttr_a | hi_ttr_b;
wire  [1:0]   hi_ttr_cm = hi_ttr_a ? itt0[6:5] : itt1[6:5];
wire          hi_ci    = hi_ttr ? hi_ttr_cm[1] :
                         (tc_e && (ui_hit || hi_pipe)) ? ui_ent[3] : 1'b0;
wire          hi_sprot = tc_e && !hi_ttr && (ui_hit || hi_pipe) && !a_super && ui_ent[4];
wire [31:0]   hi_pa   = hi_ttr ? c_ihint_addr :
                        (tc_e && (ui_hit || hi_pipe)) ? (tc_p ? {ui_pa[19:1], c_ihint_addr[12], c_ihint_addr[11:0]}
                                                 : {ui_pa, c_ihint_addr[11:0]})
                        : c_ihint_addr;
wire          hi_ok    = (hi_ttr || !tc_e || ui_hit || hi_pipe) && mmu_quiet &&
                         !hi_ci && !hi_sprot;
reg  [31:0]   hqi_addr;
reg           hqi_super, hqi_ok;
reg  [21:0]   hqi_ptag;
always @(posedge clk) begin
	hqi_addr  <= c_ihint_addr;
	hqi_super <= a_super;
	hqi_ptag  <= hi_pa[31:10];
	hqi_ok    <= nreset && hi_ok;
end
assign m_ihint_addr  = c_ihint_addr;
assign m_ihint_ptag  = hqi_ptag;
assign m_ihint_match = c_req && c_instr && hqi_ok && (hqi_addr == c_addr) &&
                       (hqi_super == a_super) && (tc == u_tc) &&
                       (ttr_q == {itt0, itt1, dtt0, dtt1}) && mmu_quiet;

endmodule

