//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_fetch.sv - instruction fetch: I-ATC, instruction cache and the   //
// instruction queue                                                        //
//                                                                          //
// Instruction cache (MC68040UM section 4): 4 Kbytes, four ways of 64 sets //
// of 16-byte lines, physically tagged (PA31-10; the set is LA9-4), a      //
// valid bit per line, replacement by the first invalid way, else a 2-bit  //
// counter.  Lines are filled by a burst (critical long word first, TBI    //
// handled by the BIU, TCI: the line is not allocated).  Cache-inhibited   //
// pages and CACR.IE = 0 fetch long words.                                  //
//                                                                          //
// I-ATC (3.3): 64 entries like the D-ATC, translating supervisor and user //
// program space; ITT0/ITT1 transparent translation.  An ATC miss asks the //
// DMU's engine for a table walk; the walk installs the entry here         //
// (iatc_wr) and the fetch looks again.                                    //
//                                                                          //
// Pipeline, one aligned long word (two words) per cycle -- D1 decodes at  //
// most one instruction a cycle, and the queue absorbs long ones:         //
//   F0  fpc: I-ATC and ITT lookup, cache tag and data RAM read            //
//   F1  translation, tag compare; a hit is the long word's words from    //
//       the fetch address on.  Anything else rolls fpc back and runs in   //
//       miss engine (walk, line fill, cache-inhibited stream, fault)      //
//   F2  predecode                                                          //
//   F3  append to the queue                                                //
//                                                                          //
// The queue holds QN words: slot 0 is always the next word D1 decodes.    //
// A redirect (D1 branch or WB) empties the queue and the pipeline.  A     //
// faulted fetch marks its words; D1 raises the fault only if it uses them //
// (8.2.1).  Only the first fetch of a redirected stream is a demand       //
// fetch; a fault on the lookahead after it is speculative, and the back   //
// end refetches it on demand before it becomes an access error.           //
//                                                                          //
// CINV/CPUSH of the instruction cache arrive from the DMU (ic_inv): all   //
// in one cycle, a line in two, a page by a scan of the 64 sets.  Bus     //
// snooping (sn_inv) invalidates a line the same way (MC68040UM table   //
// 4-3, V5/V6).                                                            //
//--------------------------------------------------------------------------//

module ap68040_fetch
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,

	input  logic        redir_v,      // highest priority
	input  logic [31:0] redir_pc,
	input  logic        d_redir_v,    // decoder (predicted branch)
	input  logic [31:0] d_redir_pc,
	input  logic        stop,         // do not start new fetches

	input  logic        smode,        // S bit: program space and TM 6/2
	input  logic [31:0] cacr,
	input  logic [31:0] tc,
	input  logic [31:0] itt0,
	input  logic [31:0] itt1,

	// decoder side
	output logic [15:0] win [8],
	output logic  [7:0] win_flt,      // the word came from a faulted fetch
	output logic  [7:0] win_fdem,     // ... the first of a redirected stream
	output logic  [7:0] win_fatc,     // ... an ATC (MMU) fault, else bus error
	output pd_t         pd0,          // predecode of win[0]
	output logic  [3:0] qcnt,         // words available in win (0..8)
	output logic [31:0] qpc,
	input  logic  [2:0] consume,
	output logic        q_odd,        // the stream starts at an odd address

	// DMU: table walks for the I-ATC, I-ATC maintenance, cache invalidation
	output logic        iw_req,
	output logic [31:0] iw_va,
	output logic        iw_fc2,
	input  logic        iw_done,
	input  logic        iatc_wr,
	input  logic [31:0] iatc_wla,
	input  logic        iatc_wfc2,
	input  atce_t       iatc_went,
	input  logic        iatc_flush_all,
	input  logic        iatc_flush_page,
	input  logic        iatc_flush_ng,
	input  logic [31:0] iatc_flush_la,
	input  logic        iatc_flush_fc2,
	input  logic        ic_inv,
	input  logic  [1:0] ic_inv_scope,  // 1 line, 2 page, 3 all
	input  logic [31:0] ic_inv_pa,
	output logic        ic_inv_done,
	input  logic        sn_inv,        // snooper: invalidate the line at sn_inv_pa
	input  logic [31:0] sn_inv_pa,
	input  logic        sn_inv_all,    // ... or every line
	output logic        sn_inv_done,

	// BIU client
	output logic        b_req,
	output busreq_t     b_breq,
	input  logic        b_gnt,
	input  logic        b_done,
	input  logic        b_err,
	input  logic        b_rvalid,
	input  logic [31:0] b_rdata,
	input  logic  [1:0] b_rbeat,
	input  logic        b_rtci
);

localparam int QN = 12;          // 16 measured no faster

wire tc_e  = tc[15];
wire tc_p  = tc[14];
wire ic_en = cacr[15];

function automatic logic ttr_hit(input logic [31:0] t, input logic [31:0] a,
                                 input logic s);
	ttr_hit = t[15] && (((a[31:24] ^ t[31:24]) & ~t[23:16]) == 8'd0) &&
	          (t[14] || (t[13] == s));
endfunction

//--------------------------------------------------------------------------
// queue
//--------------------------------------------------------------------------
logic [15:0]   qw [QN];
pd_t           qp [QN];
logic [QN-1:0] qf, qd, qa;     // faulted, demand fetch, ATC fault
logic  [4:0]   cnt;
logic [31:0]   qpc_r;
logic          odd;

assign qcnt  = (cnt > 5'd8) ? 4'd8 : cnt[3:0];
assign qpc   = qpc_r;
assign q_odd = odd;
assign pd0   = qp[0];

always_comb begin
	for (int i = 0; i < 8; i++) begin
		win[i]      = qw[i];
		win_flt[i]  = qf[i];
		win_fdem[i] = qd[i];
		win_fatc[i] = qa[i];
	end
end

// F2: the words of a chunk, predecoded into F3; F3: appended this cycle
logic  [1:0]  f2_n;
logic [15:0]  f2_w [2];
logic         f2_f, f2_d, f2_a;
pd_t          f2_p [2];
logic  [1:0]  f3_n;
logic [15:0]  f3_w [2];
pd_t          f3_p [2];
logic         f3_f, f3_d, f3_a;
genvar gp;
generate
	for (gp = 0; gp < 2; gp++) begin : g_pd
		ap68040_predec pd (.op(f2_w[gp]), .pd(f2_p[gp]));
	end
endgenerate

//--------------------------------------------------------------------------
// I-ATC
//--------------------------------------------------------------------------
logic [31:0] fpc;              // F0 fetch address (A0 clear)
logic        atc_hit;
atce_t       atc_e;
ap68040_atc iatc (
	.clk(clk), .nreset(nreset), .p8k(tc_p),
	.la(iatc_flush_page ? iatc_flush_la : fpc),
	.fc2(iatc_flush_page ? iatc_flush_fc2 : smode),
	.hit(atc_hit), .ent(atc_e),
	.wr(iatc_wr), .wla(iatc_wla), .wfc2(iatc_wfc2), .went(iatc_went),
	.flush_all(iatc_flush_all), .flush_page(iatc_flush_page),
	.flush_nonglobal(iatc_flush_ng)
);

//--------------------------------------------------------------------------
// cache arrays: tags and data in block RAM (read address registered in
// F0), valid bits in flip-flops
//--------------------------------------------------------------------------
logic  [3:0]   iv [64];
logic  [1:0]   rr;
logic  [5:0]   c_raddr;
logic          c_we;
logic  [5:0]   c_waddr;
logic  [1:0]   c_wway;
logic [21:0]   c_wtag;
logic [127:0]  cd [4];
logic [23:0]   ct [4];

logic [127:0] m_line;            // the line being filled (and the line buffer)
genvar gw;
generate
	for (gw = 0; gw < 4; gw++) begin : g_way
		ap68040_sdp_be #(.AW(6), .NB(16), .OREG(0)) dram (
			.clk(clk), .we(c_we && c_wway == 2'(gw)), .waddr(c_waddr),
			.wbe(16'hFFFF), .wdata(m_line), .raddr(c_raddr), .oce(1'b1), .q(cd[gw])
		);
		ap68040_sdp_be #(.AW(6), .NB(3), .OREG(0)) tram (
			.clk(clk), .we(c_we && c_wway == 2'(gw)), .waddr(c_waddr),
			.wbe(3'b111), .wdata({2'b00, c_wtag}), .raddr(c_raddr), .oce(1'b1), .q(ct[gw])
		);
	end
endgenerate

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
typedef enum logic [2:0] {
	S_RUN,     // pipelined fetch
	S_WALK,    // waiting for the DMU's table walk
	S_FILL,    // line fill burst
	S_CI,      // cache-inhibited long word stream
	S_HALT     // a fault was queued: wait for a redirect
} st_t;
st_t st;

logic        fnew;             // the next F0 starts a redirected stream

// F1
logic        f1_v;
logic [31:0] f1_pc;
logic        f1_s, f1_dem;
logic        f1_hit;
atce_t       f1_e;

// line buffer: the last line filled (it also serves TCI lines)
logic        lb_v;             // m_line holds the line at lb_pa (the last fill):
logic [27:0] lb_pa;            // it is also the cache's write data

// miss engine
logic        b_req_r;          // request pending (not granted yet)
logic        busy;             // a bus transaction is latched
logic        stale;            // ... for a stream that was redirected away
logic [31:0] m_la;             // the fetch address
logic [31:0] m_pa;
logic  [1:0] m_upa;
logic        m_s, m_dem, m_ci, m_alloc;
logic  [1:0] m_way;

// invalidation
logic        inv_busy, inv_hold, inv_ph;
logic  [6:0] inv_i;
logic        inv_sn;           // the running invalidation is the snooper's
logic  [1:0] inv_scope;
logic [31:0] inv_pa;

wire        redir_any = redir_v || d_redir_v;
wire [31:0] redir_npc = redir_v ? redir_pc : d_redir_pc;

//--------------------------------------------------------------------------
// F1 evaluation
//--------------------------------------------------------------------------
logic        x_ok, x_flt, x_walk, x_cach, x_hit, x_lb;
logic [31:0] x_pa;
logic  [1:0] x_cm, x_upa, x_way;
logic [31:0] x_chunk;
always_comb begin
	logic tt0, tt1;
	logic [21:0] tg;
	x_ok = 1'b0; x_flt = 1'b0; x_walk = 1'b0;
	x_pa = f1_pc; x_cm = 2'b00; x_upa = 2'b00;
	tt0 = ttr_hit(itt0, f1_pc, f1_s);
	tt1 = ttr_hit(itt1, f1_pc, f1_s);
	if (tt0 || tt1) begin
		x_ok  = 1'b1;
		x_cm  = tt0 ? itt0[6:5] : itt1[6:5];
		x_upa = tt0 ? itt0[9:8] : itt1[9:8];
	end
	else if (!tc_e)
		x_ok = 1'b1;               // cachable (MC68040UM 3.1.2)
	else if (f1_hit) begin
		x_pa  = {f1_e.pa[19:1], tc_p ? f1_pc[12] : f1_e.pa[0], f1_pc[11:0]};
		x_cm  = f1_e.cm;
		x_upa = f1_e.upa;
		if (!f1_e.r || (f1_e.s && !f1_s)) x_flt = 1'b1;   // nonresident, supervisor only
		else x_ok = 1'b1;
	end
	else x_walk = 1'b1;
	x_cach = x_ok && ic_en && !x_cm[1];
	tg     = x_pa[31:10];
	x_hit  = 1'b0;
	x_way  = 2'd0;
	for (int i = 3; i >= 0; i--)
		if (iv[f1_pc[9:4]][i] && ct[i][21:0] == tg) begin x_hit = 1'b1; x_way = 2'(i); end
	x_hit  = x_hit && x_cach;
	x_lb   = x_cach && lb_v && lb_pa == x_pa[31:4];
	x_chunk = x_lb ? m_line[127 - 32 * f1_pc[3:2] -: 32]
	               : cd[x_way][127 - 32 * f1_pc[3:2] -: 32];
end
wire f1_go      = f1_v && !redir_any;
wire f1_deliver = f1_go && (x_hit || x_lb);

//--------------------------------------------------------------------------
// F0 issue: room for this chunk and those in flight
//--------------------------------------------------------------------------
wire [5:0] inflight = (f1_v ? 6'd2 : 6'd0) + {4'd0, f2_n} + {4'd0, f3_n};
wire       room     = {1'b0, cnt} + inflight <= 6'(QN - 2);
// a snooped invalidation goes first (the snooper has priority, 4.5)
wire       f0_go    = (st == S_RUN) && !odd && !stop && !redir_any &&
                      !iatc_flush_page && !inv_busy && !sn_inv && room;

assign c_raddr = inv_busy ? inv_i[5:0] : fpc[9:4];

//--------------------------------------------------------------------------
// BIU request
//--------------------------------------------------------------------------
assign b_req = b_req_r;
always_comb begin
	b_breq = '0;
	b_breq.addr = {m_pa[31:2], 2'b00};
	b_breq.rd   = 1'b1;
	b_breq.tt   = TT_NORMAL;
	b_breq.tm   = m_s ? TM_SCODE : TM_UCODE;
	b_breq.upa  = m_upa;
	if (st == S_FILL) begin
		b_breq.siz = SIZ_LINE;
		b_breq.tln = m_way;
	end
	else begin
		b_breq.siz = SIZ_L;
		b_breq.ci  = m_ci;
	end
end

assign iw_va  = m_la;
assign iw_fc2 = m_s;

// a transaction still outstanding after this cycle
wire out_next = (b_req_r && !b_gnt) || b_gnt || (busy && !(b_done || b_err));

//--------------------------------------------------------------------------
// sequential
//--------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (!nreset) begin
		cnt   <= '0;
		qf    <= '0; qd <= '0; qa <= '0;
		qpc_r <= '0;
		odd   <= 1'b0;
		fpc   <= '0;
		fnew  <= 1'b1;
		st    <= S_RUN;
		f1_v  <= 1'b0; f1_pc <= '0; f1_s <= 1'b0; f1_dem <= 1'b0; f1_hit <= 1'b0; f1_e <= '0;
		f2_n  <= '0; f2_f <= 1'b0; f2_d <= 1'b0; f2_a <= 1'b0;
		for (int i = 0; i < 2; i++) f2_w[i] <= '0;
		f3_n  <= '0; f3_f <= 1'b0; f3_d <= 1'b0; f3_a <= 1'b0;
		for (int i = 0; i < 2; i++) begin f3_w[i] <= '0; f3_p[i] <= '0; end
		lb_v  <= 1'b0; lb_pa <= '0;
		busy  <= 1'b0; stale <= 1'b0; b_req_r <= 1'b0;
		m_la  <= '0; m_pa <= '0; m_upa <= '0; m_s <= 1'b0; m_dem <= 1'b0; m_ci <= 1'b0;
		m_alloc <= 1'b0; m_way <= '0; m_line <= '0;
		iw_req <= 1'b0;
		c_we  <= 1'b0; c_waddr <= '0; c_wway <= '0; c_wtag <= '0;
		rr    <= '0;
		inv_busy <= 1'b0; inv_hold <= 1'b0; inv_ph <= 1'b0; inv_i <= '0;
		inv_sn <= 1'b0; inv_scope <= '0; inv_pa <= '0;
		ic_inv_done <= 1'b0;
		sn_inv_done <= 1'b0;
		for (int i = 0; i < 64; i++) iv[i] <= 4'd0;
		for (int i = 0; i < QN; i++) begin qw[i] <= '0; qp[i] <= '0; end
	end
	else begin
		logic [4:0] keep;
		c_we        <= 1'b0;
		ic_inv_done <= 1'b0;
		sn_inv_done <= 1'b0;
		inv_hold    <= 1'b0;

		//------------------------------------------------------------------
		// queue: shift out what D1 consumed, append F3; F2 -> F3
		//------------------------------------------------------------------
		keep = cnt - {2'b00, consume};
		for (int i = 0; i < QN; i++) begin
			logic [5:0] src;
			logic [4:0] k;
			src = 6'(i) + {3'b000, consume};
			k   = 5'(i) - keep;
			if (5'(i) < keep) begin
				qw[i] <= qw[src[4:0]];
				qp[i] <= qp[src[4:0]];
				qf[i] <= qf[src[4:0]];
				qd[i] <= qd[src[4:0]];
				qa[i] <= qa[src[4:0]];
			end
			else if (k < {3'b000, f3_n}) begin
				qw[i] <= f3_w[k[0]];
				qp[i] <= f3_p[k[0]];
				qf[i] <= f3_f;
				qd[i] <= f3_d;
				qa[i] <= f3_a;
			end
		end
		cnt   <= keep + {3'b000, f3_n};
		qpc_r <= qpc_r + {28'd0, consume, 1'b0};
		f3_n  <= f2_n;
		f3_f  <= f2_f; f3_d <= f2_d; f3_a <= f2_a;
		for (int i = 0; i < 2; i++) begin f3_w[i] <= f2_w[i]; f3_p[i] <= f2_p[i]; end
		f2_n  <= '0;

		//------------------------------------------------------------------
		// F0 -> F1
		//------------------------------------------------------------------
		f1_v <= 1'b0;
		if (f0_go) begin
			f1_v   <= 1'b1;
			f1_pc  <= fpc;
			f1_s   <= smode;
			f1_dem <= fnew;
			f1_hit <= atc_hit;
			f1_e   <= atc_e;
			fnew   <= 1'b0;
			fpc    <= {fpc[31:2], 2'b00} + 32'd4;
		end

		//------------------------------------------------------------------
		// F1: a hit goes to F2; anything else to the miss engine, and F0
		// rolls back to the missed address
		//------------------------------------------------------------------
		if (f1_deliver) begin
			f2_n    <= f1_pc[1] ? 2'd1 : 2'd2;
			f2_w[0] <= f1_pc[1] ? x_chunk[15:0] : x_chunk[31:16];
			f2_w[1] <= x_chunk[15:0];
			f2_f <= 1'b0; f2_d <= 1'b0; f2_a <= 1'b0;
		end
		else if (f1_go) begin
			f1_v   <= 1'b0;
			fpc    <= f1_pc;
			fnew   <= f1_dem;
			m_la   <= f1_pc;
			m_pa   <= x_pa;
			m_upa  <= x_upa;
			m_s    <= f1_s;
			m_dem  <= f1_dem;
			m_ci   <= x_cm[1];
			if (x_walk) begin
				st     <= S_WALK;
				iw_req <= 1'b1;
			end
			else if (x_flt) begin
				// ATC fault: the long word's words carry it
				f2_n <= f1_pc[1] ? 2'd1 : 2'd2;
				for (int i = 0; i < 2; i++) f2_w[i] <= 16'd0;
				f2_f <= 1'b1; f2_d <= f1_dem; f2_a <= 1'b1;
				st   <= S_HALT;
			end
			else if (x_cach) begin
				logic [3:0] vv;
				vv = iv[f1_pc[9:4]];
				st      <= S_FILL;
				lb_v    <= 1'b0;          // its beats go into m_line
				m_alloc <= 1'b1;
				m_way   <= !vv[0] ? 2'd0 : !vv[1] ? 2'd1 : !vv[2] ? 2'd2 : !vv[3] ? 2'd3 : rr;
				b_req_r <= 1'b1;
			end
			else begin
				st      <= S_CI;
				b_req_r <= 1'b1;
			end
		end

		//------------------------------------------------------------------
		// miss engine
		//------------------------------------------------------------------
		if (b_gnt) begin
			b_req_r <= 1'b0;
			busy    <= 1'b1;
		end
		case (st)
		S_WALK:
			if (iw_done) begin
				iw_req <= 1'b0;
				st     <= S_RUN;
			end
		S_FILL: begin
			logic [127:0] ln;
			ln = m_line;
			if (b_rvalid) begin
				ln[127 - 32 * b_rbeat -: 32] = b_rdata;
				m_line <= ln;
				if (b_rtci) m_alloc <= 1'b0;
			end
			if (busy && b_done) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (m_alloc && !(b_rvalid && b_rtci)) begin
					c_we    <= 1'b1;
					c_waddr <= m_pa[9:4];
					c_wway  <= m_way;
					c_wtag  <= m_pa[31:10];
					iv[m_pa[9:4]][m_way] <= 1'b1;
					if (iv[m_pa[9:4]] == 4'hF) rr <= rr + 2'd1;
				end
				lb_v  <= 1'b1;
				lb_pa <= m_pa[31:4];
				st    <= S_RUN;
			end
			if (busy && b_err) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (!stale) begin
					f2_n <= m_la[1] ? 2'd1 : 2'd2;
					for (int i = 0; i < 2; i++) f2_w[i] <= 16'd0;
					f2_f <= 1'b1; f2_d <= m_dem; f2_a <= 1'b0;
					st   <= S_HALT;
				end
				else st <= S_RUN;
			end
		end
		S_CI: begin
			if (busy && b_rvalid && !stale) begin
				if (m_la[1]) begin
					f2_n    <= 2'd1;
					f2_w[0] <= b_rdata[15:0];
				end
				else begin
					f2_n    <= 2'd2;
					f2_w[0] <= b_rdata[31:16];
					f2_w[1] <= b_rdata[15:0];
				end
				f2_f <= 1'b0; f2_d <= 1'b0; f2_a <= 1'b0;
			end
			if (busy && b_done) begin
				logic [31:0] nla;
				nla   = {m_la[31:2], 2'b00} + 32'd4;
				busy  <= 1'b0;
				stale <= 1'b0;
				if (stale) st <= S_RUN;
				else begin
					m_la  <= nla;
					m_pa  <= {m_pa[31:2], 2'b00} + 32'd4;
					m_dem <= 1'b0;
					fpc   <= nla;
					fnew  <= 1'b0;
					// the page ends (or the cache mode changed): translate again
					if (nla[11:0] == 12'd0 || (!m_ci && ic_en)) st <= S_RUN;
				end
			end
			else if (busy && b_err) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (!stale) begin
					f2_n    <= m_la[1] ? 2'd1 : 2'd2;
					f2_w[0] <= 16'd0;
					f2_w[1] <= 16'd0;
					f2_f <= 1'b1; f2_d <= m_dem; f2_a <= 1'b0;
					st   <= S_HALT;
				end
				else st <= S_RUN;
			end
			else if (!busy && !b_req_r && !b_gnt && !redir_any &&
			         {1'b0, cnt} + {4'd0, f2_n} + {4'd0, f3_n} <= 6'(QN - 2))
				b_req_r <= 1'b1;        // the next long word of the stream
		end
		default: ;
		endcase

		//------------------------------------------------------------------
		// redirect: empty the queue and the pipeline.  A walk runs to its
		// end (the entry is installed); a line fill completes and is
		// installed; a cache-inhibited read completes and is discarded.
		//------------------------------------------------------------------
		if (redir_any) begin
			cnt   <= '0;
			f2_n  <= '0;
			f3_n  <= '0;
			f1_v  <= 1'b0;
			fpc   <= {redir_npc[31:1], 1'b0};
			qpc_r <= redir_npc;
			odd   <= redir_npc[0];
			fnew  <= 1'b1;
			if (st == S_FILL || st == S_CI) begin
				if (out_next) stale <= 1'b1;
				else st <= S_RUN;
			end
			if (st == S_HALT) st <= S_RUN;
		end

		//------------------------------------------------------------------
		// CINV/CPUSH of the instruction cache
		//------------------------------------------------------------------
		if ((ic_inv || sn_inv) && !inv_busy && !inv_hold && st != S_FILL && !c_we) begin
			lb_v <= 1'b0;
			inv_sn    <= !ic_inv;
			inv_scope <= ic_inv ? ic_inv_scope : 2'd1;
			inv_pa    <= ic_inv ? ic_inv_pa : sn_inv_pa;
			if (ic_inv ? (ic_inv_scope == 2'd3) : sn_inv_all) begin
				for (int i = 0; i < 64; i++) iv[i] <= 4'd0;
				ic_inv_done <= ic_inv;
				sn_inv_done <= !ic_inv;
				inv_hold    <= 1'b1;
			end
			else begin
				inv_busy <= 1'b1;
				inv_ph   <= 1'b0;
				inv_i    <= (!ic_inv || ic_inv_scope == 2'd1) ?
				            {1'b0, (ic_inv ? ic_inv_pa[9:4] : sn_inv_pa[9:4])} : 7'd0;
			end
		end
		if (inv_busy) begin
			// this cycle reads set inv_i; the previous set's tags compare
			if (inv_ph) begin
				logic [5:0] s;
				s = (inv_scope == 2'd1) ? inv_pa[9:4] : 6'(inv_i - 7'd1);
				for (int w = 0; w < 4; w++) begin
					logic m;
					if (inv_scope == 2'd1)
						m = ct[w][21:0] == inv_pa[31:10];
					else if (tc_p)
						m = ct[w][21:3] == inv_pa[31:13];
					else
						m = ct[w][21:2] == inv_pa[31:12];
					if (m) iv[s][w] <= 1'b0;
				end
			end
			inv_ph <= 1'b1;
			if (inv_scope == 2'd1) begin
				if (inv_ph) begin
					inv_busy    <= 1'b0;
					ic_inv_done <= !inv_sn;
					sn_inv_done <= inv_sn;
					inv_hold    <= 1'b1;
				end
			end
			else begin
				inv_i <= inv_i + 7'd1;
				if (inv_i == 7'd64) begin
					inv_busy    <= 1'b0;
					ic_inv_done <= 1'b1;
					inv_hold    <= 1'b1;
				end
			end
		end
	end
end

endmodule
