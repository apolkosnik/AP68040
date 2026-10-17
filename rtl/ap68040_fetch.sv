//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                            //
//                                                                          //
// ap68040_fetch.sv - instruction fetch: I-ATC, instruction cache and the   //
// instruction queue                                                        //
//                                                                          //
// Instruction cache (MC68040UM section 4): 4 Kbytes, four ways of 64 sets //
// of 16-byte lines, physically tagged (PA31-10; the set is LA9-4), a      //
// valid bit per line, replacement by the first invalid way, else a 2-bit  //
// counter.  Lines are filled by a burst (critical long word first, TBI    //
// handled by the BIU, TCI: the line is not allocated).  Cache-inhibited   //
// pages and CACR.IE = 0 fetch long words.  The data RAM holds a long word //
// per address: F1 reads its long word of each way, a fill writes each    //
// beat into the victim way as it arrives; the victim is invalidated and   //
// retagged as the fill starts and validated when it completes (a TCI or  //
// failed fill leaves it invalid, 4.6.1).                                  //
//                                                                          //
// I-ATC (3.3): 64 entries like the D-ATC, translating supervisor and user //
// program space; ITT0/ITT1 transparent translation.  An ATC miss asks the //
// DMU's engine for a table walk; the walk installs the entry here         //
// (iatc_wr) -- a failed search too, as a nonresident entry (3.3) -- and   //
// the fetch looks again.                                                   //
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
// Branch target buffer: 256 entries (BTB_AW) looked up with the F0       //
// address; a hit (the long word holds the last word of a branch D1        //
// predicted taken) ends the chunk at that word, flags it, and sends F0 to //
// the target in F1 (one bubble): the entry's, or for a return the top of //
// the fetch's return stack (pushed by the calls it takes, resynchronized //
// from D1's on every redirect).  The flagged word's target travels in a  //
// FIFO; D1 checks every flagged word against its own decode.              //
//                                                                          //
// CINV/CPUSH of the instruction cache arrive from the DMU (ic_inv): all   //
// in one cycle, a line in two, a page by a scan of the 64 sets.  Bus     //
// snooping (sn_inv) invalidates a line the same way (MC68040UM table   //
// 4-3, V5/V6).                                                            //
//--------------------------------------------------------------------------//

module ap68040_fetch
	import ap68040_pkg::*, ap68040_upkg::*;
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
	input  logic        cdis,         // CDIS: the cache is disabled (contents kept)
	input  logic        mdis,         // MDIS: no page translation

	// decoder side
	output logic [15:0] win [8],
	output logic  [7:0] win_flt,      // the word came from a faulted fetch
	output logic  [7:0] win_fdem,     // ... the first of a redirected stream
	output logic  [7:0] win_fatc,     // ... an ATC (MMU) fault, else bus error
	output logic  [7:0] win_bt,       // ... ends a branch the BTB took
	output logic [31:0] bt_tgt,       // the target of the first such word
	output pd_t         pd0,          // predecode of win[0]
	output pd_t         pd0_n,        // ... as it will be next cycle
	output logic  [3:0] qcnt,         // words available in win (0..8)
	output logic [31:0] qpc,
	input  logic  [2:0] consume,
	output logic        q_odd,        // the stream starts at an odd address
	input  logic        btb_we,       // D1 writes a BTB entry
	input  logic [BTB_AW-1:0] btb_wi,
	input  logic        btb_wv,       // valid (0 drops the entry)
	input  logic [BTB_TW-1:0] btb_wtag,
	input  logic        btb_wslot,    // the branch ends at word 1 of the long word
	input  logic  [1:0] btb_wkind,    // 0 branch, 1 call, 2 return (target from the stack)
	input  logic [30:0] btb_wtgt,     // target[31:1]
	input  logic [30:0] d_ras,        // D1's return stack, its top entry: the
	input  logic  [2:0] d_ras_tp,     // ... fetch's pointer and top entry are
	input  logic  [3:0] d_ras_n,      // ... resynchronized on every redirect

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
	output logic        sn_ihit,       // ... and removed a valid line (or the line buffer)

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

localparam int QN = 8;            // (12: about 2% faster, 100 ALMs more)

wire tc_e  = tc[15] && !mdis;
wire tc_p  = tc[14];
wire ic_en = cacr[15] && !cdis;

function automatic logic ttr_hit(input logic [31:0] t, input logic [31:0] a,
                                 input logic s);
	ttr_hit = t[15] && (((a[31:24] ^ t[31:24]) & ~t[23:16]) == 8'd0) &&
	          (t[14] || (t[13] == s));
endfunction

//--------------------------------------------------------------------------
// queue
//--------------------------------------------------------------------------
logic [15:0]   qw [QN];
// the words' predecode: D1 reads only the head's, so it is a ring (no
// shift) with the head in a register (qp0) -- loaded as D1 consumes from
// the ring, or from F3 when the queue empties into it
// (two LUT RAM banks by slot parity: F3's two words land in consecutive
// slots, one per bank, so each bank has one write a cycle)
(* ramstyle = "MLAB, no_rw_check" *) logic [$bits(pd_t)-1:0] qpr_e [8];
(* ramstyle = "MLAB, no_rw_check" *) logic [$bits(pd_t)-1:0] qpr_o [8];
logic  [3:0]   qh;             // the ring's head
pd_t           qp0;
// synthesis translate_off
pd_t           qps [QN];       // (simulation: the shifting model, checked)
// synthesis translate_on
logic [QN-1:0] qf, qd, qa;     // faulted, demand fetch, ATC fault
logic [QN-1:0] qb;             // ends a branch the BTB took
logic [30:0]   tq [4];         // their targets, in queue order
logic  [2:0]   tq_n;
logic  [4:0]   cnt;
logic [31:0]   qpc_r;
logic          odd;

assign qcnt  = (cnt > 5'd8) ? 4'd8 : cnt[3:0];
assign qpc   = qpc_r;
assign q_odd = odd;
assign pd0   = qp0;
// the head's predecode as it will be next cycle (qp0's load): D1's
// attribute ROM reads it a cycle ahead
wire [3:0] qh_n = qwrap(5'(qh) + {2'b00, consume});        // the head next cycle
pd_t       qpr_rd;
assign qpr_rd = pd_t'(qh_n[0] ? qpr_o[qh_n[3:1]] : qpr_e[qh_n[3:1]]);
assign pd0_n = (cnt == {2'b00, consume}) ? f3_p[0] : qpr_rd;
function automatic logic [3:0] qwrap(input logic [4:0] x);   // x mod QN, x < 2*QN
	qwrap = (x >= 5'(QN)) ? 4'(x - 5'(QN)) : x[3:0];
endfunction

always_comb begin
	for (int i = 0; i < 8; i++) begin
		win[i]      = qw[i];
		win_flt[i]  = qf[i];
		win_fdem[i] = qd[i];
		win_fatc[i] = qa[i];
		win_bt[i]   = qb[i];
	end
end
assign bt_tgt = {tq[0], 1'b0};

// F2: the words of a chunk, their table words read (ap68040_pdrom, the
// words a cycle after their addresses); F3: predecoded from the table
// words and appended this cycle
logic  [1:0]  f2_n;
logic [15:0]  f2_w [2];
logic         f2_f, f2_d, f2_a;
logic  [1:0]  f2_b;            // the word ends a BTB-taken branch
logic [30:0]  f2_t;
logic  [1:0]  f3_n;
logic [15:0]  f3_w [2];
pdw_t         f3_r [2];        // the table's words for f3_w
pd_t          f3_p [2];
logic         f3_f, f3_d, f3_a;
logic  [1:0]  f3_b;
logic [30:0]  f3_t;
ap68040_pdrom pdrom (.clk(clk), .a0(f2_w[0][15:3]), .a1(f2_w[1][15:3]), .q0(f3_r[0]), .q1(f3_r[1]));
genvar gp;
generate
	for (gp = 0; gp < 2; gp++) begin : g_pd
		ap68040_predec pd (.op(f3_w[gp]), .w(f3_r[gp]), .pd(f3_p[gp]));
		// synthesis translate_off
		// (the table form against the parallel pattern decoder)
		pd_t pd_ref;
		ap68040_predec_pla pdr (.op(f3_w[gp]), .pd(pd_ref));
		always_ff @(posedge clk)
			if (nreset && f3_n > 2'(gp) && f3_p[gp] !== pd_ref) begin
				$display("FAIL: fetch: the table predecode of %04x is %h, the pattern decoder's %h", f3_w[gp], f3_p[gp], pd_ref);
				$finish;
			end
		// synthesis translate_on
	end
endgenerate

//--------------------------------------------------------------------------
// I-ATC
//--------------------------------------------------------------------------
logic [31:0] fpc;              // F0 fetch address (A0 clear)

// BTB: one array per word of the long word (a branch ending at word 0,
// one ending at word 1), each valid, tag, kind, target[31:1]: two taken
// branches in one long word (BSR then RTS, a short branch then RTS) no
// longer evict each other
localparam int BTB_DW = 1 + BTB_TW + 2 + 31;
logic [31:0] f1_pc;              // F1's address
// (block RAM: F0 presents fpc, F1 compares with f1_pc; an entry D1 writes
// at the edge F0's read registers it is not trusted -- no prediction)
logic [BTB_DW-1:0] btb_q0, btb_q1, btb_q;
logic        btb_cf;
ap68040_sdpr #(.AW(BTB_AW), .DW(BTB_DW)) btb0 (
	.clk(clk), .we(btb_we && !btb_wslot), .waddr(btb_wi),
	.wdata({btb_wv, btb_wtag, btb_wkind, btb_wtgt}),
	.raddr(fpc[BTB_AW+1:2]), .q(btb_q0)
);
ap68040_sdpr #(.AW(BTB_AW), .DW(BTB_DW)) btb1 (
	.clk(clk), .we(btb_we && btb_wslot), .waddr(btb_wi),
	.wdata({btb_wv, btb_wtag, btb_wkind, btb_wtgt}),
	.raddr(fpc[BTB_AW+1:2]), .q(btb_q1)
);
always_ff @(posedge clk) btb_cf <= btb_we && (btb_wi == fpc[BTB_AW+1:2]);
// the first branch of the fetch: word 0's when the fetch starts there
wire btb_hit0 = !btb_cf && btb_q0[BTB_DW-1] && btb_q0[BTB_DW-2 -: BTB_TW] == f1_pc[31:BTB_AW+2] && !f1_pc[1];
wire btb_hit1 = !btb_cf && btb_q1[BTB_DW-1] && btb_q1[BTB_DW-2 -: BTB_TW] == f1_pc[31:BTB_AW+2];
wire btb_hit  = btb_hit0 || btb_hit1;
wire btb_qslot = !btb_hit0;
assign btb_q   = btb_hit0 ? btb_q0 : btb_q1;

// the fetch's return stack: pushed by calls the BTB takes, popped by the
// returns it takes (their target), resynchronized from D1's on redirects
// (a LUT RAM: one write a cycle -- a call's push, or the top entry on a
// redirect -- and the top read; the entries below the top are the fetch's
// own: only predictions, EX checks every return)
(* ramstyle = "MLAB, no_rw_check" *) logic [30:0] fras [8];
logic  [2:0] fras_tp;
logic  [3:0] fras_n;
logic        atc_hit;
atce_t       atc_e;
ap68040_atc #(.SAME_SEARCH(0)) iatc (
	.clk(clk), .nreset(nreset), .p8k(tc_p),
	.la(iatc_flush_page ? iatc_flush_la : fpc),
	.fc2(iatc_flush_page ? iatc_flush_fc2 : smode), .set_m(4'd0),
	.hit(atc_hit), .ent(atc_e), .hv(), .wpa(),
	.la_d(32'd0), .fc2_d(1'b0), .set_d(4'd0), .hv_d(), .wpa_d(),
	.wr(iatc_wr), .wla(iatc_wla), .wfc2(iatc_wfc2), .went(iatc_went),
	.wsame(1'b0), .wsame_hv(4'd0),
	.flush_all(iatc_flush_all), .flush_page(iatc_flush_page),
	.flush_nonglobal(iatc_flush_ng)
);

//--------------------------------------------------------------------------
// cache arrays: tags and data in block RAM (read address registered in
// F0), valid bits in flip-flops
//--------------------------------------------------------------------------
// line valid, per set and way: a word of the four ways per set in LUT
// RAMs written at the edge and read without a clock -- one copy at F1's
// set, one at the fill's set or the invalidation's (they never run at
// once).  The second is read at the set every write goes to, so it forms
// the word written (the ways not written keep their bits).  Clearing every
// line (CINVA, a snooped clear, reset) runs the invalidation over the sets
(* ramstyle = "MLAB, no_rw_check" *) logic [3:0] ivf [64];
(* ramstyle = "MLAB, no_rw_check" *) logic [3:0] ivo [64];
wire   [3:0]   ivf_q, ivo_q;
logic  [5:0]   ivo_ra;
logic          inv_rst;         // the invalidation is reset's clear
// synthesis translate_off
logic  [3:0]   iv [64];          // (simulation: the flip-flop model, checked)
// synthesis translate_off
initial for (int i = 0; i < 64; i++) iv[i] = 4'd0;
// synthesis translate_on
// synthesis translate_on
logic  [1:0]   rr;
logic  [5:0]   c_raddr;
logic          c_we;
logic  [5:0]   c_waddr;
logic  [1:0]   c_wway;
logic [21:0]   c_wtag;
logic [31:0]   cd [4];        // each way's long word at F1's address
logic [23:0]   ct [4];

logic [31:0] m_cw;               // the fill's first long word (the one fetched)
logic  [3:0] m_got;              // ... the long words of it that arrived
logic        m_vclr;             // the fill's victim line is still to be invalidated
logic        m_rrinc;            // ... the set was full: the replacement counter moves
// The data RAMs hold a long word per address ({set, A3:A2}): F1 reads its
// long word of each way directly, and a fill writes each beat into the
// victim way as it arrives.  The victim is invalidated and given its new
// tag as the fill starts, and validated when the line is complete (a
// failed or TCI fill leaves it invalid, MC68040UM 4.6.1): the line is in
// the cache for the first fetch after it -- no line buffer.  The long word
// being fetched (the burst's first) is kept for a fill that is not
// allocated (TCI, a later beat's bus error), which delivers it alone.
wire         c_dwe = (st == S_FILL) && b_rvalid && m_alloc && !m_vclr;
genvar gw;
generate
	for (gw = 0; gw < 4; gw++) begin : g_way
		ap68040_sdp_be #(.AW(8), .NB(4), .OREG(0)) dram (
			.clk(clk), .we(c_dwe && m_way == 2'(gw)), .waddr({m_pa[9:4], b_rbeat}),
			.wbe(4'hF), .wdata(b_rdata), .raddr(fpc[9:2]), .oce(1'b1), .q(cd[gw])
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
logic        f1_btok;           // F0 had room for a taken branch's target
wire         f1_bt    = f1_btok && btb_hit;   // the BTB hit (in F1)
wire         f1_bslot = btb_qslot;            // ... where the branch ends
wire   [1:0] f1_bkind = btb_q[32:31];
wire  [30:0] f1_btgt  = btb_q[30:0];
logic        f1_s, f1_dem;
logic        f1_hit;
atce_t       f1_e;

logic        m_tci;            // the fill in progress was answered with TCI

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
logic        inv_lbh;          // ... and the line buffer held its line
logic  [1:0] inv_scope;
logic [31:0] inv_pa;

wire        redir_any = redir_v || d_redir_v;
wire [31:0] redir_npc = redir_v ? redir_pc : d_redir_pc;

//--------------------------------------------------------------------------
// F1 evaluation
//--------------------------------------------------------------------------
logic        x_ok, x_flt, x_walk, x_cach, x_hit;
logic [31:0] x_pa;
logic  [1:0] x_cm, x_upa, x_way;
logic [31:0] x_chunk;
logic  [3:0] x_mj;
// the ITT matches are formed in F0 (the TTRs change only by MOVEC, which
// serializes and refetches), so F1 starts from them
logic        f1_tt0, f1_tt1;
// ... and whether translation is on (TCR.E, MDIS), taken with them: F1's
// translation and its tag compare select both start from flip-flops
logic        f1_tce;
always_comb begin
	logic tt0, tt1, use_t;
	logic [21:0] tg;
	x_ok = 1'b0; x_flt = 1'b0; x_walk = 1'b0;
	x_pa = f1_pc; x_cm = 2'b00; x_upa = 2'b00;
	tt0 = f1_tt0;
	tt1 = f1_tt1;
	if (tt0 || tt1) begin
		x_ok  = 1'b1;
		x_cm  = tt0 ? itt0[6:5] : itt1[6:5];
		x_upa = tt0 ? itt0[9:8] : itt1[9:8];
	end
	else if (!f1_tce)
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
	// the tags compare with both candidate addresses, the logical one (a
	// TTR or no translation) and the ATC's, and the translation selects
	// the outcome: it stays off the front of the compare (the ATC's
	// address matters only with a hit, x_cach is clear without one)
	use_t  = !(tt0 || tt1) && f1_tce;
	tg     = {f1_e.pa[19:1], tc_p ? f1_pc[12] : f1_e.pa[0], f1_pc[11:10]};
	x_hit  = 1'b0;
	x_way  = 2'd0;
	for (int i = 3; i >= 0; i--) begin
		x_mj[i] = ivf_q[i] && (use_t ? (ct[i][21:0] == tg) : (ct[i][21:0] == f1_pc[31:10]));
		if (x_mj[i]) begin x_hit = 1'b1; x_way = 2'(i); end
	end
	x_hit  = x_hit && x_cach;
	// (the chunk by the one-hot match, AND-OR: no encoded way on the path;
	// at most one way holds a line)
	x_chunk = '0;
	for (int i = 0; i < 4; i++) if (x_mj[i]) x_chunk = x_chunk | cd[i];
end
wire f1_go      = f1_v && !redir_any;
wire f1_deliver = f1_go && x_hit;

//--------------------------------------------------------------------------
// F0 issue: room for this chunk and those in flight
//--------------------------------------------------------------------------
wire [5:0] inflight = (f1_v ? 6'd2 : 6'd0) + {4'd0, f2_n} + {4'd0, f3_n};
// the target FIFO cannot overflow: count the flagged words on their way
wire       bt_room  = ({1'b0, tq_n} + {3'd0, f1_v && f1_bt} + {3'd0, |f2_b} + {3'd0, |f3_b}) < 4'd4;
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
	logic        frw;              // the return stack's write (below)
	logic  [2:0] frwa;
	logic [30:0] frwd;
	logic  [5:0] iw_set;           // the valid RAMs' write (below)
	logic  [3:0] iw_m;
	logic        iw_v;
	frw = 1'b0; frwa = '0; frwd = '0;
	iw_set = '0; iw_m = '0; iw_v = 1'b0;
	if (!nreset) begin
		cnt   <= '0;
		qf    <= '0; qd <= '0; qa <= '0;
		qpc_r <= '0;
		odd   <= 1'b0;
		fpc   <= '0;
		fnew  <= 1'b1;
		st    <= S_RUN;
		f1_v  <= 1'b0; f1_pc <= '0; f1_s <= 1'b0; f1_dem <= 1'b0; f1_hit <= 1'b0; f1_e <= '0;
		f1_tt0 <= 1'b0; f1_tt1 <= 1'b0; f1_tce <= 1'b0;
		f1_btok <= 1'b0;
		fras_tp <= '0; fras_n <= '0;
		f2_b  <= '0; f2_t <= '0; f3_b <= '0; f3_t <= '0;
		qb    <= '0; tq_n <= '0;
		for (int i = 0; i < 4; i++) tq[i] <= '0;
		f2_n  <= '0; f2_f <= 1'b0; f2_d <= 1'b0; f2_a <= 1'b0;
		for (int i = 0; i < 2; i++) f2_w[i] <= '0;
		f3_n  <= '0; f3_f <= 1'b0; f3_d <= 1'b0; f3_a <= 1'b0;
		for (int i = 0; i < 2; i++) f3_w[i] <= '0;
		m_tci <= 1'b0;
		busy  <= 1'b0; stale <= 1'b0; b_req_r <= 1'b0;
		m_la  <= '0; m_pa <= '0; m_upa <= '0; m_s <= 1'b0; m_dem <= 1'b0; m_ci <= 1'b0;
		m_alloc <= 1'b0; m_way <= '0; m_cw <= '0; m_vclr <= 1'b0; m_rrinc <= 1'b0;
		iw_req <= 1'b0;
		c_we  <= 1'b0; c_waddr <= '0; c_wway <= '0; c_wtag <= '0;
		rr    <= '0;
		// (reset leaves the cache alone, 5.3: its valid bits have their
		// power-up values from their initial blocks)
		inv_busy <= 1'b0; inv_hold <= 1'b0; inv_ph <= 1'b0; inv_i <= '0;
		inv_sn <= 1'b0; inv_scope <= 2'd3; inv_pa <= '0; inv_lbh <= 1'b0;
		inv_rst <= 1'b0;
		sn_ihit <= 1'b0;
		ic_inv_done <= 1'b0;
		sn_inv_done <= 1'b0;
		for (int i = 0; i < QN; i++) qw[i] <= '0;
		qh  <= '0;
		qp0 <= '0;
	end
	else begin
		logic [4:0] keep;
		c_we        <= 1'b0;
		ic_inv_done <= 1'b0;
		sn_inv_done <= 1'b0;
		sn_ihit     <= 1'b0;
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
				// synthesis translate_off
				qps[i] <= qps[src[4:0]];
				// synthesis translate_on
				qw[i] <= qw[src[4:0]];
				qf[i] <= qf[src[4:0]];
				qd[i] <= qd[src[4:0]];
				qa[i] <= qa[src[4:0]];
				qb[i] <= qb[src[4:0]];
			end
			else if (k < {3'b000, f3_n}) begin
				// synthesis translate_off
				qps[i] <= f3_p[k[0]];
				// synthesis translate_on
				qw[i] <= f3_w[k[0]];
				qf[i] <= f3_f;
				qd[i] <= f3_d;
				qa[i] <= f3_a;
				qb[i] <= f3_b[k[0]];
			end
		end
		cnt   <= keep + {3'b000, f3_n};
		// the predecode ring: appended at head + count (wherever D1's
		// consume puts the head), the head moved by consume
		begin
			// (one write statement per bank, the word by the slot's parity:
			// two statements read as two write ports and keep it in logic)
			logic [3:0] w0, w1;
			logic       we_e, we_o;
			w0 = qwrap(5'(qh) + cnt);
			w1 = qwrap(5'(qh) + cnt + 5'd1);
			we_e = (f3_n != 2'd0 && !w0[0]) || (f3_n == 2'd2 && !w1[0]);
			we_o = (f3_n != 2'd0 &&  w0[0]) || (f3_n == 2'd2 &&  w1[0]);
			if (we_e) qpr_e[w0[0] ? w1[3:1] : w0[3:1]] <= w0[0] ? f3_p[1] : f3_p[0];
			if (we_o) qpr_o[w0[0] ? w0[3:1] : w1[3:1]] <= w0[0] ? f3_p[0] : f3_p[1];
			qh  <= qh_n;
			qp0 <= pd0_n;
		end
		// D1 consumed the end of a BTB-taken branch: the stream continues
		// at its target (the words after it in the queue came from there)
		begin
			logic        cfl;
			logic  [2:0] cj;
			logic [30:0] tqn [4];
			logic  [2:0] n;
			cfl = 1'b0; cj = 3'd0;
			for (int i = 6; i >= 0; i--)
				if (3'(i) < consume && qb[i]) begin cfl = 1'b1; cj = 3'(i); end
			// (a decode that ran past the flag is refetched at WB: the PC
			// of what follows it does not matter)
			qpc_r <= cfl ? {tq[0], 1'b0} : qpc_r + {28'd0, consume, 1'b0};
			for (int i = 0; i < 4; i++) tqn[i] = tq[i];
			n = tq_n;
			if (cfl && n != 3'd0) begin
				for (int i = 0; i < 3; i++) tqn[i] = tqn[i + 1];
				n = n - 3'd1;
			end
			if (f3_n != 2'd0 && f3_b != 2'b00 && n != 3'd4) begin
				tqn[n[1:0]] = f3_t;
				n = n + 3'd1;
			end
			for (int i = 0; i < 4; i++) tq[i] <= tqn[i];
			tq_n <= n;
		end
		f3_n  <= f2_n;
		f3_f  <= f2_f; f3_d <= f2_d; f3_a <= f2_a;
		f3_b  <= f2_b; f3_t <= f2_t;
		for (int i = 0; i < 2; i++) f3_w[i] <= f2_w[i];
		f2_n  <= '0;
		f2_b  <= '0;

		//------------------------------------------------------------------
		// F0 -> F1
		//------------------------------------------------------------------
		f1_v  <= 1'b0;
		f1_btok <= 1'b0;
		if (f0_go) begin
			f1_v   <= 1'b1;
			f1_btok <= bt_room;
			f1_pc  <= fpc;
			f1_s   <= smode;
			f1_tt0 <= ttr_hit(itt0, fpc, smode);
			f1_tt1 <= ttr_hit(itt1, fpc, smode);
			f1_tce <= tc_e;
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
			if (f1_bt && !(f1_bkind == 2'd2 && fras_n == 4'd0)) begin
				// a predicted taken branch ends in this chunk: the words up
				// to its last, that one flagged; F0 goes to the target (a
				// return's from the stack) and its sequential fetch of this
				// cycle is dropped; a call pushes its return address
				logic [30:0] t;
				t = (f1_bkind == 2'd2) ? fras[fras_tp] : f1_btgt;
				f2_n <= (f1_pc[1] || !f1_bslot) ? 2'd1 : 2'd2;
				f2_b <= (f1_pc[1] || !f1_bslot) ? 2'b01 : 2'b10;
				f2_t <= t;
				f1_v <= 1'b0;
				f1_btok <= 1'b0;
				fpc  <= {t, 1'b0};
				fnew <= 1'b0;
				if (f1_bkind == 2'd1) begin
					frw  = 1'b1;
					frwa = fras_tp + 3'd1;
					frwd = {f1_pc[31:2], f1_bslot} + 31'd1;
					fras_tp <= fras_tp + 3'd1;
					if (fras_n != 4'd8) fras_n <= fras_n + 4'd1;
				end
				else if (f1_bkind == 2'd2) begin
					fras_tp <= fras_tp - 3'd1;
					fras_n  <= fras_n - 4'd1;
				end
			end
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
				vv = ivf_q;
				st      <= S_FILL;
				m_got   <= 4'd0;
				m_alloc <= 1'b1;
				m_tci   <= 1'b0;
				m_way   <= !vv[0] ? 2'd0 : !vv[1] ? 2'd1 : !vv[2] ? 2'd2 : !vv[3] ? 2'd3 : rr;
				m_vclr  <= 1'b1;
				m_rrinc <= (vv == 4'hF);
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
			logic [31:0] lw0;
			// the long word being fetched: arriving now, or kept
			lw0 = (b_rvalid && b_rbeat == m_pa[3:2]) ? b_rdata : m_cw;
			// (the fill's first cycle: no beat has arrived yet) the victim
			// invalid, its tag the new line's
			if (m_vclr && !inv_busy) begin
				m_vclr  <= 1'b0;
				iw_set = m_pa[9:4]; iw_m[m_way] = 1'b1; iw_v = 1'b0;
				c_we    <= 1'b1;
				c_waddr <= m_pa[9:4];
				c_wway  <= m_way;
				c_wtag  <= m_pa[31:10];
			end
			// (a beat before the victim is invalid is not written: the
			// line is not allocated then)
			if (b_rvalid && m_vclr) m_alloc <= 1'b0;
			if (b_rvalid) begin
				if (b_rbeat == m_pa[3:2]) m_cw <= b_rdata;
				m_got[b_rbeat] <= 1'b1;
				if (b_rtci) begin m_alloc <= 1'b0; m_tci <= 1'b1; end
			end
			if (busy && b_done) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (m_alloc && !(b_rvalid && b_rtci)) begin
					iw_set = m_pa[9:4]; iw_m[m_way] = 1'b1; iw_v = 1'b1;
					if (m_rrinc) rr <= rr + 2'd1;
				end
				st    <= S_RUN;
				if (m_tci || (b_rvalid && b_rtci)) begin
					// a TCI line is not kept (5.7.1: sequential access only
					// until the burst's last long word arrives): the long word
					// being fetched goes straight to the decoder, the next one
					// is fetched anew (unless a redirect takes the stream)
					if (!stale && !redir_any) begin
						logic [31:0] lw, nla;
						lw  = lw0;
						nla = {m_la[31:2], 2'b00} + 32'd4;
						if (m_la[1]) begin
							f2_n    <= 2'd1;
							f2_w[0] <= lw[15:0];
						end
						else begin
							f2_n    <= 2'd2;
							f2_w[0] <= lw[31:16];
							f2_w[1] <= lw[15:0];
						end
						f2_f  <= 1'b0; f2_d <= 1'b0; f2_a <= 1'b0;
						m_la  <= nla;
						m_pa  <= {m_pa[31:2], 2'b00} + 32'd4;
						m_dem <= 1'b0;
						fpc   <= nla;
						fnew  <= 1'b0;
					end
				end
			end
			if (busy && b_err) begin
				busy  <= 1'b0;
				stale <= 1'b0;
				if (stale) st <= S_RUN;
				else if (m_got[m_pa[3:2]]) begin
					// a later beat failed, the long word being fetched had
					// arrived (the burst starts with it): it is used, the line
					// is not allocated, and no exception (MC68060UM 7.9.1,
					// 5.7.1).  The next long word is fetched anew.
					logic [31:0] lw, nla;
					lw  = lw0;
					nla = {m_la[31:2], 2'b00} + 32'd4;
					if (m_la[1]) begin
						f2_n    <= 2'd1;
						f2_w[0] <= lw[15:0];
					end
					else begin
						f2_n    <= 2'd2;
						f2_w[0] <= lw[31:16];
						f2_w[1] <= lw[15:0];
					end
					f2_f  <= 1'b0; f2_d <= 1'b0; f2_a <= 1'b0;
					m_la  <= nla;
					m_pa  <= {m_pa[31:2], 2'b00} + 32'd4;
					m_dem <= 1'b0;
					fpc   <= nla;
					fnew  <= 1'b0;
					st    <= S_RUN;
				end
				else begin
					f2_n <= m_la[1] ? 2'd1 : 2'd2;
					for (int i = 0; i < 2; i++) f2_w[i] <= 16'd0;
					f2_f <= 1'b1; f2_d <= m_dem; f2_a <= 1'b0;
					st   <= S_HALT;
				end
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
			else if (!busy && !b_req_r && !b_gnt && !redir_any && !stop &&
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
			f2_b  <= '0;
			f3_b  <= '0;
			tq_n  <= '0;
			f1_v  <= 1'b0;
			f1_btok <= 1'b0;
			frw  = 1'b1;
			frwa = d_ras_tp;
			frwd = d_ras;
			fras_tp <= d_ras_tp;
			fras_n  <= d_ras_n;
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
			inv_sn    <= !ic_inv;
			inv_scope <= ic_inv ? ic_inv_scope : 2'd1;
			inv_pa    <= ic_inv ? ic_inv_pa : sn_inv_pa;
			inv_lbh   <= 1'b0;
			if (ic_inv ? (ic_inv_scope == 2'd3) : sn_inv_all) begin
				// every line: the sweep below (scope 3 matches every way)
				inv_scope <= 2'd3;
				sn_ihit   <= !ic_inv;
				inv_busy  <= 1'b1;
				inv_ph    <= 1'b0;
				inv_i     <= 7'd0;
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
					if (inv_scope == 2'd3)
						m = 1'b1;
					else if (inv_scope == 2'd1)
						m = ct[w][21:0] == inv_pa[31:10];
					else if (tc_p)
						m = ct[w][21:3] == inv_pa[31:13];
					else
						m = ct[w][21:2] == inv_pa[31:12];
					if (m) begin iw_set = s; iw_m[w] = 1'b1; iw_v = 1'b0; end
					if (m && ivo_q[w] && inv_sn) sn_ihit <= 1'b1;   // (ivo_ra: s)
				end
				if (inv_sn && inv_lbh) sn_ihit <= 1'b1;
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
					ic_inv_done <= !inv_sn && !inv_rst;
					sn_inv_done <= inv_sn;
					inv_hold    <= 1'b1;
					inv_rst     <= 1'b0;
				end
			end
		end
	end
	// the return stack's one write port
	if (nreset && frw) fras[frwa] <= frwd;
	// the valid RAMs' one write (both copies)
	if (nreset) begin
		if (iw_m != 4'd0) begin
			ivf[iw_set] <= (ivo_q & ~iw_m) | (iw_m & {4{iw_v}});
			ivo[iw_set] <= (ivo_q & ~iw_m) | (iw_m & {4{iw_v}});
		end
		// synthesis translate_off
		for (int j = 0; j < 4; j++) if (iw_m[j]) iv[iw_set][j] <= iw_v;
		if (iw_m != 4'd0 && iw_set != ivo_ra) begin
			$display("FAIL: fetch: a valid write at set %0d, the second copy read at %0d", iw_set, ivo_ra);
			$finish;
		end
		// synthesis translate_on
	end
end

// the valid copies' reads: F1's set; the invalidation's, else the fill's
wire [5:0] inv_s = (inv_scope == 2'd1) ? inv_pa[9:4] : 6'(inv_i - 7'd1);
always_comb ivo_ra = inv_busy ? inv_s : m_pa[9:4];
assign ivf_q = ivf[f1_pc[9:4]];
assign ivo_q = ivo[ivo_ra];
initial for (int i = 0; i < 64; i++) begin ivf[i] = 4'd0; ivo[i] = 4'd0; end
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && !inv_rst && (ivf_q != iv[f1_pc[9:4]] || ivo_q != iv[ivo_ra])) begin
		$display("FAIL: fetch: valid RAM %b/%b, model %b/%b", ivf_q, ivo_q, iv[f1_pc[9:4]], iv[ivo_ra]);
		$finish;
	end
// synthesis translate_on

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && cnt != 5'd0 && qp0 != qps[0]) begin
		$display("FAIL: fetch: the predecode ring's head %h, the queue's %h", qp0, qps[0]);
		$finish;
	end
// synthesis translate_on

endmodule
