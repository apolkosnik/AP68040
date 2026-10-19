//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                            //
//                                                                          //
// ap68040_dmu.sv - data memory unit: D-ATC, data cache, table walker       //
//                                                                          //
// Mirrors the back end's DC1, DC2, EX and WB stages for memory uops.       //
//                                                                          //
// Data cache (MC68040UM section 4): 4 Kbytes, four ways of 64 sets of     //
// 16-byte lines, physically tagged (PA31-10; the set is PA9-4 = LA9-4).    //
// A valid and a dirty bit per line; replacement takes the first invalid   //
// way, else a 2-bit counter.  Copyback and write-through cachable,        //
// serialized and nonserialized cache-inhibited modes.  Read-modify-write  //
// (locked) and cache-inhibited accesses bypass the cache: a line they hit //
// is pushed if dirty and invalidated first (4.3.2, 4.6.2, 7.4.1).         //
//                                                                          //
// DC1  D-ATC and TTR lookup, tag compare of the translated page           //
// DC2  fast path: a load (or unlocked read-modify-write) that hits a      //
//      cachable line, within one line, with no older store to the line    //
//      in flight, is answered from the data RAM's registered output.      //
//      Everything else stalls DC2 and runs in the engine.                 //
// WB   a copyback store that hit writes the data RAM in its WB cycle;     //
//      other stores go through the engine.                                 //
//                                                                          //
// The engine runs one job at a time: the DC2 slow path (table walk, line  //
// fill -- the dirty victim pushed afterwards --, cache-inhibited accesses //
// with a matching line pushed and invalidated first, operands crossing a  //
// line in two parts), WB stores, WB maintenance (CINV, CPUSH, PFLUSH,     //
// PTEST) and the instruction side's table walks.  While it uses the       //
// lookup ports it holds DC1 (dm_hold1), one cycle longer than it uses     //
// them so DC1 re-reads its own address.                                   //
//                                                                          //
// Bus snooping (MC68040UM 4.4, 4.7.2, 7.9) through the sn_* port: the    //
// snooper (ap68040_snoop.sv) asks for the cache; the DMU freezes the      //
// engine, holds DC1 and blocks the WB fast store, looks the physical      //
// line up in the tags, the push buffer and a push waiting for the bus,    //
// and applies the snooper's invalidate or sink write to what it found.    //
// An invalidated line also loses its hit in the DC2/EX/WB records and     //
// the engine's current part, so no store lands in a line that left.      //
// One cycle after the port is released re-reads the lookup address of    //
// DC1 or the engine before they run again.                                //
//                                                                          //
// Table walks (MC68040UM 3.2): root, pointer and page descriptors with    //
// indirect page descriptors, 4K/8K pages, accumulated W, U/M history per  //
// table 3-1 (locked read-modify-write where the table says so: the        //
// descriptor is read again with LOCK and written back with LOCK and       //
// LOCKE).  Descriptor reads see the data cache (cachable write-through,  //
// no allocate, 3.2.5); descriptor updates are noncachable and push and    //
// invalidate a matching line.  Like the 68040 (WinUAE cpummu.cpp), a      //
// failed search still creates an ATC entry, with R clear (B set for a bus //
// error): the access that looks the ATC up again takes the fault.         //
//--------------------------------------------------------------------------//

module ap68040_dmu
	import ap68040_pkg::*;
#(
	// DC1's own ATC read port and tag compare beside xlate (65 MHz timing;
	// 0: DC1's hit and way are xlate's, the port is not built)
	parameter bit DC1_ATC = 0
)
(
	input  logic        clk,
	input  logic        nreset,

	// back end
	input  logic        adv_ag,
	input  logic        dm_req,
	input  logic [31:0] dm_va,
	input  logic  [1:0] dm_mem,
	input  logic  [1:0] dm_msz,
	input  logic  [2:0] dm_fc,
	input  logic        dm_lock,
	input  logic        dm_locke,
	input  logic        dm_super,
	input  logic        dm_noalloc,
	input  logic  [1:0] dm_ack,         // acknowledge cycle: 1 IACK, 2 BKPT
	input  logic  [2:0] iack_lvl,
	input  logic        dm_older,       // uops older than DC2 still in EX or WB
	input  logic        adv_dc1,
	input  logic        adv_dc2,
	input  logic        adv_ex,
	input  logic        adv_wb,
	input  logic        kill_now,
	output logic        dm_hold1,       // DC1 must hold (the engine owns its ports)
	output logic        dc2_rdy,
	output logic [31:0] ldata,
	output logic [31:0] ldata_al,       // the aligned long (or word, low half)
	output logic        ldata_alv,      // ... is the load's data
	output logic        fault,
	output logic  [7:0] fvec,
	output logic [31:0] faddr,
	output logic [15:0] fssw,
	input  logic        st_v,
	input  logic [31:0] st_data,
	output logic        st_rdy,
	output logic        st_fault,
	output logic [15:0] st_fssw,
	output logic [31:0] st_faddr,

	// WB maintenance: CINV/CPUSH/PFLUSH/PTEST
	input  logic        mt_v,
	input  logic  [2:0] mt_op,          // MT_*
	input  logic  [1:0] mt_scope,       // 1 line, 2 page, 3 all
	input  logic  [1:0] mt_caches,      // bit 0 DC, bit 1 IC
	input  logic [31:0] mt_addr,
	input  logic  [2:0] mt_fc,          // DFC (PTEST/PFLUSH)
	input  logic        mt_ng,          // PFLUSHN/PFLUSHAN: spare global entries
	input  logic        mt_wr,          // PTESTW
	output logic        mt_done,
	output logic [31:0] mt_mmusr,       // PTEST: the MMUSR
	output logic        mt_fault,       // CPUSH: a push ended in TEA (an access error),
	output logic [15:0] mt_fssw,        // ... its SSW and
	output logic [31:0] mt_faddr,       // ... the push's address

	// control registers
	input  logic [31:0] cacr,
	input  logic [31:0] tc,
	input  logic [31:0] urp,
	input  logic [31:0] srp,
	input  logic [31:0] dtt0,
	input  logic [31:0] dtt1,
	input  logic [31:0] itt0,
	input  logic [31:0] itt1,
	input  logic        cdis,           // CDIS: caches disabled (contents kept)
	input  logic        mdis,           // MDIS: no page translation (TTRs still apply)
	output logic        tw_busy,        // a table search is running (PST)

	// instruction side: table walks for the I-ATC, ATC/cache maintenance
	input  logic        iw_req,
	input  logic [31:0] iw_va,
	input  logic        iw_fc2,
	output logic        iw_done,
	output atce_t       iw_ent,
	output logic        ic_inv,         // invalidate instruction cache lines
	output logic  [1:0] ic_inv_scope,   // 1 line, 2 page, 3 all
	output logic [31:0] ic_inv_pa,
	input  logic        ic_inv_done,
	output logic        iatc_flush_all,
	output logic        iatc_flush_page,
	output logic        iatc_flush_ng,
	output logic [31:0] iatc_flush_la,
	output logic        iatc_flush_fc2,
	output logic        iatc_wr,        // table walk result into the I-ATC
	output logic [31:0] iatc_wla,
	output logic        iatc_wfc2,
	output atce_t       iatc_went,

	// BIU client
	output logic        b_req,
	output busreq_t     b_breq,
	output logic [127:0] b_wdata,
	input  logic        b_gnt,
	input  logic        b_done,
	output logic        b_drop,         // the push the BIU holds for a retry: dropped
	input  logic        b_err,
	input  logic        b_rvalid,
	input  logic [31:0] b_rdata,
	input  logic  [1:0] b_rbeat,
	input  logic        b_ravec,
	input  logic        b_rtci,

	// bus snooper
	input  logic        sn_req,         // level: the snooper wants the cache
	input  logic [31:0] sn_pa,
	output logic        sn_look,        // pulse: sn_hit/sn_dirty/sn_line valid
	output logic        sn_hit,
	output logic        sn_dirty,
	output logic [127:0] sn_line,        // with sn_look: the line found
	input  logic        sn_inv,         // pulse: invalidate what was found
	input  logic        sn_wr,          // pulse: write into what was found
	input  logic [15:0] sn_wbe,         // the bytes, of...
	input  logic [31:0] sn_wword        // ...this long word on every long word
);

localparam logic [2:0] MT_CINV = 3'd1, MT_CPUSH = 3'd2, MT_PFLUSH = 3'd3, MT_PTEST = 3'd4;
// the SSW of a bus error on a line push (figure 8-6): a write, SIZE line,
// TT normal, TM 0 (push)
localparam logic [15:0] SSW_PUSH = 16'h0060;

// MDIS, sampled once here: DC1's registered translation decision (m1_tp)
// is formed from the pin a clock ahead, so the two always agree
logic mdis_q = 1'b0;
always_ff @(posedge clk) mdis_q <= mdis;
wire tc_e   = tc[15] && !mdis_q;
wire tc_e_n = tc[15] && !mdis;      // (tc_e at the next clock)
wire tc_p  = tc[14];
wire dc_en = cacr[31] && !cdis;

//--------------------------------------------------------------------------
// stage records
//--------------------------------------------------------------------------
typedef struct packed {
	logic        v;
	logic [31:0] a;        // logical address
	logic  [1:0] mem;
	logic  [1:0] msz;
	logic  [2:0] fc;
	logic        lock;
	logic        locke;
	logic        smode;    // FC2
	logic        noalloc;
	logic  [1:0] ack;      // acknowledge cycle (TT=3): 1 IACK, 2 BKPT
} req_t;

// translation and lookup result of one access part
typedef struct packed {
	logic        ok;       // translation known
	logic        flt;      // access fault (ATC/TTR)
	logic        walk;     // needs a table search (ATC miss or M update)
	logic [31:0] pa;
	logic  [1:0] cm;
	logic  [1:0] upa;
	logic        hit;      // the line is in the cache
	logic  [1:0] way;
} xres_t;

typedef struct packed {
	req_t        r;
	xres_t       x;        // part 0 (the only part unless split)
	logic        split;    // crosses a line: part 1 at the next line
	xres_t       x1;
	logic        fast;     // DC2 can answer from the RAM output
	logic        sfast;    // a load across a line whose first part hit:
	                       // DC2's copy holds it, one lookup for the rest
} mrec_t;

req_t  m1;
mrec_t m2, m3, m4;

function automatic logic [2:0] nbytes(input logic [1:0] msz);
	case (msz)
		SZ_B:    nbytes = 3'd1;
		SZ_W:    nbytes = 3'd2;
		SZ_L:    nbytes = 3'd4;
		default: nbytes = 3'd0;     // line
	endcase
endfunction

// MOVES to the instruction spaces is a data reference (MC68040UM table 3-2)
function automatic logic [1:0] fc_tt(input logic [2:0] fc);
	fc_tt = (fc == 3'd1 || fc == 3'd2 || fc == 3'd5 || fc == 3'd6) ? TT_NORMAL : TT_ALT;
endfunction
function automatic logic [2:0] fc_tm(input logic [2:0] fc);
	fc_tm = (fc == 3'd2) ? 3'd1 : (fc == 3'd6) ? 3'd5 : fc;
endfunction

// TTR match on A31-24 with mask, E, and the S field
function automatic logic ttr_hit(input logic [31:0] t, input logic [31:0] a,
                                 input logic s);
	ttr_hit = t[15] && (((a[31:24] ^ t[31:24]) & ~t[23:16]) == 8'd0) &&
	          (t[14] || (t[13] == s));
endfunction

// n bytes at offset o of a line, right aligned
function automatic logic [31:0] take(input logic [127:0] line, input logic [3:0] o,
                                     input logic [2:0] n);
	logic [31:0] v;
	v = '0;
	for (int i = 0; i < 4; i++)
		if (3'(i) < n) v = {v[23:0], line[127 - 8*(o + i) -: 8]};
	take = v;
endfunction

// A write of n bytes from the top of d (left aligned) at offset o of a
// line: the operand rotated onto the byte lanes, as the bus would carry
// it, is replicated over the line's four long words, and the byte enables
// pick the bytes (byte o+i is on lane (o+i) mod 4, which holds byte i).
function automatic logic [31:0] lanes32(input logic [31:0] d, input logic [1:0] o);
	case (o)
		2'd0:    lanes32 = d;
		2'd1:    lanes32 = {d[7:0], d[31:8]};
		2'd2:    lanes32 = {d[15:0], d[31:16]};
		default: lanes32 = {d[23:0], d[31:24]};
	endcase
endfunction
function automatic logic [15:0] bmask(input logic [3:0] o, input logic [2:0] n);
	logic [3:0] m;
	m = 4'b1111 << (3'd4 - n);
	bmask = {m, 12'd0} >> o;
endfunction

function automatic logic [3:0] dmask(input logic [15:0] be);
	dmask = {|be[15:12], |be[11:8], |be[7:4], |be[3:0]};
endfunction

// left-align an operand of msz
function automatic logic [31:0] lalign(input logic [31:0] d, input logic [1:0] msz);
	case (msz)
		SZ_B:    lalign = {d[7:0], 24'd0};
		SZ_W:    lalign = {d[15:0], 16'd0};
		default: lalign = d;
	endcase
endfunction

//--------------------------------------------------------------------------
// storage
//--------------------------------------------------------------------------
// line valid and dirty, per set and way: one word per set, {dirty[3:0],
// valid[3:0]}, in LUT RAMs written as flip-flops were (at the edge) and
// read without a clock.  A LUT RAM is 32 deep whatever its width up to 20,
// so the ways share a word: copies at DC1's set (valid and dirty), at the
// engine's or the snoop's (valid; a snoop freezes the engine), at the
// scan's (dirty), and at the write's set, which forms the word written
// (the ways not written keep their bits).  A clear of every line (CINVA,
// reset) sweeps the 64 sets (E_M_CLR)
(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] lsd [64];    // DC1's set
(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] lse [64];    // lv_ra
(* ramstyle = "MLAB, no_rw_check" *) logic [7:0] lsw [64];    // the write's set
logic  [5:0] lv_ra;                // the engine copy's read: see below
wire   [3:0] lvd_q;                // DC1's set (m1)
wire   [3:0] lve_q;                // the engine's (or the snoop's) set
logic  [5:0] clr_i;                // the clear sweep's set
logic        clr_rst;              // ... after reset (else CINVA: then E_M_IC)
// synthesis translate_off
logic [3:0]  lv  [64];              // (simulation: the flip-flop model, checked)
// synthesis translate_off
initial for (int i = 0; i < 64; i++) lv[i] = 4'd0;
// synthesis translate_on
// synthesis translate_on
// dirty [set][way] (the push writes the whole line): read at the scan's
// set, the snooped set, the walk's descriptor set, else the fill/push set
// (they never read at once: a snoop freezes the engine).  No clear: every
// read is of a valid line, and whatever makes a line valid writes its
// dirty bit
logic  [5:0] ld_ra;
wire   [7:0] lsm_q = lse[lv_ra];
wire   [3:0] ld_q  = lsm_q[7:4];
// the maintenance scan's set, its valid and dirty bits: a copy the write
// port keeps current (the scan's decision reads 4 bits, not 256)
logic  [5:0] sc_set;
logic  [3:0] sc_lv, sc_ld;
logic [1:0]  rrc;                  // replacement counter

// lookup ports: DC1, or the engine while it holds DC1
logic        steal, steal_q;
logic [31:0] st_la;                // engine lookup address (logical, for the ATC)
logic        st_fc2;
logic  [5:0] st_set;               // engine data/tag set

// snoop port phases: A presents the snooped set, B compares, H holds the
// cache for the snooper's action, R re-reads the owner's address
typedef enum logic [2:0] { SN_IDLE, SN_A, SN_B, SN_H, SN_R } snph_t;
snph_t       sn_ph;
wire         sn_frz  = (sn_ph != SN_IDLE);
assign       b_drop  = bo_cancel && bo_ingnt && (e_st == E_BUS_W);
wire         sn_rset = (sn_ph == SN_A) || (sn_ph == SN_B);   // B: the line stays readable in H
logic  [1:0] sn_src;               // found in: 0 a cache way, 1 push buffer, 2 queued push
logic        sn_pvm, sn_bom;       // ... and (whatever sn_src says) the push buffer's
                                   // line, the queued push's line: both go with it
logic  [1:0] sn_way;
logic        bo_cancel;            // the queued push was invalidated by a snoop
logic        bo_ingnt;             // the engine's transfer is the BIU's (granted)
logic  [3:0] bo_got;               // a line read: the long words that arrived

// The lookup set: AG's address (late: register read, address adders)
// passes one LUT; the other choices are registers, merged beforehand.
// Each way's RAMs get their own copy of that LUT (fanout 18, not 72).
wire  [5:0] la_alt   = sn_rset ? sn_pa[9:4] : steal ? st_set : m1.a[9:4];
wire        la_ag    = adv_ag && !sn_rset && !steal;

// tags: port A for lookups (DC1 or stolen), port B for engine reads/writes
logic [21:0] tq_a [4];
// the tags as the RAM read them, without the same-edge write laid over:
// DC1's compare.  Tags are written only by the engine's fill (tw_b), and a
// DC1 lookup with the engine busy is stale (m1_stale) and not used, so the
// bypass's compare and multiplexer stay off DC1's hit
logic [21:0] tq_r [4];
logic        m1_stale;          // DC1's lookup is not to be used (below)
logic [21:0] tq_b [4];
logic        tw_b;
logic  [5:0] tset_b;
logic  [1:0] tway_b;
logic [21:0] twd_b;

// data: write port for stores and fills, read port for DC1 / engine
logic  [63:0] dqc;                 // DC1's two long words, every clock
logic  [63:0] dq2, dqh2;           // DC2's two long words (at its address and
                                   // the next), and its own, held
logic         dq_new;              // DC2 took a new uop at the last edge: dqc
xres_t        x_dc1;               // DC1's translation and lookup
logic [127:0] dq_rn [4];           // unregistered output (engine)
logic         dw;
logic  [5:0]  dw_set;
logic  [1:0]  dw_way;
logic [15:0]  dw_be;
logic  [1:0]  dw_src;              // 0 dw_word on every long word, 1 bo_line, 2 pv_line
logic [31:0]  dw_word;
logic [127:0] dw_wdata;
// the previous cycle's RAM write (it landed at the last edge)
logic         dwq;
logic  [5:0]  dwq_set;
logic  [1:0]  dwq_way, dwq_src;
logic [15:0]  dwq_be;
logic [31:0]  dwq_word;
always_ff @(posedge clk) begin
	dwq <= dw; dwq_set <= dw_set; dwq_way <= dw_way; dwq_src <= dw_src;
	dwq_be <= dw_be; dwq_word <= dw_word;
end
// bytes of a long word (on every long word of a line) merged into a line
function automatic logic [127:0] mrg(input logic [127:0] l, input logic [15:0] be,
                                     input logic [31:0] w);
	for (int i = 0; i < 16; i++) if (be[i]) l[8*i +: 8] = w[8*(i % 4) +: 8];
	mrg = l;
endfunction
// the two long words of a line from long word k (the second wraps): every
// byte DC2 reads -- the operand (at most four bytes from its address) and
// AG's aligned long word -- is in them
function automatic logic [63:0] pair(input logic [127:0] l, input logic [1:0] k);
	pair = {l[127 - 32 * k -: 32], l[127 - 32 * 2'(k + 2'd1) -: 32]};
endfunction
// a write merged into such a pair (be per byte of the line as mrg's)
function automatic logic [63:0] mrg2(input logic [63:0] p, input logic [1:0] k,
                                     input logic [15:0] be, input logic [31:0] w);
	for (int b = 0; b < 8; b++) begin
		logic [3:0] lb;
		lb = 4'(4 * k + b);                       // the byte's offset in the line
		if (be[4'd15 - lb]) p[63 - 8*b -: 8] = w[31 - 8*(b % 4) -: 8];
	end
	mrg2 = p;
endfunction

// the WB fast store of this cycle (its RAM write is dw next cycle): merged
// into DC2's copy now, so a load behind it does not wait for that write
logic         wfm;
logic  [5:0]  wfm_set;
logic  [1:0]  wfm_way;
logic [15:0]  wfm_be;
logic [31:0]  wfm_word;
logic         dwp;            // this cycle's dw was merged a cycle ago (wfm)

genvar gw;
generate
	for (gw = 0; gw < 4; gw++) begin : g_way
		(* keep *) wire [5:0] la_set_n = la_ag ? dm_va[9:4] : la_alt;
		ap68040_tdp #(.AW(6), .DW(22)) tag (
			.clk(clk),
			.addr_a(la_set_n), .we_a(1'b0), .wd_a(22'd0), .q_a(tq_a[gw]),
			.qa_raw(tq_r[gw]), .qa_bv(), .qa_bd(),
			.addr_b(tset_b), .we_b(tw_b && tway_b == 2'(gw)), .wd_b(twd_b), .q_b(tq_b[gw])
		);
		ap68040_sdp_be #(.AW(6), .NB(16), .OREG(0)) dat (
			.clk(clk),
			.we(dw && dw_way == 2'(gw)), .waddr(dw_set), .wbe(dw_be), .wdata(dw_wdata),
			.raddr(la_set_n), .oce(1'b1), .q(dq_rn[gw])
		);
	end
endgenerate

// DC2's copy of the line its lookup hit (only that way is ever read: the
// fast load, a split load's first part, AG's aligned word): byte writes to
// it are merged in (the RAM output misses the writes landing at the last
// edge and this one), so a load after a store to its line stays on the
// fast path
// (Both are captured every clock and the registered advance picks: the
// stall chain behind adv_dc1 is too late for a 128-bit enable.)
// (the line by the one-hot match, AND-OR; the merges that apply to it are
// registered beside it -- their bytes, data, and whether they hit it -- and
// laid over it in DC2, off the hit path: without a match the line is not
// used)
logic [15:0] mc_be [3];
logic [31:0] mc_w  [3];
logic  [2:0] mc_v;
always_ff @(posedge clk) begin
	logic  [63:0] l;
	logic  [63:0] h2;
	// (each way's two long words picked by the registered address, then
	// the hit way: the late match selects 64 bits, not the line)
	l = '0;
	for (int j = 0; j < 4; j++) if (d1_mj[j]) l = l | pair(dq_rn[j], m1.a[3:2]);
	dqc <= l;
	mc_v[0] <= dwq && dwq_src == 2'd0 && dwq_set == m1.a[9:4] && d1_mj[dwq_way];
	mc_v[1] <= dw && dw_src == 2'd0 && dw_set == m1.a[9:4] && d1_mj[dw_way];
	mc_v[2] <= wfm && wfm_set == m1.a[9:4] && d1_mj[wfm_way];
	mc_be[0] <= dwq_be; mc_w[0] <= dwq_word;
	mc_be[1] <= dw_be;  mc_w[1] <= dw_word;
	mc_be[2] <= wfm_be; mc_w[2] <= wfm_word;
	h2 = dq2;
	if (dw && dw_src == 2'd0 && dw_set == m2.r.a[9:4] && dw_way == m2.x.way)
		h2 = mrg2(h2, m2.r.a[3:2], dw_be, dw_word);
	if (wfm && wfm_set == m2.r.a[9:4] && wfm_way == m2.x.way)
		h2 = mrg2(h2, m2.r.a[3:2], wfm_be, wfm_word);
	dqh2 <= h2;
	dq_new <= adv_dc1;
end
// DC2: its two long words of the captured line, with the merges (in the
// order they were made)
logic [63:0] dqc_m2;
always_comb begin
	dqc_m2 = dqc;
	for (int k = 0; k < 3; k++) if (mc_v[k]) dqc_m2 = mrg2(dqc_m2, m2.r.a[3:2], mc_be[k], mc_w[k]);
end
assign dq2 = dq_new ? dqc_m2 : dqh2;

//--------------------------------------------------------------------------
// D-ATC
//--------------------------------------------------------------------------
logic        atc_hit;
atce_t       atc_e;
logic  [3:0] atc_hv, atc_hvd;
logic [19:0] atc_wpa [4], atc_wpad [4];
logic  [3:0] m1_aset;            // DC1's ATC set, registered with its address
// The main ATC port's set, a flip-flop: DC1's (m1_aset) one cycle, the
// stolen address's the cycle after the engine stole the port.  The engine
// reads the port a cycle or more after it sets st_la, and DC1 is held the
// cycle after a steal ends (steal_q): neither uses the lagging cycle.
// (its fanout capped: it reaches every way's entry array, and the
// fitter duplicates it per way rather than routing one copy to all)
(* altera_attribute = "-name MAX_FANOUT 14" *) logic [3:0] atc_set_q;
// ... and the page and FC2 its tags are compared with, registered the same
// way (no multiplexer between the steal flag and the compare)
logic [31:12] atc_la_q;
logic         atc_fc2_q;
logic        atc_wr;
logic [31:0] atc_wla;
logic        atc_wfc2;
atce_t       atc_went;
logic        atc_fall, atc_fpage, atc_fng;

logic        tw_upd;              // the search updates an entry (first write: M)
logic  [3:0] tw_hw;               // ... in this way (the ATC's hit vector then)
ap68040_atc #(.SAME_SEARCH(0), .EXT_SET(1)) datc (
	.clk(clk), .nreset(nreset), .p8k(tc_p),
	.la({atc_la_q, 12'd0}), .fc2(atc_fc2_q), .set_m(atc_set_q),
	.hit(atc_hit), .ent(atc_e), .hv(atc_hv), .wpa(atc_wpa),
	.la_d(m1.a), .fc2_d(m1.smode), .set_d(m1_aset), .hv_d(atc_hvd), .wpa_d(atc_wpad),
	.wr(atc_wr), .wla(atc_wla), .wfc2(atc_wfc2), .went(atc_went),
	.wsame(tw_upd), .wsame_hv(tw_hw),
	.flush_all(atc_fall), .flush_page(atc_fpage), .flush_nonglobal(atc_fng)
);

//--------------------------------------------------------------------------
// translation and cache tag compare of a lookup
//--------------------------------------------------------------------------
function automatic xres_t xlate(input logic [31:0] la, input logic s, input logic wr,
                                input logic alt,
                                input logic hitv, input atce_t e,
                                input logic [21:0] t0, input logic [21:0] t1,
                                input logic [21:0] t2, input logic [21:0] t3,
                                input logic [3:0] valid);
	xres_t x;
	logic tt0, tt1;
	logic [21:0] tg;
	x = '0;
	tt0 = ttr_hit(dtt0, la, s);
	tt1 = ttr_hit(dtt1, la, s);
	// (MOVES to an alternate address space, SFC/DFC 0, 3, 4, 7: MC68040UM
	// 3.2.5 calls it a physical access without translation; WinUAE -- the
	// cputest oracle, and the reference this core is validated against --
	// translates every space through the root FC2 selects, and t_mmu
	// 177-180 pin that: alt is ignored here, the bus still shows TT=2, TM=FC)
	if (tt0 || tt1) begin
		x.ok  = 1'b1;
		x.pa  = la;
		x.cm  = tt0 ? dtt0[6:5] : dtt1[6:5];
		x.upa = tt0 ? dtt0[9:8] : dtt1[9:8];
		x.flt = wr && (tt0 ? dtt0[2] : dtt1[2]);
	end
	else if (!tc_e) begin
		// translation disabled, no TTR match: cachable/write-through, not
		// write protected, UPA 0 (MC68040UM 3.1.2)
		x.ok = 1'b1;
		x.pa = la;
		x.cm = 2'b00;
	end
	else if (hitv) begin
		x.pa  = {e.pa[19:1], tc_p ? la[12] : e.pa[0], la[11:0]};
		x.cm  = e.cm;
		x.upa = e.upa;
		if (!e.r || (e.s && !s) || (wr && e.w)) begin
			x.flt = 1'b1;      // nonresident, supervisor only, write protected
			x.ok  = 1'b1;
		end
		else if (wr && !e.m) x.walk = 1'b1;   // first write: set M (3.3)
		else x.ok = 1'b1;
	end
	else x.walk = 1'b1;
	tg = x.pa[31:10];
	if (x.ok && !x.flt) begin
		if (valid[0] && t0 == tg) begin x.hit = 1'b1; x.way = 2'd0; end
		if (valid[1] && t1 == tg) begin x.hit = 1'b1; x.way = 2'd1; end
		if (valid[2] && t2 == tg) begin x.hit = 1'b1; x.way = 2'd2; end
		if (valid[3] && t3 == tg) begin x.hit = 1'b1; x.way = 2'd3; end
	end
	xlate = x;
endfunction

// DC1
wire        m1_wr  = (m1.mem == M_ST) || (m1.mem == M_RMW);
// DC1's cache tag compare in parallel with the translation (the timing
// path): each way's tag against the untranslated address and against the
// physical page of each of the ATC's four ways; the ATC's hit vector, not
// its selected entry, then picks.  Whether the access may use the hit (a
// translation that is valid and does not fault) still comes from xlate.
logic       d1_hit;
logic [1:0] d1_way;
xres_t      x_dc1r;            // xlate's own result for DC1
always_comb x_dc1r = xlate(m1.a, m1.smode, m1_wr, fc_tt(m1.fc) == TT_ALT, atc_hit, atc_e,
                          tq_a[0], tq_a[1], tq_a[2], tq_a[3], lvd_q);
logic [3:0] d1_mj;            // the matching way, one-hot (for the line select)
// whether DC1's address is used untranslated (alternate space, a TTR,
// translation off, CPU space): registered with the address, and again
// every clock DC1 holds, so the compare select starts from a flip-flop
logic       m1_tp;
function automatic logic transp_of(input logic [31:0] a, input logic s, input logic [2:0] fc,
                                   input logic [1:0] ack);
	transp_of = ttr_hit(dtt0, a, s) || ttr_hit(dtt1, a, s) ||
	            !tc_e_n || (ack != 2'd0);
endfunction
always_comb begin
	logic        transp;
	logic [21:0] tq [4];
	logic  [3:0] mj;
	tq[0] = tq_r[0]; tq[1] = tq_r[1]; tq[2] = tq_r[2]; tq[3] = tq_r[3];
	transp = m1_tp;
	for (int j = 0; j < 4; j++) begin
		logic ma;
		ma = 1'b0;
		for (int i = 0; i < 4; i++)
			if (atc_hvd[i] && tq[j] == {atc_wpad[i][19:1], tc_p ? m1.a[12] : atc_wpad[i][0], m1.a[11:10]})
				ma = 1'b1;
		mj[j] = lvd_q[j] && (transp ? (tq[j] == m1.a[31:10]) : ma);
	end
	if (!DC1_ATC) begin
		// the main port's entry (its hit and page), compared with each way's
		// tag beside the untranslated address: the registered transparency
		// selects, so the TTR compare and the translation's address select
		// stay off the tag compare
		logic [21:0] tpa;
		tpa = {atc_e.pa[19:1], tc_p ? m1.a[12] : atc_e.pa[0], m1.a[11:10]};
		for (int j = 0; j < 4; j++)
			mj[j] = lvd_q[j] && (transp ? (tq[j] == m1.a[31:10]) : (atc_hit && tq[j] == tpa));
	end
	d1_hit = |mj;
	d1_way = mj[3] ? 2'd3 : mj[2] ? 2'd2 : mj[1] ? 2'd1 : 2'd0;
	d1_mj  = mj;
end
// synthesis translate_off
// (the engine's state for the bench's profile: a parameterized module's
// enum items are not reachable through the hierarchy in Verilator)
wire sim_es_s  = (e_st == E_S_START);
wire sim_es_w  = (e_st == E_W_START);
wire sim_es_m  = (e_st == E_M_START);
wire sim_es_tw = (e_st == E_TW_START);
wire sim_es_i  = (e_st == E_IDLE);
logic sim_xchk = 1'b0;        // (the bench fails the run on it)
logic sim_kchk = 1'b0;        // a job completed for a discarded uop (E_S_DONE)
always_ff @(posedge clk) begin
	xres_t xo;
	xo = xlate(m1.a, m1.smode, m1_wr, fc_tt(m1.fc) == TT_ALT, atc_hit, atc_e,
	           tq_a[0], tq_a[1], tq_a[2], tq_a[3], lvd_q);
	// (while the engine steals the ATC's main port, xlate sees its address,
	// not DC1's: DC1 waits then)
	if (nreset && !steal && m1.ack == 2'd0 && m1.v && !m1_stale &&
	    (xo.hit != x_dc1.hit || (xo.hit && xo.way != x_dc1.way)))
	begin
		$display("FAIL: DMU: the parallel tag compare says hit=%b way=%0d, xlate hit=%b way=%0d",
		         x_dc1.hit, x_dc1.way, xo.hit, xo.way);
		sim_xchk <= 1'b1;
	end
end
// synthesis translate_on

always_comb begin
	x_dc1 = x_dc1r;
	// (the same hit and way, from the parallel compare)
	x_dc1.hit = d1_hit && x_dc1.ok && !x_dc1.flt;
	x_dc1.way = d1_way;
	if (m1.ack != 2'd0) begin
		// CPU space (interrupt acknowledge) is never translated nor cached
		x_dc1     = '0;
		x_dc1.ok  = 1'b1;
		x_dc1.pa  = m1.a;
		x_dc1.cm  = 2'b10;
	end
end
wire  [4:0] m1_end   = {1'b0, m1.a[3:0]} + {2'b00, nbytes(m1.msz)};
wire        m1_split = (m1.msz != SZ_Q) && (m1_end > 5'd16);
wire  [7:0] lsd_q    = lsd[m1.a[9:4]];
wire        m1_sfast = ((m1.mem == M_LD) || (m1.mem == M_ST && x_dc1.cm == 2'b01)) &&
                       m1_split && x_dc1.ok && !x_dc1.flt && x_dc1.hit &&
                       !x_dc1.cm[1] && dc_en && (m1.ack == 2'd0) && !m1.lock;
// (locked accesses -- TAS, CAS, CAS2 -- are noncachable (MC68040UM 7.4.1):
// the slow path pushes and invalidates a hit first)
wire        m1_fast  = (m1.mem == M_LD || m1.mem == M_RMW) && !m1.lock &&
                       x_dc1.ok && !x_dc1.flt && x_dc1.hit &&
                       !x_dc1.cm[1] && dc_en && (m1.ack == 2'd0) &&
                       (m1.msz != SZ_Q) && !m1_split;

//--------------------------------------------------------------------------
// DC2
//--------------------------------------------------------------------------
// a store older than the DC2 load and not yet in the data RAM (EX, WB)
wire  [27:0] m2_line = m2.x.pa[31:4];
// (by the line within the page only, PA11-4: a store to the same offset of
// another page is taken for the line too -- the load waits for it, or takes
// the slow path; a smaller, shorter compare on DC2's hold)
function automatic logic st_line(input mrec_t m, input logic [27:0] l);
	st_line = m.r.v && (m.r.mem == M_ST || m.r.mem == M_RMW) &&
	          (m.x.pa[11:4] == l[7:0] || (m.split && m.x1.pa[11:4] == l[7:0]));
endfunction
wire hz = st_line(m3, m2_line) || st_line(m4, m2_line);
// the fast load (never split) waits only for a store whose bytes it reads:
// a plain store within one line is compared by bytes, anything else (RMW,
// split, MOVE16) by the line
wire [15:0] m2_bm = bmask(m2.r.a[3:0], nbytes(m2.r.msz));
function automatic logic st_byte(input mrec_t m);
	st_byte = st_line(m, m2_line) &&
	          (m.r.mem == M_RMW || m.split || m.r.msz == SZ_Q ||
	           (bmask(m.x.pa[3:0], nbytes(m.r.msz)) & m2_bm) != 16'd0);
endfunction
wire hzf = st_byte(m3) || st_byte(m4);
// any store older than the DC2 uop not yet performed: a table walk for the
// DC2 uop must see it (the program may just have written a descriptor)
wire st_older = (m3.r.v && (m3.r.mem == M_ST || m3.r.mem == M_RMW)) ||
                (m4.r.v && (m4.r.mem == M_ST || m4.r.mem == M_RMW));
// a RAM write to the load's set, or any engine activity, after its lookup
// read the RAM makes the captured line stale: the engine reads it again

// (a store that left WB is still being written this cycle: dw)
wire fast_now = m2.r.v && m2.fast && !hzf && !(dw && !dwp && dw_set == m2.r.a[9:4]);
// (the operand from DC2's two long words: its bytes start at a[1:0] of the
// first; a split load's bytes past the line are masked by its user)
wire [31:0] fast_data = take({dq2, 64'd0}, {2'd0, m2.r.a[1:0]}, nbytes(m2.r.msz));

// a store with its translation known needs nothing from DC2 unless it must
// allocate (copyback miss), is split, or is a MOVE16 destination
wire st_simple = m2.r.v && (m2.r.mem == M_ST) && m2.x.ok && !m2.x.flt && !m2.split &&
                 (m2.r.msz != SZ_Q) &&
                 !(dc_en && m2.x.cm == 2'b01 && !m2.x.hit && !m2.r.noalloc && !m2.r.lock);

//--------------------------------------------------------------------------
// engine
//--------------------------------------------------------------------------
typedef enum logic [5:0] {
	E_IDLE,
	E_S_START, E_S_XL, E_S_XLW, E_S_XL2, E_S_ACT, E_S_RD, E_S_RDW, E_S_RD2, E_S_BUSD,
	E_S_NEXT, E_S_DONE, E_SP_W, E_SP_2, E_SP_3,
	E_W_START, E_W_PART, E_W_DONE,
	E_M_START, E_M_PG, E_M_SCAN, E_M_SCANW, E_M_SCAN2, E_M_IC, E_M_DONE, E_M_CLR,
	E_FILL, E_FILLW, E_FILL_W, E_FILL_INS, E_PUSHV, E_PUSHVW, E_PUSHV_W, E_PUSHV_B,
	E_BUS, E_BUS_W,
	E_TW_START, E_TW_DESC, E_TW_DESCW, E_TW_DESCW2, E_TW_DLK, E_TW_DBUS, E_TW_EVAL, E_TW_UPD, E_TW_UPD_W,
	E_TW_LKW, E_TW_DONE,
	E_I_DONE
} est_t;

est_t        e_st, e_ret, e_sret, e_wret;
logic        e_dc2_done;          // DC2 has its answer (until it advances)
logic        e_wst_done;          // the WB store is done (until WB advances)
logic [31:0] e_ldata;
logic        e_flt;
logic [31:0] e_faddr;
logic [15:0] e_fssw;
logic        e_part;              // working on part 1 of a split access
xres_t       e_x;                 // translation of the current part
logic [31:0] e_va;                // logical address of the current part
logic  [2:0] e_n;                 // bytes in the current part
logic [31:0] e_acc;               // load: assembled operand; store: left aligned data
logic        e_fromline;          // the operand comes from bo_line (inhibited fill)
logic        e_fillhit;           // ... from the line a load's own fill just installed
logic        e_kill;              // the DC2 job's uop was discarded
logic        sp_job;              // the DC2 job is a fast split read
logic        sp_dw;               // a write to the next set while its
                                  // stolen read was in flight (E_SP_W)
// engine states that write no data, tag or valid bit: the RAM outputs a
// lookup captured stay good (a fast split read; the RAM re-reads the held
// DC1 set after its stolen read, steal_q)
wire  e_quiet = (e_st == E_IDLE) || (e_st == E_S_START) || (e_st == E_SP_W) ||
                (e_st == E_SP_2) || (e_st == E_SP_3) || (e_st == E_S_DONE && sp_job);
logic        e_job;               // the engine owns the DC2 uop until it is done

// bus operation built by a job, run by E_BUS
typedef struct packed {
	logic        line;            // a line transfer
	logic        rd;
	logic  [1:0] tt;
	logic  [2:0] tm;
	logic  [1:0] upa;
	logic        ci;
	logic        lock;
	logic        locke;
	logic  [1:0] ack;             // acknowledge cycle (see req_t)
	logic  [1:0] tln;
} bop_t;
bop_t        bo;
logic [31:0] bo_wd;               // write data (left aligned)
logic [127:0] bo_line;            // line write data / line read result
logic        bo_err;
logic [31:0] bo_rd;               // read data (right aligned)
logic        bo_avec;
logic        bo_tci;
logic  [2:0] bo_left;             // bytes still to do
logic  [2:0] bo_pn;               // bytes in the piece on the bus
logic [31:0] bo_pa;               // address of the piece on the bus

// fill/push state
logic  [5:0] f_set;
// the dirty RAM's one read: the snoop's compare, the scan's set, else the
// fill/push set
// (the dirty bits share the valid copy's read: the states that read
// one and not the other are given the other's set)
always_comb ld_ra = lv_ra;
logic  [1:0] f_way;
logic [21:0] f_tag;
logic        pv_v;                // push buffer holds a dirty line
logic [127:0] pv_line;
logic [27:0] pv_lpa;              // its line address
logic        pv_dirty;
logic  [1:0] pv_tln;
logic        pv_lock;             // E_PUSHV: the push is part of a locked sequence (LOCK, 4.6.2)
logic  [1:0] pv_for;              // E_PUSHV: the push serves 0 a DC2 job, 1 a WB store, 2 CPUSH
logic        pbe_r;               // a push ended in TEA: an access error at once (7.6.1)
logic [31:0] pbe_a;               // ... the push's address
logic [127:0] mv_line;            // MOVE16 line buffer

// the RAM write data: a fill or a restored victim (the buffers hold until
// the write cycle), else a word on every long word with byte enables
assign dw_wdata = (dw_src == 2'd1) ? bo_line : (dw_src == 2'd2) ? pv_line : {4{dw_word}};

// maintenance scan
logic  [7:0] ms_i;                // set*4 + way

// table walk state
typedef enum logic [1:0] { TW_ROOT, TW_PTR, TW_PAGE, TW_IND } twl_t;
twl_t        tw_lvl;
logic [31:0] tw_va;
logic        tw_s, tw_wr, tw_pt, tw_i;   // FC2, write access, PTEST, I-side
logic [31:0] tw_da;               // descriptor address
logic [31:0] tw_d;                // descriptor
logic        tw_wp;               // accumulated write protect
logic        tw_fail, tw_berr;
logic [31:0] tw_pd;               // final page descriptor
logic        tw_lkrd;             // the locked update's read is in flight (E_TW_LKW follows)

wire [6:0] tw_ri = tw_va[31:25];
wire [6:0] tw_pi = tw_va[24:18];
wire [5:0] tw_gi = tc_p ? {1'b0, tw_va[17:13]} : tw_va[17:12];

// next aligned piece of an operand
function automatic void piece(input logic [31:0] a, input logic [2:0] left,
                              output logic [1:0] bsiz, output logic [2:0] n);
	if (a[1:0] == 2'b00 && left >= 3'd4) begin bsiz = SIZ_L; n = 3'd4; end
	else if (a[0] == 1'b0 && left >= 3'd2) begin bsiz = SIZ_W; n = 3'd2; end
	else begin bsiz = SIZ_B; n = 3'd1; end
endfunction

logic [1:0] bo_siz;
logic [2:0] bo_np;
always_comb piece(bo_pa, bo_left, bo_siz, bo_np);

// bytes of a read piece from the lanes, right aligned
function automatic logic [31:0] grab(input logic [31:0] d, input logic [2:0] n,
                                     input logic [1:0] a10);
	case (n)
		3'd1:    grab = {24'd0, d[31 - 8*a10 -: 8]};
		3'd2:    grab = {16'd0, a10[1] ? d[15:0] : d[31:16]};
		default: grab = d;
	endcase
endfunction

// the victim way of a set: the first invalid way, else the counter
function automatic logic [1:0] victim(input logic [3:0] valid, input logic [1:0] rr);
	logic [1:0] v;
	v = rr;
	for (int i = 3; i >= 0; i--) if (!valid[i]) v = 2'(i);
	victim = v;
endfunction

// a line targeted by a store still in EX/WB is never evicted
function automatic logic store_holds(input logic [5:0] set, input logic [1:0] way,
                                     input mrec_t a, input mrec_t b);
	store_holds = (a.r.v && (a.r.mem == M_ST || a.r.mem == M_RMW) && a.x.hit &&
	               a.x.pa[9:4] == set && a.x.way == way) ||
	              (b.r.v && (b.r.mem == M_ST || b.r.mem == M_RMW) && b.x.hit &&
	               b.x.pa[9:4] == set && b.x.way == way);
endfunction

// SSW of an access error (MC68040UM figure 8-6): ATC (an ATC fault, clear
// for a bus error), LK, RW, SIZE (the original access size), TT and TM as
// the access drives them (MOVES: table 3-2; a MOVE16 line operand reports
// TT = 01).  MA is set by the caller for an ATC fault of the second page.
function automatic logic [15:0] mk_ssw(input logic atc, input logic lk, input logic rd,
                                       input logic [1:0] msz, input logic [2:0] fc);
	logic [1:0] sz;
	case (msz)
		SZ_B: sz = 2'b01;
		SZ_W: sz = 2'b10;
		SZ_L: sz = 2'b00;
		default: sz = 2'b11;
	endcase
	mk_ssw = {4'b0000, 1'b0, atc, lk, rd, 1'b0, sz,
	          (msz == SZ_Q) ? TT_MOVE16 : fc_tt(fc), fc_tm(fc)};
endfunction

//--------------------------------------------------------------------------
// WB fast store: a copyback store that hit, within one line
//--------------------------------------------------------------------------
wire wb_st   = m4.r.v && st_v && (m4.r.mem == M_ST || m4.r.mem == M_RMW);
wire wb_fast = wb_st && !m4.split && m4.x.ok && !m4.x.flt && m4.x.hit &&
               m4.x.cm == 2'b01 && dc_en && !m4.r.lock && (m4.r.msz != SZ_Q) &&
               (e_st == E_IDLE) && !e_wst_done;
wire  [31:0] wbf_word = lanes32(lalign(st_data, m4.r.msz), m4.r.a[1:0]);
wire  [15:0] wbf_be   = bmask(m4.r.a[3:0], nbytes(m4.r.msz));
assign wfm      = wb_fast && !sn_frz;
assign wfm_set  = m4.x.pa[9:4];
assign wfm_way  = m4.x.way;
assign wfm_be   = wbf_be;
assign wfm_word = wbf_word;
always_ff @(posedge clk) dwp <= nreset && wfm;
// a copyback store across a line, both parts hits: the first part this
// cycle, the second the next (wfs_p1), without the engine
logic        wfs_p1;
wire wb_fs = wb_st && m4.split && m4.x.ok && !m4.x.flt && m4.x.hit && m4.x.cm == 2'b01 &&
             m4.x1.ok && !m4.x1.flt && m4.x1.hit && m4.x1.cm == 2'b01 && dc_en &&
             !m4.r.lock && (m4.r.msz != SZ_Q) && (e_st == E_IDLE) && !e_wst_done && !wfs_p1;
wire   [2:0] wfs_n0  = 3'(5'd16 - {1'b0, m4.r.a[3:0]});
wire  [31:0] wfs_acc = lalign(st_data, m4.r.msz);

// a fast load behind an older store to its line waits for it (merged into
// its copy) instead of taking the engine
wire fast_wait = m2.r.v && m2.fast && (hzf || (dw && !dwp && dw_set == m2.r.a[9:4]));
// (not while an older store is in EX: it reaches WB, which has the engine
// first, before DC2's job -- the bus sees a store before a younger load's
// access, whatever the timing)
wire ex_st    = m3.r.v && (m3.r.mem != M_LD);
wire dc2_slow = m2.r.v && !fast_now && !fast_wait && !(st_simple && !e_job) && !e_dc2_done && !e_job &&
                !ex_st;
wire wb_slow  = wb_st && !wb_fast && !wb_fs && !wfs_p1 && !e_wst_done;

// store ready: the fast store answers in its own WB cycle (the engine's
// stores through the registered pulse)
logic st_rdy_r, st_fault_r;
assign st_rdy   = st_rdy_r || (wb_fast && !sn_frz) || (wfs_p1 && m4.x1.hit && !sn_frz);
assign st_fault = st_rdy_r && st_fault_r;

// a failed line fill: the long words of the line the access needs (this
// part's bytes e_va[3:0] .. +e_n-1) all arrived -- a load can complete
// from them; a store needs none from memory, only that the fill started
wire  [3:0] fill_last  = {1'b0, e_va[3:0]} + {1'b0, e_n} - 4'd1;
wire  [3:0] fill_need  = ((4'b1111 << e_va[3:2]) & (4'b1111 >> (2'd3 - fill_last[3:2])));
wire        fill_late_ok = (m2.r.mem == M_ST) ? (bo_got != 4'd0)
                                              : ((fill_need & ~bo_got) == 4'd0);

//--------------------------------------------------------------------------
// BIU request from the engine's bus step
//--------------------------------------------------------------------------
logic e_breq;
always_comb begin
	b_req  = e_breq;
	b_breq = '0;
	// acknowledge cycles (TT=3): IACK at $FFFFFFFF, a byte, TM = the level
	// (7.5.1); the breakpoint acknowledge a byte read at 0, TM = 0 (7.5.2)
	b_breq.addr  = (bo.ack == 2'd1) ? 32'hFFFF_FFFF : bo_pa;
	b_breq.siz   = (bo.ack == 2'd1) ? SIZ_B : bo.line ? SIZ_LINE : bo_siz;
	b_breq.rd    = bo.rd;
	b_breq.tt    = (bo.ack != 2'd0) ? TT_ACK : bo.tt;
	b_breq.tm    = (bo.ack == 2'd1) ? iack_lvl : (bo.ack != 2'd0) ? 3'd0 : bo.tm;
	b_breq.tln   = bo.tln;
	b_breq.upa   = bo.upa;
	b_breq.ci    = bo.ci;
	b_breq.lock  = bo.lock;
	b_breq.locke = bo.locke && !bo.rd && (bo.line || bo_left == bo_np);
	b_wdata      = bo.line ? bo_line :
	               {96'd0, (bo_np == 3'd1) ? {4{bo_wd[31:24]}} :
	                       (bo_np == 3'd2) ? {2{bo_wd[31:16]}} : bo_wd};
end

// (a stale lookup waits the clock its set is read again, rather than
// sending its uop to the engine's general path: after a miss the loads
// behind it would otherwise each take that path, every one keeping the
// engine busy for the next)
assign dm_hold1 = sn_frz || steal || steal_q || (m1.v && m1_stale) ||
                  (e_st != E_IDLE && e_st != E_W_START &&
                   e_st != E_W_PART && e_st != E_W_DONE);

// the table walk's descriptor line in the cache: the hit and its way, from
// the tags and valid bits read at the walk's set (tw_busy: the valid copy
// reads that set throughout the walk, its bus cycles included), registered
// every cycle (tw_hhq, tw_hwq; tw_hvq says the cycle before read that
// set, unfrozen).  E_TW_DLK reads the descriptor by the registered way,
// E_TW_UPD pushes or invalidates by it (waiting a cycle after a snoop
// froze the engine): the compare is off the data and bookkeeping paths
logic        tw_hw_h;
logic  [1:0] tw_hw_w;
logic        tw_hhq, tw_hvq;
logic  [1:0] tw_hwq;
always_comb begin
	tw_hw_h = 1'b0; tw_hw_w = 2'd0;
	for (int i = 0; i < 4; i++)
		if (lve_q[i] && tq_b[i] == tw_da[31:10]) begin tw_hw_h = 1'b1; tw_hw_w = 2'(i); end
end
// the data RAM's way for the engine's line readers, one select (and one
// line multiplexer) for them, every input a flip-flop: the snooper's way
// while it has the cache (the engine is frozen then), the walk's
// descriptor way, else f_way -- the fill's or push's victim way, or the
// hit way an operand or MOVE16 read (E_S_RD, E_SP_2) set
wire  [1:0]  dq_wsel = sn_frz ? sn_way : (e_st == E_TW_DLK) ? tw_hwq : f_way;
wire [127:0] dq_w    = dq_rn[dq_wsel];
// the engine's operand bytes, one extractor for its states: E_SP_3 the
// rest of a split load from the stolen read, E_S_ACT from the line a fill
// delivered (bo_line), E_S_RD2 from the read way (through the shared line
// multiplexer: f_way holds the hit way then)
wire  [2:0]  sp_n0    = 3'(5'd16 - {1'b0, m2.r.a[3:0]});
wire         etk_sp   = (e_st == E_SP_3);
wire [127:0] etk_line = (e_st == E_S_ACT) ? bo_line : dq_w;   // (f_way: e_x.way)
wire  [3:0]  etk_o    = etk_sp ? 4'd0 : e_va[3:0];
wire  [2:0]  etk_n    = etk_sp ? 3'(nbytes(m2.r.msz) - sp_n0) : e_n;
wire [31:0]  etk      = take(etk_line, etk_o, etk_n);

always_ff @(posedge clk) begin
	// a store's answer is kept until WB takes it (WB can be held in the
	// cycle it comes), and e_wst_done keeps the engine from doing the store
	// again -- a pulse there was lost and WB waited for it forever
	if (adv_wb || !wb_st) st_rdy_r <= 1'b0;
	mt_done   <= 1'b0;
	iw_done   <= 1'b0;
	atc_wr    <= 1'b0;
	atc_fall  <= 1'b0;
	atc_fpage <= 1'b0;
	iatc_flush_all  <= 1'b0;
	iatc_flush_page <= 1'b0;
	iatc_wr   <= 1'b0;
	tw_b      <= 1'b0;
	dw        <= 1'b0;
	steal_q   <= steal;
	atc_la_q  <= steal  ? st_la[31:12] : adv_ag ? dm_va[31:12] : m1.a[31:12];
	atc_fc2_q <= steal  ? st_fc2 : adv_ag ? dm_super : m1.smode;
	atc_set_q <= steal  ? (tc_p ? st_la[16:13] : st_la[15:12]) :
	             adv_ag ? (tc_p ? dm_va[16:13] : dm_va[15:12]) :
	                      (tc_p ? m1.a[16:13]  : m1.a[15:12]);

	if (!nreset) begin
		m1 <= '0; m2 <= '0; m3 <= '0; m4 <= '0;
		// (reset leaves the cache alone -- tags, state and data, MC68040UM
		// 4.3: "reset does not invalidate the cache lines"; the valid and
		// dirty bits have their power-up values from their initial blocks,
		// and CINVA clears them when the software asks)
		e_st <= E_IDLE; e_ret <= E_IDLE; e_sret <= E_IDLE; e_wret <= E_IDLE;
		e_dc2_done <= 1'b0; e_wst_done <= 1'b0;
		e_ldata <= '0; e_flt <= 1'b0; e_faddr <= '0; e_fssw <= '0;
		e_part <= 1'b0; e_x <= '0; e_va <= '0; e_n <= '0; e_acc <= '0; e_fromline <= 1'b0; e_fillhit <= 1'b0;
		e_breq <= 1'b0;
		bo <= '0; bo_wd <= '0; bo_line <= '0; bo_err <= 1'b0; bo_rd <= '0;
		bo_avec <= 1'b0; bo_tci <= 1'b0; bo_left <= '0; bo_pn <= '0; bo_pa <= '0;
		steal <= 1'b0; st_la <= '0; st_fc2 <= 1'b0; st_set <= '0;
		f_set <= '0; f_way <= '0; f_tag <= '0;
		pv_v <= 1'b0; pv_line <= '0; pv_lpa <= '0; pv_dirty <= 1'b0; pv_tln <= '0; pv_lock <= 1'b0;
		pv_for <= 2'd0; pbe_r <= 1'b0; pbe_a <= '0;
		mv_line <= '0;
		ms_i <= '0;
		sc_set <= '0; sc_lv <= '0; sc_ld <= '0;
		tw_lvl <= TW_ROOT; tw_va <= '0; tw_s <= 1'b0; tw_wr <= 1'b0; tw_pt <= 1'b0;
		tw_i <= 1'b0; tw_da <= '0; tw_d <= '0; tw_wp <= 1'b0; tw_fail <= 1'b0;
		tw_berr <= 1'b0; tw_pd <= '0; tw_upd <= 1'b0; tw_hw <= 4'd0; tw_lkrd <= 1'b0; tw_hhq <= 1'b0; tw_hwq <= '0; tw_hvq <= 1'b0;
		tset_b <= '0; tway_b <= '0; twd_b <= '0;
		dw_set <= '0; dw_way <= '0; dw_be <= '0; dw_src <= '0; dw_word <= '0;
		ic_inv <= 1'b0; ic_inv_scope <= '0; ic_inv_pa <= '0;
		rrc <= 2'd0;
		st_fault_r <= 1'b0; st_fssw <= '0; st_faddr <= '0;
		mt_mmusr <= '0; mt_fault <= 1'b0; mt_fssw <= '0; mt_faddr <= '0;
		atc_wla <= '0; atc_wfc2 <= 1'b0; atc_went <= '0; atc_fng <= 1'b0;
		iatc_flush_ng <= 1'b0; iatc_flush_la <= '0; iatc_flush_fc2 <= 1'b0;
		iatc_wla <= '0; iatc_wfc2 <= 1'b0; iatc_went <= '0;
		iw_ent <= '0;
		e_kill <= 1'b0;
		sp_dw  <= 1'b0;
		sp_job <= 1'b0;
		wfs_p1 <= 1'b0;
		e_job <= 1'b0;
		m1_stale <= 1'b0;
		sn_ph <= SN_IDLE; sn_src <= '0; sn_way <= '0; bo_cancel <= 1'b0;
		bo_ingnt <= 1'b0; sn_pvm <= 1'b0; sn_bom <= 1'b0;
		sn_look <= 1'b0; sn_hit <= 1'b0; sn_dirty <= 1'b0;
		clr_i   <= '0;
		clr_rst <= 1'b0;
	end
	else begin
		// the valid/dirty write port: at most one writer per cycle (the
		// WB fast store runs only with the engine idle, the snoop port
		// only with it frozen), applied at the end of this block
		logic [5:0] vp_set;
		logic [3:0] vp_wm;               // ways written
		logic       vp_lwe, vp_lv;       // valid: write, value
		logic       vp_dwe, vp_dv;       // dirty: write, value
		vp_set = '0; vp_wm = '0; vp_lwe = 1'b0; vp_lv = 1'b0;
		vp_dwe = 1'b0; vp_dv = 1'b0;

		//--------------------------------------------------------------
		// stage records
		//--------------------------------------------------------------
		if (adv_ag) begin
			m1.v       <= dm_req;
			m1.a       <= dm_va;
			m1.mem     <= dm_mem;
			m1.msz     <= dm_msz;
			m1.fc      <= dm_fc;
			m1.lock    <= dm_lock;
			m1.locke   <= dm_locke;
			m1.smode   <= dm_super;
			m1.noalloc <= dm_noalloc;
			m1.ack     <= dm_ack;
			m1_tp      <= transp_of(dm_va, dm_super, dm_fc, dm_ack);
			m1_aset    <= tc_p ? dm_va[16:13] : dm_va[15:12];
			m1_stale   <= (dw && dw_src != 2'd0 && dw_set == dm_va[9:4]) || !e_quiet;
		end
		else begin
			if (adv_dc1) m1.v <= 1'b0;
			m1_tp <= transp_of(m1.a, m1.smode, m1.fc, m1.ack);
			m1_aset <= tc_p ? m1.a[16:13] : m1.a[15:12];
			if ((dw && dw_src != 2'd0 && dw_set == m1.a[9:4]) || !e_quiet) m1_stale <= 1'b1;
			// held, the RAMs and the ATC read DC1's own set again every
			// clock (la_alt): a clock with the engine quiet and no write to
			// that set makes the lookup good again from the next one
			else if (!steal && !sn_rset) m1_stale <= 1'b0;
		end

		if (adv_dc1) begin
			m2.r     <= m1;
			m2.x     <= x_dc1;
			m2.split <= m1_split;
			m2.x1    <= '0;
			m2.fast  <= m1_fast && !m1_stale && !(dw && dw_src != 2'd0 && dw_set == m1.a[9:4]) &&
			            (e_st == E_IDLE);
			// (a store reads nothing: the copy's state does not matter; its
			// first part's lookup is trusted as the general path does)
			m2.sfast <= m1_sfast && (m1.mem == M_ST ||
			            (!m1_stale && !(dw && dw_src != 2'd0 && dw_set == m1.a[9:4]) &&
			             (e_st == E_IDLE)));
		end
		else begin
			if (adv_dc2) m2.r.v <= 1'b0;
			if ((dw && dw_src != 2'd0 && dw_set == m2.r.a[9:4]) || (e_st != E_IDLE)) m2.fast <= 1'b0;
			// its own split lookup (a read) leaves the copy as it was
			if (((dw && dw_src != 2'd0 && dw_set == m2.r.a[9:4]) || !e_quiet) && m2.r.mem != M_ST)
				m2.sfast <= 1'b0;
		end

		if (adv_dc2) begin
			m3 <= m2;
			e_dc2_done <= 1'b0;
		end
		else if (adv_ex) m3.r.v <= 1'b0;

		if (adv_ex) m4 <= m3;
		else if (adv_wb) m4.r.v <= 1'b0;

		if (adv_wb) e_wst_done <= 1'b0;

		//--------------------------------------------------------------
		// the WB fast store
		//--------------------------------------------------------------
		if (wb_fast && !sn_frz) begin
			dw      <= 1'b1;
			dw_set  <= m4.x.pa[9:4];
			dw_way  <= m4.x.way;
			dw_be   <= wbf_be;
			dw_src  <= 2'd0;
			dw_word <= wbf_word;
			vp_set = m4.x.pa[9:4]; vp_wm[m4.x.way] = 1'b1; vp_dwe = 1'b1; vp_dv = 1'b1;
			// answered this cycle: kept only if WB does not move
			st_rdy_r   <= !adv_wb;
			st_fault_r <= 1'b0;
			e_wst_done <= !adv_wb;
		end
		if (wb_fs && !sn_frz) begin
			dw      <= 1'b1;
			dw_set  <= m4.x.pa[9:4];
			dw_way  <= m4.x.way;
			dw_be   <= bmask(m4.r.a[3:0], wfs_n0);
			dw_src  <= 2'd0;
			dw_word <= lanes32(wfs_acc, m4.r.a[1:0]);
			vp_set = m4.x.pa[9:4]; vp_wm[m4.x.way] = 1'b1; vp_dwe = 1'b1; vp_dv = 1'b1;
			wfs_p1  <= 1'b1;
		end
		if (wfs_p1 && !sn_frz) begin
			wfs_p1 <= 1'b0;
			if (m4.x1.hit) begin
				dw      <= 1'b1;
				dw_set  <= m4.x1.pa[9:4];
				dw_way  <= m4.x1.way;
				dw_be   <= bmask(4'd0, nbytes(m4.r.msz) - wfs_n0);
				dw_src  <= 2'd0;
				dw_word <= lanes32(wfs_acc << (8 * wfs_n0), 2'd0);
				vp_set = m4.x1.pa[9:4]; vp_wm[m4.x1.way] = 1'b1; vp_dwe = 1'b1; vp_dv = 1'b1;
				st_rdy_r   <= !adv_wb;
				st_fault_r <= 1'b0;
				e_wst_done <= !adv_wb;
			end
			else begin
				// a snoop took the second line in between: the engine
				// writes the second part (as its own access would)
				e_part <= 1'b1;
				e_va   <= {m4.r.a[31:4] + 28'd1, 4'd0};
				e_x    <= m4.x1;
				e_n    <= nbytes(m4.r.msz) - wfs_n0;
				e_acc  <= wfs_acc << (8 * wfs_n0);
				bo_err <= 1'b0;
				e_st   <= E_W_PART;
			end
		end

		// the walk's line compare, registered (see tw_hhq)
		tw_hhq <= tw_hw_h;
		tw_hwq <= tw_hw_w;
		tw_hvq <= tw_busy && !sn_frz && (f_set == tw_da[9:4]);
		if (tw_busy) f_set <= tw_da[9:4];   // (lv_ra: the walk's set)

		//--------------------------------------------------------------
		// engine (frozen while the snooper has the cache)
		//--------------------------------------------------------------
		if (!sn_frz)
		case (e_st)
		E_IDLE: begin
			steal <= 1'b0;
			// a fetch's walk: its registers loaded whatever the engine takes
			// up (they are free while it is idle, and every other walk loads
			// its own), so the job choice -- the kill late in it -- is not
			// their enable
			if (iw_req && !iw_done) begin
				tw_va <= iw_va; tw_s <= iw_fc2; tw_wr <= 1'b0; tw_pt <= 1'b0; tw_i <= 1'b1;
			end
			if (wfs_p1)
				;                       // a split WB store's second part next
			else if (mt_v && !mt_done)
				e_st <= E_M_START;
			else if (wb_slow)
				e_st <= E_W_START;
			else if ((dc2_slow || (e_job && m2.r.v && !e_dc2_done)) && !kill_now) begin
				// the job's kill mark is cleared HERE, where the uop is known
				// alive (kill_now is excluded), not in E_S_START: a kill can
				// land while the engine sits in E_S_START frozen by a snoop
				// (the kill block below runs every clock, the case does not),
				// and E_S_START's body, run after the freeze, would erase it
				// -- the job then finished for a discarded uop, set
				// e_dc2_done, and the next uop into DC2 was answered at once
				// with the dead job's data (OPENSTEP: a name-cache pointer
				// read as 0 under disk DMA, 2026-10-19)
				e_kill <= 1'b0;
				e_st   <= E_S_START;
			end
			else if (iw_req && !iw_done) begin
				e_ret <= E_I_DONE;
				e_st  <= E_TW_START;
			end
		end

		//==============================================================
		// DC2 slow path
		//==============================================================
		E_S_START: begin
			// (e_kill: cleared when the job was scheduled, see E_IDLE)
			e_job  <= 1'b1;
			e_part <= 1'b0;
			e_va   <= m2.r.a;
			e_x    <= m2.x;
			e_acc  <= '0;
			e_flt  <= 1'b0;
			e_fromline <= 1'b0;
			e_fillhit  <= 1'b0;
			e_n    <= m2.split ? 3'(5'd16 - {1'b0, m2.r.a[3:0]}) : nbytes(m2.r.msz);
			sp_dw  <= 1'b0;
			sp_job <= m2.sfast;
			if (m2.sfast) begin
				// a load across a line, its first part a hit held in DC2's
				// copy: look the next line up (translation, tags and data
				// at once through the stolen ports)
				steal  <= 1'b1;
				st_la  <= {m2.r.a[31:4] + 28'd1, 4'd0};
				st_fc2 <= m2.r.smode;
				st_set <= m2.r.a[9:4] + 6'd1;
				e_st   <= E_SP_W;
			end
			else
				e_st   <= m2.x.ok ? E_S_ACT : E_S_XL;
		end
		E_SP_W: e_st <= E_SP_2;         // the RAMs register the stolen address
		E_SP_3: begin
			logic [2:0] n0;
			n0 = 3'(5'd16 - {1'b0, m2.r.a[3:0]});
			if (!e_kill && m2.sfast && m2.r.mem == M_ST && e_x.ok && !e_x.flt && !e_x.walk &&
			    e_x.hit && e_x.cm == 2'b01) begin
				// a copyback store across a line: both parts hit, nothing
				// to read; WB writes them
				m2.x1 <= e_x;
				e_st  <= E_S_DONE;
			end
			else if (!e_kill && m2.sfast && m2.r.mem == M_LD && e_x.ok && !e_x.flt && !e_x.walk &&
			    e_x.hit && !e_x.cm[1] &&
			    !hz && !st_line(m3, e_x.pa[31:4]) && !st_line(m4, e_x.pa[31:4]) &&
			    !sp_dw && !(dw && (dw_set == m2.r.a[9:4] || dw_set == st_set))) begin
				// the first part from DC2's copy, the rest from the stolen read
				// (DC2's bytes are fast_data's: its low bytes, past the
				// line, give way to the stolen read's)
				e_acc <= (fast_data & ~((32'd1 << (8 * (nbytes(m2.r.msz) - n0))) - 32'd1)) |
				         etk;
				m2.x1 <= e_x;
				e_st  <= E_S_DONE;
			end
			else begin
				steal  <= 1'b0;
				sp_job <= 1'b0;
				e_x    <= m2.x;
				e_st   <= m2.x.ok ? E_S_ACT : E_S_XL;
			end
		end
		E_SP_2: begin
			xres_t x;
			x  = xlate({m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode, (m2.r.mem != M_LD),
			           fc_tt(m2.r.fc) == TT_ALT, atc_hit, atc_e,
			           tq_a[0], tq_a[1], tq_a[2], tq_a[3], lve_q);
			// the second part's translation and lookup, registered (the
			// decision and the data next cycle; the RAM keeps reading the
			// stolen set, the line multiplexer its way)
			e_x  <= x;
			f_way <= x.way;
			e_st <= E_SP_3;
		end
		E_S_XL: begin
			// look the current part up (translation and tags) via the
			// stolen ports; one cycle for the RAM and ATC reads
			steal  <= 1'b1;
			st_la  <= e_va;
			st_fc2 <= m2.r.smode;
			st_set <= e_va[9:4];
			e_st   <= E_S_XLW;
		end
		E_S_XLW: e_st <= E_S_XL2;      // the RAMs register the stolen address
		E_S_XL2: begin
			xres_t x;
			// (after a search: the ATC holds its result, a nonresident
			// entry for a failed one)
			x = xlate(e_va, m2.r.smode, (m2.r.mem != M_LD), fc_tt(m2.r.fc) == TT_ALT,
			          atc_hit, atc_e, tq_a[0], tq_a[1], tq_a[2], tq_a[3], lve_q);
			if (m2.r.ack != 2'd0) begin
				x    = '0;
				x.ok = 1'b1;
				x.pa = e_va;
				x.cm = 2'b10;
			end
			if (x.walk && e_kill) begin
				// the uop was discarded: no search for it
				steal  <= 1'b0;
				e_job  <= 1'b0;
				e_st   <= E_IDLE;
			end
			else if (x.walk && (st_older || dm_older)) begin
				// a table search only for the oldest instruction: older
				// stores first (they need the engine at WB), and nothing
				// older may discard the access after its search has set
				// history bits or met a bus error: retry
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (x.walk) begin
				tw_va <= e_va; tw_s <= m2.r.smode; tw_wr <= (m2.r.mem != M_LD);
				tw_pt <= 1'b0; tw_i <= 1'b0;
				tw_upd <= atc_hit;          // a first write: the entry's M
				tw_hw  <= atc_hv;           // (its way: the install rewrites it)
				e_ret <= E_S_XL;            // look up again after the walk
				e_st  <= E_TW_START;
			end
			else begin
				e_x  <= x;
				e_st <= E_S_ACT;
			end
		end
		E_S_ACT: if (e_kill) begin
			// the uop was discarded (no bus operation is in flight here)
			steal <= 1'b0;
			e_job <= 1'b0;
			e_st  <= E_IDLE;
		end
		else begin
			logic cach, alloc, rd, move16;
			move16 = (m2.r.msz == SZ_Q);
			cach   = dc_en && !e_x.cm[1] && !m2.r.lock && (m2.r.ack == 2'd0) && !move16;
			alloc  = cach && !m2.r.noalloc;
			rd     = (m2.r.mem == M_LD) || (m2.r.mem == M_RMW);
			if (e_x.flt) begin
				e_flt   <= 1'b1;
				// FA is the operand's first byte even when a later part
				// faulted; MA marks an ATC fault on the second page
				e_faddr <= m2.r.a;
				e_fssw  <= mk_ssw(1'b1, m2.r.lock, m2.r.mem == M_LD, m2.r.msz, m2.r.fc) |
				           (e_part ? 16'h0800 : 16'h0000);
				e_st    <= E_S_DONE;
			end
			else if (e_fillhit) begin
				// the fill installed the line (f_way) and it is in bo_line:
				// the operand from it, the part now a hit in that way
				e_acc     <= (e_acc << (8 * e_n)) | etk;
				e_fillhit <= 1'b0;
				e_x.hit   <= 1'b1;
				e_x.way   <= f_way;
				e_st      <= E_S_NEXT;
			end
			else if (e_fromline) begin
				// an inhibited fill delivered the line: the operand from it
				e_acc <= (e_acc << (8 * e_n)) | etk;
				e_fromline <= 1'b0;
				e_x.hit <= 1'b0;
				e_x.cm  <= 2'b11;
				e_st    <= E_S_NEXT;
			end
			else if (rd && !e_part && (m2.r.ack == 2'd0) && (e_x.cm[1] || m2.r.lock) && dm_older) begin
				// a serialized read (a noncachable serialized page, or a
				// locked access) waits until every earlier instruction has
				// completed: pending writes are on the bus first, and
				// nothing older can abort the instruction after the read
				// (MC68040UM 4.3.2, 7.7).  So does any cache-inhibited
				// read: the 68040 never reads a device on a path an older
				// instruction then discards (a mispredicted branch, a
				// serializing MOVEC) -- such a read cannot be taken back.
				// Retry from the start.
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (move16 && m2.r.mem == M_LD && e_x.hit && dc_en) begin
				// MOVE16 source hit: read from the cache, no allocation
				e_st <= E_S_RD;
			end
			else if (move16 && m2.r.mem == M_ST) begin
				// MOVE16 destination: a write hit invalidates the line (the
				// write replaces all of it, 4.3.2) -- unless the data cache
				// is disabled, which leaves its contents alone
				if (e_x.hit && dc_en) begin
					vp_set = e_va[9:4]; vp_wm[e_x.way] = 1'b1;
					vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
					e_x.hit <= 1'b0;
				end
				e_st <= E_S_NEXT;
			end
			else if (m2.r.mem == M_ST) begin
				// a store: a copyback miss allocates the line for WB
				if (alloc && e_x.cm == 2'b01 && !e_x.hit) begin
					f_set  <= e_va[9:4];
					f_tag  <= e_x.pa[31:10];
					f_way  <= victim(lve_q, rrc);
					e_sret <= E_S_XL;
					e_st   <= E_FILL;
				end
				else e_st <= E_S_NEXT;
			end
			else if (cach && e_x.hit && rd) begin
				// older stores to this line must reach the RAM first: give
				// the engine back to them and retry (the line of the part
				// being read: a split's second part has its own)
				if (e_part ? (st_line(m3, e_x.pa[31:4]) || st_line(m4, e_x.pa[31:4])) : hz) begin
					steal <= 1'b0; e_st <= E_IDLE;
				end
				else e_st <= E_S_RD;
			end
			else if (alloc && rd && !e_x.hit &&
			         (e_part ? (st_line(m3, e_x.pa[31:4]) || st_line(m4, e_x.pa[31:4])) : hz)) begin
				// an older store to this line has not reached memory yet
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (alloc && rd && !e_x.hit) begin
				f_set  <= e_va[9:4];
				f_tag  <= e_x.pa[31:10];
				f_way  <= victim(lve_q, rrc);
				e_sret <= E_S_XL;
				e_st   <= E_FILL;
			end
			else if (rd) begin
				// cache inhibited, no-allocate miss, locked, MOVE16 miss,
				// IACK: a matching line is pushed (if dirty) and invalidated
				// first (4.3.2, 4.6.2; with LOCK for a locked access); the
				// bus access goes in program order.  With the data cache
				// disabled (CACR.DE clear, CDIS) the cache's contents are
				// left alone
				if (e_x.hit && dc_en && (m2.r.lock || e_x.cm[1] || move16)) begin
					f_set  <= e_va[9:4];
					f_way  <= e_x.way;
					pv_lock <= m2.r.lock;
					pv_for  <= 2'd0;
					e_sret <= E_S_ACT;
					e_x.hit <= 1'b0;
					e_st   <= E_PUSHV;
				end
				else if ((m3.r.v && m3.r.mem != M_LD) || (m4.r.v && m4.r.mem != M_LD)) begin
					// older stores go to the bus first (program order): the
					// engine is theirs until they are done, then retry
					steal <= 1'b0;
					e_st  <= E_IDLE;
				end
				else begin
					bo.line  <= move16;
					bo.rd    <= 1'b1;
					bo.tt    <= move16 ? TT_MOVE16 : fc_tt(m2.r.fc);
					bo.tm    <= fc_tm(m2.r.fc);
					bo.upa   <= e_x.upa;
					bo.ci    <= e_x.cm[1];
					bo.lock  <= m2.r.lock;
					bo.locke <= 1'b0;
					bo.ack   <= m2.r.ack;
					bo.tln   <= 2'd0;
					bo_pa    <= move16 ? {e_x.pa[31:4], 4'd0} : e_x.pa;
					bo_left  <= move16 ? 3'd4 : e_n;
					e_ret    <= E_S_BUSD;
					e_st     <= E_BUS;
				end
			end
			else e_st <= E_S_NEXT;
		end
		E_S_RD: begin
			// read the line through the stolen data port (the shared line
			// multiplexer selects f_way: the hit way's, for the operand
			// and for a MOVE16 source)
			steal  <= 1'b1;
			st_set <= e_va[9:4];
			f_way  <= e_x.way;
			e_st   <= E_S_RDW;
		end
		E_S_RDW: e_st <= E_S_RD2;
		E_S_RD2: begin
			if (m2.r.msz == SZ_Q) mv_line <= dq_w;
			else e_acc <= (e_acc << (8 * e_n)) | etk;
			e_st <= E_S_NEXT;
		end
		E_S_BUSD: begin
			// the bus read finished
			// (an acknowledge cycle's TEA is no access error: IACK takes the
			// spurious vector, BKPT the illegal instruction exception)
			if (bo_err && (m2.r.ack == 2'd0)) begin
				// FA is the operand's first byte even when a later part
				// faulted (MA marks an ATC fault only, 8.4.6.2)
				e_flt   <= 1'b1;
				e_faddr <= m2.r.a;
				e_fssw  <= mk_ssw(1'b0, m2.r.lock, 1'b1, m2.r.msz, m2.r.fc);
				e_st    <= E_S_DONE;
			end
			else begin
				if (m2.r.ack == 2'd1)
					// IACK: [7:0] vector, [8] AVEC, [9] TEA = spurious
					e_acc <= {22'd0, bo_err, bo_avec && !bo_err, bo_rd[7:0]};
				else if (m2.r.msz == SZ_Q)
					mv_line <= bo_line;
				else
					e_acc <= (e_acc << (8 * e_n)) | bo_rd;
				e_st <= E_S_NEXT;
			end
		end
		E_S_NEXT: begin
			if (m2.split && !e_part) begin
				m2.x   <= e_x;                 // part 0, for WB
				e_part <= 1'b1;
				e_va   <= {m2.r.a[31:4] + 28'd1, 4'd0};
				e_n    <= nbytes(m2.r.msz) - e_n;
				e_st   <= E_S_XL;
			end
			else begin
				if (m2.split) m2.x1 <= e_x; else m2.x <= e_x;
				e_st <= E_S_DONE;
			end
		end
		E_S_DONE: begin
			e_job      <= 1'b0;
			e_dc2_done <= !e_kill;
			// synthesis translate_off
			// a job may only complete for a uop still in DC2: a discarded
			// one carries e_kill (the bench fails the run on this)
			if (!e_kill && !m2.r.v) begin
				$display("FAIL: DMU: the slow-path job completed for a discarded DC2 uop without its kill mark (e_va=%08x)", e_va);
				sim_kchk <= 1'b1;
			end
			// synthesis translate_on
			e_ldata    <= e_acc;
			steal      <= 1'b0;
			e_st       <= E_IDLE;
			// a push this job made ended in TEA: the processor takes an
			// access error at once (7.6.1); the frame reports the push
			if (pbe_r && pv_for == 2'd0) begin
				e_flt   <= 1'b1;
				e_faddr <= pbe_a;
				e_fssw  <= SSW_PUSH | (pv_lock ? 16'h0200 : 16'h0000);
				pbe_r   <= 1'b0;
			end
		end

		//==============================================================
		// WB store through the engine (write-through, cache-inhibited,
		// locked, split, MOVE16)
		//==============================================================
		E_W_START: begin
			e_part <= 1'b0;
			e_acc  <= lalign(st_data, m4.r.msz);
			e_va   <= m4.r.a;
			e_x    <= m4.x;
			e_n    <= m4.split ? 3'(5'd16 - {1'b0, m4.r.a[3:0]}) : nbytes(m4.r.msz);
			bo_err <= 1'b0;
			e_st   <= E_W_PART;
		end
		E_W_PART: begin
			logic [15:0] be; logic upd;
			be  = bmask(e_va[3:0], e_n);
			upd = e_x.hit && dc_en && !e_x.cm[1] && !m4.r.lock && (m4.r.msz != SZ_Q);
			if (e_x.hit && dc_en && (m4.r.lock || e_x.cm[1])) begin
				// a locked write (TAS, CAS: noncachable, 7.4.1) and a
				// cache-inhibited write bypass the cache: a line they hit is
				// pushed if dirty and invalidated before the bus write
				// (4.3.2, 4.6.2) -- unless the data cache is disabled, which
				// leaves its contents alone
				f_set   <= e_va[9:4];
				f_way   <= e_x.way;
				pv_lock <= m4.r.lock;
				pv_for  <= 2'd1;
				e_x.hit <= 1'b0;
				e_sret  <= E_W_PART;
				e_st    <= E_PUSHV;
			end
			else begin
			if (upd) begin
				dw      <= 1'b1;
				dw_set  <= e_va[9:4];
				dw_way  <= e_x.way;
				dw_be   <= be;
				dw_src  <= 2'd0;
				dw_word <= lanes32(e_acc, e_va[1:0]);
				if (e_x.cm == 2'b01)
					if (be != 16'd0) begin
						vp_set = e_va[9:4]; vp_wm[e_x.way] = 1'b1; vp_dwe = 1'b1; vp_dv = 1'b1;
					end
			end
			if (upd && e_x.cm == 2'b01) begin
				e_st <= E_W_DONE;            // copyback: no bus write
			end
			else begin
				bo.line  <= (m4.r.msz == SZ_Q);
				bo.rd    <= 1'b0;
				bo.tt    <= (m4.r.msz == SZ_Q) ? TT_MOVE16 : fc_tt(m4.r.fc);
				bo.tm    <= fc_tm(m4.r.fc);
				bo.upa   <= e_x.upa;
				bo.ci    <= e_x.cm[1];
				bo.lock  <= m4.r.lock;
				bo.locke <= m4.r.locke && (!m4.split || e_part);
				bo.ack   <= m4.r.ack;
				bo.tln   <= 2'd0;
				bo_pa    <= (m4.r.msz == SZ_Q) ? {e_x.pa[31:4], 4'd0} : e_x.pa;
				bo_left  <= (m4.r.msz == SZ_Q) ? 3'd4 : e_n;
				bo_wd    <= e_acc;
				bo_line  <= mv_line;
				e_ret    <= E_W_DONE;
				e_st     <= E_BUS;
			end
			end
		end
		E_W_DONE: begin
			// a bus error on the write, or on a push the write made (7.6.1):
			// an access error
			if (bo_err || (pbe_r && pv_for == 2'd1)) begin
				st_rdy_r     <= 1'b1;
				st_fault_r   <= 1'b1;
				st_fssw    <= bo_err ? mk_ssw(1'b0, m4.r.lock, 1'b0, m4.r.msz, m4.r.fc)
				                     : (SSW_PUSH | (pv_lock ? 16'h0200 : 16'h0000));
				st_faddr   <= bo_err ? m4.r.a : pbe_a;
				if (!bo_err) pbe_r <= 1'b0;
				e_wst_done <= 1'b1;
				e_st       <= E_IDLE;
			end
			else if (m4.split && !e_part) begin
				e_part <= 1'b1;
				e_acc  <= e_acc << (8 * e_n);
				e_va   <= {m4.r.a[31:4] + 28'd1, 4'd0};
				e_x    <= m4.x1;
				e_n    <= nbytes(m4.r.msz) - e_n;
				e_st   <= E_W_PART;
			end
			else begin
				st_rdy_r     <= 1'b1;
				st_fault_r   <= 1'b0;
				e_wst_done <= 1'b1;
				e_st       <= E_IDLE;
			end
		end

		//==============================================================
		// maintenance at WB
		//==============================================================
		E_M_START: begin
			mt_fault <= 1'b0;
			case (mt_op)
				MT_PFLUSH, MT_PTEST: begin
					// both ATCs; by page, or all (N: spare global entries).
					// PTEST first discards the page's entry in both ATCs.
					atc_fng       <= (mt_op == MT_PFLUSH) && mt_ng;
					iatc_flush_ng <= (mt_op == MT_PFLUSH) && mt_ng;
					if (mt_op == MT_PFLUSH && mt_scope == 2'd3) begin
						atc_fall       <= 1'b1;
						iatc_flush_all <= 1'b1;
						e_st <= E_M_DONE;
					end
					else begin
						steal  <= 1'b1;
						st_la  <= mt_addr;
						st_fc2 <= mt_fc[2];
						iatc_flush_la  <= mt_addr;
						iatc_flush_fc2 <= mt_fc[2];
						e_st <= E_M_PG;
					end
				end
				default: begin
					// CINV / CPUSH
					if (!mt_caches[0]) e_st <= E_M_IC;
					else if (mt_op == MT_CINV && mt_scope == 2'd3) begin
						clr_i <= '0;
						e_st  <= E_M_CLR;
					end
					else begin
						// scan one set (line), or every set (page, all)
						ms_i <= (mt_scope == 2'd1) ? {mt_addr[9:4], 2'b00} : 8'd0;
						e_st <= E_M_SCAN;
					end
				end
			endcase
		end
		E_M_CLR: begin
			// every line invalid, a set a cycle (CINVA; and after reset)
			vp_set = clr_i; vp_wm = 4'hF;
			vp_lwe = 1'b1; vp_lv = 1'b0;
			clr_i <= clr_i + 6'd1;
			if (clr_i == 6'd63) begin
				clr_rst <= 1'b0;
				e_st    <= clr_rst ? E_IDLE : E_M_IC;
			end
		end
		E_M_PG: begin
			// the stolen ATC read of the page is valid now: flush it.  The
			// flush pulse compares the lookup port in the NEXT cycle, so the
			// port stays stolen (E_M_DONE / the walk's end release it)
			atc_fpage       <= 1'b1;
			iatc_flush_page <= 1'b1;
			if (mt_op == MT_PTEST) begin
				// a transparent translation answers without a search
				logic t0, t1, it0, it1, hitt, wpt, isp;
				isp = (mt_fc[1:0] == 2'b10);
				t0  = ttr_hit(dtt0, mt_addr, mt_fc[2]);
				t1  = ttr_hit(dtt1, mt_addr, mt_fc[2]);
				it0 = ttr_hit(itt0, mt_addr, mt_fc[2]);
				it1 = ttr_hit(itt1, mt_addr, mt_fc[2]);
				hitt = isp ? (it0 || it1) : (t0 || t1);
				wpt  = isp ? (it0 ? itt0[2] : itt1[2]) : (t0 ? dtt0[2] : dtt1[2]);
				if (hitt) begin
					mt_mmusr <= (mt_wr && wpt) ? 32'h0000_0800 : 32'h0000_0003;
					e_st <= E_M_DONE;
				end
				else begin
					tw_va <= mt_addr; tw_s <= mt_fc[2]; tw_wr <= mt_wr;
					tw_pt <= 1'b1; tw_i <= isp;
					tw_upd <= 1'b0;              // (the page's entry was just discarded)
					e_ret <= E_M_DONE;
					e_st  <= E_TW_START;
				end
			end
			else e_st <= E_M_DONE;
		end
		E_M_SCAN: begin
			tset_b <= ms_i[7:2];
			steal  <= 1'b1;
			st_set <= ms_i[7:2];
			sc_set <= ms_i[7:2];
			e_st   <= E_M_SCANW;
		end
		E_M_SCANW: begin
			// (a write this cycle reaches the copy at the write port)
			sc_lv <= lve_q;
			sc_ld <= ld_q;                 // (ld_ra: sc_set)
			e_st <= E_M_SCAN2;
		end
		E_M_SCAN2: begin
			logic [5:0] s; logic [1:0] w; logic m;
			s = ms_i[7:2]; w = ms_i[1:0];
			m = sc_lv[w] && ((mt_scope == 2'd3) ||
			                 (mt_scope == 2'd1 && tq_b[w] == mt_addr[31:10]) ||
			                 (mt_scope == 2'd2 && (tc_p ? tq_b[w][21:3] == mt_addr[31:13]
			                                            : tq_b[w][21:2] == mt_addr[31:12])));
			if (m && mt_op == MT_CPUSH && sc_ld[w]) begin
				// push (it is invalidated with the push)
				f_set   <= s;
				f_way   <= w;
				pv_lock <= 1'b0;
				pv_for  <= 2'd2;
				e_sret  <= E_M_SCAN;         // reread the set's tags after
				e_st    <= E_PUSHV;
			end
			else begin
				if (m) begin
					vp_set = s; vp_wm[w] = 1'b1;
					vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
				end
				if ((mt_scope == 2'd1 && w == 2'd3) || ms_i == 8'hFF) e_st <= E_M_IC;
				else begin
					ms_i <= ms_i + 8'd1;
					e_st <= (w == 2'd3) ? E_M_SCAN : E_M_SCAN2;
				end
			end
		end
		E_M_IC: begin
			// the instruction cache part of CINV/CPUSH
			steal <= 1'b0;
			if (mt_caches[1]) begin
				ic_inv       <= 1'b1;
				ic_inv_scope <= mt_scope;
				ic_inv_pa    <= mt_addr;
				if (ic_inv_done) begin
					ic_inv <= 1'b0;
					e_st   <= E_M_DONE;
				end
			end
			else e_st <= E_M_DONE;
		end
		E_M_DONE: begin
			mt_done <= 1'b1;
			steal   <= 1'b0;
			e_st    <= E_IDLE;
			// a push of CPUSH's ended in TEA: an access error (7.6.1)
			if (pbe_r && pv_for == 2'd2) begin
				mt_fault <= 1'b1;
				mt_fssw  <= SSW_PUSH;
				mt_faddr <= pbe_a;
				pbe_r    <= 1'b0;
			end
		end

		//==============================================================
		// sub-step: line fill of (f_set, f_way) with tag f_tag, critical
		// long word first.  The victim goes to the push buffer if dirty
		// and is pushed after the fill (4.6.2).  Returns to e_sret.
		//==============================================================
		E_FILL: begin
			if (store_holds(f_set, f_way, m3, m4))
				f_way <= f_way + 2'd1;      // that line belongs to a store
			else begin
				steal  <= 1'b1;
				st_set <= f_set;
				tset_b <= f_set;
				e_st   <= E_FILLW;
			end
		end
		E_FILLW: e_st <= E_FILL_W;
		E_FILL_W: begin
			if (lve_q[f_way] && ld_q[f_way]) begin     // (ld_ra, lv_ra: f_set)
				pv_v     <= 1'b1;
				pv_line  <= dq_w;
				pv_lpa   <= {tq_b[f_way], f_set};
				pv_dirty <= 1'b1;
				pv_tln   <= f_way;
				pv_lock  <= 1'b0;
				pv_for   <= 2'd0;
			end
			vp_set = f_set; vp_wm[f_way] = 1'b1;
			vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
			bo.line <= 1'b1; bo.rd <= 1'b1; bo.tt <= TT_NORMAL;
			bo.tm <= m2.r.smode ? TM_SDATA : TM_UDATA;
			bo.upa <= e_x.upa; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
			bo.ack <= 2'd0; bo.tln <= f_way;
			bo_pa   <= {f_tag, f_set, e_va[3:2], 2'b00};
			bo_left <= 3'd4;
			e_ret   <= E_FILL_INS;
			e_st    <= E_BUS;
		end
		E_FILL_INS: begin
			if (!bo_err && !bo_tci) begin
				dw      <= 1'b1;
				dw_set  <= f_set;
				dw_way  <= f_way;
				dw_be   <= 16'hFFFF;
				dw_src  <= 2'd1;
				tw_b    <= 1'b1;
				tset_b  <= f_set;
				tway_b  <= f_way;
				twd_b   <= f_tag;
				vp_set = f_set; vp_wm[f_way] = 1'b1;
				vp_lwe = 1'b1; vp_lv = 1'b1; vp_dwe = 1'b1; vp_dv = 1'b0;
				rrc <= rrc + 2'd1;
			end
			else if (pv_v) begin
				// failed or inhibited fill: the dirty victim is restored
				dw      <= 1'b1;
				dw_set  <= f_set;
				dw_way  <= f_way;
				dw_be   <= 16'hFFFF;
				dw_src  <= 2'd2;
				vp_set = f_set; vp_wm[f_way] = 1'b1;
				vp_lwe = 1'b1; vp_lv = 1'b1; vp_dwe = 1'b1; vp_dv = pv_dirty;
				pv_v <= 1'b0;
			end
			if (bo_err && fill_late_ok) begin
				// a later beat of the line failed, but what the access needs
				// arrived (MC68040UM 7.6.1, 4.6.1): no exception, the line not
				// allocated; a load takes its operand from what was read, a
				// store goes to memory (as for TCI)
				e_fromline <= (m2.r.mem != M_ST);
				e_x.cm     <= 2'b11;
				e_st       <= (m2.r.mem != M_ST) ? E_S_ACT : E_S_NEXT;
			end
			else if (bo_err) begin
				// the operand's long word did not arrive: an access fault
				e_flt   <= 1'b1;
				e_faddr <= m2.r.a;
				e_fssw  <= mk_ssw(1'b0, 1'b0, m2.r.mem == M_LD, m2.r.msz, m2.r.fc);
				e_st    <= E_S_DONE;
			end
			else if (bo_tci) begin
				// not cachable after all: the operand from the line read,
				// a store of this page writes through
				e_fromline <= (m2.r.mem != M_ST);
				e_x.cm     <= 2'b11;
				e_st       <= (m2.r.mem != M_ST) ? E_S_ACT : E_S_NEXT;
			end
			else if (pv_v) e_st <= E_PUSHV_B;
			else if (m2.r.mem == M_LD && e_sret == E_S_XL) begin
				// a load's own fill: its operand from the line just read
				// (still in bo_line), not a second lookup of that line
				e_fillhit <= 1'b1;
				e_st      <= E_S_ACT;
			end
			else e_st <= e_sret;
		end

		//==============================================================
		// sub-step: push (f_set, f_way) if dirty and invalidate it
		// (E_PUSHV), or push the buffered victim (E_PUSHV_B).  Returns
		// to e_sret.
		//==============================================================
		E_PUSHV: begin
			steal  <= 1'b1;
			st_set <= f_set;
			tset_b <= f_set;
			e_st   <= E_PUSHVW;
		end
		E_PUSHVW: e_st <= E_PUSHV_W;
		E_PUSHV_W: begin
			if (lve_q[f_way] && ld_q[f_way]) begin     // (ld_ra, lv_ra: f_set)
				pv_v     <= 1'b1;
				pv_line  <= dq_w;
				pv_lpa   <= {tq_b[f_way], f_set};
				pv_dirty <= 1'b1;
				pv_tln   <= f_way;
				e_st     <= E_PUSHV_B;
			end
			else e_st <= e_sret;
			vp_set = f_set; vp_wm[f_way] = 1'b1;
			vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
		end
		E_PUSHV_B: begin
			// (LOCK on a push inside a locked sequence, 4.6.2)
			bo.line <= 1'b1; bo.rd <= 1'b0; bo.tt <= TT_NORMAL; bo.tm <= TM_PUSH;
			bo.upa <= 2'd0; bo.ci <= 1'b0; bo.lock <= pv_lock; bo.locke <= 1'b0;
			bo.ack <= 2'd0; bo.tln <= pv_tln;
			bo_pa   <= {pv_lpa, 4'd0};
			bo_left <= 3'd4;
			bo_line <= pv_line;
			pv_v    <= 1'b0;
			e_ret   <= e_sret;
			e_st    <= E_BUS;
		end

		//==============================================================
		// sub-step: bus operation (pieces / line / IACK); returns to e_ret
		//==============================================================
		E_BUS: begin
			bo_err <= 1'b0;
			bo_got <= 4'd0;
			bo_tci <= 1'b0;
			bo_avec <= 1'b0;
			bo_rd  <= '0;
			bo_ingnt <= 1'b0;
			if (bo_cancel) begin
				// a snoop invalidated the line this push carries
				bo_cancel <= 1'b0;
				e_st      <= e_ret;
			end
			else begin
				e_breq <= 1'b1;
				e_st   <= E_BUS_W;
			end
		end
		E_BUS_W: begin
			if (bo_cancel && !b_gnt && !bo_ingnt) begin
				// not on the bus yet (the snooped master has it): dropped
				bo_cancel <= 1'b0;
				e_breq    <= 1'b0;
				e_st      <= e_ret;
			end
			else if (b_gnt) begin
				e_breq <= 1'b0;
				bo_pn  <= bo_np;
				bo_ingnt <= 1'b1;
			end
			// (granted, then retried with the bus given up: the BIU drops
			// it on bi_drop and reports it done)
			if (b_err || b_done) begin
				bo_ingnt  <= 1'b0;
				bo_cancel <= 1'b0;
			end
			if (b_rvalid) begin
				if (bo.line) begin
					bo_line[127 - 32*b_rbeat -: 32] <= b_rdata;
					bo_got[b_rbeat] <= 1'b1;
					if (b_rbeat == bo_pa[3:2]) bo_tci <= b_rtci;
				end
				else begin
					bo_rd   <= (bo_rd << (8 * bo_pn)) | grab(b_rdata, bo_pn, bo_pa[1:0]);
					bo_avec <= b_ravec;
				end
			end
			if (b_err) begin
				// a push that ends in TEA: recorded for the access error the
				// job it served takes (7.6.1); the line is gone.  A walk's
				// (a descriptor line pushed before its update) fails the
				// search as a bus error.
				if (bo.line && !bo.rd && bo.tm == TM_PUSH) begin
					if (pv_for == 2'd3) begin tw_fail <= 1'b1; tw_berr <= 1'b1; end
					else begin pbe_r <= 1'b1; pbe_a <= bo_pa; end
				end
				else bo_err <= 1'b1;
				e_st   <= e_ret;
			end
			else if (b_done) begin
				if (bo.line || bo.ack != 2'd0) e_st <= e_ret;
				else begin
					if (!bo.rd) bo_wd <= bo_wd << (8 * bo_pn);
					bo_pa   <= bo_pa + {29'd0, bo_pn};
					bo_left <= bo_left - bo_pn;
					if (bo_left == bo_pn) e_st <= e_ret;
					else e_breq <= 1'b1;
				end
			end
		end

		//==============================================================
		// table walk (MC68040UM 3.2); the entry goes into the ATC that
		// tw_i selects; returns to e_ret
		//==============================================================
		E_TW_START: begin
			tw_lvl  <= TW_ROOT;
			tw_wp   <= 1'b0;
			tw_fail <= 1'b0;
			tw_berr <= 1'b0;
			tw_lkrd <= 1'b0;
			tw_da   <= {(tw_s ? srp[31:9] : urp[31:9]), 9'd0} + {23'd0, tw_ri, 2'b00};
			e_st    <= E_TW_DESC;
		end
		E_TW_DESC: begin
			// descriptor read: from the data cache if the line is there
			// (cachable write-through, no allocate, 3.2.5), else from the bus
			steal  <= 1'b1;
			st_set <= tw_da[9:4];
			tset_b <= tw_da[9:4];
			e_st   <= E_TW_DESCW;
		end
		// The tag RAM registers its port B address: tset_b lands at the end
		// of E_TW_DESC, the RAM takes it at the end of E_TW_DESCW, its tags
		// are on tq_b in E_TW_DESCW2, and the compare registered at the end
		// of that cycle (tw_hhq, tw_hwq) is what E_TW_DLK reads.  One wait
		// state fewer sampled the compare against the set addressed BEFORE
		// (the previous descriptor's, or the pipeline's): a false hit took
		// another line's data for the descriptor, a false miss read memory
		// past a dirty cached descriptor (the NeXT kernel's page-fault loop
		// of 2026-10-19, tb/asm/t_nx8k.s).
		E_TW_DESCW:  e_st <= E_TW_DESCW2;
		E_TW_DESCW2: e_st <= E_TW_DLK;    // (the tags of the set valid: the compare registers)
		E_TW_DLK: begin
			if (tw_hhq && dc_en) begin
				tw_d <= take(dq_w, tw_da[3:0], 3'd4);
				e_st <= E_TW_EVAL;
			end
			else begin
				bo.line <= 1'b0; bo.rd <= 1'b1; bo.tt <= TT_NORMAL;
				bo.tm   <= tw_i ? TM_TBL_CODE : TM_TBL_DATA;
				bo.upa  <= 2'd0; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
				bo.ack  <= 2'd0; bo.tln <= 2'd0;
				bo_pa   <= tw_da;
				bo_left <= 3'd4;
				e_wret  <= e_ret;
				e_ret   <= E_TW_DBUS;
				e_st    <= E_BUS;
			end
		end
		E_TW_DBUS: begin
			e_ret <= e_wret;
			if (bo_err) begin
				tw_fail <= 1'b1;
				tw_berr <= 1'b1;
				e_st    <= E_TW_DONE;
			end
			else begin
				tw_d <= bo_rd;
				e_st <= E_TW_EVAL;
			end
		end
		E_TW_EVAL: begin
			case (tw_lvl)
				TW_ROOT, TW_PTR: begin
					if (!tw_d[1]) begin
						tw_fail <= 1'b1;           // invalid table descriptor
						e_st    <= E_TW_DONE;
					end
					else begin
						tw_wp <= tw_wp | tw_d[2];
						if (!tw_d[3]) e_st <= E_TW_UPD;   // set U
						else begin
							if (tw_lvl == TW_ROOT) begin
								tw_lvl <= TW_PTR;
								tw_da  <= {tw_d[31:9], 9'd0} + {23'd0, tw_pi, 2'b00};
							end
							else begin
								tw_lvl <= TW_PAGE;
								tw_da  <= (tc_p ? {tw_d[31:7], 7'd0} : {tw_d[31:8], 8'd0}) +
								          {24'd0, tw_gi, 2'b00};
							end
							e_st <= E_TW_DESC;
						end
					end
				end
				default: begin
					// page descriptor (or the one an indirect points to)
					if (tw_d[1:0] == 2'b00 || (tw_lvl == TW_IND && tw_d[1:0] == 2'b10)) begin
						tw_fail <= 1'b1;
						e_st    <= E_TW_DONE;
					end
					else if (tw_d[1:0] == 2'b10) begin
						tw_lvl <= TW_IND;
						tw_da  <= {tw_d[31:2], 2'b00};
						e_st   <= E_TW_DESC;
					end
					else begin
						logic wp, sv, setm;
						wp   = tw_wp | tw_d[2];
						sv   = tw_d[7] && !tw_s;
						// PTESTW: a probe sets M only if the write is permitted
						setm = tw_wr && !wp && !sv && !tw_d[4];
						tw_pd <= tw_d;
						tw_wp <= wp;
						if (!tw_d[3] || setm) e_st <= E_TW_UPD;
						else e_st <= E_TW_DONE;
					end
				end
			endcase
		end
		E_TW_UPD: begin
			// descriptor update, MC68040UM table 3-1: with U clear a locked
			// read-modify-write sets U (the descriptor read again with LOCK,
			// E_TW_LKW, and written back with LOCK and LOCKE), except a
			// permitted write to a clean page (U and M clear), which is a
			// plain write of U and M; U set and M to set: a plain write.
			// The update is noncachable (3.2.5): a cached copy of the
			// descriptor's line is pushed if dirty (with LOCK for a locked
			// update, 4.6.2) and invalidated first.
			logic page, sv, setm, needu, lk, h;
			logic [1:0] w;
			logic [31:0] nd;
			est_t caller;
			page  = (tw_lvl == TW_PAGE || tw_lvl == TW_IND);
			sv    = page && tw_d[7] && !tw_s;
			setm  = page && tw_wr && !tw_wp && !sv && !tw_d[4];
			needu = !tw_d[3];
			lk    = needu && !setm;
			nd    = tw_d | 32'h8 | (setm ? 32'h10 : 32'h0);
			// (back from the push: e_ret is this state, the caller in e_wret)
			caller = (e_ret == E_TW_UPD) ? e_wret : e_ret;
			h = tw_hhq; w = tw_hwq;           // (registered the cycle before, lv_ra: tw_da)
			if (!tw_hvq) ;                    // (a snoop had the set: the compare registers now)
			else begin
			e_wret <= caller;
			if (h && ld_q[w]) begin
				// a dirty copy: pushed (E_PUSHV invalidates it), then back here
				f_set   <= tw_da[9:4];
				f_way   <= w;
				pv_lock <= lk;
				pv_for  <= 2'd3;
				e_sret  <= E_TW_UPD;
				e_st    <= E_PUSHV;
			end
			else begin
				if (h) begin
					vp_set = tw_da[9:4]; vp_wm[w] = 1'b1;
					vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
				end
				bo.line <= 1'b0; bo.tt <= TT_NORMAL;
				bo.tm   <= tw_i ? TM_TBL_CODE : TM_TBL_DATA;
				bo.upa  <= 2'd0; bo.ci <= 1'b1;
				bo.ack  <= 2'd0; bo.tln <= 2'd0;
				bo_pa   <= tw_da;
				bo_left <= 3'd4;
				e_st    <= E_BUS;
				if (lk) begin
					bo.rd   <= 1'b1;
					bo.lock <= 1'b1; bo.locke <= 1'b0;
					e_ret   <= E_TW_LKW;
				end
				else begin
					tw_d  <= nd;
					if (page) tw_pd <= nd;
					bo.rd   <= 1'b0;
					bo.lock <= 1'b0; bo.locke <= 1'b0;
					bo_wd   <= nd;
					e_ret   <= E_TW_UPD_W;
				end
			end
			end
		end
		E_TW_LKW: begin
			// the locked read is in: U into it, the locked write ends it
			logic [31:0] nd;
			nd = bo_rd | 32'h8;
			if (bo_err) begin
				e_ret   <= e_wret;
				tw_fail <= 1'b1;
				tw_berr <= 1'b1;
				e_st    <= E_TW_DONE;
			end
			else begin
				tw_d <= nd;
				if (tw_lvl == TW_PAGE || tw_lvl == TW_IND) tw_pd <= nd;
				bo.rd   <= 1'b0;
				bo.lock <= 1'b1; bo.locke <= 1'b1;
				bo_pa   <= tw_da;
				bo_left <= 3'd4;
				bo_wd   <= nd;
				e_ret   <= E_TW_UPD_W;
				e_st    <= E_BUS;
			end
		end
		E_TW_UPD_W: begin
			e_ret <= e_wret;
			if (bo_err) begin
				tw_fail <= 1'b1;
				tw_berr <= 1'b1;
				e_st    <= E_TW_DONE;
			end
			else if (tw_lvl == TW_ROOT) begin
				tw_lvl <= TW_PTR;
				tw_da  <= {tw_d[31:9], 9'd0} + {23'd0, tw_pi, 2'b00};
				e_st   <= E_TW_DESC;
			end
			else if (tw_lvl == TW_PTR) begin
				tw_lvl <= TW_PAGE;
				tw_da  <= (tc_p ? {tw_d[31:7], 7'd0} : {tw_d[31:8], 8'd0}) +
				          {24'd0, tw_gi, 2'b00};
				e_st   <= E_TW_DESC;
			end
			else e_st <= E_TW_DONE;
		end
		E_TW_DONE: begin
			atce_t e;
			e = '0;
			if (!tw_fail) begin
				e.pa  = tc_p ? {tw_pd[31:13], 1'b0} : tw_pd[31:12];
				e.g   = tw_pd[10];
				e.upa = tw_pd[9:8];
				e.s   = tw_pd[7];
				e.cm  = tw_pd[6:5];
				e.m   = tw_pd[4];
				e.w   = tw_wp;
				e.r   = 1'b1;
			end
			e.b = tw_berr;
			// install (failed searches too, as the 68040 does: a nonresident
			// entry, R clear, B for a bus error)
			if (tw_i) begin
				iatc_wr   <= 1'b1;
				iatc_wla  <= tw_va;
				iatc_wfc2 <= tw_s;
				iatc_went <= e;
			end
			else begin
				atc_wr   <= 1'b1;
				atc_wla  <= tw_va;
				atc_wfc2 <= tw_s;
				atc_went <= e;
			end
			iw_ent <= e;
			if (tw_pt) begin
				// MMUSR: page frame (8K: bit 12 clear), B G U1 U0 S CM M - W T R
				mt_mmusr <= tw_berr ? 32'h0000_0800 :
				            tw_fail ? 32'h0000_0000 :
				            ((tc_p ? {tw_pd[31:13], 13'd0} : {tw_pd[31:12], 12'd0}) |
				             {21'd0, tw_pd[10], tw_pd[9:8], tw_pd[7], tw_pd[6:5],
				              tw_pd[4], 1'b0, tw_wp, 1'b0, 1'b1});
			end
			steal <= 1'b0;
			e_st  <= e_ret;
		end
		E_I_DONE: begin
			iw_done <= 1'b1;
			e_st    <= E_IDLE;
		end

		default: e_st <= E_IDLE;
		endcase

		if ((e_st == E_SP_W || e_st == E_SP_2) && dw && dw_set == st_set) sp_dw <= 1'b1;

		if (kill_now) begin
			// a kill comes with the WB uop leaving (or WB empty): every
			// record goes, including one EX moved into WB this cycle
			m1.v   <= 1'b0;
			m2.r.v <= 1'b0;
			m3.r.v <= 1'b0;
			m4.r.v <= 1'b0;
			e_dc2_done <= 1'b0;
			if (e_st != E_IDLE) e_kill <= 1'b1;
			if (e_st == E_IDLE) e_job <= 1'b0;
		end

		//--------------------------------------------------------------
		// snoop port
		//--------------------------------------------------------------
		sn_look <= 1'b0;
		case (sn_ph)
		SN_IDLE: if (sn_req) sn_ph <= SN_A;
		// the RAMs register the snooped set; a data or tag write issued
		// just before the freeze lands in this cycle, and a read of the
		// same set would miss it: present the set once more
		SN_A:    if (!dw && !tw_b) sn_ph <= SN_B;
		SN_B: begin
			logic [5:0] s;
			logic h; logic [1:0] w;
			s = sn_pa[9:4];
			h = 1'b0; w = 2'd0;
			for (int i = 0; i < 4; i++)
				if (lve_q[i] && tq_a[i] == sn_pa[31:10]) begin h = 1'b1; w = 2'(i); end
			sn_way <= w;
			sn_look <= 1'b1;
			sn_pvm <= pv_v && pv_lpa == sn_pa[31:4];
			sn_bom <= (e_st == E_BUS || e_st == E_BUS_W) && bo.line && !bo.rd &&
			          bo.tm == TM_PUSH && !bo_cancel && bo_pa[31:4] == sn_pa[31:4];
			if (h) begin
				sn_src   <= 2'd0;
				sn_hit   <= 1'b1;
				sn_dirty <= ld_q[w];          // (ld_ra: the snooped set)
			end
			else if (pv_v && pv_lpa == sn_pa[31:4]) begin
				// the dirty victim waiting for its push (4.7.2)
				sn_src   <= 2'd1;
				sn_hit   <= 1'b1;
				sn_dirty <= 1'b1;
			end
			else if ((e_st == E_BUS || e_st == E_BUS_W) && bo.line && !bo.rd &&
			         bo.tm == TM_PUSH && !bo_cancel && bo_pa[31:4] == sn_pa[31:4]) begin
				// a push queued for the bus
				sn_src   <= 2'd2;
				sn_hit   <= 1'b1;
				sn_dirty <= 1'b1;
			end
			else begin
				sn_hit   <= 1'b0;
				sn_dirty <= 1'b0;
			end
			sn_ph <= SN_H;
		end
		SN_H: begin
			logic [5:0] s;
			s = sn_pa[9:4];
			// (a line found in a way is invalidated below)
			if (sn_inv && sn_pvm) pv_v <= 1'b0;
			if (sn_inv && sn_bom) bo_cancel <= 1'b1;
			if (sn_wr) begin
				case (sn_src)
				2'd0: begin
					dw      <= 1'b1;
					dw_set  <= s;
					dw_way  <= sn_way;
					dw_be   <= sn_wbe;
					dw_src  <= 2'd0;
					dw_word <= sn_wword;
					vp_set = s; vp_wm[sn_way] = 1'b1; vp_dwe = 1'b1; vp_dv = 1'b1;
				end
				2'd1: for (int i = 0; i < 16; i++)
					if (sn_wbe[i]) pv_line[8*i +: 8] <= sn_wword[8*(i % 4) +: 8];
				default: for (int i = 0; i < 16; i++)
					if (sn_wbe[i]) bo_line[8*i +: 8] <= sn_wword[8*(i % 4) +: 8];
				endcase
			end
			if (!sn_req) sn_ph <= SN_R;
		end
		default: sn_ph <= SN_IDLE;   // SN_R: the owner's address is read again
		endcase
		// a snooped line leaves the cache (found in a way: invalidated)
		if (sn_ph == SN_H && sn_inv && sn_src == 2'd0) begin
			vp_set = sn_pa[9:4]; vp_wm[sn_way] = 1'b1;
			vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
			// (the records' hits on it: at the valid/dirty write port)
		end

		//--------------------------------------------------------------
		// a line leaving the cache -- whoever invalidates it: a snoop, an
		// inhibited or locked access's push, CINV/CPUSH -- leaves no record
		// with a hit on it: the stage records, those loaded this clock from
		// the stage before (DC1's lookup read the valid bits before this
		// write lands), both parts of a split, and the engine's own
		//--------------------------------------------------------------
		// (the record DC1 loads now loses its hit on any invalidation in its
		// set, whatever the way: its way and translation come late, and its
		// set is the untranslated page offset; a lost hit sends the access
		// to the engine, which looks the line up again)
		if (vp_lwe && !vp_lv && (|vp_wm) && adv_dc1 && m1.a[9:4] == vp_set) m2.x.hit <= 1'b0;
		if (vp_lwe && !vp_lv) begin
			for (int j = 0; j < 4; j++) if (vp_wm[j]) begin
				if (!adv_dc1) begin
					if (m2.x.hit && m2.x.pa[9:4] == vp_set && m2.x.way == 2'(j)) m2.x.hit <= 1'b0;
					if (m2.x1.hit && m2.x1.pa[9:4] == vp_set && m2.x1.way == 2'(j)) m2.x1.hit <= 1'b0;
				end
				if (adv_dc2) begin
					if (m2.x.hit && m2.x.pa[9:4] == vp_set && m2.x.way == 2'(j)) m3.x.hit <= 1'b0;
					if (m2.x1.hit && m2.x1.pa[9:4] == vp_set && m2.x1.way == 2'(j)) m3.x1.hit <= 1'b0;
				end
				else begin
					if (m3.x.hit && m3.x.pa[9:4] == vp_set && m3.x.way == 2'(j)) m3.x.hit <= 1'b0;
					if (m3.x1.hit && m3.x1.pa[9:4] == vp_set && m3.x1.way == 2'(j)) m3.x1.hit <= 1'b0;
				end
				if (adv_ex) begin
					if (m3.x.hit && m3.x.pa[9:4] == vp_set && m3.x.way == 2'(j)) m4.x.hit <= 1'b0;
					if (m3.x1.hit && m3.x1.pa[9:4] == vp_set && m3.x1.way == 2'(j)) m4.x1.hit <= 1'b0;
				end
				else begin
					if (m4.x.hit && m4.x.pa[9:4] == vp_set && m4.x.way == 2'(j)) m4.x.hit <= 1'b0;
					if (m4.x1.hit && m4.x1.pa[9:4] == vp_set && m4.x1.way == 2'(j)) m4.x1.hit <= 1'b0;
				end
				if (e_x.hit && e_x.pa[9:4] == vp_set && e_x.way == 2'(j)) e_x.hit <= 1'b0;
			end
		end

		//--------------------------------------------------------------
		// the valid/dirty write port
		//--------------------------------------------------------------
		begin
			if ((vp_dwe || vp_lwe) && vp_wm != 4'd0) begin
				logic [7:0] wo, wn;
				wo = lsw[vp_set];
				wn[7:4] = vp_dwe ? ((wo[7:4] & ~vp_wm) | (vp_wm & {4{vp_dv}})) : wo[7:4];
				wn[3:0] = vp_lwe ? ((wo[3:0] & ~vp_wm) | (vp_wm & {4{vp_lv}})) : wo[3:0];
				lsd[vp_set] <= wn;
				lse[vp_set] <= wn;
				lsw[vp_set] <= wn;
			end
			for (int j = 0; j < 4; j++)
				if (vp_wm[j]) begin
					// synthesis translate_off
					if (vp_lwe) lv[vp_set][j] <= vp_lv;
					// synthesis translate_on
					if (vp_lwe && vp_set == sc_set) sc_lv[j] <= vp_lv;
					if (vp_dwe && vp_set == sc_set) sc_ld[j] <= vp_dv;
				end
		end
	end
end

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && e_st == E_M_SCAN2 &&
	    (sc_lv != lv[sc_set] || (!sn_frz && (sc_lv & sc_ld) != (sc_lv & ld_q)))) begin
		$display("FAIL: the scan's valid/dirty copy %b %b, the set's %b", sc_lv, sc_ld, lv[sc_set]);
		$finish;
	end
// synthesis translate_on

// snoop: the line found, valid with sn_look (the RAM still reads the
// snooped set in the first H cycle)
assign sn_line = (sn_src == 2'd0) ? dq_w : (sn_src == 2'd1) ? pv_line : bo_line;

assign tw_busy = (e_st == E_TW_START) || (e_st == E_TW_DESC) || (e_st == E_TW_DESCW) || (e_st == E_TW_DESCW2) ||
                 (e_st == E_TW_DLK) || (e_st == E_TW_DBUS) || (e_st == E_TW_EVAL) ||
                 (e_st == E_TW_UPD) || (e_st == E_TW_LKW) || (e_st == E_TW_UPD_W) ||
                 (e_st == E_TW_DONE) ||
                 ((e_st == E_BUS || e_st == E_BUS_W) &&
                  (e_ret == E_TW_DBUS || e_ret == E_TW_LKW || e_ret == E_TW_UPD_W));

// DC2 answer
// once the engine has taken the DC2 uop, only its completion releases it
assign dc2_rdy = !m2.r.v || e_dc2_done || (!e_job && (fast_now || st_simple));
assign ldata   = e_dc2_done ? e_ldata : fast_data;
// for AG's capture of a load into a base or index (a short path): the long
// word of the line the address points at -- the operand itself for an
// aligned long or an even word (in the low half), else not offered
wire [31:0] al_word = dq2[63:32];
assign ldata_al  = e_dc2_done ? e_ldata :
                   (m2.r.msz == SZ_W) ? {16'd0, m2.r.a[1] ? al_word[15:0] : al_word[31:16]} : al_word;
assign ldata_alv = e_dc2_done ||
                   (m2.r.msz == SZ_L && m2.r.a[1:0] == 2'd0) ||
                   (m2.r.msz == SZ_W && !m2.r.a[0]);
assign fault   = e_dc2_done && e_flt;
assign fvec    = 8'd2;
assign faddr   = e_faddr;
assign fssw    = e_fssw;

// the valid copies' reads: DC1's lookup; the engine state's set (a
// split's next line, the slow path's, the scan's, else the fill/push set), or the snooped set while the engine is frozen
assign lvd_q = lsd_q[3:0];
wire  [7:0] lse_q = lse[lv_ra];
assign lve_q = lse_q[3:0];
always_comb
	lv_ra = (sn_ph == SN_B)                            ? sn_pa[9:4] :
	        (e_st == E_SP_2)                           ? m2.r.a[9:4] + 6'd1 :
	        (e_st == E_S_XL2 || e_st == E_S_ACT ||
	         e_st == E_W_PART)                         ? e_va[9:4] :
	        (e_st == E_M_SCANW || e_st == E_M_SCAN2)   ? sc_set : f_set;
	// (a walk's set through f_set, which follows tw_da a cycle late while
	// the walk runs: the walk's decode stays off this read address)
initial for (int i = 0; i < 64; i++) begin
	lsd[i] = 8'd0; lse[i] = 8'd0; lsw[i] = 8'd0;
end
// synthesis translate_off
// the copies against the flip-flop model (after reset's sweep)
always_ff @(posedge clk)
	if (nreset && !clr_rst && (lvd_q != lv[m1.a[9:4]] || lve_q != lv[lv_ra])) begin
		$display("FAIL: dmu: valid RAM %b/%b, model %b/%b (sets %0d %0d)",
		         lvd_q, lve_q, lv[m1.a[9:4]], lv[lv_ra], m1.a[9:4], lv_ra);
		$finish;
	end
// synthesis translate_on

endmodule
