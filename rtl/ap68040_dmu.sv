//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_dmu.sv - data memory unit: D-ATC, data cache, table walker       //
//                                                                          //
// Mirrors the back end's DC1, DC2, EX and WB stages for memory uops.       //
//                                                                          //
// Data cache (MC68040UM section 4): 4 Kbytes, four ways of 64 sets of     //
// 16-byte lines, physically tagged (PA31-10; the set is PA9-4 = LA9-4).    //
// A valid bit per line and a dirty bit per long word; replacement takes   //
// the first invalid way, else a 2-bit counter.  Copyback and write-       //
// through cachable, serialized and nonserialized cache-inhibited modes.   //
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
// table 3-1 (locked read-modify-write where the table says so).  Descriptor//
// reads see the data cache (cachable write-through, no allocate, 3.2.5);  //
// descriptor updates are noncachable and invalidate a matching line.      //
// Like the 68040 (WinUAE cpummu.cpp), a failed search still creates an    //
// ATC entry, with R clear (B set for a bus error), unless ATC_INVALID = 0 //
//--------------------------------------------------------------------------//

module ap68040_dmu
	import ap68040_pkg::*;
#(
	parameter bit ATC_INVALID = 1
)(
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
	input  logic        dm_iack,
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
	output logic [31:0] mt_mmusr,

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

wire tc_e  = tc[15] && !mdis;
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
	logic        iack;
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
logic [3:0]  lv  [64];             // line valid, per set and way
logic        ld  [64][4];          // dirty [set][way] (the push writes the whole line)
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
wire         sn_rset = (sn_ph == SN_A) || (sn_ph == SN_B);   // B: the line stays readable in H
logic  [1:0] sn_src;               // found in: 0 a cache way, 1 push buffer, 2 queued push
logic  [1:0] sn_way;
logic        bo_cancel;            // the queued push was invalidated by a snoop

// The lookup set: AG's address (late: register read, address adders)
// passes one LUT; the other choices are registers, merged beforehand.
// Each way's RAMs get their own copy of that LUT (fanout 18, not 72).
wire  [5:0] la_alt   = sn_rset ? sn_pa[9:4] : steal ? st_set : m1.a[9:4];
wire        la_ag    = adv_ag && !sn_rset && !steal;

// tags: port A for lookups (DC1 or stolen), port B for engine reads/writes
logic [21:0] tq_a [4];
logic [21:0] tq_b [4];
logic        tw_b;
logic  [5:0] tset_b;
logic  [1:0] tway_b;
logic [21:0] twd_b;

// data: write port for stores and fills, read port for DC1 / engine
logic [127:0] dq_r  [4];           // registered output (DC2 fast path)
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

genvar gw;
generate
	for (gw = 0; gw < 4; gw++) begin : g_way
		(* keep *) wire [5:0] la_set_n = la_ag ? dm_va[9:4] : la_alt;
		ap68040_tdp #(.AW(6), .DW(22)) tag (
			.clk(clk),
			.addr_a(la_set_n), .we_a(1'b0), .wd_a(22'd0), .q_a(tq_a[gw]),
			.addr_b(tset_b), .we_b(tw_b && tway_b == 2'(gw)), .wd_b(twd_b), .q_b(tq_b[gw])
		);
		ap68040_sdp_be #(.AW(6), .NB(16), .OREG(0)) dat (
			.clk(clk),
			.we(dw && dw_way == 2'(gw)), .waddr(dw_set), .wbe(dw_be), .wdata(dw_wdata),
			.raddr(la_set_n), .oce(1'b1), .q(dq_rn[gw])
		);
		// DC2's copy of the set: byte writes to it are merged in (the
		// RAM output misses the writes landing at the last edge and this
		// one), so a load after a store to its line stays on the fast path
		always_ff @(posedge clk) begin
			logic [127:0] l;
			if (adv_dc1) begin
				l = dq_rn[gw];
				if (dwq && dwq_src == 2'd0 && dwq_set == m1.a[9:4] && dwq_way == 2'(gw))
					l = mrg(l, dwq_be, dwq_word);
				if (dw && dw_src == 2'd0 && dw_set == m1.a[9:4] && dw_way == 2'(gw))
					l = mrg(l, dw_be, dw_word);
				dq_r[gw] <= l;
			end
			else if (dw && dw_src == 2'd0 && dw_set == m2.r.a[9:4] && dw_way == 2'(gw))
				dq_r[gw] <= mrg(dq_r[gw], dw_be, dw_word);
		end
	end
endgenerate

//--------------------------------------------------------------------------
// D-ATC
//--------------------------------------------------------------------------
logic        atc_hit;
atce_t       atc_e;
logic        atc_wr;
logic [31:0] atc_wla;
logic        atc_wfc2;
atce_t       atc_went;
logic        atc_fall, atc_fpage, atc_fng;

ap68040_atc datc (
	.clk(clk), .nreset(nreset), .p8k(tc_p),
	.la(steal ? st_la : m1.a), .fc2(steal ? st_fc2 : m1.smode),
	.hit(atc_hit), .ent(atc_e),
	.wr(atc_wr), .wla(atc_wla), .wfc2(atc_wfc2), .went(atc_went),
	.flush_all(atc_fall), .flush_page(atc_fpage), .flush_nonglobal(atc_fng)
);

//--------------------------------------------------------------------------
// translation and cache tag compare of a lookup
//--------------------------------------------------------------------------
function automatic xres_t xlate(input logic [31:0] la, input logic s, input logic wr,
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
	if (tt0 || tt1) begin
		x.ok  = 1'b1;
		x.pa  = la;
		x.cm  = tt0 ? dtt0[6:5] : dtt1[6:5];
		x.upa = tt0 ? dtt0[9:8] : dtt1[9:8];
		x.flt = wr && (tt0 ? dtt0[2] : dtt1[2]);
	end
	else if (!tc_e) begin
		x.ok = 1'b1;
		x.pa = la;
		x.cm = 2'b00;          // cachable, write-through (MC68040UM 3.1.2)
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
xres_t      x_dc1;
always_comb begin
	x_dc1 = xlate(m1.a, m1.smode, m1_wr, atc_hit, atc_e,
	              tq_a[0], tq_a[1], tq_a[2], tq_a[3], lv[m1.a[9:4]]);
	if (m1.iack) begin
		// CPU space (interrupt acknowledge) is never translated nor cached
		x_dc1     = '0;
		x_dc1.ok  = 1'b1;
		x_dc1.pa  = m1.a;
		x_dc1.cm  = 2'b10;
	end
end
wire  [4:0] m1_end   = {1'b0, m1.a[3:0]} + {2'b00, nbytes(m1.msz)};
wire        m1_split = (m1.msz != SZ_Q) && (m1_end > 5'd16);
wire        m1_sfast = ((m1.mem == M_LD) || (m1.mem == M_ST && x_dc1.cm == 2'b01)) &&
                       m1_split && x_dc1.ok && !x_dc1.flt && x_dc1.hit &&
                       !x_dc1.cm[1] && dc_en && !m1.iack && !m1.lock;
wire        m1_fast  = ((m1.mem == M_LD) || (m1.mem == M_RMW && !m1.lock)) &&
                       x_dc1.ok && !x_dc1.flt && x_dc1.hit &&
                       !x_dc1.cm[1] && dc_en && !m1.iack &&
                       (m1.msz != SZ_Q) && !m1_split;

//--------------------------------------------------------------------------
// DC2
//--------------------------------------------------------------------------
// a store older than the DC2 load and not yet in the data RAM (EX, WB)
wire  [27:0] m2_line = m2.x.pa[31:4];
function automatic logic st_line(input mrec_t m, input logic [27:0] l);
	st_line = m.r.v && (m.r.mem == M_ST || m.r.mem == M_RMW) &&
	          (m.x.pa[31:4] == l || (m.split && m.x1.pa[31:4] == l));
endfunction
wire hz = st_line(m3, m2_line) || st_line(m4, m2_line);
// any store older than the DC2 uop not yet performed: a table walk for the
// DC2 uop must see it (the program may just have written a descriptor)
wire st_older = (m3.r.v && (m3.r.mem == M_ST || m3.r.mem == M_RMW)) ||
                (m4.r.v && (m4.r.mem == M_ST || m4.r.mem == M_RMW));
// a RAM write to the load's set, or any engine activity, after its lookup
// read the RAM makes the captured line stale: the engine reads it again
logic m1_stale;

// (a store that left WB is still being written this cycle: dw)
wire fast_now = m2.r.v && m2.fast && !hz && !(dw && dw_set == m2.r.a[9:4]);
wire [31:0] fast_data = take(dq_r[m2.x.way], m2.r.a[3:0], nbytes(m2.r.msz));

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
	E_M_START, E_M_PG, E_M_SCAN, E_M_SCANW, E_M_SCAN2, E_M_IC, E_M_DONE,
	E_FILL, E_FILLW, E_FILL_W, E_FILL_INS, E_PUSHV, E_PUSHVW, E_PUSHV_W, E_PUSHV_B,
	E_BUS, E_BUS_W,
	E_TW_START, E_TW_DESC, E_TW_DESCW, E_TW_DLK, E_TW_DBUS, E_TW_EVAL, E_TW_UPD, E_TW_UPD_W,
	E_TW_DONE,
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
	logic        iack;
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
logic  [1:0] f_way;
logic [21:0] f_tag;
logic        pv_v;                // push buffer holds a dirty line
logic [127:0] pv_line;
logic [27:0] pv_lpa;              // its line address
logic        pv_dirty;
logic  [1:0] pv_tln;
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

// SSW of an access error (MC68040UM figure 8-6)
function automatic logic [15:0] mk_ssw(input logic atc, input logic lk, input logic rd,
                                       input logic [1:0] msz, input logic [2:0] fc);
	logic [1:0] sz;
	case (msz)
		SZ_B: sz = 2'b01;
		SZ_W: sz = 2'b10;
		SZ_L: sz = 2'b00;
		default: sz = 2'b11;
	endcase
	// a MOVE16 (line operand) reports TT = 01, the MOVE16 access
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
wire fast_wait = m2.r.v && m2.fast && (hz || (dw && dw_set == m2.r.a[9:4]));
wire dc2_slow = m2.r.v && !fast_now && !fast_wait && !(st_simple && !e_job) && !e_dc2_done && !e_job;
wire wb_slow  = wb_st && !wb_fast && !wb_fs && !wfs_p1 && !e_wst_done;

// store ready: the fast store answers in its own WB cycle (the engine's
// stores through the registered pulse)
logic st_rdy_r, st_fault_r;
assign st_rdy   = st_rdy_r || (wb_fast && !sn_frz) || (wfs_p1 && m4.x1.hit && !sn_frz);
assign st_fault = st_rdy_r && st_fault_r;

//--------------------------------------------------------------------------
// BIU request from the engine's bus step
//--------------------------------------------------------------------------
logic e_breq;
always_comb begin
	b_req  = e_breq;
	b_breq = '0;
	b_breq.addr  = bo.iack ? 32'hFFFF_FFFF : bo_pa;
	b_breq.siz   = bo.iack ? SIZ_B : bo.line ? SIZ_LINE : bo_siz;
	b_breq.rd    = bo.rd;
	b_breq.tt    = bo.iack ? TT_ACK : bo.tt;
	b_breq.tm    = bo.iack ? iack_lvl : bo.tm;
	b_breq.tln   = bo.tln;
	b_breq.upa   = bo.upa;
	b_breq.ci    = bo.ci;
	b_breq.lock  = bo.lock;
	b_breq.locke = bo.locke && !bo.rd && (bo.line || bo_left == bo_np);
	b_wdata      = bo.line ? bo_line :
	               {96'd0, (bo_np == 3'd1) ? {4{bo_wd[31:24]}} :
	                       (bo_np == 3'd2) ? {2{bo_wd[31:16]}} : bo_wd};
end

assign dm_hold1 = sn_frz || steal || steal_q ||
                  (e_st != E_IDLE && e_st != E_W_START &&
                   e_st != E_W_PART && e_st != E_W_DONE);

always_ff @(posedge clk) begin
	st_rdy_r    <= 1'b0;
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

	if (!nreset) begin
		m1 <= '0; m2 <= '0; m3 <= '0; m4 <= '0;
		e_st <= E_IDLE; e_ret <= E_IDLE; e_sret <= E_IDLE; e_wret <= E_IDLE;
		e_dc2_done <= 1'b0; e_wst_done <= 1'b0;
		e_ldata <= '0; e_flt <= 1'b0; e_faddr <= '0; e_fssw <= '0;
		e_part <= 1'b0; e_x <= '0; e_va <= '0; e_n <= '0; e_acc <= '0; e_fromline <= 1'b0;
		e_breq <= 1'b0;
		bo <= '0; bo_wd <= '0; bo_line <= '0; bo_err <= 1'b0; bo_rd <= '0;
		bo_avec <= 1'b0; bo_tci <= 1'b0; bo_left <= '0; bo_pn <= '0; bo_pa <= '0;
		steal <= 1'b0; st_la <= '0; st_fc2 <= 1'b0; st_set <= '0;
		f_set <= '0; f_way <= '0; f_tag <= '0;
		pv_v <= 1'b0; pv_line <= '0; pv_lpa <= '0; pv_dirty <= 1'b0; pv_tln <= '0;
		mv_line <= '0;
		ms_i <= '0;
		tw_lvl <= TW_ROOT; tw_va <= '0; tw_s <= 1'b0; tw_wr <= 1'b0; tw_pt <= 1'b0;
		tw_i <= 1'b0; tw_da <= '0; tw_d <= '0; tw_wp <= 1'b0; tw_fail <= 1'b0;
		tw_berr <= 1'b0; tw_pd <= '0;
		tset_b <= '0; tway_b <= '0; twd_b <= '0;
		dw_set <= '0; dw_way <= '0; dw_be <= '0; dw_src <= '0; dw_word <= '0;
		ic_inv <= 1'b0; ic_inv_scope <= '0; ic_inv_pa <= '0;
		rrc <= 2'd0;
		st_fault_r <= 1'b0; st_fssw <= '0; st_faddr <= '0;
		mt_mmusr <= '0;
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
		sn_look <= 1'b0; sn_hit <= 1'b0; sn_dirty <= 1'b0;
		for (int i = 0; i < 64; i++) begin
			lv[i] <= 4'd0;
			for (int j = 0; j < 4; j++) ld[i][j] <= 1'b0;
		end
	end
	else begin
		// the valid/dirty write port: at most one writer per cycle (the
		// WB fast store runs only with the engine idle, the snoop port
		// only with it frozen), applied at the end of this block
		logic [5:0] vp_set;
		logic [3:0] vp_wm;               // ways written
		logic       vp_lwe, vp_lv;       // valid: write, value
		logic       vp_dwe, vp_dv;       // dirty: write, value
		logic       vp_all;              // invalidate everything
		vp_set = '0; vp_wm = '0; vp_lwe = 1'b0; vp_lv = 1'b0;
		vp_dwe = 1'b0; vp_dv = 1'b0; vp_all = 1'b0;

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
			m1.iack    <= dm_iack;
			m1_stale   <= (dw && dw_src != 2'd0 && dw_set == dm_va[9:4]) || !e_quiet;
		end
		else begin
			if (adv_dc1) m1.v <= 1'b0;
			if ((dw && dw_src != 2'd0 && dw_set == m1.a[9:4]) || !e_quiet) m1_stale <= 1'b1;
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

		//--------------------------------------------------------------
		// engine (frozen while the snooper has the cache)
		//--------------------------------------------------------------
		if (!sn_frz)
		case (e_st)
		E_IDLE: begin
			steal <= 1'b0;
			if (wfs_p1)
				;                       // a split WB store's second part next
			else if (mt_v && !mt_done)
				e_st <= E_M_START;
			else if (wb_slow)
				e_st <= E_W_START;
			else if ((dc2_slow || (e_job && m2.r.v && !e_dc2_done)) && !kill_now)
				e_st <= E_S_START;
			else if (iw_req && !iw_done) begin
				tw_va <= iw_va; tw_s <= iw_fc2; tw_wr <= 1'b0; tw_pt <= 1'b0; tw_i <= 1'b1;
				e_ret <= E_I_DONE;
				e_st  <= E_TW_START;
			end
		end

		//==============================================================
		// DC2 slow path
		//==============================================================
		E_S_START: begin
			e_kill <= 1'b0;
			e_job  <= 1'b1;
			e_part <= 1'b0;
			e_va   <= m2.r.a;
			e_x    <= m2.x;
			e_acc  <= '0;
			e_flt  <= 1'b0;
			e_fromline <= 1'b0;
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
				e_acc <= (take(dq_r[m2.x.way], m2.r.a[3:0], n0) << (8 * (nbytes(m2.r.msz) - n0))) |
				         take(dq_rn[e_x.way], 4'd0, nbytes(m2.r.msz) - n0);
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
			x  = xlate({m2.r.a[31:4] + 28'd1, 4'd0}, m2.r.smode, (m2.r.mem != M_LD), atc_hit, atc_e,
			           tq_a[0], tq_a[1], tq_a[2], tq_a[3], lv[m2.r.a[9:4] + 6'd1]);
			// the second part's translation and lookup, registered (the
			// decision and the data next cycle; the RAM keeps reading the
			// stolen set)
			e_x  <= x;
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
			x = xlate(e_va, m2.r.smode, (m2.r.mem != M_LD), atc_hit, atc_e,
			          tq_a[0], tq_a[1], tq_a[2], tq_a[3], lv[e_va[9:4]]);
			if (m2.r.iack) begin
				x    = '0;
				x.ok = 1'b1;
				x.pa = e_va;
				x.cm = 2'b10;
			end
			if (x.walk && st_older) begin
				// older stores first (they need the engine at WB): retry
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (x.walk) begin
				tw_va <= e_va; tw_s <= m2.r.smode; tw_wr <= (m2.r.mem != M_LD);
				tw_pt <= 1'b0; tw_i <= 1'b0;
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
			cach   = dc_en && !e_x.cm[1] && !m2.r.lock && !m2.r.iack && !move16;
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
			else if (e_fromline) begin
				// an inhibited fill delivered the line: the operand from it
				e_acc <= (e_acc << (8 * e_n)) | take(bo_line, e_va[3:0], e_n);
				e_fromline <= 1'b0;
				e_x.hit <= 1'b0;
				e_x.cm  <= 2'b11;
				e_st    <= E_S_NEXT;
			end
			else if (rd && !e_part && !m2.r.iack && (e_x.cm == 2'b10 || m2.r.lock) && dm_older) begin
				// a serialized read (a noncachable serialized page, or a
				// locked access) waits until every earlier instruction has
				// completed: pending writes are on the bus first, and
				// nothing older can abort the instruction after the read
				// (MC68040UM 4.3.2, 7.7).  Retry from the start.
				steal <= 1'b0;
				e_st  <= E_IDLE;
			end
			else if (move16 && m2.r.mem == M_LD && e_x.hit && dc_en) begin
				// MOVE16 source hit: read from the cache, no allocation
				e_st <= E_S_RD;
			end
			else if (move16 && m2.r.mem == M_ST) begin
				// MOVE16 destination: a write hit invalidates the line
				if (e_x.hit) begin
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
					f_way  <= victim(lv[e_va[9:4]], rrc);
					e_sret <= E_S_XL;
					e_st   <= E_FILL;
				end
				else e_st <= E_S_NEXT;
			end
			else if (cach && e_x.hit && rd) begin
				// older stores to this line must reach the RAM first: give
				// the engine back to them and retry (the line of the part
				// being read: a split's second part has its own)
				if (!(e_part ? (st_line(m3, e_x.pa[31:4]) || st_line(m4, e_x.pa[31:4])) : hz))
					e_st <= E_S_RD;
				else begin steal <= 1'b0; e_st <= E_IDLE; end
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
				f_way  <= victim(lv[e_va[9:4]], rrc);
				e_sret <= E_S_XL;
				e_st   <= E_FILL;
			end
			else if (rd) begin
				// cache inhibited, no-allocate miss, locked, MOVE16 miss,
				// IACK: a matching line is pushed (if dirty) and invalidated
				// first (4.3.2); the bus access goes in program order
				if (e_x.hit && (m2.r.lock || e_x.cm[1] || move16)) begin
					f_set  <= e_va[9:4];
					f_way  <= e_x.way;
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
					bo.iack  <= m2.r.iack;
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
			// read the line through the stolen data port
			steal  <= 1'b1;
			st_set <= e_va[9:4];
			e_st   <= E_S_RDW;
		end
		E_S_RDW: e_st <= E_S_RD2;
		E_S_RD2: begin
			if (m2.r.msz == SZ_Q) mv_line <= dq_rn[e_x.way];
			else e_acc <= (e_acc << (8 * e_n)) | take(dq_rn[e_x.way], e_va[3:0], e_n);
			e_st <= E_S_NEXT;
		end
		E_S_BUSD: begin
			// the bus read finished
			if (bo_err && !m2.r.iack) begin
				e_flt   <= 1'b1;
				e_faddr <= m2.r.a;
				e_fssw  <= mk_ssw(1'b0, m2.r.lock, 1'b1, m2.r.msz, m2.r.fc);
				e_st    <= E_S_DONE;
			end
			else begin
				if (m2.r.iack)
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
			e_ldata    <= e_acc;
			steal      <= 1'b0;
			e_st       <= E_IDLE;
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
				bo.iack  <= 1'b0;
				bo.tln   <= 2'd0;
				bo_pa    <= (m4.r.msz == SZ_Q) ? {e_x.pa[31:4], 4'd0} : e_x.pa;
				bo_left  <= (m4.r.msz == SZ_Q) ? 3'd4 : e_n;
				bo_wd    <= e_acc;
				bo_line  <= mv_line;
				e_ret    <= E_W_DONE;
				e_st     <= E_BUS;
			end
		end
		E_W_DONE: begin
			if (bo_err) begin
				st_rdy_r     <= 1'b1;
				st_fault_r   <= 1'b1;
				st_fssw    <= mk_ssw(1'b0, m4.r.lock, 1'b0, m4.r.msz, m4.r.fc);
				st_faddr   <= m4.r.a;
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
						vp_all = 1'b1;
						e_st <= E_M_IC;
					end
					else begin
						// scan one set (line), or every set (page, all)
						ms_i <= (mt_scope == 2'd1) ? {mt_addr[9:4], 2'b00} : 8'd0;
						e_st <= E_M_SCAN;
					end
				end
			endcase
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
			e_st   <= E_M_SCANW;
		end
		E_M_SCANW: e_st <= E_M_SCAN2;
		E_M_SCAN2: begin
			logic [5:0] s; logic [1:0] w; logic m;
			s = ms_i[7:2]; w = ms_i[1:0];
			m = lv[s][w] && ((mt_scope == 2'd3) ||
			                 (mt_scope == 2'd1 && tq_b[w] == mt_addr[31:10]) ||
			                 (mt_scope == 2'd2 && tq_b[w][21:2] == mt_addr[31:12]));
			if (m && mt_op == MT_CPUSH && ld[s][w]) begin
				// push (it is invalidated with the push)
				f_set  <= s;
				f_way  <= w;
				e_sret <= E_M_SCAN;          // reread the set's tags after
				e_st   <= E_PUSHV;
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
			if (lv[f_set][f_way] && ld[f_set][f_way]) begin
				pv_v     <= 1'b1;
				pv_line  <= dq_rn[f_way];
				pv_lpa   <= {tq_b[f_way], f_set};
				pv_dirty <= ld[f_set][f_way];
				pv_tln   <= f_way;
			end
			vp_set = f_set; vp_wm[f_way] = 1'b1;
			vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
			bo.line <= 1'b1; bo.rd <= 1'b1; bo.tt <= TT_NORMAL;
			bo.tm <= m2.r.smode ? TM_SDATA : TM_UDATA;
			bo.upa <= e_x.upa; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
			bo.iack <= 1'b0; bo.tln <= f_way;
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
			if (bo_err) begin
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
			if (lv[f_set][f_way] && ld[f_set][f_way]) begin
				pv_v     <= 1'b1;
				pv_line  <= dq_rn[f_way];
				pv_lpa   <= {tq_b[f_way], f_set};
				pv_dirty <= ld[f_set][f_way];
				pv_tln   <= f_way;
				e_st     <= E_PUSHV_B;
			end
			else e_st <= e_sret;
			vp_set = f_set; vp_wm[f_way] = 1'b1;
			vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
		end
		E_PUSHV_B: begin
			bo.line <= 1'b1; bo.rd <= 1'b0; bo.tt <= TT_NORMAL; bo.tm <= TM_PUSH;
			bo.upa <= 2'd0; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
			bo.iack <= 1'b0; bo.tln <= pv_tln;
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
			bo_tci <= 1'b0;
			bo_avec <= 1'b0;
			bo_rd  <= '0;
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
			if (bo_cancel && !b_gnt) begin
				// not on the bus yet (the snooped master has it): dropped
				bo_cancel <= 1'b0;
				e_breq    <= 1'b0;
				e_st      <= e_ret;
			end
			else if (b_gnt) begin
				e_breq <= 1'b0;
				bo_pn  <= bo_np;
			end
			if (b_rvalid) begin
				if (bo.line) begin
					bo_line[127 - 32*b_rbeat -: 32] <= b_rdata;
					if (b_rbeat == bo_pa[3:2]) bo_tci <= b_rtci;
				end
				else begin
					bo_rd   <= (bo_rd << (8 * bo_pn)) | grab(b_rdata, bo_pn, bo_pa[1:0]);
					bo_avec <= b_ravec;
				end
			end
			if (b_err) begin
				bo_err <= 1'b1;
				e_st   <= e_ret;
			end
			else if (b_done) begin
				if (bo.line || bo.iack) e_st <= e_ret;
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
			tw_da   <= {(tw_s ? srp[31:9] : urp[31:9]), 9'd0} + {23'd0, tw_ri, 2'b00};
			e_st    <= E_TW_DESC;
		end
		E_TW_DESC: begin
			// descriptor read: from the data cache if the line is there
			// (cachable write-through, no allocate), else from the bus
			steal  <= 1'b1;
			st_set <= tw_da[9:4];
			tset_b <= tw_da[9:4];
			e_st   <= E_TW_DESCW;
		end
		E_TW_DESCW: e_st <= E_TW_DLK;
		E_TW_DLK: begin
			logic h; logic [1:0] w;
			h = 1'b0; w = 2'd0;
			for (int i = 0; i < 4; i++)
				if (lv[tw_da[9:4]][i] && tq_b[i] == tw_da[31:10]) begin h = 1'b1; w = 2'(i); end
			if (h && dc_en) begin
				tw_d <= take(dq_rn[w], tw_da[3:0], 3'd4);
				e_st <= E_TW_EVAL;
			end
			else begin
				bo.line <= 1'b0; bo.rd <= 1'b1; bo.tt <= TT_NORMAL;
				bo.tm <= tw_i ? TM_TBL_CODE : TM_TBL_DATA;
				bo.upa <= 2'd0; bo.ci <= 1'b0; bo.lock <= 1'b0; bo.locke <= 1'b0;
				bo.iack <= 1'b0; bo.tln <= 2'd0;
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
			// read-modify-write sets U, except a permitted write to a clean
			// page (U and M clear), which is a plain write of U and M; U set
			// and M to set: a plain write.  Noncachable: a cached copy of the
			// descriptor's line is invalidated.
			logic page, sv, setm, needu, lk;
			logic [31:0] nd;
			page  = (tw_lvl == TW_PAGE || tw_lvl == TW_IND);
			sv    = page && tw_d[7] && !tw_s;
			setm  = page && tw_wr && !tw_wp && !sv && !tw_d[4];
			needu = !tw_d[3];
			lk    = needu && !(setm && !tw_d[4]);
			nd    = tw_d | 32'h8 | (setm ? 32'h10 : 32'h0);
			tw_d  <= nd;
			if (page) tw_pd <= nd;
			bo.line <= 1'b0; bo.rd <= 1'b0; bo.tt <= TT_NORMAL;
			bo.tm   <= tw_i ? TM_TBL_CODE : TM_TBL_DATA;
			bo.upa  <= 2'd0; bo.ci <= 1'b1;
			bo.lock <= lk; bo.locke <= lk;
			bo.iack <= 1'b0; bo.tln <= 2'd0;
			bo_pa   <= tw_da;
			bo_left <= 3'd4;
			bo_wd   <= nd;
			vp_set = tw_da[9:4];
			vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
			for (int i = 0; i < 4; i++)
				if (lv[tw_da[9:4]][i] && tq_b[i] == tw_da[31:10]) vp_wm[i] = 1'b1;
			e_wret <= e_ret;
			e_ret  <= E_TW_UPD_W;
			e_st   <= E_BUS;
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
			// install (failed searches too, as the 68040 does)
			if (!tw_fail || ATC_INVALID) begin
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
				if (lv[s][i] && tq_a[i] == sn_pa[31:10]) begin h = 1'b1; w = 2'(i); end
			sn_way <= w;
			sn_look <= 1'b1;
			if (h) begin
				sn_src   <= 2'd0;
				sn_hit   <= 1'b1;
				sn_dirty <= ld[s][w];
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
			if (sn_inv) begin
				case (sn_src)
				2'd0: begin
					vp_set = s; vp_wm[sn_way] = 1'b1;
					vp_lwe = 1'b1; vp_lv = 1'b0; vp_dwe = 1'b1; vp_dv = 1'b0;
					// no record keeps a hit on the line that left
					if (m2.x.hit && m2.x.pa[9:4] == s && m2.x.way == sn_way) m2.x.hit <= 1'b0;
					if (m2.x1.hit && m2.x1.pa[9:4] == s && m2.x1.way == sn_way) m2.x1.hit <= 1'b0;
					if (adv_dc2) begin
						if (m2.x.hit && m2.x.pa[9:4] == s && m2.x.way == sn_way) m3.x.hit <= 1'b0;
						if (m2.x1.hit && m2.x1.pa[9:4] == s && m2.x1.way == sn_way) m3.x1.hit <= 1'b0;
					end
					else begin
						if (m3.x.hit && m3.x.pa[9:4] == s && m3.x.way == sn_way) m3.x.hit <= 1'b0;
						if (m3.x1.hit && m3.x1.pa[9:4] == s && m3.x1.way == sn_way) m3.x1.hit <= 1'b0;
					end
					if (adv_ex) begin
						if (m3.x.hit && m3.x.pa[9:4] == s && m3.x.way == sn_way) m4.x.hit <= 1'b0;
						if (m3.x1.hit && m3.x1.pa[9:4] == s && m3.x1.way == sn_way) m4.x1.hit <= 1'b0;
					end
					else begin
						if (m4.x.hit && m4.x.pa[9:4] == s && m4.x.way == sn_way) m4.x.hit <= 1'b0;
						if (m4.x1.hit && m4.x1.pa[9:4] == s && m4.x1.way == sn_way) m4.x1.hit <= 1'b0;
					end
					if (e_x.hit && e_x.pa[9:4] == s && e_x.way == sn_way) e_x.hit <= 1'b0;
				end
				2'd1: pv_v <= 1'b0;
				default: bo_cancel <= 1'b1;
				endcase
			end
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

		//--------------------------------------------------------------
		// the valid/dirty write port
		//--------------------------------------------------------------
		if (vp_all) begin
			for (int i = 0; i < 64; i++) begin
				lv[i] <= 4'd0;
				for (int j = 0; j < 4; j++) ld[i][j] <= 1'b0;
			end
		end
		else begin
			for (int j = 0; j < 4; j++)
				if (vp_wm[j]) begin
					if (vp_lwe) lv[vp_set][j] <= vp_lv;
					if (vp_dwe) ld[vp_set][j] <= vp_dv;
				end
		end
	end
end

// snoop: the line found, valid with sn_look (the RAM still reads the
// snooped set in the first H cycle)
assign sn_line = (sn_src == 2'd0) ? dq_rn[sn_way] : (sn_src == 2'd1) ? pv_line : bo_line;

assign tw_busy = (e_st == E_TW_START) || (e_st == E_TW_DESC) || (e_st == E_TW_DESCW) ||
                 (e_st == E_TW_DLK) || (e_st == E_TW_DBUS) || (e_st == E_TW_EVAL) ||
                 (e_st == E_TW_UPD) || (e_st == E_TW_UPD_W) || (e_st == E_TW_DONE);

// DC2 answer
// once the engine has taken the DC2 uop, only its completion releases it
assign dc2_rdy = !m2.r.v || e_dc2_done || (!e_job && (fast_now || st_simple));
assign ldata   = e_dc2_done ? e_ldata : fast_data;
assign fault   = e_dc2_done && e_flt;
assign fvec    = 8'd2;
assign faddr   = e_faddr;
assign fssw    = e_fssw;

endmodule
