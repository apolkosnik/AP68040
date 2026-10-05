//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_useq.sv - D2: micro-sequencer                                     //
//                                                                          //
// Turns a decoded instruction into uops by walking its microcode routine   //
// (rtl/gen/ap68040_ucode_rom.svh), one uop per cycle:                       //
//   * symbolic operands (EA0, DX, IMM, ...) become registers, immediates   //
//     or the uop's own memory access; A7 maps to USP/ISP/MSP by S and M    //
//   * an EA with memory indirection gets a pointer load (into T13 for EA0, //
//     T12 for EA1) before the uop that uses it                             //
//   * a source (An)+/-(An) update marked noupd is deferred into the uop    //
//     flagged upd2 (merged when both EAs use the same register), so an     //
//     instruction that faults on its second access restarts cleanly        //
//   * the back end's exception entry starts an exception routine           //
//--------------------------------------------------------------------------//

module ap68040_useq
	import ap68040_pkg::*, ap68040_upkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,

	// record FIFO from D1
	input  logic  [1:0] rq_n,
	input  dinst_t      rq0,
	input  dinst_t      rq1,
	output logic        rq_pop,

	input  logic        smode,
	input  logic        master,     // M bit

	input  logic        exc_go,
	input  logic  [3:0] exc_kind,
	input  logic  [4:0] exc_ssp,

	// uop FIFO towards the back end: uq0 is the head
	output logic  [1:0] uq_n,
	output uop_t        uq0,
	input  logic        uq_pop      // the back end takes uq0 this cycle
);


`include "gen/ap68040_ucode_rom.svh"

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
logic  [8:0] upc;               // address of uw_q
uword_t      uw_q;              // registered microcode word
logic        uw_v;              // uw_q is valid for the current instruction
logic        exc_mode;          // running an exception routine
logic        first;
logic        ind0, ind1;        // pointer of EA0 / EA1 already loaded
logic        def_v;             // a deferred source update is pending
logic  [4:0] def_reg;
logic  [7:0] def_amt;
logic  [4:0] ssp;
// MOVEM loop
logic        mv_act;            // the loop has started for this instruction
logic [15:0] mv_mask;           // registers still to transfer
logic [31:0] mv_off;            // address offset of the next transfer
logic  [7:0] mv_cnt;            // bytes transferred so far
logic        mv_t11;            // the address is in T11 (base register in the list)

// the instruction being sequenced: the D1 FIFO head, or nothing for an
// exception routine
dinst_t      src;
always_comb src = exc_mode ? '0 : rq0;
logic        src_first;
assign src_first = first;

uword_t      uw;
assign uw = uw_q;

// uop FIFO
uop_t        uq1;
wire         room_out = (uq_n != 2'd2);

//--------------------------------------------------------------------------
// register naming
//--------------------------------------------------------------------------
wire [4:0] sp_reg = !smode ? R_USP : (master ? R_MSP : R_ISP);

function automatic logic [4:0] areg(input logic [2:0] n, input logic [4:0] spr);
	areg = (n == 3'd7) ? spr : (R_A0 + {2'b00, n});
endfunction

function automatic logic [4:0] dreg(input logic [2:0] n);
	dreg = {2'b00, n};
endfunction

function automatic logic is_mem(input logic [3:0] m);
	is_mem = (m >= EM_AI) && (m != EM_IMM) && (m != EM_NONE);
endfunction

function automatic logic is_reg(input logic [3:0] m);
	is_reg = (m == EM_DN) || (m == EM_AN);
endfunction

function automatic logic [4:0] size_bytes(input logic [1:0] s, input logic is_sp);
	case (s)
		SZ_B:    size_bytes = is_sp ? 5'd2 : 5'd1;
		SZ_W:    size_bytes = 5'd2;
		SZ_L:    size_bytes = 5'd4;
		default: size_bytes = 5'd16;      // line (MOVE16)
	endcase
endfunction

//--------------------------------------------------------------------------
// one uop from (uw, src)
//--------------------------------------------------------------------------
typedef struct packed {
	logic        v;          // operand present
	logic  [1:0] src;
	logic  [4:0] r;
	logic        imm_v;
	logic [31:0] imm;
	logic        mem;        // the operand is this uop's memory EA
	logic        ea1;        // ... and it is EA1
} opsel_t;

uop_t        nu;
logic        n_ptr;          // nu is a pointer load (memory indirect)
logic        n_ptr_ea1;
logic        n_jump;         // pure jump micro-instruction
logic  [8:0] n_jt;
logic        n_mv_pre;       // nu is the MOVEM address copy into T11
logic [15:0] n_mv_rest;      // mask after this transfer
logic        n_def_set;      // nu defers its source update
logic  [4:0] n_def_reg;
logic  [7:0] n_def_amt;

always_comb begin
	opsel_t oa, ob, od;
	ea_t    me;              // the uop's memory EA
	logic   me_v, me_ea1;
	logic [1:0] osz, msz;
	logic [2:0] mszf;
	logic [4:0] amt;
	logic       me_sp;
	logic [31:0] q, shc;
	logic [6:0] op;
	logic [3:0] cnd;
	logic       jc_true;
	logic [31:0] dval;
	logic [31:0] pcx;

	nu        = '0;
	n_ptr     = 1'b0;
	n_ptr_ea1 = 1'b0;
	n_jump    = 1'b0;
	n_jt      = uw.jt;
	n_def_set = 1'b0;
	n_def_reg = '0;
	n_def_amt = '0;
	n_mv_pre  = 1'b0;
	n_mv_rest = '0;

	osz = (uw.sz  == 2'd0) ? src.sz : (uw.sz  - 2'd1);
	mszf = (uw.msz == 3'd0) ? {1'b0, src.sz} : (uw.msz - 3'd1);
	msz  = mszf[1:0];
	q   = (src.opw[11:9] == 3'd0) ? 32'd8 : {29'd0, src.opw[11:9]};

	// pure jump words (a routine's first jump is resolved in D1)
	case (uw.jc)
		JC_NEVER:    jc_true = 1'b0;
		JC_ALWAYS:   jc_true = 1'b1;
		JC_EXT10:    jc_true = src.ext1[10];
		JC_EXT11:    jc_true = src.ext1[11];
		JC_EA0_MEM:  jc_true = is_mem(src.ea0.m);
		JC_EA0_DN:   jc_true = (src.ea0.m == EM_DN);
		JC_EA0_IMM:  jc_true = (src.ea0.m == EM_IMM);
		JC_SZ_L:     jc_true = (src.sz == SZ_L);
		JC_AY7:      jc_true = (src.opw[2:0] == 3'd7);
		JC_SUPER:    jc_true = smode;
		JC_MASK0:    jc_true = (src.ext1 == 16'd0);
		default:     jc_true = 1'b0;
	endcase
	// a micro-instruction with a jump condition is a pure jump
	n_jump = (uw.jc != JC_NEVER);

	//----------------------------------------------------------------------
	// operand selectors
	//----------------------------------------------------------------------
	for (int k = 0; k < 3; k++) begin
		logic [5:0] s;
		opsel_t o;
		s = (k == 0) ? uw.a : (k == 1) ? uw.b : uw.d;
		o = '0;
		o.v = (s != S_NONE);
		o.src = OS_REG;
		if (s >= 6'd40) o.r = 5'(s - 6'd25);
		else case (s)
			S_EA0, S_EA0R, S_EA1, S_EA1R: begin
				ea_t e;
				e = (s == S_EA0 || s == S_EA0R) ? src.ea0 : src.ea1;
				if (e.m == EM_DN) o.r = dreg(e.r);
				else if (e.m == EM_AN) o.r = areg(e.r, sp_reg);
				else if (e.m == EM_IMM) begin
					o.src = OS_IMM; o.imm_v = 1'b1; o.imm = e.bd;
				end
				else if (s == S_EA0R || s == S_EA1R) o.v = 1'b0;
				else begin
					o.src = OS_MEM; o.mem = 1'b1; o.ea1 = (s == S_EA1);
				end
			end
			S_DX:    o.r = dreg(src.opw[11:9]);
			S_DY:    o.r = dreg(src.opw[2:0]);
			S_AX:    o.r = areg(src.opw[11:9], sp_reg);
			S_AY:    o.r = areg(src.opw[2:0], sp_reg);
			S_SP:    o.r = sp_reg;
			S_SSP:   o.r = ssp;
			S_X1R:   o.r = src.ext1[15] ? areg(src.ext1[14:12], sp_reg) : dreg(src.ext1[14:12]);
			S_X1DL:  o.r = dreg(src.ext1[14:12]);
			S_X1DH:  o.r = dreg(src.ext1[2:0]);
			S_X1DU:  o.r = dreg(src.ext1[8:6]);
			S_X2DC:  o.r = dreg(src.ext2[2:0]);
			S_X2DU:  o.r = dreg(src.ext2[8:6]);
			S_X2R:   o.r = src.ext2[15] ? areg(src.ext2[14:12], sp_reg) : dreg(src.ext2[14:12]);
			S_IMM:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.imm; end
			S_QUICK: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = q; end
			S_MOVEQ: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {{24{src.opw[7]}}, src.opw[7:0]}; end
			S_SHCNT: begin
				if (src.opw[5]) o.r = dreg(src.opw[11:9]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = q; end
			end
			S_NPC:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.npc; end
			S_PC:    begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.pc; end
			S_CONST: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {{16{uw.cval[15]}}, uw.cval}; end
			S_ZERO:  o.src = OS_ZERO;
			S_LD:    o.src = OS_MEM;
			S_CREG:  begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {28'd0, src.ext1[11], src.ext1[2:0]}; end
			S_BFO: begin
				if (src.ext1[11]) o.r = dreg(src.ext1[8:6]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[10:6]}; end
			end
			S_BFW: begin
				if (src.ext1[5]) o.r = dreg(src.ext1[2:0]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[4:0]}; end
			end
			S_OPW:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {16'd0, src.opw}; end
			S_CREGR: o.r = (src.ext1[2:0] == 3'd0) ? R_USP : (src.ext1[2:0] == 3'd3) ? R_MSP : R_ISP;
			default: o.v = 1'b0;
		endcase
		if (k == 0) oa = o; else if (k == 1) ob = o; else od = o;
	end

	//----------------------------------------------------------------------
	// operation fields
	//----------------------------------------------------------------------
	op  = uw.op_inst ? src.eop : uw.op;
	case (uw.cond_inst)
		3'd1:    cnd = src.opw[11:8];
		3'd2:    cnd = src.econd;
		3'd3:    cnd = {2'b00, src.ext1[10], src.ext1[11]};
		3'd4:    cnd = {1'b0, src.opw[10:8]};     // bit field, register
		3'd5:    cnd = {1'b1, src.opw[10:8]};     // bit field, memory
		3'd6:    cnd = {2'b00, !src.ext1[15], src.ext1[11]};  // CHK2/CMP2
		default: cnd = uw.cond;
	endcase
	nu.op     = op;
	nu.sz     = osz;
	nu.msz    = msz;
	nu.cond   = cnd;
	nu.ccr_we = uw.ccr_inst ? src.ccr : uw.ccr;
	nu.a_src  = oa.v ? oa.src : OS_ZERO;
	nu.a_reg  = oa.r;
	nu.a_sxw  = (uw.sxw == 2'd1) || (uw.sxw == 2'd2 && src.sz == SZ_W);
	nu.b_src  = ob.v ? ob.src : OS_ZERO;
	nu.b_reg  = ob.r;
	nu.imm    = oa.imm_v ? oa.imm : {{16{uw.cval[15]}}, uw.cval};
	nu.imm_b  = ob.imm;
	nu.mfc    = uw.mfc;
	nu.mlock  = uw.lock;
	nu.mlocke = uw.locke;
	nu.br     = uw.br;
	nu.target = src.target;
	nu.pred   = src.pred && (uw.br == BR_COND || uw.br == BR_IMM);
	nu.ser    = uw.ser;
	nu.t0cof  = src.t0;
	nu.pc     = src.pc;
	nu.npc    = src.npc;
	nu.first  = src_first;
	nu.last   = uw.last;
	nu.exc    = (src.exc != 8'd0) ? src.exc : 8'd0;

	// destination
	if (od.v && od.src == OS_REG) begin
		nu.d_v   = 1'b1;
		nu.d_reg = od.r;
	end

	//----------------------------------------------------------------------
	// memory access and address generation
	//----------------------------------------------------------------------
	me_v   = oa.mem || ob.mem || od.mem;
	me_ea1 = oa.mem ? oa.ea1 : ob.mem ? ob.ea1 : od.ea1;
	me     = me_ea1 ? src.ea1 : src.ea0;
	if (uw.ag == AGM_EA0 || uw.ag == AGM_LEA0) begin me = src.ea0; me_ea1 = 1'b0; end
	if (uw.ag == AGM_EA1) begin me = src.ea1; me_ea1 = 1'b1; end
	me_sp  = (me.r == 3'd7);
	amt    = size_bytes(msz, me_sp && (me.m == EM_AIP || me.m == EM_APD));

	if (uw.mem != 2'd0)
		nu.mem = uw.mem;
	else if (uw.a == S_LD || uw.b == S_LD)
		nu.mem = M_LD;
	else if (me_v) begin
		if ((ob.mem || oa.mem) && od.mem) nu.mem = M_RMW;
		else if (od.mem)                  nu.mem = M_ST;
		else                              nu.mem = M_LD;
	end


	if (me_v || uw.ag == AGM_EA0 || uw.ag == AGM_EA1 || uw.ag == AGM_LEA0) begin
		logic [4:0] bre;
		logic       mi_stage1;
		nu.ag = 1'b1;
		bre   = areg(me.r, sp_reg);
		mi_stage1 = (me.mi != 2'd0) && !(me_ea1 ? ind1 : ind0);
		case (me.m)
			EM_AI: begin nu.base_v = 1'b1; nu.base = bre; end
			EM_AIP: begin
				nu.base_v = 1'b1; nu.base = bre;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = bre;
				nu.upd_amt = {3'd0, amt};
			end
			EM_APD: begin
				nu.base_v = 1'b1; nu.base = bre;
				nu.disp = -{27'd0, amt};
				nu.upd_v = 1'b1; nu.upd_reg = bre;
			end
			EM_AD16: begin nu.base_v = 1'b1; nu.base = bre; nu.disp = me.bd; end
			EM_ABSW, EM_ABSL: nu.disp = me.bd;
			EM_PC16: begin nu.disp = me.bd; nu.mprog = 1'b1; end   // D1 added the PC
			EM_AX, EM_PCX: begin
				logic [4:0] xr;
				xr = me.xa ? areg(me.xr, sp_reg) : dreg(me.xr);
				if (me.mi == 2'd0 || mi_stage1) begin
					// address (or the pointer address) from base/index/bd
					if (me.m == EM_AX) begin nu.base_v = !me.bs; nu.base = bre; nu.disp = me.bd; end
					else begin nu.disp = me.bd; nu.mprog = !me.bs; end   // D1 added the PC
					nu.idx_v = !me.is && (me.mi != 2'd2);
					nu.idx   = xr;
					nu.idx_l = me.xl;
					nu.scale = me.sc;
				end
				else begin
					// through the pointer: T13 (EA0) / T12 (EA1) + index + od
					nu.base_v = 1'b1;
					nu.base   = me_ea1 ? (R_T0 + 5'd12) : (R_T0 + 5'd13);
					nu.disp   = me.od;
					nu.idx_v  = !me.is && (me.mi == 2'd2);
					nu.idx    = xr;
					nu.idx_l  = me.xl;
					nu.scale  = me.sc;
					nu.mprog  = 1'b0;
				end
				if (mi_stage1) begin
					// this cycle: the pointer load only
					n_ptr     = 1'b1;
					n_ptr_ea1 = me_ea1;
				end
			end
			default: ;
		endcase

		// deferral of the source update into a later uop
		if (uw.noupd && !me_ea1 && (me.m == EM_AIP || me.m == EM_APD)) begin
			logic xuse;
			// an EA1 index naming the same register cannot see a deferred
			// value: update in place then
			xuse = (src.ea1.m == EM_AX || src.ea1.m == EM_PCX) && !src.ea1.is &&
			       src.ea1.xa && (areg(src.ea1.xr, sp_reg) == bre);
			if (!xuse) begin
				nu.upd_v  = 1'b0;
				nu.pinc   = 1'b0;
				n_def_set = 1'b1;
				n_def_reg = bre;
				n_def_amt = (me.m == EM_AIP) ? {3'd0, amt} : -{3'd0, amt};
			end
		end
	end
	// the deferred update lands on the upd2 uop, memory or not (FRESTORE
	// commits (An)+ only after its frame check)
	if (uw.upd2 && def_v) begin
		logic same;
		same = nu.ag && nu.base_v && nu.base == def_reg &&
		       (me.mi == 2'd0 || !(me_ea1 ? ind1 : ind0));
		if (same) begin
			nu.disp = nu.disp + {{24{def_amt[7]}}, def_amt};
			if (me.m == EM_AIP) nu.upd_amt = nu.upd_amt + def_amt;
		end
		if (!(same && (me.m == EM_AIP || me.m == EM_APD))) begin
			nu.upd2_v   = 1'b1;
			nu.upd2_reg = def_reg;
			nu.upd2_amt = def_amt;
		end
	end

	//----------------------------------------------------------------------
	// explicit AG modes
	//----------------------------------------------------------------------
	begin
		logic [4:0]  rb, rw;
		logic        rb_v;
		logic [31:0] dsv;
		rb_v = (uw.agb != S_NONE) && (uw.agb != S_ZERO);
		case (uw.agb)
			S_X1R: rb = src.ext1[15] ? areg(src.ext1[14:12], sp_reg) : dreg(src.ext1[14:12]);
			S_X2R: rb = src.ext2[15] ? areg(src.ext2[14:12], sp_reg) : dreg(src.ext2[14:12]);
			S_AX:  rb = areg(src.opw[11:9], sp_reg);
			S_AY:  rb = areg(src.opw[2:0], sp_reg);
			S_SP:  rb = sp_reg;
			S_SSP: rb = ssp;
			default: rb = 5'(uw.agb - 6'd25);   // USP ISP MSP T0-T13 (>= 40)
		endcase
		case (uw.agw)
			S_AX:  rw = areg(src.opw[11:9], sp_reg);
			S_AY:  rw = areg(src.opw[2:0], sp_reg);
			S_SP:  rw = sp_reg;
			S_SSP: rw = ssp;
			default: rw = 5'(uw.agw - 6'd25);
		endcase
		case (uw.dsel)
			DS_IMM:    dsv = src.imm;
			DS_NIMM:   dsv = -((src.sz == SZ_W) ? {{16{src.ea0.bd[15]}}, src.ea0.bd[15:0]} : src.ea0.bd);
			DS_QUICK:  dsv = q;
			DS_NQUICK: dsv = -q;
			DS_IMMC:   dsv = src.imm + {{16{uw.cval[15]}}, uw.cval};
			DS_SZB:    dsv = {27'd0, size_bytes(src.sz, 1'b0)};
			default:   dsv = {{16{uw.cval[15]}}, uw.cval};
		endcase
		case (uw.ag)
			AGM_PUSH: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = sp_reg;
				nu.disp = -{27'd0, size_bytes(msz, 1'b1)};
				nu.upd_v = 1'b1; nu.upd_reg = sp_reg;
			end
			AGM_POP: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = sp_reg;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = sp_reg;
				nu.upd_amt = {3'd0, size_bytes(msz, 1'b1)};
			end
			AGM_POPR: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = rb;
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = rw;
				nu.upd_amt = {3'd0, size_bytes(msz, 1'b0)};
			end
			AGM_BASED, AGM_BASEDU: begin
				nu.ag = 1'b1; nu.base_v = rb_v; nu.base = rb; nu.disp = dsv;
				if (uw.ag == AGM_BASEDU) begin
					nu.upd_v = 1'b1;
					nu.upd_reg = (uw.agw != S_NONE) ? rw : rb;
				end
				else if (uw.agw != S_NONE) begin
					nu.upd_v = 1'b1; nu.upd_reg = rw;
				end
			end
			AGM_ADDC: begin
				nu.ag = 1'b1; nu.base_v = rb_v; nu.base = rb; nu.disp = dsv;
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_VAL0, AGM_ADDV: begin
				nu.ag = 1'b1;
				nu.base_v = (uw.ag == AGM_ADDV); nu.base = rb;
				if (src.ea0.m == EM_IMM)
					nu.disp = (src.sz == SZ_W) ? {{16{src.ea0.bd[15]}}, src.ea0.bd[15:0]} : src.ea0.bd;
				else begin
					nu.idx_v = 1'b1;
					nu.idx   = (src.ea0.m == EM_AN) ? areg(src.ea0.r, sp_reg) : dreg(src.ea0.r);
					nu.idx_l = (src.sz == SZ_L);
				end
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_ADDT0: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = rb;
				nu.idx_v = 1'b1; nu.idx = R_T0; nu.idx_l = 1'b1;
				nu.upd_v = 1'b1; nu.upd_reg = rw;
			end
			AGM_LEA0: begin
				if (uw.agw != S_NONE) begin nu.upd_v = !n_ptr; nu.upd_reg = rw; end
				nu.mem = M_NONE;
			end
			default: ;
		endcase
	end

	//----------------------------------------------------------------------
	// MOVEM: one transfer per set mask bit.  -(An): mask bit 0 is A7 and the
	// addresses descend from An; other modes ascend from the EA.  An is
	// updated by the last transfer only.
	//----------------------------------------------------------------------
	if (uw.loop) begin
		logic [15:0] m;
		logic  [3:0] bi, rn;
		logic  [4:0] r;
		logic  [2:0] sb;
		logic        pd, pi, lastx, base_in;
		logic  [4:0] an;
		logic [31:0] off;
		m  = mv_act ? mv_mask : src.ext1;
		pd = (src.ea0.m == EM_APD);
		pi = (src.ea0.m == EM_AIP);
		bi = 4'd0;
		for (int i = 15; i >= 0; i--) if (m[i]) bi = 4'(i);
		rn = pd ? (4'd15 - bi) : bi;
		r  = rn[3] ? areg(rn[2:0], sp_reg) : dreg(rn[2:0]);
		n_mv_rest = m & ~(16'd1 << bi);
		lastx = (n_mv_rest == 16'd0);
		sb = (src.sz == SZ_L) ? 3'd4 : 3'd2;
		an = areg(src.ea0.r, sp_reg);
		// a load list that contains the base register works from a copy
		base_in = (uw.d == S_MVR) && !pd &&
		          ((src.ea0.m == EM_AI || pi || src.ea0.m == EM_AD16 ||
		            src.ea0.m == EM_AX) && !(src.ea0.m == EM_AX && src.ea0.bs)) &&
		          src.ext1[{1'b1, src.ea0.r}];
		off = mv_act ? mv_off : (pd ? -{29'd0, sb} : 32'd0);
		nu.last = lastx;
		if (uw.d == S_MVR) begin
			nu.d_v = 1'b1; nu.d_reg = r;
		end
		else begin
			nu.a_src = OS_REG; nu.a_reg = r;
			// -(An) storing An itself writes An - size (68020 and later)
			if (pd && r == an) begin
				nu.op = OP_SUB; nu.sz = SZ_L;
				nu.a_src = OS_IMM; nu.imm = {29'd0, sb};
				nu.b_src = OS_REG; nu.b_reg = an;
			end
		end
		if (base_in && !mv_t11) begin
			// first: T11 = the address, then the transfers from T11
			uop_t p;
			p = '0;
			p.pc = nu.pc; p.npc = nu.npc; p.first = nu.first;
			p.ag = 1'b1;
			p.base_v = nu.base_v; p.base = nu.base;
			p.idx_v = nu.idx_v; p.idx = nu.idx; p.idx_l = nu.idx_l; p.scale = nu.scale;
			p.disp = nu.disp; p.mprog = nu.mprog;
			p.upd_v = 1'b1; p.upd_reg = R_T0 + 5'd11;
			p.exc = nu.exc;
			nu = p;
			n_mv_pre = 1'b1;
		end
		else begin
			if (mv_t11) begin
				nu.base_v = 1'b1; nu.base = R_T0 + 5'd11;
				nu.idx_v = 1'b0; nu.disp = 32'd0;
			end
			// address of this transfer
			nu.disp = nu.disp + off;
			if (pd) nu.disp = off;          // undo the generic -(An) offset
			nu.pinc = 1'b0;
			nu.upd_v = 1'b0;
			if (lastx && pd) begin
				nu.upd_v = 1'b1; nu.upd_reg = an;          // An = lowest address
			end
			if (lastx && pi) begin
				nu.pinc = 1'b1; nu.upd_v = 1'b1; nu.upd_reg = an;
				nu.upd_amt = (mv_act ? mv_cnt : 8'd0) + {5'd0, sb};
			end
		end
	end

	// a pointer load replaces the uop this cycle
	if (n_ptr) begin
		uop_t p;
		p = '0;
		p.pc = nu.pc; p.npc = nu.npc; p.first = nu.first; p.last = 1'b0;
		p.op = OP_MOV; p.sz = SZ_L; p.msz = SZ_L; p.ccr_we = 5'd0;
		p.a_src = OS_MEM; p.b_src = OS_ZERO;
		p.ag = 1'b1;
		p.base_v = nu.base_v; p.base = nu.base;
		p.idx_v = nu.idx_v; p.idx = nu.idx; p.idx_l = nu.idx_l; p.scale = nu.scale;
		p.disp = nu.disp;
		p.mem = M_LD; p.mprog = nu.mprog;
		p.d_v = 1'b1;
		p.d_reg = n_ptr_ea1 ? (R_T0 + 5'd12) : (R_T0 + 5'd13);
		p.exc = nu.exc;
		nu = p;
		n_def_set = 1'b0;
	end
end

//--------------------------------------------------------------------------
// sequencing.  uw_q is read one cycle ahead: the next address is chosen
// from the current word and the records, and the ROM output registered.
//--------------------------------------------------------------------------
wire step = uw_v && room_out && !flush && !exc_go;
wire emit = step && !n_jump;
wire stay = n_ptr || n_mv_pre || (uw.loop && !nu.last);   // same word again
wire done = emit && !stay && uw.last;                       // instruction ends

assign rq_pop = done && !exc_mode;

logic [8:0] na;          // next microcode address
logic       nv;
always_comb begin
	na = upc;
	nv = uw_v;
	if (exc_go) begin
		case (exc_kind)
			4'd1:    na = UA_EXC_FMT2;
			4'd2:    na = UA_EXC_FMT7;
			4'd3:    na = UA_EXC_IRQ;
			4'd5:    na = UA_EXC_IRQM;
			4'd4:    na = UA_EXC_RESET;
			default: na = UA_EXC_FMT0;
		endcase
		nv = 1'b1;
	end
	else if (flush) begin
		nv = 1'b0;
	end
	else if (step) begin
		if (n_jump)      na = jc_now ? uw.jt : upc + 9'd1;
		else if (stay)   na = upc;
		else if (!uw.last) na = upc + 9'd1;
		else if (!exc_mode && rq_n == 2'd2) na = rq1.rt;   // next instruction
		else nv = 1'b0;
	end
	else if (!uw_v && !exc_mode && rq_n != 2'd0) begin
		na = rq0.rt;                                         // first word
		nv = 1'b1;
	end
end

always_ff @(posedge clk) begin
	if (!nreset) begin
		upc      <= '0;
		uw_q     <= '0;
		uw_v     <= 1'b0;
		exc_mode <= 1'b0;
		first    <= 1'b1;
		ind0     <= 1'b0;
		ind1     <= 1'b0;
		def_v    <= 1'b0;
		def_reg  <= '0;
		def_amt  <= '0;
		ssp      <= R_ISP;
		mv_act   <= 1'b0;
		mv_mask  <= '0;
		mv_off   <= '0;
		mv_cnt   <= '0;
		mv_t11   <= 1'b0;
		uq_n     <= 2'd0;
		uq0      <= '0;
		uq1      <= '0;
	end
	else begin
		upc  <= na;
		uw_q <= ucode_rom(na);
		uw_v <= nv;

		// uop FIFO
		case ({emit, uq_pop && uq_n != 2'd0})
			2'b01: begin uq0 <= uq1; uq_n <= uq_n - 2'd1; end
			2'b10: begin
				if (uq_n == 2'd0) uq0 <= nu; else uq1 <= nu;
				uq_n <= uq_n + 2'd1;
			end
			2'b11: begin
				if (uq_n == 2'd1) uq0 <= nu;
				else begin uq0 <= uq1; uq1 <= nu; end
			end
			default: ;
		endcase

		if (emit) begin
			first <= 1'b0;
			if (n_ptr) begin
				if (n_ptr_ea1) ind1 <= 1'b1; else ind0 <= 1'b1;
			end
			else if (n_mv_pre) begin
				mv_t11 <= 1'b1;
			end
			else if (uw.loop && !nu.last) begin
				mv_act  <= 1'b1;
				mv_mask <= n_mv_rest;
				mv_off  <= (mv_act ? mv_off : ((src.ea0.m == EM_APD) ?
				            -{29'd0, (src.sz == SZ_L) ? 3'd4 : 3'd2} : 32'd0)) +
				           ((src.ea0.m == EM_APD) ? -{29'd0, (src.sz == SZ_L) ? 3'd4 : 3'd2}
				                                  :  {29'd0, (src.sz == SZ_L) ? 3'd4 : 3'd2});
				mv_cnt  <= (mv_act ? mv_cnt : 8'd0) + ((src.sz == SZ_L) ? 8'd4 : 8'd2);
			end
			else if (n_def_set) begin
				def_v   <= 1'b1;
				def_reg <= n_def_reg;
				def_amt <= n_def_amt;
			end
		end
		if (done) begin
			// the next instruction starts with clean per-instruction state
			first  <= 1'b1;
			ind0   <= 1'b0;
			ind1   <= 1'b0;
			def_v  <= 1'b0;
			mv_act <= 1'b0;
			mv_t11 <= 1'b0;
			if (exc_mode) exc_mode <= 1'b0;
		end

		if (flush) begin
			first <= 1'b1;
			ind0  <= 1'b0;
			ind1  <= 1'b0;
			def_v <= 1'b0;
			mv_act <= 1'b0;
			mv_t11 <= 1'b0;
			uq_n  <= 2'd0;
			if (!exc_go) exc_mode <= 1'b0;
		end
		if (exc_go) begin
			exc_mode <= 1'b1;
			first    <= 1'b1;
			ssp      <= exc_ssp;
			uq_n     <= 2'd0;
		end
	end
end

// jump condition of the current word (named for the sequencing block)
logic jc_now;
always_comb begin
	case (uw.jc)
		JC_ALWAYS:   jc_now = 1'b1;
		JC_EXT10:    jc_now = src.ext1[10];
		JC_EXT11:    jc_now = src.ext1[11];
		JC_EA0_MEM:  jc_now = is_mem(src.ea0.m);
		JC_EA0_DN:   jc_now = (src.ea0.m == EM_DN);
		JC_EA0_IMM:  jc_now = (src.ea0.m == EM_IMM);
		JC_SZ_L:     jc_now = (src.sz == SZ_L);
		JC_AY7:      jc_now = (src.opw[2:0] == 3'd7);
		JC_SUPER:    jc_now = smode;
		JC_MASK0:    jc_now = (src.ext1 == 16'd0);
		JC_X1A:      jc_now = src.ext1[15];
		JC_SZ_B:     jc_now = (src.sz == SZ_B);
		JC_CREG_RF:  jc_now = (src.ext1[11:0] == 12'h800) || (src.ext1[11:0] == 12'h803) ||
		                      (src.ext1[11:0] == 12'h804);
		default:     jc_now = 1'b0;
	endcase
end

endmodule
