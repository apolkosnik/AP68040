//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                            //
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
#(
	parameter bit DUAL = 1              // pair records' companions (s1_t)
)
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,

	// record FIFO from D1
	input  logic  [1:0] rq_n,
	input  dinst_t      rq0,
	input  dinst_t      rq1,
	output logic        rq_pop,
	output logic        rq_pop_e,   // an early form: implies rq_pop (D1's room)

	input  logic        smode,
	input  logic        master,     // M bit
	input  logic        trace,      // T1 or T0: every instruction alone

	input  logic        exc_go,
	input  logic        exc_pend,   // the back end prepares an exception entry: issue nothing
	input  logic  [3:0] exc_kind,
	input  logic  [4:0] exc_ssp,

	// uop FIFO towards the back end: uq0 is the head
	output logic  [1:0] uq_n,
	output uop_t        uq0,
	output s1_t         uq0s,       // ... and the second slot beside it
	input  logic        uq_pop,     // the back end takes uq0 this cycle
	output logic [19:0] dbg_pu      // observation only: dual-issue events (see the end)
);



//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
logic  [9:0] upc;               // address of uw_q
uword_t      uw_q;              // microcode word at upc (the ROM's output)
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
logic  [1:0] mv_fix;            // fix-up uops emitted after the last transfer

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
s1_t         uq1s;
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

// minus a small amount: the low bits negated, the rest the sign -- no
// 32-bit carry chain after the sequencer's selects
function automatic logic [31:0] neg5(input logic [4:0] x);
	neg5 = {{27{x != 5'd0}}, 5'd0 - x};
endfunction
function automatic logic [31:0] neg7(input logic [6:0] x);
	neg7 = {{25{x != 7'd0}}, 7'd0 - x};
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
logic  [9:0] n_jt;
logic        n_mv_pre;       // nu is the MOVEM address copy into T11
logic [15:0] n_mv_rest;      // mask after this transfer
logic        n_def_set;      // nu defers its source update
logic  [4:0] n_def_reg;
logic  [7:0] n_def_amt;

// FPU operand bytes for AG modes FEA / FUPD: opclass 010/011 by format (a
// byte through A7 moves it by two), FMOVE(M) of control registers four per
// register; the other FPU instructions move their address register by a
// run-time amount (ADDT0)
logic [6:0] fp_size;
always_comb begin
	logic [2:0] crl;
	crl = (src.ext1[12:10] == 3'd0) ? 3'b001 : src.ext1[12:10];
	case (src.ext1[15:13])
		3'b010, 3'b011: case (src.ext1[12:10])
			3'd0, 3'd1: fp_size = 7'd4;
			3'd4:       fp_size = 7'd2;
			3'd6:       fp_size = (src.ea0.r == 3'd7) ? 7'd2 : 7'd1;
			3'd5:       fp_size = 7'd8;
			default:    fp_size = 7'd12;
		endcase
		3'b100, 3'b101: fp_size = {3'd0, 2'(crl[0]) + 2'(crl[1]) + 2'(crl[2]), 2'b00};
		default: fp_size = 7'd0;
	endcase
end

logic        dbg_dm;             // (simulation: the merge and the
logic [31:0] dbg_dpre;           // displacement it was formed for;
logic        dbg_mvd;            // a MOVEM offset for AG, and the offset)
logic [31:0] dbg_mvo;
always_comb begin
	opsel_t oa, ob, od;
	ea_t    me;              // the uop's memory EA
	logic   me_v, me_ea1;
	logic [31:0] disp_pre;       // the uop's displacement before a deferred-update merge
	logic        dmerge;         // ... which applies (see the upd2 block)
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
	dbg_mvo   = '0;
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
		JC_FP_OC0:   jc_true = (src.ext1[15:13] == 3'b000);
		JC_FP_OC2:   jc_true = (src.ext1[15:13] == 3'b010);
		JC_FP_OC3:   jc_true = (src.ext1[15:13] == 3'b011);
		JC_FP_OC45:  jc_true = (src.ext1[15:14] == 2'b10);
		JC_FP_OC67:  jc_true = (src.ext1[15:14] == 2'b11);
		JC_FMT_1:    jc_true = (src.ext1[12:10] == 3'd6);
		JC_FMT_2:    jc_true = (src.ext1[12:10] == 3'd4);
		JC_FMT_8:    jc_true = (src.ext1[12:10] == 3'd5);
		JC_FMT_12:   jc_true = (src.ext1[12:10] == 3'd2 || src.ext1[12:10] == 3'd3);
		JC_FMT_7:    jc_true = (src.ext1[12:10] == 3'd7);
		JC_EA0_APD:  jc_true = (src.ea0.m == EM_APD);
		JC_EA0_AIP:  jc_true = (src.ea0.m == EM_AIP);
		JC_EXT13:    jc_true = src.ext1[13];
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
			S_NPC:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.pc + {27'd0, src.ilen, 1'b0}; end
			S_PC:    begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = src.pc; end
			S_CONST: begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {{16{uw.cval[15]}}, uw.cval}; end
			S_ZERO:  o.src = OS_ZERO;
			S_LD:    o.src = OS_MEM;
			// MOVEC register number, as the special register selector:
			// $000-$008 -> $00-$08, $800-$808 -> $10-$18
			S_CREG:  begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[11], src.ext1[3:0]}; end
			S_BFO: begin
				if (src.ext1[11]) o.r = dreg(src.ext1[8:6]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[10:6]}; end
			end
			S_BFW: begin
				if (src.ext1[5]) o.r = dreg(src.ext1[2:0]);
				else begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {27'd0, src.ext1[4:0]}; end
			end
			S_OPW:   begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {16'd0, src.opw}; end
			S_EXT1:  begin o.src = OS_IMM; o.imm_v = 1'b1; o.imm = {16'd0, src.ext1}; end
			S_FPDL:  o.r = dreg(src.ext1[6:4]);      // FMOVEM dynamic list register
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
	nu.imm_b  = {ob.imm[31], ob.imm[15:0]};   // (never a full immediate: 16 bits and the sign)
	nu.mfc    = (uw.mfc == MFC_EAP) ? MFC_NORM : uw.mfc;
	nu.dyn    = uw.dyn;
	nu.dynk   = uw.dynk;
	nu.mlock  = uw.lock;
	nu.mlocke = uw.locke;
	nu.br     = uw.br;
	nu.target = src.target;
	nu.pred   = src.pred && (uw.br != BR_NONE);
	nu.bst    = src.bst;
	nu.bhc    = src.bhc;
	nu.ser    = uw.ser;
	// FMOVEM / FMOVE of control registers to memory are T0 trace points
	// FMOVEM / FMOVE of control registers to memory are T0 trace points
	nu.t0cof  = src.t0 || (src.opw[15:6] == 10'b1111_0010_00 &&
	                       (src.ext1[15:13] == 3'b101 || src.ext1[15:13] == 3'b111));
	nu.pc     = src.pc;
	nu.ilen   = src.ilen;
	nu.ras    = src.ras;
	nu.exr     = exc_mode;
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
	if (uw.ag == AGM_EA0 || uw.ag == AGM_LEA0 || uw.ag == AGM_FEA) begin me = src.ea0; me_ea1 = 1'b0; end
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


	if (me_v || uw.ag == AGM_EA0 || uw.ag == AGM_EA1 || uw.ag == AGM_LEA0 ||
	    (uw.ag == AGM_FEA && src.ea0.m != EM_IMM)) begin
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
				nu.disp = neg5(amt);
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
	// ADDA.L (A0)+,A0 / CMPA.L -(A0),A0: the destination register is read
	// after the source EA's update (the 68000 family computes the EA first)
	nu.b_upd = nu.upd_v && !me_ea1 && (me.m == EM_AIP || me.m == EM_APD) &&
	           nu.b_src == OS_REG && nu.b_reg == nu.upd_reg;
	// MOVES Rn,<ea> computes the EA first: MOVES.L A6,-(A6) stores the
	// decremented A6 (WinUAE, the cputest oracle); MOVE reads the source first
	nu.a_upd = nu.upd_v && (me.m == EM_AIP || me.m == EM_APD) &&
	           src.opw[15:8] == 8'h0E && nu.a_src == OS_REG && nu.a_reg == nu.upd_reg;
	// the deferred update lands on the upd2 uop, memory or not (FRESTORE
	// commits (An)+ only after its frame check).  Its displacement merge is
	// applied after the MOVEM block (no MOVEM uop has upd2), so the two
	// additions are parallel rather than chained.
	disp_pre = nu.disp;
	dmerge   = 1'b0;
	if (uw.upd2 && def_v) begin
		logic same, m16;
		same = nu.ag && nu.base_v && nu.base == def_reg &&
		       (me.mi == 2'd0 || !(me_ea1 ? ind1 : ind0));
		// MOVE16 (Ax)+,(Ay)+ with Ax = Ay: both transfers use the original
		// address and the register advances once (WinUAE, the cputest
		// oracle); CMPM-style pairs see the first update
		m16 = (src.opw[15:3] == 13'h1EC4);
		if (same && m16) begin
			// the destination's own (An)+ is the only update
		end
		else if (same) begin
			dmerge = 1'b1;
			if (me.m == EM_AIP) nu.upd_amt = nu.upd_amt + def_amt;
		end
		if (!(same && (me.m == EM_AIP || me.m == EM_APD || m16))) begin
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
			DS_NQUICK: dsv = neg5(q[4:0]);      // q: 1..8
			DS_IMMC:   dsv = src.imm + {{16{uw.cval[15]}}, uw.cval};
			DS_SZB:    dsv = {27'd0, size_bytes(src.sz, 1'b0)};
			default:   dsv = {{16{uw.cval[15]}}, uw.cval};
		endcase
		case (uw.ag)
			AGM_PUSH: begin
				nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = sp_reg;
				nu.disp = neg5(size_bytes(msz, 1'b1));
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
			AGM_FEA: begin
				// the FPU operand's address into agw: (An)+ An, -(An) An -
				// size (the register moves with FUPD), #imm its place in the
				// instruction stream (D1: bd = PC + 4; a byte is the word's
				// low byte), else the EA
				nu.mem = M_NONE;
				if (src.ea0.m == EM_AIP || src.ea0.m == EM_APD) begin
					nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = areg(src.ea0.r, sp_reg);
					nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.disp = (src.ea0.m == EM_APD) ? neg7(fp_size) : 32'd0;
				end
				else if (src.ea0.m == EM_IMM) begin
					nu.ag = 1'b1; nu.base_v = 1'b0; nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.disp = src.ea0.bd + ((src.ext1[15:13] == 3'b010 &&
					                         src.ext1[12:10] == 3'd6) ? 32'd1 : 32'd0);
					nu.mprog = 1'b1;
				end
				nu.upd_v = !n_ptr; nu.upd_reg = rw;
			end
			AGM_FUPD: begin
				if (src.ea0.m == EM_AIP || src.ea0.m == EM_APD) begin
					nu.ag = 1'b1; nu.base_v = 1'b1; nu.base = areg(src.ea0.r, sp_reg);
					nu.idx_v = 1'b0; nu.pinc = 1'b0;
					nu.disp = (src.ea0.m == EM_APD) ? neg7(fp_size) : {25'd0, fp_size};
					nu.upd_v = 1'b1; nu.upd_reg = areg(src.ea0.r, sp_reg);
				end
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
		logic        pd, pi, lastx, base_in, fb_in, fx_in;
		logic  [4:0] an, xreg;
		logic [31:0] off;
		logic [15:0] low;
		logic  [3:0] k_an, k_x;
		logic        is_an, is_x;
		m  = mv_act ? mv_mask : src.ext1;
		pd = (src.ea0.m == EM_APD);
		pi = (src.ea0.m == EM_AIP);
		bi = 4'd0;
		for (int i = 15; i >= 0; i--) if (m[i]) bi = 4'(i);
		rn = pd ? (4'd15 - bi) : bi;
		r  = rn[3] ? areg(rn[2:0], sp_reg) : dreg(rn[2:0]);
		// (the lowest set bit cleared: m & ~low, no encoder on the way to
		// the last-transfer test, which pops the decoder's record)
		low   = m & (~m + 16'd1);
		n_mv_rest = m & ~low;
		lastx = (n_mv_rest == 16'd0);
		sb = (src.sz == SZ_L) ? 3'd4 : 3'd2;
		an = areg(src.ea0.r, sp_reg);
		// whether this transfer's register is An or the index register,
		// from the mask's lowest set bit directly (not through the encoder
		// and the register number: a timing path); the register numbers
		// are one-to-one, so r == an exactly when rn is {1, An}
		k_an  = pd ? (4'd15 - {1'b1, src.ea0.r}) : {1'b1, src.ea0.r};
		k_x   = {src.ea0.xa, src.ea0.xr};
		is_an = low[k_an];
		is_x  = low[k_x];
		// A load list that contains a register of the EA (the base, or the
		// index of an indexed mode) works from a copy of the address in
		// T11, and the loaded base/index commits only after the last
		// transfer (through T7/T6 and fix-up uops): a fault on a later
		// transfer restarts the instruction with the original EA.  (An)+
		// discards a loaded An: the final An is the incremented address.
		fb_in   = (uw.d == S_MVR) && !pd && !pi &&
		          (src.ea0.m == EM_AI || src.ea0.m == EM_AD16 ||
		           (src.ea0.m == EM_AX && !src.ea0.bs)) &&
		          src.ext1[{1'b1, src.ea0.r}];
		xreg    = src.ea0.xa ? areg(src.ea0.xr, sp_reg) : dreg(src.ea0.xr);
		fx_in   = (uw.d == S_MVR) && (src.ea0.m == EM_AX || src.ea0.m == EM_PCX) &&
		          !src.ea0.is && src.ext1[{src.ea0.xa, src.ea0.xr}] &&
		          !(fb_in && xreg == an);
		base_in = fb_in || fx_in ||
		          ((uw.d == S_MVR) && pi && src.ext1[{1'b1, src.ea0.r}]);
		off = mv_act ? mv_off : (pd ? -{29'd0, sb} : 32'd0);
		nu.last = lastx && !fb_in && !fx_in;
		if (uw.d == S_MVR) begin
			nu.d_v = 1'b1; nu.d_reg = r;
			if (fb_in && is_an)           nu.d_reg = R_T0 + 5'd7;
			else if (fx_in && !pd && is_x) nu.d_reg = R_T0 + 5'd6;
			else if (pi && is_an)         nu.d_v   = 1'b0;
		end
		else begin
			nu.a_src = OS_REG; nu.a_reg = r;
			// -(An) storing An itself writes An - size (68020 and later)
			if (pd && is_an) begin
				nu.op = OP_SUB; nu.sz = SZ_L;
				nu.a_src = OS_IMM; nu.imm = {29'd0, sb};
				nu.b_src = OS_REG; nu.b_reg = an;
			end
		end
		if (mv_act && mv_mask == 16'd0) begin
			// after the last transfer: commit the deferred base, then index
			uop_t p;
			logic fxb;
			fxb = fb_in && mv_fix == 2'd0;
			p = '0;
			p.pc = nu.pc; p.ilen = nu.ilen; p.exr = nu.exr; p.first = 1'b0;
			p.op = OP_MOV; p.sz = SZ_L; p.ccr_we = 5'd0;
			p.a_src = OS_REG; p.a_reg = fxb ? (R_T0 + 5'd7) : (R_T0 + 5'd6);
			p.b_src = OS_ZERO;
			p.d_v = 1'b1; p.d_reg = fxb ? an : xreg;
			p.last = !(fxb && fx_in);
			p.exc = nu.exc;
			nu = p;
			n_mv_rest = 16'd0;
		end
		else if (base_in && !mv_t11) begin
			// first: T11 = the address, then the transfers from T11
			uop_t p;
			p = '0;
			p.pc = nu.pc; p.ilen = nu.ilen; p.exr = nu.exr; p.first = nu.first;
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
			// MOVEM continuation (SSW CM): for the indexed and PC-relative
			// modes a transfer carries its offset from the calculated EA;
			// a fault stacks that EA, and RTE resumes from it
			if (src.ea0.m == EM_AX || src.ea0.m == EM_PC16 || src.ea0.m == EM_PCX) begin
				nu.cond   = 4'hC;
				nu.target = off;
			end
			// address of this transfer: the offset (within +-64) is added
			// by AG as the uop enters it, as a deferred update's is (off
			// the sequencer's EA selects)
			if (pd) nu.disp = off;          // undo the generic -(An) offset
			else begin
				nu.dmv = 1'b1;
				nu.dma = off[7:0];
				dbg_mvo = off;
			end
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

	// FMOVEM of control registers: a slot whose register is not in the
	// (static) list is no operation
	if (uw.dyn == 3'd4) begin
		logic [2:0] crl;
		crl = (src.ext1[12:10] == 3'd0) ? 3'b001 : src.ext1[12:10];
		if (!crl[2 - uw.dynk[1:0]]) begin
			nu.mem = M_NONE; nu.ag = 1'b0; nu.upd_v = 1'b0; nu.upd2_v = 1'b0;
			nu.d_v = 1'b0;
		end
		nu.dyn = 3'd0;
	end

	// FPU operand transfers read the EA's space: program space for
	// PC-relative and immediate sources
	if (uw.mfc == MFC_EAP)
		nu.mprog = (src.ea0.m == EM_PC16 || src.ea0.m == EM_PCX || src.ea0.m == EM_IMM);

	// (the addition itself is AG's, as the uop enters it: off the
	// microcode ROM's path to the uop queue.  No upd2 uop has an explicit
	// AG mode, so its displacement is still disp_pre -- checked below)
	dbg_dm   = dmerge;
	dbg_dpre = disp_pre;
	dbg_mvd  = nu.dmv;              // (MOVEM's offset, before a merge)
	if (dmerge) begin
		nu.dmv  = 1'b1;
		nu.dma  = def_amt;
	end

	// a pointer load replaces the uop this cycle
	if (n_ptr) begin
		uop_t p;
		p = '0;
		p.pc = nu.pc; p.ilen = nu.ilen; p.exr = nu.exr; p.first = nu.first; p.last = 1'b0;
		p.op = OP_MOV; p.sz = SZ_L; p.msz = SZ_L; p.ccr_we = 5'd0;
		p.a_src = OS_MEM; p.b_src = OS_ZERO;
		p.ag = 1'b1;
		p.base_v = nu.base_v; p.base = nu.base;
		p.idx_v = nu.idx_v; p.idx = nu.idx; p.idx_l = nu.idx_l; p.scale = nu.scale;
		p.disp = nu.disp; p.dmv = nu.dmv; p.dma = nu.dma;
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
//--------------------------------------------------------------------------
// the record's companion (D1's c_v/c_opw): beside the instruction's uop
// when that is the whole instruction and the pair may issue together,
// else a uop of its own in the next issue cycle (cq_v)
//--------------------------------------------------------------------------
s1_t   cs;                       // the companion, its registers named
always_comb begin
	cs = s1_dec(src.c_opw);
	cs.v = DUAL && src.c_v && !exc_mode;
	// (operand A and an address-register destination can be A7: the active
	// stack)
	if (cs.a_reg == 5'd15) cs.a_reg = sp_reg;
	if (cs.d_reg == 5'd15) cs.d_reg = sp_reg;
end
// what the uop beside a companion may be: one with no change of flow, no
// serialization or SR write, not the FPU's, and no upd2 (the second slot
// reads and writes the register file through the upd2 ports)
// (a change of flow too: the companion is the instruction the front end
// predicted after it, and goes with it when EX finds otherwise)
function automatic logic s0_ok(input uop_t u);
	s0_ok = u.first && u.last && (u.exc == 8'd0) && !u.ser &&
	        !u.upd2_v && (u.dyn == 3'd0) && !u.exr &&
	        (u.op != OP_FPU) && (u.op != OP_RTE) && (u.op != OP_RTEF) &&
	        (u.op != OP_MISC) && (u.op != OP_SRLOG) && (u.op != OP_SPW) &&
	        (u.op != OP_CHKSR) && (u.op != OP_IACKV);
endfunction
// what the uop beside a load companion may be: one with no memory access
// (the pair's access is the load's) and none of EX's traps
function automatic logic s0_ld(input uop_t u);
	s0_ld = (u.mem == M_NONE) && (u.op <= OP_DBCC) && (u.op != OP_TAS);
endfunction
// the companion touches no register the uop writes
function automatic logic s1_indep(input uop_t u, input s1_t c);
	logic [4:0] r [3];
	logic       rv [3];
	s1_indep = 1'b1;
	r[0] = c.a_reg; rv[0] = c.a_v && !c.a_imm;
	r[1] = c.b_reg; rv[1] = c.b_v;
	r[2] = c.d_reg; rv[2] = c.d_v || c.agop;
	for (int i = 0; i < 3; i++)
		if (rv[i] && ((u.d_v && u.d_reg == r[i]) || (u.upd_v && u.upd_reg == r[i])))
			s1_indep = 1'b0;
endfunction
// the companion as a uop of its own
// (the whole instruction: first and last, from the template t, uop_fl --
// Quartus 17.0 takes a member named first or last neither assigned in a
// function nor named in an assignment pattern, only assigned in an always
// block; uop_fl is passed in, as the front end reports a module variable
// read only inside a function as never read)
uop_t uop_fl;
always_comb begin
	uop_fl       = '0;
	uop_fl.first = 1'b1;
	uop_fl.last  = 1'b1;
end
function automatic uop_t s1_uop(input uop_t t, input s1_t c, input logic [31:0] pc, input ras_snap_t ras);
	uop_t u;
	u = t;
	u.pc    = pc;
	u.ilen  = 4'd1;
	u.ras   = ras;
	u.ras.k = 2'd0;
	if (c.ld) begin
		// the load's access, as the microcode has it: (Ay) the base
		u.ag     = 1'b1;
		u.base_v = 1'b1;
		u.base   = c.a_reg;
		u.mem    = M_LD;
		u.msz    = c.sz;
	end
	if (c.agop) begin
		// AG's, as the microcode has them: ADDQ/SUBQ base + disp (QUICK_A),
		// MOVEA the source as an index (VAL0: a word sign-extended)
		u.ag      = 1'b1;
		u.upd_v   = 1'b1;
		u.upd_reg = c.d_reg;
		if (c.imm != 8'd0) begin
			u.base_v = 1'b1; u.base = c.a_reg; u.disp = {{24{c.imm[7]}}, c.imm};
		end
		else begin
			u.idx_v = 1'b1; u.idx = c.a_reg; u.idx_l = !c.sxw;
		end
		return u;
	end
	case (c.op)
		S1_MOV:  u.op = OP_MOV;
		S1_ADD:  u.op = OP_ADD;
		S1_SUB:  u.op = OP_SUB;
		S1_CMP:  u.op = OP_CMP;
		S1_AND:  u.op = OP_AND;
		S1_OR:   u.op = OP_OR;
		S1_EOR:  u.op = OP_EOR;
		S1_NOT:  u.op = OP_NOT;
		S1_NEG:  u.op = OP_NEG;
		S1_CLR:  u.op = OP_CLR;
		S1_EXTB: u.op = OP_EXTB;
		S1_SWAP: u.op = OP_SWAP;
		default: u.op = OP_EXT;
	endcase
	u.sz     = c.sz;
	u.ccr_we = c.ccr_we;
	u.a_src  = c.ld ? OS_MEM : c.a_imm ? OS_IMM : c.a_v ? OS_REG : OS_ZERO;
	u.a_reg  = c.a_reg;
	u.imm    = {{24{c.imm[7]}}, c.imm};
	u.b_src  = c.b_v ? OS_REG : OS_ZERO;
	u.b_reg  = c.b_reg;
	u.d_v    = c.d_v;
	u.d_reg  = c.d_reg;
	s1_uop = u;
endfunction
logic  cq_v;                     // an unpaired companion waits to issue
uop_t  cq_u;
wire   c_emit = cq_v && room_out && !flush && !exc_go && !exc_pend;

wire step = uw_v && room_out && !flush && !exc_go && !exc_pend && !cq_v;
wire emit = step && !n_jump;
wire stay = n_ptr || n_mv_pre || (uw.loop && !nu.last);   // same word again
wire done = emit && !stay && uw.last;                       // instruction ends
// the companion issues beside the uop that ends the instruction
wire c_pair = done && cs.v && !trace && s0_ok(nu) && s1_indep(nu, cs) && (!cs.ld || s0_ld(nu));
s1_t ns;                          // the slot beside the uop written
always_comb begin
	ns = cs;
	ns.v = c_pair;
end
// (the uop written: the sequencer's, or the waiting companion's)
uop_t wu;
s1_t  wus;
wire  wv = emit || c_emit;
always_comb begin
	wu  = c_emit ? cq_u : nu;
	wus = c_emit ? '0 : ns;
end

assign rq_pop = done && !exc_mode;
// The pop from the word's own bits and the record's EA modes, not the uop
// built from them (D1's room, a timing path): a plain last word -- no
// jump, no MOVEM loop -- of an instruction with no memory-indirect EA (no
// pointer load) ends it whenever it steps.  It implies rq_pop; the rare
// pop it misses only costs D1 a clock.
assign rq_pop_e = step && !exc_mode && uw.last && (uw.jc == JC_NEVER) && !uw.loop &&
                  (rq0.ea0.mi == 2'd0) && (rq0.ea1.mi == 2'd0);
// synthesis translate_off
always_ff @(posedge clk)
	if (rq_pop_e && !rq_pop) begin
		$display("FAIL: useq: the early pop without the pop");
		$finish;
	end
// synthesis translate_on

logic [9:0] na;          // next microcode address
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
		if (n_jump)      na = jc_now ? uw.jt : upc + 10'd1;
		else if (stay)   na = upc;
		else if (!uw.last) na = upc + 10'd1;
		else if (!exc_mode && rq_n[1]) na = rq1.rt;        // next instruction
		else nv = 1'b0;
	end
	else if (!uw_v && !exc_mode && !exc_pend && rq_n != 2'd0) begin
		na = rq0.rt;                                         // first word
		nv = 1'b1;
	end
end

// the microcode ROM in block RAM: its address register is the sequencing
// register (uw_q is the word at upc, read one cycle ahead as before)
(* romstyle = "M10K" *) (* rom_style = "block" *) logic [$bits(uword_t)-1:0] urom [0:1023];
initial begin
`include "gen/ap68040_ucode_init.svh"
end
logic [$bits(uword_t)-1:0] uw_raw;
always_ff @(posedge clk) uw_raw <= urom[na];
assign uw_q = uword_t'(uw_raw);

always_ff @(posedge clk) begin
	if (!nreset) begin
		upc      <= '0;
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
		mv_fix   <= 2'd0;
		uq_n     <= 2'd0;
		uq0      <= '0;
		uq1      <= '0;
		uq0s     <= '0;
		uq1s     <= '0;
		cq_v     <= 1'b0;
		cq_u     <= '0;
	end
	else begin
		upc  <= na;
		uw_v <= nv;

		// uop FIFO: both outcomes formed without the pop (the back end's
		// stall, a late signal), which selects between them last
		if (uq_pop && uq_n != 2'd0) begin
			uq0  <= (wv && uq_n == 2'd1) ? wu : uq1;
			uq1  <= (wv && uq_n == 2'd2) ? wu : uq1;
			uq0s <= (wv && uq_n == 2'd1) ? wus : uq1s;
			uq1s <= (wv && uq_n == 2'd2) ? wus : uq1s;
			uq_n <= wv ? uq_n : uq_n - 2'd1;
		end
		else begin
			uq0  <= (wv && uq_n == 2'd0) ? wu : uq0;
			uq1  <= (wv && uq_n == 2'd1) ? wu : uq1;
			uq0s <= (wv && uq_n == 2'd0) ? wus : uq0s;
			uq1s <= (wv && uq_n == 2'd1) ? wus : uq1s;
			uq_n <= wv ? uq_n + 2'd1 : uq_n;
		end
		// a companion that did not pair issues next on its own
		if (c_emit) cq_v <= 1'b0;
		if (done && cs.v && !c_pair) begin
			cq_v <= 1'b1;
			cq_u <= s1_uop(uop_fl, cs, src.c_tk ? src.target : src.pc + {27'd0, src.ilen, 1'b0}, src.ras);
		end

		if (emit) begin
			first <= 1'b0;
			if (n_ptr) begin
				if (n_ptr_ea1) ind1 <= 1'b1; else ind0 <= 1'b1;
			end
			else if (n_mv_pre) begin
				mv_t11 <= 1'b1;
			end
			else if (uw.loop && !nu.last) begin
				if (mv_act && mv_mask == 16'd0) mv_fix <= mv_fix + 2'd1;
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
			mv_fix <= 2'd0;
			if (exc_mode) exc_mode <= 1'b0;
		end

		if (flush) begin
			first <= 1'b1;
			ind0  <= 1'b0;
			ind1  <= 1'b0;
			def_v <= 1'b0;
			mv_act <= 1'b0;
			mv_t11 <= 1'b0;
			mv_fix <= 2'd0;
			uq_n  <= 2'd0;
			cq_v  <= 1'b0;
			if (!exc_go) exc_mode <= 1'b0;
		end
		if (exc_go) begin
			exc_mode <= 1'b1;
			first    <= 1'b1;
			ssp      <= exc_ssp;
			uq_n     <= 2'd0;
			cq_v     <= 1'b0;
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
		JC_EA0_AN:   jc_now = (src.ea0.m == EM_AN);
		JC_EA0_APD:  jc_now = (src.ea0.m == EM_APD);
		JC_EA0_AIP:  jc_now = (src.ea0.m == EM_AIP);
		JC_EXT13:    jc_now = src.ext1[13];
		JC_FP_OC0:   jc_now = (src.ext1[15:13] == 3'b000);
		JC_FP_OC2:   jc_now = (src.ext1[15:13] == 3'b010);
		JC_FP_OC3:   jc_now = (src.ext1[15:13] == 3'b011);
		JC_FP_OC45:  jc_now = (src.ext1[15:14] == 2'b10);
		JC_FP_OC67:  jc_now = (src.ext1[15:14] == 2'b11);
		JC_FMT_1:    jc_now = (src.ext1[12:10] == 3'd6);
		JC_FMT_2:    jc_now = (src.ext1[12:10] == 3'd4);
		JC_FMT_8:    jc_now = (src.ext1[12:10] == 3'd5);
		JC_FMT_12:   jc_now = (src.ext1[12:10] == 3'd2 || src.ext1[12:10] == 3'd3);
		JC_FMT_7:    jc_now = (src.ext1[12:10] == 3'd7);
		JC_CREG_RF:  jc_now = (src.ext1[11:0] == 12'h800) || (src.ext1[11:0] == 12'h803) ||
		                      (src.ext1[11:0] == 12'h804);
		default:     jc_now = 1'b0;
	endcase
end

// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && emit && (dbg_mvd && (dbg_dm || dbg_mvo[31:7] != {25{dbg_mvo[7]}}))) begin
		$display("FAIL: useq: a MOVEM offset %h for AG (merge %b)", dbg_mvo, dbg_dm);
		$finish;
	end
// every second-slot instruction the microcode runs: s1_dec gives its uop
always_ff @(posedge clk)
	if (nreset && emit && nu.first && nu.last && !exc_mode && src.exc == 8'd0 &&
	    s1_dec(src.opw).v) begin
		s1_t  c;
		uop_t r;
		c = s1_dec(src.opw);
		if (c.a_reg == 5'd15) c.a_reg = sp_reg;
		if (c.d_reg == 5'd15) c.d_reg = sp_reg;
		r = s1_uop(uop_fl, c, nu.pc, nu.ras);
		if (c.agop ? (!nu.ag || !nu.upd_v || nu.upd_reg != r.upd_reg || nu.d_v || nu.ccr_we != 5'd0 ||
		              nu.mem != M_NONE || nu.br != BR_NONE || nu.upd2_v || nu.pinc || nu.ser ||
		              nu.base_v != r.base_v || (r.base_v && (nu.base != r.base || nu.disp != r.disp)) ||
		              nu.idx_v != r.idx_v || (r.idx_v && (nu.idx != r.idx || nu.idx_l != r.idx_l ||
		                                                  nu.scale != 2'd0 || nu.disp != 32'd0)) ||
		              nu.ilen != 4'd1) :
		    c.ld ? (r.op != nu.op || r.sz != nu.sz || r.ccr_we != nu.ccr_we || nu.a_src != OS_MEM ||
		            nu.a_sxw || r.b_src != nu.b_src || (r.b_src == OS_REG && r.b_reg != nu.b_reg) ||
		            r.d_v != nu.d_v || (r.d_v && r.d_reg != nu.d_reg) ||
		            nu.mem != M_LD || nu.msz != r.msz || !nu.ag || !nu.base_v || nu.base != r.base ||
		            nu.idx_v || nu.disp != 32'd0 || nu.dmv || nu.pinc || nu.upd_v || nu.upd2_v ||
		            nu.mfc != r.mfc || nu.mlock || nu.mprog || nu.br != BR_NONE ||
		            nu.ser || nu.dyn != 3'd0 || nu.ilen != 4'd1) :
		    (r.op != nu.op || r.sz != nu.sz || r.ccr_we != nu.ccr_we ||
		    r.a_src != nu.a_src || (r.a_src == OS_REG && r.a_reg != nu.a_reg) ||
		    (r.a_src == OS_IMM && r.imm != nu.imm) || nu.a_sxw ||
		    r.b_src != nu.b_src || (r.b_src == OS_REG && r.b_reg != nu.b_reg) ||
		    r.d_v != nu.d_v || (r.d_v && r.d_reg != nu.d_reg) ||
		    nu.mem != M_NONE || nu.ag || nu.br != BR_NONE || nu.upd_v || nu.upd2_v ||
		    nu.ser || nu.dyn != 3'd0 || nu.ilen != 4'd1)) begin
			$display("FAIL: useq: second-slot decode of %04x: op %0d/%0d sz %0d/%0d ccr %h/%h a %0d:%0d/%0d:%0d imm %h/%h b %0d:%0d/%0d:%0d d %b:%0d/%b:%0d (pc %h)",
			         src.opw, r.op, nu.op, r.sz, nu.sz, r.ccr_we, nu.ccr_we,
			         r.a_src, r.a_reg, nu.a_src, nu.a_reg, r.imm, nu.imm,
			         r.b_src, r.b_reg, nu.b_src, nu.b_reg, r.d_v, r.d_reg, nu.d_v, nu.d_reg, nu.pc);
			$finish;
		end
	end
always_ff @(posedge clk)
	if (nreset && emit && dbg_dm && nu.disp != dbg_dpre) begin
		$display("FAIL: useq: a merged uop's displacement %h changed from %h", nu.disp, dbg_dpre);
		$finish;
	end
// synthesis translate_on

// Dual-issue events for a logic analyser's counters, registered, one-clock
// strobes as an instruction with a companion ends:
//   [16] cv     the instruction carries a companion
//   [17] pair   ... and it issues beside the instruction's last uop
//   [18] dep    ... it does not: the uop writes a register it touches
//   [19] s0     ... it does not: that uop may not have one beside it (or
//               trace)
// with the instruction's operation word [15:0].
wire pu_split = done && cs.v && !c_pair;
wire pu_dep   = pu_split && !trace && s0_ok(nu) && (!cs.ld || s0_ld(nu)) && !s1_indep(nu, cs);
always_ff @(posedge clk)
	dbg_pu <= {pu_split && !pu_dep, pu_dep, c_pair, done && cs.v, src.opw};

endmodule
