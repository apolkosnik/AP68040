//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_decode.sv - D1: instruction parse                                 //
//                                                                          //
//   operation word | fixed extension words | immediate | EA0 ext | EA1 ext //
//                                                                          //
// Fast path: one instruction per cycle.  The queue's predecode of slot 0  //
// gives the length assuming brief index extensions; D1 only checks bit 8  //
// of the words at the index extension positions.  Slow path (a full-      //
// format extension, an FPU immediate, or more than seven words): one      //
// phase per part -- operation word with fixed words and immediate, then   //
// EA0's extension, then EA1's -- each sized from the word at the head.    //
//                                                                          //
// Records go to D2 through a two-entry FIFO, so neither side depends on   //
// the other combinationally.  PC-relative displacements, branch targets   //
// and the next PC are added here.  Branches are predicted here (Bcc by    //
// the static rule and a history table, JSR/JMP absolute and PC-relative,  //
// RTS by a return stack) and redirect fetch, unless the fetch BTB already //
// took the same branch; D1 checks every word the BTB flagged.  Illegal,  //
// line A/F, privilege, TRAP #n, odd PC and fetch faults become records    //
// carrying their exception vector.                                        //
//--------------------------------------------------------------------------//

module ap68040_decode
	import ap68040_pkg::*, ap68040_upkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,          // back end redirect: drop everything
	input  logic        ras_rv,         // ... with the return stack to restore
	input  logic  [2:0] ras_rtp,
	input  logic  [3:0] ras_rn,

	input  logic [15:0] win [8],
	input  logic  [7:0] win_flt,
	input  logic  [7:0] win_fdem,     // the faulted fetch was a demand fetch
	input  logic  [7:0] win_fatc,     // ... an ATC (MMU) fault
	input  logic  [7:0] win_bt,       // the word ends a branch the fetch BTB took
	input  logic [31:0] bt_tgt,       // its target (the first flagged word's)
	input  pd_t         pd0,
	input  logic  [3:0] qcnt,
	input  logic [31:0] qpc,
	input  logic        q_odd,
	input  logic        smode,

	output logic  [2:0] consume,
	output logic        d_redir_v,
	output logic [31:0] d_redir_pc,

	// record FIFO towards D2: rq0 is the head
	output logic  [1:0] rq_n,
	output dinst_t      rq0,
	output dinst_t      rq1,
	input  logic        rq_pop,

	// Bcc history training (from EX)
	input  logic        bht_we,
	input  logic  [7:0] bht_wa,
	input  logic        bht_dis,        // ... the branch went against the static rule

	// BTB maintenance (the fetch unit's table)
	output logic        btb_we,
	output logic [BTB_AW-1:0] btb_wi,
	output logic        btb_wv,
	output logic [BTB_TW-1:0] btb_wtag,
	output logic        btb_wslot,
	output logic  [1:0] btb_wkind,
	output logic [30:0] btb_wtgt,
	output logic [30:0] ras_o [8],    // the return stack, for the fetch's
	output logic  [2:0] ras_tp_o,
	output logic  [3:0] ras_n_o
);

`include "gen/ap68040_dec_pla.svh"

//--------------------------------------------------------------------------
// helpers
//--------------------------------------------------------------------------
function automatic logic [2:0] ea_len(input logic [3:0] idx, input logic [15:0] w,
                                      input logic [2:0] immlen);
	logic [2:0] n;
	case (idx)
		EM_AD16, EM_ABSW, EM_PC16: n = 3'd1;
		EM_ABSL: n = 3'd2;
		EM_IMM:  n = immlen;
		EM_AX, EM_PCX: begin
			n = 3'd1;
			if (w[8]) begin
				case (w[5:4])
					2'd2: n = n + 3'd1;
					2'd3: n = n + 3'd2;
					default: ;
				endcase
				case (w[1:0])
					2'd2: n = n + 3'd1;
					2'd3: n = n + 3'd2;
					default: ;
				endcase
			end
		end
		default: n = 3'd0;
	endcase
	ea_len = n;
endfunction

function automatic logic [31:0] sx16(input logic [15:0] v);
	sx16 = {{16{v[15]}}, v};
endfunction

// EA record from its extension words w0.. at instruction word offset p;
// pcb = pc + 2*p (the extension word's address, the PC-relative base),
// already added into bd for PC-relative modes
function automatic ea_t mk_ea(input logic [3:0] idx, input logic [2:0] r,
                              input logic [3:0] p, input logic [2:0] immlen,
                              input logic [31:0] pcb,
                              input logic [15:0] w0, input logic [15:0] w1,
                              input logic [15:0] w2, input logic [15:0] w3,
                              input logic [15:0] w4);
	ea_t e;
	e = '0;
	e.m    = idx;
	e.r    = r;
	e.xoff = p[2:0];
	case (idx)
		EM_AD16, EM_ABSW: e.bd = sx16(w0);
		EM_PC16: e.bd = pcb + sx16(w0);
		EM_ABSL: e.bd = {w0, w1};
		EM_IMM:  e.bd = (immlen == 3'd1) ? {16'd0, w0} : {w0, w1};
		EM_AX, EM_PCX: begin
			logic [31:0] bd;
			e.xa = w0[15];
			e.xr = w0[14:12];
			e.xl = w0[11];
			e.sc = w0[10:9];
			if (!w0[8]) begin
				bd = {{24{w0[7]}}, w0[7:0]};
			end
			else begin
				logic [15:0] o0, o1;
				e.bs = w0[7];
				e.is = w0[6];
				case (w0[5:4])
					2'd2: begin bd = sx16(w1);  o0 = w2; o1 = w3; end
					2'd3: begin bd = {w1, w2};  o0 = w3; o1 = w4; end
					default: begin bd = 32'd0; o0 = w1; o1 = w2; end
				endcase
				if (w0[1:0] != 2'd0)
					e.mi = (w0[2] && !w0[6]) ? 2'd2 : 2'd1;
				case (w0[1:0])
					2'd2: e.od = sx16(o0);
					2'd3: e.od = {o0, o1};
					default: e.od = 32'd0;
				endcase
			end
			// PC-relative with the base not suppressed: the PC is added here
			e.bd = (idx == EM_PCX && !e.bs) ? (pcb + bd) : bd;
		end
		default: ;
	endcase
	mk_ea = e;
endfunction

// effective first micro-instruction given the routine's leading jump
function automatic logic jcond(input logic [4:0] jc, input logic [3:0] e0,
                               input logic [3:0] e1, input logic [15:0] ext1,
                               input logic [1:0] s, input logic [2:0] ry);
	logic m0, m1;
	m0 = (e0 >= EM_AI) && (e0 != EM_IMM) && (e0 != EM_NONE);
	m1 = (e1 >= EM_AI) && (e1 != EM_IMM) && (e1 != EM_NONE);
	case (jc)
		JC_ALWAYS:   jcond = 1'b1;
		JC_EA0_MEM:  jcond = m0;
		JC_EA1_MEM:  jcond = m1;
		JC_EA0_REG:  jcond = (e0 == EM_DN) || (e0 == EM_AN);
		JC_EA0_DN:   jcond = (e0 == EM_DN);
		JC_EA0_AN:   jcond = (e0 == EM_AN);
		JC_EA0_IMM:  jcond = (e0 == EM_IMM);
		JC_NOT_EA0_MEM: jcond = !m0;
		JC_EXT11:    jcond = ext1[11];
		JC_EXT10:    jcond = ext1[10];
		JC_SZ_L:     jcond = (s == SZ_L);
		JC_SZ_B:     jcond = (s == SZ_B);
		JC_AY7:      jcond = (ry == 3'd7);
		JC_BOTH_MEM: jcond = m0 && m1;
		JC_MASK0:    jcond = (ext1 == 16'd0);
		JC_X1A:      jcond = ext1[15];
		JC_CREG_RF:  jcond = (ext1[11:0] == 12'h800) || (ext1[11:0] == 12'h803) ||
		                     (ext1[11:0] == 12'h804);
		JC_FP_OC0:   jcond = (ext1[15:13] == 3'b000);
		default:     jcond = 1'b0;
	endcase
endfunction

// FPU immediate length from the command word's source format
function automatic logic [2:0] fp_immlen(input logic [15:0] cmd);
	logic [2:0] crl;
	crl = (cmd[12:10] == 3'd0) ? 3'b001 : cmd[12:10];
	if (cmd[15:13] == 3'b100)
		// FMOVE(M) #imm to control registers: a long word each
		fp_immlen = {2'(crl[0]) + 2'(crl[1]) + 2'(crl[2]), 1'b0};
	else if (cmd[15:13] != 3'b010)
		fp_immlen = 3'd0;
	else case (cmd[12:10])
		3'd0, 3'd1: fp_immlen = 3'd2;   // L, S
		3'd2, 3'd3: fp_immlen = 3'd6;   // X, P
		3'd4, 3'd6: fp_immlen = 3'd1;   // W, B
		3'd5:       fp_immlen = 3'd4;   // D
		default:    fp_immlen = 3'd0;
	endcase
endfunction

function automatic logic has_ext(input logic [3:0] i);
	has_ext = (i != EM_NONE) && (i >= EM_AD16);
endfunction

//--------------------------------------------------------------------------
// the instruction at the head: attributes of its decoder entry
//--------------------------------------------------------------------------
pla_t a0;
always_comb a0 = ent_attr(pd0.ent, pd0.legal);
wire [15:0] opw = win[0];

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
typedef enum logic [1:0] { PH_IDLE, PH_EA0, PH_EA1 } ph_t;
ph_t         ph;
dinst_t      part;          // record under construction (slow path)
pd_t         part_pd;
logic  [3:0] part_len;      // words consumed so far
dinst_t      rq2;           // third record (internal)
logic  [2:0] part_imml;     // EA0 immediate length (slow path)
logic        hold_redir;    // a predicted branch is redirecting fetch

//--------------------------------------------------------------------------
// fast path
//--------------------------------------------------------------------------
wire  [2:0] fb    = pd0.b;
wire  [2:0] fp1   = pd0.p1;
wire        full0 = pd0.x0 && win[fb][8];
wire        full1 = pd0.x1 && win[fp1][8];
wire        fast  = pd0.legal && !pd0.slow && !full0 && !full1 && (pd0.tot <= 4'd7);

//--------------------------------------------------------------------------
// record assembly
//--------------------------------------------------------------------------
dinst_t      nrec;
logic        go;            // a record is produced
logic        go_part;       // a slow-path phase completes (no record yet)
logic  [3:0] use_n;
logic        flt;

// A fetch fault is raised for the lowest faulted word of the window (the
// words before it are good, so it is the first faulted word an instruction
// uses).  Codes: EXC_IFB a bus error, EXC_IFA an ATC fault, on a demand
// fetch; EXC_IFS/EXC_IFSA the same on a speculative prefetch, which the
// back end refetches on demand first.  The record's target carries the
// faulting word's address (the format $7 fault address).
logic  [7:0] f_code;
logic [31:0] f_addr;
always_comb begin
	logic [2:0] fi;
	fi = 3'd0;
	for (int i = 7; i >= 0; i--) if (win_flt[i]) fi = 3'(i);
	f_addr = qpc + {28'd0, fi, 1'b0};
	f_code = win_fdem[fi] ? (win_fatc[fi] ? EXC_IFA : EXC_IFB)
	                      : (win_fatc[fi] ? EXC_IFSA : EXC_IFS);
end
logic [31:0] ntarget;
// ntarget == bt_tgt without the target adder: a PC-relative target
// qpc + 2 + disp equals bt_tgt exactly when disp == bt_tgt - qpc - 2,
// which is formed from registers alongside the decode (the BTB check
// ends in btb_we and d_redir_v)
logic        teq;
logic        jsr_k;       // a JSR to a known target (see nrec_q)
// a call (BSR, JSR) or a return (RTS), for the return stacks: from the
// decoder's entry (the record's routine is past a first jump D1 resolved,
// which is a longer path), kept from the first part of a multi-part decode
logic        part_call, part_ret;
wire         rcall = (ph == PH_IDLE) ? (a0.rt == UA_BSR || a0.rt == UA_JSR) : part_call;
wire         rret  = (ph == PH_IDLE) ? (a0.rt == UA_RTS) : part_ret;
wire  [31:0] bt_d2 = bt_tgt - qpc - 32'd2;
// synthesis translate_off
always @(posedge clk)
	if (nreset && push && ph == PH_IDLE && bt_end && nrec.pred && teq != (ntarget == bt_tgt))
		$display("DECODE: teq %b disagrees with the target compare at %h", teq, qpc);
// synthesis translate_on
ph_t         nph;

// Branch prediction at D1 (the back end verifies every prediction in EX):
//  * Bcc: the static rule (backward taken), overruled by a 256-entry
//    history table of 2-bit counters indexed by PC[8:1].  A counter counts
//    how often the branch disagreed with the static rule, so the power-up
//    zeros mean "static"; at 2 or more the prediction is inverted.
//  * JSR/JMP to an absolute or PC-relative address: the target.
//  * RTS: an 8-entry return stack, pushed by BSR/JSR records, popped by
//    RTS records (not repaired after a redirect: a wrong entry only costs
//    the redirect EX would have made anyway).
logic  [1:0] bht_q;
ap68040_lutram #(.AW(8), .DW(2)) bht (
	.clk(clk), .we(bw_we), .waddr(bw_wa), .wdata(bw_wd), .raddr(qpc[8:1]), .q(bht_q)
);
// training counts from the table's current value (a copy read at the
// training address), not from the value D1 read when it decoded the
// branch: two passes of a short loop are often decoded before the first
// one is trained
// The update is written a cycle later from registers of its own (the read
// address is not also the write address); a training of the same counter
// in that cycle takes the value being written.
logic  [1:0] bht_t, bht_c, bht_nd;
logic        bw_we;
logic  [7:0] bw_wa;
logic  [1:0] bw_wd;
ap68040_lutram #(.AW(8), .DW(2)) bht_tr (
	.clk(clk), .we(bw_we), .waddr(bw_wa), .wdata(bw_wd), .raddr(bht_wa), .q(bht_t)
);
assign bht_c  = (bw_we && bw_wa == bht_wa) ? bw_wd : bht_t;
assign bht_nd = bht_dis ? ((bht_c == 2'd3) ? 2'd3 : bht_c + 2'd1)
                        : ((bht_c == 2'd0) ? 2'd0 : bht_c - 2'd1);
always_ff @(posedge clk) begin
	bw_we <= nreset && bht_we;
	bw_wa <= bht_wa;
	bw_wd <= bht_nd;
end
logic [31:0] ras [8];
logic  [2:0] ras_tp;           // the top entry
logic  [3:0] ras_n;            // entries held (0..8)

always_comb begin
	logic [31:0] disp;
	logic        nredir;
	nrec    = '0;
	go      = 1'b0;
	go_part = 1'b0;
	use_n   = 4'd0;
	flt     = 1'b0;
	nredir  = 1'b0;
	ntarget = '0;
	teq     = 1'b0;
	jsr_k   = 1'b0;
	disp    = '0;
	nph     = ph;

	if (ph != PH_IDLE) begin
		// slow path: the EA extension at the head
		logic [3:0] i;
		logic [2:0] l, il;
		i  = (ph == PH_EA0) ? part_pd.i0 : part_pd.i1;
		il = (ph == PH_EA0) ? part_imml : 3'd0;
		l  = ea_len(i, win[0], il);
		nrec = part;
		if ({1'b0, l} <= qcnt) begin
			use_n = {1'b0, l};
			flt   = |(win_flt & ((8'd1 << l) - 8'd1));
			if (ph == PH_EA0)
				nrec.ea0 = mk_ea(i, part.opw[2:0], part_len, il,
				                 part.pc + {27'd0, part_len, 1'b0},
				                 win[0], win[1], win[2], win[3], win[4]);
			else
				nrec.ea1 = mk_ea(i, part.opw[11:9], part_len, 3'd0,
				                 part.pc + {27'd0, part_len, 1'b0},
				                 win[0], win[1], win[2], win[3], win[4]);
			// an FPU immediate is read from the instruction stream by the
			// microcode: its address follows the extension word (the
			// operation word says FPU: the record's routine is past the
			// first word D1 resolves)
			if (ph == PH_EA0 && nrec.ea0.m == EM_IMM && part.opw[15:6] == 10'b1111_0010_00)
				nrec.ea0.bd = part.pc + 32'd4;
			if (ph == PH_EA0 && has_ext(part_pd.i1) && !flt) begin
				go_part = 1'b1;
				nph     = PH_EA1;
			end
			else begin
				go  = 1'b1;
				nph = PH_IDLE;
			end
			nrec.npc = part.pc + {27'd0, part_len + {1'b0, l}, 1'b0};
			if (flt) begin
				nrec.exc    = f_code;
				nrec.target = f_addr;
				nrec.rt     = UA_DEC_EXC;
			end
		end
	end
	else if (q_odd && qcnt == 4'd0) begin
		// the stream starts at an odd address: address error
		go       = 1'b1;
		nrec.pc  = qpc;
		nrec.npc = qpc;
		nrec.exc = 8'd3;
		nrec.rt  = UA_DEC_EXC;
	end
	else if (qcnt != 4'd0) begin
		logic [2:0] pi, nimm;
		nrec.pc    = qpc;
		nrec.opw   = opw;
		nrec.ext1  = win[1];
		nrec.ext2  = win[2];
		nrec.sz    = pd0.sz;
		nrec.eop   = a0.eop;
		nrec.econd = a0.econd;
		nrec.ccr   = a0.ccr;
		nrec.t0    = a0.t0;
		pi   = 3'd1 + {1'b0, a0.nfix};
		nimm = pd0.b - pi;
		case (nimm)
			3'd1:    nrec.imm = (a0.immk == 3'd2 || (a0.immk == 3'd1 && pd0.sz == SZ_B)) ?
			                    {24'd0, win[pi][7:0]} : sx16(win[pi]);
			3'd2:    nrec.imm = {win[pi], win[3'(pi + 1)]};
			default: nrec.imm = 32'd0;
		endcase
		nrec.ea0 = '0; nrec.ea0.m = EM_NONE;
		nrec.ea1 = '0; nrec.ea1.m = EM_NONE;

		if (!pd0.legal) begin
			// illegal / unimplemented operation word
			use_n    = 4'd1;
			go       = 1'b1;
			flt      = win_flt[0];
			nrec.exc = (opw[15:12] == 4'hA) ? 8'd10 :
			           (opw[15:12] == 4'hF) ? 8'd11 : 8'd4;
		end
		else if (fast) begin
			if (pd0.tot <= qcnt) begin
				go    = 1'b1;
				use_n = pd0.tot;
				flt   = |(win_flt & ((9'd1 << pd0.tot) - 9'd1));
				nrec.ea0 = mk_ea(pd0.i0, opw[2:0], {1'b0, fb},
				                 (pd0.sz == SZ_L) ? 3'd2 : 3'd1,
				                 qpc + {28'd0, fb, 1'b0},
				                 win[fb], win[3'(fb + 1)], 16'd0, 16'd0, 16'd0);
				nrec.ea1 = mk_ea(pd0.i1, opw[11:9], {1'b0, fp1}, 3'd0,
				                 qpc + {28'd0, fp1, 1'b0},
				                 win[fp1], win[3'(fp1 + 1)], 16'd0, 16'd0, 16'd0);
			end
		end
		else if ({1'b0, pd0.b} <= qcnt) begin
			// slow path, first phase: operation word, fixed words, immediate;
			// the EAs without extension words are complete already
			nrec.ea0 = mk_ea(pd0.i0, opw[2:0], 4'd0, 3'd0, 32'd0,
			                 16'd0, 16'd0, 16'd0, 16'd0, 16'd0);
			nrec.ea1 = mk_ea(pd0.i1, opw[11:9], 4'd0, 3'd0, 32'd0,
			                 16'd0, 16'd0, 16'd0, 16'd0, 16'd0);
			use_n = {1'b0, pd0.b};
			flt   = |(win_flt & ((8'd1 << pd0.b) - 8'd1));
			if (flt) begin
				go = 1'b1;                   // stop at the faulted part
			end
			else begin
				go_part = 1'b1;
				nph     = has_ext(pd0.i0) ? PH_EA0 : PH_EA1;
			end
		end

		nrec.npc = qpc + {27'd0, use_n, 1'b0};
		nrec.rt  = (jcond(a0.jc0, pd0.i0, pd0.i1, win[1], pd0.sz, opw[2:0])) ? a0.jt0 :
		           (a0.jc0 != JC_NEVER) ? a0.rt + 10'd1 : a0.rt;
		// fixed operand forms: (Ay)+,(Ax)+ and -(Ay),-(Ax); MOVE16
		if (a0.fea == 2'd3) begin
			if (opw[5]) begin
				nrec.ea0.m = EM_AIP; nrec.ea0.r = opw[2:0];
				nrec.ea1.m = EM_AIP; nrec.ea1.r = win[1][14:12];
			end
			else begin
				ea_t ra, aa;
				ra = '0; aa = '0;
				ra.m = opw[4] ? EM_AI : EM_AIP; ra.r = opw[2:0];
				aa.m = EM_ABSL; aa.bd = {win[1], win[2]};
				nrec.ea0 = opw[3] ? aa : ra;
				nrec.ea1 = opw[3] ? ra : aa;
			end
		end
		else if (a0.fea != 2'd0) begin
			nrec.ea0.m = (a0.fea == 2'd1) ? EM_AIP : EM_APD;
			nrec.ea1.m = (a0.fea == 2'd1) ? EM_AIP : EM_APD;
			nrec.ea0.r = opw[2:0];
			nrec.ea1.r = opw[11:9];
		end

		if (pd0.legal) begin
			if (a0.priv && !smode)
				nrec.exc = 8'd8;
			else if (a0.rt == UA_TRAP)
				nrec.exc = 8'd32 + {4'd0, opw[3:0]};
			else if ((a0.rt == UA_MOVEC_RD || a0.rt == UA_MOVEC_WR) &&
			         !((win[1][11:3] == 9'h000) ||
			           (win[1][11:3] == 9'h100 && win[1][2:0] != 3'd2)))
				// 68040 control registers: $000-$007 and $800-$807 except CAAR
				nrec.exc = 8'd4;
			else if (a0.rt == UA_ILLEGAL || a0.rt == UA_BKPT)
				nrec.exc = 8'd4;
		end
		if (flt) nrec.exc = f_code;
		if (nrec.exc != 8'd0) begin
			nrec.rt = UA_DEC_EXC;
			if (go_part) begin
				// an exception needs no further parsing
				go_part = 1'b0;
				go      = 1'b1;
				nph     = PH_IDLE;
			end
		end

		// PC-relative branches: target and prediction
		if (a0.rt == UA_BCC || a0.rt == UA_BSR) begin
			disp = (opw[7:0] == 8'h00) ? sx16(win[1]) :
			       (opw[7:0] == 8'hFF) ? {win[1], win[2]} :
			       {{24{opw[7]}}, opw[7:0]};
			ntarget = qpc + 32'd2 + disp;
			teq     = (disp == bt_d2);
			nrec.bst = disp[31];
			nrec.bhc = bht_q;
			nredir  = (a0.rt == UA_BSR) || (opw[11:8] == 4'h0) || (disp[31] ^ bht_q[1]);
		end
		else if ((a0.rt == UA_JSR || a0.rt == UA_JMP) && go && !go_part &&
		         (nrec.ea0.m == EM_ABSW || nrec.ea0.m == EM_ABSL || nrec.ea0.m == EM_PC16)) begin
			ntarget = nrec.ea0.bd;
			teq     = (nrec.ea0.m == EM_ABSL) ? ({win[1], win[2]} == bt_tgt) :
			          (nrec.ea0.m == EM_ABSW) ? (sx16(win[1]) == bt_tgt) :
			                                    (sx16(win[1]) == bt_d2);
			nredir  = 1'b1;
			// the target is known: JSR is a single push (as BSR); the
			// routine is changed as the record enters the FIFO, so the
			// return stack logic sees UA_JSR
			jsr_k = (a0.rt == UA_JSR);
		end
		else if (a0.rt == UA_RTS && ras_n != 4'd0) begin
			ntarget = ras[ras_tp];
			teq     = (ras[ras_tp] == bt_tgt);
			nredir  = 1'b1;
		end
		else if (a0.rt == UA_DBCC) begin
			disp    = sx16(win[1]);
			ntarget = qpc + 32'd2 + disp;
			teq     = (disp == bt_d2);
			nredir  = disp[31];
		end
		else if (a0.rt == UA_FBCC) begin
			disp    = opw[6] ? {win[1], win[2]} : sx16(win[1]);
			ntarget = qpc + 32'd2 + disp;
			teq     = (disp == bt_d2);
			nredir  = (opw[5:0] == 6'h0F) || disp[31];
		end
		else if (a0.rt == UA_FDBCC) begin
			disp    = sx16(win[2]);
			ntarget = qpc + 32'd4 + disp;
			teq     = (disp == bt_d2 - 32'd2);
			nredir  = disp[31];
		end
		if (a0.rt == UA_FPU_GEN && nrec.ea0.m == EM_IMM)
			nrec.ea0.bd = qpc + 32'd4;
		nrec.target = flt ? f_addr : ntarget;
		nrec.pred   = nredir && (nrec.exc == 8'd0) && pd0.legal;
	end
end

// the FIFO has room when it holds at most one record (registered)
// Words the fetch BTB flagged (the end of a branch it took).  A flag on
// the last word of a complete decode is checked against D1's own
// prediction; a flag anywhere else (a stale or aliased entry: the words
// after it came from its target) drops the entry and marks the record
// (or, for a multi-part decode, the instruction's final record) to be
// refetched at WB (EXC_SNR): this check stays off the consume path.
logic [7:0] bt_lm, bt_lb;
logic       bt_any, bt_end, bt_restart, part_bt;
logic [2:0] bt_j;
always_comb begin
	// the consumed words, and the last of them (tables, no arithmetic)
	case (use_n)
		4'd0: begin bt_lm = 8'h00; bt_lb = 8'h00; end
		4'd1: begin bt_lm = 8'h01; bt_lb = 8'h01; end
		4'd2: begin bt_lm = 8'h03; bt_lb = 8'h02; end
		4'd3: begin bt_lm = 8'h07; bt_lb = 8'h04; end
		4'd4: begin bt_lm = 8'h0F; bt_lb = 8'h08; end
		4'd5: begin bt_lm = 8'h1F; bt_lb = 8'h10; end
		4'd6: begin bt_lm = 8'h3F; bt_lb = 8'h20; end
		4'd7: begin bt_lm = 8'h7F; bt_lb = 8'h40; end
		default: begin bt_lm = 8'hFF; bt_lb = 8'h80; end
	endcase
	bt_any  = |(win_bt & bt_lm);
	bt_end  = go && !go_part && (ph == PH_IDLE) && ((win_bt & bt_lm) == bt_lb);
	bt_restart = (go || go_part) && bt_any && !bt_end;
	bt_j = 3'd0;
	for (int i = 7; i >= 0; i--) if (win_bt[i] && bt_lm[i]) bt_j = 3'(i);
end
// the record as it enters the FIFO: refetched at WB if a flag was inside
// the instruction (now, or in an earlier part)
dinst_t nrec_q;
always_comb begin
	nrec_q = nrec;
	if (jsr_k && nrec.exc == 8'd0) nrec_q.rt = UA_JSR_K;
	nrec_q.ras.v  = 1'b1;
	nrec_q.ras.tp = ras_tp;
	nrec_q.ras.n  = ras_n;
	nrec_q.ras.k  = (nrec.exc != 8'd0) ? 2'd0 :
	                rcall ? 2'd1 :
	                (rret && ras_n != 4'd0) ? 2'd2 : 2'd0;
	if (go && (bt_restart || part_bt)) begin
		nrec_q.exc  = EXC_SNR;
		nrec_q.rt   = UA_DEC_EXC;
		nrec_q.pred = 1'b0;
		nrec_q.ras.k = 2'd0;
	end
end
wire [31:0] lastpc = qpc + {27'd0, use_n - 4'd1, 1'b0};    // the decode's last word
wire        btb_ok = (ntarget[0] == 1'b0) && (nrec.exc == 8'd0);
// the entry's kind: a call pushes the fetch's return stack, a return pops it
wire  [1:0] btb_kind = rcall ? 2'd1 : rret ? 2'd2 : 2'd0;
always_comb begin
	for (int i = 0; i < 8; i++) ras_o[i] = ras[i][31:1];
	// a back-end redirect resynchronizes the fetch's stack from the
	// restored pointer
	ras_tp_o = ras_rv ? ras_rtp : ras_tp;
	ras_n_o  = ras_rv ? ras_rn  : ras_n;
end

wire room  = (rq_n != 2'd3);
wire stall = flush || hold_redir || d_redir_v || !room;
wire fire  = (go || go_part) && !stall;
wire push  = fire && go;
wire pop   = rq_pop && (rq_n != 2'd0);

assign consume = fire ? use_n[2:0] : 3'd0;

always_ff @(posedge clk) begin
	if (!nreset) begin
		rq_n       <= 2'd0;
		rq0        <= '0;
		rq1        <= '0;
		rq2        <= '0;
		ph         <= PH_IDLE;
		part       <= '0;
		part_pd    <= '0;
		part_len   <= '0;
		part_imml  <= '0;
		d_redir_v  <= 1'b0;
		d_redir_pc <= '0;
		hold_redir <= 1'b0;
		btb_we     <= 1'b0;
		btb_wi     <= '0;
		btb_wv     <= 1'b0;
		btb_wtag   <= '0;
		btb_wslot  <= 1'b0;
		btb_wkind  <= '0;
		btb_wtgt   <= '0;
		part_bt    <= 1'b0;
		part_call  <= 1'b0;
		part_ret   <= 1'b0;
		ras_tp     <= '0;
		ras_n      <= '0;
		for (int i = 0; i < 8; i++) ras[i] <= '0;
	end
	else begin
		d_redir_v  <= 1'b0;
		hold_redir <= d_redir_v;
		btb_we     <= 1'b0;

		if (bt_restart && fire) begin
			// a flagged word inside an instruction: drop the entry (the
			// record is refetched at WB)
			logic [31:0] fa;
			fa = qpc + {28'd0, bt_j, 1'b0};
			btb_we     <= 1'b1;
			btb_wv     <= 1'b0;
			btb_wi     <= fa[BTB_AW+1:2];
			btb_wtag   <= fa[31:BTB_AW+2];
		end
		// the flag of an earlier part stays with the instruction
		if (fire) part_bt <= go_part && (part_bt || bt_restart);
		if (flush) part_bt <= 1'b0;

		// the return stack follows the records in program order
		if (push && nrec_q.exc == 8'd0) begin
			if (rcall) begin
				ras[ras_tp + 3'd1] <= nrec.npc;
				ras_tp <= ras_tp + 3'd1;
				if (ras_n != 4'd8) ras_n <= ras_n + 4'd1;
			end
			else if (rret && ras_n != 4'd0) begin
				ras_tp <= ras_tp - 3'd1;
				ras_n  <= ras_n - 4'd1;
			end
		end

		// three records, so that D1 (which sees only the registered count)
		// and D2 can both move one a cycle: D2 chains into rq1 while D1
		// fills behind it
		case ({push, pop})
			2'b01: begin rq0 <= rq1; rq1 <= rq2; rq_n <= rq_n - 2'd1; end
			2'b10: begin
				case (rq_n)
					2'd0:    rq0 <= nrec_q;
					2'd1:    rq1 <= nrec_q;
					default: rq2 <= nrec_q;
				endcase
				rq_n <= rq_n + 2'd1;
			end
			2'b11: begin
				case (rq_n)
					2'd1:    rq0 <= nrec_q;
					2'd2:    begin rq0 <= rq1; rq1 <= nrec_q; end
					default: begin rq0 <= rq1; rq1 <= rq2; rq2 <= nrec_q; end
				endcase
			end
			default: ;
		endcase

		if (fire) begin
			ph <= nph;
			if (go_part) begin
				part <= nrec;
				if (ph == PH_IDLE) begin
					part_call <= rcall;
					part_ret  <= rret;
					part_pd   <= pd0;
					part_len  <= use_n;
					part_imml <= (a0.rt == UA_FPU_GEN) ? fp_immlen(win[1]) :
					             (pd0.sz == SZ_L) ? 3'd2 : 3'd1;
				end
				else
					part_len <= part_len + use_n;
			end
			if (push && ph == PH_IDLE && bt_end) begin
				// the fetch already took this branch: fine if D1 agrees
				if (!(nrec.pred && teq)) begin
					d_redir_v  <= 1'b1;
					d_redir_pc <= nrec.pred ? ntarget : nrec.npc;
					btb_we     <= 1'b1;
					btb_wv     <= nrec.pred && btb_ok;
					btb_wi     <= lastpc[BTB_AW+1:2];
					btb_wtag   <= lastpc[31:BTB_AW+2];
					btb_wslot  <= lastpc[1];
					btb_wkind  <= btb_kind;
					btb_wtgt   <= ntarget[31:1];
				end
			end
			else if (push && nrec_q.pred && ph == PH_IDLE) begin
				d_redir_v  <= 1'b1;
				d_redir_pc <= ntarget;
				// next time the fetch takes it
				btb_we     <= btb_ok;
				btb_wv     <= 1'b1;
				btb_wi     <= lastpc[BTB_AW+1:2];
				btb_wtag   <= lastpc[31:BTB_AW+2];
				btb_wslot  <= lastpc[1];
				btb_wkind  <= btb_kind;
				btb_wtgt   <= ntarget[31:1];
			end
		end
		if (flush) begin
			rq_n       <= 2'd0;
			if (ras_rv) begin
				ras_tp <= ras_rtp;
				ras_n  <= ras_rn;
			end
			ph         <= PH_IDLE;
			d_redir_v  <= 1'b0;
			hold_redir <= 1'b0;
		end
	end
end

endmodule
