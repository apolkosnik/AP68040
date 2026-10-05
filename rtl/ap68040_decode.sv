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
// and the next PC are added here.  Unconditional and statically          //
// predicted (backward) branches redirect fetch.  Illegal, line A/F,      //
// privilege, TRAP #n, odd PC and fetch faults become records carrying    //
// their exception vector.                                                 //
//--------------------------------------------------------------------------//

module ap68040_decode
	import ap68040_pkg::*, ap68040_upkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,          // back end redirect: drop everything

	input  logic [15:0] win [8],
	input  logic  [7:0] win_flt,
	input  logic  [7:0] win_fdem,     // the faulted fetch was a demand fetch
	input  logic  [7:0] win_fatc,     // ... an ATC (MMU) fault
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
	input  logic        rq_pop
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
ph_t         nph;

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
			// microcode: its address follows the extension word
			if (ph == PH_EA0 && nrec.ea0.m == EM_IMM && part.rt == UA_FPU_GEN)
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

		// PC-relative branches: target and static prediction
		if (a0.rt == UA_BCC || a0.rt == UA_BSR) begin
			disp = (opw[7:0] == 8'h00) ? sx16(win[1]) :
			       (opw[7:0] == 8'hFF) ? {win[1], win[2]} :
			       {{24{opw[7]}}, opw[7:0]};
			ntarget = qpc + 32'd2 + disp;
			nredir  = (a0.rt == UA_BSR) || (opw[11:8] == 4'h0) || disp[31];
		end
		else if (a0.rt == UA_DBCC) begin
			disp    = sx16(win[1]);
			ntarget = qpc + 32'd2 + disp;
			nredir  = disp[31];
		end
		else if (a0.rt == UA_FBCC) begin
			disp    = opw[6] ? {win[1], win[2]} : sx16(win[1]);
			ntarget = qpc + 32'd2 + disp;
			nredir  = (opw[5:0] == 6'h0F) || disp[31];
		end
		else if (a0.rt == UA_FDBCC) begin
			disp    = sx16(win[2]);
			ntarget = qpc + 32'd4 + disp;
			nredir  = disp[31];
		end
		if (a0.rt == UA_FPU_GEN && nrec.ea0.m == EM_IMM)
			nrec.ea0.bd = qpc + 32'd4;
		nrec.target = flt ? f_addr : ntarget;
		nrec.pred   = nredir && (nrec.exc == 8'd0) && pd0.legal;
	end
end

// the FIFO has room when it holds at most one record (registered)
wire room  = (rq_n != 2'd2);
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
		ph         <= PH_IDLE;
		part       <= '0;
		part_pd    <= '0;
		part_len   <= '0;
		part_imml  <= '0;
		d_redir_v  <= 1'b0;
		d_redir_pc <= '0;
		hold_redir <= 1'b0;
	end
	else begin
		d_redir_v  <= 1'b0;
		hold_redir <= d_redir_v;

		case ({push, pop})
			2'b01: begin rq0 <= rq1; rq_n <= rq_n - 2'd1; end
			2'b10: begin
				if (rq_n == 2'd0) rq0 <= nrec; else rq1 <= nrec;
				rq_n <= rq_n + 2'd1;
			end
			2'b11: begin
				if (rq_n == 2'd1) rq0 <= nrec;
				else begin rq0 <= rq1; rq1 <= nrec; end
			end
			default: ;
		endcase

		if (fire) begin
			ph <= nph;
			if (go_part) begin
				part <= nrec;
				if (ph == PH_IDLE) begin
					part_pd   <= pd0;
					part_len  <= use_n;
					part_imml <= (a0.rt == UA_FPU_GEN) ? fp_immlen(win[1]) :
					             (pd0.sz == SZ_L) ? 3'd2 : 3'd1;
				end
				else
					part_len <= part_len + use_n;
			end
			if (push && nrec.pred && ph == PH_IDLE) begin
				d_redir_v  <= 1'b1;
				d_redir_pc <= ntarget;
			end
		end
		if (flush) begin
			rq_n       <= 2'd0;
			ph         <= PH_IDLE;
			d_redir_v  <= 1'b0;
			hold_redir <= 1'b0;
		end
	end
end

endmodule
