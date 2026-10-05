//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_decode.sv - D1: instruction parse                                 //
//                                                                          //
// One instruction per cycle from the eight-word window at the queue head:  //
//   operation word | fixed extension words | immediate | EA0 ext | EA1 ext //
// The generated PLA (rtl/gen/ap68040_dec_pla.svh, checked against WinUAE  //
// for all 65536 operation words) names the routine, size, the legal EA     //
// modes and the instruction's format.  A MOVE whose two EAs together need  //
// more than eight words is parsed in two cycles (EA1 in the second).      //
//                                                                          //
// Unconditional and statically predicted (backward) PC-relative branches  //
// redirect fetch from here.  Illegal, line A/F, privilege, TRAP #n, odd    //
// PC and fetch bus errors become a record carrying the exception vector.  //
//--------------------------------------------------------------------------//

module ap68040_decode
	import ap68040_pkg::*;
	import ap68040_upkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        flush,          // back end redirect: drop everything

	input  logic [15:0] win [8],
	input  logic  [7:0] win_flt,
	input  logic  [4:0] qcnt,
	input  logic [31:0] qpc,
	input  logic        q_odd,
	input  logic        smode,

	output logic  [3:0] consume,
	output logic        d_redir_v,
	output logic [31:0] d_redir_pc,

	output logic        rec_v,
	output dinst_t      rec,
	input  logic        rec_take
);


`include "gen/ap68040_dec_pla.svh"

//--------------------------------------------------------------------------
// helpers
//--------------------------------------------------------------------------
function automatic logic [3:0] ea_idx(input logic [2:0] m, input logic [2:0] r);
	if (m != 3'd7) ea_idx = {1'b0, m};
	else if (r <= 3'd4) ea_idx = 4'd7 + {1'b0, r};
	else ea_idx = EM_NONE;
endfunction

// words of extension for an EA: w is its first extension word
function automatic logic [2:0] ea_len(input logic [3:0] idx, input logic [15:0] w,
                                      input logic [2:0] immlen);
	logic [2:0] n;
	case (idx)
		EM_AD16, EM_ABSW, EM_PC16: n = 3'd1;
		EM_ABSL: n = 3'd2;
		EM_IMM:  n = immlen;
		EM_AX, EM_PCX: begin
			if (!w[8]) n = 3'd1;
			else begin
				n = 3'd1;
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

// build the EA record from the window, the EA's first extension word at p
function automatic ea_t mk_ea(input logic [3:0] idx, input logic [2:0] r,
                              input logic [2:0] p, input logic [2:0] immlen,
                              input logic [15:0] w0, input logic [15:0] w1,
                              input logic [15:0] w2, input logic [15:0] w3,
                              input logic [15:0] w4);
	ea_t e;
	e = '0;
	e.m    = idx;
	e.r    = r;
	e.xoff = p;
	case (idx)
		EM_AD16, EM_ABSW, EM_PC16: e.bd = sx16(w0);
		EM_ABSL: e.bd = {w0, w1};
		EM_IMM:  e.bd = (immlen == 3'd1) ? {16'd0, w0} : {w0, w1};
		EM_AX, EM_PCX: begin
			e.xa = w0[15];
			e.xr = w0[14:12];
			e.xl = w0[11];
			e.sc = w0[10:9];
			if (!w0[8]) begin
				e.bd = {{24{w0[7]}}, w0[7:0]};
			end
			else begin
				logic [15:0] o0, o1;
				e.bs = w0[7];
				e.is = w0[6];
				case (w0[5:4])
					2'd2: begin e.bd = sx16(w1);  o0 = w2; o1 = w3; end
					2'd3: begin e.bd = {w1, w2};  o0 = w3; o1 = w4; end
					default: begin e.bd = 32'd0; o0 = w1; o1 = w2; end
				endcase
				if (w0[1:0] != 2'd0)
					e.mi = (w0[2] && !w0[6]) ? 2'd2 : 2'd1;
				case (w0[1:0])
					2'd2: e.od = sx16(o0);
					2'd3: e.od = {o0, o1};
					default: e.od = 32'd0;
				endcase
			end
		end
		default: ;
	endcase
	mk_ea = e;
endfunction

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
logic        ph;            // 1: second cycle of a long MOVE (EA1)
dinst_t      part;          // record under construction in phase 1
logic  [3:0] part_len;      // words consumed by phase 0
logic        hold_redir;    // a predicted branch is redirecting fetch

//--------------------------------------------------------------------------
// phase-0 parse of the instruction at win[0]
//--------------------------------------------------------------------------
pla_t        pla;
logic [15:0] opw;
logic  [1:0] sz;
logic  [3:0] i0, i1;
logic  [2:0] nimm, p0, p1, l0, l1, el_imm, ea1_immlen;
logic  [3:0] total;
logic        legal;
logic [15:0] w_p0 [5];
logic [15:0] w_p1 [5];

assign opw = win[0];
always_comb pla = dec_pla(opw);

always_comb begin
	sz = (pla.szc == 2'd3) ? opw[7:6] : pla.szc;
	i0 = ea_idx(opw[5:3], opw[2:0]);
	i1 = ea_idx(opw[8:6], opw[11:9]);
	case (pla.immk)
		3'd1:    nimm = (sz == SZ_L) ? 3'd2 : 3'd1;
		3'd2, 3'd3: nimm = 3'd1;
		3'd4:    nimm = 3'd2;
		3'd5:    nimm = (opw[7:0] == 8'h00) ? 3'd1 : (opw[7:0] == 8'hFF) ? 3'd2 : 3'd0;
		3'd6:    nimm = (opw[2:0] == 3'd2) ? 3'd1 : (opw[2:0] == 3'd3) ? 3'd2 : 3'd0;
		3'd7:    nimm = opw[6] ? 3'd2 : 3'd1;
		default: nimm = 3'd0;
	endcase
	// immediate EA operand length: the operation size, or the FPU format
	if (pla.rt == UA_FPU_GEN) begin
		case (win[1][12:10])
			3'd0, 3'd1: el_imm = 3'd2;   // L, S
			3'd2, 3'd3: el_imm = 3'd6;   // X, P
			3'd4, 3'd6: el_imm = 3'd1;   // W, B
			3'd5:       el_imm = 3'd4;   // D
			default:    el_imm = 3'd0;
		endcase
	end
	else
		el_imm = (sz == SZ_L) ? 3'd2 : 3'd1;
	ea1_immlen = 3'd0;
	p0 = 3'd1 + {1'b0, pla.nfix} + nimm;
	for (int i = 0; i < 5; i++) w_p0[i] = win[3'(p0 + i)];
	l0 = pla.ea0v ? ea_len(i0, w_p0[0], el_imm) : 3'd0;
	p1 = p0 + l0;
	for (int i = 0; i < 5; i++) w_p1[i] = win[3'(p1 + i)];
	l1 = pla.ea1v ? ea_len(i1, w_p1[0], ea1_immlen) : 3'd0;
	total = {1'b0, p1} + {1'b0, l1};
	legal = pla.match &&
	        (!pla.ea0v || (i0 != EM_NONE && pla.ea0m[i0])) &&
	        (!pla.ea1v || (i1 != EM_NONE && pla.ea1m[i1]));
end

// a MOVE whose EAs overflow the window: p1 + l1 > 8 (p1 itself is <= 6)
wire split = legal && pla.ea1v && ({1'b0, p1} + {1'b0, l1} > 4'd8);

// phase-1 parse: EA1 extension at win[0]
logic [3:0] i1s;
logic [2:0] l1s;
assign i1s = ea_idx(part.opw[8:6], part.opw[11:9]);
assign l1s = ea_len(i1s, win[0], 3'd0);

//--------------------------------------------------------------------------
// record assembly
//--------------------------------------------------------------------------
dinst_t      nrec;
logic        go;            // a record (or phase-0 part) is produced
logic  [3:0] use_n;         // words consumed
logic        flt;           // a consumed word came from a faulted fetch
logic        nredir;
logic [31:0] ntarget;

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
		JC_AY7:      jcond = (ry == 3'd7);
		JC_BOTH_MEM: jcond = m0 && m1;
		JC_MASK0:    jcond = (ext1 == 16'd0);
		JC_CREG_RF:  jcond = (ext1[11:0] == 12'h800) || (ext1[11:0] == 12'h803) ||
		                     (ext1[11:0] == 12'h804);
		default:     jcond = 1'b0;
	endcase
endfunction

always_comb begin
	logic [31:0] disp;
	logic  [2:0] avail_ok;
	nrec    = '0;
	go      = 1'b0;
	use_n   = 4'd0;
	flt     = 1'b0;
	nredir  = 1'b0;
	ntarget = '0;
	disp    = '0;

	if (ph) begin
		// second cycle of a long MOVE: EA1 extension words at the head
		nrec = part;
		if ({2'b00, l1s} <= qcnt) begin
			go    = 1'b1;
			use_n = {1'b0, l1s};
			flt   = |(win_flt & ((8'd1 << l1s) - 8'd1));
			nrec.ea1 = mk_ea(i1s, part.opw[11:9], part_len[2:0], 3'd0,
			                 win[0], win[1], win[2], win[3], win[4]);
			nrec.npc = part.pc + {27'd0, part_len + {1'b0, l1s}, 1'b0};
			if (flt) begin
				nrec.exc = 8'd2;
				nrec.rt  = UA_DEC_EXC;
			end
		end
	end
	else if (q_odd && qcnt == 5'd0) begin
		// the stream starts at an odd address: address error
		go       = 1'b1;
		nrec.pc  = qpc;
		nrec.npc = qpc;
		nrec.exc = 8'd3;
		nrec.rt  = UA_DEC_EXC;
	end
	else if (qcnt != 5'd0) begin
		nrec.pc   = qpc;
		nrec.opw  = opw;
		nrec.ext1 = win[1];
		nrec.ext2 = win[2];
		nrec.sz   = sz;
		nrec.eop  = pla.eop;
		nrec.econd = pla.econd;
		nrec.ccr  = pla.ccr;
		// immediate field: words 1 + nfix ..
		begin
			logic [2:0] pi;
			pi = 3'd1 + {1'b0, pla.nfix};
			case (nimm)
				3'd1:    nrec.imm = (pla.immk == 3'd2 || (pla.immk == 3'd1 && sz == SZ_B)) ?
				                    {24'd0, win[pi][7:0]} : sx16(win[pi]);
				3'd2:    nrec.imm = {win[pi], win[3'(pi + 1)]};
				default: nrec.imm = 32'd0;
			endcase
		end
		nrec.ea0 = mk_ea(pla.ea0v ? i0 : EM_NONE, opw[2:0], p0, el_imm,
		                 w_p0[0], w_p0[1], w_p0[2], w_p0[3], w_p0[4]);
		nrec.fimm = {w_p0[2], w_p0[3], w_p0[4], win[3'(p0 + 5)]};
		nrec.ea1 = mk_ea(pla.ea1v ? i1 : EM_NONE, opw[11:9], p1, 3'd0,
		                 w_p1[0], w_p1[1], w_p1[2], w_p1[3], w_p1[4]);
		// fixed operand forms: (Ay)+,(Ax)+ and -(Ay),-(Ax); MOVE16
		if (pla.fea == 2'd3) begin
			nrec.ea0 = '0;
			nrec.ea1 = '0;
			if (opw[5]) begin
				// MOVE16 (Ax)+,(Ay)+
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
		else if (pla.fea != 2'd0) begin
			nrec.ea0   = '0;
			nrec.ea1   = '0;
			nrec.ea0.m = (pla.fea == 2'd1) ? EM_AIP : EM_APD;
			nrec.ea1.m = (pla.fea == 2'd1) ? EM_AIP : EM_APD;
			nrec.ea0.r = opw[2:0];
			nrec.ea1.r = opw[11:9];
		end

		if (!legal) begin
			// illegal / unimplemented operation word
			use_n    = 4'd1;
			go       = 1'b1;
			flt      = win_flt[0];
			nrec.exc = (opw[15:12] == 4'hA) ? 8'd10 :
			           (opw[15:12] == 4'hF) ? 8'd11 : 8'd4;
			nrec.rt  = UA_DEC_EXC;
		end
		else if (split) begin
			// phase 0 of a long MOVE
			if ({2'b00, p1} <= qcnt) begin
				go    = 1'b1;
				use_n = {1'b0, p1};
				flt   = |(win_flt & ((8'd1 << p1) - 8'd1));
			end
		end
		else if ({1'b0, total} <= qcnt) begin
			go    = 1'b1;
			use_n = total;
			flt   = |(win_flt & ((9'd1 << total) - 9'd1));
		end

		nrec.npc = qpc + {27'd0, use_n, 1'b0};
		nrec.rt  = (jcond(pla.jc0, pla.ea0v ? i0 : EM_NONE, pla.ea1v ? i1 : EM_NONE,
		                  win[1], sz, opw[2:0])) ? pla.jt0 :
		           (pla.jc0 != JC_NEVER) ? pla.rt + 9'd1 : pla.rt;

		if (legal) begin
			if (pla.priv && !smode) begin
				nrec.exc = 8'd8;
				nrec.rt  = UA_DEC_EXC;
			end
			else if (pla.rt == UA_TRAP) begin
				nrec.exc = 8'd32 + {4'd0, opw[3:0]};
				nrec.rt  = UA_DEC_EXC;
			end
			else if ((pla.rt == UA_MOVEC_RD || pla.rt == UA_MOVEC_WR) &&
			         !((win[1][11:3] == 9'h000) ||
			           (win[1][11:3] == 9'h100 && win[1][2:0] != 3'd2))) begin
				// 68040 control registers: $000-$007 and $800-$807 except CAAR
				nrec.exc = 8'd4;
				nrec.rt  = UA_DEC_EXC;
			end
			else if (pla.rt == UA_ILLEGAL || pla.rt == UA_BKPT) begin
				nrec.exc = 8'd4;
				nrec.rt  = UA_DEC_EXC;
			end
		end
		if (flt) begin
			nrec.exc = 8'd2;
			nrec.rt  = UA_DEC_EXC;
		end

		// PC-relative branches: target and static prediction
		if (pla.rt == UA_BCC || pla.rt == UA_BSR) begin
			disp = (opw[7:0] == 8'h00) ? sx16(win[1]) :
			       (opw[7:0] == 8'hFF) ? {win[1], win[2]} :
			       {{24{opw[7]}}, opw[7:0]};
			ntarget = qpc + 32'd2 + disp;
			nredir  = (pla.rt == UA_BSR) || (opw[11:8] == 4'h0) || disp[31];
		end
		else if (pla.rt == UA_DBCC) begin
			disp    = sx16(win[1]);
			ntarget = qpc + 32'd2 + disp;
			nredir  = disp[31];
		end
		nrec.target = ntarget;
		nrec.pred   = nredir && (nrec.exc == 8'd0);
	end
end

// the output register can take a record when empty or being taken
wire out_free = !rec_v || rec_take;
wire stall    = flush || hold_redir || d_redir_v;
wire fire     = go && out_free && !stall;

assign consume = fire ? use_n : 4'd0;

always_ff @(posedge clk) begin
	if (!nreset) begin
		rec_v      <= 1'b0;
		rec        <= '0;
		ph         <= 1'b0;
		part       <= '0;
		part_len   <= '0;
		d_redir_v  <= 1'b0;
		d_redir_pc <= '0;
		hold_redir <= 1'b0;
	end
	else begin
		d_redir_v  <= 1'b0;
		hold_redir <= d_redir_v;
		if (rec_take) rec_v <= 1'b0;
		if (fire) begin
			if (!ph && split && nrec.exc == 8'd0) begin
				ph       <= 1'b1;
				part     <= nrec;
				part_len <= use_n;
			end
			else begin
				ph    <= 1'b0;
				rec_v <= 1'b1;
				rec   <= nrec;
				if (!ph && nrec.pred) begin
					d_redir_v  <= 1'b1;
					d_redir_pc <= ntarget;
				end
			end
		end
		if (flush) begin
			rec_v      <= 1'b0;
			ph         <= 1'b0;
			d_redir_v  <= 1'b0;
			hold_redir <= 1'b0;
		end
	end
end

endmodule
