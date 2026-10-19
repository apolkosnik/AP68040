//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                            //
//                                                                          //
// ap68040_predec.sv - predecode of one instruction queue word              //
//                                                                          //
// Treats the word as an operation word and works out everything D1 needs  //
// to size the instruction without decoding it: legality, decoder entry,   //
// size, and the instruction length assuming brief index extension words. //
// D1 checks bit 8 of the words at the index extension positions (a full-  //
// format extension) and the slow flag (FPU immediates) and takes its      //
// multi-cycle path for those.                                             //
//--------------------------------------------------------------------------//

module ap68040_predec
	import ap68040_pkg::*, ap68040_upkg::*;
(
	input  logic [15:0] op,
	input  pdw_t        w,          // the table's word for op[15:3]
	output pd_t         pd
);

`include "gen/ap68040_pd_alt.svh"

function automatic logic [3:0] ea_idx(input logic [2:0] m, input logic [2:0] r);
	if (m != 3'd7) ea_idx = {1'b0, m};
	else if (r <= 3'd4) ea_idx = 4'd7 + {1'b0, r};
	else ea_idx = EM_NONE;
endfunction

// extension words of an EA with a brief index extension
function automatic logic [2:0] ea_brief_len(input logic [3:0] i, input logic [1:0] sz);
	case (i)
		EM_AD16, EM_ABSW, EM_PC16, EM_AX, EM_PCX: ea_brief_len = 3'd1;
		EM_ABSL: ea_brief_len = 3'd2;
		EM_IMM:  ea_brief_len = (sz == SZ_L) ? 3'd2 : 3'd1;
		default: ea_brief_len = 3'd0;
	endcase
endfunction

always_comb begin
	pdw_t x;
	logic [24:0] aw;
	logic        use_alt;
	logic [4:0]  ak;
	logic [3:0]  i0, i1;
	logic [2:0]  nimm_l, nimm_n, l0_l, l0_n, l1_l, l1_n, b_l, b_n;
	logic [1:0]  sz;
	logic        isl, legal0;
	// the word, or the alternate the register field selects
	x = w;
	use_alt = 1'b0;
	ak = w.alt;
	case (w.kind)
		2'd1: begin use_alt = (op[2:0] == 3'd4); end
		2'd2: begin use_alt = (op[2:0] == 3'd2) || (op[2:0] == 3'd3) || (op[2:0] == 3'd4);
		            ak = w.alt + {2'd0, op[2:0]} - 5'd2; end
		2'd3: begin use_alt = 1'b1; ak = w.alt + {2'd0, op[2:0]}; end
		default: ;
	endcase
	aw = pd_alt(ak);
	if (use_alt) {x.ent, x.szc, x.nfix, x.immk, x.ea0v, x.ea1v, x.fpg, x.leg1, x.leg0} = aw;
	i0 = x.ea0v ? ea_idx(op[5:3], op[2:0]) : EM_NONE;
	i1 = x.ea1v ? ea_idx(op[8:6], op[11:9]) : EM_NONE;
	sz = (x.szc == 2'd3) ? op[7:6] : x.szc;
	// the lengths are formed for a long and for a shorter size side by
	// side, and the size picks one last (a timing path: the size comes
	// out of the table)
	isl = (x.szc == 2'd3) ? (op[7:6] == SZ_L) : (x.szc == SZ_L);
	case (x.immk)
		3'd1:       begin nimm_l = 3'd2; nimm_n = 3'd1; end
		3'd2, 3'd3: begin nimm_l = 3'd1; nimm_n = 3'd1; end
		3'd4:       begin nimm_l = 3'd2; nimm_n = 3'd2; end
		3'd5:       begin nimm_l = (op[7:0] == 8'h00) ? 3'd1 : (op[7:0] == 8'hFF) ? 3'd2 : 3'd0; nimm_n = nimm_l; end
		3'd6:       begin nimm_l = (op[2:0] == 3'd2) ? 3'd1 : (op[2:0] == 3'd3) ? 3'd2 : 3'd0; nimm_n = nimm_l; end
		3'd7:       begin nimm_l = op[6] ? 3'd2 : 3'd1; nimm_n = nimm_l; end
		default:    begin nimm_l = 3'd0; nimm_n = 3'd0; end
	endcase
	l0_l = ea_brief_len(i0, SZ_L);
	l0_n = ea_brief_len(i0, SZ_W);
	l1_l = ea_brief_len(i1, SZ_L);
	l1_n = ea_brief_len(i1, SZ_W);
	b_l  = 3'd1 + {1'b0, x.nfix} + nimm_l;
	b_n  = 3'd1 + {1'b0, x.nfix} + nimm_n;
	// EA0's legality: the word's bit for the mode, or for mode 7 the bit
	// of the register (5-7 are no modes)
	legal0 = !x.ea0v || ((op[5:3] != 3'd7) ? x.leg0[0] : (!op[2] || op[1:0] == 2'b00) && x.leg0[op[2:0]]);
	pd.legal = (x.ent != PD_NOENT) && legal0 && (!x.ea1v || x.leg1);
	pd.ent  = (x.ent == PD_NOENT) ? 9'd0 : x.ent;   // (no entry: 0, as the pattern decoder)
	pd.sz   = sz;
	pd.b    = isl ? b_l : b_n;
	pd.p1   = isl ? (b_l + l0_l) : (b_n + l0_n);
	pd.tot  = isl ? ({1'b0, b_l} + ({1'b0, l0_l} + {1'b0, l1_l}))
	              : ({1'b0, b_n} + ({1'b0, l0_n} + {1'b0, l1_n}));
	pd.x0   = (i0 == EM_AX) || (i0 == EM_PCX);
	pd.x1   = (i1 == EM_AX) || (i1 == EM_PCX);
	pd.slow = x.fpg && (i0 == EM_IMM);
	pd.i0   = i0;
	pd.i1   = i1;
	// an illegal word: D1 takes one word and the exception; b, p1 and tot
	// are not used for it (they stay off the override, a timing path)
	if (!pd.legal) begin
		pd.x0  = 1'b0;
		pd.x1  = 1'b0;
		pd.slow = 1'b0;
	end
end

endmodule

// the same predecode from the parallel pattern decoder (simulation: the
// reference the table form is checked against)
module ap68040_predec_pla
	import ap68040_pkg::*, ap68040_upkg::*;
(
	input  logic [15:0] op,
	output pd_t         pd
);

`include "gen/ap68040_dec_pla.svh"

function automatic logic [3:0] ea_idx(input logic [2:0] m, input logic [2:0] r);
	if (m != 3'd7) ea_idx = {1'b0, m};
	else if (r <= 3'd4) ea_idx = 4'd7 + {1'b0, r};
	else ea_idx = EM_NONE;
endfunction

// extension words of an EA with a brief index extension
function automatic logic [2:0] ea_brief_len(input logic [3:0] i, input logic [1:0] sz);
	case (i)
		EM_AD16, EM_ABSW, EM_PC16, EM_AX, EM_PCX: ea_brief_len = 3'd1;
		EM_ABSL: ea_brief_len = 3'd2;
		EM_IMM:  ea_brief_len = (sz == SZ_L) ? 3'd2 : 3'd1;
		default: ea_brief_len = 3'd0;
	endcase
endfunction

always_comb begin
	logic [NENT-1:0] s;
	pla_t a;
	logic [3:0] i0, i1;
	logic [2:0] nimm_l, nimm_n, l0_l, l0_n, l1_l, l1_n, b_l, b_n;
	logic [1:0] sz;
	logic       isl;
	s  = dec_sel(op);
	a  = ent_attr_oh(s);
	i0 = a.ea0v ? ea_idx(op[5:3], op[2:0]) : EM_NONE;
	i1 = a.ea1v ? ea_idx(op[8:6], op[11:9]) : EM_NONE;
	sz = (a.szc == 2'd3) ? op[7:6] : a.szc;
	// the lengths are formed for a long and for a shorter size side by
	// side, and the size picks one last (a timing path: the size comes
	// out of the decode table)
	isl = (a.szc == 2'd3) ? (op[7:6] == SZ_L) : (a.szc == SZ_L);
	case (a.immk)
		3'd1:       begin nimm_l = 3'd2; nimm_n = 3'd1; end
		3'd2, 3'd3: begin nimm_l = 3'd1; nimm_n = 3'd1; end
		3'd4:       begin nimm_l = 3'd2; nimm_n = 3'd2; end
		3'd5:       begin nimm_l = (op[7:0] == 8'h00) ? 3'd1 : (op[7:0] == 8'hFF) ? 3'd2 : 3'd0; nimm_n = nimm_l; end
		3'd6:       begin nimm_l = (op[2:0] == 3'd2) ? 3'd1 : (op[2:0] == 3'd3) ? 3'd2 : 3'd0; nimm_n = nimm_l; end
		3'd7:       begin nimm_l = op[6] ? 3'd2 : 3'd1; nimm_n = nimm_l; end
		default:    begin nimm_l = 3'd0; nimm_n = 3'd0; end
	endcase
	l0_l = ea_brief_len(i0, SZ_L);
	l0_n = ea_brief_len(i0, SZ_W);
	l1_l = ea_brief_len(i1, SZ_L);
	l1_n = ea_brief_len(i1, SZ_W);
	b_l  = 3'd1 + {1'b0, a.nfix} + nimm_l;
	b_n  = 3'd1 + {1'b0, a.nfix} + nimm_n;
	pd.legal = a.match &&
	           (!a.ea0v || (i0 != EM_NONE && a.ea0m[i0])) &&
	           (!a.ea1v || (i1 != EM_NONE && a.ea1m[i1]));
	pd.ent  = a.ent;
	pd.sz   = sz;
	pd.b    = isl ? b_l : b_n;
	pd.p1   = isl ? (b_l + l0_l) : (b_n + l0_n);
	pd.tot  = isl ? ({1'b0, b_l} + ({1'b0, l0_l} + {1'b0, l1_l}))
	              : ({1'b0, b_n} + ({1'b0, l0_n} + {1'b0, l1_n}));
	pd.x0   = (i0 == EM_AX) || (i0 == EM_PCX);
	pd.x1   = (i1 == EM_AX) || (i1 == EM_PCX);
	pd.slow = (a.rt == UA_FPU_GEN) && (i0 == EM_IMM);
	pd.i0   = i0;
	pd.i1   = i1;
	// an illegal word: D1 takes one word and the exception; b, p1 and tot
	// are not used for it (they stay off the override, a timing path)
	if (!pd.legal) begin
		pd.x0  = 1'b0;
		pd.x1  = 1'b0;
		pd.slow = 1'b0;
	end
end

endmodule
