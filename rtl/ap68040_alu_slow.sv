//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_alu_slow.sv - EX-stage two-cycle integer unit: shifts and        //
// rotates, BCD, PACK/UNPK.  Its outputs are registered by the back end     //
// before they are used (the op holds EX for a second cycle).               //
//                                                                          //
// Same conventions and 68040 flag rules as ap68040_alu.                    //
//--------------------------------------------------------------------------//

module ap68040_alu_slow
	import ap68040_pkg::*;
(
	input  logic  [6:0] op,
	input  logic  [1:0] sz,
	input  logic [31:0] a,
	input  logic [31:0] b,
	input  logic  [5:0] cnt,        // shift count: a[5:0], without a's mux
	input  logic  [4:0] flags_in,   // X N Z V C
	output logic [31:0] res,
	output logic  [4:0] flags_out
);

wire f_x = flags_in[4];
wire f_n = flags_in[3];
wire f_z = flags_in[2];
wire f_v = flags_in[1];
wire f_c = flags_in[0];

logic [31:0] szmask;
logic  [5:0] nbits;
always_comb begin
	case (sz)
		SZ_B:    begin szmask = 32'h0000_00FF; nbits = 6'd8;  end
		SZ_W:    begin szmask = 32'h0000_FFFF; nbits = 6'd16; end
		default: begin szmask = 32'hFFFF_FFFF; nbits = 6'd32; end
	endcase
end

wire [31:0] am = a & szmask;
wire [31:0] bm = b & szmask;

function automatic logic msb_of(input logic [31:0] v, input logic [1:0] s);
	case (s)
		SZ_B:    msb_of = v[7];
		SZ_W:    msb_of = v[15];
		default: msb_of = v[31];
	endcase
endfunction

function automatic logic [31:0] merge(input logic [31:0] r, input logic [31:0] bb,
                                      input logic [31:0] m);
	merge = (r & m) | (bb & ~m);
endfunction

wire b_msb = msb_of(b, sz);

//--------------------------------------------------------------------------
// BCD (byte).  Decimal corrections apply to the whole byte, so a +/-6
// low-nibble adjust ripples into the high nibble; the carry comes from the
// corrected value (68040 hardware, cputest 68040_default).
//--------------------------------------------------------------------------
wire [4:0] abcd_lo  = {1'b0, b[3:0]} + {1'b0, a[3:0]} + {4'd0, f_x};
wire [9:0] abcd_sum = {2'd0, b[7:0]} + {2'd0, a[7:0]} + {9'd0, f_x}
                    + ((abcd_lo > 5'd9) ? 10'd6 : 10'd0);
wire       abcd_c   = ((abcd_sum & 10'h3F0) > 10'h090);
wire [9:0] abcd_res = abcd_sum + (abcd_c ? 10'h060 : 10'd0);

// SBCD b - a - X and NBCD 0 - b - X share one datapath
wire [7:0] sb_l     = (op == OP_NBCD) ? 8'd0 : b[7:0];
wire [7:0] sb_r     = (op == OP_NBCD) ? b[7:0] : a[7:0];
wire       sbcd_lb  = ({1'b0, sb_l[3:0]} < ({1'b0, sb_r[3:0]} + {4'd0, f_x}));
wire [9:0] sbcd_raw = {2'd0, sb_l} - {2'd0, sb_r} - {9'd0, f_x};
wire [9:0] sbcd_cor = sbcd_raw - (sbcd_lb ? 10'd6 : 10'd0);
wire [9:0] sbcd_res = sbcd_cor - (sbcd_raw[9] ? 10'h060 : 10'd0);
wire       sbcd_c   = sbcd_cor[9];

// PACK: b holds the two unpacked bytes of the word, a the adjustment
wire [15:0] pk_sum  = b[15:0] + a[15:0];
wire  [7:0] pack_r  = {pk_sum[11:8], pk_sum[3:0]};
// UNPK: b holds the packed byte
wire [15:0] unpk_r  = {4'd0, b[7:4], 4'd0, b[3:0]} + a[15:0];

//--------------------------------------------------------------------------
// shifts and rotates: closed forms of the 68040's per-step semantics.
// count = cnt (a[5:0]; register counts are mod 64, immediate counts 1..8).
//--------------------------------------------------------------------------
logic [31:0] sh_r;
logic        sh_c, sh_x, sh_v;
// One right funnel (66 bits) does LSR, ASR, ROR, ROL, ROXR and ROXL; one
// left funnel does LSL and ASL.  The right funnel's input is the operand
// zero- or sign-extended (with a 0 below it: the bit just below the
// result is the last bit shifted out, the carry), or repeated with the
// rotate's period (the size, or the size + 1 with X for ROXR/ROXL), so a
// right shift by the rotate count leaves the rotated value at the bottom.
// A left rotate is a right rotate by the period less the count.
logic  [5:0] sh_n, sh_nm, sh_nx;
logic [65:0] rv, ro;
logic  [6:0] ra;
logic [65:0] lo;
always_comb begin
	logic [33:0] seg;
	logic [33:0] therm;
	sh_n  = cnt;
	sh_nm = sh_n & (nbits - 6'd1);
	// n mod (size + 1) for ROXL/ROXR by subtracting multiples (no divider)
	sh_nx = sh_n;
	case (nbits)
		6'd8: begin
			if (sh_nx >= 6'd36) sh_nx = sh_nx - 6'd36;
			if (sh_nx >= 6'd18) sh_nx = sh_nx - 6'd18;
			if (sh_nx >= 6'd9)  sh_nx = sh_nx - 6'd9;
		end
		6'd16: begin
			if (sh_nx >= 6'd34) sh_nx = sh_nx - 6'd34;
			if (sh_nx >= 6'd17) sh_nx = sh_nx - 6'd17;
		end
		default: if (sh_nx >= 6'd33) sh_nx = sh_nx - 6'd33;
	endcase
	// the right funnel's input and count.  A rotate's input repeats its
	// period P from bit 0 ({P, P}): shifted right by k (< the period), its
	// low bits are P rotated right by k
	case (op)
		OP_ASR:  begin
			rv = {{34{b_msb}}, bm, 1'b0};
			case (sz)
				SZ_B:    rv[65:9]  = {57{b_msb}};
				SZ_W:    rv[65:17] = {49{b_msb}};
				default: ;
			endcase
			ra = {1'b0, sh_n};
		end
		OP_ROL, OP_ROR: begin
			case (sz)
				SZ_B:    rv = {50'd0, bm[7:0], bm[7:0]};
				SZ_W:    rv = {34'd0, bm[15:0], bm[15:0]};
				default: rv = {2'd0, bm, bm};
			endcase
			ra = (op == OP_ROR) ? {1'b0, sh_nm} : {1'b0, (nbits - sh_nm) & (nbits - 6'd1)};
		end
		OP_ROXL, OP_ROXR: begin
			case (sz)
				SZ_B:    rv = {48'd0, f_x, bm[7:0], f_x, bm[7:0]};
				SZ_W:    rv = {32'd0, f_x, bm[15:0], f_x, bm[15:0]};
				default: rv = {f_x, bm, f_x, bm};
			endcase
			ra = (op == OP_ROXR) ? {1'b0, sh_nx}
			                     : ((sh_nx == 6'd0) ? 7'd0 : ({1'b0, nbits} + 7'd1 - {1'b0, sh_nx}));
		end
		default: begin   // LSR
			rv = {33'd0, bm, 1'b0};
			ra = {1'b0, sh_n};
		end
	endcase
	ro = (op == OP_ASR) ? 66'($signed(rv) >>> ra) : (rv >> ra);
	lo = {34'd0, bm} << sh_n;
	// ASL: the bits shifted out and the new sign bit must all be equal
	case (sz)
		SZ_B:    seg = lo[40:7];
		SZ_W:    seg = lo[48:15];
		default: seg = lo[64:31];
	endcase
	therm = (34'd2 << sh_n) - 34'd1;
	sh_r = bm;
	sh_c = 1'b0;
	sh_x = f_x;
	sh_v = 1'b0;
	if (sh_n != 6'd0) begin
		case (op)
			OP_ASL, OP_LSL: begin
				sh_r = lo[31:0] & szmask;
				sh_c = (sh_n <= nbits) && lo[nbits];
				sh_x = sh_c;
				if (op == OP_ASL)
					sh_v = (sh_n >= nbits) ? (bm != 0) : !((seg == 0) || (seg == therm));
			end
			OP_LSR, OP_ASR: begin
				sh_r = ro[32:1] & szmask;
				sh_c = ro[0];
				sh_x = sh_c;
			end
			OP_ROL, OP_ROR: begin
				sh_r = ro[31:0] & szmask;
				sh_c = (op == OP_ROL) ? sh_r[0] : sh_r[nbits - 6'd1];
			end
			default: begin   // ROXL, ROXR
				sh_x = ro[nbits];
				sh_r = ro[31:0] & szmask;
				sh_c = sh_x;
			end
		endcase
	end
	else if (op == OP_ROXL || op == OP_ROXR) begin
		// count 0: C = X, X unchanged
		sh_c = f_x;
	end
end

always_comb begin
	res       = b;
	flags_out = flags_in;
	case (op)
		OP_ABCD: begin
			res = {b[31:8], abcd_res[7:0]};
			flags_out = {abcd_c, f_n, f_z & (abcd_res[7:0] == 8'd0), f_v, abcd_c};
		end
		OP_SBCD, OP_NBCD: begin
			res = {b[31:8], sbcd_res[7:0]};
			flags_out = {sbcd_c, f_n, f_z & (sbcd_res[7:0] == 8'd0), f_v, sbcd_c};
		end
		OP_PACK: res = {b[31:8], pack_r};
		OP_UNPK: res = {b[31:16], unpk_r};
		OP_ASL, OP_ASR, OP_LSL, OP_LSR, OP_ROL, OP_ROR, OP_ROXL, OP_ROXR: begin
			res = merge(sh_r, b, szmask);
			flags_out = {sh_x, msb_of(sh_r, sz), (sh_r & szmask) == 32'd0, sh_v, sh_c};
		end
		default: ;
	endcase
end

endmodule
