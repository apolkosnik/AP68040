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
// The rotates use two funnels instead of a pair of shifters each: the
// operand repeated with the rotate's period from bit 0 ({P, P}) shifted
// right by k (< the period) has P rotated right by k at the bottom; a left
// rotate is a right rotate by the period less the count.
logic [65:0] rot_v, rox_v;      // the operand, and {X, operand}, repeated
logic  [5:0] rot_k, rox_k;
logic [65:0] rot_o, rox_o;
always_comb begin
	logic  [5:0] n, nm, nx, ne;
	logic [31:0] win;
	n  = cnt;
	nm = n & (nbits - 6'd1);
	// n mod (size + 1) for ROXL/ROXR by subtracting multiples (no divider)
	nx = n;
	case (nbits)
		6'd8: begin
			if (nx >= 6'd36) nx = nx - 6'd36;
			if (nx >= 6'd18) nx = nx - 6'd18;
			if (nx >= 6'd9)  nx = nx - 6'd9;
		end
		6'd16: begin
			if (nx >= 6'd34) nx = nx - 6'd34;
			if (nx >= 6'd17) nx = nx - 6'd17;
		end
		default: if (nx >= 6'd33) nx = nx - 6'd33;
	endcase
	ne = (n > nbits) ? nbits : n;
	case (sz)
		SZ_B:    begin rot_v = {50'd0, bm[7:0], bm[7:0]};   rox_v = {48'd0, f_x, bm[7:0], f_x, bm[7:0]}; end
		SZ_W:    begin rot_v = {34'd0, bm[15:0], bm[15:0]}; rox_v = {32'd0, f_x, bm[15:0], f_x, bm[15:0]}; end
		default: begin rot_v = {2'd0, bm, bm};              rox_v = {f_x, bm, f_x, bm}; end
	endcase
	rot_k = (op == OP_ROR) ? nm : ((nbits - nm) & (nbits - 6'd1));
	rox_k = (op == OP_ROXR) ? nx : ((nx == 6'd0) ? 6'd0 : (nbits + 6'd1 - nx));
	rot_o = rot_v >> rot_k;
	rox_o = rox_v >> rox_k;
	sh_r = bm;
	sh_c = 1'b0;
	sh_x = f_x;
	sh_v = 1'b0;
	win  = '0;
	if (n != 6'd0) begin
		case (op)
			OP_ASL, OP_LSL: begin
				sh_r = (bm << n) & szmask;
				sh_c = (n <= nbits) && (((bm >> (nbits - n)) & 32'd1) != 0);
				sh_x = sh_c;
				if (op == OP_ASL) begin
					if (n >= nbits) sh_v = (bm != 0);
					else begin
						win  = bm >> (nbits - 6'd1 - n);
						sh_v = !((win == 0) || (win == ((32'd2 << n) - 32'd1)));
					end
				end
			end
			OP_LSR: begin
				sh_r = bm >> n;
				sh_c = (n <= nbits) && (((bm >> (n - 6'd1)) & 32'd1) != 0);
				sh_x = sh_c;
			end
			OP_ASR: begin
				sh_r = (bm >> ne) | (b_msb ? ((~(szmask >> ne)) & szmask) : 32'd0);
				sh_c = (n >= nbits) ? b_msb : (((bm >> (n - 6'd1)) & 32'd1) != 0);
				sh_x = sh_c;
			end
			OP_ROL: begin
				sh_r = rot_o[31:0] & szmask;
				sh_c = sh_r[0];
			end
			OP_ROR: begin
				sh_r = rot_o[31:0] & szmask;
				sh_c = sh_r[nbits - 6'd1];
			end
			default: begin // OP_ROXL, OP_ROXR
				sh_x = rox_o[nbits];
				sh_r = rox_o[31:0] & szmask;
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
