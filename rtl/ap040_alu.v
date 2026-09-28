//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_alu.v - integer ALU: arithmetic, logic, BCD, single-bit shift      //
// primitives and bit operations, with MC68040 CCR semantics                //
//                                                                          //
// Conventions:                                                             //
//  - a is the source operand, b is the destination operand                 //
//    (68k "OP src,dst" computes dst = dst OP src, i.e. result = b OP a)    //
//  - operands are used in the low bits according to size; the result is    //
//    returned in the low bits, upper bits zero (the core merges by size)   //
//  - flags are {X,N,Z,V,C}                                                 //
//  - shifts/rotates are single-bit primitives; the core iterates them so   //
//    per-step X/C/V accumulate exactly like the hardware                   //
//  - for bit operations, a holds the pre-masked bit number                 //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

// PIPELINE_SUBSET is only for the integer pipeline's restricted decoder.
// The legacy sequencer retains every operation with the default value.
module ap040_alu #(parameter PIPELINE_SUBSET = 0)
(
	input       [5:0] op,
	input       [1:0] size,       // AP040_SZ_B/W/L
	input       [5:0] shcnt,      // shift/rotate count this call (1..63)
	input      [31:0] a,
	input      [31:0] b,
	input       [4:0] flags_in,   // {X,N,Z,V,C}
	output reg [31:0] result,
	output reg  [4:0] flags_out,
	// The compare-class flags without the result mux: ADD, SUB, CMP, MOVE,
	// TST, AND, OR, EOR straight from the shared adder and the masked
	// operands.  Identical to flags_out for those operations; the core's
	// branch lookahead judges a following Bcc on them within the retire
	// cycle (the full flags_out came 8 ns later through the shifter and
	// result select).  fast_ok says the operation is one of them.
	output reg  [4:0] fast_flags,
	output reg        fast_ok
);

wire f_x = flags_in[4];
wire f_z = flags_in[2];

function [31:0] reverse32;
	input [31:0] value;
	integer bit_index;
	begin
		for (bit_index = 0; bit_index < 32; bit_index = bit_index + 1)
			reverse32[bit_index] = value[31-bit_index];
	end
endfunction

// size-dependent views
wire [5:0]  nbits = (size == `AP040_SZ_B) ? 6'd8 : (size == `AP040_SZ_W) ? 6'd16 : 6'd32;
wire [31:0] szmask = (size == `AP040_SZ_B) ? 32'h0000_00FF :
                     (size == `AP040_SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
wire [31:0] am = a & szmask;
wire [31:0] bm = b & szmask;
wire        a_msb = (size == `AP040_SZ_B) ? a[7]  : (size == `AP040_SZ_W) ? a[15] : a[31];
wire        b_msb = (size == `AP040_SZ_B) ? b[7]  : (size == `AP040_SZ_W) ? b[15] : b[31];
wire shift_left = (op == `AP040_ALU_ASL1) || (op == `AP040_ALU_LSL1);
wire shift_arithmetic = (op == `AP040_ALU_ASR1);
wire shift_signfill = shift_arithmetic && b_msb;
wire [31:0] shift_input = shift_left ? reverse32(bm) :
	(shift_signfill ? (bm | ~szmask) : bm);
wire signed [32:0] shift_signed_input = {shift_signfill, shift_input};
wire [31:0] shift_right_result = shift_signed_input >>> shcnt;
wire [31:0] shared_shift_result =
	(shift_left ? reverse32(shift_right_result) : shift_right_result) & szmask;

function res_msb;
	input [31:0] r;
	begin
		res_msb = (size == `AP040_SZ_B) ? r[7] : (size == `AP040_SZ_W) ? r[15] : r[31];
	end
endfunction

function res_zero;
	input [31:0] r;
	begin
		res_zero = ((r & szmask) == 32'd0);
	end
endfunction

// Select extend before arithmetic: ADD/ADDX share an adder, and
// SUB/SUBX/CMP share a subtractor. Ordinary operations ignore incoming X.
// The fast-flag block below reads the same adders and stays exact because
// the extend is gated on the ADDX/SUBX opcode, which it never selects.
wire add_extend = !PIPELINE_SUBSET && (op == `AP040_ALU_ADDX) && f_x;
wire sub_extend = !PIPELINE_SUBSET && (op == `AP040_ALU_SUBX) && f_x;
wire [32:0] add_full = {1'b0, bm} + {1'b0, am} + {32'd0, add_extend};
wire [32:0] sub_full = {1'b0, bm} - {1'b0, am} - {32'd0, sub_extend};

always @* begin
	fast_ok = 1'b1;
	case (op)
		`AP040_ALU_ADD: fast_flags = {add_c, add_r_msb, res_zero(add_full[31:0]), add_v, add_c};
		`AP040_ALU_SUB: fast_flags = {sub_c, sub_r_msb, res_zero(sub_full[31:0]), sub_v, sub_c};
		`AP040_ALU_CMP: fast_flags = {f_x, sub_r_msb, res_zero(sub_full[31:0]), sub_v, sub_c};
		`AP040_ALU_MOVE, `AP040_ALU_TST:
		                fast_flags = {f_x, res_msb(am), res_zero(am), 1'b0, 1'b0};
		`AP040_ALU_AND: fast_flags = {f_x, res_msb(bm & am), res_zero(bm & am), 1'b0, 1'b0};
		`AP040_ALU_OR:  fast_flags = {f_x, res_msb(bm | am), res_zero(bm | am), 1'b0, 1'b0};
		`AP040_ALU_EOR: fast_flags = {f_x, res_msb(bm ^ am), res_zero(bm ^ am), 1'b0, 1'b0};
		default: begin fast_flags = flags_in; fast_ok = 1'b0; end
	endcase
end

// carry out of the sized MSB position for byte/word needs the sized bit
wire add_c  = (size == `AP040_SZ_B) ? add_full[8]  : (size == `AP040_SZ_W) ? add_full[16]  : add_full[32];
wire sub_c  = (size == `AP040_SZ_B) ? sub_full[8]  : (size == `AP040_SZ_W) ? sub_full[16]  : sub_full[32];

// explicit size selects: res_msb reads `size` from inside the function
// body, which iverilog leaves out of a continuous assignment's sensitivity
// (functions reading module state are only safe from the main @* block,
// which reads szmask directly and so re-evaluates on every size change)
wire add_r_msb  = (size == `AP040_SZ_B) ? add_full[7]  : (size == `AP040_SZ_W) ? add_full[15]  : add_full[31];
wire sub_r_msb  = (size == `AP040_SZ_B) ? sub_full[7]  : (size == `AP040_SZ_W) ? sub_full[15]  : sub_full[31];

wire add_v  = (a_msb == b_msb) && (add_r_msb  != a_msb);
wire sub_v  = (a_msb != b_msb) && (sub_r_msb  == a_msb);

// BCD helpers (byte only). The decimal corrections are applied to the
// whole byte so a +/-6 low-nibble adjust ripples binary into the high
// nibble, and the carry comes from the corrected value above bit 3.
// This matches real hardware for non-BCD digit inputs (cputest
// 68040_default: abcd.b $FF+$FF+0 = $64 with C set, not $54).
wire [4:0] bcd_al   = {1'b0, b[3:0]} + {1'b0, a[3:0]} + {4'd0, f_x};
wire [9:0] bcd_asum = {2'd0, b[7:0]} + {2'd0, a[7:0]} + {9'd0, f_x}
                    + ((bcd_al > 5'd9) ? 10'd6 : 10'd0);
wire       bcd_ac   = ((bcd_asum & 10'h3F0) > 10'h090);
wire [9:0] bcd_ares = bcd_asum + (bcd_ac ? 10'h060 : 10'd0);

// SBCD: b - a - X; the $60 adjust keys off the uncorrected byte borrow,
// the carry flag off the borrow after the low-nibble correction
wire       bcd_slb  = ({1'b0, b[3:0]} < ({1'b0, a[3:0]} + {4'd0, f_x}));
wire [9:0] bcd_sraw = {2'd0, b[7:0]} - {2'd0, a[7:0]} - {9'd0, f_x};
wire [9:0] bcd_scor = bcd_sraw - (bcd_slb ? 10'd6 : 10'd0);
wire [9:0] bcd_sres = bcd_scor - (bcd_sraw[9] ? 10'h060 : 10'd0);
wire       bcd_sc   = bcd_scor[9];

// NBCD: 0 - b - X (the SBCD datapath with a zero destination; b is the
// pipeline dst operand -- single-operand ops must not touch port a,
// which still holds the previous instruction's source)
wire       nbc_lb   = (b[3:0] != 4'd0) | f_x;
wire [9:0] nbc_raw  = 10'd0 - {2'd0, b[7:0]} - {9'd0, f_x};
wire [9:0] nbc_cor  = nbc_raw - (nbc_lb ? 10'd6 : 10'd0);
wire [9:0] nbc_res  = nbc_cor - (nbc_raw[9] ? 10'h060 : 10'd0);
wire       nbc_c    = nbc_cor[9];

// single-bit shift/rotate primitives on the sized value bm
wire sh_msb  = b_msb;
wire sh_msb2 = (size == `AP040_SZ_B) ? b[6] : (size == `AP040_SZ_W) ? b[14] : b[30];
wire sh_lsb  = b[0];

wire [31:0] shl_r  = (bm << 1) & szmask;
wire [31:0] shr_l  = bm >> 1;
wire [31:0] asr_r  = shr_l | (sh_msb ? ((szmask >> 1) ^ szmask) : 32'd0);  // sign fill
wire [31:0] rol_r  = shl_r | {31'd0, sh_msb};
wire [31:0] ror_r  = shr_l | (sh_lsb ? (szmask ^ (szmask >> 1)) : 32'd0); // lsb to msb
wire [31:0] roxl_r = shl_r | {31'd0, f_x};
wire [31:0] roxr_r = shr_l | (f_x ? (szmask ^ (szmask >> 1)) : 32'd0);

// bit operations: bit number in a (pre-masked by the core: mod 32 or mod 8)
wire [31:0] bit_mask = 32'd1 << a[4:0];
wire        bit_set  = |(b & bit_mask);

// shared intermediate results
wire [31:0] negx_res = (32'd0 - bm - {31'd0, f_x}) & szmask;
wire [31:0] ext_res  = (size == `AP040_SZ_W) ? {16'd0, {8{b[7]}}, b[7:0]}
                                             : {{16{b[15]}}, b[15:0]};

always @* begin
	result    = 32'd0;
	flags_out = flags_in;

	case (op)
		`AP040_ALU_MOVE, `AP040_ALU_TST: begin
			result = am;
			flags_out = {f_x, res_msb(am), res_zero(am), 1'b0, 1'b0};
		end

		`AP040_ALU_ADD: begin
			result = add_full[31:0] & szmask;
			flags_out = {add_c, add_r_msb, res_zero(add_full[31:0]), add_v, add_c};
		end

		`AP040_ALU_ADDX: if (!PIPELINE_SUBSET) begin
			result = add_full[31:0] & szmask;
			flags_out = {add_c, add_r_msb, f_z & res_zero(add_full[31:0]), add_v, add_c};
		end

		`AP040_ALU_SUB: begin
			result = sub_full[31:0] & szmask;
			flags_out = {sub_c, sub_r_msb, res_zero(sub_full[31:0]), sub_v, sub_c};
		end

		`AP040_ALU_SUBX: if (!PIPELINE_SUBSET) begin
			result = sub_full[31:0] & szmask;
			flags_out = {sub_c, sub_r_msb, f_z & res_zero(sub_full[31:0]), sub_v, sub_c};
		end

		`AP040_ALU_CMP: begin
			result = bm;   // destination unchanged
			flags_out = {f_x, sub_r_msb, res_zero(sub_full[31:0]), sub_v, sub_c};
		end

		`AP040_ALU_AND: begin
			result = bm & am;
			flags_out = {f_x, res_msb(bm & am), res_zero(bm & am), 1'b0, 1'b0};
		end

		`AP040_ALU_OR: begin
			result = bm | am;
			flags_out = {f_x, res_msb(bm | am), res_zero(bm | am), 1'b0, 1'b0};
		end

		`AP040_ALU_EOR: begin
			result = bm ^ am;
			flags_out = {f_x, res_msb(bm ^ am), res_zero(bm ^ am), 1'b0, 1'b0};
		end

		`AP040_ALU_NOT: if (!PIPELINE_SUBSET) begin
			result = (~bm) & szmask;
			flags_out = {f_x, res_msb(~bm), res_zero(~bm), 1'b0, 1'b0};
		end

		`AP040_ALU_NEG: if (!PIPELINE_SUBSET) begin
			// 0 - b
			result = (32'd0 - bm) & szmask;
			flags_out = {|bm ? 1'b1 : 1'b0,
			             res_msb(32'd0 - bm),
			             res_zero(32'd0 - bm),
			             res_msb(bm) & res_msb(32'd0 - bm),
			             |bm ? 1'b1 : 1'b0};
		end

		`AP040_ALU_NEGX: if (!PIPELINE_SUBSET) begin
			// 0 - b - X
			result = negx_res;
			flags_out = {(|bm | f_x),
			             res_msb(negx_res),
			             f_z & res_zero(negx_res),
			             res_msb(bm) & res_msb(negx_res),
			             (|bm | f_x)};
		end

		`AP040_ALU_CLR: if (!PIPELINE_SUBSET) begin
			result = 32'd0;
			flags_out = {f_x, 1'b0, 1'b1, 1'b0, 1'b0};
		end

		`AP040_ALU_EXT: if (!PIPELINE_SUBSET) begin
			// size W: byte to word; size L: word to long
			result = ext_res;
			flags_out = {f_x, res_msb(ext_res), res_zero(ext_res), 1'b0, 1'b0};
		end

		`AP040_ALU_EXTB: if (!PIPELINE_SUBSET) begin
			result = {{24{b[7]}}, b[7:0]};
			flags_out = {f_x, b[7], (b[7:0] == 8'd0), 1'b0, 1'b0};
		end

		`AP040_ALU_SWAP: if (!PIPELINE_SUBSET) begin
			result = {b[15:0], b[31:16]};
			flags_out = {f_x, b[15], (b == 32'd0), 1'b0, 1'b0};
		end

		`AP040_ALU_TAS: if (!PIPELINE_SUBSET) begin
			result = {24'd0, 1'b1, b[6:0]};
			flags_out = {f_x, b[7], (b[7:0] == 8'd0), 1'b0, 1'b0};
		end

		// BCD: on the real 68040 the architecturally undefined N and V
		// flags are left unchanged (verified with cputest 68040_default
		// reference data on hardware); X/C carry out, Z is sticky
		`AP040_ALU_ABCD: if (!PIPELINE_SUBSET) begin
			result = {24'd0, bcd_ares[7:0]};
			flags_out = {bcd_ac, flags_in[3], f_z & (bcd_ares[7:0] == 8'd0), flags_in[1], bcd_ac};
		end

		`AP040_ALU_SBCD: if (!PIPELINE_SUBSET) begin
			result = {24'd0, bcd_sres[7:0]};
			flags_out = {bcd_sc, flags_in[3], f_z & (bcd_sres[7:0] == 8'd0), flags_in[1], bcd_sc};
		end

		`AP040_ALU_NBCD: if (!PIPELINE_SUBSET) begin
			result = {24'd0, nbc_res[7:0]};
			flags_out = {nbc_c, flags_in[3], f_z & (nbc_res[7:0] == 8'd0), flags_in[1], nbc_c};
		end

		`AP040_ALU_ASL1, `AP040_ALU_LSL1, `AP040_ALU_ASR1,
		`AP040_ALU_LSR1, `AP040_ALU_ROL1, `AP040_ALU_ROR1,
		`AP040_ALU_ROXL1, `AP040_ALU_ROXR1: begin : sh_barrel
			// single-cycle barrel: closed forms equal to composing shcnt
			// (1..63) of the former one-bit steps.  Verified equivalences:
			// shifts: C=X=last bit out; ASL V = the top shcnt+1 bits of the
			// source are not all equal (any-step MSB change); plain rotates
			// leave X and take C from the bit that wrapped last; ROXx
			// rotates the (size+1)-bit {X,value} container by shcnt mod
			// (size+1) and reports C = the rotated X for every nonzero
			// count, including exact multiples of size+1.
			reg  [5:0] n, nm, nx, ne;
			reg [32:0] w, rot, cmask, rotate_in, rotate_mask;
			reg [5:0] rotate_width, rotate_amount, rotate_left;
			reg rotate_extend, rotate_right;
			reg [31:0] r, sext, win;
			reg        c, x2, vf;
			n  = shcnt;
			r  = 32'd0; c = 1'b0; x2 = f_x; vf = 1'b0;
			nm = n & (nbits - 6'd1);
			// n mod (size+1) for the ROXx container without a variable-modulus
			// divider: the three sizes give constant moduli 9, 17 and 33, and
			// with n < 64 each is a few conditional subtractions.  The
			// variable form synthesized a 33-bit divider that sat in every
			// register shift's result and flag path (6 ns, 2026-09-17).
			case (nbits)
				6'd8:  nx = (n >= 6'd63) ? n - 6'd63 : (n >= 6'd54) ? n - 6'd54 :
				            (n >= 6'd45) ? n - 6'd45 : (n >= 6'd36) ? n - 6'd36 :
				            (n >= 6'd27) ? n - 6'd27 : (n >= 6'd18) ? n - 6'd18 :
				            (n >= 6'd9)  ? n - 6'd9  : n;
				6'd16: nx = (n >= 6'd51) ? n - 6'd51 : (n >= 6'd34) ? n - 6'd34 :
				            (n >= 6'd17) ? n - 6'd17 : n;
				default: nx = (n >= 6'd33) ? n - 6'd33 : n;
			endcase
			ne = (n > nbits) ? nbits : n;
			cmask = (33'd2 << nbits) - 33'd1;
			w = ({32'd0, f_x} << nbits) | {1'b0, bm};
			rotate_extend = (op == `AP040_ALU_ROXL1) || (op == `AP040_ALU_ROXR1);
			rotate_right = (op == `AP040_ALU_ROR1) || (op == `AP040_ALU_ROXR1);
			rotate_width = nbits + rotate_extend;
			rotate_amount = rotate_extend ? nx : nm;
			rotate_left = rotate_right ? (rotate_width - rotate_amount) : rotate_amount;
			rotate_in = rotate_extend ? w : {1'b0, bm};
			rotate_mask = rotate_extend ? cmask : {1'b0, szmask};
			rot = ((rotate_in << rotate_left) | (rotate_in >> (rotate_width - rotate_left))) & rotate_mask;
			case (op)
				`AP040_ALU_ASL1, `AP040_ALU_LSL1: begin
					r = shared_shift_result;
					c = (n <= nbits) && (((bm >> (nbits - n)) & 32'd1) != 0);
					x2 = c;
					if (op == `AP040_ALU_ASL1) begin
						if (n >= nbits) vf = (bm != 0);
						else begin
							win = bm >> (nbits - 6'd1 - n);
							vf = !((win == 0) ||
							       (win == ((32'd2 << n) - 32'd1)));
						end
					end
				end
				`AP040_ALU_LSR1: begin
					r = shared_shift_result;
					c = (n <= nbits) && (((bm >> (n - 6'd1)) & 32'd1) != 0);
					x2 = c;
				end
				`AP040_ALU_ASR1: begin
					// Sign-extend for arithmetic shifts before the shared signed right shift.
					r = shared_shift_result;
					c = (n >= nbits) ? b_msb
					                 : (((bm >> (n - 6'd1)) & 32'd1) != 0);
					x2 = c;
				end
				`AP040_ALU_ROL1: begin
					r = rot[31:0] & szmask;
					c = r[0];
				end
				`AP040_ALU_ROR1: begin
					r = rot[31:0] & szmask;
					c = ((r >> (nbits - 6'd1)) & 32'd1) != 0;
				end
				`AP040_ALU_ROXL1: begin
					x2 = ((rot >> nbits) & 33'd1) != 0;
					r = rot[31:0] & szmask;
					c = x2;
				end
				default: begin // AP040_ALU_ROXR1
					x2 = ((rot >> nbits) & 33'd1) != 0;
					r = rot[31:0] & szmask;
					c = x2;
				end
			endcase
			result = r;
			flags_out = {x2, res_msb(r), res_zero(r), vf, c};
		end

		`AP040_ALU_BTST: if (!PIPELINE_SUBSET) begin
			result = bm;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end

		`AP040_ALU_BCHG: if (!PIPELINE_SUBSET) begin
			result = (bm ^ bit_mask) & szmask;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end

		`AP040_ALU_BCLR: if (!PIPELINE_SUBSET) begin
			result = bm & ~bit_mask;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end

		`AP040_ALU_BSET: if (!PIPELINE_SUBSET) begin
			result = (bm | bit_mask) & szmask;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end

		default: begin
			result = bm;
			flags_out = flags_in;
		end
	endcase
end

endmodule
