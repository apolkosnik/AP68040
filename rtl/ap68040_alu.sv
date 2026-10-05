//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_alu.sv - EX-stage integer unit: arithmetic, logic, BCD, shifts   //
// and rotates, bit operations, condition codes                             //
//                                                                          //
// Conventions (same as the 68k "OP src,dst"):                              //
//   a  source operand, b destination operand, res = b OP a                 //
//   res is the full 32-bit register image: for byte and word sizes the     //
//   bits above the operand size are b's (so a data register write merges)  //
//   flags are {X,N,Z,V,C}; flags_out is the complete new CCR, the uop's    //
//   ccr_we mask decides which bits are actually written                    //
//                                                                          //
// Undefined-flag behaviour is the 68040's, as WinUAE's newcpu_common.cpp   //
// models it (verified against the cputest 68040 corpus on the previous     //
// core): BCD leaves N and V unchanged; CHK writes N, and C per             //
// setchkundefinedflags; CHK2/CMP2 write only Z and C.                      //
//--------------------------------------------------------------------------//

module ap68040_alu
	import ap68040_pkg::*;
(
	input  logic  [6:0] op,
	input  logic  [1:0] sz,
	input  logic  [3:0] cond,
	input  logic [31:0] a,
	input  logic [31:0] b,
	input  logic [31:0] ea,
	input  logic  [4:0] flags_in,   // X N Z V C
	output logic [31:0] res,
	output logic  [4:0] flags_out,
	output logic        cc_true,    // cond evaluated on flags_in
	output logic        trap        // CHK / CHK2 / TRAPcc exception
);

wire f_x = flags_in[4];
wire f_n = flags_in[3];
wire f_z = flags_in[2];
wire f_v = flags_in[1];
wire f_c = flags_in[0];

//--------------------------------------------------------------------------
// condition codes (MC68040UM table 3-19 / PRM 3.6)
//--------------------------------------------------------------------------
always_comb begin
	case (cond)
		4'h0: cc_true = 1'b1;                       // T
		4'h1: cc_true = 1'b0;                       // F
		4'h2: cc_true = !f_c && !f_z;               // HI
		4'h3: cc_true = f_c || f_z;                 // LS
		4'h4: cc_true = !f_c;                       // CC
		4'h5: cc_true = f_c;                        // CS
		4'h6: cc_true = !f_z;                       // NE
		4'h7: cc_true = f_z;                        // EQ
		4'h8: cc_true = !f_v;                       // VC
		4'h9: cc_true = f_v;                        // VS
		4'hA: cc_true = !f_n;                       // PL
		4'hB: cc_true = f_n;                        // MI
		4'hC: cc_true = f_n == f_v;                 // GE
		4'hD: cc_true = f_n != f_v;                 // LT
		4'hE: cc_true = !f_z && (f_n == f_v);       // GT
		default: cc_true = f_z || (f_n != f_v);     // LE
	endcase
end

//--------------------------------------------------------------------------
// sized views
//--------------------------------------------------------------------------
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

function automatic logic carry_of(input logic [32:0] v, input logic [1:0] s);
	case (s)
		SZ_B:    carry_of = v[8];
		SZ_W:    carry_of = v[16];
		default: carry_of = v[32];
	endcase
endfunction

wire a_msb = msb_of(a, sz);
wire b_msb = msb_of(b, sz);

// merge a sized result into b's upper bits
function automatic logic [31:0] merge(input logic [31:0] r, input logic [31:0] bb,
                                      input logic [31:0] m);
	merge = (r & m) | (bb & ~m);
endfunction

//--------------------------------------------------------------------------
// adder: one shared add/subtract with optional X-in
//--------------------------------------------------------------------------
logic        is_sub, use_x, b_zero;
always_comb begin
	is_sub = (op == OP_SUB) || (op == OP_SUBX) || (op == OP_CMP) ||
	         (op == OP_NEG) || (op == OP_NEGX);
	use_x  = (op == OP_ADDX) || (op == OP_SUBX) || (op == OP_NEGX);
	b_zero = (op == OP_NEG) || (op == OP_NEGX);
end

// NEG is 0 - b: the subtrahend is b, the minuend zero
wire [31:0] add_l  = b_zero ? 32'd0 : bm;
wire [31:0] add_r  = b_zero ? bm    : am;
wire        cin    = use_x & f_x;
wire [32:0] sum    = is_sub ? ({1'b0, add_l} - {1'b0, add_r} - {32'd0, cin})
                            : ({1'b0, add_l} + {1'b0, add_r} + {32'd0, cin});
wire        sum_c  = carry_of(sum, sz);
wire        sum_n  = msb_of(sum[31:0], sz);
wire        sum_z  = (sum[31:0] & szmask) == 32'd0;
wire        l_msb  = b_zero ? 1'b0  : b_msb;
wire        r_msb  = b_zero ? b_msb : a_msb;
wire        sum_v  = is_sub ? ((l_msb != r_msb) && (sum_n != l_msb))
                            : ((l_msb == r_msb) && (sum_n != l_msb));

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
// count = a[5:0] (register counts are mod 64, immediate counts 1..8).
//--------------------------------------------------------------------------
logic [31:0] sh_r;
logic        sh_c, sh_x, sh_v;
always_comb begin
	logic  [5:0] n, nm, nx, ne;
	logic [32:0] w, rot, cmask;
	logic [31:0] win;
	n  = a[5:0];
	nm = n & (nbits - 6'd1);
	nx = n % (nbits + 6'd1);
	ne = (n > nbits) ? nbits : n;
	cmask = (33'd2 << nbits) - 33'd1;
	w  = ({32'd0, f_x} << nbits) | {1'b0, bm};
	sh_r = bm;
	sh_c = 1'b0;
	sh_x = f_x;
	sh_v = 1'b0;
	rot  = '0;
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
				sh_r = ((bm << nm) | (bm >> (nbits - nm))) & szmask;
				sh_c = sh_r[0];
			end
			OP_ROR: begin
				sh_r = ((bm >> nm) | (bm << (nbits - nm))) & szmask;
				sh_c = ((sh_r >> (nbits - 6'd1)) & 32'd1) != 0;
			end
			OP_ROXL: begin
				rot  = ((w << nx) | (w >> (nbits + 6'd1 - nx))) & cmask;
				sh_x = ((rot >> nbits) & 33'd1) != 0;
				sh_r = rot[31:0] & szmask;
				sh_c = sh_x;
			end
			default: begin // OP_ROXR
				rot  = ((w >> nx) | (w << (nbits + 6'd1 - nx))) & cmask;
				sh_x = ((rot >> nbits) & 33'd1) != 0;
				sh_r = rot[31:0] & szmask;
				sh_c = sh_x;
			end
		endcase
	end
	else if (op == OP_ROXL || op == OP_ROXR) begin
		// count 0: C = X, X unchanged
		sh_c = f_x;
	end
end

//--------------------------------------------------------------------------
// bit operations: a holds the bit number, already reduced mod 32 (register)
// or mod 8 (memory byte) by the size
//--------------------------------------------------------------------------
wire  [4:0] bitn     = (sz == SZ_B) ? {2'b00, a[2:0]} : a[4:0];
wire [31:0] bit_mask = 32'd1 << bitn;
wire        bit_set  = |(b & bit_mask);

//--------------------------------------------------------------------------
// CHK (b = Dn, a = upper bound), sized W or L, signed
//--------------------------------------------------------------------------
wire        chk_dn_neg = b_msb;
wire        chk_ub_neg = a_msb;
wire [32:0] chk_diff   = {1'b0, am} - {1'b0, bm};   // bound - Dn
wire        chk_gt     = (chk_ub_neg != chk_dn_neg) ? chk_ub_neg
                                                    : msb_of(chk_diff[31:0], sz);
wire        chk_trap   = chk_dn_neg || chk_gt;
// setchkundefinedflags (68040): C set on trap per the src/dst relation
wire        chk_c      = (chk_dn_neg && !chk_ub_neg) ||
                         (!chk_ub_neg && !chk_dn_neg && chk_gt) ||
                         (chk_dn_neg && chk_ub_neg && msb_of(chk_diff[31:0], sz));

//--------------------------------------------------------------------------
// CHK2/CMP2: b is the value, a the bound; signedness from cond[1]
// (signed when the instruction's register is an address register or the
// bounds compare signed: the 68040 compares as signed when the bounds are
// signed, i.e. lower > upper as unsigned is the wrap case).  The two steps
// keep the lower-bound result in chk2_lo for the second.
//--------------------------------------------------------------------------
// handled in the back end (needs state across two uops)

//--------------------------------------------------------------------------
// result and flags
//--------------------------------------------------------------------------
always_comb begin
	logic [31:0] r;
	r         = b;
	flags_out = flags_in;
	trap      = 1'b0;
	res       = b;

	case (op)
		OP_MOV: begin
			r = merge(am, b, szmask);
			flags_out = {f_x, msb_of(am, sz), am == 32'd0, 1'b0, 1'b0};
		end

		OP_ADD, OP_SUB: begin
			r = merge(sum[31:0], b, szmask);
			flags_out = {sum_c, sum_n, sum_z, sum_v, sum_c};
		end

		OP_ADDX, OP_SUBX, OP_NEGX: begin
			r = merge(sum[31:0], b, szmask);
			flags_out = {sum_c, sum_n, f_z & sum_z, sum_v, sum_c};
		end

		OP_NEG: begin
			r = merge(sum[31:0], b, szmask);
			flags_out = {sum_c, sum_n, sum_z, sum_v, sum_c};
		end

		OP_CMP: begin
			r = b;
			flags_out = {f_x, sum_n, sum_z, sum_v, sum_c};
		end

		OP_AND, OP_OR, OP_EOR, OP_NOT: begin
			logic [31:0] l;
			case (op)
				OP_AND:  l = bm & am;
				OP_OR:   l = bm | am;
				OP_EOR:  l = bm ^ am;
				default: l = ~bm & szmask;
			endcase
			r = merge(l, b, szmask);
			flags_out = {f_x, msb_of(l, sz), l == 32'd0, 1'b0, 1'b0};
		end

		OP_CLR: begin
			r = merge(32'd0, b, szmask);
			flags_out = {f_x, 1'b0, 1'b1, 1'b0, 1'b0};
		end

		OP_EXT: begin
			if (sz == SZ_W) begin
				r = {b[31:16], {8{b[7]}}, b[7:0]};
				flags_out = {f_x, b[7], b[7:0] == 8'd0, 1'b0, 1'b0};
			end
			else begin
				r = {{16{b[15]}}, b[15:0]};
				flags_out = {f_x, b[15], b[15:0] == 16'd0, 1'b0, 1'b0};
			end
		end

		OP_EXTB: begin
			r = {{24{b[7]}}, b[7:0]};
			flags_out = {f_x, b[7], b[7:0] == 8'd0, 1'b0, 1'b0};
		end

		OP_SWAP: begin
			r = {b[15:0], b[31:16]};
			flags_out = {f_x, b[15], b == 32'd0, 1'b0, 1'b0};
		end

		OP_TAS: begin
			r = {b[31:8], 1'b1, b[6:0]};
			flags_out = {f_x, b[7], b[7:0] == 8'd0, 1'b0, 1'b0};
		end

		OP_ABCD: begin
			r = {b[31:8], abcd_res[7:0]};
			flags_out = {abcd_c, f_n, f_z & (abcd_res[7:0] == 8'd0), f_v, abcd_c};
		end

		OP_SBCD, OP_NBCD: begin
			r = {b[31:8], sbcd_res[7:0]};
			flags_out = {sbcd_c, f_n, f_z & (sbcd_res[7:0] == 8'd0), f_v, sbcd_c};
		end

		OP_PACK: r = {b[31:8], pack_r};
		OP_UNPK: r = {b[31:16], unpk_r};

		OP_ASL, OP_ASR, OP_LSL, OP_LSR, OP_ROL, OP_ROR, OP_ROXL, OP_ROXR: begin
			r = merge(sh_r, b, szmask);
			flags_out = {sh_x, msb_of(sh_r, sz), (sh_r & szmask) == 32'd0, sh_v, sh_c};
		end

		OP_BTST: begin
			r = b;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end
		OP_BCHG: begin
			r = b ^ bit_mask;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end
		OP_BCLR: begin
			r = b & ~bit_mask;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end
		OP_BSET: begin
			r = b | bit_mask;
			flags_out = {f_x, f_n, !bit_set, f_v, f_c};
		end

		OP_SCC: r = {b[31:8], {8{cc_true}}};

		OP_EA: r = ea;

		OP_DBCC: r = {b[31:16], b[15:0] - 16'd1};

		OP_TRAPCC: trap = cc_true;

		OP_CHK: begin
			trap = chk_trap;
			flags_out = {f_x, chk_dn_neg, f_z, f_v, chk_trap ? chk_c : 1'b0};
		end

		OP_CCRLOG: begin
			case (cond[1:0])
				2'd0:    flags_out = flags_in & a[4:0];
				2'd1:    flags_out = flags_in | a[4:0];
				2'd2:    flags_out = flags_in ^ a[4:0];
				default: flags_out = a[4:0];
			endcase
		end

		default: r = b;
	endcase
	res = r;
end

endmodule
