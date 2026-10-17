//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                            //
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
	// the shift's kind, one-hot, decoded from op as the uop enters EX and
	// registered (the funnel's amount and input selects start from these
	// flip-flops, not from a decode of op): ASL LSL ASR LSR ROL ROR ROXL ROXR
	input  logic  [7:0] shk,
	input  logic  [1:0] sz,
	input  logic [31:0] a,
	input  logic [31:0] b,
	input  logic  [5:0] cnt,        // shift count: a[5:0], without a's mux
	input  logic  [5:0] nx_q,       // ROXL/ROXR: the count mod (size + 1), registered
	output logic  [5:0] nx,         // ... formed here from cnt (for that register)
	input  logic  [6:0] nxl_q,      // ROXL: the funnel's amount, size + 1 - nx, registered
	output logic  [6:0] nxl,        // ... formed here from nx (for that register)
	input  logic  [4:0] flags_in,   // X N Z V C
	output logic [31:0] res,
	output logic  [4:0] flags_out,
	output logic        sim_c       // (simulation: the carry as the funnel forms it)
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
	logic [31:0] yv, tm;
	logic        vl;
	sh_n  = cnt;
	sh_nm = sh_n & (nbits - 6'd1);
	// n mod (size + 1) for ROXL/ROXR by subtracting multiples (no divider):
	// formed here (nx), registered by the back end in a cycle of its own,
	// and used from that register (nx_q) -- the subtractions stay off the
	// shifter's path
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
	nx    = sh_nx;
	// ROXL's funnel amount, size + 1 - the count, formed with the count
	// (its subtraction in the pre-cycle too, off the funnel's path)
	nxl   = (sh_nx == 6'd0) ? 7'd0 : ({1'b0, nbits} + 7'd1 - {1'b0, sh_nx});
	sh_nx = nx_q;
	// the right funnel's input and count.  A rotate's input repeats its
	// period P from bit 0 ({P, P}): shifted right by k (< the period), its
	// low bits are P rotated right by k
	if (shk[2]) begin               // ASR
		rv = {{34{b_msb}}, bm, 1'b0};
		case (sz)
			SZ_B:    rv[65:9]  = {57{b_msb}};
			SZ_W:    rv[65:17] = {49{b_msb}};
			default: ;
		endcase
		ra = {1'b0, sh_n};
	end
	else if (shk[4] || shk[5]) begin    // ROL, ROR
		case (sz)
			SZ_B:    rv = {50'd0, bm[7:0], bm[7:0]};
			SZ_W:    rv = {34'd0, bm[15:0], bm[15:0]};
			default: rv = {2'd0, bm, bm};
		endcase
		// ROL by k: the doubled operand shifted right by size - 1 - k
		// (the count's complement within the size: no subtraction) and
		// taken one bit up (ro[32:1], as a right shift's result)
		ra = shk[5] ? {1'b0, sh_nm} : {1'b0, ~sh_nm & (nbits - 6'd1)};
	end
	else if (shk[6] || shk[7]) begin    // ROXL, ROXR
		case (sz)
			SZ_B:    rv = {48'd0, f_x, bm[7:0], f_x, bm[7:0]};
			SZ_W:    rv = {32'd0, f_x, bm[15:0], f_x, bm[15:0]};
			default: rv = {f_x, bm, f_x, bm};
		endcase
		ra = shk[7] ? {1'b0, sh_nx} : nxl_q;
	end
	else if (shk[0] || shk[1]) begin    // ASL, LSL
		// a left shift through the same funnel: the operand above 31
		// zeros, shifted right by 31 - n (the count inverted, no
		// subtraction), is the operand shifted left by n; n >= 32
		// shifts everything out (C of a long shifted by 32: below)
		rv = {3'd0, bm, 31'd0};
		ra = sh_n[5] ? 7'd127 : {2'd0, ~sh_n[4:0]};
	end
	else begin                          // LSR
		rv = {33'd0, bm, 1'b0};
		ra = {1'b0, sh_n};
	end
	ro = shk[2] ? 66'($signed(rv) >>> ra) : (rv >> ra);
	lo = ro;                      // (ASL, LSL: the operand shifted left by n)
	// ASL: V when the bits shifted out and the new sign bit are not all
	// equal, i.e. the top count+1 bits of the operand (at the size) differ
	// somewhere: an adjacent pair among them differs.  yv[i] = x[i] ^ x[i-1],
	// tm marks the top count positions; no shifter on the path
	yv = bm ^ {bm[30:0], 1'b0};
	for (int i = 0; i < 32; i++)
		tm[i] = (6'(i) < nbits) && ((nbits - 6'd1 - 6'(i)) < sh_n);
	vl = (yv & tm) != 32'd0;
	sh_r = bm;
	sh_v = 1'b0;
	if (sh_n != 6'd0) begin
		if (shk[0] || shk[1]) begin         // ASL, LSL
			sh_r = lo[31:0] & szmask;
			if (shk[0])
				sh_v = (sh_n >= nbits) ? (bm != 0) : vl;
		end
		else if (shk[3] || shk[2] || shk[4]) sh_r = ro[32:1] & szmask;   // LSR, ASR, ROL
		else sh_r = ro[31:0] & szmask;      // ROR, ROXL, ROXR
	end
	// The carry (the last bit moved out) and X come from the operand bit
	// at that step's index -- one 32:1 multiplexer on the operand, the
	// index a subtraction from the count -- not from the funnel: C and X
	// are needed in the op's first cycle (N and Z come from the registered
	// result in the second), and the funnel with the flag multiplexers
	// behind it does not close at 60 MHz.  The funnel's own carry (sim_c)
	// is checked against it in simulation.
	//   LSL/ASL by n (1..size):  bit size-n;  n > size: 0
	//   LSR by n:                bit n-1;     n > size: 0;  ASR: the sign
	//   ROL by k = n mod size:   bit size-k (k = 0: bit 0)
	//   ROR by k:                bit k-1     (k = 0: bit size-1)
	//   ROXL by m = n mod (size+1): bit size-m;  ROXR: bit m-1;  m = 0: X
	//   count 0: C = 0 (ROXL/ROXR: X); X unchanged (shifts: X = C)
	begin : direct_c
		logic [5:0] ck, ci;
		logic       left, rox, rot, c_bit;
		rox  = shk[6] || shk[7];
		rot  = shk[4] || shk[5];
		left = shk[0] || shk[1] || shk[4] || shk[6];
		ck   = rox ? sh_nx : rot ? sh_nm : sh_n;
		ci   = left ? (nbits - ck) : (ck - 6'd1);
		if (rot && sh_nm == 6'd0) ci = shk[4] ? 6'd0 : (nbits - 6'd1);
		c_bit = bm[ci[4:0]];
		if (sh_n == 6'd0)             sh_c = rox ? f_x : 1'b0;
		else if (rox)                 sh_c = (sh_nx == 6'd0) ? f_x : c_bit;
		else if (rot)                 sh_c = c_bit;
		else if (sh_n > nbits)        sh_c = shk[2] ? b_msb : 1'b0;
		else                          sh_c = c_bit;
		sh_x = (sh_n != 6'd0 && !rot) ? sh_c : f_x;
	end
end
// (simulation) the carry from the funnel, the form the direct one replaces
always_comb begin
	sim_c = 1'b0;
	if (sh_n != 6'd0) begin
		case (op)
			OP_ASL, OP_LSL: sim_c = (sh_n <= nbits) && (sh_n[5] ? bm[0] : lo[nbits]);
			OP_LSR, OP_ASR: sim_c = ro[0];
			OP_ROL:         sim_c = ro[1];
			OP_ROR:         sim_c = ro[nbits - 6'd1];
			default:        sim_c = ro[nbits];   // ROXL, ROXR
		endcase
	end
	else if (op == OP_ROXL || op == OP_ROXR) sim_c = f_x;
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
			// (N and Z: the back end forms them from the registered result
			// in the op's second cycle; nothing of the funnel reaches the
			// flags here)
			res = merge(sh_r, b, szmask);
			flags_out = {sh_x, 1'b0, 1'b0, sh_v, sh_c};
		end
		default: ;
	endcase
end

endmodule
