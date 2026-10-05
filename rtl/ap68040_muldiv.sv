//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_muldiv.sv - iterative multiply and divide (from AP040's          //
// ap040_muldiv.v, which the cputest corpus validated)                       //
//                                                                          //
// MUL: 32x32 -> 64, signed or unsigned, one registered DSP product         //
// DIV: 64/32 -> q32,r32 restoring, 4 bits per cycle; ovf when the true     //
//      quotient does not fit 32 bits.  Signed quotients truncate toward    //
//      zero, the remainder takes the dividend's sign.                       //
// kill abandons a running operation (pipeline flush).                       //
//--------------------------------------------------------------------------//

module ap68040_muldiv
(
	input             clk,
	input             nreset,
	input             kill,

	input             start,
	input             is_div,
	input             sign_op,
	input      [31:0] op_a,        // multiplier / divisor
	input      [31:0] op_hi,       // dividend high (div only)
	input      [31:0] op_lo,       // multiplicand / dividend low

	output            busy,
	output reg        done,        // one cycle pulse
	output reg [31:0] res_hi,      // product high / remainder
	output reg [31:0] res_lo,      // product low / quotient
	output reg        ovf
);

reg        running;
reg        div_r;
reg        neg_q;                 // negate quotient / product
reg        neg_r;                 // negate remainder
reg  [6:0] count;
reg [31:0] den;                   // divisor / multiplier (absolute)
reg [31:0] mcand;                 // multiplicand (absolute)
reg [63:0] prod;                  // registered DSP product
reg [96:0] acc;

// absolute values for signed operations
wire [31:0] abs_a  = (sign_op && op_a[31]) ? (32'd0 - op_a) : op_a;
wire [63:0] dvd    = {op_hi, op_lo};
wire [63:0] abs_d  = (sign_op && op_hi[31]) ? (64'd0 - dvd) : dvd;
wire [31:0] abs_m  = (sign_op && op_lo[31]) ? (32'd0 - op_lo) : op_lo;

// divide: four cascaded restoring steps per cycle (64 bits = 16 rounds),
// each exactly one former one-bit iteration; the divisor is an explicit
// argument so the function stays pure (module-level variables read from
// inside a function are unreliable in continuous assignments under iverilog)
function [96:0] div_step;
	input [96:0] a;
	input [31:0] d;
	reg   [96:0] sh;
	reg   [33:0] t;
	begin
		sh = {a[95:0], 1'b0};
		t  = {1'b0, sh[96:64]} - {2'b00, d};
		if (!t[33]) div_step = {t[32:0], sh[63:1], 1'b1};
		else        div_step = sh;
	end
endfunction

wire [96:0] div4 = div_step(div_step(div_step(div_step(acc, den), den), den), den);

wire [63:0] q_raw = div_r ? acc[63:0] : prod;
wire [31:0] r_raw = acc[95:64];

always @(posedge clk) begin
	if (!nreset) begin
		running <= 0;
		done    <= 0;
		div_r   <= 0;
		neg_q   <= 0;
		neg_r   <= 0;
		count   <= 0;
		den     <= 0;
		mcand   <= 0;
		prod    <= 0;
		acc     <= 0;
		res_hi  <= 0;
		res_lo  <= 0;
		ovf     <= 0;
	end
	else if (kill) begin
		running <= 1'b0;
		done    <= 1'b0;
	end
	else begin
		done <= 0;

		if (start) begin
			div_r   <= is_div;
			running <= 1;
			ovf     <= 0;
			if (is_div) begin
				den   <= abs_a;
				acc   <= {33'd0, abs_d};
				count <= 7'd16;
				neg_q <= sign_op && (op_hi[31] ^ op_a[31]);
				neg_r <= sign_op && op_hi[31];
			end
			else begin
				den   <= abs_a;
				mcand <= abs_m;
				count <= 7'd1;
				neg_q <= sign_op && (op_a[31] ^ op_lo[31]) && (op_a != 0) && (op_lo != 0);
				neg_r <= 0;
			end
		end
		else if (running) begin
			if (count != 0) begin
				count <= count - 7'd1;
				if (div_r) begin
					acc <= div4;
				end
				else begin
					// one registered 32x32 DSP-tree product, exactly the
					// value the former 32-cycle shift-add loop accumulated
					prod <= mcand * den;
				end
			end
			else begin
				running <= 1'b0;
				done    <= 1'b1;
				if (div_r) begin
					res_lo <= neg_q ? (32'd0 - q_raw[31:0]) : q_raw[31:0];
					res_hi <= neg_r ? (32'd0 - r_raw) : r_raw;
					ovf    <= (|q_raw[63:32]) |
					          (sign_op & (neg_q ? (q_raw[31:0] > 32'h8000_0000)
					                            : q_raw[31]));
				end
				else begin
					res_lo <= neg_q ? (32'd0 - q_raw[31:0]) : q_raw[31:0];
					res_hi <= neg_q ? (~q_raw[63:32] + {31'd0, (q_raw[31:0] == 32'd0)})
					                : q_raw[63:32];
					ovf    <= 0;
				end
			end
		end
	end
end

assign busy = running;

endmodule
