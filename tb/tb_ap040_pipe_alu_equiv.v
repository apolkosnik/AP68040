//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 75: the ROX count    //
// reduction without a divider)                                              //
//                                                                          //
// tb_ap040_pipe_alu_equiv.v - ap040_pipe_alu.v against rtl/ap040/ap040_alu.v //
//                                                                          //
// Until this milestone the two ALUs were one file: ap040_pipe_alu.v is      //
// rtl/ap040/ap040_alu.v renamed, plus MULU/MULS. The FSM core's copy passes //
// 3,797/3,801 cputest slices, so it is the reference here, and the pipe     //
// copy must agree with it bit for bit -- result and all five flags -- on    //
// every operation the two share. MULU/MULS exist only in the pipe copy and  //
// are not compared.                                                        //
//                                                                          //
// Part A is exhaustive on the dimension milestone 75 changes: all 8 shift/  //
// rotate operations x 3 sizes x every count 1..63 x both X-in values x 16   //
// operand patterns. Count 0 is outside the reference's contract -- its      //
// header says 1..63, and rtl/ap040/ap040_core.v never calls it with 0 --    //
// and since milestone 87 the pipe copy defines it (a register count can be  //
// 0), so at 0 the two copies differ by design. That case is checked by      //
// tb_ap040_pipe_shiftreg.v and, against the FSM CORE rather than its ALU,   //
// by tb_ap040_pipe_dual.v.                                                  //
// The ROX rotates take their count mod (size+1), and the                    //
// reduction is now three constant compares instead of a `%`. A wrong        //
// threshold or an off-by-one shows up only at specific counts, so every     //
// count is tried rather than sampled.                                       //
//                                                                          //
// Part B is a random differential over every shared operation, all sizes,  //
// 512 vectors each, from a fixed-seed xorshift so a failure reproduces.     //
//                                                                          //
// The first 20 mismatches are printed with the inputs that caused them; the //
// summary counts all of them.                                               //
//--------------------------------------------------------------------------//
`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_alu_equiv;
	reg  [5:0]  op;
	reg  [1:0]  size;
	reg  [5:0]  shcnt;
	reg  [31:0] a, b;
	reg  [4:0]  flags_in;
	wire [31:0] r_ref, r_dut;
	wire [4:0]  f_ref, f_dut;

	ap040_alu u_ref (
		.op(op), .size(size), .shcnt(shcnt), .a(a), .b(b),
		.flags_in(flags_in), .result(r_ref), .flags_out(f_ref));

	ap040_pipe_alu u_dut (
		.op(op), .size(size), .shcnt(shcnt), .a(a), .b(b),
		.flags_in(flags_in), .result(r_dut), .flags_out(f_dut));

	integer errors = 0, checks = 0, shown = 0;
	integer o, sz, cnt, x, pat, v;
	reg [31:0] rnd;

	function [31:0] xorshift32;
		input [31:0] s;
		reg   [31:0] t;
		begin
			t = s;
			t = t ^ (t << 13);
			t = t ^ (t >> 17);
			t = t ^ (t << 5);
			xorshift32 = t;
		end
	endfunction

	function [31:0] pattern;
		input integer i;
		begin
			case (i)
				0:  pattern = 32'h0000_0000;
				1:  pattern = 32'hFFFF_FFFF;
				2:  pattern = 32'h0000_0001;
				3:  pattern = 32'h8000_0000;
				4:  pattern = 32'h0000_8000;
				5:  pattern = 32'h0000_0080;
				6:  pattern = 32'hAAAA_AAAA;
				7:  pattern = 32'h5555_5555;
				8:  pattern = 32'h1234_5678;
				9:  pattern = 32'h8000_FFFF;
				10: pattern = 32'h0000_FF00;
				11: pattern = 32'h7FFF_FFFF;
				12: pattern = 32'h0001_0000;
				13: pattern = 32'hFEDC_BA98;
				14: pattern = 32'h0000_0100;
				default: pattern = 32'h0000_FF80;
			endcase
		end
	endfunction

	task compare;
		input string tag;
		begin
			#1;
			checks = checks + 1;
			if (r_ref !== r_dut || f_ref !== f_dut) begin
				errors = errors + 1;
				if (shown < 20) begin
					shown = shown + 1;
					$display("FAIL: %0s op=%0d size=%0d cnt=%0d a=%h b=%h fin=%b: ref %h/%b dut %h/%b",
					         tag, op, size, shcnt, a, b, flags_in,
					         r_ref, f_ref, r_dut, f_dut);
				end
			end
		end
	endtask

	initial begin
		rnd = 32'h2545_F491;
		op = 0; size = 0; shcnt = 0; a = 0; b = 0; flags_in = 0;

		// Part A: every shift/rotate at every count
		for (o = {26'd0, `AP040_ALU_ASL1}; o <= {26'd0, `AP040_ALU_ROXR1}; o = o + 1)
			for (sz = 0; sz < 3; sz = sz + 1)
				for (cnt = 1; cnt < 64; cnt = cnt + 1)   // 0 is outside the reference's range
					for (x = 0; x < 2; x = x + 1)
						for (pat = 0; pat < 16; pat = pat + 1) begin
							rnd = xorshift32(rnd);
							op = o[5:0]; size = sz[1:0]; shcnt = cnt[5:0];
							b = pattern(pat); a = rnd;
							flags_in = {x[0], rnd[3:0]};
							compare("shift/rotate");
						end

		// Part B: every shared operation, random vectors
		for (o = {26'd0, `AP040_ALU_MOVE}; o <= {26'd0, `AP040_ALU_BSET}; o = o + 1)
			for (sz = 0; sz < 4; sz = sz + 1)
				for (v = 0; v < 512; v = v + 1) begin
					rnd = xorshift32(rnd); a = rnd;
					rnd = xorshift32(rnd); b = rnd;
					rnd = xorshift32(rnd);
					op = o[5:0]; size = sz[1:0];
					shcnt = (rnd[5:0] == 6'd0) ? 6'd1 : rnd[5:0]; flags_in = rnd[10:6];
					compare("random");
				end

		if (errors == 0) $display("ALL TESTS PASSED (%0d checks)", checks);
		else $display("%0d CHECK(S) FAILED of %0d", errors, checks);
		$finish;
	end
endmodule
