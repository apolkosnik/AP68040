//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 59: absolute EAs)         //
//                                                                          //
// tb_ap040_pipe_absalu.v - the ALU family and the unary ops on $xxx        //
//                                                                          //
// Mode 111 reg 000 and 001 are the only modes left whose extension words   //
// are the ADDRESS itself rather than an offset from something.             //
//                                                                          //
// They ride held_is_abs, which already gathers one word or two and already //
// sets id_is_abs so ea_target is simply eac_imm. What it did not carry is  //
// an OPERATION: every absolute form until this milestone was a MOVE.       //
// held_alu_op and held_alu_nowrite already existed for the displacement    //
// kinds, so this reuses them rather than adding more.                      //
//                                                                          //
// The absolute READ-MODIFY-WRITE composes without new datapath, which is   //
// the part worth noting: id_is_abs makes ea_target eac_imm, and milestone  //
// 48's store half writes to eaf_ea_target, which is that same value.       //
// Nothing had to learn that an absolute address could also be a            //
// destination.                                                             //
//                                                                          //
// Memory: $0480 = 00000007, $0484 = 11112222,                              //
//         $0488 = FFFF1234, $048C = 80000000.                              //
//                                                                          //
//   MOVE.L #5,D0                                                           //
//   ADD.L  $0480.W,D0   D0 = 0000000C   -- the SHORT absolute form         //
//   CLR.L  $0484.L      $0484 -> 00000000                                  //
//   NOT.W  $0488.L      $0488 -> 00001234                                  //
//   TST.L  $048C.L      N set, memory untouched                            //
//                                                                          //
// D0 is the check that the OPERATION is carried and not defaulted. Every   //
// absolute form before this one was a MOVE, so a decoder that left         //
// id_alu_op alone would move 7 into D0 instead of adding it -- a plausible //
// value, which is why the addend and the operand differ.                   //
//                                                                          //
// The short form is used once deliberately: its single extension word is   //
// SIGN-EXTENDED to 32 bits, a different gather length from the long form   //
// used by the other three, so both widths are exercised.                   //
//                                                                          //
// NOT.W is sized so the low word has to survive: a Long NOT would leave    //
// 0000EDCB, and a clear would leave 00000000, neither of which is          //
// 00001234.                                                                //
//                                                                          //
// TST is checked by its flag and by memory being untouched, for the reason //
// tb_ap040_pipe_unarymem.v documents -- its result is the value it read.   //
//                                                                          //
// The two MOVEQs at the top are the same kind of poison milestone 58       //
// needed, and were added for the same reason: a mutation proved the bench  //
// could not otherwise see the bug. A unary form names no source register,  //
// so decode leaves the destination field pointing at whatever ir[11:9]     //
// happens to hold -- D1 for this CLR, D3 for this NOT. With both registers //
// zero, an operation that defaulted to MOVE stored ZERO, which is exactly  //
// what CLR should store and exactly what NOT.W of FFFF should leave. Both  //
// checks passed against a decoder that had lost the operation entirely.    //
// With D1 and D3 holding $7F the same mutation writes 0000007F and         //
// 007F1234 instead.                                                        //
//                                                                          //
// That is the second time this pattern has bitten, so it is worth stating  //
// as a rule rather than a note: an instruction whose unused register       //
// fields read as zero will hide a defaulted operation whenever zero is     //
// also the right answer.                                                   //
//                                                                          //
// On milestone 58's RTL none of the four decode.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_absalu;

localparam PROG_WORDS      = 32;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
// A pseudo-random clock enable (milestone 94). Every bench in this suite
// tied ce high, and eight of the thirteen defects three rounds of external
// review found lived behind that: a cycle with ce low is a cycle that did
// not happen, and the core has to treat it that way. Driven on the falling
// edge so it is stable across every rising one, and left high until reset
// releases so the reset sequence itself is unchanged.
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`endif

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1]  = 16'h727F;   // MOVEQ #$7F,D1  -- poison; see header
	dut.u_l1.mem[2]  = 16'h767F;   // MOVEQ #$7F,D3  -- poison
	dut.u_l1.mem[3]  = 16'h203C;   // MOVE.L #$00000005,D0
	dut.u_l1.mem[4]  = 16'h0000;
	dut.u_l1.mem[5]  = 16'h0005;
	dut.u_l1.mem[6]  = 16'hD0B8;   // ADD.L $0480.W,D0   (short absolute)
	dut.u_l1.mem[7]  = 16'h0480;
	dut.u_l1.mem[8]  = 16'h42B9;   // CLR.L $00000484.L   (dest field names D1)
	dut.u_l1.mem[9]  = 16'h0000;
	dut.u_l1.mem[10] = 16'h0484;
	dut.u_l1.mem[11] = 16'h4679;   // NOT.W $00000488.L   (dest field names D3)
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0488;
	dut.u_l1.mem[14] = 16'h4AB9;   // TST.L $0000048C.L
	dut.u_l1.mem[15] = 16'h0000;
	dut.u_l1.mem[16] = 16'h048C;

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000007
	dut.u_l1.mem[65] = 16'h0007;
	dut.u_l1.mem[66] = 16'h1111;   // $0484 = 11112222
	dut.u_l1.mem[67] = 16'h2222;
	dut.u_l1.mem[68] = 16'hFFFF;   // $0488 = FFFF1234
	dut.u_l1.mem[69] = 16'h1234;
	dut.u_l1.mem[70] = 16'h8000;   // $048C = 80000000
	dut.u_l1.mem[71] = 16'h0000;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_000C) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0000000c (ADD.L $0480.W,D0; 00000007 means the operation defaulted to MOVE)",
		         dbg_d0);
	end
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: $0484 = %h%h, expected 00000000 (CLR.L to an absolute address)",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'h0000_1234) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected 00001234 (NOT.W must leave the low word; Long would give 0000edcb)",
		         dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if ({dut.u_l1.mem[70], dut.u_l1.mem[71]} !== 32'h8000_0000) begin
		errors = errors + 1;
		$display("FAIL: $048C = %h%h, expected 80000000 (TST must leave memory alone)",
		         dut.u_l1.mem[70], dut.u_l1.mem[71]);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. TST.L of 80000000 sets N.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (TST.L of a negative longword)", dbg_ccr[3:0]);
	end

	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid ||
	    dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
		errors = errors + 1;
		$display("FAIL: a stage is still valid after the program should have drained");
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
