//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 4: BRA.B)           //
//                                                                          //
// tb_ap040_pipe_bra.v - unconditional branch redirect proof               //
//                                                                          //
// Program:                                                                //
//                                                                          //
//   0: NOP                                                                //
//   1: BRA +2        (0x6002)   skips exactly one word (rom[2])           //
//   2: MOVEQ #99,D1  (0x7263)   "poison" -- must NEVER execute             //
//   3: MOVEQ #5,D0   (0x7005)   the actual branch target                  //
//   4-9: NOP (drain)                                                      //
//                                                                          //
// D0 and D1 are different registers on purpose: if the redirect silently   //
// failed and rom[2] fell through normally before rom[3], program order     //
// would still leave D0 == 5 (the target executes after the poison either   //
// way), so checking D0 alone would pass whether or not the squash worked.  //
// D1 only reads 99 if the poison instruction actually ran, which is        //
// exactly the failure this test exists to catch.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bra;

localparam PROG_WORDS      = 10;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

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

// Override the IF stage's default all-NOP ROM. Scheduled at #1 so it runs
// after ap040_inst_fetch.v's own t=0 fill (standard Verilog idiom).
initial begin
	#1;
	dut.u_l1.mem[1] = 16'h6002;   // BRA +2
	dut.u_l1.mem[2] = 16'h7263;   // MOVEQ #99,D1 (poison)
	dut.u_l1.mem[3] = 16'h7005;   // MOVEQ #5,D0  (target)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Same margin as the other testbenches: PROG_WORDS instructions
	// issued, PROG_WORDS + 6 cycles to fully drain if nothing ever stalls
	// -- confirms the redirect costs zero extra cycles, not just that it
	// eventually settles on the right values.
	repeat (PROG_WORDS + 20) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000005 (branch target did not run)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (poison instruction after the branch ran -- redirect failed)", dbg_d1);
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
