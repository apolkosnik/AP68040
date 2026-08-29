//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 5: Bcc.B,           //
// misprediction recovery, CCR forwarding)                                  //
//                                                                          //
// tb_ap040_pipe_bcc.v - conditional branch + CCR forwarding proof         //
//                                                                          //
// Two sub-cases, each with an IMMEDIATELY ADJACENT flag-setter -- the      //
// tightest gap, and the only one that actually exercises CCR forwarding    //
// (a distant setter would pass off the plain committed ccr register with   //
// no forwarding at all) -- and each writing a DISTINCT register, so a      //
// broken forward and a broken recovery produce distinguishable failures    //
// instead of both quietly reaching the "right" value via program order.    //
//                                                                          //
//  0: NOP                                                                 //
//  1: MOVEQ #0,D0    (0x7000)  Z=1                                        //
//  2: BEQ +2 words   (0x6702)  adjacent to #1; condition true --           //
//                              correctly predicted taken                   //
//  3: MOVEQ #99,D1   (0x7263)  poison: must NOT run                       //
//  4: MOVEQ #5,D2    (0x7405)  target: proves case 1                      //
//                                                                          //
//  5: MOVEQ #1,D0    (0x7001)  Z=0                                        //
//  6: BEQ +4 words   (0x6704)  adjacent to #5; condition FALSE --          //
//                              speculatively predicted taken anyway,       //
//                              must be caught and recovered; target = #9   //
//  7: MOVEQ #7,D3    (0x7607)  correct fall-through: must run             //
//  8: BRA +2 words   (0x6002)  jumps over #9 -- the correct path must      //
//                              never reach the poison below just because   //
//                              it's the next word in memory (that was a    //
//                              real bug in this test's first draft: #9     //
//                              sitting directly after #7 meant the correct //
//                              fall-through path reached it too, making it //
//                              useless as a distinguishing check)          //
//  9: MOVEQ #88,D3   (0x7658)  branch's (wrong) taken target: must NOT run //
// 10-15: NOP (drain)                                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bcc;

localparam PROG_WORDS      = 16;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3;
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

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

// Override the IF stage's default all-NOP ROM. Scheduled at #1 so it runs
// after ap040_inst_fetch.v's own t=0 fill (standard Verilog idiom).
//
// This test needs a fourth distinct register to keep case 1 and case 2
// unambiguous (see the file header), so ap040_regfile.v gained a dbg_d3
// tap this milestone -- dreg[3] was already tracked internally, only the
// output port was missing, the same kind of small additive change as the
// WRITE_THROUGH parameter in milestone 2. Every existing instantiation of
// ap040_regfile.v is unaffected: Verilog allows leaving a new output port
// unconnected, so ap040_core.v and every ap040/ testbench needs no change.
initial begin
	#1;
	dut.u_l1.mem[1]  = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[2]  = 16'h6702;   // BEQ +2 words (correctly taken)
	dut.u_l1.mem[3]  = 16'h7263;   // MOVEQ #99,D1 (poison, must not run)
	dut.u_l1.mem[4]  = 16'h7405;   // MOVEQ #5,D2  (target, must run)

	dut.u_l1.mem[5]  = 16'h7001;   // MOVEQ #1,D0
	dut.u_l1.mem[6]  = 16'h6704;   // BEQ +4 words (mispredicted -- not taken), target = rom[9]
	dut.u_l1.mem[7]  = 16'h7607;   // MOVEQ #7,D3  (fall-through, must run)
	dut.u_l1.mem[8]  = 16'h6002;   // BRA +2 words, jumps over rom[9]
	dut.u_l1.mem[9]  = 16'h7658;   // MOVEQ #88,D3 (wrong taken target, must not run)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// More margin than the other testbenches: this program is 9
	// instructions plus drain, and a misprediction costs recovery cycles.
	repeat (PROG_WORDS + 20) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0001) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000001", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (case 1 poison ran -- CCR forward broken, BEQ wrongly not-taken)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case 1 target did not run)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000007 (case 2 misprediction recovery broken -- wrong taken-path poison ran, or correct fall-through did not)", dbg_d3);
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
