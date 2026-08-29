//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 3: ADD Dn,Dm and    //
// dual-operand forwarding)                                                 //
//                                                                          //
// tb_ap040_pipe_add.v - dual-operand register forwarding proof            //
//                                                                          //
// Program:                                                                //
//                                                                          //
//   0: NOP                                                                //
//   1: MOVEQ #3,D0   (0x7003)                                             //
//   2: MOVEQ #4,D1   (0x7204)                                             //
//   3: ADD.L D0,D1   (0xD280)   D1 = D1 + D0                              //
//   4-9: NOP (drain)                                                      //
//                                                                          //
// At the cycle ADD reaches EA-fetch: the D0 producer (MOVEQ #3,D0, issued  //
// 2 instructions earlier) is committing in WB that same cycle -- port A    //
// exercises the regfile's own WRITE_THROUGH bypass again, a repeat         //
// confidence check from milestone 2. The D1 producer (MOVEQ #4,D1, issued  //
// 1 instruction earlier) is still combinational in EX that same cycle --   //
// port B *must* go through the new EX-forward compare added this          //
// milestone, or the test reads stale D1 (0, not 4) and the sum comes out   //
// wrong. D1 == 7 only if both forwarding paths -- one old, one new -- are  //
// both actually working.                                                  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_add;

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
	dut.u_l1.mem[1] = 16'h7003;   // MOVEQ #3,D0
	dut.u_l1.mem[2] = 16'h7204;   // MOVEQ #4,D1
	dut.u_l1.mem[3] = 16'hD280;   // ADD.L D0,D1
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Same margin as the other two testbenches: PROG_WORDS instructions
	// issued, PROG_WORDS + 6 cycles to fully drain if nothing ever stalls.
	repeat (PROG_WORDS + 20) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0003) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000003", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000007 (port-B EX-forward failed)", dbg_d1);
	end
	// XNZVC: ADD.L D0,D1 was the last flag-writing instruction, result
	// small and positive -- N=0, Z=0, V=0, C=0; X don't-care here.
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000", dbg_ccr[3:0]);
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
