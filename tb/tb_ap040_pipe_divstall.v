//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 69: commit during a stall) //
//                                                                          //
// tb_ap040_pipe_divstall.v - what retires while the divider is running     //
//                                                                          //
// The divider holds EX for over thirty cycles. EX's own output registers    //
// hold correctly while it does -- milestone 52 moved their gate from        //
// stall_in to ex_stall for exactly that -- but WB does not know EX is       //
// stalled, and commits whatever EX's output registers hold on EVERY one of  //
// those cycles.                                                            //
//                                                                          //
// tb_ap040_pipe_div.v never saw it, because every instruction that precedes //
// a divide there is a MOVE, and committing a MOVE thirty times leaves the   //
// same value as committing it once. tb_ap040_pipe_integration2.v showed     //
// the same MOVE retiring five times in its trace, which is where this came  //
// from; the harm needed an instruction that is NOT idempotent.              //
//                                                                          //
//   MOVE.L #100,D0 / MOVE.L #7,D2                                          //
//   MOVEQ  #0,D1                                                           //
//   ADDQ.L #1,D1        <- in WB while the divide stalls EX                //
//   DIVU.W D2,D0        D0 = 0002000E                                      //
//                                                                          //
// D1 = 1 alone does NOT catch it, and the first version of this bench     //
// assumed it would. WB commits exe_result_data, a value REGISTERED in EX:   //
// the ADDQ is not re-executed on each spurious commit, its computed result  //
// 1 is simply written again, and the register cannot tell once from thirty. //
// Every commit path has that property today, which is why the re-commit    //
// was harmless -- and why no value check anywhere could have seen it.       //
//                                                                          //
// So the check is a COUNT. dbg_commits is the number of register commits    //
// since reset; this program has exactly five instructions that write a      //
// register (two MOVE.L, MOVEQ, ADDQ, DIVU). Anything above five is a        //
// commit that happened while EX was stalled.                                //
//                                                                          //
// The divide itself is checked too, so a fix that "solves" the double       //
// commit by breaking the stall is caught.                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_divstall;

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
wire [31:0] dbg_commits;

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
	.dbg_ccr(dbg_ccr), .dbg_commits(dbg_commits)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$00000064,D0   (100)
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0064;
	dut.u_l1.mem[4]  = 16'h243C;   // MOVE.L #$00000007,D2
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0007;
	dut.u_l1.mem[7]  = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[8]  = 16'h5281;   // ADDQ.L #1,D1  -- NOT idempotent
	dut.u_l1.mem[9]  = 16'h80C2;   // DIVU.W D2,D0
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 320) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d1 !== 32'h0000_0001) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000001 (ADDQ retired %0d times; WB re-committed it while the divider stalled EX)",
		         dbg_d1, dbg_d1);
	end
	if (dbg_commits !== 32'd5) begin
		errors = errors + 1;
		$display("FAIL: %0d register commits, expected 5 (the surplus were re-commits during the divider's stall)",
		         dbg_commits);
	end
	if (dbg_d0 !== 32'h0002_000E) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0002000e (100/7 -- the divide must still complete)", dbg_d0);
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
