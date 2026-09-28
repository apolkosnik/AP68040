//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: a faulting     //
// (A7)+ access)                                                            //
//                                                                          //
// tb_ap040_pipe_faultpi.v - two writes to A7, two different stacks         //
//                                                                          //
// One instruction can write A7 twice. An autoincrement operand updates the //
// address register through the second write port, and the instruction's    //
// own result goes through the first. Normally both mean the same A7, so    //
// one bank select serves them.                                             //
//                                                                          //
// An exception breaks that. DIVU.W (A7)+,D0 in USER mode with a zero       //
// divisor has to do both of these, and they are on DIFFERENT stacks:       //
//                                                                          //
//   - the postincrement belongs to the instruction, which is a user-mode   //
//     instruction, so it updates USP                                       //
//   - the exception frame goes on the SUPERVISOR stack and its new stack   //
//     pointer is the instruction's result, so that updates ISP             //
//                                                                          //
// Exception entry sets S on the way through, so a select taken from the    //
// SR at commit names the supervisor stack for both, and the postincrement  //
// lands on ISP -- on top of the frame pointer that was just written there. //
// USP is left where it started and ISP holds a user-stack address.         //
//                                                                          //
//   USP = $800, ISP = $1000, the word at $800 is zero                      //
//   MOVE D1,SR with D1 = 0      drop to user mode                          //
//   DIVU.W (A7)+,D0             divisor 0 -> vector 5, format $2 frame     //
//                                                                          //
// USP must be $802 and ISP must be $FF4: the frame is twelve bytes, and    //
// the Word operand stepped A7 by two. The failing values are $800 and      //
// $802 -- the postincrement's result, on the wrong stack, and no           //
// postincrement at all on the right one.                                   //
//                                                                          //
// This is the ordinary shape of a fault on a stack operand, not a contrived//
// one: any user-mode instruction that faults while autoincrementing        //
// through A7 does it.                                                      //
//                                                                          //
// The handler's marker and the poison after the faulting instruction are   //
// both checked, so a run where the exception simply did not happen cannot  //
// pass on the register values alone.                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_faultpi;

localparam PROG_WORDS      = 40;
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
wire [31:0] dbg_d2, dbg_d3;
wire [15:0] dbg_sr;
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

	.dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_sr(dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000800,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0800;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,USP
	dut.u_l1.mem[ 5] = 16'h0800;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h1000;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D0,ISP
	dut.u_l1.mem[10] = 16'h0804;
	dut.u_l1.mem[11] = 16'h203C;   // MOVE.L #$00000064,D0  (the dividend)
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0064;
	dut.u_l1.mem[14] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[15] = 16'h46C1;   // MOVE D1,SR   (S -> 0)
	dut.u_l1.mem[16] = 16'h80DF;   // DIVU.W (A7)+,D0   divisor at $800 = 0
	dut.u_l1.mem[17] = 16'h7466;   // MOVEQ #$66,D2 (poison: must not run)
	dut.u_l1.mem[18] = 16'h4E71;   // NOP

	// Divide-by-zero handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[385] = 16'h4E71;  // NOP

	// The user stack's first word is the divisor: zero.
	dut.u_l1.mem[512] = 16'h0000;  // $0800
	dut.u_l1.mem[513] = 16'h0000;

	// Vector 5 (divide by zero) -> $700.
	dut.u_l1.mem[3594] = 16'h0000;
	dut.u_l1.mem[3595] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the divide-by-zero handler must run)", dbg_d3);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000 (the instruction after the faulting one must not run)", dbg_d2);
	end
	if (dut.u_cpu.u_regfile.usp !== 32'h0000_0802) begin
		errors = errors + 1;
		$display("FAIL: USP = %h, expected 00000802 (the postincrement belongs to a USER-mode instruction and must land on USP; 00000800 means it did not land there at all)",
		         dut.u_cpu.u_regfile.usp);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0FF4) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00000ff4 (the format $2 frame is twelve bytes below $1000; 00000802 means the postincrement was banked as supervisor and overwrote it)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dbg_sr[13] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: SR.S = %b, expected 1 (the handler runs in supervisor mode)", dbg_sr[13]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
