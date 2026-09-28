//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 57: PC-relative EAs)      //
//                                                                          //
// tb_ap040_pipe_pcrel.v - (d16,PC) and (d8,PC,Xn)                          //
//                                                                          //
// Amiga code is position-independent throughout, and LEA msg(pc),A0 is its //
// signature idiom.                                                         //
//                                                                          //
// Both modes gather exactly one extension word, and it is the SAME word    //
// the register-based modes gather -- a displacement for 010, a brief       //
// format for 011 -- so they ride the same gather kinds with one more       //
// carried property. All that differs is the BASE: the program counter of   //
// the EXTENSION WORD, which is the opcode's PC plus two, rather than An.   //
// eac_pc was already threaded for the exception frames, so nothing new     //
// reaches EA-fetch; (d8,PC,Xn) then comes along for free on top of         //
// milestone 56's index arithmetic.                                         //
//                                                                          //
//   MOVE.L #5,D1                                                           //
//   LEA    (d16,PC),A1      A1 = $0480                                     //
//   MOVE.L (d16,PC),D0      D0 = [$0480] = 11112222                        //
//   ADD.L  (d16,PC),D1      D1 = 5 + [$0484] = 0000000C                    //
//   LEA    (d16,PC),A2      A2 = $0400, a NEGATIVE displacement            //
//   LEA    (0,PC,D1.L),A3   A3 = $041A + $0C = $0426                       //
//                                                                          //
// The base being PC+2 and not the opcode's own PC is the thing most easily //
// got wrong, and every check here is two bytes away from the wrong answer: //
// A1 would be $047E, A2 $03FE, and the two loads would straddle their      //
// longwords. The displacements are deliberately all different -- 0076,     //
// 0072, 0072, FFEA -- because each instruction sits at a different PC, so  //
// a single shared constant would not have addressed the same place twice.  //
// That is also why this cannot be checked with one instruction repeated.   //
//                                                                          //
// A2's displacement is negative, reaching BACKWARD past the start of the   //
// program to $0400.                                                        //
//                                                                          //
// A3 uses D1 -- a value computed two instructions earlier by the           //
// PC-relative ADD -- as an index, so it checks the PC base and milestone   //
// 56's index path compose, and that port C still forwards when the base    //
// is not a register at all.                                                //
//                                                                          //
// On milestone 56's RTL none of the five decode.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_pcrel;

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
	dut.u_l1.mem[1]  = 16'h223C;   // MOVE.L #$00000005,D1
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0005;
	dut.u_l1.mem[4]  = 16'h43FA;   // LEA (d16,PC),A1     base $040A -> $0480
	dut.u_l1.mem[5]  = 16'h0076;
	dut.u_l1.mem[6]  = 16'h203A;   // MOVE.L (d16,PC),D0  base $040E -> $0480
	dut.u_l1.mem[7]  = 16'h0072;
	dut.u_l1.mem[8]  = 16'hD2BA;   // ADD.L (d16,PC),D1   base $0412 -> $0484
	dut.u_l1.mem[9]  = 16'h0072;
	dut.u_l1.mem[10] = 16'h45FA;   // LEA (d16,PC),A2     base $0416 -> $0400
	dut.u_l1.mem[11] = 16'hFFEA;
	dut.u_l1.mem[12] = 16'h47FB;   // LEA (0,PC,D1.L),A3  base $041A -> $0426
	dut.u_l1.mem[13] = 16'h1800;

	dut.u_l1.mem[64] = 16'h1111;   // $0480 = 11112222
	dut.u_l1.mem[65] = 16'h2222;
	dut.u_l1.mem[66] = 16'h0000;   // $0484 = 00000007
	dut.u_l1.mem[67] = 16'h0007;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000480 (LEA (d16,PC); 0000047e means the base was the opcode PC, not PC+2)",
		         dut.u_cpu.u_regfile.areg[1]);
	end
	if (dbg_d0 !== 32'h1111_2222) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11112222 (MOVE.L (d16,PC),D0)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_000C) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000000c (5 + [$0484]; the ALU family with a PC-relative source)", dbg_d1);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_0400) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00000400 (a NEGATIVE PC displacement, reaching back past the program)",
		         dut.u_cpu.u_regfile.areg[2]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_0426) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 00000426 (PC base composed with milestone 56's index path)",
		         dut.u_cpu.u_regfile.areg[3]);
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
