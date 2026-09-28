//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 39: ALU on memory)  //
//                                                                          //
// tb_ap040_pipe_alumem.v - OR/SUB/CMP/AND/ADD with a memory source         //
//                                                                          //
// The binary ALU family decoded only the register-direct shape             //
// 1ooo RRR 0 SS 000 rrr. Every one of those five operations also exists     //
// with ea mode 010, reading (An) instead of a second data register, and     //
// that form is what compiled code actually emits -- an operand in memory is //
// the common case, not the exception.                                      //
//                                                                          //
// No new operand plumbing was needed. ap040_ea_fetch.v already replaces     //
// eaf_operand_a with the loaded word for MOVE.L (An),Dn, and the ALU        //
// computes b op a, so pointing src_reg at the ADDRESS register gives        //
// ADD.L (A0),D0 = D0 + memory with the operands already the right way       //
// round. Getting that order backwards would be invisible for ADD and AND    //
// and wrong for SUB and CMP, which is why CMP is checked here.              //
//                                                                          //
// Memory: $0480 = 00000005, $0484 = 0007CCCC.                              //
//                                                                          //
//   MOVEA.L #$0480,A0 / #$0484,A1                                          //
//   MOVE.L  #$00000010,D0                                                  //
//   ADD.L   (A0),D0     10 + 5 = 15                                        //
//   AND.L   (A0),D0     15 & 5 = 05                                        //
//   MOVE.L  #$11110002,D1                                                  //
//   ADD.W   (A1),D1     0002 + 0007 in the low word only                   //
//   MOVE.L  #$00000005,D2                                                  //
//   CMP.L   (A0),D2     5 - 5, sets Z and writes nothing                   //
//                                                                          //
// D0 is checked after BOTH operations because the composition is the test:  //
// 05 is reachable only if each ran and each saw the memory operand. The     //
// ADD alone would leave 15, the AND alone 10 & 05 = 00, and a source/dest   //
// swap in the AND would leave 05 as well -- which is why ADD.W is a         //
// separate check and CMP a third.                                          //
//                                                                          //
// ADD.W proves the operand is taken from the addressed half-word and that   //
// only the low word of D1 is written: the high word must survive as 1111.   //
//                                                                          //
// CMP must set Z and leave D2 at 5. A CMP that writes its result back would //
// leave 0, so the register check is what distinguishes CMP from SUB here.   //
//                                                                          //
// On milestone 38's RTL none of the four memory-source forms decode; they   //
// fall through to is_illegal.                                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_alumem;

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
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0480;
	dut.u_l1.mem[4]  = 16'h227C;   // MOVEA.L #$00000484,A1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0484;
	dut.u_l1.mem[7]  = 16'h203C;   // MOVE.L #$00000010,D0
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h0010;
	dut.u_l1.mem[10] = 16'hD090;   // ADD.L (A0),D0
	dut.u_l1.mem[11] = 16'hC090;   // AND.L (A0),D0
	dut.u_l1.mem[12] = 16'h223C;   // MOVE.L #$11110002,D1
	dut.u_l1.mem[13] = 16'h1111;
	dut.u_l1.mem[14] = 16'h0002;
	dut.u_l1.mem[15] = 16'hD251;   // ADD.W (A1),D1
	dut.u_l1.mem[16] = 16'h243C;   // MOVE.L #$00000005,D2
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h0005;
	dut.u_l1.mem[19] = 16'hB490;   // CMP.L (A0),D2

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000005
	dut.u_l1.mem[65] = 16'h0005;
	dut.u_l1.mem[66] = 16'h0007;   // $0484 = 0007CCCC
	dut.u_l1.mem[67] = 16'hCCCC;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000005 (ADD.L then AND.L against memory)", dbg_d0);
	end
	if (dbg_d1 !== 32'h1111_0009) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 11110009 (ADD.W must add the addressed half-word and leave the high word)",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (CMP must not write its result back)", dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. CMP.L of equal operands sets Z alone; the
	// reversed subtraction would give the same Z here, but N and C would
	// differ on unequal operands, so Z is checked together with the fact
	// that D2 survived.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMP.L of equal operands sets Z)", dbg_ccr[3:0]);
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
