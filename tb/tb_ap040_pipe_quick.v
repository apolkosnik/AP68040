//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 27: ADDQ and SUBQ)  //
//                                                                          //
// tb_ap040_pipe_quick.v - an immediate that never leaves the opcode        //
//                                                                          //
// 0101 qqq d SS 000 rrr. Unlike the ORI family of milestone 26 the operand //
// is IN the opcode word, so these need no gather: they reuse the direct     //
// src_a_is_imm path MOVEQ has used since milestone 2 and cost a predicate  //
// and nothing else. ir[8] picks SUBQ over ADDQ.                            //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #1,D0      7001   D0 = 00000001                               //
//   2: ADDQ.L #3,D0     5680   D0 = 00000004                               //
//   3: MOVEQ #1,D1      7201   D1 = 00000001                               //
//   4: SUBQ.L #8,D1     5181   D1 = FFFFFFF9   quick field 0 means EIGHT   //
//   5: MOVEQ #-1,D2     74FF   D2 = FFFFFFFF                               //
//   6: ADDQ.W #1,D2     5242   D2 = FFFF0000   word wraps, upper kept      //
//                                                                          //
// SUBQ.L #8 is the encoding check: the quick field holds 0 for eight, the   //
// same convention the shift count uses, so reading it literally subtracts   //
// nothing and leaves D1 at 00000001. It is also the check that ir[8]        //
// selects SUBQ -- decoded as ADDQ the answer is 00000009, not FFFFFFF9.     //
//                                                                          //
// ADDQ.W #1 on all-ones is the size check from the awkward direction: the   //
// word half wraps to 0000 while D2[31:16] must survive as FFFF, so a        //
// Long-sized decode gives 00000000 and loses the distinction between "size  //
// wrong" and "instruction skipped" only if the upper half is ignored --     //
// which is why the full 32-bit value is compared.                          //
//                                                                          //
// On milestone 26's RTL none of the three decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_quick;

localparam PROG_WORDS      = 14;
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
	dut.u_l1.mem[1] = 16'h7001;   // MOVEQ #1,D0
	dut.u_l1.mem[2] = 16'h5680;   // ADDQ.L #3,D0
	dut.u_l1.mem[3] = 16'h7201;   // MOVEQ #1,D1
	dut.u_l1.mem[4] = 16'h5181;   // SUBQ.L #8,D1
	dut.u_l1.mem[5] = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[6] = 16'h5242;   // ADDQ.W #1,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0004) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000004 (ADDQ.L #3)", dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_FFF9) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected FFFFFFF9 (SUBQ.L #8: quick 0 means 8, and ir[8] must select SUB)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFF0000 (ADDQ.W must wrap the word and keep D2[31:16])", dbg_d2);
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
