//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 25: ABCD, SBCD,     //
// NBCD and TAS -- the last of the ALU's reachable set)                     //
//                                                                          //
// tb_ap040_pipe_bcd.v - decimal adjust, and the two excluded slots         //
//                                                                          //
// NBCD and TAS are the slots the extswap and unary predicates deliberately //
// left out in milestones 20 and 21, named there as exclusions. ABCD/SBCD   //
// are the ir[7:6]=00 members of the ir[8]=1 slot ADDX/SUBX occupy. With    //
// these four, every operation ap040_pipe_alu.v implements in its register  //
// forms is reachable from decode.                                          //
//                                                                          //
// All four are Byte, and that is load-bearing rather than incidental: the  //
// ALU returns {24'd0, result[7:0]} for the BCD trio and {1'b1, b[6:0]} for //
// TAS, so anything wider would let execute's merge overwrite Dn[31:8]      //
// instead of preserving it.                                                //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #$19,D0    7019   D0 = 00000019   BCD nineteen                //
//   2: MOVEQ #1,D1      7201   D1 = 00000001                               //
//   3: ABCD   D1,D0     C101   D0 = 00000020   decimal adjust, NOT 1A      //
//   4: MOVEQ #-1,D2     74FF   D2 = FFFFFFFF                               //
//   5: TAS    D2        4AC2   D2 = FFFFFFFF   bit 7 already set           //
//   6: MOVEQ #0,D1      7200   D1 = 00000000                               //
//   7: TAS    D1        4AC1   D1 = 00000080   bit 7 set, rest untouched   //
//                                                                          //
// ABCD is the check that separates a decimal adjust from a plain add: 19   //
// plus 1 is 1A in binary and 20 in BCD, so a decoder that reached          //
// AP040_ALU_ADD instead would leave 0000001A.                              //
//                                                                          //
// The two TAS cases pin the byte width from both directions. D2 starts     //
// all-ones, so a Long-sized TAS would return {1'b1, b[6:0]} as 000000FF    //
// and destroy D2[31:8]; Byte-sized it splices FF back over FF and D2 is    //
// unchanged, which is the correct answer and an incorrect one would be     //
// loud. D1 starts at zero and must end at exactly 80 -- proof the set      //
// happened at all, which the D2 case alone cannot give.                    //
//                                                                          //
// On milestone 24's RTL none of the three decodes.                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bcd;

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
	dut.u_l1.mem[1] = 16'h7019;   // MOVEQ #$19,D0
	dut.u_l1.mem[2] = 16'h7201;   // MOVEQ #1,D1
	dut.u_l1.mem[3] = 16'hC101;   // ABCD D1,D0
	dut.u_l1.mem[4] = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[5] = 16'h4AC2;   // TAS  D2
	dut.u_l1.mem[6] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[7] = 16'h4AC1;   // TAS  D1
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0020) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000020 (ABCD decimal-adjusts; 0000001A means a plain ADD)", dbg_d0);
	end
	if (dbg_d2 !== 32'hFFFF_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFFFFFF (TAS is Byte; a wider one destroys D2[31:8])", dbg_d2);
	end
	if (dbg_d1 !== 32'h0000_0080) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000080 (TAS must set bit 7)", dbg_d1);
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
