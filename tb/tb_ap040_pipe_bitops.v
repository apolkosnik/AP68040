//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 24: the bit group)  //
//                                                                          //
// tb_ap040_pipe_bitops.v - operands arranged the other way round           //
//                                                                          //
// BTST/BCHG/BCLR/BSET with a dynamic bit number are the first instructions //
// here whose operands run OPPOSITE to the binary family. The bit number is //
// ir[11:9] and the target is ir[2:0]; ap040_pipe_alu.v builds bit_mask     //
// from operand a and tests operand b, so src_reg must come from ir[11:9]   //
// and dest_reg from ir[2:0]. Every earlier instruction takes its source    //
// from ir[2:0], so this milestone is the first to override src_reg at all. //
// A swapped pair is exactly what this testbench is built to catch.         //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #4,D0      7004   D0 = 00000004   the BIT NUMBER              //
//   2: MOVEQ #1,D1      7201   D1 = 00000001   the TARGET                  //
//   3: BSET   D0,D1     01C1   D1 = 00000011   bit 4 set                   //
//   4: BCLR   D0,D1     0181   D1 = 00000001   bit 4 cleared again         //
//   5: MOVEQ #0,D2      7400   D2 = 00000000                               //
//   6: BCHG   D0,D2     0142   D2 = 00000010   bit 4 toggled on            //
//                                                                          //
// The operand order is what every check here turns on. Read the pair the   //
// binary family's way -- number from ir[2:0], target from ir[11:9] -- and   //
// BSET would set bit 1 of D0 rather than bit 4 of D1, leaving D1 at        //
// 00000001 and corrupting D0 to 00000006. Both registers are checked, so   //
// a swap cannot hide in the one that happens to look plausible.            //
//                                                                          //
// BSET then BCLR on the same bit returns D1 to its starting value, which    //
// on its own is indistinguishable from neither instruction running. D2     //
// carries the independent proof: it starts at zero, so its final 00000010  //
// can only come from BCHG actually toggling bit 4.                         //
//                                                                          //
// On milestone 23's RTL none of the three decodes.                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bitops;

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
	dut.u_l1.mem[1] = 16'h7004;   // MOVEQ #4,D0
	dut.u_l1.mem[2] = 16'h7201;   // MOVEQ #1,D1
	dut.u_l1.mem[3] = 16'h01C1;   // BSET D0,D1
	dut.u_l1.mem[4] = 16'h0181;   // BCLR D0,D1
	dut.u_l1.mem[5] = 16'h7400;   // MOVEQ #0,D2
	dut.u_l1.mem[6] = 16'h0142;   // BCHG D0,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0004) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000004 (the bit number must not be written)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0001) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000001 (BSET then BCLR of bit 4)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0010) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000010 (BCHG must toggle bit 4 of D2, not of D0)", dbg_d2);
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
