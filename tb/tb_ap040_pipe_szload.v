//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 35: sized loads)    //
//                                                                          //
// tb_ap040_pipe_szload.v - byte and word loads from (An)                   //
//                                                                          //
// Everything touching memory has been Long only since milestone 9b. The L1 //
// always returns a full longword on port B -- address_b names the HIGH word //
// and the low one is implicit -- so a sized load is a lane select on the    //
// way out rather than a narrower access, and needs no L1 change at all.     //
// (A sized STORE does, which is why this milestone is loads only.)          //
//                                                                          //
// The selected value lands in the LOW bits because ap040_pipe_alu.v masks   //
// operand a by size and ap040_execute.v splices the result back by size, so //
// the rest of the path already does the right thing once the right bits     //
// arrive.                                                                   //
//                                                                          //
// Memory at $0480: 11 22 33 44                                              //
//                                                                          //
//   MOVEA.L #$0480,A0                                                       //
//   MOVE.L  (A0),D0      D0 = 11223344   whole longword                     //
//   MOVE.W  (A0),D1      D1 = FFFF1122   HIGH word, upper half preserved     //
//   MOVEA.L #$0481,A1                                                       //
//   MOVE.B  (A1),D2      D2 = FFFFFF22   ODD byte address                    //
//                                                                           //
// D1 and D2 are pre-loaded with all-ones so the preserved upper bits are     //
// visible: a load that ignored the size would give 11223344 in both, and     //
// one that zero-filled instead of splicing would give 00001122 and           //
// 00000022. Only a correct sized load gives FFFF1122 and FFFFFF22.           //
//                                                                           //
// The byte load uses an ODD address deliberately. Both bytes of the high     //
// word are 11 and 22, so reading address bit 0 the wrong way round returns   //
// 11 instead of 22 -- a wrong answer rather than a coincidentally right one. //
//                                                                           //
// On milestone 34's RTL the byte and word forms do not decode.               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_szload;

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
	dut.u_l1.mem[1]  = 16'h72FF;   // MOVEQ #-1,D1   (so the splice is visible)
	dut.u_l1.mem[2]  = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[3]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[4]  = 16'h0000;
	dut.u_l1.mem[5]  = 16'h0480;
	dut.u_l1.mem[6]  = 16'h2010;   // MOVE.L (A0),D0
	dut.u_l1.mem[7]  = 16'h3210;   // MOVE.W (A0),D1
	dut.u_l1.mem[8]  = 16'h227C;   // MOVEA.L #$00000481,A1
	dut.u_l1.mem[9]  = 16'h0000;
	dut.u_l1.mem[10] = 16'h0481;
	dut.u_l1.mem[11] = 16'h1411;   // MOVE.B (A1),D2

	dut.u_l1.mem[64] = 16'h1122;
	dut.u_l1.mem[65] = 16'h3344;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 40) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11223344 (the Long load must be unchanged)", dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_1122) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected FFFF1122 (Word: high half of the pair, spliced into D1[15:0])", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_FF22) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFFFF22 (Byte at an ODD address; FFFFFF11 means bit 0 read backwards)", dbg_d2);
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
