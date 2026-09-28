//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 37: sized loads for //
// the gathering modes)                                                      //
//                                                                          //
// tb_ap040_pipe_szgath.v - a size that has to survive the gather           //
//                                                                          //
// (d16,An) and absolute are the last two load modes still Long. They differ //
// from (An), (An)+ and -(An) in one way that matters here: they GATHER, so  //
// by the time the instruction is emitted the opcode word is gone and its    //
// size cannot be read off it. It has to be held across the extension words, //
// exactly as the immediate forms hold theirs since milestone 26.            //
//                                                                          //
// Memory at $0480: 11 22 33 44                                              //
//                                                                          //
//   MOVEQ   #-1,D0 / #-1,D1 / #-1,D2      so splices are visible            //
//   MOVEA.L #$0480,A0                                                       //
//   MOVE.W  (2,A0),D0        D0 = FFFF3344   displacement + Word            //
//   MOVE.B  ($0481).W,D1     D1 = FFFFFF22   one-word address + Byte        //
//   MOVE.W  ($0482).L,D2     D2 = FFFF3344   two-word address + Word        //
//                                                                          //
// Each line pairs a gather width with an operand size, which is the         //
// combination this milestone can get wrong: holding the size but applying   //
// the wrong gather width, or gathering correctly and defaulting the size to //
// Long. A Long default shows immediately, because all three destinations    //
// start as all-ones and a Long load would overwrite them completely.        //
//                                                                          //
// D0 and D2 name the same bytes by different routes -- once as a            //
// displacement from A0 and once as a two-word absolute address -- so they   //
// must agree. If either mode applied the size to the wrong one, they        //
// diverge.                                                                  //
//                                                                          //
// The byte load again uses an ODD address, where the neighbouring byte      //
// holds a different value, so a wrong lane gives a wrong answer.            //
//                                                                          //
// On milestone 36's RTL none of the three decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_szgath;

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
	dut.u_l1.mem[1]  = 16'h70FF;   // MOVEQ #-1,D0
	dut.u_l1.mem[2]  = 16'h72FF;   // MOVEQ #-1,D1
	dut.u_l1.mem[3]  = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[4]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0480;
	dut.u_l1.mem[7]  = 16'h3028;   // MOVE.W (2,A0),D0
	dut.u_l1.mem[8]  = 16'h0002;
	dut.u_l1.mem[9]  = 16'h1238;   // MOVE.B ($0481).W,D1
	dut.u_l1.mem[10] = 16'h0481;
	dut.u_l1.mem[11] = 16'h3439;   // MOVE.W ($00000482).L,D2
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0482;

	dut.u_l1.mem[64] = 16'h1122;
	dut.u_l1.mem[65] = 16'h3344;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 44) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hFFFF_3344) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected FFFF3344 ((d16,An) + Word; a Long default overwrites D0[31:16])", dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_FF22) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected FFFFFF22 (one-word address + Byte, at an ODD address)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_3344) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFF3344 (two-word address + Word)", dbg_d2);
	end
	// The two routes to the same bytes must agree.
	if (dbg_d0 !== dbg_d2) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h and D2 = %h name the same bytes by different modes and disagree", dbg_d0, dbg_d2);
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
