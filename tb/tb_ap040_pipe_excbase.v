//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 93: one frame, one //
// base)                                                                    //
//                                                                          //
// tb_ap040_pipe_excbase.v - an exception frame split across two stacks     //
//                                                                          //
// The exception sequencer reads the stack pointer out of the register file //
// live, once per beat, rather than resolving it once and keeping it. The   //
// register file is not still while that happens: an OLDER instruction that //
// writes A7 commits between one beat and the next, and the two halves of   //
// the frame land 512 bytes apart.                                          //
//                                                                          //
//   ISP = $1000 ; MOVEA.L #$1200,A7 ; TRAP #0                              //
//                                                                          //
// The whole frame belongs at $11F8, eight bytes below the pointer the      //
// MOVEA installed, and the final ISP is $11F8. What happens instead is     //
// that beat 0 is written from the OLD pointer and beat 1 from the new one, //
// so there is half a frame at $0FF8 and half at $11FC, and the RTE that    //
// reads $11F8 gets whatever was already there.                             //
//                                                                          //
// Both halves are checked, and so is the word at $0FF8, because a frame    //
// that is merely in the wrong PLACE is a different defect from one that is //
// in two places.                                                           //
//                                                                          //
// The format and vector word pins it down: format $0, vector 32, so $0080. //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_excbase;

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
wire [31:0] dbg_d1, dbg_d2, dbg_d3;
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

	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_sr (dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP    (ISP = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h2E7C;   // MOVEA.L #$00001200,A7
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h1200;
	dut.u_l1.mem[ 9] = 16'h4E40;   // TRAP #0
	dut.u_l1.mem[10] = 16'h4E71;   // NOP (drain)

	// TRAP #0 handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[385] = 16'h4E71;  // NOP

	// Vector 32 -> $700.
	dut.u_l1.mem[3648] = 16'h0000;
	dut.u_l1.mem[3649] = 16'h0700;

	// $11F8 and $11FC, where the frame belongs.
	dut.u_l1.mem[1788] = 16'h9999;  dut.u_l1.mem[1789] = 16'h9999;
	dut.u_l1.mem[1790] = 16'h9999;  dut.u_l1.mem[1791] = 16'h9999;
	// $0FF8, where half of it went.
	dut.u_l1.mem[1532] = 16'h6666;  dut.u_l1.mem[1533] = 16'h6666;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the TRAP handler must run)", dbg_d3);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_11F8) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 000011f8 (eight bytes below the pointer MOVEA installed)", dut.u_cpu.u_regfile.isp);
	end
	if (dut.u_l1.mem[1789] !== 16'h0000 || dut.u_l1.mem[1788] === 16'h9999) begin
		errors = errors + 1;
		$display("FAIL: frame word0 at $11F8 = %h%h, still the sentinel or wrong -- the first beat went somewhere else",
		         dut.u_l1.mem[1788], dut.u_l1.mem[1789]);
	end
	if (dut.u_l1.mem[1790] !== 16'h0414 || dut.u_l1.mem[1791] !== 16'h0080) begin
		errors = errors + 1;
		$display("FAIL: frame word1 at $11FC = %h%h, expected 04140080 (the return address and format $0 vector 32)",
		         dut.u_l1.mem[1790], dut.u_l1.mem[1791]);
	end
	if ({dut.u_l1.mem[1532], dut.u_l1.mem[1533]} !== 32'h6666_6666) begin
		errors = errors + 1;
		$display("FAIL: $0FF8 = %h%h, expected 66666666 -- a beat of the frame was written from the stack pointer as it stood BEFORE the older instruction committed",
		         dut.u_l1.mem[1532], dut.u_l1.mem[1533]);
	end
	if (writes !== 2) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted, expected 2 (a format $0 frame is two beats)", writes);
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
