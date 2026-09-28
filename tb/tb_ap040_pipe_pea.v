//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 60: PEA)                  //
//                                                                          //
// tb_ap040_pipe_pea.v - pushing an effective address                       //
//                                                                          //
// Amiga library calls push their arguments, and a pointer argument is a    //
// PEA.                                                                     //
//                                                                          //
// It is LEA with a different destination: the same effective address, sent //
// to the stack instead of to An. So it rides held_is_lea with one carried  //
// property, and the push itself is the BSR/JSR/LINK path, whose address is //
// already operand_b - 4 once eac_dest_reg names A7. The only new wiring is //
// the DATA -- BSR pushes a return address, LINK pushes the old An, PEA     //
// pushes the effective address itself.                                     //
//                                                                          //
// A7 = $0600, A0 = $0480, D1 = 2. Memory at those addresses holds values   //
// that look nothing like addresses, which is the point:                    //
//                                                                          //
//   $0480 = DEADBEEF   $0488 = CAFEBABE   $048C = FEEDFACE                 //
//                                                                          //
//   PEA (A0)             pushes $00000480, NOT DEADBEEF                    //
//   PEA (8,A0)           pushes $00000488                                  //
//   PEA (4,A0,D1.L*4)    pushes $0000048C                                  //
//   PEA (d16,PC)         pushes $00000400                                  //
//                                                                          //
// The failure PEA invites is pushing the CONTENTS rather than the address  //
// -- it is a memory-mode instruction that must not read memory -- so every //
// slot is checked against a value that would be unmistakable if it did.    //
//                                                                          //
// The four modes are deliberately different: register indirect needs no    //
// gather at all, the displacement form gathers one word, the indexed form  //
// gathers a brief word and uses the third read port, and the PC-relative   //
// form takes its base from eac_pc rather than from a register. A single    //
// mode would have shown only that the push works.                          //
//                                                                          //
// A7 = $05F0 at the end is four pushes of four bytes and nothing else --   //
// it would be $05F4 if any one of them had been skipped.                   //
//                                                                          //
// On milestone 59's RTL none of the four decode.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_pea;

localparam PROG_WORDS      = 16;
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
	dut.u_l1.mem[4]  = 16'h223C;   // MOVE.L #$00000002,D1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0002;
	dut.u_l1.mem[7]  = 16'h4850;   // PEA (A0)
	dut.u_l1.mem[8]  = 16'h4868;   // PEA (8,A0)
	dut.u_l1.mem[9]  = 16'h0008;
	dut.u_l1.mem[10] = 16'h4870;   // PEA (4,A0,D1.L*4)
	dut.u_l1.mem[11] = 16'h1C04;
	dut.u_l1.mem[12] = 16'h487A;   // PEA (d16,PC)   base $041A -> $0400
	dut.u_l1.mem[13] = 16'hFFE6;

	dut.u_l1.mem[64] = 16'hDEAD;   // $0480 = DEADBEEF
	dut.u_l1.mem[65] = 16'hBEEF;
	dut.u_l1.mem[68] = 16'hCAFE;   // $0488 = CAFEBABE
	dut.u_l1.mem[69] = 16'hBABE;
	dut.u_l1.mem[70] = 16'hFEED;   // $048C = FEEDFACE
	dut.u_l1.mem[71] = 16'hFACE;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// A7 is the address bank's register 7, which in supervisor mode is the
	// ISP. See tb_ap040_pipe_move_mem.v's header for why the poke has to
	// land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $05FC is word index 254, $05F8 is 252, $05F4 is 250, $05F0 is 248.
	if ({dut.u_l1.mem[254], dut.u_l1.mem[255]} !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: [$05FC] = %h%h, expected 00000480 (PEA (A0); deadbeef means it pushed the CONTENTS)",
		         dut.u_l1.mem[254], dut.u_l1.mem[255]);
	end
	if ({dut.u_l1.mem[252], dut.u_l1.mem[253]} !== 32'h0000_0488) begin
		errors = errors + 1;
		$display("FAIL: [$05F8] = %h%h, expected 00000488 (PEA (8,A0); cafebabe means it pushed the contents)",
		         dut.u_l1.mem[252], dut.u_l1.mem[253]);
	end
	if ({dut.u_l1.mem[250], dut.u_l1.mem[251]} !== 32'h0000_048C) begin
		errors = errors + 1;
		$display("FAIL: [$05F4] = %h%h, expected 0000048c (PEA (4,A0,D1.L*4); feedface means it pushed the contents)",
		         dut.u_l1.mem[250], dut.u_l1.mem[251]);
	end
	if ({dut.u_l1.mem[248], dut.u_l1.mem[249]} !== 32'h0000_0400) begin
		errors = errors + 1;
		$display("FAIL: [$05F0] = %h%h, expected 00000400 (PEA (d16,PC))",
		         dut.u_l1.mem[248], dut.u_l1.mem[249]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_05F0) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 000005f0 (four pushes of four bytes; 000005f4 means one was skipped)",
		         dut.u_cpu.u_regfile.isp);
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
