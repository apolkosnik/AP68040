//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 45: ADDA #imm)      //
//                                                                          //
// tb_ap040_pipe_adda_imm.v - opening and closing a stack frame             //
//                                                                          //
// ADDA.L #n,A7 and SUBA.L #n,A7 are how every compiled function opens and  //
// closes its frame, so milestone 44's family was not yet usable without    //
// them.                                                                    //
//                                                                          //
// They needed no new gather kind and no sign-extension flag. held_is_imm   //
// already assembles immediates, already routes them into operand_a through //
// id_src_a_is_imm, and already writes An without condition codes for       //
// MOVEA.L #imm,An. gather_disp already SIGN-EXTENDS its single-word form,  //
// so the Word immediate arrives 32 bits wide and correct -- milestone 44's //
// id_sxt_w is not involved in these at all.                                //
//                                                                          //
// One rule had to be separated rather than inferred. id_writes_ccr derived //
// "sets no condition codes" from held_imm_areg, i.e. from the destination  //
// being an address register, which held for every immediate form until     //
// now. CMPA breaks it: its destination IS an address register and it DOES  //
// set condition codes. held_imm_ccr carries that directly.                 //
//                                                                          //
//   A7 = $0600                                                             //
//   SUBA.L #$20,A7     open a frame                                        //
//   ADDA.L #$10,A7     close half of it -- A7 = $05F0                      //
//   MOVEA.L #$1000,A1                                                      //
//   ADDA.W #$FFF8,A1   A1 = 00000FF8                                       //
//   MOVEA.L #$0FF8,A3                                                      //
//   CMPA.L #$0FF8,A3   sets Z, writes nothing                              //
//   ADDA.L #$4,A3      A3 = 00000FFC -- and Z must survive it              //
//                                                                          //
// The frame is opened and closed by DIFFERENT amounts on purpose. A        //
// balanced pair returns A7 to $0600, which is also where it lands if       //
// neither instruction ran at all. $20 out and $10 back gives $05F0, and    //
// every interesting failure is a different value: $0610 if the SUBA was    //
// skipped or if the two operand orders are swapped (A7 - n against n - A7  //
// is exactly the mistake SUB's operand ordering invites), $05E0 if the     //
// ADDA was skipped.                                                        //
//                                                                          //
// ADDA.W #$FFF8 is the sign-extension check. A zero-extended immediate     //
// gives $00010FF8 -- a pointer a full 64K away from the right one, and the //
// reason this matters for real code rather than only for a bench.          //
//                                                                          //
// The last ADDA sits after the CMPA so that Z can be checked surviving it. //
//                                                                          //
// On milestone 44's RTL none of the immediate forms decode.                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_adda_imm;

localparam PROG_WORDS      = 24;
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
	dut.u_l1.mem[1]  = 16'h9FFC;   // SUBA.L #$00000020,A7
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0020;
	dut.u_l1.mem[4]  = 16'hDFFC;   // ADDA.L #$00000010,A7
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0010;
	dut.u_l1.mem[7]  = 16'h227C;   // MOVEA.L #$00001000,A1
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h1000;
	dut.u_l1.mem[10] = 16'hD2FC;   // ADDA.W #$FFF8,A1
	dut.u_l1.mem[11] = 16'hFFF8;
	dut.u_l1.mem[12] = 16'h267C;   // MOVEA.L #$00000FF8,A3
	dut.u_l1.mem[13] = 16'h0000;
	dut.u_l1.mem[14] = 16'h0FF8;
	dut.u_l1.mem[15] = 16'hB7FC;   // CMPA.L #$00000FF8,A3
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h0FF8;
	dut.u_l1.mem[18] = 16'hD7FC;   // ADDA.L #$00000004,A3
	dut.u_l1.mem[19] = 16'h0000;
	dut.u_l1.mem[20] = 16'h0004;
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

	if (dut.u_cpu.u_regfile.isp !== 32'h0000_05F0) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 000005f0 ($0600 - $20 + $10; 0610 means the SUBA was skipped or reversed, 05e0 the ADDA)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0FF8) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000ff8 (ADDA.W #$FFF8 must sign-extend; zero-extending gives 00010ff8)",
		         dut.u_cpu.u_regfile.areg[1]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_0FFC) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 00000ffc (CMPA must write nothing, then ADDA.L #4)",
		         dut.u_cpu.u_regfile.areg[3]);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. CMPA set Z although its destination is an
	// address register; the ADDA after it must leave the flags alone.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMPA sets Z; ADDA after it must set nothing)",
		         dbg_ccr[3:0]);
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
