//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 61: LEA/PEA absolute)     //
//                                                                          //
// tb_ap040_pipe_eaabs.v - LEA $xxx,An and PEA $xxx                         //
//                                                                          //
// PEA $xxx.L is how a string constant gets pushed. LEA $xxx.L,An is a real //
// encoding a compiler may emit even though MOVEA.L #imm,An has the same    //
// effect -- milestone 42 noted that equivalence and then left the opcode   //
// itself undecoded, which is a gap rather than a redundancy, and milestone //
// 60 named it while closing the other PEA modes.                           //
//                                                                          //
// Both ride held_is_abs, which already gathers the address and already     //
// sets id_is_abs so ea_target is eac_imm. The property they add is that    //
// the instruction delivers the ADDRESS rather than the contents: no memory //
// read at all, and no condition codes. Milestone 59 did not need that      //
// distinction, because every absolute form it reached did read memory.     //
//                                                                          //
// $0480 = DEADBEEF and $0484 = CAFEBABE, so anything that reads instead of //
// addressing is unmistakable.                                              //
//                                                                          //
//   MOVEQ  #-1,D0        poison: see below                                 //
//   LEA    $00000480.L,A1    A1 = 00000480                                 //
//   LEA    $0484.W,A2        A2 = 00000484, the SHORT form                 //
//   PEA    $00000480.L       pushes 00000480                               //
//   PEA    $0484.W           pushes 00000484                               //
//   MOVEQ  #1,D0             leaves Z clear and N clear                    //
//                                                                          //
// The MOVEQ #-1 is the poison this series has needed repeatedly. An        //
// absolute LEA or PEA reads no memory, so if id_is_mem_src were left set   //
// the instruction would load instead -- and the checks would still see an  //
// address if that load happened to return one. Seeding D0, and seeding the //
// targets with DEADBEEF and CAFEBABE, makes every wrong route produce      //
// something that is plainly not an address.                                //
//                                                                          //
// The final MOVEQ #1 is there so the CCR check means something: neither    //
// LEA nor PEA may touch the flags, and MOVEQ #1 leaves N and Z both clear  //
// where the DEADBEEF a wrongly-reading LEA would deliver sets N.           //
//                                                                          //
// Both widths are used: the short form gathers one sign-extended word, the //
// long form two.                                                           //
//                                                                          //
// On milestone 60's RTL none of the four decode.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_eaabs;

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
	dut.u_l1.mem[1]  = 16'h70FF;   // MOVEQ #-1,D0   -- poison
	dut.u_l1.mem[2]  = 16'h43F9;   // LEA $00000480.L,A1
	dut.u_l1.mem[3]  = 16'h0000;
	dut.u_l1.mem[4]  = 16'h0480;
	dut.u_l1.mem[5]  = 16'h45F8;   // LEA $0484.W,A2   (short absolute)
	dut.u_l1.mem[6]  = 16'h0484;
	dut.u_l1.mem[7]  = 16'h4879;   // PEA $00000480.L
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h0480;
	dut.u_l1.mem[10] = 16'h4878;   // PEA $0484.W
	dut.u_l1.mem[11] = 16'h0484;
	dut.u_l1.mem[12] = 16'h7001;   // MOVEQ #1,D0

	dut.u_l1.mem[64] = 16'hDEAD;   // $0480 = DEADBEEF
	dut.u_l1.mem[65] = 16'hBEEF;
	dut.u_l1.mem[66] = 16'hCAFE;   // $0484 = CAFEBABE
	dut.u_l1.mem[67] = 16'hBABE;
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

	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000480 (LEA $xxx.L; deadbeef means it read instead of addressing)",
		         dut.u_cpu.u_regfile.areg[1]);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_0484) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00000484 (LEA $xxx.W, the short form)", dut.u_cpu.u_regfile.areg[2]);
	end
	// $05FC is word index 254, $05F8 is 252.
	if ({dut.u_l1.mem[254], dut.u_l1.mem[255]} !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: [$05FC] = %h%h, expected 00000480 (PEA $xxx.L)",
		         dut.u_l1.mem[254], dut.u_l1.mem[255]);
	end
	if ({dut.u_l1.mem[252], dut.u_l1.mem[253]} !== 32'h0000_0484) begin
		errors = errors + 1;
		$display("FAIL: [$05F8] = %h%h, expected 00000484 (PEA $xxx.W)",
		         dut.u_l1.mem[252], dut.u_l1.mem[253]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_05F8) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 000005f8 (two pushes of four bytes)", dut.u_cpu.u_regfile.isp);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. MOVEQ #1 leaves them all clear; neither LEA
	// nor PEA may have touched them, and a wrongly-reading LEA would have
	// delivered deadbeef and set N.
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000 (LEA and PEA set no flags)", dbg_ccr[3:0]);
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
