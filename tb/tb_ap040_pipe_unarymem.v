//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 58: unary ops on memory)  //
//                                                                          //
// tb_ap040_pipe_unarymem.v - CLR, NOT, NEG and TST against memory          //
//                                                                          //
// Until now none of these could name anything but a data register, and     //
// CLR and TST in particular are everywhere -- zeroing a field, polling a   //
// flag.                                                                    //
//                                                                          //
// They split across two paths that already exist, and the split is not     //
// arbitrary: ap040_pipe_alu.v computes NOT, NEG, NEGX and CLR from operand //
// B, and TST from operand A. Milestone 48's read-modify-write crossover    //
// puts the loaded value in B; the ordinary memory-source path puts it in   //
// A. So TST is a plain load with no store and the other four are RMWs,     //
// with no new datapath on either route.                                    //
//                                                                          //
// Memory: $0480 = 11112222, $0484 = FFFF0000,                              //
//         $0488 = 00000005, $048C = 0080FFFF.                              //
//                                                                          //
//   CLR.W (A0)   $0480 -> 00002222                                         //
//   NOT.L (A1)   $0484 -> 0000FFFF                                         //
//   NEG.L (A2)   $0488 -> FFFFFFFB                                         //
//   TST.B (A3)   reads the byte 00 -> Z, writes nothing                    //
//                                                                          //
// CLR is SIZED on purpose. A Long clear would leave 00000000 and look      //
// perfectly reasonable, so only a Word one shows the byte enables reaching //
// the unary path -- the half the instruction does not name has to survive. //
//                                                                          //
// TST.B is sized for the same reason: the byte at $048C is 00, so a Byte   //
// TST sets Z where a Long TST of 0080FFFF would leave N and Z both clear.  //
//                                                                          //
// The MOVEQ #-1,D0 at the top exists solely to make a WRONGLY ROUTED TST   //
// visible, and it was added after a mutation proved the bench could not    //
// otherwise see one. Routing TST through the RMW crossover makes its       //
// operand the DESTINATION register rather than the loaded value -- which   //
// is D0 here, since a unary memory op names no data register and decode    //
// leaves the field at zero. With D0 also zero, the mis-routed TST computed //
// zero, set Z anyway and stored a zero byte over a byte that was already   //
// zero: every check passed. With D0 = FFFFFFFF the same mis-routing sets   //
// N instead of Z and writes FF into $048C, and both checks catch it.       //
//                                                                          //
// The general point is worth keeping: when an instruction's correct result //
// equals what it read, a bench has to make the WRONG path produce          //
// something distinctive, because the right one produces nothing.           //
//                                                                          //
// NEG is checked with a positive operand so its result is negative and     //
// distinguishable from NOT's, which is the other way round here: ~FFFF0000 //
// is 0000FFFF and -5 is FFFFFFFB, and neither could be produced by the     //
// other operation on its own operand.                                      //
//                                                                          //
// On milestone 57's RTL none of the four decode.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_unarymem;

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
	dut.u_l1.mem[1]  = 16'h70FF;   // MOVEQ #-1,D0  -- see header: poisons the RMW path
	dut.u_l1.mem[2]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[3]  = 16'h0000;
	dut.u_l1.mem[4]  = 16'h0480;
	dut.u_l1.mem[5]  = 16'h4250;   // CLR.W (A0)
	dut.u_l1.mem[6]  = 16'h227C;   // MOVEA.L #$00000484,A1
	dut.u_l1.mem[7]  = 16'h0000;
	dut.u_l1.mem[8]  = 16'h0484;
	dut.u_l1.mem[9]  = 16'h4691;   // NOT.L (A1)
	dut.u_l1.mem[10] = 16'h247C;   // MOVEA.L #$00000488,A2
	dut.u_l1.mem[11] = 16'h0000;
	dut.u_l1.mem[12] = 16'h0488;
	dut.u_l1.mem[13] = 16'h4492;   // NEG.L (A2)
	dut.u_l1.mem[14] = 16'h267C;   // MOVEA.L #$0000048C,A3
	dut.u_l1.mem[15] = 16'h0000;
	dut.u_l1.mem[16] = 16'h048C;
	dut.u_l1.mem[17] = 16'h4A13;   // TST.B (A3)

	dut.u_l1.mem[64] = 16'h1111;   // $0480 = 11112222
	dut.u_l1.mem[65] = 16'h2222;
	dut.u_l1.mem[66] = 16'hFFFF;   // $0484 = FFFF0000
	dut.u_l1.mem[67] = 16'h0000;
	dut.u_l1.mem[68] = 16'h0000;   // $0488 = 00000005
	dut.u_l1.mem[69] = 16'h0005;
	dut.u_l1.mem[70] = 16'h0080;   // $048C = 0080FFFF
	dut.u_l1.mem[71] = 16'hFFFF;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0000_2222) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 00002222 (CLR.W must clear only the half it names; 00000000 means it went Long)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0000_FFFF) begin
		errors = errors + 1;
		$display("FAIL: $0484 = %h%h, expected 0000ffff (NOT.L)", dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'hFFFF_FFFB) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected fffffffb (NEG.L of 5)", dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if ({dut.u_l1.mem[70], dut.u_l1.mem[71]} !== 32'h0080_FFFF) begin
		errors = errors + 1;
		$display("FAIL: $048C = %h%h, expected 0080ffff (TST must leave memory alone)",
		         dut.u_l1.mem[70], dut.u_l1.mem[71]);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. The byte at $048C is 00, so a Byte TST sets
	// Z; a Long TST of 0080ffff would leave N and Z both clear. This is the
	// only check that can distinguish TST from a wrongly-routed RMW, since
	// TST's result is the value it read and storing it back is invisible.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (TST.B of a zero byte sets Z)", dbg_ccr[3:0]);
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
