//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 20: the unary       //
// register group)                                                          //
//                                                                          //
// tb_ap040_pipe_unary.v - NOT, NEG, CLR, TST and NEGX on Dn                //
//                                                                          //
// These take their register from ir[2:0], not ir[11:9] like the binary     //
// family, so decode points dest_reg at d_rn the way Scc does. The ALU is   //
// not consistent about which operand a unary op reads -- NOT/NEG/NEGX work //
// on b, TST shares MOVE's arm and works on a, CLR reads neither -- so both //
// selectors point at the same Dn and every member is covered without a     //
// special case. This testbench is built to catch that going wrong.         //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #1,D0     7001   D0 = 00000001                                //
//   2: NOT.L  D0       4680   D0 = FFFFFFFE      (reads b)                 //
//   3: MOVEQ #5,D1     7205   D1 = 00000005                                //
//   4: NEG.W  D1       4441   D1 = 0000FFFB      (word neg, upper kept)    //
//   5: MOVEQ #$7F,D2   747F   D2 = 0000007F                                //
//   6: TST.L  D2       4A82   flags only, D2 UNCHANGED  (reads a)          //
//   7: CLR.B  D2       4202   D2 = 00000000                                //
//                                                                          //
// NOT.L reading operand b is the load-bearing check: src_reg and dest_reg  //
// must BOTH resolve to D0, and if only the ir[11:9] field were wired the   //
// destination would be D3 and D0 would keep 00000001.                      //
//                                                                          //
// NEG.W proves the unary path honours milestone 18's size field -- the     //
// result is a word negate spliced into D1[15:0] with D1[31:16] preserved,  //
// not a longword FFFFFFFB.                                                 //
//                                                                          //
// TST proves the no-write member: it must set flags and leave D2 at 7F. If //
// id_writes_reg let it through, D2 would be overwritten (harmlessly, with  //
// its own value) and the following CLR.B would still pass -- so the check  //
// that actually bites is CLR.B clearing only the low byte, leaving         //
// 0000007F -> 00000000 provable while D2[31:8] was already zero. The       //
// preceding TST check pins D2 at 7F before CLR runs.                       //
//                                                                          //
// On milestone 19's RTL none of the five decodes: they fall through to     //
// illegal and every register check fails.                                  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_unary;

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
	dut.u_l1.mem[2] = 16'h4680;   // NOT.L  D0
	dut.u_l1.mem[3] = 16'h7205;   // MOVEQ #5,D1
	dut.u_l1.mem[4] = 16'h4441;   // NEG.W  D1
	dut.u_l1.mem[5] = 16'h747F;   // MOVEQ #$7F,D2
	dut.u_l1.mem[6] = 16'h4A82;   // TST.L  D2
	dut.u_l1.mem[7] = 16'h4202;   // CLR.B  D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hFFFF_FFFE) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected FFFFFFFE (NOT.L reads operand b; both selectors must be D0)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_FFFB) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000FFFB (NEG.W must splice into D1[15:0])", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000 (CLR.B after TST left D2 at 7F)", dbg_d2);
	end

	// CLR is last and always sets Z with N=V=C=0. dbg_ccr[3:0] is {N,Z,V,C}.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CLR sets Z)", dbg_ccr[3:0]);
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
