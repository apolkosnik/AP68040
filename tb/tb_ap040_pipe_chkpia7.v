//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 106: a faulting     //
// CHK through (A7)+)                                                       //
//                                                                          //
// tb_ap040_pipe_chkpia7.v - the postincrement of a DEFERRED fault          //
//                                                                          //
// tb_ap040_pipe_faultpi.v already proves that a user-mode instruction      //
// faulting through (A7)+ leaves the postincrement on USP and the frame on  //
// ISP. It proves it with DIVU.W, whose fault is known the moment the       //
// operand arrives, and it passes.                                          //
//                                                                          //
// CHK is the same shape and does NOT pass, because its fault is not        //
// decided in the same cycle. chk_now needs the BOUND, which for a memory   //
// source is the loaded word, so it is qualified by                         //
// mem_pending && l1_rvalid_b; when that cycle is not also the cycle the    //
// entry is taken, the verdict goes into exc_pend_chk and the exception is  //
// entered LATER. an_wr_any, which carries the postincrement, is            //
// combinational on the eac_* fields -- so whether the increment survives   //
// depends on whether those fields still describe the CHK when the deferred //
// entry finally runs. That is a race, and the cputest corpus says this     //
// core loses it: 5,668 rounds of Basic/CHK.W disagree, every one of them   //
// an opcode with A7 as the EA register (459f, 45a7, 479f, 47a7), and every //
// one reading the ORIGINAL A7 back instead of the incremented one.         //
//                                                                          //
//   USP = $800, ISP = $1000, the BOUND word at $800 is 10                  //
//   MOVE D1,SR with D1 = 0       drop to user mode                         //
//   CHK.W (A7)+,D0 with D0 = 20  20 > 10 -> vector 6, format $2 frame      //
//                                                                          //
// USP must be $802 and ISP must be $FF4: twelve bytes of frame, and the    //
// Word operand stepped A7 by two. $800 is the failure this was written     //
// for -- the postincrement discarded with the flush.                       //
//                                                                          //
// D0 = 20 against a bound of 10 traps on the OVER path, not the negative   //
// one, so the stacked N must be CLEAR. It is checked here too, because the //
// deferred verdict carries exc_pend_chk_n alongside exc_pend_chk and a fix //
// that re-derived the fault at entry time instead of carrying the latch    //
// would get the increment right and N wrong. The handler files the stacked //
// SR to memory the way tb_ap040_pipe_chkpi.v does, and reads it back       //
// through D4.                                                             //
//                                                                          //
// TST.L D5 with D5 = $FFFFFF80 immediately before the CHK is the poison    //
// for that N check. Without it the MOVE to SR has just cleared the whole    //
// CCR, so N is already 0 going in and a core that stacked the UNMODIFIED    //
// SR would show N clear too, for free. With it the live N is SET on the     //
// way in and the stacked N must still come back CLEAR: the two can only     //
// agree if CHK really wrote it.                                             //
//                                                                          //
// The handler's marker and the poison after the faulting instruction are   //
// both checked, so a run where the exception never happened cannot pass on //
// the register values alone.                                               //
//                                                                          //
// Vector 6 sits at word index 3596 of the L1 window, which maps a byte     //
// address to (address - PC_RESET) >> 1 over 2**AW words.                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_chkpia7;

localparam PROG_WORDS      = 40;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
// A cycle with ce low is a cycle that did not happen, and a deferred fault
// is exactly the kind of thing that notices. Driven on the falling edge so
// it is stable across every rising one.
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
wire [31:0] dbg_d2, dbg_d3, dbg_d4;
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

	.dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4),
	.dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000800,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0800;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,USP
	dut.u_l1.mem[ 5] = 16'h0800;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h1000;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D0,ISP
	dut.u_l1.mem[10] = 16'h0804;
	dut.u_l1.mem[11] = 16'h203C;   // MOVE.L #$00000014,D0  (the value: 20)
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0014;
	dut.u_l1.mem[14] = 16'h7A80;   // MOVEQ #$80,D5  -> sets the live N
	dut.u_l1.mem[15] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[16] = 16'h46C1;   // MOVE D1,SR   (S -> 0, CCR -> 0)
	dut.u_l1.mem[17] = 16'h4A85;   // TST.L D5     -> live N SET going into CHK
	dut.u_l1.mem[18] = 16'h419F;   // CHK.W (A7)+,D0   bound at $800 = 10
	dut.u_l1.mem[19] = 16'h7466;   // MOVEQ #$66,D2 (poison: must not run)
	dut.u_l1.mem[20] = 16'h4E71;   // NOP

	// CHK handler @ word idx 384 (byte $700): file the stacked SR away.
	dut.u_l1.mem[384] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[385] = 16'h2817;  // MOVE.L (A7),D4   the stacked {SR, PC_hi}
	dut.u_l1.mem[386] = 16'h4E71;  // NOP

	// The user stack's first word is the BOUND: 10.
	dut.u_l1.mem[512] = 16'd10;    // $0800
	dut.u_l1.mem[513] = 16'h0000;

	// Vector 6 (CHK) -> $700.
	dut.u_l1.mem[3596] = 16'h0000;
	dut.u_l1.mem[3597] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 160) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the CHK handler must run)", dbg_d3);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000 (the instruction after the faulting one must not run)", dbg_d2);
	end
	if (dut.u_cpu.u_regfile.usp !== 32'h0000_0802) begin
		errors = errors + 1;
		$display("FAIL: USP = %h, expected 00000802 (CHK's postincrement belongs to a USER-mode instruction and must land on USP even though the fault is decided a cycle later; 00000800 means the deferred entry discarded it)",
		         dut.u_cpu.u_regfile.usp);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0FF4) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00000ff4 (the format $2 frame is twelve bytes below $1000; 00000802 means the postincrement was banked as supervisor and overwrote it)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dbg_sr[13] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: SR.S = %b, expected 1 (the handler runs in supervisor mode)", dbg_sr[13]);
	end
	// The stacked SR is the top word of the format $2 frame, so D4's HIGH
	// half is it. N is bit 3. 20 > 10 traps on the OVER path: N must be
	// CLEAR, and the live N was SET on the way in.
	if (dbg_d4[27] !== 1'b0) begin
		errors = errors + 1;
		$display("FAIL: stacked SR = %h, N = %b, expected 0 (CHK traps here because the value EXCEEDS the bound, which clears N; the live N was set going in, so a stale SR shows 1)",
		         dbg_d4[31:16], dbg_d4[27]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
