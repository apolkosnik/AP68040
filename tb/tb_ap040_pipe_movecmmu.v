//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 113: the MMU        //
// registers through MOVEC, and a selector that names nothing)              //
//                                                                          //
// tb_ap040_pipe_movecmmu.v - write masks, read-back, and the two orders    //
//                                                                          //
// Milestone 113 takes TC, ITT0/1, DTT0/1, MMUSR, URP and SRP in as         //
// storage: MOVEC writes them through ap040_core.v's masks                  //
// (ap040_core.v:3850-3870) and reads them back, and nothing translates.    //
// The corpus only READS them, from reset, so it sees zero every time and   //
// cannot tell a mask from a missing register or one selector from         //
// another. Here every register gets its own pattern, so a read that lands  //
// on the wrong register or a write that skips its mask shows as a value:   //
//                                                                          //
//   TC     <- $1234F5A5   reads $0000C000   (& $0000C000)                  //
//   ITT0   <- $A5A5FFFF   reads $A5A5E364   (& $FFFFE364)                  //
//   ITT1   <- $5A5AFFFF   reads $5A5AE364                                  //
//   DTT0   <- $3C3CFFFF   reads $3C3CE364                                  //
//   DTT1   <- $C3C3FFFF   reads $C3C3E364                                  //
//   MMUSR  <- $87654321   reads $87654321   (unmasked)                     //
//   URP    <- $1111FFFF   reads $1111FE00   (& $FFFFFE00)                  //
//   SRP    <- $2222FFFF   reads $2222FE00                                  //
//                                                                          //
// The same milestone fixed two defects in a MOVEC whose selector is not a  //
// 68040 register, which the corpus's MOVEC2 sweep of all 4,096 selectors  //
// found, and each has its own observable here:                             //
//                                                                          //
//  - In supervisor mode it is an illegal instruction, and the frame push   //
//    owned the MOVEC's own Rn as its destination: the new stack pointer    //
//    was written into it. D5 = $55 goes in, the MOVEC names D5, and D5 is  //
//    saved to $604 after the handler returns. It must still be $55.        //
//                                                                          //
//  - In user mode it is a PRIVILEGE violation, not an illegal one: the     //
//    68040 checks S before it fetches the extension word                   //
//    (ap040_core.v:6053). This core raised vector 4. The two handlers      //
//    count into $600 (vector 4) and $602 (vector 8): one illegal, from     //
//    the supervisor MOVEC, and two privilege violations -- the invalid     //
//    selector and a valid one (VBR) behind it, both from user mode. D0     //
//    and D1 are the user MOVECs' destinations and must keep what the       //
//    read-back put there.                                                  //
//                                                                          //
// Both handlers step the stacked PC past the 4-byte MOVEC and RTE.         //
// Vector 4 -> $700, vector 8 -> $780; VBR is $2000.                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movecmmu;

localparam PROG_WORDS      = 400;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
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
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),
	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;

task chkd;
	input [63:0]  name;
	input [31:0]  got;
	input [31:0]  want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", name, got, want);
		end
	end
endtask

task chkw;
	input [255:0] what;
	input integer widx;
	input  [15:0] want;
	begin
		if (dut.u_l1.mem[widx] !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %04x, expected %04x", what, dut.u_l1.mem[widx], want);
		end
	end
endtask

// MOVE.L #imm,D0 ; MOVEC D0,<sel>  -- five words from word index w
task wr_creg;
	input integer w;
	input [31:0] value;
	input [15:0] sel;
	begin
		dut.u_l1.mem[w]   = 16'h203C;
		dut.u_l1.mem[w+1] = value[31:16];
		dut.u_l1.mem[w+2] = value[15:0];
		dut.u_l1.mem[w+3] = 16'h4E7B;
		dut.u_l1.mem[w+4] = sel;
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 1] = 16'h0000;
	dut.u_l1.mem[ 2] = 16'h1000;
	dut.u_l1.mem[ 3] = 16'h4E7B;   // MOVEC D0,ISP      (A7 = $1000)
	dut.u_l1.mem[ 4] = 16'h0804;
	dut.u_l1.mem[ 5] = 16'h7A55;   // MOVEQ #$55,D5
	dut.u_l1.mem[ 6] = 16'h4E7A;   // MOVEC CAAR,D5     -- not a 68040 register: vector 4
	dut.u_l1.mem[ 7] = 16'h5802;
	dut.u_l1.mem[ 8] = 16'h21C5;   // MOVE.L D5,$0604.W -- what the illegal left in D5
	dut.u_l1.mem[ 9] = 16'h0604;

	wr_creg(10, 32'h1234_F5A5, 16'h0003);   // TC
	wr_creg(15, 32'hA5A5_FFFF, 16'h0004);   // ITT0
	wr_creg(20, 32'h5A5A_FFFF, 16'h0005);   // ITT1
	wr_creg(25, 32'h3C3C_FFFF, 16'h0006);   // DTT0
	wr_creg(30, 32'hC3C3_FFFF, 16'h0007);   // DTT1
	wr_creg(35, 32'h8765_4321, 16'h0805);   // MMUSR
	wr_creg(40, 32'h1111_FFFF, 16'h0806);   // URP
	wr_creg(45, 32'h2222_FFFF, 16'h0807);   // SRP

	dut.u_l1.mem[50] = 16'h4E7A;  dut.u_l1.mem[51] = 16'h1003;   // MOVEC TC,D1
	dut.u_l1.mem[52] = 16'h4E7A;  dut.u_l1.mem[53] = 16'h2004;   // MOVEC ITT0,D2
	dut.u_l1.mem[54] = 16'h4E7A;  dut.u_l1.mem[55] = 16'h3005;   // MOVEC ITT1,D3
	dut.u_l1.mem[56] = 16'h4E7A;  dut.u_l1.mem[57] = 16'h4006;   // MOVEC DTT0,D4
	dut.u_l1.mem[58] = 16'h4E7A;  dut.u_l1.mem[59] = 16'h5007;   // MOVEC DTT1,D5
	dut.u_l1.mem[60] = 16'h4E7A;  dut.u_l1.mem[61] = 16'h6805;   // MOVEC MMUSR,D6
	dut.u_l1.mem[62] = 16'h4E7A;  dut.u_l1.mem[63] = 16'h7806;   // MOVEC URP,D7
	dut.u_l1.mem[64] = 16'h4E7A;  dut.u_l1.mem[65] = 16'h0807;   // MOVEC SRP,D0

	dut.u_l1.mem[66] = 16'h027C;  dut.u_l1.mem[67] = 16'hDFFF;   // ANDI #$DFFF,SR -- user mode
	dut.u_l1.mem[68] = 16'h4E7A;  dut.u_l1.mem[69] = 16'h0802;   // MOVEC CAAR,D0  -- user: vector 8
	dut.u_l1.mem[70] = 16'h4E7A;  dut.u_l1.mem[71] = 16'h1801;   // MOVEC VBR,D1   -- user: vector 8
	dut.u_l1.mem[72] = 16'h60FE;                                 // BRA.B -2

	// $600/$602 count the two vectors; $604 receives D5.
	dut.u_l1.mem[256] = 16'h0000;
	dut.u_l1.mem[257] = 16'h0000;
	dut.u_l1.mem[258] = 16'hAAAA;
	dut.u_l1.mem[259] = 16'hAAAA;

	// $700: vector 4 (illegal instruction)
	dut.u_l1.mem[384] = 16'h5278;  dut.u_l1.mem[385] = 16'h0600;  // ADDQ.W #1,$0600.W
	dut.u_l1.mem[386] = 16'h58AF;  dut.u_l1.mem[387] = 16'h0002;  // ADDQ.L #4,2(A7)
	dut.u_l1.mem[388] = 16'h4E73;                                 // RTE

	// $780: vector 8 (privilege violation)
	dut.u_l1.mem[448] = 16'h5278;  dut.u_l1.mem[449] = 16'h0602;  // ADDQ.W #1,$0602.W
	dut.u_l1.mem[450] = 16'h58AF;  dut.u_l1.mem[451] = 16'h0002;  // ADDQ.L #4,2(A7)
	dut.u_l1.mem[452] = 16'h4E73;                                 // RTE

	// Vector 4 -> $700, vector 8 -> $780
	dut.u_l1.mem[3592] = 16'h0000;  dut.u_l1.mem[3593] = 16'h0700;
	dut.u_l1.mem[3600] = 16'h0000;  dut.u_l1.mem[3601] = 16'h0780;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chkd("TC   D1", dbg_d1, 32'h0000_C000);
	chkd("ITT0 D2", dbg_d2, 32'hA5A5_E364);
	chkd("ITT1 D3", dbg_d3, 32'h5A5A_E364);
	chkd("DTT0 D4", dbg_d4, 32'h3C3C_E364);
	chkd("DTT1 D5", dbg_d5, 32'hC3C3_E364);
	chkd("MMUSR D6", dbg_d6, 32'h8765_4321);
	chkd("URP  D7", dbg_d7, 32'h1111_FE00);
	chkd("SRP  D0", dbg_d0, 32'h2222_FE00);

	// One illegal (the supervisor MOVEC with an invalid selector); two
	// privilege violations (both user-mode MOVECs, the invalid selector
	// included); and D5 as the illegal left it -- the frame must not write Rn.
	chkw("vector 4 count at $600", 256, 16'h0001);
	chkw("vector 8 count at $602", 257, 16'h0002);
	chkw("saved D5 high at $604",  258, 16'h0000);
	chkw("saved D5 low at $606",   259, 16'h0055);

	if (dbg_sr[13] !== 1'b0) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected user mode at the end (S clear)", dbg_sr);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
