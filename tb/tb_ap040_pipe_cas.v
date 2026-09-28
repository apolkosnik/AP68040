//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: CAS)           //
//                                                                          //
// tb_ap040_pipe_cas.v - CAS Dc,Du,<ea> in all three sizes                  //
//                                                                          //
// Every expected value below comes from an independent Python model of    //
// the 68k rule, not from the RTL: CMP <ea> - Dc at the operand size sets   //
// N, Z, V and C (X is untouched); equal stores Du at <ea>, not equal loads //
// <ea> into Dc's low byte/word/long and leaves the rest of Dc.             //
//                                                                          //
//   CAS.B D1,D2,(A0)         $5A = $5A: $800 <- $C3, Z                     //
//   CAS.W D3,D4,(A1)+        $8001 vs $0001: D3 = $AAAA8001, N; A1 = $812  //
//   CAS.L D5,D6,-(A2)        equal: $824 <- $01020304, Z; A2 = $824        //
//   CAS.L D0,D7,(4,A3,A4.L)  equal: $844 <- $11223344, Z. The store must   //
//                            go where the load went -- port C moves from   //
//                            the index to Du while the load is out, and    //
//                            the address recomputed at completion was      //
//                            $834 + Du. Du is written by the instruction   //
//                            just before, so port C also has to forward.   //
//   CAS.W D0,D4,$10(A5)      $8000 vs $0001: D0 = $FFFF8000, V             //
//   CAS.B D2,D3,$0870.W      $10 vs $C3: D2 = $00000010, C                 //
// Each CCR goes to $A00 + 2k by MOVE CCR,<ea>; A1 and A2 to $A10/$A14.     //
// Not equal writes nothing, as ap040_core.v's S_CAS4 does: writing the     //
// loaded value back leaves memory as it was, so the bench counts the       //
// write posts the L1 accepts at each CAS address -- one at each equal one, //
// none at the others.                                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_cas;

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
integer eq_posts = 0, ne_posts = 0;

always @(posedge clk) if (nreset && dut.l1_wren_b && !dut.l1_wr_busy) begin
	if (dut.l1_addr_b == 32'h800 || dut.l1_addr_b == 32'h824 || dut.l1_addr_b == 32'h844)
		eq_posts = eq_posts + 1;
	if (dut.l1_addr_b == 32'h810 || dut.l1_addr_b == 32'h860 || dut.l1_addr_b == 32'h870)
		ne_posts = ne_posts + 1;
end

task chk;
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

task chka;
	input [127:0] name;
	input [31:0] got;
	input [31:0] want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", name, got, want);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[  0] = 16'h46FC;  dut.u_l1.mem[  1] = 16'h2700;                               // $400 MOVE #$2700,SR (X clear; nothing below sets it)
	dut.u_l1.mem[  2] = 16'h207C;  dut.u_l1.mem[  3] = 16'h0000;  dut.u_l1.mem[  4] = 16'h0800; // $404 MOVEA.L #$800,A0
	dut.u_l1.mem[  5] = 16'h227C;  dut.u_l1.mem[  6] = 16'h0000;  dut.u_l1.mem[  7] = 16'h0810; // $40A MOVEA.L #$810,A1
	dut.u_l1.mem[  8] = 16'h247C;  dut.u_l1.mem[  9] = 16'h0000;  dut.u_l1.mem[ 10] = 16'h0828; // $410 MOVEA.L #$828,A2
	dut.u_l1.mem[ 11] = 16'h267C;  dut.u_l1.mem[ 12] = 16'h0000;  dut.u_l1.mem[ 13] = 16'h0830; // $416 MOVEA.L #$830,A3
	dut.u_l1.mem[ 14] = 16'h287C;  dut.u_l1.mem[ 15] = 16'h0000;  dut.u_l1.mem[ 16] = 16'h0010; // $41C MOVEA.L #$10,A4
	dut.u_l1.mem[ 17] = 16'h2A7C;  dut.u_l1.mem[ 18] = 16'h0000;  dut.u_l1.mem[ 19] = 16'h0850; // $422 MOVEA.L #$850,A5
	dut.u_l1.mem[ 20] = 16'h223C;  dut.u_l1.mem[ 21] = 16'h1234;  dut.u_l1.mem[ 22] = 16'h565A; // $428 MOVE.L #$1234565A,D1
	dut.u_l1.mem[ 23] = 16'h243C;  dut.u_l1.mem[ 24] = 16'h0000;  dut.u_l1.mem[ 25] = 16'h00C3; // $42E MOVE.L #$C3,D2
	dut.u_l1.mem[ 26] = 16'h0AD0;  dut.u_l1.mem[ 27] = 16'h0081;                               // $434 CAS.B D1,D2,(A0)            equal
	dut.u_l1.mem[ 28] = 16'h42F8;  dut.u_l1.mem[ 29] = 16'h0A00;                               // $438 MOVE CCR,$0A00.W
	dut.u_l1.mem[ 30] = 16'h263C;  dut.u_l1.mem[ 31] = 16'hAAAA;  dut.u_l1.mem[ 32] = 16'h0001; // $43C MOVE.L #$AAAA0001,D3
	dut.u_l1.mem[ 33] = 16'h283C;  dut.u_l1.mem[ 34] = 16'h0000;  dut.u_l1.mem[ 35] = 16'h7777; // $442 MOVE.L #$7777,D4
	dut.u_l1.mem[ 36] = 16'h0CD9;  dut.u_l1.mem[ 37] = 16'h0103;                               // $448 CAS.W D3,D4,(A1)+           not equal
	dut.u_l1.mem[ 38] = 16'h42F8;  dut.u_l1.mem[ 39] = 16'h0A02;                               // $44C MOVE CCR,$0A02.W
	dut.u_l1.mem[ 40] = 16'h2A3C;  dut.u_l1.mem[ 41] = 16'hCAFE;  dut.u_l1.mem[ 42] = 16'hBABE; // $450 MOVE.L #$CAFEBABE,D5
	dut.u_l1.mem[ 43] = 16'h2C3C;  dut.u_l1.mem[ 44] = 16'h0102;  dut.u_l1.mem[ 45] = 16'h0304; // $456 MOVE.L #$01020304,D6
	dut.u_l1.mem[ 46] = 16'h0EE2;  dut.u_l1.mem[ 47] = 16'h0185;                               // $45C CAS.L D5,D6,-(A2)           equal
	dut.u_l1.mem[ 48] = 16'h42F8;  dut.u_l1.mem[ 49] = 16'h0A04;                               // $460 MOVE CCR,$0A04.W
	dut.u_l1.mem[ 50] = 16'h203C;  dut.u_l1.mem[ 51] = 16'h5566;  dut.u_l1.mem[ 52] = 16'h7788; // $464 MOVE.L #$55667788,D0
	dut.u_l1.mem[ 53] = 16'h2E3C;  dut.u_l1.mem[ 54] = 16'h1122;  dut.u_l1.mem[ 55] = 16'h3344; // $46A MOVE.L #$11223344,D7        Du, written just before
	dut.u_l1.mem[ 56] = 16'h0EF3;  dut.u_l1.mem[ 57] = 16'h01C0;  dut.u_l1.mem[ 58] = 16'hC804; // $470 CAS.L D0,D7,(4,A3,A4.L)     equal
	dut.u_l1.mem[ 59] = 16'h42F8;  dut.u_l1.mem[ 60] = 16'h0A06;                               // $476 MOVE CCR,$0A06.W
	dut.u_l1.mem[ 61] = 16'h203C;  dut.u_l1.mem[ 62] = 16'hFFFF;  dut.u_l1.mem[ 63] = 16'h0001; // $47A MOVE.L #$FFFF0001,D0        Dc, written just before
	dut.u_l1.mem[ 64] = 16'h0CED;  dut.u_l1.mem[ 65] = 16'h0100;  dut.u_l1.mem[ 66] = 16'h0010; // $480 CAS.W D0,D4,$10(A5)         not equal, V
	dut.u_l1.mem[ 67] = 16'h42F8;  dut.u_l1.mem[ 68] = 16'h0A08;                               // $486 MOVE CCR,$0A08.W
	dut.u_l1.mem[ 69] = 16'h0AF8;  dut.u_l1.mem[ 70] = 16'h00C2;  dut.u_l1.mem[ 71] = 16'h0870; // $48A CAS.B D2,D3,$0870.W         not equal, C
	dut.u_l1.mem[ 72] = 16'h42F8;  dut.u_l1.mem[ 73] = 16'h0A0A;                               // $490 MOVE CCR,$0A0A.W
	dut.u_l1.mem[ 74] = 16'h21C9;  dut.u_l1.mem[ 75] = 16'h0A10;                               // $494 MOVE.L A1,$0A10.W
	dut.u_l1.mem[ 76] = 16'h21CA;  dut.u_l1.mem[ 77] = 16'h0A14;                               // $498 MOVE.L A2,$0A14.W
	dut.u_l1.mem[ 78] = 16'h60FE;                                                              // $49C BRA.B -2

	dut.u_l1.mem[512] = 16'h5A00;                                  // $800
	dut.u_l1.mem[520] = 16'h8001;                                  // $810
	dut.u_l1.mem[530] = 16'hCAFE;  dut.u_l1.mem[531] = 16'hBABE;   // $824
	dut.u_l1.mem[546] = 16'h5566;  dut.u_l1.mem[547] = 16'h7788;   // $844
	dut.u_l1.mem[560] = 16'h8000;                                  // $860
	dut.u_l1.mem[568] = 16'h1000;                                  // $870
	for (i = 768; i < 780; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00-$A17
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("CAS.B equal: $800",          512, 16'hC300);
	chk("CAS.W not equal: $810",      520, 16'h8001);
	chk("CAS.L equal: $824",          530, 16'h0102);  chk("CAS.L equal: $826", 531, 16'h0304);
	chk("CAS.L indexed: $844",        546, 16'h1122);  chk("CAS.L indexed: $846", 547, 16'h3344);
	chk("CAS.W not equal: $860",      560, 16'h8000);
	chk("CAS.B not equal: $870",      568, 16'h1000);
	chk("CCR CAS.B equal",            768, 16'h0004);
	chk("CCR CAS.W (A1)+",            769, 16'h0008);
	chk("CCR CAS.L -(A2)",            770, 16'h0004);
	chk("CCR CAS.L indexed",          771, 16'h0004);
	chk("CCR CAS.W V",                772, 16'h0002);
	chk("CCR CAS.B C",                773, 16'h0001);
	chk("A1 ($A10)",                  776, 16'h0000);  chk("A1 ($A12)", 777, 16'h0812);
	chk("A2 ($A14)",                  778, 16'h0000);  chk("A2 ($A16)", 779, 16'h0824);
	chka("D0", dbg_d0, 32'hFFFF_8000);
	chka("D1", dbg_d1, 32'h1234_565A);
	chka("D2", dbg_d2, 32'h0000_0010);
	chka("D3", dbg_d3, 32'hAAAA_8001);
	chka("D4", dbg_d4, 32'h0000_7777);
	chka("D5", dbg_d5, 32'hCAFE_BABE);
	chka("D6", dbg_d6, 32'h0102_0304);
	chka("D7", dbg_d7, 32'h1122_3344);
	chka("equal posts", eq_posts, 32'd3);
	chka("unequal posts", ne_posts, 32'd0);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
