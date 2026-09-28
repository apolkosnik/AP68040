//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: MOVE16, CAS2)  //
//                                                                          //
// tb_ap040_pipe_m16cas2.v - line moves and the double compare-and-swap     //
//                                                                          //
// Expected values come from an independent Python model of the rules in   //
// ap040_core.v's S_M16_* and S_CAS2_*, not from the pipelined RTL.         //
// MOVE16: both addresses aligned down to 16; the An steps by 16 from its  //
// own, unaligned value; (Ax)+,(Ay)+ with Ax = Ay steps once.               //
//   MOVE16 (A0)+,(A1)+    A0 $805, A1 $84A: $800 -> $840; $815, $85A      //
//   MOVE16 (A2)+,($8C0).L $860 -> $8C0; A2 $870                           //
//   MOVE16 ($880).L,(A3)+ -> $8E0 (A3 $8E3); A3 $8F3                       //
//   MOVE16 (A4),($900).L  A4 $8A8: $8A0 -> $900; A4 stays                  //
//   MOVE16 ($800).L,(A4)  $800 -> $8A0, over what was just copied out      //
//   MOVE16 (A5)+,(A5)+    $920 onto itself; A5 $920 -> $930, once          //
// CAS2: the first operand against Dc1, and only if equal the second       //
// against Dc2; the flags are the comparison that decided (X kept). Both   //
// equal stores Du1/Du2; otherwise Dc1 then Dc2 load, the second winning.  //
//   CAS2.L D0:D1,D2:D3,(A0):(A1)   equal twice: $980/$984 <- $AAAAAAAA,   //
//                                  $BBBBBBBB; CCR X Z                      //
//   CAS2.W D4:D5,D6:D7,(A2):(A3)   $0005 vs $0003: D4 $FFFF0005, D5      //
//                                  $FFFF0007, nothing stored; CCR X        //
//   CAS2.W D4:D4,D6:D7,(A4):(A5)   $1234 equal, $5678 not: D4 is Dc1 and  //
//                                  Dc2, and ends $00005678; CCR X          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_m16cas2;

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
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3;
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
	.dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;

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

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[  0] = 16'h46FC;  dut.u_l1.mem[  1] = 16'h2700;                               // $400 MOVE #$2700,SR
	dut.u_l1.mem[  2] = 16'h207C;  dut.u_l1.mem[  3] = 16'h0000;  dut.u_l1.mem[  4] = 16'h0805; // $404 MOVEA.L #$805,A0
	dut.u_l1.mem[  5] = 16'h227C;  dut.u_l1.mem[  6] = 16'h0000;  dut.u_l1.mem[  7] = 16'h084A; // $40A MOVEA.L #$84A,A1
	dut.u_l1.mem[  8] = 16'h247C;  dut.u_l1.mem[  9] = 16'h0000;  dut.u_l1.mem[ 10] = 16'h0860; // $410 MOVEA.L #$860,A2
	dut.u_l1.mem[ 11] = 16'h267C;  dut.u_l1.mem[ 12] = 16'h0000;  dut.u_l1.mem[ 13] = 16'h08E3; // $416 MOVEA.L #$8E3,A3
	dut.u_l1.mem[ 14] = 16'h287C;  dut.u_l1.mem[ 15] = 16'h0000;  dut.u_l1.mem[ 16] = 16'h08A8; // $41C MOVEA.L #$8A8,A4
	dut.u_l1.mem[ 17] = 16'h2A7C;  dut.u_l1.mem[ 18] = 16'h0000;  dut.u_l1.mem[ 19] = 16'h0920; // $422 MOVEA.L #$920,A5
	dut.u_l1.mem[ 20] = 16'hF620;  dut.u_l1.mem[ 21] = 16'h9000;                               // $428 MOVE16 (A0)+,(A1)+        $800 -> $840
	dut.u_l1.mem[ 22] = 16'hF602;  dut.u_l1.mem[ 23] = 16'h0000;  dut.u_l1.mem[ 24] = 16'h08C0; // $42C MOVE16 (A2)+,($8C0).L     $860 -> $8C0
	dut.u_l1.mem[ 25] = 16'hF60B;  dut.u_l1.mem[ 26] = 16'h0000;  dut.u_l1.mem[ 27] = 16'h0880; // $432 MOVE16 ($880).L,(A3)+     $880 -> $8E0
	dut.u_l1.mem[ 28] = 16'hF614;  dut.u_l1.mem[ 29] = 16'h0000;  dut.u_l1.mem[ 30] = 16'h0900; // $438 MOVE16 (A4),($900).L      $8A0 -> $900
	dut.u_l1.mem[ 31] = 16'hF61C;  dut.u_l1.mem[ 32] = 16'h0000;  dut.u_l1.mem[ 33] = 16'h0800; // $43E MOVE16 ($800).L,(A4)      $800 -> $8A0
	dut.u_l1.mem[ 34] = 16'hF625;  dut.u_l1.mem[ 35] = 16'hD000;                               // $444 MOVE16 (A5)+,(A5)+        $920 -> itself, one +16
	dut.u_l1.mem[ 36] = 16'h21C8;  dut.u_l1.mem[ 37] = 16'h0A10;                               // $448 MOVE.L A0,$0A10.W
	dut.u_l1.mem[ 38] = 16'h21C9;  dut.u_l1.mem[ 39] = 16'h0A14;                               // $44C MOVE.L A1,$0A14.W
	dut.u_l1.mem[ 40] = 16'h21CA;  dut.u_l1.mem[ 41] = 16'h0A18;                               // $450 MOVE.L A2,$0A18.W
	dut.u_l1.mem[ 42] = 16'h21CB;  dut.u_l1.mem[ 43] = 16'h0A1C;                               // $454 MOVE.L A3,$0A1C.W
	dut.u_l1.mem[ 44] = 16'h21CC;  dut.u_l1.mem[ 45] = 16'h0A20;                               // $458 MOVE.L A4,$0A20.W
	dut.u_l1.mem[ 46] = 16'h21CD;  dut.u_l1.mem[ 47] = 16'h0A24;                               // $45C MOVE.L A5,$0A24.W
	dut.u_l1.mem[ 48] = 16'h207C;  dut.u_l1.mem[ 49] = 16'h0000;  dut.u_l1.mem[ 50] = 16'h0980; // $460 MOVEA.L #$980,A0
	dut.u_l1.mem[ 51] = 16'h227C;  dut.u_l1.mem[ 52] = 16'h0000;  dut.u_l1.mem[ 53] = 16'h0984; // $466 MOVEA.L #$984,A1
	dut.u_l1.mem[ 54] = 16'h247C;  dut.u_l1.mem[ 55] = 16'h0000;  dut.u_l1.mem[ 56] = 16'h0990; // $46C MOVEA.L #$990,A2
	dut.u_l1.mem[ 57] = 16'h267C;  dut.u_l1.mem[ 58] = 16'h0000;  dut.u_l1.mem[ 59] = 16'h0994; // $472 MOVEA.L #$994,A3
	dut.u_l1.mem[ 60] = 16'h287C;  dut.u_l1.mem[ 61] = 16'h0000;  dut.u_l1.mem[ 62] = 16'h09A0; // $478 MOVEA.L #$9A0,A4
	dut.u_l1.mem[ 63] = 16'h2A7C;  dut.u_l1.mem[ 64] = 16'h0000;  dut.u_l1.mem[ 65] = 16'h09A4; // $47E MOVEA.L #$9A4,A5
	dut.u_l1.mem[ 66] = 16'h203C;  dut.u_l1.mem[ 67] = 16'h1111;  dut.u_l1.mem[ 68] = 16'h1111; // $484 MOVE.L #$11111111,D0
	dut.u_l1.mem[ 69] = 16'h223C;  dut.u_l1.mem[ 70] = 16'h2222;  dut.u_l1.mem[ 71] = 16'h2222; // $48A MOVE.L #$22222222,D1
	dut.u_l1.mem[ 72] = 16'h243C;  dut.u_l1.mem[ 73] = 16'hAAAA;  dut.u_l1.mem[ 74] = 16'hAAAA; // $490 MOVE.L #$AAAAAAAA,D2
	dut.u_l1.mem[ 75] = 16'h263C;  dut.u_l1.mem[ 76] = 16'hBBBB;  dut.u_l1.mem[ 77] = 16'hBBBB; // $496 MOVE.L #$BBBBBBBB,D3
	dut.u_l1.mem[ 78] = 16'h44FC;  dut.u_l1.mem[ 79] = 16'h0010;                               // $49C MOVE #$10,CCR
	dut.u_l1.mem[ 80] = 16'h0EFC;  dut.u_l1.mem[ 81] = 16'h8080;  dut.u_l1.mem[ 82] = 16'h90C1; // $4A0 CAS2.L D0:D1,D2:D3,(A0):(A1)  both equal
	dut.u_l1.mem[ 83] = 16'h42F8;  dut.u_l1.mem[ 84] = 16'h0A00;                               // $4A6 MOVE CCR,$0A00.W
	dut.u_l1.mem[ 85] = 16'h283C;  dut.u_l1.mem[ 86] = 16'hFFFF;  dut.u_l1.mem[ 87] = 16'h0003; // $4AA MOVE.L #$FFFF0003,D4
	dut.u_l1.mem[ 88] = 16'h2A3C;  dut.u_l1.mem[ 89] = 16'hFFFF;  dut.u_l1.mem[ 90] = 16'h0007; // $4B0 MOVE.L #$FFFF0007,D5
	dut.u_l1.mem[ 91] = 16'h2C3C;  dut.u_l1.mem[ 92] = 16'h6666;  dut.u_l1.mem[ 93] = 16'h6666; // $4B6 MOVE.L #$66666666,D6
	dut.u_l1.mem[ 94] = 16'h2E3C;  dut.u_l1.mem[ 95] = 16'h7777;  dut.u_l1.mem[ 96] = 16'h7777; // $4BC MOVE.L #$77777777,D7
	dut.u_l1.mem[ 97] = 16'h44FC;  dut.u_l1.mem[ 98] = 16'h0010;                               // $4C2 MOVE #$10,CCR
	dut.u_l1.mem[ 99] = 16'h0CFC;  dut.u_l1.mem[100] = 16'hA184;  dut.u_l1.mem[101] = 16'hB1C5; // $4C6 CAS2.W D4:D5,D6:D7,(A2):(A3)  first differs
	dut.u_l1.mem[102] = 16'h42F8;  dut.u_l1.mem[103] = 16'h0A02;                               // $4CC MOVE CCR,$0A02.W
	dut.u_l1.mem[104] = 16'h21C4;  dut.u_l1.mem[105] = 16'h0A30;                               // $4D0 MOVE.L D4,$0A30.W
	dut.u_l1.mem[106] = 16'h21C5;  dut.u_l1.mem[107] = 16'h0A34;                               // $4D4 MOVE.L D5,$0A34.W
	dut.u_l1.mem[108] = 16'h283C;  dut.u_l1.mem[109] = 16'h0000;  dut.u_l1.mem[110] = 16'h1234; // $4D8 MOVE.L #$1234,D4
	dut.u_l1.mem[111] = 16'h44FC;  dut.u_l1.mem[112] = 16'h0010;                               // $4DE MOVE #$10,CCR
	dut.u_l1.mem[113] = 16'h0CFC;  dut.u_l1.mem[114] = 16'hC184;  dut.u_l1.mem[115] = 16'hD1C4; // $4E2 CAS2.W D4:D4,D6:D7,(A4):(A5)  second differs, Dc1 = Dc2
	dut.u_l1.mem[116] = 16'h42F8;  dut.u_l1.mem[117] = 16'h0A04;                               // $4E8 MOVE CCR,$0A04.W
	dut.u_l1.mem[118] = 16'h21C4;  dut.u_l1.mem[119] = 16'h0A38;                               // $4EC MOVE.L D4,$0A38.W
	dut.u_l1.mem[120] = 16'h60FE;                                                              // $4F0 BRA.B -2

	dut.u_l1.mem[512] = 16'h1757;
	dut.u_l1.mem[513] = 16'h1858;
	dut.u_l1.mem[514] = 16'h1959;
	dut.u_l1.mem[515] = 16'h1A5A;
	dut.u_l1.mem[516] = 16'h1B5B;
	dut.u_l1.mem[517] = 16'h1C5C;
	dut.u_l1.mem[518] = 16'h1D5D;
	dut.u_l1.mem[519] = 16'h1E5E;
	dut.u_l1.mem[544] = 16'h0000;
	dut.u_l1.mem[545] = 16'h0000;
	dut.u_l1.mem[546] = 16'h0000;
	dut.u_l1.mem[547] = 16'h0000;
	dut.u_l1.mem[548] = 16'h0000;
	dut.u_l1.mem[549] = 16'h0000;
	dut.u_l1.mem[550] = 16'h0000;
	dut.u_l1.mem[551] = 16'h0000;
	dut.u_l1.mem[560] = 16'h4787;
	dut.u_l1.mem[561] = 16'h4888;
	dut.u_l1.mem[562] = 16'h4989;
	dut.u_l1.mem[563] = 16'h4A8A;
	dut.u_l1.mem[564] = 16'h4B8B;
	dut.u_l1.mem[565] = 16'h4C8C;
	dut.u_l1.mem[566] = 16'h4D8D;
	dut.u_l1.mem[567] = 16'h4E8E;
	dut.u_l1.mem[576] = 16'h5797;
	dut.u_l1.mem[577] = 16'h5898;
	dut.u_l1.mem[578] = 16'h5999;
	dut.u_l1.mem[579] = 16'h5A9A;
	dut.u_l1.mem[580] = 16'h5B9B;
	dut.u_l1.mem[581] = 16'h5C9C;
	dut.u_l1.mem[582] = 16'h5D9D;
	dut.u_l1.mem[583] = 16'h5E9E;
	dut.u_l1.mem[592] = 16'h67A7;
	dut.u_l1.mem[593] = 16'h68A8;
	dut.u_l1.mem[594] = 16'h69A9;
	dut.u_l1.mem[595] = 16'h6AAA;
	dut.u_l1.mem[596] = 16'h6BAB;
	dut.u_l1.mem[597] = 16'h6CAC;
	dut.u_l1.mem[598] = 16'h6DAD;
	dut.u_l1.mem[599] = 16'h6EAE;
	dut.u_l1.mem[608] = 16'h0000;
	dut.u_l1.mem[609] = 16'h0000;
	dut.u_l1.mem[610] = 16'h0000;
	dut.u_l1.mem[611] = 16'h0000;
	dut.u_l1.mem[612] = 16'h0000;
	dut.u_l1.mem[613] = 16'h0000;
	dut.u_l1.mem[614] = 16'h0000;
	dut.u_l1.mem[615] = 16'h0000;
	dut.u_l1.mem[624] = 16'h0000;
	dut.u_l1.mem[625] = 16'h0000;
	dut.u_l1.mem[626] = 16'h0000;
	dut.u_l1.mem[627] = 16'h0000;
	dut.u_l1.mem[628] = 16'h0000;
	dut.u_l1.mem[629] = 16'h0000;
	dut.u_l1.mem[630] = 16'h0000;
	dut.u_l1.mem[631] = 16'h0000;
	dut.u_l1.mem[640] = 16'h0000;
	dut.u_l1.mem[641] = 16'h0000;
	dut.u_l1.mem[642] = 16'h0000;
	dut.u_l1.mem[643] = 16'h0000;
	dut.u_l1.mem[644] = 16'h0000;
	dut.u_l1.mem[645] = 16'h0000;
	dut.u_l1.mem[646] = 16'h0000;
	dut.u_l1.mem[647] = 16'h0000;
	dut.u_l1.mem[656] = 16'hA7E7;
	dut.u_l1.mem[657] = 16'hA8E8;
	dut.u_l1.mem[658] = 16'hA9E9;
	dut.u_l1.mem[659] = 16'hAAEA;
	dut.u_l1.mem[660] = 16'hABEB;
	dut.u_l1.mem[661] = 16'hACEC;
	dut.u_l1.mem[662] = 16'hADED;
	dut.u_l1.mem[663] = 16'hAEEE;
	dut.u_l1.mem[704] = 16'h1111;  dut.u_l1.mem[705] = 16'h1111;   // $980
	dut.u_l1.mem[706] = 16'h2222;  dut.u_l1.mem[707] = 16'h2222;   // $984
	dut.u_l1.mem[712] = 16'h0005;                                  // $990
	dut.u_l1.mem[714] = 16'h0007;                                  // $994
	dut.u_l1.mem[720] = 16'h1234;                                  // $9A0
	dut.u_l1.mem[722] = 16'h5678;                                  // $9A4
	for (i = 768; i < 800; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00-$A3F
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 2400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("line $840", 544, 16'h1757);
	chk("line $842", 545, 16'h1858);
	chk("line $844", 546, 16'h1959);
	chk("line $846", 547, 16'h1A5A);
	chk("line $848", 548, 16'h1B5B);
	chk("line $84A", 549, 16'h1C5C);
	chk("line $84C", 550, 16'h1D5D);
	chk("line $84E", 551, 16'h1E5E);
	chk("line $8C0", 608, 16'h4787);
	chk("line $8C2", 609, 16'h4888);
	chk("line $8C4", 610, 16'h4989);
	chk("line $8C6", 611, 16'h4A8A);
	chk("line $8C8", 612, 16'h4B8B);
	chk("line $8CA", 613, 16'h4C8C);
	chk("line $8CC", 614, 16'h4D8D);
	chk("line $8CE", 615, 16'h4E8E);
	chk("line $8E0", 624, 16'h5797);
	chk("line $8E2", 625, 16'h5898);
	chk("line $8E4", 626, 16'h5999);
	chk("line $8E6", 627, 16'h5A9A);
	chk("line $8E8", 628, 16'h5B9B);
	chk("line $8EA", 629, 16'h5C9C);
	chk("line $8EC", 630, 16'h5D9D);
	chk("line $8EE", 631, 16'h5E9E);
	chk("line $900", 640, 16'h67A7);
	chk("line $902", 641, 16'h68A8);
	chk("line $904", 642, 16'h69A9);
	chk("line $906", 643, 16'h6AAA);
	chk("line $908", 644, 16'h6BAB);
	chk("line $90A", 645, 16'h6CAC);
	chk("line $90C", 646, 16'h6DAD);
	chk("line $90E", 647, 16'h6EAE);
	chk("line $8A0", 592, 16'h1757);
	chk("line $8A2", 593, 16'h1858);
	chk("line $8A4", 594, 16'h1959);
	chk("line $8A6", 595, 16'h1A5A);
	chk("line $8A8", 596, 16'h1B5B);
	chk("line $8AA", 597, 16'h1C5C);
	chk("line $8AC", 598, 16'h1D5D);
	chk("line $8AE", 599, 16'h1E5E);
	chk("line $920", 656, 16'hA7E7);
	chk("line $922", 657, 16'hA8E8);
	chk("line $924", 658, 16'hA9E9);
	chk("line $926", 659, 16'hAAEA);
	chk("line $928", 660, 16'hABEB);
	chk("line $92A", 661, 16'hACEC);
	chk("line $92C", 662, 16'hADED);
	chk("line $92E", 663, 16'hAEEE);
	chk("line $800", 512, 16'h1757);
	chk("line $802", 513, 16'h1858);
	chk("line $804", 514, 16'h1959);
	chk("line $806", 515, 16'h1A5A);
	chk("line $808", 516, 16'h1B5B);
	chk("line $80A", 517, 16'h1C5C);
	chk("line $80C", 518, 16'h1D5D);
	chk("line $80E", 519, 16'h1E5E);
	chk("A0 lo", 777, 16'h0815);  chk("A1 lo", 779, 16'h085A);
	chk("A2 lo", 781, 16'h0870);  chk("A3 lo", 783, 16'h08F3);
	chk("A4 lo", 785, 16'h08A8);  chk("A5 lo", 787, 16'h0930);
	chk("A0 hi", 776, 16'h0000);  chk("A5 hi", 786, 16'h0000);
	chk("CAS2.L $980", 704, 16'hAAAA);  chk("CAS2.L $982", 705, 16'hAAAA);
	chk("CAS2.L $984", 706, 16'hBBBB);  chk("CAS2.L $986", 707, 16'hBBBB);
	chk("CAS2.L CCR",  768, 16'h0014);
	chk("CAS2.W $990", 712, 16'h0005);  chk("CAS2.W $994", 714, 16'h0007);
	chk("CAS2.W CCR",  769, 16'h0010);
	chk("CAS2.W D4 hi", 792, 16'hFFFF); chk("CAS2.W D4 lo", 793, 16'h0005);
	chk("CAS2.W D5 hi", 794, 16'hFFFF); chk("CAS2.W D5 lo", 795, 16'h0007);
	chk("CAS2.W $9A0", 720, 16'h1234);  chk("CAS2.W $9A4", 722, 16'h5678);
	chk("CAS2.W Dc1=Dc2 CCR", 770, 16'h0010);
	chk("CAS2.W Dc1=Dc2 D4 hi", 796, 16'h0000); chk("CAS2.W Dc1=Dc2 D4 lo", 797, 16'h5678);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
