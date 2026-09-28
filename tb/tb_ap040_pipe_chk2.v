//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: CHK2/CMP2)     //
//                                                                          //
// tb_ap040_pipe_chk2.v - bounds checks in all sizes, Dn and An, CHK2 trap  //
//                                                                          //
// Every expected value below comes from an independent Python model of the //
// rule ap040_core.v's S_CHK2_D implements, not from the pipelined RTL: the //
// lower bound is at <ea>, the upper at <ea> + size, both sign-extended by  //
// size; a data register compares at the operand size, an address register  //
// compares all 32 bits; with lower <= upper Rn is out when below the lower //
// or above the upper, with the bounds reversed only when it is both. Z is  //
// Rn equal to either bound, C is out of bounds; X is left alone (set here  //
// once, before the first compare, and never cleared).                      //
//                                                                          //
//   CMP2.B (A0),D1/D2/D3       in, on the upper bound, above               //
//   CMP2.W 2(A1),D4            -128 in [-256, 256]: signed                 //
//   CMP2.W 6(A1),D5/D6         bounds reversed: 0 out, 512 in              //
//   CMP2.L (A2),A3             on the upper bound                          //
//   CMP2.W (A4),D7 / A5        $10000: the low word is the lower bound,    //
//                              the whole An is above the upper             //
//   CMP2.W (0,A6,D0.W),D4      port C is the index before it is anything   //
//                              else; reversed bounds at $816: out          //
//   CMP2.B $8F0(PC),D1         the PC base is the displacement word        //
//   CHK2.B (A0),D1             in bounds: no trap                          //
//   CHK2.B (A0),D3             out: vector 6, format 2, stacked SR $2711   //
//                              -- the frame goes to the stack, not to the  //
//                              bounds (the sequencer used to keep the port)//
//   CHK2.L (A2),A3             equal to the upper bound: no trap           //
//   CMP2.W 4(A4),A5 / 2(A1),A5 An = -8, -16, -300 against [-256, -16] and  //
//                              [-256, 256]: a bound left unextended moves  //
//                              every one of these (MOVEA sets A5 without   //
//                              touching the flags)                         //
// Each CCR goes to $A00 + 2k by MOVE CCR,<ea>. The vector-6 handler at     //
// $780 copies its frame to $A40 and counts itself at $A4C.                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_chk2;

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
	dut.u_l1.mem[  2] = 16'h203C;  dut.u_l1.mem[  3] = 16'h0000;  dut.u_l1.mem[  4] = 16'h1000; // $404 MOVE.L #$1000,D0
	dut.u_l1.mem[  5] = 16'h4E7B;  dut.u_l1.mem[  6] = 16'h0804;                               // $40A MOVEC D0,ISP
	dut.u_l1.mem[  7] = 16'h7016;                                                              // $40E MOVEQ #$16,D0
	dut.u_l1.mem[  8] = 16'h207C;  dut.u_l1.mem[  9] = 16'h0000;  dut.u_l1.mem[ 10] = 16'h0800; // $410 MOVEA.L #$800,A0
	dut.u_l1.mem[ 11] = 16'h227C;  dut.u_l1.mem[ 12] = 16'h0000;  dut.u_l1.mem[ 13] = 16'h0810; // $416 MOVEA.L #$810,A1
	dut.u_l1.mem[ 14] = 16'h247C;  dut.u_l1.mem[ 15] = 16'h0000;  dut.u_l1.mem[ 16] = 16'h0820; // $41C MOVEA.L #$820,A2
	dut.u_l1.mem[ 17] = 16'h267C;  dut.u_l1.mem[ 18] = 16'h0000;  dut.u_l1.mem[ 19] = 16'h2000; // $422 MOVEA.L #$2000,A3
	dut.u_l1.mem[ 20] = 16'h287C;  dut.u_l1.mem[ 21] = 16'h0000;  dut.u_l1.mem[ 22] = 16'h0828; // $428 MOVEA.L #$828,A4
	dut.u_l1.mem[ 23] = 16'h2A7C;  dut.u_l1.mem[ 24] = 16'h0001;  dut.u_l1.mem[ 25] = 16'h0000; // $42E MOVEA.L #$10000,A5
	dut.u_l1.mem[ 26] = 16'h2C7C;  dut.u_l1.mem[ 27] = 16'h0000;  dut.u_l1.mem[ 28] = 16'h0800; // $434 MOVEA.L #$800,A6
	dut.u_l1.mem[ 29] = 16'h223C;  dut.u_l1.mem[ 30] = 16'hFFFF;  dut.u_l1.mem[ 31] = 16'hFF30; // $43A MOVE.L #$FFFFFF30,D1
	dut.u_l1.mem[ 32] = 16'h243C;  dut.u_l1.mem[ 33] = 16'h1234;  dut.u_l1.mem[ 34] = 16'h5650; // $440 MOVE.L #$12345650,D2
	dut.u_l1.mem[ 35] = 16'h7660;                                                              // $446 MOVEQ #$60,D3
	dut.u_l1.mem[ 36] = 16'h283C;  dut.u_l1.mem[ 37] = 16'h0000;  dut.u_l1.mem[ 38] = 16'hFF80; // $448 MOVE.L #$FF80,D4
	dut.u_l1.mem[ 39] = 16'h7A00;                                                              // $44E MOVEQ #0,D5
	dut.u_l1.mem[ 40] = 16'h2C3C;  dut.u_l1.mem[ 41] = 16'h0000;  dut.u_l1.mem[ 42] = 16'h0200; // $450 MOVE.L #$200,D6
	dut.u_l1.mem[ 43] = 16'h2E3C;  dut.u_l1.mem[ 44] = 16'h0001;  dut.u_l1.mem[ 45] = 16'h0000; // $456 MOVE.L #$10000,D7
	dut.u_l1.mem[ 46] = 16'h44FC;  dut.u_l1.mem[ 47] = 16'h0010;                               // $45C MOVE #$10,CCR (X: CMP2 must leave it)
	dut.u_l1.mem[ 48] = 16'h00D0;  dut.u_l1.mem[ 49] = 16'h1000;                               // $460 CMP2.B (A0),D1
	dut.u_l1.mem[ 50] = 16'h42F8;  dut.u_l1.mem[ 51] = 16'h0A00;                               // $464 MOVE CCR,$0A00.W
	dut.u_l1.mem[ 52] = 16'h00D0;  dut.u_l1.mem[ 53] = 16'h2000;                               // $468 CMP2.B (A0),D2
	dut.u_l1.mem[ 54] = 16'h42F8;  dut.u_l1.mem[ 55] = 16'h0A02;                               // $46C MOVE CCR,$0A02.W
	dut.u_l1.mem[ 56] = 16'h00D0;  dut.u_l1.mem[ 57] = 16'h3000;                               // $470 CMP2.B (A0),D3
	dut.u_l1.mem[ 58] = 16'h42F8;  dut.u_l1.mem[ 59] = 16'h0A04;                               // $474 MOVE CCR,$0A04.W
	dut.u_l1.mem[ 60] = 16'h02E9;  dut.u_l1.mem[ 61] = 16'h4000;  dut.u_l1.mem[ 62] = 16'h0002; // $478 CMP2.W 2(A1),D4
	dut.u_l1.mem[ 63] = 16'h42F8;  dut.u_l1.mem[ 64] = 16'h0A06;                               // $47E MOVE CCR,$0A06.W
	dut.u_l1.mem[ 65] = 16'h02E9;  dut.u_l1.mem[ 66] = 16'h5000;  dut.u_l1.mem[ 67] = 16'h0006; // $482 CMP2.W 6(A1),D5            reversed bounds
	dut.u_l1.mem[ 68] = 16'h42F8;  dut.u_l1.mem[ 69] = 16'h0A08;                               // $488 MOVE CCR,$0A08.W
	dut.u_l1.mem[ 70] = 16'h02E9;  dut.u_l1.mem[ 71] = 16'h6000;  dut.u_l1.mem[ 72] = 16'h0006; // $48C CMP2.W 6(A1),D6            reversed bounds
	dut.u_l1.mem[ 73] = 16'h42F8;  dut.u_l1.mem[ 74] = 16'h0A0A;                               // $492 MOVE CCR,$0A0A.W
	dut.u_l1.mem[ 75] = 16'h04D2;  dut.u_l1.mem[ 76] = 16'hB000;                               // $496 CMP2.L (A2),A3
	dut.u_l1.mem[ 77] = 16'h42F8;  dut.u_l1.mem[ 78] = 16'h0A0C;                               // $49A MOVE CCR,$0A0C.W
	dut.u_l1.mem[ 79] = 16'h02D4;  dut.u_l1.mem[ 80] = 16'h7000;                               // $49E CMP2.W (A4),D7             the low word
	dut.u_l1.mem[ 81] = 16'h42F8;  dut.u_l1.mem[ 82] = 16'h0A0E;                               // $4A2 MOVE CCR,$0A0E.W
	dut.u_l1.mem[ 83] = 16'h02D4;  dut.u_l1.mem[ 84] = 16'hD000;                               // $4A6 CMP2.W (A4),A5             all 32 bits
	dut.u_l1.mem[ 85] = 16'h42F8;  dut.u_l1.mem[ 86] = 16'h0A10;                               // $4AA MOVE CCR,$0A10.W
	dut.u_l1.mem[ 87] = 16'h02F6;  dut.u_l1.mem[ 88] = 16'h4000;  dut.u_l1.mem[ 89] = 16'h0000; // $4AE CMP2.W (0,A6,D0.W),D4      EA $816
	dut.u_l1.mem[ 90] = 16'h42F8;  dut.u_l1.mem[ 91] = 16'h0A12;                               // $4B4 MOVE CCR,$0A12.W
	dut.u_l1.mem[ 92] = 16'h00FA;  dut.u_l1.mem[ 93] = 16'h1000;  dut.u_l1.mem[ 94] = 16'h0434; // $4B8 CMP2.B $8F0(PC),D1
	dut.u_l1.mem[ 95] = 16'h42F8;  dut.u_l1.mem[ 96] = 16'h0A14;                               // $4BE MOVE CCR,$0A14.W
	dut.u_l1.mem[ 97] = 16'h00D0;  dut.u_l1.mem[ 98] = 16'h1800;                               // $4C2 CHK2.B (A0),D1             in bounds
	dut.u_l1.mem[ 99] = 16'h42F8;  dut.u_l1.mem[100] = 16'h0A16;                               // $4C6 MOVE CCR,$0A16.W
	dut.u_l1.mem[101] = 16'h00D0;  dut.u_l1.mem[102] = 16'h3800;                               // $4CA CHK2.B (A0),D3             out: vector 6
	dut.u_l1.mem[103] = 16'h42F8;  dut.u_l1.mem[104] = 16'h0A18;                               // $4CE MOVE CCR,$0A18.W
	dut.u_l1.mem[105] = 16'h04D2;  dut.u_l1.mem[106] = 16'hB800;                               // $4D2 CHK2.L (A2),A3             equal: no trap
	dut.u_l1.mem[107] = 16'h42F8;  dut.u_l1.mem[108] = 16'h0A1A;                               // $4D6 MOVE CCR,$0A1A.W
	dut.u_l1.mem[109] = 16'h3A7C;  dut.u_l1.mem[110] = 16'hFFF8;                               // $4DA MOVEA.W #$FFF8,A5          flags untouched
	dut.u_l1.mem[111] = 16'h02EC;  dut.u_l1.mem[112] = 16'hD000;  dut.u_l1.mem[113] = 16'h0004; // $4DE CMP2.W 4(A4),A5            -8 above [-256, -16]
	dut.u_l1.mem[114] = 16'h42F8;  dut.u_l1.mem[115] = 16'h0A1C;                               // $4E4 MOVE CCR,$0A1C.W
	dut.u_l1.mem[116] = 16'h3A7C;  dut.u_l1.mem[117] = 16'hFFF0;                               // $4E8 MOVEA.W #$FFF0,A5
	dut.u_l1.mem[118] = 16'h02EC;  dut.u_l1.mem[119] = 16'hD000;  dut.u_l1.mem[120] = 16'h0004; // $4EC CMP2.W 4(A4),A5            -16: the upper bound
	dut.u_l1.mem[121] = 16'h42F8;  dut.u_l1.mem[122] = 16'h0A1E;                               // $4F2 MOVE CCR,$0A1E.W
	dut.u_l1.mem[123] = 16'h3A7C;  dut.u_l1.mem[124] = 16'hFED4;                               // $4F6 MOVEA.W #$FED4,A5
	dut.u_l1.mem[125] = 16'h02E9;  dut.u_l1.mem[126] = 16'hD000;  dut.u_l1.mem[127] = 16'h0002; // $4FA CMP2.W 2(A1),A5            -300 below [-256, 256]
	dut.u_l1.mem[128] = 16'h42F8;  dut.u_l1.mem[129] = 16'h0A20;                               // $500 MOVE CCR,$0A20.W
	dut.u_l1.mem[130] = 16'h60FE;                                                              // $504 BRA.B -2

	// Bounds.
	dut.u_l1.mem[512] = 16'h1050;                                  // $800: .B $10, $50
	dut.u_l1.mem[521] = 16'hFF00;  dut.u_l1.mem[522] = 16'h0100;   // $812: .W -256, 256
	dut.u_l1.mem[523] = 16'h0100;  dut.u_l1.mem[524] = 16'hFF00;   // $816: .W 256, -256
	dut.u_l1.mem[528] = 16'h0000;  dut.u_l1.mem[529] = 16'h1000;   // $820: .L $1000
	dut.u_l1.mem[530] = 16'h0000;  dut.u_l1.mem[531] = 16'h2000;   //       .L $2000
	dut.u_l1.mem[532] = 16'h0000;  dut.u_l1.mem[533] = 16'h7FFF;   // $828: .W 0, $7FFF
	dut.u_l1.mem[534] = 16'hFF00;  dut.u_l1.mem[535] = 16'hFFF0;   // $82C: .W -256, -16
	dut.u_l1.mem[632] = 16'h3140;                                  // $8F0: .B $31, $40
	for (i = 768; i < 810; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00-$A53

	// $780: vector 6.
	dut.u_l1.mem[448] = 16'h31D7;  dut.u_l1.mem[449] = 16'h0A40;                               // $780 MOVE.W (A7),$0A40.W
	dut.u_l1.mem[450] = 16'h21EF;  dut.u_l1.mem[451] = 16'h0002;  dut.u_l1.mem[452] = 16'h0A42; // $784 MOVE.L 2(A7),$0A42.W
	dut.u_l1.mem[453] = 16'h31EF;  dut.u_l1.mem[454] = 16'h0006;  dut.u_l1.mem[455] = 16'h0A46; // $78A MOVE.W 6(A7),$0A46.W
	dut.u_l1.mem[456] = 16'h21EF;  dut.u_l1.mem[457] = 16'h0008;  dut.u_l1.mem[458] = 16'h0A48; // $790 MOVE.L 8(A7),$0A48.W
	dut.u_l1.mem[459] = 16'h5278;  dut.u_l1.mem[460] = 16'h0A4C;                               // $796 ADDQ.W #1,$0A4C.W
	dut.u_l1.mem[461] = 16'h4E73;                                                              // $79A RTE

	// Vector 6 -> $780.
	dut.u_l1.mem[3596] = 16'h0000;  dut.u_l1.mem[3597] = 16'h0780;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("CCR CMP2.B D1",           768, 16'h0010);
	chk("CCR CMP2.B D2",           769, 16'h0014);
	chk("CCR CMP2.B D3",           770, 16'h0011);
	chk("CCR CMP2.W D4",           771, 16'h0010);
	chk("CCR CMP2.W D5 rev",       772, 16'h0011);
	chk("CCR CMP2.W D6 rev",       773, 16'h0010);
	chk("CCR CMP2.L A3",           774, 16'h0014);
	chk("CCR CMP2.W D7",           775, 16'h0014);
	chk("CCR CMP2.W A5",           776, 16'h0011);
	chk("CCR CMP2.W idx D4",       777, 16'h0011);
	chk("CCR CMP2.B pcrel D1",     778, 16'h0011);
	chk("CCR CHK2.B D1",           779, 16'h0010);
	chk("CCR CHK2.B D3 (after RTE)", 780, 16'h0011);
	chk("CCR CHK2.L A3",           781, 16'h0014);
	chk("CCR CMP2.W A5 -8",        782, 16'h0011);
	chk("CCR CMP2.W A5 -16",       783, 16'h0014);
	chk("CCR CMP2.W A5 -300",      784, 16'h0011);
	chk("handler entries",          806, 16'h0001);
	chk("CHK2 frame SR",            800, 16'h2711);
	chk("CHK2 frame PC hi",         801, 16'h0000);  chk("CHK2 frame PC lo", 802, 16'h04CE);
	chk("CHK2 frame format",        803, 16'h2018);
	chk("CHK2 frame IA hi",         804, 16'h0000);  chk("CHK2 frame IA lo", 805, 16'h04CA);
	chk("stack $FF4 (SR)",         1530, 16'h2711);
	chk("stack $FFA (format)",     1533, 16'h2018);
	chk("bounds $800 untouched",    512, 16'h1050);
	chk("bounds $802 untouched",    513, 16'h4E71);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
