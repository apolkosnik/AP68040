//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: the memory    //
// forms left)                                                              //
//                                                                          //
// tb_ap040_pipe_memx.v - two loads, two sizes, four extension words and   //
// three registers                                                          //
//                                                                          //
// Every expected value comes from an independent Python model of the 68k  //
// rules, not from the RTL.                                                 //
//   ADDX.L -(A0),-(A1)   $7FFFFFFF + $00000001 + X: $80000001, N V; Z was //
//                        set and is cleared                                //
//   SUBX.W -(A2),-(A3)   $0000 - $0001 - X: $FFFE, X N C                   //
//   ABCD -(A4),-(A4)     $19 + $28 = $47 at $84E; the source is $84F, the //
//                        one register stepped twice                        //
//   SBCD -(A7),-(A5)     $00 - $01 = $99, X C; A7 steps two for a byte    //
//   CMPM.W (A0)+,(A1)+   $8000 - $7FFF: V                                  //
//   CMPM.B (A2)+,(A2)+   $01 - $00; A2 ends two bytes on                   //
//   PACK -(A0),-(A1),#$1111   $0305 -> $46; a word read, a byte written   //
//   UNPK -(A2),-(A3),#$3030   $47 -> $3437; a byte read, a word written   //
//   PACK -(A4),-(A4),#0       $0102 -> $12 at $8BD                        //
//   ORI.L / CMPI.L #imm32,(xxx).L   four extension words: the immediate's //
//                        high half is the first of them                    //
//   MULU.L (A0)+,D1:D2   $10000 * $30000: D1 = 3, D2 = 0, A0 += 4         //
//   DIVU.L -(A1),D3:D4   100 / 7: D4 = 14, D3 = 2, A1 -= 4                //
//   DIVS.L (A7)+,D5:D6   64-bit 100 / -3: D6 = -33, D5 = 1, A7 += 4       //
// The last three write three registers each: EX writes the An step early. //
//   DIVS.L (A7)+,D5:D6 again, in user mode, straight behind MOVE #0,SR:  //
//                        100 / 2; the step lands in USP, $17FC -> $1800 //
// A LEA reads that An straight behind each, and every An is saved.        //
// ABCD/SBCD leave N and V undefined, so their CCRs are checked on X Z C.  //
// Then, in user mode, ADDX.L/SUBX.W/ABCD/CMPM.W/ADDX.L/ADDX.L/CMPM.L with //
// the destination An from the instruction straight ahead, two ahead, one //
// An for both, (Ax)+ straight ahead, a load straight ahead, DIVU.L        //
// (A1)+,D3:D4 and MULU.L (A2)+,D1:D2 straight ahead (the step EX writes   //
// early): the destination address is latched at the first load's issue. //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_memx;

localparam PROG_WORDS      = 600;
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
wire [31:0] dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6;
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
	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6),
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

task chkm;   // masked
	input [255:0] what;
	input integer widx;
	input  [15:0] mask;
	input  [15:0] want;
	begin
		if ((dut.u_l1.mem[widx] & mask) !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %04x (mask %04x), expected %04x", what, dut.u_l1.mem[widx], mask, want);
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

	dut.u_l1.mem[  0] = 16'h46FC;  dut.u_l1.mem[  1] = 16'h2700;                               // $400 MOVE #$2700,SR
	dut.u_l1.mem[  2] = 16'h203C;  dut.u_l1.mem[  3] = 16'h0000;  dut.u_l1.mem[  4] = 16'h1000; // $404 MOVE.L #$1000,D0
	dut.u_l1.mem[  5] = 16'h4E7B;  dut.u_l1.mem[  6] = 16'h0804;                               // $40A MOVEC D0,ISP
	dut.u_l1.mem[  7] = 16'h207C;  dut.u_l1.mem[  8] = 16'h0000;  dut.u_l1.mem[  9] = 16'h0810; // $40E MOVEA.L #$810,A0
	dut.u_l1.mem[ 10] = 16'h227C;  dut.u_l1.mem[ 11] = 16'h0000;  dut.u_l1.mem[ 12] = 16'h0820; // $414 MOVEA.L #$820,A1
	dut.u_l1.mem[ 13] = 16'h247C;  dut.u_l1.mem[ 14] = 16'h0000;  dut.u_l1.mem[ 15] = 16'h0830; // $41A MOVEA.L #$830,A2
	dut.u_l1.mem[ 16] = 16'h267C;  dut.u_l1.mem[ 17] = 16'h0000;  dut.u_l1.mem[ 18] = 16'h0840; // $420 MOVEA.L #$840,A3
	dut.u_l1.mem[ 19] = 16'h287C;  dut.u_l1.mem[ 20] = 16'h0000;  dut.u_l1.mem[ 21] = 16'h0850; // $426 MOVEA.L #$850,A4
	dut.u_l1.mem[ 22] = 16'h2A7C;  dut.u_l1.mem[ 23] = 16'h0000;  dut.u_l1.mem[ 24] = 16'h0860; // $42C MOVEA.L #$860,A5
	dut.u_l1.mem[ 25] = 16'h44FC;  dut.u_l1.mem[ 26] = 16'h0014;                               // $432 MOVE #$14,CCR
	dut.u_l1.mem[ 27] = 16'hD388;                                                              // $436 ADDX.L -(A0),-(A1)
	dut.u_l1.mem[ 28] = 16'h42F8;  dut.u_l1.mem[ 29] = 16'h0A00;                               // $438 MOVE CCR,$0A00.W
	dut.u_l1.mem[ 30] = 16'h44FC;  dut.u_l1.mem[ 31] = 16'h0010;                               // $43C MOVE #$10,CCR
	dut.u_l1.mem[ 32] = 16'h974A;                                                              // $440 SUBX.W -(A2),-(A3)
	dut.u_l1.mem[ 33] = 16'h42F8;  dut.u_l1.mem[ 34] = 16'h0A02;                               // $442 MOVE CCR,$0A02.W
	dut.u_l1.mem[ 35] = 16'h44FC;  dut.u_l1.mem[ 36] = 16'h0004;                               // $446 MOVE #$04,CCR
	dut.u_l1.mem[ 37] = 16'hC90C;                                                              // $44A ABCD -(A4),-(A4)          one register, two steps
	dut.u_l1.mem[ 38] = 16'h42F8;  dut.u_l1.mem[ 39] = 16'h0A04;                               // $44C MOVE CCR,$0A04.W
	dut.u_l1.mem[ 40] = 16'h44FC;  dut.u_l1.mem[ 41] = 16'h0000;                               // $450 MOVE #$00,CCR
	dut.u_l1.mem[ 42] = 16'h8B0F;                                                              // $454 SBCD -(A7),-(A5)          A7 steps two for a byte
	dut.u_l1.mem[ 43] = 16'h42F8;  dut.u_l1.mem[ 44] = 16'h0A06;                               // $456 MOVE CCR,$0A06.W
	dut.u_l1.mem[ 45] = 16'h21CF;  dut.u_l1.mem[ 46] = 16'h0A30;                               // $45A MOVE.L A7,$0A30.W
	dut.u_l1.mem[ 47] = 16'h21CD;  dut.u_l1.mem[ 48] = 16'h0A34;                               // $45E MOVE.L A5,$0A34.W
	dut.u_l1.mem[ 49] = 16'h2E7C;  dut.u_l1.mem[ 50] = 16'h0000;  dut.u_l1.mem[ 51] = 16'h1000; // $462 MOVEA.L #$1000,A7
	dut.u_l1.mem[ 52] = 16'h44FC;  dut.u_l1.mem[ 53] = 16'h0000;                               // $468 MOVE #$00,CCR
	dut.u_l1.mem[ 54] = 16'hB348;                                                              // $46C CMPM.W (A0)+,(A1)+
	dut.u_l1.mem[ 55] = 16'h42F8;  dut.u_l1.mem[ 56] = 16'h0A08;                               // $46E MOVE CCR,$0A08.W
	dut.u_l1.mem[ 57] = 16'hB50A;                                                              // $472 CMPM.B (A2)+,(A2)+        one register, two steps
	dut.u_l1.mem[ 58] = 16'h42F8;  dut.u_l1.mem[ 59] = 16'h0A0A;                               // $474 MOVE CCR,$0A0A.W
	dut.u_l1.mem[ 60] = 16'h21C8;  dut.u_l1.mem[ 61] = 16'h0A38;                               // $478 MOVE.L A0,$0A38.W
	dut.u_l1.mem[ 62] = 16'h21C9;  dut.u_l1.mem[ 63] = 16'h0A3C;                               // $47C MOVE.L A1,$0A3C.W
	dut.u_l1.mem[ 64] = 16'h21CA;  dut.u_l1.mem[ 65] = 16'h0A40;                               // $480 MOVE.L A2,$0A40.W
	dut.u_l1.mem[ 66] = 16'h21CB;  dut.u_l1.mem[ 67] = 16'h0A44;                               // $484 MOVE.L A3,$0A44.W
	dut.u_l1.mem[ 68] = 16'h21CC;  dut.u_l1.mem[ 69] = 16'h0A48;                               // $488 MOVE.L A4,$0A48.W
	dut.u_l1.mem[ 70] = 16'h207C;  dut.u_l1.mem[ 71] = 16'h0000;  dut.u_l1.mem[ 72] = 16'h0880; // $48C MOVEA.L #$880,A0
	dut.u_l1.mem[ 73] = 16'h227C;  dut.u_l1.mem[ 74] = 16'h0000;  dut.u_l1.mem[ 75] = 16'h0890; // $492 MOVEA.L #$890,A1
	dut.u_l1.mem[ 76] = 16'h247C;  dut.u_l1.mem[ 77] = 16'h0000;  dut.u_l1.mem[ 78] = 16'h08A0; // $498 MOVEA.L #$8A0,A2
	dut.u_l1.mem[ 79] = 16'h267C;  dut.u_l1.mem[ 80] = 16'h0000;  dut.u_l1.mem[ 81] = 16'h08B0; // $49E MOVEA.L #$8B0,A3
	dut.u_l1.mem[ 82] = 16'h287C;  dut.u_l1.mem[ 83] = 16'h0000;  dut.u_l1.mem[ 84] = 16'h08C0; // $4A4 MOVEA.L #$8C0,A4
	dut.u_l1.mem[ 85] = 16'h8348;  dut.u_l1.mem[ 86] = 16'h1111;                               // $4AA PACK -(A0),-(A1),#$1111
	dut.u_l1.mem[ 87] = 16'h878A;  dut.u_l1.mem[ 88] = 16'h3030;                               // $4AE UNPK -(A2),-(A3),#$3030
	dut.u_l1.mem[ 89] = 16'h894C;  dut.u_l1.mem[ 90] = 16'h0000;                               // $4B2 PACK -(A4),-(A4),#0
	dut.u_l1.mem[ 91] = 16'h21C8;  dut.u_l1.mem[ 92] = 16'h0A4C;                               // $4B6 MOVE.L A0,$0A4C.W
	dut.u_l1.mem[ 93] = 16'h21C9;  dut.u_l1.mem[ 94] = 16'h0A50;                               // $4BA MOVE.L A1,$0A50.W
	dut.u_l1.mem[ 95] = 16'h21CA;  dut.u_l1.mem[ 96] = 16'h0A54;                               // $4BE MOVE.L A2,$0A54.W
	dut.u_l1.mem[ 97] = 16'h21CB;  dut.u_l1.mem[ 98] = 16'h0A58;                               // $4C2 MOVE.L A3,$0A58.W
	dut.u_l1.mem[ 99] = 16'h21CC;  dut.u_l1.mem[100] = 16'h0A5C;                               // $4C6 MOVE.L A4,$0A5C.W
	dut.u_l1.mem[101] = 16'h44FC;  dut.u_l1.mem[102] = 16'h0010;                               // $4CA MOVE #$10,CCR
	dut.u_l1.mem[103] = 16'h00B9;  dut.u_l1.mem[104] = 16'hF0F0;  dut.u_l1.mem[105] = 16'hF0F0;  dut.u_l1.mem[106] = 16'h0000;  dut.u_l1.mem[107] = 16'h08D0; // $4CE ORI.L #$F0F0F0F0,($8D0).L
	dut.u_l1.mem[108] = 16'h42F8;  dut.u_l1.mem[109] = 16'h0A0C;                               // $4D8 MOVE CCR,$0A0C.W
	dut.u_l1.mem[110] = 16'h0CB9;  dut.u_l1.mem[111] = 16'h1234;  dut.u_l1.mem[112] = 16'h5678;  dut.u_l1.mem[113] = 16'h0000;  dut.u_l1.mem[114] = 16'h08D4; // $4DC CMPI.L #$12345678,($8D4).L
	dut.u_l1.mem[115] = 16'h42F8;  dut.u_l1.mem[116] = 16'h0A0E;                               // $4E6 MOVE CCR,$0A0E.W
	dut.u_l1.mem[117] = 16'h207C;  dut.u_l1.mem[118] = 16'h0000;  dut.u_l1.mem[119] = 16'h08E0; // $4EA MOVEA.L #$8E0,A0
	dut.u_l1.mem[120] = 16'h243C;  dut.u_l1.mem[121] = 16'h0003;  dut.u_l1.mem[122] = 16'h0000; // $4F0 MOVE.L #$30000,D2
	dut.u_l1.mem[123] = 16'h4C18;  dut.u_l1.mem[124] = 16'h2401;                               // $4F6 MULU.L (A0)+,D1:D2
	dut.u_l1.mem[125] = 16'h4DD0;                                                              // $4FA LEA (A0),A6               A0 right behind
	dut.u_l1.mem[126] = 16'h42F8;  dut.u_l1.mem[127] = 16'h0A10;                               // $4FC MOVE CCR,$0A10.W
	dut.u_l1.mem[128] = 16'h21CE;  dut.u_l1.mem[129] = 16'h0A60;                               // $500 MOVE.L A6,$0A60.W
	dut.u_l1.mem[130] = 16'h227C;  dut.u_l1.mem[131] = 16'h0000;  dut.u_l1.mem[132] = 16'h08F4; // $504 MOVEA.L #$8F4,A1
	dut.u_l1.mem[133] = 16'h7864;                                                              // $50A MOVEQ #100,D4
	dut.u_l1.mem[134] = 16'h4C61;  dut.u_l1.mem[135] = 16'h4003;                               // $50C DIVU.L -(A1),D3:D4
	dut.u_l1.mem[136] = 16'h4DD1;                                                              // $510 LEA (A1),A6
	dut.u_l1.mem[137] = 16'h42F8;  dut.u_l1.mem[138] = 16'h0A12;                               // $512 MOVE CCR,$0A12.W
	dut.u_l1.mem[139] = 16'h21CE;  dut.u_l1.mem[140] = 16'h0A64;                               // $516 MOVE.L A6,$0A64.W
	dut.u_l1.mem[141] = 16'h2F3C;  dut.u_l1.mem[142] = 16'hFFFF;  dut.u_l1.mem[143] = 16'hFFFD; // $51A MOVE.L #-3,-(A7)
	dut.u_l1.mem[144] = 16'h7A00;                                                              // $520 MOVEQ #0,D5
	dut.u_l1.mem[145] = 16'h7C64;                                                              // $522 MOVEQ #100,D6
	dut.u_l1.mem[146] = 16'h4C5F;  dut.u_l1.mem[147] = 16'h6C05;                               // $524 DIVS.L (A7)+,D5:D6        64-bit, A7
	dut.u_l1.mem[148] = 16'h4DD7;                                                              // $528 LEA (A7),A6
	dut.u_l1.mem[149] = 16'h42F8;  dut.u_l1.mem[150] = 16'h0A14;                               // $52A MOVE CCR,$0A14.W
	dut.u_l1.mem[151] = 16'h21CE;  dut.u_l1.mem[152] = 16'h0A68;                               // $52E MOVE.L A6,$0A68.W
	dut.u_l1.mem[153] = 16'h21C5;  dut.u_l1.mem[154] = 16'h0A70;                               // $532 MOVE.L D5,$0A70.W
	dut.u_l1.mem[155] = 16'h21C6;  dut.u_l1.mem[156] = 16'h0A74;                               // $536 MOVE.L D6,$0A74.W
	dut.u_l1.mem[157] = 16'h21FC;  dut.u_l1.mem[158] = 16'h0000;  dut.u_l1.mem[159] = 16'h0002;  dut.u_l1.mem[160] = 16'h17FC; // $53A MOVE.L #2,$17FC.W
	dut.u_l1.mem[161] = 16'h207C;  dut.u_l1.mem[162] = 16'h0000;  dut.u_l1.mem[163] = 16'h17FC; // $542 MOVEA.L #$17FC,A0
	dut.u_l1.mem[164] = 16'h4E60;                                                              // $548 MOVE A0,USP
	dut.u_l1.mem[165] = 16'h7A00;                                                              // $54A MOVEQ #0,D5
	dut.u_l1.mem[166] = 16'h7C64;                                                              // $54C MOVEQ #100,D6
	dut.u_l1.mem[167] = 16'h46FC;  dut.u_l1.mem[168] = 16'h0000;                               // $54E MOVE #0,SR                user: A7 is USP
	dut.u_l1.mem[169] = 16'h4C5F;  dut.u_l1.mem[170] = 16'h6C05;                               // $552 DIVS.L (A7)+,D5:D6        USP, straight behind
	dut.u_l1.mem[171] = 16'h4DD7;                                                              // $556 LEA (A7),A6
	dut.u_l1.mem[172] = 16'h21CE;  dut.u_l1.mem[173] = 16'h0A6C;                               // $558 MOVE.L A6,$0A6C.W
	// Every two-load form again, its destination An produced by the
	// instruction straight ahead (EX's forward), two ahead, by a load, and by
	// a three-register divide and multiply whose An step EX writes early.
	// The destination address is latched when the FIRST load issues (it was
	// the fit's worst path formed live for the second), so it has to be
	// right that early.
	dut.u_l1.mem[174] = 16'h207C;  dut.u_l1.mem[175] = 16'h0000;  dut.u_l1.mem[176] = 16'h0984;     // $55C MOVEA.L #$984,A0
	dut.u_l1.mem[177] = 16'h247C;  dut.u_l1.mem[178] = 16'h0000;  dut.u_l1.mem[179] = 16'h0994;     // $562 MOVEA.L #$994,A2
	dut.u_l1.mem[180] = 16'h44FC;  dut.u_l1.mem[181] = 16'h0004;                                    // $568 MOVE #$04,CCR
	dut.u_l1.mem[182] = 16'h224A;                                                                   // $56C MOVEA.L A2,A1               A1 straight from EX
	dut.u_l1.mem[183] = 16'hD388;                                                                   // $56E ADDX.L -(A0),-(A1)
	dut.u_l1.mem[184] = 16'h42F8;  dut.u_l1.mem[185] = 16'h0A80;                                    // $570 MOVE CCR,$0A80.W
	dut.u_l1.mem[186] = 16'h21C8;  dut.u_l1.mem[187] = 16'h0A90;                                    // $574 MOVE.L A0,$0A90.W
	dut.u_l1.mem[188] = 16'h21C9;  dut.u_l1.mem[189] = 16'h0A94;                                    // $578 MOVE.L A1,$0A94.W
	dut.u_l1.mem[190] = 16'h247C;  dut.u_l1.mem[191] = 16'h0000;  dut.u_l1.mem[192] = 16'h09A2;     // $57C MOVEA.L #$9A2,A2
	dut.u_l1.mem[193] = 16'h267C;  dut.u_l1.mem[194] = 16'h0000;  dut.u_l1.mem[195] = 16'h09AE;     // $582 MOVEA.L #$9AE,A3
	dut.u_l1.mem[196] = 16'h44FC;  dut.u_l1.mem[197] = 16'h0014;                                    // $588 MOVE #$14,CCR
	dut.u_l1.mem[198] = 16'h588B;                                                                   // $58C ADDQ.L #4,A3                A3 two ahead
	dut.u_l1.mem[199] = 16'h4E71;                                                                   // $58E NOP
	dut.u_l1.mem[200] = 16'h974A;                                                                   // $590 SUBX.W -(A2),-(A3)
	dut.u_l1.mem[201] = 16'h42F8;  dut.u_l1.mem[202] = 16'h0A82;                                    // $592 MOVE CCR,$0A82.W
	dut.u_l1.mem[203] = 16'h21CA;  dut.u_l1.mem[204] = 16'h0A98;                                    // $596 MOVE.L A2,$0A98.W
	dut.u_l1.mem[205] = 16'h21CB;  dut.u_l1.mem[206] = 16'h0A9C;                                    // $59A MOVE.L A3,$0A9C.W
	dut.u_l1.mem[207] = 16'h2A7C;  dut.u_l1.mem[208] = 16'h0000;  dut.u_l1.mem[209] = 16'h09C2;     // $59E MOVEA.L #$9C2,A5
	dut.u_l1.mem[210] = 16'h44FC;  dut.u_l1.mem[211] = 16'h0004;                                    // $5A4 MOVE #$04,CCR
	dut.u_l1.mem[212] = 16'h284D;                                                                   // $5A8 MOVEA.L A5,A4               A4 straight from EX
	dut.u_l1.mem[213] = 16'hC90C;                                                                   // $5AA ABCD -(A4),-(A4)
	dut.u_l1.mem[214] = 16'h42F8;  dut.u_l1.mem[215] = 16'h0A84;                                    // $5AC MOVE CCR,$0A84.W
	dut.u_l1.mem[216] = 16'h21CC;  dut.u_l1.mem[217] = 16'h0AA0;                                    // $5B0 MOVE.L A4,$0AA0.W
	dut.u_l1.mem[218] = 16'h207C;  dut.u_l1.mem[219] = 16'h0000;  dut.u_l1.mem[220] = 16'h09D0;     // $5B4 MOVEA.L #$9D0,A0
	dut.u_l1.mem[221] = 16'h227C;  dut.u_l1.mem[222] = 16'h0000;  dut.u_l1.mem[223] = 16'h09DE;     // $5BA MOVEA.L #$9DE,A1
	dut.u_l1.mem[224] = 16'h44FC;  dut.u_l1.mem[225] = 16'h0010;                                    // $5C0 MOVE #$10,CCR
	dut.u_l1.mem[226] = 16'h5489;                                                                   // $5C4 ADDQ.L #2,A1                A1 straight from EX
	dut.u_l1.mem[227] = 16'hB348;                                                                   // $5C6 CMPM.W (A0)+,(A1)+
	dut.u_l1.mem[228] = 16'h42F8;  dut.u_l1.mem[229] = 16'h0A86;                                    // $5C8 MOVE CCR,$0A86.W
	dut.u_l1.mem[230] = 16'h21C8;  dut.u_l1.mem[231] = 16'h0AA4;                                    // $5CC MOVE.L A0,$0AA4.W
	dut.u_l1.mem[232] = 16'h21C9;  dut.u_l1.mem[233] = 16'h0AA8;                                    // $5D0 MOVE.L A1,$0AA8.W
	dut.u_l1.mem[234] = 16'h2C7C;  dut.u_l1.mem[235] = 16'h0000;  dut.u_l1.mem[236] = 16'h09F0;     // $5D4 MOVEA.L #$9F0,A6
	dut.u_l1.mem[237] = 16'h207C;  dut.u_l1.mem[238] = 16'h0000;  dut.u_l1.mem[239] = 16'h0B08;     // $5DA MOVEA.L #$B08,A0
	dut.u_l1.mem[240] = 16'h44FC;  dut.u_l1.mem[241] = 16'h0004;                                    // $5E0 MOVE #$04,CCR
	dut.u_l1.mem[242] = 16'h2256;                                                                   // $5E4 MOVEA.L (A6),A1             A1 loaded straight ahead
	dut.u_l1.mem[243] = 16'hD388;                                                                   // $5E6 ADDX.L -(A0),-(A1)
	dut.u_l1.mem[244] = 16'h42F8;  dut.u_l1.mem[245] = 16'h0A88;                                    // $5E8 MOVE CCR,$0A88.W
	dut.u_l1.mem[246] = 16'h21C8;  dut.u_l1.mem[247] = 16'h0AAC;                                    // $5EC MOVE.L A0,$0AAC.W
	dut.u_l1.mem[248] = 16'h21C9;  dut.u_l1.mem[249] = 16'h0AB0;                                    // $5F0 MOVE.L A1,$0AB0.W
	dut.u_l1.mem[250] = 16'h227C;  dut.u_l1.mem[251] = 16'h0000;  dut.u_l1.mem[252] = 16'h0B20;     // $5F4 MOVEA.L #$B20,A1
	dut.u_l1.mem[253] = 16'h207C;  dut.u_l1.mem[254] = 16'h0000;  dut.u_l1.mem[255] = 16'h0B34;     // $5FA MOVEA.L #$B34,A0
	dut.u_l1.mem[256] = 16'h44FC;  dut.u_l1.mem[257] = 16'h0000;                                    // $600 MOVE #$00,CCR
	dut.u_l1.mem[258] = 16'h7864;                                                                   // $604 MOVEQ #100,D4
	dut.u_l1.mem[259] = 16'h4C59;  dut.u_l1.mem[260] = 16'h4003;                                    // $606 DIVU.L (A1)+,D3:D4          A1 stepped early by EX
	dut.u_l1.mem[261] = 16'hD388;                                                                   // $60A ADDX.L -(A0),-(A1)          -(A1) is the divisor's longword
	dut.u_l1.mem[262] = 16'h42F8;  dut.u_l1.mem[263] = 16'h0A8A;                                    // $60C MOVE CCR,$0A8A.W
	dut.u_l1.mem[264] = 16'h21C8;  dut.u_l1.mem[265] = 16'h0AB4;                                    // $610 MOVE.L A0,$0AB4.W
	dut.u_l1.mem[266] = 16'h21C9;  dut.u_l1.mem[267] = 16'h0AB8;                                    // $614 MOVE.L A1,$0AB8.W
	dut.u_l1.mem[268] = 16'h243C;  dut.u_l1.mem[269] = 16'h0003;  dut.u_l1.mem[270] = 16'h0000;     // $618 MOVE.L #$30000,D2
	dut.u_l1.mem[271] = 16'h247C;  dut.u_l1.mem[272] = 16'h0000;  dut.u_l1.mem[273] = 16'h0B40;     // $61E MOVEA.L #$B40,A2
	dut.u_l1.mem[274] = 16'h207C;  dut.u_l1.mem[275] = 16'h0000;  dut.u_l1.mem[276] = 16'h0B50;     // $624 MOVEA.L #$B50,A0
	dut.u_l1.mem[277] = 16'h44FC;  dut.u_l1.mem[278] = 16'h0010;                                    // $62A MOVE #$10,CCR
	dut.u_l1.mem[279] = 16'h4C1A;  dut.u_l1.mem[280] = 16'h2401;                                    // $62E MULU.L (A2)+,D1:D2          A2 stepped early by EX
	dut.u_l1.mem[281] = 16'hB588;                                                                   // $632 CMPM.L (A0)+,(A2)+
	dut.u_l1.mem[282] = 16'h42F8;  dut.u_l1.mem[283] = 16'h0A8C;                                    // $634 MOVE CCR,$0A8C.W
	dut.u_l1.mem[284] = 16'h21C8;  dut.u_l1.mem[285] = 16'h0ABC;                                    // $638 MOVE.L A0,$0ABC.W
	dut.u_l1.mem[286] = 16'h21CA;  dut.u_l1.mem[287] = 16'h0AC0;                                    // $63C MOVE.L A2,$0AC0.W
	dut.u_l1.mem[288] = 16'h60FE;                                                                   // $640 BRA.B -2

	dut.u_l1.mem[518] = 16'h7FFF;  dut.u_l1.mem[519] = 16'hFFFF;   // $80C ADDX source
	dut.u_l1.mem[526] = 16'h0000;  dut.u_l1.mem[527] = 16'h0001;   // $81C ADDX destination
	dut.u_l1.mem[535] = 16'h0001;                                  // $82E SUBX source
	dut.u_l1.mem[543] = 16'h0000;                                  // $83E SUBX destination
	dut.u_l1.mem[551] = 16'h1928;                                  // $84E ABCD: $19 dst, $28 src
	dut.u_l1.mem[559] = 16'h0000;                                  // $85E SBCD destination byte $85F
	dut.u_l1.mem[1535] = 16'h0100;                                 // $FFE SBCD source byte $01
	dut.u_l1.mem[575] = 16'h0305;                                  // $87E PACK source
	dut.u_l1.mem[583] = 16'h0000;                                  // $88E PACK destination byte $88F
	dut.u_l1.mem[591] = 16'h0047;                                  // $89E UNPK source byte $89F
	dut.u_l1.mem[599] = 16'h0000;                                  // $8AE UNPK destination
	dut.u_l1.mem[606] = 16'h0000;  dut.u_l1.mem[607] = 16'h0102;   // $8BC / $8BE PACK -(A4),-(A4)
	dut.u_l1.mem[616] = 16'h0F0F;  dut.u_l1.mem[617] = 16'h0000;   // $8D0 ORI.L
	dut.u_l1.mem[618] = 16'h1234;  dut.u_l1.mem[619] = 16'h5678;   // $8D4 CMPI.L
	dut.u_l1.mem[624] = 16'h0001;  dut.u_l1.mem[625] = 16'h0000;   // $8E0 MULU.L source
	dut.u_l1.mem[632] = 16'h0000;  dut.u_l1.mem[633] = 16'h0007;   // $8F0 DIVU.L source
	for (i = 768; i < 896; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00-$AFF
	dut.u_l1.mem[ 704] = 16'h0000;   // $980
	dut.u_l1.mem[ 705] = 16'h0005;   // $982
	dut.u_l1.mem[ 712] = 16'h0000;   // $990
	dut.u_l1.mem[ 713] = 16'h0007;   // $992
	dut.u_l1.mem[ 720] = 16'h0003;   // $9A0
	dut.u_l1.mem[ 728] = 16'h0010;   // $9B0
	dut.u_l1.mem[ 736] = 16'h1528;   // $9C0
	dut.u_l1.mem[ 744] = 16'h1234;   // $9D0
	dut.u_l1.mem[ 751] = 16'h5555;   // $9DE
	dut.u_l1.mem[ 752] = 16'h1234;   // $9E0
	dut.u_l1.mem[ 760] = 16'h0000;   // $9F0
	dut.u_l1.mem[ 761] = 16'h0B14;   // $9F2
	dut.u_l1.mem[ 898] = 16'h1111;   // $B04
	dut.u_l1.mem[ 899] = 16'h1111;   // $B06
	dut.u_l1.mem[ 904] = 16'h2222;   // $B10
	dut.u_l1.mem[ 905] = 16'h2222;   // $B12
	dut.u_l1.mem[ 912] = 16'h0000;   // $B20
	dut.u_l1.mem[ 913] = 16'h0007;   // $B22
	dut.u_l1.mem[ 920] = 16'h0000;   // $B30
	dut.u_l1.mem[ 921] = 16'h0100;   // $B32
	dut.u_l1.mem[ 928] = 16'h0001;   // $B40
	dut.u_l1.mem[ 929] = 16'h0000;   // $B42
	dut.u_l1.mem[ 930] = 16'h0000;   // $B44
	dut.u_l1.mem[ 931] = 16'h0009;   // $B46
	dut.u_l1.mem[ 936] = 16'h0000;   // $B50
	dut.u_l1.mem[ 937] = 16'h000A;   // $B52
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("ADDX.L $81C",          526, 16'h8000);  chk("ADDX.L $81E", 527, 16'h0001);
	chk("ADDX.L CCR",           768, 16'h000A);
	chk("SUBX.W $83E",          543, 16'hFFFE);
	chk("SUBX.W CCR",           769, 16'h0019);
	chk("ABCD $84E",            551, 16'h4728);
	chkm("ABCD CCR",            770, 16'h0015, 16'h0000);
	chk("SBCD $85E",            559, 16'h0099);
	chkm("SBCD CCR",            771, 16'h0015, 16'h0011);
	chk("SBCD A7 hi",           792, 16'h0000);  chk("SBCD A7 lo", 793, 16'h0FFE);
	chk("SBCD A5 hi",           794, 16'h0000);  chk("SBCD A5 lo", 795, 16'h085F);
	chk("CMPM.W CCR",           772, 16'h0002);
	chk("CMPM.B CCR",           773, 16'h0000);
	chk("A0 hi",                796, 16'h0000);  chk("A0 lo", 797, 16'h080E);
	chk("A1 hi",                798, 16'h0000);  chk("A1 lo", 799, 16'h081E);
	chk("A2 hi",                800, 16'h0000);  chk("A2 lo", 801, 16'h0830);
	chk("A3 hi",                802, 16'h0000);  chk("A3 lo", 803, 16'h083E);
	chk("A4 hi",                804, 16'h0000);  chk("A4 lo", 805, 16'h084E);
	chk("PACK $88E",            583, 16'h0046);
	chk("UNPK $8AE",            599, 16'h3437);
	chk("PACK same $8BC",       606, 16'h0012);
	chk("PACK A0 lo",           807, 16'h087E);  chk("PACK A1 lo", 809, 16'h088F);
	chk("UNPK A2 lo",           811, 16'h089F);  chk("UNPK A3 lo", 813, 16'h08AE);
	chk("PACK same A4 lo",      815, 16'h08BD);
	chk("ORI.L $8D0",           616, 16'hFFFF);  chk("ORI.L $8D2", 617, 16'hF0F0);
	chk("ORI.L CCR",            774, 16'h0018);
	chk("CMPI.L CCR",           775, 16'h0014);
	chk("MULU.L CCR",           776, 16'h0010);
	chk("MULU.L A0 hi",         816, 16'h0000);  chk("MULU.L A0 lo", 817, 16'h08E4);
	chk("DIVU.L CCR",           777, 16'h0010);
	chk("DIVU.L A1 hi",         818, 16'h0000);  chk("DIVU.L A1 lo", 819, 16'h08F0);
	chk("DIVS.L CCR",           778, 16'h0018);
	chk("DIVS.L A7 hi",         820, 16'h0000);  chk("DIVS.L A7 lo", 821, 16'h1000);
	chka("D1 (MULU.L high)",    dbg_d1, 32'h0000_0003);
	chka("D2 (MULU.L low)",     dbg_d2, 32'h0000_0000);
	chka("D3 (DIVU.L rem)",     dbg_d3, 32'h0000_0002);
	chka("D4 (DIVU.L quot)",    dbg_d4, 32'h0000_000E);
	chk("DIVS.L D5 hi",         824, 16'h0000);  chk("DIVS.L D5 lo", 825, 16'h0001);
	chk("DIVS.L D6 hi",         826, 16'hFFFF);  chk("DIVS.L D6 lo", 827, 16'hFFDF);
	chk("user DIVS.L USP hi",   822, 16'h0000);  chk("user DIVS.L USP lo", 823, 16'h1800);
	chka("D5 (user DIVS.L rem)",  dbg_d5, 32'h0000_0000);
	chk("ADDX.L, Ax from EX $990", 712, 16'h0000);  chk("ADDX.L, Ax from EX $992", 713, 16'h000C);
	chk("SUBX.W, Ax two ahead $9B0", 728, 16'h000C);
	chk("ABCD, one An forwarded $9C0", 736, 16'h4328);
	chk("ADDX.L, Ax loaded $B10", 904, 16'h3333);  chk("ADDX.L, Ax loaded $B12", 905, 16'h3333);
	chk("ADDX.L, Ax stepped by DIVU.L $B20", 912, 16'h0000);  chk("ADDX.L, Ax stepped by DIVU.L $B22", 913, 16'h0107);
	chk("fwd CCR 1", 832, 16'h0000);
	chk("fwd CCR 2", 833, 16'h0000);
	chkm("fwd CCR 3", 834, 16'h0015, 16'h0000);
	chk("fwd CCR 4", 835, 16'h0014);
	chk("fwd CCR 5", 836, 16'h0000);
	chk("fwd CCR 6", 837, 16'h0000);
	chk("fwd CCR 7", 838, 16'h0019);
	chk("fwd $A90 hi", 840, 16'h0000);  chk("fwd $A92 lo", 841, 16'h0980);
	chk("fwd $A94 hi", 842, 16'h0000);  chk("fwd $A96 lo", 843, 16'h0990);
	chk("fwd $A98 hi", 844, 16'h0000);  chk("fwd $A9A lo", 845, 16'h09A0);
	chk("fwd $A9C hi", 846, 16'h0000);  chk("fwd $A9E lo", 847, 16'h09B0);
	chk("fwd $AA0 hi", 848, 16'h0000);  chk("fwd $AA2 lo", 849, 16'h09C0);
	chk("fwd $AA4 hi", 850, 16'h0000);  chk("fwd $AA6 lo", 851, 16'h09D2);
	chk("fwd $AA8 hi", 852, 16'h0000);  chk("fwd $AAA lo", 853, 16'h09E2);
	chk("fwd $AAC hi", 854, 16'h0000);  chk("fwd $AAE lo", 855, 16'h0B04);
	chk("fwd $AB0 hi", 856, 16'h0000);  chk("fwd $AB2 lo", 857, 16'h0B10);
	chk("fwd $AB4 hi", 858, 16'h0000);  chk("fwd $AB6 lo", 859, 16'h0B30);
	chk("fwd $AB8 hi", 860, 16'h0000);  chk("fwd $ABA lo", 861, 16'h0B20);
	chk("fwd $ABC hi", 862, 16'h0000);  chk("fwd $ABE lo", 863, 16'h0B54);
	chk("fwd $AC0 hi", 864, 16'h0000);  chk("fwd $AC2 lo", 865, 16'h0B48);
	chka("D6 (user DIVS.L quot)", dbg_d6, 32'h0000_0032);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
