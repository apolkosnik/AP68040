//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: MOVES and CAS //
// under T0)                                                                //
//                                                                          //
// tb_ap040_pipe_movestrace.v - MOVES and CAS are T0 changes of flow        //
//                                                                          //
// The 68040 traces MOVES and CAS under T0, as it does MOVE to SR, MOVEC to //
// a control register and NOP (ap040_core.v's t0_special). The pipelined    //
// core decoded MOVES without adding it to t0_flow_static, so the           //
// instruction after a MOVES retired before any trace, and the trace that   //
// did come belonged to a later change of flow (review 13). CAS arrived in  //
// the same milestone and is checked here with it.                          //
//                                                                          //
// ISP = $1000, SR = $2708 then ORI #$4000,SR (T0). Five MOVES, a CAS, two //
// MOVE USP and a CAS2, each followed by a MOVEQ #k,D3 that must NOT have   //
// run at a trace:                                                         //
//   $44C MOVES.L (A0),D1          load, (An)                              //
//   $452 MOVES.W D2,(A1)+         store, the An steps to $822              //
//   $458 MOVES.B $10(A2),D4       load, a gathered displacement           //
//   $460 MOVES.L A3,-(A4)         store of an An, A4 steps to $84C        //
//   $466 MOVES.L D6,(4,A0,D0.W)   store, a gathered brief index           //
//   $46E CAS.L D2,D6,(A0)         not equal: D2 = $A1B2C3D4, N            //
//   $474 MOVE USP,A6              NOT traced: only the write direction is //
//   $478 MOVE A0,USP              traced (ap040_core.v's t0_special)       //
//   $47C CAS2.L D0:D0,D6:D6,(A0):(A0)   $A1B2C3D4 vs 8: not equal, N     //
// The vector-9 handler at $780 appends {D3, the frame's three longwords}  //
// to a log at $900 -- 16 bytes an entry -- and RTEs; on the eighth entry  //
// it saves D1, D4, A1, A4 and D2 to $980 and stops. Each frame is        //
// format 2: SR, the next instruction's address, $2024, the traced         //
// instruction's own address.                                               //
// D3 at each entry is the value the PREVIOUS case's MOVEQ left.           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movestrace;

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
wire [31:0] dbg_d3, dbg_d5;
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
	.dbg_d3 (dbg_d3), .dbg_d5 (dbg_d5),
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

// One log entry: D3 at the trace, then the frame's SR/PC/format/address.
task chk_entry;
	input integer   n;          // 1..8
	input [31:0]    d3;
	input [15:0]    sr;
	input [31:0]    pc;
	input [31:0]    ia;
	integer w;
	begin
		w = 640 + 8 * (n - 1);
		if ({dut.u_l1.mem[w], dut.u_l1.mem[w+1]} !== d3 ||
		    dut.u_l1.mem[w+2] !== sr ||
		    {dut.u_l1.mem[w+3], dut.u_l1.mem[w+4]} !== pc ||
		    dut.u_l1.mem[w+5] !== 16'h2024 ||
		    {dut.u_l1.mem[w+6], dut.u_l1.mem[w+7]} !== ia) begin
			errors = errors + 1;
			$display("FAIL: trace %0d: D3 %04x%04x SR %04x PC %04x%04x fmt %04x IA %04x%04x, expected D3 %08x SR %04x PC %08x fmt 2024 IA %08x",
			         n, dut.u_l1.mem[w], dut.u_l1.mem[w+1], dut.u_l1.mem[w+2],
			         dut.u_l1.mem[w+3], dut.u_l1.mem[w+4], dut.u_l1.mem[w+5],
			         dut.u_l1.mem[w+6], dut.u_l1.mem[w+7], d3, sr, pc, ia);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h46FC;  dut.u_l1.mem[ 1] = 16'h2700;                               // $400 MOVE #$2700,SR
	dut.u_l1.mem[ 2] = 16'h203C;  dut.u_l1.mem[ 3] = 16'h0000;  dut.u_l1.mem[ 4] = 16'h1000;  // $404 MOVE.L #$1000,D0
	dut.u_l1.mem[ 5] = 16'h4E7B;  dut.u_l1.mem[ 6] = 16'h0804;                               // $40A MOVEC D0,ISP
	dut.u_l1.mem[ 7] = 16'h207C;  dut.u_l1.mem[ 8] = 16'h0000;  dut.u_l1.mem[ 9] = 16'h0810;  // $40E MOVEA.L #$810,A0
	dut.u_l1.mem[10] = 16'h227C;  dut.u_l1.mem[11] = 16'h0000;  dut.u_l1.mem[12] = 16'h0820;  // $414 MOVEA.L #$820,A1
	dut.u_l1.mem[13] = 16'h247C;  dut.u_l1.mem[14] = 16'h0000;  dut.u_l1.mem[15] = 16'h0830;  // $41A MOVEA.L #$830,A2
	dut.u_l1.mem[16] = 16'h267C;  dut.u_l1.mem[17] = 16'h1234;  dut.u_l1.mem[18] = 16'h5678;  // $420 MOVEA.L #$12345678,A3
	dut.u_l1.mem[19] = 16'h287C;  dut.u_l1.mem[20] = 16'h0000;  dut.u_l1.mem[21] = 16'h0850;  // $426 MOVEA.L #$850,A4
	dut.u_l1.mem[22] = 16'h2A7C;  dut.u_l1.mem[23] = 16'h0000;  dut.u_l1.mem[24] = 16'h0900;  // $42C MOVEA.L #$900,A5
	dut.u_l1.mem[25] = 16'h263C;  dut.u_l1.mem[26] = 16'hDEAD;  dut.u_l1.mem[27] = 16'hBEEF;  // $432 MOVE.L #$DEADBEEF,D3
	dut.u_l1.mem[28] = 16'h243C;  dut.u_l1.mem[29] = 16'h0000;  dut.u_l1.mem[30] = 16'hCAFE;  // $438 MOVE.L #$CAFE,D2
	dut.u_l1.mem[31] = 16'h2C3C;  dut.u_l1.mem[32] = 16'h7654;  dut.u_l1.mem[33] = 16'h3210;  // $43E MOVE.L #$76543210,D6
	dut.u_l1.mem[34] = 16'h7008;                                                             // $444 MOVEQ #8,D0
	dut.u_l1.mem[35] = 16'h78FF;                                                             // $446 MOVEQ #-1,D4 (N)
	dut.u_l1.mem[36] = 16'h007C;  dut.u_l1.mem[37] = 16'h4000;                               // $448 ORI #$4000,SR (T0)
	dut.u_l1.mem[38] = 16'h0E90;  dut.u_l1.mem[39] = 16'h1000;                               // $44C MOVES.L (A0),D1
	dut.u_l1.mem[40] = 16'h7655;                                                             // $450 MOVEQ #$55,D3
	dut.u_l1.mem[41] = 16'h0E59;  dut.u_l1.mem[42] = 16'h2800;                               // $452 MOVES.W D2,(A1)+
	dut.u_l1.mem[43] = 16'h7666;                                                             // $456 MOVEQ #$66,D3
	dut.u_l1.mem[44] = 16'h0E2A;  dut.u_l1.mem[45] = 16'h4000;  dut.u_l1.mem[46] = 16'h0010;  // $458 MOVES.B $10(A2),D4
	dut.u_l1.mem[47] = 16'h7677;                                                             // $45E MOVEQ #$77,D3
	dut.u_l1.mem[48] = 16'h0EA4;  dut.u_l1.mem[49] = 16'hB800;                               // $460 MOVES.L A3,-(A4)
	dut.u_l1.mem[50] = 16'h7611;                                                             // $464 MOVEQ #$11,D3
	dut.u_l1.mem[51] = 16'h0EB0;  dut.u_l1.mem[52] = 16'h6800;  dut.u_l1.mem[53] = 16'h0004;  // $466 MOVES.L D6,(4,A0,D0.W)
	dut.u_l1.mem[54] = 16'h7622;                                                             // $46C MOVEQ #$22,D3
	dut.u_l1.mem[55] = 16'h0ED0;  dut.u_l1.mem[56] = 16'h0182;                               // $46E CAS.L D2,D6,(A0)
	dut.u_l1.mem[57] = 16'h7633;                                                             // $472 MOVEQ #$33,D3
	dut.u_l1.mem[58] = 16'h4E6E;                                                             // $474 MOVE USP,A6 (not traced)
	dut.u_l1.mem[59] = 16'h7644;                                                             // $476 MOVEQ #$44,D3
	dut.u_l1.mem[60] = 16'h4E60;                                                             // $478 MOVE A0,USP
	dut.u_l1.mem[61] = 16'h7655;                                                             // $47A MOVEQ #$55,D3
	dut.u_l1.mem[62] = 16'h0EFC;  dut.u_l1.mem[63] = 16'h8180;  dut.u_l1.mem[64] = 16'h8180;  // $47C CAS2.L D0:D0,D6:D6,(A0):(A0)
	dut.u_l1.mem[65] = 16'h7666;                                                             // $482 MOVEQ #$66,D3
	dut.u_l1.mem[66] = 16'h60FE;                                                             // $484 BRA.B -2

	// Operands.
	dut.u_l1.mem[520] = 16'hA1B2;  dut.u_l1.mem[521] = 16'hC3D4;   // $810
	dut.u_l1.mem[526] = 16'h0000;  dut.u_l1.mem[527] = 16'h0000;   // $81C
	dut.u_l1.mem[528] = 16'h0000;                                  // $820
	dut.u_l1.mem[544] = 16'h5A00;                                  // $840: byte $5A
	dut.u_l1.mem[550] = 16'h0000;  dut.u_l1.mem[551] = 16'h0000;   // $84C
	for (i = 640; i < 716; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // the log and $980

	// $780: vector 9.
	dut.u_l1.mem[448] = 16'h5285;                                // $780 ADDQ.L #1,D5
	dut.u_l1.mem[449] = 16'h2AC3;                                // $782 MOVE.L D3,(A5)+
	dut.u_l1.mem[450] = 16'h2AD7;                                // $784 MOVE.L (A7),(A5)+
	dut.u_l1.mem[451] = 16'h2AEF;  dut.u_l1.mem[452] = 16'h0004;  // $786 MOVE.L 4(A7),(A5)+
	dut.u_l1.mem[453] = 16'h2AEF;  dut.u_l1.mem[454] = 16'h0008;  // $78A MOVE.L 8(A7),(A5)+
	dut.u_l1.mem[455] = 16'h7E08;                                // $78E MOVEQ #8,D7
	dut.u_l1.mem[456] = 16'hBA87;                                // $790 CMP.L D7,D5
	dut.u_l1.mem[457] = 16'h6702;                                // $792 BEQ.B $796
	dut.u_l1.mem[458] = 16'h4E73;                                // $794 RTE
	dut.u_l1.mem[459] = 16'h21C1;  dut.u_l1.mem[460] = 16'h0980;  // $796 MOVE.L D1,$0980.W
	dut.u_l1.mem[461] = 16'h21C4;  dut.u_l1.mem[462] = 16'h0984;  // $79A MOVE.L D4,$0984.W
	dut.u_l1.mem[463] = 16'h21C9;  dut.u_l1.mem[464] = 16'h0988;  // $79E MOVE.L A1,$0988.W
	dut.u_l1.mem[465] = 16'h21CC;  dut.u_l1.mem[466] = 16'h098C;  // $7A2 MOVE.L A4,$098C.W
	dut.u_l1.mem[467] = 16'h21C2;  dut.u_l1.mem[468] = 16'h0990;  // $7A6 MOVE.L D2,$0990.W
	dut.u_l1.mem[469] = 16'h60FE;                                // $7AA BRA.B -2

	// Vector 9 -> $780.
	dut.u_l1.mem[3602] = 16'h0000;  dut.u_l1.mem[3603] = 16'h0780;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d5 !== 32'h0000_0008) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000008 (one trace per MOVES, CAS, MOVE A0,USP and CAS2)", dbg_d5);
	end
	if (dbg_d3 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000055 (the last MOVEQ must not run)", dbg_d3);
	end
	chk_entry(1, 32'hDEAD_BEEF, 16'h6708, 32'h0000_0450, 32'h0000_044C);
	chk_entry(2, 32'h0000_0055, 16'h6700, 32'h0000_0456, 32'h0000_0452);
	chk_entry(3, 32'h0000_0066, 16'h6700, 32'h0000_045E, 32'h0000_0458);
	chk_entry(4, 32'h0000_0077, 16'h6700, 32'h0000_0464, 32'h0000_0460);
	chk_entry(5, 32'h0000_0011, 16'h6700, 32'h0000_046C, 32'h0000_0466);
	chk_entry(6, 32'h0000_0022, 16'h6708, 32'h0000_0472, 32'h0000_046E);
	chk_entry(7, 32'h0000_0044, 16'h6700, 32'h0000_047A, 32'h0000_0478);
	chk_entry(8, 32'h0000_0055, 16'h6708, 32'h0000_0482, 32'h0000_047C);
	// The transfers themselves.
	chk("D1 ($980)",           704, 16'hA1B2);  chk("D1 ($982)", 705, 16'hC3D4);
	chk("D4 ($984)",           706, 16'hFFFF);  chk("D4 ($986)", 707, 16'hFF5A);
	chk("A1 ($988)",           708, 16'h0000);  chk("A1 ($98A)", 709, 16'h0822);
	chk("A4 ($98C)",           710, 16'h0000);  chk("A4 ($98E)", 711, 16'h084C);
	chk("MOVES.W store $820",  528, 16'hCAFE);
	chk("MOVES.L A3 $84C",     550, 16'h1234);  chk("MOVES.L A3 $84E", 551, 16'h5678);
	chk("MOVES.L D6 $81C",     526, 16'h7654);  chk("MOVES.L D6 $81E", 527, 16'h3210);
	chk("CAS D2 ($990)",       712, 16'hA1B2);  chk("CAS D2 ($992)", 713, 16'hC3D4);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
