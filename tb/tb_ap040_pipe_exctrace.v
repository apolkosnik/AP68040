//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: no trace      //
// after an exception)                                                      //
//                                                                          //
// tb_ap040_pipe_exctrace.v - a traced faulting instruction enters its own //
// handler                                                                  //
//                                                                          //
// On the 68040 no exception entry leaves a trace pending: ap040_core.v's   //
// S_EXC0 clears T1/T0 and arms nothing, and WinUAE keeps the 68020's       //
// post-TRAP trace "except on 68040 or 68060". The pipeline armed one after //
// every entry made under T1 or T0, so a second, vector-9 frame went on the //
// stack before the primary handler ran (review 14).                        //
//                                                                          //
// Every vector used points at one handler at $700, which logs the frame's //
// format/vector word and A7 at entry to (A5)+, then clears T, resets ISP  //
// to $1000 and jumps to the next case through A4. If a trace got in first //
// the log shows $2024 where the primary vector belongs, and one entry too //
// many.                                                                    //
//   under T1, then again under T0:                                         //
//     MOVEQ / NOP        the control: an ordinary traced instruction does  //
//                        take vector 9 -- $2024, A7 $FF4                   //
//     ILLEGAL            $0010, $FF8                                       //
//     TRAPV (V set)      $201C, $FF4                                       //
//     DIVU.W #0,D1       $2014, $FF4                                       //
//     CHK2.B (A0),D1     $2018, $FF4 (D1 = $30, bounds $10-$20)            //
//     RTR to $601        $200C, $FEE: the address error; the RTR's pop is  //
//                        backed out, so the frame is below its 6 bytes     //
//     RESET, user mode   $0020, $FF8                                       //
//     TRAP #0            $0080, $FF8                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_exctrace;

// Every word the fetch stage issues counts, refetches too, and since
// bundle 10 each SR write refetches what follows it: 400 ran out before
// the last entries. The L1 is NOP-filled, so the slack is NOPs.
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

task chk_entry;
	input integer   k;
	input [127:0]   name;
	input [15:0]    fv;
	input [31:0]    a7;
	integer w;
	begin
		w = 768 + 3 * k;
		if (dut.u_l1.mem[w] !== fv || {dut.u_l1.mem[w+1], dut.u_l1.mem[w+2]} !== a7) begin
			errors = errors + 1;
			$display("FAIL: entry %0d (%0s): fmt/vec %04x A7 %04x%04x, expected %04x %08x",
			         k, name, dut.u_l1.mem[w], dut.u_l1.mem[w+1], dut.u_l1.mem[w+2], fv, a7);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[  0] = 16'h46FC;  dut.u_l1.mem[  1] = 16'h2700;                               // $400 MOVE #$2700,SR
	dut.u_l1.mem[  2] = 16'h203C;  dut.u_l1.mem[  3] = 16'h0000;  dut.u_l1.mem[  4] = 16'h1000; // $404 MOVE.L #$1000,D0
	dut.u_l1.mem[  5] = 16'h4E7B;  dut.u_l1.mem[  6] = 16'h0804;                               // $40A MOVEC D0,ISP
	dut.u_l1.mem[  7] = 16'h207C;  dut.u_l1.mem[  8] = 16'h0000;  dut.u_l1.mem[  9] = 16'h0800; // $40E MOVEA.L #$800,A0          CHK2 bounds
	dut.u_l1.mem[ 10] = 16'h267C;  dut.u_l1.mem[ 11] = 16'h0000;  dut.u_l1.mem[ 12] = 16'h1800; // $414 MOVEA.L #$1800,A3
	dut.u_l1.mem[ 13] = 16'h4E63;                                                              // $41A MOVE A3,USP
	dut.u_l1.mem[ 14] = 16'h2A7C;  dut.u_l1.mem[ 15] = 16'h0000;  dut.u_l1.mem[ 16] = 16'h0A00; // $41C MOVEA.L #$A00,A5          the log
	dut.u_l1.mem[ 17] = 16'h7230;                                                              // $422 MOVEQ #$30,D1             out of CHK2 bounds
	dut.u_l1.mem[ 18] = 16'h287C;  dut.u_l1.mem[ 19] = 16'h0000;  dut.u_l1.mem[ 20] = 16'h0430; // $424 T1 ctl: MOVEA.L #next,A4
	dut.u_l1.mem[ 21] = 16'h46FC;  dut.u_l1.mem[ 22] = 16'hA000;                               // $42A MOVE #$A000,SR
	dut.u_l1.mem[ 23] = 16'h7401;                                                              // $42E MOVEQ #1,D2 -- traced
	dut.u_l1.mem[ 24] = 16'h287C;  dut.u_l1.mem[ 25] = 16'h0000;  dut.u_l1.mem[ 26] = 16'h043C; // $430 T1 ill: MOVEA.L #next,A4
	dut.u_l1.mem[ 27] = 16'h46FC;  dut.u_l1.mem[ 28] = 16'hA000;                               // $436 MOVE #$A000,SR
	dut.u_l1.mem[ 29] = 16'h4AFC;                                                              // $43A ILLEGAL
	dut.u_l1.mem[ 30] = 16'h287C;  dut.u_l1.mem[ 31] = 16'h0000;  dut.u_l1.mem[ 32] = 16'h0448; // $43C T1 trapv: MOVEA.L #next,A4
	dut.u_l1.mem[ 33] = 16'h46FC;  dut.u_l1.mem[ 34] = 16'hA002;                               // $442 MOVE #$A002,SR
	dut.u_l1.mem[ 35] = 16'h4E76;                                                              // $446 TRAPV
	dut.u_l1.mem[ 36] = 16'h287C;  dut.u_l1.mem[ 37] = 16'h0000;  dut.u_l1.mem[ 38] = 16'h0456; // $448 T1 div: MOVEA.L #next,A4
	dut.u_l1.mem[ 39] = 16'h46FC;  dut.u_l1.mem[ 40] = 16'hA000;                               // $44E MOVE #$A000,SR
	dut.u_l1.mem[ 41] = 16'h82FC;  dut.u_l1.mem[ 42] = 16'h0000;                               // $452 DIVU.W #0,D1
	dut.u_l1.mem[ 43] = 16'h287C;  dut.u_l1.mem[ 44] = 16'h0000;  dut.u_l1.mem[ 45] = 16'h0464; // $456 T1 chk2: MOVEA.L #next,A4
	dut.u_l1.mem[ 46] = 16'h46FC;  dut.u_l1.mem[ 47] = 16'hA000;                               // $45C MOVE #$A000,SR
	dut.u_l1.mem[ 48] = 16'h00D0;  dut.u_l1.mem[ 49] = 16'h1800;                               // $460 CHK2.B (A0),D1
	dut.u_l1.mem[ 50] = 16'h287C;  dut.u_l1.mem[ 51] = 16'h0000;  dut.u_l1.mem[ 52] = 16'h047A; // $464 T1 rtr: MOVEA.L #next,A4
	dut.u_l1.mem[ 53] = 16'h2F3C;  dut.u_l1.mem[ 54] = 16'h0000;  dut.u_l1.mem[ 55] = 16'h0601; // $46A MOVE.L #$601,-(A7)
	dut.u_l1.mem[ 56] = 16'h3F3C;  dut.u_l1.mem[ 57] = 16'h0000;                               // $470 MOVE.W #0,-(A7)
	dut.u_l1.mem[ 58] = 16'h46FC;  dut.u_l1.mem[ 59] = 16'hA000;                               // $474 MOVE #$A000,SR
	dut.u_l1.mem[ 60] = 16'h4E77;                                                              // $478 RTR                       odd
	dut.u_l1.mem[ 61] = 16'h287C;  dut.u_l1.mem[ 62] = 16'h0000;  dut.u_l1.mem[ 63] = 16'h0486; // $47A T1 priv: MOVEA.L #next,A4
	dut.u_l1.mem[ 64] = 16'h46FC;  dut.u_l1.mem[ 65] = 16'h8000;                               // $480 MOVE #$8000,SR
	dut.u_l1.mem[ 66] = 16'h4E70;                                                              // $484 RESET                     user mode
	dut.u_l1.mem[ 67] = 16'h287C;  dut.u_l1.mem[ 68] = 16'h0000;  dut.u_l1.mem[ 69] = 16'h0492; // $486 T1 trap: MOVEA.L #next,A4
	dut.u_l1.mem[ 70] = 16'h46FC;  dut.u_l1.mem[ 71] = 16'hA000;                               // $48C MOVE #$A000,SR
	dut.u_l1.mem[ 72] = 16'h4E40;                                                              // $490 TRAP #0
	dut.u_l1.mem[ 73] = 16'h287C;  dut.u_l1.mem[ 74] = 16'h0000;  dut.u_l1.mem[ 75] = 16'h049E; // $492 T0 ctl: MOVEA.L #next,A4
	dut.u_l1.mem[ 76] = 16'h46FC;  dut.u_l1.mem[ 77] = 16'h6000;                               // $498 MOVE #$6000,SR
	dut.u_l1.mem[ 78] = 16'h4E71;                                                              // $49C NOP -- traced
	dut.u_l1.mem[ 79] = 16'h287C;  dut.u_l1.mem[ 80] = 16'h0000;  dut.u_l1.mem[ 81] = 16'h04AA; // $49E T0 ill: MOVEA.L #next,A4
	dut.u_l1.mem[ 82] = 16'h46FC;  dut.u_l1.mem[ 83] = 16'h6000;                               // $4A4 MOVE #$6000,SR
	dut.u_l1.mem[ 84] = 16'h4AFC;                                                              // $4A8 ILLEGAL
	dut.u_l1.mem[ 85] = 16'h287C;  dut.u_l1.mem[ 86] = 16'h0000;  dut.u_l1.mem[ 87] = 16'h04B6; // $4AA T0 trapv: MOVEA.L #next,A4
	dut.u_l1.mem[ 88] = 16'h46FC;  dut.u_l1.mem[ 89] = 16'h6002;                               // $4B0 MOVE #$6002,SR
	dut.u_l1.mem[ 90] = 16'h4E76;                                                              // $4B4 TRAPV
	dut.u_l1.mem[ 91] = 16'h287C;  dut.u_l1.mem[ 92] = 16'h0000;  dut.u_l1.mem[ 93] = 16'h04C4; // $4B6 T0 div: MOVEA.L #next,A4
	dut.u_l1.mem[ 94] = 16'h46FC;  dut.u_l1.mem[ 95] = 16'h6000;                               // $4BC MOVE #$6000,SR
	dut.u_l1.mem[ 96] = 16'h82FC;  dut.u_l1.mem[ 97] = 16'h0000;                               // $4C0 DIVU.W #0,D1
	dut.u_l1.mem[ 98] = 16'h287C;  dut.u_l1.mem[ 99] = 16'h0000;  dut.u_l1.mem[100] = 16'h04D2; // $4C4 T0 chk2: MOVEA.L #next,A4
	dut.u_l1.mem[101] = 16'h46FC;  dut.u_l1.mem[102] = 16'h6000;                               // $4CA MOVE #$6000,SR
	dut.u_l1.mem[103] = 16'h00D0;  dut.u_l1.mem[104] = 16'h1800;                               // $4CE CHK2.B (A0),D1
	dut.u_l1.mem[105] = 16'h287C;  dut.u_l1.mem[106] = 16'h0000;  dut.u_l1.mem[107] = 16'h04E8; // $4D2 T0 rtr: MOVEA.L #next,A4
	dut.u_l1.mem[108] = 16'h2F3C;  dut.u_l1.mem[109] = 16'h0000;  dut.u_l1.mem[110] = 16'h0601; // $4D8 MOVE.L #$601,-(A7)
	dut.u_l1.mem[111] = 16'h3F3C;  dut.u_l1.mem[112] = 16'h0000;                               // $4DE MOVE.W #0,-(A7)
	dut.u_l1.mem[113] = 16'h46FC;  dut.u_l1.mem[114] = 16'h6000;                               // $4E2 MOVE #$6000,SR
	dut.u_l1.mem[115] = 16'h4E77;                                                              // $4E6 RTR                       odd
	dut.u_l1.mem[116] = 16'h287C;  dut.u_l1.mem[117] = 16'h0000;  dut.u_l1.mem[118] = 16'h04F4; // $4E8 T0 priv: MOVEA.L #next,A4
	dut.u_l1.mem[119] = 16'h46FC;  dut.u_l1.mem[120] = 16'h4000;                               // $4EE MOVE #$4000,SR
	dut.u_l1.mem[121] = 16'h4E70;                                                              // $4F2 RESET                     user mode
	dut.u_l1.mem[122] = 16'h287C;  dut.u_l1.mem[123] = 16'h0000;  dut.u_l1.mem[124] = 16'h0500; // $4F4 T0 trap: MOVEA.L #next,A4
	dut.u_l1.mem[125] = 16'h46FC;  dut.u_l1.mem[126] = 16'h6000;                               // $4FA MOVE #$6000,SR
	dut.u_l1.mem[127] = 16'h4E40;                                                              // $4FE TRAP #0
	dut.u_l1.mem[128] = 16'h60FE;                                                              // $500 BRA.B -2

	dut.u_l1.mem[512] = 16'h1020;                                  // $800: CHK2 bounds $10, $20
	for (i = 768; i < 832; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // the log

	// $700: every vector.
	dut.u_l1.mem[384] = 16'h3AEF;  dut.u_l1.mem[385] = 16'h0006;                               // $700 MOVE.W 6(A7),(A5)+
	dut.u_l1.mem[386] = 16'h2ACF;                                                              // $704 MOVE.L A7,(A5)+
	dut.u_l1.mem[387] = 16'h46FC;  dut.u_l1.mem[388] = 16'h2700;                               // $706 MOVE #$2700,SR
	dut.u_l1.mem[389] = 16'h2E7C;  dut.u_l1.mem[390] = 16'h0000;  dut.u_l1.mem[391] = 16'h1000; // $70A MOVEA.L #$1000,A7
	dut.u_l1.mem[392] = 16'h4ED4;                                                              // $710 JMP (A4)

	// Vectors 3, 4, 5, 6, 7, 8, 9 and 32 -> $700.
	for (i = 3; i <= 9; i = i + 1) begin
		dut.u_l1.mem[3584 + 2 * i] = 16'h0000;  dut.u_l1.mem[3585 + 2 * i] = 16'h0700;
	end
	dut.u_l1.mem[3648] = 16'h0000;  dut.u_l1.mem[3649] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 2400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk_entry( 0, "T1 traced control", 16'h2024, 32'h00000FF4);
	chk_entry( 1, "T1 ILLEGAL", 16'h0010, 32'h00000FF8);
	chk_entry( 2, "T1 TRAPV", 16'h201C, 32'h00000FF4);
	chk_entry( 3, "T1 DIVU #0", 16'h2014, 32'h00000FF4);
	chk_entry( 4, "T1 CHK2", 16'h2018, 32'h00000FF4);
	chk_entry( 5, "T1 odd RTR", 16'h200C, 32'h00000FEE);
	chk_entry( 6, "T1 user RESET", 16'h0020, 32'h00000FF8);
	chk_entry( 7, "T1 TRAP #0", 16'h0080, 32'h00000FF8);
	chk_entry( 8, "T0 traced control", 16'h2024, 32'h00000FF4);
	chk_entry( 9, "T0 ILLEGAL", 16'h0010, 32'h00000FF8);
	chk_entry(10, "T0 TRAPV", 16'h201C, 32'h00000FF4);
	chk_entry(11, "T0 DIVU #0", 16'h2014, 32'h00000FF4);
	chk_entry(12, "T0 CHK2", 16'h2018, 32'h00000FF4);
	chk_entry(13, "T0 odd RTR", 16'h200C, 32'h00000FEE);
	chk_entry(14, "T0 user RESET", 16'h0020, 32'h00000FF8);
	chk_entry(15, "T0 TRAP #0", 16'h0080, 32'h00000FF8);
	if (dut.u_l1.mem[816] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: a seventeenth log entry, fmt/vec %04x", dut.u_l1.mem[816]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
