//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: TRAPV, MOVE    //
// USP, RESET, RTR)                                                         //
//                                                                          //
// tb_ap040_pipe_sysmisc.v - four system instructions, both privilege modes //
//                                                                          //
// Expected values follow the 68040 rules and ap040_core.v, not this RTL:   //
//   MOVE A0,USP / MOVE USP,A1   A1 = $1800: the read is one instruction    //
//                               behind the write, whose commit is a cycle  //
//                               late for it                                //
//   TRAPV, V clear              nothing                                    //
//   TRAPV, V set                vector 7, format 2: SR $2702, PC $436,     //
//                               $201C, the TRAPV's address $434            //
//   RESET (supervisor)          nothing                                    //
//   RTR, {CCR $15, PC}          CCR X Z C, A7 back at $1000                //
//   RTR, {CCR $0A, PC $601}     address error, format 2: SR $270A -- the   //
//                               popped CCR --, the RTR's own address,      //
//                               $200C, $600; A7 keeps its value, so the    //
//                               frame sits at $FEE (ap040_core.v S_RET3)   //
//   user mode, RTR              A7 is USP: $17FA back to $1800, CCR $04.   //
//                               RTE's own stack restore writes ISP/MSP     //
//                               directly, which RTR must not do            //
//   user mode, MOVE USP,A2 / RESET / MOVE A0,USP                           //
//                               vector 8 each; the handler logs the PC     //
//                               and steps over the instruction; A2 keeps   //
//                               $5555                                      //
// The two MOVEQ #k,D7 after the RTRs must never run.                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_sysmisc;

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
wire [31:0] dbg_d7;
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
	.dbg_d7 (dbg_d7),
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
	dut.u_l1.mem[  2] = 16'h7E00;                                                              // $404 MOVEQ #0,D7
	dut.u_l1.mem[  3] = 16'h203C;  dut.u_l1.mem[  4] = 16'h0000;  dut.u_l1.mem[  5] = 16'h1000; // $406 MOVE.L #$1000,D0
	dut.u_l1.mem[  6] = 16'h4E7B;  dut.u_l1.mem[  7] = 16'h0804;                               // $40C MOVEC D0,ISP
	dut.u_l1.mem[  8] = 16'h207C;  dut.u_l1.mem[  9] = 16'h0000;  dut.u_l1.mem[ 10] = 16'h1800; // $410 MOVEA.L #$1800,A0
	dut.u_l1.mem[ 11] = 16'h247C;  dut.u_l1.mem[ 12] = 16'h0000;  dut.u_l1.mem[ 13] = 16'h5555; // $416 MOVEA.L #$5555,A2
	dut.u_l1.mem[ 14] = 16'h2A7C;  dut.u_l1.mem[ 15] = 16'h0000;  dut.u_l1.mem[ 16] = 16'h0A70; // $41C MOVEA.L #$0A70,A5          the privilege log
	dut.u_l1.mem[ 17] = 16'h4E60;                                                              // $422 MOVE A0,USP
	dut.u_l1.mem[ 18] = 16'h4E69;                                                              // $424 MOVE USP,A1
	dut.u_l1.mem[ 19] = 16'h21C9;  dut.u_l1.mem[ 20] = 16'h0A00;                               // $426 MOVE.L A1,$0A00.W
	dut.u_l1.mem[ 21] = 16'h44FC;  dut.u_l1.mem[ 22] = 16'h0000;                               // $42A MOVE #0,CCR
	dut.u_l1.mem[ 23] = 16'h4E76;                                                              // $42E TRAPV                      V clear: nothing
	dut.u_l1.mem[ 24] = 16'h44FC;  dut.u_l1.mem[ 25] = 16'h0002;                               // $430 MOVE #$02,CCR
	dut.u_l1.mem[ 26] = 16'h4E76;                                                              // $434 TRAPV                      V set: vector 7
	dut.u_l1.mem[ 27] = 16'h4E70;                                                              // $436 RESET                      supervisor: nothing
	dut.u_l1.mem[ 28] = 16'h2F3C;  dut.u_l1.mem[ 29] = 16'h0000;  dut.u_l1.mem[ 30] = 16'h044A; // $438 MOVE.L #rtr_tgt,-(A7)
	dut.u_l1.mem[ 31] = 16'h3F3C;  dut.u_l1.mem[ 32] = 16'h0015;                               // $43E MOVE.W #$0015,-(A7)        X Z C
	dut.u_l1.mem[ 33] = 16'h44FC;  dut.u_l1.mem[ 34] = 16'h0000;                               // $442 MOVE #0,CCR
	dut.u_l1.mem[ 35] = 16'h4E77;                                                              // $446 RTR
	dut.u_l1.mem[ 36] = 16'h7E77;                                                              // $448 MOVEQ #$77,D7              never
	dut.u_l1.mem[ 37] = 16'h42F8;  dut.u_l1.mem[ 38] = 16'h0A04;                               // $44A MOVE CCR,$0A04.W
	dut.u_l1.mem[ 39] = 16'h21CF;  dut.u_l1.mem[ 40] = 16'h0A06;                               // $44E MOVE.L A7,$0A06.W
	dut.u_l1.mem[ 41] = 16'h2F3C;  dut.u_l1.mem[ 42] = 16'h0000;  dut.u_l1.mem[ 43] = 16'h0601; // $452 MOVE.L #$601,-(A7)
	dut.u_l1.mem[ 44] = 16'h3F3C;  dut.u_l1.mem[ 45] = 16'h000A;                               // $458 MOVE.W #$000A,-(A7)        N V
	dut.u_l1.mem[ 46] = 16'h44FC;  dut.u_l1.mem[ 47] = 16'h0000;                               // $45C MOVE #0,CCR
	dut.u_l1.mem[ 48] = 16'h4E77;                                                              // $460 RTR                        odd PC: vector 3
	dut.u_l1.mem[ 49] = 16'h60FE;                                                              // $462 BRA.B -2                   never
	dut.u_l1.mem[ 50] = 16'h2E7C;  dut.u_l1.mem[ 51] = 16'h0000;  dut.u_l1.mem[ 52] = 16'h1000; // $464 MOVEA.L #$1000,A7
	dut.u_l1.mem[ 53] = 16'h46FC;  dut.u_l1.mem[ 54] = 16'h0000;                               // $46A MOVE #0,SR                 user: A7 is USP, $1800
	dut.u_l1.mem[ 55] = 16'h2F3C;  dut.u_l1.mem[ 56] = 16'h0000;  dut.u_l1.mem[ 57] = 16'h047C; // $46E MOVE.L #rtr_utgt,-(A7)
	dut.u_l1.mem[ 58] = 16'h3F3C;  dut.u_l1.mem[ 59] = 16'h0004;                               // $474 MOVE.W #$0004,-(A7)
	dut.u_l1.mem[ 60] = 16'h4E77;                                                              // $478 RTR                        user: USP moves
	dut.u_l1.mem[ 61] = 16'h7E66;                                                              // $47A MOVEQ #$66,D7              never
	dut.u_l1.mem[ 62] = 16'h42F8;  dut.u_l1.mem[ 63] = 16'h0A0A;                               // $47C MOVE CCR,$0A0A.W
	dut.u_l1.mem[ 64] = 16'h21CF;  dut.u_l1.mem[ 65] = 16'h0A0C;                               // $480 MOVE.L A7,$0A0C.W
	dut.u_l1.mem[ 66] = 16'h4E6A;                                                              // $484 MOVE USP,A2                user: vector 8
	dut.u_l1.mem[ 67] = 16'h4E70;                                                              // $486 RESET                      user: vector 8
	dut.u_l1.mem[ 68] = 16'h4E60;                                                              // $488 MOVE A0,USP                user: vector 8
	dut.u_l1.mem[ 69] = 16'h21CA;  dut.u_l1.mem[ 70] = 16'h0A10;                               // $48A MOVE.L A2,$0A10.W
	dut.u_l1.mem[ 71] = 16'h60FE;                                                              // $48E BRA.B -2

	for (i = 768; i < 832; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00-$A7F

	// $700: vector 3 -- log the frame and A7, then carry on in supervisor.
	dut.u_l1.mem[384] = 16'h31D7;  dut.u_l1.mem[385] = 16'h0A50;                               // $700 MOVE.W (A7),$0A50.W
	dut.u_l1.mem[386] = 16'h21EF;  dut.u_l1.mem[387] = 16'h0002;  dut.u_l1.mem[388] = 16'h0A52; // $704 MOVE.L 2(A7),$0A52.W
	dut.u_l1.mem[389] = 16'h31EF;  dut.u_l1.mem[390] = 16'h0006;  dut.u_l1.mem[391] = 16'h0A56; // $70A MOVE.W 6(A7),$0A56.W
	dut.u_l1.mem[392] = 16'h21EF;  dut.u_l1.mem[393] = 16'h0008;  dut.u_l1.mem[394] = 16'h0A58; // $710 MOVE.L 8(A7),$0A58.W
	dut.u_l1.mem[395] = 16'h21CF;  dut.u_l1.mem[396] = 16'h0A5C;                               // $716 MOVE.L A7,$0A5C.W
	dut.u_l1.mem[397] = 16'h4EF8;  dut.u_l1.mem[398] = 16'h0464;                               // $71A JMP $0464.W (phase 2)

	// $740: vector 7.
	dut.u_l1.mem[416] = 16'h31D7;  dut.u_l1.mem[417] = 16'h0A40;                               // $740 MOVE.W (A7),$0A40.W
	dut.u_l1.mem[418] = 16'h21EF;  dut.u_l1.mem[419] = 16'h0002;  dut.u_l1.mem[420] = 16'h0A42; // $744 MOVE.L 2(A7),$0A42.W
	dut.u_l1.mem[421] = 16'h31EF;  dut.u_l1.mem[422] = 16'h0006;  dut.u_l1.mem[423] = 16'h0A46; // $74A MOVE.W 6(A7),$0A46.W
	dut.u_l1.mem[424] = 16'h21EF;  dut.u_l1.mem[425] = 16'h0008;  dut.u_l1.mem[426] = 16'h0A48; // $750 MOVE.L 8(A7),$0A48.W
	dut.u_l1.mem[427] = 16'h5278;  dut.u_l1.mem[428] = 16'h0A4C;                               // $756 ADDQ.W #1,$0A4C.W
	dut.u_l1.mem[429] = 16'h4E73;                                                              // $75A RTE

	// $780: vector 8 -- count, log the PC, step over the instruction.
	dut.u_l1.mem[448] = 16'h5278;  dut.u_l1.mem[449] = 16'h0A60;                               // $780 ADDQ.W #1,$0A60.W
	dut.u_l1.mem[450] = 16'h2AEF;  dut.u_l1.mem[451] = 16'h0002;                               // $784 MOVE.L 2(A7),(A5)+
	dut.u_l1.mem[452] = 16'h54AF;  dut.u_l1.mem[453] = 16'h0002;                               // $788 ADDQ.L #2,2(A7)
	dut.u_l1.mem[454] = 16'h4E73;                                                              // $78C RTE

	// Vectors 3, 7 and 8.
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0700;
	dut.u_l1.mem[3598] = 16'h0000;  dut.u_l1.mem[3599] = 16'h0740;
	dut.u_l1.mem[3600] = 16'h0000;  dut.u_l1.mem[3601] = 16'h0780;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("MOVE USP,A1 hi",          768, 16'h0000);  chk("MOVE USP,A1 lo", 769, 16'h1800);
	chk("TRAPV frame SR",          800, 16'h2702);
	chk("TRAPV frame PC hi",       801, 16'h0000);  chk("TRAPV frame PC lo", 802, 16'h0436);
	chk("TRAPV frame format",      803, 16'h201C);
	chk("TRAPV frame IA hi",       804, 16'h0000);  chk("TRAPV frame IA lo", 805, 16'h0434);
	chk("TRAPV entries",           806, 16'h0001);
	chk("RTR CCR",                 770, 16'h0015);
	chk("RTR A7 hi",               771, 16'h0000);  chk("RTR A7 lo", 772, 16'h1000);
	chk("RTR odd: frame SR",       808, 16'h270A);
	chk("RTR odd: frame PC hi",    809, 16'h0000);  chk("RTR odd: frame PC lo", 810, 16'h0460);
	chk("RTR odd: frame format",   811, 16'h200C);
	chk("RTR odd: address hi",     812, 16'h0000);  chk("RTR odd: address lo", 813, 16'h0600);
	chk("RTR odd: A7 hi",          814, 16'h0000);  chk("RTR odd: A7 lo", 815, 16'h0FEE);
	chk("user RTR CCR",            773, 16'h0004);
	chk("user RTR USP hi",         774, 16'h0000);  chk("user RTR USP lo", 775, 16'h1800);
	chk("privilege entries",       816, 16'h0003);
	chk("priv MOVE USP,A2 hi",     824, 16'h0000);  chk("priv MOVE USP,A2 lo", 825, 16'h0484);
	chk("priv RESET hi",           826, 16'h0000);  chk("priv RESET lo", 827, 16'h0486);
	chk("priv MOVE A0,USP hi",     828, 16'h0000);  chk("priv MOVE A0,USP lo", 829, 16'h0488);
	chk("A2 untouched hi",         776, 16'h0000);  chk("A2 untouched lo", 777, 16'h5555);
	if (dbg_d7 !== 32'd0) begin
		errors = errors + 1;
		$display("FAIL: D7 = %h, expected 00000000 (an RTR fell through)", dbg_d7);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
