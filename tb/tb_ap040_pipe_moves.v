//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 114: MOVES)         //
//                                                                          //
// tb_ap040_pipe_moves.v - a MOVE whose register is in its extension word  //
//                                                                          //
// MOVES <ea>,Rn and Rn,<ea>: 0000 1110 ss <ea>, then a word {A/D, reg,     //
// direction}, then the EA's own words. On a flat memory the alternate      //
// function codes change nothing, which leaves four things to get right,    //
// each checked here:                                                       //
//                                                                          //
//  - a load into An is SIGN-EXTENDED to 32 bits, byte and word alike       //
//    (ap040_core.v's S_MOVES_RD); into Dn it merges like any MOVE.         //
//  - a store of the An its own (An)+/-(An) steps writes the STEPPED value  //
//    -- the reference reads Rn after ea_start -- where MOVE stores the      //
//    original.                                                             //
//  - no condition code changes: ORI #$15,CCR before the block, and MOVE     //
//    CCR,D7 after it must read $15.                                        //
//  - it is privileged, and a user-mode MOVES must do NOTHING before its     //
//    vector-8 entry: no store and no address-register step. It is the      //
//    first privileged instruction here that touches memory.                //
//                                                                          //
//   MOVES.L D1,(A0)            $800 := $11223344                           //
//   MOVES.B 4(A0),A3           $F0 -> A3 = $FFFFFFF0                       //
//   MOVES.W (A1)+,D2           $8001 -> D2 = $AAAA8001, A1 = $822          //
//   MOVES.W -(A2),A4           $7FFE -> A4 = $00007FFE, A2 = $83E          //
//   MOVES.L A2,-(A2)           A2 = $83A, and $83A := $0000083A            //
//   MOVES.B D1,$0850.W         $850 := $44                                 //
//   MOVES.L D1,(8,A0,D6.W)     $80C := $11223344                           //
//   ANDI #$DFFF,SR ; MOVES.L D1,(A5)+     user mode: vector 8, $860 and    //
//                                         A5 untouched                     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_moves;

localparam PROG_WORDS      = 300;
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
wire [31:0] dbg_d2, dbg_d7;
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
	.dbg_d2 (dbg_d2), .dbg_d7 (dbg_d7), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
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

task chka;
	input [63:0] name;
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

	dut.u_l1.mem[ 0] = 16'h203C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h1000;  // MOVE.L #$1000,D0
	dut.u_l1.mem[ 3] = 16'h4E7B;  dut.u_l1.mem[ 4] = 16'h0804;                               // MOVEC D0,ISP
	dut.u_l1.mem[ 5] = 16'h207C;  dut.u_l1.mem[ 6] = 16'h0000;  dut.u_l1.mem[ 7] = 16'h0800;  // MOVEA.L #$800,A0
	dut.u_l1.mem[ 8] = 16'h227C;  dut.u_l1.mem[ 9] = 16'h0000;  dut.u_l1.mem[10] = 16'h0820;  // MOVEA.L #$820,A1
	dut.u_l1.mem[11] = 16'h247C;  dut.u_l1.mem[12] = 16'h0000;  dut.u_l1.mem[13] = 16'h0840;  // MOVEA.L #$840,A2
	dut.u_l1.mem[14] = 16'h2A7C;  dut.u_l1.mem[15] = 16'h0000;  dut.u_l1.mem[16] = 16'h0860;  // MOVEA.L #$860,A5
	dut.u_l1.mem[17] = 16'h223C;  dut.u_l1.mem[18] = 16'h1122;  dut.u_l1.mem[19] = 16'h3344;  // MOVE.L #$11223344,D1
	dut.u_l1.mem[20] = 16'h243C;  dut.u_l1.mem[21] = 16'hAAAA;  dut.u_l1.mem[22] = 16'hAAAA;  // MOVE.L #$AAAAAAAA,D2
	dut.u_l1.mem[23] = 16'h7C04;                                                             // MOVEQ #4,D6
	dut.u_l1.mem[24] = 16'h003C;  dut.u_l1.mem[25] = 16'h0015;                               // ORI #$15,CCR

	dut.u_l1.mem[26] = 16'h0E90;  dut.u_l1.mem[27] = 16'h1800;                               // MOVES.L D1,(A0)
	dut.u_l1.mem[28] = 16'h0E28;  dut.u_l1.mem[29] = 16'hB000;  dut.u_l1.mem[30] = 16'h0004;  // MOVES.B 4(A0),A3
	dut.u_l1.mem[31] = 16'h0E59;  dut.u_l1.mem[32] = 16'h2000;                               // MOVES.W (A1)+,D2
	dut.u_l1.mem[33] = 16'h0E62;  dut.u_l1.mem[34] = 16'hC000;                               // MOVES.W -(A2),A4
	dut.u_l1.mem[35] = 16'h0EA2;  dut.u_l1.mem[36] = 16'hA800;                               // MOVES.L A2,-(A2)
	dut.u_l1.mem[37] = 16'h0E38;  dut.u_l1.mem[38] = 16'h1800;  dut.u_l1.mem[39] = 16'h0850;  // MOVES.B D1,$0850.W
	dut.u_l1.mem[40] = 16'h0EB0;  dut.u_l1.mem[41] = 16'h1800;  dut.u_l1.mem[42] = 16'h6008;  // MOVES.L D1,(8,A0,D6.W)
	dut.u_l1.mem[43] = 16'h42C7;                                                             // MOVE CCR,D7
	dut.u_l1.mem[44] = 16'h027C;  dut.u_l1.mem[45] = 16'hDFFF;                               // ANDI #$DFFF,SR
	dut.u_l1.mem[46] = 16'h0E9D;  dut.u_l1.mem[47] = 16'h1800;                               // MOVES.L D1,(A5)+ (user)
	dut.u_l1.mem[48] = 16'h60FE;                                                             // BRA.B -2

	// $780: vector 8.
	dut.u_l1.mem[448] = 16'h5278;  dut.u_l1.mem[449] = 16'h0600;  // ADDQ.W #1,$0600.W
	dut.u_l1.mem[450] = 16'h60FE;                                 // BRA.B -2
	dut.u_l1.mem[3600] = 16'h0000; dut.u_l1.mem[3601] = 16'h0780;

	// Data. Word index = (address - $400) / 2.
	dut.u_l1.mem[256] = 16'h0000;                                   // $600 vector-8 count
	dut.u_l1.mem[512] = 16'h9999;  dut.u_l1.mem[513] = 16'h9999;    // $800
	dut.u_l1.mem[514] = 16'hF077;                                   // $804 (byte $F0)
	dut.u_l1.mem[518] = 16'h9999;  dut.u_l1.mem[519] = 16'h9999;    // $80C
	dut.u_l1.mem[528] = 16'h8001;                                   // $820
	dut.u_l1.mem[541] = 16'h9999;  dut.u_l1.mem[542] = 16'h9999;    // $83A
	dut.u_l1.mem[543] = 16'h7FFE;                                   // $83E
	dut.u_l1.mem[552] = 16'h9988;                                   // $850
	dut.u_l1.mem[560] = 16'hCCCC;  dut.u_l1.mem[561] = 16'hCCCC;    // $860
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("MOVES.L D1,(A0) high",        512, 16'h1122);
	chk("MOVES.L D1,(A0) low",         513, 16'h3344);
	chk("MOVES.L A2,-(A2) high",       541, 16'h0000);
	chk("MOVES.L A2,-(A2) low",        542, 16'h083A);
	chk("MOVES.B D1,$0850.W",          552, 16'h4488);
	chk("MOVES.L D1,(8,A0,D6.W) high", 518, 16'h1122);
	chk("MOVES.L D1,(8,A0,D6.W) low",  519, 16'h3344);
	chk("user MOVES stored nothing",   560, 16'hCCCC);
	chk("user MOVES stored nothing",   561, 16'hCCCC);
	chk("vector 8 count at $600",      256, 16'h0001);

	chka("A1", dut.u_cpu.u_regfile.areg[1], 32'h0000_0822);
	chka("A2", dut.u_cpu.u_regfile.areg[2], 32'h0000_083A);
	chka("A3", dut.u_cpu.u_regfile.areg[3], 32'hFFFF_FFF0);
	chka("A4", dut.u_cpu.u_regfile.areg[4], 32'h0000_7FFE);
	chka("A5", dut.u_cpu.u_regfile.areg[5], 32'h0000_0860);
	chka("D2", dbg_d2, 32'hAAAA_8001);
	chka("CCR (D7)", {16'd0, dbg_d7[15:0]}, 32'h0000_0015);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
