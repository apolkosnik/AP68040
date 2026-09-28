//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 114: MOVE memory to //
// memory)                                                                  //
//                                                                          //
// tb_ap040_pipe_movemm.v - the ordering a two-address MOVE has to keep     //
//                                                                          //
// A memory-to-memory MOVE is one instruction here: EA-fetch loads the      //
// source as any MOVE load does, forms the destination address when that   //
// load completes, and EX stores the data there as a read-modify-write     //
// store with a different address. What the 68k defines, and what each     //
// line below pins, is that the SOURCE's effective address is evaluated     //
// first, side effects included:                                            //
//                                                                          //
//   MOVE.L (A0)+,(A0)+          reads $800, writes $804; A0 = $808         //
//   MOVE.W -(A1),-(A1)          reads $81E, writes $81C; A1 = $81C         //
//   MOVE.B (A2)+,(A2)           reads $830, writes $831; A2 = $831         //
//   MOVE.L (A3)+,(0,A4,A3.L)    the index is the INCREMENTED A3: $1144     //
//   MOVE.B (A7)+,(A7)+          A7 steps TWO for a byte, both times        //
//   MOVE.W $10(A5),$00000A00.L  displacement to absolute long              //
//   MOVE.L #$CAFEF00D,(4,A0,D0.W)  immediate source; SMI D7 after it       //
//   MOVE.L $00000880.L,$00000A10.L four extension words                    //
//   MOVE.W (2,A1,D1.W),(6,A2,D2.L) both sides indexed: port C serves the   //
//                                  source at the load and the destination  //
//                                  when it completes                       //
//                                                                          //
// When the source's (An)+/-(An) and the destination's name the same An,   //
// the destination's update already contains the source's, and the second  //
// write port's is dropped -- the first three lines are that case.          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movemm;

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
wire [31:0] dbg_d6, dbg_d7;
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
	.dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
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
	dut.u_l1.mem[11] = 16'h247C;  dut.u_l1.mem[12] = 16'h0000;  dut.u_l1.mem[13] = 16'h0830;  // MOVEA.L #$830,A2
	dut.u_l1.mem[14] = 16'h267C;  dut.u_l1.mem[15] = 16'h0000;  dut.u_l1.mem[16] = 16'h0840;  // MOVEA.L #$840,A3
	dut.u_l1.mem[17] = 16'h287C;  dut.u_l1.mem[18] = 16'h0000;  dut.u_l1.mem[19] = 16'h0900;  // MOVEA.L #$900,A4
	dut.u_l1.mem[20] = 16'h2A7C;  dut.u_l1.mem[21] = 16'h0000;  dut.u_l1.mem[22] = 16'h0860;  // MOVEA.L #$860,A5
	dut.u_l1.mem[23] = 16'h7008;   // MOVEQ #8,D0
	dut.u_l1.mem[24] = 16'h7204;   // MOVEQ #4,D1
	dut.u_l1.mem[25] = 16'h7401;   // MOVEQ #1,D2

	dut.u_l1.mem[26] = 16'h20D8;                                 // MOVE.L (A0)+,(A0)+
	dut.u_l1.mem[27] = 16'h3321;                                 // MOVE.W -(A1),-(A1)
	dut.u_l1.mem[28] = 16'h149A;                                 // MOVE.B (A2)+,(A2)
	dut.u_l1.mem[29] = 16'h299B;  dut.u_l1.mem[30] = 16'hB800;   // MOVE.L (A3)+,(0,A4,A3.L)
	dut.u_l1.mem[31] = 16'h1EDF;                                 // MOVE.B (A7)+,(A7)+
	dut.u_l1.mem[32] = 16'h33ED;  dut.u_l1.mem[33] = 16'h0010;   // MOVE.W $10(A5),$00000A00.L
	dut.u_l1.mem[34] = 16'h0000;  dut.u_l1.mem[35] = 16'h0A00;
	dut.u_l1.mem[36] = 16'h21BC;  dut.u_l1.mem[37] = 16'hCAFE;   // MOVE.L #$CAFEF00D,(4,A0,D0.W)
	dut.u_l1.mem[38] = 16'hF00D;  dut.u_l1.mem[39] = 16'h0004;
	dut.u_l1.mem[40] = 16'h5BC7;                                 // SMI D7
	dut.u_l1.mem[41] = 16'h23F9;  dut.u_l1.mem[42] = 16'h0000;   // MOVE.L $00000880.L,$00000A10.L
	dut.u_l1.mem[43] = 16'h0880;  dut.u_l1.mem[44] = 16'h0000;
	dut.u_l1.mem[45] = 16'h0A10;
	dut.u_l1.mem[46] = 16'h35B1;  dut.u_l1.mem[47] = 16'h1002;   // MOVE.W (2,A1,D1.W),(6,A2,D2.L)
	dut.u_l1.mem[48] = 16'h2806;
	dut.u_l1.mem[49] = 16'h2C0F;                                 // MOVE.L A7,D6
	dut.u_l1.mem[50] = 16'h60FE;                                 // BRA.B -2

	// Data. Word index = (address - $400) / 2.
	dut.u_l1.mem[512]  = 16'h1111;  dut.u_l1.mem[513]  = 16'h2222;   // $800
	dut.u_l1.mem[514]  = 16'hDEAD;  dut.u_l1.mem[515]  = 16'hBEEF;   // $804
	dut.u_l1.mem[526]  = 16'hAAAA;  dut.u_l1.mem[527]  = 16'h3333;   // $81C, $81E
	dut.u_l1.mem[529]  = 16'h5A5A;                                   // $822
	dut.u_l1.mem[536]  = 16'h4455;                                   // $830/$831
	dut.u_l1.mem[544]  = 16'h6666;  dut.u_l1.mem[545]  = 16'h7777;   // $840
	dut.u_l1.mem[568]  = 16'hBEEF;                                   // $870
	dut.u_l1.mem[576]  = 16'h1234;  dut.u_l1.mem[577]  = 16'h5678;   // $880
	dut.u_l1.mem[1536] = 16'h8811;  dut.u_l1.mem[1537] = 16'h9922;   // $1000, $1002
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("source $800 untouched",               512, 16'h1111);
	chk("MOVE.L (A0)+,(A0)+ at $804 high",     514, 16'h1111);
	chk("MOVE.L (A0)+,(A0)+ at $804 low",      515, 16'h2222);
	chk("MOVE.W -(A1),-(A1) at $81C",          526, 16'h3333);
	chk("MOVE.B (A2)+,(A2) at $831",           536, 16'h4444);
	chk("MOVE.L (A3)+,(0,A4,A3.L) $1144 high", 1698, 16'h6666);
	chk("MOVE.L (A3)+,(0,A4,A3.L) $1144 low",  1699, 16'h7777);
	chk("MOVE.B (A7)+,(A7)+ at $1002",         1537, 16'h8822);
	chk("MOVE.W $10(A5),$A00.L",               768, 16'hBEEF);
	chk("MOVE.L #imm,(4,A0,D0.W) $814 high",   522, 16'hCAFE);
	chk("MOVE.L #imm,(4,A0,D0.W) $814 low",    523, 16'hF00D);
	chk("MOVE.L $880.L,$A10.L high",           776, 16'h1234);
	chk("MOVE.L $880.L,$A10.L low",            777, 16'h5678);
	chk("MOVE.W (2,A1,D1),(6,A2,D2) $838",     540, 16'h5A5A);

	chka("A0", dut.u_cpu.u_regfile.areg[0], 32'h0000_0808);
	chka("A1", dut.u_cpu.u_regfile.areg[1], 32'h0000_081C);
	chka("A2", dut.u_cpu.u_regfile.areg[2], 32'h0000_0831);
	chka("A3", dut.u_cpu.u_regfile.areg[3], 32'h0000_0844);
	chka("A7 (D6)", dbg_d6, 32'h0000_1004);
	if (dbg_d7[7:0] !== 8'hFF) begin
		errors = errors + 1;
		$display("FAIL: SMI after MOVE.L #$CAFEF00D = %h, expected FF (MOVE sets N from the data)",
		         dbg_d7[7:0]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
