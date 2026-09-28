//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 113: the unary      //
// family at (d16,An) and (d8,An,Xn), every operation)                      //
//                                                                          //
// tb_ap040_pipe_unarygx.v - NEGX, CLR, NEG, NOT and TST, gathered          //
//                                                                          //
// Milestone 111 decoded CLR and TST at (d16,An) and (d8,An,Xn) and held    //
// NEG, NOT and NEGX back as "wrong through this carrier". The reason was   //
// the operation selector: unary_mem_op was built from is_neg_mem and its   //
// siblings, which carry the (An)/(An)+/-(An) mode test, so for a gathered  //
// form every one was false and the selector fell through to TST. TST       //
// returns operand A, which the read-modify-write crossover fills with the  //
// data register ir[11:9] names -- so a gathered CLR.L $10(A0) STORED D1    //
// (review 11 finding 1), and NEG/NOT/NEGX stored D2/D3/D0.                 //
//                                                                          //
// That register is the whole trick of this bench. Every one of them holds  //
// a poison value here, because a zero there hides the defect: CLR of a     //
// zero register looks like CLR.                                            //
//                                                                          //
//   D0 $11111111 (NEGX)  D1 $12345678 (CLR)  D2 $CAFEBABE (NEG)            //
//   D3 $0F0F0F0F (NOT)   D5 $55555555 (TST)  D6 = 4, the index             //
//                                                                          //
//   CLR.L   $10(A0)          $810: $A1B2C3D4 -> $00000000                  //
//   NEG.L   $20(A0)          $820: $00000001 -> $FFFFFFFF                  //
//   NOT.W   $30(A0)          $830: $00FF     -> $FF00                      //
//   ANDI    #0,CCR           X := 0                                        //
//   NEGX.B  $40(A0)          $840: $01       -> $FF, X := 1                //
//   CLR.W   $10(A0,D6.W)     $814: $1357     -> $0000                      //
//   NEG.B   $20(A0,D6.W)     $824: $05       -> $FB, X := 1                //
//   NOT.L   $30(A0,D6.W)     $834: $12345678 -> $EDCBA987                  //
//   NEGX.W  $40(A0,D6.W)     $844: $0000     -> $FFFF (0 - 0 - X)          //
//   TST.L   $50(A0)          $850: $80000000, CCR := X N = $18             //
//                                                                          //
// The last NEGX needs the X the NEG before it left, so it also proves the  //
// flag chain through the gathered carrier. The registers must come out     //
// exactly as they went in: nothing here writes one.                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_unarygx;

localparam PROG_WORDS      = 200;
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

task chkd;
	input [63:0]  name;
	input [31:0]  got;
	input [31:0]  want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h (no instruction here writes a register)",
			         name, got, want);
		end
	end
endtask

// MOVE.L #imm,Dn -- three words from word index w
task ldd;
	input integer w;
	input [2:0]  n;
	input [31:0] value;
	begin
		dut.u_l1.mem[w]   = {4'b0010, n, 9'b000_111_100};
		dut.u_l1.mem[w+1] = value[31:16];
		dut.u_l1.mem[w+2] = value[15:0];
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h207C;   // MOVEA.L #$00000800,A0
	dut.u_l1.mem[ 1] = 16'h0000;
	dut.u_l1.mem[ 2] = 16'h0800;
	dut.u_l1.mem[ 3] = 16'h7C04;   // MOVEQ #4,D6     -- the index
	ldd( 4, 3'd0, 32'h1111_1111);  // D0 -- NEGX's ir[11:9]
	ldd( 7, 3'd1, 32'h1234_5678);  // D1 -- CLR's
	ldd(10, 3'd2, 32'hCAFE_BABE);  // D2 -- NEG's
	ldd(13, 3'd3, 32'h0F0F_0F0F);  // D3 -- NOT's
	ldd(16, 3'd5, 32'h5555_5555);  // D5 -- TST's

	dut.u_l1.mem[19] = 16'h42A8;  dut.u_l1.mem[20] = 16'h0010;   // CLR.L  $10(A0)
	dut.u_l1.mem[21] = 16'h44A8;  dut.u_l1.mem[22] = 16'h0020;   // NEG.L  $20(A0)
	dut.u_l1.mem[23] = 16'h4668;  dut.u_l1.mem[24] = 16'h0030;   // NOT.W  $30(A0)
	dut.u_l1.mem[25] = 16'h023C;  dut.u_l1.mem[26] = 16'h0000;   // ANDI   #0,CCR
	dut.u_l1.mem[27] = 16'h4028;  dut.u_l1.mem[28] = 16'h0040;   // NEGX.B $40(A0)
	dut.u_l1.mem[29] = 16'h4270;  dut.u_l1.mem[30] = 16'h6010;   // CLR.W  $10(A0,D6.W)
	dut.u_l1.mem[31] = 16'h4430;  dut.u_l1.mem[32] = 16'h6020;   // NEG.B  $20(A0,D6.W)
	dut.u_l1.mem[33] = 16'h46B0;  dut.u_l1.mem[34] = 16'h6030;   // NOT.L  $30(A0,D6.W)
	dut.u_l1.mem[35] = 16'h4070;  dut.u_l1.mem[36] = 16'h6040;   // NEGX.W $40(A0,D6.W)
	dut.u_l1.mem[37] = 16'h4AA8;  dut.u_l1.mem[38] = 16'h0050;   // TST.L  $50(A0)
	dut.u_l1.mem[39] = 16'h60FE;                                 // BRA.B -2

	// Operands. Word index = (address - $400) / 2.
	dut.u_l1.mem[520] = 16'hA1B2;  dut.u_l1.mem[521] = 16'hC3D4;   // $810
	dut.u_l1.mem[522] = 16'h1357;  dut.u_l1.mem[523] = 16'h2468;   // $814
	dut.u_l1.mem[528] = 16'h0000;  dut.u_l1.mem[529] = 16'h0001;   // $820
	dut.u_l1.mem[530] = 16'h05EE;                                  // $824 (byte $05)
	dut.u_l1.mem[536] = 16'h00FF;  dut.u_l1.mem[537] = 16'h7777;   // $830
	dut.u_l1.mem[538] = 16'h1234;  dut.u_l1.mem[539] = 16'h5678;   // $834
	dut.u_l1.mem[544] = 16'h0166;                                  // $840 (byte $01)
	dut.u_l1.mem[546] = 16'h0000;  dut.u_l1.mem[547] = 16'h9999;   // $844
	dut.u_l1.mem[552] = 16'h8000;  dut.u_l1.mem[553] = 16'h0000;   // $850
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chkw("CLR.L  $10(A0) high",      520, 16'h0000);
	chkw("CLR.L  $10(A0) low",       521, 16'h0000);
	chkw("NEG.L  $20(A0) high",      528, 16'hFFFF);
	chkw("NEG.L  $20(A0) low",       529, 16'hFFFF);
	chkw("NOT.W  $30(A0)",           536, 16'hFF00);
	chkw("NOT.W  neighbour $832",    537, 16'h7777);
	chkw("NEGX.B $40(A0) + $841",    544, 16'hFF66);
	chkw("CLR.W  idx $814",          522, 16'h0000);
	chkw("CLR.W  neighbour $816",    523, 16'h2468);
	chkw("NEG.B  idx $824 + $825",   530, 16'hFBEE);
	chkw("NOT.L  idx $834 high",     538, 16'hEDCB);
	chkw("NOT.L  idx $834 low",      539, 16'hA987);
	chkw("NEGX.W idx $844",          546, 16'hFFFF);
	chkw("NEGX.W neighbour $846",    547, 16'h9999);
	chkw("TST.L  operand untouched", 552, 16'h8000);

	chkd("D0", dbg_d0, 32'h1111_1111);
	chkd("D1", dbg_d1, 32'h1234_5678);
	chkd("D2", dbg_d2, 32'hCAFE_BABE);
	chkd("D3", dbg_d3, 32'h0F0F_0F0F);
	chkd("D5", dbg_d5, 32'h5555_5555);

	if (dbg_ccr !== 5'h18) begin
		errors = errors + 1;
		$display("FAIL: CCR = %h, expected 18 (X from the NEGX.W, N from the TST.L of $80000000)",
		         dbg_ccr);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
