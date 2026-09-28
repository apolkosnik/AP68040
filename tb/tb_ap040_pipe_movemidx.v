//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 115: indexed MOVEM) //
//                                                                          //
// tb_ap040_pipe_movemidx.v                                                 //
//                                                                          //
// MOVEM with (d8,An,Xn), both ways, and (d8,PC,Xn) for the load. The       //
// sequencer starts from ea_target, which forms base + index + d8. For the  //
// PC form the base is the address of the BRIEF word, which the register    //
// mask pushes to opcode + 4 -- eac_pc_base carries it.                     //
//                                                                          //
//   MOVEM.L D0-D2,(4,A0,D3.W)     A0 = $800, D3 = 8: $80C, $810, $814      //
//   MOVEM.L ($28,PC,D7.W),D4-D6   D7 = 2: the table at $450                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movemidx;

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
wire [31:0] dbg_d4, dbg_d5, dbg_d6;
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
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
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

	dut.u_l1.mem[ 0] = 16'h203C;  dut.u_l1.mem[ 1] = 16'h1111;  dut.u_l1.mem[ 2] = 16'h1111;  // MOVE.L #$11111111,D0
	dut.u_l1.mem[ 3] = 16'h223C;  dut.u_l1.mem[ 4] = 16'h2222;  dut.u_l1.mem[ 5] = 16'h2222;  // MOVE.L #$22222222,D1
	dut.u_l1.mem[ 6] = 16'h243C;  dut.u_l1.mem[ 7] = 16'h3333;  dut.u_l1.mem[ 8] = 16'h3333;  // MOVE.L #$33333333,D2
	dut.u_l1.mem[ 9] = 16'h207C;  dut.u_l1.mem[10] = 16'h0000;  dut.u_l1.mem[11] = 16'h0800;  // MOVEA.L #$800,A0
	dut.u_l1.mem[12] = 16'h7608;                                                             // MOVEQ #8,D3
	dut.u_l1.mem[13] = 16'h48F0;  dut.u_l1.mem[14] = 16'h0007;  dut.u_l1.mem[15] = 16'h3004;  // MOVEM.L D0-D2,(4,A0,D3.W)
	dut.u_l1.mem[16] = 16'h7E02;                                                             // MOVEQ #2,D7
	dut.u_l1.mem[17] = 16'h4CFB;  dut.u_l1.mem[18] = 16'h0070;  dut.u_l1.mem[19] = 16'h7028;  // MOVEM.L ($28,PC,D7.W),D4-D6
	dut.u_l1.mem[20] = 16'h60FE;                                                             // BRA.B -2

	// $450: the table the PC-indexed load reads.
	dut.u_l1.mem[40] = 16'hAAAA;  dut.u_l1.mem[41] = 16'h0001;
	dut.u_l1.mem[42] = 16'hAAAA;  dut.u_l1.mem[43] = 16'h0002;
	dut.u_l1.mem[44] = 16'hAAAA;  dut.u_l1.mem[45] = 16'h0003;
	for (i = 516; i < 526; i = i + 1) dut.u_l1.mem[i] = 16'hEEEE;   // $808-$81B
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("$808 below the block", 517, 16'hEEEE);
	chk("$80C high", 518, 16'h1111);  chk("$80C low", 519, 16'h1111);
	chk("$810 high", 520, 16'h2222);  chk("$810 low", 521, 16'h2222);
	chk("$814 high", 522, 16'h3333);  chk("$814 low", 523, 16'h3333);
	chk("$818 above the block", 524, 16'hEEEE);
	chka("D4", dbg_d4, 32'hAAAA_0001);
	chka("D5", dbg_d5, 32'hAAAA_0002);
	chka("D6", dbg_d6, 32'hAAAA_0003);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
