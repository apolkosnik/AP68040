//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 115: EXG, the       //
// 68020 TST/BTST/CMPI operands, and the PC base behind an immediate)       //
//                                                                          //
// tb_ap040_pipe_exgtst.v                                                   //
//                                                                          //
// EXG writes two registers: the main port takes Rx := Ry, and the second   //
// port -- (An)+'s -- takes Ry := Rx, read on port B. All three forms:      //
//   EXG D0,D1   EXG A0,A1   EXG D2,A0                                      //
//                                                                          //
// TST takes An (.W/.L), a PC-relative operand and an immediate; BTST takes //
// an immediate as its DATA (BTST Dn,#imm) and PC-relative operands; CMPI   //
// takes PC-relative operands. Each result is kept by an Scc to $600-$607.  //
//                                                                          //
// The last three are the point of eac_pc_base: a PC-relative displacement //
// is relative to the address of ITS OWN extension word, and an immediate   //
// ahead of it moves that word two or four bytes past the opcode's +2:      //
//   BTST #3,(d16,PC)             base = opcode + 4                         //
//   CMPI.W #$1234,(d16,PC)       base = opcode + 4                         //
//   CMPI.L #$56789ABC,(2,PC,D7)  base = opcode + 6                         //
// A base two bytes early lands on neighbouring data and flips the answer.  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_exgtst;

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
wire [31:0] dbg_d0, dbg_d1, dbg_d2;
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
	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
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
	dut.u_l1.mem[ 6] = 16'h243C;  dut.u_l1.mem[ 7] = 16'h5555;  dut.u_l1.mem[ 8] = 16'h5555;  // MOVE.L #$55555555,D2
	dut.u_l1.mem[ 9] = 16'h207C;  dut.u_l1.mem[10] = 16'h3333;  dut.u_l1.mem[11] = 16'h3333;  // MOVEA.L #$33333333,A0
	dut.u_l1.mem[12] = 16'h227C;  dut.u_l1.mem[13] = 16'h4444;  dut.u_l1.mem[14] = 16'h4444;  // MOVEA.L #$44444444,A1
	dut.u_l1.mem[15] = 16'hC141;                                                             // EXG D0,D1
	dut.u_l1.mem[16] = 16'hC149;                                                             // EXG A0,A1
	dut.u_l1.mem[17] = 16'hC588;                                                             // EXG D2,A0
	dut.u_l1.mem[18] = 16'h247C;  dut.u_l1.mem[19] = 16'h0000;  dut.u_l1.mem[20] = 16'h8000;  // MOVEA.L #$8000,A2
	dut.u_l1.mem[21] = 16'h4A4A;                                                             // TST.W A2
	dut.u_l1.mem[22] = 16'h5BF8;  dut.u_l1.mem[23] = 16'h0600;                               // SMI $0600.W
	dut.u_l1.mem[24] = 16'h4ABC;  dut.u_l1.mem[25] = 16'h0000;  dut.u_l1.mem[26] = 16'h0000;  // TST.L #0
	dut.u_l1.mem[27] = 16'h57F8;  dut.u_l1.mem[28] = 16'h0601;                               // SEQ $0601.W
	dut.u_l1.mem[29] = 16'h4A3A;  dut.u_l1.mem[30] = 16'h0084;                               // TST.B $4C0(PC)
	dut.u_l1.mem[31] = 16'h5BF8;  dut.u_l1.mem[32] = 16'h0602;                               // SMI $0602.W
	dut.u_l1.mem[33] = 16'h7C04;                                                             // MOVEQ #4,D6
	dut.u_l1.mem[34] = 16'h0D3C;  dut.u_l1.mem[35] = 16'h0010;                               // BTST D6,#$10
	dut.u_l1.mem[36] = 16'h56F8;  dut.u_l1.mem[37] = 16'h0603;                               // SNE $0603.W
	dut.u_l1.mem[38] = 16'h0D3A;  dut.u_l1.mem[39] = 16'h0073;                               // BTST D6,$4C1(PC)
	dut.u_l1.mem[40] = 16'h56F8;  dut.u_l1.mem[41] = 16'h0604;                               // SNE $0604.W
	dut.u_l1.mem[42] = 16'h083A;  dut.u_l1.mem[43] = 16'h0003;  dut.u_l1.mem[44] = 16'h006A;  // BTST #3,$4C2(PC)
	dut.u_l1.mem[45] = 16'h56F8;  dut.u_l1.mem[46] = 16'h0605;                               // SNE $0605.W
	dut.u_l1.mem[47] = 16'h0C7A;  dut.u_l1.mem[48] = 16'h1234;  dut.u_l1.mem[49] = 16'h0062;  // CMPI.W #$1234,$4C4(PC)
	dut.u_l1.mem[50] = 16'h57F8;  dut.u_l1.mem[51] = 16'h0606;                               // SEQ $0606.W
	dut.u_l1.mem[52] = 16'h7E02;                                                             // MOVEQ #2,D7
	dut.u_l1.mem[53] = 16'h0CBB;  dut.u_l1.mem[54] = 16'h5678;  dut.u_l1.mem[55] = 16'h9ABC;  // CMPI.L #$56789ABC,
	dut.u_l1.mem[56] = 16'h7056;                                                             //   ($56,PC,D7.W) = $4C8
	dut.u_l1.mem[57] = 16'h57F8;  dut.u_l1.mem[58] = 16'h0607;                               // SEQ $0607.W
	dut.u_l1.mem[59] = 16'h60FE;                                                             // BRA.B -2

	// Data at $4C0. Word index = (address - $400) / 2.
	dut.u_l1.mem[ 96] = 16'h8010;   // $4C0 = $80 (TST.B: negative), $4C1 = $10 (bit 4 set)
	dut.u_l1.mem[ 97] = 16'h0800;   // $4C2 = $08 (bit 3 set)
	dut.u_l1.mem[ 98] = 16'h1234;   // $4C4
	dut.u_l1.mem[100] = 16'h5678;   // $4C8
	dut.u_l1.mem[101] = 16'h9ABC;

	for (i = 256; i < 260; i = i + 1) dut.u_l1.mem[i] = 16'h0000;   // $600-$607
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chka("D0", dbg_d0, 32'h2222_2222);
	chka("D1", dbg_d1, 32'h1111_1111);
	chka("D2", dbg_d2, 32'h4444_4444);
	chka("A0", dut.u_cpu.u_regfile.areg[0], 32'h5555_5555);
	chka("A1", dut.u_cpu.u_regfile.areg[1], 32'h3333_3333);
	chk("$600/1 TST.W An, TST.L #imm",        256, 16'hFFFF);
	chk("$602/3 TST pcrel, BTST Dn,#imm",        257, 16'hFFFF);
	chk("$604/5 BTST Dn/#n pcrel",    258, 16'hFFFF);
	chk("$606/7 CMPI.W/.L pcrel",   259, 16'hFFFF);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
