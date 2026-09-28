//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 115: the long       //
// multiply and divide)                                                     //
//                                                                          //
// tb_ap040_pipe_mull.v - MULU.L/MULS.L and DIVU.L/DIVS.L                   //
//                                                                          //
// The rules are ap040_core.v's EK_MD_L, each pinned by one line:           //
//                                                                          //
//   MULU.L D1,D0          $12345 * $10000: the low half, and V set        //
//                         because the high half is not zero  ($600 SVS)    //
//   MULS.L D3,D4:D2       -2 * 3 = -6 in 64 bits: D4 = $FFFFFFFF,          //
//                         D2 = $FFFFFFFA, N set               ($601 SMI)   //
//   DIVU.L D6,D7:D5       100 / 7: D5 = 14, remainder D7 = 2               //
//   DIVS.L 4(A0),D1:D0    -256 (64-bit) / 16: D0 = -16, D1 = 0             //
//   DIVU.L #2,D2:D3       5:0 / 2 does not fit 32 bits: V set, and D2/D3   //
//                         keep 5 and 0                        ($602 SVS)   //
//   ORI #1,CCR; DIVU.L #0,D5    vector 5, C cleared on the way  ($603 SCC) //
//   MULU.L #$10000,D6:D6  Dh = Dl: the register keeps the LOW half, 0      //
//   DIVS.L (6,A0,D7.W),D1:D0    a 64-bit dividend with an indexed source:  //
//                         port C reads D7 for the index at the load, then  //
//                         D1 (Dr) when it completes: D0 = $3FFFFFFC        //
//   MULU.L (A1)+,D6:D6    one data register, so an (An)+ source is allowed //
//                         -- and the second port must carry the STEP:      //
//                         A1 = $80C, D6 = the low half, 0                  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_mull;

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
	dut.u_l1.mem[ 8] = 16'h203C;  dut.u_l1.mem[ 9] = 16'h0001;  dut.u_l1.mem[10] = 16'h2345;  // MOVE.L #$12345,D0
	dut.u_l1.mem[11] = 16'h223C;  dut.u_l1.mem[12] = 16'h0001;  dut.u_l1.mem[13] = 16'h0000;  // MOVE.L #$10000,D1
	dut.u_l1.mem[14] = 16'h4C01;  dut.u_l1.mem[15] = 16'h0000;                               // MULU.L D1,D0
	dut.u_l1.mem[16] = 16'h59F8;  dut.u_l1.mem[17] = 16'h0600;                               // SVS $0600.W
	dut.u_l1.mem[18] = 16'h243C;  dut.u_l1.mem[19] = 16'hFFFF;  dut.u_l1.mem[20] = 16'hFFFE;  // MOVE.L #-2,D2
	dut.u_l1.mem[21] = 16'h7603;                                                             // MOVEQ #3,D3
	dut.u_l1.mem[22] = 16'h4C03;  dut.u_l1.mem[23] = 16'h2C04;                               // MULS.L D3,D4:D2
	dut.u_l1.mem[24] = 16'h5BF8;  dut.u_l1.mem[25] = 16'h0601;                               // SMI $0601.W
	dut.u_l1.mem[26] = 16'h7A64;                                                             // MOVEQ #100,D5
	dut.u_l1.mem[27] = 16'h7C07;                                                             // MOVEQ #7,D6
	dut.u_l1.mem[28] = 16'h4C46;  dut.u_l1.mem[29] = 16'h5007;                               // DIVU.L D6,D7:D5
	dut.u_l1.mem[30] = 16'h72FF;                                                             // MOVEQ #-1,D1
	dut.u_l1.mem[31] = 16'h203C;  dut.u_l1.mem[32] = 16'hFFFF;  dut.u_l1.mem[33] = 16'hFF00;  // MOVE.L #$FFFFFF00,D0
	dut.u_l1.mem[34] = 16'h4C68;  dut.u_l1.mem[35] = 16'h0C01;  dut.u_l1.mem[36] = 16'h0004;  // DIVS.L 4(A0),D1:D0
	dut.u_l1.mem[37] = 16'h7405;                                                             // MOVEQ #5,D2
	dut.u_l1.mem[38] = 16'h7600;                                                             // MOVEQ #0,D3
	dut.u_l1.mem[39] = 16'h4C7C;  dut.u_l1.mem[40] = 16'h3402;                               // DIVU.L #2,D2:D3
	dut.u_l1.mem[41] = 16'h0000;  dut.u_l1.mem[42] = 16'h0002;
	dut.u_l1.mem[43] = 16'h59F8;  dut.u_l1.mem[44] = 16'h0602;                               // SVS $0602.W
	dut.u_l1.mem[45] = 16'h003C;  dut.u_l1.mem[46] = 16'h0001;                               // ORI #1,CCR
	dut.u_l1.mem[47] = 16'h4C7C;  dut.u_l1.mem[48] = 16'h5005;                               // DIVU.L #0,D5
	dut.u_l1.mem[49] = 16'h0000;  dut.u_l1.mem[50] = 16'h0000;
	dut.u_l1.mem[51] = 16'h54F8;  dut.u_l1.mem[52] = 16'h0603;                               // SCC $0603.W
	dut.u_l1.mem[53] = 16'h2C3C;  dut.u_l1.mem[54] = 16'h0001;  dut.u_l1.mem[55] = 16'h0000;  // MOVE.L #$10000,D6
	dut.u_l1.mem[56] = 16'h4C3C;  dut.u_l1.mem[57] = 16'h6406;                               // MULU.L #$10000,D6:D6
	dut.u_l1.mem[58] = 16'h0001;  dut.u_l1.mem[59] = 16'h0000;
	dut.u_l1.mem[60] = 16'h4C70;  dut.u_l1.mem[61] = 16'h0C01;  dut.u_l1.mem[62] = 16'h7006;  // DIVS.L (6,A0,D7.W),D1:D0
	dut.u_l1.mem[63] = 16'h227C;  dut.u_l1.mem[64] = 16'h0000;  dut.u_l1.mem[65] = 16'h0808;  // MOVEA.L #$808,A1
	dut.u_l1.mem[66] = 16'h2C3C;  dut.u_l1.mem[67] = 16'h4000;  dut.u_l1.mem[68] = 16'h0000;  // MOVE.L #$40000000,D6
	dut.u_l1.mem[69] = 16'h4C19;  dut.u_l1.mem[70] = 16'h6406;                               // MULU.L (A1)+,D6:D6
	dut.u_l1.mem[71] = 16'h60FE;                                                             // BRA.B -2

	// $700: vector 5.
	dut.u_l1.mem[384] = 16'h5278;  dut.u_l1.mem[385] = 16'h0610;  // ADDQ.W #1,$0610.W
	dut.u_l1.mem[386] = 16'h4E73;                                 // RTE
	dut.u_l1.mem[3594] = 16'h0000; dut.u_l1.mem[3595] = 16'h0700;

	// Data. Word index = (address - $400) / 2.
	dut.u_l1.mem[256] = 16'h0000;  dut.u_l1.mem[257] = 16'h0000;   // $600-$603
	dut.u_l1.mem[264] = 16'h0000;                                  // $610
	dut.u_l1.mem[514] = 16'h0000;  dut.u_l1.mem[515] = 16'h0010;   // $804 = 16
	dut.u_l1.mem[516] = 16'h0000;  dut.u_l1.mem[517] = 16'h0004;   // $808 = 4
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chka("D0", dbg_d0, 32'h3FFF_FFFC);
	chka("D1", dbg_d1, 32'h0000_0000);
	chka("D2", dbg_d2, 32'h0000_0005);
	chka("D3", dbg_d3, 32'h0000_0000);
	chka("D4", dbg_d4, 32'hFFFF_FFFF);
	chka("D5", dbg_d5, 32'h0000_000E);
	chka("D6", dbg_d6, 32'h0000_0000);   // both MULU.L Dh=Dl forms keep the low half
	chka("A1", dut.u_cpu.u_regfile.areg[1], 32'h0000_080C);
	chka("D7", dbg_d7, 32'h0000_0002);
	chk("$600 MULU.L V, $601 MULS.L N", 256, 16'hFFFF);
	chk("$602 DIVL V, $603 C cleared",  257, 16'hFFFF);
	chk("$610 vector 5 count",          264, 16'h0001);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
