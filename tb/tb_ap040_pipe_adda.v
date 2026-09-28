//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 44: ADDA/SUBA/CMPA) //
//                                                                          //
// tb_ap040_pipe_adda.v - pointer arithmetic, and the first sign extension  //
//                                                                          //
// Every array walk and every stack adjustment is made of these. The shape   //
// is 1ooo AAA 0 11 mmm rrr for Word and 1ooo AAA 1 11 mmm rrr for Long,     //
// which is exactly the ir[7:6]==11 case every ALU shape so far has been     //
// carrying a `!= 2'b11` term to exclude.                                    //
//                                                                          //
// Three things make it unlike any ALU form before it:                       //
//                                                                          //
//   - The destination is An and the write is ALWAYS 32 bits. ADDA.W         //
//     updates the whole address register, not its low half.                 //
//   - ADDA and SUBA set NO condition codes. CMPA does, and writes no        //
//     register.                                                             //
//   - The Word form SIGN-EXTENDS its source to 32 bits and then operates on //
//     the full width. Nothing here did that before: id_size has always      //
//     meant one width for both the memory access and the ALU, and for these //
//     they differ. id_sxt_w carries the difference -- Word-sized read,      //
//     Word-sized auto-increment step, Long ALU.                             //
//                                                                          //
// Memory: $0480 = 00000020, $0484 = FFF01111.                              //
//                                                                          //
//   MOVEA.L #$12340001,A1 / MOVE.L #$0000FFFE,D0                           //
//   ADDA.W  D0,A1      A1 = 1233FFFF                                       //
//   MOVEA.L #$0480,A2 / #$1000,A3                                          //
//   ADDA.L  (A2),A3    A3 = 00001020                                       //
//   MOVEA.L #$2000,A4 / #$0484,A5                                          //
//   SUBA.W  (A5)+,A4   A4 = 00002010, A5 = 00000486                        //
//   MOVEA.L #$1020,A6                                                      //
//   CMPA.L  A3,A6      sets Z, writes nothing                              //
//   ADDA.W  D0,A2      A2 = 0000047E -- and Z must survive it              //
//                                                                          //
// The two Word cases fail DIFFERENTLY, which is why both are here:          //
//                                                                          //
//   A1 catches a 16-bit writeback. $12340001 + (-2) borrows out of the low  //
//     word, so a correct 32-bit result is 1233FFFF while splicing only the  //
//     low half leaves 1234FFFF. Zero-extending also leaves 1234FFFF, so     //
//     this check says "something is wrong" without saying which.            //
//   A4 catches zero extension. $2000 - (-16) is 00002010 done properly and  //
//     FFFF2010 with a zero-extended source, while a 16-bit writeback would  //
//     still leave 00002010 here. So A4 names the bug A1 only detects.       //
//                                                                          //
// A5 is the auto-increment step: two, not four. eac_size stays Long for     //
// these instructions, so a step taken from it rather than from the          //
// effective memory size would advance by four and land on $0488.            //
//                                                                          //
// The final ADDA is placed AFTER the CMPA so that Z can be checked          //
// afterwards: ADDA setting condition codes is the plausible bug, since it   //
// shares its opcode nibble and its ALU operation with ADD, which does set   //
// them.                                                                     //
//                                                                          //
// On milestone 43's RTL none of these decode -- ir[7:6]==11 was excluded    //
// everywhere.                                                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_adda;

localparam PROG_WORDS      = 32;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
// A pseudo-random clock enable (milestone 94). Every bench in this suite
// tied ce high, and eight of the thirteen defects three rounds of external
// review found lived behind that: a cycle with ce low is a cycle that did
// not happen, and the core has to treat it that way. Driven on the falling
// edge so it is stable across every rising one, and left high until reset
// releases so the reset sequence itself is unchanged.
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
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1]  = 16'h227C;   // MOVEA.L #$12340001,A1
	dut.u_l1.mem[2]  = 16'h1234;
	dut.u_l1.mem[3]  = 16'h0001;
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$0000FFFE,D0
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'hFFFE;
	dut.u_l1.mem[7]  = 16'hD2C0;   // ADDA.W D0,A1
	dut.u_l1.mem[8]  = 16'h247C;   // MOVEA.L #$00000480,A2
	dut.u_l1.mem[9]  = 16'h0000;
	dut.u_l1.mem[10] = 16'h0480;
	dut.u_l1.mem[11] = 16'h267C;   // MOVEA.L #$00001000,A3
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h1000;
	dut.u_l1.mem[14] = 16'hD7D2;   // ADDA.L (A2),A3
	dut.u_l1.mem[15] = 16'h287C;   // MOVEA.L #$00002000,A4
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h2000;
	dut.u_l1.mem[18] = 16'h2A7C;   // MOVEA.L #$00000484,A5
	dut.u_l1.mem[19] = 16'h0000;
	dut.u_l1.mem[20] = 16'h0484;
	dut.u_l1.mem[21] = 16'h98DD;   // SUBA.W (A5)+,A4
	dut.u_l1.mem[22] = 16'h2C7C;   // MOVEA.L #$00001020,A6
	dut.u_l1.mem[23] = 16'h0000;
	dut.u_l1.mem[24] = 16'h1020;
	dut.u_l1.mem[25] = 16'hBDCB;   // CMPA.L A3,A6
	dut.u_l1.mem[26] = 16'hD4C0;   // ADDA.W D0,A2

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000020
	dut.u_l1.mem[65] = 16'h0020;
	dut.u_l1.mem[66] = 16'hFFF0;   // $0484 = FFF01111
	dut.u_l1.mem[67] = 16'h1111;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.areg[1] !== 32'h1233_FFFF) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 1233ffff (ADDA.W must write all 32 bits; a low-word splice leaves 1234ffff)",
		         dut.u_cpu.u_regfile.areg[1]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_1020) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 00001020 (ADDA.L (A2),A3)", dut.u_cpu.u_regfile.areg[3]);
	end
	if (dut.u_cpu.u_regfile.areg[4] !== 32'h0000_2010) begin
		errors = errors + 1;
		$display("FAIL: A4 = %h, expected 00002010 (SUBA.W must SIGN-extend; zero-extending leaves ffff2010)",
		         dut.u_cpu.u_regfile.areg[4]);
	end
	if (dut.u_cpu.u_regfile.areg[5] !== 32'h0000_0486) begin
		errors = errors + 1;
		$display("FAIL: A5 = %h, expected 00000486 (a Word postincrement steps by 2; from eac_size it would reach 0488)",
		         dut.u_cpu.u_regfile.areg[5]);
	end
	if (dut.u_cpu.u_regfile.areg[6] !== 32'h0000_1020) begin
		errors = errors + 1;
		$display("FAIL: A6 = %h, expected 00001020 (CMPA must not write its result back)",
		         dut.u_cpu.u_regfile.areg[6]);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_047E) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 0000047e (ADDA.W D0,A2 after the CMPA)", dut.u_cpu.u_regfile.areg[2]);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. The CMPA set Z; the ADDA that follows it
	// must leave the flags alone, which ADD in the same nibble does not.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMPA sets Z; the ADDA after it must set nothing)",
		         dbg_ccr[3:0]);
	end

	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid ||
	    dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
		errors = errors + 1;
		$display("FAIL: a stage is still valid after the program should have drained");
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
