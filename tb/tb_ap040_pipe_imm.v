//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 26: immediate       //
// source -- ORI, ANDI, SUBI, ADDI, EORI, CMPI)                             //
//                                                                          //
// tb_ap040_pipe_imm.v - the first source that is not a register            //
//                                                                          //
// Every instruction before this took both operands from registers. These   //
// take their source from extension words, so they are the first new class  //
// the gather machinery has carried since MOVEC. Byte and word forms use    //
// one extension word and longs use two, which is the distinction           //
// held_is_long already drew for the long branches, so the width handling   //
// is reused rather than rebuilt.                                           //
//                                                                          //
// ap040_ea_fetch.v's operand_a mux already selects eac_imm on               //
// src_a_is_imm -- the path MOVEQ has used since milestone 2 -- so decode    //
// routes the gathered word in through machinery that already existed.       //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #1,D0                7001         D0 = 00000001               //
//   2: ADDI.L #$12345678,D0       0680 1234 5678  D0 = 12345679            //
//   5: MOVEQ #-1,D1               72FF         D1 = FFFFFFFF               //
//   6: ANDI.W #$0FF0,D1           0241 0FF0    D1 = FFFF0FF0               //
//   8: MOVEQ #$7F,D2              747F         D2 = 0000007F               //
//   9: CMPI.B #$7F,D2             0C02 007F    flags only, D2 unchanged     //
//                                                                          //
// ADDI.L is the two-word gather: its immediate cannot be reached at all if  //
// the high half is dropped, and 12345679 is wrong in a visible way if the  //
// halves are assembled in the wrong order.                                 //
//                                                                          //
// ANDI.W is the one-word gather AND the size check together: the result    //
// keeps D1[31:16] as FFFF while the low half becomes 0FF0, so a Long-sized //
// decode gives 00000FF0 and a dropped word gives FFFFFFFF.                 //
//                                                                          //
// CMPI.B is the member that writes no register, the immediate-source        //
// counterpart of CMP and TST. If it were let through to commit, D2 would   //
// be overwritten with the comparison result while its flags still looked   //
// correct -- so D2 staying 0000007F is the check, and the Z flag confirms   //
// the comparison really happened rather than the instruction being skipped. //
//                                                                          //
// On milestone 25's RTL none of the three decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_imm;

localparam PROG_WORDS      = 20;
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
	dut.u_l1.mem[1]  = 16'h7001;   // MOVEQ #1,D0
	dut.u_l1.mem[2]  = 16'h0680;   // ADDI.L #$12345678,D0
	dut.u_l1.mem[3]  = 16'h1234;   //   high word
	dut.u_l1.mem[4]  = 16'h5678;   //   low word
	dut.u_l1.mem[5]  = 16'h72FF;   // MOVEQ #-1,D1
	dut.u_l1.mem[6]  = 16'h0241;   // ANDI.W #$0FF0,D1
	dut.u_l1.mem[7]  = 16'h0FF0;   //   immediate
	dut.u_l1.mem[8]  = 16'h747F;   // MOVEQ #$7F,D2
	dut.u_l1.mem[9]  = 16'h0C02;   // CMPI.B #$7F,D2
	dut.u_l1.mem[10] = 16'h007F;   //   immediate
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 24) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h1234_5679) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 12345679 (ADDI.L must gather BOTH extension words, high first)", dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_0FF0) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected FFFF0FF0 (ANDI.W: one word, and the splice keeps D1[31:16])", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_007F) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 0000007F (CMPI must not write its destination)", dbg_d2);
	end

	// CMPI.B #$7F,D2 with D2 = 7F: equal, so Z=1 and N=V=C=0.
	// dbg_ccr[3:0] is {N,Z,V,C}. This is what tells CMPI apart from an
	// instruction that was skipped entirely.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMPI of equal operands sets Z)", dbg_ccr[3:0]);
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
