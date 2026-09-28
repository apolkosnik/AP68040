//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 90: the quick       //
// forms' other two destinations)                                           //
//                                                                          //
// tb_ap040_pipe_quickdst.v - ADDQ/SUBQ to an address register and to       //
// memory                                                                   //
//                                                                          //
// 0101 qqq d SS mmm rrr has reached only mmm=000 since milestone 27. The   //
// two destinations it lacked are both ones compiled code leans on:         //
// SUBQ.L #8,A7 is how a small stack frame opens, and ADDQ.L #1,(A0) is a   //
// counter that lives in memory.                                            //
//                                                                          //
// They are not one feature. The memory forms are milestone 89's            //
// read-modify-write with the immediate coming out of the opcode instead    //
// of an extension word, and they set condition codes. The An form writes   //
// a register, sets NONE, and is 32 bits wide whatever the size field says  //
// -- the ADDA/SUBA rule from milestone 44. A decode that treats the two    //
// alike is wrong in both directions, and this bench is built to say which. //
//                                                                          //
// Memory before:                                                           //
//   $0480 = 00000010   $0484 = 00201234   $0488 = 11223344                 //
//   $048C = 00000100   $0490 = 55555555   $0494 = 00000001                 //
//                                                                          //
//   MOVEQ   #$44,D0                                                        //
//   MOVEA.L #$0600,A7 ; SUBQ.L #8,A7 ; ADDQ.L #4,A7      A7 -> $05FC       //
//   MOVEA.L #$0000FFFF,A1 ; ADDQ.W #1,A1                 A1 -> $00010000   //
//   MOVEA.L #$00000004,A2 ; SUBQ.W #8,A2                 A2 -> $FFFFFFFC   //
//   MOVEA.L #$0700,A3 ; MOVEQ #0,D5 ; ADDQ.L #1,A3                         //
//   BNE.B +2 ; MOVEQ #$55,D6                             D6 -> $55         //
//   MOVEA.L #$0480,A4 ; ADDQ.L #1,(A4)                   -> 00000011       //
//   MOVEA.L #$0484,A5 ; SUBQ.W #8,(A5)                   -> 00181234       //
//   MOVEA.L #$0489,A6 ; ADDQ.B #1,(A6)+       -> 11233344, A6 -> $048A     //
//   MOVEA.L #$0490,A0 ; SUBQ.L #2,-(A0)       -> $048C = 000000FE,         //
//                                                A0 -> $048C               //
//   MOVEA.L #$0494,A4 ; SUBQ.L #1,(A4) ; BNE.B +2 ; MOVEQ #$33,D7          //
//                                                                          //
// Each case is aimed at a specific way this can go wrong:                  //
//                                                                          //
//   A7 = $05FC is the destination that matters most, and it is also the    //
//     A7-banking check: the quick forms reach the register through the     //
//     ordinary destination port, so the supervisor stack pointer is what   //
//     must move. SUBQ's quick field holds 0 for EIGHT, so a literal read   //
//     leaves A7 at $0600 + 4.                                              //
//   A1 = $00010000 and A2 = $FFFFFFFC are the width rule, taken from both  //
//     directions. A Word-sized operation on an address register is still   //
//     32 bits wide: a truly Word-wide ADDQ.W leaves A1 at $00000000 and a  //
//     truly Word-wide SUBQ.W leaves A2 at $0000FFFC. Byte is not tested    //
//     because it does not exist -- ADDQ.B #n,An is illegal, and the shape  //
//     excludes it.                                                         //
//   D6 = $55 is the condition-code rule for the An form. MOVEQ #0 sets Z   //
//     and the ADDQ that follows it must leave Z alone, so the BNE is not   //
//     taken and the marker runs. An An-destination decode that writes      //
//     flags clears Z on a result of $0701 and D6 stays 0.                  //
//   D7 = $33 is the same rule for the MEMORY form, which DOES write        //
//     flags: the subtraction reaches exactly zero, so Z must be set by     //
//     the memory result. The instruction before it left Z clear, so a      //
//     memory form that writes no flags leaves D7 at 0.                     //
//   $0484 keeping 1234 is the sized-lane check and $0488 uses the ODD      //
//     address $0489, so the byte lands in the low half of the high word.   //
//   $048C and A0 are the predecrement pair: the access must land at the    //
//     DECREMENTED address and both halves must agree on it. $0490 keeping  //
//     55555555 is the other half of that check -- a form that decremented  //
//     only the register would write at $0490 and leave $048C alone.        //
//   D0 = $44 is the "writes NO register" check for the memory forms.       //
//   Five write posts is what proves no form stored twice and none of the   //
//     An-destination instructions touched memory at all. Memory contents    //
//     alone cannot say that, which is the rule milestone 89 left behind.   //
//                                                                          //
// On milestone 89's RTL every one of the nine is an illegal instruction.   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_quickdst;

localparam PROG_WORDS      = 64;
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
wire [31:0] dbg_d0, dbg_d6, dbg_d7;
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

	.dbg_d0 (dbg_d0), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

// One count per store, whatever the request is held for -- see
// tb_ap040_pipe_immmem.v, where the rule this enforces was established.
integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h7044;   // MOVEQ #$44,D0
	dut.u_l1.mem[ 2] = 16'h2E7C;   // MOVEA.L #$00000600,A7
	dut.u_l1.mem[ 3] = 16'h0000;
	dut.u_l1.mem[ 4] = 16'h0600;
	dut.u_l1.mem[ 5] = 16'h518F;   // SUBQ.L #8,A7
	dut.u_l1.mem[ 6] = 16'h588F;   // ADDQ.L #4,A7
	dut.u_l1.mem[ 7] = 16'h227C;   // MOVEA.L #$0000FFFF,A1
	dut.u_l1.mem[ 8] = 16'h0000;
	dut.u_l1.mem[ 9] = 16'hFFFF;
	dut.u_l1.mem[10] = 16'h5249;   // ADDQ.W #1,A1
	dut.u_l1.mem[11] = 16'h247C;   // MOVEA.L #$00000004,A2
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0004;
	dut.u_l1.mem[14] = 16'h514A;   // SUBQ.W #8,A2
	dut.u_l1.mem[15] = 16'h267C;   // MOVEA.L #$00000700,A3
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h0700;
	dut.u_l1.mem[18] = 16'h7A00;   // MOVEQ #0,D5  (sets Z)
	dut.u_l1.mem[19] = 16'h528B;   // ADDQ.L #1,A3 (must not touch Z)
	dut.u_l1.mem[20] = 16'h6602;   // BNE.B -> index 22, skipping the marker
	dut.u_l1.mem[21] = 16'h7C55;   // MOVEQ #$55,D6 (marker)
	dut.u_l1.mem[22] = 16'h287C;   // MOVEA.L #$00000480,A4
	dut.u_l1.mem[23] = 16'h0000;
	dut.u_l1.mem[24] = 16'h0480;
	dut.u_l1.mem[25] = 16'h5294;   // ADDQ.L #1,(A4)
	dut.u_l1.mem[26] = 16'h2A7C;   // MOVEA.L #$00000484,A5
	dut.u_l1.mem[27] = 16'h0000;
	dut.u_l1.mem[28] = 16'h0484;
	dut.u_l1.mem[29] = 16'h5155;   // SUBQ.W #8,(A5)
	dut.u_l1.mem[30] = 16'h2C7C;   // MOVEA.L #$00000489,A6
	dut.u_l1.mem[31] = 16'h0000;
	dut.u_l1.mem[32] = 16'h0489;
	dut.u_l1.mem[33] = 16'h521E;   // ADDQ.B #1,(A6)+
	dut.u_l1.mem[34] = 16'h207C;   // MOVEA.L #$00000490,A0
	dut.u_l1.mem[35] = 16'h0000;
	dut.u_l1.mem[36] = 16'h0490;
	dut.u_l1.mem[37] = 16'h55A0;   // SUBQ.L #2,-(A0)
	dut.u_l1.mem[38] = 16'h287C;   // MOVEA.L #$00000494,A4
	dut.u_l1.mem[39] = 16'h0000;
	dut.u_l1.mem[40] = 16'h0494;
	dut.u_l1.mem[41] = 16'h5394;   // SUBQ.L #1,(A4)  -> exactly zero
	dut.u_l1.mem[42] = 16'h6602;   // BNE.B -> index 44, skipping the marker
	dut.u_l1.mem[43] = 16'h7E33;   // MOVEQ #$33,D7 (marker)
	dut.u_l1.mem[44] = 16'h4E71;   // NOP (drain)

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000010
	dut.u_l1.mem[65] = 16'h0010;
	dut.u_l1.mem[66] = 16'h0020;   // $0484 = 00201234
	dut.u_l1.mem[67] = 16'h1234;
	dut.u_l1.mem[68] = 16'h1122;   // $0488 = 11223344
	dut.u_l1.mem[69] = 16'h3344;
	dut.u_l1.mem[70] = 16'h0000;   // $048C = 00000100
	dut.u_l1.mem[71] = 16'h0100;
	dut.u_l1.mem[72] = 16'h5555;   // $0490 = 55555555 (must stay)
	dut.u_l1.mem[73] = 16'h5555;
	dut.u_l1.mem[74] = 16'h0000;   // $0494 = 00000001
	dut.u_l1.mem[75] = 16'h0001;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// ------------------------------------------- the An destination
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_05FC) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 000005fc (SUBQ.L #8 then ADDQ.L #4 from $0600; 00000604 means the quick field 0 was read as zero)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0001_0000) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00010000 (ADDQ.W on an address register is still 32 bits wide; 00000000 means it wrapped as a Word)",
		         dut.u_cpu.u_regfile.areg[1]);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'hFFFF_FFFC) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected fffffffc (SUBQ.W on an address register is still 32 bits wide; 0000fffc means it wrapped as a Word)",
		         dut.u_cpu.u_regfile.areg[2]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_0701) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 00000701 (ADDQ.L #1 on $0700)", dut.u_cpu.u_regfile.areg[3]);
	end
	if (dbg_d6 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 00000055 (ADDQ to an address register must set NO condition codes, so the Z from MOVEQ #0 survives it)",
		         dbg_d6);
	end

	// --------------------------------------- the memory destination
	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 00000011 (ADDQ.L #1,(A4))",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0018_1234) begin
		errors = errors + 1;
		$display("FAIL: $0484 = %h%h, expected 00181234 (SUBQ.W must leave the low half alone)",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'h1123_3344) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected 11233344 (ADDQ.B #1 at the ODD address $0489)",
		         dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if (dut.u_cpu.u_regfile.areg[6] !== 32'h0000_048A) begin
		errors = errors + 1;
		$display("FAIL: A6 = %h, expected 0000048a (the read half must still autoincrement, by one for a Byte)",
		         dut.u_cpu.u_regfile.areg[6]);
	end
	if ({dut.u_l1.mem[70], dut.u_l1.mem[71]} !== 32'h0000_00FE) begin
		errors = errors + 1;
		$display("FAIL: $048C = %h%h, expected 000000fe (SUBQ.L #2,-(A0) must land at the DECREMENTED address)",
		         dut.u_l1.mem[70], dut.u_l1.mem[71]);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_048C) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 0000048c (-(A0) must leave A0 at the decremented address)",
		         dut.u_cpu.u_regfile.areg[0]);
	end
	if ({dut.u_l1.mem[72], dut.u_l1.mem[73]} !== 32'h5555_5555) begin
		errors = errors + 1;
		$display("FAIL: $0490 = %h%h, expected 55555555 (the predecrement form must not write at the ORIGINAL A0)",
		         dut.u_l1.mem[72], dut.u_l1.mem[73]);
	end
	if ({dut.u_l1.mem[74], dut.u_l1.mem[75]} !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: $0494 = %h%h, expected 00000000 (SUBQ.L #1 on 00000001)",
		         dut.u_l1.mem[74], dut.u_l1.mem[75]);
	end
	if (dbg_d7 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D7 = %h, expected 00000033 (ADDQ/SUBQ to MEMORY must set the condition codes, and that subtraction reached exactly zero)",
		         dbg_d7);
	end
	if (dbg_d0 !== 32'h0000_0044) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000044 (a quick form with a memory destination writes NO register)",
		         dbg_d0);
	end
	if (writes !== 5) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted to the L1, expected 5 (one per memory destination; more means a store fired twice or an An-destination form reached memory)",
		         writes);
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
