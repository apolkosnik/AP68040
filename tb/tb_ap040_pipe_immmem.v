//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 89: an immediate    //
// into memory)                                                             //
//                                                                          //
// tb_ap040_pipe_immmem.v - ORI/ANDI/SUBI/ADDI/EORI/CMPI with a memory      //
// destination                                                              //
//                                                                          //
// The last decode gap tb_ap040_pipe_dual.v found at milestone 83. The      //
// read-modify-write datapath has existed since milestone 48; what is new   //
// is that the ALU's second operand arrives as a gathered immediate rather  //
// than out of a register, which collides with the one field the gather     //
// already owns. For every other gathered form eac_imm is a DISPLACEMENT    //
// that EA-fetch adds to the base register. Here it is the operand, so the  //
// base has to be left alone -- and id_src_a_is_imm, which every            //
// register-destination immediate form sets, has to stay CLEAR, because     //
// operand_a is the ADDRESS.                                                //
//                                                                          //
// Memory before:                                                           //
//   $0480 = 00000010   $0484 = 00000020   $0488 = FFFFAAAA                 //
//   $048C = 11223344   $0490 = FF000000   $0494 = 12345678                 //
//   $0498 = 11223344                                                       //
//                                                                          //
//   MOVEQ   #$44,D0                                                        //
//   ADDI.L  #5,(A0)          A0=$0480   ->  00000015                       //
//   SUBI.L  #3,(A1)          A1=$0484   ->  0000001D                       //
//   ANDI.W  #5,(A2)          A2=$0488   ->  0005AAAA                       //
//   ORI.B   #$0F,(A3)        A3=$048D   ->  112F3344                       //
//   ADDI.B  #1,(A4)+         A4=$0491   ->  FF010000, A4 -> $0492          //
//   EORI.W  #$FFFF,-(A5)     A5=$0498   ->  1234A987, A5 -> $0496          //
//   MOVEQ   #$7F,D3 ; CMPI.L #$11223344,(A6) ; BNE.B +2 ; MOVEQ #$21,D3    //
//   MOVEQ   #$7E,D4 ; CMPI.L #$11223343,(A6) ; BCS.B +2 ; MOVEQ #$22,D4    //
//   MOVE.L  (A0),D2                                                        //
//                                                                          //
// Each case is aimed at a specific way this can go wrong:                   //
//                                                                          //
//   $0480 = 15 is the ADDRESS check as much as the store check: if the     //
//     immediate is still treated as a displacement the access lands at     //
//     $0485 and this longword never changes at all. 1A would mean the      //
//     store fired twice, the hazard tb_ap040_pipe_alurmw.v was built for.  //
//   $0484 = 1D rather than FFFFFFE3 is the operand CROSSOVER check. The    //
//     ALU computes b op a and SUBI must be memory MINUS the immediate, so  //
//     the loaded value has to be b and the immediate a -- the mirror of    //
//     the register-source direction, where the register is a.              //
//   $0488 keeping AAAA is the sized-lane check for a Word immediate, and   //
//     $048C uses the ODD address $048D so the byte lands in the low half   //
//     of the high word rather than the aligned case.                       //
//   $0490 = FF010000 and A4 = $0492 prove the autoincrement modes work:    //
//     the read half steps An exactly as it does for a register source,     //
//     and the STORE still goes to the pre-increment address.               //
//   $0494 = 1234A987 and A5 = $0496 are the same proof for -(An), where    //
//     the address is the DECREMENTED one and both halves must agree on it. //
//   $0498 unchanged is NOT enough to prove CMPI issues no store, and the   //
//     mutation that clears its nowrite flag is what showed it: the ALU     //
//     returns the DESTINATION unchanged for a compare, so a spurious       //
//     store writes the same bytes back and memory looks untouched. It is   //
//     still a real bus write, and on the bus-attached top it would be a    //
//     write cycle to an address the program only read. The bench counts    //
//     the writes the core posts to the L1 instead: this program has        //
//     exactly six stores, and both CMPIs must add none.                    //
//     $0498 unchanged stays as the separate check that the six that DO     //
//     happen went where they were supposed to. D3 = $21 and D4 = $22 are the two comparisons' own results,   //
//     read through real branches rather than by peeking at the flags.      //
//     Both markers are reached by NOT taking a branch, and the poison      //
//     value is loaded BEFORE the compare -- the first draft put the        //
//     poison on the fall-through path and the marker after it, so the      //
//     marker ran whichever way the branch went and the check was           //
//     vacuous. The mutation that crosses CMPI's operands back over is      //
//     what found that, having passed the bench.                            //
//                                                                          //
//     The equal comparison alone cannot see a REVERSED compare -- a-b and  //
//     b-a are both zero -- so the second one is off by one and tests C,    //
//     the borrow: memory minus the immediate is +1 and clears it, the      //
//     reverse is -1 and sets it.                                           //
//   D0 = $44 is the "writes NO register" check, and it is not idle: these  //
//     forms reach the completion block through held_is_imm, whose other    //
//     members all write the register named by the opcode. D0 is where a    //
//     leaked write would land.                                             //
//                                                                          //
// The final MOVE.L reads $0480 back through the ordinary load path, so the //
// store is confirmed by the pipeline and not only by the bench peeking at  //
// the array.                                                               //
//                                                                          //
// On milestone 88's RTL all seven are illegal instructions.                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_immmem;

localparam PROG_WORDS      = 56;
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
wire [31:0] dbg_d0, dbg_d2, dbg_d3, dbg_d4;
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

	.dbg_d0 (dbg_d0), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

// Every store this core issues posts to the L1's one-entry write buffer,
// and a post is exactly the cycle wren_b is accepted with the buffer empty
// -- one per store, however many cycles the request is held for. Six is
// the whole program's count: the six read-modify-writes and nothing else.
integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0480;
	dut.u_l1.mem[ 4] = 16'h7044;   // MOVEQ #$44,D0
	dut.u_l1.mem[ 5] = 16'h0690;   // ADDI.L #$00000005,(A0)
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'h0005;
	dut.u_l1.mem[ 8] = 16'h227C;   // MOVEA.L #$00000484,A1
	dut.u_l1.mem[ 9] = 16'h0000;
	dut.u_l1.mem[10] = 16'h0484;
	dut.u_l1.mem[11] = 16'h0491;   // SUBI.L #$00000003,(A1)
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0003;
	dut.u_l1.mem[14] = 16'h247C;   // MOVEA.L #$00000488,A2
	dut.u_l1.mem[15] = 16'h0000;
	dut.u_l1.mem[16] = 16'h0488;
	dut.u_l1.mem[17] = 16'h0252;   // ANDI.W #$0005,(A2)
	dut.u_l1.mem[18] = 16'h0005;
	dut.u_l1.mem[19] = 16'h267C;   // MOVEA.L #$0000048D,A3
	dut.u_l1.mem[20] = 16'h0000;
	dut.u_l1.mem[21] = 16'h048D;
	dut.u_l1.mem[22] = 16'h0013;   // ORI.B #$0F,(A3)
	dut.u_l1.mem[23] = 16'h000F;
	dut.u_l1.mem[24] = 16'h287C;   // MOVEA.L #$00000491,A4
	dut.u_l1.mem[25] = 16'h0000;
	dut.u_l1.mem[26] = 16'h0491;
	dut.u_l1.mem[27] = 16'h061C;   // ADDI.B #$01,(A4)+
	dut.u_l1.mem[28] = 16'h0001;
	dut.u_l1.mem[29] = 16'h2A7C;   // MOVEA.L #$00000498,A5
	dut.u_l1.mem[30] = 16'h0000;
	dut.u_l1.mem[31] = 16'h0498;
	dut.u_l1.mem[32] = 16'h0A65;   // EORI.W #$FFFF,-(A5)
	dut.u_l1.mem[33] = 16'hFFFF;
	dut.u_l1.mem[34] = 16'h2C7C;   // MOVEA.L #$00000498,A6
	dut.u_l1.mem[35] = 16'h0000;
	dut.u_l1.mem[36] = 16'h0498;
	dut.u_l1.mem[37] = 16'h767F;   // MOVEQ #$7F,D3 (the "Z was clear" value)
	dut.u_l1.mem[38] = 16'h0C96;   // CMPI.L #$11223344,(A6)
	dut.u_l1.mem[39] = 16'h1122;
	dut.u_l1.mem[40] = 16'h3344;
	dut.u_l1.mem[41] = 16'h6602;   // BNE.B -> index 43, skipping the marker
	dut.u_l1.mem[42] = 16'h7621;   // MOVEQ #$21,D3 (marker: only on equal)
	dut.u_l1.mem[43] = 16'h787E;   // MOVEQ #$7E,D4 (the "C was set" value)
	dut.u_l1.mem[44] = 16'h0C96;   // CMPI.L #$11223343,(A6)
	dut.u_l1.mem[45] = 16'h1122;
	dut.u_l1.mem[46] = 16'h3343;
	dut.u_l1.mem[47] = 16'h6502;   // BCS.B -> index 49, skipping the marker
	dut.u_l1.mem[48] = 16'h7822;   // MOVEQ #$22,D4 (marker: only on no borrow)
	dut.u_l1.mem[49] = 16'h2410;   // MOVE.L (A0),D2
	dut.u_l1.mem[50] = 16'h4E71;   // NOP (drain)

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000010
	dut.u_l1.mem[65] = 16'h0010;
	dut.u_l1.mem[66] = 16'h0000;   // $0484 = 00000020
	dut.u_l1.mem[67] = 16'h0020;
	dut.u_l1.mem[68] = 16'hFFFF;   // $0488 = FFFFAAAA
	dut.u_l1.mem[69] = 16'hAAAA;
	dut.u_l1.mem[70] = 16'h1122;   // $048C = 11223344
	dut.u_l1.mem[71] = 16'h3344;
	dut.u_l1.mem[72] = 16'hFF00;   // $0490 = FF000000
	dut.u_l1.mem[73] = 16'h0000;
	dut.u_l1.mem[74] = 16'h1234;   // $0494 = 12345678
	dut.u_l1.mem[75] = 16'h5678;
	dut.u_l1.mem[76] = 16'h1122;   // $0498 = 11223344
	dut.u_l1.mem[77] = 16'h3344;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0000_0015) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 00000015 (ADDI.L #5,(A0); 00000010 means the immediate was added to the ADDRESS, 0000001a means the store fired twice)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0000_001D) begin
		errors = errors + 1;
		$display("FAIL: $0484 = %h%h, expected 0000001d (SUBI.L #3,(A1) is memory MINUS the immediate; ffffffe3 is the reverse)",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'h0005_AAAA) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected 0005aaaa (ANDI.W must leave the low half alone)",
		         dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if ({dut.u_l1.mem[70], dut.u_l1.mem[71]} !== 32'h112F_3344) begin
		errors = errors + 1;
		$display("FAIL: $048C = %h%h, expected 112f3344 (ORI.B #$0F at the ODD address $048D)",
		         dut.u_l1.mem[70], dut.u_l1.mem[71]);
	end
	if ({dut.u_l1.mem[72], dut.u_l1.mem[73]} !== 32'hFF01_0000) begin
		errors = errors + 1;
		$display("FAIL: $0490 = %h%h, expected ff010000 (ADDI.B #1,(A4)+ writes the byte at $0491 only)",
		         dut.u_l1.mem[72], dut.u_l1.mem[73]);
	end
	if (dut.u_cpu.u_regfile.areg[4] !== 32'h0000_0492) begin
		errors = errors + 1;
		$display("FAIL: A4 = %h, expected 00000492 (the read half must still autoincrement)",
		         dut.u_cpu.u_regfile.areg[4]);
	end
	if ({dut.u_l1.mem[74], dut.u_l1.mem[75]} !== 32'h1234_A987) begin
		errors = errors + 1;
		$display("FAIL: $0494 = %h%h, expected 1234a987 (EORI.W #$FFFF,-(A5) must land at the DECREMENTED address $0496)",
		         dut.u_l1.mem[74], dut.u_l1.mem[75]);
	end
	if (dut.u_cpu.u_regfile.areg[5] !== 32'h0000_0496) begin
		errors = errors + 1;
		$display("FAIL: A5 = %h, expected 00000496 (-(A5) must leave A5 at the decremented address)",
		         dut.u_cpu.u_regfile.areg[5]);
	end
	if ({dut.u_l1.mem[76], dut.u_l1.mem[77]} !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: $0498 = %h%h, expected 11223344 (CMPI must not write memory)",
		         dut.u_l1.mem[76], dut.u_l1.mem[77]);
	end
	if (dbg_d3 !== 32'h0000_0021) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000021 (0000007f means CMPI.L #$11223344,(A6) did not set Z on equal values)",
		         dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0022) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000022 (0000007e means CMPI.L #$11223343,(A6) borrowed -- the compare ran immediate MINUS memory)",
		         dbg_d4);
	end
	if (dbg_d0 !== 32'h0000_0044) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000044 (an immediate with a memory destination writes NO register)",
		         dbg_d0);
	end
	if (writes !== 6) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted to the L1, expected 6 (one per read-modify-write; 8 means the two CMPIs stored, 7 means one store fired twice)",
		         writes);
	end
	if (dbg_d2 !== 32'h0000_0015) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000015 (MOVE.L (A0),D2 must read the stored value back)", dbg_d2);
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
