//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 97: the other four //
// ways to reach an odd address)                                            //
//                                                                          //
// tb_ap040_pipe_oddtarget.v - BRA, BSR, RTS and RTE                        //
//                                                                          //
// An instruction address must be even. Milestone 17 made an odd JMP or JSR //
// target take an address error, and the check it added reads ea_target --  //
// which is the only target those two instructions have and the only one    //
// any other instruction does NOT have. So everything else walked into an   //
// odd address and executed whatever sat at the even one below it.          //
//                                                                          //
// Four sources, four places to look:                                       //
//                                                                          //
//   BRA/Bcc  a displacement, which decode has already turned into a        //
//   BSR      redirect -- but eac_pc + 2 + eac_imm is the same sum here,    //
//   DBcc     once decode actually puts the displacement there: for the     //
//            WORD and LONG forms it was handing EA-fetch a zero, so the    //
//            sum was the instruction's own address and never odd           //
//   RTS      the longword just loaded, in mem_lane                         //
//   RTE      the frame's own PC field, assembled from the two pops         //
//                                                                          //
// Each case jumps to an odd address and must take vector 3 instead. The    //
// handler counts and returns through JMP (A2), reloaded before each case,  //
// so one handler serves all four and the COUNT is what says every one of   //
// them trapped rather than just the first.                                 //
//                                                                          //
// The markers say the opposite thing: each is set by the instruction the   //
// handler returns to, so four markers and a count of four together mean    //
// four faults and four recoveries rather than one fault and three          //
// fall-throughs.                                                           //
//                                                                          //
// The write count is what catches BSR's other half. An odd JSR has had its //
// push suppressed since milestone 17 -- the return address of a subroutine //
// that was never entered has no business on the stack -- and BSR never     //
// got the same treatment. Fifteen writes: three setup pushes and four      //
// twelve-byte frames, three beats each. Sixteen means BSR pushed.          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_oddtarget;

localparam PROG_WORDS      = 230;
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
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_sr (dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000600,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0600;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (A7 = $0600)
	dut.u_l1.mem[ 5] = 16'h0804;

	// ---- BRA to an odd target
	dut.u_l1.mem[ 6] = 16'h247C;   // MOVEA.L #$0000041A,A2   (resume 1)
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h041A;
	dut.u_l1.mem[ 9] = 16'h6001;   // BRA.B +1  -> $0415, odd
	dut.u_l1.mem[10] = 16'h4E71;
	dut.u_l1.mem[11] = 16'h4E71;
	dut.u_l1.mem[12] = 16'h4E71;
	dut.u_l1.mem[13] = 16'h7611;   // MOVEQ #$11,D3   (resume 1)

	// ---- BSR to an odd target
	dut.u_l1.mem[14] = 16'h247C;   // MOVEA.L #$0000042A,A2   (resume 2)
	dut.u_l1.mem[15] = 16'h0000;
	dut.u_l1.mem[16] = 16'h042A;
	dut.u_l1.mem[17] = 16'h6101;   // BSR.B +1  -> $0425, odd
	dut.u_l1.mem[18] = 16'h4E71;
	dut.u_l1.mem[19] = 16'h4E71;
	dut.u_l1.mem[20] = 16'h4E71;
	dut.u_l1.mem[21] = 16'h7822;   // MOVEQ #$22,D4   (resume 2)

	// ---- RTS to an odd return address
	dut.u_l1.mem[22] = 16'h247C;   // MOVEA.L #$00000444,A2   (resume 3)
	dut.u_l1.mem[23] = 16'h0000;
	dut.u_l1.mem[24] = 16'h0444;
	dut.u_l1.mem[25] = 16'h203C;   // MOVE.L #$00000601,D0
	dut.u_l1.mem[26] = 16'h0000;
	dut.u_l1.mem[27] = 16'h0601;
	dut.u_l1.mem[28] = 16'h2F00;   // MOVE.L D0,-(A7)
	dut.u_l1.mem[29] = 16'h4E75;   // RTS
	dut.u_l1.mem[30] = 16'h4E71;
	dut.u_l1.mem[31] = 16'h4E71;
	dut.u_l1.mem[32] = 16'h4E71;
	dut.u_l1.mem[33] = 16'h4E71;
	dut.u_l1.mem[34] = 16'h7A33;   // MOVEQ #$33,D5   (resume 3)

	// ---- RTE to an odd restored PC, from a frame built by hand
	dut.u_l1.mem[35] = 16'h247C;   // MOVEA.L #$00000464,A2   (resume 4)
	dut.u_l1.mem[36] = 16'h0000;
	dut.u_l1.mem[37] = 16'h0464;
	dut.u_l1.mem[38] = 16'h203C;   // MOVE.L #$06010000,D0  ({PC_lo, FmtVec})
	dut.u_l1.mem[39] = 16'h0601;
	dut.u_l1.mem[40] = 16'h0000;
	dut.u_l1.mem[41] = 16'h2F00;   // MOVE.L D0,-(A7)
	dut.u_l1.mem[42] = 16'h203C;   // MOVE.L #$27000000,D0  ({SR, PC_hi})
	dut.u_l1.mem[43] = 16'h2700;
	dut.u_l1.mem[44] = 16'h0000;
	dut.u_l1.mem[45] = 16'h2F00;   // MOVE.L D0,-(A7)
	dut.u_l1.mem[46] = 16'h4E73;   // RTE   -> $00000601, odd
	dut.u_l1.mem[47] = 16'h4E71;
	dut.u_l1.mem[48] = 16'h4E71;
	dut.u_l1.mem[49] = 16'h4E71;
	dut.u_l1.mem[50] = 16'h7C44;   // MOVEQ #$44,D6   (resume 4)
	// ---- BRA.W to an odd target: the gathered form, whose displacement
	// ---- decode had been dropping on the floor
	dut.u_l1.mem[51] = 16'h247C;   // MOVEA.L #$00000476,A2   (resume 5)
	dut.u_l1.mem[52] = 16'h0000;
	dut.u_l1.mem[53] = 16'h0476;
	dut.u_l1.mem[54] = 16'h6000;   // BRA.W
	dut.u_l1.mem[55] = 16'h0001;   //   -> $046F, odd
	dut.u_l1.mem[56] = 16'h4E71;
	dut.u_l1.mem[57] = 16'h4E71;
	dut.u_l1.mem[58] = 16'h4E71;
	dut.u_l1.mem[59] = 16'h7055;   // MOVEQ #$55,D0   (resume 5)

	// ---- DBF to an odd target. It must not decrement its counter either.
	dut.u_l1.mem[60] = 16'h247C;   // MOVEA.L #$0000048A,A2   (resume 6)
	dut.u_l1.mem[61] = 16'h0000;
	dut.u_l1.mem[62] = 16'h048A;
	dut.u_l1.mem[63] = 16'h7E03;   // MOVEQ #3,D7   (the loop counter)
	dut.u_l1.mem[64] = 16'h51CF;   // DBF D7,
	dut.u_l1.mem[65] = 16'h0001;   //   -> $0483, odd
	dut.u_l1.mem[66] = 16'h4E71;
	dut.u_l1.mem[67] = 16'h4E71;
	dut.u_l1.mem[68] = 16'h4E71;
	dut.u_l1.mem[69] = 16'h7266;   // MOVEQ #$66,D1   (resume 6)
	// ---- an RTE whose frame has BOTH a bad format and an odd PC. The
	// ---- format error outranks the address error: the frame was never
	// ---- valid, so the PC field in it means nothing.
	dut.u_l1.mem[71] = 16'h247C;   // MOVEA.L #$000004AC,A2   (resume 7)
	dut.u_l1.mem[72] = 16'h0000;
	dut.u_l1.mem[73] = 16'h04AE;
	dut.u_l1.mem[74] = 16'h203C;   // MOVE.L #$0601F000,D0  ({PC_lo, FmtVec})
	dut.u_l1.mem[75] = 16'h0601;
	dut.u_l1.mem[76] = 16'hF000;
	dut.u_l1.mem[77] = 16'h2F00;   // MOVE.L D0,-(A7)
	dut.u_l1.mem[78] = 16'h203C;   // MOVE.L #$27000000,D0  ({SR, PC_hi})
	dut.u_l1.mem[79] = 16'h2700;
	dut.u_l1.mem[80] = 16'h0000;
	dut.u_l1.mem[81] = 16'h2F00;   // MOVE.L D0,-(A7)
	dut.u_l1.mem[82] = 16'h7055;   // MOVEQ #$55,D0  -- D0 is phase 5's marker,
	                               //   and building this frame borrowed it
	dut.u_l1.mem[83] = 16'h4E73;   // RTE   -- format $F, PC $0601
	dut.u_l1.mem[84] = 16'h4E71;
	dut.u_l1.mem[85] = 16'h4E71;
	dut.u_l1.mem[86] = 16'h4E71;
	dut.u_l1.mem[87] = 16'h4E71;   // resume 7
	// A terminal loop, not a NOP tail. The stack has been walking down
	// through seven frames by now, and a tail of NOPs let the fetcher run
	// off the end of the program and straight into them -- which is an
	// illegal instruction inside the frames, an eighth exception, and a
	// failure that looks like the core's and is the bench's.
	dut.u_l1.mem[88] = 16'h60FE;   // BRA.B -2, to itself

	// Format-error handler @ word idx 416 (byte $740). It adds ONE, so the
	// count says which vector was taken: 13 is twelve address errors' worth
	// plus this, and 14 would mean the address error won.
	dut.u_l1.mem[416] = 16'h5282;  // ADDQ.L #1,D2
	dut.u_l1.mem[417] = 16'h4ED2;  // JMP (A2)

	// Vector 14 (format error) -> $740.
	dut.u_l1.mem[3612] = 16'h0000;  dut.u_l1.mem[3613] = 16'h0740;

	// Address-error handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h5482;  // ADDQ.L #2,D2   -- MOVEQ would clear D2
	dut.u_l1.mem[385] = 16'h4ED2;  // JMP (A2)

	// Vector 3 (address error) -> $700.
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d2 !== 32'h0000_000D) begin
		errors = errors + 1;
		$display("FAIL: the handler count is %0d, expected 13 -- six address errors at two each, plus one format error. 14 means the last RTE took an address error on a frame whose format was never valid. An odd target must fault whichever instruction reached it, not only a JMP or a JSR.",
		         dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000011 (BRA to an odd target did not fault and return)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0022) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000022 (BSR to an odd target did not fault and return)", dbg_d4);
	end
	if (dbg_d5 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000033 (RTS to an odd return address did not fault and return)", dbg_d5);
	end
	if (dbg_d6 !== 32'h0000_0044) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 00000044 (RTE to an odd restored PC did not fault and return)", dbg_d6);
	end
	if (dbg_d0 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000055 (BRA.W to an odd target did not fault and return; the WORD form's displacement reaches EA-fetch through id_imm, which was zero for it)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0066) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000066 (DBF to an odd target did not fault and return)", dbg_d1);
	end
	if (dbg_d7 !== 32'h0000_0003) begin
		errors = errors + 1;
		$display("FAIL: D7 = %h, expected 00000003 (a DBcc that faults on its target must not have decremented its counter)", dbg_d7);
	end
	if (writes !== 25) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted, expected 25 (five setup pushes, six three-beat address-error frames and one two-beat format-error frame). 26 means the BSR pushed a return address for a subroutine it never entered.",
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
