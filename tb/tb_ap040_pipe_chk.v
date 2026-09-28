//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 64: CHK)                  //
//                                                                          //
// tb_ap040_pipe_chk.v - bounds checking, vector 6                          //
//                                                                          //
// CHK traps if Dn's low word is negative or exceeds the bound. Its operand //
// shape is the one that exposed milestone 63's defect -- the BOUND is the  //
// <ea> side, which for a memory source is mem_lane and not operand_a -- so //
// the fault is derived from loaded data and is LATCHED from the start      //
// here, rather than being found to need it afterwards.                     //
//                                                                          //
// N is DEFINED on the two trapping paths and nowhere else: SET when the    //
// value is negative, CLEARED when it merely exceeds the bound. It is       //
// written into the STACKED SR, which is what a handler reads and what RTE  //
// restores.                                                                //
//                                                                          //
// Milestone 112 replaced this bench's old reading of the non-trapping case //
// ("undefined, so left unchanged") with the rule the cputest corpus holds  //
// the 68040 to, as rtl/ap040/ap040_core.v implements it: N tracks the      //
// value's sign on every path, C is CLEARED in bounds, and a trap sets C    //
// for a negative value against a non-negative bound or a value at or above //
// a non-negative bound. Z, V and X are never touched. Both traps here are  //
// one of those cases, so both frames must carry C; and the live C before   //
// each is clear (the MOVEQ poison clears it), so an unchanged C fails.     //
//                                                                          //
// The in-bounds rule gets a fourth CHK at the end, after ORI #$1F,CCR sets //
// every flag: CHK #30,D0 with D0 = 20 must leave X, Z and V set and N and  //
// C clear, CCR = $16. RTL that left the flags alone keeps $1F.             //
//                                                                          //
// All three outcomes run in ONE program, because the handler returns with  //
// RTE -- which milestone 54 made trustworthy:                              //
//                                                                          //
//   MOVEA.L #$0500,A0                                                      //
//   MOVE.L #-1,D0    / CHK #10,D0   traps, N must be SET                   //
//   MOVE.L #5,D0     / CHK #10,D0   must NOT trap                          //
//   MOVE.L #20,D0    / CHK #10,D0   traps, N must be CLEAR                 //
//   LSL.L #3,D0      / CHK D4,D0    24, straight behind: traps, N CLEAR    //
//   ADDQ.L #1,D0     / CHK D4,D0    5, straight behind: must NOT trap      //
//   DIVU.W #1,D0     / CHK D4,D0    $9000, straight behind: traps, N SET   //
//   MOVE.L (A1),D0   / CHK (A2),D0  7 loaded, bound 6: traps, N CLEAR      //
//   ADD.L D4,D4      / CHK D4,D0    bound 12 straight behind: no trap     //
//   ORI #$1F,CCR     / CHK #30,D0   in bounds: CCR must become $16         //
// The four behind their value's producer are the perf-1b timing fix: CHK //
// no longer judges EX's forward, and waits a bubble for the commit.      //
//                                                                          //
//   handler: MOVE.L (A7),D2    the stacked {SR, PC_hi}                     //
//            MOVE.L D2,(A0)+   filed away, one slot per trap               //
//            ADDQ.L #1,D1      count it                                    //
//            RTE                                                           //
//                                                                          //
// Filing the stacked SR to memory is what lets BOTH N values be checked.   //
// Only the last trap's flags survive in the CCR, so a bench that read the  //
// live CCR could confirm one case and would have to take the other on      //
// trust -- and the two cases are precisely what distinguishes CHK from a   //
// plain "out of range" test.                                               //
//                                                                          //
// D1 = 2 is the other half: exactly two traps, so the in-range CHK did     //
// not trap. A CHK that always trapped would give 3, and one that never     //
// did would give 0 while leaving both memory slots untouched.              //
//                                                                          //
// The MOVEQs into D3 are poison, and a mutation proved they were needed.   //
// Each CHK is preceded by a MOVE that loads D0 -- and that MOVE sets the   //
// live N from the very value CHK is about to judge. So stacking the        //
// UNMODIFIED SR gives the right N by pure coincidence, and the first       //
// version of this bench passed against RTL that never wrote N into the     //
// frame at all. Each MOVEQ now sets the live N to the OPPOSITE of what     //
// CHK must stack, so the two can no longer agree by accident.              //
//                                                                          //
// Vector 6 sits at VBR $2000 + 24, word index 3596.                        //
//                                                                          //
// On milestone 63's RTL none of the three CHKs decode.                     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_chk;

localparam PROG_WORDS      = 160;
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
	// Mainline
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00000500,A0  (where the handler files SRs)
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0500;
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$FFFFFFFF,D0   (-1)
	dut.u_l1.mem[5]  = 16'hFFFF;
	dut.u_l1.mem[6]  = 16'hFFFF;
	dut.u_l1.mem[7]  = 16'h7601;   // MOVEQ #1,D3   -- live N := 0, the OPPOSITE of CHK's
	dut.u_l1.mem[8]  = 16'h41BC;   // CHK #10,D0   -- negative: traps, stacked N must be 1
	dut.u_l1.mem[9]  = 16'h000A;
	dut.u_l1.mem[10] = 16'h203C;   // MOVE.L #$00000005,D0
	dut.u_l1.mem[11] = 16'h0000;
	dut.u_l1.mem[12] = 16'h0005;
	dut.u_l1.mem[13] = 16'h41BC;   // CHK #10,D0   -- in range: must NOT trap
	dut.u_l1.mem[14] = 16'h000A;
	dut.u_l1.mem[15] = 16'h203C;   // MOVE.L #$00000014,D0   (20)
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h0014;
	dut.u_l1.mem[18] = 16'h76FF;   // MOVEQ #-1,D3  -- live N := 1, the OPPOSITE of CHK's
	dut.u_l1.mem[19] = 16'h41BC;   // CHK #10,D0   -- over bound: traps, stacked N must be 0
	dut.u_l1.mem[20] = 16'h000A;
	// The checked register produced by the instruction straight ahead:
	// CHK waits out a bubble rather than judge EX's forward.
	dut.u_l1.mem[21] = 16'h780A;   // MOVEQ #10,D4  -- the bound, in a register: CHK D4,D0 is one
	                               //   word, and only a one-word CHK can sit straight behind
	                               //   anything (a gathered one never does)
	dut.u_l1.mem[22] = 16'h7003;   // MOVEQ #3,D0
	dut.u_l1.mem[23] = 16'hE788;   // LSL.L #3,D0   -- D0 = 24, from the shifter straight ahead
	dut.u_l1.mem[24] = 16'h4184;   // CHK D4,D0    -- over bound: trap 3, N clear, C set
	dut.u_l1.mem[25] = 16'h7004;   // MOVEQ #4,D0
	dut.u_l1.mem[26] = 16'h5280;   // ADDQ.L #1,D0  -- D0 = 5, straight ahead
	dut.u_l1.mem[27] = 16'h4184;   // CHK D4,D0    -- in range: must NOT trap
	dut.u_l1.mem[28] = 16'h203C;   // MOVE.L #$00009000,D0
	dut.u_l1.mem[29] = 16'h0000;
	dut.u_l1.mem[30] = 16'h9000;
	dut.u_l1.mem[31] = 16'h80FC;   // DIVU.W #1,D0 -- D0 = $9000, EX held while CHK waits
	dut.u_l1.mem[32] = 16'h0001;
	dut.u_l1.mem[33] = 16'h4184;   // CHK D4,D0    -- word negative: trap 4, N set, C set
	dut.u_l1.mem[34] = 16'h4E71;   // NOP
	dut.u_l1.mem[35] = 16'h4E71;   // NOP
	// The BOUND produced straight ahead (2026-09-24): CHK reads it through
	// the address view, and waits a cycle rather than judge EX's forward.
	// Read from D4's old value, 6, the in-range value 10 would trap.
	dut.u_l1.mem[36] = 16'h700A;   // MOVEQ #10,D0
	dut.u_l1.mem[37] = 16'h7806;   // MOVEQ #6,D4
	dut.u_l1.mem[38] = 16'hD884;   // ADD.L D4,D4  -- the bound, 12, from the ALU straight ahead
	dut.u_l1.mem[39] = 16'h4184;   // CHK D4,D0    -- 10 in 0..12: must NOT trap
	dut.u_l1.mem[40] = 16'h227C;   // MOVEA.L #$00000580,A1
	dut.u_l1.mem[41] = 16'h0000;
	dut.u_l1.mem[42] = 16'h0580;
	dut.u_l1.mem[43] = 16'h247C;   // MOVEA.L #$00000590,A2
	dut.u_l1.mem[44] = 16'h0000;
	dut.u_l1.mem[45] = 16'h0590;
	dut.u_l1.mem[46] = 16'h2011;   // MOVE.L (A1),D0 -- D0 = 7, loaded straight ahead
	dut.u_l1.mem[47] = 16'h4192;   // CHK (A2),D0  -- bound 6 in memory: trap 5, N clear, C set
	dut.u_l1.mem[48] = 16'h203C;   // MOVE.L #$00000014,D0   (20 again)
	dut.u_l1.mem[49] = 16'h0000;
	dut.u_l1.mem[50] = 16'h0014;
	dut.u_l1.mem[51] = 16'h003C;   // ORI #$1F,CCR  -- every flag set
	dut.u_l1.mem[52] = 16'h001F;
	dut.u_l1.mem[53] = 16'h41BC;   // CHK #30,D0   -- 20 is in bounds: CCR := $16
	dut.u_l1.mem[54] = 16'h001E;
	dut.u_l1.mem[192] = 16'h0000;  // $580: the loaded value, 7
	dut.u_l1.mem[193] = 16'h0007;
	dut.u_l1.mem[200] = 16'h0006;  // $590: the bound, 6

	// CHK handler @ word idx 512 (byte $800)
	dut.u_l1.mem[512] = 16'h2417;  // MOVE.L (A7),D2   -- stacked {SR, PC_hi}
	dut.u_l1.mem[513] = 16'h20C2;  // MOVE.L D2,(A0)+
	dut.u_l1.mem[514] = 16'h5281;  // ADDQ.L #1,D1
	dut.u_l1.mem[515] = 16'h4E73;  // RTE

	// Vector table: vector 6 -> $800 (word idx 3596/3597)
	dut.u_l1.mem[3596] = 16'h0000;
	dut.u_l1.mem[3597] = 16'h0800;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// Three exception frames are pushed and never popped (the handler
	// returns with JMP, not RTE), so A7 must point somewhere real and
	// clear of the program. See tb_ap040_pipe_move_mem.v's header for why
	// the poke has to land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 500) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d1 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: trap count D1 = %h, expected 00000005 (00000006 or more means an in-range CHK trapped)",
		         dbg_d1);
	end
	// The handler filed each stacked {SR, PC_hi} at $0500 and $0504, word
	// indices 128 and 130. SR bit 3 is N.
	if (dut.u_l1.mem[128][3] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 1 = %h, N must be SET (the value was negative)", dut.u_l1.mem[128]);
	end
	if (dut.u_l1.mem[130][3] !== 1'b0) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 2 = %h, N must be CLEAR (the value merely exceeded the bound)",
		         dut.u_l1.mem[130]);
	end
	// SR bit 0 is C. Both traps are cases that set it, and the MOVEQ before
	// each left the live C clear.
	if (dut.u_l1.mem[128][0] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 1 = %h, C must be SET (negative value, non-negative bound)",
		         dut.u_l1.mem[128]);
	end
	if (dut.u_l1.mem[130][0] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 2 = %h, C must be SET (value at or above a non-negative bound)",
		         dut.u_l1.mem[130]);
	end
	// The three traps straight behind their value's producer, at $0508,
	// $050C and $0510: over, negative, over.
	if (dut.u_l1.mem[132][3] !== 1'b0 || dut.u_l1.mem[132][0] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 3 (behind LSL.L) = %h, expected N clear and C set", dut.u_l1.mem[132]);
	end
	if (dut.u_l1.mem[134][3] !== 1'b1 || dut.u_l1.mem[134][0] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 4 (behind DIVU.W) = %h, expected N set and C set", dut.u_l1.mem[134]);
	end
	if (dut.u_l1.mem[136][3] !== 1'b0 || dut.u_l1.mem[136][0] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: stacked SR of trap 5 (behind a load) = %h, expected N clear and C set", dut.u_l1.mem[136]);
	end
	if (dbg_ccr !== 5'h16) begin
		errors = errors + 1;
		$display("FAIL: CCR = %h after the in-bounds CHK, expected 16 (X, Z, V kept; N from the value, C cleared)",
		         dbg_ccr);
	end
	if (dbg_d0 !== 32'h0000_0014) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000014 (CHK writes no register)", dbg_d0);
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
