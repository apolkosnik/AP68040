//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 16: RTS, RTE)       //
//                                                                          //
// tb_ap040_pipe_rts_rte.v - the return half of BSR/JSR's push and          //
// illegal/TRAP/priv's exception-entry push                                 //
//                                                                          //
// Two chained round trips, deliberately NOT tested in isolation -- the      //
// whole point of this milestone is proving this pipeline can actually        //
// RETURN, not just enter:                                                    //
//                                                                          //
// Phase A -- plain BSR/RTS, entirely in supervisor mode. A subroutine call     //
// and return, verified three ways: the subroutine itself ran (D1), execution    //
// actually resumed at the CORRECT return address afterward (D2 -- reachable      //
// only via RTS's own pop, never by falling into it any other way), and A7        //
// (ISP, still supervisor here) is bit-for-bit back to its PRE-call value once     //
// the push and the pop have both happened -- a real round-trip proof, not just    //
// "the pop ran." The subroutine itself lives at a SEPARATE address (word idx       //
// 20), deliberately not adjacent to the return point -- an earlier draft placed     //
// them next to each other, and RTS's own return fell straight through back INTO     //
// the subroutine body a second time, an easy trap for any future test reusing        //
// this pattern to fall into again.                                                    //
//                                                                          //
// Phase B -- TRAP while in USER mode, RTE back out. Drops to user mode (the      //
// only mechanism this pipeline has -- see tb_ap040_pipe_sup.v), then executes      //
// TRAP #5 (never privileged) FROM user mode: the frame must land on ISP (the       //
// supervisor stack), NOT USP, per milestone 15's own fix -- verified here for       //
// the first time with an ACTUAL RTE consuming that frame afterward, not just        //
// inspecting its pushed bytes. The handler runs a marker, then RTE pops SR+PC:       //
// D4 proves PC was restored to the exact post-TRAP address (unreachable any          //
// other way -- TRAP never falls through); SR.S==0 at the end proves the ORIGINAL      //
// (user) mode was restored, not just "some" mode; a THIRD, POST-RTE BSR proves        //
// USP banking still works correctly afterward, not just that RTE "looked" done         //
// -- its push must land on USP ($50, seeded before dropping to user mode), and          //
// ISP must end up EXACTLY back at its original value ($600) once BOTH round               //
// trips (BSR/RTS -- net zero -- and TRAP/RTE -- also net zero) have completed.            //
//                                                                          //
// Word layout (PC_RESET-relative):                                         //
//  0: NOP                                                                 //
//  1: BSR.B <+$24>     (0x6124)  -> SUBR (word idx 20), return addr = idx 2's //
//  2: MOVEQ #$22,D2    (0x7422)  after-return marker: must run                //
//  3: MOVEQ #$00,D3    (0x7600)                                                //
//  4: MOVE D3,SR       (0x46C3)  drop to user mode                             //
//  5: TRAP #5          (0x4E45)  vector 37; own addr $40A, return addr $40C     //
//  6: MOVEQ #$44,D4    (0x7844)  post-RTE marker: must run, reachable ONLY       //
//                                via RTE's own PC restore                         //
//  7: BSR.B <+2>       (0x6102)  -> target idx 9 (post-RTE USP banking check)      //
//  8: MOVEQ #$88,D5    (0x7A88)  poison: must NOT run                               //
//  9: MOVEQ #$66,D6    (0x7C66)  target: must run                                    //
//                                                                          //
//  Subroutine @ word idx 20 (byte $428), deliberately separate from idx 2 --   //
//  see header:                                                                  //
//  20: MOVEQ #$11,D1   (0x7211)  subroutine marker: must run                     //
//  21: RTS             (0x4E75)                                                   //
//                                                                          //
//  TRAP #5 handler @ byte $500 (word idx 128):                             //
//   MOVEQ #$55,D7      (marker: handler ran)                                //
//   RTE                (pops SR+PC, restores user mode, redirects to idx 6)   //
//                                                                          //
//  Vector table: vector 37 -> $500 (word idx 3658/3659, same PC_RESET-        //
//  relative wraparound convention every exception test since milestone 14      //
//  has used).                                                                   //
//                                                                          //
//  A7 starts at $600 (ISP, supervisor throughout phase A); USP is seeded to     //
//  $50 BEFORE dropping to user mode (a hierarchical poke, same as ISP -- not     //
//  MOVEC, to keep this test focused on RTS/RTE rather than re-exercising           //
//  MOVEC's own already-tested mechanism).                                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rts_rte;

localparam PROG_WORDS      = 40;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3;
wire [31:0] dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;
wire [15:0] dbg_sr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
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
	.dbg_ccr(dbg_ccr), .dbg_sr(dbg_sr)
);

integer errors = 0;

initial begin
	#1;
	// Main program.
	dut.u_l1.mem[ 1] = 16'h6124;  // BSR.B <+$24> -> SUBR (idx 20)
	dut.u_l1.mem[ 2] = 16'h7422;  // MOVEQ #$22,D2 (after-return marker)
	dut.u_l1.mem[ 3] = 16'h7600;  // MOVEQ #$00,D3
	dut.u_l1.mem[ 4] = 16'h46C3;  // MOVE D3,SR (drop to user mode)
	dut.u_l1.mem[ 5] = 16'h4E45;  // TRAP #5 (vector 37)
	dut.u_l1.mem[ 6] = 16'h7844;  // MOVEQ #$44,D4 (post-RTE marker)
	dut.u_l1.mem[ 7] = 16'h6102;  // BSR.B <+2> -> idx 9
	dut.u_l1.mem[ 8] = 16'h7A88;  // MOVEQ #$88,D5 (poison: must not run)
	dut.u_l1.mem[ 9] = 16'h7C66;  // MOVEQ #$66,D6 (target: must run)
	// A tight self-loop, not NOP padding: plain NOPs let execution free-run
	// off the end of the real program into uninitialized (all-zero =
	// illegal-instruction) memory once the generous cycle budget below
	// outlasts however many NOPs were there -- caught by exactly that
	// happening on an early draft (it wandered all the way back into the
	// subroutine a second time and corrupted the final snapshot). BRA.B
	// <-2> jumps to its own address, so nothing downstream is EVER
	// reachable again regardless of simulation length.
	dut.u_l1.mem[10] = 16'h60FE;  // BRA.B <-2> (self-loop)

	// Subroutine @ word idx 20 -- deliberately NOT adjacent to idx 2 (the
	// return point), see header.
	dut.u_l1.mem[20] = 16'h7211;  // MOVEQ #$11,D1 (subroutine marker)
	dut.u_l1.mem[21] = 16'h4E75;  // RTS
	dut.u_l1.mem[22] = 16'h4E71;  // NOP -- IF may briefly fetch past RTS
	dut.u_l1.mem[23] = 16'h4E71;  // before its own redirect resolves; see
	dut.u_l1.mem[24] = 16'h4E71;  // the idx 10-19 comment above for why
	dut.u_l1.mem[25] = 16'h4E71;  // this speculative window must never be
	                                // uninitialized (illegal) memory.

	// TRAP #5 handler @ word idx 128 (byte $500).
	dut.u_l1.mem[128] = 16'h7E55;  // MOVEQ #$55,D7
	dut.u_l1.mem[129] = 16'h4E73;  // RTE
	dut.u_l1.mem[130] = 16'h4E71;  // NOP -- same reasoning, for RTE's own
	dut.u_l1.mem[131] = 16'h4E71;  // brief speculative window
	dut.u_l1.mem[132] = 16'h4E71;
	dut.u_l1.mem[133] = 16'h4E71;

	// Vector table: vector 37 -> $500.
	dut.u_l1.mem[3658] = 16'h0000;
	dut.u_l1.mem[3659] = 16'h0500;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// See tb_ap040_pipe_move_mem.v's header for why the poke must land
	// here, past the reset edge's own NBA region.
	dut.u_regfile.isp = 32'h0000_0600;
	dut.u_regfile.usp = 32'h0000_0050;

	repeat (PROG_WORDS + 100) @(posedge clk);

	// -------------------------------------------------- Phase A: BSR/RTS
	if (dbg_d1 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000011 (BSR's subroutine did not run)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0022) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000022 (RTS did not resume at the correct return address)", dbg_d2);
	end

	// -------------------------------------------------- Phase B: TRAP/RTE
	if (dbg_d4 !== 32'h0000_0044) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000044 (RTE did not restore PC correctly -- this address is unreachable any other way)", dbg_d4);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000 (poison ran -- BSR's redirect broken after the RTE round trip)", dbg_d5);
	end
	if (dbg_d6 !== 32'h0000_0066) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 00000066 (post-RTE BSR target did not run)", dbg_d6);
	end
	if (dbg_d7 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D7 = %h, expected 00000055 (TRAP handler did not run)", dbg_d7);
	end
	// Full register, not just bit 13: the SR value at the moment of the
	// fault was genuinely all-zero (D3=0 before MOVE-to-SR, nothing since
	// touches IPL/M/T), so this is a real, known constant, not just a
	// convenient bit to spot-check -- checking only SR.S let a real
	// mutation (exe_sr_data_c sourcing RTE's restore from the wrong field)
	// slip through uncaught, since that wrong value's bit 13 happened to
	// also read 0 for this program's specific addresses.
	if (dbg_sr !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected 0000 (RTE must restore the ORIGINAL (user) mode SR exactly, not just force S=0)", dbg_sr);
	end

	// TRAP #5's pushed frame, read directly out of the L1 array (still
	// there after RTE's pop -- a pop only reads and advances A7, it never
	// clears the source memory). SR=$0000 (user mode, S=0, everything else
	// 0 -- the LIVE SR at the moment of the fault, before it was forced to
	// supervisor for the handler), PC=$0000040C -- TRAP's RETURN address
	// (id_next_pc), NOT its own -- TRAP is architecturally a subroutine
	// call, the SAME distinction tb_ap040_pipe_exc.v's own milestone-14
	// writeup already established (illegal/priv stack their OWN address;
	// TRAP stacks the FOLLOWING instruction's) -- FmtVec=$0094 (format 0,
	// vector 37 -> 37*4=$94).
	if (dut.u_l1.mem[16'h00FC] !== 16'h0000 || dut.u_l1.mem[16'h00FD] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: TRAP frame word0 (SR:PChi) = %h%h, expected 00000000",
		          dut.u_l1.mem[16'h00FC], dut.u_l1.mem[16'h00FD]);
	end
	if (dut.u_l1.mem[16'h00FE] !== 16'h040C || dut.u_l1.mem[16'h00FF] !== 16'h0094) begin
		errors = errors + 1;
		$display("FAIL: TRAP frame word1 (PClo:FmtVec) = %h%h, expected 040C0094",
		          dut.u_l1.mem[16'h00FE], dut.u_l1.mem[16'h00FF]);
	end

	// The real point of this whole test: BOTH round trips (BSR/RTS -- one
	// push, one pop -- and TRAP/RTE -- one push, one pop) must leave ISP
	// EXACTLY back where it started ($600), and the frame's own stack
	// access must NEVER have touched USP (still $50 until the post-RTE
	// BSR explicitly decrements it to $4C).
	if (dut.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00000600 (both round trips together must be net zero)", dut.u_regfile.isp);
	end
	if (dut.u_regfile.usp !== 32'h0000_004C) begin
		errors = errors + 1;
		$display("FAIL: USP = %h, expected 0000004c (the post-RTE BSR's push, in user mode, must decrement USP by 4 from $50)", dut.u_regfile.usp);
	end

	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000", dbg_ccr[3:0]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
