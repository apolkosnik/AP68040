//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 13: BSR, JSR)       //
//                                                                          //
// tb_ap040_pipe_bsr.v - push + A7 decrement + always-correct redirect     //
//                                                                          //
// Two sub-cases, chained (case A's target falls straight through into case  //
// B's BSR, proving normal execution resumes cleanly, not just that the      //
// push/branch mechanism fires once):                                       //
//                                                                          //
// Case A -- BSR.B: unlike JMP, BSR reuses Bcc's speculative decode-time      //
// redirect verbatim (unconditionally taken, always correct -- see            //
// ap040_decode.v's header), so there is NO misprediction/flush here at all    //
// -- IF jumps straight to the target with zero wasted fetches. What's         //
// actually under test: the return address pushed to -(A7) is the address       //
// of the instruction immediately AFTER the BSR (not the BSR's own address,       //
// not the target's), A7 is decremented by exactly 4, and the poison            //
// immediately behind the BSR (reachable only if the "always taken" part          //
// were somehow wrong) never runs.                                                //
//                                                                          //
// Case B -- BSR.W: same mechanism, but exercises the gather (one extension     //
// word) this pipeline already built for Bcc.W/DBcc/JMP -- proving BSR's        //
// return address (id_next_pc, threaded since milestone 7) correctly accounts    //
// for the extension word's own width, not just a flat +2.                       //
//                                                                          //
//  0: NOP                                                                 //
//  1: BSR.B <+2>       (0x6102)  target = index 3                          //
//  2: MOVEQ #99,D1     (0x7263)  poison A: must NOT run                     //
//  3: MOVEQ #5,D2      (0x7405)  target A: must run, falls through into --   //
//  4: BSR.W <ext>      (0x6100)  target = index 7                            //
//  5: <ext: 0x0004>                                                          //
//  6: MOVEQ #88,D3     (0x7658)  poison B: must NOT run                       //
//  7: MOVEQ #9,D4      (0x7809)  target B: must run                            //
//  8-9: NOP (drain)                                                        //
//                                                                          //
//  A7 starts at byte address $600. Case A pushes $404 (return addr) at        //
//  $5FC (word index $FE/$FF) and leaves A7=$5FC. Case B pushes $40C at         //
//  $5F8 (word index $FC/$FD) and leaves A7=$5F8.                                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bsr;

localparam PROG_WORDS      = 20;
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
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1] = 16'h6102;   // BSR.B <+2> -> index 3
	dut.u_l1.mem[2] = 16'h7263;   // MOVEQ #99,D1 (poison A, must not run)
	dut.u_l1.mem[3] = 16'h7405;   // MOVEQ #5,D2  (target A, must run)

	dut.u_l1.mem[4] = 16'h6100;   // BSR.W <ext> -> index 7
	dut.u_l1.mem[5] = 16'h0004;   // ext: disp=+4
	dut.u_l1.mem[6] = 16'h7658;   // MOVEQ #88,D3 (poison B, must not run)
	dut.u_l1.mem[7] = 16'h7809;   // MOVEQ #9,D4  (target B, must run)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// See tb_ap040_pipe_move_mem.v's header for why the poke must land
	// here, past the reset edge's own NBA region. A7 (sr_s=1/sr_m=0
	// hardwired in ap040_pipe_core.v, so A7 always banks to ISP) is
	// ap040_pipe_regfile.v's internal `isp` register.
	dut.u_regfile.isp = 32'h0000_0600;

	repeat (PROG_WORDS + 30) @(posedge clk);

	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (case A poison ran -- BSR.B's redirect or push/stall broken)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case A target did not run)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (case B poison ran -- BSR.W's gather/redirect broken)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0009) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000009 (case B target did not run -- id_next_pc wrong for the word form?)", dbg_d4);
	end
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000 (BSR must never update flags)", dbg_ccr[3:0]);
	end

	// A7 must be decremented by exactly 4 per BSR -- read directly, same
	// hierarchical-poke style used to seed it.
	if (dut.u_regfile.isp !== 32'h0000_05F8) begin
		errors = errors + 1;
		$display("FAIL: A7 (isp) = %h, expected 000005f8 (two BSRs, -4 each, from 00000600)", dut.u_regfile.isp);
	end

	// Both pushes' return addresses, read directly out of the L1 array --
	// the same array every test already pokes program words into, so no
	// separate memory model or port-driving is needed. This checks what
	// actually landed in mem[], not through the write path's own read-
	// after-write forwarding (which only covers an UNDRAINED write, and
	// both of these have long since drained).
	if (dut.u_l1.mem[16'h0FE] !== 16'h0000 || dut.u_l1.mem[16'h0FF] !== 16'h0404) begin
		errors = errors + 1;
		$display("FAIL: case A push = %h%h, expected 00000404 (return address at A7-4, word index $FE/$FF)",
		          dut.u_l1.mem[16'h0FE], dut.u_l1.mem[16'h0FF]);
	end
	if (dut.u_l1.mem[16'h0FC] !== 16'h0000 || dut.u_l1.mem[16'h0FD] !== 16'h040C) begin
		errors = errors + 1;
		$display("FAIL: case B push = %h%h, expected 0000040c (return address at A7-8, word index $FC/$FD)",
		          dut.u_l1.mem[16'h0FC], dut.u_l1.mem[16'h0FD]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
