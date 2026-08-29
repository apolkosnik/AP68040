//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 13: BSR, JSR)       //
//                                                                          //
// tb_ap040_pipe_jsr.v - EA-computed-target redirect + push, combined      //
//                                                                          //
// JSR is JMP's EA-target computation (tb_ap040_pipe_jmp.v) plus BSR's       //
// push/A7-decrement mechanism (tb_ap040_pipe_bsr.v) at the same time --      //
// unlike BSR, JSR's target is NOT known at decode time (it comes from a       //
// register, possibly plus a displacement), so it does NOT get Bcc/BSR's       //
// speculative decode-time redirect (held_is_jsr is explicitly excluded          //
// from redirect_from_gather -- see ap040_decode.v's header). IF instead          //
// keeps fetching sequentially past the JSR until ap040_execute.v resolves          //
// the EA and fires an unconditional misprediction, exactly like JMP.               //
//                                                                          //
// Two sub-cases, chained (case A falls straight through into case B):        //
//                                                                          //
// Case A -- JSR (A0): D1 (poison) proves the sequential-fetch-then-flush        //
// recovery works; D2 (real target) proves the EA (A0's plain value) was          //
// used. The push side proves the return address pushed to -(A7) is the           //
// address of the instruction immediately after this single-word JSR (NOT           //
// the JSR's own address, NOT the target) -- reusing operand_b/eaf_operand_b          //
// exactly as BSR does (see ap040_ea_fetch.v's header), just fed by JSR's             //
// own eac_next_pc instead of BSR's.                                               //
//                                                                          //
// Case B -- JSR (d16,A1): exercises the gather (one extension word) at the        //
// same time as the push -- D3 (poison) proves held_is_jsr's exclusion from          //
// redirect_from_gather still holds even though this instruction ALSO writes           //
// a register (unlike JMP, which writes none) -- a broken guard here could            //
// plausibly manifest as a bogus early push instead of a bogus early jump.             //
// D4 (real target) proves A1 + displacement was used, not A1 alone. The              //
// pushed return address must account for the extension word's own width               //
// (id_next_pc = PC+4, not PC+2), same distinction BSR.W's case B proved.               //
//                                                                          //
//  0: NOP                                                                 //
//  1: JSR (A0)         (0x4E90)  A0 = byte addr $406 (word index 3)         //
//  2: MOVEQ #77,D1     (0x724D)  poison A: must NOT run                     //
//  3: MOVEQ #5,D2      (0x7405)  target A: must run, falls through into --   //
//  4: JSR (d16,A1)     (0x4EA9)  A1 = byte addr $400; disp=+$0E -> EA=$40E    //
//  5: <ext: 0x000E>              (word index 7)                              //
//  6: MOVEQ #88,D3     (0x7658)  poison B: must NOT run                       //
//  7: MOVEQ #9,D4      (0x7809)  target B: must run                            //
//  8-9: NOP (drain)                                                        //
//                                                                          //
//  A7 starts at byte address $600, same seed/expectations as                //
//  tb_ap040_pipe_bsr.v (the return-address arithmetic works out identical      //
//  byte-for-byte, since both programs share the same instruction spacing):     //
//  case A pushes $404 at $5FC (word index $FE/$FF), leaves A7=$5FC; case B      //
//  pushes $40C at $5F8 (word index $FC/$FD), leaves A7=$5F8.                    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_jsr;

// Generous relative to the 8-word real program, same reasoning as
// tb_ap040_pipe_jmp.v: IF races several words past EACH JSR before
// ap040_execute.v's unconditional misprediction resolves and redirects it
// back, and this program has two JSRs to burn that budget on.
localparam PROG_WORDS      = 30;
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
	dut.u_l1.mem[1] = 16'h4E90;   // JSR (A0)
	dut.u_l1.mem[2] = 16'h724D;   // MOVEQ #77,D1 (poison A, must not run)
	dut.u_l1.mem[3] = 16'h7405;   // MOVEQ #5,D2  (target A, must run)

	dut.u_l1.mem[4] = 16'h4EA9;   // JSR (d16,A1)
	dut.u_l1.mem[5] = 16'h000E;   // ext: disp=+$0E -> word index 7
	dut.u_l1.mem[6] = 16'h7658;   // MOVEQ #88,D3 (poison B, must not run)
	dut.u_l1.mem[7] = 16'h7809;   // MOVEQ #9,D4  (target B, must run)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// See tb_ap040_pipe_move_mem.v's header for why the poke must land
	// here, past the reset edge's own NBA region.
	dut.u_regfile.areg[0] = 32'h0000_0406;
	dut.u_regfile.areg[1] = 32'h0000_0400;
	dut.u_regfile.isp     = 32'h0000_0600;

	repeat (PROG_WORDS + 25) @(posedge clk);

	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (case A poison ran -- JSR (An) misprediction/recovery broken)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case A target did not run -- JSR (An) target computation wrong)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (case B poison ran -- JSR (d16,An) gather wrongly triggered IF's speculative redirect)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0009) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000009 (case B target did not run -- displacement not added to A1)", dbg_d4);
	end
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000 (JSR must never update flags)", dbg_ccr[3:0]);
	end

	// A7 must be decremented by exactly 4 per JSR -- read directly, same
	// hierarchical-poke style used to seed it.
	if (dut.u_regfile.isp !== 32'h0000_05F8) begin
		errors = errors + 1;
		$display("FAIL: A7 (isp) = %h, expected 000005f8 (two JSRs, -4 each, from 00000600)", dut.u_regfile.isp);
	end

	// Both pushes' return addresses, read directly out of the L1 array,
	// same style as tb_ap040_pipe_bsr.v.
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
