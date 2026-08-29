//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 11: JMP (An),       //
// JMP (d16,An))                                                            //
//                                                                          //
// tb_ap040_pipe_jmp.v - EA-computed-target redirect proof                 //
//                                                                          //
// Two sub-cases, chained one after the other (case A's target falls        //
// straight through into case B's JMP, so this also proves normal execution //
// resumes cleanly after a JMP recovery, not just that the recovery fires    //
// at all):                                                                 //
//                                                                          //
// Case A -- JMP (A0): decode never speculates a target for JMP at all (no   //
// literal displacement exists to guess with -- see ap040_decode.v's         //
// header), so IF's default behavior is to keep fetching SEQUENTIALLY past    //
// the JMP -- straight into the poison at the next word. ap040_execute.v      //
// treats this as an UNCONDITIONAL misprediction once EA resolves An's         //
// value, reusing the exact ex_mispredict/ex_recovery_pc/flush path Bcc's       //
// misprediction recovery already proved in milestone 5. D1 (the poison's        //
// destination) must stay untouched; D2 (the real target) must run.               //
//                                                                          //
// Case B -- JMP (d16,A1): same mechanism, but the target needs REAL EA          //
// arithmetic (A1 + displacement, reusing MOVE.L (d16,An),Dn's machinery          //
// wholesale -- see ap040_ea_fetch.v's header). Distinguishes two            //
// independent failure modes from case A's: D3 (poison) proves the               //
// gather's redirect-suppression guard (held_is_jmp) actually works -- a          //
// broken guard would have IF speculatively jump to the raw displacement           //
// value interpreted as an address, landing who-knows-where, not simply           //
// "keep going sequentially" the way case A's absence-of-redirect does. D4          //
// (the real target) proves the displacement was actually ADDED, not just          //
// A1 alone (which would land back on case A's own target, word index 3,           //
// re-running MOVEQ #5,D2 -- harmless-looking but NOT what D4 is checking).          //
//                                                                          //
//  0: NOP                                                                 //
//  1: JMP (A0)         (0x4ED0)  A0 = byte addr $406 (word index 3)         //
//  2: MOVEQ #77,D1     (0x724D)  poison A: must NOT run                     //
//  3: MOVEQ #5,D2      (0x7405)  target A: must run, falls through into --   //
//  4: JMP (d16,A1)     (0x4EE9)  A1 = byte addr $400; disp=+$0E -> EA=$40E    //
//  5: <ext: 0x000E>              (word index 7)                              //
//  6: MOVEQ #88,D3     (0x7658)  poison B: must NOT run                       //
//  7: MOVEQ #9,D4      (0x7809)  target B: must run                            //
//  8-9: NOP (drain)                                                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_jmp;

// Generous relative to the 8-word real program: ap040_inst_fetch.v's
// `issued` counter counts every word FETCHED, including ones later
// discarded by a misprediction flush (see its header) -- decode never
// speculates a target for JMP at all, so IF races several words past EACH
// JMP before ap040_execute.v's unconditional-misprediction resolves and
// redirects it back, and TWO JMPs in this program each burn that budget
// independently. 10 (this test's first draft) silently ran out before
// case B's post-redirect execution ever happened; caught by D4 staying 0,
// not a hang -- worth remembering for any future test chaining multiple
// JMP/Bcc mispredictions.
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
	dut.u_l1.mem[1] = 16'h4ED0;   // JMP (A0)
	dut.u_l1.mem[2] = 16'h724D;   // MOVEQ #77,D1 (poison A, must not run)
	dut.u_l1.mem[3] = 16'h7405;   // MOVEQ #5,D2  (target A, must run)

	dut.u_l1.mem[4] = 16'h4EE9;   // JMP (d16,A1)
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

	repeat (PROG_WORDS + 25) @(posedge clk);

	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (case A poison ran -- JMP (An) misprediction/recovery broken)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case A target did not run -- JMP (An) target computation wrong)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (case B poison ran -- JMP (d16,An) gather wrongly triggered IF's speculative redirect)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0009) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000009 (case B target did not run -- displacement not added to A1)", dbg_d4);
	end
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000 (JMP must never update flags)", dbg_ccr[3:0]);
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
