//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 8: DBcc)            //
//                                                                          //
// tb_ap040_pipe_dbcc.v - DBcc decrement/compare + dynamic-write proof     //
//                                                                          //
// Two sub-cases, each isolating one half of DBcc's runtime-dependent        //
// behavior (see ap040_execute.v's header for the full semantics):           //
//                                                                          //
// Case A -- DBF D0,<self> (cond always false): a real loop, branching back  //
// to its own opcode word until the counter expires. Exercises the gather    //
// mechanism repeatedly (each iteration re-fetches the same two words),      //
// correctly-predicted-taken iterations (decode assumed taken, counter        //
// hadn't expired -- no mispredict), and the final iteration where the        //
// counter DOES expire (assumed taken, actually not -- a real misprediction   //
// recovered to eaf_next_pc, exactly like a not-taken Bcc). D0 must land on    //
// $FFFF (decremented on every iteration including the last, per the header    //
// comment: cond false always writes, independent of branch-taken).           //
//                                                                          //
//  0: NOP                                                                 //
//  1: MOVEQ #2,D0     (0x7002)  loop count                                //
//  2: DBF D0,<ext>    (0x51C8)  cond=F(1) -- opcode word; ext word @ #3     //
//  3: <ext: 0xFFFE>             disp=-2, target = addr(#2) -- branches to    //
//                                its own opcode word                        //
//  4: MOVEQ #7,D3     (0x7607)  correct exit fall-through: must run EXACTLY //
//                               once, only after the counter expires         //
//                                                                          //
// Case B -- DBT D1,<poison> (cond always true): the condition is checked     //
// BEFORE the counter, per ap040_core.v's S_DBCC1 -- a true condition must    //
// terminate immediately WITHOUT decrementing, even though decode's           //
// "assume taken" policy still speculatively redirected IF to the branch      //
// target. Two independent checks catch two independent bug classes: D1       //
// unchanged at 9 proves the write was actually suppressed (not just that     //
// the branch was); D2 == 5 proves the fall-through path -- and the             //
// misprediction recovery that reaches it -- actually ran. The poison sits     //
// behind a BRA, not directly after the correct fall-through -- milestone 5's  //
// lesson applies here too: a poison reachable by the CORRECT path too         //
// (simply because it is the next word in memory) proves nothing, and an       //
// early draft of this test made exactly that mistake.                        //
//                                                                          //
//  5: MOVEQ #9,D1     (0x7209)  case B counter -- must stay UNTOUCHED        //
//  6: DBT D1,<ext>    (0x50C9)  cond=T(0) -- opcode word; ext word @ #7      //
//  7: <ext: 0x0006>             disp=+6, target = addr(#10) (poison) --       //
//                               discarded on the correctly-working path,      //
//                               since DBT never branches                     //
//  8: MOVEQ #5,D2     (0x7405)  correct fall-through: must run               //
//  9: BRA +2 words    (0x6002)  jumps over #10 -- isolates the poison from    //
//                               the correct path                             //
// 10: MOVEQ #77,D1    (0x724D)  poison: DBT's speculative taken target --     //
//                               must NOT run (would also corrupt D1,          //
//                               independently of the write-suppression bug    //
//                               above)                                       //
// 11-19: NOP (drain)                                                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_dbcc;

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
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1]  = 16'h7002;   // MOVEQ #2,D0
	dut.u_l1.mem[2]  = 16'h51C8;   // DBF D0,<ext>
	dut.u_l1.mem[3]  = 16'hFFFE;   // ext: disp=-2, target=rom[2] (self)
	dut.u_l1.mem[4]  = 16'h7607;   // MOVEQ #7,D3 (loop exit, must run once)

	dut.u_l1.mem[5]  = 16'h7209;   // MOVEQ #9,D1
	dut.u_l1.mem[6]  = 16'h50C9;   // DBT D1,<ext>
	dut.u_l1.mem[7]  = 16'h0006;   // ext: disp=+6, target=rom[10] (poison, never taken)
	dut.u_l1.mem[8]  = 16'h7405;   // MOVEQ #5,D2 (fall-through, must run)
	dut.u_l1.mem[9]  = 16'h6002;   // BRA +2 words, jumps over rom[10]
	dut.u_l1.mem[10] = 16'h724D;   // MOVEQ #77,D1 (poison, must not run)
end

// D0's CCR is unaffected throughout: neither MOVEQ #2 nor DBF/DBT touch it
// after the CCR is last written by... there is no flag-writing instruction
// in this program at all, so ccr must simply stay at its reset value (0)
// the entire run -- the plainest possible proof that DBcc never updates it
// (a corrupting bug would show up as a nonzero ccr with no other source).
initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Case A loops 3 times (2 taken-and-correctly-predicted, 1 expired-and-
	// mispredicted) before falling through; generous margin for the repeated
	// gather + one recovery.
	repeat (PROG_WORDS + 40) @(posedge clk);

	if (dbg_d0 !== 32'h0000_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0000ffff (decrement wrong, or write not applied on the expiring iteration)", dbg_d0);
	end
	if (dbg_d3 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000007 (case A loop-exit fall-through/recovery broken)", dbg_d3);
	end
	if (dbg_d1 !== 32'h0000_0009) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000009 (case B: DBT wrongly decremented or branched -- write not suppressed on cond_true)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case B fall-through/recovery broken)", dbg_d2);
	end
	if (dbg_ccr !== 5'h0) begin
		errors = errors + 1;
		$display("FAIL: CCR = %b, expected 00000 (DBcc must never update flags)", dbg_ccr);
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
