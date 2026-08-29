//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 9b: MOVE.L (An),Dn) //
//                                                                          //
// tb_ap040_pipe_move_mem.v - first memory-referencing instruction proof   //
//                                                                          //
// The point of this test, beyond "does MOVE.L (An),Dn load the right       //
// value": prove the pipeline's first genuine stall (ap040_ea_fetch.v's     //
// mem_issue/mem_complete FSM, see its header) actually holds IF/ID/EA-calc  //
// for exactly one cycle and resumes cleanly -- neither losing nor           //
// duplicating the instruction immediately behind it. D1 (written by a       //
// MOVEQ placed directly after the memory instruction) is that proof: if      //
// the stall chain drops a cycle wrong, D1 either never gets written (the     //
// drain-check catches it) or gets written from the wrong fetched word        //
// (the exact-value check catches it).                                       //
//                                                                          //
// This also doubles as the first real exercise of the "unified L1, prefill  //
// it and use it directly" testing pattern: A0's TARGET data lives in the     //
// exact same dut.u_l1.mem[] array the program itself does, just at a         //
// different (PC_RESET-relative, see ap040_ea_fetch.v's header) index --      //
// there is no separate data-memory model to keep in sync.                    //
//                                                                          //
// A0 itself is poked directly (dut.u_regfile.areg[0]), not loaded by an      //
// instruction -- MOVEA/LEA don't exist yet (deferred, see                    //
// AP040_IMPLEMENTATION_PLAN.md section 6). This means EX-forwarding INTO      //
// the address computation (a producer instruction writing An immediately     //
// before this one reads it) is NOT exercised by this test, even though       //
// ap040_ea_fetch.v's operand_a mux already covers it mechanically (same       //
// mux every register-direct source has used since milestone 2) -- worth       //
// a real test once MOVEA/LEA exists rather than assumed correct now.         //
//                                                                          //
//  0: NOP                                                                 //
//  1: MOVE.L (A0),D0  (0x2010)  A0 = byte addr $600 (word index $100,        //
//                               PC_RESET-relative -- see header); loads      //
//                               the longword at mem[$100]:mem[$101]          //
//  2: MOVEQ #5,D1     (0x7205)  proves clean resumption after the stall      //
//  3-9: NOP (drain)                                                        //
//                                                                          //
//  data (word index $100/$101, byte addr $600): $1234_5678                  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_move_mem;

localparam PROG_WORDS      = 10;
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
	dut.u_l1.mem[1] = 16'h2010;   // MOVE.L (A0),D0
	dut.u_l1.mem[2] = 16'h7205;   // MOVEQ #5,D1

	dut.u_l1.mem[16'h100] = 16'h1234;   // data high word
	dut.u_l1.mem[16'h101] = 16'h5678;   // data low word
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// A0 is poked here, one full edge after reset deasserts -- not just
	// "after nreset=1" but past the SAME edge's NBA region: at the very
	// edge nreset=1 takes effect, ap040_pipe_regfile.v's own reset block
	// still samples nreset==0 in the active region (this poke's initial
	// block and that always block both trigger off the identical edge) and
	// schedules areg[]<=0 as a non-blocking update -- which then commits
	// AFTER this poke's blocking assignment in the same timestep, silently
	// overwriting it. A real bug this test's first draft hit (poke visible
	// for one delta, gone by the next clock read) -- unlike ap040_pipe_l1.v's
	// mem[], which has no reset logic at all, so the ROM pokes above never
	// hit this race. Waiting one more edge sidesteps it entirely.
	dut.u_regfile.areg[0] = 32'h0000_0600;   // word index $100, PC_RESET-relative

	// One extra cycle over the usual "PROG_WORDS + 20" margin for the
	// memory stall.
	repeat (PROG_WORDS + 21) @(posedge clk);

	if (dbg_d0 !== 32'h1234_5678) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 12345678 (memory read wrong -- address, word order, or the mem_issue/mem_complete FSM)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000005 (stall chain lost or duplicated the instruction behind the memory read)", dbg_d1);
	end
	// MOVE.L of a positive nonzero value: N=0,Z=0,V=0,C=0 (AP040_ALU_MOVE,
	// same as every prior MOVE.L/MOVEQ test -- see ap040_execute.v's header).
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000", dbg_ccr[3:0]);
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
