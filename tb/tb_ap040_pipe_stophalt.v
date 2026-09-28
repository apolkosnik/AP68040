//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 110: a stopped       //
// machine issues nothing)                                                   //
//                                                                          //
// tb_ap040_pipe_stophalt.v - the halt itself, not the SR it loaded          //
//                                                                          //
// tb_ap040_pipe_stop.v proves STOP is privileged, that a supervisor STOP    //
// replaces SR, and that the privilege check happens before the SR write.    //
// It did NOT prove the halt, through two separate attempts at a poison, and //
// this bench exists because of that.                                       //
//                                                                          //
// Milestone 108 stopped the machine by holding the fetch stage's stall.     //
// if_valid in ap040_inst_fetch.v is if_pend && l1_rvalid_a and never        //
// consults stall_in -- that only blocks `advance` -- so a stalled fetch     //
// still presents a VALID word and decode consumes it again and again. STOP  //
// flushes on the way out and that flush fetches one more word, which is the //
// one that circulates. A store tail posted 796 writes after the STOP.       //
//                                                                          //
// The shape matters. The milestone-108 bench put its STOP inside a handler  //
// reached through a privilege trap and could not reproduce this at all,     //
// even with a tail of ADDQ; the defect wants the STOP in a plain mainline   //
// flow, which is what this bench is:                                        //
//                                                                          //
//   A7 = $900, then STOP #$2500 in supervisor mode                         //
//   a tail of ADDQ.L #1,D5, sixteen words of it                            //
//                                                                          //
// Every word of the tail has an observable effect, so whichever one         //
// survives to circulate is counted in D5. And dbg_commits is sampled twice: //
// a stopped machine retires nothing between the samples, whatever it is     //
// fetching. Either check alone would have caught milestone 108; both are    //
// here because two previous poisons each looked sufficient as well.         //
//                                                                          //
// PROG_WORDS is the fetch stage's instruction ISSUE budget and is set high  //
// on purpose: too low a budget stops the machine by itself and the bench    //
// would pass for that reason instead.                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stophalt;

localparam PROG_WORDS      = 4000;
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
wire [31:0] dbg_d5, dbg_commits;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),
	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_d5 (dbg_d5), .dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr),
	.dbg_commits(dbg_commits)
);

integer errors = 0;
integer i;
integer commits_early;

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h203C;   // MOVE.L #$00000900,D0
	dut.u_l1.mem[ 1] = 16'h0000;
	dut.u_l1.mem[ 2] = 16'h0900;
	dut.u_l1.mem[ 3] = 16'h4E7B;   // MOVEC D0,ISP    (A7 = $900)
	dut.u_l1.mem[ 4] = 16'h0804;
	dut.u_l1.mem[ 5] = 16'h4E72;   // STOP #$2500  -- supervisor, T bits clear
	dut.u_l1.mem[ 6] = 16'h2500;

	// The tail. Every word of it counts itself into D5 if it ever runs.
	for (i = 7; i < 23; i = i + 1)
		dut.u_l1.mem[i] = 16'h5285;   // ADDQ.L #1,D5
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat (400 * `AP040_PIPE_WAIT_SCALE) @(posedge clk);
	commits_early = dbg_commits;
	repeat (1600 * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_sr !== 16'h2500) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected 2500 (the STOP must load its immediate)", dbg_sr);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000 (a stopped machine runs no ADDQ; this counts the ones it ran)",
		         dbg_d5);
	end
	if (dbg_commits !== commits_early) begin
		errors = errors + 1;
		$display("FAIL: commits went %0d -> %0d after the STOP; a stopped machine retires nothing",
		         commits_early, dbg_commits);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
