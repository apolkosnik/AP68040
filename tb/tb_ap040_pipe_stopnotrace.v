//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 110: T0 does not     //
// trace a STOP that changes nothing)                                        //
//                                                                          //
// tb_ap040_pipe_stopnotrace.v - the half of the rule that says "do not"      //
//                                                                          //
// ap040_core.v:6622's S_STOP_LD traces a STOP under T0 only when its        //
// immediate CHANGES T1/T0/S/M or the interrupt mask. WinUAE's MakeFromSR    //
// returns before its trace decision when none of those move, so a STOP      //
// that rewrites the SR with what it already held does not generate T0.      //
//                                                                          //
// STOP rides the immediate-to-SR path through this decoder and inherited    //
// that family's unconditional T0 classification, so it traced anyway:       //
//                                                                          //
//   A7 = $900, SR = $6715 with T0 set                                      //
//   STOP #$6715 -- byte for byte what SR already holds                     //
//                                                                          //
// D4 is the vector-9 handler's marker and must stay zero. D5 and the commit //
// count prove the machine really stopped rather than merely not tracing.    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stopnotrace;

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
wire [31:0] dbg_d4, dbg_d5, dbg_commits;
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
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr),
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
	dut.u_l1.mem[ 5] = 16'h223C;   // MOVE.L #$00006715,D1
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'h6715;
	dut.u_l1.mem[ 8] = 16'h46C1;   // MOVE D1,SR   -- T0 set
	dut.u_l1.mem[ 9] = 16'h4E72;   // STOP #$6715  -- changes nothing at all
	dut.u_l1.mem[10] = 16'h6715;

	// The tail. Every word of it counts itself into D5 if it ever runs.
	for (i = 11; i < 27; i = i + 1)
		dut.u_l1.mem[i] = 16'h5285;   // ADDQ.L #1,D5

	// Vector 9's handler, which must NOT run.
	dut.u_l1.mem[384] = 16'h7866;  // MOVEQ #$66,D4
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2
	dut.u_l1.mem[3602] = 16'h0000;
	dut.u_l1.mem[3603] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat (400 * `AP040_PIPE_WAIT_SCALE) @(posedge clk);
	commits_early = dbg_commits;
	repeat (1600 * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_sr !== 16'h6715) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected 6715 (the STOP must load its immediate)", dbg_sr);
	end
	if (dbg_d4 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000000 (T0 must not trace a STOP that changes no control bit)",
		         dbg_d4);
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
