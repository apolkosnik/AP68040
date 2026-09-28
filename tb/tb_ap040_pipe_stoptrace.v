//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 110: a traced STOP)  //
//                                                                          //
// tb_ap040_pipe_stoptrace.v - vector 9 instead of a halt                    //
//                                                                          //
// ap040_core.v:6622's S_STOP_LD makes its trace decision BEFORE it reaches  //
// S_STOPPED: a traced STOP raises vector 9 and the handler runs, and the    //
// machine is not stopped at all. This core stopped first and traced from    //
// inside the halt, so the handler's FIRST instruction executed repeatedly   //
// on the circulating fetch word and its second never executed.             //
//                                                                          //
//   A7 = $900, SR = $A700 so T1 is set                                     //
//   STOP #$2500 at $412, its extension word at $414                        //
//                                                                          //
// D4 and D3 are the two handler instructions. Both must run exactly once,   //
// which is the part a halted core cannot do. The stacked PC must be $416,   //
// the word after the extension -- the prior RTL stacked whichever prefetched//
// word survived the flush instead.                                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stoptrace;

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
wire [31:0] dbg_d3, dbg_d4, dbg_d5, dbg_commits;
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
	.dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5),
	.dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr),
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
	dut.u_l1.mem[ 5] = 16'h223C;   // MOVE.L #$0000A700,D1
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'hA700;
	dut.u_l1.mem[ 8] = 16'h46C1;   // MOVE D1,SR  -- T1 set
	dut.u_l1.mem[ 9] = 16'h4E72;   // STOP #$2500  at $412
	dut.u_l1.mem[10] = 16'h2500;   //   extension  at $414

	// The tail, which the trace handler's own loop keeps us out of.
	for (i = 11; i < 27; i = i + 1)
		dut.u_l1.mem[i] = 16'h5285;   // ADDQ.L #1,D5

	// Trace handler (vector 9) at $700. BOTH instructions must run.
	dut.u_l1.mem[384] = 16'h7866;  // MOVEQ #$66,D4
	dut.u_l1.mem[385] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[386] = 16'h60FE;  // BRA.B -2

	// Vector 9 -> $700.
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

	if (dbg_d4 !== 32'h0000_0066) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000066 (the trace handler must run)", dbg_d4);
	end
	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the handler's SECOND instruction must run too; a halted core repeats only the first)",
		         dbg_d3);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000 (the trace must divert before the tail)", dbg_d5);
	end
	if (dut.u_l1.mem[636] !== 16'h0416) begin
		errors = errors + 1;
		$display("FAIL: stacked PC low = %04x, expected 0416 (the word after STOP's extension, not whichever prefetch survived the flush)",
		         dut.u_l1.mem[636]);
	end
	if (dut.u_l1.mem[637] !== 16'h2024) begin
		errors = errors + 1;
		$display("FAIL: stacked format/vector = %04x, expected 2024 (format 2, vector 9)",
		         dut.u_l1.mem[637]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
