//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 1: NOP skeleton)    //
//                                                                          //
// tb_ap040_pipe_nop.v - smoke test: push a stream of NOPs through the six  //
// pipeline stages and confirm each instruction visibly occupies IF, ID,    //
// EA-calc, EA-fetch, EX and WB in turn, one cycle apart, in PC order, then //
// drains cleanly with no stage left asserting valid.                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_nop;

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
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc)
);

integer errors = 0;
integer cycle  = 0;

// One-cycle-old snapshot of every stage's {valid,pc}, taken just before the
// edge that should carry it one stage further downstream -- this is the
// "did we actually build a pipeline" check: stage N+1 this cycle must equal
// stage N last cycle.
reg        p_if_valid,  p_id_valid,  p_eac_valid, p_eaf_valid, p_ex_valid;
reg [31:0] p_if_pc,     p_id_pc,     p_eac_pc,    p_eaf_pc,    p_ex_pc;

always @(posedge clk) begin
	if (nreset && ce) begin
		cycle = cycle + 1;

		if (dbg_id_valid !== p_if_valid) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: ID valid %b did not follow IF valid %b", cycle, dbg_id_valid, p_if_valid);
		end
		if (p_if_valid && dbg_id_pc !== p_if_pc) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: ID pc %h did not follow IF pc %h", cycle, dbg_id_pc, p_if_pc);
		end

		if (dbg_eac_valid !== p_id_valid) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: EA-calc valid %b did not follow ID valid %b", cycle, dbg_eac_valid, p_id_valid);
		end
		if (p_id_valid && dbg_eac_pc !== p_id_pc) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: EA-calc pc %h did not follow ID pc %h", cycle, dbg_eac_pc, p_id_pc);
		end

		if (dbg_eaf_valid !== p_eac_valid) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: EA-fetch valid %b did not follow EA-calc valid %b", cycle, dbg_eaf_valid, p_eac_valid);
		end
		if (p_eac_valid && dbg_eaf_pc !== p_eac_pc) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: EA-fetch pc %h did not follow EA-calc pc %h", cycle, dbg_eaf_pc, p_eac_pc);
		end

		if (dbg_ex_valid !== p_eaf_valid) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: EX valid %b did not follow EA-fetch valid %b", cycle, dbg_ex_valid, p_eaf_valid);
		end
		if (p_eaf_valid && dbg_ex_pc !== p_eaf_pc) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: EX pc %h did not follow EA-fetch pc %h", cycle, dbg_ex_pc, p_eaf_pc);
		end

		if (dbg_wb_valid !== p_ex_valid) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: WB valid %b did not follow EX valid %b", cycle, dbg_wb_valid, p_ex_valid);
		end
		if (p_ex_valid && dbg_wb_pc !== p_ex_pc) begin
			errors = errors + 1;
			$display("FAIL @cycle %0d: WB pc %h did not follow EX pc %h", cycle, dbg_wb_pc, p_ex_pc);
		end
	end

	p_if_valid  <= dbg_if_valid;  p_if_pc  <= dbg_if_pc;
	p_id_valid  <= dbg_id_valid;  p_id_pc  <= dbg_id_pc;
	p_eac_valid <= dbg_eac_valid; p_eac_pc <= dbg_eac_pc;
	p_eaf_valid <= dbg_eaf_valid; p_eaf_pc <= dbg_eaf_pc;
	p_ex_valid  <= dbg_ex_valid;  p_ex_pc  <= dbg_ex_pc;
end

// PC sequencing check: IF must issue PC_RESET, PC_RESET+2, ... in order,
// with no gaps or repeats.
integer expect_idx = 0;
always @(posedge clk) begin
	if (nreset && ce && dbg_if_valid) begin
		if (dbg_if_pc !== PC_RESET + (expect_idx << 1)) begin
			errors = errors + 1;
			$display("FAIL: IF issued pc %h out of sequence, expected %h", dbg_if_pc, PC_RESET + (expect_idx << 1));
		end
		expect_idx = expect_idx + 1;
	end
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Long enough for all PROG_WORDS instructions to clear all 6 stages
	// (PROG_WORDS + 6 cycles) plus margin.
	repeat (PROG_WORDS + 20) @(posedge clk);

	if (expect_idx !== PROG_WORDS) begin
		errors = errors + 1;
		$display("FAIL: IF issued %0d of %0d instructions", expect_idx, PROG_WORDS);
	end
	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid || dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
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
