//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: a paused clock  //
// enable)                                                                  //
//                                                                          //
// tb_ap040_pipe_cepause.v - ce low must SUSPEND the core, not corrupt it   //
//                                                                          //
// ce is how the real system runs this core slower than its clock, so a     //
// cycle with ce low has to be a cycle that did not happen. Two things here //
// did not honour that, and both are invisible with ce tied high -- which   //
// is how every other bench in this suite runs it.                          //
//                                                                          //
// The first is the commit-eligibility flag. ap040_pipe_cpu.v computes it   //
// as `ce && !ex_stall`, so it goes LOW during a disabled cycle while EX's  //
// result registers, correctly gated on ce, hold their pending value. When  //
// the enable returns, the value is still sitting in EX and is no longer    //
// eligible: the register write, the CCR write and the SR write are all     //
// dropped. It is a flag about what EX did last cycle, so it has to hold    //
// across cycles that did not happen rather than clear in them.             //
//                                                                          //
// The second is the memory request. ap040_pipe_l1.v has no ce -- correctly, //
// since memory in a real system is not gated by the CPU's enable -- so a   //
// request left asserted through a disabled cycle is seen again, and the    //
// write buffer accepts it again every time it drains.                      //
//                                                                          //
// ce alternates every cycle here, which is the harshest legal pattern: the //
// core sees a disabled cycle between every pair of enabled ones.           //
//                                                                          //
//   MOVEQ #5,D0 ; MOVEQ #6,D1 ; MOVEQ #7,D2 ; MOVEQ #8,D3                  //
//   MOVEA.L #$0480,A0 ; MOVE.L D0,(A0) ; ADDQ.L #1,D1 ; MOVE.L (A0),D4     //
//                                                                          //
// Four immediate writes, one address-register write, one store, one        //
// read-modify of a register and one load, which is enough that a dropped   //
// commit shows up as a zero rather than as a stale value. The program is   //
// otherwise ordinary; the enable is the whole experiment.                  //
//                                                                          //
// Both directions of the port are counted, because both repeat and only    //
// one of them can be seen in memory afterwards. A repeated READ leaves no  //
// trace at all in RAM, and against a device it is a FIFO drained several   //
// times or a status register cleared on read. The load asserts its request //
// for exactly one cycle when the core is not paused -- mem_pending goes up //
// behind it -- so the count is one per load whatever the memory latency.   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_cepause;

localparam PROG_WORDS      = 24;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;
// Alternate on the falling edge so ce is stable across every rising edge.
always @(negedge clk) if (nreset) ce <= ~ce;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
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
	.dbg_d4 (dbg_d4),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

integer reads = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.rd_b)
		reads = reads + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h7005;   // MOVEQ #5,D0
	dut.u_l1.mem[ 2] = 16'h7206;   // MOVEQ #6,D1
	dut.u_l1.mem[ 3] = 16'h7407;   // MOVEQ #7,D2
	dut.u_l1.mem[ 4] = 16'h7608;   // MOVEQ #8,D3
	dut.u_l1.mem[ 5] = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'h0480;
	dut.u_l1.mem[ 8] = 16'h2080;   // MOVE.L D0,(A0)
	dut.u_l1.mem[ 9] = 16'h5281;   // ADDQ.L #1,D1   -> 7
	dut.u_l1.mem[10] = 16'h2810;   // MOVE.L (A0),D4 -> 5
	dut.u_l1.mem[11] = 16'h4E71;   // NOP (drain)

	dut.u_l1.mem[64] = 16'hFFFF;   // $0480 = FFFFFFFF
	dut.u_l1.mem[65] = 16'hFFFF;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Twice the usual budget: half the cycles do not happen.
	repeat ((PROG_WORDS + 120) * 2 * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000005 (a commit pending when ce went low must still land when it returns)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000007 (MOVEQ #6 then ADDQ.L #1)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000007", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0008) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000008", dbg_d3);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000480", dut.u_cpu.u_regfile.areg[0]);
	end
	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 00000005",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if (dbg_d4 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000005 (MOVE.L (A0),D4 must read back what the store put there)", dbg_d4);
	end
	if (reads !== 1) begin
		errors = errors + 1;
		$display("FAIL: %0d read requests reached the L1, expected 1. A read left asserted across a disabled cycle is issued again, and a repeated read leaves no trace in memory at all.",
		         reads);
	end
	if (writes !== 1) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted to the L1, expected 1. A request left asserted across a disabled cycle is accepted again: memory has no ce and does not know the core was paused.",
		         writes);
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
