//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25)                   //
//                                                                          //
// tb_ap040_pipe_program_local.v - the self-checking programs on the L1     //
//                                                                          //
// tb_ap040_pipe_program.v runs tests/ap040/asm's programs on the bus16    //
// top, where the 16-bit bus supplies less than a word a cycle. There the   //
// fetch rarely holds two words, so decode's phase-8 path -- a gathered     //
// instruction completed in its opcode's cycle from the fetch's second      //
// word -- seldom runs, and neither do the producer/consumer pairs it       //
// brings together that decode's gather used to keep a cycle apart. This   //
// is the same programs on ap040_pipe_core.v, the CPU on the one-cycle L1   //
// array, which supplies two words a cycle: every multi-word instruction    //
// the fetch has ready completes in one decode cycle.                       //
//                                                                          //
// Only programs that need nothing but the protocol words and no MMU run    //
// here (run_pipe_verilator.py's PROGRAMS_LOCAL): this top has no bus to    //
// raise errors or interrupts on and no table walker.                       //
//   $F100 w  failing test number       $F102 w  $600D pass / $BAD0 fail   //
// Both are read from the L1's port B as it ACCEPTS a write (wren_b and not //
// wr_busy), the array's own handshake.                                     //
//                                                                          //
// The array is 64K words from $0, so the programs' tables, stack and      //
// protocol words sit where they do on the 64 KB bus bench, and the reset   //
// vectors at $0 are read as there (RESET_VECTORS=1).                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_program_local;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

reg ce = 1;
`ifdef AP040_PIPE_CE_RANDOM
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`endif

ap040_pipe_core #(
	.PC_RESET     (32'h0000_0000),
	.PROG_WORDS   (32'h7FFF_FFFF),
	.L1_AW        (16),
	.RESET_VECTORS(1)
) dut
(
	.clk (clk), .nreset (nreset), .ce (ce), .irq_lvl (3'd0),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid(), .dbg_eac_pc(), .dbg_eaf_valid(), .dbg_eaf_pc(),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0(), .dbg_d1(), .dbg_d2(), .dbg_d3(),
	.dbg_d4(), .dbg_d5(), .dbg_d6(), .dbg_d7(),
	.dbg_ccr(), .dbg_sr(), .dbg_commits()
);

wire [31:0] dbg_pc = dut.u_cpu.u_eaf.eac_pc;

// The protocol words, as the array accepts them.
wire        wr_acc = nreset && dut.l1_wren_b && !dut.l1_wr_busy;
reg  [15:0] last_test;
integer     result;       // 0 running, 1 passed, 2 failed
always @(posedge clk)
	if (wr_acc && dut.l1_size_b == `AP040_SZ_W) begin
		if (dut.l1_addr_b == 32'h0000_F100) last_test <= dut.l1_data_b[15:0];
		if (dut.l1_addr_b == 32'h0000_F102) begin
			if (dut.l1_data_b[15:0] == 16'h600D)      result <= 1;
			else if (dut.l1_data_b[15:0] == 16'hBAD0) result <= 2;
		end
	end

reg [1023:0] prog_file;
integer prog_fd, timeout, i;

initial begin
	result = 0; last_test = 16'h0000;
	if (!$value$plusargs("prog=%s", prog_file)) begin
		$display("FAIL: missing +prog=<hexfile>");
		$finish;
	end
	$display("tb_ap040_pipe_program_local: running %0s", prog_file);
	prog_fd = $fopen(prog_file, "r");
	if (prog_fd == 0) begin
		$display("FATAL: cannot open program image %0s -- nothing to test", prog_file);
		$fatal(1);
	end
	$fclose(prog_fd);
	// Zeroed, as the bus bench's memory is, then the image.
	for (i = 0; i < 65536; i = i + 1) dut.u_l1.mem[i] = 16'h0000;
	$readmemh(prog_file, dut.u_l1.mem);
	repeat (10) @(posedge clk);
	nreset = 1;
	timeout = 0;
	while (result == 0 && timeout < 20000000 * `AP040_PIPE_WAIT_SCALE) begin
		@(posedge clk);
		timeout = timeout + 1;
	end
	if (result == 1)
		$display("ALL TESTS PASSED (%0d cycles)", timeout);
	else if (result == 2)
		$display("FAIL: test %0d failed, pc=%h", last_test, dbg_pc);
	else
		$display("FAIL: timeout, pc=%h, last test number %0d", dbg_pc, last_test);
	$finish;
end

endmodule
