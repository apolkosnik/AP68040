//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 95: an exception   //
// behind a MOVEC)                                                          //
//                                                                          //
// tb_ap040_pipe_excmovec.v - the fourth way to change A7 before a frame    //
//                                                                          //
// Milestone 93 made the exception's verdict wait for an older write to A7  //
// to reach the register file, and listed four ways such a write can be in  //
// flight: either of EX's two result ports, either of their commits, and    //
// MOVEC's auxiliary write AT ITS COMMIT. It missed the fifth: a MOVEC      //
// still IN EX, one cycle before that commit.                               //
//                                                                          //
//   ISP = $1000 ; MOVEC D0,ISP with $1200 ; TRAP #0                        //
//                                                                          //
// The frame belongs at $11F8 and ISP must end there. With the verdict      //
// latching a cycle early it is built from $1000 instead, which puts it at  //
// $0FF8 -- on a stack the program had just stopped using.                  //
//                                                                          //
// An intervening NOP makes it pass, which is the signature of a missing    //
// interlock rather than a wrong address: the value is not lost, the        //
// exception just did not wait for it.                                      //
//                                                                          //
// The first write's address is recorded rather than inferred from ISP, so  //
// a frame that starts in the right place and a final pointer that happens  //
// to match are two separate checks.                                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_excmovec;

localparam PROG_WORDS      = 40;
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
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4;
wire [15:0] dbg_sr;
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
	.dbg_d4 (dbg_d4), .dbg_sr (dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

integer writes = 0;
// The address the core drove at the FIRST write it posted. For a program
// whose only writes are an exception frame, that is where the frame begins
// -- which is a different claim from where the stack pointer ends up, and
// both are worth checking separately.
integer first_wr_addr = -1;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy) begin
		if (first_wr_addr < 0) first_wr_addr = dut.u_cpu.l1_addr_b;
		writes = writes + 1;
	end

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP    (ISP = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00001200,D0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h1200;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D0,ISP    (ISP = $1200)
	dut.u_l1.mem[10] = 16'h0804;
	dut.u_l1.mem[11] = 16'h4E40;   // TRAP #0   -- immediately behind it
	dut.u_l1.mem[12] = 16'h4E71;   // NOP (drain)

	// TRAP #0 handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[385] = 16'h4E71;  // NOP

	// Vector 32 -> $700.
	dut.u_l1.mem[3648] = 16'h0000;
	dut.u_l1.mem[3649] = 16'h0700;

	// $11F8, where the frame belongs, and $0FF8, where it went.
	dut.u_l1.mem[1788] = 16'h9999;  dut.u_l1.mem[1789] = 16'h9999;
	dut.u_l1.mem[1532] = 16'h6666;  dut.u_l1.mem[1533] = 16'h6666;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the TRAP handler must run)", dbg_d3);
	end
	if (first_wr_addr !== 32'h0000_11F8) begin
		errors = errors + 1;
		$display("FAIL: the frame began at %h, expected 000011f8. A MOVEC to the active stack pointer that is still in EX has not reached the register file, and the exception's verdict must wait for it exactly as it waits for the other four ways A7 can be in flight.",
		         first_wr_addr);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_11F8) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 000011f8", dut.u_cpu.u_regfile.isp);
	end
	if ({dut.u_l1.mem[1532], dut.u_l1.mem[1533]} !== 32'h6666_6666) begin
		errors = errors + 1;
		$display("FAIL: $0FF8 = %h%h, expected 66666666 (the frame was built from the stack pointer MOVEC had just replaced)",
		         dut.u_l1.mem[1532], dut.u_l1.mem[1533]);
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
