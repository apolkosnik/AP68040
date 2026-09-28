//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 95: a trace frame  //
// and the instruction it has not run yet)                                  //
//                                                                          //
// tb_ap040_pipe_tracea7.v - whose A7 update is it                          //
//                                                                          //
// A trace exception belongs to the instruction that just FINISHED. The     //
// instruction behind it is sitting in EA-fetch, held, waiting for the      //
// pipeline to drain before the entry is built -- and it has not executed.  //
//                                                                          //
// Milestone 93 taught the frame's base to take this instruction's own A7   //
// update into account, which is right for a fault: a (A7)+ operand that    //
// faults has already incremented. For a TRACE entry it is wrong, because   //
// the instruction being held is not the one the exception is for, and it   //
// has done nothing at all.                                                 //
//                                                                          //
//   ISP = $1000, T1 set ; NOP (traced) ; MOVE.L (A7)+,D1 (held)            //
//                                                                          //
// The trace frame is format $2, twelve bytes, and belongs at $0FF4. The    //
// held load's increment puts it at $0FF8 instead -- four bytes adrift,     //
// which is exactly the Long step that instruction would have taken if it   //
// had run.                                                                 //
//                                                                          //
// The address of the first write is what is checked. The final stack       //
// pointer moves again as the handler is itself traced, so the frame's      //
// own base is the only stable statement.                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_tracea7;

localparam PROG_WORDS      = 32;
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
// -- a different claim from where the stack pointer ends up.
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
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (A7 = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$0000A700,D0   (T1, S, IPL 7)
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'hA700;
	dut.u_l1.mem[ 9] = 16'h46C0;   // MOVE D0,SR   -- not itself traced
	dut.u_l1.mem[10] = 16'h4E71;   // NOP          -- traced
	dut.u_l1.mem[11] = 16'h221F;   // MOVE.L (A7)+,D1  -- held for the entry
	dut.u_l1.mem[12] = 16'h4E71;   // NOP

	// Trace handler @ word idx 512 (byte $800).
	dut.u_l1.mem[512] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[513] = 16'h4E71;  // NOP

	// Vector 9 (trace) -> $800.
	dut.u_l1.mem[3602] = 16'h0000;  dut.u_l1.mem[3603] = 16'h0800;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the trace handler must run)", dbg_d3);
	end
	if (first_wr_addr !== 32'h0000_0FF4) begin
		errors = errors + 1;
		$display("FAIL: the trace frame began at %h, expected 00000ff4. The instruction held behind a traced one has not executed, and its own A7 update must not move the frame; 00000ff8 is the held MOVE.L (A7)+ stepping the pointer it never got to step.",
		         first_wr_addr);
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
