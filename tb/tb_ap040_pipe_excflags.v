//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 96: the flags a   //
// fault leaves behind)                                                     //
//                                                                          //
// tb_ap040_pipe_excflags.v - the frame and the handler must agree          //
//                                                                          //
// A fault has flag effects of its own, and they are architectural state:   //
// CHK sets N from its comparison, and a divide by zero clears C while      //
// leaving X, N, Z and V alone. Both are what rtl/ap040/ap040_core.v does,  //
// and it is the core that passes the cputest corpus.                       //
//                                                                          //
// CHK sets C as well, which this bench did not check until milestone 112:  //
// a trapping CHK sets it for a negative operand against a non-negative     //
// bound, for an operand at or above a non-negative bound, and for a        //
// negative operand above a negative bound. -1 against 20 is the first      //
// case, so the frame carries N and C: $2709, not $2708.                    //
//                                                                          //
// This core applied CHK's N to the STACKED status word and not to the      //
// register the handler runs with, so the two disagreed: a negative operand //
// with N clear on entry stacked $2708 and entered the handler with $2700.  //
// A handler that branches on its own flags and one that reads them out of  //
// the frame would take different paths. The divide cleared C in neither.   //
//                                                                          //
//   ORI #$1F,CCR ; CHK D1,D0 with D0 negative   N set in both              //
//   the handler branches on N, and the frame's own word is read back       //
//                                                                          //
// The handler's FIRST instruction is the branch, because anything else     //
// would write the flags before they could be read. The stacked word is     //
// read out of memory by the bench, so the two claims are independent.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_excflags;

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
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (A7 = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$FFFFFFFF,D0   (negative)
	dut.u_l1.mem[ 7] = 16'hFFFF;
	dut.u_l1.mem[ 8] = 16'hFFFF;
	dut.u_l1.mem[ 9] = 16'h7214;   // MOVEQ #20,D1   (the bound; clears N)
	dut.u_l1.mem[10] = 16'h4181;   // CHK.W D1,D0    -- negative, so vector 6
	dut.u_l1.mem[11] = 16'h4E71;   // NOP

	// CHK handler @ word idx 384 (byte $700). The branch comes FIRST.
	dut.u_l1.mem[384] = 16'h6A02;  // BPL.B -> skip the marker if N is CLEAR
	dut.u_l1.mem[385] = 16'h7633;  // MOVEQ #$33,D3   (N was set, as it must be)
	dut.u_l1.mem[386] = 16'h4E71;  // NOP

	// Vector 6 (CHK) -> $700.
	dut.u_l1.mem[3596] = 16'h0000;  dut.u_l1.mem[3597] = 16'h0700;

	// $0FF4, where the format $2 frame begins.
	dut.u_l1.mem[1530] = 16'h9999;  dut.u_l1.mem[1531] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033. The CHK handler's FIRST instruction branched on N and found it clear. CHK sets N from its comparison, and the operand was negative -- the status register the handler runs with has to say so, not just the word on the stack.",
		         dbg_d3);
	end
	if (dut.u_l1.mem[1530] !== 16'h2709) begin
		errors = errors + 1;
		$display("FAIL: the stacked status word = %h, expected 2709 (supervisor, interrupts masked, N and C set by the CHK)",
		         dut.u_l1.mem[1530]);
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
