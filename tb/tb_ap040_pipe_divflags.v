//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 96: the flags a   //
// divide by zero leaves)                                                   //
//                                                                          //
// tb_ap040_pipe_divflags.v - C is cleared, the rest are not touched        //
//                                                                          //
// DIVU and DIVS by zero clear C and leave X, N, Z and V exactly as they    //
// were, then take vector 5. rtl/ap040/ap040_core.v does that explicitly    //
// and it is the core that passes the cputest corpus; this one left C       //
// alone, so both the frame and the handler carried a flag the instruction  //
// had cleared.                                                             //
//                                                                          //
//   ORI #$1F,CCR    every condition code set                               //
//   DIVU.W D2,D0 with D2 = 0                                               //
//                                                                          //
// The stacked word must be $271E and the handler's own C must be clear:    //
// the four other flags survive, so a fix that cleared the whole            //
// condition-code byte would fail here rather than pass.                    //
//                                                                          //
// The handler's FIRST instruction is the branch, because anything else     //
// would write the flags before they could be read.                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_divflags;

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
	dut.u_l1.mem[ 6] = 16'h7400;   // MOVEQ #0,D2    (the divisor)
	dut.u_l1.mem[ 7] = 16'h203C;   // MOVE.L #$00000064,D0  (100)
	dut.u_l1.mem[ 8] = 16'h0000;
	dut.u_l1.mem[ 9] = 16'h0064;
	dut.u_l1.mem[10] = 16'h003C;   // ORI #$001F,CCR  -- every flag set
	dut.u_l1.mem[11] = 16'h001F;
	dut.u_l1.mem[12] = 16'h80C2;   // DIVU.W D2,D0   -- vector 5
	dut.u_l1.mem[13] = 16'h4E71;   // NOP

	// Divide-by-zero handler @ word idx 384 (byte $700). The branch first.
	dut.u_l1.mem[384] = 16'h6502;  // BCS.B -> skip the marker if C is SET
	dut.u_l1.mem[385] = 16'h7833;  // MOVEQ #$33,D4   (C was clear, as it must be)
	dut.u_l1.mem[386] = 16'h4E71;  // NOP

	// Vector 5 -> $700.
	dut.u_l1.mem[3594] = 16'h0000;  dut.u_l1.mem[3595] = 16'h0700;

	// $0FF4, where the format $2 frame begins.
	dut.u_l1.mem[1530] = 16'h9999;  dut.u_l1.mem[1531] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000033. The handler's FIRST instruction branched on C and found it SET. A divide by zero clears C before it takes its vector.",
		         dbg_d4);
	end
	if (dut.u_l1.mem[1530] !== 16'h271E) begin
		errors = errors + 1;
		$display("FAIL: the stacked status word = %h, expected 271e (C cleared by the divide, X/N/Z/V left as the ORI set them)",
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
