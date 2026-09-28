//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 95: a fault judged //
// on an unfinished divide)                                                 //
//                                                                          //
// tb_ap040_pipe_divfault.v - CHK and divide-by-zero read a value in        //
// progress                                                                 //
//                                                                          //
// A divide holds EX for thirty-two cycles, and while it does, the value it //
// is going to produce does not exist yet -- but it is being forwarded. The //
// CHK comparison and the divisor test both read that forward and both      //
// decide immediately, so they judge whichever intermediate the iterative   //
// divider happens to be showing.                                           //
//                                                                          //
// TRAPcc's own detector has carried `!stall_in` since milestone 74 for      //
// exactly this reason. The other two never got it.                          //
//                                                                          //
//   D0 = 100, D2 = 7, D1 = 20 ; DIVU.W D2,D0 ; CHK.W D1,D0                 //
//   D0 = 100, D1 = 100        ; DIVU.W D2,D0 ; DIVU.W D0,D1                //
//                                                                          //
// Each fault instruction is IMMEDIATELY behind its divide, and the bound   //
// is a register rather than an immediate: both operands then come through  //
// the forward that is showing the divider's intermediate state.            //
//                                                                          //
// Neither instruction may trap, and both do: the CHK sees an intermediate  //
// above its bound, and the second divide sees an intermediate of zero. The //
// handler sets D4, so a run where either one trapped says which by         //
// leaving it set.                                                          //
//                                                                          //
// Both pass with a NOP in between, which is the signature: the values are  //
// right by the time anyone looks, and the fault was decided before then.   //
//--------------------------------------------------------------------------//


`timescale 1ns/1ps

module tb_ap040_pipe_divfault;

localparam PROG_WORDS      = 44;
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
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000064,D0   (100)
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0064;
	dut.u_l1.mem[ 4] = 16'h7407;   // MOVEQ #7,D2
	dut.u_l1.mem[ 5] = 16'h7214;   // MOVEQ #20,D1   (the CHK bound)
	dut.u_l1.mem[ 6] = 16'h80C2;   // DIVU.W D2,D0   -> D0 = 0002000E
	dut.u_l1.mem[ 7] = 16'h4181;   // CHK.W D1,D0    -- 14 is within 0..20
	dut.u_l1.mem[ 8] = 16'h203C;   // MOVE.L #$00000064,D0   (100 again)
	dut.u_l1.mem[ 9] = 16'h0000;
	dut.u_l1.mem[10] = 16'h0064;
	dut.u_l1.mem[11] = 16'h223C;   // MOVE.L #$00000064,D1   (100)
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0064;
	dut.u_l1.mem[14] = 16'h80C2;   // DIVU.W D2,D0   -> D0 = 0002000E
	dut.u_l1.mem[15] = 16'h82C0;   // DIVU.W D0,D1   -- divisor 14
	dut.u_l1.mem[16] = 16'h7633;   // MOVEQ #$33,D3  (the program finished)
	dut.u_l1.mem[17] = 16'h4E71;   // NOP (drain)

	// One handler for both vectors, at word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h7866;  // MOVEQ #$66,D4   (a trap happened)
	dut.u_l1.mem[385] = 16'h4E71;  // NOP

	// Vector 5 (divide by zero) and vector 6 (CHK) -> $700.
	dut.u_l1.mem[3594] = 16'h0000;  dut.u_l1.mem[3595] = 16'h0700;
	dut.u_l1.mem[3596] = 16'h0000;  dut.u_l1.mem[3597] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000000. Something trapped: a CHK against a bound its operand is within, or a divide whose divisor is 14. Both judged an intermediate of the divide still running in EX.",
		         dbg_d4);
	end
	if (dbg_d0 !== 32'h0002_000E) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0002000e (100 / 7 = 14 remainder 2)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0002_0007) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00020007 (100 / 14 = 7 remainder 2; 00000064 means the second divide trapped and never ran)", dbg_d1);
	end
	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the program must reach its end without trapping)", dbg_d3);
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
