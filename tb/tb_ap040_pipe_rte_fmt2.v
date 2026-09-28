//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 54: RTE from format $2)   //
//                                                                          //
// tb_ap040_pipe_rte_fmt2.v - returning from an address-error handler       //
//                                                                          //
// RTE read the format word off the stack from the very beginning and       //
// ignored it, which was harmless while nothing could push anything but a   //
// format $0 frame. Milestone 17 made address error push format $2, and     //
// from then on this was a real defect rather than a hypothetical one: an   //
// RTE returning from an address-error handler popped eight bytes off a     //
// twelve-byte frame and left A7 four bytes low. Not a crash -- the return  //
// itself works, because the SR, PC and format word sit at the same         //
// offsets in both frames -- but the stack walks downward once per return.  //
//                                                                          //
// That is exactly why the check here is A7 and not the resumption. The     //
// program resumes correctly either way, so a bench that only confirmed     //
// "we got back" would have passed against the bug for as long as it has    //
// existed.                                                                 //
//                                                                          //
//   A7 = $0600, A0 = $0407 (odd)                                           //
//   JMP (A0)          -> address error, format $2, A7 -> $05F4             //
//   handler @ $0800:  MOVEQ #$11,D1                                        //
//                     RTE           -> must pop TWELVE bytes               //
//   MOVEQ #$2A,D0     <- resumption point                                  //
//                                                                          //
// D1 says the handler ran, D0 says the RTE returned to the stacked PC,     //
// and A7 = $0600 says it popped the whole frame. The push took A7 to       //
// $05F4; popping eight instead of twelve leaves $05FC, four low, which is  //
// exactly the address-field longword format $2 adds and is a value         //
// nothing else in this program could produce.                              //
//                                                                          //
// FMTERR is still not implemented and this bench does not pretend          //
// otherwise: a format nibble that is neither $0 nor $2 is treated as $0.   //
// Raising vector 14 means starting an exception from a branch already      //
// mid-pop, and nothing in this core pushes any other format.               //
//                                                                          //
// Vector 3 sits at VBR $2000 + 12, word index 3590.                        //
//                                                                          //
// On milestone 53's RTL the return works and A7 comes back $05FC.          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rte_fmt2;

localparam PROG_WORDS      = 32;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
// A pseudo-random clock enable (milestone 94). Every bench in this suite
// tied ce high, and eight of the thirteen defects three rounds of external
// review found lived behind that: a cycle with ce low is a cycle that did
// not happen, and the core has to treat it that way. Driven on the falling
// edge so it is stable across every rising one, and left high until reset
// releases so the reset sequence itself is unchanged.
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
wire [31:0] dbg_d0, dbg_d1, dbg_d2;
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

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	// Mainline
	dut.u_l1.mem[1]  = 16'h4ED0;   // JMP (A0)  -- A0 is odd: address error
	dut.u_l1.mem[2]  = 16'h702A;   // MOVEQ #$2A,D0  <- RTE resumes here
	dut.u_l1.mem[3]  = 16'h4E71;   // NOP

	// Address-error handler @ word idx 512 (byte $800)
	dut.u_l1.mem[512] = 16'h7211;  // MOVEQ #$11,D1
	dut.u_l1.mem[513] = 16'h4E73;  // RTE

	// Vector table: vector 3 -> $800 (word idx 3590/3591)
	dut.u_l1.mem[3590] = 16'h0000;
	dut.u_l1.mem[3591] = 16'h0800;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// Three exception frames are pushed and never popped (the handler
	// returns with JMP, not RTE), so A7 must point somewhere real and
	// clear of the program. See tb_ap040_pipe_move_mem.v's header for why
	// the poke has to land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.areg[0] = 32'h0000_0407;  // A0: odd JMP target
	dut.u_cpu.u_regfile.isp     = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d1 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000011 (the address-error handler must run)", dbg_d1);
	end
	if (dbg_d0 !== 32'h0000_002A) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0000002a (RTE must resume at the stacked PC)", dbg_d0);
	end
	// The point of the milestone. A format $2 frame is twelve bytes; popping
	// eight returns correctly and leaves A7 four low, once per return.
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (000005fc means RTE popped an 8-byte frame off a 12-byte one)",
		         dut.u_cpu.u_regfile.isp);
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
