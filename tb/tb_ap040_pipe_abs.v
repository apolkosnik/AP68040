//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 29: absolute        //
// addressing as a memory source)                                           //
//                                                                          //
// tb_ap040_pipe_abs.v - an address with no register term                   //
//                                                                          //
// MOVE.L (xxx).W,Dn and MOVE.L (xxx).L,Dn reuse both the gather of         //
// milestone 26 (one word sign-extended for .W, two for .L) and the         //
// memory-read path milestone 9b built for MOVE.L (An),Dn. The one new      //
// thing is that the address has no register term: ap040_ea_fetch.v         //
// computed ea_target as operand_a + eac_imm for every access so far, which //
// is right for (An) and (d16,An) and wrong here. eac_is_abs drops the      //
// register term rather than asking decode to find a register reading zero, //
// because no register does.                                                //
//                                                                          //
// Program and data:                                                        //
//                                                                          //
//   mem[64],[65] = CAFE BABE   the longword at byte address $0480          //
//   mem[66],[67] = 0BAD F00D   the longword at byte address $0484          //
//                                                                          //
//   1: MOVE.L ($0480).W,D0   2038 0480        D0 = CAFEBABE                //
//   3: MOVE.L ($00000484).L,D1  2039 0000 0484  D1 = 0BADF00D              //
//                                                                          //
// The two forms are both here because they gather different widths from    //
// the same predicate, and because .W sign-extends its single word while    //
// .L does not -- an address above $7FFF would expose that, but these two   //
// only need to prove each width reaches the right place.                   //
//                                                                          //
// The addresses are four bytes apart and the data deliberately unalike, so //
// a stale or duplicated gather shows up as the wrong longword rather than  //
// as zero. Reading the register term back in -- the bug eac_is_abs exists  //
// to prevent -- would add D0's or D1's own contents to the address and     //
// land somewhere else entirely.                                            //
//                                                                          //
// On milestone 28's RTL neither decodes.                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_abs;

localparam PROG_WORDS      = 20;
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
	dut.u_l1.mem[1] = 16'h2038;   // MOVE.L ($0480).W,D0
	dut.u_l1.mem[2] = 16'h0480;
	dut.u_l1.mem[3] = 16'h2239;   // MOVE.L ($00000484).L,D1
	dut.u_l1.mem[4] = 16'h0000;
	dut.u_l1.mem[5] = 16'h0484;

	// Data: byte address $0480 is word index 64 relative to PC_RESET.
	dut.u_l1.mem[64] = 16'hCAFE;
	dut.u_l1.mem[65] = 16'hBABE;
	dut.u_l1.mem[66] = 16'h0BAD;
	dut.u_l1.mem[67] = 16'hF00D;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 30) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hCAFE_BABE) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected CAFEBABE (MOVE.L (xxx).W: one word, no register term)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0BAD_F00D) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0BADF00D (MOVE.L (xxx).L: two words, no register term)", dbg_d1);
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
