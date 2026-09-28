//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 21: SWAP, EXT and   //
// EXTB)                                                                    //
//                                                                          //
// tb_ap040_pipe_extswap.v - the group whose ir[7:6] is NOT a size          //
//                                                                          //
// SWAP, EXT.W, EXT.L and EXTB.L share one opcode word, 0100 100x oo 000    //
// rrr, and all read operand b like the unary group. The trap is that their //
// ir[7:6] is an OPCODE selector rather than std_size, so reusing           //
// add_op_size would read EXT.W as Long and give EXT.L and EXTB.L the       //
// invalid size 3. This milestone gives them their own mapping: EXT.W is    //
// the only member sized Word, because it alone produces a result that is   //
// spliced into Dn[15:0]; SWAP and EXTB.L build full longwords that must    //
// pass through execute's merge unmasked.                                   //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #-2,D0     70FE   D0 = FFFFFFFE                               //
//   2: SWAP   D0        4840   D0 = FFFEFFFF   halves exchanged, all 32    //
//   3: MOVEQ #$7F,D1    727F   D1 = 0000007F                               //
//   4: EXT.W  D1        4881   D1 = 0000007F   byte 7F is positive         //
//   5: MOVEQ #-1,D2     74FF   D2 = FFFFFFFF                               //
//   6: EXT.W  D2        4882   D2 = FFFFFFFF   byte FF -> word FFFF        //
//   7: EXTB.L D2        49C2   D2 = FFFFFFFF   byte FF -> long, whole reg  //
//                                                                          //
// SWAP is the check that catches a wrong size: it returns                  //
// {b[15:0],b[31:16]}, so if it were sized Word the merge would keep        //
// D0[31:16] = FFFF and write only the low half, giving FFFFFFFE back       //
// unchanged instead of FFFEFFFF.                                           //
//                                                                          //
// The two EXT.W cases separate "sized correctly" from "happens to look     //
// right". D1 starts 0000007F and ends unchanged, which a broken size would //
// also produce; D2 starts FFFFFFFF and its byte FF must sign-extend to     //
// word FFFF while D2[31:16] stays FFFF, so the value only survives if the  //
// word splice is real. EXTB.L then rewrites the whole register from the    //
// same byte, which a Word-sized EXTB would leave masked.                   //
//                                                                          //
// On milestone 20's RTL none of the four decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_extswap;

localparam PROG_WORDS      = 14;
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
	dut.u_l1.mem[1] = 16'h70FE;   // MOVEQ #-2,D0
	dut.u_l1.mem[2] = 16'h4840;   // SWAP   D0
	dut.u_l1.mem[3] = 16'h727F;   // MOVEQ #$7F,D1
	dut.u_l1.mem[4] = 16'h4881;   // EXT.W  D1
	dut.u_l1.mem[5] = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[6] = 16'h4882;   // EXT.W  D2
	dut.u_l1.mem[7] = 16'h49C2;   // EXTB.L D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hFFFE_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected FFFEFFFF (SWAP must be Long or the merge masks it)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_007F) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000007F (EXT.W of a positive byte)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFFFFFF (EXT.W word splice, then EXTB.L whole register)", dbg_d2);
	end

	// EXTB.L is last: result FFFFFFFF, so N=1, Z=0, V=0, C=0.
	// dbg_ccr[3:0] is {N,Z,V,C}.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (EXTB.L of FF is negative)", dbg_ccr[3:0]);
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
