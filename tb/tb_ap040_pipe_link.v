//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 49: LINK/UNLK)      //
//                                                                          //
// tb_ap040_pipe_link.v - a real function prologue and epilogue             //
//                                                                          //
// Every compiled function opens with LINK and closes with UNLK:            //
//                                                                          //
//   LINK An,#d16   mem[A7-4] <- An ;  An <- A7-4 ;  A7 <- A7-4+d16         //
//   UNLK An        An <- mem[An]   ;  A7 <- An+4                           //
//                                                                          //
// Both write TWO registers, which is what makes them fit here: milestone   //
// 30's second write port already exists for autoincrement and is free,     //
// because neither instruction autoincrements. LINK's memory write reuses   //
// the BSR/JSR push path, whose address is already operand_b - 4, so        //
// pointing eac_dest_reg at A7 makes it come out right with no new          //
// arithmetic; only the pushed DATA is new, the old An instead of a return  //
// address. UNLK needs no new memory path at all -- a plain load with An as //
// both the address and the destination.                                    //
//                                                                          //
//   A7 = $0600, A6 = $11112222                                             //
//   LINK   A6,#-8        mem[$05FC] = 11112222, A6 = $05FC, A7 = $05F4     //
//   MOVE.L #$AABBCCDD,D0                                                   //
//   MOVE.L D0,(A7)       mem[$05F4] = AABBCCDD                             //
//   UNLK   A6            A6 = 11112222, A7 = $0600                        //
//                                                                          //
// The checks CHAIN, because LINK's two register results are both undone by //
// the UNLK that follows and cannot be read directly at the end:            //
//                                                                          //
//   A7 = $05F4 is proven by the store landing at $05F4 and nowhere else.   //
//     A zero-extended displacement would put A7 at $000105F4 and leave     //
//     that longword untouched.                                             //
//   A6 = $05FC is proven by the UNLK reading 11112222 back. Had A6 been    //
//     wrong, UNLK would have popped from somewhere else -- NOP fill, so    //
//     4E714E71 -- and restored that instead.                               //
//   A7 = $0600 at the end proves the frame was balanced, and it is NOT the //
//     value A7 would keep if both instructions had done nothing, because   //
//     the store at $05F4 could not then have happened.                     //
//                                                                          //
// UNLK is last so the N flag set by the preceding MOVE can be checked      //
// surviving it: neither LINK nor UNLK touches condition codes, and both    //
// move data through paths that otherwise would.                            //
//                                                                          //
// On milestone 48's RTL neither instruction decodes.                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_link;

localparam PROG_WORDS      = 16;
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
	dut.u_l1.mem[1]  = 16'h2C7C;   // MOVEA.L #$11112222,A6
	dut.u_l1.mem[2]  = 16'h1111;
	dut.u_l1.mem[3]  = 16'h2222;
	dut.u_l1.mem[4]  = 16'h4E56;   // LINK A6,#-8
	dut.u_l1.mem[5]  = 16'hFFF8;
	dut.u_l1.mem[6]  = 16'h203C;   // MOVE.L #$AABBCCDD,D0
	dut.u_l1.mem[7]  = 16'hAABB;
	dut.u_l1.mem[8]  = 16'hCCDD;
	dut.u_l1.mem[9]  = 16'h2E80;   // MOVE.L D0,(A7)
	dut.u_l1.mem[10] = 16'h4E5E;   // UNLK A6
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// A7 is the address bank's register 7, which in supervisor mode is the
	// ISP. See tb_ap040_pipe_move_mem.v's header for why the poke has to
	// land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $05FC is word index 254, $05F4 is word index 250.
	if ({dut.u_l1.mem[254], dut.u_l1.mem[255]} !== 32'h1111_2222) begin
		errors = errors + 1;
		$display("FAIL: mem[$05FC] = %h%h, expected 11112222 (LINK must push the OLD A6)",
		         dut.u_l1.mem[254], dut.u_l1.mem[255]);
	end
	if ({dut.u_l1.mem[250], dut.u_l1.mem[251]} !== 32'hAABB_CCDD) begin
		errors = errors + 1;
		$display("FAIL: mem[$05F4] = %h%h, expected aabbccdd (A7 must be $05F4; a zero-extended d16 puts it at $000105f4)",
		         dut.u_l1.mem[250], dut.u_l1.mem[251]);
	end
	if (dut.u_cpu.u_regfile.areg[6] !== 32'h1111_2222) begin
		errors = errors + 1;
		$display("FAIL: A6 = %h, expected 11112222 (UNLK pops through A6; 4e714e71 means A6 pointed at NOP fill)",
		         dut.u_cpu.u_regfile.areg[6]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (UNLK must restore the stack it was handed)",
		         dut.u_cpu.u_regfile.isp);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. MOVE.L of aabbccdd set N; the UNLK after it
	// must leave the flags alone.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (neither LINK nor UNLK touches condition codes)",
		         dbg_ccr[3:0]);
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
