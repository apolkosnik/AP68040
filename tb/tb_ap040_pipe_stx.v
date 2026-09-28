//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 32: stores to       //
// (An)+ and -(An))                                                          //
//                                                                          //
// tb_ap040_pipe_stx.v - where two mechanisms meet                          //
//                                                                          //
// This adds no new mechanism. It is the point where milestone 30's address //
// update meets milestone 31's store path, and the reason it needs its own  //
// testbench is that the two take their address register from OPPOSITE      //
// operands: a load's An is the source at ir[2:0], a store's is the         //
// destination at ir[11:9]. One an_base wire in ap040_ea_fetch.v resolves    //
// that, and everything downstream -- the increment, the access address and  //
// the register the update commits to -- follows from it.                    //
//                                                                          //
// Program (A0 seeded to $0480, A1 to $0480; no MOVEA yet):                  //
//                                                                          //
//   1: MOVE.L #$11112222,D0   203C 1111 2222                                //
//   4: MOVE.L D0,(A0)+        20C0     -> $0480, A0 = $0484                 //
//   5: MOVE.L #$33334444,D1   223C 3333 4444                                //
//   8: MOVE.L D1,(A0)+        20C1     -> $0484, A0 = $0488                 //
//   9: MOVE.L (A1)+,D2        2419     read $0480 back, A1 = $0484          //
//                                                                          //
// The second store is the load-bearing one: it can only land at $0484 if    //
// the first store's A0 update was forwarded to it, which is the second      //
// forward path on a STORE rather than a load -- a combination neither       //
// milestone 30 nor 31 exercised on its own.                                 //
//                                                                          //
// Memory is checked directly at both addresses, and the read-back through   //
// (A1)+ confirms a load and a store agree about where $0480 is. If the      //
// store used operand_a (the data) as its address the way every load does,   //
// both writes would land somewhere unrelated and both memory checks fail.   //
//                                                                          //
// A0 is checked too: two postincrements from $0480 must leave $0488. That   //
// distinguishes "the update went to the right register" from "the update    //
// happened" -- with eaf_an_reg still taking eac_src_reg, a store would      //
// update the DATA register instead.                                         //
//                                                                          //
// On milestone 31's RTL neither store form decodes.                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stx;

localparam PROG_WORDS      = 24;
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
	dut.u_l1.mem[1] = 16'h203C;   // MOVE.L #$11112222,D0
	dut.u_l1.mem[2] = 16'h1111;
	dut.u_l1.mem[3] = 16'h2222;
	dut.u_l1.mem[4] = 16'h20C0;   // MOVE.L D0,(A0)+
	dut.u_l1.mem[5] = 16'h223C;   // MOVE.L #$33334444,D1
	dut.u_l1.mem[6] = 16'h3333;
	dut.u_l1.mem[7] = 16'h4444;
	dut.u_l1.mem[8] = 16'h20C1;   // MOVE.L D1,(A0)+
	dut.u_l1.mem[9] = 16'h2419;   // MOVE.L (A1)+,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	@(posedge clk);
	dut.u_cpu.u_regfile.areg[0] = 32'h0000_0480;
	dut.u_cpu.u_regfile.areg[1] = 32'h0000_0480;

	repeat ((PROG_WORDS + 30) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// Byte $0480 is word index 64; $0484 is 66.
	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h1111_2222) begin
		errors = errors + 1;
		$display("FAIL: memory at $0480 = %h%h, expected 11112222",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	// Only reachable if the first store's A0 update was forwarded.
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h3333_4444) begin
		errors = errors + 1;
		$display("FAIL: memory at $0484 = %h%h, expected 33334444 (the A0 update did not forward to the second store)",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if (dbg_d2 !== 32'h1111_2222) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 11112222 (a load and a store disagree about $0480)", dbg_d2);
	end
	// Two postincrements: the update must go to A0, not to the data register.
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0488) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000488 (the store's update went to the wrong register)",
		         dut.u_cpu.u_regfile.areg[0]);
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
