//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 68: MOVEM control modes)  //
//                                                                          //
// tb_ap040_pipe_movemctl.v - MOVEM to (An) and from (d16,An)              //
//                                                                          //
// The control modes differ from the autoincrement ones in THREE ways at    //
// once, and all three belong to the MODE rather than to the direction:     //
//                                                                          //
//   - no register writeback at all                                         //
//   - the address walks UPWARD, even for a store                           //
//   - the mask is numbered bit 0 = D0, even for a store                    //
//                                                                          //
// Only the predecrement store reverses the numbering and walks downward.   //
// Milestone 50 tied both to "is a store", which was indistinguishable from //
// the truth while -(An) was the only store mode that existed.              //
//                                                                          //
//   MOVEA.L #$0500,A0 ; D0/D1/D2 = 11111111 / 22222222 / 33333333          //
//   MOVEM.L D0-D2,(A0)      mask 0007, upward from $0500, A0 UNCHANGED     //
//   clobber D0 and D1                                                      //
//   MOVEM.L (4,A0),D0-D1    mask 0003, reads $0504 and $0508              //
//                                                                          //
// The store is checked in memory rather than by a round trip, because a    //
// round trip through one mode cannot distinguish the numbering: any        //
// self-consistent ordering restores what it saved. $0500 holding 11111111  //
// says bit 0 meant D0 and the walk went up; reversed numbering would have  //
// stored A7 there, and a downward walk would have put D0 below $0500.      //
//                                                                          //
// A0 = $00000500 at the end is the writeback check, and it is the reason   //
// the load uses a DISPLACEMENT: reading back through (4,A0) only finds     //
// 22222222 and 33333333 if A0 was left alone by the store.                 //
//                                                                          //
// (d16,An) also gathers a SECOND word after the mask, so id_imm carries    //
// {mask, displacement} and the mask moved to the high half for every mode. //
//                                                                          //
// On milestone 67's RTL neither form decodes.                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movemctl;

localparam PROG_WORDS      = 40;
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
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00000500,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0500;
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$11111111,D0
	dut.u_l1.mem[5]  = 16'h1111;
	dut.u_l1.mem[6]  = 16'h1111;
	dut.u_l1.mem[7]  = 16'h223C;   // MOVE.L #$22222222,D1
	dut.u_l1.mem[8]  = 16'h2222;
	dut.u_l1.mem[9]  = 16'h2222;
	dut.u_l1.mem[10] = 16'h243C;   // MOVE.L #$33333333,D2
	dut.u_l1.mem[11] = 16'h3333;
	dut.u_l1.mem[12] = 16'h3333;
	dut.u_l1.mem[13] = 16'h48D0;   // MOVEM.L D0-D2,(A0)
	dut.u_l1.mem[14] = 16'h0007;
	dut.u_l1.mem[15] = 16'h203C;   // MOVE.L #$AAAAAAAA,D0
	dut.u_l1.mem[16] = 16'hAAAA;
	dut.u_l1.mem[17] = 16'hAAAA;
	dut.u_l1.mem[18] = 16'h223C;   // MOVE.L #$BBBBBBBB,D1
	dut.u_l1.mem[19] = 16'hBBBB;
	dut.u_l1.mem[20] = 16'hBBBB;
	dut.u_l1.mem[21] = 16'h4CE8;   // MOVEM.L (4,A0),D0-D1
	dut.u_l1.mem[22] = 16'h0003;   //   mask
	dut.u_l1.mem[23] = 16'h0004;   //   displacement
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

	repeat ((PROG_WORDS + 220) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $0500 is word index 128, $0504 is 130, $0508 is 132.
	if ({dut.u_l1.mem[128], dut.u_l1.mem[129]} !== 32'h1111_1111) begin
		errors = errors + 1;
		$display("FAIL: [$0500] = %h%h, expected 11111111 (bit 0 means D0 here, and the walk goes UP)",
		         dut.u_l1.mem[128], dut.u_l1.mem[129]);
	end
	if ({dut.u_l1.mem[130], dut.u_l1.mem[131]} !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: [$0504] = %h%h, expected 22222222", dut.u_l1.mem[130], dut.u_l1.mem[131]);
	end
	if ({dut.u_l1.mem[132], dut.u_l1.mem[133]} !== 32'h3333_3333) begin
		errors = errors + 1;
		$display("FAIL: [$0508] = %h%h, expected 33333333", dut.u_l1.mem[132], dut.u_l1.mem[133]);
	end
	if (dbg_d0 !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 22222222 (MOVEM.L (4,A0) -- only right if A0 was left alone)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h3333_3333) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 33333333", dbg_d1);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0500) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000500 (a control mode writes the address register BACK not at all; 0000050c means it did)",
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
