//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 19: the register-   //
// to-register ALU family)                                                  //
//                                                                          //
// tb_ap040_pipe_aluops.v - OR, SUB, CMP and AND reach the ALU              //
//                                                                          //
// ap040_pipe_alu.v has implemented thirty-three operations since it was    //
// forked from rtl_old/ap040_alu.v. Before this milestone the decoder could //
// emit exactly two of them, ADD and MOVE, so the other thirty-one were     //
// unreachable dead logic -- which is also why a standalone synthesis of    //
// this core measured the ALU at 36 ALUTs. They all share one encoding      //
// shape, 1ooo RRR 0 SS 000 rrr, so reaching them needed a widened          //
// predicate and an op map, no new datapath.                                //
//                                                                          //
// Program:                                                                //
//                                                                          //
//   1: MOVEQ #$0F,D0   700F   D0 = 0000000F                                //
//   2: MOVEQ #$33,D1   7233   D1 = 00000033                                //
//   3: AND.L  D0,D1    C280   D1 = 0F & 33 = 00000003                      //
//   4: OR.L   D0,D1    8280   D1 = 03 | 0F = 0000000F                      //
//   5: MOVEQ #$20,D2   7420   D2 = 00000020                                //
//   6: SUB.L  D0,D2    9480   D2 = 20 - 0F = 00000011                      //
//   7: CMP.L  D2,D2    B482   flags only: D2 - D2 = 0, Z=1, D2 UNCHANGED   //
//                                                                          //
// SUB proves operand ORDER, not just that a subtractor exists: the ALU     //
// computes b - a and ap040_ea_fetch.v resolves operand_b from the          //
// destination, so a decoder that swapped them would give 0F - 20 = EF here //
// instead of 11. CMP proves the one member of the family that writes no    //
// register -- if id_writes_reg let it through, D2 would be clobbered to 0  //
// while the flags still looked right.                                      //
//                                                                          //
// On the pre-milestone RTL none of the four decodes: they fall through to  //
// illegal and the register checks fail.                                    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_aluops;

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
	dut.u_l1.mem[1] = 16'h700F;   // MOVEQ #$0F,D0
	dut.u_l1.mem[2] = 16'h7233;   // MOVEQ #$33,D1
	dut.u_l1.mem[3] = 16'hC280;   // AND.L  D0,D1
	dut.u_l1.mem[4] = 16'h8280;   // OR.L   D0,D1
	dut.u_l1.mem[5] = 16'h7420;   // MOVEQ #$20,D2
	dut.u_l1.mem[6] = 16'h9480;   // SUB.L  D0,D2
	dut.u_l1.mem[7] = 16'hB482;   // CMP.L  D2,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_000F) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0000000F", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_000F) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000000F (AND then OR)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000011 (SUB is b-a = 20-0F; CMP must not write)", dbg_d2);
	end

	// CMP.L D2,D2 is last: D2-D2 = 0, so Z=1 and N=V=C=0. dbg_ccr[3:0] is
	// {N,Z,V,C}.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMP of equal operands sets Z)", dbg_ccr[3:0]);
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
