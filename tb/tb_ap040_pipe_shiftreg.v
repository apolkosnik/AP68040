//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 87)                 //
//                                                                          //
// tb_ap040_pipe_shiftreg.v - shifts and rotates counted by a register      //
//                                                                          //
// 1110 ccc d ss i tt rrr: bit 5 says where the count comes from, and until //
// this milestone only the immediate form (bit 5 clear) decoded -- the      //
// register form was an illegal instruction. The differential against the   //
// FSM core found it in milestone 83, by generating one.                    //
//                                                                          //
// The count is a third register read, and it goes through port A, which    //
// decode points at the count register, so it forwards from EX and WB like  //
// any other source operand. Two things the immediate form never had to     //
// answer come with it: a count of more than 32, and a count of ZERO --     //
// which the immediate encoding cannot express at all, 0 there meaning 8.   //
//                                                                          //
//   MOVE.L #$12345678,D0 / MOVEQ #4,D1 / ASL.L D1,D0    -> $23456780       //
//   MOVE.L #$80000001,D4 / MOVEQ #33,D3 / ROL.L D3,D4   -> $00000003,      //
//                                          the count taken modulo 32       //
//   MOVE.L #0,D5 / MOVE.L #-1,D6 / ADD.L #1,D6          -> D6 = 0, X = 1   //
//   ROXL.L D5,D6                 count 0: D6 unchanged, X unchanged, and   //
//                                C takes X rather than a carry out of a    //
//                                shift that never happened                 //
//   SCS D7                       -> $FF, which is the C above              //
//                                                                          //
// The zero-count case is the one worth having a bench for: the closed-form  //
// barrel in ap040_pipe_alu.v computes a carry for it and would write that  //
// to X, so D7 here is really a check on X as much as on C.                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_shiftreg;

localparam PROG_WORDS      = 64;
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
	.clk (clk), .nreset (nreset), .ce  (ce),

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

task check32;
	input string  what;
	input [31:0]  got, want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", what, got, want);
		end
	end
endtask

task poke;
	input [31:0] addr;
	input [15:0] word;
	begin dut.u_l1.mem[(addr - PC_RESET) >> 1] = word; end
endtask

initial begin
	#1;
	poke(32'h0400, 16'h203C); poke(32'h0402, 16'h1234); poke(32'h0404, 16'h5678);
	poke(32'h0406, 16'h7204);                      // MOVEQ #4,D1
	poke(32'h0408, 16'hE3A0);                      // ASL.L D1,D0
	poke(32'h040A, 16'h283C); poke(32'h040C, 16'h8000); poke(32'h040E, 16'h0001);
	poke(32'h0410, 16'h7621);                      // MOVEQ #33,D3
	poke(32'h0412, 16'hE7BC);                      // ROL.L D3,D4
	poke(32'h0414, 16'h2A3C); poke(32'h0416, 16'h0000); poke(32'h0418, 16'h0000);
	poke(32'h041A, 16'h2C3C); poke(32'h041C, 16'hFFFF); poke(32'h041E, 16'hFFFF);
	poke(32'h0420, 16'h0686); poke(32'h0422, 16'h0000); poke(32'h0424, 16'h0001);
	poke(32'h0426, 16'hEBB6);                      // ROXL.L D5,D6   (count 0)
	poke(32'h0428, 16'h55C7);                      // SCS D7
	poke(32'h042A, 16'h4E71);                      // NOP
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D0 (ASL.L by a register count of 4)",   dbg_d0,                32'h2345_6780);
	check32("D4 (ROL.L by 33, taken modulo 32)",     dut.u_cpu.u_regfile.dreg[4], 32'h0000_0003);
	check32("D6 (ROXL.L by a count of ZERO)",        dut.u_cpu.u_regfile.dreg[6], 32'h0000_0000);
	check32("D7 (SCS: C took X, which a zero count leaves alone)",
	                                                 dut.u_cpu.u_regfile.dreg[7], 32'h0000_00FF);

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
