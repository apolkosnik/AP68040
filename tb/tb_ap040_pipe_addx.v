//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 22: ADDX and SUBX)  //
//                                                                          //
// tb_ap040_pipe_addx.v - the X flag actually reaches the ALU               //
//                                                                          //
// ADDX/SUBX occupy 1ooo Rx 1 SS 00 0 Ry -- the ir[8]=1 slot the binary     //
// family excludes as "the Dn -> <ea> direction", which with a register-    //
// direct source is not encodable, so the slot is theirs. They are the      //
// first instructions here to consume a condition-code INPUT rather than    //
// only producing flags, which is what this testbench is built to prove.    //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #-1,D0    70FF   D0 = FFFFFFFF                                //
//   2: MOVEQ #1,D1     7201   D1 = 00000001                                //
//   3: ADD.L  D0,D1    D280   D1 = 00000000, carry out sets X              //
//   4: MOVEQ #5,D2     7405   D2 = 00000005                                //
//   5: CLR.L  D0       4280   D0 = 0; CLR preserves X (flags {f_x,0,1,0,0}) //
//   6: ADDX.L D0,D2    D580   D2 = 5 + 0 + X                               //
//                                                                          //
// D2 is 6 only if the X set two instructions earlier survived and was      //
// consumed. Both failure modes give 5: an ADDX that never decodes leaves   //
// D2 at its MOVEQ value, and an ADDX that decoded but ignored the carry-in //
// computes 5 + 0. So the single check separates "reached the ALU" from     //
// "reached it with the flag".                                              //
//                                                                          //
// CLR is not incidental. It zeroes D0 so the sum isolates the X            //
// contribution, and it is chosen because it writes Z and leaves X alone --  //
// an instruction that clobbered X would make the test pass or fail for the //
// wrong reason.                                                            //
//                                                                          //
// On milestone 21's RTL ADDX does not decode and D2 stays 00000005.        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_addx;

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
	dut.u_l1.mem[1] = 16'h70FF;   // MOVEQ #-1,D0
	dut.u_l1.mem[2] = 16'h7201;   // MOVEQ #1,D1
	dut.u_l1.mem[3] = 16'hD280;   // ADD.L  D0,D1   -> X = 1
	dut.u_l1.mem[4] = 16'h7405;   // MOVEQ #5,D2
	dut.u_l1.mem[5] = 16'h4280;   // CLR.L  D0      -- keeps X
	dut.u_l1.mem[6] = 16'hD580;   // ADDX.L D0,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000000 (CLR.L)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (FFFFFFFF + 1 wraps)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0006) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000006 (5 + 0 + X; 5 means X never reached the ALU)", dbg_d2);
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
