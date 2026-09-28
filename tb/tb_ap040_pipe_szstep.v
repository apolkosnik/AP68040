//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 36: sized (An)+ and //
// -(An))                                                                    //
//                                                                          //
// tb_ap040_pipe_szstep.v - the step follows the size                       //
//                                                                          //
// Milestone 35 made loads sized but left the auto-increment modes Long,     //
// because their step has to follow the operand size: one, two or four       //
// rather than always four. This adds that, and with it the 68000's stack    //
// exception -- a BYTE access through A7 steps by TWO, not one, so the stack //
// pointer stays even.                                                       //
//                                                                          //
// Memory at $0480: 11 22 33 44                                              //
//                                                                          //
//   MOVEA.L #$0480,A0                                                       //
//   MOVE.B  (A0)+,D0     D0 = ..11, A0 = $0481   step ONE                   //
//   MOVE.B  (A0)+,D1     D1 = ..22, A0 = $0482   consecutive bytes          //
//   MOVE.W  (A0)+,D2     D2 = ..3344, A0 = $0484 step TWO                   //
//                                                                          //
// The two byte loads are the whole point. If the step were still four, the  //
// second would read $0484 and give a different value; if it were two, it    //
// would read $0482 and give 33. Only a step of one gives 22, so D1 alone    //
// separates all three cases. D2 then confirms a word steps by two, because  //
// it must land on $0482 and read 3344.                                      //
//                                                                          //
// A0's final value is checked as well: 1 + 1 + 2 from $0480 is $0484. That  //
// is a different quantity from the data checks -- it catches a step used for //
// the ACCESS but not for the update, which the loads alone would not see.   //
//                                                                          //
// The A7 byte exception is implemented (an_is_a7 in ap040_ea_fetch.v) but   //
// not exercised here: A7 banks to ISP/USP/MSP by processor state, so a test //
// for it belongs with the supervisor benches rather than this one.          //
//                                                                          //
// On milestone 35's RTL the byte and word forms of these modes do not       //
// decode.                                                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_szstep;

localparam PROG_WORDS      = 28;
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
	dut.u_l1.mem[1] = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[2] = 16'h0000;
	dut.u_l1.mem[3] = 16'h0480;
	dut.u_l1.mem[4] = 16'h1018;   // MOVE.B (A0)+,D0
	dut.u_l1.mem[5] = 16'h1218;   // MOVE.B (A0)+,D1
	dut.u_l1.mem[6] = 16'h3418;   // MOVE.W (A0)+,D2

	dut.u_l1.mem[64] = 16'h1122;
	dut.u_l1.mem[65] = 16'h3344;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 40) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0[7:0] !== 8'h11) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected low byte 11", dbg_d0);
	end
	// The load-bearing check: a step of 4 reads $0484, a step of 2 reads
	// $0482 and gives 33. Only a step of ONE gives 22.
	if (dbg_d1[7:0] !== 8'h22) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected low byte 22 (a byte step must be ONE; 33 means two, and 4 lands elsewhere)", dbg_d1);
	end
	if (dbg_d2[15:0] !== 16'h3344) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected low word 3344 (a word step must be TWO)", dbg_d2);
	end
	// Separate quantity: catches a step used for the access but not the update.
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0484) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000484 (1 + 1 + 2 from 0480)", dut.u_cpu.u_regfile.areg[0]);
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
