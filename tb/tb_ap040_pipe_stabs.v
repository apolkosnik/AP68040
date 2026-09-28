//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 34: absolute stores)//
//                                                                          //
// tb_ap040_pipe_stabs.v - a store whose address comes from the gather      //
//                                                                          //
// Every store so far took its address from a register. This one takes it    //
// from extension words, which makes it the first store to need HELD state:  //
// "this is a store" and the DATA register have to survive the gather, the   //
// same shape held_is_imm has used since milestone 26.                       //
//                                                                          //
// The two roles nearly collide. A gathered value has meant an operand       //
// (milestone 26), a branch target, or a load address (milestone 29); here   //
// it is a store address while the DATA comes from a register, so src_reg    //
// must point at ir[2:0] and l1_addr_word must prefer eac_imm over the       //
// address register an ordinary store would use.                             //
//                                                                          //
// Program:                                                                  //
//                                                                           //
//   1: MOVE.L #$C0FFEE00,D0    203C C0FF EE00                               //
//   4: MOVE.L D0,($0480).W     21C0 0480       store, address from a gather //
//   6: MOVE.L #$5EED1234,D1    223C 5EED 1234                               //
//   9: MOVE.L D1,($000004A0).L 23C1 0000 04A0  the two-word form            //
//  12: MOVE.L ($0480).L,D2     2439 0000 0480  read the first back          //
//                                                                           //
// The two forms gather different widths from one predicate, so both are      //
// here. Both addresses are checked directly in memory AND one is read back   //
// through the core, so a store that landed at the wrong address shows as     //
// the other value rather than as zero.                                       //
//                                                                           //
// The values are chosen to be distinguishable from an address: if the store  //
// wrote its own address instead of the data -- the mistake available when    //
// operand roles invert -- memory would hold 00000480, which neither          //
// expected value resembles.                                                  //
//                                                                           //
// On milestone 33's RTL neither store form decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stabs;

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
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$C0FFEE00,D0
	dut.u_l1.mem[2]  = 16'hC0FF;
	dut.u_l1.mem[3]  = 16'hEE00;
	dut.u_l1.mem[4]  = 16'h21C0;   // MOVE.L D0,($0480).W
	dut.u_l1.mem[5]  = 16'h0480;
	dut.u_l1.mem[6]  = 16'h223C;   // MOVE.L #$5EED1234,D1
	dut.u_l1.mem[7]  = 16'h5EED;
	dut.u_l1.mem[8]  = 16'h1234;
	dut.u_l1.mem[9]  = 16'h23C1;   // MOVE.L D1,($000004A0).L
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h04A0;
	dut.u_l1.mem[12] = 16'h2439;   // MOVE.L ($00000480).L,D2
	dut.u_l1.mem[13] = 16'h0000;
	dut.u_l1.mem[14] = 16'h0480;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 40) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $0480 is word index 64, $04A0 is 80.
	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'hC0FF_EE00) begin
		errors = errors + 1;
		$display("FAIL: memory at $0480 = %h%h, expected C0FFEE00 (the one-word address form)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[80], dut.u_l1.mem[81]} !== 32'h5EED_1234) begin
		errors = errors + 1;
		$display("FAIL: memory at $04A0 = %h%h, expected 5EED1234 (the two-word address form)",
		         dut.u_l1.mem[80], dut.u_l1.mem[81]);
	end
	if (dbg_d2 !== 32'hC0FF_EE00) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected C0FFEE00 (a load and a store disagree about $0480)", dbg_d2);
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
