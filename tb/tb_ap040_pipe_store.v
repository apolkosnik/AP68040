//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 31: MOVE.L Dn,(An)) //
//                                                                          //
// tb_ap040_pipe_store.v - the first store                                  //
//                                                                          //
// Until now the only thing that reached memory was a push: BSR/JSR's return //
// address and the exception frames, all writing a value the pipeline        //
// computed for itself. This is the first instruction that writes a          //
// PROGRAM-supplied value to a PROGRAM-supplied address, and it reuses       //
// milestone 12's write buffer unchanged -- l1_data_b is 32 bits wide and    //
// the L1 splits a longword into its two 16-bit beats.                       //
//                                                                          //
// The operand roles are the reverse of every load so far. The data is Dn at //
// ir[2:0] and the address is An at ir[11:9], so decode makes dest_reg an    //
// ADDRESS register index and the store takes its address from operand_b.    //
// ea_target is operand_a + eac_imm and operand_a is the DATA here, which is //
// why the store gets its own arm in l1_addr_word rather than reusing it.    //
//                                                                          //
// Program (A0 and A1 seeded after reset; there is no MOVEA yet):            //
//                                                                          //
//   A0 = $0480, A1 = $0484                                                  //
//   1: MOVE.L #$CAFEBABE,D0   203C CAFE BABE                                //
//   4: MOVE.L D0,(A0)         2080          store to $0480                  //
//   5: MOVE.L #$0BADF00D,D1   223C 0BAD F00D                                //
//   8: MOVE.L D1,(A1)         2281          store to $0484                  //
//   9: MOVE.L (A0),D2         2410          read $0480 back                 //
//                                                                          //
// The read-back is the real check: it proves the store reached memory       //
// rather than merely that the instruction retired. Two different values go  //
// to two adjacent addresses so a store that lands at the wrong one, or that //
// writes the address instead of the data, shows up as the other value       //
// rather than as zero.                                                      //
//                                                                          //
// D2 is compared against CAFEBABE specifically, not against D0: comparing   //
// two registers the program itself set would pass if BOTH stores were       //
// dropped and the read returned stale memory that happened to match.        //
//                                                                          //
// On milestone 30's RTL the store does not decode.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_store;

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
	dut.u_l1.mem[1] = 16'h203C;   // MOVE.L #$CAFEBABE,D0
	dut.u_l1.mem[2] = 16'hCAFE;
	dut.u_l1.mem[3] = 16'hBEEF;
	dut.u_l1.mem[4] = 16'h2080;   // MOVE.L D0,(A0)
	dut.u_l1.mem[5] = 16'h223C;   // MOVE.L #$0BADF00D,D1
	dut.u_l1.mem[6] = 16'h0BAD;
	dut.u_l1.mem[7] = 16'hF00D;
	dut.u_l1.mem[8] = 16'h2281;   // MOVE.L D1,(A1)
	dut.u_l1.mem[9] = 16'h2410;   // MOVE.L (A0),D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Seeded after reset: the regfile clears areg, and no instruction here
	// can load an address register yet.
	@(posedge clk);
	dut.u_cpu.u_regfile.areg[0] = 32'h0000_0480;
	dut.u_cpu.u_regfile.areg[1] = 32'h0000_0484;

	repeat ((PROG_WORDS + 30) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hCAFE_BEEF) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected CAFEBEEF", dbg_d0);
	end
	if (dbg_d1 !== 32'h0BAD_F00D) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0BADF00D", dbg_d1);
	end
	// The load-bearing check: reading $0480 back must give what was stored
	// there, not what went to $0484 and not the address itself.
	if (dbg_d2 !== 32'hCAFE_BEEF) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected CAFEBEEF (the store did not reach $0480)", dbg_d2);
	end
	// And the second store must have landed at its own address.
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0BAD_F00D) begin
		errors = errors + 1;
		$display("FAIL: memory at $0484 = %h%h, expected 0BADF00D",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
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
