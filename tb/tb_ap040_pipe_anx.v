//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 30: (An)+ and -(An))//
//                                                                          //
// tb_ap040_pipe_anx.v - two register writes from one instruction           //
//                                                                          //
// Every instruction before this wrote at most one register. MOVE.L (An)+,Dn //
// writes two: the data to Dn and the updated address to An. The pipeline   //
// had exactly one commit path, and the regfile's aux port was no way out   //
// -- it reaches only USP/ISP/MSP, never a GPR -- so this milestone adds a   //
// second write port, a second commit gate, and a second EX forward.        //
//                                                                          //
// The second forward is the part that is easy to get wrong and silent when  //
// it is: the An update is a real architectural write, so an instruction     //
// immediately after must see it. This program is built so it does.         //
//                                                                          //
// Data (byte address $0480 is word index 64):                              //
//   mem[64],[65] = 1111 2222     longword at $0480                         //
//   mem[66],[67] = 3333 4444     longword at $0484                         //
//                                                                          //
// Program:                                                                 //
//   1: MOVE.L ($0480).W,A0   2078 0480   A0 = ... no: see below            //
//                                                                          //
// A0 is loaded by a MOVEQ-style sequence instead, because MOVEA is not     //
// implemented: the test uses the absolute form from milestone 29 to load a //
// DATA register, then relies on (An)+ reading through A0 after A0 is set   //
// by the address-update path itself.                                       //
//                                                                          //
//   1: MOVE.L (A0)+,D0       2018        D0 = 11112222, A0 = 4             //
//   2: MOVE.L (A0)+,D1       2218        D1 = 33334444, A0 = 8             //
//   3: MOVE.L -(A0),D2       2420        A0 = 4, D2 = 33334444             //
//                                                                          //
// A0 starts at 0, which is PC_RESET in this core's flat L1, so (A0)+ reads //
// the word pair at index 0. To land on the data above the test instead     //
// seeds A0 through the second write port itself: the FIRST (A0)+ is the    //
// one under test for its data, and the SECOND proves the update was        //
// forwarded, because it can only read the next longword if A0 advanced and //
// that new value reached the very next instruction.                        //
//                                                                          //
// -(A0) then proves the other direction and that the access uses the        //
// DECREMENTED address: it must re-read the longword the second load just   //
// consumed, not the one after it.                                          //
//                                                                          //
// On milestone 29's RTL none of the three decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_anx;

localparam PROG_WORDS      = 20;
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
	dut.u_l1.mem[1] = 16'h2018;   // MOVE.L (A0)+,D0
	dut.u_l1.mem[2] = 16'h2218;   // MOVE.L (A0)+,D1
	dut.u_l1.mem[3] = 16'h2420;   // MOVE.L -(A0),D2

	// Byte address $0480 is word index 64 relative to PC_RESET.
	dut.u_l1.mem[64] = 16'h1111;
	dut.u_l1.mem[65] = 16'h2222;
	dut.u_l1.mem[66] = 16'h3333;
	dut.u_l1.mem[67] = 16'h4444;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// A0 is seeded directly, and AFTER reset releases: this core has no
	// MOVEA yet, so no instruction can load an address register, and the
	// regfile clears areg on reset -- seeding at time 0 the way the L1's
	// memory is seeded would simply be erased. A0 = 0 is not a usable
	// address either, since l1_addr_b is (byte address - PC_RESET) >> 1.
	// The first instruction cannot read A0 until it reaches EA-fetch several
	// cycles from now, so one cycle of margin is enough.
	@(posedge clk);
	dut.u_cpu.u_regfile.areg[0] = 32'h0000_0480;

	repeat ((PROG_WORDS + 30) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h1111_2222) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11112222 ((A0)+ reads the ORIGINAL address)", dbg_d0);
	end
	// The load-bearing check: D1 can only hold the SECOND longword if the
	// first instruction's A0 update reached the very next instruction, which
	// is the second EX forward this milestone added.
	if (dbg_d1 !== 32'h3333_4444) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 33334444 (the A0 update did not forward to the next instruction)", dbg_d1);
	end
	// -(A0) must re-read what the second load consumed: proof of both the
	// decrement and that the ACCESS uses the decremented address.
	if (dbg_d2 !== 32'h3333_4444) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 33334444 (-(An) must access the DECREMENTED address)", dbg_d2);
	end
	// And A0 itself must end where -(A0) left it.
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0484) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000484 (two increments then one decrement)", dut.u_cpu.u_regfile.areg[0]);
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
