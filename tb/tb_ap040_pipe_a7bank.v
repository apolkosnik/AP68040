//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: which stack a  //
// write to A7 lands on)                                                    //
//                                                                          //
// tb_ap040_pipe_a7bank.v - an A7 write followed by a mode switch           //
//                                                                          //
// A7 is three registers. ap040_pipe_regfile.v picks between USP, ISP and   //
// MSP with one select, driven from the forwarded SR -- and uses that same  //
// select for reads AND for writes.                                         //
//                                                                          //
// The forward exists for the reads and is right for them: an instruction   //
// in EA-fetch reading A7 the cycle a MOVE-to-SR is in EX has to see the    //
// bank that MOVE-to-SR is switching to. A WRITE to A7 is the other way     //
// round. It belongs to an OLDER instruction, one that has already gone     //
// through EX, and its bank was fixed when it executed -- before the        //
// younger MOVE-to-SR existed as far as it is concerned.                    //
//                                                                          //
// With both on one select, a MOVE-to-SR one instruction behind an A7 write //
// redirects that write into the bank it is switching TO:                   //
//                                                                          //
//   MOVEQ   #$50,D6 ; MOVEC D6,USP       USP = $50, a value to protect     //
//   MOVEQ   #0,D0                        the SR the switch will write      //
//   MOVEA.L #$12345678,A7                supervisor: this is ISP           //
//   MOVE    D0,SR                        S -> 0, one instruction later     //
//                                                                          //
// ISP must be $12345678 and USP must still be $50. The failing behaviour   //
// puts $12345678 in USP and leaves ISP at zero, so both halves of the      //
// check move together and either one alone would be ambiguous.             //
//                                                                          //
// This is not a corner of the architecture. It is the instruction pair a   //
// supervisor uses to hand control to user code: set up the supervisor      //
// stack, then drop privilege. Getting it wrong loses the supervisor stack  //
// pointer AND corrupts the user one, and the next exception returns        //
// through whatever the user stack happened to hold.                        //
//                                                                          //
// The mode switch is checked too, so a decoder that quietly failed to      //
// execute the MOVE-to-SR at all could not pass by leaving S set.           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_a7bank;

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
wire [31:0] dbg_d0, dbg_d1;
wire [15:0] dbg_sr;
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

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_sr(dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h7C50;   // MOVEQ #$50,D6
	dut.u_l1.mem[ 2] = 16'h4E7B;   // MOVEC D6,USP
	dut.u_l1.mem[ 3] = 16'h6800;
	dut.u_l1.mem[ 4] = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[ 5] = 16'h2E7C;   // MOVEA.L #$12345678,A7   (supervisor: ISP)
	dut.u_l1.mem[ 6] = 16'h1234;
	dut.u_l1.mem[ 7] = 16'h5678;
	dut.u_l1.mem[ 8] = 16'h46C0;   // MOVE D0,SR              (S -> 0)
	dut.u_l1.mem[ 9] = 16'h4E71;   // NOP
	dut.u_l1.mem[10] = 16'h220F;   // MOVE.L A7,D1  (user mode: reads USP)
	dut.u_l1.mem[11] = 16'h4E71;   // NOP (drain)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 100) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.isp !== 32'h1234_5678) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 12345678 (MOVEA.L #$12345678,A7 executed in SUPERVISOR mode, so it writes ISP -- the MOVE-to-SR behind it must not redirect it)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dut.u_cpu.u_regfile.usp !== 32'h0000_0050) begin
		errors = errors + 1;
		$display("FAIL: USP = %h, expected 00000050 (12345678 here means the older A7 write was banked through the YOUNGER instruction's SR)",
		         dut.u_cpu.u_regfile.usp);
	end
	if (dbg_d1 !== 32'h0000_0050) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000050 (MOVE.L A7,D1 runs in USER mode and must read USP; 12345678 is the write-through bypass forwarding a write that went to a DIFFERENT bank of A7)",
		         dbg_d1);
	end
	if (dbg_sr[13] !== 1'b0) begin
		errors = errors + 1;
		$display("FAIL: SR.S = %b, expected 0 (MOVE D0,SR with D0 = 0 must actually switch to user mode)", dbg_sr[13]);
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
