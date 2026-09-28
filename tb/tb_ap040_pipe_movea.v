//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 33: MOVEA.L)        //
//                                                                          //
// tb_ap040_pipe_movea.v - a program can finally load an address register   //
//                                                                          //
// Destination mode 001 writes an ADDRESS register. Until now nothing could //
// -- the second write port of milestone 30 updates An as a side effect of  //
// (An)+/-(An), but no instruction targeted one -- so every testbench that  //
// needed an address had to poke dut.u_cpu.u_regfile.areg[] directly after reset. //
// This is the first one that does not, and it is written that way on       //
// purpose: the setup is now part of the program under test.                //
//                                                                          //
// MOVEA sets NO condition codes, unlike every other MOVE. A register check //
// cannot see that, so the CCR is asserted explicitly below.                //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEA.L #$00000480,A0   207C 0000 0480                              //
//   4: MOVE.L  #$FEEDFACE,D1   223C FEED FACE                              //
//   7: CMPI.L  #0,D1           0C81 0000 0000   sets N=1 (D1 is negative)   //
//  10: MOVE.L  D1,(A0)         2081             store through the loaded A0 //
//  11: MOVEA.L D0,A1           2240             A1 = D0 = 0                 //
//  12: MOVE.L  (A0),D2         2410             read it back                //
//                                                                          //
// The store and read-back prove A0 really holds $0480: if MOVEA had written //
// a DATA register instead of an address register -- the single-bit mistake  //
// this milestone can make -- A0 would still be 0 and both would go astray.  //
//                                                                          //
// The CMPI before the two MOVEAs is there so the CCR holds a known non-zero //
// state when they execute. N stays set only if neither MOVEA touched the    //
// flags; had they written CCR from their data, the second one moves D0 = 0  //
// and would leave Z set and N clear.                                        //
//                                                                          //
// On milestone 32's RTL neither MOVEA form decodes.                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movea;

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
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0480;
	dut.u_l1.mem[4]  = 16'h223C;   // MOVE.L #$FEEDFACE,D1
	dut.u_l1.mem[5]  = 16'hFEED;
	dut.u_l1.mem[6]  = 16'hFACE;
	dut.u_l1.mem[7]  = 16'h0C81;   // CMPI.L #0,D1
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h0000;
	dut.u_l1.mem[10] = 16'h2081;   // MOVE.L D1,(A0)
	dut.u_l1.mem[11] = 16'h2240;   // MOVEA.L D0,A1
	dut.u_l1.mem[12] = 16'h2410;   // MOVE.L (A0),D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// No poking: A0 is loaded by the program itself.
	repeat ((PROG_WORDS + 34) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000480 (MOVEA.L #imm,An)", dut.u_cpu.u_regfile.areg[0]);
	end
	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'hFEED_FACE) begin
		errors = errors + 1;
		$display("FAIL: memory at $0480 = %h%h, expected FEEDFACE (the store used a wrong A0)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if (dbg_d2 !== 32'hFEED_FACE) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FEEDFACE", dbg_d2);
	end
	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000000 (MOVEA.L Dn,An)", dut.u_cpu.u_regfile.areg[1]);
	end
	// N must still be set from the CMPI: MOVEA writes no flags. Had it
	// written them, the second MOVEA moved D0 = 0 and would leave Z set.
	// dbg_ccr[3:0] is {N,Z,V,C}.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (MOVEA must not write condition codes)", dbg_ccr[3:0]);
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
