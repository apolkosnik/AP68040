//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 78: the flush cycle)      //
//                                                                          //
// tb_ap040_pipe_excexc.v - the instruction right behind an exception entry //
//                                                                          //
// An exception entry sits in EA-fetch for its frame push and vector read.  //
// The instruction after it waits in EA-calc and moves into EA-fetch the    //
// cycle the entry departs -- which is the cycle EX raises flush for it.    //
// The output block ignores that cycle, but the stage's combinational side  //
// effects did not, so for one cycle that instruction was live: a store     //
// wrote, a push wrote, and a faulting instruction's own frame beat 0 went   //
// out at ISP-8 while the first entry's A7 was still uncommitted -- on top  //
// of the first frame.                                                      //
//                                                                          //
// Two shapes, both common in real code:                                    //
//                                                                          //
//   MOVEA.L #$0B00,A0 / MOVEQ #$11,D0                                       //
//   TRAP #1            handler: A0 := $0C00, D0 := $22, D2 := 1; RTE       //
//   MOVE.L D0,(A0)     must write $22 to $0C00 and NOTHING to $0B00        //
//                      (the speculative store wrote $11 to the OLD A0)     //
//   MOVEQ #20,D0                                                           //
//   CHK.W #10,D0       six-word frame at $05F4; handler: D4 := $33; RTE    //
//   TRAP #4            its speculative beat 0 landed at $05F8 -- the CHK    //
//                      frame's {PC_lo, fmt/vec} -- so the CHK handler's    //
//                      RTE went to $2700; handler: D5 := $44; RTE          //
//   MOVEQ #$55,D6                                                          //
//                                                                          //
// Checks: [$0B00] still 0, [$0C00] = $22, D2/D4/D5/D6 markers, ISP $0600,  //
// SR $2700, drained. On milestone-77 RTL [$0B00] = $11 and the CHK/TRAP    //
// pair never returns.                                                      //
//                                                                          //
// Vectors: n at word index 3584 + 2n.                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_excexc;

localparam PROG_WORDS      = 96;
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

initial begin
	#1;
	dut.u_l1.mem[1 ] = 16'h207C;   // MOVEA.L #$0B00,A0
	dut.u_l1.mem[2 ] = 16'h0000;
	dut.u_l1.mem[3 ] = 16'h0B00;
	dut.u_l1.mem[4 ] = 16'h7011;   // MOVEQ #$11,D0
	dut.u_l1.mem[5 ] = 16'h4E41;   // TRAP #1
	dut.u_l1.mem[6 ] = 16'h2080;   // MOVE.L D0,(A0)     the instruction behind the entry
	dut.u_l1.mem[7 ] = 16'h7014;   // MOVEQ #20,D0
	dut.u_l1.mem[8 ] = 16'h41BC;   // CHK.W #10,D0       six-word frame
	dut.u_l1.mem[9 ] = 16'h000A;
	dut.u_l1.mem[10] = 16'h4E44;   // TRAP #4            the instruction behind the entry, itself an entry
	dut.u_l1.mem[11] = 16'h7C55;   // MOVEQ #$55,D6
	dut.u_l1.mem[12] = 16'h4E71;   // NOP

	dut.u_l1.mem[512] = 16'h207C;  // TRAP #1 handler @ $0800: MOVEA.L #$0C00,A0
	dut.u_l1.mem[513] = 16'h0000;
	dut.u_l1.mem[514] = 16'h0C00;
	dut.u_l1.mem[515] = 16'h7022;  // MOVEQ #$22,D0
	dut.u_l1.mem[516] = 16'h7401;  // MOVEQ #1,D2
	dut.u_l1.mem[517] = 16'h4E73;  // RTE

	dut.u_l1.mem[544] = 16'h7833;  // CHK handler @ $0840: MOVEQ #$33,D4
	dut.u_l1.mem[545] = 16'h4E73;  // RTE

	dut.u_l1.mem[576] = 16'h7A44;  // TRAP #4 handler @ $0880: MOVEQ #$44,D5
	dut.u_l1.mem[577] = 16'h4E73;  // RTE

	dut.u_l1.mem[896]  = 16'h0000; dut.u_l1.mem[897]  = 16'h0000;   // [$0B00]
	dut.u_l1.mem[1024] = 16'h0000; dut.u_l1.mem[1025] = 16'h0000;   // [$0C00]

	dut.u_l1.mem[3596] = 16'h0000; dut.u_l1.mem[3597] = 16'h0840;   // vector 6
	dut.u_l1.mem[3650] = 16'h0000; dut.u_l1.mem[3651] = 16'h0800;   // vector 33
	dut.u_l1.mem[3656] = 16'h0000; dut.u_l1.mem[3657] = 16'h0880;   // vector 36
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("[$0B00] (the old A0: nothing may land here)", {dut.u_l1.mem[896],  dut.u_l1.mem[897]},  32'h0000_0000);
	check32("[$0C00] (the new A0 gets the new D0)",        {dut.u_l1.mem[1024], dut.u_l1.mem[1025]}, 32'h0000_0022);
	check32("D2 (TRAP #1 handler ran)",  dbg_d2,                32'h0000_0001);
	check32("D4 (CHK handler ran)",      dut.u_cpu.u_regfile.dreg[4], 32'h0000_0033);
	check32("D5 (TRAP #4 handler ran)",  dut.u_cpu.u_regfile.dreg[5], 32'h0000_0044);
	check32("D6 (end reached)",          dut.u_cpu.u_regfile.dreg[6], 32'h0000_0055);
	check32("ISP",                       dut.u_cpu.u_regfile.isp,     32'h0000_0600);
	check32("SR",                        {16'd0, dut.u_cpu.sr},       32'h0000_2700);

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
