//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 93: the MOVEC      //
// interlock has to stop the stage it is in)                                //
//                                                                          //
// tb_ap040_pipe_creghold.v - a stall that held the wrong stages            //
//                                                                          //
// Milestone 92 gave EA-fetch a one-cycle hazard for a reader of A7 behind  //
// a MOVEC to a stack pointer, and put it in eaf_stall. That signal tells   //
// the stages BEHIND this one to wait. It does not stop this one: the       //
// instruction still retires, and its memory request still goes out, every  //
// cycle the hazard lasts.                                                  //
//                                                                          //
// The register-read check that came with it passed anyway, and the reason  //
// is worth stating: the instruction executed TWICE, and the second pass    //
// -- after MOVEC had landed -- wrote the right answer over the first. A    //
// check that reads only the final value cannot tell that from working.     //
//                                                                          //
// Three consumers, each immediately behind its own MOVEC, because they     //
// fail in three different ways:                                            //
//                                                                          //
//   MOVEC D0,ISP ($1200) ; MOVE.L (A7),D1    a READ of the new pointer     //
//   MOVEC D0,ISP ($1200) ; MOVE.L D2,-(A7)   a WRITE through it            //
//   MOVEC D0,ISP ($1200) ; ADDQ.L #4,A7      an UPDATE of it               //
//                                                                          //
// The read is the one that self-corrects. The push does not: run twice it  //
// posts twice and decrements twice, so it leaves $11F8 instead of $11FC    //
// and puts data in a word nothing asked it to touch. The update does not   //
// either: run against the stale $1000 it gives $1004.                      //
//                                                                          //
// Sentinels sit at $0FFC and $0FF8, where a push against the stale pointer //
// would land, so the failure names itself instead of only showing up as a  //
// count.                                                                   //
//                                                                          //
// The register commits are counted too, because an instruction that runs   //
// twice commits twice and a value check cannot always see that. The        //
// counter follows the MAIN write port only, so the program's count is      //
// seven: the four MOVE.L immediates, the load, the MOVEQ and the ADDQ.     //
// MOVEC writes a control register through the auxiliary port and the       //
// push's decrement of A7 goes through the second one, so neither is        //
// counted.                                                                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_creghold;

localparam PROG_WORDS      = 48;
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
wire [31:0] dbg_d1, dbg_d2, dbg_d3;
wire [31:0] dbg_commits;
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

	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_sr (dbg_sr),
	.dbg_ccr(dbg_ccr), .dbg_commits(dbg_commits)
);

integer errors = 0;

integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00001200,D0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h1200;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1200)
	dut.u_l1.mem[10] = 16'h0804;
	dut.u_l1.mem[11] = 16'h2217;   // MOVE.L (A7),D1
	dut.u_l1.mem[12] = 16'h745A;   // MOVEQ #$5A,D2   (ahead of the MOVEC, so the
	dut.u_l1.mem[13] = 16'h203C;   // MOVE.L #$00001200,D0   push is the instruction
	dut.u_l1.mem[14] = 16'h0000;   //                        immediately behind it)
	dut.u_l1.mem[15] = 16'h1200;
	dut.u_l1.mem[16] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1200)
	dut.u_l1.mem[17] = 16'h0804;
	dut.u_l1.mem[18] = 16'h2F02;   // MOVE.L D2,-(A7)
	dut.u_l1.mem[19] = 16'h203C;   // MOVE.L #$00001200,D0
	dut.u_l1.mem[20] = 16'h0000;
	dut.u_l1.mem[21] = 16'h1200;
	dut.u_l1.mem[22] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1200)
	dut.u_l1.mem[23] = 16'h0804;
	dut.u_l1.mem[24] = 16'h588F;   // ADDQ.L #4,A7
	dut.u_l1.mem[25] = 16'h4E71;   // NOP (drain)

	// $1000 -- what a stale read would return.
	dut.u_l1.mem[1536] = 16'h1111;
	dut.u_l1.mem[1537] = 16'h2222;
	// $1200 -- what the new pointer points at.
	dut.u_l1.mem[1792] = 16'hAABB;
	dut.u_l1.mem[1793] = 16'hCCDD;
	// $11FC -- where the push must land.
	dut.u_l1.mem[1790] = 16'h9999;
	dut.u_l1.mem[1791] = 16'h9999;
	// $0FFC and $0FF8 -- where a push against the stale pointer would land.
	dut.u_l1.mem[1534] = 16'h7777;
	dut.u_l1.mem[1535] = 16'h7777;
	dut.u_l1.mem[1532] = 16'h6666;
	dut.u_l1.mem[1533] = 16'h6666;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d1 !== 32'hAABB_CCDD) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected aabbccdd (MOVE.L (A7),D1 must read through the pointer MOVEC just wrote; 11112222 is the old one)", dbg_d1);
	end
	if ({dut.u_l1.mem[1790], dut.u_l1.mem[1791]} !== 32'h0000_005A) begin
		errors = errors + 1;
		$display("FAIL: $11FC = %h%h, expected 0000005a (MOVE.L D2,-(A7) must push once, through the new pointer)",
		         dut.u_l1.mem[1790], dut.u_l1.mem[1791]);
	end
	if ({dut.u_l1.mem[1534], dut.u_l1.mem[1535]} !== 32'h7777_7777 ||
	    {dut.u_l1.mem[1532], dut.u_l1.mem[1533]} !== 32'h6666_6666) begin
		errors = errors + 1;
		$display("FAIL: $0FFC/$0FF8 = %h%h/%h%h, expected 77777777/66666666 (the push went through the STALE stack pointer)",
		         dut.u_l1.mem[1534], dut.u_l1.mem[1535], dut.u_l1.mem[1532], dut.u_l1.mem[1533]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_1204) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00001204 (push to $11FC, then ADDQ.L #4 on the pointer MOVEC wrote; 00001004 means ADDQ ran against the stale one)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dbg_commits !== 32'd7) begin
		errors = errors + 1;
		$display("FAIL: %0d register commits, expected 7. An instruction held by a hazard that does not stop its own stage retires once per cycle of it, and for ADDQ the last pass writes the right answer over the earlier ones -- so the value is right and only the count says it ran more than once.",
		         dbg_commits);
	end
	if (writes !== 1) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted to the L1, expected 1. A hazard that only holds the stages BEHIND this one lets the instruction retire every cycle it lasts -- and a push that runs twice pushes twice.",
		         writes);
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
