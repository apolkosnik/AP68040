//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 10: MOVE.L          //
// (d16,An),Dn)                                                             //
//                                                                          //
// tb_ap040_pipe_move_disp.v - real EA-arithmetic + gather-vs-redirect proof //
//                                                                          //
// Two sub-cases, each isolating one of the two new mechanisms milestone 10  //
// actually added (see ap040_decode.v/ap040_ea_fetch.v's headers):           //
//                                                                          //
// Case A -- MOVE.L (d16,A0),D0 with a genuinely nonzero displacement:        //
// proves EA = An + disp is computed and used, not just An alone (a          //
// zero-displacement test couldn't distinguish this from plain (An), the      //
// SAME class of mistake milestone 5's "poison reachable via the correct       //
// path" lesson warns about, just for an address instead of a control-flow      //
// path).                                                                     //
//                                                                          //
// Case B -- MOVE.L (d16,A1),D1 immediately followed by a MOVEQ: proves the    //
// gather does NOT trigger IF's speculative redirect (id_redirect_valid/        //
// id_redirect_pc gated `!held_is_move_disp` -- see ap040_decode.v's header).    //
// If that guard were missing or wrong, IF would speculatively jump to           //
// held_pc + 2 + disp (the DISPLACEMENT value misinterpreted as a branch          //
// target) instead of falling through to the MOVEQ -- the displacement is         //
// deliberately chosen small and positive so a broken guard's "redirect"           //
// would land inside this program's own ROM on an innocuous-looking NOP            //
// rather than crash the sim, making D2 (never written on the correct path)         //
// the distinguishing signal, same shape as milestone 7's poison-opcode             //
// discipline.                                                                     //
//                                                                          //
//  0: NOP                                                                 //
//  1: MOVE.L (d16,A0),D0  (0x2028)  A0 = byte addr $600 (word index $100,    //
//  2: <ext: 0x0004>                 PC_RESET-relative); disp=+4 -> EA=$604    //
//                                   (word index $102). Data DIFFERENT from    //
//                                   $100/$101 (case A) so a disp=0 bug would   //
//                                   read the wrong, distinguishable value.     //
//                                                                          //
//  3: MOVE.L (d16,A1),D1  (0x2229)  A1 = byte addr $700 (word index $180);    //
//  4: <ext: 0x0004>                 disp=+4 -> EA=$704 (word index $182 --    //
//                                   a full 2-word shift, same as case A, so   //
//                                   the "must not be read" and "result" pairs //
//                                   below don't overlap at the word level)    //
//  5: MOVEQ #9,D2         (0x7409)  proves gather completion did NOT           //
//                                   redirect IF; D2 stays untouched if a       //
//                                   broken guard skips straight past this      //
//  6-9: NOP (drain)                                                        //
//                                                                          //
//  data (word index $100/$101, byte addr $600): $1111_2222 (must NOT be read) //
//  data (word index $102/$103, byte addr $604): $3333_4444 (case A result)    //
//  data (word index $180/$181, byte addr $700): $5555_6666 (must NOT be read) //
//  data (word index $182/$183, byte addr $704): $7777_8888 (case B result)    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_move_disp;

localparam PROG_WORDS      = 10;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1] = 16'h2028;   // MOVE.L (d16,A0),D0
	dut.u_l1.mem[2] = 16'h0004;   // ext: disp=+4

	dut.u_l1.mem[3] = 16'h2229;   // MOVE.L (d16,A1),D1
	dut.u_l1.mem[4] = 16'h0004;   // ext: disp=+4
	dut.u_l1.mem[5] = 16'h7409;   // MOVEQ #9,D2

	dut.u_l1.mem[16'h100] = 16'h1111;   // must NOT be read (case A, disp=0 trap)
	dut.u_l1.mem[16'h101] = 16'h2222;
	dut.u_l1.mem[16'h102] = 16'h3333;   // case A result ($600+4)
	dut.u_l1.mem[16'h103] = 16'h4444;

	dut.u_l1.mem[16'h180] = 16'h5555;   // must NOT be read (case B, disp=0 trap)
	dut.u_l1.mem[16'h181] = 16'h6666;
	dut.u_l1.mem[16'h182] = 16'h7777;   // case B result ($700+2)
	dut.u_l1.mem[16'h183] = 16'h8888;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// See tb_ap040_pipe_move_mem.v's header for why the poke must land here,
	// past the reset edge's own NBA region, not immediately after nreset=1.
	dut.u_regfile.areg[0] = 32'h0000_0600;
	dut.u_regfile.areg[1] = 32'h0000_0700;

	repeat (PROG_WORDS + 25) @(posedge clk);

	if (dbg_d0 !== 32'h3333_4444) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 33334444 (EA = An + disp not computed, or wrong -- may have read An alone)", dbg_d0);
	end
	if (dbg_d1 !== 32'h7777_8888) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 77778888 (EA = An + disp not computed, or wrong)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0009) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000009 (move-disp gather wrongly triggered IF's speculative redirect)", dbg_d2);
	end
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000", dbg_ccr[3:0]);
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
