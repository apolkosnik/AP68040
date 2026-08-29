//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 14: exceptions)     //
//                                                                          //
// tb_ap040_pipe_exc.v - illegal instruction / TRAP #n exception entry      //
//                                                                          //
// This pipeline's first exception-entry test: format $0 (4-word) frame     //
// contents, A7's decrement, and the vector-table-driven redirect, for the   //
// two vectors this milestone implements -- see ap040_decode.v's and         //
// ap040_ea_fetch.v's headers for why format $2 (address error) is deferred    //
// to its own milestone. Two sub-cases, DELIBERATELY chained through a real     //
// JMP (not just back-to-back like BSR/JSR's chaining) to prove the exception's  //
// flush/redirect composes correctly with ordinary control flow afterward, not    //
// just that it fires once in isolation:                                          //
//                                                                          //
// Case A -- illegal instruction (opcode 0x0000, matches nothing this decoder      //
// recognizes): vector 4. The frame's stacked PC is the illegal opcode's OWN         //
// address (you can't "return past" it) -- D1 (poison, the word right behind it)      //
// must NEVER run, proving the exception fires before any fall-through. The           //
// handler (marker MOVEQ #7,D2) then JMPs back to resume the mainline program AT       //
// the TRAP instruction below -- exercising a SECOND, ordinary misprediction            //
// recovery (JMP's own, unrelated to the exception machinery) immediately after          //
// an exception recovery, with nothing left over from the first to corrupt it.            //
//                                                                          //
// Case B -- TRAP #5 (vector 32+5=37): format $0 again, but the frame's stacked      //
// PC is the FOLLOWING instruction's address (id_next_pc) -- TRAP is architecturally    //
// a subroutine call, not a fault, so its "return address" is the normal              //
// after-the-instruction one, the same distinction BSR's push already established.     //
// D3 (poison, the word right behind the TRAP) must never run either. A7 is NOT         //
// reseeded between cases -- it keeps decrementing from wherever case A left it,        //
// incidentally proving the frame push works correctly a second time from a             //
// DIFFERENT base, the same "chain through the same register" property BSR's two        //
// cases already established for A7.                                                    //
//                                                                          //
//  Mainline (PC_RESET-relative word index):                                //
//   0: NOP                                                                //
//   1: 0x0000            (illegal)     own addr $402                      //
//   2: MOVEQ #99,D1       (0x7263)      poison A: must NOT run              //
//   3: TRAP #5            (0x4E45)      own addr $406, return addr $408      //
//                                        (resumed here by the illegal          //
//                                        handler's JMP, not fallen into)        //
//   4: MOVEQ #88,D3       (0x7658)      poison B: must NOT run                  //
//   5+: NOP (drain)                                                              //
//                                                                          //
//  Illegal handler @ byte $440 (word idx 32):                              //
//   MOVEQ #7,D2           (0x7407)      marker: illegal handler ran         //
//   JMP (A2)              (0x4ED2)      A2 seeded = $406 (TRAP's own addr)   //
//                                                                          //
//  TRAP handler @ byte $460 (word idx 48):                                 //
//   MOVEQ #9,D4           (0x7809)      marker: TRAP handler ran            //
//   NOP (drain)                                                            //
//                                                                          //
//  Vector table (word-index-computed PC_RESET-relative, see                //
//  ap040_ea_fetch.v's header): vector 4 -> $440, vector 37 -> $460.         //
//                                                                          //
//  A7 starts at byte $600. Case A pushes an 8-byte format-$0 frame at        //
//  $5F8, leaves A7=$5F8. Case B pushes another at $5F0, leaves A7=$5F0.       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_exc;

localparam PROG_WORDS      = 40;
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
wire [31:0] dbg_d4, dbg_d5, dbg_d6, dbg_d7;
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
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	// Mainline
	dut.u_l1.mem[1] = 16'h0000;   // illegal (matches nothing)
	dut.u_l1.mem[2] = 16'h7263;   // MOVEQ #99,D1 (poison A, must not run)
	dut.u_l1.mem[3] = 16'h4E45;   // TRAP #5 (vector 37)
	dut.u_l1.mem[4] = 16'h7658;   // MOVEQ #88,D3 (poison B, must not run)

	// Illegal handler @ word idx 32 (byte $440)
	dut.u_l1.mem[32] = 16'h7407;  // MOVEQ #7,D2
	dut.u_l1.mem[33] = 16'h4ED2;  // JMP (A2)

	// TRAP handler @ word idx 48 (byte $460)
	dut.u_l1.mem[48] = 16'h7809;  // MOVEQ #9,D4
	dut.u_l1.mem[49] = 16'h4E71;  // NOP (drain)

	// Vector table: vector 4 -> $440 (word idx 3592/3593),
	// vector 37 -> $460 (word idx 3658/3659) -- see header.
	dut.u_l1.mem[3592] = 16'h0000;
	dut.u_l1.mem[3593] = 16'h0440;
	dut.u_l1.mem[3658] = 16'h0000;
	dut.u_l1.mem[3659] = 16'h0460;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// See tb_ap040_pipe_move_mem.v's header for why the poke must land
	// here, past the reset edge's own NBA region.
	dut.u_regfile.areg[2] = 32'h0000_0406;  // A2: resume-mainline target for JMP
	dut.u_regfile.isp     = 32'h0000_0600;  // A7

	repeat (PROG_WORDS + 80) @(posedge clk);

	// -------------------------------------------------- Case A: illegal
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (poison A ran -- illegal instruction fell through instead of trapping)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000007 (illegal handler did not run)", dbg_d2);
	end
	// Frame @ SP=$5F8 (word idx $0FC/$0FE): SR=$2700 (AP040_SR_RESET's own
	// value -- milestone 15 made SR a real register, and nothing in this
	// program writes it before the fault, so it's still the reset value:
	// S=1/IPL=7, ccr=0), PC=$00000402 (the illegal opcode's OWN address),
	// FmtVec=$0010 (format 0, vector 4 -> 4*4=$10).
	if (dut.u_l1.mem[16'h0FC] !== 16'h2700 || dut.u_l1.mem[16'h0FD] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: illegal frame word0 (SR:PChi) = %h%h, expected 27000000",
		          dut.u_l1.mem[16'h0FC], dut.u_l1.mem[16'h0FD]);
	end
	if (dut.u_l1.mem[16'h0FE] !== 16'h0402 || dut.u_l1.mem[16'h0FF] !== 16'h0010) begin
		errors = errors + 1;
		$display("FAIL: illegal frame word1 (PClo:FmtVec) = %h%h, expected 04020010",
		          dut.u_l1.mem[16'h0FE], dut.u_l1.mem[16'h0FF]);
	end

	// -------------------------------------------------- Case B: TRAP #5
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (poison B ran -- TRAP fell through instead of trapping)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0009) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000009 (TRAP handler did not run)", dbg_d4);
	end
	// Frame @ SP=$5F0 (word idx $0F8/$0FA): SR=$2700 (same reset value as
	// case A -- the illegal handler's MOVEQ #7,D2 sets N=Z=V=C=0, X
	// unchanged, so it doesn't perturb SR either), PC=$00000408 (the
	// FOLLOWING instruction's address -- TRAP's return address, not its
	// own), FmtVec=$0094 (format 0, vector 37 -> 37*4=$94).
	if (dut.u_l1.mem[16'h0F8] !== 16'h2700 || dut.u_l1.mem[16'h0F9] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: TRAP frame word0 (SR:PChi) = %h%h, expected 27000000",
		          dut.u_l1.mem[16'h0F8], dut.u_l1.mem[16'h0F9]);
	end
	if (dut.u_l1.mem[16'h0FA] !== 16'h0408 || dut.u_l1.mem[16'h0FB] !== 16'h0094) begin
		errors = errors + 1;
		$display("FAIL: TRAP frame word1 (PClo:FmtVec) = %h%h, expected 04080094",
		          dut.u_l1.mem[16'h0FA], dut.u_l1.mem[16'h0FB]);
	end

	// A7 decremented by exactly 8 twice (two format-$0 frames), from $600.
	if (dut.u_regfile.isp !== 32'h0000_05F0) begin
		errors = errors + 1;
		$display("FAIL: A7 (isp) = %h, expected 000005f0 (two format-$0 frames, -8 each, from 00000600)", dut.u_regfile.isp);
	end

	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000", dbg_ccr[3:0]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
