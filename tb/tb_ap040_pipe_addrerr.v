//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 17: address error)  //
//                                                                          //
// tb_ap040_pipe_addrerr.v - format $2 exception entry, odd JMP/JSR targets //
//                                                                          //
// This pipeline's first format-$2 (6-word, 12-byte) frame -- every earlier   //
// exception (illegal, TRAP, privilege violation) is format $0. Two cases,     //
// chained through a manual JMP-based resume (the same technique                //
// tb_ap040_pipe_exc.v's illegal-instruction handler already established --       //
// RTE only handles format $0, per ap040_ea_fetch.v's header, so it can't be       //
// used to return from either of these):                                           //
//                                                                          //
// Case A -- JMP (An) with an odd target. Verifies the MODE-DEPENDENT, bit-exact    //
// stacked-PC quirk ap040_core.v's own S_JMP1 encodes: the frame's PC field is        //
// the JMP instruction's OWN address + 2 (gencpu's incpc(2), NOT this decoder's         //
// own id_next_pc -- the two formulas coincide for JMP (An) specifically, but            //
// this is verified against the real quirk, not assumed to always match). D1/D2           //
// (poison, right behind the JMP) must never run -- the SAME freeze-the-whole-             //
// pipeline-upstream mechanism illegal/TRAP/priv already rely on protects this            //
// too, now proven for a THIRD, longer (format-$2, one more write beat) exception          //
// sequence.                                                                                //
//                                                                          //
// Case B -- JSR (An) with an odd target. Genuinely DIFFERENT from JMP's own          //
// convention, not just a copy: the frame's PC field is the odd target itself           //
// (ea_target, RAW, not rounded), per ap040_core.v's own S_JSR1 comment -- "the           //
// fault [is] on the INSTRUCTION FETCH at the odd target, so the frame names               //
// THAT target, not this instruction." The extra address-field longword (both               //
// cases) DOES round down (`{ea_target[31:1],1'b0}`) -- the PC field and the                 //
// address field are NOT the same value for JSR, a real, easy-to-get-wrong                    //
// distinction this test specifically isolates. D4/D5 (poison) must never run                  //
// either, but for a DIFFERENT reason than case A's: JSR's push is skipped                       //
// entirely for an odd target (ap040_core.v's own S_JSR1, verified, not assumed) --               //
// there is no return address to protect if the call itself never completes, so                   //
// ISP must decrement by EXACTLY 12 (one frame) for this case too, not 12+4 (a                      //
// frame plus an attempted, now-orphaned push).                                                      //
//                                                                          //
// Word layout (PC_RESET-relative):                                         //
//  0: NOP                                                                 //
//  1: JMP (A0)         (0x4ED0)  A0 = $407 (odd) -- own addr $402           //
//  2: MOVEQ #$63,D1    (0x7263)  poison A: must NOT run                      //
//  3: MOVEQ #$05,D2    (0x7405)  must NOT run                                 //
//  4: JSR (A1)         (0x4E91)  A1 = $40D (odd) -- own addr $408             //
//  5: MOVEQ #$77,D4    (0x7877)  poison B: must NOT run                        //
//  6: MOVEQ #$88,D5    (0x7A88)  must NOT run                                   //
//                                                                          //
//  Address error is ONE vector (3) for BOTH cases -- there is no separate    //
//  "handler B" vector to dispatch to, so a single shared handler @ byte      //
//  $500 (word idx 128) must tell the two entries apart itself. D7 (reset     //
//  value 0, never otherwise touched) is used purely as an entry counter:     //
//   ADD.L D7,D7        (0xDE87)  Z=1 iff D7==0 (first entry); doubles D7     //
//                                otherwise (harmless, D7 isn't checked)      //
//   BNE.B ->$50A        (0x6606)  second entry: skip straight to handler B   //
//   MOVEQ #$11,D3      (0x7611)  first entry only: marker, handler ran once  //
//   MOVEQ #1,D7        (0x7E01)  first entry only: arm the flag              //
//   JMP (A2)           (0x4ED2)  first entry only: A2 seeded = $408, resumes //
//                                at the JSR -- which is ALSO an odd target,   //
//                                so it re-faults through this SAME vector,    //
//                                landing back here a second time. (An         //
//                                earlier version of this test gave handler A  //
//                                an unconditional JMP back to the JSR with no //
//                                way to tell entries apart -- that re-faulted  //
//                                forever, draining the stack 12 bytes/pass    //
//                                and never reaching D6/handler-B territory;    //
//                                caught by tracing eac_pc/eaf_pc/isp cycle-by- //
//                                cycle, not by inspection.)                    //
//  $50A: MOVEQ #$22,D6  (0x7C22)  second entry: marker, handler B path ran     //
//  $50C: BRA.B <-2>     (0x60FE)  self-loop -- see tb_ap040_pipe_rts_rte.v's   //
//                                header for why NOP padding alone isn't        //
//                                enough once a generous cycle budget outlasts    //
//                                however many words of it there are.             //
//                                                                          //
//  Vector table: vector 3 (address error) -> $500 (word idx 3590/3591, same    //
//  PC_RESET-relative wraparound convention every exception test since             //
//  milestone 14 has used).                                                         //
//                                                                          //
//  A7 starts at $600 (ISP). Case A's frame lands at $5F4 (ISP-12); case B's         //
//  lands at $5E8 (ISP-12 again, from THAT new base) -- BOTH decrements are            //
//  exactly 12, neither more nor less, proving format $2's own size and JSR's           //
//  skipped-push both landed correctly.                                                  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_addrerr;

localparam PROG_WORDS      = 60;
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
wire [15:0] dbg_sr;

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
	.dbg_ccr(dbg_ccr), .dbg_sr(dbg_sr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1] = 16'h4ED0;   // JMP (A0)
	dut.u_l1.mem[2] = 16'h7263;   // MOVEQ #$63,D1 (poison A, must not run)
	dut.u_l1.mem[3] = 16'h7405;   // MOVEQ #$05,D2 (must not run)

	dut.u_l1.mem[4] = 16'h4E91;   // JSR (A1)
	dut.u_l1.mem[5] = 16'h7877;   // MOVEQ #$77,D4 (poison B, must not run)
	dut.u_l1.mem[6] = 16'h7A88;   // MOVEQ #$88,D5 (must not run)

	// Shared address-error handler @ word idx 128 (byte $500) -- see header
	// for why one shared handler, distinguishing entries via D7, is required.
	dut.u_l1.mem[128] = 16'hDE87; // ADD.L D7,D7 (Z=1 iff first entry)
	dut.u_l1.mem[129] = 16'h6606; // BNE.B -> $50A (second entry: handler B path)
	dut.u_l1.mem[130] = 16'h7611; // MOVEQ #$11,D3 (first entry only)
	dut.u_l1.mem[131] = 16'h7E01; // MOVEQ #1,D7 (first entry only: arm flag)
	dut.u_l1.mem[132] = 16'h4ED2; // JMP (A2) -- resume at the JSR (idx 4)
	dut.u_l1.mem[133] = 16'h7C22; // MOVEQ #$22,D6 (second entry: handler B path)
	dut.u_l1.mem[134] = 16'h60FE; // BRA.B <-2> (self-loop)

	// Vector table: vector 3 (address error) -> $500.
	dut.u_l1.mem[3590] = 16'h0000;
	dut.u_l1.mem[3591] = 16'h0500;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// See tb_ap040_pipe_move_mem.v's header for why the poke must land
	// here, past the reset edge's own NBA region.
	dut.u_regfile.areg[0] = 32'h0000_0407;  // A0: odd JMP target
	dut.u_regfile.areg[1] = 32'h0000_040D;  // A1: odd JSR target
	dut.u_regfile.areg[2] = 32'h0000_0408;  // A2: handler A's resume target
	dut.u_regfile.isp     = 32'h0000_0600;

	repeat (PROG_WORDS + 140) @(posedge clk);

	// -------------------------------------------------- Case A: JMP odd
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (poison A ran -- odd JMP target did not fault)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000011 (address-error handler A did not run)", dbg_d3);
	end
	// Frame A @ SP=$5F4 (word idx $FA/$FC/$FE): SR=$2700 (reset default,
	// nothing in this program touches CCR before this point), PC=$00000404
	// (JMP's OWN address + 2, NOT id_next_pc -- see header), FmtVec=$200C
	// (format 2, vector 3 -> 3*4=$0C), extra address field=$00000406
	// (the odd target $407 with its LSB cleared).
	if (dut.u_l1.mem[16'h00FA] !== 16'h2700 || dut.u_l1.mem[16'h00FB] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: JMP frame word0 (SR:PChi) = %h%h, expected 27000000",
		          dut.u_l1.mem[16'h00FA], dut.u_l1.mem[16'h00FB]);
	end
	if (dut.u_l1.mem[16'h00FC] !== 16'h0404 || dut.u_l1.mem[16'h00FD] !== 16'h200C) begin
		errors = errors + 1;
		$display("FAIL: JMP frame word1 (PClo:FmtVec) = %h%h, expected 0404200C",
		          dut.u_l1.mem[16'h00FC], dut.u_l1.mem[16'h00FD]);
	end
	if (dut.u_l1.mem[16'h00FE] !== 16'h0000 || dut.u_l1.mem[16'h00FF] !== 16'h0406) begin
		errors = errors + 1;
		$display("FAIL: JMP frame word2 (address field) = %h%h, expected 00000406",
		          dut.u_l1.mem[16'h00FE], dut.u_l1.mem[16'h00FF]);
	end

	// -------------------------------------------------- Case B: JSR odd
	if (dbg_d4 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000000 (poison B ran -- odd JSR target did not fault)", dbg_d4);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000", dbg_d5);
	end
	if (dbg_d6 !== 32'h0000_0022) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 00000022 (address-error handler B did not run)", dbg_d6);
	end
	// Frame B @ SP=$5E8 (word idx $F4/$F6/$F8): SR=$2700, PC=$0000040D --
	// the RAW odd target itself (JSR's own convention, genuinely different
	// from JMP's above -- see header), FmtVec=$200C again, extra address
	// field=$0000040C (the odd target $40D with its LSB cleared -- NOT the
	// same value as the PC field, a real distinction this test isolates).
	if (dut.u_l1.mem[16'h00F4] !== 16'h2700 || dut.u_l1.mem[16'h00F5] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: JSR frame word0 (SR:PChi) = %h%h, expected 27000000",
		          dut.u_l1.mem[16'h00F4], dut.u_l1.mem[16'h00F5]);
	end
	if (dut.u_l1.mem[16'h00F6] !== 16'h040D || dut.u_l1.mem[16'h00F7] !== 16'h200C) begin
		errors = errors + 1;
		$display("FAIL: JSR frame word1 (PClo:FmtVec) = %h%h, expected 040D200C",
		          dut.u_l1.mem[16'h00F6], dut.u_l1.mem[16'h00F7]);
	end
	if (dut.u_l1.mem[16'h00F8] !== 16'h0000 || dut.u_l1.mem[16'h00F9] !== 16'h040C) begin
		errors = errors + 1;
		$display("FAIL: JSR frame word2 (address field) = %h%h, expected 0000040C",
		          dut.u_l1.mem[16'h00F8], dut.u_l1.mem[16'h00F9]);
	end

	// The real point of this whole test: BOTH decrements are exactly 12
	// (one format-$2 frame each), never 12+4 -- JSR's own push must have
	// been skipped entirely, not attempted and then orphaned.
	if (dut.u_regfile.isp !== 32'h0000_05E8) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 000005e8 (two format-$2 frames, -12 each, from $600 -- JSR's push must never have been attempted)", dut.u_regfile.isp);
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
