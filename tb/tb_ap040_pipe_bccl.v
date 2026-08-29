//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 7: multi-word       //
// fetch/decode, Bcc.W/Bcc.L)                                               //
//                                                                          //
// tb_ap040_pipe_bccl.v - Bcc.L (32-bit displacement) proof                //
//                                                                          //
// Same two-case shape as tb_ap040_pipe_bccw.v, but the WORD form's         //
// single-cycle gather no longer applies: opcode low byte == 0xFF selects   //
// the LONG form, which needs TWO extension words (ext_pending goes 2 -> 1  //
// -> complete), the only path that proves ap040_decode.v's word count      //
// actually advances across more than one gather cycle and that             //
// disp_acc's shift-and-combine is high-word-first (ap040_core.v:1817's     //
// pattern -- see ap040_decode.v's header) rather than swapped.             //
//                                                                          //
// Case 2's fall-through check (opcode_pc + 6, three words total) is the    //
// one that would fail if id_next_pc's long-form arithmetic were wrong --   //
// distinct from tb_ap040_pipe_bccw.v's opcode_pc + 4 check, so a bug that  //
// only affects the long form specifically (as opposed to the shared        //
// mechanism) cannot hide behind the word-form test passing.                //
//                                                                          //
// A stale (pre-milestone-7) recovery for case 2 lands on this instruction's //
// FIRST extension word (opcode_pc + 2 = ext hi's own address, not the       //
// correct opcode_pc + 6) and tries to decode it as a fresh opcode -- same   //
// self-healing blind spot as tb_ap040_pipe_bccw.v's case 2 (a misdecoded    //
// "unimplemented" bubble there just advances +2 and lands back on the real  //
// ext lo word, which itself is harmless too, then +2 again lands right back //
// on the correct fall-through -- no observable difference). So ext hi here  //
// doubles as a poison OPCODE (MOVEQ #0xAA,D1), exactly as in the word-form  //
// test -- see its header for why a value-only poison isn't enough. Discarded//
// on the correctly-working path, same reasoning as there.                  //
//                                                                          //
//  0: NOP                                                                 //
//  1: MOVEQ #0,D0    (0x7000)  Z=1                                        //
//  2: BEQ.L <ext,ext>(0x67ff)  adjacent to #1; condition true --           //
//                              correctly predicted taken; ext hi/lo @ #3/#4//
//  3: <ext hi: 0x0000>                                                     //
//  4: <ext lo: 0x0006>         disp=0x00000006, base=addr(#3), target=#6   //
//  5: MOVEQ #99,D1   (0x7263)  poison: must NOT run                       //
//  6: MOVEQ #5,D2    (0x7405)  target: proves case 1                      //
//                                                                          //
//  7: MOVEQ #1,D0    (0x7001)  Z=0                                        //
//  8: BEQ.L <ext,ext>(0x67ff)  adjacent to #7; condition FALSE --          //
//                              speculatively predicted taken anyway,       //
//                              must be caught and recovered; ext @ #9/#10  //
//  9: <ext hi: 0x72AA>         discarded / poison opcode if a stale         //
//                              recovery wrongly lands here -- see above     //
// 10: <ext lo: 0x0008>         discarded (mispredicted branch's target      //
//                              is never used) regardless                   //
// 11: MOVEQ #7,D3    (0x7607)  correct fall-through: must run             //
//                              (= opcode_pc(#8) + 6, proving id_next_pc's   //
//                              long-form arithmetic)                       //
// 12: BRA +2 words   (0x6002)  jumps over #13 -- same isolation discipline //
//                              as milestone 5's test                       //
// 13: MOVEQ #88,D3   (0x7658)  branch's (wrong) taken speculative target,  //
//                              reached only if gather_disp itself were     //
//                              computed wrong: must NOT run                //
// 14-25: NOP (drain)                                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bccl;

localparam PROG_WORDS      = 26;
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
	dut.u_l1.mem[1]  = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[2]  = 16'h67ff;   // BEQ.L <ext,ext> (correctly taken)
	dut.u_l1.mem[3]  = 16'h0000;   // ext hi
	dut.u_l1.mem[4]  = 16'h0006;   // ext lo: disp=+6, target=rom[6]
	dut.u_l1.mem[5]  = 16'h7263;   // MOVEQ #99,D1 (poison, must not run)
	dut.u_l1.mem[6]  = 16'h7405;   // MOVEQ #5,D2  (target, must run)

	dut.u_l1.mem[7]  = 16'h7001;   // MOVEQ #1,D0
	dut.u_l1.mem[8]  = 16'h67ff;   // BEQ.L <ext,ext> (mispredicted -- not taken)
	dut.u_l1.mem[9]  = 16'h72AA;   // discarded disp value / poison opcode if
	                                // a stale recovery wrongly lands here --
	                                // see file header
	dut.u_l1.mem[10] = 16'h0008;   // discarded regardless
	dut.u_l1.mem[11] = 16'h7607;   // MOVEQ #7,D3  (fall-through, must run)
	dut.u_l1.mem[12] = 16'h6002;   // BRA +2 words, jumps over rom[13]
	dut.u_l1.mem[13] = 16'h7658;   // MOVEQ #88,D3 (wrong taken target, must not run)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat (PROG_WORDS + 20) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0001) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000001", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000 (case 1 poison ran -- BEQ.L wrongly not-taken -- or case 2's stale-recovery poison opcode ran -- id_next_pc's long-form arithmetic wrong)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case 1 target did not run -- gather/redirect broken)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000007 (case 2 misprediction recovery broken -- id_next_pc's long-form arithmetic wrong, wrong-taken poison ran, or fall-through did not)", dbg_d3);
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
