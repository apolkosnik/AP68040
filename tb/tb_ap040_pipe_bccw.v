//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 7: multi-word       //
// fetch/decode, Bcc.W/Bcc.L)                                               //
//                                                                          //
// tb_ap040_pipe_bccw.v - Bcc.W (16-bit displacement) proof                //
//                                                                          //
// Same two-case shape as milestone 5's tb_ap040_pipe_bcc.v (correctly-     //
// predicted-taken + misprediction/recovery), but every branch here uses    //
// the WORD form (opcode low byte == 0x00, one 16-bit extension word)       //
// instead of the byte-form displacement -- this is what actually exercises //
// ap040_decode.v's new gather state machine; Bcc.B never touches it.       //
//                                                                          //
// Case 2's fall-through check is the one that would fail if id_next_pc     //
// were wrong (or still hardcoded as opcode_pc+2, milestone <=6's coincidence//
// -- see ap040_execute.v's header): a 2-word instruction's correct         //
// recovery address is opcode_pc+4, not opcode_pc+2. A stale +2 recovery     //
// would land on this instruction's OWN extension word and try to decode it //
// as a fresh opcode -- but a plain "must not run" poison there is not       //
// enough: since that word is immediately followed in memory by the real    //
// next instruction, a stale recovery that decodes it as a harmless          //
// "unimplemented" bubble would just advance +2 anyway and land right back   //
// on the correct instruction, silently self-healing with no observable      //
// difference (found by mutation-testing this very case). So this word       //
// doubles as a poison OPCODE (MOVEQ #0xAA,D1), not just a poison            //
// displacement VALUE -- a stale recovery landing here visibly corrupts D1   //
// (never otherwise written by this program); on the correctly-working       //
// path the value is simply discarded (a mispredicted branch's displacement  //
// is never used), so this has no effect at all there.                      //
//                                                                          //
//  0: NOP                                                                 //
//  1: MOVEQ #0,D0    (0x7000)  Z=1                                        //
//  2: BEQ.W <ext>    (0x6700)  adjacent to #1; condition true --           //
//                              correctly predicted taken; ext word @ #3    //
//  3: <ext: 0x0004>            displacement, base=addr(#3), target=#5      //
//  4: MOVEQ #99,D1   (0x7263)  poison: must NOT run                       //
//  5: MOVEQ #5,D2    (0x7405)  target: proves case 1                      //
//                                                                          //
//  6: MOVEQ #1,D0    (0x7001)  Z=0                                        //
//  7: BEQ.W <ext>    (0x6700)  adjacent to #6; condition FALSE --          //
//                              speculatively predicted taken anyway,       //
//                              must be caught and recovered; ext word @ #8 //
//  8: <ext: 0x72AA>            discarded displacement value that doubles   //
//                              as a poison opcode -- see above; base=addr(#8)//
//  9: MOVEQ #7,D3    (0x7607)  correct fall-through: must run             //
//                              (= opcode_pc(#7) + 4, proving id_next_pc)   //
// 10: BRA +2 words   (0x6002)  jumps over #11 -- same isolation discipline //
//                              as milestone 5's test                       //
// 11: MOVEQ #88,D3   (0x7658)  branch's (wrong) taken speculative target,  //
//                              reached only if gather_disp itself were     //
//                              computed wrong: must NOT run                //
// 12-23: NOP (drain)                                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bccw;

localparam PROG_WORDS      = 24;
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
	dut.u_l1.mem[2]  = 16'h6700;   // BEQ.W <ext> (correctly taken)
	dut.u_l1.mem[3]  = 16'h0004;   // ext word: disp=+4, target=rom[5]
	dut.u_l1.mem[4]  = 16'h7263;   // MOVEQ #99,D1 (poison, must not run)
	dut.u_l1.mem[5]  = 16'h7405;   // MOVEQ #5,D2  (target, must run)

	dut.u_l1.mem[6]  = 16'h7001;   // MOVEQ #1,D0
	dut.u_l1.mem[7]  = 16'h6700;   // BEQ.W <ext> (mispredicted -- not taken)
	dut.u_l1.mem[8]  = 16'h72AA;   // discarded disp value / poison opcode if
	                                // a stale recovery wrongly lands here --
	                                // see file header
	dut.u_l1.mem[9]  = 16'h7607;   // MOVEQ #7,D3  (fall-through, must run)
	dut.u_l1.mem[10] = 16'h6002;   // BRA +2 words, jumps over rom[11]
	dut.u_l1.mem[11] = 16'h7658;   // MOVEQ #88,D3 (wrong taken target, must not run)
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
		$display("FAIL: D1 = %h, expected 00000000 (case 1 poison ran -- BEQ.W wrongly not-taken -- or case 2's stale-recovery poison opcode ran -- id_next_pc wrong)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (case 1 target did not run -- gather/redirect broken)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000007 (case 2 misprediction recovery broken -- id_next_pc wrong, wrong-taken poison ran, or fall-through did not)", dbg_d3);
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
