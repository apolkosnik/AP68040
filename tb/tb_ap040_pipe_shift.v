//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 23: shifts and      //
// rotates, immediate count)                                                //
//                                                                          //
// tb_ap040_pipe_shift.v - the count reaches the barrel                     //
//                                                                          //
// ap040_pipe_alu.v's barrel has taken a 1..63 count since it was forked,   //
// composing the one-bit steps in a single cycle. ap040_execute.v hardwired //
// shcnt to 1, so even if a shift had decoded it would have moved one bit.  //
// This milestone plumbs a real count field ID -> EA-calc -> EA-fetch -> EX //
// alongside milestone 18's size, and opens all eight operations from one   //
// predicate: ir[4:3] picks the family and ir[8] the direction.             //
//                                                                          //
// Program:                                                                 //
//                                                                          //
//   1: MOVEQ #1,D0      7001   D0 = 00000001                               //
//   2: LSL.L  #4,D0     E988   D0 = 00000010   count 4, not 1              //
//   3: MOVEQ #-1,D1     72FF   D1 = FFFFFFFF                               //
//   4: LSR.W  #8,D1     E049   D1 = FFFF00FF   count field 0 means EIGHT   //
//   5: MOVEQ #1,D2      7401   D2 = 00000001                               //
//   6: ROR.L  #1,D2     E29A   D2 = 80000000   rotate, not shift           //
//                                                                          //
// Each line is aimed at a specific way this could be wrong. LSL.L #4        //
// giving 10 rather than 02 is the whole point of the count field -- with   //
// shcnt still hardwired to 1 the answer is 02. LSR.W #8 checks the count   //
// ENCODING, where a literal 0 in ir[11:9] means eight: read literally the  //
// shift would be by zero and D1 would stay FFFFFFFF. It is also sized      //
// Word, so D1[31:16] must survive as FFFF while the low half becomes 00FF, //
// which a Long-sized decode would turn into 00FFFFFF. ROR.L #1 separates   //
// the rotate families from the shift families: a logical shift of 1 right  //
// gives 00000000, a rotate wraps the bit to the top.                       //
//                                                                          //
// On milestone 22's RTL none of the three decodes.                         //
//                                                                          //
// The opcodes here were COMPUTED from the field layout, not assembled by   //
// hand. Two hand-written ones were wrong on the first run -- a count field //
// of 6 where 8 was meant, and a destination of D0 where D2 was meant -- and //
// both produced plausible-looking wrong answers that read as RTL faults.   //
// The second was the more instructive: ROR.L #1,D0 rotated the LSL result  //
// from 00000010 to 00000008, so the D0 check failed for a reason that had  //
// nothing to do with D0's own instruction.                                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_shift;

localparam PROG_WORDS      = 14;
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
	dut.u_l1.mem[1] = 16'h7001;   // MOVEQ #1,D0
	dut.u_l1.mem[2] = 16'hE988;   // LSL.L  #4,D0
	dut.u_l1.mem[3] = 16'h72FF;   // MOVEQ #-1,D1
	dut.u_l1.mem[4] = 16'hE049;   // LSR.W  #8,D1   (count field 0 = 8)
	dut.u_l1.mem[5] = 16'h7401;   // MOVEQ #1,D2
	dut.u_l1.mem[6] = 16'hE29A;   // ROR.L  #1,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0010) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000010 (LSL.L #4; 00000002 means shcnt is still 1)", dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_00FF) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected FFFF00FF (LSR.W #8: count 0 encodes 8, and the word splice keeps D1[31:16])", dbg_d1);
	end
	if (dbg_d2 !== 32'h8000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 80000000 (ROR.L #1 must wrap, not shift out)", dbg_d2);
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
