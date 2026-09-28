//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 52: DIVU/DIVS)      //
//                                                                          //
// tb_ap040_pipe_div.v - 32/16 divide, signed and unsigned, and overflow    //
//                                                                          //
// OR's opmode 011/111 -- the mirror of multiply's slot in nibble 1100.     //
//                                                                          //
// Unlike multiply this cannot be combinational, so it is an iterative      //
// restoring divider in ap040_execute.v that holds the pipeline. It reuses  //
// milestone 48's local-stall machinery wholesale, including the            //
// output-register gate that milestone moved from stall_in to ex_stall --   //
// which was UNREACHABLE then and is exercised on every divide now. Without //
// it the instruction would retire while the divider was still running.     //
//                                                                          //
// Thirty-two steps, not sixteen: a 32-bit quotient is computed and then    //
// checked for fitting in 16 bits, which is the same test as the 68k's      //
// "upper word of the dividend >= divisor" precondition without having to   //
// be reasoned about separately.                                            //
//                                                                          //
// Memory: $0480 = 00012222.                                                //
//                                                                          //
//   MOVEA.L #$0480,A0                                                      //
//   MOVE.L  #100,D0 / #7,D1                                                //
//   DIVU.W  D1,D0      D0 = 0002000E   (q 14 low, r 2 high)                //
//   MOVE.L  #-100,D2                                                       //
//   DIVS.W  D1,D2      D2 = FFFEFFF2   (q -14, r -2)                       //
//   MOVE.L  #$00100000,D1                                                  //
//   DIVU.W  (A0),D1    quotient needs 21 bits -> V, D1 UNCHANGED           //
//                                                                          //
// The three cases are each aimed at something different:                   //
//                                                                          //
//   D0 fixes the result LAYOUT -- quotient low, remainder high. Swapped it //
//     would read 000E0002, and 100/7 is chosen so the two halves differ.   //
//   D2 fixes the SIGN RULES, which are not symmetric: the quotient takes   //
//     the XOR of the operand signs, but the remainder takes the DIVIDEND's //
//     sign, not the divisor's and not the quotient's. Here both come out   //
//     negative, and a remainder signed from the divisor would give 0002.   //
//   D1 fixes OVERFLOW, the first case in this core where an instruction    //
//     completes, sets a flag and writes NOTHING. It is checked by the      //
//     destination still holding 00100000 -- a divider that wrote a         //
//     truncated quotient would leave 00000000 there.                       //
//                                                                          //
// The overflowing divide is LAST so V can be read at the end. V is set by  //
// nothing else in this program, so 0010 is unambiguous.                    //
//                                                                          //
// Division by zero is an exception and cannot be checked in the same       //
// program; it has its own bench, tb_ap040_pipe_divzero.v.                  //
//                                                                          //
// On milestone 51's RTL none of the three divides decode.                  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_div;

localparam PROG_WORDS      = 32;
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
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$00000064,D0   (100)
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0064;
	dut.u_l1.mem[7]  = 16'h223C;   // MOVE.L #$00000007,D1
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h0007;
	dut.u_l1.mem[10] = 16'h80C1;   // DIVU.W D1,D0
	dut.u_l1.mem[11] = 16'h243C;   // MOVE.L #$FFFFFF9C,D2   (-100)
	dut.u_l1.mem[12] = 16'hFFFF;
	dut.u_l1.mem[13] = 16'hFF9C;
	dut.u_l1.mem[14] = 16'h85C1;   // DIVS.W D1,D2
	dut.u_l1.mem[15] = 16'h223C;   // MOVE.L #$00100000,D1
	dut.u_l1.mem[16] = 16'h0010;
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h82D0;   // DIVU.W (A0),D1  -- overflows

	dut.u_l1.mem[64] = 16'h0001;   // $0480 = 00012222
	dut.u_l1.mem[65] = 16'h2222;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 320) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0002_000E) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0002000e (100/7: quotient LOW, remainder HIGH; 000e0002 means they are swapped)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h0010_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00100000 (an overflowing divide must write NOTHING)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFE_FFF2) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected fffefff2 (-100/7: q -14, and the remainder takes the DIVIDEND's sign)",
		         dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. The last divide overflowed, so V alone is
	// set. Nothing else in this program sets V, which makes 0010 unambiguous.
	if (dbg_ccr[3:0] !== 4'b0010) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0010 (an overflowing divide sets V and nothing else)",
		         dbg_ccr[3:0]);
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
