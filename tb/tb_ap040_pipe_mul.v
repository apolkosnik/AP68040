//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 51: MULU/MULS)      //
//                                                                          //
// tb_ap040_pipe_mul.v - 16x16 -> 32 multiply, signed and unsigned          //
//                                                                          //
// Nibble 1100 with ir[7:6]==11 is the slot every ALU shape so far has been //
// excluding: it is AND's opmode 011/111, which the 68k gives to multiply   //
// rather than to a second AND direction.                                   //
//                                                                          //
// A 16x16 multiply is one DSP block on Cyclone V and comfortably inside a  //
// 40 MHz cycle, so unlike divide this needs no sequencer -- it is decode   //
// plus two ALU cases. The 32-bit product goes to the WHOLE of Dn, so       //
// id_size is Long while the source is a word, and id_sxt_w forces the      //
// memory read and the autoincrement step to Word exactly as ADDA.W does.   //
//                                                                          //
// Memory: $0480 = 00039999.                                                //
//                                                                          //
//   MOVEA.L #$0480,A0 / MOVE.L #2,D1                                       //
//   MULU.W (A0),D1     D1 = 2 * 0003 = 6                                   //
//   MOVE.L  #$0000FFFF,D0                                                  //
//   MULU.W D1,D0       D0 = 0005FFFA                                       //
//   MOVE.L  #$0000FFFF,D2                                                  //
//   MULS.W D1,D2       D2 = FFFFFFFA                                       //
//                                                                          //
// D0 and D2 are the point. Identical source bits ($0000FFFF) and the same  //
// multiplier (6), and the two instructions must disagree: 0005FFFA read    //
// unsigned, FFFFFFFA read signed. Neither value is reachable from the      //
// other by any masking, so one pair of checks settles both opcodes.        //
//                                                                          //
// D0 also proves the product is not truncated to the operand size. A       //
// 16-bit splice would leave 0000FFFA, which is why the multiplier is 6 and //
// not 1 -- the product has to overflow 16 bits for that to be visible.     //
//                                                                          //
// D1 proves the memory read took the HIGH half-word, the one the address   //
// names: reading the low half instead gives 2 * $9999 = 00013332.          //
//                                                                          //
// N is checked from MULS's negative result, which says the flags come from //
// bit 31 of the 32-bit product rather than from bit 15 through res_msb.    //
// V and C must be clear -- a multiply never sets them on this family --    //
// and X is not checked here because no instruction in this bench reads it. //
//                                                                          //
// On milestone 50's RTL none of the three multiplies decode.               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_mul;

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
	dut.u_l1.mem[4]  = 16'h223C;   // MOVE.L #$00000002,D1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0002;
	dut.u_l1.mem[7]  = 16'hC2D0;   // MULU.W (A0),D1
	dut.u_l1.mem[8]  = 16'h203C;   // MOVE.L #$0000FFFF,D0
	dut.u_l1.mem[9]  = 16'h0000;
	dut.u_l1.mem[10] = 16'hFFFF;
	dut.u_l1.mem[11] = 16'hC0C1;   // MULU.W D1,D0
	dut.u_l1.mem[12] = 16'h243C;   // MOVE.L #$0000FFFF,D2
	dut.u_l1.mem[13] = 16'h0000;
	dut.u_l1.mem[14] = 16'hFFFF;
	dut.u_l1.mem[15] = 16'hC5C1;   // MULS.W D1,D2

	dut.u_l1.mem[64] = 16'h0003;   // $0480 = 00039999
	dut.u_l1.mem[65] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0005_FFFA) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0005fffa (MULU $ffff*6; 0000fffa means the product was truncated to 16 bits)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0006) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000006 (MULU.W (A0) must read the HIGH half-word; the low one gives 00013332)",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_FFFA) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected fffffffa (MULS on the SAME bits D0 multiplied unsigned)", dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. MULS left a negative 32-bit product, so N
	// must come from bit 31 -- read through res_msb at Word size it would be
	// bit 15 of fffffffa, which is also 1, so the value alone would not
	// settle it; what does is that the product is Long-sized at all, which
	// D0's 0005fffa establishes. V and C are never set by a multiply.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (MULS product is negative; V and C are never set)",
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
