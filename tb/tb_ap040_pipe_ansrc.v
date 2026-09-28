//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 43: An as a source) //
//                                                                          //
// tb_ap040_pipe_ansrc.v - reading an address register as an operand        //
//                                                                          //
// Milestone 42's bench found this hole rather than a bug. is_move_rr        //
// requires source mode 000, so until now NO instruction in this decoder     //
// could read An as a source operand at all: MOVE.L A0,D0 was illegal, and   //
// so was every ADD/SUB/CMP form that takes one.                            //
//                                                                          //
// The fix is pure decode. An lives in the same 4-bit unified register       //
// space this stage already uses (8+n), so naming it in id_src_reg is the    //
// entire change -- no new port, no datapath, nothing threaded anywhere.     //
//                                                                          //
//   MOVEA.L #$1234,A0                                                      //
//   MOVE.L  A0,D0     D0 = 00001234                                        //
//   MOVEQ   #$10,D1                                                        //
//   ADD.L   A0,D1     D1 = 00001244                                        //
//   MOVE.L  #$FFFF0005,D2                                                  //
//   MOVE.W  A0,D2     D2 = FFFF1234 -- low word only                       //
//   CMP.L   A0,D0     sets Z, writes nothing                               //
//                                                                          //
// MOVE.W is the size check: the high half of D2 is preloaded with FFFF      //
// precisely so that a Word move writing all 32 bits is visible. A0 is       //
// $00001234, so a full-width write would leave 00001234 and a correct one   //
// leaves FFFF1234 -- values that differ only where the bug would be.        //
//                                                                          //
// CMP.L A0,D0 compares an address register against the data register that   //
// was loaded FROM it, so Z is set only if both readings of A0 agree, and    //
// D0 surviving as 00001234 is the usual check that CMP writes nothing.      //
//                                                                          //
// The forms this must NOT decode are checked separately, in                 //
// tb_ap040_pipe_ansrc_illegal.v: Byte is never allowed with an address      //
// register, and AND and OR do not take one at any size.                     //
//                                                                          //
// On milestone 42's RTL none of the four An-source forms decode.           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_ansrc;

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
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00001234,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h1234;
	dut.u_l1.mem[4]  = 16'h2008;   // MOVE.L A0,D0
	dut.u_l1.mem[5]  = 16'h7210;   // MOVEQ #$10,D1
	dut.u_l1.mem[6]  = 16'hD288;   // ADD.L A0,D1
	dut.u_l1.mem[7]  = 16'h243C;   // MOVE.L #$FFFF0005,D2
	dut.u_l1.mem[8]  = 16'hFFFF;
	dut.u_l1.mem[9]  = 16'h0005;
	dut.u_l1.mem[10] = 16'h3408;   // MOVE.W A0,D2
	dut.u_l1.mem[11] = 16'hB088;   // CMP.L A0,D0
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_1234) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00001234 (MOVE.L A0,D0; CMP must not overwrite it)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_1244) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00001244 (ADD.L A0,D1)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_1234) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected ffff1234 (MOVE.W A0,D2 must write the low word only)", dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. CMP.L of equal operands sets Z alone; the
	// reversed subtraction would give the same Z here, but N and C would
	// differ on unequal operands, so Z is checked together with the fact
	// that D2 survived.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMP.L of equal operands sets Z)", dbg_ccr[3:0]);
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
