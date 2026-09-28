//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 47: EOR Dn,Dm)      //
//                                                                          //
// tb_ap040_pipe_eor.v - the one ir[8]=1 form with a register destination   //
//                                                                          //
// ap040_decode.v has carried a comment naming this hole since the binary   //
// family was first written: ir[8]=1 is the Dn -> <ea> direction, and for   //
// nibble 1011 that is EOR rather than a second CMP. ap040_pipe_alu.v has   //
// implemented AP040_ALU_EOR since the fork. Only decode was missing.       //
//                                                                          //
// The operand roles are REVERSED from the ir[8]=0 family: ir[11:9] is the  //
// SOURCE and ir[2:0] the DESTINATION, the same arrangement bitop_shape     //
// uses. EOR's result is symmetric, so getting that backwards produces the  //
// RIGHT VALUE IN THE WRONG REGISTER -- invisible in any check that only    //
// looks at the destination. D0 is therefore checked as carefully as D1.    //
//                                                                          //
//   MOVE.L #$FF00FF00,D0                                                   //
//   MOVE.L #$0F0F0F0F,D1                                                   //
//   EOR.L  D0,D1     D1 = F00FF00F, D0 UNCHANGED                           //
//   MOVE.L #$12345678,D2                                                   //
//   EOR.W  D0,D2     D2 = 1234A978 -- low word only                        //
//                                                                          //
// The operands are chosen so that a swap is visible in EVERY register. A   //
// reversed decoder sends both EORs into D0 -- the EOR.L leaving F00FF00F   //
// and the EOR.W then xoring D2's low word into it for F00FA677 -- while    //
// D1 and D2 keep the values their MOVEs put there. Verified by swapping    //
// the two register fields on purpose and confirming exactly that.          //
//                                                                          //
// EOR.W checks the size splice and leaves N set, since $A978 is negative   //
// as a word. A Long EOR would give 1234A978 too, so N is what separates    //
// them: as a longword that result is positive, and the flags come from the //
// sized result.                                                            //
//                                                                          //
// Mode 001 is deliberately not reached -- 1011 RRR 1 SS 001 rrr is CMPM.   //
//                                                                          //
// On milestone 46's RTL neither EOR decodes.                               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_eor;

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
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$FF00FF00,D0
	dut.u_l1.mem[2]  = 16'hFF00;
	dut.u_l1.mem[3]  = 16'hFF00;
	dut.u_l1.mem[4]  = 16'h223C;   // MOVE.L #$0F0F0F0F,D1
	dut.u_l1.mem[5]  = 16'h0F0F;
	dut.u_l1.mem[6]  = 16'h0F0F;
	dut.u_l1.mem[7]  = 16'hB181;   // EOR.L D0,D1
	dut.u_l1.mem[8]  = 16'h243C;   // MOVE.L #$12345678,D2
	dut.u_l1.mem[9]  = 16'h1234;
	dut.u_l1.mem[10] = 16'h5678;
	dut.u_l1.mem[11] = 16'hB142;   // EOR.W D0,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hFF00_FF00) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected ff00ff00 (D0 is the SOURCE; f00fa677 means both EORs were written here)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'hF00F_F00F) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected f00ff00f (EOR.L D0,D1; 0f0f0f0f means it was written to D0 instead)",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'h1234_A978) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 1234a978 (EOR.W must touch the low word only)", dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. $A978 is negative as a WORD and positive as
	// a longword, so N is what says the flags came from the sized result --
	// the value in D2 alone cannot tell a Word EOR from a Long one here.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (EOR.W result a978 is negative as a word)", dbg_ccr[3:0]);
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
