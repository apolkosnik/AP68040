//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 53: MUL/DIV #imm)         //
//                                                                          //
// tb_ap040_pipe_muldiv_imm.v - multiply and divide by a constant           //
//                                                                          //
// MULU #10,D0 and DIVU #10,D0 are everywhere in compiled code, and they    //
// need no new gather kind: held_is_imm already assembles a Word immediate, //
// already routes it into operand_a through id_src_a_is_imm, and already    //
// writes a data register with condition codes.                             //
//                                                                          //
// Two properties had to be CARRIED rather than inferred, which is the same //
// pattern milestones 40, 45 and 46 each hit and the plan predicted would   //
// recur: this gather's operation had always been an ALU op, and a divide   //
// is not one -- it is a sequencer flag ap040_execute.v keys off. So        //
// held_imm_div/held_imm_divs join held_imm_op, held_imm_ccr and the rest.  //
//                                                                          //
//   MOVE.L #100,D0                                                         //
//   DIVU.W #7,D0        D0 = 0002000E                                      //
//   MOVE.L #6,D1                                                           //
//   MULS.W #$FFFF,D1    D1 = FFFFFFFA                                      //
//   MOVE.L #-100,D2                                                        //
//   DIVS.W #7,D2        D2 = FFFEFFF2                                      //
//                                                                          //
// D1 is the one that matters most here. gather_disp always sign-extends,   //
// so every immediate reaches the ALU sign-extended whether the instruction //
// wanted that or not. For MULS that is correct; for MULU it is harmless    //
// only because the multiplier reads bits [15:0] and nothing above. The     //
// check that this distinction is real rather than assumed is that MULS     //
// #$FFFF must give FFFFFFFA and not 0005FFFA -- if the sign were being     //
// lost somewhere in the gather, the signed form would collapse onto the    //
// unsigned one, which is the same failure milestone 51 checked on the      //
// register path.                                                           //
//                                                                          //
// D0 and D2 check that the DIVIDE flag survives the gather at all: an      //
// immediate divide whose held_imm_div was not carried would arrive as an   //
// ALU op with AP040_ALU_MOVE and quietly move the immediate into Dn --     //
// leaving 00000007 there, not a quotient.                                  //
//                                                                          //
// The final DIVS leaves a negative quotient, so N is set and V is clear:   //
// no overflow, unlike tb_ap040_pipe_div.v's last case.                     //
//                                                                          //
// On milestone 52's RTL none of the immediate forms decode.                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_muldiv_imm;

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
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$00000064,D0   (100)
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0064;
	dut.u_l1.mem[4]  = 16'h80FC;   // DIVU.W #7,D0
	dut.u_l1.mem[5]  = 16'h0007;
	dut.u_l1.mem[6]  = 16'h223C;   // MOVE.L #$00000006,D1
	dut.u_l1.mem[7]  = 16'h0000;
	dut.u_l1.mem[8]  = 16'h0006;
	dut.u_l1.mem[9]  = 16'hC3FC;   // MULS.W #$FFFF,D1
	dut.u_l1.mem[10] = 16'hFFFF;
	dut.u_l1.mem[11] = 16'h243C;   // MOVE.L #$FFFFFF9C,D2   (-100)
	dut.u_l1.mem[12] = 16'hFFFF;
	dut.u_l1.mem[13] = 16'hFF9C;
	dut.u_l1.mem[14] = 16'h85FC;   // DIVS.W #7,D2
	dut.u_l1.mem[15] = 16'h0007;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 320) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0002_000E) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0002000e (DIVU #7; 00000007 means held_imm_div was not carried and it moved the immediate)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_FFFA) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected fffffffa (MULS #$ffff; 0005fffa means the immediate lost its sign)",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFE_FFF2) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected fffefff2 (-100/7: q -14, and the remainder takes the DIVIDEND's sign)",
		         dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. The last divide gives a negative quotient
	// and does NOT overflow, so N alone is set -- the opposite of
	// tb_ap040_pipe_div.v's last case, which is how the two benches together
	// say V tracks overflow rather than the sign.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (negative quotient, no overflow)", dbg_ccr[3:0]);
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
