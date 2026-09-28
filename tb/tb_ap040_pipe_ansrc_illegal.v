//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 43: An as a source) //
//                                                                          //
// tb_ap040_pipe_ansrc_illegal.v - the address-register forms that must NOT //
// decode                                                                   //
//                                                                          //
// Reaching source mode 001 is easy to OVERreach. The 68k restricts it and   //
// so must this decoder:                                                    //
//                                                                          //
//   - Byte is never allowed. An address register has no byte operand, and   //
//     the exclusion has to be written twice because the two families        //
//     encode size differently: MOVE puts Byte at ir[13:12]==01, the binary  //
//     ALU family at ir[7:6]==00.                                           //
//   - AND and OR do not take an address register at any size. Only ADD,     //
//     SUB and CMP do.                                                      //
//   - ADDQ.B and SUBQ.B have no address-register destination either        //
//     (milestone 90), which is a third place the same "Byte is never       //
//     allowed" rule has to be written: the quick forms encode size at      //
//     ir[7:6] like the ALU family, but they are a separate shape.          //
//                                                                          //
// A decoder that simply widened the mode field would accept all three of    //
// these and pass tb_ap040_pipe_ansrc.v -- which is why that bench alone is  //
// not enough, and why this one exists rather than a note in a header.       //
//                                                                          //
// Each case runs for real and must take the illegal-instruction exception:  //
//                                                                          //
//   A0 = $AA, D0 = $11, D1 = $55                                           //
//   MOVE.B A0,D0    would leave D0 = 000000AA                              //
//   ADD.B  A0,D1    would leave D1 = 000000FF                              //
//   AND.L  A0,D1    would leave D1 = $AA & whatever D1 then held           //
//   ADDQ.B #1,A0    would leave A0 = 000000AB                              //
//                                                                          //
// Every operand differs from every other, so no wrong decode can land on    //
// the right answer by coincidence. They do CHAIN, though, if more than one  //
// exclusion is missing: a decoder that admits all three leaves D1 = 000000AA//
// ($55 + $AA = $FF, then $FF & $AA), not the 00000000 the AND alone would   //
// give. The trap COUNT is therefore the check that says how many exclusions //
// held; the register values say which wrong decode ran.                     //
//                                                                          //
// That distinction was verified by breaking the decoder on purpose --       //
// widening the mode field and dropping both restrictions -- and confirming  //
// this bench reports 0 traps with D0 = 000000AA and D1 = 000000AA. Without  //
// that check it would be a bench that passes on RTL predating the feature   //
// entirely, which it also does, since every An-source form was illegal      //
// before milestone 43.                                                      //
//                                                                          //
// The handler counts traps into D2 and returns through JMP (A2), the same   //
// shape tb_ap040_pipe_exc.v uses. A2 is reloaded by the mainline before     //
// each case, so one handler serves all three and the count is the proof     //
// that every one of them trapped rather than just the first.               //
//                                                                          //
// Word index N is byte address $400 + 2N, and the vector table sits at      //
// VBR = $2000, so vector 4 lands at word index 3592 -- the same arithmetic  //
// tb_ap040_pipe_exc.v documents.                                           //
//                                                                          //
// On a decoder that admits all four, D2 counts 0.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_ansrc_illegal;

localparam PROG_WORDS      = 84;
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
	// Mainline
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$000000AA,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h00AA;
	dut.u_l1.mem[4]  = 16'h7011;   // MOVEQ #$11,D0
	dut.u_l1.mem[5]  = 16'h7255;   // MOVEQ #$55,D1
	dut.u_l1.mem[6]  = 16'h247C;   // MOVEA.L #$00000420,A2  (resume 1)
	dut.u_l1.mem[7]  = 16'h0000;
	dut.u_l1.mem[8]  = 16'h0420;
	dut.u_l1.mem[9]  = 16'h1008;   // MOVE.B A0,D0   -- Byte with an An source

	// Resume 1 @ word idx 16 (byte $420)
	dut.u_l1.mem[16] = 16'h247C;   // MOVEA.L #$00000440,A2  (resume 2)
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h0440;
	dut.u_l1.mem[19] = 16'hD208;   // ADD.B A0,D1    -- Byte in the ALU family

	// Resume 2 @ word idx 32 (byte $440)
	dut.u_l1.mem[32] = 16'h247C;   // MOVEA.L #$00000460,A2  (resume 3)
	dut.u_l1.mem[33] = 16'h0000;
	dut.u_l1.mem[34] = 16'h0460;
	dut.u_l1.mem[35] = 16'hC288;   // AND.L A0,D1    -- AND never takes an An

	// Resume 3 @ word idx 48 (byte $460): the fourth case.
	dut.u_l1.mem[48] = 16'h247C;   // MOVEA.L #$00000480,A2  (resume 4)
	dut.u_l1.mem[49] = 16'h0000;
	dut.u_l1.mem[50] = 16'h0480;
	dut.u_l1.mem[51] = 16'h5208;   // ADDQ.B #1,A0  -- Byte with an An DESTINATION

	// Resume 4 @ word idx 64 (byte $480): nothing left to do but drain.
	dut.u_l1.mem[64] = 16'h4E71;   // NOP

	// Illegal handler @ word idx 512 (byte $800). Deliberately far past the
	// mainline: word index 48 is followed by NOP fill, and PROG_WORDS is an
	// instruction-ISSUE budget rather than an address bound, so whatever
	// budget remains after the third trap is spent walking NOPs forward. A
	// handler placed just past the mainline gets walked INTO and counts a
	// fourth, spurious trap -- so the distance here is load-bearing, not
	// cosmetic.
	dut.u_l1.mem[512] = 16'h5282;   // ADDQ.L #1,D2
	dut.u_l1.mem[513] = 16'h4ED2;   // JMP (A2)

	// Vector table: vector 4 -> $800
	dut.u_l1.mem[3592] = 16'h0000;
	dut.u_l1.mem[3593] = 16'h0800;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// Four exception frames are pushed and never popped (the handler
	// returns with JMP, not RTE), so A7 must point somewhere real and
	// clear of the program. See tb_ap040_pipe_move_mem.v's header for why
	// the poke has to land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 160) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d2 !== 32'h0000_0004) begin
		errors = errors + 1;
		$display("FAIL: trap count D2 = %h, expected 00000004 (all four excluded forms must be illegal)",
		         dbg_d2);
	end
	if (dbg_d0 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000011 (MOVE.B A0,D0 must not decode; it would leave 000000aa)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000055 (ADD.B A0,D1 would leave 000000ff, AND.L A0,D1 would leave 0)",
		         dbg_d1);
	end

	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_00AA) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 000000aa (ADDQ.B #1,A0 must not decode; it would leave 000000ab)",
		         dut.u_cpu.u_regfile.areg[0]);
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
