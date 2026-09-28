//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 107: Scc to a       //
// memory destination)                                                      //
//                                                                          //
// tb_ap040_pipe_sccmem.v - the condition byte, stored                      //
//                                                                          //
// Scc existed only as Scc Dn: is_scc_rr tested if_opcode[5:3] == 000 and    //
// every other destination fell through to illegal. That is 2,561,872 rounds //
// of the cputest corpus, 33.4% of everything this core cannot decode and    //
// the single largest gap in it.                                            //
//                                                                          //
// This bench covers all seven legal destination modes. The destination      //
// register matters more than it looks: while this was being built the       //
// corpus failed only A6 and A7 in modes (An) and (An)+ and passed A0-A5, so //
// the register is checked across the range rather than at one convenient    //
// value.                                                                    //
//                                                                          //
//   A0 = $800  A1 = $840  A2 = $810  A3 = $824  A6 = $830  A7 = $900        //
//   MOVEQ #0,D0     sets Z, so SNE is FALSE and SF is always false          //
//                                                                          //
//   ST  (A0)    $800 = $FF                                                 //
//   SF  (A2)+   $810 = $00 and A2 -> $811                                  //
//   ST  -(A3)   A3 -> $823 and $823 = $FF                                  //
//   ST  (A6)    $830 = $FF                                                 //
//   ST  (A7)    $900 = $FF   -- through the stack pointer                  //
//   SNE (A1)    $840 = $00   -- Z is set, so the condition is false         //
//   ST  $20(A0)        $820 = $FF                                          //
//   SF  $50(A0,D0.W)   $850 = $00   -- D0 is zero, so the index is zero     //
//   ST  ($860).W       $860 = $FF                                          //
//   ST  ($870).L       $870 = $FF                                          //
//                                                                          //
// The MOVEQ #$7B,D3 after the last of those is not decoration. (xxx).L is   //
// a THREE-word instruction, and the count of extension words a mode gathers //
// lives in two places in this decoder -- held_is_long and ext_pending.      //
// Adding Scc to one and not the other is milestone 89's bug exactly, and it //
// happened again here: the second half of the address was executed as the   //
// next instruction. D3 is what notices, because with the gather short by a  //
// word the MOVEQ never runs.                                               //
//                                                                          //
// Every target byte is poisoned to $5A first, which is neither $00 nor $FF, //
// so "the store never happened" cannot be mistaken for "the store wrote     //
// false". SF and SNE are what make the $00 cases meaningful: a core that    //
// stored $FF unconditionally would pass every ST check and fail these.      //
//                                                                          //
// The byte NEXT to each target is checked too, and must still be $5A. Scc   //
// writes one byte; the RMW store path it rides carries a size, and a size   //
// that came through as Word or Long would be invisible to the value checks  //
// alone while corrupting the neighbour.                                     //
//                                                                          //
// The last instruction is ST with destination mode 7 register 5, which is   //
// a RESERVED encoding and must stay an illegal instruction. That is what    //
// ea_not_alt is actually for here: a mutation dropping it left DBcc and     //
// TRAPcc working anyway, because their own decode wires win the ternary     //
// chains, so the class predicate earns its place only on the reserved       //
// mode-7 values. D4 is set by the vector-4 handler and is how this bench    //
// knows the encoding was refused rather than quietly stored somewhere.      //
//                                                                          //
// A2 and A3 prove the autoincrement and autodecrement happened, and by how  //
// much: one byte, not the two a Word access would step -- except that A7 is //
// the register where a byte access steps by TWO on a real 68040, which is   //
// why the A7 case here is (A7) and not (A7)+.                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_sccmem;

localparam PROG_WORDS      = 200;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
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
wire [31:0] dbg_d0, dbg_d3, dbg_d4, dbg_d5;
wire [15:0] dbg_sr;
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

	.dbg_d0 (dbg_d0), .dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5),
	.dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;

task chk_byte;
	input [255:0] what;
	input integer widx;
	input         high;     // 1 = even address (high byte), 0 = odd
	input   [7:0] want;
	reg     [7:0] got;
	begin
		got = high ? dut.u_l1.mem[widx][15:8] : dut.u_l1.mem[widx][7:0];
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %02x, expected %02x (5a means the store never happened)",
			         what, got, want);
		end
	end
endtask

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000900,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0900;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP    (A7 = $900)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h207C;   // MOVEA.L #$00000800,A0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h0800;
	dut.u_l1.mem[ 9] = 16'h227C;   // MOVEA.L #$00000840,A1
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h0840;
	dut.u_l1.mem[12] = 16'h247C;   // MOVEA.L #$00000810,A2
	dut.u_l1.mem[13] = 16'h0000;
	dut.u_l1.mem[14] = 16'h0810;
	dut.u_l1.mem[15] = 16'h267C;   // MOVEA.L #$00000824,A3
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h0824;
	dut.u_l1.mem[18] = 16'h2C7C;   // MOVEA.L #$00000830,A6
	dut.u_l1.mem[19] = 16'h0000;
	dut.u_l1.mem[20] = 16'h0830;
	dut.u_l1.mem[21] = 16'h7000;   // MOVEQ #0,D0   -> Z set, N clear
	dut.u_l1.mem[22] = 16'h50D0;   // ST  (A0)
	dut.u_l1.mem[23] = 16'h51DA;   // SF  (A2)+
	dut.u_l1.mem[24] = 16'h50E3;   // ST  -(A3)
	dut.u_l1.mem[25] = 16'h50D6;   // ST  (A6)
	dut.u_l1.mem[26] = 16'h50D7;   // ST  (A7)
	dut.u_l1.mem[27] = 16'h56D1;   // SNE (A1)   -> Z set, condition false
	dut.u_l1.mem[28] = 16'h50E8;   // ST  $0020(A0)        -> $820
	dut.u_l1.mem[29] = 16'h0020;
	dut.u_l1.mem[30] = 16'h51F0;   // SF  $50(A0,D0.W)     -> $850
	dut.u_l1.mem[31] = 16'h0050;
	dut.u_l1.mem[32] = 16'h50F8;   // ST  ($0860).W        -> $860
	dut.u_l1.mem[33] = 16'h0860;
	dut.u_l1.mem[34] = 16'h50F9;   // ST  ($00000870).L    -> $870
	dut.u_l1.mem[35] = 16'h0000;
	dut.u_l1.mem[36] = 16'h0870;
	dut.u_l1.mem[37] = 16'h2A0F;   // MOVE.L A7,D5  -- A7 before the trap below
	dut.u_l1.mem[38] = 16'h767B;   // MOVEQ #$7B,D3  -- the stream resumed
	dut.u_l1.mem[39] = 16'h50FD;   // ST <mode 7/5>  -- reserved, must be ILLEGAL
	dut.u_l1.mem[40] = 16'h4E71;   // NOP
	dut.u_l1.mem[41] = 16'h4E71;   // NOP
	dut.u_l1.mem[42] = 16'h4E71;   // NOP
	dut.u_l1.mem[43] = 16'h60FE;   // BRA.B -2  (reached only if it was not)

	// Illegal-instruction handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h782C;  // MOVEQ #$2C,D4
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2

	// Vector 4 (illegal instruction) -> $700.
	dut.u_l1.mem[3592] = 16'h0000;
	dut.u_l1.mem[3593] = 16'h0700;

	// The six target words, poisoned. $5A is neither a true nor a false Scc
	// byte, so an absent store cannot look like a false one.
	dut.u_l1.mem[512] = 16'h5A5A;  // $800/$801
	dut.u_l1.mem[520] = 16'h5A5A;  // $810/$811
	dut.u_l1.mem[529] = 16'h5A5A;  // $822/$823
	dut.u_l1.mem[536] = 16'h5A5A;  // $830/$831
	dut.u_l1.mem[544] = 16'h5A5A;  // $840/$841
	dut.u_l1.mem[640] = 16'h5A5A;  // $900/$901
	dut.u_l1.mem[528] = 16'h5A5A;  // $820/$821
	dut.u_l1.mem[552] = 16'h5A5A;  // $850/$851
	dut.u_l1.mem[560] = 16'h5A5A;  // $860/$861
	dut.u_l1.mem[568] = 16'h5A5A;  // $870/$871
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 2000) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk_byte("ST  (A0)  -> $800", 512, 1, 8'hFF);
	chk_byte("SF  (A2)+ -> $810", 520, 1, 8'h00);
	chk_byte("ST  -(A3) -> $823", 529, 0, 8'hFF);
	chk_byte("ST  (A6)  -> $830", 536, 1, 8'hFF);
	chk_byte("ST  (A7)  -> $900", 640, 1, 8'hFF);
	chk_byte("SNE (A1)  -> $840", 544, 1, 8'h00);
	chk_byte("ST  $20(A0)      -> $820", 528, 1, 8'hFF);
	chk_byte("SF  $50(A0,D0.W) -> $850", 552, 1, 8'h00);
	chk_byte("ST  ($860).W     -> $860", 560, 1, 8'hFF);
	chk_byte("ST  ($870).L     -> $870", 568, 1, 8'hFF);

	// One byte, not two: the neighbour of every target must be untouched.
	chk_byte("neighbour of $800", 512, 0, 8'h5A);
	chk_byte("neighbour of $810", 520, 0, 8'h5A);
	chk_byte("neighbour of $823", 529, 1, 8'h5A);
	chk_byte("neighbour of $830", 536, 0, 8'h5A);
	chk_byte("neighbour of $900", 640, 0, 8'h5A);
	chk_byte("neighbour of $840", 544, 0, 8'h5A);
	chk_byte("neighbour of $820", 528, 0, 8'h5A);
	chk_byte("neighbour of $850", 552, 0, 8'h5A);
	chk_byte("neighbour of $860", 560, 0, 8'h5A);
	chk_byte("neighbour of $870", 568, 0, 8'h5A);

	if (dbg_d4 !== 32'h0000_002C) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 0000002c (ST with a reserved mode-7 destination must be an illegal instruction)",
		         dbg_d4);
	end
	if (dbg_d3 !== 32'h0000_007B) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 0000007b (the MOVEQ after ST ($870).L must run; a short gather executes the address half instead)",
		         dbg_d3);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_0811) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00000811 ((A2)+ must step one BYTE)",
		         dut.u_cpu.u_regfile.areg[2]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_0823) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 00000823 (-(A3) must step one BYTE before the store)",
		         dut.u_cpu.u_regfile.areg[3]);
	end
	// A7 is captured into D5 by the mainline, BEFORE the deliberate illegal
	// instruction below pushes a frame on it -- reading the live ISP at the
	// end would only prove the handler's frame is eight bytes.
	if (dbg_d5 !== 32'h0000_0900) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000900 (ST (A7) must not disturb the stack pointer)",
		         dbg_d5);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
