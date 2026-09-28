//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 56: (d8,An,Xn))           //
//                                                                          //
// tb_ap040_pipe_index.v - indexed addressing, the array access             //
//                                                                          //
// MOVE.L (0,A0,D1.L*4),D2 is what a compiler emits for a[i], and mode 110  //
// was the largest addressing-mode gap left.                                //
//                                                                          //
// It gathers ONE extension word like (d16,An), so it rides the same two    //
// gather kinds rather than adding more. What differs is what that word     //
// MEANS -- the brief format, not a displacement -- so for this mode id_imm //
// carries the word VERBATIM and ap040_ea_fetch.v unpacks it. Packing it    //
// that way is why no new per-stage fields were needed: the extension word  //
// already IS the packed form.                                              //
//                                                                          //
// The index register needed a THIRD register read port. An is on port A    //
// and the destination operand is on port B for everything except a plain   //
// load, so two ports genuinely do not reach. It forwards like the other    //
// two, which the third case below depends on.                              //
//                                                                          //
// Array at $0480: 11111111, 22222222, 33333333.                            //
//                                                                          //
//   MOVEA.L #$0480,A0 / MOVE.L #2,D1                                       //
//   MOVE.L (0,A0,D1.L*4),D2    -> $0488, D2 = 33333333                     //
//   MOVE.L #$0000FFFF,D1                                                   //
//   MOVE.L (8,A0,D1.W*4),D0    -> $0484, D0 = 22222222                     //
//   LEA    (8,A0,D1.W*4),A2    -> A2 = 00000484, the SAME EA made visible  //
//   MOVEA.L #4,A1                                                          //
//   ADD.L  (0,A0,A1.L),D1      -> $0484, D1 = 22232221                     //
//                                                                          //
// Each case isolates one field of the brief word:                          //
//                                                                          //
//   D2 is the SCALE. Index 2 unscaled addresses $0482, which is inside the //
//     first element and straddles two, reading 11112222 -- a plausible     //
//     value, not a crash, which is why the array entries are distinct.     //
//   D0 is the Word index's SIGN EXTENSION and the BYTE displacement. The   //
//     index is -1, so the address goes four bytes BACKWARD and eight       //
//     forward; zero-extending instead lands $3FFFC away. The displacement  //
//     being 8 also checks it is read as a byte and not as the 16-bit       //
//     field every other mode uses -- the high half of this word is the     //
//     index specification, so a 16-bit read would fold that in.            //
//   D1 is the D/A bit: the index is an ADDRESS register here, and it also  //
//     shows the mode works as an ALU source, not only as a MOVE.           //
//   A2 is the Word index's SIGN EXTENSION, and it has to be an LEA. A      //
//     LOAD cannot see the difference: this L1 wraps mod 8192 and           //
//     65536*scale is always a multiple of 8192, so a zero-extended index   //
//     lands on the SAME word as a sign-extended one. That was found by     //
//     mutating the extension to zero-fill and watching the bench still     //
//     pass. LEA puts the full 32-bit address in An, where $00000484 and    //
//     $00040484 are plainly different.                                     //
//                                                                          //
// D1 is loaded from memory and then used as an index two instructions      //
// later, so port C's forwarding is exercised rather than assumed.          //
//                                                                          //
// On milestone 55's RTL none of the three decode.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_index;

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
	dut.u_l1.mem[7]  = 16'h2430;   // MOVE.L (0,A0,D1.L*4),D2
	dut.u_l1.mem[8]  = 16'h1C00;
	dut.u_l1.mem[9]  = 16'h223C;   // MOVE.L #$0000FFFF,D1
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'hFFFF;
	dut.u_l1.mem[12] = 16'h2030;   // MOVE.L (8,A0,D1.W*4),D0
	dut.u_l1.mem[13] = 16'h1408;
	dut.u_l1.mem[14] = 16'h45F0;   // LEA (8,A0,D1.W*4),A2   -- same EA, observable
	dut.u_l1.mem[15] = 16'h1408;
	dut.u_l1.mem[16] = 16'h227C;   // MOVEA.L #$00000004,A1
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h0004;
	dut.u_l1.mem[19] = 16'hD2B0;   // ADD.L (0,A0,A1.L),D1
	dut.u_l1.mem[20] = 16'h9800;

	dut.u_l1.mem[64] = 16'h1111;   // $0480 = 11111111
	dut.u_l1.mem[65] = 16'h1111;
	dut.u_l1.mem[66] = 16'h2222;   // $0484 = 22222222
	dut.u_l1.mem[67] = 16'h2222;
	dut.u_l1.mem[68] = 16'h3333;   // $0488 = 33333333
	dut.u_l1.mem[69] = 16'h3333;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 22222222 (Word index -1 must SIGN-extend; the byte displacement is 8)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h2223_2221) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 22232221 (an ADDRESS register as the index, on an ALU source)",
		         dbg_d1);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_0484) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00000484 (LEA (8,A0,D1.W*4); 00040484 means the Word index was zero-extended)",
		         dut.u_cpu.u_regfile.areg[2]);
	end
	if (dbg_d2 !== 32'h3333_3333) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 33333333 (scale 4; unscaled reads $0482 and gives 11112222)", dbg_d2);
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
