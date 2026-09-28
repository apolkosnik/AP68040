//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 67: MOVEM.W)              //
//                                                                          //
// tb_ap040_pipe_movemw.v - the Word form, and its sign extension           //
//                                                                          //
// ir[6] is the SIZE, so widening the shape to admit it is most of the      //
// decode work. The behaviour worth testing is on the LOAD side: MOVEM.W    //
// SIGN-EXTENDS each word into the whole 32-bit register. It does not       //
// preserve the upper half -- it replaces it with the sign, which is what   //
// separates it from a pair of half-width writes.                           //
//                                                                          //
//   MOVE.L #$AAAA8001,D0 / #$00000002,D1                                   //
//   MOVEM.W D0-D1,-(A7)     stores 8001 and 0002 as WORDS                  //
//   clobber D0 and D1                                                      //
//   MOVEM.W (A7)+,D0-D1     D0 = FFFF8001, D1 = 00000002                   //
//                                                                          //
// $8001 is negative as a word, and D0's upper half starts as AAAA, so the  //
// three possible answers are all distinct: FFFF8001 if sign-extended,      //
// 00008001 if zero-extended, AAAA8001 if the upper half were preserved.    //
// D1 is positive, so it also confirms the extension is by SIGN and not     //
// unconditional.                                                           //
//                                                                          //
// A7 returning to $0600 does NOT prove the step was two: two registers at  //
// four bytes each would also balance. What proves it is that the two words //
// land in ADJACENT slots, $05FC and $05FE -- with a Long step they would   //
// be four apart and the word between them untouched.                       //
//                                                                          //
// On milestone 66's RTL neither form decodes: the shape pinned ir[6].      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movemw;

localparam PROG_WORDS      = 40;
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
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$AAAA8001,D0
	dut.u_l1.mem[2]  = 16'hAAAA;
	dut.u_l1.mem[3]  = 16'h8001;
	dut.u_l1.mem[4]  = 16'h223C;   // MOVE.L #$00000002,D1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0002;
	dut.u_l1.mem[7]  = 16'h48A7;   // MOVEM.W D0-D1,-(A7)
	dut.u_l1.mem[8]  = 16'hC000;
	dut.u_l1.mem[9]  = 16'h203C;   // MOVE.L #$11112222,D0
	dut.u_l1.mem[10] = 16'h1111;
	dut.u_l1.mem[11] = 16'h2222;
	dut.u_l1.mem[12] = 16'h223C;   // MOVE.L #$33334444,D1
	dut.u_l1.mem[13] = 16'h3333;
	dut.u_l1.mem[14] = 16'h4444;
	dut.u_l1.mem[15] = 16'h4C9F;   // MOVEM.W (A7)+,D0-D1
	dut.u_l1.mem[16] = 16'h0003;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// A7 is the address bank's register 7, which in supervisor mode is the
	// ISP. See tb_ap040_pipe_move_mem.v's header for why the poke has to
	// land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 220) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $05FC is word index 254, $05FE is 255 -- ADJACENT, which is what says
	// the step was two.
	if (dut.u_l1.mem[254] !== 16'h8001) begin
		errors = errors + 1;
		$display("FAIL: [$05FC] = %h, expected 8001 (D0's low word, stored as a WORD)", dut.u_l1.mem[254]);
	end
	if (dut.u_l1.mem[255] !== 16'h0002) begin
		errors = errors + 1;
		$display("FAIL: [$05FE] = %h, expected 0002 (D1's low word, in the ADJACENT slot)", dut.u_l1.mem[255]);
	end
	if (dbg_d0 !== 32'hFFFF_8001) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected ffff8001 (00008001 = zero-extended, aaaa8001 = upper half kept)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0002) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000002 (positive, so the extension must be by SIGN)", dbg_d1);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600", dut.u_cpu.u_regfile.isp);
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
