//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 73: MOVEM $xxx.L)         //
//                                                                          //
// tb_ap040_pipe_movemabsl.v - the three-word gather                        //
//                                                                          //
// The last MOVEM mode, and the first instruction in this decoder with      //
// THREE extension words: mask, then a 32-bit address. The shared gather    //
// topped out at two, but the shift register it already keeps does the      //
// work -- after the mask and the high word, disp_acc is {mask, addr_hi}    //
// and the completing word is addr_lo, so gather_disp is the whole address  //
// and the mask is disp_acc[31:16]. Hence the mask now has its own field,   //
// id_movem_mask, for every MOVEM mode, replacing milestone 68's packing.   //
//                                                                          //
//   MOVE.L #$11111111,D0 / #$22222222,D1                                   //
//   MOVEM.L D0-D1,$00000500.L        store, three words                   //
//   MOVEQ  #$2A,D2                                                         //
//   clobber D0, D1                                                         //
//   MOVEM.L $00000500.L,D0-D1        load, three words                    //
//                                                                          //
// CORRECTION. The first version of this header claimed the MOVEQ #$2A,D2  //
// after the store checked the three-word length through next_pc. It does   //
// not, and a mutation said so: with the xlong term of id_next_pc broken    //
// the bench still passed. A plain MOVEM's id_next_pc is never CONSUMED --  //
// IF fetches linearly and the gather stalls it for exactly ext_pending      //
// cycles, so fetch lands after the third word whatever next_pc says, and   //
// only an exception or a return would read it. The held_is_xlong term is   //
// correct and, today, unobservable. D2 = 2A is kept only as a "the         //
// sequence continued" sanity check and proves nothing about the gather.    //
//                                                                          //
// What DOES guard the three-word gather is the ADDRESS and the MASK, and   //
// both were verified by mutation rather than argued:                        //
//   - gathering two words instead of three assembles {mask, addr_hi} as    //
//     the address, the block lands out of sight, and the memory and        //
//     round-trip checks all fail;                                          //
//   - reading the mask from disp_acc[15:0] instead of [31:16] takes        //
//     addr_hi (zero) as the mask, nothing is transferred, and the same     //
//     checks fail.                                                         //
// Under both mutations D2 was still 2A.                                    //
//                                                                          //
// On milestone 72's RTL neither form decodes.                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movemabsl;

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
	dut.u_l1.mem[1  ] = 16'h203C;   // MOVE.L #$11111111,D0
	dut.u_l1.mem[2  ] = 16'h1111;
	dut.u_l1.mem[3  ] = 16'h1111;
	dut.u_l1.mem[4  ] = 16'h223C;   // MOVE.L #$22222222,D1
	dut.u_l1.mem[5  ] = 16'h2222;
	dut.u_l1.mem[6  ] = 16'h2222;
	dut.u_l1.mem[7  ] = 16'h48F9;   // MOVEM.L D0-D1,$00000500.L   (3 extension words)
	dut.u_l1.mem[8  ] = 16'h0003;
	dut.u_l1.mem[9  ] = 16'h0000;
	dut.u_l1.mem[10 ] = 16'h0500;
	dut.u_l1.mem[11 ] = 16'h742A;   // MOVEQ #$2A,D2   -- proves next_pc stepped over all THREE words
	dut.u_l1.mem[12 ] = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[13 ] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[14 ] = 16'h4CF9;   // MOVEM.L $00000500.L,D0-D1   (3 extension words)
	dut.u_l1.mem[15 ] = 16'h0003;
	dut.u_l1.mem[16 ] = 16'h0000;
	dut.u_l1.mem[17 ] = 16'h0500;
	dut.u_l1.mem[18 ] = 16'h4E71;   // NOP

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

	// $0500 is word index 128, $0504 is 130, $0508 is 132.
	if (dbg_d2 !== 32'h0000_002A) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 0000002a (the instruction after a 3-word MOVEM must run; next_pc must skip all three)",
		         dbg_d2);
	end
	if ({dut.u_l1.mem[128], dut.u_l1.mem[129]} !== 32'h1111_1111) begin
		errors = errors + 1;
		$display("FAIL: [$0500] = %h%h, expected 11111111 (store to $xxx.L)", dut.u_l1.mem[128], dut.u_l1.mem[129]);
	end
	if ({dut.u_l1.mem[130], dut.u_l1.mem[131]} !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: [$0504] = %h%h, expected 22222222", dut.u_l1.mem[130], dut.u_l1.mem[131]);
	end
	if (dbg_d0 !== 32'h1111_1111 || dbg_d1 !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: D0/D1 = %h/%h, expected 11111111/22222222 (load from $xxx.L after a clobber)", dbg_d0, dbg_d1);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (a control mode writes no address register)", dut.u_cpu.u_regfile.isp);
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
