//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 50: MOVEM.L)        //
//                                                                          //
// tb_ap040_pipe_movem.v - saving and restoring a register set             //
//                                                                          //
// The two autoincrement forms, which are the prologue/epilogue idiom:      //
//                                                                          //
//   MOVEM.L <list>,-(An)   $48E0+n   registers to memory, predecrementing  //
//   MOVEM.L (An)+,<list>   $4CD8+n   memory to registers, postincrementing //
//                                                                          //
// One instruction, up to sixteen memory accesses, so it is a sequencer in  //
// ap040_ea_fetch.v alongside the exception-frame and RTE ones -- and a     //
// THIRD register write port, because one instruction writing sixteen       //
// registers cannot use a writeback path that carries one result per        //
// instruction.                                                             //
//                                                                          //
// The mask is numbered differently in the two directions, and that is what //
// this bench is really about. For the predecrementing store bit 0 is A7    //
// and bit 15 is D0; for the load bit 0 is D0 and bit 15 is A7. Walking     //
// from bit 0 upward is correct for both only because the register index is //
// read off as 15 - bit in one case and bit in the other.                   //
//                                                                          //
//   D0/D1/D2/A0 = 11111111 / 22222222 / 33333333 / 44444444                //
//   MOVEM.L D0-D2/A0,-(A7)    mask E080, A7 $0600 -> $05F0                 //
//   clobber all four with AAAAAAAA / BBBBBBBB / CCCCCCCC / DDDDDDDD        //
//   MOVEM.L (A7)+,D0-D2/A0    mask 0107, A7 $05F0 -> $0600                 //
//                                                                          //
// A round trip alone would NOT catch a mis-numbered mask, because any      //
// self-consistent ordering restores what it saved. So the memory is        //
// checked directly, at both ends of the block:                             //
//                                                                          //
//   $05FC must hold 44444444 -- A0, the LOWEST set bit of the store mask,  //
//     written FIRST and therefore at the HIGHEST address.                  //
//   $05F0 must hold 11111111 -- D0, bit 15, written last and lowest.       //
//                                                                          //
//     Confirmed by mutating the store to number its mask the way the load  //
//     does (register index = bit, not 15 - bit). Six checks then fail, and //
//     the signature is specific: $05FC comes back 00000000, because bit 7  //
//     selects D7 under that numbering instead of A0, and $05F0 comes back  //
//     00000600, because bit 15 selects A7. Not a plausible-looking wrong   //
//     answer but a clearly foreign one, which is what makes it easy to     //
//     recognise if the numbering is ever touched again.                    //
//                                                                          //
// The clobber values are chosen so that a load which restored the right    //
// registers from the wrong addresses would be visible: every register      //
// holds a different value and so does every slot.                          //
//                                                                          //
// A7 returning to $0600 proves the two sequencers moved the address by     //
// the same amount in opposite directions, and it is not vacuous, since the //
// store's four slots are checked to have been written where they were.     //
//                                                                          //
// On milestone 49's RTL neither form decodes.                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movem;

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
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$11111111,D0
	dut.u_l1.mem[2]  = 16'h1111;
	dut.u_l1.mem[3]  = 16'h1111;
	dut.u_l1.mem[4]  = 16'h223C;   // MOVE.L #$22222222,D1
	dut.u_l1.mem[5]  = 16'h2222;
	dut.u_l1.mem[6]  = 16'h2222;
	dut.u_l1.mem[7]  = 16'h243C;   // MOVE.L #$33333333,D2
	dut.u_l1.mem[8]  = 16'h3333;
	dut.u_l1.mem[9]  = 16'h3333;
	dut.u_l1.mem[10] = 16'h207C;   // MOVEA.L #$44444444,A0
	dut.u_l1.mem[11] = 16'h4444;
	dut.u_l1.mem[12] = 16'h4444;
	dut.u_l1.mem[13] = 16'h48E7;   // MOVEM.L D0-D2/A0,-(A7)
	dut.u_l1.mem[14] = 16'hE080;
	dut.u_l1.mem[15] = 16'h203C;   // MOVE.L #$AAAAAAAA,D0
	dut.u_l1.mem[16] = 16'hAAAA;
	dut.u_l1.mem[17] = 16'hAAAA;
	dut.u_l1.mem[18] = 16'h223C;   // MOVE.L #$BBBBBBBB,D1
	dut.u_l1.mem[19] = 16'hBBBB;
	dut.u_l1.mem[20] = 16'hBBBB;
	dut.u_l1.mem[21] = 16'h243C;   // MOVE.L #$CCCCCCCC,D2
	dut.u_l1.mem[22] = 16'hCCCC;
	dut.u_l1.mem[23] = 16'hCCCC;
	dut.u_l1.mem[24] = 16'h207C;   // MOVEA.L #$DDDDDDDD,A0
	dut.u_l1.mem[25] = 16'hDDDD;
	dut.u_l1.mem[26] = 16'hDDDD;
	dut.u_l1.mem[27] = 16'h4CDF;   // MOVEM.L (A7)+,D0-D2/A0
	dut.u_l1.mem[28] = 16'h0107;
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

	// The store's slots, at both ends of the block. $05FC is word index 254,
	// $05F0 is 248.
	if ({dut.u_l1.mem[254], dut.u_l1.mem[255]} !== 32'h4444_4444) begin
		errors = errors + 1;
		$display("FAIL: mem[$05FC] = %h%h, expected 44444444 (A0 is the store mask's LOWEST bit, so it goes highest; 00000000 means the index was read as bit rather than 15-bit)",
		         dut.u_l1.mem[254], dut.u_l1.mem[255]);
	end
	if ({dut.u_l1.mem[248], dut.u_l1.mem[249]} !== 32'h1111_1111) begin
		errors = errors + 1;
		$display("FAIL: mem[$05F0] = %h%h, expected 11111111 (D0 is bit 15, written last and lowest)",
		         dut.u_l1.mem[248], dut.u_l1.mem[249]);
	end

	if (dbg_d0 !== 32'h1111_1111) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11111111 (restored); aaaaaaaa means the load never wrote it", dbg_d0);
	end
	if (dbg_d1 !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 22222222 (restored)", dbg_d1);
	end
	if (dbg_d2 !== 32'h3333_3333) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 33333333 (restored)", dbg_d2);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h4444_4444) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 44444444 (restored)", dut.u_cpu.u_regfile.areg[0]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (the two sequencers must move it equally and oppositely)",
		         dut.u_cpu.u_regfile.isp);
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
