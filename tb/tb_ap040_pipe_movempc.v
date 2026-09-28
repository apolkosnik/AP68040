//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 72: MOVEM (d16,PC), $xxx.W)//
//                                                                          //
// tb_ap040_pipe_movempc.v - a constant table, and a fixed save area        //
//                                                                          //
// Both are control modes and both gather a second word after the mask, so  //
// they ride milestone 68's two-word MOVEM gather with two more properties. //
// $xxx.L would need a THIRD word and is not reached.                        //
//                                                                          //
//   MOVEM.L (d16,PC),D0-D1     D0 = 11111111, D1 = 22222222 from $0900     //
//   MOVEM.L D0-D1,$0500.W      store to a fixed address                    //
//   clobber D0, D1                                                         //
//   MOVEM.L $0500.W,D0-D1      load them back                              //
//                                                                          //
// The PC-relative base is the thing to test. For every other PC-relative   //
// mode it is PC+2, the address of the extension word. For MOVEM the mask   //
// sits between the opcode and the displacement, so the displacement word   //
// is at PC+4 and THAT is the base. A core that reused the PC+2 base reads   //
// the table one word early: D0 becomes {NOP fill, 1111} = 4E711111 and D1  //
// becomes 11112222, both unmistakable. The assembler computes the           //
// displacement from opcode+4 to match.                                     //
//                                                                          //
// The absolute pair is checked in memory and by round trip: the store must //
// land at $0500 and $0504 with bit 0 = D0 (a control mode, so not          //
// reversed), and the load must bring both values back after a clobber.     //
// No address register is involved anywhere, so none may change.            //
//                                                                          //
// On milestone 71's RTL none of the three decode.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movempc;

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
	dut.u_l1.mem[1  ] = 16'h4CFA;   // MOVEM.L (d16,PC),D0-D1   base is PC+4 (after the mask)
	dut.u_l1.mem[2  ] = 16'h0003;
	dut.u_l1.mem[3  ] = 16'h04FA;
	dut.u_l1.mem[4  ] = 16'h48F8;   // MOVEM.L D0-D1,$0500.W    (store, absolute short)
	dut.u_l1.mem[5  ] = 16'h0003;
	dut.u_l1.mem[6  ] = 16'h0500;
	dut.u_l1.mem[7  ] = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[8  ] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[9  ] = 16'h4CF8;   // MOVEM.L $0500.W,D0-D1    (load, absolute short)
	dut.u_l1.mem[10 ] = 16'h0003;
	dut.u_l1.mem[11 ] = 16'h0500;
	dut.u_l1.mem[12 ] = 16'h4E71;   // NOP
	dut.u_l1.mem[640] = 16'h1111;   // table $0900: 11111111, 22222222
	dut.u_l1.mem[641] = 16'h1111;
	dut.u_l1.mem[642] = 16'h2222;
	dut.u_l1.mem[643] = 16'h2222;

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
	if (dbg_d0 !== 32'h1111_1111) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11111111 (4e711111 means the PC-relative base was PC+2, one word early)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 22222222 (11112222 means the base was one word early)", dbg_d1);
	end
	// $0500 is word index 128, $0504 is 130.
	if ({dut.u_l1.mem[128], dut.u_l1.mem[129]} !== 32'h1111_1111) begin
		errors = errors + 1;
		$display("FAIL: [$0500] = %h%h, expected 11111111 (MOVEM store to $xxx.W, bit 0 = D0)",
		         dut.u_l1.mem[128], dut.u_l1.mem[129]);
	end
	if ({dut.u_l1.mem[130], dut.u_l1.mem[131]} !== 32'h2222_2222) begin
		errors = errors + 1;
		$display("FAIL: [$0504] = %h%h, expected 22222222", dut.u_l1.mem[130], dut.u_l1.mem[131]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600 || dut.u_cpu.u_regfile.areg[0] !== 32'h0) begin
		errors = errors + 1;
		$display("FAIL: A7/A0 = %h/%h, expected 00000600/0 (control modes touch no address register)",
		         dut.u_cpu.u_regfile.isp, dut.u_cpu.u_regfile.areg[0]);
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
