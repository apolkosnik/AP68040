//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 71: JMP/JSR target modes) //
//                                                                          //
// tb_ap040_pipe_jmpmodes.v - six ways to get somewhere                     //
//                                                                          //
// JSR (d16,PC) is how position-independent code calls anything nearby,     //
// JSR $xxx.L is the absolute call, and JMP (d8,PC,Xn) is a jump table.     //
// JMP and JSR reached only (An) and (d16,An) before this.                  //
//                                                                          //
// Every EA path they need already existed -- LEA and PEA built them -- so   //
// the indexed and PC-relative forms ride held_is_jmp/held_is_jsr with the  //
// held_ea_* properties, and the absolute forms ride held_is_abs with two    //
// new properties beside held_abs_lea and held_abs_push. Decode only.       //
//                                                                          //
//   JSR (d16,PC)        -> sub1        checkpoint 1                        //
//   JSR $sub2.L                        checkpoint 2                        //
//   JMP (0,PC,D0.L)     D0 = 4, lands two words on, over a poison          //
//   ADDQ.L #1,D2                       checkpoint 3                        //
//   JSR (0,A0,D1.L*4)   A0 = sub3-8, D1 = 2 -> sub3   checkpoint 4         //
//   JMP $done.W                        over a poison                       //
//   done: JMP (d16,PC) -> fin          over a poison                       //
//                                                                          //
// D2 = 4 needs all four calls or landings; a JMP that fell through instead  //
// of jumping hits a MOVEQ #-1,D2 poison, and one that went to the wrong    //
// place either hangs or leaves the count short. Three JSR/RTS pairs must   //
// also return A7 exactly to where it began.                                //
//                                                                          //
// The indexed PC-relative JMP is the one to look at: its base is the       //
// EXTENSION word's address, not the opcode's, and D0 is the index -- so    //
// the landing is base + 4 + 0, two words past the brief word. The         //
// assembler asserts that arithmetic rather than trusting it.               //
//                                                                          //
// On milestone 70's RTL none of the six forms decode.                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_jmpmodes;

localparam PROG_WORDS      = 200;
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
	dut.u_l1.mem[1  ] = 16'h7400;   // MOVEQ #0,D2   checkpoints
	dut.u_l1.mem[2  ] = 16'h4EBA;   // JSR (d16,PC) -> sub1
	dut.u_l1.mem[3  ] = 16'h03FA;
	dut.u_l1.mem[4  ] = 16'h4EB9;   // JSR $sub2.L  (absolute long)
	dut.u_l1.mem[5  ] = 16'h0000;
	dut.u_l1.mem[6  ] = 16'h0808;
	dut.u_l1.mem[7  ] = 16'h7004;   // MOVEQ #4,D0
	dut.u_l1.mem[8  ] = 16'h4EFB;   // JMP (0,PC,D0.L) -> skips one word
	dut.u_l1.mem[9  ] = 16'h0800;
	dut.u_l1.mem[10 ] = 16'h74FF;   // MOVEQ #-1,D2   poison, jumped over
	dut.u_l1.mem[11 ] = 16'h5282;   // ADDQ.L #1,D2  checkpoint 3 (jump-table landing)
	dut.u_l1.mem[12 ] = 16'h207C;   // MOVEA.L #(sub3-8),A0
	dut.u_l1.mem[13 ] = 16'h0000;
	dut.u_l1.mem[14 ] = 16'h0808;
	dut.u_l1.mem[15 ] = 16'h7202;   // MOVEQ #2,D1
	dut.u_l1.mem[16 ] = 16'h4EB0;   // JSR (0,A0,D1.L*4) -> sub3
	dut.u_l1.mem[17 ] = 16'h1C00;
	dut.u_l1.mem[18 ] = 16'h4EF8;   // JMP $done.W  (absolute short)
	dut.u_l1.mem[19 ] = 16'h042A;
	dut.u_l1.mem[20 ] = 16'h74FF;   // MOVEQ #-1,D2   poison, jumped over
	dut.u_l1.mem[21 ] = 16'h4EFA;   // done: JMP (d16,PC) -> fin
	dut.u_l1.mem[22 ] = 16'h0004;
	dut.u_l1.mem[23 ] = 16'h74FF;   // MOVEQ #-1,D2   poison, jumped over
	dut.u_l1.mem[24 ] = 16'h4E71;   // fin: NOP
	dut.u_l1.mem[512] = 16'h5282;   // sub1: ADDQ.L #1,D2
	dut.u_l1.mem[513] = 16'h4E75;   // RTS
	dut.u_l1.mem[516] = 16'h5282;   // sub2: ADDQ.L #1,D2
	dut.u_l1.mem[517] = 16'h4E75;   // RTS
	dut.u_l1.mem[520] = 16'h5282;   // sub3: ADDQ.L #1,D2
	dut.u_l1.mem[521] = 16'h4E75;   // RTS

end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// BSR, LINK and MOVEM all push, so A7 must point somewhere real. See
	// tb_ap040_pipe_move_mem.v's header for why the poke lands here.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $04A0 is word index 80.
	if (dbg_d2 !== 32'h0000_0004) begin
		errors = errors + 1;
		$display("FAIL: checkpoints D2 = %h, expected 00000004 (ffffffff = a JMP fell through onto a poison)",
		         dbg_d2);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (three JSR/RTS pairs must balance)", dut.u_cpu.u_regfile.isp);
	end
	if (dbg_d0 !== 32'h0000_0004 || dbg_d1 !== 32'h0000_0002) begin
		errors = errors + 1;
		$display("FAIL: D0/D1 = %h/%h, expected 4/2 (the index registers must be untouched)", dbg_d0, dbg_d1);
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
