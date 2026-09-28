//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 18: byte and word   //
// operand sizes)                                                           //
//                                                                          //
// tb_ap040_pipe_size.v - sized results keep the destination's upper bits   //
//                                                                          //
// The ALU has implemented all three widths since it was forked from        //
// rtl_old/ap040_alu.v -- nbits/szmask/sized MSB/sized carry are all there. //
// What kept the pipeline Long-only was the decoder baking size into its    //
// match predicates (ir[7:6]==10 for ADD, ir[13:12]==10 for MOVE) and       //
// ap040_execute.v hardwiring .size(AP040_SZ_L). This milestone plumbs a    //
// real size field ID -> EA-calc -> EA-fetch -> EX and merges the result by //
// size at the commit point.                                                //
//                                                                          //
// Program:                                                                //
//                                                                          //
//   1: MOVEQ #-1,D0   70FF   D0 = FFFFFFFF                                 //
//   2: MOVEQ #1,D1    7201   D1 = 00000001                                 //
//   3: ADD.B  D1,D0   D001   D0 = FFFFFF00  byte wraps, upper 24 kept      //
//   4: MOVEQ #-1,D2   74FF   D2 = FFFFFFFF                                 //
//   5: MOVE.W D1,D2   3401   D2 = FFFF0001  word moved, upper 16 kept      //
//   6: ADD.W  D1,D2   D441   D2 = FFFF0002  word add, upper 16 kept        //
//                                                                          //
// Each of the three sized instructions proves something the Long-only      //
// pipeline could not do. ADD.B wrapping FF+01 to 00 shows the ALU is       //
// genuinely operating at byte width rather than producing 00000100, and    //
// D0's surviving FFFFFF shows the commit merge is a splice and not a       //
// whole-register overwrite. MOVE.W exercises the OTHER size encoding --    //
// MOVE's ir[13:12] (01=B, 11=W, 10=L) is not ADD's ir[7:6] (00=B, 01=W,    //
// 10=L) -- so a decoder that got one right and the other wrong fails here. //
//                                                                          //
// On the pre-milestone RTL none of the three even decodes: the predicates  //
// require Long, so they fall through to illegal/NOP and every register     //
// check below fails.                                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_size;

localparam PROG_WORDS      = 12;
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
	dut.u_l1.mem[1] = 16'h70FF;   // MOVEQ #-1,D0
	dut.u_l1.mem[2] = 16'h7201;   // MOVEQ #1,D1
	dut.u_l1.mem[3] = 16'hD001;   // ADD.B  D1,D0
	dut.u_l1.mem[4] = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[5] = 16'h3401;   // MOVE.W D1,D2
	dut.u_l1.mem[6] = 16'hD441;   // ADD.W  D1,D2
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 20) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hFFFF_FF00) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected FFFFFF00 (ADD.B must wrap at 8 bits and keep D0[31:8])", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0001) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000001", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_0002) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFF0002 (MOVE.W then ADD.W must keep D2[31:16])", dbg_d2);
	end

	// ADD.W is the last flag-writing instruction: word result 0002, so
	// N=0 Z=0 V=0 C=0. dbg_ccr[3:0] is {N,Z,V,C}.
	if (dbg_ccr[3:0] !== 4'b0000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0000", dbg_ccr[3:0]);
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
