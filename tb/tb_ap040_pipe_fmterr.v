//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 76: RTE format error)     //
//                                                                          //
// tb_ap040_pipe_fmterr.v - an RTE that finds a frame it cannot pop         //
//                                                                          //
// RTE reads the format nibble off the stack. $0 is eight bytes, $2 and $3  //
// twelve, and anything else is a format error: vector 14, a format-$0      //
// frame whose PC is the RTE INSTRUCTION ITSELF (not the instruction after  //
// it), stacked below the bad frame with A7 otherwise untouched. That PC    //
// convention is the whole point of the exception: a handler can repair     //
// the frame and return, and the RTE runs again.                            //
//                                                                          //
// This program does exactly that, so the check is end to end:             //
//                                                                          //
//   A7 = $0600, supervisor from reset (SR $2700)                           //
//   push $B008 / PC=$0420 / SR=$2700 by hand   (a format-$B frame @ $05F8) //
//   ORI #$1F,CCR                                SR := $271F                //
//   RTE @ $041E                                 -> format error, vector 14  //
//   handler @ $0800: LEA (A7),A0                                            //
//                    MOVE.W (A0)+,D2            stacked SR      -> $271F     //
//                    MOVE.L (A0)+,D3            stacked PC      -> $0000041E //
//                    MOVE.W (A0)+,D4            format/vector   -> $0038     //
//                    MOVE.W (A0)+,D5            bad frame's SR  -> $2700     //
//                    MOVE.L (A0)+,D6            bad frame's PC  -> $00000420 //
//                    MOVEQ #0,D7                                             //
//                    MOVE.W D7,(A0)             repair: format word := $0000 //
//                    RTE                        -> back to the RTE at $041E  //
//   the RTE runs again on a format-$0 frame, pops eight bytes, lands at     //
//   $0420: MOVEQ #$2A,D1, SR = $2700 from the frame                         //
//                                                                          //
// Then the other half of the nibble check: a twelve-byte frame with format  //
// $3 (address field / $3008 / PC / SR), RTE, MOVEQ #$33,D0 at the landing   //
// point. A7 back at $0600 says twelve bytes were popped, not eight, and D3  //
// still $041E says no second format error was taken.                        //
//                                                                          //
// What each value proves. D3 is the convention that makes repair possible   //
// (the RTE's own address, not $0420). D2 is the live SR at the fault, CCR    //
// included -- the frame is stacked from EA-fetch's forwarded view. D4 is the  //
// vector. D5/D6 show the handler found the bad frame exactly eight bytes    //
// above its own, i.e. the format error left A7 where it was. D1 and the      //
// final SR show the repaired RTE completed. A7 = $0600 closes both frames.    //
//                                                                          //
// On milestone-75 RTL only $2 is recognised: the first RTE pops eight bytes  //
// and lands at $0420 directly, so D1 and the final SR look right and D2..D6  //
// stay zero; the format-$3 frame is popped as eight bytes too, so A7 ends    //
// $05FC. (The control run said so; a first draft of this header claimed     //
// part 2 would pass there.)                                                  //
//                                                                          //
// Vector 14 sits at word index 3612 (see tb_ap040_pipe_rte_fmt2.v for the    //
// PC_RESET-relative aliasing that puts vector n at 3584 + 2n).               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_fmterr;

localparam PROG_WORDS      = 128;
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

task check32;
	input string  what;
	input [31:0]  got, want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", what, got, want);
		end
	end
endtask

initial begin
	#1;
	// Part 1: a format-$B frame, built by hand, then RTE
	dut.u_l1.mem[1 ] = 16'h203C;   // MOVE.L #$0000B008,D0
	dut.u_l1.mem[2 ] = 16'h0000;
	dut.u_l1.mem[3 ] = 16'hB008;
	dut.u_l1.mem[4 ] = 16'h3F00;   // MOVE.W D0,-(A7)      format/vector word @ $05FE
	dut.u_l1.mem[5 ] = 16'h203C;   // MOVE.L #$00000420,D0
	dut.u_l1.mem[6 ] = 16'h0000;
	dut.u_l1.mem[7 ] = 16'h0420;
	dut.u_l1.mem[8 ] = 16'h2F00;   // MOVE.L D0,-(A7)      PC @ $05FA
	dut.u_l1.mem[9 ] = 16'h203C;   // MOVE.L #$00002700,D0
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h2700;
	dut.u_l1.mem[12] = 16'h3F00;   // MOVE.W D0,-(A7)      SR @ $05F8; A7 = $05F8
	dut.u_l1.mem[13] = 16'h003C;   // ORI #$1F,CCR         SR := $271F
	dut.u_l1.mem[14] = 16'h001F;
	dut.u_l1.mem[15] = 16'h4E73;   // RTE @ $041E          format $B: vector 14
	dut.u_l1.mem[16] = 16'h722A;   // MOVEQ #$2A,D1  @ $0420   landing point after the repair
	// Part 2: a format-$3 frame (twelve bytes), then RTE
	dut.u_l1.mem[17] = 16'h203C;   // MOVE.L #$DEADBEEF,D0
	dut.u_l1.mem[18] = 16'hDEAD;
	dut.u_l1.mem[19] = 16'hBEEF;
	dut.u_l1.mem[20] = 16'h2F00;   // MOVE.L D0,-(A7)      address field @ $05FC
	dut.u_l1.mem[21] = 16'h203C;   // MOVE.L #$00003008,D0
	dut.u_l1.mem[22] = 16'h0000;
	dut.u_l1.mem[23] = 16'h3008;
	dut.u_l1.mem[24] = 16'h3F00;   // MOVE.W D0,-(A7)      format $3 @ $05FA
	dut.u_l1.mem[25] = 16'h203C;   // MOVE.L #$00000444,D0
	dut.u_l1.mem[26] = 16'h0000;
	dut.u_l1.mem[27] = 16'h0444;
	dut.u_l1.mem[28] = 16'h2F00;   // MOVE.L D0,-(A7)      PC @ $05F6
	dut.u_l1.mem[29] = 16'h203C;   // MOVE.L #$00002700,D0
	dut.u_l1.mem[30] = 16'h0000;
	dut.u_l1.mem[31] = 16'h2700;
	dut.u_l1.mem[32] = 16'h3F00;   // MOVE.W D0,-(A7)      SR @ $05F4; A7 = $05F4
	dut.u_l1.mem[33] = 16'h4E73;   // RTE                  format $3: twelve bytes
	dut.u_l1.mem[34] = 16'h7033;   // MOVEQ #$33,D0  @ $0444
	dut.u_l1.mem[35] = 16'h4E71;   // NOP

	// Format-error handler @ $0800
	dut.u_l1.mem[512] = 16'h41D7;  // LEA (A7),A0
	dut.u_l1.mem[513] = 16'h3418;  // MOVE.W (A0)+,D2      stacked SR
	dut.u_l1.mem[514] = 16'h2618;  // MOVE.L (A0)+,D3      stacked PC
	dut.u_l1.mem[515] = 16'h3818;  // MOVE.W (A0)+,D4      format/vector word
	dut.u_l1.mem[516] = 16'h3A18;  // MOVE.W (A0)+,D5      the bad frame's SR
	dut.u_l1.mem[517] = 16'h2C18;  // MOVE.L (A0)+,D6      the bad frame's PC
	dut.u_l1.mem[518] = 16'h7E00;  // MOVEQ #0,D7
	dut.u_l1.mem[519] = 16'h3087;  // MOVE.W D7,(A0)       repair the format word
	dut.u_l1.mem[520] = 16'h4E73;  // RTE                  -> the RTE at $041E again

	// Vector 14 -> $0800
	dut.u_l1.mem[3612] = 16'h0000;
	dut.u_l1.mem[3613] = 16'h0800;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// Part 1
	check32("D1 (landing point after the repaired RTE)",       dbg_d1,                32'h0000_002A);
	check32("D2 (stacked SR, the live SR with CCR at the RTE)", dbg_d2,                32'h0000_271F);
	check32("D3 (stacked PC, the RTE's own address)",          dut.u_cpu.u_regfile.dreg[3], 32'h0000_041E);
	check32("D4 (format/vector word: format 0, vector 14)",    dut.u_cpu.u_regfile.dreg[4], 32'h0000_0038);
	check32("D5 (bad frame's SR, eight bytes above the handler's)", dut.u_cpu.u_regfile.dreg[5], 32'h0000_2700);
	check32("D6 (bad frame's PC)",                             dut.u_cpu.u_regfile.dreg[6], 32'h0000_0420);
	// Part 2
	check32("D0 (landing point after the format-3 RTE)",       dbg_d0,                32'h0000_0033);
	// Both parts
	check32("A7 (both frames popped whole)",                   dut.u_cpu.u_regfile.isp,     32'h0000_0600);
	check32("SR (from the last frame popped)",                 {16'd0, dut.u_cpu.sr},       32'h0000_2700);

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
