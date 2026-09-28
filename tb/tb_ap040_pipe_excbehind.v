//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: an exception      //
// behind a long instruction)                                               //
//                                                                          //
// tb_ap040_pipe_excbehind.v - the frame stacks the flags of what is ahead  //
//                                                                          //
// An exception's verdict can latch while an older instruction is still in //
// EX: a TRAP right behind a 34-cycle divide, an ILLEGAL right behind a    //
// multiply. The status register it sees then has none of that            //
// instruction's flags. The frame must stack them anyway -- its beats are  //
// held while EX is stalled and go out in EX's final cycle, when the flags //
// are forwarded. Instruction prefetch put the corpus's terminal ILLEGAL in //
// exactly this place behind every divide from (xxx).L, where the driver's //
// snapshot of the verdict read the stale value.                           //
//   MOVEQ #5,D1; DIVS.W #-1,D1; TRAP #0          stacked SR $2708 (N)     //
//   MOVEQ #3,D2; MULU.L #$40000000,D2; ILLEGAL    stacked SR $2708 (N)     //
//   MOVEQ #0,D3; TRAP #0                          stacked SR $2704 (Z)     //
// Handlers log the stacked SR at (A5)+; the ILLEGAL's steps over it.       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_excbehind;

localparam PROG_WORDS      = 400;
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
wire [31:0] dbg_d1, dbg_d2;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),
	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
	.dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;

task chk;
	input [255:0] what;
	input integer widx;
	input  [15:0] want;
	begin
		if (dut.u_l1.mem[widx] !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %04x, expected %04x", what, dut.u_l1.mem[widx], want);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[  0] = 16'h203C;  dut.u_l1.mem[  1] = 16'h0000;  dut.u_l1.mem[  2] = 16'h1000; // $400 MOVE.L #$1000,D0
	dut.u_l1.mem[  3] = 16'h4E7B;  dut.u_l1.mem[  4] = 16'h0804;                               // $406 MOVEC D0,ISP
	dut.u_l1.mem[  5] = 16'h2A7C;  dut.u_l1.mem[  6] = 16'h0000;  dut.u_l1.mem[  7] = 16'h0A00; // $40A MOVEA.L #$A00,A5          the log
	dut.u_l1.mem[  8] = 16'h7205;                                                              // $410 MOVEQ #5,D1
	dut.u_l1.mem[  9] = 16'h83FC;  dut.u_l1.mem[ 10] = 16'hFFFF;                               // $412 DIVS.W #-1,D1             -5: N, 34 cycles in EX
	dut.u_l1.mem[ 11] = 16'h4E40;                                                              // $416 TRAP #0                   its verdict comes during the divide
	dut.u_l1.mem[ 12] = 16'h7403;                                                              // $418 MOVEQ #3,D2
	dut.u_l1.mem[ 13] = 16'h4C3C;  dut.u_l1.mem[ 14] = 16'h2000;  dut.u_l1.mem[ 15] = 16'h4000;  dut.u_l1.mem[ 16] = 16'h0000; // $41A MULU.L #$40000000,D2      $C0000000: N
	dut.u_l1.mem[ 17] = 16'h4AFC;                                                              // $422 ILLEGAL                   ...during the multiply
	dut.u_l1.mem[ 18] = 16'h7600;                                                              // $424 MOVEQ #0,D3
	dut.u_l1.mem[ 19] = 16'h4E40;                                                              // $426 TRAP #0                   control: nothing ahead, Z
	dut.u_l1.mem[ 20] = 16'h60FE;                                                              // $428 BRA.B -2

	// $700: TRAP #0 -- log the stacked SR, return.
	dut.u_l1.mem[384] = 16'h3AD7;                                  // MOVE.W (A7),(A5)+
	dut.u_l1.mem[385] = 16'h4E73;                                  // RTE
	// $720: ILLEGAL -- the same, stepping over it.
	dut.u_l1.mem[400] = 16'h3AD7;                                  // MOVE.W (A7),(A5)+
	dut.u_l1.mem[401] = 16'h54AF;  dut.u_l1.mem[402] = 16'h0002;   // ADDQ.L #2,2(A7)
	dut.u_l1.mem[403] = 16'h4E73;                                  // RTE
	dut.u_l1.mem[3648] = 16'h0000;  dut.u_l1.mem[3649] = 16'h0700; // vector 32
	dut.u_l1.mem[3592] = 16'h0000;  dut.u_l1.mem[3593] = 16'h0720; // vector 4
	for (i = 768; i < 776; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00: the log
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("SR stacked behind DIVS.W",  768, 16'h2708);
	chk("SR stacked behind MULU.L",  769, 16'h2708);
	chk("SR stacked, nothing ahead", 770, 16'h2704);
	chk("nothing more logged",       771, 16'h0000);
	if (dbg_d1 !== 32'h0000_FFFB) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000fffb", dbg_d1);
	end
	if (dbg_d2 !== 32'hC000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected c0000000", dbg_d2);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
