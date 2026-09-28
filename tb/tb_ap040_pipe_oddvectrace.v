//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 113: an odd vector  //
// under a pending trace)                                                   //
//                                                                          //
// tb_ap040_pipe_oddvectrace.v - the address error outranks the trace      //
//                                                                          //
// A traced TRAP owes a trace once its entry completes. If that entry's     //
// vector comes back odd, it owes an address error as well, and the 68040  //
// takes the ADDRESS ERROR: ap040_core.v's S_EXC_JMP checks the handler's   //
// parity first and cancels texc_pend before raising it. This core armed    //
// the trace on the entry regardless, the trace outranks an address error   //
// in exc_vec_num, and starting it cleared vecodd_pend -- so vector 9 was   //
// taken with the alias as its PC and the TRAP as its address, and the      //
// address-error handler never ran (review 11 finding 3).                   //
//                                                                          //
// Both trace modes, in one program. T1 traces everything; T0 traces only a //
// change of flow, which an exception entry is.                            //
//                                                                          //
//   A7 = $1000, ORI #$8000,SR (T1), TRAP #0    vector 32 -> $701           //
//   $780 (vector 3): ADDQ.L #1,D4                                          //
//                    second time through: stop                             //
//                    first time: ORI #$4000,SR (T0), TRAP #1               //
//                                                  vector 33 -> $741       //
//   $7C0 (vector 9): MOVEQ #$99,D5 -- the trace handler, never entered     //
//                                                                          //
// Frames, each TRAP's format 0 followed by its address error's format 2:  //
//   $FF8  TRAP #0, SR $A700      $FEC  AE: SR $2700, PC $80, addr $700     //
//   $FE4  TRAP #1, SR $6709      $FD8  AE: SR $2709, PC $84, addr $740     //
// The AE frames' SR has T clear: the TRAP entries cleared it. The second   //
// pair carries N and C from the handler's CMPI.L #2 with D4 = 1. D4 = 2,   //
// and D5 = 0 says no trace was ever taken.                                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_oddvectrace;

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
wire [31:0] dbg_d3, dbg_d4, dbg_d5;
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
	.dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5),
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

	dut.u_l1.mem[ 0] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 1] = 16'h0000;
	dut.u_l1.mem[ 2] = 16'h1000;
	dut.u_l1.mem[ 3] = 16'h4E7B;   // MOVEC D0,ISP    (A7 = $1000)
	dut.u_l1.mem[ 4] = 16'h0804;
	dut.u_l1.mem[ 5] = 16'h007C;   // ORI #$8000,SR   -- T1
	dut.u_l1.mem[ 6] = 16'h8000;
	dut.u_l1.mem[ 7] = 16'h4E40;   // TRAP #0   -- vector 32, handler $701
	dut.u_l1.mem[ 8] = 16'h60FE;   // BRA.B -2

	// $700 and $740: the two odd handlers' aliases, poisoned.
	dut.u_l1.mem[384] = 16'h7677;  // MOVEQ #$77,D3
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2
	dut.u_l1.mem[416] = 16'h7677;  // MOVEQ #$77,D3
	dut.u_l1.mem[417] = 16'h60FE;  // BRA.B -2

	// $780: vector 3.
	dut.u_l1.mem[448] = 16'h5284;  // ADDQ.L #1,D4
	dut.u_l1.mem[449] = 16'h0C84;  // CMPI.L #2,D4
	dut.u_l1.mem[450] = 16'h0000;
	dut.u_l1.mem[451] = 16'h0002;
	dut.u_l1.mem[452] = 16'h6706;  // BEQ.B +6 -> $790
	dut.u_l1.mem[453] = 16'h007C;  // ORI #$4000,SR   -- T0
	dut.u_l1.mem[454] = 16'h4000;
	dut.u_l1.mem[455] = 16'h4E41;  // TRAP #1   -- vector 33, handler $741
	dut.u_l1.mem[456] = 16'h60FE;  // BRA.B -2  ($790)

	// $7C0: vector 9, the trace handler. Entering it at all is the failure.
	dut.u_l1.mem[480] = 16'h7A99;  // MOVEQ #$99,D5
	dut.u_l1.mem[481] = 16'h60FE;  // BRA.B -2

	// Vectors 3, 9, 32 ($701) and 33 ($741).
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0780;
	dut.u_l1.mem[3602] = 16'h0000;  dut.u_l1.mem[3603] = 16'h07C0;
	dut.u_l1.mem[3648] = 16'h0000;  dut.u_l1.mem[3649] = 16'h0701;
	dut.u_l1.mem[3650] = 16'h0000;  dut.u_l1.mem[3651] = 16'h0741;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 800) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0002) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000002 (both odd vectors must reach the address-error handler)",
		         dbg_d4);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000 (the trace handler must never be entered)", dbg_d5);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (an odd handler's alias must never run)", dbg_d3);
	end

	// Under T1: TRAP #0's frame at $FF8, its address error's at $FEC.
	chk("T1 TRAP frame $FF8 (SR)",       1532, 16'hA700);
	chk("T1 AE frame $FEC (SR)",         1526, 16'h2700);
	chk("T1 AE frame $FF0 (PC low)",     1528, 16'h0080);
	chk("T1 AE frame $FF2 (format/vec)", 1529, 16'h200C);
	chk("T1 AE frame $FF6 (addr low)",   1531, 16'h0700);
	// Under T0: TRAP #1's frame at $FE4, its address error's at $FD8.
	chk("T0 TRAP frame $FE4 (SR)",       1522, 16'h6709);
	chk("T0 AE frame $FD8 (SR)",         1516, 16'h2709);
	chk("T0 AE frame $FDC (PC low)",     1518, 16'h0084);
	chk("T0 AE frame $FDE (format/vec)", 1519, 16'h200C);
	chk("T0 AE frame $FE2 (addr low)",   1521, 16'h0740);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
