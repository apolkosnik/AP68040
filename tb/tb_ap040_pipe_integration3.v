//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (integration, after milestone 69)    //
//                                                                          //
// tb_ap040_pipe_integration3.v - a mispredict on top of every stall        //
//                                                                          //
// Milestone 69's integration bench found a lost redirect: EX's recovery is //
// a one-cycle event, and it landed in the one cycle the front end was      //
// stalled by a memory load that had been fetched speculatively behind a    //
// not-taken branch. The fix covers every stall source uniformly. This      //
// bench holds it to that: decode predicts every branch TAKEN, so each      //
// not-taken BNE below is a recovery -- and each one is placed immediately  //
// after a different multi-cycle instruction.                               //
//                                                                          //
//   DIVU.W #7,D0 / CMPI.W #14,D0 / BNE fail     recovery behind a divide  //
//   MOVEM.L (A7)+,D1 / CMPI / BNE fail          recovery behind a MOVEM   //
//   ADD.L D0,(A0) / MOVE.L (A0),D1 / CMPI / BNE recovery behind RMW+load  //
//   BSR f1 -> BSR f2 -> RTS -> RTS / CMPI / BNE recovery behind nested RTS //
//                                                                          //
// D2 counts checkpoints; a wrongly-taken BNE lands on fail and sets it to  //
// -1, so the outcome is a single number: 4 if every recovery landed, -1 if //
// any branch went the wrong way, and fewer than 4 if the pipeline hung     //
// somewhere in between -- the failure mode milestone 69 actually saw.      //
//                                                                          //
// The RMW step also reads back through the write buffer's forward, and     //
// the nested call unwinds two return addresses through RTS.               //
//                                                                          //
// fail OPENS WITH A LOAD, and that is the whole mechanism. Decode fetches   //
// the predicted-taken target speculatively; if that target is a memory     //
// instruction, its mem_issue stalls the front end in exactly the cycle the //
// not-taken recovery arrives, and an IF that only advances when unstalled  //
// loses the redirect. The first version of this bench put a MOVEQ at fail, //
// nothing stalled, and it PASSED on the RTL with the bug -- the control    //
// run said so, and the bench was corrected rather than the claim kept.     //
//                                                                          //
// fail falls straight into done, because a BRA.B to the next instruction   //
// is not encodable -- a byte displacement of zero selects the word form.  //
//                                                                          //
// Data and subroutines sit past $0800, out of fall-through reach.          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_integration3;

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
	dut.u_l1.mem[1  ] = 16'h207C;   // MOVEA.L #$0900,A0
	dut.u_l1.mem[2  ] = 16'h0000;
	dut.u_l1.mem[3  ] = 16'h0900;
	dut.u_l1.mem[4  ] = 16'h7400;   // MOVEQ #0,D2   checkpoints
	dut.u_l1.mem[5  ] = 16'h203C;   // MOVE.L #100,D0
	dut.u_l1.mem[6  ] = 16'h0000;
	dut.u_l1.mem[7  ] = 16'h0064;
	dut.u_l1.mem[8  ] = 16'h80FC;   // DIVU.W #7,D0  -> 0002000E
	dut.u_l1.mem[9  ] = 16'h0007;
	dut.u_l1.mem[10 ] = 16'h0C40;   // CMPI.W #14,D0
	dut.u_l1.mem[11 ] = 16'h000E;
	dut.u_l1.mem[12 ] = 16'h6600;   // BNE.W fail  (not taken: recovery after a DIVIDE)
	dut.u_l1.mem[13 ] = 16'h005E;
	dut.u_l1.mem[14 ] = 16'h5282;   // ADDQ.L #1,D2  checkpoint 1
	dut.u_l1.mem[15 ] = 16'h223C;   // MOVE.L #$12345678,D1
	dut.u_l1.mem[16 ] = 16'h1234;
	dut.u_l1.mem[17 ] = 16'h5678;
	dut.u_l1.mem[18 ] = 16'h48E7;   // MOVEM.L D1,-(A7)
	dut.u_l1.mem[19 ] = 16'h4000;
	dut.u_l1.mem[20 ] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[21 ] = 16'h4CDF;   // MOVEM.L (A7)+,D1
	dut.u_l1.mem[22 ] = 16'h0002;
	dut.u_l1.mem[23 ] = 16'h0C81;   // CMPI.L #$12345678,D1
	dut.u_l1.mem[24 ] = 16'h1234;
	dut.u_l1.mem[25 ] = 16'h5678;
	dut.u_l1.mem[26 ] = 16'h6600;   // BNE.W fail  (not taken: recovery after a MOVEM)
	dut.u_l1.mem[27 ] = 16'h0042;
	dut.u_l1.mem[28 ] = 16'h5282;   // ADDQ.L #1,D2  checkpoint 2
	dut.u_l1.mem[29 ] = 16'h7005;   // MOVEQ #5,D0
	dut.u_l1.mem[30 ] = 16'hD190;   // ADD.L D0,(A0)  [$0900] 10 -> 15
	dut.u_l1.mem[31 ] = 16'h2210;   // MOVE.L (A0),D1
	dut.u_l1.mem[32 ] = 16'h0C81;   // CMPI.L #15,D1
	dut.u_l1.mem[33 ] = 16'h0000;
	dut.u_l1.mem[34 ] = 16'h000F;
	dut.u_l1.mem[35 ] = 16'h6600;   // BNE.W fail  (not taken: recovery after RMW+load)
	dut.u_l1.mem[36 ] = 16'h0030;
	dut.u_l1.mem[37 ] = 16'h5282;   // ADDQ.L #1,D2  checkpoint 3
	dut.u_l1.mem[38 ] = 16'h6100;   // BSR.W f1
	dut.u_l1.mem[39 ] = 16'h03B2;
	dut.u_l1.mem[40 ] = 16'h0C81;   // CMPI.L #$2B,D1
	dut.u_l1.mem[41 ] = 16'h0000;
	dut.u_l1.mem[42 ] = 16'h002B;
	dut.u_l1.mem[43 ] = 16'h6600;   // BNE.W fail  (not taken: recovery after RTS)
	dut.u_l1.mem[44 ] = 16'h0020;
	dut.u_l1.mem[45 ] = 16'h5282;   // ADDQ.L #1,D2  checkpoint 4
	dut.u_l1.mem[46 ] = 16'h601E;   // BRA.B done
	dut.u_l1.mem[47 ] = 16'h0000;
	dut.u_l1.mem[60 ] = 16'h2610;   // fail: MOVE.L (A0),D3   -- a LOAD, so the speculative fetch of it stalls IF
	dut.u_l1.mem[61 ] = 16'h74FF;   // MOVEQ #-1,D2   (falls into done)
	dut.u_l1.mem[62 ] = 16'h4E71;   // done: NOP
	dut.u_l1.mem[512] = 16'h6100;   // f1: BSR.W f2
	dut.u_l1.mem[513] = 16'h000E;
	dut.u_l1.mem[514] = 16'h5281;   // ADDQ.L #1,D1
	dut.u_l1.mem[515] = 16'h4E75;   // RTS
	dut.u_l1.mem[520] = 16'h722A;   // f2: MOVEQ #$2A,D1
	dut.u_l1.mem[521] = 16'h4E75;   // RTS
	dut.u_l1.mem[640] = 16'h0000;   // data $0900 = 10
	dut.u_l1.mem[641] = 16'h000A;

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
		$display("FAIL: checkpoints D2 = %h, expected 00000004 (ffffffff = a BNE went the wrong way; less = hung)",
		         dbg_d2);
	end
	if (dbg_d1 !== 32'h0000_002B) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000002b (f2 set $2A, f1 added 1)", dbg_d1);
	end
	if ({dut.u_l1.mem[640], dut.u_l1.mem[641]} !== 32'h0000_000F) begin
		errors = errors + 1;
		$display("FAIL: [$0900] = %h%h, expected 0000000f (the RMW)", dut.u_l1.mem[640], dut.u_l1.mem[641]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (MOVEM and two BSR/RTS pairs must balance)", dut.u_cpu.u_regfile.isp);
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
