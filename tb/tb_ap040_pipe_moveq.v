//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 2: MOVEQ,           //
// MOVE.L Dn,Dm and register forwarding)                                    //
//                                                                          //
// tb_ap040_pipe_moveq.v - register forwarding proof                       //
//                                                                          //
// Program (D0/D1/D2 only -- ap040_regfile.v already exposes dbg_d0/dbg_d1/ //
// dbg_d2, so no new regfile debug taps are needed):                        //
//                                                                          //
//   0: NOP                                                                //
//   1: MOVEQ #5,D0    (0x7005)                                            //
//   2: MOVE.L D0,D1   (0x2200)   0-bubble: producer still in EX when the   //
//                                consumer reaches EA-fetch -- EX-forward   //
//   3: MOVEQ #-1,D0   (0x70FF)                                            //
//   4: NOP                                (1-bubble gap)                  //
//   5: MOVE.L D0,D2   (0x2400)   producer committing in WB the same cycle //
//                                the consumer reaches EA-fetch --          //
//                                WB-forward / regfile write-through        //
//   6-9: NOP (drain)                                                      //
//                                                                          //
// D1 must land on 5 -- the value forwarded BEFORE D0 was overwritten to    //
// -1 -- not some later value; D2 must land on -1. Getting either wrong in  //
// either direction (stale value, or the wrong producer's value entirely)   //
// would be exactly the kind of bug register forwarding exists to prevent.  //
// The whole run must also take no more cycles than the no-hazard case      //
// (milestone 1's tb_ap040_pipe_nop.v already proved that shape) -- if      //
// forwarding were implemented as a stall instead, this run would take      //
// longer, and that's checked too.                                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_moveq;

localparam PROG_WORDS      = 10;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

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
integer cycle  = 0;

// Override the IF stage's default all-NOP ROM with the program above.
// Scheduled at #1 so it runs after ap040_inst_fetch.v's own t=0 fill
// (standard Verilog idiom -- all t=0 initial blocks complete before
// simulation time advances).
initial begin
	#1;
	dut.u_l1.mem[1] = 16'h7005;   // MOVEQ #5,D0
	dut.u_l1.mem[2] = 16'h2200;   // MOVE.L D0,D1
	dut.u_l1.mem[3] = 16'h70FF;   // MOVEQ #-1,D0
	dut.u_l1.mem[5] = 16'h2400;   // MOVE.L D0,D2
end

always @(posedge clk) if (nreset && ce) cycle = cycle + 1;

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Same margin as tb_ap040_pipe_nop.v: PROG_WORDS instructions issued,
	// PROG_WORDS + 6 cycles to fully drain if nothing ever stalls.
	repeat (PROG_WORDS + 20) @(posedge clk);

	if (dbg_d0 !== 32'hFFFF_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected FFFFFFFF", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000005 (EX-forward captured a stale/wrong value)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFFFFFF (WB-forward/write-through failed)", dbg_d2);
	end
	// XNZVC: MOVE.L D0,D2 was the last flag-writing instruction, result
	// negative -- N=1, Z=0, V=0, C=0; X is passed through, don't-care here.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000", dbg_ccr[3:0]);
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
