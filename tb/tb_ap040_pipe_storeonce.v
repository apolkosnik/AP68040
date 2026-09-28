//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: one store per   //
// store)                                                                   //
//                                                                          //
// tb_ap040_pipe_storeonce.v - a store held behind a stalled EX             //
//                                                                          //
// EA-fetch's write request is combinational off eac_valid. The L1 posts a  //
// write whenever the request is asserted and its buffer is empty, and      //
// nothing in the request says "this instruction has already had its        //
// turn". While the store's own wr_stall holds it, that is exactly right:   //
// the buffer is full, so nothing is accepted and the request must stay up. //
// While something ELSE holds EA-fetch it is wrong: the instruction cannot  //
// retire, the request stays asserted, and the buffer drains and accepts    //
// again, once per drain, for as long as the stall lasts.                   //
//                                                                          //
// A divide is the longest such stall this core has. DIVU.W takes           //
// thirty-two iterative cycles in ap040_execute.v, EX holds stall_in high   //
// throughout, and a store behind it is frozen in EA-fetch the whole time.  //
//                                                                          //
//   MOVEA.L #$0480,A0 ; MOVE.L #1000,D0 ; MOVEQ #7,D2                      //
//   DIVU.W  D2,D0        D0 -> {remainder 6, quotient 142} = 0006008E      //
//   MOVE.L  D0,(A0)      $0480 -> 0006008E                                 //
//                                                                          //
// The stored VALUE is right either way, which is why this needs a count    //
// rather than a value check: every repeat writes the same frozen operands  //
// to the same frozen address, so RAM ends up correct and a bench that      //
// reads memory sees nothing wrong. A device register does not work that    //
// way -- a write-to-clear status bit, a FIFO port, a DMA trigger -- and    //
// on a real bus each repeat is a separate write cycle.                     //
//                                                                          //
// The count is of ACCEPTED posts, not of cycles the request was asserted:  //
// the L1 latches a write exactly when wren_b is high and its one-entry     //
// buffer is empty, which is one per store for a store that behaves.        //
//                                                                          //
// The second half is the same defect with no divide in sight. A second     //
// store follows a MULU, which holds EX for its own cycle, and then a       //
// third stands alone as the control: if the count is wrong for all three   //
// the request is simply free-running, and if it is wrong only for the      //
// first two it is the stall.                                              //
//                                                                          //
// A store is not the only request this happens to, and gating the store    //
// alone is not the fix. Every piece of bookkeeping that records a request  //
// having happened -- mem_pending for a read, the exception sequencer's     //
// phase, MOVEM's beat counter -- lives in the same `!stall_in` block, so   //
// a read and a frame beat repeat for exactly the same reason. Both are     //
// counted here behind the same divide:                                     //
//                                                                          //
//   DIVU.W D2,D0 ; MOVE.L (A3),D6      one read, not one per stalled cycle //
//   DIVU.W D2,D7 ; TRAP #0             two frame writes, not one per cycle //
//                                                                          //
// TRAP #0's frame is format $0, which is two 32-bit beats, and its         //
// handler returns with RTE. The read total is four, not two: the load,     //
// the vector fetch, and RTE's own two pops, which come back through the    //
// same port. The first draft expected two and forgot the RTE -- the count  //
// is only useful if it is the right count, so it is spelled out here.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_storeonce;

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
wire [31:0] dbg_d0, dbg_d6;
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

	.dbg_d0 (dbg_d0), .dbg_d6 (dbg_d6),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

// Accepted posts, not asserted cycles: the L1 latches a write exactly when
// wren_b is high and its one-entry buffer is empty.
integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

// Port B read requests. A correct core asserts one for exactly one cycle
// per access, whatever the memory latency: mem_pending goes up behind it.
integer reads = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.rd_b)
		reads = reads + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0480;
	dut.u_l1.mem[ 4] = 16'h203C;   // MOVE.L #$000003E8,D0   (1000)
	dut.u_l1.mem[ 5] = 16'h0000;
	dut.u_l1.mem[ 6] = 16'h03E8;
	dut.u_l1.mem[ 7] = 16'h7407;   // MOVEQ #7,D2
	dut.u_l1.mem[ 8] = 16'h80C2;   // DIVU.W D2,D0  -> 0006008E
	dut.u_l1.mem[ 9] = 16'h2080;   // MOVE.L D0,(A0)
	dut.u_l1.mem[10] = 16'h227C;   // MOVEA.L #$00000488,A1
	dut.u_l1.mem[11] = 16'h0000;
	dut.u_l1.mem[12] = 16'h0488;
	dut.u_l1.mem[13] = 16'h7603;   // MOVEQ #3,D3
	dut.u_l1.mem[14] = 16'h7805;   // MOVEQ #5,D4
	dut.u_l1.mem[15] = 16'hC6C4;   // MULU.W D4,D3   -> D3 = 15
	dut.u_l1.mem[16] = 16'h2283;   // MOVE.L D3,(A1)
	dut.u_l1.mem[17] = 16'h247C;   // MOVEA.L #$00000490,A2
	dut.u_l1.mem[18] = 16'h0000;
	dut.u_l1.mem[19] = 16'h0490;
	dut.u_l1.mem[20] = 16'h7A21;   // MOVEQ #$21,D5
	dut.u_l1.mem[21] = 16'h2485;   // MOVE.L D5,(A2)   (the control: no stall)
	dut.u_l1.mem[22] = 16'h267C;   // MOVEA.L #$00000498,A3
	dut.u_l1.mem[23] = 16'h0000;
	dut.u_l1.mem[24] = 16'h0498;
	dut.u_l1.mem[25] = 16'h283C;   // MOVE.L #$000003E8,D4   (1000)
	dut.u_l1.mem[26] = 16'h0000;
	dut.u_l1.mem[27] = 16'h03E8;
	dut.u_l1.mem[28] = 16'h88C2;   // DIVU.W D2,D4   (holds EX)
	dut.u_l1.mem[29] = 16'h2C13;   // MOVE.L (A3),D6 -> $5A5A5A5A
	dut.u_l1.mem[30] = 16'h2E3C;   // MOVE.L #$000003E8,D7
	dut.u_l1.mem[31] = 16'h0000;
	dut.u_l1.mem[32] = 16'h03E8;
	dut.u_l1.mem[33] = 16'h8EC2;   // DIVU.W D2,D7   (holds EX)
	dut.u_l1.mem[34] = 16'h4E40;   // TRAP #0        (frame, then RTE back)
	dut.u_l1.mem[35] = 16'h4E71;   // NOP (drain)

	// TRAP #0 handler @ word idx 256 (byte $600): straight back.
	dut.u_l1.mem[256] = 16'h4E73;  // RTE

	// Vector 32 (TRAP #0) -> $600. Word index as every exception bench uses.
	dut.u_l1.mem[3648] = 16'h0000;
	dut.u_l1.mem[3649] = 16'h0600;

	dut.u_l1.mem[64] = 16'h0000;   // $0480
	dut.u_l1.mem[65] = 16'h0000;
	dut.u_l1.mem[68] = 16'h0000;   // $0488
	dut.u_l1.mem[69] = 16'h0000;
	dut.u_l1.mem[72] = 16'h0000;   // $0490
	dut.u_l1.mem[73] = 16'h0000;
	dut.u_l1.mem[76] = 16'h5A5A;   // $0498 = 5A5A5A5A
	dut.u_l1.mem[77] = 16'h5A5A;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 160) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0006_008E) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 0006008e (1000 / 7 = 142 remainder 6)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'h0000_000F) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected 0000000f (3 * 5)",
		         dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if ({dut.u_l1.mem[72], dut.u_l1.mem[73]} !== 32'h0000_0021) begin
		errors = errors + 1;
		$display("FAIL: $0490 = %h%h, expected 00000021",
		         dut.u_l1.mem[72], dut.u_l1.mem[73]);
	end
	if (dbg_d6 !== 32'h5A5A_5A5A) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 5a5a5a5a (MOVE.L (A3),D6 behind a divide)", dbg_d6);
	end
	if (reads !== 4) begin
		errors = errors + 1;
		$display("FAIL: %0d read requests reached the L1, expected 4 (one load, one vector fetch, two RTE pops). A read held behind a stalled EX must not be re-issued every cycle -- mem_pending, which would stop it, is set in the same block the stall freezes.",
		         reads);
	end
	if (writes !== 5) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted to the L1, expected 5 (three stores and TRAP #0's two frame beats). A store held behind a stalled EX must not be accepted again every time the write buffer drains -- the value is right either way, so only the count can see it.",
		         writes);
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
