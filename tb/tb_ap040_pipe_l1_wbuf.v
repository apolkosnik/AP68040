//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 12: posted write    //
// buffer)                                                                  //
//                                                                          //
// tb_ap040_pipe_l1_wbuf.v - ap040_pipe_l1.v's write buffer, standalone     //
//                                                                          //
// No pipeline instruction drives wren_b yet (BSR/JSR's stack push will be   //
// the first), so this is a direct unit test against ap040_pipe_l1.v itself  //
// -- the first standalone-module pipe testbench, rather than instantiating  //
// the full ap040_pipe_core.v the way every other tb_ap040_pipe_*.v does.     //
//                                                                          //
// Three cases:                                                             //
//                                                                          //
// A. Post-then-drain-then-land: post one write, confirm wr_busy is high      //
//    the cycle it's posted and drops the cycle after (exactly one drain      //
//    cycle, not longer), then confirm the value actually landed in mem[]      //
//    -- checked through PORT A (two 16-bit reads, high then low word),         //
//    deliberately NOT through port B's own q_b, so a bug where q_b's read-      //
//    after-write forwarding masks a write that never really reached mem[]       //
//    would be caught rather than hidden.                                        //
//                                                                          //
// B. Back-to-back posts: post write 1, then immediately try to post write 2     //
//    while wr_busy is still high (held stable, per the module's own                //
//    contract -- see its header) -- confirm write 2 is NOT accepted until           //
//    wr_busy drops (exactly one cycle later), and BOTH writes eventually land        //
//    correctly, neither lost nor corrupting the other's address.                     //
//                                                                          //
// C. Read-after-write forwarding: post a write, then on the VERY NEXT cycle           //
//    (buffer still undrained) issue a READ to the SAME address -- q_b must             //
//    reflect the BUFFERED value, not stale mem[] content.                                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_l1_wbuf;

localparam AW = 8;

reg clk = 0;
reg nreset = 0;

always #5 clk = ~clk;

reg  [AW-1:0] address_a;
reg           wren_a_r;
wire          en_a = 1'b1;
wire   [15:0] q_a;

reg  [AW-1:0] address_b;
reg    [31:0] data_b;
reg           wren_b;
wire          wr_busy;
wire   [31:0] q_b;

ap040_pipe_l1 #(.AW(AW), .DW(16)) dut
(
	.clock     (clk),
	.nreset    (nreset),

	.address_a (address_a),
	.data_a    (16'h0),
	.wren_a    (1'b0),
	.en_a      (en_a),
	.q_a       (q_a),

	.address_b (address_b),
	.data_b    (data_b),
	.wren_b    (wren_b),
	.wr_busy   (wr_busy),
	.q_b       (q_b)
);

integer errors = 0;

// string, not a fixed-width [N:0] vector -- a first draft used [255:0] and
// silently truncated every message over 32 characters (keeping only the
// low/rightmost bits, i.e. the TAIL of the message), garbling exactly
// which check had failed and making an already-confusing race-condition
// debugging session (see the #1 comment above) actively misleading.
// -g2012 (already required for this whole test suite) makes SystemVerilog's
// unbounded string type available, which has no such limit.
task check32;
	input [31:0] got;
	input [31:0] expected;
	input string msg;
	begin
		if (got !== expected) begin
			errors = errors + 1;
			$display("FAIL: %0s: got %h, expected %h", msg, got, expected);
		end
	end
endtask

task check1;
	input got;
	input expected;
	input string msg;
	begin
		if (got !== expected) begin
			errors = errors + 1;
			$display("FAIL: %0s: got %b, expected %b", msg, got, expected);
		end
	end
endtask

initial begin
	address_a = 0; address_b = 0; data_b = 0; wren_b = 0;

	nreset = 0;
	repeat (2) @(posedge clk);
	#1;
	nreset = 1;
	@(posedge clk);
	#1;

	// Discipline followed throughout: every @(posedge clk) is immediately
	// followed by #1, and ONLY THEN is any DUT-driven signal read or any
	// new stimulus asserted. wr_busy is a continuous assign off
	// wbuf_valid, itself updated via a non-blocking assignment in the
	// DUT's always block; a testbench process resuming from
	// @(posedge clk) runs in the SAME active region as that update, before
	// the NBA region commits it -- reading it (or, just as easily,
	// asserting NEW stimulus that the DUT's own always block might also
	// observe as applying to the edge that JUST fired rather than the
	// NEXT one) in that same instant races the scheduler genuinely, not
	// hypothetically. A first draft of this test skipped #1 in most
	// places and got inconsistent results depending on WHERE the race
	// happened to land: case A's own first check passed by scheduling
	// luck while an analogous stimulus-timing mistake in case B silently
	// posted a write one full edge earlier than intended. #1 removes the
	// ambiguity everywhere, not just where a failure happened to surface.

	// -------------------- Case A: post, drain, land --------------------
	address_b = 8'h10; data_b = 32'hAABB_CCDD; wren_b = 1;
	@(posedge clk); #1;
	wren_b = 0;
	// wr_busy must be high THIS cycle -- the write just posted, not yet
	// drained (drain happens on the NEXT edge).
	check1(wr_busy, 1'b1, "case A: wr_busy not asserted right after posting");
	@(posedge clk); #1;
	// One drain cycle later, the buffer must be empty again.
	check1(wr_busy, 1'b0, "case A: wr_busy still asserted one cycle after posting (drain took too long)");

	// Confirm the write actually landed in mem[], via port A -- not q_b.
	address_a = 8'h10;
	@(posedge clk); @(posedge clk); #1;
	check32({16'h0, q_a}, {16'h0, 16'hAABB}, "case A: high word did not land in mem[] (port A)");
	address_a = 8'h11;
	@(posedge clk); @(posedge clk); #1;
	check32({16'h0, q_a}, {16'h0, 16'hCCDD}, "case A: low word did not land in mem[] (port A)");

	// -------------------- Case B: back-to-back posts --------------------
	// Write 1 posts normally.
	address_b = 8'h20; data_b = 32'h1111_2222; wren_b = 1;
	@(posedge clk); #1;
	check1(wr_busy, 1'b1, "case B: wr_busy not asserted right after the first post");

	// Write 2's request is asserted NOW, while wr_busy is still high, and
	// HELD STABLE (per the module's contract) until it lands -- this is
	// the real requester protocol (matching how ap040_ea_fetch.v already
	// holds a pending request stable across its own stall_in), not a
	// single blind attempt. Draining write 1 and accepting a genuinely NEW
	// post are mutually exclusive per edge (see the module header), so
	// from HERE (request 2 arriving while busy) this needs TWO more
	// edges: one to drain write 1 (write 2 not yet accepted), one more to
	// actually accept write 2 -- the "at most one cycle" bound in the
	// module header is from the moment wr_busy is OBSERVED to drop, not
	// from the moment a request first starts waiting behind a busy write.
	address_b = 8'h30; data_b = 32'h3333_4444; wren_b = 1;
	@(posedge clk); #1;
	// Write 1 just drained. Write 2 -- held stable since before this edge
	// -- was NOT accepted this same edge (drain took priority, per the
	// module's own priority rule): wr_busy must read LOW here, for
	// exactly this one edge, before write 2 gets its turn.
	check1(wr_busy, 1'b0, "case B: wr_busy still high right after write 1's drain (should be low for exactly one edge before write 2 is accepted)");
	@(posedge clk); #1;
	// Write 2 -- held stable the whole time -- is accepted THIS edge
	// (wr_busy read low going in), so it's now pending.
	check1(wr_busy, 1'b1, "case B: write 2 was not accepted the edge after wr_busy dropped");
	wren_b = 0;
	@(posedge clk); #1;
	// And drains here.
	check1(wr_busy, 1'b0, "case B: second post never drained (lost, or stuck)");

	address_a = 8'h20;
	@(posedge clk); @(posedge clk); #1;
	check32({16'h0, q_a}, {16'h0, 16'h1111}, "case B: first write's high word wrong/missing");
	address_a = 8'h21;
	@(posedge clk); @(posedge clk); #1;
	check32({16'h0, q_a}, {16'h0, 16'h2222}, "case B: first write's low word wrong/missing");
	address_a = 8'h30;
	@(posedge clk); @(posedge clk); #1;
	check32({16'h0, q_a}, {16'h0, 16'h3333}, "case B: second write's high word wrong/missing");
	address_a = 8'h31;
	@(posedge clk); @(posedge clk); #1;
	check32({16'h0, q_a}, {16'h0, 16'h4444}, "case B: second write's low word wrong/missing");

	// -------------------- Case C: read-after-write forwarding -----------
	address_b = 8'h40; data_b = 32'hDEAD_BEEF; wren_b = 1;
	@(posedge clk); #1;
	wren_b = 0;
	// Buffer is now holding $DEADBEEF at $40, undrained (wr_busy high).
	// Issue a read to the SAME address on this, the very next cycle.
	check1(wr_busy, 1'b1, "case C: buffer not holding the write when the forwarding read is issued");
	address_b = 8'h40;
	@(posedge clk); #1;
	check32(q_b, 32'hDEAD_BEEF, "case C: read did not forward the buffered (undrained) write");

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
