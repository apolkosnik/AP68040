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
// Four cases. The buffer takes a new write in the very cycle it drains the //
// last one (restructuring plan, phase 5): wr_busy is up only while a       //
// write is being HELD, which the normal build never does, so a run of      //
// stores posts one per edge. Held writes are the slow build's (0-3 extra   //
// cycles each), and there the bench waits, with a bound.                    //
//                                                                          //
// A. Post-then-drain-then-land: post one write; it is held in the buffer   //
//    (wbuf_valid) while wr_busy stays low, drains on the next edge, and    //
//    the value is in mem[] -- checked through PORT A (two 16-bit reads,    //
//    high then low word), deliberately NOT through port B's own q_b, so a  //
//    write that never reached mem[] cannot hide behind the port it came in //
//    by.                                                                   //
//                                                                          //
// B. A run of four posts, one presented per cycle and held until taken:    //
//    in the normal build all four go in four edges, never refused, and     //
//    every one lands at its own address.                                   //
//                                                                          //
// C. Order: a Long, then straight behind it a Byte inside that same        //
//    longword. The Byte is taken in the edge the Long drains, so both are  //
//    in flight together, and memory must end with the Byte over the Long   //
//    -- the other order leaves the Long's byte there instead.              //
//                                                                          //
// D. A read never overtakes a write: a read issued the cycle after a post  //
//    to the same address, and one issued after two back-to-back posts to  //
//    one longword, return what was written last.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_l1_wbuf;

localparam AW = 8;

reg clk = 0;
reg nreset = 0;

always #5 clk = ~clk;

// Both ports take a 32-bit BYTE address since milestone 81. This bench
// still reasons in word indices, so it instantiates the module at
// PC_RESET 0 and converts with ba() -- the array layout, and so every
// index below, is unchanged.
reg  [31:0]   address_a;
reg           wren_a_r;
reg           en_a = 1'b0;      // port A is REQUESTED since milestone 80 (see read_a below)
wire   [15:0] q_a;
wire          rvalid_a;

reg  [31:0]   address_b;
reg    [31:0] data_b;
reg           wren_b;
reg           rd_b = 1'b0;      // and so is a port-B read
reg     [1:0] size_b = `AP040_SZ_L;
wire          wr_busy;
wire   [31:0] q_b;
wire          rvalid_b;

function [31:0] ba;
	input [31:0] idx;
	ba = idx << 1;
endfunction

ap040_pipe_l1 #(.AW(AW), .DW(16), .PC_RESET(32'd0)) dut
(
	.clock     (clk),
	.nreset    (nreset),

	.address_a (address_a),
	.data_a    (16'h0),
	.wren_a    (1'b0),
	.en_a      (en_a),
	.q_a       (q_a),
	.rvalid_a  (rvalid_a),

	.address_b (address_b),
	.data_b    (data_b),
	.wren_b    (wren_b),
	// Port B is sized since milestone 86: Long throughout, but for case
	// C's Byte.
	.size_b    (size_b),
	.rd_b      (rd_b),
	.wr_busy   (wr_busy),
	.q_b       (q_b),
	.rvalid_b  (rvalid_b)
);

// Milestone 80: reads are requests that return with a valid, and under
// AP040_PIPE_L1_SLOW the return and the write buffer's drain take 0-3 extra
// cycles. The exact-cycle drain checks below are the module's contract in
// the normal build; the slow build waits instead, with a bound.
task read_a;
	input [31:0] addr;   // word index; converted by ba() below
	integer n;
	begin
		address_a = ba(addr); en_a = 1;
		@(posedge clk); #1;
		en_a = 0; n = 0;
		while (!rvalid_a && n < 8) begin @(posedge clk); #1; n = n + 1; end
		if (!rvalid_a) begin errors = errors + 1; $display("FAIL: port A read of word %h never returned", addr); end
	end
endtask

// The buffer empties: one edge in the normal build, a bounded wait in the
// slow one.
task wait_empty;
	input string msg;
	integer n;
	begin
`ifdef AP040_PIPE_L1_SLOW
		n = 0;
		while (dut.wbuf_valid && n < 8) begin @(posedge clk); #1; n = n + 1; end
		check1(dut.wbuf_valid, 1'b0, msg);
`else
		@(posedge clk); #1;
		check1(dut.wbuf_valid, 1'b0, msg);
`endif
	end
endtask

// Present a write and hold it until taken: the edge on which wr_busy was
// low going in. The normal build must never refuse one.
task post;
	input [31:0] addr;   // word index
	input [31:0] data;
	input  [1:0] sz;
	input        odd;    // Byte only: the low byte of the word
	input string msg;
	integer n;
	begin
		address_b = ba(addr) | {31'd0, odd}; data_b = data; size_b = sz; wren_b = 1;
		n = 0;
		while (wr_busy && n < 8) begin
`ifndef AP040_PIPE_L1_SLOW
			errors = errors + 1;
			$display("FAIL: %0s: refused with nothing held", msg);
`endif
			@(posedge clk); #1; n = n + 1;
		end
		@(posedge clk); #1;
		wren_b = 0; size_b = `AP040_SZ_L;
		check1(dut.wbuf_valid, 1'b1, {msg, ": not held in the buffer once taken"});
	end
endtask

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
	post(8'h10, 32'hAABB_CCDD, `AP040_SZ_L, 1'b0, "case A");
`ifndef AP040_PIPE_L1_SLOW
	// Held, but not busy: the next write could be taken as this one drains.
	check1(wr_busy, 1'b0, "case A: wr_busy up for a write that is not being held");
`endif
	wait_empty("case A: the write did not drain on the next edge");

	// Confirm the write actually landed in mem[], via port A -- not q_b.
	read_a(8'h10);
	check32({16'h0, q_a}, {16'h0, 16'hAABB}, "case A: high word did not land in mem[] (port A)");
	read_a(8'h11);
	check32({16'h0, q_a}, {16'h0, 16'hCCDD}, "case A: low word did not land in mem[] (port A)");

	// -------------------- Case B: a run of posts, one per edge -----------
	// post() leaves the next request to be presented straight after the
	// edge that took this one, so the four occupy consecutive edges when
	// nothing is refused; it counts a refusal as a failure in the normal
	// build.
	begin : case_b
		time t0;
		t0 = $time;
		post(8'h20, 32'h1111_2222, `AP040_SZ_L, 1'b0, "case B: write 1");
		post(8'h22, 32'h3333_4444, `AP040_SZ_L, 1'b0, "case B: write 2");
		post(8'h24, 32'h5555_6666, `AP040_SZ_L, 1'b0, "case B: write 3");
		post(8'h26, 32'h7777_8888, `AP040_SZ_L, 1'b0, "case B: write 4");
`ifndef AP040_PIPE_L1_SLOW
		if ($time - t0 != 40) begin
			errors = errors + 1;
			$display("FAIL: case B: four posts took %0d ns, want four edges (40)", $time - t0);
		end
`endif
	end
	wait_empty("case B: the last post never drained (lost, or stuck)");

	read_a(8'h20); check32({16'h0, q_a}, {16'h0, 16'h1111}, "case B: write 1's high word");
	read_a(8'h21); check32({16'h0, q_a}, {16'h0, 16'h2222}, "case B: write 1's low word");
	read_a(8'h22); check32({16'h0, q_a}, {16'h0, 16'h3333}, "case B: write 2's high word");
	read_a(8'h23); check32({16'h0, q_a}, {16'h0, 16'h4444}, "case B: write 2's low word");
	read_a(8'h24); check32({16'h0, q_a}, {16'h0, 16'h5555}, "case B: write 3's high word");
	read_a(8'h25); check32({16'h0, q_a}, {16'h0, 16'h6666}, "case B: write 3's low word");
	read_a(8'h26); check32({16'h0, q_a}, {16'h0, 16'h7777}, "case B: write 4's high word");
	read_a(8'h27); check32({16'h0, q_a}, {16'h0, 16'h8888}, "case B: write 4's low word");

	// -------------------- Case C: the later write wins --------------------
	// A Long at word $50, then a Byte at its second byte: taken in the edge
	// the Long drains. $11AA/$3344 only if the Byte went in second.
	post(8'h50, 32'h1122_3344, `AP040_SZ_L, 1'b0, "case C: the Long");
	post(8'h50, 32'h0000_00AA, `AP040_SZ_B, 1'b1, "case C: the Byte");
	wait_empty("case C: the Byte never drained");
	read_a(8'h50); check32({16'h0, q_a}, {16'h0, 16'h11AA}, "case C: the Long's byte is over the Byte written after it");
	read_a(8'h51); check32({16'h0, q_a}, {16'h0, 16'h3344}, "case C: the Long's low word");

	// -------------------- Case D: a read never overtakes a write --------
	// Until milestone 86 this checked a FORWARD: the buffered value was
	// merged into the read. The port is sized now and an overlap is no
	// longer a comparison of addresses, so the read waits for the drain
	// instead -- the same ordering ap040_pipe_membus.v has always had on
	// the bus side. What the caller sees is unchanged, and that is what is
	// checked: the value written, from a read issued while it is still
	// buffered.
	post(8'h40, 32'hDEAD_BEEF, `AP040_SZ_L, 1'b0, "case D: the write");
	address_b = ba(8'h40); rd_b = 1;
	@(posedge clk); #1;
	rd_b = 0;
	begin : wait_d1
		integer n; n = 0;
		while (!rvalid_b && n < 8) begin @(posedge clk); #1; n = n + 1; end
	end
	check1(rvalid_b, 1'b1, "case D: the read never returned");
	check32(q_b, 32'hDEAD_BEEF, "case D: a read issued while the write was still buffered did not see it");
	// Two posts to one longword back to back, then the read: the second's.
	post(8'h44, 32'h0101_0101, `AP040_SZ_L, 1'b0, "case D: the first of two");
	post(8'h44, 32'h0202_0202, `AP040_SZ_L, 1'b0, "case D: the second of two");
	address_b = ba(8'h44); rd_b = 1;
	@(posedge clk); #1;
	rd_b = 0;
	begin : wait_d2
		integer n; n = 0;
		while (!rvalid_b && n < 8) begin @(posedge clk); #1; n = n + 1; end
	end
	check1(rvalid_b, 1'b1, "case D: the read behind two posts never returned");
	check32(q_b, 32'h0202_0202, "case D: a read behind two posts to one longword did not see the second");

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
