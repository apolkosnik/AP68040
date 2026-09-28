//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: a redirect      //
// that lands on an acknowledgement; 2026-09-24: the prefetch stream,       //
// review 15)                                                               //
//                                                                          //
// tb_ap040_pipe_busredirect.v - ap040_pipe_imu.v and ap040_pipe_membus.v  //
// on their own                                                             //
//                                                                          //
// Driven directly rather than through the core, for the same reason        //
// tb_ap040_pipe_l1_wbuf.v drives the array directly: the events under test //
// are COINCIDENCES, and the only way to test a coincidence is to create    //
// it. Reaching one through a program means waiting for a mispredict to     //
// land on the exact cycle a fetch is acknowledged, which depends on the    //
// memory latency of the day.                                               //
//                                                                          //
// Instruction fetch is restartable. A new en_a abandons whatever was in    //
// flight and asks for the new address, which is what the array does and    //
// what ap040_inst_fetch.v assumes when a branch redirects it. Milestone 92 //
// found the wrapper publishing the abandoned word when the redirect came   //
// in the acknowledgement's own cycle, and cancelling the new request with  //
// it. Since the prefetch stream (2026-09-24) the bridge reads aligned      //
// longwords into a four-entry window, answers a request from the window    //
// when it can, and reads ahead on its own; the memory here answers every   //
// read with the whole longword, as a 32-bit bus does. Each word names its  //
// own address: $Cnnn, nnn the word index in an 8 KB image the address     //
// space aliases onto.                                                      //
//                                                                          //
//   0. Out of reset, with no request: the bus stays idle. pf_base is zero  //
//      then, and the stream fetched $0 unasked (review 15) -- on a bus     //
//      where $0 never answers, the reset PC's fetch waited behind it for   //
//      ever.                                                               //
//   1. A plain fetch of $400: the first transaction is that longword, as  //
//      an instruction read, and the word arrives.                          //
//   2. Request $1000, and in the cycle the bus acknowledges it, request    //
//      $800. The word for $1000 must NOT become valid, and $800 must go    //
//      out on the bus afterwards.                                          //
//   3. The sequential pattern. $400 misses; $402 is in the longword just   //
//      returned and must be answered without a second read of it; $404 is //
//      requested in the very cycle its prefetch is acknowledged and must   //
//      be answered from the arriving longword; $408 is requested while its //
//      prefetch is on the bus and must wait for that read, not issue       //
//      another.                                                            //
//   4. Writes into a window that spans the top of the address space       //
//      (review 15). A stream from $FFFFFFF8 holds $FFFFFFF8, $FFFFFFFC,   //
//      $0 and $4: a word written at $FFFFFFFC, then one written at $0,    //
//      must each be what the next fetch there returns. Then a stream from //
//      $0 and a longword written at $FFFFFFFE, which touches the window    //
//      only with its second half.                                          //
//   5. Controls, away from the top: a write outside a full window leaves   //
//      it full (nothing is read again), a longword at $3FE reaches into a  //
//      window at $400 with its second half, a word inside one at $408, a   //
//      word at $410 just past a full window at $400 leaves it full, and a  //
//      longword at $40E reaches its last longword with its first half.    //
//   6. A write accepted, and a fetch of the same longword requested in    //
//      that very cycle, the longword outside the window (behind it, as a  //
//      store onto the next instruction finds it once the fetch has run    //
//      ahead). The write must reach the bus before the read of it, and   //
//      the fetch return the written word. The prefetch stream's guard      //
//      looked only at the window as it stood, so the read went out in the  //
//      write's own cycle, ahead of it: t_integer.s 192 ran the old         //
//      instruction, and t_exceptions.s 141's fetch passed the write that  //
//      armed its bus error -- both only under random ce with the slow L1.  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_busredirect;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

reg  [31:0] address_a = 32'd0;
reg         en_a      = 1'b0;
wire [15:0] q_a;
wire        rvalid_a;

reg  [31:0] address_b = 32'd0;
reg  [31:0] data_b    = 32'd0;
reg         wren_b    = 1'b0;
reg   [1:0] size_b    = `AP040_SZ_L;
reg         rd_b      = 1'b0;
wire        wr_busy;
wire [31:0] q_b;
wire        rvalid_b;

wire        mem_req, mem_write, mem_instr;
wire  [1:0] mem_size;
wire [31:0] mem_addr, mem_wdata;
wire  [2:0] mem_fc;
reg         mem_ack   = 1'b0;
reg  [31:0] mem_rdata = 32'd0;

wire        f_req, f_sup, f_free, f_ack, f_flt, f_flt_bus, f_w_accept;
wire [31:0] f_addr;
wire [29:0] f_w_sla;
ap040_pipe_imu imu (
	.clk(clk), .nreset(nreset),
	.address_a(address_a), .en_a(en_a), .q_a(q_a), .q_a2(), .rvalid_a(rvalid_a),
	.rflt_a(), .rflt_a_bus(),
	// the fetch side alone: no invalidation, no faults, nothing translated
	.sup(1'b1), .pf_inval(1'b0), .quiesce(1'b0),
	.ic_en(1'b0), .itt0(32'd0), .itt1(32'd0),
	.cm_req(1'b0), .cm_ic(1'b0), .cm_scope(2'd0), .cm_addr(32'd0), .cm_done(),
	.sn_req(1'b0), .sn_addr(32'd0),
	.pf_xlat(1'b0), .x_req(), .x_addr(), .x_sup(), .x_pass(1'b0), .x_flt(1'b0), .x_pa(32'd0), .x_cm(2'b00),
	.pk_addr(), .pk_sup(), .pk_hit(1'b0), .pk_pa(32'd0), .pk_cm(2'b00),
	.f_req(f_req), .f_addr(f_addr), .f_sup(f_sup), .f_free(f_free),
	.f_ack(f_ack), .f_rdata(mem_rdata), .f_flt(f_flt), .f_flt_bus(f_flt_bus),
	.w_accept(f_w_accept), .w_sla(f_w_sla)
);
ap040_pipe_membus dut (
	.clk(clk), .nreset(nreset),
	.f_req(f_req), .f_addr(f_addr), .f_sup(f_sup), .f_free(f_free),
	.f_ack(f_ack), .f_flt(f_flt), .f_flt_bus(f_flt_bus),
	.w_accept(f_w_accept), .w_sla(f_w_sla),
	.address_b(address_b), .la_b(address_b), .data_b(data_b), .wren_b(wren_b),
	.size_b(size_b), .rd_b(rd_b), .wr_busy(wr_busy),
	.q_b(q_b), .rvalid_b(rvalid_b),
	.sup_b(1'b1),
	.mem_req(mem_req), .mem_write(mem_write), .mem_instr(mem_instr),
	.mem_size(mem_size), .mem_addr(mem_addr), .mem_wdata(mem_wdata),
	.mem_fc(mem_fc),
	.mem_ack(mem_ack), .mem_rdata(mem_rdata),
	// no FC override, no faults
	.fc_ovr(1'b0), .fc_ovr_val(3'd0),
	.mem_flt(1'b0), .mem_flt_bus(1'b0), .mem_pass(mem_req), .wr_sync(1'b0), .wr_drop(!wren_b),
	.rflt_b(), .wflt(), .idle(), .flt_bus(),
	.xlat_e(1'b0), .xlat_p(1'b0), .pb_req(), .pb_addr(), .pb_fc(), .pb_done(1'b0), .pb_mmusr(32'd0), .flt_ma(),
	.rx(1'b0), .rx_addr(32'd0), .rx_size(2'd0), .rx_fc(3'd0)
);

integer errors = 0;
integer i;

// ---------------------------------------------------------------- memory
// 8 KB of words; every address aliases onto it by its low thirteen bits, so
// $FFFFFFFC and $0 are words 4094 and 0 and a longword at $FFFFFFFE is the
// last word and the first.
reg [15:0] mem [0:4095];
function [11:0] wi;
	input [31:0] a;
	begin wi = a[12:1]; end
endfunction
function [15:0] img;   // the word an address holds before anything writes it
	input [31:0] a;
	begin img = {4'hC, wi(a)}; end
endfunction

// Every transaction the memory accepts, in order.
reg  [31:0] log_addr  [0:1023];
reg         log_write [0:1023];
reg         log_instr [0:1023];
reg   [1:0] log_size  [0:1023];
integer     log_n = 0;

// Two cycles from request to acknowledgement: accepted, then answered. The
// cycle between is where a request is on the bus and not yet answered.
reg resp_busy = 1'b0;
always @(posedge clk) begin
	if (!nreset) begin
		resp_busy <= 1'b0;
		mem_ack   <= 1'b0;
	end else begin
		mem_ack <= 1'b0;
		if (!resp_busy && mem_req && !mem_ack) begin
			resp_busy <= 1'b1;
			log_addr[log_n]  = mem_addr;
			log_write[log_n] = mem_write;
			log_instr[log_n] = mem_instr;
			log_size[log_n]  = mem_size;
			log_n = log_n + 1;
		end else if (resp_busy) begin
			resp_busy <= 1'b0;
			mem_ack   <= 1'b1;
			if (mem_write) begin
				case (mem_size)
				`AP040_SZ_L: begin
					mem[wi(mem_addr)]         <= mem_wdata[31:16];
					mem[wi(mem_addr + 32'd2)] <= mem_wdata[15:0];
				end
				`AP040_SZ_W: mem[wi(mem_addr)] <= mem_wdata[15:0];
				default: begin
					if (mem_addr[0]) mem[wi(mem_addr)][7:0]  <= mem_wdata[7:0];
					else             mem[wi(mem_addr)][15:8] <= mem_wdata[7:0];
				end
				endcase
				mem_rdata <= 32'h0;
			end else
				mem_rdata <= {mem[wi(mem_addr)], mem[wi(mem_addr + 32'd2)]};
		end
	end
end

// Phase 0's check: any request before the first en_a.
reg     asked = 1'b0;
integer unasked = 0;
always @(posedge clk) if (nreset) begin
	if (en_a) asked <= 1'b1;
	if (mem_req && !asked) unasked = unasked + 1;
end

// ---------------------------------------------------------------- helpers
task step; begin @(posedge clk); #1; end endtask

task fail;
	input [8*96-1:0] msg;
	begin
		errors = errors + 1;
		$display("FAIL: %0s", msg);
	end
endtask

task req;   // one cycle of en_a
	input [31:0] a;
	begin
		en_a = 1'b1; address_a = a;
		step;
		en_a = 1'b0;
	end
endtask

// The answer to the latest request: immediately for a hit, after the read
// for a miss. rvalid_a is a level -- the latest request's answer is in q_a
// -- and a miss drops it in the cycle it is accepted.
task want_q;
	input [8*48-1:0] what;
	input     [15:0] want;
	integer n;
	begin
		n = 0;
		while (!rvalid_a && n < 64) begin step; n = n + 1; end
		if (!rvalid_a || q_a !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s delivered rvalid=%b q=%h, expected 1 and %h", what, rvalid_a, q_a, want);
		end
	end
endtask

// Returns in the cycle the memory acknowledges a read of a: an en_a set now
// is seen by the bridge together with that acknowledgement.
task on_ack_of;
	input [31:0] a;
	integer n;
	begin
		n = 0;
		while (!(mem_ack && !mem_write && mem_addr == a) && n < 64) begin step; n = n + 1; end
		if (n >= 64) begin
			errors = errors + 1;
			$display("FAIL: no acknowledged read of %h", a);
		end
	end
endtask

// Returns in a cycle a read of a is on the bus and not yet acknowledged.
task on_bus;
	input [31:0] a;
	integer n;
	begin
		n = 0;
		while (!(mem_req && !mem_ack && !mem_write && mem_addr == a) && n < 64) begin step; n = n + 1; end
		if (n >= 64) begin
			errors = errors + 1;
			$display("FAIL: no read of %h went out", a);
		end
	end
endtask

// Reads of the longword holding a since log entry from.
function integer reads_of;
	input [31:0] a;
	input integer from;
	integer k;
	begin
		reads_of = 0;
		for (k = from; k < log_n; k = k + 1)
			if (!log_write[k] && log_addr[k][31:2] == a[31:2]) reads_of = reads_of + 1;
	end
endfunction

// A write through port B, held until accepted, then until it has left the
// bridge and the memory has taken it.
task write_b;
	input [31:0] a;
	input  [1:0] sz;
	input [31:0] d;
	integer n;
	begin
		n = 0;
		while (wr_busy && n < 64) begin step; n = n + 1; end
		wren_b = 1'b1; address_b = a; size_b = sz; data_b = d;
		step;
		wren_b = 1'b0;
		n = 0;
		while ((wr_busy || resp_busy || mem_ack) && n < 64) begin step; n = n + 1; end
		repeat (2) step;
	end
endtask

task settle;   // long enough for the window to fill, and more
	begin repeat (24) step; end
endtask

integer mark;

initial begin
	for (i = 0; i < 4096; i = i + 1) mem[i] = {4'hC, i[11:0]};

	nreset = 0;
	repeat (2) @(posedge clk);
	#1 nreset = 1;

	// ------------------------------------------- phase 0: nothing unasked
	repeat (8) step;
	if (unasked != 0 || log_n != 0)
		fail("the bridge went to the bus before the fetch unit asked for anything");

	// ---------------------------------------------- phase 1: a plain fetch
	req(32'h0000_0400);
	want_q("phase 1's fetch of $400", img(32'h0000_0400));
	if (log_n < 1 || log_addr[0] !== 32'h0000_0400 || !log_instr[0] || log_write[0] ||
	    log_size[0] !== `AP040_SZ_L) begin
		errors = errors + 1;
		$display("FAIL: the first transaction was %h (instr=%b write=%b size=%0d), expected an instruction longword read of 00000400",
		         log_addr[0], log_instr[0], log_write[0], log_size[0]);
	end
	settle;

	// ------------------------------- phase 2: a redirect ON the acknowledge
	mark = log_n;
	req(32'h0000_1000);
	on_ack_of(32'h0000_1000);
	// The coincidence: the read of $1000 is acknowledged as $800 is asked for.
	en_a = 1'b1; address_a = 32'h0000_0800;
	step;
	en_a = 1'b0;
	if (rvalid_a) begin
		errors = errors + 1;
		$display("FAIL: the abandoned fetch of $1000 was published as valid (q = %h) although a redirect to $0800 was accepted in the same cycle",
		         q_a);
	end
	want_q("the redirected fetch of $800", img(32'h0000_0800));
	if (reads_of(32'h0000_0800, mark) != 1)
		fail("the redirect to $800 did not go out on the bus exactly once");
	settle;

	// --------------------------- phase 3: the ordinary sequential pattern
	mark = log_n;
	req(32'h0000_0400);
	want_q("phase 3's first word, $400", img(32'h0000_0400));
	req(32'h0000_0402);
	want_q("$402, the second word of the same longword", img(32'h0000_0402));
	// $404 on its prefetch's acknowledgement.
	on_ack_of(32'h0000_0404);
	req(32'h0000_0404);
	want_q("$404, asked for as its prefetch returned", img(32'h0000_0404));
	// $408 while its prefetch is on the bus.
	on_bus(32'h0000_0408);
	req(32'h0000_0408);
	want_q("$408, asked for while its read was on the bus", img(32'h0000_0408));
	req(32'h0000_040A);
	want_q("$40A", img(32'h0000_040A));
	settle;
	if (reads_of(32'h0000_0400, mark) != 1) fail("the longword at $400 was read more than once");
	if (reads_of(32'h0000_0404, mark) != 1) fail("the longword at $404 was read again although it was arriving when asked for");
	if (reads_of(32'h0000_0408, mark) != 1) fail("the longword at $408 was read again although it was on the bus when asked for");

	// ---------------------- phase 4: a window across the top of the space
	req(32'hFFFF_FFF8);
	want_q("$FFFFFFF8", img(32'hFFFF_FFF8));
	settle;
	if (reads_of(32'hFFFF_FFFC, 0) != 1 || reads_of(32'h0000_0000, 0) != 1 || reads_of(32'h0000_0004, 0) != 1)
		fail("the stream from $FFFFFFF8 did not run on through $0 and $4");
	write_b(32'hFFFF_FFFC, `AP040_SZ_W, 32'h0000_BEEF);
	req(32'hFFFF_FFFC);
	want_q("$FFFFFFFC after a word was written there", 16'hBEEF);
	settle;
	// The window is now $FFFFFFFC, $0, $4, $8.
	write_b(32'h0000_0000, `AP040_SZ_W, 32'h0000_F00D);
	req(32'h0000_0000);
	want_q("$0 after a word was written there", 16'hF00D);
	settle;
	req(32'h0000_1000);
	want_q("$1000", img(32'h0000_1000));
	req(32'h0000_0000);
	want_q("$0 as a new stream", 16'hF00D);
	settle;
	write_b(32'hFFFF_FFFE, `AP040_SZ_L, 32'h1234_5678);
	req(32'h0000_0000);
	want_q("$0 after a longword at $FFFFFFFE", 16'h5678);
	if (mem[wi(32'hFFFF_FFFE)] !== 16'h1234)
		fail("the longword at $FFFFFFFE did not reach memory as written");
	settle;

	// ------------------------------------- phase 5: controls, away from it
	req(32'h0000_0400);
	want_q("$400 for phase 5", img(32'h0000_0400));
	settle;
	mark = log_n;
	write_b(32'h0000_0FF0, `AP040_SZ_L, 32'h0BAD_0BAD);
	req(32'h0000_0400);
	want_q("$400 after a write elsewhere", img(32'h0000_0400));
	req(32'h0000_040C);
	want_q("$40C after a write elsewhere", img(32'h0000_040C));
	if (reads_of(32'h0000_0400, mark) != 0 || reads_of(32'h0000_040C, mark) != 0)
		fail("a write outside the window emptied it");
	req(32'h0000_0400);
	want_q("$400 again", img(32'h0000_0400));
	settle;
	write_b(32'h0000_03FE, `AP040_SZ_L, 32'hAAAA_BBBB);
	req(32'h0000_0400);
	want_q("$400 after a longword at $3FE", 16'hBBBB);
	settle;
	write_b(32'h0000_0408, `AP040_SZ_W, 32'h0000_1357);
	req(32'h0000_0408);
	want_q("$408 after a word was written there", 16'h1357);
	// The window's four longwords are its whole reach: the read in flight is
	// always one of them. A write just past a full window leaves it alone...
	req(32'h0000_1000);
	want_q("$1000", img(32'h0000_1000));
	req(32'h0000_0400);
	want_q("$400 for the edge", 16'hBBBB);
	settle;
	mark = log_n;
	write_b(32'h0000_0410, `AP040_SZ_W, 32'h0000_2468);
	req(32'h0000_0400);
	want_q("$400 after a word just past the window", 16'hBBBB);
	if (reads_of(32'h0000_0400, mark) != 0 || reads_of(32'h0000_0404, mark) != 0 ||
	    reads_of(32'h0000_0408, mark) != 0 || reads_of(32'h0000_040C, mark) != 0)
		fail("a word written just past a full window emptied it");
	// ...and a longword whose FIRST half is in its last longword reaches it.
	write_b(32'h0000_040E, `AP040_SZ_L, 32'hCCCC_DDDD);
	req(32'h0000_040E);
	want_q("$40E after a longword there", 16'hCCCC);
	req(32'h0000_0410);
	want_q("$410 after a longword at $40E", 16'hDDDD);

	// ---------------- phase 6: a fetch in its write's own cycle, outside the window
	req(32'h0000_0600);
	want_q("$600 for phase 6", img(32'h0000_0600));
	settle;
	mark = log_n;
	wren_b = 1'b1; address_b = 32'h0000_0500; size_b = `AP040_SZ_W; data_b = 32'h0000_7777;
	en_a   = 1'b1; address_a = 32'h0000_0500;
	step;
	wren_b = 1'b0; en_a = 1'b0;
	want_q("$500 fetched in the cycle its write was accepted", 16'h7777);
	begin : order6
		integer k, first;
		first = -1;
		for (k = mark; k < log_n; k = k + 1)
			if (first < 0 && log_addr[k][31:2] == 30'h140) first = k;
		if (first < 0 || !log_write[first])
			fail("a fetch went out ahead of the write to its longword accepted in the same cycle");
	end

	if (unasked != 0) fail("a request went out before the first en_a");

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

// A stuck run should say so rather than hang the suite.
initial begin
	#200000;
	$display("FAIL: timed out");
	$finish;
end

endmodule
