//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25, caches stage B)   //
//                                                                          //
// tb_ap040_pipe_icache.v - the instruction cache, at the IMU's ports       //
//                                                                          //
// ap040_pipe_imu.v and ap040_pipe_membus.v, driven at the fetch stage's    //
// port A (and port B for a write), untranslated, against a memory whose    //
// latency each test sets and which can answer one read with a bus error.  //
// What a program cannot see, because it is timing or order: which         //
// longwords a miss reads and in what order, what a hit costs and what it   //
// delivers, which way a fill takes, what CINV and CPUSH leave, what an     //
// error on a fill's later beat does, and what a read the window abandons  //
// costs. Each word of memory names its own address ($Cnnn, nnn the word    //
// index), as in tb_ap040_pipe_busredirect.v.                               //
//                                                                          //
//   1. A miss at a line's third longword reads it first, then the fourth,  //
//      the first and the second, each once (MC68040UM 4.1, 4.6.1); the     //
//      line is valid only after the fourth. The first read goes in the     //
//      cycle the miss is seen: on the bus three cycles after the request's.//
//   2. A hit is answered from the cache: the request, the set read, the    //
//      compare, the answer -- rvalid three cycles after the request's     //
//      cycle -- and no bus read.                                          //
//   3. Hits stream: each hit answers with the half-line (the longword and  //
//      the next when they share it) and the window's next read goes in    //
//      the cycle it is answered, so eight longwords of cached code arrive  //
//      in ten cycles.                                                      //
//   4. A CPU write does not reach the cache (4.5): a write into a cached    //
//      line leaves the line, and a fetch of the written longword returns   //
//      the old word with IE set -- and memory's with IE clear.            //
//   5. Replacement: a fill takes the first invalid way of its set; with     //
//      all four valid, the way the counter names -- the counter advancing  //
//      with every half-line looked up and once more after naming a way     //
//      (4.1). Six lines of one set: two replacements.                     //
//   6. CINV: a line, a page (4 KB), everything; each leaves the rest. A    //
//      page takes a row a set, 64 rows; done is one clock long. A CINV     //
//      waits for a fill under way, and then takes its line too.           //
//   7. A bus error on a fill's later beat abandons the line: no fault     //
//      reaches a fetch that did not ask for that longword, the line is not //
//      valid, and a fetch of the longword errored, now asked for, reads    //
//      the line again from that longword. An error on the beat a fetch     //
//      waits for faults that fetch; a fetch waiting in the line for        //
//      another beat is not faulted, and reads the line again from its own. //
//   8. A read the window abandons before its lookup's result is answered   //
//      at once and fills nothing.                                          //
//   9. IE set while a fetch's bus read is out: the read is still answered  //
//      to the window. IE cleared during a fill: the fill completes, the    //
//      line is valid, and the fetches after it read single longwords.      //
//  10. An instruction TTR with CM 1x: single longword reads, no fill.       //
//  11. The line read buffer: during a fill on a slow memory, the window's  //
//      reads of the line's later longwords are answered as each arrives.  //
//  12. Snoops (Table 4-3, V5/V6): another master's write invalidates the   //
//      line holding its address and no other, in whichever way; a snoop of //
//      a line being read keeps it from being made valid; and a snoop at    //
//      any moment across a fill, from its first read to after its last,    //
//      leaves that line invalid -- including the cycle the fill writes its  //
//      set's tags, when the tag copy's row cannot be read. Snoops back to  //
//      back are each taken.                                                //
//  13. A CINVP row and a snoop clearing the same set in the same cycle:    //
//      both lines go. Swept across the page's 64 rows.                    //
//  14. A miss invalidating its victim as a snoop clears another line of    //
//      that set: the victim must go too -- its data is overwritten half a  //
//      line at a time, and when the fill is abandoned (an error on its     //
//      third beat) the victim's tag is still there. Swept across the miss. //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_icache;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

reg  [31:0] address_a = 32'd0;
reg         en_a      = 1'b0;
wire [15:0] q_a, q_a2;
wire        rvalid_a, rflt_a, rflt_a_bus;
reg         pf_inval  = 1'b0;

reg  [31:0] address_b = 32'd0;
reg  [31:0] data_b    = 32'd0;
reg         wren_b    = 1'b0;
wire        wr_busy;

reg         ic_en = 1'b0;
reg  [31:0] itt0 = 32'd0;
reg         cm_req = 1'b0, cm_ic = 1'b1;
reg   [1:0] cm_scope = 2'b11;
reg  [31:0] cm_addr = 32'd0;
wire        cm_done;
reg         sn_req = 1'b0;
reg  [31:0] sn_addr = 32'd0;

wire        mem_req, mem_write, mem_instr;
wire  [1:0] mem_size;
wire [31:0] mem_addr, mem_wdata;
wire  [2:0] mem_fc;
reg         mem_ack   = 1'b0;
reg         mem_flt   = 1'b0;
reg  [31:0] mem_rdata = 32'd0;

wire        f_req, f_sup, f_free, f_ack, f_flt, f_flt_bus, f_w_accept;
wire [31:0] f_addr;
wire [29:0] f_w_sla;

ap040_pipe_imu u_imu (
	.clk(clk), .nreset(nreset),
	.address_a(address_a), .en_a(en_a), .q_a(q_a), .q_a2(q_a2), .rvalid_a(rvalid_a),
	.rflt_a(rflt_a), .rflt_a_bus(rflt_a_bus),
	.sup(1'b1), .pf_inval(pf_inval), .quiesce(1'b0),
	.ic_en(ic_en), .itt0(itt0), .itt1(32'd0),
	.cm_req(cm_req), .cm_ic(cm_ic), .cm_scope(cm_scope), .cm_addr(cm_addr), .cm_done(cm_done),
	.sn_req(sn_req), .sn_addr(sn_addr),
	.pf_xlat(1'b0), .x_req(), .x_addr(), .x_sup(), .x_pass(1'b0), .x_flt(1'b0), .x_pa(32'd0), .x_cm(2'b00),
	.pk_addr(), .pk_sup(), .pk_hit(1'b0), .pk_pa(32'd0), .pk_cm(2'b00),
	.f_req(f_req), .f_addr(f_addr), .f_sup(f_sup), .f_free(f_free),
	.f_ack(f_ack), .f_rdata(mem_rdata), .f_flt(f_flt), .f_flt_bus(f_flt_bus),
	.w_accept(f_w_accept), .w_sla(f_w_sla)
);
ap040_pipe_membus u_bus (
	.clk(clk), .nreset(nreset),
	.f_req(f_req), .f_addr(f_addr), .f_sup(f_sup), .f_free(f_free),
	.f_ack(f_ack), .f_flt(f_flt), .f_flt_bus(f_flt_bus),
	.w_accept(f_w_accept), .w_sla(f_w_sla),
	.address_b(address_b), .la_b(address_b), .data_b(data_b), .wren_b(wren_b),
	.size_b(`AP040_SZ_L), .rd_b(1'b0), .wr_busy(wr_busy), .wr_busy_w(),
	.q_b(), .rvalid_b(), .sup_b(1'b1),
	.fc_ovr(1'b0), .fc_ovr_val(3'd0),
	.mem_req(mem_req), .mem_write(mem_write), .mem_instr(mem_instr),
	.mem_size(mem_size), .mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_fc(mem_fc),
	.mem_ack(mem_ack), .mem_rdata(mem_rdata),
	.mem_flt(mem_flt), .mem_flt_bus(mem_flt), .mem_pass(mem_req), .wr_sync(1'b0), .wr_drop(!wren_b),
	.rflt_b(), .wflt(), .idle(), .flt_bus(),
	.xlat_e(1'b0), .xlat_p(1'b0), .pb_req(), .pb_addr(), .pb_fc(), .pb_done(1'b0), .pb_mmusr(32'd0), .flt_ma(),
	.rx(1'b0), .rx_addr(32'd0), .rx_size(2'd0), .rx_fc(3'd0)
);

integer errors = 0;
integer i, n, t0, lg;

// ---------------------------------------------------------------- memory
// 8 KB of words; addresses alias onto it by their low thirteen bits.
reg [15:0] mem [0:4095];
function [11:0] wi;
	input [31:0] a;
	begin wi = a[12:1]; end
endfunction
function [31:0] img;   // the longword an address holds before anything writes it
	input [31:0] a;
	begin img = {4'hC, wi(a), 4'hC, wi(a + 32'd2)}; end
endfunction

// Every transaction the memory accepts, in order.
reg  [31:0] log_addr  [0:4095];
reg         log_write [0:4095];
integer     log_n = 0;

// mem_lat cycles between accepting a request and answering it; a read of
// berr_addr, while armed, is answered with a bus error instead (once).
integer mem_lat = 1, mem_cnt = 0;
reg     mem_busy = 1'b0;
reg     berr_arm = 1'b0;
reg [31:0] berr_addr = 32'd0;
always @(posedge clk) begin
	mem_ack <= 1'b0;
	mem_flt <= 1'b0;
	if (!nreset) begin
		mem_busy <= 1'b0;
	end else if (mem_req && !mem_ack && !mem_flt) begin
		if (!mem_busy) begin
			mem_busy <= 1'b1; mem_cnt = mem_lat;
			log_addr[log_n]  = mem_addr;
			log_write[log_n] = mem_write;
			log_n = log_n + 1;
		end else if (mem_cnt > 0) mem_cnt = mem_cnt - 1;
		else begin
			mem_busy <= 1'b0;
			if (!mem_write && berr_arm && mem_addr == berr_addr) begin
				mem_flt  <= 1'b1;
				berr_arm <= 1'b0;
			end else begin
				mem_ack <= 1'b1;
				if (mem_write) begin
					mem[wi(mem_addr)]         <= mem_wdata[31:16];
					mem[wi(mem_addr + 32'd2)] <= mem_wdata[15:0];
				end else
					mem_rdata <= {mem[wi(mem_addr)], mem[wi(mem_addr + 32'd2)]};
			end
		end
	end
end

// ---------------------------------------------------------------- probes
// The unit's lookups (each reads a half-line) and its fills' starts, for
// test 5's model of the replacement counter.
integer lookups = 0, replaced = 0;
always @(posedge clk) if (nreset) begin
	if (u_imu.lk_go) lookups = lookups + 1;
end
function lvalid;   // the line holding physical address a is valid
	input [31:0] a;
	integer w;
	begin
		lvalid = 1'b0;
		for (w = 0; w < 4; w = w + 1)
			if (u_imu.vld[a[9:4]][w] && u_imu.u_arr.tags.mem[a[9:4]][w*22 +: 22] == a[31:10])
				lvalid = 1'b1;
	end
endfunction
integer cm_pulses = 0;
always @(posedge clk) if (nreset && cm_done) cm_pulses = cm_pulses + 1;
// 12: the sweep must reach a snoop of the line being read, and a snoop
// whose tag-copy read met that set's tag write
integer sn_fills = 0, sn_junks = 0, merged = 0;
always @(posedge clk) if (nreset) begin
	if (u_imu.sn_fill) sn_fills = sn_fills + 1;
	if (u_imu.sn_v2 && u_imu.sn_junk2) sn_junks = sn_junks + 1;
	// 13, 14: a snoop's clear and another change to the same set together
	if (u_imu.sn_v2 && u_imu.o_v && (u_imu.o_set == u_imu.sn_l2[5:0]) && (u_imu.sn_clr != 4'd0) &&
	    (u_imu.cm_rdv || u_imu.lk_fill))
		merged = merged + 1;
end


// ---------------------------------------------------------------- helpers
task step; begin @(posedge clk); #1; end endtask
task fail;
	input [8*100-1:0] msg;
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
// the answer to the latest request, within 400 cycles; n counts the cycles
task wait_q;
	begin
		n = 0;
		while (!rvalid_a && n < 400) begin step; n = n + 1; end
		if (!rvalid_a) fail("a fetch was never answered");
	end
endtask
task fetch_want;   // request a, and want its word
	input [31:0] a;
	input [8*48-1:0] what;
	begin
		req(a);
		wait_q;
		if (rflt_a || q_a !== (a[1] ? img(a) : img(a) >> 16) & 16'hFFFF) begin
			errors = errors + 1;
			$display("FAIL: %0s: fetch of %h gave %h flt %b, want %h", what, a, q_a, rflt_a,
			         (a[1] ? img(a) : img(a) >> 16) & 16'hFFFF);
		end
	end
endtask
task quiet;   // until nothing has moved on the bus for 30 cycles
	integer k, last;
	begin
		k = 0; last = log_n;
		while (k < 30) begin
			step;
			if (log_n != last || mem_req) begin k = 0; last = log_n; end else k = k + 1;
		end
	end
endtask
task cinv;   // CINV/CPUSH IC: scope 01 line, 10 page, 11 all
	input [1:0] scope;
	input [31:0] a;
	integer k;
	begin
		cm_scope = scope; cm_addr = a; cm_ic = 1'b1; cm_req = 1'b1;
		k = 0;
		while (!cm_done && k < 400) begin step; k = k + 1; end
		n = k;
		if (!cm_done) fail("CINV never finished");
		step;              // the CPU's acknowledgement lands the cycle after done
		cm_req = 1'b0;
		step;              // ...and the request is down a clock before another:
		                   // a unit takes each request once
	end
endtask
task snoop;   // one cycle of sn_req
	input [31:0] a;
	begin
		sn_addr = a; sn_req = 1'b1;
		step;
		sn_req = 1'b0;
	end
endtask
task write32;   // port B, accepted when wr_busy is low
	input [31:0] a;
	input [31:0] d;
	begin
		address_b = a; data_b = d; wren_b = 1'b1;
		#0;
		while (wr_busy) step;
		step;
		wren_b = 1'b0;
	end
endtask
task want_log;   // the reads from log index lg on are these four, in order
	input [31:0] a0, a1, a2, a3;
	input [8*48-1:0] what;
	begin
		if (log_n < lg + 4 || log_addr[lg] !== a0 || log_addr[lg + 1] !== a1 ||
		    log_addr[lg + 2] !== a2 || log_addr[lg + 3] !== a3) begin
			errors = errors + 1;
			$display("FAIL: %0s: reads %h %h %h %h, want %h %h %h %h", what,
			         log_addr[lg], log_addr[lg + 1], log_addr[lg + 2], log_addr[lg + 3], a0, a1, a2, a3);
		end
	end
endtask

integer k, w, fe, rep_m, cyc, d;
initial begin
	for (i = 0; i < 4096; i = i + 1) mem[i] = {4'hC, i[11:0]};
	repeat (4) step;
	nreset = 1'b1;
	repeat (2) step;
	ic_en = 1'b1;

	//------------------------------------------------------------- test 1
	mem_lat = 3;
	lg = log_n;
	req(32'h0000_0108);
	k = 1;                   // cycles since the request's
	while (!mem_req && k < 20) begin step; k = k + 1; end
	if (k != 3) begin
		$display("    the fill's first read on the bus %0d cycle(s) after the request's", k);
		fail("1: a miss's first read did not go in the cycle the miss was seen");
	end
	wait_q;
	if (rflt_a || q_a !== 16'hC084) fail("1: a miss did not return its word");
	quiet;
	want_log(32'h108, 32'h10C, 32'h100, 32'h104, "1: the line, from the longword asked for");
	for (k = lg; k < lg + 4; k = k + 1) if (log_write[k]) fail("1: a fill wrote");
	if (!lvalid(32'h100)) fail("1: the line is not valid after its fill");

	//------------------------------------------------------------- test 2
	lg = log_n;
	req(32'h0000_0100);      // a redirect below the window, into the cached line
	wait_q;
	if (n != 2) begin
		$display("    rvalid %0d cycle(s) after the request's", n + 1);
		fail("2: a hit was not answered three cycles after its request");
	end
	if (q_a !== 16'hC080) fail("2: a hit returned the wrong word");
	quiet;
	for (k = lg; k < log_n; k = k + 1)
		if (log_addr[k][31:4] == 28'h10) fail("2: a hit read the bus");

	//------------------------------------------------------------- test 3
	// $100-$11F cached (test 1's prefetch filled $110 too); fetch like the
	// fetch unit, the next longword the cycle an answer is in.
	if (!lvalid(32'h110)) fail("3: the next line was not filled by test 1's prefetch (the test no longer tests)");
	req(32'h0000_0180);      // somewhere else first, so the window is empty
	wait_q;
	quiet;
	t0 = $time;
	for (k = 0; k < 8; k = k + 1) begin
		req(32'h0000_0100 + k * 4);
		wait_q;
		if (q_a !== img(32'h100 + k * 4) >> 16) fail("3: a streamed fetch returned the wrong word");
	end
	cyc = ($time - t0) / 10;
	$display("test 3: eight cached longwords in %0d cycles", cyc);
	if (cyc > 10) fail("3: cached code did not stream at a longword a cycle");

	//------------------------------------------------------------- test 4
	quiet;
	write32(32'h0000_0104, 32'h1234_5678);
	quiet;
	if (mem[wi(32'h104)] !== 16'h1234) fail("4: the write did not reach memory");
	if (!lvalid(32'h100)) fail("4: a CPU write disturbed the cached line");
	req(32'h0000_0104);
	wait_q;
	if (q_a !== 16'hC082) fail("4: a fetch after a write into a cached line did not return the cached word");
	ic_en = 1'b0;
	req(32'h0000_0180);
	wait_q;
	req(32'h0000_0104);
	wait_q;
	if (q_a !== 16'h1234) fail("4: with IE clear, a fetch did not return memory's word");
	ic_en = 1'b1;
	mem[wi(32'h104)] = 16'hC082; mem[wi(32'h106)] = 16'hC083;
	quiet;

	//------------------------------------------------------------- test 5
	cinv(2'b11, 32'd0);
	for (k = 0; k < 64; k = k + 1) if (u_imu.vld[k] !== 4'd0) fail("5: CINVA left a valid line");
	// set 5: $050, $450, $850, $C50, $1050, $1450 -- and each fill's way
	for (k = 0; k < 6; k = k + 1) begin
		// the counter the rule gives: every lookup so far, and once more
		// after each way it named
		rep_m = (lookups + replaced) % 4;
		fe = 0;
		req(32'h0000_0050 + k * 32'h400);
		// the fill's first read: its way is chosen
		while (!(u_imu.fl_act) && fe < 400) begin step; fe = fe + 1; end
		w = u_imu.fl_way;
		if (k < 4 && w != k) begin
			$display("    line %0d took way %0d", k, w);
			fail("5: a fill with an invalid way free did not take the first");
		end
		if (k >= 4) begin
			// the counter as it stood when the miss named the way: every
			// lookup up to and including this one, and one per way named
			rep_m = (lookups + replaced) % 4;
			if (w != rep_m) begin
				$display("    line %0d took way %0d, the counter says %0d (%0d lookups, %0d replacements)",
				         k, w, rep_m, lookups, replaced);
				fail("5: a full set's fill did not take the way the counter names");
			end
			replaced = replaced + 1;
		end
		wait_q;
		quiet;
		if (k == 4) begin
			for (i = 0; i < 4; i = i + 1)
				if (i != w && !lvalid(32'h0000_0050 + i * 32'h400)) fail("5: a line other than the replaced one was lost");
			if (lvalid(32'h0000_0050 + w * 32'h400)) fail("5: the replaced line is still valid");
			if (!lvalid(32'h1050)) fail("5: the fifth line is not valid");
		end
	end
	if (!lvalid(32'h1450)) fail("5: the sixth line is not valid");

	//------------------------------------------------------------- test 6
	// lines in both 4 KB pages of the 8 KB memory
	cinv(2'b11, 32'd0);
	fetch_want(32'h0000_0200, "6: fill"); quiet;
	fetch_want(32'h0000_0600, "6: fill"); quiet;
	fetch_want(32'h0000_1200, "6: fill"); quiet;
	fetch_want(32'h0000_1600, "6: fill"); quiet;
	if (!lvalid(32'h200) || !lvalid(32'h600) || !lvalid(32'h1200) || !lvalid(32'h1600))
		fail("6: the lines were not filled (the test no longer tests)");
	k = cm_pulses;
	cinv(2'b01, 32'h0000_0608);        // a line: $600's
	if (n > 4) fail("6: CINVL took more than a row");
	if (lvalid(32'h600)) fail("6: CINVL left its line");
	if (!lvalid(32'h200) || !lvalid(32'h1200) || !lvalid(32'h1600)) fail("6: CINVL took another line");
	cinv(2'b10, 32'h0000_1FFC);        // a page: $1000-$1FFF
	if (n < 64) fail("6: CINVP did not read every set");
	if (lvalid(32'h1200) || lvalid(32'h1600)) fail("6: CINVP left a line of its page");
	if (!lvalid(32'h200)) fail("6: CINVP took a line of another page");
	if (cm_pulses != k + 2) fail("6: CINV's done was not one clock per operation");
	// the page's and line's lines are fetched from the bus again
	lg = log_n;
	fetch_want(32'h0000_1200, "6: refetch"); quiet;
	if (log_n == lg) fail("6: a line CINVP invalidated was answered without a bus read");
	lg = log_n;
	fetch_want(32'h0000_0200, "6: kept"); quiet;
	for (k = lg; k < log_n; k = k + 1)
		if (log_addr[k][31:4] == 28'h20) fail("6: a line no CINV named was read again");
	// a CINV while a fill is under way waits for it, then takes its line
	mem_lat = 12;
	req(32'h0000_0A00);
	while (!u_imu.fl_act) step;
	lg = log_n;
	cm_scope = 2'b11; cm_addr = 32'd0; cm_ic = 1'b1; cm_req = 1'b1;
	k = 0;
	while (!cm_done && k < 400) begin step; k = k + 1; end
	if (u_imu.fl_act || log_n < lg + 3) fail("6: CINV finished with a fill under way");
	step; cm_req = 1'b0; step;
	if (lvalid(32'h0A00)) fail("6: CINVA left the line filled while it waited");
	wait_q;
	mem_lat = 1;
	quiet;

	//------------------------------------------------------------- test 7
	// A later beat's error, nobody asking for it: fetch $300 and nothing
	// more; the window reads ahead into $308, whose read errors.
	cinv(2'b11, 32'd0);
	mem_lat = 3;
	berr_arm = 1'b1; berr_addr = 32'h0000_0308;
	lg = log_n;
	fetch_want(32'h0000_0300, "7: the line's first longword");
	quiet;
	if (berr_arm) fail("7: the error was never met (the test no longer tests)");
	if (rflt_a) fail("7: an error on a beat nobody asked for faulted the fetch");
	if (lvalid(32'h300)) fail("7: the line was left valid after an error on a beat");
	// now asked for: the line is read again, from $308
	lg = log_n;
	fetch_want(32'h0000_0308, "7: the errored longword, asked for");
	quiet;
	want_log(32'h308, 32'h30C, 32'h300, 32'h304, "7: the line again, from the longword asked for");
	if (!lvalid(32'h300)) fail("7: the line read again is not valid");
	// the beat a fetch waits for: its error is the fetch's
	cinv(2'b11, 32'd0);
	berr_arm = 1'b1; berr_addr = 32'h0000_0340;
	req(32'h0000_0340);
	wait_q;
	if (!rflt_a || !rflt_a_bus || q_a !== 16'h4AFC) fail("7: an error on the longword a fetch waits for did not fault it");
	quiet;
	if (lvalid(32'h340)) fail("7: a line whose first beat errored is valid");
	// a fetch waiting in the line for another beat: $3A8 first, so the fill
	// reads $3A8 $3AC $3A0 $3A4; once $3AC is in, the fetch unit asks for
	// $3A4 -- the fill's last -- and $3A0's read errors
	cinv(2'b11, 32'd0);
	mem_lat = 6;
	berr_arm = 1'b1; berr_addr = 32'h0000_03A0;
	fetch_want(32'h0000_03A8, "7: the second line's third longword");
	while (!(mem_ack && mem_addr == 32'h3AC)) step;
	step;
	lg = log_n;
	req(32'h0000_03A4);
	wait_q;
	if (rflt_a || q_a !== 16'hC1D2) fail("7: a fetch waiting in the line for another beat took that beat's error");
	quiet;
	if (berr_arm) fail("7: $3A0's error was never met (the test no longer tests)");
	k = lg;
	while (k < log_n && log_addr[k] !== 32'h3A4) k = k + 1;
	if (k == log_n || log_addr[k] !== 32'h3A4 || (k + 1 < log_n && log_addr[k + 1] !== 32'h3A8))
		fail("7: the line was not read again from the longword waited for");
	if (!lvalid(32'h3A0)) fail("7: the line read again is not valid");
	mem_lat = 1;

	//------------------------------------------------------------- test 8
	cinv(2'b11, 32'd0);
	mem_lat = 3;
	lg = log_n;
	req(32'h0000_0480);          // a miss...
	req(32'h0000_0200);          // ...abandoned the next cycle
	wait_q;
	quiet;
	for (k = lg; k < log_n; k = k + 1)
		if (log_addr[k][31:4] == 28'h48) fail("8: a read abandoned before its lookup filled its line");
	if (lvalid(32'h480)) fail("8: an abandoned read's line is valid");
	mem_lat = 1;

	//------------------------------------------------------------- test 9
	cinv(2'b11, 32'd0);
	ic_en = 1'b0;
	mem_lat = 8;
	req(32'h0000_0500);
	while (!mem_req) step;       // the direct read is out...
	ic_en = 1'b1;                // ...and IE set
	wait_q;
	if (rflt_a || q_a !== 16'hC280) fail("9: a bus read out when IE was set was not answered");
	quiet;
	// IE cleared during a fill
	cinv(2'b11, 32'd0);
	req(32'h0000_0540);
	while (!u_imu.fl_act) step;
	ic_en = 1'b0;
	wait_q;
	if (q_a !== 16'hC2A0) fail("9: the fetch whose fill IE was cleared under was not answered");
	quiet;
	if (!lvalid(32'h540)) fail("9: a fill IE was cleared under was not completed");
	lg = log_n;
	fetch_want(32'h0000_0580, "9: after IE cleared");
	quiet;
	if (log_n == lg || log_addr[lg] !== 32'h580) fail("9: the fetch after IE cleared did not read its own longword");
	if (lvalid(32'h580)) fail("9: a fetch with IE clear filled a line");
	ic_en = 1'b1;
	mem_lat = 1;

	//------------------------------------------------------------- test 10
	cinv(2'b11, 32'd0);
	itt0 = 32'h0000_C040;        // $00xxxxxx, either mode, CM 10
	lg = log_n;
	fetch_want(32'h0000_0608, "10: inhibited");
	quiet;
	if (log_addr[lg] !== 32'h608 || (log_n > lg + 1 && log_addr[lg + 1] == 32'h600))
		fail("10: an inhibited fetch did not read its own longword alone");
	if (lvalid(32'h600)) fail("10: an inhibited fetch filled a line");
	itt0 = 32'd0;

	//------------------------------------------------------------- test 11
	cinv(2'b11, 32'd0);
	mem_lat = 10;
	fetch_want(32'h0000_0700, "11: the line's first");
	// the window reads $704 next: answered when it arrives, not at the fill's end
	while (!(mem_ack && mem_addr == 32'h704)) step;
	step;
	if (!u_imu.fl_act) fail("11: the fill ended with its second longword (the test no longer tests)");
	if (!((30'h1C1 - u_imu.pf_base) < {27'd0, u_imu.pf_cnt}))
		fail("11: a longword the fill brought was not handed to the window as it arrived");
	req(32'h0000_0704);
	wait_q;
	if (n > 1 || q_a !== 16'hC382) fail("11: the fetch of a longword already in did not hit the window");
	mem_lat = 1;
	quiet;

	//------------------------------------------------------------- test 12
	cinv(2'b11, 32'd0);
	// four lines of set $30: $300 $700 $B00 $F00
	for (k = 0; k < 4; k = k + 1) begin fetch_want(32'h0000_0300 + k * 32'h400, "12: fill"); quiet; end
	for (k = 0; k < 4; k = k + 1)
		if (!lvalid(32'h0000_0300 + k * 32'h400)) fail("12: a line was not filled (the test no longer tests)");
	snoop(32'h0000_0B08);             // $B00's line, in its way
	repeat (4) step;
	if (lvalid(32'h0B00)) fail("12: a snooped line is still valid");
	if (!lvalid(32'h0300) || !lvalid(32'h0700) || !lvalid(32'h0F00)) fail("12: a snoop took another line of its set");
	lg = log_n;
	fetch_want(32'h0000_0B00, "12: refetch");
	quiet;
	if (log_n == lg) fail("12: a snooped line was answered without a bus read");
	// back to back: $300 and $700 in consecutive cycles
	sn_addr = 32'h0000_0300; sn_req = 1'b1; step;
	sn_addr = 32'h0000_0704; step;
	sn_req = 1'b0;
	repeat (4) step;
	if (lvalid(32'h0300) || lvalid(32'h0700)) fail("12: snoops back to back were not both taken");
	if (!lvalid(32'h0F00)) fail("12: back-to-back snoops took a third line");
	// across a fill: on a slow memory, the snoop d cycles after the miss
	mem_lat = 3;
	for (d = 0; d < 40; d = d + 1) begin
		cinv(2'b11, 32'd0);
		req(32'h0000_1240 + d * 16);
		repeat (d) step;
		snoop(32'h0000_1240 + d * 16 + 4);
		wait_q;
		quiet;
		if (lvalid(32'h0000_1240 + d * 16)) begin
			$display("    snooped %0d cycle(s) after the miss", d);
			fail("12: a line snooped across its fill is valid");
		end
	end
	mem_lat = 1;
	$display("test 12: %0d snoop(s) of a line being read, %0d at a tag write", sn_fills, sn_junks);
	if (sn_fills == 0) fail("12: no snoop met a fill under way (the test no longer tests)");
	if (sn_junks == 0) fail("12: no snoop's row read met a fill's tag write (the test no longer tests)");

	//------------------------------------------------------------- test 13
	// set $24: $0240 in page 0, $1240 in page 1. CINVP page 0 takes $0240;
	// the snoop takes $1240. Set $24's row is compared 36 rows in.
	merged = 0;
	for (d = 30; d < 44; d = d + 1) begin
		cinv(2'b11, 32'd0);
		fetch_want(32'h0000_0240, "13: fill"); quiet;
		fetch_want(32'h0000_1240, "13: fill"); quiet;
		if (!lvalid(32'h0240) || !lvalid(32'h1240)) fail("13: the lines were not filled (the test no longer tests)");
		cm_scope = 2'b10; cm_addr = 32'h0000_0000; cm_ic = 1'b1; cm_req = 1'b1;
		fork
			begin
				k = 0;
				while (!cm_done && k < 400) begin step; k = k + 1; end
				step; cm_req = 1'b0; step;
			end
			begin
				repeat (d) step;
				sn_addr = 32'h0000_1240; sn_req = 1'b1; step; sn_req = 1'b0;
			end
		join
		repeat (4) step;
		if (lvalid(32'h0240)) begin
			$display("    snooped %0d cycle(s) into the CINVP", d);
			fail("13: CINVP's line survived a snoop of the same set in the same cycle");
		end
		if (lvalid(32'h1240)) begin
			$display("    snooped %0d cycle(s) into the CINVP", d);
			fail("13: the snooped line survived a CINVP row of the same set in the same cycle");
		end
	end
	if (merged == 0) fail("13: no snoop met the CINVP's row of its set (the test no longer tests)");

	//------------------------------------------------------------- test 14
	// set $2A: four lines $02A0 $06A0 $0AA0 $0EA0 in ways 0-3, then $12A0
	// misses. Its lookup advances the counter, which then names the victim;
	// the snoop takes the line in the way after it, d cycles after the
	// fetch's request (the victim is cleared two cycles after it). $12A0's
	// third beat errors: its first half-line is already in the victim's way.
	merged = 0;
	mem_lat = 2;
	for (d = 0; d < 6; d = d + 1) begin
		cinv(2'b11, 32'd0);
		for (k = 0; k < 4; k = k + 1) begin fetch_want(32'h0000_02A0 + k * 32'h400, "14: fill"); quiet; end
		req(32'h0000_0180); wait_q; quiet;   // the window away from the set
		berr_arm = 1'b1; berr_addr = 32'h0000_12A8;
		w = (u_imu.rep + 1) % 4;              // the victim-to-be
		fork
			req(32'h0000_12A0);
			begin
				repeat (d) step;
				snoop(32'h0000_02A0 + ((w + 1) % 4) * 32'h400);
			end
		join
		wait_q;
		quiet;
		if (berr_arm) fail("14: the fill's error was never met (the test no longer tests)");
		berr_arm = 1'b0;
		if (lvalid(32'h0000_02A0 + w * 32'h400)) begin
			$display("    d=%0d: the victim, way %0d, is still valid", d, w);
			fail("14: a miss's victim survived a snoop of its set in the same cycle");
		end
		if (lvalid(32'h0000_02A0 + ((w + 1) % 4) * 32'h400)) fail("14: the snooped line survived");
		for (k = 0; k < 4; k = k + 1) begin
			req(32'h0000_0180); wait_q;
			fetch_want(32'h0000_02A0 + k * 32'h400, "14: a line of the set after the abandoned fill");
			quiet;
		end
	end
	mem_lat = 1;
	if (merged == 0) fail("14: no snoop met a victim's clear in the same cycle (the test no longer tests)");

	repeat (20) step;
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("%0d CHECK(S) FAILED", errors);
	$finish;
end

initial begin
	#2000000;
	$display("FAIL: timed out");
	$finish;
end

endmodule
