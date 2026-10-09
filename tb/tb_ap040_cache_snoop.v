// tb_ap040_cache_snoop.v -- directed bench for ap040_cache's snoop and
// port-B arbitration paths (audit findings 5.1-5.3).  These are exactly
// the paths with no other coverage: every other bench ties s_stb low.
//
// The bench drives the cache DIRECTLY: a flat memory model with a fixed
// read latency sits on the m_* side, the c_* side is exercised with
// read/write tasks, ce can be held low for chosen windows, and snoops
// are single-clk pulses INDEPENDENT of ce -- the shape cpu_wrapper's
// snoop CDC actually delivers.
//
//   T1 (5.1)  a snoop landing in a ce-frozen window must still
//             invalidate: the following read of changed memory must
//             miss and refetch.
//   T2 (5.2a) a snoop hitting the set of an in-flight fill must not be
//             undone by the fill's tag writeback, and the snooped way
//             must not be revalidated with stale data.  Swept across
//             the whole fill window.
//   T3 (5.2c) a snoop in the same cycle a lookup is accepted must not
//             let that lookup serve the killed line.  Swept.
//   T4 (5.3)  a snoop displacing a line-crossing store's invalidates
//             must not lose either set.  Swept.
//   T5 (5.4)  a bus error during a line fill (or a passed access) must
//             abandon the transfer instead of re-issuing it forever,
//             must not validate the partly-filled line, and must leave
//             the cache able to serve the exception handler's own
//             accesses.  The error is swept across all four beats.
//   T10       a completed-line sideband arriving with the first memory beat
//             must populate the fill tail locally, preserve late CPU ack,
//             and leave the completed line hitting without more bus traffic.
//   T11       an aligned write-through hit updates only its matching cached
//             longword and preserves all other ways in the same set.
//   T12       a sequential instruction hit reads the following longword ahead,
//             serves it one cycle faster (including a word at offset two),
//             chains within the line, and survives intervening D-cache traffic.
//   T13       snoops may defer a store's first-row invalidate beyond memory
//             acknowledgement; a no-gap read waits until that invalidate lands.
//
// Every test reprograms memory behind the cache and requires the next
// read to return the NEW value: a stale cached longword is the failure
// signature throughout.

`timescale 1ns/1ps

module tb_ap040_cache_snoop;

reg clk = 0;
always #5 clk = ~clk;

reg nreset = 0;
reg ce_run = 1;          // when 0, ce is forced low (frozen window)
wire ce = ce_run;

reg         cinv_req = 0;
reg         cinv_ic = 0, cinv_dc = 0;
wire        cinv_done;

reg         c_req = 0, c_write = 0, c_instr = 0, c_nocache = 0;
reg  [1:0]  c_size = 0;
reg  [31:0] c_addr = 0, c_wdata = 0;
wire        c_ack;
wire [31:0] c_rdata;

wire        m_req, m_write, m_instr;
wire  [1:0] m_size;
wire [31:0] m_addr, m_wdata;
reg  [1:0]  mem_lat = 2'd2;   // cycles before m_ack; 0 models a
                              // downstream controller-cache HIT, which is
                              // how fast this port can really answer
reg         m_ack = 0;
reg  [31:0] m_rdata = 0;
reg         m_err = 0;
reg         m_line_valid = 0;
reg  [31:4] m_line_tag = 0;
reg [127:0] m_line_data = 0;

reg         s_stb = 0;
reg  [31:0] s_addr = 0;

reg         snoop_storm = 0;
reg         s_stb_storm = 0;
// free-running chipset snoop traffic on its own driver: port B is taken
// every other cycle, which is what blitter/copper/display DMA looks like
// to this cache.  ORed into the DUT input so the snoop task keeps its own.
always @(negedge clk) begin
	if (snoop_storm) s_stb_storm <= ~s_stb_storm;
	else             s_stb_storm <= 1'b0;
end

ap040_cache dut
(
	.clk(clk), .nreset(nreset), .ce(ce),
	.ie(1'b1), .de(1'b1),
	.cinv_req(cinv_req), .cinv_ic(cinv_ic), .cinv_dc(cinv_dc),
	.cinv_done(cinv_done),
	.c_req(c_req), .c_write(c_write), .c_instr(c_instr),
	.c_size(c_size), .c_addr(c_addr), .c_wdata(c_wdata),
	.c_hint_addr(c_addr), .c_hint_instr(c_instr),
	.c_hint_ptag(c_addr[31:10]), .c_hint_match(c_req),
	.c_fc(3'd5), .c_nocache(c_nocache), .c_post_ok(1'b0),
	.c_ack(c_ack), .c_rdata(c_rdata),
	.m_req(m_req), .m_write(m_write), .m_instr(m_instr),
	.m_size(m_size), .m_addr(m_addr), .m_wdata(m_wdata),
	.m_fc(), .m_ack(m_ack), .m_rdata(m_rdata),
	.m_line_valid(m_line_valid), .m_line_tag(m_line_tag),
	.m_line_data(m_line_data), .m_err(m_err),
	.s_stb(s_stb | s_stb_storm), .s_addr(s_stb_storm ? 32'h0000_C300 : s_addr)
);

integer errors = 0;

//---------------------------------------------------------------------------
// flat memory with a 2-cycle grant: enough latency that fill beats and
// their ack cycles are deterministic for the sweep offsets below
//---------------------------------------------------------------------------
reg [31:0] mem [0:16383];   // 64KB
reg  [1:0] mlat = 0;
integer mread_count = 0;

// Fault injection: while err_arm is set, an access whose address matches
// err_addr (line-aligned, beat selected by err_beat) reports a bus error
// the way ap040_bus16_adapter does -- m_err for one qualified cycle and
// NO m_ack, ever, for that transfer.
reg         err_arm = 0;
reg  [31:0] err_addr = 0;
reg  [1:0]  err_beat = 0;
integer     err_count = 0;
wire        err_hit = err_arm && m_req &&
                      (m_addr[31:4] == err_addr[31:4]) &&
                      (m_addr[3:2] == err_beat);

always @(posedge clk) begin
	m_ack <= 0;
	m_err <= 0;
	if (m_req && ce) begin
		if (mlat != mem_lat) mlat <= mlat + 1'd1;
		else begin
			mlat <= 0;
			if (err_hit) begin
				m_err <= 1;
				err_count = err_count + 1;
			end
			else begin
				m_ack <= 1;
				if (m_write) begin
					// longword stores only in this bench
					mem[m_addr[15:2]] <= m_wdata;
				end
				else m_rdata <= mem[m_addr[15:2]];
			end
		end
	end
	else mlat <= 0;
end

always @(posedge clk) begin
	if (m_ack && m_req && !m_write) mread_count = mread_count + 1;
end

//---------------------------------------------------------------------------
// helpers
//---------------------------------------------------------------------------
task cpu_read;
	input  [31:0] a;
	output [31:0] d;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = 0; c_size = 2'b10; c_addr = a;
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: read timeout at %h", a);
			errors = errors + 1;
		end
		d = c_rdata;
		@(negedge clk);
		c_req = 0;
		@(posedge clk);
	end
endtask

task cpu_read_count_sized;
	input  [31:0] a;
	input   [1:0] sz;
	output [31:0] d;
	output integer cycles;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = 0; c_size = sz; c_addr = a;
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: counted read timeout at %h", a);
			errors = errors + 1;
		end
		d = c_rdata;
		cycles = guard;
		@(negedge clk);
		c_req = 0;
		@(posedge clk);
	end
endtask

// Two reads with NO request-low cycle between them.  cpu_read above
// drops c_req and idles a cycle after each access, which lets a pending
// CI invalidate land before the next request is looked up -- so it can
// never expose an FSM that accepts while the invalidate is still owed.
task cpu_read_btb;
	input  [31:0] a1;
	input         ci1;
	input  [31:0] a2;
	output [31:0] o1;
	output [31:0] o2;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = 0; c_size = 2'b10; c_addr = a1; c_nocache = ci1;
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk); guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: btb first read timeout at %h", a1);
			errors = errors + 1;
		end
		o1 = c_rdata;
		// present the next access immediately: c_req never falls
		@(negedge clk);
		c_addr = a2; c_nocache = 0;
		@(posedge clk);
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk); guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: btb second read timeout at %h", a2);
			errors = errors + 1;
		end
		o2 = c_rdata;
		@(negedge clk);
		c_req = 0;
		@(posedge clk);
	end
endtask

// A cache-inhibited read that HITS, immediately followed by a WRITE.
// store_inv asserts combinationally while a write waits in C_IDLE and it
// blocks ci_inv; if the FSM also refuses to accept while ci_inv_pend is
// set, the write and the invalidate block each other forever.
task cpu_ci_read_then_write;
	input [31:0] a;
	input [31:0] wa;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = 0; c_size = 2'b10; c_addr = a; c_nocache = 1;
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk); guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: CI read never completed");
			errors = errors + 1;
		end
		// present the store with no idle gap
		@(negedge clk);
		c_nocache = 0; c_write = 1; c_addr = wa; c_wdata = 32'hDEAD_5170;
		guard = 0;
		while (!(c_ack && ce) && guard < 300) begin
			@(posedge clk); guard = guard + 1;
		end
		if (guard >= 300) begin
			$display("FAIL: DEADLOCK -- store after a cache-inhibited hit never completed");
			errors = errors + 1;
		end
		@(negedge clk);
		c_req = 0; c_write = 0;
		repeat (3) @(posedge clk);
	end
endtask

task cpu_write;
	input [31:0] a;
	input [31:0] d;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = 1; c_size = 2'b10; c_addr = a; c_wdata = d;
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: write timeout at %h", a);
			errors = errors + 1;
		end
		@(negedge clk);
		c_req = 0; c_write = 0;
		@(posedge clk);
	end
endtask

task cpu_write_sized;
	input [31:0] a;
	input  [1:0] sz;
	input [31:0] d;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = 1; c_size = sz; c_addr = a; c_wdata = d;
		guard = 0;
		while (!(c_ack && ce) && guard < 200) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (guard >= 200) begin
			$display("FAIL: sized write timeout at %h", a);
			errors = errors + 1;
		end
		@(negedge clk);
		c_req = 0; c_write = 0;
		@(posedge clk);
	end
endtask

// A faulting access, driven the way the core drives one: c_req is held
// until the bus error is seen (the core samples berr on the same
// qualified edge), then dropped as the core enters exception processing.
// Returns with the cache expected to be idle again.
task cpu_access_berr;
	input [31:0] a;
	input        wr;
	integer guard;
	begin
		@(negedge clk);
		c_req = 1; c_write = wr; c_size = 2'b10; c_addr = a;
		c_wdata = 32'hBADD_0BAD;
		guard = 0;
		while (!(m_err && ce) && guard < 300) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (guard >= 300) begin
			$display("FAIL: no bus error reported for %h", a);
			errors = errors + 1;
		end
		@(negedge clk);
		c_req = 0; c_write = 0;
		@(posedge clk);
	end
endtask

// After a fault the cache must stop driving the bus: no master request
// may survive more than a couple of cycles once the core has withdrawn.
task expect_bus_idle;
	input integer tno;
	integer guard;
	begin
		guard = 0;
		while (m_req && guard < 40) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (m_req) begin
			$display("FAIL test %0d: cache still driving m_req after a bus error (livelock)",
			         tno);
			errors = errors + 1;
		end
	end
endtask

task snoop;   // one free-running clk pulse, regardless of ce
	input [31:0] a;
	begin
		@(negedge clk);
		s_stb = 1; s_addr = a;
		@(negedge clk);
		s_stb = 0;
	end
endtask

task expect_read;
	input [31:0] a;
	input [31:0] v;
	input integer tno;
	reg [31:0] d;
	begin
		cpu_read(a, d);
		if (d !== v) begin
			$display("FAIL test %0d: read %h got %h expected %h",
			         tno, a, d, v);
			errors = errors + 1;
		end
	end
endtask

integer i, off;
integer guard5;
integer line_base;
integer line_guard;
integer fill_cycles;
integer normal_cycles;
integer fast_cycles;
integer fast2_cycles;
reg [31:0] d;
reg [31:0] d2;

initial begin
	for (i = 0; i < 16384; i = i + 1) mem[i] = 32'h1111_0000 + i;
	repeat (4) @(negedge clk);
	nreset = 1;
	// let the reset sweep finish (2**ROWIW rows; 512 at the 16 KB geometry)
	repeat (600) @(posedge clk);

	//------------------------------------------------------------------
	// T1 (5.1): snoop during a frozen ce window
	//------------------------------------------------------------------
	expect_read(32'h0000_1000, mem[32'h1000>>2], 1);  // warm the line
	mem[32'h1000>>2] = 32'hAAAA_0001;                 // DMA changes memory
	ce_run = 0;                                       // clkena frozen
	repeat (3) @(negedge clk);
	snoop(32'h0000_1000);                             // arrives mid-freeze
	repeat (3) @(negedge clk);
	ce_run = 1;
	expect_read(32'h0000_1000, 32'hAAAA_0001, 1);     // must refetch

	//------------------------------------------------------------------
	// T2 (5.2a): snoop the set of an in-flight fill, swept across the
	// whole fill.  Line X (way already valid) is the snoop's target;
	// line Y (same set) is being filled.
	//------------------------------------------------------------------
	for (off = 0; off < 24; off = off + 1) begin
		cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
		@(negedge clk);
		while (!cinv_done) @(posedge clk);
		cinv_req = 0;
		@(negedge clk);

		expect_read(32'h0000_2000, mem[32'h2000>>2], 2);  // X valid
		mem[32'h2000>>2] = 32'hBBBB_0000 + off;           // X changes in memory

		// start the fill of Y and snoop X's set mid-flight
		fork
			expect_read(32'h0000_3000, mem[32'h3000>>2], 2);  // Y: same set
			begin
				repeat (off + 1) @(negedge clk);
				snoop(32'h0000_2000);
			end
		join

		expect_read(32'h0000_2000, 32'hBBBB_0000 + off, 2);   // X must refetch
	end

	//------------------------------------------------------------------
	// T3 (5.2c): snoop in the acceptance cycle of a would-be hit, swept.
	// The concurrent read may legally serve either value (the snoop is
	// unordered against it); the assertion is that the snoop's
	// invalidate survives the collision: the NEXT read must refetch.
	// (The silicon-only half of 5.2c -- mixed-port DONT_CARE producing
	// garbage tags and a false hit on a wrong way -- is not observable
	// under iverilog's deterministic old-data model; the force-miss fix
	// covers it by construction.)
	//------------------------------------------------------------------
	for (off = 0; off < 6; off = off + 1) begin
		expect_read(32'h0000_4000, mem[32'h4000>>2], 3);  // warm
		mem[32'h4000>>2] = 32'hCCCC_0000 + off;
		fork
			cpu_read(32'h0000_4000, d);   // either value is legal here
			begin
				repeat (off) @(negedge clk);
				snoop(32'h0000_4000);
			end
		join
		expect_read(32'h0000_4000, 32'hCCCC_0000 + off, 3);
	end

	//------------------------------------------------------------------
	// T4 (5.3): a line-crossing store's two set invalidates vs a snoop,
	// swept across the acceptance/pass window
	//------------------------------------------------------------------
	for (off = 0; off < 8; off = off + 1) begin
		expect_read(32'h0000_5008, mem[32'h5008>>2], 4);  // set A line
		expect_read(32'h0000_5010, mem[32'h5010>>2], 4);  // set B line
		// the store crosses from set A's line into set B's
		fork
			begin
				@(negedge clk);
				c_req = 1; c_write = 1; c_size = 2'b10;
				c_addr = 32'h0000_500E; c_wdata = 32'hDD00_0000 + off;
				while (!(c_ack && ce)) @(posedge clk);
				@(negedge clk);
				c_req = 0; c_write = 0;
				@(posedge clk);
			end
			begin
				repeat (off) @(negedge clk);
				snoop(32'h0000_5300);   // unrelated address, same bank
			end
		join
		// both lines the store touched must have been invalidated:
		// change them in memory and require refetches
		mem[32'h5008>>2] = 32'hEEEE_0000 + off;
		mem[32'h5010>>2] = 32'hFFFF_0000 + off;
		expect_read(32'h0000_5008, 32'hEEEE_0000 + off, 4);
		expect_read(32'h0000_5010, 32'hFFFF_0000 + off, 4);
	end

	//------------------------------------------------------------------
	// T5 (5.4): bus error during a fill, swept across the beats.  The
	// faulting line must be abandoned (no re-issue, no validation), and
	// the cache must keep serving -- an exception handler runs next.
	//------------------------------------------------------------------
	for (off = 0; off < 4; off = off + 1) begin
		// level-held request, exactly as the core drives it: the FSM
		// takes it when it next reaches C_IDLE, and a cache wedged by a
		// mishandled bus error simply never gets there
		cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
		@(negedge clk);
		guard5 = 0;
		while (!cinv_done && guard5 < 4000) begin
			@(posedge clk);
			guard5 = guard5 + 1;
		end
		cinv_req = 0;
		if (guard5 >= 4000) begin
			$display("FAIL test 5: cache wedged after a bus error (beat %0d): CINV never completes",
			         off);
			errors = errors + 1;
			off = 4;   // no point sweeping a wedged cache
		end
		repeat (600) @(posedge clk);

		err_arm = 1;
		err_addr = 32'h0000_6000;
		err_beat = off[1:0];
		err_count = 0;
		cpu_access_berr(32'h0000_6004, 1'b0);
		err_arm = 0;
		expect_bus_idle(5);
		if (err_count > 2) begin
			$display("FAIL test 5: faulting fill re-issued %0d times (beat %0d)",
			         err_count, off);
			errors = errors + 1;
		end

		// the handler's own accesses must work, and the abandoned line
		// must NOT have been validated: memory changes underneath it
		// and the refetch has to see the new contents
		mem[32'h6004>>2] = 32'h5EED_0000 + off;
		mem[32'h6008>>2] = 32'h5EED_1000 + off;
		expect_read(32'h0000_7000, mem[32'h7000>>2], 5);
		expect_read(32'h0000_6004, 32'h5EED_0000 + off, 5);
		expect_read(32'h0000_6008, 32'h5EED_1000 + off, 5);
	end

	// T5b: the same for a passed (write-through) access -- the store
	// faults, the cache must release the bus and stay usable
	err_arm = 1;
	err_addr = 32'h0000_6800;
	err_beat = 2'd0;
	err_count = 0;
	cpu_access_berr(32'h0000_6800, 1'b1);
	err_arm = 0;
	expect_bus_idle(5);
	expect_read(32'h0000_7100, mem[32'h7100>>2], 5);

	//------------------------------------------------------------------
	// T6 (5.4b): an aborted fill must not leave the line it was EVICTING
	// hitting over the dead fill's data.  T5 above cannot see this: it
	// invalidates the whole cache first, so the victim way is empty and
	// the abandoned beats really are unreachable.  Here the row is fully
	// populated first, exactly as it is in a running system, so the
	// refill's beats land on top of a live line whose tag and valid bit
	// survive the abort.  In NetBSD terms: a user miss bus-errors
	// mid-fill and the next supervisor hit on that row is served kernel
	// tag over user data -- the tc_windup panic, where the timehands
	// pointer came back as a user address.
	// row = addr[9:4], so these five addresses share one row and the
	// fifth fill must evict one of the four primed ways.
	expect_read(32'h0000_8000, mem[32'h8000>>2], 6);
	expect_read(32'h0000_8400, mem[32'h8400>>2], 6);
	expect_read(32'h0000_8800, mem[32'h8800>>2], 6);
	expect_read(32'h0000_8C00, mem[32'h8C00>>2], 6);

	err_arm = 1;
	err_addr = 32'h0000_9000;
	err_beat = 2'd2;          // beats 0 and 1 land before the error
	err_count = 0;
	cpu_access_berr(32'h0000_9000, 1'b0);
	err_arm = 0;
	expect_bus_idle(6);

	// every primed line must still read its OWN data (or miss and refetch
	// it); none may be served the aborted fill's beats
	expect_read(32'h0000_8000, mem[32'h8000>>2], 6);
	expect_read(32'h0000_8004, mem[32'h8004>>2], 6);
	expect_read(32'h0000_8400, mem[32'h8400>>2], 6);
	expect_read(32'h0000_8404, mem[32'h8404>>2], 6);
	expect_read(32'h0000_8800, mem[32'h8800>>2], 6);
	expect_read(32'h0000_8804, mem[32'h8804>>2], 6);
	expect_read(32'h0000_8C00, mem[32'h8C00>>2], 6);
	expect_read(32'h0000_8C04, mem[32'h8C04>>2], 6);

	//------------------------------------------------------------------
	// T7: a cache-inhibited read that HITS a resident line must
	// invalidate it as it bypasses (WinUAE dcache040: hit under
	// CACHE_DISABLE_MMU -> push+invalidate, then the uncached access).
	// Retaining the line let PRE-DMA data hit again once the mapping
	// turned cacheable -- the value below comes back A instead of C on
	// the old cache, with no bus request.
	//------------------------------------------------------------------
	expect_read(32'h0000_A000, mem[32'hA000>>2], 7);  // prime: value A
	mem[32'hA000>>2] = 32'hD11A_0002;                 // DMA writes B
	c_nocache = 1;
	expect_read(32'h0000_A000, 32'hD11A_0002, 7);     // CI read: memory B,
	c_nocache = 0;                                    // and the line dies
	mem[32'hA000>>2] = 32'hD11A_0003;                 // DMA writes C
	expect_read(32'h0000_A000, 32'hD11A_0003, 7);     // must MISS: value C

	//------------------------------------------------------------------
	// T8: back-to-back CI-then-cacheable reads with NO request-low cycle,
	// under a port-B stealing snoop swept across the completion.
	//
	// This does NOT currently discriminate: it passes whether or not the
	// FSM holds acceptance while a CI invalidate is owed, because
	// ci_inv_pend is raised on the first cycle of C_PASS and the access
	// runs to m_ack, so the invalidate always lands during the memory
	// latency.  It is kept because it is the only coverage of the no-gap
	// request path, and it would catch a future change that raised the
	// pending invalidate later (at completion rather than at lookup),
	// which is exactly when the window would become real.
	//------------------------------------------------------------------
	// A snoop must be stealing port B as the CI access completes,
	// otherwise the invalidate lands in the very cycle the FSM returns
	// to C_IDLE and the window never opens.  Sweep the snoop across the
	// completion so at least one offset collides.
	for (off = 0; off < 8; off = off + 1) begin
		cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
		@(negedge clk);
		guard5 = 0;
		while (!cinv_done && guard5 < 4000) begin
			@(posedge clk); guard5 = guard5 + 1;
		end
		cinv_req = 0;
		repeat (20) @(posedge clk);

		mem[32'hB000>>2] = 32'hB77B_0000 + off;
		expect_read(32'h0000_B000, mem[32'hB000>>2], 8);   // prime
		mem[32'hB000>>2] = 32'hB77B_0080 + off;            // DMA changes it
		fork
			cpu_read_btb(32'h0000_B000, 1'b1, 32'h0000_B000, d, d2);
			begin
				repeat (off) @(negedge clk);
				snoop(32'h0000_C300);   // unrelated row, steals port B
			end
		join
		if (d2 !== 32'hB77B_0080 + off) begin
			$display("FAIL test 8 (snoop offset %0d): back-to-back cacheable read got %h (stale line) expected %h",
			         off, d2, 32'hB77B_0080 + off);
			errors = errors + 1;
			off = 8;
		end
	end
	d = 32'hB77B_0080;   // silence the unused-check below
	d2 = 32'hB77B_0080;

	//------------------------------------------------------------------
	// T9: a store issued right after a cache-inhibited HIT must complete.
	// The CI hit owes a row invalidate; a waiting store asserts store_inv
	// combinationally, which blocks that invalidate.  If acceptance is
	// also held while the invalidate is owed, the two block each other
	// and the CPU wedges -- programs hang or loop forever.
	//------------------------------------------------------------------
	// Chipset DMA snoops CONSTANTLY on a real Amiga (blitter, copper,
	// display), and a snoop owns port B whenever it fires -- so the CI
	// invalidate cannot land during the bypassed access the way it does
	// in a quiet bench.  Drive that traffic while the pair runs.
	// Run it at BOTH memory speeds.  With three cycles of latency the
	// invalidate always lands during C_PASS and the hazard is invisible;
	// a downstream cache hit answers in one, which is when the CI
	// invalidate is still owed as the store arrives.
	expect_read(32'h0000_D000, mem[32'hD000>>2], 9);   // prime the line
	snoop_storm = 1;
	cpu_ci_read_then_write(32'h0000_D000, 32'h0000_D400);
	snoop_storm = 0;
	repeat (6) @(posedge clk);

	mem_lat = 2'd0;                                    // controller-cache hit
	expect_read(32'h0000_D800, mem[32'hD800>>2], 9);   // prime
	snoop_storm = 1;
	cpu_ci_read_then_write(32'h0000_D800, 32'h0000_DC00);
	snoop_storm = 0;
	mem_lat = 2'd2;
	repeat (6) @(posedge clk);

	//------------------------------------------------------------------
	// T10: the first ordinary beat starts a cache miss.  Model a host that
	// captured the surrounding line alongside that response, then require
	// the remaining three words to be copied locally.  The requested word
	// is fetched first and acknowledged as soon as it arrives (MC68040UM
	// 4.6.1); the host keeps the retained line valid until the fill is
	// complete, so only that one bus beat goes to memory.
	//------------------------------------------------------------------
	cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
	@(negedge clk);
	while (!cinv_done) @(posedge clk);
	cinv_req = 0;
	repeat (4) @(posedge clk);

	line_base = mread_count;
	@(negedge clk);
	c_req = 1; c_write = 0; c_size = 2'b10;
	c_addr = 32'h0000_E008; c_nocache = 0;

	// Let exactly one bus read complete, then expose its full retained line.
	line_guard = 0;
	while (!(m_ack && m_req) && line_guard < 200) begin
		@(posedge clk);
		line_guard = line_guard + 1;
	end
	if (line_guard >= 200) begin
		$display("FAIL test 10: first bus beat timed out");
		errors = errors + 1;
	end
	@(negedge clk);
	m_line_tag = 28'h0000E00;
	m_line_data = {mem[32'hE000>>2], mem[32'hE004>>2],
	               mem[32'hE008>>2], mem[32'hE00C>>2]};
	m_line_valid = 1;

	line_guard = 0;
	while (!(c_ack && ce) && line_guard < 200) begin
		@(posedge clk);
		line_guard = line_guard + 1;
	end
	if (line_guard >= 200) begin
		$display("FAIL test 10: sideband-assisted fill timed out");
		errors = errors + 1;
	end
	if (c_rdata !== mem[32'hE008>>2]) begin
		$display("FAIL test 10: sideband data %h expected %h",
		         c_rdata, mem[32'hE008>>2]);
		errors = errors + 1;
	end
	if ((mread_count - line_base) != 1) begin
		$display("FAIL test 10: assisted fill issued %0d bus reads, expected 1",
		         mread_count - line_base);
		errors = errors + 1;
	end
	@(negedge clk);
	c_req = 0;
	// the retained line stays valid until the cache has copied the rest
	line_guard = 0;
	while (dut.cst != 3'd0 && line_guard < 200) begin
		@(posedge clk);
		line_guard = line_guard + 1;
	end
	@(negedge clk);
	m_line_valid = 0;
	if ((mread_count - line_base) != 1) begin
		$display("FAIL test 10: fill completion issued %0d bus reads, expected 1",
		         mread_count - line_base);
		errors = errors + 1;
	end

	line_base = mread_count;
	expect_read(32'h0000_E000, mem[32'hE000>>2], 10);
	expect_read(32'h0000_E004, mem[32'hE004>>2], 10);
	expect_read(32'h0000_E00C, mem[32'hE00C>>2], 10);
	if (mread_count != line_base) begin
		$display("FAIL test 10: completed-line hits issued %0d extra reads",
		         mread_count - line_base);
		errors = errors + 1;
	end

	//------------------------------------------------------------------
	// T11: fill four tags in one set, update one with a CPU store, then
	// require every line to hit.  The old invalidate-on-write policy
	// discarded the whole set and generated four new line fills here.
	//------------------------------------------------------------------
	cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
	@(negedge clk);
	while (!cinv_done) @(posedge clk);
	cinv_req = 0;
	repeat (4) @(posedge clk);

	mem[32'h1000>>2] = 32'h1111_A001;
	mem[32'h2000>>2] = 32'h2222_A002;
	mem[32'h3000>>2] = 32'h3333_A003;
	mem[32'h4000>>2] = 32'h4444_A004;
	expect_read(32'h0000_1000, 32'h1111_A001, 11);
	expect_read(32'h0000_2000, 32'h2222_A002, 11);
	expect_read(32'h0000_3000, 32'h3333_A003, 11);
	expect_read(32'h0000_4000, 32'h4444_A004, 11);

	cpu_write(32'h0000_2000, 32'hBEEF_2000);
	line_base = mread_count;
	expect_read(32'h0000_2000, 32'hBEEF_2000, 11);
	expect_read(32'h0000_1000, 32'h1111_A001, 11);
	expect_read(32'h0000_3000, 32'h3333_A003, 11);
	expect_read(32'h0000_4000, 32'h4444_A004, 11);

	// Big-endian byte and word lanes merge into that same resident word.
	cpu_write_sized(32'h0000_2001, 2'b00, 32'h0000_00AA);
	expect_read(32'h0000_2000, 32'hBEAA_2000, 11);
	cpu_write_sized(32'h0000_2002, 2'b01, 32'h0000_CCDD);
	expect_read(32'h0000_2000, 32'hBEAA_CCDD, 11);

	// A failed write must not commit its speculative lookup data.
	err_arm = 1;
	err_addr = 32'h0000_2000;
	err_beat = 0;
	cpu_access_berr(32'h0000_2000, 1'b1);
	err_arm = 0;
	expect_bus_idle(11);
	expect_read(32'h0000_2000, 32'hBEAA_CCDD, 11);
	if (mread_count != line_base) begin
		$display("FAIL test 11: store-hit update caused %0d refill reads",
		         mread_count - line_base);
		errors = errors + 1;
	end

	//------------------------------------------------------------------
	// T12: a normal instruction hit pre-reads the next longword.  The
	// predicted request must complete faster without issuing memory traffic;
	// a word request at offset two still extracts the correct big-endian lane.
	// D-cache traffic between predicted instruction words is safe because the
	// cache banks have independent tags and the predictor owns a data copy.
	//------------------------------------------------------------------
	cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
	@(negedge clk);
	while (!cinv_done) @(posedge clk);
	cinv_req = 0;
	repeat (4) @(posedge clk);

	mem[32'hF000>>2] = 32'h1234_5678;
	mem[32'hF004>>2] = 32'h89AB_CDEF;
	mem[32'hF008>>2] = 32'h0BAD_F00D;
	mem[32'hF00C>>2] = 32'hCAFE_BABE;
	c_instr = 1;
	line_base = mread_count;
	cpu_read_count_sized(32'h0000_F000, 2'b10, d, fill_cycles);   // fill
	// Force the ordinary seed to require a new RAM read. Without this,
	// idle-admission completion also makes the repeated F000 a two-clock
	// hit, legitimately tying the predictor rather than testing fallback.
	@(negedge clk); c_addr = 32'h0000_F00C;
	repeat (2) @(posedge clk);
	cpu_read_count_sized(32'h0000_F000, 2'b10, d, normal_cycles); // seed
	cpu_read_count_sized(32'h0000_F006, 2'b01, d, fast_cycles);   // use/chain
	if (d !== 32'h0000_CDEF) begin
		$display("FAIL test 12: predicted word read got %h expected 0000CDEF", d);
		errors = errors + 1;
	end
	cpu_read_count_sized(32'h0000_F008, 2'b10, d, fast2_cycles);  // use/chain
	if (d !== 32'h0BAD_F00D) begin
		$display("FAIL test 12: chained long read got %h expected 0BADF00D", d);
		errors = errors + 1;
	end
	if (fast_cycles >= normal_cycles || fast2_cycles >= normal_cycles) begin
		$display("FAIL test 12: lookahead latency normal=%0d fast=%0d chained=%0d",
		         normal_cycles, fast_cycles, fast2_cycles);
		errors = errors + 1;
	end

	// An unrelated data hit may overwrite the RAM outputs, but not the
	// predictor's private copy of F00C.
	c_instr = 0;
	expect_read(32'h0000_1000, 32'h1111_A001, 12);
	// the data read's fill completes behind its early acknowledge; the
	// predicted fetch is timed from an idle cache, as the test intends
	while (dut.cst != 3'd0) @(posedge clk);
	@(negedge clk);
	c_instr = 1;
	cpu_read_count_sized(32'h0000_F00C, 2'b10, d, fast_cycles);
	if (d !== 32'hCAFE_BABE || fast_cycles >= normal_cycles) begin
		$display("FAIL test 12: prediction across data traffic got %h in %0d cycles (normal %0d)",
		         d, fast_cycles, normal_cycles);
		errors = errors + 1;
	end
	if ((mread_count - line_base) != 8) begin
		$display("FAIL test 12: instruction plus intervening data fills used %0d memory beats, expected 8",
		         mread_count - line_base);
		errors = errors + 1;
	end
	c_instr = 0;

	//------------------------------------------------------------------
	// T13: consecutive snoops defer a store's first-row invalidate past
	// its memory acknowledgement.  A no-gap read of that same row must
	// not consume the tag RAM output from the invalidate's write cycle:
	// the primitive returns OLD data, and M10K may return unknown data.
	// Exercise a snoop ending at read acceptance, during the lookup,
	// and later, at each supported memory latency.
	//------------------------------------------------------------------
	for (i = 0; i < 4; i = i + 1) begin
		mem_lat = i;
		for (off = 0; off < 4; off = off + 1) begin
			expect_read(32'h0000_E000, mem[32'hE000>>2], 10);
			@(negedge clk);
			c_req = 1; c_write = 1; c_addr = 32'h0000_E000;
			c_size = 2'b10; c_wdata = 32'h5700_0000 + (i << 8) + off;
			s_stb = 1; s_addr = 32'h0000_E310;
			guard5 = 0;
			while (!(c_ack && ce) && guard5 < 200) begin
				@(posedge clk); guard5 = guard5 + 1;
			end
			if (guard5 == 200) $fatal(1, "T13: store timed out");
			@(negedge clk);
			c_write = 0; // request remains high, same address
			fork
				begin
					guard5 = 0;
					// The store ack is gone at the preceding falling edge.
					@(posedge clk);
					while (!(c_ack && ce) && guard5 < 200) begin
						@(posedge clk); guard5 = guard5 + 1;
					end
					if (guard5 == 200) $fatal(1, "T13: read timed out");
					if (c_rdata !== c_wdata) begin
						$display("FAIL test 13 (latency %0d, snoop tail %0d): got %h expected %h",
						         i, off, c_rdata, c_wdata);
						errors = errors + 1;
					end
					@(negedge clk); c_req = 0;
				end
				begin
					repeat (off) @(negedge clk);
					s_stb = 0;
				end
			join
			repeat (6) @(posedge clk);
		end
	end
	mem_lat = 2'd2;

	//------------------------------------------------------------------
	// T14: directly exercise current-request RAM settlement, stale indices,
	// qualified physical tag changes and a ce-independent invalidation.
	//------------------------------------------------------------------
	cinv_req = 1; cinv_ic = 1; cinv_dc = 1;
	@(negedge clk);
	while (!cinv_done) @(posedge clk);
	cinv_req = 0;
	repeat (4) @(posedge clk);
	mem[32'hA000>>2] = 32'h1234_5678;
	mem[32'hA004>>2] = 32'h9ABC_DEF0;
	mem[32'hB000>>2] = 32'hDEAD_BEEF;
	c_instr = 0;
	expect_read(32'hA000, 32'h1234_5678, 14);
	while (dut.cst != 3'd0) @(posedge clk);   // the fill completes behind its early ack
	@(negedge clk); c_addr = 32'hA000;
	repeat (2) @(posedge clk);
	cpu_read_count_sized(32'hA001, 2'b00, d, fast_cycles);
	// a settled data read the hint vouches for is acknowledged in its
	// request cycle (fast_hit)
	if (d !== 32'h34 || fast_cycles != 1) begin
		$display("FAIL test 14: settled byte data=%h cycles=%0d",d,fast_cycles);
		errors = errors + 1;
	end
	cpu_read_count_sized(32'hA002, 2'b01, d, fast_cycles);
	if (d !== 32'h5678 || fast_cycles != 1) begin
		$display("FAIL test 14: settled word data=%h cycles=%0d",d,fast_cycles);
		errors = errors + 1;
	end
	@(negedge clk); c_addr = 32'hA004;
	repeat (2) @(posedge clk);
	cpu_read_count_sized(32'hA000, 2'b10, d, normal_cycles);
	if (d !== 32'h1234_5678 || normal_cycles != 3) begin
		$display("FAIL test 14: stale word must fallback data=%h cycles=%0d",d,normal_cycles);
		errors = errors + 1;
	end
	// Same index, different physical tag: never reuse A000 as B000.
	line_base = mread_count;
	cpu_read_count_sized(32'hB000, 2'b10, d, fill_cycles);
	if (d !== 32'hDEAD_BEEF || mread_count == line_base) begin
		$display("FAIL test 14: physical tag mismatch data=%h reads=%0d",d,mread_count-line_base);
		errors = errors + 1;
	end
	// Data RAM holds the old A000 across ce=0, but a snoop invalidates
	// tags and metadata in the free-running clock domain.
	@(negedge clk); c_addr = 32'hA000;
	repeat (2) @(posedge clk);
	@(negedge clk); ce_run = 0;
	mem[32'hA000>>2] = 32'h7654_3210;
	snoop(32'hA000);
	repeat (2) @(posedge clk);
	@(negedge clk); ce_run = 1;
	line_base = mread_count;
	cpu_read_count_sized(32'hA000, 2'b10, d, fill_cycles);
	if (d !== 32'h7654_3210 || mread_count == line_base) begin
		$display("FAIL test 14: frozen-ce snoop data=%h reads=%0d",d,mread_count-line_base);
		errors = errors + 1;
	end
	$display("EARLY_ADMISSION_TESTS_COMPLETE");

	//-------------------------------------------------- line-crossing reads
	// A longword at offset 13-15 or a word at 15 takes word 3 of one line
	// and word 0 of the next.  Cold first line: bypass.  First line hit,
	// second missing: the second line is filled and the pair acknowledged
	// on its first beat.  Both hit: served from the cache in one extra
	// cycle with no memory read.  Set wrap: the next line carries the
	// next tag.  A store to the second line is seen by the next read.
	mem[32'hC00C>>2] = 32'h1122_3344;
	mem[32'hC010>>2] = 32'h5566_7788;
	// cold first line: bypassed to memory (this bench's flat model does not
	// split a misaligned access, so only the completion and the memory
	// access are checked)
	line_base = mread_count;
	cpu_read_count_sized(32'hC00E, 2'b10, d, fill_cycles);
	if (mread_count == line_base) begin
		$display("FAIL test 15a: cold crossing read did not reach memory reads=%0d", mread_count-line_base);
		errors = errors + 1;
	end
	cpu_read_count_sized(32'hC00C, 2'b10, d, normal_cycles);   // warm the first line
	if (d !== 32'h1122_3344) begin
		$display("FAIL test 15b: warm read data=%h", d);
		errors = errors + 1;
	end
	repeat (24) @(posedge clk);                                 // let the fill land
	line_base = mread_count;
	cpu_read_count_sized(32'hC00E, 2'b10, d, fill_cycles);     // first hit, second fills
	if (d !== 32'h3344_5566 || mread_count == line_base) begin
		$display("FAIL test 15c: crossing read with second-line fill data=%h reads=%0d", d, mread_count-line_base);
		errors = errors + 1;
	end
	repeat (24) @(posedge clk);                                 // the pair was acknowledged on beat 0; let beats 1-3 land
	line_base = mread_count;
	cpu_read_count_sized(32'hC010, 2'b10, d, normal_cycles);   // the second line is now cached
	if (d !== 32'h5566_7788 || mread_count != line_base || normal_cycles != 3) begin
		$display("FAIL test 15d: second line not cached after the crossing fill data=%h reads=%0d cycles=%0d", d, mread_count-line_base, normal_cycles);
		errors = errors + 1;
	end
	line_base = mread_count;
	cpu_read_count_sized(32'hC00D, 2'b10, d, normal_cycles);   // both hit
	if (d !== 32'h2233_4455 || mread_count != line_base || normal_cycles > 5) begin
		$display("FAIL test 15e: double-hit crossing long data=%h reads=%0d cycles=%0d", d, mread_count-line_base, normal_cycles);
		errors = errors + 1;
	end
	cpu_read_count_sized(32'hC00F, 2'b01, d, normal_cycles);   // word at 15
	if (d[15:0] !== 16'h4455 || mread_count != line_base || normal_cycles > 5) begin
		$display("FAIL test 15f: double-hit crossing word data=%h reads=%0d cycles=%0d", d, mread_count-line_base, normal_cycles);
		errors = errors + 1;
	end
	cpu_read_count_sized(32'hC00F, 2'b10, d, normal_cycles);   // long at 15
	if (d !== 32'h4455_6677 || mread_count != line_base) begin
		$display("FAIL test 15g: double-hit crossing long at 15 data=%h reads=%0d", d, mread_count-line_base);
		errors = errors + 1;
	end
	cpu_write(32'hC010, 32'h9999_8888);                          // store into the second line
	repeat (8) @(posedge clk);
	line_base = mread_count;
	cpu_read_count_sized(32'hC00E, 2'b10, d, normal_cycles);
	if (d !== 32'h3344_9999) begin
		$display("FAIL test 15h: crossing read after a store to the second line data=%h", d);
		errors = errors + 1;
	end
	// set wrap: line 1FF0 is the last set, its successor 2000 the first set with the next tag
	mem[32'h1FFC>>2] = 32'hAABB_CCDD;
	mem[32'h2000>>2] = 32'hEEFF_0011;
	cpu_read_count_sized(32'h1FFC, 2'b10, d, normal_cycles);
	repeat (24) @(posedge clk);
	cpu_read_count_sized(32'h2000, 2'b10, d, normal_cycles);
	repeat (24) @(posedge clk);
	line_base = mread_count;
	cpu_read_count_sized(32'h1FFE, 2'b10, d, normal_cycles);
	if (d !== 32'hCCDD_EEFF || mread_count != line_base || normal_cycles > 5) begin
		$display("FAIL test 15i: set-wrap crossing read data=%h reads=%0d cycles=%0d", d, mread_count-line_base, normal_cycles);
		errors = errors + 1;
	end
	$display("CROSSING_READ_TESTS_COMPLETE");

	if (errors == 0) $display("ALL TESTS PASSED");
	else $display("TEST FAILED with %0d errors", errors);
	$finish;
end

initial begin
	#4000000;
	$display("FAIL: global timeout");
	$finish;
end

endmodule
