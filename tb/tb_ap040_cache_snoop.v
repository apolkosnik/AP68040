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
//   T11       write-through with update-on-hit: a store that fits in a
//             longword and hits leaves the line valid with the store
//             merged in, at every size and lane; a missing store does
//             not allocate; a snoop on the store's set anywhere between
//             acceptance and the ack leaves the line dead, a snoop on
//             another set in that window does not stop the merge.
//             Swept at every memory latency.
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
// CE_DIV > 1 runs the cache the way P2 runs it on silicon: on the fast
// clock with a divided enable (Minimig.sv: clk_114, CORE_DIV 4).  Every
// bench task already waits on (c_ack && ce), so the sweeps below, which
// step snoops by FAST cycles, then cover every phase of the enable.
parameter CE_DIV = 1;
// POST_OK 1 posts every store: the core is acknowledged on capture and the
// write drains behind it.  Every bench task waits on (c_ack && ce), so with
// posting the task returns while the drain is still on the bus; the tests
// that look at memory afterwards (expect_read, the write-through checks)
// therefore exercise exactly the ordering the drain has to keep.
parameter POST_OK = 0;
reg [1:0] ce_ph = 0;
always @(posedge clk) ce_ph <= (ce_ph == CE_DIV - 1) ? 2'd0 : ce_ph + 2'd1;
wire ce = ce_run && (CE_DIV == 1 || ce_ph == 2'd0);

// Fault injection on the lookup guard.  Plusargs are matched by PREFIX, so
// the names must not be prefixes of each other.  Meaningful only with
// CE_DIV > 1 (at 1 every cycle is "right after a tick").
//
//   +inj_acc_settle   the acceptance-cycle term forced blind in the fast
//                     cycle right after each tick -- the cycle in which the
//                     live address translation has not settled at 114 MHz.
//                     Must PASS: this is the multicycle argument in
//                     Minimig.sdc, and if it is wrong T3 fails here.
//   +inj_acc_whole    the same term blind for the WHOLE acceptance wait.
//   +inj_look_whole   the compare-cycle term blind for the WHOLE C_LOOK
//                     window.
//
// Measured under -DSNOOP_MIXED_X (the silicon-faithful tag-row model; with
// the default old-data answer a snoop's invalidate is simply read back a
// cycle later and neither guard is ever load-bearing):
//
//                        CE_DIV 1        CE_DIV 4
//   guard intact         PASS            PASS
//   +inj_acc_whole       FAIL            PASS
//   +inj_look_whole      PASS            FAIL
//   +inj_acc_settle      FAIL (=whole)   PASS
//
// Each term is load-bearing in one regime and redundant in the other, and
// the reasons are the cycle counts.  At divide 1 the compare is the cycle
// after the read is issued, so a snoop in the acceptance cycle collides
// with that read and the compare sees garbage: the ACCEPTANCE term is what
// forces the miss.  At divide 4 the compare is four cycles later and the
// tag row is re-read every cycle, so that collision is cleaned before it
// is looked at -- the acceptance term can be arbitrarily late, which is
// what Minimig.sdc's relaxation of core|mem_addr -> look_snooped needs --
// while a snoop inside the four-cycle C_LOOK window can still land after
// the last re-read the compare will see: the COMPARE term is what forces
// the miss there, and blinding it lets a garbage victim way through to
// C_TAGW (the +inj_look_whole failure is that chain).  So the split is
// not just safe but required, and the relaxation is proven by a control
// that fails where the regime is different.  The suite carries the two
// failing controls as neglegs, one per divide.
//   +inj_fillguard    blind the WRITEBACK protection as well -- tag_we's
//                     !fill_snooped && !snoop_fill_row (ap040_cache.v:439),
//                     which is a separate guard from the lookup terms above.
//                     For diagnosis only: it exists to tell overlapping
//                     protection apart from a redundant term.  Corruption
//                     that appears only when this AND a lookup term are
//                     blinded shows the two overlap on that sequence; it does
//                     not show the lookup term independently necessary.
// +inj_dma_snoop: drop the snoop's invalidate for exactly as long as a
// posted store is pending -- the failure T15 exists to catch, "the chipset
// wrote memory and the CPU never learned of it because it was busy
// draining a write of its own".  T15 must FAIL under this; if it does not,
// it is not testing what it claims to.  The rest of the suite is not
// disturbed, because outside a pending store sb_v is low and the
// invalidate is untouched.
reg inj_dma_snoop = 0;
reg inj_acc_settle = 0, inj_acc_whole = 0, inj_look_whole = 0, inj_fillguard = 0;
initial begin
	inj_dma_snoop  = $test$plusargs("inj_dma_snoop");
	inj_acc_settle = $test$plusargs("inj_acc_settle");
	inj_acc_whole  = $test$plusargs("inj_acc_whole");
	inj_look_whole = $test$plusargs("inj_look_whole");
	inj_fillguard  = $test$plusargs("inj_fillguard");
end
initial if ($test$plusargs("inj_fillguard")) begin
	force dut.fill_snooped   = 1'b0;
	force dut.snoop_fill_row = 1'b0;
end
reg ce_d = 0;
always @(posedge clk) ce_d <= ce;
always @(ce_d)
	if (inj_acc_settle) begin if (ce_d) force dut.snoop_look_row_acc = 1'b0; else release dut.snoop_look_row_acc; end
always @(dut.sb_v)
	if (inj_dma_snoop) begin if (dut.sb_v) force dut.snoop_wr = 1'b0; else release dut.snoop_wr; end
always @(dut.rd_accept)
	if (inj_acc_whole) begin if (dut.rd_accept) force dut.snoop_look_row_acc = 1'b0; else release dut.snoop_look_row_acc; end
always @(dut.cst)
	if (inj_look_whole) begin if (dut.cst == 3'd1) force dut.snoop_look_row_look = 1'b0; else release dut.snoop_look_row_look; end

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
	.c_fc(3'd5), .c_nocache(c_nocache),
	.c_post_ok(POST_OK[0]),
	.sb_busy(),
	.c_ack(c_ack), .c_rdata(c_rdata),
	.m_req(m_req), .m_write(m_write), .m_instr(m_instr),
	.m_size(m_size), .m_addr(m_addr), .m_wdata(m_wdata),
	.m_fc(), .m_ack(m_ack), .m_rdata(m_rdata), .m_err(m_err),
	.s_stb(s_stb | s_stb_storm), .s_addr(s_stb_storm ? 32'h0000_C300 : s_addr)
);

integer errors = 0;

//---------------------------------------------------------------------------
// flat memory with a 2-cycle grant: enough latency that fill beats and
// their ack cycles are deterministic for the sweep offsets below
//---------------------------------------------------------------------------
reg [31:0] mem [0:16383];   // 64KB
reg  [1:0] mlat = 0;

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

// The acknowledge (and the error) is a LEVEL held until the cache samples
// it on an enable edge, then dropped -- the real adapter's contract ("a
// level acknowledge ... until chip-select drops", ap040_bus16_adapter.v).
// It used to be a one-cycle pulse in the fast cycle after the latency
// count completed; the count advances only on enable cycles, so at CE_DIV
// 4 that pulse always landed in a non-enable cycle, the cache (sampling
// under ce) never saw it, and every read timed out.  At CE_DIV 1 the
// level is consumed on the very next edge, so it is the same one-cycle
// pulse the existing tests were written against.
always @(posedge clk) begin
	if ((m_ack || m_err) && ce) begin
		m_ack <= 0;                       // consumed on a qualified edge
		m_err <= 0;
	end
	else if (m_req && !m_ack && !m_err) begin
		if (ce) begin
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
						// lanes as the bus adapter places them: the
						// data is right-aligned by size
						case (m_size)
							2'b00: case (m_addr[1:0])
								2'd0: mem[m_addr[15:2]][31:24] <= m_wdata[7:0];
								2'd1: mem[m_addr[15:2]][23:16] <= m_wdata[7:0];
								2'd2: mem[m_addr[15:2]][15:8]  <= m_wdata[7:0];
								default: mem[m_addr[15:2]][7:0] <= m_wdata[7:0];
							endcase
							2'b01: if (m_addr[1]) mem[m_addr[15:2]][15:0]  <= m_wdata[15:0];
							       else           mem[m_addr[15:2]][31:16] <= m_wdata[15:0];
							default: mem[m_addr[15:2]] <= m_wdata;
						endcase
					end
					else m_rdata <= mem[m_addr[15:2]];
				end
			end
		end
	end
	else if (!m_req) mlat <= 0;
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
		// hold the withdrawal until an ENABLE edge has sampled it: the core's
		// mem_req is a ce-gated register, so on silicon a request can never
		// drop for less than one enable period; a one-fast-cycle gap here is
		// something the real core cannot produce, and at CE_DIV 4 it left the
		// cache's err_hold set forever (no tick ever saw c_req low)
		while (!ce) @(posedge clk);
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
		// hold the withdrawal until an ENABLE edge has sampled it: the core's
		// mem_req is a ce-gated register, so on silicon a request can never
		// drop for less than one enable period; a one-fast-cycle gap here is
		// something the real core cannot produce, and at CE_DIV 4 it left the
		// cache's err_hold set forever (no tick ever saw c_req low)
		while (!ce) @(posedge clk);
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
		repeat (3 * CE_DIV) @(posedge clk);
	end
endtask

// memory READ acknowledges consumed by the cache: unchanged across an
// access proves that access was served from a line
integer mreads = 0;
always @(posedge clk) if (m_ack && ce && !m_write) mreads = mreads + 1;
integer mwrites = 0;
always @(posedge clk) if (m_ack && ce && m_write) mwrites = mwrites + 1;
// +trace: one line per enable edge (and per snoop) with the state the
// tests reason about.  Off by default; it is how a wrong-line hit gets
// read without a waveform.
reg trace_on = 0;
initial trace_on = $test$plusargs("trace");
always @(posedge clk) if (trace_on && (ce || s_stb))
	$display("TR t=%0t ce=%b cst=%0d sb_v=%b | c_req=%b w=%b nc=%b a=%h ack=%b rd=%h | look_hit=%b v=%b hw=%0d lsn=%b r_way=%0d r_row=%h | m_req=%b m_w=%b m_a=%h m_ack=%b | tag_we=%b cd_we=%b inv=%b idx=%h s_stb=%b s_a=%h",
	         $time, ce, dut.cst, dut.sb_v, c_req, c_write, c_nocache, c_addr, c_ack, c_rdata,
	         dut.look_hit, dut.tag_q[91:88], dut.hit_way, dut.look_snooped, dut.r_way, dut.r_row,
	         dut.m_req, dut.m_write, dut.m_addr, m_ack, dut.tag_we, dut.cd_we, dut.inv_we, dut.inv_idx, s_stb, s_addr);

// The drain owns the master side by itself (ap040_cache.v, "THE DRAIN IS
// NOT A STATE").  While sb_v is set nothing else may be on the bus, and
// the FSM may sit in C_PASS with it only for the store's own merge cycle.
// A fill under the drain would read memory the store has not reached; a
// second store would either overtake the first or steal its acknowledge.
reg pass_seen = 0;
always @(posedge clk) if (ce) begin
	if (dut.sb_v && dut.m_req && !dut.m_write) begin
		$display("FAIL invariant: a read issued on the master side under a draining store (addr %h)", dut.m_addr);
		errors = errors + 1;
	end
	if (dut.sb_v && (dut.m_addr !== dut.sb_addr || dut.m_wdata !== dut.sb_wdata)) begin
		$display("FAIL invariant: master side not the buffered store while it drains");
		errors = errors + 1;
	end
	if (dut.cst == 3'd4 && dut.sb_v) begin
		$display("FAIL invariant: C_FILL entered under a draining store");
		errors = errors + 1;
	end
	if (dut.cst == 3'd6 && dut.sb_v && !dut.pass_first) begin
		$display("FAIL invariant: C_PASS held under a draining store beyond its merge cycle");
		errors = errors + 1;
	end
end
task cpu_write_sz;
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
			$display("FAIL: write timeout at %h", a);
			errors = errors + 1;
		end
		@(negedge clk);
		c_req = 0; c_write = 0; c_size = 2'b10;
		while (!ce) @(posedge clk);
		@(posedge clk);
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
		// hold the withdrawal until an ENABLE edge has sampled it: the core's
		// mem_req is a ce-gated register, so on silicon a request can never
		// drop for less than one enable period; a one-fast-cycle gap here is
		// something the real core cannot produce, and at CE_DIV 4 it left the
		// cache's err_hold set forever (no tick ever saw c_req low)
		while (!ce) @(posedge clk);
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
		// hold the withdrawal until an ENABLE edge has sampled it: the core's
		// mem_req is a ce-gated register, so on silicon a request can never
		// drop for less than one enable period; a one-fast-cycle gap here is
		// something the real core cannot produce, and at CE_DIV 4 it left the
		// cache's err_hold set forever (no tick ever saw c_req low)
		while (!ce) @(posedge clk);
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

// -DSNOOP_MIXED_X: give the tag row silicon's mixed-port read-during-write
// (DONT_CARE -> X in the sim model) so the collision the lookup guard
// exists for is OBSERVABLE.  Under the model's default old-data answer a
// snoop's invalidate is simply read back a cycle later and the guard is
// never load-bearing -- which is why an injection on it could not fail.
`ifdef SNOOP_MIXED_X
defparam dut.ctag_ram.rdw_mixed = "DONT_CARE";

// Directed adversarial don't-care row.
//
// A don't-care word only matters if acting on it is UNSAFE, and a random
// word is safe: with TAGW 22 it misses, the lookup refills, and the answer
// is right.  That is why an LFSR poison leaves the negative controls
// passing.  This builds the unsafe row on purpose -- the tag the lookup is
// asking for, valid, in a way that does not hold this line -- so a lookup
// that acts on it HITS, takes the wrong way, and returns that way's word.
//
// row = { rr[1:0], valid[3:0], tag3, tag2, tag1, tag0 }, TAGW 22.
// The way is chosen as the lowest that is not the legitimate holder of
// a_tag in the row actually stored, so hit_way is unambiguous and the data
// behind it is never this line's.  Every data way is seeded with a marker
// at reset, so an unsafe lookup that reaches a way this line never filled
// returns POISON_DAT rather than a plausible word; a way filled for some
// OTHER line returns that line's data, which expect_read also catches.
localparam [31:0] POISON_DAT = 32'hBADD_0BAD;
function [1:0] wrong_way;
	input [93:0] row;
	input [21:0] tg;
	begin
		if      (!(row[88] && row[21:0]  == tg)) wrong_way = 2'd0;
		else if (!(row[89] && row[43:22] == tg)) wrong_way = 2'd1;
		else if (!(row[90] && row[65:44] == tg)) wrong_way = 2'd2;
		else                                     wrong_way = 2'd3;
	end
endfunction
wire [93:0] real_row = dut.ctag_ram.mem[dut.a_row];
// +trace_wb: every writeback attempt while the permuted row is armed, with
// what allowed or blocked it.  The claim that the writeback protection is
// what saves T12 needs this, not an inference from the source.
integer wb_n = 0;
always @(posedge clk) if (nreset && perm_en && dut.cst == 3'd5 &&
                          $test$plusargs("trace_wb") && wb_n < 40) begin
	wb_n = wb_n + 1;
	$display("WB t=%0t C_TAGW tag_we=%b fill_snooped=%b snoop_fill_row=%b -> %s",
	         $time, dut.tag_we, dut.fill_snooped, dut.snoop_fill_row,
	         dut.tag_we ? "WROTE" : "blocked");
end
wire  [1:0] pway     = wrong_way(real_row, dut.a_tag);
// T12 drives the collided row itself; see that test for why.
reg        perm_en  = 1'b0;
reg [93:0] perm_row = 94'd0;
always @(*) begin
	dut.ctag_ram.poison_en  = 1'b1;
	dut.ctag_ram.poison_row = perm_en ? perm_row :
		{2'b00, (4'd1 << pway),
		 (pway == 2'd3) ? dut.a_tag : 22'h3F_FFFC,
		 (pway == 2'd2) ? dut.a_tag : 22'h3F_FFFD,
		 (pway == 2'd1) ? dut.a_tag : 22'h3F_FFFE,
		 (pway == 2'd0) ? dut.a_tag : 22'h3F_FFFF};
end

// No separate guard assertion.  One was written and removed: it flagged any
// request whose row a snoop touched anywhere in the window, and that is not
// the contract -- the row is re-read every cycle, so a snoop landing before
// the last re-read the compare sees is harmless, which is exactly why
// +inj_look_whole is safe at divide 1.  Narrowing it to "the compare used a
// collided row" converges on the RTL's own expression, which proves nothing.
//
// The end-to-end check is the contract: with the directed row above, a
// lookup that acts on a don't-care row returns the wrong way's word, and
// expect_read catches it.  That tests the protection, not its spelling.
`endif
// A posted store is acknowledged before it reaches memory.  A test that
// then changes memory "behind the cache" means AFTER the store is visible
// there, so it waits for the drain first -- otherwise the store drains over
// the change and the cache correctly returns the store, failing the test's
// own assumption rather than the cache.
// A chipset/DMA write, as one modelled action: the backing memory changes
// and the snoop follows it.  Every test above instead sets mem[] by hand at
// a moment of its own choosing and pulses snoop separately -- always with
// the CPU quiescent -- so a DMA write landing while a POSTED CPU store has
// not yet reached memory was never exercised.  That window is open in the
// shipped image (POST_STORES = 1), which is why this is worth testing
// whatever happens to the store queue.
//
// skew delays the snoop behind the memory write the way the real path does:
// the chipset write lands in the controller, and the wrapper's CDC
// (chip_snoop_tgl -> two flops -> s_stb) delivers the invalidate a few
// cycles later.  skew 0 is the simultaneous case.
task dma_write;
	input [31:0] a;
	input [31:0] d;
	input integer skew;
	integer k;
	begin
		mem[a[15:2]] = d;
		for (k = 0; k < skew; k = k + 1) @(negedge clk);
		snoop(a);
	end
endtask

task wait_drain;
	integer guard;
	begin
		guard = 0;
		while (dut.sb_v && guard < 400) begin
			@(posedge clk);
			guard = guard + 1;
		end
		if (guard >= 400) begin
			$display("FAIL: posted store never drained");
			errors = errors + 1;
		end
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
integer hit_cycles;
integer mr0;
reg [31:0] d;
reg [31:0] d2;
reg [31:0] model;

initial begin
	for (i = 0; i < 16384; i = i + 1) mem[i] = 32'h1111_0000 + i;
`ifdef SNOOP_MIXED_X
	// Marker behind every way, so an unsafe lookup that reaches a way this
	// line never filled returns something unmistakable rather than a zero
	// that could be mistaken for ordinary uninitialised memory.  Legitimate
	// fills overwrite the entries they use, so this does not disturb a
	// correct run.
	for (i = 0; i < 512; i = i + 1) begin
		dut.cdata_way0.mem[i] = POISON_DAT;
		dut.cdata_way1.mem[i] = POISON_DAT;
		dut.cdata_way2.mem[i] = POISON_DAT;
		dut.cdata_way3.mem[i] = POISON_DAT;
	end
`endif
	repeat (4) @(negedge clk);
	nreset = 1;
	// let the reset sweep finish.  It walks 128 rows at one per ENABLE
	// cycle (ap040_cache.v C_SWEEP exits to C_IDLE at sweep_cnt == 127),
	// so a fixed count of fast clocks is only right at CE_DIV 1 -- at 4 it
	// left the first read mid-sweep.  Wait on the state instead.
	@(posedge clk);
	while (dut.cst == 3'd7) @(posedge clk);
	repeat (4) @(posedge clk);

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
	// under the deterministic old-data model; the force-miss fix
	// covers it by construction.)
	//------------------------------------------------------------------
	for (off = 0; off < 6 * CE_DIV; off = off + 1) begin
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
				// hold the withdrawal until an ENABLE edge has sampled it: the core's
				// mem_req is a ce-gated register, so on silicon a request can never
				// drop for less than one enable period; a one-fast-cycle gap here is
				// something the real core cannot produce, and at CE_DIV 4 it left the
				// cache's err_hold set forever (no tick ever saw c_req low)
				while (!ce) @(posedge clk);
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
		@(posedge clk);
		while (dut.cst == 3'd7) @(posedge clk);   // sweep runs at one row per ENABLE cycle
		repeat (4) @(posedge clk);

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
		repeat (20 * CE_DIV) @(posedge clk);

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
	repeat (6 * CE_DIV) @(posedge clk);

	mem_lat = 2'd0;                                    // controller-cache hit
	expect_read(32'h0000_D800, mem[32'hD800>>2], 9);   // prime
	snoop_storm = 1;
	cpu_ci_read_then_write(32'h0000_D800, 32'h0000_DC00);
	snoop_storm = 0;
	mem_lat = 2'd2;
	repeat (6 * CE_DIV) @(posedge clk);

	//------------------------------------------------------------------
	// T10: consecutive snoops defer a store's first-row invalidate past
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
			if (guard5 == 200) $fatal(1, "T10: store timed out");
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
					if (guard5 == 200) $fatal(1, "T10: read timed out");
					if (c_rdata !== c_wdata) begin
						$display("FAIL test 10 (latency %0d, snoop tail %0d): got %h expected %h",
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
			repeat (6 * CE_DIV) @(posedge clk);
		end
	end
	mem_lat = 2'd2;

	//------------------------------------------------------------------
	// T11: write-through with update-on-hit (see the header).  Sets are
	// address bits 9:4: $F200 shares set $20 with $7200 on another line,
	// $F310 (set $31) is unrelated to $7300 (set $30).
	//------------------------------------------------------------------
	for (i = 0; i < 4; i = i + 1) begin
		mem_lat = i;
		// (a) a longword store that hits: the next read is a hit and
		// returns the stored value, memory was written through
		expect_read(32'h0000_7000, mem[32'h7000>>2], 11);
		mr0 = mreads;
		cpu_write(32'h0000_7004, 32'hA5A5_0000 + i);
		expect_read(32'h0000_7004, 32'hA5A5_0000 + i, 11);
		if (mreads != mr0) begin
			$display("FAIL test 11a (latency %0d): read after a hitting store went to memory", i);
			errors = errors + 1;
		end
		if (mem[32'h7004>>2] !== 32'hA5A5_0000 + i) begin
			$display("FAIL test 11a (latency %0d): memory not written through", i);
			errors = errors + 1;
		end
		// (b) byte and word stores merge into the line, checked against
		// a software model of the longword; every read must hit
		model = mem[32'h7008>>2];
		for (off = 0; off < 4; off = off + 1) begin
			cpu_write_sz(32'h0000_7008 + off, 2'b00, 32'h0000_0080 + (i << 4) + off);
			case (off)
				0: model[31:24] = 8'h80 + (i << 4) + off;
				1: model[23:16] = 8'h80 + (i << 4) + off;
				2: model[15:8]  = 8'h80 + (i << 4) + off;
				default: model[7:0] = 8'h80 + (i << 4) + off;
			endcase
			mr0 = mreads;
			expect_read(32'h0000_7008, model, 11);
			if (mreads != mr0) begin
				$display("FAIL test 11b (latency %0d): read after a byte store at lane %0d went to memory", i, off);
				errors = errors + 1;
			end
		end
		cpu_write_sz(32'h0000_7008, 2'b01, 32'h0000_C000 + i);
		model[31:16] = 16'hC000 + i;
		cpu_write_sz(32'h0000_700A, 2'b01, 32'h0000_D000 + i);
		model[15:0] = 16'hD000 + i;
		mr0 = mreads;
		expect_read(32'h0000_7008, model, 11);
		if (mreads != mr0) begin
			$display("FAIL test 11b (latency %0d): read after word stores went to memory", i);
			errors = errors + 1;
		end
		if (mem[32'h7008>>2] !== model) begin
			$display("FAIL test 11b (latency %0d): memory %h, model %h", i, mem[32'h7008>>2], model);
			errors = errors + 1;
		end
		// (c) a store that misses must not allocate: the read fetches
		mr0 = mreads;
		cpu_write(32'h0000_7100 + (i << 4), 32'h3C3C_0000 + i);
		expect_read(32'h0000_7100 + (i << 4), 32'h3C3C_0000 + i, 11);
		if (mreads == mr0) begin
			$display("FAIL test 11c (latency %0d): a missing store allocated a line", i);
			errors = errors + 1;
		end
		// (d) a snoop on the store's set, swept from acceptance past the
		// ack: the line must be dead afterwards, whether the merge was
		// suppressed or the snoop cleared the row after it
		for (off = 0; off < 8; off = off + 1) begin
			expect_read(32'h0000_7200, mem[32'h7200>>2], 11);
			fork
				cpu_write(32'h0000_7204, 32'h5E5E_0000 + (i << 8) + off);
				begin
					repeat (off) @(negedge clk);
					snoop(32'h0000_F200);
				end
			join
			wait_drain;
			mem[32'h7204>>2] = 32'h6F6F_0000 + (i << 8) + off;
			expect_read(32'h0000_7204, 32'h6F6F_0000 + (i << 8) + off, 11);
		end
		// (e) a snoop on another set in the same window must not stop
		// the merge: the read after it hits with the stored value
		for (off = 0; off < 8; off = off + 1) begin
			expect_read(32'h0000_7300, mem[32'h7300>>2], 11);
			fork
				cpu_write(32'h0000_7304, 32'h7A7A_0000 + (i << 8) + off);
				begin
					repeat (off) @(negedge clk);
					snoop(32'h0000_F310);
				end
			join
			mr0 = mreads;
			expect_read(32'h0000_7304, 32'h7A7A_0000 + (i << 8) + off, 11);
			if (mreads != mr0) begin
				$display("FAIL test 11e (latency %0d, snoop at %0d): an unrelated snoop stopped the merge", i, off);
				errors = errors + 1;
			end
		end
	end
	mem_lat = 2'd2;

	//------------------------------------------------------------------
	// T14: the drain is not a state.  With posting, a store is
	// acknowledged at capture and drains by itself; the cache must keep
	// serving HITS under it, and hold exactly the accesses that need the
	// master side -- a miss, a bypass, the next store -- until the write
	// has landed, so nothing can overtake it.  Each part runs at every
	// memory latency, and the invariant monitors above watch the bus
	// throughout.  Without posting nothing here can be observed (the store
	// returns after its own drain) and the section is skipped, not faked.
	if (POST_OK[0]) begin
		for (i = 0; i < 4; i = i + 1) begin
			mem_lat = i;
			// (a) a hit under the drain is served while the store is still
			// on the bus: it must take exactly the cycles a plain hit takes,
			// touch no memory, and the drain must still land.  (Whether the
			// drain is still running at the acknowledge depends on the
			// latency; the hit's own latency does not, and that is the
			// claim.)
			expect_read(32'h0000_7400, mem[32'h7400>>2], 14);   // warm it
			@(negedge clk);
			c_req = 1; c_write = 0; c_size = 2'b10; c_addr = 32'h0000_7400;
			hit_cycles = 0;
			while (!(c_ack && ce) && hit_cycles < 200) begin
				@(posedge clk); hit_cycles = hit_cycles + 1;
			end
			@(negedge clk); c_req = 0;
			while (!ce) @(posedge clk);
			@(posedge clk);
			cpu_write(32'h0000_7500, 32'hA14A_0000 + i);        // returns at capture
			mr0 = mreads;
			@(negedge clk);
			c_req = 1; c_write = 0; c_size = 2'b10; c_addr = 32'h0000_7400;
			guard5 = 0;
			while (!(c_ack && ce) && guard5 < 200) begin
				@(posedge clk); guard5 = guard5 + 1;
			end
			if (guard5 >= 200) begin
				$display("FAIL test 14a (latency %0d): hit under the drain never acknowledged", i);
				errors = errors + 1;
			end
			else begin
				if (guard5 != hit_cycles) begin
					$display("FAIL test 14a (latency %0d): the hit under the drain took %0d cycles, a plain hit %0d", i, guard5, hit_cycles);
					errors = errors + 1;
				end
				if (c_rdata !== mem[32'h7400>>2]) begin
					$display("FAIL test 14a (latency %0d): hit under the drain read %h expected %h", i, c_rdata, mem[32'h7400>>2]);
					errors = errors + 1;
				end
			end
			@(negedge clk); c_req = 0;
			while (!ce) @(posedge clk);
			@(posedge clk);
			if (mreads != mr0) begin
				$display("FAIL test 14a (latency %0d): the hit went to memory", i);
				errors = errors + 1;
			end
			wait_drain;
			if (mem[32'h7500>>2] !== 32'hA14A_0000 + i) begin
				$display("FAIL test 14a (latency %0d): the store never landed (%h)", i, mem[32'h7500>>2]);
				errors = errors + 1;
			end
			// (b) a miss under the drain waits for it and then refills from
			// memory that already holds the store: store to a line that is
			// NOT resident (no allocate on a write miss), read it back at
			// once, and require the stored value.  Also a miss on another
			// line, which must simply complete.
			cpu_write(32'h0000_7604, 32'hB14B_0000 + i);
			expect_read(32'h0000_7604, 32'hB14B_0000 + i, 14);
			cpu_write(32'h0000_7704, 32'hC14C_0000 + i);
			expect_read(32'h0000_7800, mem[32'h7800>>2], 14);
			wait_drain;
			if (mem[32'h7704>>2] !== 32'hC14C_0000 + i) begin
				$display("FAIL test 14b (latency %0d): the second store never landed", i);
				errors = errors + 1;
			end
			// (c) back-to-back stores stay in order and neither is lost: the
			// second is captured only after the first has drained.
			mr0 = mwrites;
			cpu_write(32'h0000_7900, 32'hD14D_0000 + i);
			cpu_write(32'h0000_7904, 32'hE14E_0000 + i);
			if (mwrites < mr0 + 1) begin
				$display("FAIL test 14c (latency %0d): the second store was accepted before the first drained", i);
				errors = errors + 1;
			end
			wait_drain;
			if (mem[32'h7900>>2] !== 32'hD14D_0000 + i || mem[32'h7904>>2] !== 32'hE14E_0000 + i) begin
				$display("FAIL test 14c (latency %0d): stores out of order or lost (%h %h)", i, mem[32'h7900>>2], mem[32'h7904>>2]);
				errors = errors + 1;
			end
			// (d) a cache-inhibited read under the drain waits for it (the
			// invariant monitor catches an early one) and returns memory.
			cpu_write(32'h0000_7A00, 32'hF14F_0000 + i);
			mem[32'h7A04>>2] = 32'h0A0A_0000 + i;
			@(negedge clk);
			c_req = 1; c_write = 0; c_size = 2'b10; c_addr = 32'h0000_7A04; c_nocache = 1;
			guard5 = 0;
			while (!(c_ack && ce) && guard5 < 200) begin
				@(posedge clk); guard5 = guard5 + 1;
			end
			if (guard5 >= 200 || c_rdata !== 32'h0A0A_0000 + i) begin
				$display("FAIL test 14d (latency %0d): bypass under the drain got %h", i, c_rdata);
				errors = errors + 1;
			end
			@(negedge clk); c_req = 0; c_nocache = 0;
			while (!ce) @(posedge clk);
			@(posedge clk);
			wait_drain;
			// (e) a snoop on the store's own set while it drains kills the
			// merged line; the read after the drain must refetch and see
			// the store, which memory now holds.  Swept across the drain.
			for (off = 0; off < 6; off = off + 1) begin
				expect_read(32'h0000_7B00, mem[32'h7B00>>2], 14);
				cpu_write(32'h0000_7B04, 32'h1B1B_0000 + (i << 8) + off);
				repeat (off) @(negedge clk);
				snoop(32'h0000_FB00);
				wait_drain;
				expect_read(32'h0000_7B04, 32'h1B1B_0000 + (i << 8) + off, 14);
			end
			// (f) a miss held in C_LOOK under the drain, snooped while it
			// waits: the refill must still complete and return memory.
			for (off = 0; off < 6; off = off + 1) begin
				mem[32'h7C00>>2] = 32'h2C2C_0000 + (i << 8) + off;
				snoop(32'h0000_7C00);             // make sure it is not resident
				cpu_write(32'h0000_7D00, 32'h3D3D_0000 + (i << 8) + off);
				fork
					expect_read(32'h0000_7C00, 32'h2C2C_0000 + (i << 8) + off, 14);
					begin
						repeat (off) @(negedge clk);
						snoop(32'h0000_7C00);
					end
				join
				wait_drain;
			end
		end
		mem_lat = 2'd2;
	end

	//------------------------------------------------------------------
	// T15: a chipset write against a CPU store that has not reached
	// memory.  The snoop tests above run with the CPU quiescent; this one
	// opens the window posting creates and drives a DMA write into it,
	// swept across the whole drain at every memory latency.
	//
	// The invariant is NOT who wins.  Two bus masters writing the same
	// address race by nature and either order is legal, so a test that
	// demanded one would be asserting a policy the hardware never
	// promised.  What the machine must never do is end up INCONSISTENT:
	// whatever the CPU reads afterwards has to be what memory holds.  A
	// cache that kept the copy it merged at capture while memory held the
	// DMA's word would serve its own stale value indefinitely, and no
	// existing test could see it.
	//------------------------------------------------------------------
	for (i = 0; i < 4; i = i + 1) begin
		mem_lat = i;
		for (off = 0; off < 10; off = off + 1) begin
			// (a) same address: the CPU's store and the DMA's write
			// collide.  Afterwards the CPU's view and memory must agree,
			// and the value must be one of the two that were written --
			// never a third.
			expect_read(32'h0000_8000, mem[32'h8000>>2], 15);
			cpu_write(32'h0000_8000, 32'hC0C0_0000 + (i << 8) + off);
			dma_write(32'h0000_8000, 32'hD0D0_0000 + (i << 8) + off, off);
			wait_drain;
			cpu_read(32'h0000_8000, d);
			if (d !== mem[32'h8000>>2]) begin
				$display("FAIL test 15a (latency %0d, skew %0d): CPU reads %h, memory holds %h",
				         i, off, d, mem[32'h8000>>2]);
				errors = errors + 1;
			end
			if (d !== 32'hC0C0_0000 + (i << 8) + off &&
			    d !== 32'hD0D0_0000 + (i << 8) + off) begin
				$display("FAIL test 15a (latency %0d, skew %0d): CPU reads %h, neither the store %h nor the DMA %h",
				         i, off, d, 32'hC0C0_0000 + (i << 8) + off, 32'hD0D0_0000 + (i << 8) + off);
				errors = errors + 1;
			end

			// (b) the same LINE, a different longword: the store merges a
			// whole longword into the line and the snoop clears the set,
			// so neither write may take the other's word with it.  Both
			// must stand, and the CPU must see both.
			expect_read(32'h0000_8100, mem[32'h8100>>2], 15);
			cpu_write(32'h0000_8104, 32'hC1C1_0000 + (i << 8) + off);
			dma_write(32'h0000_8100, 32'hD1D1_0000 + (i << 8) + off, off);
			wait_drain;
			if (mem[32'h8100>>2] !== 32'hD1D1_0000 + (i << 8) + off) begin
				$display("FAIL test 15b (latency %0d, skew %0d): the DMA word was lost, memory holds %h",
				         i, off, mem[32'h8100>>2]);
				errors = errors + 1;
			end
			if (mem[32'h8104>>2] !== 32'hC1C1_0000 + (i << 8) + off) begin
				$display("FAIL test 15b (latency %0d, skew %0d): the store was lost, memory holds %h",
				         i, off, mem[32'h8104>>2]);
				errors = errors + 1;
			end
			expect_read(32'h0000_8100, 32'hD1D1_0000 + (i << 8) + off, 15);
			expect_read(32'h0000_8104, 32'hC1C1_0000 + (i << 8) + off, 15);

			// (c) a DMA write to ANOTHER line while a store is in flight.
			// A miss is held behind the drain, so the refill happens after
			// the store lands: the hold must not let the read be satisfied
			// from a line the snoop has already killed.
			expect_read(32'h0000_8200, mem[32'h8200>>2], 15);
			cpu_write(32'h0000_8300, 32'hC2C2_0000 + (i << 8) + off);
			dma_write(32'h0000_8200, 32'hD2D2_0000 + (i << 8) + off, off);
			wait_drain;
			expect_read(32'h0000_8200, 32'hD2D2_0000 + (i << 8) + off, 15);
			if (mem[32'h8300>>2] !== 32'hC2C2_0000 + (i << 8) + off) begin
				$display("FAIL test 15c (latency %0d, skew %0d): the store was lost, memory holds %h",
				         i, off, mem[32'h8300>>2]);
				errors = errors + 1;
			end
		end
	end
	mem_lat = 2'd2;

`ifdef SNOOP_MIXED_X
	//------------------------------------------------------------------
	// T12: can a collided row become OBSERVABLE corruption of a whole set?
	//
	// The divide-4 control fails by a wrong-way hit on the request that
	// collided.  Divide 1 is supposed to fail differently: the collided row
	// is written BACK on the fill, because tags_next and val_next are built
	// from tag_q, so the damage lands on OTHER lines in the set and shows up
	// on a later read.  A row whose ways carry tags the test will ask for
	// later is a legal don't-care value -- the memory promises nothing about
	// the word, so any bit pattern is allowed -- and refusing to use one
	// would restrict the model rather than the design.
	//
	// Four lines in one set, distinct data, then a collided row carrying
	// their four tags with the way associations rotated by one.  If the
	// blinded acceptance term lets that row reach the fill's writeback, each
	// of the four now answers from the way holding its neighbour's data.
	//
	// RESULT: no counterexample, and it is not the acceptance term that
	// saves it.  The experiment fires -- 8 collisions with the permuted row,
	// one on an acceptance cycle -- and the four lines read back correctly
	// with +inj_acc_whole, +inj_look_whole and +inj_fillguard blinded in
	// every combination.  Two observations from +trace_wb: of the 8 writeback
	// attempts while the row was armed, 6 were blocked by fill_snooped and 2
	// wrote, so the writeback guard is real and does act here; and blinding
	// it as well still produces nothing, so it is not the whole story either.
	// The rest is that the permuted row does not carry the REQUESTING line's
	// tag, so the lookup misses and refills from a row that has since been
	// re-read clean.  This route is doubly protected and hard to reach.
	//
	// The divide-1 acceptance term is load-bearing on the LOOKUP route
	// instead, which T13 below demonstrates.  T12 stays as a positive
	// regression over the writeback route.
	//------------------------------------------------------------------
	for (off = 0; off < 8 * CE_DIV; off = off + 1) begin
		// 1. populate: four tags, one set (addr[9:4] = 6'h15), four ways
		for (i = 0; i < 4; i = i + 1)
			expect_read(32'h0000_8150 + (i << 10),
			            mem[(32'h8150 + (i << 10)) >> 2], 12);
		// 2. the collided row: same four tags, rotated one way to the left,
		//    every way valid.  row = {rr, valid[3:0], tag3, tag2, tag1, tag0}
		perm_row = {2'b00, 4'b1111,
		            22'h0000_20 + 22'd2,   // way3 <- tag of line 2
		            22'h0000_20 + 22'd1,   // way2 <- tag of line 1
		            22'h0000_20 + 22'd0,   // way1 <- tag of line 0
		            22'h0000_20 + 22'd3};  // way0 <- tag of line 3
		perm_en = 1'b1;
		// 3. a miss into the same set, with a snoop swept across its
		//    acceptance window so the collision lands on the read whose row
		//    the fill writes back
		fork
			cpu_read(32'h0000_9150, d);
			begin
				repeat (off) @(negedge clk);
				snoop(32'h0000_8150);
			end
		join
		perm_en = 1'b0;
		// 4. every populated line must still answer with its own data
		for (i = 0; i < 4; i = i + 1)
			expect_read(32'h0000_8150 + (i << 10),
			            mem[(32'h8150 + (i << 10)) >> 2], 12);
	end
`endif

`ifdef SNOOP_MIXED_X
	//------------------------------------------------------------------
	// T13: the acceptance-cycle collision, on a read whose value is CHECKED.
	//
	// T3 sweeps a snoop across the acceptance window already, but its
	// concurrent read is deliberately unchecked -- the snoop is unordered
	// against it, so either value is legal there.  That is why the directed
	// row was delivered on acceptance cycles and nothing was ever observed:
	// the one read that could have shown it is the one the bench ignores.
	//
	// Either value is still legal here.  What is NOT legal is a third one.
	// With the row directed to hit a way that does not hold this line, a
	// lookup that acts on it returns that way's word -- the seeded marker,
	// or a neighbour's data -- and neither is the old or the new value.
	//------------------------------------------------------------------
	for (off = 0; off < 8 * CE_DIV; off = off + 1) begin
		expect_read(32'h0000_A150, mem[32'hA150>>2], 13);   // warm
		d2 = mem[32'hA150>>2];                              // the old value
		mem[32'hA150>>2] = 32'h1313_0000 + off;             // the new one
		fork
			cpu_read(32'h0000_A150, d);
			begin
				repeat (off) @(negedge clk);
				snoop(32'h0000_A150);
			end
		join
		if (d !== d2 && d !== mem[32'hA150>>2]) begin
			$display("FAIL test 13 (off %0d): read %h, neither the old value %h nor the new %h",
			         off, d, d2, mem[32'hA150>>2]);
			errors = errors + 1;
		end
		expect_read(32'h0000_A150, mem[32'hA150>>2], 13);   // and it refetches
	end
`endif

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
