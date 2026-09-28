//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// tb_ap040_program.v - runs an assembled self-checking program image       //
//                                                                          //
// The program image (built by build_tests.sh) is loaded with $readmemh     //
// from the file given with +prog=<file>. The program reports through       //
// memory-mapped registers:                                                 //
//   $F100 word: failing test number                                        //
//   $F102 word: $BAD0 = failed, $600D = all passed                         //
//   $F110 word: interrupt request level (0 releases the lines)             //
//   $F120 byte: writes must carry FC=1 (MOVES/DFC check)                   //
//                                                                          //
// The image runs twice: once with back-to-back bus ready and once with     //
// varied wait states.                                                      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_program;

`ifdef AP040_EXACT_SIEVE_MONITOR
exact_sieve_monitor sieve_monitor();
`endif

reg clk = 0;
reg nreset = 0;

always #5 clk = ~clk;

wire [15:0] data_in;
wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds;
wire  [1:0] busstate;
wire        longword;
wire        nresetout;
wire  [2:0] fc;
wire [31:0] cacr_out, vbr_out;
wire        debug_busy, debug_fault, debug_halted;
wire [255:0] debug_status;
wire        walker_req, walker_we;
wire [31:0] walker_addr, walker_wdat;
reg         walker_ack;
reg  [31:0] walker_data;
reg         walker_berr_r;    // one-shot walker bus error, armed via $F146
reg         wberr_arm;

reg         mem_ready;
reg         berr_armed;
reg   [1:0] irq_exc_armed;
reg   [2:0] irq_fetch_stall;
wire        berr_d = berr_armed && nreset && (busstate != 2'b01) &&
                     (addr_out[15:0] == 16'hF140);

// $F154 arms a one-shot bus error on an instruction FETCH at the written
// address (0 disarms).  This is the only way to reach the queue's
// speculative-fault paths: the page guard rules out translation
// differences, so only a physical berr can fault a fetch ahead of demand.
reg         fberr_armed = 0;
reg  [15:0] fberr_addr = 0;
wire        fberr = fberr_armed && nreset && (busstate == 2'b00) &&
                    (addr_out[15:0] == fberr_addr);

wire        berr = berr_d | fberr;

wire        clkena_in = (busstate == 2'b01) | mem_ready | berr;

reg   [2:0] ipl_lvl;
reg  [15:0] ipl_delay = 0;   // $F148: delayed level-2 IPL countdown
integer   clkcount = 0;      // free-running clk counter for $F108 stamps
integer   stamp_prev = 0;
reg   [7:0] ipl_pulse = 0;   // $F14C: withdraw the request after N cycles
reg   [7:0] ipl_step  = 0;   // $F150: downgrade the request after N cycles
reg   [2:0] ipl_next  = 0;   // $F150: level the encoder falls back to
// A device that has let go of IPL must never produce an interrupt.  Rather
// than time a program against it, watch the invariant directly: count how
// long the pins have been idle and fail if an autovectored interrupt is
// accepted well after that.  The 12-cycle margin is comfortably past the
// core's two-stage IPL synchronizer and the hold tracking it (~5 cycles),
// while a core that retains a withdrawn level indefinitely is caught.
reg  [15:0] ipl_idle_for = 0;
reg         irq_seen_q = 0;
// Shadow IPEND claims over the core's own synchronized level: bit L is set
// while a level-L request is visibly asserted AND has qualified against the
// live mask; it is cleared when the synchronized level falls below L (the
// device let go, taking any claim with it) or when an acceptance consumes
// it.  A level hidden behind a higher request holds no claim of its own:
// when the encoder falls back to it, it must requalify like a fresh
// request.  This is the reference model for the mask invariant below.
reg   [6:1] tb_qual = 0;
// Previous core state, for the exception-prefetch invariant below.
reg   [7:0] epf_state_q = 0;
integer ql;
always @(posedge clk) begin
	if (!nreset) tb_qual <= 0;
	else begin
		for (ql = 1; ql <= 6; ql = ql + 1) begin
			if (dut.core.irq_lvl < ql)
				tb_qual[ql] <= 0;
			else if (dut.core.irq_lvl == ql && ql > dut.core.sr[10:8])
				tb_qual[ql] <= 1;
		end
		if ((dut.core.state == 8'd34) && dut.core.exc_is_irq &&
		    !irq_seen_q && dut.core.irq_lvl_l != 3'd7 &&
		    dut.core.irq_lvl_l != 3'd0)
			tb_qual[dut.core.irq_lvl_l] <= 0;
	end
end
// +exctrace: print every exception entry (vector, pc) for A/B diffing
reg [7:0] et_prev = 0;
always @(posedge clk) clkcount = clkcount + 1;
always @(posedge clk) begin
	et_prev <= dut.core.state;
	if ($test$plusargs("exctrace") &&
	    dut.core.state == 8'd34 && et_prev != 8'd34) begin
		$display("EXC vec=%0d pc=%08x spc=%08x sr=%04x",
		         dut.core.exc_vec, dut.core.pc,
		         dut.core.exc_spc, dut.core.sr);
		// vector 55 means an unsupported FP data type reached the FPU:
		// dump the register file so the offending operand is visible
		// without a rebuild (a denormal/unnormal register value here is
		// itself a defect -- no AP040 path may create one).
		if (dut.core.exc_vec == 8'd55) begin : et_fpdump
			integer efr;
			for (efr = 0; efr < 8; efr = efr + 1)
				$display("  FP%0d = %x %04x %x", efr,
				         dut.core.g_fpu.fpu.fr_s[efr],
				         dut.core.g_fpu.fpu.fr_e[efr],
				         dut.core.g_fpu.fpu.fr_m[efr]);
		end
	end
end

// The internal caches are ON by default here, as in the shipping build.
// -DAP040_TB_CACHE=0 builds the g_nocache configuration instead, for
// cache-vs-no-cache cycle comparisons (t_cache's architected
// stale-until-CINVA expectations only hold with them on).
`ifndef AP040_TB_CACHE
`define AP040_TB_CACHE 1
`endif

ap040_tg68k_compat #(.AP040_ENABLE_CACHE(`AP040_TB_CACHE)) dut
(
	.clk(clk),
	.nreset(nreset),
	.cache_allow_all(1'b1),
	.cache_snoop_stb(1'b0), .cache_snoop_addr(32'd0),
	.cache_z2_ena(1'b0),
	.cache_z3_base0(5'd0),
	.cache_z3_ena0(1'b0),
	.cache_z3_base1(4'd0),
	.cache_z3_ena1(1'b0),
	.clkena_in(clkena_in),
	.data_in(data_in),
	.ipl(~ipl_lvl),
	.ipl_autovector(1'b1),
	.berr(berr),

	.addr_out(addr_out),
	.data_write(data_write),
	.nwr(nwr),
	.nuds(nuds),
	.nlds(nlds),
	.busstate(busstate),
	.longword(longword),
	.nresetout(nresetout),
	.fc(fc),

	.mmu_addr_log(),
	.mmu_addr_phys(),
	.mmu_cache_inhibit(),
	.walker_req(walker_req),
	.walker_we(walker_we),
	.walker_addr(walker_addr),
	.walker_wdat(walker_wdat),
	.walker_ack(walker_ack),
	.walker_data(walker_data),
	.walker_berr(walker_berr_r),
	.cache_req(),
	.cache_addr(),
	.cache_data(16'd0),
	.cache_ack(1'b0),
	.cache_burst(),
	.cache_burst_len(),
	.cache_ramaddr(),

	.cacr_out(cacr_out),
	.vbr_out(vbr_out),
	.debug_busy(debug_busy),
	.debug_fault(debug_fault),
	.debug_halted(debug_halted),
	.debug_status(debug_status)
);

wire [31:0] dbg_pc = debug_status[31:0];

wire [15:0] dbg_ir = debug_status[63:48];
reg  [7:0]  prev_core_state;
always @(posedge clk) if (dut.core.ce) prev_core_state <= dut.core.state;

//---------------------------------------------------------------------------
// 64 KB memory model
//---------------------------------------------------------------------------

reg [15:0] mem [0:32767];

// Phase 2 models the cpu_cache_new handshake: the acknowledge is a LEVEL
// that stays high -- with the data captured when it rose -- until the bus
// is sampled idle (cpu_ack clears only on !cpu_cs).  A request issued
// with no sampled idle gap is therefore served the PREVIOUS data, which
// is the hardware failure mode behind the cputest FADD.P ([]) stale
// pointer word and the FABS.X ([0]) shifted operand window.
reg        lvl_hold;
reg [15:0] lvl_data;

integer errors;
integer phase;
integer result;          // 0 running, 1 pass, 2 fail

assign data_in = (phase == 2 && lvl_hold) ? lvl_data : mem[addr_out[15:1]];
reg [1023:0] prog_file;
reg [1023:0] dump_file;

function [2:0] latency;
	input integer ph;
	input integer n;
	begin
		if (ph == 0) latency = 0;
		else         latency = (n * 7 + 3) % 6;
	end
endfunction

reg [2:0] lat_cnt;
integer lat_idx;
reg       walker_pending;
reg       walker_armed;
reg       walker_we_latch;
reg [31:0] walker_addr_latch;
reg [31:0] walker_wdat_latch;
reg  [2:0] walker_lat_cnt;
integer   walker_lat_idx;
integer   walker_cycles;

always @(posedge clk) begin
	if (phase != 2) mem_ready <= 0;
	if (!nreset) begin
		mem_ready <= 0;
		lvl_hold <= 0;
		lat_cnt <= latency(phase, 0);
		lat_idx <= 1;
		berr_armed <= 1;
		irq_exc_armed <= 0;
		irq_fetch_stall <= 0;
	end
	else if (berr) begin
		// One physical bus error per arming.  The restarted access
		// succeeds, proving that the adapter released the failed
		// sub-cycle.  Each one-shot clears only itself: a fetch berr
		// must not eat the armed data berr or vice versa.
		if (berr_d) berr_armed  <= 0;
		if (fberr)  fberr_armed <= 0;
		mem_ready <= 0;
		lvl_hold <= 0;
	end
	else if (phase == 2) begin
		// level acknowledge: drops only when the bus is sampled idle
		if (busstate == 2'b01) begin
			mem_ready <= 0;
			lvl_hold <= 0;
		end
		else if (!lvl_hold) begin
			if (lat_cnt == 0) begin
				mem_ready <= 1;
				lvl_hold <= 1;
				lvl_data <= mem[addr_out[15:1]];
				lat_cnt <= latency(phase, lat_idx);
				lat_idx <= lat_idx + 1;
			end
			else lat_cnt <= lat_cnt - 1'd1;
		end
		// else: hold acknowledge and captured data (stale if the CPU
		// started a new access without an idle gap)
	end
	else if (irq_fetch_stall != 0 && busstate != 2'b01 && !mem_ready) begin
		// Keep the first handler refill outstanding while IPL synchronizes.
		irq_fetch_stall <= irq_fetch_stall - 1'd1;
	end
	else if (busstate != 2'b01 && !mem_ready) begin
		if (lat_cnt == 0) begin
			mem_ready <= 1;
			lat_cnt   <= latency(phase, lat_idx);
			lat_idx   <= lat_idx + 1;
		end
		else lat_cnt <= lat_cnt - 1'd1;
	end
	// The exception program uses this write-only test register to arm a
	// second physical bus error for its faulting-MOVES case.
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF142)
		berr_armed <= 1;

	// $F154: one-shot fetch bus error (see the fberr wire above; the
	// fire-cycle bookkeeping lives in the main berr branch)
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF154) begin
		fberr_armed <= |data_write;
		fberr_addr  <= data_write;
	end

	// $F146 arms a one-shot bus error on the NEXT table-walker descriptor
	// access, for the PTEST MMUSR B-bit test.
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF146)
		wberr_arm <= 1;

	// Mode 1 raises IPL during stacking. Mode 2 raises it after the vector
	// has been read and stalls the first handler refill for synchronization.
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF144) begin
		irq_exc_armed <= data_write[1:0];
			`ifdef AP040_TRACE
			$display("TRACE armed exception-time IRQ mode=%0d pc=%h",
			         data_write[1:0], dbg_pc);
			`endif
	end
	else if (irq_exc_armed == 1 && dut.core.state == 8'd34 &&
	         dut.core.exc_vec == 8'd32) begin
		ipl_lvl <= 3'd2;
		irq_exc_armed <= 0;
			`ifdef AP040_TRACE
			$display("TRACE raised stacking-time IPL2 pc=%h", dbg_pc);
			`endif
	end
	else if (irq_exc_armed == 2 && dut.core.state == 8'd42 &&
	         dut.core.exc_vec == 8'd32) begin
		ipl_lvl <= 3'd2;
		irq_exc_armed <= 0;
		irq_fetch_stall <= 3'd5;
		`ifdef AP040_TRACE
		$display("TRACE raised handler-refill IPL2 pc=%h", dbg_pc);
		`endif
	end

	// $F148 arms a delayed level-2 interrupt: the IPL lines rise the
	// written number of clk cycles later.  The FPU soak in t_fpu sweeps
	// this against background (released) FPU execution.
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF148)
		ipl_delay <= data_write;
	else if (ipl_delay != 0) begin
		ipl_delay <= ipl_delay - 1'd1;
		if (ipl_delay == 16'd1) ipl_lvl <= 3'd2;
	end

	// phantom-interrupt invariant
	if (ipl_lvl == 3'd0) begin
		if (ipl_idle_for != 16'hffff) ipl_idle_for <= ipl_idle_for + 1'd1;
	end
	else
		ipl_idle_for <= 0;
	irq_seen_q <= (dut.core.state == 8'd34) && dut.core.exc_is_irq;
	if ((dut.core.state == 8'd34) && dut.core.exc_is_irq && !irq_seen_q &&
	    ipl_idle_for > 16'd12) begin
		errors = errors + 1;
		$display("FAIL: interrupt accepted %0d cycles after IPL went idle (phantom)",
		         ipl_idle_for);
	end

	// mask invariant: a level 1-6 interrupt is accepted strictly above the
	// SR mask, with exactly one exception: a request that QUALIFIED
	// against the mask while asserted keeps its claim across a later mask
	// raise (IPEND; test 136).  tb_qual below models those claims from
	// the pins alone, so an acceptance at or below the mask without a
	// claim -- e.g. a hold retargeted to a level that never qualified --
	// is a phantom.  State 34 is S_EXC0; sr still holds the pre-exception
	// mask on its first cycle (the throwaway second pass re-enters at
	// S_EXC1, so it cannot trip this).
	if ((dut.core.state == 8'd34) && dut.core.exc_is_irq && !irq_seen_q &&
	    dut.core.irq_lvl_l != 3'd7 &&
	    dut.core.irq_lvl_l <= dut.core.sr[10:8] &&
	    !tb_qual[dut.core.irq_lvl_l]) begin
		errors = errors + 1;
		$display("FAIL: level %0d interrupt accepted at or below mask %0d (pc=%h)",
		         dut.core.irq_lvl_l, dut.core.sr[10:8], dbg_pc);
	end

	// X2.2 queue invariant (audit 3.5): exception_prefetch issues the
	// first vector-stream word unconditionally, so it must never run
	// while a queue fetch is still outstanding -- the second request
	// would collide with the in-flight one at the bus adapter.  The core
	// enforces this remotely, in S_EXC0's epf_pend wait, so check it
	// here at the point that DEPENDS on it: a future resequencing of the
	// exception path then fails the suite instead of the hardware.
	// State 178 is S_EPF_FILL, reachable only from exception_prefetch
	// and from state 179 (S_EPF_GAP, the word-to-word handshake gap).
	epf_state_q <= dut.core.state;
	if (nreset && dut.core.state == 8'd178 && epf_state_q != 8'd178 &&
	    epf_state_q != 8'd179 && dut.core.epf_pend) begin
		errors = errors + 1;
		$display("FAIL: exception_prefetch entered with a queue fetch outstanding (pc=%h)",
		         dbg_pc);
	end

	// $F14C models a device that WITHDRAWS its request: IPL rises to the
	// written level and drops again after the written number of cycles,
	// without waiting to be acknowledged.  A 68040 requires the request
	// to be held until acknowledged, so nothing may be taken from it.
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF14C) begin
		ipl_lvl   <= data_write[2:0];
		ipl_pulse <= data_write[15:8];
	end
	else if (ipl_pulse != 0) begin
		ipl_pulse <= ipl_pulse - 1'd1;
		if (ipl_pulse == 8'd1) ipl_lvl <= 3'd0;
	end

	// $F150 models TWO devices sharing the IPL encoder: the higher one
	// (bits [2:0]) withdraws after the written number of cycles while a
	// lower one (bits [6:4]) keeps requesting, so the lines DOWNGRADE
	// instead of going idle.  The lower level is a fresh request that is
	// only ever taken if it qualifies against the mask on its own.
	if (nreset && mem_ready && busstate == 2'b11 &&
	    addr_out[15:0] == 16'hF150) begin
		ipl_lvl  <= data_write[2:0];
		ipl_next <= data_write[6:4];
		ipl_step <= data_write[15:8];
	end
	else if (ipl_step != 0) begin
		ipl_step <= ipl_step - 1'd1;
		if (ipl_step == 8'd1) ipl_lvl <= ipl_next;
	end
end

// Dedicated 32-bit physical table-walker memory port.  It deliberately has
// an independent latency profile and never asserts mem_ready on the 16-bit
// CPU bus, so all MMU tests fail if descriptor traffic leaks onto that bus.
always @(posedge clk) begin
	walker_ack <= 0;
	walker_berr_r <= 0;
	if (!nreset) begin
		wberr_arm        <= 0;
		walker_pending   <= 0;
		walker_armed     <= 1;
		walker_we_latch  <= 0;
		walker_addr_latch <= 0;
		walker_wdat_latch <= 0;
		walker_data      <= 0;
		walker_lat_cnt   <= latency(phase, 0);
		walker_lat_idx   <= 1;
		walker_cycles    <= 0;
	end
	else begin
		if (!walker_req) walker_armed <= 1;
		if (walker_req && (busstate != 2'b01)) begin
			errors = errors + 1;
			$display("FAIL: walker and 16-bit CPU bus active together (pc=%h)", dbg_pc);
			result = 2;
		end

		if (walker_req && walker_armed && !walker_pending) begin
			walker_pending    <= 1;
			walker_armed      <= 0;
			walker_we_latch   <= walker_we;
			walker_addr_latch <= walker_addr;
			walker_wdat_latch <= walker_wdat;
			walker_lat_cnt    <= latency(phase, walker_lat_idx);
			walker_lat_idx    <= walker_lat_idx + 1;
			walker_cycles     <= walker_cycles + 1;
		end
		else if (walker_pending) begin
			if (walker_lat_cnt != 0)
				walker_lat_cnt <= walker_lat_cnt - 1'd1;
			else if (wberr_arm) begin
				// injected physical bus error on this descriptor access
				wberr_arm      <= 0;
				walker_pending <= 0;
				walker_berr_r  <= 1;
			end
			else begin
				if (walker_addr_latch[31:16] != 0 ||
				    walker_addr_latch[1:0] != 0) begin
					errors = errors + 1;
					$display("FAIL: invalid walker address %h", walker_addr_latch);
					result = 2;
				end
				else if (walker_we_latch) begin
					mem[walker_addr_latch[15:1]] = walker_wdat_latch[31:16];
					mem[walker_addr_latch[15:1] + 1'b1] = walker_wdat_latch[15:0];
				end
				else begin
					walker_data <= {mem[walker_addr_latch[15:1]],
					                mem[walker_addr_latch[15:1] + 1'b1]};
				end
				walker_pending <= 0;
				walker_ack     <= 1;
			end
		end
	end
end

// bus monitor and write commit
always @(posedge clk) begin
	if (nreset && mem_ready) begin
		// Reset vectors, exception frames and exception vectors are
		// supervisor-data cycles; the first handler opcode is supervisor
		// program.  This also catches a leaked MOVES SFC/DFC override.
		// A posted store drains after its requester moved on: a user store
		// issued just before a trap drains while the core is already in
		// exception processing, with its own (user) FC.  Attribute those
		// cycles to the store, not to the exception.
		if (dut.core.in_exc && !dut.cache_posting) begin
			if (busstate == 2'b00 && fc !== 3'd6) begin
				errors = errors + 1;
				$display("FAIL: exception handler fetch used FC=%0d, expected 6", fc);
				result = 2;
			end
			else if ((busstate == 2'b10 || busstate == 2'b11) && fc !== 3'd5) begin
				errors = errors + 1;
				$display("FAIL: exception/reset data cycle used FC=%0d, expected 5", fc);
				result = 2;
			end
		end

		if (addr_out[31:16] != 0) begin
			errors = errors + 1;
			$display("FAIL: access outside memory model at %h (pc=%h)", addr_out, dbg_pc);
			result = 2;
		end

		if (busstate == 2'b11) begin
			if (!nuds) mem[addr_out[15:1]][15:8] = data_write[15:8];
			if (!nlds) mem[addr_out[15:1]][7:0]  = data_write[7:0];

			// test control registers
			if (addr_out[15:0] == 16'hF102 && !nuds && !nlds) begin
				if (data_write == 16'h600D) result = 1;
				else begin
					errors = errors + 1;					$display("FAIL: program reports failure, test %0d (phase %0d, pc=%h, ill=%0d, addr=%0d)",
					         mem[16'hF100 >> 1], phase, dbg_pc,
					         mem[16'h3602 >> 1], mem[16'h361E >> 1]);
					// t_exceptions stamps $3670 with the handler that
					// rejected a frame, so a shared hfail is still
					// attributable (X2.3a).
					if (mem[16'hF100 >> 1] == 98) begin
						$display("     hfail from handler id %0d",
						         mem[16'h3670 >> 1]);
						$display("     berr fault addr=%04x%04x stacked pc=%04x%04x armed=%04x%04x",
						         mem[16'h3674 >> 1], mem[16'h3676 >> 1],
						         mem[16'h3678 >> 1], mem[16'h367A >> 1],
						         mem[16'h3654 >> 1], mem[16'h3656 >> 1]);
					end
					result = 2;
				end
			end
			// $F108: cycle-stamp marker.  Writing a tag prints the clk
			// count since the previous stamp, so a program can bracket a
			// block of instructions and get its cost without a waveform.
			// Used by the FPU latency probe (hw/fptime.s).
			if (addr_out[15:0] == 16'hF108) begin
				$display("STAMP tag=%04x cycles=%0d", data_write,
				         clkcount - stamp_prev);
				stamp_prev = clkcount;
			end
			if (addr_out[15:0] == 16'hF110) begin
				ipl_lvl <= data_write[2:0];
			end
			if (addr_out[15:1] == (16'hF120 >> 1) && fc !== 3'd1) begin
				errors = errors + 1;
				$display("FAIL: write to F120 with FC=%0d, expected 1", fc);
			end
			// DMA-style poke behind the CPU's back for the cache tests
			if (addr_out[15:0] == 16'hF130) begin
				mem[16'h3500 >> 1] = data_write;
				mem[16'h3502 >> 1] = 16'h0000;
			end
		end
	end
end

// Locked-RMW indivisibility (plan section 8): once a data read has
// completed inside a TAS/CAS/CAS2 (lk_cyc), no instruction fetch may
// appear on the bus until the locked write completes.  Further reads are
// legal (misaligned splits, CAS2's second operand, a memory-indirect
// pointer), and a fault path disarms at its first frame write.  This
// catches a background queue fetch issued between the locked read and
// the locked write.
reg lk_window = 0;
always @(posedge clk) begin
	if (!nreset) lk_window <= 0;
	else begin
		if (mem_ready && busstate == 2'b10 && dut.core.lk_cyc)
			lk_window <= 1;
		else if (mem_ready && busstate == 2'b11)
			lk_window <= 0;
		// A locked sequence can also end WITHOUT a write: CAS/CAS2 whose
		// comparison fails performs no memory write, and a faulted one is
		// abandoned.  lk_cyc is held from decode to fetch_next across the
		// whole indivisible sequence, so once it drops the sequence is over
		// and the next fetch is legal.  Closing only on the write reported
		// every failed CAS as a violation.
		else if (!dut.core.lk_cyc)
			lk_window <= 0;
		if (lk_window && busstate == 2'b00) begin
			errors = errors + 1;
			$display("FAIL: instruction fetch inside a locked RMW (pc=%h addr=%h state=%0d lk=%b epf_pend=%b ir=%04x)",
			         dbg_pc, addr_out, dut.core.state, dut.core.lk_cyc,
			         dut.core.epf_pend, dbg_ir);
			lk_window <= 0;
		end
	end
end

// unexpected halt detection
always @(posedge clk) begin
	if (nreset && (debug_fault || debug_halted) && result == 0) begin
		errors = errors + 1;
		$display("FAIL: core halted, fault=%b pc=%h ir=%h prev_state=%0d in_exc=%b mem_flt=%b",
		         debug_fault, dbg_pc, dbg_ir, prev_core_state, dut.core.in_exc,
		         dut.core.mem_flt);
		// the last failure code the program managed to report, if any --
		// a halt after a failed report leaves the real code visible here
		$display("  failcode=%04x", mem[16'hF100 >> 1]);
		result = 2;
	end
end

`ifdef AP040_TRACE
always @(posedge clk) if (nreset && dut.core.ce) begin
	if (dut.core.state == 7'd4)
		$display("TRACE decode pc=%h ir=%h sr=%h in_exc=%b", dut.core.pc,
		         dut.core.ir, dut.core.sr, dut.core.in_exc);
	if (dut.core.state == 7'd34)
		$display("TRACE exc vec=%0d fmt=%0d spc=%h", dut.core.exc_vec, dut.core.exc_fmt, dut.core.exc_spc);
end
always @(posedge clk) if (nreset && mem_ready && busstate == 2'b11 &&
                          addr_out >= 32'h3600 && addr_out < 32'h3640)
	$display("TRACE cntwr addr=%h data=%h uds=%b lds=%b", addr_out, data_write, nuds, nlds);
`endif

//---------------------------------------------------------------------------
// phase driver
//---------------------------------------------------------------------------

// +prof: per-state cycle histogram -- the tb_prof the plan's cycle
// claims come from (X2.8).  Counts every clk cycle by core state; the
// stall column is the subset spent with clkena_in low (bus wait).
// Printed and cleared at the end of each phase.
integer prof_cnt [0:255];
integer prof_stall [0:255];
integer prof_on = 0;
integer prof_fetch_brf = 0;     // S_FETCH cycles consuming a branch-refill seed
integer prof_inline_brf = 0;    // refill target dispatched without S_FETCH
integer prof_inline_brf_from [0:255];
reg [7:0] prof_prev_state = 0;

// +memlat: request-to-acknowledge latency for the core's memory port,
// split by operation class.  S_MRD costs ~7.6 cycles on a CACHED load
// while stalling on the bus for only 12% of them, so the round trip
// through the MMU and cache -- not the wait for memory -- is what the
// core is paying.  This measures that path directly, per class, which
// is the prerequisite the plan sets before touching the handshake.
integer memlat_on = 0;
integer memlat_run;            // cycles since the current request went out
integer memlat_n    [0:2];     // 0 = ifetch, 1 = data read, 2 = data write
integer memlat_sum  [0:2];
integer memlat_max  [0:2];
integer memlat_hist [0:2][0:31];
integer memlat_cls;
integer mli, mlj;
// How much of S_MRD/S_MWR is spent waiting for the fetch queue to give
// the shared memory port back, rather than waiting for memory itself.
integer memlat_portwait;
integer memlat_mrd;
integer pi;
initial begin
	prof_on = $test$plusargs("prof");
	memlat_on = $test$plusargs("memlat");
	memlat_run = -1;
	memlat_portwait = 0;
	memlat_mrd = 0;
	for (mli = 0; mli < 3; mli = mli + 1) begin
		memlat_n[mli] = 0; memlat_sum[mli] = 0; memlat_max[mli] = 0;
		for (mlj = 0; mlj < 32; mlj = mlj + 1) memlat_hist[mli][mlj] = 0;
	end
	for (pi = 0; pi < 256; pi = pi + 1) begin
		prof_cnt[pi] = 0;
		prof_stall[pi] = 0;
		prof_inline_brf_from[pi] = 0;
	end
end
always @(posedge clk) if (memlat_on && nreset) begin
	// state 9 = S_MRD, 10 = S_MWR
	if (dut.core.state == 8'd9 || dut.core.state == 8'd10) begin
		memlat_mrd = memlat_mrd + 1;
		if (!dut.core.m_issued && dut.core.epf_pend)
			memlat_portwait = memlat_portwait + 1;
	end
	if (dut.core.mem_req && memlat_run < 0) begin
		// request just went out: classify it and start counting
		memlat_run <= 0;
		memlat_cls <= dut.core.mem_instr ? 0 : (dut.core.mem_write ? 2 : 1);
	end
	else if (memlat_run >= 0) begin
		if (dut.core.mem_ack) begin
			memlat_n[memlat_cls]   = memlat_n[memlat_cls] + 1;
			memlat_sum[memlat_cls] = memlat_sum[memlat_cls] + memlat_run + 1;
			if (memlat_run + 1 > memlat_max[memlat_cls])
				memlat_max[memlat_cls] = memlat_run + 1;
			memlat_hist[memlat_cls][(memlat_run + 1) > 31 ? 31 : memlat_run + 1] =
				memlat_hist[memlat_cls][(memlat_run + 1) > 31 ? 31 : memlat_run + 1] + 1;
			memlat_run <= -1;
		end
		else memlat_run <= memlat_run + 1;
	end
end

always @(posedge clk) if (prof_on && nreset) begin
	prof_cnt[dut.core.state] = prof_cnt[dut.core.state] + 1;
	if (!clkena_in)
		prof_stall[dut.core.state] = prof_stall[dut.core.state] + 1;
	if (dut.core.state == 8'd3 && dut.core.epf_brf)
		prof_fetch_brf = prof_fetch_brf + 1;
	if (dut.core.state == 8'd4 && dut.core.epf_brf &&
	    dut.core.epf_head == 3'd1 && dut.core.epf_count == 4'd3 &&
	    prof_prev_state != 8'd3) begin
		prof_inline_brf = prof_inline_brf + 1;
		prof_inline_brf_from[prof_prev_state] =
			prof_inline_brf_from[prof_prev_state] + 1;
	end
	prof_prev_state <= dut.core.state;
end

task memlat_dump;
	input integer ph;
	integer c, b;
	begin
		$display("MEMLAT phase %0d: S_MRD/S_MWR %0d cycles, %0d waiting for the fetch queue to release the port (%0d%%)",
		         ph, memlat_mrd, memlat_portwait,
		         (memlat_mrd == 0) ? 0 : (memlat_portwait * 100) / memlat_mrd);
		memlat_mrd = 0; memlat_portwait = 0;
		for (c = 0; c < 3; c = c + 1) begin
			if (memlat_n[c] != 0) begin
				$display("MEMLAT phase %0d %0s: n=%0d avg=%0d.%0d max=%0d",
				         ph,
				         (c == 0) ? "ifetch " : (c == 1) ? "dataread" : "datawrite",
				         memlat_n[c],
				         memlat_sum[c] / memlat_n[c],
				         (memlat_sum[c] * 10 / memlat_n[c]) % 10,
				         memlat_max[c]);
				for (b = 0; b < 32; b = b + 1)
					if (memlat_hist[c][b] != 0)
						$display("MEMLAT     %0d cyc: %0d", b, memlat_hist[c][b]);
			end
			memlat_n[c] = 0; memlat_sum[c] = 0; memlat_max[c] = 0;
			for (b = 0; b < 32; b = b + 1) memlat_hist[c][b] = 0;
		end
	end
endtask

task prof_dump;
	input integer ph;
	integer total, fetch_immf;
	begin
		total = 0;
		for (pi = 0; pi < 256; pi = pi + 1) total = total + prof_cnt[pi];
		$display("PROF phase %0d: %0d cycles total", ph, total);
		for (pi = 0; pi < 256; pi = pi + 1)
			if (prof_cnt[pi] != 0)
				$display("PROF   state %0d: %0d (%0d stalled)",
				         pi, prof_cnt[pi], prof_stall[pi]);
		fetch_immf = prof_cnt[8'd3] + prof_cnt[8'd8];
		$display("PROF   S_FETCH+S_IMMF occupancy: %0d / %0d = %0d%%",
		         fetch_immf, total, (fetch_immf * 100) / total);
		$display("PROF   branch-refill S_FETCH occupancy: %0d", prof_fetch_brf);
		$display("PROF   inline branch-refill dispatches: %0d", prof_inline_brf);
		for (pi = 0; pi < 256; pi = pi + 1)
			if (prof_inline_brf_from[pi] != 0)
				$display("PROF     from state %0d: %0d",
				         pi, prof_inline_brf_from[pi]);
		prof_fetch_brf = 0;
		prof_inline_brf = 0;
		for (pi = 0; pi < 256; pi = pi + 1) begin
			prof_cnt[pi] = 0;
			prof_stall[pi] = 0;
			prof_inline_brf_from[pi] = 0;
		end
	end
endtask

integer timeout;
integer maxcycles = 20000000;
integer selected_phase = -1;
integer i;

task run_phase;
	input integer ph;
	begin
		phase   = ph;
		result  = 0;
		ipl_lvl = 0;
		irq_exc_armed = 0;
		irq_fetch_stall = 0;
		fberr_armed = 0;

		for (i = 0; i < 32768; i = i + 1) mem[i] = 16'h0000;
		$readmemh(prog_file, mem);
		// interrupt-injection capability word: t_fpu's IRQ soak runs
		// only where the bench can deliver IPL
		mem[16'hF160 >> 1] = 16'h0007;	// coarse + fine IPL + berr injection

		nreset = 0;
		repeat (10) @(posedge clk);
		nreset = 1;

		timeout = 0;
		while (result == 0 && timeout < maxcycles) begin
			@(posedge clk);
			timeout = timeout + 1;
		end

		if (result == 0) begin
			errors = errors + 1;
			$display("FAIL: phase %0d timeout, pc=%h ir=%h fault=%b halted=%b",
			         ph, dbg_pc, dbg_ir, debug_fault, debug_halted);
		end
		else if (result == 1)
			$display("phase %0d passed (%0d cycles)", ph, timeout);
		if (prof_on) prof_dump(ph);
		if (memlat_on) memlat_dump(ph);
	end
endtask

initial begin
	errors = 0;
	if ($value$plusargs("maxcycles=%d", maxcycles)) begin end
	if ($value$plusargs("phase=%d", selected_phase)) begin end
	if (!$value$plusargs("prog=%s", prog_file)) begin
		$display("FAIL: missing +prog=<hexfile>");
		$finish;
	end
	$display("tb_ap040_program: running %0s", prog_file);

	if (selected_phase >= 0 && selected_phase <= 2)
		run_phase(selected_phase);
	else begin
		run_phase(0);
		run_phase(1);
		run_phase(2);
	end

	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	// differential testing: dump the data window for comparison
	if (errors == 0 && $value$plusargs("dump=%s", dump_file))
		$writememh(dump_file, mem, 'h3000 >> 1, ('h4000 >> 1) - 1);
	$finish;
end

endmodule
