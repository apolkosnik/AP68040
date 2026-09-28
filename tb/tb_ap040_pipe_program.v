//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24)                   //
//                                                                          //
// tb_ap040_pipe_program.v - the sequential core's self-checking programs   //
//                                                                          //
// tests/ap040/asm/*.s were written for rtl/ap040/ap040_core.v and run by   //
// tests/ap040/tb_ap040_program.v. This is that bench around the pipelined  //
// core: the same 64 KB memory, the same three bus phases (back-to-back     //
// ready, varied wait states, and a level-held acknowledge that serves a    //
// request issued without an idle gap the PREVIOUS data), the same reset    //
// vectors at $0 (RESET_VECTORS=1), and the same protocol registers:        //
//   $F100 w  failing test number       $F102 w  $600D pass / $BAD0 fail    //
//   $F108 w  cycle stamp               $F110 w  interrupt level            //
//   $F120    writes must carry FC=1    $F130 w  DMA-style poke of $3500    //
//   $F134 w  a poke's address          $F136 w  poke the word there,       //
//            behind the CPU (the data cache's tests; caches stage C)       //
//   $F138 w  a peek's address          $F13A r  memory's word there,       //
//            whatever the data cache holds (copyback; caches stage D)      //
//   $F166 r  how many times RESET has driven the reset line (stage F)      //
//   $F15C w  a chipset write's address $F15E w  write the word there, as   //
//            the chipset would: behind the CPU, and snooped (stage F)      //
//   $F148 w  level 2 after N cycles    $F14C w  level, withdrawn after N   //
//   $F150 w  two devices: level, then a lower one after N cycles           //
//   $F144 w  level 2 while TRAP #0 is stacked (1), or once its vector has  //
//            been read, holding the handler's first fetch a few cycles (2) //
//   $F160 r  capability word: 7, coarse and fine interrupts, bus errors;   //
//            not bit 5 -- IPLDLY is calibrated to the sequential core's    //
//            cycles for t_exceptions' test 136, which is bypassed here,    //
//            and its rule (IPEND) is this bench's claim invariant below    //
//   $F140/$F142/$F154/$F146: bus errors on a data cycle, re-armed, on a    //
//            fetch at an address, and on the next table-walk descriptor    //
//                                                                          //
// The program image is +prog=<hex>, built by tests/ap040/build_tests.sh.   //
//                                                                          //
// Kept from the reference bench, on this core's own signals: the phantom-  //
// interrupt invariant (nothing accepted well after the level went idle)    //
// and the mask invariant (a level 1-6 interrupt accepted at or below the   //
// mask only on a claim it made while it qualified); and, added here, the   //
// other half of that rule: a claim is not lost -- a request that           //
// qualified and is still asserted is taken, whatever the mask does after.  //
// Not kept, because                                                        //
// they name the sequential core's states: the exception-prefetch queue     //
// invariant, the locked read-modify-write fetch window (this core has no   //
// bus lock yet), and the exception-cycle function-code checks (tb_ap040_   //
// pipe_excfc.v covers those). $F144's two moments are this core's own:     //
// a frame write of vector 32 on the bus, and vector 32's handler address   //
// arriving. The first was a frame beat leaving EA-fetch until the store    //
// buffer (caches stage E): that can come before $F144's own write, still   //
// buffered, reaches the bus and arms it -- on the bus the two keep their   //
// order.                                                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_program;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

wire [15:0] data_in;
wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds;
wire  [1:0] busstate;
wire        longword;
wire  [2:0] fc;

reg         mem_ready;
wire        walker_req, walker_we;
wire [31:0] walker_addr, walker_wdat;
reg         walker_ack = 0;
reg  [31:0] walker_data = 0;
reg         walker_berr_r = 0;   // one-shot walker bus error, armed via $F146
reg         wberr_arm = 0;
reg         berr_armed;
reg         fberr_armed = 0;
reg  [15:0] poke_addr = 0;
reg  [15:0] dma_addr = 0;        // $F15C/$F15E: a chipset write, snooped
reg         dma_snoop = 0;
// RESET's line (caches stage F, the card's wrapper: cpu_wrapper.v resets
// the chipset's side from it). It rises only once every earlier write is
// done -- a write the program made before RESET reaches its device before
// the device is reset; the manual gives no order, program order does --
// the bus stays idle while it is up, it is up for 128 of the core's cycles
// (ap040_core.v's S_RESET_HOLD), and $F166 counts it. Each level 7
// interrupt taken is one nmi_ack pulse.
reg         rst_q = 0;
integer     rst_n = 0, rst_len = 0, nmi_ent = 0, nmi_acks = 0;
reg  [15:0] peek_addr = 0;
reg  [15:0] fberr_addr = 0;
// $F156: a one-shot bus error on the next data write sub-cycle to the
// longword written there (0 disarms) -- a posted write's, a push's, an
// exception frame's (caches stage R). $F158: the same for a data read -- a
// line read's beat, say. $F15A: how many cycles the armed write sub-cycle is
// held before it errs (0: at once) -- so a program can have instructions
// arrive while a write's bus error is still to come.
reg         wberr_d_armed = 0;
reg  [15:0] wberr_d_addr = 0;
reg  [15:0] wberr_d_delay = 0;
reg  [15:0] wberr_d_wait = 0;
reg         rberr_d_armed = 0;
reg  [15:0] rberr_d_addr = 0;
wire        berr_d = berr_armed && nreset && (busstate != 2'b01) &&
                     (addr_out[15:0] == 16'hF140);
wire        fberr  = fberr_armed && nreset && (busstate == 2'b00) &&
                     (addr_out[15:0] == fberr_addr);
wire        wberr_hit = wberr_d_armed && nreset && (busstate == 2'b11) &&
                        (addr_out[15:2] == wberr_d_addr[15:2]);
wire        wberr_d = wberr_hit && (wberr_d_wait >= wberr_d_delay);
wire        rberr_d = rberr_d_armed && nreset && (busstate == 2'b10) &&
                      (addr_out[15:2] == rberr_d_addr[15:2]);
wire        berr   = berr_d | fberr | wberr_d | rberr_d;

wire        bus_clkena = (busstate == 2'b01) | mem_ready | berr;

reg   [2:0] ipl_lvl;
reg   [1:0] irq_exc_armed = 0;
reg   [2:0] irq_fetch_stall = 0;
reg  [15:0] ipl_delay = 0;
reg   [7:0] ipl_pulse = 0;
reg   [7:0] ipl_step  = 0;
reg   [2:0] ipl_next  = 0;
integer     clkcount = 0, stamp_prev = 0;
always @(posedge clk) clkcount = clkcount + 1;

// The core's enable, pseudo-random under AP040_PIPE_CE_RANDOM as in every
// milestone bench; the bus keeps its own.
reg ce = 1;
`ifdef AP040_PIPE_CE_RANDOM
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`endif

ap040_pipe_bus16 #(
	.PC_RESET     (32'h0000_0400),
	.PROG_WORDS   (32'h7FFF_FFFF),
	.RESET_VECTORS(1)
) dut
(
	.clk (clk), .nreset (nreset), .ce (ce), .clkena_in (bus_clkena),
	.irq_lvl (ipl_lvl), .berr (berr),
	// production's windows: chip RAM (every program here) cacheable, the
	// rest not; the chipset's writes from $F15C/$F15E (caches stage F)
	.cache_allow_all (1'b0), .cache_z2_ena (1'b1), .cache_z3_base0 (5'd8), .cache_z3_ena0 (1'b1),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (dma_snoop), .snoop_addr ({16'd0, dma_addr}),
	.walker_req (walker_req), .walker_we (walker_we), .walker_addr (walker_addr),
	.walker_wdat (walker_wdat), .walker_ack (walker_ack), .walker_data (walker_data),
	.walker_berr (walker_berr_r),
	.data_in (data_in), .addr_out(addr_out), .data_write(data_write),
	.nwr (nwr), .nuds(nuds), .nlds(nlds),
	.busstate(busstate), .longword(longword), .fc(fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid(), .dbg_eac_pc(), .dbg_eaf_valid(), .dbg_eaf_pc(),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0(), .dbg_d1(), .dbg_d2(), .dbg_d3(),
	.dbg_d4(), .dbg_d5(), .dbg_d6(), .dbg_d7(),
	.dbg_ccr(), .dbg_sr(), .dbg_commits()
);

wire [31:0] dbg_pc = dut.u_cpu.u_eaf.eac_pc;

//---------------------------------------------------------------------------
// 64 KB memory, three phases
//---------------------------------------------------------------------------

reg [15:0] mem [0:32767];
reg        lvl_hold;
reg [15:0] lvl_data;
integer errors, phase, result;
assign data_in = (phase == 2 && lvl_hold) ? lvl_data :
                 (addr_out[15:0] == 16'hF13A) ? mem[peek_addr[15:1]] : mem[addr_out[15:1]];
reg [1023:0] prog_file;
integer prog_fd;

function [2:0] latency;
	input integer ph;
	input integer n;
	begin
		if (ph == 0) latency = 0;
		else         latency = (n * 7 + 3) % 6;
	end
endfunction

reg [2:0] lat_cnt;
integer   lat_idx;

always @(posedge clk) begin
	if (phase != 2) mem_ready <= 0;
	if (!nreset) begin
		mem_ready <= 0; lvl_hold <= 0;
		lat_cnt <= latency(phase, 0); lat_idx <= 1;
		berr_armed <= 1;
	end
	else if (berr) begin
		// one physical bus error per arming; the restarted access succeeds
		if (berr_d) berr_armed  <= 0;
		if (fberr)  fberr_armed <= 0;
		if (wberr_d) begin wberr_d_armed <= 0; wberr_d_wait <= 0; end
		if (rberr_d) rberr_d_armed <= 0;
		mem_ready <= 0; lvl_hold <= 0;
	end
	else if (wberr_hit) begin
		// the armed write sub-cycle, held until its error is due
		mem_ready <= 0; lvl_hold <= 0;
		wberr_d_wait <= wberr_d_wait + 16'd1;
	end
	else if (phase == 2) begin
		if (busstate == 2'b01) begin mem_ready <= 0; lvl_hold <= 0; end
		else if (!lvl_hold) begin
			if (lat_cnt == 0) begin
				mem_ready <= 1; lvl_hold <= 1;
				lvl_data  <= (addr_out[15:0] == 16'hF13A) ? mem[peek_addr[15:1]] : mem[addr_out[15:1]];
				lat_cnt <= latency(phase, lat_idx); lat_idx <= lat_idx + 1;
			end
			else lat_cnt <= lat_cnt - 1'd1;
		end
	end
	else if (irq_fetch_stall != 0 && busstate != 2'b01 && !mem_ready) begin
		// keep the handler's first fetch outstanding while the level syncs
		irq_fetch_stall <= irq_fetch_stall - 1'd1;
	end
	else if (busstate != 2'b01 && !mem_ready) begin
		if (lat_cnt == 0) begin
			mem_ready <= 1;
			lat_cnt <= latency(phase, lat_idx); lat_idx <= lat_idx + 1;
		end
		else lat_cnt <= lat_cnt - 1'd1;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF142)
		berr_armed <= 1;
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF154) begin
		fberr_armed <= |data_write;
		fberr_addr  <= data_write;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF156) begin
		wberr_d_armed <= |data_write;
		wberr_d_addr  <= data_write;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF15A)
		wberr_d_delay <= data_write;
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF158) begin
		rberr_d_armed <= |data_write;
		rberr_d_addr  <= data_write;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF144)
		irq_exc_armed <= data_write[1:0];
	else if (irq_exc_armed == 1 && dut.u_cpu.u_eaf.exc_go && dut.u_cpu.u_eaf.exc_vec_r == 8'd32 &&
	         mem_ready && busstate == 2'b11) begin
		ipl_lvl <= 3'd2;
		irq_exc_armed <= 0;
	end
	else if (irq_exc_armed == 2 && dut.u_cpu.u_eaf.exc_vec_done && dut.u_cpu.u_eaf.exc_vec_r == 8'd32) begin
		ipl_lvl <= 3'd2;
		irq_exc_armed <= 0;
		irq_fetch_stall <= 3'd5;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF148)
		ipl_delay <= data_write;
	else if (ipl_delay != 0) begin
		ipl_delay <= ipl_delay - 1'd1;
		if (ipl_delay == 16'd1) ipl_lvl <= 3'd2;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF14C) begin
		ipl_lvl   <= data_write[2:0];
		ipl_pulse <= data_write[15:8];
	end
	else if (ipl_pulse != 0) begin
		ipl_pulse <= ipl_pulse - 1'd1;
		if (ipl_pulse == 8'd1) ipl_lvl <= 3'd0;
	end
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF150) begin
		ipl_lvl  <= data_write[2:0];
		ipl_next <= data_write[6:4];
		ipl_step <= data_write[15:8];
	end
	else if (ipl_step != 0) begin
		ipl_step <= ipl_step - 1'd1;
		if (ipl_step == 8'd1) ipl_lvl <= ipl_next;
	end
end

// $F146 arms a one-shot bus error on the NEXT table-walker descriptor
// access (PTEST's MMUSR B bit).
always @(posedge clk)
	if (nreset && mem_ready && busstate == 2'b11 && addr_out[15:0] == 16'hF146)
		wberr_arm <= 1;

// The table walker's own 32-bit physical port, tb_ap040_program.v's model:
// an independent latency profile, and never an acknowledge on the 16-bit
// bus, so a descriptor leaking onto that bus fails every MMU test.
                                                                            //
// Ordering (2026-09-25). A table search must see every write the core has
// committed to: a descriptor stored and then searched through is read as
// stored (t_walk_order.s). The sequential core's bench asserted, and this
// one did until the MMU moved to the memory units' ports, that the walker
// and the 16-bit bus are never active together -- which held because a
// walk only ever ran for the transaction at the head of the bus. With
// translation beside the pipeline a search runs while fetches and reads
// are on the bus, as the units are meant to; what must not happen is a
// search overtaking a WRITE. So the bytes of each write are counted from
// the cycle the memory side commits to it -- the DMU accepting a write
// translation could have refused, or passing an untranslated one to the bus
// controller -- until they land on the 16-bit bus, and the walker may start
// an access only with none outstanding. A write a bus error aborted never
// lands: the count is cleared once the memory side holds no write, and at
// that point it must be zero unless such an abort happened. Copyback
// (caches stage D): a write the data cache takes instead of sending has
// landed there -- the walker searches through that cache -- and a push, or
// a copyback write sent alone, is counted from the cycle the DMU hands it
// on. Since caches stage E every write the DMU sends goes into its store
// buffer first: held there, it has not landed.
reg        walker_pending, walker_armed, walker_we_latch;
reg [31:0] walker_addr_latch, walker_wdat_latch;
integer    wo_pend = 0;         // committed write bytes not yet landed
reg        wo_lost = 0;         // a write was aborted by a bus error since the last clear
function integer sz_bytes;
	input [1:0] sz;
	begin
		sz_bytes = (sz == `AP040_SZ_L) ? 4 : (sz == `AP040_SZ_W) ? 2 : 1;
	end
endfunction
// The memory side's commitment: the DMU's acceptance of a tentative write
// (its wr_busy low: w_acc), or an untranslated write taken into its store
// buffer.
wire wo_commit_t = dut.u_dmu.w_acc;
wire wo_commit_u = dut.u_dmu.w_thru;
wire wo_cb       = dut.u_dmu.s_cb;                              // into the cache, not the bus
wire wo_push     = dut.u_dmu.pb_wr || dut.u_dmu.su_pw;          // the cache's own writes
wire wo_holds    = (dut.u_dmu.ws == 3'd5) || dut.u_bus.w_pend ||  // WS_POST, or with membus,
                   !dut.u_dmu.sb_empty ||                       // or in the store buffer,
                   dut.u_dmu.pb_act || dut.u_dmu.su_post;       // or the cache's to send
always @(posedge clk) begin
	if (!nreset) begin
		wo_pend = 0; wo_lost = 0;
	end else begin
		// a walker access starting in the cycle just ended (the model below
		// takes it at this edge), judged on the count before this edge's
		if (walker_req && walker_armed && !walker_pending && wo_pend != 0) begin
			errors = errors + 1;
			$display("FAIL: table walker access at %h with %0d committed write bytes not yet on the bus (pc=%h)",
			         walker_addr, wo_pend, dbg_pc);
			result = 2;
		end
		if (wo_commit_t) wo_pend = wo_pend + sz_bytes(dut.u_dmu.w_size);
		if (wo_commit_u) wo_pend = wo_pend + sz_bytes(dut.l1_size_b);
		if (wo_cb)       wo_pend = wo_pend - sz_bytes(dut.u_dmu.w_size);
		if (wo_push)     wo_pend = wo_pend + (dut.u_dmu.pb_wr ? 4 : sz_bytes(dut.u_dmu.su_size));
		if (mem_ready && busstate == 2'b11) wo_pend = wo_pend - (!nuds ? 1 : 0) - (!nlds ? 1 : 0);
		if (berr && busstate == 2'b11) wo_lost = 1;
		if (!wo_commit_t && !wo_commit_u && !wo_push && !wo_holds) begin
			if (wo_pend != 0 && !wo_lost) begin
				errors = errors + 1;
				$display("FAIL: %0d write bytes committed but never written on the bus (pc=%h)", wo_pend, dbg_pc);
				result = 2;
			end
			wo_pend = 0; wo_lost = 0;
		end
	end
end
reg  [2:0] walker_lat_cnt;
integer    walker_lat_idx;
always @(posedge clk) begin
	walker_ack    <= 0;
	walker_berr_r <= 0;
	if (!nreset) begin
		walker_pending <= 0; walker_armed <= 1; walker_we_latch <= 0;
		walker_addr_latch <= 0; walker_wdat_latch <= 0; walker_data <= 0;
		walker_lat_cnt <= latency(phase, 0); walker_lat_idx <= 1;
	end else begin
		if (!walker_req) walker_armed <= 1;
		if (walker_req && walker_armed && !walker_pending) begin
			walker_pending    <= 1;
			walker_armed      <= 0;
			walker_we_latch   <= walker_we;
			walker_addr_latch <= walker_addr;
			walker_wdat_latch <= walker_wdat;
			walker_lat_cnt    <= latency(phase, walker_lat_idx);
			walker_lat_idx    <= walker_lat_idx + 1;
		end else if (walker_pending) begin
			if (walker_lat_cnt != 0) walker_lat_cnt <= walker_lat_cnt - 1'd1;
			else if (wberr_arm) begin
				wberr_arm <= 0; walker_pending <= 0; walker_berr_r <= 1;
			end else begin
				if (walker_addr_latch[31:16] != 0 || walker_addr_latch[1:0] != 0) begin
					errors = errors + 1;
					$display("FAIL: invalid walker address %h", walker_addr_latch);
					result = 2;
				end else if (walker_we_latch) begin
					mem[walker_addr_latch[15:1]] = walker_wdat_latch[31:16];
					mem[walker_addr_latch[15:1] + 1'b1] = walker_wdat_latch[15:0];
				end else
					walker_data <= {mem[walker_addr_latch[15:1]], mem[walker_addr_latch[15:1] + 1'b1]};
				walker_pending <= 0;
				walker_ack     <= 1;
			end
		end
	end
end

//---------------------------------------------------------------------------
// interrupt invariants, on this core's acceptance (u_eaf's irq_take)
//---------------------------------------------------------------------------

reg  [15:0] ipl_idle_for = 0;
reg  [15:0] claim_age [1:6];
integer    ca;
initial for (ca = 1; ca <= 6; ca = ca + 1) claim_age[ca] = 0;
reg   [6:1] tb_qual = 0;
integer     ql;
wire        acc     = dut.u_cpu.irq_ack;
wire  [2:0] acc_lvl = dut.u_cpu.irq_take_lvl;
always @(posedge clk) begin
	if (!nreset) tb_qual <= 0;
	else begin
		for (ql = 1; ql <= 6; ql = ql + 1) begin
			if (dut.u_cpu.u_irq.lvl < ql)
				tb_qual[ql] <= 0;
			// against the mask as it stands once an acceptance has raised it:
			// the SR register only shows the entry's new mask when the entry
			// retires, and a claim re-made in between is not one (u_irq's mask_c)
			else if (dut.u_cpu.u_irq.lvl == ql && ql > dut.u_cpu.u_irq.mask_c)
				tb_qual[ql] <= 1;
		end
		if (acc && acc_lvl != 3'd7 && acc_lvl != 3'd0) tb_qual[acc_lvl] <= 0;
	end
	if (ipl_lvl == 3'd0) begin
		if (ipl_idle_for != 16'hffff) ipl_idle_for <= ipl_idle_for + 1'd1;
	end
	else ipl_idle_for <= 0;
	// A claim is honoured: level L qualified (tb_qual[L]) and is still what
	// the core sees, so it is taken within a generous bound -- whatever the
	// mask has done since. IPEND, t_exceptions' test 136, independent of
	// where a request happens to land in an instruction.
	for (ca = 1; ca <= 6; ca = ca + 1) begin
		if (!nreset || !tb_qual[ca] || dut.u_cpu.u_irq.lvl != ca) claim_age[ca] <= 0;
		else claim_age[ca] <= claim_age[ca] + 1'd1;
		if (nreset && claim_age[ca] == 16'd400) begin
			errors = errors + 1;
			$display("FAIL: a qualified level %0d request is still not taken after 400 cycles (claim lost, pc=%h)", ca, dbg_pc);
		end
	end
	if (nreset && acc && ipl_idle_for > 16'd12) begin
		errors = errors + 1;
		$display("FAIL: interrupt accepted %0d cycles after the level went idle (phantom)", ipl_idle_for);
	end
	if (nreset && acc && acc_lvl != 3'd7 && acc_lvl <= dut.u_cpu.sr_resolved_ea[10:8] && !tb_qual[acc_lvl]) begin
		errors = errors + 1;
		$display("FAIL: level %0d interrupt accepted at or below mask %0d (pc=%h)",
		         acc_lvl, dut.u_cpu.sr_resolved_ea[10:8], dbg_pc);
	end
end

//---------------------------------------------------------------------------
// bus monitor and write commit
//---------------------------------------------------------------------------

always @(posedge clk) begin
	dma_snoop <= 1'b0;
	if (!nreset) begin
		rst_n = 0; rst_len = 0; nmi_ent = 0; nmi_acks = 0;
		mem[15'h78B3] = 16'h0000;
	end else begin
		if (dut.reset_out && !rst_q) begin
			rst_n = rst_n + 1;
			if (!dut.u_dmu.sb_empty || dut.u_bus.w_pend) begin
				errors = errors + 1;
				$display("FAIL: RESET drove the reset line with a write still pending (pc=%h)", dbg_pc);
				result = 2;
			end
		end
		if (dut.reset_out && busstate != 2'b01) begin
			errors = errors + 1;
			$display("FAIL: a bus cycle while RESET drove the reset line (pc=%h)", dbg_pc);
			result = 2;
		end
		if (dut.reset_out && dut.ce) rst_len = rst_len + 1;
		if (!dut.reset_out && rst_q) begin
			if (rst_len != 128) begin
				errors = errors + 1;
				$display("FAIL: RESET drove the reset line for %0d cycles, not 128 (pc=%h)", rst_len, dbg_pc);
				result = 2;
			end
			rst_len = 0;
			mem[15'h78B3] = rst_n;
		end
		if (dut.ce && dut.u_cpu.u_eaf.exc_vec_done && !dut.u_cpu.u_eaf.stall_in && dut.u_cpu.u_eaf.eac_is_irq &&
		    (dut.u_cpu.u_eaf.exc_vec_r == 8'd31)) nmi_ent = nmi_ent + 1;
		if (dut.nmi_ack) nmi_acks = nmi_acks + 1;
	end
	rst_q <= dut.reset_out;
	if (nreset && mem_ready) begin
		if (addr_out[31:16] != 0) begin
			errors = errors + 1;
			$display("FAIL: access outside memory model at %h (pc=%h)", addr_out, dbg_pc);
			result = 2;
		end
		if (busstate == 2'b11) begin
			if (!nuds) mem[addr_out[15:1]][15:8] = data_write[15:8];
			if (!nlds) mem[addr_out[15:1]][7:0]  = data_write[7:0];
			if (addr_out[15:0] == 16'hF102 && !nuds && !nlds) begin
				if (data_write == 16'h600D) result = 1;
				else begin
					errors = errors + 1;
					$display("FAIL: program reports failure, test %0d (phase %0d, pc=%h)",
					         mem[15'h7880], phase, dbg_pc);
					result = 2;
				end
			end
			if (addr_out[15:0] == 16'hF108) begin
				$display("STAMP tag=%04x cycles=%0d", data_write, clkcount - stamp_prev);
				stamp_prev = clkcount;
			end
			if (addr_out[15:0] == 16'hF110) ipl_lvl <= data_write[2:0];
			if (addr_out[15:1] == (16'hF120 >> 1) && fc !== 3'd1) begin
				errors = errors + 1;
				$display("FAIL: write to F120 with FC=%0d, expected 1", fc);
			end
			if (addr_out[15:0] == 16'hF130) begin
				mem[15'h1A80] = data_write;
				mem[15'h1A81] = 16'h0000;
			end
			if (addr_out[15:0] == 16'hF134) poke_addr = data_write;
			if (addr_out[15:0] == 16'hF138) peek_addr = data_write;
			if (addr_out[15:0] == 16'hF136) mem[poke_addr[15:1]] = data_write;
			if (addr_out[15:0] == 16'hF15C) dma_addr = data_write;
			if (addr_out[15:0] == 16'hF15E) begin
				mem[dma_addr[15:1]] = data_write;
				dma_snoop <= 1'b1;
			end
		end
	end
end

// +exctrace: every exception entry, on the cycle its verdict registers.
reg exc_go_q = 0;
always @(posedge clk) begin
	exc_go_q <= dut.u_cpu.u_eaf.exc_go;
	if ($test$plusargs("exctrace") && nreset && dut.u_cpu.u_eaf.exc_go && !exc_go_q)
		$display("EXC %0t vec=%0d pc=%h sr=%h phase=%0d", $time, dut.u_cpu.u_eaf.exc_vec_r,
		         dut.u_cpu.u_eaf.eac_pc, dut.u_cpu.u_eaf.sr_in, phase);
end

// +storetrace: every store EA-fetch posts, with what the snoop compared.
always @(posedge clk)
	if ($test$plusargs("storetrace") && nreset && dut.u_cpu.eaf_l1_wren_b && !dut.u_cpu.l1_wr_busy)
		$display("ST %0t pc=%h a=%h departs=%b", $time, dut.u_cpu.u_eaf.eac_pc,
		         dut.u_cpu.eaf_l1_addr_b, dut.u_cpu.eaf_departs);
always @(posedge clk)
	if ($test$plusargs("storetrace") && nreset && dut.u_cpu.sq_v)
		$display("SQ %0t a=%h dep=%b eac=%b %h-%h id=%b %h-%h dec=%b %h if=%b %h hit=%b late=%b", $time,
		         dut.u_cpu.sq_a, dut.u_cpu.sq_dep, dut.u_cpu.eac_valid, dut.u_cpu.eac_pc, dut.u_cpu.eac_next_pc,
		         dut.u_cpu.id_valid, dut.u_cpu.id_pc, dut.u_cpu.id_next_pc,
		         dut.u_cpu.dec_holding, dut.u_cpu.dec_hold_pc, dut.u_cpu.if_valid_id, dut.u_cpu.if_pc,
		         dut.u_cpu.smc_hit, dut.u_cpu.st_smc_late);

always @(posedge clk)
	if ($test$plusargs("storetrace") && nreset && dut.u_cpu.ex_st_req && !dut.u_cpu.l1_wr_busy)
		$display("STX %0t a=%h eac=%b %h-%h id=%b %h-%h dec=%b %h if=%b %h smc=%b stall=%b", $time,
		         dut.u_cpu.snx_a, dut.u_cpu.eac_valid, dut.u_cpu.eac_pc, dut.u_cpu.eac_next_pc,
		         dut.u_cpu.id_valid, dut.u_cpu.id_pc, dut.u_cpu.id_next_pc,
		         dut.u_cpu.dec_holding, dut.u_cpu.dec_hold_pc, dut.u_cpu.if_valid_id, dut.u_cpu.if_pc,
		         dut.u_cpu.st_smc, dut.u_cpu.ex_stall);

// +irqtrace: the interrupt input and its hold, every cycle a level is up or
// the delayed request is counting.
always @(posedge clk)
	if ($test$plusargs("irqtrace") && nreset && (ipl_lvl != 0 || ipl_delay != 0))
		$display("IRQ %0t pins=%0d dly=%0d lvl=%0d hold=%0d mask=%0d live=%0d pend=%b ack=%b eac=%h wb=%b/%h", $time,
		         ipl_lvl, ipl_delay, dut.u_cpu.u_irq.lvl, dut.u_cpu.u_irq.hold, dut.u_cpu.sr[10:8],
		         dut.u_cpu.sr_resolved_ea[10:8], dut.u_cpu.irq_pend, dut.u_cpu.irq_ack, dut.u_cpu.u_eaf.eac_pc,
		         dut.u_cpu.dbg_wb_valid, dut.u_cpu.dbg_wb_pc);

// +trace: every retirement, for a first look at where a program went.
always @(posedge clk)
	if ($test$plusargs("trace") && nreset && dut.u_cpu.dbg_wb_valid)
		$display("TRACE %0t wb pc=%h sr=%h d0=%h d7=%h a7=%h", $time, dut.u_cpu.dbg_wb_pc,
		         dut.u_cpu.sr, dut.u_cpu.u_regfile.dreg[0], dut.u_cpu.u_regfile.dreg[7],
		         dut.u_cpu.u_regfile.isp);

// +stagetrace=<first time in ps>: every stage, every cycle, for 400 cycles
// from then -- where an instruction stopped, and what held it.
reg [63:0] st_from = 0;
integer    st_left = 400;
initial if (!$value$plusargs("stagetrace=%d", st_from)) st_from = 0;
always @(posedge clk)
	if (st_from != 0 && nreset && $time >= st_from && st_left > 0) begin
		st_left = st_left - 1;
		$display("ST %0t id %b/%h eac %b/%h eaf %b/%h ex %b/%h wb %b/%h | stall ea %b ex %b hz %b busy %b wflt %b idle %b quiet %b pend a%b b%b w%b | bus %b%b %h ack %b flt %b rv_b %b/%b | exc go %b ph %0d vec %0d aerr %b/%b owe %b",
		         $time, dut.u_cpu.id_valid, dut.u_cpu.u_id.id_pc, dut.u_cpu.eac_valid, dut.u_cpu.u_eaf.eac_pc,
		         dut.u_cpu.u_eaf.eaf_valid, dut.u_cpu.u_eaf.eaf_pc, dut.u_cpu.exe_valid, dut.u_cpu.u_ex.exe_pc,
		         dut.u_cpu.dbg_wb_valid, dut.u_cpu.dbg_wb_pc,
		         dut.u_cpu.u_eaf.eaf_stall, dut.u_cpu.u_ex.ex_stall, dut.u_cpu.u_eaf.hold_hazard,
		         dut.u_bus.busy, dut.l1_wflt, dut.l1_idle, dut.l1_quiet,
		         dut.u_imu.a_pend, dut.u_bus.b_pend, dut.u_bus.w_pend,
		         dut.u_bus.mem_req, dut.u_bus.mem_write, dut.u_bus.mem_addr, dut.u_bus.mem_ack, dut.u_bus.mem_flt,
		         dut.u_bus.rvalid_b, dut.u_bus.rflt_b,
		         dut.u_cpu.u_eaf.exc_go, dut.u_cpu.u_eaf.exc_ph, dut.u_cpu.u_eaf.exc_vec_r,
		         dut.u_cpu.u_eaf.aerr_now, dut.u_cpu.u_eaf.exc_pend_aerr, dut.u_cpu.u_eaf.owe);
	end

// MC68040UM 10.3: CINV, CPUSH, PFLUSH and PTEST let every earlier write
// complete first -- software pushes the cache before a DMA reads memory,
// and memory must then hold everything written. The program cannot see
// this itself (its own reads wait for the store buffer), so the bench
// checks it where the instruction starts: no write in the DMU's store
// buffer, none with the bus controller (caches stage E).
always @(posedge clk)
	if (nreset && dut.ce && dut.u_cpu.u_eaf.pm_start && (!dut.u_dmu.sb_empty || dut.u_bus.w_pend)) begin
		errors = errors + 1;
		$display("FAIL: a cache or ATC maintenance instruction started with a write still pending (pc=%h)", dbg_pc);
		result = 2;
	end

//---------------------------------------------------------------------------
// phase driver
//---------------------------------------------------------------------------

integer timeout, i;

task run_phase;
	input integer ph;
	begin
		phase = ph; result = 0; ipl_lvl = 0; fberr_armed = 0; irq_exc_armed = 0; irq_fetch_stall = 0;
		for (i = 0; i < 32768; i = i + 1) mem[i] = 16'h0000;
		prog_fd = $fopen(prog_file, "r");
		if (prog_fd == 0) begin
			$display("FATAL: cannot open program image %0s -- nothing to test", prog_file);
			$fatal(1);
		end
		$fclose(prog_fd);
		$readmemh(prog_file, mem);
		mem[15'h78B0] = 16'h0007;
		nreset = 0;
		repeat (10) @(posedge clk);
		nreset = 1;
		timeout = 0;
		while (result == 0 && timeout < 20000000) begin
			@(posedge clk);
			timeout = timeout + 1;
		end
		if (result == 0) begin
			errors = errors + 1;
			$display("FAIL: phase %0d timeout, pc=%h, last test number %0d", ph, dbg_pc, mem[15'h7880]);
		end
		else if (result == 1)
			$display("phase %0d passed (%0d cycles)", ph, timeout);
		if (nmi_acks != nmi_ent) begin
			errors = errors + 1;
			$display("FAIL: %0d level 7 interrupts taken, %0d NMI acknowledges", nmi_ent, nmi_acks);
		end
	end
endtask

initial begin
	errors = 0;
	if (!$value$plusargs("prog=%s", prog_file)) begin
		$display("FAIL: missing +prog=<hexfile>");
		$finish;
	end
	$display("tb_ap040_pipe_program: running %0s", prog_file);
	run_phase(0);
	run_phase(1);
	run_phase(2);
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
