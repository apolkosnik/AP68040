//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_tg68k_compat.v - top-level adapter presenting a TG68K-like port    //
// set to cpu_wrapper.v (see AP040_IMPLEMENTATION_PLAN.md section 4)        //
//                                                                          //
// The MMU/walker/cache sideband ports exist so the wrapper interface is    //
// stable across the milestones; with the MMU and caches still disabled     //
// they are tied to their idle values and the physical address equals the   //
// logical address.                                                         //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_tg68k_compat
#(
	parameter AP040_HAS_MMU      = 1,
	parameter AP040_HAS_FPU      = 1,
	parameter AP040_ENABLE_CACHE = 1,
	parameter AP040_FAST_SIM     = 0,
	// Post stores to on-board RAM (the MiSTer wrapper's behaviour).  A bench
	// that bus-errors a RAM write to test the fault path sets it to 0.
	parameter AP040_POST_STORES  = 1
)
(
	input         clk,
	input         nreset,
	input         clkena_in,

	// Physical cacheability windows for the internal caches when no MMU
	// translation supplies CM attributes: only configured fast RAM may be
	// cached (chip RAM is chipset-DMA-written and never snooped here; IO
	// and unconfigured space must never be cached).  cache_allow_all
	// bypasses the windows for flat simulation environments.
	input         cache_allow_all,
	// chipset/DMA write snoop, already in this clock domain
	input         cache_snoop_stb,
	input  [31:0] cache_snoop_addr,
	input         cache_z2_ena,
	input   [4:0] cache_z3_base0,
	input         cache_z3_ena0,
	input   [3:0] cache_z3_base1,
	input         cache_z3_ena1,
	input  [15:0] data_in,
	input  [2:0]  ipl,
	input         ipl_autovector,
	input         berr,

	output [31:0] addr_out,
	output [15:0] data_write,
	output        nwr,
	output        nuds,
	output        nlds,
	output [1:0]  busstate,
	output        longword,
	output        nresetout,
	output [2:0]  fc,
	output        nmi_ack_toggle,
	// Cache-maintenance event for systems that compile out ap040_cache and
	// use an external cache on the TG68K bus instead.
	output        cache_maint_req,
	output        cache_maint_ic,
	output        cache_maint_dc,

	output [31:0] mmu_addr_log,
	output [31:0] mmu_addr_phys,
	output        mmu_cache_inhibit,

	output        walker_req,
	output        walker_we,
	output [31:0] walker_addr,
	output [31:0] walker_wdat,
	input         walker_ack,
	input  [31:0] walker_data,
	input         walker_berr,

	output        cache_req,
	output [31:0] cache_addr,
	input  [15:0] cache_data,
	input         cache_ack,
	output        cache_burst,
	output [2:0]  cache_burst_len,
	output [28:1] cache_ramaddr,

	output [31:0] cacr_out,
	output [31:0] vbr_out,
	output        debug_busy,
	output        debug_fault,
	output        debug_halted,
	output [255:0] debug_status,
	output [127:0] debug_status2
);

// core to MMU
wire        mem_req;
wire        mem_write;
wire        mem_instr;
wire  [1:0] mem_size;
wire [31:0] mem_addr;
wire [31:0] mem_hint_addr, mm_hint_addr;
wire        mem_hint_away, mem_fast_ready, mem_ack_q;
wire        mem_hint_instr, mm_hint_instr, mm_hint_match, mm_hint_wmatch;
wire [21:0] mm_hint_ptag;
wire [31:0] mem_ihint_addr, mm_ihint_addr;   // the instruction hint bus (P175)
wire [21:0] mm_ihint_ptag;
wire        mm_ihint_match;
wire [31:0] mem_wdata;
wire  [2:0] mem_fc;
wire        mem_ack;

// ---- P171: two request channels from the core, one port into the MMU ----
// The core keeps the fetch queue's requests (ifr_*) apart from its data
// requests (mem_*), so a data request can be issued and hinted while a
// fetch is outstanding.  One channel is presented to the MMU/cache at a
// time; a presented request is never switched until it completes; when
// both wait, the data request goes first (the CPU is stalled on it, the
// fetch is speculative).  Acknowledge and fault return to the presented
// channel only; the platform's berr belongs to it as well.
wire        ifr_req, ifr_ack, ifr_flt, ifr_berr;
wire [31:0] ifr_addr;
wire  [1:0] ifr_size;
wire  [2:0] ifr_fc;
wire        core_req, core_write, core_instr;   // the data channel as the core drives it
wire  [1:0] core_size;
wire [31:0] core_addr, core_wdata;
wire  [2:0] core_fc;
wire        core_ack, core_flt, core_berr;
reg         pres_v, pres_instr;
// P184: a fetch in its first presented clock that the instruction hint
// carried the clock before (row and word) is served by the instruction
// mirrors (fast_ihit); the architectural banks stay on the data hint then,
// so the data side's idle read is not lost to every one-clock fetch.  If
// the fetch does not hit, the cache holds its registered lookup for that
// clock and the banks follow the fetch from the next (pres_v).
reg   [9:0] ihint_q;
always @(posedge clk) ihint_q <= mem_ihint_addr[11:2];
wire        ifetch_hinted_first = !pres_v && (ihint_q == ifr_addr[11:2]);
wire        sel_instr = pres_v ? pres_instr : !core_req;
// The MMU's W_DROP state (and the original core's exception entry) rely
// on a request-low cycle after a fault before the next request is seen;
// with two channels the other one would otherwise take the port in the
// very next cycle and c_req would never fall.
reg         flt_gap;
assign mem_req   = !flt_gap && (sel_instr ? ifr_req  : core_req);
assign mem_write = sel_instr ? 1'b0     : core_write;
assign mem_instr = sel_instr;
assign mem_size  = sel_instr ? ifr_size : core_size;
assign mem_addr  = sel_instr ? ifr_addr : core_addr;
assign mem_wdata = core_wdata;
assign mem_fc    = sel_instr ? ifr_fc   : core_fc;
wire        p_done = mem_ack | mem_flt | berr;
assign core_ack = mem_ack && !sel_instr;
assign ifr_ack  = mem_ack &&  sel_instr;
// MMU faults and bus errors stay separate (the frame's ATC bit), each
// delivered to the channel presented when it happened.
assign core_flt  = mem_flt && !sel_instr;
assign ifr_flt   = mem_flt &&  sel_instr;
assign core_berr = berr    && !sel_instr;
assign ifr_berr  = berr    &&  sel_instr;
always @(posedge clk) begin
	if (!nreset) begin pres_v <= 0; pres_instr <= 0; flt_gap <= 0; end
	else begin
		flt_gap <= p_done && (mem_flt | berr);
		if (pres_v) begin
			if (p_done) pres_v <= 0;
		end
		else if (mem_req && !p_done) begin
			pres_v <= 1; pres_instr <= sel_instr;
		end
	end
end
wire [31:0] mem_rdata;
wire        mem_flt_mmu;
// Core-side stall watchdog.  Every prior watchdog counts a DOWNSTREAM
// request (CPU port, walker port), so a transaction lost between the
// core and those ports -- or a wedge in the clock-enable machinery the
// downstream layers are gated by -- stalls the core forever with every
// watchdog blind: the live NetBSD freeze shows exactly that (identical
// silent halt across three memory-path-hardened builds, no fault frame
// ever stacked).  This one watches the CORE's own held request on the
// free-running clock (a clkena wedge cannot stop the count) and injects
// an access error through the same mem_flt input the MMU uses; the
// well-tested fault path then reports it.  2^21 cycles at 28 MHz is
// ~75 ms -- beyond every legitimate stall including all downstream
// timeout chains.  If a freeze persists with no fault reported even
// with this armed, the core is not holding a request at all: a
// clock-enable or internal-FSM wedge, which is itself the decisive
// diagnostic.
wire        core_stall_flt;
ap040_bus_timeout #(.COUNTER_BITS(21)) core_stall_watchdog (
	.clk(clk),
	.nreset(nreset),
	.req(mem_req),
	.complete(mem_ack | mem_flt_mmu),
	.berr(core_stall_flt)
);
wire        mem_flt = mem_flt_mmu | core_stall_flt;

// MMU to cache
wire        mm_req, mm_write, mm_instr;
wire  [1:0] mm_size;
wire [31:0] mm_addr, mm_wdata;
wire  [2:0] mm_fc;
wire        mm_ack, mm_nocache;
wire [31:0] mm_rdata;
wire        mm_line_stb, mem_line_stb, mm_busy;
wire        mmu_walker_req;
// a fill or posted store may still own the CPU bus after its requester
// was released: hold the walker off, as wombat_cpu does
assign walker_req = mmu_walker_req && !mm_busy;
wire [31:4] mm_line_tag, mem_line_tag;
wire [127:0] mm_line_data, mem_line_data;

// cache to bus adapter
wire        b_req, b_write, b_instr;
wire  [1:0] b_size;
wire [31:0] b_addr, b_wdata;
wire  [2:0] b_fc;
wire        b_ack;
wire [31:0] b_rdata;

// CINV sideband
wire        cinv_req, cinv_ic, cinv_dc, cinv_done;

// control registers and PTEST/PFLUSH sideband
wire [31:0] w_tc, w_urp, w_srp, w_itt0, w_itt1, w_dtt0, w_dtt1;
wire        pt_req, pt_write, pt_done;
wire [31:0] pt_addr, pt_mmusr;
wire  [2:0] pt_fcw;
wire        pf_req, pf_done;
wire  [1:0] pf_mode;
wire [31:0] pf_addr;
wire  [2:0] pf_fcw;

ap040_core #(
	.AP040_HAS_MMU(AP040_HAS_MMU),
	.AP040_HAS_FPU(AP040_HAS_FPU),
	.AP040_ENABLE_CACHE(AP040_ENABLE_CACHE),
	.AP040_FAST_SIM(AP040_FAST_SIM)
) core (
	.clk(clk),
	.nreset(nreset),
	.ce(clkena_in),

	.mem_req(core_req),
	.mem_write(core_write),
	.mem_instr(core_instr),
	.mem_size(core_size),
	.mem_addr(core_addr),
	.mem_hint_addr(mem_hint_addr),
	.mem_hint_instr(mem_hint_instr),
	.mem_hint_away(mem_hint_away),
	.mem_fast_ready(mem_fast_ready),
	.mem_ack_q(mem_ack_q),
	.mem_ihint_addr(mem_ihint_addr),
	.mem_wdata(core_wdata),
	.mem_fc(core_fc),
	.mem_ack(core_ack),
	.ifr_req(ifr_req), .ifr_addr(ifr_addr), .ifr_size(ifr_size), .ifr_fc(ifr_fc),
	.ifr_ack(ifr_ack), .ifr_flt(ifr_flt), .ifr_berr(ifr_berr), .ifr_pres(sel_instr),
	.mem_rdata(mem_rdata),
	.mem_line_stb(mem_line_stb),
	.mem_line_tag(mem_line_tag),
	.mem_line_data(mem_line_data),
	.mem_flt(core_flt),

	.tc_out(w_tc),
	.urp_out(w_urp),
	.srp_out(w_srp),
	.itt0_out(w_itt0),
	.itt1_out(w_itt1),
	.dtt0_out(w_dtt0),
	.dtt1_out(w_dtt1),
	.pt_req(pt_req),
	.pt_write(pt_write),
	.pt_addr(pt_addr),
	.pt_fc(pt_fcw),
	.pt_done(pt_done),
	.pt_mmusr(pt_mmusr),
	.pf_req(pf_req),
	.pf_mode(pf_mode),
	.pf_addr(pf_addr),
	.pf_fc(pf_fcw),
	.pf_done(pf_done),
	.cinv_req(cinv_req),
	.cinv_ic(cinv_ic),
	.cinv_dc(cinv_dc),
	.cinv_done(cinv_done),

	.ipl(ipl),
	.ipl_autovector(ipl_autovector),
	.berr(core_berr),
	.nmi_ack_toggle(nmi_ack_toggle),

	.nresetout(nresetout),
	.cacr_out(cacr_out),
	.vbr_out(vbr_out),

	.debug_busy(debug_busy),
	.debug_fault(debug_fault),
	.debug_halted(debug_halted),
	.debug_status(debug_status),
	.debug_status2(debug_status2)
);

ap040_mmu mmu (
	.clk(clk),
	.nreset(nreset),
	.ce(clkena_in),

	.tc(w_tc),
	.urp(w_urp),
	.srp(w_srp),
	.itt0(w_itt0),
	.itt1(w_itt1),
	.dtt0(w_dtt0),
	.dtt1(w_dtt1),

	.c_req(mem_req),
	.c_write(mem_write),
	.c_instr(mem_instr),
	.c_size(mem_size),
	.c_addr(mem_addr),
	.c_hint_addr(mem_hint_addr),
	.c_hint_instr(mem_hint_instr),
	.c_ihint_addr(mem_ihint_addr),
	.c_wdata(mem_wdata),
	.c_fc(mem_fc),
	.c_ack(mem_ack),
	.c_rdata(mem_rdata),
	.c_line_stb(mem_line_stb),
	.c_line_tag(mem_line_tag),
	.c_line_data(mem_line_data),
	.c_flt(mem_flt_mmu),

	.pt_req(pt_req),
	.pt_write(pt_write),
	.pt_addr(pt_addr),
	.pt_fc(pt_fcw),
	.pt_done(pt_done),
	.pt_mmusr(pt_mmusr),

	.pf_req(pf_req),
	.pf_mode(pf_mode),
	.pf_addr(pf_addr),
	.pf_fc(pf_fcw),
	.pf_done(pf_done),

	.m_req(mm_req),
	.m_write(mm_write),
	.m_instr(mm_instr),
	.m_size(mm_size),
	.m_addr(mm_addr),
	.m_hint_addr(mm_hint_addr),
	.m_hint_instr(mm_hint_instr),
	.m_hint_ptag(mm_hint_ptag),
	.m_hint_match(mm_hint_match),
	.m_hint_wmatch(mm_hint_wmatch),
	.m_ihint_addr(mm_ihint_addr),
	.m_ihint_ptag(mm_ihint_ptag),
	.m_ihint_match(mm_ihint_match),
	.m_wdata(mm_wdata),
	.m_fc(mm_fc),
	.m_ack(mm_ack),
	.m_rdata(mm_rdata),
	.m_line_stb(mm_line_stb),
	.m_line_tag(mm_line_tag),
	.m_line_data(mm_line_data),

	.walker_req(mmu_walker_req),
	.walker_we(walker_we),
	.walker_addr(walker_addr),
	.walker_wdat(walker_wdat),
	.walker_ack(walker_ack),
	.walker_data(walker_data),
	.walker_berr(walker_berr),

	.phys_addr(mmu_addr_phys),
	.cache_inhibit(mmu_cache_inhibit),
	.m_nocache(mm_nocache)
);

// The MMU's table walker writes U/M bits into page descriptors over its
// own port, behind the data cache.  Nothing else invalidates those lines,
// so a descriptor the CPU had previously read AS DATA would go stale --
// the last coherence hole once the internal caches are enabled (audit
// 5.5).  The walker sits inside this module, so the invalidate is
// generated here rather than plumbed through the SoC: its address is
// already physical, and the cache is physically tagged.
//
// The chipset pulse arrives from a CDC edge detector and cannot be held,
// so it wins the port; the walker's own invalidate waits at most a cycle.
// A second walker write cannot arrive that fast (each is a full memory
// transaction), so one pending slot is enough.
reg         wsnp_pend;
reg  [31:0] wsnp_addr;
reg         walker_wr_d;
wire        walker_wr_edge = (walker_req & walker_we) & ~walker_wr_d;

always @(posedge clk) begin
	if (!nreset) begin
		walker_wr_d <= 1'b0;
		wsnp_pend   <= 1'b0;
		wsnp_addr   <= 32'd0;
	end
	else begin
		walker_wr_d <= walker_req & walker_we;
		if (walker_wr_edge) begin
			wsnp_pend <= 1'b1;
			wsnp_addr <= walker_addr;
		end
		else if (wsnp_pend && !cache_snoop_stb)
			wsnp_pend <= 1'b0;      // issued on the port this cycle
	end
end

wire        snp_stb  = cache_snoop_stb | wsnp_pend;
wire [31:0] snp_addr = cache_snoop_stb ? cache_snoop_addr : wsnp_addr;

// a posted store is draining (the bench's FC check attributes those bus
// cycles to the store, not to whatever the core is doing by then)
wire        cache_posting;
generate
if (AP040_ENABLE_CACHE != 0) begin : g_cache
	// With the snoop port wired up, chip RAM is cacheable too: a chipset
	// write invalidates the line before the CPU can see stale data.  ROM
	// and IO stay out (nothing snoops those, and IO must never be cached).
	//
	// DATA ONLY.  The snoop invalidate reaches just the D bank
	// (ap040_cache port B writes row {1'b0, set}, and store_inv
	// likewise), so the sentence above was never true of the I-cache:
	// code written into chip RAM by the blitter, trackdisk DMA, or a CPU
	// decruncher stayed stale in the I bank, and A500-era programs that
	// predate caches never CINV.  Phenomena's Enigma crashed exactly
	// here -- it runs with the internal caches forced off and fails with
	// them on, from the first commit that enabled them.  On a real 040
	// Amiga this cannot happen because 68040.library marks chip RAM
	// noncacheable through the MMU; with the MMU off, nothing does.
	// So instruction fetches from the chip window bypass the cache, and
	// only the snooped D side caches chip RAM.  cache_allow_all (the
	// benches' everything-cacheable mode; production ties it 0) keeps
	// the bypass out of simulation programs, which run at low addresses
	// and would otherwise lose all I-cache coverage.
	wire cache_chip = (mm_addr[31:21] == 11'd0);          // $000000-$1fffff
	wire cache_win =
		((mm_addr[31:27] == cache_z3_base0) && cache_z3_ena0) ||
		((mm_addr[31:28] == cache_z3_base1) && cache_z3_ena1) ||
		(!mm_addr[31:24] && (mm_addr[23] ^ |mm_addr[22:21]) && cache_z2_ena) ||
		cache_chip;
	wire cache_allow = cache_allow_all | cache_win;

	ap040_cache cache (
		.clk(clk),
		.nreset(nreset),
		.ce(clkena_in),

		.ie(cacr_out[15]),
		.de(cacr_out[31]),

		.cinv_req(cinv_req),
		.cinv_ic(cinv_ic),
		.cinv_dc(cinv_dc),
		.cinv_done(cinv_done),

		.c_req(mm_req),
		.c_write(mm_write),
		.c_instr(mm_instr),
		.c_size(mm_size),
		.c_addr(mm_addr),
		.c_hint_addr(mm_hint_addr),
		.c_hint_instr(mm_hint_instr),
		.c_hint_ptag(mm_hint_ptag),
		.c_hint_match(mm_hint_match),
		.c_hint_wmatch(mm_hint_wmatch),
		.c_hint_away(mem_hint_away),
		.c_fast_ready(mem_fast_ready),
		.c_ack_q(mem_ack_q),
		.c_ihint_addr(mm_ihint_addr),
		.c_ihint_ptag(mm_ihint_ptag),
		.c_ihint_match(mm_ihint_match),
		.c_ihold(mem_req && sel_instr && !ifetch_hinted_first),
		.c_wdata(mm_wdata),
		.c_fc(mm_fc),
		.c_nocache(mm_nocache | ~cache_allow |
		           (mm_instr & cache_chip & ~cache_allow_all)),
		// Post stores to on-board RAM as the MiSTer wrapper does (its
		// predicate: RAM or the VRAM window); the cache drains them from
		// its captured copy, so the bench needs no queue of its own.  The
		// bench's magic registers at $F1xx are its I/O (the bus-error
		// target at $F140 among them) and are never posted.
		// Tied low until 2026-09-19, which left every posted-store path
		// unexercised by the suite.
		.c_post_ok((AP040_POST_STORES != 0) && (mm_addr[31:30] == 2'b00) && (mm_addr[15:8] != 8'hF1)),
		// the same on the hint's physical tag (pa[31:10]): the magic page
		// group $F000-$F3FF is excluded whole
		.c_post_ok_hint((AP040_POST_STORES != 0) && (mm_hint_ptag[21:20] == 2'b00) && (mm_hint_ptag[5:0] != 6'b111100)),
		.s_stb(snp_stb),
		.s_addr(snp_addr),
		.c_ack(mm_ack),
		.c_posting(cache_posting),
		.c_rdata(mm_rdata),
		.c_line_stb(mm_line_stb),
		.c_line_tag(mm_line_tag),
		.c_line_data(mm_line_data),
		.c_busy(mm_busy),
		.m_posted(),

		.m_req(b_req),
		.m_write(b_write),
		.m_instr(b_instr),
		.m_size(b_size),
		.m_addr(b_addr),
		.m_wdata(b_wdata),
		.m_fc(b_fc),
		.m_ack(b_ack),
		.m_rdata(b_rdata),
		.m_line_valid(1'b0),
		.m_line_tag(28'd0),
		.m_line_data(128'd0),
		.m_err(berr)
	);
end
else begin : g_nocache
	assign cache_posting = 1'b0;
	// no internal caches: the MMU talks straight to the bus adapter and
	// CINV/CPUSH complete immediately (a 68040 whose caches never fill).
	// The Minimig build uses this and relies on cpu_cache_new in the RAM
	// controllers, which also snoops chipset DMA writes.
	assign b_req    = mm_req;
	assign b_write  = mm_write;
	assign b_instr  = mm_instr;
	assign b_size   = mm_size;
	assign b_addr   = mm_addr;
	assign b_wdata  = mm_wdata;
	assign b_fc     = mm_fc;
	assign mm_ack   = b_ack;
	assign mm_rdata = b_rdata;
	assign mm_line_stb = 1'b0;
	assign mm_line_tag = 28'd0;
	assign mm_line_data = 128'd0;
	assign mm_busy = 1'b0;
	assign cinv_done = 1'b1;
	wire unused_nc = mm_nocache | cinv_req | cinv_ic | cinv_dc |
	                 (|cacr_out);
end
endgenerate

ap040_bus16_adapter bus16 (
	.clk(clk),
	.nreset(nreset),
	.clkena_in(clkena_in),

	.mem_req(b_req),
	.mem_berr(berr),
	.mem_write(b_write),
	.mem_instr(b_instr),
	.mem_size(b_size),
	.mem_addr(b_addr),
	.mem_wdata(b_wdata),
	.mem_fc(b_fc),
	.mem_ack(b_ack),
	.mem_rdata(b_rdata),

	.data_in(data_in),
	.addr_out(addr_out),
	.data_write(data_write),
	.nwr(nwr),
	.nuds(nuds),
	.nlds(nlds),
	.busstate(busstate),
	.longword(longword),
	.fc(fc)
);

assign mmu_addr_log = mem_addr;
assign cache_maint_req = cinv_req;
assign cache_maint_ic  = cinv_ic;
assign cache_maint_dc  = cinv_dc;

// external cache/burst interface idle until milestone G
assign cache_req       = 1'b0;
assign cache_addr      = 32'd0;
assign cache_burst     = 1'b0;
assign cache_burst_len = 3'd0;
assign cache_ramaddr   = 28'd0;

// unused sideband inputs, referenced to keep lint quiet
wire unused_sideband = cache_ack | (|cache_data);

endmodule
