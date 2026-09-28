//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_core.v - CPU core: fetch, decode, execute, exceptions              //
//                                                                          //
// Implemented (milestones B-G, see AP040_IMPLEMENTATION_PLAN.md):          //
//  - full 68000/68010 integer set, 68020+ pieces: all extension word EAs   //
//    incl. memory indirect, 32/64-bit MUL/DIV, LINK.L, TRAPcc, bitfields,  //
//    CAS/CAS2, CHK2/CMP2, and the 040 set: MOVE16, MOVEC registers,        //
//    CINV/CPUSH/PFLUSH/PTEST with MMU/cache sidebands                      //
//  - exceptions: formats $0/$1/$2/$3 ($4 RTE-accepted when built FPU-less //
//    but never generated) and format $7 access errors with                 //
//    instruction restart, MOVEM saved-EA continuation and EA rollback,     //
//    RTE with format validation and $1 throwaway continuation, trace       //
//    (T1/T0), autovectored interrupts with M-bit master/interrupt stack    //
//    switching                                                             //
//  - integrated 68040 FPU arithmetic, conversions, control and condition   //
//    operations; FSAVE/FRESTORE support NULL, IDLE and rev-$40/$41 state   //
//                                                                          //
// Known gaps, all documented in tests/ap040/README:                        //
//  - TAS/CAS/CAS2 are not bus-locked (single-master fabric here)           //
//  - MMU faults and physical berr both raise format $7; the SSW ATC bit   //
//    distinguishes a translation fault from a physical bus error          //
//  - interrupts are always autovectored (ipl_autovector is ignored)        //
//  - true pipelined arithmetic BUSY state frames are not generated         //
//  - access faults replay operands; CT and WB2/WB1 are not generated       //
//                                                                          //
// The whole core advances only when ce (clkena_in) is high.                //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_core
#(
	parameter AP040_HAS_MMU      = 1,
	parameter AP040_HAS_FPU      = 0,
	parameter AP040_ENABLE_CACHE = 0,
	parameter AP040_FAST_SIM     = 0,
	parameter [7:0] AP040_FPU_REVISION = 8'h41   // FPU state-frame ABI: $41 (68040), $40 (older NeXT)
)
(
	input             clk,
	input             nreset,
	input             ce,

	// internal memory transaction to ap040_bus16_adapter
	output reg        mem_req,
	output reg        mem_write,
	output            mem_instr,
	output reg  [1:0] mem_size,
	// The instruction fetch channel (P171): the fetch queue's requests on
	// their own registers, so a data request may be issued -- and hinted --
	// while a fetch is outstanding.  The wrapper presents one channel at a
	// time to the cache and returns that channel's acknowledge and fault.
	output reg        ifr_req,
	output reg [31:0] ifr_addr,
	output reg  [1:0] ifr_size,
	output reg  [2:0] ifr_fc,
	input             ifr_ack,
	input             ifr_flt,     // MMU fault on the fetch channel (P171)
	input             ifr_berr,    // physical bus error on the fetch channel (P171)
	input             ifr_pres,    // the fetch is the channel presented to the MMU/cache now (P171)
	output     [31:0] mem_addr,
	output     [31:0] mem_hint_addr,
	output     [31:0] mem_ihint_addr,   // the instruction side's own hint (P175)   // next access's address, one cycle early
	output            mem_hint_instr,
	output            mem_hint_away,  // the hint is not the presented request (P182)
	input             mem_fast_ready, // the cache can hit a hinted read in one clock now (P182)
	input             mem_ack_q,      // the cache acknowledges from its register this clock (P204)
	output reg [31:0] mem_wdata,
	output      [2:0] mem_fc,
	input             mem_ack,
	input      [31:0] mem_rdata,
	input             mem_flt,     // access error pulse from the MMU
	// Instruction line sideband from the cache (see ap040_cache): the whole
	// line of the last acknowledged instruction hit, one cycle later.
	input             mem_line_stb,
	input      [31:4] mem_line_tag,
	input     [127:0] mem_line_data,

	// MMU control register values and PTEST/PFLUSH sideband
	output     [31:0] tc_out,
	output     [31:0] urp_out,
	output     [31:0] srp_out,
	output     [31:0] itt0_out,
	output     [31:0] itt1_out,
	output     [31:0] dtt0_out,
	output     [31:0] dtt1_out,
	output reg        pt_req,
	output reg        pt_write,
	output reg [31:0] pt_addr,
	output      [2:0] pt_fc,
	input             pt_done,
	input      [31:0] pt_mmusr,
	output reg        pf_req,
	output reg  [1:0] pf_mode,
	output reg [31:0] pf_addr,
	output      [2:0] pf_fc,
	input             pf_done,
	output reg        cinv_req,
	output reg        cinv_ic,
	output reg        cinv_dc,
	input             cinv_done,

	input       [2:0] ipl,
	input             ipl_autovector,
	input             berr,
	// Retained event for legacy peripherals which need to observe that a
	// level-7 interrupt was accepted.  A toggle is used so a slower clock
	// enable domain cannot miss the event.
	output            nmi_ack_toggle,

	output            nresetout,
	output     [31:0] cacr_out,
	output     [31:0] vbr_out,

	output            debug_busy,
	output            debug_fault,
	output            debug_halted,
	output    [255:0] debug_status,
	// Second debug bus, for the halt post-mortem beacon: the stack
	// registers and the faulting address.  A double fault taken while
	// stacking an exception frame is only explicable with these -- the
	// active A7 alone cannot say whether the stack switch failed or the
	// supervisor stack pointer was already wrong.
	output    [127:0] debug_status2
);

// Revision $40 omits CMDREG3B and the following reserved longword from
// UNIMP frames. Old non-Turbo NeXT kernels depend on this 44-byte layout.
// Keep $41 as the default for existing integrations (52-byte UNIMP).
localparam FPU_REV40 = AP040_FPU_REVISION == 8'h40;
localparam [31:0] FPU_IDLE_HEADER = {AP040_FPU_REVISION, 24'd0};
localparam [31:0] FPU_UNIMP_HEADER = {AP040_FPU_REVISION,
	(FPU_REV40 ? 8'h28 : 8'h30), 16'd0};
localparam [31:0] FPU_BUSY_HEADER = {AP040_FPU_REVISION, 8'h60, 16'd0};
localparam [31:0] FPU_UNIMP_BYTES = FPU_REV40 ? 32'd44 : 32'd52;
localparam [3:0] FPU_UNIMP_LAST = FPU_REV40 ? 4'd10 : 4'd12;
// synthesis translate_off
initial begin
	if (AP040_FPU_REVISION != 8'h40 && AP040_FPU_REVISION != 8'h41)
		$fatal(1, "AP040_FPU_REVISION must be 0x40 or 0x41");
end
// synthesis translate_on

//---------------------------------------------------------------------------
// architectural state
//---------------------------------------------------------------------------

reg [31:0] pc;          // next word to fetch from the instruction stream
reg [31:0] pc_i;        // address of the current instruction
reg [15:0] sr;
reg [31:0] vbr;
reg [31:0] cacr;
reg  [2:0] sfc, dfc;
reg [31:0] tc;          // 040 TC (E/P bits stored, used at milestone E)
reg [31:0] itt0, itt1, dtt0, dtt1;
reg [31:0] mmusr;
reg [31:0] urp, srp;
reg [15:0] ir;
// Simulation dispatch event, including consecutive same-state retirement.
// No hardware consumer: synthesis trims this internal profiling register.
// Sample after eval/NBA, and resynchronize the observer while reset is low.
reg perf_dispatch_toggle /* verilator public_flat_rd */;
// FPGA power-up distinguishes the first (cold) reset from later RSTI
// assertions.  The 68040 preserves MMU register contents on reset except for
// the E bits in TC and the four TTRs.
reg        mmu_reset_seen = 1'b0;

wire sr_s = sr[`AP040_SR_S];
wire sr_m = sr[`AP040_SR_M];

assign vbr_out  = vbr;
assign cacr_out = cacr;
assign tc_out   = tc;
assign urp_out  = urp;
assign srp_out  = srp;
assign itt0_out = itt0;
assign itt1_out = itt1;
assign dtt0_out = dtt0;
assign dtt1_out = dtt1;
assign pt_fc    = dfc;
assign pf_fc    = dfc;

// interrupt input synchronization (active low pins, must be stable for two
// consecutive samples like the real part)
reg [2:0] ipl_s1, ipl_s2;
reg [2:0] irq_lvl;
reg [2:0] irq_hold_lvl;          // mask-qualified level retained until accepted
reg       irq_ack_t;             // toggled by the FSM when a level 1-6 IRQ is taken
reg       irq_ack_d;             // sampler-side shadow of irq_ack_t
reg       nmi_arm;
reg       nmi_ack_t;              // toggled by the FSM when an NMI is taken
reg       nmi_ack_d;              // sampler-side shadow of nmi_ack_t
assign nmi_ack_toggle = nmi_ack_t;
always @(posedge clk) begin
	if (!nreset) begin
		ipl_s1 <= 3'b111;
		ipl_s2 <= 3'b111;
		irq_lvl <= 3'd0;
		irq_hold_lvl <= 3'd0;
		irq_ack_d <= 1'b0;
		nmi_arm <= 1'b0;
		nmi_ack_d <= 1'b0;
	end
	else begin
		ipl_s1 <= ipl;
		ipl_s2 <= ipl_s1;
		if (ipl_s1 == ipl_s2) begin
			irq_lvl <= ~ipl_s2;
			// Re-arm the edge-triggered level 7 as soon as the pins leave
			// it.  This tracks the pins rather than instruction execution,
			// so an NMI cannot be missed because the CPU happened to be
			// stalled waiting on memory (nmi_ack clears it on acceptance).
			if (~ipl_s2 != 3'd7) nmi_arm <= 1;
		end
		// Once a mask-qualified level 1-6 request has reached IPEND, an
		// instruction which subsequently raises the SR mask must not make it
		// disappear.  Retain the highest sampled level until exception entry
		// acknowledges it.  This is separate from level 7's edge latch.
		irq_ack_d <= irq_ack_t;
		if (irq_ack_t != irq_ack_d)
			irq_hold_lvl <= 3'd0;
		// The hold exists to survive a MASK change, not the source going
		// away.  A 68040 requires the requesting device to keep IPL
		// asserted until the CPU acknowledges; if it drops the request
		// first, the interrupt is simply not taken.  Track the pins down
		// so a withdrawn request cannot fire later as a phantom
		// interrupt at the next instruction boundary.  The level the
		// encoder falls back to is a DIFFERENT device's request that was
		// hidden behind the withdrawn one: it must qualify against the
		// mask on its own, or the hold would fire it at or below the
		// mask (levels 1-6 are taken only strictly above SR[10:8]).
		else if (irq_hold_lvl > irq_lvl)
			irq_hold_lvl <= (irq_lvl > sr[10:8]) ? irq_lvl : 3'd0;
		else if (irq_lvl != 3'd0 && irq_lvl != 3'd7 &&
		         irq_lvl > sr[10:8] && irq_lvl > irq_hold_lvl)
			irq_hold_lvl <= irq_lvl;

		// acceptance wins over re-arming in the same cycle
		nmi_ack_d <= nmi_ack_t;
		if (nmi_ack_t != nmi_ack_d) nmi_arm <= 0;
	end
end

// Recognise the level one cycle earlier than the registered irq_lvl.
// Both samples are synchroniser flops, so the coherence check that guards
// against reading a transient encoder value (3 -> 5 passing through 7
// would look like an NMI) still holds; only the extra register stage is
// removed.  cputest's irq tests time the request to land just before an
// instruction boundary, and the third cycle pushed recognition past it --
// hardware stacked the address AFTER the tested instruction.
wire [2:0] irq_lvl_live = (ipl_s1 == ipl_s2) ? ~ipl_s2 : irq_lvl;

wire       nmi_pend = irq_lvl_live == 3'd7 && nmi_arm;
wire       irq_live = irq_lvl_live != 3'd0 && irq_lvl_live != 3'd7 &&
                      irq_lvl_live > sr[10:8];
wire [2:0] irq_take_lvl = nmi_pend ? 3'd7 :
                          (irq_live && irq_lvl_live > irq_hold_lvl)
                              ? irq_lvl_live : irq_hold_lvl;
wire       irq_pend = nmi_pend || irq_live || irq_hold_lvl != 3'd0;


wire unused_in = ipl_autovector;

// An access error comes either from the MMU (translation fault) or from the
// bus (a physical bus error: no device answered).  Both are only sampled
// while a transfer is outstanding, which is the only time they are tested.
wire mem_err = mem_req && (mem_flt | berr);   // data channel only: the wrapper gates each channel's faults (P171)

// X2.2b stage 1: the memory port carries two channels, and mem_instr_q tags
// which one owns the transaction in flight -- aerr_start already builds the
// fault frame from it.  What was NOT explicit is the acknowledge: a data
// state reaches its ack branch only when m_issued, and m_issued can only be
// set while !epf_pend, so today an ack is attributed by construction rather
// than by inspection.  That construction IS the serialization stage 2
// removes, at which point an unqualified mem_ack would be delivered to
// whichever channel happened to be looking.  Qualify both channels now,
// while the stall still guarantees the answer, so the change that matters
// later is not also the change that introduces the qualification.
//
// Behaviour is identical today: epf_pend is set only by instruction issues
// (issue_ifetch's port-free branch and the fill engine, both mem_instr_q=1),
// exception_prefetch owns the port outright on the exception path, and the
// data states set mem_instr_q=0 at their own issue.
wire d_ack = mem_ack;                   // data channel acknowledge
wire i_ack = ifr_ack;                   // instruction channel acknowledge (P171)
wire d_err = mem_err;
wire i_err = ifr_req && (ifr_flt | ifr_berr);

//---------------------------------------------------------------------------
// register file
//---------------------------------------------------------------------------

reg         rf_we;
reg   [3:0] rf_waddr;
reg  [31:0] rf_wdata;
reg   [3:0] rr_a;
reg   [3:0] rr_b;
wire [31:0] rf_rdata_a;
wire [31:0] rf_rdata_b;
// Adjacent retirement dispatches before the previous registered write
// commits. Forward both capture ports, including merge_sz's upper bits.
wire [31:0] rf_capture_a = (rf_we && rf_waddr == rr_a) ? rf_wdata : rf_rdata_a;
wire [31:0] rf_capture_b = (rf_we && rf_waddr == rr_b) ? rf_wdata : rf_rdata_b;
reg         aux_we;
reg   [1:0] aux_sel;
reg  [31:0] aux_wdata;
wire [31:0] usp_q, isp_q, msp_q;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_a0, dbg_a7;

// rfw/aux writes cross into the register-file block on the following clock
// edge.  Most decoded instructions select an asynchronous read port and do
// not consume it until a later state, but BSR.B and MOVE USP,An use these
// direct views in decode.  When fetch_next dispatches a resident opcode
// without the old S_FETCH bubble, forward the write which is committing on
// this edge so those two consumers see architectural retirement order.
wire [31:0] dbg_a7_wb = (rf_we && rf_waddr == 4'd15) ? rf_wdata : dbg_a7;
wire [31:0] usp_wb = (aux_we && aux_sel == 2'd0) ? aux_wdata : usp_q;

// Experimental ownership/issue checkpoint, absent from release builds.
// The pipeline consumes resident integer/memory instructions and brief PEA. It drains before
// returning unsupported words to the sequencer; IRQ/trace cancels younger
// work at a retirement boundary, preserving the committing instruction.
// Architectural state stays in this core; no bulk import/export or shadow RF.
`ifdef AP040_EXPERIMENTAL_PIPELINE
localparam S_EXPERIMENT_PIPE = 8'd250;
localparam S_PIPE_LOAD_RETURN = 8'd249;
`ifdef AP040_EXPERIMENTAL_PIPELINE_LOADS
localparam PIPE_LOADS = 1;
`else
localparam PIPE_LOADS = 0;
`endif
`ifdef AP040_EXPERIMENTAL_PIPELINE_STORES
localparam PIPE_STORES = 1;
`else
localparam PIPE_STORES = 0;
`endif
`ifdef AP040_EXPERIMENTAL_PIPELINE_PEA
localparam PIPE_PEA = 1;
`else
localparam PIPE_PEA = 0;
`endif
// Broader integer stream: register shifts, d16 LEA and brief indexed MOVE.
`ifdef AP040_EXPERIMENTAL_PIPELINE_P6
localparam PIPE_P6 = 1;
`else
localparam PIPE_P6 = 0;
`endif
`ifdef AP040_PIPELINE_MEMORY_ENTRY
localparam PIPE_MEMORY_ENTRY = 1;
`else
localparam PIPE_MEMORY_ENTRY = 0;
`endif
`ifdef AP040_PIPELINE_COMPARE
localparam PIPE_COMPARE = 1;
`else
localparam PIPE_COMPARE = 0;
`endif
wire pipe_load_req, pipe_load_write;
wire [31:0] pipe_load_wdata;
wire [4:0] pipe_load_ccr;
wire [31:0] pipe_load_addr, pipe_load_pc;
wire [15:0] pipe_load_opcode;
wire [1:0] pipe_load_size;
reg pipe_load_active;
wire pipe_load_return = state == S_PIPE_LOAD_RETURN;
wire pipe_load_abort = pipe_load_active &&
    ((state == S_MRD) || (state == S_MRD_B) ||
     (state == S_MWR) || (state == S_MWR_B)) && d_err;
// Ordinary reads forward the acknowledgement; split reads use the buffered
// return state. Faults use aerr_start and discard younger pipeline records.
wire pipe_load_direct = pipe_load_active && (state == S_MRD || state == S_MWR) && m_issued && d_ack && !d_err;
wire pipe_read_retire = pipe_load_direct && state == S_MRD &&
    !irq_pend && !sr[15] && !sr[14];
wire pipe_load_ack = pipe_load_return || pipe_load_direct || (ce && pipe_load_abort);
// Select the response payload from registered state, independently of ack.
// Only S_PIPE_LOAD_RETURN consumes the buffered split-transfer value;
// ordinary responses consume mem_rdata under the unchanged ack/fault gates.
wire [31:0] pipe_load_value = pipe_load_return ? m_val : mem_rdata;

wire [31:0] pipe_rdata_a, pipe_rdata_b, pipe_old_dst;
wire [3:0] pipe_old_dst_reg;
wire pipe_supported, pipe_next_supported, pipe_ready, pipe_retire, pipe_we, pipe_idle;
wire pipe_wb_retire, pipe_wb_branch_taken;
wire [3:0] pipe_src, pipe_dst, pipe_wdst;
wire [31:0] pipe_data, pipe_pc, pipe_next_pc;
wire [1:0] pipe_words;
wire [15:0] pipe_opcode;
wire [4:0] pipe_ccr;
wire [5:0] pipe_alu_op, pipe_alu_shcnt;
wire [1:0] pipe_alu_size;
wire [31:0] pipe_alu_src, pipe_alu_b;
wire [4:0] pipe_alu_flags_in;
// A short unsupported boundary should retain the sequencer's fast path.
// PEA already wins in isolation; other entries need a supported successor.
// Forced-decode tests deliberately bypass this policy for opcode coverage.
`ifdef AP040_PIPELINE_FORCE_DECODE
wire pipe_entry_ok = 1'b1;
`elsif AP040_PIPELINE_PEA_ENTRY_ONLY
wire pipe_entry_ok = pipe_words == 2 && (ir & 16'hfff8) == 16'h4870;
`elsif AP040_PIPELINE_SELECTIVE
wire pipe_entry_ok = pipe_words == 2 || pipe_next_supported;
`elsif AP040_PIPELINE_MEMORY_ENTRY
wire pipe_entry_ok = (pipe_words == 2 && (ir & 16'hf1f8) != 16'h41e8) ||
    (PIPE_COMPARE && (ir & 16'hff38) == 16'h4a10 && ir[7:6] != 3);
`else
wire pipe_entry_ok = 1'b1;
`endif
// A late same-page PEA/indexed-memory extension can join after background fetch.
// Do not wait across a page or after a speculative fetch fault.
wire pipe_wait_indexed = PIPE_MEMORY_ENTRY &&
    ((PIPE_P6 && ir[15:14] == 0 && ir[13:12] != 0 &&
      ((ir[5:3] == 6 && (ir[8:6] == 0 || ir[8:6] == 1) &&
        !(ir[13:12] == 1 && ir[8:6] == 1)) ||
       (ir[8:6] == 6 && ir[5:4] == 0 && !(ir[13:12] == 1 && ir[3])))) ||
     (PIPE_COMPARE && (((ir & 16'hf138) == 16'hb030 ||
                       (ir & 16'hff38) == 16'h4a30) && ir[7:6] != 3)));
wire pipe_pea_wait = (state == S_DECODE) &&
    ((PIPE_PEA && (ir & 16'hfff8) == 16'h4870) || pipe_wait_indexed) &&
    !epf_ready_pc && epf_armed &&
    epf_next == pc && pc_i[11:0] != 12'hffe && !epf_err;
wire pipe_claim = (state == S_DECODE) && pipe_supported && pipe_entry_ok && !(ir[15:12] == 6 && (sr[15] || sr[14]));
wire pipe_owner = state == S_EXPERIMENT_PIPE;
wire [2:0] pipe_ext_head = epf_head + (pipe_rf_owner ? 3'd1 : 3'd0);
// Pipeline fetch ownership spans the memory sequencer and return edge.
// Independent RF ports also preserve partial-Dn operands during CE pauses.
wire pipe_rf_owner = pipe_owner || pipe_load_active || pipe_load_return;
wire pipe_empty_after_retire;
`ifdef AP040_PIPELINE_EARLY_DRAIN
wire pipe_exit_ready = pipe_empty_after_retire;
`else
wire pipe_exit_ready = pipe_idle;
`endif
wire pipe_drain = (pipe_owner || pipe_read_retire) && pipe_exit_ready;
// Memory completion still retires through S_EXPERIMENT_PIPE, but ID can fill
// while it waits. Do not consume a queued word being invalidated by a store.
// ID does not read operands on admission. With independent RF ports it can
// overlap the preceding registered write; EX sees the committed/pending value.
wire pipe_input = (pipe_claim && (!rf_we || PIPE_MEMORY_ENTRY) && !aux_we) ||
    (pipe_rf_owner && epf_ready_pc && pipe_supported &&
     !(pipe_load_active && d_ack && mem_write &&
       (mem_addr_q + 32'd3 >= epf_next) && (mem_addr_q < epf_ftail)) &&
     !irq_pend && !sr[15] && !sr[14]);
wire pipe_branch_taken = pipe_owner && pipe_wb_branch_taken;
wire pipe_cancel = pipe_owner && pipe_wb_retire &&
    (irq_pend || tr_t1 || (tr_t0 && t0_force));
wire pipe_write = (pipe_owner || pipe_read_retire) && pipe_retire && pipe_we;
wire pipe_load_launch = pipe_owner && pipe_load_req && !pipe_load_active;
ap040_pipeline_integer #(
    .EXTERNAL_STATE(1), .EXTERNAL_ALU(1), .ENABLE_LOADS(PIPE_LOADS), .ENABLE_STORES(PIPE_STORES),
    .ENABLE_PEA(PIPE_PEA), .ENABLE_INDEXLOAD(PIPE_P6), .ENABLE_SHIFTS(PIPE_P6),
    .ENABLE_DISP_LEA(PIPE_P6), .ENABLE_COMPARE(PIPE_COMPARE), .ENABLE_BRANCH(1), .ENABLE_FAST_READ_RETIRE(1)
) integer_pipeline (
    .clk(clk), .nreset(nreset), .ce(ce), .flush(pipe_load_abort),
    .kill_younger(pipe_cancel || pipe_branch_taken), .idle(pipe_idle), .empty_after_retire(pipe_empty_after_retire),
    .external_dst(pipe_old_dst), .read_old_dst(pipe_old_dst_reg), .external_a(pipe_rdata_a), .external_b(pipe_rdata_b), .external_sp(dbg_a7_wb), .external_ccr(sr[4:0]),
    .external_alu_selected(pipe_rf_owner), .external_alu_result(alu_res), .external_alu_flags(alu_fl),
    .external_alu_fast_flags(alu_fast_fl), .external_alu_fast_ok(alu_fast_ok),
    .ex_op(pipe_alu_op), .ex_size(pipe_alu_size), .shcnt(pipe_alu_shcnt), .src(pipe_alu_src), .b(pipe_alu_b), .flags_in(pipe_alu_flags_in),
    .read_src(pipe_src), .read_dst(pipe_dst), .in_supported(pipe_supported),
    .next_opcode(epf_data[epf_head]), .next_extension(epf_data[(epf_head + 3'd1) & 3'd7]),
    .next_valid(epf_ready_pc), .next_extension_valid(epf_ready_pc2), .next_supported(pipe_next_supported),
    .in_valid(pipe_input), .in_ready(pipe_ready),
    .in_pc(pipe_rf_owner ? pc : pc_i),
    .in_opcode(pipe_rf_owner ? epf_data[epf_head] : ir),
    .in_extension(epf_data[pipe_ext_head]),
    .in_extension_valid(pipe_rf_owner ? epf_ready_pc2 : epf_ready_pc), .in_words(pipe_words),
    .retire_ready(pipe_owner || pipe_read_retire), .retire_valid(pipe_retire), .retire_wb_valid(pipe_wb_retire), .retire_branch_taken(pipe_wb_branch_taken), .retire_we(pipe_we),
    .retire_dst(pipe_wdst), .retire_data(pipe_data), .retire_ccr(pipe_ccr),
    .retire_pc(pipe_pc), .retire_next_pc(pipe_next_pc), .retire_opcode(pipe_opcode),
    .load_req(pipe_load_req), .load_write(pipe_load_write),
    .load_wdata(pipe_load_wdata), .load_ccr(pipe_load_ccr), .load_addr(pipe_load_addr), .load_size(pipe_load_size),
    .load_pc(pipe_load_pc), .load_opcode(pipe_load_opcode),
    .load_ack(pipe_load_ack), .load_fault(1'b0), .load_data(pipe_load_value),
    .retire_fault(), .retire_fault_addr(),
    .fallback_valid(), .fallback_pc(), .fallback_opcode()
);
`else
wire pipe_claim = 1'b0;
wire pipe_drain = 1'b0;
wire pipe_load_launch = 1'b0;
`endif

`ifdef AP040_EXPERIMENTAL_PIPELINE
always @(posedge clk) begin
    if (!nreset) pipe_load_active <= 0;
    else if (ce) begin
        if (pipe_owner && pipe_load_req && !pipe_load_active)
            pipe_load_active <= 1;
        if (pipe_load_return || pipe_load_abort || pipe_load_direct) pipe_load_active <= 0;
    end
end
`endif

wire [31:0] rsr_base_rf;   // P196: the queue head's EA register (regfile port F)
ap040_regfile
`ifdef AP040_EXPERIMENTAL_PIPELINE
#(.EXTRA_READS(1))
`endif
regfile
(
	.clk(clk), .ce(ce), .nreset(nreset),
	.sr_s(sr_s), .sr_m(sr_m),
`ifdef AP040_EXPERIMENTAL_PIPELINE
    .we(pipe_write || rf_we),
    .waddr(pipe_write ? pipe_wdst : rf_waddr),
    .wdata(pipe_write ? pipe_data : rf_wdata),
    .raddr_a(rr_a), .rdata_a(rf_rdata_a),
    .raddr_b(rr_b), .rdata_b(rf_rdata_b),
    .raddr_c(pipe_src), .rdata_c(pipe_rdata_a),
    .raddr_d(pipe_dst), .rdata_d(pipe_rdata_b),
    .raddr_e(pipe_old_dst_reg), .rdata_e(pipe_old_dst),
    .raddr_f({1'b1, rd_ir[2:0]}), .rdata_f(rsr_base_rf),
`else
	.we(rf_we), .waddr(rf_waddr), .wdata(rf_wdata),
    .raddr_c(4'd0), .rdata_c(), .raddr_d(4'd0), .rdata_d(),
    .raddr_e(4'd0), .rdata_e(),
    .raddr_f({1'b1, rd_ir[2:0]}), .rdata_f(rsr_base_rf),
	.raddr_a(rr_a), .rdata_a(rf_rdata_a),
	.raddr_b(rr_b), .rdata_b(rf_rdata_b),
`endif
	.aux_we(aux_we), .aux_sel(aux_sel), .aux_wdata(aux_wdata),
	.usp_q(usp_q), .isp_q(isp_q), .msp_q(msp_q),
	.dbg_d0(dbg_d0), .dbg_d1(dbg_d1), .dbg_d2(dbg_d2),
	.dbg_a0(dbg_a0), .dbg_a7(dbg_a7)
);

//---------------------------------------------------------------------------
// ALU and multiply/divide
//---------------------------------------------------------------------------

reg   [5:0] alu_op;
reg   [1:0] op_size;
reg  [31:0] src_val, dst_val;
reg  [31:0] sh_val;
reg   [4:0] sh_fl;
reg   [7:0] state;
wire [31:0] alu_res;
wire  [4:0] alu_fl;

localparam S_SHIFT      = 8'd29;   // forward declaration for the alu_b mux
// hoisted like the localparam: the ALU instance below consumes it
reg  [5:0] sh_cnt;

wire alu_is_bitop = (alu_op >= `AP040_ALU_BTST) && (alu_op <= `AP040_ALU_BSET);
reg         p_sextw;
reg         p_dst_mem_bit;    // bit op destination is memory (modulo 8)

// ALU operations targeting a register retire at operand capture. Register
// sources have settled on port A. Memory retirement remains guarded by the
// existing d_err-before-d_ack completion path; no early acknowledge is used.
// Split reads use their assembled m_val only after all bytes have completed.
wire        regs_alu_fire = (state == S_PIPE_REGS || state == S_PIPE_SDONE ||
                            state == S_MRD) &&
                            (exec_kind == EK_ALU) && (p_dst == DK_REG);
// Only the successful-read arm consumes this value. Keep acknowledgement
// out of the data select; it qualifies the ordered read-to-write handoff below.
wire move_store_read_context = state == S_MRD && r_m_ret == S_PIPE_SDONE &&
    p_src == SK_MEM && p_dst == DK_MEM && exec_kind == EK_ALU &&
    alu_op == `AP040_ALU_MOVE && !p_rmw && !p_wbsup;
// P179: a register write landing this clock blocks only when it is the
// destination base's own (port A shows the old value); a source (An)+/-(An)
// update of another register no longer sends the store through S_EA_DISP.
wire dst_base_landing = rf_we && rf_waddr == {1'b1,dst_rn_r};
// P179: d16(An) too, with its displacement at the queue head (popped in
// the acknowledge clock, as the ea_start fallback's inline path already did).
wire move_store_read_d16 = dst_mode_r == 3'b101 && epf_ready_pc && !epf_flushed;
wire move_store_read_ready = move_store_read_context &&
    (dst_mode_r == 3'b010 || dst_mode_r == 3'b011 || dst_mode_r == 3'b100 ||
     move_store_read_d16) &&
    rr_a == {1'b1,dst_rn_r} && !dst_base_landing && !aux_we;
wire [31:0] move_store_read_addr = dst_mode_r == 3'b100 ?
    rf_rdata_a - an_adj(dst_rn_r,p_dsize) :
    dst_mode_r == 3'b101 ? rf_rdata_a + sxw(epf_data[epf_head]) : rf_rdata_a;
wire move_store_read_handoff = move_store_read_ready && m_issued && d_ack && !d_err;
// Settled register MOVE store; require the following instruction prefetched.
wire reg_move_store_prepare = state == S_PIPE_START && exec_kind == EK_ALU &&
    alu_op == `AP040_ALU_MOVE && p_src == SK_REG && p_dst == DK_MEM &&
    !p_rmw && !p_wbsup && !p_dst_mem_bit && rr_b == p_sreg && epf_ready_pc2;
wire [31:0] alu_src = reg_move_store_prepare ? rf_capture_b :
                     move_store_read_context ? mem_rdata :
                     (regs_alu_fire && state == S_MRD) ? mem_rdata :
                     (regs_alu_fire && state == S_PIPE_SDONE) ? m_val :
                     (regs_alu_fire && p_src == SK_REG) ? rf_capture_a : src_val;
// A register shift with a nonzero count retires in S_PIPE_REGS as well:
// the ALU composes the whole count in one call (as S_SHIFT does), so the
// execute, shift and writeback states are skipped.
wire        shift_fire = (state == S_PIPE_REGS) && (exec_kind == EK_SHIFT) &&
                         (p_dst == DK_REG);
wire  [5:0] shift_cnt  = (p_src == SK_REG) ? rf_capture_a[5:0] : src_val[5:0];
// ADDQ/SUBQ to memory can compute once the ordinary read succeeds.
// This select depends on registered operand/state fields, not acknowledge.
wire quick_rmw_direct = state == S_MRD && r_m_ret == S_PIPE_DDONE &&
    exec_kind == EK_ALU && p_dst == DK_MEM && p_src == SK_IMPL &&
    p_rmw && !p_wbsup && ir[15:12] == 4'h5;
wire [31:0] alu_dst = quick_rmw_direct ? mem_rdata :
                     (regs_alu_fire || shift_fire) ? rf_capture_b : dst_val;
wire [31:0] alu_a = alu_is_bitop ? (p_dst_mem_bit ? {29'd0, alu_src[2:0]}
                                                  : {27'd0, alu_src[4:0]}) :
                    p_sextw      ? {{16{alu_src[15]}}, alu_src[15:0]} : alu_src;
wire [31:0] alu_b = (state == S_SHIFT) ? sh_val : alu_dst;
wire  [4:0] alu_fin = (state == S_SHIFT) ? sh_fl : sr[4:0];

`ifdef AP040_EXPERIMENTAL_PIPELINE
wire [5:0] core_alu_op = pipe_rf_owner ? pipe_alu_op : alu_op;
wire [1:0] core_alu_size = pipe_rf_owner ? pipe_alu_size : op_size;
wire [5:0] core_alu_shcnt = pipe_rf_owner ? pipe_alu_shcnt : ((state == S_SHIFT) ? sh_cnt : (shift_fire ? shift_cnt : 6'd1));
wire [31:0] core_alu_a = pipe_rf_owner ? pipe_alu_src : alu_a;
wire [31:0] core_alu_b = pipe_rf_owner ? pipe_alu_b : alu_b;
wire [4:0] core_alu_flags_in = pipe_rf_owner ? pipe_alu_flags_in : alu_fin;
`else
wire [5:0] core_alu_op = alu_op;
wire [1:0] core_alu_size = op_size;
wire [5:0] core_alu_shcnt = (state == S_SHIFT) ? sh_cnt : (shift_fire ? shift_cnt : 6'd1);
wire [31:0] core_alu_a = alu_a;
wire [31:0] core_alu_b = alu_b;
wire [4:0] core_alu_flags_in = alu_fin;
`endif
ap040_alu alu
(
	.op(core_alu_op), .size(core_alu_size),
	.a(core_alu_a), .b(core_alu_b),
	.flags_in(core_alu_flags_in),
	.shcnt(core_alu_shcnt),
	.result(alu_res), .flags_out(alu_fl),
	.fast_flags(alu_fast_fl), .fast_ok(alu_fast_ok)
);

reg         md_start, md_isdiv, md_sign;
reg  [31:0] md_a, md_hi, md_lo;
wire        md_done, md_ovf;
wire [31:0] md_rhi, md_rlo;

ap040_muldiv muldiv
(
	.clk(clk), .nreset(nreset), .ce(ce),
	.start(md_start), .is_div(md_isdiv), .sign_op(md_sign),
	.op_a(md_a), .op_hi(md_hi), .op_lo(md_lo),
	.done(md_done), .res_hi(md_rhi), .res_lo(md_rlo), .ovf(md_ovf)
);

//---------------------------------------------------------------------------
// states
//---------------------------------------------------------------------------

localparam S_START      = 8'd0;
localparam S_BOOT0      = 8'd1;
localparam S_BOOT1      = 8'd2;
localparam S_FETCH      = 8'd3;
localparam S_DECODE      = 8'd4;
localparam S_NEXT      = 8'd5;
localparam S_HALT      = 8'd6;
localparam S_STOPPED      = 8'd7;
localparam S_IMMF      = 8'd8;
localparam S_MRD      = 8'd9;
localparam S_MWR      = 8'd10;
localparam S_EA_DISP      = 8'd11;
localparam S_EA_BASE      = 8'd12;
localparam S_EA_D16      = 8'd13;
localparam S_EA_EXTW      = 8'd14;
localparam S_EA_EXTW2      = 8'd15;
localparam S_EA_BD      = 8'd16;
localparam S_EA_MIND      = 8'd17;
localparam S_EA_OD      = 8'd18;
localparam S_EA_ABS      = 8'd19;
localparam S_PIPE_START      = 8'd20;
localparam S_PIPE_SREG      = 8'd21;
localparam S_PIPE_SRD      = 8'd22;
localparam S_PIPE_SDONE      = 8'd23;
localparam S_PIPE_DST      = 8'd24;
localparam S_PIPE_DREG      = 8'd25;
// X2.3 step 1: both regfile read ports are independent and combinational,
// so a register source and a register destination can be fetched in ONE
// cycle instead of walking SREG then DST then DREG.
localparam S_PIPE_REGS      = 8'd190;
localparam S_PIPE_DEA      = 8'd26;
localparam S_PIPE_DDONE      = 8'd27;
localparam S_EXEC      = 8'd28;
// S_SHIFT = 29 declared above
localparam S_MD_WAIT      = 8'd30;
localparam S_MD_WB2      = 8'd31;
localparam S_MDL_RDR      = 8'd32;
localparam S_MDL_GO      = 8'd33;
localparam S_EXC0      = 8'd34;
localparam S_EXC1      = 8'd35;
localparam S_EXC2      = 8'd36;
localparam S_EXC3      = 8'd37;
localparam S_EXC4      = 8'd38;
localparam S_EXC5      = 8'd39;
localparam S_EXC6      = 8'd40;
localparam S_EXC_VEC      = 8'd41;
localparam S_EXC_JMP      = 8'd42;
localparam S_RTE_SR      = 8'd43;
localparam S_RTE_PC      = 8'd44;
localparam S_RTE_FMT      = 8'd45;
localparam S_RTE_FIN      = 8'd46;
localparam S_RTE_FIN2      = 8'd47;
localparam S_RET1      = 8'd48;
localparam S_RET2      = 8'd49;
localparam S_RET3      = 8'd50;
localparam S_BCC_EXT      = 8'd51;
localparam S_BSR_PUSH      = 8'd52;
localparam S_DBCC1      = 8'd53;
localparam S_DBCC2      = 8'd54;
localparam S_JMP1      = 8'd55;
localparam S_JSR1      = 8'd56;
localparam S_JSR2      = 8'd57;
localparam S_LEA1      = 8'd58;
localparam S_PEA1      = 8'd59;
localparam S_PEA2      = 8'd60;
localparam S_LINK1      = 8'd61;
localparam S_LINK2      = 8'd62;
localparam S_LINK3      = 8'd63;
localparam S_LINK4      = 8'd64;
localparam S_UNLK1      = 8'd65;
localparam S_UNLK2      = 8'd66;
localparam S_UNLK3      = 8'd67;
localparam S_MOVEM_SET      = 8'd68;
localparam S_MOVEM_SET2      = 8'd69;
localparam S_MOVEM_LOOP      = 8'd70;
localparam S_MOVEM_RD      = 8'd71;
localparam S_MOVEM_WR      = 8'd72;
localparam S_MOVEM_LD      = 8'd73;
localparam S_MOVEP1      = 8'd74;
localparam S_MOVEP2      = 8'd75;
localparam S_MOVEP_WR      = 8'd76;
localparam S_MOVEP_RD      = 8'd77;
localparam S_EXG1      = 8'd78;
localparam S_EXG2      = 8'd79;
localparam S_USP1      = 8'd80;
localparam S_MOVEC1      = 8'd81;
localparam S_MOVEC2      = 8'd82;
localparam S_MOVES1      = 8'd83;
localparam S_MOVES2      = 8'd84;
localparam S_MOVES_WR      = 8'd85;
localparam S_MOVES_RD      = 8'd86;
localparam S_PTEST1      = 8'd87;
localparam S_RESET_HOLD      = 8'd88;
localparam S_M16_SRC      = 8'd89;
localparam S_M16_DST      = 8'd90;
localparam S_M16_DST2      = 8'd91;
localparam S_M16_RD      = 8'd92;
localparam S_M16_RD2      = 8'd93;
localparam S_M16_WR      = 8'd94;
localparam S_M16_WR2      = 8'd95;
localparam S_M16_INC      = 8'd96;
localparam S_M16_INC2      = 8'd97;
localparam S_SROP      = 8'd98;
localparam S_SHIFT_WB      = 8'd99;
localparam S_TRAPCC      = 8'd100;
localparam S_STOP_LD      = 8'd101;
localparam S_MOVEM_EA      = 8'd102;
localparam S_MDL_RDQ      = 8'd103;
localparam S_MDL_EXT      = 8'd104;
localparam S_PFLUSH1      = 8'd105;
localparam S_PFLUSH2      = 8'd106;
localparam S_PTEST2      = 8'd107;
localparam S_AERR0      = 8'd108;
localparam S_AERR_U      = 8'd109;
localparam S_AERR_SP      = 8'd110;
localparam S_AERR_WR      = 8'd111;
localparam S_BF0      = 8'd112;
localparam S_BF1      = 8'd113;
localparam S_BF_REG      = 8'd114;
localparam S_BF_REG2      = 8'd115;
localparam S_BF_REGX      = 8'd116;
localparam S_BF_MEM0      = 8'd117;
localparam S_BF_MEM1      = 8'd118;
localparam S_BF_MEM2      = 8'd119;
localparam S_BF_EXECM      = 8'd120;
localparam S_BF_WR1      = 8'd121;
localparam S_BF_WR2      = 8'd122;
localparam S_CAS1      = 8'd123;
localparam S_CAS2      = 8'd124;
localparam S_CAS3      = 8'd125;
localparam S_CAS4      = 8'd126;
localparam S_BF_X2     = 8'd127;
localparam S_BF_X3     = 8'd128;
localparam S_BF_X4     = 8'd129;
localparam S_BF_M2     = 8'd130;
localparam S_BF_M3     = 8'd131;
localparam S_BF_M4     = 8'd132;
localparam S_CINV2     = 8'd133;
localparam S_CHK2_A    = 8'd134;
localparam S_CHK2_B    = 8'd135;
localparam S_CHK2_C    = 8'd136;
localparam S_CHK2_D    = 8'd137;
localparam S_BTSTI     = 8'd138;
localparam S_BTSTI2    = 8'd139;
localparam S_CAS2_0    = 8'd140;
localparam S_CAS2_1    = 8'd141;
localparam S_CAS2_2    = 8'd142;
localparam S_CAS2_3    = 8'd143;
localparam S_CAS2_4    = 8'd144;
localparam S_CAS2_5    = 8'd145;
localparam S_CAS2_6    = 8'd146;
localparam S_CAS2_W2   = 8'd147;
localparam S_CAS2_W3   = 8'd148;
localparam S_CAS2_F    = 8'd149;
localparam S_CAS2_F2   = 8'd150;
localparam S_FSAVE1    = 8'd151;
localparam S_MOVEM_FIN = 8'd197;   // commit a held MOVEM index register
localparam S_RTE_SSW   = 8'd198;   // format-7 continuation status
localparam S_RTE_EA    = 8'd199;   // CM's saved MOVEM effective address
localparam S_FREST1    = 8'd152;
localparam S_FPU_DEC   = 8'd153;
localparam S_FPU_AN    = 8'd154;
localparam S_FPU_EA    = 8'd155;
localparam S_FPU_DREG  = 8'd156;
localparam S_FPU_IMM   = 8'd157;
localparam S_FPU_RD    = 8'd158;
localparam S_FPU_RD2   = 8'd159;
localparam S_FPU_GO    = 8'd160;
localparam S_FPU_WR    = 8'd161;
localparam S_FPU_CR    = 8'd162;
localparam S_FPU_CR2   = 8'd163;
localparam S_FPU_MVM   = 8'd164;
localparam S_FPU_MVM2  = 8'd165;
localparam S_FPU_MVM3  = 8'd166;
localparam S_FBCC      = 8'd167;
localparam S_FSCC0     = 8'd168;
localparam S_FSCC1     = 8'd169;
localparam S_FDBCC     = 8'd170;
localparam S_FREST2    = 8'd171;
localparam S_FPU_MVML  = 8'd172;
localparam S_FPU_CRD   = 8'd173;
localparam S_EXC4B     = 8'd174;
localparam S_MRD_B     = 8'd175;
localparam S_MWR_B     = 8'd176;
localparam S_FPU_CRI   = 8'd177;
localparam S_EPF_FILL  = 8'd178;
localparam S_EPF_GAP   = 8'd179;
localparam S_EPF_READY = 8'd180;
localparam S_POST_EXC  = 8'd181;
localparam S_FSAVE_U   = 8'd182;
localparam S_FSAVE_UD  = 8'd183;
localparam S_FREST_U   = 8'd184;
localparam S_FREST_UD  = 8'd185;
localparam S_FSAVE_B   = 8'd186;
localparam S_FSAVE_BD  = 8'd187;
localparam S_FREST_B   = 8'd188;
localparam S_FREST_BD  = 8'd189;
// Exception format is encoded in the registered entry state.  Loading
// exc_fmt from these shallow state decodes removes the old direct path from
// the instruction decoder into the exception-frame format register.
localparam S_EXC0_F2       = 8'd191;
localparam S_EXC0_F3       = 8'd192;
localparam S_EXC0_F4       = 8'd193;
localparam S_POST_EXC_F2   = 8'd194;
localparam S_POST_EXC_F3   = 8'd195;
localparam S_POST_EXC_F4   = 8'd196;

// exec kinds
localparam EK_ALU     = 4'd0;
localparam EK_SHIFT   = 4'd1;
localparam EK_MD_W    = 4'd2;   // word multiply/divide
localparam EK_MD_L    = 4'd3;   // long multiply/divide (extension in x_ext)
localparam EK_CHK     = 4'd4;
localparam EK_SCC     = 4'd5;
localparam EK_PACK    = 4'd6;
localparam EK_UNPK    = 4'd7;

// src/dst kinds
localparam SK_NONE = 3'd0;
localparam SK_REG  = 3'd1;
localparam SK_IMM  = 3'd2;
localparam SK_MEM  = 3'd3;
localparam SK_IMPL = 3'd4;   // src_val preloaded at decode

localparam DK_NONE = 3'd0;
localparam DK_REG  = 3'd1;
localparam DK_MEM  = 3'd2;
localparam DK_SR   = 3'd3;
localparam DK_CCR  = 3'd4;

// ret_kind for RTS/RTR/RTD
localparam RK_RTS = 2'd0;
localparam RK_RTR = 2'd1;
localparam RK_RTD = 2'd2;

//---------------------------------------------------------------------------
// control registers of the execution engine
//---------------------------------------------------------------------------

reg  [7:0] r_imm_ret, r_ea_ret, r_m_ret;
reg        mrd_hinted;   // P182
reg        st_hinted;    // P196: the hint bus carried this store when it issued
// P180: PEA d16(An)/d16(PC) pushes from S_EA_D16
wire pea_d16_push = state == S_EA_D16 && r_ea_ret == S_PEA1;
reg  [2:0] m_bidx;                // byte index of a split transfer
reg [31:0] m_acc;                 // assembled bytes of a split transfer
reg  [1:0] imm_n;
reg        m_issued;
reg [31:0] imm;
reg [31:0] x_ext;              // saved copy of imm (survives EA fetches)

// Instruction fetch queue.  The 68040 does not begin handler execution until
// four longwords have been fetched; that architectural requirement built this
// eight-word FIFO, and the same storage now carries every instruction word.
// The fill engine at the end of the state machine appends to the tail
// whenever the memory port is idle, S_FETCH and S_IMMF pop the head, and any
// flow, context or FC change flushes the whole thing.
reg [15:0] epf_data [0:7];
reg  [3:0] epf_count;            // words resident
reg  [2:0] epf_head, epf_fill;   // pop / append indices
reg [31:0] epf_base;             // exception prefetch origin
reg [31:0] epf_next;             // address of the word at the head
reg [31:0] epf_ftail;            // address of the next word to be fetched
reg        epf_super;            // FC the queue was filled under
reg        epf_armed;            // the fill engine owns this stream
reg        epf_pend;             // a queue fetch is outstanding
// The fetch channel can take a redirect's target now: nothing is
// outstanding, or the outstanding speculative fetch has not been
// presented to the platform yet (the data channel holds the port) and is
// replaced in place -- it never went out, so nothing is killed and its
// acknowledge, when it comes, is the target's (P171).
wire        ifr_replaceable = epf_pend && ifr_req && !ifr_pres && !ifr_ack;
wire        ifr_avail = (!epf_pend && !ifr_req && !ifr_ack) || ifr_replaceable;
reg        i_ack_d;              // an instruction fetch acknowledged last cycle: its line offer lands now (P171)
reg        epf_pend_lw;          // ... and it returns two words
reg        epf_kill;             // ... whose data a flush has abandoned
reg        epf_err;              // the fill engine faulted: re-issue on demand
reg        epf_brf;              // queue was seeded by branch-refill buffer
reg        epf_pend_seed;         // ... the outstanding fetch is a redirect's own
reg        brf_seed_ok;           // the last acknowledged fetch was a redirect's
reg  [3:0] brf_seed_n;            // words the last redirect seeded from the buffer
reg        brf_seed_req;          // seed the queue from the buffer this cycle (one shared block)
reg  [4:0] brf_seed_a;            // ... starting at this sector word
// The cache identifies its offered line physically; the queue and the
// refill buffer are logical.  Record the logical line and context of every
// acknowledged instruction fetch: the offer that follows is that line.
reg [31:4] iline_log;
reg        iline_super;

// Small instruction branch-refill buffer.  The main queue is forward-only:
// a taken backwards branch otherwise discards its words and pays another
// cache/MMU handshake even when a tight loop was fetched only moments ago.
// One 64-byte sector retains recently completed instruction fetches and can
// seed up to four contiguous words at a redirect target.  A shared tag keeps
// both the loop target and modest forward prefetch resident without the cost
// of two independent tag comparators.  It is tagged by logical address and
// supervisor context, and every
// architectural queue flush (exception, CINV, PFLUSH, MOVEC, context change)
// invalidates them.  Ordinary control-flow redirects deliberately do not.
reg [31:0] brf_data [0:15];
reg [25:0] brf_tag;
reg        brf_super;
reg [31:0] brf_valid;
// The sector's valid-run lengths: from each word to the sector's end,
// capped at the queue's eight, from the registered valid bits alone.
// Every redirect site's refill test (four words from the target) and
// seed count (up to eight) are then one 32-way mux on the target's
// sector word, where each site used to select and count the valid bits
// in an eight-step chain from the target -- the tail of the
// flags -> carrier -> refill seed -> epf_ftail path.  (2026-09-18)
reg  [3:0] brf_run [0:31];
function [3:0] brf_prefix8;
    input [7:0] valid_words;
    begin
        casez (valid_words)
            8'b???????0: brf_prefix8 = 0;
            8'b??????01: brf_prefix8 = 1;
            8'b?????011: brf_prefix8 = 2;
            8'b????0111: brf_prefix8 = 3;
            8'b???01111: brf_prefix8 = 4;
            8'b??011111: brf_prefix8 = 5;
            8'b?0111111: brf_prefix8 = 6;
            8'b01111111: brf_prefix8 = 7;
            8'b11111111: brf_prefix8 = 8;
            default: brf_prefix8 = 0;
        endcase
    end
endfunction
integer brf_ri;
always @* begin
    for (brf_ri = 0; brf_ri < 32; brf_ri = brf_ri + 1)
        brf_run[brf_ri] = brf_prefix8(brf_valid >> brf_ri);
end

// Combinational within the state machine's always block: the port claim and
// the flush both have to be visible to the fill engine, which runs after the
// case so that a state claiming the port this cycle always wins it.
reg        epf_issue;            // the port was claimed by a state this cycle
reg        epf_flushed;          // the queue was flushed this cycle
reg  [1:0] epf_pop;              // words consumed this cycle
reg  [3:0] epf_fillw;            // words appended this cycle (up to a line)
// Blocking per-clock carrier: only fetch_next's qualified resident-opcode
// branch sets this. Descriptor controls are written once after case(state),
// rather than expanded inside every caller of the generic completion task.
reg        rd_queue_pop;
reg        retire_req;
reg        dgo;                   // an arm dispatched a resident branch target (decode_dbcc_brf): done once
reg        pgo;                   // an arm redirected the flow (go_pc): performed once after the case
reg        sgo;                   // dispatch_branch put the target fetch out at the pop (issue_ifetch once after the case)
reg        rfw_now;               // an arm called rfw this cycle ...
reg  [3:0] rfw_now_a;             // ... for this register
reg        igo;                   // an arm asked for extension words (immf): served once after the case
reg  [1:0] igo_n;
reg  [7:0] igo_ret;
reg        xgo;                   // an arm raised an exception (exc): entered once after the case
reg  [7:0] xgo_vec;
reg  [3:0] xgo_fmt;
reg [31:0] xgo_spc, xgo_addr;
reg        mgo;                   // an arm issued a data transfer (mrd/mwr): set up once after the case
reg        mgo_wr;
reg  [1:0] mgo_sz;
reg  [7:0] mgo_ret;
reg [31:0] mgo_a, mgo_d;            // an arm asked for the retire boundary (fetch_next): done once after the case

// A resident word needs no bus cycle at all, so a fetch consumes it in the
// very cycle it would otherwise have spent issuing a request.
wire       epf_ready_pc = epf_armed && (epf_count != 4'd0) &&
                          (epf_next == pc) && (epf_super == sr_s);
wire       epf_ready_pc2 = epf_armed && (epf_count > 4'd1) &&
                           (epf_next == pc) && (epf_super == sr_s);
// The word arriving this cycle bypasses the queue: a fetch that ran the
// queue dry still completes in the acknowledge cycle, exactly as the
// pre-queue demand fetch did.
wire       epf_fwd_pc = epf_pend && i_ack && !epf_kill && epf_armed &&
                        (epf_count == 4'd0) && (epf_next == ifr_addr) &&
                        (epf_next == pc) && (epf_super == sr_s);
wire [15:0] epf_fwd_word = epf_pend_lw ? mem_rdata[31:16] : mem_rdata[15:0];

reg        m_wr;
reg  [1:0] m_size;
reg [31:0] m_addr_r, m_wdat, m_val;

// Registered request address and space.  While no request is in flight the
// output pins carry a HINT instead: the address the core is most likely to
// request next.  The cache reads its index-addressed RAMs every idle cycle,
// so a hint that turns out right lets the following request complete at its
// admission edge; a wrong hint costs nothing, because the cache trusts only
// its own index/validity record and a fresh tag compare, never the hint.
// Nothing downstream may act on c_addr without c_req, and nothing in this
// module reads the pins: every internal consumer uses the registered copies.
//   S_MRD before issue     : the operand address the next cycle requests
//   S_DECODE of a short Bcc: its target, requested by finish_bcc this cycle
//   operand pipeline states: the source/destination address about to issue
//   otherwise              : the fill engine's next sequential fetch
reg [31:0] mem_addr_q;
reg        mem_instr_q;

reg  [2:0] ea_mode;
reg  [2:0] ea_rn;
reg  [1:0] ea_size;
reg        ea_pcmode;
reg [31:0] ea_pcb;
reg [15:0] extw;
reg [31:0] ea_base_v, ea_idx_v, ea_mind;
reg        ea_post, ea_odl, ea_absl;
reg [31:0] ea_addr;

reg  [2:0] p_src, p_dst;
reg  [3:0] p_sreg, p_dreg;
reg  [1:0] p_ssize, p_dsize;
reg        p_rmw, p_wbsup, p_flags;
reg  [3:0] exec_kind;
reg  [2:0] src_mode_r, src_rn_r, dst_mode_r, dst_rn_r;
reg [31:0] dst_addr;

reg        sh_vacc;
reg        sh_rox;
reg        sh_any;

reg  [7:0] exc_vec;
reg  [3:0] exc_fmt;
reg [31:0] exc_spc, exc_addr, exc_sp;
reg        exc_is_irq, exc_pass2;
reg [15:0] sr_saved;
reg        texc_pend;            // 040: T0 trace survives a non-internal
reg [31:0] texc_pc;              //   integer exception; fires at handler
reg        flow_t0_pend;         // T0 redirect waits for target predecode
reg [31:0] flow_t0_oldpc;
reg  [2:0] irq_lvl_l;

reg [15:0] rte_sr;
// RTE restores the SR and redirects in one step, so a request that the
// popped mask unblocks has to be qualified against THAT mask -- the sr
// register still holds the pre-RTE value in the cycle the decision is
// made.  Everything else is identical to the wires above.
wire       rte_irq_live = irq_lvl_live != 3'd0 && irq_lvl_live != 3'd7 &&
                          irq_lvl_live > rte_sr[10:8];
wire       rte_irq_hold = irq_hold_lvl != 3'd0 &&
                          irq_hold_lvl > rte_sr[10:8];
wire       rte_irq_pend = nmi_pend || rte_irq_live || rte_irq_hold;
wire [2:0] rte_irq_lvl  = nmi_pend ? 3'd7 :
                          (rte_irq_live && irq_lvl_live > irq_hold_lvl)
                              ? irq_lvl_live : irq_hold_lvl;
reg [31:0] rte_pc;

reg  [1:0] ret_kind;
reg [31:0] br_base, br_tgt;
reg        br_long;

reg [15:0] mm_mask;
reg        mm_dir;             // 1 = mem to reg
reg        mm_predec, mm_postinc;
reg  [1:0] mm_size;
reg [31:0] mm_addr, mm_init_an;
reg        mm_base_ea;            // the EA depends on An (see S_MOVEM_LD)
reg        mm_base_pend;          // a loaded base register awaits commit
reg [31:0] mm_base_val;           // its value, written with the last transfer
// Hold a loaded index as well as the base until the transfer list completes.
// The format-7 CM path below additionally preserves the calculated EA:
// register deferral alone cannot protect an indirect pointer overwritten by
// an earlier store. MC68040UM, SSW CM and access-error RTE (pp. 8-25/8-27).
reg        mm_idx_pend;
reg [31:0] mm_idx_val;
reg  [3:0] mm_idx_reg;            // captured from the extension word
reg        mm_idx_en;             // ... and whether the EA uses it at all
reg        mm_resume;             // RTE supplied the EA for the next MOVEM
reg [31:0] mm_start_ea;           // original EA, before any operand transfer
reg  [3:0] mm_reg;

reg  [2:0] mp_cnt, mp_idx;     // byte counts: 2 for word, 4 for long
reg        mp_dir;             // 1 = reg to mem
reg [31:0] mp_addr, mp_val;

reg [31:0] t_a, t_b;
reg  [1:0] srop_kind;          // 0 OR, 1 AND, 2 EOR
reg        srop_sr;            // to SR (else CCR)

reg        mvc_dir;            // 1 = general to control
reg        fc_ovr_v;
reg        lk_cyc;               // TAS/CAS/CAS2 locked-RMW operand cycle
reg        aer_lk, aer_m16;      // SSW LK; MOVE16 line-fault shape
reg  [1:0] aer_tt;               // SSW TT (MOVES to FC 0/3/4/7 reports TT1)
reg [31:0] aer_wd;               // faulted write data for the WB3 slot
reg  [2:0] fc_ovr;

reg  [2:0] m16_form;
reg  [2:0] m16_dst_rn;
reg [31:0] m16_src, m16_dst, m16_an;
reg  [1:0] m16_idx;
reg [31:0] m16buf [0:3];
reg        m16_rd_done;

reg  [7:0] rst_cnt;
reg        fault_r;
reg  [2:0] fc_r;

// Trace bits sampled when the instruction starts.  T0 includes actual
// control-flow changes plus the 68040's pipeline-synchronizing instruction
// list (NOP, SR/control-register changes, cache/MMU maintenance, etc.).
reg        tr_t1, tr_t0, t0_force;

// bitfield and CAS working registers. The bitfield datapath is spread
// over several states so each stage holds at most one wide variable
// shifter (timing: a single-cycle version costs ~28ns of logic).
reg [31:0] bf_off;
reg  [5:0] bf_w;
reg [31:0] bf_addr;
reg  [2:0] bf_bib;              // bit offset inside the first byte
reg  [2:0] bf_span;             // bytes touched (1..5)
reg [31:0] bf_w1;
reg  [7:0] bf_w2;
reg [31:0] bf_du;
reg [39:0] bf_t40;              // shifted window (mem) / rotated reg (reg form)
reg [31:0] bf_field;            // extracted field, right aligned
reg [31:0] bf_ones;             // width ones mask, right aligned
reg [39:0] bf_maskl;            // field mask, left aligned in the work domain
// One rotator for both register-form rotations: left by the offset into the
// work domain (S_BF_REGX), and back out, left by -offset (S_BF_X4).
wire [31:0] bf_rot = rotl32((state == S_BF_REGX) ? dst_val : bf_t40[39:8],
                            (state == S_BF_REGX) ? bf_off[4:0] : 5'd0 - bf_off[4:0]);
reg [31:0] cas_dc;

// access error (format $7) context and EA register-update rollback
reg        in_exc;              // exception stacking in progress
reg [31:0] aer_fa, aer_sp;
reg        aer_bus;          // fault came from the bus, not the ATC
reg        aer_ma;           // ATC fault occurred on second page of transfer
reg        aer_cm;
reg [31:0] aer_ea;
reg        aer_wr;
reg  [1:0] aer_sz;
reg  [2:0] aer_tm;
reg  [4:0] aer_idx;
reg        u0_v, u1_v;
reg  [3:0] u0_reg, u1_reg;
reg [31:0] u0_old, u1_old;

assign mem_fc    = fc_r;
assign nresetout = (state != S_RESET_HOLD);

//---------------------------------------------------------------------------
// decode helpers (combinational, from ir)
//---------------------------------------------------------------------------

wire [3:0] ir_hi     = ir[15:12];
wire [2:0] d_reg9    = ir[11:9];
wire [2:0] d_op8_6   = ir[8:6];
wire [2:0] d_mode    = ir[5:3];
wire [2:0] d_rn      = ir[2:0];

// MOVE sizes: 01=B 11=W 10=L
wire [1:0] move_size = (ir[13:12] == 2'b01) ? `AP040_SZ_B :
                       (ir[13:12] == 2'b11) ? `AP040_SZ_W : `AP040_SZ_L;
wire [1:0] std_size  = ir[7:6];   // 00=B 01=W 10=L

wire ea_is_imm     = (d_mode == 3'b111) && (d_rn == 3'b100);
// Destination is not data alterable: an address register, program space
// (both PC-relative encodings) or an immediate operand.
wire dst_not_alt   = (d_mode == 3'b001) ||
                     (d_mode == 3'b111 && d_rn > 3'b001);
// Source is not a data-addressing mode: An direct and the three reserved
// mode-7 register values.  PC-relative and immediate remain valid sources.
wire src_not_data  = (d_mode == 3'b001) ||
                     (d_mode == 3'b111 && d_rn > 3'b100);

function [31:0] sxw;
	input [15:0] v;
	begin sxw = {{16{v[15]}}, v}; end
endfunction

function [31:0] sxb;
	input [7:0] v;
	begin sxb = {{24{v[7]}}, v}; end
endfunction

function [31:0] merge_sz;
	input [31:0] old;
	input [31:0] v;
	input [1:0] size;
	begin
		case (size)
			`AP040_SZ_B: merge_sz = {old[31:8],  v[7:0]};
			`AP040_SZ_W: merge_sz = {old[31:16], v[15:0]};
			default:     merge_sz = v;
		endcase
	end
endfunction

function [31:0] an_adj;
	input [2:0] regn;
	input [1:0] size;
	begin
		if (size == `AP040_SZ_B && regn == 3'd7) an_adj = 32'd2;
		else an_adj = {29'd0, (size == `AP040_SZ_B) ? 3'd1 :
		                      (size == `AP040_SZ_W) ? 3'd2 : 3'd4};
	end
endfunction

function cond_true;
	input [3:0] cond;
	begin
		case (cond)
			4'h0: cond_true = 1;
			4'h1: cond_true = 0;
			4'h2: cond_true = !sr[0] && !sr[2];
			4'h3: cond_true =  sr[0] ||  sr[2];
			4'h4: cond_true = !sr[0];
			4'h5: cond_true =  sr[0];
			4'h6: cond_true = !sr[2];
			4'h7: cond_true =  sr[2];
			4'h8: cond_true = !sr[1];
			4'h9: cond_true =  sr[1];
			4'hA: cond_true = !sr[3];
			4'hB: cond_true =  sr[3];
			4'hC: cond_true =  sr[3] ==  sr[1];
			4'hD: cond_true =  sr[3] !=  sr[1];
			4'hE: cond_true = !sr[2] && (sr[3] == sr[1]);
			default: cond_true = sr[2] || (sr[3] != sr[1]);
		endcase
	end
endfunction

// cond_true on explicit flags {X,N,Z,V,C} = fl[4:0]: the lookahead branch
// judges the retiring producer's flags before they reach sr
function cond_true_fl;
	input [3:0] cond;
	input [4:0] fl;
	begin
		case (cond)
			4'h0: cond_true_fl = 1;
			4'h1: cond_true_fl = 0;
			4'h2: cond_true_fl = !fl[0] && !fl[2];
			4'h3: cond_true_fl =  fl[0] ||  fl[2];
			4'h4: cond_true_fl = !fl[0];
			4'h5: cond_true_fl =  fl[0];
			4'h6: cond_true_fl = !fl[2];
			4'h7: cond_true_fl =  fl[2];
			4'h8: cond_true_fl = !fl[1];
			4'h9: cond_true_fl =  fl[1];
			4'hA: cond_true_fl = !fl[3];
			4'hB: cond_true_fl =  fl[3];
			4'hC: cond_true_fl =  fl[3] ==  fl[1];
			4'hD: cond_true_fl =  fl[3] !=  fl[1];
			4'hE: cond_true_fl = !fl[2] && (fl[3] == fl[1]);
			default: cond_true_fl = fl[2] || (fl[3] != fl[1]);
		endcase
	end
endfunction

function [3:0] ffs16;
	input [15:0] m;
	integer k;
	begin
		ffs16 = 4'd0;
		for (k = 15; k >= 0; k = k - 1)
			if (m[k]) ffs16 = k[3:0];
	end
endfunction

function [31:0] rotl32;
	input [31:0] v;
	input [4:0] n;
	reg [63:0] rotated;
	begin
		// Repeated input makes wraparound part of one barrel selection.
		rotated = {v, v} << n;
		rotl32 = rotated[63:32];
	end
endfunction

function [31:0] rotr32;
	input [31:0] v;
	input [4:0] n;
	begin
		rotr32 = (n == 0) ? v : ((v >> n) | (v << (6'd32 - {1'b0, n})));
	end
endfunction

// fixed 32-bit leading zero count (shallow priority encoder)
function [5:0] clz32;
	input [31:0] v;
	integer k;
	begin
		clz32 = 6'd32;
		for (k = 0; k < 32; k = k + 1)
			if (v[k]) clz32 = 6'd31 - k[5:0];
	end
endfunction

// new right-aligned field value per bitfield operation; du must already be
// masked to the field width, ones is the width mask (no shifters in here)
function [31:0] bf_newf;
	input [2:0] op;      // ir[10:8]
	input [31:0] field;
	input [31:0] du;
	input [31:0] ones;
	begin
		case (op)
			3'd2: bf_newf = (~field) & ones;            // BFCHG
			3'd4: bf_newf = 32'd0;                      // BFCLR
			3'd6: bf_newf = ones;                       // BFSET
			default: bf_newf = du;                      // BFINS
		endcase
	end
endfunction

// MOVEC control register read mux
function [31:0] movec_rd;
	input [11:0] code;
	begin
		case (code)
			12'h000: movec_rd = {29'd0, sfc};
			12'h001: movec_rd = {29'd0, dfc};
			12'h002: movec_rd = cacr;
			12'h003: movec_rd = tc;
			12'h004: movec_rd = itt0;
			12'h005: movec_rd = itt1;
			12'h006: movec_rd = dtt0;
			12'h007: movec_rd = dtt1;
			12'h800: movec_rd = usp_q;
			12'h801: movec_rd = vbr;
			12'h803: movec_rd = msp_q;
			12'h804: movec_rd = isp_q;
			12'h805: movec_rd = mmusr;
			12'h806: movec_rd = urp;
			12'h807: movec_rd = srp;
			default: movec_rd = 32'd0;
		endcase
	end
endfunction

function movec_valid;
	input [11:0] code;
	begin
		case (code)
			12'h000, 12'h001, 12'h002, 12'h003, 12'h004, 12'h005, 12'h006,
			12'h007, 12'h800, 12'h801, 12'h803, 12'h804, 12'h805, 12'h806,
			12'h807: movec_valid = 1;
			default: movec_valid = 0;
		endcase
	end
endfunction

//---------------------------------------------------------------------------
// micro operation tasks (all nonblocking assignments)
//---------------------------------------------------------------------------

// s = supervisor bit of the context the fetch belongs to. Passed
// explicitly because RTE restores SR in its dispatch cycle: the first
// fetch of the restored context must use its FC, not the handler's
// (the MMU translates user and supervisor code through different
// roots, and a faulting fetch reports this FC in the SSW TM field).
//---------------------------------------------------------------------------
// FPU (milestone H): the core owns decode, EAs and memory traffic; the
// ap040_fpu engine owns registers, conversion and arithmetic
//---------------------------------------------------------------------------

reg         fpu_req;
reg   [2:0] fpu_class;
reg   [6:0] fpu_opm;
reg   [2:0] fpu_fmt;
reg   [2:0] fpu_srcr, fpu_dstr;
reg  [95:0] fpb;
reg   [1:0] fpu_crsel;
reg         fpu_crwe;
reg  [31:0] fpu_crwd;
reg         fpu_iawe;
reg         fpu_bsun;
reg   [2:0] fpu_fmsel;
reg         fpu_fmwe;
reg  [95:0] fpu_fmwd;
reg         fpu_rst;
reg         fpu_fsave_ack;
reg         fpu_frestore_idle;
reg         fpu_frestore_unimp;
reg  [15:0] fp_restore_cmd1, fp_restore_cmd3;
reg   [2:0] fp_restore_stag, fp_restore_dtag, fp_restore_flags;
reg  [95:0] fp_restore_fpt, fp_restore_et;
reg   [7:0] fp_restore_cusavepc;
reg         fp_restore_et15, fp_restore_fpt15;
reg   [1:0] fp_cnt;               // long transfers remaining
reg   [3:0] fp_nb;                // operand bytes
reg         fp_st;                // 1: store direction
reg         fp_st_epend;          // enabled store exception: deliver post-write
reg   [7:0] fp_st_evec;           // its vector
reg   [7:0] fp_list;              // FMOVEM register list (as processed)
reg   [1:0] fp_mode;              // FMOVEM mode bits
reg   [2:0] fp_creg;              // control list bits {FPCR,FPSR,FPIAR}
reg   [5:0] fp_pred;              // FScc/FDBcc/FTRAPcc predicate
reg         fp_rev;               // FMOVEM 040 quirk: reverse longword order
reg         fp_lsb;               // FMOVEM predec store: mask consumed LSB first
reg   [3:0] fp_n;                 // loop index
reg         fp_ea_pd, fp_ea_pi;   // predecrement / postincrement EA
reg         fp_ea_v;              // t_a holds a resolved operand address
reg   [6:0] fp_adj;               // total An adjustment (up to 8*12 = 96 bytes)

wire        fpu_done, fpu_unimp, fpu_unsupp, fpu_exc_req, fpu_used;
wire        fpu_accepted;
// Background (released) FPU operation tracking: the core continues
// integer execution while the FPU finishes register-destination
// arithmetic.  An enabled arithmetic exception from a released op is
// held pending and delivered pre-instruction at the next FPU dispatch.
reg         fpu_bg;
reg         fpu_pend_exc;
reg   [7:0] fpu_pend_vec;
wire        fpu_fstate_unimp;
wire  [7:0] fpu_cur_vec;
wire        fpu_frestore_e1_pend;
wire        fpu_frestore_resume;
wire  [2:0] fpu_fstate_grs;
wire        fpu_fstate_wbte15;
reg         fpu_pendcap;
reg   [2:0] fp_restore_grs;
reg         fp_restore_wbte15;
reg  [95:0] fp_restore_wbt;
reg  [31:0] fp_restore_fpiar;
reg         fp_restore_busy;
reg   [4:0] fpb_n;
wire        fpu_fstate_busy;
wire [95:0] fpu_fstate_wbt;
wire [31:0] fpu_fstate_fpiar_c;
wire        fpu_bsun_en;
wire  [7:0] fpu_exc_vec;
wire [95:0] fpu_dout;
wire  [3:0] fpu_cc;
wire [31:0] fpu_crrd;
wire [95:0] fpu_fmrd;
wire [15:0] fpu_fstate_cmd1, fpu_fstate_cmd3;
wire  [2:0] fpu_fstate_stag, fpu_fstate_dtag, fpu_fstate_flags;
wire [95:0] fpu_fstate_fpt, fpu_fstate_et;

generate if (AP040_HAS_FPU) begin : g_fpu
	ap040_fpu fpu
	(
		.clk(clk), .nreset(nreset), .ce(ce),
		.req(fpu_req), .op_class(fpu_class), .opmode(fpu_opm),
		.src_fmt(fpu_fmt), .src_r(fpu_srcr), .dst_r(fpu_dstr),
		.din(fpb), .done(fpu_done), .accepted(fpu_accepted),
		.unimp(fpu_unimp), .unsupp(fpu_unsupp),
		.exc_req(fpu_exc_req), .exc_vec(fpu_exc_vec), .dout(fpu_dout),
		.fpcc(fpu_cc),
		.cr_sel(fpu_crsel), .cr_we(fpu_crwe), .cr_wdata(fpu_crwd),
		.cr_rdata(fpu_crrd),
		.bsun_req(fpu_bsun), .bsun_enable(fpu_bsun_en),
		.ia_we(fpu_iawe), .ia_wdata(pc_i),
		.fm_sel(fpu_fmsel), .fm_we(fpu_fmwe), .fm_wdata(fpu_fmwd),
		.fm_rdata(fpu_fmrd),
		.fpu_used(fpu_used),
		.fstate_unimp(fpu_fstate_unimp),
		.fstate_cmd1(fpu_fstate_cmd1), .fstate_cmd3(fpu_fstate_cmd3),
		.fstate_stag(fpu_fstate_stag), .fstate_dtag(fpu_fstate_dtag),
		.fstate_flags(fpu_fstate_flags),
		.fstate_fpt(fpu_fstate_fpt), .fstate_et(fpu_fstate_et),
		.pend_capture(fpu_pendcap), .cur_vec(fpu_cur_vec),
		.frestore_e1_pend(fpu_frestore_e1_pend),
		.frestore_resume(fpu_frestore_resume),
		.frestore_cusavepc(fp_restore_cusavepc),
		.frestore_et15(fp_restore_et15), .frestore_fpt15(fp_restore_fpt15),
		.fstate_grs(fpu_fstate_grs), .fstate_wbte15(fpu_fstate_wbte15),
		.frestore_grs(fp_restore_grs), .frestore_wbte15(fp_restore_wbte15),
		.fstate_busy(fpu_fstate_busy), .fstate_wbt(fpu_fstate_wbt),
		.fstate_fpiar_c(fpu_fstate_fpiar_c),
		.frestore_wbt(fp_restore_wbt), .frestore_fpiar(fp_restore_fpiar),
		.frestore_busy(fp_restore_busy),
		.fsave_ack(fpu_fsave_ack), .frestore_idle(fpu_frestore_idle),
		.frestore_unimp(fpu_frestore_unimp),
		.frestore_cmd1(fp_restore_cmd1), .frestore_cmd3(fp_restore_cmd3),
		.frestore_stag(fp_restore_stag), .frestore_dtag(fp_restore_dtag),
		.frestore_flags(fp_restore_flags),
		.frestore_fpt(fp_restore_fpt), .frestore_et(fp_restore_et),
		.fp_reset(fpu_rst)
	);
end else begin : g_nofpu
	assign fpu_done = 0;
	assign fpu_accepted = 0;
	assign fpu_unimp = 0;
	assign fpu_unsupp = 0;
	assign fpu_exc_req = 0;
	assign fpu_exc_vec = 0;
	assign fpu_bsun_en = 0;
	assign fpu_dout = 0;
	assign fpu_cc = 0;
	assign fpu_crrd = 0;
	assign fpu_fmrd = 0;
	assign fpu_used = 0;
	assign fpu_fstate_unimp = 0;
	assign fpu_cur_vec = 0;
	assign fpu_frestore_e1_pend = 0;
	assign fpu_frestore_resume = 0;
	assign fpu_fstate_grs = 0;
	assign fpu_fstate_wbte15 = 0;
	assign fpu_fstate_busy = 0;
	assign fpu_fstate_wbt = 0;
	assign fpu_fstate_fpiar_c = 0;
	assign fpu_fstate_cmd1 = 0;
	assign fpu_fstate_cmd3 = 0;
	assign fpu_fstate_stag = 0;
	assign fpu_fstate_dtag = 0;
	assign fpu_fstate_flags = 0;
	assign fpu_fstate_fpt = 0;
	assign fpu_fstate_et = 0;
end endgenerate

// operand byte count per source format field
function [3:0] fp_bytes;
	input [2:0] fmt;
	begin
		case (fmt)
			3'd0, 3'd1: fp_bytes = 4;         // L, S
			3'd4:       fp_bytes = 2;         // W
			3'd6:       fp_bytes = 1;         // B
			3'd5:       fp_bytes = 8;         // D
			default:    fp_bytes = 12;        // X, P
		endcase
	end
endfunction

// Both revisions use a 100-byte BUSY frame (header + 96-byte payload).
// Offsets per Previous/WinUAE's 68040
// fpuop_save writer (WBTEMP at +24, FPIARCU at +40, CMDREG3B at +52,
// STAG/GRS at +60, CMDREG1B at +64, DTAG/WBTE15 at +68, flags at +72,
// FPTEMP at +76, ETEMP at +88; CU_SAVEPC and the reserved words zero)
function [31:0] fsave_busy_word;
	input [4:0] n;
	begin
		case (n)
			5'd0:  fsave_busy_word = FPU_BUSY_HEADER;
			5'd6:  fsave_busy_word = {fpu_fstate_wbt[95:80], 16'd0};
			5'd7:  fsave_busy_word = fpu_fstate_wbt[63:32];
			5'd8:  fsave_busy_word = fpu_fstate_wbt[31:0];
			5'd10: fsave_busy_word = fpu_fstate_fpiar_c;
			5'd13: fsave_busy_word = {fpu_fstate_cmd3, 16'd0};
			5'd15: fsave_busy_word = {fpu_fstate_stag, 3'd0,
			                          fpu_fstate_grs, 23'd0};
			5'd16: fsave_busy_word = {fpu_fstate_cmd1, 16'd0};
			5'd17: fsave_busy_word = {fpu_fstate_dtag, 8'd0,
			                          fpu_fstate_wbte15, 20'd0};
			5'd18: fsave_busy_word = {5'd0, fpu_fstate_flags[2],
			                                fpu_fstate_flags[1], 4'd0,
			                                fpu_fstate_flags[0], 20'd0};
			5'd19: fsave_busy_word = fpu_fstate_fpt[95:64];
			5'd20: fsave_busy_word = fpu_fstate_fpt[63:32];
			5'd21: fsave_busy_word = fpu_fstate_fpt[31:0];
			5'd22: fsave_busy_word = fpu_fstate_et[95:64];
			5'd23: fsave_busy_word = fpu_fstate_et[63:32];
			5'd24: fsave_busy_word = fpu_fstate_et[31:0];
			default: fsave_busy_word = 32'd0;
		endcase
	end
endfunction

function [31:0] fsave_unimp_word;
	input [3:0] n;
	begin
		// Internal fields use the $41 indices. $40 skips words 1 and 2.
		case ((FPU_REV40 && n != 0) ? n + 4'd2 : n)
			4'd0:  fsave_unimp_word = FPU_UNIMP_HEADER;
			4'd1:  fsave_unimp_word = {fpu_fstate_cmd3, 16'd0};
			4'd2:  fsave_unimp_word = 32'd0;
			4'd3:  fsave_unimp_word = {fpu_fstate_stag, 3'd0,
			                           fpu_fstate_grs, 23'd0};
			4'd4:  fsave_unimp_word = {fpu_fstate_cmd1, 16'd0};
			4'd5:  fsave_unimp_word = {fpu_fstate_dtag, 8'd0,
			                           fpu_fstate_wbte15, 20'd0};
			4'd6:  fsave_unimp_word = {5'd0, fpu_fstate_flags[2],
			                                  fpu_fstate_flags[1], 4'd0,
			                                  fpu_fstate_flags[0], 20'd0};
			4'd7:  fsave_unimp_word = fpu_fstate_fpt[95:64];
			4'd8:  fsave_unimp_word = fpu_fstate_fpt[63:32];
			4'd9:  fsave_unimp_word = fpu_fstate_fpt[31:0];
			4'd10: fsave_unimp_word = fpu_fstate_et[95:64];
			4'd11: fsave_unimp_word = fpu_fstate_et[63:32];
			default: fsave_unimp_word = fpu_fstate_et[31:0];
		endcase
	end
endfunction

// IEEE condition predicate over FPSR condition codes {N, Z, I, NAN}
function fp_cond;
	input [5:0] pred;
	input [3:0] cc;
	reg n, z, nan;
	begin
		n = cc[3]; z = cc[2]; nan = cc[0];
		case (pred[3:0])
			4'h0: fp_cond = 0;                          // F / SF
			4'h1: fp_cond = z;                          // EQ
			4'h2: fp_cond = !(nan | z | n);             // OGT
			4'h3: fp_cond = z | !(nan | n);             // OGE
			4'h4: fp_cond = n & !(nan | z);             // OLT
			4'h5: fp_cond = z | (n & !nan);             // OLE
			4'h6: fp_cond = !(nan | z);                 // OGL
			4'h7: fp_cond = !nan;                       // OR
			4'h8: fp_cond = nan;                        // UN
			4'h9: fp_cond = nan | z;                    // UEQ
			4'hA: fp_cond = nan | !(n | z);             // UGT
			4'hB: fp_cond = nan | z | !n;               // UGE
			4'hC: fp_cond = nan | (n & !z);             // ULT
			4'hD: fp_cond = nan | z | n;                // ULE
			4'hE: fp_cond = !z;                         // NE
			default: fp_cond = 1;                       // T / ST
		endcase
	end
endfunction

// Non-branch instructions which the MC68040 defines as changes of flow for
// T0 tracing because they synchronize/refill the instruction pipeline.
// gencpu marks them with trace_t0_68040_only(); note MOVEC is listed for
// the TO-control-register direction only (i_MOVE2C, $4E7B).  Reading a
// control register ($4E7A, i_MOVEC2) changes nothing and does not trace --
// cputest Basic/MOVEC2 expects the trace after the $4E7B in its sequence,
// not after the $4E7A that precedes it.
// Taken branches/returns are handled by go_pc; FDBcc/FMOVEM need extension
// word information and set t0_force in their decode states.
function t0_special;
	input [15:0] op;
	begin
		t0_special =
		    op == 16'h007c || op == 16'h027c || op == 16'h0a7c || // to SR
		    op == 16'h4e71 ||                                      // NOP
		    // STOP is absent: it never reaches the fetch_next boundary --
		    // S_STOP_LD makes its own T0 decision (changed-bits rule).
		    op == 16'h4e7b ||                                      // MOVEC to CR
		    (op & 16'hfff8) == 16'h4e60 ||                         // MOVE An,USP
		    // MOVE USP,An ($4E68-F) does not trace: gencpu marks only
		    // i_MVR2USP with trace_t0_68040_only, the same on-silicon
		    // narrowing hardware already proved for MOVEC ($4E7B only).
		    (op & 16'hffc0) == 16'h46c0 ||                         // MOVE to SR
		    (op[15:12] == 4'h0 && op[11:8] == 4'he &&
		     op[7:6] != 2'b11) ||                                  // MOVES
		    (op[15:12] == 4'h0 && op[11] && !op[8] && op[7:6] == 2'b11 &&
		     op[10:9] != 2'b00) ||                                 // CAS/CAS2
		    // op[8] discriminates CAS ($0AC0/$0CC0/$0EC0, clear) from the
		    // dynamic bit ops BSET Dn,<ea> for D5-D7 ($0Bxx/$0Dxx/$0Fxx,
		    // set): hardware cputest basic/all failed BSET.B D5,(A6)
		    // under T0 with a phantom trace before the bit was added.
		    op[15:8] == 8'hf4 || op[15:8] == 8'hf5 ||              // CINV/CPUSH/PFLUSH/PTEST
		    op[15:8] == 8'hf3;                                     // FSAVE/FRESTORE
	end
endfunction

// Unimplemented FP instruction: vector 11, format $2.  Register/immediate
// forms identify the faulting FP instruction, while a resolved memory form
// carries its operand EA (hardware FINT/FINTRZ corpus behavior).
// Malformed source EA.  A hardware opmode reports the plain format-$0
// F-line; an FPSP-emulated one reports through the unimplemented route.
// Both are written from ONE exc() site so the format and PC selection is
// a 2:1 mux on a single predicate rather than two more writers into the
// exception-format priority tree, which is the core's critical path.
task go_fp_ea_fault;
	input hw;
	begin
		fpu_req <= 0;
		exc(`AP040_VEC_FLINE, hw ? 4'd0 : 4'd2,
		    hw ? pc_i : pc,
		    hw ? 32'd0 : (fp_ea_v ? t_a : pc_i));
	end
endtask

task go_fp_unimp;
	begin
		fpu_req <= 0;
		exc(`AP040_VEC_FLINE, 4'd2, pc, fp_ea_v ? t_a : pc_i);
	end
endtask

// Unsupported data type (denormal/unnormal operands, packed decimal).
// EVERY vector-55 fault stacks format $3: opclass 011 stores carry the
// next PC and the destination address; source faults keep the PC on the
// FP instruction with the source address in EA -- and when the source
// has no address (register or immediate operand) the EA field simply
// reads zero.  Hardware cputest FABS.P #imm and FDMOVE.S Dn both expect
// 00,00,42,05,00,00,30,dc,00,00,00,00; the format-$0 no-EA variant this
// implementation used before came from the UM's "pre-instruction" prose
// and does not match the reference (WinUAE raises all of these through
// its post-instruction path with regs.fp_ea = 0).
task go_fp_unsupp;
	input        post;
	input        packed_early;
	input        has_ea;
	input [31:0] ea;
	begin
		fpu_req <= 0;
		// Datatype faults are POST-instruction exceptions: format $3 with
		// the FOLLOWING instruction's PC, for sources as well as stores.
		// AP040 used to stack the faulting instruction's own PC for source
		// operands (and pc-2 for a statically unsupported packed store).
		// The v20 corpus masked the stacked PC out of every frame
		// comparison, so nothing contradicted it; the v24 corpus checks
		// that field and rejects both, and WinUAE agrees explicitly
		// (fpp.cpp, "simplification: always mid/post-instruction
		// exception" -> newcpu_common.cpp stacks currpc with format $3).
		if (post) exc(`AP040_VEC_FP_UNSUP, 4'd3, pc, ea);
		else      exc(`AP040_VEC_FP_UNSUP, 4'd3, pc, has_ea ? ea : 32'd0);
	end
endtask

// Does the 040 execute this opmode in hardware, or is it one of the
// FPSP-emulated ones (FINT, FSIN, FMOD, ...)?  Mirrors ap040_fpu's
// op_in_hw.  It matters at the malformed-EA sites below: WinUAE's
// get_fp_value runs fault_if_unimplemented_680x0 BEFORE it rejects a Dn
// or An source, so an unimplemented INSTRUCTION with an illegal EA is
// still reported through the FPSP route (vector 11, format $2, PC of the
// following instruction), not as a plain format-$0 F-line.
function fp_op_in_hw;
	input [6:0] op;
	begin
		case (op)
			7'h00, 7'h40, 7'h44,          // FMOVE, FSMOVE, FDMOVE
			7'h18, 7'h58, 7'h5C,          // FABS, FSABS, FDABS
			7'h1A, 7'h5A, 7'h5E,          // FNEG, FSNEG, FDNEG
			7'h38, 7'h3A,                 // FCMP, FTST
			7'h22, 7'h62, 7'h66,          // FADD, FSADD, FDADD
			7'h28, 7'h68, 7'h6C,          // FSUB, FSSUB, FDSUB
			7'h23, 7'h27, 7'h63, 7'h67,   // FMUL, FSGLMUL, FSMUL, FDMUL
			7'h20, 7'h24, 7'h60, 7'h64,   // FDIV, FSGLDIV, FSDIV, FDDIV
			7'h04, 7'h41, 7'h45:          // FSQRT, FSSQRT, FDSQRT
				fp_op_in_hw = 1;
			default:
				fp_op_in_hw = 0;
		endcase
	end
endfunction

// 68040 F-line opmode classification (WinUAE fault_if_nonexisting_opmode):
// 0 = existing (hardware, or the unimplemented-instruction FPSP route with
// its format-$2 frame); 1 = nonexisting, reported IMMEDIATELY as vector 11
// with the plain format-$0 frame and NO FPIAR or FPSR side effects;
// 2 = opmodes $78-$7F, which the 040 reports as vector 4 of all things
// (WinUAE: "Unexpected, isn't it?!").
function [1:0] fp_opmode_class;
	input [6:0] op;
	begin
		case (op)
			7'h05, 7'h07, 7'h0B, 7'h13, 7'h17, 7'h1B,
			7'h29, 7'h2A, 7'h2B, 7'h2C, 7'h2D, 7'h2E, 7'h2F,
			7'h39, 7'h3B, 7'h3C, 7'h3D, 7'h3E, 7'h3F,
			7'h42, 7'h43, 7'h46, 7'h47,
			7'h48, 7'h49, 7'h4A, 7'h4B, 7'h4C, 7'h4D, 7'h4E, 7'h4F,
			7'h50, 7'h51, 7'h52, 7'h53, 7'h54, 7'h55, 7'h56, 7'h57,
			7'h59, 7'h5B, 7'h5D, 7'h5F,
			7'h61, 7'h65, 7'h69, 7'h6A, 7'h6B, 7'h6D, 7'h6E, 7'h6F,
			7'h70, 7'h71, 7'h72, 7'h73, 7'h74, 7'h75, 7'h76, 7'h77:
				fp_opmode_class = 2'd1;
			7'h78, 7'h79, 7'h7A, 7'h7B, 7'h7C, 7'h7D, 7'h7E, 7'h7F:
				fp_opmode_class = 2'd2;
			default:
				fp_opmode_class = 2'd0;
		endcase
	end
endfunction

// Malformed FPU command/effective-address combinations are still F-line
// opcodes.  The 68040 reports vector 11 with the normal format-$0 frame;
// this is distinct from the format-$2 FPSP route above.
task go_fp_fline;
	begin
		fpu_req <= 0;
		exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
	end
endtask

// Abandon the queue and stop the fill engine.  Whatever is in flight is
// reaped and discarded, so nothing can re-arm the engine between here and the
// redirect: only issue_ifetch arms it again.
task epf_flush;
	begin
		epf_count <= 0;
		epf_head  <= 0;
		epf_fill  <= 0;
		epf_armed <= 0;
		epf_err   <= 0;
		epf_brf   <= 0;
		brf_valid <= 0;
		if (epf_pend) epf_kill <= 1;
		epf_flushed = 1;
	end
endtask

// Point the queue at a, under function code s, and start filling.  A queue
// already tracking that address in that context is kept: the words are
// resident, in flight, or about to be requested, and the caller's S_FETCH
// pops them.  Anything else is a flow, context or FC change and flushes.
task issue_ifetch;
	input [31:0] a;
	input        s;
	reg line_hit, refill_hit;
	begin
		line_hit = brf_tag == a[31:6] && brf_super == s;
		refill_hit = line_hit && (brf_run[a[5:1]] >= 4'd4);
		if (epf_armed && epf_next == a && epf_super == s) begin
			// the stream already runs here: nothing to do
			// A drained branch-refill stream reached its fall-through path;
			// allow normal speculative filling to resume.
			if (epf_count == 4'd0) epf_brf <= 0;
		end
		else begin
			epf_head  <= 0;
			epf_err   <= 0;
			epf_next  <= a;
			epf_super <= s;
			epf_armed <= 1;
			epf_flushed = 1;
			if (epf_pend) epf_kill <= 1;
			else epf_kill <= 0;
			if (refill_hit) begin : brf_seed
				// Seed every valid word from the target to the end of the
				// sector, up to the queue's eight.  Four (the entry
				// condition) cover the canonical two-op DBcc loop; a longer
				// loop body seeded whole needs no fetch at all, where the
				// four-word seed left it taking demand fetches once its
				// sequences got short (Sieve inner loop, 2026-09-14).
				// Only the word count is computed here; the data muxes live
				// in one shared block at the end of the always block, since
				// this task is expanded at eight call sites (eight copies of
				// eight 16-way word muxes doubled the core's logic).  The
				// count is the sector's valid run from the target (brf_run).
				epf_brf <= 1;
				brf_seed_n = brf_run[a[5:1]];
				brf_seed_req = 1;
				brf_seed_a   = a[5:1];
				epf_count <= brf_seed_n;
				epf_fill  <= brf_seed_n[2:0];
				epf_ftail <= a + {27'd0, brf_seed_n, 1'b0};
			end
			else begin
				epf_brf   <= 0;
				epf_count <= 0;
				epf_fill  <= 0;
				epf_ftail <= a;
			end
			if (!refill_hit && ifr_avail) begin
				// The port is free: issue the redirect now rather than
				// leaving it to the engine one cycle later.  A longword
				// aligned fetch takes both words in one request.  Alignment
				// is what makes it safe: an aligned longword cannot span a
				// page, so it cannot translate or fault differently than the
				// two halves would have (the exception prefetch below stays
				// word-wise precisely because its $FFE entry CAN span).
				ifr_req <= 1;
				ifr_size <= a[1] ? `AP040_SZ_W : `AP040_SZ_L;
				ifr_addr <= a;
				ifr_fc <= s ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
				epf_pend <= 1;
				epf_pend_lw <= ~a[1];
				epf_pend_seed <= 1;
				epf_kill <= 0;
				epf_issue = 1;
			end
		end
	end
endtask

// Start the architecturally required four-longword prefetch which concludes
// reset and exception processing.  Word requests are intentional: an entry
// at page offset $FFE must translate/fault the second page independently.
task exception_prefetch;
	input [31:0] a;
	input        s;
	begin
		epf_flush;
		epf_base <= a;
		epf_next <= a;
		epf_ftail <= a;
		epf_super <= s;
		pc <= a;
		pc_i <= a;
		ifr_req <= 1;
		ifr_size <= `AP040_SZ_W; ifr_addr <= a;
		ifr_fc <= s ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
		epf_issue = 1;
		state <= S_EPF_FILL;
	end
endtask

// Enter the processor-halted state after an unrecoverable double fault.
// A bus error terminates the external cycle without returning mem_ack, so a
// held mem_req would otherwise be re-issued by the bus adapter on the next
// idle clock.  Quiesce every request source at the point of entry as well as
// in S_HALT itself.
task fatal_halt;
	begin
		mem_req <= 0;
		ifr_req <= 0;
		m_issued <= 0;
		pt_req <= 0;
		pf_req <= 0;
		cinv_req <= 0;
		fpu_req <= 0;
		epf_flush;
		epf_pend <= 0;
		epf_kill <= 0;
		fc_ovr_v <= 0;
		lk_cyc <= 0;
		fault_r <= 1;
		state <= S_HALT;
	end
endtask

task rfw;
	input [3:0] a;
	input [31:0] d;
	begin
		rf_we <= 1; rf_waddr <= a; rf_wdata <= d;
		// this cycle's write, for the dispatches after the case that
		// select a port the write would land on (dispatch_dbcc)
		rfw_now = 1; rfw_now_a = a;
	end
endtask

// SK_IMM source with a register destination: consume the immediate here,
// select the destination on port B and go straight to S_PIPE_REGS
// (capture and retire) instead of letting S_PIPE_START copy the same
// values a cycle later.  Same guards as immf's decode-time inline; when
// they fail the established path is taken.
task immf_reg;
	input [1:0] n;
	input [3:0] da;
	reg  [31:0] v;
	begin
		v = 32'd0;
		if (state == S_DECODE && !epf_flushed && !epf_pend && !ifr_ack &&
		    ((n == 2'd2) ? epf_ready_pc2 : epf_ready_pc)) begin
			v = (n == 2'd2) ? {epf_data[epf_head], epf_data[epf_head + 3'd1]}
			                : {16'd0, epf_data[epf_head]};
			imm <= v; x_ext <= v; src_val <= v;
			imm_n <= n; r_imm_ret <= S_PIPE_START;
			pc <= pc + ((n == 2'd2) ? 32'd4 : 32'd2);
			epf_pop = n;
			epf_issue = 1;
			rr_b <= da;
			state <= S_PIPE_REGS;
		end
		else immf(n, S_PIPE_START);
	end
endtask

// Extension-word fetch.  immf is called from some fifty sites; the call
// now only loads carriers and immf_now runs once after the case statement
// (the queue-head pop and the S_IMMF entry).  No arm writes imm, pc,
// imm_n, r_imm_ret or state after its immf call on the same path
// (audited 2026-09-15), and no arm flushes the queue after it.
task immf;
	input [1:0] n;
	input [7:0] ret;
	begin
		igo = 1; igo_n = n; igo_ret = ret;
	end
endtask

task immf_now;
	input [1:0] n;
	input [7:0] ret;
	begin
		imm_n <= n; imm <= 0;
		r_imm_ret <= ret;
		// Decode commonly asks for an extension word which the prefetch queue
		// already holds.  Consume it here instead of entering S_IMMF merely to
		// perform the same pop on the following edge.  Keep later-state callers
		// on the established path: some build multi-part EA/FPU operands around
		// this task and need their current state transition as a boundary.
		//
		// Do not cross an instruction-bus completion or an outstanding queue
		// fill.  Some return states can quickly start a data/MMU transaction;
		// advancing them while the old fetch still owns the physical bus lets
		// the table walker overlap that bus.  epf_issue also suppresses a new
		// speculative fill on this same edge, preserving that ownership rule.
		if (state == S_DECODE && !epf_flushed && !epf_pend && !ifr_ack &&
		    n == 2'd2 && epf_ready_pc2) begin
			imm <= {epf_data[epf_head], epf_data[epf_head + 3'd1]};
			pc <= pc + 32'd4;
			epf_pop = 2'd2;
			epf_issue = 1;
			state <= ret;
		end
		else if (state == S_DECODE && !epf_flushed && !epf_pend && !ifr_ack &&
		         epf_ready_pc) begin
			imm <= {16'd0, epf_data[epf_head]};
			pc <= pc + 32'd2;
			epf_pop = 2'd1;
			epf_issue = 1;
			if (n == 2'd1) state <= ret;
			else begin imm_n <= 2'd1; state <= S_IMMF; end
		end
		else state <= S_IMMF;
	end
endtask

// A data transfer that crosses a page boundary would be translated once
// and then incremented physically by the bus adapter, so the second page
// would be accessed through the first page's mapping.  Such transfers are
// issued one byte at a time instead: every byte is translated on its own,
// faults report the byte that failed, and the history bits of both pages
// are updated.  Only misaligned transfers can cross a boundary.
wire  [2:0] m_nbytes = (m_size == `AP040_SZ_B) ? 3'd1 :
                       (m_size == `AP040_SZ_W) ? 3'd2 : 3'd4;
wire [31:0] m_pgmask = tc[14] ? 32'h0000_1FFF : 32'h0000_0FFF;
wire        m_cross  = tc[15] &&
                       (((m_addr_r & m_pgmask) + {29'd0, m_nbytes}) >
                        (m_pgmask + 32'd1));

// Data transfer issue.  mrd/mwr are called from some eighty sites; each
// call used to expand the request setup and the early-issue decision
// (a second address and data mux onto mem_addr_q/mem_wdata) in place.
// The calls now only load carriers; mem_issue runs once after the case
// statement, where the same registered state and port conditions hold.
// No arm writes m_*, mem_*, r_m_ret or state after its mrd/mwr call on
// the same path (audited 2026-09-15).
task mrd;
	input [31:0] a;
	input [1:0] size;
	input [7:0] ret;
	begin
		mgo = 1; mgo_wr = 0; mgo_a = a; mgo_sz = size; mgo_ret = ret;
	end
endtask

task mwr;
	input [31:0] a;
	input [1:0] size;
	input [31:0] d;
	input [7:0] ret;
	begin
		mgo = 1; mgo_wr = 1; mgo_a = a; mgo_sz = size; mgo_d = d; mgo_ret = ret;
	end
endtask

task mem_issue;
	begin
		m_addr_r <= mgo_a; m_size <= mgo_sz; m_wr <= mgo_wr; r_m_ret <= mgo_ret;
		if (mgo_wr) m_wdat <= mgo_d;
		state <= mgo_wr ? S_MWR : S_MRD;
        // Select the destination base/index while the source is being read.
        // Skip any source extension consumed on this edge; the three-bit
        // queue index wraps at eight words. Do not consume destination words.
        if (!mgo_wr && mgo_ret == S_PIPE_SDONE && p_src == SK_MEM &&
            p_dst == DK_MEM && exec_kind == EK_ALU &&
            alu_op == `AP040_ALU_MOVE && !p_rmw && dst_mode_r == 3'b110 &&
            epf_ready_pc && !epf_flushed && epf_count > {2'd0,epf_pop} &&
            !epf_data[epf_head + {1'b0,epf_pop}][8]) begin
            rr_a <= {1'b1,dst_rn_r};
            rr_b <= epf_data[epf_head + {1'b0,epf_pop}][15:12];
        end
        // P179: the simple destination modes' base as well, so a read
        // acknowledged in its first S_MRD clock finds it settled on port A
        // (the S_MRD preselect below only lands a clock later).
        else if (!mgo_wr && mgo_ret == S_PIPE_SDONE && p_src == SK_MEM &&
            p_dst == DK_MEM && exec_kind == EK_ALU &&
            alu_op == `AP040_ALU_MOVE && !p_rmw &&
            (dst_mode_r == 3'b010 || dst_mode_r == 3'b011 ||
             dst_mode_r == 3'b100 || dst_mode_r == 3'b101))
            rr_a <= {1'b1,dst_rn_r};
		// Normal within-page operand reads from on-board RAM (from the operand
		// pipe) and within-page destination writes (from S_EXEC) claim the shared
		// port while entering S_MRD/S_MWR, removing the request-setup cycle;
		// MMIO, page-crossing transfers, exception frames and the system/FPU
		// helpers keep it.  Completion, faults and retirement are unchanged.
		// The whitelist is also what keeps MOVES correct: fc_ovr_v/fc_ovr are
		// registers written by S_MOVES_RD/WR in the cycle they call mrd/mwr,
		// so an issue from THOSE states would read the previous space (FC 5
		// for FC 1; Adam Polkosnik's 95e29fb carries the function code on
		// the carriers instead).  t_moves_fc fails the moment a MOVES state
		// is added below.
		// The stack pops and the MOVEM transfers issue in place as well
		// (2026-09-17): their addresses are registers or the forwarded A7,
		// and each of these states hints the read a cycle ahead (hint_pop),
		// so the read is acknowledged in the cycle it is presented.  Alan
		// withdrew the same sites in 2026-09-14 because every in-place site
		// was a source on the mem_addr_q mux of the hint-to-acknowledge
		// path; on the dedicated hint bus mem_addr_q is a register input.
		if (((!mgo_wr && (state == S_PIPE_START || state == S_PIPE_SRD ||
		                  state == S_PIPE_DEA || state == S_DECODE ||
		                  state == S_RET1 || state == S_UNLK1 ||
		                  state == S_MOVEM_LOOP || retire_move_read || retire_store_read || retire_read_read || ret_after_unlk || fpu_rd_next || mm_rd_next || hint_early_read || hint_fpu_read || pipe_load_launch)) ||
		     (mgo_wr && (reg_move_store_prepare || move_store_read_handoff || state == S_EXEC || state == S_PIPE_DEA || state == S_MOVEM_LOOP ||
		                 // the pushes: BSR.B from decode, BSR.W, JSR, PEA,
		                 // LINK -- registered data (pc, ea_addr, port A
		                 // selected a state earlier) at dbg_a7 - 4
		                 state == S_DECODE || state == S_BCC_EXT ||
		                 state == S_JSR1 || state == S_PEA1 || pea_d16_push ||
		                 state == S_LINK2 || hint_st_fpu || hint_st_fmovem || hint_st_fpgo || pipe_load_launch))) &&
		    mgo_a[31:28] == 4'h0 &&
            ((!mem_req && !mem_ack) || (mgo_wr && move_store_read_handoff) ||
             (!mgo_wr && (retire_store_read || retire_read_read || ret_after_unlk || fpu_rd_next || mm_rd_next))) &&
        // A transfer wholly inside 4KB cannot cross either supported MMU
        // page size. Crossing accesses retain delayed issue and byte splitting.
		    ((mgo_sz == `AP040_SZ_B) ||
		     ((mgo_sz == `AP040_SZ_W) && mgo_a[11:0] != 12'hfff) ||
		     ((mgo_sz == `AP040_SZ_L) && mgo_a[11:0] <= 12'hffc))) begin
			mem_req <= 1; mem_write <= mgo_wr; mem_instr_q <= 0;
			mem_size <= mgo_sz; mem_addr_q <= mgo_a;
			if (mgo_wr) mem_wdata <= mgo_d;
			fc_r <= fc_ovr_v ? fc_ovr :
			        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
			m_issued <= 1;
			epf_issue = 1;
			// P182: whether the hint bus carried this read when it issued
			// (row and word; a false match only costs a held clock)
			mrd_hinted <= !mgo_wr && !mem_hint_instr &&
			              (mem_hint_addr[11:2] == mgo_a[11:2]);
			st_hinted  <= mgo_wr && !mem_hint_instr &&
			              (mem_hint_addr[11:2] == mgo_a[11:2]);
		end
		else m_issued <= 0;
	end
endtask


// A register write landing this cycle is the base register's (port A
// shows the old value): see hint_ext_ok and ea_start's inline paths.
wire        base_landing  = rf_we && (rf_waddr == rr_a);

task ea_start;
	input [2:0] mode;
	input [2:0] rn;
	input [1:0] size;
	input [7:0] ret;
	begin
		ea_mode <= mode; ea_rn <= rn; ea_size <= size;
		// Select An while entering the EA engine.  The asynchronous register
		// file output is then settled when S_EA_DISP handles the simple An
		// modes, avoiding a separate base-register state.
		rr_a <= {1'b1, rn};
		ea_pcmode <= 0; ea_pcb <= pc;
		r_ea_ret <= ret; state <= S_EA_DISP;
	end
endtask

// Ordinary operands can select the base register and set up their extension
// request together. Do not consume an extension here: S_IMMF supplies the
// qualified edge for port A to settle, preserves queue/fault handling, and
// retains the indexed pending-write fallback before capturing the base.
// Specialized EA callers keep their existing S_EA_DISP boundary.
task ea_operand_start;
	input [2:0] mode;
	input [2:0] rn;
	input [1:0] size;
	input [7:0] ret;
	reg ext_inline;
	reg        direct, drd;      // a direct path resolved here: read / no read
	reg [31:0] da;               // its address
	reg [7:0]  dret;             // where its read returns
	begin
		ea_start(mode, rn, size, ret);
		direct = 0; drd = 0; da = 32'd0; dret = ret;
		// From S_PIPE_START, with the base register already on port A and
		// the extension word resident in the queue, consume it here instead
		// of spending S_IMMF on the same pop.  A pending register/stack
		// write keeps the old path so the base captured below is never
		// stale.  An outstanding queue fetch no longer blocks this: the
		// data transfer it leads to cannot issue while that fetch is
		// pending (mrd/mwr and S_MRD/S_MWR all hold on epf_pend), so
		// advancing the EA computation overlaps the fetch instead of
		// waiting for it in S_IMMF.
		ext_inline = (state == S_PIPE_START) && !epf_flushed &&
		             !ifr_ack && epf_ready_pc && (rr_a == {1'b1, rn}) &&
		             !base_landing && !aux_we;
		case (mode)
			// (An), (An)+, -(An) destination with the base settled on port
			// A: record the address (and the register update) here, as
			// S_EA_DISP would a cycle later, and land where S_PIPE_DEA
			// would have.  A pending register write keeps the old path.
			3'b010, 3'b011, 3'b100:
				if (ret == S_PIPE_DEA && (state == S_PIPE_START) &&
				    (rr_a == {1'b1, rn}) && !base_landing && !aux_we) begin
					da = (mode == 3'b100) ? rf_rdata_a - an_adj(rn, size) : rf_rdata_a;
					if (mode == 3'b011) begin
						rfw({1'b1, rn}, rf_rdata_a + an_adj(rn, size));
						u_rec({1'b1, rn}, rf_rdata_a);
					end
					else if (mode == 3'b100) begin
						rfw({1'b1, rn}, da);
						u_rec({1'b1, rn}, rf_rdata_a);
					end
					dst_addr <= da;
					direct = 1; drd = p_rmw; dret = S_PIPE_DDONE;
				end
			3'b101:
				if (ext_inline && ret == S_PIPE_SRD) begin
					// d16(An) source: base on port A, displacement at the
					// queue head, so the read issues now.  Its hint went
					// out this cycle (hint_pipe_addr computes the same
					// sum), and S_EA_D16 and S_PIPE_SRD are skipped.
					pc <= pc + 32'd2;
					epf_pop = 2'd1;
					epf_issue = 1;
					if (p_dst == DK_REG) rr_b <= p_dreg;
					da = rf_rdata_a + sxw(epf_data[epf_head]);
					direct = 1; drd = 1; dret = S_PIPE_SDONE;
				end
				else if (ext_inline && ret == S_PIPE_DEA) begin
					// d16(An) destination: same, landing where S_PIPE_DEA
					// would have (a read-modify-write reads now).
					pc <= pc + 32'd2;
					epf_pop = 2'd1;
					epf_issue = 1;
					da = rf_rdata_a + sxw(epf_data[epf_head]);
					dst_addr <= da;
					direct = 1; drd = p_rmw; dret = S_PIPE_DDONE;
				end
				else if (ext_inline) begin
					imm <= {16'd0, epf_data[epf_head]};
					pc <= pc + 32'd2;
					epf_pop = 2'd1;
					epf_issue = 1;
					state <= S_EA_D16;
				end
				else immf(2'd1, S_EA_D16);
			3'b110:
				if (ext_inline) begin
					extw <= epf_data[epf_head];
					rr_b <= epf_data[epf_head][15:12];
					ea_base_v <= rf_rdata_a;
					pc <= pc + 32'd2;
					epf_pop = 2'd1;
					epf_issue = 1;
					state <= S_EA_EXTW2;
				end
				else immf(2'd1, S_EA_EXTW);
			3'b111: begin
				case (rn)
					3'b000: begin ea_absl <= 0; immf(2'd1, S_EA_ABS); end
					3'b001: begin ea_absl <= 1; immf(2'd2, S_EA_ABS); end
					3'b010: begin ea_pcmode <= 1; immf(2'd1, S_EA_D16); end
					3'b011: begin ea_pcmode <= 1; immf(2'd1, S_EA_EXTW); end
					default: begin end
				endcase
			end
			default: begin end
		endcase
		// the one read issue of this task: every direct path above lands here
		if (direct) begin
			if (drd) mrd(da, size, dret);
            else if (reg_move_store_prepare) begin
                if (p_flags) sr[4:0] <= alu_fl;
                mwr(da, size, alu_res, S_NEXT);
            end
			else state <= S_EXEC;
		end
	end
endtask

// Exception entry.  exc is called from some fifty sites; the call now
// only loads carriers and exc_now runs once after the case statement.
// The few sites that override the entry state on the same path after
// the call (a BSUN/TRAPcc entry delayed behind an FPSR write, the trace
// taken at the retire boundary) call exc_now directly.
task exc;
	input [7:0] vec;
	input [3:0] fmt;
	input [31:0] spc;
	input [31:0] addr;
	begin
		xgo = 1; xgo_vec = vec; xgo_fmt = fmt; xgo_spc = spc; xgo_addr = addr;
	end
endtask

task exc_now;
	input [7:0] vec;
	input [3:0] fmt;
	input [31:0] spc;
	input [31:0] addr;
	begin
		exc_vec <= vec; exc_spc <= spc; exc_addr <= addr;
		exc_is_irq <= 0; exc_pass2 <= 0;
		// A T0 trace does NOT survive an exception on the 68040.  This
		// used to arm one for illegal/privilege/A-line/F-line, reading
		// WinUAE's Exception_cpu_oldpc as if every exception ran through
		// it.  Only the INTERNAL exceptions do -- gencpu emits
		// exception_cpu() solely for divide-by-zero, CHK, TRAPV, TRAP #n
		// and the RTE format error -- and on a 68040 that path then forces
		// t0 = false for exactly those vectors, while everything else
		// (op_illg's vector 4 included) goes through plain Exception(),
		// whose exception_check_trace clears T0 outright.  So no exception
		// leaves a T0 trace pending.  Hardware agrees: cputest basic/all
		// and fbasic/all both reported "Got unexpected trace exception"
		// after the ILLEGAL that terminates every test, on plain integer
		// instructions as well as FP ones.
		//
		// The texc machinery itself stays: it still delivers a trace that
		// a simultaneous INTERRUPT preempted (see fetch_next and go_pc),
		// which is a different and real case.
		// Exception stack and vector accesses are always supervisor-data
		// references.  In particular, do not let a faulting MOVES retain its
		// SFC/DFC override into exception processing.
		fc_ovr_v <= 0;
		flow_t0_pend <= 0;
		// Exception processing owns the memory port from here: the fetch
		// queue is abandoned and cannot re-arm until the handler's
		// prefetch runs.  A queue fetch already on the bus is left to
		// finish -- dropping a request the cache has accepted would lose
		// its acknowledge -- and its data is discarded by epf_kill.
		epf_flush;
		// A queue fetch the platform has not accepted yet is withdrawn; one
		// already on the bus is left to finish under epf_kill (P171).
		if (!epf_pend || !ifr_pres) begin ifr_req <= 0; epf_pend <= 0; epf_kill <= 0; end
		mem_req <= 0;
		// A faulted/aborted locked sequence ends here: the 040 drops LOCK
		// on the fault, and a stale lk_cyc would throttle the handler's
		// fetch queue (fetch_next is not on the exception entry path).
		lk_cyc <= 0;
		// Carry the format in the entry state instead of driving exc_fmt
		// directly from the full instruction decoder.  The entry state loads
		// the architectural frame field on the following qualified edge.
		case (fmt)
			4'd2: state <= S_EXC0_F2;
			4'd3: state <= S_EXC0_F3;
			4'd4: state <= S_EXC0_F4;
			default: state <= S_EXC0;
		endcase
	end
endtask

// Common exception-entry work.  Each caller supplies a constant selected by
// a registered state, keeping the exc_fmt input cone short while retaining
// the original exception latency.
task exc0_enter;
	input [3:0] fmt;
	begin
		if (!fpu_bg && !epf_pend) begin
			exc_fmt <= fmt;
			// Kill any address-register rollback records on the way into a
			// non-access exception.  They belong only to the faulting access.
			u0_v <= 0;
			u1_v <= 0;
			sr_saved <= sr;
			sr[13] <= 1;
			sr[15:14] <= 2'b00;
			in_exc <= 1;
			if (exc_is_irq) begin
				sr[10:8] <= irq_lvl_l;
				if (irq_lvl_l == 3'd7) nmi_ack_t <= ~nmi_ack_t;
				else irq_ack_t <= ~irq_ack_t;
			end
			state <= S_EXC1;
		end
	end
endtask

// access error entry: capture the fault shape from the outstanding request
// from_ifr: the fault is on the fetch channel, whose request lives in the
// ifr_* registers (P171); the data channel's is in mem_*.  The frame is
// built from the live request either way, not from a copy landing on
// this edge.
task aerr_start;
	input from_ifr;
	reg [2:0] fcx;
	begin
		fcx = from_ifr ? ifr_fc : fc_r;
		aer_bus  <= from_ifr ? (ifr_berr && !ifr_flt)
		                     : (berr && !mem_flt);   // physical bus error, not an ATC fault
		// FA is the initial byte of the original transfer, even when a
		// page-crossing access has been split and a later byte faults.
		aer_fa   <= from_ifr ? ifr_addr : m_addr_r;
		aer_wr   <= from_ifr ? 1'b0 : mem_write;
		aer_sz   <= from_ifr ? ifr_size : m_cross ? m_size : mem_size;
		aer_wd   <= (!from_ifr && m_cross) ? m_wdat : mem_wdata;
		aer_lk   <= lk_cyc;
		// memory ops wait in S_MRD/S_MWR: the requesting context is
		// identified by the continuation state, not by `state` itself
		aer_m16  <= !from_ifr &&
		            (r_m_ret >= S_M16_RD2 && r_m_ret <= S_M16_INC2);
		// Only operand transfers, not faults while calculating the EA,
		// carry CM. A fault fetching a resumed MOVEM retains its saved EA.
		aer_cm <= mm_resume || (!from_ifr &&
		          (r_m_ret == S_MOVEM_LD || r_m_ret == S_MOVEM_LOOP));
		aer_ea <= mm_start_ea;
		mm_resume <= 0;
		// MOVES faults report the alternate space in TT/TM: FC 0, 3, 4 and 7
		// keep the raw FC with TT = 10; FC 2 and 6 are remapped onto the
		// corresponding data space (WinUAE mmu_bus_error's ismoves block)
		aer_tt   <= (fc_ovr_v && (fcx[1:0] == 2'b00 || fcx[1:0] == 2'b11))
		            ? 2'b10 : 2'b00;
		aer_tm   <= (fc_ovr_v && (fcx[1:0] == 2'b00 || fcx[1:0] == 2'b11)) ? fcx :
		            (fc_ovr_v && fcx[1]) ? {fcx[2], 2'b01} : fcx;
		aer_ma   <= mem_flt && !from_ifr && m_cross &&
		            ((mem_addr_q & ~m_pgmask) != (m_addr_r & ~m_pgmask));
		// Preserve fc_r above for the SSW, then force all frame/vector cycles
		// back to supervisor data space.
		fc_ovr_v <= 0;
		epf_flush;
		mem_req  <= 0;
		// A queue fetch the platform has not accepted yet is withdrawn
		// (a request is only committed once it is the presented channel);
		// one already on the bus is left to finish under epf_kill.
		if (!ifr_pres) begin ifr_req <= 0; epf_pend <= 0; epf_kill <= 0; end
		// The locked sequence ends at the fault (aer_lk above still
		// captures the pre-edge value): the 040 drops LOCK, and a stale
		// lk_cyc would throttle the handler's fetch queue.
		lk_cyc   <= 0;
		state    <= S_AERR0;
	end
endtask

// record an address register update for rollback on an access error
task u_rec;
	input [3:0] r;
	input [31:0] old;
	begin
		if (!u0_v) begin
			u0_v <= 1; u0_reg <= r; u0_old <= old;
		end
		else begin
			u1_v <= 1; u1_reg <= r; u1_old <= old;
		end
	end
endtask

// Access-error SSW. CP/CU/CT stay clear; CM preserves a MOVEM's original EA.
// MOVE16 line faults report SIZE = line with TT0; locked TAS/CAS cycles
// report LK with RW clear (WinUAE mmu_bus_error).
wire [15:0] aer_ssw = {3'b000, aer_cm, aer_ma, ~aer_bus, aer_lk,
                       (~aer_wr & ~aer_lk), 1'b0,
                       aer_m16 ? 2'b11 :
                       (aer_sz == `AP040_SZ_B) ? 2'b01 :
                       (aer_sz == `AP040_SZ_W) ? 2'b10 : 2'b00,
                       aer_m16 ? 2'b01 : aer_tt,
                       aer_tm};

// format $7 frame contents, one word per index (30 words)
function [15:0] aerr_word;
	input [4:0] idx;
	begin
		case (idx)
			5'd0:  aerr_word = sr_saved;
			5'd1:  aerr_word = pc_i[31:16];
			5'd2:  aerr_word = pc_i[15:0];
			5'd3:  aerr_word = 16'h7008;               // format $7, vector 2
			// CM needs the ORIGINAL effective address, not the failed
			// transfer address. All other EA-field behavior is unchanged.
			5'd4:  aerr_word = aer_cm ? aer_ea[31:16] : aer_fa[31:16];
			5'd5:  aerr_word = aer_cm ? aer_ea[15:0] :
			                   aer_m16 ? {aer_fa[15:4], 4'd0} : aer_fa[15:0];
			5'd6:  aerr_word = aer_ssw;
			// WB3S stays CLEAR: this core RESTARTS the faulting
			// instruction after the handler repairs the mapping, so it
			// must not also advertise a pending writeback.  An OS that
			// honours the 040 frame (NetBSD trap.c: "the 68040 doesn't
			// re-run instructions that cause write page faults ... we
			// have to write the value out to memory ourselves") performs
			// every VALID WB3 -- combined with the restart, an RMW store
			// like ld.elf_so's relocation add.l lands TWICE.  Captured
			// live: init's ctor pointer held link VA + 2x load base and
			// NetBSD hung looping on the resulting wild ifetch.  WB3D/A
			// keep the write data for diagnostics; valid stays 0.
			5'd7:  aerr_word = 16'd0;
			5'd10: aerr_word = aer_fa[31:16];          // initial fault address
			5'd11: aerr_word = aer_fa[15:0];
			5'd12: aerr_word = aer_fa[31:16];          // WB3A
			5'd13: aerr_word = aer_fa[15:0];
			5'd14: aerr_word = aer_wd[31:16];          // WB3D
			5'd15: aerr_word = aer_wd[15:0];
			default: aerr_word = 16'd0;                // writeback/push slots
		endcase
	end
endfunction

// A memory-mode MOVEM opcode: the only instruction an RTE-supplied CM
// effective address (mm_resume) may be consumed by.  Every other opcode
// entering ir clears the flag, so a forged or edited CM frame cannot
// leak into a later instruction.  Called at the four opcode-load sites
// (retire dispatch, refill dispatch, the exception-prefetch dispatch and
// the lookahead arm's successor) -- this core has no single dispatch task.
function movem_mem_op;
	input [15:0] fw;
	begin
		movem_mem_op = ((fw & 16'hfb80) == 16'h4880) && (fw[5:3] >= 3'd2);
	end
endfunction

// One combinational register-only descriptor serves ordinary decode and
// lookahead at an older ALU retirement. It reads opcode bits only; operands
// and CCR remain owned by the existing execution pipeline. Fetches initiated
// in S_DECODE must use the ordinary path because this decoder still sees ir.
wire [15:0] rd_ir = (state == S_DECODE) ? ir : epf_data[epf_head];
wire [2:0] rd_mode = rd_ir[5:3];
wire [1:0] rd_sz = rd_ir[7:6];
reg rd_valid, rd_quick, rd_flags, rd_wbsup, rd_sextw;
reg rd_imm;                       // immediate source from the queue (lookahead only)
reg rd_shift;                     // register shift (lookahead only)
reg [1:0] rd_immn;                // ... its word count
reg [31:0] rd_qimm;               // quick/MOVEQ immediate
reg [5:0] rd_alu;
reg [1:0] rd_size, rd_ssize, rd_dsize;
reg [3:0] rd_sa, rd_da;
wire [3:0] rd_qval = (rd_ir[11:9] == 3'd0) ? 4'd8 : {1'b0, rd_ir[11:9]};
// the words behind the queue head, for a lookahead immediate
wire [15:0] rd_w1 = epf_data[epf_head + 3'd1];
wire [15:0] rd_w2 = epf_data[epf_head + 3'd2];
wire [31:0] rd_immv = (rd_immn == 2'd2) ? {rd_w1, rd_w2} : {16'd0, rd_w1};
// A short conditional branch at the queue head (Bcc.B, not BSR, not the
// word/long displacement forms) resolves at the producer's retire on the
// producer's own flags: the register-only producers write sr[4:0] from
// alu_fl on this edge, stores leave the flags alone.
wire        rd_is_bcc = (rd_ir[15:12] == 4'h6) && (rd_ir[11:8] != 4'h1) &&
                        (rd_ir[7:0] != 8'h00) && (rd_ir[7:0] != 8'hFF);
wire [31:0] rd_bcc_t  = pc + 32'd2 + sxb(rd_ir[7:0]);
// BRA.B (condition 0000, always true) is resolved by the lookahead arm
// from ANY retire, not only a flag producer's: the redirect goes out at
// the pop instead of from S_DECODE a cycle later.  Only from a state
// without a target arm of its own on go_pc_t_early (the not-taken
// Bcc.W/FBcc and the DBcc/FDBcc exits keep S_DECODE's path, as does a
// LINK/PEA retire from S_MWR) and not after a system retire.  (2026-09-19)
wire        rd_is_bra = rd_is_bcc && (rd_ir[11:8] == 4'h0);
wire        rd_bra_state = !sys_retire &&
                           (state != S_BCC_EXT) && (state != S_DBCC1) &&
                           (state != S_FBCC) && (state != S_FDBCC) &&
                           !((state == S_MWR) && (r_m_ret != S_NEXT));
// The redirect target and the resident-dispatch target formed from
// registers by the current state, so that the hoisted go_pc_now and
// decode_dbcc_brf_now start their refill-seed compare, seed loop and
// tail adder from settled data and only their enables arrive late
// (the acknowledge sits on finish_bcc's choice between the two).  Every
// caller's own expression is listed here; the carrier tasks check the
// two agree in simulation.
wire [31:0] rgo_bcc_ext_t = br_base + (br_long ? imm : sxw(imm[15:0]));
wire [31:0] rgo_dbcc_t    = br_base + sxw(imm[15:0]);
wire [31:0] rgo_decode_t  = pc + sxb(ir[7:0]);
wire        rgo_cond      = cond_true(ir[11:8]);
wire [31:0] go_pc_t_early =
`ifdef AP040_EXPERIMENTAL_PIPELINE
    pipe_branch_taken ? pipe_next_pc :
`endif
    (state == S_MRD && r_m_ret == S_RET2) ? mem_rdata :
	(state == S_RET2 || state == S_RET3)     ? m_val :
	(state == S_BCC_EXT)                     ? rgo_bcc_ext_t :
	(state == S_BSR_PUSH || state == S_JSR2) ? br_tgt :
	// the push acknowledge's own redirect (a BSR/JSR whose target fetch
	// went out at the pop); the lookahead arm's go_pc from a retiring
	// store (r_m_ret == S_NEXT) keeps rd_bcc_t
	((state == S_MWR) && (r_m_ret != S_NEXT)) ? br_tgt :
	(state == S_DBCC1)                       ? rgo_dbcc_t :
	(state == S_JMP1)                        ? ea_addr :
	// P191: a JSR whose target fetch could not go out at the pop (dispatched
	// from a store's acknowledge, or through S_DECODE) raises it here
	(state == S_JSR1)                        ? ea_addr :
	(state == S_FBCC)  ? pc_i + 32'd2 + (ir[6] ? imm : sxw(imm[15:0])) :
	(state == S_FDBCC) ? pc_i + 32'd4 + sxw(imm[15:0]) :
	(state == S_DECODE)                      ? rgo_decode_t :
	// the retire cycle: the queue head's own target -- the lookahead arm's
	// short Bcc, or dispatch_branch's unconditional transfer.  The early
	// fetch (sgo) takes this same wire, so the seed cone behind epf_ftail
	// has ONE target and the carriers only enable it (build 7: a second
	// issue_ifetch target let the lookahead's flags select the target,
	// -1.575 ns).  The select is the registered head opcode.
	                                           (rd_is_bcc ? rd_bcc_t : bd_t);
// decode_dbcc_brf's callers: S_DBCC1 and S_BCC_EXT (their own target when
// the branch is taken, the lookahead's when it retires instead), the
// decode-time Bcc.B, and the lookahead arm in any other state.
// decode_dbcc_brf's target is go_pc_t_early as well: in every state where
// its carrier can fire the two bodies redirect to the same address (the
// lookahead arm's rd_bcc_t, S_DECODE's Bcc.B, S_DBCC1's and S_BCC_EXT's own
// taken target), and one shared wire keeps the synthesizer from muxing the
// two targets on the carriers -- which put the branch lookahead's flags in
// front of the whole refill-seed cone into epf_ftail (build 4, -2.36 ns).
// The carrier task's simulation check compares against the caller's
// argument, so a state where they differed would be reported.
wire [31:0] dbrf_a_early = go_pc_t_early;
wire  [4:0] alu_fast_fl;
wire        alu_fast_ok;
// Forward final pipeline WB flags before SR's sequential update, just as
// legacy ALU lookahead forwards its producer's flags before retirement.
`ifdef AP040_EXPERIMENTAL_PIPELINE
wire [4:0] pipe_branch_ccr = pipe_retire ? pipe_ccr : sr[4:0];
`else
wire [4:0] pipe_branch_ccr = sr[4:0];
`endif
wire  [4:0] rd_bcc_fl = pipe_drain ? pipe_branch_ccr : (regs_alu_fire && p_flags) ? alu_fast_fl : sr[4:0];
wire legacy_alu_fast_ok = (alu_op == `AP040_ALU_ADD) || (alu_op == `AP040_ALU_SUB) ||
    (alu_op == `AP040_ALU_CMP) || (alu_op == `AP040_ALU_MOVE) || (alu_op == `AP040_ALU_TST) ||
    (alu_op == `AP040_ALU_AND) || (alu_op == `AP040_ALU_OR) || (alu_op == `AP040_ALU_EOR);
`ifdef AP040_EXPERIMENTAL_PIPELINE
wire        rd_bcc_fl_ok = !(regs_alu_fire && p_flags) || (pipe_rf_owner ? legacy_alu_fast_ok : alu_fast_ok);
`else
wire        rd_bcc_fl_ok = !(regs_alu_fire && p_flags) || alu_fast_ok;
`endif
wire        rd_bcc_taken = cond_true_fl(rd_ir[11:8], rd_bcc_fl);
always @* begin
	rd_valid = 0; rd_quick = 0; rd_flags = 1; rd_wbsup = 0; rd_sextw = 0;
	rd_imm = 0; rd_immn = 2'd1; rd_qimm = {28'd0, rd_qval}; rd_shift = 0;
	rd_alu = `AP040_ALU_MOVE;
	rd_size = rd_sz; rd_ssize = rd_sz; rd_dsize = rd_sz;
	rd_sa = {rd_mode[0], rd_ir[2:0]};
	rd_da = {1'b0, rd_ir[11:9]};
	case (rd_ir[15:12])
		4'h0: begin
			// ORI/ANDI/SUBI/ADDI/EORI/CMPI #imm,Dn at lookahead only: the
			// immediate is the word (or two) behind the opcode in the
			// queue and is consumed with it.  In S_DECODE the immediate
			// sits at the head instead and immf_reg already handles it.
			if (!rd_ir[8] && rd_mode == 3'b000 && rd_sz != 2'b11 &&
			    rd_ir[11:9] != 3'b100 && rd_ir[11:9] != 3'b111 &&
			    (state != S_DECODE) &&
			    (epf_count >= ((rd_sz == `AP040_SZ_L) ? 4'd3 : 4'd2))) begin
				rd_valid = 1; rd_imm = 1;
				rd_immn = (rd_sz == `AP040_SZ_L) ? 2'd2 : 2'd1;
				rd_da = {1'b0, rd_ir[2:0]};
				case (rd_ir[11:9])
					3'b000: rd_alu = `AP040_ALU_OR;
					3'b001: rd_alu = `AP040_ALU_AND;
					3'b010: rd_alu = `AP040_ALU_SUB;
					3'b011: rd_alu = `AP040_ALU_ADD;
					3'b101: rd_alu = `AP040_ALU_EOR;
					default: begin rd_alu = `AP040_ALU_CMP; rd_wbsup = 1; end
				endcase
			end
		end
		4'hE: begin
			// register shift/rotate at lookahead only: the count is
			// immediate (SK_IMPL) or in Dn on port A; S_PIPE_REGS retires it
			if (rd_sz != 2'b11 && (state != S_DECODE)) begin
				rd_valid = 1; rd_shift = 1;
				case (rd_ir[4:3])
					2'b00: rd_alu = rd_ir[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1;
					2'b01: rd_alu = rd_ir[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1;
					2'b10: rd_alu = rd_ir[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1;
					default: rd_alu = rd_ir[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1;
				endcase
				rd_da = {1'b0, rd_ir[2:0]};
				if (rd_ir[5]) rd_sa = {1'b0, rd_ir[11:9]};
				else begin
					rd_quick = 1;
					rd_qimm = {26'd0, (rd_ir[11:9] == 3'd0) ? 6'd8 : {3'd0, rd_ir[11:9]}};
				end
			end
		end
		4'h7: begin
			// MOVEQ at lookahead only (S_DECODE keeps its own body)
			if (!rd_ir[8] && (state != S_DECODE)) begin
				rd_valid = 1; rd_quick = 1;
				rd_qimm = {{24{rd_ir[7]}}, rd_ir[7:0]};
				rd_alu = `AP040_ALU_MOVE; rd_size = `AP040_SZ_L;
				rd_ssize = `AP040_SZ_L; rd_dsize = `AP040_SZ_L;
				rd_da = {1'b0, rd_ir[11:9]};
			end
		end
		4'h1, 4'h2, 4'h3: begin
			rd_size = (rd_ir[15:12] == 4'h1) ? `AP040_SZ_B :
			          (rd_ir[15:12] == 4'h2) ? `AP040_SZ_L : `AP040_SZ_W;
			rd_ssize = rd_size; rd_dsize = rd_size;
			rd_da = {rd_ir[6], rd_ir[11:9]};
			if (rd_mode[2:1] == 0 && rd_ir[8:7] == 0 &&
			    !(rd_size == `AP040_SZ_B && (rd_mode[0] || rd_ir[6]))) begin
				rd_valid = 1;
				if (rd_ir[6]) begin
					rd_flags = 0; rd_sextw = (rd_size == `AP040_SZ_W);
					rd_size = `AP040_SZ_L;
				end
			end
		end
		4'h5: begin
			// Size 11 is Scc/DBcc; byte An is illegal, not a quick op.
			if (rd_sz != 2'b11 && rd_mode[2:1] == 0 &&
			    !(rd_sz == `AP040_SZ_B && rd_mode[0])) begin
				rd_valid = 1; rd_quick = 1;
				rd_alu = rd_ir[8] ? `AP040_ALU_SUB : `AP040_ALU_ADD;
				rd_da = {rd_mode[0], rd_ir[2:0]};
				if (rd_mode[0]) begin rd_size = `AP040_SZ_L; rd_flags = 0; end
			end
		end
		4'h8, 4'hC: begin
			// Reverse-direction register encodings are BCD/PACK/EXG,
			// not OR/AND aliases. Size 11 belongs to MUL/DIV.
			rd_alu = rd_ir[14] ? `AP040_ALU_AND : `AP040_ALU_OR;
			if (!rd_ir[8] && rd_sz != 2'b11 && rd_mode == 0) rd_valid = 1;
		end
		4'h9, 4'hD: begin
			rd_alu = rd_ir[14] ? `AP040_ALU_ADD : `AP040_ALU_SUB;
			if (rd_mode[2:1] == 0) begin
				if (rd_sz == 2'b11) begin
					// ADDA/SUBA sign extend word sources, preserve CCR.
					rd_valid = 1; rd_flags = 0; rd_da[3] = 1;
					rd_size = `AP040_SZ_L;
					rd_ssize = rd_ir[8] ? `AP040_SZ_L : `AP040_SZ_W;
					rd_sextw = !rd_ir[8];
				end
				else if (!rd_ir[8]) begin
					rd_valid = !(rd_sz == `AP040_SZ_B && rd_mode[0]);
				end
				else if (!rd_mode[0]) begin
					// Register ADDX/SUBX consume the settled predecessor CCR.
					rd_valid = 1;
					rd_alu = rd_ir[14] ? `AP040_ALU_ADDX : `AP040_ALU_SUBX;
				end
			end
		end
		4'hB: begin
			rd_alu = `AP040_ALU_CMP; rd_wbsup = 1;
			if (rd_sz == 2'b11 && rd_mode[2:1] == 0) begin
				rd_valid = 1; rd_da[3] = 1; rd_size = `AP040_SZ_L;
				rd_ssize = rd_ir[8] ? `AP040_SZ_L : `AP040_SZ_W;
				rd_sextw = !rd_ir[8];
			end
			else if (rd_sz != 2'b11 && !rd_ir[8] && rd_mode[2:1] == 0) begin
				rd_valid = !(rd_sz == `AP040_SZ_B && rd_mode[0]);
			end
			else if (rd_sz != 2'b11 && rd_ir[8] && rd_mode == 0) begin
				rd_valid = 1; rd_alu = `AP040_ALU_EOR; rd_wbsup = 0;
				rd_sa = {1'b0, rd_ir[11:9]}; rd_da = {1'b0, rd_ir[2:0]};
			end
		end
		default: begin end
	endcase
end

task dispatch_reg_decode;
	begin
		alu_op <= rd_alu;
		op_size <= rd_size; p_ssize <= rd_ssize; p_dsize <= rd_dsize;
		p_src <= rd_quick ? SK_IMPL : (rd_imm ? SK_IMM : SK_REG);
		if (rd_quick) src_val <= rd_qimm;
		exec_kind <= rd_shift ? EK_SHIFT : EK_ALU;
		if (rd_shift) sh_rox <= (rd_ir[4:3] == 2'b10);
		if (rd_imm) begin
			// consume the immediate word(s) along with the opcode
			src_val <= rd_immv; imm <= rd_immv; x_ext <= rd_immv;
			epf_pop = 2'd1 + rd_immn;
			pc <= pc + 32'd2 + {29'd0, rd_immn, 1'b0};
		end
		p_dst <= DK_REG; p_sreg <= rd_sa; p_dreg <= rd_da;
		p_flags <= rd_flags; p_wbsup <= rd_wbsup; p_sextw <= rd_sextw;
		rr_a <= rd_sa; rr_b <= rd_da;
		state <= S_PIPE_REGS;
	end
endtask

// Step B: enter the next instruction from the decode record at a producer's
// retire, for every class the record covers (the descriptor's classes
// keep dispatch_reg_decode).  The immediate forms consume their words
// here, as the descriptor's immediate class does.
wire [31:0] n_immv = (n_immn == 2'd2) ? {rd_w1, rd_w2} : {16'd0, rd_w1};
wire        n_words_ok = ((n_next == NX_IMMF_PSTART) || (n_next == NX_IMMREG)) ?
                         (epf_count >= (4'd1 + {2'd0, n_immn})) : 1'b1;
// The pipe start's source-memory paths read port A unforwarded (decode
// selected it a cycle earlier, so any landing write had landed); a record
// applied at a producer's retire enters the pipe start while that write
// lands, so a producer writing the record's base register defers the
// record to S_DECODE (the memory-source lookahead's rule).
wire        n_base_hazard = (regs_alu_fire || shift_fire) && !p_wbsup && (p_dreg == n_rr_a);
// A retire from a system state (SR, USP, MOVEC, MOVES, CINV, PFLUSH, RTE,
// STOP, the exception sequences) may change the A7 bank or an auxiliary
// register on this edge: the next opcode decodes in place after it.
wire        sys_retire = (state == S_HALT) ||
                          (state == S_STOPPED) ||
                          (state == S_EXC0) ||
                          (state == S_EXC1) ||
                          (state == S_EXC2) ||
                          (state == S_EXC3) ||
                          (state == S_EXC4) ||
                          (state == S_EXC5) ||
                          (state == S_EXC6) ||
                          (state == S_EXC_VEC) ||
                          (state == S_EXC_JMP) ||
                          (state == S_RTE_SR) ||
                          (state == S_RTE_PC) ||
                          (state == S_RTE_FMT) ||
                          (state == S_RTE_FIN) ||
                          (state == S_RTE_FIN2) ||
                          (state == S_USP1) ||
                          (state == S_MOVEC1) ||
                          (state == S_MOVEC2) ||
                          (state == S_MOVES1) ||
                          (state == S_MOVES2) ||
                          (state == S_MOVES_WR) ||
                          (state == S_MOVES_RD) ||
                          (state == S_PTEST1) ||
                          (state == S_RESET_HOLD) ||
                          (state == S_SROP) ||
                          (state == S_TRAPCC) ||
                          (state == S_STOP_LD) ||
                          (state == S_PFLUSH1) ||
                          (state == S_PFLUSH2) ||
                          (state == S_PTEST2) ||
                          (state == S_CINV2) ||
                          (state == S_EXC4B) ||
                          (state == S_EPF_FILL) ||
                          (state == S_EPF_GAP) ||
                          (state == S_EPF_READY) ||
                          (state == S_POST_EXC) ||
                          (state == S_EXC0_F2) ||
                          (state == S_EXC0_F3) ||
                          (state == S_EXC0_F4) ||
                          (state == S_POST_EXC_F2) ||
                          (state == S_POST_EXC_F3) ||
                          (state == S_POST_EXC_F4);
wire        n_apply_ok = !rd_valid && !n_inplace && (n_next != NX_NONE) && n_words_ok &&
                        (state != S_DECODE) && !aux_we;
// Forward an address-register ALU result into the following indirect MOVE.
// Actual issue remains gated by apply_record after fetch_next accepts retirement.
wire retire_move_read = state == S_PIPE_REGS && regs_alu_fire && !p_wbsup &&
    p_dreg[3] && n_apply_ok && epf_ready_pc2 &&
    n_p_src_v && n_p_src == SK_MEM && n_p_dst_v && n_p_dst == DK_MEM &&
    n_alu_op_v && n_alu_op == `AP040_ALU_MOVE &&
    n_src_mode_r_v && n_src_mode_r == 3'b010 && n_src_rn_r_v && n_p_ssize_v &&
    p_dreg == {1'b1,n_src_rn_r};
// the descriptor classes dispatch at every ordinary retire as well
// dovi: the descriptor dispatch bounded to the ALU/shift/store producers
// again (step C); the record handover stays at every retire.
wire        n_desc_ok  = rd_valid && (state != S_DECODE) && !aux_we &&
                         (regs_alu_fire || shift_fire || pipe_drain ||
                          ((state == S_MWR) && d_ack && (r_m_ret == S_NEXT)));
// The unconditional transfers whose target the queue already holds --
// BRA.W/.L, BSR.W/.L, JSR and JMP abs.W, abs.L and d16(PC) -- dispatch
// from the retire that pops them straight into their branch state
// (S_BCC_EXT, S_JSR1, S_JMP1), the extension words consumed with the
// opcode as the record's immediate forms are, and when the port is free
// the target fetch goes out in that same cycle, hinted (hint_bd): the
// redirect the branch state raises two to five cycles later then finds
// the stream already at the target and issues nothing, and go_pc_now
// dispatches the resident word at once.  The decode cycle and the
// target's demand fetch overlap the state's own work (the push of a BSR
// or JSR, which the fetch in flight holds off until its acknowledge).
// Every term here is registered queue data, pc, epf_count and state; the
// ALU flags never enter, and the conditional forms keep the lookahead
// arm and S_DECODE.  An odd target skips the fetch and lets the branch
// state raise its address error as before; a fault on the early fetch is
// recorded by the fill engine like any speculative fill's and re-raised
// on demand.  (2026-09-18)
wire        bd_bra    = (rd_ir[15:12] == 4'h6) && (rd_ir[11:9] == 3'b000) &&
                        ((rd_ir[7:0] == 8'h00) || (rd_ir[7:0] == 8'hFF));
// P212: conditional Bcc.W/.L too -- S_BCC_EXT evaluates the condition a
// clock after the retire, on the flags the retiring instruction wrote; no
// early target fetch for them (bd_go), since the branch may fall through
wire        bd_bcc    = (rd_ir[15:12] == 4'h6) && (rd_ir[11:9] != 3'b000) &&
                        ((rd_ir[7:0] == 8'h00) || (rd_ir[7:0] == 8'hFF));
wire        bd_jsrjmp = (rd_ir[15:8] == 8'h4E) && rd_ir[7] &&
                        (rd_ir[5:3] == 3'b111) && (rd_ir[2:0] <= 3'd2);
wire        bd_jmp    = bd_jsrjmp && rd_ir[6];
wire        bd_long   = (bd_bra || bd_bcc) ? (rd_ir[7:0] == 8'hFF) : (rd_ir[2:0] == 3'd1);
wire        bd_abs    = bd_jsrjmp && !rd_ir[1];
wire  [1:0] bd_n      = bd_long ? 2'd2 : 2'd1;
wire [31:0] bd_immv   = bd_long ? {rd_w1, rd_w2} : {16'd0, rd_w1};
wire [31:0] bd_disp   = bd_long ? {rd_w1, rd_w2} : sxw(rd_w1);
wire [31:0] bd_t      = bd_abs ? bd_disp : (pc + 32'd2 + bd_disp);
wire        bd_ok     = (bd_bra || bd_bcc || bd_jsrjmp) &&
                        (epf_count >= (4'd1 + {2'd0, bd_n})) &&
                        (state != S_DECODE) && !aux_we && !sys_retire;
// the early fetch: the port free, an even target that is not the
// fall-through (the stream is there already, and go_pc_now dispatches a
// resident target itself), T1 clear (a traced redirect takes the
// exception path and the fetch would be wasted)
wire [31:0] bd_fall   = pc + 32'd2 + {29'd0, bd_n, 1'b0};
// The fetch's target rides go_pc_t_early (see there), so the retiring
// state must be one whose arm of that wire is the head's target: the
// states with a redirect target of their own that can also retire into
// a pop (a not-taken Bcc.W/FBcc, a DBcc/FDBcc exit) keep the branch
// state's later fetch.  mem_req covers the acknowledge cycle (the
// request is held until it), so the acknowledge itself stays out.
wire        bd_go     = bd_ok && !bd_bcc && !bd_t[0] && (bd_t != bd_fall) &&
                        ifr_avail && !sr[15] &&
                        (state != S_BCC_EXT) && (state != S_DBCC1) &&
                        (state != S_FBCC) && (state != S_FDBCC) &&
                        (state != S_MWR);

task dispatch_branch;
	begin
		epf_pop = 2'd1 + bd_n;
		pc <= pc + 32'd2 + {29'd0, bd_n, 1'b0};
		// as immf_now's inline pop: no speculative fill under a state
		// that is about to push
		epf_issue = 1;
		if (bd_bra || bd_bcc) begin
			br_base <= pc + 32'd2;
			br_long <= bd_long;
			imm     <= bd_immv;
			state   <= S_BCC_EXT;
		end
		else begin
			ea_addr   <= bd_t;
			ea_mode   <= 3'b111;
			ea_rn     <= {1'b0, rd_ir[1:0]};
			ea_pcmode <= rd_ir[1];
			state     <= bd_jmp ? S_JMP1 : S_JSR1;
		end
		if (bd_go) begin
			sgo = 1;
			// synthesis translate_off
			if (bd_t !== go_pc_t_early)
				$display("AP040 dispatch_branch: early target %h differs from go_pc_t_early %h in state %0d", bd_t, go_pc_t_early, state);
			// synthesis translate_on
		end
	end
endtask

// DBcc takes the same dispatch from the pop into S_DBCC1: the displacement
// is consumed with the opcode and Dn is selected on port A, which the
// state reads settled a cycle later exactly as it did after S_DECODE.
// Refused when the retiring arm writes that Dn this cycle (rfw_now: the
// write would land while S_DBCC1 reads the port).  The state's target,
// condition, count, refill dispatch and exit are untouched; the decode
// cycle goes, and so does the speculative fill that cycle used to let
// out in front of the state (immf's inline pop then fell to S_IMMF).
// (2026-09-18)
wire        dd_ok = (rd_ir[15:12] == 4'h5) && (rd_ir[7:3] == 5'b11001) &&
                    (epf_count >= 4'd2) && (state != S_DECODE) &&
                    !aux_we && !sys_retire;

// P190: LEA d16(An),Am takes the same dispatch into S_EA_D16, which retires
// a LEA itself (the forwarded base plus the displacement): the decode cycle
// goes.  Refused when the retiring arm writes the base on this edge.
wire        ld_ok = (rd_ir[15:12] == 4'h4) && (rd_ir[8:6] == 3'b111) &&
                    (rd_ir[5:3] == 3'b101) && (epf_count >= 4'd2) &&
                    (state != S_DECODE) && !aux_we && !sys_retire;

task dispatch_lea;
	begin
		epf_pop = 2'd2;
		pc <= pc + 32'd4;
		epf_issue = 1;
		imm       <= {16'd0, rd_w1};
		rr_a      <= {1'b1, rd_ir[2:0]};
		ea_mode   <= 3'b101;
		ea_rn     <= rd_ir[2:0];
		ea_size   <= `AP040_SZ_L;
		ea_pcmode <= 0;
		r_ea_ret  <= S_LEA1;
		state     <= S_EA_D16;
	end
endtask

// P190: LINK.W An,#d and UNLK An the same way, into S_LINK2 / S_UNLK1 with
// An on port A, as S_DECODE left them.  Both states read An and A7 unforwarded
// (rf_rdata_a, dbg_a7), so a retiring arm that writes either on this edge
// keeps the decode cycle.
wire        lk_ok = (rd_ir[15:3] == 13'b0100_1110_0101_0) && (epf_count >= 4'd2) &&
                    (state != S_DECODE) && !aux_we && !sys_retire;
wire        ul_ok = (rd_ir[15:3] == 13'b0100_1110_0101_1) && (epf_count >= 4'd1) &&
                    (state != S_DECODE) && !aux_we && !sys_retire;

task dispatch_link;
	begin
		epf_pop = 2'd2;
		pc <= pc + 32'd4;
		epf_issue = 1;
		imm     <= {16'd0, rd_w1};
		br_long <= 0;
		rr_a    <= {1'b1, rd_ir[2:0]};
		state   <= S_LINK2;
	end
endtask

task dispatch_unlk;
	begin
		epf_pop = 2'd1;
		pc <= pc + 32'd2;
		epf_issue = 1;
		rr_a    <= {1'b1, rd_ir[2:0]};
		state   <= S_UNLK1;
	end
endtask

// P197: an FPU general op (cpGEN, $F200-$F23F, not an illegal EA) with its
// command word resident goes straight to S_FPU_DEC, which does all of the
// decode's work for it from imm; PEA d16(An) into S_EA_D16, which pushes
// (P180).  PEA is refused when the retiring arm writes An or A7.
wire        fd_ok = (rd_ir[15:6] == 10'b1111_0010_00) &&
                    !((rd_ir[5:3] == 3'b111) && (rd_ir[2:0] > 3'b100)) &&
                    (AP040_HAS_FPU != 0) && (epf_count >= 4'd2) &&
                    (state != S_DECODE) && !aux_we && !sys_retire;
wire        pd_ok = (rd_ir[15:3] == 13'b0100_1000_0110_1) && (epf_count >= 4'd2) &&
                    (state != S_DECODE) && !aux_we && !sys_retire;

task dispatch_fpu;
	begin
		epf_pop = 2'd2;
		pc <= pc + 32'd4;
		epf_issue = 1;
		imm   <= {16'd0, rd_w1};
		rr_a  <= {1'b1, rd_ir[2:0]};   // P223: the EA base, for S_FPU_DEC's inline S_FPU_AN
		state <= S_FPU_DEC;
	end
endtask

task dispatch_pea;
	begin
		epf_pop = 2'd2;
		pc <= pc + 32'd4;
		epf_issue = 1;
		imm       <= {16'd0, rd_w1};
		rr_a      <= {1'b1, rd_ir[2:0]};
		ea_mode   <= 3'b101;
		ea_rn     <= rd_ir[2:0];
		ea_size   <= `AP040_SZ_L;
		ea_pcmode <= 0;
		r_ea_ret  <= S_PEA1;
		state     <= S_EA_D16;
	end
endtask

task dispatch_ret;
	begin
		if (rd_ir[0]) ret_kind <= RK_RTS;
		else begin
			ret_kind <= RK_RTD;
			imm      <= {16'd0, rd_w1};
			epf_pop  = 2'd2;
			pc       <= pc + 32'd4;
		end
		mrd(dbg_a7_wb, `AP040_SZ_L, S_RET2);
		mem_issue;
		mgo = 0;
	end
endtask

task dispatch_dbcc;
	begin
		epf_pop = 2'd2;
		pc <= pc + 32'd4;
		epf_issue = 1;
		br_base <= pc + 32'd2;
		imm     <= {16'd0, rd_w1};
		rr_a    <= {1'b0, rd_ir[2:0]};
		state   <= S_DBCC1;
	end
endtask
// Step D: the record applied in the decode cycle itself.  The immediate
// forms take the body's own inline paths (immf / immf_reg), so the queue
// ownership rules and the deferred S_IMMF case are unchanged; rr_b for the
// immediate-to-register form is set by immf_reg on its inline path only, as
// the body did.
task apply_record_decode;
	begin
		// The record supersedes whatever the reduced body did for this
		// opcode, including an exception it raised on a path the reducer
		// left behind: the body's state write used to be overridden by the
		// writes below, so the deferred exception entry is cancelled here.
		xgo = 0;
		if (n_p_src_v) p_src <= n_p_src;
		if (n_p_dst_v) p_dst <= n_p_dst;
		if (n_p_rmw_v) p_rmw <= n_p_rmw;
		if (n_p_wbsup_v) p_wbsup <= n_p_wbsup;
		if (n_p_flags_v) p_flags <= n_p_flags;
		if (n_p_sextw_v) p_sextw <= n_p_sextw;
		if (n_p_dst_mem_bit_v) p_dst_mem_bit <= n_p_dst_mem_bit;
		if (n_exec_kind_v) exec_kind <= n_exec_kind;
		if (n_op_size_v) op_size <= n_op_size;
		if (n_alu_op_v) alu_op <= n_alu_op;
		if (n_p_dreg_v) p_dreg <= n_p_dreg;
		if (n_p_dsize_v) p_dsize <= n_p_dsize;
		if (n_p_ssize_v) p_ssize <= n_p_ssize;
		if (n_p_sreg_v) p_sreg <= n_p_sreg;
		if (n_dst_rn_r_v) dst_rn_r <= n_dst_rn_r;
		if (n_dst_mode_r_v) dst_mode_r <= n_dst_mode_r;
		if (n_src_rn_r_v) src_rn_r <= n_src_rn_r;
		if (n_src_mode_r_v) src_mode_r <= n_src_mode_r;
		if (n_rr_a_v) rr_a <= n_rr_a;
		if (n_rr_b_v && n_next != NX_IMMREG) rr_b <= n_rr_b;
		if (n_src_val_v) src_val <= n_src_val;
		if (n_sh_rox_v) sh_rox <= n_sh_rox;
		if (n_md_isdiv_v) md_isdiv <= n_md_isdiv;
		if (n_md_sign_v) md_sign <= n_md_sign;
		if (n_lk_cyc_v) lk_cyc <= n_lk_cyc;
		case (n_next)
			NX_PSTART: state <= S_PIPE_START;
			NX_PREGS:  state <= S_PIPE_REGS;
			NX_IMMF_PSTART: immf(n_immn, S_PIPE_START);
			NX_IMMREG: immf_reg(n_immn, n_rr_b);
			default: begin end
		endcase
	end
endtask

task apply_record;
	begin
		if (n_p_src_v) p_src <= n_p_src;
		if (n_p_dst_v) p_dst <= n_p_dst;
		if (n_p_rmw_v) p_rmw <= n_p_rmw;
		if (n_p_wbsup_v) p_wbsup <= n_p_wbsup;
		if (n_p_flags_v) p_flags <= n_p_flags;
		if (n_p_sextw_v) p_sextw <= n_p_sextw;
		if (n_p_dst_mem_bit_v) p_dst_mem_bit <= n_p_dst_mem_bit;
		if (n_exec_kind_v) exec_kind <= n_exec_kind;
		if (n_op_size_v) op_size <= n_op_size;
		if (n_alu_op_v) alu_op <= n_alu_op;
		if (n_p_dreg_v) p_dreg <= n_p_dreg;
		if (n_p_dsize_v) p_dsize <= n_p_dsize;
		if (n_p_ssize_v) p_ssize <= n_p_ssize;
		if (n_p_sreg_v) p_sreg <= n_p_sreg;
		if (n_dst_rn_r_v) dst_rn_r <= n_dst_rn_r;
		if (n_dst_mode_r_v) dst_mode_r <= n_dst_mode_r;
		if (n_src_rn_r_v) src_rn_r <= n_src_rn_r;
		if (n_src_mode_r_v) src_mode_r <= n_src_mode_r;
		if (n_rr_a_v) rr_a <= n_rr_a;
		if (n_rr_b_v) rr_b <= n_rr_b;
		if (n_src_val_v) src_val <= n_src_val;
		if (n_sh_rox_v) sh_rox <= n_sh_rox;
		if (n_md_isdiv_v) md_isdiv <= n_md_isdiv;
		if (n_md_sign_v) md_sign <= n_md_sign;
		if (n_lk_cyc_v) lk_cyc <= n_lk_cyc;
		case (n_next)
			NX_PSTART: begin
                if (retire_move_read) begin
                    x_ext <= imm;
                    mrd(alu_res,n_p_ssize,S_PIPE_SDONE);
                    // Lookahead runs after the common mgo dispatcher.
                    // Consume this new request here; do not leave it queued.
                    mem_issue;
                    mgo = 0;
                    // P179: mem_issue's destination preselect reads the
                    // retiring instruction's registered fields; select the
                    // new MOVE's simple destination base from its record.
                    if (n_dst_mode_r_v && n_dst_rn_r_v &&
                        (n_dst_mode_r == 3'b010 || n_dst_mode_r == 3'b011 ||
                         n_dst_mode_r == 3'b100 || n_dst_mode_r == 3'b101))
                        rr_a <= {1'b1, n_dst_rn_r};
                end
                else if ((retire_store_read || retire_read_read) &&
                         !(rfw_now && (rfw_now_a == {1'b1, rd_ir[2:0]})) &&
                         !((rsr_pd || rsr_pi) && rfw_now)) begin
                    // P196: the source read at the retiring store's acknowledge
                    x_ext <= imm;
                    // P207: the (An)+/-(An) update and its undo record (the
                    // retire cleared both records on this edge already)
                    if (rsr_pi) rfw({1'b1, rd_ir[2:0]}, rsr_base + rsr_adj);
                    else if (rsr_pd) rfw({1'b1, rd_ir[2:0]}, rsr_addr);
                    if (rsr_pd || rsr_pi) begin
                        u0_v <= 1; u0_reg <= {1'b1, rd_ir[2:0]}; u0_old <= rsr_base;
                    end
                    if (n_p_dst_v && n_p_dst == DK_REG && n_p_dreg_v) rr_b <= n_p_dreg;
                    if (rsr_d16) begin
                        epf_pop = 2'd2;
                        pc <= pc + 32'd4;
                    end
                    mrd(rsr_addr, n_p_ssize, S_PIPE_SDONE);
                    mem_issue;
                    mgo = 0;
                    if (n_p_dst_v && n_p_dst == DK_MEM && n_dst_mode_r_v && n_dst_rn_r_v &&
                        (n_dst_mode_r == 3'b010 || n_dst_mode_r == 3'b011 ||
                         n_dst_mode_r == 3'b100 || n_dst_mode_r == 3'b101))
                        rr_a <= {1'b1, n_dst_rn_r};
                end
                else state <= S_PIPE_START;
            end
			NX_PREGS:  state <= S_PIPE_REGS;
			NX_IMMF_PSTART: begin
				imm <= n_immv;
				epf_pop = 2'd1 + n_immn;
				pc <= pc + 32'd2 + {29'd0, n_immn, 1'b0};
				state <= S_PIPE_START;
			end
			NX_IMMREG: begin
				src_val <= n_immv; imm <= n_immv; x_ext <= n_immv;
				epf_pop = 2'd1 + n_immn;
				pc <= pc + 32'd2 + {29'd0, n_immn, 1'b0};
				state <= S_PIPE_REGS;
			end
			default: begin end
		endcase
	end
endtask

// The retire boundary.  Every state arm that completes an instruction
// calls fetch_next; the work itself (the interrupt/trace sampling, the
// pop into decode or the demand fetch) writes some sixty registers, and
// expanding it at each of its eighty call sites gave every one of those
// registers an eighty-way enable tree.  The call now only raises a
// carrier; fetch_next_body runs once, right after the case statement and
// before the lookahead arm and the fill engine that consume what it
// sets (rd_queue_pop, epf_pop, epf_issue, epf_flushed, the seed request).
// Ordering is preserved: no arm writes any of the body's registers after
// its fetch_next call on the same path (audited 2026-09-15), so the body's
// nonblocking writes land exactly where the inlined copies did.
task fetch_next;
	begin
		retire_req = 1;
	end
endtask

task fetch_next_body;
    input [31:0] next_pc, current_pc;
	begin
		fc_ovr_v <= 0;
		lk_cyc <= 0;
		u0_v <= 0;
		u1_v <= 0;
		// An interrupt already sampled at the completing instruction's
		// boundary wins over a simultaneous T1/T0 trace on the 68040
		// (WinUAE do_specialties: the trace converts to a PENDING trace
		// before interrupts are sampled).  This ordering is visible when an
		// odd IRQ vector produces a nested address error: its SR must
		// contain the accepted interrupt mask, not the trace exception's
		// pre-IRQ context.  The trace event itself is NOT lost: it is
		// delivered at the interrupt handler's entry through the S_EXC_JMP
		// texc machinery, like the T0 survival.
		if (irq_pend) begin
			if (tr_t1 || (tr_t0 && t0_force)) begin
				texc_pend <= 1;
				texc_pc <= current_pc;
				tr_t1 <= 0;
			end
			exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
			exc_spc <= next_pc; exc_addr <= 0;
			exc_is_irq <= 1; exc_pass2 <= 0;
			irq_lvl_l <= irq_take_lvl;
			// As with trace, an interrupt recognized at the instruction
			// boundary must see the just-completed register writeback.
			epf_flush;
			state <= S_POST_EXC;
		end
		else if (tr_t1 || (tr_t0 && t0_force)) begin
			tr_t1 <= 0;
			exc_now(`AP040_VEC_TRACE, 4'd2, next_pc, current_pc);
			// Instruction writeback is registered separately.  Do not let
			// S_EXC0 sample Dn/An/A7 on the same edge that commits it.
			state <= S_POST_EXC_F2;
		end
		// The boundary checks above are the only architectural work between a
		// completed instruction and consumption of an already-resident opcode.
		// Retire directly into decode when the queue has that word, instead of
		// spending a separate S_FETCH cycle to perform the same pop/defaults.
		// A flush issued earlier in this clock (CINV/PFLUSH/MOVEC or a
		// self-modifying store) wins even though epf_armed changes through a
		// nonblocking assignment and still appears live here.
		else if (epf_ready_pc && !epf_flushed) begin
			in_exc <= 0;
			epf_pop = 2'd1;
			ir <= epf_data[epf_head];
			if (!movem_mem_op(epf_data[epf_head])) mm_resume <= 0;
			perf_dispatch_toggle <= ~perf_dispatch_toggle;
			pc_i <= next_pc;
			pc <= next_pc + 32'd2;
			tr_t1 <= sr[15];
			tr_t0 <= sr[14];
			flow_t0_pend <= 0;
			t0_force <= t0_special(epf_data[epf_head]);
			p_src <= SK_NONE; p_dst <= DK_NONE;
			p_rmw <= 0; p_wbsup <= 0; p_flags <= 1; p_sextw <= 0;
			p_dst_mem_bit <= 0;
			exec_kind <= EK_ALU;
			state <= S_DECODE;
			rd_queue_pop = 1;
		end
		else begin
			issue_ifetch(next_pc, sr_s);
			pc_i <= next_pc;
			state <= S_FETCH;
		end
	end
endtask

// Shared in-order completion for an ALU operation with a settled register
// destination. Called only after operand/fault qualification at each site.
task retire_operand_alu;
	begin
		if (p_flags) sr[4:0] <= alu_fl;
		if (!p_wbsup) begin
			if (p_dreg[3]) rfw(p_dreg, alu_res);
			else rfw(p_dreg, merge_sz(rf_capture_b, alu_res, op_size));
		end
		fetch_next;
	end
endtask

// True while an effective address is being computed, i.e. while a data
// access is known to be coming.  Keeps speculative instruction fetches
// off the shared memory port just ahead of it.
wire ea_state = (state == S_EA_DISP)  || (state == S_EA_BASE) ||
                (state == S_EA_D16)   || (state == S_EA_EXTW) ||
                // Brief MOVE destination calculation has time to overlap a
                // fetch before its store; delaying it stalls the next operand.
                ((state == S_EA_EXTW2) &&
                 !(p_src == SK_MEM && p_dst == DK_MEM && exec_kind == EK_ALU &&
                   alu_op == `AP040_ALU_MOVE && !p_rmw && !extw[8] &&
                   r_ea_ret == S_PIPE_DEA)) || (state == S_EA_BD) ||
                (state == S_EA_MIND)  || (state == S_EA_OD)   ||
                (state == S_EA_ABS)   || (state == S_PIPE_SRD) ||
                (state == S_PIPE_DEA);

wire [31:0] exc_fsize = (exc_fmt == 4'd2 || exc_fmt == 4'd3) ? 32'd12 :
                        (exc_fmt == 4'd4) ? 32'd16 : 32'd8;

task go_illegal;
	begin
		exc(`AP040_VEC_ILLEGAL, 4'd0, pc_i, 32'd0);
	end
endtask

task go_priv;
	begin
		exc(`AP040_VEC_PRIV, 4'd0, pc_i, 32'd0);
	end
endtask

// A taken DBcc is the hot branch-refill consumer: the focused loop accounts
// for every direct refill dispatch from S_DBCC1.  When its complete target
// window is already resident, let the ordinary refill path seed the queue and
// consume its first opcode here.  Keeping this out of generic go_pc avoids
// widening every redirect path with the branch-buffer read mux, while reusing
// issue_ifetch avoids a second set of wide branch-buffer-to-queue writers.
// The refill-buffer dispatch is requested through a carrier and performed
// once by decode_dbcc_brf_now after the lookahead arm (its last caller)
// and before the fill engine, which must see the queue arm and the port
// claim it makes, on the state-selected target dbrf_a_early (the body's
// refill-word mux, seed loop and issue_ifetch must start from settled
// data: the enable arrives through the acknowledge).  It was expanded at
// seven sites (S_DBCC1, the five finish_bcc callers, the lookahead arm).
task decode_dbcc_brf;
	input [31:0] a;
	begin
		dgo = 1;
		// synthesis translate_off
		if (a !== dbrf_a_early)
			$display("AP040 decode_dbcc_brf: early target %h differs from the caller's %h in state %0d", dbrf_a_early, a, state);
		// synthesis translate_on
	end
endtask

task decode_dbcc_brf_now;
	input [31:0] a;
	reg  [15:0] fw;
	begin
		fw = a[1] ? brf_data[a[5:2]][15:0]
		          : brf_data[a[5:2]][31:16];
		issue_ifetch(a, sr_s);
		epf_head  <= 3'd1;
		epf_count <= brf_seed_n - 4'd1;
		epf_next  <= a + 32'd2;
		epf_issue = 1;

		in_exc <= 0;
		ir <= fw;
		if (!movem_mem_op(fw)) mm_resume <= 0;
		perf_dispatch_toggle <= ~perf_dispatch_toggle;
		pc_i <= a;
		pc <= a + 32'd2;
		tr_t1 <= sr[15];
		tr_t0 <= sr[14];
		flow_t0_pend <= 0;
		t0_force <= t0_special(fw);
		p_src <= SK_NONE; p_dst <= DK_NONE;
		p_rmw <= 0; p_wbsup <= 0; p_flags <= 1; p_sextw <= 0;
		p_dst_mem_bit <= 0;
		exec_kind <= EK_ALU;
		fc_ovr_v <= 0;
		state <= S_DECODE;
        // A resident register MOVE target can enter execution with settled
        // selectors just like the ordinary retirement lookahead decoder.
        if (fw[15:14] == 0 && fw[13:12] != 0 && fw[5:4] == 0 && fw[8:7] == 0 &&
            !(fw[13:12] == 1 && (fw[3] || fw[6]))) begin
            alu_op <= `AP040_ALU_MOVE;
            op_size <= fw[6] ? `AP040_SZ_L : fw[13:12] == 1 ? `AP040_SZ_B :
                       fw[13:12] == 2 ? `AP040_SZ_L : `AP040_SZ_W;
            p_ssize <= fw[13:12] == 1 ? `AP040_SZ_B : fw[13:12] == 2 ? `AP040_SZ_L : `AP040_SZ_W;
            p_dsize <= fw[13:12] == 1 ? `AP040_SZ_B : fw[13:12] == 2 ? `AP040_SZ_L : `AP040_SZ_W;
            p_src <= SK_REG; p_dst <= DK_REG;
            p_sreg <= {fw[3],fw[2:0]}; p_dreg <= {fw[6],fw[11:9]};
            rr_a <= {fw[3],fw[2:0]}; rr_b <= {fw[6],fw[11:9]};
            p_flags <= !fw[6]; p_sextw <= fw[6] && fw[13:12] == 3;
            state <= S_PIPE_REGS;
        end
`ifndef AP040_DISABLE_REFILL_LOAD_DECODE
        // The target word is already resident and validated by the refill
        // buffer. Decode ordinary MOVE from a simple An-based source while
        // installing it, so S_PIPE_START receives settled RF read addresses
        // without an intervening S_DECODE cycle. No access issues here:
        // S_PIPE_START retains memory arbitration, An undo and fault handling.
        // Include MOVEA.W/L with full-width, flag-preserving retirement.
        // Extension/indexed modes outside d16(An) stay on decode.
        if (fw[15:14] == 2'b00 && fw[13:12] != 2'b00 &&
            fw[8:7] == 2'b00 && !(fw[6] && fw[13:12] == 2'b01) &&
            (fw[5:3] == 3'b010 || fw[5:3] == 3'b011 ||
             fw[5:3] == 3'b100 || fw[5:3] == 3'b101)) begin
            alu_op <= `AP040_ALU_MOVE;
            op_size <= fw[6] ? `AP040_SZ_L : fw[13:12] == 2'b01 ? `AP040_SZ_B :
                       fw[13:12] == 2'b10 ? `AP040_SZ_L : `AP040_SZ_W;
            p_ssize <= fw[13:12] == 2'b01 ? `AP040_SZ_B :
                       fw[13:12] == 2'b10 ? `AP040_SZ_L : `AP040_SZ_W;
            p_dsize <= fw[13:12] == 2'b01 ? `AP040_SZ_B :
                       fw[13:12] == 2'b10 ? `AP040_SZ_L : `AP040_SZ_W;
            p_src <= SK_MEM; p_dst <= DK_REG;
            p_dreg <= {fw[6], fw[11:9]};
            p_flags <= !fw[6];
            p_sextw <= fw[6] && fw[13:12] == 2'b11;
            src_mode_r <= fw[5:3]; src_rn_r <= fw[2:0];
            rr_a <= {1'b1, fw[2:0]}; rr_b <= {fw[6], fw[11:9]};
            state <= S_PIPE_START;
        end
        // Decode a resident memory-to-memory MOVE target without consuming
        // either operand. The normal operand path retains address updates,
        // source-fault priority and destination issue ordering.
        if (fw[15:14] == 2'b00 && fw[13:12] != 2'b00 &&
            (fw[8:6] == 3'b010 || fw[8:6] == 3'b011 || fw[8:6] == 3'b100) &&
            (fw[5:3] == 3'b010 || fw[5:3] == 3'b011 ||
             fw[5:3] == 3'b100 || fw[5:3] == 3'b101)) begin
            alu_op <= `AP040_ALU_MOVE;
            op_size <= fw[13:12] == 2'b01 ? `AP040_SZ_B :
                       fw[13:12] == 2'b10 ? `AP040_SZ_L : `AP040_SZ_W;
            p_ssize <= fw[13:12] == 2'b01 ? `AP040_SZ_B :
                       fw[13:12] == 2'b10 ? `AP040_SZ_L : `AP040_SZ_W;
            p_dsize <= fw[13:12] == 2'b01 ? `AP040_SZ_B :
                       fw[13:12] == 2'b10 ? `AP040_SZ_L : `AP040_SZ_W;
            p_src <= SK_MEM; p_dst <= DK_MEM;
            src_mode_r <= fw[5:3]; src_rn_r <= fw[2:0];
            dst_mode_r <= fw[8:6]; dst_rn_r <= fw[11:9];
            rr_a <= {1'b1,fw[2:0]};
            state <= S_PIPE_START;
        end
`endif
	end
endtask

// jump to a control flow target with odd address check
// Flow redirect.  go_pc is called from a dozen sites and expands the
// trace/interrupt sampling and issue_ifetch (the refill-buffer seed
// compare and the queue re-arm) at each; the call now only raises the
// carrier and go_pc_now runs once after the case statement, before the
// exception-entry arm its trace/address-error cases feed, on the
// state-selected target go_pc_t_early (see there for why not the argument).
task go_pc;
	input [31:0] t;
	begin
		pgo = 1;
		// synthesis translate_off
		if (t !== go_pc_t_early)
			$display("AP040 go_pc: early target %h differs from the caller's %h in state %0d", go_pc_t_early, t, state);
		// synthesis translate_on
	end
endtask

task go_pc_now;
	input [31:0] t;
	begin
		// The format-$2 address field contains the referenced address with A0
		// cleared, not the raw odd target.
		if (t[0]) exc(`AP040_VEC_ADDRERR, 4'd2, pc_i, {t[31:1], 1'b0});
		else if (tr_t1) begin
			tr_t1 <= 0;
			tr_t0 <= 0;
			pc <= t;
			if (irq_pend) begin
				// WinUAE do_specialties: the completing instruction's
				// trace converts to a PENDING trace and the interrupt is
				// sampled after that conversion, so the interrupt
				// exception processes first and the trace is delivered at
				// its handler entry (the S_EXC_JMP texc machinery, same
				// as the T0 survival), with the traced instruction in the
				// format-$2 address field.  The trace event is never
				// lost and never precedes the interrupt.
				texc_pend <= 1;
				texc_pc <= pc_i;
				exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
				exc_spc <= t; exc_addr <= 0;
				exc_is_irq <= 1; exc_pass2 <= 0;
				irq_lvl_l <= irq_take_lvl;
				epf_flush;
				state <= S_POST_EXC;
			end
			else exc(`AP040_VEC_TRACE, 4'd2, t, pc_i);
		end
		else if (tr_t0) begin
			// The 040 resolves a T0 change-of-flow trace only after the
			// target word has entered the pipeline.  An immediately decoded
			// ILLEGAL at the target wins and cancels this trace; a normal
			// target is not executed before vector 9 is taken.
			tr_t0 <= 0;
			flow_t0_pend <= 1;
			flow_t0_oldpc <= pc_i;
			pc <= t;
			issue_ifetch(t, sr_s);
			pc_i <= t;
			state <= S_FETCH;
		end
		else begin
			pc <= t;
			fc_ovr_v <= 0;
			if (irq_pend) begin
				exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
				exc_spc <= t; exc_addr <= 0;
				exc_is_irq <= 1; exc_pass2 <= 0;
				irq_lvl_l <= irq_take_lvl;
				// BSR/JSR and taken DBcc can commit A7/Dn on the
				// same edge that redirects here.  Let that registered
				// writeback become visible before S_EXC0 snapshots it.
				epf_flush;
				state <= S_POST_EXC;
			end
			// The stream already runs at the target with its first word
			// resident (dispatch_branch put the fetch out at the pop, or
			// the target is the word behind the branch): pop it now, as
			// S_FETCH would a cycle later.  issue_ifetch would find the
			// stream and issue nothing.  (2026-09-18)
			else if (epf_armed && (epf_next == t) && (epf_super == sr_s) &&
			         (epf_count != 4'd0) && !epf_flushed) begin
				in_exc <= 0;
				epf_pop = 2'd1;
				ir <= epf_data[epf_head];
				if (!movem_mem_op(epf_data[epf_head])) mm_resume <= 0;
				perf_dispatch_toggle <= ~perf_dispatch_toggle;
				pc_i <= t;
				pc <= t + 32'd2;
				tr_t1 <= sr[15];
				tr_t0 <= sr[14];
				flow_t0_pend <= 0;
				t0_force <= t0_special(epf_data[epf_head]);
				p_src <= SK_NONE; p_dst <= DK_NONE;
				p_rmw <= 0; p_wbsup <= 0; p_flags <= 1; p_sextw <= 0;
				p_dst_mem_bit <= 0;
				exec_kind <= EK_ALU;
				state <= S_DECODE;
			end
			else begin
				issue_ifetch(t, sr_s);
				pc_i <= t;
				state <= S_FETCH;
			end
		end
	end
endtask

// Bcc calculates and validates its target even when the condition is false.
// The 68040 therefore takes an address error for an odd target on a
// not-taken conditional branch as well as on a taken one.
function brf_refill_hit;
	input [31:0] a;
	begin
		brf_refill_hit = (brf_tag == a[31:6]) && (brf_super == sr_s) &&
		                 (brf_run[a[5:1]] >= 4'd4);
	end
endfunction

task finish_bcc;
	input [31:0] t;
	input        taken;
	begin
		if (t[0]) exc(`AP040_VEC_ADDRERR, 4'd2, pc_i, {t[31:1], 1'b0});
		// A taken branch whose target window sits in the refill buffer
		// dispatches its target directly, as DBcc does, instead of
		// spending S_FETCH on a word the buffer already holds.  Only the
		// ordinary idle-bus case; traces and interrupts keep go_pc's order.
		// An outstanding speculative queue fetch does not block this: the
		// buffer supplies the target words and issue_ifetch marks that
		// fetch killed, so its later acknowledge appends nothing.  Only the
		// acknowledge cycle itself is excluded (its append shares the ring).
		else if (taken && !tr_t1 && !tr_t0 && !irq_pend &&
		         brf_refill_hit(t) && !ifr_ack &&
		         (!epf_armed || epf_next != t || epf_super != sr_s))
			decode_dbcc_brf(t);
		else if (taken) go_pc(t);
		else fetch_next;
	end
endtask

task pipe_go;
	begin
		state <= S_PIPE_START;
	end
endtask

// Decode has already identified both register operands for an ordinary
// dyadic ALU instruction.  Select both asynchronous ports now so the next
// state can capture/retire directly instead of spending S_PIPE_START merely
// copying the same addresses out of p_sreg/p_dreg.
task pipe_go_regpair;
	input [3:0] sa;
	input [3:0] da;
	begin
		rr_a <= sa;
		rr_b <= da;
		state <= S_PIPE_REGS;
	end
endtask

task pipe_go_regdst;
	input [3:0] da;
	begin
		rr_b <= da;
		state <= S_PIPE_REGS;
	end
endtask

wire        hint_data = (state == S_MRD) && !m_issued;
// Store hints: the address a store presents next cycle, from the state
// that issues it (dst_addr from S_EXEC, A7 - 4 from the five push
// states, mm_addr from the MOVEM loop, m_addr_r from S_MWR's own issue),
// so the MMU's write-side verdict is registered when the request arrives
// and the cache acknowledges a posted store in its request cycle
// (fast_store).  A store that was not hinted takes the registered
// acknowledge as before.  (2026-09-19)
wire hint_st_reg_move = reg_move_store_prepare &&
    rr_a == {1'b1, dst_rn_r} && !base_landing && !aux_we &&
    ((dst_mode_r == 3'b101 && epf_ready_pc) || dst_mode_r == 3'b010 ||
     dst_mode_r == 3'b011 || dst_mode_r == 3'b100);
wire        hint_st_exec  = (state == S_EXEC) && (p_dst == DK_MEM);
wire        hint_st_move_ea = (state == S_PIPE_DEA) && !p_rmw &&
    exec_kind == EK_ALU && p_dst == DK_MEM && !p_wbsup &&
    ((alu_op == `AP040_ALU_MOVE && (p_src == SK_MEM || p_src == SK_IMM)) ||
     alu_op == `AP040_ALU_CLR);
wire        hint_st_pushf = ((state == S_DECODE) && (ir[15:8] == 8'h61) &&
                             (ir[7:0] != 8'h00) && (ir[7:0] != 8'hFF)) ||
                            ((state == S_BCC_EXT) && (ir[11:8] == 4'h1)) ||
                            (state == S_JSR1) || pea_d16_push;
wire        hint_st_push  = (state == S_PEA1) || (state == S_LINK2);
wire        hint_st_movem = (state == S_MOVEM_LOOP) && !mm_dir;
wire        hint_st_mwr   = (state == S_MWR) && !m_issued;
// FPU result data and transfer index are registered before these states.
// Hints do not issue stores; ordinary mwr ordering/fault handling still owns them.
wire hint_st_fpu = state == S_FPU_WR &&
    !((fp_nb <= 4'd4 && fp_n != 4'd0) ||
      (fp_nb == 4'd8 && fp_n == 4'd2) ||
      (fp_nb == 4'd12 && fp_n == 4'd3));
wire hint_st_fmovem = state == S_FPU_MVM2 && fp_st && fp_n != 4'd3;
// P225: the FPU store's first longword from S_FPU_GO in the done clock
wire hint_st_fpgo = state == S_FPU_GO && fpu_done && fp_st && (d_mode != 3'b000) && (fp_nb >= 4'd4);
wire [31:0] hint_st_fpu_addr = t_a +
    ((hint_st_fpu && fp_nb <= 4'd2) ? 32'd0 : {26'd0, fp_n, 2'b00});
wire        hint_store    = hint_st_reg_move || hint_st_move_ea || hint_st_exec || hint_st_pushf || hint_st_push ||
                            hint_st_movem || hint_st_mwr || hint_st_fpu || hint_st_fmovem || hint_st_fpgo;
wire [31:0] hint_store_addr = hint_st_reg_move ? hint_dst_addr :
                              hint_st_move_ea ? ea_addr :
                              hint_st_exec  ? dst_addr :
                              hint_st_mwr   ? m_addr_r :
                              (hint_st_fpu || hint_st_fmovem) ? hint_st_fpu_addr :
                              hint_st_fpgo ? t_a :
                              // P189: the predecrement form stores at mm_addr - size
                              hint_st_movem ? (mm_predec ? mm_addr - ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2)
                                                         : mm_addr) :
                              hint_st_pushf ? (dbg_a7_wb - 32'd4) :
                                              (dbg_a7 - 32'd4);
wire        hint_bcc  = (state == S_DECODE) && (ir[15:12] == 4'h6) &&
                        (ir[11:8] != 4'h1) &&
                        (ir[7:0] != 8'h00) && (ir[7:0] != 8'hFF);
// The operand pipeline's own reads (the mrd sites in S_PIPE_START,
// S_PIPE_SRD and S_PIPE_DEA) issue at the end of those states from the
// same values shown here, so the hint precedes each by exactly one cycle.
wire        hint_pipe_src = (state == S_PIPE_START) && (p_src == SK_MEM);
// The registered part of ea_operand_start's inline qualification.  The
// in-cycle terms (mem_ack, epf_flushed) stay out: the hint is
// speculative, and through the address bus they would close a
// combinational loop (acknowledge -> hint -> translation -> acknowledge,
// 784 nodes in the fitter, 2026-09-14).  A hint that guesses wrong only
// costs the read its idle-read match.
// A register write landing this cycle disqualifies the hint and the
// direct read only when it is the base register's (port A would show the
// old value); a write to any other register, the usual case after a
// lookahead or record dispatch, leaves them alone.
wire        hint_ext_ok   = epf_ready_pc && !base_landing && !aux_we;
wire        hint_pipe_dst = (state == S_PIPE_START) && (p_src != SK_MEM) &&
                            (p_dst == DK_MEM) && p_rmw &&
                            (rr_a == {1'b1, dst_rn_r}) &&
                            (((dst_mode_r == 3'b101) && hint_ext_ok) ||
                             (dst_mode_r == 3'b010) || (dst_mode_r == 3'b011) ||
                             (dst_mode_r == 3'b100));
wire        hint_pipe = hint_pipe_src || hint_pipe_dst;
wire [31:0] hint_d16_addr = rf_rdata_a + sxw(epf_data[epf_head]);
wire [31:0] hint_dst_addr = (dst_mode_r == 3'b101) ? hint_d16_addr :
                            (dst_mode_r == 3'b100) ? rf_rdata_a - an_adj(dst_rn_r, p_dsize) :
                            rf_rdata_a;
// P209: the simple source modes hint from the forwarded port (the address
// S_PIPE_START's read uses): after ADDQ/SUBQ/LEA to the base register its
// write lands in this very clock, and port A alone showed the old value
// (127k of Whetstone's reads lost their one-clock hit to it)
wire [31:0] hint_pipe_addr = hint_pipe_dst ? hint_dst_addr :
                             (src_mode_r == 3'b100)
                           ? rf_capture_a - an_adj(src_rn_r, p_ssize) :
                             ((src_mode_r == 3'b101) && hint_ext_ok &&
                              (rr_a == {1'b1, src_rn_r}))
                           ? hint_d16_addr : rf_capture_a;
// The brief extension and base are complete; source faults still return
// through the ordinary read machinery before any result can retire.
wire hint_indexed_read = state == S_EA_EXTW2 && !extw[8] &&
    r_ea_ret == S_PIPE_SRD && p_src == SK_MEM &&
    (p_dst == DK_REG || (p_dst == DK_MEM && exec_kind == EK_ALU &&
                        alu_op == `AP040_ALU_MOVE && !p_rmw));
wire hint_displacement_read = state == S_EA_D16 &&
    r_ea_ret == S_PIPE_SRD && p_src == SK_MEM &&
    (p_dst == DK_REG || (p_dst == DK_MEM && exec_kind == EK_ALU &&
                        alu_op == `AP040_ALU_MOVE && !p_rmw));
wire hint_early_read = hint_indexed_read || hint_displacement_read;
// Isolated experiment: hint exactly the next ordinary FPU operand read.
wire hint_fpu_operand = state == S_FPU_RD &&
    !((fp_nb <= 4'd4 && fp_n != 0) ||
      (fp_nb == 4'd8 && fp_n == 2) || (fp_nb == 4'd12 && fp_n == 3));
wire hint_fpu_movem = state == S_FPU_MVM2 && !fp_st && fp_n != 3;
wire hint_fpu_mvm0  = state == S_FPU_MVM && !fp_st && fp_list != 8'd0;   // P227
wire hint_fpu_read = hint_fpu_operand || hint_fpu_movem || hint_fpu_mvm0;
wire [31:0] hint_fpu_addr = t_a +
    (((hint_fpu_operand && fp_nb <= 2) || hint_fpu_mvm0) ? 32'd0 : {26'd0,fp_n,2'b00});

wire [31:0] hint_displacement_addr = (ea_pcmode ? ea_pcb : rf_rdata_a) + sxw(imm[15:0]);
wire [31:0] hint_indexed_offset = (extw[11] ? rf_rdata_b : sxw(rf_rdata_b[15:0])) << extw[10:9];
wire [31:0] hint_indexed_addr = ea_base_v + hint_indexed_offset + sxb(extw[7:0]);
wire        hint_ea   = (state == S_PIPE_SRD) ||
                        ((state == S_PIPE_DEA) && p_rmw);
// Redirect states present their target on the hint bus one cycle before
// go_pc's issue_ifetch puts it on the request bus, so the demand fetch at
// the target finds the cache's idle read already on it: a hinted two-clock
// instruction read instead of a three-clock lookup (Alan's measurement:
// three cycles from the redirect edge to the target's first decode against
// four).  Every source is a register or one adder over registers, and each
// is exactly the value the state hands to go_pc/finish_bcc.  A state that
// does not redirect after all (a not-taken Bcc.W, a DBcc exit, RTR's CCR
// word in S_RET2) has hinted a useless address, which costs nothing: the
// only request a redirect-cycle hint can serve is the redirect's own, and
// the fill engine's next fetch is hinted by epf_ftail in its own cycle.
// Bcc.B is hinted from S_DECODE by hint_bcc above.
wire        hint_redir = (state == S_BCC_EXT) || (state == S_DBCC1) ||
                         (state == S_BSR_PUSH) || (state == S_JSR2) ||
                         (state == S_JMP1) || (state == S_RET2) ||
                         (state == S_RET3);
wire [31:0] hint_redir_addr =
    (state == S_BCC_EXT) ? br_base + (br_long ? imm : sxw(imm[15:0])) :
    (state == S_DBCC1)   ? br_base + sxw(imm[15:0]) :
    ((state == S_BSR_PUSH) || (state == S_JSR2)) ? br_tgt :
    (state == S_JMP1)    ? ea_addr : m_val;
// The reads mem_issue now claims the port for in place from their own
// states (RTS's pop from S_DECODE, RTD/RTR's from S_RET1, UNLK's (An) read
// from S_UNLK1, MOVEM's loads from S_MOVEM_LOOP) are hinted in the same
// cycle as data reads, with the address expression their mrd uses, so the
// one-clock hit serves them: the request goes out at the end of the state
// and is acknowledged in the cycle it is presented.  When the in-place
// issue is refused (a queue fetch in flight, MMIO, misaligned) the read
// goes out from S_MRD, which hints itself (hint_data).
wire        hint_pop = ((state == S_DECODE) && (ir == 16'h4E75)) ||
                       (state == S_RET1) || (state == S_UNLK1) ||
                       ((state == S_MOVEM_LOOP) && mm_dir && !mm_predec);
wire [31:0] hint_pop_addr = (state == S_DECODE)     ? dbg_a7_wb :
                            (state == S_RET1)       ? dbg_a7 :
                            (state == S_MOVEM_LOOP) ? mm_addr : rf_rdata_a;
// While a producer retires with a short Bcc at the queue head, hint its
// target: if the lookahead arm takes the branch this cycle (go_pc from the
// arm), the demand fetch it issues is the hint's; if the branch is not
// taken, or the arm does not fire, a fill issued this cycle has merely lost
// its idle-read match.  The guess is deliberately flag-free so the ALU's
// flags never enter the hint path.
wire        hint_ftb = rd_is_bcc && (pipe_drain || state == S_PIPE_REGS || state == S_EXEC ||
                                     state == S_PIPE_SDONE || state == S_MRD || state == S_MWR);
// While an unconditional transfer with a resident target sits at the
// queue head and the port is free, hint its target: if this cycle's
// retire pops it, dispatch_branch's fetch is the hint's (a hinted
// two-clock read); if not, a fill issued this cycle has merely lost its
// idle-read match.  The acknowledge stays out of the select (it would
// put the acknowledge in front of the hint's translation).
// Present the ordered pipeline memory offer as a data hint. The request
// depends on EX/WB state, never on memory acknowledgement or the hint itself.
`ifdef AP040_EXPERIMENTAL_PIPELINE
wire hint_p2 = pipe_load_launch;
wire [31:0] hint_p2_addr = pipe_load_addr;
`else
wire hint_p2 = 1'b0;
wire [31:0] hint_p2_addr = 32'd0;
`endif
wire        hint_bd  = bd_ok && ifr_avail && !sr[15];
wire [31:0] hint_addr = hint_data  ? m_addr_r :
                        retire_move_read ? alu_res :
                        hint_p2    ? hint_p2_addr :
                        hint_store ? hint_store_addr :
                        hint_pipe  ? hint_pipe_addr :
                        hint_displacement_read ? hint_displacement_addr :
                        hint_indexed_read ? hint_indexed_addr :
                        hint_fpu_read ? hint_fpu_addr :
                        hint_ea    ? ea_addr :
                        hint_pop   ? hint_pop_addr :
                        hint_pop_addr;   // the instruction targets ride mem_ihint_addr (P175)
// The request bus carries only registered state.  The hint rides its own
// bus, which only RAM address inputs and the MMU's hint copy listen to,
// so the address arithmetic behind it never enters a request-cycle path
// (a combinational cache acknowledge on the shared bus failed timing by
// 6.6 ns through rr_a -> register file -> hint adder, 2026-09-14).
// While a request is held (its translation walking) the hint bus repeats
// it, so the cache's idle read stays on the request.
assign mem_addr  = mem_addr_q;
assign mem_instr = mem_instr_q;
// A pending data request is hinted ahead of an outstanding fetch, and a
// data hint ahead of the fetch's own repeat (P171); the fetch keeps its
// repeat only when nothing on the data side wants the bus.
wire        data_hint_any = retire_move_read || hint_data || hint_store || hint_pipe || hint_ea || hint_early_read || hint_fpu_read || hint_pop || hint_p2;
// Fetch queue supply policy (P171).  A fetch is only needed once per
// 16-byte line (the cache offers the rest of the line after each hit),
// and its issue->offer latency is 2-3 clocks.  The floor (P171_FILL_TH,
// words) is the starvation trigger: below it the fetch goes out even if
// the data side wants the port.  Above it, with P171_FILL_IDLE, the
// fetch goes out only into an idle port slot -- no data request
// outstanding and none hinted for the next cycle -- up to
// P171_FILL_IDLE_TH words, so long instructions do not run the queue
// dry while short loops keep the port for their operands.
// Measured (Permute(7) lat 0 / Whetstone, against P170 1,263,600 / 25,671,327):
//   floor 2, no idle fill      1,174,917 / 25,742,999
//   floor 4, no idle fill      1,201,412 / 25,539,756
//   floor 2 + idle fill to 6   1,191,990 / 25,462,081
//   floor 3 + idle fill to 6   1,191,988 / 25,381,095   <- default
//   floor 4 + idle fill to 6   1,202,064 / 25,472,006
// An idle cap above 6 overflows the eight-entry ring with a longword
// fetch (cap 8 corrupts Whetstone).  Whether the hinted request counts
// as "wanted" made no difference (the data side wins the arbiter on
// arrival anyway); it is kept because it is free.
`ifndef P171_FILL_TH
`define P171_FILL_TH 4
`endif
`ifndef P171_FILL_IDLE_TH
`define P171_FILL_IDLE_TH 6
`endif
wire        epf_port_wanted = mem_req || data_hint_any;
wire        epf_fill_floor  = epf_ftail[1] ? (epf_count <= (`P171_FILL_TH + 4'd1)) : (epf_count <= `P171_FILL_TH);
`ifdef P171_FILL_FLAT
wire        epf_fill_idle   = 1'b0;
`else
wire        epf_fill_idle   = !epf_port_wanted &&
                              (epf_ftail[1] ? (epf_count <= (`P171_FILL_IDLE_TH + 4'd1)) : (epf_count <= `P171_FILL_IDLE_TH));
`endif
// P175b: the clock after an instruction acknowledge is the line offer's
// clock (the cache offers the acknowledged line then, and the core takes
// it only while no fetch is outstanding); a fill issued in it refuses the
// offer and fetches the same words a longword at a time.
wire        epf_fill_ok     = (epf_fill_floor || epf_fill_idle) && !i_ack_d;
// The presented request repeats on the hint bus until its acknowledge
// (the MMU translates the hint through the same copy it refills from the
// request); in the acknowledge cycle the hint moves on to whatever waits
// behind it, so the next request meets a translated hint.
// P182: the MOVE destination store's hint in the source read's predicted
// acknowledge clock.  A MOVE source read issued in place is acknowledged in
// its first S_MRD clock when it hits; move_store_read_handoff then issues the
// store in place on that edge, and without a hint the cache takes its
// two-clock registered path for it.  So in that first clock the hint bus
// shows the store (a register-derived flag, never the live acknowledge,
// selects it); if the read does not complete there it re-hints itself from
// the next clock and finishes through the registered lookup.
reg mrd_fresh;
always @(posedge clk) mrd_fresh <= (state != S_MRD) || (m_issued && d_ack);
// P192: and no fetch holds the port -- the arbiter serves a presented fetch
// first, so the read cannot be acknowledged this clock (the P175 merge had
// dropped P182's fetch term along with the fetch's use of this bus).
// P204: or the read took the registered path and its acknowledge is ack_r's
// this clock (known at the clock's start; the cache admits nothing then)
wire hint_move_store = move_store_read_ready && m_issued &&
                       ((mrd_fresh && mrd_hinted && mem_fast_ready) || (!mrd_fresh && mem_ack_q)) && !ifr_pres;
// P196: the read-after-store handoff.  A store that retires its instruction
// (S_MWR, r_m_ret == S_NEXT), hinted when it issued and meeting a cache that
// can post it now, is predicted to be acknowledged in its first clock; in
// that clock the data hint shows the next instruction's source read -- a
// record-dispatched op with an (An) or d16(An) source, its base from regfile
// port F (the queue head's EA register, a write landing now forwarded) --
// and if the acknowledge does come, apply_record issues that read in place
// on the same edge instead of passing through S_PIPE_START.  The cache
// looks the store's tag up at its own row meanwhile (c_hint_away) and may
// answer the read during the store's C_PASS (fast_accept_pp).
reg mwr_fresh;
always @(posedge clk) mwr_fresh <= (state != S_MWR) || (m_issued && d_ack);
wire [31:0] rsr_base = (rf_we && rf_waddr == {1'b1, rd_ir[2:0]}) ? rf_wdata : rsr_base_rf;
wire        rsr_d16  = (n_src_mode_r == 3'b101);
// P207: (An)+ and -(An) sources too; the handoff writes the address
// register and its undo record as S_PIPE_START would
// the record's decoded mode, not ir[5:3]: CMPM (Ay)+ and ADDX/SUBX/ABCD/SBCD
// -(Ay) keep their memory mode elsewhere in the opcode (the self-test's
// CMPM after a store lost its A0 increment when this read ir[5:3])
wire        rsr_pd   = (n_src_mode_r == 3'b100);
wire        rsr_pi   = (n_src_mode_r == 3'b011);
wire [31:0] rsr_adj  = an_adj(rd_ir[2:0], n_p_ssize);
wire [31:0] rsr_addr = rsr_d16 ? (rsr_base + sxw(rd_w1)) : rsr_pd ? (rsr_base - rsr_adj) : rsr_base;
wire        rsr_head = n_apply_ok && (n_next == NX_PSTART) && n_p_src_v && (n_p_src == SK_MEM) &&
                       n_src_mode_r_v && n_p_ssize_v &&
                       // port F reads {1, ir[2:0]}: the record's source register must be it
                       n_src_rn_r_v && (n_src_rn_r == rd_ir[2:0]) &&
                       ((n_src_mode_r == 3'b010) || (n_src_mode_r == 3'b011) || (n_src_mode_r == 3'b100) ||
                        ((n_src_mode_r == 3'b101) && epf_ready_pc2));
wire        hint_rsr = (state == S_MWR) && (r_m_ret == S_NEXT) && m_issued &&
                       ((mwr_fresh && st_hinted && mem_fast_ready) || (!mwr_fresh && mem_ack_q)) && !ifr_pres && rsr_head &&
`ifdef AP040_EXPERIMENTAL_PIPELINE
                       !pipe_rf_owner && !pipe_write &&
`endif
                       !sr[15];
wire        retire_store_read = hint_rsr && d_ack;
// P219: the same handoff after a read that retires in its own acknowledge
// (retire_operand_alu: MOVEA, MOVE/ALU to a register): the next
// instruction's (An)/d16(An) source read is hinted in the predicted
// acknowledge clock and issued on it.  Not (An)+/-(An) (the retiring
// register write holds the one write port) and not when the next base is
// the register being loaded.
// P232: P219 is DISABLED -- with it the Mac OS 8.1 boot ends in a Sad Mac
// (0000000F/00000002) in the full-machine sim and on hardware (P220, P229);
// the bisect of P216-P220 put it on P219.  Left in place, gated off, until the
// fault is understood.
wire        hint_rrr = 1'b0 && (state == S_MRD) && m_issued && (r_m_ret == S_PIPE_SDONE) &&
                       (p_src == SK_MEM) && (p_dst == DK_REG) && (exec_kind == EK_ALU) &&
`ifdef AP040_EXPERIMENTAL_PIPELINE
                       !pipe_load_active && !pipe_rf_owner && !pipe_write &&
`endif
                       ((mrd_fresh && mrd_hinted && mem_fast_ready) || (!mrd_fresh && mem_ack_q)) &&
                       !ifr_pres && rsr_head &&
                       ((n_src_mode_r == 3'b010) || (n_src_mode_r == 3'b101)) &&
                       !(!p_wbsup && (p_dreg == {1'b1, rd_ir[2:0]})) && !sr[15];
wire        retire_read_read = hint_rrr && d_ack;
// P198: the return after UNLK.  UNLK retires in its read's acknowledge with
// A7 already moved (written at the read's issue); when the queue head is
// RTS, or RTD with its displacement resident, the data hint shows A7 in
// UNLK's predicted acknowledge clock and the return's pop issues in place
// on that acknowledge, instead of from S_DECODE a clock later.
wire        rsp_head = epf_ready_pc && (rd_ir == 16'h4E75);   // RTS only (RTD finishes through S_RET2: no gain)
wire        hint_rsp = (state == S_MRD) && (r_m_ret == S_UNLK3) && m_issued &&
                       ((mrd_fresh && mrd_hinted && mem_fast_ready) || (!mrd_fresh && mem_ack_q)) && !ifr_pres && rsp_head &&
                       (d_rn != 3'd7) && !sr[15];
wire        ret_after_unlk = hint_rsp && d_ack;
// P205: the next longword of a multiword FPU operand.  The S_MRD
// acknowledge already queues it (mrd from the ack branch); in a clock the
// current beat is predicted to complete, the data hint shows the next beat
// and the acknowledge issues it in place, so each beat after the first
// costs one clock instead of an issue clock plus a hit clock.
wire        fpu_rd_more = ((r_m_ret == S_FPU_RD) &&
                           ((fp_nb == 4'd8 && fp_n == 4'd1) ||
                            (fp_nb == 4'd12 && (fp_n == 4'd1 || fp_n == 4'd2)))) ||
                          ((r_m_ret == S_FPU_MVM3) && !fp_st && (fp_n != 4'd3));
wire        hint_fpn = (state == S_MRD) && fpu_rd_more && m_issued &&
                       ((mrd_fresh && mrd_hinted && mem_fast_ready) || (!mrd_fresh && mem_ack_q)) &&
                       !ifr_pres && !sr[15];
wire [31:0] fpn_addr = t_a + {28'd0, fp_n[1:0], 2'b00};
wire        fpu_rd_next = hint_fpn && d_ack;
// P223: S_FPU_DEC resolves an (An)/(An)+/-(An) operand itself when the base is
// already on port A (dispatch_fpu selects it); S_FPU_AN's size adjust
wire        fpu_an_now = (rr_a == {1'b1, d_rn}) && !aux_we;
wire  [3:0] fpu_an_nb  = fp_bytes(imm[12:10]);
wire  [6:0] fpu_an_adj = (fpu_an_nb == 4'd1 && d_rn == 3'd7) ? 7'd2 : {3'b000, fpu_an_nb};
// P216: MOVEM load chaining -- the next register's read hinted in the current
// one's predicted acknowledge and issued in place on it (see movem_ld_ack)
wire [31:0] mmn_addr = mm_addr + ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
wire        hint_mmn = (state == S_MRD) && (r_m_ret == S_MOVEM_LD) && m_issued && (mm_mask != 16'd0) &&
                       ((mrd_fresh && mrd_hinted && mem_fast_ready) || (!mrd_fresh && mem_ack_q)) &&
                       !ifr_pres && !sr[15];
wire        mm_rd_next = hint_mmn && d_ack;
assign mem_hint_away = hint_move_store || hint_rsr || hint_rrr || hint_rsp || hint_fpn || hint_mmn;
// P175: two hint buses.  The data bus carries the outstanding data request
// and the data-side hints; the instruction bus carries the presented
// fetch until it clears, else the redirect targets, else the queue's next
// fetch.  Each side's idle read in the cache is indexed by its own bus, so
// a fetch and a data access can both be hinted in the same clock.
assign mem_hint_addr  = hint_move_store ? move_store_read_addr :
                        (hint_rsr || hint_rrr) ? rsr_addr :
                        hint_rsp ? dbg_a7_wb :
                        hint_fpn ? fpn_addr :
                        hint_mmn ? mmn_addr :
                        mem_req ? mem_addr_q : hint_addr;
assign mem_hint_instr = 1'b0;
// P177: a return-address stack.  The RTS target fetch is issued in the
// pop read's acknowledge clock from the popped data, so nothing can hint
// it -- except a prediction: the return address pushed by the matching
// JSR/BSR (recorded at the push store's acknowledge).  During the pop
// read the instruction hint bus carries the prediction; when the popped
// address agrees, the target fetch is a one-clock hit.  A wrong
// prediction costs nothing (the fetch is un-hinted, as before).
// The redirect hints (this one, hint_bcc, hint_redir, hint_ftb, hint_bd)
// take the bus over a fetch that is outstanding but not presented: such
// a fetch is replaced by the redirect (ifr_avail) in the next clock.
reg  [31:0] ras [0:7];
reg   [2:0] ras_sp;
wire        hint_ras = (state == S_MRD) && (r_m_ret == S_RET2) && ((ret_kind == RK_RTS) || (ret_kind == RK_RTD)) && m_issued;
wire [31:0] ras_top  = ras[ras_sp - 3'd1];
assign mem_ihint_addr = (ifr_req && ifr_pres) ? ifr_addr :
                        hint_ras   ? ras_top :
                        hint_bcc   ? (pc + sxb(ir[7:0])) :
                        hint_redir ? hint_redir_addr :
                        hint_ftb   ? rd_bcc_t :
                        hint_bd    ? bd_t :
                        ifr_req    ? ifr_addr : epf_ftail;

//---------------------------------------------------------------------------
// main state machine
//---------------------------------------------------------------------------

// ---- generated by gen_decoder.py from the S_DECODE body: the decode record of the queue head
localparam NX_NONE = 3'd0, NX_PSTART = 3'd1, NX_PREGS = 3'd2, NX_IMMF_PSTART = 3'd3, NX_IMMREG = 3'd4;
wire [3:0] rd_ir_hi     = rd_ir[15:12];
wire [2:0] rdd_reg9     = rd_ir[11:9];
wire [2:0] rdd_op8_6    = rd_ir[8:6];
wire [2:0] rdd_mode     = rd_ir[5:3];
wire [2:0] rdd_rn       = rd_ir[2:0];
wire [1:0] rdd_move_size = (rd_ir[13:12] == 2'b01) ? `AP040_SZ_B : (rd_ir[13:12] == 2'b11) ? `AP040_SZ_W : `AP040_SZ_L;
wire [1:0] rdd_std_size = rd_ir[7:6];
wire       rdd_ea_is_imm = (rdd_mode == 3'b111) && (rdd_rn == 3'b100);
wire       rdd_dst_not_alt = (rdd_mode == 3'b001) || ((rdd_mode == 3'b111) && (rdd_rn > 3'b001));
wire       rdd_src_not_data = (rdd_mode == 3'b001) || ((rdd_mode == 3'b111) && (rdd_rn > 3'b100));
reg        n_inplace;
reg  [2:0] n_next;
reg  [1:0] n_immn;
reg [2:0]    n_p_src; reg n_p_src_v;
reg [2:0]    n_p_dst; reg n_p_dst_v;
reg          n_p_rmw; reg n_p_rmw_v;
reg          n_p_wbsup; reg n_p_wbsup_v;
reg          n_p_flags; reg n_p_flags_v;
reg          n_p_sextw; reg n_p_sextw_v;
reg          n_p_dst_mem_bit; reg n_p_dst_mem_bit_v;
reg [3:0]    n_exec_kind; reg n_exec_kind_v;
reg [1:0]    n_op_size; reg n_op_size_v;
reg [5:0]    n_alu_op; reg n_alu_op_v;
reg [3:0]    n_p_dreg; reg n_p_dreg_v;
reg [1:0]    n_p_dsize; reg n_p_dsize_v;
reg [1:0]    n_p_ssize; reg n_p_ssize_v;
reg [3:0]    n_p_sreg; reg n_p_sreg_v;
reg [2:0]    n_dst_rn_r; reg n_dst_rn_r_v;
reg [2:0]    n_dst_mode_r; reg n_dst_mode_r_v;
reg [2:0]    n_src_rn_r; reg n_src_rn_r_v;
reg [2:0]    n_src_mode_r; reg n_src_mode_r_v;
reg [3:0]    n_rr_a; reg n_rr_a_v;
reg [3:0]    n_rr_b; reg n_rr_b_v;
reg [31:0]   n_src_val; reg n_src_val_v;
reg          n_sh_rox; reg n_sh_rox_v;
reg          n_md_isdiv; reg n_md_isdiv_v;
reg          n_md_sign; reg n_md_sign_v;
reg          n_lk_cyc; reg n_lk_cyc_v;
always @* begin : decode_record
	n_inplace = 0; n_next = NX_NONE; n_immn = 2'd1;
	n_p_src = SK_NONE; n_p_src_v = 1;
	n_p_dst = DK_NONE; n_p_dst_v = 1;
	n_p_rmw = 1'b0; n_p_rmw_v = 1;
	n_p_wbsup = 1'b0; n_p_wbsup_v = 1;
	n_p_flags = 1'b1; n_p_flags_v = 1;
	n_p_sextw = 1'b0; n_p_sextw_v = 1;
	n_p_dst_mem_bit = 1'b0; n_p_dst_mem_bit_v = 1;
	n_exec_kind = EK_ALU; n_exec_kind_v = 1;
	n_op_size = 0; n_op_size_v = 0;
	n_alu_op = 0; n_alu_op_v = 0;
	n_p_dreg = 0; n_p_dreg_v = 0;
	n_p_dsize = 0; n_p_dsize_v = 0;
	n_p_ssize = 0; n_p_ssize_v = 0;
	n_p_sreg = 0; n_p_sreg_v = 0;
	n_dst_rn_r = 0; n_dst_rn_r_v = 0;
	n_dst_mode_r = 0; n_dst_mode_r_v = 0;
	n_src_rn_r = 0; n_src_rn_r_v = 0;
	n_src_mode_r = 0; n_src_mode_r_v = 0;
	n_rr_a = 0; n_rr_a_v = 0;
	n_rr_b = 0; n_rr_b_v = 0;
	n_src_val = 0; n_src_val_v = 0;
	n_sh_rox = 0; n_sh_rox_v = 0;
	n_md_isdiv = 0; n_md_isdiv_v = 0;
	n_md_sign = 0; n_md_sign_v = 0;
	n_lk_cyc = 0; n_lk_cyc_v = 0;
				// Generic source-memory forms all use the opcode's low An.
				// Select it while decoding so simple An modes can consume the
				// asynchronous port-A value as soon as the pipeline starts.
				// Decode cases which need another register override this below.
				begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end
				case (rd_ir_hi)
					//-------------------------------------------- 0x0: bit/imm
					4'h0: begin
						if (rd_ir[8] && rdd_mode == 3'b001) begin
							// MOVEP
							n_inplace = 1;
							n_inplace = 1;
							n_inplace = 1;
						end
						else if (rd_ir[8]) begin
							// dynamic bit op, bit number in Dn
							// As for the static forms, an address register is
							// never a bit destination.  BTST may read program
							// space or an immediate operand, but the modifying
							// forms require a data-alterable destination.
							if (rdd_mode == 3'b001) n_inplace = 1;
							else if (rdd_mode == 3'b111 &&
							         ((rd_ir[7:6] == 2'b00) ? (rdd_rn > 3'b100)
							                             : (rdd_rn > 3'b001)))
								n_inplace = 1;
							else begin
								begin n_alu_op = `AP040_ALU_BTST + {4'd0, rd_ir[7:6]}; n_alu_op_v = 1; end
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_reg9}; n_p_sreg_v = 1; end begin n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; end
								if (rd_ir[7:6] == 2'b00) begin n_p_wbsup = 1; n_p_wbsup_v = 1; end // BTST
								if (rdd_ea_is_imm)
									n_inplace = 1;
								else if (rdd_mode == 3'b000) begin
									begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
									begin n_p_dsize = `AP040_SZ_L; n_p_dsize_v = 1; end
									begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
									n_next = NX_PSTART;
								end
								else begin
									begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
									begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
									begin n_p_dst = DK_MEM; n_p_dst_v = 1; end
									begin n_p_dst_mem_bit = 1; n_p_dst_mem_bit_v = 1; end
									begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
									begin n_p_rmw = 1; n_p_rmw_v = 1; end
									n_next = NX_PSTART;
								end
							end
						end
						else if (rdd_reg9 == 3'b100) begin
							// static bit op, bit number in extension word
							// (checked before the size=11 group: BSET is 00xx11)
							// BTST only reads, so it accepts program space and
							// an immediate operand; BCHG/BCLR/BSET write and
							// need a data alterable destination.  An address
							// register is never allowed.
							if (rdd_mode == 3'b001) n_inplace = 1;
							else if (rdd_mode == 3'b111 &&
							         ((rd_ir[7:6] == 2'b00) ? (rdd_rn > 3'b100)
							                             : (rdd_rn > 3'b001)))
								n_inplace = 1;
							else begin
							begin n_alu_op = `AP040_ALU_BTST + {4'd0, rd_ir[7:6]}; n_alu_op_v = 1; end
							begin n_p_src = SK_IMM; n_p_src_v = 1; end
							if (rdd_mode == 3'b000) begin
								begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
								begin n_p_dsize = `AP040_SZ_L; n_p_dsize_v = 1; end
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
							end
							else begin
								begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
								begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end
								begin n_p_dst_mem_bit = 1; n_p_dst_mem_bit_v = 1; end
								begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
								begin n_p_rmw = 1; n_p_rmw_v = 1; end
							end
							if (rd_ir[7:6] == 2'b00) begin n_p_wbsup = 1; n_p_wbsup_v = 1; end
							begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end
							end
						end
						else if (rdd_reg9 == 3'b111 && rdd_std_size != 2'b11) begin
							// MOVES (0000 1110 11 is CAS.L, not implemented)
							// Validate the effective-address encoding before the
							// privilege check.  Invalid MOVES encodings take vector
							// 4 even in user mode; only a valid MOVES is privileged.
							if (rdd_mode < 3'b010 ||
							    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
							else if (!sr_s) n_inplace = 1;
							else begin
								begin n_op_size = rdd_std_size; n_op_size_v = 1; end
								n_inplace = 1;
							end
						end
						else if (rdd_std_size == 2'b11) begin
							if (!rdd_reg9[2] && rdd_reg9[1:0] != 2'b11) begin
								// CHK2/CMP2: bounds pair at a control EA
								if (rdd_mode < 3'b010 || rdd_mode == 3'b011 ||
								    rdd_mode == 3'b100 || rdd_ea_is_imm) n_inplace = 1;
								else begin
									begin n_op_size = rdd_reg9[1] ? `AP040_SZ_L :
									           rdd_reg9[0] ? `AP040_SZ_W : `AP040_SZ_B; n_op_size_v = 1; end
									n_inplace = 1;
								end
							end
							else if (rdd_reg9[2] && rdd_reg9[1:0] != 2'b00 && rdd_ea_is_imm) begin
								// CAS2.W/.L: two extension words follow
								if (rdd_reg9[1:0] == 2'b01) n_inplace = 1;   // no CAS2.B
								else begin
									begin n_alu_op = `AP040_ALU_CMP; n_alu_op_v = 1; end
									begin n_op_size = (rdd_reg9[1:0] == 2'b10) ? `AP040_SZ_W : `AP040_SZ_L; n_op_size_v = 1; end
									begin begin n_lk_cyc = 1; n_lk_cyc_v = 1; end n_inplace = 1; end
								end
							end
							else if (rdd_reg9[2] && rdd_reg9[1:0] != 2'b00) begin
								// CAS (memory only)
								if (rdd_mode < 3'b010 ||
								    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
								else begin
									begin n_alu_op = `AP040_ALU_CMP; n_alu_op_v = 1; end
									begin n_op_size = (rdd_reg9[1:0] == 2'b01) ? `AP040_SZ_B :
									           (rdd_reg9[1:0] == 2'b10) ? `AP040_SZ_W : `AP040_SZ_L; n_op_size_v = 1; end
									begin begin n_lk_cyc = 1; n_lk_cyc_v = 1; end n_inplace = 1; end
								end
							end
							else n_inplace = 1;   // CAS2 / CHK2 / CMP2
						end
						else begin
							// ORI/ANDI/SUBI/ADDI/EORI/CMPI
							if (rdd_ea_is_imm && (rdd_reg9 == 3'b000 || rdd_reg9 == 3'b001 || rdd_reg9 == 3'b101)) begin
								// to CCR (byte) or SR (word, privileged)
								if (rdd_std_size == 2'b01 && !sr_s) n_inplace = 1;
								else if (rdd_std_size > 2'b01) n_inplace = 1;
								else begin
									n_inplace = 1;
									n_inplace = 1;
									n_inplace = 1;
								end
							end
							else if (rdd_mode == 3'b001) n_inplace = 1;
							// The destination must be data alterable, so the
							// PC-relative and immediate encodings of mode 7
							// are illegal.  CMPI is the exception: the 68020
							// and later allow it to read program space.
							else if (rdd_mode == 3'b111 && rdd_rn > 3'b001 &&
							         !(rdd_reg9 == 3'b110 && rdd_rn < 3'b100))
								n_inplace = 1;
							else begin
								case (rdd_reg9)
									3'b000: begin n_alu_op = `AP040_ALU_OR; n_alu_op_v = 1; end
									3'b001: begin n_alu_op = `AP040_ALU_AND; n_alu_op_v = 1; end
									3'b010: begin n_alu_op = `AP040_ALU_SUB; n_alu_op_v = 1; end
									3'b011: begin n_alu_op = `AP040_ALU_ADD; n_alu_op_v = 1; end
									3'b101: begin n_alu_op = `AP040_ALU_EOR; n_alu_op_v = 1; end
									default: begin n_alu_op = `AP040_ALU_CMP; n_alu_op_v = 1; end
								endcase
								if (rdd_reg9 == 3'b110) begin n_p_wbsup = 1; n_p_wbsup_v = 1; end // CMPI
								begin n_op_size = rdd_std_size; n_op_size_v = 1; end
								begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
								begin n_p_src = SK_IMM; n_p_src_v = 1; end
								if (rdd_mode == 3'b000) begin
									begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
									begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_IMMREG; end
								end
								else begin
									begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_p_rmw = 1; n_p_rmw_v = 1; end
									begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
									begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end
								end
							end
						end
					end

					//------------------------------------------- 0x1-0x3: MOVE
					4'h1, 4'h2, 4'h3: begin
						if (rd_valid) begin end // descriptor applied after state case
						else if (rdd_move_size == `AP040_SZ_B &&
						    (rdd_mode == 3'b001 || rdd_op8_6 == 3'b001)) n_inplace = 1;
						else if (rdd_op8_6 == 3'b111 && rdd_reg9 > 3'b001) n_inplace = 1;
						else begin
							begin n_alu_op = `AP040_ALU_MOVE; n_alu_op_v = 1; end
							begin n_op_size = rdd_move_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_move_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_move_size; n_p_dsize_v = 1; end
							// source
							if (rdd_mode == 3'b000 || rdd_mode == 3'b001) begin
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {rdd_mode[0], rdd_rn}; n_p_sreg_v = 1; end
							end
							else if (rdd_ea_is_imm) begin n_p_src = SK_IMM; n_p_src_v = 1; end
							else begin
								begin n_p_src = SK_MEM; n_p_src_v = 1; end
								begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
							end
							// destination
							if (rdd_op8_6 == 3'b000) begin
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
							end
							else if (rdd_op8_6 == 3'b001) begin
								// MOVEA: full register, no flags, word sexts
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b1, rdd_reg9}; n_p_dreg_v = 1; end
								begin n_p_flags = 0; n_p_flags_v = 1; end
								if (rdd_move_size == `AP040_SZ_W) begin n_p_sextw = 1; n_p_sextw_v = 1; end
								begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
							end
							else begin
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end
								begin n_dst_mode_r = rdd_op8_6; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
								// A register or immediate source leaves port A
								// free for the destination base and port B for
								// the source register, so S_PIPE_START can start
								// the destination EA at once (memory-destination
								// fast path).  A memory source keeps port A.
								if (rdd_mode == 3'b000 || rdd_mode == 3'b001 || rdd_ea_is_imm)
									begin n_rr_a = {1'b1, rdd_reg9}; n_rr_a_v = 1; end
								if (rdd_mode == 3'b000 || rdd_mode == 3'b001)
									begin n_rr_b = {rdd_mode[0], rdd_rn}; n_rr_b_v = 1; end
							end
							if (rdd_ea_is_imm && (rdd_op8_6 == 3'b000 || rdd_op8_6 == 3'b001))
								begin n_immn = (rdd_move_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_rr_b = {rdd_op8_6[0], rdd_reg9}; n_rr_b_v = 1; n_next = NX_IMMREG; end
							else if (rdd_ea_is_imm)
								begin n_immn = (rdd_move_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end
							else n_next = NX_PSTART;
						end
					end

					//------------------------------------------------ 0x4: misc
					4'h4: begin
						if (rd_ir[11:0] == 12'hAFC) n_inplace = 1;   // ILLEGAL
						else if (rdd_op8_6 == 3'b111) begin
							if (rdd_mode == 3'b000) begin
								if (rdd_reg9 == 3'b100) begin
									// EXTB.L
									begin n_alu_op = `AP040_ALU_EXTB; n_alu_op_v = 1; end
									begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
									begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
									n_next = NX_PSTART;
								end
								else n_inplace = 1;
							end
							else if (rdd_mode == 3'b001 || (rdd_mode == 3'b011) ||
							         (rdd_mode == 3'b100) || rdd_ea_is_imm) n_inplace = 1;
							else n_inplace = 1; // LEA
						end
						else if (rdd_op8_6 == 3'b110) begin
							// CHK.W
							begin n_exec_kind = EK_CHK; n_exec_kind_v = 1; end
							begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
							begin n_p_ssize = `AP040_SZ_W; n_p_ssize_v = 1; end
							if (rdd_mode == 3'b001) n_inplace = 1;
							else begin
								if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end end
								else if (rdd_ea_is_imm) begin n_p_src = SK_IMM; n_p_src_v = 1; end
								else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end end
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
								if (rdd_ea_is_imm) begin n_immn = 2'd1; n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; n_next = NX_IMMREG; end
								else n_next = NX_PSTART;
							end
						end
						else if (rdd_op8_6 == 3'b100 &&
						         !(rd_ir[11:9] == 3'b100 && rdd_mode == 3'b001)) begin
							// CHK.L (0100 ddd 100; 0100 100 000 001 rrr is LINK.L)
							begin n_exec_kind = EK_CHK; n_exec_kind_v = 1; end
							begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
							begin n_p_ssize = `AP040_SZ_L; n_p_ssize_v = 1; end
							if (rdd_mode == 3'b001) n_inplace = 1;
							else begin
								if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end end
								else if (rdd_ea_is_imm) begin n_p_src = SK_IMM; n_p_src_v = 1; end
								else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end end
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
								if (rdd_ea_is_imm) begin n_immn = 2'd2; n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; n_next = NX_IMMREG; end
								else n_next = NX_PSTART;
							end
						end
						else case (rd_ir[11:9])
							3'b000: begin
								if (rdd_op8_6 == 3'b011) begin
									// MOVE from SR (privileged on 68010+)
									// EA legality is decoded before privilege.  In user
									// mode MOVE SR,An/PC/#imm is vector 4, not vector 8.
									if (rdd_dst_not_alt) n_inplace = 1;
									else if (!sr_s) n_inplace = 1;
									else begin
										begin n_p_src = SK_IMPL; n_p_src_v = 1; end n_inplace = 1;
										begin n_alu_op = `AP040_ALU_MOVE; n_alu_op_v = 1; end
										begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
										begin n_p_dsize = `AP040_SZ_W; n_p_dsize_v = 1; end
										begin n_p_flags = 0; n_p_flags_v = 1; end
										if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
										else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
									end
								end
								else if (rdd_op8_6[2]) n_inplace = 1;
								else begin
									// NEGX
									begin n_alu_op = `AP040_ALU_NEGX; n_alu_op_v = 1; end
									begin n_op_size = rdd_std_size; n_op_size_v = 1; end
									begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
									begin n_p_rmw = 1; n_p_rmw_v = 1; end
									if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
									else if (rdd_dst_not_alt) n_inplace = 1;
									else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
								end
							end

							3'b001: begin
								if (rdd_op8_6 == 3'b011) begin
									// MOVE from CCR
									begin n_p_src = SK_IMPL; n_p_src_v = 1; end n_inplace = 1;
									begin n_alu_op = `AP040_ALU_MOVE; n_alu_op_v = 1; end
									begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
									begin n_p_dsize = `AP040_SZ_W; n_p_dsize_v = 1; end
									begin n_p_flags = 0; n_p_flags_v = 1; end
										if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
										else if (rdd_dst_not_alt) n_inplace = 1;
										else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
								end
								else if (rdd_op8_6[2]) n_inplace = 1;
								else begin
									// CLR (pure write on 68040)
									begin n_alu_op = `AP040_ALU_CLR; n_alu_op_v = 1; end
									begin n_op_size = rdd_std_size; n_op_size_v = 1; end
									begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
									if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
									else if (rdd_dst_not_alt) n_inplace = 1;
									else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
								end
							end

							3'b010: begin
								if (rdd_op8_6 == 3'b011) begin
									// MOVE to CCR
									begin n_alu_op = `AP040_ALU_MOVE; n_alu_op_v = 1; end
									begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
									begin n_p_ssize = `AP040_SZ_W; n_p_ssize_v = 1; end
									begin n_p_flags = 0; n_p_flags_v = 1; end
									begin n_p_dst = DK_CCR; n_p_dst_v = 1; end
									if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end n_next = NX_PSTART; end
									else if (rdd_src_not_data) n_inplace = 1;
									else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end end
									else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
								end
								else if (rdd_op8_6[2]) n_inplace = 1;
								else begin
									// NEG
									begin n_alu_op = `AP040_ALU_NEG; n_alu_op_v = 1; end
									begin n_op_size = rdd_std_size; n_op_size_v = 1; end
									begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
									begin n_p_rmw = 1; n_p_rmw_v = 1; end
									if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
									else if (rdd_dst_not_alt) n_inplace = 1;
									else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
								end
							end

							3'b011: begin
								if (rdd_op8_6 == 3'b011) begin
									// MOVE to SR (privileged)
									// An is not a legal source; encoding rejection wins
									// over the privilege check just as for MOVE from SR.
									if (rdd_src_not_data) n_inplace = 1;
									else if (!sr_s) n_inplace = 1;
									else begin
										begin n_alu_op = `AP040_ALU_MOVE; n_alu_op_v = 1; end
										begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
										begin n_p_ssize = `AP040_SZ_W; n_p_ssize_v = 1; end
										begin n_p_flags = 0; n_p_flags_v = 1; end
										begin n_p_dst = DK_SR; n_p_dst_v = 1; end
										if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end n_next = NX_PSTART; end
										else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end end
										else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
									end
								end
								else if (rdd_op8_6[2]) n_inplace = 1;
								else begin
									// NOT
									begin n_alu_op = `AP040_ALU_NOT; n_alu_op_v = 1; end
									begin n_op_size = rdd_std_size; n_op_size_v = 1; end
									begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
									begin n_p_rmw = 1; n_p_rmw_v = 1; end
									if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
									else if (rdd_dst_not_alt) n_inplace = 1;
									else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
								end
							end

							3'b100: begin
								if (rdd_op8_6[2]) n_inplace = 1;
								else case (rdd_op8_6[1:0])
								2'b00: begin
									if (rdd_mode == 3'b001) begin
										// LINK.L An,#bd32
										n_inplace = 1;
										begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end
										n_inplace = 1;
									end
									else begin
										// NBCD
										begin n_alu_op = `AP040_ALU_NBCD; n_alu_op_v = 1; end
										begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
										begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
										begin n_p_rmw = 1; n_p_rmw_v = 1; end
										if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
										else if (rdd_dst_not_alt) n_inplace = 1;
										else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
									end
								end
								2'b01: begin
									if (rdd_mode == 3'b000) begin
										// SWAP
										begin n_alu_op = `AP040_ALU_SWAP; n_alu_op_v = 1; end
										begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
										begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
										n_next = NX_PSTART;
									end
									else if (rdd_mode == 3'b001) n_inplace = 1; // BKPT
									else if (rdd_mode == 3'b011 || rdd_mode == 3'b100 || rdd_ea_is_imm) n_inplace = 1;
									else n_inplace = 1; // PEA
								end
								default: begin
									if (rdd_mode == 3'b000) begin
										// EXT.W / EXT.L
										begin n_alu_op = `AP040_ALU_EXT; n_alu_op_v = 1; end
										begin n_op_size = rdd_op8_6[0] ? `AP040_SZ_L : `AP040_SZ_W; n_op_size_v = 1; end
										begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
										n_next = NX_PSTART;
									end
									else begin
										// MOVEM registers to memory
										n_inplace = 1;
										n_inplace = 1;
										n_inplace = 1;
										n_inplace = 1;
										if (rdd_mode == 3'b011 || rdd_mode < 3'b010 || rdd_ea_is_imm ||
										    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
										else n_inplace = 1;
									end
								end
								endcase
							end

							3'b101: begin
								if (rdd_op8_6[2]) n_inplace = 1;
								else if (rdd_op8_6 == 3'b011) begin
									// TAS (not bus locked yet).  The 040
									// reports its operand cycles as a locked
									// RMW: an access error carries SSW LK
									// with RW clear.
									begin n_alu_op = `AP040_ALU_TAS; n_alu_op_v = 1; end
									begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
									begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
									begin n_p_rmw = 1; n_p_rmw_v = 1; end
									if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
									else if (rdd_dst_not_alt) n_inplace = 1;
									else begin
										begin n_lk_cyc = 1; n_lk_cyc_v = 1; end
										begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART;
									end
								end
								else begin
									// TST (An/imm/PC modes allowed on 020+)
									begin n_alu_op = `AP040_ALU_TST; n_alu_op_v = 1; end
									begin n_op_size = rdd_std_size; n_op_size_v = 1; end
									begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end
									begin n_p_wbsup = 1; n_p_wbsup_v = 1; end
									if (rdd_mode == 3'b000 || rdd_mode == 3'b001) begin
										if (rdd_mode == 3'b001 && rdd_std_size == `AP040_SZ_B) n_inplace = 1;
										else begin
											begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {rdd_mode[0], rdd_rn}; n_p_sreg_v = 1; end
											n_next = NX_PSTART;
										end
									end
									else if (rdd_ea_is_imm) begin
										begin n_p_src = SK_IMM; n_p_src_v = 1; end
										begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end
									end
									else begin
										begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
										n_next = NX_PSTART;
									end
								end
							end

							3'b110: begin
								if (rdd_op8_6[2]) n_inplace = 1;
								else if (!rdd_op8_6[1]) begin
									// MULx.L / DIVx.L with extension word
									begin n_exec_kind = EK_MD_L; n_exec_kind_v = 1; end
									begin n_md_isdiv = rdd_op8_6[0]; n_md_isdiv_v = 1; end
									begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
									begin n_p_ssize = `AP040_SZ_L; n_p_ssize_v = 1; end
									if (rdd_mode == 3'b001) n_inplace = 1;
									else begin
										if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end end
										else if (rdd_ea_is_imm) begin n_p_src = SK_IMM; n_p_src_v = 1; end
										else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end end
										// extension word first, then any immediate
										n_inplace = 1;
									end
								end
								else begin
									// MOVEM memory to registers
									n_inplace = 1;
									n_inplace = 1;
									n_inplace = 1;
									n_inplace = 1;
									if (rdd_mode == 3'b100 || rdd_mode < 3'b010 || rdd_ea_is_imm) n_inplace = 1;
									else n_inplace = 1;
								end
							end

							default: begin // 3'b111
								if (rdd_op8_6 == 3'b010) begin
									// JSR
									if (rdd_mode < 3'b010 || rdd_mode == 3'b011 ||
									    rdd_mode == 3'b100 || rdd_ea_is_imm) n_inplace = 1;
									else n_inplace = 1;
								end
								else if (rdd_op8_6 == 3'b011) begin
									// JMP
									if (rdd_mode < 3'b010 || rdd_mode == 3'b011 ||
									    rdd_mode == 3'b100 || rdd_ea_is_imm) n_inplace = 1;
									else n_inplace = 1;
								end
								else if (rdd_op8_6 == 3'b001) begin
									casez (rd_ir[5:0])
										6'b00????: n_inplace = 1;
										6'b010???: begin n_inplace = 1; begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end n_inplace = 1; end // LINK.W
										6'b011???: begin begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end n_inplace = 1; end
										6'b100???: begin // MOVE An,USP
											if (!sr_s) n_inplace = 1;
											else begin begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end n_inplace = 1; end
										end
										6'b101???: begin // MOVE USP,An
											if (!sr_s) n_inplace = 1;
											else begin n_inplace = 1; n_inplace = 1; end
										end
										6'b110000: begin // RESET
											if (!sr_s) n_inplace = 1;
											else begin n_inplace = 1; n_inplace = 1; end
										end
										6'b110001: n_inplace = 1;   // NOP
										6'b110010: begin // STOP
											if (!sr_s) n_inplace = 1;
											else n_inplace = 1;
										end
										6'b110011: begin // RTE
											if (!sr_s) n_inplace = 1;
											else begin
												// A bus/access fault while RTE is loading internal state
												// from the old frame is a double bus fault (MC68040 UM
												// 8.2), not a new format-$7 exception.
												n_inplace = 1;
												n_inplace = 1;
											end
										end
										6'b110100: begin n_inplace = 1; n_inplace = 1; end
										// RTS: the pop issues from decode (S_RET1 skipped)
										6'b110101: begin n_inplace = 1; n_inplace = 1; end
										6'b110110: begin // TRAPV
											if (sr[1]) n_inplace = 1;
											else n_inplace = 1;
										end
										6'b110111: begin
											n_inplace = 1;
											n_inplace = 1;
										end
										6'b111010, 6'b111011: begin // MOVEC
											if (!sr_s) n_inplace = 1;
											else begin
												n_inplace = 1;
												n_inplace = 1;
											end
										end
										default: n_inplace = 1;
									endcase
								end
								else n_inplace = 1;
							end
						endcase
					end

					//------------------------------ 0x5: ADDQ/SUBQ/Scc/DBcc
					4'h5: begin
						if (rd_valid) begin end
						else if (rd_ir[7:6] == 2'b11) begin
							if (rdd_mode == 3'b001) begin
								// DBcc
								n_inplace = 1;
								begin n_rr_a = {1'b0, rdd_rn}; n_rr_a_v = 1; end
								n_inplace = 1;
							end
							else if (rdd_mode == 3'b111 && rdd_rn >= 3'b010 && rdd_rn <= 3'b100) begin
								// TRAPcc (optional operand words are consumed
								// but otherwise ignored)
								if (rdd_rn == 3'b010)
									n_inplace = 1;
								else if (rdd_rn == 3'b011)
									n_inplace = 1;
								else begin
									if (cond_true(rd_ir[11:8]))
										n_inplace = 1;
									else n_inplace = 1;
								end
							end
							else begin
								// Scc
								begin n_exec_kind = EK_SCC; n_exec_kind_v = 1; end
								begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
								begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
								begin n_p_flags = 0; n_p_flags_v = 1; end
								if (rdd_mode == 3'b000) begin begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end begin n_rr_b = {1'b0, rdd_rn}; n_rr_b_v = 1; n_next = NX_PREGS; end end
								else if (rdd_dst_not_alt) n_inplace = 1;
								else begin begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end n_next = NX_PSTART; end
							end
						end
						else begin
							// Register quick forms are handled by rd_valid above.
							begin n_alu_op = rd_ir[8] ? `AP040_ALU_SUB : `AP040_ALU_ADD; n_alu_op_v = 1; end
							begin n_p_src = SK_IMPL; n_p_src_v = 1; end
							begin n_src_val = {28'd0, (rdd_reg9 == 3'd0) ? 4'd8 : {1'b0, rdd_reg9}}; n_src_val_v = 1; end
							if (rdd_dst_not_alt) n_inplace = 1; // includes byte An
							else begin
								begin n_op_size = rdd_std_size; n_op_size_v = 1; end
								begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_p_rmw = 1; n_p_rmw_v = 1; end
								begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
								n_next = NX_PSTART;
							end
						end
					end

					//---------------------------------------- 0x6: Bcc/BSR/BRA
					4'h6: begin
						if (rd_ir[7:0] == 8'h00 || rd_ir[7:0] == 8'hFF) begin
							n_inplace = 1;
							n_inplace = 1;
							n_inplace = 1;
						end
						else if (rd_ir[11:8] == 4'h1) begin : bsr_b
							// BSR.B; an odd target faults with A7 untouched
							reg [31:0] bt;
							bt = pc + sxb(rd_ir[7:0]);
							if (bt[0]) n_inplace = 1;
							else begin
								n_inplace = 1;
								n_inplace = 1;
							end
						end
						else n_inplace = 1;
					end

					//------------------------------------------- 0x7: MOVEQ
					4'h7: begin
						if (rd_ir[8]) n_inplace = 1;
						else begin
							n_inplace = 1;
							n_inplace = 1;
							n_inplace = 1;
							n_inplace = 1; n_inplace = 1;
							n_inplace = 1;
						end
					end

					//------------------------------------- 0x8: OR/DIV/SBCD
					4'h8: begin
						if (rd_valid) begin end
						else if (rdd_op8_6 == 3'b011 || rdd_op8_6 == 3'b111) begin
							// DIVU.W / DIVS.W
							begin n_exec_kind = EK_MD_W; n_exec_kind_v = 1; end
							begin n_md_isdiv = 1; n_md_isdiv_v = 1; end
							begin n_md_sign = rdd_op8_6[2]; n_md_sign_v = 1; end
							begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
							begin n_p_ssize = `AP040_SZ_W; n_p_ssize_v = 1; end
							begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
							if (rdd_mode == 3'b001) n_inplace = 1;
							else if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end begin n_rr_a = {1'b0, rdd_rn}; n_rr_a_v = 1; n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; n_next = NX_PREGS; end end
							else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end end
							else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
						end
						else if (rd_ir[8] && rdd_mode[2:1] == 2'b00) begin
							case (rdd_op8_6[1:0])
								2'b00: begin
									// SBCD
									begin n_alu_op = `AP040_ALU_SBCD; n_alu_op_v = 1; end
									begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
									begin n_p_ssize = `AP040_SZ_B; n_p_ssize_v = 1; end begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
									if (!rdd_mode[0]) begin
										begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end
										begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
									end
									else begin
										begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = 3'b100; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
										begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = 3'b100; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
										begin n_p_rmw = 1; n_p_rmw_v = 1; end
									end
									n_next = NX_PSTART;
								end
								2'b01: begin
									// PACK
									begin n_exec_kind = EK_PACK; n_exec_kind_v = 1; end
									begin n_p_flags = 0; n_p_flags_v = 1; end
									begin n_p_ssize = `AP040_SZ_W; n_p_ssize_v = 1; end begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
									if (!rdd_mode[0]) begin
										begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end
										begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
									end
									else begin
										begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = 3'b100; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
										begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = 3'b100; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
									end
									begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end
								end
								2'b10: begin
									// UNPK
									begin n_exec_kind = EK_UNPK; n_exec_kind_v = 1; end
									begin n_p_flags = 0; n_p_flags_v = 1; end
									begin n_p_ssize = `AP040_SZ_B; n_p_ssize_v = 1; end begin n_p_dsize = `AP040_SZ_W; n_p_dsize_v = 1; end
									if (!rdd_mode[0]) begin
										begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end
										begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
									end
									else begin
										begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = 3'b100; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
										begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = 3'b100; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
									end
									begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end
								end
								default: n_inplace = 1;
							endcase
						end
						else begin
							// OR
							begin n_alu_op = `AP040_ALU_OR; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
							if (!rd_ir[8]) begin
								// <ea> OR Dn -> Dn
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
								if (rdd_mode == 3'b001) n_inplace = 1;
								else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end end
								else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
							end
							else begin
								// Dn OR <ea> -> <ea>
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_reg9}; n_p_sreg_v = 1; end begin n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; end
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_p_rmw = 1; n_p_rmw_v = 1; end
								begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
								// The register-to-EA OR form is memory-only.
								// Mode 000/001 combinations are reserved for the
								// SBCD/PACK/UNPK subfamily above.
								if (rdd_mode < 3'b010 ||
								    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
								else n_next = NX_PSTART;
							end
						end
					end

					//------------------------------------ 0x9/0xD: SUB/ADD
					4'h9, 4'hD: begin : dec_addsub
						reg is_add;
						is_add = (rd_ir_hi == 4'hD);
						if (rd_valid) begin end
						else if (rdd_op8_6 == 3'b011 || rdd_op8_6 == 3'b111) begin
							// ADDA/SUBA
							begin n_alu_op = is_add ? `AP040_ALU_ADD : `AP040_ALU_SUB; n_alu_op_v = 1; end
							begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
							begin n_p_ssize = rdd_op8_6[2] ? `AP040_SZ_L : `AP040_SZ_W; n_p_ssize_v = 1; end
							begin n_p_sextw = !rdd_op8_6[2]; n_p_sextw_v = 1; end
							begin n_p_flags = 0; n_p_flags_v = 1; end
							begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b1, rdd_reg9}; n_p_dreg_v = 1; end
							if (rdd_ea_is_imm) begin
								begin n_p_src = SK_IMM; n_p_src_v = 1; end
								begin n_immn = rdd_op8_6[2] ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end
							end
							else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
						end
						else if (rd_ir[8] && rdd_mode[2:1] == 2'b00 && rdd_std_size != 2'b11) begin
							// Predecrement ADDX/SUBX; register form is shared.
							begin n_alu_op = is_add ? `AP040_ALU_ADDX : `AP040_ALU_SUBX; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
							begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = 3'b100; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
							begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = 3'b100; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
							begin n_p_rmw = 1; n_p_rmw_v = 1; end
							n_next = NX_PSTART;
						end
						else begin
							begin n_alu_op = is_add ? `AP040_ALU_ADD : `AP040_ALU_SUB; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
							if (!rd_ir[8]) begin
								// <ea> op Dn -> Dn
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
								if (rdd_mode == 3'b001 && rdd_std_size == `AP040_SZ_B) n_inplace = 1;
								else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end end
								else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
							end
							else begin
								// Dn op <ea> -> <ea>
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_reg9}; n_p_sreg_v = 1; end begin n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; end
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_p_rmw = 1; n_p_rmw_v = 1; end
								begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
								// The register-to-EA ADD/SUB form is memory-only;
								// register-direct encodings belong to ADDX/SUBX.
								if (rdd_mode < 3'b010 ||
								    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
								else n_next = NX_PSTART;
							end
						end
					end

					//---------------------------------------------- 0xA: A-line
					4'hA: n_inplace = 1;

					//---------------------------------- 0xB: CMP/CMPA/EOR/CMPM
					4'hB: begin
						if (rd_valid) begin end
						else if (rdd_op8_6 == 3'b011 || rdd_op8_6 == 3'b111) begin
							// CMPA
							begin n_alu_op = `AP040_ALU_CMP; n_alu_op_v = 1; end
							begin n_op_size = `AP040_SZ_L; n_op_size_v = 1; end
							begin n_p_ssize = rdd_op8_6[2] ? `AP040_SZ_L : `AP040_SZ_W; n_p_ssize_v = 1; end
							begin n_p_sextw = !rdd_op8_6[2]; n_p_sextw_v = 1; end
							begin n_p_wbsup = 1; n_p_wbsup_v = 1; end
							begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b1, rdd_reg9}; n_p_dreg_v = 1; end
							if (rdd_ea_is_imm) begin
								begin n_p_src = SK_IMM; n_p_src_v = 1; end
								begin n_immn = rdd_op8_6[2] ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end
							end
							else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
						end
						else if (!rd_ir[8]) begin
							// CMP <ea>,Dn
							begin n_alu_op = `AP040_ALU_CMP; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end
							begin n_p_wbsup = 1; n_p_wbsup_v = 1; end
							begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
							if (rdd_mode == 3'b001 && rdd_std_size == `AP040_SZ_B) n_inplace = 1;
							else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end end
							else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
						end
						else if (rdd_mode == 3'b001) begin
							// CMPM (Ay)+,(Ax)+
							begin n_alu_op = `AP040_ALU_CMP; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
							begin n_p_wbsup = 1; n_p_wbsup_v = 1; end
							begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = 3'b011; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
							begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = 3'b011; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
							begin n_p_rmw = 1; n_p_rmw_v = 1; end
							n_next = NX_PSTART;
						end
						else begin
							// EOR Dn,<ea>
							begin n_alu_op = `AP040_ALU_EOR; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
							begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_reg9}; n_p_sreg_v = 1; end begin n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; end
							if (rdd_dst_not_alt) n_inplace = 1;
							else begin
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_p_rmw = 1; n_p_rmw_v = 1; end
								begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
								n_next = NX_PSTART;
							end
						end
					end

					//------------------------------------ 0xC: AND/MUL/EXG
					4'hC: begin
						if (rd_valid) begin end
						else if (rdd_op8_6 == 3'b011 || rdd_op8_6 == 3'b111) begin
							// MULU.W / MULS.W
							begin n_exec_kind = EK_MD_W; n_exec_kind_v = 1; end
							begin n_md_isdiv = 0; n_md_isdiv_v = 1; end
							begin n_md_sign = rdd_op8_6[2]; n_md_sign_v = 1; end
							begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
							begin n_p_ssize = `AP040_SZ_W; n_p_ssize_v = 1; end
							begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
							if (rdd_mode == 3'b001) n_inplace = 1;
							else if (rdd_mode == 3'b000) begin begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end begin n_rr_a = {1'b0, rdd_rn}; n_rr_a_v = 1; n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; n_next = NX_PREGS; end end
							else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = 2'd1; n_next = NX_IMMF_PSTART; end end
							else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
						end
						else if (rd_ir[8] && rdd_op8_6[1:0] == 2'b00 && rdd_mode[2:1] == 2'b00) begin
							// ABCD
							begin n_alu_op = `AP040_ALU_ABCD; n_alu_op_v = 1; end
							begin n_op_size = `AP040_SZ_B; n_op_size_v = 1; end
							begin n_p_ssize = `AP040_SZ_B; n_p_ssize_v = 1; end begin n_p_dsize = `AP040_SZ_B; n_p_dsize_v = 1; end
							if (!rdd_mode[0]) begin
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_rn}; n_p_sreg_v = 1; end
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
							end
							else begin
								begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = 3'b100; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_dst_mode_r = 3'b100; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_reg9; n_dst_rn_r_v = 1; end
								begin n_p_rmw = 1; n_p_rmw_v = 1; end
							end
							n_next = NX_PSTART;
						end
						else if (rd_ir[8] && (rdd_op8_6[1:0] == 2'b01) && rdd_mode[2:1] == 2'b00) begin
							// EXG Dn,Dn (mode 000) / EXG An,An (mode 001)
							begin n_rr_a = {rdd_mode[0], rdd_reg9}; n_rr_a_v = 1; end
							begin n_rr_b = {rdd_mode[0], rdd_rn}; n_rr_b_v = 1; end
							n_inplace = 1;
						end
						else if (rd_ir[8] && rdd_op8_6[1:0] == 2'b10 && rdd_mode == 3'b001) begin
							// EXG Dn,An
							begin n_rr_a = {1'b0, rdd_reg9}; n_rr_a_v = 1; end
							begin n_rr_b = {1'b1, rdd_rn}; n_rr_b_v = 1; end
							n_inplace = 1;
						end
						else begin
							// AND
							begin n_alu_op = `AP040_ALU_AND; n_alu_op_v = 1; end
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_ssize = rdd_std_size; n_p_ssize_v = 1; end begin n_p_dsize = rdd_std_size; n_p_dsize_v = 1; end
							if (!rd_ir[8]) begin
								begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_reg9}; n_p_dreg_v = 1; end
								if (rdd_mode == 3'b001) n_inplace = 1;
								else if (rdd_ea_is_imm) begin begin n_p_src = SK_IMM; n_p_src_v = 1; end begin n_immn = (rdd_std_size == `AP040_SZ_L) ? 2'd2 : 2'd1; n_next = NX_IMMF_PSTART; end end
								else begin begin n_p_src = SK_MEM; n_p_src_v = 1; end begin n_src_mode_r = rdd_mode; n_src_mode_r_v = 1; end begin n_src_rn_r = rdd_rn; n_src_rn_r_v = 1; end n_next = NX_PSTART; end
							end
							else begin
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_reg9}; n_p_sreg_v = 1; end begin n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; end
								begin n_p_dst = DK_MEM; n_p_dst_v = 1; end begin n_p_rmw = 1; n_p_rmw_v = 1; end
								begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
								// The register-to-EA AND form is memory-only.
								// Mode 000 combinations not claimed by ABCD/EXG
								// are reserved, rather than AND Dn,Dn aliases.
								if (rdd_mode < 3'b010 ||
								    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
								else n_next = NX_PSTART;
							end
						end
					end

					//---------------------------------------- 0xE: shifts
					4'hE: begin
						if (rd_ir[7:6] == 2'b11) begin
							if (rd_ir[11]) begin
								// bitfield group; ext word first
								// modify ops need an alterable EA
								if (rdd_mode == 3'b001 || rdd_mode == 3'b011 ||
								    rdd_mode == 3'b100 || rdd_ea_is_imm) n_inplace = 1;
								else if ((rdd_mode == 3'b111 && rdd_rn > 3'b001) &&
								         (rd_ir[10:8] == 3'd2 || rd_ir[10:8] == 3'd4 ||
								          rd_ir[10:8] == 3'd6 || rd_ir[10:8] == 3'd7)) n_inplace = 1;
								else n_inplace = 1;
							end
							else begin
								// memory shift by one, word
								begin n_exec_kind = EK_SHIFT; n_exec_kind_v = 1; end
								begin n_sh_rox = (rd_ir[10:9] == 2'b10); n_sh_rox_v = 1; end
								case (rd_ir[10:9])
									2'b00: begin n_alu_op = rd_ir[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1; n_alu_op_v = 1; end
									2'b01: begin n_alu_op = rd_ir[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1; n_alu_op_v = 1; end
									2'b10: begin n_alu_op = rd_ir[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1; n_alu_op_v = 1; end
									default: begin n_alu_op = rd_ir[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1; n_alu_op_v = 1; end
								endcase
								begin n_op_size = `AP040_SZ_W; n_op_size_v = 1; end
								begin n_p_dsize = `AP040_SZ_W; n_p_dsize_v = 1; end
								begin n_p_src = SK_NONE; n_p_src_v = 1; end   // count of one
								begin n_p_rmw = 1; n_p_rmw_v = 1; end
								// Memory shifts require a memory-alterable EA: Dn/An
								// direct and all program-space encodings are illegal.
								if (rdd_mode < 3'b010 ||
								    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
								else begin
									begin n_p_dst = DK_MEM; n_p_dst_v = 1; end
									begin n_dst_mode_r = rdd_mode; n_dst_mode_r_v = 1; end begin n_dst_rn_r = rdd_rn; n_dst_rn_r_v = 1; end
									n_next = NX_PSTART;
								end
							end
						end
						else begin
							// register shift
							begin n_exec_kind = EK_SHIFT; n_exec_kind_v = 1; end
							begin n_sh_rox = (rd_ir[4:3] == 2'b10); n_sh_rox_v = 1; end
							case (rd_ir[4:3])
								2'b00: begin n_alu_op = rd_ir[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1; n_alu_op_v = 1; end
								2'b01: begin n_alu_op = rd_ir[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1; n_alu_op_v = 1; end
								2'b10: begin n_alu_op = rd_ir[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1; n_alu_op_v = 1; end
								default: begin n_alu_op = rd_ir[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1; n_alu_op_v = 1; end
							endcase
							begin n_op_size = rdd_std_size; n_op_size_v = 1; end
							begin n_p_dst = DK_REG; n_p_dst_v = 1; end begin n_p_dreg = {1'b0, rdd_rn}; n_p_dreg_v = 1; end
							if (rd_ir[5]) begin
								begin n_p_src = SK_REG; n_p_src_v = 1; end begin n_p_sreg = {1'b0, rdd_reg9}; n_p_sreg_v = 1; end begin n_rr_b = {1'b0, rdd_reg9}; n_rr_b_v = 1; end
							end
							else begin
								begin n_p_src = SK_IMPL; n_p_src_v = 1; end
								begin n_src_val = {26'd0, (rdd_reg9 == 3'd0) ? 6'd8 : {3'd0, rdd_reg9}}; n_src_val_v = 1; end
							end
							n_next = NX_PSTART;
						end
					end

					//------------------------------------------ 0xF: 040 group
					default: begin
						if (rd_ir[11:8] == 4'h4) begin
							// CINV/CPUSH: write-through caches hold no dirty
							// data, so both invalidate the selected caches
							// (scope is widened to ALL, which is safe)
							// Scope bit patterns 000 and 100 are unassigned
							// F-line encodings.  Classify them before privilege.
							if (rd_ir[4:3] == 2'b00) n_inplace = 1;
							else if (!sr_s) n_inplace = 1;
							else begin
								n_inplace = 1;
								n_inplace = 1;
								n_inplace = 1;
								n_inplace = 1;
								n_inplace = 1;
							end
						end
						else if (rd_ir[11:8] == 4'h5) begin
							if (rd_ir[7:5] == 3'b000) begin
								// PFLUSH group
								if (!sr_s) n_inplace = 1;
								else begin
									n_inplace = 1;
									n_inplace = 1;
									if (rd_ir[4]) begin
										// PFLUSHAN / PFLUSHA
										n_inplace = 1;
									end
									else begin
										begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end
										n_inplace = 1;
									end
								end
							end
							else if (rd_ir[7:6] == 2'b01) begin
								// PTEST
								// Only F548..F54F and F568..F56F are PTEST;
								// the rest of this quadrant is unassigned F-line.
								if (rd_ir[4:3] != 2'b01) n_inplace = 1;
								else if (!sr_s) n_inplace = 1;
								else begin
									begin n_rr_a = {1'b1, rdd_rn}; n_rr_a_v = 1; end
									n_inplace = 1;
								end
							end
							else n_inplace = 1;
						end
						else if (rd_ir[11:8] == 4'h2) begin
							// FPU coprocessor space (cpid 1)
							if (AP040_HAS_FPU == 0)
								n_inplace = 1;
							else case (rd_ir[7:6])
								2'b00: begin                         // general
									// Mode-7 registers 5..7 are reserved for every
									// coprocessor command.  Reject them before fetching
									// an extension word; malformed primary opcodes take
									// the F-line vector, independent of the next word.
									if (rdd_mode == 3'b111 && rdd_rn > 3'b100)
										n_inplace = 1;
									else n_inplace = 1;
								end
								2'b01: begin                         // FScc/FDBcc/FTRAPcc
									// As in the general command space, mode-7
									// registers 5..7 are primary-word F-line errors.
									if (rdd_mode == 3'b111 && rdd_rn > 3'b100)
										n_inplace = 1;
									else n_inplace = 1;
								end
								2'b10:   n_inplace = 1;      // FBcc.W
								default: n_inplace = 1;      // FBcc.L
							endcase
						end
						else if (rd_ir[11:8] == 4'h3) begin
							// FSAVE/FRESTORE state-frame model: NULL, IDLE and the
							// revision-$41 unimplemented-instruction frame are
							// implemented.  A true BUSY arithmetic-exception frame
							// remains outside this non-pipelined FPU's state model.
							if (rd_ir[7:6] == 2'b00) begin
								// FSAVE: control alterable or -(An)
								// Malformed coprocessor EAs are F-line faults and
								// are classified before privilege.
								if (rdd_mode < 3'b010 || rdd_mode == 3'b011 ||
								    (rdd_mode == 3'b111 && rdd_rn > 3'b001)) n_inplace = 1;
								else if (!sr_s) n_inplace = 1;
								else n_inplace = 1;
							end
							else if (rd_ir[7:6] == 2'b01) begin
								// FRESTORE: control, (An)+ or PC relative
								if (rdd_mode < 3'b010 || rdd_mode == 3'b100 ||
								    (rdd_mode == 3'b111 && rdd_rn >= 3'b100)) n_inplace = 1;
								else if (!sr_s) n_inplace = 1;
								else n_inplace = 1;
							end
							else n_inplace = 1;
						end
						else if (rd_ir[11:8] == 4'h6 && rd_ir[7:5] == 3'b000) begin
							// MOVE16 with absolute long operand
							n_inplace = 1;
							n_inplace = 1;
						end
						else if (rd_ir[11:8] == 4'h6 && rd_ir[7:3] == 5'b00100) begin
							// MOVE16 (Ax)+,(Ay)+
							n_inplace = 1;
							n_inplace = 1;
						end
						else n_inplace = 1;
					end
				endcase
end

`ifdef DECODE_CHECK
// Equivalence check: the record computed in the S_DECODE cycle (rd_ir == ir)
// against the registers the body wrote, one cycle later.  Descriptor-covered
// opcodes are skipped (dispatch_reg_decode overrides the body there).
reg        chk_v; reg [15:0] chk_ir; reg [31:0] chk_pc; reg [2:0] chk_next;
reg [2:0]    chk_p_src; reg chk_p_src_v;
reg [2:0]    chk_p_dst; reg chk_p_dst_v;
reg          chk_p_rmw; reg chk_p_rmw_v;
reg          chk_p_wbsup; reg chk_p_wbsup_v;
reg          chk_p_flags; reg chk_p_flags_v;
reg          chk_p_sextw; reg chk_p_sextw_v;
reg          chk_p_dst_mem_bit; reg chk_p_dst_mem_bit_v;
reg [3:0]    chk_exec_kind; reg chk_exec_kind_v;
reg [1:0]    chk_op_size; reg chk_op_size_v;
reg [5:0]    chk_alu_op; reg chk_alu_op_v;
reg [3:0]    chk_p_dreg; reg chk_p_dreg_v;
reg [1:0]    chk_p_dsize; reg chk_p_dsize_v;
reg [1:0]    chk_p_ssize; reg chk_p_ssize_v;
reg [3:0]    chk_p_sreg; reg chk_p_sreg_v;
reg [2:0]    chk_dst_rn_r; reg chk_dst_rn_r_v;
reg [2:0]    chk_dst_mode_r; reg chk_dst_mode_r_v;
reg [2:0]    chk_src_rn_r; reg chk_src_rn_r_v;
reg [2:0]    chk_src_mode_r; reg chk_src_mode_r_v;
reg [3:0]    chk_rr_a; reg chk_rr_a_v;
reg [3:0]    chk_rr_b; reg chk_rr_b_v;
reg [31:0]   chk_src_val; reg chk_src_val_v;
reg          chk_sh_rox; reg chk_sh_rox_v;
reg          chk_md_isdiv; reg chk_md_isdiv_v;
reg          chk_md_sign; reg chk_md_sign_v;
reg          chk_lk_cyc; reg chk_lk_cyc_v;
always @(posedge clk) begin
	chk_v <= 0;
	if (ce && state == S_DECODE && !rd_valid && !n_inplace) begin
		chk_v <= 1; chk_ir <= ir; chk_pc <= pc_i; chk_next <= n_next;
		chk_p_src <= n_p_src; chk_p_src_v <= n_p_src_v;
		chk_p_dst <= n_p_dst; chk_p_dst_v <= n_p_dst_v;
		chk_p_rmw <= n_p_rmw; chk_p_rmw_v <= n_p_rmw_v;
		chk_p_wbsup <= n_p_wbsup; chk_p_wbsup_v <= n_p_wbsup_v;
		chk_p_flags <= n_p_flags; chk_p_flags_v <= n_p_flags_v;
		chk_p_sextw <= n_p_sextw; chk_p_sextw_v <= n_p_sextw_v;
		chk_p_dst_mem_bit <= n_p_dst_mem_bit; chk_p_dst_mem_bit_v <= n_p_dst_mem_bit_v;
		chk_exec_kind <= n_exec_kind; chk_exec_kind_v <= n_exec_kind_v;
		chk_op_size <= n_op_size; chk_op_size_v <= n_op_size_v;
		chk_alu_op <= n_alu_op; chk_alu_op_v <= n_alu_op_v;
		chk_p_dreg <= n_p_dreg; chk_p_dreg_v <= n_p_dreg_v;
		chk_p_dsize <= n_p_dsize; chk_p_dsize_v <= n_p_dsize_v;
		chk_p_ssize <= n_p_ssize; chk_p_ssize_v <= n_p_ssize_v;
		chk_p_sreg <= n_p_sreg; chk_p_sreg_v <= n_p_sreg_v;
		chk_dst_rn_r <= n_dst_rn_r; chk_dst_rn_r_v <= n_dst_rn_r_v;
		chk_dst_mode_r <= n_dst_mode_r; chk_dst_mode_r_v <= n_dst_mode_r_v;
		chk_src_rn_r <= n_src_rn_r; chk_src_rn_r_v <= n_src_rn_r_v;
		chk_src_mode_r <= n_src_mode_r; chk_src_mode_r_v <= n_src_mode_r_v;
		chk_rr_a <= n_rr_a; chk_rr_a_v <= n_rr_a_v;
		chk_rr_b <= n_rr_b; chk_rr_b_v <= n_rr_b_v;
		chk_src_val <= n_src_val; chk_src_val_v <= n_src_val_v;
		chk_sh_rox <= n_sh_rox; chk_sh_rox_v <= n_sh_rox_v;
		chk_md_isdiv <= n_md_isdiv; chk_md_isdiv_v <= n_md_isdiv_v;
		chk_md_sign <= n_md_sign; chk_md_sign_v <= n_md_sign_v;
		chk_lk_cyc <= n_lk_cyc; chk_lk_cyc_v <= n_lk_cyc_v;
	end
	if (chk_v && ce) begin
		if (chk_p_src_v && chk_p_src !== p_src) $display("DECODE_CHECK pc=%h ir=%h field p_src record=%h body=%h", chk_pc, chk_ir, chk_p_src, p_src);
		if (chk_p_dst_v && chk_p_dst !== p_dst) $display("DECODE_CHECK pc=%h ir=%h field p_dst record=%h body=%h", chk_pc, chk_ir, chk_p_dst, p_dst);
		if (chk_p_rmw_v && chk_p_rmw !== p_rmw) $display("DECODE_CHECK pc=%h ir=%h field p_rmw record=%h body=%h", chk_pc, chk_ir, chk_p_rmw, p_rmw);
		if (chk_p_wbsup_v && chk_p_wbsup !== p_wbsup) $display("DECODE_CHECK pc=%h ir=%h field p_wbsup record=%h body=%h", chk_pc, chk_ir, chk_p_wbsup, p_wbsup);
		if (chk_p_flags_v && chk_p_flags !== p_flags) $display("DECODE_CHECK pc=%h ir=%h field p_flags record=%h body=%h", chk_pc, chk_ir, chk_p_flags, p_flags);
		if (chk_p_sextw_v && chk_p_sextw !== p_sextw) $display("DECODE_CHECK pc=%h ir=%h field p_sextw record=%h body=%h", chk_pc, chk_ir, chk_p_sextw, p_sextw);
		if (chk_p_dst_mem_bit_v && chk_p_dst_mem_bit !== p_dst_mem_bit) $display("DECODE_CHECK pc=%h ir=%h field p_dst_mem_bit record=%h body=%h", chk_pc, chk_ir, chk_p_dst_mem_bit, p_dst_mem_bit);
		if (chk_exec_kind_v && chk_exec_kind !== exec_kind) $display("DECODE_CHECK pc=%h ir=%h field exec_kind record=%h body=%h", chk_pc, chk_ir, chk_exec_kind, exec_kind);
		if (chk_op_size_v && chk_op_size !== op_size) $display("DECODE_CHECK pc=%h ir=%h field op_size record=%h body=%h", chk_pc, chk_ir, chk_op_size, op_size);
		if (chk_alu_op_v && chk_alu_op !== alu_op) $display("DECODE_CHECK pc=%h ir=%h field alu_op record=%h body=%h", chk_pc, chk_ir, chk_alu_op, alu_op);
		if (chk_p_dreg_v && chk_p_dreg !== p_dreg) $display("DECODE_CHECK pc=%h ir=%h field p_dreg record=%h body=%h", chk_pc, chk_ir, chk_p_dreg, p_dreg);
		if (chk_p_dsize_v && chk_p_dsize !== p_dsize) $display("DECODE_CHECK pc=%h ir=%h field p_dsize record=%h body=%h", chk_pc, chk_ir, chk_p_dsize, p_dsize);
		if (chk_p_ssize_v && chk_p_ssize !== p_ssize) $display("DECODE_CHECK pc=%h ir=%h field p_ssize record=%h body=%h", chk_pc, chk_ir, chk_p_ssize, p_ssize);
		if (chk_p_sreg_v && chk_p_sreg !== p_sreg) $display("DECODE_CHECK pc=%h ir=%h field p_sreg record=%h body=%h", chk_pc, chk_ir, chk_p_sreg, p_sreg);
		if (chk_dst_rn_r_v && chk_dst_rn_r !== dst_rn_r) $display("DECODE_CHECK pc=%h ir=%h field dst_rn_r record=%h body=%h", chk_pc, chk_ir, chk_dst_rn_r, dst_rn_r);
		if (chk_dst_mode_r_v && chk_dst_mode_r !== dst_mode_r) $display("DECODE_CHECK pc=%h ir=%h field dst_mode_r record=%h body=%h", chk_pc, chk_ir, chk_dst_mode_r, dst_mode_r);
		if (chk_src_rn_r_v && chk_src_rn_r !== src_rn_r) $display("DECODE_CHECK pc=%h ir=%h field src_rn_r record=%h body=%h", chk_pc, chk_ir, chk_src_rn_r, src_rn_r);
		if (chk_src_mode_r_v && chk_src_mode_r !== src_mode_r) $display("DECODE_CHECK pc=%h ir=%h field src_mode_r record=%h body=%h", chk_pc, chk_ir, chk_src_mode_r, src_mode_r);
		if (chk_rr_a_v && chk_rr_a !== rr_a) $display("DECODE_CHECK pc=%h ir=%h field rr_a record=%h body=%h", chk_pc, chk_ir, chk_rr_a, rr_a);
		if (chk_rr_b_v && chk_rr_b !== rr_b && !(chk_next == NX_IMMREG && state == S_IMMF)) $display("DECODE_CHECK pc=%h ir=%h field rr_b record=%h body=%h", chk_pc, chk_ir, chk_rr_b, rr_b);
		if (chk_src_val_v && chk_src_val !== src_val) $display("DECODE_CHECK pc=%h ir=%h field src_val record=%h body=%h", chk_pc, chk_ir, chk_src_val, src_val);
		if (chk_sh_rox_v && chk_sh_rox !== sh_rox) $display("DECODE_CHECK pc=%h ir=%h field sh_rox record=%h body=%h", chk_pc, chk_ir, chk_sh_rox, sh_rox);
		if (chk_md_isdiv_v && chk_md_isdiv !== md_isdiv) $display("DECODE_CHECK pc=%h ir=%h field md_isdiv record=%h body=%h", chk_pc, chk_ir, chk_md_isdiv, md_isdiv);
		if (chk_md_sign_v && chk_md_sign !== md_sign) $display("DECODE_CHECK pc=%h ir=%h field md_sign record=%h body=%h", chk_pc, chk_ir, chk_md_sign, md_sign);
		if (chk_lk_cyc_v && chk_lk_cyc !== lk_cyc) $display("DECODE_CHECK pc=%h ir=%h field lk_cyc record=%h body=%h", chk_pc, chk_ir, chk_lk_cyc, lk_cyc);
		if (chk_next == NX_PSTART && state != S_PIPE_START) $display("DECODE_CHECK pc=%h ir=%h next PSTART but state=%0d", chk_pc, chk_ir, state);
		if (chk_next == NX_PREGS && state != S_PIPE_REGS) $display("DECODE_CHECK pc=%h ir=%h next PREGS but state=%0d", chk_pc, chk_ir, state);
		if (chk_next == NX_IMMF_PSTART && state != S_PIPE_START && state != S_IMMF) $display("DECODE_CHECK pc=%h ir=%h next IMMF_PSTART but state=%0d", chk_pc, chk_ir, state);
		if (chk_next == NX_IMMREG && state != S_PIPE_REGS && state != S_IMMF) $display("DECODE_CHECK pc=%h ir=%h next IMMREG but state=%0d", chk_pc, chk_ir, state);
		if (chk_next == NX_NONE) $display("DECODE_CHECK pc=%h ir=%h record has no next (in-place expected) state=%0d", chk_pc, chk_ir, state);
	end
end
`endif

always @(posedge clk) begin
	// Combinational carriers, valid only inside this block: the fetch queue's
	// fill engine runs after the case statement and has to see what the case
	// did to the memory port and to the queue in the same cycle.
	epf_issue   = 0;
	epf_flushed = 0;
	epf_pop     = 2'd0;
	epf_fillw   = 4'd0;
	rd_queue_pop = 0;
	retire_req = 0;
	dgo = 0;
	pgo = 0;
	sgo = 0;
	rfw_now = 0; rfw_now_a = 4'd0;
	igo = 0; igo_n = 2'd0; igo_ret = 8'd0;
	xgo = 0; xgo_vec = 8'd0; xgo_fmt = 4'd0; xgo_spc = 32'd0; xgo_addr = 32'd0;
	mgo = 0; mgo_wr = 0; mgo_sz = 2'd0; mgo_ret = 8'd0; mgo_a = 32'd0; mgo_d = 32'd0;
	brf_seed_n  = 4'd0;
	brf_seed_req = 0;
	brf_seed_a  = 5'bxxxxx; // Unused unless brf_seed_req assigns a valid target.

	if (!nreset) begin
		state <= S_START;
		pc <= 0; pc_i <= 0;
		sr <= `AP040_SR_RESET;
		vbr <= 0; cacr <= 0;
		sfc <= 0; dfc <= 0;
		if (!mmu_reset_seen) begin
			tc <= 0;
			itt0 <= 0; itt1 <= 0; dtt0 <= 0; dtt1 <= 0;
			mmusr <= 0; urp <= 0; srp <= 0;
		end
		else begin
			// RSTI clears only the translation-enable bits.  TC.P and all
			// other MMU register fields retain their previous values.
			tc   <= tc   & 32'h0000_4000;
			itt0 <= itt0 & 32'hFFFF_7FFF;
			itt1 <= itt1 & 32'hFFFF_7FFF;
			dtt0 <= dtt0 & 32'hFFFF_7FFF;
			dtt1 <= dtt1 & 32'hFFFF_7FFF;
		end
		mmu_reset_seen <= 1;
		ir <= 0;
		perf_dispatch_toggle <= 0;
		mem_req <= 0; mem_write <= 0; mem_instr_q <= 0; ifr_req <= 0; ifr_addr <= 0; ifr_size <= 0; ifr_fc <= 0;
		mem_size <= `AP040_SZ_W; mem_addr_q <= 0; mem_wdata <= 0;
		fc_r <= `AP040_FC_SUPER_DATA;
		rf_we <= 0; rf_waddr <= 0; rf_wdata <= 0;
		fpu_req <= 0; fpu_class <= 0; fpu_opm <= 0; fpu_fmt <= 0;
		fpu_srcr <= 0; fpu_dstr <= 0; fpb <= 0;
		fpu_bg <= 0; fpu_pend_exc <= 0; fpu_pend_vec <= 0;
		fpu_pendcap <= 0; fp_restore_grs <= 0; fp_restore_wbte15 <= 0;
		fp_restore_wbt <= 0; fp_restore_fpiar <= 0; fp_restore_busy <= 0;
		fpb_n <= 0;
		fpu_crsel <= 0; fpu_crwe <= 0; fpu_crwd <= 0; fpu_iawe <= 0;
		fpu_bsun <= 0;
		fpu_fmsel <= 0; fpu_fmwe <= 0; fpu_fmwd <= 0; fpu_rst <= 0;
		fpu_fsave_ack <= 0; fpu_frestore_idle <= 0; fpu_frestore_unimp <= 0;
		fp_restore_cmd1 <= 0; fp_restore_cmd3 <= 0;
		fp_restore_stag <= 0; fp_restore_dtag <= 0; fp_restore_flags <= 0;
		fp_restore_fpt <= 0; fp_restore_et <= 0;
		fp_restore_cusavepc <= 0;
		fp_restore_et15 <= 0; fp_restore_fpt15 <= 0;
		fp_cnt <= 0; fp_nb <= 0; fp_st <= 0; fp_list <= 0; fp_mode <= 0;
		fp_st_epend <= 0; fp_st_evec <= 0;
		fp_rev <= 0; fp_lsb <= 0;
		fp_creg <= 0; fp_pred <= 0; fp_n <= 0;
		fp_ea_pd <= 0; fp_ea_pi <= 0; fp_adj <= 0; fp_ea_v <= 0;
		m_bidx <= 0; m_acc <= 0;
		rr_a <= 0; rr_b <= 0;
		aux_we <= 0; aux_sel <= 0; aux_wdata <= 0;
		md_start <= 0; md_isdiv <= 0; md_sign <= 0;
		md_a <= 0; md_hi <= 0; md_lo <= 0;
		alu_op <= 0; op_size <= 0;
		src_val <= 0; dst_val <= 0; dst_addr <= 0;
		sh_val <= 0; sh_fl <= 0; sh_cnt <= 0; sh_vacc <= 0; sh_rox <= 0;
		sh_any <= 0;
		r_imm_ret <= 0; r_ea_ret <= 0; r_m_ret <= 0;
		imm_n <= 0; m_issued <= 0; imm <= 0; x_ext <= 0;
		epf_count <= 0; epf_head <= 0; epf_fill <= 0;
		epf_base <= 0; epf_next <= 0; epf_super <= 0;
		epf_ftail <= 0; epf_armed <= 0; epf_pend <= 0;
		epf_pend_seed <= 0; brf_seed_ok <= 0; iline_log <= 0; iline_super <= 0;
		epf_pend_lw <= 0; epf_kill <= 0; epf_err <= 0; epf_brf <= 0;
		ras_sp <= 0;
		// Queue/refill payload is invalid while the count/valid controls below
		// are clear.  Do not reset it: payload reset muxes only consume FPGA
		// packing resources and the words are overwritten before becoming valid.
		brf_tag <= 0; brf_super <= 0; brf_valid <= 0;
		m_wr <= 0; m_size <= 0; m_addr_r <= 0; m_wdat <= 0; m_val <= 0;
		ea_mode <= 0; ea_rn <= 0; ea_size <= 0;
		ea_pcmode <= 0; ea_pcb <= 0; extw <= 0;
		ea_base_v <= 0; ea_idx_v <= 0; ea_mind <= 0;
		ea_post <= 0; ea_odl <= 0; ea_absl <= 0; ea_addr <= 0;
		p_src <= 0; p_dst <= 0; p_sreg <= 0; p_dreg <= 0;
		p_ssize <= 0; p_dsize <= 0;
		p_rmw <= 0; p_wbsup <= 0; p_flags <= 0; p_sextw <= 0;
		p_dst_mem_bit <= 0;
		exec_kind <= EK_ALU;
		src_mode_r <= 0; src_rn_r <= 0; dst_mode_r <= 0; dst_rn_r <= 0;
		exc_vec <= 0; exc_fmt <= 0; exc_spc <= 0; exc_addr <= 0; exc_sp <= 0;
		exc_is_irq <= 0; exc_pass2 <= 0; sr_saved <= 0; irq_lvl_l <= 0;
		texc_pend <= 0; texc_pc <= 0;
		flow_t0_pend <= 0; flow_t0_oldpc <= 0;
		rte_sr <= 0; rte_pc <= 0; ret_kind <= 0;
		br_base <= 0; br_tgt <= 0; br_long <= 0;
		mm_mask <= 0; mm_dir <= 0; mm_predec <= 0; mm_postinc <= 0;
		mm_size <= 0; mm_addr <= 0; mm_init_an <= 0; mm_reg <= 0;
		mm_base_ea <= 0; mm_base_pend <= 0; mm_base_val <= 0;
		mm_idx_pend <= 0; mm_idx_val <= 0; mm_idx_reg <= 0; mm_idx_en <= 0;
		mm_resume <= 0; mm_start_ea <= 0;
		aer_cm <= 0; aer_ea <= 0;
		mp_cnt <= 0; mp_idx <= 0; mp_dir <= 0; mp_addr <= 0; mp_val <= 0;
		t_a <= 0; t_b <= 0; srop_kind <= 0; srop_sr <= 0;
		mvc_dir <= 0; fc_ovr_v <= 0; fc_ovr <= 0;
		lk_cyc <= 0; aer_lk <= 0; aer_m16 <= 0; aer_tt <= 0; aer_wd <= 0;
		m16_form <= 0; m16_dst_rn <= 0; m16_src <= 0; m16_dst <= 0;
		m16_an <= 0; m16_idx <= 0; m16_rd_done <= 0;
		// m16buf is filled completely before S_M16_WR can consume it.
		rst_cnt <= 0;
		fault_r <= 0;
		nmi_ack_t <= 0;
		irq_ack_t <= 0;
		bf_off <= 0; bf_w <= 0; bf_addr <= 0; bf_bib <= 0; bf_span <= 0;
		bf_w1 <= 0; bf_w2 <= 0; bf_du <= 0; cas_dc <= 0;
		bf_t40 <= 0; bf_field <= 0; bf_ones <= 0; bf_maskl <= 0;
		in_exc <= 0;
		aer_fa <= 0; aer_sp <= 0; aer_wr <= 0;
		aer_sz <= 0; aer_tm <= 0; aer_idx <= 0; aer_bus <= 0; aer_ma <= 0;
		u0_v <= 0; u1_v <= 0;
		u0_reg <= 0; u1_reg <= 0; u0_old <= 0; u1_old <= 0;
		pt_req <= 0; pt_write <= 0; pt_addr <= 0;
		pf_req <= 0; pf_mode <= 0; pf_addr <= 0;
		cinv_req <= 0; cinv_ic <= 0; cinv_dc <= 0;
		tr_t1 <= 0; tr_t0 <= 0; t0_force <= 0;
	end
	else if (ce) begin
		rf_we <= 0;
		aux_we <= 0;
		md_start <= 0;
		fpu_req <= 0; fpu_crwe <= 0; fpu_fmwe <= 0; fpu_iawe <= 0;
		fpu_bsun <= 0;
		// released-operation completion: results retire inside the FPU;
		// an enabled arithmetic exception becomes a pending pre-instruction
		// exception for the next FPU dispatch point
		fpu_pendcap <= 0;   // default BEFORE the retire branch's set
		if (fpu_bg && fpu_done) fpu_bg <= 0;
		if (fpu_bg && fpu_exc_req) begin
			fpu_bg <= 0;
			fpu_pend_exc <= 1;
			fpu_pend_vec <= fpu_exc_vec;
			// let the FPU prepare the FSAVE e1 frame from its shadow
			fpu_pendcap <= 1;
		end
		fpu_rst <= 0;
		fpu_fsave_ack <= 0;
		fpu_frestore_idle <= 0;
		fpu_frestore_unimp <= 0;
		if (mem_ack) mem_req <= 0;
		if (ifr_ack) ifr_req <= 0;
		i_ack_d <= i_ack;
		// P177: the return-address stack (see mem_ihint_addr)
		if ((state == S_MWR) && d_ack && ((r_m_ret == S_JSR2) || (r_m_ret == S_BSR_PUSH))) begin
			ras[ras_sp] <= m_wdat;
			ras_sp <= ras_sp + 3'd1;
		end
		else if ((state == S_MRD) && d_ack && (r_m_ret == S_RET2) && ((ret_kind == RK_RTS) || (ret_kind == RK_RTD)))
			ras_sp <= ras_sp - 3'd1;
		// Every acknowledged instruction fetch, whether the queue engine's,
		// a redirect's or the exception prefetch's own, names the line the
		// cache will offer next cycle: record it and its context here, not
		// on any one issuer's path.  Only a queue fetch that was neither
		// killed nor flushed may let the offer replace the refill sector.
		if (i_ack) begin
			iline_log <= ifr_addr[31:4];    // the fetch's own registers (P171)
			iline_super <= ifr_fc[2];
			brf_seed_ok <= epf_pend ? (epf_pend_seed && !epf_kill && !epf_flushed)
			                        : 1'b1;
		end

		//-------------------------------------------------- line sideband
		// The cache offers the whole line of the last instruction hit one
		// cycle after acknowledging it.  Take every word from the fill tail
		// to the end of the line that fits in the ring, so the stream runs
		// ahead by up to eight words without another bus request.  This
		// runs BEFORE the state case and uses registered state only: the
		// word count must not hang off this cycle's pop/flush decisions
		// (that path ran through the ALU and missed timing by 7 ns).  A
		// flush or a redirect's own seed later in this cycle overrides the
		// ring writes by nonblocking order, and the merged bookkeeping below
		// drops the count/tail advance when epf_flushed is set.  Refused
		// while a queue fetch is outstanding or being acknowledged (its
		// return appends the same words) and when the tail is not in the
		// offered line.  The line also seeds the branch-refill sector
		// buffer; a same-cycle CPU-write invalidation below wins over it.
		if (mem_line_stb && epf_armed && !epf_pend && !ifr_ack &&
		    (epf_super == iline_super)) begin : line_offer
			reg [3:0] avail, room, n;
			integer i;
			brf_seed_ok <= 0;
			if (epf_ftail[31:4] == iline_log) begin
				avail = 4'd8 - {1'b0, epf_ftail[3:1]};
				room  = 4'd8 - epf_count;
				n = (avail < room) ? avail : room;
				begin : rotate_line
					reg [2:0] rotation, distance;
					reg [255:0] rotated;
					rotation = epf_ftail[3:1] - epf_fill;
					rotated = {mem_line_data, mem_line_data} << {rotation, 4'b0};
					for (i = 0; i < 8; i = i + 1) begin
						distance = i[2:0] - epf_fill;
						if ({1'b0, distance} < n)
							epf_data[i] <= rotated[(255 - 16*i) -: 16];
					end
				end
				epf_fillw = n;
				epf_issue = 1;
			end
			if ((brf_tag == iline_log[31:6] && brf_super == epf_super) ||
			    brf_seed_ok) begin
				for (i = 0; i < 4; i = i + 1)
					brf_data[{iline_log[5:4], i[1:0]}] <= mem_line_data[(127 - 32 * i) -: 32];
				if (brf_tag == iline_log[31:6] && brf_super == epf_super)
					brf_valid <= brf_valid | (32'h000000FF << {iline_log[5:4], 3'd0});
				else
					brf_valid <= 32'h000000FF << {iline_log[5:4], 3'd0};
				brf_tag <= iline_log[31:6];
				brf_super <= epf_super;
			end
		end

		// A completing CPU write that lands inside the fetch queue's
		// window [epf_next, epf_ftail) flushes it, so the rewritten
		// words are refetched.  Stronger than the architected CPUSH/CINV
		// requirement, and deliberate: the previous fetch-buffered core
		// booted DiagROM but not AmigaOS on hardware with exactly this
		// stale-prefetch hazard, invisible to CINV-disciplined tests.
		// Data ops never overlap an outstanding queue fetch (they hold
		// issue on epf_pend), so the flush here is pure bookkeeping.
		// Both ranges are logical addresses; ftail never wraps past the
		// page guard, so the plain compares suffice.
		if (d_ack && mem_write && epf_armed && epf_count != 4'd0 &&
		    (mem_addr_q + 32'd3 >= epf_next) && (mem_addr_q < epf_ftail))
			epf_flush;
		// The refill buffer now outlives the fetch window (it keeps the last
		// redirect's sector), so a CPU write into that sector invalidates
		// it directly: the same stale-prefetch hazard, one buffer further.
		if (d_ack && mem_write &&
		    ((mem_addr_q[31:6] == brf_tag) || ((mem_addr_q + 32'd3) >> 6 == brf_tag)))
			brf_valid <= 0;

		case (state)
			//------------------------------------------------------------ boot
			S_START: begin
				// Reset vector acquisition and the first target prefetch are part
				// of reset exception processing.  Any access fault before the
				// first opcode arrives is therefore a double bus fault.
				in_exc <= 1;
				m_addr_r <= `AP040_VEC_RESET_ISP; m_size <= `AP040_SZ_L;
				m_wr <= 0; m_issued <= 0; r_m_ret <= S_BOOT0;
				state <= S_MRD;
			end

			S_BOOT0: begin
				rfw(4'd15, m_val);
				mrd(`AP040_VEC_RESET_PC, `AP040_SZ_L, S_BOOT1);
			end

			S_BOOT1: begin
				if (m_val[0]) fatal_halt;
				else exception_prefetch(m_val, sr_s);
			end

			// Reset/exception processing concludes by fetching four longwords.
			// Any fault in this window is itself a double bus fault.
			S_EPF_FILL: begin
				if (i_err) fatal_halt;
				else if (i_ack) begin
					epf_data[epf_fill] <= mem_rdata[15:0];
					if (epf_fill == 3'd7) begin
						epf_count <= 4'd8;
						epf_head <= 0;
						// eight words wrap the ring: the append index
						// belongs back at the head, or the engine's first
						// fill would land on a resident word
						epf_fill <= 0;
						epf_next <= epf_base;
						// the handler's stream continues past the four
						// architectural longwords under the fill engine
						epf_ftail <= epf_base + 32'd16;
						epf_armed <= 1;
						state <= S_EPF_READY;
					end
					else begin
						epf_fill <= epf_fill + 3'd1;
						state <= S_EPF_GAP;
					end
				end
			end

			// Give the MMU/cache request handshake a full low cycle between
			// words.  Changing an address while req remains asserted can make a
			// completed request look like a duplicate transaction.
			S_EPF_GAP: begin
				ifr_req <= 1;
				ifr_size <= `AP040_SZ_W;
				ifr_addr <= epf_base + {28'd0, epf_fill, 1'b0};
				ifr_fc <= epf_super ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
				state <= S_EPF_FILL;
			end

			S_EPF_READY: begin
				issue_ifetch(epf_base, epf_super);
				pc_i <= epf_base;
				state <= S_FETCH;
			end

			//----------------------------------------------------------- fetch
			// A pure consumer of the fetch queue: the opcode word is either
			// resident, arriving on the bus this very cycle, or the queue is
			// not tracking this address and has to be re-armed here.
			S_FETCH: if (epf_ready_pc || epf_fwd_pc) begin : fetch_word
				reg [15:0] fw;
				fw = epf_fwd_pc ? epf_fwd_word : epf_data[epf_head];
				// Exception processing includes the first handler refill.  An
				// interrupt which becomes pending after vector fetch but before
				// this opcode arrives must still run before the handler executes.
				// Discard the fetched word and stack a return to the handler entry.
				if (in_exc && irq_pend) begin
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
					exc_spc <= pc; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= irq_take_lvl;
					epf_flush;
					state <= S_EXC0;
				end
				else if (flow_t0_pend) begin
					// The completed change-of-flow instruction's T0 trace fires
					// at the redirect target, ahead of the target instruction.
					// The frame PC is the target; its address field identifies
					// the branch/return.
					//
					// This used to be suppressed when the target was ILLEGAL
					// ($4AFC), to avoid double-reporting under the old model
					// where a T0 trace ALSO survived the illegal-instruction
					// exception.  That survivor path is gone (no exception
					// leaves a T0 trace pending on the 040), so the
					// suppression only lost the trace: cputest branches
					// straight into its terminating ILLEGAL and expects
					// vector 9 with the branch target stacked.
					flow_t0_pend <= 0;
					exc(`AP040_VEC_TRACE, 4'd2, pc, flow_t0_oldpc);
				end
				else begin
					// All four exception-prefetch longwords are now resident; the
					// first buffered handler instruction begins normal execution.
					in_exc <= 0;
					epf_pop = 2'd1;
					ir <= fw;
					if (!movem_mem_op(fw)) mm_resume <= 0;
					perf_dispatch_toggle <= ~perf_dispatch_toggle;
					pc <= pc + 32'd2;
					// per-instruction defaults
					tr_t1 <= sr[15];
					tr_t0 <= sr[14];
					flow_t0_pend <= 0;
					t0_force <= t0_special(fw);
					p_src <= SK_NONE; p_dst <= DK_NONE;
					p_rmw <= 0; p_wbsup <= 0; p_flags <= 1; p_sextw <= 0;
					p_dst_mem_bit <= 0;
					exec_kind <= EK_ALU;
					fc_ovr_v <= 0;
					state <= S_DECODE;
				end
			end
			// The queue does not run this stream -- a redirect that could not
			// claim the port, or an SR write that changed the FC one cycle
			// after the fetch was armed -- so re-arm it on the live context.
			else if (!epf_armed || epf_next != pc || epf_super != sr_s)
				issue_ifetch(pc, sr_s);

			//---------------------------- page-crossing split transfers
			// One byte per bus transaction, most significant first, so each
			// byte is translated through its own page.
			S_MRD_B: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				if (!m_issued && epf_pend) begin
				end
				else if (!m_issued) begin
					mem_req <= 1; mem_write <= 0; mem_instr_q <= 0;
					mem_size <= `AP040_SZ_B;
					mem_addr_q <= m_addr_r + {29'd0, m_bidx};
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start(0);
				end
				else if (d_ack) begin : mrd_b
					reg [31:0] acc;
					acc = {m_acc[23:0], mem_rdata[7:0]};
					m_acc <= acc;
					m_issued <= 0;
					if (m_bidx + 3'd1 == m_nbytes) begin
						m_val <= acc;
						state <= r_m_ret;
					end
					else m_bidx <= m_bidx + 3'd1;
				end
			end

			S_MWR_B: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				if (!m_issued && epf_pend) begin
				end
				else if (!m_issued) begin : mwr_b
					reg [7:0] byv;
					case (m_size)
						`AP040_SZ_W: byv = m_bidx[0] ? m_wdat[7:0] : m_wdat[15:8];
						`AP040_SZ_L: case (m_bidx[1:0])
							2'd0: byv = m_wdat[31:24];
							2'd1: byv = m_wdat[23:16];
							2'd2: byv = m_wdat[15:8];
							default: byv = m_wdat[7:0];
						endcase
						default: byv = m_wdat[7:0];
					endcase
					mem_req <= 1; mem_write <= 1; mem_instr_q <= 0;
					mem_size <= `AP040_SZ_B;
					mem_addr_q <= m_addr_r + {29'd0, m_bidx};
					mem_wdata <= {24'd0, byv};
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start(0);
				end
				else if (d_ack) begin
					m_issued <= 0;
					if (m_bidx + 3'd1 == m_nbytes) state <= r_m_ret;
					else m_bidx <= m_bidx + 3'd1;
				end
			end

			//------------------------------------------------- generic helpers
			S_NEXT: fetch_next;

			// Post-instruction trace/IRQ commit barrier.  rf_we/aux_we from
			// the completing instruction have reached the register file by
			// the time S_EXC0 runs on the following qualified edge.
			S_POST_EXC:    state <= S_EXC0;
			S_POST_EXC_F2: state <= S_EXC0_F2;
			S_POST_EXC_F3: state <= S_EXC0_F3;
			S_POST_EXC_F4: state <= S_EXC0_F4;

			// Extension-word fetch, the queue's second consumer.  A longword
			// immediate whose two words are both available is taken in one
			// pass, whether they come from the queue or off the bus.
			S_IMMF: begin
				if (imm_n == 2'd2 && epf_fwd_pc && epf_pend_lw) begin
					imm      <= mem_rdata;
					pc       <= pc + 32'd4;
					epf_pop   = 2'd2;
					state    <= r_imm_ret;
				end
				else if (imm_n == 2'd2 && epf_ready_pc2) begin
					imm      <= {epf_data[epf_head], epf_data[epf_head + 3'd1]};
					pc       <= pc + 32'd4;
					epf_pop   = 2'd2;
					state    <= r_imm_ret;
				end
				else if (epf_ready_pc || epf_fwd_pc) begin
					imm <= {imm[15:0],
					        epf_fwd_pc ? epf_fwd_word : epf_data[epf_head]};
					pc <= pc + 32'd2;
					epf_pop = 2'd1;
					if (imm_n == 2'd1) begin
						// Prepare the indexed EA read port while consuming its
						// extension. S_EA_EXTW2 retains all brief/full-format
						// arithmetic and legality checks; no extra EA adder.
						// Serialize any pending register/stack-bank write through
						// the old EXTW stage rather than capturing an old base.
						if (r_imm_ret == S_EA_EXTW && !rf_we && !aux_we) begin
							extw <= epf_fwd_pc ? epf_fwd_word : epf_data[epf_head];
							rr_b <= epf_fwd_pc ? epf_fwd_word[15:12] :
							        epf_data[epf_head][15:12];
							ea_base_v <= ea_pcmode ? ea_pcb : rf_rdata_a;
							state <= S_EA_EXTW2;
						end
						else state <= r_imm_ret;
					end
					else imm_n <= imm_n - 2'd1;
				end
				else if (!epf_armed || epf_next != pc || epf_super != sr_s)
					issue_ifetch(pc, sr_s);
			end

			S_MRD: begin
                // Select the simple MOVE destination while source memory is busy.
                // This performs no destination access or architectural update.
                if (r_m_ret == S_PIPE_SDONE && p_src == SK_MEM &&
                    p_dst == DK_MEM && exec_kind == EK_ALU &&
                    alu_op == `AP040_ALU_MOVE && !p_rmw &&
                    (dst_mode_r == 3'b010 || dst_mode_r == 3'b011 ||
                     dst_mode_r == 3'b100 || dst_mode_r == 3'b101)) rr_a <= {1'b1, dst_rn_r};
                // Prepare only register selectors; extension consumption and
                // destination access still wait for source read success.
                if (r_m_ret == S_PIPE_SDONE && p_src == SK_MEM &&
                    p_dst == DK_MEM && exec_kind == EK_ALU &&
                    alu_op == `AP040_ALU_MOVE && !p_rmw && dst_mode_r == 3'b110 &&
                    epf_ready_pc && !epf_data[epf_head][8]) begin
                    rr_a <= {1'b1, dst_rn_r};
                    rr_b <= epf_data[epf_head][15:12];
                end
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				// (a fetch in flight no longer holds a data transfer: P171)
				if (!m_issued && m_cross) begin
					m_bidx <= 0;
					m_acc <= 0;
					state <= S_MRD_B;
				end
				else if (!m_issued) begin
					mem_req <= 1; mem_write <= 0; mem_instr_q <= 0;
					mem_size <= m_size; mem_addr_q <= m_addr_r;
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start(0);
				end
				else if (d_ack) begin
					// Port B has settled during the read. Reuse the shared ALU
					// for all ordinary memory-to-register operations, not just
					// MOVE. Keep the original completion/fault ordering and leave
					// non-ALU/system consumers on their capture/execute path.
`ifdef AP040_EXPERIMENTAL_PIPELINE
                    // A normal pipeline read forwards the acknowledgement
                    // directly into WB. Split reads keep the buffered return.
                    if (pipe_load_direct) begin
                        if (pipe_read_retire && pipe_retire) begin
                            sr[4:0] <= pipe_ccr;
                            pc_i <= pipe_pc;
                            ir <= pipe_opcode;
                        end
                        state <= S_EXPERIMENT_PIPE;
                        if (pipe_read_retire && pipe_retire && pipe_exit_ready && !pipe_input)
                            fetch_next;
                    end else
`endif
					if (r_m_ret == S_PIPE_SDONE && p_src == SK_MEM &&
					    p_dst == DK_REG && exec_kind == EK_ALU) begin
						retire_operand_alu;
					end
                    // A completed quick arithmetic read can prepare its ordered
                    // store now. Split reads keep DDONE/EXEC; errors win above.
                    else if (quick_rmw_direct) begin
                        if (p_flags) sr[4:0] <= alu_fl;
                        mwr(dst_addr, p_dsize, alu_res, S_NEXT);
                    end
                    // The source read has succeeded and the simple destination
                    // base is settled. Keep its undo/update order, then replace
                    // the completed read with the ordered destination write.
                    else if (move_store_read_ready) begin
                        src_val <= mem_rdata;
                        dst_addr <= move_store_read_addr;
                        if (dst_mode_r == 3'b011) begin
                            rfw({1'b1,dst_rn_r},rf_rdata_a + an_adj(dst_rn_r,p_dsize));
                            u_rec({1'b1,dst_rn_r},rf_rdata_a);
                        end else if (dst_mode_r == 3'b100) begin
                            rfw({1'b1,dst_rn_r},move_store_read_addr);
                            u_rec({1'b1,dst_rn_r},rf_rdata_a);
                        end else if (dst_mode_r == 3'b101) begin
                            pc <= pc + 32'd2;
                            epf_pop = 2'd1;
                            epf_issue = 1;
                        end
                        if (p_flags) sr[4:0] <= alu_fl;
                        mwr(move_store_read_addr,p_dsize,alu_res,S_NEXT);
                    end
                    // Begin the destination EA once the source read succeeds.
                    // Faulting or page-split reads retain their original path.
                    else if (r_m_ret == S_PIPE_SDONE && p_src == SK_MEM &&
                             p_dst == DK_MEM && exec_kind == EK_ALU &&
                             alu_op == `AP040_ALU_MOVE && !p_rmw &&
                             (dst_mode_r == 3'b110 || dst_mode_r == 3'b101 ||
                              dst_mode_r == 3'b010 || dst_mode_r == 3'b011 ||
                              dst_mode_r == 3'b100)) begin
                        src_val <= mem_rdata;
                        ea_start(dst_mode_r, dst_rn_r, p_dsize, S_PIPE_DEA);
                        // d16 destination: select its base and request the
                        // extension together. S_IMMF still provides the
                        // register-settling and precise extension-fault edge.
                        // The queued displacement and settled destination base
                        // can resolve together once the source read succeeds.
                        if (dst_mode_r == 3'b101 && epf_ready_pc && !epf_flushed &&
                            rr_a == {1'b1,dst_rn_r} && !dst_base_landing && !aux_we) begin
                            ea_addr <= rf_rdata_a + sxw(epf_data[epf_head]);
                            pc <= pc + 32'd2;
                            epf_pop = 2'd1;
                            epf_issue = 1;
                            state <= S_PIPE_DEA;
                        end else if (dst_mode_r == 3'b101) immf(2'd1, S_EA_D16);
                        // A settled base lets the successful source response
                        // perform the existing EA_DISP work. Pending writes
                        // retain EA_DISP so aliased source updates land first.
                        if ((dst_mode_r == 3'b010 || dst_mode_r == 3'b011 ||
                             dst_mode_r == 3'b100) &&
                            rr_a == {1'b1,dst_rn_r} && !dst_base_landing && !aux_we) begin
                            ea_addr <= (dst_mode_r == 3'b100) ?
                                rf_rdata_a - an_adj(dst_rn_r,p_dsize) : rf_rdata_a;
                            if (dst_mode_r == 3'b011) begin
                                rfw({1'b1,dst_rn_r},rf_rdata_a + an_adj(dst_rn_r,p_dsize));
                                u_rec({1'b1,dst_rn_r},rf_rdata_a);
                            end else if (dst_mode_r == 3'b100) begin
                                rfw({1'b1,dst_rn_r},rf_rdata_a - an_adj(dst_rn_r,p_dsize));
                                u_rec({1'b1,dst_rn_r},rf_rdata_a);
                            end
                            state <= S_PIPE_DEA;
                        end
                        if (dst_mode_r == 3'b110 && epf_ready_pc && !epf_flushed && !epf_data[epf_head][8] &&
                            rr_a == {1'b1,dst_rn_r} && rr_b == epf_data[epf_head][15:12] &&
                            !rf_we && !aux_we) begin
                            extw <= epf_data[epf_head];
                            ea_base_v <= rf_capture_a;
                            pc <= pc + 32'd2;
                            epf_pop = 2'd1;
                            epf_issue = 1;
                            state <= S_EA_EXTW2;
                        end
                    end
					else if (r_m_ret == S_PIPE_SDONE && p_dst == DK_REG) begin
						src_val <= mem_rdata;
						dst_val <= rf_rdata_b;
						state <= S_EXEC;
					end
					// UNLK: A7 was written with the read issue; the
					// popped frame pointer lands here and the
					// instruction retires.
                    // Normal RTS commits A7 and redirects at read acknowledgement.
                    // Fault, odd-target, trace/IRQ and split reads keep RET2.
                    // P199: RTD too (A7 + 4 + d16), which went through S_RET2
                    // and left its target fetch a clock late.
                    else if (r_m_ret == S_RET2 && (ret_kind == RK_RTS || ret_kind == RK_RTD) &&
                             !mem_rdata[0] && !tr_t1 && !tr_t0 && !irq_pend) begin
                        rfw(4'd15, dbg_a7 + 32'd4 + ((ret_kind == RK_RTD) ? sxw(imm[15:0]) : 32'd0));
                        go_pc(mem_rdata);
                    end
                    // Final ordinary FPU operand beat can launch at ack.
                    // Split transfers still return through S_FPU_RD.
                    else if (r_m_ret == S_FPU_RD &&
                             ((fp_nb <= 4'd4 && fp_n != 4'd0) ||
                              (fp_nb == 4'd8 && fp_n == 4'd2) ||
                              (fp_nb == 4'd12 && fp_n == 4'd3))) begin
                        if (fp_nb == 4'd1) fpb[95:88] <= mem_rdata[7:0];
                        else if (fp_nb == 4'd2) fpb[95:80] <= mem_rdata[15:0];
                        else case (fp_n)
                            4'd1: fpb[95:64] <= mem_rdata;
                            4'd2: fpb[63:32] <= mem_rdata;
                            default: fpb[31:0] <= mem_rdata;
                        endcase
                        fpu_iawe <= 1;
                        fpu_req <= 1;
                        state <= S_FPU_GO;
                    end
                    // Queue the next ordinary multiword FPU operand read
                    // after a successful beat. Faults and split transfers retain
                    // their existing paths; no FPU register commits here.
                    else if (r_m_ret == S_FPU_RD &&
                             ((fp_nb == 4'd8 && fp_n == 4'd1) ||
                              (fp_nb == 4'd12 && (fp_n == 4'd1 || fp_n == 4'd2)))) begin
                        if (fp_n == 4'd1) fpb[95:64] <= mem_rdata;
                        else fpb[63:32] <= mem_rdata;
                        mrd(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L, S_FPU_RD);
                        fp_n <= fp_n + 4'd1;
                    end
                    // Capture a successful FMOVEM read beat at acknowledgement.
                    // The register commit remains in S_FPU_MVM2 after all words.
                    else if (r_m_ret == S_FPU_MVM3 && !fp_st) begin
                        case (fp_n)
                            4'd1: fpb[95:64] <= mem_rdata;
                            4'd2: fpb[63:32] <= mem_rdata;
                            default: fpb[31:0] <= mem_rdata;
                        endcase
                        // P205: the register's next beat straight from the
                        // acknowledge (in place when hinted, fpu_rd_next)
                        if (fp_n != 4'd3) begin
                            mrd(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L, S_FPU_MVM3);
                            fp_n <= fp_n + 4'd1;
                        end
                        else state <= S_FPU_MVM2;
                    end
					else if (r_m_ret == S_UNLK3) begin
						rfw({1'b1, d_rn}, mem_rdata);
						fetch_next;
					end
					// MOVEM load: the transfer retires on its acknowledge,
					// exactly as S_MOVEM_LD would a cycle later (that state
					// stays for the byte-split path, which returns to r_m_ret
					// as a state).  With the loop's in-place issue and the
					// one-clock hit a resident load is two cycles per
					// register: S_MOVEM_LOOP and this acknowledge.  (2026-09-17)
					else if (r_m_ret == S_MOVEM_LD) begin : movem_ld_ack
						reg [31:0] lv;
						lv = (mm_size == `AP040_SZ_W) ? sxw(mem_rdata[15:0]) : mem_rdata;
						if (mm_base_ea && mm_reg == {1'b1, d_rn}) begin
							if (!mm_postinc) begin
								mm_base_pend <= 1;
								mm_base_val  <= lv;
							end
						end
						else if (mm_idx_en && mm_reg == mm_idx_reg) begin
							mm_idx_pend <= 1;
							mm_idx_val  <= lv;
						end
						else rfw(mm_reg, lv);
						mm_addr <= mm_addr + ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
						// P216: the next register's load straight from this
						// acknowledge (in place when hinted, mm_rd_next): the
						// loop's step for it, one clock a register
						if (mm_mask != 16'd0) begin : movem_chain
							reg [3:0] bi;
							bi = ffs16(mm_mask);
							mm_mask <= mm_mask & ~(16'd1 << bi);
							mm_reg  <= bi;
							mrd(mmn_addr, mm_size, S_MOVEM_LD);
						end
						else state <= S_MOVEM_LOOP;
					end
					else begin
						m_val <= mem_rdata;
						state <= r_m_ret;
					end
				end
			end

			S_MWR: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				// (a fetch in flight no longer holds a data transfer: P171)
				if (!m_issued && m_cross) begin
					m_bidx <= 0;
					state <= S_MWR_B;
				end
				else if (!m_issued) begin
					mem_req <= 1; mem_write <= 1; mem_instr_q <= 0;
					mem_size <= m_size; mem_addr_q <= m_addr_r;
					mem_wdata <= m_wdat;
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start(0);
				end
				else if (d_ack) begin
					// A completed store with nothing left to do retires
					// straight into the next opcode, as a completed operand
					// read does through retire_operand_alu.
					`ifdef AP040_EXPERIMENTAL_PIPELINE
                    if (pipe_load_direct) state <= S_EXPERIMENT_PIPE;
                    else
`endif
                    // Successful ordinary FMOVEM stores need no MVM3 copy step.
                    if (r_m_ret == S_FPU_MVM3 && fp_st) state <= S_FPU_MVM2;
                    else if (r_m_ret == S_NEXT) fetch_next;
					// LINK and PEA: A7 lands with the push's acknowledge and
					// the instruction retires, as S_LINK4/S_PEA2 would a cycle
					// later (both stay for the byte-split path).  (2026-09-17)
					else if (r_m_ret == S_LINK4) begin
						rfw(4'd15, t_a + (br_long ? imm : sxw(imm[15:0])));
						fetch_next;
					end
					else if (r_m_ret == S_PEA2) begin
						rfw(4'd15, dbg_a7 - 32'd4);
						fetch_next;
					end
					// BSR/JSR whose target fetch went out at the pop
					// (dispatch_branch): the stream is already at
					// br_tgt, so the redirect S_BSR_PUSH/S_JSR2 would
					// raise a cycle later can be raised here without
					// the deferred-issue problem (go_pc's issue_ifetch
					// finds the stream and issues nothing).  Any other
					// push keeps its redirect state.  (2026-09-18)
					else if ((r_m_ret == S_BSR_PUSH || r_m_ret == S_JSR2) &&
					         epf_armed && (epf_next == br_tgt) &&
					         (epf_super == sr_s)) begin
						rfw(4'd15, dbg_a7 - 32'd4);
						go_pc(br_tgt);
					end
					else state <= r_m_ret;
				end
			end

			//------------------------------------------------------- EA engine
			S_EA_DISP: begin
				case (ea_mode)
					3'b010: begin
						ea_addr <= rf_rdata_a;
						state <= r_ea_ret;
					end
					3'b011: begin
						ea_addr <= rf_rdata_a;
						rfw({1'b1, ea_rn}, rf_rdata_a + an_adj(ea_rn, ea_size));
						u_rec({1'b1, ea_rn}, rf_rdata_a);
						state <= r_ea_ret;
					end
					3'b100: begin
						ea_addr <= rf_rdata_a - an_adj(ea_rn, ea_size);
						rfw({1'b1, ea_rn}, rf_rdata_a - an_adj(ea_rn, ea_size));
						u_rec({1'b1, ea_rn}, rf_rdata_a);
						state <= r_ea_ret;
					end
					3'b101: begin
						immf(2'd1, S_EA_D16);
					end
					3'b110: begin
						immf(2'd1, S_EA_EXTW);
					end
					default: begin // 111
						case (ea_rn)
							3'b000: begin ea_absl <= 0; immf(2'd1, S_EA_ABS); end
							3'b001: begin ea_absl <= 1; immf(2'd2, S_EA_ABS); end
							3'b010: begin ea_pcmode <= 1; immf(2'd1, S_EA_D16); end
							3'b011: begin ea_pcmode <= 1; immf(2'd1, S_EA_EXTW); end
							default: go_illegal;
						endcase
					end
				endcase
			end

			S_EA_D16: begin
				ea_addr <= (ea_pcmode ? ea_pcb : rf_rdata_a) + sxw(imm[15:0]);
				state <= r_ea_ret;
                if (hint_displacement_read) begin
                    if (p_dst == DK_REG) rr_b <= p_dreg;
                    mrd(hint_displacement_addr, p_ssize, S_PIPE_SDONE);
                end
`ifdef AP040_EXPERIMENTAL_LEA
                // P180: PEA pushes its address from here (S_PEA1's work, with
                // the forwarded base and A7); S_PEA2 still updates A7.
                if (r_ea_ret == S_PEA1)
                    mwr(dbg_a7_wb - 32'd4, `AP040_SZ_L,
                        (ea_pcmode ? ea_pcb : rf_capture_a) + sxw(imm[15:0]), S_PEA2);
                // LEA has no operand access or flags to finish after EA.
                // Retire only after its extension has completed normally.
                if (r_ea_ret == S_LEA1) begin
                    rfw({1'b1, d_reg9},
                        (ea_pcmode ? ea_pcb : rf_capture_a) + sxw(imm[15:0]));
                    fetch_next;
                end
`endif
			end

			S_EA_EXTW: begin
				extw <= imm[15:0];
				rr_b <= {imm[15], imm[14:12]};
				// remember which register the index came from, for MOVEM
				mm_idx_reg <= {imm[15], imm[14:12]};
				mm_idx_en  <= !imm[8] || !imm[6];   // full format may suppress it
				ea_base_v <= ea_pcmode ? ea_pcb : rf_rdata_a;
				state <= S_EA_EXTW2;
			end

			S_EA_EXTW2: begin : ea_extw2
				reg [31:0] idx;
				idx = extw[11] ? rf_rdata_b : sxw(rf_rdata_b[15:0]);
				idx = idx << extw[10:9];
				if (!extw[8]) begin
					ea_addr <= ea_base_v + idx + sxb(extw[7:0]);
					state <= r_ea_ret;
                    if (hint_indexed_read) begin
                        if (p_dst == DK_REG) rr_b <= p_dreg;
                        mrd(hint_indexed_addr, p_ssize, S_PIPE_SDONE);
                    end
				end
				// The PRM marks IS=1 with I/IS[2]=1 "reserved", but real 68040
				// silicon EXECUTES those encodings instead of trapping -- captured
				// on a Quadra 800, vec=0 (docs/ap68040-memind-reserved.md).
				// Suppressing the index makes pre- and post-indexing
				// arithmetically identical (the code below already forces
				// ea_idx_v to 0 when IS is set), so I/IS[1:0] just selects the
				// outer displacement and the address comes out correct.
				// Rejecting them made MOVE.L ([bd.W,A6]),D1 and
				// MOVE.L ([bd.W=0,A6],od.W=4),D1 take vector 4 where hardware
				// returns a value.
				else if (extw[5:4] == 2'b00 || extw[3] ||
				         extw[2:0] == 3'b100) begin
					go_illegal;
				end
				else begin
					ea_base_v <= extw[7] ? 32'd0 : ea_base_v;
					ea_idx_v  <= extw[6] ? 32'd0 : idx;
					ea_post   <= extw[2];
					ea_odl    <= (extw[1:0] == 2'b11);
					if (extw[5:4] == 2'b01) begin
						imm <= 0;
						state <= S_EA_BD;
					end
					else immf((extw[5:4] == 2'b10) ? 2'd1 : 2'd2, S_EA_BD);
				end
			end

			S_EA_BD: begin : ea_bd
				reg [31:0] bd;
				bd = (extw[5:4] == 2'b01) ? 32'd0 :
				     (extw[5:4] == 2'b10) ? sxw(imm[15:0]) : imm;
				if (extw[2:0] == 3'b000) begin
					ea_addr <= ea_base_v + ea_idx_v + bd;
					state <= r_ea_ret;
				end
				else if (mm_resume && r_ea_ret == S_MOVEM_EA) begin
					// Consume extension words but NEVER reread the indirect
					// pointer: an earlier MOVEM store may have overwritten it.
					// S_MOVEM_EA takes the address restored from the frame.
					if (extw[1:0] == 2'b01) state <= r_ea_ret;
					else immf(extw[1:0] == 2'b10 ? 2'd1 : 2'd2, S_EA_OD);
				end
				else begin
					// memory indirect: pre-indexed adds the index before the
					// indirection, post-indexed after
					mrd(ea_base_v + bd + (ea_post ? 32'd0 : ea_idx_v),
					    `AP040_SZ_L, S_EA_MIND);
				end
			end

			S_EA_MIND: begin
				ea_mind <= m_val;
				case (extw[1:0])
					2'b01: begin
						ea_addr <= m_val + (ea_post ? ea_idx_v : 32'd0);
						state <= r_ea_ret;
					end
					2'b10: immf(2'd1, S_EA_OD);
					default: immf(2'd2, S_EA_OD);
				endcase
			end

			S_EA_OD: begin
				ea_addr <= ea_mind + (ea_post ? ea_idx_v : 32'd0) +
				           (ea_odl ? imm : sxw(imm[15:0]));
				state <= r_ea_ret;
			end

			S_EA_ABS: begin
				ea_addr <= ea_absl ? imm : sxw(imm[15:0]);
				state <= r_ea_ret;
			end

			//------------------------------------------------ operand pipeline
			S_PIPE_START: begin : pipe_start
				reg dst_go;   // start the memory destination's EA (one call site)
				dst_go = 0;
				// x_ext keeps a decode-time immediate through EA fetches;
				// for long MUL/DIV it was already captured in S_MDL_EXT
				if (exec_kind != EK_MD_L) x_ext <= imm;
				// A register destination needs no EA, so its operand can be
				// read on port B in the SAME cycle the source is read on
				// port A (X2.3).  The old path spent one state per port.
				case (p_src)
					SK_MEM: begin
						// Decode already selected the source An.  Resolve the three
						// simple modes here and bypass S_EA_DISP; extension-bearing
						// modes overlap EA selection with extension-request setup.
						case (src_mode_r)
							// The base comes through the forwarded port: a record
							// handed over at a retire that writes this register
							// reaches here while that write lands.
							3'b010: begin
								if (p_dst == DK_REG) rr_b <= p_dreg;
								mrd(rf_capture_a, p_ssize, S_PIPE_SDONE);
							end
							3'b011: begin
								if (p_dst == DK_REG) rr_b <= p_dreg;
								mrd(rf_capture_a, p_ssize, S_PIPE_SDONE);
								rfw({1'b1, src_rn_r},
								    rf_capture_a + an_adj(src_rn_r, p_ssize));
								u_rec({1'b1, src_rn_r}, rf_capture_a);
							end
							3'b100: begin : pipe_predec_read
								reg [31:0] predec_addr;
								predec_addr = rf_capture_a - an_adj(src_rn_r, p_ssize);
								if (p_dst == DK_REG) rr_b <= p_dreg;
								mrd(predec_addr, p_ssize, S_PIPE_SDONE);
								rfw({1'b1, src_rn_r}, predec_addr);
								u_rec({1'b1, src_rn_r}, rf_capture_a);
							end
							default:
								ea_operand_start(src_mode_r, src_rn_r, p_ssize,
								         S_PIPE_SRD);
						endcase
					end
					SK_REG:
						if (p_dst == DK_REG) begin
							rr_a <= p_sreg; rr_b <= p_dreg;
							state <= S_PIPE_REGS;
						end
						// Memory destination with the ports already placed by
						// decode: capture the source from port B and start the
						// destination EA now, skipping S_PIPE_SREG, S_PIPE_DST
						// and (for the simple modes) S_EA_DISP.
						else if (p_dst == DK_MEM && !p_dst_mem_bit &&
						         (rr_b == p_sreg) && (rr_a == {1'b1, dst_rn_r})) begin
							src_val <= rf_capture_b;
							dst_go = 1;
						end
						else begin rr_a <= p_sreg; state <= S_PIPE_SREG; end
					SK_IMM: begin
						src_val <= imm;
						if (p_dst == DK_REG) begin
							rr_b <= p_dreg; state <= S_PIPE_REGS;
						end
						else if (p_dst == DK_MEM) dst_go = 1;
						else state <= S_PIPE_DST;
					end
					default:
						if (p_dst == DK_REG) begin
							rr_b <= p_dreg; state <= S_PIPE_REGS;
						end
						// A memory destination with no source operand starts
						// its EA here instead of spending S_PIPE_DST on the
						// same dispatch (and S_EA_DISP on extension modes).
						else if (p_dst == DK_MEM) dst_go = 1;
						else state <= S_PIPE_DST;
				endcase
				// one expansion of the EA-start task serves every memory
				// destination above (each expansion carries its own read
				// issue logic; four of them doubled the core's logic)
				if (dst_go) ea_operand_start(dst_mode_r, dst_rn_r, p_dsize, S_PIPE_DEA);
			end

			// The EA is finished by now, so port B is free: point it at a
			// register destination WHILE the source read is in flight, and
			// both operands land together when the read returns (X2.3).
			S_PIPE_SRD: begin
				if (p_dst == DK_REG) rr_b <= p_dreg;
				mrd(ea_addr, p_ssize, S_PIPE_SDONE);
			end
			S_PIPE_SREG:  begin src_val <= rf_rdata_a; state <= S_PIPE_DST; end

			S_PIPE_DST: begin
				case (p_dst)
					DK_MEM: ea_start(dst_mode_r, dst_rn_r, p_dsize, S_PIPE_DEA);
					DK_REG: begin rr_b <= p_dreg; state <= S_PIPE_DREG; end
					default: state <= S_EXEC;
				endcase
			end

			S_PIPE_DEA: begin
				dst_addr <= ea_addr;
				if (p_rmw) mrd(ea_addr, p_dsize, S_PIPE_DDONE);
                else if (exec_kind == EK_ALU && p_dst == DK_MEM && !p_wbsup &&
                         ((alu_op == `AP040_ALU_MOVE && (p_src == SK_MEM || p_src == SK_IMM)) ||
                          alu_op == `AP040_ALU_CLR)) begin
                    // Required source access and destination EA are complete.
                    // MOVE and CLR need no destination operand; reuse the ordinary ALU
                    // flags and ordered write path one state earlier.
                    if (p_flags) sr[4:0] <= alu_fl;
                    mwr(ea_addr, p_dsize, alu_res, S_NEXT);
                end
				else state <= S_EXEC;
			end

			S_PIPE_DDONE: begin dst_val <= m_val; state <= S_EXEC; end
			S_PIPE_DREG:  begin dst_val <= rf_rdata_b; state <= S_EXEC; end

			// Share retirement for register and completed-memory sources. Keep
			// non-ALU and memory-destination operations on the original path.
			S_PIPE_REGS, S_PIPE_SDONE: begin
				if (exec_kind == EK_ALU && p_dst == DK_REG) begin
					retire_operand_alu;
				end
				else if (shift_fire && (shift_cnt != 6'd0)) begin
					// register shift, whole count in one ALU call (a zero
					// count keeps the S_SHIFT path for its special flags)
					sr[4:0] <= alu_fl;
					rfw(p_dreg, merge_sz(rf_capture_b, alu_res, op_size));
					fetch_next;
				end
				else begin
					// Forwarded ports: a lookahead dispatch lands here one
					// cycle after its producer's register write, which is
					// still in flight (rf_we) in this cycle.
					if (state == S_PIPE_SDONE) src_val <= m_val;
					else if (p_src == SK_REG) src_val <= rf_capture_a;
					if (p_dst == DK_REG) begin
						dst_val <= rf_capture_b;
						state <= S_EXEC;
					end
					else state <= S_PIPE_DST;
				end
			end

			//-------------------------------------------------------- execute
			S_EXEC: begin
				case (exec_kind)
					EK_SHIFT: begin
						sh_val <= dst_val;
						sh_fl <= sr[4:0];
						sh_vacc <= 0;
						sh_any <= 0;
						sh_cnt <= (p_src == SK_NONE) ? 6'd1 : src_val[5:0];
						state <= S_SHIFT;
					end

					EK_MD_W: begin
						if (md_isdiv && src_val[15:0] == 16'd0) begin
							// 68040 DIVU/DIVS divide-by-zero preserves X/N/Z/V
							// but clears C before taking vector 5.
							sr[0] <= 1'b0;
							exc(`AP040_VEC_DIVZERO, 4'd2, pc, pc_i);
						end
						else begin
							md_a  <= md_sign ? sxw(src_val[15:0]) : {16'd0, src_val[15:0]};
							md_hi <= md_sign ? {32{dst_val[31]}} : 32'd0;
							md_lo <= md_isdiv ? dst_val
							         : (md_sign ? sxw(dst_val[15:0]) : {16'd0, dst_val[15:0]});
							md_start <= 1;
							state <= S_MD_WAIT;
						end
					end

					EK_MD_L: begin
						// stage the read of Dl/Dq named in the extension word
						md_sign <= x_ext[11];
						rr_b <= {1'b0, x_ext[14:12]};
						state <= S_MDL_RDQ;
					end

					EK_CHK: begin : ek_chk
						reg signed [31:0] v, bound;
						v = (op_size == `AP040_SZ_W) ? $signed(sxw(dst_val[15:0]))
						                             : $signed(dst_val);
						bound = (op_size == `AP040_SZ_W) ? $signed(sxw(src_val[15:0]))
						                                 : $signed(src_val);
						// 68040 flags: N always tracks the value's sign; C is
						// cleared in bounds and set on a trap only for these
						// sign combinations; Z, V and X are left unchanged
						// (cputest 68040_default reference on hardware)
						sr[3] <= (v < 0);
						if (v < 0 || v > bound) begin
							sr[0] <= (v < 0 && bound >= 0) ||
							         (bound >= 0 && v >= bound) ||
							         (v < 0 && bound < v);
							exc(`AP040_VEC_CHK, 4'd2, pc, pc_i);
						end
						else begin
							sr[0] <= 0;
							fetch_next;
						end
					end

					EK_SCC: begin : ek_scc
						reg [31:0] r;
						r = {24'd0, {8{cond_true(ir[11:8])}}};
						if (p_dst == DK_REG) begin
							rfw(p_dreg, merge_sz(dst_val, r, `AP040_SZ_B));
							fetch_next;
						end
						else mwr(dst_addr, `AP040_SZ_B, r, S_NEXT);
					end

					EK_PACK: begin : ek_pack
						reg [15:0] v;
						v = src_val[15:0] + x_ext[15:0];
						if (p_dst == DK_REG) begin
							rfw(p_dreg, merge_sz(dst_val, {24'd0, v[11:8], v[3:0]}, `AP040_SZ_B));
							fetch_next;
						end
						else mwr(dst_addr, `AP040_SZ_B, {24'd0, v[11:8], v[3:0]}, S_NEXT);
					end

					EK_UNPK: begin : ek_unpk
						reg [15:0] v;
						v = {4'd0, src_val[7:4], 4'd0, src_val[3:0]} + x_ext[15:0];
						if (p_dst == DK_REG) begin
							rfw(p_dreg, merge_sz(dst_val, {16'd0, v}, `AP040_SZ_W));
							fetch_next;
						end
						else mwr(dst_addr, `AP040_SZ_W, {16'd0, v}, S_NEXT);
					end

					default: begin // EK_ALU
						if (p_flags) sr[4:0] <= alu_fl;
						if (p_wbsup) fetch_next;
						else case (p_dst)
							DK_MEM: mwr(dst_addr, p_dsize, alu_res, S_NEXT);
							DK_REG: begin
								if (p_dreg[3])
									rfw(p_dreg, alu_res);
								else
									rfw(p_dreg, merge_sz(dst_val, alu_res, op_size));
								fetch_next;
							end
							// SR settles first so the next fetch uses the new
						// S bit's FC (and trace enables)
						DK_SR:  begin sr <= alu_res[15:0] & `AP040_SR_MASK; state <= S_NEXT; end
							DK_CCR: begin sr[4:0] <= alu_res[4:0]; fetch_next; end
							default: fetch_next;
						endcase
					end
				endcase
			end

			//--------------------------------------------------------- shifts
			S_SHIFT: begin
				if (sh_cnt == 6'd0) begin
					sr[4] <= sh_fl[4];
					sr[3] <= (op_size == `AP040_SZ_B) ? sh_val[7] :
					         (op_size == `AP040_SZ_W) ? sh_val[15] : sh_val[31];
					sr[2] <= ((sh_val & ((op_size == `AP040_SZ_B) ? 32'hFF :
					          (op_size == `AP040_SZ_W) ? 32'hFFFF : 32'hFFFFFFFF)) == 0);
					sr[1] <= sh_vacc;
					// zero count: C=0 for shifts/rotates, C=X for ROXx
					sr[0] <= sh_any ? sh_fl[0] : (sh_rox ? sh_fl[4] : 1'b0);
					state <= S_SHIFT_WB;
				end
				else begin
					// single-cycle barrel: the ALU composed the whole count,
					// commit value and flags directly
					sh_val <= alu_res;
					sr[4] <= alu_fl[4];
					sr[3] <= alu_fl[3];
					sr[2] <= alu_fl[2];
					sr[1] <= alu_fl[1];
					sr[0] <= alu_fl[0];
					state <= S_SHIFT_WB;
				end
			end

			S_SHIFT_WB: begin
				if (p_dst == DK_REG) begin
					rfw(p_dreg, merge_sz(dst_val, sh_val, op_size));
					fetch_next;
				end
				else mwr(dst_addr, p_dsize, sh_val, S_NEXT);
			end

			//------------------------------------------------ multiply/divide
			S_MDL_EXT: begin
				x_ext <= imm;
				if (p_src == SK_IMM) immf(2'd2, S_PIPE_START);
				else state <= S_PIPE_START;
			end

			S_MDL_RDQ: begin
				// rf_rdata_b is Dl (multiply) or Dq (divide low dividend)
				if (md_isdiv && src_val == 32'd0) begin
					// 68040 DIVL divide-by-zero preserves X/N/Z/V
					// but clears C before taking vector 5.
					sr[0] <= 1'b0;
					exc(`AP040_VEC_DIVZERO, 4'd2, pc, pc_i);
				end
				else if (md_isdiv && x_ext[10]) begin
					dst_val <= rf_rdata_b;
					rr_a <= {1'b0, x_ext[2:0]};   // Dr holds the high dividend
					state <= S_MDL_RDR;
				end
				else begin
					md_a  <= src_val;
					md_hi <= md_isdiv ? (x_ext[11] ? {32{rf_rdata_b[31]}} : 32'd0) : 32'd0;
					md_lo <= rf_rdata_b;
					md_start <= 1;
					state <= S_MD_WAIT;
				end
			end

			S_MDL_RDR: begin
				md_a  <= src_val;
				md_hi <= rf_rdata_a;
				md_lo <= dst_val;
				md_start <= 1;
				state <= S_MD_WAIT;
			end

			S_MD_WAIT: if (md_done) begin
				if (exec_kind == EK_MD_W) begin
					if (md_isdiv) begin : mdw_div
						reg ovf_w;
						ovf_w = md_ovf |
						        (md_sign ? (($signed(md_rlo) > 32'sd32767) ||
						                    ($signed(md_rlo) < -32'sd32768))
						                 : (md_rlo > 32'h0000_FFFF));
						if (ovf_w) begin
							sr[1] <= 1; sr[0] <= 0;
							fetch_next;
						end
						else begin
							rfw(p_dreg, {md_rhi[15:0], md_rlo[15:0]});
							sr[3] <= md_rlo[15];
							sr[2] <= (md_rlo[15:0] == 16'd0);
							sr[1] <= 0; sr[0] <= 0;
							fetch_next;
						end
					end
					else begin
						rfw(p_dreg, md_rlo);
						sr[3] <= md_rlo[31];
						sr[2] <= (md_rlo == 32'd0);
						sr[1] <= 0; sr[0] <= 0;
						fetch_next;
					end
				end
				else begin // EK_MD_L
					if (md_isdiv) begin
						if (md_ovf) begin
							// 68040 divide overflow (32- and 64-bit forms):
							// V=1, C=0, N/Z and the destination registers
							// are left unchanged
							sr[1] <= 1; sr[0] <= 0;
							fetch_next;
						end
						else begin
							rfw({1'b0, x_ext[14:12]}, md_rlo);  // quotient to Dq
							sr[3] <= md_rlo[31];
							sr[2] <= (md_rlo == 32'd0);
							sr[1] <= 0;
							sr[0] <= 0;
							if (x_ext[2:0] != x_ext[14:12]) state <= S_MD_WB2;
							else fetch_next;
						end
					end
					else begin
						rfw({1'b0, x_ext[14:12]}, md_rlo);  // low product to Dl
						if (x_ext[10]) begin
							sr[3] <= md_rhi[31];
							sr[2] <= (md_rhi == 32'd0) && (md_rlo == 32'd0);
							sr[1] <= 0; sr[0] <= 0;
							// the 68040 writes Dh before Dl (020/030 write Dl
							// first), so with Dh==Dl the register must keep
							// the LOW half: skip the high write
							if (x_ext[2:0] != x_ext[14:12]) state <= S_MD_WB2;
							else fetch_next;
						end
						else begin
							sr[3] <= md_rlo[31];
							sr[2] <= (md_rlo == 32'd0);
							sr[1] <= (x_ext[11] ? (md_rhi != {32{md_rlo[31]}})
							                    : (md_rhi != 32'd0));
							sr[0] <= 0;
							fetch_next;
						end
					end
				end
			end

			S_MD_WB2: begin
				rfw({1'b0, x_ext[2:0]}, md_rhi);   // remainder to Dr / high to Dh
				fetch_next;
			end

			//------------------------------------------------------ exceptions
			//------------------------------------- access error (format $7)
			S_AERR0: begin
				// The address-register rollback below must run BEFORE the S
				// bit changes the stack selection.  A7 is a shadowed
				// register -- writes reach USP, ISP or MSP according to
				// S/M -- so undoing an A7 update from a USER-mode
				// instruction after setting S restores the user value into
				// the SUPERVISOR pointer.  The frame is then stacked in
				// user space, that write faults, and the fault-during-
				// exception halts the core: the silent NetBSD freeze,
				// captured on hardware as A7=1dfff9b8 (a user stack) in
				// supervisor mode with IR=209f (MOVE.L (A7)+,(A0), libc's
				// __cerror storing through the errno pointer).  Roll back
				// first, in the faulting instruction's own context, and
				// only then enter the exception.
				sr_saved <= sr;
				aer_idx <= 0;
				state <= S_AERR_U;
			end

			S_AERR_U: begin
				// roll back address register updates so RTE restarts the
				// instruction from a clean context (68040 restart model)
				if (u1_v) begin
					rfw(u1_reg, u1_old);
					u1_v <= 0;
				end
				else if (u0_v) begin
					rfw(u0_reg, u0_old);
					u0_v <= 0;
				end
				else begin
					// rollback complete: now switch to supervisor state
					sr[13] <= 1;
					sr[15:14] <= 2'b00;
					in_exc <= 1;
					state <= S_AERR_SP;
				end
			end

			S_AERR_SP: begin
				aer_sp <= dbg_a7 - 32'd60;
				state <= S_AERR_WR;
			end

			S_AERR_WR: begin
				if (aer_idx == 5'd30) begin
					rfw(4'd15, aer_sp);
					exc_vec <= `AP040_VEC_BUSERR;
					state <= S_EXC_VEC;
				end
				else begin
					aer_idx <= aer_idx + 5'd1;
					mwr(aer_sp + {26'd0, aer_idx, 1'b0}, `AP040_SZ_W,
					    {16'd0, aerr_word(aer_idx)}, S_AERR_WR);
				end
			end

			// An abandoned queue fetch may still be on the bus under the
			// pre-exception function code, and fpu_bg may still be making the
			// FPU quiescent.  exc0_enter holds the encoded state until both
			// conditions clear, then snapshots the exception context.
			S_EXC0:    exc0_enter(4'd0);
			S_EXC0_F2: exc0_enter(4'd2);
			S_EXC0_F3: exc0_enter(4'd3);
			S_EXC0_F4: exc0_enter(4'd4);

			S_EXC1: begin
				exc_sp <= dbg_a7 - exc_fsize;
				mwr(dbg_a7 - exc_fsize,
				    `AP040_SZ_W, {16'd0, sr_saved}, S_EXC2);
			end

			S_EXC2: mwr(exc_sp + 32'd2, `AP040_SZ_L, exc_spc, S_EXC3);

			S_EXC3: mwr(exc_sp + 32'd6, `AP040_SZ_W,
			            {16'd0, exc_fmt, 2'b00, exc_vec, 2'b00},
			            (exc_fmt == 4'd2 || exc_fmt == 4'd3 ||
			             exc_fmt == 4'd4) ? S_EXC4 : S_EXC5);

			// Formats $2/$3 carry one additional longword.  Format $4,
			// recognized only by LC/EC configurations, carries two.
			S_EXC4: mwr(exc_sp + 32'd8, `AP040_SZ_L, exc_addr,
			            (exc_fmt == 4'd4) ? S_EXC4B : S_EXC5);

			S_EXC4B: mwr(exc_sp + 32'd12, `AP040_SZ_L, pc_i, S_EXC5);

			S_EXC5: begin
				// this A7 write commits on the next ce edge, while SR.M is
				// still set for the master stack case
				rfw(4'd15, exc_sp);
				if (exc_is_irq && sr[12] && !exc_pass2) state <= S_EXC6;
				else state <= S_EXC_VEC;
			end

			S_EXC6: begin
				// interrupt with M set: clear M and build a format $1
				// throwaway frame on the interrupt stack.  Its SR image is
				// the ORIGINAL SR with only S forced (WinUAE: regs.sr |=
				// 1<<13 before the second push): the original trace bits and
				// interrupt mask survive, and M stays set so that RTE's
				// format $1 continuation switches back to the master stack
				// where the real frame lives.
				sr[12] <= 0;
				sr_saved <= sr_saved | 16'h2000;
				exc_fmt <= 4'd1;
				exc_pass2 <= 1;
				state <= S_EXC1;
			end

			S_EXC_VEC: mrd(vbr + {22'd0, exc_vec, 2'b00}, `AP040_SZ_L, S_EXC_JMP);

			S_EXC_JMP: begin
				if (m_val[0]) begin
					if (exc_vec == 8'd2 || exc_vec == 8'd3) begin
						// odd bus/address error handler: double fault, halt
						fatal_halt;
					end
					else begin
						// Any other odd handler address becomes an address
						// error.  The frame's PC field identifies the vector
						// that supplied the odd address as its OFFSET --
						// 4 * vector, WITHOUT vbr -- not the original
						// exception's next-PC context.  WinUAE says so in
						// as many words on the path it models explicitly
						// ("offset, not vbr + offset").  Hardware settled
						// it: with cputest's own vbr ($403e4e68) a frame
						// built from vbr + 4*vec read $403e4e88 where the
						// corpus expects $00000020 for vector 8.  The
						// address field carries the odd target with A0
						// cleared.
						texc_pend <= 0;
						exc(`AP040_VEC_ADDRERR, 4'd2,
						    {22'd0, exc_vec, 2'b00},
						    {m_val[31:1], 1'b0});
					end
				end
				else if (texc_pend) begin
					// the surviving T0 trace: vector 9, format $2, stacked
					// PC = handler entry, address field = the instruction
					// that took the original exception.  WinUAE's DOTRACE
					// fires before a pending interrupt is sampled.
					texc_pend <= 0;
					pc <= m_val;
					exc(`AP040_VEC_TRACE, 4'd2, m_val, texc_pc);
				end
				else if (irq_pend) begin
					// An interrupt pending when another exception finishes is
					// stacked before the original handler executes.  Its frame
					// returns to that handler address.
					pc <= m_val;
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
					exc_spc <= m_val; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= irq_take_lvl;
					epf_flush;
					state <= S_EXC0;
				end
				else begin
					exception_prefetch(m_val, sr_s);
				end
			end

			//------------------------------------------------------------- RTE
			S_RTE_SR: begin
				mm_resume <= 0;
				mrd(dbg_a7, `AP040_SZ_W, S_RTE_PC);
			end
			S_RTE_PC:  begin rte_sr <= m_val[15:0]; mrd(dbg_a7 + 32'd2, `AP040_SZ_L, S_RTE_FMT); end
			S_RTE_FMT: begin rte_pc <= m_val; mrd(dbg_a7 + 32'd6, `AP040_SZ_W, S_RTE_FIN); end

			S_RTE_FIN: begin
				case (m_val[15:12])
					4'd0, 4'd1: begin
						rfw(4'd15, dbg_a7 + 32'd8);
						ret_kind <= {1'b0, m_val[12]};  // reuse: bit0 = again
						state <= S_RTE_FIN2;
					end
					4'd2, 4'd3: begin
						rfw(4'd15, dbg_a7 + 32'd12);
						ret_kind <= 2'b00;
						state <= S_RTE_FIN2;
					end
					4'd4: begin
						// A full MC68040 does not recognize format $4.  The
						// eight-word frame belongs to the LC/EC variants.
						if (AP040_HAS_FPU != 0)
							exc(`AP040_VEC_FMTERR, 4'd0, pc_i, 32'd0);
						else begin
							rfw(4'd15, dbg_a7 + 32'd16);
							ret_kind <= 2'b00;
							state <= S_RTE_FIN2;
						end
					end
					4'd7: begin
						mrd(dbg_a7 + 32'd12, `AP040_SZ_W, S_RTE_SSW);
					end
					default: exc(`AP040_VEC_FMTERR, 4'd0, pc_i, 32'd0);
				endcase
			end

			S_RTE_SSW: begin
				if (m_val[12]) mrd(dbg_a7 + 32'd8, `AP040_SZ_L, S_RTE_EA);
				else begin
					rfw(4'd15, dbg_a7 + 32'd60);
					ret_kind <= 2'b00;
					state <= S_RTE_FIN2;
				end
			end

			S_RTE_EA: begin
				mm_start_ea <= m_val;
				mm_resume <= 1;
				rfw(4'd15, dbg_a7 + 32'd60);
				ret_kind <= 2'b00;
				state <= S_RTE_FIN2;
			end

			S_RTE_FIN2: begin
				// CM returns into an unfinished instruction, not an interrupt/
				// trace boundary. Sample those again after MOVEM completes.
				sr <= rte_sr & `AP040_SR_MASK;
				if (ret_kind[0]) begin
					// format $1: continue with the next frame; the popped
					// SR becomes the "before" image for the odd-PC quirk
					state <= S_RTE_SR;
				end
				else if (rte_pc[0]) begin
					// The odd restored PC is detected after the RTE has
					// committed its SR.  Consequently the address-error frame
					// carries the restored SR, just as RTR carries its popped
					// CCR (68040_ae RTE/RTR corpus behavior).
					exc(`AP040_VEC_ADDRERR, 4'd2, pc_i,
					    {rte_pc[31:1], 1'b0});
				end
				else if (!mm_resume && (tr_t1 || tr_t0)) begin
					// the RTE itself was traced (T set before the RTE)
					tr_t1 <= 0;
					tr_t0 <= 0;
					pc <= rte_pc;
					exc(`AP040_VEC_TRACE, 4'd2, rte_pc, pc_i);
				end
				else if (!mm_resume && rte_irq_pend) begin
					// The restored mask unblocks a pending request: it is
					// taken AT this boundary, before the instruction RTE
					// returns to.  This path used to go straight to
					// S_FETCH without sampling interrupts at all -- unlike
					// fetch_next and go_pc -- so the target instruction ran
					// first and the interrupt was reported one instruction
					// late.  cputest enters every test through RTE, which
					// is why irq/all saw it on hardware while the
					// MOVE-to-SR path looked correct.
					in_exc <= 0;
					pc <= rte_pc;
					pc_i <= rte_pc;
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, rte_irq_lvl};
					exc_spc <= rte_pc; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= rte_irq_lvl;
					epf_flush;
					state <= S_POST_EXC;
				end
				else begin
					// fetch under the restored context's FC (SR is being
					// written this same cycle)
					in_exc <= 0;
					pc <= rte_pc;
					pc_i <= rte_pc;
					issue_ifetch(rte_pc, rte_sr[13]);
					state <= S_FETCH;
				end
			end

			//------------------------------------------------ RTS / RTR / RTD
			S_RET1: begin
				if (ret_kind == RK_RTR) mrd(dbg_a7, `AP040_SZ_W, S_RET2);
				else mrd(dbg_a7, `AP040_SZ_L, S_RET2);
			end

			S_RET2: begin
				case (ret_kind)
					RK_RTR: begin
						sr[4:0] <= m_val[4:0];
						mrd(dbg_a7 + 32'd2, `AP040_SZ_L, S_RET3);
					end
					// odd return address: the 68040 backs the pop out of A7
					// before taking the address error (gencpu cpu_level>=4
					// rolls areg7 back), so the fault frame sees the
					// pre-return stack pointer
					RK_RTD: begin
						if (!m_val[0]) rfw(4'd15, dbg_a7 + 32'd4 + sxw(imm[15:0]));
						go_pc(m_val);
					end
					default: begin
						if (!m_val[0]) rfw(4'd15, dbg_a7 + 32'd4);
						go_pc(m_val);
					end
				endcase
			end

			S_RET3: begin
				if (m_val[0]) begin
					// Odd return address: A7 keeps its pre-RTR value (the
					// 68040 backs the pop out before the address error,
					// cputest 68040_ae RTR round 0 checks A7 exactly), and
					// the frame stacks the SR with the CCR already popped in
					// S_RET2 -- the v20 corpus data validates that frame
					// byte.  (Newer WinUAE models a 68040 quirk stacking the
					// pre-RTR SR instead, exception3_read_prefetch_68040bug;
					// the v20 generator predates it, and the corpus is the
					// hardware acceptance test.)  The PC field identifies
					// the pre-opcode pipeline word.
					exc(`AP040_VEC_ADDRERR, 4'd2, pc_i,
					    {m_val[31:1], 1'b0});
				end
				else begin
					rfw(4'd15, dbg_a7 + 32'd6);
					go_pc(m_val);
				end
			end

			//------------------------------------------------------- branches
			S_BCC_EXT: begin : bcc_ext
				reg [31:0] tgt;
				tgt = br_base + (br_long ? imm : sxw(imm[15:0]));
				if (ir[11:8] == 4'h1) begin
					if (tgt[0]) go_pc(tgt); // odd target: fault BEFORE the push
					else begin
						br_tgt <= tgt;
						// the forwarded A7: dispatched from the pop
						// (dispatch_branch) this state runs while the
						// retiring instruction's A7 write is landing
						mwr(dbg_a7_wb - 32'd4, `AP040_SZ_L, pc, S_BSR_PUSH);
					end
				end
				else finish_bcc(tgt, cond_true(ir[11:8]));
			end

			S_BSR_PUSH: begin
				rfw(4'd15, dbg_a7 - 32'd4);
				go_pc(br_tgt);
			end

			S_DBCC1: begin : dbcc1
				// 68040 checks the branch-target parity BEFORE the
				// condition (gencpu cpu_level>=4 emits the odd test ahead
				// of cctrue): DBT to an odd label faults even though the
				// loop exits without branching (cputest 68040_ae DBcc.W).
				reg [31:0] tgt;
				reg [15:0] w;
				reg refill_hit;
				tgt = br_base + sxw(imm[15:0]);
				refill_hit = brf_tag == tgt[31:6] && brf_super == sr_s &&
				             (brf_run[tgt[5:1]] >= 4'd4);
				if (tgt[0]) go_pc(tgt);
				else if (cond_true(ir[11:8])) fetch_next;
				else begin
					// Decode selected Dn before fetching the displacement, so
					// its combinational read has already settled on entry here.
					// Commit the decrement and redirect together; go_pc's
					// interrupt barrier preserves this same-edge writeback.
					w = rf_rdata_a[15:0] - 16'd1;
					rfw({1'b0, d_rn}, {rf_rdata_a[31:16], w});
					if (w != 16'hFFFF) begin
						// The generic redirect keeps trace/interrupt priority.
						// Only the ordinary idle-bus loop case dispatches here.
						if (!tr_t1 && !tr_t0 && !irq_pend && refill_hit &&
						    ifr_avail &&
						    (!epf_armed || epf_next != tgt || epf_super != sr_s))
							decode_dbcc_brf(tgt);
						else
							go_pc(tgt);
					end
					else fetch_next;
				end
			end

			//------------------------------------------- jumps and stack frame
			S_JMP1:
				if (ea_addr[0])
					// gencpu's i_JMP does incpc(2) before
					// exception3_read_prefetch_only, and that path is NOT
					// gated on cpu_level, so the frame PC is measured from
					// wherever the PC had reached -- not from the
					// instruction address.  For (An), (d16,An) and absw
					// nothing has synced it yet, giving pc_i + 2; the
					// INDEXED modes resolve their extension against the
					// real PC first, so it has already advanced past the
					// extension word and the frame reads pc_i + 6.  Both
					// values are what the v24 AE group records.
					exc(`AP040_VEC_ADDRERR, 4'd2,
					    (ea_mode == 3'b110 ||
					     (ea_mode == 3'b111 && ea_rn == 3'd3))
					        ? pc_i + 32'd6 : pc_i + 32'd2,
					    {ea_addr[31:1], 1'b0});
				else go_pc(ea_addr);

			S_JSR1: begin
				if (ea_addr[0])
					// Unlike JMP, gencpu guards i_JSR's odd-target case with
					// cpu_level <= 1.  A 68040 therefore takes the fault on
					// the INSTRUCTION FETCH at the odd target, so the frame
					// names that target, not this instruction.
					exc(`AP040_VEC_ADDRERR, 4'd2, ea_addr,
					    {ea_addr[31:1], 1'b0});
				else begin
					br_tgt <= ea_addr;
					// the forwarded A7: see S_BCC_EXT
					mwr(dbg_a7_wb - 32'd4, `AP040_SZ_L, pc, S_JSR2);
					// P191: the target fetch now, behind the push, instead of
					// from S_JSR2 after its acknowledge (issue_ifetch does
					// nothing when the pop already armed the stream there)
					if (ifr_avail && !sr[15]) sgo = 1;
				end
			end

			// The redirect is issued here, one cycle after the push's
			// acknowledge, so the port is free and the target fetch goes
			// out at once: a redirect from inside the acknowledge cycle
			// is deferred to the fill engine, whose fill does not seed the
			// branch refill sector (v3, Sieve offset 0: +49 K cycles).
			// The push's undo record retires with the push: go_pc, unlike
			// fetch_next, does not clear it, and a fetch fault at the
			// target must not roll A7 back past the push.
			S_JSR2: begin
				rfw(4'd15, dbg_a7 - 32'd4);
				go_pc(br_tgt);
			end

			S_LEA1: begin
				rfw({1'b1, d_reg9}, ea_addr);
				fetch_next;
			end

			S_PEA1: mwr(dbg_a7 - 32'd4, `AP040_SZ_L, ea_addr, S_PEA2);

			S_PEA2: begin
				rfw(4'd15, dbg_a7 - 32'd4);
				fetch_next;
			end

			// LINK: decode selected An on port A; push it, and write An
			// with the same edge (undo-recorded so a faulting push
			// restarts with the old An).  A7 follows once the push has
			// completed.
			S_LINK2: begin : link2
				reg [31:0] spn;
				spn = dbg_a7 - 32'd4;
				t_a <= spn;
				mwr(spn, `AP040_SZ_L, (d_rn == 3'd7) ? spn : rf_rdata_a, S_LINK4);
				rfw({1'b1, d_rn}, spn);
				u_rec({1'b1, d_rn}, rf_rdata_a);
			end

			S_LINK4: begin
				rfw(4'd15, t_a + (br_long ? imm : sxw(imm[15:0])));
				fetch_next;
			end

			// UNLK: pop through An, write A7 with the read issue
			// (undo-recorded), retire in the read's acknowledge cycle.
			S_UNLK1: begin
				mrd(rf_rdata_a, `AP040_SZ_L, S_UNLK3);
				rfw(4'd15, rf_rdata_a + 32'd4);
				u_rec(4'd15, dbg_a7);
			end

			// Reached only through the byte-split (page-crossing) read
			// path, which returns to r_m_ret as a state; the aligned
			// case retires in the S_MRD acknowledge.
			S_UNLK3: begin
				rfw({1'b1, d_rn}, m_val);
				fetch_next;
			end

			//---------------------------------------------------------- MOVEM
			S_MOVEM_SET: begin
				mm_mask <= imm[15:0];
				// The manual uses the saved EA only for indexed/PC-relative
				// modes. Ordinary modes still calculate their address normally.
				if (d_mode != 3'b110 && !(d_mode == 3'b111 &&
				    (d_rn == 3'b010 || d_rn == 3'b011))) mm_resume <= 0;
				mm_idx_pend <= 0;
				// only the indexed modes carry one; every other mode reaches
				// here without having run S_EA_EXTW, so clear it explicitly
				if (d_mode != 3'b110 &&
				    !(d_mode == 3'b111 && d_rn == 3'b011)) mm_idx_en <= 0;
				// the EA depends on An for these modes: a LOADED base
				// register must not be written mid-loop (restart safety;
				// see S_MOVEM_LD)
				mm_base_ea <= (d_mode == 3'b010) || (d_mode == 3'b011) ||
				              (d_mode == 3'b101) || (d_mode == 3'b110);
				mm_base_pend <= 0;
				if (mm_predec || mm_postinc) begin
					rr_a <= {1'b1, d_rn};
					state <= S_MOVEM_SET2;
				end
				else ea_start(d_mode, d_rn, mm_size, S_MOVEM_EA);
			end

			S_MOVEM_SET2: begin
				mm_addr <= rf_rdata_a;
				mm_start_ea <= rf_rdata_a;
				mm_init_an <= rf_rdata_a;
				// a store loop reads its first register on port A from the
				// loop's first cycle (see S_MOVEM_LOOP)
				if (!mm_dir) rr_a <= mm_predec ? (4'd15 - ffs16(mm_mask)) : ffs16(mm_mask);
				state <= S_MOVEM_LOOP;
			end

			S_MOVEM_EA: begin
				mm_addr <= mm_resume ? mm_start_ea : ea_addr;
				if (!mm_resume) mm_start_ea <= ea_addr;
				mm_resume <= 0;
				if (!mm_dir) rr_a <= ffs16(mm_mask);
				state <= S_MOVEM_LOOP;
			end

			S_MOVEM_LOOP: begin
				if (mm_mask == 16'd0) begin
					// One write port, and the base and the index can both be
					// pending, so a held index takes a cycle of its own.
					if (mm_idx_pend) begin
						rfw(mm_idx_reg, mm_idx_val);
						mm_idx_pend <= 0;
						state <= S_MOVEM_FIN;
					end
					else begin
						if (mm_predec || mm_postinc)
							rfw({1'b1, d_rn}, mm_addr);
						else if (mm_base_pend)
							rfw({1'b1, d_rn}, mm_base_val);
						fetch_next;
					end
				end
				else begin : movem_step
					reg [3:0]  bit_i, bit_n, cur, nxt;
					reg [15:0] rest;
					reg [31:0] sz, sa, v;
					bit_i = ffs16(mm_mask);
					rest  = mm_mask & ~(16'd1 << bit_i);
					bit_n = ffs16(rest);
					sz  = (mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2;
					cur = mm_predec ? (4'd15 - bit_i) : bit_i;
					nxt = mm_predec ? (4'd15 - bit_n) : bit_n;
					mm_mask <= rest;
					mm_reg  <= cur;
					if (mm_dir && !mm_predec)
						// load: issued here (in place when the port is free),
						// retired on its acknowledge in S_MRD
						mrd(mm_addr, mm_size, S_MOVEM_LD);
					else begin
						// store: port A has shown this register since the
						// previous cycle (selected at loop entry, then here for
						// each next one), so the store issues from the loop
						// itself; S_MOVEM_RD is no longer entered.  The predec
						// form with the base register in its list stores the
						// initial value minus the size (68020+).  (2026-09-17)
						sa = mm_predec ? (mm_addr - sz) : mm_addr;
						v  = (mm_predec && cur == {1'b1, d_rn}) ? (mm_init_an - sz)
						                                        : rf_rdata_a;
						mwr(sa, mm_size, v, S_MOVEM_LOOP);
						mm_addr <= mm_predec ? sa : (sa + sz);
						rr_a <= nxt;
					end
				end
			end

			// The index was committed on the previous edge; finish exactly as
			// the loop exit would have.
			S_MOVEM_FIN: begin
				if (mm_predec || mm_postinc)
					rfw({1'b1, d_rn}, mm_addr);
				else if (mm_base_pend)
					rfw({1'b1, d_rn}, mm_base_val);
				fetch_next;
			end

			S_MOVEM_RD: begin : movem_rd
				reg [31:0] v;
				// predec MOVEM with the base register in the list: the
				// 68020/030/040 store the initial value minus the operation
				// size (the 68000/010 store the undecremented value)
				v = (mm_predec && mm_reg == {1'b1, d_rn})
				    ? (mm_init_an - ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2))
				    : rf_rdata_a;
				if (mm_predec) mwr(mm_addr, mm_size, v, S_MOVEM_LOOP);
				else begin
					mwr(mm_addr, mm_size, v, S_MOVEM_LOOP);
					mm_addr <= mm_addr + ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
				end
			end

			S_MOVEM_LD: begin : movem_ld
				reg [31:0] lv;
				lv = (mm_size == `AP040_SZ_W) ? sxw(m_val[15:0]) : m_val;
				// A loaded register that is also the EA base is not written
				// mid-loop: a fault on a LATER transfer restarts the whole
				// instruction and would recompute the EA from the loaded
				// DATA.  For (An)+ the final address writeback wins anyway
				// (the 040 leaves the postincremented address, not the
				// memory value); for the control modes the loaded value is
				// held and committed with the last transfer.
				if (mm_base_ea && mm_reg == {1'b1, d_rn}) begin
					if (!mm_postinc) begin
						mm_base_pend <= 1;
						mm_base_val  <= lv;
					end
				end
				else if (mm_idx_en && mm_reg == mm_idx_reg) begin
					mm_idx_pend <= 1;
					mm_idx_val  <= lv;
				end
				else rfw(mm_reg, lv);
				mm_addr <= mm_addr + ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
				state <= S_MOVEM_LOOP;
			end

			//---------------------------------------------------------- MOVEP
			S_MOVEP1: begin
				rr_a <= {1'b1, d_rn};
				rr_b <= {1'b0, d_reg9};
				state <= S_MOVEP2;
			end

			S_MOVEP2: begin
				mp_addr <= rf_rdata_a + sxw(imm[15:0]);
				mp_val <= rf_rdata_b;
				mp_idx <= 0;
				if (mp_dir) state <= S_MOVEP_WR;
				else state <= S_MOVEP_RD;
			end

			S_MOVEP_WR: begin
				if (mp_idx == mp_cnt) fetch_next;
				else begin : movep_wr
					reg [7:0] byv;
					case ({mp_cnt[2], mp_idx[1:0]})
						{1'b1, 2'd0}: byv = mp_val[31:24];
						{1'b1, 2'd1}: byv = mp_val[23:16];
						{1'b1, 2'd2}: byv = mp_val[15:8];
						{1'b1, 2'd3}: byv = mp_val[7:0];
						{1'b0, 2'd0}: byv = mp_val[15:8];
						default:      byv = mp_val[7:0];
					endcase
					mp_idx <= mp_idx + 3'd1;
					mwr(mp_addr + {28'd0, mp_idx[1:0], 1'b0}, `AP040_SZ_B,
					    {24'd0, byv}, S_MOVEP_WR);
				end
			end

			S_MOVEP_RD: begin
				if (mp_idx != 0) begin
					mp_val <= {mp_val[23:0], m_val[7:0]};
				end
				if (mp_idx == mp_cnt) begin : movep_fin
					reg [31:0] nv;
					nv = {mp_val[23:0], m_val[7:0]};
					if (mp_cnt[2]) rfw({1'b0, d_reg9}, nv);
					else rfw({1'b0, d_reg9}, {rf_rdata_b[31:16], nv[15:0]});
					fetch_next;
				end
				else begin
					mp_idx <= mp_idx + 3'd1;
					mrd(mp_addr + {28'd0, mp_idx[1:0], 1'b0}, `AP040_SZ_B, S_MOVEP_RD);
				end
			end

			//----------------------------------------------------- EXG / misc
			S_EXG1: begin
				t_a <= rf_rdata_a;
				rfw(rr_a, rf_rdata_b);
				state <= S_EXG2;
			end

			S_EXG2: begin
				rfw(rr_b, t_a);
				fetch_next;
			end

			S_USP1: begin
				aux_we <= 1; aux_sel <= 2'd0; aux_wdata <= rf_rdata_a;
				fetch_next;
			end

			//---------------------------------------------------------- MOVEC
			S_MOVEC1: begin
				epf_flush;
				if (!movec_valid(imm[11:0])) go_illegal;
				else if (mvc_dir) begin
					rr_a <= {imm[15], imm[14:12]};
					state <= S_MOVEC2;
				end
				else begin
					rfw({imm[15], imm[14:12]}, movec_rd(imm[11:0]));
					state <= S_NEXT;
				end
			end

			// A killed queue fetch may still be on the bus (with its own
			// request channel the engine issues right up to the MOVEC, P171);
			// an MMU register write under it would make the MMU re-translate
			// an accepted request, so the write waits for it to retire, as
			// PTEST/PFLUSH do.
			S_MOVEC2: if (epf_pend) epf_flush;
			else begin
				epf_flush;      // control-register access serializes fetch
				case (imm[11:0])
					12'h000: sfc <= rf_rdata_a[2:0];
					12'h001: dfc <= rf_rdata_a[2:0];
					12'h002: cacr <= rf_rdata_a & 32'h8000_8000;
					12'h003: begin
						tc <= rf_rdata_a & 32'h0000_C000;
					end
					12'h004: itt0 <= rf_rdata_a & 32'hFFFF_E364;
					12'h005: itt1 <= rf_rdata_a & 32'hFFFF_E364;
					12'h006: dtt0 <= rf_rdata_a & 32'hFFFF_E364;
					12'h007: dtt1 <= rf_rdata_a & 32'hFFFF_E364;
					12'h800: begin aux_we <= 1; aux_sel <= 2'd0; aux_wdata <= rf_rdata_a; end
					12'h801: vbr <= rf_rdata_a;
					12'h803: begin aux_we <= 1; aux_sel <= 2'd2; aux_wdata <= rf_rdata_a; end
					12'h804: begin aux_we <= 1; aux_sel <= 2'd1; aux_wdata <= rf_rdata_a; end
					12'h805: mmusr <= rf_rdata_a;
					12'h806: urp <= rf_rdata_a & 32'hFFFF_FE00;
					default: srp <= rf_rdata_a & 32'hFFFF_FE00;
				endcase
				// MMU register accesses never invalidate either ATC.  Software
				// must issue PFLUSH explicitly when a register write changes a
				// translation (MC68040 UM 3.7.4).
				state <= S_NEXT;
			end

			//---------------------------------------------------------- MOVES
			S_MOVES1: begin
				x_ext <= imm;
				ea_start(d_mode, d_rn, op_size, S_MOVES2);
			end

			S_MOVES2: begin
				if (x_ext[11]) begin
					rr_a <= {x_ext[15], x_ext[14:12]};
					state <= S_MOVES_WR;
				end
				else begin
					rr_b <= {x_ext[15], x_ext[14:12]};   // old value for merge
					fc_ovr_v <= 1; fc_ovr <= sfc;
					mrd(ea_addr, op_size, S_MOVES_RD);
				end
			end

			S_MOVES_WR: begin
				fc_ovr_v <= 1; fc_ovr <= dfc;
				mwr(ea_addr, op_size, rf_rdata_a, S_NEXT);
			end

			S_MOVES_RD: begin
				if (x_ext[15])
					rfw({x_ext[15], x_ext[14:12]},
					    (op_size == `AP040_SZ_W) ? sxw(m_val[15:0]) :
					    (op_size == `AP040_SZ_B) ? sxb(m_val[7:0]) : m_val);
				else
					rfw({x_ext[15], x_ext[14:12]},
					    merge_sz(rf_rdata_b, m_val, op_size));
				fetch_next;
			end

			//------------------------------------------------- PTEST / PFLUSH
			S_PTEST1: begin
				epf_flush;      // PTEST replaces the matching ATC entry
				pt_addr <= rf_rdata_a;
				pt_write <= ~ir[5];
				state <= S_PTEST2;
			end

			// The table walker has its own memory port.  An abandoned queue
			// fetch may still be on the CPU bus, so the probe waits for it to
			// retire rather than running two masters at once.
			S_PTEST2: if (!pt_req) begin
				if (!epf_pend) pt_req <= 1;
			end
			else if (pt_done) begin
				pt_req <= 0;
				mmusr <= pt_mmusr;
				fetch_next;
			end

			S_PFLUSH1: begin
				epf_flush;
				pf_addr <= rf_rdata_a;
				state <= S_PFLUSH2;
			end

			S_PFLUSH2: if (!pf_req) begin
				if (!epf_pend) pf_req <= 1;
			end
			else if (pf_done) begin
				pf_req <= 0;
				fetch_next;
			end

			S_CINV2: if (cinv_done) begin
				cinv_req <= 0;
				fetch_next;
			end

			//----------------------------------------------------- CHK2/CMP2
			S_CHK2_A: begin
				x_ext <= imm;
				ea_start(d_mode, d_rn, op_size, S_CHK2_B);
			end

			S_CHK2_B: begin
				dst_addr <= ea_addr;
				mrd(ea_addr, op_size, S_CHK2_C);
			end

			S_CHK2_C: begin
				src_val <= m_val;              // lower bound
				rr_a <= {x_ext[15], x_ext[14:12]};
				mrd(dst_addr + ((op_size == `AP040_SZ_B) ? 32'd1 :
				                (op_size == `AP040_SZ_W) ? 32'd2 : 32'd4),
				    op_size, S_CHK2_D);
			end

			S_CHK2_D: begin : chk2d
				reg signed [31:0] rn, lb, ub;
				reg oob;
				// operands sign-extended by size; address registers use
				// their full value
				if (x_ext[15]) rn = $signed(rf_rdata_a);
				else rn = (op_size == `AP040_SZ_B) ? $signed(sxb(rf_rdata_a[7:0])) :
				          (op_size == `AP040_SZ_W) ? $signed(sxw(rf_rdata_a[15:0])) :
				          $signed(rf_rdata_a);
				lb = (op_size == `AP040_SZ_B) ? $signed(sxb(src_val[7:0])) :
				     (op_size == `AP040_SZ_W) ? $signed(sxw(src_val[15:0])) :
				     $signed(src_val);
				ub = (op_size == `AP040_SZ_B) ? $signed(sxb(m_val[7:0])) :
				     (op_size == `AP040_SZ_W) ? $signed(sxw(m_val[15:0])) :
				     $signed(m_val);
				oob = (lb <= ub) ? (rn < lb || rn > ub) : (rn < lb && rn > ub);
				sr[2] <= (rn == lb) || (rn == ub);
				sr[0] <= oob;
				if (x_ext[11] && oob)
					exc(`AP040_VEC_CHK, 4'd2, pc, pc_i);
				else fetch_next;
			end

			//------------------------------------------------- BTST Dn,#imm
			S_BTSTI: begin
				x_ext <= imm;
				rr_a <= p_sreg;
				state <= S_BTSTI2;
			end

			S_BTSTI2: begin
				// The immediate destination is byte-sized, so the dynamic
				// bit number is modulo 8.  Explicitly widen the array index.
				sr[2] <= ~x_ext[{2'b00, rf_rdata_a[2:0]}];
				fetch_next;
			end

			//------------------------------------------------------------ CAS2
			// x_ext[31:16] = first, x_ext[15:0] = second extension word;
			// not bus locked (single CPU master on this fabric)
			S_CAS2_0: begin
				x_ext <= imm;
				rr_a <= {imm[31], imm[30:28]};   // Rn1 (address)
				rr_b <= {imm[15], imm[14:12]};   // Rn2
				state <= S_CAS2_1;
			end

			S_CAS2_1: begin
				t_a <= rf_rdata_a;
				t_b <= rf_rdata_b;
				mrd(rf_rdata_a, op_size, S_CAS2_2);
			end

			S_CAS2_2: begin
				bf_w1 <= m_val;                  // first memory operand
				mrd(t_b, op_size, S_CAS2_3);
			end

			S_CAS2_3: begin
				bf_field <= m_val;               // second memory operand
				rr_a <= {1'b0, x_ext[18:16]};    // Dc1
				rr_b <= {1'b0, x_ext[2:0]};      // Dc2
				state <= S_CAS2_4;
			end

			S_CAS2_4: begin
				cas_dc <= rf_rdata_a;
				bf_du <= rf_rdata_b;
				src_val <= rf_rdata_a;           // ALU: mem1 - Dc1
				dst_val <= bf_w1;
				state <= S_CAS2_5;
			end

			S_CAS2_5: begin
				sr[4:0] <= alu_fl;
				// CMP-only decisions avoid the general shift/result flag mux.
				if (alu_fast_fl[2]) begin
					src_val <= bf_du;            // ALU: mem2 - Dc2
					dst_val <= bf_field;
					state <= S_CAS2_6;
				end
				else state <= S_CAS2_F;
			end

			S_CAS2_6: begin
				sr[4:0] <= alu_fl;
				// CMP-only decisions avoid the general shift/result flag mux.
				if (alu_fast_fl[2]) begin
					rr_a <= {1'b0, x_ext[24:22]};   // Du1
					rr_b <= {1'b0, x_ext[8:6]};     // Du2
					state <= S_CAS2_W2;
				end
				else state <= S_CAS2_F;
			end

			S_CAS2_W2: mwr(t_a, op_size, rf_rdata_a, S_CAS2_W3);

			S_CAS2_W3: mwr(t_b, op_size, rf_rdata_b, S_NEXT);

			S_CAS2_F: begin
				rfw({1'b0, x_ext[18:16]}, merge_sz(cas_dc, bf_w1, op_size));
				state <= S_CAS2_F2;
			end

			S_CAS2_F2: begin
				rfw({1'b0, x_ext[2:0]}, merge_sz(bf_du, bf_field, op_size));
				fetch_next;
			end

			//------------------------------------------- FSAVE / FRESTORE
			// NULL frame ($00000000) when untouched, 4-byte revision-specific
			// IDLE frame, or the 44/52-byte UNIMP frame retained by the FPU.
			// The generic EA engine has
			// already adjusted -(An) by one longword; extend that adjustment
			// to the complete exception frame before issuing any writes.
			S_FSAVE1: begin
				// wait for a background op -- and for the one-cycle frame
				// preparation that follows its deferred-exception retire
				// (fpu_pendcap), or the pend would be judged frameless
				if (fpu_bg || fpu_pendcap) state <= S_FSAVE1;
				else if (fpu_pend_exc && !fpu_fstate_unimp) begin
					// Frameless fallback: a pend whose frame state is gone
					// because an earlier FSAVE already extracted it
					// (fsave_ack) or an FRESTORE of IDLE replaced it.
					// Capture arms fstate_unimp for BOTH classes -- e1 to
					// the $30 frame, e3 to the $41/$60 BUSY frame -- so a
					// pend that still owns its frame takes one of the two
					// branches below and is EXTRACTED, as on the real 040.
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else if (fpu_fstate_unimp && fpu_fstate_busy) begin
					// an e3 arithmetic pend extracts as the 100-byte
					// revision-specific $60 BUSY frame
					fpu_pend_exc <= 0;
					t_a <= (ea_mode == 3'b100) ? ea_addr - 32'd96 : ea_addr;
					if (ea_mode == 3'b100)
						rfw({1'b1, ea_rn}, ea_addr - 32'd96);
					fpb_n <= 0;
					state <= S_FSAVE_B;
				end
				else if (fpu_fstate_unimp) begin
					// pending state (unimplemented instruction, or a
					// prepared/lingering arithmetic e1 frame) is extracted
					// into the $28/$30 frame; extraction consumes the pend
					fpu_pend_exc <= 0;
					t_a <= (ea_mode == 3'b100) ? ea_addr - (FPU_UNIMP_BYTES - 32'd4) : ea_addr;
					if (ea_mode == 3'b100)
						rfw({1'b1, ea_rn}, ea_addr - (FPU_UNIMP_BYTES - 32'd4));
					fp_n <= 0;
					state <= S_FSAVE_U;
				end
				else mwr(ea_addr, `AP040_SZ_L,
				             fpu_used ? FPU_IDLE_HEADER : 32'h0000_0000, S_NEXT);
			end

			S_FSAVE_U:
				mwr(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L,
				    fsave_unimp_word(fp_n), S_FSAVE_UD);

			S_FSAVE_UD: begin
				if (fp_n == FPU_UNIMP_LAST) begin
					// Do not acknowledge/lose the pending state until the final
					// bus write has completed successfully.
					fpu_fsave_ack <= 1;
					fetch_next;
				end
				else begin
					fp_n <= fp_n + 4'd1;
					state <= S_FSAVE_U;
				end
			end

			S_FSAVE_B:
				mwr(t_a + {25'd0, fpb_n, 2'b00}, `AP040_SZ_L,
				    fsave_busy_word(fpb_n), S_FSAVE_BD);

			S_FSAVE_BD: begin
				if (fpb_n == 5'd24) begin
					fpu_fsave_ack <= 1;
					fetch_next;
				end
				else begin
					fpb_n <= fpb_n + 5'd1;
					state <= S_FSAVE_B;
				end
			end

			S_FREST1:
				if (fpu_bg) state <= S_FREST1;       // wait for background op
				else mrd(ea_addr, `AP040_SZ_L, S_FREST2);

			S_FREST2: begin
				// version byte 0 = NULL frame: reset the FPU state.
				// Match the configured revision for IDLE, UNIMP and BUSY.
				// Read the complete 44/52-byte UNIMP payload so
				// bus faults remain precise before installing any FPU state.
				// A completed FRESTORE replaces the FPU state wholesale, so
				// a pending deferred exception from the OLD context is
				// discarded with it (on silicon the pending state lives
				// inside the FPU; FRESTORE neither reports it -- only FSAVE
				// is exempt from reporting, and WinUAE's fpuop_restore
				// never calls fp_exception_pending -- nor may it leak into
				// the new context, whose FPIAR is already reset).
				if (m_val[31:24] == 8'd0) begin
					fpu_rst <= 1;
					fpu_pend_exc <= 0;
					fetch_next;
				end
				else if (m_val == FPU_IDLE_HEADER) begin
					fpu_frestore_idle <= 1;
					fpu_pend_exc <= 0;
					fetch_next;
				end
				else if (m_val == FPU_UNIMP_HEADER) begin
					fp_restore_busy <= 0;
					// $40 has no CMDREG3B; do not inherit it from a prior BUSY.
					if (FPU_REV40) fp_restore_cmd3 <= 0;
					fp_n <= 4'd1;
					mrd(ea_addr + 32'd4, `AP040_SZ_L, S_FREST_U);
				end
				else if (m_val == FPU_BUSY_HEADER) begin
					fp_restore_busy <= 1;
					fpb_n <= 5'd1;
					mrd(ea_addr + 32'd4, `AP040_SZ_L, S_FREST_B);
				end
				else exc(`AP040_VEC_FMTERR, 4'd0, pc_i, 32'd0);
			end

			S_FREST_U: begin
				case (FPU_REV40 ? fp_n + 4'd2 : fp_n)
					4'd1: fp_restore_cmd3 <= m_val[31:16];
					4'd3: begin
					fp_restore_stag <= m_val[31:29];
					fp_restore_grs  <= m_val[25:23];
				end
					4'd4: fp_restore_cmd1 <= m_val[31:16];
					4'd5: begin
					fp_restore_dtag   <= m_val[31:29];
					fp_restore_wbte15 <= m_val[20];
				end
					4'd6: fp_restore_flags <= {m_val[26], m_val[25], m_val[20]};
					4'd7: fp_restore_fpt[95:64] <= m_val;
					4'd8: fp_restore_fpt[63:32] <= m_val;
					4'd9: fp_restore_fpt[31:0] <= m_val;
					4'd10: fp_restore_et[95:64] <= m_val;
					4'd11: fp_restore_et[63:32] <= m_val;
					4'd12: fp_restore_et[31:0] <= m_val;
					default: ; // reserved longword at offset $08
				endcase
				if (fp_n == FPU_UNIMP_LAST) state <= S_FREST_UD;
				else begin
					fp_n <= fp_n + 4'd1;
					mrd(ea_addr + ({28'd0, fp_n} << 2) + 32'd4,
					    `AP040_SZ_L, S_FREST_U);
				end
			end

			S_FREST_B: begin
				case (fpb_n)
					5'd2:  fp_restore_cusavepc <= m_val[31:24];
					5'd6:  fp_restore_wbt[95:64] <= m_val;
					5'd7:  fp_restore_wbt[63:32] <= m_val;
					5'd8:  fp_restore_wbt[31:0]  <= m_val;
					5'd10: fp_restore_fpiar <= m_val;
					5'd13: fp_restore_cmd3 <= m_val[31:16];
					5'd15: begin
						fp_restore_stag <= m_val[31:29];
						fp_restore_et15 <= m_val[28];
						fp_restore_grs  <= m_val[25:23];
					end
					5'd16: fp_restore_cmd1 <= m_val[31:16];
					5'd17: begin
						fp_restore_dtag   <= m_val[31:29];
						fp_restore_fpt15  <= m_val[28];
						fp_restore_wbte15 <= m_val[20];
					end
					5'd18: fp_restore_flags <= {m_val[26], m_val[25], m_val[20]};
					5'd19: fp_restore_fpt[95:64] <= m_val;
					5'd20: fp_restore_fpt[63:32] <= m_val;
					5'd21: fp_restore_fpt[31:0]  <= m_val;
					5'd22: fp_restore_et[95:64]  <= m_val;
					5'd23: fp_restore_et[63:32]  <= m_val;
					5'd24: fp_restore_et[31:0]   <= m_val;
					default: ;
				endcase
				if (fpb_n == 5'd24) state <= S_FREST_BD;
				else begin
					fpb_n <= fpb_n + 5'd1;
					mrd(ea_addr + ({27'd0, fpb_n} << 2) + 32'd4,
					    `AP040_SZ_L, S_FREST_B);
				end
			end

			S_FREST_BD: begin
				if (ea_mode == 3'b011)
					rfw({1'b1, ea_rn}, ea_addr + 32'd100);
				fpu_frestore_unimp <= 1;
				// CU_SAVEPC=$fe resumes the prepared arithmetic command.
				// Wait through the existing background interlock before the
				// next FP dispatch/FSAVE; completion may re-arm a NEW exception.
				// All frame reads have succeeded before any execution starts.
				fpu_bg <= fpu_frestore_resume;
				fpu_pend_exc <= !fpu_frestore_resume && fpu_frestore_e1_pend;
				fpu_pend_vec <= fpu_cur_vec;
				fetch_next;
			end

			S_FREST_UD: begin
				fp_restore_busy <= 0;
				if (ea_mode == 3'b011)
					rfw({1'b1, ea_rn}, ea_addr + FPU_UNIMP_BYTES);
				fpu_frestore_unimp <= 1;
				// see S_FREST2: a completed FRESTORE discards the old
				// context's pending deferred exception -- and a restored
				// arithmetic E1 frame re-arms one for the NEW context,
				// vectored by the (already restored) FPSR/FPCR enables,
				// delivered pre-instruction at the next FPU dispatch
				fpu_pend_exc <= fpu_frestore_e1_pend;
				fpu_pend_vec <= fpu_cur_vec;
				fetch_next;
			end

			//------------------------------------------------------------- FPU
			S_FPU_DEC: begin
				// P208: the background operation's done pulse (never raised
				// with an enabled exception; the FPU is back in F_IDLE) is
				// retirement enough: fpu_bg itself clears on this same edge
				if (fpu_bg && !fpu_done) begin
					// hold the dispatch until the background FPU
					// operation has retired
				end
				else if (fpu_pend_exc) begin
					// pre-instruction delivery of the pending enabled
					// arithmetic exception, FPSP style: the stacked PC is
					// the FPU instruction being dispatched, FPIAR still
					// identifies the faulting one
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else begin
				fpu_class <= imm[15:13];
				fpu_opm   <= imm[6:0];
				fpu_fmt   <= imm[12:10];
				fpu_srcr  <= imm[12:10];
				fpu_dstr  <= imm[9:7];
				fp_nb     <= fp_bytes(imm[12:10]);
				fp_st     <= 0;
				fp_st_epend <= 0;
				fp_n      <= 0;
				fp_ea_pd  <= 0;
				fp_ea_v   <= 0;
				fp_ea_pi  <= 0;
				case (imm[15:13])
					3'b000: begin
						// FPm to FPn general.  Nonexisting opmodes fault
						// before any side effect: no FPIAR update, no FPSR
						// status clear, plain F-line (or, for $78-$7F, the
						// integer illegal vector).
						if (fp_opmode_class(imm[6:0]) == 2'd1) go_fp_fline;
						else if (fp_opmode_class(imm[6:0]) == 2'd2) go_illegal;
						else begin
							fpu_iawe <= 1;
							fpu_req <= 1;
							state <= S_FPU_GO;
						end
					end
					3'b001: go_fp_fline;   // undefined opclass
					3'b010: begin
						// <ea>{fmt} to FPn general.  The opmode check does
						// not apply to FMOVECR, whose low bits are a ROM
						// offset rather than an opmode.
						if (imm[12:10] != 3'd7 &&
						    fp_opmode_class(imm[6:0]) == 2'd1) go_fp_fline;
						else if (imm[12:10] != 3'd7 &&
						         fp_opmode_class(imm[6:0]) == 2'd2) go_illegal;
						else if (imm[12:10] == 3'd7) begin
							// FMOVECR: no EA; not hardware on the 040
							fpu_iawe <= 1;
							fpu_req <= 1;
							state <= S_FPU_GO;
						end
						else if (d_mode == 3'b000) begin
							// A data-register EA only supplies a 32-bit value.  D/X/P
							// source formats are malformed FPU commands and take the
							// normal F-line vector, not the integer illegal-op vector.
							if (fp_bytes(imm[12:10]) > 4'd4) begin
								// A recognized FP operation with an unimplemented Dn
								// source format still records its instruction address
								// before taking vector 11.  Allow that side-port write
								// to commit before exception entry snapshots FPIAR.
								// ...except a PACKED source, which WinUAE
								// rejects outright for a Dn EA (get_fp_value
								// case 0 size 3 returns 0 on the 040 without
								// consulting the opmode at all).
								fpu_iawe <= 1;
								go_fp_ea_fault(fp_op_in_hw(imm[6:0]) ||
								               imm[12:10] == 3'd3);
								state <= (fp_op_in_hw(imm[6:0]) ||
								          imm[12:10] == 3'd3) ?
								         S_POST_EXC : S_POST_EXC_F2;
							end
							else begin
								rr_a <= {1'b0, d_rn};
								fpu_iawe <= 1;
								state <= S_FPU_DREG;
							end
						end
						else if (d_mode == 3'b001) begin
							// Arithmetic opmode validation precedes source-EA
							// validation on the 040.  An is not a legal source, but
							// the recognized command has already updated FPIAR --
							// and an FPSP-emulated opmode reports through the
							// unimplemented-instruction route even here.
							fpu_iawe <= 1;
							go_fp_ea_fault(fp_op_in_hw(imm[6:0]));
							state <= fp_op_in_hw(imm[6:0]) ?
							         S_POST_EXC : S_POST_EXC_F2;
						end
						else if (ea_is_imm) begin
							fpb <= 0;
							state <= S_FPU_IMM;
						end
						// P181: (An) too -- S_FPU_AN resolves it in one state
						// (no pre/post adjust: fp_ea_pd/pi stay 0) instead of
						// S_EA_DISP + S_FPU_EA.
						else if ((d_mode == 3'b011 || d_mode == 3'b100 || d_mode == 3'b010) && fpu_an_now) begin
							// P223: S_FPU_AN's work here, the base being on port A
							fp_adj   <= fpu_an_adj;
							fp_ea_v  <= 1;
							fp_ea_pd <= (d_mode == 3'b100);
							fp_ea_pi <= (d_mode == 3'b011);
							t_a      <= (d_mode == 3'b100) ? (rf_capture_a - {25'd0, fpu_an_adj}) : rf_capture_a;
							state    <= S_FPU_RD;
						end
						else if (d_mode == 3'b011 || d_mode == 3'b100 || d_mode == 3'b010) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
					3'b011: begin
						// FMOVE FPn,<ea>{fmt}; FPIAR is updated on the paths
						// that actually engage the FPU, not on the malformed
						// encodings that F-line out
						fpu_srcr <= imm[9:7];
						fp_st <= 1;
						// Packed stores must engage the FPU after EA resolution.
						// It prepares the BUSY frame (command, source, E1/T) that
						// the FPSP needs; directly raising vector 55 here leaves
						// FSAVE reporting IDLE and the handler reading stale stack.
						if (d_mode == 3'b000) begin
							// Packed output is a datatype fault even when the
							// nominal destination is Dn.  Datatype classification
							// wins over the unimplemented-EA check and, with no
							// addressable destination, the format-$3 EA is zero.
							if (imm[12:10] == 3'd3 || imm[12:10] == 3'd7) begin
								fpu_iawe <= 1;
								fpu_req <= 1;
								state <= S_FPU_GO;
							end
							// A data register cannot hold a double or
							// extended result: the 68040 reports these as
							// unimplemented FP instructions, not as integer
							// illegal instructions (WinUAE put_fp_value:
							// "68040+ generates unimplemented effective mode
							// exception even if destination EA is Dn or An")
							else if (fp_bytes(imm[12:10]) > 4'd4) begin
								// rejected store destination: F-line, and as
								// with the An/PC-relative cases above the 040
								// records no FPIAR for it
								go_fp_fline;
								state <= S_POST_EXC;
							end
							else begin
								rr_a <= {1'b0, d_rn};
								fpu_iawe <= 1;
								fpu_req <= 1;
								state <= S_FPU_GO;
							end
						end
						else if (d_mode == 3'b001) begin
							// An is not a legal FMOVE-out destination.  FPIAR is
							// NOT written: for opclass 011 the 040 only records
							// it once the store's EA has been accepted (WinUAE
							// fpuop_arithmetic case 3 reaches maybe_set_fpiar
							// only after put_fp_value succeeds; an An
							// destination returns through fpu_noinst first).
							go_fp_fline;
							state <= S_POST_EXC;
						end
						else if (dst_not_alt ||
						         (d_mode == 3'b111 && d_rn[1])) begin
							// Non-alterable or PC-relative destination: F-line
							// with NO FPIAR side effect.  Same rule as the An
							// destination above -- opclass 011 records FPIAR
							// only once put_fp_value has accepted the store.
							go_fp_fline;
							state <= S_POST_EXC;
						end
						// P203: an (An) destination too, as P181 did for the
						// sources (FMOVE.X FPn,(A7) in the ROM's FPU glue)
						else if ((d_mode == 3'b011 || d_mode == 3'b100 || d_mode == 3'b010) && fpu_an_now) begin
							// P223: S_FPU_AN's work here, the base being on port A
							fp_adj   <= fpu_an_adj;
							fp_ea_v  <= 1;
							fp_ea_pd <= (d_mode == 3'b100);
							fp_ea_pi <= (d_mode == 3'b011);
							t_a      <= (d_mode == 3'b100) ? (rf_capture_a - {25'd0, fpu_an_adj}) : rf_capture_a;
							fpu_iawe <= 1;
							fpu_req  <= 1;
							state    <= S_FPU_GO;
						end
						else if (d_mode == 3'b011 || d_mode == 3'b100 || d_mode == 3'b010) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
					3'b100, 3'b101: begin : fp_crm
						// FMOVEM control registers.  An empty selection means
						// FPIAR (WinUAE: "All control register bits unset =
						// FPIAR"), and every malformed combination is an
						// F-line trap, not an integer illegal instruction.
						reg [6:0] cnt;
						reg [2:0] crsel;
						reg       multi;
						crsel = (imm[12:10] == 3'd0) ? 3'b001 : imm[12:10];
						multi = (crsel != 3'b100) && (crsel != 3'b010) &&
						        (crsel != 3'b001);
						cnt = ({6'd0, crsel[2]} + {6'd0, crsel[1]} +
						       {6'd0, crsel[0]}) << 2;
						fp_creg <= crsel;
						fp_st <= imm[13];
						if (imm[13]) t0_force <= 1; // FMOVEM control regs to memory
						fp_nb <= cnt[3:0];
						fp_adj <= cnt;
						if (d_mode == 3'b000) begin
							// Dn: a single register only
							if (multi) go_fp_fline;
							else begin
								rr_a <= {1'b0, d_rn};
								state <= S_FPU_CRD;
							end
						end
						else if (d_mode == 3'b001) begin
							// An: only FPIAR may be transferred
							if (crsel != 3'b001) go_fp_fline;
							else begin
								rr_a <= {1'b1, d_rn};
								state <= S_FPU_CRD;
							end
						end
						else if (ea_is_imm) begin
							// an immediate source may load several registers
							// back to back; an immediate destination is a
							// malformed encoding
							if (imm[13]) go_fp_fline;
							else immf(2'd2, S_FPU_CRI);
						end
						else if (imm[13] && d_mode == 3'b111 && d_rn[1])
							go_fp_fline;   // PC-relative destination
						else if (d_mode == 3'b011 || d_mode == 3'b100) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
					default: begin : fp_mvm
						// FMOVEM FP register list, 12 bytes per register.
						// 68040 EA legality: stores reject (An)+ and the
						// PC-relative modes, loads reject -(An); Dn, An and
						// immediate F-line in both directions.
						// 68040 ordering quirks (WinUAE fmovem2mem):
						//   loads always map mask bit 7 to FP0;
						//   a store whose mask convention disagrees with its
						//   EA direction writes each register's three longs
						//   in REVERSED order (low mantissa first);
						//   predec stores consume the mask LSB first so the
						//   ascending walk reproduces the descending layout.
						reg [6:0] cnt;
						reg is_st;
						cnt = ({6'd0, imm[7]} + {6'd0, imm[6]} + {6'd0, imm[5]} +
						       {6'd0, imm[4]} + {6'd0, imm[3]} + {6'd0, imm[2]} +
						       {6'd0, imm[1]} + {6'd0, imm[0]}) * 7'd12;
						is_st = (imm[15:13] == 3'b111);
						fp_mode <= imm[12:11];
						fp_st <= is_st;
						if (is_st) t0_force <= 1;
						fp_list <= imm[7:0];
						fp_adj <= cnt;
						fp_lsb <= is_st && (d_mode == 3'b100);
						fp_rev <= is_st && (imm[12] == (d_mode == 3'b100));
						if (d_mode < 3'b010 || ea_is_imm) go_fp_fline;
						else if (is_st && (d_mode == 3'b011 ||
						         (d_mode == 3'b111 && d_rn[1]))) go_fp_fline;
						else if (!is_st && d_mode == 3'b100) go_fp_fline;
						else if (imm[11]) begin
							// dynamic list in a data register
							rr_a <= {1'b0, imm[6:4]};
							state <= S_FPU_MVML;
						end
						else if ((d_mode == 3'b011 || d_mode == 3'b100) && fpu_an_now) begin
							// P224: S_FPU_AN's work here (static list, base on port A)
							fp_adj   <= cnt;
							fp_ea_v  <= 1;
							fp_ea_pd <= (d_mode == 3'b100);
							fp_ea_pi <= (d_mode == 3'b011);
							t_a      <= (d_mode == 3'b100) ? (rf_capture_a - {25'd0, cnt}) : rf_capture_a;
							state    <= S_FPU_MVM;
						end
						else if (d_mode == 3'b011 || d_mode == 3'b100) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
				endcase
				end
			end

			S_FPU_MVML: begin : fp_mvml
				// latch the dynamic FMOVEM list, then resolve the EA
				reg [6:0] cnt;
				cnt = ({6'd0, rf_rdata_a[7]} + {6'd0, rf_rdata_a[6]} +
				       {6'd0, rf_rdata_a[5]} + {6'd0, rf_rdata_a[4]} +
				       {6'd0, rf_rdata_a[3]} + {6'd0, rf_rdata_a[2]} +
				       {6'd0, rf_rdata_a[1]} + {6'd0, rf_rdata_a[0]}) * 7'd12;
				fp_list <= rf_rdata_a[7:0];
				fp_adj <= cnt;
				if (d_mode == 3'b011 || d_mode == 3'b100) begin
					rr_a <= {1'b1, d_rn};
					state <= S_FPU_AN;
				end
				else if (d_mode < 3'b010 || ea_is_imm) go_fp_fline;
				else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
			end

			S_FPU_AN: begin : fp_an
				// (An)+ / -(An): manual base handling, register written
				// back only at successful completion (restart safe)
				reg [6:0] adj;
				adj = (fp_nb == 4'd1 && d_rn == 3'd7) ? 7'd2 : {3'b000, fp_nb};
				if (fpu_class[2] == 1'b0 && fpu_class != 3'b010 &&
				    fpu_class != 3'b011) adj = fp_adj;   // never taken; clarity
				if (fpu_class == 3'b100 || fpu_class == 3'b101 ||
				    fpu_class == 3'b110 || fpu_class == 3'b111)
					adj = fp_adj;
				fp_adj <= adj;
				fp_ea_v <= 1;
				fp_ea_pd <= (d_mode == 3'b100);
				fp_ea_pi <= (d_mode == 3'b011);
				t_a <= (d_mode == 3'b100) ? (rf_rdata_a - {25'd0, adj})
				                          : rf_rdata_a;
				case (fpu_class)
					3'b010: state <= S_FPU_RD;
					3'b011: begin
						fpu_iawe <= 1;
						fpu_req <= 1;
						state <= S_FPU_GO;
					end
					3'b100, 3'b101: state <= S_FPU_CR;
					default: state <= S_FPU_MVM;
				endcase
			end

			S_FPU_EA: begin
				t_a <= ea_addr;
				fp_ea_v <= 1;
				case (fpu_class)
					3'b010: state <= S_FPU_RD;
					3'b011: begin
						fpu_iawe <= 1;
						fpu_req <= 1;
						state <= S_FPU_GO;
					end
					3'b100, 3'b101: state <= S_FPU_CR;
					default: state <= S_FPU_MVM;
				endcase
			end

			S_FPU_DREG: begin
				// data register source, left aligned by format
				case (fpu_fmt)
					3'd4: fpb <= {rf_rdata_a[15:0], 80'd0};
					3'd6: fpb <= {rf_rdata_a[7:0], 88'd0};
					default: fpb <= {rf_rdata_a, 64'd0};
				endcase
				fpu_iawe <= 1;
				fpu_req <= 1;
				state <= S_FPU_GO;
			end

			S_FPU_IMM: begin
				// immediate operand: words arrive via the imm register
				if (fp_n != 4'd0) begin
					if (fp_nb == 4'd2) fpb[95:80] <= imm[15:0];
					else if (fp_nb == 4'd1) fpb[95:88] <= imm[7:0];
					else case (fp_n)
						4'd1: fpb[95:64] <= imm[31:0];
						4'd2: fpb[63:32] <= imm[31:0];
						default: fpb[31:0] <= imm[31:0];
					endcase
				end
				if ((fp_nb <= 4'd2 && fp_n != 4'd0) ||
				    (fp_nb == 4'd4 && fp_n == 4'd1) ||
				    (fp_nb == 4'd8 && fp_n == 4'd2) ||
				    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
					fpu_iawe <= 1;
					fpu_req <= 1;
					state <= S_FPU_GO;
				end
				else begin
					fp_n <= fp_n + 4'd1;
					immf((fp_nb <= 4'd2) ? 2'd1 : 2'd2, S_FPU_IMM);
				end
			end

			S_FPU_RD: begin
				// memory operand read loop
				if (fp_n != 4'd0) begin
					if (fp_nb == 4'd1) fpb[95:88] <= m_val[7:0];
					else if (fp_nb == 4'd2) fpb[95:80] <= m_val[15:0];
					else case (fp_n)
						4'd1: fpb[95:64] <= m_val;
						4'd2: fpb[63:32] <= m_val;
						default: fpb[31:0] <= m_val;
					endcase
				end
				if ((fp_nb <= 4'd4 && fp_n != 4'd0) ||
				    (fp_nb == 4'd8 && fp_n == 4'd2) ||
				    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
					fpu_iawe <= 1;
					fpu_req <= 1;
					state <= S_FPU_GO;
				end
				else begin
					if (fp_nb == 4'd1)
						mrd(t_a, `AP040_SZ_B, S_FPU_RD);
					else if (fp_nb == 4'd2)
						mrd(t_a, `AP040_SZ_W, S_FPU_RD);
					else
						mrd(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L, S_FPU_RD);
					fp_n <= fp_n + 4'd1;
				end
			end

			S_FPU_GO: begin
				// Both FPSP routes -- unimplemented INSTRUCTION and
				// unsupported DATA TYPE -- report after the operand has
				// been fetched, so the (An)+ / -(An) update stands.
				// WinUAE applies it in get_fp_value when the EA is
				// computed, ahead of either check, and only the 68060
				// takes it back (mmufixup).  Withholding it left the
				// handler pointed at an operand already consumed.
				if (fpu_unimp) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					go_fp_unimp;
				end
				else if (fpu_unsupp) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					go_fp_unsupp(fp_st, 1'b0, fp_ea_v,
					               fp_ea_v ? t_a : 32'd0);
				end
				else if (fpu_exc_req && !fp_st) begin
					fpu_req <= 0;
					exc(fpu_exc_vec, 4'd0, pc, pc_i);
				end
				else if (fpu_accepted && !fp_st) begin
					// register-destination arithmetic past every datatype
					// check: release it to the background and continue
					// integer execution.  Post-increment/-decrement address
					// register updates do not depend on the result.
					fpu_bg <= 1;
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					fetch_next;
				end
				else if (fpu_done) begin
					if (!fp_st) begin
						if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
						else if (fp_ea_pi)
							rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
						fetch_next;
					end
					else if (fpu_exc_req &&
					         (fpu_exc_vec == `AP040_VEC_FP_SNAN ||
					          fpu_exc_vec == `AP040_VEC_FP_OPERR) &&
					         (fpu_fmt == 3'd0 || fpu_fmt == 3'd4 ||
					          fpu_fmt == 3'd6)) begin
						// Enabled integer-store SNAN/OPERR: the 040 does not
						// write the destination (WinUAE
						// fault_if_68040_integer_nonmaskable returns before
						// the store).  Post-instruction format $3, EA = the
						// operand address, 0 for a Dn destination.  The
						// (An)+/-(An) update still commits: the instruction
						// completed, only the store is suppressed.
						fpu_req <= 0;
						if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
						else if (fp_ea_pi)
							rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
						exc(fpu_exc_vec, 4'd3, pc,
						    (d_mode == 3'b000) ? 32'd0 : t_a);
					end
					else if (d_mode == 3'b000) begin : fp_stdn
						// store to a data register with size merge
						case (fpu_fmt)
							3'd4: rfw({1'b0, d_rn},
							          {rf_rdata_a[31:16], fpu_dout[95:80]});
							3'd6: rfw({1'b0, d_rn},
							          {rf_rdata_a[31:8], fpu_dout[95:88]});
							default: rfw({1'b0, d_rn}, fpu_dout[95:64]);
						endcase
						// enabled float-format exception on a Dn store: the
						// destination IS written (WinUAE put_fp_value, then
						// fpsr_check_arithmetic_exception), then the trap is
						// post-instruction with EA = 0
						if (fpu_exc_req) begin
							fpu_req <= 0;
							exc(fpu_exc_vec, 4'd3, pc, 32'd0);
						end
						else fetch_next;
					end
					else begin
						fp_n <= 0;
						// float-format memory store with an enabled
						// exception: write memory first, deliver after the
						// last write (WinUAE order: put_fp_value completes,
						// then fp_exception_pending(false))
						fp_st_epend <= fpu_exc_req;
						fp_st_evec  <= fpu_exc_vec;
						if (fp_nb >= 4'd4) begin
							// P225: the first longword goes out from here (hinted by
							// hint_st_fpgo), not a clock later from S_FPU_WR
							mwr(t_a, `AP040_SZ_L, fpu_dout[95:64], S_FPU_WR);
							fp_n <= 4'd1;
						end
						else state <= S_FPU_WR;
					end
				end
			end

			S_FPU_WR: begin
				// memory store loop from the FPU result
				if ((fp_nb <= 4'd4 && fp_n != 4'd0) ||
				    (fp_nb == 4'd8 && fp_n == 4'd2) ||
				    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					if (fp_st_epend) begin
						// post-instruction delivery of the enabled store
						// exception, after the destination was written
						fp_st_epend <= 0;
						exc(fp_st_evec, 4'd3, pc, t_a);
					end
					else fetch_next;
				end
				else begin
					if (fp_nb == 4'd1)
						mwr(t_a, `AP040_SZ_B, {24'd0, fpu_dout[95:88]}, S_FPU_WR);
					else if (fp_nb == 4'd2)
						mwr(t_a, `AP040_SZ_W, {16'd0, fpu_dout[95:80]}, S_FPU_WR);
					else begin : fp_wrl
						reg [31:0] wv;
						case (fp_n)
							4'd0: wv = fpu_dout[95:64];
							4'd1: wv = fpu_dout[63:32];
							default: wv = fpu_dout[31:0];
						endcase
						mwr(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L, wv, S_FPU_WR);
					end
					fp_n <= fp_n + 4'd1;
				end
			end

			S_FPU_CRD: begin
				// single control register, data or address register operand
				fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
				if (!fp_st) begin
					fpu_crwe <= 1;
					fpu_crwd <= rf_rdata_a;
					fetch_next;
				end
				else state <= S_FPU_CR2;
			end

			S_FPU_CRI: begin : fp_cri
				// FMOVEM.L #imm,<control list>: one longword per selected
				// register, consumed in FPCR, FPSR, FPIAR order
				reg [2:0] rest;
				rest = fp_creg[2] ? {1'b0, fp_creg[1:0]} :
				       fp_creg[1] ? {fp_creg[2], 1'b0, fp_creg[0]} :
				                    3'b000;
				fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
				fpu_crwe <= 1;
				fpu_crwd <= imm;
				fp_creg <= rest;
				if (rest != 3'd0) immf(2'd2, S_FPU_CRI);
				else fetch_next;
			end

			S_FPU_CR2: begin
				// Control-register read is valid one cycle after crsel.  A
				// register-direct FMOVE (Dn, or An for FPIAR) completes
				// here; an FMOVEM list emits the selected long and returns
				// to the list sequencer.
				if (d_mode == 3'b000 || d_mode == 3'b001) begin
					rfw({d_mode[0], d_rn}, fpu_crrd);
					fetch_next;
				end
				else mwr(t_a, `AP040_SZ_L, fpu_crrd, S_FPU_CR);
			end

			S_FPU_CR: begin : fp_cr
				// control register list transfer, FPCR/FPSR/FPIAR order
				if (fp_n[0]) begin
					// completion of the previous long
					if (!fp_st) begin
						fpu_crwe <= 1;
						fpu_crwd <= m_val;
					end
					t_a <= t_a + 32'd4;
					fp_n <= 0;
				end
				else if (fp_creg == 3'd0) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a - {25'd0, fp_adj});
					else if (fp_ea_pi) rfw({1'b1, d_rn}, t_a);
					fetch_next;
				end
				else begin
					fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
					fp_creg <= fp_creg[2] ? {1'b0, fp_creg[1:0]} :
					           fp_creg[1] ? {fp_creg[2], 1'b0, fp_creg[0]} :
					                        {fp_creg[2:1], 1'b0};
					fp_n <= 4'd1;
					if (fp_st) state <= S_FPU_CR2;   // wait for crrd
					else mrd(t_a, `AP040_SZ_L, S_FPU_CR);
				end
			end

			S_FPU_MVM: begin : fp_mvm_sel
				// FMOVEM register loop.  Loads map mask bit 7 to FP0
				// regardless of the mode field (68040); stores follow the
				// mode's convention.  Predec stores walk the mask from the
				// LSB so the ascending address walk reproduces the layout
				// of the hardware's descending one.
				reg [2:0] b;
				reg found;
				integer j;
				found = 0; b = 0;
				for (j = 7; j >= 0; j = j - 1)
					if (!found && fp_list[fp_lsb ? (3'd7 - j[2:0]) : j[2:0]]) begin
						b = fp_lsb ? (3'd7 - j[2:0]) : j[2:0];
						found = 1;
					end
					if (!found) begin
						if (fp_ea_pd)
							rfw({1'b1, d_rn}, t_a - {25'd0, fp_adj});
						else if (fp_ea_pi) rfw({1'b1, d_rn}, t_a);
					fetch_next;
				end
				else begin
					fp_list <= fp_list & ~(8'd1 << b);
					fpu_fmsel <= (!fp_st || fp_mode[1]) ? (3'd7 - b) : b;
					// FMOVEM stores share the FPU's ordinary source-register
					// read port.  S_FPU_MVM2 provides a full setup cycle before
					// fm_rdata is consumed; loads still write through fm_sel.
					if (fp_st)
						fpu_srcr <= (!fp_st || fp_mode[1]) ? (3'd7 - b) : b;
					if (fp_ea_pd) t_a <= t_a;   // base already lowered
					if (!fp_st) begin
						// P227: a load's first longword goes out from here
						// (no register read to set up; hinted by hint_fpu_mvm0)
						mrd(t_a, `AP040_SZ_L, S_FPU_MVM3);
						fp_n <= 4'd1;
					end
					else begin
						fp_n <= 0;
						state <= S_FPU_MVM2;
					end
				end
			end

			S_FPU_MVM2: begin
				// one register = three longs; fm_rdata valid here
				if (fp_n == 4'd3) begin
					if (!fp_st) begin
						fpu_fmwe <= 1;
						fpu_fmwd <= fpb;
					end
					t_a <= t_a + 32'd12;
					if (fp_list == 8'd0) begin
						// P226: the last register -- finish here as S_FPU_MVM
						// would a clock later (t_a has not advanced yet)
						if (fp_ea_pd)
							rfw({1'b1, d_rn}, t_a + 32'd12 - {25'd0, fp_adj});
						else if (fp_ea_pi) rfw({1'b1, d_rn}, t_a + 32'd12);
						fetch_next;
					end
					else state <= S_FPU_MVM;
				end
				else begin : fp_mvm_x
					reg [31:0] wv;
					case (fp_rev ? (4'd2 - fp_n) : fp_n)
						4'd0: wv = fpu_fmrd[95:64];
						4'd1: wv = fpu_fmrd[63:32];
						default: wv = fpu_fmrd[31:0];
					endcase
					if (fp_st)
						mwr(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L, wv,
						    S_FPU_MVM3);
					else
						mrd(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L,
						    S_FPU_MVM3);
					fp_n <= fp_n + 4'd1;
				end
			end

			S_FPU_MVM3: begin
				if (!fp_st) case (fp_n)
					4'd1: fpb[95:64] <= m_val;
					4'd2: fpb[63:32] <= m_val;
					default: fpb[31:0] <= m_val;
				endcase
				state <= S_FPU_MVM2;
			end

			//--------------------------------------- FBcc / FScc / FDBcc
			S_FBCC: begin : fbcc
				if (fpu_bg) state <= S_FBCC;         // wait for background op
				else if (fpu_pend_exc) begin
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else begin : fbcc_run
				// the 6-bit predicate field aliases: WinUAE's fpp_cond masks
				// with 0x1f, so bit 5 has no effect and is NOT a trap
				// (table68k defines FBcc for all 64 encodings)
				reg [31:0] disp;
				disp = ir[6] ? imm : sxw(imm[15:0]);
				if (ir[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1;
					exc(`AP040_VEC_FP_BSUN, 4'd0, pc_i, 32'd0);
				end
				else begin
					if (ir[4] && fpu_cc[0]) fpu_bsun <= 1;
					if (fp_cond(ir[5:0], fpu_cc))
						go_pc(pc_i + 32'd2 + disp);
					else fetch_next;
				end
				end
			end

			S_FSCC0: begin
				if (fpu_bg) state <= S_FSCC0;        // wait for background op
				else if (fpu_pend_exc) begin
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else begin : fscc0_run
				fp_pred <= imm[5:0];
				// On the 68040 FDBcc, FScc and FTRAPcc record the command
				// address in FPIAR once their extension word has decoded.  FBcc
				// is the exception: it leaves FPIAR alone on the normal path.
				// Do this before EA processing so the side effect also precedes
				// a later operand/access exception.
				fpu_iawe <= 1;
				if (d_mode == 3'b001) begin
					// FDBcc Dn,disp
					t0_force <= 1;       // every FDBcc is T0-traced on 040
					rr_a <= {1'b0, d_rn};
					immf(2'd1, S_FDBCC);
				end
				else if (d_mode == 3'b111 && d_rn == 3'b010)
					immf(2'd1, S_FSCC1);        // FTRAPcc.W
				else if (d_mode == 3'b111 && d_rn == 3'b011)
					immf(2'd2, S_FSCC1);        // FTRAPcc.L
				else if (d_mode == 3'b111 && d_rn == 3'b100)
					state <= S_FSCC1;           // FTRAPcc
				else if (d_mode == 3'b000) begin
					rr_a <= {1'b0, d_rn};
					state <= S_FSCC1;
				end
				else if (ea_is_imm || (d_mode == 3'b111 && d_rn[1]))
					go_fp_fline;
				else ea_start(d_mode, d_rn, `AP040_SZ_B, S_FSCC1);
				end
			end

			S_FSCC1: begin : fscc1
				reg c;
				c = fp_cond(fp_pred, fpu_cc);
				if (fp_pred[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1;
					exc_now(`AP040_VEC_FP_BSUN, 4'd0, pc_i, 32'd0);
					// fpu_bsun is a side port sampled on the following edge.
					// Let it commit before exception entry snapshots FPSR.
					state <= S_POST_EXC;
				end
				else begin
					if (fp_pred[4] && fpu_cc[0]) fpu_bsun <= 1;
				if (d_mode == 3'b111 && (d_rn == 3'b010 || d_rn == 3'b011 ||
				                         d_rn == 3'b100)) begin
					// FTRAPcc
					if (c) begin
						exc_now(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);
						// A signaling unordered predicate records BSUN even when
						// disabled.  Delay entry so that FPSR write is visible.
						if (fp_pred[4] && fpu_cc[0]) state <= S_POST_EXC_F2;
					end
					else fetch_next;
				end
				else if (d_mode == 3'b000) begin
					rfw({1'b0, d_rn}, {rf_rdata_a[31:8], {8{c}}});
					fetch_next;
				end
				else mwr(ea_addr, `AP040_SZ_B, {24'd0, {8{c}}}, S_NEXT);
				end
			end

			S_FDBCC: begin : fdbcc
				reg [15:0] cnt;
				if (fp_pred[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1;
					exc_now(`AP040_VEC_FP_BSUN, 4'd0, pc_i, 32'd0);
					state <= S_POST_EXC;
				end
				else begin
					if (fp_pred[4] && fpu_cc[0]) fpu_bsun <= 1;
					if (fp_cond(fp_pred, fpu_cc)) fetch_next;
					else begin
						cnt = rf_rdata_a[15:0] - 16'd1;
						rfw({1'b0, d_rn}, {rf_rdata_a[31:16], cnt});
						if (cnt != 16'hFFFF)
							go_pc(pc_i + 32'd4 + sxw(imm[15:0]));
						else fetch_next;
					end
				end
			end

			S_RESET_HOLD: begin
				epf_flush;
				if (rst_cnt == 8'd0) fetch_next;
				else rst_cnt <= rst_cnt - 8'd1;
			end

			//--------------------------------------------------------- MOVE16
			S_M16_SRC: begin
				if (m16_form == 3'd4) m16_dst_rn <= imm[14:12];
				rr_a <= {1'b1, d_rn};
				state <= S_M16_DST;
			end

			S_M16_DST: begin
				m16_an <= rf_rdata_a;
				case (m16_form)
					3'd0, 3'd2: begin  // (An)[+] to abs
						m16_src <= rf_rdata_a & 32'hFFFF_FFF0;
						m16_dst <= imm & 32'hFFFF_FFF0;
					end
					3'd1, 3'd3: begin  // abs to (An)[+]
						m16_src <= imm & 32'hFFFF_FFF0;
						m16_dst <= rf_rdata_a & 32'hFFFF_FFF0;
					end
					default: begin     // (Ax)+ to (Ay)+
						m16_src <= rf_rdata_a & 32'hFFFF_FFF0;
						rr_b <= {1'b1, m16_dst_rn};
					end
				endcase
				state <= (m16_form == 3'd4) ? S_M16_DST2 : S_M16_RD;
				m16_idx <= 0;
				m16_rd_done <= 0;
			end

			S_M16_DST2: begin
				m16_dst <= rf_rdata_b & 32'hFFFF_FFF0;
				t_b <= rf_rdata_b;
				state <= S_M16_RD;
			end

			S_M16_RD: mrd(m16_src + {28'd0, m16_idx, 2'b00}, `AP040_SZ_L, S_M16_RD2);

			S_M16_RD2: begin
				m16buf[m16_idx] <= m_val;
				if (m16_idx == 2'd3) begin
					m16_idx <= 0;
					state <= S_M16_WR;
				end
				else begin
					m16_idx <= m16_idx + 2'd1;
					state <= S_M16_RD;
				end
			end

			S_M16_WR: mwr(m16_dst + {28'd0, m16_idx, 2'b00}, `AP040_SZ_L,
			              m16buf[m16_idx], S_M16_WR2);

			S_M16_WR2: begin
				if (m16_idx == 2'd3) state <= S_M16_INC;
				else begin
					m16_idx <= m16_idx + 2'd1;
					state <= S_M16_WR;
				end
			end

			S_M16_INC: begin
				case (m16_form)
					3'd0, 3'd1: begin  // (An)+ forms
						rfw({1'b1, d_rn}, m16_an + 32'd16);
						fetch_next;
					end
					3'd4: begin
						rfw({1'b1, d_rn}, m16_an + 32'd16);
						if (m16_dst_rn != d_rn) state <= S_M16_INC2;
						else fetch_next;
					end
					default: fetch_next;
				endcase
			end

			S_M16_INC2: begin
				rfw({1'b1, m16_dst_rn}, t_b + 32'd16);
				fetch_next;
			end

			//------------------------------------------------------ bitfields
			S_BF0: begin
				x_ext <= imm;
				if (imm[11]) rr_a <= {1'b0, imm[8:6]};   // offset from Dn
				if (imm[5])  rr_b <= {1'b0, imm[2:0]};   // width from Dn
				state <= S_BF1;
			end

			S_BF1: begin
				bf_off <= x_ext[11] ? rf_rdata_a : {27'd0, x_ext[10:6]};
				bf_w <= x_ext[5] ? ((rf_rdata_b[4:0] == 5'd0) ? 6'd32 : {1'b0, rf_rdata_b[4:0]})
				                 : ((x_ext[4:0] == 5'd0) ? 6'd32 : {1'b0, x_ext[4:0]});
				if (d_mode == 3'b000) begin
					rr_a <= {1'b0, d_rn};
					state <= S_BF_REG;
				end
				else ea_start(d_mode, d_rn, `AP040_SZ_B, S_BF_MEM0);
			end

			S_BF_REG: begin
				dst_val <= rf_rdata_a;    // register operand
				if (ir[10:8] == 3'd7) begin
					rr_a <= {1'b0, x_ext[14:12]};   // BFINS source
					state <= S_BF_REG2;
				end
				else state <= S_BF_REGX;
			end

			S_BF_REG2: begin
				bf_du <= rf_rdata_a;
				state <= S_BF_REGX;
			end

			S_BF_REGX: begin
				// stage 1: rotate the operand so the field is left aligned,
				// then share the memory form's extract/insert stages
				bf_t40 <= {bf_rot, 8'd0};
				state <= S_BF_M2;
			end

			S_BF_X4: begin
				// stage 4: rotate back (the shared rotator turns right by
				// rotating left by -offset) and write the register
				rfw({1'b0, d_rn}, bf_rot);
				fetch_next;
			end

			S_BF_MEM0: begin : bf_mem0
				reg [2:0] span;
				span = ({3'd0, bf_off[2:0]} + bf_w + 6'd7) >> 3;
				bf_addr <= ea_addr + {{3{bf_off[31]}}, bf_off[31:3]};
				bf_bib <= bf_off[2:0];
				bf_span <= span;
				if (ir[10:8] == 3'd7) rr_b <= {1'b0, x_ext[14:12]};
				// Only read bytes containing the field.  A short field at a
				// page end must not fault on an unmapped following page
				// (OPENSTEP's WindowServer, BFTST 3(A1){6:2} at the last mapped
				// byte; Adam Polkosnik 3458e64, t_bitfield_mmu).
				mrd(ea_addr + {{3{bf_off[31]}}, bf_off[31:3]},
				    (span == 3'd1) ? `AP040_SZ_B :
				    (span <= 3'd3) ? `AP040_SZ_W : `AP040_SZ_L, S_BF_MEM1);
			end

			S_BF_MEM1: begin
				// m_val is right aligned for byte/word reads; the bitfield
				// datapath consumes a left-aligned 40-bit memory window.
				case (bf_span)
					3'd1: bf_w1 <= {m_val[7:0], 24'd0};
					3'd2, 3'd3: bf_w1 <= {m_val[15:0], 16'd0};
					default: bf_w1 <= m_val;
				endcase
				bf_w2 <= 8'd0;
				bf_du <= rf_rdata_b;
				if (bf_span == 3'd3) mrd(bf_addr + 32'd2, `AP040_SZ_B, S_BF_MEM2);
				else if (bf_span == 3'd5) mrd(bf_addr + 32'd4, `AP040_SZ_B, S_BF_MEM2);
				else state <= S_BF_EXECM;
			end

			S_BF_MEM2: begin
				if (bf_span == 3'd3) bf_w1[15:8] <= m_val[7:0];
				else bf_w2 <= m_val[7:0];
				state <= S_BF_EXECM;
			end

			S_BF_EXECM: begin
				// stage 1: left align the window on the field start bit
				bf_t40 <= {bf_w1, bf_w2} << bf_bib;
				state <= S_BF_M2;
			end

			S_BF_M2: begin
				// stage 2: extract the field; width masks in the t40 domain
				bf_field <= (bf_w == 6'd32) ? bf_t40[39:8]
				                            : (bf_t40[39:8] >> (6'd32 - bf_w));
				bf_ones <= (bf_w == 6'd32) ? 32'hFFFF_FFFF
				                           : ((32'd1 << bf_w) - 32'd1);
				bf_maskl <= (bf_w == 6'd32) ? {32'hFFFF_FFFF, 8'd0}
				                            : ({32'hFFFF_FFFF, 8'd0} << (6'd32 - bf_w));
				state <= S_BF_M3;
			end

			S_BF_M3: begin : bf_m3
				reg [31:0] nf;
				nf = bf_newf(ir[10:8], bf_field, bf_du & bf_ones, bf_ones);
				sr[3] <= (ir[10:8] == 3'd7) ? nf[bf_w - 6'd1] : bf_field[bf_w - 6'd1];
				sr[2] <= (ir[10:8] == 3'd7) ? (nf == 32'd0) : (bf_field == 32'd0);
				sr[1] <= 0;
				sr[0] <= 0;
				case (ir[10:8])
					3'd0: fetch_next;
					3'd1: begin rfw({1'b0, x_ext[14:12]}, bf_field); fetch_next; end
					3'd3: begin
						rfw({1'b0, x_ext[14:12]},
						    bf_field | (bf_field[bf_w - 6'd1] ? ~bf_ones : 32'd0));
						fetch_next;
					end
					3'd5: begin : bfffo_m
						reg [31:0] al;
						al = bf_t40[39:8] & bf_maskl[39:8];
						rfw({1'b0, x_ext[14:12]},
						    bf_off + {26'd0, (al == 32'd0) ? bf_w : clz32(al)});
						fetch_next;
					end
					default: begin
						// stage 3: substitute the new field, still left aligned
						bf_t40 <= (bf_t40 & ~bf_maskl) |
						          ((({nf, 8'd0}) << (6'd32 - bf_w)) & bf_maskl);
						state <= (d_mode == 3'b000) ? S_BF_X4 : S_BF_M4;
					end
				endcase
			end

			S_BF_M4: begin : bf_m4
				// stage 4: shift back into the memory window; the top bf_bib
				// bits of the original window pass through unchanged
				reg [39:0] head, nw40;
				head = ~(40'hFF_FFFF_FFFF >> bf_bib);
				nw40 = ({bf_w1, bf_w2} & head) | (bf_t40 >> bf_bib);
				bf_w1 <= nw40[39:8];
				bf_w2 <= nw40[7:0];
				state <= S_BF_WR1;
			end

			S_BF_WR1: begin
				case (bf_span)
					3'd1: mwr(bf_addr, `AP040_SZ_B, {24'd0, bf_w1[31:24]}, S_NEXT);
					3'd2: mwr(bf_addr, `AP040_SZ_W, {16'd0, bf_w1[31:16]}, S_NEXT);
					3'd3: mwr(bf_addr, `AP040_SZ_W, {16'd0, bf_w1[31:16]}, S_BF_WR2);
					3'd4: mwr(bf_addr, `AP040_SZ_L, bf_w1, S_NEXT);
					default: mwr(bf_addr, `AP040_SZ_L, bf_w1, S_BF_WR2);
				endcase
			end

			S_BF_WR2: begin
				if (bf_span == 3'd3)
					mwr(bf_addr + 32'd2, `AP040_SZ_B, {24'd0, bf_w1[15:8]}, S_NEXT);
				else
					mwr(bf_addr + 32'd4, `AP040_SZ_B, {24'd0, bf_w2}, S_NEXT);
			end

			//------------------------------------------------------------ CAS
			S_CAS1: begin
				x_ext <= imm;
				ea_start(d_mode, d_rn, op_size, S_CAS2);
			end

			S_CAS2: begin
				dst_addr <= ea_addr;
				rr_a <= {1'b0, x_ext[2:0]};   // Dc
				rr_b <= {1'b0, x_ext[8:6]};   // Du
				mrd(ea_addr, op_size, S_CAS3);
			end

			S_CAS3: begin
				src_val <= rf_rdata_a;   // Dc: ALU computes operand - Dc
				dst_val <= m_val;
				cas_dc <= rf_rdata_a;
				bf_du <= rf_rdata_b;
				state <= S_CAS4;
			end

			S_CAS4: begin
				sr[4:0] <= alu_fl;
				// CMP-only decisions avoid the general shift/result flag mux.
				if (alu_fast_fl[2])
					mwr(dst_addr, op_size, bf_du, S_NEXT);   // equal: update
				else begin
					rfw({1'b0, x_ext[2:0]}, merge_sz(cas_dc, dst_val, op_size));
					fetch_next;
				end
			end

			//----------------------------------------- immediate to CCR / SR
			S_SROP: begin : srop
				reg [15:0] nv;
				case (srop_kind)
					2'd0: nv = sr | imm[15:0];
					2'd1: nv = sr & imm[15:0];
					default: nv = sr ^ imm[15:0];
				endcase
				if (srop_sr) begin
					// let SR settle before the next dispatch (see DK_SR)
					sr <= nv & `AP040_SR_MASK;
					state <= S_NEXT;
				end
				else begin
					sr[4:0] <= nv[4:0];
					fetch_next;
				end
			end

			//---------------------------------------------------------- decode
`ifdef AP040_EXPERIMENTAL_PIPELINE
            S_PIPE_LOAD_RETURN: state <= S_EXPERIMENT_PIPE;
            S_EXPERIMENT_PIPE: begin
                // The pipeline offers memory only as older WB commits or later.
                // Reuse mrd/mwr for MMU checks, page splits and format-7 faults.
                // Fault context belongs to this load, not the advanced IF PC.
                if (pipe_load_req && !pipe_load_active) begin
                    pc_i <= pipe_load_pc;
                    ir <= pipe_load_opcode;
                    if (pipe_load_write) begin
                        // Match the sequencer's MOVE-store fault CCR. An
                        // updates remain deferred until successful retirement.
                        sr[4:0] <= pipe_load_ccr;
                        mwr(pipe_load_addr, pipe_load_size, pipe_load_wdata, S_PIPE_LOAD_RETURN);
                    end else mrd(pipe_load_addr, pipe_load_size, S_PIPE_LOAD_RETURN);
                end
                if (pipe_input && pipe_ready) begin
                    epf_pop = pipe_words;
                    pc <= pc + {29'd0, pipe_words, 1'b0};
                    perf_dispatch_toggle <= ~perf_dispatch_toggle;
                end
                if (pipe_retire) begin
                    if (!(pipe_load_launch && pipe_load_write)) sr[4:0] <= pipe_ccr;
                    // A same-edge memory launch owns the fault context and
                    // MOVE-store flags, even while older WB commits.
                    if (!pipe_load_launch) begin
                        pc_i <= pipe_pc;
                        ir <= pipe_opcode;
                    end
                end
                if (pipe_cancel) begin
                    // WB commits, ID/EX die. The exception frame names the
                    // first unexecuted instruction, not the advanced IF PC.
                    pc <= pipe_next_pc;
                    fetch_next;
                end else if (pipe_branch_taken) begin
                    go_pc(pipe_next_pc);
                end else if (pipe_exit_ready && !pipe_input) begin
                    // The last WB may commit on this edge when early drain is
                    // enabled. The next legacy state sees its RF pending-write
                    // bypass and settled CCR. Keep the usual IRQ/trace boundary.
                    fetch_next;
                end
            end
`endif
			S_DECODE: begin
`ifdef AP040_EXPERIMENTAL_PIPELINE
                if (pipe_pea_wait) begin
                    // Wait only within the current fetch page; faults and
                    // cross-page extensions retain the demand sequencer.
                end else if (pipe_claim) begin
                    if (pipe_input && pipe_ready) begin
                        // S_DECODE already consumed the opcode; consume only
                        // a resident extension alongside pipeline admission.
                        if (pipe_words == 2) begin
                            epf_pop = 2'd1;
                            pc <= pc + 32'd2;
                        end
                        state <= S_EXPERIMENT_PIPE;
                    end
                end else begin
`endif
				// Step D: the body keeps only the paths the record marks in
				// place (generated by step_d.py); the record over ir (rd_ir
				// selects ir here) is applied after it, so its values win
				// over the shared statements kept for the in-place paths.
				begin
					rr_a <= {1'b1, d_rn};
					case (ir_hi)
						4'h0:
						begin
							if (ir[8] && d_mode == 3'b001)
							begin
								mp_dir <= ir[7];
								mp_cnt <= ir[6] ? 3'd4 : 3'd2;
								immf(2'd1, S_MOVEP1);
							end
							else
								if (ir[8])
								begin
									if (d_mode == 3'b001)
										go_illegal;
									else
										if (d_mode == 3'b111 && ((ir[7:6] == 2'b00) ? (d_rn > 3'b100) : (d_rn > 3'b001)))
											go_illegal;
										else
										begin
											alu_op <= `AP040_ALU_BTST + {4'd0, ir[7:6]};
											p_src <= SK_REG;
											p_sreg <= {1'b0, d_reg9};
											rr_b <= {1'b0, d_reg9};
											if (ir[7:6] == 2'b00)
												p_wbsup <= 1;
											if (ea_is_imm)
												immf(2'd1, S_BTSTI);
										end
								end
								else
									if (d_reg9 == 3'b100)
									begin
										if (d_mode == 3'b001)
											go_illegal;
										else
											if (d_mode == 3'b111 && ((ir[7:6] == 2'b00) ? (d_rn > 3'b100) : (d_rn > 3'b001)))
												go_illegal;
									end
									else
										if (d_reg9 == 3'b111 && std_size != 2'b11)
										begin
											if (d_mode < 3'b010 || (d_mode == 3'b111 && d_rn > 3'b001))
												go_illegal;
											else
												if (!sr_s)
													go_priv;
												else
												begin
													op_size <= std_size;
													immf(2'd1, S_MOVES1);
												end
										end
										else
											if (std_size == 2'b11)
											begin
												if (!d_reg9[2] && d_reg9[1:0] != 2'b11)
												begin
													if (d_mode < 3'b010 || d_mode == 3'b011 || d_mode == 3'b100 || ea_is_imm)
														go_illegal;
													else
													begin
														op_size <= d_reg9[1] ? `AP040_SZ_L :
									           d_reg9[0] ? `AP040_SZ_W : `AP040_SZ_B;
														immf(2'd1, S_CHK2_A);
													end
												end
												else
													if (d_reg9[2] && d_reg9[1:0] != 2'b00 && ea_is_imm)
													begin
														if (d_reg9[1:0] == 2'b01)
															go_illegal;
														else
														begin
															alu_op <= `AP040_ALU_CMP;
															op_size <= (d_reg9[1:0] == 2'b10) ? `AP040_SZ_W : `AP040_SZ_L;
															begin
																lk_cyc <= 1;
																immf(2'd2, S_CAS2_0);
															end
														end
													end
													else
														if (d_reg9[2] && d_reg9[1:0] != 2'b00)
														begin
															if (d_mode < 3'b010 || (d_mode == 3'b111 && d_rn > 3'b001))
																go_illegal;
															else
															begin
																alu_op <= `AP040_ALU_CMP;
																op_size <= (d_reg9[1:0] == 2'b01) ? `AP040_SZ_B :
									           (d_reg9[1:0] == 2'b10) ? `AP040_SZ_W : `AP040_SZ_L;
																begin
																	lk_cyc <= 1;
																	immf(2'd1, S_CAS1);
																end
															end
														end
														else
															go_illegal;
											end
											else
											begin
												if (ea_is_imm && (d_reg9 == 3'b000 || d_reg9 == 3'b001 || d_reg9 == 3'b101))
												begin
													if (std_size == 2'b01 && !sr_s)
														go_priv;
													else
														if (std_size > 2'b01)
															go_illegal;
														else
														begin
															srop_kind <= (d_reg9 == 3'b000) ? 2'd0 :
									             (d_reg9 == 3'b001) ? 2'd1 : 2'd2;
															srop_sr <= (std_size == 2'b01);
															immf(2'd1, S_SROP);
														end
												end
												else
													if (d_mode == 3'b001)
														go_illegal;
													else
														if (d_mode == 3'b111 && d_rn > 3'b001 && !(d_reg9 == 3'b110 && d_rn < 3'b100))
															go_illegal;
											end
						end
						4'h1, 4'h2, 4'h3:
						begin
							if (rd_valid)
							begin end
							else
								if (move_size == `AP040_SZ_B && (d_mode == 3'b001 || d_op8_6 == 3'b001))
									go_illegal;
								else
									if (d_op8_6 == 3'b111 && d_reg9 > 3'b001)
										go_illegal;
						end
						4'h4:
						begin
							if (ir[11:0] == 12'hAFC)
								go_illegal;
							else
								if (d_op8_6 == 3'b111)
								begin
									if (d_mode == 3'b000)
									begin
										if (d_reg9 == 3'b100)
										begin end
										else
											go_illegal;
									end
									else
										if (d_mode == 3'b001 || (d_mode == 3'b011) || (d_mode == 3'b100) || ea_is_imm)
											go_illegal;
										else
										begin
                                        ea_start(d_mode, d_rn, `AP040_SZ_L, S_LEA1);
`ifdef AP040_EXPERIMENTAL_LEA
                                        // Select the base while fetching d16;
                                        // the return edge sees a settled RF port.
                                        if (d_mode == 3'b101)
                                            immf(2'd1, S_EA_D16);
`endif
                                    end
								end
								else
									if (d_op8_6 == 3'b110)
									begin
										exec_kind <= EK_CHK;
										op_size <= `AP040_SZ_W;
										p_ssize <= `AP040_SZ_W;
										if (d_mode == 3'b001)
											go_illegal;
									end
									else
										if (d_op8_6 == 3'b100 && !(ir[11:9] == 3'b100 && d_mode == 3'b001))
										begin
											exec_kind <= EK_CHK;
											op_size <= `AP040_SZ_L;
											p_ssize <= `AP040_SZ_L;
											if (d_mode == 3'b001)
												go_illegal;
										end
										else
											case (ir[11:9])
												3'b000:
												begin
													if (d_op8_6 == 3'b011)
													begin
														if (dst_not_alt)
															go_illegal;
														else
															if (!sr_s)
																go_priv;
															else
															begin
																p_src <= SK_IMPL;
																src_val <= {16'd0, sr};
																alu_op <= `AP040_ALU_MOVE;
																op_size <= `AP040_SZ_W;
																p_dsize <= `AP040_SZ_W;
																p_flags <= 0;
																if (d_mode == 3'b000)
																begin
																	p_dst <= DK_REG;
																	p_dreg <= {1'b0, d_rn};
																	pipe_go_regdst({1'b0, d_rn});
																end
																else
																begin
																	p_dst <= DK_MEM;
																	dst_mode_r <= d_mode;
																	dst_rn_r <= d_rn;
																	pipe_go;
																end
															end
													end
													else
														if (d_op8_6[2])
															go_illegal;
														else
														begin
															alu_op <= `AP040_ALU_NEGX;
															op_size <= std_size;
															p_dsize <= std_size;
															p_rmw <= 1;
															if (d_mode == 3'b000)
															begin end
															else
																if (dst_not_alt)
																	go_illegal;
														end
												end
												3'b001:
												begin
													if (d_op8_6 == 3'b011)
													begin
														p_src <= SK_IMPL;
														src_val <= {27'd0, sr[4:0]};
														alu_op <= `AP040_ALU_MOVE;
														op_size <= `AP040_SZ_W;
														p_dsize <= `AP040_SZ_W;
														p_flags <= 0;
														if (d_mode == 3'b000)
														begin
															p_dst <= DK_REG;
															p_dreg <= {1'b0, d_rn};
															pipe_go_regdst({1'b0, d_rn});
														end
														else
															if (dst_not_alt)
																go_illegal;
															else
															begin
																p_dst <= DK_MEM;
																dst_mode_r <= d_mode;
																dst_rn_r <= d_rn;
																pipe_go;
															end
													end
													else
														if (d_op8_6[2])
															go_illegal;
														else
														begin
															alu_op <= `AP040_ALU_CLR;
															op_size <= std_size;
															p_dsize <= std_size;
															if (d_mode == 3'b000)
															begin end
															else
																if (dst_not_alt)
																	go_illegal;
														end
												end
												3'b010:
												begin
													if (d_op8_6 == 3'b011)
													begin
														alu_op <= `AP040_ALU_MOVE;
														op_size <= `AP040_SZ_W;
														p_ssize <= `AP040_SZ_W;
														p_flags <= 0;
														p_dst <= DK_CCR;
														if (d_mode == 3'b000)
														begin end
														else
															if (src_not_data)
																go_illegal;
													end
													else
														if (d_op8_6[2])
															go_illegal;
														else
														begin
															alu_op <= `AP040_ALU_NEG;
															op_size <= std_size;
															p_dsize <= std_size;
															p_rmw <= 1;
															if (d_mode == 3'b000)
															begin end
															else
																if (dst_not_alt)
																	go_illegal;
														end
												end
												3'b011:
												begin
													if (d_op8_6 == 3'b011)
													begin
														if (src_not_data)
															go_illegal;
														else
															if (!sr_s)
																go_priv;
													end
													else
														if (d_op8_6[2])
															go_illegal;
														else
														begin
															alu_op <= `AP040_ALU_NOT;
															op_size <= std_size;
															p_dsize <= std_size;
															p_rmw <= 1;
															if (d_mode == 3'b000)
															begin end
															else
																if (dst_not_alt)
																	go_illegal;
														end
												end
												3'b100:
												begin
													if (d_op8_6[2])
														go_illegal;
													else
														case (d_op8_6[1:0])
															2'b00:
															begin
																if (d_mode == 3'b001)
																begin
																	br_long <= 1;
																	rr_a <= {1'b1, d_rn};
																	immf(2'd2, S_LINK2);
																end
																else
																begin
																	alu_op <= `AP040_ALU_NBCD;
																	op_size <= `AP040_SZ_B;
																	p_dsize <= `AP040_SZ_B;
																	p_rmw <= 1;
																	if (d_mode == 3'b000)
																	begin end
																	else
																		if (dst_not_alt)
																			go_illegal;
																end
															end
															2'b01:
															begin
																if (d_mode == 3'b000)
																begin end
																else
																	if (d_mode == 3'b001)
																		go_illegal;
																	else
																		if (d_mode == 3'b011 || d_mode == 3'b100 || ea_is_imm)
																			go_illegal;
																		else
																		begin
																			ea_start(d_mode, d_rn, `AP040_SZ_L, S_PEA1);
`ifdef AP040_EXPERIMENTAL_LEA
																			// P180: as LEA -- request d16 from decode
																			// (popped inline), and push from S_EA_D16.
																			if (d_mode == 3'b101)
																				immf(2'd1, S_EA_D16);
`endif
																		end
															end
															default:
															begin
																if (d_mode == 3'b000)
																begin end
																else
																begin
																	mm_dir <= 0;
																	mm_size <= d_op8_6[0] ? `AP040_SZ_L : `AP040_SZ_W;
																	mm_predec <= (d_mode == 3'b100);
																	mm_postinc <= 0;
																	if (d_mode == 3'b011 || d_mode < 3'b010 || ea_is_imm || (d_mode == 3'b111 && d_rn > 3'b001))
																		go_illegal;
																	else
																		immf(2'd1, S_MOVEM_SET);
																end
															end
														endcase
												end
												3'b101:
												begin
													if (d_op8_6[2])
														go_illegal;
													else
														if (d_op8_6 == 3'b011)
														begin
															alu_op <= `AP040_ALU_TAS;
															op_size <= `AP040_SZ_B;
															p_dsize <= `AP040_SZ_B;
															p_rmw <= 1;
															if (d_mode == 3'b000)
															begin end
															else
																if (dst_not_alt)
																	go_illegal;
														end
														else
														begin
															alu_op <= `AP040_ALU_TST;
															op_size <= std_size;
															p_ssize <= std_size;
															p_wbsup <= 1;
															if (d_mode == 3'b000 || d_mode == 3'b001)
															begin
																if (d_mode == 3'b001 && std_size == `AP040_SZ_B)
																	go_illegal;
															end
														end
												end
												3'b110:
												begin
													if (d_op8_6[2])
														go_illegal;
													else
														if (!d_op8_6[1])
														begin
															exec_kind <= EK_MD_L;
															md_isdiv <= d_op8_6[0];
															op_size <= `AP040_SZ_L;
															p_ssize <= `AP040_SZ_L;
															if (d_mode == 3'b001)
																go_illegal;
															else
															begin
																if (d_mode == 3'b000)
																begin
																	p_src <= SK_REG;
																	p_sreg <= {1'b0, d_rn};
																end
																else
																	if (ea_is_imm)
																		p_src <= SK_IMM;
																	else
																	begin
																		p_src <= SK_MEM;
																		src_mode_r <= d_mode;
																		src_rn_r <= d_rn;
																	end
																immf(2'd1, S_MDL_EXT);
															end
														end
														else
														begin
															mm_dir <= 1;
															mm_size <= d_op8_6[0] ? `AP040_SZ_L : `AP040_SZ_W;
															mm_predec <= 0;
															mm_postinc <= (d_mode == 3'b011);
															if (d_mode == 3'b100 || d_mode < 3'b010 || ea_is_imm)
																go_illegal;
															else
																immf(2'd1, S_MOVEM_SET);
														end
												end
												default:
												begin
													if (d_op8_6 == 3'b010)
													begin
														if (d_mode < 3'b010 || d_mode == 3'b011 || d_mode == 3'b100 || ea_is_imm)
															go_illegal;
														else
															ea_start(d_mode, d_rn, `AP040_SZ_L, S_JSR1);
													end
													else
														if (d_op8_6 == 3'b011)
														begin
															if (d_mode < 3'b010 || d_mode == 3'b011 || d_mode == 3'b100 || ea_is_imm)
																go_illegal;
															else
																ea_start(d_mode, d_rn, `AP040_SZ_L, S_JMP1);
														end
														else
															if (d_op8_6 == 3'b001)
															begin
																casez (ir[5:0])
																	6'b00????: exc(`AP040_VEC_TRAP + {4'd0, ir[3:0]}, 4'd0, pc, 32'd0);
																	6'b010???:
																	begin
																		br_long <= 0;
																		rr_a <= {1'b1, d_rn};
																		immf(2'd1, S_LINK2);
																	end
																	6'b011???:
																	begin
																		rr_a <= {1'b1, d_rn};
																		state <= S_UNLK1;
																	end
																	6'b100???:
																	begin
																		if (!sr_s)
																			go_priv;
																		else
																		begin
																			rr_a <= {1'b1, d_rn};
																			state <= S_USP1;
																		end
																	end
																	6'b101???:
																	begin
																		if (!sr_s)
																			go_priv;
																		else
																		begin
																			rfw({1'b1, d_rn}, usp_wb);
																			fetch_next;
																		end
																	end
																	6'b110000:
																	begin
																		if (!sr_s)
																			go_priv;
																		else
																		begin
																			rst_cnt <= 8'd127;
																			state <= S_RESET_HOLD;
																		end
																	end
																	6'b110001: fetch_next;
																	6'b110010:
																	begin
																		if (!sr_s)
																			go_priv;
																		else
																			immf(2'd1, S_STOP_LD);
																	end
																	6'b110011:
																	begin
																		if (!sr_s)
																			go_priv;
																		else
																		begin
																			in_exc <= 1;
																			state <= S_RTE_SR;
																		end
																	end
																	6'b110100:
																	begin
																		ret_kind <= RK_RTD;
																		immf(2'd1, S_RET1);
																	end
																	6'b110101:
																	begin
																		ret_kind <= RK_RTS;
																		mrd(dbg_a7_wb, `AP040_SZ_L, S_RET2);
																	end
																	6'b110110:
																	begin
																		if (sr[1])
																			exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);
																		else
																			fetch_next;
																	end
																	6'b110111:
																	begin
																		ret_kind <= RK_RTR;
																		state <= S_RET1;
																	end
																	6'b111010, 6'b111011:
																	begin
																		if (!sr_s)
																			go_priv;
																		else
																		begin
																			mvc_dir <= ir[0];
																			immf(2'd1, S_MOVEC1);
																		end
																	end
																	default: go_illegal;
																endcase
															end
															else
																go_illegal;
												end
											endcase
						end
						4'h5:
						begin
							if (rd_valid)
							begin end
							else
								if (ir[7:6] == 2'b11)
								begin
									if (d_mode == 3'b001)
									begin
										br_base <= pc;
										rr_a <= {1'b0, d_rn};
										immf(2'd1, S_DBCC1);
									end
									else
										if (d_mode == 3'b111 && d_rn >= 3'b010 && d_rn <= 3'b100)
										begin
											if (d_rn == 3'b010)
												immf(2'd1, cond_true(ir[11:8]) ? S_TRAPCC : S_NEXT);
											else
												if (d_rn == 3'b011)
													immf(2'd2, cond_true(ir[11:8]) ? S_TRAPCC : S_NEXT);
												else
												begin
													if (cond_true(ir[11:8]))
														exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);
													else
														fetch_next;
												end
										end
										else
										begin
											exec_kind <= EK_SCC;
											op_size <= `AP040_SZ_B;
											p_dsize <= `AP040_SZ_B;
											p_flags <= 0;
											if (d_mode == 3'b000)
											begin end
											else
												if (dst_not_alt)
													go_illegal;
										end
								end
								else
								begin
									alu_op <= ir[8] ? `AP040_ALU_SUB : `AP040_ALU_ADD;
									p_src <= SK_IMPL;
									src_val <= {28'd0, (d_reg9 == 3'd0) ? 4'd8 : {1'b0, d_reg9}};
									if (dst_not_alt)
										go_illegal;
								end
						end
						4'h6:
						begin
							if (ir[7:0] == 8'h00 || ir[7:0] == 8'hFF)
							begin
								br_base <= pc;
								br_long <= (ir[7:0] == 8'hFF);
								immf((ir[7:0] == 8'hFF) ? 2'd2 : 2'd1, S_BCC_EXT);
							end
							else
								if (ir[11:8] == 4'h1)
								begin : bsr_b
									reg [31:0] bt;
									bt = pc + sxb(ir[7:0]);
									if (bt[0])
										go_pc(bt);
									else
									begin
										br_tgt <= bt;
										mwr(dbg_a7_wb - 32'd4, `AP040_SZ_L, pc, S_BSR_PUSH);
									end
								end
								else
									finish_bcc(pc + sxb(ir[7:0]), cond_true(ir[11:8]));
						end
						4'h7:
						begin
							if (ir[8])
								go_illegal;
							else
							begin
								rfw({1'b0, d_reg9}, sxb(ir[7:0]));
								sr[3] <= ir[7];
								sr[2] <= (ir[7:0] == 8'd0);
								sr[1] <= 0;
								sr[0] <= 0;
								fetch_next;
							end
						end
						4'h8:
						begin
							if (rd_valid)
							begin end
							else
								if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111)
								begin
									exec_kind <= EK_MD_W;
									md_isdiv <= 1;
									md_sign <= d_op8_6[2];
									op_size <= `AP040_SZ_W;
									p_ssize <= `AP040_SZ_W;
									p_dst <= DK_REG;
									p_dreg <= {1'b0, d_reg9};
									if (d_mode == 3'b001)
										go_illegal;
								end
								else
									if (ir[8] && d_mode[2:1] == 2'b00)
									begin end   // SBCD/PACK/UNPK: the decode record's (the reducer had left a bogus go_illegal here)
									else
									begin
										alu_op <= `AP040_ALU_OR;
										op_size <= std_size;
										p_ssize <= std_size;
										p_dsize <= std_size;
										if (!ir[8])
										begin
											p_dst <= DK_REG;
											p_dreg <= {1'b0, d_reg9};
											if (d_mode == 3'b001)
												go_illegal;
										end
										else
										begin
											p_src <= SK_REG;
											p_sreg <= {1'b0, d_reg9};
											rr_b <= {1'b0, d_reg9};
											p_dst <= DK_MEM;
											p_rmw <= 1;
											dst_mode_r <= d_mode;
											dst_rn_r <= d_rn;
											if (d_mode < 3'b010 || (d_mode == 3'b111 && d_rn > 3'b001))
												go_illegal;
										end
									end
						end
						4'h9, 4'hD:
						begin : dec_addsub
							reg is_add;
							is_add = (ir_hi == 4'hD);
							if (rd_valid)
							begin end
							else
								if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111)
								begin end
								else
									if (ir[8] && d_mode[2:1] == 2'b00 && std_size != 2'b11)
									begin end
									else
									begin
										alu_op <= is_add ? `AP040_ALU_ADD : `AP040_ALU_SUB;
										op_size <= std_size;
										p_ssize <= std_size;
										p_dsize <= std_size;
										if (!ir[8])
										begin
											p_dst <= DK_REG;
											p_dreg <= {1'b0, d_reg9};
											if (d_mode == 3'b001 && std_size == `AP040_SZ_B)
												go_illegal;
										end
										else
										begin
											p_src <= SK_REG;
											p_sreg <= {1'b0, d_reg9};
											rr_b <= {1'b0, d_reg9};
											p_dst <= DK_MEM;
											p_rmw <= 1;
											dst_mode_r <= d_mode;
											dst_rn_r <= d_rn;
											if (d_mode < 3'b010 || (d_mode == 3'b111 && d_rn > 3'b001))
												go_illegal;
										end
									end
						end
						4'hA: exc(`AP040_VEC_ALINE, 4'd0, pc_i, 32'd0);
						4'hB:
						begin
							if (rd_valid)
							begin end
							else
								if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111)
								begin end
								else
									if (!ir[8])
									begin
										alu_op <= `AP040_ALU_CMP;
										op_size <= std_size;
										p_ssize <= std_size;
										p_wbsup <= 1;
										p_dst <= DK_REG;
										p_dreg <= {1'b0, d_reg9};
										if (d_mode == 3'b001 && std_size == `AP040_SZ_B)
											go_illegal;
									end
									else
										if (d_mode == 3'b001)
										begin end
										else
										begin
											alu_op <= `AP040_ALU_EOR;
											op_size <= std_size;
											p_dsize <= std_size;
											p_src <= SK_REG;
											p_sreg <= {1'b0, d_reg9};
											rr_b <= {1'b0, d_reg9};
											if (dst_not_alt)
												go_illegal;
										end
						end
						4'hC:
						begin
							if (rd_valid)
							begin end
							else
								if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111)
								begin
									exec_kind <= EK_MD_W;
									md_isdiv <= 0;
									md_sign <= d_op8_6[2];
									op_size <= `AP040_SZ_W;
									p_ssize <= `AP040_SZ_W;
									p_dst <= DK_REG;
									p_dreg <= {1'b0, d_reg9};
									if (d_mode == 3'b001)
										go_illegal;
								end
								else
									if (ir[8] && d_op8_6[1:0] == 2'b00 && d_mode[2:1] == 2'b00)
									begin end
									else
										if (ir[8] && (d_op8_6[1:0] == 2'b01) && d_mode[2:1] == 2'b00)
										begin
											rr_a <= {d_mode[0], d_reg9};
											rr_b <= {d_mode[0], d_rn};
											state <= S_EXG1;
										end
										else
											if (ir[8] && d_op8_6[1:0] == 2'b10 && d_mode == 3'b001)
											begin
												rr_a <= {1'b0, d_reg9};
												rr_b <= {1'b1, d_rn};
												state <= S_EXG1;
											end
											else
											begin
												alu_op <= `AP040_ALU_AND;
												op_size <= std_size;
												p_ssize <= std_size;
												p_dsize <= std_size;
												if (!ir[8])
												begin
													p_dst <= DK_REG;
													p_dreg <= {1'b0, d_reg9};
													if (d_mode == 3'b001)
														go_illegal;
												end
												else
												begin
													p_src <= SK_REG;
													p_sreg <= {1'b0, d_reg9};
													rr_b <= {1'b0, d_reg9};
													p_dst <= DK_MEM;
													p_rmw <= 1;
													dst_mode_r <= d_mode;
													dst_rn_r <= d_rn;
													if (d_mode < 3'b010 || (d_mode == 3'b111 && d_rn > 3'b001))
														go_illegal;
												end
											end
						end
						4'hE:
						begin
							if (ir[7:6] == 2'b11)
							begin
								if (ir[11])
								begin
									if (d_mode == 3'b001 || d_mode == 3'b011 || d_mode == 3'b100 || ea_is_imm)
										go_illegal;
									else
										if ((d_mode == 3'b111 && d_rn > 3'b001) && (ir[10:8] == 3'd2 || ir[10:8] == 3'd4 || ir[10:8] == 3'd6 || ir[10:8] == 3'd7))
											go_illegal;
										else
											immf(2'd1, S_BF0);
								end
								else
								begin
									exec_kind <= EK_SHIFT;
									sh_rox <= (ir[10:9] == 2'b10);
									case (ir[10:9])
										2'b00: alu_op <= ir[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1;
										2'b01: alu_op <= ir[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1;
										2'b10: alu_op <= ir[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1;
										default: alu_op <= ir[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1;
									endcase
									op_size <= `AP040_SZ_W;
									p_dsize <= `AP040_SZ_W;
									p_src <= SK_NONE;
									p_rmw <= 1;
									if (d_mode < 3'b010 || (d_mode == 3'b111 && d_rn > 3'b001))
										go_illegal;
								end
							end
						end
						default:
						begin
							if (ir[11:8] == 4'h4)
							begin
								if (ir[4:3] == 2'b00)
									go_fp_fline;
								else
									if (!sr_s)
										go_priv;
									else
									begin
										cinv_ic <= ir[7];
										cinv_dc <= ir[6];
										epf_flush;
										cinv_req <= 1;
										state <= S_CINV2;
									end
							end
							else
								if (ir[11:8] == 4'h5)
								begin
									if (ir[7:5] == 3'b000)
									begin
										if (!sr_s)
											go_priv;
										else
										begin
											epf_flush;
											pf_mode <= ir[4:3];
											if (ir[4])
											begin
												state <= S_PFLUSH2;
											end
											else
											begin
												rr_a <= {1'b1, d_rn};
												state <= S_PFLUSH1;
											end
										end
									end
									else
										if (ir[7:6] == 2'b01)
										begin
											if (ir[4:3] != 2'b01)
												go_fp_fline;
											else
												if (!sr_s)
													go_priv;
												else
												begin
													rr_a <= {1'b1, d_rn};
													state <= S_PTEST1;
												end
										end
										else
											exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
								end
								else
									if (ir[11:8] == 4'h2)
									begin
										if (AP040_HAS_FPU == 0)
											exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
										else
											case (ir[7:6])
												2'b00:
												begin
													if (d_mode == 3'b111 && d_rn > 3'b100)
														exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
													else
														immf(2'd1, S_FPU_DEC);
												end
												2'b01:
												begin
													if (d_mode == 3'b111 && d_rn > 3'b100)
														exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
													else
														immf(2'd1, S_FSCC0);
												end
												2'b10: immf(2'd1, S_FBCC);
												default: immf(2'd2, S_FBCC);
											endcase
									end
									else
										if (ir[11:8] == 4'h3)
										begin
											if (ir[7:6] == 2'b00)
											begin
												if (d_mode < 3'b010 || d_mode == 3'b011 || (d_mode == 3'b111 && d_rn > 3'b001))
													go_fp_fline;
												else
													if (!sr_s)
														go_priv;
													else
														ea_start(d_mode, d_rn, `AP040_SZ_L, S_FSAVE1);
											end
											else
												if (ir[7:6] == 2'b01)
												begin
													if (d_mode < 3'b010 || d_mode == 3'b100 || (d_mode == 3'b111 && d_rn >= 3'b100))
														go_fp_fline;
													else
														if (!sr_s)
															go_priv;
														else
															ea_start(d_mode, d_rn, `AP040_SZ_L, S_FREST1);
												end
												else
													exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
										end
										else
											if (ir[11:8] == 4'h6 && ir[7:5] == 3'b000)
											begin
												m16_form <= {1'b0, ir[4:3]};
												immf(2'd2, S_M16_SRC);
											end
											else
												if (ir[11:8] == 4'h6 && ir[7:3] == 5'b00100)
												begin
													m16_form <= 3'd4;
													immf(2'd1, S_M16_SRC);
												end
												else
													exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
						end
					endcase
				end
				if (!n_inplace) apply_record_decode;
`ifdef AP040_EXPERIMENTAL_PIPELINE
                end
`endif
			end
			S_STOP_LD: begin
				epf_flush;
				sr <= imm[15:0] & `AP040_SR_MASK;
				// T1 traces STOP unconditionally.  T0 traces it only when
				// the written SR changes T1/T0/S/M or the interrupt mask:
				// WinUAE's MakeFromSR returns before its trace decision
				// when none of those bits change ("STOP SR-modification
				// does not generate T0"), and STOP has no check_t0_trace
				// like the MOVE/ORI/ANDI/EORI-to-SR family, so an
				// upper-identical STOP under T0 does not trace on the 040.
				if (tr_t1 || (tr_t0 && {imm[15:12], imm[10:8]} !=
				                       {sr[15:12], sr[10:8]})) begin
					tr_t1 <= 0;
					tr_t0 <= 0;
					exc(`AP040_VEC_TRACE, 4'd2, pc, pc_i);
				end
				else state <= S_STOPPED;
			end

			S_STOPPED: begin
				if (irq_pend) begin
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
					exc_spc <= pc; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= irq_take_lvl;
					epf_flush;
					state <= S_EXC0;
				end
			end

			//------------------------------------------------------- TRAPcc
			S_TRAPCC: exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);

			//--------------------------------------------------------- halted
			S_HALT: begin
				mem_req <= 0;
				ifr_req <= 0;
				m_issued <= 0;
				pt_req <= 0;
				pf_req <= 0;
				cinv_req <= 0;
				fpu_req <= 0;
				epf_flush;
				epf_pend <= 0;
				epf_kill <= 0;
			end

			default: fatal_halt;
		endcase

`ifdef AP040_EXPERIMENTAL_PIPELINE
        if (pipe_rf_owner && !pipe_owner && pipe_input && pipe_ready) begin
            epf_pop = pipe_words;
            pc <= pc + {29'd0, pipe_words, 1'b0};
            perf_dispatch_toggle <= ~perf_dispatch_toggle;
        end
`endif
		// The retire boundary the arm above asked for (see fetch_next).
		if (mgo) mem_issue;
		if (igo) immf_now(igo_n, igo_ret);
		if (retire_req) begin
`ifdef AP040_EXPERIMENTAL_PIPELINE
            fetch_next_body((pipe_owner || pipe_read_retire) ? pipe_next_pc : pc,
                            (pipe_owner || pipe_read_retire) ? pipe_pc : pc_i);
`else
            fetch_next_body(pc, pc_i);
`endif
        end
`ifdef AP040_EXPERIMENTAL_PIPELINE
        // The sequencer may keep its existing lookahead while it owns the
        // core. Pipeline entry still uses the S_DECODE admission policy;
        // its drained exit uses the normal fetch boundary.
        // Early drain permits lookahead only when final WB leaves no younger
        // work. Keeping an idle-only guard here loses the call/fetch shortcut.
        if (pipe_owner && !pipe_exit_ready) rd_queue_pop = 0;
`ifdef AP040_PIPELINE_FORCE_DECODE
        // Diagnostic mode exercises every supported opcode in the pipeline.
        rd_queue_pop = 0;
`endif
`endif
`ifdef AP040_EXPERIMENTAL_PIPELINE
        // Route indexed MOVE to admission even when its extension is late.
        // The same-page wait above admits brief format once resident; full
        // format falls back to the sequencer after the extension arrives.
        if (PIPE_MEMORY_ENTRY && PIPE_P6 && rd_queue_pop && epf_count >= 1 &&
            (epf_count < 2 || !epf_data[epf_head + 3'd1][8]) &&
            epf_data[epf_head][15:14] == 0 && epf_data[epf_head][13:12] != 0 &&
            ((epf_data[epf_head][5:3] == 6 &&
              (epf_data[epf_head][8:6] == 0 || epf_data[epf_head][8:6] == 1) &&
              !(epf_data[epf_head][13:12] == 1 && epf_data[epf_head][8:6] == 1)) ||
             (epf_data[epf_head][8:6] == 6 && epf_data[epf_head][5:4] == 0 &&
              !(epf_data[epf_head][13:12] == 1 && epf_data[epf_head][3]))))
            rd_queue_pop = 0;
`endif
`ifdef AP040_EXPERIMENTAL_PIPELINE
        // Let supported memory compares reach the existing admission policy
        // instead of having resident lookahead immediately choose legacy EA.
        if (PIPE_MEMORY_ENTRY && PIPE_COMPARE && rd_queue_pop && epf_count >= 1 &&
            epf_data[epf_head][7:6] != 3 &&
            (((epf_data[epf_head] & 16'hff38) == 16'h4a10) ||
             ((epf_count < 2 || !epf_data[epf_head + 3'd1][8]) &&
              (((epf_data[epf_head] & 16'hff38) == 16'h4a30) ||
               ((epf_data[epf_head] & 16'hf138) == 16'hb030)))))
            rd_queue_pop = 0;
`endif
		// go_pc_now and exc_now run after the lookahead arm (below): the
		// arm's forward taken Bcc raises pgo, and go_pc's own address-error
		// and trace cases raise xgo, which exc_now must see.  Nothing the
		// arm writes is written by either when they fire in the same cycle
		// (a retire that pops never carries an exception).  (2026-09-17)

		// Single shared-descriptor control writer. In normal decode only
		// covered opcode families suppress their legacy body; unrelated
		// system/FPU/control decoding carries no global rd_valid qualifier.
		// For lookahead, rd_queue_pop proves fetch_next passed IRQ/trace,
		// resident-PC/context and same-cycle flush checks. The bounded ALU
		// producer set cannot change the active A7 bank or emit aux writes.
		// S_DECODE itself never uses lookahead: rd_ir still selects ir there.
		// A completed store with nothing left to do (S_MWR, r_m_ret ==
		// S_NEXT) is a producer too: it retires through the same
		// fetch_next, writes no register at that point, and its
		// self-modifying-code flush already blocks the pop.
		// Steps C/C2/C3: every retire that pops the next opcode (rd_queue_pop
		// is set only by fetch_next's pop branch) hands it over from the
		// descriptor or the decode record when they cover it, from this one
		// site: the same call inside fetch_next was inlined at 83 sites and
		// cost 8,400 ALMs of duplicated record muxes.
		if (rd_valid && !pipe_claim && ((state == S_DECODE) || (rd_queue_pop && n_desc_ok)))
			dispatch_reg_decode;
		else if (rd_queue_pop && n_apply_ok)
			apply_record;
		// The branch lookahead: fetch_next has just dispatched the Bcc at
		// the head into S_DECODE; resolve it here instead.  Taken: the
		// same redirect finish_bcc would take a cycle later (refill
		// dispatch or go_pc), with the trace and interrupt cases left to
		// the ordinary path.  Not taken: the word behind the branch is
		// dispatched in its place.  A queue fetch acknowledging in this
		// cycle is not excluded: the refill seed is written after the
		// acknowledge's append (brf_seed_data), so it wins, and the
		// acknowledge would otherwise sit in this decision's path (the
		// address hint's translation to the acknowledge, -1.26 ns).
		else if (rd_is_bcc && rd_queue_pop && !aux_we && (state != S_DECODE) &&
		         (rd_bcc_fl_ok || rd_is_bra) && !rd_bcc_t[0] &&
		         !sr[15] && !sr[14] && !irq_pend &&
		         (regs_alu_fire || pipe_drain ||
		          ((state == S_MWR) && d_ack && (r_m_ret == S_NEXT)) ||
		          // BRA.B needs no flags: from any retire whose state
		          // has no target arm of its own on go_pc_t_early
		          // (2026-09-19)
		          (rd_is_bra && rd_bra_state))) begin
			// Taken: only the refill-buffer dispatch (the loop case); a
			// target outside the buffered sector keeps the ordinary
			// S_DECODE path (a go_pc expansion here cost 700 ALMs and
			// routing).  Not taken: the word behind the branch dispatches.
			if (rd_bcc_taken) begin
				if (brf_refill_hit(rd_bcc_t) &&
				    (!epf_armed || epf_next != rd_bcc_t || epf_super != sr_s))
					decode_dbcc_brf(rd_bcc_t);
				// A target outside the sector: the same go_pc that
				// finish_bcc would call from S_DECODE a cycle later, raised
				// here through its carrier (go_pc_now runs after this arm
				// and takes rd_bcc_t as its state-selected target), so the
				// demand fetch goes out at the end of the producer's retire
				// cycle instead of the decode cycle.  The arm's own guards
				// (no trace, no interrupt, even target) are the ones that
				// select go_pc's plain redirect path.  Only when the port is
				// free now: a redirect raised in an acknowledge cycle or under
				// a queue fetch is deferred to the fill engine, whose fill does
				// not adopt the target's sector, and every later branch into
				// it then pays a demand fetch (the corpus ran 1.3 % slower);
				// S_DECODE, a cycle later, usually finds the port free.
				// (2026-09-17)
				else if (ifr_avail) go_pc(rd_bcc_t);
			end
			else if (epf_count >= 4'd2) begin
				ir <= epf_data[epf_head + 3'd1];
				if (!movem_mem_op(epf_data[epf_head + 3'd1])) mm_resume <= 0;
				t0_force <= t0_special(epf_data[epf_head + 3'd1]);
				pc_i <= pc + 32'd2;
				pc <= pc + 32'd4;
				epf_pop = 2'd2;
			end
		end
		// An unconditional transfer with a resident target at the head:
		// into its branch state now, with the target fetch out when the
		// port is free (see dispatch_branch).  (2026-09-18)
		else if (rd_queue_pop && bd_ok)
			dispatch_branch;
		// DBcc with its displacement resident: into S_DBCC1 now, unless the
		// retiring arm writes its Dn on this edge (see dispatch_dbcc).
		else if (rd_queue_pop && dd_ok &&
		         !(rfw_now && (rfw_now_a == {1'b0, rd_ir[2:0]})))
			dispatch_dbcc;
		else if (rd_queue_pop && lk_ok &&
		         !(rfw_now && ((rfw_now_a == {1'b1, rd_ir[2:0]}) || (rfw_now_a == 4'd15))))
			dispatch_link;
		else if (rd_queue_pop && ul_ok &&
		         !(rfw_now && ((rfw_now_a == {1'b1, rd_ir[2:0]}) || (rfw_now_a == 4'd15))))
			dispatch_unlk;
		else if (rd_queue_pop && fd_ok)
			dispatch_fpu;
		else if (rd_queue_pop && ret_after_unlk)
			dispatch_ret;
`ifdef AP040_EXPERIMENTAL_LEA
		else if (rd_queue_pop && pd_ok &&
		         !(rfw_now && ((rfw_now_a == {1'b1, rd_ir[2:0]}) || (rfw_now_a == 4'd15))))
			dispatch_pea;
		else if (rd_queue_pop && ld_ok &&
		         !(rfw_now && (rfw_now_a == {1'b1, rd_ir[2:0]})))
			dispatch_lea;
`endif

		// The resident-target dispatch an arm or the lookahead asked for: it
		// arms the queue and claims the port, so it runs before the fill engine.
		if (pgo) go_pc_now(go_pc_t_early);
		if (xgo) exc_now(xgo_vec, xgo_fmt, xgo_spc, xgo_addr);
		if (dgo) decode_dbcc_brf_now(dbrf_a_early);
		// dispatch_branch's early target fetch: the stream is re-armed at
		// the target and the fetch issued (the port was checked free), so
		// this runs before the fill engine like the redirects above.  The
		// target is the shared early-target wire (equal to bd_t whenever
		// sgo is raised; dispatch_branch checks it in simulation).
		if (sgo) issue_ifetch(go_pc_t_early, sr_s);

		//-------------------------------------------------- fetch queue engine
		// The queue fills itself: whenever the memory port is idle, the
		// stream is armed, and there is room for the whole request, the
		// next words of the instruction stream are fetched while the core
		// executes.  This runs after the case statement so that any state
		// which claimed the port this cycle keeps it; epf_issue/epf_flushed
		// carry that decision here combinationally.
		if (epf_pend && i_ack) begin
			// A longword request returns the word at the fetch address in
			// [31:16] and its successor in [15:0]; a word request returns
			// one word in [15:0].
			epf_pend <= 0;
			epf_kill <= 0;
			if (!epf_kill && !epf_flushed) begin
				if (epf_pend_lw) begin
					epf_data[epf_fill]        <= mem_rdata[31:16];
					epf_data[epf_fill + 3'd1] <= mem_rdata[15:0];
					epf_fillw = 4'd2;
				end
				else begin
					epf_data[epf_fill] <= mem_rdata[15:0];
					epf_fillw = 4'd1;
				end
				// A longword instruction fetch is naturally aligned and never
				// crosses a 32-byte sector, so both returned words update one
				// entry.  The buffer keeps the sector of the last redirect: a
				// speculative fill that runs past a loop's end must not evict
				// the loop it will branch back into, and must not touch the
				// buffer's data either, or stale valid bits would describe
				// words from another sector.
				if ((brf_tag == ifr_addr[31:6] && brf_super == epf_super) ||
				    epf_pend_seed) begin
					if (epf_pend_lw)
						brf_data[ifr_addr[5:2]] <= mem_rdata;
					else if (ifr_addr[1])
						brf_data[ifr_addr[5:2]][15:0] <= mem_rdata[15:0];
					else
						brf_data[ifr_addr[5:2]][31:16] <= mem_rdata[15:0];
					if (brf_tag == ifr_addr[31:6] && brf_super == epf_super) begin
						brf_valid[ifr_addr[5:1]] <= 1;
						if (epf_pend_lw) brf_valid[ifr_addr[5:1] + 5'd1] <= 1;
					end
					else begin
						brf_valid <= (epf_pend_lw ? 32'd3
						                          : 32'd1)
						                  << ifr_addr[5:1];
						brf_tag <= ifr_addr[31:6];
						brf_super <= epf_super;
					end
				end
			end
		end
		else if (epf_pend && i_err) begin
			// A fault on a queue fetch.  If the core is waiting for exactly
			// this word the access error is taken now, with the faulting
			// request still in the mem_* registers that build the frame.
			// A fault on a word fetched ahead of demand is only recorded:
			// the fetch is re-issued when execution actually reaches it, and
			// faults again there with the live context.
			ifr_req  <= 0;
			// the frame builder (aerr_start) reads the faulting request from
			// the mem_* registers: carry the fetch's over (P171)
			mem_addr_q <= ifr_addr; mem_size <= ifr_size; fc_r <= ifr_fc;
			mem_write <= 0; mem_instr_q <= 1;
			epf_pend <= 0;
			epf_kill <= 0;
			if (epf_kill || epf_flushed) begin
				// abandoned before the fault: nothing to report
			end
			else if ((state == S_FETCH || state == S_IMMF) &&
			         epf_count == 4'd0 && epf_next == ifr_addr) begin
				if (in_exc) fatal_halt;
				else aerr_start(1);
			end
			else epf_err <= 1;
		end
		// A recorded fault re-arms when execution reaches the faulting word.
		else if (epf_err && epf_armed && !epf_pend && epf_count == 4'd0 &&
		         epf_next == epf_ftail &&
		         (state == S_FETCH || state == S_IMMF))
			epf_err <= 0;
		// Self-fill.  The fetch stays inside the page the core is already
		// executing from: an aligned fetch within that page cannot translate
		// or fault differently than the fetch that got the core here, which
		// is what makes a speculative fetch safe.
		// epf_kill is not tested here: it only qualifies an outstanding
		// fetch, and the issue below clears it.
		// lk_cyc: a TAS/CAS/CAS2 operand sequence must stay indivisible at
		// the core/adapter boundary (plan section 8), so no SPECULATIVE
		// fetch may be interleaved between the locked read and write.  A
		// demand fetch (S_IMMF starving on an empty queue) must still be
		// served: lk_cyc is set at decode, BEFORE the extension words are
		// consumed, and those fetches precede the locked read.  This also
		// keeps a stale lk_cyc after a faulted CAS from starving the
		// handler's first instruction.
		// A queue seeded from the branch-refill buffer used to refuse
		// speculative filling until it ran dry; with the cache returning
		// whole lines, one fetch of the continuation brings the rest of
		// the loop, so the stream fills like any other.
		else if (epf_armed && !epf_pend && !epf_err &&
		         !epf_issue && !epf_flushed &&
		         !ifr_req && !ifr_ack &&
		         (!lk_cyc || state == S_IMMF) &&
		         (epf_super == sr_s) &&
		         (epf_ftail[31:12] == pc[31:12]) &&
		         state != S_EPF_FILL && state != S_EPF_GAP &&
		         // Computing an effective address means a DATA access is
		         // imminent, and the port is shared: a speculative fetch
		         // started here is still in flight when S_MRD wants it, so
		         // the data access queues behind a whole fill.  Skipping
		         // these slots costs the queue a little run-ahead and is
		         // worth 12% on loop code (bench_loop).  Demand fetches are
		         // untouched -- S_FETCH and S_IMMF are not EA states.
                 // Leave memory slots for stack transfers once four words
                 // are queued. Demand fetch and other instruction families
                 // retain their usual admission and refill thresholds.
                 (epf_count <= 4'd4 ||
                  !((ir & 16'hfff0) == 16'h4e50 || ir == 16'h4e75 ||
                    ((ir & 16'hfb80) == 16'h4880 && ir[5:3] >= 2))) &&
		         // the supply policy (epf_fill_ok, defined with the hint mux):
		         // a starvation floor, and above it a fill only into an idle
		         // port slot (P171)
		         epf_fill_ok)
		begin
			epf_brf <= 0;
			epf_pend_seed <= 0;
			ifr_req <= 1;
			ifr_size <= epf_ftail[1] ? `AP040_SZ_W : `AP040_SZ_L;
			ifr_addr <= epf_ftail;
			ifr_fc <= epf_super ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
			epf_pend <= 1;
			epf_pend_lw <= ~epf_ftail[1];
			epf_kill <= 0;
		end

		// The redirect's refill seed, once for every issue_ifetch site: the
		// words from the buffer land last, over any ring write earlier in
		// this cycle (a killed fetch's append, a line offer).
		if (brf_seed_req) begin : brf_seed_data
			reg [15:0] lane_word [0:7];
			reg [1:0] row;
			reg [2:0] lane_sel;
			integer lane, si;
			// One word from each of eight lanes, then rotate the lanes.
			// Address wrap is identical to the original 5-bit sector index.
			for (lane = 0; lane < 8; lane = lane + 1) begin
				row = brf_seed_a[4:3] + (lane[2:0] < brf_seed_a[2:0]);
				lane_word[lane] = lane[0] ? brf_data[{row, lane[2:1]}][15:0]
				                             : brf_data[{row, lane[2:1]}][31:16];
			end
			for (si = 0; si < 8; si = si + 1) begin
				lane_sel = brf_seed_a[2:0] + si[2:0];
				if (si[3:0] < brf_seed_n) epf_data[si] <= lane_word[lane_sel];
			end
		end

		// Queue bookkeeping in one place, so that a pop and an append in the
		// same cycle cannot lose each other's update.  A flush has already
		// written the whole set and wins.
		if (!epf_flushed && (epf_pop != 2'd0 || epf_fillw != 4'd0)) begin
			epf_count <= epf_count + epf_fillw - {2'd0, epf_pop};
			if (epf_pop != 2'd0) begin
				epf_head <= epf_head + {1'b0, epf_pop};
				epf_next <= epf_next + {29'd0, epf_pop, 1'b0};
			end
			if (epf_fillw != 4'd0) begin
				epf_fill  <= epf_fill + epf_fillw[2:0];
				epf_ftail <= epf_ftail + {27'd0, epf_fillw, 1'b0};
			end
		end
	end
end

//---------------------------------------------------------------------------
// debug/status
//---------------------------------------------------------------------------

assign debug_busy   = mem_req;
assign debug_fault  = fault_r;
assign debug_halted = (state == S_HALT);

assign debug_status2 = {
	aer_fa,                      // [127:96] address whose access faulted
	usp_q,                       // [95:64]
	isp_q,                       // [63:32]
	16'd0, exc_vec, 5'd0, in_exc, fault_r, 1'b0   // [31:0]
};

assign debug_status = {
	16'hA040,                    // [255:240] magic
	6'd0, fault_r, unused_in,    // [239:232]
	state,                       // [231:224]
	dbg_a0,                      // [223:192]
	dbg_d2,                      // [191:160]
	dbg_d1,                      // [159:128]
	dbg_d0,                      // [127:96]
	dbg_a7,                      // [95:64]
	ir,                          // [63:48]
	sr,                          // [47:32]
	pc                           // [31:0]
};

endmodule
