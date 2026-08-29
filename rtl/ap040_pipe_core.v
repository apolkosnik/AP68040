//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 17: address error)       //
//                                                                          //
// ap040_pipe_core.v - top level: wires IF/ID/EA-calc/EA-fetch/EX/WB into a //
// real six-stage pipeline with a synchronous stall/flush chain,            //
// instantiates this directory's own register file and ALU forks and a new //
// architectural CCR, drives the commit (regfile write + CCR update)        //
// directly off EX's registered output, and exposes debug taps for the      //
// test.                                                                    //
//                                                                          //
// Deliberately named ap040_pipe_core, not ap040_core: it must never        //
// collide with, or be silently substitutable for, the working sequential   //
// core in rtl/ap040/ap040_core.v (see the note in                          //
// rtl/ap040/experimental/README about ap040_core_fetchbuffer.v reusing     //
// that name). This module is not referenced by ap040.qip, cpu_wrapper.v    //
// or Minimig.sv -- it is built and tested standalone until a later         //
// milestone wires it in deliberately.                                     //
//                                                                          //
// Folder independence (new this milestone): earlier milestones reused      //
// rtl/ap040/ap040_regfile.v and rtl/ap040/ap040_alu.v directly, adding a    //
// couple of small, deliberately-additive changes to the shared files       //
// (a WRITE_THROUGH parameter, a debug tap). The user decided that's not    //
// the structure wanted: rtl/ap040/ stays completely untouched from here    //
// on, and rtl/ap040_pipe/ is fully self-contained -- deleting either        //
// directory can never affect the other. Both shared files are forked into  //
// this directory as ap040_pipe_regfile.v/ap040_pipe_alu.v (same collision- //
// avoidance naming as this module itself), and ap040_pipe_defs.svh no      //
// longer reaches into rtl/ap040/ for constants either.                     //
//                                                                          //
// Stall handshake: each stage's *_stall output is driven straight from its //
// own stall_in input (no stage has multi-cycle work yet), so the six-stage //
// chain currently collapses to "never stall" -- but the wiring is real,    //
// stage by stage, so a later milestone only has to change one stage's      //
// local stall condition rather than restructure the pipeline. The whole    //
// core advances only when ce is high, matching the clock-enable discipline //
// used throughout rtl/ap040/ap040_core.v.                                  //
//                                                                          //
// Commit and forwarding: the register file is instantiated here (not      //
// inside a stage module), mirroring where the existing sequential core     //
// instantiates it (ap040_core.v:222-233). The write port is driven         //
// directly off exe_* (EX's registered output -- the instruction            //
// conceptually "in WB" this cycle). A same-cycle read of the register      //
// being written is resolved inside the regfile itself (unconditional in    //
// this fork -- see ap040_pipe_regfile.v); the other hazard case -- a        //
// producer still combinationally computing in EX, one stage ahead of a      //
// consumer's EA-fetch cycle -- is resolved in ap040_ea_fetch.v via the      //
// ex_fwd_* bus below. See ap040_ea_fetch.v's header comment for the full    //
// picture, including the mutation test that confirmed a separate            //
// WB-forward mux there would have been redundant.                          //
//                                                                          //
// CCR/SR forwarding: the same write-through shape used for GPRs is reused   //
// for the architectural SR register: sr_resolved = commit_sr ? exe_sr_data  //
// : commit_ccr ? {sr[15:5],exe_result_flags} : sr (milestone 15 widened     //
// this from a CCR-only 5-bit version -- see its own note below). This is    //
// the only forwarding source ap040_execute.v's Bcc condition check (and,    //
// since milestone 6, Scc's byte fill) needs -- see its header comment for   //
// why an EX-live tap (the second source GPR forwarding needed) isn't        //
// necessary here.                                                          //
//                                                                          //
// commit_reg vs commit_ccr (new this milestone): these were a single        //
// `commit` signal through milestone 5, correct because every writing        //
// instruction so far (MOVEQ/MOVE.L/ADD.L) always did both together. Scc     //
// breaks that -- it writes a register but must never touch CCR (confirmed   //
// against ap040_core.v:2122-2130's EK_SCC) -- so the regfile write enable   //
// and the CCR update enable are now genuinely separate signals, gated by    //
// exe_writes_reg and exe_writes_ccr respectively.                          //
//                                                                          //
// Field set note (2026-08-21 code-quality pass): the original milestone-2  //
// draft threaded opcode/is_nop/is_move_rr/unimpl through every stage as    //
// one-hot-ish flags. A review found most of that unread past the stage     //
// that produced it, and the scheme didn't generalize. Decode now emits a   //
// compact control word instead; see ap040_decode.v's header for the        //
// reasoning, confirmed again by milestone 3 (ADD) and milestone 6 (Scc)     //
// each needing only new fields/values, never a restructure.                //
//                                                                          //
// Milestone 4 added BRA.B's redirect as a genuinely separate, orthogonal   //
// path (id_redirect_valid/id_redirect_pc, decode -> IF) rather than         //
// folding it into the control word -- it isn't an ALU operation or a        //
// register write, it's IF's own next-fetch address.                        //
//                                                                          //
// Milestone 5 generalized that redirect to the whole Bcc family (BRA is     //
// just Bcc's always-true case, per ap040_decode.v's header) and added the   //
// real misprediction path: ex_mispredict/ex_recovery_pc (from EX, once the  //
// real condition is known) take PRIORITY over decode's speculative guess    //
// at IF's redirect mux, and broadcast as `flush` to ID/EA-calc/EA-fetch to  //
// discard whatever was speculatively advanced down the (wrong) guess.       //
//                                                                          //
// Milestone 7 adds Bcc.W/Bcc.L (16-/32-bit displacement) via a multi-word   //
// gather state machine confined entirely to ap040_decode.v -- IF still      //
// hands over one word per cycle, oblivious to instruction boundaries; see   //
// its header comment. The one change visible at this level is *_next_pc,   //
// threaded alongside *_pc through every stage: ex_recovery_pc now reads     //
// eaf_next_pc directly instead of computing eaf_pc + 32'd2, since that       //
// arithmetic assumed every instruction was exactly one word (true by        //
// coincidence until Bcc.W/Bcc.L).                                          //
//                                                                          //
// Milestone 8 adds DBcc via a THIRD gate on the same commit signals rather   //
// than new ones: *_is_dbcc threads alongside *_is_branch/*_is_scc through    //
// every stage (mechanical pass-through, same shape as those two), and        //
// ap040_execute.v's writes_reg_resolved -- not this file -- is what makes     //
// commit_reg dynamic for it (DBcc writes Dn only when its runtime condition   //
// says so; see ap040_execute.v's header). ex_mispredict/ex_recovery_pc are    //
// reused completely unchanged: DBcc's "don't branch" outcomes are, from       //
// this file's point of view, indistinguishable from a not-taken Bcc.         //
//                                                                          //
// Milestone 13 adds BSR/JSR and, with them, this pipeline's first real        //
// memory WRITE: u_l1's port B (data_b/wren_b) is no longer tied off, driven    //
// instead by ap040_ea_fetch.v's own l1_data_b/l1_wren_b outputs -- the SAME     //
// port A/JMP/JSR's reads already share, since an instruction is never both      //
// a read and a push at once. *_is_bsr/*_is_jsr thread through the same           //
// mechanical pass-through shape as *_is_jmp; the actual push logic (address,      //
// data, stall) lives entirely in ap040_ea_fetch.v, and the A7 write reuses the     //
// existing commit_reg path below unchanged (ap040_decode.v points its              //
// eac_dest_reg at A7's unified index, same as any other register-writing            //
// instruction) -- nothing new needed at this level for that half either.             //
//                                                                          //
// Milestone 15 replaces the bare 5-bit `ccr` register with a real 16-bit    //
// `sr` (T1/T0/S/M/IPL/CCR, matching AP040_SR_RESET's layout exactly) and     //
// adds VBR/SFC/DFC/CACR -- the supervisor state MOVEC/MOVE-to-SR/privilege   //
// violation all need. sr_s/sr_m feeding ap040_pipe_regfile.v's A7 bank are   //
// finally REAL bits of live state (sr[13]/sr[12]), not hardwired constants;  //
// the regfile's own aux_we/aux_sel/aux_wdata port -- present since the        //
// register file was first written, unused until now -- gets its first real    //
// driver, for MOVEC's USP/ISP/MSP targets. Three new commit paths join         //
// commit_reg/commit_ccr: commit_sr (MOVE-to-SR, or an exception forcing S=1     //
// unconditionally on entry) writes the WHOLE sr register at once; commit_creg    //
// (MOVEC's write direction) writes exactly one of VBR/SFC/DFC/CACR/USP/ISP/       //
// MSP, selected by ap040_execute.v's exe_creg_sel. See ap040_ea_fetch.v's           //
// header for where the privilege check itself happens (dynamically, off the         //
// live S bit -- decode alone can't know it) and ap040_execute.v's header for         //
// the exact SR-masking arithmetic exception entry applies.                            //
//                                                                          //
// Milestone 16 adds RTS/RTE, closing the loop BSR/JSR (the push) and                //
// illegal/TRAP/priv (the exception-entry push) opened: this pipeline can now         //
// actually RETURN, not just enter. No new top-level architectural state is            //
// needed here at all -- both instructions reuse machinery this file already           //
// wires (the regfile's commit_reg path for A7's new value, commit_sr for RTE's         //
// popped SR) -- see ap040_ea_fetch.v's header for the new 2-beat pop sequencer          //
// RTE needed and why RTS didn't (it reuses mem_issue/mem_complete verbatim).             //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_core
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10,
	parameter         L1_AW      = 12   // ap040_pipe_l1.v size: 2**L1_AW words
)
(
	input  clk,
	input  nreset,
	input  ce,

	output        dbg_if_valid,  output [31:0] dbg_if_pc,
	output        dbg_id_valid,  output [31:0] dbg_id_pc,
	output        dbg_eac_valid, output [31:0] dbg_eac_pc,
	output        dbg_eaf_valid, output [31:0] dbg_eaf_pc,
	output        dbg_ex_valid,  output [31:0] dbg_ex_pc,
	output        dbg_wb_valid,  output [31:0] dbg_wb_pc,

	output [31:0] dbg_d0, output [31:0] dbg_d1, output [31:0] dbg_d2, output [31:0] dbg_d3,
	output [31:0] dbg_d4, output [31:0] dbg_d5, output [31:0] dbg_d6, output [31:0] dbg_d7,
	output  [4:0] dbg_ccr,
	// Full 16-bit SR (milestone 15, new) -- dbg_ccr stays sr[4:0] for every
	// existing testbench's sake; this is the ONLY way a test can observe
	// S/M/T1/T0/IPL without a real MOVE-from-SR instruction (deliberately
	// not built this milestone -- see ap040_decode.v's header).
	output [15:0] dbg_sr
);

wire        if_valid;  wire [31:0] if_pc;  wire [15:0] if_opcode;

wire        id_valid;  wire [31:0] id_pc;  wire [31:0] id_next_pc;
wire  [3:0] id_dest_reg, id_src_reg;
wire [31:0] id_imm;
wire  [5:0] id_alu_op;
wire        id_src_a_is_imm, id_writes_reg, id_writes_ccr;
wire        id_is_branch, id_is_scc, id_is_dbcc, id_is_mem_src, id_is_jmp;
wire        id_is_bsr, id_is_jsr, id_is_trap, id_is_illegal;
wire        id_is_movesr, id_is_movec;
wire        id_is_rts, id_is_rte;
wire  [3:0] id_cond;

wire        eac_valid; wire [31:0] eac_pc; wire [31:0] eac_next_pc;
wire  [3:0] eac_dest_reg, eac_src_reg;
wire [31:0] eac_imm;
wire  [5:0] eac_alu_op;
wire        eac_src_a_is_imm, eac_writes_reg, eac_writes_ccr;
wire        eac_is_branch, eac_is_scc, eac_is_dbcc, eac_is_mem_src, eac_is_jmp;
wire        eac_is_bsr, eac_is_jsr, eac_is_trap, eac_is_illegal;
wire        eac_is_movesr, eac_is_movec;
wire        eac_is_rts, eac_is_rte;
wire  [3:0] eac_cond;

wire        eaf_valid; wire [31:0] eaf_pc; wire [31:0] eaf_next_pc;
wire  [3:0] eaf_dest_reg;
wire [31:0] eaf_operand_a, eaf_operand_b;
wire  [5:0] eaf_alu_op;
wire        eaf_writes_reg, eaf_writes_ccr;
wire        eaf_is_branch, eaf_is_scc, eaf_is_dbcc, eaf_is_jmp;
wire        eaf_is_bsr, eaf_is_jsr, eaf_is_trap, eaf_is_illegal;
wire        eaf_is_priv, eaf_is_movesr, eaf_is_movec;
wire        eaf_is_addrerr;
wire        eaf_movec_dir;
wire  [2:0] eaf_movec_sel;
wire [15:0] eaf_sr_snapshot;
wire        eaf_is_rts, eaf_is_rte;
wire [15:0] eaf_rte_sr_data;
wire  [3:0] eaf_cond;

wire        exe_valid; wire [31:0] exe_pc;
wire  [3:0] exe_dest_reg;
wire [31:0] exe_result_data;
wire        exe_writes_reg, exe_writes_ccr;
wire  [4:0] exe_result_flags;
wire        exe_writes_sr;
wire [15:0] exe_sr_data;
wire        exe_writes_creg;
wire  [2:0] exe_creg_sel;
wire [31:0] exe_creg_data;

wire        wb_valid;  wire [31:0] wb_pc;

// Backward stall chain: WB has no downstream, so its stall_in is tied 0;
// every earlier stage's stall_in is the next stage's *_stall output.
wire id_stall, ea_stall, eaf_stall, ex_stall, wb_stall;

// Bcc/BRA speculative redirect (decode -> IF), see ap040_decode.v's header.
wire        id_redirect_valid;
wire [31:0] id_redirect_pc;

// Misprediction recovery (EX -> everything upstream of it), see
// ap040_execute.v's header for why the check happens at EX.
wire        ex_mispredict;
wire [31:0] ex_recovery_pc;

// Recovery takes priority over a fresh speculative guess -- fixing a
// confirmed wrong guess matters more than starting a new one the same
// cycle (and in practice they concern different instructions anyway).
wire        final_redirect_valid = ex_mispredict || id_redirect_valid;
wire [31:0] final_redirect_pc    = ex_mispredict ? ex_recovery_pc : id_redirect_pc;

// Broadcast flush: discards whatever ID/EA-calc/EA-fetch are currently
// holding, all speculatively advanced down the (wrong) assumed-taken guess.
wire flush = ex_mispredict;

// EA-fetch's regfile operand read ports.
wire  [3:0] raddr_a, raddr_b;
wire [31:0] rdata_a, rdata_b;

// EX-forward tap (combinational, live this cycle). The other hazard case --
// a producer committing the same cycle a consumer reads it -- needs no
// separate top-level wiring: it's resolved inside ap040_pipe_regfile.v
// itself. See ap040_ea_fetch.v's header comment for the mutation test that
// confirmed this empirically.
wire        ex_fwd_valid;
wire  [3:0] ex_fwd_dest;
wire [31:0] ex_fwd_data;

// SR's OWN EX-forward tap (milestone 15, new) -- see ap040_execute.v's
// header for why this exists in addition to sr_resolved's write-through
// (commit_sr) term below: a MOVE-to-SR/exception still sitting in EX this
// cycle, not yet committed, must still be visible to whatever's reading
// live S/M bits one stage EARLIER (ap040_ea_fetch.v's privilege check,
// ap040_pipe_regfile.v's own A7 bank select) the SAME cycle.
wire        ex_sr_fwd_valid;
wire [15:0] ex_sr_fwd_data;

// The instruction committing this cycle. Four separate gates, not one --
// see this file's header comment on commit_reg vs commit_ccr, and the new
// milestone-15 note on commit_sr/commit_creg.
wire commit_reg  = exe_valid && exe_writes_reg;
wire commit_ccr  = exe_valid && exe_writes_ccr;
wire commit_sr   = exe_valid && exe_writes_sr;
wire commit_creg = exe_valid && exe_writes_creg;

// Architectural SR (milestone 15: widened from a bare 5-bit CCR to the real
// 16-bit register -- T1/T0/S/M/-/IPL/-/-/-/CCR, AP040_SR_RESET's own
// layout). commit_sr (MOVE-to-SR, or exception entry forcing S=1/T1:T0=00 --
// see ap040_execute.v's header for exactly which) writes all 16 bits at
// once; commit_ccr (every ordinary flag-setting ALU op) still only ever
// touches the low 5 -- Scc/DBcc/etc.'s existing behavior is completely
// unchanged, this is a strictly additive second write path, not a
// replacement of the first.
reg [15:0] sr;

always @(posedge clk) begin
	if (!nreset)
		sr <= `AP040_SR_RESET;
	else if (ce) begin
		if (commit_sr)      sr <= exe_sr_data;
		else if (commit_ccr) sr[4:0] <= exe_result_flags;
	end
end

assign dbg_ccr = sr[4:0];
assign dbg_sr  = sr;

// SR resolution: THREE sources, priority-ordered, not two -- ex_sr_fwd_valid
// (a producer still IN EX this cycle, not yet committed -- the same "one
// stage ahead" case ex_fwd_* covers for GPRs) takes priority over commit_sr
// (a producer committing THIS cycle, mirroring the regfile's own write-
// through bypass and CCR's prior single-purpose version of this same
// mux), which takes priority over the plain registered value. Both new
// terms were added together, in the same milestone-15 pass that first
// gave ap040_ea_fetch.v a live SR consumer earlier than EX -- see
// ap040_execute.v's header for the actual hazard this fixes (verified by
// tb_ap040_pipe_sup.v initially FAILING without ex_sr_fwd_valid: a BSR one
// instruction behind a MOVE-to-SR read A7 through the stale, pre-switch
// bank for exactly one cycle). ap040_execute.v's Bcc/Scc condition check
// only ever reads the low 5 bits of this (still wired as a separate
// `ccr_in` port there, unchanged); ap040_ea_fetch.v's exception-frame push
// and privilege check need the FULL 16 bits, hence sr_resolved staying the
// whole register, not just CCR.
wire [15:0] sr_resolved = ex_sr_fwd_valid ? ex_sr_fwd_data :
                          commit_sr       ? exe_sr_data :
                          commit_ccr      ? {sr[15:5], exe_result_flags} : sr;

//---------------------------------------------------------------------------
// Supervisor control registers (milestone 15, new): VBR/SFC/DFC/CACR, the
// four this core's current scope actually needs (the MMU registers --
// TC/ITT0/ITT1/DTT0/DTT1/URP/SRP/MMUSR -- are explicitly out of scope, per
// the user's own framing; ap040_decode.v's MOVEC selector validation
// rejects them as illegal rather than silently accepting or dropping them).
// USP/ISP/MSP already exist -- ap040_pipe_regfile.v has held all three
// since this fork was first written -- so they're not duplicated here; see
// its aux_we wiring below for how MOVEC reaches them directly, bypassing
// whichever bank sr_s/sr_m currently have A7 pointed at.
//
// CACR is intentionally a plain, behaviorally inert register: this
// substrate's unified L1 (see section 5a of AP040_IMPLEMENTATION_PLAN.md)
// has no per-way enable/disable concept to actually gate, the same
// "diminished capacity" the plan doc already flagged for CINV/CPUSH.
// Masked identically to ap040_core.v's own S_MOVEC2 (`& 32'h8000_8000`) so
// a read-back is bit-exact even though neither bit does anything here.
//---------------------------------------------------------------------------

reg [31:0] vbr;
reg  [2:0] sfc, dfc;
reg [31:0] cacr;

always @(posedge clk) begin
	if (!nreset) begin
		vbr  <= 32'h0;
		sfc  <= 3'h0;
		dfc  <= 3'h0;
		cacr <= 32'h0;
	end else if (ce && commit_creg) begin
		case (exe_creg_sel)
			`AP040_CREG_SFC:  sfc  <= exe_creg_data[2:0];
			`AP040_CREG_DFC:  dfc  <= exe_creg_data[2:0];
			`AP040_CREG_CACR: cacr <= exe_creg_data & 32'h8000_8000;
			`AP040_CREG_VBR:  vbr  <= exe_creg_data;
			default: ;   // USP/ISP/MSP route through the regfile's aux port instead
		endcase
	end
end

wire [31:0] usp_q, isp_q, msp_q;   // ap040_pipe_regfile.v's own state, read
                                    // back here for MOVEC's read direction

// MOVEC's write direction reaching USP/ISP/MSP bypasses ap040_pipe_regfile.v's
// normal commit_reg path (which would bank through whichever of the three
// sr_s/sr_m currently selects) and drives its aux port directly instead --
// exactly the mechanism that port has existed for since this fork was first
// written, unused until now. aux_sel's 0=USP/1=ISP/2=MSP numbering is one
// bit-shift away from AP040_CREG_USP/ISP/MSP's own 4/5/6 -- see
// ap040_pipe_defs.svh's comment on why that ordering was chosen.
wire        aux_we    = commit_creg && (exe_creg_sel == `AP040_CREG_USP ||
                                          exe_creg_sel == `AP040_CREG_ISP ||
                                          exe_creg_sel == `AP040_CREG_MSP);
wire [1:0]  aux_sel    = exe_creg_sel[1:0];   // USP=4'b100->00, ISP=101->01, MSP=110->10
wire [31:0] aux_wdata  = exe_creg_data;

//---------------------------------------------------------------------------
// Register file: this directory's own fork (ap040_pipe_regfile.v, see its
// header), instantiated here rather than inside a stage, mirroring where
// the sequential core instantiates its own (ap040_core.v:222-233). Read
// port B is the destination register's current value, read unconditionally
// for every instruction -- see ap040_ea_fetch.v's header comment for why
// that's safe and why no control bit gates it.
//---------------------------------------------------------------------------

ap040_pipe_regfile u_regfile
(
	.clk      (clk),
	.ce       (ce),
	.nreset   (nreset),

	// Real, live bits now (milestone 15) -- was hardwired 1'b1/1'b0 through
	// milestone 14, since nothing touched them yet. sr_resolved, not the
	// raw sr register: MOVE-to-SR's own write-through forward, same reason
	// every other same-cycle producer/consumer pair in this pipeline needs
	// one -- an immediately-following instruction that touches A7 (say, a
	// BSR right after a MOVE-to-SR that just dropped to user mode) is in
	// EA-fetch reading THIS port the exact cycle MOVE-to-SR's commit lands;
	// the raw registered `sr` wouldn't reflect that write until the NEXT
	// cycle, banking A7 through the stale PRE-switch stack for one cycle.
	// sr[13]/sr[12] reset to AP040_SR_RESET's own S=1/M=0, so A7 still
	// banks to ISP at reset, unchanged behavior for every earlier test.
	.sr_s     (sr_resolved[13]),
	.sr_m     (sr_resolved[12]),

	.we       (commit_reg),
	.waddr    (exe_dest_reg),
	.wdata    (exe_result_data),

	.raddr_a  (raddr_a),
	.rdata_a  (rdata_a),
	.raddr_b  (raddr_b),
	.rdata_b  (rdata_b),

	.aux_we   (aux_we),
	.aux_sel  (aux_sel),
	.aux_wdata(aux_wdata),
	.usp_q    (usp_q),
	.isp_q    (isp_q),
	.msp_q    (msp_q),

	.dbg_d0   (dbg_d0),
	.dbg_d1   (dbg_d1),
	.dbg_d2   (dbg_d2),
	.dbg_d3   (dbg_d3),
	.dbg_d4   (dbg_d4),
	.dbg_d5   (dbg_d5),
	.dbg_d6   (dbg_d6),
	.dbg_d7   (dbg_d7),
	.dbg_a0   (),
	.dbg_a7   ()
);

//---------------------------------------------------------------------------
// Stages
//---------------------------------------------------------------------------

// Unified L1. Port A is IF's (milestone 9a). Port B is EA-fetch's data
// read (milestone 9b, MOVE.L/JMP/JSR) AND, since milestone 13, its write
// (BSR/JSR's push) -- both directions driven by the SAME EA-fetch instance,
// mutually exclusive per the always-one-instruction-at-a-time discipline
// this pipeline already has.
wire [L1_AW-1:0] l1_addr_a;
wire      [15:0] l1_rdata_a;
wire [L1_AW-1:0] l1_addr_b;
wire       [31:0] l1_q_b;
wire              l1_wren_b;
wire       [31:0] l1_data_b;
wire              l1_wr_busy;

ap040_pipe_l1 #(
	.AW(L1_AW),
	.DW(16)
) u_l1
(
	.clock     (clk),
	.nreset    (nreset),

	.address_a (l1_addr_a),
	.data_a    (16'h0),
	.wren_a    (1'b0),
	// Must match ap040_inst_fetch.v's OWN if_pc/pc update condition exactly
	// -- see ap040_pipe_l1.v's header (milestone 10 fix: without this, a
	// stall lets q_a free-run past the word if_opcode is supposed to keep
	// presenting).
	.en_a      (ce && !id_stall),
	.q_a       (l1_rdata_a),

	.address_b (l1_addr_b),
	.data_b    (l1_data_b),
	.wren_b    (l1_wren_b),
	.wr_busy   (l1_wr_busy),
	.q_b       (l1_q_b)
);

ap040_inst_fetch #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS),
	.L1_AW     (L1_AW)
) u_if
(
	.clk       (clk),
	.nreset    (nreset),
	.ce        (ce),
	.stall_in  (id_stall),

	.redirect_valid (final_redirect_valid),
	.redirect_pc    (final_redirect_pc),

	.l1_addr_a  (l1_addr_a),
	.l1_rdata_a (l1_rdata_a),

	.if_valid  (if_valid),
	.if_pc     (if_pc),
	.if_opcode (if_opcode)
);

ap040_decode u_id
(
	.clk             (clk),
	.nreset          (nreset),
	.ce              (ce),
	.stall_in        (ea_stall),
	.flush           (flush),

	.if_valid        (if_valid),
	.if_pc           (if_pc),
	.if_opcode       (if_opcode),

	.id_stall        (id_stall),

	.id_redirect_valid (id_redirect_valid),
	.id_redirect_pc    (id_redirect_pc),

	.id_valid        (id_valid),
	.id_pc           (id_pc),
	.id_next_pc      (id_next_pc),
	.id_dest_reg     (id_dest_reg),
	.id_src_reg      (id_src_reg),
	.id_imm          (id_imm),
	.id_alu_op       (id_alu_op),
	.id_src_a_is_imm (id_src_a_is_imm),
	.id_writes_reg   (id_writes_reg),
	.id_writes_ccr   (id_writes_ccr),
	.id_is_branch    (id_is_branch),
	.id_is_scc       (id_is_scc),
	.id_is_dbcc      (id_is_dbcc),
	.id_is_mem_src   (id_is_mem_src),
	.id_is_jmp       (id_is_jmp),
	.id_is_bsr       (id_is_bsr),
	.id_is_jsr       (id_is_jsr),
	.id_is_trap      (id_is_trap),
	.id_is_illegal   (id_is_illegal),
	.id_is_movesr    (id_is_movesr),
	.id_is_movec     (id_is_movec),
	.id_is_rts       (id_is_rts),
	.id_is_rte       (id_is_rte),
	.id_cond         (id_cond)
);

ap040_ea_calc u_eac
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (eaf_stall),
	.flush            (flush),

	.id_valid         (id_valid),
	.id_pc            (id_pc),
	.id_next_pc       (id_next_pc),
	.id_dest_reg      (id_dest_reg),
	.id_src_reg       (id_src_reg),
	.id_imm           (id_imm),
	.id_alu_op        (id_alu_op),
	.id_src_a_is_imm  (id_src_a_is_imm),
	.id_writes_reg    (id_writes_reg),
	.id_writes_ccr    (id_writes_ccr),
	.id_is_branch     (id_is_branch),
	.id_is_scc        (id_is_scc),
	.id_is_dbcc       (id_is_dbcc),
	.id_is_mem_src    (id_is_mem_src),
	.id_is_jmp        (id_is_jmp),
	.id_is_bsr        (id_is_bsr),
	.id_is_jsr        (id_is_jsr),
	.id_is_trap       (id_is_trap),
	.id_is_illegal    (id_is_illegal),
	.id_is_movesr     (id_is_movesr),
	.id_is_movec      (id_is_movec),
	.id_is_rts        (id_is_rts),
	.id_is_rte        (id_is_rte),
	.id_cond          (id_cond),

	.ea_stall         (ea_stall),

	.eac_valid        (eac_valid),
	.eac_pc           (eac_pc),
	.eac_next_pc      (eac_next_pc),
	.eac_dest_reg     (eac_dest_reg),
	.eac_src_reg      (eac_src_reg),
	.eac_imm          (eac_imm),
	.eac_alu_op       (eac_alu_op),
	.eac_src_a_is_imm (eac_src_a_is_imm),
	.eac_writes_reg   (eac_writes_reg),
	.eac_writes_ccr   (eac_writes_ccr),
	.eac_is_branch    (eac_is_branch),
	.eac_is_scc       (eac_is_scc),
	.eac_is_dbcc      (eac_is_dbcc),
	.eac_is_mem_src   (eac_is_mem_src),
	.eac_is_jmp       (eac_is_jmp),
	.eac_is_bsr       (eac_is_bsr),
	.eac_is_jsr       (eac_is_jsr),
	.eac_is_trap      (eac_is_trap),
	.eac_is_illegal   (eac_is_illegal),
	.eac_is_movesr    (eac_is_movesr),
	.eac_is_movec     (eac_is_movec),
	.eac_is_rts       (eac_is_rts),
	.eac_is_rte       (eac_is_rte),
	.eac_cond         (eac_cond)
);

ap040_ea_fetch #(
	.PC_RESET (PC_RESET),
	.L1_AW    (L1_AW)
) u_eaf
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (ex_stall),
	.flush            (flush),

	.eac_valid        (eac_valid),
	.eac_pc           (eac_pc),
	.eac_next_pc      (eac_next_pc),
	.eac_dest_reg     (eac_dest_reg),
	.eac_src_reg      (eac_src_reg),
	.eac_imm          (eac_imm),
	.eac_alu_op       (eac_alu_op),
	.eac_src_a_is_imm (eac_src_a_is_imm),
	.eac_writes_reg   (eac_writes_reg),
	.eac_writes_ccr   (eac_writes_ccr),
	.eac_is_branch    (eac_is_branch),
	.eac_is_scc       (eac_is_scc),
	.eac_is_dbcc      (eac_is_dbcc),
	.eac_is_mem_src   (eac_is_mem_src),
	.eac_is_jmp       (eac_is_jmp),
	.eac_is_bsr       (eac_is_bsr),
	.eac_is_jsr       (eac_is_jsr),
	.eac_is_trap      (eac_is_trap),
	.eac_is_illegal   (eac_is_illegal),
	.eac_is_movesr    (eac_is_movesr),
	.eac_is_movec     (eac_is_movec),
	.eac_is_rts       (eac_is_rts),
	.eac_is_rte       (eac_is_rte),
	.eac_cond         (eac_cond),

	.sr_in            (sr_resolved),
	.isp_in           (isp_q),
	.msp_in           (msp_q),

	.raddr_a          (raddr_a),
	.rdata_a          (rdata_a),
	.raddr_b          (raddr_b),
	.rdata_b          (rdata_b),

	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),

	.l1_addr_b        (l1_addr_b),
	.l1_q_b           (l1_q_b),
	.l1_wren_b        (l1_wren_b),
	.l1_data_b        (l1_data_b),
	.l1_wr_busy       (l1_wr_busy),

	.eaf_stall        (eaf_stall),

	.eaf_valid        (eaf_valid),
	.eaf_pc           (eaf_pc),
	.eaf_next_pc      (eaf_next_pc),
	.eaf_dest_reg     (eaf_dest_reg),
	.eaf_operand_a    (eaf_operand_a),
	.eaf_operand_b    (eaf_operand_b),
	.eaf_alu_op       (eaf_alu_op),
	.eaf_writes_reg   (eaf_writes_reg),
	.eaf_writes_ccr   (eaf_writes_ccr),
	.eaf_is_branch    (eaf_is_branch),
	.eaf_is_scc       (eaf_is_scc),
	.eaf_is_dbcc      (eaf_is_dbcc),
	.eaf_is_jmp       (eaf_is_jmp),
	.eaf_is_bsr       (eaf_is_bsr),
	.eaf_is_jsr       (eaf_is_jsr),
	.eaf_is_trap      (eaf_is_trap),
	.eaf_is_illegal   (eaf_is_illegal),
	.eaf_is_priv      (eaf_is_priv),
	.eaf_is_addrerr   (eaf_is_addrerr),
	.eaf_is_movesr    (eaf_is_movesr),
	.eaf_is_movec     (eaf_is_movec),
	.eaf_movec_dir    (eaf_movec_dir),
	.eaf_movec_sel    (eaf_movec_sel),
	.eaf_sr_snapshot  (eaf_sr_snapshot),
	.eaf_is_rts       (eaf_is_rts),
	.eaf_is_rte       (eaf_is_rte),
	.eaf_rte_sr_data  (eaf_rte_sr_data),
	.eaf_cond         (eaf_cond)
);

ap040_execute u_ex
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (wb_stall),

	.eaf_valid        (eaf_valid),
	.eaf_pc           (eaf_pc),
	.eaf_next_pc      (eaf_next_pc),
	.eaf_dest_reg     (eaf_dest_reg),
	.eaf_operand_a    (eaf_operand_a),
	.eaf_operand_b    (eaf_operand_b),
	.eaf_alu_op       (eaf_alu_op),
	.eaf_writes_reg   (eaf_writes_reg),
	.eaf_writes_ccr   (eaf_writes_ccr),
	.eaf_is_branch    (eaf_is_branch),
	.eaf_is_scc       (eaf_is_scc),
	.eaf_is_dbcc      (eaf_is_dbcc),
	.eaf_is_jmp       (eaf_is_jmp),
	.eaf_is_bsr       (eaf_is_bsr),
	.eaf_is_jsr       (eaf_is_jsr),
	.eaf_is_trap      (eaf_is_trap),
	.eaf_is_illegal   (eaf_is_illegal),
	.eaf_is_priv      (eaf_is_priv),
	.eaf_is_addrerr   (eaf_is_addrerr),
	.eaf_is_movesr    (eaf_is_movesr),
	.eaf_is_movec     (eaf_is_movec),
	.eaf_movec_dir    (eaf_movec_dir),
	.eaf_movec_sel    (eaf_movec_sel),
	.eaf_sr_snapshot  (eaf_sr_snapshot),
	.eaf_is_rts       (eaf_is_rts),
	.eaf_is_rte       (eaf_is_rte),
	.eaf_rte_sr_data  (eaf_rte_sr_data),
	.eaf_cond         (eaf_cond),

	.ccr_in           (sr_resolved[4:0]),

	.sfc_in           ({29'd0, sfc}),
	.dfc_in           ({29'd0, dfc}),
	.cacr_in          (cacr),
	.vbr_in           (vbr),
	.usp_in           (usp_q),
	.isp_in           (isp_q),
	.msp_in           (msp_q),

	.ex_stall         (ex_stall),

	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),

	.ex_sr_fwd_valid  (ex_sr_fwd_valid),
	.ex_sr_fwd_data   (ex_sr_fwd_data),

	.ex_mispredict    (ex_mispredict),
	.ex_recovery_pc   (ex_recovery_pc),

	.exe_valid        (exe_valid),
	.exe_pc           (exe_pc),
	.exe_dest_reg     (exe_dest_reg),
	.exe_result_data  (exe_result_data),
	.exe_writes_reg   (exe_writes_reg),
	.exe_writes_ccr   (exe_writes_ccr),
	.exe_result_flags (exe_result_flags),

	.exe_writes_sr    (exe_writes_sr),
	.exe_sr_data      (exe_sr_data),
	.exe_writes_creg  (exe_writes_creg),
	.exe_creg_sel     (exe_creg_sel),
	.exe_creg_data    (exe_creg_data)
);

ap040_writeback u_wb
(
	.clk              (clk),
	.nreset           (nreset),
	.ce               (ce),
	.stall_in         (1'b0),

	.exe_valid        (exe_valid),
	.exe_pc           (exe_pc),

	.wb_stall         (wb_stall),

	.wb_valid         (wb_valid),
	.wb_pc            (wb_pc)
);

assign dbg_if_valid  = if_valid;  assign dbg_if_pc  = if_pc;
assign dbg_id_valid  = id_valid;  assign dbg_id_pc  = id_pc;
assign dbg_eac_valid = eac_valid; assign dbg_eac_pc = eac_pc;
assign dbg_eaf_valid = eaf_valid; assign dbg_eaf_pc = eaf_pc;
assign dbg_ex_valid  = exe_valid; assign dbg_ex_pc  = exe_pc;
assign dbg_wb_valid  = wb_valid;  assign dbg_wb_pc  = wb_pc;

endmodule
