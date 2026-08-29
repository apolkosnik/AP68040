//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 17: address error)       //
//                                                                          //
// ap040_execute.v - EX stage                                              //
//                                                                          //
// Instantiates ap040_pipe_alu.v (this directory's own fork of              //
// rtl/ap040/ap040_alu.v -- see its header) unmodified -- it is purely      //
// combinational and stateless, so it drops into a single pipeline stage    //
// as-is. `op` and `b` are both plain data connections to decode/EA-fetch's //
// fields, not hardcoded -- MOVEQ/MOVE.L decode to AP040_ALU_MOVE (result = //
// a, N/Z from the result, V/C cleared, X passed through from flags_in --   //
// confirmed by reading ap040_pipe_alu.v:139-142) and ignore `b` entirely;  //
// ADD.L Dn,Dm decodes to AP040_ALU_ADD and uses both operands (confirmed   //
// against ap040_pipe_alu.v:144-147: result/flags_out come entirely from    //
// a/b, never flags_in, so this op needs no CCR forwarding). A further op   //
// only needs decode to emit a different eaf_alu_op -- no new mux logic     //
// here.                                                                    //
//                                                                          //
// Exposes the pipeline's "EX-forward" tap: the live combinational result   //
// this cycle (ALU output, or the Scc byte-merge below), for                //
// ap040_ea_fetch.v to bypass around the regfile when the producer is       //
// still here (one stage ahead of the consumer's own EA-fetch cycle) and    //
// hasn't committed yet. See ap040_ea_fetch.v's header comment for the      //
// full forwarding picture, including the port-B compare added in           //
// milestone 3 for the case where the destination register is also read     //
// as an ALU operand.                                                       //
//                                                                          //
// Bcc condition check happens HERE, not in decode or any earlier stage,    //
// and that placement was derived from the actual pipeline timing, not      //
// assumed: an immediately-adjacent flag-setter (the tightest case) is      //
// only committing (driving exe_result_flags/commit_ccr --                  //
// ap040_pipe_core.v's write-through-style ccr_in source) at the exact      //
// cycle the branch itself reaches EX. Checking any earlier (e.g. in id_*,  //
// where GPR fields are recognized) is too early -- the producer would      //
// still be sitting in EA-calc, short of both this core's forwarding        //
// sources. So ccr_in already carries a correctly-forwarded value by the    //
// time it reaches this port; no separate CCR-forward mux lives in this     //
// file, unlike the GPR case which needed one (ex_fwd_*) precisely because   //
// GPR reads happen a stage earlier, in ap040_ea_fetch.v.                   //
//                                                                          //
// cond_true() mirrors ap040_core.v:720-740's condition-code table exactly  //
// -- an independent copy, not a shared function, same non-coupling         //
// precedent as sxb/d_reg9 in ap040_decode.v (milestone 3): this core's      //
// decode/execute logic stays decoupled from the sequential core's. Every   //
// one of its 16 encodings (T/F/14 real conditions) is implemented, not a    //
// subset -- reused as-is by both Bcc and, this milestone, Scc, since they   //
// share the same 4-bit condition-code field position (if_opcode[11:8]).    //
//                                                                          //
// Scc.B Dn: result = cond_true(cc) ? 8'hFF : 8'h00, merged into             //
// eaf_operand_b's upper 24 bits (the destination register's current,       //
// already-forwarded value -- confirmed against ap040_core.v:2122-2130's    //
// EK_SCC: same fill, same merge, no flag change at all). The merge happens //
// here, in the caller, not inside ap040_pipe_alu.v -- matching that file's  //
// own documented convention that it never merges by size internally        //
// (callers do). "No flag change" is why exe_writes_ccr exists as a signal   //
// distinct from exe_writes_reg: Scc writes a register but must never        //
// update CCR.                                                              //
//                                                                          //
// ex_recovery_pc (changed milestone 7): now eaf_next_pc directly, not       //
// eaf_pc + 32'd2. Every instruction before Bcc.W/Bcc.L was exactly one      //
// word, so "+2" was correct only by coincidence of scope; ap040_decode.v    //
// now computes the actual next-instruction address itself (accounting for   //
// however many extension words a branch consumed) and threads it as its     //
// own field -- see its header comment. No arithmetic is left here at all.   //
//                                                                          //
// DBcc Dn,<label> (milestone 8, new): decode always speculatively           //
// redirected IF to the branch target (same "assume taken" policy as Bcc),   //
// so this stage's only new job is computing the REAL outcome and comparing  //
// it to that guess -- reusing ex_mispredict/ex_recovery_pc exactly as they   //
// already exist, no new redirect path. Per ap040_core.v's S_DBCC1/S_DBCC2    //
// (ap040_core.v:3040-3059) and its 0x5 decode group (ap040_core.v:5415-      //
// 5421), verified rather than assumed:                                      //
//   - if cond_true(cc): the loop terminates WITHOUT decrementing Dn at all   //
//     -- register write and branch are both suppressed.                     //
//   - else: Dn's low 16 bits decrement by one (high 16 bits pass through     //
//     unchanged -- a word-sized RMW on a 32-bit register, not a full-word    //
//     overwrite); if the result != $FFFF the branch IS taken (to the same    //
//     target decode already guessed), otherwise the loop has expired and     //
//     falls through. Either way -- taken or expired -- Dn's decremented      //
//     value is written back; only the cond_true case above skips the write   //
//     entirely.                                                             //
// This makes DBcc the first instruction whose exe_writes_reg depends on a    //
// RUNTIME value rather than a static decode-time bit: eaf_writes_reg stays   //
// 0 from decode (like Bcc), and writes_reg_resolved below overrides it        //
// dynamically off cond_result for eaf_is_dbcc, gating both the regfile        //
// commit (ap040_pipe_core.v's commit_reg) and the EX-forward tap the same     //
// way. exe_writes_ccr is untouched (stays 0, from decode) -- DBcc never        //
// affects flags on real hardware, confirmed by the absence of any CCR         //
// update in ap040_core.v's S_DBCC1/S_DBCC2.                                  //
//                                                                          //
// NOT implemented (deferred, same discipline as TRAPcc's exception-delivery  //
// dependency in milestone 6): the real 68040 checks the branch TARGET's       //
// parity before the condition, even on an iteration that would not have       //
// branched (ap040_core.v's own S_DBCC1 comment, citing cputest 68040_ae       //
// DBcc.W) -- an odd target always faults. That needs exception delivery       //
// (vector fetch, supervisor frame push), which this pipeline doesn't have     //
// yet; a target-parity check with nowhere to deliver the fault would be       //
// worse than not checking at all, so it's left for whichever milestone        //
// builds exception delivery, not silently guessed at here.                   //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_execute
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // WB cannot accept this cycle

	input             eaf_valid,
	input      [31:0] eaf_pc,
	input      [31:0] eaf_next_pc,
	input       [3:0] eaf_dest_reg,
	input      [31:0] eaf_operand_a,
	input      [31:0] eaf_operand_b,
	input       [5:0] eaf_alu_op,
	input             eaf_writes_reg,
	input             eaf_writes_ccr,
	input             eaf_is_branch,
	input             eaf_is_scc,
	input             eaf_is_dbcc,
	input             eaf_is_jmp,
	input             eaf_is_bsr,
	input             eaf_is_jsr,
	input             eaf_is_trap,
	input             eaf_is_illegal,
	input             eaf_is_priv,
	input             eaf_is_addrerr,
	input             eaf_is_movesr,
	input             eaf_is_movec,
	input             eaf_movec_dir,
	input       [2:0] eaf_movec_sel,
	input             eaf_is_rts,
	input             eaf_is_rte,
	input      [15:0] eaf_rte_sr_data,
	input       [3:0] eaf_cond,

	input       [4:0] ccr_in,    // already write-through-forwarded by
	                              // ap040_pipe_core.v -- see header comment
	// The OLD (pre-fault) SR, for exception entry's masking arithmetic --
	// ap040_ea_fetch.v's OWN already-correctly-forwarded read of the SAME
	// value (its exc_sr_word), threaded straight down rather than read
	// live again here: this stage's own EX-live SR forward (ex_sr_fwd_*
	// below) feeds INTO ap040_pipe_core.v's sr_resolved, so taking a fresh
	// "live" read of sr_resolved back INTO this stage would be a genuine
	// combinational loop, not just a staleness risk -- see header.
	input      [15:0] eaf_sr_snapshot,

	// MOVEC's read direction (control register -> GPR): the seven control
	// registers' CURRENT values, fed straight from ap040_pipe_core.v (their
	// actual home), not threaded through ap040_decode.v/ap040_ea_calc.v/
	// ap040_ea_fetch.v at all -- these are live architectural state, not
	// per-instruction pipeline data, the same reasoning ccr_in is wired
	// straight in rather than passed stage by stage.
	input      [31:0] sfc_in,
	input      [31:0] dfc_in,
	input      [31:0] cacr_in,
	input      [31:0] vbr_in,
	input      [31:0] usp_in,
	input      [31:0] isp_in,
	input      [31:0] msp_in,

	output            ex_stall,   // to EA-fetch: no local stall of its own yet

	// "EX-forward" tap (combinational, live this cycle)
	output            ex_fwd_valid,
	output      [3:0] ex_fwd_dest,
	output     [31:0] ex_fwd_data,

	// SR's OWN EX-forward tap (milestone 15, new) -- the same "producer
	// still here, one stage ahead of a consumer's EA-fetch cycle" case
	// ex_fwd_* already covers for GPRs, now needed for SR too: a MOVE-to-SR
	// (or an exception) sitting HERE, not yet committed, must still be
	// visible to whatever's reading the live S/M bits one stage earlier in
	// ap040_ea_fetch.v (its eac_is_priv check, or ap040_pipe_regfile.v's
	// own sr_s/sr_m bank select) THIS SAME cycle -- see
	// ap040_pipe_core.v's sr_resolved for where this feeds in.
	output            ex_sr_fwd_valid,
	output     [15:0] ex_sr_fwd_data,

	// Misprediction signal (combinational, live this cycle) -- see header
	// comment. ap040_pipe_core.v broadcasts this as `flush` to ID/EA-calc/
	// EA-fetch and redirects IF to ex_recovery_pc.
	output            ex_mispredict,
	output     [31:0] ex_recovery_pc,

	output reg        exe_valid,
	output reg [31:0] exe_pc,
	output reg  [3:0] exe_dest_reg,
	output reg [31:0] exe_result_data,
	output reg        exe_writes_reg,
	output reg        exe_writes_ccr,
	output reg  [4:0] exe_result_flags,

	// New commit paths, milestone 15 -- see header. Both gated by exe_valid
	// exactly like exe_writes_reg/exe_writes_ccr already are; neither ever
	// fires together with the other (MOVE-to-SR/an exception writes SR;
	// MOVEC's write direction writes a control register; mutually
	// exclusive by construction).
	output reg        exe_writes_sr,
	output reg [15:0] exe_sr_data,
	output reg        exe_writes_creg,
	output reg  [2:0] exe_creg_sel,
	output reg [31:0] exe_creg_data
);

assign ex_stall = stall_in;

wire [31:0] alu_result;
wire [4:0]  alu_flags;

ap040_pipe_alu alu
(
	.op        (eaf_alu_op),
	.size      (`AP040_SZ_L),
	.shcnt     (6'd1),
	.a         (eaf_operand_a),
	.b         (eaf_operand_b),
	.flags_in  (ccr_in),
	.result    (alu_result),
	.flags_out (alu_flags)
);

// ap040_core.v:720-740 equivalent. ccr_in bit order matches AP040_SR_C/V/
// Z/N (ap040_defs.svh): bit0=C, bit1=V, bit2=Z, bit3=N.
function cond_true;
	input [3:0] cond;
	input [4:0] ccr;
	begin
		case (cond)
			4'h0: cond_true = 1'b1;                              // T (BRA)
			4'h1: cond_true = 1'b0;                              // F (unused by Bcc -- BSR's slot)
			4'h2: cond_true = !ccr[0] && !ccr[2];                 // HI
			4'h3: cond_true =  ccr[0] ||  ccr[2];                 // LS
			4'h4: cond_true = !ccr[0];                            // CC
			4'h5: cond_true =  ccr[0];                            // CS
			4'h6: cond_true = !ccr[2];                            // NE
			4'h7: cond_true =  ccr[2];                            // EQ
			4'h8: cond_true = !ccr[1];                            // VC
			4'h9: cond_true =  ccr[1];                            // VS
			4'hA: cond_true = !ccr[3];                            // PL
			4'hB: cond_true =  ccr[3];                            // MI
			4'hC: cond_true =  ccr[3] ==  ccr[1];                 // GE
			4'hD: cond_true =  ccr[3] !=  ccr[1];                 // LT
			4'hE: cond_true = !ccr[2] && (ccr[3] == ccr[1]);      // GT
			default: cond_true = ccr[2] || (ccr[3] != ccr[1]);    // LE
		endcase
	end
endfunction

// Shared by Bcc (branch taken/not-taken), Scc (byte fill value), and DBcc
// (loop-terminate vs decrement-and-test) -- all three decode the condition
// into the same eaf_cond field, so this is the one evaluation all consume.
wire cond_result = cond_true(eaf_cond, ccr_in);

// DBcc: decrement is a word-sized RMW -- low 16 bits of operand_a (Dn, read
// via EA-fetch's normal port-A path since decode set src_reg=dest_reg=Dn),
// high 16 bits pass through untouched. Only meaningful when eaf_is_dbcc and
// cond_result is false (see header); computed unconditionally here because
// it's pure combinational arithmetic with no side effect on its own.
wire [15:0] dbcc_dec          = eaf_operand_a[15:0] - 16'd1;
wire        dbcc_expired      = (dbcc_dec == 16'hFFFF);
wire        dbcc_branch_taken = !cond_result && !dbcc_expired;
wire [31:0] dbcc_result       = {eaf_operand_a[31:16], dbcc_dec};

// DBcc writes Dn whenever it enters the decrement path at all (cond false),
// regardless of whether the decremented value then causes a taken branch or
// a loop-expired fall-through -- see header comment. Every other instruction
// still uses decode's static eaf_writes_reg unchanged.
wire writes_reg_resolved = eaf_is_dbcc ? (eaf_valid && !cond_result) : eaf_writes_reg;

// Both of DBcc's "don't branch" outcomes -- condition true, or the
// decremented counter expired -- are architecturally identical to Bcc's
// not-taken case from IF's point of view: decode guessed taken, so anything
// but a taken branch is a misprediction recovered to eaf_next_pc.
//
// JMP (milestone 11): decode never guesses a target for it at all (no
// literal displacement exists to speculate with -- see ap040_decode.v's
// header), so its implicit "guess" is IF's own default sequential advance.
// That's correct only by the coincidence of the target happening to equal
// eaf_next_pc, so JMP is modeled as an UNCONDITIONAL misprediction once it
// reaches EX -- reusing ex_mispredict/ex_recovery_pc/flush exactly as they
// already exist, no new redirect path. The recovery target is
// eaf_operand_a itself: ap040_ea_fetch.v routed the computed EA there
// instead of a register value (see its header) specifically so EX doesn't
// need a separate "JMP target" input.
//
// JSR (milestone 13): the SAME reasoning as JMP -- its target is An + a
// possible displacement, not known until a register is read, so decode
// never speculates one and JSR joins JMP in this OR-chain. BSR is
// deliberately NOT here: it reuses Bcc's byte/word/long gather and
// speculative decode-time redirect verbatim (unconditionally taken, always
// correct -- see ap040_decode.v's header), so by the time a BSR reaches EX
// the redirect has already happened and there is nothing left to correct.
//
// TRAP #n / illegal instruction / privilege violation / odd JMP-JSR target
// (milestones 14/15/17): the SAME reasoning as JMP/JSR one more time --
// none of the four has a literal target decode could possibly speculate
// with (the handler address comes from ap040_ea_fetch.v's own vector-table
// read, resolved into eaf_operand_a exactly like JMP/JSR's EA -- see its
// header), so all four join this OR-chain unconditionally. eaf_is_priv
// fires for a privilege violation on MOVE-to-SR/MOVEC/RTE; eaf_is_addrerr
// fires for an odd JMP/JSR target -- see ap040_ea_fetch.v's header for
// where both are actually detected (dynamic, not a decode-time fact the
// way illegal/TRAP are). Reusing this SAME aggregate wire is also why
// address error needed no further changes anywhere else in this file:
// ex_mispredict/ex_recovery_pc/combined_result/exe_writes_sr_c/
// exe_sr_data_c all key off exc_reaching_ex already, not the individual
// flags.
wire exc_reaching_ex = eaf_is_trap || eaf_is_illegal || eaf_is_priv || eaf_is_addrerr;

// RTS/RTE (milestone 16) join the SAME unconditional-redirect club one
// more time: RTS's popped PC (routed into eaf_operand_a exactly like
// JMP/JSR's EA, see ap040_ea_fetch.v's header for why it reuses
// mem_issue/mem_complete verbatim) and RTE's own popped PC (its 2-beat
// sequencer's own finalize) are both targets decode could never have
// speculated with either.
wire redirect_always = eaf_is_jmp || eaf_is_jsr || eaf_is_rts || eaf_is_rte ||
                        exc_reaching_ex;

assign ex_mispredict   = eaf_valid && ((eaf_is_branch && !cond_result) ||
                                        (eaf_is_dbcc   && !dbcc_branch_taken) ||
                                        redirect_always);
assign ex_recovery_pc  = redirect_always ? eaf_operand_a : eaf_next_pc;

wire [31:0] scc_fill   = {24'd0, {8{cond_result}}};
wire [31:0] scc_merged = {eaf_operand_b[31:8], scc_fill[7:0]};

// MOVEC's read direction (milestone 15): the selected control register's
// CURRENT value (fed straight in from ap040_pipe_core.v -- see the port
// list above), routed into combined_result exactly like any other GPR-
// writing instruction's result -- no new commit path needed for this
// direction at all, it rides the SAME commit_reg/ex_fwd_* machinery every
// register-writing instruction already uses. Meaningless (and unread) for
// the write direction, same "compute always, consume conditionally"
// precedent used throughout this pipeline.
wire [31:0] creg_read_value = (eaf_movec_sel == `AP040_CREG_SFC)  ? sfc_in  :
                               (eaf_movec_sel == `AP040_CREG_DFC)  ? dfc_in  :
                               (eaf_movec_sel == `AP040_CREG_CACR) ? cacr_in :
                               (eaf_movec_sel == `AP040_CREG_VBR)  ? vbr_in  :
                               (eaf_movec_sel == `AP040_CREG_USP)  ? usp_in  :
                               (eaf_movec_sel == `AP040_CREG_ISP)  ? isp_in  :
                                                                      msp_in;  // AP040_CREG_MSP

// BSR/JSR/TRAP/illegal/priv/RTS/RTE: eaf_operand_b already IS the result --
// ap040_ea_fetch.v computed the new A7 value there (the same expression it
// used for the push/pop address), so this stage just selects it, the same
// "precomputed elsewhere, EX only picks" shape scc_merged/dbcc_result
// already use. No generic-ALU op is used for this (eaf_operand_a is busy
// carrying the redirect/handler target and can't also carry a literal
// increment/decrement operand) -- see ap040_ea_fetch.v's header.
wire [31:0] combined_result = eaf_is_scc  ? scc_merged :
                               eaf_is_dbcc ? dbcc_result :
                               (eaf_is_bsr || eaf_is_jsr || eaf_is_rts || eaf_is_rte || exc_reaching_ex)
                                 ? eaf_operand_b :
                               (eaf_is_movec && !eaf_movec_dir) ? creg_read_value :
                                                                    alu_result;

// MOVE to SR (milestone 15, new): writes the WHOLE live SR, not just CCR --
// a genuinely different commit path from exe_writes_ccr's low-5-bits-only
// one (Scc/Bcc/DBcc's flags never touch S/M/T/IPL, and this instruction
// must never touch THEM through the ALU's flags_out path either). Its
// value is simply eaf_operand_a's low 16 bits -- the source Dn's already-
// forwarded value, same port A every register-direct source has used since
// milestone 2, no new operand path.
//
// Exception entry (illegal/TRAP/priv) ALSO writes the whole live SR: T1/T0
// cleared, S forced to 1 (entering the handler in supervisor mode
// unconditionally, per ap040_core.v's own S_EXC0: `sr[13]<=1; sr[15:14]<=
// 2'b00;`), M/IPL/CCR preserved from whatever they were the instant before
// the fault -- computed here off eaf_sr_snapshot (ap040_ea_fetch.v's OWN
// forwarded read of the SAME live SR, one cycle earlier, for the PUSHED
// copy -- see its header for why threading it down, not re-reading a live
// value here, is required, not just tidier).
//
// RTE (milestone 16) is a THIRD source: restores the SR it just popped
// (eaf_rte_sr_data, already masked with AP040_SR_MASK by
// ap040_ea_fetch.v's own finalize step -- see its header), the architectural
// inverse of the exception-entry case just above. All three sources are
// mutually exclusive by construction: a MOVE-to-SR or RTE that faults a
// privilege violation instead has its own eaf_is_movesr/eaf_is_rte cleared
// by ap040_ea_fetch.v's exc_vec_done branch, so exc_reaching_ex never
// overlaps with either.
wire        exe_writes_sr_c = eaf_valid && (eaf_is_movesr || eaf_is_rte || exc_reaching_ex);
wire [15:0] exe_sr_data_c   = exc_reaching_ex ? ((eaf_sr_snapshot & 16'h1FFF) | 16'h2000) :
                               eaf_is_rte      ? eaf_rte_sr_data :
                                                  eaf_operand_a[15:0];

// MOVEC's write direction (milestone 15, new): writes ONE of the seven
// control registers ap040_pipe_core.v now owns, selected by eaf_movec_sel
// (ap040_decode.v's validated selector code -- an invalid one never
// reaches here, having become id_is_illegal instead, see its header).
// Value is eaf_operand_a again, same source-register path as MOVE-to-SR.
// Suppressed (like eaf_is_movec itself) whenever this MOVEC just faulted a
// privilege violation instead -- ap040_ea_fetch.v already cleared
// eaf_is_movec for that case, so eaf_movec_dir is simply never consulted.
//
// RTE's A7 restore (milestone 16, new) ALSO routes through this same
// direct-to-ISP/MSP path, deliberately NOT through the normal commit_reg/
// A7-bank-selected write every other A7-touching instruction uses. A real
// bug, found by this exact test failing, not by inspection: RTE's SR
// restore (exe_writes_sr, above) and its A7 restore commit on the SAME
// cycle, and ap040_pipe_core.v's sr_resolved -- correctly, per its own
// milestone-15 fix -- makes a same-cycle SR write visible to THAT SAME
// cycle's regfile bank-select. Going through commit_reg here would mean
// RTE's OWN A7 write gets banked by the SR it is ITSELF in the middle of
// restoring -- i.e. the NEW (post-restore) S bit, not the OLD one active
// while the frame was actually being popped -- silently landing the
// restored ISP/MSP value in USP instead whenever RTE returns to a
// different mode than it ran in. Using exe_writes_creg's fixed ISP/MSP
// target (selected off eaf_sr_snapshot[12], the M bit AS OF ea_fetch's own
// cycle -- the OLD context, before this same instruction's restore lands)
// sidesteps the race entirely, exactly the same "bypass the live bank,
// write the supervisor stack directly" reasoning exc_sp_bank already
// established for the exception-entry push.
wire exe_writes_creg_c = (eaf_valid && eaf_is_movec && eaf_movec_dir) ||
                          (eaf_valid && eaf_is_rte);
wire  [2:0] exe_creg_sel_c  = eaf_is_rte ? (eaf_sr_snapshot[12] ? `AP040_CREG_MSP : `AP040_CREG_ISP)
                                          : eaf_movec_sel;
wire [31:0] exe_creg_data_c = eaf_is_rte ? eaf_operand_b : eaf_operand_a;

assign ex_fwd_valid = eaf_valid && writes_reg_resolved;
assign ex_fwd_dest  = eaf_dest_reg;
assign ex_fwd_data  = combined_result;

assign ex_sr_fwd_valid = exe_writes_sr_c;
assign ex_sr_fwd_data  = exe_sr_data_c;

always @(posedge clk) begin
	if (!nreset) begin
		exe_valid        <= 1'b0;
		exe_pc           <= 32'h0;
		exe_dest_reg     <= 4'h0;
		exe_result_data  <= 32'h0;
		exe_writes_reg   <= 1'b0;
		exe_writes_ccr   <= 1'b0;
		exe_result_flags <= 5'h0;
		exe_writes_sr    <= 1'b0;
		exe_sr_data      <= 16'h0;
		exe_writes_creg  <= 1'b0;
		exe_creg_sel     <= 3'h0;
		exe_creg_data    <= 32'h0;
	end else if (ce && !stall_in) begin
		exe_valid        <= eaf_valid;
		exe_pc           <= eaf_pc;
		exe_dest_reg     <= eaf_dest_reg;
		exe_result_data  <= combined_result;
		exe_writes_reg   <= writes_reg_resolved;
		exe_writes_ccr   <= eaf_writes_ccr;
		exe_result_flags <= alu_flags;
		exe_writes_sr    <= exe_writes_sr_c;
		exe_sr_data      <= exe_sr_data_c;
		exe_writes_creg  <= exe_writes_creg_c;
		exe_creg_sel     <= exe_creg_sel_c;
		exe_creg_data    <= exe_creg_data_c;
	end
end

endmodule
