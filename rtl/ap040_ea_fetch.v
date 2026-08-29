//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 17: address error)       //
//                                                                          //
// ap040_ea_fetch.v - EA-fetch stage                                       //
//                                                                          //
// This is where the pipeline's register forwarding lives. The top-level   //
// regfile (ap040_pipe_core.v, its own ap040_pipe_regfile.v fork) is read    //
// combinationally here on BOTH ports -- port A at eac_src_reg, port B at   //
// eac_dest_reg -- and each resolved, independently, as a flat priority mux //
// rather than a chain of ternaries:                                       //
//                                                                          //
//   ex_fwd_*  the instruction currently in EX (still combinational this   //
//             cycle, not yet committed) -- "producer one stage ahead"     //
//                                                                          //
// Port B (eac_dest_reg) is read unconditionally, for every instruction,   //
// not just ones that need it: it's a pure regfile read with no side       //
// effect, and AP040_ALU_MOVE (MOVEQ/MOVE.L's op) never references its `b` //
// input at all (ap040_pipe_alu.v:139-142), so fetching it for those two is //
// simply unused, not wrong -- and the same is true for a branch, whose      //
// eac_dest_reg field is meaningless leftover bits, harmless because         //
// eac_writes_reg is always 0 for one. Scc, this milestone, is the first     //
// instruction that actually NEEDS port B's value for something other than  //
// forwarding to a later instruction: it's the register Scc's byte-merge     //
// preserves the upper bits of (ap040_execute.v). This is the                //
// generalization the milestone-2 altitude review asked for instead of a     //
// per-opcode special case: adding another instruction needs no new mux      //
// shape here.                                                               //
//                                                                          //
// The other hazard case -- a producer committing to the regfile the exact //
// same cycle a consumer reads it -- does NOT need a second mux on either   //
// port: it's resolved inside ap040_pipe_regfile.v itself (its write-through //
// bypass, unconditional in that file -- see its header). A mutation test    //
// during milestone 2 development confirmed this empirically for port A --   //
// forcing this stage's own would-be WB-forward mux to always miss still     //
// passed, because rdata_a was already correct by the time it got here.      //
// Keeping a second, provably redundant mux here would be dead logic; see    //
// ap040_pipe_core.v for the write-through wiring.                           //
//                                                                          //
// id_is_branch/id_is_scc/id_is_dbcc/id_cond/id_next_pc ride through          //
// unchanged -- this stage has nothing to compute for any of them;            //
// ap040_execute.v is where the condition is actually checked, the Scc merge  //
// actually happens, the DBcc decrement/compare happens, and eaf_next_pc is   //
// finally consumed (as ex_recovery_pc), per its header comment on why.       //
//                                                                          //
// DBcc (milestone 8) needs no new mux shape here either: its loop-counter    //
// register is both eac_src_reg and eac_dest_reg (ap040_decode.v sets both     //
// to the same Dn), so operand_a already resolves to Dn's current,             //
// correctly-forwarded value via the existing port-A priority mux -- exactly   //
// the read a decrement needs, with no dedicated DBcc case added to either     //
// port's select logic.                                                      //
//                                                                          //
// flush: when ap040_execute.v detects a mispredicted branch, everything     //
// speculatively fetched behind it -- including whatever is sitting here -- //
// must be discarded. Same shape as stall_in but forces a bubble instead     //
// of holding.                                                              //
//                                                                          //
// MOVE.L (An),Dn (milestone 9b, new) is this pipeline's first memory read,   //
// and its first genuine stall. The address itself needs NO new logic: An's  //
// value is already what `operand_a` resolves to (decode set eac_src_reg to   //
// An's unified regfile index, see ap040_decode.v's header) -- the SAME       //
// priority mux, with the SAME EX-forward source, that every register-direct  //
// operand has used since milestone 2. What's new is turning that value into  //
// an actual ap040_pipe_l1.v port-B access:                                   //
//                                                                            //
//   mem_issue    (eac_valid && eac_is_mem_src && !mem_pending) -- the cycle  //
//                this stage first sees an unserviced memory instruction.     //
//                l1_addr_b is driven combinationally off operand_a THIS      //
//                cycle (ap040_pipe_l1.v registers l1_q_b from it, valid      //
//                next cycle); eaf_valid goes to 0 (nothing for EX yet); and  //
//                eaf_stall's mem_issue term freezes ap040_ea_calc.v's        //
//                OUTPUT for exactly this one cycle -- not a separate pending-//
//                instruction latch in THIS stage, just reusing the stall     //
//                chain's existing freeze-on-stall_in behavior (every stage    //
//                has had it since milestone 1). That's why eac_* is         //
//                guaranteed to still describe the SAME instruction next      //
//                cycle with no extra bookkeeping here.                       //
//   mem_complete (mem_pending, i.e. the cycle after issuing) -- eac_* is      //
//                still frozen on this same instruction, l1_q_b now holds the  //
//                fetched longword. Finishes exactly like the normal path      //
//                below, except eaf_operand_a comes from l1_q_b instead of     //
//                operand_a/regfile/forwarding. eaf_stall drops (mem_issue is  //
//                false once mem_pending is set), so ap040_ea_calc.v is free   //
//                to advance a NEW instruction into its output starting THIS   //
//                cycle -- the memory latency is hidden from whatever comes    //
//                next, not paid twice.                                       //
//                                                                            //
// L1 addressing: ap040_pipe_l1.v is addressed PC_RESET-relative (see          //
// ap040_inst_fetch.v's header) -- PC_RESET is the L1 window's BASE address,   //
// not "where execution starts" in isolation, and both ports must agree on    //
// it or the "unified" memory silently stops being unified. This stage takes  //
// the SAME PC_RESET parameter ap040_inst_fetch.v does (ap040_pipe_core.v     //
// passes the identical value to both) rather than assuming address 0.        //
//                                                                            //
// MOVE.L (d16,An),Dn (milestone 10, new): needed NO changes to the mem_issue/ //
// mem_complete FSM above -- ap040_decode.v now reuses id_imm to carry the      //
// sign-extended displacement (0 for the plain (An) case), and this stage's     //
// address computation became `operand_a + eac_imm` unconditionally (see        //
// where l1_addr_b is assigned below) rather than a per-mode branch. The         //
// gather that assembles the extension word happens entirely in                  //
// ap040_decode.v, same as Bcc.W/DBcc -- this stage never knows a gather           //
// happened, exactly the same "looks identical from EA-calc onward" property       //
// those two established.                                                          //
//                                                                            //
// flush: when ap040_execute.v detects a mispredicted branch, everything     //
// speculatively fetched behind it -- including whatever is sitting here -- //
// must be discarded. An outstanding L1 request (mem_pending) is simply       //
// abandoned, not unwound -- a read has no side effect, so its eventual        //
// l1_q_b is just never consumed once mem_pending clears.                     //
//                                                                            //
// JMP (An) / JMP (d16,An) (milestone 11, new): the EA (operand_a + eac_imm,   //
// the exact same expression l1_addr_b already computes) is what JMP needs     //
// as its RESULT, not an address to dereference -- eac_is_mem_src stays 0 for  //
// JMP (ap040_decode.v never sets it), so this stage's mem_issue/mem_complete   //
// FSM never triggers for it and JMP rides the plain, non-stalling "else"       //
// path below like any register-direct instruction. The only change there is    //
// eaf_operand_a: `eac_is_jmp ? (operand_a + eac_imm) : operand_a` -- routing     //
// the computed target into the SAME field ap040_execute.v already reads for      //
// its EX-forward tap and (via a new case there) ex_recovery_pc. See its           //
// header for why JMP is modeled as an unconditional misprediction rather than      //
// a new redirect mechanism.                                                        //
//                                                                            //
// BSR / JSR (milestone 13, new): this pipeline's first genuine memory WRITE,   //
// and its second kind of stall (mem_issue's read-side FSM has existed since     //
// milestone 9b; this is the write-side counterpart). Both push the return        //
// address (eac_next_pc -- already exactly right, no new arithmetic: it's the      //
// address of whatever comes after the whole BSR/JSR, extension words              //
// included, the same field id_next_pc has threaded since milestone 7) onto        //
// -(A7), then decrement A7 by 4. Both quantities the push needs -- the WRITE       //
// ADDRESS (A7-4) and the NEW A7 VALUE to commit -- are the SAME expression,         //
// and ap040_decode.v set eac_dest_reg to A7's unified index (4'd15) for both        //
// instructions specifically so operand_b (port B, "the destination register's       //
// current value", already resolved with full EX-forwarding) gives us A7's            //
// CURRENT value for free -- no new regfile read, no new forwarding mux. JSR's         //
// own EA target (An + eac_imm, via operand_a, exactly like JMP) and BSR/JSR's          //
// push address (operand_b - 4) are simply two DIFFERENT fields, resolved from           //
// two DIFFERENT ports, with no conflict.                                                //
//                                                                            //
//   l1_addr_b (extended): now selects between ea_target (a memory-source read     //
//   or JMP/JSR's redirect target) and push_addr (operand_b - 4) depending on        //
//   which this cycle's instruction actually is -- mutually exclusive by              //
//   construction, an instruction is never more than one of these at once.             //
//   l1_wren_b/l1_data_b: asserted whenever eac_is_bsr||eac_is_jsr, driving the          //
//   SAME push_addr and eac_next_pc as the data -- held stable (not gated by any          //
//   "have we issued yet" latch) for as long as eac_* itself stays stable, which           //
//   the stall below guarantees.                                                            //
//   wr_stall (`eac_valid && (eac_is_bsr||eac_is_jsr) && l1_wr_busy`): unlike a               //
//   read, a write needs no "wait for the response" phase -- ap040_pipe_l1.v's                //
//   OWN contract (see its header) is that asserting wren_b while wr_busy is LOW               //
//   always succeeds THIS cycle, so the only thing to wait for is the port being                //
//   free at all. When wr_stall is false (either this isn't a push, or it is and                 //
//   l1_wr_busy just read low), the push -- if any -- is being accepted THIS SAME                 //
//   cycle, and the instruction proceeds to EX in that SAME cycle carrying                         //
//   eaf_operand_b = operand_b - 4 as its result (ap040_execute.v routes this into                  //
//   the regfile commit for A7, exactly the same "dedicated combinational result,                    //
//   not the generic ALU" pattern DBcc's decrement and Scc's byte-merge already use;                  //
//   see its header for why the generic ALU wasn't used here either -- eaf_operand_a                   //
//   is busy carrying JSR's redirect target and can't also carry a "4" operand).                        //
//                                                                            //
// TRAP #n / illegal instruction (milestone 14, new): this pipeline's first     //
// exception entry, and its first multi-beat memory sequence -- BSR/JSR's push    //
// was exactly one write; a format-$0 frame is TWO (SR:PC_hi, then PC_lo:         //
// FmtVec), followed by a THIRD access, a plain read, to fetch the handler          //
// address out of the vector table. A small state register (exc_ph, BEAT0/          //
// BEAT1/VECRD) sequences these three L1 accesses one at a time; eac_* stays          //
// frozen throughout via exc_stall feeding eaf_stall, the SAME freeze-on-stall         //
// mechanism every multi-cycle operation in this stage already uses (mem_issue/        //
// wr_stall above). Each beat reuses wr_stall's own accept-when-!l1_wr_busy idiom       //
// independently (retried every cycle it's needed, exactly like BSR/JSR's single        //
// write already does) rather than a new handshake shape.                                //
//                                                                            //
// Frame contents (format $0 only this milestone -- see ap040_decode.v's header    //
// for why address-error's format $2 is deferred to its own milestone):             //
//   SR word:  this pipeline has no real T1/T0/M/IPL state (sr_s/sr_m are            //
//             hardwired in ap040_pipe_core.v), so the system byte is a fixed        //
//             8'b0010_0000 (S=1, everything else 0, matching AP040_SR_RESET's        //
//             S=1/M=0) -- ccr_in supplies the real, live low byte (this stage's       //
//             CCR forwarding source, same signal ap040_execute.v's Bcc/Scc            //
//             condition check already uses -- see ap040_pipe_core.v).                  //
//   PC field: id_pc (the faulting instruction's OWN address) for illegal --             //
//             you can't "return past" an illegal opcode -- id_next_pc (the               //
//             FOLLOWING instruction, TRAP's actual return address) for TRAP,              //
//             the same subroutine-call semantics BSR/JSR's return-address push             //
//             already established. Verified against ap040_core.v's go_illegal              //
//             (spc=pc_i, its own convention for "current instruction's address")             //
//             vs. its vector-32+n TRAP dispatch (spc=pc, its convention for "the              //
//             next instruction" -- see that file for the pc/pc_i distinction).                //
//   Format/vector-offset word: {format(4)=0, 00, vector(8), 00} -- vector*4 falls               //
//             naturally out of this bit placement, no explicit shift needed.                     //
//                                                                            //
// Vector fetch: this substrate has NO real VBR register consulted here (see              //
// milestone 15 below -- one now EXISTS in ap040_pipe_core.v, but this pipeline's           //
// only two implemented vectors are both low, fixed numbers, so nothing here has             //
// needed to add it in yet) and, unlike a real 68040, no separate low-memory region           //
// distinct from ap040_pipe_l1.v's PC_RESET-relative window -- vector*4 is used                //
// as an ABSOLUTE address, fed through the exact SAME `l1_addr_word - PC_RESET) >>              //
// 1` conversion every other address on this port already goes through. This is                 //
// not a special case: PC_RESET's own value (0x400 in every testbench so far,                    //
// deliberately -- exactly the byte size of a full 256-entry 68k vector table) was                 //
// already chosen with headroom for the vector table to occupy the address range                    //
// BELOW it, and the unsigned wraparound in that subtraction lands vector*4 in the                    //
// mathematically-correct modular slot of the SAME L1 array with zero new address                     //
// logic -- confirmed arithmetically, not assumed, before relying on it.                                //
//                                                                                          //
// MOVE to SR / MOVEC / privilege violation (milestone 15, new): the first DYNAMIC          //
// exception trigger. Illegal/TRAP are decode-time facts; whether a privileged             //
// opcode actually FAULTS depends on the LIVE S bit (sr_in[13]) -- eac_is_priv is           //
// computed right here, the earliest point that value exists, and folds straight            //
// into eac_is_exc/exc_pc_field/exc_vec_num alongside illegal/TRAP (own address,            //
// vector 8, format $0 -- go_priv's exact convention). The frame push itself uses            //
// isp_in/msp_in DIRECTLY, never port B/A7 (see exc_sp_bank below for why: a fault             //
// taken WHILE ALREADY IN USER MODE must still land its frame on a SUPERVISOR                  //
// stack, not USP) -- which also means MOVEC's READ direction pointing port B at                //
// its intended GPR destination for the NORMAL case is fine as-is, no rerouting                  //
// needed there at all. eaf_dest_reg IS still overridden on the COMMIT side once                  //
// exc_vec_done finalizes (A7, not Rn, is what the exception's OWN result writes),                 //
// since that's a genuinely different register than whatever the now-suppressed                    //
// original instruction intended. MOVE-to-SR and MOVEC's WRITE direction never                      //
// needed ANY of this: decode points their eac_dest_reg at A7 unconditionally (same                  //
// "dummy anchor" trick BSR uses) purely so raddr_b's default read is harmless, not                   //
// because anything downstream actually depends on it. Their SR/control-register                      //
// write itself (when NOT faulting) is entirely ap040_execute.v's job -- this stage                     //
// just threads eaf_is_movesr/eaf_is_movec/eaf_movec_dir/eaf_movec_sel through                           //
// unchanged, the same mechanical pass-through every other classification flag gets.                      //
//                                                                                          //
// RTS / RTE (milestone 16, new): RTS is deliberately NOT a new mechanism -- it              //
// reuses mem_issue/mem_complete VERBATIM (ap040_decode.v sets eac_is_mem_src for            //
// it, same as MOVE.L (An),Dn), since a pop is structurally just a 32-bit read from            //
// (A7) with the register hardcoded instead of decoded from opcode bits. The ONLY              //
// addition is mem_complete's eaf_operand_b, which now also has to carry A7's NEW               //
// (post-pop) value forward for the commit -- one ternary, not a new FSM. RTE is                 //
// genuinely new: a privilege-violating RTE (checked exactly like MOVE-to-SR/MOVEC,                //
// via eac_is_priv -- see above) never reaches its own machinery at all, redirected                 //
// into the SAME exception-entry sequencer instead; a supervisor RTE gets its own 2-                 //
// beat READ sequencer (ret_ph/ret_pending/ret_dword0), the mirror image of the                       //
// exception-entry sequencer's own WRITE beats, reading back the exact two dwords a                    //
// format-$0 push wrote. Format $0 is assumed unconditionally once the pop completes --                 //
// this pipeline has no mechanism that could ever have pushed anything else, so a real                   //
// FMTERR fallback for an unrecognized frame format is deliberately deferred, not                         //
// forgotten -- see AP040_IMPLEMENTATION_PLAN.md.                                                          //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_ea_fetch
#(
	parameter [31:0] PC_RESET = 32'h0000_0400,   // must match ap040_inst_fetch.v's -- see header
	parameter         L1_AW    = 12               // must match the ap040_pipe_l1.v instance's AW
)
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // EX cannot accept this cycle
	input             flush,      // EX detected a misprediction: force a bubble

	input             eac_valid,
	input      [31:0] eac_pc,
	input      [31:0] eac_next_pc,
	input       [3:0] eac_dest_reg,
	input       [3:0] eac_src_reg,
	input      [31:0] eac_imm,
	input       [5:0] eac_alu_op,
	input             eac_src_a_is_imm,
	input             eac_writes_reg,
	input             eac_writes_ccr,
	input             eac_is_branch,
	input             eac_is_scc,
	input             eac_is_dbcc,
	input             eac_is_mem_src,
	input             eac_is_jmp,
	input             eac_is_bsr,
	input             eac_is_jsr,
	input             eac_is_trap,
	input             eac_is_illegal,
	input             eac_is_movesr,
	input             eac_is_movec,
	input             eac_is_rts,
	input             eac_is_rte,
	input       [3:0] eac_cond,

	// Architectural SR (milestone 15: widened from a 5-bit CCR-only port to
	// the full 16-bit register, write-through forwarded -- see
	// ap040_pipe_core.v's sr_resolved). The exception frame's SR word is now
	// simply THIS value, no synthesis -- and sr_in[13] (S) is what a
	// privilege check actually reads -- see header.
	input      [15:0] sr_in,

	// ap040_pipe_regfile.v's OWN ISP/MSP state, read directly -- an
	// exception's stack access must always target one of these, never
	// whatever bank port B/A7 is currently reading -- see exc_sp_bank above.
	input      [31:0] isp_in,
	input      [31:0] msp_in,

	// regfile operand read ports (driven combinationally by this stage;
	// ap040_pipe_core.v wires raddr_a/b <-> rdata_a/b straight to the
	// ap040_pipe_regfile.v instance)
	output     [3:0]  raddr_a,
	input      [31:0] rdata_a,
	output     [3:0]  raddr_b,
	input      [31:0] rdata_b,

	// EX-forward source, see header comment
	input             ex_fwd_valid,
	input       [3:0] ex_fwd_dest,
	input      [31:0] ex_fwd_data,

	// ap040_pipe_l1.v port B -- read for a memory-source instruction or
	// JMP/JSR's redirect target; write for BSR/JSR's push -- see header.
	output [L1_AW-1:0] l1_addr_b,
	input        [31:0] l1_q_b,
	output              l1_wren_b,
	output       [31:0] l1_data_b,
	input               l1_wr_busy,

	output            eaf_stall,  // to EA-calc

	output reg        eaf_valid,
	output reg [31:0] eaf_pc,
	output reg [31:0] eaf_next_pc,
	output reg  [3:0] eaf_dest_reg,
	output reg [31:0] eaf_operand_a,
	output reg [31:0] eaf_operand_b,
	output reg  [5:0] eaf_alu_op,
	output reg        eaf_writes_reg,
	output reg        eaf_writes_ccr,
	output reg        eaf_is_branch,
	output reg        eaf_is_scc,
	output reg        eaf_is_dbcc,
	output reg        eaf_is_jmp,
	output reg        eaf_is_bsr,
	output reg        eaf_is_jsr,
	output reg        eaf_is_trap,
	output reg        eaf_is_illegal,
	output reg        eaf_is_priv,
	output reg        eaf_is_addrerr,
	output reg        eaf_is_movesr,
	output reg        eaf_is_movec,
	output reg        eaf_movec_dir,
	output reg  [2:0] eaf_movec_sel,
	// The live SR AS OF THIS STAGE'S OWN cycle (already correctly EX-
	// forwarded/write-through resolved via sr_in) -- threaded straight
	// down to ap040_execute.v for its exception-entry SR-masking
	// arithmetic, rather than having it read sr_in live a second time one
	// cycle later, which would close a combinational loop through its own
	// EX-forward output -- see its header.
	output reg [15:0] eaf_sr_snapshot,
	output reg        eaf_is_rts,
	output reg        eaf_is_rte,
	// RTE's popped SR (masked, format-$0-frame's word0 high half) --
	// ap040_execute.v's new commit source for restoring it, same shape as
	// eaf_sr_snapshot above but this one's a REAL architectural value
	// being adopted, not just a forwarded read -- see ap040_execute.v's
	// header for why it needs its own field rather than reusing
	// eaf_operand_a/b (both already busy carrying the redirect target and
	// the new A7 value).
	output reg [15:0] eaf_rte_sr_data,
	output reg  [3:0] eaf_cond
);

reg mem_pending;   // an L1 port-B request is in flight for the CURRENT eac_*
                    // instruction; its result becomes valid next cycle -- see
                    // header.

// Moved ahead of eac_is_push/the exception-detection wires below (both now
// need ea_target, milestone 17) -- iverilog requires a wire's declaration
// to textually precede any use of it in another continuous assignment,
// even though nothing here structurally depends on file order otherwise.
wire fwd_a_from_ex = ex_fwd_valid && (ex_fwd_dest == eac_src_reg);

// Flat 3-way select on port A: immediate, else forwarded, else the
// regfile's own (possibly write-through-bypassed) read. eac_src_a_is_imm
// and fwd_a_from_ex are mutually exclusive in practice (MOVEQ never has a
// meaningful eac_src_reg), so this is one priority mux, not two stacked.
// For a memory-source instruction this IS the effective address (An's
// value, decode having set eac_src_reg to An's unified index) -- see header.
wire [31:0] operand_a = eac_src_a_is_imm ? eac_imm :
                         fwd_a_from_ex   ? ex_fwd_data :
                                           rdata_a;

// The effective address: operand_a (An's value, resolved by the mux above)
// PLUS eac_imm (the sign-extended displacement for a (d16,An) form, or
// exactly 0 for the plain (An) form -- see ap040_decode.v's header for why
// id_imm is zeroed for the plain case). One formula for both EA modes and
// both consumers (a memory-source MOVE dereferences it below; JMP uses it
// directly as its result) -- adding a future EA mode that also produces an
// address+offset shape (indexed, etc.) needs no new logic here, just
// decode emitting the right eac_imm.
wire [31:0] ea_target = operand_a + eac_imm;

// Address error on an odd JMP/JSR target (milestone 17, new): a SECOND
// dynamic exception trigger, same reasoning as eac_is_priv below -- "this
// is a JMP/JSR" is a decode-time fact, but "the target happens to be odd"
// depends on ea_target (above), not known until here either. Format $2
// (the 6-word frame, one extra "instruction address" longword), not
// format $0 -- see exc_pc_field/exc_addr_field below for the real,
// mode-dependent bit-exact stacked-PC quirks this needed, verified
// against ap040_core.v's own S_JMP1/S_JSR1, not guessed.
wire eac_is_jmp_odd  = eac_is_jmp && ea_target[0];
wire eac_is_jsr_odd  = eac_is_jsr && ea_target[0];
wire eac_is_addrerr  = eac_is_jmp_odd || eac_is_jsr_odd;
wire eac_is_fmt2     = eac_is_addrerr;   // the only format-$2 source so far

wire mem_issue    = eac_valid && eac_is_mem_src && !mem_pending;
wire mem_complete = mem_pending;
// BSR/JSR's push -- no "pending" latch needed, see header: a write either
// succeeds immediately (l1_wr_busy low) or must wait for the port, but
// never needs a separate multi-cycle completion phase the way a read does.
// eac_is_jsr_odd (milestone 17) is excluded here: an odd JSR target never
// pushes at all, per ap040_core.v's own S_JSR1 -- it goes straight to the
// address-error exception instead (below), the fault taken on the
// INSTRUCTION FETCH at the odd target, not on the call itself.
wire eac_is_push  = eac_is_bsr || (eac_is_jsr && !eac_is_jsr_odd);
wire wr_stall     = eac_valid && eac_is_push && l1_wr_busy;

// TRAP #n / illegal instruction exception entry -- see header. exc_ph
// sequences the frame's writes and the vector-table read one at a time;
// exc_vec_pending mirrors mem_pending's own shape for the read's one-cycle
// latency. EXC_BEAT2 (milestone 17, new) is a THIRD write beat, used only
// for format $2's extra "instruction address" longword -- see
// eac_is_fmt2/exc_writing below for why format $0 exceptions skip it
// entirely rather than visiting an empty state.
localparam EXC_BEAT0 = 2'd0, EXC_BEAT1 = 2'd1, EXC_BEAT2 = 2'd2, EXC_VECRD = 2'd3;
reg [1:0] exc_ph;
reg       exc_vec_pending;

// Privilege violation (milestone 15, new): the ONLY dynamic exception
// trigger in this pipeline -- illegal/TRAP are static decode-time facts,
// but "is this opcode privileged" (MOVE to SR, MOVEC) says nothing about
// whether it actually FAULTS; that depends on the LIVE, forwarded S bit,
// which doesn't exist until here. eac_is_priv_capable is what
// ap040_decode.v recognized; eac_is_priv is whether it actually fires THIS
// cycle -- see header for why the check couldn't happen any earlier.
wire eac_is_priv_capable = eac_is_movesr || eac_is_movec || eac_is_rte;
wire eac_is_priv         = eac_is_priv_capable && !sr_in[13];

wire eac_is_exc    = eac_is_trap || eac_is_illegal || eac_is_priv || eac_is_addrerr;
wire exc_active    = eac_valid && eac_is_exc;
wire exc_writing   = exc_active && !exc_vec_pending &&
                      (exc_ph == EXC_BEAT0 || exc_ph == EXC_BEAT1 ||
                       (exc_ph == EXC_BEAT2 && eac_is_fmt2));
wire exc_beat_ack  = exc_writing && !l1_wr_busy;    // this beat accepted THIS cycle
wire exc_vec_issue = exc_active && !exc_vec_pending && (exc_ph == EXC_VECRD);
wire exc_vec_done  = exc_active && exc_vec_pending;
wire exc_stall     = exc_active && !exc_vec_done;

// RTE (milestone 16, new): a genuinely supervisor RTE (eac_is_priv already
// false means sr_in[13] was 1) gets its own 2-beat READ sequencer -- the
// mirror image of the exception-entry sequencer's own WRITE beats above,
// reading back the SAME two dwords a format-$0 frame push wrote: dword0 @
// A7 = {SR, PC_hi}, dword1 @ A7+4 = {PC_lo, FmtVec}. ret_dword0 stashes
// the first read's result while the second is in flight (this stage's
// only two "carry a value forward" registers, eaf_operand_a/b, are both
// needed for the FINAL result -- the redirect target and the new A7 --
// not an intermediate one). A privilege-violating RTE never reaches this
// at all: eac_is_priv already took priority via eac_is_exc above, so
// eac_is_rte_active can safely assume supervisor.
//
// Format $0 is assumed UNCONDITIONALLY once the pop completes -- this
// pipeline has no mechanism that could ever have pushed anything else
// (format $2/$3/$4/$7 don't exist here yet), so a real FMTERR fallback
// for a stack frame this core didn't push itself is deliberately
// deferred, not overlooked -- see header.
wire eac_is_rte_active = eac_is_rte && !eac_is_priv;

localparam RET_BEAT0 = 1'd0, RET_BEAT1 = 1'd1;
reg        ret_ph;
reg        ret_pending;   // this beat's read is in flight; l1_q_b valid NEXT cycle -- same shape as mem_pending
reg [31:0] ret_dword0;    // captured {SR, PC_hi} after beat 0 completes

wire ret_active   = eac_valid && eac_is_rte_active;
wire ret_issue    = ret_active && !ret_pending;
wire ret_complete = ret_active && ret_pending;
wire ret_done     = ret_complete && (ret_ph == RET_BEAT1);
wire ret_stall    = ret_active && !ret_done;

assign eaf_stall = stall_in || mem_issue || wr_stall || exc_stall || ret_stall;
assign raddr_a    = eac_src_reg;
// A privilege violation reroutes port B to A7 REGARDLESS of what the
// faulting instruction's own eac_dest_reg says (MOVEC's read direction
// points it at the destination GPR, Rn, for its NORMAL case) -- but NOT via
// port B/operand_b at all (see exc_new_sp below for why: an exception's OWN
// stack access must never go through the CURRENTLY active bank, which could
// be USP). raddr_b stays the plain, unconditional eac_dest_reg -- MOVEC's
// read direction doesn't actually need port B for its result either (that
// comes from creg_read_value in ap040_execute.v, entirely bypassing this
// port), so there is no live conflict left to resolve here at all.
assign raddr_b    = eac_dest_reg;

wire fwd_b_from_ex = ex_fwd_valid && (ex_fwd_dest == eac_dest_reg);

// Port B has no immediate case -- it's always "the destination register's
// current value" -- so it's a flat 2-way select.
wire [31:0] operand_b = fwd_b_from_ex ? ex_fwd_data : rdata_b;

// BSR/JSR's push address AND the new A7 value to commit are the SAME
// expression, from operand_b (A7's current value via port B, decode having
// set eac_dest_reg to A7's unified index for both -- see header).
wire [31:0] push_addr = operand_b - 32'd4;

// TRAP #n / illegal instruction / privilege violation: A7's NEW value after
// a format-$0 (8-byte) frame -- but NOT computed from operand_b/port B,
// unlike BSR/JSR's push_addr above. An exception's own stack access must
// ALWAYS target a SUPERVISOR stack (ISP, or MSP if M=1 -- never USP),
// regardless of which bank was active when the fault occurred: a privilege
// violation taken WHILE ALREADY IN USER MODE (S=0) would otherwise read
// port B's A7 as USP and silently push its own exception frame onto the
// user stack -- confirmed wrong against ap040_core.v's own S_EXC0/S_EXC1
// ordering (`sr[13]<=1` commits a FULL CYCLE before `dbg_a7` -- itself
// bank-selected off the now-already-supervisor sr -- is read for the
// stack pointer), not assumed. isp_in/msp_in are ap040_pipe_regfile.v's
// OWN state, read directly (same "fed straight from its real home"
// precedent ap040_execute.v's MOVEC-read inputs already established),
// completely bypassing whatever bank sr_s/sr_m currently have port B
// pointed at. Recomputed live every cycle of the sequence rather than
// latched once: eac_*/sr_in are frozen by exc_stall the whole time anyway
// (nothing downstream can change them), so a latch would just be a
// redundant copy.
//
// exc_sr_word (simplified, milestone 15): no more synthesis -- sr_in IS a
// real, live SR now (was a fixed system-byte constant over just CCR before
// this milestone), so the pushed word is simply the value being saved,
// exactly as architecturally required.
//
// exc_frame_size (milestone 17, new): format $2 is 12 bytes (6 words),
// format $0 is 8 (4 words) -- the ONLY difference in overall frame shape
// this milestone introduces; everything else about the sequencer (which
// stack, how M/S select it) is unchanged.
wire [31:0] exc_sp_bank    = sr_in[12] ? msp_in : isp_in;   // M selects ISP vs MSP; S is irrelevant here
wire [31:0] exc_frame_size = eac_is_fmt2 ? 32'd12 : 32'd8;
wire [31:0] exc_new_sp     = exc_sp_bank - exc_frame_size;
wire [15:0] exc_sr_word    = sr_in;
// Illegal and privilege violation both stack the FAULTING instruction's OWN
// address (go_illegal's/go_priv's shared pc_i convention -- you can't
// "return past" either kind of fault); TRAP stacks the FOLLOWING
// instruction's (its return address, a subroutine call).
//
// Odd JMP/JSR target (milestone 17, new): NEITHER of the above two
// conventions -- verified against ap040_core.v's own S_JMP1/S_JSR1, not
// guessed, and the two are genuinely DIFFERENT from each other:
//   JMP:  pc_i+2 -- gencpu's i_JMP already did incpc(2) before the fault
//         is even detected, regardless of how many extension words this
//         JMP gathered; this decoder's own id_next_pc (held_pc+2+width)
//         would be WRONG here for the (d16,An) form specifically, since
//         real hardware's "+2" reflects the OPCODE WORD alone, not the
//         whole gathered instruction -- eac_pc+2 is used directly instead.
//         (The v24 AE corpus's OTHER value, pc_i+6 for indexed/absolute-
//         long modes, doesn't apply: this decoder has neither mode.)
//   JSR:  ea_target ITSELF, not any PC-relative value at all -- unlike
//         JMP, a 68040 takes an odd JSR's fault on the INSTRUCTION FETCH
//         at the odd target, so the frame names THAT target, not the
//         call site. This is also why JSR's push never happens for this
//         case (see eac_is_push above) -- there is no return address to
//         protect if the call itself never completes.
wire [31:0] exc_pc_field   = eac_is_jmp_odd ? (eac_pc + 32'd2) :
                              eac_is_jsr_odd ? ea_target :
                              (eac_is_illegal || eac_is_priv) ? eac_pc : eac_next_pc;
wire  [7:0] exc_vec_num    = eac_is_illegal ? 8'd4 : eac_is_priv ? 8'd8 :
                              eac_is_addrerr ? 8'd3 : eac_imm[7:0];
wire [15:0] exc_vecoff_word = {eac_is_fmt2 ? 4'd2 : 4'd0, 2'b00, exc_vec_num, 2'b00};
// Format $2's own extra "instruction address" longword -- the odd target
// itself, LSB cleared (ap040_core.v's own convention for this field,
// identical for both JMP and JSR despite their differing PC fields above).
wire [31:0] exc_addr_field = {ea_target[31:1], 1'b0};

// Beat0 @ exc_new_sp: SR, then PC's high word. Beat1 @ exc_new_sp+4: PC's
// low word, then the format/vector-offset word. Beat2 @ exc_new_sp+8
// (format $2 only): the extra address field -- five/six 16-bit frame
// words packed into two or three 32-bit L1 writes, see header.
wire [31:0] exc_beat_addr = (exc_ph == EXC_BEAT2) ? (exc_new_sp + 32'd8) :
                             (exc_ph == EXC_BEAT1) ? (exc_new_sp + 32'd4) : exc_new_sp;
wire [31:0] exc_wdata     = (exc_ph == EXC_BEAT0) ? {exc_sr_word, exc_pc_field[31:16]} :
                             (exc_ph == EXC_BEAT1) ? {exc_pc_field[15:0], exc_vecoff_word} :
                                                       exc_addr_field;   // EXC_BEAT2

// Vector table address: vector*4, used as an ABSOLUTE address fed through
// the SAME PC_RESET-relative conversion below -- see header for why no
// special-casing (a real VBR, a separate low-memory region) is needed.
wire [31:0] exc_vec_addr = {22'd0, exc_vec_num, 2'b00};

// RTE's own two read-beat addresses: A7 (dword0), A7+4 (dword1) -- via
// operand_a/port A, same as RTS's mem_issue/mem_complete reuse (decode set
// eac_src_reg=A7 for RTE too, see ap040_decode.v's header).
wire [31:0] ret_addr = (ret_ph == RET_BEAT1) ? (operand_a + 32'd4) : operand_a;

// Driven unconditionally, same "compute always, gate consumption" precedent
// as raddr_b -- harmless when none of eac_is_mem_src/eac_is_jmp/eac_is_push/
// eac_is_exc/eac_is_rte_active is set, nothing reads l1_q_b or l1_wr_busy
// that cycle. PC_RESET-relative to match ap040_inst_fetch.v's own L1
// addressing -- see header. Five-way select: a READ target (memory-source,
// JMP/JSR's redirect, or now RTS via the same mem_issue/mem_complete path),
// the BSR/JSR PUSH address, an exception frame WRITE beat, the exception's
// own vector-table READ, or RTE's own pop READ -- mutually exclusive by
// construction (an instruction is never more than one of these at once).
wire [31:0] l1_addr_word = eac_is_push  ? push_addr :
                            exc_writing ? exc_beat_addr :
                            (exc_vec_issue || exc_vec_pending) ? exc_vec_addr :
                            ret_active  ? ret_addr :
                                                                  ea_target;
assign l1_addr_b = (l1_addr_word - PC_RESET) >> 1;
assign l1_wren_b = (eac_valid && eac_is_push) || exc_writing;
assign l1_data_b = exc_writing ? exc_wdata : eac_next_pc;

always @(posedge clk) begin
	if (!nreset) begin
		eaf_valid      <= 1'b0;
		eaf_pc         <= 32'h0;
		eaf_next_pc    <= 32'h0;
		eaf_dest_reg   <= 4'h0;
		eaf_operand_a  <= 32'h0;
		eaf_operand_b  <= 32'h0;
		eaf_alu_op     <= 6'h0;
		eaf_writes_reg <= 1'b0;
		eaf_writes_ccr <= 1'b0;
		eaf_is_branch  <= 1'b0;
		eaf_is_scc     <= 1'b0;
		eaf_is_dbcc    <= 1'b0;
		eaf_is_jmp     <= 1'b0;
		eaf_is_bsr     <= 1'b0;
		eaf_is_jsr     <= 1'b0;
		eaf_is_trap    <= 1'b0;
		eaf_is_illegal <= 1'b0;
		eaf_is_priv    <= 1'b0;
		eaf_is_addrerr <= 1'b0;
		eaf_is_movesr  <= 1'b0;
		eaf_is_movec   <= 1'b0;
		eaf_movec_dir  <= 1'b0;
		eaf_movec_sel  <= 3'h0;
		eaf_sr_snapshot<= 16'h0;
		eaf_is_rts     <= 1'b0;
		eaf_is_rte     <= 1'b0;
		eaf_rte_sr_data<= 16'h0;
		eaf_cond       <= 4'h0;
		mem_pending    <= 1'b0;
		exc_ph          <= EXC_BEAT0;
		exc_vec_pending <= 1'b0;
		ret_ph          <= RET_BEAT0;
		ret_pending     <= 1'b0;
	end else if (ce) begin
		if (flush) begin
			eaf_valid       <= 1'b0;
			mem_pending     <= 1'b0;
			// Abandon a mid-flight exception sequence the same way an
			// abandoned mem_pending read is: nothing downstream of a flush
			// consumes what was in progress, but exc_ph/exc_vec_pending
			// MUST reset here too, or the next (unrelated) exception
			// instruction would resume mid-sequence instead of starting at
			// beat0. ret_ph/ret_pending need the same treatment for a
			// mid-flight RTE.
			exc_ph          <= EXC_BEAT0;
			exc_vec_pending <= 1'b0;
			ret_ph          <= RET_BEAT0;
			ret_pending     <= 1'b0;
		end else if (!stall_in) begin
			if (mem_issue) begin
				eaf_valid   <= 1'b0;
				mem_pending <= 1'b1;
			end else if (mem_complete) begin
				eaf_valid      <= eac_valid;
				eaf_pc         <= eac_pc;
				eaf_next_pc    <= eac_next_pc;
				eaf_dest_reg   <= eac_dest_reg;
				eaf_operand_a  <= l1_q_b;
				// RTS: the popped value (l1_q_b, into eaf_operand_a above)
				// is the redirect target, exactly like JMP/JSR/exceptions
				// already route through eaf_operand_a -- but this stage
				// ALSO owes A7 its post-pop value (old+4), which is NOT
				// what a plain MOVE.L (An),Dn would put in eaf_operand_b
				// (that instruction's operand_b is simply unused). See
				// header.
				eaf_operand_b  <= eac_is_rts ? (operand_a + 32'd4) : operand_b;
				eaf_alu_op     <= eac_alu_op;
				eaf_writes_reg <= eac_writes_reg;
				eaf_writes_ccr <= eac_writes_ccr;
				eaf_is_branch  <= eac_is_branch;
				eaf_is_scc     <= eac_is_scc;
				eaf_is_dbcc    <= eac_is_dbcc;
				eaf_is_jmp     <= 1'b0;   // a mem-source instruction is never also a JMP/BSR/JSR/TRAP/illegal/priv/movesr/movec
				eaf_is_bsr     <= 1'b0;
				eaf_is_jsr     <= 1'b0;
				eaf_is_trap    <= 1'b0;
				eaf_is_illegal <= 1'b0;
				eaf_is_priv    <= 1'b0;
				eaf_is_addrerr <= 1'b0;
				eaf_is_movesr  <= 1'b0;
				eaf_is_movec   <= 1'b0;
				eaf_is_rts     <= eac_is_rts;
				eaf_is_rte     <= 1'b0;
				eaf_sr_snapshot<= sr_in;
				eaf_cond       <= eac_cond;
				mem_pending    <= 1'b0;
			end else if (wr_stall) begin
				// Waiting for l1_wr_busy to clear -- see header. eac_* stays
				// frozen (eaf_stall propagates backward), so this re-evaluates
				// identically next cycle with the SAME push request still
				// asserted, until the port is free.
				eaf_valid <= 1'b0;
			end else if (exc_writing) begin
				// Posting one beat of the exception frame -- see header.
				// exc_beat_ack means l1_wr_busy read low THIS cycle, so the
				// combinational wren_b above is being accepted right now;
				// advance to the next beat. Otherwise the port's still
				// busy: hold exc_ph, retry the SAME beat next cycle (eac_*
				// stays frozen via exc_stall, exactly like wr_stall above).
				// After beat 1, format $2 needs one more beat (EXC_BEAT2,
				// the extra address field); format $0 skips straight to
				// the vector-table read -- exc_writing's own gating above
				// already ensures EXC_BEAT2 is never ENTERED for a format
				// $0 exception, so no separate check is needed once we're
				// actually leaving beat 2.
				eaf_valid <= 1'b0;
				if (exc_beat_ack) begin
					case (exc_ph)
						EXC_BEAT0: exc_ph <= EXC_BEAT1;
						EXC_BEAT1: exc_ph <= eac_is_fmt2 ? EXC_BEAT2 : EXC_VECRD;
						default:   exc_ph <= EXC_VECRD;   // EXC_BEAT2 done
					endcase
				end
			end else if (exc_vec_issue) begin
				// Both frame writes are posted; the vector-table address is
				// being driven THIS cycle (l1_addr_b, see the combinational
				// block above) -- l1_q_b registers its data by next cycle,
				// same one-cycle latency mem_issue/mem_complete already
				// relies on.
				eaf_valid       <= 1'b0;
				exc_vec_pending <= 1'b1;
			end else if (exc_vec_done) begin
				// l1_q_b now holds the handler address fetched last cycle.
				// eaf_operand_a carries it into ap040_execute.v's
				// ex_recovery_pc exactly like JMP/JSR's redirect target;
				// eaf_operand_b carries A7's new (post-push) value into the
				// regfile commit, exactly like BSR/JSR's decrement -- zero
				// new mux shapes in ap040_execute.v beyond adding these two
				// flags to existing OR-chains, see its header.
				eaf_valid      <= eac_valid;
				eaf_pc         <= eac_pc;
				eaf_next_pc    <= eac_next_pc;
				// eac_dest_reg is ALREADY A7 for illegal/TRAP/MOVE-to-SR/
				// MOVEC-write/JSR (decode's static "dummy anchor" trick,
				// same as BSR) -- the cases that genuinely need overriding
				// here are a privilege violation on MOVEC's READ direction
				// (eac_dest_reg is Rn, the instruction's own, now-
				// suppressed, real destination) and an odd JMP target
				// (JMP, unlike JSR, writes NO register at all normally, so
				// decode never pointed eac_dest_reg at A7 for it in the
				// first place): the exception's OWN result (the new
				// supervisor SP, see exc_sp_bank above) must commit to A7
				// in both cases, not whatever eac_dest_reg otherwise says.
				eaf_dest_reg   <= (eac_is_priv || eac_is_addrerr) ? 4'd15 : eac_dest_reg;
				eaf_operand_a  <= l1_q_b;
				eaf_operand_b  <= exc_new_sp;
				eaf_alu_op     <= eac_alu_op;
				// UNCONDITIONALLY 1, not forwarded from eac_writes_reg:
				// every exception entry writes A7 the new SP, full stop --
				// illegal/TRAP already had eac_writes_reg=1 for this exact
				// reason, but MOVE-to-SR/MOVEC-write have eac_writes_reg=0
				// in their OWN normal case (they don't write a GPR at all
				// normally -- see ap040_decode.v's header), which would
				// silently skip A7's commit on their privilege-violation
				// path if forwarded here.
				eaf_writes_reg <= 1'b1;
				eaf_writes_ccr <= eac_writes_ccr;
				eaf_is_branch  <= 1'b0;
				eaf_is_scc     <= 1'b0;
				eaf_is_dbcc    <= 1'b0;
				eaf_is_jmp     <= 1'b0;
				eaf_is_bsr     <= 1'b0;
				eaf_is_jsr     <= 1'b0;
				eaf_is_trap    <= eac_is_trap;
				eaf_is_illegal <= eac_is_illegal;
				eaf_is_priv    <= eac_is_priv;
				eaf_is_addrerr <= eac_is_addrerr;
				// The original (now-suppressed) instruction's own semantics
				// must not reach EX -- a MOVEC/MOVE-to-SR that just faulted
				// is NOT also still a MOVEC/MOVE-to-SR as far as
				// ap040_execute.v is concerned, exactly like a mem-source
				// instruction is never also a JMP above. Same for an odd
				// JMP/JSR target -- it is NOT also still a JMP/JSR (its
				// normal redirect never happened; the exception's own
				// target, from the vector table, is what eaf_operand_a
				// carries instead).
				eaf_is_movesr  <= 1'b0;
				eaf_is_movec   <= 1'b0;
				// An RTE that just privilege-violated is NOT also still an
				// RTE as far as ap040_execute.v is concerned -- same
				// reasoning as movesr/movec above; its own pop never
				// happened (ret_ph's sequencer never even started, gated
				// on eac_is_rte_active).
				eaf_is_rts     <= 1'b0;
				eaf_is_rte     <= 1'b0;
				// The value ap040_execute.v's exception-masking arithmetic
				// needs -- captured HERE (this stage's own already-forwarded
				// read), not re-read live one cycle later there, to avoid a
				// combinational loop through EX's own SR forward -- see its
				// header.
				eaf_sr_snapshot <= sr_in;
				eaf_cond       <= eac_cond;
				exc_ph          <= EXC_BEAT0;
				exc_vec_pending <= 1'b0;
			end else if (ret_issue) begin
				// Posting this beat's read address (ret_addr, driven
				// combinationally above) -- l1_q_b registers its data by
				// next cycle, same one-cycle latency mem_issue/mem_complete
				// and the exception's own vector-fetch already rely on.
				eaf_valid   <= 1'b0;
				ret_pending <= 1'b1;
			end else if (ret_complete && ret_ph == RET_BEAT0) begin
				// l1_q_b now holds dword0 ({SR, PC_hi}) -- stash it (this
				// stage's carry-forward registers, eaf_operand_a/b, are
				// both needed for the FINAL result below, not an
				// intermediate one), advance to beat 1.
				ret_dword0  <= l1_q_b;
				ret_ph      <= RET_BEAT1;
				ret_pending <= 1'b0;
				eaf_valid   <= 1'b0;
			end else if (ret_done) begin
				// l1_q_b now holds dword1 ({PC_lo, FmtVec}) -- finalize.
				// Format $0 assumed unconditionally (see header); the
				// format word itself (l1_q_b[15:0]) is read off the stack
				// but deliberately not consulted for behavior yet.
				eaf_valid       <= eac_valid;
				eaf_pc          <= eac_pc;
				eaf_next_pc     <= eac_next_pc;
				eaf_dest_reg    <= eac_dest_reg;   // already A7 -- unused for RTE's OWN write now, see below
				eaf_operand_a   <= {ret_dword0[15:0], l1_q_b[31:16]};   // popped PC -> redirect target
				eaf_operand_b   <= operand_a + 32'd8;                    // new A7 (format $0, 8-byte frame)
				eaf_alu_op      <= eac_alu_op;
				// NOT 1: RTE's A7 restore does NOT go through the normal
				// commit_reg/A7-bank path at all -- see ap040_execute.v's
				// header for the real race that forces this (RTE's own SR
				// restore commits the SAME cycle, and would otherwise bank
				// this very write through the NEW, just-restored S bit
				// instead of the OLD one). eaf_operand_b (the new A7 value,
				// still computed above) is instead consumed by
				// ap040_execute.v's exe_writes_creg path, writing directly
				// to ISP/MSP.
				eaf_writes_reg  <= 1'b0;
				eaf_writes_ccr  <= 1'b0;
				eaf_is_branch   <= 1'b0;
				eaf_is_scc      <= 1'b0;
				eaf_is_dbcc     <= 1'b0;
				eaf_is_jmp      <= 1'b0;
				eaf_is_bsr      <= 1'b0;
				eaf_is_jsr      <= 1'b0;
				eaf_is_trap     <= 1'b0;
				eaf_is_illegal  <= 1'b0;
				eaf_is_priv     <= 1'b0;
				eaf_is_addrerr  <= 1'b0;
				eaf_is_movesr   <= 1'b0;
				eaf_is_movec    <= 1'b0;
				eaf_is_rts      <= 1'b0;
				eaf_is_rte      <= 1'b1;
				// Popped SR, masked -- ap040_execute.v's new commit source
				// for restoring it (exe_writes_sr/exe_sr_data) -- see its
				// header for why this needs its own field rather than
				// reusing eaf_sr_snapshot (that one's a forwarded READ of
				// the CURRENT live SR, not the value being ADOPTED).
				eaf_rte_sr_data <= ret_dword0[31:16] & `AP040_SR_MASK;
				eaf_sr_snapshot <= sr_in;
				eaf_cond        <= eac_cond;
				ret_ph          <= RET_BEAT0;
				ret_pending     <= 1'b0;
			end else begin
				eaf_valid      <= eac_valid;
				eaf_pc         <= eac_pc;
				eaf_next_pc    <= eac_next_pc;
				eaf_dest_reg   <= eac_dest_reg;
				// JMP/JSR: route the computed EA itself, not the register
				// value alone -- see header. BSR has no source read at all
				// (operand_a is simply unused for it), so it falls through
				// to the same default every register-direct instruction
				// already used.
				eaf_operand_a  <= (eac_is_jmp || eac_is_jsr) ? ea_target : operand_a;
				// BSR/JSR: the NEW A7 value (== push_addr, the same
				// expression already used for the write address) -- see
				// header for why the decrement happens HERE, once, rather
				// than being recomputed in ap040_execute.v.
				eaf_operand_b  <= (eac_is_bsr || eac_is_jsr) ? push_addr : operand_b;
				eaf_alu_op     <= eac_alu_op;
				eaf_writes_reg <= eac_writes_reg;
				eaf_writes_ccr <= eac_writes_ccr;
				eaf_is_branch  <= eac_is_branch;
				eaf_is_scc     <= eac_is_scc;
				eaf_is_dbcc    <= eac_is_dbcc;
				eaf_is_jmp     <= eac_is_jmp;
				eaf_is_bsr     <= eac_is_bsr;
				eaf_is_jsr     <= eac_is_jsr;
				eaf_is_trap    <= eac_is_trap;
				eaf_is_illegal <= eac_is_illegal;
				// This branch is only reached for a MOVE-to-SR/MOVEC
				// instruction when eac_is_priv is FALSE this cycle (any
				// privilege-violating one routes into exc_writing/
				// exc_vec_issue/exc_vec_done above instead, via exc_active)
				// -- so threading them straight through here is exactly the
				// "genuinely not faulting" case, no additional gating
				// needed.
				eaf_is_priv    <= 1'b0;
				// Same reasoning for an odd JMP/JSR target: only a
				// genuinely EVEN target ever reaches this plain redirect
				// path (eac_is_addrerr routes the odd case into
				// exc_writing/exc_vec_issue/exc_vec_done above instead).
				eaf_is_addrerr <= 1'b0;
				eaf_is_movesr  <= eac_is_movesr;
				eaf_is_movec   <= eac_is_movec;
				eaf_movec_dir  <= eac_imm[3];
				eaf_movec_sel  <= eac_imm[2:0];
				// RTS/RTE never reach this branch (RTS routes through
				// mem_issue/mem_complete above, RTE through its own ret_ph
				// sequencer or the exception path) -- zeroed here purely for
				// the same "every field explicitly assigned in every branch"
				// discipline every other flag already follows.
				eaf_is_rts     <= 1'b0;
				eaf_is_rte     <= 1'b0;
				eaf_sr_snapshot<= sr_in;
				eaf_cond       <= eac_cond;
			end
		end
	end
end

endmodule
