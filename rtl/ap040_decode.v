//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 17: address error)       //
//                                                                          //
// ap040_decode.v - ID stage                                               //
//                                                                          //
// Recognizes NOP, MOVEQ, register-direct-to-register-direct MOVE.L,        //
// register-direct ADD.L Dn,Dm, register-direct Scc.B Dn, DBcc Dn,<label>,  //
// MOVE.L (An),Dn, MOVE.L (d16,An),Dn, JMP (An), JMP (d16,An), BSR (all      //
// three displacement widths), JSR (An), JSR (d16,An), TRAP #n, and the      //
// whole Bcc family including its 16-/32-bit displacement forms (BRA        //
// included, as its "always true" special case -- see below). Anything      //
// ELSE is now genuinely ILLEGAL (milestone 14, changed from prior          //
// milestones): what used to be a harmless bubble (id_writes_reg stays 0,   //
// nothing downstream ever saw id_unimpl) now raises vector 4 through the   //
// real exception-entry path -- see id_is_illegal below and                 //
// ap040_ea_fetch.v's header for the mechanism. A future milestone MAY      //
// split the 1010/1111-top-nibble opcode classes into their own A-line/     //
// F-line vectors (10/11) instead of folding them into ILLEGAL -- not done  //
// here because nothing in this decoder's current scope actually produces   //
// those top nibbles, so there is no test coverage to verify the split      //
// against; documented rather than silently wrong.                          //
//                                                                          //
// TRAP #n (milestone 14, new): 0100 1110 0100 nnnn (0x4E40-0x4E4F, n =     //
// vector-32 in the low 4 bits). Single word, no gather, no EA, no operand   //
// read at all -- the only thing decode contributes beyond recognizing the   //
// opcode is computing the actual vector NUMBER (32+n) into id_imm, reused    //
// exactly the way BSR/JMP/JSR already reuse it for a displacement (a          //
// "general-purpose field", per MOVE.L (d16,An),Dn's header note) rather        //
// than adding a dedicated port for one 8-bit value. Like BSR/JSR, decode        //
// points id_dest_reg at A7 (4'd15) and sets id_writes_reg -- TRAP pushes an      //
// exception frame and decrements A7 exactly like a subroutine call, just         //
// with the target and frame contents supplied by ap040_ea_fetch.v's new          //
// exception-entry sequencer instead of a literal branch displacement.             //
//                                                                          //
// Illegal instruction (milestone 14, new): id_is_illegal is precisely the   //
// boolean every opcode-recognition wire above this comment does NOT match    //
// -- the same expression id_unimpl used to compute, renamed and (for the      //
// first time) actually threaded downstream, since ap040_ea_fetch.v's           //
// exception-entry sequencer is now a real consumer, not a future one. Also       //
// points id_dest_reg at A7/sets id_writes_reg, same shape as TRAP -- an           //
// illegal-instruction exception pushes a frame and decrements A7 too, per         //
// ap040_core.v's go_illegal task (format $0, vector 4). The one difference         //
// from TRAP: the frame's stacked PC is the illegal instruction's OWN address        //
// (id_pc), not the following instruction's (id_next_pc) -- you can't "return         //
// past" an illegal opcode the way TRAP's handler returns past the TRAP itself;        //
// see ap040_ea_fetch.v's header for where that distinction is actually applied.        //
//                                                                          //
// BSR (milestone 13, new): the SAME opcode class as Bcc (0110 cccc          //
// dddddddd), cc==0001, which is_branch_opcode has excluded since milestone  //
// 4 specifically because it needed a stack push. Reuses Bcc's byte/word/    //
// long gather AND its speculative decode-time redirect verbatim -- BSR is    //
// UNCONDITIONALLY taken (like BRA's cond_true(0)==1 always), so decode's      //
// "assume taken" guess is always correct and needs no EX-side correction.     //
// Kept as its own id_is_bsr flag rather than folded into id_is_branch,         //
// though, because ap040_execute.v's Bcc path checks cond_true(eaf_cond) --      //
// and BSR's condition-code field bits (0001, the literal "F" encoding) would     //
// make that check ALWAYS evaluate false and wrongly fire a misprediction if       //
// BSR were misclassified as a plain branch. Verified against ap040_core.v's        //
// S_BCC_EXT (ap040_core.v:3020-3031, the `ir[11:8]==4'h1` arm) and its byte-         //
// form counterpart in the main 0x6 decode.                                          //
//                                                                          //
// JSR (milestone 13, new): 0100 1110 10 mmm rrr -- one bit different from       //
// JMP's 0100 1110 11 mmm rrr (bits7:6 == 10, not 11; verified against            //
// ap040_core.v's `d_op8_6 == 3'b010` JSR arm vs. `3'b011` JMP arm, same 0x4E       //
// "misc" decode group). Reuses JMP's EA resolution and gather completely           //
// unchanged (id_src_reg = An, id_imm = displacement or 0) for the redirect          //
// target -- what's new is that JSR ALSO needs A7's value for the push, and           //
// port B (id_dest_reg, normally unused by JMP) is free for exactly that: JSR          //
// sets id_dest_reg to A7's unified index (4'd15) -- the register the push+             //
// decrement actually write -- so ap040_ea_fetch.v's EXISTING operand_b mux              //
// (port B, driven by dest_reg) resolves to A7's current, correctly-forwarded             //
// value with zero new mux logic; BSR (which has no EA read at all) reuses the             //
// same trick more simply -- see ap040_ea_fetch.v's header for both.                        //
//                                                                          //
// BSR/JSR both need id_writes_reg=1 STATICALLY (unlike DBcc's runtime-         //
// conditioned write) -- they always decrement A7 whenever they execute at        //
// all -- and id_writes_ccr=0 (neither touches flags, confirmed by the           //
// absence of any CCR update in ap040_core.v's S_BSR_PUSH/S_JSR1/S_JSR2).          //
// The actual push (memory write) and the A7 decrement are                          //
// ap040_ea_fetch.v/ap040_execute.v's job -- see their headers.                       //
//                                                                          //
// JMP (An) / JMP (d16,An) (milestone 11, new): opcode 0100 1110 11 mmm rrr //
// (0x4EC0-0x4EFF), mmm=010 for (An) [single word, no gather] or mmm=101    //
// for (d16,An) [gathers exactly like MOVE.L (d16,An),Dn]. Verified against //
// ap040_core.v:5356 (`d_op8_6 == 3'b011` selects JMP within the 0x4EC0-    //
// 0x4EFF "misc" group's default/mode-111 arm; JSR is `d_op8_6 == 3'b010`   //
// in the same arm, not yet implemented) and its illegal-mode guard         //
// (d_mode<2, ==3, ==4, or immediate all illegal -- (An)/(d16,An) are the   //
// two modes this decoder supports, matching the MOVE.L (An)/(d16,An)       //
// scope already built). rtl_old's S_JMP1 (ap040_core.v:3064) is                //
// `go_pc(ea_addr)` unconditionally once the EA resolves (an odd-target      //
// address-error check is deferred -- same exception-delivery dependency     //
// TRAPcc and DBcc's odd-target check already defer on).                     //
//                                                                          //
// JMP's target reuses EVERYTHING MOVE.L (An)/(d16,An),Dn already built for   //
// EA resolution (id_src_reg = An's unified index, id_imm = the                //
// displacement or 0) -- what's different is what CONSUMES that EA. A         //
// memory-source MOVE dereferences it (ap040_ea_fetch.v's mem_issue/            //
// mem_complete FSM); JMP does NOT dereference anything -- the computed          //
// address IS the result, routed straight into eaf_operand_a on the plain,        //
// non-stalling path (id_is_mem_src stays 0 for JMP; see                           //
// ap040_ea_fetch.v's header for where the `operand_a + eac_imm` add happens         //
// for it). JMP also can't use the Bcc/BRA speculative-redirect mechanism             //
// (id_redirect_valid/id_redirect_pc) the way branches do: that mechanism             //
// needs a LITERAL displacement known at decode time, and JMP's target is a            //
// REGISTER value not resolved until EA-fetch. Instead, id_is_jmp threads               //
// through to ap040_execute.v, which treats an unresolved JMP as an                     //
// UNCONDITIONAL misprediction against decode's implicit "keep going                     //
// sequentially" non-guess -- reusing ex_mispredict/ex_recovery_pc completely              //
// unchanged, the same wiring DBcc reused from Bcc in milestone 8. See                     //
// ap040_execute.v's header for the exact mechanism.                                        //
//                                                                          //
// MOVE.L (d16,An),Dn (milestone 10, new): a THIRD trigger onto the same     //
// gather state machine DBcc added a second one to -- (d16,An) always         //
// carries exactly one 16-bit extension word (the displacement), same        //
// word-form shape as DBcc, so held_is_move_disp/held_dest_reg join            //
// held_is_dbcc/held_reg rather than a parallel mechanism. Two things this     //
// mode needed that neither Bcc nor DBcc did:                                 //
//  - TWO different held registers, not one: dest (Dn) and src (An) are        //
//    different registers here, unlike DBcc where the loop counter is both.    //
//    held_dest_reg is new; held_reg (renamed in spirit, not in code, from      //
//    "DBcc's loop counter") now generically means "the extra register field    //
//    -- meaning depends on held_is_dbcc/held_is_move_disp".                    //
//  - This gather must NOT trigger IF's speculative redirect. Every prior        //
//    gather user (Bcc.W/L, DBcc) is something that MIGHT branch, so             //
//    redirect_from_gather fired unconditionally on any gather completion         //
//    through milestone 9. (d16,An)'s displacement is a MEMORY offset, not an     //
//    instruction address -- redirecting IF there would be a real, silent         //
//    correctness bug (confirmed by mutation-testing the guard back out; see       //
//    ap040_execute.v-adjacent test notes / AP040_IMPLEMENTATION_PLAN.md).          //
//    redirect_from_gather is now gated `&& !held_is_move_disp`.                    //
//                                                                          //
// The effective address itself needs no new machinery in ap040_ea_fetch.v:      //
// EA = An + displacement, and id_imm already exists as a general-purpose         //
// "extra 32-bit value" field (MOVEQ's immediate today) -- reused here to          //
// carry the sign-extended displacement (gather_disp, the SAME wire Bcc/DBcc       //
// already compute the identical way). ap040_ea_fetch.v's address computation       //
// becomes `operand_a + eac_imm` unconditionally for every memory-source            //
// instruction: for MOVE.L (An),Dn, eac_imm is 0 (see the id_imm fix below),         //
// so the addition is a no-op; for (d16,An) it's the real displacement. One          //
// formula, not a per-mode branch -- see ap040_ea_fetch.v's header.                  //
//                                                                          //
// id_imm fix (this milestone, affects the PLAIN (An) case too): the single-       //
// word decode branch previously set id_imm unconditionally to the sign-             //
// extended opcode LOW BYTE for every instruction, correct for MOVEQ and             //
// harmless-because-unused for everything else -- until ap040_ea_fetch.v             //
// started actually ADDING eac_imm to the address this milestone. MOVE.L             //
// (An),Dn's own opcode low byte (e.g. 0x10 for (A0),D0) would have been              //
// silently added to every plain-(An) address as a phantom displacement.              //
// Fixed by explicitly zeroing id_imm for is_move_mem_l in that branch; caught         //
// by re-running tb_ap040_pipe_move_mem.v (unaffected, since eac_imm is 0             //
// either way for its address -- gap closed by mutation-testing the fix back           //
// out, see the plan doc).                                                            //
//                                                                          //
// DBcc Dn,<label> (milestone 8, new): reuses the word-form gather this      //
// module already built for Bcc.W -- DBcc always carries exactly one 16-bit //
// extension word (the branch displacement), never byte or long forms, so   //
// it is added as a second trigger onto the SAME gather state machine        //
// (held_is_dbcc/held_reg alongside held_cond/held_pc), not a parallel one.  //
// Verified bit-for-bit against ap040_core.v's own decode                   //
// (ap040_core.v:5415-5421's `d_mode == 3'b001` -> DBcc arm of its 0x5       //
// ADDQ/SUBQ/Scc/DBcc group, immf(2'd1, S_DBCC1) confirming the single-word  //
// extension) and the same base-address convention as every other branch     //
// form (br_base <= pc, captured before the extension word is fetched --     //
// ap040_core.v:5419-5420 -- i.e. opcode_pc + 2, the formula id_redirect_pc  //
// already computes for the word-gather case, so DBcc needs no new base       //
// arithmetic here at all).                                                 //
//                                                                          //
// Decode still only speculatively redirects IF to the branch target,        //
// unconditionally assuming the loop continues -- the real decision (does    //
// the condition hold, and if not, does the decremented counter reach -1)    //
// needs a register value that isn't available until EA-fetch/EX, exactly    //
// the same timing argument milestone 5 made for Bcc's condition check; see  //
// ap040_execute.v's header for the decrement/compare logic and why DBcc      //
// reuses Bcc's mispredict/recovery wiring (ex_mispredict/ex_recovery_pc)     //
// completely unchanged -- both of DBcc's "don't branch" outcomes (condition //
// true, or counter expired) are, from IF's perspective, the identical         //
// fall-through-to-eaf_next_pc recovery Bcc already has.                     //
//                                                                          //
// The one thing DBcc needs that Bcc never did: it both reads AND writes Dn, //
// and unlike every register-writing instruction so far, whether it writes   //
// at all depends on a runtime value (the condition), not a static decode     //
// bit -- so id_writes_reg stays 0 here (like Bcc) and ap040_execute.v gates  //
// the real write dynamically off cond_result. id_dest_reg/id_src_reg are     //
// still threaded as Dn's own number (same field position as Scc's dest,      //
// if_opcode[2:0]) so EA-fetch reads Dn's current value into operand_a the     //
// normal way -- no new regfile port shape needed.                           //
//                                                                          //
// Multi-word gather (new this milestone): ap040_inst_fetch.v still hands   //
// over exactly one word per cycle, oblivious to instruction boundaries --  //
// deliberately NOT the real 68040's wide-prefetch-buffer approach (doc/    //
// The_68040_processor_I_Design_and_impleme.pdf describes an 8-word buffer  //
// and a 3-word decode window; disproportionate to what's needed here and   //
// not what section 16's "architecturally-equivalent, not cycle-exact"      //
// non-goal calls for). Instead, THIS stage recognizes when it needs more   //
// words than the one it's looking at, and spends the next 1-2 cycles       //
// treating ap040_inst_fetch.v's incoming words as extension DATA rather    //
// than fresh opcodes, emitting exactly one complete id_* entry once        //
// assembled. ext_pending/held_is_long/held_pc/held_cond/disp_acc are the    //
// gather state; none of it is visible outside this module -- once a        //
// Bcc.W/Bcc.L instruction is fully assembled it looks identical to Bcc.B    //
// from EA-calc onward, so nothing downstream needs to know gathering ever   //
// happened.                                                                //
//                                                                          //
// Two facts this relies on, verified against ap040_core.v's own            //
// implementation rather than assumed:                                      //
//  - the branch-displacement base is ALWAYS opcode_pc + 2 regardless of     //
//    form (ap040_core.v's br_base <= pc is captured at the same decode      //
//    instant for all three forms, before any extension word is fetched) --  //
//    so only the displacement's width/source changes between forms, not     //
//    the base formula Bcc.B already uses.                                   //
//  - word order for the 32-bit form is high-word-first: ap040_core.v:1817's //
//    imm <= {imm[15:0], epf_data[...]} shifts each new word in as the new   //
//    low half, so the first-fetched word becomes the high half once the     //
//    second arrives.                                                       //
//                                                                          //
// id_next_pc (new this milestone, every instruction not just branches):     //
// every instruction implemented before this one was exactly one word, so    //
// ap040_execute.v's fall-through/recovery address (eaf_pc + 2) was correct  //
// only by coincidence of scope. A 2-/3-word Bcc breaks that. The general    //
// fix -- needed for every future variable-length instruction, not just      //
// this one -- is for decode to compute the ACTUAL next-instruction address  //
// itself and thread it as its own field, rather than leave "+2" arithmetic  //
// at EX guessing at instruction length. id_pc stays "this instruction's     //
// own opcode address" (debug taps, the branch-target base above);           //
// id_next_pc is the new, separate "address of whatever comes after".        //
//                                                                          //
// flush during a gather (new correctness case): if a mispredict from an     //
// OLDER, already-in-flight branch arrives while this stage is mid-gather,   //
// the instruction being assembled is on the wrong path too (younger than    //
// the mispredicted branch) and must be abandoned, not just have its         //
// (already-0) id_valid re-zeroed -- flush resets ext_pending as well, so    //
// the next incoming word (from the recovery-redirected fetch stream) is     //
// correctly treated as a fresh opcode, not leftover extension data from a   //
// discarded instruction.                                                    //
//                                                                          //
// Bcc/BRA redirect: ap040_core.v itself treats BRA as nothing but the      //
// always-true case of the SAME Bcc mechanism -- ap040_core.v:4802-4819's   //
// 4'h6 decode calls one shared finish_bcc(target, cond_true(ir[11:8])) for //
// every condition code including T (ir[11:8]==0). This decoder does the    //
// same: id_redirect_valid/id_redirect_pc fire COMBINATIONALLY the instant   //
// the full displacement is known -- immediately, from if_opcode alone, for  //
// the byte form (same cycle it's fetched, before it's even latched into     //
// id_*); on the gather-completing cycle for the word/long forms -- for      //
// EVERY branch regardless of its actual condition, the "always assume       //
// taken" policy the real 68040 uses. For BRA (cond_true(0)==1 always) the   //
// guess is always right and nothing downstream ever needs to correct it.    //
// For a real Bcc, the guess can be wrong; id_is_branch/id_cond are threaded //
// onward (through ap040_ea_calc.v/ap040_ea_fetch.v unchanged) so            //
// ap040_execute.v can check the real condition once CCR forwarding actually //
// has something to forward (see its header comment for why EX, not here,   //
// is where that check has to happen) and signal a flush back through this  //
// stage's `flush` input if the guess was wrong.                            //
//                                                                          //
// Scc.B Dn shares id_cond's field position with Bcc (both put the 4-bit     //
// condition at if_opcode[11:8]), so it reuses id_cond directly -- no new    //
// condition field, and ap040_execute.v's cond_true() (all 16 encodings,     //
// not a subset) is reused by both consumers as-is. Scc's dest register is   //
// at if_opcode[2:0], NOT if_opcode[11:9] like MOVEQ/MOVE.L/ADD.L's dest --  //
// for Scc that upper field is the condition code instead -- so             //
// id_dest_reg's source is a per-opcode mux, the same shape id_alu_op        //
// already uses.                                                            //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_decode
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // EA-calc cannot accept this cycle
	input             flush,      // EX detected a misprediction: force a bubble

	input             if_valid,
	input      [31:0] if_pc,
	input      [15:0] if_opcode,
	// The fetch of this word faulted (2026-09-24). ap040_pipe_membus.v hands
	// over ILLEGAL, $4AFC, in its place -- one word, no redirect, no gather
	// -- so nothing here decodes the faulted fetch's stale data; if_flt_bus
	// says it was a physical bus error rather than the MMU's refusal.
	input             if_flt,
	input             if_flt_bus,
	// The word after if_opcode, when the fetch has it (phase 8): a gathered
	// instruction whose first extension word it is completes in one cycle,
	// and take2 tells the fetch that both words were taken.
	input             if_valid2,
	input      [15:0] if_word2,
	input             if_flt2,
	output            take2,

	output            id_stall,   // to IF: no local stall of its own yet

	// Bcc/BRA redirect, combinational -- see header comment
	output            id_redirect_valid,
	output     [31:0] id_redirect_pc,

	output reg        id_valid,
	// The words decode holds and has not emitted (2026-09-24): a gather in
	// progress starts at dec_hold_pc. ap040_pipe_cpu.v snoops stores against
	// them, as the prefetch stream is snooped in ap040_pipe_membus.v.
	output            dec_holding,
	output     [31:0] dec_hold_pc,
	output reg [31:0] id_pc,
	output reg [31:0] id_next_pc,
	output reg  [3:0] id_dest_reg,
	output reg  [3:0] id_src_reg,
	output reg [31:0] id_imm,
	// The EA's own extension value when id_imm is busy carrying an OPERAND
	// (milestone 112): an immediate-to-memory or ADDQ/SUBQ-to-memory form at
	// (d16,An), (d8,An,Xn) or an absolute address has both. EA-fetch takes
	// its displacement, index word and absolute address from here whenever
	// id_immrmw is set, and from id_imm otherwise. Zero for every form that
	// has no EA extension, which reproduces the old masked displacement.
	output reg [31:0] id_ea_ext,
	output reg  [6:0] id_mm,        // MOVE mem-to-mem: {xm, mm, #imm src, dst (An)+, -(An), abs, idx}
	output reg  [2:0] id_moves,
	output reg  [1:0] id_mvfsr,     // MOVE from SR/CCR: {it is one, CCR rather than SR}
	output reg  [1:0] id_pc_off,    // immediate words between the opcode and a PC-relative displacement
	output reg  [2:0] id_movep,     // MOVEP: {movep, long, register -> memory}
	output reg  [6:0] id_ml,        // MULx.L/DIVx.L: {it is one, divide, signed, 64-bit, Dh/Dr}
	output reg  [4:0] id_bf,        // bitfield: {it is one, register operand, op}; the word is in id_ea_ext
	output reg  [2:0] id_ck2,       // CHK2/CMP2: {it is one, CHK2, Rn is an An}
	output reg  [4:0] id_cas,       // CAS: {CAS2, CAS, Du}
	output reg  [3:0] id_m16,       // MOVE16: {it is one, form}     // MOVES: {moves, byte load into An (sign-extend), store of its own stepped An}
	output reg  [5:0] id_alu_op,
	output reg  [1:0] id_size,
	output reg  [5:0] id_shcnt,
	output reg        id_shift_reg,   // the count is in the register id_src_reg names
	output reg        id_src_a_is_imm,
	output reg        id_writes_reg,
	output reg        id_writes_ccr,
	output reg        id_is_branch,
	output reg        id_is_scc,
	output reg        id_is_dbcc,
	output reg        id_is_mem_src,
	output reg        id_is_abs,
	output reg        id_is_store,
	output reg        id_is_postinc,
	output reg        id_is_predec,
	output reg        id_is_jmp,
	output reg        id_is_lea,
	output reg        id_sxt_w,
	output reg        id_ea_indexed,
	output reg        id_ea_pcrel,
	output reg        id_is_rmw,
	// The RMW's second operand is the gathered immediate, not a register,
	// and eac_imm is therefore NOT a displacement (milestone 89).
	output reg        id_immrmw,
	// A memory destination written without being read (restructuring plan,
	// phase 2): CLR and Scc. The M68000PRM (CLR p. 4-74, Scc p. 4-173)
	// gives their preliminary read as MC68000/MC68008 behaviour; here they
	// were read-modify-writes, a whole read for a value neither uses.
	output reg        id_st_only,
	// A store whose address carries a displacement (milestone 91).
	output reg        id_st_disp,
	output reg        id_is_trapcc,
	output reg        id_is_chk,
	// CHK.L rather than CHK.W. Not derivable downstream: id_size is Long for
	// BOTH forms because the word read is forced through id_sxt_w instead,
	// and the gathered CHK #imm sets neither.
	output reg        id_chk_long,
	output reg        id_is_immsr,
	output reg        id_is_stop,
	output reg        id_immsr_to_sr,
	output reg        id_is_pea,
	output reg        id_is_link,
	output reg        id_is_div,
	output reg        id_div_signed,
	output reg        id_is_movem,
	output reg        id_movem_dir,
	output reg        id_movem_word,
	output reg        id_movem_down,
	output reg        id_movem_wb,
	output reg        id_movem_pcrel,
	output reg        id_movem_abs,
	output reg [15:0] id_movem_mask,
	output reg        id_is_unlk,
	output reg        id_is_bsr,
	output reg        id_is_jsr,
	output reg        id_is_trap,
	output reg        id_is_illegal,
	// Which unimplemented-opcode vector this is: 0 illegal (4), 1 A-line
	// (10), 2 F-line (11). ap040_core.v:6318 raises A-line with the same
	// format-0 frame and stacked PC an illegal instruction gets, so the
	// whole path is shared and only the vector number differs.
	output reg  [1:0] id_illegal_kind,
	output reg        id_is_movesr,
	output reg        id_is_movec,
	output reg        id_is_rts,
	output reg        id_is_nop,       // T0 trace treats NOP as a change of flow (milestone 79)
	output reg  [5:0] id_cinv,         // CINV/CPUSH: {valid, IC, DC, push, scope[1:0]}
	output reg  [4:0] id_pmmu,         // PTEST/PFLUSH: {valid, PTEST, write, mode}
	// An instruction whose fetch faulted: {valid, bus error, the faulted
	// word's offset in words from id_pc}. It issues as the ILLEGAL it was
	// handed, and EA-fetch takes the access error in its place.
	output reg  [5:0] id_fflt,
	output reg        id_bnt,          // a conditional branch predicted NOT taken (2026-09-24)
	output reg        id_is_reset,     // RESET: privileged, nothing else (milestone 117)
	output reg        id_is_rte,
	output reg        id_is_rtr,       // RTR: rides RTE's pops (milestone 117)
	output reg  [3:0] id_cond,
	// The F-line coprocessor-1 instructions, for ap040_pipe_fpu.v (2026-09-24)
	output reg        id_fp,
	output reg  [8:0] id_fp_op,       // opcode[8:0]: kind, EA mode, EA register
	output reg [15:0] id_fp_cmd,      // the command / condition word
	output reg [95:0] id_fp_imm,      // every word gathered after the opcode, right-aligned
	// A full-format extension word (2026-09-24): {valid, the MOVE destination's,
	// BS, IS, post-indexed, memory indirect}, and its displacements.
	output reg  [5:0] id_fx,
	output reg [31:0] id_fx_bd,
	output reg [31:0] id_fx_od
);

assign id_stall = stall_in;

// ap040_core.v:667-670 equivalents. All the opcodes decoded so far happen
// to share these same two field positions, except Scc (see header comment).
wire [2:0] d_reg9 = if_opcode[11:9];   // MOVEQ/MOVE.L/ADD.L dest Dn
wire [2:0] d_rn   = if_opcode[2:0];    // MOVE.L/ADD.L src Dn; Scc dest Dn

//--------------------------------------------------------------------------//
// The shared effective-address classification (milestone 107).             //
//                                                                          //
// Everything below this point used to decide, per instruction family, which //
// addressing modes it accepted and how many extension words each one        //
// gathers -- 29 separate *_shape wires testing if_opcode[5:3] in 92 places, //
// and a SECOND enumeration further down listing every instruction that      //
// gathers at all. The two had to agree, and milestone 89 is the record of   //
// what happens when they do not.                                            //
//                                                                          //
// ap040_core.v does it once. ea_start/S_EA_DISP (ap040_core.v:1764 and      //
// :2648) is a single case over the mode field that every instruction enters //
// with a return state, and dst_not_alt/src_not_data (ap040_core.v:940) are  //
// the two 68k mode CLASSES that say which modes an instruction may use.     //
// That core passes all 658 Basic corpus slices, so these are not re-derived //
// from the manual here; they are lifted, and cited.                         //
//                                                                          //
// The FSM's sequencing does not come with them -- a six-stage pipeline      //
// cannot call an EA subroutine and return -- but the classification is      //
// pure combinational decode and transfers whole. A family now says which    //
// CLASS of modes it takes, not which modes.                                 //
//--------------------------------------------------------------------------//
wire [2:0] ea_mode  = if_opcode[5:3];
wire [2:0] ea_reg   = if_opcode[2:0];

wire ea_is_dn       = (ea_mode == 3'b000);
wire ea_is_an       = (ea_mode == 3'b001);
wire ea_is_ind      = (ea_mode == 3'b010);   // (An)
wire ea_is_pi       = (ea_mode == 3'b011);   // (An)+
wire ea_is_pd       = (ea_mode == 3'b100);   // -(An)
wire ea_is_d16      = (ea_mode == 3'b101);   // (d16,An)
wire ea_is_idx      = (ea_mode == 3'b110);   // (d8,An,Xn)
wire ea_mode7       = (ea_mode == 3'b111);
wire ea_is_absw     = ea_mode7 && (ea_reg == 3'b000);
wire ea_is_absl     = ea_mode7 && (ea_reg == 3'b001);
wire ea_is_pcd16    = ea_mode7 && (ea_reg == 3'b010);
wire ea_is_pcidx    = ea_mode7 && (ea_reg == 3'b011);
wire ea_is_imm      = ea_mode7 && (ea_reg == 3'b100);
wire ea_is_abs      = ea_is_absw || ea_is_absl;
wire ea_is_pcrel    = ea_is_pcd16 || ea_is_pcidx;

// ap040_core.v:940, verbatim in meaning. "Alterable" excludes An direct and
// everything in mode 7 above (xxx).L -- the PC-relative pair, immediate, and
// the three reserved encodings. "Data" additionally allows the PC-relative
// pair and immediate, excluding only An direct and the reserved three.
wire ea_not_alt     = ea_is_an || (ea_mode7 && (ea_reg > 3'b001));
wire ea_not_data    = ea_is_an || (ea_mode7 && (ea_reg > 3'b100));

// Extension words the mode itself gathers, from ap040_core.v:2648's
// S_EA_DISP. Immediate is NOT here: its count follows the operand size,
// which is the instruction's business and not the mode's.
wire [1:0] ea_ext_words = (ea_is_d16 || ea_is_idx || ea_is_absw ||
                           ea_is_pcd16 || ea_is_pcidx) ? 2'd1 :
                          ea_is_absl                   ? 2'd2 : 2'd0;

// MOVEQ: 0111 rrr 0 iiiiiiii  (rrr = dest Dn, iiiiiiii = 8-bit immediate)
wire is_moveq = (if_opcode[15:12] == 4'b0111) && (if_opcode[8] == 1'b0);

// MOVE.L Dn,Dm: 00 10 RRR 000 000 rrr (RRR = dest Dn, rrr = src Dn,
// dest/src EA modes both 000 = data-register-direct; no other EA mode yet).
// The size field (if_opcode[13:12]) is matched against the raw '10' = Long
// encoding rather than ap040_core.v's move_size decode (ap040_core.v:673-
// 674) because byte/word MOVE aren't implemented yet; switch to that
// convention when they are.
// Size is no longer baked into the match: MOVE's ir[13:12] is 01=B, 11=W,
// 10=L (00 is not a MOVE at all), and the ALU has implemented all three
// widths since it was forked -- only this decoder and execute's hardwired
// .size(AP040_SZ_L) kept the pipeline Long-only.
wire is_move_rr = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                   (if_opcode[8:6]  == 3'b000) && (if_opcode[5:3]  == 3'b000);

// ADD.L Dn,Dm: 1101 RRR 0 10 000 rrr (RRR = dest Dm, rrr = src Dn; ir[8]=0
// selects the "<ea>+Dn->Dn" direction, ir[7:6]=std_size=10=Long, ir[5:3]=000
// = ea is Dn direct). Verified bit-for-bit against ap040_core.v's own
// SUB/ADD decode (ap040_core.v:4928-4974, d_op8_6/std_size at :668,:675),
// not guessed. No other size/direction/EA mode yet.
// ADD's std_size is ir[7:6] and maps onto AP040_SZ_B/W/L directly (0/1/2);
// 11 is ADDA, a different instruction, and stays excluded.
// The whole "<ea> op Dn -> Dn" family with a register-direct source. They
// share ONE encoding shape -- 1ooo RRR 0 SS 000 rrr -- and differ only in
// the top nibble, so one predicate and one op map reach all of them. The
// ALU has implemented every one since it was forked from rtl_old; decode
// reach was the only thing missing, which is why it could emit exactly two
// of the ALU's thirty-three operations.
//
// Excluded deliberately: ir[8]=1 is the other direction (Dn -> <ea>), a
// memory write this pipeline has no path for yet; ir[7:6]=11 is the
// ADDA/SUBA/CMPA address-register form, a different instruction; and 1010 /
// 1110 / 1111 are A-line, the shift group and F-line, none of which is in
// the enumerated nibble list below.
wire alu_rr_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b0) &&
                    (if_opcode[7:6]  != 2'b11) && (if_opcode[5:3] == 3'b000);
wire is_or_rr  = alu_rr_shape && (if_opcode[14:12] == 3'b000);   // 1000
wire is_sub_rr = alu_rr_shape && (if_opcode[14:12] == 3'b001);   // 1001
wire is_cmp_rr = alu_rr_shape && (if_opcode[14:12] == 3'b011);   // 1011 (ir[8]=1 would be EOR)

// EOR Dn,Dm (milestone 47): 1011 RRR 1 SS 000 rrr. The comment above has
// named this hole since the binary family was written -- ir[8]=1 is the
// Dn -> <ea> direction, which for nibble 1011 is EOR rather than a second
// CMP, and ap040_pipe_alu.v has implemented AP040_ALU_EOR since the fork.
// Only decode was missing.
//
// The operand roles are REVERSED from the ir[8]=0 family: ir[11:9] is the
// SOURCE here and ir[2:0] the DESTINATION, the same arrangement bitop_shape
// uses. EOR's result is symmetric, so getting that backwards would be
// invisible in the result and visible only in which register changed --
// which is what tb_ap040_pipe_eor.v checks.
//
// Mode 001 must stay out: 1011 RRR 1 SS 001 rrr is CMPM, a different
// instruction entirely. Only mode 000 is reached here; the memory
// destinations are the read-modify-write direction and need a store path
// this pipeline does not have yet.
wire is_eor_rr = (if_opcode[15:12] == 4'b1011) && (if_opcode[8] == 1'b1) &&
                 (if_opcode[7:6]  != 2'b11)    && (if_opcode[5:3] == 3'b000);
wire is_and_rr = alu_rr_shape && (if_opcode[14:12] == 3'b100);   // 1100
wire is_add_rr = alu_rr_shape && (if_opcode[14:12] == 3'b101);   // 1101
wire is_alu_rr = is_or_rr || is_sub_rr || is_cmp_rr || is_and_rr || is_add_rr;

// Source mode 001 -- an ADDRESS register as the source (milestone 43). The
// LEA bench found this hole: is_move_rr above requires source mode 000, so
// until now NO instruction in this decoder could read An as a source
// operand, and MOVE.L A0,D0 was illegal.
//
// It is pure decode. An lives in the same 4-bit unified register space the
// rest of this stage already uses (8+n), so naming it in id_src_reg is the
// whole change -- no new port, no datapath, nothing threaded anywhere.
//
// The 68k restricts it, and so does this:
//   - Byte is never allowed. An address register has no byte operand.
//   - Only ADD, SUB and CMP take one. AND and OR do not, and admitting them
//     would decode instructions a real 68040 rejects.
// The size fields differ between the two families, which is why the Byte
// exclusion is written twice: MOVE encodes Byte as ir[13:12]==01, the ALU
// family as ir[7:6]==00.
wire is_move_an = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                  (if_opcode[13:12] != 2'b01) &&
                  (if_opcode[8:6]   == 3'b000) && (if_opcode[5:3] == 3'b001);
wire alu_an_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b0) &&
                    (if_opcode[7:6]  != 2'b11) && (if_opcode[7:6] != 2'b00) &&
                    (if_opcode[5:3]  == 3'b001);
wire is_alu_an  = alu_an_shape &&
                  ((if_opcode[14:12] == 3'b001) ||   // SUB
                   (if_opcode[14:12] == 3'b011) ||   // CMP
                   (if_opcode[14:12] == 3'b101));    // ADD
wire is_cmp_an  = alu_an_shape && (if_opcode[14:12] == 3'b011);
// One wire for every site that only cares "this instruction's source is An".
wire is_an_src  = is_move_an || is_alu_an;

// ADDA/SUBA/CMPA (milestone 44): 1ooo AAA 0 11 mmm rrr for the Word form and
// 1ooo AAA 1 11 mmm rrr for the Long one. Pointer arithmetic -- what every
// array walk and every stack adjustment is made of. The three families that
// have one are SUB (1001), CMP (1011) and ADD (1101), and opmode 011/111 is
// exactly ir[7:6]==11 with ir[8] as the size bit, which is why the shapes
// above all carry a `!= 2'b11` term: this is the instruction they were
// excluding.
//
// Three things make it different from every ALU form so far:
//
//  - The destination is An, and the write is ALWAYS 32 bits wide. ADDA.W
//    updates the whole address register, not its low half.
//  - ADDA and SUBA set NO condition codes at all. CMPA does -- it is a
//    compare -- and writes no register, the same split CMP already has.
//  - The Word form SIGN-EXTENDS its source to 32 bits and then operates on
//    the full width. Nothing in this pipeline did that before: id_size has
//    always meant one width for the memory access AND the ALU, and here
//    those differ. id_sxt_w carries the difference -- it forces a Word-sized
//    read (and a Word-sized auto-increment step) while id_size stays Long
//    for the ALU, and sign-extends whatever the source turned out to be.
//    Zero-extending instead is a silently wrong answer for any negative
//    offset, which is most of what SUBA is used for.
//
// Source modes here are the five that need no extension word: Dn, An, (An),
// (An)+ and -(An). The gathering modes -- (d16,An) and, most valuable of
// all, #imm, which is how a stack frame is opened and closed -- are the
// next step and are NOT reached by this milestone.
wire adda_shape = ((if_opcode[15:12] == 4'b1001) ||    // SUBA
                   (if_opcode[15:12] == 4'b1011) ||    // CMPA
                   (if_opcode[15:12] == 4'b1101)) &&   // ADDA
                  (if_opcode[7:6] == 2'b11);
wire adda_mode_ok = (if_opcode[5:3] == 3'b000) || (if_opcode[5:3] == 3'b001) ||
                    (if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b011) ||
                    (if_opcode[5:3] == 3'b100);
wire is_adda    = adda_shape && adda_mode_ok;
wire is_cmpa    = is_adda && (if_opcode[15:12] == 4'b1011);
wire is_adda_w  = is_adda && (if_opcode[8] == 1'b0);
wire is_adda_mem = is_adda && ((if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b011) ||
                               (if_opcode[5:3] == 3'b100));
wire is_adda_pi = is_adda && (if_opcode[5:3] == 3'b011);
wire is_adda_pd = is_adda && (if_opcode[5:3] == 3'b100);
// Dn direct is the one source mode here that is NOT an address register.
wire is_adda_areg_src = is_adda && (if_opcode[5:3] != 3'b000);

// ADDA/SUBA/CMPA with an IMMEDIATE source (milestone 45), mode 111 reg 100.
// ADDA.L #n,A7 is how a stack frame is opened and closed, so the family is
// not useful for compiled code without it.
//
// This needs no new gather kind and no sign-extension flag. held_is_imm
// already assembles immediates, already routes them into operand_a through
// id_src_a_is_imm, and already writes An without condition codes for
// MOVEA.L #imm,An. And gather_disp SIGN-EXTENDS its single-word form
// already -- the Word immediate arrives 32 bits wide and correct, so
// id_sxt_w is not involved here at all.
//
// One thing did have to be separated. id_writes_ccr derived "sets no
// condition codes" from held_imm_areg, i.e. from the destination being an
// address register, which held for every immediate form until now. CMPA
// breaks it: the destination IS an address register and it DOES set
// condition codes. held_imm_ccr carries that directly instead of inferring
// it, and reproduces the old behaviour exactly for the older forms.
wire is_adda_imm  = adda_shape && (if_opcode[5:0] == 6'b111100);
wire is_cmpa_imm  = is_adda_imm && (if_opcode[15:12] == 4'b1011);
wire is_adda_imm_l = is_adda_imm && (if_opcode[8] == 1'b1);

// ADDA/SUBA/CMPA with a (d16,An) source (milestone 46). This rides
// milestone 40's gather kind rather than adding a tenth: it is the same
// one extension word, the same held_reg base, the same gather_disp. What
// differs is what the instruction DOES with the loaded value, so the kind
// grew three properties instead of being duplicated --
//
//   held_alu_areg  the destination is An, and the ALU width is Long
//   held_alu_ccr   this form sets condition codes (CMPA does, ADDA/SUBA not)
//   held_alu_sxt   the Word form sign-extends its source before operating
//
// -- each of which was previously a constant of the kind. That is the third
// time a "property of the kind" has had to become a carried bit rather than
// an inference (held_alu_op in milestone 40, held_imm_ccr in 45), and the
// pattern is the same each time: a new instruction shares a gather's SHAPE
// but not its semantics.
// ALU-to-memory, the ir[8]=1 direction (milestone 48): 1ooo RRR 1 SS mmm rrr
// with mmm naming a memory destination. This is a READ-MODIFY-WRITE --
// <ea> op Dn -> <ea> -- and it is the first instruction in this pipeline
// that both loads and stores, which is why it needed a real datapath
// addition rather than decode alone. See ap040_execute.v's header.
//
// Operand roles: ir[11:9] is the SOURCE data register and mmm/rrr the
// DESTINATION, as in EOR above. The register file's two read ports both go
// to work: id_src_reg names An so operand_a is the ADDRESS base, and
// id_dest_reg names Dn so operand_b is the data. ap040_ea_fetch.v then
// crosses them over, because the ALU computes b op a and SUB.L D0,(A0) must
// be memory MINUS D0, not the other way round.
//
// Nibble 1011 is EOR in this direction, not CMP -- there is no "CMP to
// memory" -- so the op map differs from alu_nib_op by that one entry.
//
// Modes 000 and 001 must stay out: with ir[8]=1 they are ADDX/SUBX (and
// CMPM for nibble 1011), entirely different instructions. Mode 101,
// (d16,An), would need a tenth gather kind and is not reached here.
wire alu_dst_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b1) &&
                     (if_opcode[7:6]  != 2'b11) &&
                     ((if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b011) ||
                      (if_opcode[5:3] == 3'b100));
wire is_alu_dst = alu_dst_shape &&
                  ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                   (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                   (if_opcode[14:12] == 3'b101));
wire is_alu_dst_pi = is_alu_dst && (if_opcode[5:3] == 3'b011);
wire is_alu_dst_pd = is_alu_dst && (if_opcode[5:3] == 3'b100);
wire [5:0] alu_nib_dst_op = (if_opcode[14:12] == 3'b000) ? `AP040_ALU_OR  :
                            (if_opcode[14:12] == 3'b001) ? `AP040_ALU_SUB :
                            (if_opcode[14:12] == 3'b011) ? `AP040_ALU_EOR :
                            (if_opcode[14:12] == 3'b100) ? `AP040_ALU_AND :
                                                           `AP040_ALU_ADD;

// LINK An,#d16 and UNLK An (milestone 49): $4E50+n and $4E58+n. Every
// compiled function opens with one and closes with the other.
//
// LINK  pushes An, points An at the new top of stack, then moves A7 by the
//       displacement (negative, to allocate locals):
//         mem[A7-4] <- An ;  An <- A7-4 ;  A7 <- A7-4+d16
// UNLK  undoes it:
//         An <- mem[An] ;  A7 <- An+4
//
// Both write TWO registers, which is what makes them fit: milestone 30's
// second write port already exists for autoincrement and is free here,
// because neither instruction autoincrements. LINK's memory write reuses
// eac_is_push, the BSR/JSR path, whose address is already operand_b - 4 --
// so pointing eac_dest_reg at A7 makes push_addr come out right with no new
// arithmetic. Only the pushed DATA is new: BSR pushes eac_next_pc, LINK
// pushes the old An.
//
// UNLK needs no new memory path at all: a plain load through
// mem_issue/mem_complete with An as both the address and the destination.
//
// LINK is the TENTH gather kind, and as the plan predicted after milestone
// 46 it carries its own properties rather than inferring them from shape.
// MOVEM.L (milestone 50), the two autoincrement forms -- which are the
// prologue/epilogue idiom and the reason the instruction matters:
//
//   MOVEM.L <list>,-(An)   $48E0+n   registers to memory, predecrementing
//   MOVEM.L (An)+,<list>   $4CD8+n   memory to registers, postincrementing
//
// An extension word carries a 16-bit register MASK, and the instruction is
// genuinely multi-cycle: one memory access per set bit, up to sixteen. It
// gets a sequencer in ap040_ea_fetch.v alongside the exception-frame and
// RTE ones, and a THIRD register write port, because one instruction
// writing sixteen registers cannot use a writeback path that carries one
// result per instruction.
//
// The mask is numbered differently in the two directions, which is the
// detail worth stating once rather than rediscovering: for the
// predecrementing store, bit 0 is A7 and bit 15 is D0, so walking the mask
// from bit 0 upward stores A7 first at the highest address; for every other
// mode, including the postincrementing load, bit 0 is D0 and bit 15 is A7.
// Taking lowest-set-bit first in both cases, the register index is
// 15 - bit for the store and bit itself for the load, which is why they
// reverse cleanly into one sequencer.
// MULU.W/MULS.W <ea>,Dn (milestone 51): 1100 DDD 011 mmm rrr and
// 1100 DDD 111 mmm rrr. Nibble 1100 with ir[7:6]==11 is the slot every ALU
// shape above has been excluding -- it is AND's opmode 011/111, which the
// 68k gives to multiply rather than to a second AND direction.
//
// A 16x16 -> 32 multiply is one DSP block and needs no sequencer, so this
// is decode plus two ALU cases. The 32-bit result goes to the whole of Dn,
// which is why id_size is Long while the SOURCE is a word: id_sxt_w forces
// the memory read (and the autoincrement step) to Word, exactly as it does
// for ADDA.W. Its sign extension is irrelevant here in a way worth stating
// -- the ALU reads only bits [15:0] of each operand, so MULU is not made
// signed by arriving sign-extended.
//
// Source modes here are Dn and the three register-indirect ones. An is not
// a legal source for multiply and is excluded by construction.
// DIVU.W/DIVS.W <ea>,Dn (milestone 52): 1000 DDD 011 mmm rrr and
// 1000 DDD 111 mmm rrr -- OR's opmode 011/111, the mirror of multiply's
// slot in nibble 1100.
//
// Unlike multiply, these cannot be combinational: a 32/16 divide is an
// iterative sequencer in ap040_execute.v, which is why it lives there and
// not in ap040_pipe_alu.v. Dn / <ea> gives a 16-bit quotient in Dn[15:0]
// and a 16-bit remainder in Dn[31:16].
//
// They also bring two conditions nothing in this core had before:
//
//   Divisor zero  -> a real EXCEPTION, vector 5. The divisor is known in
//     ap040_ea_fetch.v, so this is detected there and routed through the
//     same dynamic-exception path an odd JMP target already uses, rather
//     than needing anything new.
//   Quotient too wide -> V is set and the destination is left UNCHANGED.
//     That is the first instruction here that can fail without writing
//     anything, and it reuses DBcc's writes_reg_resolved hook.
wire div_shape = (if_opcode[15:12] == 4'b1000) && (if_opcode[7:6] == 2'b11);
wire is_div    = div_shape && mul_mode_ok;
wire is_divs   = is_div && (if_opcode[8] == 1'b1);
wire is_div_mem = is_div && (if_opcode[5:3] != 3'b000);
wire is_div_pi = is_div && (if_opcode[5:3] == 3'b011);
wire is_div_pd = is_div && (if_opcode[5:3] == 3'b100);

wire mul_shape = (if_opcode[15:12] == 4'b1100) && (if_opcode[7:6] == 2'b11);
wire mul_mode_ok = (if_opcode[5:3] == 3'b000) || (if_opcode[5:3] == 3'b010) ||
                   (if_opcode[5:3] == 3'b011) || (if_opcode[5:3] == 3'b100);
wire is_mul    = mul_shape && mul_mode_ok;
wire is_muls   = is_mul && (if_opcode[8] == 1'b1);
wire is_mul_mem = is_mul && (if_opcode[5:3] != 3'b000);
wire is_mul_pi = is_mul && (if_opcode[5:3] == 3'b011);
// MUL/DIV with an ABSOLUTE source (milestone 111). mul_mode_ok admits
// 000/010/011/100 only, and the corpus' MULS/MULU/DIVS/DIVU slices are
// dominated by mode 111 reg 001 -- 372,692 rounds between the four.
wire is_mul_abs    = mul_shape && abs_mode;
wire is_div_abs    = div_shape && abs_mode;
wire is_muldiv_abs = is_mul_abs || is_div_abs;
wire is_muldiv_abs_l = is_muldiv_abs && abs_long;
wire is_muldiv_abs_signed = is_muldiv_abs && (if_opcode[8] == 1'b1);
// ...and at (d16,An) and (d8,An,Xn), which is now what dominates those four
// families -- ~61k rounds each. They ride held_is_alu_disp, which already
// gives the An as the address base and handles the brief extension word.
wire muldiv_gather_mode = (if_opcode[5:3] == 3'b101) || ea_indexed_mode || ea_pcrel_mode;
wire is_mul_gather      = mul_shape && muldiv_gather_mode;
wire is_div_gather      = div_shape && muldiv_gather_mode;
wire is_muldiv_gather_s = (is_mul_gather || is_div_gather) && (if_opcode[8] == 1'b1);
wire is_mul_pd = is_mul && (if_opcode[5:3] == 3'b100);

// MULU/MULS/DIVU/DIVS with an IMMEDIATE source (milestone 53), mode 111
// reg 100. MULU #10,D0 and DIVU #10,D0 are everywhere in compiled code, and
// they need no new gather kind: held_is_imm already assembles a Word
// immediate, already routes it into operand_a through id_src_a_is_imm, and
// already writes a data register with condition codes.
//
// Two properties had to be carried rather than inferred, which is the same
// pattern milestones 40, 45 and 46 each hit: this gather's operation was
// always an ALU op, and a divide is not one -- it is a sequencer flag that
// ap040_execute.v keys off. held_imm_div/held_imm_divs carry it.
//
// The immediate arrives sign-extended (gather_disp always does), which is
// right for MULS and DIVS and harmless for MULU and DIVU: the multiplier
// reads only bits [15:0], and the divider takes its divisor from the same
// low word.
wire is_mul_imm = mul_shape && (if_opcode[5:0] == 6'b111100);
wire is_div_imm = div_shape && (if_opcode[5:0] == 6'b111100);
wire is_muldiv_imm = is_mul_imm || is_div_imm;
wire is_muldiv_imm_signed = is_muldiv_imm && (if_opcode[8] == 1'b1);

// ir[6] is the SIZE, so it must not be pinned by the shape: 0 is Word and
// 1 is Long (milestone 67). MOVEM.W's load direction SIGN-EXTENDS each word
// into the whole 32-bit register rather than preserving the upper half,
// which is the behaviour worth testing -- the store direction simply writes
// the low half.
// The CONTROL modes (An) and (d16,An) join the autoincrement ones
// (milestone 68). They differ from -(An)/(An)+ in three ways at once, and
// all three are properties of the MODE rather than of the direction:
//
//   - no register writeback at all
//   - the address walks UPWARD, even for a store
//   - the mask is numbered bit 0 = D0, even for a store
//
// Only the PREDECREMENT store reverses the numbering and walks downward.
// Milestone 50 tied both of those to "is a store", which was indistinguish-
// able from the truth while -(An) was the only store mode.
wire movem_shape_st = (if_opcode[15:7] == 9'b010010001);
wire movem_shape_ld = (if_opcode[15:7] == 9'b010011001);
// (d16,PC) for the load direction and $xxx.W for both (milestone 72). Both
// are control modes -- no writeback, upward walk, bit 0 = D0 -- and both
// gather a SECOND word after the mask, so they ride the two-word gather
// (d16,An) already uses. $xxx.L would need a THIRD word and is not reached.
//
// The PC-relative base is the address of the DISPLACEMENT word. For every
// other PC-relative mode that is PC+2; for MOVEM the mask word sits in
// between, so it is PC+4. ap040_ea_fetch.v carries that +4 with a comment,
// because it is the one place the two MOVEM extension words change an
// address rather than just a length.
wire movem_mode_pcd = (if_opcode[5:3] == 3'b111) && (if_opcode[2:0] == 3'b010);
wire movem_mode_absw = (if_opcode[5:0] == 6'b111000);
// $xxx.L (milestone 73): mask plus a 32-bit address is a THREE-word gather,
// the first in this decoder. It costs less than it sounds. disp_acc shifts
// every gathered word in, so after the mask and the high address word it
// holds {mask, addr_hi} and the completing word is addr_lo -- gather_disp
// is then the whole address for free, and the mask is disp_acc[31:16].
// That is also why the mask now has its OWN field, id_movem_mask, for every
// MOVEM mode: packing it into id_imm's high half (milestone 68) only worked
// while nothing needed all 32 bits of id_imm for an address.
wire movem_mode_absl = (if_opcode[5:0] == 6'b111001);
// (d8,An,Xn) both ways and (d8,PC,Xn) for the load (milestone 115): a mask
// word and a brief word, and ap040_ea_fetch.v starts from ea_target, which
// has formed base + index + d8 since milestone 81.
wire movem_mode_idx   = (if_opcode[5:3] == 3'b110);
wire movem_mode_pcidx = (if_opcode[5:0] == 6'b111011);
wire movem_mode_ctl = (if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b101) ||
                      movem_mode_absw || movem_mode_absl || movem_mode_idx;
wire is_movem_st = movem_shape_st && ((if_opcode[5:3] == 3'b100) || movem_mode_ctl);
wire is_movem_ld = movem_shape_ld && ((if_opcode[5:3] == 3'b011) || movem_mode_ctl ||
                                      movem_mode_pcd || movem_mode_pcidx);
wire is_movem_x  = ((movem_shape_st || movem_shape_ld) && movem_mode_idx) ||
                   (movem_shape_ld && movem_mode_pcidx);
wire is_movem_pcrel = movem_shape_ld && movem_mode_pcd;
wire is_movem_absw  = (movem_shape_st || movem_shape_ld) && movem_mode_absw;
wire is_movem_absl  = (movem_shape_st || movem_shape_ld) && movem_mode_absl;
wire is_movem_w  = (if_opcode[6] == 1'b0);
// (d16,An) gathers a SECOND word after the mask, so it is a long gather --
// and id_imm then carries {mask, displacement} rather than the mask alone.
wire is_movem_disp = ((movem_shape_st || movem_shape_ld) && (if_opcode[5:3] == 3'b101)) ||
                     is_movem_pcrel || is_movem_absw || is_movem_x;
wire is_movem_down = movem_shape_st && (if_opcode[5:3] == 3'b100);
wire is_movem_wb   = (movem_shape_st && (if_opcode[5:3] == 3'b100)) ||
                     (movem_shape_ld && (if_opcode[5:3] == 3'b011));
wire is_movem    = is_movem_st || is_movem_ld;

wire is_link = (if_opcode[15:3] == 13'b0100111001010);
// LINK.L An,#d32 (milestone 113): the same instruction with a longword
// displacement, $4808+n. held_is_link carries both; held_is_long says which.
wire is_link_l = (if_opcode[15:3] == 13'b0100100000001);
// PACK/UNPK Dx,Dy,#adj (milestone 113): 1000 yyy 1p1 000 xxx with p=0 PACK,
// p=1 UNPK. Dx is operand A and the adjustment rides eac_ea_ext into
// operand B (ap040_ea_fetch.v); Dy is written at Byte (PACK) or Word (UNPK)
// and keeps its upper bits. No condition codes.
// -(Ax),-(Ay) (r/m = ir[3], milestone 117) is MOVE memory-to-memory with
// the two sizes different: a word read for PACK and a byte written, the
// other way for UNPK. The load's size is id_size; ap040_ea_fetch.v derives
// the store's from the op.
wire is_pack_rr  = (if_opcode[15:12] == 4'b1000) && (if_opcode[8:3] == 6'b101000);
wire is_unpk_rr  = (if_opcode[15:12] == 4'b1000) && (if_opcode[8:3] == 6'b110000);
wire is_pack_m   = (if_opcode[15:12] == 4'b1000) && (if_opcode[8:3] == 6'b101001);
wire is_unpk_m   = (if_opcode[15:12] == 4'b1000) && (if_opcode[8:3] == 6'b110001);
wire is_packunpk = is_pack_rr || is_unpk_rr || is_pack_m || is_unpk_m;
// RTD #d16 (milestone 113): RTS, then A7 += d16. The pop address is A7
// itself, so the displacement cannot ride eac_imm -- EA-fetch adds that to
// the address -- and goes in eac_ea_ext, which RTS's new-A7 sum adds.
wire is_rtd = (if_opcode == 16'h4E74);
wire is_unlk = (if_opcode[15:3] == 13'b0100111001011);

// Read-modify-write with a (d16,An) destination (milestone 55) --
// ADD.L D0,(8,A0), the struct-field update, and the most-used RMW mode.
//
// The DATAPATH needs nothing: milestone 48's sequencing already loads from
// ea_target and stores back to eaf_ea_target, and for mode 101 ea_target is
// already operand_a + the displacement. This is decode alone, riding
// milestone 40's gather kind with three more carried properties -- the
// fifth time that pattern has come up.
//
// One of those properties is the OP MAP itself, not just a flag. In the
// ir[8]=1 direction nibble 1011 is EOR, not CMP, so this kind can no longer
// take held_alu_op from alu_nib_op unconditionally.
// Indexed addressing, (d8,An,Xn) -- mode 110 (milestone 56). The array
// access: MOVE.L (0,A0,D1.L),D2 is what a compiler emits for a[i], and it
// is the largest addressing-mode gap left.
//
// It gathers ONE extension word like (d16,An) does, so it rides the same
// two gather kinds rather than adding more. What differs is what that word
// MEANS -- the brief format, not a displacement:
//
//   [15]    D/A for the index register    [11]   index size, 0=Word 1=Long
//   [14:12] index register number         [10:9] scale, 1/2/4/8
//   [7:0]   signed byte displacement
//
// So for this mode id_imm carries the extension word VERBATIM rather than a
// sign-extended displacement, and ap040_ea_fetch.v decodes it. Packing it
// that way is why no new per-stage fields were needed: the word already is
// the packed form.
//
// The index register needs a THIRD read port. An is on port A and the
// destination operand is on port B for everything except a plain load, so
// two ports genuinely do not reach.
wire ea_indexed_mode = (if_opcode[5:3] == 3'b110);

// PC-relative addressing (milestone 57): mode 111 reg 010 is (d16,PC) and
// reg 011 is (d8,PC,Xn). Amiga code is position-independent throughout, and
// LEA msg(pc),A0 is its signature idiom.
//
// Both gather exactly one extension word, and it is the SAME word the
// register-based modes gather -- a displacement for 010, a brief format for
// 011 -- so they ride the same gather kinds with one more carried property.
// All that differs is the BASE: the program counter of the extension word,
// which is the opcode's PC plus two, instead of An.
//
// That base is already available downstream as eac_pc, so nothing new is
// threaded; ap040_ea_fetch.v just selects it. Note this means the EA is NOT
// resolved at decode time even though the PC is known then -- keeping it in
// ap040_ea_fetch.v is what lets (d8,PC,Xn) share the index arithmetic
// milestone 56 built rather than needing its own.
wire ea_pcdisp_mode = (if_opcode[5:3] == 3'b111) && (if_opcode[2:0] == 3'b010);
wire ea_pcidx_mode  = (if_opcode[5:3] == 3'b111) && (if_opcode[2:0] == 3'b011);
wire ea_pcrel_mode  = ea_pcdisp_mode || ea_pcidx_mode;

wire alu_dst_disp_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b1) &&
                          (if_opcode[7:6]  != 2'b11) && (if_opcode[5:3] == 3'b101);
// The same ir[8]=1 memory-destination direction with an INDEXED address
// (milestone 111). The EA-fetch datapath has computed base+index+disp since
// milestone 81; only this decoder's per-family mode lists kept the direction
// to (An)/(An)+/-(An)/(d16,An). Indexed is 42.8% of everything the ALU
// block cannot decode.
wire alu_dst_idx_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b1) &&
                         (if_opcode[7:6]  != 2'b11) && ea_indexed_mode;
wire is_alu_dst_idx = alu_dst_idx_shape &&
                      ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                       (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                       (if_opcode[14:12] == 3'b101));
wire is_alu_dst_disp = alu_dst_disp_shape &&
                       ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                        (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                        (if_opcode[14:12] == 3'b101));

// (d8,An,Xn) and both PC-relative modes join (d16,An) here (milestone 112):
// they are the same one-extension-word gather, and held_ea_indexed /
// held_ea_pcrel below say which address it forms.
wire is_adda_disp = adda_shape && ((if_opcode[5:3] == 3'b101) || ea_indexed_mode ||
                                   ea_pcrel_mode);
// ...and at an absolute address, which is the other half of what the corpus
// had undecoded in these three families.
wire is_adda_abs   = adda_shape && abs_mode;
wire is_adda_abs_l = is_adda_abs && abs_long;
wire is_cmpa_abs   = is_adda_abs && (if_opcode[15:12] == 4'b1011);
wire is_cmpa_disp = is_adda_disp && (if_opcode[15:12] == 4'b1011);

// SUB and CMP are b - a, and ap040_ea_fetch.v resolves operand_b from
// eac_dest_reg and operand_a from the source, so `SUB Dn,Dm` computes
// Dm - Dn as it must. Verified against ap040_pipe_alu.v's sub_full.
// Single-operand forms on Dn: 0100 oooo SS 000 rrr. The register field is
// ir[2:0], NOT ir[11:9] like the binary family above, so dest_reg follows
// Scc's shape rather than ADD's -- and src_reg already defaults to ir[2:0],
// so operand_a and operand_b both resolve to the same Dn. That matters
// because the ALU is not consistent about which one a unary op reads: NOT,
// NEG and NEGX work on b, TST works on a (it shares MOVE's arm), and CLR
// reads neither. Pointing both at Dn covers all four without special cases.
//
// ir[7:6]=11 is excluded: for 0x4A that slot is TAS, a different
// instruction. 0x48 (SWAP/EXT/PEA/MOVEM) and 0x4E (the NOP/JMP/RTS misc
// group) are not in the enumerated list, so neither is disturbed.
wire unary_rr_shape = (if_opcode[15:12] == 4'b0100) &&
                      (if_opcode[7:6] != 2'b11) && (if_opcode[5:3] == 3'b000);
// The same five operations on MEMORY (milestone 58): CLR.L (A0), TST.B
// (A0), NOT/NEG/NEGX on a memory destination. CLR and TST in particular are
// everywhere -- zeroing a field, polling a flag -- and until now none of
// them could name anything but a data register.
//
// They split across two paths that already exist, and the split is not
// arbitrary: ap040_pipe_alu.v computes NOT/NEG/NEGX/CLR from operand B and
// TST from operand A. Milestone 48's read-modify-write crossover puts the
// loaded value in B, and the ordinary memory-source path puts it in A. So
// TST is a plain load with no store, and the other four are RMWs, with no
// new datapath either way.
//
// CLR still performs the read its result does not need. That matches the
// 68000 and 68010; the 68040 suppresses it. Harmless here, and noted rather
// than silently divergent.
wire unary_mem_shape = (if_opcode[15:12] == 4'b0100) && (if_opcode[7:6] != 2'b11) &&
                       ((if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b011) ||
                        (if_opcode[5:3] == 3'b100));
wire is_negx_mem = unary_mem_shape && (if_opcode[11:8] == 4'b0000);
wire is_clr_mem  = unary_mem_shape && (if_opcode[11:8] == 4'b0010);
wire is_neg_mem  = unary_mem_shape && (if_opcode[11:8] == 4'b0100);
wire is_not_mem  = unary_mem_shape && (if_opcode[11:8] == 4'b0110);
wire is_tst_mem  = unary_mem_shape && (if_opcode[11:8] == 4'b1010);
wire is_unary_mem = is_negx_mem || is_clr_mem || is_neg_mem || is_not_mem || is_tst_mem;
// Everything but TST writes memory back, so everything but TST is an RMW.
wire is_unary_rmw = is_unary_mem && !is_tst_mem;
// NEGX/CLR/NEG/NOT/TST at (d16,An) and (d8,An,Xn) (milestone 111).
// unary_mem_shape admits modes 010/011/100 and unary_abs_shape the
// absolutes; these two are what was between them.
wire unary_disp_shape = (if_opcode[15:12] == 4'b0100) && (if_opcode[7:6] != 2'b11) &&
                        (if_opcode[5:3] == 3'b101);
wire unary_idx_shape  = (if_opcode[15:12] == 4'b0100) && (if_opcode[7:6] != 2'b11) &&
                        ea_indexed_mode;
wire is_unary_gather  = ((unary_disp_shape || unary_idx_shape) &&
                         ((if_opcode[11:8] == 4'b0000) || (if_opcode[11:8] == 4'b0010) ||
                          (if_opcode[11:8] == 4'b0100) || (if_opcode[11:8] == 4'b0110) ||
                          (if_opcode[11:8] == 4'b1010))) ||
                        // TST at (d16,PC)/(d8,PC,Xn) (milestone 115). The PC
                        // shapes are defined below; this is a continuous
                        // assignment, so the order does not matter.
                        ((if_opcode[15:8] == 8'h4A) && (if_opcode[7:6] != 2'b11) &&
                         (if_opcode[5:0] == 6'b111010 || if_opcode[5:0] == 6'b111011));
// CLR and TST only, for now. TST reads and does not write back, and measured
// clean at zero wrong rounds when this was first written (milestone 111). The operand-dependent members of the family come out
// wrong through this carrier -- NEG by 6,402 corpus rounds and NOT by
// 6,210 -- because they need the MEMORY value as operand B and are not
// getting it, while CLR's result does not depend on the operand at all and
// so is right either way. They stay undecoded until that is understood:
// an instruction that traps is recoverable, one that quietly computes the
// wrong answer is not.
// TST reads and does not write back; the other four are read-modify-write.
wire is_unary_gather_tst = is_unary_gather && (if_opcode[11:8] == 4'b1010);
wire is_unary_gather_pc  = is_unary_gather && (if_opcode[5:3] == 3'b111);
wire is_unary_mem_pi = is_unary_mem && (if_opcode[5:3] == 3'b011);
wire is_unary_mem_pd = is_unary_mem && (if_opcode[5:3] == 3'b100);
// From the operation nibble alone, NOT from is_*_mem: those carry
// unary_mem_shape's three modes, so for the gathered (d16,An)/(d8,An,Xn)
// forms every one of them was false and NEG, NOT and NEGX all fell through
// to TST -- which reads a, the crossed-over data register, and stored it.
// That was the whole of the "operand B" defect milestone 111 held them
// back for (milestone 113).
wire [5:0] unary_mem_op = (if_opcode[11:8] == 4'b0000) ? `AP040_ALU_NEGX :
                          (if_opcode[11:8] == 4'b0010) ? `AP040_ALU_CLR  :
                          (if_opcode[11:8] == 4'b0100) ? `AP040_ALU_NEG  :
                          (if_opcode[11:8] == 4'b0110) ? `AP040_ALU_NOT  :
                                                         `AP040_ALU_TST;

wire is_negx_rr = unary_rr_shape && (if_opcode[11:8] == 4'b0000);   // 0x40
wire is_clr_rr  = unary_rr_shape && (if_opcode[11:8] == 4'b0010);   // 0x42
wire is_neg_rr  = unary_rr_shape && (if_opcode[11:8] == 4'b0100);   // 0x44
wire is_not_rr  = unary_rr_shape && (if_opcode[11:8] == 4'b0110);   // 0x46
wire is_tst_rr  = unary_rr_shape && (if_opcode[11:8] == 4'b1010);   // 0x4A
wire is_unary_rr = is_negx_rr || is_clr_rr || is_neg_rr || is_not_rr || is_tst_rr;

// SWAP / EXT.W / EXT.L / EXTB.L: 0100 100x oo 000 rrr. All three read
// operand b and take their register from ir[2:0], so they ride the unary
// group's selector arrangement. What they do NOT share is the size field:
// ir[7:6] here is an OPCODE selector, not std_size -- 01=SWAP, 10=EXT.W,
// 11=EXT.L with ir[8]=0 or EXTB.L with ir[8]=1 -- so add_op_size would read
// EXT.W as Long and EXT.L/EXTB.L as the invalid size 3. They get their own
// mapping instead.
//
// ir[7:6]=00 is NBCD and stays out. ir[5:3]!=000 is PEA/MOVEM, which shares
// this opcode and must not be disturbed.
wire extswap_shape = (if_opcode[15:12] == 4'b0100) &&
                     (if_opcode[11:9]  == 3'b100) && (if_opcode[5:3] == 3'b000);
wire is_swap_rr = extswap_shape && (if_opcode[8] == 1'b0) && (if_opcode[7:6] == 2'b01);
wire is_extw_rr = extswap_shape && (if_opcode[8] == 1'b0) && (if_opcode[7:6] == 2'b10);
wire is_extl_rr = extswap_shape && (if_opcode[8] == 1'b0) && (if_opcode[7:6] == 2'b11);
wire is_extb_rr = extswap_shape && (if_opcode[8] == 1'b1) && (if_opcode[7:6] == 2'b11);
wire is_extswap_rr = is_swap_rr || is_extw_rr || is_extl_rr || is_extb_rr;

wire [5:0] extswap_op = is_swap_rr ? `AP040_ALU_SWAP :
                        is_extb_rr ? `AP040_ALU_EXTB : `AP040_ALU_EXT;

// EXT.W is the only member whose result is a WORD spliced into Dn[15:0].
// SWAP returns {b[15:0],b[31:16]} and EXTB.L returns a sign-extended
// longword; both must pass through whole, so Long is what stops execute's
// merge from masking them.
wire [1:0] extswap_size = is_extw_rr ? `AP040_SZ_W : `AP040_SZ_L;

wire [5:0] unary_rr_op = is_negx_rr ? `AP040_ALU_NEGX :
                         is_clr_rr  ? `AP040_ALU_CLR  :
                         is_neg_rr  ? `AP040_ALU_NEG  :
                         is_not_rr  ? `AP040_ALU_NOT  :
                                      `AP040_ALU_TST;

// ADDX/SUBX, register form: 1ooo Rx 1 SS 00 0 Ry. This is the ir[8]=1 slot
// the binary family above excludes as "the Dn -> <ea> direction" -- but with
// a register-direct source that direction is not encodable (ADD Dn,Dn has no
// encoding), so the slot belongs to ADDX/SUBX instead. Destination is
// ir[11:9] and source ir[2:0], exactly as the binary family, so the default
// selectors already apply.
//
// Only 1101 and 1001 are enumerated, deliberately. 1011 with ir[8]=1 is EOR
// Dn,Dn, which IS a real encoding and must not be swallowed here, and 1100
// with ir[8]=1 is the ABCD group. ir[3]=1 would be the -(Ay),-(Ax) memory
// form, which has no path yet.
// Shift/rotate, immediate count, register destination:
// 1110 ccc d ss 0 tt rrr. ir[4:3] picks the family (00=arithmetic, 01=
// logical, 10=ROX, 11=RO) and ir[8] the direction, so eight operations come
// out of one predicate. The destination is ir[2:0] and the barrel reads
// operand b, so this rides the unary group's selector arrangement.
//
// ir[5]=1 puts the count in a second data register, needing a read port
// this pipeline has no path for; ir[7:6]=11 is the memory form, one bit at
// a time on an <ea>, and on 68020+ that slot is also the bitfield group.
// Both stay out.
// Bit operations, dynamic bit number: 0000 nnn 1 oo 000 rrr, with ir[7:6]
// picking BTST/BCHG/BCLR/BSET.
//
// These are the first instructions here whose operands are arranged the
// OPPOSITE way from the binary family. The bit NUMBER is ir[11:9] and the
// target is ir[2:0], and ap040_pipe_alu.v builds bit_mask from operand a
// while testing operand b -- so src_reg must be ir[11:9] and dest_reg
// ir[2:0]. Every other instruction so far takes its source from ir[2:0],
// which is why src_reg has never needed an override until now.
//
// A data-register target is always a LONG operation with the bit number
// taken mod 32 (the ALU's a[4:0]), so the default size applies. The
// byte-wide form belongs to a memory target and has no path yet; ir[8]=0 is
// the static form, whose bit number is an extension word; ir[5:3]=001 with
// this shape is MOVEP. All three stay out.
// ADDQ/SUBQ: 0101 qqq d SS 000 rrr. The immediate is IN the opcode, so
// unlike the ORI family below these need no gather at all -- they reuse the
// direct src_a_is_imm path MOVEQ has used since milestone 2, which is why
// they cost a predicate and nothing else.
//
// ir[8] picks SUBQ over ADDQ. ir[7:6]=11 is the Scc/DBcc slot that already
// owns this nibble and stays out; ir[5:3]!=000 is a memory or address
// destination, still with no path. The quick field is 1..7 literally with 0
// meaning EIGHT, the same encoding the shift count uses.
// MOVE #imm,Dn: 00 SS ddd 000 111 100. Source mode 111 reg 100 is the
// immediate addressing mode, so this reuses milestone 26's gather wholesale
// -- one extension word for byte and word, two for long -- with two
// differences from the ORI family. Its size lives in ir[13:12] with MOVE's
// own mapping rather than ir[7:6], and its destination is ir[11:9] rather
// than ir[2:0], which is what held_imm_dest9 selects at the completing end.
// MOVE.L (xxx).W,Dn and MOVE.L (xxx).L,Dn: source mode 111 with reg 000 or
// 001, the absolute addressing modes. These reuse BOTH the gather (one word
// sign-extended for .W, two for .L) and the memory-read path milestone 9b
// built for MOVE.L (An),Dn -- the only genuinely new thing is that the
// address has no register term at all.
//
// ap040_ea_fetch.v computes ea_target as operand_a + eac_imm for every
// memory access so far, which is right for (An) and (d16,An) and wrong here:
// there is no An to add. eac_is_abs drops the register term rather than
// trying to find a register that reads zero.
//
// Long only, matching the memory-source support that already exists; a
// sized memory read is a separate question this does not open.
wire is_move_abs = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                   (if_opcode[8:6] == 3'b000) &&
                   (if_opcode[5:3] == 3'b111) && (if_opcode[2:1] == 2'b00);
wire is_move_abs_l = is_move_abs && if_opcode[0];

// Absolute addressing for the ALU family and the unary ops (milestone 59):
// ADD.L $1234.L,D0 and CLR.L $1234.L. Mode 111 with reg 000 for the Word
// form and 001 for the Long one -- the only modes left whose extension
// words are the ADDRESS itself rather than an offset from something.
//
// They ride held_is_abs, which already gathers one word or two and already
// sets id_is_abs so ea_target is simply eac_imm. What it did not carry is
// an OPERATION: every absolute form until now was a MOVE. held_alu_op and
// held_alu_nowrite already exist for the displacement kinds, so this reuses
// them rather than adding more, and held_abs_rmw says whether the result
// goes back to that address.
//
// The absolute RMW composes without new datapath: id_is_abs makes
// ea_target eac_imm, and milestone 48's store half writes to
// eaf_ea_target, which is that same value. Nothing had to learn that an
// absolute address could also be a destination.
wire abs_mode   = (if_opcode[5:3] == 3'b111) && (if_opcode[2:1] == 2'b00);
wire abs_long   = if_opcode[0];

wire alu_abs_shape = (if_opcode[15]  == 1'b1)  && (if_opcode[8]  == 1'b0) &&
                     (if_opcode[7:6] != 2'b11) && abs_mode;
wire is_alu_abs = alu_abs_shape &&
                  ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                   (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                   (if_opcode[14:12] == 3'b101));
wire is_cmp_abs = alu_abs_shape && (if_opcode[14:12] == 3'b011);

wire unary_abs_shape = (if_opcode[15:12] == 4'b0100) && (if_opcode[7:6] != 2'b11) && abs_mode;
wire is_negx_abs = unary_abs_shape && (if_opcode[11:8] == 4'b0000);
wire is_clr_abs  = unary_abs_shape && (if_opcode[11:8] == 4'b0010);
wire is_neg_abs  = unary_abs_shape && (if_opcode[11:8] == 4'b0100);
wire is_not_abs  = unary_abs_shape && (if_opcode[11:8] == 4'b0110);
wire is_tst_abs  = unary_abs_shape && (if_opcode[11:8] == 4'b1010);
wire is_unary_abs = is_negx_abs || is_clr_abs || is_neg_abs || is_not_abs || is_tst_abs;
wire [5:0] unary_abs_op = is_negx_abs ? `AP040_ALU_NEGX :
                          is_clr_abs  ? `AP040_ALU_CLR  :
                          is_neg_abs  ? `AP040_ALU_NEG  :
                          is_not_abs  ? `AP040_ALU_NOT  :
                                        `AP040_ALU_TST;
// The ir[8]=1 memory-destination direction at an ABSOLUTE address
// (milestone 111), the sibling of is_alu_dst_idx above.
wire alu_dst_abs_shape = (if_opcode[15]  == 1'b1)  && (if_opcode[8]  == 1'b1) &&
                         (if_opcode[7:6] != 2'b11) && abs_mode;
wire is_alu_dst_abs = alu_dst_abs_shape &&
                      ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                       (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                       (if_opcode[14:12] == 3'b101));
// BTST/BCHG/BCLR/BSET beyond Dn (milestone 113).
//
// STATIC, 0000 1000 tt mmm rrr + a bit-number word: on Dn it is the
// register-destination immediate form (the number is operand A, Dn operand
// B, the ALU's bit ops already read them that way), and on memory it is
// the immediate-to-memory form milestone 89 built, with its extension word
// after the number exactly as ADDI's EA follows its immediate. Not folded
// into is_immmem, whose .L test is ir[7:6]=10 -- which here is BCLR, a
// one-word number, and would have gathered two.
//
// DYNAMIC, 0000 rrr 1 tt mmm rrr: the Dn,<ea> read-modify-write family's
// layout exactly -- number register at ir[11:9], operand at the EA -- so it
// rides the same three carriers (direct, held_is_alu_disp, held_is_abs).
//
// Memory operands are BYTES and the bit number is taken modulo 8;
// ap040_execute.v masks it, since the shared ALU uses a[4:0] as the FSM
// core's does. BTST writes nothing back: it is marked read-modify-write for
// the operand crossover alone, and ap040_ea_fetch.v drops its store.
wire bit_static_shape = (if_opcode[15:8] == 8'b0000_1000);
wire is_bit_btst  = (if_opcode[7:6] == 2'b00);
wire is_bit_s_rr  = bit_static_shape && ea_is_dn;
wire is_bit_s_mem = bit_static_shape && (ea_is_ind || ea_is_pi || ea_is_pd || ea_is_d16 ||
                                         ea_is_idx || ea_is_abs || (is_bit_btst && ea_is_pcrel));
wire is_bit_s_x   = is_bit_s_mem && (ea_ext_words != 2'd0);
wire bit_dyn_shape = (if_opcode[15:12] == 4'b0000) && (if_opcode[8] == 1'b1);
wire is_bit_d_mem = bit_dyn_shape && (ea_is_ind || ea_is_pi || ea_is_pd);
// BTST, and only BTST, also reads a PC-relative operand (milestone 115); it
// writes nothing, so the RMW carrier's dropped store is all it needs.
wire is_bit_d_gx  = bit_dyn_shape && (ea_is_d16 || ea_is_idx || (is_bit_btst && ea_is_pcrel));
// BTST Dn,#imm: the immediate is the DATA and Dn the bit number, the other
// way round from every other immediate form (milestone 115).
wire is_btst_dimm = bit_dyn_shape && is_bit_btst && ea_is_imm;
wire is_bit_d_abs = bit_dyn_shape && ea_is_abs;

// The other single-operand read-modify-writes on memory (milestone 114):
// the one-bit memory SHIFTS, 1110 0tt d 11 <ea>, always a word; TAS, $4AC0,
// and NBCD, $4800, both a byte. Each reads operand B in ap040_pipe_alu.v,
// which is where the RMW crossover puts the loaded value, so they ride the
// unary family's three carriers unchanged: direct for (An)/(An)+/-(An),
// held_is_alu_disp for (d16,An)/(d8,An,Xn), held_is_abs for the absolutes.
// None of them was decoded on memory at all; the memory shifts alone are
// 89,000 corpus rounds.
wire shm_shape  = (if_opcode[15:11] == 5'b11100) && (if_opcode[7:6] == 2'b11);
wire [5:0] shm_op =
	(if_opcode[10:9] == 2'b00) ? (if_opcode[8] ? `AP040_ALU_ASL1  : `AP040_ALU_ASR1)  :
	(if_opcode[10:9] == 2'b01) ? (if_opcode[8] ? `AP040_ALU_LSL1  : `AP040_ALU_LSR1)  :
	(if_opcode[10:9] == 2'b10) ? (if_opcode[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1) :
	                             (if_opcode[8] ? `AP040_ALU_ROL1  : `AP040_ALU_ROR1);
wire tas_shape  = (if_opcode[15:6] == 10'b0100_1010_11);
wire nbcd_shape = (if_opcode[15:6] == 10'b0100_1000_00);
wire ux_shape   = shm_shape || tas_shape || nbcd_shape;
wire [5:0] ux_op   = shm_shape ? shm_op : tas_shape ? `AP040_ALU_TAS : `AP040_ALU_NBCD;
wire [1:0] ux_size = shm_shape ? `AP040_SZ_W : `AP040_SZ_B;
wire is_ux_d    = ux_shape && (ea_is_ind || ea_is_pi || ea_is_pd);
wire is_ux_g    = ux_shape && (ea_is_d16 || ea_is_idx);
wire is_ux_abs  = ux_shape && ea_is_abs;

// MOVEA from a register other than a Long Dn, and from (An)/(An)+/-(An)
// (milestone 114). is_movea_rr only ever took MOVEA.L Dn,An. A Word source
// is sign-extended to all 32 bits (id_sxt_w, the ADDA.W path), and MOVEA
// sets no condition codes.
wire movea_shape   = (if_opcode[15:14] == 2'b00) &&
                     ((if_opcode[13:12] == 2'b11) || (if_opcode[13:12] == 2'b10)) &&
                     (if_opcode[8:6] == 3'b001);
wire movea_w       = (if_opcode[13:12] == 2'b11);
wire is_movea_reg  = movea_shape && (ea_is_dn || ea_is_an) && !is_movea_rr;
wire is_movea_memd = movea_shape && (ea_is_ind || ea_is_pi || ea_is_pd);

// MOVES <ea>,Rn / Rn,<ea> (milestone 114): 0000 1110 ss <ea> + a register
// word {A/D, reg, direction} ahead of the EA's own words. Privileged. The
// alternate function codes change nothing on a flat memory, so this is a
// MOVE whose register comes from the extension word and which sets no flags:
// a load writes Rn -- sign-extended to 32 bits when Rn is an An, as
// ap040_core.v's S_MOVES_RD does -- and a store writes Rn to <ea> through
// the MOVE store carriers. Only the alterable memory modes; the others stay
// illegal, which is what the reference raises for them even in user mode.
wire is_moves = (if_opcode[15:8] == 8'h0E) && (if_opcode[7:6] != 2'b11) &&
                (ea_is_ind || ea_is_pi || ea_is_pd || ea_is_d16 || ea_is_idx || ea_is_abs);

// MOVE to SR/CCR from every data mode, and MOVE from SR/CCR to Dn and
// every alterable memory mode (milestone 114). Until now only MOVE Dn,SR
// existed (is_movesr).
//
// TO: the ORI/ANDI/EORI-to-SR machinery with ALU_MOVE for the operation --
// the value replaces, as STOP's does. A register or memory source arrives
// in eaf_operand_a like any load's, an immediate through the gather as the
// immediate forms'. The SR form is privileged through eac_immsr_to_sr,
// which eac_is_priv_capable already counts.
//
// FROM: the Scc-to-memory carrier (milestone 107), and for the same reason:
// the value is the live status register, which is only exact in EX, where
// ccr_in is forwarded. ap040_execute.v builds the word and stores it (or
// merges it into Dn); the RMW read the carrier performs is unused. The SR
// form is privileged; the CCR form is not.
wire mvto_shape = (if_opcode[15:6] == 10'b0100_0110_11) || (if_opcode[15:6] == 10'b0100_0100_11);
wire mvto_sr    = if_opcode[9];                                   // $46C0 SR, $44C0 CCR
wire is_mvto_dn   = mvto_shape && !mvto_sr && ea_is_dn;          // MOVE Dn,SR is is_movesr
wire is_mvto_memd = mvto_shape && (ea_is_ind || ea_is_pi || ea_is_pd);
wire is_mvto_g    = mvto_shape && (ea_is_d16 || ea_is_idx || ea_is_abs || ea_is_pcrel);
wire is_mvto_imm  = mvto_shape && ea_is_imm;
wire mvf_shape  = (if_opcode[15:6] == 10'b0100_0000_11) || (if_opcode[15:6] == 10'b0100_0010_11);
wire mvf_ccr    = if_opcode[9];                                   // $42C0 CCR, $40C0 SR
wire is_mvf_dn    = mvf_shape && ea_is_dn;
wire is_mvf_memd  = mvf_shape && (ea_is_ind || ea_is_pi || ea_is_pd);
wire is_mvf_g     = mvf_shape && (ea_is_d16 || ea_is_idx || ea_is_abs);

// EXG (milestone 115): 1100 xxx 1 ooooo yyy, opmode 01000 Dx,Dy, 01001
// Ax,Ay, 10001 Dx,Ay. The main port writes Rx with Ry's value (ALU_MOVE of
// operand A), the second port -- (An)+'s -- writes Ry with Rx's, which port
// B read as the destination. No condition codes.
wire exg_dd  = (if_opcode[7:3] == 5'b01000);
wire exg_aa  = (if_opcode[7:3] == 5'b01001);
wire exg_da  = (if_opcode[7:3] == 5'b10001);
wire is_exg  = (if_opcode[15:12] == 4'b1100) && if_opcode[8] && (exg_dd || exg_aa || exg_da);
// TST, which alone of the unary family takes An (Word and Long), a
// PC-relative operand and an immediate on the 68020 and later (milestone
// 115).
wire tst_shape    = (if_opcode[15:8] == 8'h4A) && (if_opcode[7:6] != 2'b11);
wire is_tst_an    = tst_shape && ea_is_an && (if_opcode[7:6] != 2'b00);
wire is_tst_pcrel = tst_shape && ea_is_pcrel;
wire is_tst_imm   = tst_shape && ea_is_imm;

// MOVEP (milestone 115): 0000 ddd 1 oo 001 aaa + d16. oo = 00 .W and 01 .L
// from memory, 10 .W and 11 .L to memory. Alternate bytes -- (d16,Ay),
// +2, +4, +6 -- so it is a sequencer in ap040_ea_fetch.v, like MOVEM's,
// not a single access; decode only gathers the displacement and says which
// of the four it is. No condition codes; a Word load keeps Dx's upper half.
wire is_movep = (if_opcode[15:12] == 4'b0000) && if_opcode[8] && (if_opcode[5:3] == 3'b001);

// MULU.L/MULS.L and DIVU.L/DIVS.L (milestone 115): 0100 1100 0d <ea> +
// {0, Dl/Dq, signed, 64-bit, 000000000, Dh/Dr}, the register word ahead of
// the EA's words. Any data mode. EX multiplies (a registered product) or
// divides (the word divider, widened), writes Dl/Dq through the main port
// and Dh/Dr through the second, and ap040_ea_fetch.v reads Dr on port C for
// a 64-bit dividend. With an (An)+/-(An) source a two-register form
// writes THREE registers against two retire ports; ap040_execute.v writes
// the An step early, in a cycle its own stall leaves port 2 free
// (milestone 117).
wire is_ml = (if_opcode[15:7] == 9'b0100_1100_0) && !ea_not_data;

// The bitfield instructions (milestone 116): 1110 1ooo 11 <ea> + a word
// {0, Dn, Do, offset, Dw, width}. o = 0 BFTST 1 BFEXTU 2 BFCHG 3 BFEXTS
// 4 BFCLR 5 BFFFO 6 BFSET 7 BFINS. Dn or a control mode, and the four that
// only read also take the PC-relative modes. A sequencer in
// ap040_ea_fetch.v does the work -- offset and width from registers,
// a one- to five-byte memory window, four compute stages, the write-back --
// and decode only gathers the words; the extension word itself travels in
// id_ea_ext, which nothing else in these instructions uses.
// CHK2/CMP2 (milestone 117): 0000 0ss0 11 <ea> + {D/A, Rn, CHK2, 0...}, ss
// = 00 .B, 01 .W, 10 .L, at a control mode. Two bounds at ea and ea + size,
// read and compared by a sequencer in ap040_ea_fetch.v; Z and C only; CHK2
// out of bounds takes CHK's vector-6 entry.
wire is_ck2 = (if_opcode[15:11] == 5'b00000) && (if_opcode[8:6] == 3'b011) &&
              (if_opcode[10:9] != 2'b11) &&
              (ea_is_ind || ea_is_d16 || ea_is_idx || ea_is_abs || ea_is_pcrel);

// CAS Dc,Du,<ea> (milestone 117): 0000 1ss0 11 <ea> + {0, Du, 0, Dc}, ss =
// 01 .B, 10 .W, 11 .L, at a memory alterable mode. An ordinary load: Dc on
// port B as its destination; ap040_ea_fetch.v compares when the value
// arrives, reads Du on port C, and hands EX either a store of Du (equal) or
// a sized write of the value into Dc (not equal), with CMP's flags.
// MOVE16 (milestone 117): $F600-$F61F {(Ay)+ to (xxx).L, (xxx).L to (Ay)+,
// (Ay) to (xxx).L, (xxx).L to (Ay)} by ir[4:3], with the address in two
// extension words; $F620-$F627 (Ax)+,(Ay)+ with Ay in the one word's
// [14:12]. Sixteen bytes, both addresses aligned down to the line;
// ap040_ea_fetch.v copies them a longword at a time. id_m16 = {it is one,
// form}, form 4 the two-register one, as ap040_core.v numbers them.
wire is_m16_abs = (if_opcode[15:5] == 11'b11110110000);
wire is_m16_pp  = (if_opcode[15:3] == 13'b1111011000100);
wire is_m16     = is_m16_abs || is_m16_pp;

// CAS2 (milestone 117): $0CFC .W, $0EFC .L, then {Rn1, Du1, Dc1} and
// {Rn2, Du2, Dc2}. ap040_ea_fetch.v sequences it; id_imm carries both words.
wire is_cas2 = (if_opcode == 16'h0CFC) || (if_opcode == 16'h0EFC);

wire is_cas = (if_opcode[15:11] == 5'b00001) && (if_opcode[8:6] == 3'b011) &&
              (if_opcode[10:9] != 2'b00) &&
              (ea_is_ind || ea_is_pi || ea_is_pd || ea_is_d16 || ea_is_idx || ea_is_abs);

wire [2:0] bf_op    = if_opcode[10:8];
wire bf_ro          = (bf_op == 3'd0) || (bf_op == 3'd1) || (bf_op == 3'd3) || (bf_op == 3'd5);
wire is_bf          = (if_opcode[15:11] == 5'b11101) && (if_opcode[7:6] == 2'b11) &&
                      (ea_is_dn || ea_is_ind || ea_is_d16 || ea_is_idx || ea_is_abs ||
                       (bf_ro && ea_is_pcrel));

wire is_abs_alu = is_alu_abs || is_unary_abs || is_alu_dst_abs || is_bit_d_abs || is_ux_abs;

// The 8/9/b/c/d families with an IMMEDIATE source (milestone 111). ADD.L
// #imm,D0 has two encodings -- ADDI at 0x06xx and this one at 0xD0BC -- and
// only the first was decoded. There is no EA extension word here, just the
// immediate, so this rides the held_imm_* machinery with no field conflict.
wire alu_immsrc_shape = (if_opcode[15]  == 1'b1)  && (if_opcode[8]  == 1'b0) &&
                        (if_opcode[7:6] != 2'b11) && ea_is_imm;
wire is_alu_immsrc = alu_immsrc_shape &&
                     ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                      (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                      (if_opcode[14:12] == 3'b101));
wire is_alu_immsrc_cmp = is_alu_immsrc && (if_opcode[14:12] == 3'b011);
wire is_alu_immsrc_l   = is_alu_immsrc && (if_opcode[7:6] == 2'b10);
wire is_abs_alu_l = is_abs_alu && abs_long;

wire is_move_imm = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                   (if_opcode[8:6] == 3'b000) && (if_opcode[5:0] == 6'b111100);

wire quick_shape = (if_opcode[15:12] == 4'b0101) &&
                   (if_opcode[7:6] != 2'b11) && (if_opcode[5:3] == 3'b000);
wire is_subq_rr = quick_shape && (if_opcode[8] == 1'b1);
// ir[8] alone, so the two destinations added below share it: the direction
// bit means the same thing whatever the destination is.
wire [5:0] quick_op  = if_opcode[8] ? `AP040_ALU_SUB : `AP040_ALU_ADD;
wire [3:0] quick_val = (if_opcode[11:9] == 3'd0) ? 4'd8 : {1'b0, if_opcode[11:9]};

// ADDQ/SUBQ to an ADDRESS register and to MEMORY (milestone 90). The quick
// forms have reached only a data register since milestone 22, and both of
// the destinations they lack are ones compiled code leans on: ADDQ.L #4,A7
// and SUBQ.L #8,A7 are how a small stack adjustment is written, and
// ADDQ.L #1,(A0) is a counter that lives in memory.
//
// An destination (mode 001): Byte is ILLEGAL here. The size field still
// picks Word or Long in the encoding, but the operation is 32 bits wide
// either way and sets NO condition codes -- the same rule ADDA/SUBA follow
// (milestone 44). id_size is therefore forced Long rather than read from
// ir[7:6], and the Word form needs no sign extension, since the immediate
// is 1 to 8 and arrives zero-extended.
//
// Memory destination (modes 010/011/100): the read-modify-write milestone
// 89 built, with the immediate coming out of the opcode instead of a
// gathered extension word. id_immrmw does the same job here -- eac_imm is
// the OPERAND, so the address base must be left alone -- and
// id_src_a_is_imm stays clear, because operand_a is the address.
wire quick_an_shape  = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] != 2'b11) &&
                       (if_opcode[7:6] != 2'b00) && (if_opcode[5:3] == 3'b001);
// ADDQ/SUBQ to (d16,An), (d8,An,Xn) or an absolute address (milestone 112).
// The quick value lives in the OPCODE, which is gone by the time the gather
// completes, so held_quick_val carries it; the EA extension rides id_ea_ext.
wire is_quick_gx = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] != 2'b11) &&
                   (ea_is_d16 || ea_is_idx || ea_is_abs);
wire quick_mem_shape = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] != 2'b11) &&
                       ((if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b011) ||
                        (if_opcode[5:3] == 3'b100));
wire is_quick_mem_pi = quick_mem_shape && (if_opcode[5:3] == 3'b011);
wire is_quick_mem_pd = quick_mem_shape && (if_opcode[5:3] == 3'b100);
// Everything the three destinations genuinely share: the operation and the
// immediate. Size and every write consequence differ, so they are NOT on
// this wire.
wire quick_any       = quick_shape || quick_an_shape || quick_mem_shape;

// Immediate source: ORI/ANDI/SUBI/ADDI/EORI/CMPI, 0000 ooo 0 SS 000 rrr.
// The first instruction class here whose SOURCE is an extension word rather
// than a register, so it is also the first new class the gather machinery
// has had to carry since MOVEC.
//
// ir[8]=0 separates these from the dynamic bit group above, which shares the
// 0000 nibble. ir[11:9]=100 is the STATIC bit group (BTST #n and friends),
// whose extension word is a bit number rather than an operand, so it is not
// enumerated. ir[5:3]!=000 is a memory destination, still with no path.
//
// Byte and word immediates occupy one extension word, longs two, which is
// exactly the distinction held_is_long already draws for the long branches.
// gather_disp sign-extends the single-word form, harmless here because the
// ALU masks by size.
wire immop_shape = (if_opcode[15:12] == 4'b0000) && (if_opcode[8] == 1'b0) &&
                   (if_opcode[7:6] != 2'b11) && (if_opcode[5:3] == 3'b000);
// ORI/ANDI/EORI to CCR and to SR (milestone 62). ORI #$0700,SR is how
// interrupts get masked and ANDI #$F8FF,SR is how they come back, so these
// are not optional for code that runs with interrupts at all.
//
// They share immop_shape's nibble but not its mode field: 111/100 with size
// 00 addresses CCR and size 01 addresses SR, where immop_shape requires
// mode 000. Disjoint by construction, so the two families cannot collide.
//
// Only three of the six immediate operations are legal here -- ORI, ANDI
// and EORI. SUBI, ADDI and CMPI have no CCR or SR form, and admitting them
// would decode instructions a real 68040 rejects.
//
// The result does NOT go through the ALU. The operand is the status
// register, not a GPR, so ap040_execute.v computes it from eaf_sr_snapshot
// -- the live, forwarded SR already threaded there -- and commits it on the
// same exe_writes_sr path MOVE-to-SR and RTE use. The CCR form touches only
// the low byte, which is why it is computed separately rather than masked
// afterwards.
//
// The SR forms are PRIVILEGED; the CCR forms are not.
wire immsr_shape = (if_opcode[15:12] == 4'b0000) && (if_opcode[8] == 1'b0) &&
                   (if_opcode[5:0] == 6'b111100) &&
                   ((if_opcode[7:6] == 2'b00) || (if_opcode[7:6] == 2'b01)) &&
                   ((if_opcode[11:9] == 3'b000) || (if_opcode[11:9] == 3'b001) ||
                    (if_opcode[11:9] == 3'b101));
wire is_immsr    = immsr_shape;
wire is_immsr_sr = immsr_shape && (if_opcode[7:6] == 2'b01);

// STOP #imm (milestone 108). One opcode, and 1,048,576 corpus rounds -- the
// largest single item left after Scc. It is an immediate-to-SR with two
// differences: the SR is REPLACED rather than OR/AND/EOR'd, and afterwards
// the processor stops until an interrupt.
//
// It rides the immsr machinery because that already gathers one extension
// word, already writes SR, and above all is already in eac_is_priv_capable
// -- ap040_core.v:6029 decides go_priv before it even fetches the extension
// word, and every judged STOP round in the corpus is a user-mode one that
// must raise vector 8 rather than stop anything.
wire is_stop = (if_opcode == 16'h4E72);
wire [5:0] immsr_op = (if_opcode[11:9] == 3'b001) ? `AP040_ALU_AND :
                      (if_opcode[11:9] == 3'b101) ? `AP040_ALU_EOR :
                                                    `AP040_ALU_OR;

wire is_ori_i  = immop_shape && (if_opcode[11:9] == 3'b000);
wire is_andi_i = immop_shape && (if_opcode[11:9] == 3'b001);
wire is_subi_i = immop_shape && (if_opcode[11:9] == 3'b010);
wire is_addi_i = immop_shape && (if_opcode[11:9] == 3'b011);
wire is_eori_i = immop_shape && (if_opcode[11:9] == 3'b101);
wire is_cmpi_i = immop_shape && (if_opcode[11:9] == 3'b110);
wire is_imm_alu = is_ori_i || is_andi_i || is_subi_i || is_addi_i ||
                  is_eori_i || is_cmpi_i;

// ir[11:9] alone, so the memory-destination family below can share it:
// both shapes encode the operation in the same field and differ only in
// their mode bits, which is what makes them one family in the first place.
wire [5:0] imm_alu_op = (if_opcode[11:9] == 3'b000) ? `AP040_ALU_OR  :
                        (if_opcode[11:9] == 3'b001) ? `AP040_ALU_AND :
                        (if_opcode[11:9] == 3'b010) ? `AP040_ALU_SUB :
                        (if_opcode[11:9] == 3'b011) ? `AP040_ALU_ADD :
                        (if_opcode[11:9] == 3'b101) ? `AP040_ALU_EOR :
                                                      `AP040_ALU_CMP;

// ORI/ANDI/SUBI/ADDI/EORI/CMPI with a MEMORY destination (milestone 89):
// 0000 ooo 0 SS mmm rrr with mmm one of 010/011/100 -- the same three modes
// alu_dst_shape admits for the register-source direction, and the last
// decode gap tb_ap040_pipe_dual.v found at milestone 83.
//
// This is a read-modify-write, and the pipeline already has one: OR.B
// D0,(A0) loads, computes and stores through mem_issue/mem_complete and
// ap040_execute.v's ex_st_req. The only thing new is where the ALU's second
// operand comes from. That path crosses its operands over -- the loaded
// value must be the ALU's b, because SUB.L D0,(A0) is memory MINUS D0 --
// and puts the register source in a. Here a is the gathered immediate
// instead, which is one more mux on a REGISTERED assignment rather than a
// new sequencer.
//
// The collision to avoid is eac_imm: for every other gathered form it is a
// displacement ap040_ea_fetch.v ADDS to the base register, and here it is
// the operand itself. id_immrmw tells that stage to leave the base alone,
// and id_src_a_is_imm stays CLEAR so operand_a still reads An -- the
// opposite of the register-destination immediate forms, where operand_a is
// the immediate and no address exists at all.
//
// CMPI writes no memory, so it is a plain sized load plus a compare: the
// operand crossover still applies (memory minus immediate), but id_is_rmw
// stays clear and no store is ever requested.
// (d16,An), (d8,An,Xn) and the absolutes join the three register-indirect
// modes (milestone 112). A LONG immediate with an (xxx).L destination is
// four extension words, and disp_acc keeps only the two before the current
// one; disp_acc3, the word before those, holds the immediate's high half
// (milestone 117).
wire immmem_modes = ea_is_ind || ea_is_pi || ea_is_pd || ea_is_d16 || ea_is_idx ||
                    ea_is_absw || ea_is_absl;
wire immmem_shape = (if_opcode[15:12] == 4'b0000) && (if_opcode[8] == 1'b0) &&
                    (if_opcode[7:6] != 2'b11) && immmem_modes;
wire is_immmem    = immmem_shape &&
                    ((if_opcode[11:9] == 3'b000) || (if_opcode[11:9] == 3'b001) ||
                     (if_opcode[11:9] == 3'b010) || (if_opcode[11:9] == 3'b011) ||
                     (if_opcode[11:9] == 3'b101) || (if_opcode[11:9] == 3'b110));
wire is_immmem_pi = is_immmem && (if_opcode[5:3] == 3'b011);
wire is_immmem_pd = is_immmem && (if_opcode[5:3] == 3'b100);
wire is_cmpi_mem  = is_immmem && (if_opcode[11:9] == 3'b110);
// The forms above whose EA has an extension word of its own.
wire is_immx      = is_immmem && (ea_ext_words != 2'd0);
// CMPI, alone of the six, also compares against a PC-relative operand on
// the 68020 and later (milestone 115). It is the immediate-plus-EA form
// with one difference: the PC the displacement is relative to is the
// address of the DISPLACEMENT word, which the immediate has pushed two or
// four bytes past the usual opcode + 2 -- id_pc_off says how far.
wire is_cmpi_pc   = (if_opcode[15:8] == 8'h0C) && (if_opcode[7:6] != 2'b11) && ea_is_pcrel;

wire bitop_shape = (if_opcode[15:12] == 4'b0000) && (if_opcode[8] == 1'b1) &&
                   (if_opcode[5:3] == 3'b000);
wire is_btst_rr  = bitop_shape && (if_opcode[7:6] == 2'b00);

wire [5:0] bitop_op = (if_opcode[7:6] == 2'b00) ? `AP040_ALU_BTST :
                      (if_opcode[7:6] == 2'b01) ? `AP040_ALU_BCHG :
                      (if_opcode[7:6] == 2'b10) ? `AP040_ALU_BCLR :
                                                  `AP040_ALU_BSET;

// 1110 ccc d ss i tt rrr. Bit 5 is the count SOURCE: 0 an immediate in
// ccc, 1 the register ccc names (milestone 87). Size 11 is the
// one-bit-at-a-time memory form and is still not decoded.
wire shift_shape    = (if_opcode[15:12] == 4'b1110) && (if_opcode[7:6] != 2'b11);
wire shift_reg_cnt  = shift_shape && if_opcode[5];

wire [5:0] shift_op =
	(if_opcode[4:3] == 2'b00) ? (if_opcode[8] ? `AP040_ALU_ASL1  : `AP040_ALU_ASR1)  :
	(if_opcode[4:3] == 2'b01) ? (if_opcode[8] ? `AP040_ALU_LSL1  : `AP040_ALU_LSR1)  :
	(if_opcode[4:3] == 2'b10) ? (if_opcode[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1) :
	                            (if_opcode[8] ? `AP040_ALU_ROL1  : `AP040_ALU_ROR1);

// The immediate count field is 1..7 literally and 0 means EIGHT, not zero.
// A register count is whatever the register holds, modulo 64, and CAN be
// zero -- see ap040_pipe_alu.v's own note on that case.
// ap040_pipe_alu.v's barrel takes 1..63 and composes the one-bit steps in a
// single cycle, so nothing iterates here.
wire [5:0] shift_cnt = (if_opcode[11:9] == 3'd0) ? 6'd8 : {3'd0, if_opcode[11:9]};

wire addx_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b1) &&
                  (if_opcode[7:6]  != 2'b11) && (if_opcode[5:3] == 3'b000);
wire is_addx_rr = addx_shape && (if_opcode[14:12] == 3'b101);   // 1101
wire is_subx_rr = addx_shape && (if_opcode[14:12] == 3'b001);   // 1001
wire is_x_rr    = is_addx_rr || is_subx_rr;

// The memory forms, milestone 117: ADDX/SUBX/ABCD/SBCD -(Ay),-(Ax) and
// CMPM (Ay)+,(Ax)+ -- the same slots with ir[3] set. Two loads, source then
// destination, then the ordinary ALU op on them (b = the destination), the
// result stored back where the second load came from (none for CMPM). They
// ride MOVE memory-to-memory's machinery -- the source's step through port
// 2, the destination's address from its An on port B -- with id_mm[6]
// asking ap040_ea_fetch.v for the second load.
wire xm_shape  = if_opcode[15] && if_opcode[8] && (if_opcode[7:6] != 2'b11) && (if_opcode[5:3] == 3'b001);
wire is_addx_m = xm_shape && (if_opcode[14:12] == 3'b101);
wire is_subx_m = xm_shape && (if_opcode[14:12] == 3'b001);
wire is_cmpm   = xm_shape && (if_opcode[14:12] == 3'b011);
wire is_abcd_m = (if_opcode[15:12] == 4'b1100) && (if_opcode[8:3] == 6'b100001);
wire is_sbcd_m = (if_opcode[15:12] == 4'b1000) && (if_opcode[8:3] == 6'b100001);
wire is_xm     = is_addx_m || is_subx_m || is_cmpm || is_abcd_m || is_sbcd_m;
wire [5:0] xm_op = is_addx_m ? `AP040_ALU_ADDX : is_subx_m ? `AP040_ALU_SUBX :
                   is_abcd_m ? `AP040_ALU_ABCD : is_sbcd_m ? `AP040_ALU_SBCD : `AP040_ALU_CMP;

// ap040_pipe_alu.v computes b + a + X and b - a - X, and carries the 68000's
// sticky Z (cleared but never set) through f_z, so these need nothing from
// this stage beyond reaching them -- ccr_in is already wired to the ALU.
wire [5:0] x_rr_op = is_addx_rr ? `AP040_ALU_ADDX : `AP040_ALU_SUBX;

// ABCD/SBCD, register form: 1x00 Rx 1 00 00 0 Ry. These are the ir[7:6]=00
// members of the same ir[8]=1 slot ADDX/SUBX occupy, and like them take the
// destination from ir[11:9] and the source from ir[2:0]. BCD is byte-only
// by definition, and the ALU returns {24'd0, result[7:0]}, so Byte is what
// makes execute's merge preserve the destination's upper bits.
wire is_abcd_rr = addx_shape && (if_opcode[14:12] == 3'b100) && (if_opcode[7:6] == 2'b00);
wire is_sbcd_rr = addx_shape && (if_opcode[14:12] == 3'b000) && (if_opcode[7:6] == 2'b00);
wire is_bcd2_rr = is_abcd_rr || is_sbcd_rr;
wire [5:0] bcd2_op = is_abcd_rr ? `AP040_ALU_ABCD : `AP040_ALU_SBCD;

// NBCD Dn (0100 1000 00 000 rrr) and TAS Dn (0100 1010 11 000 rrr): the two
// slots the extswap and unary predicates deliberately left out, named there
// as NBCD and TAS. Both are single-operand on Dn, both read operand b, and
// both are Byte -- TAS returns {1'b1, b[6:0]}, so anything wider would let
// the merge overwrite Dn[31:8] instead of preserving it.
wire is_nbcd_rr = (if_opcode[15:12] == 4'b0100) && (if_opcode[11:8] == 4'b1000) &&
                  (if_opcode[7:6] == 2'b00) && (if_opcode[5:3] == 3'b000);
wire is_tas_rr  = (if_opcode[15:12] == 4'b0100) && (if_opcode[11:8] == 4'b1010) &&
                  (if_opcode[7:6] == 2'b11) && (if_opcode[5:3] == 3'b000);
wire is_bcd1_rr = is_nbcd_rr || is_tas_rr;
wire [5:0] bcd1_op = is_nbcd_rr ? `AP040_ALU_NBCD : `AP040_ALU_TAS;

// The same family with a MEMORY source: 1ooo RRR 0 SS 010 aaa, i.e. the
// binary shape above with ea mode 010 instead of register-direct. It reuses
// the memory-read path milestone 9b built, and needs no new operand
// arrangement: ap040_ea_fetch.v replaces eaf_operand_a with the loaded data,
// operand_b is still the destination Dn, and the ALU computes b op a -- so
// ADD.L (A0),D0 is D0 + memory, the right way round, for free.
//
// src_reg must name the ADDRESS register (unified index 8+n) so operand_a
// resolves to the address before the read replaces it, exactly as
// MOVE.L (An),Dn does.
//
// Modes 011 and 100 -- (An)+ and -(An) -- join mode 010 here rather than
// getting a shape of their own. They differ only in the address-register
// update, which ap040_ea_fetch.v already performs off eac_is_postinc/
// eac_is_predec through the second write port milestone 30 added for
// MOVE.L (An)+,Dn, and its an_write is deliberately independent of
// writes_reg -- so CMP.L (A0)+,D0, which writes no data register at all,
// still advances A0. That independence was already there; this is the first
// instruction that depends on it.
wire alu_mem_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b0) &&
                     (if_opcode[7:6]  != 2'b11) &&
                     ((if_opcode[5:3] == 3'b010) || (if_opcode[5:3] == 3'b011) ||
                      (if_opcode[5:3] == 3'b100));
wire is_alu_mem = alu_mem_shape &&
                  ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                   (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                   (if_opcode[14:12] == 3'b101));
wire is_cmp_mem = alu_mem_shape && (if_opcode[14:12] == 3'b011);
wire is_alu_pi  = is_alu_mem && (if_opcode[5:3] == 3'b011);
wire is_alu_pd  = is_alu_mem && (if_opcode[5:3] == 3'b100);

// And once more with ea mode 101, (d16,An). This one cannot be decoded in a
// single cycle -- the displacement is an extension word -- so unlike mode 010
// it is not a wire change but a new kind on the shared gather state machine,
// the eighth. It needs nothing the machine does not already hold: held_reg is
// An, held_dest_reg is Dn, held_mv_size is the size, and gather_disp is
// already the sign-extended displacement that MOVE.L (d16,An),Dn feeds into
// id_imm. Only the operation itself is new state (held_alu_op), because every
// prior gather kind had a fixed one.
wire alu_disp_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b0) &&
                      (if_opcode[7:6]  != 2'b11) && (if_opcode[5:3] == 3'b101);
wire alu_idx_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b0) &&
                     (if_opcode[7:6]  != 2'b11) && ea_indexed_mode;
wire alu_pcrel_shape = (if_opcode[15]   == 1'b1)  && (if_opcode[8]   == 1'b0) &&
                       (if_opcode[7:6]  != 2'b11) && ea_pcrel_mode;
wire is_alu_pcrel = alu_pcrel_shape &&
                    ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                     (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                     (if_opcode[14:12] == 3'b101));
wire is_cmp_pcrel = alu_pcrel_shape && (if_opcode[14:12] == 3'b011);
wire is_alu_idx = alu_idx_shape &&
                  ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                   (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                   (if_opcode[14:12] == 3'b101));
wire is_cmp_idx = alu_idx_shape && (if_opcode[14:12] == 3'b011);
wire is_alu_disp = alu_disp_shape &&
                   ((if_opcode[14:12] == 3'b000) || (if_opcode[14:12] == 3'b001) ||
                    (if_opcode[14:12] == 3'b011) || (if_opcode[14:12] == 3'b100) ||
                    (if_opcode[14:12] == 3'b101));
wire is_cmp_disp = alu_disp_shape && (if_opcode[14:12] == 3'b011);

// LEA <ea>,An (milestone 42): 0100 AAA 111 mmm rrr. Every function prologue
// and every array index uses it, and it is the cheapest useful instruction
// left -- it computes an address and writes it to An, touching no memory and
// no condition codes.
//
// The pipeline can already produce a computed address as an operand:
// ap040_ea_fetch.v routes ea_target into eaf_operand_a for JMP and JSR.
// LEA joins that ternary, so the whole instruction is a decode plus one
// flag threaded to that one line. With id_alu_op = MOVE the address then
// lands in An through the ordinary writeback.
//
// Mode 010 needs no gather (the address IS An, with id_imm forced to zero);
// mode 101 is the ninth kind on the shared gather state machine and reuses
// gather_disp exactly as MOVE.L (d16,An),Dn does. The absolute forms need
// no new work at all and are deliberately NOT added here: LEA (xxx).L,An is
// MOVEA.L #imm,An, which held_is_imm has assembled since milestone 26.
wire lea_shape   = (if_opcode[15:12] == 4'b0100) && (if_opcode[8:6] == 3'b111);
wire is_lea_an   = lea_shape && (if_opcode[5:3] == 3'b010);
wire is_lea_disp = lea_shape && (if_opcode[5:3] == 3'b101);
// LEA with an indexed EA (milestone 56). Beyond being common in its own
// right, this is the only way a bench can OBSERVE the computed address: a
// load cannot distinguish a sign-extended Word index from a zero-extended
// one, because this L1 wraps mod 8192 and 65536*scale is always a multiple
// of 8192, so both land on the same word. LEA puts the full 32-bit address
// in An where it can be read.
wire is_lea_idx  = lea_shape && ea_indexed_mode;
wire is_lea_pcrel = lea_shape && ea_pcrel_mode;

// PEA <ea> (milestone 60): 0100 1000 01 mmm rrr. Amiga library calls push
// their arguments, and a pointer argument is a PEA.
//
// It is LEA with a different destination: the same effective address, sent
// to the stack instead of to An. So it rides held_is_lea with one carried
// property, and the push itself is the BSR/JSR/LINK path -- eac_is_push,
// whose address is already operand_b - 4 once eac_dest_reg names A7. The
// only new wiring is the DATA: BSR pushes a return address and LINK pushes
// the old An, and PEA pushes the effective address itself.
//
// Absolute is deliberately NOT reached here. LEA never needed it, because
// LEA $xxx.L,An is MOVEA.L #imm,An, but PEA has no such equivalent, so it
// is a genuine gap rather than a redundant one -- it needs a push property
// on held_is_abs, which is the next step.
// CHK.W <ea>,Dn (milestone 64): 0100 DDD 110 mmm rrr. Bounds checking --
// trap vector 6 if Dn's low word is negative or exceeds the bound.
//
// Disjoint from everything else in nibble 0100 by ir[8], which opmode 110
// forces high: every unary selector (NEGX/CLR/NEG/NOT/TST) has it low, LEA
// is opmode 111, and PEA, TAS, MOVE-to/from-SR and MOVEM all sit at other
// ir[8:6] values.
//
// Its operand shape is the one that exposed milestone 52's defect: the
// BOUND comes from <ea>, which for a memory source is mem_lane and not
// operand_a. The fault is therefore derived from loaded data and must be
// LATCHED -- see ap040_ea_fetch.v.
//
// N is defined only on the two trapping paths: set when the value is
// negative, cleared when it merely exceeds the bound. It is written into
// the STACKED SR rather than the live one, which is what the handler
// actually observes. In the non-trapping case the 68k leaves N, Z, V and C
// undefined; this core leaves them unchanged.
// CHK.W is opmode 110 and CHK.L, which the 68020 added, is opmode 100.
// Only the word form was decoded -- 131,280 corpus rounds (milestone 111).
wire chk_is_long = (if_opcode[8:6] == 3'b100);
// CHK.L is on (milestone 113). Its 4,860 wrong rounds at milestone 111 were
// two things: the long immediate gathered ONE word -- held_is_long and the
// ext_pending list did not name it, so the bound's low half ran as the next
// instruction -- and the CHK flag rule every CHK lacked, fixed in 112.
wire chk_shape = (if_opcode[15:12] == 4'b0100) &&
                 ((if_opcode[8:6] == 3'b110) || (if_opcode[8:6] == 3'b100));
wire is_chk     = chk_shape && ((if_opcode[5:3] == 3'b000) || (if_opcode[5:3] == 3'b010) ||
                                (if_opcode[5:3] == 3'b011) || (if_opcode[5:3] == 3'b100));
wire is_chk_mem = is_chk && (if_opcode[5:3] != 3'b000);
wire is_chk_pi  = is_chk && (if_opcode[5:3] == 3'b011);
wire is_chk_pd  = is_chk && (if_opcode[5:3] == 3'b100);
wire is_chk_imm = chk_shape && (if_opcode[5:0] == 6'b111100);
// CHK.W with its bound at (d16,An), (d8,An,Xn) or PC-relative (milestone
// 112). Rides held_is_alu_disp: the checked Dn comes back as operand B
// from held_dest_reg exactly as for the direct forms, and the bound is the
// memory lane. Writes no register and no condition codes.
//
// Gated OFF for now: it decodes correctly but exposes the CHK flag defect
// every CHK in this core has -- a NON-trapping CHK must still write N and C
// (ap040_core.v:2899, verified against hardware), and this core writes no
// CCR on that path. 4,428 corpus rounds, all that signature.
wire chk_gather_en = 1'b1;   // on again with the flag fix (milestone 112)
wire is_chk_gather = chk_gather_en && chk_shape &&
                     ((if_opcode[5:3] == 3'b101) || ea_indexed_mode || ea_pcrel_mode);
// ...and at an absolute address (milestone 113), on the held_is_abs carrier
// MULU/DIVU abs already use: the bound is the memory lane, the checked Dn is
// operand B from ir[11:9], and nothing is written but the flags.
wire is_chk_abs   = chk_shape && abs_mode;
wire is_chk_abs_l = is_chk_abs && abs_long;

wire pea_shape = (if_opcode[15:12] == 4'b0100) && (if_opcode[11:6] == 6'b100001);
wire is_pea_an    = pea_shape && (if_opcode[5:3] == 3'b010);
wire is_pea_disp  = pea_shape && (if_opcode[5:3] == 3'b101);
wire is_pea_idx   = pea_shape && ea_indexed_mode;
wire is_pea_pcrel = pea_shape && ea_pcrel_mode;
wire is_pea_gather = is_pea_disp || is_pea_idx || is_pea_pcrel;

// LEA and PEA with an ABSOLUTE address (milestone 61). PEA $xxx.L is how a
// string constant gets pushed, and LEA $xxx.L,An is a real encoding a
// compiler may emit even though MOVEA.L #imm,An has the same effect --
// milestone 42 noted the equivalence and then left the opcode itself
// undecoded, which is a gap rather than a redundancy.
//
// Both ride held_is_abs, which already gathers the address and already sets
// id_is_abs so ea_target is eac_imm. The property they add is that the
// instruction delivers the ADDRESS rather than the contents: no memory read
// at all, and no condition codes. That distinction milestone 59 did not
// need, because every absolute form it reached did read memory.
wire is_lea_abs = lea_shape && abs_mode;
wire is_pea_abs = pea_shape && abs_mode;
wire is_eaonly_abs   = is_lea_abs || is_pea_abs;
wire is_eaonly_abs_l = is_eaonly_abs && abs_long;

// One op map for all three shapes: the nibble alone picks the operation.
wire [5:0] alu_nib_op = (if_opcode[14:12] == 3'b000) ? `AP040_ALU_OR  :
                        (if_opcode[14:12] == 3'b001) ? `AP040_ALU_SUB :
                        (if_opcode[14:12] == 3'b011) ? `AP040_ALU_CMP :
                        (if_opcode[14:12] == 3'b100) ? `AP040_ALU_AND :
                                                       `AP040_ALU_ADD;

wire [5:0] alu_rr_op = is_or_rr  ? `AP040_ALU_OR  :
                       is_sub_rr ? `AP040_ALU_SUB :
                       is_cmp_rr ? `AP040_ALU_CMP :
                       is_and_rr ? `AP040_ALU_AND :
                                   `AP040_ALU_ADD;

wire [1:0] add_op_size  = if_opcode[7:6];
wire [1:0] move_op_size = (if_opcode[13:12] == 2'b01) ? `AP040_SZ_B :
                          (if_opcode[13:12] == 2'b11) ? `AP040_SZ_W : `AP040_SZ_L;

// Bcc family (BRA included as cc==0000; cc==0001 is BSR, excluded -- needs
// a stack push, not this milestone). The displacement byte selects which
// form: a real byte value is the short form (this cycle is a complete
// instruction); 8'h00 selects one 16-bit extension word; 8'hFF selects two
// (a 32-bit displacement, 68020+).
wire is_branch_opcode = (if_opcode[15:12] == 4'b0110) && (if_opcode[11:8] != 4'h1);
wire is_branch_byte   = is_branch_opcode && (if_opcode[7:0] != 8'h00) && (if_opcode[7:0] != 8'hFF);
wire is_branch_word   = is_branch_opcode && (if_opcode[7:0] == 8'h00);
wire is_branch_long   = is_branch_opcode && (if_opcode[7:0] == 8'hFF);

// BSR: the excluded cc==0001 case of the SAME 0x6 opcode class -- see
// header. Same byte/word/long form selection as Bcc.
wire is_bsr_opcode = (if_opcode[15:12] == 4'b0110) && (if_opcode[11:8] == 4'h1);
wire is_bsr_byte   = is_bsr_opcode && (if_opcode[7:0] != 8'h00) && (if_opcode[7:0] != 8'hFF);
wire is_bsr_word   = is_bsr_opcode && (if_opcode[7:0] == 8'h00);
wire is_bsr_long   = is_bsr_opcode && (if_opcode[7:0] == 8'hFF);

// Scc.B Dn: 0101 cccc 11 000 rrr (cccc = condition, mode 000 = data-
// register-direct dest at rrr). Verified against ap040_core.v:4745-4776's
// own ADDQ/SUBQ/Scc/DBcc decode: ir[7:6]==11 with mode 000 selects Scc
// (mode 001 is DBcc, mode 111+d_rn 010-100 is TRAPcc -- both excluded here
// by construction, not by explicit exclusion). No other destination EA
// mode yet -- register-direct only, same scope discipline as every other
// instruction so far.
wire scc_opcode = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] == 2'b11);
wire is_scc_rr = scc_opcode && ea_is_dn;

// Scc <ea> (milestone 107), the first family written against the shared
// classifier instead of its own shape wire. ap040_core.v:6075-6097 sorts the
// whole 0101 cccc 11 space in one pass: mode 001 is DBcc, mode 111 with reg
// 010-100 is TRAPcc, mode 000 is Scc to Dn, and anything else is Scc to
// memory unless the destination is not alterable, which is illegal.
//
// ea_not_alt is the same predicate as that core's dst_not_alt, and it
// already excludes BOTH of the other two readings -- DBcc's mode 001 is An
// direct, TRAPcc's three encodings are mode 7 above (xxx).L -- so the memory
// arm needs no separate exclusion for either, and cannot silently steal an
// encoding from them. That is the whole point of a mode CLASS: the family
// says what it takes, and the reserved encodings are excluded by
// construction rather than by a list somebody has to keep in step.
wire is_scc_mem        = scc_opcode && !ea_is_dn && !ea_not_alt;
wire is_scc_mem_direct = is_scc_mem && (ea_ext_words == 2'd0);
wire is_scc_mem_gather = is_scc_mem && (ea_ext_words != 2'd0);
wire is_scc_mem_pi     = is_scc_mem_direct && ea_is_pi;
wire is_scc_mem_pd     = is_scc_mem_direct && ea_is_pd;

// DBcc Dn: 0101 cccc 11 001 rrr (cccc = condition, mode 001 = DBcc -- mode
// 000 is Scc above, mode 111+d_rn 010-100 is TRAPcc, both excluded here by
// construction). Verified against the same ap040_core.v:5415-5421 decode
// as the header comment. rrr (d_rn) is the loop-counter register, both read
// and written -- unlike Scc's dest, there is no separate "src" field.
wire is_dbcc = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] == 2'b11) &&
               (if_opcode[5:3]  == 3'b001);

// MOVE.L (An),Dn: 0010 DDD 000 010 aaa (DDD = dest Dn at ir[11:9], aaa =
// source An at ir[2:0]). ir[15:12]==0010 alone already selects size=Long
// within the MOVE class (ir[15:14]=00 is fixed for MOVE, ir[13:12]=10=Long
// is baked into 4'b0010 -- no separate size check needed, unlike
// is_move_rr's raw-'10' note above). Verified against ap040_core.v's own
// MOVE decode (ap040_core.v:5021-5052): d_op8_6==000 selects DK_REG
// (register-direct dest) there, d_mode==010 selects its SK_MEM/(An) source
// -- rtl_old routes (An) through its fully general src_mode_r/src_rn_r EA
// machinery, which covers every addressing mode uniformly. This decoder
// special-cases JUST register-indirect for now, since ITS effective address
// needs no arithmetic at all: EA = An's value, resolved through the exact
// same regfile-port-A / EX-forward path register-direct source operands
// already use (id_src_reg = An's UNIFIED index, 8+n, same 4-bit space
// ap040_pipe_regfile.v already uses for A0-A6) -- see ap040_ea_fetch.v's
// header for the mechanism that turns that resolved value into an actual
// memory access rather than an ALU operand.
// MOVE.B/.W/.L (An),Dn (milestone 35). Was Long only: ir[15:12]==0010 IS
// the long size, so widening it to ir[13:12]!=00 is what admits byte and
// word. Everything else about the instruction is unchanged -- the L1 always
// returns a full longword on port B, so a sized load is lane selection on
// the way out, not a different access. See mem_lane in ap040_ea_fetch.v.
//
// The other load modes stay Long for now: (An)+ and -(An) would also need
// their step to follow the size (1/2/4 rather than always 4), which is a
// separate change from the lane select this proves.
wire is_move_mem_l = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                      (if_opcode[8:6]  == 3'b000) &&
                      (if_opcode[5:3]  == 3'b010);

// MOVE.L (An)+,Dn and MOVE.L -(An),Dn (milestone 30). Same shape as the
// plain (An) form above, with source mode 011 or 100 instead of 010, and
// they reuse its memory-read path unchanged. What is new is that they write
// a SECOND register: the updated An alongside the data in Dn. See
// ap040_pipe_regfile.v's second write port and ap040_ea_fetch.v's an_new.
wire is_move_pi = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                  (if_opcode[8:6] == 3'b000) && (if_opcode[5:3]  == 3'b011);
wire is_move_pd = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                  (if_opcode[8:6] == 3'b000) && (if_opcode[5:3]  == 3'b100);
wire is_move_ax = is_move_pi || is_move_pd;

// MOVE.L Dn,(An) (milestone 31): the first STORE. Destination mode 010 with
// a register-direct source, so the data is Dn at ir[2:0] and the address is
// An at ir[11:9].
//
// dest_reg is {1'b1, ir[11:9]} rather than {1'b0, ...}: it names an ADDRESS
// register, so operand_b resolves to An and the store takes its address from
// there. ea_target is operand_a + eac_imm and operand_a is the DATA here,
// which is why ap040_ea_fetch.v gives the store its own arm in l1_addr_word
// rather than reusing ea_target.
//
// Nothing is written to a register -- id_writes_reg stays 0 -- but MOVE sets
// N and Z from the data, so id_writes_ccr does not.
// The store forms take Dn OR An as their source (milestone 112). MOVE.B
// An,<ea> is the one exception -- a byte cannot come from an address
// register -- and stays illegal. The data register is {ir[3], ir[2:0]}.
wire move_st_src_ok = (if_opcode[5:3] == 3'b000) ||
                      ((if_opcode[5:3] == 3'b001) && (if_opcode[13:12] != 2'b01));
wire is_move_st_an = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                     (if_opcode[8:6] == 3'b010) && move_st_src_ok;

// ... and the same store to (An)+ / -(An) (milestone 32). This is where
// milestone 30's address update meets milestone 31's store path, and the two
// take their address register from OPPOSITE operands: a load's An is the
// source at ir[2:0], a store's is the destination at ir[11:9]. See
// ap040_ea_fetch.v's an_base.
wire is_move_st_pi = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                     (if_opcode[8:6] == 3'b011) && move_st_src_ok;
wire is_move_st_pd = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                     (if_opcode[8:6] == 3'b100) && move_st_src_ok;
wire is_move_st = is_move_st_an || is_move_st_pi || is_move_st_pd;

// ... and the same store to (d16,An) (milestone 91). Every compiled
// function writes its locals this way, and destination mode 101 was the
// last one the MOVE family could not reach: loads have had it since
// milestone 10 and the ALU family since milestone 40.
//
// It is the ELEVENTH gather kind and needs no new operand plumbing. A
// store already takes its data from operand_a (the Dn at ir[2:0], through
// eac_src_reg) and its address from operand_b (the An at ir[11:9], through
// eac_dest_reg) -- read from opposite ends of the opcode, as
// is_move_st_pi's comment above explains. All that is new is the offset on
// the address, and ap040_ea_fetch.v's store branch was already adding one
// for the predecrement mode, so id_st_disp selects a third value there
// rather than putting a second adder on the L1 address path.
//
// Source mode 000 only, matching the three store modes above: MOVE.L
// An,(d16,A6) and memory-to-memory MOVE have no path in this decoder for
// ANY destination mode, so this one does not invent one.
//
// It gathers and does NOT branch, which is the trap recorded against
// milestone 40: redirect_from_gather fires for every gather kind that is
// not on its exclusion list, and a store that forgot to join it would
// jump to held_pc + 2 + the displacement.
// ...and to (d8,An,Xn) (milestone 114): the same gather, with
// held_ea_indexed saying the word is a brief extension. ea_target already
// forms base + index + d8 for the loads, and this store takes the load's
// adder (see ap040_ea_fetch.v's l1_addr_word).
wire is_move_st_disp = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                       ((if_opcode[8:6] == 3'b101) || (if_opcode[8:6] == 3'b110)) &&
                       move_st_src_ok;

// MOVEA.L (milestone 33): destination mode 001, so the destination is an
// ADDRESS register. Two source forms, the two that matter most here --
// MOVEA.L Dn,An and MOVEA.L #imm,An. The immediate form is what finally
// lets a program load an address register, which until now only a
// testbench poking areg[] directly could do.
//
// MOVEA sets NO condition codes, unlike every other MOVE. That is the one
// thing about it that is easy to get wrong and invisible in a register
// check, so it has its own assertion in the testbench.
// MOVEA with an ABSOLUTE source (milestone 111): 190,018 corpus rounds
// between the Word and Long forms. Like MOVEA everywhere else it writes An,
// sets no condition codes, and the Word form sign-extends to all 32 bits.
// .W and .L only. There is no MOVEA.B: a byte move with an An destination
// is illegal, and "!= 2'b00" swept 432 of them in as MOVEA.
wire is_movea_abs   = (if_opcode[15:14] == 2'b00) &&
                      ((if_opcode[13:12] == 2'b11) || (if_opcode[13:12] == 2'b10)) &&
                      (if_opcode[8:6] == 3'b001) && abs_mode;
wire is_movea_abs_w = is_movea_abs && (if_opcode[13:12] == 2'b11);
wire is_movea_absl  = is_movea_abs && abs_long;
// MOVEA at (d16,An) and (d8,An,Xn) is ADDA-at-a-displacement with MOVE for
// the operation: held_alu_areg and held_alu_sxt already do everything else.
wire is_movea_gather = (if_opcode[15:14] == 2'b00) &&
                       ((if_opcode[13:12] == 2'b11) || (if_opcode[13:12] == 2'b10)) &&
                       (if_opcode[8:6] == 3'b001) &&
                       ((if_opcode[5:3] == 3'b101) || ea_indexed_mode || ea_pcrel_mode);
wire is_movea_gather_w = is_movea_gather && (if_opcode[13:12] == 2'b11);
wire is_movea_rr  = (if_opcode[15:12] == 4'b0010) && (if_opcode[8:6] == 3'b001) &&
                    (if_opcode[5:3] == 3'b000);
// MOVE.L Dn,(xxx).W and MOVE.L Dn,(xxx).L (milestone 34): an absolute
// STORE. Destination mode 111 with reg 000 or 001, register-direct source.
// This is the first store whose address comes from the gather rather than a
// register, so it is the first to need held state -- the same shape
// held_is_imm uses, carrying "this is a store" and the DATA register across
// the extension words.
//
// The data register is ir[2:0], which held_reg already captures.
// The destination's mode is ir[8:6] and its REGISTER field is ir[11:9], the
// mirror of a source's ir[5:3]/ir[2:0]. So absolute short is reg 000 and
// absolute long is reg 001, and ir[9] is what separates them -- not ir[0],
// which is where the same distinction lives for a source operand.
// All three sizes, and Dn or An as the source (milestone 112); it was
// MOVE.L Dn only. The size comes from held_mv_size's MOVE default.
wire is_st_abs = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                 (if_opcode[8:6] == 3'b111) &&
                 (if_opcode[11:10] == 2'b00)   && move_st_src_ok;
wire is_st_abs_l = is_st_abs && if_opcode[9];

// MOVEA.W #imm as well as .L (milestone 112): one word, which gather_disp
// already sign-extends to the 32 bits MOVEA.W writes.
wire is_movea_imm = (if_opcode[15:14] == 2'b00) &&
                    ((if_opcode[13:12] == 2'b10) || (if_opcode[13:12] == 2'b11)) &&
                    (if_opcode[8:6] == 3'b001) && (if_opcode[5:0] == 6'b111100);
wire is_movea_imm_l = is_movea_imm && (if_opcode[13:12] == 2'b10);

// MOVE.L (d16,An),Dn: 0010 DDD 000 101 aaa -- same shape as is_move_mem_l
// above, mode field 101 instead of 010 (verified against the same
// ap040_core.v:5021-5052 MOVE decode -- d_mode==101 is SK_MEM there too,
// just a different EA mode within the same general machinery). Always
// carries exactly one 16-bit extension word (the displacement) -- routed
// through the shared gather state machine below, not a parallel one.
wire is_move_idx = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                   (if_opcode[8:6] == 3'b000) && ea_indexed_mode;
wire is_move_pcrel = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                     (if_opcode[8:6] == 3'b000) && ea_pcrel_mode;
wire is_move_disp = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                     (if_opcode[8:6]  == 3'b000) &&
                     (if_opcode[5:3]  == 3'b101);

// MOVE memory-to-memory (milestone 114): every data source mode but the two
// registers -- the immediate included -- to every alterable memory mode.
// 464,000 corpus rounds, the largest block left.
//
// It is one instruction, not two micro-ops. The SOURCE is an ordinary MOVE
// load: eac_src_reg is its An, eac_imm its displacement, brief word or
// absolute, and its (An)+/-(An) update rides the second write port as for
// any load. The DESTINATION is the read-modify-write store with a different
// address: decode points eac_dest_reg at the destination An, which every
// load already reads on port B and nobody used, id_ea_ext carries the
// destination's own extension, and ap040_ea_fetch.v forms that address when
// the load completes and hands it to EX as eaf_ea_target. EX stores the
// ALU_MOVE of the loaded value there, and a destination (An)+/-(An) writes
// its updated An through the main port, which a store leaves free.
//
// The source's extension words arrive first. held_mm_src_ext captures them
// as the last one arrives, because disp_acc only keeps the two words before
// the current one and a (xxx).L -> (xxx).L MOVE is four.
//
// id_mm is {mm, source immediate, destination (An)+, -(An), absolute,
// indexed}; a destination that is none of those is (An) or (d16,An), whose
// extension is zero or the displacement.
wire [2:0] mvd_mode = if_opcode[8:6];
wire mvd_ind   = (mvd_mode == 3'b010);
wire mvd_pi    = (mvd_mode == 3'b011);
wire mvd_pd    = (mvd_mode == 3'b100);
wire mvd_d16   = (mvd_mode == 3'b101);
wire mvd_idx   = (mvd_mode == 3'b110);
wire mvd_absw  = (mvd_mode == 3'b111) && (if_opcode[11:9] == 3'b000);
wire mvd_absl  = (mvd_mode == 3'b111) && (if_opcode[11:9] == 3'b001);
wire mvd_mem   = mvd_ind || mvd_pi || mvd_pd || mvd_d16 || mvd_idx || mvd_absw || mvd_absl;
wire [1:0] mvd_words = (mvd_d16 || mvd_idx || mvd_absw) ? 2'd1 : mvd_absl ? 2'd2 : 2'd0;
wire mvs_mem   = ea_is_ind || ea_is_pi || ea_is_pd || ea_is_d16 || ea_is_idx || ea_is_abs ||
                 ea_is_pcrel;
wire [1:0] mvs_words = ea_is_imm ? ((if_opcode[13:12] == 2'b10) ? 2'd2 : 2'd1) : ea_ext_words;
wire is_move_mm   = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] != 2'b00) &&
                    (mvs_mem || ea_is_imm) && mvd_mem;
wire is_move_mm_d = is_move_mm && (mvs_words == 2'd0) && (mvd_words == 2'd0);
wire is_move_mm_g = is_move_mm && !is_move_mm_d;
wire [2:0] mm_words = {1'b0, mvs_words} + {1'b0, mvd_words};

// JMP <ea>: 0100 1110 11 mmm rrr -- see header. Only the two EA modes this
// decoder already resolves elsewhere (An), (d16,An) are recognized; every
// other legal JMP mode (indexed, absolute, PC-relative) falls through to
// id_unimpl, same scope discipline as every prior instruction.
wire is_jmp_opcode = (if_opcode[15:6] == 10'b0100111011);
wire is_jmp_an     = is_jmp_opcode && (if_opcode[5:3] == 3'b010);
wire is_jmp_disp   = is_jmp_opcode && (if_opcode[5:3] == 3'b101);

// JSR <ea>: 0100 1110 10 mmm rrr -- see header. Same two EA modes as JMP.
wire is_jsr_opcode = (if_opcode[15:6] == 10'b0100111010);
wire is_jsr_an     = is_jsr_opcode && (if_opcode[5:3] == 3'b010);
wire is_jsr_disp   = is_jsr_opcode && (if_opcode[5:3] == 3'b101);

// JMP/JSR with indexed, PC-relative and absolute targets (milestone 71).
// JSR (d16,PC) is how position-independent code calls anything nearby,
// JSR $xxx.L is the absolute call, and JMP (d8,PC,Xn) is a jump table.
//
// Every path they need exists: ea_base for PC-relative, idx_val for the
// index, eac_imm for absolute, the push through eac_dest_reg = A7, and the
// odd-target address error off ea_target. So the indexed and PC-relative
// forms ride held_is_jmp/held_is_jsr with the held_ea_* properties LEA
// already carries, and the absolute forms ride held_is_abs with two more
// properties beside held_abs_lea and held_abs_push. Decode only.
wire is_jmp_idx    = is_jmp_opcode && ea_indexed_mode;
wire is_jmp_pcrel  = is_jmp_opcode && ea_pcrel_mode;
wire is_jmp_abs    = is_jmp_opcode && abs_mode;
wire is_jsr_idx    = is_jsr_opcode && ea_indexed_mode;
wire is_jsr_pcrel  = is_jsr_opcode && ea_pcrel_mode;
wire is_jsr_abs    = is_jsr_opcode && abs_mode;
wire is_jmp_gather = is_jmp_disp || is_jmp_idx || is_jmp_pcrel;
wire is_jsr_gather = is_jsr_disp || is_jsr_idx || is_jsr_pcrel;
wire is_jmpjsr_abs   = is_jmp_abs || is_jsr_abs;
wire is_jmpjsr_abs_l = is_jmpjsr_abs && abs_long;

// TRAP #n: 0100 1110 0100 nnnn (0x4E40-0x4E4F) -- see header. Distinct
// if_opcode[15:6] value (10'b0100111001) from both JSR's (...010) and JMP's
// (...011), so no overlap is possible with either.
wire is_trap = (if_opcode[15:4] == 12'h4E4);

// TRAPcc (milestone 74): 0101 cccc 11 111 opm -- vector 7 if the condition
// holds, otherwise a NOP. opm 100 is the bare form, 010 carries one
// extension word and 011 two; the immediate is not used by the hardware,
// it is left on the stack for the handler. Scc is mode 000 and DBcc mode
// 001, so mode 111 was unclaimed in this opcode space.
//
// The condition is evaluated in ap040_ea_fetch.v, where exceptions start
// and where the live CCR already arrives as sr_in -- not in EX, where
// Bcc/Scc/DBcc evaluate theirs, because by EX it is too late to push a
// frame. held_cond already captures cccc for the gathering forms.
wire trapcc_shape     = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:3] == 5'b11111);
wire is_trapcc        = trapcc_shape && (if_opcode[2:0] == 3'b100);
wire is_trapcc_w      = trapcc_shape && (if_opcode[2:0] == 3'b010);
wire is_trapcc_l      = trapcc_shape && (if_opcode[2:0] == 3'b011);
wire is_trapcc_gather = is_trapcc_w || is_trapcc_l;

// RTS / RTE (milestone 16, new): the "other half" of BSR/JSR's push and
// illegal/TRAP/priv's exception-entry push, respectively -- both are pure
// A7-pop instructions with no register field in the opcode at all (unlike
// every OTHER A7-touching instruction so far), so id_dest_reg/id_src_reg
// are both hardcoded to A7 (4'd15) rather than derived from if_opcode
// bits. RTS reuses ap040_ea_fetch.v's existing mem_issue/mem_complete FSM
// verbatim (id_is_mem_src=1, same as MOVE.L (An),Dn) -- its "read" is a
// single 32-bit pop, structurally identical to a memory-source MOVE
// reading from (An), just with An hardcoded to A7 and the popped value
// becoming a PC redirect (ex_recovery_pc) instead of a GPR result. RTE
// needs genuinely new machinery instead (id_is_mem_src stays 0 for it):
// TWO reads (SR+PC, then format), not one, plus a DYNAMIC privilege
// check (RTE is supervisor-only, verified against ap040_core.v's `if
// (!sr_s) go_priv` on its own dispatch) that must take priority over
// attempting the pop at all -- see ap040_ea_fetch.v's header for the new
// sequencer and ap040_execute.v's for the SR-restore commit path.
//
// MOVE to SR, register-direct source only: 0100 0110 11 000 rrr
// (0x46C0-0x46C7) -- see header. Privileged; the actual privilege check is
// DYNAMIC (this stage has no visibility into the live S bit), so it happens
// in ap040_ea_fetch.v against the forwarded SR -- see its header. bits[15:6]
// (10'b0100011011) share no prefix with JMP's/JSR's/TRAP's own 0x4Exx
// ranges, so no collision is possible with any of them.
wire is_movesr = (if_opcode[15:6] == 10'b0100011011) && (if_opcode[5:3] == 3'b000);

// MOVEC Rc,Rn / MOVEC Rn,Rc: 0100 1110 0111 101d (0x4E7A read-direction,
// 0x4E7B write-direction, d = if_opcode[0]) -- verified against
// ap040_core.v's own S_MOVEC1/S_MOVEC2 dispatch (`ir[0]` selects mvc_dir the
// same way). Always carries exactly one 16-bit extension word (the
// register/selector field), routed through the shared gather state machine
// below like every other word-form instruction -- see header.
wire is_movec_opcode = (if_opcode[15:1] == 15'b010011100111101);

// MOVEC's extension-word fields -- meaningful only during the gather-
// completion cycle (if_opcode holds the extension word by then, not the
// original opcode), harmless otherwise, same "compute always" precedent as
// gather_disp below. Selector validity restricted to what this core
// actually models this milestone (SFC/DFC/CACR/VBR/USP/ISP/MSP) -- every
// other real MOVEC target (TC/ITT0/ITT1/DTT0/DTT1/URP/SRP/MMUSR) is MMU
// state, explicitly out of scope per the user's own framing ("don't need to
// worry about MMU registers yet"); an otherwise-valid MOVEC naming one of
// those is treated as illegal (vector 4), matching ap040_core.v's own
// movec_valid() gate on go_illegal, not silently accepted or dropped.
//
// Milestone 113 takes the MMU registers in as STORAGE: TC, ITT0/1, DTT0/1,
// MMUSR, URP and SRP read back what was written, through the write masks
// ap040_core.v applies (ap040_core.v:3850-3870), and translate nothing --
// this core has no MMU. That is the whole of what the corpus asks of them:
// 130,760 MOVEC rounds read one, and every one trapped as illegal. The set
// is now exactly ap040_core.v's movec_valid().
wire [11:0] movec_raw_sel = xw[11:0];
wire        movec_sel_valid = (movec_raw_sel == 12'h000) || (movec_raw_sel == 12'h001) ||
                               (movec_raw_sel == 12'h002) || (movec_raw_sel == 12'h003) ||
                               (movec_raw_sel == 12'h004) || (movec_raw_sel == 12'h005) ||
                               (movec_raw_sel == 12'h006) || (movec_raw_sel == 12'h007) ||
                               (movec_raw_sel == 12'h800) || (movec_raw_sel == 12'h801) ||
                               (movec_raw_sel == 12'h803) || (movec_raw_sel == 12'h804) ||
                               (movec_raw_sel == 12'h805) || (movec_raw_sel == 12'h806) ||
                               (movec_raw_sel == 12'h807);
wire  [3:0] movec_sel_code = (movec_raw_sel == 12'h000) ? `AP040_CREG_SFC   :
                              (movec_raw_sel == 12'h001) ? `AP040_CREG_DFC   :
                              (movec_raw_sel == 12'h002) ? `AP040_CREG_CACR  :
                              (movec_raw_sel == 12'h003) ? `AP040_CREG_TC    :
                              (movec_raw_sel == 12'h004) ? `AP040_CREG_ITT0  :
                              (movec_raw_sel == 12'h005) ? `AP040_CREG_ITT1  :
                              (movec_raw_sel == 12'h006) ? `AP040_CREG_DTT0  :
                              (movec_raw_sel == 12'h007) ? `AP040_CREG_DTT1  :
                              (movec_raw_sel == 12'h801) ? `AP040_CREG_VBR   :
                              (movec_raw_sel == 12'h800) ? `AP040_CREG_USP   :
                              (movec_raw_sel == 12'h804) ? `AP040_CREG_ISP   :
                              (movec_raw_sel == 12'h805) ? `AP040_CREG_MMUSR :
                              (movec_raw_sel == 12'h806) ? `AP040_CREG_URP   :
                              (movec_raw_sel == 12'h807) ? `AP040_CREG_SRP   :
                                                            `AP040_CREG_MSP;   // 12'h803
wire  [3:0] movec_gpr = {xw[15], xw[14:12]};

// RTS/RTE (milestone 16): single fixed opcodes, no gather, no register
// field at all -- unlike every other A7-touching instruction so far, the
// register these decrement/increment isn't encoded anywhere in the
// opcode, it's always A7. RTS: 0100 1110 0111 0101 (0x4E75). RTE:
// 0100 1110 0111 0011 (0x4E73) -- one bit different from RTS (bit1),
// verified against ap040_core.v's own casez arms (`6'b110101`/
// `6'b110011` within the same 0x4E7x "misc" group MOVEC also lives in).
// RTE is privileged (`if (!sr_s) go_priv` in ap040_core.v); RTS is not --
// see ap040_ea_fetch.v's header for where that dynamic check actually
// happens, same mechanism MOVEC/MOVE-to-SR already established.
wire is_rts = (if_opcode == 16'h4E75);

// TRAPV, MOVE USP and RESET (milestone 117). TRAPV is TRAPcc's bare form
// with the condition fixed at VS: vector 7 when V is set, format 2, the
// frame TRAPcc already builds. MOVE An,USP / MOVE USP,An ($4E60-$4E6F) are
// MOVEC to and from USP without an extension word, so they inherit MOVEC's
// privilege check -- and its T0 rule, which traces the write direction
// only, the same narrowing ap040_core.v's t0_special records for MOVE
// An,USP. RESET is privileged and otherwise does nothing this core can
// see: the bus reset it drives reaches nothing on this fabric.
wire is_trapv    = (if_opcode == 16'h4E76);
wire is_move_usp = (if_opcode[15:4] == 12'h4E6);
wire usp_to      = !if_opcode[3];            // MOVE An,USP
wire is_reset    = (if_opcode == 16'h4E70);
// RTR (milestone 117) is an RTE here: the same two pops -- {CCR, PC high}
// then {PC low, the next word} -- off A7, told apart downstream by
// id_is_rtr. See ap040_ea_fetch.v's RET sequencer for where they part.
wire is_rtr = (if_opcode == 16'h4E77);
wire is_rte = (if_opcode == 16'h4E73) || is_rtr;

wire is_nop = (if_opcode == `AP040_OP_NOP);

// Illegal instruction: precisely what none of the above (nor NOP/MOVEQ/
// MOVE.L/ADD.L/Bcc/Scc below) recognizes. Computed once here, as a single
// wire, rather than repeating the same ten-term negation at both id_unimpl's
// old two use sites (the gather-completion branch, which is never illegal by
// construction, and the single-word final-else branch, which is the only
// place this actually varies) -- see header for why this now drives a real
// exception instead of a silent bubble. is_movec_opcode is excluded for
// documentation clarity, not correctness -- it always routes into the
// gather-start branch instead, so this wire is never actually consulted for
// it, but an invalid MOVEC selector DOES become illegal, one level down
// (movec_illegal_gather below), once the extension word is known.
// ------------------------------------------------------------------ FPU
// F-line coprocessor 1 (2026-09-24): ap040_pipe_fpu.v runs these from
// EA-fetch; decode recognises them and gathers their words. The kind is
// ir[8:6], as in ap040_core.v's cpID-1 dispatch: 000 general and 001
// FScc/FDBcc/FTRAPcc carry a command (condition) word first; 010/011 are
// FBcc with a word or long displacement; 100 FSAVE and 101 FRESTORE have
// only their EA's words. 110/111, and a general or conditional form whose
// mode-7 register is 5..7, stay F-line exceptions, taken before any word
// is fetched, as the reference rejects them.
wire [2:0] fp_kind      = if_opcode[8:6];
wire       fp_bad7      = (if_opcode[5:3] == 3'b111) && (if_opcode[2:0] > 3'b100);
wire       is_fp        = (if_opcode[15:9] == 7'b1111_001) && (fp_kind <= 3'b101) &&
                          !((fp_kind[2:1] == 2'b00) && fp_bad7);
wire       fp_nocmd     = fp_kind[2] || fp_kind[1];
wire       fp_is_dbcc   = (fp_kind == 3'b001) && (if_opcode[5:3] == 3'b001);
wire       fp_is_trapcc = (fp_kind == 3'b001) && (if_opcode[5:3] == 3'b111) && (if_opcode[2:0] >= 3'b010);
// The EA field is an address: not FBcc's (its condition), nor FDBcc's or
// FTRAPcc's, whose mode-1 and mode-7 2..4 encodings name no EA.
wire       fp_has_ea    = is_fp && !(fp_kind == 3'b010 || fp_kind == 3'b011) && !fp_is_dbcc && !fp_is_trapcc;
// Words before the command word is known. A general form takes its command
// word alone and then as many more as that word asks for (fp_more): the
// reference consumes an EA's words only for the opclasses that use it, and
// an immediate's length is the command word's format.
wire [2:0] fp_words     = (fp_kind == 3'b010) ? 3'd1 :
                          (fp_kind == 3'b011) ? 3'd2 :
                          fp_kind[2]          ? {1'b0, ea_ext_words} :
                          fp_is_dbcc          ? 3'd2 :
                          fp_is_trapcc        ? ((if_opcode[2:0] == 3'b010) ? 3'd2 :
                                                 (if_opcode[2:0] == 3'b011) ? 3'd3 : 3'd1) :
                          (fp_kind == 3'b001) ? (3'd1 + {1'b0, ea_ext_words}) : 3'd1;
function [2:0] fp_after_cmd;
	input [15:0] c;
	input  [1:0] eaw;     // the EA's own extension words
	input        eaimm;   // the EA is #<data>
	reg    [2:0] sel;
	begin
		sel = (c[12:10] == 3'd0) ? 3'b001 : c[12:10];
		case (c[15:13])
			3'b000, 3'b001: fp_after_cmd = 3'd0;   // FPm,FPn; undefined
			3'b010: fp_after_cmd = (c[12:10] == 3'd7) ? 3'd0 :      // FMOVECR
			                       !eaimm ? {1'b0, eaw} :
			                       (c[12:10] == 3'd0 || c[12:10] == 3'd1) ? 3'd2 :
			                       (c[12:10] == 3'd4 || c[12:10] == 3'd6) ? 3'd1 :
			                       (c[12:10] == 3'd5) ? 3'd4 : 3'd6;
			3'b100: fp_after_cmd = !eaimm ? {1'b0, eaw} :
			                       (({2'b00, sel[2]} + {2'b00, sel[1]} + {2'b00, sel[0]}) << 1);
			default: fp_after_cmd = eaimm ? 3'd0 : {1'b0, eaw};   // stores, FMOVEM: an immediate faults
		endcase
	end
endfunction

wire is_aline = (if_opcode[15:12] == 4'hA);
// CINV/CPUSH (2026-09-24): F4xx with a scope; scope 00 stays an F-line
// exception, classified ahead of privilege as ap040_core.v does. The whole
// field goes on -- caches (bits 7-6: IC, DC), push (5), scope (4-3: line,
// page, all) -- with An, the physical address of a line or page, read as
// the source (caches stage B); EA-fetch runs it at the caches as it runs
// PFLUSH at the MMU.
wire is_cinv  = (if_opcode[15:8] == 8'hF4) && (if_opcode[4:3] != 2'b00);
// PFLUSH (F500-F51F) and PTEST (F548-F54F write, F568-F56F read), for
// rtl/ap040/ap040_mmu.v beside the bus (2026-09-24). The rest of PTEST's
// quadrant stays F-line, classified ahead of privilege as ap040_core.v has it.
wire is_pflush = (if_opcode[15:5] == 11'b1111_0101_000);
wire is_ptest  = (if_opcode[15:8] == 8'hF5) && (if_opcode[7:6] == 2'b01) && (if_opcode[4:3] == 2'b01);
wire is_pmmu   = is_pflush || is_ptest;
wire is_fline = (if_opcode[15:12] == 4'hF) && !is_m16 && !is_fp && !is_cinv && !is_pmmu;
wire is_illegal = !is_nop && !is_cinv && !is_pmmu && !is_xm && !is_trapv && !is_move_usp && !is_reset && !is_moveq && !is_move_rr && !is_alu_rr && !is_alu_mem && !is_an_src && !is_adda && !is_eor_rr && !is_alu_dst && !is_unary_mem && !is_chk && !is_chk_imm && !is_trapcc && !is_unlk && !is_link && !is_movem && !is_mul && !is_div && !is_muldiv_imm && !is_alu_dst_disp && !is_alu_dst_idx && !is_unary_gather && !is_move_idx && !is_alu_idx && !is_lea_idx &&
                   !is_move_pcrel && !is_alu_pcrel && !is_lea_pcrel && !is_abs_alu && !is_muldiv_abs && !is_movea_abs && !is_mul_gather && !is_div_gather && !is_movea_gather && !is_adda_abs && !is_chk_gather && !is_quick_gx && !is_pea_an && !is_pea_gather && !is_eaonly_abs && !is_unary_rr && !is_extswap_rr && !is_x_rr && !shift_shape && !bitop_shape && !is_bcd1_rr && !is_bcd2_rr && !is_imm_alu && !is_alu_immsrc && !is_immmem && !is_move_imm && !is_move_abs && !is_move_ax && !is_move_st && !is_move_st_disp && !is_movea_rr && !is_movea_imm && !is_st_abs && !quick_shape && !quick_an_shape && !quick_mem_shape &&
                   !is_branch_byte && !is_scc_rr && !is_scc_mem_direct && !is_stop && !is_move_mem_l &&
                   !is_jmp_an && !is_bsr_byte && !is_jsr_an && !is_trap &&
                   !is_movesr && !is_movec_opcode && !is_rts && !is_rte && !is_lea_an &&
                   !is_immsr && !is_chk_abs && !is_bit_s_rr && !is_bit_s_mem && !is_bit_d_mem &&
                   !is_bit_d_gx && !is_packunpk && !is_link_l && !is_rtd && !is_move_mm &&
                   !is_ux_d && !is_ux_g && !is_movea_reg && !is_movea_memd && !is_moves &&
                   !is_mvto_dn && !is_mvto_memd && !is_mvto_g && !is_mvto_imm &&
                   !is_mvf_dn && !is_mvf_memd && !is_mvf_g &&
                   !is_exg && !is_tst_an && !is_tst_imm && !is_btst_dimm && !is_cmpi_pc && !is_movep &&
                   !is_ml && !is_bf && !is_ck2 && !is_cas && !is_m16 && !is_cas2 && !is_fp;

// Gather state -- see header comment. 0 = idle/normal decode cycle;
// 1 = this cycle's if_opcode completes the gather; 2 = one more word
// needed after this one (only ever set to 2 at gather start, long form).
// held_is_dbcc/held_reg (milestone 8) distinguish a DBcc gather from a
// Bcc.W gather sharing this same state machine; held_is_move_disp/
// held_dest_reg (milestone 10) add a third kind, MOVE.L (d16,An),Dn. held_reg
// is "the extra register field" generically -- An for move-disp, the loop
// counter for DBcc, meaningless when neither flag is set; held_dest_reg is
// move-disp-only (Dn), meaningless otherwise.
// held_is_jmp (milestone 11) adds a fourth kind: JMP (d16,An)'s gather. It
// needs held_reg (An) but not held_dest_reg (JMP writes no register).
// held_is_bsr/held_is_jsr (milestone 13) add a fifth and sixth kind:
// BSR.W/L reuses held_is_long/held_cond exactly like a real Bcc (its
// condition field is unused architecturally but costs nothing to carry);
// JSR (d16,An) needs held_reg (An), same as JMP's gather, but always
// writes A7 (4'd15) rather than reading held_dest_reg the way move-disp's
// Dn destination does.
// held_is_movec (milestone 15) adds a seventh kind: MOVEC's gather needs no
// held_reg at all (the extension word carries BOTH the register selector
// AND the general-register field itself -- see movec_gpr/movec_sel_code
// above, read directly off if_opcode at gather completion, not shifted into
// disp_acc). It needs one thing nothing else does: the DIRECTION bit,
// which lives in the ORIGINAL opcode word (if_opcode[0], read/write), not
// the extension word -- captured into held_movec_dir at gather START,
// before if_opcode stops meaning the opcode at all.
reg  [2:0]  ext_pending;
reg         held_is_imm;
reg         held_imm_mem;
reg         held_imm_mem_pi;
reg         held_imm_mem_pd;
reg  [5:0]  held_imm_op;
reg  [1:0]  held_imm_size;
reg         held_imm_nowrite;
reg         held_imm_dest9;
reg         held_abs_lea;       // absolute, delivering the address to An
reg         held_abs_push;      // absolute, pushing the address
reg         held_abs_jmp;       // absolute, and the address is a JMP target
reg         held_abs_jsr;       // absolute, and the address is a JSR target
reg         held_abs_alu;       // this absolute form carries an operation, not just a MOVE
reg         held_abs_rmw;
reg         held_abs_div;      // this absolute form is a DIVU/DIVS
reg         held_abs_divs;     // ...and it is the signed one
reg         held_abs_sxt;      // its word operand sign-extends to 32 bits
reg         held_abs_areg;     // ...and its destination is An (MOVEA)       // ...and writes its result back to that address
reg         held_is_abs;
reg         held_imm_areg;
reg         held_is_stabs;
reg         held_st_disp;
// A word or long form branch. It gathers like everything else and then
// redirects, so nothing downstream needed its displacement until the
// odd-target check did (milestone 97) -- and id_imm was handing it zero.
reg         held_is_branch;
// (d16,An) and absolute loads gather, so unlike (An)/(An)+/-(An) their size
// cannot be read off if_opcode at the completing end -- it is held here.
reg  [1:0]  held_mv_size;
reg         held_is_long;
reg         held_is_xlong;      // three extension words (MOVEM $xxx.L)
reg  [2:0]  held_ext_n;         // how many extension words this gather collects
reg         held_is_dbcc;
reg         held_is_move_disp;
// The eighth gather kind (milestone 40). held_alu_op is the first gather
// state that carries an OPERATION: every earlier kind had a fixed one, so
// id_alu_op could be chosen from the kind flags alone.
reg         held_is_alu_disp;
// Scc to a gathered destination (milestone 107). The condition already rides
// held_cond and the EA register already rides held_reg, so the family adds
// only "this is an Scc" and "its destination was absolute".
reg         held_is_scc_mem;
reg         held_st_only;       // CLR or Scc: written, not read (see id_st_only)
reg         held_is_stop;
reg         held_scc_abs;
reg         held_is_lea;        // the ninth kind: LEA (d16,An),Am
reg         held_is_movem;      // the eleventh kind: MOVEM.L
reg         held_movem_dir;
reg         held_movem_word;
reg         held_movem_down;
reg         held_movem_wb;
reg         held_movem_pcrel;
reg         held_movem_abs;
reg         held_imm_div;       // this immediate form is a DIVIDE, not an ALU op
reg         held_imm_divs;
reg         held_lea_push;      // this LEA-shaped form pushes instead of writing An
reg         held_is_trapcc;     // TRAPcc.W / TRAPcc.L: the immediate is ignored
reg         held_is_link;       // the tenth kind: LINK An,#d16
reg         held_imm_chk;       // this immediate form is a CHK bound
reg         held_imm_chk_l;     // ...and it is the Long form
reg         held_chk_l;         // any gathered CHK that is CHK.L
reg         held_is_pack;       // PACK/UNPK Dx,Dy,#adj
reg         held_pack_unpk;     // ...UNPK rather than PACK
reg         held_pack_mem;      // ...-(Ax),-(Ay)
reg         held_is_rtd;        // RTD #d16
reg         held_mm;            // MOVE memory-to-memory
reg         held_mm_simm;       // ...whose source is an immediate
reg         held_mm_spi;        // ...whose source is (An)+
reg         held_mm_spd;        // ...or -(An)
reg         held_mm_sabs;       // ...or absolute
reg         held_mm_sbrief;     // ...or indexed (a brief extension word)
reg  [1:0]  held_mm_swords;     // source extension words
reg  [1:0]  held_mm_dwords;     // destination extension words
reg         held_mm_dpi, held_mm_dpd, held_mm_dabs, held_mm_didx;
reg  [31:0] held_mm_src_ext;    // the source's extension, captured as it completes
reg         held_moves;         // MOVES
reg  [1:0]  held_moves_size;
reg         held_moves_pi, held_moves_pd, held_moves_disp, held_moves_abs, held_moves_idx;
reg         held_mvto, held_mvto_sr, held_mvf, held_mvf_ccr, held_mvx_abs;   // MOVE to/from SR/CCR
reg  [1:0]  held_pc_off;        // words of immediate ahead of a PC-relative displacement
reg         held_movep, held_movep_long, held_movep_wr;
reg         held_ml, held_ml_div, held_ml_dn, held_ml_imm, held_ml_pi, held_ml_pd, held_ml_abs, held_ml_idx;
reg         held_bf, held_bf_dn, held_bf_abs, held_bf_idx;
reg  [2:0]  held_bf_op;
reg         held_ck2, held_ck2_abs;
reg  [1:0]  held_ck2_size;
reg         held_cas, held_cas_pi, held_cas_pd;
reg         held_m16;
reg         held_cas2;
// The F-line gather: which kind, whether the command word is still to come
// (it is the first gathered word), the EA's shape, and every word in order.
reg         held_fp, held_fp_nocmd, held_fp_first, held_fp_gen, held_fp_imm, held_fp_idx, held_fp_abs;
reg   [8:0] held_fp_op;
reg   [1:0] held_fp_eaw;
reg  [15:0] fp_cmd_r;
reg  [95:0] fp_acc;
reg   [2:0] held_m16_form;
reg         held_is_immsr;      // ORI/ANDI/EORI to CCR or SR
reg         held_immsr_sr;      // ...to SR rather than CCR
reg         held_imm_ccr;       // does this immediate form set condition codes?
reg   [5:0] held_alu_op;
reg         held_alu_nowrite;   // CMP: flags only, as held_imm_nowrite is for CMPI
reg         held_alu_areg;      // destination is An (the ADDA family)
reg         held_alu_ccr;       // does this form set condition codes?
reg         held_alu_sxt;       // sign-extend a Word source to 32 bits
reg         held_ea_pcrel;      // the base is the PC, not An
reg         held_ea_indexed;    // the gathered word is a brief format, not a displacement
reg         held_alu_chk;       // this displacement form is a CHK
reg         held_immx;          // an operand in id_imm AND an EA extension
reg         held_immx_quick;    // ...whose operand is the ADDQ/SUBQ value
reg         held_immx_l;        // ...whose immediate is a longword
reg         held_immx_absl;     // ...whose EA is (xxx).L
reg         held_immx_abs;      // ...whose EA is absolute at all
reg  [3:0]  held_quick_val;
reg         held_st_an;         // a store's source register is An, not Dn
reg         held_alu_div;       // this displacement form is a DIVU/DIVS
reg         held_alu_divs;      // ...and the signed one
reg         held_alu_rmw;       // this form reads AND writes memory       // sign-extend a Word source to 32 bits
reg         held_is_jmp;
reg         held_is_bsr;
reg         held_is_jsr;
reg         held_is_movec;
reg         held_movec_dir;
reg  [2:0]  held_reg;
reg  [2:0]  held_dest_reg;
reg  [31:0] held_pc;
assign dec_hold_pc = held_pc;
reg  [3:0]  held_cond;
reg  [31:0] disp_acc;
reg  [15:0] disp_acc3;      // the word before disp_acc's two (milestone 117)

// A general F-line form's command word, arriving, may ask for more.
wire [2:0] fp_more   = fp_after_cmd(xw, h_fp_eaw, h_fp_imm);
wire       fp_extend = h_fp && h_fp_gen && h_fp_first && (fp_more != 3'd0);
// ------------------------------------------------ full-format extension
// (2026-09-24) An indexed EA's extension word with bit 8 set is the full
// format: base and index suppress, a null, word or long base displacement,
// and memory indirection with a null, word or long outer one. It was read
// as the brief format and executed silently wrong (the FPU differential
// found MOVE.B (d8,A5,Xn) with $21FC). The words that follow it are
// counted by the word itself.
//
// Where the extension word is the instruction's last word as counted so
// far -- every single EA, and a MOVE's destination -- the instruction is
// DECODED at that word, exactly as for a brief word, so every field that
// reads the words before it by position (an immediate, a bitfield or
// MOVES word, a MOVEM mask, the FPU's command) reads what it always did;
// then the displacement words are gathered with id_valid held low, and
// id_next_pc moves past them. A memory-to-memory MOVE's SOURCE word has
// the destination's words after it, so there the gather is extended in
// the middle instead, and the source's capture is not repeated. Reserved
// encodings -- BD SIZE 00, bit 3, I/IS 100 -- are illegal, as
// ap040_core.v's S_EA_EXTW2 has them; so are two full-format EAs in one
// MOVE.
reg         held_fx_seen, held_fx_dst, held_fx_bad, held_fx_tail;
assign dec_holding = (ext_pending != 3'd0) || held_fx_tail;
wire   gathering   = dec_holding;
reg  [15:0] held_fx_ext;
reg   [2:0] held_fx_left, held_fx_more;
reg   [1:0] held_fx_bdw, held_fx_odw;
reg  [63:0] fx_acc;

// Phase 8 (2026-09-25): a gathered instruction completes in its opcode
// cycle when its first extension word is already in the fetch's second
// slot. gstart is the gather-start condition, g_* what that start loads
// into each held register, h_* what the completing logic reads: the
// register while a gather is under way, the fresh field in the fast case.
// How many extension words the gather starting on this opcode collects,
// which is also the instruction's length beyond its opcode. One
// expression feeds both ext_pending and held_ext_n (milestone 113): the
// next-PC sum used to be rebuilt from held_is_long/held_is_xlong, which
// say how gather_disp combines words and not how many there are, so an
// immediate-plus-EA form -- ADDI.W #1,$10(A0), BSET #3,$10(A0), three
// words -- reported itself two bytes short. That is the PC an interrupt
// or a trace after it stacks.
// Three bits since milestone 114: a memory-to-memory MOVE.L between two
// (xxx).L addresses is four words.
wire [2:0] gather_words = is_fp ? fp_words :
                     is_move_mm_g ? mm_words :
                     is_m16_abs ? 3'd2 : is_m16_pp ? 3'd1 : is_cas2 ? 3'd2 :
                     is_cas ? (3'd1 + {1'b0, ea_ext_words}) :
                     is_ck2 ? (3'd1 + {1'b0, ea_ext_words}) :
                     is_bf ? (3'd1 + {1'b0, ea_ext_words}) :
                     is_ml ? (3'd1 + (ea_is_imm ? 3'd2 : {1'b0, ea_ext_words})) :
                     is_cmpi_pc ? (((if_opcode[7:6] == 2'b10) ? 3'd2 : 3'd1) + {1'b0, ea_ext_words}) :
                     is_moves ? (3'd1 + {1'b0, ea_ext_words}) :
                     is_quick_gx ? {1'b0, ea_ext_words} :
                     is_bit_s_x ? (3'd1 + {1'b0, ea_ext_words}) :
                     is_immx ? (((if_opcode[7:6] == 2'b10) ? 3'd2 : 3'd1) + {1'b0, ea_ext_words}) :
                     is_movem_absl ? 3'd3 :
                     (is_branch_long || is_bsr_long ||
                      ((is_imm_alu || is_immmem) && if_opcode[7:6] == 2'b10) ||
                      is_alu_immsrc_l ||
                      (is_move_imm && if_opcode[13:12] == 2'b10) ||
                      is_move_abs_l || is_movea_imm_l || is_st_abs_l ||
                      is_adda_imm_l || is_abs_alu_l ||
                      is_eaonly_abs_l || is_movem_disp ||
                      is_jmpjsr_abs_l || is_trapcc_l || is_muldiv_abs_l || is_movea_absl || is_adda_abs_l ||
                      // Scc asks the shared classifier instead of adding
                      // a name to this list. This assignment and
                      // held_is_long above are the two lists milestone 89
                      // was, and adding Scc to one and not the other
                      // reproduced that bug exactly: (xxx).L gathered one
                      // word, so the second half of the address was
                      // executed as the next instruction.
                      (is_scc_mem_gather && ea_ext_words == 2'd2) ||
                      // CHK.L #imm and CHK <(xxx).L>, and LINK.L's
                      // displacement -- in held_is_long above too.
                      (is_chk_imm && chk_is_long) || is_chk_abs_l || is_link_l ||
                      ((is_mvto_g || is_mvf_g) && ea_is_absl) ||
                      (is_tst_imm && (if_opcode[7:6] == 2'b10))) ? 3'd2 : 3'd1;
wire        gstart    = is_branch_word || is_branch_long || is_dbcc || is_move_disp || is_jmp_disp || is_bsr_word || is_bsr_long || is_jsr_disp || is_movec_opcode || is_imm_alu || is_alu_immsrc || is_immmem || is_move_imm || is_move_abs || is_movea_imm || is_st_abs || is_move_st_disp || is_alu_disp || is_lea_disp || is_adda_imm || is_adda_disp || is_link || is_movem || is_muldiv_imm || is_alu_dst_disp || is_alu_dst_idx || is_unary_gather || is_move_idx || is_alu_idx || is_lea_idx || is_move_pcrel || is_alu_pcrel || is_lea_pcrel || is_abs_alu || is_muldiv_abs || is_movea_abs || is_mul_gather || is_div_gather || is_movea_gather || is_adda_abs || is_chk_gather || is_quick_gx || is_pea_gather || is_eaonly_abs || is_immsr || is_chk_imm || is_jmp_idx || is_jmp_pcrel || is_jsr_idx || is_jsr_pcrel || is_jmpjsr_abs || is_trapcc_gather || is_scc_mem_gather || is_stop || is_chk_abs || is_bit_s_rr || is_bit_s_mem || is_bit_d_gx || is_packunpk || is_link_l || is_rtd || is_move_mm_g || is_ux_g || is_moves || is_mvto_g || is_mvto_imm || is_mvf_g || is_tst_imm || is_btst_dimm || is_cmpi_pc || is_movep || is_ml || is_bf || is_ck2 || is_cas || is_m16 || is_cas2 || (is_fp && (fp_words != 3'd0));
wire        gath_cont = (ext_pending != 3'd0) && !if_flt;
localparam  WIDE_DECODE = 1'b1;
wire        fast      = WIDE_DECODE && if_valid && !if_flt && if_valid2 && !if_flt2 &&
                        !held_fx_tail && (ext_pending == 3'd0) && gstart;
assign      take2     = fast;
wire [15:0] xw        = fast ? if_word2 : if_opcode;
wire          g_id_valid = 1'b0;
wire [31:0]   g_pc = if_pc;
wire [3:0]    g_cond = if_opcode[11:8];
wire          g_is_long = is_branch_long || is_bsr_long ||
                       				                 ((is_imm_alu || is_immmem) && if_opcode[7:6] == 2'b10) ||
                       				                 is_alu_immsrc_l ||
                       				                 (is_move_imm && if_opcode[13:12] == 2'b10) ||
                       				                 is_move_abs_l || is_movea_imm_l || is_st_abs_l || is_adda_imm_l ||
                       				                 is_abs_alu_l || is_eaonly_abs_l || is_movem_disp ||
                       				                 is_jmpjsr_abs_l || is_trapcc_l || is_muldiv_abs_l || is_movea_absl || is_adda_abs_l ||
                       				                 (is_scc_mem_gather && ea_is_absl) ||
                       				                 (is_chk_imm && chk_is_long) || is_chk_abs_l || is_link_l ||
                       				                 ((is_mvto_g || is_mvf_g) && ea_is_absl) ||
                       				                 (is_tst_imm && (if_opcode[7:6] == 2'b10));
wire          g_is_imm = is_quick_gx || is_alu_immsrc || is_imm_alu || is_immmem || is_move_imm || is_movea_imm || is_adda_imm ||
                      				                    is_bit_s_rr || is_bit_s_mem || is_tst_imm || is_btst_dimm || is_cmpi_pc ||
                      				                    is_muldiv_imm || is_chk_imm;
wire          g_imm_mem = is_immmem || is_quick_gx || is_bit_s_mem || is_cmpi_pc;
wire          g_imm_mem_pi = is_immmem_pi || (is_bit_s_mem && ea_is_pi);
wire          g_imm_mem_pd = is_immmem_pd || (is_bit_s_mem && ea_is_pd);
wire          g_imm_chk = is_chk_imm;
wire          g_imm_chk_l = is_chk_imm && chk_is_long;
wire          g_is_immsr = is_immsr || is_stop || is_mvto_imm;
wire          g_immsr_sr = is_immsr_sr || is_stop || (is_mvto_imm && mvto_sr);
wire          g_is_stop = is_stop;
wire [5:0]    g_imm_op = (is_bit_s_rr || is_bit_s_mem) ? bitop_op :
                      				                    is_tst_imm   ? `AP040_ALU_TST   :
                      				                    is_btst_dimm ? `AP040_ALU_BTSTR :
                      				                    is_quick_gx ? quick_op :
                      				                    (is_stop || is_mvto_imm) ? `AP040_ALU_MOVE :
                      				                    is_alu_immsrc ? alu_nib_op :
                      				                    is_immsr    ? immsr_op :
                      				                    is_mul_imm  ? (is_muldiv_imm_signed ? `AP040_ALU_MULS : `AP040_ALU_MULU) :
                      				                    is_div_imm  ? `AP040_ALU_MOVE :
                      				                    is_adda_imm ? alu_nib_op :
                      				                    (is_move_imm || is_movea_imm) ? `AP040_ALU_MOVE : imm_alu_op;
wire [1:0]    g_imm_size = (is_bit_s_mem || is_btst_dimm) ? `AP040_SZ_B :
                        				                    (is_movea_imm || is_adda_imm || is_muldiv_imm || is_chk_imm ||
                        				                     is_bit_s_rr) ? `AP040_SZ_L :
                        				                    is_move_imm  ? move_op_size : if_opcode[7:6];
wire          g_imm_nowrite = is_cmpi_i || is_cmpi_mem || is_cmpa_imm || is_chk_imm || is_cmpi_pc ||
                           				                    ((is_bit_s_rr || is_bit_s_mem) && is_bit_btst) || is_tst_imm || is_btst_dimm ||
                           				                    is_alu_immsrc_cmp;
wire          g_imm_dest9 = is_move_imm || is_movea_imm || is_adda_imm || is_muldiv_imm || is_btst_dimm ||
                         				                    is_chk_imm || is_alu_immsrc;
wire          g_imm_areg = is_movea_imm || is_adda_imm;
wire          g_imm_ccr = (is_imm_alu || is_immmem || is_quick_gx || is_move_imm || is_cmpa_imm || is_cmpi_pc ||
                       				                     is_bit_s_rr || is_bit_s_mem || is_tst_imm || is_btst_dimm ||
                       				                     is_muldiv_imm || is_alu_immsrc) && !is_chk_imm;
wire          g_imm_div = is_div_imm;
wire          g_imm_divs = is_div_imm && is_muldiv_imm_signed;
wire          g_is_abs = is_move_abs || is_abs_alu || is_eaonly_abs || is_jmpjsr_abs || is_chk_abs ||
                      				                    is_muldiv_abs || is_movea_abs || is_adda_abs;
wire          g_abs_jmp = is_jmp_abs;
wire          g_abs_jsr = is_jsr_abs;
wire          g_abs_lea = is_lea_abs;
wire          g_abs_push = is_pea_abs;
wire          g_abs_alu = is_abs_alu || is_mul_abs || is_adda_abs || is_chk_abs;
wire          g_abs_rmw = (is_unary_abs && !is_tst_abs) || is_alu_dst_abs || is_bit_d_abs || is_ux_abs;
wire          g_abs_div = is_div_abs;
wire          g_abs_divs = is_div_abs && is_muldiv_abs_signed;
wire          g_abs_sxt = is_muldiv_abs || is_movea_abs_w || (is_chk_abs && !chk_is_long) ||
                       				                    (is_adda_abs && (if_opcode[8] == 1'b0));
wire          g_abs_areg = is_movea_abs || is_adda_abs;
wire          g_is_stabs = is_st_abs;
wire          g_st_disp = is_move_st_disp;
wire          g_is_branch = is_branch_word || is_branch_long;
wire [1:0]    g_mv_size = (is_ux_g || is_ux_abs) ? ux_size :
                       				                    (is_bit_d_gx || is_bit_d_abs) ? `AP040_SZ_B :
                       				                    (is_muldiv_abs || is_movea_abs || is_mul_gather ||
                       				                     is_div_gather || is_movea_gather || is_adda_abs ||
                       				                     is_chk_gather || is_chk_abs) ? `AP040_SZ_L :
                       				                    is_abs_alu    ? add_op_size :
                       				                    is_adda_disp ? `AP040_SZ_L :
                       				                    (is_alu_disp || is_alu_dst_disp || is_alu_idx ||
                       				                     is_alu_pcrel || is_alu_dst_idx ||
                       				                     is_unary_gather) ? add_op_size : move_op_size;
wire          g_is_dbcc = is_dbcc;
wire          g_is_move_disp = is_move_disp || is_move_idx || is_move_pcrel;
wire          g_ea_indexed = is_move_idx || is_alu_idx || is_lea_idx || is_pea_idx ||
                          				                     is_jmp_idx || is_jsr_idx || (is_scc_mem_gather && ea_is_idx) ||
                          				                     is_alu_dst_idx || (is_unary_gather && unary_idx_shape) ||
                          				                     ((is_immx || is_quick_gx || is_bit_s_x || is_bit_d_gx) && ea_is_idx) ||
                          				                     (is_move_st_disp && (if_opcode[8:6] == 3'b110)) ||
                          				                     (is_ux_g && ea_is_idx) ||
                          				                     ((is_bit_d_gx || is_unary_gather_pc || is_cmpi_pc || is_bit_s_x) && ea_is_pcidx) ||
                          				                     is_movem_x || (is_ml && (ea_is_idx || ea_is_pcidx)) ||
                          				                     (is_bf && (ea_is_idx || ea_is_pcidx)) ||
                          				                     (is_ck2 && (ea_is_idx || ea_is_pcidx)) || (is_cas && ea_is_idx) ||
                          				                     (is_moves && ea_is_idx) ||
                          				                     (is_mvto_g && (ea_is_idx || ea_is_pcidx)) || (is_mvf_g && ea_is_idx) ||
                          				                     (is_move_mm_g && (ea_is_idx || ea_is_pcidx)) ||
                          				                     ((is_mul_gather || is_div_gather || is_movea_gather ||
                          				                       is_adda_disp || is_chk_gather) &&
                          				                      (ea_indexed_mode || ea_pcidx_mode)) ||
                          				                     ((is_move_pcrel || is_alu_pcrel || is_lea_pcrel ||
                          				                       is_pea_pcrel || is_jmp_pcrel || is_jsr_pcrel) && ea_pcidx_mode) ||
                          				                     (fp_has_ea && (ea_is_idx || ea_is_pcidx));
wire          g_ea_pcrel = is_move_pcrel || is_alu_pcrel || is_lea_pcrel || is_pea_pcrel ||
                        				                     (is_move_mm_g && ea_is_pcrel) || (is_mvto_g && ea_is_pcrel) ||
                        				                     ((is_bit_d_gx || is_unary_gather_pc || is_cmpi_pc || is_bit_s_x) && ea_is_pcrel) ||
                        				                     (is_movem_x && movem_mode_pcidx) || (is_ml && ea_is_pcrel) ||
                        				                     (is_bf && ea_is_pcrel) || (is_ck2 && ea_is_pcrel) ||
                        				                     is_jmp_pcrel || is_jsr_pcrel ||
                        				                     ((is_mul_gather || is_div_gather || is_movea_gather ||
                        				                       is_adda_disp || is_chk_gather) && ea_pcrel_mode) ||
                        				                     (fp_has_ea && ea_is_pcrel);
wire          g_is_scc_mem = is_scc_mem_gather;
wire          g_st_only = is_scc_mem_gather ||
                       				                     ((is_unary_gather || is_unary_abs) && (if_opcode[11:8] == 4'b0010));
wire          g_scc_abs = is_scc_mem_gather && ea_is_abs;
wire          g_is_alu_disp = is_alu_disp || is_adda_disp || is_alu_dst_disp || is_alu_idx || is_bit_d_gx ||
                           				                     is_ux_g ||
                           				                     is_alu_pcrel || is_alu_dst_idx || is_unary_gather ||
                           				                     is_mul_gather || is_div_gather || is_movea_gather ||
                           				                     is_chk_gather;
wire [5:0]    g_alu_op = (is_ux_g || is_ux_abs) ? ux_op :
                      				                     (is_bit_d_gx || is_bit_d_abs) ? bitop_op :
                      				                     is_mul_abs       ? (is_muldiv_abs_signed ? `AP040_ALU_MULS
                      				                                                              : `AP040_ALU_MULU) :
                      				                     is_mul_gather    ? (is_muldiv_gather_s ? `AP040_ALU_MULS
                      				                                                            : `AP040_ALU_MULU) :
                      				                     (is_div_gather || is_movea_gather) ? `AP040_ALU_MOVE :
                      				                     is_unary_abs     ? unary_abs_op   :
                      				                     is_unary_gather  ? unary_mem_op   :
                      				                     (is_alu_dst_disp || is_alu_dst_idx ||
                      				                      is_alu_dst_abs) ? alu_nib_dst_op : alu_nib_op;
wire          g_alu_nowrite = is_cmp_disp || is_cmpa_disp || is_alu_dst_disp || is_cmp_idx ||
                           				                     is_cmp_pcrel || is_cmp_abs || is_unary_abs || is_alu_dst_idx ||
                           				                     is_alu_dst_abs || is_unary_gather || is_cmpa_abs || is_chk_gather ||
                           				                     is_chk_abs || is_bit_d_gx || is_bit_d_abs || is_ux_g || is_ux_abs;
wire          g_alu_areg = is_adda_disp || is_movea_gather;
wire          g_alu_ccr = is_alu_disp || is_cmpa_disp || is_alu_dst_disp || is_alu_idx || is_bit_d_gx ||
                       				                     is_ux_g ||
                       				                     is_alu_pcrel || is_alu_dst_idx || is_unary_gather ||
                       				                     is_mul_gather || is_div_gather;
wire          g_alu_sxt = (is_adda_disp && (if_opcode[8] == 1'b0)) ||
                       				                     is_mul_gather || is_div_gather || is_movea_gather_w ||
                       				                     (is_chk_gather && !chk_is_long);
wire          g_alu_chk = is_chk_gather || is_chk_abs;
wire          g_chk_l = (is_chk_imm || is_chk_gather || is_chk_abs) && chk_is_long;
wire          g_immx = is_immx || is_quick_gx || is_bit_s_x || is_cmpi_pc;
wire          g_immx_quick = is_quick_gx;
wire          g_immx_l = (is_immx || is_cmpi_pc) && (if_opcode[7:6] == 2'b10);
wire          g_immx_absl = (is_immx || is_quick_gx || is_bit_s_x) && ea_is_absl;
wire          g_immx_abs = (is_immx || is_quick_gx || is_bit_s_x) && ea_is_abs;
wire [3:0]    g_quick_val = quick_val;
wire          g_st_an = (is_move_st_disp || is_st_abs) && if_opcode[3];
wire          g_alu_div = is_div_gather;
wire          g_alu_divs = is_div_gather && is_muldiv_gather_s;
wire          g_alu_rmw = is_alu_dst_disp || is_alu_dst_idx || is_bit_d_gx || is_ux_g ||
                       				                     (is_unary_gather && !is_unary_gather_tst);
wire          g_is_jmp = is_jmp_gather;
wire          g_is_lea = is_lea_disp || is_lea_idx || is_lea_pcrel || is_pea_gather;
wire          g_lea_push = is_pea_gather;
wire          g_is_link = is_link || is_link_l;
wire          g_is_pack = is_packunpk;
wire          g_pack_unpk = is_unpk_rr || is_unpk_m;
wire          g_pack_mem = is_pack_m || is_unpk_m;
wire          g_is_rtd = is_rtd;
wire          g_mm = is_move_mm_g;
wire          g_mm_simm = ea_is_imm;
wire          g_mm_spi = ea_is_pi;
wire          g_mm_spd = ea_is_pd;
wire          g_mm_sabs = ea_is_abs;
wire          g_mm_sbrief = ea_is_idx || ea_is_pcidx;
wire [1:0]    g_mm_swords = mvs_words;
wire [1:0]    g_mm_dwords = mvd_words;
wire          g_mm_dpi = mvd_pi;
wire          g_mm_dpd = mvd_pd;
wire          g_mm_dabs = mvd_absw || mvd_absl;
wire          g_mm_didx = mvd_idx;
wire [31:0]   g_mm_src_ext = 32'h0;
wire          g_moves = is_moves;
wire [1:0]    g_moves_size = if_opcode[7:6];
wire          g_moves_pi = ea_is_pi;
wire          g_moves_pd = ea_is_pd;
wire          g_moves_disp = ea_is_d16 || ea_is_idx;
wire          g_moves_abs = ea_is_abs;
wire          g_moves_idx = ea_is_idx;
wire          g_mvto = is_mvto_g;
wire          g_mvto_sr = mvto_sr;
wire          g_mvf = is_mvf_g;
wire          g_mvf_ccr = mvf_ccr;
wire          g_mvx_abs = ea_is_abs;
wire          g_movep = is_movep;
wire          g_movep_long = if_opcode[6];
wire          g_movep_wr = if_opcode[7];
wire          g_ml = is_ml;
wire          g_ml_div = if_opcode[6];
wire          g_ml_dn = ea_is_dn;
wire          g_ml_imm = ea_is_imm;
wire          g_ml_pi = ea_is_pi;
wire          g_ml_pd = ea_is_pd;
wire          g_ml_abs = ea_is_abs;
wire          g_ml_idx = ea_is_idx || ea_is_pcidx;
wire          g_bf = is_bf;
wire          g_bf_dn = ea_is_dn;
wire          g_bf_abs = ea_is_abs;
wire          g_bf_idx = ea_is_idx || ea_is_pcidx;
wire [2:0]    g_bf_op = bf_op;
wire          g_ck2 = is_ck2;
wire          g_ck2_abs = ea_is_abs;
wire [1:0]    g_ck2_size = if_opcode[10:9];
wire          g_cas = is_cas;
wire          g_m16 = is_m16;
wire          g_cas2 = is_cas2;
wire [2:0]    g_m16_form = is_m16_pp ? 3'd4 : {1'b0, if_opcode[4:3]};
wire          g_fp = is_fp;
wire          g_fx_seen = 1'b0;
wire          g_fx_bad = 1'b0;
wire [2:0]    g_fx_left = 3'd0;
wire [63:0]   g_fx_acc = 64'd0;
wire [8:0]    g_fp_op = if_opcode[8:0];
wire          g_fp_nocmd = fp_nocmd;
wire          g_fp_first = is_fp && !fp_nocmd;
wire          g_fp_gen = is_fp && (fp_kind == 3'b000);
wire [1:0]    g_fp_eaw = ea_ext_words;
wire          g_fp_imm = (if_opcode[5:0] == 6'b111100);
wire          g_fp_idx = ea_is_idx || ea_is_pcidx;
wire          g_fp_abs = fp_has_ea && ea_is_abs;
wire [95:0]   g_fp_acc = 96'd0;
wire          g_cas_pi = ea_is_pi;
wire          g_cas_pd = ea_is_pd;
wire [1:0]    g_pc_off = (is_cmpi_pc && (if_opcode[7:6] == 2'b10)) ? 2'd2 :
                      				                  (is_cmpi_pc || is_bit_s_x || is_movem_x || is_ml || is_bf || is_ck2 ||
                      				                   (is_fp && !fp_nocmd)) ? 2'd1 : 2'd0;
wire          g_is_trapcc = is_trapcc_gather;
wire          g_is_movem = is_movem;
wire          g_movem_dir = is_movem_ld;
wire          g_movem_word = is_movem_w;
wire          g_movem_down = is_movem_down;
wire          g_movem_wb = is_movem_wb;
wire          g_movem_pcrel = is_movem_pcrel;
wire          g_movem_abs = is_movem_absw || is_movem_absl;
wire          g_is_xlong = is_movem_absl;
wire          g_is_bsr = is_bsr_word || is_bsr_long;
wire          g_is_jsr = is_jsr_gather;
wire          g_is_movec = is_movec_opcode;
wire          g_movec_dir = if_opcode[0];
wire [2:0]    g_reg = if_opcode[2:0];
wire [2:0]    g_dest_reg = if_opcode[11:9];
wire [2:0]    g_ext_pending = gather_words;
wire [2:0]    g_ext_n = gather_words;
wire  [2:0] xp        = fast ? g_ext_pending : ext_pending;
wire [63:0] fx_acc_x  = fast ? g_fx_acc : fx_acc;
wire [95:0] fp_acc_x  = fast ? g_fp_acc : fp_acc;
wire [31:0]   h_pc = fast ? g_pc : held_pc;
wire [3:0]    h_cond = fast ? g_cond : held_cond;
wire          h_is_long = fast ? g_is_long : held_is_long;
wire          h_is_imm = fast ? g_is_imm : held_is_imm;
wire          h_imm_mem = fast ? g_imm_mem : held_imm_mem;
wire          h_imm_mem_pi = fast ? g_imm_mem_pi : held_imm_mem_pi;
wire          h_imm_mem_pd = fast ? g_imm_mem_pd : held_imm_mem_pd;
wire          h_imm_chk = fast ? g_imm_chk : held_imm_chk;
wire          h_imm_chk_l = fast ? g_imm_chk_l : held_imm_chk_l;
wire          h_is_immsr = fast ? g_is_immsr : held_is_immsr;
wire          h_immsr_sr = fast ? g_immsr_sr : held_immsr_sr;
wire          h_is_stop = fast ? g_is_stop : held_is_stop;
wire [5:0]    h_imm_op = fast ? g_imm_op : held_imm_op;
wire [1:0]    h_imm_size = fast ? g_imm_size : held_imm_size;
wire          h_imm_nowrite = fast ? g_imm_nowrite : held_imm_nowrite;
wire          h_imm_dest9 = fast ? g_imm_dest9 : held_imm_dest9;
wire          h_imm_areg = fast ? g_imm_areg : held_imm_areg;
wire          h_imm_ccr = fast ? g_imm_ccr : held_imm_ccr;
wire          h_imm_div = fast ? g_imm_div : held_imm_div;
wire          h_imm_divs = fast ? g_imm_divs : held_imm_divs;
wire          h_is_abs = fast ? g_is_abs : held_is_abs;
wire          h_abs_jmp = fast ? g_abs_jmp : held_abs_jmp;
wire          h_abs_jsr = fast ? g_abs_jsr : held_abs_jsr;
wire          h_abs_lea = fast ? g_abs_lea : held_abs_lea;
wire          h_abs_push = fast ? g_abs_push : held_abs_push;
wire          h_abs_alu = fast ? g_abs_alu : held_abs_alu;
wire          h_abs_rmw = fast ? g_abs_rmw : held_abs_rmw;
wire          h_abs_div = fast ? g_abs_div : held_abs_div;
wire          h_abs_divs = fast ? g_abs_divs : held_abs_divs;
wire          h_abs_sxt = fast ? g_abs_sxt : held_abs_sxt;
wire          h_abs_areg = fast ? g_abs_areg : held_abs_areg;
wire          h_is_stabs = fast ? g_is_stabs : held_is_stabs;
wire          h_st_disp = fast ? g_st_disp : held_st_disp;
wire          h_is_branch = fast ? g_is_branch : held_is_branch;
wire [1:0]    h_mv_size = fast ? g_mv_size : held_mv_size;
wire          h_is_dbcc = fast ? g_is_dbcc : held_is_dbcc;
wire          h_is_move_disp = fast ? g_is_move_disp : held_is_move_disp;
wire          h_ea_indexed = fast ? g_ea_indexed : held_ea_indexed;
wire          h_ea_pcrel = fast ? g_ea_pcrel : held_ea_pcrel;
wire          h_is_scc_mem = fast ? g_is_scc_mem : held_is_scc_mem;
wire          h_st_only = fast ? g_st_only : held_st_only;
wire          h_scc_abs = fast ? g_scc_abs : held_scc_abs;
wire          h_is_alu_disp = fast ? g_is_alu_disp : held_is_alu_disp;
wire [5:0]    h_alu_op = fast ? g_alu_op : held_alu_op;
wire          h_alu_nowrite = fast ? g_alu_nowrite : held_alu_nowrite;
wire          h_alu_areg = fast ? g_alu_areg : held_alu_areg;
wire          h_alu_ccr = fast ? g_alu_ccr : held_alu_ccr;
wire          h_alu_sxt = fast ? g_alu_sxt : held_alu_sxt;
wire          h_alu_chk = fast ? g_alu_chk : held_alu_chk;
wire          h_chk_l = fast ? g_chk_l : held_chk_l;
wire          h_immx = fast ? g_immx : held_immx;
wire          h_immx_quick = fast ? g_immx_quick : held_immx_quick;
wire          h_immx_l = fast ? g_immx_l : held_immx_l;
wire          h_immx_absl = fast ? g_immx_absl : held_immx_absl;
wire          h_immx_abs = fast ? g_immx_abs : held_immx_abs;
wire [3:0]    h_quick_val = fast ? g_quick_val : held_quick_val;
wire          h_st_an = fast ? g_st_an : held_st_an;
wire          h_alu_div = fast ? g_alu_div : held_alu_div;
wire          h_alu_divs = fast ? g_alu_divs : held_alu_divs;
wire          h_alu_rmw = fast ? g_alu_rmw : held_alu_rmw;
wire          h_is_jmp = fast ? g_is_jmp : held_is_jmp;
wire          h_is_lea = fast ? g_is_lea : held_is_lea;
wire          h_lea_push = fast ? g_lea_push : held_lea_push;
wire          h_is_link = fast ? g_is_link : held_is_link;
wire          h_is_pack = fast ? g_is_pack : held_is_pack;
wire          h_pack_unpk = fast ? g_pack_unpk : held_pack_unpk;
wire          h_pack_mem = fast ? g_pack_mem : held_pack_mem;
wire          h_is_rtd = fast ? g_is_rtd : held_is_rtd;
wire          h_mm = fast ? g_mm : held_mm;
wire          h_mm_simm = fast ? g_mm_simm : held_mm_simm;
wire          h_mm_spi = fast ? g_mm_spi : held_mm_spi;
wire          h_mm_spd = fast ? g_mm_spd : held_mm_spd;
wire          h_mm_sabs = fast ? g_mm_sabs : held_mm_sabs;
wire          h_mm_sbrief = fast ? g_mm_sbrief : held_mm_sbrief;
wire [1:0]    h_mm_swords = fast ? g_mm_swords : held_mm_swords;
wire [1:0]    h_mm_dwords = fast ? g_mm_dwords : held_mm_dwords;
wire          h_mm_dpi = fast ? g_mm_dpi : held_mm_dpi;
wire          h_mm_dpd = fast ? g_mm_dpd : held_mm_dpd;
wire          h_mm_dabs = fast ? g_mm_dabs : held_mm_dabs;
wire          h_mm_didx = fast ? g_mm_didx : held_mm_didx;
wire [31:0]   h_mm_src_ext = fast ? g_mm_src_ext : held_mm_src_ext;
wire          h_moves = fast ? g_moves : held_moves;
wire [1:0]    h_moves_size = fast ? g_moves_size : held_moves_size;
wire          h_moves_pi = fast ? g_moves_pi : held_moves_pi;
wire          h_moves_pd = fast ? g_moves_pd : held_moves_pd;
wire          h_moves_disp = fast ? g_moves_disp : held_moves_disp;
wire          h_moves_abs = fast ? g_moves_abs : held_moves_abs;
wire          h_moves_idx = fast ? g_moves_idx : held_moves_idx;
wire          h_mvto = fast ? g_mvto : held_mvto;
wire          h_mvto_sr = fast ? g_mvto_sr : held_mvto_sr;
wire          h_mvf = fast ? g_mvf : held_mvf;
wire          h_mvf_ccr = fast ? g_mvf_ccr : held_mvf_ccr;
wire          h_mvx_abs = fast ? g_mvx_abs : held_mvx_abs;
wire          h_movep = fast ? g_movep : held_movep;
wire          h_movep_long = fast ? g_movep_long : held_movep_long;
wire          h_movep_wr = fast ? g_movep_wr : held_movep_wr;
wire          h_ml = fast ? g_ml : held_ml;
wire          h_ml_div = fast ? g_ml_div : held_ml_div;
wire          h_ml_dn = fast ? g_ml_dn : held_ml_dn;
wire          h_ml_imm = fast ? g_ml_imm : held_ml_imm;
wire          h_ml_pi = fast ? g_ml_pi : held_ml_pi;
wire          h_ml_pd = fast ? g_ml_pd : held_ml_pd;
wire          h_ml_abs = fast ? g_ml_abs : held_ml_abs;
wire          h_ml_idx = fast ? g_ml_idx : held_ml_idx;
wire          h_bf = fast ? g_bf : held_bf;
wire          h_bf_dn = fast ? g_bf_dn : held_bf_dn;
wire          h_bf_abs = fast ? g_bf_abs : held_bf_abs;
wire          h_bf_idx = fast ? g_bf_idx : held_bf_idx;
wire [2:0]    h_bf_op = fast ? g_bf_op : held_bf_op;
wire          h_ck2 = fast ? g_ck2 : held_ck2;
wire          h_ck2_abs = fast ? g_ck2_abs : held_ck2_abs;
wire [1:0]    h_ck2_size = fast ? g_ck2_size : held_ck2_size;
wire          h_cas = fast ? g_cas : held_cas;
wire          h_m16 = fast ? g_m16 : held_m16;
wire          h_cas2 = fast ? g_cas2 : held_cas2;
wire [2:0]    h_m16_form = fast ? g_m16_form : held_m16_form;
wire          h_fp = fast ? g_fp : held_fp;
wire          h_fx_seen = fast ? g_fx_seen : held_fx_seen;
wire          h_fx_bad = fast ? g_fx_bad : held_fx_bad;
wire [2:0]    h_fx_left = fast ? g_fx_left : held_fx_left;
wire [8:0]    h_fp_op = fast ? g_fp_op : held_fp_op;
wire          h_fp_nocmd = fast ? g_fp_nocmd : held_fp_nocmd;
wire          h_fp_first = fast ? g_fp_first : held_fp_first;
wire          h_fp_gen = fast ? g_fp_gen : held_fp_gen;
wire [1:0]    h_fp_eaw = fast ? g_fp_eaw : held_fp_eaw;
wire          h_fp_imm = fast ? g_fp_imm : held_fp_imm;
wire          h_fp_idx = fast ? g_fp_idx : held_fp_idx;
wire          h_fp_abs = fast ? g_fp_abs : held_fp_abs;
wire          h_cas_pi = fast ? g_cas_pi : held_cas_pi;
wire          h_cas_pd = fast ? g_cas_pd : held_cas_pd;
wire [1:0]    h_pc_off = fast ? g_pc_off : held_pc_off;
wire          h_is_trapcc = fast ? g_is_trapcc : held_is_trapcc;
wire          h_is_movem = fast ? g_is_movem : held_is_movem;
wire          h_movem_dir = fast ? g_movem_dir : held_movem_dir;
wire          h_movem_word = fast ? g_movem_word : held_movem_word;
wire          h_movem_down = fast ? g_movem_down : held_movem_down;
wire          h_movem_wb = fast ? g_movem_wb : held_movem_wb;
wire          h_movem_pcrel = fast ? g_movem_pcrel : held_movem_pcrel;
wire          h_movem_abs = fast ? g_movem_abs : held_movem_abs;
wire          h_is_xlong = fast ? g_is_xlong : held_is_xlong;
wire          h_is_bsr = fast ? g_is_bsr : held_is_bsr;
wire          h_is_jsr = fast ? g_is_jsr : held_is_jsr;
wire          h_is_movec = fast ? g_is_movec : held_is_movec;
wire          h_movec_dir = fast ? g_movec_dir : held_movec_dir;
wire [2:0]    h_reg = fast ? g_reg : held_reg;
wire [2:0]    h_dest_reg = fast ? g_dest_reg : held_dest_reg;
wire [2:0]    h_ext_n = fast ? g_ext_n : held_ext_n;

wire        fx_src_pos  = !h_fp_first &&
                          (h_mm ? (h_mm_sbrief && (xp == ({1'b0, h_mm_dwords} + 3'd1)))
                                   : (h_ea_indexed && (xp == 3'd1)));
wire        fx_dst_pos  = h_mm && h_mm_didx && (xp == 3'd1);
wire        fx_here     = (fx_src_pos || fx_dst_pos) && !h_fx_seen && xw[8];
wire        fx_twice    = fx_dst_pos && h_fx_seen && xw[8];
wire  [1:0] fx_bdw      = (xw[5:4] == 2'b10) ? 2'd1 : (xw[5:4] == 2'b11) ? 2'd2 : 2'd0;
wire  [1:0] fx_odw      = (xw[1:0] == 2'b10) ? 2'd1 : (xw[1:0] == 2'b11) ? 2'd2 : 2'd0;
wire  [2:0] fx_more     = {1'b0, fx_bdw} + {1'b0, fx_odw};
wire        fx_reserved = (xw[5:4] == 2'b00) || xw[3] || (xw[2:0] == 3'b100);
wire        fx_final    = fx_here && (xp == 3'd1);
wire  [3:0] fx_len      = {1'b0, h_ext_n} + {1'b0, fx_more};
wire        fx_mid_ext  = fx_here && !fx_final && !fx_reserved && !fx_len[3] && (fx_more != 3'd0);
wire        fx_tail_go  = fx_final && !fx_reserved && (fx_more != 3'd0);
wire        fx_bad_now  = h_fx_bad || fx_twice || (fx_here && (fx_reserved || (!fx_final && fx_len[3])));
wire        fx_valid_now = !fx_bad_now && (h_fx_seen || fx_here);
wire [15:0] fx_ext_now  = fx_here ? xw : held_fx_ext;
wire        fx_dst_now  = fx_here ? fx_dst_pos : held_fx_dst;
wire  [1:0] fx_bdw_now  = fx_here ? fx_bdw : held_fx_bdw;
wire  [1:0] fx_odw_now  = fx_here ? fx_odw : held_fx_odw;
wire [63:0] fx_acc_now  = (h_fx_left != 3'd0) ? {fx_acc_x[47:0], xw} : fx_acc_x;
wire [31:0] fx_od_now   = (fx_odw_now == 2'd2) ? fx_acc_now[31:0] :
                          (fx_odw_now == 2'd1) ? {{16{fx_acc_now[15]}}, fx_acc_now[15:0]} : 32'd0;
wire [15:0] fx_bdw1     = (fx_odw_now == 2'd2) ? fx_acc_now[47:32] :
                          (fx_odw_now == 2'd1) ? fx_acc_now[31:16] : fx_acc_now[15:0];
wire [31:0] fx_bd_now   = (fx_bdw_now == 2'd2) ? ((fx_odw_now == 2'd2) ? fx_acc_now[63:32] :
                                                 (fx_odw_now == 2'd1) ? fx_acc_now[47:16] : fx_acc_now[31:0]) :
                          (fx_bdw_now == 2'd1) ? {{16{fx_bdw1[15]}}, fx_bdw1} : 32'd0;
wire completing_gather = (xp == 3'd1) && !fp_extend;

// The full displacement as of the completing cycle: word form sign-extends
// if_opcode alone (nothing was usefully shifted into disp_acc for a 1-word
// gather); long form combines the word shifted in last cycle with this
// cycle's word, high-word-first (see header comment).
wire [31:0] gather_disp = h_is_long ? {disp_acc[15:0], xw}
                                       : {{16{xw[15]}}, xw};

// Combinational redirect: fires immediately for the byte form (same cycle
// it's fetched, gated to only when not already mid-gather -- if_opcode
// during a gather cycle is data, not an opcode, and could coincidentally
// bit-match the byte-form pattern), or on the exact cycle a word/long
// gather completes -- EXCEPT a move-disp, JMP, or JSR gather: move-disp's
// displacement is a memory offset, not a branch target; JMP/JSR have no
// literal target to speculate with at all (their target is a register
// value, not known until EA-fetch -- see header comment). BSR is NOT
// excluded here -- unconditionally taken, same as BRA, so the "assume
// taken" guess is always correct -- see this file's header.
// Static branch prediction (2026-09-24): backward taken, forward not taken,
// for the CONDITIONAL branches. Every branch used to be predicted taken, so
// a forward branch that fell through -- the common if-then shape -- paid a
// full recovery: 4 cycles on the local array and 6 on the bus against the
// sequential core's 1.5. A forward conditional branch now does not redirect;
// id_bnt tells EX, which redirects to the target when it IS taken. BRA and
// BSR always go, and a backward Bcc and DBcc -- loops -- keep the old guess.
wire bnt_byte   = is_branch_byte && (if_opcode[11:8] != 4'h0) && !if_opcode[7];
wire bnt_gather = h_is_branch && (h_cond != 4'h0) && !gather_disp[31];
wire redirect_from_byte   = if_valid && ((is_branch_byte && !bnt_byte) || is_bsr_byte) && (ext_pending == 3'd0);
// ... and NOT an immediate-source ALU op (milestone 26): its gathered words
// are an operand, not a displacement, so there is no target to speculate
// with. Without this exclusion the completing gather redirects to
// held_pc + 2 + the immediate, which for ADDI.L #$12345678 is a wild jump
// and the rest of the program never runs.
// ... and only when the completing word is actually here (milestone 80): a
// fetch bubble mid-gather must not redirect on a stale if_opcode.
wire rfg_exact = if_valid && !if_flt && completing_gather && !h_is_move_disp && !h_is_alu_disp && !h_is_lea &&
                             !h_is_link && !h_is_movem && !h_is_jmp &&
                             !h_is_jsr && !h_is_movec && !h_is_imm &&
                             !h_is_abs && !h_is_stabs && !h_is_immsr && !h_st_disp &&
                             // An Scc is not a change of flow whatever its
                             // destination: without this the gathered forms
                             // redirected the fetch to their own displacement
                             // and the next instruction came from there.
                             !h_is_scc_mem &&
                             !h_is_trapcc && !h_is_pack && !h_is_rtd && !h_mm &&
                             !h_moves && !h_mvto && !h_mvf && !h_movep && !h_ml && !h_bf &&
                             !h_ck2 && !h_cas && !h_m16 && !h_cas2 && !h_fp &&
                             !bnt_gather;
// The redirect reaches port A's address, so phase 8 splits it (the first fit
// with the fast path ran the whole gather-start classification into it,
// qn -> if_opcode -> gstart -> h_* -> this list -> the L1's read, the core
// top's worst path). Only a one-extension-word gather can complete in the
// opcode's cycle, and of those only Bcc.W, BSR.W and DBcc redirect -- the
// list above, which is negative, names every other kind -- so the fast case
// is those three decoded from the opcode alone, and a held gather completing
// is the list over the held registers themselves. The check below holds the
// split to rfg_exact, the one expression that is right in both cases.
wire [31:0] gather_disp_slow = held_is_long ? {disp_acc[15:0], if_opcode} : {{16{if_opcode[15]}}, if_opcode};
wire        fp_extend_slow   = held_fp && held_fp_gen && held_fp_first &&
                               (fp_after_cmd(if_opcode, held_fp_eaw, held_fp_imm) != 3'd0);
wire        completing_slow  = (ext_pending == 3'd1) && !fp_extend_slow;
wire        bnt_slow         = held_is_branch && (held_cond != 4'h0) && !gather_disp_slow[31];
wire rfg_slow = if_valid && !if_flt && completing_slow && !held_is_move_disp && !held_is_alu_disp && !held_is_lea &&
                             !held_is_link && !held_is_movem && !held_is_jmp &&
                             !held_is_jsr && !held_is_movec && !held_is_imm &&
                             !held_is_abs && !held_is_stabs && !held_is_immsr && !held_st_disp &&
                             !held_is_scc_mem &&
                             !held_is_trapcc && !held_is_pack && !held_is_rtd && !held_mm &&
                             !held_moves && !held_mvto && !held_mvf && !held_movep && !held_ml && !held_bf &&
                             !held_ck2 && !held_cas && !held_m16 && !held_cas2 && !held_fp &&
                             !bnt_slow;
wire rfg_fast = WIDE_DECODE && if_valid && !if_flt && if_valid2 && !if_flt2 && !held_fx_tail &&
                (ext_pending == 3'd0) &&
                (is_bsr_word || is_dbcc || (is_branch_word && ((if_opcode[11:8] == 4'h0) || if_word2[15])));
wire redirect_from_gather = rfg_fast || rfg_slow;

// MOVEC gather-completion helper: an otherwise-recognized MOVEC whose
// extension-word selector names something this core doesn't model (the MMU
// registers, out of scope -- see movec_sel_valid's own comment) becomes
// illegal, exactly the same vector/format id_is_illegal already drives for
// any other unrecognized opcode.
wire movec_illegal_gather = h_is_movec && !movec_sel_valid;

// A memory-to-memory MOVE's source extension, from the words as they stand:
// two words are a longword (absolute or immediate), one is a brief word
// (indexed) or a sign-extended displacement, absolute or immediate.
wire [31:0] mm_src_now = (h_mm_swords == 2'd2) ? {disp_acc[15:0], xw} :
                         h_mm_sbrief           ? {16'd0, xw} :
                                                    {{16{xw[15]}}, xw};
// ...and the destination's, which is always the LAST word or two.
wire [31:0] mm_dst_now = (h_mm_dwords == 2'd2) ? {disp_acc[15:0], xw} :
                         (h_mm_dwords == 2'd0) ? 32'h0 :
                         h_mm_didx             ? {16'd0, xw} :
                                                    {{16{xw[15]}}, xw};
// With no destination words the completing word is the source's last, not
// yet captured; otherwise the capture has already happened (or the source
// had no words and the capture is the zero it started as).
wire [31:0] mm_imm_now = (h_mm_dwords == 2'd0) ? mm_src_now : h_mm_src_ext;

// MOVES at completion: the register word is the FIRST gathered word, so it
// is this one, the one before, or the one before that, by how many there
// were; the EA's words, if any, are the last ones.
wire [15:0] moves_ext   = (h_ext_n == 3'd1) ? xw :
                          (h_ext_n == 3'd2) ? disp_acc[15:0] : disp_acc[31:16];
wire  [3:0] moves_r     = {moves_ext[15], moves_ext[14:12]};
wire        moves_wr    = moves_ext[11];                    // Rn -> <ea>
wire        moves_std   = moves_wr && h_moves_disp;      // a store on the displacement carrier
wire        moves_an_ld = !moves_wr && moves_ext[15];       // a load into An: sign-extended
// MULx.L/DIVx.L at completion, the register word found the same way.
wire [15:0] ml_ext  = moves_ext;
wire  [2:0] ml_dl   = ml_ext[14:12];
wire  [2:0] ml_dh   = ml_ext[2:0];
wire        ml_s    = ml_ext[11];
wire        ml_64   = ml_ext[10];
wire [31:0] ml_ea   = (h_ext_n == 3'd1) ? 32'h0 :
                      (h_ext_n == 3'd3) ? {disp_acc[15:0], xw} :
                      h_ml_idx          ? {16'd0, xw} :
                                             {{16{xw[15]}}, xw};
// Bitfields at completion: the first gathered word is the bitfield word.
wire [15:0] bf_ext   = moves_ext;
wire        bf_mod   = (h_bf_op == 3'd2) || (h_bf_op == 3'd4) ||
                       (h_bf_op == 3'd6) || (h_bf_op == 3'd7);
// Which register is written: the EA's Dn for a modifying op on a register,
// the word's Dn for BFEXTU/BFEXTS/BFFFO; BFTST and memory modifies write
// none, and a memory BFINS reads its source through port B instead.
wire        bf_wr    = (h_bf_dn && bf_mod) ||
                       (h_bf_op == 3'd1) || (h_bf_op == 3'd3) || (h_bf_op == 3'd5);
// The F-line EA's words are the last ones: two for (xxx).L, else one, a
// displacement, a brief word verbatim, or a word address. FSAVE/FRESTORE
// have no command word ahead of them.
// An EA with no words of its own -- (An) above all -- has no displacement;
// the completing word is then the command word, not an EA word.
wire [31:0] fp_eadisp = (h_fp_eaw == 2'd0) ? 32'd0 :
                        ((h_fp_nocmd ? 3'd2 : 3'd3) == h_ext_n) ? {disp_acc[15:0], xw} :
                        h_fp_idx ? {16'd0, xw} : {{16{xw[15]}}, xw};
wire [31:0] moves_eaext = (h_ext_n == 3'd1) ? 32'h0 :
                          (h_ext_n == 3'd3) ? {disp_acc[15:0], xw} :
                          h_moves_idx       ? {16'd0, xw} :
                                                 {{16{xw[15]}}, xw};

assign id_redirect_valid = redirect_from_byte || redirect_from_gather;
assign id_redirect_pc    = rfg_fast ? (if_pc + 32'd2 + {{16{if_word2[15]}}, if_word2}) :
                           rfg_slow ? (held_pc + 32'd2 + gather_disp_slow) :
                                      (if_pc + 32'd2 + {{24{if_opcode[7]}}, if_opcode[7:0]});
`ifdef VERILATOR
always @(posedge clk)
	if (nreset && ce && if_valid) begin
		if (rfg_exact != redirect_from_gather)
			$error("ap040_decode: the split gather redirect (%b) disagrees with the whole one (%b) at %h",
			       redirect_from_gather, rfg_exact, if_pc);
		else if (rfg_exact && (id_redirect_pc != h_pc + 32'd2 + gather_disp))
			$error("ap040_decode: the split gather redirect goes to %h, the whole one to %h",
			       id_redirect_pc, h_pc + 32'd2 + gather_disp);
	end
`endif


// The opcode word of an instruction with extension words: hold what
// it is and start gathering (see the gather header above). A task so
// that the fast path runs the same statements.
task start_gather;
begin
	// Opcode word of a word/long-form branch, a DBcc,
	// MOVE.L (d16,An),Dn, JMP (d16,An), a word/long-form BSR,
	// JSR (d16,An), or MOVEC (all word-form except long-branch/
	// long-BSR): not a complete instruction yet -- hold what we
	// know, start gathering.
	id_valid <= g_id_valid;
	held_pc <= g_pc;
	held_cond <= g_cond;
	held_is_long <= g_is_long;
	held_is_imm <= g_is_imm;
	// The destination is memory, not a register: the operand
	// crossover, the address base and every "writes no
	// register" consequence key off this one flag.
	held_imm_mem <= g_imm_mem;
	held_imm_mem_pi <= g_imm_mem_pi;
	held_imm_mem_pd <= g_imm_mem_pd;
	held_imm_chk <= g_imm_chk;
	held_imm_chk_l <= g_imm_chk_l;
	held_is_immsr <= g_is_immsr;
	held_immsr_sr <= g_immsr_sr;
	held_is_stop <= g_is_stop;
	held_imm_op <= g_imm_op;
	// ADDA.W operates on the full 32 bits; only its SOURCE is a
	// word, and gather_disp has already sign-extended that.
	held_imm_size <= g_imm_size;
	held_imm_nowrite <= g_imm_nowrite;
	// CHK's register is ir[11:9] like the rest of these; without
	// dest9 it would read ir[2:0], which for mode 111/100 is the
	// constant 4 -- so CHK #10,D0 checked D4 and never trapped.
	held_imm_dest9 <= g_imm_dest9;
	held_imm_areg <= g_imm_areg;
	held_imm_ccr <= g_imm_ccr;
	held_imm_div <= g_imm_div;
	held_imm_divs <= g_imm_divs;
	held_is_abs <= g_is_abs;
	held_abs_jmp <= g_abs_jmp;
	held_abs_jsr <= g_abs_jsr;
	held_abs_lea <= g_abs_lea;
	held_abs_push <= g_abs_push;
	held_abs_alu <= g_abs_alu;
	// TST reads without writing; everything else in the unary
	// group writes its result back, and the binary family's
	// destination is a register, not the address.
	held_abs_rmw <= g_abs_rmw;
	held_abs_div <= g_abs_div;
	held_abs_divs <= g_abs_divs;
	held_abs_sxt <= g_abs_sxt;
	held_abs_areg <= g_abs_areg;
	held_is_stabs <= g_is_stabs;
	held_st_disp <= g_st_disp;
	held_is_branch <= g_is_branch;
	// The ALU family takes its size from ir[7:6]; MOVE's lives in
	// ir[13:12] with a different encoding, hence two wires.
	held_mv_size <= g_mv_size;
	held_is_dbcc <= g_is_dbcc;
	held_is_move_disp <= g_is_move_disp;
	held_ea_indexed <= g_ea_indexed;
	held_ea_pcrel <= g_ea_pcrel;
	held_is_scc_mem <= g_is_scc_mem;
	held_st_only <= g_st_only;
	held_scc_abs <= g_scc_abs;
	held_is_alu_disp <= g_is_alu_disp;
	// The ir[8]=1 direction has its own op map: nibble 1011 is
	// EOR there, not CMP.
	held_alu_op <= g_alu_op;
	// An RMW's destination is memory, so it writes no register.
	held_alu_nowrite <= g_alu_nowrite;
	held_alu_areg <= g_alu_areg;
	held_alu_ccr <= g_alu_ccr;
	held_alu_sxt <= g_alu_sxt;
	held_alu_chk <= g_alu_chk;
	held_chk_l <= g_chk_l;
	held_immx <= g_immx;
	held_immx_quick <= g_immx_quick;
	held_immx_l <= g_immx_l;
	held_immx_absl <= g_immx_absl;
	held_immx_abs <= g_immx_abs;
	held_quick_val <= g_quick_val;
	held_st_an <= g_st_an;
	held_alu_div <= g_alu_div;
	held_alu_divs <= g_alu_divs;
	held_alu_rmw <= g_alu_rmw;
	held_is_jmp <= g_is_jmp;
	held_is_lea <= g_is_lea;
	held_lea_push <= g_lea_push;
	held_is_link <= g_is_link;
	held_is_pack <= g_is_pack;
	held_pack_unpk <= g_pack_unpk;
	held_pack_mem <= g_pack_mem;
	held_is_rtd <= g_is_rtd;
	held_mm <= g_mm;
	held_mm_simm <= g_mm_simm;
	held_mm_spi <= g_mm_spi;
	held_mm_spd <= g_mm_spd;
	held_mm_sabs <= g_mm_sabs;
	held_mm_sbrief <= g_mm_sbrief;
	held_mm_swords <= g_mm_swords;
	held_mm_dwords <= g_mm_dwords;
	held_mm_dpi <= g_mm_dpi;
	held_mm_dpd <= g_mm_dpd;
	held_mm_dabs <= g_mm_dabs;
	held_mm_didx <= g_mm_didx;
	held_mm_src_ext <= g_mm_src_ext;
	held_moves <= g_moves;
	held_moves_size <= g_moves_size;
	held_moves_pi <= g_moves_pi;
	held_moves_pd <= g_moves_pd;
	held_moves_disp <= g_moves_disp;
	held_moves_abs <= g_moves_abs;
	held_moves_idx <= g_moves_idx;
	held_mvto <= g_mvto;
	held_mvto_sr <= g_mvto_sr;
	held_mvf <= g_mvf;
	held_mvf_ccr <= g_mvf_ccr;
	held_mvx_abs <= g_mvx_abs;
	held_movep <= g_movep;
	held_movep_long <= g_movep_long;
	held_movep_wr <= g_movep_wr;
	held_ml <= g_ml;
	held_ml_div <= g_ml_div;
	held_ml_dn <= g_ml_dn;
	held_ml_imm <= g_ml_imm;
	held_ml_pi <= g_ml_pi;
	held_ml_pd <= g_ml_pd;
	held_ml_abs <= g_ml_abs;
	held_ml_idx <= g_ml_idx;
	held_bf <= g_bf;
	held_bf_dn <= g_bf_dn;
	held_bf_abs <= g_bf_abs;
	held_bf_idx <= g_bf_idx;
	held_bf_op <= g_bf_op;
	held_ck2 <= g_ck2;
	held_ck2_abs <= g_ck2_abs;
	held_ck2_size <= g_ck2_size;
	held_cas <= g_cas;
	held_m16 <= g_m16;
	held_cas2 <= g_cas2;
	held_m16_form <= g_m16_form;
	held_fp <= g_fp;
	held_fx_seen <= g_fx_seen;
	held_fx_bad <= g_fx_bad;
	held_fx_left <= g_fx_left;
	fx_acc <= g_fx_acc;
	held_fp_op <= g_fp_op;
	held_fp_nocmd <= g_fp_nocmd;
	held_fp_first <= g_fp_first;
	held_fp_gen <= g_fp_gen;
	held_fp_eaw <= g_fp_eaw;
	held_fp_imm <= g_fp_imm;
	held_fp_idx <= g_fp_idx;
	held_fp_abs <= g_fp_abs;
	fp_acc <= g_fp_acc;
	held_cas_pi <= g_cas_pi;
	held_cas_pd <= g_cas_pd;
	held_pc_off <= g_pc_off;
	held_is_trapcc <= g_is_trapcc;
	held_is_movem <= g_is_movem;
	held_movem_dir <= g_movem_dir;
	held_movem_word <= g_movem_word;
	held_movem_down <= g_movem_down;
	held_movem_wb <= g_movem_wb;
	held_movem_pcrel <= g_movem_pcrel;
	held_movem_abs <= g_movem_abs;
	held_is_xlong <= g_is_xlong;
	held_is_bsr <= g_is_bsr;
	held_is_jsr <= g_is_jsr;
	held_is_movec <= g_is_movec;
	held_movec_dir <= g_movec_dir;
	                                  // in the OPCODE word, not the
	                                  // extension word -- see header.
	held_reg <= g_reg;
	held_dest_reg <= g_dest_reg;
	ext_pending <= g_ext_pending;
	held_ext_n <= g_ext_n;
end
endtask

always @(posedge clk) begin
	if (!nreset) begin
		id_valid        <= 1'b0;
		id_pc           <= 32'h0;
		id_next_pc      <= 32'h0;
		id_dest_reg     <= 4'h0;
		id_src_reg      <= 4'h0;
		id_imm          <= 32'h0;
		id_ea_ext       <= 32'h0;
		id_alu_op       <= 6'h0;
		id_size         <= `AP040_SZ_L;
		id_shcnt        <= 6'd1;
		id_shift_reg    <= 1'b0;
		id_src_a_is_imm <= 1'b0;
		id_writes_reg   <= 1'b0;
		id_writes_ccr   <= 1'b0;
		id_is_branch    <= 1'b0;
		id_is_scc       <= 1'b0;
		id_is_dbcc      <= 1'b0;
		id_is_mem_src   <= 1'b0;
		id_is_abs       <= 1'b0;
		id_is_store     <= 1'b0;
		id_is_postinc   <= 1'b0;
		id_is_predec    <= 1'b0;
		id_is_jmp       <= 1'b0;
		id_is_lea       <= 1'b0;
		id_sxt_w        <= 1'b0;
		id_ea_indexed   <= 1'b0;
		id_ea_pcrel     <= 1'b0;
		id_is_rmw       <= 1'b0;
		id_immrmw       <= 1'b0;
		id_st_only      <= 1'b0;
		id_st_disp      <= 1'b0;
		id_is_trapcc    <= 1'b0;
		id_is_chk       <= 1'b0;
		id_chk_long     <= 1'b0;
		id_is_immsr     <= 1'b0;
		id_is_stop      <= 1'b0;
		id_immsr_to_sr  <= 1'b0;
		id_is_pea       <= 1'b0;
		id_is_link      <= 1'b0;
		id_is_div       <= 1'b0;
		id_div_signed   <= 1'b0;
		id_is_movem     <= 1'b0;
		id_movem_dir    <= 1'b0;
		id_movem_word   <= 1'b0;
		id_movem_down   <= 1'b0;
		id_movem_wb     <= 1'b0;
		id_movem_pcrel  <= 1'b0;
		id_movem_abs    <= 1'b0;
		id_movem_mask   <= 16'h0;
		id_is_unlk      <= 1'b0;
		id_is_bsr       <= 1'b0;
		id_is_jsr       <= 1'b0;
		id_is_trap      <= 1'b0;
		id_is_illegal   <= 1'b0;
		id_illegal_kind <= 2'd0;
		id_is_movesr    <= 1'b0;
		id_is_movec     <= 1'b0;
		id_is_rts       <= 1'b0;
		id_is_nop       <= 1'b0;
		id_cinv         <= 6'd0;
		id_pmmu         <= 5'd0;
		id_fflt         <= 6'd0;
		id_bnt          <= 1'b0;
		id_is_reset     <= 1'b0;
		id_is_rte       <= 1'b0;
		id_is_rtr       <= 1'b0;
		id_cond         <= 4'h0;
		id_fp           <= 1'b0;
		id_fp_op        <= 9'd0;
		id_fp_cmd       <= 16'd0;
		id_fp_imm       <= 96'd0;
		id_fx           <= 6'd0;
		id_fx_bd        <= 32'd0;
		id_fx_od        <= 32'd0;
		held_fx_seen    <= 1'b0;
		held_fx_dst     <= 1'b0;
		held_fx_bad     <= 1'b0;
		held_fx_tail    <= 1'b0;
		held_fx_ext     <= 16'd0;
		held_fx_left    <= 3'd0;
		held_fx_more    <= 3'd0;
		held_fx_bdw     <= 2'd0;
		held_fx_odw     <= 2'd0;
		fx_acc          <= 64'd0;
		ext_pending     <= 3'd0;
		held_is_imm      <= 1'b0;
		held_imm_mem     <= 1'b0;
		held_imm_mem_pi  <= 1'b0;
		held_imm_mem_pd  <= 1'b0;
		held_imm_op      <= 6'd0;
		held_imm_size    <= `AP040_SZ_L;
		held_imm_nowrite <= 1'b0;
		held_imm_dest9   <= 1'b0;
		held_abs_lea     <= 1'b0;
		held_abs_push    <= 1'b0;
		held_abs_jmp     <= 1'b0;
		held_abs_jsr     <= 1'b0;
		held_abs_alu     <= 1'b0;
		held_abs_rmw     <= 1'b0;
		held_abs_div     <= 1'b0;
		held_abs_divs    <= 1'b0;
		held_abs_sxt     <= 1'b0;
		held_abs_areg    <= 1'b0;
		held_is_abs      <= 1'b0;
		held_imm_areg    <= 1'b0;
		held_is_stabs    <= 1'b0;
		held_st_disp     <= 1'b0;
		held_is_branch   <= 1'b0;
		held_mv_size     <= `AP040_SZ_L;
		held_is_long    <= 1'b0;
		held_is_xlong   <= 1'b0;
		held_ext_n      <= 3'd0;
		held_is_dbcc    <= 1'b0;
		held_is_move_disp <= 1'b0;
		held_is_alu_disp  <= 1'b0;
		held_is_scc_mem   <= 1'b0;
		held_st_only      <= 1'b0;
		held_is_stop      <= 1'b0;
		held_scc_abs      <= 1'b0;
		held_alu_op       <= `AP040_ALU_MOVE;
		held_alu_nowrite  <= 1'b0;
		held_alu_areg     <= 1'b0;
		held_alu_ccr      <= 1'b0;
		held_alu_sxt      <= 1'b0;
		held_alu_div      <= 1'b0;
		held_alu_chk      <= 1'b0;
		held_immx         <= 1'b0;
		held_immx_quick   <= 1'b0;
		held_immx_l       <= 1'b0;
		held_immx_absl    <= 1'b0;
		held_immx_abs     <= 1'b0;
		held_quick_val    <= 4'd0;
		held_st_an        <= 1'b0;
		held_alu_divs     <= 1'b0;
		held_alu_rmw      <= 1'b0;
		held_ea_indexed   <= 1'b0;
		held_ea_pcrel     <= 1'b0;
		held_is_jmp     <= 1'b0;
		held_is_lea     <= 1'b0;
		held_imm_chk    <= 1'b0;
		held_imm_chk_l  <= 1'b0;
		held_chk_l      <= 1'b0;
		held_is_pack    <= 1'b0;
		held_pack_unpk  <= 1'b0;
		held_pack_mem   <= 1'b0;
		held_is_rtd     <= 1'b0;
		held_mm         <= 1'b0;
		held_mm_simm    <= 1'b0;
		held_mm_spi     <= 1'b0;
		held_mm_spd     <= 1'b0;
		held_mm_sabs    <= 1'b0;
		held_mm_sbrief  <= 1'b0;
		held_mm_swords  <= 2'd0;
		held_mm_dwords  <= 2'd0;
		held_mm_dpi     <= 1'b0;
		held_mm_dpd     <= 1'b0;
		held_mm_dabs    <= 1'b0;
		held_mm_didx    <= 1'b0;
		held_mm_src_ext <= 32'h0;
		id_mm           <= 7'd0;
		held_moves      <= 1'b0;
		held_moves_size <= 2'd0;
		held_moves_pi   <= 1'b0;
		held_moves_pd   <= 1'b0;
		held_moves_disp <= 1'b0;
		held_moves_abs  <= 1'b0;
		held_moves_idx  <= 1'b0;
		held_mvto       <= 1'b0;
		held_mvto_sr    <= 1'b0;
		held_mvf        <= 1'b0;
		held_mvf_ccr    <= 1'b0;
		held_mvx_abs    <= 1'b0;
		held_pc_off     <= 2'd0;
		id_pc_off       <= 2'd0;
		held_movep      <= 1'b0;
		held_movep_long <= 1'b0;
		held_movep_wr   <= 1'b0;
		id_movep        <= 3'd0;
		held_ml         <= 1'b0;
		held_ml_div     <= 1'b0;
		held_ml_dn      <= 1'b0;
		held_ml_imm     <= 1'b0;
		held_ml_pi      <= 1'b0;
		held_ml_pd      <= 1'b0;
		held_ml_abs     <= 1'b0;
		held_ml_idx     <= 1'b0;
		id_ml           <= 7'd0;
		held_bf         <= 1'b0;
		held_bf_dn      <= 1'b0;
		held_bf_abs     <= 1'b0;
		held_bf_idx     <= 1'b0;
		held_bf_op      <= 3'd0;
		id_bf           <= 5'd0;
		held_ck2        <= 1'b0;
		held_ck2_abs    <= 1'b0;
		held_ck2_size   <= 2'd0;
		id_ck2          <= 3'd0;
		held_cas        <= 1'b0;
		held_m16        <= 1'b0;
		held_fp         <= 1'b0;
		held_fp_op      <= 9'd0;
		held_fp_nocmd   <= 1'b0;
		held_fp_first   <= 1'b0;
		held_fp_gen     <= 1'b0;
		held_fp_imm     <= 1'b0;
		held_fp_idx     <= 1'b0;
		held_fp_abs     <= 1'b0;
		held_fp_eaw     <= 2'd0;
		fp_cmd_r        <= 16'd0;
		fp_acc          <= 96'd0;
		held_cas2       <= 1'b0;
		held_m16_form   <= 3'd0;
		id_m16          <= 4'd0;
		held_cas_pi     <= 1'b0;
		held_cas_pd     <= 1'b0;
		id_cas          <= 5'd0;
		id_mvfsr        <= 2'd0;
		id_moves        <= 3'd0;
		held_is_immsr   <= 1'b0;
		held_immsr_sr   <= 1'b0;
		held_imm_ccr    <= 1'b0;
		held_imm_div    <= 1'b0;
		held_imm_divs   <= 1'b0;
		held_lea_push   <= 1'b0;
		held_is_link    <= 1'b0;
		held_is_trapcc  <= 1'b0;
		held_is_movem   <= 1'b0;
		held_movem_dir  <= 1'b0;
		held_movem_word <= 1'b0;
		held_movem_down <= 1'b0;
		held_movem_wb   <= 1'b0;
		held_movem_pcrel<= 1'b0;
		held_movem_abs  <= 1'b0;
		held_is_bsr     <= 1'b0;
		held_is_jsr     <= 1'b0;
		held_is_movec   <= 1'b0;
		held_movec_dir  <= 1'b0;
		held_reg        <= 3'h0;
		held_dest_reg   <= 3'h0;
		held_pc         <= 32'h0;
		held_cond       <= 4'h0;
		disp_acc        <= 32'h0;
		disp_acc3       <= 16'h0;
	end else if (ce) begin
		if (flush) begin
			id_valid    <= 1'b0;
			ext_pending <= 3'd0;   // abandon any in-progress gather too
			held_fx_tail <= 1'b0;
		end else if (!stall_in && if_valid) begin
			id_fflt <= 6'd0;
			// A faulted word is never an extension word: whatever the gather
			// in progress was, the instruction is abandoned at it, and issues
			// below as the fault -- see id_fflt.
			if (held_fx_tail && !if_flt) begin
				// A full-format EA's displacement words; the instruction was
				// decoded at its extension word (see fx_* above).
				fx_acc       <= {fx_acc[47:0], if_opcode};
				held_fx_left <= held_fx_left - 3'd1;
				ext_pending  <= ext_pending - 3'd1;
				if (ext_pending == 3'd1) begin
					id_valid     <= 1'b1;
					id_next_pc   <= id_next_pc + {28'd0, held_fx_more, 1'b0};
					id_fx_bd     <= fx_bd_now;
					id_fx_od     <= fx_od_now;
					held_fx_tail <= 1'b0;
				end
			end else if (gath_cont || gstart) begin
				// A fresh opcode starts its gather (C, below); with its first
				// extension word already here as well (fast, phase 8) that word is
				// taken in the same cycle, by the same logic that takes it from a
				// held gather -- reading the fields C is loading rather than their
				// registers (h_*), and the word from the second fetch slot (xw).
				if (!gath_cont) start_gather;
				if (gath_cont || fast) begin
				// Gathering: if_opcode is extension-word data, never a
				// fresh opcode.
				disp_acc <= {disp_acc[15:0], xw};
				disp_acc3 <= disp_acc[31:16];
				if (h_fx_left != 3'd0) begin
					fx_acc       <= {fx_acc_x[47:0], xw};
					held_fx_left <= h_fx_left - 3'd1;
				end
				if (fx_here) begin
					held_fx_seen <= 1'b1;
					held_fx_ext  <= xw;
					held_fx_dst  <= fx_dst_pos;
					held_fx_bdw  <= fx_bdw;
					held_fx_odw  <= fx_odw;
					held_fx_more <= fx_more;
					held_fx_left <= (fx_mid_ext || fx_tail_go) ? fx_more : 3'd0;
					fx_acc       <= 64'd0;
					if (fx_reserved || (!fx_final && fx_len[3])) held_fx_bad <= 1'b1;
				end
				if (fx_twice) held_fx_bad <= 1'b1;
				if (h_fp) begin
					fp_acc <= {fp_acc_x[79:0], xw};
					if (h_fp_first) begin
						fp_cmd_r      <= xw;
						held_fp_first <= 1'b0;
					end
				end
				// A memory-to-memory MOVE's last SOURCE word is the one with
				// exactly the destination's words still to come.
				if (h_mm && (xp == ({1'b0, h_mm_dwords} + 3'd1)) && !h_fx_seen)
					held_mm_src_ext <= mm_src_now;
				if (completing_gather) begin
					id_valid        <= 1'b1;
					id_pc           <= h_pc;
					// Unconditional regardless of movec_illegal_gather: this
					// is genuinely "the next instruction's address" -- the
					// extension word was consumed either way, so the gather
					// is the same width whether or not its selector turned
					// out valid. id_is_illegal's own exception path reads
					// id_pc (held_pc, already set above), not id_next_pc, for
					// its stacked PC -- see ap040_ea_fetch.v's header.
					id_next_pc      <= h_pc + 32'd2 + {28'd0, h_ext_n, 1'b0};
					// A store's address register is its DESTINATION, at
					// ir[11:9] -- the opposite end of the opcode from a
					// load's, which is why this sits above the rest.
					id_dest_reg     <= h_m16     ? {1'b1, bf_ext[14:12]} :
					                   h_cas     ? {1'b0, bf_ext[2:0]} :
					                   h_ck2     ? {bf_ext[15], bf_ext[14:12]} :
					                   h_bf      ? ((h_bf_dn && bf_mod) ? {1'b0, h_reg}
					                                                           : {1'b0, bf_ext[14:12]}) :
					                   h_ml      ? {1'b0, ml_dl} :
					                   h_movep   ? {1'b0, h_dest_reg} :
					                   h_moves   ? ((moves_wr && !moves_std) ? {1'b1, h_reg} : moves_r) :
					                   h_mm      ? {1'b1, h_dest_reg} :
					                   h_is_pack ? {h_pack_mem, h_dest_reg} :
					                   h_is_rtd  ? 4'd15 :
					                   h_st_disp ? {h_st_an, h_reg} :
					                    (h_is_abs && (h_abs_push || h_abs_jsr)) ? 4'd15 :
					                    (h_is_abs && h_abs_lea)  ? {1'b1, h_dest_reg} :
					                    (h_is_abs && h_abs_areg) ? {1'b1, h_dest_reg} :
					                    h_is_abs  ? {1'b0, h_dest_reg} :
					                    (h_is_imm && h_imm_mem) ? 4'd0 :
					                    h_is_imm  ? (h_imm_dest9 ? {h_imm_areg, h_dest_reg}
					                                                     : {1'b0, h_reg}) :
					                    h_is_dbcc ? {1'b0, h_reg} :
					                    h_is_move_disp ? {1'b0, h_dest_reg} :
					                    h_is_alu_disp  ? {h_alu_areg, h_dest_reg} :
					                    (h_is_lea && h_lea_push) ? 4'd15 :
					                    h_is_lea  ? {1'b1, h_dest_reg} :
					                    h_is_link ? 4'd15 :
					                    (h_is_bsr || h_is_jsr) ? 4'd15 :
					                    // An invalid selector is an exception, whose frame
					                    // push owns A7 like every other one: pointing it at
					                    // Rn wrote the new stack pointer into the MOVEC's
					                    // register (milestone 113).
					                    h_is_movec ? ((h_movec_dir || movec_illegal_gather) ? 4'd15
					                                                                             : movec_gpr) : 4'h0;
					// move-disp and JMP/JSR-disp all read An as their EA base
					// (held_reg, unified index 8+n); DBcc's held_reg means
					// its loop counter instead; BSR needs no source read at
					// all (its target came from decode's own redirect_pc,
					// not a register), so held_reg is simply unused for it.
					// MOVEC's write direction (Rn -> control register) reads
					// Rn as its source, same as any other register-direct
					// source operand; its read direction needs no source
					// read at all (the "operand" is a control register,
					// resolved entirely in ap040_ea_fetch.v/ap040_execute.v
					// -- see their headers), so movec_gpr only appears here
					// for the write-direction half.
					// An immediate with a memory destination reads An as its
					// address base, exactly like every other memory form --
					// the immediate rides in id_imm instead of displacing it.
					id_src_reg      <= h_fp ? {1'b1, h_reg} :
					                   (h_ck2 || h_cas || h_m16) ? {1'b1, h_reg} :
					                   h_bf      ? {!h_bf_dn, h_reg} :
					                   h_ml      ? {!h_ml_dn, h_reg} :
					                   h_movep   ? {1'b1, h_reg} :
					                   h_moves   ? ((moves_wr && !moves_std) ? moves_r : {1'b1, h_reg}) :
					                   h_mm      ? {1'b1, h_reg} :
					                   h_is_pack ? {h_pack_mem, h_reg} :
					                   h_is_rtd  ? 4'd15 :
					                   h_st_disp ? {1'b1, h_dest_reg} :
					                    h_is_stabs ? {h_st_an, h_reg} :
					                    h_imm_mem ? {1'b1, h_reg} :
					                    // An Scc destination reads An as its address
					                    // base, like every other gathered memory form.
					                    // An absolute one reads nothing, and held_reg
					                    // is the low three opcode bits, which for
					                    // (xxx).W/(xxx).L are the mode-7 selector --
					                    // so the absolute case must not reach here.
					                    (h_is_scc_mem && !h_scc_abs) ? {1'b1, h_reg} :
					                    ((h_mvto || h_mvf) && !h_mvx_abs) ? {1'b1, h_reg} :
					                    h_is_dbcc ? {1'b0, h_reg} :
					                    (h_is_move_disp || h_is_alu_disp || h_is_lea || h_is_link ||
					                     h_is_movem || h_is_jmp || h_is_jsr) ? {1'b1, h_reg} :
					                    (h_is_movec && h_movec_dir) ? movec_gpr : 4'h0;
					// gather_disp is already the sign-extended displacement
					// word (same wire Bcc/DBcc use for their target math) --
					// move-disp and JMP/JSR-disp all reuse it verbatim as
					// id_imm, no new arithmetic. Zero for every other gather
					// kind, matching the plain-(An)/JMP(An)/JSR(An) id_imm
					// fix below (this field is a real address offset now,
					// not just MOVEQ's unused-elsewhere immediate -- see
					// header). MOVEC repurposes it once more (same
					// "general-purpose field" precedent TRAP's vector number
					// already established) to carry {direction, selector
					// code} packed together -- ap040_ea_fetch.v/
					// ap040_execute.v extract both from eac_imm[3:0] rather
					// than needing two more dedicated ports threaded through
					// every stage.
					// MOVEM: the mask is the FIRST gathered word and the EA the rest.
					// One word -- the mask is completing now, no EA. Two -- the mask
					// was shifted in, the displacement is completing. Three -- the
					// mask is two words back and the completing pair is the address.
					id_movem_mask   <= h_is_xlong ? disp_acc[31:16] :
					                   h_is_long  ? disp_acc[15:0]  : xw;
					// For an operand-plus-EA form the OPERAND is what id_imm
					// carries; the words arrived operand first, so they sit in
					// disp_acc by the time the EA's last word is if_opcode.
					id_ea_ext       <= h_bf ? {16'd0, bf_ext} :
					                   h_mm ? mm_dst_now :
					                   (h_is_pack || h_is_rtd) ? gather_disp :
					                   !h_immx ? 32'h0 :
					                   h_immx_absl  ? {disp_acc[15:0], xw} :
					                   h_ea_indexed ? {16'd0, xw} :
					                                     {{16{xw[15]}}, xw};
					id_imm          <= h_fp ? fp_eadisp :
					                   (h_m16 || h_cas2) ? {disp_acc[15:0], xw} :
					                   (h_bf || h_ck2 || h_cas) ? ml_ea :
					                   h_ml ? ml_ea :
					                   h_moves ? moves_eaext :
					                   h_mm ? mm_imm_now :
					                   h_immx ? (h_immx_quick ? {28'd0, h_quick_val} :
					                                (h_immx_l && h_immx_absl) ? {disp_acc3, disp_acc[31:16]} :
					                                h_immx_l     ? disp_acc :
					                                h_immx_absl  ? {{16{disp_acc[31]}}, disp_acc[31:16]} :
					                                                  {{16{disp_acc[15]}}, disp_acc[15:0]}) :
					                   h_is_movem ? (h_is_xlong ? {disp_acc[15:0], xw} :
					                                    h_is_long  ? {{16{xw[15]}}, xw} :
					                                                    32'h0) :
					                   h_ea_indexed ? {16'd0, xw} :
					                   (h_is_move_disp || h_is_alu_disp || h_is_lea || h_is_link ||
					                    h_is_movem || h_is_jmp || h_is_jsr ||
					                    h_is_imm || h_is_abs || h_is_stabs || h_st_disp ||
					                    h_is_immsr || h_is_scc_mem || h_mvto || h_mvf || h_movep ||
					                    // ...and the branches, whose displacement EA-fetch
					                    // needs to see whether the target is odd.
					                    h_is_branch || h_is_bsr || h_is_dbcc) ? gather_disp :
					                    h_is_movec ? {27'd0, h_movec_dir, movec_sel_code} : 32'h0;
					id_alu_op       <= h_is_pack     ? (h_pack_unpk ? `AP040_ALU_UNPK
					                                                      : `AP040_ALU_PACK) :
					                   h_is_immsr    ? h_imm_op :
					                   h_is_imm      ? h_imm_op :
					                   (h_is_alu_disp || h_abs_alu) ? h_alu_op :
					                                                        `AP040_ALU_MOVE;
					id_size         <= h_ck2 ? h_ck2_size :
					                   (h_cas || h_cas2) ? (h_ck2_size - 2'd1) :
					                   h_movep ? (h_movep_long ? `AP040_SZ_L : `AP040_SZ_W) :
					                   h_moves ? (moves_an_ld ? `AP040_SZ_L : h_moves_size) :
					                   h_is_pack ? ((h_pack_unpk ^ h_pack_mem) ? `AP040_SZ_W : `AP040_SZ_B) :
					                   h_is_imm ? h_imm_size :
					                   h_is_scc_mem ? `AP040_SZ_B :
					                   (h_mvto || h_mvf) ? `AP040_SZ_W :
					                   (h_is_move_disp || h_is_alu_disp || h_is_abs ||
					                    h_st_disp || h_is_stabs || h_mm) ? h_mv_size :
					                                                        `AP040_SZ_L;
					id_shcnt        <= 6'd1;
					id_shift_reg    <= 1'b0;
					// The gathered word IS the source: ap040_ea_fetch.v's
					// operand_a mux already takes eac_imm on this flag, the
					// path MOVEQ has used since milestone 2.
					id_src_a_is_imm <= (h_is_imm && !h_imm_mem) || h_is_immsr || (h_ml && h_ml_imm) ||
					                   (h_mm && h_mm_simm);
					// DBcc's write is dynamic (see header); BSR/JSR's is
					// static -- both always decrement A7 when they execute
					// at all. MOVEC's read direction writes a real GPR
					// (Rn <- the selected control register); its write
					// direction writes no GPR at all (the control register
					// write happens via ap040_execute.v's new exe_writes_
					// creg path, not commit_reg) -- and an invalid selector
					// writes nothing either, having already become illegal
					// above.
					// An absolute store writes memory, not a register, and an
					// absolute JMP writes neither: the address it gathered is
					// where it goes, not a result. held_abs_jmp has to be
					// excluded by name, the way id_writes_ccr below excludes
					// it -- a JMP is not an ALU operation, so held_abs_alu is
					// 0 and the term after it is true for free. Without this
					// the absolute path's own destination, the opcode's bits
					// [11:9], took the target: JMP fixes those at 111, so
					// both forms wrote D7 (milestone 106).
					id_writes_reg   <= h_is_move_disp || h_is_bsr || h_is_jsr || h_is_pack || h_is_rtd ||
					                    (h_mm && (h_mm_dpi || h_mm_dpd)) ||
					                    (h_moves && !moves_wr) || (h_movep && !h_movep_wr) ||
					                    h_ml || (h_bf && bf_wr) || h_cas ||
					                    (h_m16 && (h_m16_form == 3'd4)) ||
					                    (h_is_abs && h_abs_jsr) ||
					                    (h_is_abs && !h_abs_jmp &&
					                     (!h_abs_alu || !h_alu_nowrite)) ||
					                    (h_is_imm && !h_imm_nowrite && !h_imm_mem) ||
					                    (h_is_alu_disp && !h_alu_nowrite) || h_is_lea || h_is_link ||
					                    (h_is_movec && !h_movec_dir && !movec_illegal_gather);
					// MOVEA sets no condition codes.
					id_writes_ccr   <= h_is_move_disp || (h_is_alu_disp && h_alu_ccr) || h_mm ||
					                   h_ml || h_bf || h_ck2 || h_cas || h_cas2 ||
					                    (h_is_abs && !h_abs_lea && !h_abs_push &&
					                     !h_abs_jmp && !h_abs_jsr && !h_alu_chk &&
					                     !(h_abs_areg && !h_alu_nowrite)) ||
					                    h_is_stabs || h_st_disp ||
					                    (h_is_imm && h_imm_ccr);
					id_is_branch    <= !h_is_dbcc && !h_is_move_disp && !h_is_alu_disp && !h_is_lea &&
					                    !h_is_link && !h_is_movem && !h_is_jmp &&
					                    !h_is_bsr && !h_is_jsr && !h_is_movec &&
					                    !h_is_imm && !h_is_abs && !h_is_stabs && !h_st_disp &&
					                    !h_is_immsr && !h_is_trapcc && !h_is_pack && !h_is_rtd &&
					                    !h_mm && !h_moves && !h_movep && !h_ml && !h_bf && !h_ck2 &&
					                    !h_cas && !h_m16 && !h_cas2 && !h_fp &&
					                    // This list is NEGATIVE: anything gathered and not
					                    // named here is a branch, and its id_imm is read as
					                    // a branch displacement. An Scc left off it had its
					                    // destination displacement checked for an odd branch
					                    // target, which raised an address error on every
					                    // gathered form landing on an odd byte.
					                    !h_is_scc_mem && !h_mvto && !h_mvf;
					id_is_scc       <= h_is_scc_mem;
					id_is_dbcc      <= h_is_dbcc;
					// LEA and PEA deliver the address itself, so unlike every
					// other absolute form they read nothing.
					id_is_mem_src   <= h_is_move_disp || h_is_alu_disp || h_imm_mem || h_is_rtd ||
					                   (h_mm && !h_mm_simm) || (h_moves && !moves_wr) ||
					                   (h_is_pack && h_pack_mem) ||
					                   (h_ml && !h_ml_dn && !h_ml_imm) || h_cas ||
					                   h_is_scc_mem || h_mvto || h_mvf ||
					                   (h_is_abs && !h_abs_lea && !h_abs_push &&
					                    !h_abs_jmp && !h_abs_jsr);
					id_is_abs       <= h_is_abs || h_is_stabs || h_scc_abs || h_immx_abs ||
					                   (h_mm && h_mm_sabs) || (h_moves && h_moves_abs) ||
					                   ((h_mvto || h_mvf) && h_mvx_abs) || (h_ml && h_ml_abs) ||
					                   (h_bf && h_bf_abs) || ((h_ck2 || h_cas) && h_ck2_abs) ||
					                   (h_fp && h_fp_abs);
					id_is_store     <= h_is_stabs || h_st_disp || (h_moves && moves_wr);
					// The autoincrement modes of the immediate-to-memory family
					// (milestone 89); every other gathered form addresses with a
					// displacement, an index or an absolute, none of which steps An.
					id_is_postinc   <= h_imm_mem_pi || (h_mm && h_mm_spi) || (h_moves && h_moves_pi) ||
					                   (h_ml && h_ml_pi) || (h_cas && h_cas_pi);
					id_is_predec    <= h_imm_mem_pd || (h_mm && h_mm_spd) || (h_moves && h_moves_pd) ||
					                   (h_is_pack && h_pack_mem) ||
					                   (h_ml && h_ml_pd) || (h_cas && h_cas_pd);
					// A MOVES store of the An its own (An)+/-(An) steps writes the
					// STEPPED value, as ap040_core.v's S_MOVES_WR reads Rn after
					// ea_start (milestone 114). A MOVE stores the original.
					id_moves        <= {h_moves, h_moves && moves_an_ld && (h_moves_size == `AP040_SZ_B),
					                    h_moves && moves_wr && (h_moves_pi || h_moves_pd) &&
					                    (moves_r == {1'b1, h_reg})};
					// PACK/UNPK -(Ax),-(Ay) is a memory-to-memory move to -(Ay).
					// The rest are masked with held_mm: a PACK opcode's own bits
					// read as a MOVE destination would be anything.
					id_mm           <= {1'b0, h_mm || (h_is_pack && h_pack_mem),
					                    h_mm_simm && h_mm, h_mm_dpi && h_mm,
					                    (h_mm_dpd && h_mm) || (h_is_pack && h_pack_mem),
					                    h_mm_dabs && h_mm, h_mm_didx && h_mm};
					id_is_jmp       <= h_is_jmp || (h_is_abs && h_abs_jmp);
					id_is_lea       <= (h_is_lea && !h_lea_push) || (h_is_abs && h_abs_lea);
					id_sxt_w        <= (h_is_alu_disp && h_alu_sxt) ||
					                   (h_moves && moves_an_ld && (h_moves_size == `AP040_SZ_W)) ||
					                   (h_is_abs && h_abs_sxt);
					id_st_disp      <= h_st_disp || (h_moves && moves_std);
					id_is_rmw       <= (h_is_alu_disp && h_alu_rmw) || h_abs_rmw ||
					                    (h_imm_mem && !h_imm_nowrite) || h_is_scc_mem || h_mvf;
					id_immrmw       <= h_imm_mem;
					id_st_only      <= h_st_only;
					id_ea_indexed   <= h_ea_indexed;
					id_ea_pcrel     <= h_ea_pcrel;
					id_is_pea       <= (h_is_lea && h_lea_push) || (h_is_abs && h_abs_push);
					id_is_chk       <= (h_is_imm && h_imm_chk) ||
					                   ((h_is_alu_disp || h_is_abs) && h_alu_chk);
					id_chk_long     <= h_chk_l;
					id_is_trapcc    <= h_is_trapcc;
					id_is_immsr     <= h_is_immsr || h_mvto;
					id_immsr_to_sr  <= h_immsr_sr || (h_mvto && h_mvto_sr);
					id_mvfsr        <= {h_mvf, h_mvf_ccr};
					id_pc_off       <= h_pc_off;
					id_movep        <= {h_movep, h_movep_long, h_movep_wr};
					id_ml           <= {h_ml, h_ml_div, ml_s, ml_64, ml_dh};
					id_bf           <= {h_bf, h_bf_dn, h_bf_op};
					id_ck2          <= {h_ck2, bf_ext[11], bf_ext[15]};
					id_cas          <= {h_cas2, h_cas, bf_ext[8:6]};
					id_m16          <= {h_m16, h_m16_form};
					id_is_stop      <= h_is_stop;
					id_is_link      <= h_is_link;
					id_is_div       <= (h_ml && h_ml_div) ||
					                   (h_is_imm && h_imm_div) ||
					                   (h_is_abs && h_abs_div) ||
					                   (h_is_alu_disp && h_alu_div);
					id_div_signed   <= (h_ml && ml_s) ||
					                   (h_is_imm && h_imm_divs) ||
					                   (h_is_abs && h_abs_divs) ||
					                   (h_is_alu_disp && h_alu_divs);
					id_is_movem     <= h_is_movem;
					id_movem_dir    <= h_movem_dir;
					id_movem_word   <= h_movem_word;
					id_movem_down   <= h_movem_down;
					id_movem_wb     <= h_movem_wb;
					id_movem_pcrel  <= h_movem_pcrel;
					id_movem_abs    <= h_movem_abs;
					id_is_unlk      <= 1'b0;
					id_is_bsr       <= h_is_bsr;
					id_is_jsr       <= h_is_jsr || (h_is_abs && h_abs_jsr);
					id_is_trap      <= 1'b0;
					id_is_illegal   <= movec_illegal_gather;
					id_illegal_kind <= 2'd0;
					id_is_movesr    <= 1'b0;
					// Kept for an invalid selector too: MOVEC is privileged
					// BEFORE its selector is looked at (ap040_core.v:6053
					// takes go_priv without fetching the extension word), so
					// a user-mode MOVEC with any selector must raise vector 8,
					// and that needs eac_is_priv_capable (milestone 113).
					id_is_movec     <= h_is_movec;
					id_is_rts       <= h_is_rtd;
					id_is_nop       <= 1'b0;
					id_cinv         <= 6'd0;
					id_pmmu         <= 5'd0;
					id_bnt          <= bnt_gather;
					id_is_reset     <= 1'b0;
					id_is_rte       <= 1'b0;
					id_is_rtr       <= 1'b0;
					id_cond         <= h_cond;
					id_fp           <= h_fp;
					id_fp_op        <= h_fp_op;
					id_fp_cmd       <= h_fp_first ? xw : fp_cmd_r;
					id_fp_imm       <= {fp_acc_x[79:0], xw};
					id_fx           <= {fx_valid_now, fx_dst_now, fx_ext_now[7], fx_ext_now[6], fx_ext_now[2],
					                    fx_ext_now[2:0] != 3'd0};
					id_fx_bd        <= fx_bd_now;
					id_fx_od        <= fx_od_now;
					// A reserved encoding is an illegal instruction and nothing else:
					// every flag that would reach memory or a register is cleared.
					if (fx_bad_now) begin
						id_is_illegal   <= 1'b1;
						id_illegal_kind <= 2'd0;
						// ...whose entry writes the new SP: A7, as every other
						// illegal (movec_illegal_gather included) names. Left at the
						// instruction's own destination, the SP went into D4 and the
						// handler ran on the old stack (tb_ap040_pipe_fxdual).
						id_dest_reg     <= 4'd15;
						id_mm <= 7'd0;
						id_moves <= 3'd0;
						id_mvfsr <= 2'd0;
						id_movep <= 3'd0;
						id_ml <= 7'd0;
						id_bf <= 5'd0;
						id_ck2 <= 3'd0;
						id_cas <= 5'd0;
						id_m16 <= 4'd0;
						id_writes_reg <= 1'd0;
						id_writes_ccr <= 1'd0;
						id_is_branch <= 1'd0;
						id_is_scc <= 1'd0;
						id_is_dbcc <= 1'd0;
						id_is_mem_src <= 1'd0;
						id_is_store <= 1'd0;
						// A MOVE whose DESTINATION word is reserved has had its source
						// EA evaluated first in ap040_core.v (ea_start commits an
						// (An)+/-(An) step in S_EA_BASE), and then takes vector 4 at the
						// destination's S_EA_EXTW2: the step stands. No hardware
						// capture settles it either way; the reference is the oracle.
						if (!fx_dst_now) begin
							id_is_postinc <= 1'd0;
							id_is_predec <= 1'd0;
						end
						id_is_jmp <= 1'd0;
						id_is_lea <= 1'd0;
						id_is_rmw <= 1'd0;
						id_immrmw <= 1'd0;
						id_st_only <= 1'b0;
						id_st_disp <= 1'd0;
						id_is_trapcc <= 1'd0;
						id_is_chk <= 1'd0;
						id_is_immsr <= 1'd0;
						id_is_stop <= 1'd0;
						id_is_pea <= 1'd0;
						id_is_link <= 1'd0;
						id_is_div <= 1'd0;
						id_is_movem <= 1'd0;
						id_is_unlk <= 1'd0;
						id_is_bsr <= 1'd0;
						id_is_jsr <= 1'd0;
						id_is_trap <= 1'd0;
						id_is_movesr <= 1'd0;
						id_is_movec <= 1'd0;
						id_is_rts <= 1'd0;
						id_is_nop <= 1'd0;
						id_cinv         <= 6'd0;
						id_pmmu         <= 5'd0;
						id_is_reset <= 1'd0;
						id_is_rte <= 1'd0;
						id_is_rtr <= 1'd0;
						id_fp <= 1'd0;
						id_bnt <= 1'd0;
					end
					ext_pending     <= 3'd0;
					// Decoded at the extension word; its displacements come next.
					if (fx_tail_go && !fx_bad_now) begin
						id_valid     <= 1'b0;
						held_fx_tail <= 1'b1;
						ext_pending  <= fx_more;
					end
				end else begin
					id_valid    <= 1'b0;
					ext_pending <= xp - 3'd1;
					if (fp_extend) begin
						ext_pending <= fp_more;
						held_ext_n  <= h_ext_n + fp_more;
					end
					if (fx_mid_ext) begin
						ext_pending <= xp - 3'd1 + fx_more;
						held_ext_n  <= h_ext_n + fx_more;
					end
				end
				end
			end else begin
				id_valid        <= if_valid;
				id_pc           <= if_pc;
				id_next_pc      <= if_pc + 32'd2;
				id_dest_reg     <= is_xm ? {1'b1, d_reg9} :
				                   is_move_usp ? (usp_to ? 4'd15 : {1'b1, d_rn}) :
				                   is_exg ? {exg_aa, d_reg9} :
				                   (is_move_st || is_movea_rr || is_lea_an || is_adda || is_move_mm_d ||
				                    is_movea_reg || is_movea_memd) ? {1'b1, d_reg9} :
				                    (is_alu_dst || is_bit_d_mem) ? {1'b0, d_reg9} :
				                    is_unlk ? {1'b1, d_rn} :
				                    (is_unary_mem || is_ux_d || is_mvf_memd) ? 4'h0 :
				                    is_mvf_dn ? {1'b0, d_rn} :
				                    is_pea_an ? 4'd15 :
				                    is_chk ? {1'b0, d_reg9} :
				                    (is_mul || is_div) ? {1'b0, d_reg9} :
				                    quick_an_shape  ? {1'b1, d_rn} :
				                    (quick_mem_shape || is_scc_mem_direct) ? 4'd0 :
				                    (is_scc_rr || is_unary_rr || is_extswap_rr || shift_shape || bitop_shape || is_bcd1_rr || quick_shape || is_eor_rr) ? {1'b0, d_rn} :
				                    (is_bsr_byte || is_jsr_an || is_trap || is_illegal || is_movesr || is_rts || is_rte) ? 4'd15 : {1'b0, d_reg9};
				// is_move_mem_l/is_jmp_an/is_jsr_an's src_reg is An, not Dn
				// -- the unified index's top bit (8+n vs 0+n) is the ONLY
				// thing that distinguishes "read this register's value as
				// an operand" (every other instruction so far) from "read
				// this register's value as an ADDRESS" here;
				// ap040_ea_fetch.v's existing operand_a mux (regfile read +
				// EX-forward) doesn't need to know the difference, it just
				// resolves whichever register this is. BSR.B needs no
				// source read at all (see the gather-completion note
				// above), so it's simply absent from this OR-chain. TRAP/
				// illegal need no source read either -- same reasoning as
				// BSR.
				// RTS/RTE need A7 on port A too (not just port B, unlike
				// BSR/TRAP/illegal/MOVE-to-SR/MOVEC-write, which never read
				// A7's value for anything besides the push/exception-frame
				// address) -- they READ A7 to compute where to pop FROM,
				// the same way MOVE.L (An),Dn reads An to compute where to
				// read from, just with the register hardcoded instead of
				// coming from opcode bits. See ap040_ea_fetch.v's header
				// for why RTS deliberately reuses that exact mem_issue/
				// mem_complete path (id_is_mem_src below) instead of
				// getting its own sequencer the way RTE needs.
				id_src_reg      <= is_xm ? {1'b1, d_rn} :
				                   (is_pmmu || is_cinv) ? {1'b1, d_rn} :
				                   (is_move_usp && usp_to) ? {1'b1, d_rn} :
				                   is_exg ? {!exg_dd, d_rn} :
				                   is_tst_an ? {1'b1, d_rn} :
				                   (bitop_shape || is_eor_rr || shift_reg_cnt) ? {1'b0, d_reg9} :
				                    (is_move_mem_l || is_jmp_an || is_jsr_an || is_move_ax || is_alu_mem || is_lea_an || is_pea_an || is_an_src || is_adda_areg_src || is_alu_dst || is_bit_d_mem || is_move_mm_d || is_ux_d || is_movea_memd ||
				                     is_mvto_memd || is_mvf_memd ||
				                     (is_movea_reg && ea_is_an) || is_unlk || is_mul_mem || is_div_mem || is_unary_mem || is_chk_mem || quick_mem_shape || is_scc_mem_direct) ? {1'b1, d_rn} :
				                    (is_rts || is_rte) ? 4'd15 :
				                    is_move_st ? {if_opcode[3], d_rn} : {1'b0, d_rn};
				// Zeroed for is_move_mem_l/is_jmp_an/is_jsr_an/is_rts/is_rte
				// (was the sign-extended opcode low byte for EVERY
				// instruction here, harmless until ap040_ea_fetch.v started
				// adding eac_imm to the address for memory-source
				// instructions -- see header). Still meaningful for MOVEQ.
				// TRAP repurposes this same general-purpose field for its
				// actual vector NUMBER (32+n, not just n -- ap040_ea_fetch.v
				// consumes it as-is, no +32 needed downstream) -- see header.
				// is_move_ax joins the zero list: ea_target is operand_a +
				// eac_imm, and these two modes have no displacement at all.
				// Left at the default -- the sign-extended low opcode byte,
				// which MOVEQ needs -- MOVE.L (A0)+,D0 would read A0 + $18.
				// is_alu_mem joins the zero list for the same reason: ea mode
				// 010 has no displacement, and the default here would make
				// ADD.L (A0),D0 read A0 + $FFFFFF90 -- the sign-extended low
				// byte of its own opcode, which lands back in the
				// instruction stream.
				id_imm          <= is_xm ? 32'h0 :
				                   is_move_usp ? {27'd0, usp_to, `AP040_CREG_USP} :
				                   (is_move_mem_l || is_jmp_an || is_jsr_an || is_rts || is_rte ||
				                    is_move_ax || is_move_st || is_alu_mem || is_lea_an || is_pea_an ||
				                    is_an_src || is_adda || is_alu_dst || is_bit_d_mem || is_move_mm_d || is_unlk || is_mul || is_div || is_unary_mem || is_chk || is_scc_mem_direct ||
				                    is_ux_d || is_movea_reg || is_movea_memd || is_mvto_dn || is_mvto_memd ||
				                    is_mvf_dn || is_mvf_memd || is_exg || is_tst_an) ? 32'h0 :
				                    is_trap ? (32'd32 + {28'd0, if_opcode[3:0]}) :
				                    quick_any ? {28'd0, quick_val} :
				                              {{24{if_opcode[7]}}, if_opcode[7:0]};
				id_shcnt        <= is_ux_d ? 6'd1 : shift_cnt;
				id_shift_reg    <= if_valid && shift_reg_cnt;
				id_alu_op       <= is_xm         ? xm_op :
				                   is_exg        ? `AP040_ALU_EXG :
				                   is_tst_an     ? `AP040_ALU_TST :
				                   is_ux_d       ? ux_op       :
				                   is_bit_d_mem  ? bitop_op    :
				                   is_alu_mem    ? alu_nib_op  :
				                   quick_any     ? quick_op    :
				                   is_bcd1_rr    ? bcd1_op     :
				                   is_bcd2_rr    ? bcd2_op     :
				                   bitop_shape   ? bitop_op    :
				                   shift_shape   ? shift_op    :
				                   is_x_rr       ? x_rr_op     :
				                   (is_alu_an || is_adda) ? alu_nib_op :
				                   is_unary_mem  ? unary_mem_op :
				                   is_mul        ? (is_muls ? `AP040_ALU_MULS : `AP040_ALU_MULU) :
				                   is_alu_dst    ? alu_nib_dst_op :
				                   is_eor_rr     ? `AP040_ALU_EOR :
				                   is_alu_rr     ? alu_rr_op   :
				                   is_unary_rr   ? unary_rr_op :
				                   is_extswap_rr ? extswap_op  : `AP040_ALU_MOVE;
				// Everything else here (MOVEQ, Scc, the memory/branch forms)
				// is Long or drives its own width, so Long stays the default.
				id_size         <= is_xm ? ((is_abcd_m || is_sbcd_m) ? `AP040_SZ_B : if_opcode[7:6]) :
				                   is_exg ? `AP040_SZ_L :
				                   is_tst_an ? add_op_size :
				                   (is_mvto_memd || is_mvf_dn || is_mvf_memd) ? `AP040_SZ_W :
				                   is_ux_d ? ux_size :
				                   is_bit_d_mem ? `AP040_SZ_B :
				                   (is_move_mem_l || is_move_ax || is_move_st || is_move_mm_d) ? move_op_size :
				                   is_scc_mem_direct ? `AP040_SZ_B :
				                   (quick_shape || quick_mem_shape) ? if_opcode[7:6] :
				                   (is_bcd1_rr || is_bcd2_rr) ? `AP040_SZ_B :
				                   is_extswap_rr ? extswap_size :
				                   (is_alu_rr || is_unary_rr || is_x_rr || shift_shape || is_alu_mem || is_alu_an || is_eor_rr || is_alu_dst || is_unary_mem) ? add_op_size :
				                   (is_move_rr || is_move_an) ? move_op_size : `AP040_SZ_L;
				id_src_a_is_imm <= if_valid && (is_moveq || quick_shape || quick_an_shape);
				id_writes_reg   <= if_valid && (is_moveq || is_move_rr || (is_alu_rr && !is_cmp_rr) || (is_alu_mem && !is_cmp_mem) || (is_alu_an && !is_cmp_an) || is_move_an || is_eor_rr || is_x_rr || shift_shape ||
				                               (bitop_shape && !is_btst_rr) ||
				                               (is_move_mm_d && (mvd_pi || mvd_pd)) ||
				                               is_movea_reg || is_movea_memd || is_mvf_dn || is_exg ||
				                               (is_move_usp && !usp_to) || is_xm ||
				                               is_bcd1_rr || is_bcd2_rr || quick_shape || quick_an_shape || (is_unary_rr && !is_tst_rr) || is_extswap_rr || is_scc_rr || is_move_mem_l || is_move_ax || is_movea_rr || is_bsr_byte || is_jsr_an || is_trap || is_illegal || is_rts || is_rte || is_lea_an || is_pea_an || (is_adda && !is_cmpa) || is_unlk || is_mul || is_div);
				id_writes_ccr   <= if_valid && (is_xm || is_moveq || is_move_rr || is_alu_rr || is_alu_mem || is_alu_an || is_move_an || is_eor_rr || is_alu_dst || is_bit_d_mem || is_move_mm_d || is_ux_d || is_tst_an || is_unary_rr || is_extswap_rr || is_x_rr || shift_shape || bitop_shape || is_bcd1_rr || is_bcd2_rr || quick_shape || quick_mem_shape || is_move_mem_l || is_move_ax || is_move_st || is_cmpa || is_mul || is_div || is_unary_mem);
				id_is_branch    <= if_valid && is_branch_byte;
				id_is_scc       <= if_valid && (is_scc_rr || is_scc_mem_direct);
				id_is_dbcc      <= 1'b0;
				// RTS reuses the SAME mem_issue/mem_complete FSM MOVE.L
				// (An),Dn already built -- see ap040_ea_fetch.v's header --
				// so it needs id_is_mem_src just like is_move_mem_l does.
				// RTE deliberately does NOT: it needs its own 2-beat
				// sequencer (SR+PC+format, not a single 32-bit value), and
				// (unlike RTS) a dynamic privilege check that must take
				// priority over any read at all.
				// An RMW is a memory SOURCE as well as a destination: the read
				// half uses the same mem_issue/mem_complete path everything
				// else does.
				id_is_mem_src   <= if_valid && (is_xm || is_move_mem_l || is_rts || is_move_ax || is_alu_mem ||
				                               is_adda_mem || is_alu_dst || is_bit_d_mem || is_move_mm_d || is_ux_d || is_movea_memd ||
				                               is_mvto_memd || is_mvf_memd ||
				                               is_unlk || is_mul_mem || is_div_mem ||
				                               is_unary_mem || is_chk_mem || quick_mem_shape || is_scc_mem_direct);
				id_is_abs       <= 1'b0;
				id_is_store     <= if_valid && is_move_st;
				id_is_postinc   <= if_valid && (is_cmpm || is_move_pi || is_move_st_pi || is_alu_pi || is_adda_pi || is_alu_dst_pi || (is_bit_d_mem && ea_is_pi) || (is_move_mm_d && ea_is_pi) || ((is_ux_d || is_movea_memd || is_mvto_memd || is_mvf_memd) && ea_is_pi) || is_mul_pi || is_div_pi || is_unary_mem_pi || is_chk_pi || is_quick_mem_pi || is_scc_mem_pi);
				id_is_predec    <= if_valid && ((is_xm && !is_cmpm) || is_move_pd || is_move_st_pd || is_alu_pd || is_adda_pd || is_alu_dst_pd || (is_bit_d_mem && ea_is_pd) || (is_move_mm_d && ea_is_pd) || ((is_ux_d || is_movea_memd || is_mvto_memd || is_mvf_memd) && ea_is_pd) || is_mul_pd || is_div_pd || is_unary_mem_pd || is_chk_pd || is_quick_mem_pd || is_scc_mem_pd);
				id_is_jmp       <= if_valid && is_jmp_an;
				// No memory access, no condition codes: the whole
				// instruction is ea_target landing in An via ALU_MOVE.
				id_is_lea       <= if_valid && is_lea_an;
				id_is_pea       <= if_valid && is_pea_an;
				id_is_chk       <= if_valid && is_chk;
				id_chk_long     <= if_valid && is_chk && chk_is_long;
				id_is_trapcc    <= if_valid && (is_trapcc || is_trapv);
				id_is_immsr     <= if_valid && (is_mvto_dn || is_mvto_memd);
				id_is_stop      <= 1'b0;
				id_immsr_to_sr  <= if_valid && is_mvto_memd && mvto_sr;
				id_mvfsr        <= {if_valid && (is_mvf_dn || is_mvf_memd), mvf_ccr};
				id_pc_off       <= 2'd0;
				id_movep        <= 3'd0;
				id_ml           <= 7'd0;
				id_bf           <= 5'd0;
				id_ck2          <= 3'd0;
				id_cas          <= 5'd0;
				id_m16          <= 4'd0;
				// id_size stays Long for the ALU; this forces the READ (and
				// the auto-increment step) to Word and sign-extends it.
				// Long for the ALU and the writeback; Word for the memory read
				// and the autoincrement step. See the header above for why the
				// sign extension it also implies does not make MULU signed.
				// CHK.L reads a full longword and sign-extends nothing.
				id_sxt_w        <= if_valid && (is_adda_w || is_mul || is_div ||
				                               ((is_movea_reg || is_movea_memd) && movea_w) ||
				                               (is_chk && !chk_is_long));
				// Scc to memory rides the read-modify-write store path
				// (milestone 107): its byte is not known until EX, where the
				// condition is evaluated, so it cannot use the EA-fetch
				// store that MOVE-to-memory uses. Like CLR above it performs
				// a read its result does not need -- noted rather than
				// silently divergent; the 68040 does not.
				id_is_rmw       <= if_valid && (is_alu_dst || is_bit_d_mem || is_ux_d || is_unary_rmw || quick_mem_shape || is_scc_mem_direct ||
				                               is_mvf_memd);
				id_immrmw       <= if_valid && quick_mem_shape;
				id_st_only      <= if_valid && (is_clr_mem || is_scc_mem_direct);
				id_ea_ext       <= 32'h0;
				id_mm           <= is_xm ? {if_valid, if_valid, 1'b0, is_cmpm, !is_cmpm, 2'b00}
				                         : {1'b0, is_move_mm_d && if_valid, 1'b0, mvd_pi, mvd_pd, 2'b00};
				id_moves        <= 3'd0;
				id_st_disp      <= 1'b0;
				id_ea_indexed   <= 1'b0;
				id_ea_pcrel     <= 1'b0;
				id_is_link      <= 1'b0;
				id_is_div       <= if_valid && is_div;
				id_div_signed   <= if_valid && is_divs;
				id_is_movem     <= 1'b0;
				id_movem_dir    <= 1'b0;
				id_movem_word   <= 1'b0;
				id_movem_down   <= 1'b0;
				id_movem_wb     <= 1'b0;
				id_movem_pcrel  <= 1'b0;
				id_movem_abs    <= 1'b0;
				id_movem_mask   <= 16'h0;
				id_is_unlk      <= if_valid && is_unlk;
				id_is_bsr       <= if_valid && is_bsr_byte;
				id_is_jsr       <= if_valid && is_jsr_an;
				id_is_trap      <= if_valid && is_trap;
				id_is_illegal   <= if_valid && is_illegal;
				id_illegal_kind <= is_aline ? 2'd1 : is_fline ? 2'd2 : 2'd0;
				id_is_movesr    <= if_valid && is_movesr;
				// MOVEC always gathers; MOVE USP is MOVEC without the word.
				id_is_movec     <= if_valid && is_move_usp;
				id_is_rts       <= if_valid && is_rts;
				id_is_nop       <= if_valid && is_nop;
				id_cinv         <= {if_valid && is_cinv, if_opcode[7:3]};
				id_pmmu         <= {if_valid && is_pmmu, is_ptest, !if_opcode[5], if_opcode[4:3]};
				id_bnt          <= if_valid && bnt_byte;
				id_is_reset     <= if_valid && is_reset;
				id_is_rte       <= if_valid && is_rte;
				id_is_rtr       <= if_valid && is_rtr;
				id_cond         <= is_trapv ? 4'h9 : if_opcode[11:8];   // TRAPV: VS
				// FSAVE/FRESTORE through Dn, An, (An), (An)+ or -(An): no words
				id_fp           <= is_fp;
				id_fp_op        <= if_opcode[8:0];
				id_fp_cmd       <= 16'd0;
				id_fp_imm       <= 96'd0;
				id_fx           <= 6'd0;
				// A fetch fault, as the ILLEGAL membus substituted: the
				// instruction it belongs to starts where the gather did, if one
				// was in progress, and the frame's fault address is this word.
				id_fflt         <= {if_flt, if_flt_bus, gathering ? (if_pc[4:1] - held_pc[4:1]) : 4'd0};
				if (if_flt && gathering) begin
					id_pc        <= held_pc;
					ext_pending  <= 3'd0;
					held_fx_tail <= 1'b0;
				end
			end
		end else if (!stall_in) begin
			// A fetch bubble (milestone 80): the L1 has not returned the next
			// word. Nothing is consumed -- a gather in progress keeps its
			// count and its held words -- and nothing is issued. With the
			// one-cycle L1 a word arrived every unstalled cycle, so the
			// gather counted cycles; the slow L1 build's first seven benches
			// showed it consuming the held opcode as its own immediate.
			id_valid <= 1'b0;
		end
	end
end

endmodule
