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

	output            id_stall,   // to IF: no local stall of its own yet

	// Bcc/BRA redirect, combinational -- see header comment
	output            id_redirect_valid,
	output     [31:0] id_redirect_pc,

	output reg        id_valid,
	output reg [31:0] id_pc,
	output reg [31:0] id_next_pc,
	output reg  [3:0] id_dest_reg,
	output reg  [3:0] id_src_reg,
	output reg [31:0] id_imm,
	output reg  [5:0] id_alu_op,
	output reg        id_src_a_is_imm,
	output reg        id_writes_reg,
	output reg        id_writes_ccr,
	output reg        id_is_branch,
	output reg        id_is_scc,
	output reg        id_is_dbcc,
	output reg        id_is_mem_src,
	output reg        id_is_jmp,
	output reg        id_is_bsr,
	output reg        id_is_jsr,
	output reg        id_is_trap,
	output reg        id_is_illegal,
	output reg        id_is_movesr,
	output reg        id_is_movec,
	output reg        id_is_rts,
	output reg        id_is_rte,
	output reg  [3:0] id_cond
);

assign id_stall = stall_in;

// ap040_core.v:667-670 equivalents. All the opcodes decoded so far happen
// to share these same two field positions, except Scc (see header comment).
wire [2:0] d_reg9 = if_opcode[11:9];   // MOVEQ/MOVE.L/ADD.L dest Dn
wire [2:0] d_rn   = if_opcode[2:0];    // MOVE.L/ADD.L src Dn; Scc dest Dn

// MOVEQ: 0111 rrr 0 iiiiiiii  (rrr = dest Dn, iiiiiiii = 8-bit immediate)
wire is_moveq = (if_opcode[15:12] == 4'b0111) && (if_opcode[8] == 1'b0);

// MOVE.L Dn,Dm: 00 10 RRR 000 000 rrr (RRR = dest Dn, rrr = src Dn,
// dest/src EA modes both 000 = data-register-direct; no other EA mode yet).
// The size field (if_opcode[13:12]) is matched against the raw '10' = Long
// encoding rather than ap040_core.v's move_size decode (ap040_core.v:673-
// 674) because byte/word MOVE aren't implemented yet; switch to that
// convention when they are.
wire is_move_rr = (if_opcode[15:14] == 2'b00) && (if_opcode[13:12] == 2'b10) &&
                   (if_opcode[8:6]  == 3'b000) && (if_opcode[5:3]  == 3'b000);

// ADD.L Dn,Dm: 1101 RRR 0 10 000 rrr (RRR = dest Dm, rrr = src Dn; ir[8]=0
// selects the "<ea>+Dn->Dn" direction, ir[7:6]=std_size=10=Long, ir[5:3]=000
// = ea is Dn direct). Verified bit-for-bit against ap040_core.v's own
// SUB/ADD decode (ap040_core.v:4928-4974, d_op8_6/std_size at :668,:675),
// not guessed. No other size/direction/EA mode yet.
wire is_add_rr = (if_opcode[15:12] == 4'b1101) && (if_opcode[8] == 1'b0) &&
                  (if_opcode[7:6]  == 2'b10)    && (if_opcode[5:3] == 3'b000);

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
wire is_scc_rr = (if_opcode[15:12] == 4'b0101) && (if_opcode[7:6] == 2'b11) &&
                  (if_opcode[5:3]  == 3'b000);

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
wire is_move_mem_l = (if_opcode[15:12] == 4'b0010) &&
                      (if_opcode[8:6]  == 3'b000) &&
                      (if_opcode[5:3]  == 3'b010);

// MOVE.L (d16,An),Dn: 0010 DDD 000 101 aaa -- same shape as is_move_mem_l
// above, mode field 101 instead of 010 (verified against the same
// ap040_core.v:5021-5052 MOVE decode -- d_mode==101 is SK_MEM there too,
// just a different EA mode within the same general machinery). Always
// carries exactly one 16-bit extension word (the displacement) -- routed
// through the shared gather state machine below, not a parallel one.
wire is_move_disp = (if_opcode[15:12] == 4'b0010) &&
                     (if_opcode[8:6]  == 3'b000) &&
                     (if_opcode[5:3]  == 3'b101);

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

// TRAP #n: 0100 1110 0100 nnnn (0x4E40-0x4E4F) -- see header. Distinct
// if_opcode[15:6] value (10'b0100111001) from both JSR's (...010) and JMP's
// (...011), so no overlap is possible with either.
wire is_trap = (if_opcode[15:4] == 12'h4E4);

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
wire [11:0] movec_raw_sel = if_opcode[11:0];
wire        movec_sel_valid = (movec_raw_sel == 12'h000) || (movec_raw_sel == 12'h001) ||
                               (movec_raw_sel == 12'h002) || (movec_raw_sel == 12'h801) ||
                               (movec_raw_sel == 12'h800) || (movec_raw_sel == 12'h804) ||
                               (movec_raw_sel == 12'h803);
wire  [2:0] movec_sel_code = (movec_raw_sel == 12'h000) ? `AP040_CREG_SFC  :
                              (movec_raw_sel == 12'h001) ? `AP040_CREG_DFC  :
                              (movec_raw_sel == 12'h002) ? `AP040_CREG_CACR :
                              (movec_raw_sel == 12'h801) ? `AP040_CREG_VBR  :
                              (movec_raw_sel == 12'h800) ? `AP040_CREG_USP  :
                              (movec_raw_sel == 12'h804) ? `AP040_CREG_ISP  :
                                                            `AP040_CREG_MSP;  // 12'h803
wire  [3:0] movec_gpr = {if_opcode[15], if_opcode[14:12]};

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
wire is_rte = (if_opcode == 16'h4E73);

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
wire is_illegal = !is_nop && !is_moveq && !is_move_rr && !is_add_rr &&
                   !is_branch_byte && !is_scc_rr && !is_move_mem_l &&
                   !is_jmp_an && !is_bsr_byte && !is_jsr_an && !is_trap &&
                   !is_movesr && !is_movec_opcode && !is_rts && !is_rte;

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
reg  [1:0]  ext_pending;
reg         held_is_long;
reg         held_is_dbcc;
reg         held_is_move_disp;
reg         held_is_jmp;
reg         held_is_bsr;
reg         held_is_jsr;
reg         held_is_movec;
reg         held_movec_dir;
reg  [2:0]  held_reg;
reg  [2:0]  held_dest_reg;
reg  [31:0] held_pc;
reg  [3:0]  held_cond;
reg  [31:0] disp_acc;

wire completing_gather = (ext_pending == 2'd1);

// The full displacement as of the completing cycle: word form sign-extends
// if_opcode alone (nothing was usefully shifted into disp_acc for a 1-word
// gather); long form combines the word shifted in last cycle with this
// cycle's word, high-word-first (see header comment).
wire [31:0] gather_disp = held_is_long ? {disp_acc[15:0], if_opcode}
                                       : {{16{if_opcode[15]}}, if_opcode};

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
wire redirect_from_byte   = if_valid && (is_branch_byte || is_bsr_byte) && (ext_pending == 2'd0);
wire redirect_from_gather = completing_gather && !held_is_move_disp && !held_is_jmp &&
                             !held_is_jsr && !held_is_movec;

// MOVEC gather-completion helper: an otherwise-recognized MOVEC whose
// extension-word selector names something this core doesn't model (the MMU
// registers, out of scope -- see movec_sel_valid's own comment) becomes
// illegal, exactly the same vector/format id_is_illegal already drives for
// any other unrecognized opcode.
wire movec_illegal_gather = held_is_movec && !movec_sel_valid;

assign id_redirect_valid = redirect_from_byte || redirect_from_gather;
assign id_redirect_pc    = redirect_from_gather
                          ? (held_pc + 32'd2 + gather_disp)
                          : (if_pc   + 32'd2 + {{24{if_opcode[7]}}, if_opcode[7:0]});

always @(posedge clk) begin
	if (!nreset) begin
		id_valid        <= 1'b0;
		id_pc           <= 32'h0;
		id_next_pc      <= 32'h0;
		id_dest_reg     <= 4'h0;
		id_src_reg      <= 4'h0;
		id_imm          <= 32'h0;
		id_alu_op       <= 6'h0;
		id_src_a_is_imm <= 1'b0;
		id_writes_reg   <= 1'b0;
		id_writes_ccr   <= 1'b0;
		id_is_branch    <= 1'b0;
		id_is_scc       <= 1'b0;
		id_is_dbcc      <= 1'b0;
		id_is_mem_src   <= 1'b0;
		id_is_jmp       <= 1'b0;
		id_is_bsr       <= 1'b0;
		id_is_jsr       <= 1'b0;
		id_is_trap      <= 1'b0;
		id_is_illegal   <= 1'b0;
		id_is_movesr    <= 1'b0;
		id_is_movec     <= 1'b0;
		id_is_rts       <= 1'b0;
		id_is_rte       <= 1'b0;
		id_cond         <= 4'h0;
		ext_pending     <= 2'd0;
		held_is_long    <= 1'b0;
		held_is_dbcc    <= 1'b0;
		held_is_move_disp <= 1'b0;
		held_is_jmp     <= 1'b0;
		held_is_bsr     <= 1'b0;
		held_is_jsr     <= 1'b0;
		held_is_movec   <= 1'b0;
		held_movec_dir  <= 1'b0;
		held_reg        <= 3'h0;
		held_dest_reg   <= 3'h0;
		held_pc         <= 32'h0;
		held_cond       <= 4'h0;
		disp_acc        <= 32'h0;
	end else if (ce) begin
		if (flush) begin
			id_valid    <= 1'b0;
			ext_pending <= 2'd0;   // abandon any in-progress gather too
		end else if (!stall_in) begin
			if (ext_pending != 2'd0) begin
				// Gathering: if_opcode is extension-word data, never a
				// fresh opcode.
				disp_acc <= {disp_acc[15:0], if_opcode};
				if (completing_gather) begin
					id_valid        <= 1'b1;
					id_pc           <= held_pc;
					// Unconditional regardless of movec_illegal_gather: this
					// is genuinely "the next instruction's address" -- the
					// extension word was consumed either way, so the gather
					// is the same width whether or not its selector turned
					// out valid. id_is_illegal's own exception path reads
					// id_pc (held_pc, already set above), not id_next_pc, for
					// its stacked PC -- see ap040_ea_fetch.v's header.
					id_next_pc      <= held_pc + 32'd2 + (held_is_long ? 32'd4 : 32'd2);
					id_dest_reg     <= held_is_dbcc ? {1'b0, held_reg} :
					                    held_is_move_disp ? {1'b0, held_dest_reg} :
					                    (held_is_bsr || held_is_jsr) ? 4'd15 :
					                    held_is_movec ? (held_movec_dir ? 4'd15 : movec_gpr) : 4'h0;
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
					id_src_reg      <= held_is_dbcc ? {1'b0, held_reg} :
					                    (held_is_move_disp || held_is_jmp || held_is_jsr) ? {1'b1, held_reg} :
					                    (held_is_movec && held_movec_dir) ? movec_gpr : 4'h0;
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
					id_imm          <= (held_is_move_disp || held_is_jmp || held_is_jsr) ? gather_disp :
					                    held_is_movec ? {28'd0, held_movec_dir, movec_sel_code} : 32'h0;
					id_alu_op       <= `AP040_ALU_MOVE;
					id_src_a_is_imm <= 1'b0;
					// DBcc's write is dynamic (see header); BSR/JSR's is
					// static -- both always decrement A7 when they execute
					// at all. MOVEC's read direction writes a real GPR
					// (Rn <- the selected control register); its write
					// direction writes no GPR at all (the control register
					// write happens via ap040_execute.v's new exe_writes_
					// creg path, not commit_reg) -- and an invalid selector
					// writes nothing either, having already become illegal
					// above.
					id_writes_reg   <= held_is_move_disp || held_is_bsr || held_is_jsr ||
					                    (held_is_movec && !held_movec_dir && !movec_illegal_gather);
					id_writes_ccr   <= held_is_move_disp;
					id_is_branch    <= !held_is_dbcc && !held_is_move_disp && !held_is_jmp &&
					                    !held_is_bsr && !held_is_jsr && !held_is_movec;
					id_is_scc       <= 1'b0;
					id_is_dbcc      <= held_is_dbcc;
					id_is_mem_src   <= held_is_move_disp;
					id_is_jmp       <= held_is_jmp;
					id_is_bsr       <= held_is_bsr;
					id_is_jsr       <= held_is_jsr;
					id_is_trap      <= 1'b0;
					id_is_illegal   <= movec_illegal_gather;
					id_is_movesr    <= 1'b0;
					id_is_movec     <= held_is_movec && !movec_illegal_gather;
					id_is_rts       <= 1'b0;
					id_is_rte       <= 1'b0;
					id_cond         <= held_cond;
					ext_pending     <= 2'd0;
				end else begin
					id_valid    <= 1'b0;
					ext_pending <= ext_pending - 2'd1;
				end
			end else if (is_branch_word || is_branch_long || is_dbcc || is_move_disp || is_jmp_disp ||
			              is_bsr_word || is_bsr_long || is_jsr_disp || is_movec_opcode) begin
				// Opcode word of a word/long-form branch, a DBcc,
				// MOVE.L (d16,An),Dn, JMP (d16,An), a word/long-form BSR,
				// JSR (d16,An), or MOVEC (all word-form except long-branch/
				// long-BSR): not a complete instruction yet -- hold what we
				// know, start gathering.
				id_valid      <= 1'b0;
				held_pc       <= if_pc;
				held_cond     <= if_opcode[11:8];
				held_is_long  <= is_branch_long || is_bsr_long;
				held_is_dbcc  <= is_dbcc;
				held_is_move_disp <= is_move_disp;
				held_is_jmp   <= is_jmp_disp;
				held_is_bsr   <= is_bsr_word || is_bsr_long;
				held_is_jsr   <= is_jsr_disp;
				held_is_movec <= is_movec_opcode;
				held_movec_dir<= if_opcode[0];   // MOVEC's direction bit lives
				                                  // in the OPCODE word, not the
				                                  // extension word -- see header.
				held_reg      <= if_opcode[2:0];
				held_dest_reg <= if_opcode[11:9];
				ext_pending   <= (is_branch_long || is_bsr_long) ? 2'd2 : 2'd1;
			end else begin
				id_valid        <= if_valid;
				id_pc           <= if_pc;
				id_next_pc      <= if_pc + 32'd2;
				id_dest_reg     <= is_scc_rr ? {1'b0, d_rn} :
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
				id_src_reg      <= (is_move_mem_l || is_jmp_an || is_jsr_an) ? {1'b1, d_rn} :
				                    (is_rts || is_rte) ? 4'd15 : {1'b0, d_rn};
				// Zeroed for is_move_mem_l/is_jmp_an/is_jsr_an/is_rts/is_rte
				// (was the sign-extended opcode low byte for EVERY
				// instruction here, harmless until ap040_ea_fetch.v started
				// adding eac_imm to the address for memory-source
				// instructions -- see header). Still meaningful for MOVEQ.
				// TRAP repurposes this same general-purpose field for its
				// actual vector NUMBER (32+n, not just n -- ap040_ea_fetch.v
				// consumes it as-is, no +32 needed downstream) -- see header.
				id_imm          <= (is_move_mem_l || is_jmp_an || is_jsr_an || is_rts || is_rte) ? 32'h0 :
				                    is_trap ? (32'd32 + {28'd0, if_opcode[3:0]}) :
				                              {{24{if_opcode[7]}}, if_opcode[7:0]};
				id_alu_op       <= is_add_rr ? `AP040_ALU_ADD : `AP040_ALU_MOVE;
				id_src_a_is_imm <= if_valid && is_moveq;
				id_writes_reg   <= if_valid && (is_moveq || is_move_rr || is_add_rr || is_scc_rr || is_move_mem_l || is_bsr_byte || is_jsr_an || is_trap || is_illegal || is_rts || is_rte);
				id_writes_ccr   <= if_valid && (is_moveq || is_move_rr || is_add_rr || is_move_mem_l);
				id_is_branch    <= if_valid && is_branch_byte;
				id_is_scc       <= if_valid && is_scc_rr;
				id_is_dbcc      <= 1'b0;
				// RTS reuses the SAME mem_issue/mem_complete FSM MOVE.L
				// (An),Dn already built -- see ap040_ea_fetch.v's header --
				// so it needs id_is_mem_src just like is_move_mem_l does.
				// RTE deliberately does NOT: it needs its own 2-beat
				// sequencer (SR+PC+format, not a single 32-bit value), and
				// (unlike RTS) a dynamic privilege check that must take
				// priority over any read at all.
				id_is_mem_src   <= if_valid && (is_move_mem_l || is_rts);
				id_is_jmp       <= if_valid && is_jmp_an;
				id_is_bsr       <= if_valid && is_bsr_byte;
				id_is_jsr       <= if_valid && is_jsr_an;
				id_is_trap      <= if_valid && is_trap;
				id_is_illegal   <= if_valid && is_illegal;
				id_is_movesr    <= if_valid && is_movesr;
				id_is_movec     <= 1'b0;   // MOVEC never reaches this branch -- it always gathers
				id_is_rts       <= if_valid && is_rts;
				id_is_rte       <= if_valid && is_rte;
				id_cond         <= if_opcode[11:8];
			end
		end
	end
end

endmodule
