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

// Milestone 81 split this module in two. ap040_pipe_cpu.v is the pipeline
// itself, with its two memory ports at its boundary; ap040_pipe_core.v is
// the thin wrapper that pairs it with ap040_pipe_l1.v's array, which is what
// every tb_ap040_pipe_*.v instantiates and preloads. ap040_pipe_sys.v pairs
// the same CPU with ap040_pipe_membus.v and an external bus instead.
//
// The ports below are ap040_pipe_l1.v's own protocol, unchanged: a request
// (l1_req_a / l1_rd_b) and a return (l1_rvalid_a / l1_rvalid_b) per port,
// byte addresses, one outstanding read on port B, a posted write buffer
// behind l1_wr_busy. What is on the other side of them is not this module's
// business any more.
module ap040_pipe_cpu
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10,
	// 1: reset exception processing reads the initial ISP and PC from $0
	// and $4, as a 68040 does; 0: the fetch starts at PC_RESET with ISP
	// zero, which every milestone bench was written against.
	parameter         RESET_VECTORS = 0
)
(
	input  clk,
	input  nreset,
	input  ce,
	input  [2:0] irq_lvl,   // the requested interrupt level, active high; 0 none (2026-09-24)

	// instruction port: 16-bit reads
	output [31:0] l1_addr_a,
	output        l1_req_a,
	input  [15:0] l1_rdata_a,
	input  [15:0] l1_rdata_a2,     // the word after it (phase 8)
	input         l1_rvalid_a,

	// data port: 32-bit reads, sized writes
	output [31:0] l1_addr_b,
	output        l1_rd_b,
	output        l1_wren_b,
	// The privilege of the port-B access (milestone 92), and of the
	// instruction fetch (milestone 93).
	output        l1_sup_b,
	output        l1_fc_ovr,    // MOVES: l1_fc_val is the function code of this access
	output  [2:0] l1_fc_val,
	output        l1_sup_a,
	output  [1:0] l1_size_b,
	output [31:0] l1_data_b,
	input         l1_wr_busy,   // for a write presented this cycle; read only then (see ap040_pipe_membus.v wr_busy_w)
	output        l1_inval_a,   // empty the prefetch stream: CINV/CPUSH's refetch
	// Access faults from the memory side (2026-09-24): with the return, the
	// fetch or read faulted; the tentative write being presented faulted;
	// and which kind it was (a physical bus error rather than the MMU).
	input         l1_rflt_a,
	input         l1_rflt_a_bus,  // ...the fetch's was a physical bus error
	input         l1_rflt_b,
	input         l1_wflt,
	input         l1_flt_bus,
	input         l1_flt_ma,    // ...past a page boundary a transfer crossed
	output        l1_wr_sync,   // translation can refuse a data write now
	// The MMU's registers, for rtl/ap040/ap040_mmu.v beside the bus.
	output [31:0] mmu_tc, mmu_urp, mmu_srp, mmu_itt0, mmu_itt1, mmu_dtt0, mmu_dtt1,
	// PTEST/PFLUSH, to the MMU's sidebands; the memory side's idle.
	input         l1_idle,
	output        l1_quiet,     // start no fetch: the MMU's registers or ATC are changing
	output        l1_wr_drop,   // no write presented, in a ce cycle: a refusal has been seen
	output        pt_req, pt_write,
	output [31:0] pt_addr,
	output  [2:0] pt_fc,
	input         pt_done,
	input  [31:0] pt_mmusr,
	output        pf_req,
	output  [1:0] pf_mode,
	output [31:0] pf_addr,
	output  [2:0] pf_fc,
	input         pf_done,
	// CINV/CPUSH at the caches (ap040_ea_fetch.v), and CACR's enables
	output        cm_req,
	output        cm_ic, cm_dc, cm_push,
	output  [1:0] cm_scope,
	output [31:0] cm_addr,
	input         cm_done,
	// a write-back's bus error, held by the DMU until EA-fetch takes it
	// (caches stage R; ap040_ea_fetch.v's pae_hold): tie pw_pend low where
	// nothing reports one
	input         pw_pend,
	input  [15:0] pw_ssw,
	input  [31:0] pw_fa,
	input   [7:0] pw_wb1s,
	input [127:0] pw_pd,
	input         pw_exc,
	output        pw_ack,
	output        ic_en,        // CACR IE
	output        dc_en,        // CACR DE
	// for the card's wrapper (caches stage F; ap040_tg68k_compat's own):
	// CACR and VBR as they stand, RESET driving the reset line, and one
	// pulse for each level 7 interrupt taken
	output [31:0] cacr_out,
	output [31:0] vbr_out,
	output        reset_out,
	output        nmi_ack,
	output        halted_out,   // a double fault: halted until reset
	// what a port-B access is, for the data cache: no allocation, MOVE16,
	// locked (see ap040_ea_fetch.v)
	output        l1_nalloc_b,
	output        l1_m16_b,
	output        l1_lock_b,
	input  [31:0] l1_q_b,
	input         l1_rvalid_b,

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
	output [15:0] dbg_sr,
	output [31:0] dbg_commits
);

wire        if_valid;  wire [31:0] if_pc;  wire [15:0] if_opcode;
wire        if_valid2; wire [15:0] if_word2; wire if_flt, if_flt_bus, if_flt2, id_take2;
wire [31:0] if_end;

wire        id_valid;  wire [31:0] id_pc;  wire [31:0] id_next_pc;
wire  [3:0] id_dest_reg, id_src_reg;
wire [31:0] id_imm;
wire [31:0] id_ea_ext;
wire  [6:0] id_mm;
wire  [2:0] id_moves;
wire  [1:0] id_mvfsr;
wire  [1:0] id_pc_off;
wire  [2:0] id_movep;
wire  [6:0] id_ml;
wire  [4:0] id_bf;
wire  [2:0] id_ck2;
wire  [4:0] id_cas;
wire  [3:0] id_m16;
wire        id_fp;
wire  [8:0] id_fp_op;
wire [15:0] id_fp_cmd;
wire [95:0] id_fp_imm;
wire  [5:0] id_fx;
wire [31:0] id_fx_bd, id_fx_od;
wire  [5:0] id_alu_op;
wire  [1:0] id_size;
wire  [5:0] id_shcnt;
wire        id_shift_reg;
wire        id_is_abs;
wire        id_is_postinc, id_is_predec;
wire        id_is_store;
wire        eac_is_store;
// L1 port B has two producers since milestone 48. ap040_ea_fetch.v drives
// every read and every store issued from a register; ap040_execute.v drives
// the store half of a read-modify-write, whose data is the ALU result and
// therefore does not exist a stage earlier. EX wins -- it is the older
// instruction -- and port_taken tells EA-fetch to behave as if it had never
// been fetched that cycle.
wire        eaf_l1_wren_b;
wire  [1:0] eaf_l1_size_b;
wire [31:0] eaf_l1_data_b;
wire        ex_st_req;
wire [31:0] ex_st_addr, ex_st_data;
wire  [1:0] ex_st_size;
wire        eac_is_abs;
wire        eac_is_postinc, eac_is_predec;
wire        eaf_writes_an;
wire  [1:0] eaf_an_sel;
wire  [3:0] eaf_an_reg;
wire [31:0] eaf_an_data;
wire  [3:0] exe_dest_reg2;
wire [31:0] exe_result_data2;
wire        exe_writes_reg2;
wire  [1:0] exe_an_sel;
wire        ex_creg_sp;
wire        ex_creg_any;
wire        ex_an_early_we;
wire  [3:0] ex_an_early_reg;
wire [31:0] ex_an_early_data;
wire  [1:0] ex_an_early_sel;
wire        ex_st_sup;
wire        ex_aerr;      // EX's store was refused: abandoned, and owed (ap040_ea_fetch.v)
wire        ex_aerr_rd;   // ...or its pipelined load's read faulted
wire        ex_ld_busy;   // EX's pipelined load is outstanding: port B is its (phase 5)
wire        eaf_ld_pend, eaf_ld_sxw;
// A write to A7 that has not landed in the register file yet: one in EX
// through either port, or one committing this cycle, whose value the file
// only shows from the NEXT cycle. MOVEC counts twice over -- aux_we is its
// COMMIT, and ex_creg_sp is the cycle before that, while it is still in EX.
// Listing only the commit left an exception one instruction behind a MOVEC
// building its frame on the stack that MOVEC had just replaced.
wire        a7_busy = (ex_fwd_valid  && (ex_fwd_dest  == 4'd15)) ||
                      (ex_fwd2_valid && (ex_fwd2_dest == 4'd15)) ||
                      (commit_reg    && (exe_dest_reg  == 4'd15)) ||
                      (commit_reg2   && (exe_dest_reg2 == 4'd15)) ||
                      (ex_an_early_we && (ex_an_early_reg == 4'd15)) ||
                      aux_we || ex_creg_sp;
wire        ex_fwd2_valid;
wire        ex_fwd2_slow;
wire        ex_res_slow;       // EX holds a staged MUL.L: its result is due from WB
wire  [3:0] ex_fwd2_dest;
wire [31:0] ex_fwd2_data;
wire        id_src_a_is_imm, id_writes_reg, id_writes_ccr;
wire        id_is_branch, id_is_scc, id_is_dbcc, id_is_mem_src, id_is_jmp;
wire        id_st_only, eac_st_only;
wire        id_is_lea, id_sxt_w, id_is_rmw, id_immrmw, id_st_disp, id_is_link, id_is_unlk, id_ea_indexed, id_ea_pcrel, id_is_pea, id_is_immsr, id_immsr_to_sr, id_is_chk, id_is_trapcc;
wire        id_is_movem, id_movem_dir, id_movem_word, id_movem_down, id_movem_wb, id_movem_pcrel, id_movem_abs;
wire [15:0] id_movem_mask, eac_movem_mask;
wire        id_is_div, id_div_signed;
wire        id_chk_long, eac_chk_long;
wire        id_is_bsr, id_is_jsr, id_is_trap, id_is_illegal;
wire  [1:0] id_illegal_kind, eac_illegal_kind;
wire        id_is_movesr, id_is_movec;
wire        id_is_rts, id_is_rte, id_is_nop, id_is_reset, id_is_rtr, id_bnt;
wire  [3:0] id_cond;

wire        eac_valid; wire [31:0] eac_pc; wire [31:0] eac_next_pc;
wire  [3:0] eac_dest_reg, eac_src_reg;
wire [31:0] eac_imm;
wire [31:0] eac_ea_ext;
wire  [6:0] eac_mm;
wire  [2:0] eac_moves;
wire  [1:0] eac_mvfsr;
wire [31:0] eac_pc_base;
wire  [2:0] eac_movep;
wire  [6:0] eac_ml;
wire  [4:0] eac_bf;
wire  [2:0] eac_ck2;
wire  [4:0] eac_cas;
wire  [3:0] eac_m16;
wire        eac_fp;
wire  [8:0] eac_fp_op;
wire [15:0] eac_fp_cmd;
wire [95:0] eac_fp_imm;
wire  [5:0] eac_fx;
wire [31:0] eac_fx_bd, eac_fx_od;
wire  [5:0] eac_alu_op;
wire  [1:0] eac_size;
wire  [5:0] eac_shcnt;
wire        eac_shift_reg;
wire        eac_src_a_is_imm, eac_writes_reg, eac_writes_ccr;
wire        eac_is_branch, eac_is_scc, eac_is_dbcc, eac_is_mem_src, eac_is_jmp;
wire        eac_is_lea, eac_sxt_w, eac_is_rmw, eac_immrmw, eac_st_disp, eac_is_link, eac_is_unlk, eac_ea_indexed, eac_ea_pcrel, eac_is_pea, eac_is_immsr, eac_immsr_to_sr, eac_is_chk, eac_is_trapcc;
wire        eaf_is_trapcc, eaf_is_fmterr, eaf_is_trace;
wire        eaf_is_chk;
wire        eaf_chk_ok;
wire        eaf_is_immsr, eaf_immsr_to_sr;
wire        id_is_stop, eac_is_stop, eaf_is_stop, eaf_halt, ex_flush;
wire        eaf_is_pea;
wire        eac_is_movem, eac_movem_dir, eac_movem_word, eac_movem_down, eac_movem_wb, eac_movem_pcrel, eac_movem_abs, eac_is_div, eac_div_signed;
wire        eaf_is_div, eaf_div_signed, eaf_is_divzero;
wire        rf3_we;
wire [15:0] eaf_wr_mask;   // the registers EA-fetch's instruction may write (phase 4)
wire  [3:0] agu_reg;       // EA-calculate's base register (phase 4), port D
wire [31:0] agu_rdata;
wire  [3:0] agu_xreg;      // ...and an indexed form's index, port E
wire [31:0] agu_xdata;
wire        eac_agu_ok;
wire [31:0] eac_agu_ea;
wire [31:0] eac_agu_an;
wire        eac_agu_anfw;
wire  [3:0] eac_agu_reg;
wire  [3:0] rf3_addr;
wire [31:0] rf3_data;
wire        eaf_is_rmw, eaf_is_link, eaf_is_mm, eaf_is_xm, eaf_bnt;
wire  [1:0] eaf_mvfsr;
wire  [6:0] eaf_ml;
wire  [2:0] eaf_bf;
wire  [2:0] eaf_ck2;
wire  [4:0] eaf_casf;
wire  [5:0] eaf_rtr_ccr;
wire [31:0] eaf_ea_target;
wire        eac_is_bsr, eac_is_jsr, eac_is_trap, eac_is_illegal;
wire        eac_is_movesr, eac_is_movec;
wire        eac_is_rts, eac_is_rte, eac_is_nop, eac_is_reset, eac_is_rtr, eac_bnt;
wire  [5:0] id_cinv, eac_cinv;
wire  [4:0] id_pmmu, eac_pmmu;
wire  [5:0] id_fflt, eac_fflt;
wire        pm_mmusr_we;
wire [31:0] pm_mmusr_val;
wire        dec_holding, smc_hit;
wire [31:0] dec_hold_pc;
wire        eaf_refetch, ex_pf_inval;
wire  [3:0] eac_cond;

wire        eaf_valid; wire [31:0] eaf_pc; wire [31:0] eaf_next_pc;
wire  [3:0] eaf_dest_reg;
wire [31:0] eaf_operand_a, eaf_operand_b;
wire  [5:0] eaf_alu_op;
wire  [1:0] eaf_size;
wire  [5:0] eaf_shcnt;
wire        eaf_writes_reg, eaf_writes_ccr;
wire        eaf_is_branch, eaf_is_scc, eaf_is_dbcc, eaf_is_jmp;
wire        eaf_is_bsr, eaf_is_jsr, eaf_is_trap, eaf_is_illegal;
wire        eaf_is_priv, eaf_is_movesr, eaf_is_movec;
wire        eaf_is_addrerr;
wire        eaf_movec_dir;
wire  [3:0] eaf_movec_sel;
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
wire  [3:0] exe_creg_sel;
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
// Reset exception processing (2026-09-24, RESET_VECTORS): the fetch is
// held while two supervisor-data longword reads through port B -- nothing
// else can want it, no instruction has been fetched -- load the ISP from
// $0 and the PC from $4; the PC then redirects the fetch like a branch.
localparam [1:0] RV_SSP = 2'd0, RV_PC = 2'd1, RV_GO = 2'd2, RV_DONE = 2'd3;
reg  [1:0] rv_ph;
reg        rv_pend;
reg [31:0] rv_pc;
wire       rv_active = (rv_ph == RV_SSP) || (rv_ph == RV_PC);
wire       rv_issue  = rv_active && !rv_pend;
wire       rv_back   = rv_active && rv_pend && l1_rvalid_b;
wire       rv_go     = (rv_ph == RV_GO);
always @(posedge clk)
	if (!nreset) begin
		rv_ph <= (RESET_VECTORS != 0) ? RV_SSP : RV_DONE; rv_pend <= 1'b0; rv_pc <= 32'd0;
	end else if (ce) begin
		if (rv_issue) rv_pend <= 1'b1;
		else if (rv_back) begin
			rv_pend <= 1'b0;
			if (rv_ph == RV_PC) begin rv_pc <= l1_q_b; rv_ph <= RV_GO; end
			else rv_ph <= RV_PC;
		end else if (rv_go) rv_ph <= RV_DONE;
	end

wire        final_redirect_valid = ex_mispredict || id_redirect_valid || rv_go;
wire [31:0] final_redirect_pc    = rv_go ? rv_pc : ex_mispredict ? ex_recovery_pc : id_redirect_pc;

// Broadcast flush: discards whatever ID/EA-calc/EA-fetch are currently
// holding, all speculatively advanced down the (wrong) assumed-taken guess.
wire flush = ex_flush;

// EA-fetch's regfile operand read ports.
wire  [3:0] raddr_a, raddr_b, raddr_c;
wire [31:0] rdata_a, rdata_b, rdata_c;

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
// Commit ONCE per instruction (milestone 69). EX's output registers hold
// while EX is stalled -- correctly, since milestone 52 gated them on
// ex_stall -- but WB has no view of that stall and would otherwise commit
// the held instruction on every cycle of it. tb_ap040_pipe_integration2.v's
// trace showed one MOVE retiring five times behind a divide.
//
// It was harmless: every commit here writes a value REGISTERED in EX, and
// writing the same registered value again changes nothing. It is gated
// anyway, because that idempotence is a property of what happens to be
// committed today, not of the commit path, and the first non-idempotent
// commit added later would silently multiply. exe_fresh is high exactly in
// the cycle after EX wrote its outputs.
reg exe_fresh;
// It must HOLD across a disabled cycle rather than clear in one (milestone
// 92). EX's output registers are gated on ce, so a cycle with ce low leaves
// the pending result exactly where it was and it still owes its commit;
// computing this as `ce && !ex_stall` dropped that commit instead, and with
// ce alternating every cycle a four-write program committed nothing at all.
always @(posedge clk) begin
	if (!nreset)  exe_fresh <= 1'b0;
	else if (ce)  exe_fresh <= !ex_stall;
end
wire commit_reg  = exe_valid && exe_fresh && exe_writes_reg;
// The second commit: an (An)+/-(An) address update riding alongside the
// ordinary result, gated by its own exe_writes_reg2.
wire commit_reg2 = exe_valid && exe_fresh && exe_writes_reg2;
wire commit_ccr  = exe_valid && exe_fresh && exe_writes_ccr;
wire commit_sr   = exe_valid && exe_fresh && exe_writes_sr;

// STOP (milestone 108). The instruction loads SR and then the processor
// stops until an interrupt arrives. This core has no interrupt input, so
// "until an interrupt" is "until reset" -- which is the architecturally
// correct behaviour for a machine with nothing pending, not a shortcut, and
// is stated here rather than left for a reader to discover.
//
// The stop is taken on the STOP's own commit, so a STOP that faulted has
// already become an exception entry in ap040_ea_fetch.v and never sets it:
// a user-mode STOP raises a privilege violation and the machine keeps
// running the handler, which is what every judged corpus round tests.
//
// Stalling the fetch is not enough on its own: by the time a STOP reaches
// EX the four instructions behind it are already in ID, EA-calc and
// EA-fetch, and they would commit. So the STOP flushes them exactly as a
// mispredicted branch does -- through ap040_execute.v's ex_flush, which
// folds it into the expression that already computes the mispredict flush
// rather than adding a level here -- and the stall then keeps the fetch
// quiet. It is
// taken while the STOP is still IN ex, so the STOP itself completes and only
// younger work is discarded.
wire stop_now = eaf_valid && eaf_is_stop;
reg stopped;
// An interrupt above the mask STOP loaded wakes it (2026-09-24): the
// instruction after the STOP then arrives in EA-fetch with the interrupt
// armed, and its address is what the entry stacks, as on the 68040.
wire irq_pend_c;
// A double fault's halt is not woken by anything: only reset ends it.
reg halted;
always @(posedge clk) begin
	if (!nreset)            stopped <= 1'b0;
	else if (ce && stop_now) stopped <= 1'b1;
	else if (ce && stopped && irq_pend_c && !halted) stopped <= 1'b0;
end
always @(posedge clk) begin
	if (!nreset)                         halted <= 1'b0;
	else if (ce && stop_now && eaf_halt) halted <= 1'b1;
end

// The write mask's check (restructuring plan, phase 4): EA-fetch's wr_mask
// is what an earlier stage may compare its operands against, so it has to
// cover every register the instruction really writes -- the main and second
// commit ports in WB, EX's early An step, EA-fetch's MOVEM port, and the
// stack-pointer banks. Shadows of the mask ride with the instruction as
// EX's and WB's registers do: taken on every advance, as those registers
// are, since not every way out of EA-fetch is an eaf_departs (an entry
// leaving behind a held RTE is not), and a bubble's mask is never used.
// MOVEM's port writes only while the MOVEM is in EA-fetch.
// EX's copy is a register the address stage uses; WB's is for the check.
reg [15:0] ex_wr_mask;
always @(posedge clk)
	if (!nreset)                 ex_wr_mask <= 16'd0;
	else if (ce && !ex_stall)    ex_wr_mask <= eaf_wr_mask;
`ifdef VERILATOR
wire [15:0] wm_ex = ex_wr_mask;
reg  [15:0] wm_wb = 16'd0;
always @(posedge clk)
	if (!nreset) begin
		wm_wb <= 16'd0;
	end else if (ce) begin
		if (!ex_stall) wm_wb <= wm_ex;
		if (commit_reg && !wm_wb[exe_dest_reg])
			$error("wr_mask: %h wrote register %0d in WB, outside its mask %h", exe_pc, exe_dest_reg, wm_wb);
		if (commit_reg2 && !wm_wb[exe_dest_reg2])
			$error("wr_mask: %h wrote register %0d on port 2 in WB, outside its mask %h", exe_pc, exe_dest_reg2, wm_wb);
		if (ex_an_early_we && !wm_ex[ex_an_early_reg])
			$error("wr_mask: EX's early An step wrote register %0d, outside its mask %h", ex_an_early_reg, wm_ex);
		if (rf3_we && !eaf_wr_mask[rf3_addr])
			$error("wr_mask: EA-fetch's MOVEM port wrote register %0d, outside the mask %h", rf3_addr, eaf_wr_mask);
		if (aux_we && !rv_isp_we && !wm_wb[15])
			$error("wr_mask: %h wrote a stack pointer, outside its mask %h", exe_pc, wm_wb);
	end
`endif

// Debug-only: how many register commits have happened. Idempotence hides a
// multiple commit from every value check, so a bench that cares counts.
reg [31:0] dbg_commit_count;
always @(posedge clk) begin
	if (!nreset)          dbg_commit_count <= 32'd0;
	else if (ce && commit_reg) dbg_commit_count <= dbg_commit_count + 32'd1;
end
assign dbg_commits = dbg_commit_count;
wire commit_creg = exe_valid && exe_writes_creg;
// A MOVEC to a translation register holds the memory side's fetches from
// EA-fetch (eaf_mmu_quiet) until it has committed: in EX, and in the cycle
// it writes. The first fetch after it -- its refetch -- is translated by
// the new value.
function mmu_creg;
	input [3:0] sel;
	begin
		mmu_creg = (sel == `AP040_CREG_TC)   || (sel == `AP040_CREG_URP)  || (sel == `AP040_CREG_SRP) ||
		           (sel == `AP040_CREG_ITT0) || (sel == `AP040_CREG_ITT1) ||
		           (sel == `AP040_CREG_DTT0) || (sel == `AP040_CREG_DTT1);
	end
endfunction
wire eaf_mmu_quiet;
assign l1_quiet = eaf_mmu_quiet ||
                  (eaf_valid && eaf_is_movec && eaf_movec_dir && mmu_creg(eaf_movec_sel)) ||
                  (commit_creg && mmu_creg(exe_creg_sel));

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
// Youngest first. EX's whole-SR write beats everything; then EX's CCR result
// (milestone 74 -- the flags an ALU op in EX will register this cycle, which
// the frame push and TRAPcc in EA-fetch could not previously see); then WB's
// commits; then the register.
wire        ex_ccr_fwd_valid;
wire        ex_br_resolve, ex_br_taken;
wire  [4:0] ex_ccr_fwd_data;
wire [15:0] sr_base     = commit_sr  ? exe_sr_data :
                          commit_ccr ? {sr[15:5], exe_result_flags} : sr;
// Two views, on purpose. EX must NOT see its own in-flight flags: with the
// CCR forward folded into the view EX reads, alu_flags fed ccr_in fed the
// ALU fed alu_flags -- a combinational loop that broke ADDX, the one op
// whose result depends on an input flag. So EX keeps the pre-forward view,
// and only EA-fetch, one stage upstream, gets the forwarded one.
wire [15:0] sr_resolved    = ex_sr_fwd_valid  ? ex_sr_fwd_data : sr_base;
`ifdef VERILATOR
// The register file banks A7 by sr_base, not EX's forward: EX may change S
// or M only in a cycle it is also emptying everything behind it.
always @(posedge clk)
	if (nreset && ce && ex_sr_fwd_valid && (ex_sr_fwd_data[13:12] != sr_base[13:12]) && !ex_flush)
		$error("ap040_pipe_cpu: EX changes S/M (%h -> %h) without emptying what is behind it", sr_base, ex_sr_fwd_data);
`endif
wire [15:0] sr_resolved_ea = ex_sr_fwd_valid  ? ex_sr_fwd_data :
                             ex_ccr_fwd_valid ? {sr_base[15:5], ex_ccr_fwd_data} : sr_base;

// The interrupt request (ap040_pipe_irq.v): judged against the SR the
// instruction in EA-fetch sees, held against the committed one.
wire       irq_pend, irq_ack, irq_ack_nmi;
wire [2:0] irq_take_lvl;
ap040_pipe_irq u_irq
(
	.clk        (clk),
	.nreset     (nreset),
	.irq_lvl_in (irq_lvl),
	.mask       (sr[10:8]),
	.mask_live  (sr_resolved_ea[10:8]),
	.ack        (irq_ack && ce),
	.ack_nmi    (irq_ack_nmi && ce),
	.sr_commit  (commit_sr && ce),
	.pend       (irq_pend),
	.take_lvl   (irq_take_lvl),
	.pend_c     (irq_pend_c)
);

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
// CACR: DE (bit 31) and IE (bit 15), masked as ap040_core.v's S_MOVEC2
// (`& 32'h8000_8000`) so a read-back is bit-exact. IE enables the
// instruction cache in the IMU (caches stage B); DE is still inert -- the
// data cache comes with stage C.
//---------------------------------------------------------------------------

reg [31:0] vbr;
reg  [2:0] sfc, dfc;
reg [31:0] cacr;
// The MMU registers (milestone 113): what MOVEC wrote, through
// ap040_core.v's write masks (ap040_core.v:3850-3870), and nothing else --
// there is no MMU here for them to configure.
reg [31:0] tc, itt0, itt1, dtt0, dtt1, mmusr, urp, srp;

always @(posedge clk) begin
	if (!nreset) begin
		vbr  <= 32'h0;
		sfc  <= 3'h0;
		dfc  <= 3'h0;
		cacr <= 32'h0;
		tc   <= 32'h0;
		itt0 <= 32'h0;
		itt1 <= 32'h0;
		dtt0 <= 32'h0;
		dtt1 <= 32'h0;
		mmusr <= 32'h0;
		urp  <= 32'h0;
		srp  <= 32'h0;
	end else if (ce && commit_creg) begin
		case (exe_creg_sel)
			`AP040_CREG_SFC:  sfc  <= exe_creg_data[2:0];
			`AP040_CREG_DFC:  dfc  <= exe_creg_data[2:0];
			`AP040_CREG_CACR: cacr <= exe_creg_data & 32'h8000_8000;
			`AP040_CREG_VBR:  vbr  <= exe_creg_data;
			`AP040_CREG_TC:    tc    <= exe_creg_data & 32'h0000_C000;
			`AP040_CREG_ITT0:  itt0  <= exe_creg_data & 32'hFFFF_E364;
			`AP040_CREG_ITT1:  itt1  <= exe_creg_data & 32'hFFFF_E364;
			`AP040_CREG_DTT0:  dtt0  <= exe_creg_data & 32'hFFFF_E364;
			`AP040_CREG_DTT1:  dtt1  <= exe_creg_data & 32'hFFFF_E364;
			`AP040_CREG_MMUSR: mmusr <= exe_creg_data;
			`AP040_CREG_URP:   urp   <= exe_creg_data & 32'hFFFF_FE00;
			`AP040_CREG_SRP:   srp   <= exe_creg_data & 32'hFFFF_FE00;
			default: ;   // USP/ISP/MSP route through the regfile's aux port instead
		endcase
	end
	// PTEST's result (2026-09-24). It comes from EA-fetch, younger than any
	// MOVEC committing in the same cycle, so it is applied after.
	if (nreset && ce && pm_mmusr_we) mmusr <= pm_mmusr_val;
end
assign ic_en = cacr[15];
assign dc_en = cacr[31];
assign cacr_out = cacr;
assign vbr_out  = vbr;
assign nmi_ack  = irq_ack_nmi && ce;
assign halted_out = halted;

assign mmu_tc = tc;     assign mmu_urp = urp;   assign mmu_srp = srp;
assign mmu_itt0 = itt0; assign mmu_itt1 = itt1; assign mmu_dtt0 = dtt0; assign mmu_dtt1 = dtt1;
// A data TTR can refuse a write (its W bit) with TC.E clear, so either
// makes a write tentative.
assign l1_wr_sync = tc[15] || dtt0[15] || dtt1[15];

wire [31:0] usp_q, isp_q, msp_q;   // ap040_pipe_regfile.v's own state, read
                                    // back here for MOVEC's read direction

// MOVEC's write direction reaching USP/ISP/MSP bypasses ap040_pipe_regfile.v's
// normal commit_reg path (which would bank through whichever of the three
// sr_s/sr_m currently selects) and drives its aux port directly instead --
// exactly the mechanism that port has existed for since this fork was first
// written, unused until now. aux_sel's 0=USP/1=ISP/2=MSP numbering is one
// bit-shift away from AP040_CREG_USP/ISP/MSP's own 4/5/6 -- see
// ap040_pipe_defs.svh's comment on why that ordering was chosen.
// ...and the reset sequence's initial ISP, before any instruction exists.
wire        rv_isp_we = ce && rv_back && (rv_ph == RV_SSP);
wire        aux_we    = rv_isp_we ||
                        (commit_creg && (exe_creg_sel == `AP040_CREG_USP ||
                                          exe_creg_sel == `AP040_CREG_ISP ||
                                          exe_creg_sel == `AP040_CREG_MSP));
wire [1:0]  aux_sel    = rv_isp_we ? 2'd1 : exe_creg_sel[1:0];   // USP=4'b100->00, ISP=101->01, MSP=110->10
wire [31:0] aux_wdata  = rv_isp_we ? l1_q_b : exe_creg_data;

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
	// milestone 14, since nothing touched them yet. sr_base: the register
	// with WB's commit written through, so an instruction reading A7 in the
	// cycle MOVE to SR commits banks it the new way. Not EX's forward
	// (sr_resolved) any more (2026-09-24): every instruction that changes S
	// or M empties everything behind it as it passes EX -- MOVE to SR and
	// the immediates to SR refetch (sr_wr_refetch), RTE, STOP and an
	// exception redirect -- so whatever read A7 in that cycle is discarded,
	// and the forward was EX's SR logic in front of every register read,
	// the start of the bus16 top's worst path. The check below holds it.
	// sr[13]/sr[12] reset to AP040_SR_RESET's own S=1/M=0, so A7 still
	// banks to ISP at reset, unchanged behavior for every earlier test.
	.sr_s     (sr_base[13]),
	.sr_m     (sr_base[12]),
	// ...and the architectural view for the write side -- see the port's
	// own comment in ap040_pipe_regfile.v (milestone 92).
	.sr_s_w   (sr_base[13]),
	.sp_sel2_w(ex_an_early_we ? ex_an_early_sel : exe_an_sel),
	.sr_m_w   (sr_base[12]),

	.we       (commit_reg),
	.waddr    (exe_dest_reg),
	.wdata    (exe_result_data),

	.raddr_a  (raddr_a),
	.rdata_a  (rdata_a),
	.raddr_c  (raddr_c),
	.rdata_c  (rdata_c),
	.raddr_b  (raddr_b),
	.rdata_b  (rdata_b),
	.raddr_d  (agu_reg),
	.rdata_d  (agu_rdata),
	.raddr_e  (agu_xreg),
	.rdata_e  (agu_xdata),

	.we3      (rf3_we),
	.waddr3   (rf3_addr),
	.wdata3   (rf3_data),
	// Port 2 also takes a MULL/DIVL's early An step (milestone 117), in a
	// cycle EX's own stall keeps commit_reg2 low.
	.we2      (commit_reg2 || ex_an_early_we),
	.waddr2   (ex_an_early_we ? ex_an_early_reg  : exe_dest_reg2),
	.wdata2   (ex_an_early_we ? ex_an_early_data : exe_result_data2),

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
wire      [31:0] eaf_l1_addr_b;

// The arbitration itself: EX's read-modify-write store beat takes the data
// port from EA-fetch for its cycle (ex_st_req, which EA-fetch sees as
// port_taken).
wire eaf_l1_rd_b;
wire eaf_l1_sup_b;
// Whoever owns port B this cycle owns its privilege too (milestone 93).
assign l1_sup_b  = rv_active ? 1'b1 : ex_st_req ? ex_st_sup : eaf_l1_sup_b;
wire        eaf_l1_fc_ovr;
wire  [2:0] eaf_l1_fc_val;
assign l1_fc_ovr = !rv_active && !ex_st_req && eaf_l1_fc_ovr;
assign l1_fc_val = eaf_l1_fc_val;
// For the data cache: the reset vectors allocate nothing; EX's store beat
// is locked for TAS and CAS.
wire        eaf_l1_nalloc, eaf_l1_m16, eaf_l1_lock;
assign l1_nalloc_b = rv_active || (!ex_st_req && eaf_l1_nalloc);
assign l1_m16_b    = !rv_active && !ex_st_req && eaf_l1_m16;
assign l1_lock_b   = !rv_active && (ex_st_req ? ((eaf_alu_op == `AP040_ALU_TAS) || eaf_casf[4]) : eaf_l1_lock);
// The fetch's privilege is the mode the fetched instruction will RUN in,
// which for the first instruction of a handler is supervisor -- and the
// exception's own SR write has not committed when that fetch goes out.
// sr_resolved carries EX's pending write; the committed register does not.
assign l1_sup_a  = sr_resolved[13];
// EA-fetch's requests wait for EX's pipelined load as they do for EX's
// store (port_taken); every request of its own already asks, and these
// gates are the port's own guarantee that nothing younger overtakes it.
wire        eaf_rd_out = eaf_l1_rd_b   && !ex_ld_busy;
wire        eaf_wr_out = eaf_l1_wren_b && !ex_ld_busy;
assign l1_rd_b   = ce && (rv_active ? rv_issue : eaf_rd_out);
assign l1_inval_a = ce && ex_pf_inval;

// The store snoop (2026-09-24). A store can land on an instruction already
// fetched behind it -- in EA-calc, in decode's gather, or the word the
// fetch is presenting -- where ap040_pipe_membus.v's stream snoop can no
// longer reach it; the storing instruction then owes a refetch of what
// follows. Ranges are half-open byte ranges; a gather in progress owns
// everything from its first word up to the fetch word.
// Since phase 8 the fetch keeps a queue, so its range is every word fetched
// and not yet taken -- if_pc up to if_end, the fetch in flight included --
// and it is judged only when that range is not empty.
//
// A store from EA-fetch is judged a cycle LATE, on its registered address:
// judged live, the compare hung off the port-B address, and was 817ba673's
// worst path (-0.249 ns). Nothing younger can pass the store in that cycle,
// so the ranges then are a superset of the ranges when it went out. If the
// storing instruction left EA-fetch as it stored, it is in EX now and EX
// raises the refetch (st_smc_late); if not -- a MOVEM mid-list -- EA-fetch
// keeps it for the departure (smc_hit).
reg  [31:0] sq_a;
reg   [1:0] sq_sz;
reg         sq_v, sq_dep;
wire        eaf_departs;
wire        sq_acc  = ce && !rv_active && !ex_st_req && eaf_wr_out && !l1_wr_busy;
wire [31:0] snp_e   = sq_a + ((sq_sz == `AP040_SZ_L) ? 32'd4 : (sq_sz == `AP040_SZ_W) ? 32'd2 : 32'd1);
wire [31:0] snp_dlo = dec_holding ? dec_hold_pc : if_pc;
wire        if_owns = dec_holding || (if_end != if_pc);
wire        snp_dec = if_owns && (sq_a < if_end) && (snp_e > snp_dlo);
wire        snp_id  = id_valid  && (sq_a < id_next_pc)  && (snp_e > id_pc);
wire        snp_eac = eac_valid && (sq_a < eac_next_pc) && (snp_e > eac_pc);
wire        snp_hit = sq_v && (snp_dec || snp_id || (sq_dep && snp_eac));
assign smc_hit      = snp_hit && !sq_dep;
wire   st_smc_late  = snp_hit && sq_dep;
always @(posedge clk)
	if (!nreset) begin sq_v <= 1'b0; sq_dep <= 1'b0; sq_a <= 32'd0; sq_sz <= 2'd0; end
	else if (ce) begin
		if (sq_acc) begin
			sq_v <= 1'b1; sq_a <= eaf_l1_addr_b; sq_sz <= eaf_l1_size_b; sq_dep <= eaf_departs;
		end else if (!sq_dep || !ex_stall) sq_v <= 1'b0;   // judged: held while EX holds it
	end
// EX's read-modify-write store (and MOVE #imm to memory, which goes the
// same way): one instruction more is younger than it, the one in EA-fetch.
wire [31:0] snx_a   = ex_st_addr;
wire [31:0] snx_e   = ex_st_addr + ((ex_st_size == `AP040_SZ_L) ? 32'd4 :
                                    (ex_st_size == `AP040_SZ_W) ? 32'd2 : 32'd1);
wire        snx_dec = if_owns && (snx_a < if_end) && (snx_e > snp_dlo);
wire        snx_id  = id_valid  && (snx_a < id_next_pc)  && (snx_e > id_pc);
wire        snx_eac = eac_valid && (snx_a < eac_next_pc) && (snx_e > eac_pc);
wire        st_smc  = ex_st_req && !l1_wr_busy && (snx_dec || snx_id || snx_eac);
assign l1_addr_b = rv_active ? {29'd0, (rv_ph == RV_PC), 2'b00} : ex_st_req ? ex_st_addr : eaf_l1_addr_b;
// Gated by ce, all of them (milestone 92). ap040_pipe_l1.v has no clock
// enable, and neither does real memory: a request left asserted through a
// disabled cycle is a request the memory sees again. The write buffer
// accepted one store 134 times that way. en_a needs no gate here --
// ap040_inst_fetch.v already builds it from ce.
assign l1_wren_b = ce && !rv_active && (ex_st_req ? 1'b1  : eaf_wr_out);
assign l1_wr_drop = ce && !(!rv_active && (ex_st_req || eaf_wr_out));
assign l1_size_b   = rv_active ? `AP040_SZ_L : ex_st_req ? ex_st_size : eaf_l1_size_b;
assign l1_data_b = ex_st_req ? ex_st_data : eaf_l1_data_b;


ap040_inst_fetch #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) u_if
(
	.clk       (clk),
	.nreset    (nreset),
	.ce        (ce),
	.stall_in  (id_stall || stopped || rv_active),
	.take2     (id_take2),
	.hold      (stopped || rv_active),

	.flush          (flush),
	.redirect_valid (final_redirect_valid),
	.redirect_pc    (final_redirect_pc),

	.l1_addr_a  (l1_addr_a),
	.l1_req_a   (l1_req_a),
	.l1_rdata_a (l1_rdata_a),
	.l1_rdata_a2(l1_rdata_a2),
	.l1_rvalid_a(l1_rvalid_a),
	.l1_rflt_a  (l1_rflt_a),
	.l1_rflt_a_bus(l1_rflt_a_bus),

	.if_valid  (if_valid),
	.if_pc     (if_pc),
	.if_opcode (if_opcode),
	.if_flt    (if_flt),
	.if_flt_bus(if_flt_bus),
	.if_valid2 (if_valid2),
	.if_word2  (if_word2),
	.if_flt2   (if_flt2),
	.if_end    (if_end)
);

// A stopped machine must present decode with NOTHING, not merely stop
// fetching new words (milestone 110). if_valid is if_pend && l1_rvalid_a in
// ap040_inst_fetch.v and does not consult stall_in at all, which only
// blocks `advance` -- so holding the stall preserved a VALID word and decode
// consumed it again and again. STOP's own flush even fetches one more word
// on the way out, and that is the word that circulated: a MOVE.L D0,(A0)
// tail posted 796 writes after the STOP, and an ADDQ tail counted D0 up
// past 1590 retirements.
//
// Qualifying it here rather than inside the fetch stage keeps the fetch's
// own PC and pending-read bookkeeping untouched, so an outstanding read
// still completes and STOP's SR commit is unaffected.
wire if_valid_id = if_valid && !stopped;

ap040_decode u_id
(
	.clk             (clk),
	.nreset          (nreset),
	.ce              (ce),
	.stall_in        (ea_stall),
	.flush           (flush),

	.if_valid        (if_valid_id),
	.if_pc           (if_pc),
	.if_opcode       (if_opcode),
	// The fetch fault travels with its word, through the fetch's queue.
	.if_flt          (if_flt),
	.if_flt_bus      (if_flt_bus),
	.if_valid2       (if_valid2 && !stopped),
	.if_word2        (if_word2),
	.if_flt2         (if_flt2),
	.take2           (id_take2),

	.id_stall        (id_stall),

	.id_redirect_valid (id_redirect_valid),
	.id_redirect_pc    (id_redirect_pc),

	.id_valid        (id_valid),
	.dec_holding     (dec_holding),
	.dec_hold_pc     (dec_hold_pc),
	.id_pc           (id_pc),
	.id_next_pc      (id_next_pc),
	.id_dest_reg     (id_dest_reg),
	.id_src_reg      (id_src_reg),
	.id_imm          (id_imm),
	.id_ea_ext          (id_ea_ext),
	.id_mm              (id_mm),
	.id_moves           (id_moves),
	.id_mvfsr           (id_mvfsr),
	.id_pc_off          (id_pc_off),
	.id_movep           (id_movep),
	.id_ml              (id_ml),
	.id_bf              (id_bf),
	.id_ck2             (id_ck2),
	.id_cas             (id_cas),
	.id_m16             (id_m16),
	.id_fp              (id_fp),
	.id_fp_op           (id_fp_op),
	.id_fp_cmd          (id_fp_cmd),
	.id_fp_imm          (id_fp_imm),
	.id_fx              (id_fx),
	.id_fx_bd           (id_fx_bd),
	.id_fx_od           (id_fx_od),
	.id_alu_op       (id_alu_op),
	.id_size         (id_size),
	.id_shcnt        (id_shcnt),
	.id_shift_reg    (id_shift_reg),
	.id_src_a_is_imm (id_src_a_is_imm),
	.id_writes_reg   (id_writes_reg),
	.id_writes_ccr   (id_writes_ccr),
	.id_is_branch    (id_is_branch),
	.id_is_scc       (id_is_scc),
	.id_is_dbcc      (id_is_dbcc),
	.id_is_mem_src   (id_is_mem_src),
	.id_is_abs       (id_is_abs),
	.id_is_store     (id_is_store),
	.id_is_postinc   (id_is_postinc),
	.id_is_predec    (id_is_predec),
	.id_is_jmp       (id_is_jmp),
	.id_is_lea       (id_is_lea),
	.id_sxt_w        (id_sxt_w),
	.id_ea_indexed   (id_ea_indexed),
	.id_ea_pcrel     (id_ea_pcrel),
	.id_is_rmw       (id_is_rmw),
	.id_immrmw       (id_immrmw),
	.id_st_only      (id_st_only),
	.id_st_disp      (id_st_disp),
	.id_chk_long     (id_chk_long),
	.id_is_chk       (id_is_chk),
	.id_is_trapcc    (id_is_trapcc),
	.id_is_immsr     (id_is_immsr),
	.id_is_stop      (id_is_stop),
	.id_immsr_to_sr  (id_immsr_to_sr),
	.id_is_pea       (id_is_pea),
	.id_is_link      (id_is_link),
	.id_is_div       (id_is_div),
	.id_div_signed   (id_div_signed),
	.id_is_movem     (id_is_movem),
	.id_movem_dir    (id_movem_dir),
	.id_movem_word   (id_movem_word),
	.id_movem_down   (id_movem_down),
	.id_movem_wb     (id_movem_wb),
	.id_movem_pcrel  (id_movem_pcrel),
	.id_movem_abs    (id_movem_abs),
	.id_movem_mask   (id_movem_mask),
	.id_is_unlk      (id_is_unlk),
	.id_is_bsr       (id_is_bsr),
	.id_is_jsr       (id_is_jsr),
	.id_is_trap      (id_is_trap),
	.id_is_illegal   (id_is_illegal),
	.id_illegal_kind (id_illegal_kind),
	.id_is_movesr    (id_is_movesr),
	.id_is_movec     (id_is_movec),
	.id_is_rts       (id_is_rts),
	.id_is_nop       (id_is_nop),
	.id_cinv         (id_cinv),
	.id_pmmu         (id_pmmu),
	.id_fflt         (id_fflt),
	.id_bnt          (id_bnt),
	.id_is_rtr       (id_is_rtr),
	.id_is_reset     (id_is_reset),
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
	.id_ea_ext           (id_ea_ext),
	.id_mm               (id_mm),
	.id_moves            (id_moves),
	.id_mvfsr            (id_mvfsr),
	.id_pc_off           (id_pc_off),
	.id_movep            (id_movep),
	.id_ml               (id_ml),
	.id_bf               (id_bf),
	.id_ck2              (id_ck2),
	.id_cas              (id_cas),
	.id_m16              (id_m16),
	.id_fp               (id_fp),
	.id_fp_op            (id_fp_op),
	.id_fp_cmd           (id_fp_cmd),
	.id_fp_imm           (id_fp_imm),
	.id_fx               (id_fx),
	.id_fx_bd            (id_fx_bd),
	.id_fx_od            (id_fx_od),
	.id_alu_op        (id_alu_op),
	.id_size          (id_size),
	.id_shcnt         (id_shcnt),
	.id_shift_reg     (id_shift_reg),
	.id_src_a_is_imm  (id_src_a_is_imm),
	.id_writes_reg    (id_writes_reg),
	.id_writes_ccr    (id_writes_ccr),
	.id_is_branch     (id_is_branch),
	.id_is_scc        (id_is_scc),
	.id_is_dbcc       (id_is_dbcc),
	.id_is_mem_src    (id_is_mem_src),
	.id_is_abs        (id_is_abs),
	.id_is_store      (id_is_store),
	.id_is_postinc    (id_is_postinc),
	.id_is_predec     (id_is_predec),
	.id_is_jmp        (id_is_jmp),
	.id_is_lea        (id_is_lea),
	.id_sxt_w         (id_sxt_w),
	.id_ea_indexed    (id_ea_indexed),
	.id_ea_pcrel      (id_ea_pcrel),
	.id_is_rmw        (id_is_rmw),
	.id_immrmw        (id_immrmw),
	.id_st_only       (id_st_only),
	.agu_reg          (agu_reg),
	.agu_rdata        (agu_rdata),
	.agu_xreg         (agu_xreg),
	.agu_xdata        (agu_xdata),
	.ahead1_wr_mask   (eaf_wr_mask),
	.ahead2_wr_mask   (ex_wr_mask),
	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),
	.ex_res_slow      (ex_res_slow),
	.ex_an_valid      (eaf_valid && eaf_writes_an && !ex_fwd2_slow),
	.ex_an_reg        (eaf_an_reg),
	.ex_an_data       (eaf_an_data),
	.eac_agu_ok       (eac_agu_ok),
	.eac_agu_ea       (eac_agu_ea),
	.eac_agu_an       (eac_agu_an),
	.eac_agu_anfw     (eac_agu_anfw),
	.eac_agu_reg      (eac_agu_reg),
	.id_st_disp       (id_st_disp),
	.id_chk_long      (id_chk_long),
	.id_is_chk        (id_is_chk),
	.id_is_trapcc     (id_is_trapcc),
	.id_is_immsr      (id_is_immsr),
	.id_is_stop       (id_is_stop),
	.id_immsr_to_sr   (id_immsr_to_sr),
	.id_is_pea        (id_is_pea),
	.id_is_link       (id_is_link),
	.id_is_div        (id_is_div),
	.id_div_signed    (id_div_signed),
	.id_is_movem      (id_is_movem),
	.id_movem_dir     (id_movem_dir),
	.id_movem_word    (id_movem_word),
	.id_movem_down    (id_movem_down),
	.id_movem_wb      (id_movem_wb),
	.id_movem_pcrel   (id_movem_pcrel),
	.id_movem_abs     (id_movem_abs),
	.id_movem_mask    (id_movem_mask),
	.id_is_unlk       (id_is_unlk),
	.id_is_bsr        (id_is_bsr),
	.id_is_jsr        (id_is_jsr),
	.id_is_trap       (id_is_trap),
	.id_is_illegal    (id_is_illegal),
	.id_illegal_kind  (id_illegal_kind),
	.id_is_movesr     (id_is_movesr),
	.id_is_movec      (id_is_movec),
	.id_is_rts        (id_is_rts),
	.id_is_nop        (id_is_nop),
	.id_cinv          (id_cinv),
	.id_pmmu          (id_pmmu),
	.id_fflt          (id_fflt),
	.id_bnt           (id_bnt),
	.id_is_rtr        (id_is_rtr),
	.id_is_reset      (id_is_reset),
	.id_is_rte        (id_is_rte),
	.id_cond          (id_cond),

	.ea_stall         (ea_stall),

	.eac_valid        (eac_valid),
	.eac_pc           (eac_pc),
	.eac_next_pc      (eac_next_pc),
	.eac_dest_reg     (eac_dest_reg),
	.eac_src_reg      (eac_src_reg),
	.eac_imm          (eac_imm),
	.eac_ea_ext          (eac_ea_ext),
	.eac_mm              (eac_mm),
	.eac_moves           (eac_moves),
	.eac_mvfsr           (eac_mvfsr),
	.eac_pc_base         (eac_pc_base),
	.eac_movep           (eac_movep),
	.eac_ml              (eac_ml),
	.eac_bf              (eac_bf),
	.eac_ck2             (eac_ck2),
	.eac_cas             (eac_cas),
	.eac_m16             (eac_m16),
	.eac_fp              (eac_fp),
	.eac_fp_op           (eac_fp_op),
	.eac_fp_cmd          (eac_fp_cmd),
	.eac_fp_imm          (eac_fp_imm),
	.eac_fx              (eac_fx),
	.eac_fx_bd           (eac_fx_bd),
	.eac_fx_od           (eac_fx_od),
	.eac_alu_op       (eac_alu_op),
	.eac_size         (eac_size),
	.eac_shcnt        (eac_shcnt),
	.eac_shift_reg    (eac_shift_reg),
	.eac_src_a_is_imm (eac_src_a_is_imm),
	.eac_writes_reg   (eac_writes_reg),
	.eac_writes_ccr   (eac_writes_ccr),
	.eac_is_branch    (eac_is_branch),
	.eac_is_scc       (eac_is_scc),
	.eac_is_dbcc      (eac_is_dbcc),
	.eac_is_mem_src   (eac_is_mem_src),
	.eac_is_abs       (eac_is_abs),
	.eac_is_store     (eac_is_store),
	.eac_is_postinc   (eac_is_postinc),
	.eac_is_predec    (eac_is_predec),
	.eac_is_jmp       (eac_is_jmp),
	.eac_is_lea       (eac_is_lea),
	.eac_sxt_w        (eac_sxt_w),
	.eac_is_rmw       (eac_is_rmw),
	.eac_immrmw       (eac_immrmw),
	.eac_st_only      (eac_st_only),
	.eac_st_disp      (eac_st_disp),
	.eac_ea_indexed   (eac_ea_indexed),
	.eac_ea_pcrel     (eac_ea_pcrel),
	.eac_chk_long     (eac_chk_long),
	.eac_is_chk       (eac_is_chk),
	.eac_is_trapcc    (eac_is_trapcc),
	.eac_is_immsr     (eac_is_immsr),
	.eac_is_stop      (eac_is_stop),
	.eac_immsr_to_sr  (eac_immsr_to_sr),
	.eac_is_pea       (eac_is_pea),
	.eac_is_link      (eac_is_link),
	.eac_is_unlk      (eac_is_unlk),
	.eac_is_movem     (eac_is_movem),
	.eac_movem_dir    (eac_movem_dir),
	.eac_movem_word   (eac_movem_word),
	.eac_movem_down   (eac_movem_down),
	.eac_movem_wb     (eac_movem_wb),
	.eac_movem_pcrel  (eac_movem_pcrel),
	.eac_movem_abs    (eac_movem_abs),
	.eac_movem_mask   (eac_movem_mask),
	.eac_is_div       (eac_is_div),
	.eac_div_signed   (eac_div_signed),
	.eac_is_bsr       (eac_is_bsr),
	.eac_is_jsr       (eac_is_jsr),
	.eac_is_trap      (eac_is_trap),
	.eac_is_illegal   (eac_is_illegal),
	.eac_illegal_kind (eac_illegal_kind),
	.eac_is_movesr    (eac_is_movesr),
	.eac_is_movec     (eac_is_movec),
	.eac_is_rts       (eac_is_rts),
	.eac_is_nop       (eac_is_nop),
	.eac_cinv         (eac_cinv),
	.eac_pmmu         (eac_pmmu),
	.eac_fflt         (eac_fflt),
	.eac_bnt          (eac_bnt),
	.eac_is_rtr       (eac_is_rtr),
	.eac_is_reset     (eac_is_reset),
	.eac_is_rte       (eac_is_rte),
	.eac_cond         (eac_cond)
);

ap040_ea_fetch #(
	.DUMMY    (0)
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
	.eac_ea_ext          (eac_ea_ext),
	.eac_mm              (eac_mm),
	.eac_moves           (eac_moves),
	.eac_mvfsr           (eac_mvfsr),
	.eac_pc_base         (eac_pc_base),
	.eac_movep           (eac_movep),
	.eac_ml              (eac_ml),
	.eac_bf              (eac_bf),
	.eac_ck2             (eac_ck2),
	.eac_cas             (eac_cas),
	.eac_m16             (eac_m16),
	.eac_fp              (eac_fp),
	.eac_fp_op           (eac_fp_op),
	.eac_fp_cmd          (eac_fp_cmd),
	.eac_fp_imm          (eac_fp_imm),
	.eac_fx              (eac_fx),
	.eac_fx_bd           (eac_fx_bd),
	.eac_fx_od           (eac_fx_od),
	.eac_alu_op       (eac_alu_op),
	.eac_size         (eac_size),
	.eac_shcnt        (eac_shcnt),
	.eac_shift_reg    (eac_shift_reg),
	.eac_src_a_is_imm (eac_src_a_is_imm),
	.eac_writes_reg   (eac_writes_reg),
	.eac_writes_ccr   (eac_writes_ccr),
	.eac_is_branch    (eac_is_branch),
	.eac_is_scc       (eac_is_scc),
	.eac_is_dbcc      (eac_is_dbcc),
	.eac_is_mem_src   (eac_is_mem_src),
	.eac_is_abs       (eac_is_abs),
	.eac_is_store     (eac_is_store),
	.eac_is_postinc   (eac_is_postinc),
	.eac_is_predec    (eac_is_predec),
	.eac_is_jmp       (eac_is_jmp),
	.eac_is_lea       (eac_is_lea),
	.eac_sxt_w        (eac_sxt_w),
	.eac_is_rmw       (eac_is_rmw),
	.eac_immrmw       (eac_immrmw),
	.eac_st_only      (eac_st_only),
	.eac_st_disp      (eac_st_disp),
	.eac_ea_indexed   (eac_ea_indexed),
	.eac_ea_pcrel     (eac_ea_pcrel),
	.eac_chk_long     (eac_chk_long),
	.eac_is_chk       (eac_is_chk),
	.eac_is_trapcc    (eac_is_trapcc),
	.eac_is_immsr     (eac_is_immsr),
	.eac_is_stop      (eac_is_stop),
	.eac_immsr_to_sr  (eac_immsr_to_sr),
	.eac_is_pea       (eac_is_pea),
	.eac_is_link      (eac_is_link),
	.eac_is_unlk      (eac_is_unlk),
	.eac_is_movem     (eac_is_movem),
	.eac_movem_dir    (eac_movem_dir),
	.eac_movem_word   (eac_movem_word),
	.eac_movem_down   (eac_movem_down),
	.eac_movem_wb     (eac_movem_wb),
	.eac_movem_pcrel  (eac_movem_pcrel),
	.eac_movem_abs    (eac_movem_abs),
	.eac_movem_mask   (eac_movem_mask),
	.eac_is_div       (eac_is_div),
	.eac_div_signed   (eac_div_signed),
	.eaf_is_div       (eaf_is_div),
	.eaf_ld_pend      (eaf_ld_pend),
	.eaf_ld_sxw       (eaf_ld_sxw),
	.eaf_div_signed   (eaf_div_signed),
	.rf3_we           (rf3_we),
	.wr_mask          (eaf_wr_mask),
	.eac_agu_ok       (eac_agu_ok),
	.eac_agu_ea       (eac_agu_ea),
	.eac_agu_an       (eac_agu_an),
	.rf3_addr         (rf3_addr),
	.rf3_data         (rf3_data),
	.eaf_is_link      (eaf_is_link),
	.eaf_is_pea       (eaf_is_pea),
	.eaf_is_chk       (eaf_is_chk),
	.eaf_chk_ok       (eaf_chk_ok),
	.eaf_is_trapcc    (eaf_is_trapcc),
	.eaf_is_fmterr    (eaf_is_fmterr),
	.eaf_is_trace     (eaf_is_trace),
	.eaf_is_immsr     (eaf_is_immsr),
	.eaf_is_stop      (eaf_is_stop),
	.eaf_halt         (eaf_halt),
	.eaf_refetch      (eaf_refetch),
	.eaf_departs      (eaf_departs),
	.eaf_immsr_to_sr  (eaf_immsr_to_sr),
	.port_taken       (ex_st_req || ex_ld_busy),
	.wb_busy          (exe_valid),
	.ex_br_resolve    (ex_br_resolve),
	.ex_br_taken      (ex_br_taken),
	.eaf_is_rmw       (eaf_is_rmw),
	.eaf_is_mm        (eaf_is_mm),
	.eaf_is_xm        (eaf_is_xm),
	.eaf_bnt          (eaf_bnt),
	.eaf_mvfsr        (eaf_mvfsr),
	.eaf_ml           (eaf_ml),
	.eaf_bf           (eaf_bf),
	.eaf_ck2          (eaf_ck2),
	.eaf_casf         (eaf_casf),
	.eaf_rtr_ccr      (eaf_rtr_ccr),
	.eaf_ea_target    (eaf_ea_target),
	.eac_is_bsr       (eac_is_bsr),
	.eac_is_jsr       (eac_is_jsr),
	.eac_is_trap      (eac_is_trap),
	.eac_is_illegal   (eac_is_illegal),
	.eac_illegal_kind (eac_illegal_kind),
	.eac_is_movesr    (eac_is_movesr),
	.eac_is_movec     (eac_is_movec),
	.eac_is_rts       (eac_is_rts),
	.eac_is_nop       (eac_is_nop),
	.eac_cinv         (eac_cinv),
	.eac_pmmu         (eac_pmmu),
	.eac_fflt         (eac_fflt),
	.mem_idle         (l1_idle),
	.mmu_quiet        (eaf_mmu_quiet),
	.pt_req (pt_req), .pt_write (pt_write), .pt_addr (pt_addr), .pt_fc (pt_fc),
	.pt_done (pt_done), .pt_mmusr (pt_mmusr), .mmusr_we (pm_mmusr_we), .mmusr_val (pm_mmusr_val),
	.reset_out        (reset_out),
	.pf_req (pf_req), .pf_mode (pf_mode), .pf_addr (pf_addr), .pf_fc (pf_fc), .pf_done (pf_done),
	.cm_req (cm_req), .cm_ic (cm_ic), .cm_dc (cm_dc), .cm_push (cm_push), .cm_scope (cm_scope),
	.cm_addr (cm_addr), .cm_done (cm_done),
	.pw_pend (pw_pend), .pw_ssw (pw_ssw), .pw_fa (pw_fa), .pw_wb1s (pw_wb1s), .pw_pd (pw_pd),
	.pw_exc (pw_exc), .pw_ack (pw_ack),
	.l1_nalloc_b (eaf_l1_nalloc), .l1_m16_b (eaf_l1_m16), .l1_lock_b (eaf_l1_lock),
	.smc_hit          (smc_hit),
	.l1_rflt_b        (l1_rflt_b),
	.l1_wflt          (l1_wflt),
	.l1_flt_bus       (l1_flt_bus),
	.l1_flt_ma        (l1_flt_ma),
	.ex_aerr          (ex_aerr),
	.ex_aerr_rd       (ex_aerr_rd),
	.ex_st_addr       (ex_st_addr),
	.ex_st_data       (ex_st_data),
	.ex_st_size       (ex_st_size),
	.ex_st_sup        (ex_st_sup),
	.eac_bnt          (eac_bnt),
	.eac_is_rtr       (eac_is_rtr),
	.eac_is_reset     (eac_is_reset),
	.eac_is_rte       (eac_is_rte),
	.eac_cond         (eac_cond),

	.sr_in            (sr_resolved_ea),
	.ccr_nofwd        (sr_resolved[4:0]),
	.ccr_fwd_busy     (ex_ccr_fwd_valid),
	.irq_pend         (irq_pend),
	.irq_take_lvl     (irq_take_lvl),
	.irq_ack          (irq_ack),
	.irq_ack_nmi      (irq_ack_nmi),
	.isp_in           (isp_q),
	.msp_in           (msp_q),
	.usp_in           (usp_q),

	.raddr_a          (raddr_a),
	.rdata_a          (rdata_a),
	.raddr_c          (raddr_c),
	.rdata_c          (rdata_c),
	.raddr_b          (raddr_b),
	.rdata_b          (rdata_b),

	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),
	.ex_fwd2_valid    (ex_fwd2_valid),
	.ex_fwd2_dest     (ex_fwd2_dest),
	.ex_fwd2_data     (ex_fwd2_data),
	.ex_fwd2_slow     (ex_fwd2_slow),
	.ex_res_slow      (ex_res_slow),

	.l1_addr_b        (eaf_l1_addr_b),
	.l1_q_b           (l1_q_b),
	.l1_rvalid_b      (l1_rvalid_b),
	.l1_rd_b          (eaf_l1_rd_b),
	.l1_sup_b         (eaf_l1_sup_b),
	.l1_fc_ovr        (eaf_l1_fc_ovr),
	.l1_fc_val        (eaf_l1_fc_val),
	.sfc_in3          (sfc),
	.dfc_in3          (dfc),
	.l1_wren_b        (eaf_l1_wren_b),
	.l1_size_b          (eaf_l1_size_b),
	.l1_data_b        (eaf_l1_data_b),
	.l1_wr_busy       (l1_wr_busy),

	.eaf_stall        (eaf_stall),

	.eaf_valid        (eaf_valid),
	.eaf_pc           (eaf_pc),
	.eaf_next_pc      (eaf_next_pc),
	.eaf_dest_reg     (eaf_dest_reg),
	.eaf_operand_a    (eaf_operand_a),
	.eaf_operand_b    (eaf_operand_b),
	.eaf_alu_op       (eaf_alu_op),
	.eaf_size         (eaf_size),
	.eaf_shcnt        (eaf_shcnt),
	.eaf_writes_an    (eaf_writes_an),
	.eaf_an_sel       (eaf_an_sel),
	.ex_creg_sp       (ex_creg_sp),
	.ex_creg_any      (ex_creg_any),
	.ex_wr_a7         (eaf_valid && ex_wr_mask[15]),
	.a7_busy          (a7_busy),
	.vbr_in           (vbr),
	.creg_busy        (ex_creg_any || commit_creg),
	.eaf_an_reg       (eaf_an_reg),
	.eaf_an_data      (eaf_an_data),
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
	.eaf_is_divzero   (eaf_is_divzero),
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
	.eaf_size         (eaf_size),
	.eaf_shcnt        (eaf_shcnt),
	.eaf_writes_an    (eaf_writes_an),
	.eaf_an_sel       (eaf_an_sel),
	.eaf_an_reg       (eaf_an_reg),
	.eaf_an_data      (eaf_an_data),
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
	.eaf_is_divzero   (eaf_is_divzero),
	.eaf_is_movesr    (eaf_is_movesr),
	.eaf_is_movec     (eaf_is_movec),
	.eaf_movec_dir    (eaf_movec_dir),
	.eaf_movec_sel    (eaf_movec_sel),
	.eaf_sr_snapshot  (eaf_sr_snapshot),
	.eaf_is_rts       (eaf_is_rts),
	.eaf_is_rte       (eaf_is_rte),
	.eaf_rte_sr_data  (eaf_rte_sr_data),
	.eaf_cond         (eaf_cond),

	// sr_base, not sr_resolved: an instruction writing the SR computes its
	// result from its snapshot, never from ccr_in, so EX's own SR forward
	// cannot matter here -- but it put that forward on the path into the
	// ALU's X input, and so ahead of every forwarded result (the worst
	// path at 63f63af4 began eaf_alu_op -> ex_sr_fwd_data -> ALU).
	.ccr_in           (sr_base[4:0]),

	.sfc_in           ({29'd0, sfc}),
	.dfc_in           ({29'd0, dfc}),
	.cacr_in          (cacr),
	.vbr_in           (vbr),
	.usp_in           (usp_q),
	.isp_in           (isp_q),
	.msp_in           (msp_q),
	.tc_in            (tc),
	.itt0_in          (itt0),
	.itt1_in          (itt1),
	.dtt0_in          (dtt0),
	.dtt1_in          (dtt1),
	.mmusr_in         (mmusr),
	.urp_in           (urp),
	.srp_in           (srp),

	.eaf_is_rmw       (eaf_is_rmw),
	.eaf_is_mm        (eaf_is_mm),
	.eaf_is_xm        (eaf_is_xm),
	.eaf_bnt          (eaf_bnt),
	.eaf_mvfsr        (eaf_mvfsr),
	.eaf_ml           (eaf_ml),
	.eaf_bf           (eaf_bf),
	.eaf_ck2          (eaf_ck2),
	.eaf_casf         (eaf_casf),
	.eaf_rtr_ccr      (eaf_rtr_ccr),
	.eaf_is_link      (eaf_is_link),
	.eaf_is_pea       (eaf_is_pea),
	.eaf_is_chk       (eaf_is_chk),
	.eaf_chk_ok       (eaf_chk_ok),
	.eaf_is_trapcc    (eaf_is_trapcc),
	.eaf_is_fmterr    (eaf_is_fmterr),
	.eaf_is_trace     (eaf_is_trace),
	.eaf_is_immsr     (eaf_is_immsr),
	.eaf_is_stop      (eaf_is_stop),
	.eaf_refetch      (eaf_refetch),
	.st_smc           (st_smc),
	.st_smc_late      (st_smc_late),
	.eaf_immsr_to_sr  (eaf_immsr_to_sr),
	.eaf_is_div       (eaf_is_div),
	.eaf_ld_pend      (eaf_ld_pend),
	.eaf_ld_sxw       (eaf_ld_sxw),
	.eaf_div_signed   (eaf_div_signed),
	.eaf_ea_target    (eaf_ea_target),
	.l1_wr_busy       (l1_wr_busy),
	.l1_wflt          (l1_wflt),
	.ex_aerr          (ex_aerr),
	.ex_aerr_rd       (ex_aerr_rd),
	.ex_ccr_fwd_valid (ex_ccr_fwd_valid),
	.ex_br_resolve    (ex_br_resolve),
	.ex_br_taken      (ex_br_taken),
	.ex_ccr_fwd_data  (ex_ccr_fwd_data),
	.ex_st_req        (ex_st_req),
	.l1_q_b           (l1_q_b),
	.l1_rvalid_b      (l1_rvalid_b),
	.l1_rflt_b        (l1_rflt_b),
	.ex_ld_busy       (ex_ld_busy),
	.ex_st_addr       (ex_st_addr),
	.ex_st_data       (ex_st_data),
	.ex_st_size       (ex_st_size),
	.ex_stall         (ex_stall),

	.ex_fwd_valid     (ex_fwd_valid),
	.ex_fwd2_valid    (ex_fwd2_valid),
	.ex_fwd2_dest     (ex_fwd2_dest),
	.ex_fwd2_data     (ex_fwd2_data),
	.ex_fwd2_slow     (ex_fwd2_slow),
	.ex_res_slow      (ex_res_slow),
	.ex_pf_inval      (ex_pf_inval),
	.ex_fwd_dest      (ex_fwd_dest),
	.ex_fwd_data      (ex_fwd_data),

	.ex_sr_fwd_valid  (ex_sr_fwd_valid),
	.ex_sr_fwd_data   (ex_sr_fwd_data),

	.ex_mispredict    (ex_mispredict),
	.ex_flush         (ex_flush),
	.ex_recovery_pc   (ex_recovery_pc),

	.exe_valid        (exe_valid),
	.exe_pc           (exe_pc),
	.exe_dest_reg     (exe_dest_reg),
	.exe_result_data  (exe_result_data),
	.exe_writes_reg   (exe_writes_reg),
	.exe_dest_reg2    (exe_dest_reg2),
	.exe_result_data2 (exe_result_data2),
	.exe_writes_reg2  (exe_writes_reg2),
	.exe_an_sel       (exe_an_sel),
	.exe_writes_ccr   (exe_writes_ccr),
	.exe_result_flags (exe_result_flags),

	.exe_writes_sr    (exe_writes_sr),
	.exe_sr_data      (exe_sr_data),
	.ex_creg_sp       (ex_creg_sp),
	.ex_creg_any      (ex_creg_any),
	.ex_st_sup        (ex_st_sup),
	.ex_an_early_we   (ex_an_early_we),
	.ex_an_early_reg  (ex_an_early_reg),
	.ex_an_early_data (ex_an_early_data),
	.ex_an_early_sel  (ex_an_early_sel),
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
