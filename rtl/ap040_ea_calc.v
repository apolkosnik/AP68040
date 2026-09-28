//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 17: address error)       //
//                                                                          //
// ap040_ea_calc.v - EA-calc stage                                         //
//                                                                          //
// MOVEQ, register-direct MOVE.L/ADD.L, Bcc/BRA (any displacement width),   //
// Scc, DBcc, JMP, BSR, and JSR all have no memory effective address to     //
// compute, so this is still a pure pass-through -- it exists to occupy the //
// register slot a real address calculation will use once an EA mode that   //
// actually needs arithmetic is added, rather than being spliced in later   //
// and reshaping the pipeline. id_is_branch/id_is_scc/id_cond/              //
// id_writes_ccr/id_next_pc ride through unchanged for the same reason --   //
// this stage has nothing to compute for any of them; only                  //
// ap040_execute.v actually evaluates a condition or merges a byte, per its //
// header comment. id_next_pc is just one more field in the same            //
// pass-through shape.                                                     //
//                                                                          //
// MOVE.L (An),Dn (milestone 9b) and JMP (An)/(d16,An) (milestone 11) are    //
// ALSO pure pass-throughs here, deliberately: their effective addresses     //
// are An's raw value (plus id_imm's displacement for the (d16,An) forms),   //
// no arithmetic -- id_is_mem_src/id_is_jmp thread through unchanged, same   //
// shape as every other flag above. This is why the EA-calc/EA-fetch split   //
// exists as two stages already (per the original plan): the day an EA mode  //
// needs real arithmetic (indexed modes, etc.), it lands HERE, and EA-       //
// fetch's memory-access/stall logic (see its header) doesn't need to        //
// change at all -- it already just consumes whatever address EA-calc        //
// resolved.                                                                 //
//                                                                          //
// BSR/JSR (milestone 13, new) are the SAME story one level further: the     //
// push address (A7-4) and the memory write itself are both computed and     //
// issued entirely in ap040_ea_fetch.v, from id_dest_reg's register value    //
// (port B) -- id_is_bsr/id_is_jsr just thread through here unchanged, same  //
// as every other flag.                                                     //
//                                                                          //
// TRAP #n / illegal instruction (milestone 14, new): same story a third     //
// time. Both need A7's value (port B, id_dest_reg already pointed at A7 by   //
// ap040_decode.v, same trick BSR/JSR use) and nothing else EA-calc could      //
// compute -- the frame contents, the multi-beat push, and the vector-table    //
// read all live in ap040_ea_fetch.v's new exception-entry sequencer. id_is_    //
// trap/id_is_illegal just thread through unchanged, same shape as every       //
// other flag above.                                                          //
//                                                                          //
// MOVE to SR / MOVEC (milestone 15, new): a fourth and fifth flavor of the    //
// same story, PLUS a genuinely dynamic wrinkle this stage still doesn't       //
// need to know about -- whether either one actually FAULTS (privilege         //
// violation) depends on the live, forwarded S bit, which doesn't exist         //
// until ap040_ea_fetch.v/ap040_execute.v. id_is_movesr/id_is_movec thread       //
// through unchanged either way; the fault decision and its consequences         //
// (suppressing the normal SR/control-register write, rerouting port B to         //
// A7 for the exception's own push) are entirely ap040_ea_fetch.v's job -- see     //
// its header.                                                                     //
//                                                                          //
// RTS / RTE (milestone 16, new): a sixth and seventh flavor, same story        //
// again. RTS reuses id_is_mem_src (ap040_decode.v now sets it for RTS too,       //
// same FSM MOVE.L (An),Dn already uses) so it needs NO new pass-through field       //
// at all beyond id_is_rts itself, purely for ap040_execute.v's redirect/commit       //
// classification. RTE gets its own new sequencer entirely in ap040_ea_fetch.v         //
// (two reads, not one, plus a dynamic privilege check) -- this stage still has          //
// nothing to compute for it either.                                                      //
//                                                                          //
// flush: when ap040_execute.v detects a mispredicted branch, everything     //
// speculatively fetched behind it -- including whatever is sitting here -- //
// must be discarded. Same shape as stall_in but forces a bubble instead     //
// of holding.                                                              //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_ea_calc
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // EA-fetch cannot accept this cycle
	input             flush,      // EX detected a misprediction: force a bubble

	input             id_valid,
	input      [31:0] id_pc,
	input      [31:0] id_next_pc,
	input       [3:0] id_dest_reg,
	input       [3:0] id_src_reg,
	input      [31:0] id_imm,
	input      [31:0] id_ea_ext,
	input       [6:0] id_mm,
	input       [2:0] id_moves,
	input       [1:0] id_mvfsr,
	input       [1:0] id_pc_off,
	input       [2:0] id_movep,
	input       [6:0] id_ml,
	input       [4:0] id_bf,
	input       [2:0] id_ck2,
	input       [4:0] id_cas,
	input       [3:0] id_m16,
	input             id_fp,        // the F-line (2026-09-24), see ap040_pipe_fpu.v
	input       [8:0] id_fp_op,
	input      [15:0] id_fp_cmd,
	input      [95:0] id_fp_imm,
	input       [5:0] id_fx,        // a full-format extension (2026-09-24)
	input      [31:0] id_fx_bd,
	input      [31:0] id_fx_od,
	input       [5:0] id_alu_op,
	input       [1:0] id_size,
	input       [5:0] id_shcnt,
	input             id_shift_reg,
	input             id_src_a_is_imm,
	input             id_writes_reg,
	input             id_writes_ccr,
	input             id_is_branch,
	input             id_is_scc,
	input             id_is_dbcc,
	input             id_is_mem_src,
	input             id_is_abs,
	input             id_is_store,
	input             id_is_postinc,
	input             id_is_predec,
	input             id_is_jmp,
	input             id_is_lea,
	input             id_sxt_w,
	input             id_ea_indexed,
	input             id_ea_pcrel,
	input             id_is_rmw,
	input             id_immrmw,
	input             id_st_only,
	// The address stage (restructuring plan, phase 4): the base's value from
	// the register file's port D (agu_reg names it), EX's An step, and what
	// the two instructions ahead may still write.
	output      [3:0] agu_reg,
	input      [31:0] agu_rdata,
	output      [3:0] agu_xreg,         // an indexed form's index register, port E
	input      [31:0] agu_xdata,
	input      [15:0] ahead1_wr_mask,   // the instruction in EA-fetch
	input      [15:0] ahead2_wr_mask,   // ...and in EX
	input             ex_fwd_valid,
	input       [3:0] ex_fwd_dest,
	input      [31:0] ex_fwd_data,      // EX's result: final in any cycle this stage takes an instruction
	input             ex_res_slow,      // ...unless it is a staged MUL.L's, formed only into WB
	input             ex_an_valid,      // EX's An step, a register: eaf_an_data
	input       [3:0] ex_an_reg,
	input      [31:0] ex_an_data,
	input             id_st_disp,
	input             id_is_div,
	input             id_div_signed,
	input             id_is_movem,
	input             id_movem_dir,
	input             id_movem_word,
	input             id_movem_down,
	input             id_movem_wb,
	input             id_movem_pcrel,
	input             id_movem_abs,
	input      [15:0] id_movem_mask,
	input             id_is_trapcc,
	input             id_is_chk,
	input             id_chk_long,
	input             id_is_immsr,
	input             id_is_stop,
	input             id_immsr_to_sr,
	input             id_is_pea,
	input             id_is_link,
	input             id_is_unlk,
	input             id_is_bsr,
	input             id_is_jsr,
	input             id_is_trap,
	input             id_is_illegal,
	input       [1:0] id_illegal_kind,
	input             id_is_movesr,
	input             id_is_movec,
	input             id_is_rts,
	input             id_is_nop,
	input       [5:0] id_cinv,
	input       [4:0] id_pmmu,
	input       [5:0] id_fflt,
	input             id_bnt,
	input             id_is_rtr,
	input             id_is_reset,
	input             id_is_rte,
	input       [3:0] id_cond,

	output            ea_stall,   // to ID: EA-fetch is full, or an address waits for its base

	output reg        eac_valid,
	output reg [31:0] eac_pc,
	output reg [31:0] eac_next_pc,
	output reg  [3:0] eac_dest_reg,
	output reg  [3:0] eac_src_reg,
	output reg [31:0] eac_imm,
	output reg [31:0] eac_ea_ext,
	output reg  [6:0] eac_mm,
	output reg  [2:0] eac_moves,
	output reg  [1:0] eac_mvfsr,
	// The PC a PC-relative EA is relative to: the address of its displacement
	// word, which an immediate ahead of it pushes on by two or four bytes
	// (milestone 115). Registered here so EA-fetch's address path sees a
	// register where it used to see eac_pc + 2.
	output reg [31:0] eac_pc_base,
	output reg  [2:0] eac_movep,
	output reg  [6:0] eac_ml,
	output reg  [4:0] eac_bf,
	output reg  [2:0] eac_ck2,
	output reg  [4:0] eac_cas,
	output reg  [3:0] eac_m16,
	output reg        eac_fp,
	output reg  [8:0] eac_fp_op,
	output reg [15:0] eac_fp_cmd,
	output reg [95:0] eac_fp_imm,
	output reg  [5:0] eac_fx,
	output reg [31:0] eac_fx_bd,
	output reg [31:0] eac_fx_od,
	output reg  [5:0] eac_alu_op,
	output reg  [1:0] eac_size,
	output reg  [5:0] eac_shcnt,
	output reg        eac_shift_reg,
	output reg        eac_src_a_is_imm,
	output reg        eac_writes_reg,
	output reg        eac_writes_ccr,
	output reg        eac_is_branch,
	output reg        eac_is_scc,
	output reg        eac_is_dbcc,
	output reg        eac_is_mem_src,
	output reg        eac_is_abs,
	output reg        eac_is_store,
	output reg        eac_is_postinc,
	output reg        eac_is_predec,
	output reg        eac_is_jmp,
	output reg        eac_is_lea,
	output reg        eac_sxt_w,
	output reg        eac_ea_indexed,
	output reg        eac_ea_pcrel,
	output reg        eac_is_rmw,
	output reg        eac_immrmw,
	output reg        eac_st_only,   // written, not read: no load, and an EX store all the same
	output reg        eac_agu_ok,    // eac_agu_ea is this instruction's address
	output reg [31:0] eac_agu_ea,
	output reg [31:0] eac_agu_an,    // ...and with (An)+/-(An), eac_agu_an is An's new value
	output reg        eac_agu_anfw,  // ...An's only write, so the next instruction may take it
	output reg  [3:0] eac_agu_reg,
	output reg        eac_st_disp,
	output reg        eac_is_div,
	output reg        eac_div_signed,
	output reg        eac_is_movem,
	output reg        eac_movem_dir,
	output reg        eac_movem_word,
	output reg        eac_movem_down,
	output reg        eac_movem_wb,
	output reg        eac_movem_pcrel,
	output reg        eac_movem_abs,
	output reg [15:0] eac_movem_mask,
	output reg        eac_is_trapcc,
	output reg        eac_is_chk,
	output reg        eac_chk_long,
	output reg        eac_is_immsr,
	output reg        eac_is_stop,
	output reg        eac_immsr_to_sr,
	output reg        eac_is_pea,
	output reg        eac_is_link,
	output reg        eac_is_unlk,
	output reg        eac_is_bsr,
	output reg        eac_is_jsr,
	output reg        eac_is_trap,
	output reg        eac_is_illegal,
	output reg  [1:0] eac_illegal_kind,
	output reg        eac_is_movesr,
	output reg        eac_is_movec,
	output reg        eac_is_rts,
	output reg        eac_is_nop,
	output reg  [5:0] eac_cinv,
	output reg  [4:0] eac_pmmu,
	output reg  [5:0] eac_fflt,
	output reg        eac_bnt,
	output reg        eac_is_rtr,
	output reg        eac_is_reset,
	output reg        eac_is_rte,
	output reg  [3:0] eac_cond
);


// ---- the address stage (restructuring plan, phase 4) ----
// For the simple forms -- (An), (An)+, -(An), (d16,An), (d16,PC),
// absolute, and the brief-format indexed (d8,An,Xn) and (d8,PC,Xn) -- of a
// load, a read-modify-write (CLR/Scc's write-only store included) and a
// store, the address is formed here, a stage early, and registered for
// EA-fetch, which uses it: no forward reaches the L1 address this way. An
// index resolves from the same places a base does, through port E.
// The base is the youngest of: the instruction ahead's own An step (formed
// here a cycle ago, a register), EX's result, EX's An step (a register),
// the register file (WB's commit bypassed there). EX's result is taken only
// into eac_agu_ea -- a register -- and only when this stage takes the
// instruction, which is only when EX advances, so it is final. If the
// instruction ahead may write the base any other way, or EX may by a path
// that is not forwarded (a MULL/DIVL high word, a stack-pointer bank, an
// RTE's pops), the instruction waits here, and a bubble goes on. A MOVEM's
// loads need nothing of their own: one is only ever outstanding for the
// MOVEM in EA-fetch (asserted there), whose write set is all sixteen.
wire        agu_special = id_mm[5] || id_mm[6] || id_moves[2] || id_movep[2] || id_ml[6] || id_bf[4] ||
                          id_ck2[2] || id_cas[3] || id_cas[4] || id_m16[3] || id_fp || id_fx[5] ||
                          id_is_movem || id_is_rts || id_is_rte || id_is_rtr || id_cinv[5] || id_pmmu[4] ||
                          id_fflt[5] || id_mvfsr[1];   // mvfsr[0] is CCR-or-SR: opcode bit 9, whatever the instruction
// A plain store's base is its destination register; a displacement
// store's, like a load's, is the source field's (decode points it at An,
// and the data comes through port B).
wire        agu_st     = id_is_store && !id_st_disp;
wire        agu_ld     = (id_is_mem_src || id_st_only || id_st_disp) && !agu_st;
wire        agu_class  = (agu_st || agu_ld) && !agu_special;
assign      agu_reg    = agu_st ? id_dest_reg : id_src_reg;
// (d16,PC): the extension word's own address, which no register holds.
wire [31:0] agu_pcb    = id_pc + 32'd2 + {29'd0, id_pc_off, 1'b0};
wire        agu_nobase = id_is_abs || id_ea_pcrel;
wire  [1:0] agu_size   = id_sxt_w ? `AP040_SZ_W : id_size;
wire [31:0] agu_step   = (agu_size == `AP040_SZ_L) ? 32'd4 :
                         (agu_size == `AP040_SZ_W) ? 32'd2 :
                         (agu_reg == 4'd15)        ? 32'd2 : 32'd1;
wire        agu_a1_hit = eac_valid && eac_agu_anfw && (eac_agu_reg == agu_reg);
wire        agu_ex_hit = ex_fwd_valid && (ex_fwd_dest == agu_reg);
wire        agu_an_hit = ex_an_valid && (ex_an_reg == agu_reg);
wire [31:0] agu_base   = id_ea_pcrel ? agu_pcb :
                         agu_a1_hit ? eac_agu_an :
                         agu_ex_hit ? ex_fwd_data :   // EX's main write wins over its An step
                         agu_an_hit ? ex_an_data : agu_rdata;
wire [31:0] agu_ext    = id_immrmw ? id_ea_ext : id_imm;
// The instruction ahead's An step already holds every older write to the
// register, so nothing further back is looked at when it is taken.
wire        agu_hz     = (ahead1_wr_mask[agu_reg] && !agu_a1_hit) ||
                         (ahead2_wr_mask[agu_reg] && !agu_a1_hit && !agu_ex_hit && !agu_an_hit) ||
                         (agu_ex_hit && !agu_a1_hit && ex_res_slow);
// The brief extension word, verbatim in agu_ext: [15] D/A and [14:12] the
// index register, [11] its size (a Word index is sign-extended), [10:9] the
// scale, [7:0] a signed byte displacement. The full format is EA-fetch's
// (id_fx, special above).
wire        agu_idx    = id_ea_indexed;
assign      agu_xreg   = {agu_ext[15], agu_ext[14:12]};
wire        agu_x_a1   = eac_valid && eac_agu_anfw && (eac_agu_reg == agu_xreg);
wire        agu_x_ex   = ex_fwd_valid && (ex_fwd_dest == agu_xreg);
wire        agu_x_an   = ex_an_valid && (ex_an_reg == agu_xreg);
wire [31:0] agu_xval   = agu_x_a1 ? eac_agu_an :
                         agu_x_ex ? ex_fwd_data :
                         agu_x_an ? ex_an_data : agu_xdata;
wire [31:0] agu_xsz    = agu_ext[11] ? agu_xval : {{16{agu_xval[15]}}, agu_xval[15:0]};
wire [31:0] agu_xsc    = agu_xsz << agu_ext[10:9];
wire [31:0] agu_d8     = {{24{agu_ext[7]}}, agu_ext[7:0]};
wire        agu_x_hz   = agu_idx && ((ahead1_wr_mask[agu_xreg] && !agu_x_a1) ||
                                     (ahead2_wr_mask[agu_xreg] && !agu_x_a1 && !agu_x_ex && !agu_x_an) ||
                                     (agu_x_ex && !agu_x_a1 && ex_res_slow));
wire        agu_wait   = id_valid && agu_class && ((!agu_nobase && agu_hz) || agu_x_hz);
wire        agu_ok     = agu_class && (agu_nobase || !agu_hz) && !agu_x_hz;
wire [31:0] agu_dec    = agu_base - agu_step;
wire [31:0] agu_ea     = id_is_abs    ? (agu_st ? id_imm : agu_ext) :
                         id_is_predec ? agu_dec :
                         agu_st       ? agu_base :
                         agu_idx      ? (agu_base + agu_xsc + agu_d8) : (agu_base + agu_ext);
// An's own write is its only one unless the instruction's result goes there
// too (MOVE.L (A0)+,A0): then the result, not the step, is what follows.
wire        agu_anfw   = agu_ok && (id_is_postinc || id_is_predec) &&
                         !(id_writes_reg && (id_dest_reg == agu_reg));

// ID holds while the instruction waits for its base or its index.
assign ea_stall = stall_in || agu_wait;

// Every simple access reaches EA-fetch with its address formed: EA-fetch's
// own views stay only for the check there, and step 4 takes them out.
`ifdef VERILATOR
reg eac_agu_class = 1'b0;
always @(posedge clk)
	if (!nreset)                         eac_agu_class <= 1'b0;
	else if (ce && !flush && !stall_in)  eac_agu_class <= id_valid && agu_class;
always @(posedge clk)
	if (nreset && ce && eac_valid && eac_agu_class && !eac_agu_ok)
		$error("agu: %h reached EA-fetch without its address", eac_pc);
`endif

always @(posedge clk) begin
	if (!nreset) begin
		eac_valid        <= 1'b0;
		eac_pc           <= 32'h0;
		eac_next_pc      <= 32'h0;
		eac_dest_reg     <= 4'h0;
		eac_src_reg      <= 4'h0;
		eac_imm          <= 32'h0;
		eac_ea_ext       <= 32'h0;
		eac_mm           <= 7'd0;
		eac_moves        <= 3'd0;
		eac_mvfsr        <= 2'd0;
		eac_pc_base      <= 32'h0;
		eac_movep        <= 3'd0;
		eac_ml           <= 7'd0;
		eac_bf           <= 5'd0;
		eac_ck2          <= 3'd0;
		eac_cas          <= 5'd0;
		eac_m16          <= 4'd0;
		eac_fp           <= 1'b0;
		eac_fp_op        <= 9'd0;
		eac_fp_cmd       <= 16'd0;
		eac_fp_imm       <= 96'd0;
		eac_fx           <= 6'd0;
		eac_fx_bd        <= 32'd0;
		eac_fx_od        <= 32'd0;
		eac_alu_op       <= 6'h0;
		eac_size         <= `AP040_SZ_L;
		eac_shcnt        <= 6'd1;
		eac_shift_reg    <= 1'b0;
		eac_src_a_is_imm <= 1'b0;
		eac_writes_reg   <= 1'b0;
		eac_writes_ccr   <= 1'b0;
		eac_is_branch    <= 1'b0;
		eac_is_scc       <= 1'b0;
		eac_is_dbcc      <= 1'b0;
		eac_is_mem_src   <= 1'b0;
		eac_is_abs       <= 1'b0;
		eac_is_store     <= 1'b0;
		eac_is_postinc   <= 1'b0;
		eac_is_predec    <= 1'b0;
		eac_is_jmp       <= 1'b0;
		eac_is_lea       <= 1'b0;
		eac_sxt_w        <= 1'b0;
		eac_ea_indexed   <= 1'b0;
		eac_ea_pcrel     <= 1'b0;
		eac_is_rmw       <= 1'b0;
		eac_immrmw       <= 1'b0;
		eac_st_only      <= 1'b0;
		eac_agu_ok       <= 1'b0;
		eac_agu_ea       <= 32'h0;
		eac_agu_an       <= 32'h0;
		eac_agu_anfw     <= 1'b0;
		eac_agu_reg      <= 4'd0;
		eac_st_disp      <= 1'b0;
		eac_is_div       <= 1'b0;
		eac_div_signed   <= 1'b0;
		eac_is_movem     <= 1'b0;
		eac_movem_dir    <= 1'b0;
		eac_movem_word   <= 1'b0;
		eac_movem_down   <= 1'b0;
		eac_movem_wb     <= 1'b0;
		eac_movem_pcrel  <= 1'b0;
		eac_movem_abs    <= 1'b0;
		eac_movem_mask   <= 16'h0;
		eac_is_trapcc    <= 1'b0;
		eac_is_chk       <= 1'b0;
		eac_chk_long     <= 1'b0;
		eac_is_immsr     <= 1'b0;
		eac_is_stop      <= 1'b0;
		eac_immsr_to_sr  <= 1'b0;
		eac_is_pea       <= 1'b0;
		eac_is_link      <= 1'b0;
		eac_is_unlk      <= 1'b0;
		eac_is_bsr       <= 1'b0;
		eac_is_jsr       <= 1'b0;
		eac_is_trap      <= 1'b0;
		eac_is_illegal   <= 1'b0;
		eac_illegal_kind <= 2'd0;
		eac_is_movesr    <= 1'b0;
		eac_is_movec     <= 1'b0;
		eac_is_rts       <= 1'b0;
		eac_is_nop       <= 1'b0;
		eac_cinv         <= 6'd0;
		eac_pmmu         <= 5'd0;
		eac_fflt         <= 6'd0;
		eac_bnt          <= 1'b0;
		eac_is_rtr       <= 1'b0;
		eac_is_reset     <= 1'b0;
		eac_is_rte       <= 1'b0;
		eac_cond         <= 4'h0;
	end else if (ce) begin
		if (flush) begin
			eac_valid <= 1'b0;
		end else if (!stall_in) begin
			eac_valid        <= id_valid && !agu_wait;
			eac_pc           <= id_pc;
			eac_next_pc      <= id_next_pc;
			eac_dest_reg     <= id_dest_reg;
			eac_src_reg      <= id_src_reg;
			eac_imm          <= id_imm;
			eac_ea_ext       <= id_ea_ext;
			eac_mm           <= id_mm;
			eac_moves        <= id_moves;
			eac_mvfsr        <= id_mvfsr;
			eac_pc_base      <= agu_pcb;
			eac_movep        <= id_movep;
			eac_ml           <= id_ml;
			eac_bf           <= id_bf;
			eac_ck2          <= id_ck2;
			eac_cas          <= id_cas;
			eac_m16          <= id_m16;
			eac_fp           <= id_fp;
			eac_fp_op        <= id_fp_op;
			eac_fp_cmd       <= id_fp_cmd;
			eac_fp_imm       <= id_fp_imm;
			eac_fx           <= id_fx;
			eac_fx_bd        <= id_fx_bd;
			eac_fx_od        <= id_fx_od;
			eac_alu_op       <= id_alu_op;
			eac_size         <= id_size;
			eac_shcnt        <= id_shcnt;
			eac_shift_reg    <= id_shift_reg;
			eac_src_a_is_imm <= id_src_a_is_imm;
			eac_writes_reg   <= id_writes_reg;
			eac_writes_ccr   <= id_writes_ccr;
			eac_is_branch    <= id_is_branch;
			eac_is_scc       <= id_is_scc;
			eac_is_dbcc      <= id_is_dbcc;
			eac_is_mem_src   <= id_is_mem_src && !id_st_only;   // CLR/Scc: nothing to read
			eac_is_abs       <= id_is_abs;
			eac_is_store     <= id_is_store;
			eac_is_postinc   <= id_is_postinc;
			eac_is_predec    <= id_is_predec;
			eac_is_jmp       <= id_is_jmp;
			eac_is_lea       <= id_is_lea;
			eac_sxt_w        <= id_sxt_w;
			eac_ea_indexed   <= id_ea_indexed;
			eac_ea_pcrel     <= id_ea_pcrel;
			eac_is_rmw       <= id_is_rmw;
			eac_immrmw       <= id_immrmw;
			eac_st_only      <= id_st_only;
			eac_agu_ok       <= id_valid && agu_ok;
			eac_agu_ea       <= agu_ea;
			eac_agu_an       <= id_is_postinc ? (agu_base + agu_step) : agu_dec;
			eac_agu_anfw     <= id_valid && agu_anfw;
			eac_agu_reg      <= agu_reg;
			eac_st_disp      <= id_st_disp;
			eac_is_div       <= id_is_div;
			eac_div_signed   <= id_div_signed;
			eac_is_movem     <= id_is_movem;
			eac_movem_dir    <= id_movem_dir;
			eac_movem_word   <= id_movem_word;
			eac_movem_down   <= id_movem_down;
			eac_movem_wb     <= id_movem_wb;
			eac_movem_pcrel  <= id_movem_pcrel;
			eac_movem_abs    <= id_movem_abs;
			eac_movem_mask   <= id_movem_mask;
			eac_is_trapcc    <= id_is_trapcc;
			eac_is_chk       <= id_is_chk;
			eac_chk_long     <= id_chk_long;
			eac_is_immsr     <= id_is_immsr;
			eac_is_stop      <= id_is_stop;
			eac_immsr_to_sr  <= id_immsr_to_sr;
			eac_is_pea       <= id_is_pea;
			eac_is_link      <= id_is_link;
			eac_is_unlk      <= id_is_unlk;
			eac_is_bsr       <= id_is_bsr;
			eac_is_jsr       <= id_is_jsr;
			eac_is_trap      <= id_is_trap;
			eac_is_illegal   <= id_is_illegal;
			eac_illegal_kind <= id_illegal_kind;
			eac_is_movesr    <= id_is_movesr;
			eac_is_movec     <= id_is_movec;
			eac_is_rts       <= id_is_rts;
			eac_is_nop       <= id_is_nop;
			eac_cinv         <= id_cinv;
			eac_pmmu         <= id_pmmu;
			eac_fflt         <= id_fflt;
			eac_bnt          <= id_bnt;
			eac_is_rtr       <= id_is_rtr;
			eac_is_reset     <= id_is_reset;
			eac_is_rte       <= id_is_rte;
			eac_cond         <= id_cond;
		end
	end
end

endmodule
