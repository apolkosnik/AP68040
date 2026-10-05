//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_backend.sv - AG, DC1, DC2, EX and WB stages                       //
//                                                                          //
//   AG   read the front register file, effective address, (An)+/-(An)     //
//        and LEA-class register updates (written to the front file)       //
//   DC1  data ATC / cache lookup (ap68040_dmu)                             //
//   DC2  way select and operand alignment (ap68040_dmu)                    //
//   EX   ALU, shifter, multiply/divide, branch resolution, front-file      //
//        result write                                                      //
//   WB   back (architectural) register file, CCR/SR, stores, exceptions,  //
//        interrupts, trace, serialization                                  //
//                                                                          //
// Operand values are read at AG.  A result produced by an older uop in EX  //
// is broadcast and captured by every younger uop in AG/DC1/DC2 whose       //
// operand names that register, so EX never forwards; address operands     //
// (base, index, register updates) instead interlock at AG on any older    //
// uop that will write the register in EX.                                  //
//                                                                          //
// Every redirect (branch mispredict, exception, serialization) happens in  //
// WB, from state registered at the end of EX.                              //
//--------------------------------------------------------------------------//

module ap68040_backend
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,

	// uops from the sequencer
	input  logic        in_v,
	input  uop_t        in_u,
	output logic        in_rdy,

	// redirect of the front end (registered)
	output logic        redir_v,
	output logic [31:0] redir_pc,
	output logic        flush,          // kill everything younger than WB

	// exception entry for the sequencer
	output logic        exc_go,         // pulse: run the exception routine
	output logic  [3:0] exc_kind,       // routine selector (see EK_*)
	output logic  [4:0] exc_ssp,        // supervisor stack register to use

	// micro-branch result back to the sequencer
	output logic        ucond_v,
	output logic        ucond,

	// architectural state the front end needs
	output logic [15:0] sr,
	output logic [31:0] vbr,
	output logic [31:0] cacr,
	output logic  [2:0] sfc,
	output logic  [2:0] dfc,

	// data memory unit (DC1/DC2 lookups; WB stores)
	output logic        dm_req,         // AG -> DC1: a memory uop advances
	output logic [31:0] dm_va,
	output logic  [1:0] dm_mem,
	output logic  [1:0] dm_msz,
	output logic  [2:0] dm_fc,
	output logic        dm_lock,
	output logic        dm_locke,
	output logic        dm_super,
	output logic        dm_noalloc,     // exception stacking / vector fetch
	output logic        dm_iack,        // interrupt acknowledge cycle
	input  logic        dm_hold1,       // the DMU holds DC1
	// WB maintenance (CINV/CPUSH/PFLUSH/PTEST) in the DMU
	output logic        mt_v,
	output logic  [2:0] mt_op,
	output logic  [1:0] mt_scope,
	output logic  [1:0] mt_caches,
	output logic [31:0] mt_addr,
	output logic  [2:0] mt_fc,
	output logic        mt_ng,
	output logic        mt_wr,
	input  logic        mt_done,
	input  logic [31:0] mt_mmusr,
	output logic        adv_dc1,        // DC1 -> DC2
	output logic        adv_dc2,        // DC2 -> EX
	output logic        adv_ex,         // EX -> WB
	output logic        adv_wb,         // WB completes
	input  logic        dm_dc2_rdy,     // DC2 memory op has its result
	input  logic [31:0] dm_ldata,       // load data, right aligned
	input  logic        dm_fault,       // DC2 access fault (with info below)
	input  logic  [7:0] dm_fvec,        // 2 access error, 3 address error
	input  logic [31:0] dm_faddr,
	input  logic [15:0] dm_fssw,
	output logic        dm_st_v,        // WB store data valid this cycle
	output logic [31:0] dm_st_data,
	input  logic        dm_st_rdy,      // store accepted (WB may complete)
	input  logic        dm_st_fault,
	input  logic [15:0] dm_st_fssw,
	input  logic [31:0] dm_st_faddr,

	// interrupts (synchronized level, 0-7)
	input  logic  [2:0] ipl,
	output logic  [2:0] iack_lvl,
	output logic        rsto_req,
	input  logic        rsto_busy,

	output logic        kill_now,       // this cycle: younger uops are discarded
	output logic        adv_ag,         // AG -> DC1
	output logic [31:0] dtt0,
	output logic [31:0] dtt1,
	output logic [31:0] itt0,
	output logic [31:0] itt1,
	output logic [31:0] tc,
	output logic [31:0] urp,
	output logic [31:0] srp,

	// debug
	output logic [31:0] dbg_pc,
	output logic        dbg_retire,
	output logic        halted
);

//--------------------------------------------------------------------------
// exception routine kinds (sequencer entry points)
//--------------------------------------------------------------------------
localparam logic [3:0] EK_FMT0  = 4'd0;   // format $0 (PC = exc_pc)
localparam logic [3:0] EK_FMT2  = 4'd1;   // format $2 (+ address)
localparam logic [3:0] EK_FMT7  = 4'd2;   // access error
localparam logic [3:0] EK_IRQ   = 4'd3;   // interrupt (IACK + format $0/$1)
localparam logic [3:0] EK_RESET = 4'd4;   // reset: SSP/PC from 0/4
localparam logic [3:0] EK_IRQM  = 4'd5;   // interrupt with M set (throwaway frame)

//--------------------------------------------------------------------------
// pipeline registers
//--------------------------------------------------------------------------
logic        ag_v, dc1_v, dc2_v, ex_v, wb_v;
uop_t        ag_u, dc1_u, dc2_u, ex_u, wb_u;

// operand values carried with the uop
logic [31:0] dc1_a, dc1_b, dc1_ea;
logic [31:0] dc2_a, dc2_b, dc2_ea;
logic [31:0] ex_a,  ex_b,  ex_ea,  ex_ld;
logic        ex_fault;
logic  [7:0] ex_fvec;
logic [31:0] ex_faddr;
logic [15:0] ex_fssw;

// results carried into WB
logic [31:0] wb_res;
logic  [4:0] wb_ccr;          // CCR after this uop
logic        wb_dwe;          // result register write (after EX kills)
logic [31:0] wb_upd_val, wb_upd2_val;
logic        wb_redir;        // the uop redirects the front end
logic [31:0] wb_redir_pc;
logic  [7:0] wb_exc;          // exception vector, 0 none
logic        wb_exc_pcv;      // the frame PC is wb_exc_pc (odd change of flow)
logic [31:0] wb_exc_pc;
logic        wb_oddrte;       // RTE to an odd PC: its SR is committed first
logic [31:0] wb_exc_addr;
logic [15:0] wb_exc_ssw;
logic        wb_st;           // store at WB
logic [31:0] wb_st_data;
logic [15:0] wb_sr_new;       // SR write
logic        wb_sr_we;

// AG register updates travel with the uop for the back file
logic [31:0] dc1_upd, dc1_upd2, dc2_upd, dc2_upd2, ex_upd, ex_upd2;

//--------------------------------------------------------------------------
// register files
//--------------------------------------------------------------------------
// Register files (ap68040_rf, MLAB): the front file holds the newest
// values (AG address updates, EX results), the back file the architectural
// ones (WB).  fv[r] says the front copy of r is current; a redirect or an
// exception clears fv, so every register reads the back file again -- no
// copy.  Five read ports at AG: base, index, upd2 register, A, B.
logic [31:0] fv;
logic  [2:0] fwe, bwe;
logic  [4:0] fwa [3], bwa [3];
logic [31:0] fwd [3], bwd [3];
logic  [4:0] rra [5];
logic [31:0] frd [5], brd [5], rrd [5];
logic  [4:0] ccr_f, ccr_b;

// special registers (architectural, written at WB)
logic [15:0] sr_r;            // T1 T0 S M 0 I2 I1 I0 (CCR lives in ccr_b)
logic [31:0] vbr_r, cacr_r;
logic  [2:0] sfc_r, dfc_r;
logic [31:0] tc_r, itt0_r, itt1_r, dtt0_r, dtt1_r, mmusr_r, urp_r, srp_r;

// exception information for the exception microroutine
logic [31:0] xi_pc, xi_addr;
logic [15:0] xi_sr, xi_vecw, xi_ssw;
logic [31:0] xi_vaddr, xi_ea, xi_w3a, xi_w3d;
logic  [1:0] xw_ph;           // exception entry: back-file writes of T5-T13

assign sr   = {sr_r[15:5], ccr_b};
assign dtt0 = dtt0_r;
assign dtt1 = dtt1_r;
assign itt0 = itt0_r;
assign itt1 = itt1_r;
assign tc   = tc_r;
assign urp  = urp_r;
assign srp  = srp_r;
assign vbr  = vbr_r;
assign cacr = cacr_r;
assign sfc  = sfc_r;
assign dfc  = dfc_r;

//--------------------------------------------------------------------------
// stall / advance (bubbles collapse: a stage moves into an empty one)
//--------------------------------------------------------------------------
logic wb_hold, ex_hold, dc2_hold, ag_hold;
logic stall_wb, stall_ex, stall_dc2, stall_dc1, stall_ag;

// EX holds for a multicycle unit; DC2 for memory; WB for stores/serial ops
logic md_busy;                // multiply/divide running (register)
logic ex_md_start;
// DC2 holds until the data memory unit has its result
assign dc2_hold  = dc2_v && (dc2_u.mem != M_NONE) && (dc2_u.exc == 8'd0) && !dm_dc2_rdy;
assign stall_wb  = wb_v  && wb_hold;
assign stall_ex  = ex_v  && (ex_hold  || stall_wb);
assign stall_dc2 = dc2_v && (dc2_hold || stall_ex);
assign stall_dc1 = dc1_v && (stall_dc2 || dm_hold1);
assign stall_ag  = ag_v  && (ag_hold  || stall_dc1);

assign adv_wb  = wb_v  && !stall_wb;
assign adv_ex  = ex_v  && !stall_ex;
assign adv_dc2 = dc2_v && !stall_dc2;
assign adv_dc1 = dc1_v && !stall_dc1;
assign adv_ag  = ag_v  && !stall_ag;

assign in_rdy  = !stall_ag && !flush && !stopped;

//--------------------------------------------------------------------------
// AG
//--------------------------------------------------------------------------
// interlock: base, index and update registers must not have an older
// writer in DC1/DC2/EX (their value only exists after EX)
function automatic logic pend_w(input logic [4:0] r);
	pend_w = (dc1_v && dc1_u.d_v && dc1_u.d_reg == r) ||
	         (dc2_v && dc2_u.d_v && dc2_u.d_reg == r) ||
	         (ex_v  && ex_u.d_v  && ex_u.d_reg  == r);
endfunction

wire ag_interlock = ag_v && (
	(ag_u.base_v && pend_w(ag_u.base)) ||
	(ag_u.idx_v  && pend_w(ag_u.idx))  ||
	(ag_u.upd_v  && pend_w(ag_u.upd_reg)) ||
	(ag_u.upd2_v && pend_w(ag_u.upd2_reg)));

assign ag_hold = ag_interlock;

ap68040_rf #(.NR(5)) rff (.clk(clk), .we(fwe), .wa(fwa), .wd(fwd), .ra(rra), .rd(frd));
ap68040_rf #(.NR(5)) rfb (.clk(clk), .we(bwe), .wa(bwa), .wd(bwd), .ra(rra), .rd(brd));
assign rra[0] = ag_u.base;
assign rra[1] = ag_u.idx;
assign rra[2] = ag_u.upd2_reg;
assign rra[3] = ag_u.a_reg;
assign rra[4] = ag_u.b_reg;
always_comb for (int i = 0; i < 5; i++) rrd[i] = fv[rra[i]] ? frd[i] : brd[i];

wire [31:0] ag_base = ag_u.base_v ? rrd[0] : 32'd0;
wire [31:0] ag_ix_r = rrd[1];
wire [31:0] ag_ix   = ag_u.idx_v ? ((ag_u.idx_l ? ag_ix_r : {{16{ag_ix_r[15]}}, ag_ix_r[15:0]})
                                    << ag_u.scale) : 32'd0;
wire [31:0] ag_ea   = ag_base + ag_ix + ag_u.disp;
wire [31:0] ag_pinc = ag_base + {{24{ag_u.upd_amt[7]}}, ag_u.upd_amt};
wire [31:0] ag_updv = ag_u.pinc ? ag_pinc : ag_ea;
wire [31:0] ag_u2v  = rrd[2] + {{24{ag_u.upd2_amt[7]}}, ag_u.upd2_amt};
// bit field memory transfers sized by the field (see ucode.py _bf_mem_load)
wire        ag_bfh    = (ag_u.mem != M_NONE) && (ag_u.op == OP_MOV) && (ag_u.cond == 4'hE);
wire        ag_bft    = (ag_u.mem != M_NONE) && (ag_u.op == OP_MOV) && (ag_u.cond == 4'hD);
wire        ag_rte7   = (ag_u.mem != M_NONE) && (ag_u.op == OP_MOV) && (ag_u.cond == 4'hB);
wire        ag_cancel = (ag_bft && !(bf_nb == 3'd3 || bf_nb == 3'd5)) ||
                        (ag_rte7 && !rte_fmt7);
// a continued MOVEM transfers from the stacked EA
wire        ag_cm     = cm_pend && (ag_u.pc == cm_pc) && (ag_u.mem != M_NONE) &&
                        (ag_u.op == OP_MOV) && (ag_u.cond == 4'hC);
wire  [1:0] ag_msz    = ag_bfh ? ((bf_nb == 3'd1) ? SZ_B : (bf_nb <= 3'd3) ? SZ_W : SZ_L)
                               : ag_u.msz;
wire [31:0] ag_maddr  = ag_cm ? cm_ea + ag_u.target :
                        (ag_bft && bf_nb == 3'd3) ? ag_ea - 32'd2 : ag_ea;

// operand read with the EX broadcast of this cycle (see the snoop below)
logic        exw_v;           // EX writes the front file this cycle
logic  [4:0] exw_reg;
logic [31:0] exw_val;

function automatic logic [31:0] opnd(input logic [1:0] src, input logic [4:0] r,
                                     input logic [31:0] rv,
                                     input logic [31:0] imm, input logic sxw);
	logic [31:0] v;
	case (src)
		OS_REG:  v = (exw_v && exw_reg == r) ? exw_val : rv;
		OS_IMM:  v = imm;
		default: v = 32'd0;
	endcase
	opnd = v;
endfunction

wire [31:0] ag_a = ag_u.a_upd ? ag_updv : opnd(ag_u.a_src, ag_u.a_reg, rrd[3], ag_u.imm, ag_u.a_sxw);
wire [31:0] ag_b = ag_u.b_upd ? ag_updv : opnd(ag_u.b_src, ag_u.b_reg, rrd[4], ag_u.imm_b, 1'b0);

// memory request to the DMU, issued as the uop leaves AG
logic s_bit;
assign s_bit      = sr_r[13];
assign dm_req     = adv_ag && (ag_u.mem != M_NONE) && (ag_u.exc == 8'd0) && !ag_cancel;
assign dm_va      = ag_maddr;
assign dm_mem     = ag_u.mem;
assign dm_msz     = ag_msz;
assign dm_lock    = ag_u.mlock;
assign dm_locke   = ag_u.mlocke;
// FC2 of the access selects the root and the supervisor checks: MOVES
// translates its SFC/DFC space (WinUAE: super = (sfc & 4) != 0)
assign dm_super   = dm_fc[2];
assign dm_noalloc = (ag_u.mfc == MFC_SUP);
assign dm_iack    = (ag_u.mfc == MFC_IACK);
always_comb begin
	case (ag_u.mfc)
		MFC_SFC: dm_fc = sfc_r;
		MFC_DFC: dm_fc = dfc_r;
		MFC_SUP: dm_fc = 3'd5;
		MFC_IACK: dm_fc = 3'd7;
		default: dm_fc = ag_u.mprog ? {s_bit, 2'b10} : {s_bit, 2'b01};
	endcase
end

//--------------------------------------------------------------------------
// EX
//--------------------------------------------------------------------------
// operand values entering EX: A or B may be the load data
wire [31:0] ex_av0 = (ex_u.a_src == OS_MEM) ? ex_ld : ex_a;
wire [31:0] ex_av  = ex_u.a_sxw ? {{16{ex_av0[15]}}, ex_av0[15:0]} : ex_av0;
wire [31:0] ex_bv  = (ex_u.b_src == OS_MEM) ? ex_ld : ex_b;

logic [31:0] alu_res;
logic  [4:0] alu_flags;
logic        alu_cc, alu_trap;

ap68040_alu alu (
	.op(ex_u.op), .sz(ex_u.sz), .cond(ex_u.cond),
	.a(ex_av), .b(ex_bv), .ea(ex_ea),
	.flags_in(ccr_f),
	.res(alu_res), .flags_out(alu_flags),
	.cc_true(alu_cc), .trap(alu_trap)
);

// multiply / divide
logic        md_done, md_ovf;
logic [31:0] md_hi, md_lo;
logic [31:0] md_hiin;         // OP_MDHI latch (64-bit dividend high)
logic [31:0] md_rem;          // remainder / product high, for OP_MDRES
logic [31:0] md_lol;          // quotient / product low, for OP_MDRES
logic        md_ovfl;         // the last divide overflowed
logic        md_go;           // this EX uop has started the unit
wire         ex_is_md   = (ex_u.op == OP_MUL) || (ex_u.op == OP_DIV);
wire         ex_div0    = (ex_u.op == OP_DIV) &&
                          ((ex_u.sz == SZ_W) ? (ex_av[15:0] == 16'd0) : (ex_av == 32'd0));
assign ex_md_start = ex_v && ex_is_md && !md_go && !ex_div0 && !md_busy && !ex_fault;

// word forms: DIVU.W/DIVS.W divide the 32-bit Dn by a 16-bit source;
// MULU.W/MULS.W multiply the low words
wire        md_sign = ex_u.cond[0];
wire [31:0] md_a    = (ex_u.sz == SZ_W) ?
                      (md_sign ? {{16{ex_av[15]}}, ex_av[15:0]} : {16'd0, ex_av[15:0]}) : ex_av;
wire [31:0] md_blo  = (ex_u.op == OP_MUL && ex_u.sz == SZ_W) ?
                      (md_sign ? {{16{ex_bv[15]}}, ex_bv[15:0]} : {16'd0, ex_bv[15:0]}) : ex_bv;
wire [31:0] md_bhi  = (ex_u.op == OP_DIV) ?
                      ((ex_u.cond[1]) ? md_hiin : (md_sign ? {32{ex_bv[31]}} : 32'd0)) : 32'd0;

ap68040_muldiv md (
	.clk(clk), .nreset(nreset), .kill(kill_now),
	.start(ex_md_start), .is_div(ex_u.op == OP_DIV), .sign_op(md_sign),
	.op_a(md_a), .op_hi(md_bhi), .op_lo(md_blo),
	.busy(md_busy), .done(md_done), .res_hi(md_hi), .res_lo(md_lo), .ovf(md_ovf)
);

// EX holds while the unit runs, until the cycle its result is presented
assign ex_hold = (ex_v && ex_is_md && !ex_div0 && !ex_fault && !md_done) ||
                 (ex_v && ex_slow && !ex_ph);

// multiply/divide result and flags
logic [31:0] md_res;
logic  [4:0] md_flags;
logic        md_dkill;
always_comb begin
	md_res   = ex_bv;
	md_flags = ccr_f;
	md_dkill = 1'b0;
	if (ex_u.op == OP_MUL) begin
		if (ex_u.sz == SZ_W || !ex_u.cond[1]) begin
			// 32-bit product (MULx.W is 16x16 -> 32)
			md_res   = md_lo;
			md_flags = {ccr_f[4], md_lo[31], md_lo == 32'd0,
			            (ex_u.sz == SZ_L) &&
			            (md_sign ? (md_hi != {32{md_lo[31]}}) : (md_hi != 32'd0)),
			            1'b0};
		end
		else begin
			// 64-bit: this uop writes Dl; OP_MDRES writes Dh
			md_res   = md_lo;
			md_flags = {ccr_f[4], md_hi[31], {md_hi, md_lo} == 64'd0, 1'b0, 1'b0};
		end
	end
	else begin
		// divide
		if (ex_u.sz == SZ_W) begin
			// quotient must fit 16 bits
			logic ovf16;
			ovf16 = md_ovf ||
			        (md_sign ? (md_lo[31:15] != {17{md_lo[15]}}) : (md_lo[31:16] != 16'd0));
			if (ovf16) begin
				md_dkill = 1'b1;
				md_flags = {ccr_f[4], ccr_f[3], ccr_f[2], 1'b1, 1'b0};
			end
			else begin
				md_res   = {md_hi[15:0], md_lo[15:0]};
				md_flags = {ccr_f[4], md_lo[15], md_lo[15:0] == 16'd0, 1'b0, 1'b0};
			end
		end
		else begin
			if (md_ovf) begin
				md_dkill = 1'b1;
				md_flags = {ccr_f[4], ccr_f[3], ccr_f[2], 1'b1, 1'b0};
			end
			else begin
				md_res   = md_lo;
				md_flags = {ccr_f[4], md_lo[31], md_lo == 32'd0, 1'b0, 1'b0};
			end
		end
	end
end

// latch for CAS / bit fields
logic [31:0] ex_latch;

//--------------------------------------------------------------------------
// bit field unit.  BFSET latched the offset (bf_off) and width (bf_w,
// 1..32).  Register form (cond[3] = 0): the field is in B, numbered from
// bit 31, wrapping.  Memory form: B is the long word at EA + offset/8 and
// ex_latch[7:0] the byte after it; the field starts offset & 7 bits in.
// A is the BFINS source.  Flags come from the field, for BFINS from the
// inserted value.
//--------------------------------------------------------------------------
logic [31:0] bf_off;
logic  [5:0] bf_w;
logic  [2:0] bf_nb;             // memory form: bytes holding the field (1..5)

// MOVEM continuation (MC68040UM 8.4.6.5, SSW CM)
logic        rte_fmt7;          // the RTE's frame is format $7
logic        cm_n_v;            // ... and its SSW has CM set
logic [31:0] cm_n_ea;           // ... its EA
logic        cm_pend;           // the next instruction continues a MOVEM
logic [31:0] cm_ea, cm_pc;
logic [31:0] wb_mv_ea;          // the WB MOVEM transfer's calculated EA

// leading zero count of a nonzero 32-bit value, as a balanced tree
function automatic logic [4:0] clz32(input logic [31:0] v);
	logic [4:0] n;
	logic [31:0] x;
	x = v;
	n[4] = (x[31:16] == 16'd0); if (n[4]) x = {x[15:0], 16'd0};
	n[3] = (x[31:24] == 8'd0);  if (n[3]) x = {x[23:0], 8'd0};
	n[2] = (x[31:28] == 4'd0);  if (n[2]) x = {x[27:0], 4'd0};
	n[1] = (x[31:30] == 2'd0);  if (n[1]) x = {x[29:0], 2'd0};
	n[0] = !x[31];
	clz32 = n;
endfunction
logic  [7:0] bf_lonew;          // memory form: the new byte after the long
logic [31:0] bf_res;
logic  [7:0] bf_lo_out;
logic  [4:0] bf_flags;
always_comb begin
	logic [4:0]  o;
	logic [31:0] top, rot, fld, ins_top, nrot, nreg, zf;
	logic [39:0] win, mask40, ins40, nwin;
	logic [5:0]  lz;
	logic        nflag, zflag;
	logic [2:0]  t;
	t   = ex_u.cond[2:0];
	top = (bf_w == 6'd32) ? 32'hFFFF_FFFF : ~(32'hFFFF_FFFF >> bf_w);
	ins_top = (bf_w == 6'd32) ? ex_av : (ex_av << (6'd32 - bf_w));
	if (!ex_u.cond[3]) begin
		o   = bf_off[4:0];
		rot = (ex_bv << o) | ((o == 5'd0) ? 32'd0 : (ex_bv >> (6'd32 - {1'b0, o})));
		win = '0; mask40 = '0; ins40 = '0;
	end
	else begin
		o   = {2'b00, bf_off[2:0]};
		// the bytes read, left justified: B holds the head transfer (byte,
		// word or long), the latch the tail byte
		case (bf_nb)
			3'd1:    win = {ex_bv[7:0], 32'd0};
			3'd2:    win = {ex_bv[15:0], 24'd0};
			3'd3:    win = {ex_bv[15:0], ex_latch[7:0], 16'd0};
			3'd4:    win = {ex_bv, 8'd0};
			default: win = {ex_bv, ex_latch[7:0]};
		endcase
		rot = win[39:8] << o | ({24'd0, win[7:0]} >> (6'd8 - {1'b0, o}));
		mask40 = {top, 8'h00} >> o;
		ins40  = {ins_top, 8'h00} >> o;
	end
	fld = (bf_w == 6'd32) ? rot : (rot >> (6'd32 - bf_w));
	case (t)
		3'd2: nrot = rot ^ top;                     // BFCHG
		3'd4: nrot = rot & ~top;                    // BFCLR
		3'd6: nrot = rot | top;                     // BFSET
		default: nrot = (rot & ~top) | (ins_top & top);   // BFINS
	endcase
	nreg = (nrot >> o) | ((o == 5'd0) ? 32'd0 : (nrot << (6'd32 - {1'b0, o})));
	case (t)
		3'd2: nwin = win ^ mask40;
		3'd4: nwin = win & ~mask40;
		3'd6: nwin = win | mask40;
		default: nwin = (win & ~mask40) | (ins40 & mask40);
	endcase
	// leading zeros of the field (FFO): a log-depth count over the top-
	// aligned field; a zero field gives the width
	zf = rot & top;
	lz = (zf == 32'd0) ? bf_w : {1'b0, clz32(zf)};
	if (t == 3'd7) begin
		nflag = ins_top[31];
		zflag = (ins_top & top) == 32'd0;
	end
	else begin
		nflag = rot[31];
		zflag = zf == 32'd0;
	end
	bf_flags = {ccr_f[4], nflag, zflag, 1'b0, 1'b0};
	// the bytes written back: the head right justified, the tail byte
	bf_lo_out = (bf_nb == 3'd3) ? nwin[23:16] : nwin[7:0];
	case (t)
		3'd1: bf_res = fld;                                            // BFEXTU
		3'd3: bf_res = (bf_w == 6'd32) ? fld :                         // BFEXTS
		               (fld | (rot[31] ? (32'hFFFF_FFFF << bf_w) : 32'd0));
		3'd5: bf_res = bf_off + {26'd0, lz};                           // BFFFO
		3'd0: bf_res = ex_bv;                                          // BFTST
		default: bf_res = !ex_u.cond[3] ? nreg :
		                  (bf_nb == 3'd1) ? {24'd0, nwin[39:32]} :
		                  (bf_nb == 3'd2 || bf_nb == 3'd3) ? {16'd0, nwin[39:24]} :
		                  nwin[39:8];
	endcase
end

// CHK2/CMP2 lower-bound result kept between the two uops
logic        chk2_lo_lt, chk2_lo_eq;

//--------------------------------------------------------------------------
// EX result selection
//--------------------------------------------------------------------------
logic [31:0] ex_res;
logic  [4:0] ex_flags;
logic        ex_dkill;        // suppress the register write
logic        ex_taken;        // control transfer taken
logic [31:0] ex_target;
logic  [7:0] ex_xvec;         // exception raised in EX
logic [31:0] ex_st;           // store data
logic        ex_sr_we;
logic [15:0] ex_sr_new;

// special register read
function automatic logic [31:0] spr_read(input logic [7:0] n);
	case (n)
		8'h00: spr_read = {29'd0, sfc_r};
		8'h01: spr_read = {29'd0, dfc_r};
		8'h02: spr_read = cacr_r;
		8'h03: spr_read = tc_r;
		8'h04: spr_read = itt0_r;
		8'h05: spr_read = itt1_r;
		8'h06: spr_read = dtt0_r;
		8'h07: spr_read = dtt1_r;
		8'h09: spr_read = vbr_r;
		8'h0D: spr_read = mmusr_r;
		8'h0E: spr_read = urp_r;
		8'h0F: spr_read = srp_r;
		8'h10: spr_read = {16'd0, sr_r[15:5], ccr_f};     // SR
		8'h11: spr_read = {27'd0, ccr_f};                   // CCR
		8'h20: spr_read = xi_pc;
		8'h21: spr_read = {16'd0, xi_sr};
		8'h22: spr_read = {16'd0, xi_vecw};
		8'h23: spr_read = xi_addr;
		8'h24: spr_read = {16'd0, xi_ssw};
		8'h25: spr_read = vbr_r + {22'd0, xi_vecw[9:0]};  // vector address
		default: spr_read = 32'd0;
	endcase
endfunction

// slow (two-cycle) operations: computed in EX's first cycle, registered,
// used in the second, so their logic never reaches the result path
function automatic logic is_slow(input logic [6:0] op);
	case (op)
		OP_ASL, OP_ASR, OP_LSL, OP_LSR, OP_ROL, OP_ROR, OP_ROXL, OP_ROXR,
		OP_ABCD, OP_SBCD, OP_NBCD, OP_PACK, OP_UNPK, OP_BF, OP_CHK2B,
		OP_SPR, OP_SRLOG, OP_RTE, OP_RTEF, OP_IACKV, OP_MISC: is_slow = 1'b1;
		default: is_slow = 1'b0;
	endcase
endfunction

typedef struct packed {
	logic [31:0] res;
	logic  [4:0] flags;
	logic        dkill;
	logic        taken;
	logic [31:0] target;
	logic  [7:0] xvec;
	logic        sr_we;
	logic [15:0] sr_new;
} exo_t;

wire  ex_slow = is_slow(ex_u.op);
logic ex_ph;                    // second cycle of a slow op
exo_t xf, xs, xs_q;

logic [31:0] sl_res;
logic  [4:0] sl_flags;
ap68040_alu_slow alu_s (
	.op(ex_u.op), .sz(ex_u.sz), .a(ex_av), .b(ex_bv), .flags_in(ccr_f),
	.res(sl_res), .flags_out(sl_flags)
);

// fast operations
always_comb begin
	xf.res    = alu_res;
	xf.flags  = alu_flags;
	xf.dkill  = 1'b0;
	xf.taken  = 1'b0;
	xf.target = ex_u.target;
	xf.xvec   = 8'd0;
	xf.sr_we  = 1'b0;
	xf.sr_new = {sr_r[15:5], alu_flags};
	case (ex_u.op)
		OP_MUL, OP_DIV: begin
			xf.res   = md_res;
			xf.flags = md_flags;
			xf.dkill = md_dkill;
			if (ex_div0) begin
				xf.xvec  = 8'd5;
				xf.flags = {ccr_f[4:1], 1'b0};
			end
		end
		OP_MDRES: begin
			xf.res   = ex_u.cond[0] ? md_lol : md_rem;
			xf.dkill = md_ovfl;
		end
		OP_MDHI:   xf.res = ex_av;
		OP_BCC:    xf.taken = alu_cc;
		OP_DBCC: begin
			xf.taken = !alu_cc && (alu_res[15:0] != 16'hFFFF);
			xf.dkill = alu_cc;
		end
		OP_TRAPCC: if (alu_trap) xf.xvec = 8'd7;
		OP_CHK:    if (alu_trap) xf.xvec = 8'd6;
		OP_CCRLOG: xf.res = ex_bv;
		OP_SPW:    xf.res = ex_av;
		OP_LATCH:  xf.res = ex_av;
		OP_CAS2C: begin
			if (!ex_u.cond[0] || cas2_eq) xf.flags = cmp_flags;
			else xf.flags = ccr_f;
		end
		OP_CAS2R: begin
			xf.res   = (ex_av & ex_szm) | (ex_bv & ~ex_szm);
			xf.dkill = cas2_eq;
		end
		OP_BFSET:  xf.res = ex_u.cond[0] ? {24'd0, bf_lonew} : {{3{ex_av[31]}}, ex_av[31:3]};
		OP_CAS: begin
			xf.flags = alu_flags;
			xf.res   = ex_bv;
		end
		default: ;
	endcase
	// control transfers
	case (ex_u.br)
		BR_IMM:  begin xf.taken = 1'b1; xf.target = ex_u.target; end
		BR_EA:   begin xf.taken = 1'b1; xf.target = ex_ea; end
		BR_A:    begin xf.taken = 1'b1; xf.target = ex_av; end
		BR_B:    begin xf.taken = 1'b1; xf.target = ex_bv; end
		default: ;
	endcase
end

// slow operations (first EX cycle)
always_comb begin
	xs.res    = sl_res;
	xs.flags  = sl_flags;
	xs.dkill  = 1'b0;
	xs.taken  = 1'b0;
	xs.target = ex_u.target;
	xs.xvec   = 8'd0;
	xs.sr_we  = 1'b0;
	xs.sr_new = {sr_r[15:5], ccr_f};
	case (ex_u.op)
		OP_CHK2B: begin
			// WinUAE i_CHK2: signed compares of the sign-extended bounds
			// (lower in ex_latch, upper in A) with B, which is sign-extended
			// only when it is a data register (cond[1]); the 68040 leaves N, V
			logic signed [31:0] lo, up, rg;
			logic z, c;
			case (ex_u.sz)
				SZ_B: begin lo = {{24{ex_latch[7]}}, ex_latch[7:0]}; up = {{24{ex_av[7]}}, ex_av[7:0]};
				            rg = ex_u.cond[1] ? {{24{ex_bv[7]}}, ex_bv[7:0]} : ex_bv; end
				SZ_W: begin lo = {{16{ex_latch[15]}}, ex_latch[15:0]}; up = {{16{ex_av[15]}}, ex_av[15:0]};
				            rg = ex_u.cond[1] ? {{16{ex_bv[15]}}, ex_bv[15:0]} : ex_bv; end
				default: begin lo = ex_latch; up = ex_av; rg = ex_bv; end
			endcase
			z = (up == rg) || (lo == rg);
			c = !z && ((lo <= up) ? ((rg < lo) || (rg > up)) : ((rg > up) && (rg < lo)));
			xs.res   = ex_bv;
			xs.dkill = 1'b1;
			xs.flags = {ccr_f[4], ccr_f[3], z, ccr_f[1], c};
			if (ex_u.cond[0] && c) xs.xvec = 8'd6;
		end
		OP_SRLOG: begin
			logic [15:0] cur, n;
			cur = {sr_r[15:5], ccr_f};
			case (ex_u.cond[1:0])
				2'd0:    n = cur & ex_av[15:0];
				2'd1:    n = cur | ex_av[15:0];
				2'd2:    n = cur ^ ex_av[15:0];
				default: n = ex_av[15:0];
			endcase
			xs.sr_new = n & 16'hF71F;
			xs.sr_we  = 1'b1;
			xs.flags  = xs.sr_new[4:0];
		end
		OP_SPR: begin
			logic [31:0] v, m;
			v = spr_read(ex_u.imm[7:0]);
			m = (ex_u.sz == SZ_B) ? 32'h0000_00FF : (ex_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
			xs.res = (v & m) | (ex_bv & ~m);
			xs.flags = ccr_f;
		end
		OP_MISC: begin
			xs.flags = ccr_f;
			xs.res   = ex_av;              // maintenance: the address
			if (ex_u.cond == 4'd2) begin
				// STOP #imm: SR = imm, then wait for an interrupt
				xs.sr_new = ex_av[15:0] & 16'hF71F;
				xs.sr_we  = 1'b1;
				xs.flags  = xs.sr_new[4:0];
			end
		end
		OP_IACKV: begin
			// IACK data: [7:0] vector, [8] AVEC, [9] TEA (spurious)
			logic [7:0] v;
			xs.flags = ccr_f;
			if (!ex_u.cond[0]) begin
				v = ex_av[9] ? 8'd24 : ex_av[8] ? (8'd24 + {5'd0, irq_lvl_r}) : ex_av[7:0];
				xs.res = {22'd0, v, 2'b00};                 // format $0 word
			end
			else begin
				v = ex_latch[9] ? 8'd24 : ex_latch[8] ? (8'd24 + {5'd0, irq_lvl_r}) : ex_latch[7:0];
				xs.res = vbr_r + {22'd0, v, 2'b00};         // vector address
			end
		end
		OP_RTEF: begin
			// 68040 frame formats: $0 8, $1 8 (throwaway), $2 12, $3 12,
			// $7 60 bytes; others are a format error -- including $4, which
			// "the MC68040 does not generate or recognize" (MC68040UM 8.4.5;
			// only the LC/EC040 use it)
			xs.flags = ccr_f;
			if (ex_u.cond[0]) begin
				// FRESTORE header (FPU in reset state): only a NULL frame
				// (version byte $00) is compatible, anything else is a
				// format error at the FRESTORE
				xs.res = ex_av;
				if (ex_av[31:24] != 8'd0) xs.xvec = 8'd14;
			end
			else case (ex_av[15:12])
				4'h0, 4'h1: xs.res = 32'd8;
				4'h2, 4'h3: xs.res = 32'd12;
				4'h7:       xs.res = 32'd60;
				default: begin xs.res = 32'd0; xs.xvec = 8'd14; end
			endcase
		end
		OP_RTE: begin
			xs.sr_new = ex_av[15:0] & 16'hF71F;
			xs.sr_we  = 1'b1;
			xs.flags  = xs.sr_new[4:0];
			xs.taken  = 1'b1;
			xs.target = rte_fmt1 ? ex_u.pc : ex_bv;
		end
		OP_BF: begin
			xs.res   = bf_res;
			xs.flags = bf_flags;
		end
		default: ;
	endcase
end

// EX outputs: the registered slow result in a slow op's second cycle
always_comb begin
	exo_t x;
	x = ex_slow ? xs_q : xf;
	ex_res    = x.res;
	ex_flags  = x.flags;
	ex_dkill  = x.dkill;
	ex_taken  = x.taken;
	ex_target = x.target;
	ex_xvec   = x.xvec;
	ex_sr_we  = x.sr_we;
	ex_sr_new = x.sr_new;
end

// CAS: flags of (memory - Dc) and the equal decision
wire [31:0] cas_dc_m = ex_latch;
logic [31:0] cas_res;
logic  [4:0] cas_flags;
logic        cas_cc, cas_tr;
ap68040_alu cas_alu (
	.op(OP_CMP), .sz(ex_u.sz), .cond(4'd0),
	.a(cas_dc_m), .b(ex_bv), .ea(32'd0), .flags_in(ccr_f),
	.res(cas_res), .flags_out(cas_flags), .cc_true(cas_cc), .trap(cas_tr)
);
wire cas_eq = cas_flags[2];

// compare B - A (CAS2)
logic [31:0] cmp_res;
logic  [4:0] cmp_flags;
logic        cmp_cc, cmp_tr;
ap68040_alu cmp_alu (
	.op(OP_CMP), .sz(ex_u.sz), .cond(4'd0),
	.a(ex_av), .b(ex_bv), .ea(32'd0), .flags_in(ccr_f),
	.res(cmp_res), .flags_out(cmp_flags), .cc_true(cmp_cc), .trap(cmp_tr)
);
logic        cas2_eq;           // CAS2: both pairs equal so far
logic        rte_fmt1;          // RTE: the frame is a throwaway ($1)
wire  [31:0] ex_szm = (ex_u.sz == SZ_B) ? 32'h0000_00FF :
                      (ex_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;

// store data, and stores the EX op cancels
logic ex_stkill;
always_comb begin
	ex_stkill = 1'b0;
	case (ex_u.op)
		OP_CAS:  ex_st = cas_eq ? ex_av : ex_bv;
		OP_CAS2W: begin
			ex_st     = (ex_u.cond[0] && !cas2_eq) ? ex_bv : ex_av;
			ex_stkill = !ex_u.cond[0] && !cas2_eq;
		end
		default: ex_st = ex_res;
	endcase
end

// a store into the instruction stream just ahead (see EX -> WB)
wire ex_smc = (ex_u.mem == M_ST || ex_u.mem == M_RMW) &&
              ((ex_ea[31:6] == ex_u.npc[31:6]) ||
               (ex_ea[31:6] == ex_u.npc[31:6] + 26'd1));
logic smc_pend;

// mispredict: the actual path differs from the one the front end took
wire        ex_br      = (ex_u.br != BR_NONE) || (ex_u.op == OP_RTE);
wire [31:0] ex_next    = ex_taken ? ex_target : ex_u.npc;
wire        ex_mispred = ex_br && ((ex_taken != ex_u.pred) ||
                                   (ex_taken && ex_u.pred && ex_target != ex_u.target));

// a change of flow to an odd address is an address error at the
// instruction (format $2, the address with A0 cleared), before any of its
// side effects -- the BSR/JSR push, the RTS/RTR pop -- except that RTE has
// committed its SR and RTR its CCR.  Bcc and DBcc validate their target
// taken or not (DBcc before its condition: Dn is left untouched).
// The frame PC follows gencpu, as the cputest 68040 AE group records it:
// the instruction, except JMP (opcode + 2; + 6 for the indexed modes), JSR
// (the odd target: the fault is the fetch there) and the handler fetch of
// an exception (the vector offset, without VBR).
wire [31:0] ex_otgt = ex_taken ? ex_target : ex_u.target;
wire        ex_odd  = ex_br && ex_otgt[0] &&
                      (ex_taken || ex_u.op == OP_BCC || ex_u.op == OP_DBCC);
logic [31:0] ex_odd_pc;
always_comb begin
	ex_odd_pc = ex_u.pc;
	if (ex_u.br == BR_EA)
		ex_odd_pc = ex_u.pc + (((ex_u.imm_b[5:3] == 3'd6) ||
		                        (ex_u.imm_b[5:0] == 6'o73)) ? 32'd6 : 32'd2);
	else if (ex_u.br == BR_B && ex_u.op == OP_MOV)
		ex_odd_pc = ex_otgt;
	else if (ex_u.br == BR_A && ex_u.op == OP_MOV && ex_u.cond == 4'hF)
		ex_odd_pc = ex_ea - vbr_r;
end

// front-file write by EX, and its broadcast to younger operands
wire ex_d_eff = ex_u.d_v && !ex_dkill && !(ex_u.op == OP_CAS && cas_eq);
assign exw_v   = adv_ex && ex_d_eff && !ex_fault && (ex_xvec == 8'd0) && !ex_odd &&
                 (ex_u.exc == 8'd0);
assign exw_reg = ex_u.d_reg;
assign exw_val = (ex_u.op == OP_CAS) ? ((ex_bv & ex_szm) | (ex_latch & ~ex_szm)) : ex_res;

//--------------------------------------------------------------------------
// WB
//--------------------------------------------------------------------------
// interrupt recognition at instruction boundaries: a level above the mask,
// or a new level 7 (edge triggered, not masked)
logic  [2:0] ipl_q;
logic        nmi_edge;
logic  [2:0] irq_lvl_r;       // level being acknowledged
// the mask in force at the boundary: an instruction that writes SR (MOVE
// to SR, RTE, ANDI/ORI/EORI to SR) decides with its new mask, so a request
// it unmasks is taken at that very boundary (cputest irq/all).  A request
// that already beat the mask holds IPEND (ipend_r) and is taken at the next
// boundary although the instruction raises the mask; IPEND lasts only while
// the request is held at that level or above.
logic        ipend_r;
logic  [2:0] ipend_lvl;
wire  [2:0]  irq_mask = (wb_v && wb_sr_we && wb_exc == 8'd0) ? wb_sr_new[10:8] : sr_r[10:8];
wire         irq_pend = nmi_edge ||
                        (ipl_q != 3'd7 && (ipl_q > irq_mask ||
                                           (ipend_r && ipl_q >= ipend_lvl)));
wire  [2:0]  irq_lvl  = nmi_edge ? 3'd7 : ipl_q;

logic        stopped;         // STOP: waiting for an interrupt
logic [31:0] stop_pc;         // the instruction after STOP
logic        trace_defer;     // a trace held back by an interrupt
logic [31:0] trace_addr;      // its instruction address
logic        rst_issued;      // RESET instruction: RSTO requested
logic        wb_cof;          // the uop's control transfer was taken

wire wb_is_exc   = wb_v && (wb_exc != 8'd0);
wire wb_commit   = adv_wb && !wb_is_exc;
wire wb_bound    = wb_commit && wb_u.last;
wire [31:0] wb_next = wb_redir_pc;

// trace uses the T bits the instruction started with (sr_r before its own
// SR write): T1 every instruction, T0 taken transfers and the 68040 list
wire tr_now      = sr_r[15] || (sr_r[14] && (wb_u.t0cof || wb_cof));
wire wb_is_stop  = (wb_u.op == OP_MISC) && (wb_u.cond == 4'd2);
// STOP is traced under T1, and under T0 only when it changes the upper SR
// byte (WinUAE MakeFromSR_x); a traced STOP does not stop
wire stop_traced = wb_is_stop && (sr_r[15] || (sr_r[14] && (wb_sr_new[15:8] != sr_r[15:8])));
// an interrupt accepted at a boundary where a trace is also due goes first;
// the trace is held (trace_defer) and taken at the end of the interrupt's
// exception processing, stacking the handler's entry address
wire trace_due   = wb_bound && (wb_is_stop ? stop_traced : (tr_now || trace_defer));
// a MOVEM continuation armed by RTE runs before a pending interrupt
wire cm_block    = cm_pend || (wb_v && wb_u.op == OP_RTE && cm_n_v);
wire take_irq    = irq_pend && !cm_block && (wb_bound || (stopped && !wb_v));
wire take_trace  = trace_due && !take_irq;
// a speculative fetch fault (code 1) is no exception: WB refetches the
// instruction once on demand, and only a second fault there is an access
// error (rt_armed/rt_pc)
// exceptions that stack the next instruction: the instruction completed
wire wb_post_trap = (wb_exc >= 8'd32 && wb_exc < 8'd48) ||
                    wb_exc == 8'd5 || wb_exc == 8'd6 || wb_exc == 8'd7;
wire wb_refetch  = adv_wb && wb_v && (wb_exc == EXC_IFS || wb_exc == EXC_IFSA);
logic        rt_armed;
logic [31:0] rt_pc;
wire x_go        = (adv_wb && wb_is_exc && !wb_refetch) || take_trace || take_irq;

// register file write ports.  Front: the AG address updates and the EX
// result.  Back: the WB commit (a result beats an address update), the
// address update of a trap after its instruction, and at exception entry
// the routine's operands (T5-T13, three per cycle).
always_comb begin
	logic wcom, wpost;
	fwe[0] = adv_ag && ag_u.upd_v;  fwa[0] = ag_u.upd_reg;  fwd[0] = ag_updv;
	fwe[1] = adv_ag && ag_u.upd2_v; fwa[1] = ag_u.upd2_reg; fwd[1] = ag_u2v;
	fwe[2] = exw_v;                 fwa[2] = exw_reg;       fwd[2] = exw_val;
	wcom  = wb_commit;
	wpost = adv_wb && wb_is_exc && wb_post_trap;
	bwe[0] = (wcom || wpost) && wb_u.upd2_v; bwa[0] = wb_u.upd2_reg; bwd[0] = wb_upd2_val;
	bwe[1] = (wcom || wpost) && wb_u.upd_v;  bwa[1] = wb_u.upd_reg;  bwd[1] = wb_upd_val;
	bwe[2] = wcom && wb_dwe;                 bwa[2] = wb_u.d_reg;    bwd[2] = wb_res;
	case (xw_ph)
		2'd1: begin
			bwe = 3'b111;
			bwa[0] = R_T0 + 5'd8;  bwd[0] = xi_vaddr;
			bwa[1] = R_T0 + 5'd9;  bwd[1] = xi_pc;
			bwa[2] = R_T0 + 5'd10; bwd[2] = {16'd0, xi_sr};
		end
		2'd2: begin
			bwe = 3'b111;
			bwa[0] = R_T0 + 5'd11; bwd[0] = xi_addr;
			bwa[1] = R_T0 + 5'd12; bwd[1] = {16'd0, xi_vecw};
			bwa[2] = R_T0 + 5'd13; bwd[2] = {16'd0, xi_ssw};
		end
		2'd3: begin
			bwe = 3'b111;
			bwa[0] = R_T0 + 5'd5;  bwd[0] = xi_ea;
			bwa[1] = R_T0 + 5'd6;  bwd[1] = xi_w3a;
			bwa[2] = R_T0 + 5'd7;  bwd[2] = xi_w3d;
		end
		default: ;
	endcase
end


// WB holds: store not accepted yet; RESET until RSTO is done
wire wb_reset    = (wb_u.op == OP_MISC) && (wb_u.cond == 4'd1);
wire wb_mt       = (wb_u.op == OP_MISC) && (wb_u.cond >= 4'd3) && (wb_u.cond <= 4'd5);
logic mt_seen;                // the DMU finished this WB uop's maintenance
assign wb_hold = wb_v && !wb_is_exc &&
                 ((wb_st && !dm_st_rdy) || (wb_st && dm_st_fault) ||
                  (wb_reset && (!rst_issued || rsto_busy)) ||
                  (wb_mt && !mt_seen));

assign mt_v      = wb_v && wb_mt && !wb_is_exc && !mt_seen;
assign mt_op     = (wb_u.cond == 4'd3) ? (wb_u.imm_b[5] ? 3'd2 : 3'd1) :
                   (wb_u.cond == 4'd4) ? 3'd3 : 3'd4;
assign mt_scope  = (wb_u.cond == 4'd3) ? wb_u.imm_b[4:3] :
                   (wb_u.cond == 4'd4) ? (wb_u.imm_b[4] ? 2'd3 : 2'd2) : 2'd2;
assign mt_caches = wb_u.imm_b[7:6];
assign mt_addr   = wb_res;
assign mt_fc     = dfc_r;
assign mt_ng     = (wb_u.cond == 4'd4) && !wb_u.imm_b[3];
assign mt_wr     = (wb_u.cond == 4'd5) && !wb_u.imm_b[5];

logic        rst_seq;         // reset exception pending (first cycles)

// same-cycle kill of every younger uop: WB takes an exception or redirects
assign kill_now = (adv_wb && (wb_is_exc || wb_redir)) || x_go;

// exception entry values
logic  [7:0] x_vec;
logic  [3:0] x_fmt;
logic [31:0] x_pc, x_addr;
logic [15:0] x_osr, x_ssw, x_nsr;
logic  [3:0] x_kind;
logic  [4:0] x_ssp;
logic        x_wfault;
logic        x_cm;             // SSW CM: a MOVEM continues from x_cm_ea
logic [31:0] x_cm_ea;
always_comb begin
	logic [15:0] cur;
	// the SR the exception sees: after this instruction when it completed
	cur = wb_commit ? (wb_sr_we ? (wb_sr_new & 16'hF71F) : {sr_r[15:5], wb_ccr})
	                : {sr_r[15:5], wb_ccr};
	if (stopped && !wb_v) cur = {sr_r[15:5], ccr_b};
	if (adv_wb && wb_is_exc && wb_oddrte) cur = wb_sr_new & 16'hF71F;
	x_osr  = cur;
	x_ssw  = 16'd0;
	x_addr = wb_u.pc;
	x_pc   = wb_u.pc;
	x_vec  = wb_exc;
	if (adv_wb && wb_is_exc) begin
		x_vec  = wb_exc;
		x_addr = wb_exc_addr;
		x_ssw  = wb_exc_ssw;
		if (x_cm) x_ssw = x_ssw | 16'h1000;
		// TRAP #n, TRAPcc, CHK and divide-by-zero stack the next
		// instruction; faults and illegal opcodes the instruction
		x_pc   = ((wb_exc >= 8'd32 && wb_exc < 8'd48) ||
		          wb_exc == 8'd5 || wb_exc == 8'd6 || wb_exc == 8'd7) ? wb_u.npc : wb_u.pc;
		if (wb_exc_pcv) x_pc = wb_exc_pc;
	end
	else if (take_irq) begin
		x_vec  = 8'd24 + {5'd0, irq_lvl};   // replaced by the IACK result
		x_pc   = (stopped && !wb_v) ? stop_pc : wb_next;
	end
	else begin
		// trace: format $2, PC = where execution continues, address = the
		// traced instruction
		x_vec  = 8'd9;
		x_pc   = wb_next;
		x_addr = trace_defer ? trace_addr : wb_u.pc;
	end
	x_fmt  = (x_vec == 8'd2) ? 4'h7 :
	         (x_vec == 8'd3 || x_vec == 8'd5 || x_vec == 8'd6 || x_vec == 8'd7 ||
	          x_vec == 8'd9) ? 4'h2 : 4'h0;
	x_kind = take_irq && !(adv_wb && wb_is_exc) ? (cur[12] ? EK_IRQM : EK_IRQ) :
	         (x_fmt == 4'h7) ? EK_FMT7 : (x_fmt == 4'h2) ? EK_FMT2 : EK_FMT0;
	x_ssp  = cur[12] ? R_MSP : R_ISP;
	x_wfault = (x_fmt == 4'h7) && !x_ssw[8];

	// the SR inside the handler: S set, T cleared; an interrupt raises the
	// mask to its level and clears M
	x_nsr  = {2'b00, 1'b1, cur[12], cur[11:0]};
	if (take_irq && !(adv_wb && wb_is_exc)) begin
		x_nsr[12]   = 1'b0;
		x_nsr[10:8] = irq_lvl;
	end
end

// an access error of a MOVEM transfer in an indexed or PC-relative mode,
// or of a continued MOVEM (also its refetch), sets CM and stacks the EA
always_comb begin
	x_cm    = adv_wb && wb_is_exc && (wb_exc == 8'd2) &&
	          ((wb_u.op == OP_MOV && wb_u.cond == 4'hC && wb_u.mem != M_NONE) ||
	           (cm_pend && wb_u.pc == cm_pc));
	x_cm_ea = (cm_pend && wb_u.pc == cm_pc) ? cm_ea : wb_mv_ea;
end

always_ff @(posedge clk) begin
	redir_v <= 1'b0;
	exc_go  <= 1'b0;
	flush   <= 1'b0;
	ucond_v <= 1'b0;
	dbg_retire <= 1'b0;

	if (!nreset) begin
		ag_v  <= 1'b0; dc1_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
		fv <= 32'd0;
		xw_ph <= 2'd0;
		ccr_f   <= 5'd0;
		ccr_b   <= 5'd0;
		sr_r    <= 16'h2700;
		vbr_r   <= 32'd0;
		cacr_r  <= 32'd0;
		sfc_r   <= 3'd0;
		dfc_r   <= 3'd0;
		tc_r    <= 32'd0;
		itt0_r  <= 32'd0; itt1_r <= 32'd0; dtt0_r <= 32'd0; dtt1_r <= 32'd0;
		mmusr_r <= 32'd0; urp_r  <= 32'd0; srp_r  <= 32'd0;
		xi_pc   <= 32'd0; xi_addr <= 32'd0; xi_sr <= 16'd0; xi_vecw <= 16'd0; xi_ssw <= 16'd0;
		md_go   <= 1'b0;
		ex_ph   <= 1'b0;
		xs_q    <= '0;
		md_hiin <= 32'd0;
		md_rem  <= 32'd0;
		md_lol  <= 32'd0;
		md_ovfl <= 1'b0;
		ex_latch <= 32'd0;
		bf_off   <= 32'd0;
		bf_w     <= 6'd32;
		bf_lonew <= 8'd0;
		chk2_lo_lt <= 1'b0; chk2_lo_eq <= 1'b0;
		cas2_eq <= 1'b0;
		rte_fmt1 <= 1'b0;
		ipl_q   <= 3'd0;
		nmi_edge <= 1'b0;
		irq_lvl_r <= 3'd0;
		ipend_r  <= 1'b0;
		ipend_lvl <= 3'd0;
		rt_armed <= 1'b0;
		rt_pc    <= 32'd0;
		smc_pend <= 1'b0;
		rte_fmt7 <= 1'b0;
		cm_n_v   <= 1'b0;
		cm_n_ea  <= 32'd0;
		cm_pend  <= 1'b0;
		cm_ea    <= 32'd0;
		cm_pc    <= 32'd0;
		stopped <= 1'b0;
		stop_pc <= 32'd0;
		trace_defer <= 1'b0;
		trace_addr <= 32'd0;
		rst_issued <= 1'b0;
		mt_seen <= 1'b0;
		wb_cof  <= 1'b0;
		halted  <= 1'b0;
		rst_seq <= 1'b1;
		exc_kind <= EK_RESET;
		exc_ssp <= R_ISP;
		redir_pc <= 32'd0;
	end
	else begin
		//------------------------------------------------------------------
		// reset exception: the routine loads ISP and PC from 0 and 4
		//------------------------------------------------------------------
		if (rst_seq) begin
			rst_seq  <= 1'b0;
			exc_go   <= 1'b1;
			exc_kind <= EK_RESET;
			exc_ssp  <= R_ISP;
		end

		ipl_q <= ipl;
		if (ipl_q == 3'd0 || ipl_q == 3'd7 || (ipend_r && ipl_q < ipend_lvl))
			ipend_r <= 1'b0;
		else if (ipl_q > sr_r[10:8] && (!ipend_r || ipl_q > ipend_lvl)) begin
			ipend_r   <= 1'b1;
			ipend_lvl <= ipl_q;
		end
		if (ipl == 3'd7 && ipl_q != 3'd7) nmi_edge <= 1'b1;
		else if (ipl != 3'd7) nmi_edge <= 1'b0;

		//------------------------------------------------------------------
		// AG -> DC1
		//------------------------------------------------------------------
		if (adv_ag) begin
			dc1_u   <= ag_u;
			dc1_u.msz <= ag_msz;
			if (ag_cancel) begin
				// no tail byte: a load gives zero, a store nothing
				dc1_u.mem   <= M_NONE;
				dc1_u.a_src <= OS_ZERO;
			end
			dc1_a   <= ag_a;
			dc1_b   <= ag_b;
			dc1_ea  <= ag_u.ag ? ag_maddr : 32'd0;
			dc1_upd <= ag_updv;
			dc1_upd2 <= ag_u2v;
		end
		if (!stall_dc1) dc1_v <= adv_ag;
		else begin
			if (exw_v && dc1_u.a_src == OS_REG && dc1_u.a_reg == exw_reg) dc1_a <= exw_val;
			if (exw_v && dc1_u.b_src == OS_REG && dc1_u.b_reg == exw_reg) dc1_b <= exw_val;
		end
		// snoop into the uop entering DC1
		if (adv_ag && exw_v) begin
			if (ag_u.a_src == OS_REG && ag_u.a_reg == exw_reg) dc1_a <= exw_val;
			if (ag_u.b_src == OS_REG && ag_u.b_reg == exw_reg) dc1_b <= exw_val;
		end

		//------------------------------------------------------------------
		// DC1 -> DC2
		//------------------------------------------------------------------
		if (adv_dc1) begin
			dc2_u    <= dc1_u;
			dc2_a    <= (exw_v && dc1_u.a_src == OS_REG && dc1_u.a_reg == exw_reg) ? exw_val : dc1_a;
			dc2_b    <= (exw_v && dc1_u.b_src == OS_REG && dc1_u.b_reg == exw_reg) ? exw_val : dc1_b;
			dc2_ea   <= dc1_ea;
			dc2_upd  <= dc1_upd;
			dc2_upd2 <= dc1_upd2;
		end
		else if (dc2_v && exw_v) begin
			if (dc2_u.a_src == OS_REG && dc2_u.a_reg == exw_reg) dc2_a <= exw_val;
			if (dc2_u.b_src == OS_REG && dc2_u.b_reg == exw_reg) dc2_b <= exw_val;
		end
		if (!stall_dc2) dc2_v <= adv_dc1;

		//------------------------------------------------------------------
		// DC2 -> EX
		//------------------------------------------------------------------
		if (adv_dc2) begin
			ex_u     <= dc2_u;
			ex_a     <= (exw_v && dc2_u.a_src == OS_REG && dc2_u.a_reg == exw_reg) ? exw_val : dc2_a;
			ex_b     <= (exw_v && dc2_u.b_src == OS_REG && dc2_u.b_reg == exw_reg) ? exw_val : dc2_b;
			ex_ea    <= dc2_ea;
			ex_ld    <= dm_ldata;
			ex_upd   <= dc2_upd;
			ex_upd2  <= dc2_upd2;
			ex_fault <= (dc2_u.mem != M_NONE) && dm_fault;
			ex_fvec  <= dm_fvec;
			ex_faddr <= dm_faddr;
			ex_fssw  <= dm_fssw;
			md_go    <= 1'b0;
		end
		if (!stall_ex) ex_v <= adv_dc2;
		if (ex_md_start) md_go <= 1'b1;
		if (ex_v && ex_slow && !ex_ph) begin
			ex_ph <= 1'b1;
			xs_q  <= xs;
		end
		if (adv_ex || kill_now) ex_ph <= 1'b0;

		//------------------------------------------------------------------
		// EX -> WB
		//------------------------------------------------------------------
		if (adv_ex) begin
			wb_u        <= ex_u;
			wb_res      <= exw_val;
			wb_dwe      <= ex_d_eff;
			wb_upd_val  <= ex_upd;
			wb_upd2_val <= ex_upd2;
			wb_st       <= (ex_u.mem == M_ST || ex_u.mem == M_RMW) && !ex_stkill;
			if (ex_u.op == OP_CAS2C)
				cas2_eq <= ex_u.cond[0] ? (cas2_eq && cmp_flags[2]) : cmp_flags[2];
			wb_st_data  <= ex_st;
			// a store into the instruction stream just ahead (the fetch
			// queue and the instructions in flight) refetches after itself:
			// the 68040 does not promise this, the previous core's AmigaOS
			// boot depended on it (t_integer 192)
			// (a store from a uop before the last one -- MOVEM, a bit
			// field, CAS2 -- defers the refetch to the instruction's end:
			// redirecting there would drop the instruction's remaining uops)
			wb_redir    <= ex_mispred || ex_u.ser || ex_sr_we ||
			               (ex_u.last && (smc_pend || ex_smc));
			if (ex_u.last) smc_pend <= 1'b0;
			else if (ex_smc) smc_pend <= 1'b1;
			// where execution continues, predicted correctly or not
			wb_redir_pc <= ex_br ? ex_next : ex_u.npc;
			wb_sr_we    <= ex_sr_we;
			wb_cof      <= ex_br && ex_taken;
			wb_sr_new   <= ex_sr_new;
			if (ex_u.exc != 8'd0) begin
				logic ifs, ifa;
				// instruction fetch faults: a speculative one is refetched
				// once (wb_refetch) and becomes an access error when it
				// faults again at the same instruction
				ifs = (ex_u.exc == EXC_IFS || ex_u.exc == EXC_IFSA);
				ifa = (ex_u.exc == EXC_IFA || ex_u.exc == EXC_IFSA);
				if (ifs && !(rt_armed && rt_pc == ex_u.pc))
					wb_exc <= ex_u.exc;
				else if (ifs || ex_u.exc == EXC_IFB || ex_u.exc == EXC_IFA)
					wb_exc <= 8'd2;
				else
					wb_exc <= ex_u.exc;
				// a faulted instruction fetch: the faulting word's address;
				// SSW ATC for an MMU fault, read, long, TT normal, TM the
				// program space of the instruction's privilege
				if (ifs || ex_u.exc == EXC_IFB || ex_u.exc == EXC_IFA) begin
					wb_exc_addr <= ex_u.target;
					wb_exc_ssw  <= {5'd0, ifa, 1'b0, 1'b1, 3'b000, 2'b00,
					                sr_r[13] ? 3'd6 : 3'd2};
				end
				else begin
					wb_exc_addr <= ex_u.pc;
					wb_exc_ssw  <= 16'd0;
				end
			end
			else if (ex_fault) begin
				wb_exc      <= ex_fvec;
				wb_exc_addr <= ex_faddr;
				wb_exc_ssw  <= ex_fssw;
			end
			else if (ex_xvec == 8'd0 && ex_odd) begin
				wb_exc      <= 8'd3;
				wb_exc_addr <= {ex_otgt[31:1], 1'b0};
				wb_exc_ssw  <= 16'd0;
			end
			else begin
				wb_exc      <= ex_xvec;
				wb_exc_addr <= ex_u.pc;
				wb_exc_ssw  <= 16'd0;
			end
			wb_exc_pcv <= ex_u.exc == 8'd0 && !ex_fault && ex_xvec == 8'd0 && ex_odd;
			wb_exc_pc  <= ex_odd_pc;
			wb_oddrte  <= ex_u.exc == 8'd0 && !ex_fault && ex_xvec == 8'd0 && ex_odd &&
			              ex_u.op == OP_RTE;
			// CCR after this uop (CHK, CHK2, TRAPcc and divide-by-zero set
			// their flags and then trap: the stacked SR carries them)
			if (ex_u.exc == 8'd0 && !ex_fault) begin
				logic [4:0] nf;
				nf = (ex_u.op == OP_CAS) ? cas_flags : ex_flags;
				ccr_f  <= (nf & ex_u.ccr_we) | (ccr_f & ~ex_u.ccr_we);
				wb_ccr <= (nf & ex_u.ccr_we) | (ccr_f & ~ex_u.ccr_we);
			end
			else wb_ccr <= ccr_f;
			if (ex_u.op == OP_MDHI)  md_hiin  <= ex_av;
			if (ex_u.op == OP_LATCH || (ex_u.op == OP_IACKV && !ex_u.cond[0])) ex_latch <= ex_av;
			if (ex_u.op == OP_RTEF && !ex_u.cond[0]) begin
				rte_fmt1 <= (ex_av[15:12] == 4'h1);
				rte_fmt7 <= (ex_av[15:12] == 4'h7);
			end
			if (ex_u.op == OP_MISC && ex_u.cond == 4'd6) begin
				cm_n_v  <= rte_fmt7 && ex_bv[12];
				cm_n_ea <= ex_av;
			end
			wb_mv_ea <= ex_ea - ex_u.target;
			if (ex_u.op == OP_BFSET && !ex_u.cond[0]) begin
				logic [5:0] w;
				logic [6:0] e;
				w = (ex_bv[4:0] == 5'd0) ? 6'd32 : {1'b0, ex_bv[4:0]};
				e = {4'd0, ex_av[2:0]} + {1'b0, w} + 7'd7;
				bf_off <= ex_av;
				bf_w   <= w;
				bf_nb  <= e[5:3];
			end
			if (ex_u.op == OP_BF) bf_lonew <= bf_lo_out;
		end
		if (md_done) begin
			md_rem  <= md_hi;
			md_lol  <= md_lo;
		end
		// overflow of the uop leaving EX (word forms check 16 bits)
		if (adv_ex && (ex_u.op == OP_DIV || ex_u.op == OP_MUL)) md_ovfl <= (ex_u.op == OP_DIV) && md_dkill;
		if (!stall_wb) wb_v <= adv_ex;

		//------------------------------------------------------------------
		// WB commit
		//------------------------------------------------------------------
		if (wb_commit) begin
			dbg_retire <= wb_u.last;
			// architectural commit: the back file's write ports (bwe)
			ccr_b <= wb_ccr;
			if (wb_u.op == OP_SPW) begin
				case (wb_u.imm_b[7:0])
					8'h00: sfc_r  <= wb_res[2:0];
					8'h01: dfc_r  <= wb_res[2:0];
					8'h02: cacr_r <= wb_res & 32'h8000_8000;
					8'h03: tc_r   <= wb_res & 32'h0000_C000;
					8'h04: itt0_r <= wb_res & 32'hFFFF_E364;
					8'h05: itt1_r <= wb_res & 32'hFFFF_E364;
					8'h06: dtt0_r <= wb_res & 32'hFFFF_E364;
					8'h07: dtt1_r <= wb_res & 32'hFFFF_E364;
					8'h09: vbr_r  <= wb_res;
					8'h0D: mmusr_r <= wb_res;
					8'h0E: urp_r  <= wb_res & 32'hFFFF_FE00;
					8'h0F: srp_r  <= wb_res & 32'hFFFF_FE00;
					default: ;
				endcase
			end
			if (wb_sr_we) begin
				sr_r  <= {wb_sr_new[15:5], 5'd0};
				ccr_b <= wb_sr_new[4:0];
			end
			if (wb_is_stop && !stop_traced) begin
				stopped <= 1'b1;
				stop_pc <= wb_u.npc;
			end
			if (wb_redir) begin
				flush    <= 1'b1;
				redir_v  <= 1'b1;
				redir_pc <= wb_redir_pc;
				// the front file restarts from the back one (fv cleared below)
				ccr_f <= wb_sr_we ? wb_sr_new[4:0] : wb_ccr;
			end
			if (wb_bound) trace_defer <= 1'b0;
			if (wb_bound) rt_armed <= 1'b0;
			if (wb_bound) begin
				// RTE of a format $7 frame with CM arms the continuation of
				// the MOVEM it returns to; any other instruction ends it
				cm_pend <= (wb_u.op == OP_RTE) && cm_n_v;
				cm_ea   <= cm_n_ea;
				cm_pc   <= wb_next;
				if (wb_u.op == OP_RTE) cm_n_v <= 1'b0;
			end
		end
		if (wb_refetch) begin
			flush    <= 1'b1;
			redir_v  <= 1'b1;
			redir_pc <= wb_u.pc;
			ccr_f    <= ccr_b;
			rt_armed <= 1'b1;
			rt_pc    <= wb_u.pc;
		end

		//------------------------------------------------------------------
		// exception entry: a faulting/trapping uop (its effects discarded,
		// trap flags kept), a trace after an instruction, or an interrupt
		// at an instruction boundary.  The front file restarts from the
		// back file plus this uop's committed writes; T8..T13 carry the
		// routine's operands.
		//------------------------------------------------------------------
		if (x_go) begin
			logic [15:0] vw;
			rt_armed <= 1'b0;
			cm_pend  <= 1'b0;
			smc_pend <= 1'b0;
			vw = {x_fmt, 2'b00, x_vec, 2'b00};
			flush    <= 1'b1;
			redir_v  <= 1'b0;
			xw_ph    <= 2'd1;          // T5-T13 first, then exc_go
			exc_kind <= x_kind;
			exc_ssp  <= x_ssp;
			xi_sr    <= x_osr;
			xi_vecw  <= vw;
			xi_pc    <= x_pc;
			xi_addr  <= x_addr;
			xi_ssw   <= x_ssw;
			sr_r     <= {x_nsr[15:5], 5'd0};
			ccr_b    <= x_osr[4:0];
			ccr_f    <= x_osr[4:0];
			stopped  <= 1'b0;
			if (take_irq && !(adv_wb && wb_is_exc)) begin
				irq_lvl_r <= irq_lvl;
				ipend_r   <= 1'b0;
				if (irq_lvl == 3'd7) nmi_edge <= 1'b0;
				// a trace due at this boundary is taken at the handler
				if (trace_due) begin
					trace_defer <= 1'b1;
					trace_addr  <= wb_u.pc;
				end
			end
			// the routine's operands, written into the back file by the
			// xw sequence: T8 vector address, T9 PC, T10 SR, T11 address,
			// T12 format/vector word, T13 SSW; format $7: T5 the EA field
			// (the MOVEM EA with CM, else FA), T6/T7 WB3A/WB3D of a write
			// fault (WB3S stays clear: the instruction restarts)
			xi_vaddr <= vbr_r + {22'd0, x_vec, 2'b00};
			xi_ea    <= x_cm ? x_cm_ea : x_addr;
			xi_w3a   <= x_wfault ? x_addr : 32'd0;
			xi_w3d   <= x_wfault ? wb_st_data : 32'd0;
		end

		// exception entry, continued: three cycles of back-file writes
		if (xw_ph != 2'd0) begin
			xw_ph <= xw_ph + 2'd1;
			if (xw_ph == 2'd3) begin
				xw_ph  <= 2'd0;
				exc_go <= 1'b1;
			end
		end

		// front-valid bits: a front write makes the register's front copy
		// current; a redirect, a refetch or an exception drops them all
		for (int i = 0; i < 3; i++) if (fwe[i]) fv[fwa[i]] <= 1'b1;
		if ((wb_commit && wb_redir) || wb_refetch || x_go) fv <= 32'd0;

		// a store that took a bus error at WB becomes an access fault of
		// its instruction (WB holds one cycle, then takes it)
		if (wb_v && wb_st && dm_st_rdy && dm_st_fault && wb_exc == 8'd0) begin
			wb_exc      <= 8'd2;
			wb_exc_addr <= dm_st_faddr;
			wb_exc_ssw  <= dm_st_fssw;
		end
		// maintenance done: WB completes next cycle; PTEST sets MMUSR
		if (wb_v && wb_mt && mt_done) begin
			mt_seen <= 1'b1;
			if (wb_u.cond == 4'd5) mmusr_r <= mt_mmusr;
		end
		if (adv_wb) mt_seen <= 1'b0;

		// RESET instruction: RSTO for 512 bus clocks, WB waits
		if (wb_v && wb_reset && !rst_issued && !wb_is_exc) rst_issued <= 1'b1;
		if (adv_wb) rst_issued <= 1'b0;

		//------------------------------------------------------------------
		// flush: kill every stage (the redirecting/excepting uop has left)
		//------------------------------------------------------------------
		if (flush) begin
			ag_v <= 1'b0; dc1_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
		end
		else begin
			// D2 -> AG (exactly when in_rdy says the uop is taken)
			if (!stall_ag) ag_v <= in_v && !stopped;
			if (!stall_ag && in_v && !stopped) ag_u <= in_u;
		end
		if (kill_now) begin
			ag_v <= 1'b0; dc1_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
		end
	end
end

assign dm_st_v    = wb_v && wb_st && !wb_is_exc;
assign iack_lvl   = irq_lvl_r;
assign rsto_req   = wb_v && wb_reset && !rst_issued && !wb_is_exc;
assign dm_st_data = wb_st_data;
assign dbg_pc     = wb_u.pc;
assign ucond      = 1'b0;

endmodule
