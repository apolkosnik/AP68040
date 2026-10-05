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
logic [31:0] rf_f [32];       // front: newest values
logic [31:0] rf_b [32];       // back: architectural
logic  [4:0] ccr_f, ccr_b;

// special registers (architectural, written at WB)
logic [15:0] sr_r;            // T1 T0 S M 0 I2 I1 I0 (CCR lives in ccr_b)
logic [31:0] vbr_r, cacr_r;
logic  [2:0] sfc_r, dfc_r;
logic [31:0] tc_r, itt0_r, itt1_r, dtt0_r, dtt1_r, mmusr_r, urp_r, srp_r;

// exception information for the exception microroutine
logic [31:0] xi_pc, xi_addr;
logic [15:0] xi_sr, xi_vecw, xi_ssw;

assign sr   = {sr_r[15:5], ccr_b};
assign dtt0 = dtt0_r;
assign dtt1 = dtt1_r;
assign itt0 = itt0_r;
assign itt1 = itt1_r;
assign tc   = tc_r;
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
assign stall_dc1 = dc1_v && stall_dc2;
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

wire [31:0] ag_base = ag_u.base_v ? rf_f[ag_u.base] : 32'd0;
wire [31:0] ag_ix_r = rf_f[ag_u.idx];
wire [31:0] ag_ix   = ag_u.idx_v ? ((ag_u.idx_l ? ag_ix_r : {{16{ag_ix_r[15]}}, ag_ix_r[15:0]})
                                    << ag_u.scale) : 32'd0;
wire [31:0] ag_ea   = ag_base + ag_ix + ag_u.disp;
wire [31:0] ag_pinc = ag_base + {{24{ag_u.upd_amt[7]}}, ag_u.upd_amt};
wire [31:0] ag_updv = ag_u.pinc ? ag_pinc : ag_ea;
wire [31:0] ag_u2v  = rf_f[ag_u.upd2_reg] + {{24{ag_u.upd2_amt[7]}}, ag_u.upd2_amt};
wire [31:0] ag_maddr = ag_ea;

// operand read with the EX broadcast of this cycle (see the snoop below)
logic        exw_v;           // EX writes the front file this cycle
logic  [4:0] exw_reg;
logic [31:0] exw_val;

function automatic logic [31:0] opnd(input logic [1:0] src, input logic [4:0] r,
                                     input logic [31:0] imm, input logic sxw);
	logic [31:0] v;
	case (src)
		OS_REG:  v = (exw_v && exw_reg == r) ? exw_val : rf_f[r];
		OS_IMM:  v = imm;
		default: v = 32'd0;
	endcase
	opnd = v;
endfunction

wire [31:0] ag_a = opnd(ag_u.a_src, ag_u.a_reg, ag_u.imm, ag_u.a_sxw);
wire [31:0] ag_b = opnd(ag_u.b_src, ag_u.b_reg, ag_u.imm_b, 1'b0);

// memory request to the DMU, issued as the uop leaves AG
logic s_bit;
assign s_bit      = sr_r[13];
assign dm_req     = adv_ag && (ag_u.mem != M_NONE) && (ag_u.exc == 8'd0);
assign dm_va      = ag_maddr;
assign dm_mem     = ag_u.mem;
assign dm_msz     = ag_u.msz;
assign dm_lock    = ag_u.mlock;
assign dm_locke   = ag_u.mlocke;
assign dm_super   = (ag_u.mfc == MFC_SUP) ? 1'b1 : s_bit;
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
assign ex_hold = ex_v && ex_is_md && !ex_div0 && !ex_fault && !md_done;

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
		win = {ex_bv, ex_latch[7:0]};
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
	// leading zeros of the field (FFO)
	zf = rot & top;
	lz = bf_w;
	for (int i = 0; i < 32; i++)
		if (zf[i] && (6'(31 - i) < lz)) lz = 6'(31 - i);
	if (t == 3'd7) begin
		nflag = ins_top[31];
		zflag = (ins_top & top) == 32'd0;
	end
	else begin
		nflag = rot[31];
		zflag = zf == 32'd0;
	end
	bf_flags = {ccr_f[4], nflag, zflag, 1'b0, 1'b0};
	bf_lo_out = nwin[7:0];
	case (t)
		3'd1: bf_res = fld;                                            // BFEXTU
		3'd3: bf_res = (bf_w == 6'd32) ? fld :                         // BFEXTS
		               (fld | (rot[31] ? (32'hFFFF_FFFF << bf_w) : 32'd0));
		3'd5: bf_res = bf_off + {26'd0, lz};                           // BFFFO
		3'd0: bf_res = ex_bv;                                          // BFTST
		default: bf_res = ex_u.cond[3] ? nwin[39:8] : nreg;
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

always_comb begin
	ex_res    = alu_res;
	ex_flags  = alu_flags;
	ex_dkill  = 1'b0;
	ex_taken  = 1'b0;
	ex_target = ex_u.target;
	ex_xvec   = 8'd0;
	ex_sr_we  = 1'b0;
	ex_sr_new = {sr_r[15:5], alu_flags};

	case (ex_u.op)
		OP_MUL, OP_DIV: begin
			ex_res   = md_res;
			ex_flags = md_flags;
			ex_dkill = md_dkill;
			if (ex_div0) begin
				ex_xvec  = 8'd5;
				ex_flags = {ccr_f[4:1], 1'b0};
			end
		end
		OP_MDRES: begin
			ex_res   = ex_u.cond[0] ? md_lol : md_rem;
			ex_dkill = md_ovfl;
		end
		OP_MDHI:  ex_res = ex_av;
		OP_BCC: begin
			ex_taken = alu_cc;
		end
		OP_DBCC: begin
			ex_taken = !alu_cc && (alu_res[15:0] != 16'hFFFF);
			ex_dkill = alu_cc;
		end
		OP_TRAPCC: if (alu_trap) ex_xvec = 8'd7;
		OP_CHK:    if (alu_trap) ex_xvec = 8'd6;
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
			ex_flags = {ccr_f[4], ccr_f[3], z, ccr_f[1], c};
			if (ex_u.cond[0] && c) ex_xvec = 8'd6;
		end
		OP_CCRLOG: ex_res = ex_bv;
		OP_SRLOG: begin
			logic [15:0] cur;
			cur = {sr_r[15:5], ccr_f};
			case (ex_u.cond[1:0])
				2'd0:    ex_sr_new = cur & ex_av[15:0];
				2'd1:    ex_sr_new = cur | ex_av[15:0];
				2'd2:    ex_sr_new = cur ^ ex_av[15:0];
				default: ex_sr_new = ex_av[15:0];
			endcase
			ex_sr_new = ex_sr_new & 16'hF71F;
			ex_sr_we  = 1'b1;
			ex_flags  = ex_sr_new[4:0];
		end
		OP_SPR: begin
			logic [31:0] v, m;
			v = spr_read(ex_u.imm[7:0]);
			m = (ex_u.sz == SZ_B) ? 32'h0000_00FF : (ex_u.sz == SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
			ex_res = (v & m) | (ex_bv & ~m);
		end
		OP_SPW: ex_res = ex_av;
		OP_LATCH: ex_res = ex_av;
		OP_CAS2C: begin
			if (!ex_u.cond[0] || cas2_eq) ex_flags = cmp_flags;
			else ex_flags = ccr_f;
		end
		OP_MISC: begin
			if (ex_u.cond == 4'd2) begin
				// STOP #imm: SR = imm, then wait for an interrupt
				ex_sr_new = ex_av[15:0] & 16'hF71F;
				ex_sr_we  = 1'b1;
				ex_flags  = ex_sr_new[4:0];
			end
		end
		OP_IACKV: begin
			// IACK data: [7:0] vector, [8] AVEC, [9] TEA (spurious)
			logic [7:0] v;
			v = ex_latch[9] ? 8'd24 : ex_latch[8] ? (8'd24 + {5'd0, irq_lvl_r}) : ex_latch[7:0];
			if (!ex_u.cond[0]) begin
				v = ex_av[9] ? 8'd24 : ex_av[8] ? (8'd24 + {5'd0, irq_lvl_r}) : ex_av[7:0];
				ex_res = {22'd0, v, 2'b00};                 // format $0 word
			end
			else
				ex_res = vbr_r + {22'd0, v, 2'b00};         // vector address
		end
		OP_RTEF: begin
			// 68040 frame formats (WinUAE i_RTE): $0 8, $1 8 (throwaway),
			// $2 12, $3 12, $4 16, $7 60 bytes; others: format error
			case (ex_av[15:12])
				4'h0, 4'h1: ex_res = 32'd8;
				4'h2, 4'h3: ex_res = 32'd12;
				4'h4:       ex_res = 32'd16;
				4'h7:       ex_res = 32'd60;
				default: begin ex_res = 32'd0; ex_xvec = 8'd14; end
			endcase
		end
		OP_RTE: begin
			ex_sr_new = ex_av[15:0] & 16'hF71F;
			ex_sr_we  = 1'b1;
			ex_flags  = ex_sr_new[4:0];
			ex_taken  = 1'b1;
			ex_target = rte_fmt1 ? ex_u.pc : ex_bv;
		end
		OP_CAS2R: begin
			ex_res   = (ex_av & ex_szm) | (ex_bv & ~ex_szm);
			ex_dkill = cas2_eq;
		end
		OP_BF: begin
			ex_res   = bf_res;
			ex_flags = bf_flags;
		end
		OP_BFSET: ex_res = ex_u.cond[0] ? {24'd0, bf_lonew} : {{3{ex_av[31]}}, ex_av[31:3]};
		OP_CAS: begin
			// compare memory (B) with Dc (latch); equal: store Du (A),
			// else: Dc = memory and the memory value is written back
			logic [32:0] d;
			ex_flags = alu_flags;     // the uop runs OP_CAS through CMP below
			ex_res   = ex_bv;
		end
		default: ;
	endcase

	// control transfers
	case (ex_u.br)
		BR_IMM:  begin ex_taken = 1'b1; ex_target = ex_u.target; end
		BR_EA:   begin ex_taken = 1'b1; ex_target = ex_ea; end
		BR_A:    begin ex_taken = 1'b1; ex_target = ex_av; end
		BR_B:    begin ex_taken = 1'b1; ex_target = ex_bv; end
		default: ;
	endcase
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

// mispredict: the actual path differs from the one the front end took
wire        ex_br      = (ex_u.br != BR_NONE) || (ex_u.op == OP_RTE);
wire [31:0] ex_next    = ex_taken ? ex_target : ex_u.npc;
wire        ex_mispred = ex_br && ((ex_taken != ex_u.pred) ||
                                   (ex_taken && ex_u.pred && ex_target != ex_u.target));

// front-file write by EX, and its broadcast to younger operands
wire ex_d_eff = ex_u.d_v && !ex_dkill && !(ex_u.op == OP_CAS && cas_eq);
assign exw_v   = adv_ex && ex_d_eff && !ex_fault && (ex_xvec == 8'd0) && (ex_u.exc == 8'd0);
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
wire         irq_pend = nmi_edge || (ipl_q != 3'd7 && ipl_q > sr_r[10:8]);
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
wire [31:0] wb_next = wb_redir ? wb_redir_pc : wb_u.npc;

// trace uses the T bits the instruction started with (sr_r before its own
// SR write): T1 every instruction, T0 taken transfers and the 68040 list
wire tr_now      = sr_r[15] || (sr_r[14] && (wb_u.t0cof || wb_cof));
wire wb_is_stop  = (wb_u.op == OP_MISC) && (wb_u.cond == 4'd2);
// STOP is traced under T1, and under T0 only when it changes the upper SR
// byte (WinUAE MakeFromSR_x); a traced STOP does not stop
wire stop_traced = wb_is_stop && (sr_r[15] || (sr_r[14] && (wb_sr_new[15:8] != sr_r[15:8])));
wire take_trace  = wb_bound && (wb_is_stop ? stop_traced : (tr_now || trace_defer));
wire take_irq    = irq_pend && ((wb_bound && !take_trace) || (stopped && !wb_v));
wire x_go        = (adv_wb && wb_is_exc) || take_trace || take_irq;

// WB holds: store not accepted yet; RESET until RSTO is done
wire wb_reset    = (wb_u.op == OP_MISC) && (wb_u.cond == 4'd1);
assign wb_hold = wb_v && !wb_is_exc &&
                 ((wb_st && !dm_st_rdy) || (wb_reset && (!rst_issued || rsto_busy)));

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
always_comb begin
	logic [15:0] cur;
	// the SR the exception sees: after this instruction when it completed
	cur = wb_commit ? (wb_sr_we ? (wb_sr_new & 16'hF71F) : {sr_r[15:5], wb_ccr})
	                : {sr_r[15:5], wb_ccr};
	if (stopped && !wb_v) cur = {sr_r[15:5], ccr_b};
	x_osr  = cur;
	x_ssw  = 16'd0;
	x_addr = wb_u.pc;
	x_pc   = wb_u.pc;
	x_vec  = wb_exc;
	if (adv_wb && wb_is_exc) begin
		x_vec  = wb_exc;
		x_addr = wb_exc_addr;
		x_ssw  = wb_exc_ssw;
		// TRAP #n, TRAPcc, CHK and divide-by-zero stack the next
		// instruction; faults and illegal opcodes the instruction
		x_pc   = ((wb_exc >= 8'd32 && wb_exc < 8'd48) ||
		          wb_exc == 8'd5 || wb_exc == 8'd6 || wb_exc == 8'd7) ? wb_u.npc : wb_u.pc;
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
	// the SR inside the handler: S set, T cleared; an interrupt raises the
	// mask to its level and clears M
	x_nsr  = {2'b00, 1'b1, cur[12], cur[11:0]};
	if (take_irq && !(adv_wb && wb_is_exc)) begin
		x_nsr[12]   = 1'b0;
		x_nsr[10:8] = irq_lvl;
	end
end

always_ff @(posedge clk) begin
	redir_v <= 1'b0;
	exc_go  <= 1'b0;
	flush   <= 1'b0;
	ucond_v <= 1'b0;
	dbg_retire <= 1'b0;

	if (!nreset) begin
		ag_v  <= 1'b0; dc1_v <= 1'b0; dc2_v <= 1'b0; ex_v <= 1'b0; wb_v <= 1'b0;
		for (int i = 0; i < 32; i++) begin rf_f[i] <= 32'd0; rf_b[i] <= 32'd0; end
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
		stopped <= 1'b0;
		stop_pc <= 32'd0;
		trace_defer <= 1'b0;
		trace_addr <= 32'd0;
		rst_issued <= 1'b0;
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
		if (ipl == 3'd7 && ipl_q != 3'd7) nmi_edge <= 1'b1;
		else if (ipl != 3'd7) nmi_edge <= 1'b0;

		//------------------------------------------------------------------
		// AG -> DC1
		//------------------------------------------------------------------
		if (adv_ag) begin
			dc1_u   <= ag_u;
			dc1_a   <= ag_a;
			dc1_b   <= ag_b;
			dc1_ea  <= ag_u.ag ? ag_ea : 32'd0;
			dc1_upd <= ag_updv;
			dc1_upd2 <= ag_u2v;
			if (ag_u.upd_v)  rf_f[ag_u.upd_reg]  <= ag_updv;
			if (ag_u.upd2_v) rf_f[ag_u.upd2_reg] <= ag_u2v;
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

		//------------------------------------------------------------------
		// EX -> WB
		//------------------------------------------------------------------
		if (exw_v) rf_f[exw_reg] <= exw_val;
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
			wb_redir    <= ex_mispred || ex_u.ser || ex_sr_we ||
			               ((ex_u.mem == M_ST || ex_u.mem == M_RMW) &&
			                ((ex_ea[31:6] == ex_u.npc[31:6]) ||
			                 (ex_ea[31:6] == ex_u.npc[31:6] + 26'd1)));
			wb_redir_pc <= ex_mispred ? ex_next : ex_u.npc;
			wb_sr_we    <= ex_sr_we;
			wb_cof      <= ex_br && ex_taken;
			wb_sr_new   <= ex_sr_new;
			if (ex_u.exc != 8'd0) begin
				wb_exc      <= ex_u.exc;
				wb_exc_addr <= ex_u.pc;
				wb_exc_ssw  <= 16'd0;
			end
			else if (ex_fault) begin
				wb_exc      <= ex_fvec;
				wb_exc_addr <= ex_faddr;
				wb_exc_ssw  <= ex_fssw;
			end
			else begin
				wb_exc      <= ex_xvec;
				wb_exc_addr <= ex_u.pc;
				wb_exc_ssw  <= 16'd0;
			end
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
			if (ex_u.op == OP_RTEF) rte_fmt1 <= (ex_av[15:12] == 4'h1);
			if (ex_u.op == OP_BFSET && !ex_u.cond[0]) begin
				bf_off <= ex_av;
				bf_w   <= (ex_bv[4:0] == 5'd0) ? 6'd32 : {1'b0, ex_bv[4:0]};
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
			// architectural commit (a result beats an address update)
			if (wb_u.upd2_v)   rf_b[wb_u.upd2_reg] <= wb_upd2_val;
			if (wb_u.upd_v)    rf_b[wb_u.upd_reg] <= wb_upd_val;
			if (wb_dwe)        rf_b[wb_u.d_reg]   <= wb_res;
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
				for (int i = 0; i < 32; i++) rf_f[i] <= rf_b[i];
				// this uop's own writes go to both files
				if (wb_u.upd2_v) rf_f[wb_u.upd2_reg] <= wb_upd2_val;
				if (wb_u.upd_v)  rf_f[wb_u.upd_reg] <= wb_upd_val;
				if (wb_dwe)      rf_f[wb_u.d_reg]   <= wb_res;
				ccr_f <= wb_sr_we ? wb_sr_new[4:0] : wb_ccr;
			end
			if (wb_bound) trace_defer <= 1'b0;
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
			vw = {x_fmt, 2'b00, x_vec, 2'b00};
			flush    <= 1'b1;
			redir_v  <= 1'b0;
			exc_go   <= 1'b1;
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
				if (irq_lvl == 3'd7) nmi_edge <= 1'b0;
				// a trace due at this boundary is taken at the handler
				if (take_trace || (wb_bound && tr_now)) begin
					trace_defer <= 1'b1;
					trace_addr  <= wb_u.pc;
				end
			end
			for (int i = 0; i < 32; i++) rf_f[i] <= rf_b[i];
			if (wb_commit) begin
				if (wb_u.upd2_v) rf_f[wb_u.upd2_reg] <= wb_upd2_val;
				if (wb_u.upd_v)  rf_f[wb_u.upd_reg] <= wb_upd_val;
				if (wb_dwe)      rf_f[wb_u.d_reg]   <= wb_res;
			end
			rf_b[R_T0 + 8]  <= vbr_r + {22'd0, x_vec, 2'b00};
			rf_b[R_T0 + 9]  <= x_pc;
			rf_b[R_T0 + 10] <= {16'd0, x_osr};
			rf_b[R_T0 + 11] <= x_addr;
			rf_b[R_T0 + 12] <= {16'd0, vw};
			rf_b[R_T0 + 13] <= {16'd0, x_ssw};
			rf_f[R_T0 + 8]  <= vbr_r + {22'd0, x_vec, 2'b00};
			rf_f[R_T0 + 9]  <= x_pc;
			rf_f[R_T0 + 10] <= {16'd0, x_osr};
			rf_f[R_T0 + 11] <= x_addr;
			rf_f[R_T0 + 12] <= {16'd0, vw};
			rf_f[R_T0 + 13] <= {16'd0, x_ssw};
		end

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
