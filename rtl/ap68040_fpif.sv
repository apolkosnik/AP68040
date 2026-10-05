//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_fpif.sv - the FPU at the EX stage                                //
//                                                                          //
// Wraps the floating point unit (ap040_fpu: registers, conversion,        //
// arithmetic, FSAVE state) for the pipeline.  Microcode moves the data    //
// (memory, Dn, FMOVEM, frames); the OP_FPU uops below run the FPU side in //
// EX, in program order, and act only once they are the oldest uop in     //
// flight (WB empty), so no FPU side effect is ever taken back by an older //
// exception.  Sub-operation in uop.cond:                                  //
//                                                                          //
//   CHK   first uop of every FPU instruction (A = extension word, B =     //
//         operation word): waits for a background operation, delivers a  //
//         pending arithmetic exception (pre-instruction: format $0, PC =  //
//         this instruction), latches the words, raises the encoding       //
//         faults (F-line, illegal, unimplemented EA) before any side      //
//         effect, and records FPIAR where the 68040 does                  //
//   EAL   latches the operand address (for later exception frames)        //
//   DISP  the FPU operation: FPm/<ea>/Dn to FPn, FMOVECR, FPn to <ea>;    //
//         A, B and the latch carry the operand's long words               //
//   GET   long word 1 or 2 (imm) of a store result or an FMOVEM register //
//   END   last uop of a store, FMOVEM, FSAVE or FRESTORE: raises a store  //
//         exception after the stores, acknowledges FSAVE, applies         //
//         FRESTORE                                                        //
//   CRR   CRW   control register slot read / write (imm / immb: 0 FPCR,  //
//         1 FPSR, 2 FPIAR; a slot not in the list does nothing)           //
//   LIST  FMOVEM: latches the register list (static, or Dn = A);          //
//         result: the signed byte adjustment of (An)+ / -(An)             //
//   MVR   MVW   FMOVEM: raw register read / write of the next slot        //
//   COND  FBcc / FScc / FTRAPcc;  DBCC  FDBcc (B = Dn)                     //
//   SV0   SVW   FSAVE: frame size (signed for -(An)) / frame word imm/4   //
//   RS0   RSW   FRESTORE: header check, frame size / frame word immb/4    //
//                                                                          //
// Register-destination arithmetic past every datatype check is released  //
// to the background (as the 68040 runs its FPU beside the integer unit); //
// an enabled exception of a released operation becomes pending and is    //
// taken at the next FPU instruction.  The rules follow the previous       //
// core's corpus-validated FPU sequencing (WinUAE fpp.cpp).                 //
//--------------------------------------------------------------------------//

module ap68040_fpif
	import ap68040_pkg::*;
#(
	parameter logic [7:0] FPU_REVISION = 8'h41
)(
	input  logic        clk,
	input  logic        nreset,

	// the uop in EX
	input  logic        ex_v,           // an OP_FPU uop is in EX
	input  logic  [3:0] sub,
	input  logic  [7:0] imm,            // operand A constant
	input  logic  [7:0] immb,           // operand B constant
	input  logic [31:0] av,
	input  logic [31:0] bv,
	input  logic [31:0] latch,
	input  logic [31:0] ea,             // the uop's EA (EAL)
	input  logic [31:0] pc,
	input  logic        safe,           // no older uop is in flight
	input  logic        adv,            // the uop leaves EX this cycle
	input  logic        kill,

	output logic        hold,
	output logic [31:0] res,
	output logic        dkill,          // no register write
	output logic        taken,          // FBcc / FDBcc
	output logic  [7:0] xvec,           // exception, 0 none
	output logic  [3:0] xfmt,
	output logic        xnext,          // stacked PC: the next instruction
	output logic [31:0] xaddr,          // frame address field
	output logic        xcommit,        // the uop's address updates stand
	output logic        st_kill,        // stores of this instruction suppressed

	// dynamic cancels at AG
	output logic  [7:0] mv_slots,       // FMOVEM slots that hold a register
	output logic  [2:0] cr_slots,       // FMOVEM control slots FPCR FPSR FPIAR
	output logic  [4:0] sv_words,       // FSAVE frame long words
	output logic  [4:0] rs_words        // FRESTORE frame long words
);

localparam logic [3:0] FC_CHK = 4'd0, FC_DISP = 4'd1, FC_GET = 4'd2, FC_END = 4'd3,
                       FC_CRR = 4'd4, FC_CRW = 4'd5, FC_LIST = 4'd6, FC_MVR = 4'd7,
                       FC_MVW = 4'd8, FC_COND = 4'd9, FC_DBCC = 4'd10, FC_SV0 = 4'd11,
                       FC_SVW = 4'd12, FC_RS0 = 4'd13, FC_RSW = 4'd14, FC_EAL = 4'd15;

localparam logic [7:0] VEC_ILL = 8'd4, VEC_TRAPCC = 8'd7, VEC_FLINE = 8'd11,
                       VEC_FMTERR = 8'd14, VEC_BSUN = 8'd48, VEC_OPERR = 8'd52,
                       VEC_SNAN = 8'd54, VEC_UNSUP = 8'd55;

localparam bit          REV40       = (FPU_REVISION == 8'h40);
localparam logic [31:0] IDLE_HDR    = {FPU_REVISION, 24'd0};
localparam logic [31:0] UNIMP_HDR   = {FPU_REVISION, REV40 ? 8'h28 : 8'h30, 16'd0};
localparam logic [31:0] BUSY_HDR    = {FPU_REVISION, 8'h60, 16'd0};
localparam logic [4:0]  UNIMP_WORDS = REV40 ? 5'd11 : 5'd13;

//--------------------------------------------------------------------------
// helpers
//--------------------------------------------------------------------------
function automatic logic [3:0] fp_bytes(input logic [2:0] f);
	case (f)
		3'd0, 3'd1: fp_bytes = 4'd4;
		3'd4:       fp_bytes = 4'd2;
		3'd6:       fp_bytes = 4'd1;
		3'd5:       fp_bytes = 4'd8;
		default:    fp_bytes = 4'd12;
	endcase
endfunction

// WinUAE fault_if_nonexisting_opmode: 1 = F-line (format $0, no side
// effects), 2 = opmodes $78-$7F, which the 68040 reports as illegal
function automatic logic [1:0] opmode_class(input logic [6:0] op);
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
			opmode_class = 2'd1;
		7'h78, 7'h79, 7'h7A, 7'h7B, 7'h7C, 7'h7D, 7'h7E, 7'h7F:
			opmode_class = 2'd2;
		default:
			opmode_class = 2'd0;
	endcase
endfunction

// the opmodes the 68040 executes in hardware (the others: the FPSP)
function automatic logic op_in_hw(input logic [6:0] op);
	case (op)
		7'h00, 7'h40, 7'h44, 7'h18, 7'h58, 7'h5C, 7'h1A, 7'h5A, 7'h5E,
		7'h38, 7'h3A, 7'h22, 7'h62, 7'h66, 7'h28, 7'h68, 7'h6C,
		7'h23, 7'h27, 7'h63, 7'h67, 7'h20, 7'h24, 7'h60, 7'h64,
		7'h04, 7'h41, 7'h45:
			op_in_hw = 1'b1;
		default:
			op_in_hw = 1'b0;
	endcase
endfunction

// IEEE predicate over the FPSR condition codes {N, Z, I, NAN}
function automatic logic fp_cond(input logic [5:0] pred, input logic [3:0] cc);
	logic n, z, nan;
	n = cc[3]; z = cc[2]; nan = cc[0];
	case (pred[3:0])
		4'h0: fp_cond = 1'b0;
		4'h1: fp_cond = z;
		4'h2: fp_cond = !(nan | z | n);
		4'h3: fp_cond = z | !(nan | n);
		4'h4: fp_cond = n & !(nan | z);
		4'h5: fp_cond = z | (n & !nan);
		4'h6: fp_cond = !(nan | z);
		4'h7: fp_cond = !nan;
		4'h8: fp_cond = nan;
		4'h9: fp_cond = nan | z;
		4'hA: fp_cond = nan | !(n | z);
		4'hB: fp_cond = nan | z | !n;
		4'hC: fp_cond = nan | (n & !z);
		4'hD: fp_cond = nan | z | n;
		4'hE: fp_cond = !z;
		default: fp_cond = 1'b1;
	endcase
endfunction

// operand window: A, B, latch, left aligned by the source format
function automatic logic [95:0] din_of(input logic [2:0] f, input logic [31:0] a,
                                       input logic [31:0] b, input logic [31:0] c);
	case (f)
		3'd4:       din_of = {a[15:0], 80'd0};
		3'd6:       din_of = {a[7:0], 88'd0};
		3'd5:       din_of = {a, b, 32'd0};
		3'd2, 3'd3: din_of = {a, b, c};
		default:    din_of = {a, 64'd0};
	endcase
endfunction

//--------------------------------------------------------------------------
// the FPU
//--------------------------------------------------------------------------
logic        f_req, f_crwe, f_iawe, f_bsun, f_fmwe, f_rst;
logic        f_fsave_ack, f_rest_idle, f_rest_unimp, f_pendcap;
logic  [2:0] f_class, f_fmt, f_srcr, f_dstr, f_fmsel;
logic  [6:0] f_opm;
logic [95:0] f_din, f_fmwd;
logic  [1:0] f_crsel;
logic [31:0] f_crwd, f_iapc;
logic        f_done, f_accepted, f_unimp, f_unsupp, f_excreq, f_used;
logic  [7:0] f_excvec, f_curvec;
logic [95:0] f_dout, f_fmrd;
logic  [3:0] f_cc;
logic [31:0] f_crrd;
logic        f_bsun_en;
logic        s_unimp, s_busy, s_wbte15, s_e1pend, s_resume;
logic [15:0] s_cmd1, s_cmd3;
logic  [2:0] s_stag, s_dtag, s_flags, s_grs;
logic [95:0] s_fpt, s_et, s_wbt;
logic [31:0] s_fpiarc;
logic [15:0] r_cmd1, r_cmd3;
logic  [2:0] r_stag, r_dtag, r_flags, r_grs;
logic [95:0] r_fpt, r_et, r_wbt;
logic [31:0] r_fpiar;
logic  [7:0] r_cusavepc;
logic        r_et15, r_fpt15, r_wbte15, r_busy;

ap040_fpu fpu (
	.clk(clk), .nreset(nreset), .ce(1'b1),
	.req(f_req), .op_class(f_class), .opmode(f_opm),
	.src_fmt(f_fmt), .src_r(f_srcr), .dst_r(f_dstr),
	.din(f_din), .done(f_done), .accepted(f_accepted),
	.unimp(f_unimp), .unsupp(f_unsupp),
	.exc_req(f_excreq), .exc_vec(f_excvec), .dout(f_dout),
	.fpcc(f_cc),
	.cr_sel(f_crsel), .cr_we(f_crwe), .cr_wdata(f_crwd), .cr_rdata(f_crrd),
	.bsun_req(f_bsun), .bsun_enable(f_bsun_en),
	.ia_we(f_iawe), .ia_wdata(f_iapc),
	.fm_sel(f_fmsel), .fm_we(f_fmwe), .fm_wdata(f_fmwd), .fm_rdata(f_fmrd),
	.fpu_used(f_used),
	.fstate_unimp(s_unimp),
	.fstate_cmd1(s_cmd1), .fstate_cmd3(s_cmd3),
	.fstate_stag(s_stag), .fstate_dtag(s_dtag), .fstate_flags(s_flags),
	.fstate_fpt(s_fpt), .fstate_et(s_et),
	.fsave_ack(f_fsave_ack), .frestore_idle(f_rest_idle), .frestore_unimp(f_rest_unimp),
	.pend_capture(f_pendcap), .cur_vec(f_curvec),
	.frestore_e1_pend(s_e1pend), .frestore_resume(s_resume),
	.frestore_cusavepc(r_cusavepc), .frestore_et15(r_et15), .frestore_fpt15(r_fpt15),
	.fstate_grs(s_grs), .fstate_wbte15(s_wbte15),
	.fstate_busy(s_busy), .fstate_wbt(s_wbt), .fstate_fpiar_c(s_fpiarc),
	.frestore_wbt(r_wbt), .frestore_fpiar(r_fpiar), .frestore_busy(r_busy),
	.frestore_cmd1(r_cmd1), .frestore_cmd3(r_cmd3),
	.frestore_stag(r_stag), .frestore_dtag(r_dtag), .frestore_flags(r_flags),
	.frestore_fpt(r_fpt), .frestore_et(r_et),
	.frestore_grs(r_grs), .frestore_wbte15(r_wbte15),
	.fp_reset(f_rst)
);

//--------------------------------------------------------------------------
// state
//--------------------------------------------------------------------------
logic [15:0] opw_q, ext_q;     // the instruction's words (latched by CHK)
logic        bg;               // a released operation is running
logic        pend;             // its enabled exception, for the next FPU instruction
logic  [7:0] pend_vec;
logic        go;               // DISP: the request is out
logic        done_q;           // DISP (store): the outcome is known
logic [31:0] eaa;              // the operand address
logic        eav;
logic        sx_v;             // a store's exception, raised by END
logic  [7:0] sx_vec;
logic  [3:0] sx_fmt;
logic [31:0] sx_addr;
logic        sx_kill;          // the stores are suppressed
logic  [7:0] list;             // FMOVEM: the register list
logic        mv_lsb, mv_rev, mv_st, mv_m1;
logic  [2:0] slot;
logic        mvr_wait;         // MVR: the raw read port's set-up cycle
logic        crr_wait;         // CRR: the control register read's cycle
logic [95:0] get_q;            // the long words GET returns
logic  [1:0] sv_kind;          // FSAVE frame: 0 NULL, 1 IDLE, 2 UNIMP, 3 BUSY
logic  [1:0] rs_kind;          // FRESTORE frame, same codes

// the words decoded: CHK's own, else the latched ones
wire [15:0] opw = (sub == FC_CHK) ? bv[15:0] : opw_q;
wire [15:0] ext = (sub == FC_CHK) ? av[15:0] : ext_q;
wire  [2:0] eam = opw[5:3];
wire  [2:0] ear = opw[2:0];
wire  [2:0] ocl = ext[15:13];
wire  [2:0] fmt = ext[12:10];
wire  [6:0] opm = ext[6:0];
wire        is_gen   = (opw[15:6] == 10'b1111_0010_00);
wire        is_cc    = (opw[15:6] == 10'b1111_0010_01);   // FScc FDBcc FTRAPcc
wire        is_bcc   = (opw[15:7] == 9'b1111_0010_1);
wire        is_fsave = (opw[15:6] == 10'b1111_0011_00);
wire        is_frest = (opw[15:6] == 10'b1111_0011_01);
wire        is_ftrap = is_cc && (eam == 3'd7) && (ear >= 3'd2) && (ear <= 3'd4);
wire        ea_dn    = (eam == 3'd0);
wire        ea_an    = (eam == 3'd1);
wire        ea_pi    = (eam == 3'd3);
wire        ea_pd    = (eam == 3'd4);
wire        ea_imm   = (eam == 3'd7) && (ear == 3'd4);
wire        ea_pcrel = (eam == 3'd7) && (ear == 3'd2 || ear == 3'd3);
wire        ea_dalt  = (eam >= 3'd2) && !(eam == 3'd7 && ear >= 3'd2);
// control registers of FMOVE(M): FPCR FPSR FPIAR; an empty list is FPIAR
wire  [2:0] crl      = (ext[12:10] == 3'd0) ? 3'b001 : ext[12:10];

//--------------------------------------------------------------------------
// CHK: the encoding faults, in the 68040's order (old core S_FPU_DEC)
//--------------------------------------------------------------------------
logic chk_x, chk_ill, chk_un, chk_ia;
always_comb begin
	logic [1:0] oc;
	chk_x = 1'b0; chk_ill = 1'b0; chk_un = 1'b0; chk_ia = 1'b0;
	oc = opmode_class(opm);
	if (is_gen) case (ocl)
		3'b000: begin
			if (oc == 2'd1) chk_x = 1'b1;
			else if (oc == 2'd2) begin chk_x = 1'b1; chk_ill = 1'b1; end
		end
		3'b001: chk_x = 1'b1;
		3'b010: begin
			if (fmt != 3'd7 && oc == 2'd1) chk_x = 1'b1;
			else if (fmt != 3'd7 && oc == 2'd2) begin chk_x = 1'b1; chk_ill = 1'b1; end
			else if (fmt == 3'd7) ;
			else if (ea_dn && fp_bytes(fmt) > 4'd4) begin
				// an unimplemented Dn source format records FPIAR first;
				// a software opmode reports through the FPSP route
				chk_x = 1'b1; chk_ia = 1'b1;
				chk_un = !(op_in_hw(opm) || fmt == 3'd3);
			end
			else if (ea_an) begin
				chk_x = 1'b1; chk_ia = 1'b1; chk_un = !op_in_hw(opm);
			end
		end
		3'b011: begin
			if (ea_dn) begin
				if (!(fmt == 3'd3 || fmt == 3'd7) && fp_bytes(fmt) > 4'd4) chk_x = 1'b1;
			end
			else if (!ea_dalt) chk_x = 1'b1;
		end
		3'b100, 3'b101: begin
			if (ea_dn && !(crl == 3'b100 || crl == 3'b010 || crl == 3'b001)) chk_x = 1'b1;
			else if (ea_an && crl != 3'b001) chk_x = 1'b1;
			else if (ext[13] && (ea_imm || ea_pcrel)) chk_x = 1'b1;
		end
		default: begin   // FMOVEM data registers
			if (eam < 3'd2 || ea_imm) chk_x = 1'b1;
			else if (ext[13] && (ea_pi || ea_pcrel)) chk_x = 1'b1;
			else if (!ext[13] && ea_pd) chk_x = 1'b1;
		end
	endcase
end

// FMOVEM slot -> list bit -> register
function automatic logic [2:0] slot_bit(input logic [2:0] k, input logic lsb);
	slot_bit = lsb ? k : (3'd7 - k);
endfunction
always_comb for (int k = 0; k < 8; k++) mv_slots[k] = list[slot_bit(3'(k), mv_lsb)];
wire [2:0] cur_bit = slot_bit(slot, mv_lsb);
wire [2:0] cur_reg = (!mv_st || mv_m1) ? (3'd7 - cur_bit) : cur_bit;  // loads: bit 7 = FP0
wire       cur_on  = list[cur_bit];
assign cr_slots = {crl[0], crl[1], crl[2]};       // bit 0 = FPCR slot
function automatic logic [1:0] crsel_of(input logic [1:0] k);   // the FPU's cr_sel
	crsel_of = (k == 2'd0) ? 2'd2 : (k == 2'd1) ? 2'd1 : 2'd0;
endfunction

// FMOVEM list and adjustment (LIST, combinational for its result)
logic [7:0] l_now;
logic [6:0] l_bytes;
always_comb begin
	logic [3:0] n;
	l_now = ext[11] ? av[7:0] : ext[7:0];
	n = 4'd0;
	for (int i = 0; i < 8; i++) n = n + {3'd0, l_now[i]};
	l_bytes = 7'(n) * 7'd12;
end

// FSAVE frame
logic [1:0] sv_now;
always_comb begin
	if (!f_used)                sv_now = 2'd0;
	else if (s_unimp && s_busy) sv_now = 2'd3;
	else if (s_unimp)           sv_now = 2'd2;
	else                        sv_now = 2'd1;
end
function automatic logic [6:0] kbytes(input logic [1:0] k);
	kbytes = (k == 2'd3) ? 7'd100 : (k == 2'd2) ? {UNIMP_WORDS, 2'b00} : 7'd4;
endfunction
assign sv_words = (sv_kind == 2'd3) ? 5'd25 : (sv_kind == 2'd2) ? UNIMP_WORDS : 5'd1;
assign rs_words = (rs_kind == 2'd3) ? 5'd25 : (rs_kind == 2'd2) ? UNIMP_WORDS : 5'd1;
function automatic logic [1:0] rs_kind_of(input logic [31:0] h);
	rs_kind_of = (h == BUSY_HDR) ? 2'd3 : (h == UNIMP_HDR) ? 2'd2 :
	             (h[31:24] == 8'd0) ? 2'd0 : 2'd1;
endfunction
wire rs_ok = (av[31:24] == 8'd0) || (av == IDLE_HDR) || (av == UNIMP_HDR) || (av == BUSY_HDR);

function automatic logic [31:0] busy_word(input logic [4:0] n);
	case (n)
		5'd0:  busy_word = BUSY_HDR;
		5'd6:  busy_word = {s_wbt[95:80], 16'd0};
		5'd7:  busy_word = s_wbt[63:32];
		5'd8:  busy_word = s_wbt[31:0];
		5'd10: busy_word = s_fpiarc;
		5'd13: busy_word = {s_cmd3, 16'd0};
		5'd15: busy_word = {s_stag, 3'd0, s_grs, 23'd0};
		5'd16: busy_word = {s_cmd1, 16'd0};
		5'd17: busy_word = {s_dtag, 8'd0, s_wbte15, 20'd0};
		5'd18: busy_word = {5'd0, s_flags[2], s_flags[1], 4'd0, s_flags[0], 20'd0};
		5'd19: busy_word = s_fpt[95:64];
		5'd20: busy_word = s_fpt[63:32];
		5'd21: busy_word = s_fpt[31:0];
		5'd22: busy_word = s_et[95:64];
		5'd23: busy_word = s_et[63:32];
		5'd24: busy_word = s_et[31:0];
		default: busy_word = 32'd0;
	endcase
endfunction
function automatic logic [31:0] unimp_word(input logic [4:0] n0);
	logic [4:0] n;
	n = (REV40 && n0 != 5'd0) ? n0 + 5'd2 : n0;   // $40 omits words 1 and 2
	case (n)
		5'd0:  unimp_word = UNIMP_HDR;
		5'd1:  unimp_word = {s_cmd3, 16'd0};
		5'd3:  unimp_word = {s_stag, 3'd0, s_grs, 23'd0};
		5'd4:  unimp_word = {s_cmd1, 16'd0};
		5'd5:  unimp_word = {s_dtag, 8'd0, s_wbte15, 20'd0};
		5'd6:  unimp_word = {5'd0, s_flags[2], s_flags[1], 4'd0, s_flags[0], 20'd0};
		5'd7:  unimp_word = s_fpt[95:64];
		5'd8:  unimp_word = s_fpt[63:32];
		5'd9:  unimp_word = s_fpt[31:0];
		5'd10: unimp_word = s_et[95:64];
		5'd11: unimp_word = s_et[63:32];
		5'd12: unimp_word = s_et[31:0];
		default: unimp_word = 32'd0;
	endcase
endfunction

//--------------------------------------------------------------------------
// the uop in EX
//--------------------------------------------------------------------------
wire chk_pend = pend && !is_fsave && !is_frest;   // FSAVE/FRESTORE: their own rules

always_comb begin
	hold    = 1'b0;
	res     = 32'd0;
	dkill   = 1'b0;
	taken   = 1'b0;
	xvec    = 8'd0;
	xfmt    = 4'd0;
	xnext   = 1'b0;
	xaddr   = pc;
	xcommit = 1'b0;
	st_kill = sx_kill;
	if (ex_v) begin
		if (!safe) hold = 1'b1;
		else case (sub)
			FC_CHK: begin
				if (bg) hold = 1'b1;
				else if (chk_pend) begin
					xvec = pend_vec;
				end
				else if (chk_x) begin
					xvec = chk_ill ? VEC_ILL : VEC_FLINE;
					if (chk_un) begin xfmt = 4'd2; xnext = 1'b1; end
				end
			end
			FC_DISP: begin
				if (ocl == 3'b011) begin
					// a store: its outcome first; its exception comes from END
					if (!done_q) hold = 1'b1;
					if (ea_dn) begin
						case (fmt)
							3'd4:    res = {bv[31:16], f_dout[95:80]};
							3'd6:    res = {bv[31:8], f_dout[95:88]};
							default: res = f_dout[95:64];
						endcase
						dkill = sx_kill;
					end
					else case (fmt)
						3'd4:    res = {16'd0, f_dout[95:80]};
						3'd6:    res = {24'd0, f_dout[95:88]};
						default: res = f_dout[95:64];
					endcase
				end
				else if (go && f_unimp) begin
					xvec = VEC_FLINE; xfmt = 4'd2; xnext = 1'b1;
					xaddr = eav ? eaa : pc; xcommit = 1'b1;
				end
				else if (go && f_unsupp) begin
					xvec = VEC_UNSUP; xfmt = 4'd3; xnext = 1'b1;
					xaddr = eav ? eaa : 32'd0; xcommit = 1'b1;
				end
				else if (go && f_excreq) begin
					xvec = f_excvec; xnext = 1'b1;
				end
				else if (!(go && (f_accepted || f_done))) hold = 1'b1;
			end
			FC_GET: res = (imm[1:0] == 2'd1) ? get_q[63:32] : get_q[31:0];
			FC_END: begin
				if (sx_v) begin
					xvec = sx_vec; xfmt = sx_fmt; xnext = 1'b1; xaddr = sx_addr;
					xcommit = 1'b1;
				end
			end
			FC_CRR: begin
				res   = f_crrd;
				dkill = !cr_slots[imm[1:0]];
				if (!crr_wait) hold = 1'b1;
			end
			FC_LIST: res = (ea_pd) ? -{25'd0, l_bytes} : {25'd0, l_bytes};
			FC_MVR: begin
				res = mv_rev ? f_fmrd[31:0] : f_fmrd[95:64];
				if (!mvr_wait) hold = 1'b1;
			end
			FC_COND, FC_DBCC: begin
				logic [5:0] pred;
				logic c;
				pred = is_bcc ? opw[5:0] : ext[5:0];
				c = fp_cond(pred, f_cc);
				if (pred[4] && f_cc[0] && f_bsun_en) begin
					xvec = VEC_BSUN;
				end
				else if (sub == FC_DBCC) begin
					res   = {bv[31:16], bv[15:0] - 16'd1};
					dkill = c;
					taken = !c && (bv[15:0] != 16'd0);
				end
				else if (is_bcc) taken = c;
				else if (is_ftrap) begin
					if (c) begin xvec = VEC_TRAPCC; xfmt = 4'd2; xnext = 1'b1; end
				end
				else res = {bv[31:8], {8{c}}};      // FScc
			end
			FC_SV0: begin
				if (bg) hold = 1'b1;
				else if (pend && !s_unimp) begin
					// frameless fallback: an earlier FSAVE already took the
					// pending exception's frame
					xvec = pend_vec;
				end
				res = ea_pd ? -{25'd0, kbytes(sv_now)} : {25'd0, kbytes(sv_now)};
			end
			FC_SVW: res = (sv_kind == 2'd3) ? busy_word(imm[6:2]) :
			              (sv_kind == 2'd2) ? unimp_word(imm[6:2]) :
			              (sv_kind == 2'd1) ? IDLE_HDR : 32'd0;
			FC_RS0: begin
				if (bg) hold = 1'b1;
				res = {25'd0, kbytes(rs_kind_of(av))};
				// an unknown frame is a format error at the FRESTORE
				if (!rs_ok) xvec = VEC_FMTERR;
			end
			default: ;
		endcase
	end
end

//--------------------------------------------------------------------------
// sequential
//--------------------------------------------------------------------------
wire act = ex_v && safe;

always_ff @(posedge clk) begin
	if (!nreset) begin
		opw_q <= '0; ext_q <= '0;
		bg <= 1'b0; pend <= 1'b0; pend_vec <= '0;
		go <= 1'b0; done_q <= 1'b0; eaa <= '0; eav <= 1'b0;
		sx_v <= 1'b0; sx_vec <= '0; sx_fmt <= '0; sx_addr <= '0; sx_kill <= 1'b0;
		list <= '0; mv_lsb <= 1'b0; mv_rev <= 1'b0; mv_st <= 1'b0; mv_m1 <= 1'b0;
		slot <= '0; mvr_wait <= 1'b0; crr_wait <= 1'b0; get_q <= '0;
		sv_kind <= '0; rs_kind <= '0;
		f_req <= 1'b0; f_crwe <= 1'b0; f_iawe <= 1'b0; f_bsun <= 1'b0; f_fmwe <= 1'b0; f_rst <= 1'b0;
		f_fsave_ack <= 1'b0; f_rest_idle <= 1'b0; f_rest_unimp <= 1'b0; f_pendcap <= 1'b0;
		f_class <= '0; f_fmt <= '0; f_srcr <= '0; f_dstr <= '0; f_fmsel <= '0; f_opm <= '0;
		f_din <= '0; f_fmwd <= '0; f_crsel <= '0; f_crwd <= '0; f_iapc <= '0;
		r_cmd1 <= '0; r_cmd3 <= '0; r_stag <= '0; r_dtag <= '0; r_flags <= '0; r_grs <= '0;
		r_fpt <= '0; r_et <= '0; r_wbt <= '0; r_fpiar <= '0; r_cusavepc <= '0;
		r_et15 <= 1'b0; r_fpt15 <= 1'b0; r_wbte15 <= 1'b0; r_busy <= 1'b0;
	end
	else begin
		f_req <= 1'b0; f_crwe <= 1'b0; f_iawe <= 1'b0; f_bsun <= 1'b0; f_fmwe <= 1'b0;
		f_rst <= 1'b0; f_fsave_ack <= 1'b0; f_rest_idle <= 1'b0; f_rest_unimp <= 1'b0;
		f_pendcap <= 1'b0;

		// a released operation: completion, or its enabled exception
		// pending for the next FPU instruction (the FPU prepares the frame)
		if (bg && f_done) bg <= 1'b0;
		if (bg && f_excreq) begin
			bg        <= 1'b0;
			pend      <= 1'b1;
			pend_vec  <= f_excvec;
			f_pendcap <= 1'b1;
		end

		if (act) case (sub)
			FC_CHK: if (adv) begin
				opw_q <= opw;
				ext_q <= ext;
				eav   <= 1'b0;
				sx_v  <= 1'b0;
				sx_kill <= 1'b0;
				go    <= 1'b0;
				done_q <= 1'b0;
				if (chk_pend) pend <= 1'b0;
				// FScc FDBcc FTRAPcc record FPIAR once decoded; so do the
				// encodings that fault after recording it
				else if ((is_cc || (chk_x && chk_ia))) begin
					f_iawe <= 1'b1;
					f_iapc <= pc;
				end
			end
			FC_EAL: begin
				// an immediate operand has no address (its frames: EA 0)
				eaa <= ea;
				eav <= !ea_imm;
			end
			FC_DISP: begin
				if (!go) begin
					go      <= 1'b1;
					f_req   <= 1'b1;
					f_iawe  <= 1'b1;
					f_iapc  <= pc;
					f_class <= ocl;
					f_opm   <= opm;
					f_fmt   <= fmt;
					f_srcr  <= (ocl == 3'b011) ? ext[9:7] : ext[12:10];
					f_dstr  <= ext[9:7];
					f_din   <= din_of(fmt, av, bv, latch);
				end
				else if (ocl == 3'b011) begin
					if (f_unimp || f_unsupp || f_done) begin
						done_q <= 1'b1;
						get_q  <= f_dout;
					end
					if (f_unimp) begin
						sx_v <= 1'b1; sx_vec <= VEC_FLINE; sx_fmt <= 4'd2;
						sx_addr <= eav ? eaa : pc; sx_kill <= 1'b1;
					end
					else if (f_unsupp) begin
						sx_v <= 1'b1; sx_vec <= VEC_UNSUP; sx_fmt <= 4'd3;
						sx_addr <= ea_dn ? 32'd0 : eaa; sx_kill <= 1'b1;
					end
					else if (f_done && f_excreq) begin
						// an enabled SNAN/OPERR integer store writes nothing;
						// otherwise the destination is written, then the trap
						sx_v <= 1'b1; sx_vec <= f_excvec; sx_fmt <= 4'd3;
						sx_addr <= ea_dn ? 32'd0 : eaa;
						sx_kill <= (f_excvec == VEC_SNAN || f_excvec == VEC_OPERR) &&
						           (fmt == 3'd0 || fmt == 3'd4 || fmt == 3'd6);
					end
				end
				else if (adv && f_accepted && !f_done && !f_excreq) bg <= 1'b1;
			end
			FC_CRR: begin
				// the FPU reads the selected register combinationally:
				// select, then take it the next cycle
				if (adv) crr_wait <= 1'b0;
				else begin
					f_crsel  <= crsel_of(imm[1:0]);
					crr_wait <= 1'b1;
				end
			end
			FC_CRW: if (adv) begin
				f_crsel <= crsel_of(immb[1:0]);
				f_crwe  <= cr_slots[immb[1:0]];
				f_crwd  <= av;
			end
			FC_LIST: if (adv) begin
				list   <= l_now;
				mv_st  <= ext[13];
				mv_m1  <= ext[12];
				mv_lsb <= ext[13] && ea_pd;
				mv_rev <= ext[13] && (ext[12] == ea_pd);
				slot   <= 3'd0;
			end
			FC_MVR: begin
				if (!mvr_wait) begin
					// the raw image comes through the source-register port
					f_srcr   <= cur_reg;
					mvr_wait <= 1'b1;
				end
				else if (adv) begin
					get_q    <= mv_rev ? {f_fmrd[31:0], f_fmrd[63:32], f_fmrd[95:64]} : f_fmrd;
					mvr_wait <= 1'b0;
					slot     <= slot + 3'd1;
				end
			end
			FC_MVW: if (adv) begin
				if (cur_on) begin
					f_fmsel <= cur_reg;
					f_fmwe  <= 1'b1;
					f_fmwd  <= {av, bv, latch};
				end
				slot <= slot + 3'd1;
			end
			FC_COND, FC_DBCC: if (adv) begin
				logic [5:0] pred;
				pred = is_bcc ? opw[5:0] : ext[5:0];
				// a signaling predicate on unordered sets BSUN
				if (pred[4] && f_cc[0]) f_bsun <= 1'b1;
			end
			FC_END: if (adv) begin
				sx_v <= 1'b0; sx_kill <= 1'b0;
				if (is_fsave) f_fsave_ack <= (sv_kind >= 2'd2);
				if (is_frest) case (rs_kind)
					2'd0: begin f_rst <= 1'b1; pend <= 1'b0; end
					2'd1: begin f_rest_idle <= 1'b1; pend <= 1'b0; end
					2'd2: begin
						// a restored arithmetic E1 frame re-arms its exception
						// for the next FPU instruction
						r_busy <= 1'b0;
						f_rest_unimp <= 1'b1;
						pend     <= s_e1pend;
						pend_vec <= f_curvec;
					end
					default: begin
						// BUSY: CU_SAVEPC $fe resumes the prepared command
						r_busy <= 1'b1;
						f_rest_unimp <= 1'b1;
						bg       <= s_resume;
						pend     <= !s_resume && s_e1pend;
						pend_vec <= f_curvec;
					end
				endcase
			end
			FC_SV0: if (adv) begin
				sv_kind <= sv_now;
				// a frame extraction consumes the pending exception; the
				// frameless fallback was taken as this uop's exception
				pend <= 1'b0;
			end
			FC_RS0: if (adv) begin
				rs_kind <= rs_kind_of(av);
				r_busy  <= (av == BUSY_HDR);
			end
			FC_RSW: if (adv) begin
				logic [4:0] k;
				k = immb[6:2];
				if (rs_kind == 2'd3) case (k)
					5'd2:  r_cusavepc <= av[31:24];
					5'd6:  r_wbt[95:64] <= av;
					5'd7:  r_wbt[63:32] <= av;
					5'd8:  r_wbt[31:0]  <= av;
					5'd10: r_fpiar <= av;
					5'd13: r_cmd3 <= av[31:16];
					5'd15: begin r_stag <= av[31:29]; r_et15 <= av[28]; r_grs <= av[25:23]; end
					5'd16: r_cmd1 <= av[31:16];
					5'd17: begin r_dtag <= av[31:29]; r_fpt15 <= av[28]; r_wbte15 <= av[20]; end
					5'd18: r_flags <= {av[26], av[25], av[20]};
					5'd19: r_fpt[95:64] <= av;
					5'd20: r_fpt[63:32] <= av;
					5'd21: r_fpt[31:0]  <= av;
					5'd22: r_et[95:64]  <= av;
					5'd23: r_et[63:32]  <= av;
					5'd24: r_et[31:0]   <= av;
					default: ;
				endcase
				else if (rs_kind == 2'd2) case (REV40 ? k + 5'd2 : k)
					5'd1:  r_cmd3 <= av[31:16];
					5'd3:  begin r_stag <= av[31:29]; r_grs <= av[25:23]; end
					5'd4:  r_cmd1 <= av[31:16];
					5'd5:  begin r_dtag <= av[31:29]; r_wbte15 <= av[20]; end
					5'd6:  r_flags <= {av[26], av[25], av[20]};
					5'd7:  r_fpt[95:64] <= av;
					5'd8:  r_fpt[63:32] <= av;
					5'd9:  r_fpt[31:0]  <= av;
					5'd10: r_et[95:64]  <= av;
					5'd11: r_et[63:32]  <= av;
					5'd12: r_et[31:0]   <= av;
					default: ;
				endcase
			end
			default: ;
		endcase
		if (!(ex_v && sub == FC_CRR)) crr_wait <= 1'b0;
		if (kill) begin
			go <= 1'b0; mvr_wait <= 1'b0; crr_wait <= 1'b0;
		end
	end
end

endmodule
