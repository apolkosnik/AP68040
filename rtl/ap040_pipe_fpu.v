//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: the FPU)          //
//                                                                          //
// ap040_pipe_fpu.v - the F-line (cpID 1) sequencer and the FPU engine      //
//                                                                          //
// rtl/ap040/ap040_fpu.v is the engine: FP0-FP7, FPCR/FPSR/FPIAR, the       //
// conversions and the arithmetic, and it is shared with the sequential     //
// core unchanged. Everything around it -- which operand, from where, in    //
// what order, and which of a dozen exception shapes a malformed or         //
// faulting instruction takes -- is ap040_core.v's S_FPU_* / S_FBCC /       //
// S_FSCC* / S_FDBCC / S_FSAVE* / S_FREST* states, transliterated here      //
// state for state and under the same names. That core passes the FPU      //
// groups of the cputest corpus; its comments carry the reasons (WinUAE,    //
// hardware), and are not repeated where the logic is the same.             //
//                                                                          //
// An F-line instruction runs alone. ap040_ea_fetch.v starts this           //
// sequencer when the instruction is in EA-fetch and EX is empty, so every  //
// older instruction has retired or is committing, and it holds the stage   //
// while the sequencer runs. What differs from the reference:               //
//                                                                          //
//   Register READS go through port A (rreg -> rdata), one cycle, as the    //
//   reference's rr_a -> rf_rdata_a did. The pipeline is drained, so the    //
//   register file's own read, with its commit bypass, is the value.        //
//                                                                          //
//   Register WRITES are two retire channels instead of rfw. w1 is the      //
//   instruction's result (FMOVE to Dn, FScc Dn, FDBcc's counter, a         //
//   control register to Dn/An) on the main port. w2 is port 2: the        //
//   (An)+/-(An) step, or -- on an exception, whose entry owns the main     //
//   port for the new SP -- whichever one register the faulting instruction //
//   still commits. The reference's writes are all at its completion or    //
//   just before its exception, so no two of one instruction collide.      //
//                                                                          //
//   Memory goes through port B, one access at a time: mrd/mwr park the     //
//   state to return to, exactly as the reference's did, and the access    //
//   states issue, wait, and return with m_val.                             //
//                                                                          //
//   Immediates and branch displacements are GATHERED by decode (imm, the   //
//   words after the command word, right-aligned), where the reference      //
//   pulled them from its prefetch queue with immf.                         //
//                                                                          //
//   (An)+/-(An) is always handled here, from An's value (S_FPU_AN's        //
//   "manual base handling"), including the FScc/FSAVE/FRESTORE forms the   //
//   reference sent through ea_start. Control modes arrive resolved as     //
//   ea_addr (ap040_ea_fetch.v's ea_target).                               //
//                                                                          //
//   The result is retire requests: fin with exc, or with w1/w2 and a       //
//   redirect (FBcc/FDBcc taken, as the reference's go_pc).                 //
//                                                                          //
// A register-destination operation is released to the background as in    //
// the reference (fpu_bg): the instruction retires, integer execution goes  //
// on, and the next F-line instruction waits for it. An enabled exception   //
// it raises is delivered, pre-instruction, by that next F-line             //
// instruction (fpu_pend_exc).                                              //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_pipe_fpu
#(
	parameter [7:0] AP040_FPU_REVISION = 8'h41
)
(
	input             clk,
	input             nreset,
	input             ce,

	// The F-line instruction in EA-fetch.
	input             start,        // begin: the instruction is here and EX is empty
	input             clear,        // it leaves EA-fetch (retired, or its exception entered)
	input       [8:0] op,           // opcode[8:0]: kind [8:6], EA mode [5:3], register [2:0]
	input      [15:0] cmd,          // the command / condition word (kinds 000 and 001)
	input      [95:0] imm,          // the words gathered after it, right-aligned
	input      [31:0] pc_i,         // this instruction's address
	input      [31:0] pc,           // the next instruction's
	input      [31:0] ea_addr,      // a control mode's resolved address
	input             sup,          // SR.S

	output reg  [3:0] rreg,         // register read (port A)
	input      [31:0] rdata,

	output            mem_rd,       // port B, one access at a time
	output            mem_wr,
	output     [31:0] mem_addr,
	output      [1:0] mem_size,
	output     [31:0] mem_wdata,
	input             mem_rd_ok,    // the read request was taken this cycle
	input             mem_wr_ok,    // the write was accepted this cycle
	input             mem_rvalid,
	input      [31:0] mem_rdata,

	output            active,       // started and not finished
	output            fin,          // finished: the outcome below stands until clear
	output reg        exc,
	output reg  [7:0] exc_vec,
	output reg  [1:0] exc_fmt,      // 0, 2 or 3
	output reg [31:0] exc_pc,
	output reg [31:0] exc_addr,
	output reg        w1_en,        // result: main port
	output reg  [3:0] w1_reg,
	output reg [31:0] w1_val,
	output reg        w2_en,        // step, or a faulting instruction's one write: port 2
	output reg  [3:0] w2_reg,
	output reg [31:0] w2_val,
	output reg        redirect,
	output reg [31:0] target,
	output            t0_flow,      // a change of flow for T0 tracing
	output            bg_busy       // a released operation is still running
);

localparam FPU_REV40 = AP040_FPU_REVISION == 8'h40;
localparam [31:0] FPU_IDLE_HEADER  = {AP040_FPU_REVISION, 24'd0};
localparam [31:0] FPU_UNIMP_HEADER = {AP040_FPU_REVISION, (FPU_REV40 ? 8'h28 : 8'h30), 16'd0};
localparam [31:0] FPU_BUSY_HEADER  = {AP040_FPU_REVISION, 8'h60, 16'd0};
localparam [31:0] FPU_UNIMP_BYTES  = FPU_REV40 ? 32'd44 : 32'd52;
localparam  [3:0] FPU_UNIMP_LAST   = FPU_REV40 ? 4'd10 : 4'd12;

localparam [5:0] S_IDLE     = 6'd0,
                 S_DISPATCH = 6'd1,
                 S_FPU_DEC  = 6'd2,
                 S_FPU_MVML = 6'd3,
                 S_FPU_AN   = 6'd4,
                 S_FPU_EA   = 6'd5,
                 S_FPU_DREG = 6'd6,
                 S_FPU_IMM  = 6'd7,
                 S_FPU_RD   = 6'd8,
                 S_FPU_GO   = 6'd9,
                 S_FPU_WR   = 6'd10,
                 S_FPU_CRD  = 6'd11,
                 S_FPU_CRI  = 6'd12,
                 S_FPU_CR2  = 6'd13,
                 S_FPU_CR   = 6'd14,
                 S_FPU_MVM  = 6'd15,
                 S_FPU_MVM2 = 6'd16,
                 S_FPU_MVM3 = 6'd17,
                 S_FBCC     = 6'd18,
                 S_FSCC0    = 6'd19,
                 S_FSCC1    = 6'd20,
                 S_FDBCC    = 6'd21,
                 S_FSAVE1   = 6'd22,
                 S_FSAVE_U  = 6'd23,
                 S_FSAVE_UD = 6'd24,
                 S_FSAVE_B  = 6'd25,
                 S_FSAVE_BD = 6'd26,
                 S_FREST1   = 6'd27,
                 S_FREST2   = 6'd28,
                 S_FREST_U  = 6'd29,
                 S_FREST_UD = 6'd30,
                 S_FREST_B  = 6'd31,
                 S_FREST_BD = 6'd32,
                 S_MRD      = 6'd33,     // mrd: request out
                 S_MRDW     = 6'd34,     // mrd: waiting for the data
                 S_MWR      = 6'd35,     // mwr: until the write is accepted
                 S_POST     = 6'd36,     // one cycle for a side-port write to land
                 S_FIN      = 6'd37,
                 S_FSCC_EA  = 6'd38,     // FScc/FSAVE/FRESTORE: -(An)/(An)+ from An
                 S_FSAVE_EA = 6'd39,
                 S_FREST_EA = 6'd40;

reg  [5:0] state, m_ret;
// Finished as S_FIN is entered (restructuring plan, phase 6B): everything
// the outcome is made of -- exc, w1/w2, the redirect -- is registered on the
// way in, and fin and a forced T0 flow were one more cycle, set in S_FIN
// itself, for nothing. The registers keep them up for as long as the state
// stands (until clear).
reg        fin_q, t0_flow_q;
assign fin     = fin_q || (state == S_FIN);
assign t0_flow = t0_flow_q || ((state == S_FIN) && t0_force);
reg [31:0] m_addr, m_wdata, m_val;
reg  [1:0] m_size;

wire [2:0] d_mode    = op[5:3];
wire [2:0] d_rn      = op[2:0];
wire       ea_is_imm = (d_mode == 3'b111) && (d_rn == 3'b100);
wire       dst_not_alt = (d_mode == 3'b001) || (d_mode == 3'b111 && d_rn > 3'b001);

// ---------------------------------------------------------------- engine
reg         fpu_req;
reg   [2:0] fpu_class;
reg   [6:0] fpu_opm;
reg   [2:0] fpu_fmt;
reg   [2:0] fpu_srcr, fpu_dstr;
reg  [95:0] fpb;
reg   [1:0] fpu_crsel;
reg         fpu_crwe;
reg  [31:0] fpu_crwd;
reg         fpu_iawe;
reg         fpu_bsun;
reg   [2:0] fpu_fmsel;
reg         fpu_fmwe;
reg  [95:0] fpu_fmwd;
reg         fpu_rst;
reg         fpu_fsave_ack;
reg         fpu_frestore_idle;
reg         fpu_frestore_unimp;
reg  [15:0] fp_restore_cmd1, fp_restore_cmd3;
reg   [2:0] fp_restore_stag, fp_restore_dtag, fp_restore_flags;
reg  [95:0] fp_restore_fpt, fp_restore_et;
reg   [7:0] fp_restore_cusavepc;
reg         fp_restore_et15, fp_restore_fpt15;
reg   [3:0] fp_nb;
reg         fp_st;
reg         fp_st_epend;
reg   [7:0] fp_st_evec;
reg   [7:0] fp_list;
reg   [1:0] fp_mode;
reg   [2:0] fp_creg;
reg   [5:0] fp_pred;
reg         fp_rev;
reg         fp_lsb;
reg   [3:0] fp_n;
reg         fp_ea_pd, fp_ea_pi;
reg         fp_ea_v;
reg   [6:0] fp_adj;
reg  [31:0] t_a;
reg         fpu_bg;
reg         fpu_pend_exc;
reg   [7:0] fpu_pend_vec;
reg         fpu_pendcap;
reg   [2:0] fp_restore_grs;
reg         fp_restore_wbte15;
reg  [95:0] fp_restore_wbt;
reg  [31:0] fp_restore_fpiar;
reg         fp_restore_busy;
reg   [4:0] fpb_n;
reg         t0_force;
reg  [31:0] an_val;     // (An)+/-(An): An as read
reg  [2:0]  imm_left;   // FMOVEM.L #imm: longwords still to consume

wire        fpu_done, fpu_unimp, fpu_unsupp, fpu_exc_req, fpu_used;
wire        fpu_accepted;
wire        fpu_fstate_unimp;
wire  [7:0] fpu_cur_vec;
wire        fpu_frestore_e1_pend;
wire        fpu_frestore_resume;
wire  [2:0] fpu_fstate_grs;
wire        fpu_fstate_wbte15;
wire        fpu_fstate_busy;
wire [95:0] fpu_fstate_wbt;
wire [31:0] fpu_fstate_fpiar_c;
wire        fpu_bsun_en;
wire  [7:0] fpu_exc_vec;
wire [95:0] fpu_dout;
wire  [3:0] fpu_cc;
wire [31:0] fpu_crrd;
wire [95:0] fpu_fmrd;
wire [15:0] fpu_fstate_cmd1, fpu_fstate_cmd3;
wire  [2:0] fpu_fstate_stag, fpu_fstate_dtag, fpu_fstate_flags;
wire [95:0] fpu_fstate_fpt, fpu_fstate_et;

ap040_fpu fpu
(
	.clk(clk), .nreset(nreset), .ce(ce),
	.req(fpu_req), .op_class(fpu_class), .opmode(fpu_opm),
	.src_fmt(fpu_fmt), .src_r(fpu_srcr), .dst_r(fpu_dstr),
	.din(fpb), .done(fpu_done), .accepted(fpu_accepted),
	.unimp(fpu_unimp), .unsupp(fpu_unsupp),
	.exc_req(fpu_exc_req), .exc_vec(fpu_exc_vec), .dout(fpu_dout),
	.fpcc(fpu_cc),
	.cr_sel(fpu_crsel), .cr_we(fpu_crwe), .cr_wdata(fpu_crwd),
	.cr_rdata(fpu_crrd),
	.bsun_req(fpu_bsun), .bsun_enable(fpu_bsun_en),
	.ia_we(fpu_iawe), .ia_wdata(pc_i),
	.fm_sel(fpu_fmsel), .fm_we(fpu_fmwe), .fm_wdata(fpu_fmwd),
	.fm_rdata(fpu_fmrd),
	.fpu_used(fpu_used),
	.fstate_unimp(fpu_fstate_unimp),
	.fstate_cmd1(fpu_fstate_cmd1), .fstate_cmd3(fpu_fstate_cmd3),
	.fstate_stag(fpu_fstate_stag), .fstate_dtag(fpu_fstate_dtag),
	.fstate_flags(fpu_fstate_flags),
	.fstate_fpt(fpu_fstate_fpt), .fstate_et(fpu_fstate_et),
	.pend_capture(fpu_pendcap), .cur_vec(fpu_cur_vec),
	.frestore_e1_pend(fpu_frestore_e1_pend),
	.frestore_resume(fpu_frestore_resume),
	.frestore_cusavepc(fp_restore_cusavepc),
	.frestore_et15(fp_restore_et15), .frestore_fpt15(fp_restore_fpt15),
	.fstate_grs(fpu_fstate_grs), .fstate_wbte15(fpu_fstate_wbte15),
	.frestore_grs(fp_restore_grs), .frestore_wbte15(fp_restore_wbte15),
	.fstate_busy(fpu_fstate_busy), .fstate_wbt(fpu_fstate_wbt),
	.fstate_fpiar_c(fpu_fstate_fpiar_c),
	.frestore_wbt(fp_restore_wbt), .frestore_fpiar(fp_restore_fpiar),
	.frestore_busy(fp_restore_busy),
	.fsave_ack(fpu_fsave_ack), .frestore_idle(fpu_frestore_idle),
	.frestore_unimp(fpu_frestore_unimp),
	.frestore_cmd1(fp_restore_cmd1), .frestore_cmd3(fp_restore_cmd3),
	.frestore_stag(fp_restore_stag), .frestore_dtag(fp_restore_dtag),
	.frestore_flags(fp_restore_flags),
	.frestore_fpt(fp_restore_fpt), .frestore_et(fp_restore_et),
	.fp_reset(fpu_rst)
);

assign bg_busy   = fpu_bg || fpu_pendcap;
assign active    = (state != S_IDLE) && !fin;
assign mem_rd    = (state == S_MRD);
assign mem_wr    = (state == S_MWR);
assign mem_addr  = m_addr;
assign mem_size  = m_size;
assign mem_wdata = m_wdata;

// ---------------------------------------------------------------- helpers
function [3:0] fp_bytes;
	input [2:0] fmt;
	begin
		case (fmt)
			3'd0, 3'd1: fp_bytes = 4;
			3'd4:       fp_bytes = 2;
			3'd6:       fp_bytes = 1;
			3'd5:       fp_bytes = 8;
			default:    fp_bytes = 12;
		endcase
	end
endfunction

function [31:0] fsave_busy_word;
	input [4:0] n;
	begin
		case (n)
			5'd0:  fsave_busy_word = FPU_BUSY_HEADER;
			5'd6:  fsave_busy_word = {fpu_fstate_wbt[95:80], 16'd0};
			5'd7:  fsave_busy_word = fpu_fstate_wbt[63:32];
			5'd8:  fsave_busy_word = fpu_fstate_wbt[31:0];
			5'd10: fsave_busy_word = fpu_fstate_fpiar_c;
			5'd13: fsave_busy_word = {fpu_fstate_cmd3, 16'd0};
			5'd15: fsave_busy_word = {fpu_fstate_stag, 3'd0, fpu_fstate_grs, 23'd0};
			5'd16: fsave_busy_word = {fpu_fstate_cmd1, 16'd0};
			5'd17: fsave_busy_word = {fpu_fstate_dtag, 8'd0, fpu_fstate_wbte15, 20'd0};
			5'd18: fsave_busy_word = {5'd0, fpu_fstate_flags[2], fpu_fstate_flags[1], 4'd0,
			                          fpu_fstate_flags[0], 20'd0};
			5'd19: fsave_busy_word = fpu_fstate_fpt[95:64];
			5'd20: fsave_busy_word = fpu_fstate_fpt[63:32];
			5'd21: fsave_busy_word = fpu_fstate_fpt[31:0];
			5'd22: fsave_busy_word = fpu_fstate_et[95:64];
			5'd23: fsave_busy_word = fpu_fstate_et[63:32];
			5'd24: fsave_busy_word = fpu_fstate_et[31:0];
			default: fsave_busy_word = 32'd0;
		endcase
	end
endfunction

function [31:0] fsave_unimp_word;
	input [3:0] n;
	begin
		case ((FPU_REV40 && n != 0) ? n + 4'd2 : n)
			4'd0:  fsave_unimp_word = FPU_UNIMP_HEADER;
			4'd1:  fsave_unimp_word = {fpu_fstate_cmd3, 16'd0};
			4'd2:  fsave_unimp_word = 32'd0;
			4'd3:  fsave_unimp_word = {fpu_fstate_stag, 3'd0, fpu_fstate_grs, 23'd0};
			4'd4:  fsave_unimp_word = {fpu_fstate_cmd1, 16'd0};
			4'd5:  fsave_unimp_word = {fpu_fstate_dtag, 8'd0, fpu_fstate_wbte15, 20'd0};
			4'd6:  fsave_unimp_word = {5'd0, fpu_fstate_flags[2], fpu_fstate_flags[1], 4'd0,
			                           fpu_fstate_flags[0], 20'd0};
			4'd7:  fsave_unimp_word = fpu_fstate_fpt[95:64];
			4'd8:  fsave_unimp_word = fpu_fstate_fpt[63:32];
			4'd9:  fsave_unimp_word = fpu_fstate_fpt[31:0];
			4'd10: fsave_unimp_word = fpu_fstate_et[95:64];
			4'd11: fsave_unimp_word = fpu_fstate_et[63:32];
			default: fsave_unimp_word = fpu_fstate_et[31:0];
		endcase
	end
endfunction

function fp_cond;
	input [5:0] pred;
	input [3:0] cc;
	reg n, z, nan;
	begin
		n = cc[3]; z = cc[2]; nan = cc[0];
		case (pred[3:0])
			4'h0: fp_cond = 0;
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
			default: fp_cond = 1;
		endcase
	end
endfunction

function fp_op_in_hw;
	input [6:0] o;
	begin
		case (o)
			7'h00, 7'h40, 7'h44, 7'h18, 7'h58, 7'h5C, 7'h1A, 7'h5A, 7'h5E,
			7'h38, 7'h3A, 7'h22, 7'h62, 7'h66, 7'h28, 7'h68, 7'h6C,
			7'h23, 7'h27, 7'h63, 7'h67, 7'h20, 7'h24, 7'h60, 7'h64,
			7'h04, 7'h41, 7'h45:
				fp_op_in_hw = 1;
			default:
				fp_op_in_hw = 0;
		endcase
	end
endfunction

function [1:0] fp_opmode_class;
	input [6:0] o;
	begin
		case (o)
			7'h05, 7'h07, 7'h0B, 7'h13, 7'h17, 7'h1B,
			7'h29, 7'h2A, 7'h2B, 7'h2C, 7'h2D, 7'h2E, 7'h2F,
			7'h39, 7'h3B, 7'h3C, 7'h3D, 7'h3E, 7'h3F,
			7'h42, 7'h43, 7'h46, 7'h47,
			7'h48, 7'h49, 7'h4A, 7'h4B, 7'h4C, 7'h4D, 7'h4E, 7'h4F,
			7'h50, 7'h51, 7'h52, 7'h53, 7'h54, 7'h55, 7'h56, 7'h57,
			7'h59, 7'h5B, 7'h5D, 7'h5F,
			7'h61, 7'h65, 7'h69, 7'h6A, 7'h6B, 7'h6D, 7'h6E, 7'h6F,
			7'h70, 7'h71, 7'h72, 7'h73, 7'h74, 7'h75, 7'h76, 7'h77:
				fp_opmode_class = 2'd1;
			7'h78, 7'h79, 7'h7A, 7'h7B, 7'h7C, 7'h7D, 7'h7E, 7'h7F:
				fp_opmode_class = 2'd2;
			default:
				fp_opmode_class = 2'd0;
		endcase
	end
endfunction

function [31:0] sxw;
	input [15:0] v;
	begin sxw = {{16{v[15]}}, v}; end
endfunction

// The An step of an (An)+/-(An) byte access: A7 keeps the stack even.
function [6:0] an_step_b;
	input [3:0] nb;
	input [2:0] rn;
	begin an_step_b = (nb == 4'd1 && rn == 3'd7) ? 7'd2 : {3'b000, nb}; end
endfunction

// The immediate's k-th longword of n, first first: decode right-aligns.
function [31:0] imm_long;
	input [95:0] v;
	input  [2:0] n;
	input  [2:0] k;
	reg    [2:0] idx;
	begin
		idx = n - 3'd1 - k;
		case (idx)
			3'd0:    imm_long = v[31:0];
			3'd1:    imm_long = v[63:32];
			default: imm_long = v[95:64];
		endcase
	end
endfunction

// ---------------------------------------------------------------- the FSM
// The reference's tasks, as blocks that set the next state.
task mrd;
	input [31:0] a;
	input  [1:0] sz;
	input  [5:0] ret;
	begin
		m_addr <= a; m_size <= sz; m_ret <= ret; state <= S_MRD;
	end
endtask

task mwr;
	input [31:0] a;
	input  [1:0] sz;
	input [31:0] d;
	input  [5:0] ret;
	begin
		m_addr <= a; m_size <= sz; m_wdata <= d; m_ret <= ret; state <= S_MWR;
	end
endtask

task go_exc;
	input  [7:0] vec;
	input  [1:0] fmt;
	input [31:0] spc;
	input [31:0] addr;
	begin
		exc <= 1'b1; exc_vec <= vec; exc_fmt <= fmt; exc_pc <= spc; exc_addr <= addr;
		fpu_req <= 1'b0;
		state <= S_FIN;
	end
endtask

task fetch_next;
	begin state <= S_FIN; end
endtask

task go_pc;
	input [31:0] t;
	begin redirect <= 1'b1; target <= t; t0_flow_q <= 1'b1; state <= S_FIN; end
endtask

// The (An)+/-(An) step, committed with the outcome.
task an_step;
	input [31:0] v;
	begin w2_en <= 1'b1; w2_reg <= {1'b1, d_rn}; w2_val <= v; end
endtask

task go_fp_fline;
	begin go_exc(`AP040_VEC_FLINE, 2'd0, pc_i, 32'd0); end
endtask

task go_fp_ea_fault;
	input hw;
	begin
		go_exc(`AP040_VEC_FLINE, hw ? 2'd0 : 2'd2, hw ? pc_i : pc,
		       hw ? 32'd0 : (fp_ea_v ? t_a : pc_i));
	end
endtask

task go_fp_unimp;
	begin go_exc(`AP040_VEC_FLINE, 2'd2, pc, fp_ea_v ? t_a : pc_i); end
endtask

task go_fp_unsupp;
	input        post;
	input        has_ea;
	input [31:0] ea;
	begin
		if (post) go_exc(`AP040_VEC_FP_UNSUP, 2'd3, pc, ea);
		else      go_exc(`AP040_VEC_FP_UNSUP, 2'd3, pc, has_ea ? ea : 32'd0);
	end
endtask

always @(posedge clk) begin
	if (!nreset) begin
		state <= S_IDLE; m_ret <= S_IDLE;
		m_addr <= 32'd0; m_wdata <= 32'd0; m_val <= 32'd0; m_size <= `AP040_SZ_L;
		fin_q <= 1'b0; exc <= 1'b0; exc_vec <= 8'd0; exc_fmt <= 2'd0;
		exc_pc <= 32'd0; exc_addr <= 32'd0;
		w1_en <= 1'b0; w1_reg <= 4'd0; w1_val <= 32'd0;
		w2_en <= 1'b0; w2_reg <= 4'd0; w2_val <= 32'd0;
		redirect <= 1'b0; target <= 32'd0; t0_flow_q <= 1'b0; t0_force <= 1'b0;
		rreg <= 4'd0;
		fpu_req <= 1'b0; fpu_class <= 3'd0; fpu_opm <= 7'd0; fpu_fmt <= 3'd0;
		fpu_srcr <= 3'd0; fpu_dstr <= 3'd0; fpb <= 96'd0;
		fpu_crsel <= 2'd0; fpu_crwe <= 1'b0; fpu_crwd <= 32'd0; fpu_iawe <= 1'b0;
		fpu_bsun <= 1'b0; fpu_fmsel <= 3'd0; fpu_fmwe <= 1'b0; fpu_fmwd <= 96'd0;
		fpu_rst <= 1'b0; fpu_fsave_ack <= 1'b0; fpu_frestore_idle <= 1'b0;
		fpu_frestore_unimp <= 1'b0;
		fp_restore_cmd1 <= 16'd0; fp_restore_cmd3 <= 16'd0;
		fp_restore_stag <= 3'd0; fp_restore_dtag <= 3'd0; fp_restore_flags <= 3'd0;
		fp_restore_fpt <= 96'd0; fp_restore_et <= 96'd0; fp_restore_cusavepc <= 8'd0;
		fp_restore_et15 <= 1'b0; fp_restore_fpt15 <= 1'b0;
		fp_nb <= 4'd0; fp_st <= 1'b0; fp_st_epend <= 1'b0; fp_st_evec <= 8'd0;
		fp_list <= 8'd0; fp_mode <= 2'd0; fp_creg <= 3'd0; fp_pred <= 6'd0;
		fp_rev <= 1'b0; fp_lsb <= 1'b0; fp_n <= 4'd0;
		fp_ea_pd <= 1'b0; fp_ea_pi <= 1'b0; fp_ea_v <= 1'b0; fp_adj <= 7'd0; t_a <= 32'd0;
		fpu_bg <= 1'b0; fpu_pend_exc <= 1'b0; fpu_pend_vec <= 8'd0; fpu_pendcap <= 1'b0;
		fp_restore_grs <= 3'd0; fp_restore_wbte15 <= 1'b0; fp_restore_wbt <= 96'd0;
		fp_restore_fpiar <= 32'd0; fp_restore_busy <= 1'b0; fpb_n <= 5'd0;
		an_val <= 32'd0; imm_left <= 3'd0;
	end else if (ce) begin
		// One-cycle side ports, as the reference.
		fpu_req <= 1'b0; fpu_crwe <= 1'b0; fpu_fmwe <= 1'b0; fpu_iawe <= 1'b0;
		fpu_bsun <= 1'b0;
		fpu_pendcap <= 1'b0;
		if (fpu_bg && fpu_done) fpu_bg <= 1'b0;
		if (fpu_bg && fpu_exc_req) begin
			fpu_bg       <= 1'b0;
			fpu_pend_exc <= 1'b1;
			fpu_pend_vec <= fpu_exc_vec;
			fpu_pendcap  <= 1'b1;
		end
		fpu_rst <= 1'b0;
		fpu_fsave_ack <= 1'b0;
		fpu_frestore_idle <= 1'b0;
		fpu_frestore_unimp <= 1'b0;

		if (clear) begin
			state <= S_IDLE;
			fin_q <= 1'b0;
		end else case (state)
		S_IDLE: if (start) begin
			exc <= 1'b0; w1_en <= 1'b0; w2_en <= 1'b0; redirect <= 1'b0;
			t0_flow_q <= 1'b0; t0_force <= 1'b0;
			fp_ea_v <= 1'b0; fp_ea_pd <= 1'b0; fp_ea_pi <= 1'b0;
			rreg <= {1'b1, d_rn};   // An, for the modes that step it
			// A general operation whose EA is not An's own -- (An), (An)+,
			// -(An), which read an_val -- needs nothing DISPATCH captures, and
			// DISPATCH would only send it to S_FPU_DEC (phase 6B): straight
			// there. The F-line forms go through DISPATCH as ever.
			if (op[8:6] == 3'b000 && d_mode != 3'b010 && d_mode != 3'b011 && d_mode != 3'b100 &&
			    !(d_mode == 3'b111 && d_rn > 3'b100))
				state <= S_FPU_DEC;
			else
				state <= S_DISPATCH;
		end

		// ap040_core.v's F-line decode for cpID 1 (its S_FETCH case
		// ir[11:8] == 2 / 3), the part not already in ap040_decode.v.
		S_DISPATCH: begin
			an_val <= rdata;
			case (op[8:6])
			3'b000: begin
				if (d_mode == 3'b111 && d_rn > 3'b100) go_fp_fline;
				else state <= S_FPU_DEC;
			end
			3'b001: begin
				if (d_mode == 3'b111 && d_rn > 3'b100) go_fp_fline;
				else state <= S_FSCC0;
			end
			3'b010, 3'b011: state <= S_FBCC;
			3'b100: begin
				// FSAVE: control alterable or -(An); malformed EAs are
				// F-line faults, classified before privilege.
				if (d_mode < 3'b010 || d_mode == 3'b011 ||
				    (d_mode == 3'b111 && d_rn > 3'b001)) go_fp_fline;
				else if (!sup) go_exc(`AP040_VEC_PRIV, 2'd0, pc_i, 32'd0);
				else begin
					t0_force <= 1'b1;   // t0_special: FSAVE/FRESTORE
					state <= S_FSAVE_EA;
				end
			end
			3'b101: begin
				// FRESTORE: control, (An)+ or PC relative
				if (d_mode < 3'b010 || d_mode == 3'b100 ||
				    (d_mode == 3'b111 && d_rn >= 3'b100)) go_fp_fline;
				else if (!sup) go_exc(`AP040_VEC_PRIV, 2'd0, pc_i, 32'd0);
				else begin
					t0_force <= 1'b1;
					state <= S_FREST_EA;
				end
			end
			default: go_exc(`AP040_VEC_FLINE, 2'd0, pc_i, 32'd0);
			endcase
		end

		// FSAVE/FRESTORE resolve their EA as ea_start(.., SZ_L, ..) did:
		// -(An) and (An)+ step by four at once, and the frame paths below
		// override the step with the frame's size.
		S_FSAVE_EA: begin
			if (d_mode == 3'b100) begin
				t_a <= an_val - 32'd4;
				an_step(an_val - 32'd4);
			end else t_a <= (d_mode == 3'b010) ? an_val : ea_addr;
			state <= S_FSAVE1;
		end
		S_FREST_EA: begin
			if (d_mode == 3'b011) begin
				t_a <= an_val;
				an_step(an_val + 32'd4);
			end else t_a <= (d_mode == 3'b010) ? an_val : ea_addr;
			state <= S_FREST1;
		end

		//----------------------------------------------------- FSAVE/FRESTORE
		S_FSAVE1: begin
			if (fpu_bg || fpu_pendcap) state <= S_FSAVE1;
			else if (fpu_pend_exc && !fpu_fstate_unimp) begin
				// the -(An) step already stands, as ea_start committed it
				fpu_pend_exc <= 1'b0;
				go_exc(fpu_pend_vec, 2'd0, pc_i, pc_i);
			end
			else if (fpu_fstate_unimp && fpu_fstate_busy) begin
				fpu_pend_exc <= 1'b0;
				if (d_mode == 3'b100) begin
					t_a <= t_a - 32'd96;
					an_step(t_a - 32'd96);
				end
				fpb_n <= 5'd0;
				state <= S_FSAVE_B;
			end
			else if (fpu_fstate_unimp) begin
				fpu_pend_exc <= 1'b0;
				if (d_mode == 3'b100) begin
					t_a <= t_a - (FPU_UNIMP_BYTES - 32'd4);
					an_step(t_a - (FPU_UNIMP_BYTES - 32'd4));
				end
				fp_n <= 4'd0;
				state <= S_FSAVE_U;
			end
			else mwr(t_a, `AP040_SZ_L, fpu_used ? FPU_IDLE_HEADER : 32'h0000_0000, S_FIN);
		end

		S_FSAVE_U:
			mwr(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L, fsave_unimp_word(fp_n), S_FSAVE_UD);

		S_FSAVE_UD: begin
			if (fp_n == FPU_UNIMP_LAST) begin
				fpu_fsave_ack <= 1'b1;
				fetch_next;
			end else begin
				fp_n <= fp_n + 4'd1;
				state <= S_FSAVE_U;
			end
		end

		S_FSAVE_B:
			mwr(t_a + {25'd0, fpb_n, 2'b00}, `AP040_SZ_L, fsave_busy_word(fpb_n), S_FSAVE_BD);

		S_FSAVE_BD: begin
			if (fpb_n == 5'd24) begin
				fpu_fsave_ack <= 1'b1;
				fetch_next;
			end else begin
				fpb_n <= fpb_n + 5'd1;
				state <= S_FSAVE_B;
			end
		end

		S_FREST1:
			if (fpu_bg) state <= S_FREST1;
			else mrd(t_a, `AP040_SZ_L, S_FREST2);

		S_FREST2: begin
			if (m_val[31:24] == 8'd0) begin
				fpu_rst <= 1'b1;
				fpu_pend_exc <= 1'b0;
				fetch_next;
			end
			else if (m_val == FPU_IDLE_HEADER) begin
				fpu_frestore_idle <= 1'b1;
				fpu_pend_exc <= 1'b0;
				fetch_next;
			end
			else if (m_val == FPU_UNIMP_HEADER) begin
				fp_restore_busy <= 1'b0;
				if (FPU_REV40) fp_restore_cmd3 <= 16'd0;
				fp_n <= 4'd1;
				mrd(t_a + 32'd4, `AP040_SZ_L, S_FREST_U);
			end
			else if (m_val == FPU_BUSY_HEADER) begin
				fp_restore_busy <= 1'b1;
				fpb_n <= 5'd1;
				mrd(t_a + 32'd4, `AP040_SZ_L, S_FREST_B);
			end
			else go_exc(`AP040_VEC_FMTERR, 2'd0, pc_i, 32'd0);   // (An)+'s step stands
		end

		S_FREST_U: begin
			case (FPU_REV40 ? fp_n + 4'd2 : fp_n)
				4'd1: fp_restore_cmd3 <= m_val[31:16];
				4'd3: begin
					fp_restore_stag <= m_val[31:29];
					fp_restore_grs  <= m_val[25:23];
				end
				4'd4: fp_restore_cmd1 <= m_val[31:16];
				4'd5: begin
					fp_restore_dtag   <= m_val[31:29];
					fp_restore_wbte15 <= m_val[20];
				end
				4'd6: fp_restore_flags <= {m_val[26], m_val[25], m_val[20]};
				4'd7: fp_restore_fpt[95:64] <= m_val;
				4'd8: fp_restore_fpt[63:32] <= m_val;
				4'd9: fp_restore_fpt[31:0] <= m_val;
				4'd10: fp_restore_et[95:64] <= m_val;
				4'd11: fp_restore_et[63:32] <= m_val;
				4'd12: fp_restore_et[31:0] <= m_val;
				default: ;
			endcase
			if (fp_n == FPU_UNIMP_LAST) state <= S_FREST_UD;
			else begin
				fp_n <= fp_n + 4'd1;
				mrd(t_a + ({28'd0, fp_n} << 2) + 32'd4, `AP040_SZ_L, S_FREST_U);
			end
		end

		S_FREST_B: begin
			case (fpb_n)
				5'd2:  fp_restore_cusavepc <= m_val[31:24];
				5'd6:  fp_restore_wbt[95:64] <= m_val;
				5'd7:  fp_restore_wbt[63:32] <= m_val;
				5'd8:  fp_restore_wbt[31:0]  <= m_val;
				5'd10: fp_restore_fpiar <= m_val;
				5'd13: fp_restore_cmd3 <= m_val[31:16];
				5'd15: begin
					fp_restore_stag <= m_val[31:29];
					fp_restore_et15 <= m_val[28];
					fp_restore_grs  <= m_val[25:23];
				end
				5'd16: fp_restore_cmd1 <= m_val[31:16];
				5'd17: begin
					fp_restore_dtag   <= m_val[31:29];
					fp_restore_fpt15  <= m_val[28];
					fp_restore_wbte15 <= m_val[20];
				end
				5'd18: fp_restore_flags <= {m_val[26], m_val[25], m_val[20]};
				5'd19: fp_restore_fpt[95:64] <= m_val;
				5'd20: fp_restore_fpt[63:32] <= m_val;
				5'd21: fp_restore_fpt[31:0]  <= m_val;
				5'd22: fp_restore_et[95:64]  <= m_val;
				5'd23: fp_restore_et[63:32]  <= m_val;
				5'd24: fp_restore_et[31:0]   <= m_val;
				default: ;
			endcase
			if (fpb_n == 5'd24) state <= S_FREST_BD;
			else begin
				fpb_n <= fpb_n + 5'd1;
				mrd(t_a + ({27'd0, fpb_n} << 2) + 32'd4, `AP040_SZ_L, S_FREST_B);
			end
		end

		S_FREST_BD: begin
			if (d_mode == 3'b011) an_step(t_a + 32'd100);
			fpu_frestore_unimp <= 1'b1;
			fpu_bg       <= fpu_frestore_resume;
			fpu_pend_exc <= !fpu_frestore_resume && fpu_frestore_e1_pend;
			fpu_pend_vec <= fpu_cur_vec;
			fetch_next;
		end

		S_FREST_UD: begin
			fp_restore_busy <= 1'b0;
			if (d_mode == 3'b011) an_step(t_a + FPU_UNIMP_BYTES);
			fpu_frestore_unimp <= 1'b1;
			fpu_pend_exc <= fpu_frestore_e1_pend;
			fpu_pend_vec <= fpu_cur_vec;
			fetch_next;
		end

		//------------------------------------------------------------ general
		S_FPU_DEC: begin
			if (fpu_bg) begin
				// hold the dispatch until the background operation retires
			end
			else if (fpu_pend_exc) begin
				fpu_pend_exc <= 1'b0;
				go_exc(fpu_pend_vec, 2'd0, pc_i, pc_i);
			end
			else begin
			fpu_class <= cmd[15:13];
			fpu_opm   <= cmd[6:0];
			fpu_fmt   <= cmd[12:10];
			fpu_srcr  <= cmd[12:10];
			fpu_dstr  <= cmd[9:7];
			fp_nb     <= fp_bytes(cmd[12:10]);
			fp_st     <= 1'b0;
			fp_st_epend <= 1'b0;
			fp_n      <= 4'd0;
			fp_ea_pd  <= 1'b0;
			fp_ea_v   <= 1'b0;
			fp_ea_pi  <= 1'b0;
			case (cmd[15:13])
				3'b000: begin
					if (fp_opmode_class(cmd[6:0]) == 2'd1) go_fp_fline;
					else if (fp_opmode_class(cmd[6:0]) == 2'd2)
						go_exc(`AP040_VEC_ILLEGAL, 2'd0, pc_i, 32'd0);
					else begin
						fpu_iawe <= 1'b1;
						fpu_req <= 1'b1;
						state <= S_FPU_GO;
					end
				end
				3'b001: go_fp_fline;
				3'b010: begin
					if (cmd[12:10] != 3'd7 && fp_opmode_class(cmd[6:0]) == 2'd1) go_fp_fline;
					else if (cmd[12:10] != 3'd7 && fp_opmode_class(cmd[6:0]) == 2'd2)
						go_exc(`AP040_VEC_ILLEGAL, 2'd0, pc_i, 32'd0);
					else if (cmd[12:10] == 3'd7) begin
						fpu_iawe <= 1'b1;
						fpu_req <= 1'b1;
						state <= S_FPU_GO;
					end
					else if (d_mode == 3'b000) begin
						if (fp_bytes(cmd[12:10]) > 4'd4) begin
							fpu_iawe <= 1'b1;
							go_fp_ea_fault(fp_op_in_hw(cmd[6:0]) || cmd[12:10] == 3'd3);
							state <= S_POST;
						end
						else begin
							rreg <= {1'b0, d_rn};
							fpu_iawe <= 1'b1;
							state <= S_FPU_DREG;
						end
					end
					else if (d_mode == 3'b001) begin
						fpu_iawe <= 1'b1;
						go_fp_ea_fault(fp_op_in_hw(cmd[6:0]));
						state <= S_POST;
					end
					else if (ea_is_imm) begin
						fpb <= 96'd0;
						state <= S_FPU_IMM;
					end
					else if (d_mode == 3'b011 || d_mode == 3'b100) state <= S_FPU_AN;
					else state <= S_FPU_EA;
				end
				3'b011: begin
					fpu_srcr <= cmd[9:7];
					fp_st <= 1'b1;
					if (d_mode == 3'b000) begin
						if (cmd[12:10] == 3'd3 || cmd[12:10] == 3'd7) begin
							fpu_iawe <= 1'b1;
							fpu_req <= 1'b1;
							state <= S_FPU_GO;
						end
						else if (fp_bytes(cmd[12:10]) > 4'd4) begin
							go_fp_fline;
							state <= S_POST;
						end
						else begin
							rreg <= {1'b0, d_rn};
							fpu_iawe <= 1'b1;
							fpu_req <= 1'b1;
							state <= S_FPU_GO;
						end
					end
					else if (d_mode == 3'b001) begin
						go_fp_fline;
						state <= S_POST;
					end
					else if (dst_not_alt || (d_mode == 3'b111 && d_rn[1])) begin
						go_fp_fline;
						state <= S_POST;
					end
					else if (d_mode == 3'b011 || d_mode == 3'b100) state <= S_FPU_AN;
					else state <= S_FPU_EA;
				end
				3'b100, 3'b101: begin : fp_crm
					reg [6:0] cnt;
					reg [2:0] crsel;
					reg       multi;
					crsel = (cmd[12:10] == 3'd0) ? 3'b001 : cmd[12:10];
					multi = (crsel != 3'b100) && (crsel != 3'b010) && (crsel != 3'b001);
					cnt = ({6'd0, crsel[2]} + {6'd0, crsel[1]} + {6'd0, crsel[0]}) << 2;
					fp_creg <= crsel;
					fp_st <= cmd[13];
					if (cmd[13]) t0_force <= 1'b1;
					fp_nb <= cnt[3:0];
					fp_adj <= cnt;
					if (d_mode == 3'b000) begin
						if (multi) go_fp_fline;
						else begin
							rreg <= {1'b0, d_rn};
							state <= S_FPU_CRD;
						end
					end
					else if (d_mode == 3'b001) begin
						if (crsel != 3'b001) go_fp_fline;
						else begin
							rreg <= {1'b1, d_rn};
							state <= S_FPU_CRD;
						end
					end
					else if (ea_is_imm) begin
						if (cmd[13]) go_fp_fline;
						else begin
							imm_left <= {2'd0, crsel[2]} + {2'd0, crsel[1]} + {2'd0, crsel[0]};
							fp_n <= 4'd0;
							state <= S_FPU_CRI;
						end
					end
					else if (cmd[13] && d_mode == 3'b111 && d_rn[1]) go_fp_fline;
					else if (d_mode == 3'b011 || d_mode == 3'b100) state <= S_FPU_AN;
					else state <= S_FPU_EA;
				end
				default: begin : fp_mvm
					reg [6:0] cnt;
					reg is_st;
					cnt = ({6'd0, cmd[7]} + {6'd0, cmd[6]} + {6'd0, cmd[5]} +
					       {6'd0, cmd[4]} + {6'd0, cmd[3]} + {6'd0, cmd[2]} +
					       {6'd0, cmd[1]} + {6'd0, cmd[0]}) * 7'd12;
					is_st = (cmd[15:13] == 3'b111);
					fp_mode <= cmd[12:11];
					fp_st <= is_st;
					if (is_st) t0_force <= 1'b1;
					fp_list <= cmd[7:0];
					fp_adj <= cnt;
					fp_lsb <= is_st && (d_mode == 3'b100);
					fp_rev <= is_st && (cmd[12] == (d_mode == 3'b100));
					if (d_mode < 3'b010 || ea_is_imm) go_fp_fline;
					else if (is_st && (d_mode == 3'b011 || (d_mode == 3'b111 && d_rn[1]))) go_fp_fline;
					else if (!is_st && d_mode == 3'b100) go_fp_fline;
					else if (cmd[11]) begin
						rreg <= {1'b0, cmd[6:4]};
						state <= S_FPU_MVML;
					end
					else if (d_mode == 3'b011 || d_mode == 3'b100) state <= S_FPU_AN;
					else state <= S_FPU_EA;
				end
			endcase
			end
		end

		S_FPU_MVML: begin : fp_mvml
			reg [6:0] cnt;
			cnt = ({6'd0, rdata[7]} + {6'd0, rdata[6]} + {6'd0, rdata[5]} + {6'd0, rdata[4]} +
			       {6'd0, rdata[3]} + {6'd0, rdata[2]} + {6'd0, rdata[1]} + {6'd0, rdata[0]}) * 7'd12;
			fp_list <= rdata[7:0];
			fp_adj <= cnt;
			if (d_mode == 3'b011 || d_mode == 3'b100) state <= S_FPU_AN;
			else if (d_mode < 3'b010 || ea_is_imm) go_fp_fline;
			else state <= S_FPU_EA;
		end

		S_FPU_AN: begin : fp_an
			reg [6:0] adj;
			adj = an_step_b(fp_nb, d_rn);
			if (fpu_class[2]) adj = fp_adj;   // FMOVEM: the whole list
			fp_adj   <= adj;
			fp_ea_v  <= 1'b1;
			fp_ea_pd <= (d_mode == 3'b100);
			fp_ea_pi <= (d_mode == 3'b011);
			t_a <= (d_mode == 3'b100) ? (an_val - {25'd0, adj}) : an_val;
			case (fpu_class)
				3'b010: state <= S_FPU_RD;
				3'b011: begin
					fpu_iawe <= 1'b1;
					fpu_req <= 1'b1;
					state <= S_FPU_GO;
				end
				3'b100, 3'b101: state <= S_FPU_CR;
				default: state <= S_FPU_MVM;
			endcase
		end

		S_FPU_EA: begin
			t_a <= (d_mode == 3'b010) ? an_val : ea_addr;   // (An): An itself
			fp_ea_v <= 1'b1;
			case (fpu_class)
				3'b010: state <= S_FPU_RD;
				3'b011: begin
					fpu_iawe <= 1'b1;
					fpu_req <= 1'b1;
					state <= S_FPU_GO;
				end
				3'b100, 3'b101: state <= S_FPU_CR;
				default: state <= S_FPU_MVM;
			endcase
		end

		S_FPU_DREG: begin
			case (fpu_fmt)
				3'd4: fpb <= {rdata[15:0], 80'd0};
				3'd6: fpb <= {rdata[7:0], 88'd0};
				default: fpb <= {rdata, 64'd0};
			endcase
			fpu_iawe <= 1'b1;
			fpu_req <= 1'b1;
			state <= S_FPU_GO;
		end

		// The immediate, gathered by decode and right-aligned: the same
		// left-aligned operand S_FPU_IMM assembled from the queue.
		S_FPU_IMM: begin
			case (fp_nb)
				4'd1:  fpb <= {imm[7:0], 88'd0};
				4'd2:  fpb <= {imm[15:0], 80'd0};
				4'd4:  fpb <= {imm[31:0], 64'd0};
				4'd8:  fpb <= {imm[63:0], 32'd0};
				default: fpb <= imm;
			endcase
			fpu_iawe <= 1'b1;
			fpu_req <= 1'b1;
			state <= S_FPU_GO;
		end

		S_FPU_RD: begin
			if (fp_n != 4'd0) begin
				if (fp_nb == 4'd1) fpb[95:88] <= m_val[7:0];
				else if (fp_nb == 4'd2) fpb[95:80] <= m_val[15:0];
				else case (fp_n)
					4'd1: fpb[95:64] <= m_val;
					4'd2: fpb[63:32] <= m_val;
					default: fpb[31:0] <= m_val;
				endcase
			end
			if ((fp_nb <= 4'd4 && fp_n != 4'd0) ||
			    (fp_nb == 4'd8 && fp_n == 4'd2) ||
			    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
				fpu_iawe <= 1'b1;
				fpu_req <= 1'b1;
				state <= S_FPU_GO;
			end
			else begin
				if (fp_nb == 4'd1)
					mrd(t_a, `AP040_SZ_B, S_FPU_RD);
				else if (fp_nb == 4'd2)
					mrd(t_a, `AP040_SZ_W, S_FPU_RD);
				else
					mrd(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L, S_FPU_RD);
				fp_n <= fp_n + 4'd1;
			end
		end

		S_FPU_GO: begin
			if (fpu_unimp) begin
				if (fp_ea_pd) an_step(t_a);
				else if (fp_ea_pi) an_step(t_a + {25'd0, fp_adj});
				go_fp_unimp;
			end
			else if (fpu_unsupp) begin
				if (fp_ea_pd) an_step(t_a);
				else if (fp_ea_pi) an_step(t_a + {25'd0, fp_adj});
				go_fp_unsupp(fp_st, fp_ea_v, fp_ea_v ? t_a : 32'd0);
			end
			else if (fpu_exc_req && !fp_st) begin
				go_exc(fpu_exc_vec, 2'd0, pc, pc_i);
			end
			else if (fpu_accepted && !fp_st) begin
				fpu_bg <= 1'b1;
				if (fp_ea_pd) an_step(t_a);
				else if (fp_ea_pi) an_step(t_a + {25'd0, fp_adj});
				fetch_next;
			end
			else if (fpu_done) begin
				if (!fp_st) begin
					if (fp_ea_pd) an_step(t_a);
					else if (fp_ea_pi) an_step(t_a + {25'd0, fp_adj});
					fetch_next;
				end
				else if (fpu_exc_req &&
				         (fpu_exc_vec == `AP040_VEC_FP_SNAN || fpu_exc_vec == `AP040_VEC_FP_OPERR) &&
				         (fpu_fmt == 3'd0 || fpu_fmt == 3'd4 || fpu_fmt == 3'd6)) begin
					if (fp_ea_pd) an_step(t_a);
					else if (fp_ea_pi) an_step(t_a + {25'd0, fp_adj});
					go_exc(fpu_exc_vec, 2'd3, pc, (d_mode == 3'b000) ? 32'd0 : t_a);
				end
				else if (d_mode == 3'b000) begin : fp_stdn
					reg [31:0] v;
					case (fpu_fmt)
						3'd4: v = {rdata[31:16], fpu_dout[95:80]};
						3'd6: v = {rdata[31:8], fpu_dout[95:88]};
						default: v = fpu_dout[95:64];
					endcase
					if (fpu_exc_req) begin
						// written, then the trap: port 2, the entry owns the main one
						w2_en <= 1'b1; w2_reg <= {1'b0, d_rn}; w2_val <= v;
						go_exc(fpu_exc_vec, 2'd3, pc, 32'd0);
					end
					else begin
						w1_en <= 1'b1; w1_reg <= {1'b0, d_rn}; w1_val <= v;
						fetch_next;
					end
				end
				else begin
					fp_n <= 4'd0;
					fp_st_epend <= fpu_exc_req;
					fp_st_evec  <= fpu_exc_vec;
					state <= S_FPU_WR;
				end
			end
		end

		S_FPU_WR: begin
			if ((fp_nb <= 4'd4 && fp_n != 4'd0) ||
			    (fp_nb == 4'd8 && fp_n == 4'd2) ||
			    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
				if (fp_ea_pd) an_step(t_a);
				else if (fp_ea_pi) an_step(t_a + {25'd0, fp_adj});
				if (fp_st_epend) begin
					fp_st_epend <= 1'b0;
					go_exc(fp_st_evec, 2'd3, pc, t_a);
				end
				else fetch_next;
			end
			else begin
				if (fp_nb == 4'd1)
					mwr(t_a, `AP040_SZ_B, {24'd0, fpu_dout[95:88]}, S_FPU_WR);
				else if (fp_nb == 4'd2)
					mwr(t_a, `AP040_SZ_W, {16'd0, fpu_dout[95:80]}, S_FPU_WR);
				else begin : fp_wrl
					reg [31:0] wv;
					case (fp_n)
						4'd0: wv = fpu_dout[95:64];
						4'd1: wv = fpu_dout[63:32];
						default: wv = fpu_dout[31:0];
					endcase
					mwr(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L, wv, S_FPU_WR);
				end
				fp_n <= fp_n + 4'd1;
			end
		end

		S_FPU_CRD: begin
			fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
			if (!fp_st) begin
				fpu_crwe <= 1'b1;
				fpu_crwd <= rdata;
				fetch_next;
			end
			else state <= S_FPU_CR2;
		end

		// FMOVEM.L #imm,<control list>: one longword per register, FPCR
		// first, from the gathered immediate.
		S_FPU_CRI: begin : fp_cri
			reg [2:0] rest;
			rest = fp_creg[2] ? {1'b0, fp_creg[1:0]} :
			       fp_creg[1] ? {fp_creg[2], 1'b0, fp_creg[0]} : 3'b000;
			fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
			fpu_crwe <= 1'b1;
			fpu_crwd <= imm_long(imm, imm_left, fp_n[2:0]);
			fp_creg <= rest;
			fp_n <= fp_n + 4'd1;
			if (rest == 3'd0) fetch_next;
		end

		S_FPU_CR2: begin
			if (d_mode == 3'b000 || d_mode == 3'b001) begin
				w1_en <= 1'b1; w1_reg <= {d_mode[0], d_rn}; w1_val <= fpu_crrd;
				fetch_next;
			end
			else mwr(t_a, `AP040_SZ_L, fpu_crrd, S_FPU_CR);
		end

		S_FPU_CR: begin : fp_cr
			if (fp_n[0]) begin
				if (!fp_st) begin
					fpu_crwe <= 1'b1;
					fpu_crwd <= m_val;
				end
				t_a <= t_a + 32'd4;
				fp_n <= 4'd0;
			end
			else if (fp_creg == 3'd0) begin
				if (fp_ea_pd) an_step(t_a - {25'd0, fp_adj});
				else if (fp_ea_pi) an_step(t_a);
				fetch_next;
			end
			else begin
				fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
				fp_creg <= fp_creg[2] ? {1'b0, fp_creg[1:0]} :
				           fp_creg[1] ? {fp_creg[2], 1'b0, fp_creg[0]} :
				                        {fp_creg[2:1], 1'b0};
				fp_n <= 4'd1;
				if (fp_st) state <= S_FPU_CR2;
				else mrd(t_a, `AP040_SZ_L, S_FPU_CR);
			end
		end

		S_FPU_MVM: begin : fp_mvm_sel
			reg [2:0] b;
			reg found;
			integer j;
			found = 0; b = 0;
			for (j = 7; j >= 0; j = j - 1)
				if (!found && fp_list[fp_lsb ? (3'd7 - j[2:0]) : j[2:0]]) begin
					b = fp_lsb ? (3'd7 - j[2:0]) : j[2:0];
					found = 1;
				end
			if (!found) begin
				if (fp_ea_pd) an_step(t_a - {25'd0, fp_adj});
				else if (fp_ea_pi) an_step(t_a);
				fetch_next;
			end
			else begin
				fp_list <= fp_list & ~(8'd1 << b);
				fpu_fmsel <= (!fp_st || fp_mode[1]) ? (3'd7 - b) : b;
				if (fp_st) fpu_srcr <= (!fp_st || fp_mode[1]) ? (3'd7 - b) : b;
				fp_n <= 4'd0;
				state <= S_FPU_MVM2;
			end
		end

		S_FPU_MVM2: begin
			if (fp_n == 4'd3) begin
				if (!fp_st) begin
					fpu_fmwe <= 1'b1;
					fpu_fmwd <= fpb;
				end
				t_a <= t_a + 32'd12;
				state <= S_FPU_MVM;
			end
			else begin : fp_mvm_x
				reg [31:0] wv;
				case (fp_rev ? (4'd2 - fp_n) : fp_n)
					4'd0: wv = fpu_fmrd[95:64];
					4'd1: wv = fpu_fmrd[63:32];
					default: wv = fpu_fmrd[31:0];
				endcase
				if (fp_st)
					mwr(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L, wv, S_FPU_MVM3);
				else
					mrd(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L, S_FPU_MVM3);
				fp_n <= fp_n + 4'd1;
			end
		end

		S_FPU_MVM3: begin
			if (!fp_st) case (fp_n)
				4'd1: fpb[95:64] <= m_val;
				4'd2: fpb[63:32] <= m_val;
				default: fpb[31:0] <= m_val;
			endcase
			state <= S_FPU_MVM2;
		end

		//--------------------------------------------- FBcc / FScc / FDBcc
		S_FBCC: begin
			if (fpu_bg) state <= S_FBCC;
			else if (fpu_pend_exc) begin
				fpu_pend_exc <= 1'b0;
				go_exc(fpu_pend_vec, 2'd0, pc_i, pc_i);
			end
			else begin : fbcc_run
				reg [31:0] disp;
				disp = op[6] ? imm[31:0] : sxw(imm[15:0]);
				if (op[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1'b1;
					go_exc(`AP040_VEC_FP_BSUN, 2'd0, pc_i, 32'd0);
					state <= S_POST;
				end
				else begin
					if (op[4] && fpu_cc[0]) fpu_bsun <= 1'b1;
					if (fp_cond(op[5:0], fpu_cc)) go_pc(pc_i + 32'd2 + disp);
					else fetch_next;
				end
			end
		end

		S_FSCC0: begin
			if (fpu_bg) state <= S_FSCC0;
			else if (fpu_pend_exc) begin
				fpu_pend_exc <= 1'b0;
				go_exc(fpu_pend_vec, 2'd0, pc_i, pc_i);
			end
			else begin
				fp_pred <= cmd[5:0];
				fpu_iawe <= 1'b1;
				if (d_mode == 3'b001) begin
					t0_force <= 1'b1;
					rreg <= {1'b0, d_rn};
					state <= S_FDBCC;
				end
				else if (d_mode == 3'b111 && (d_rn == 3'b010 || d_rn == 3'b011 || d_rn == 3'b100))
					state <= S_FSCC1;
				else if (d_mode == 3'b000) begin
					rreg <= {1'b0, d_rn};
					state <= S_FSCC1;
				end
				else if (ea_is_imm || (d_mode == 3'b111 && d_rn[1])) go_fp_fline;
				else state <= S_FSCC_EA;
			end
		end

		// ea_start(.., SZ_B, ..): the step is one, or two through A7.
		S_FSCC_EA: begin
			if (d_mode == 3'b100) begin
				t_a <= an_val - {25'd0, an_step_b(4'd1, d_rn)};
				an_step(an_val - {25'd0, an_step_b(4'd1, d_rn)});
			end else if (d_mode == 3'b011) begin
				t_a <= an_val;
				an_step(an_val + {25'd0, an_step_b(4'd1, d_rn)});
			end else t_a <= (d_mode == 3'b010) ? an_val : ea_addr;
			state <= S_FSCC1;
		end

		S_FSCC1: begin : fscc1
			reg c;
			c = fp_cond(fp_pred, fpu_cc);
			if (fp_pred[4] && fpu_cc[0] && fpu_bsun_en) begin
				fpu_bsun <= 1'b1;
				go_exc(`AP040_VEC_FP_BSUN, 2'd0, pc_i, 32'd0);
				state <= S_POST;
			end
			else begin
				if (fp_pred[4] && fpu_cc[0]) fpu_bsun <= 1'b1;
				if (d_mode == 3'b111 && (d_rn == 3'b010 || d_rn == 3'b011 || d_rn == 3'b100)) begin
					if (c) begin
						go_exc(`AP040_VEC_TRAPCC, 2'd2, pc, pc_i);
						if (fp_pred[4] && fpu_cc[0]) state <= S_POST;
					end
					else fetch_next;
				end
				else if (d_mode == 3'b000) begin
					w1_en <= 1'b1; w1_reg <= {1'b0, d_rn}; w1_val <= {rdata[31:8], {8{c}}};
					fetch_next;
				end
				else mwr(t_a, `AP040_SZ_B, {24'd0, {8{c}}}, S_FIN);
			end
		end

		S_FDBCC: begin : fdbcc
			reg [15:0] cnt;
			if (fp_pred[4] && fpu_cc[0] && fpu_bsun_en) begin
				fpu_bsun <= 1'b1;
				go_exc(`AP040_VEC_FP_BSUN, 2'd0, pc_i, 32'd0);
				state <= S_POST;
			end
			else begin
				if (fp_pred[4] && fpu_cc[0]) fpu_bsun <= 1'b1;
				if (fp_cond(fp_pred, fpu_cc)) fetch_next;
				else begin
					cnt = rdata[15:0] - 16'd1;
					w1_en <= 1'b1; w1_reg <= {1'b0, d_rn}; w1_val <= {rdata[31:16], cnt};
					if (cnt != 16'hFFFF) go_pc(pc_i + 32'd4 + sxw(imm[15:0]));
					else fetch_next;
				end
			end
		end

		//------------------------------------------------------ port B access
		S_MRD:  if (mem_rd_ok) state <= S_MRDW;
		S_MRDW: if (mem_rvalid) begin
			m_val <= (m_size == `AP040_SZ_B) ? {24'd0, mem_rdata[7:0]} :
			         (m_size == `AP040_SZ_W) ? {16'd0, mem_rdata[15:0]} : mem_rdata;
			state <= m_ret;
		end
		S_MWR:  if (mem_wr_ok) state <= m_ret;

		// One cycle for a side-port write (FPIAR, BSUN in FPSR) to land
		// before the outcome is taken.
		S_POST: state <= S_FIN;

		S_FIN: begin
			fin_q <= 1'b1;
			if (t0_force) t0_flow_q <= 1'b1;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
