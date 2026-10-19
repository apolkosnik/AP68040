//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_fpu.v - on-chip FPU (milestone H)                                  //
//                                                                          //
// Programming model: FP0-FP7 in extended precision (96-bit memory image,   //
// 80 significant bits), FPCR (mode + exception enables), FPSR (condition   //
// codes, quotient, exception and accrued bytes), FPIAR.                    //
//                                                                          //
// Implemented hardware subset: data movement and B/W/L/S/D/X conversion,  //
// FABS/FNEG/FTST/FCMP, and iterative FADD/FSUB/FMUL/FDIV/FSQRT with FPCR   //
// precision/rounding, FPSR status/accrual, signaling NaNs and enabled      //
// arithmetic-exception vectors. Single/double stores generate gradual      //
// underflow; unsupported denormal/unnormal inputs take vector 55.          //
//                                                                          //
// Transcendentals, FMOVECR and the remaining software subset assert        //
// `unimp`, following the 68040 FPSP route; packed decimal takes the        //
// unsupported-data-type path.  Revision-$41                               //
// UNIMP state is retained for FSAVE/FRESTORE; a true BUSY arithmetic       //
// exception frame remains a documented integration gap.                   //
//                                                                          //
// The core owns instruction decode, effective addresses and memory         //
// transfers; operands arrive left-aligned in a 96-bit window. FMOVEM       //
// traffic uses the raw register port (fm_*), architecturally exact bits    //
// with no condition code or rounding side effects.                         //
//                                                                          //
// All state advances only when ce is high (clkena discipline).             //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_fpu
#(
	// the arithmetic pipeline (FMOVE/FADD/FSUB/FMUL on ordinary operands);
	// 0: every operation through the sequential state machine
	parameter PIPE = 1
)
(
	input             clk,
	input             nreset,
	input             ce,

	// command port: pulse req with the fields valid; done pulses on
	// completion, unimp pulses instead when the op is not in hardware
	input             req,
	input       [2:0] op_class,    // extension word [15:13]
	input       [6:0] opmode,      // extension word [6:0]
	input       [2:0] src_fmt,     // extension word [12:10] (opclass 2/3)
	input       [2:0] src_r,       // FPm
	input       [2:0] dst_r,       // FPn
	input      [95:0] din,         // memory operand, left aligned
	input             din_nice,    // din is an ordinary operand for the
	                               // pipeline (nice_mem: the interface forms
	                               // it as it registers din)
	output reg        done,
	// the operation has passed operand classification: unimp/unsupp can no
	// longer occur, only completion or an enabled arithmetic exception.
	// The core uses this to run register-destination arithmetic in the
	// background while integer execution continues.
	output            accepted,
	output            wb_ok,       // F_WB this cycle, no enabled exception
	output reg        unimp,       // unimplemented instruction -> vector 11
	output reg        unsupp,      // unsupported data type -> vector 55
	output reg        exc_req,       // enabled arithmetic exception
	output reg [7:0]  exc_vec,
	output reg [95:0] dout,        // store result, left aligned

	output      [3:0] fpcc,        // FPSR condition codes {N,Z,I,NAN}

	// control registers (FMOVE/FMOVEM of FPCR/FPSR/FPIAR run in the core)
	input       [1:0] cr_sel,      // 0 FPIAR, 1 FPSR, 2 FPCR
	input             cr_we,
	input      [31:0] cr_wdata,
	output     [31:0] cr_rdata,
	input             bsun_req,     // signaling conditional on unordered
	output            bsun_enable,

	// FPIAR update on dispatch of an FP instruction (core provides PC)
	input             ia_we,
	input      [31:0] ia_wdata,

	// raw register port for FMOVEM (X format image, no side effects)
	input       [2:0] fm_sel,
	input             fm_we,
	input      [95:0] fm_wdata,
	output     [95:0] fm_rdata,

	// FSAVE/FRESTORE state.  An unimplemented instruction leaves the
	// revision-$41 68040 state-frame payload here until a successful FSAVE
	// acknowledges it.  FRESTORE can reinstate that pending state.
	output reg        fpu_used,    // 0: NULL frame, 1: IDLE or exception frame
	output            pdone,       // the request went into the pipeline this
	                               // cycle (done, combinationally; done itself
	                               // pulses only for the state machine)
	output      [7:0] okv_next,    // the register-ready flags next cycle, and
	input             sel_x,       // those of src_r/dst_r selected outside
	input             aok_x,       // into flip-flops, from the registers'
	input             bok_x,       // next values (else selected here)
	input       [6:0] opmode_n,    // with sel_x: opmode, op_class and src_fmt
	input       [2:0] op_class_n,  // as they will be next cycle (the static
	input       [2:0] src_fmt_n,   // pipeline eligibility is registered)
	output            pbusy,       // pipelined operations in flight: FPU state
	                               // (registers, FPSR, condition codes) is not
	                               // current until they complete
	output reg        fstate_unimp,
	output reg [15:0] fstate_cmd1,
	output reg [15:0] fstate_cmd3,
	output reg  [2:0] fstate_stag,
	output reg  [2:0] fstate_dtag,
	output reg  [2:0] fstate_flags, // {E1,E3,T}
	output reg [95:0] fstate_fpt,
	output reg [95:0] fstate_et,
	input             fsave_ack,
	input             frestore_idle,
	input             frestore_unimp,
	// Pending-exception frame support (FSAVE e1 tier): the core pulses
	// pend_capture when a released op's enabled exception is deferred;
	// the e1-class frame is then prepared from the operand shadow so a
	// later FSAVE emits it instead of trapping.  cur_vec is the
	// priority vector over the live FPSR/FPCR enables;
	// frestore_e1_pend classifies a frame being restored as a
	// re-armable arithmetic E1 state.
	input             pend_capture,
	output      [7:0] cur_vec,
	output            frestore_e1_pend,
	// FPSP requests execution of the normalized BUSY command with CU_SAVEPC=fe.
	input       [7:0] frestore_cusavepc,
	input             frestore_et15, frestore_fpt15,
	output            frestore_resume,
	output reg  [2:0] fstate_grs,
	output reg        fstate_wbte15,
	output reg        fstate_busy,      // the pending frame is $41/$60 BUSY
	output reg [95:0] fstate_wbt,       // WBTEMP: the internal intermediate
	output reg [31:0] fstate_fpiar_c,   // FPIARCU
	input             frestore_busy,
	// FRESTORE loads the frame's words into the frame registers above as
	// it reads them (MC68000PRM: the frame is loaded from the format word
	// on through higher addresses); frestore_unimp then acts on them
	input             frestore_we,
	input       [3:0] frestore_wf,      // the field: see the write below
	input      [31:0] frestore_wd,
	input             fp_reset     // FRESTORE of a NULL frame
);

//---------------------------------------------------------------------------
// architectural state
//---------------------------------------------------------------------------

// FP registers as the 80 significant bits: {sign, exp[14:0], man[63:0]}.
// The two command-time read views are mirrored inside a small MLAB regfile.
wire [79:0] fr_src_raw;
wire [79:0] fr_dst_raw;
reg   [7:0] fr_valid;
// each register's unsupported data type (denormal/unnormal: the dispatch's
// datatype check), kept beside it as it is written: the check reads a flag,
// not a classification of the register file's read data
reg   [7:0] fr_unsup;
// Simulation/debug mirrors retain the long-standing hierarchical names used
// by the AP68040 benches.  The live datapath does not read them, so synthesis
// removes these arrays and their update logic.
reg        fr_s [0:7];
reg [14:0] fr_e [0:7];
reg [63:0] fr_m [0:7];

// Reset and FRESTORE NULL invalidate the payload instead of distributing a
// reset across 640 data bits.  Only these two command-time read views remain;
// FMOVEM stores share the source view and all later FPn uses are shadowed.
wire        fr_src_s = fr_valid[src_r] ? fr_src_raw[79]    : 1'b0;
wire [14:0] fr_src_e = fr_valid[src_r] ? fr_src_raw[78:64] : 15'h7FFF;
wire [63:0] fr_src_m = fr_valid[src_r] ? fr_src_raw[63:0]  : 64'hFFFF_FFFF_FFFF_FFFF;
wire        fr_dst_s = fr_valid[dst_r] ? fr_dst_raw[79]    : 1'b0;
wire [14:0] fr_dst_e = fr_valid[dst_r] ? fr_dst_raw[78:64] : 15'h7FFF;
wire [63:0] fr_dst_m = fr_valid[dst_r] ? fr_dst_raw[63:0]  : 64'hFFFF_FFFF_FFFF_FFFF;
wire        fr_src_unsup = fr_valid[src_r] && fr_unsup[src_r];

reg [31:0] fpcr;                  // [15:8] enables, [7:6] prec, [5:4] rnd
reg [31:0] fpsr;                  // [27:24] cc, [23:16] quot, [15:8] exc, [7:3] aexc
reg [31:0] fpiar;

assign fpcc = fpsr[27:24];
        // {N, Z, I, NAN}

wire [1:0] rnd_mode = fpcr[5:4];  // 00 RN, 01 RZ, 10 RM, 11 RP

assign cr_rdata = (cr_sel == 2'd0) ? fpiar :
                  (cr_sel == 2'd1) ? fpsr : fpcr;
assign bsun_enable = fpcr[15];

// Raw FMOVEM image: X format memory layout {s, e, 16'b0, m}.  Stores reuse
// the normal source-register read port; the core selects src_r one cycle
// before consuming fm_rdata.  fm_sel remains the independent restore port.
assign fm_rdata = {fr_src_s, fr_src_e, 16'd0, fr_src_m};

//---------------------------------------------------------------------------
// unpacked working format
//   value = (-1)^s * man * 2^(exp-16383), man[63] = integer bit
//   tag: 00 number (normalized here), 01 zero, 10 infinity, 11 NaN
//---------------------------------------------------------------------------

localparam T_NUM  = 2'd0;
localparam T_ZERO = 2'd1;
localparam T_INF  = 2'd2;
localparam T_NAN  = 2'd3;

// {s, e[16:0], m[63:0], t[1:0]} = 84 bits
// the opmodes F_EXEC sends to F_BIN (its ex_bin), and their op_kind
function bin_op;
	input [6:0] op;
	case (op)
		7'h22, 7'h62, 7'h66, 7'h28, 7'h68, 7'h6C, 7'h23, 7'h63, 7'h67, 7'h27,
		7'h20, 7'h60, 7'h64, 7'h24: bin_op = 1'b1;
		default: bin_op = 1'b0;
	endcase
endfunction
function [3:0] bin_kind;
	input [6:0] op;
	bin_kind = (op == 7'h23 || op == 7'h27 || op == 7'h63 || op == 7'h67) ? 4'd2 :
	           (op == 7'h20 || op == 7'h24 || op == 7'h60 || op == 7'h64) ? 4'd3 : 4'd1;
endfunction

function [83:0] unpack_x;
	input        s;
	input [14:0] e;
	input [63:0] m;
	reg    [1:0] t;
	begin
		if (e == 15'h7FFF) t = (m[62:0] != 63'd0) ? T_NAN : T_INF;
		else if (m == 64'd0) t = T_ZERO;
		else t = T_NUM;   // unnormals arrive here and are normalized below
		unpack_x = {s, {2'd0, e}, m, t};
	end
endfunction

// Only TRUE denormals (zero exponent, integer bit clear, nonzero
// fraction) and true unnormals (nonzero finite exponent, integer bit
// clear, nonzero mantissa) take the unimplemented-data-type route.
// PSEUDO-DENORMALS -- zero exponent with the integer bit SET -- are
// legal operands the hardware computes with directly, using the RAW
// exponent field (floatx80_is_denormal requires the integer bit clear;
// hardware cputest FDIV/FDADD with $0000-8xxx operands executes).
// Unnormal ZEROS (nonzero exponent, mantissa zero) behave as zeros.
function unsupported_x;
	input [14:0] e;
	input [63:0] m;
	begin
		unsupported_x = ((e == 15'd0) && !m[63] && (m[62:0] != 0)) ||
		                ((e != 15'd0) && (e != 15'h7FFF) && !m[63] &&
		                 (m != 64'd0));
	end
endfunction

function is_snan_x;
	input [14:0] e;
	input [63:0] m;
	begin
		is_snan_x = (e == 15'h7FFF) && (m[62:0] != 63'd0) && !m[62];
	end
endfunction

// leading zeros of a 64-bit value (64 when zero): a tree, pairs merged
// level by level (the upper half's count, or the lower's with the upper
// half's size added when the upper half is empty)
function [6:0] clz64;
	input [63:0] v;
	reg        val [0:63];
	reg  [5:0] cnt [0:63];
	integer    k, i;
	begin
		for (i = 0; i < 64; i = i + 1) begin val[i] = v[i]; cnt[i] = 6'd0; end
		for (k = 1; k <= 6; k = k + 1)
			for (i = 0; i < (64 >> k); i = i + 1) begin
				cnt[i] = val[2*i+1] ? cnt[2*i+1] : (cnt[2*i] | (6'd1 << (k - 1)));
				val[i] = val[2*i+1] | val[2*i];
			end
		clz64 = val[0] ? {1'b0, cnt[0]} : 7'd64;
	end
endfunction

// 68040 state-frame tags (Table 9-15).  Register operands retain their raw
// extended image in the frame even when it is an unnormal value.
function [2:0] frame_tag_x;
	input [14:0] e;
	input [63:0] m;
	begin
		if (m == 64'd0 && e != 15'h7FFF) frame_tag_x = 3'd1;       // zero
		else if (unsupported_x(e, m))    frame_tag_x = 3'd4;       // X denorm/unnormal
		else if (e == 15'h7FFF && m[62:0] != 0) frame_tag_x = 3'd3; // NaN
		else if (e == 15'h7FFF)          frame_tag_x = 3'd2;       // infinity
		else                              frame_tag_x = 3'd0;       // normal
	end
endfunction

//---------------------------------------------------------------------------
// FSM
//---------------------------------------------------------------------------

localparam F_IDLE  = 4'd0;
localparam F_SRC   = 4'd1;   // classify/convert the memory operand
localparam F_NORM  = 4'd2;   // normalize integer or denormal sources
localparam F_EXEC  = 4'd3;   // move/abs/neg/tst/cmp/arith dispatch
localparam F_WB    = 4'd4;   // register writeback + condition codes
localparam F_SHR   = 4'd5;   // staged right shift for integer stores
localparam F_PACKI = 4'd6;   // integer store rounding and assembly
localparam F_BIN   = 4'd7;   // binary op specials and setup
localparam F_ADDX  = 4'd9;   // mantissa add/subtract
localparam F_MULT  = 4'd10;  // multiply loop
localparam F_DIVL  = 4'd11;  // divide loop
localparam F_SQRTL = 4'd12;  // square root loop
localparam F_NORM2 = 4'd13;  // post-operation normalize
localparam F_ROUND = 4'd14;  // precision rounding and range checks

localparam F_PACKS = 4'd15;  // denormal single/double store packing
localparam F_UNFL  = 5'd16;  // gradual underflow at single/double precision
localparam F_STDONE = 5'd17; // store completion: settled-status trap check
localparam F_RESTORE_A = 5'd18; // finish ET15 denormalization, shift FPTEMP
localparam F_RESTORE_B = 5'd19; // finish FPTE15 denormalization
localparam F_RESTORE_N = 5'd20; // normalize the prepared destination
localparam F_CAP   = 5'd21;  // a datatype or unimplemented-instruction frame captured
localparam F_NLZ   = 5'd22;  // the normalizer's count registered, before its shift

reg  [4:0] fst;
// unimp/unsupp decisions are made in the dispatch cycle (register
// sources, still F_IDLE), during source conversion (F_SRC), and by the
// destination-operand checks in F_EXEC and F_BIN.  Only the arithmetic
// and rounding states are strictly past every such decision: from here
// on nothing but done or an enabled-exception exc_req can follow.
// (the alignment shift of an add runs after every datatype check: the
// operation is accepted as it starts)
// fst == F_WB, kept the same way (wb_ok and the core's hold start from a
// flip-flop, not from a decode of the state register)
reg  wb_r;
assign wb_ok = wb_r && !(|(fpsr[15:8] & fpcr[15:8]));
// synthesis translate_off
always @(posedge clk)
	if (nreset && wb_r !== (fst == F_WB)) begin
		$display("FAIL: FPU wb_r %b, the state %0d", wb_r, fst);
		$finish;
	end
// synthesis translate_on
// accepted: a register kept with every state assignment (acc_r: the state
// it goes to is one of these), so the core's holds start from a flip-flop
// and not from a decode of the state register; checked in simulation
wire acc_dec = (fst == F_ADDX) || (fst == F_MULT) ||
               (fst == F_SHR && sh_ret == F_ADDX) ||
               (fst == F_DIVL) || (fst == F_SQRTL) ||
               (fst == F_NORM2) || (fst == F_NLZ && nz_ret == F_NORM2) || (fst == F_ROUND);
reg  acc_r;
function acc_of;
	input [4:0] st;
	begin
		acc_of = (st == F_ADDX) || (st == F_MULT) || (st == F_DIVL) ||
		         (st == F_SQRTL) || (st == F_NORM2) || (st == F_ROUND);
	end
endfunction
assign accepted = acc_r;
// synthesis translate_off
always @(posedge clk)
	if (nreset && acc_r !== acc_dec) begin
		$display("FAIL: FPU accepted register %b, the state's %b (fst %0d)", acc_r, acc_dec, fst);
		$finish;
	end
// synthesis translate_on

reg  [2:0] r_fmt, r_dst;
reg        r_ae7;           // accrued-IOP before this instruction (fault backout)
reg        r_unimp;         // memory-source software op using normal converter
reg        r_resume;        // normalized BUSY operands bypass datatype retraps
reg  [2:0] r_stag;          // source tag retained while that conversion runs
reg  [6:0] r_op;
reg [95:0] r_din;
// the memory operand's significand in the working format, by r_fmt: one
// bus (and one negator, of the integer left-aligned: the same bits as a
// byte or word negated in place) for every unpack of r_din into a_m.  A
// single or double's integer bit is its exponent's: set for a normal
// number, clear for zero, a denormal, infinity and a NaN
wire [31:0] unp_i  = (r_fmt == 3'd0) ? r_din[95:64] :
                     (r_fmt == 3'd4) ? {r_din[95:80], 16'd0} : {r_din[95:88], 24'd0};
wire [31:0] unp_im = r_din[95] ? (32'd0 - unp_i) : unp_i;
wire        unp_hs = (r_din[94:87] != 8'd0) && (r_din[94:87] != 8'hFF);
wire        unp_hd = (r_din[94:84] != 11'd0) && (r_din[94:84] != 11'h7FF);
wire [63:0] unp_m  = (r_fmt == 3'd1) ? {unp_hs, r_din[86:64], 40'd0} :
                     (r_fmt == 3'd5) ? {unp_hd, r_din[83:32], 11'd0} :
                     (r_fmt == 3'd0 || r_fmt == 3'd4 || r_fmt == 3'd6) ? {unp_im, 32'd0} :
                     r_din[63:0];

// operand/result in working format
reg         a_s;
reg  [16:0] a_e;
reg  [63:0] a_m;
reg   [1:0] a_t;

// staged shifter: mantissa with 3 rounding bits {G,R,S}, count up to 127
reg [66:0] sh_v;              // {m[63:0], G, R, S}
reg  [6:0] sh_cnt;

// second (destination) operand and arithmetic scratch
reg         b_s;
reg  [16:0] b_e;
reg  [63:0] b_m;
reg   [1:0] b_t;
reg   [2:0] grs;              // result rounding bits before F_ROUND
reg         eff_sub;          // effective magnitude subtract
reg  [64:0] acc_hi;           // multiply accumulator high / divide remainder
reg  [63:0] acc_lo;           // multiply accumulator low / dividend feed
reg  [66:0] qv;               // divide quotient / sqrt root
reg  [68:0] srem;             // sqrt remainder
reg  [65:0] srad;             // sqrt radicand feed (its nonzero top: the
                              // 132-bit radicand's low 66 bits are zero)
reg   [6:0] loop_n;
reg [127:0] mul_pd;           // registered DSP full product (F_MULT)
reg   [3:0] op_kind;          // 0 none, 1 add, 2 mul, 3 div, 4 sqrt
reg   [4:0] sh_ret;           // staged shifter return state
reg   [1:0] r_pr;             // rounding precision latched for F_UNFL
reg         mv_st;            // a move's F_ROUND starts: record its exception shadow
// a datatype or unimplemented-instruction frame is captured in F_CAP, the
// cycle after the decision, from the shadow registers (sh_src, sh_stag,
// sh_dst, sh_dtag: one form for every site -- the dispatch records the
// operands as such a frame wants them) and these: the command word, which
// frame, its T and E1 flags, and a packed operand's E1
reg  [15:0] cap_cmd;
reg         cap_unimp, cap_t, cap_e1, cap_pk;
// the normalizer takes two cycles: F_NLZ registers the leading-zero count
// (nlz_q), the state it returns to (nz_ret: F_NORM, F_NORM2, F_RESTORE_N)
// shifts by it -- the count and the shift in one cycle, with the select
// before and the result multiplexer after, did not close at 60 MHz
reg   [6:0] nlz_q;
reg   [4:0] nz_ret;
reg signed [17:0] e_w;        // working exponent (wrap safe)

// These states already run in separate cycles. Select their input before
// normalization so source, restored destination and add/subtract result
// share one leading-zero encoder and one 67-bit left shifter. Operand
// normalization supplies zero GRS; result normalization preserves it.
// The select is registered with the state (nsel[0]: F_RESTORE_N, nsel[1]:
// F_NORM2, each entered from one place), so the encoder and shifter start
// from flip-flops rather than from a decode of fst.
reg  [1:0] nsel;

// v >> n with every bit shifted out ORed into bit 0 (n >= 67: all out):
// seven stages, each ORing what it shifts out into the sticky bit
function [66:0] shr_sticky;
	input [66:0] v;
	input [6:0]  n;
	reg   [66:0] x;
	reg          s;
	integer      k;
	begin
		x = v; s = 1'b0;
		for (k = 0; k < 7; k = k + 1)
			if (n[k]) begin
				s = s | |(x & ((67'd1 << (1 << k)) - 67'd1));
				x = x >> (1 << k);
			end
		shr_sticky = x | {66'd0, s};
	end
endfunction
// synthesis translate_off
always @(posedge clk)
	if (nsel != {fst == F_NORM2 || (fst == F_NLZ && nz_ret == F_NORM2),
	             fst == F_RESTORE_N || (fst == F_NLZ && nz_ret == F_RESTORE_N)})
		$display("FPU: nsel %b disagrees with state %0d", nsel, fst);
// synthesis translate_on

// integer store bookkeeping
reg        pk_neg;
reg  [1:0] pk_isz;            // 0 byte, 1 word, 2 long

// A pending unimplemented state re-signals on the next FP instruction ONLY
// when software put it there with FRESTORE.  That is the architectural
// re-entry mechanism: the FPSP restores a frame and the next FP dispatch
// hands control back to it.  A state left by our OWN trap must NOT
// re-signal: on a 68040 the unimplemented instruction simply did not
// execute, and the following FP instruction runs normally (WinUAE never
// consults fpu_exp_state at dispatch -- it is written by the exception and
// read only by FSAVE/FRESTORE).  Re-signalling there traps every later FP
// instruction until an FSAVE that a handler is not required to perform;
// worse, the FPSP's own emulation uses FP instructions, so the retrap
// nests until the stack faults during exception processing -- a double
// fault, which halts the core silently.
reg        fstate_resig;

// hardware subset at this stage: move/abs/neg/tst/cmp families
function op_in_hw;
	input [6:0] op;
	begin
		case (op)
			7'h00, 7'h40, 7'h44,          // FMOVE, FSMOVE, FDMOVE
			7'h18, 7'h58, 7'h5C,          // FABS, FSABS, FDABS
			7'h1A, 7'h5A, 7'h5E,          // FNEG, FSNEG, FDNEG
			7'h38, 7'h3A,                 // FCMP, FTST
			7'h22, 7'h62, 7'h66,          // FADD, FSADD, FDADD
			7'h28, 7'h68, 7'h6C,          // FSUB, FSSUB, FDSUB
			7'h23, 7'h27, 7'h63, 7'h67,   // FMUL, FSGLMUL, FSMUL, FDMUL
			7'h20, 7'h24, 7'h60, 7'h64,   // FDIV, FSGLDIV, FSDIV, FDDIV
			7'h04, 7'h41, 7'h45:          // FSQRT, FSSQRT, FDSQRT
				op_in_hw = 1;
			default:
				op_in_hw = 0;
		endcase
	end
endfunction

// result precision for an opmode: 0 extended (per FPCR), 1 single, 2 double
function [1:0] prec_of;
	input [6:0] op;
	begin
		if (op == 7'h24 || op == 7'h27) prec_of = 2'd1;   // FSGLDIV/FSGLMUL
		else if (op >= 7'h40) prec_of = op[2] ? 2'd2 : 2'd1;
		// FPCR rounding precision: 00 extended, 01 single, 10 double, and
		// the reserved encoding 11 rounds as double (softfloat's 68k glue
		// falls through its default into the double case, and hardware
		// cputest agrees).
		else prec_of = (fpcr[7:6] == 2'b01) ? 2'd1 :
		               (fpcr[7:6] == 2'b00) ? 2'd0 : 2'd2;
	end
endfunction


// FSGLMUL and FSGLDIV round the significand to single precision but keep
// the EXTENDED exponent range: they go through roundSigAndPackFloatx80,
// which has no expOffset, unlike the roundAndPackFloatx80 used by every
// FPCR-precision and FS/FD operation.  Their overflow saturation value is
// the full all-ones mantissa at the extended maximum, again unlike the
// masked significand roundAndPackFloatx80 produces.
function op_sgl;
	input [6:0] op;
	begin
		op_sgl = (op == 7'h24) || (op == 7'h27);
	end
endfunction

// IEEE round-up decision from {lsb, G, R|S} and sign
function round_up;
	input       lsb, g, rs;
	input       sign;
	begin
		case (rnd_mode)
			2'b00:   round_up = g && (rs || lsb);   // nearest even
			2'b01:   round_up = 1'b0;               // toward zero
			2'b10:   round_up = sign && (g || rs);  // toward minus
			default: round_up = !sign && (g || rs); // toward plus
		endcase
	end
endfunction

// Highest-priority enabled FPSR exception, matching the architectural
// vector assignment.  Input bit 7 is BSUN and bit 0 is INEX1.
function [7:0] fp_exception_vector;
	input [7:0] ex;
	begin
		if      (ex[7]) fp_exception_vector = `AP040_VEC_FP_BSUN;
		else if (ex[6]) fp_exception_vector = `AP040_VEC_FP_SNAN;
		else if (ex[5]) fp_exception_vector = `AP040_VEC_FP_OPERR;
		else if (ex[4]) fp_exception_vector = `AP040_VEC_FP_OVFL;
		else if (ex[3]) fp_exception_vector = `AP040_VEC_FP_UNFL;
		else if (ex[2]) fp_exception_vector = `AP040_VEC_FP_DZ;
		else             fp_exception_vector = `AP040_VEC_FP_INEX;
	end
endfunction

// Command shadow is captured at dispatch (or from the restored frame).
// Operand shadow is captured at F_EXEC/F_BIN entry, where the classified
// source is in hand -- F_WB has already consumed the working operand.
reg [15:0] sh_cmd;
reg [95:0] sh_src;
reg  [2:0] sh_stag;
reg [95:0] sh_dst;
reg  [2:0] sh_dtag;
// internal intermediate for the BUSY frame's WBTEMP: captured in
// F_ROUND at the overflow/underflow/inexact decision (rounded for
// OVFL/INEX, unrounded for UNFL, per WinUAE fpp_get_internal[_round])
reg        wb_s;
reg [16:0] wb_e;
reg [63:0] wb_m;
reg  [2:0] wb_grs;
reg        fstate_e1;    // the prepared/restored frame is an arithmetic
                         // E1 state, not an unimplemented instruction

// Collapse FMOVEM restore and arithmetic result writes into the regfile's
//---------------------------------------------------------------------------
// arithmetic pipeline
//---------------------------------------------------------------------------
// FMOVE, FABS, FNEG, FADD, FSUB and FMUL (FPCR precision and the FS/FD
// forms; FMOVE also from a long, word or byte integer) whose
// operands are ordinary -- zeros, or normalized numbers whose unbiased
// exponent is within +-60 ("nice") -- can neither overflow nor underflow
// at any rounding precision, and with every FPCR exception enable clear
// they raise nothing that traps.  They run in a pipeline that takes one
// a cycle and writes its results in order; the sequential state machine
// starts an operation only once the pipeline is empty, and everything
// that reads FPU state waits for it (pbusy).  The arithmetic is the state
// machine's, stage for stage, so results and FPSR are bit-identical
// (tb/tb_fpu_pipe.sv runs both side by side):
//   P1  zero cases, operand order and exponent difference (FMUL: the
//       exponent sum)                                          (F_BIN)
//   P2  alignment shift with sticky (FMUL: the 32x32 partial products)
//                                                              (F_SHR)
//   P3  add or subtract (FMUL: their sum, one-bit normalize)   (F_ADDX)
//   P4  normalize after a subtraction, or an integer source    (F_NORM2)
//   P5  round to the precision                                 (F_ROUND)
//   P6  register, FPSR condition codes, INEX                   (F_WB)
// A source that an operation in P5 or P6 is writing when the next one is
// dispatched is forwarded into its P1 (from P6, or P7: P6 one cycle on);
// one written by P1-P4 holds the dispatch.

function pipe_opm;
	input [6:0] op;
	begin
		case (op)
			7'h00, 7'h40, 7'h44, 7'h22, 7'h62, 7'h66,
			7'h28, 7'h68, 7'h6C, 7'h23, 7'h63, 7'h67,
			7'h18, 7'h58, 7'h5C, 7'h1A, 7'h5A, 7'h5E: pipe_opm = 1'b1;
			default: pipe_opm = 1'b0;
		endcase
	end
endfunction

// 0 move, 1 add/subtract, 2 multiply
function [1:0] pipe_kind;
	input [6:0] op;
	begin
		case (op)
			7'h22, 7'h62, 7'h66, 7'h28, 7'h68, 7'h6C: pipe_kind = 2'd1;
			7'h23, 7'h63, 7'h67:                      pipe_kind = 2'd2;
			default:                                  pipe_kind = 2'd0;
		endcase
	end
endfunction

// an ordinary extended value: zero (not infinity), or normalized with
// an exponent of 16383 +- 60
function nice_x;
	input [14:0] e;
	input [63:0] m;
	begin
		nice_x = (m == 64'd0) ? (e != 15'h7FFF)
		                      : (m[63] && e >= 15'd16323 && e <= 15'd16443);
	end
endfunction

reg   [7:0] fr_nice;          // per register: nice_x of its contents
reg         req_pend;         // a request waiting for the pipeline
wire        rq = req || req_pend;

// stage registers
reg         p1_v, p2_v, p3_v, p4_v, p5_v, p6_v, p7_v;
reg   [6:0] p1_op, p2_op, p3_op, p4_op, p5_op;
reg   [2:0] p1_d, p2_d, p3_d, p4_d, p5_d, p6_d;
reg         p1_as, p1_bs, p1_az, p1_bz;
reg  [16:0] p1_ae, p1_be;
reg  [63:0] p1_am, p1_bm;
reg   [1:0] p1_af, p1_bf;     // forwarded from: 0 no, 1 P6, 2 P7
reg   [1:0] p2_k, p3_k;
reg         p2_z, p3_z, p4_z, p5_z, p6_z, p7_z;   // the result is zero
reg         p2_s, p3_s, p4_s, p5_s, p6_s, p7_s;
reg  signed [17:0] p2_e, p3_e, p4_e, p5_e;
reg  [16:0] p6_e, p7_e;
reg  [63:0] p2_bm, p2_sm, p3_bm, p4_m, p5_m, p6_m, p7_m;
reg   [6:0] p2_sh;
reg         p2_es, p3_es, p4_n;
reg  [66:0] p3_sv;
reg   [2:0] p4_g, p5_g;
reg         p6_inx, p6_nice;
reg         p1_ni, p2_ni, p3_ni;   // an integer source: normalize in P4

// The pipeline's P4 normalizes with the same encoder and shifter: it holds
// an operation only while the state machine is idle (p4_v, a register,
// selects it).
wire [63:0] norm_m = nsel[0] ? b_m :
                     nsel[1] ? acc_hi[63:0] :
                     p4_v    ? p4_m : a_m;
wire [6:0] norm_lz = clz64(norm_m);
// (the state machine shifts by the registered count; the pipeline, which
// holds an operation only while the state machine is idle, by the count
// of the cycle)
wire [6:0] norm_cnt = p4_v ? norm_lz : nlz_q;
wire [66:0] norm_shifted =
    {norm_m, (nsel[1] ? grs : (!nsel[0] && p4_v) ? p4_g : 3'd0)} << norm_cnt;
// one right shifter with sticky for F_SHR and the pipeline's P2 alignment
// (P2 holds an operation only while the state machine is idle)
wire        shr_fsm = (fst == F_SHR);
wire [66:0] shr_out = shr_sticky(shr_fsm ? sh_v : {p2_sm, 3'd0}, shr_fsm ? sh_cnt : p2_sh);
// one incrementer for the roundings of the shifted value (sh_v[66:3]):
// the integer store's at bit 0, a denormal single's or double's store
// (F_PACKS) and the underflow rounding (F_UNFL) at the precision's last
// kept bit -- in states of their own.  (The bits below that position ride
// along: the increment carries into the kept bits as it would from the
// masked value, so a caller masks them off the sum.)
wire [64:0] shk    = {1'b0, sh_v[66:3]};
wire [64:0] shk_i  = shk + ((fst == F_PACKS) ? ((r_fmt == 3'd1) ? 65'h100_0000_0000 : 65'h800) :
                            (fst == F_UNFL)  ? ((r_pr == 2'd1) ? 65'h100_0000_0000 :
                                                (r_pr == 2'd2) ? 65'h800 : 65'd1) : 65'd1);

// one 64x64 multiplier for both: the state machine multiplies only in
// F_MULT, which it reaches with the pipeline empty
// (its operands are registers of its own, loaded by whichever goes to
// multiply: F_BIN as it enters F_MULT, P1 for an FMUL)
// The product takes two cycles: the four 32x32 partial products are
// registered (in the DSP blocks), their sum is formed the next cycle --
// a 64x64 product does not close in one 60 MHz cycle without register
// retiming
reg   [63:0] mul_x, mul_y;
reg   [63:0] mul_hh, mul_hl, mul_lh, mul_ll;
always @(posedge clk) if (ce) begin
	mul_hh <= mul_x[63:32] * mul_y[63:32];
	mul_hl <= mul_x[63:32] * mul_y[31:0];
	mul_lh <= mul_x[31:0]  * mul_y[63:32];
	mul_ll <= mul_x[31:0]  * mul_y[31:0];
end
// ({hh, ll} is a concatenation; the cross products sum in 65 bits and
// land on bits 96:32: one 65-bit and one 96-bit adder, not three of 128)
wire  [64:0] mul_cross = {1'b0, mul_hl} + {1'b0, mul_lh};
wire [127:0] mul_out = {mul_hh, mul_ll} + {31'd0, mul_cross, 32'd0};

// One incrementer for the roundings of a_m at a precision's last kept bit:
// F_ROUND's (the operation's precision) and a single or double register
// store's (F_SRC, by the store format), in states of their own.
wire  [1:0] rd_pr   = (fst == F_SRC) ? ((r_fmt == 3'd1) ? 2'd1 : 2'd2) : prec_of(r_op);
wire [63:0] rd_mask = (rd_pr == 2'd1) ? 64'hFFFF_FF00_0000_0000 :
                      (rd_pr == 2'd2) ? 64'hFFFF_FFFF_FFFF_F800 : 64'hFFFF_FFFF_FFFF_FFFF;
wire [64:0] rd_mi   = {1'b0, a_m & rd_mask} +
                      ((rd_pr == 2'd1) ? 65'h100_0000_0000 : (rd_pr == 2'd2) ? 65'h800 : 65'd1);

// The single and double stores, in one form (F_SRC: a register's value
// rounded to the format, or its special value; F_PACKS: a denormal after
// the staged shift): the exponent field sd_E and the fraction sd_F as a
// double's, the single's being their upper bits -- one exponent subtract,
// one overflow test, one output multiplexer for both formats.  The
// rounding takes the shared incrementer (rd_mi: rd_pr is the format's).
reg        sd_isS, sd_den, sd_up, sd_inx, sd_ov, sd_tomax, sd_pr;
reg signed [17:0] sd_sE;
reg [17:0] sd_eun;
reg [16:0] sd_enc;
reg [63:0] sd_m;
reg [10:0] sd_E;
reg [51:0] sd_F;
reg [95:0] sd_out;
always @* begin
	sd_isS = (r_fmt == 3'd1);
	sd_sE  = $signed({a_e[16], a_e}) - 18'sd16383;
	sd_den = sd_isS ? (sd_sE < -18'sd126) : (sd_sE < -18'sd1022);
	sd_up  = sd_isS ? round_up(a_m[40], a_m[39], (a_m[38:0] != 0), a_s) :
	                  round_up(a_m[11], a_m[10], (a_m[9:0] != 0), a_s);
	sd_inx = sd_isS ? (a_m[39:0] != 0) : (a_m[10:0] != 0);
	// (a rounding carry out of the significand leaves sd_m zero: the
	// minimum of the next exponent)
	sd_m   = sd_up ? rd_mi[63:0] : (a_m & rd_mask);
	sd_eun = {1'b0, a_e} + {17'd0, sd_up && rd_mi[64]};
	sd_ov  = sd_eun > (sd_isS ? 18'd16510 : 18'd17406);
	sd_enc = sd_eun[16:0] - (sd_isS ? 17'd16256 : 17'd15360);
	sd_tomax = (rnd_mode == 2'b01) || (rnd_mode == 2'b10 && !a_s) || (rnd_mode == 2'b11 && a_s);
	// F_PACKS: the denormal's rounding at the format's last bit
	sd_pr  = sd_isS ? round_up(sh_v[43], sh_v[42], sh_v[41:0] != 0, a_s) :
	                  round_up(sh_v[14], sh_v[13], sh_v[12:0] != 0, a_s);
	if (fst == F_PACKS) begin
		sd_E = {10'd0, sd_pr ? shk_i[63] : shk[63]};
		sd_F = sd_pr ? shk_i[62:11] : shk[62:11];
	end
	else if (a_t != T_NUM) begin
		sd_E = (a_t == T_ZERO) ? 11'd0 : 11'h7FF;
		sd_F = (a_t == T_NAN) ? (a_m[62:11] | 52'h8_0000_0000_0000) : 52'd0;
	end
	else if (sd_ov) begin
		sd_E = sd_tomax ? 11'h7FE : 11'h7FF;
		sd_F = sd_tomax ? {52{1'b1}} : 52'd0;
	end
	else begin
		sd_E = sd_enc[10:0];
		sd_F = sd_m[62:11];
	end
	sd_out = sd_isS ? {a_s, sd_E[7:0], sd_F[51:29], 64'd0} : {a_s, sd_E, sd_F, 32'd0};
end

// The arithmetic results into F_ROUND -- the add's sum, the product, the
// quotient and the root -- through one window select: rn_r holds the
// result with its leading bit at 67 or 66 and rn_s the sticky of the bits
// below it; a leading bit at 67 (never for the root) takes the upper
// window and adds one to the exponent (one less for the quotient).  One
// source of a_m and grs for the four, not two each.
wire [67:0] add_sum = {1'b0, b_m, 3'd0} + {1'b0, sh_v};
reg  [67:0] rn_r;
reg         rn_s, rn_hi;
reg  [63:0] rn_m;
reg   [2:0] rn_g;
always @* begin
	case (fst)
		F_ADDX:  begin rn_r = add_sum;            rn_s = 1'b0; end
		F_MULT:  begin rn_r = mul_pd[127:60];     rn_s = (mul_pd[59:0] != 60'd0); end
		F_SQRTL: begin rn_r = {qv, 1'b0};         rn_s = (srem != 69'd0); end
		default: begin rn_r = {qv, 1'b0};         rn_s = (acc_hi != 65'd0); end
	endcase
	rn_hi = rn_r[67] && (fst != F_SQRTL);
	rn_m  = rn_hi ? rn_r[67:4] : rn_r[66:3];
	rn_g  = rn_hi ? {rn_r[3], rn_r[2], rn_r[1] | rn_r[0] | rn_s} :
	                {rn_r[2], rn_r[1], rn_r[0] | rn_s};
end

// FDIV and FSQRT: one restoring datapath, two steps a cycle.  Divide:
// the remainder shifts one bit (zeros enter; the first, integer step does
// not shift) and the divisor is the trial; square root: the remainder
// shifts two bits (the radicand's next two enter) and the trial is the
// root so far with 01.  The borrow of each trial subtraction is the bit.
wire        ds_sq = (fst == F_SQRTL);
wire        ds_d0 = !ds_sq && (loop_n == 7'd0);
reg  [68:0] ds_rem1, ds_rem;  // the remainder after one step, after two
reg         ds_q1, ds_q2;
always @* begin : dsq
	reg [68:0] r2a, rem1, r2b, t1, t2;
	reg [69:0] d1, d2;
	r2a = ds_sq ? {srem[66:0], srad[65:64]} :
	      ds_d0 ? {4'd0, acc_hi} : {3'd0, acc_hi[63:0], 1'b0};
	t1  = ds_sq ? {1'b0, qv[65:0], 2'b01} : {5'd0, a_m};
	d1  = {1'b0, r2a} - {1'b0, t1};
	ds_q1 = !d1[69];
	rem1 = ds_q1 ? d1[68:0] : r2a;
	ds_rem1 = rem1;
	r2b = ds_sq ? {rem1[66:0], srad[63:62]} : {3'd0, rem1[63:0], 1'b0};
	t2  = ds_sq ? {1'b0, qv[64:0], ds_q1, 2'b01} : t1;
	d2  = {1'b0, r2b} - {1'b0, t2};
	ds_q2 = !d2[69];
	ds_rem = ds_q2 ? d2[68:0] : r2b;
end

assign pbusy = p1_v || p2_v || p3_v || p4_v || p5_v || p6_v;
assign pdone = pipe_issue;

// the youngest writer in flight of register r: 0 none, 1 P1-P4 (wait),
// 2 P5 (forward from P6), 3 P6 (forward from P7)
// (from per-register vectors kept alongside the stages: the dispatch
// feeds pdone, and so the core's stall path -- no destination compares)
reg   [7:0] pb14, pb5, pb6;   // register r written by P1-P4, P5, P6
function [1:0] pw;
	input [2:0] r;
	begin
		if (pb14[r]) pw = 2'd1;
		else if (pb5[r]) pw = 2'd2;
		else if (pb6[r]) pw = 2'd3;
		else pw = 2'd0;
	end
endfunction

// the pipeline's register writers one cycle on
wire  [7:0] pb14_n = (pipe_issue ? (8'd1 << dst_r) : 8'd0) | (p1_v ? (8'd1 << p1_d) : 8'd0) |
                     (p2_v ? (8'd1 << p2_d) : 8'd0) | (p3_v ? (8'd1 << p3_d) : 8'd0);
wire  [7:0] pb5_n  = p4_v ? (8'd1 << p4_d) : 8'd0;
wire  [7:0] pb6_n  = p5_v ? (8'd1 << p5_d) : 8'd0;

// okv[r]: register r may be a pipelined operation's operand now -- not
// being written by P1-P4, and nice (P5's or P6's result as it will be, or
// the register's own).  Registered from the next state, so the dispatch
// (pdone, and the core's stall path) selects one flip-flop per operand.
reg   [7:0] okv;
reg   [7:0] okv_n;

// P5's result will be nice, decided a stage early (a register: it feeds
// the dispatch, pdone, and so the core's stall path): its exponent before
// rounding within +-59 (rounding adds at most one), P4's normalize taking
// up to 64 off -- conservative: a result this misses is only not
// forwarded (the dispatch waits for its write and reads the flag then)
reg  p5_nice;
wire p5_nice_n = p4_z || (p4_n ? (p4_e >= 18'sd16388 && p4_e <= 18'sd16442)
                                : (p4_e >= 18'sd16324 && p4_e <= 18'sd16442));

// the memory operand: single, double or extended; or an integer (long,
// word, byte) for a move, normalized in P4 as F_NORM does
reg         q_s, q_z, q_n, q_i;
reg  [16:0] q_e;
reg  [63:0] q_m;
always @* begin
	q_s = din[95]; q_z = 1'b0; q_n = 1'b0; q_i = 1'b0; q_e = 17'd0; q_m = 64'd0;
	case (src_fmt)
		3'd0: begin
			q_i = 1'b1; q_n = 1'b1;
			q_z = (din[95:64] == 32'd0);
			q_e = 17'd16383 + 17'd31;
			q_m = {(din[95] ? (32'd0 - din[95:64]) : din[95:64]), 32'd0};
		end
		3'd4: begin
			q_i = 1'b1; q_n = 1'b1;
			q_z = (din[95:80] == 16'd0);
			q_e = 17'd16383 + 17'd15;
			q_m = {(din[95] ? (16'd0 - din[95:80]) : din[95:80]), 48'd0};
		end
		3'd6: begin
			q_i = 1'b1; q_n = 1'b1;
			q_z = (din[95:88] == 8'd0);
			q_e = 17'd16383 + 17'd7;
			q_m = {(din[95] ? (8'd0 - din[95:88]) : din[95:88]), 56'd0};
		end
		3'd1: begin
			q_z = (din[94:64] == 31'd0);
			q_n = q_z || (din[94:87] >= 8'd67 && din[94:87] <= 8'd187);
			q_e = q_z ? 17'd0 : ({9'd0, din[94:87]} + 17'd16256);
			q_m = q_z ? 64'd0 : {1'b1, din[86:64], 40'd0};
		end
		3'd5: begin
			q_z = (din[94:32] == 63'd0);
			q_n = q_z || (din[94:84] >= 11'd963 && din[94:84] <= 11'd1083);
			q_e = q_z ? 17'd0 : ({6'd0, din[94:84]} + 17'd15360);
			q_m = q_z ? 64'd0 : {1'b1, din[83:32], 11'd0};
		end
		default: begin
			q_z = (din[63:0] == 64'd0);
			q_n = nice_x(din[94:80], din[63:0]);
			q_e = {2'd0, din[94:80]};
			q_m = din[63:0];
		end
	endcase
end

wire [1:0] q_kind = pipe_kind(opmode);
wire       q_reg  = (op_class == 3'b000);
wire       q_mem  = (op_class == 3'b010) &&
                    ((src_fmt == 3'd1 || src_fmt == 3'd5 || src_fmt == 3'd2) ||
                     // an integer (always ordinary) only for a move: a binary
                     // operation needs its source normalized before P1
                     ((src_fmt == 3'd0 || src_fmt == 3'd4 || src_fmt == 3'd6) &&
                      q_kind == 2'd0));
// what the operation's words alone decide: it can go into the pipeline
// (opmode, a register or memory source of a format it takes), its source
// is memory, it is a move (no destination operand); with sel_x
// registered from the next words, off the dispatch's path
function [2:0] q_static;
	input [6:0] opm;
	input [2:0] cls, fmt;
	reg [1:0] k;
	reg       m;
	begin
		k = pipe_kind(opm);
		m = (cls == 3'b010) && ((fmt == 3'd1 || fmt == 3'd5 || fmt == 3'd2) ||
		                        ((fmt == 3'd0 || fmt == 3'd4 || fmt == 3'd6) && k == 2'd0));
		q_static = {pipe_opm(opm) && ((cls == 3'b000) || m), m, k == 2'd0};
	end
endfunction
reg  [2:0] qs_q;
always @(posedge clk) qs_q <= q_static(opmode_n, op_class_n, src_fmt_n);
wire [2:0] qs_l = q_static(opmode, op_class, src_fmt);
wire [2:0] qs   = sel_x ? qs_q : qs_l;
// synthesis translate_off
always @(posedge clk)
	if (nreset && sel_x && qs_q != qs_l) begin
		$display("FAIL: the registered pipeline eligibility %b, the words' %b", qs_q, qs_l);
		$finish;
	end
// synthesis translate_on

wire [1:0] q_aw   = pw(src_r);
wire [1:0] q_bw   = pw(dst_r);
wire       q_aok  = qs[1] ? din_nice : sel_x ? aok_x : okv[src_r];
wire       q_bok  = qs[0] ? 1'b1 : sel_x ? bok_x : okv[dst_r];
assign okv_next = okv_n;
// (not in FRESTORE's unimplemented-frame cycle at all: the frame's
// resume decode stays off the dispatch, and with it the core's stall path;
// a request then goes to the state machine)
wire pipe_issue = (PIPE != 0) && ce && rq && (fst == F_IDLE) &&
                  !frestore_unimp &&
                  !fstate_unimp && !fstate_e1 && !fstate_busy && !fstate_resig &&
                  (fpcr[15:8] == 8'd0) && qs[2] && q_aok && q_bok;

// synthesis translate_off
always @(posedge clk) begin : pb_check
	integer r;
	for (r = 0; r < 8; r = r + 1)
		if (nreset && (pb14[r] != ((p1_v && p1_d == r) || (p2_v && p2_d == r) ||
		                           (p3_v && p3_d == r) || (p4_v && p4_d == r)) ||
		               pb5[r] != (p5_v && p5_d == r) || pb6[r] != (p6_v && p6_d == r))) begin
			$display("FAIL: pipe writer vectors disagree with the stages (r %0d)", r);
			$finish;
		end
end
always @(posedge clk)
	if (ce && rq && q_mem && din_nice !== q_n) begin
		$display("FAIL: din_nice %b, the operand classifies as %b (fmt %0d din %h)",
		         din_nice, q_n, src_fmt, din);
		$finish;
	end
// synthesis translate_on

// P1: the operands (forwarded), the zero cases, order and exponents
reg         s1_as, s1_bs, s1_az, s1_bz;
reg  [16:0] s1_ae, s1_be;
reg  [63:0] s1_am, s1_bm;
always @* begin
	{s1_as, s1_ae, s1_am, s1_az} = (p1_af == 2'd1) ? {p6_s, p6_e, p6_m, p6_z} :
	                               (p1_af == 2'd2) ? {p7_s, p7_e, p7_m, p7_z} :
	                                                 {p1_as, p1_ae, p1_am, p1_az};
	{s1_bs, s1_be, s1_bm, s1_bz} = (p1_bf == 2'd1) ? {p6_s, p6_e, p6_m, p6_z} :
	                               (p1_bf == 2'd2) ? {p7_s, p7_e, p7_m, p7_z} :
	                                                 {p1_bs, p1_be, p1_bm, p1_bz};
end

always @(posedge clk) begin
	if (!nreset) begin
		p1_v <= 0; p2_v <= 0; p3_v <= 0; p4_v <= 0; p5_v <= 0; p6_v <= 0; p7_v <= 0;
		pb14 <= 8'd0; pb5 <= 8'd0; pb6 <= 8'd0; okv <= 8'd0;
	end
	else if (ce) begin
		//----------------------------------------------------------- dispatch
		p1_v <= pipe_issue;
		pb14 <= pb14_n;
		pb5  <= pb5_n;
		pb6  <= pb6_n;
		okv  <= okv_n;
		if (pipe_issue) begin
			p1_op <= opmode;
			p1_d  <= dst_r;
			p1_af <= (q_mem || q_aw == 2'd0) ? 2'd0 : (q_aw == 2'd2) ? 2'd1 : 2'd2;
			p1_bf <= (q_bw == 2'd0) ? 2'd0 : (q_bw == 2'd2) ? 2'd1 : 2'd2;
			p1_ni <= q_mem && q_i;
			if (q_mem) begin
				p1_as <= q_s; p1_ae <= q_e; p1_am <= q_m; p1_az <= q_z;
			end
			else begin
				p1_as <= fr_src_s; p1_ae <= {2'd0, fr_src_e}; p1_am <= fr_src_m;
				p1_az <= (fr_src_m == 64'd0);
			end
			p1_bs <= fr_dst_s; p1_be <= {2'd0, fr_dst_e}; p1_bm <= fr_dst_m;
			p1_bz <= (fr_dst_m == 64'd0);
		end

		//----------------------------------------------------------------- P1
		p2_v  <= p1_v;
		p2_ni <= p1_ni;
		p2_op <= p1_op;
		p2_d  <= p1_d;
		p2_z  <= 1'b0;
		p2_es <= 1'b0;
		p2_sh <= 7'd0;
		p2_sm <= 64'd0;
		case (pipe_kind(p1_op))
			2'd1: begin : p1_add
				reg        sa, aswap;
				reg [16:0] dd;
				// FSUB family: the source sign folded
				sa = (p1_op == 7'h28 || p1_op == 7'h68 || p1_op == 7'h6C) ? ~s1_as : s1_as;
				if (s1_az && s1_bz) begin
					p2_k <= 2'd0; p2_z <= 1'b1;
					p2_s <= (sa == s1_bs) ? sa : (rnd_mode == 2'b10);
					p2_e <= 18'sd0; p2_bm <= 64'd0;
				end
				else if (s1_az) begin
					// a zero source: the destination, rounded
					p2_k <= 2'd0; p2_s <= s1_bs;
					p2_e <= $signed({1'b0, s1_be}); p2_bm <= s1_bm;
				end
				else if (s1_bz) begin
					p2_k <= 2'd0; p2_s <= sa;
					p2_e <= $signed({1'b0, s1_ae}); p2_bm <= s1_am;
				end
				else begin
					aswap = (s1_ae > s1_be) || (s1_ae == s1_be && s1_am > s1_bm);
					dd    = aswap ? (s1_ae - s1_be) : (s1_be - s1_ae);
					p2_k  <= 2'd1;
					p2_es <= (sa != s1_bs);
					p2_s  <= aswap ? sa : s1_bs;
					p2_e  <= $signed({1'b0, aswap ? s1_ae : s1_be});
					p2_bm <= aswap ? s1_am : s1_bm;
					p2_sm <= aswap ? s1_bm : s1_am;
					p2_sh <= (dd > 17'd66) ? 7'd67 : dd[6:0];
				end
			end
			2'd2: begin
				p2_s <= s1_as ^ s1_bs;
				if (s1_az || s1_bz) begin
					p2_k <= 2'd0; p2_z <= 1'b1; p2_e <= 18'sd0; p2_bm <= 64'd0;
				end
				else begin
					p2_k  <= 2'd2;
					p2_e  <= $signed({1'b0, s1_ae}) + $signed({1'b0, s1_be}) - 18'sd16383;
					p2_bm <= s1_am;
					p2_sm <= s1_bm;
				end
			end
			default: begin
				// FMOVE; FABS clears the sign, FNEG flips it (F_EXEC)
				p2_k <= 2'd0; p2_z <= s1_az;
				p2_s <= (p1_op == 7'h18 || p1_op == 7'h58 || p1_op == 7'h5C) ? 1'b0 :
				        (p1_op == 7'h1A || p1_op == 7'h5A || p1_op == 7'h5E) ? ~s1_as : s1_as;
				p2_e <= s1_az ? 18'sd0 : $signed({1'b0, s1_ae});
				p2_bm <= s1_am;
			end
		endcase

		//----------------------------------------------------------------- P2
		p3_v  <= p2_v;
		p3_ni <= p2_ni;
		p3_op <= p2_op;
		p3_d  <= p2_d;
		p3_k  <= p2_k;
		p3_z  <= p2_z;
		p3_s  <= p2_s;
		p3_e  <= p2_e;
		p3_es <= p2_es;
		p3_bm <= p2_bm;
		p3_sv <= shr_out;

		//----------------------------------------------------------------- P3
		p4_v  <= p3_v;
		p4_op <= p3_op;
		p4_d  <= p3_d;
		p4_s  <= p3_s;
		p4_z  <= p3_z;
		p4_e  <= p3_e;
		p4_n  <= p3_ni && !p3_z;      // an integer move: normalize
		p4_g  <= 3'd0;
		p4_m  <= p3_bm;
		case (p3_k)
			2'd1: begin : p3_add
				reg [67:0] sum;
				reg [66:0] diff;
				if (!p3_es) begin
					sum = {1'b0, p3_bm, 3'd0} + {1'b0, p3_sv};
					if (sum[67]) begin
						p4_m <= sum[67:4];
						p4_g <= {sum[3], sum[2], sum[1] | sum[0]};
						p4_e <= p3_e + 18'sd1;
					end
					else begin
						p4_m <= sum[66:3];
						p4_g <= sum[2:0];
					end
				end
				else begin
					diff = {p3_bm, 3'd0} - p3_sv;
					if (diff == 67'd0) begin
						p4_z <= 1'b1;
						p4_s <= (rnd_mode == 2'b10);
					end
					else begin
						p4_m <= diff[66:3];
						p4_g <= diff[2:0];
						p4_n <= 1'b1;
					end
				end
			end
			2'd2: begin
				// the partial products P2 registered, summed here
				if (mul_out[127]) begin
					p4_m <= mul_out[127:64];
					p4_g <= {mul_out[63], mul_out[62], (mul_out[61:0] != 0)};
					p4_e <= p3_e + 18'sd1;
				end
				else begin
					p4_m <= mul_out[126:63];
					p4_g <= {mul_out[62], mul_out[61], (mul_out[60:0] != 0)};
				end
			end
			default: ;
		endcase

		//----------------------------------------------------------------- P4
		p5_v  <= p4_v;
		p5_op <= p4_op;
		p5_d  <= p4_d;
		p5_s  <= p4_s;
		p5_z  <= p4_z;
		p5_e  <= p4_e;
		p5_m  <= p4_m;
		p5_g  <= p4_g;
		p5_nice <= p5_nice_n;
		if (p4_n) begin : p4_norm
			reg [6:0]  lz;
			reg [66:0] nv;
			lz = norm_lz;                 // (the shared encoder: p4_v selects)
			if (lz == 7'd64) begin
				p5_m <= {p4_g, 61'd0};
				p5_g <= 3'd0;
				p5_e <= p4_e - 18'sd64;
			end
			else begin
				nv = norm_shifted;
				p5_m <= nv[66:3];
				p5_g <= nv[2:0];
				p5_e <= p4_e - {11'd0, lz};
			end
		end

		//----------------------------------------------------------------- P5
		p6_v <= p5_v;
		p6_d <= p5_d;
		p6_s <= p5_s;
		p6_z <= p5_z;
		p6_inx <= 1'b0;
		p6_nice <= p5_nice;
		if (p5_z) begin
			p6_e <= 17'd0;
			p6_m <= 64'd0;
		end
		else begin : p5_round
			reg [64:0] mr, mk, mi;
			reg [63:0] rmask, rinc;
			reg        inx, up;
			reg signed [17:0] er;
			case (prec_of(p5_op))
				2'd1: begin
					up = round_up(p5_m[40], p5_m[39], (p5_m[38:0] != 0) || (p5_g != 0), p5_s);
					inx = (p5_m[39:0] != 0) || (p5_g != 0);
					rmask = 64'hFFFF_FF00_0000_0000;
					rinc  = 64'h0000_0100_0000_0000;
				end
				2'd2: begin
					up = round_up(p5_m[11], p5_m[10], (p5_m[9:0] != 0) || (p5_g != 0), p5_s);
					inx = (p5_m[10:0] != 0) || (p5_g != 0);
					rmask = 64'hFFFF_FFFF_FFFF_F800;
					rinc  = 64'h0000_0000_0000_0800;
				end
				default: begin
					up = round_up(p5_m[0], p5_g[2], p5_g[1:0] != 0, p5_s);
					inx = (p5_g != 0);
					rmask = 64'hFFFF_FFFF_FFFF_FFFF;
					rinc  = 64'h0000_0000_0000_0001;
				end
			endcase
			mk = {1'b0, p5_m & rmask};
			mi = mk + {1'b0, rinc};
			mr = up ? mi : mk;
			er = (up && mi[64]) ? (p5_e + 18'sd1) : p5_e;
			if (mr[64]) mr = {2'b01, 63'd0};
			p6_e <= er[16:0];
			p6_m <= mr[63:0];
			p6_inx <= inx;
		end

		//------------------------------------------------- P6 (and P7 after)
		p7_v <= p6_v;
		p7_s <= p6_s;
		p7_e <= p6_e;
		p7_m <= p6_m;
		p7_z <= p6_z;
	end
end

// single physical write port.  The core cannot dispatch FMOVEM while the FPU
// is in F_WB; keeping the explicit priority also matches the old procedural
// assignment order if that invariant is ever violated.
wire fr_wb_we = (fst == F_WB) && !(|(fpsr[15:8] & fpcr[15:8])) &&
                (r_op != 7'h38) && (r_op != 7'h3A);
wire [14:0] fr_wb_e = (a_t == T_ZERO) ? 15'd0 :
                       (a_t == T_INF || a_t == T_NAN) ? 15'h7FFF : a_e[14:0];
wire [63:0] fr_wb_m = (a_t == T_ZERO) ? 64'd0 : a_m;
wire        fr_bank_we = fm_we || fr_wb_we || p6_v;
wire  [2:0] fr_bank_wa = p6_v ? p6_d : fr_wb_we ? r_dst : fm_sel;
wire [79:0] fr_bank_wd = p6_v ? {p6_s, p6_z ? 15'd0 : p6_e[14:0], p6_z ? 64'd0 : p6_m} :
                         fr_wb_we ? {a_s, fr_wb_e, fr_wb_m} :
                                    {fm_wdata[95], fm_wdata[94:80],
                                     fm_wdata[63:0]};
// the written value's classes, formed for each source and then selected
// (the write's source decode -- the state -- stays off the classifiers)
wire fr_bank_uns  = p6_v     ? unsupported_x(p6_z ? 15'd0 : p6_e[14:0], p6_z ? 64'd0 : p6_m) :
                    fr_wb_we ? unsupported_x(fr_wb_e, fr_wb_m) :
                               unsupported_x(fm_wdata[94:80], fm_wdata[63:0]);
wire fr_bank_nice = p6_v     ? nice_x(p6_z ? 15'd0 : p6_e[14:0], p6_z ? 64'd0 : p6_m) :
                    fr_wb_we ? nice_x(fr_wb_e, fr_wb_m) :
                               nice_x(fm_wdata[94:80], fm_wdata[63:0]);

// okv one cycle on: the writers' next state, P5's niceness as it is
// decided, P6's as it moves on, and the register file's flags after this
// cycle's write (every write makes the register valid) or FRESTORE NULL
always @* begin : okv_nx
	integer r;
	reg wnice;
	wnice = fr_bank_nice;
	for (r = 0; r < 8; r = r + 1)
		okv_n[r] = pb14_n[r] ? 1'b0 :
		           pb5_n[r]  ? p5_nice_n :
		           pb6_n[r]  ? p5_nice :
		           fp_reset  ? 1'b0 :
		           (fr_bank_we && fr_bank_wa == r) ? wnice :
		           (fr_valid[r] && fr_nice[r]);
end
// synthesis translate_off
always @(posedge clk)
	if (nreset && sel_x && (aok_x != okv[src_r] || bok_x != okv[dst_r])) begin
		$display("FAIL: the outside selection of okv %b%b, src %0d dst %0d: %b%b",
		         aok_x, bok_x, src_r, dst_r, okv[src_r], okv[dst_r]);
		$finish;
	end
always @(posedge clk) begin : okv_check
	integer r;
	for (r = 0; r < 8; r = r + 1)
		if (nreset && okv[r] != (pw(r) == 2'd1 ? 1'b0 : pw(r) == 2'd2 ? p5_nice :
		                         pw(r) == 2'd3 ? p6_nice : (fr_valid[r] && fr_nice[r]))) begin
			$display("FAIL: okv[%0d] %b disagrees with the pipeline and register flags", r, okv[r]);
			$finish;
		end
end
// synthesis translate_on

// synthesis translate_off
always @(posedge clk)
	if (ce && fst == F_NORM && a_t == T_NUM && a_m == 64'd0 && norm_shifted[66:3] != 64'd0) begin
		$display("FAIL: F_NORM of a zero significand gave %h", norm_shifted[66:3]);
		$finish;
	end
always @(posedge clk)
	if (ce && !fr_bank_we && fr_src_unsup != unsupported_x(fr_src_e, fr_src_m)) begin
		$display("FAIL: fr_unsup[%0d] = %b, the register classifies as %b",
		         src_r, fr_src_unsup, unsupported_x(fr_src_e, fr_src_m));
		$finish;
	end
// synthesis translate_on

ap040_fp_regfile fpregs
(
	.clk(clk), .ce(ce), .we(fr_bank_we),
	.waddr(fr_bank_wa), .wdata(fr_bank_wd),
	.raddr_a(src_r), .rdata_a(fr_src_raw),
	.raddr_b(dst_r), .rdata_b(fr_dst_raw)
);

integer k;
always @(posedge clk) begin
	if (!nreset) begin
		for (k = 0; k < 8; k = k + 1) begin
			fr_s[k] <= 0;
			fr_e[k] <= 15'h7FFF;
			fr_m[k] <= 64'hFFFF_FFFF_FFFF_FFFF;
		end
	end
	else if (ce) begin
		if (fp_reset) begin
			for (k = 0; k < 8; k = k + 1) begin
				fr_s[k] <= 0;
				fr_e[k] <= 15'h7FFF;
				fr_m[k] <= 64'hFFFF_FFFF_FFFF_FFFF;
			end
		end
		if (fr_bank_we) begin
			fr_s[fr_bank_wa] <= fr_bank_wd[79];
			fr_e[fr_bank_wa] <= fr_bank_wd[78:64];
			fr_m[fr_bank_wa] <= fr_bank_wd[63:0];
		end
	end
end

assign cur_vec = fp_exception_vector(fpsr[15:8] & fpcr[15:8]);

// WinUAE fp_is_dyadic: the operations with a destination operand
function dyadic;
	input [6:0] op;
	begin
		dyadic = ((op & 7'h30) == 7'h20) || (op == 7'h38);
	end
endfunction

// MC68040UM table 9-16: OVFL/UNFL/INEX from the five arithmetic ops
// take the BUSY/E3 frame; moves, absolute value and negate use E1.
function arith5;
	input [6:0] op;
	begin
		// Include forced-precision sqrt and its saved 04 -> 05 encoding.
		// Masking with 3f would instead include FDMOVE (44).
		arith5 = ((op & 7'h30) == 7'h20) || op == 7'h04 ||
		         op == 7'h05 || op == 7'h41 || op == 7'h45;
	end
endfunction

wire [6:0] fr_cmd_op = (fstate_cmd1[6:0] == 7'h05) ? 7'h04
                                                     : fstate_cmd1[6:0];
assign frestore_resume = frestore_busy && frestore_cusavepc == 8'hfe &&
                         (fstate_cmd1[15:13] == 3'd0 ||
                          fstate_cmd1[15:13] == 3'd2) && op_in_hw(fr_cmd_op);

// FPSP get_op (mk_norm/fix_stag) and bugfix set ETE15/FPTE15 for normal
// operands whose exponent is below $4000 too: e.g. 0.1 has e=$3ffb,
// ETE15=1. The extension flag alone does NOT mean a wrapped negative
// biased exponent. Only a set flag together with e[14] requires a shift.
// For that negative range, shift by 0x8000-e, truncating (not rounding);
// counts above 63 produce signed zero. WinUAE's unconditional esign test
// also loses ordinary FPSP operands; do not reproduce that behavior here.
// Reuse F_SHR for both operands instead of adding two 64-bit barrel shifters.
wire restore_et_negative = frestore_et15 && fstate_et[94];
wire restore_fpt_negative = frestore_fpt15 && fstate_fpt[94];
function [6:0] restore_shift;
	input esign;
	input [14:0] e;
	begin
		restore_shift = !(esign && e[14]) ? 7'd0 :
		                (e < 15'h7fc1) ? 7'd64 : (7'd0 - e[6:0]);
	end
endfunction
assign frestore_e1_pend = (fstate_flags[2] || fstate_flags[1]) &&
                          op_in_hw(fr_cmd_op) &&
                          (|(fpsr[15:8] & fpcr[15:8]));

function [15:0] frame_cmd1;
	input [15:0] cmd;
	begin
		// The 040 stores FSQRT's internal command encoding as 5 rather than 4.
		frame_cmd1 = (cmd[6:0] == 7'h04) ? (cmd | 16'h0001) : cmd;
	end
endfunction

function [15:0] frame_cmd3;
	input [15:0] cmd;
	begin
		frame_cmd3 = (cmd & 16'h03C3) |
		             ((cmd & 16'h0038) >> 1) |
		             ((cmd & 16'h0004) << 3);
	end
endfunction

task capture_unimp;
	input [15:0] cmd;
	input [95:0] src;
	input  [2:0] stag;
	input [95:0] dst;
	input  [2:0] dtag;
	reg   [15:0] c1;
	begin
		c1 = frame_cmd1(cmd);
		fstate_cmd1  <= c1;
		fstate_cmd3  <= frame_cmd3(c1);
		fstate_stag  <= stag;
		fstate_dtag  <= dtag;
		// E1/E3/T are the ARITHMETIC exception-pending bits and stay CLEAR
		// on the unimplemented-instruction frame.  WinUAE sets them only in
		// fpsr_check_arithmetic_exception (e1/e3 classes) and for a packed
		// operand in fp_unimp_datatype; fp_unimp_instruction leaves the
		// reset_fsave_data zeros in place.  This frame carries the
		// instruction in CMDREG1B and its operand in ETEMP -- there is no
		// pending arithmetic exception for the FPSP to complete, and
		// claiming one sends its dispatch down the arithmetic path.
		fstate_flags <= 3'b000; // E1=E3=T=0

		fstate_fpt   <= dst;
		fstate_et    <= src;
		fstate_e1    <= 0;
		fstate_grs   <= 0;
		fstate_wbte15 <= 0;
		fstate_busy  <= 0;
		fstate_unimp <= 1;
		fstate_resig <= 0;   // our own trap: the next FP instruction runs
		fpu_used     <= 1;
		unimp        <= 1;
	end
endtask

// A datatype fault (vector 55) must leave a BUSY frame the FPSP can
// parse.  Previously the fault was raised with no capture at all, so a
// handler's FSAVE saw the IDLE frame ($4100 -- measured) and had no
// CMDREG, ETEMP, tags or FPIARCU to emulate the operand from.
//
// WinUAE fp_unimp_datatype's 68040 recipe: fpu_exp_state = 2 (BUSY),
// cmdreg1b = the command word (FSQRT 4->5, which frame_cmd1 does),
// fpiarcu = fpiar, E1 set for a packed operand, and for OPCLASS 011 the
// T bit with the register value in BOTH ETEMP and FPTEMP.  For opclass
// 000/010 the source goes to ETEMP with its tag, and a dyadic operation
// also carries the destination in FPTEMP.
task capture_datatype;
	input [15:0] cmd;
	input [95:0] src;
	input  [2:0] stag;
	input [95:0] dst;
	input  [2:0] dtag;
	input        t_flag;    // OPCLASS 011 register store
	input        e1_flag;   // packed operand
	reg   [15:0] c1;
	begin
		c1 = frame_cmd1(cmd);
		fstate_cmd1  <= c1;
		fstate_cmd3  <= frame_cmd3(c1);
		fstate_stag  <= stag;
		fstate_dtag  <= dtag;
		fstate_flags <= {e1_flag, 1'b0, t_flag};   // {E1,E3,T}
		fstate_fpt   <= dst;
		fstate_et    <= src;
		fstate_grs   <= 0;
		fstate_wbte15 <= 0;
		fstate_wbt   <= 0;
		// Stores can dispatch on the same edge as the FPIAR side-port
		// update.  Capture this instruction, not the previous FPIAR value.
		fstate_fpiar_c <= ia_we ? ia_wdata : fpiar;
		fstate_busy  <= 1;
		fstate_e1    <= e1_flag;
		// fstate_unimp marks "a frame is prepared", so FSAVE extracts it
		// instead of reporting IDLE; fstate_busy selects the $41/$60
		// format.  Both are needed, exactly as the e3 arm above does.
		fstate_unimp <= 1;
		fstate_resig <= 0;   // our own trap: the next FP instruction runs
		fpu_used     <= 1;
	end
endtask

always @(posedge clk) begin
	if (!nreset) begin
		begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end
		done <= 0; unimp <= 0; unsupp <= 0; exc_req <= 0; exc_vec <= 0;
		fpcr <= 0; fpsr <= 0; fpiar <= 0;
		fpu_used <= 0;
		fstate_unimp <= 0;
		fstate_resig <= 0;
		fstate_cmd1 <= 0; fstate_cmd3 <= 0;
		fstate_stag <= 0; fstate_dtag <= 0; fstate_flags <= 0;
		fstate_fpt <= 0; fstate_et <= 0;
		fstate_e1 <= 0; fstate_grs <= 0; fstate_wbte15 <= 0;
		fstate_busy <= 0; fstate_wbt <= 0; fstate_fpiar_c <= 0;
		sh_cmd <= 0; sh_src <= 0; sh_stag <= 0;
		cap_cmd <= 0; cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
		nlz_q <= 0; nz_ret <= F_NORM;
		sh_dst <= 0; sh_dtag <= 0;
		wb_s <= 0; wb_e <= 0; wb_m <= 0; wb_grs <= 0;
		dout <= 0;
		r_fmt <= 0; r_dst <= 0; r_op <= 0; r_din <= 0;
		r_ae7 <= 0;
		r_unimp <= 0; r_stag <= 0; r_resume <= 0;
		a_s <= 0; a_e <= 0; a_m <= 0; a_t <= 0;
		sh_v <= 0; sh_cnt <= 0;
		pk_neg <= 0; pk_isz <= 0;
		b_s <= 0; b_e <= 0; b_m <= 0; b_t <= 0;
		grs <= 0; eff_sub <= 0; acc_hi <= 0; acc_lo <= 0;
		qv <= 0; srem <= 0; srad <= 0; loop_n <= 0; op_kind <= 0;
		sh_ret <= F_PACKI; e_w <= 0; r_pr <= 0; mv_st <= 0;
		nsel <= 2'd0;
		// Invalid entries read as the default positive nonsignaling NaN.
		fr_valid <= 0;
		fr_unsup <= 0;
		fr_nice  <= 0;
		req_pend <= 0;
	end
	else if (ce) begin
		if (fr_bank_we) begin
			fr_unsup[fr_bank_wa] <= fr_bank_uns;
			fr_nice[fr_bank_wa]  <= fr_bank_nice;
		end
		// a request the pipeline or the state machine cannot take yet
		// (the pipeline draining, or a source still being computed)
		req_pend <= rq && !(fst == F_IDLE && (pipe_issue || !pbusy) &&
		                    !(frestore_unimp && frestore_resume));
		// the multiplier's operands for an FMUL in P1 (the state machine
		// is idle whenever the pipeline holds an operation)
		if (p1_v && pipe_kind(p1_op) == 2'd2) begin
			mul_x <= s1_am;
			mul_y <= s1_bm;
		end
		// the pipeline's writeback (P6): FPSR as F_WB and F_ROUND leave it
		if (p6_v) begin
			fr_valid[p6_d] <= 1;
			fpsr[15:8]  <= p6_inx ? 8'h02 : 8'h00;    // INEX2
			if (p6_inx) fpsr[3] <= 1;                 // accrued INEX
			fpsr[27:24] <= {p6_s, p6_z, 1'b0, 1'b0};
		end
		done <= 0;
		unimp <= 0;
		unsupp <= 0;
		exc_req <= 0;

		// side ports, independent of the FSM
		if (cr_we) begin
			case (cr_sel)
				2'd0: fpiar <= cr_wdata;
				2'd1: fpsr <= cr_wdata & 32'h0FFF_FFF8;
				// 68040 FPCR keeps bits 15:0 (WinUAE fpcr_mask = 0xffff for
				// the 040; only 6888x/060 mask the low nibble)
				default: fpcr <= cr_wdata & 32'h0000_FFFF;
			endcase
			fpu_used <= 1;
		end
		if (ia_we) fpiar <= ia_wdata;
		if (bsun_req) begin
			fpsr[15] <= 1;
			fpsr[7] <= 1;
			fpu_used <= 1;
		end
		if (fm_we) begin
			fr_valid[fm_sel] <= 1;
			fpu_used <= 1;
		end
		if (pend_capture) begin : pcap
			// A released op's enabled exception was just deferred by the
			// core.  Prepare the FPSP-parseable $30 e1 frame from the
			// shadow (WinUAE fpsr_check_arithmetic_exception's 68040
			// fsave_data recipe).  The e3 class -- OVFL/UNFL/INEX from
			// the five arithmetic ops -- takes the else arm and prepares
			// the $41/$60 BUSY frame.  Both arms set fstate_unimp, so a
			// later FSAVE EXTRACTS the prepared frame instead of trapping.
			reg [7:0] pv;
			reg       st, sn;
			pv = fp_exception_vector(fpsr[15:8] & fpcr[15:8]);
			// (table 9-16 and WinUAE's 68040 recipe: an opclass 011
			// store's frame has T set, its source in ETEMP -- a SNAN
			// quieted -- and in FPTEMP; SNAN/OPERR/DZ keep a dyadic
			// operation's destination in FPTEMP; OVFL/UNFL/INEX of FMOVE
			// to a register, FABS and FNEG the intermediate result)
			st = (sh_cmd[15:13] == 3'b011);
			sn = (pv == `AP040_VEC_FP_SNAN) || (pv == `AP040_VEC_FP_OPERR) ||
			     (pv == `AP040_VEC_FP_DZ);
			if (st || sn || !arith5(sh_cmd[6:0])) begin
				fstate_cmd1  <= frame_cmd1(sh_cmd);
				fstate_cmd3  <= frame_cmd3(frame_cmd1(sh_cmd));
				fstate_stag  <= sh_stag;
				fstate_flags <= {1'b1, 1'b0, st};   // E1, T for a store
				if (st) begin
					fstate_fpt  <= sh_dst;              // the register stored
					fstate_dtag <= sh_dtag;
				end
				else if (sn) begin
					fstate_fpt  <= dyadic(sh_cmd[6:0]) ? sh_dst : 96'd0;
					fstate_dtag <= dyadic(sh_cmd[6:0]) ? sh_dtag : 3'd0;
				end
				else begin
					fstate_fpt  <= {wb_s, wb_e[14:0], 16'd0, wb_m};   // the intermediate
					fstate_dtag <= 0;
				end
				fstate_et    <= (st && pv == `AP040_VEC_FP_SNAN) ? (sh_src | {33'd0, 1'b1, 62'd0}) : sh_src;
				fstate_grs   <= (pv == `AP040_VEC_FP_SNAN) ? 3'd7 : 3'd1;
				fstate_wbte15 <= (pv == `AP040_VEC_FP_SNAN) || (st && pv == `AP040_VEC_FP_UNFL);
				fstate_busy  <= 0;
				fstate_e1    <= 1;
				fstate_unimp <= 1;
			end
			else begin
				// e3 class: OVFL/UNFL/INEX from the five arithmetic ops
				// prepare the $41/$60 BUSY frame with the internal
				// intermediate captured at F_ROUND (WinUAE's 68040 BUSY
				// recipe: e3, swizzled CMDREG3B, WBTEMP+GRS, src and dst
				// operands -- dst only for the dyadic ops)
				fstate_cmd1  <= frame_cmd1(sh_cmd);
				fstate_cmd3  <= frame_cmd3(frame_cmd1(sh_cmd));
				fstate_stag  <= sh_stag;
				fstate_et    <= sh_src;
				if ((sh_cmd[5:4] == 2'b10)) begin  // dyadic: 0x20-0x2F range
					fstate_fpt  <= sh_dst;
					fstate_dtag <= sh_dtag;
				end
				else begin
					fstate_fpt  <= 0;
					fstate_dtag <= 0;
				end
				fstate_flags <= 3'b010;   // E3
				fstate_grs   <= wb_grs;
				fstate_wbte15 <= (pv == `AP040_VEC_FP_UNFL);
				fstate_wbt   <= {wb_s, wb_e[14:0], 16'd0, wb_m};
				fstate_fpiar_c <= fpiar;
				fstate_busy  <= 1;
				fstate_e1    <= 1;        // arithmetic E-state (either tier)
				fstate_unimp <= 1;
			end
		end
		if (fp_reset) begin
			fpcr <= 0; fpsr <= 0; fpiar <= 0;
			mv_st <= 0;
			fpu_used <= 0;
			fstate_unimp <= 0;
			fstate_e1 <= 0;
			fstate_busy <= 0;
			// FRESTORE of a NULL frame returns every data register to the
			// architectural default NaN through the validity view above.
			fr_valid <= 0;
			fr_unsup <= 0;
			fr_nice  <= 0;
		end
		if (fsave_ack) begin
			fstate_unimp <= 0;
			fstate_resig <= 0;
		end
		if (frestore_idle) begin
			fpu_used <= 1;
			fstate_unimp <= 0;
			fstate_resig <= 0;
		end
		if (frestore_we) case (frestore_wf)
			4'd0:  fstate_wbt[95:64] <= frestore_wd;
			4'd1:  fstate_wbt[63:32] <= frestore_wd;
			4'd2:  fstate_wbt[31:0]  <= frestore_wd;
			4'd3:  fstate_fpiar_c    <= frestore_wd;
			4'd4:  fstate_cmd3       <= frestore_wd[31:16];
			4'd5:  begin fstate_stag <= frestore_wd[31:29]; fstate_grs <= frestore_wd[25:23]; end
			4'd6:  fstate_cmd1       <= frestore_wd[31:16];
			4'd7:  begin fstate_dtag <= frestore_wd[31:29]; fstate_wbte15 <= frestore_wd[20]; end
			4'd8:  fstate_flags      <= {frestore_wd[26], frestore_wd[25], frestore_wd[20]};
			4'd9:  fstate_fpt[95:64] <= frestore_wd;
			4'd10: fstate_fpt[63:32] <= frestore_wd;
			4'd11: fstate_fpt[31:0]  <= frestore_wd;
			4'd12: fstate_et[95:64]  <= frestore_wd;
			4'd13: fstate_et[63:32]  <= frestore_wd;
			4'd14: fstate_et[31:0]   <= frestore_wd;
			default: ;
		endcase
		if (frestore_unimp) begin
			fpu_used <= 1;
			fstate_unimp <= 1;
			// A restored frame must NOT re-signal the unimplemented trap.
			// This used to set fstate_resig, on the reading that FRESTORE
			// is how the FPSP asks to be re-entered.  It is not, and the
			// cost was severe: any handler that FSAVEs and FRESTOREs an
			// UNIMP frame -- which is what a debugger or a partial FPSP
			// does -- made the NEXT FP instruction take Line-F, even a
			// hardware opcode the FPU executes directly.  Seen on
			// hardware as $202C on FMUL.L immediately after an
			// FTWOTOX.X trap, and reproduced in t_fpu 197.
			//
			// WinUAE is unambiguous: fpu_exp_state is written by the
			// exception and read only by FSAVE/FRESTORE, never at
			// dispatch.  What FRESTORE can re-arm is fp_exp_pend, and
			// that holds an ARITHMETIC vector (50 DZ, 51 UNFL, 52 OPERR,
			// 53 OVFL, 54 SNAN) delivered as itself -- never vector 11.
			// AP040 already models that separately through fstate_e1 and
			// frestore_e1_pend below, so nothing is lost here.
			fstate_resig <= 0;
			// (the frame's fields are in place: FRESTORE wrote them)
			fstate_busy <= frestore_busy;
			// an E1 frame whose command the hardware implements is a
			// deferred ARITHMETIC exception, not an unimplemented
			// instruction: the core re-arms the pend (frestore_e1_pend)
			// and delivery happens at the next dispatch; if the enables
			// were cleared before the restore, the state simply executes
			// through (see the F_IDLE dispatch gate)
			fstate_e1 <= (fstate_flags[2] || fstate_flags[1]) &&
			             op_in_hw(fr_cmd_op);
		end

		nsel <= 2'd0;
		case (fst)
			F_IDLE: if (frestore_unimp && frestore_resume) begin
				// The frame, not the current register bank, supplies BOTH operands.
				// FPIAR/FPCR are separately restored by software; do not replace
				// FPIAR with the FRESTORE instruction address or frame FPIARCU.
				fpsr[15:8] <= 0;
				fstate_unimp <= 0;
				fstate_resig <= 0;
				fstate_e1 <= 0;
				fstate_busy <= 0;
				r_resume <= 1;
				r_unimp <= 0;
				r_ae7 <= fpsr[7];
				r_op <= fr_cmd_op;
				r_fmt <= 3'd2; // already converted to extended, even opclass 0
				r_dst <= fstate_cmd1[9:7];
				sh_cmd <= fstate_cmd1;
				{a_s, a_e, a_m, a_t} <= unpack_x(fstate_et[95],
				    restore_et_negative ? 15'd0 : fstate_et[94:80], fstate_et[63:0]);
				{b_s, b_e, b_m, b_t} <= unpack_x(fstate_fpt[95],
				    restore_fpt_negative ? 15'd0 : fstate_fpt[94:80], fstate_fpt[63:0]);
				sh_v <= {fstate_et[63:0], 3'd0};
				sh_cnt <= restore_shift(frestore_et15, fstate_et[94:80]);
				loop_n <= restore_shift(frestore_fpt15, fstate_fpt[94:80]);
				sh_ret <= F_RESTORE_A;
				begin fst <= F_SHR; wb_r <= (F_SHR == F_WB); acc_r <= 1'b0; end
			end
			else if (pipe_issue) begin
				// into the pipeline: complete as far as the core is concerned
				// (pdone, this cycle)
				fpu_used <= 1;
			end
			else if (rq && !pbusy) begin
				if (fstate_unimp && !fstate_e1 && fstate_resig) begin
					// A restored exception frame remains pending until FSAVE.
					// Re-enter the software package without destroying its state.
					unimp <= 1;
					fpu_used <= 1;
				end
				else begin : new_fp_command
				// a normal dispatch consumes any leftover frame state: a
				// delivered-but-unsaved pend frame, or a restored e1 state
				// whose enables were cleared before the restore
				fstate_e1 <= 0;
				fstate_unimp <= 0;
				fstate_resig <= 0;
				fstate_busy <= 0;
				// FPSR exception status is per instruction.  The accrued
				// exception byte is intentionally retained until software
				// writes FPSR.  FMOVECR (opclass 010 fmt 7) faults before
				// WinUAE's fpsr_clear_status runs, so it must leave the
				// exception byte untouched; nonexisting opmodes never reach
				// the FPU at all (classified in the core).
				if (!(op_class == 3'b010 && src_fmt == 3'd7))
					fpsr[15:8] <= 8'd0;
				r_fmt <= src_fmt;
				r_ae7 <= fpsr[7];
				r_dst <= dst_r;
				r_op <= opmode;
				// Preserve opclass and source register/format before execution.
				sh_cmd <= {op_class, src_fmt, dst_r, opmode};
				r_din <= din;
				r_unimp <= 0;
				r_resume <= 0;
				// Retain the destination operand once at dispatch.  Later FPU
				// states use this existing exception shadow instead of building
				// another 80-bit asynchronous FP-register read port around r_dst.
				sh_dst  <= {fr_dst_s, fr_dst_e, 16'd0, fr_dst_m};
				sh_dtag <= frame_tag_x(fr_dst_e, fr_dst_m);
				{b_s, b_e, b_m, b_t} <=
					unpack_x(fr_dst_s, fr_dst_e, fr_dst_m);
				// the operands as a datatype or unimplemented-instruction
				// frame records them (F_CAP): the source raw -- a register's
				// image; a memory operand in extended layout, a packed one
				// in its split layout -- with its tag (an S/D denormal's is
				// set where it is found); a store's register in both
				if (op_class == 3'b011) begin
					sh_dst  <= {fr_src_s, fr_src_e, 16'd0, fr_src_m};
					sh_dtag <= frame_tag_x(fr_src_e, fr_src_m);
				end
				if (op_class == 3'b010 && src_fmt == 3'd7) begin
					sh_src  <= 96'd0;
					sh_stag <= 3'd1;
				end
				else if (op_class == 3'b010 && src_fmt == 3'd3) begin
					sh_src  <= {32'd0, din[63:0]};
					sh_stag <= 3'd7;
					sh_dst  <= {32'd0, din[63:32], din[95:64]};
					sh_dtag <= 3'd0;
				end
				else if (op_class == 3'b010) begin
					sh_src  <= {din[95], din[94:80], 16'd0, din[63:0]};
					sh_stag <= frame_tag_x(din[94:80], din[63:0]);
				end
				else begin
					sh_src  <= {fr_src_s, fr_src_e, 16'd0, fr_src_m};
					sh_stag <= frame_tag_x(fr_src_e, fr_src_m);
					r_stag  <= frame_tag_x(fr_src_e, fr_src_m);
				end
				if (op_class == 3'b011) begin
					// FMOVE FPn,<ea>: packed decimal and denormal/unnormal
					// register contents are unsupported data types
					// OPCLASS 011: T set, and the register value goes to
					// BOTH ETEMP and FPTEMP (WinUAE marks the FPTEMP/dtag
					// half undocumented but writes it).
					if (src_fmt == 3'd3 || src_fmt == 3'd7) begin
						cap_cmd <= {op_class, src_fmt, dst_r, opmode};
						cap_unimp <= 0; cap_t <= 1; cap_e1 <= 1; cap_pk <= 0;   // T, packed -> E1
						begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
					end
					else if (fr_src_unsup) begin
						cap_cmd <= {op_class, src_fmt, dst_r, opmode};
						cap_unimp <= 0; cap_t <= 1; cap_e1 <= 0; cap_pk <= 0;   // T, not packed
						begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
					end
					else begin
						{a_s, a_e, a_m, a_t} <=
							unpack_x(fr_src_s, fr_src_e, fr_src_m);
						begin fst <= F_SRC; wb_r <= (F_SRC == F_WB); acc_r <= 1'b0; end   // F_SRC routes stores via r_op = STORE
						r_op <= 7'h7F;  // internal: store
					end
				end
				else if (op_class == 3'b010 && src_fmt == 3'd7) begin : save_fmovecr
					cap_cmd <= {op_class, src_fmt, dst_r, opmode};
					cap_unimp <= 1; cap_pk <= 0;
					begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
				end
				else if (!op_in_hw(opmode)) begin : save_unimp_command
					if (op_class == 3'b000) begin
						cap_cmd <= {op_class, src_fmt, dst_r, opmode};
						cap_unimp <= 1; cap_pk <= 0;
						begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
					end
					else if (src_fmt == 3'd3) begin
						// Packed operands use the same split payload in the short
						// unimplemented frame as in a datatype BUSY frame.
						cap_cmd <= {op_class, src_fmt, dst_r, opmode};
						cap_unimp <= 1; cap_pk <= 1;   // E1 tells get_op to unpack
						begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
					end
					else begin
						// Reuse the normal sequential source converter instead of
						// synthesizing a second wide combinational conversion path.
						r_unimp <= 1;
						a_t <= T_NUM;
						begin fst <= F_SRC; wb_r <= (F_SRC == F_WB); acc_r <= 1'b0; end
					end
				end
				else if (op_class == 3'b000) begin
					if (fr_src_unsup) begin
						// OPCLASS 000: source in ETEMP; a dyadic op also
						// carries its destination in FPTEMP
						cap_cmd <= {op_class, src_fmt, dst_r, opmode};
						cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
						begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
					end
					else begin : disp_rr
					reg [83:0] ua;
					ua = unpack_x(fr_src_s, fr_src_e, fr_src_m);
					{a_s, a_e, a_m, a_t} <= ua;
					// a binary operation on a non-NaN source needs nothing
					// from F_EXEC but its bookkeeping (no SNaN to quiet, no
					// sign to change): straight to F_BIN
					if (bin_op(opmode) && ua[1:0] != T_NAN) begin
						op_kind <= bin_kind(opmode);
						grs <= 3'd0;
						e_w <= $signed({ua[82], ua[82:66]});
						begin fst <= F_BIN; wb_r <= (F_BIN == F_WB); acc_r <= 1'b0; end
					end
					else if ((opmode == 7'h00 || opmode == 7'h40 || opmode == 7'h44) &&
					         ua[1:0] != T_NAN) begin
						// a move: F_EXEC's work for it, here (GRS, the
						// working exponent; F_ROUND records the exception
						// shadow as it starts: mv_st), then F_ROUND
						mv_st <= 1'b1;
						grs <= 3'd0;
						e_w <= $signed({ua[82], ua[82:66]});
						begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
					end
					else begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
					end
				end
				else begin
					// opclass 010, memory source: packed decimal (fmt 3) is
					// an unsupported data type.  FMOVECR was handled above so
					// its exception could retain a complete state frame.
					if (src_fmt == 3'd3) begin
						// The core has fetched all twelve bytes.  The 040 stores
						// the first packed longword in FPTEMP_LO, not ETEMP_EX;
						// FPSP get_op copies it back before calling decbin.
						// Match Previous fp_unimp_datatype's split frame layout
						// (the dispatch's sh_src/sh_dst for a packed operand).
						cap_cmd <= {op_class, src_fmt, dst_r, opmode};
						cap_unimp <= 0; cap_t <= 0; cap_e1 <= 1; cap_pk <= 0;   // packed -> E1, stag 7
						begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
					end
					else begin
						a_t <= T_NUM;   // provisional; F_SRC classifies
						begin fst <= F_SRC; wb_r <= (F_SRC == F_WB); acc_r <= 1'b0; end
					end
				end
				end
			end

			F_SRC: begin : f_src
				if (r_op == 7'h7F) begin : f_store
					// register store: X is raw; S/D pack directly (fixed
					// rounding bit positions); B/W/L go through the shifter
					case (r_fmt)
						3'd2: begin
							// raw X pass-through can set no status bits, so
							// it alone skips the F_STDONE enabled-trap check
							dout <= {a_s, a_e[14:0], 16'd0, a_m};
							done <= 1; fpu_used <= 1;
							begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end
						end
						3'd1, 3'd5: begin : pk_sd
							// single or double: one form for both (sd_out); the
							// single's fields are the double's upper bits
							reg signed [17:0] den_sh;
							if (a_t != T_NUM) begin
								// NaN payload passes through commonNaN form:
								// the quiet bit is FORCED in the output and a
								// signaling source raises SNAN
								dout <= sd_out;
								if (a_t == T_NAN && !a_m[62]) begin
									fpsr[14] <= 1;
									fpsr[7]  <= 1;
								end
								fpu_used <= 1; if (fpcr[15:8] == 8'd0) begin done <= 1; begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end end else begin fst <= F_STDONE; wb_r <= (F_STDONE == F_WB); acc_r <= 1'b0; end  // no enabled exception possible: done now
							end
							else if (sd_den) begin
								// Shift into the IEEE denormal range, retaining
								// G/R/S for the selected rounding mode.
								den_sh = (sd_isS ? -18'sd126 : -18'sd1022) - sd_sE;
								sh_v <= {a_m, 3'd0};
								sh_cnt <= (den_sh > 18'sd127) ? 7'd127 : den_sh[6:0];
								sh_ret <= F_PACKS;
								begin fst <= F_SHR; wb_r <= (F_SHR == F_WB); acc_r <= 1'b0; end
							end
							else begin
								dout <= sd_out;
								if (sd_ov) begin
									// (INEX2 only for bits actually lost, as
									// softfloat's 68K roundAndPackFloat32 raises
									// inexact; accrued INEX comes with OVFL)
									fpsr[12] <= 1; fpsr[6] <= 1;
									if (sd_inx) fpsr[9] <= 1;
									fpsr[3] <= 1;
								end
								else if (sd_inx) begin fpsr[9] <= 1; fpsr[3] <= 1; end
								fpu_used <= 1; if (fpcr[15:8] == 8'd0) begin done <= 1; begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end end else begin fst <= F_STDONE; wb_r <= (F_STDONE == F_WB); acc_r <= 1'b0; end  // no enabled exception possible: done now
							end
						end
						default: begin : pk_i
							// B/W/L: shift the mantissa so the integer part
							// lands in sh_v[66:3]; E = a_e - 16383 is the
							// position of the integer bit
							reg signed [17:0] sE;
							reg [63:0] qm;
							pk_isz <= (r_fmt == 3'd0) ? 2'd2 :
							          (r_fmt == 3'd4) ? 2'd1 : 2'd0;
							pk_neg <= a_s;
								sE = $signed({a_e[16], a_e}) - 18'sd16383;
							qm = a_m | 64'h4000_0000_0000_0000;
							if (a_t == T_ZERO) begin
								sh_v <= 0; sh_cnt <= 0; begin fst <= F_PACKI; wb_r <= (F_PACKI == F_WB); acc_r <= 1'b0; end
							end
							else if (a_t == T_NAN) begin
								// floatx80_to_int32/16/8: the QUIETED NaN's
								// top payload bits are stored; OPERR is only
								// raised for an already-quiet NaN, a
								// signaling one raises SNAN instead
								dout <= {(r_fmt == 3'd0) ? qm[63:32] :
								         (r_fmt == 3'd4) ? {qm[63:48], 16'd0} :
								                           {qm[63:56], 24'd0}, 64'd0};
								if (!a_m[62]) begin
									fpsr[14] <= 1;
									fpsr[7]  <= 1;
								end
								else begin
									fpsr[13] <= 1;
									fpsr[7]  <= 1;
								end
								fpu_used <= 1; if (fpcr[15:8] == 8'd0) begin done <= 1; begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end end else begin fst <= F_STDONE; wb_r <= (F_STDONE == F_WB); acc_r <= 1'b0; end  // no enabled exception possible: done now
							end
							else if (a_t == T_INF || sE > 18'sd62) begin
								// infinity or far out of range: OPERR with
								// sign-dependent saturation
								sh_v <= 67'h7FFFFFFFFFFFFFFFF;
								sh_cnt <= 7'd127;   // marker
								begin fst <= F_PACKI; wb_r <= (F_PACKI == F_WB); acc_r <= 1'b0; end
							end
							else if (sE < -18'sd1) begin
								sh_v <= {64'd0, 1'b0, 1'b0, 1'b1};  // sticky only
								sh_cnt <= 0; begin fst <= F_PACKI; wb_r <= (F_PACKI == F_WB); acc_r <= 1'b0; end
							end
							else begin
								sh_v <= {a_m, 3'd0};
								sh_cnt <= 7'd63 - sE[6:0];
								sh_ret <= F_PACKI;
								begin fst <= F_SHR; wb_r <= (F_SHR == F_WB); acc_r <= 1'b0; end
							end
						end
					endcase
				end
				else begin : f_load
					// memory operand to working format
					case (r_fmt)
						3'd0: begin : cv_l
							a_s <= r_din[95];
							a_t <= (r_din[95:64] == 32'd0) ? T_ZERO : T_NUM;
							a_m <= unp_m;
							a_e <= 17'd16383 + 17'd31;   // integer bit at 63
							r_stag <= (r_din[95:64] == 32'd0) ? 3'd1 : 3'd0;
							begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
						end
						3'd4: begin : cv_w
							a_s <= r_din[95];
							a_t <= (r_din[95:80] == 16'd0) ? T_ZERO : T_NUM;
							a_m <= unp_m;
							a_e <= 17'd16383 + 17'd15;
							r_stag <= (r_din[95:80] == 16'd0) ? 3'd1 : 3'd0;
							begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
						end
						3'd6: begin : cv_b
							a_s <= r_din[95];
							a_t <= (r_din[95:88] == 8'd0) ? T_ZERO : T_NUM;
							a_m <= unp_m;
							a_e <= 17'd16383 + 17'd7;
							r_stag <= (r_din[95:88] == 8'd0) ? 3'd1 : 3'd0;
							begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
						end
						3'd1: begin : cv_s
							a_s <= r_din[95];
							if (r_din[94:87] == 8'hFF) begin
								r_stag <= (r_din[86:64] != 0) ? 3'd3 : 3'd2;
								a_t <= (r_din[86:64] != 0) ? T_NAN : T_INF;
								a_e <= 17'h07FFF;
								// 68040 extended NaNs are unnormal: preserve the
								// source payload and leave the explicit integer bit clear.
								// (The quiet bit is bit 62 and is set below for an
								// SNaN.)  Infinity is canonicalized during writeback.
								a_m <= unp_m;
								begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
							end
							else if (r_din[94:87] == 8'd0) begin
								if (r_din[86:64] != 0) begin
									// denormalized single: an unsupported
									// data type (vector 55), not an
									// unimplemented instruction
									if (r_unimp) begin
										a_t <= T_NUM; a_e <= 17'd16257;
										a_m <= unp_m;
										r_stag <= 3'd5; begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
									end
									else begin
										sh_stag <= 3'd5;
										cap_cmd <= {3'b010, r_fmt, r_dst, r_op};
										cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
										begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
									end
								end
								else begin
									r_stag <= 3'd1;
									a_t <= T_ZERO;
									a_m <= unp_m;        // (zero: exponent and fraction 0)
									a_e <= 0;
									begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
								end
							end
							else begin
								r_stag <= 3'd0;
								a_t <= T_NUM;
								a_e <= {9'd0, r_din[94:87]} + 17'd16256; // -127+16383
								a_m <= unp_m;
								// a binary operation: straight to F_BIN (as
								// for a register source)
								if (bin_op(r_op) && !r_unimp) begin
									op_kind <= bin_kind(r_op);
									grs <= 3'd0;
									e_w <= $signed({9'd0, r_din[94:87]}) + 18'sd16256;
									begin fst <= F_BIN; wb_r <= (F_BIN == F_WB); acc_r <= 1'b0; end
								end
								else if ((r_op == 7'h00 || r_op == 7'h40 || r_op == 7'h44) && !r_unimp) begin
									// a move: F_EXEC's work for it, here
									// (as for a register source; the shadow
									// as F_ROUND starts)
									mv_st <= !r_resume;
									grs <= 3'd0;
									e_w <= $signed({9'd0, r_din[94:87]}) + 18'sd16256;
									begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
								end
								else begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
							end
						end
						3'd5: begin : cv_d
							a_s <= r_din[95];
							if (r_din[94:84] == 11'h7FF) begin
								r_stag <= (r_din[83:32] != 0) ? 3'd3 : 3'd2;
								a_t <= (r_din[83:32] != 0) ? T_NAN : T_INF;
								a_e <= 17'h07FFF;
								// Match the 68040 NaN encoding: bit 63 remains clear;
								// only the payload (and, when needed, quiet bit 62) is
								// carried into the extended significand.
								a_m <= unp_m;
								begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
							end
							else if (r_din[94:84] == 11'd0) begin
								if (r_din[83:32] != 0) begin
									// denormalized double: likewise the
									// data type trap, vector 55
									if (r_unimp) begin
										a_t <= T_NUM; a_e <= 17'd15361;
										a_m <= unp_m;
										r_stag <= 3'd5; begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
									end
									else begin
										sh_stag <= 3'd5;
										cap_cmd <= {3'b010, r_fmt, r_dst, r_op};
										cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
										begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
									end
								end
								else begin
									r_stag <= 3'd1;
									a_t <= T_ZERO;
									a_m <= unp_m;        // (zero: exponent and fraction 0)
									a_e <= 0;
									begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
								end
							end
							else begin
								r_stag <= 3'd0;
								a_t <= T_NUM;
								a_e <= {6'd0, r_din[94:84]} + 17'd15360; // -1023+16383
								a_m <= unp_m;
								if (bin_op(r_op) && !r_unimp) begin
									op_kind <= bin_kind(r_op);
									grs <= 3'd0;
									e_w <= $signed({7'd0, r_din[94:84]}) + 18'sd15360;
									begin fst <= F_BIN; wb_r <= (F_BIN == F_WB); acc_r <= 1'b0; end
								end
								else if ((r_op == 7'h00 || r_op == 7'h40 || r_op == 7'h44) && !r_unimp) begin
									// a move: F_EXEC's work for it, here
									mv_st <= !r_resume;
									grs <= 3'd0;
									e_w <= $signed({7'd0, r_din[94:84]}) + 18'sd15360;
									begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
								end
								else begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
							end
						end
						default: begin : cv_x
							if (unsupported_x(r_din[94:80], r_din[63:0])) begin
								if (r_unimp) begin
									{a_s, a_e, a_m, a_t} <=
										unpack_x(r_din[95], r_din[94:80], r_din[63:0]);
									r_stag <= 3'd4; begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
								end
								else begin
									cap_cmd <= {3'b010, r_fmt, r_dst, r_op};
									cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
									begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
								end
							end
							else if (r_unimp) begin
								r_stag <= frame_tag_x(r_din[94:80], r_din[63:0]);
								{a_s, a_e, a_m, a_t} <=
									unpack_x(r_din[95], r_din[94:80], r_din[63:0]);
								begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
							end
							else begin : cv_xs
								// a supported extended operand is normalized
								// already (unnormals and denormals trapped
								// above): F_NORM has nothing to do, so the
								// operation starts as for a register source
								reg [83:0] ua;
								reg  [2:0] tg;
								ua = unpack_x(r_din[95], r_din[94:80], r_din[63:0]);
								tg = frame_tag_x(r_din[94:80], r_din[63:0]);
								r_stag <= tg;
								{a_s, a_e, a_m, a_t} <= ua;
								if (bin_op(r_op) && ua[1:0] != T_NAN) begin
									op_kind <= bin_kind(r_op);
									grs <= 3'd0;
									e_w <= $signed({ua[82], ua[82:66]});
									begin fst <= F_BIN; wb_r <= (F_BIN == F_WB); acc_r <= 1'b0; end
								end
								else if ((r_op == 7'h00 || r_op == 7'h40 || r_op == 7'h44) &&
								         ua[1:0] != T_NAN) begin
									mv_st <= !r_resume;
									grs <= 3'd0;
									e_w <= $signed({ua[82], ua[82:66]});
									begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
								end
								else begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
							end
						end
					endcase
				end
			end

			F_NLZ: begin
				// the normalizer's count, registered; its input select
				// (nsel) stays as the entering state set it
				nlz_q <= norm_lz;
				nsel  <= nsel;
				begin fst <= nz_ret; wb_r <= (nz_ret == F_WB); acc_r <= acc_of(nz_ret); end
			end

			F_CAP: begin
				// the datatype or unimplemented-instruction frame, from the
				// shadow registers the deciding site prepared (one form for
				// every site: the frame's operand multiplexers do not grow
				// with the sites)
				if (cap_unimp) begin
					capture_unimp(cap_cmd, sh_src, sh_stag, sh_dst, sh_dtag);
					if (cap_pk) fstate_flags <= 3'b100;   // E1 tells get_op to unpack
				end
				else begin
					unsupp <= 1;
					capture_datatype(cap_cmd, sh_src, sh_stag, sh_dst, sh_dtag, cap_t, cap_e1);
				end
				begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end
			end

			F_RESTORE_A: begin
				// Ignore discarded GRS: exponent extension is a truncating
				// format conversion, not an arithmetic rounding operation.
				a_m <= sh_v[66:3];
				if (a_t == T_NUM && sh_v[66:3] == 0) a_t <= T_ZERO;
				sh_src <= {a_s, a_e[14:0], 16'd0, sh_v[66:3]};
				r_stag <= frame_tag_x(a_e[14:0], sh_v[66:3]);
				sh_stag <= frame_tag_x(a_e[14:0], sh_v[66:3]);
				sh_v <= {b_m, 3'd0};
				sh_cnt <= loop_n;
				sh_ret <= F_RESTORE_B;
				begin fst <= F_SHR; wb_r <= (F_SHR == F_WB); acc_r <= 1'b0; end
			end

			F_RESTORE_B: begin
				b_m <= sh_v[66:3];
				if (b_t == T_NUM && sh_v[66:3] == 0) b_t <= T_ZERO;
				sh_dst <= {b_s, b_e[14:0], 16'd0, sh_v[66:3]};
				sh_dtag <= frame_tag_x(b_e[14:0], sh_v[66:3]);
				begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_RESTORE_N; acc_r <= 1'b0; end
				nsel <= 2'b01;
			end

			F_RESTORE_N: begin
				// Use signed working exponents for true extended denormals.
				// The normal instruction path still traps on these operands.
				if (b_t == T_NUM) begin
					b_m <= norm_shifted[66:3];
					b_e <= b_e - {10'd0, nlz_q};
				end
				begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM; acc_r <= 1'b0; end
			end

			F_NORM: begin
				if (a_t != T_NUM) begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
				else begin
					// a zero significand normalizes to zero: a_m takes the
					// shifter's output either way, the zero test only
					// decides the class and exponent (a timing path)
					a_m <= norm_shifted[66:3];
					if (a_m == 64'd0) begin
						a_t <= T_ZERO; a_e <= 0;
					end
					else a_e <= a_e - {10'd0, nlz_q};
					begin fst <= F_EXEC; wb_r <= (F_EXEC == F_WB); acc_r <= 1'b0; end
				end
			end

			F_EXEC: begin
				// operand shadow for a possible deferred-exception frame
				if (!r_resume) begin
					sh_src  <= {a_s, a_e[14:0], 16'd0, a_m};
					sh_stag <= r_stag;
				end
				if (r_unimp) begin
					// (sh_src/sh_stag written above, this cycle)
					cap_cmd <= sh_cmd;
					cap_unimp <= 1; cap_pk <= 0;
					r_unimp <= 0;
					begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
				end
				// FCMP evaluates its destination without passing through
				// F_BIN, so its unsupported-datatype check lives here -- and
				// it must precede the SNaN bookkeeping: the datatype fault
				// is taken before the arithmetic ever inspects a NaN, so the
				// status byte stays clean.
				else if (!r_resume && r_op == 7'h38 &&
				    unsupported_x(b_e[14:0], b_m)) begin
					cap_cmd <= sh_cmd;
					cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
					begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
				end
				else begin
				// Quiet signaling NaNs after recording SNAN.  Source and
				// FCMP destination checks occur before result dispatch so an
				// enabled SNAN is observed in F_WB and inhibits writeback.
				if (a_t == T_NAN && !a_m[62]) begin
					a_m[62] <= 1;
					fpsr[14] <= 1;
					fpsr[7] <= 1;
				end
				if (r_op == 7'h38 && b_t == T_NAN && !b_m[62]) begin
					fpsr[14] <= 1;
					fpsr[7] <= 1;
				end
				case (r_op)
					7'h18, 7'h58, 7'h5C:
						if (a_t != T_NAN) a_s <= 0;                       // FABS
					7'h1A, 7'h5A, 7'h5E:
						if (a_t != T_NAN) a_s <= ~a_s;                    // FNEG
					default: ;
				endcase
				grs <= 3'd0;
				e_w <= $signed({a_e[16], a_e});
				case (r_op)
					7'h38, 7'h3A: begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end               // FCMP/FTST
					7'h00, 7'h40, 7'h44,
					7'h18, 7'h58, 7'h5C,
					7'h1A, 7'h5A, 7'h5E: begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end     // move class
					7'h04, 7'h41, 7'h45: begin               // FSQRT
						op_kind <= 4'd4;
						begin fst <= F_BIN; wb_r <= (F_BIN == F_WB); acc_r <= 1'b0; end
					end
					default: begin : ex_bin
						op_kind <= (r_op == 7'h23 || r_op == 7'h27 ||
						            r_op == 7'h63 || r_op == 7'h67) ? 4'd2 :
						           (r_op == 7'h20 || r_op == 7'h24 ||
						            r_op == 7'h60 || r_op == 7'h64) ? 4'd3 : 4'd1;
						begin fst <= F_BIN; wb_r <= (F_BIN == F_WB); acc_r <= 1'b0; end
					end
				endcase
				end
			end

			F_BIN: begin : f_bin
				reg        s_a;
				// NaNs went through F_EXEC, which saved the source before
				// quieting it. Keep that image for SNAN and datatype recovery.
				if (!r_resume && a_t != T_NAN) begin
					sh_src  <= {a_s, a_e[14:0], 16'd0, a_m};
					sh_stag <= r_stag;
				end
				// FSUB family: fold the source sign
				s_a = (op_kind == 4'd1 &&
				       (r_op == 7'h28 || r_op == 7'h68 || r_op == 7'h6C))
				      ? ~a_s : a_s;
				if (!r_resume && op_kind != 4'd4 &&
				    unsupported_x(b_e[14:0], b_m)) begin
					// The destination datatype fault is taken with a clean
					// status byte: undo the source-SNaN record F_EXEC made a
					// cycle earlier (the exception byte was zeroed at
					// dispatch; r_ae7 holds the prior accrued-IOP bit)
					fpsr[14] <= 0;
					fpsr[7]  <= r_ae7;
					cap_cmd <= sh_cmd;
					cap_unimp <= 0; cap_t <= 0; cap_e1 <= 0; cap_pk <= 0;
					begin fst <= F_CAP; wb_r <= (F_CAP == F_WB); acc_r <= 1'b0; end
				end
				else begin
				if (op_kind != 4'd4 && b_t == T_NAN && !b_m[62]) begin
					fpsr[14] <= 1;
					fpsr[7] <= 1;
					b_m[62] <= 1;
				end
				if (a_t == T_NAN || (op_kind != 4'd4 && b_t == T_NAN)) begin
					// NaN propagation, destination NaN preferred
					if (op_kind != 4'd4 && b_t == T_NAN) begin
						a_s <= b_s;
						a_e <= b_e;
						a_m <= b_m | 64'h4000_0000_0000_0000;
					end
					else a_m <= a_m | 64'h4000_0000_0000_0000;
					a_t <= T_NAN;
					begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
				end
				else case (op_kind)
					4'd4: begin : bin_sqrt
						if (a_t == T_ZERO) begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end         // sqrt(+-0)=+-0
						else if (a_s) begin
							// negative: operand error, POSITIVE default NaN
							// (floatx80_default_nan has the sign bit clear)
							a_t <= T_NAN;
							a_s <= 0;
							a_m <= 64'hFFFF_FFFF_FFFF_FFFF;
							a_e <= 17'h07FFF;
							fpsr[13] <= 1;
							fpsr[7] <= 1;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (a_t == T_INF) begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						else begin : sq_go
							reg signed [17:0] sE;
							sE = $signed({a_e[16], a_e}) - 18'sd16383;
							e_w <= (sE >>> 1) + 18'sd16383;
							if (a_m == 64'h8000_0000_0000_0000 && !sE[0]) begin
								// Exact square root of an even power of two.
								grs <= 0;
								begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
							end
							else begin
								srad <= sE[0] ? {a_m, 2'b00} : {1'b0, a_m, 1'b0};
								srem <= 0;
								qv <= 0;
								loop_n <= 0;
								begin fst <= F_SQRTL; wb_r <= (F_SQRTL == F_WB); acc_r <= 1'b1; end
							end
						end
					end
					4'd1: begin : bin_add
						if (a_t == T_INF || b_t == T_INF) begin
							if (a_t == T_INF && b_t == T_INF &&
							    s_a != b_s) begin
								a_t <= T_NAN;      // inf - inf: default NaN
								a_s <= 0;
								a_m <= 64'hFFFF_FFFF_FFFF_FFFF;
								a_e <= 17'h07FFF;
								fpsr[13] <= 1;
								fpsr[7] <= 1;
							end
							else begin
								// both-inf same sign returns the DEST operand
								// raw (68040 has addsub_swap_inf clear); a
								// lone infinity keeps its own mantissa bits
								a_s <= (a_t == T_INF && b_t != T_INF) ? s_a : b_s;
								a_m <= (b_t == T_INF) ? b_m : a_m;
								a_t <= T_INF;
							end
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (a_t == T_ZERO && b_t == T_ZERO) begin
							a_s <= (s_a == b_s) ? s_a : (rnd_mode == 2'b10);
							a_t <= T_ZERO;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (a_t == T_ZERO) begin
							a_s <= b_s;
							a_e <= b_e;
							a_m <= b_m;
							a_t <= T_NUM;
							e_w <= $signed({b_e[16], b_e});
							begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
						end
						else if (b_t == T_ZERO) begin
							a_s <= s_a;
							e_w <= $signed({a_e[16], a_e});
							begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
						end
						else begin : bin_addnum
							reg        aswap;
							reg [16:0] d;
							aswap = ($signed(a_e) > $signed(b_e)) ||
							        (a_e == b_e && a_m > b_m);
							eff_sub <= (s_a != b_s);
							if (aswap) begin
								b_s <= s_a; b_e <= a_e; b_m <= a_m;
								a_s <= b_s; a_e <= b_e; a_m <= b_m;
								d = a_e - b_e;
								e_w <= $signed({a_e[16], a_e});
							end
							else begin
								a_s <= s_a;
								d = b_e - a_e;
								e_w <= $signed({b_e[16], b_e});
							end
							sh_v <= {aswap ? b_m : a_m, 3'd0};
							sh_cnt <= (d > 17'd66) ? 7'd67 : d[6:0];
							sh_ret <= F_ADDX;
							begin fst <= F_SHR; wb_r <= (F_SHR == F_WB); acc_r <= 1'b1; end
						end
					end
					4'd2: begin : bin_mul
						reg [63:0] am_eff, bm_eff;
						reg        chop, a_p2, b_p2;
						chop   = (r_op == 7'h27);
						am_eff = chop ? (a_m & 64'hFFFF_FF00_0000_0000) : a_m;
						bm_eff = chop ? (b_m & 64'hFFFF_FF00_0000_0000) : b_m;
						// an exact power of two (after the chop): the compare
						// in parts on the operand, beside the chop's select
						a_p2 = (a_m[63:40] == 24'h80_0000) && (chop || a_m[39:0] == 40'd0);
						b_p2 = (b_m[63:40] == 24'h80_0000) && (chop || b_m[39:0] == 40'd0);
						// Latch chopped operands before testing the fast path.  The
						// old code used nonblocking masks and then immediately
						// tested the unmasked values, bypassing the chop.
						a_m <= am_eff;
						b_m <= bm_eff;
						// FSGLMUL chops both mantissas to single precision
						// before multiplying (the result is single rounded)
						if (a_t == T_INF || b_t == T_INF) begin
							if (a_t == T_ZERO || b_t == T_ZERO) begin
								a_t <= T_NAN;      // 0 * inf: default NaN
								a_s <= 0;
								a_m <= 64'hFFFF_FFFF_FFFF_FFFF;
								a_e <= 17'h07FFF;
								fpsr[13] <= 1;
								fpsr[7] <= 1;
							end
							else begin
								// pass-through infinity keeps the raw
								// mantissa of the winning operand (dest
								// first, matching floatx80_mul's order)
								a_t <= T_INF;
								a_s <= a_s ^ b_s;
								a_m <= (b_t == T_INF) ? b_m : a_m;
							end
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (a_t == T_ZERO || b_t == T_ZERO) begin
							a_t <= T_ZERO;
							a_s <= a_s ^ b_s;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else begin
							a_s <= a_s ^ b_s;
							e_w <= $signed({a_e[16], a_e}) +
							       $signed({b_e[16], b_e}) - 18'sd16383;
							if (a_p2 || b_p2) begin
								// Multiplication by an exact power of two only changes
								// sign/exponent and needs no iterative multiply.
								a_m <= a_p2 ? bm_eff : am_eff;
								grs <= 0;
								begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
							end
							else begin
								acc_hi <= 0;
								acc_lo <= 0;
								loop_n <= 0;
								mul_x  <= am_eff;
								mul_y  <= bm_eff;
								begin fst <= F_MULT; wb_r <= (F_MULT == F_WB); acc_r <= 1'b1; end
							end
						end
					end
					default: begin : bin_div
						// FSGLDIV divides the FULL-width mantissas and only
						// rounds the quotient to single precision: unlike
						// FSGLMUL there is no operand chop (floatx80_sgldiv
						// has no aSig/bSig masking).
						reg [63:0] am_eff, bm_eff;
						am_eff = a_m;
						bm_eff = b_m;
						// FPn = FPn / source (b = dividend, a = divisor).
						// Special-case order follows floatx80_div: the
						// dividend-infinity case comes BEFORE the divisor-zero
						// check, so inf/0 is a clean infinity with NO DZ.
						// 0/0 and inf/inf produce the default NaN: positive
						// sign, all-ones mantissa (floatx80_default_nan).
						if (a_t == T_INF && b_t == T_INF) begin
							a_t <= T_NAN;
							a_s <= 0;
							a_m <= 64'hFFFF_FFFF_FFFF_FFFF;
							a_e <= 17'h07FFF;
							fpsr[13] <= 1;
							fpsr[7] <= 1;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (a_t == T_ZERO && b_t == T_ZERO) begin
							a_t <= T_NAN;
							a_s <= 0;
							a_m <= 64'hFFFF_FFFF_FFFF_FFFF;
							a_e <= 17'h07FFF;
							fpsr[13] <= 1;
							fpsr[7] <= 1;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (b_t == T_INF) begin
							// dividend infinity passes through with its raw
							// mantissa bits (the 040 does not clear the
							// integer bit; that is 68060 behaviour)
							a_t <= T_INF;
							a_s <= a_s ^ b_s;
							a_m <= b_m;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (a_t == T_ZERO) begin
							// finite dividend / zero: DZ + created infinity
							// (all-zero mantissa, floatx80_default_infinity)
							a_t <= T_INF;
							a_s <= a_s ^ b_s;
							a_m <= 64'd0;
							fpsr[10] <= 1;
							fpsr[4] <= 1;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else if (b_t == T_ZERO || a_t == T_INF) begin
							a_t <= T_ZERO;
							a_s <= a_s ^ b_s;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else begin
							a_s <= a_s ^ b_s;
							e_w <= $signed({b_e[16], b_e}) -
							       $signed({a_e[16], a_e}) + 18'sd16383;
							if (am_eff == 64'h8000_0000_0000_0000 || bm_eff == am_eff) begin
								// Division by a power of two, or equal normalized
								// significands, is exact after exponent adjustment.
								a_m <= (am_eff == 64'h8000_0000_0000_0000) ? bm_eff :
								       64'h8000_0000_0000_0000;
								grs <= 0;
								begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
							end
							else begin
								// remainder = dividend (chopped for FSGLDIV,
								// which the b_m write below cannot supply yet)
								acc_hi <= {1'b0, bm_eff};
								qv <= 0;
								loop_n <= 0;
								begin fst <= F_DIVL; wb_r <= (F_DIVL == F_WB); acc_r <= 1'b1; end
							end
						end
					end
				endcase
				end
			end

			F_ADDX: begin : f_addx
				reg [66:0] diff;
				if (!eff_sub) begin
					a_m <= rn_m;
					grs <= rn_g;
					if (rn_hi) e_w <= e_w + 18'sd1;
					a_s <= b_s;
					a_t <= T_NUM;
					begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
				end
				else begin
					diff = {b_m, 3'd0} - sh_v;
					if (diff == 67'd0) begin
						a_t <= T_ZERO;
						a_s <= (rnd_mode == 2'b10);
						begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
					end
					else begin
						a_s <= b_s;
						a_t <= T_NUM;
						acc_hi <= {1'b0, diff[66:3]};
						grs <= diff[2:0];
						begin fst <= F_NLZ; wb_r <= 1'b0; nz_ret <= F_NORM2; acc_r <= 1'b1; end
						nsel <= 2'b10;
					end
				end
			end

			F_NORM2: begin
				if (acc_hi[63:0] == 64'd0 && grs == 3'd0) begin
					a_t <= T_ZERO;
					begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
				end
				else if (nlz_q == 7'd64) begin
					a_m <= {grs, 61'd0};
					grs <= 0;
					e_w <= e_w - 18'sd64;
					begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
				end
				else begin
					a_m <= norm_shifted[66:3];
					grs <= norm_shifted[2:0];
					e_w <= e_w - {11'd0, nlz_q};
					begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
				end
			end

			F_MULT: begin : f_mult
				if (loop_n == 7'd2) begin : mul_fin
					// mul_pd is the registered full 128-bit product, exactly
					// the value the former 32-cycle radix-4 loop accumulated.
					a_m <= rn_m;
					grs <= rn_g;
					if (rn_hi) e_w <= e_w + 18'sd1;
					a_t <= T_NUM;
					begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
				end
				else begin
					// the 64x64 DSP product (it replaced a 32-cycle radix-4
					// loop): the partial products are registered in the
					// first cycle (loop_n 0), summed into mul_pd in the second
					if (loop_n == 7'd1) mul_pd <= mul_out;
					loop_n <= loop_n + 7'd1;
				end
			end

			F_DIVL: begin : f_divl
				if (loop_n == 7'd34) begin
					a_m <= rn_m;
					grs <= rn_g;
					if (!rn_hi) e_w <= e_w - 18'sd1;
					a_t <= T_NUM;
					begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
				end
				else if (loop_n == 7'd0) begin
					// the integer quotient bit compares unshifted (the
					// shared datapath's first step, ds_d0)
					acc_hi <= ds_rem1[64:0];
					qv <= {qv[65:0], ds_q1};
					loop_n <= 7'd1;
				end
				else begin
					// two restoring fraction bits per cycle (66 = 2 x 33)
					acc_hi <= ds_rem[64:0];
					qv <= {qv[64:0], ds_q1, ds_q2};
					loop_n <= loop_n + 7'd1;
				end
			end

			F_SQRTL: begin : f_sqrtl
				if (loop_n == 7'd33) begin
					a_m <= rn_m;
					grs <= rn_g;
					a_t <= T_NUM;
					begin fst <= F_ROUND; wb_r <= (F_ROUND == F_WB); acc_r <= 1'b1; end
				end
				else begin
					// two result digits per cycle (66 = 2 x 33)
					srad <= {srad[61:0], 4'b0000};
					srem <= ds_rem;
					qv <= {qv[64:0], ds_q1, ds_q2};
					loop_n <= loop_n + 7'd1;
				end
			end

			F_ROUND: begin : f_round
				reg [64:0] mr, mi;
				reg [63:0] rmask;            // the precision's kept bits
				reg        inx, up, ovf, unf, tomax;
				reg [1:0]  pr;
				reg signed [17:0] er, emin, emax;
				pr = prec_of(r_op);
				// a move started here (mv_st): its exception shadow, the
				// one form from the working registers (as F_BIN and F_EXEC
				// record theirs), for every source format
				if (mv_st) begin
					sh_src  <= {a_s, a_e[14:0], 16'd0, a_m};
					sh_stag <= r_stag;
					mv_st   <= 1'b0;
				end
				if (a_t != T_NUM) begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
				else begin
					case (pr)
						2'd1: begin
							up = round_up(a_m[40], a_m[39],
							              (a_m[38:0] != 0) || (grs != 0), a_s);
							inx = (a_m[39:0] != 0) || (grs != 0);
							rmask = 64'hFFFF_FF00_0000_0000;
						end
						2'd2: begin
							up = round_up(a_m[11], a_m[10],
							              (a_m[9:0] != 0) || (grs != 0), a_s);
							inx = (a_m[10:0] != 0) || (grs != 0);
							rmask = 64'hFFFF_FFFF_FFFF_F800;
						end
						default: begin
							up = round_up(a_m[0], grs[2], grs[1:0] != 0, a_s);
							inx = (grs != 0);
							rmask = 64'hFFFF_FFFF_FFFF_FFFF;
						end
					endcase
					// one incrementer for the three precisions, formed beside
					// the rounding decision (which only selects it): the
					// sticky compare and the carry chain are not in series
					mi = rd_mi;           // rd_pr is pr here
					mr = up ? mi : {1'b0, a_m & rmask};
					er = (up && mi[64]) ? (e_w + 18'sd1) : e_w;
					if (mr[64]) mr = {2'b01, 63'd0};
					// Range control: the rounding precision narrows the
					// exponent range as well as the significand (softfloat's
					// SOFTFLOAT_68K roundAndPackFloatx80 expOffset, 0x3F80
					// single / 0x3C00 double), EXCEPT for FSGLMUL/FSGLDIV,
					// which round through roundSigAndPackFloatx80 and keep
					// the extended range.
					// Extended (and the sgl class) use the RAW exponent
					// convention: a working exponent of ZERO still packs as
					// a normal result with exponent field 0 (the
					// pseudo-denormal encoding) and no flags; tininess only
					// starts below that, shifting by -er (softfloat probes:
					// (0001-8000)/2 -> 0000-8000 clean, /4 -> 0000-4000
					// UNFL, 2^-16380*2^-13 -> 0000-0020.. shift 10).
					emax = op_sgl(r_op) ? 18'sd32766 :
					       (pr == 2'd1) ? 18'sd16510 :
					       (pr == 2'd2) ? 18'sd17406 : 18'sd32766;
					emin = op_sgl(r_op) ? 18'sd0 :
					       (pr == 2'd1) ? 18'sd16257 :
					       (pr == 2'd2) ? 18'sd15361 : 18'sd0;
					// (decided for the exponent with and without the rounding
					// carry side by side, the carry selecting: a timing path)
					ovf = (up && mi[64]) ? (e_w >= emax) : (e_w > emax);
					// underflow from the intermediate result, before rounding
					// (9.7.5: checked for underflow, then rounded; softfloat's
					// roundAndPackFloatx80: zExp < expOffset + 1): a rounding
					// carry into the normal range does not undo it
					unf = (e_w < emin);
					// WBTEMP capture for a possible BUSY frame: rounded for
					// OVFL and the plain inexact path, unrounded for UNFL --
					// the intermediate before rounding, its exponent with its
					// significand (table 9-16): not the exponent a rounding
					// carry raised
					// -- for OVFL and INEX of the five arithmetic ops (the E3
					// frame) WinUAE's getFloatInternalRoundedSome: the
					// precision's kept bits rounded (the exponent with a
					// rounding carry), the bits below them the unrounded
					// significand's, one up when the rounding went up (table
					// 9-16: INEX's intermediate before rounding).  For FMOVE
					// to a register, FABS and FNEG it is the E1 frame's FPTEMP
					// instead: the unrounded extended intermediate (WinUAE
					// getFloatInternalFloatx80)
					wb_s   <= a_s;
					wb_e   <= (unf || !arith5(r_op)) ? e_w[16:0] : er[16:0];
					wb_m   <= (unf || !arith5(r_op)) ? a_m :
					          ((mr[63:0] & rd_mask) | ({24'd0, up ? (a_m[39:0] + 40'd1) : a_m[39:0]} & ~rd_mask));
					wb_grs <= grs;
					if (ovf) begin
						fpsr[12] <= 1;              // OVFL
						fpsr[6]  <= 1;              // accrued OVFL
						fpsr[3]  <= 1;              // accrued INEX (from OVFL)
						// INEX2 itself only reports actually-discarded bits
						// (roundAndPackFloatx80 raises inexact on overflow
						// only when zSig0 & roundMask is nonzero)
						if (inx) fpsr[9] <= 1;
						tomax = (rnd_mode == 2'b01) ||
						        (rnd_mode == 2'b10 && !a_s) ||
						        (rnd_mode == 2'b11 && a_s);
						if (tomax) begin
							a_e <= emax[16:0];
							a_m <= op_sgl(r_op) ? 64'hFFFF_FFFF_FFFF_FFFF :
							       (pr == 2'd1) ? 64'hFFFF_FF00_0000_0000 :
							       (pr == 2'd2) ? 64'hFFFF_FFFF_FFFF_F800 :
							                      64'hFFFF_FFFF_FFFF_FFFF;
						end
						else begin
							a_t <= T_INF;
							a_m <= 64'd0;   // created infinity: all-zero
						end
						begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
					end
					else if (unf) begin : f_unf
						// Gradual underflow.  At single or double rounding
						// precision the result is only subnormal with respect
						// to that precision, so it still fits the extended
						// format as a normal number: shift the significand
						// down to the precision's minimum exponent, round
						// there, and renormalize in F_UNFL.  A result below
						// the extended minimum exponent would need the
						// denormalized encoding (and a signed working
						// exponent throughout), which this implementation
						// does not have; those flush to zero.
						reg signed [17:0] extra;
						// the UNROUNDED significand (a_m, grs) is shifted:
						// by its own exponent, not one a carry of the
						// precision's rounding raised (9.7.5: denormalize,
						// then round) -- emin - er shifted a carried result
						// a bit too little and doubled it
						extra = emin - e_w;
						fpsr[11] <= 1;              // UNFL status (tiny)
						// accrued UNFL is only recorded when the result is
						// ALSO inexact (updateaccrued: UNFL && INEX2); the
						// deep-flush is always inexact, the F_UNFL path
						// decides for itself
						if (extra > 18'sd66) begin
							fpsr[9] <= 1;           // INEX2
							fpsr[3] <= 1;
							fpsr[5] <= 1;           // accrued UNFL
							// The significand shifts out entirely, so the
							// magnitude is below everything representable at
							// emin.  Round-to-nearest and round-to-zero give
							// zero, but a DIRECTED mode pointing away from
							// zero must not: IEEE and softfloat both round a
							// nonzero value up to the smallest representable
							// magnitude.  At single/double precision that is
							// still a normal extended number at emin (one
							// unit in the last place of the narrowed
							// significand).
							// EXTENDED and FSGL were flushed to zero here
							// on the grounds that they would need the true
							// denormal encoding.  They do -- and the
							// writeback can express it: the register form
							// is {sign, exponent, 16'd0, significand}, so
							// exponent field 0 with the integer bit clear
							// is just a value, not a new representation.
							// Flushing instead returned -0 where the
							// answer is the minimum negative denormal and
							// wrongly set the Z condition code.  WinUAE's
							// softfloat rounds these up; a differential
							// FMUL.X toward -inf caught it (AP040
							// 8000-0000000000000000 vs reference
							// 8000-0000000000000001).  A denormal left in
							// a register faults as an unsupported data
							// type on its next use, which is exactly how a
							// 68040 hands it to the FPSP.
							if ((rnd_mode == 2'b11 && !a_s) ||
							    (rnd_mode == 2'b10 && a_s)) begin
								a_t <= T_NUM;
								if (pr != 2'd0 && !op_sgl(r_op)) begin
									a_e <= emin[16:0];
									a_m <= (pr == 2'd1)
									       ? 64'h0000_0100_0000_0000
									       : 64'h0000_0000_0000_0800;
								end
								else begin
									// smallest representable magnitude:
									// exponent 0, one ulp of this
									// operation's significand granularity
									// (FSGL keeps single's 24-bit step)
									a_e <= 17'd0;
									a_m <= op_sgl(r_op)
									       ? 64'h0000_0100_0000_0000
									       : 64'h0000_0000_0000_0001;
								end
							end
							else a_t <= T_ZERO;
							begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
						end
						else begin
							sh_v <= {a_m, grs};
							sh_cnt <= extra[6:0];
							e_w <= emin;
							r_pr <= pr;
							sh_ret <= F_UNFL;
							begin fst <= F_SHR; wb_r <= (F_SHR == F_WB); acc_r <= 1'b0; end
						end
					end
					else begin
						// Blanket INEX: FABS/FNEG round through the same
						// path as every op and report discarded bits even
						// for extended sources.  Twice hardware-adjudicated:
						// the 2026-08-07 FABS.X capture (FPSR $0208) and the
						// 2026-08-09 rerun on the revert-built bitstream,
						// which failed cputest exactly here.
						if (inx) begin
							fpsr[9] <= 1;
							fpsr[3] <= 1;
						end
						a_m <= mr[63:0];
						a_e <= er[16:0];
						begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
					end
				end
			end

			F_WB: begin : f_wb
				reg        n, z, nan;
				reg        ds, dz, gt, eq, dbig;
				reg [83:0] du;
				if (|(fpsr[15:8] & fpcr[15:8])) begin
					// Enabled exceptions suppress architectural destination
					// writeback (fr_wb_we, fr_valid below).  The core builds
					// the normal arithmetic-exception stack frame for the
					// selected vector.  The condition codes are still set
					// from the default result that is not written, as for
					// a completed operation: WinUAE (the oracle) sets them in
					// fp_arithmetic before fpsr_check_arithmetic_exception,
					// and the 68060 does the same (Motorola's ftest.sa:
					// FPSR $09004080 after FADD.B #2,FP0 with FP0 a negative
					// SNAN); the M68040FPSP's enabled SNAN/OPERR exits leave
					// the user FPSR as the hardware set it.
					exc_req <= 1;
					exc_vec <= fp_exception_vector(fpsr[15:8] & fpcr[15:8]);
				end
				if (r_op == 7'h38) begin : f_cmp
					// FCMP: condition codes from FPn - source
					du = {b_s, b_e, b_m, b_t};
					ds = du[83];
					dz = (du[1:0] == T_ZERO);
					nan = (a_t == T_NAN) || (du[1:0] == T_NAN);
					n = 0; z = 0;
					if (nan) begin
						// cmp_signed_nan (68040): the propagated NaN keeps
						// its sign and N reports it; the destination NaN is
						// preferred, matching propagateFloatx80NaN(a, b)
						n = (du[1:0] == T_NAN) ? ds : a_s;
					end
					else begin
						eq = (dz && a_t == T_ZERO) ||
						     (du[1:0] == a_t && ds == a_s && du[1:0] != T_ZERO &&
						      (du[1:0] == T_INF ||
						       (du[82:66] == a_e && du[65:2] == a_m)));
						if (eq) begin
							z = 1;
							// equal ZEROS and equal INFINITIES report the
							// destination sign in N; equal finite values
							// compare as +0 (floatx80_cmp packs (0,0,0)),
							// so N stays clear even for two negatives
							n = (dz || du[1:0] == T_INF) ? ds : 1'b0;
						end
						else begin
							if (dz)                 gt = a_s;
							else if (a_t == T_ZERO) gt = !ds;
							else if (ds != a_s)     gt = !ds;
							else begin
								if (du[1:0] == T_INF)     dbig = 1;
								else if (a_t == T_INF)    dbig = 0;
								else dbig = ($signed(du[82:66]) > $signed(a_e)) ||
								            (du[82:66] == a_e && du[65:2] > a_m);
								gt = ds ? !dbig : dbig;
							end
							n = !gt;
						end
					end
					fpsr[27:24] <= {n, z, 1'b0, nan};
				end
				else if (r_op == 7'h3A) begin
					// FTST
					fpsr[27:24] <= {a_s,
					                (a_t == T_ZERO),
					                (a_t == T_INF),
					                (a_t == T_NAN)};
				end
				else begin
					// writeback with condition codes, canonical encodings
					// for the special classes
					// Infinity keeps whatever mantissa the operation left
					// in a_m: created infinities carry all-zero bits and
					// pass-through infinities keep the operand's raw image
					// (inf_clear_intbit is a 68060 flag, not 68040).
					if (!(|(fpsr[15:8] & fpcr[15:8]))) fr_valid[r_dst] <= 1;
					fpsr[27:24] <= {a_s,
					                (a_t == T_ZERO),
					                (a_t == T_INF),
					                (a_t == T_NAN)};
				end
				fpu_used <= 1;
				if (!(|(fpsr[15:8] & fpcr[15:8]))) done <= 1;
				begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end
			end

			F_SHR: begin : f_shr
				// right shift with sticky collection, in one cycle: the
				// value shifted by sh_cnt, every bit shifted out ORed into
				// bit 0 ({int[66:3], G, R, S}; the former 32/16/8/4/2/1
				// steps composed to the same)
				if (sh_cnt != 0) sh_v <= shr_out;
				sh_cnt <= 7'd0;
				begin fst <= sh_ret; wb_r <= (sh_ret == F_WB); acc_r <= acc_of(sh_ret); end
			end

			F_PACKI: begin : f_packi
				// round and range check the integer store
				reg  [64:0] mg;
				reg  [31:0] lo;
				reg         ovf, hi32, hi16, hi8;
				if (sh_cnt == 7'd127) begin
					// infinity / far out of range: OPERR, saturating to the
					// SIGN-DEPENDENT extreme (roundAndPackInt32/16/8)
					fpsr[13] <= 1;                       // OPERR
					fpsr[7]  <= 1;                       // accrued IOP
					dout <= {(pk_isz == 2'd2) ? (pk_neg ? 32'h8000_0000 : 32'h7FFF_FFFF) :
					         (pk_isz == 2'd1) ? (pk_neg ? {16'h8000, 16'd0} : {16'h7FFF, 16'd0}) :
					                            (pk_neg ? {8'h80, 24'd0} : {8'h7F, 24'd0}), 64'd0};
				end
				else begin
					// the rounded magnitude; its range checked as it is (an
					// n-bit signed result holds below 2^(n-1), or exactly
					// 2^(n-1) when negative), and only the low long word
					// negated for the store
					mg = round_up(sh_v[3], sh_v[2], (sh_v[1:0] != 0), pk_neg) ? shk_i : shk;
					lo = pk_neg ? (32'd0 - mg[31:0]) : mg[31:0];
					hi32 = (mg[64:32] != 0) || (mg[31] && !(pk_neg && mg[30:0] == 0));
					hi16 = (mg[64:16] != 0) || (mg[15] && !(pk_neg && mg[14:0] == 0));
					hi8  = (mg[64:8]  != 0) || (mg[7]  && !(pk_neg && mg[6:0]  == 0));
					ovf = (pk_isz == 2'd2) ? hi32 : (pk_isz == 2'd1) ? hi16 : hi8;
					if (ovf) begin
						fpsr[13] <= 1;                   // OPERR
						fpsr[7]  <= 1;
						dout <= {(pk_isz == 2'd2) ? (pk_neg ? 32'h8000_0000 : 32'h7FFF_FFFF) :
						         (pk_isz == 2'd1) ? (pk_neg ? {16'h8000, 16'd0} : {16'h7FFF, 16'd0}) :
						                            (pk_neg ? {8'h80, 24'd0} : {8'h7F, 24'd0}), 64'd0};
					end
					else begin
						dout <= {(pk_isz == 2'd2) ? lo :
						         (pk_isz == 2'd1) ? {lo[15:0], 16'd0} :
						                            {lo[7:0], 24'd0}, 64'd0};
						if (sh_v[2:0] != 0) begin
							fpsr[9] <= 1;                // INEX2
							fpsr[3] <= 1;                // accrued INEX
						end
					end
				end
				fpu_used <= 1;
				if (fpcr[15:8] == 8'd0) begin done <= 1; begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end end else begin fst <= F_STDONE; wb_r <= (F_STDONE == F_WB); acc_r <= 1'b0; end  // no enabled exception possible: done now
			end

			F_UNFL: begin : f_unfl
				// Round the shifted significand at the boundary of the
				// rounding precision.  e_w holds that precision's minimum
				// exponent, which the shift in F_SHR has already scaled the
				// significand to.
				reg [64:0] mr, mk, mi;
				reg [63:0] rmask;
				reg        up, inx2;
				if (r_pr == 2'd0) begin
					up = round_up(sh_v[3], sh_v[2],
					              (sh_v[1:0] != 0), a_s);
					inx2 = (sh_v[2:0] != 0);
					rmask = 64'hFFFF_FFFF_FFFF_FFFF;
				end
				else if (r_pr == 2'd1) begin
					up = round_up(sh_v[43], sh_v[42],
					              (sh_v[41:0] != 0), a_s);
					inx2 = (sh_v[42:0] != 0);
					rmask = 64'hFFFF_FF00_0000_0000;
				end
				else begin
					up = round_up(sh_v[14], sh_v[13],
					              (sh_v[12:0] != 0), a_s);
					inx2 = (sh_v[13:0] != 0);
					rmask = 64'hFFFF_FFFF_FFFF_F800;
				end
				// the increment (the shared incrementer, shk_i) is formed
				// beside the rounding decision, which only selects it (a
				// timing path)
				mk = {1'b0, sh_v[66:3] & rmask};
				mi = {shk_i[64], shk_i[63:0] & rmask};
				mr = up ? mi : mk;
				if (inx2) begin
					fpsr[9] <= 1;                   // INEX2
					fpsr[3] <= 1;
					fpsr[5] <= 1;                   // accrued UNFL (UNFL&&INEX2)
				end
				// At extended precision the result is subnormal for the
				// destination format itself, so it takes the denormalized
				// encoding: exponent field zero with the integer bit clear.
				// At single or double precision the result is subnormal only
				// with respect to that precision and keeps that precision's
				// minimum exponent, matching softfloat's packFloatx80 of
				// (expOffset + 1).  Rounding that carries into the integer
				// bit has reached the next exponent up either way.
				// (the class and exponent decided beside the rounding: an
				// incremented significand that does not carry out is never
				// zero, so the result is zero only unrounded with mk zero)
				if (up && mi[64]) begin
					a_m <= 64'h8000_0000_0000_0000;
					a_e <= e_w[16:0] + 17'd1;
					begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
				end
				else begin
					// (a zero takes a_m = 0 as well: the zero test only
					// decides the class and exponent)
					a_m <= mr[63:0];
					if (!up && mk[63:0] == 64'd0) a_t <= T_ZERO;
					else a_e <= ((up ? mi[63] : mk[63]) || e_w != 18'sd1) ? e_w[16:0] : 17'd0;
					begin fst <= F_WB; wb_r <= (F_WB == F_WB); acc_r <= 1'b0; end
				end
			end

			F_PACKS: begin : f_packs
				// Finish a single/double denormal after the staged shift.
				// A carry from the fraction becomes the minimum normal.
				// (sd_out packs the rounded shifted significand: its
				// integer bit is the exponent field's lowest)
				reg inx;
				inx = sd_isS ? (sh_v[42:0] != 0) : (sh_v[13:0] != 0);
				dout <= sd_out;
				// Tininess is detected BEFORE rounding.  Reaching F_PACKS
				// already means the value is below this precision's
				// minimum normal exponent (the sE < -126 / -1022 test at
				// the entry above), so UNFL is raised even when the
				// conversion is EXACT, and even when rounding carries the
				// significand back up to the minimum normal.  Gating it
				// on `inx && tiny` -- tininess measured AFTER rounding --
				// lost both cases: an exact extended 2^-127 stored as
				// single 00400000 reported no UNFL, and a value just
				// below the single normal boundary rounded to 00800000
				// reported only INEX.  softfloat sets UNFL for both.
				// Accrued UNFL still follows the 040's updateaccrued rule
				// (UNFL && INEX2), which is why it stays inside the
				// inexact branch.
				fpsr[11] <= 1;                        // UNFL status
				if (inx) begin
					fpsr[9] <= 1;                 // INEX2
					fpsr[3] <= 1;                 // accrued INEX
					fpsr[5] <= 1;                 // accrued UNFL
				end
				fpu_used <= 1;
				if (fpcr[15:8] == 8'd0) begin done <= 1; begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end end else begin fst <= F_STDONE; wb_r <= (F_STDONE == F_WB); acc_r <= 1'b0; end  // no enabled exception possible: done now
			end

			F_STDONE: begin
				// Store completion one cycle after the exits above, so the
				// status byte has settled: an ENABLED exception is reported
				// for post-instruction delivery by the core (write-then-trap
				// for float formats; the integer SNAN/OPERR set suppresses
				// the write, decided core-side).  Disabled exceptions
				// complete in hardware with the architectural default
				// result already in dout: AP040 runs without FPSP, so the
				// real 040's nonmaskable store set is served by the same
				// defaults FPSP would have stored.
				done <= 1;
				if (|(fpsr[15:8] & fpcr[15:8])) begin
					exc_req <= 1;
					exc_vec <= fp_exception_vector(fpsr[15:8] & fpcr[15:8]);
				end
				begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end
			end

			default: begin fst <= F_IDLE; wb_r <= (F_IDLE == F_WB); acc_r <= 1'b0; end
		endcase
	end
end

endmodule

// Two mirrored simple-dual-port banks provide two asynchronous reads and one
// synchronous write.  Cyclone V MLABs support flow-through reads; keeping the
// memory alone in this module gives Quartus the canonical inference shape.
module ap040_fp_regfile
(
	input             clk,
	input             ce,
	input             we,
	input       [2:0] waddr,
	input      [79:0] wdata,
	input       [2:0] raddr_a,
	output     [79:0] rdata_a,
	input       [2:0] raddr_b,
	output     [79:0] rdata_b
);

// READ DURING WRITE.  no_rw_check does not promise the accesses never
// coincide -- it says the read data is undefined when they do, and asks the
// fitter not to spend logic on it.  This file coincides constantly: counting
// cycles where a read port sits on the register being written gives 1,329 in
// t_fpu alone, 57 in t_fpu_resume, 6 in t_fpu_frames, the very first being a
// write to FP0 with both read ports on FP0.
//
// The integer register file had the same shape and did not boot.  So the same
// remedy: hold the write one enabled cycle and answer a read of the pending
// address from pend_wdata, which discards exactly the datum the attribute
// leaves undefined.  Reads are otherwise unchanged, and an MLAB is still
// inferred because the attribute stays.  The hold also keeps the RAM's write
// inputs registered, which matters on its own: see ap040_regfile for the
// board result when the RAM was written straight from the datapath
// (06d90f6fb, yellow screen) and the non-unate clock edge TimeQuest reports
// for these cells.  Keep pend_* in front of the RAM.
(* ramstyle = "MLAB, no_rw_check" *) reg [79:0] bank_a [0:7];
(* ramstyle = "MLAB, no_rw_check" *) reg [79:0] bank_b [0:7];

reg        pend_we;
reg  [2:0] pend_waddr;
reg [79:0] pend_wdata;

wire hit_a = pend_we && (pend_waddr == raddr_a);
wire hit_b = pend_we && (pend_waddr == raddr_b);
assign rdata_a = hit_a ? pend_wdata : bank_a[raddr_a];
assign rdata_b = hit_b ? pend_wdata : bank_b[raddr_b];

always @(posedge clk) begin
	if (ce) begin
		// the write held from the previous enabled cycle
		if (pend_we) begin
			bank_a[pend_waddr] <= pend_wdata;
			bank_b[pend_waddr] <= pend_wdata;
		end
		pend_we    <= we;
		pend_waddr <= waddr;
		pend_wdata <= wdata;
	end
end

endmodule
