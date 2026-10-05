//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_pkg.sv - shared types and encodings                               //
//--------------------------------------------------------------------------//

package ap68040_pkg;

//--------------------------------------------------------------------------
// Register numbers (5 bits).  A7 never appears here: the decoder maps it
// to USP, ISP or MSP from the S and M bits current at decode time.
//--------------------------------------------------------------------------
localparam logic [4:0] R_D0  = 5'd0;
localparam logic [4:0] R_A0  = 5'd8;
localparam logic [4:0] R_USP = 5'd15;
localparam logic [4:0] R_ISP = 5'd16;
localparam logic [4:0] R_MSP = 5'd17;
localparam logic [4:0] R_T0  = 5'd18;   // T0..T13 = 18..31, microcode temps

//--------------------------------------------------------------------------
// Bus encodings (MC68040UM tables 5-2, 5-3, 7-1)
//--------------------------------------------------------------------------
localparam logic [1:0] SIZ_L    = 2'b00;
localparam logic [1:0] SIZ_B    = 2'b01;
localparam logic [1:0] SIZ_W    = 2'b10;
localparam logic [1:0] SIZ_LINE = 2'b11;

localparam logic [1:0] TT_NORMAL = 2'd0;
localparam logic [1:0] TT_MOVE16 = 2'd1;
localparam logic [1:0] TT_ALT    = 2'd2;
localparam logic [1:0] TT_ACK    = 2'd3;

localparam logic [2:0] TM_PUSH     = 3'd0;
localparam logic [2:0] TM_UDATA    = 3'd1;
localparam logic [2:0] TM_UCODE    = 3'd2;
localparam logic [2:0] TM_TBL_DATA = 3'd3;
localparam logic [2:0] TM_TBL_CODE = 3'd4;
localparam logic [2:0] TM_SDATA    = 3'd5;
localparam logic [2:0] TM_SCODE    = 3'd6;

// One internal bus transaction, as a requester hands it to the BIU.  The
// BIU only runs naturally aligned transfers; requesters split misaligned
// operands (MC68040UM 7.3).  For a line, addr[3:2] names the first long
// word and the slave wraps.
typedef struct packed {
	logic [31:0] addr;
	logic  [1:0] siz;     // SIZ_*
	logic        rd;      // 1 read, 0 write
	logic  [1:0] tt;
	logic  [2:0] tm;
	logic  [1:0] tln;
	logic  [1:0] upa;
	logic        ci;      // CIOUT
	logic        lock;    // LOCK for this transfer
	logic        locke;   // LOCKE: last transfer of the locked sequence
} busreq_t;

//--------------------------------------------------------------------------
// ATC entry (MC68040UM figure 3-21); pa is PA31-12 (PA12 comes from LA12
// with 8 Kbyte pages)
//--------------------------------------------------------------------------
typedef struct packed {
	logic [19:0] pa;
	logic        g;       // global
	logic  [1:0] upa;     // U1 U0
	logic        s;       // supervisor only
	logic  [1:0] cm;      // cache mode
	logic        m;       // modified
	logic        w;       // write protected (accumulated)
	logic        r;       // resident (table search succeeded)
	logic        b;       // the table search took a bus error
} atce_t;

//--------------------------------------------------------------------------
// Operation sizes
//--------------------------------------------------------------------------
localparam logic [1:0] SZ_B = 2'd0;
localparam logic [1:0] SZ_W = 2'd1;
localparam logic [1:0] SZ_L = 2'd2;
localparam logic [1:0] SZ_Q = 2'd3;   // line / double, context dependent

//--------------------------------------------------------------------------
// Micro-operations
//--------------------------------------------------------------------------
// operand sources
localparam logic [1:0] OS_REG  = 2'd0;
localparam logic [1:0] OS_IMM  = 2'd1;
localparam logic [1:0] OS_MEM  = 2'd2;   // the uop's load data
localparam logic [1:0] OS_ZERO = 2'd3;

// memory access of a uop
localparam logic [1:0] M_NONE = 2'd0;
localparam logic [1:0] M_LD   = 2'd1;
localparam logic [1:0] M_ST   = 2'd2;
localparam logic [1:0] M_RMW  = 2'd3;    // load, then store to the same place

// function code of a data access
localparam logic [2:0] MFC_NORM = 3'd0;  // user/supervisor data by S
localparam logic [2:0] MFC_SFC  = 3'd1;  // MOVES read
localparam logic [2:0] MFC_DFC  = 3'd2;  // MOVES write
localparam logic [2:0] MFC_SUP  = 3'd3;  // supervisor data (exception stacking)
localparam logic [2:0] MFC_IACK = 3'd4;  // interrupt acknowledge (TT=3)

// control flow
// internal exception codes of instruction fetch faults (uop.exc); real
// vectors otherwise.  The back end turns them into vector 2.
localparam logic [7:0] EXC_IFS  = 8'd1;    // speculative fetch, bus error
localparam logic [7:0] EXC_IFB  = 8'd2;    // demand fetch, bus error
localparam logic [7:0] EXC_IFSA = 8'hFD;   // speculative fetch, ATC fault
localparam logic [7:0] EXC_IFA  = 8'hFE;   // demand fetch, ATC fault

localparam logic [2:0] BR_NONE = 3'd0;
localparam logic [2:0] BR_COND = 3'd1;   // decision from the EX op (Bcc, DBcc, FBcc)
localparam logic [2:0] BR_IMM  = 3'd2;   // always, to uop.target
localparam logic [2:0] BR_EA   = 3'd3;   // always, to the effective address
localparam logic [2:0] BR_A    = 3'd4;   // always, to operand A (RTS: load data)
localparam logic [2:0] BR_B    = 3'd5;   // always, to operand B (JSR)

// EX operations
localparam logic [6:0]
	OP_MOV   = 7'd0,   // res = A                     N Z, V=C=0
	OP_ADD   = 7'd1,   // res = B + A
	OP_ADDX  = 7'd2,
	OP_SUB   = 7'd3,   // res = B - A
	OP_SUBX  = 7'd4,
	OP_CMP   = 7'd5,   // flags of B - A, no result
	OP_AND   = 7'd6,
	OP_OR    = 7'd7,
	OP_EOR   = 7'd8,
	OP_NOT   = 7'd9,   // res = ~B
	OP_NEG   = 7'd10,  // res = 0 - B
	OP_NEGX  = 7'd11,
	OP_CLR   = 7'd12,
	OP_EXT   = 7'd13,  // sz W: byte->word, sz L: word->long
	OP_EXTB  = 7'd14,  // byte->long
	OP_SWAP  = 7'd15,
	OP_TAS   = 7'd16,
	OP_ABCD  = 7'd17,
	OP_SBCD  = 7'd18,
	OP_NBCD  = 7'd19,
	OP_PACK  = 7'd20,  // A = adjustment, B = unpacked word
	OP_UNPK  = 7'd21,  // A = adjustment, B = packed byte
	OP_ASL   = 7'd22,  // B shifted by A[5:0]
	OP_ASR   = 7'd23,
	OP_LSL   = 7'd24,
	OP_LSR   = 7'd25,
	OP_ROL   = 7'd26,
	OP_ROR   = 7'd27,
	OP_ROXL  = 7'd28,
	OP_ROXR  = 7'd29,
	OP_BTST  = 7'd30,  // bit A of B
	OP_BCHG  = 7'd31,
	OP_BCLR  = 7'd32,
	OP_BSET  = 7'd33,
	OP_SCC   = 7'd34,  // res = cond ? $FF : 0
	OP_EA    = 7'd35,  // res = effective address
	OP_BCC   = 7'd36,  // branch decision from cond
	OP_DBCC  = 7'd37,  // res = B.w - 1; taken = !cond && res.w != -1
	OP_TRAPCC= 7'd38,  // exception 7 if cond
	OP_CHK   = 7'd39,  // B against 0..A, exception 6
	OP_CHK2A = 7'd40,  // CHK2/CMP2 step 1: compare B with lower bound A
	OP_CHK2B = 7'd41,  // step 2: compare with upper bound A; cond[0] = trap
	OP_CCRLOG= 7'd42,  // CCR = CCR (cond: 0 AND 1 OR 2 EOR 3 MOVE) A
	OP_SRLOG = 7'd43,  // res = SR (cond as above) A, written to SR at WB
	OP_SPR   = 7'd44,  // res = special register imm[7:0]
	OP_SPW   = 7'd45,  // special register imm[7:0] = A at WB
	OP_LATCH = 7'd46,  // EX latch = A (third operand for CAS and bit fields)
	OP_CAS   = 7'd47,  // compare B with latch; store A if equal
	OP_MUL   = 7'd48,  // cond[0] signed, cond[1] 64-bit
	OP_DIV   = 7'd49,  // cond[0] signed, cond[1] 64-bit dividend, sz W/L
	OP_MDHI  = 7'd50,  // multiply/divide high input = A
	OP_MDRES = 7'd51,  // res = remainder / product high
	OP_BF    = 7'd52,  // bit field, cond = BF op
	OP_BFSET = 7'd53,  // bit field offset/width setup
	OP_MISC  = 7'd54,  // WB-side operation in cond (RESET, STOP, NOP, CINV...)
	OP_FPU   = 7'd55,  // floating point, see the FPU interface
	OP_CHKSR = 7'd56,  // privilege/format checks for RTE etc (cond)
	OP_CAS2C = 7'd57,  // CAS2 compare: cond 0 first pair, 1 second (if equal so far)
	OP_CAS2W = 7'd58,  // CAS2 store: cond 0 A only if equal, 1 equal ? A : B
	OP_CAS2R = 7'd59,  // CAS2 register: B merged with A unless equal
	OP_RTEF  = 7'd60,  // RTE: frame length of format word A (exception 14)
	OP_RTE   = 7'd61,  // RTE: SR = A, jump to B (throwaway frame: this RTE)
	OP_IACKV = 7'd62;  // interrupt vector from the IACK data

typedef struct packed {
	logic [31:0] pc;        // address of the 68040 instruction
	logic [31:0] npc;       // address of the next sequential instruction
	logic        first;     // first uop of the instruction
	logic        last;      // last uop: the instruction completes with it
	// execution
	logic  [6:0] op;
	logic  [1:0] sz;
	logic  [3:0] cond;
	logic  [4:0] ccr_we;    // X N Z V C
	// operands
	logic  [1:0] a_src;
	logic  [4:0] a_reg;
	logic        a_sxw;     // sign-extend operand A from a word
	logic  [1:0] b_src;
	logic  [4:0] b_reg;
	logic [31:0] imm;       // operand A immediate (or a constant)
	logic [31:0] imm_b;     // operand B immediate
	// address generation: EA = base + idx*scale + disp
	logic        ag;        // compute an effective address
	logic        base_v;
	logic  [4:0] base;
	logic        idx_v;
	logic  [4:0] idx;
	logic        idx_l;
	logic  [1:0] scale;
	logic [31:0] disp;
	logic        pinc;      // postincrement: EA = base, update = base + amt
	logic        upd_v;     // AG write: upd_reg = pinc ? base + amt : EA
	logic  [4:0] upd_reg;
	logic  [7:0] upd_amt;   // signed: postincrement amount (and a deferred offset)
	logic        upd2_v;    // AG write: upd2_reg += upd2_amt
	logic  [4:0] upd2_reg;
	logic  [7:0] upd2_amt;  // signed
	// memory
	logic  [1:0] mem;
	logic  [1:0] msz;
	logic  [2:0] mfc;
	logic        mlock;
	logic        mlocke;    // last transfer of the locked sequence
	logic        mprog;     // program space (PC-relative operand)
	// result
	logic        d_v;
	logic  [4:0] d_reg;
	// control flow
	logic  [2:0] br;
	logic [31:0] target;
	logic        pred;      // the front end followed target
	// exceptions and control
	logic  [7:0] exc;       // decode-time exception vector, 0 = none
	logic        ser;       // serialize: flush and refetch npc after WB
	logic        t0cof;     // the instruction is on the 68040 T0 trace list
	logic        b_upd;     // operand B is this uop's own (An)+/-(An) register:
	                        // it reads the updated value (source EA updates)
} uop_t;

//--------------------------------------------------------------------------
// Decoded instruction (D1 -> D2)
//--------------------------------------------------------------------------
// EA mode index, MC68040 order
localparam logic [3:0] EM_DN = 4'd0, EM_AN = 4'd1, EM_AI = 4'd2, EM_AIP = 4'd3,
                       EM_APD = 4'd4, EM_AD16 = 4'd5, EM_AX = 4'd6,
                       EM_ABSW = 4'd7, EM_ABSL = 4'd8, EM_PC16 = 4'd9,
                       EM_PCX = 4'd10, EM_IMM = 4'd11, EM_NONE = 4'd15;

typedef struct packed {
	logic  [3:0] m;       // EM_*
	logic  [2:0] r;       // register field
	logic  [2:0] xoff;    // word offset of the first extension word
	// index modes (brief or full extension word)
	logic        xa;      // index is an address register
	logic  [2:0] xr;
	logic        xl;      // long index
	logic  [1:0] sc;
	logic        bs;      // base suppressed
	logic        is;      // index suppressed
	logic  [1:0] mi;      // 0 none, 1 preindexed, 2 postindexed memory indirect
	logic [31:0] bd;      // displacement / absolute address / immediate
	logic [31:0] od;      // outer displacement
} ea_t;

// predecode of a queue word taken as an operation word (computed as the
// word enters the instruction queue, so D1 never decodes in its loop)
typedef struct packed {
	logic        legal;   // a legal operation word
	logic  [8:0] ent;     // decoder entry
	logic  [1:0] sz;
	logic  [2:0] b;       // operation word + fixed words + immediate (1..5)
	logic  [2:0] p1;      // first extension word of EA1 (b + EA0 words)
	logic  [3:0] tot;     // length assuming brief index extensions
	logic        x0;      // EA0 has an index extension (at b)
	logic        x1;      // EA1 has an index extension (at p1)
	logic        slow;    // parsed in several cycles (FPU immediate)
	logic  [3:0] i0;      // EA0 mode index, EM_NONE when absent
	logic  [3:0] i1;
} pd_t;

typedef struct packed {
	logic [31:0] pc;
	logic [31:0] npc;
	logic [15:0] opw;
	logic [15:0] ext1;
	logic [15:0] ext2;
	logic  [8:0] rt;      // first micro-instruction
	logic  [6:0] eop;
	logic  [3:0] econd;
	logic  [4:0] ccr;
	logic  [1:0] sz;
	logic  [7:0] exc;     // decode-time exception vector, 0 none
	logic [31:0] imm;     // immediate field
	logic [63:0] fimm;    // further immediate words (FPU .D/.X/.P)
	ea_t         ea0;
	ea_t         ea1;
	logic [31:0] target;  // PC-relative branch target
	logic        pred;    // the front end followed target
	logic        t0;      // on the 68040 T0 trace list
} dinst_t;

endpackage
