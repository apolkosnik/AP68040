//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_defs.svh - common constants shared by all AP040 modules            //
//                                                                          //
// Encodings below follow the existing Minimig cpu_wrapper contract         //
// (TG68K style busstate) and the MC68040 architectural values.             //
//--------------------------------------------------------------------------//

`ifndef AP040_DEFS_SVH
`define AP040_DEFS_SVH

// External bus state, identical to the TG68K/cpu_wrapper contract:
// 0: fetch code, 1: no memaccess, 2: read data, 3: write data
`define AP040_BUS_FETCH   2'b00
`define AP040_BUS_IDLE    2'b01
`define AP040_BUS_READ    2'b10
`define AP040_BUS_WRITE   2'b11

// Internal transfer sizes
`define AP040_SZ_B        2'd0
`define AP040_SZ_W        2'd1
`define AP040_SZ_L        2'd2

// Function codes
`define AP040_FC_USER_DATA   3'd1
`define AP040_FC_USER_PROG   3'd2
`define AP040_FC_SUPER_DATA  3'd5
`define AP040_FC_SUPER_PROG  3'd6
`define AP040_FC_CPU_SPACE   3'd7

// Reset vector locations (VBR does not apply to the reset vectors)
`define AP040_VEC_RESET_ISP  32'h0000_0000
`define AP040_VEC_RESET_PC   32'h0000_0004

// SR bit positions
`define AP040_SR_C   0
`define AP040_SR_V   1
`define AP040_SR_Z   2
`define AP040_SR_N   3
`define AP040_SR_X   4
`define AP040_SR_M   12
`define AP040_SR_S   13
`define AP040_SR_T0  14
`define AP040_SR_T1  15

// SR after reset: T=0, S=1, M=0, IPL=7
`define AP040_SR_RESET  16'h2700

// MC68040 CACR bits
`define AP040_CACR_IE   15
`define AP040_CACR_DE   31

// exception vector numbers
`define AP040_VEC_BUSERR   8'd2
`define AP040_VEC_ADDRERR  8'd3
`define AP040_VEC_ILLEGAL  8'd4
`define AP040_VEC_DIVZERO  8'd5
`define AP040_VEC_CHK      8'd6
`define AP040_VEC_TRAPCC   8'd7
`define AP040_VEC_PRIV     8'd8
`define AP040_VEC_TRACE    8'd9
`define AP040_VEC_ALINE    8'd10
`define AP040_VEC_FLINE    8'd11
`define AP040_VEC_FMTERR   8'd14
`define AP040_VEC_AUTOVEC  8'd24   // + interrupt level
`define AP040_VEC_TRAP     8'd32   // + trap number
`define AP040_VEC_FP_BSUN  8'd48
`define AP040_VEC_FP_INEX  8'd49
`define AP040_VEC_FP_DZ    8'd50
`define AP040_VEC_FP_UNFL  8'd51
`define AP040_VEC_FP_OPERR 8'd52
`define AP040_VEC_FP_OVFL  8'd53
`define AP040_VEC_FP_SNAN  8'd54
`define AP040_VEC_FP_UNSUP 8'd55   // unsupported data type (format $0/$3)

// SR write mask on MC68040: T1 T0 S M IPL2:0 XNZVC
`define AP040_SR_MASK   16'hF71F
`define AP040_CCR_MASK  16'h001F

// ALU operations
`define AP040_ALU_MOVE   6'd0
`define AP040_ALU_ADD    6'd1
`define AP040_ALU_ADDX   6'd2
`define AP040_ALU_SUB    6'd3
`define AP040_ALU_SUBX   6'd4
`define AP040_ALU_CMP    6'd5
`define AP040_ALU_AND    6'd6
`define AP040_ALU_OR     6'd7
`define AP040_ALU_EOR    6'd8
`define AP040_ALU_NOT    6'd9
`define AP040_ALU_NEG    6'd10
`define AP040_ALU_NEGX   6'd11
`define AP040_ALU_CLR    6'd12
`define AP040_ALU_TST    6'd13
`define AP040_ALU_EXT    6'd14
`define AP040_ALU_EXTB   6'd15
`define AP040_ALU_SWAP   6'd16
`define AP040_ALU_TAS    6'd17
`define AP040_ALU_ABCD   6'd18
`define AP040_ALU_SBCD   6'd19
`define AP040_ALU_NBCD   6'd20
`define AP040_ALU_ASL1   6'd21
`define AP040_ALU_ASR1   6'd22
`define AP040_ALU_LSL1   6'd23
`define AP040_ALU_LSR1   6'd24
`define AP040_ALU_ROL1   6'd25
`define AP040_ALU_ROR1   6'd26
`define AP040_ALU_ROXL1  6'd27
`define AP040_ALU_ROXR1  6'd28
`define AP040_ALU_BTST   6'd29
`define AP040_ALU_BCHG   6'd30
`define AP040_ALU_BCLR   6'd31
`define AP040_ALU_BSET   6'd32

`endif // AP040_DEFS_SVH
