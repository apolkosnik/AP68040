//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_core.v - CPU core: fetch, decode, execute, exceptions              //
//                                                                          //
// Implemented (milestones B-G, see AP040_IMPLEMENTATION_PLAN.md):          //
//  - full 68000/68010 integer set, 68020+ pieces: all extension word EAs   //
//    incl. memory indirect, 32/64-bit MUL/DIV, LINK.L, TRAPcc, bitfields,  //
//    CAS/CAS2, CHK2/CMP2, and the 040 set: MOVE16, MOVEC registers,        //
//    CINV/CPUSH/PFLUSH/PTEST with MMU/cache sidebands                      //
//  - exceptions: formats $0/$1/$2/$3 ($4 RTE-accepted when built FPU-less //
//    but never generated) and format $7 access errors with                 //
//    pure instruction restart and EA register rollback (MMU faults),       //
//    RTE with format validation and $1 throwaway continuation, trace       //
//    (T1/T0), autovectored interrupts with M-bit master/interrupt stack    //
//    switching                                                             //
//  - integrated 68040 FPU arithmetic, conversions, control and condition   //
//    operations; FSAVE/FRESTORE support NULL, IDLE and rev-$41 UNIMP state //
//                                                                          //
// Known gaps, all documented in tests/ap040/README:                        //
//  - TAS/CAS/CAS2 are not bus-locked (single-master fabric here)           //
//  - MMU faults and physical berr both raise format $7; the SSW ATC bit   //
//    distinguishes a translation fault from a physical bus error          //
//  - interrupts are always autovectored (ipl_autovector is ignored)        //
//  - true pipelined arithmetic BUSY state frames are not generated         //
//  - access faults use pure restart; CM/CT and WB2/WB1 are not generated   //
//                                                                          //
// The whole core advances only when ce (clkena_in) is high.                //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_core
#(
	parameter AP040_HAS_MMU      = 1,
	parameter AP040_HAS_FPU      = 0,
	parameter AP040_ENABLE_CACHE = 0,
	parameter AP040_FAST_SIM     = 0
)
(
	input             clk,
	input             nreset,
	input             ce,

	// internal memory transaction to ap040_bus16_adapter
	output reg        mem_req,
	output reg        mem_write,
	output reg        mem_instr,
	output reg  [1:0] mem_size,
	output reg [31:0] mem_addr,
	output reg [31:0] mem_wdata,
	output      [2:0] mem_fc,
	input             mem_ack,
	input      [31:0] mem_rdata,
	input             mem_flt,     // access error pulse from the MMU

	// MMU control register values and PTEST/PFLUSH sideband
	output     [31:0] tc_out,
	output     [31:0] urp_out,
	output     [31:0] srp_out,
	output     [31:0] itt0_out,
	output     [31:0] itt1_out,
	output     [31:0] dtt0_out,
	output     [31:0] dtt1_out,
	output reg        pt_req,
	output reg        pt_write,
	output reg [31:0] pt_addr,
	output      [2:0] pt_fc,
	input             pt_done,
	input      [31:0] pt_mmusr,
	output reg        pf_req,
	output reg  [1:0] pf_mode,
	output reg [31:0] pf_addr,
	output      [2:0] pf_fc,
	input             pf_done,
	output reg        cinv_req,
	output reg        cinv_ic,
	output reg        cinv_dc,
	input             cinv_done,

	input       [2:0] ipl,
	input             ipl_autovector,
	input             berr,
	// Retained event for legacy peripherals which need to observe that a
	// level-7 interrupt was accepted.  A toggle is used so a slower clock
	// enable domain cannot miss the event.
	output            nmi_ack_toggle,

	output            nresetout,
	output     [31:0] cacr_out,
	output     [31:0] vbr_out,

	output            debug_busy,
	output            debug_fault,
	output            debug_halted,
	output    [255:0] debug_status,
	// Second debug bus, for the halt post-mortem beacon: the stack
	// registers and the faulting address.  A double fault taken while
	// stacking an exception frame is only explicable with these -- the
	// active A7 alone cannot say whether the stack switch failed or the
	// supervisor stack pointer was already wrong.
	output    [127:0] debug_status2
);

//---------------------------------------------------------------------------
// architectural state
//---------------------------------------------------------------------------

reg [31:0] pc;          // next word to fetch from the instruction stream
reg [31:0] pc_i;        // address of the current instruction
reg [15:0] sr;
reg [31:0] vbr;
reg [31:0] cacr;
reg  [2:0] sfc, dfc;
reg [31:0] tc;          // 040 TC (E/P bits stored, used at milestone E)
reg [31:0] itt0, itt1, dtt0, dtt1;
reg [31:0] mmusr;
reg [31:0] urp, srp;
reg [15:0] ir;
// FPGA power-up distinguishes the first (cold) reset from later RSTI
// assertions.  The 68040 preserves MMU register contents on reset except for
// the E bits in TC and the four TTRs.
reg        mmu_reset_seen = 1'b0;

wire sr_s = sr[`AP040_SR_S];
wire sr_m = sr[`AP040_SR_M];

assign vbr_out  = vbr;
assign cacr_out = cacr;
assign tc_out   = tc;
assign urp_out  = urp;
assign srp_out  = srp;
assign itt0_out = itt0;
assign itt1_out = itt1;
assign dtt0_out = dtt0;
assign dtt1_out = dtt1;
assign pt_fc    = dfc;
assign pf_fc    = dfc;

// interrupt input synchronization (active low pins, must be stable for two
// consecutive samples like the real part)
reg [2:0] ipl_s1, ipl_s2;
reg [2:0] irq_lvl;
reg [2:0] irq_hold_lvl;          // mask-qualified level retained until accepted
reg       irq_ack_t;             // toggled by the FSM when a level 1-6 IRQ is taken
reg       irq_ack_d;             // sampler-side shadow of irq_ack_t
reg       nmi_arm;
reg       nmi_ack_t;              // toggled by the FSM when an NMI is taken
reg       nmi_ack_d;              // sampler-side shadow of nmi_ack_t
assign nmi_ack_toggle = nmi_ack_t;
always @(posedge clk) begin
	if (!nreset) begin
		ipl_s1 <= 3'b111;
		ipl_s2 <= 3'b111;
		irq_lvl <= 3'd0;
		irq_hold_lvl <= 3'd0;
		irq_ack_d <= 1'b0;
		nmi_arm <= 1'b0;
		nmi_ack_d <= 1'b0;
	end
	else begin
		ipl_s1 <= ipl;
		ipl_s2 <= ipl_s1;
		if (ipl_s1 == ipl_s2) begin
			irq_lvl <= ~ipl_s2;
			// Re-arm the edge-triggered level 7 as soon as the pins leave
			// it.  This tracks the pins rather than instruction execution,
			// so an NMI cannot be missed because the CPU happened to be
			// stalled waiting on memory (nmi_ack clears it on acceptance).
			if (~ipl_s2 != 3'd7) nmi_arm <= 1;
		end
		// Once a mask-qualified level 1-6 request has reached IPEND, an
		// instruction which subsequently raises the SR mask must not make it
		// disappear.  Retain the highest sampled level until exception entry
		// acknowledges it.  This is separate from level 7's edge latch.
		irq_ack_d <= irq_ack_t;
		if (irq_ack_t != irq_ack_d)
			irq_hold_lvl <= 3'd0;
		// The hold exists to survive a MASK change, not the source going
		// away.  A 68040 requires the requesting device to keep IPL
		// asserted until the CPU acknowledges; if it drops the request
		// first, the interrupt is simply not taken.  Track the pins down
		// so a withdrawn request cannot fire later as a phantom
		// interrupt at the next instruction boundary.  The level the
		// encoder falls back to is a DIFFERENT device's request that was
		// hidden behind the withdrawn one: it must qualify against the
		// mask on its own, or the hold would fire it at or below the
		// mask (levels 1-6 are taken only strictly above SR[10:8]).
		else if (irq_hold_lvl > irq_lvl)
			irq_hold_lvl <= (irq_lvl > sr[10:8]) ? irq_lvl : 3'd0;
		else if (irq_lvl != 3'd0 && irq_lvl != 3'd7 &&
		         irq_lvl > sr[10:8] && irq_lvl > irq_hold_lvl)
			irq_hold_lvl <= irq_lvl;

		// acceptance wins over re-arming in the same cycle
		nmi_ack_d <= nmi_ack_t;
		if (nmi_ack_t != nmi_ack_d) nmi_arm <= 0;
	end
end

// Recognise the level one cycle earlier than the registered irq_lvl.
// Both samples are synchroniser flops, so the coherence check that guards
// against reading a transient encoder value (3 -> 5 passing through 7
// would look like an NMI) still holds; only the extra register stage is
// removed.  cputest's irq tests time the request to land just before an
// instruction boundary, and the third cycle pushed recognition past it --
// hardware stacked the address AFTER the tested instruction.
wire [2:0] irq_lvl_live = (ipl_s1 == ipl_s2) ? ~ipl_s2 : irq_lvl;

wire       nmi_pend = irq_lvl_live == 3'd7 && nmi_arm;
wire       irq_live = irq_lvl_live != 3'd0 && irq_lvl_live != 3'd7 &&
                      irq_lvl_live > sr[10:8];
wire [2:0] irq_take_lvl = nmi_pend ? 3'd7 :
                          (irq_live && irq_lvl_live > irq_hold_lvl)
                              ? irq_lvl_live : irq_hold_lvl;
wire       irq_pend = nmi_pend || irq_live || irq_hold_lvl != 3'd0;


wire unused_in = ipl_autovector;

// An access error comes either from the MMU (translation fault) or from the
// bus (a physical bus error: no device answered).  Both are only sampled
// while a transfer is outstanding, which is the only time they are tested.
wire mem_err = mem_req && (mem_flt | berr);

// X2.2b stage 1: the memory port carries two channels, and mem_instr tags
// which one owns the transaction in flight -- aerr_start already builds the
// fault frame from it.  What was NOT explicit is the acknowledge: a data
// state reaches its ack branch only when m_issued, and m_issued can only be
// set while !epf_pend, so today an ack is attributed by construction rather
// than by inspection.  That construction IS the serialization stage 2
// removes, at which point an unqualified mem_ack would be delivered to
// whichever channel happened to be looking.  Qualify both channels now,
// while the stall still guarantees the answer, so the change that matters
// later is not also the change that introduces the qualification.
//
// Behaviour is identical today: epf_pend is set only by instruction issues
// (issue_ifetch's port-free branch and the fill engine, both mem_instr=1),
// exception_prefetch owns the port outright on the exception path, and the
// data states set mem_instr=0 at their own issue.
wire d_ack = mem_ack && !mem_instr;   // data channel acknowledge
wire i_ack = mem_ack &&  mem_instr;   // instruction channel acknowledge
wire d_err = mem_err && !mem_instr;
wire i_err = mem_err &&  mem_instr;

//---------------------------------------------------------------------------
// register file
//---------------------------------------------------------------------------

reg         rf_we;
reg   [3:0] rf_waddr;
reg  [31:0] rf_wdata;
reg   [3:0] rr_a;
reg   [3:0] rr_b;
wire [31:0] rf_rdata_a;
wire [31:0] rf_rdata_b;
reg         aux_we;
reg   [1:0] aux_sel;
reg  [31:0] aux_wdata;
wire [31:0] usp_q, isp_q, msp_q;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_a0, dbg_a7;

ap040_regfile regfile
(
	.clk(clk), .ce(ce), .nreset(nreset),
	.sr_s(sr_s), .sr_m(sr_m),
	.we(rf_we), .waddr(rf_waddr), .wdata(rf_wdata),
	.raddr_a(rr_a), .rdata_a(rf_rdata_a),
	.raddr_b(rr_b), .rdata_b(rf_rdata_b),
	.aux_we(aux_we), .aux_sel(aux_sel), .aux_wdata(aux_wdata),
	.usp_q(usp_q), .isp_q(isp_q), .msp_q(msp_q),
	.dbg_d0(dbg_d0), .dbg_d1(dbg_d1), .dbg_d2(dbg_d2),
	.dbg_a0(dbg_a0), .dbg_a7(dbg_a7)
);

//---------------------------------------------------------------------------
// ALU and multiply/divide
//---------------------------------------------------------------------------

reg   [5:0] alu_op;
reg   [1:0] op_size;
reg  [31:0] src_val, dst_val;
reg  [31:0] sh_val;
reg   [4:0] sh_fl;
reg   [7:0] state;
wire [31:0] alu_res;
wire  [4:0] alu_fl;

localparam S_SHIFT      = 8'd29;   // forward declaration for the alu_b mux
// hoisted like the localparam: the ALU instance below consumes it
reg  [5:0] sh_cnt;

wire alu_is_bitop = (alu_op >= `AP040_ALU_BTST) && (alu_op <= `AP040_ALU_BSET);
reg         p_sextw;
reg         p_dst_mem_bit;    // bit op destination is memory (modulo 8)

wire [31:0] alu_a = alu_is_bitop ? (p_dst_mem_bit ? {29'd0, src_val[2:0]}
                                                  : {27'd0, src_val[4:0]}) :
                    p_sextw      ? {{16{src_val[15]}}, src_val[15:0]} : src_val;
wire [31:0] alu_b = (state == S_SHIFT) ? sh_val : dst_val;
wire  [4:0] alu_fin = (state == S_SHIFT) ? sh_fl : sr[4:0];

ap040_alu alu
(
	.op(alu_op), .size(op_size),
	.a(alu_a), .b(alu_b),
	.flags_in(alu_fin),
	.shcnt((state == S_SHIFT) ? sh_cnt : 6'd1),
	.result(alu_res), .flags_out(alu_fl)
);

reg         md_start, md_isdiv, md_sign;
reg  [31:0] md_a, md_hi, md_lo;
wire        md_done, md_ovf;
wire [31:0] md_rhi, md_rlo;

ap040_muldiv muldiv
(
	.clk(clk), .nreset(nreset), .ce(ce),
	.start(md_start), .is_div(md_isdiv), .sign_op(md_sign),
	.op_a(md_a), .op_hi(md_hi), .op_lo(md_lo),
	.done(md_done), .res_hi(md_rhi), .res_lo(md_rlo), .ovf(md_ovf)
);

//---------------------------------------------------------------------------
// states
//---------------------------------------------------------------------------

localparam S_START      = 8'd0;
localparam S_BOOT0      = 8'd1;
localparam S_BOOT1      = 8'd2;
localparam S_FETCH      = 8'd3;
localparam S_DECODE      = 8'd4;
localparam S_NEXT      = 8'd5;
localparam S_HALT      = 8'd6;
localparam S_STOPPED      = 8'd7;
localparam S_IMMF      = 8'd8;
localparam S_MRD      = 8'd9;
localparam S_MWR      = 8'd10;
localparam S_EA_DISP      = 8'd11;
localparam S_EA_BASE      = 8'd12;
localparam S_EA_D16      = 8'd13;
localparam S_EA_EXTW      = 8'd14;
localparam S_EA_EXTW2      = 8'd15;
localparam S_EA_BD      = 8'd16;
localparam S_EA_MIND      = 8'd17;
localparam S_EA_OD      = 8'd18;
localparam S_EA_ABS      = 8'd19;
localparam S_PIPE_START      = 8'd20;
localparam S_PIPE_SREG      = 8'd21;
localparam S_PIPE_SRD      = 8'd22;
localparam S_PIPE_SDONE      = 8'd23;
localparam S_PIPE_DST      = 8'd24;
localparam S_PIPE_DREG      = 8'd25;
// X2.3 step 1: both regfile read ports are independent and combinational,
// so a register source and a register destination can be fetched in ONE
// cycle instead of walking SREG then DST then DREG.
localparam S_PIPE_REGS      = 8'd190;
localparam S_PIPE_DEA      = 8'd26;
localparam S_PIPE_DDONE      = 8'd27;
localparam S_EXEC      = 8'd28;
// S_SHIFT = 29 declared above
localparam S_MD_WAIT      = 8'd30;
localparam S_MD_WB2      = 8'd31;
localparam S_MDL_RDR      = 8'd32;
localparam S_MDL_GO      = 8'd33;
localparam S_EXC0      = 8'd34;
localparam S_EXC1      = 8'd35;
localparam S_EXC2      = 8'd36;
localparam S_EXC3      = 8'd37;
localparam S_EXC4      = 8'd38;
localparam S_EXC5      = 8'd39;
localparam S_EXC6      = 8'd40;
localparam S_EXC_VEC      = 8'd41;
localparam S_EXC_JMP      = 8'd42;
localparam S_RTE_SR      = 8'd43;
localparam S_RTE_PC      = 8'd44;
localparam S_RTE_FMT      = 8'd45;
localparam S_RTE_FIN      = 8'd46;
localparam S_RTE_FIN2      = 8'd47;
localparam S_RET1      = 8'd48;
localparam S_RET2      = 8'd49;
localparam S_RET3      = 8'd50;
localparam S_BCC_EXT      = 8'd51;
localparam S_BSR_PUSH      = 8'd52;
localparam S_DBCC1      = 8'd53;
localparam S_DBCC2      = 8'd54;
localparam S_JMP1      = 8'd55;
localparam S_JSR1      = 8'd56;
localparam S_JSR2      = 8'd57;
localparam S_LEA1      = 8'd58;
localparam S_PEA1      = 8'd59;
localparam S_PEA2      = 8'd60;
localparam S_LINK1      = 8'd61;
localparam S_LINK2      = 8'd62;
localparam S_LINK3      = 8'd63;
localparam S_LINK4      = 8'd64;
localparam S_UNLK1      = 8'd65;
localparam S_UNLK2      = 8'd66;
localparam S_UNLK3      = 8'd67;
localparam S_MOVEM_SET      = 8'd68;
localparam S_MOVEM_SET2      = 8'd69;
localparam S_MOVEM_LOOP      = 8'd70;
localparam S_MOVEM_RD      = 8'd71;
localparam S_MOVEM_WR      = 8'd72;
localparam S_MOVEM_LD      = 8'd73;
localparam S_MOVEP1      = 8'd74;
localparam S_MOVEP2      = 8'd75;
localparam S_MOVEP_WR      = 8'd76;
localparam S_MOVEP_RD      = 8'd77;
localparam S_EXG1      = 8'd78;
localparam S_EXG2      = 8'd79;
localparam S_USP1      = 8'd80;
localparam S_MOVEC1      = 8'd81;
localparam S_MOVEC2      = 8'd82;
localparam S_MOVES1      = 8'd83;
localparam S_MOVES2      = 8'd84;
localparam S_MOVES_WR      = 8'd85;
localparam S_MOVES_RD      = 8'd86;
localparam S_PTEST1      = 8'd87;
localparam S_RESET_HOLD      = 8'd88;
localparam S_M16_SRC      = 8'd89;
localparam S_M16_DST      = 8'd90;
localparam S_M16_DST2      = 8'd91;
localparam S_M16_RD      = 8'd92;
localparam S_M16_RD2      = 8'd93;
localparam S_M16_WR      = 8'd94;
localparam S_M16_WR2      = 8'd95;
localparam S_M16_INC      = 8'd96;
localparam S_M16_INC2      = 8'd97;
localparam S_SROP      = 8'd98;
localparam S_SHIFT_WB      = 8'd99;
localparam S_TRAPCC      = 8'd100;
localparam S_STOP_LD      = 8'd101;
localparam S_MOVEM_EA      = 8'd102;
localparam S_MDL_RDQ      = 8'd103;
localparam S_MDL_EXT      = 8'd104;
localparam S_PFLUSH1      = 8'd105;
localparam S_PFLUSH2      = 8'd106;
localparam S_PTEST2      = 8'd107;
localparam S_AERR0      = 8'd108;
localparam S_AERR_U      = 8'd109;
localparam S_AERR_SP      = 8'd110;
localparam S_AERR_WR      = 8'd111;
localparam S_BF0      = 8'd112;
localparam S_BF1      = 8'd113;
localparam S_BF_REG      = 8'd114;
localparam S_BF_REG2      = 8'd115;
localparam S_BF_REGX      = 8'd116;
localparam S_BF_MEM0      = 8'd117;
localparam S_BF_MEM1      = 8'd118;
localparam S_BF_MEM2      = 8'd119;
localparam S_BF_EXECM      = 8'd120;
localparam S_BF_WR1      = 8'd121;
localparam S_BF_WR2      = 8'd122;
localparam S_CAS1      = 8'd123;
localparam S_CAS2      = 8'd124;
localparam S_CAS3      = 8'd125;
localparam S_CAS4      = 8'd126;
localparam S_BF_X2     = 8'd127;
localparam S_BF_X3     = 8'd128;
localparam S_BF_X4     = 8'd129;
localparam S_BF_M2     = 8'd130;
localparam S_BF_M3     = 8'd131;
localparam S_BF_M4     = 8'd132;
localparam S_CINV2     = 8'd133;
localparam S_CHK2_A    = 8'd134;
localparam S_CHK2_B    = 8'd135;
localparam S_CHK2_C    = 8'd136;
localparam S_CHK2_D    = 8'd137;
localparam S_BTSTI     = 8'd138;
localparam S_BTSTI2    = 8'd139;
localparam S_CAS2_0    = 8'd140;
localparam S_CAS2_1    = 8'd141;
localparam S_CAS2_2    = 8'd142;
localparam S_CAS2_3    = 8'd143;
localparam S_CAS2_4    = 8'd144;
localparam S_CAS2_5    = 8'd145;
localparam S_CAS2_6    = 8'd146;
localparam S_CAS2_W2   = 8'd147;
localparam S_CAS2_W3   = 8'd148;
localparam S_CAS2_F    = 8'd149;
localparam S_CAS2_F2   = 8'd150;
localparam S_FSAVE1    = 8'd151;
localparam S_FREST1    = 8'd152;
localparam S_FPU_DEC   = 8'd153;
localparam S_FPU_AN    = 8'd154;
localparam S_FPU_EA    = 8'd155;
localparam S_FPU_DREG  = 8'd156;
localparam S_FPU_IMM   = 8'd157;
localparam S_FPU_RD    = 8'd158;
localparam S_FPU_RD2   = 8'd159;
localparam S_FPU_GO    = 8'd160;
localparam S_FPU_WR    = 8'd161;
localparam S_FPU_CR    = 8'd162;
localparam S_FPU_CR2   = 8'd163;
localparam S_FPU_MVM   = 8'd164;
localparam S_FPU_MVM2  = 8'd165;
localparam S_FPU_MVM3  = 8'd166;
localparam S_FBCC      = 8'd167;
localparam S_FSCC0     = 8'd168;
localparam S_FSCC1     = 8'd169;
localparam S_FDBCC     = 8'd170;
localparam S_FREST2    = 8'd171;
localparam S_FPU_MVML  = 8'd172;
localparam S_FPU_CRD   = 8'd173;
localparam S_EXC4B     = 8'd174;
localparam S_MRD_B     = 8'd175;
localparam S_MWR_B     = 8'd176;
localparam S_FPU_CRI   = 8'd177;
localparam S_EPF_FILL  = 8'd178;
localparam S_EPF_GAP   = 8'd179;
localparam S_EPF_READY = 8'd180;
localparam S_POST_EXC  = 8'd181;
localparam S_FSAVE_U   = 8'd182;
localparam S_FSAVE_UD  = 8'd183;
localparam S_FREST_U   = 8'd184;
localparam S_FREST_UD  = 8'd185;
localparam S_FSAVE_B   = 8'd186;
localparam S_FSAVE_BD  = 8'd187;
localparam S_FREST_B   = 8'd188;
localparam S_FREST_BD  = 8'd189;

// exec kinds
localparam EK_ALU     = 4'd0;
localparam EK_SHIFT   = 4'd1;
localparam EK_MD_W    = 4'd2;   // word multiply/divide
localparam EK_MD_L    = 4'd3;   // long multiply/divide (extension in x_ext)
localparam EK_CHK     = 4'd4;
localparam EK_SCC     = 4'd5;
localparam EK_PACK    = 4'd6;
localparam EK_UNPK    = 4'd7;

// src/dst kinds
localparam SK_NONE = 3'd0;
localparam SK_REG  = 3'd1;
localparam SK_IMM  = 3'd2;
localparam SK_MEM  = 3'd3;
localparam SK_IMPL = 3'd4;   // src_val preloaded at decode

localparam DK_NONE = 3'd0;
localparam DK_REG  = 3'd1;
localparam DK_MEM  = 3'd2;
localparam DK_SR   = 3'd3;
localparam DK_CCR  = 3'd4;

// ret_kind for RTS/RTR/RTD
localparam RK_RTS = 2'd0;
localparam RK_RTR = 2'd1;
localparam RK_RTD = 2'd2;

//---------------------------------------------------------------------------
// control registers of the execution engine
//---------------------------------------------------------------------------

reg  [7:0] r_imm_ret, r_ea_ret, r_m_ret;
reg  [2:0] m_bidx;                // byte index of a split transfer
reg [31:0] m_acc;                 // assembled bytes of a split transfer
reg  [1:0] imm_n;
reg        m_issued;
reg [31:0] imm;
reg [31:0] x_ext;              // saved copy of imm (survives EA fetches)

// Instruction fetch queue.  The 68040 does not begin handler execution until
// four longwords have been fetched; that architectural requirement built this
// eight-word FIFO, and the same storage now carries every instruction word.
// The fill engine at the end of the state machine appends to the tail
// whenever the memory port is idle, S_FETCH and S_IMMF pop the head, and any
// flow, context or FC change flushes the whole thing.
reg [15:0] epf_data [0:7];
reg  [3:0] epf_count;            // words resident
reg  [2:0] epf_head, epf_fill;   // pop / append indices
reg [31:0] epf_base;             // exception prefetch origin
reg [31:0] epf_next;             // address of the word at the head
reg [31:0] epf_ftail;            // address of the next word to be fetched
reg        epf_super;            // FC the queue was filled under
reg        epf_armed;            // the fill engine owns this stream
reg        epf_pend;             // a queue fetch is outstanding
reg        epf_pend_lw;          // ... and it returns two words
reg        epf_kill;             // ... whose data a flush has abandoned
reg        epf_err;              // the fill engine faulted: re-issue on demand

// Combinational within the state machine's always block: the port claim and
// the flush both have to be visible to the fill engine, which runs after the
// case so that a state claiming the port this cycle always wins it.
reg        epf_issue;            // the port was claimed by a state this cycle
reg        epf_flushed;          // the queue was flushed this cycle
reg  [1:0] epf_pop;              // words consumed this cycle
reg  [1:0] epf_fillw;            // words appended this cycle

// A resident word needs no bus cycle at all, so a fetch consumes it in the
// very cycle it would otherwise have spent issuing a request.
wire       epf_ready_pc = epf_armed && (epf_count != 4'd0) &&
                          (epf_next == pc) && (epf_super == sr_s);
wire       epf_ready_pc2 = epf_armed && (epf_count > 4'd1) &&
                           (epf_next == pc) && (epf_super == sr_s);
// The word arriving this cycle bypasses the queue: a fetch that ran the
// queue dry still completes in the acknowledge cycle, exactly as the
// pre-queue demand fetch did.
wire       epf_fwd_pc = epf_pend && i_ack && !epf_kill && epf_armed &&
                        (epf_count == 4'd0) && (epf_next == mem_addr) &&
                        (epf_next == pc) && (epf_super == sr_s);
wire [15:0] epf_fwd_word = epf_pend_lw ? mem_rdata[31:16] : mem_rdata[15:0];

reg        m_wr;
reg  [1:0] m_size;
reg [31:0] m_addr_r, m_wdat, m_val;

reg  [2:0] ea_mode;
reg  [2:0] ea_rn;
reg  [1:0] ea_size;
reg        ea_pcmode;
reg [31:0] ea_pcb;
reg [15:0] extw;
reg [31:0] ea_base_v, ea_idx_v, ea_mind;
reg        ea_post, ea_odl, ea_absl;
reg [31:0] ea_addr;

reg  [2:0] p_src, p_dst;
reg  [3:0] p_sreg, p_dreg;
reg  [1:0] p_ssize, p_dsize;
reg        p_rmw, p_wbsup, p_flags;
reg  [3:0] exec_kind;
reg  [2:0] src_mode_r, src_rn_r, dst_mode_r, dst_rn_r;
reg [31:0] dst_addr;

reg        sh_vacc;
reg        sh_rox;
reg        sh_any;

reg  [7:0] exc_vec;
reg  [3:0] exc_fmt;
reg [31:0] exc_spc, exc_addr, exc_sp;
reg        exc_is_irq, exc_pass2;
reg [15:0] sr_saved;
reg        texc_pend;            // 040: T0 trace survives a non-internal
reg [31:0] texc_pc;              //   integer exception; fires at handler
reg        flow_t0_pend;         // T0 redirect waits for target predecode
reg [31:0] flow_t0_oldpc;
reg  [2:0] irq_lvl_l;

reg [15:0] rte_sr;
// RTE restores the SR and redirects in one step, so a request that the
// popped mask unblocks has to be qualified against THAT mask -- the sr
// register still holds the pre-RTE value in the cycle the decision is
// made.  Everything else is identical to the wires above.
wire       rte_irq_live = irq_lvl_live != 3'd0 && irq_lvl_live != 3'd7 &&
                          irq_lvl_live > rte_sr[10:8];
wire       rte_irq_hold = irq_hold_lvl != 3'd0 &&
                          irq_hold_lvl > rte_sr[10:8];
wire       rte_irq_pend = nmi_pend || rte_irq_live || rte_irq_hold;
wire [2:0] rte_irq_lvl  = nmi_pend ? 3'd7 :
                          (rte_irq_live && irq_lvl_live > irq_hold_lvl)
                              ? irq_lvl_live : irq_hold_lvl;
reg [31:0] rte_pc;

reg  [1:0] ret_kind;
reg [31:0] br_base, br_tgt;
reg        br_long;

reg [15:0] mm_mask;
reg        mm_dir;             // 1 = mem to reg
reg        mm_predec, mm_postinc;
reg  [1:0] mm_size;
reg [31:0] mm_addr, mm_init_an;
reg        mm_base_ea;            // the EA depends on An (see S_MOVEM_LD)
reg        mm_base_pend;          // a loaded base register awaits commit
reg [31:0] mm_base_val;           // its value, written with the last transfer
reg  [3:0] mm_reg;

reg  [2:0] mp_cnt, mp_idx;     // byte counts: 2 for word, 4 for long
reg        mp_dir;             // 1 = reg to mem
reg [31:0] mp_addr, mp_val;

reg [31:0] t_a, t_b;
reg  [1:0] srop_kind;          // 0 OR, 1 AND, 2 EOR
reg        srop_sr;            // to SR (else CCR)

reg        mvc_dir;            // 1 = general to control
reg        fc_ovr_v;
reg        lk_cyc;               // TAS/CAS/CAS2 locked-RMW operand cycle
reg        aer_lk, aer_m16;      // SSW LK; MOVE16 line-fault shape
reg  [1:0] aer_tt;               // SSW TT (MOVES to FC 0/3/4/7 reports TT1)
reg [31:0] aer_wd;               // faulted write data for the WB3 slot
reg  [2:0] fc_ovr;

reg  [2:0] m16_form;
reg  [2:0] m16_dst_rn;
reg [31:0] m16_src, m16_dst, m16_an;
reg  [1:0] m16_idx;
reg [31:0] m16buf [0:3];
reg        m16_rd_done;

reg  [7:0] rst_cnt;
reg        fault_r;
reg  [2:0] fc_r;

// Trace bits sampled when the instruction starts.  T0 includes actual
// control-flow changes plus the 68040's pipeline-synchronizing instruction
// list (NOP, SR/control-register changes, cache/MMU maintenance, etc.).
reg        tr_t1, tr_t0, t0_force;

// bitfield and CAS working registers. The bitfield datapath is spread
// over several states so each stage holds at most one wide variable
// shifter (timing: a single-cycle version costs ~28ns of logic).
reg [31:0] bf_off;
reg  [5:0] bf_w;
reg [31:0] bf_addr;
reg  [2:0] bf_bib;              // bit offset inside the first byte
reg  [2:0] bf_span;             // bytes touched (1..5)
reg [31:0] bf_w1;
reg  [7:0] bf_w2;
reg [31:0] bf_du;
reg [39:0] bf_t40;              // shifted window (mem) / rotated reg (reg form)
reg [31:0] bf_field;            // extracted field, right aligned
reg [31:0] bf_ones;             // width ones mask, right aligned
reg [39:0] bf_maskl;            // field mask, left aligned in the work domain
reg [31:0] cas_dc;

// access error (format $7) context and EA register-update rollback
reg        in_exc;              // exception stacking in progress
reg [31:0] aer_fa, aer_sp;
reg        aer_bus;          // fault came from the bus, not the ATC
reg        aer_ma;           // ATC fault occurred on second page of transfer
reg        aer_wr;
reg  [1:0] aer_sz;
reg  [2:0] aer_tm;
reg  [4:0] aer_idx;
reg        u0_v, u1_v;
reg  [3:0] u0_reg, u1_reg;
reg [31:0] u0_old, u1_old;

assign mem_fc    = fc_r;
assign nresetout = (state != S_RESET_HOLD);

//---------------------------------------------------------------------------
// decode helpers (combinational, from ir)
//---------------------------------------------------------------------------

wire [3:0] ir_hi     = ir[15:12];
wire [2:0] d_reg9    = ir[11:9];
wire [2:0] d_op8_6   = ir[8:6];
wire [2:0] d_mode    = ir[5:3];
wire [2:0] d_rn      = ir[2:0];

// MOVE sizes: 01=B 11=W 10=L
wire [1:0] move_size = (ir[13:12] == 2'b01) ? `AP040_SZ_B :
                       (ir[13:12] == 2'b11) ? `AP040_SZ_W : `AP040_SZ_L;
wire [1:0] std_size  = ir[7:6];   // 00=B 01=W 10=L

wire ea_is_imm     = (d_mode == 3'b111) && (d_rn == 3'b100);
// Destination is not data alterable: an address register, program space
// (both PC-relative encodings) or an immediate operand.
wire dst_not_alt   = (d_mode == 3'b001) ||
                     (d_mode == 3'b111 && d_rn > 3'b001);
// Source is not a data-addressing mode: An direct and the three reserved
// mode-7 register values.  PC-relative and immediate remain valid sources.
wire src_not_data  = (d_mode == 3'b001) ||
                     (d_mode == 3'b111 && d_rn > 3'b100);

function [31:0] sxw;
	input [15:0] v;
	begin sxw = {{16{v[15]}}, v}; end
endfunction

function [31:0] sxb;
	input [7:0] v;
	begin sxb = {{24{v[7]}}, v}; end
endfunction

function [31:0] merge_sz;
	input [31:0] old;
	input [31:0] v;
	input [1:0] size;
	begin
		case (size)
			`AP040_SZ_B: merge_sz = {old[31:8],  v[7:0]};
			`AP040_SZ_W: merge_sz = {old[31:16], v[15:0]};
			default:     merge_sz = v;
		endcase
	end
endfunction

function [31:0] an_adj;
	input [2:0] regn;
	input [1:0] size;
	begin
		if (size == `AP040_SZ_B && regn == 3'd7) an_adj = 32'd2;
		else an_adj = {29'd0, (size == `AP040_SZ_B) ? 3'd1 :
		                      (size == `AP040_SZ_W) ? 3'd2 : 3'd4};
	end
endfunction

function cond_true;
	input [3:0] cond;
	begin
		case (cond)
			4'h0: cond_true = 1;
			4'h1: cond_true = 0;
			4'h2: cond_true = !sr[0] && !sr[2];
			4'h3: cond_true =  sr[0] ||  sr[2];
			4'h4: cond_true = !sr[0];
			4'h5: cond_true =  sr[0];
			4'h6: cond_true = !sr[2];
			4'h7: cond_true =  sr[2];
			4'h8: cond_true = !sr[1];
			4'h9: cond_true =  sr[1];
			4'hA: cond_true = !sr[3];
			4'hB: cond_true =  sr[3];
			4'hC: cond_true =  sr[3] ==  sr[1];
			4'hD: cond_true =  sr[3] !=  sr[1];
			4'hE: cond_true = !sr[2] && (sr[3] == sr[1]);
			default: cond_true = sr[2] || (sr[3] != sr[1]);
		endcase
	end
endfunction

function [3:0] ffs16;
	input [15:0] m;
	integer k;
	begin
		ffs16 = 4'd0;
		for (k = 15; k >= 0; k = k - 1)
			if (m[k]) ffs16 = k[3:0];
	end
endfunction

function [31:0] rotl32;
	input [31:0] v;
	input [4:0] n;
	begin
		rotl32 = (n == 0) ? v : ((v << n) | (v >> (6'd32 - {1'b0, n})));
	end
endfunction

function [31:0] rotr32;
	input [31:0] v;
	input [4:0] n;
	begin
		rotr32 = (n == 0) ? v : ((v >> n) | (v << (6'd32 - {1'b0, n})));
	end
endfunction

// fixed 32-bit leading zero count (shallow priority encoder)
function [5:0] clz32;
	input [31:0] v;
	integer k;
	begin
		clz32 = 6'd32;
		for (k = 0; k < 32; k = k + 1)
			if (v[k]) clz32 = 6'd31 - k[5:0];
	end
endfunction

// new right-aligned field value per bitfield operation; du must already be
// masked to the field width, ones is the width mask (no shifters in here)
function [31:0] bf_newf;
	input [2:0] op;      // ir[10:8]
	input [31:0] field;
	input [31:0] du;
	input [31:0] ones;
	begin
		case (op)
			3'd2: bf_newf = (~field) & ones;            // BFCHG
			3'd4: bf_newf = 32'd0;                      // BFCLR
			3'd6: bf_newf = ones;                       // BFSET
			default: bf_newf = du;                      // BFINS
		endcase
	end
endfunction

// MOVEC control register read mux
function [31:0] movec_rd;
	input [11:0] code;
	begin
		case (code)
			12'h000: movec_rd = {29'd0, sfc};
			12'h001: movec_rd = {29'd0, dfc};
			12'h002: movec_rd = cacr;
			12'h003: movec_rd = tc;
			12'h004: movec_rd = itt0;
			12'h005: movec_rd = itt1;
			12'h006: movec_rd = dtt0;
			12'h007: movec_rd = dtt1;
			12'h800: movec_rd = usp_q;
			12'h801: movec_rd = vbr;
			12'h803: movec_rd = msp_q;
			12'h804: movec_rd = isp_q;
			12'h805: movec_rd = mmusr;
			12'h806: movec_rd = urp;
			12'h807: movec_rd = srp;
			default: movec_rd = 32'd0;
		endcase
	end
endfunction

function movec_valid;
	input [11:0] code;
	begin
		case (code)
			12'h000, 12'h001, 12'h002, 12'h003, 12'h004, 12'h005, 12'h006,
			12'h007, 12'h800, 12'h801, 12'h803, 12'h804, 12'h805, 12'h806,
			12'h807: movec_valid = 1;
			default: movec_valid = 0;
		endcase
	end
endfunction

//---------------------------------------------------------------------------
// micro operation tasks (all nonblocking assignments)
//---------------------------------------------------------------------------

// s = supervisor bit of the context the fetch belongs to. Passed
// explicitly because RTE restores SR in its dispatch cycle: the first
// fetch of the restored context must use its FC, not the handler's
// (the MMU translates user and supervisor code through different
// roots, and a faulting fetch reports this FC in the SSW TM field).
//---------------------------------------------------------------------------
// FPU (milestone H): the core owns decode, EAs and memory traffic; the
// ap040_fpu engine owns registers, conversion and arithmetic
//---------------------------------------------------------------------------

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
reg   [1:0] fp_cnt;               // long transfers remaining
reg   [3:0] fp_nb;                // operand bytes
reg         fp_st;                // 1: store direction
reg         fp_st_epend;          // enabled store exception: deliver post-write
reg   [7:0] fp_st_evec;           // its vector
reg   [7:0] fp_list;              // FMOVEM register list (as processed)
reg   [1:0] fp_mode;              // FMOVEM mode bits
reg   [2:0] fp_creg;              // control list bits {FPCR,FPSR,FPIAR}
reg   [5:0] fp_pred;              // FScc/FDBcc/FTRAPcc predicate
reg         fp_rev;               // FMOVEM 040 quirk: reverse longword order
reg         fp_lsb;               // FMOVEM predec store: mask consumed LSB first
reg   [3:0] fp_n;                 // loop index
reg         fp_ea_pd, fp_ea_pi;   // predecrement / postincrement EA
reg         fp_ea_v;              // t_a holds a resolved operand address
reg         fp_force_unsupp;      // packed store: trap after resolving EA
reg   [6:0] fp_adj;               // total An adjustment (up to 8*12 = 96 bytes)

wire        fpu_done, fpu_unimp, fpu_unsupp, fpu_exc_req, fpu_used;
wire        fpu_accepted;
// Background (released) FPU operation tracking: the core continues
// integer execution while the FPU finishes register-destination
// arithmetic.  An enabled arithmetic exception from a released op is
// held pending and delivered pre-instruction at the next FPU dispatch.
reg         fpu_bg;
reg         fpu_pend_exc;
reg   [7:0] fpu_pend_vec;
wire        fpu_fstate_unimp;
wire  [7:0] fpu_cur_vec;
wire        fpu_frestore_e1_pend;
wire  [2:0] fpu_fstate_grs;
wire        fpu_fstate_wbte15;
reg         fpu_pendcap;
reg   [2:0] fp_restore_grs;
reg         fp_restore_wbte15;
reg  [95:0] fp_restore_wbt;
reg  [31:0] fp_restore_fpiar;
reg         fp_restore_busy;
reg   [4:0] fpb_n;
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

generate if (AP040_HAS_FPU) begin : g_fpu
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
end else begin : g_nofpu
	assign fpu_done = 0;
	assign fpu_accepted = 0;
	assign fpu_unimp = 0;
	assign fpu_unsupp = 0;
	assign fpu_exc_req = 0;
	assign fpu_exc_vec = 0;
	assign fpu_bsun_en = 0;
	assign fpu_dout = 0;
	assign fpu_cc = 0;
	assign fpu_crrd = 0;
	assign fpu_fmrd = 0;
	assign fpu_used = 0;
	assign fpu_fstate_unimp = 0;
	assign fpu_cur_vec = 0;
	assign fpu_frestore_e1_pend = 0;
	assign fpu_fstate_grs = 0;
	assign fpu_fstate_wbte15 = 0;
	assign fpu_fstate_busy = 0;
	assign fpu_fstate_wbt = 0;
	assign fpu_fstate_fpiar_c = 0;
	assign fpu_fstate_cmd1 = 0;
	assign fpu_fstate_cmd3 = 0;
	assign fpu_fstate_stag = 0;
	assign fpu_fstate_dtag = 0;
	assign fpu_fstate_flags = 0;
	assign fpu_fstate_fpt = 0;
	assign fpu_fstate_et = 0;
end endgenerate

// operand byte count per source format field
function [3:0] fp_bytes;
	input [2:0] fmt;
	begin
		case (fmt)
			3'd0, 3'd1: fp_bytes = 4;         // L, S
			3'd4:       fp_bytes = 2;         // W
			3'd6:       fp_bytes = 1;         // B
			3'd5:       fp_bytes = 8;         // D
			default:    fp_bytes = 12;        // X, P
		endcase
	end
endfunction

// Revision-$41 MC68040 unimplemented-instruction frame.  The size field in
// the header is the payload size (48 bytes), making 13 longwords total.
// the $41/$60 BUSY frame: 25 longwords, offsets per WinUAE's 68040
// fpuop_save writer (WBTEMP at +24, FPIARCU at +40, CMDREG3B at +52,
// STAG/GRS at +60, CMDREG1B at +64, DTAG/WBTE15 at +68, flags at +72,
// FPTEMP at +76, ETEMP at +88; CU_SAVEPC and the reserved words zero)
function [31:0] fsave_busy_word;
	input [4:0] n;
	begin
		case (n)
			5'd0:  fsave_busy_word = 32'h4160_0000;
			5'd6:  fsave_busy_word = {fpu_fstate_wbt[95:80], 16'd0};
			5'd7:  fsave_busy_word = fpu_fstate_wbt[63:32];
			5'd8:  fsave_busy_word = fpu_fstate_wbt[31:0];
			5'd10: fsave_busy_word = fpu_fstate_fpiar_c;
			5'd13: fsave_busy_word = {fpu_fstate_cmd3, 16'd0};
			5'd15: fsave_busy_word = {fpu_fstate_stag, 3'd0,
			                          fpu_fstate_grs, 23'd0};
			5'd16: fsave_busy_word = {fpu_fstate_cmd1, 16'd0};
			5'd17: fsave_busy_word = {fpu_fstate_dtag, 8'd0,
			                          fpu_fstate_wbte15, 20'd0};
			5'd18: fsave_busy_word = {5'd0, fpu_fstate_flags[2],
			                                fpu_fstate_flags[1], 4'd0,
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
		case (n)
			4'd0:  fsave_unimp_word = 32'h4130_0000;
			4'd1:  fsave_unimp_word = {fpu_fstate_cmd3, 16'd0};
			4'd2:  fsave_unimp_word = 32'd0;
			4'd3:  fsave_unimp_word = {fpu_fstate_stag, 3'd0,
			                           fpu_fstate_grs, 23'd0};
			4'd4:  fsave_unimp_word = {fpu_fstate_cmd1, 16'd0};
			4'd5:  fsave_unimp_word = {fpu_fstate_dtag, 8'd0,
			                           fpu_fstate_wbte15, 20'd0};
			4'd6:  fsave_unimp_word = {5'd0, fpu_fstate_flags[2],
			                                  fpu_fstate_flags[1], 4'd0,
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

// IEEE condition predicate over FPSR condition codes {N, Z, I, NAN}
function fp_cond;
	input [5:0] pred;
	input [3:0] cc;
	reg n, z, nan;
	begin
		n = cc[3]; z = cc[2]; nan = cc[0];
		case (pred[3:0])
			4'h0: fp_cond = 0;                          // F / SF
			4'h1: fp_cond = z;                          // EQ
			4'h2: fp_cond = !(nan | z | n);             // OGT
			4'h3: fp_cond = z | !(nan | n);             // OGE
			4'h4: fp_cond = n & !(nan | z);             // OLT
			4'h5: fp_cond = z | (n & !nan);             // OLE
			4'h6: fp_cond = !(nan | z);                 // OGL
			4'h7: fp_cond = !nan;                       // OR
			4'h8: fp_cond = nan;                        // UN
			4'h9: fp_cond = nan | z;                    // UEQ
			4'hA: fp_cond = nan | !(n | z);             // UGT
			4'hB: fp_cond = nan | z | !n;               // UGE
			4'hC: fp_cond = nan | (n & !z);             // ULT
			4'hD: fp_cond = nan | z | n;                // ULE
			4'hE: fp_cond = !z;                         // NE
			default: fp_cond = 1;                       // T / ST
		endcase
	end
endfunction

// Non-branch instructions which the MC68040 defines as changes of flow for
// T0 tracing because they synchronize/refill the instruction pipeline.
// gencpu marks them with trace_t0_68040_only(); note MOVEC is listed for
// the TO-control-register direction only (i_MOVE2C, $4E7B).  Reading a
// control register ($4E7A, i_MOVEC2) changes nothing and does not trace --
// cputest Basic/MOVEC2 expects the trace after the $4E7B in its sequence,
// not after the $4E7A that precedes it.
// Taken branches/returns are handled by go_pc; FDBcc/FMOVEM need extension
// word information and set t0_force in their decode states.
function t0_special;
	input [15:0] op;
	begin
		t0_special =
		    op == 16'h007c || op == 16'h027c || op == 16'h0a7c || // to SR
		    op == 16'h4e71 ||                                      // NOP
		    // STOP is absent: it never reaches the fetch_next boundary --
		    // S_STOP_LD makes its own T0 decision (changed-bits rule).
		    op == 16'h4e7b ||                                      // MOVEC to CR
		    (op & 16'hfff8) == 16'h4e60 ||                         // MOVE An,USP
		    // MOVE USP,An ($4E68-F) does not trace: gencpu marks only
		    // i_MVR2USP with trace_t0_68040_only, the same on-silicon
		    // narrowing hardware already proved for MOVEC ($4E7B only).
		    (op & 16'hffc0) == 16'h46c0 ||                         // MOVE to SR
		    (op[15:12] == 4'h0 && op[11:8] == 4'he &&
		     op[7:6] != 2'b11) ||                                  // MOVES
		    (op[15:12] == 4'h0 && op[11] && !op[8] && op[7:6] == 2'b11 &&
		     op[10:9] != 2'b00) ||                                 // CAS/CAS2
		    // op[8] discriminates CAS ($0AC0/$0CC0/$0EC0, clear) from the
		    // dynamic bit ops BSET Dn,<ea> for D5-D7 ($0Bxx/$0Dxx/$0Fxx,
		    // set): hardware cputest basic/all failed BSET.B D5,(A6)
		    // under T0 with a phantom trace before the bit was added.
		    op[15:8] == 8'hf4 || op[15:8] == 8'hf5 ||              // CINV/CPUSH/PFLUSH/PTEST
		    op[15:8] == 8'hf3;                                     // FSAVE/FRESTORE
	end
endfunction

// Unimplemented FP instruction: vector 11, format $2.  Register/immediate
// forms identify the faulting FP instruction, while a resolved memory form
// carries its operand EA (hardware FINT/FINTRZ corpus behavior).
// Malformed source EA.  A hardware opmode reports the plain format-$0
// F-line; an FPSP-emulated one reports through the unimplemented route.
// Both are written from ONE exc() site so the format and PC selection is
// a 2:1 mux on a single predicate rather than two more writers into the
// exception-format priority tree, which is the core's critical path.
task go_fp_ea_fault;
	input hw;
	begin
		fpu_req <= 0;
		exc(`AP040_VEC_FLINE, hw ? 4'd0 : 4'd2,
		    hw ? pc_i : pc,
		    hw ? 32'd0 : (fp_ea_v ? t_a : pc_i));
	end
endtask

task go_fp_unimp;
	begin
		fpu_req <= 0;
		exc(`AP040_VEC_FLINE, 4'd2, pc, fp_ea_v ? t_a : pc_i);
	end
endtask

// Unsupported data type (denormal/unnormal operands, packed decimal).
// EVERY vector-55 fault stacks format $3: opclass 011 stores carry the
// next PC and the destination address; source faults keep the PC on the
// FP instruction with the source address in EA -- and when the source
// has no address (register or immediate operand) the EA field simply
// reads zero.  Hardware cputest FABS.P #imm and FDMOVE.S Dn both expect
// 00,00,42,05,00,00,30,dc,00,00,00,00; the format-$0 no-EA variant this
// implementation used before came from the UM's "pre-instruction" prose
// and does not match the reference (WinUAE raises all of these through
// its post-instruction path with regs.fp_ea = 0).
task go_fp_unsupp;
	input        post;
	input        packed_early;
	input        has_ea;
	input [31:0] ea;
	begin
		fpu_req <= 0;
		// Datatype faults are POST-instruction exceptions: format $3 with
		// the FOLLOWING instruction's PC, for sources as well as stores.
		// AP040 used to stack the faulting instruction's own PC for source
		// operands (and pc-2 for a statically unsupported packed store).
		// The v20 corpus masked the stacked PC out of every frame
		// comparison, so nothing contradicted it; the v24 corpus checks
		// that field and rejects both, and WinUAE agrees explicitly
		// (fpp.cpp, "simplification: always mid/post-instruction
		// exception" -> newcpu_common.cpp stacks currpc with format $3).
		if (post) exc(`AP040_VEC_FP_UNSUP, 4'd3, pc, ea);
		else      exc(`AP040_VEC_FP_UNSUP, 4'd3, pc, has_ea ? ea : 32'd0);
	end
endtask

// Does the 040 execute this opmode in hardware, or is it one of the
// FPSP-emulated ones (FINT, FSIN, FMOD, ...)?  Mirrors ap040_fpu's
// op_in_hw.  It matters at the malformed-EA sites below: WinUAE's
// get_fp_value runs fault_if_unimplemented_680x0 BEFORE it rejects a Dn
// or An source, so an unimplemented INSTRUCTION with an illegal EA is
// still reported through the FPSP route (vector 11, format $2, PC of the
// following instruction), not as a plain format-$0 F-line.
function fp_op_in_hw;
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
				fp_op_in_hw = 1;
			default:
				fp_op_in_hw = 0;
		endcase
	end
endfunction

// 68040 F-line opmode classification (WinUAE fault_if_nonexisting_opmode):
// 0 = existing (hardware, or the unimplemented-instruction FPSP route with
// its format-$2 frame); 1 = nonexisting, reported IMMEDIATELY as vector 11
// with the plain format-$0 frame and NO FPIAR or FPSR side effects;
// 2 = opmodes $78-$7F, which the 040 reports as vector 4 of all things
// (WinUAE: "Unexpected, isn't it?!").
function [1:0] fp_opmode_class;
	input [6:0] op;
	begin
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
				fp_opmode_class = 2'd1;
			7'h78, 7'h79, 7'h7A, 7'h7B, 7'h7C, 7'h7D, 7'h7E, 7'h7F:
				fp_opmode_class = 2'd2;
			default:
				fp_opmode_class = 2'd0;
		endcase
	end
endfunction

// Malformed FPU command/effective-address combinations are still F-line
// opcodes.  The 68040 reports vector 11 with the normal format-$0 frame;
// this is distinct from the format-$2 FPSP route above.
task go_fp_fline;
	begin
		fpu_req <= 0;
		exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
	end
endtask

// Abandon the queue and stop the fill engine.  Whatever is in flight is
// reaped and discarded, so nothing can re-arm the engine between here and the
// redirect: only issue_ifetch arms it again.
task epf_flush;
	begin
		epf_count <= 0;
		epf_head  <= 0;
		epf_fill  <= 0;
		epf_armed <= 0;
		epf_err   <= 0;
		if (epf_pend) epf_kill <= 1;
		epf_flushed = 1;
	end
endtask

// Point the queue at a, under function code s, and start filling.  A queue
// already tracking that address in that context is kept: the words are
// resident, in flight, or about to be requested, and the caller's S_FETCH
// pops them.  Anything else is a flow, context or FC change and flushes.
task issue_ifetch;
	input [31:0] a;
	input        s;
	begin
		if (epf_armed && epf_next == a && epf_super == s) begin
			// the stream already runs here: nothing to do
		end
		else begin
			epf_count <= 0;
			epf_head  <= 0;
			epf_fill  <= 0;
			epf_err   <= 0;
			epf_next  <= a;
			epf_ftail <= a;
			epf_super <= s;
			epf_armed <= 1;
			epf_flushed = 1;
			if (epf_pend) epf_kill <= 1;
			else if (!mem_req && !mem_ack) begin
				// The port is free: issue the redirect now rather than
				// leaving it to the engine one cycle later.  A longword
				// aligned fetch takes both words in one request.  Alignment
				// is what makes it safe: an aligned longword cannot span a
				// page, so it cannot translate or fault differently than the
				// two halves would have (the exception prefetch below stays
				// word-wise precisely because its $FFE entry CAN span).
				mem_req <= 1; mem_write <= 0; mem_instr <= 1;
				mem_size <= a[1] ? `AP040_SZ_W : `AP040_SZ_L;
				mem_addr <= a;
				fc_r <= s ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
				epf_pend <= 1;
				epf_pend_lw <= ~a[1];
				epf_kill <= 0;
				epf_issue = 1;
			end
		end
	end
endtask

// Start the architecturally required four-longword prefetch which concludes
// reset and exception processing.  Word requests are intentional: an entry
// at page offset $FFE must translate/fault the second page independently.
task exception_prefetch;
	input [31:0] a;
	input        s;
	begin
		epf_flush;
		epf_base <= a;
		epf_next <= a;
		epf_ftail <= a;
		epf_super <= s;
		pc <= a;
		pc_i <= a;
		mem_req <= 1; mem_write <= 0; mem_instr <= 1;
		mem_size <= `AP040_SZ_W; mem_addr <= a;
		fc_r <= s ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
		epf_issue = 1;
		state <= S_EPF_FILL;
	end
endtask

// Enter the processor-halted state after an unrecoverable double fault.
// A bus error terminates the external cycle without returning mem_ack, so a
// held mem_req would otherwise be re-issued by the bus adapter on the next
// idle clock.  Quiesce every request source at the point of entry as well as
// in S_HALT itself.
task fatal_halt;
	begin
		mem_req <= 0;
		m_issued <= 0;
		pt_req <= 0;
		pf_req <= 0;
		cinv_req <= 0;
		fpu_req <= 0;
		epf_flush;
		epf_pend <= 0;
		epf_kill <= 0;
		fc_ovr_v <= 0;
		lk_cyc <= 0;
		fault_r <= 1;
		state <= S_HALT;
	end
endtask

task rfw;
	input [3:0] a;
	input [31:0] d;
	begin
		rf_we <= 1; rf_waddr <= a; rf_wdata <= d;
	end
endtask

task immf;
	input [1:0] n;
	input [7:0] ret;
	begin
		imm_n <= n; imm <= 0;
		r_imm_ret <= ret; state <= S_IMMF;
	end
endtask

// A data transfer that crosses a page boundary would be translated once
// and then incremented physically by the bus adapter, so the second page
// would be accessed through the first page's mapping.  Such transfers are
// issued one byte at a time instead: every byte is translated on its own,
// faults report the byte that failed, and the history bits of both pages
// are updated.  Only misaligned transfers can cross a boundary.
wire  [2:0] m_nbytes = (m_size == `AP040_SZ_B) ? 3'd1 :
                       (m_size == `AP040_SZ_W) ? 3'd2 : 3'd4;
wire [31:0] m_pgmask = tc[14] ? 32'h0000_1FFF : 32'h0000_0FFF;
wire        m_cross  = tc[15] &&
                       (((m_addr_r & m_pgmask) + {29'd0, m_nbytes}) >
                        (m_pgmask + 32'd1));

task mrd;
	input [31:0] a;
	input [1:0] size;
	input [7:0] ret;
	begin
		m_addr_r <= a; m_size <= size; m_wr <= 0; m_issued <= 0;
		r_m_ret <= ret; state <= S_MRD;
	end
endtask

task mwr;
	input [31:0] a;
	input [1:0] size;
	input [31:0] d;
	input [7:0] ret;
	begin
		m_addr_r <= a; m_size <= size; m_wdat <= d; m_wr <= 1; m_issued <= 0;
		r_m_ret <= ret; state <= S_MWR;
	end
endtask

task ea_start;
	input [2:0] mode;
	input [2:0] rn;
	input [1:0] size;
	input [7:0] ret;
	begin
		ea_mode <= mode; ea_rn <= rn; ea_size <= size;
		ea_pcmode <= 0; ea_pcb <= pc;
		r_ea_ret <= ret; state <= S_EA_DISP;
	end
endtask

task exc;
	input [7:0] vec;
	input [3:0] fmt;
	input [31:0] spc;
	input [31:0] addr;
	begin
		exc_vec <= vec; exc_fmt <= fmt; exc_spc <= spc; exc_addr <= addr;
		exc_is_irq <= 0; exc_pass2 <= 0;
		// A T0 trace does NOT survive an exception on the 68040.  This
		// used to arm one for illegal/privilege/A-line/F-line, reading
		// WinUAE's Exception_cpu_oldpc as if every exception ran through
		// it.  Only the INTERNAL exceptions do -- gencpu emits
		// exception_cpu() solely for divide-by-zero, CHK, TRAPV, TRAP #n
		// and the RTE format error -- and on a 68040 that path then forces
		// t0 = false for exactly those vectors, while everything else
		// (op_illg's vector 4 included) goes through plain Exception(),
		// whose exception_check_trace clears T0 outright.  So no exception
		// leaves a T0 trace pending.  Hardware agrees: cputest basic/all
		// and fbasic/all both reported "Got unexpected trace exception"
		// after the ILLEGAL that terminates every test, on plain integer
		// instructions as well as FP ones.
		//
		// The texc machinery itself stays: it still delivers a trace that
		// a simultaneous INTERRUPT preempted (see fetch_next and go_pc),
		// which is a different and real case.
		// Exception stack and vector accesses are always supervisor-data
		// references.  In particular, do not let a faulting MOVES retain its
		// SFC/DFC override into exception processing.
		fc_ovr_v <= 0;
		flow_t0_pend <= 0;
		// Exception processing owns the memory port from here: the fetch
		// queue is abandoned and cannot re-arm until the handler's
		// prefetch runs.  A queue fetch already on the bus is left to
		// finish -- dropping a request the cache has accepted would lose
		// its acknowledge -- and its data is discarded by epf_kill.
		epf_flush;
		if (!epf_pend) mem_req <= 0;
		// A faulted/aborted locked sequence ends here: the 040 drops LOCK
		// on the fault, and a stale lk_cyc would throttle the handler's
		// fetch queue (fetch_next is not on the exception entry path).
		lk_cyc <= 0;
		state <= S_EXC0;
	end
endtask

// access error entry: capture the fault shape from the outstanding request
task aerr_start;
	begin
		aer_bus  <= berr && !mem_flt;   // physical bus error, not an ATC fault
		// FA is the initial byte of the original transfer, even when a
		// page-crossing access has been split and a later byte faults.
		aer_fa   <= mem_instr ? mem_addr : m_addr_r;
		aer_wr   <= mem_write;
		aer_sz   <= (!mem_instr && m_cross) ? m_size : mem_size;
		aer_wd   <= (!mem_instr && m_cross) ? m_wdat : mem_wdata;
		aer_lk   <= lk_cyc;
		// memory ops wait in S_MRD/S_MWR: the requesting context is
		// identified by the continuation state, not by `state` itself
		aer_m16  <= !mem_instr &&
		            (r_m_ret >= S_M16_RD2 && r_m_ret <= S_M16_INC2);
		// MOVES faults report the alternate space in TT/TM: FC 0, 3, 4 and 7
		// keep the raw FC with TT = 10; FC 2 and 6 are remapped onto the
		// corresponding data space (WinUAE mmu_bus_error's ismoves block)
		aer_tt   <= (fc_ovr_v && (fc_r[1:0] == 2'b00 || fc_r[1:0] == 2'b11))
		            ? 2'b10 : 2'b00;
		aer_tm   <= (fc_ovr_v && (fc_r[1:0] == 2'b00 || fc_r[1:0] == 2'b11)) ? fc_r :
		            (fc_ovr_v && fc_r[1]) ? {fc_r[2], 2'b01} : fc_r;
		aer_ma   <= mem_flt && !mem_instr && m_cross &&
		            ((mem_addr & ~m_pgmask) != (m_addr_r & ~m_pgmask));
		// Preserve fc_r above for the SSW, then force all frame/vector cycles
		// back to supervisor data space.
		fc_ovr_v <= 0;
		epf_flush;
		mem_req  <= 0;
		// The locked sequence ends at the fault (aer_lk above still
		// captures the pre-edge value): the 040 drops LOCK, and a stale
		// lk_cyc would throttle the handler's fetch queue.
		lk_cyc   <= 0;
		state    <= S_AERR0;
	end
endtask

// record an address register update for rollback on an access error
task u_rec;
	input [3:0] r;
	input [31:0] old;
	begin
		if (!u0_v) begin
			u0_v <= 1; u0_reg <= r; u0_old <= old;
		end
		else begin
			u1_v <= 1; u1_reg <= r; u1_old <= old;
		end
	end
endtask

// Access-error SSW.  CP/CU/CT/CM stay clear: pure instruction restart.
// MOVE16 line faults report SIZE = line with TT0; locked TAS/CAS cycles
// report LK with RW clear (WinUAE mmu_bus_error).
wire [15:0] aer_ssw = {4'b0000, aer_ma, ~aer_bus, aer_lk,
                       (~aer_wr & ~aer_lk), 1'b0,
                       aer_m16 ? 2'b11 :
                       (aer_sz == `AP040_SZ_B) ? 2'b01 :
                       (aer_sz == `AP040_SZ_W) ? 2'b10 : 2'b00,
                       aer_m16 ? 2'b01 : aer_tt,
                       aer_tm};

// format $7 frame contents, one word per index (30 words)
function [15:0] aerr_word;
	input [4:0] idx;
	begin
		case (idx)
			5'd0:  aerr_word = sr_saved;
			5'd1:  aerr_word = pc_i[31:16];
			5'd2:  aerr_word = pc_i[15:0];
			5'd3:  aerr_word = 16'h7008;               // format $7, vector 2
			// WinUAE stacks the fault address in EA as well (aligned to the
			// line for MOVE16); handlers that honour CM/CT never see those
			// bits set here, so the field is informational
			5'd4:  aerr_word = aer_fa[31:16];
			5'd5:  aerr_word = aer_m16 ? {aer_fa[15:4], 4'd0} : aer_fa[15:0];
			5'd6:  aerr_word = aer_ssw;
			// WB3S stays CLEAR: this core RESTARTS the faulting
			// instruction after the handler repairs the mapping, so it
			// must not also advertise a pending writeback.  An OS that
			// honours the 040 frame (NetBSD trap.c: "the 68040 doesn't
			// re-run instructions that cause write page faults ... we
			// have to write the value out to memory ourselves") performs
			// every VALID WB3 -- combined with the restart, an RMW store
			// like ld.elf_so's relocation add.l lands TWICE.  Captured
			// live: init's ctor pointer held link VA + 2x load base and
			// NetBSD hung looping on the resulting wild ifetch.  WB3D/A
			// keep the write data for diagnostics; valid stays 0.
			5'd7:  aerr_word = 16'd0;
			5'd10: aerr_word = aer_fa[31:16];          // initial fault address
			5'd11: aerr_word = aer_fa[15:0];
			5'd12: aerr_word = aer_fa[31:16];          // WB3A
			5'd13: aerr_word = aer_fa[15:0];
			5'd14: aerr_word = aer_wd[31:16];          // WB3D
			5'd15: aerr_word = aer_wd[15:0];
			default: aerr_word = 16'd0;                // writeback/push slots
		endcase
	end
endfunction

task fetch_next;
	begin
		fc_ovr_v <= 0;
		lk_cyc <= 0;
		u0_v <= 0;
		u1_v <= 0;
		// An interrupt already sampled at the completing instruction's
		// boundary wins over a simultaneous T1/T0 trace on the 68040
		// (WinUAE do_specialties: the trace converts to a PENDING trace
		// before interrupts are sampled).  This ordering is visible when an
		// odd IRQ vector produces a nested address error: its SR must
		// contain the accepted interrupt mask, not the trace exception's
		// pre-IRQ context.  The trace event itself is NOT lost: it is
		// delivered at the interrupt handler's entry through the S_EXC_JMP
		// texc machinery, like the T0 survival.
		if (irq_pend) begin
			if (tr_t1 || (tr_t0 && t0_force)) begin
				texc_pend <= 1;
				texc_pc <= pc_i;
				tr_t1 <= 0;
			end
			exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
			exc_fmt <= 0; exc_spc <= pc; exc_addr <= 0;
			exc_is_irq <= 1; exc_pass2 <= 0;
			irq_lvl_l <= irq_take_lvl;
			// As with trace, an interrupt recognized at the instruction
			// boundary must see the just-completed register writeback.
			epf_flush;
			state <= S_POST_EXC;
		end
		else if (tr_t1 || (tr_t0 && t0_force)) begin
			tr_t1 <= 0;
			exc(`AP040_VEC_TRACE, 4'd2, pc, pc_i);
			// Instruction writeback is registered separately.  Do not let
			// S_EXC0 sample Dn/An/A7 on the same edge that commits it.
			state <= S_POST_EXC;
		end
		else begin
			issue_ifetch(pc, sr_s);
			pc_i <= pc;
			state <= S_FETCH;
		end
	end
endtask

// True while an effective address is being computed, i.e. while a data
// access is known to be coming.  Keeps speculative instruction fetches
// off the shared memory port just ahead of it.
wire ea_state = (state == S_EA_DISP)  || (state == S_EA_BASE) ||
                (state == S_EA_D16)   || (state == S_EA_EXTW) ||
                (state == S_EA_EXTW2) || (state == S_EA_BD)   ||
                (state == S_EA_MIND)  || (state == S_EA_OD)   ||
                (state == S_EA_ABS)   || (state == S_PIPE_SRD) ||
                (state == S_PIPE_DEA);

wire [31:0] exc_fsize = (exc_fmt == 4'd2 || exc_fmt == 4'd3) ? 32'd12 :
                        (exc_fmt == 4'd4) ? 32'd16 : 32'd8;

task go_illegal;
	begin
		exc(`AP040_VEC_ILLEGAL, 4'd0, pc_i, 32'd0);
	end
endtask

task go_priv;
	begin
		exc(`AP040_VEC_PRIV, 4'd0, pc_i, 32'd0);
	end
endtask

// jump to a control flow target with odd address check
task go_pc;
	input [31:0] t;
	begin
		// The format-$2 address field contains the referenced address with A0
		// cleared, not the raw odd target.
		if (t[0]) exc(`AP040_VEC_ADDRERR, 4'd2, pc_i, {t[31:1], 1'b0});
		else if (tr_t1) begin
			tr_t1 <= 0;
			tr_t0 <= 0;
			pc <= t;
			if (irq_pend) begin
				// WinUAE do_specialties: the completing instruction's
				// trace converts to a PENDING trace and the interrupt is
				// sampled after that conversion, so the interrupt
				// exception processes first and the trace is delivered at
				// its handler entry (the S_EXC_JMP texc machinery, same
				// as the T0 survival), with the traced instruction in the
				// format-$2 address field.  The trace event is never
				// lost and never precedes the interrupt.
				texc_pend <= 1;
				texc_pc <= pc_i;
				exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
				exc_fmt <= 0; exc_spc <= t; exc_addr <= 0;
				exc_is_irq <= 1; exc_pass2 <= 0;
				irq_lvl_l <= irq_take_lvl;
				epf_flush;
				state <= S_POST_EXC;
			end
			else exc(`AP040_VEC_TRACE, 4'd2, t, pc_i);
		end
		else if (tr_t0) begin
			// The 040 resolves a T0 change-of-flow trace only after the
			// target word has entered the pipeline.  An immediately decoded
			// ILLEGAL at the target wins and cancels this trace; a normal
			// target is not executed before vector 9 is taken.
			tr_t0 <= 0;
			flow_t0_pend <= 1;
			flow_t0_oldpc <= pc_i;
			pc <= t;
			issue_ifetch(t, sr_s);
			pc_i <= t;
			state <= S_FETCH;
		end
		else begin
			pc <= t;
			fc_ovr_v <= 0;
			if (irq_pend) begin
				exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
				exc_fmt <= 0; exc_spc <= t; exc_addr <= 0;
				exc_is_irq <= 1; exc_pass2 <= 0;
				irq_lvl_l <= irq_take_lvl;
				// BSR/JSR and taken DBcc can commit A7/Dn on the
				// same edge that redirects here.  Let that registered
				// writeback become visible before S_EXC0 snapshots it.
				epf_flush;
				state <= S_POST_EXC;
			end
			else begin
				issue_ifetch(t, sr_s);
				pc_i <= t;
				state <= S_FETCH;
			end
		end
	end
endtask

// Bcc calculates and validates its target even when the condition is false.
// The 68040 therefore takes an address error for an odd target on a
// not-taken conditional branch as well as on a taken one.
task finish_bcc;
	input [31:0] t;
	input        taken;
	begin
		if (t[0]) exc(`AP040_VEC_ADDRERR, 4'd2, pc_i, {t[31:1], 1'b0});
		else if (taken) go_pc(t);
		else fetch_next;
	end
endtask

task pipe_go;
	begin
		state <= S_PIPE_START;
	end
endtask

//---------------------------------------------------------------------------
// main state machine
//---------------------------------------------------------------------------

integer li;

always @(posedge clk) begin
	// Combinational carriers, valid only inside this block: the fetch queue's
	// fill engine runs after the case statement and has to see what the case
	// did to the memory port and to the queue in the same cycle.
	epf_issue   = 0;
	epf_flushed = 0;
	epf_pop     = 2'd0;
	epf_fillw   = 2'd0;

	if (!nreset) begin
		state <= S_START;
		pc <= 0; pc_i <= 0;
		sr <= `AP040_SR_RESET;
		vbr <= 0; cacr <= 0;
		sfc <= 0; dfc <= 0;
		if (!mmu_reset_seen) begin
			tc <= 0;
			itt0 <= 0; itt1 <= 0; dtt0 <= 0; dtt1 <= 0;
			mmusr <= 0; urp <= 0; srp <= 0;
		end
		else begin
			// RSTI clears only the translation-enable bits.  TC.P and all
			// other MMU register fields retain their previous values.
			tc   <= tc   & 32'h0000_4000;
			itt0 <= itt0 & 32'hFFFF_7FFF;
			itt1 <= itt1 & 32'hFFFF_7FFF;
			dtt0 <= dtt0 & 32'hFFFF_7FFF;
			dtt1 <= dtt1 & 32'hFFFF_7FFF;
		end
		mmu_reset_seen <= 1;
		ir <= 0;
		mem_req <= 0; mem_write <= 0; mem_instr <= 0;
		mem_size <= `AP040_SZ_W; mem_addr <= 0; mem_wdata <= 0;
		fc_r <= `AP040_FC_SUPER_DATA;
		rf_we <= 0; rf_waddr <= 0; rf_wdata <= 0;
		fpu_req <= 0; fpu_class <= 0; fpu_opm <= 0; fpu_fmt <= 0;
		fpu_srcr <= 0; fpu_dstr <= 0; fpb <= 0;
		fpu_bg <= 0; fpu_pend_exc <= 0; fpu_pend_vec <= 0;
		fpu_pendcap <= 0; fp_restore_grs <= 0; fp_restore_wbte15 <= 0;
		fp_restore_wbt <= 0; fp_restore_fpiar <= 0; fp_restore_busy <= 0;
		fpb_n <= 0;
		fpu_crsel <= 0; fpu_crwe <= 0; fpu_crwd <= 0; fpu_iawe <= 0;
		fpu_bsun <= 0;
		fpu_fmsel <= 0; fpu_fmwe <= 0; fpu_fmwd <= 0; fpu_rst <= 0;
		fpu_fsave_ack <= 0; fpu_frestore_idle <= 0; fpu_frestore_unimp <= 0;
		fp_restore_cmd1 <= 0; fp_restore_cmd3 <= 0;
		fp_restore_stag <= 0; fp_restore_dtag <= 0; fp_restore_flags <= 0;
		fp_restore_fpt <= 0; fp_restore_et <= 0;
		fp_cnt <= 0; fp_nb <= 0; fp_st <= 0; fp_list <= 0; fp_mode <= 0;
		fp_st_epend <= 0; fp_st_evec <= 0;
		fp_rev <= 0; fp_lsb <= 0;
		fp_creg <= 0; fp_pred <= 0; fp_n <= 0;
		fp_ea_pd <= 0; fp_ea_pi <= 0; fp_adj <= 0; fp_ea_v <= 0;
		fp_force_unsupp <= 0;
		m_bidx <= 0; m_acc <= 0;
		rr_a <= 0; rr_b <= 0;
		aux_we <= 0; aux_sel <= 0; aux_wdata <= 0;
		md_start <= 0; md_isdiv <= 0; md_sign <= 0;
		md_a <= 0; md_hi <= 0; md_lo <= 0;
		alu_op <= 0; op_size <= 0;
		src_val <= 0; dst_val <= 0; dst_addr <= 0;
		sh_val <= 0; sh_fl <= 0; sh_cnt <= 0; sh_vacc <= 0; sh_rox <= 0;
		sh_any <= 0;
		r_imm_ret <= 0; r_ea_ret <= 0; r_m_ret <= 0;
		imm_n <= 0; m_issued <= 0; imm <= 0; x_ext <= 0;
		epf_count <= 0; epf_head <= 0; epf_fill <= 0;
		epf_base <= 0; epf_next <= 0; epf_super <= 0;
		epf_ftail <= 0; epf_armed <= 0; epf_pend <= 0;
		epf_pend_lw <= 0; epf_kill <= 0; epf_err <= 0;
		for (li = 0; li < 8; li = li + 1) epf_data[li] <= 0;
		m_wr <= 0; m_size <= 0; m_addr_r <= 0; m_wdat <= 0; m_val <= 0;
		ea_mode <= 0; ea_rn <= 0; ea_size <= 0;
		ea_pcmode <= 0; ea_pcb <= 0; extw <= 0;
		ea_base_v <= 0; ea_idx_v <= 0; ea_mind <= 0;
		ea_post <= 0; ea_odl <= 0; ea_absl <= 0; ea_addr <= 0;
		p_src <= 0; p_dst <= 0; p_sreg <= 0; p_dreg <= 0;
		p_ssize <= 0; p_dsize <= 0;
		p_rmw <= 0; p_wbsup <= 0; p_flags <= 0; p_sextw <= 0;
		p_dst_mem_bit <= 0;
		exec_kind <= EK_ALU;
		src_mode_r <= 0; src_rn_r <= 0; dst_mode_r <= 0; dst_rn_r <= 0;
		exc_vec <= 0; exc_fmt <= 0; exc_spc <= 0; exc_addr <= 0; exc_sp <= 0;
		exc_is_irq <= 0; exc_pass2 <= 0; sr_saved <= 0; irq_lvl_l <= 0;
		texc_pend <= 0; texc_pc <= 0;
		flow_t0_pend <= 0; flow_t0_oldpc <= 0;
		rte_sr <= 0; rte_pc <= 0; ret_kind <= 0;
		br_base <= 0; br_tgt <= 0; br_long <= 0;
		mm_mask <= 0; mm_dir <= 0; mm_predec <= 0; mm_postinc <= 0;
		mm_size <= 0; mm_addr <= 0; mm_init_an <= 0; mm_reg <= 0;
		mm_base_ea <= 0; mm_base_pend <= 0; mm_base_val <= 0;
		mp_cnt <= 0; mp_idx <= 0; mp_dir <= 0; mp_addr <= 0; mp_val <= 0;
		t_a <= 0; t_b <= 0; srop_kind <= 0; srop_sr <= 0;
		mvc_dir <= 0; fc_ovr_v <= 0; fc_ovr <= 0;
		lk_cyc <= 0; aer_lk <= 0; aer_m16 <= 0; aer_tt <= 0; aer_wd <= 0;
		m16_form <= 0; m16_dst_rn <= 0; m16_src <= 0; m16_dst <= 0;
		m16_an <= 0; m16_idx <= 0; m16_rd_done <= 0;
		for (li = 0; li < 4; li = li + 1) m16buf[li] <= 0;
		rst_cnt <= 0;
		fault_r <= 0;
		nmi_ack_t <= 0;
		irq_ack_t <= 0;
		bf_off <= 0; bf_w <= 0; bf_addr <= 0; bf_bib <= 0; bf_span <= 0;
		bf_w1 <= 0; bf_w2 <= 0; bf_du <= 0; cas_dc <= 0;
		bf_t40 <= 0; bf_field <= 0; bf_ones <= 0; bf_maskl <= 0;
		in_exc <= 0;
		aer_fa <= 0; aer_sp <= 0; aer_wr <= 0;
		aer_sz <= 0; aer_tm <= 0; aer_idx <= 0; aer_bus <= 0; aer_ma <= 0;
		u0_v <= 0; u1_v <= 0;
		u0_reg <= 0; u1_reg <= 0; u0_old <= 0; u1_old <= 0;
		pt_req <= 0; pt_write <= 0; pt_addr <= 0;
		pf_req <= 0; pf_mode <= 0; pf_addr <= 0;
		cinv_req <= 0; cinv_ic <= 0; cinv_dc <= 0;
		tr_t1 <= 0; tr_t0 <= 0; t0_force <= 0;
	end
	else if (ce) begin
		rf_we <= 0;
		aux_we <= 0;
		md_start <= 0;
		fpu_req <= 0; fpu_crwe <= 0; fpu_fmwe <= 0; fpu_iawe <= 0;
		fpu_bsun <= 0;
		// released-operation completion: results retire inside the FPU;
		// an enabled arithmetic exception becomes a pending pre-instruction
		// exception for the next FPU dispatch point
		fpu_pendcap <= 0;   // default BEFORE the retire branch's set
		if (fpu_bg && fpu_done) fpu_bg <= 0;
		if (fpu_bg && fpu_exc_req) begin
			fpu_bg <= 0;
			fpu_pend_exc <= 1;
			fpu_pend_vec <= fpu_exc_vec;
			// let the FPU prepare the FSAVE e1 frame from its shadow
			fpu_pendcap <= 1;
		end
		fpu_rst <= 0;
		fpu_fsave_ack <= 0;
		fpu_frestore_idle <= 0;
		fpu_frestore_unimp <= 0;
		if (mem_ack) mem_req <= 0;

		// A completing CPU write that lands inside the fetch queue's
		// window [epf_next, epf_ftail) flushes it, so the rewritten
		// words are refetched.  Stronger than the architected CPUSH/CINV
		// requirement, and deliberate: the previous fetch-buffered core
		// booted DiagROM but not AmigaOS on hardware with exactly this
		// stale-prefetch hazard, invisible to CINV-disciplined tests.
		// Data ops never overlap an outstanding queue fetch (they hold
		// issue on epf_pend), so the flush here is pure bookkeeping.
		// Both ranges are logical addresses; ftail never wraps past the
		// page guard, so the plain compares suffice.
		if (d_ack && mem_write && epf_armed && epf_count != 4'd0 &&
		    (mem_addr + 32'd3 >= epf_next) && (mem_addr < epf_ftail))
			epf_flush;

		case (state)
			//------------------------------------------------------------ boot
			S_START: begin
				// Reset vector acquisition and the first target prefetch are part
				// of reset exception processing.  Any access fault before the
				// first opcode arrives is therefore a double bus fault.
				in_exc <= 1;
				m_addr_r <= `AP040_VEC_RESET_ISP; m_size <= `AP040_SZ_L;
				m_wr <= 0; m_issued <= 0; r_m_ret <= S_BOOT0;
				state <= S_MRD;
			end

			S_BOOT0: begin
				rfw(4'd15, m_val);
				mrd(`AP040_VEC_RESET_PC, `AP040_SZ_L, S_BOOT1);
			end

			S_BOOT1: begin
				if (m_val[0]) fatal_halt;
				else exception_prefetch(m_val, sr_s);
			end

			// Reset/exception processing concludes by fetching four longwords.
			// Any fault in this window is itself a double bus fault.
			S_EPF_FILL: begin
				if (i_err) fatal_halt;
				else if (i_ack) begin
					epf_data[epf_fill] <= mem_rdata[15:0];
					if (epf_fill == 3'd7) begin
						epf_count <= 4'd8;
						epf_head <= 0;
						// eight words wrap the ring: the append index
						// belongs back at the head, or the engine's first
						// fill would land on a resident word
						epf_fill <= 0;
						epf_next <= epf_base;
						// the handler's stream continues past the four
						// architectural longwords under the fill engine
						epf_ftail <= epf_base + 32'd16;
						epf_armed <= 1;
						state <= S_EPF_READY;
					end
					else begin
						epf_fill <= epf_fill + 3'd1;
						state <= S_EPF_GAP;
					end
				end
			end

			// Give the MMU/cache request handshake a full low cycle between
			// words.  Changing an address while req remains asserted can make a
			// completed request look like a duplicate transaction.
			S_EPF_GAP: begin
				mem_req <= 1; mem_write <= 0; mem_instr <= 1;
				mem_size <= `AP040_SZ_W;
				mem_addr <= epf_base + {28'd0, epf_fill, 1'b0};
				fc_r <= epf_super ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
				state <= S_EPF_FILL;
			end

			S_EPF_READY: begin
				issue_ifetch(epf_base, epf_super);
				pc_i <= epf_base;
				state <= S_FETCH;
			end

			//----------------------------------------------------------- fetch
			// A pure consumer of the fetch queue: the opcode word is either
			// resident, arriving on the bus this very cycle, or the queue is
			// not tracking this address and has to be re-armed here.
			S_FETCH: if (epf_ready_pc || epf_fwd_pc) begin : fetch_word
				reg [15:0] fw;
				fw = epf_fwd_pc ? epf_fwd_word : epf_data[epf_head];
				// Exception processing includes the first handler refill.  An
				// interrupt which becomes pending after vector fetch but before
				// this opcode arrives must still run before the handler executes.
				// Discard the fetched word and stack a return to the handler entry.
				if (in_exc && irq_pend) begin
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
					exc_fmt <= 0; exc_spc <= pc; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= irq_take_lvl;
					epf_flush;
					state <= S_EXC0;
				end
				else if (flow_t0_pend) begin
					// The completed change-of-flow instruction's T0 trace fires
					// at the redirect target, ahead of the target instruction.
					// The frame PC is the target; its address field identifies
					// the branch/return.
					//
					// This used to be suppressed when the target was ILLEGAL
					// ($4AFC), to avoid double-reporting under the old model
					// where a T0 trace ALSO survived the illegal-instruction
					// exception.  That survivor path is gone (no exception
					// leaves a T0 trace pending on the 040), so the
					// suppression only lost the trace: cputest branches
					// straight into its terminating ILLEGAL and expects
					// vector 9 with the branch target stacked.
					flow_t0_pend <= 0;
					exc(`AP040_VEC_TRACE, 4'd2, pc, flow_t0_oldpc);
				end
				else begin
					// All four exception-prefetch longwords are now resident; the
					// first buffered handler instruction begins normal execution.
					in_exc <= 0;
					epf_pop = 2'd1;
					ir <= fw;
					pc <= pc + 32'd2;
					// per-instruction defaults
					tr_t1 <= sr[15];
					tr_t0 <= sr[14];
					flow_t0_pend <= 0;
					t0_force <= t0_special(fw);
					p_src <= SK_NONE; p_dst <= DK_NONE;
					p_rmw <= 0; p_wbsup <= 0; p_flags <= 1; p_sextw <= 0;
					p_dst_mem_bit <= 0;
					exec_kind <= EK_ALU;
					fc_ovr_v <= 0;
					state <= S_DECODE;
				end
			end
			// The queue does not run this stream -- a redirect that could not
			// claim the port, or an SR write that changed the FC one cycle
			// after the fetch was armed -- so re-arm it on the live context.
			else if (!epf_armed || epf_next != pc || epf_super != sr_s)
				issue_ifetch(pc, sr_s);

			//---------------------------- page-crossing split transfers
			// One byte per bus transaction, most significant first, so each
			// byte is translated through its own page.
			S_MRD_B: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				if (!m_issued && epf_pend) begin
				end
				else if (!m_issued) begin
					mem_req <= 1; mem_write <= 0; mem_instr <= 0;
					mem_size <= `AP040_SZ_B;
					mem_addr <= m_addr_r + {29'd0, m_bidx};
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start;
				end
				else if (d_ack) begin : mrd_b
					reg [31:0] acc;
					acc = {m_acc[23:0], mem_rdata[7:0]};
					m_acc <= acc;
					m_issued <= 0;
					if (m_bidx + 3'd1 == m_nbytes) begin
						m_val <= acc;
						state <= r_m_ret;
					end
					else m_bidx <= m_bidx + 3'd1;
				end
			end

			S_MWR_B: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				if (!m_issued && epf_pend) begin
				end
				else if (!m_issued) begin : mwr_b
					reg [7:0] byv;
					case (m_size)
						`AP040_SZ_W: byv = m_bidx[0] ? m_wdat[7:0] : m_wdat[15:8];
						`AP040_SZ_L: case (m_bidx[1:0])
							2'd0: byv = m_wdat[31:24];
							2'd1: byv = m_wdat[23:16];
							2'd2: byv = m_wdat[15:8];
							default: byv = m_wdat[7:0];
						endcase
						default: byv = m_wdat[7:0];
					endcase
					mem_req <= 1; mem_write <= 1; mem_instr <= 0;
					mem_size <= `AP040_SZ_B;
					mem_addr <= m_addr_r + {29'd0, m_bidx};
					mem_wdata <= {24'd0, byv};
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start;
				end
				else if (d_ack) begin
					m_issued <= 0;
					if (m_bidx + 3'd1 == m_nbytes) state <= r_m_ret;
					else m_bidx <= m_bidx + 3'd1;
				end
			end

			//------------------------------------------------- generic helpers
			S_NEXT: fetch_next;

			// Post-instruction trace/IRQ commit barrier.  rf_we/aux_we from
			// the completing instruction have reached the register file by
			// the time S_EXC0 runs on the following qualified edge.
			S_POST_EXC: state <= S_EXC0;

			// Extension-word fetch, the queue's second consumer.  A longword
			// immediate whose two words are both available is taken in one
			// pass, whether they come from the queue or off the bus.
			S_IMMF: begin
				if (imm_n == 2'd2 && epf_fwd_pc && epf_pend_lw) begin
					imm      <= mem_rdata;
					pc       <= pc + 32'd4;
					epf_pop   = 2'd2;
					state    <= r_imm_ret;
				end
				else if (imm_n == 2'd2 && epf_ready_pc2) begin
					imm      <= {epf_data[epf_head], epf_data[epf_head + 3'd1]};
					pc       <= pc + 32'd4;
					epf_pop   = 2'd2;
					state    <= r_imm_ret;
				end
				else if (epf_ready_pc || epf_fwd_pc) begin
					imm <= {imm[15:0],
					        epf_fwd_pc ? epf_fwd_word : epf_data[epf_head]};
					pc <= pc + 32'd2;
					epf_pop = 2'd1;
					if (imm_n == 2'd1) state <= r_imm_ret;
					else imm_n <= imm_n - 2'd1;
				end
				else if (!epf_armed || epf_next != pc || epf_super != sr_s)
					issue_ifetch(pc, sr_s);
			end

			S_MRD: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				if (!m_issued && epf_pend) begin
				end
				else if (!m_issued && m_cross) begin
					m_bidx <= 0;
					m_acc <= 0;
					state <= S_MRD_B;
				end
				else if (!m_issued) begin
					mem_req <= 1; mem_write <= 0; mem_instr <= 0;
					mem_size <= m_size; mem_addr <= m_addr_r;
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start;
				end
				else if (d_ack) begin
					m_val <= mem_rdata;
					state <= r_m_ret;
				end
			end

			S_MWR: begin
				// A queue fetch owns the memory port: hold this transfer
				// until it completes.  Only the issue is delayed -- the
				// acknowledge branches below stay unreachable meanwhile,
				// so the fetch's ack is never mistaken for this one's.
				if (!m_issued && epf_pend) begin
				end
				else if (!m_issued && m_cross) begin
					m_bidx <= 0;
					state <= S_MWR_B;
				end
				else if (!m_issued) begin
					mem_req <= 1; mem_write <= 1; mem_instr <= 0;
					mem_size <= m_size; mem_addr <= m_addr_r;
					mem_wdata <= m_wdat;
					fc_r <= fc_ovr_v ? fc_ovr :
					        (sr_s ? `AP040_FC_SUPER_DATA : `AP040_FC_USER_DATA);
					m_issued <= 1;
				end
				else if (d_err) begin
					if (in_exc) fatal_halt;
					else aerr_start;
				end
				else if (d_ack) begin
					state <= r_m_ret;
				end
			end

			//------------------------------------------------------- EA engine
			S_EA_DISP: begin
				case (ea_mode)
					3'b010, 3'b011, 3'b100: begin
						rr_a <= {1'b1, ea_rn};
						state <= S_EA_BASE;
					end
					3'b101: begin
						rr_a <= {1'b1, ea_rn};
						immf(2'd1, S_EA_D16);
					end
					3'b110: begin
						rr_a <= {1'b1, ea_rn};
						immf(2'd1, S_EA_EXTW);
					end
					default: begin // 111
						case (ea_rn)
							3'b000: begin ea_absl <= 0; immf(2'd1, S_EA_ABS); end
							3'b001: begin ea_absl <= 1; immf(2'd2, S_EA_ABS); end
							3'b010: begin ea_pcmode <= 1; immf(2'd1, S_EA_D16); end
							3'b011: begin ea_pcmode <= 1; immf(2'd1, S_EA_EXTW); end
							default: go_illegal;
						endcase
					end
				endcase
			end

			S_EA_BASE: begin
				case (ea_mode)
					3'b010: ea_addr <= rf_rdata_a;
					3'b011: begin
						ea_addr <= rf_rdata_a;
						rfw({1'b1, ea_rn}, rf_rdata_a + an_adj(ea_rn, ea_size));
						u_rec({1'b1, ea_rn}, rf_rdata_a);
					end
					default: begin // 100
						ea_addr <= rf_rdata_a - an_adj(ea_rn, ea_size);
						rfw({1'b1, ea_rn}, rf_rdata_a - an_adj(ea_rn, ea_size));
						u_rec({1'b1, ea_rn}, rf_rdata_a);
					end
				endcase
				state <= r_ea_ret;
			end

			S_EA_D16: begin
				ea_addr <= (ea_pcmode ? ea_pcb : rf_rdata_a) + sxw(imm[15:0]);
				state <= r_ea_ret;
			end

			S_EA_EXTW: begin
				extw <= imm[15:0];
				rr_b <= {imm[15], imm[14:12]};
				ea_base_v <= ea_pcmode ? ea_pcb : rf_rdata_a;
				state <= S_EA_EXTW2;
			end

			S_EA_EXTW2: begin : ea_extw2
				reg [31:0] idx;
				idx = extw[11] ? rf_rdata_b : sxw(rf_rdata_b[15:0]);
				idx = idx << extw[10:9];
				if (!extw[8]) begin
					ea_addr <= ea_base_v + idx + sxb(extw[7:0]);
					state <= r_ea_ret;
				end
				else if (extw[5:4] == 2'b00 || extw[3] ||
				         extw[2:0] == 3'b100 || (extw[6] && extw[2])) begin
					go_illegal;
				end
				else begin
					ea_base_v <= extw[7] ? 32'd0 : ea_base_v;
					ea_idx_v  <= extw[6] ? 32'd0 : idx;
					ea_post   <= extw[2];
					ea_odl    <= (extw[1:0] == 2'b11);
					if (extw[5:4] == 2'b01) begin
						imm <= 0;
						state <= S_EA_BD;
					end
					else immf((extw[5:4] == 2'b10) ? 2'd1 : 2'd2, S_EA_BD);
				end
			end

			S_EA_BD: begin : ea_bd
				reg [31:0] bd;
				bd = (extw[5:4] == 2'b01) ? 32'd0 :
				     (extw[5:4] == 2'b10) ? sxw(imm[15:0]) : imm;
				if (extw[2:0] == 3'b000) begin
					ea_addr <= ea_base_v + ea_idx_v + bd;
					state <= r_ea_ret;
				end
				else begin
					// memory indirect: pre-indexed adds the index before the
					// indirection, post-indexed after
					mrd(ea_base_v + bd + (ea_post ? 32'd0 : ea_idx_v),
					    `AP040_SZ_L, S_EA_MIND);
				end
			end

			S_EA_MIND: begin
				ea_mind <= m_val;
				case (extw[1:0])
					2'b01: begin
						ea_addr <= m_val + (ea_post ? ea_idx_v : 32'd0);
						state <= r_ea_ret;
					end
					2'b10: immf(2'd1, S_EA_OD);
					default: immf(2'd2, S_EA_OD);
				endcase
			end

			S_EA_OD: begin
				ea_addr <= ea_mind + (ea_post ? ea_idx_v : 32'd0) +
				           (ea_odl ? imm : sxw(imm[15:0]));
				state <= r_ea_ret;
			end

			S_EA_ABS: begin
				ea_addr <= ea_absl ? imm : sxw(imm[15:0]);
				state <= r_ea_ret;
			end

			//------------------------------------------------ operand pipeline
			S_PIPE_START: begin
				// x_ext keeps a decode-time immediate through EA fetches;
				// for long MUL/DIV it was already captured in S_MDL_EXT
				if (exec_kind != EK_MD_L) x_ext <= imm;
				// A register destination needs no EA, so its operand can be
				// read on port B in the SAME cycle the source is read on
				// port A (X2.3).  The old path spent one state per port.
				case (p_src)
					SK_MEM: ea_start(src_mode_r, src_rn_r, p_ssize, S_PIPE_SRD);
					SK_REG:
						if (p_dst == DK_REG) begin
							rr_a <= p_sreg; rr_b <= p_dreg;
							state <= S_PIPE_REGS;
						end
						else begin rr_a <= p_sreg; state <= S_PIPE_SREG; end
					SK_IMM: begin
						src_val <= imm;
						if (p_dst == DK_REG) begin
							rr_b <= p_dreg; state <= S_PIPE_REGS;
						end
						else state <= S_PIPE_DST;
					end
					default:
						if (p_dst == DK_REG) begin
							rr_b <= p_dreg; state <= S_PIPE_REGS;
						end
						else state <= S_PIPE_DST;
				endcase
			end

			// The EA is finished by now, so port B is free: point it at a
			// register destination WHILE the source read is in flight, and
			// both operands land together when the read returns (X2.3).
			S_PIPE_SRD: begin
				if (p_dst == DK_REG) rr_b <= p_dreg;
				mrd(ea_addr, p_ssize, S_PIPE_SDONE);
			end
			S_PIPE_SDONE: begin
				src_val <= m_val;
				if (p_dst == DK_REG) begin
					dst_val <= rf_rdata_b;   // port B was set at S_PIPE_SRD
					state <= S_EXEC;
				end
				else state <= S_PIPE_DST;
			end
			S_PIPE_SREG:  begin src_val <= rf_rdata_a; state <= S_PIPE_DST; end

			S_PIPE_DST: begin
				case (p_dst)
					DK_MEM: ea_start(dst_mode_r, dst_rn_r, p_dsize, S_PIPE_DEA);
					DK_REG: begin rr_b <= p_dreg; state <= S_PIPE_DREG; end
					default: state <= S_EXEC;
				endcase
			end

			S_PIPE_DEA: begin
				dst_addr <= ea_addr;
				if (p_rmw) mrd(ea_addr, p_dsize, S_PIPE_DDONE);
				else state <= S_EXEC;
			end

			S_PIPE_DDONE: begin dst_val <= m_val; state <= S_EXEC; end
			S_PIPE_DREG:  begin dst_val <= rf_rdata_b; state <= S_EXEC; end

			// Both operands captured together.  src_val is taken only for a
			// REGISTER source: an immediate source was latched in
			// S_PIPE_START and a sourceless op never reads it.
			S_PIPE_REGS: begin
				if (p_src == SK_REG) src_val <= rf_rdata_a;
				dst_val <= rf_rdata_b;
				state <= S_EXEC;
			end

			//-------------------------------------------------------- execute
			S_EXEC: begin
				case (exec_kind)
					EK_SHIFT: begin
						sh_val <= dst_val;
						sh_fl <= sr[4:0];
						sh_vacc <= 0;
						sh_any <= 0;
						sh_cnt <= (p_src == SK_NONE) ? 6'd1 : src_val[5:0];
						state <= S_SHIFT;
					end

					EK_MD_W: begin
						if (md_isdiv && src_val[15:0] == 16'd0) begin
							// 68040 DIVU/DIVS divide-by-zero preserves X/N/Z/V
							// but clears C before taking vector 5.
							sr[0] <= 1'b0;
							exc(`AP040_VEC_DIVZERO, 4'd2, pc, pc_i);
						end
						else begin
							md_a  <= md_sign ? sxw(src_val[15:0]) : {16'd0, src_val[15:0]};
							md_hi <= md_sign ? {32{dst_val[31]}} : 32'd0;
							md_lo <= md_isdiv ? dst_val
							         : (md_sign ? sxw(dst_val[15:0]) : {16'd0, dst_val[15:0]});
							md_start <= 1;
							state <= S_MD_WAIT;
						end
					end

					EK_MD_L: begin
						// stage the read of Dl/Dq named in the extension word
						md_sign <= x_ext[11];
						rr_b <= {1'b0, x_ext[14:12]};
						state <= S_MDL_RDQ;
					end

					EK_CHK: begin : ek_chk
						reg signed [31:0] v, bound;
						v = (op_size == `AP040_SZ_W) ? $signed(sxw(dst_val[15:0]))
						                             : $signed(dst_val);
						bound = (op_size == `AP040_SZ_W) ? $signed(sxw(src_val[15:0]))
						                                 : $signed(src_val);
						// 68040 flags: N always tracks the value's sign; C is
						// cleared in bounds and set on a trap only for these
						// sign combinations; Z, V and X are left unchanged
						// (cputest 68040_default reference on hardware)
						sr[3] <= (v < 0);
						if (v < 0 || v > bound) begin
							sr[0] <= (v < 0 && bound >= 0) ||
							         (bound >= 0 && v >= bound) ||
							         (v < 0 && bound < v);
							exc(`AP040_VEC_CHK, 4'd2, pc, pc_i);
						end
						else begin
							sr[0] <= 0;
							fetch_next;
						end
					end

					EK_SCC: begin : ek_scc
						reg [31:0] r;
						r = {24'd0, {8{cond_true(ir[11:8])}}};
						if (p_dst == DK_REG) begin
							rfw(p_dreg, merge_sz(dst_val, r, `AP040_SZ_B));
							fetch_next;
						end
						else mwr(dst_addr, `AP040_SZ_B, r, S_NEXT);
					end

					EK_PACK: begin : ek_pack
						reg [15:0] v;
						v = src_val[15:0] + x_ext[15:0];
						if (p_dst == DK_REG) begin
							rfw(p_dreg, merge_sz(dst_val, {24'd0, v[11:8], v[3:0]}, `AP040_SZ_B));
							fetch_next;
						end
						else mwr(dst_addr, `AP040_SZ_B, {24'd0, v[11:8], v[3:0]}, S_NEXT);
					end

					EK_UNPK: begin : ek_unpk
						reg [15:0] v;
						v = {4'd0, src_val[7:4], 4'd0, src_val[3:0]} + x_ext[15:0];
						if (p_dst == DK_REG) begin
							rfw(p_dreg, merge_sz(dst_val, {16'd0, v}, `AP040_SZ_W));
							fetch_next;
						end
						else mwr(dst_addr, `AP040_SZ_W, {16'd0, v}, S_NEXT);
					end

					default: begin // EK_ALU
						if (p_flags) sr[4:0] <= alu_fl;
						if (p_wbsup) fetch_next;
						else case (p_dst)
							DK_MEM: mwr(dst_addr, p_dsize, alu_res, S_NEXT);
							DK_REG: begin
								if (p_dreg[3])
									rfw(p_dreg, alu_res);
								else
									rfw(p_dreg, merge_sz(dst_val, alu_res, op_size));
								fetch_next;
							end
							// SR settles first so the next fetch uses the new
						// S bit's FC (and trace enables)
						DK_SR:  begin sr <= alu_res[15:0] & `AP040_SR_MASK; state <= S_NEXT; end
							DK_CCR: begin sr[4:0] <= alu_res[4:0]; fetch_next; end
							default: fetch_next;
						endcase
					end
				endcase
			end

			//--------------------------------------------------------- shifts
			S_SHIFT: begin
				if (sh_cnt == 6'd0) begin
					sr[4] <= sh_fl[4];
					sr[3] <= (op_size == `AP040_SZ_B) ? sh_val[7] :
					         (op_size == `AP040_SZ_W) ? sh_val[15] : sh_val[31];
					sr[2] <= ((sh_val & ((op_size == `AP040_SZ_B) ? 32'hFF :
					          (op_size == `AP040_SZ_W) ? 32'hFFFF : 32'hFFFFFFFF)) == 0);
					sr[1] <= sh_vacc;
					// zero count: C=0 for shifts/rotates, C=X for ROXx
					sr[0] <= sh_any ? sh_fl[0] : (sh_rox ? sh_fl[4] : 1'b0);
					state <= S_SHIFT_WB;
				end
				else begin
					// single-cycle barrel: the ALU composed the whole count,
					// commit value and flags directly
					sh_val <= alu_res;
					sr[4] <= alu_fl[4];
					sr[3] <= alu_fl[3];
					sr[2] <= alu_fl[2];
					sr[1] <= alu_fl[1];
					sr[0] <= alu_fl[0];
					state <= S_SHIFT_WB;
				end
			end

			S_SHIFT_WB: begin
				if (p_dst == DK_REG) begin
					rfw(p_dreg, merge_sz(dst_val, sh_val, op_size));
					fetch_next;
				end
				else mwr(dst_addr, p_dsize, sh_val, S_NEXT);
			end

			//------------------------------------------------ multiply/divide
			S_MDL_EXT: begin
				x_ext <= imm;
				if (p_src == SK_IMM) immf(2'd2, S_PIPE_START);
				else state <= S_PIPE_START;
			end

			S_MDL_RDQ: begin
				// rf_rdata_b is Dl (multiply) or Dq (divide low dividend)
				if (md_isdiv && src_val == 32'd0) begin
					// 68040 DIVL divide-by-zero preserves X/N/Z/V
					// but clears C before taking vector 5.
					sr[0] <= 1'b0;
					exc(`AP040_VEC_DIVZERO, 4'd2, pc, pc_i);
				end
				else if (md_isdiv && x_ext[10]) begin
					dst_val <= rf_rdata_b;
					rr_a <= {1'b0, x_ext[2:0]};   // Dr holds the high dividend
					state <= S_MDL_RDR;
				end
				else begin
					md_a  <= src_val;
					md_hi <= md_isdiv ? (x_ext[11] ? {32{rf_rdata_b[31]}} : 32'd0) : 32'd0;
					md_lo <= rf_rdata_b;
					md_start <= 1;
					state <= S_MD_WAIT;
				end
			end

			S_MDL_RDR: begin
				md_a  <= src_val;
				md_hi <= rf_rdata_a;
				md_lo <= dst_val;
				md_start <= 1;
				state <= S_MD_WAIT;
			end

			S_MD_WAIT: if (md_done) begin
				if (exec_kind == EK_MD_W) begin
					if (md_isdiv) begin : mdw_div
						reg ovf_w;
						ovf_w = md_ovf |
						        (md_sign ? (($signed(md_rlo) > 32'sd32767) ||
						                    ($signed(md_rlo) < -32'sd32768))
						                 : (md_rlo > 32'h0000_FFFF));
						if (ovf_w) begin
							sr[1] <= 1; sr[0] <= 0;
							fetch_next;
						end
						else begin
							rfw(p_dreg, {md_rhi[15:0], md_rlo[15:0]});
							sr[3] <= md_rlo[15];
							sr[2] <= (md_rlo[15:0] == 16'd0);
							sr[1] <= 0; sr[0] <= 0;
							fetch_next;
						end
					end
					else begin
						rfw(p_dreg, md_rlo);
						sr[3] <= md_rlo[31];
						sr[2] <= (md_rlo == 32'd0);
						sr[1] <= 0; sr[0] <= 0;
						fetch_next;
					end
				end
				else begin // EK_MD_L
					if (md_isdiv) begin
						if (md_ovf) begin
							// 68040 divide overflow (32- and 64-bit forms):
							// V=1, C=0, N/Z and the destination registers
							// are left unchanged
							sr[1] <= 1; sr[0] <= 0;
							fetch_next;
						end
						else begin
							rfw({1'b0, x_ext[14:12]}, md_rlo);  // quotient to Dq
							sr[3] <= md_rlo[31];
							sr[2] <= (md_rlo == 32'd0);
							sr[1] <= 0;
							sr[0] <= 0;
							if (x_ext[2:0] != x_ext[14:12]) state <= S_MD_WB2;
							else fetch_next;
						end
					end
					else begin
						rfw({1'b0, x_ext[14:12]}, md_rlo);  // low product to Dl
						if (x_ext[10]) begin
							sr[3] <= md_rhi[31];
							sr[2] <= (md_rhi == 32'd0) && (md_rlo == 32'd0);
							sr[1] <= 0; sr[0] <= 0;
							// the 68040 writes Dh before Dl (020/030 write Dl
							// first), so with Dh==Dl the register must keep
							// the LOW half: skip the high write
							if (x_ext[2:0] != x_ext[14:12]) state <= S_MD_WB2;
							else fetch_next;
						end
						else begin
							sr[3] <= md_rlo[31];
							sr[2] <= (md_rlo == 32'd0);
							sr[1] <= (x_ext[11] ? (md_rhi != {32{md_rlo[31]}})
							                    : (md_rhi != 32'd0));
							sr[0] <= 0;
							fetch_next;
						end
					end
				end
			end

			S_MD_WB2: begin
				rfw({1'b0, x_ext[2:0]}, md_rhi);   // remainder to Dr / high to Dh
				fetch_next;
			end

			//------------------------------------------------------ exceptions
			//------------------------------------- access error (format $7)
			S_AERR0: begin
				// The address-register rollback below must run BEFORE the S
				// bit changes the stack selection.  A7 is a shadowed
				// register -- writes reach USP, ISP or MSP according to
				// S/M -- so undoing an A7 update from a USER-mode
				// instruction after setting S restores the user value into
				// the SUPERVISOR pointer.  The frame is then stacked in
				// user space, that write faults, and the fault-during-
				// exception halts the core: the silent NetBSD freeze,
				// captured on hardware as A7=1dfff9b8 (a user stack) in
				// supervisor mode with IR=209f (MOVE.L (A7)+,(A0), libc's
				// __cerror storing through the errno pointer).  Roll back
				// first, in the faulting instruction's own context, and
				// only then enter the exception.
				sr_saved <= sr;
				aer_idx <= 0;
				state <= S_AERR_U;
			end

			S_AERR_U: begin
				// roll back address register updates so RTE restarts the
				// instruction from a clean context (68040 restart model)
				if (u1_v) begin
					rfw(u1_reg, u1_old);
					u1_v <= 0;
				end
				else if (u0_v) begin
					rfw(u0_reg, u0_old);
					u0_v <= 0;
				end
				else begin
					// rollback complete: now switch to supervisor state
					sr[13] <= 1;
					sr[15:14] <= 2'b00;
					in_exc <= 1;
					state <= S_AERR_SP;
				end
			end

			S_AERR_SP: begin
				aer_sp <= dbg_a7 - 32'd60;
				state <= S_AERR_WR;
			end

			S_AERR_WR: begin
				if (aer_idx == 5'd30) begin
					rfw(4'd15, aer_sp);
					exc_vec <= `AP040_VEC_BUSERR;
					state <= S_EXC_VEC;
				end
				else begin
					aer_idx <= aer_idx + 5'd1;
					mwr(aer_sp + {26'd0, aer_idx, 1'b0}, `AP040_SZ_W,
					    {16'd0, aerr_word(aer_idx)}, S_AERR_WR);
				end
			end

			S_EXC0: begin
				if (fpu_bg) state <= S_EXC0;   // FSAVE-quiescent exception
				// An abandoned queue fetch may still be on the bus under the
				// pre-exception function code.  Exception processing owns the
				// port from in_exc onwards, so let it retire first.
				else if (epf_pend) state <= S_EXC0;
				else begin : exc0_run
				// Kill any address-register rollback records on the way into
				// a non-access exception.  The records exist solely so the
				// access-error path can restore the FAULTING instruction's
				// (An)+/-(An) side effects; only fetch_next and the S_AERR
				// consumer clear them, and an instruction whose EA succeeded
				// but which then raises CHK / zero-divide / an FP trap
				// reaches here with its record still live.  Left alone it
				// survives exception_prefetch into the handler, where the
				// next access error "rolls back" an unrelated instruction's
				// register to a stale value -- possibly from the other
				// privilege context.
				u0_v <= 0;
				u1_v <= 0;
				sr_saved <= sr;
				sr[13] <= 1;
				sr[15:14] <= 2'b00;
				in_exc <= 1;
				if (exc_is_irq) begin
					sr[10:8] <= irq_lvl_l;
					if (irq_lvl_l == 3'd7) nmi_ack_t <= ~nmi_ack_t;
					else irq_ack_t <= ~irq_ack_t;
				end
				state <= S_EXC1;
				end
			end

			S_EXC1: begin
				exc_sp <= dbg_a7 - exc_fsize;
				mwr(dbg_a7 - exc_fsize,
				    `AP040_SZ_W, {16'd0, sr_saved}, S_EXC2);
			end

			S_EXC2: mwr(exc_sp + 32'd2, `AP040_SZ_L, exc_spc, S_EXC3);

			S_EXC3: mwr(exc_sp + 32'd6, `AP040_SZ_W,
			            {16'd0, exc_fmt, 2'b00, exc_vec, 2'b00},
			            (exc_fmt == 4'd2 || exc_fmt == 4'd3 ||
			             exc_fmt == 4'd4) ? S_EXC4 : S_EXC5);

			// Formats $2/$3 carry one additional longword.  Format $4,
			// recognized only by LC/EC configurations, carries two.
			S_EXC4: mwr(exc_sp + 32'd8, `AP040_SZ_L, exc_addr,
			            (exc_fmt == 4'd4) ? S_EXC4B : S_EXC5);

			S_EXC4B: mwr(exc_sp + 32'd12, `AP040_SZ_L, pc_i, S_EXC5);

			S_EXC5: begin
				// this A7 write commits on the next ce edge, while SR.M is
				// still set for the master stack case
				rfw(4'd15, exc_sp);
				if (exc_is_irq && sr[12] && !exc_pass2) state <= S_EXC6;
				else state <= S_EXC_VEC;
			end

			S_EXC6: begin
				// interrupt with M set: clear M and build a format $1
				// throwaway frame on the interrupt stack.  Its SR image is
				// the ORIGINAL SR with only S forced (WinUAE: regs.sr |=
				// 1<<13 before the second push): the original trace bits and
				// interrupt mask survive, and M stays set so that RTE's
				// format $1 continuation switches back to the master stack
				// where the real frame lives.
				sr[12] <= 0;
				sr_saved <= sr_saved | 16'h2000;
				exc_fmt <= 4'd1;
				exc_pass2 <= 1;
				state <= S_EXC1;
			end

			S_EXC_VEC: mrd(vbr + {22'd0, exc_vec, 2'b00}, `AP040_SZ_L, S_EXC_JMP);

			S_EXC_JMP: begin
				if (m_val[0]) begin
					if (exc_vec == 8'd2 || exc_vec == 8'd3) begin
						// odd bus/address error handler: double fault, halt
						fatal_halt;
					end
					else begin
						// Any other odd handler address becomes an address
						// error.  The frame's PC field identifies the vector
						// that supplied the odd address as its OFFSET --
						// 4 * vector, WITHOUT vbr -- not the original
						// exception's next-PC context.  WinUAE says so in
						// as many words on the path it models explicitly
						// ("offset, not vbr + offset").  Hardware settled
						// it: with cputest's own vbr ($403e4e68) a frame
						// built from vbr + 4*vec read $403e4e88 where the
						// corpus expects $00000020 for vector 8.  The
						// address field carries the odd target with A0
						// cleared.
						texc_pend <= 0;
						exc(`AP040_VEC_ADDRERR, 4'd2,
						    {22'd0, exc_vec, 2'b00},
						    {m_val[31:1], 1'b0});
					end
				end
				else if (texc_pend) begin
					// the surviving T0 trace: vector 9, format $2, stacked
					// PC = handler entry, address field = the instruction
					// that took the original exception.  WinUAE's DOTRACE
					// fires before a pending interrupt is sampled.
					texc_pend <= 0;
					pc <= m_val;
					exc(`AP040_VEC_TRACE, 4'd2, m_val, texc_pc);
				end
				else if (irq_pend) begin
					// An interrupt pending when another exception finishes is
					// stacked before the original handler executes.  Its frame
					// returns to that handler address.
					pc <= m_val;
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
					exc_fmt <= 0; exc_spc <= m_val; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= irq_take_lvl;
					epf_flush;
					state <= S_EXC0;
				end
				else begin
					exception_prefetch(m_val, sr_s);
				end
			end

			//------------------------------------------------------------- RTE
			S_RTE_SR:  mrd(dbg_a7, `AP040_SZ_W, S_RTE_PC);
			S_RTE_PC:  begin rte_sr <= m_val[15:0]; mrd(dbg_a7 + 32'd2, `AP040_SZ_L, S_RTE_FMT); end
			S_RTE_FMT: begin rte_pc <= m_val; mrd(dbg_a7 + 32'd6, `AP040_SZ_W, S_RTE_FIN); end

			S_RTE_FIN: begin
				case (m_val[15:12])
					4'd0, 4'd1: begin
						rfw(4'd15, dbg_a7 + 32'd8);
						ret_kind <= {1'b0, m_val[12]};  // reuse: bit0 = again
						state <= S_RTE_FIN2;
					end
					4'd2, 4'd3: begin
						rfw(4'd15, dbg_a7 + 32'd12);
						ret_kind <= 2'b00;
						state <= S_RTE_FIN2;
					end
					4'd4: begin
						// A full MC68040 does not recognize format $4.  The
						// eight-word frame belongs to the LC/EC variants.
						if (AP040_HAS_FPU != 0)
							exc(`AP040_VEC_FMTERR, 4'd0, pc_i, 32'd0);
						else begin
							rfw(4'd15, dbg_a7 + 32'd16);
							ret_kind <= 2'b00;
							state <= S_RTE_FIN2;
						end
					end
					4'd7: begin
						// access error frame: restart semantics, the
						// continuation/writeback fields are not consumed
						rfw(4'd15, dbg_a7 + 32'd60);
						ret_kind <= 2'b00;
						state <= S_RTE_FIN2;
					end
					default: exc(`AP040_VEC_FMTERR, 4'd0, pc_i, 32'd0);
				endcase
			end

			S_RTE_FIN2: begin
				sr <= rte_sr & `AP040_SR_MASK;
				if (ret_kind[0]) begin
					// format $1: continue with the next frame; the popped
					// SR becomes the "before" image for the odd-PC quirk
					state <= S_RTE_SR;
				end
				else if (rte_pc[0]) begin
					// The odd restored PC is detected after the RTE has
					// committed its SR.  Consequently the address-error frame
					// carries the restored SR, just as RTR carries its popped
					// CCR (68040_ae RTE/RTR corpus behavior).
					exc(`AP040_VEC_ADDRERR, 4'd2, pc_i,
					    {rte_pc[31:1], 1'b0});
				end
				else if (tr_t1 || tr_t0) begin
					// the RTE itself was traced (T set before the RTE)
					tr_t1 <= 0;
					tr_t0 <= 0;
					pc <= rte_pc;
					exc(`AP040_VEC_TRACE, 4'd2, rte_pc, pc_i);
				end
				else if (rte_irq_pend) begin
					// The restored mask unblocks a pending request: it is
					// taken AT this boundary, before the instruction RTE
					// returns to.  This path used to go straight to
					// S_FETCH without sampling interrupts at all -- unlike
					// fetch_next and go_pc -- so the target instruction ran
					// first and the interrupt was reported one instruction
					// late.  cputest enters every test through RTE, which
					// is why irq/all saw it on hardware while the
					// MOVE-to-SR path looked correct.
					in_exc <= 0;
					pc <= rte_pc;
					pc_i <= rte_pc;
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, rte_irq_lvl};
					exc_fmt <= 0; exc_spc <= rte_pc; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= rte_irq_lvl;
					epf_flush;
					state <= S_POST_EXC;
				end
				else begin
					// fetch under the restored context's FC (SR is being
					// written this same cycle)
					in_exc <= 0;
					pc <= rte_pc;
					pc_i <= rte_pc;
					issue_ifetch(rte_pc, rte_sr[13]);
					state <= S_FETCH;
				end
			end

			//------------------------------------------------ RTS / RTR / RTD
			S_RET1: begin
				if (ret_kind == RK_RTR) mrd(dbg_a7, `AP040_SZ_W, S_RET2);
				else mrd(dbg_a7, `AP040_SZ_L, S_RET2);
			end

			S_RET2: begin
				case (ret_kind)
					RK_RTR: begin
						sr[4:0] <= m_val[4:0];
						mrd(dbg_a7 + 32'd2, `AP040_SZ_L, S_RET3);
					end
					// odd return address: the 68040 backs the pop out of A7
					// before taking the address error (gencpu cpu_level>=4
					// rolls areg7 back), so the fault frame sees the
					// pre-return stack pointer
					RK_RTD: begin
						if (!m_val[0]) rfw(4'd15, dbg_a7 + 32'd4 + sxw(imm[15:0]));
						go_pc(m_val);
					end
					default: begin
						if (!m_val[0]) rfw(4'd15, dbg_a7 + 32'd4);
						go_pc(m_val);
					end
				endcase
			end

			S_RET3: begin
				if (m_val[0]) begin
					// Odd return address: A7 keeps its pre-RTR value (the
					// 68040 backs the pop out before the address error,
					// cputest 68040_ae RTR round 0 checks A7 exactly), and
					// the frame stacks the SR with the CCR already popped in
					// S_RET2 -- the v20 corpus data validates that frame
					// byte.  (Newer WinUAE models a 68040 quirk stacking the
					// pre-RTR SR instead, exception3_read_prefetch_68040bug;
					// the v20 generator predates it, and the corpus is the
					// hardware acceptance test.)  The PC field identifies
					// the pre-opcode pipeline word.
					exc(`AP040_VEC_ADDRERR, 4'd2, pc_i,
					    {m_val[31:1], 1'b0});
				end
				else begin
					rfw(4'd15, dbg_a7 + 32'd6);
					go_pc(m_val);
				end
			end

			//------------------------------------------------------- branches
			S_BCC_EXT: begin : bcc_ext
				reg [31:0] tgt;
				tgt = br_base + (br_long ? imm : sxw(imm[15:0]));
				if (ir[11:8] == 4'h1) begin
					if (tgt[0]) go_pc(tgt); // odd target: fault BEFORE the push
					else begin
						br_tgt <= tgt;
						mwr(dbg_a7 - 32'd4, `AP040_SZ_L, pc, S_BSR_PUSH);
					end
				end
				else finish_bcc(tgt, cond_true(ir[11:8]));
			end

			S_BSR_PUSH: begin
				rfw(4'd15, dbg_a7 - 32'd4);
				go_pc(br_tgt);
			end

			S_DBCC1: begin : dbcc1
				// 68040 checks the branch-target parity BEFORE the
				// condition (gencpu cpu_level>=4 emits the odd test ahead
				// of cctrue): DBT to an odd label faults even though the
				// loop exits without branching (cputest 68040_ae DBcc.W).
				reg [31:0] tgt;
				tgt = br_base + sxw(imm[15:0]);
				if (tgt[0]) go_pc(tgt);
				else if (cond_true(ir[11:8])) fetch_next;
				else begin
					rr_a <= {1'b0, d_rn};
					state <= S_DBCC2;
				end
			end

			S_DBCC2: begin : dbcc2
				reg [15:0] w;
				w = rf_rdata_a[15:0] - 16'd1;
				rfw({1'b0, d_rn}, {rf_rdata_a[31:16], w});
				if (w != 16'hFFFF) go_pc(br_base + sxw(imm[15:0]));
				else fetch_next;
			end

			//------------------------------------------- jumps and stack frame
			S_JMP1:
				if (ea_addr[0])
					// gencpu's i_JMP does incpc(2) before
					// exception3_read_prefetch_only, and that path is NOT
					// gated on cpu_level, so the frame PC is measured from
					// wherever the PC had reached -- not from the
					// instruction address.  For (An), (d16,An) and absw
					// nothing has synced it yet, giving pc_i + 2; the
					// INDEXED modes resolve their extension against the
					// real PC first, so it has already advanced past the
					// extension word and the frame reads pc_i + 6.  Both
					// values are what the v24 AE group records.
					exc(`AP040_VEC_ADDRERR, 4'd2,
					    (ea_mode == 3'b110 ||
					     (ea_mode == 3'b111 && ea_rn == 3'd3))
					        ? pc_i + 32'd6 : pc_i + 32'd2,
					    {ea_addr[31:1], 1'b0});
				else go_pc(ea_addr);

			S_JSR1: begin
				if (ea_addr[0])
					// Unlike JMP, gencpu guards i_JSR's odd-target case with
					// cpu_level <= 1.  A 68040 therefore takes the fault on
					// the INSTRUCTION FETCH at the odd target, so the frame
					// names that target, not this instruction.
					exc(`AP040_VEC_ADDRERR, 4'd2, ea_addr,
					    {ea_addr[31:1], 1'b0});
				else begin
					br_tgt <= ea_addr;
					mwr(dbg_a7 - 32'd4, `AP040_SZ_L, pc, S_JSR2);
				end
			end

			S_JSR2: begin
				rfw(4'd15, dbg_a7 - 32'd4);
				go_pc(br_tgt);
			end

			S_LEA1: begin
				rfw({1'b1, d_reg9}, ea_addr);
				fetch_next;
			end

			S_PEA1: mwr(dbg_a7 - 32'd4, `AP040_SZ_L, ea_addr, S_PEA2);

			S_PEA2: begin
				rfw(4'd15, dbg_a7 - 32'd4);
				fetch_next;
			end

			S_LINK1: begin
				rr_a <= {1'b1, d_rn};
				state <= S_LINK2;
			end

			S_LINK2: begin : link2
				reg [31:0] spn;
				spn = dbg_a7 - 32'd4;
				t_a <= spn;
				mwr(spn, `AP040_SZ_L, (d_rn == 3'd7) ? spn : rf_rdata_a, S_LINK3);
			end

			S_LINK3: begin
				rfw({1'b1, d_rn}, t_a);
				state <= S_LINK4;
			end

			S_LINK4: begin
				rfw(4'd15, t_a + (br_long ? imm : sxw(imm[15:0])));
				fetch_next;
			end

			S_UNLK1: begin
				t_a <= rf_rdata_a;
				mrd(rf_rdata_a, `AP040_SZ_L, S_UNLK2);
			end

			S_UNLK2: begin
				rfw(4'd15, t_a + 32'd4);
				state <= S_UNLK3;
			end

			S_UNLK3: begin
				rfw({1'b1, d_rn}, m_val);
				fetch_next;
			end

			//---------------------------------------------------------- MOVEM
			S_MOVEM_SET: begin
				mm_mask <= imm[15:0];
				// the EA depends on An for these modes: a LOADED base
				// register must not be written mid-loop (restart safety;
				// see S_MOVEM_LD)
				mm_base_ea <= (d_mode == 3'b010) || (d_mode == 3'b011) ||
				              (d_mode == 3'b101) || (d_mode == 3'b110);
				mm_base_pend <= 0;
				if (mm_predec || mm_postinc) begin
					rr_a <= {1'b1, d_rn};
					state <= S_MOVEM_SET2;
				end
				else ea_start(d_mode, d_rn, mm_size, S_MOVEM_EA);
			end

			S_MOVEM_SET2: begin
				mm_addr <= rf_rdata_a;
				mm_init_an <= rf_rdata_a;
				state <= S_MOVEM_LOOP;
			end

			S_MOVEM_EA: begin
				mm_addr <= ea_addr;
				state <= S_MOVEM_LOOP;
			end

			S_MOVEM_LOOP: begin
				if (mm_mask == 16'd0) begin
					if (mm_predec || mm_postinc)
						rfw({1'b1, d_rn}, mm_addr);
					else if (mm_base_pend)
						rfw({1'b1, d_rn}, mm_base_val);
					fetch_next;
				end
				else begin : movem_step
					reg [3:0] bit_i;
					bit_i = ffs16(mm_mask);
					mm_mask <= mm_mask & ~(16'd1 << bit_i);
					if (mm_predec) begin
						mm_reg <= 4'd15 - bit_i;
						mm_addr <= mm_addr - ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
						rr_a <= 4'd15 - bit_i;
						state <= S_MOVEM_RD;
					end
					else begin
						mm_reg <= bit_i;
						if (mm_dir) mrd(mm_addr, mm_size, S_MOVEM_LD);
						else begin
							rr_a <= bit_i;
							state <= S_MOVEM_RD;
						end
					end
				end
			end

			S_MOVEM_RD: begin : movem_rd
				reg [31:0] v;
				// predec MOVEM with the base register in the list: the
				// 68020/030/040 store the initial value minus the operation
				// size (the 68000/010 store the undecremented value)
				v = (mm_predec && mm_reg == {1'b1, d_rn})
				    ? (mm_init_an - ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2))
				    : rf_rdata_a;
				if (mm_predec) mwr(mm_addr, mm_size, v, S_MOVEM_LOOP);
				else begin
					mwr(mm_addr, mm_size, v, S_MOVEM_LOOP);
					mm_addr <= mm_addr + ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
				end
			end

			S_MOVEM_LD: begin : movem_ld
				reg [31:0] lv;
				lv = (mm_size == `AP040_SZ_W) ? sxw(m_val[15:0]) : m_val;
				// A loaded register that is also the EA base is not written
				// mid-loop: a fault on a LATER transfer restarts the whole
				// instruction and would recompute the EA from the loaded
				// DATA.  For (An)+ the final address writeback wins anyway
				// (the 040 leaves the postincremented address, not the
				// memory value); for the control modes the loaded value is
				// held and committed with the last transfer.
				if (mm_base_ea && mm_reg == {1'b1, d_rn}) begin
					if (!mm_postinc) begin
						mm_base_pend <= 1;
						mm_base_val  <= lv;
					end
				end
				else rfw(mm_reg, lv);
				mm_addr <= mm_addr + ((mm_size == `AP040_SZ_L) ? 32'd4 : 32'd2);
				state <= S_MOVEM_LOOP;
			end

			//---------------------------------------------------------- MOVEP
			S_MOVEP1: begin
				rr_a <= {1'b1, d_rn};
				rr_b <= {1'b0, d_reg9};
				state <= S_MOVEP2;
			end

			S_MOVEP2: begin
				mp_addr <= rf_rdata_a + sxw(imm[15:0]);
				mp_val <= rf_rdata_b;
				mp_idx <= 0;
				if (mp_dir) state <= S_MOVEP_WR;
				else state <= S_MOVEP_RD;
			end

			S_MOVEP_WR: begin
				if (mp_idx == mp_cnt) fetch_next;
				else begin : movep_wr
					reg [7:0] byv;
					case ({mp_cnt[2], mp_idx[1:0]})
						{1'b1, 2'd0}: byv = mp_val[31:24];
						{1'b1, 2'd1}: byv = mp_val[23:16];
						{1'b1, 2'd2}: byv = mp_val[15:8];
						{1'b1, 2'd3}: byv = mp_val[7:0];
						{1'b0, 2'd0}: byv = mp_val[15:8];
						default:      byv = mp_val[7:0];
					endcase
					mp_idx <= mp_idx + 3'd1;
					mwr(mp_addr + {28'd0, mp_idx[1:0], 1'b0}, `AP040_SZ_B,
					    {24'd0, byv}, S_MOVEP_WR);
				end
			end

			S_MOVEP_RD: begin
				if (mp_idx != 0) begin
					mp_val <= {mp_val[23:0], m_val[7:0]};
				end
				if (mp_idx == mp_cnt) begin : movep_fin
					reg [31:0] nv;
					nv = {mp_val[23:0], m_val[7:0]};
					if (mp_cnt[2]) rfw({1'b0, d_reg9}, nv);
					else rfw({1'b0, d_reg9}, {rf_rdata_b[31:16], nv[15:0]});
					fetch_next;
				end
				else begin
					mp_idx <= mp_idx + 3'd1;
					mrd(mp_addr + {28'd0, mp_idx[1:0], 1'b0}, `AP040_SZ_B, S_MOVEP_RD);
				end
			end

			//----------------------------------------------------- EXG / misc
			S_EXG1: begin
				t_a <= rf_rdata_a;
				rfw(rr_a, rf_rdata_b);
				state <= S_EXG2;
			end

			S_EXG2: begin
				rfw(rr_b, t_a);
				fetch_next;
			end

			S_USP1: begin
				aux_we <= 1; aux_sel <= 2'd0; aux_wdata <= rf_rdata_a;
				fetch_next;
			end

			//---------------------------------------------------------- MOVEC
			S_MOVEC1: begin
				epf_flush;
				if (!movec_valid(imm[11:0])) go_illegal;
				else if (mvc_dir) begin
					rr_a <= {imm[15], imm[14:12]};
					state <= S_MOVEC2;
				end
				else begin
					rfw({imm[15], imm[14:12]}, movec_rd(imm[11:0]));
					state <= S_NEXT;
				end
			end

			S_MOVEC2: begin
				epf_flush;      // control-register access serializes fetch
				case (imm[11:0])
					12'h000: sfc <= rf_rdata_a[2:0];
					12'h001: dfc <= rf_rdata_a[2:0];
					12'h002: cacr <= rf_rdata_a & 32'h8000_8000;
					12'h003: begin
						tc <= rf_rdata_a & 32'h0000_C000;
					end
					12'h004: itt0 <= rf_rdata_a & 32'hFFFF_E364;
					12'h005: itt1 <= rf_rdata_a & 32'hFFFF_E364;
					12'h006: dtt0 <= rf_rdata_a & 32'hFFFF_E364;
					12'h007: dtt1 <= rf_rdata_a & 32'hFFFF_E364;
					12'h800: begin aux_we <= 1; aux_sel <= 2'd0; aux_wdata <= rf_rdata_a; end
					12'h801: vbr <= rf_rdata_a;
					12'h803: begin aux_we <= 1; aux_sel <= 2'd2; aux_wdata <= rf_rdata_a; end
					12'h804: begin aux_we <= 1; aux_sel <= 2'd1; aux_wdata <= rf_rdata_a; end
					12'h805: mmusr <= rf_rdata_a;
					12'h806: urp <= rf_rdata_a & 32'hFFFF_FE00;
					default: srp <= rf_rdata_a & 32'hFFFF_FE00;
				endcase
				// MMU register accesses never invalidate either ATC.  Software
				// must issue PFLUSH explicitly when a register write changes a
				// translation (MC68040 UM 3.7.4).
				state <= S_NEXT;
			end

			//---------------------------------------------------------- MOVES
			S_MOVES1: begin
				x_ext <= imm;
				ea_start(d_mode, d_rn, op_size, S_MOVES2);
			end

			S_MOVES2: begin
				if (x_ext[11]) begin
					rr_a <= {x_ext[15], x_ext[14:12]};
					state <= S_MOVES_WR;
				end
				else begin
					rr_b <= {x_ext[15], x_ext[14:12]};   // old value for merge
					fc_ovr_v <= 1; fc_ovr <= sfc;
					mrd(ea_addr, op_size, S_MOVES_RD);
				end
			end

			S_MOVES_WR: begin
				fc_ovr_v <= 1; fc_ovr <= dfc;
				mwr(ea_addr, op_size, rf_rdata_a, S_NEXT);
			end

			S_MOVES_RD: begin
				if (x_ext[15])
					rfw({x_ext[15], x_ext[14:12]},
					    (op_size == `AP040_SZ_W) ? sxw(m_val[15:0]) :
					    (op_size == `AP040_SZ_B) ? sxb(m_val[7:0]) : m_val);
				else
					rfw({x_ext[15], x_ext[14:12]},
					    merge_sz(rf_rdata_b, m_val, op_size));
				fetch_next;
			end

			//------------------------------------------------- PTEST / PFLUSH
			S_PTEST1: begin
				epf_flush;      // PTEST replaces the matching ATC entry
				pt_addr <= rf_rdata_a;
				pt_write <= ~ir[5];
				state <= S_PTEST2;
			end

			// The table walker has its own memory port.  An abandoned queue
			// fetch may still be on the CPU bus, so the probe waits for it to
			// retire rather than running two masters at once.
			S_PTEST2: if (!pt_req) begin
				if (!epf_pend) pt_req <= 1;
			end
			else if (pt_done) begin
				pt_req <= 0;
				mmusr <= pt_mmusr;
				fetch_next;
			end

			S_PFLUSH1: begin
				epf_flush;
				pf_addr <= rf_rdata_a;
				state <= S_PFLUSH2;
			end

			S_PFLUSH2: if (!pf_req) begin
				if (!epf_pend) pf_req <= 1;
			end
			else if (pf_done) begin
				pf_req <= 0;
				fetch_next;
			end

			S_CINV2: if (cinv_done) begin
				cinv_req <= 0;
				fetch_next;
			end

			//----------------------------------------------------- CHK2/CMP2
			S_CHK2_A: begin
				x_ext <= imm;
				ea_start(d_mode, d_rn, op_size, S_CHK2_B);
			end

			S_CHK2_B: begin
				dst_addr <= ea_addr;
				mrd(ea_addr, op_size, S_CHK2_C);
			end

			S_CHK2_C: begin
				src_val <= m_val;              // lower bound
				rr_a <= {x_ext[15], x_ext[14:12]};
				mrd(dst_addr + ((op_size == `AP040_SZ_B) ? 32'd1 :
				                (op_size == `AP040_SZ_W) ? 32'd2 : 32'd4),
				    op_size, S_CHK2_D);
			end

			S_CHK2_D: begin : chk2d
				reg signed [31:0] rn, lb, ub;
				reg oob;
				// operands sign-extended by size; address registers use
				// their full value
				if (x_ext[15]) rn = $signed(rf_rdata_a);
				else rn = (op_size == `AP040_SZ_B) ? $signed(sxb(rf_rdata_a[7:0])) :
				          (op_size == `AP040_SZ_W) ? $signed(sxw(rf_rdata_a[15:0])) :
				          $signed(rf_rdata_a);
				lb = (op_size == `AP040_SZ_B) ? $signed(sxb(src_val[7:0])) :
				     (op_size == `AP040_SZ_W) ? $signed(sxw(src_val[15:0])) :
				     $signed(src_val);
				ub = (op_size == `AP040_SZ_B) ? $signed(sxb(m_val[7:0])) :
				     (op_size == `AP040_SZ_W) ? $signed(sxw(m_val[15:0])) :
				     $signed(m_val);
				oob = (lb <= ub) ? (rn < lb || rn > ub) : (rn < lb && rn > ub);
				sr[2] <= (rn == lb) || (rn == ub);
				sr[0] <= oob;
				if (x_ext[11] && oob)
					exc(`AP040_VEC_CHK, 4'd2, pc, pc_i);
				else fetch_next;
			end

			//------------------------------------------------- BTST Dn,#imm
			S_BTSTI: begin
				x_ext <= imm;
				rr_a <= p_sreg;
				state <= S_BTSTI2;
			end

			S_BTSTI2: begin
				// The immediate destination is byte-sized, so the dynamic
				// bit number is modulo 8.  Explicitly widen the array index.
				sr[2] <= ~x_ext[{2'b00, rf_rdata_a[2:0]}];
				fetch_next;
			end

			//------------------------------------------------------------ CAS2
			// x_ext[31:16] = first, x_ext[15:0] = second extension word;
			// not bus locked (single CPU master on this fabric)
			S_CAS2_0: begin
				x_ext <= imm;
				rr_a <= {imm[31], imm[30:28]};   // Rn1 (address)
				rr_b <= {imm[15], imm[14:12]};   // Rn2
				state <= S_CAS2_1;
			end

			S_CAS2_1: begin
				t_a <= rf_rdata_a;
				t_b <= rf_rdata_b;
				mrd(rf_rdata_a, op_size, S_CAS2_2);
			end

			S_CAS2_2: begin
				bf_w1 <= m_val;                  // first memory operand
				mrd(t_b, op_size, S_CAS2_3);
			end

			S_CAS2_3: begin
				bf_field <= m_val;               // second memory operand
				rr_a <= {1'b0, x_ext[18:16]};    // Dc1
				rr_b <= {1'b0, x_ext[2:0]};      // Dc2
				state <= S_CAS2_4;
			end

			S_CAS2_4: begin
				cas_dc <= rf_rdata_a;
				bf_du <= rf_rdata_b;
				src_val <= rf_rdata_a;           // ALU: mem1 - Dc1
				dst_val <= bf_w1;
				state <= S_CAS2_5;
			end

			S_CAS2_5: begin
				sr[4:0] <= alu_fl;
				if (alu_fl[2]) begin
					src_val <= bf_du;            // ALU: mem2 - Dc2
					dst_val <= bf_field;
					state <= S_CAS2_6;
				end
				else state <= S_CAS2_F;
			end

			S_CAS2_6: begin
				sr[4:0] <= alu_fl;
				if (alu_fl[2]) begin
					rr_a <= {1'b0, x_ext[24:22]};   // Du1
					rr_b <= {1'b0, x_ext[8:6]};     // Du2
					state <= S_CAS2_W2;
				end
				else state <= S_CAS2_F;
			end

			S_CAS2_W2: mwr(t_a, op_size, rf_rdata_a, S_CAS2_W3);

			S_CAS2_W3: mwr(t_b, op_size, rf_rdata_b, S_NEXT);

			S_CAS2_F: begin
				rfw({1'b0, x_ext[18:16]}, merge_sz(cas_dc, bf_w1, op_size));
				state <= S_CAS2_F2;
			end

			S_CAS2_F2: begin
				rfw({1'b0, x_ext[2:0]}, merge_sz(bf_du, bf_field, op_size));
				fetch_next;
			end

			//------------------------------------------- FSAVE / FRESTORE
			// NULL frame ($00000000) when the FPU is untouched, 4-byte IDLE
			// frame ($41000000) once it has state, or the 52-byte revision-$41
			// UNIMP frame retained by the FPU.  The generic EA engine has
			// already adjusted -(An) by one longword; extend that adjustment
			// to the complete exception frame before issuing any writes.
			S_FSAVE1: begin
				// wait for a background op -- and for the one-cycle frame
				// preparation that follows its deferred-exception retire
				// (fpu_pendcap), or the pend would be judged frameless
				if (fpu_bg || fpu_pendcap) state <= S_FSAVE1;
				else if (fpu_pend_exc && !fpu_fstate_unimp) begin
					// Frameless fallback: a pend whose frame state is gone
					// because an earlier FSAVE already extracted it
					// (fsave_ack) or an FRESTORE of IDLE replaced it.
					// Capture arms fstate_unimp for BOTH classes -- e1 to
					// the $30 frame, e3 to the $41/$60 BUSY frame -- so a
					// pend that still owns its frame takes one of the two
					// branches below and is EXTRACTED, as on the real 040.
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else if (fpu_fstate_unimp && fpu_fstate_busy) begin
					// an e3 arithmetic pend extracts as the 100-byte
					// $41/$60 BUSY frame
					fpu_pend_exc <= 0;
					t_a <= (ea_mode == 3'b100) ? ea_addr - 32'd96 : ea_addr;
					if (ea_mode == 3'b100)
						rfw({1'b1, ea_rn}, ea_addr - 32'd96);
					fpb_n <= 0;
					state <= S_FSAVE_B;
				end
				else if (fpu_fstate_unimp) begin
					// pending state (unimplemented instruction, or a
					// prepared/lingering arithmetic e1 frame) is extracted
					// into the $30 frame; extraction consumes the pend
					fpu_pend_exc <= 0;
					t_a <= (ea_mode == 3'b100) ? ea_addr - 32'd48 : ea_addr;
					if (ea_mode == 3'b100)
						rfw({1'b1, ea_rn}, ea_addr - 32'd48);
					fp_n <= 0;
					state <= S_FSAVE_U;
				end
				else mwr(ea_addr, `AP040_SZ_L,
				             fpu_used ? 32'h4100_0000 : 32'h0000_0000, S_NEXT);
			end

			S_FSAVE_U:
				mwr(t_a + {26'd0, fp_n, 2'b00}, `AP040_SZ_L,
				    fsave_unimp_word(fp_n), S_FSAVE_UD);

			S_FSAVE_UD: begin
				if (fp_n == 4'd12) begin
					// Do not acknowledge/lose the pending state until the final
					// bus write has completed successfully.
					fpu_fsave_ack <= 1;
					fetch_next;
				end
				else begin
					fp_n <= fp_n + 4'd1;
					state <= S_FSAVE_U;
				end
			end

			S_FSAVE_B:
				mwr(t_a + {25'd0, fpb_n, 2'b00}, `AP040_SZ_L,
				    fsave_busy_word(fpb_n), S_FSAVE_BD);

			S_FSAVE_BD: begin
				if (fpb_n == 5'd24) begin
					fpu_fsave_ack <= 1;
					fetch_next;
				end
				else begin
					fpb_n <= fpb_n + 5'd1;
					state <= S_FSAVE_B;
				end
			end

			S_FREST1:
				if (fpu_bg) state <= S_FREST1;       // wait for background op
				else mrd(ea_addr, `AP040_SZ_L, S_FREST2);

			S_FREST2: begin
				// version byte 0 = NULL frame: reset the FPU state.
				// $41/$00 is the MC68040 IDLE frame.  $41/$30 is the 52-byte
				// unimplemented-instruction frame; read its complete payload so
				// bus faults remain precise before installing any FPU state.
				// A completed FRESTORE replaces the FPU state wholesale, so
				// a pending deferred exception from the OLD context is
				// discarded with it (on silicon the pending state lives
				// inside the FPU; FRESTORE neither reports it -- only FSAVE
				// is exempt from reporting, and WinUAE's fpuop_restore
				// never calls fp_exception_pending -- nor may it leak into
				// the new context, whose FPIAR is already reset).
				if (m_val[31:24] == 8'd0) begin
					fpu_rst <= 1;
					fpu_pend_exc <= 0;
					fetch_next;
				end
				else if (m_val == 32'h4100_0000) begin
					fpu_frestore_idle <= 1;
					fpu_pend_exc <= 0;
					fetch_next;
				end
				else if (m_val == 32'h4130_0000) begin
					fp_restore_busy <= 0;
					fp_n <= 4'd1;
					mrd(ea_addr + 32'd4, `AP040_SZ_L, S_FREST_U);
				end
				else if (m_val == 32'h4160_0000) begin
					fp_restore_busy <= 1;
					fpb_n <= 5'd1;
					mrd(ea_addr + 32'd4, `AP040_SZ_L, S_FREST_B);
				end
				else exc(`AP040_VEC_FMTERR, 4'd0, pc_i, 32'd0);
			end

			S_FREST_U: begin
				case (fp_n)
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
					default: ; // reserved longword at offset $08
				endcase
				if (fp_n == 4'd12) state <= S_FREST_UD;
				else begin
					fp_n <= fp_n + 4'd1;
					mrd(ea_addr + ({28'd0, fp_n} << 2) + 32'd4,
					    `AP040_SZ_L, S_FREST_U);
				end
			end

			S_FREST_B: begin
				case (fpb_n)
					5'd6:  fp_restore_wbt[95:64] <= m_val;
					5'd7:  fp_restore_wbt[63:32] <= m_val;
					5'd8:  fp_restore_wbt[31:0]  <= m_val;
					5'd10: fp_restore_fpiar <= m_val;
					5'd13: fp_restore_cmd3 <= m_val[31:16];
					5'd15: begin
						fp_restore_stag <= m_val[31:29];
						fp_restore_grs  <= m_val[25:23];
					end
					5'd16: fp_restore_cmd1 <= m_val[31:16];
					5'd17: begin
						fp_restore_dtag   <= m_val[31:29];
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
					mrd(ea_addr + ({27'd0, fpb_n} << 2) + 32'd4,
					    `AP040_SZ_L, S_FREST_B);
				end
			end

			S_FREST_BD: begin
				if (ea_mode == 3'b011)
					rfw({1'b1, ea_rn}, ea_addr + 32'd100);
				fpu_frestore_unimp <= 1;
				fpu_pend_exc <= fpu_frestore_e1_pend;
				fpu_pend_vec <= fpu_cur_vec;
				fetch_next;
			end

			S_FREST_UD: begin
				fp_restore_busy <= 0;
				if (ea_mode == 3'b011)
					rfw({1'b1, ea_rn}, ea_addr + 32'd52);
				fpu_frestore_unimp <= 1;
				// see S_FREST2: a completed FRESTORE discards the old
				// context's pending deferred exception -- and a restored
				// arithmetic E1 frame re-arms one for the NEW context,
				// vectored by the (already restored) FPSR/FPCR enables,
				// delivered pre-instruction at the next FPU dispatch
				fpu_pend_exc <= fpu_frestore_e1_pend;
				fpu_pend_vec <= fpu_cur_vec;
				fetch_next;
			end

			//------------------------------------------------------------- FPU
			S_FPU_DEC: begin
				if (fpu_bg) begin
					// hold the dispatch until the background FPU
					// operation has retired
				end
				else if (fpu_pend_exc) begin
					// pre-instruction delivery of the pending enabled
					// arithmetic exception, FPSP style: the stacked PC is
					// the FPU instruction being dispatched, FPIAR still
					// identifies the faulting one
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else begin
				fpu_class <= imm[15:13];
				fpu_opm   <= imm[6:0];
				fpu_fmt   <= imm[12:10];
				fpu_srcr  <= imm[12:10];
				fpu_dstr  <= imm[9:7];
				fp_nb     <= fp_bytes(imm[12:10]);
				fp_st     <= 0;
				fp_st_epend <= 0;
				fp_n      <= 0;
				fp_ea_pd  <= 0;
				fp_ea_v   <= 0;
				fp_ea_pi  <= 0;
				fp_force_unsupp <= 0;
				case (imm[15:13])
					3'b000: begin
						// FPm to FPn general.  Nonexisting opmodes fault
						// before any side effect: no FPIAR update, no FPSR
						// status clear, plain F-line (or, for $78-$7F, the
						// integer illegal vector).
						if (fp_opmode_class(imm[6:0]) == 2'd1) go_fp_fline;
						else if (fp_opmode_class(imm[6:0]) == 2'd2) go_illegal;
						else begin
							fpu_iawe <= 1;
							fpu_req <= 1;
							state <= S_FPU_GO;
						end
					end
					3'b001: go_fp_fline;   // undefined opclass
					3'b010: begin
						// <ea>{fmt} to FPn general.  The opmode check does
						// not apply to FMOVECR, whose low bits are a ROM
						// offset rather than an opmode.
						if (imm[12:10] != 3'd7 &&
						    fp_opmode_class(imm[6:0]) == 2'd1) go_fp_fline;
						else if (imm[12:10] != 3'd7 &&
						         fp_opmode_class(imm[6:0]) == 2'd2) go_illegal;
						else if (imm[12:10] == 3'd7) begin
							// FMOVECR: no EA; not hardware on the 040
							fpu_iawe <= 1;
							fpu_req <= 1;
							state <= S_FPU_GO;
						end
						else if (d_mode == 3'b000) begin
							// A data-register EA only supplies a 32-bit value.  D/X/P
							// source formats are malformed FPU commands and take the
							// normal F-line vector, not the integer illegal-op vector.
							if (fp_bytes(imm[12:10]) > 4'd4) begin
								// A recognized FP operation with an unimplemented Dn
								// source format still records its instruction address
								// before taking vector 11.  Allow that side-port write
								// to commit before exception entry snapshots FPIAR.
								// ...except a PACKED source, which WinUAE
								// rejects outright for a Dn EA (get_fp_value
								// case 0 size 3 returns 0 on the 040 without
								// consulting the opmode at all).
								fpu_iawe <= 1;
								go_fp_ea_fault(fp_op_in_hw(imm[6:0]) ||
								               imm[12:10] == 3'd3);
								state <= S_POST_EXC;
							end
							else begin
								rr_a <= {1'b0, d_rn};
								fpu_iawe <= 1;
								state <= S_FPU_DREG;
							end
						end
						else if (d_mode == 3'b001) begin
							// Arithmetic opmode validation precedes source-EA
							// validation on the 040.  An is not a legal source, but
							// the recognized command has already updated FPIAR --
							// and an FPSP-emulated opmode reports through the
							// unimplemented-instruction route even here.
							fpu_iawe <= 1;
							go_fp_ea_fault(fp_op_in_hw(imm[6:0]));
							state <= S_POST_EXC;
						end
						else if (ea_is_imm) begin
							fpb <= 0;
							state <= S_FPU_IMM;
						end
						else if (d_mode == 3'b011 || d_mode == 3'b100) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
					3'b011: begin
						// FMOVE FPn,<ea>{fmt}; FPIAR is updated on the paths
						// that actually engage the FPU, not on the malformed
						// encodings that F-line out
						fpu_srcr <= imm[9:7];
						fp_st <= 1;
						// Packed output is an unsupported data type.  For a memory
						// destination this is a post-instruction exception whose
						// format-$3 frame must contain the calculated EA, so do not
						// trap until normal EA resolution has completed.
						if (imm[12:10] == 3'd3 || imm[12:10] == 3'd7)
							fp_force_unsupp <= 1;
						if (d_mode == 3'b000) begin
							// Packed output is a datatype fault even when the
							// nominal destination is Dn.  Datatype classification
							// wins over the unimplemented-EA check and, with no
							// addressable destination, the format-$3 EA is zero.
							if (imm[12:10] == 3'd3 || imm[12:10] == 3'd7) begin
								fpu_iawe <= 1;
								go_fp_unsupp(1'b1, 1'b1, 1'b0, 32'd0);
								state <= S_POST_EXC;
							end
							// A data register cannot hold a double or
							// extended result: the 68040 reports these as
							// unimplemented FP instructions, not as integer
							// illegal instructions (WinUAE put_fp_value:
							// "68040+ generates unimplemented effective mode
							// exception even if destination EA is Dn or An")
							else if (fp_bytes(imm[12:10]) > 4'd4) begin
								// rejected store destination: F-line, and as
								// with the An/PC-relative cases above the 040
								// records no FPIAR for it
								go_fp_fline;
								state <= S_POST_EXC;
							end
							else begin
								rr_a <= {1'b0, d_rn};
								fpu_iawe <= 1;
								fpu_req <= 1;
								state <= S_FPU_GO;
							end
						end
						else if (d_mode == 3'b001) begin
							// An is not a legal FMOVE-out destination.  FPIAR is
							// NOT written: for opclass 011 the 040 only records
							// it once the store's EA has been accepted (WinUAE
							// fpuop_arithmetic case 3 reaches maybe_set_fpiar
							// only after put_fp_value succeeds; an An
							// destination returns through fpu_noinst first).
							go_fp_fline;
							state <= S_POST_EXC;
						end
						else if (dst_not_alt ||
						         (d_mode == 3'b111 && d_rn[1])) begin
							// Non-alterable or PC-relative destination: F-line
							// with NO FPIAR side effect.  Same rule as the An
							// destination above -- opclass 011 records FPIAR
							// only once put_fp_value has accepted the store.
							go_fp_fline;
							state <= S_POST_EXC;
						end
						else if (d_mode == 3'b011 || d_mode == 3'b100) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
					3'b100, 3'b101: begin : fp_crm
						// FMOVEM control registers.  An empty selection means
						// FPIAR (WinUAE: "All control register bits unset =
						// FPIAR"), and every malformed combination is an
						// F-line trap, not an integer illegal instruction.
						reg [6:0] cnt;
						reg [2:0] crsel;
						reg       multi;
						crsel = (imm[12:10] == 3'd0) ? 3'b001 : imm[12:10];
						multi = (crsel != 3'b100) && (crsel != 3'b010) &&
						        (crsel != 3'b001);
						cnt = ({6'd0, crsel[2]} + {6'd0, crsel[1]} +
						       {6'd0, crsel[0]}) << 2;
						fp_creg <= crsel;
						fp_st <= imm[13];
						if (imm[13]) t0_force <= 1; // FMOVEM control regs to memory
						fp_nb <= cnt[3:0];
						fp_adj <= cnt;
						if (d_mode == 3'b000) begin
							// Dn: a single register only
							if (multi) go_fp_fline;
							else begin
								rr_a <= {1'b0, d_rn};
								state <= S_FPU_CRD;
							end
						end
						else if (d_mode == 3'b001) begin
							// An: only FPIAR may be transferred
							if (crsel != 3'b001) go_fp_fline;
							else begin
								rr_a <= {1'b1, d_rn};
								state <= S_FPU_CRD;
							end
						end
						else if (ea_is_imm) begin
							// an immediate source may load several registers
							// back to back; an immediate destination is a
							// malformed encoding
							if (imm[13]) go_fp_fline;
							else immf(2'd2, S_FPU_CRI);
						end
						else if (imm[13] && d_mode == 3'b111 && d_rn[1])
							go_fp_fline;   // PC-relative destination
						else if (d_mode == 3'b011 || d_mode == 3'b100) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
					default: begin : fp_mvm
						// FMOVEM FP register list, 12 bytes per register.
						// 68040 EA legality: stores reject (An)+ and the
						// PC-relative modes, loads reject -(An); Dn, An and
						// immediate F-line in both directions.
						// 68040 ordering quirks (WinUAE fmovem2mem):
						//   loads always map mask bit 7 to FP0;
						//   a store whose mask convention disagrees with its
						//   EA direction writes each register's three longs
						//   in REVERSED order (low mantissa first);
						//   predec stores consume the mask LSB first so the
						//   ascending walk reproduces the descending layout.
						reg [6:0] cnt;
						reg is_st;
						cnt = ({6'd0, imm[7]} + {6'd0, imm[6]} + {6'd0, imm[5]} +
						       {6'd0, imm[4]} + {6'd0, imm[3]} + {6'd0, imm[2]} +
						       {6'd0, imm[1]} + {6'd0, imm[0]}) * 7'd12;
						is_st = (imm[15:13] == 3'b111);
						fp_mode <= imm[12:11];
						fp_st <= is_st;
						if (is_st) t0_force <= 1;
						fp_list <= imm[7:0];
						fp_adj <= cnt;
						fp_lsb <= is_st && (d_mode == 3'b100);
						fp_rev <= is_st && (imm[12] == (d_mode == 3'b100));
						if (d_mode < 3'b010 || ea_is_imm) go_fp_fline;
						else if (is_st && (d_mode == 3'b011 ||
						         (d_mode == 3'b111 && d_rn[1]))) go_fp_fline;
						else if (!is_st && d_mode == 3'b100) go_fp_fline;
						else if (imm[11]) begin
							// dynamic list in a data register
							rr_a <= {1'b0, imm[6:4]};
							state <= S_FPU_MVML;
						end
						else if (d_mode == 3'b011 || d_mode == 3'b100) begin
							rr_a <= {1'b1, d_rn};
							state <= S_FPU_AN;
						end
						else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
					end
				endcase
				end
			end

			S_FPU_MVML: begin : fp_mvml
				// latch the dynamic FMOVEM list, then resolve the EA
				reg [6:0] cnt;
				cnt = ({6'd0, rf_rdata_a[7]} + {6'd0, rf_rdata_a[6]} +
				       {6'd0, rf_rdata_a[5]} + {6'd0, rf_rdata_a[4]} +
				       {6'd0, rf_rdata_a[3]} + {6'd0, rf_rdata_a[2]} +
				       {6'd0, rf_rdata_a[1]} + {6'd0, rf_rdata_a[0]}) * 7'd12;
				fp_list <= rf_rdata_a[7:0];
				fp_adj <= cnt;
				if (d_mode == 3'b011 || d_mode == 3'b100) begin
					rr_a <= {1'b1, d_rn};
					state <= S_FPU_AN;
				end
				else if (d_mode < 3'b010 || ea_is_imm) go_fp_fline;
				else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FPU_EA);
			end

			S_FPU_AN: begin : fp_an
				// (An)+ / -(An): manual base handling, register written
				// back only at successful completion (restart safe)
				reg [6:0] adj;
				adj = (fp_nb == 4'd1 && d_rn == 3'd7) ? 7'd2 : {3'b000, fp_nb};
				if (fpu_class[2] == 1'b0 && fpu_class != 3'b010 &&
				    fpu_class != 3'b011) adj = fp_adj;   // never taken; clarity
				if (fpu_class == 3'b100 || fpu_class == 3'b101 ||
				    fpu_class == 3'b110 || fpu_class == 3'b111)
					adj = fp_adj;
				fp_adj <= adj;
				fp_ea_v <= 1;
				fp_ea_pd <= (d_mode == 3'b100);
				fp_ea_pi <= (d_mode == 3'b011);
				t_a <= (d_mode == 3'b100) ? (rf_rdata_a - {25'd0, adj})
				                          : rf_rdata_a;
				case (fpu_class)
					3'b010: state <= S_FPU_RD;
					3'b011: begin
						fpu_iawe <= 1;
						if (fp_force_unsupp) begin
							// post-instruction: the address register update
							// stands here too (see S_FPU_GO)
							if (d_mode == 3'b100)
								rfw({1'b1, d_rn},
								    rf_rdata_a - {25'd0, adj});
							else
								rfw({1'b1, d_rn},
								    rf_rdata_a + {25'd0, adj});
							go_fp_unsupp(1'b1, 1'b1, 1'b1,
							    (d_mode == 3'b100) ?
								    (rf_rdata_a - {25'd0, adj}) : rf_rdata_a);
							state <= S_POST_EXC;
						end
						else begin fpu_req <= 1; state <= S_FPU_GO; end
					end
					3'b100, 3'b101: state <= S_FPU_CR;
					default: state <= S_FPU_MVM;
				endcase
			end

			S_FPU_EA: begin
				t_a <= ea_addr;
				fp_ea_v <= 1;
				case (fpu_class)
					3'b010: state <= S_FPU_RD;
					3'b011: begin
						fpu_iawe <= 1;
						if (fp_force_unsupp) begin
							go_fp_unsupp(1'b1, 1'b1, 1'b1, ea_addr);
							state <= S_POST_EXC;
						end
						else begin fpu_req <= 1; state <= S_FPU_GO; end
					end
					3'b100, 3'b101: state <= S_FPU_CR;
					default: state <= S_FPU_MVM;
				endcase
			end

			S_FPU_DREG: begin
				// data register source, left aligned by format
				case (fpu_fmt)
					3'd4: fpb <= {rf_rdata_a[15:0], 80'd0};
					3'd6: fpb <= {rf_rdata_a[7:0], 88'd0};
					default: fpb <= {rf_rdata_a, 64'd0};
				endcase
				fpu_iawe <= 1;
				fpu_req <= 1;
				state <= S_FPU_GO;
			end

			S_FPU_IMM: begin
				// immediate operand: words arrive via the imm register
				if (fp_n != 4'd0) begin
					if (fp_nb == 4'd2) fpb[95:80] <= imm[15:0];
					else if (fp_nb == 4'd1) fpb[95:88] <= imm[7:0];
					else case (fp_n)
						4'd1: fpb[95:64] <= imm[31:0];
						4'd2: fpb[63:32] <= imm[31:0];
						default: fpb[31:0] <= imm[31:0];
					endcase
				end
				if ((fp_nb <= 4'd2 && fp_n != 4'd0) ||
				    (fp_nb == 4'd4 && fp_n == 4'd1) ||
				    (fp_nb == 4'd8 && fp_n == 4'd2) ||
				    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
					fpu_iawe <= 1;
					fpu_req <= 1;
					state <= S_FPU_GO;
				end
				else begin
					fp_n <= fp_n + 4'd1;
					immf((fp_nb <= 4'd2) ? 2'd1 : 2'd2, S_FPU_IMM);
				end
			end

			S_FPU_RD: begin
				// memory operand read loop
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
					fpu_iawe <= 1;
					fpu_req <= 1;
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
				// Both FPSP routes -- unimplemented INSTRUCTION and
				// unsupported DATA TYPE -- report after the operand has
				// been fetched, so the (An)+ / -(An) update stands.
				// WinUAE applies it in get_fp_value when the EA is
				// computed, ahead of either check, and only the 68060
				// takes it back (mmufixup).  Withholding it left the
				// handler pointed at an operand already consumed.
				if (fpu_unimp) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					go_fp_unimp;
				end
				else if (fpu_unsupp) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					go_fp_unsupp(fp_st, 1'b0, fp_ea_v,
					               fp_ea_v ? t_a : 32'd0);
				end
				else if (fpu_exc_req && !fp_st) begin
					fpu_req <= 0;
					exc(fpu_exc_vec, 4'd0, pc, pc_i);
				end
				else if (fpu_accepted && !fp_st) begin
					// register-destination arithmetic past every datatype
					// check: release it to the background and continue
					// integer execution.  Post-increment/-decrement address
					// register updates do not depend on the result.
					fpu_bg <= 1;
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					fetch_next;
				end
				else if (fpu_done) begin
					if (!fp_st) begin
						if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
						else if (fp_ea_pi)
							rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
						fetch_next;
					end
					else if (fpu_exc_req &&
					         (fpu_exc_vec == `AP040_VEC_FP_SNAN ||
					          fpu_exc_vec == `AP040_VEC_FP_OPERR) &&
					         (fpu_fmt == 3'd0 || fpu_fmt == 3'd4 ||
					          fpu_fmt == 3'd6)) begin
						// Enabled integer-store SNAN/OPERR: the 040 does not
						// write the destination (WinUAE
						// fault_if_68040_integer_nonmaskable returns before
						// the store).  Post-instruction format $3, EA = the
						// operand address, 0 for a Dn destination.  The
						// (An)+/-(An) update still commits: the instruction
						// completed, only the store is suppressed.
						fpu_req <= 0;
						if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
						else if (fp_ea_pi)
							rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
						exc(fpu_exc_vec, 4'd3, pc,
						    (d_mode == 3'b000) ? 32'd0 : t_a);
					end
					else if (d_mode == 3'b000) begin : fp_stdn
						// store to a data register with size merge
						case (fpu_fmt)
							3'd4: rfw({1'b0, d_rn},
							          {rf_rdata_a[31:16], fpu_dout[95:80]});
							3'd6: rfw({1'b0, d_rn},
							          {rf_rdata_a[31:8], fpu_dout[95:88]});
							default: rfw({1'b0, d_rn}, fpu_dout[95:64]);
						endcase
						// enabled float-format exception on a Dn store: the
						// destination IS written (WinUAE put_fp_value, then
						// fpsr_check_arithmetic_exception), then the trap is
						// post-instruction with EA = 0
						if (fpu_exc_req) begin
							fpu_req <= 0;
							exc(fpu_exc_vec, 4'd3, pc, 32'd0);
						end
						else fetch_next;
					end
					else begin
						fp_n <= 0;
						// float-format memory store with an enabled
						// exception: write memory first, deliver after the
						// last write (WinUAE order: put_fp_value completes,
						// then fp_exception_pending(false))
						fp_st_epend <= fpu_exc_req;
						fp_st_evec  <= fpu_exc_vec;
						state <= S_FPU_WR;
					end
				end
			end

			S_FPU_WR: begin
				// memory store loop from the FPU result
				if ((fp_nb <= 4'd4 && fp_n != 4'd0) ||
				    (fp_nb == 4'd8 && fp_n == 4'd2) ||
				    (fp_nb == 4'd12 && fp_n == 4'd3)) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a);
					else if (fp_ea_pi)
						rfw({1'b1, d_rn}, t_a + {25'd0, fp_adj});
					if (fp_st_epend) begin
						// post-instruction delivery of the enabled store
						// exception, after the destination was written
						fp_st_epend <= 0;
						exc(fp_st_evec, 4'd3, pc, t_a);
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
				// single control register, data or address register operand
				fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
				if (!fp_st) begin
					fpu_crwe <= 1;
					fpu_crwd <= rf_rdata_a;
					fetch_next;
				end
				else state <= S_FPU_CR2;
			end

			S_FPU_CRI: begin : fp_cri
				// FMOVEM.L #imm,<control list>: one longword per selected
				// register, consumed in FPCR, FPSR, FPIAR order
				reg [2:0] rest;
				rest = fp_creg[2] ? {1'b0, fp_creg[1:0]} :
				       fp_creg[1] ? {fp_creg[2], 1'b0, fp_creg[0]} :
				                    3'b000;
				fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
				fpu_crwe <= 1;
				fpu_crwd <= imm;
				fp_creg <= rest;
				if (rest != 3'd0) immf(2'd2, S_FPU_CRI);
				else fetch_next;
			end

			S_FPU_CR2: begin
				// Control-register read is valid one cycle after crsel.  A
				// register-direct FMOVE (Dn, or An for FPIAR) completes
				// here; an FMOVEM list emits the selected long and returns
				// to the list sequencer.
				if (d_mode == 3'b000 || d_mode == 3'b001) begin
					rfw({d_mode[0], d_rn}, fpu_crrd);
					fetch_next;
				end
				else mwr(t_a, `AP040_SZ_L, fpu_crrd, S_FPU_CR);
			end

			S_FPU_CR: begin : fp_cr
				// control register list transfer, FPCR/FPSR/FPIAR order
				if (fp_n[0]) begin
					// completion of the previous long
					if (!fp_st) begin
						fpu_crwe <= 1;
						fpu_crwd <= m_val;
					end
					t_a <= t_a + 32'd4;
					fp_n <= 0;
				end
				else if (fp_creg == 3'd0) begin
					if (fp_ea_pd) rfw({1'b1, d_rn}, t_a - {25'd0, fp_adj});
					else if (fp_ea_pi) rfw({1'b1, d_rn}, t_a);
					fetch_next;
				end
				else begin
					fpu_crsel <= fp_creg[2] ? 2'd2 : (fp_creg[1] ? 2'd1 : 2'd0);
					fp_creg <= fp_creg[2] ? {1'b0, fp_creg[1:0]} :
					           fp_creg[1] ? {fp_creg[2], 1'b0, fp_creg[0]} :
					                        {fp_creg[2:1], 1'b0};
					fp_n <= 4'd1;
					if (fp_st) state <= S_FPU_CR2;   // wait for crrd
					else mrd(t_a, `AP040_SZ_L, S_FPU_CR);
				end
			end

			S_FPU_MVM: begin : fp_mvm_sel
				// FMOVEM register loop.  Loads map mask bit 7 to FP0
				// regardless of the mode field (68040); stores follow the
				// mode's convention.  Predec stores walk the mask from the
				// LSB so the ascending address walk reproduces the layout
				// of the hardware's descending one.
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
						if (fp_ea_pd)
							rfw({1'b1, d_rn}, t_a - {25'd0, fp_adj});
						else if (fp_ea_pi) rfw({1'b1, d_rn}, t_a);
					fetch_next;
				end
				else begin
					fp_list <= fp_list & ~(8'd1 << b);
					fpu_fmsel <= (!fp_st || fp_mode[1]) ? (3'd7 - b) : b;
					if (fp_ea_pd) t_a <= t_a;   // base already lowered
					fp_n <= 0;
					state <= S_FPU_MVM2;
				end
			end

			S_FPU_MVM2: begin
				// one register = three longs; fm_rdata valid here
				if (fp_n == 4'd3) begin
					if (!fp_st) begin
						fpu_fmwe <= 1;
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
						mwr(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L, wv,
						    S_FPU_MVM3);
					else
						mrd(t_a + {28'd0, fp_n[1:0], 2'b00}, `AP040_SZ_L,
						    S_FPU_MVM3);
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

			//--------------------------------------- FBcc / FScc / FDBcc
			S_FBCC: begin : fbcc
				if (fpu_bg) state <= S_FBCC;         // wait for background op
				else if (fpu_pend_exc) begin
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else begin : fbcc_run
				// the 6-bit predicate field aliases: WinUAE's fpp_cond masks
				// with 0x1f, so bit 5 has no effect and is NOT a trap
				// (table68k defines FBcc for all 64 encodings)
				reg [31:0] disp;
				disp = ir[6] ? imm : sxw(imm[15:0]);
				if (ir[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1;
					exc(`AP040_VEC_FP_BSUN, 4'd0, pc_i, 32'd0);
				end
				else begin
					if (ir[4] && fpu_cc[0]) fpu_bsun <= 1;
					if (fp_cond(ir[5:0], fpu_cc))
						go_pc(pc_i + 32'd2 + disp);
					else fetch_next;
				end
				end
			end

			S_FSCC0: begin
				if (fpu_bg) state <= S_FSCC0;        // wait for background op
				else if (fpu_pend_exc) begin
					fpu_pend_exc <= 0;
					exc(fpu_pend_vec, 4'd0, pc_i, pc_i);
				end
				else begin : fscc0_run
				fp_pred <= imm[5:0];
				// On the 68040 FDBcc, FScc and FTRAPcc record the command
				// address in FPIAR once their extension word has decoded.  FBcc
				// is the exception: it leaves FPIAR alone on the normal path.
				// Do this before EA processing so the side effect also precedes
				// a later operand/access exception.
				fpu_iawe <= 1;
				if (d_mode == 3'b001) begin
					// FDBcc Dn,disp
					t0_force <= 1;       // every FDBcc is T0-traced on 040
					rr_a <= {1'b0, d_rn};
					immf(2'd1, S_FDBCC);
				end
				else if (d_mode == 3'b111 && d_rn == 3'b010)
					immf(2'd1, S_FSCC1);        // FTRAPcc.W
				else if (d_mode == 3'b111 && d_rn == 3'b011)
					immf(2'd2, S_FSCC1);        // FTRAPcc.L
				else if (d_mode == 3'b111 && d_rn == 3'b100)
					state <= S_FSCC1;           // FTRAPcc
				else if (d_mode == 3'b000) begin
					rr_a <= {1'b0, d_rn};
					state <= S_FSCC1;
				end
				else if (ea_is_imm || (d_mode == 3'b111 && d_rn[1]))
					go_fp_fline;
				else ea_start(d_mode, d_rn, `AP040_SZ_B, S_FSCC1);
				end
			end

			S_FSCC1: begin : fscc1
				reg c;
				c = fp_cond(fp_pred, fpu_cc);
				if (fp_pred[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1;
					exc(`AP040_VEC_FP_BSUN, 4'd0, pc_i, 32'd0);
					// fpu_bsun is a side port sampled on the following edge.
					// Let it commit before exception entry snapshots FPSR.
					state <= S_POST_EXC;
				end
				else begin
					if (fp_pred[4] && fpu_cc[0]) fpu_bsun <= 1;
				if (d_mode == 3'b111 && (d_rn == 3'b010 || d_rn == 3'b011 ||
				                         d_rn == 3'b100)) begin
					// FTRAPcc
					if (c) begin
						exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);
						// A signaling unordered predicate records BSUN even when
						// disabled.  Delay entry so that FPSR write is visible.
						if (fp_pred[4] && fpu_cc[0]) state <= S_POST_EXC;
					end
					else fetch_next;
				end
				else if (d_mode == 3'b000) begin
					rfw({1'b0, d_rn}, {rf_rdata_a[31:8], {8{c}}});
					fetch_next;
				end
				else mwr(ea_addr, `AP040_SZ_B, {24'd0, {8{c}}}, S_NEXT);
				end
			end

			S_FDBCC: begin : fdbcc
				reg [15:0] cnt;
				if (fp_pred[4] && fpu_cc[0] && fpu_bsun_en) begin
					fpu_bsun <= 1;
					exc(`AP040_VEC_FP_BSUN, 4'd0, pc_i, 32'd0);
					state <= S_POST_EXC;
				end
				else begin
					if (fp_pred[4] && fpu_cc[0]) fpu_bsun <= 1;
					if (fp_cond(fp_pred, fpu_cc)) fetch_next;
					else begin
						cnt = rf_rdata_a[15:0] - 16'd1;
						rfw({1'b0, d_rn}, {rf_rdata_a[31:16], cnt});
						if (cnt != 16'hFFFF)
							go_pc(pc_i + 32'd4 + sxw(imm[15:0]));
						else fetch_next;
					end
				end
			end

			S_RESET_HOLD: begin
				epf_flush;
				if (rst_cnt == 8'd0) fetch_next;
				else rst_cnt <= rst_cnt - 8'd1;
			end

			//--------------------------------------------------------- MOVE16
			S_M16_SRC: begin
				if (m16_form == 3'd4) m16_dst_rn <= imm[14:12];
				rr_a <= {1'b1, d_rn};
				state <= S_M16_DST;
			end

			S_M16_DST: begin
				m16_an <= rf_rdata_a;
				case (m16_form)
					3'd0, 3'd2: begin  // (An)[+] to abs
						m16_src <= rf_rdata_a & 32'hFFFF_FFF0;
						m16_dst <= imm & 32'hFFFF_FFF0;
					end
					3'd1, 3'd3: begin  // abs to (An)[+]
						m16_src <= imm & 32'hFFFF_FFF0;
						m16_dst <= rf_rdata_a & 32'hFFFF_FFF0;
					end
					default: begin     // (Ax)+ to (Ay)+
						m16_src <= rf_rdata_a & 32'hFFFF_FFF0;
						rr_b <= {1'b1, m16_dst_rn};
					end
				endcase
				state <= (m16_form == 3'd4) ? S_M16_DST2 : S_M16_RD;
				m16_idx <= 0;
				m16_rd_done <= 0;
			end

			S_M16_DST2: begin
				m16_dst <= rf_rdata_b & 32'hFFFF_FFF0;
				t_b <= rf_rdata_b;
				state <= S_M16_RD;
			end

			S_M16_RD: mrd(m16_src + {28'd0, m16_idx, 2'b00}, `AP040_SZ_L, S_M16_RD2);

			S_M16_RD2: begin
				m16buf[m16_idx] <= m_val;
				if (m16_idx == 2'd3) begin
					m16_idx <= 0;
					state <= S_M16_WR;
				end
				else begin
					m16_idx <= m16_idx + 2'd1;
					state <= S_M16_RD;
				end
			end

			S_M16_WR: mwr(m16_dst + {28'd0, m16_idx, 2'b00}, `AP040_SZ_L,
			              m16buf[m16_idx], S_M16_WR2);

			S_M16_WR2: begin
				if (m16_idx == 2'd3) state <= S_M16_INC;
				else begin
					m16_idx <= m16_idx + 2'd1;
					state <= S_M16_WR;
				end
			end

			S_M16_INC: begin
				case (m16_form)
					3'd0, 3'd1: begin  // (An)+ forms
						rfw({1'b1, d_rn}, m16_an + 32'd16);
						fetch_next;
					end
					3'd4: begin
						rfw({1'b1, d_rn}, m16_an + 32'd16);
						if (m16_dst_rn != d_rn) state <= S_M16_INC2;
						else fetch_next;
					end
					default: fetch_next;
				endcase
			end

			S_M16_INC2: begin
				rfw({1'b1, m16_dst_rn}, t_b + 32'd16);
				fetch_next;
			end

			//------------------------------------------------------ bitfields
			S_BF0: begin
				x_ext <= imm;
				if (imm[11]) rr_a <= {1'b0, imm[8:6]};   // offset from Dn
				if (imm[5])  rr_b <= {1'b0, imm[2:0]};   // width from Dn
				state <= S_BF1;
			end

			S_BF1: begin
				bf_off <= x_ext[11] ? rf_rdata_a : {27'd0, x_ext[10:6]};
				bf_w <= x_ext[5] ? ((rf_rdata_b[4:0] == 5'd0) ? 6'd32 : {1'b0, rf_rdata_b[4:0]})
				                 : ((x_ext[4:0] == 5'd0) ? 6'd32 : {1'b0, x_ext[4:0]});
				if (d_mode == 3'b000) begin
					rr_a <= {1'b0, d_rn};
					state <= S_BF_REG;
				end
				else ea_start(d_mode, d_rn, `AP040_SZ_B, S_BF_MEM0);
			end

			S_BF_REG: begin
				dst_val <= rf_rdata_a;    // register operand
				if (ir[10:8] == 3'd7) begin
					rr_a <= {1'b0, x_ext[14:12]};   // BFINS source
					state <= S_BF_REG2;
				end
				else state <= S_BF_REGX;
			end

			S_BF_REG2: begin
				bf_du <= rf_rdata_a;
				state <= S_BF_REGX;
			end

			S_BF_REGX: begin
				// stage 1: rotate the operand so the field is left aligned
				bf_t40 <= {rotl32(dst_val, bf_off[4:0]), 8'd0};
				state <= S_BF_X2;
			end

			S_BF_X2: begin
				// stage 2: extract the field; precompute width masks
				bf_field <= (bf_w == 6'd32) ? bf_t40[39:8]
				                            : (bf_t40[39:8] >> (6'd32 - bf_w));
				bf_ones <= (bf_w == 6'd32) ? 32'hFFFF_FFFF
				                           : ((32'd1 << bf_w) - 32'd1);
				bf_maskl <= {((bf_w == 6'd32) ? 32'hFFFF_FFFF
				                              : (32'hFFFF_FFFF << (6'd32 - bf_w))), 8'd0};
				state <= S_BF_X3;
			end

			S_BF_X3: begin : bf_x3
				reg [31:0] nf, newr;
				nf = bf_newf(ir[10:8], bf_field, bf_du & bf_ones, bf_ones);
				sr[3] <= (ir[10:8] == 3'd7) ? nf[bf_w - 6'd1] : bf_field[bf_w - 6'd1];
				sr[2] <= (ir[10:8] == 3'd7) ? (nf == 32'd0) : (bf_field == 32'd0);
				sr[1] <= 0;
				sr[0] <= 0;
				case (ir[10:8])
					3'd0: fetch_next;                              // BFTST
					3'd1: begin rfw({1'b0, x_ext[14:12]}, bf_field); fetch_next; end
					3'd3: begin                                    // BFEXTS
						rfw({1'b0, x_ext[14:12]},
						    bf_field | (bf_field[bf_w - 6'd1] ? ~bf_ones : 32'd0));
						fetch_next;
					end
					3'd5: begin : bfffo_x                          // BFFFO
						// left-aligned field = window AND left mask: no shifter
						reg [31:0] al;
						al = bf_t40[39:8] & bf_maskl[39:8];
						rfw({1'b0, x_ext[14:12]},
						    bf_off + {26'd0, (al == 32'd0) ? bf_w : clz32(al)});
						fetch_next;
					end
					default: begin                                 // CHG/CLR/SET/INS
						// stage 3: place the new field, still left aligned
						newr = (bf_t40[39:8] & ~bf_maskl[39:8]) |
						       (((bf_w == 6'd32) ? nf : (nf << (6'd32 - bf_w))) & bf_maskl[39:8]);
						bf_t40[39:8] <= newr;
						state <= S_BF_X4;
					end
				endcase
			end

			S_BF_X4: begin
				// stage 4: rotate back and write the register
				rfw({1'b0, d_rn}, rotr32(bf_t40[39:8], bf_off[4:0]));
				fetch_next;
			end

			S_BF_MEM0: begin
				bf_addr <= ea_addr + {{3{bf_off[31]}}, bf_off[31:3]};
				bf_bib <= bf_off[2:0];
				bf_span <= ({3'd0, bf_off[2:0]} + bf_w + 6'd7) >> 3;
				if (ir[10:8] == 3'd7) rr_b <= {1'b0, x_ext[14:12]};
				mrd(ea_addr + {{3{bf_off[31]}}, bf_off[31:3]}, `AP040_SZ_L, S_BF_MEM1);
			end

			S_BF_MEM1: begin
				bf_w1 <= m_val;
				bf_du <= rf_rdata_b;
				if (bf_span == 3'd5) mrd(bf_addr + 32'd4, `AP040_SZ_B, S_BF_MEM2);
				else begin
					bf_w2 <= 8'd0;
					state <= S_BF_EXECM;
				end
			end

			S_BF_MEM2: begin
				bf_w2 <= m_val[7:0];
				state <= S_BF_EXECM;
			end

			S_BF_EXECM: begin
				// stage 1: left align the window on the field start bit
				bf_t40 <= {bf_w1, bf_w2} << bf_bib;
				state <= S_BF_M2;
			end

			S_BF_M2: begin
				// stage 2: extract the field; width masks in the t40 domain
				bf_field <= (bf_w == 6'd32) ? bf_t40[39:8]
				                            : (bf_t40[39:8] >> (6'd32 - bf_w));
				bf_ones <= (bf_w == 6'd32) ? 32'hFFFF_FFFF
				                           : ((32'd1 << bf_w) - 32'd1);
				bf_maskl <= (bf_w == 6'd32) ? {32'hFFFF_FFFF, 8'd0}
				                            : ({32'hFFFF_FFFF, 8'd0} << (6'd32 - bf_w));
				state <= S_BF_M3;
			end

			S_BF_M3: begin : bf_m3
				reg [31:0] nf;
				nf = bf_newf(ir[10:8], bf_field, bf_du & bf_ones, bf_ones);
				sr[3] <= (ir[10:8] == 3'd7) ? nf[bf_w - 6'd1] : bf_field[bf_w - 6'd1];
				sr[2] <= (ir[10:8] == 3'd7) ? (nf == 32'd0) : (bf_field == 32'd0);
				sr[1] <= 0;
				sr[0] <= 0;
				case (ir[10:8])
					3'd0: fetch_next;
					3'd1: begin rfw({1'b0, x_ext[14:12]}, bf_field); fetch_next; end
					3'd3: begin
						rfw({1'b0, x_ext[14:12]},
						    bf_field | (bf_field[bf_w - 6'd1] ? ~bf_ones : 32'd0));
						fetch_next;
					end
					3'd5: begin : bfffo_m
						reg [31:0] al;
						al = bf_t40[39:8] & bf_maskl[39:8];
						rfw({1'b0, x_ext[14:12]},
						    bf_off + {26'd0, (al == 32'd0) ? bf_w : clz32(al)});
						fetch_next;
					end
					default: begin
						// stage 3: substitute the new field, still left aligned
						bf_t40 <= (bf_t40 & ~bf_maskl) |
						          ((({nf, 8'd0}) << (6'd32 - bf_w)) & bf_maskl);
						state <= S_BF_M4;
					end
				endcase
			end

			S_BF_M4: begin : bf_m4
				// stage 4: shift back into the memory window; the top bf_bib
				// bits of the original window pass through unchanged
				reg [39:0] head, nw40;
				head = ~(40'hFF_FFFF_FFFF >> bf_bib);
				nw40 = ({bf_w1, bf_w2} & head) | (bf_t40 >> bf_bib);
				bf_w1 <= nw40[39:8];
				bf_w2 <= nw40[7:0];
				state <= S_BF_WR1;
			end

			S_BF_WR1: begin
				case (bf_span)
					3'd1: mwr(bf_addr, `AP040_SZ_B, {24'd0, bf_w1[31:24]}, S_NEXT);
					3'd2: mwr(bf_addr, `AP040_SZ_W, {16'd0, bf_w1[31:16]}, S_NEXT);
					3'd3: mwr(bf_addr, `AP040_SZ_W, {16'd0, bf_w1[31:16]}, S_BF_WR2);
					3'd4: mwr(bf_addr, `AP040_SZ_L, bf_w1, S_NEXT);
					default: mwr(bf_addr, `AP040_SZ_L, bf_w1, S_BF_WR2);
				endcase
			end

			S_BF_WR2: begin
				if (bf_span == 3'd3)
					mwr(bf_addr + 32'd2, `AP040_SZ_B, {24'd0, bf_w1[15:8]}, S_NEXT);
				else
					mwr(bf_addr + 32'd4, `AP040_SZ_B, {24'd0, bf_w2}, S_NEXT);
			end

			//------------------------------------------------------------ CAS
			S_CAS1: begin
				x_ext <= imm;
				ea_start(d_mode, d_rn, op_size, S_CAS2);
			end

			S_CAS2: begin
				dst_addr <= ea_addr;
				rr_a <= {1'b0, x_ext[2:0]};   // Dc
				rr_b <= {1'b0, x_ext[8:6]};   // Du
				mrd(ea_addr, op_size, S_CAS3);
			end

			S_CAS3: begin
				src_val <= rf_rdata_a;   // Dc: ALU computes operand - Dc
				dst_val <= m_val;
				cas_dc <= rf_rdata_a;
				bf_du <= rf_rdata_b;
				state <= S_CAS4;
			end

			S_CAS4: begin
				sr[4:0] <= alu_fl;
				if (alu_fl[2])
					mwr(dst_addr, op_size, bf_du, S_NEXT);   // equal: update
				else begin
					rfw({1'b0, x_ext[2:0]}, merge_sz(cas_dc, dst_val, op_size));
					fetch_next;
				end
			end

			//----------------------------------------- immediate to CCR / SR
			S_SROP: begin : srop
				reg [15:0] nv;
				case (srop_kind)
					2'd0: nv = sr | imm[15:0];
					2'd1: nv = sr & imm[15:0];
					default: nv = sr ^ imm[15:0];
				endcase
				if (srop_sr) begin
					// let SR settle before the next dispatch (see DK_SR)
					sr <= nv & `AP040_SR_MASK;
					state <= S_NEXT;
				end
				else begin
					sr[4:0] <= nv[4:0];
					fetch_next;
				end
			end

			//---------------------------------------------------------- decode
			S_DECODE: begin
				case (ir_hi)
					//-------------------------------------------- 0x0: bit/imm
					4'h0: begin
						if (ir[8] && d_mode == 3'b001) begin
							// MOVEP
							mp_dir <= ir[7];
							mp_cnt <= ir[6] ? 3'd4 : 3'd2;
							immf(2'd1, S_MOVEP1);
						end
						else if (ir[8]) begin
							// dynamic bit op, bit number in Dn
							// As for the static forms, an address register is
							// never a bit destination.  BTST may read program
							// space or an immediate operand, but the modifying
							// forms require a data-alterable destination.
							if (d_mode == 3'b001) go_illegal;
							else if (d_mode == 3'b111 &&
							         ((ir[7:6] == 2'b00) ? (d_rn > 3'b100)
							                             : (d_rn > 3'b001)))
								go_illegal;
							else begin
								alu_op <= `AP040_ALU_BTST + {4'd0, ir[7:6]};
								p_src <= SK_REG; p_sreg <= {1'b0, d_reg9};
								if (ir[7:6] == 2'b00) p_wbsup <= 1; // BTST
								if (ea_is_imm)
									immf(2'd1, S_BTSTI);
								else if (d_mode == 3'b000) begin
									op_size <= `AP040_SZ_L;
									p_dsize <= `AP040_SZ_L;
									p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
									pipe_go;
								end
								else begin
									op_size <= `AP040_SZ_B;
									p_dsize <= `AP040_SZ_B;
									p_dst <= DK_MEM;
									p_dst_mem_bit <= 1;
									dst_mode_r <= d_mode; dst_rn_r <= d_rn;
									p_rmw <= 1;
									pipe_go;
								end
							end
						end
						else if (d_reg9 == 3'b100) begin
							// static bit op, bit number in extension word
							// (checked before the size=11 group: BSET is 00xx11)
							// BTST only reads, so it accepts program space and
							// an immediate operand; BCHG/BCLR/BSET write and
							// need a data alterable destination.  An address
							// register is never allowed.
							if (d_mode == 3'b001) go_illegal;
							else if (d_mode == 3'b111 &&
							         ((ir[7:6] == 2'b00) ? (d_rn > 3'b100)
							                             : (d_rn > 3'b001)))
								go_illegal;
							else begin
							alu_op <= `AP040_ALU_BTST + {4'd0, ir[7:6]};
							p_src <= SK_IMM;
							if (d_mode == 3'b000) begin
								op_size <= `AP040_SZ_L;
								p_dsize <= `AP040_SZ_L;
								p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
							end
							else begin
								op_size <= `AP040_SZ_B;
								p_dsize <= `AP040_SZ_B;
								p_dst <= DK_MEM;
								p_dst_mem_bit <= 1;
								dst_mode_r <= d_mode; dst_rn_r <= d_rn;
								p_rmw <= 1;
							end
							if (ir[7:6] == 2'b00) p_wbsup <= 1;
							immf(2'd1, S_PIPE_START);
							end
						end
						else if (d_reg9 == 3'b111 && std_size != 2'b11) begin
							// MOVES (0000 1110 11 is CAS.L, not implemented)
							// Validate the effective-address encoding before the
							// privilege check.  Invalid MOVES encodings take vector
							// 4 even in user mode; only a valid MOVES is privileged.
							if (d_mode < 3'b010 ||
							    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
							else if (!sr_s) go_priv;
							else begin
								op_size <= std_size;
								immf(2'd1, S_MOVES1);
							end
						end
						else if (std_size == 2'b11) begin
							if (!d_reg9[2] && d_reg9[1:0] != 2'b11) begin
								// CHK2/CMP2: bounds pair at a control EA
								if (d_mode < 3'b010 || d_mode == 3'b011 ||
								    d_mode == 3'b100 || ea_is_imm) go_illegal;
								else begin
									op_size <= d_reg9[1] ? `AP040_SZ_L :
									           d_reg9[0] ? `AP040_SZ_W : `AP040_SZ_B;
									immf(2'd1, S_CHK2_A);
								end
							end
							else if (d_reg9[2] && d_reg9[1:0] != 2'b00 && ea_is_imm) begin
								// CAS2.W/.L: two extension words follow
								if (d_reg9[1:0] == 2'b01) go_illegal;   // no CAS2.B
								else begin
									alu_op <= `AP040_ALU_CMP;
									op_size <= (d_reg9[1:0] == 2'b10) ? `AP040_SZ_W : `AP040_SZ_L;
									begin lk_cyc <= 1; immf(2'd2, S_CAS2_0); end
								end
							end
							else if (d_reg9[2] && d_reg9[1:0] != 2'b00) begin
								// CAS (memory only)
								if (d_mode < 3'b010 ||
								    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
								else begin
									alu_op <= `AP040_ALU_CMP;
									op_size <= (d_reg9[1:0] == 2'b01) ? `AP040_SZ_B :
									           (d_reg9[1:0] == 2'b10) ? `AP040_SZ_W : `AP040_SZ_L;
									begin lk_cyc <= 1; immf(2'd1, S_CAS1); end
								end
							end
							else go_illegal;   // CAS2 / CHK2 / CMP2
						end
						else begin
							// ORI/ANDI/SUBI/ADDI/EORI/CMPI
							if (ea_is_imm && (d_reg9 == 3'b000 || d_reg9 == 3'b001 || d_reg9 == 3'b101)) begin
								// to CCR (byte) or SR (word, privileged)
								if (std_size == 2'b01 && !sr_s) go_priv;
								else if (std_size > 2'b01) go_illegal;
								else begin
									srop_kind <= (d_reg9 == 3'b000) ? 2'd0 :
									             (d_reg9 == 3'b001) ? 2'd1 : 2'd2;
									srop_sr <= (std_size == 2'b01);
									immf(2'd1, S_SROP);
								end
							end
							else if (d_mode == 3'b001) go_illegal;
							// The destination must be data alterable, so the
							// PC-relative and immediate encodings of mode 7
							// are illegal.  CMPI is the exception: the 68020
							// and later allow it to read program space.
							else if (d_mode == 3'b111 && d_rn > 3'b001 &&
							         !(d_reg9 == 3'b110 && d_rn < 3'b100))
								go_illegal;
							else begin
								case (d_reg9)
									3'b000: alu_op <= `AP040_ALU_OR;
									3'b001: alu_op <= `AP040_ALU_AND;
									3'b010: alu_op <= `AP040_ALU_SUB;
									3'b011: alu_op <= `AP040_ALU_ADD;
									3'b101: alu_op <= `AP040_ALU_EOR;
									default: alu_op <= `AP040_ALU_CMP;
								endcase
								if (d_reg9 == 3'b110) p_wbsup <= 1; // CMPI
								op_size <= std_size;
								p_ssize <= std_size; p_dsize <= std_size;
								p_src <= SK_IMM;
								if (d_mode == 3'b000) begin
									p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
								end
								else begin
									p_dst <= DK_MEM; p_rmw <= 1;
									dst_mode_r <= d_mode; dst_rn_r <= d_rn;
								end
								immf((std_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START);
							end
						end
					end

					//------------------------------------------- 0x1-0x3: MOVE
					4'h1, 4'h2, 4'h3: begin
						if (move_size == `AP040_SZ_B &&
						    (d_mode == 3'b001 || d_op8_6 == 3'b001)) go_illegal;
						else if (d_op8_6 == 3'b111 && d_reg9 > 3'b001) go_illegal;
						else begin
							alu_op <= `AP040_ALU_MOVE;
							op_size <= move_size;
							p_ssize <= move_size; p_dsize <= move_size;
							// source
							if (d_mode == 3'b000 || d_mode == 3'b001) begin
								p_src <= SK_REG; p_sreg <= {d_mode[0], d_rn};
							end
							else if (ea_is_imm) p_src <= SK_IMM;
							else begin
								p_src <= SK_MEM;
								src_mode_r <= d_mode; src_rn_r <= d_rn;
							end
							// destination
							if (d_op8_6 == 3'b000) begin
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
							end
							else if (d_op8_6 == 3'b001) begin
								// MOVEA: full register, no flags, word sexts
								p_dst <= DK_REG; p_dreg <= {1'b1, d_reg9};
								p_flags <= 0;
								if (move_size == `AP040_SZ_W) p_sextw <= 1;
								op_size <= `AP040_SZ_L;
							end
							else begin
								p_dst <= DK_MEM;
								dst_mode_r <= d_op8_6; dst_rn_r <= d_reg9;
							end
							if (ea_is_imm)
								immf((move_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START);
							else pipe_go;
						end
					end

					//------------------------------------------------ 0x4: misc
					4'h4: begin
						if (ir[11:0] == 12'hAFC) go_illegal;   // ILLEGAL
						else if (d_op8_6 == 3'b111) begin
							if (d_mode == 3'b000) begin
								if (d_reg9 == 3'b100) begin
									// EXTB.L
									alu_op <= `AP040_ALU_EXTB;
									op_size <= `AP040_SZ_L;
									p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
									pipe_go;
								end
								else go_illegal;
							end
							else if (d_mode == 3'b001 || (d_mode == 3'b011) ||
							         (d_mode == 3'b100) || ea_is_imm) go_illegal;
							else ea_start(d_mode, d_rn, `AP040_SZ_L, S_LEA1); // LEA
						end
						else if (d_op8_6 == 3'b110) begin
							// CHK.W
							exec_kind <= EK_CHK;
							op_size <= `AP040_SZ_W;
							p_ssize <= `AP040_SZ_W;
							if (d_mode == 3'b001) go_illegal;
							else begin
								if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; end
								else if (ea_is_imm) p_src <= SK_IMM;
								else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; end
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
								if (ea_is_imm) immf(2'd1, S_PIPE_START);
								else pipe_go;
							end
						end
						else if (d_op8_6 == 3'b100 &&
						         !(ir[11:9] == 3'b100 && d_mode == 3'b001)) begin
							// CHK.L (0100 ddd 100; 0100 100 000 001 rrr is LINK.L)
							exec_kind <= EK_CHK;
							op_size <= `AP040_SZ_L;
							p_ssize <= `AP040_SZ_L;
							if (d_mode == 3'b001) go_illegal;
							else begin
								if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; end
								else if (ea_is_imm) p_src <= SK_IMM;
								else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; end
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
								if (ea_is_imm) immf(2'd2, S_PIPE_START);
								else pipe_go;
							end
						end
						else case (ir[11:9])
							3'b000: begin
								if (d_op8_6 == 3'b011) begin
									// MOVE from SR (privileged on 68010+)
									// EA legality is decoded before privilege.  In user
									// mode MOVE SR,An/PC/#imm is vector 4, not vector 8.
									if (dst_not_alt) go_illegal;
									else if (!sr_s) go_priv;
									else begin
										p_src <= SK_IMPL; src_val <= {16'd0, sr};
										alu_op <= `AP040_ALU_MOVE;
										op_size <= `AP040_SZ_W;
										p_dsize <= `AP040_SZ_W;
										p_flags <= 0;
										if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
										else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
									end
								end
								else if (d_op8_6[2]) go_illegal;
								else begin
									// NEGX
									alu_op <= `AP040_ALU_NEGX;
									op_size <= std_size;
									p_dsize <= std_size;
									p_rmw <= 1;
									if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
									else if (dst_not_alt) go_illegal;
									else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
								end
							end

							3'b001: begin
								if (d_op8_6 == 3'b011) begin
									// MOVE from CCR
									p_src <= SK_IMPL; src_val <= {27'd0, sr[4:0]};
									alu_op <= `AP040_ALU_MOVE;
									op_size <= `AP040_SZ_W;
									p_dsize <= `AP040_SZ_W;
									p_flags <= 0;
										if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
										else if (dst_not_alt) go_illegal;
										else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
								end
								else if (d_op8_6[2]) go_illegal;
								else begin
									// CLR (pure write on 68040)
									alu_op <= `AP040_ALU_CLR;
									op_size <= std_size;
									p_dsize <= std_size;
									if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
									else if (dst_not_alt) go_illegal;
									else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
								end
							end

							3'b010: begin
								if (d_op8_6 == 3'b011) begin
									// MOVE to CCR
									alu_op <= `AP040_ALU_MOVE;
									op_size <= `AP040_SZ_W;
									p_ssize <= `AP040_SZ_W;
									p_flags <= 0;
									p_dst <= DK_CCR;
									if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; pipe_go; end
									else if (src_not_data) go_illegal;
									else if (ea_is_imm) begin p_src <= SK_IMM; immf(2'd1, S_PIPE_START); end
									else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
								end
								else if (d_op8_6[2]) go_illegal;
								else begin
									// NEG
									alu_op <= `AP040_ALU_NEG;
									op_size <= std_size;
									p_dsize <= std_size;
									p_rmw <= 1;
									if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
									else if (dst_not_alt) go_illegal;
									else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
								end
							end

							3'b011: begin
								if (d_op8_6 == 3'b011) begin
									// MOVE to SR (privileged)
									// An is not a legal source; encoding rejection wins
									// over the privilege check just as for MOVE from SR.
									if (src_not_data) go_illegal;
									else if (!sr_s) go_priv;
									else begin
										alu_op <= `AP040_ALU_MOVE;
										op_size <= `AP040_SZ_W;
										p_ssize <= `AP040_SZ_W;
										p_flags <= 0;
										p_dst <= DK_SR;
										if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; pipe_go; end
										else if (ea_is_imm) begin p_src <= SK_IMM; immf(2'd1, S_PIPE_START); end
										else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
									end
								end
								else if (d_op8_6[2]) go_illegal;
								else begin
									// NOT
									alu_op <= `AP040_ALU_NOT;
									op_size <= std_size;
									p_dsize <= std_size;
									p_rmw <= 1;
									if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
									else if (dst_not_alt) go_illegal;
									else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
								end
							end

							3'b100: begin
								if (d_op8_6[2]) go_illegal;
								else case (d_op8_6[1:0])
								2'b00: begin
									if (d_mode == 3'b001) begin
										// LINK.L An,#bd32
										br_long <= 1;
										immf(2'd2, S_LINK1);
									end
									else begin
										// NBCD
										alu_op <= `AP040_ALU_NBCD;
										op_size <= `AP040_SZ_B;
										p_dsize <= `AP040_SZ_B;
										p_rmw <= 1;
										if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
										else if (dst_not_alt) go_illegal;
										else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
									end
								end
								2'b01: begin
									if (d_mode == 3'b000) begin
										// SWAP
										alu_op <= `AP040_ALU_SWAP;
										op_size <= `AP040_SZ_L;
										p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
										pipe_go;
									end
									else if (d_mode == 3'b001) go_illegal; // BKPT
									else if (d_mode == 3'b011 || d_mode == 3'b100 || ea_is_imm) go_illegal;
									else ea_start(d_mode, d_rn, `AP040_SZ_L, S_PEA1); // PEA
								end
								default: begin
									if (d_mode == 3'b000) begin
										// EXT.W / EXT.L
										alu_op <= `AP040_ALU_EXT;
										op_size <= d_op8_6[0] ? `AP040_SZ_L : `AP040_SZ_W;
										p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
										pipe_go;
									end
									else begin
										// MOVEM registers to memory
										mm_dir <= 0;
										mm_size <= d_op8_6[0] ? `AP040_SZ_L : `AP040_SZ_W;
										mm_predec <= (d_mode == 3'b100);
										mm_postinc <= 0;
										if (d_mode == 3'b011 || d_mode < 3'b010 || ea_is_imm ||
										    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
										else immf(2'd1, S_MOVEM_SET);
									end
								end
								endcase
							end

							3'b101: begin
								if (d_op8_6[2]) go_illegal;
								else if (d_op8_6 == 3'b011) begin
									// TAS (not bus locked yet).  The 040
									// reports its operand cycles as a locked
									// RMW: an access error carries SSW LK
									// with RW clear.
									alu_op <= `AP040_ALU_TAS;
									op_size <= `AP040_SZ_B;
									p_dsize <= `AP040_SZ_B;
									p_rmw <= 1;
									if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
									else if (dst_not_alt) go_illegal;
									else begin
										lk_cyc <= 1;
										p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go;
									end
								end
								else begin
									// TST (An/imm/PC modes allowed on 020+)
									alu_op <= `AP040_ALU_TST;
									op_size <= std_size;
									p_ssize <= std_size;
									p_wbsup <= 1;
									if (d_mode == 3'b000 || d_mode == 3'b001) begin
										if (d_mode == 3'b001 && std_size == `AP040_SZ_B) go_illegal;
										else begin
											p_src <= SK_REG; p_sreg <= {d_mode[0], d_rn};
											pipe_go;
										end
									end
									else if (ea_is_imm) begin
										p_src <= SK_IMM;
										immf((std_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START);
									end
									else begin
										p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn;
										pipe_go;
									end
								end
							end

							3'b110: begin
								if (d_op8_6[2]) go_illegal;
								else if (!d_op8_6[1]) begin
									// MULx.L / DIVx.L with extension word
									exec_kind <= EK_MD_L;
									md_isdiv <= d_op8_6[0];
									op_size <= `AP040_SZ_L;
									p_ssize <= `AP040_SZ_L;
									if (d_mode == 3'b001) go_illegal;
									else begin
										if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; end
										else if (ea_is_imm) p_src <= SK_IMM;
										else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; end
										// extension word first, then any immediate
										immf(2'd1, S_MDL_EXT);
									end
								end
								else begin
									// MOVEM memory to registers
									mm_dir <= 1;
									mm_size <= d_op8_6[0] ? `AP040_SZ_L : `AP040_SZ_W;
									mm_predec <= 0;
									mm_postinc <= (d_mode == 3'b011);
									if (d_mode == 3'b100 || d_mode < 3'b010 || ea_is_imm) go_illegal;
									else immf(2'd1, S_MOVEM_SET);
								end
							end

							default: begin // 3'b111
								if (d_op8_6 == 3'b010) begin
									// JSR
									if (d_mode < 3'b010 || d_mode == 3'b011 ||
									    d_mode == 3'b100 || ea_is_imm) go_illegal;
									else ea_start(d_mode, d_rn, `AP040_SZ_L, S_JSR1);
								end
								else if (d_op8_6 == 3'b011) begin
									// JMP
									if (d_mode < 3'b010 || d_mode == 3'b011 ||
									    d_mode == 3'b100 || ea_is_imm) go_illegal;
									else ea_start(d_mode, d_rn, `AP040_SZ_L, S_JMP1);
								end
								else if (d_op8_6 == 3'b001) begin
									casez (ir[5:0])
										6'b00????: exc(`AP040_VEC_TRAP + {4'd0, ir[3:0]}, 4'd0, pc, 32'd0);
										6'b010???: begin br_long <= 0; immf(2'd1, S_LINK1); end // LINK.W
										6'b011???: begin rr_a <= {1'b1, d_rn}; state <= S_UNLK1; end
										6'b100???: begin // MOVE An,USP
											if (!sr_s) go_priv;
											else begin rr_a <= {1'b1, d_rn}; state <= S_USP1; end
										end
										6'b101???: begin // MOVE USP,An
											if (!sr_s) go_priv;
											else begin rfw({1'b1, d_rn}, usp_q); fetch_next; end
										end
										6'b110000: begin // RESET
											if (!sr_s) go_priv;
											else begin rst_cnt <= 8'd127; state <= S_RESET_HOLD; end
										end
										6'b110001: fetch_next;   // NOP
										6'b110010: begin // STOP
											if (!sr_s) go_priv;
											else immf(2'd1, S_STOP_LD);
										end
										6'b110011: begin // RTE
											if (!sr_s) go_priv;
											else begin
												// A bus/access fault while RTE is loading internal state
												// from the old frame is a double bus fault (MC68040 UM
												// 8.2), not a new format-$7 exception.
												in_exc <= 1;
												state <= S_RTE_SR;
											end
										end
										6'b110100: begin ret_kind <= RK_RTD; immf(2'd1, S_RET1); end
										6'b110101: begin ret_kind <= RK_RTS; state <= S_RET1; end
										6'b110110: begin // TRAPV
											if (sr[1]) exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);
											else fetch_next;
										end
										6'b110111: begin
											ret_kind <= RK_RTR;
											state <= S_RET1;
										end
										6'b111010, 6'b111011: begin // MOVEC
											if (!sr_s) go_priv;
											else begin
												mvc_dir <= ir[0];
												immf(2'd1, S_MOVEC1);
											end
										end
										default: go_illegal;
									endcase
								end
								else go_illegal;
							end
						endcase
					end

					//------------------------------ 0x5: ADDQ/SUBQ/Scc/DBcc
					4'h5: begin
						if (ir[7:6] == 2'b11) begin
							if (d_mode == 3'b001) begin
								// DBcc
								br_base <= pc;
								immf(2'd1, S_DBCC1);
							end
							else if (d_mode == 3'b111 && d_rn >= 3'b010 && d_rn <= 3'b100) begin
								// TRAPcc (optional operand words are consumed
								// but otherwise ignored)
								if (d_rn == 3'b010)
									immf(2'd1, cond_true(ir[11:8]) ? S_TRAPCC : S_NEXT);
								else if (d_rn == 3'b011)
									immf(2'd2, cond_true(ir[11:8]) ? S_TRAPCC : S_NEXT);
								else begin
									if (cond_true(ir[11:8]))
										exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);
									else fetch_next;
								end
							end
							else begin
								// Scc
								exec_kind <= EK_SCC;
								op_size <= `AP040_SZ_B;
								p_dsize <= `AP040_SZ_B;
								p_flags <= 0;
								if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
								else if (dst_not_alt) go_illegal;
								else begin p_dst <= DK_MEM; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
							end
						end
						else begin
							// ADDQ/SUBQ
							alu_op <= ir[8] ? `AP040_ALU_SUB : `AP040_ALU_ADD;
							p_src <= SK_IMPL;
							src_val <= {28'd0, (d_reg9 == 3'd0) ? 4'd8 : {1'b0, d_reg9}};
							if (d_mode == 3'b001) begin
								// to An: whole register, no flags, any size but byte
								if (std_size == `AP040_SZ_B) go_illegal;
								else begin
									op_size <= `AP040_SZ_L;
									p_dst <= DK_REG; p_dreg <= {1'b1, d_rn};
									p_flags <= 0;
									pipe_go;
								end
							end
							else begin
								op_size <= std_size;
								p_dsize <= std_size;
								if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
								else if (dst_not_alt) go_illegal;
								else begin p_dst <= DK_MEM; p_rmw <= 1; dst_mode_r <= d_mode; dst_rn_r <= d_rn; pipe_go; end
							end
						end
					end

					//---------------------------------------- 0x6: Bcc/BSR/BRA
					4'h6: begin
						if (ir[7:0] == 8'h00 || ir[7:0] == 8'hFF) begin
							br_base <= pc;
							br_long <= (ir[7:0] == 8'hFF);
							immf((ir[7:0] == 8'hFF) ? 2'd2 : 2'd1, S_BCC_EXT);
						end
						else if (ir[11:8] == 4'h1) begin : bsr_b
							// BSR.B; an odd target faults with A7 untouched
							reg [31:0] bt;
							bt = pc + sxb(ir[7:0]);
							if (bt[0]) go_pc(bt);
							else begin
								br_tgt <= bt;
								mwr(dbg_a7 - 32'd4, `AP040_SZ_L, pc, S_BSR_PUSH);
							end
						end
						else finish_bcc(pc + sxb(ir[7:0]), cond_true(ir[11:8]));
					end

					//------------------------------------------- 0x7: MOVEQ
					4'h7: begin
						if (ir[8]) go_illegal;
						else begin
							rfw({1'b0, d_reg9}, sxb(ir[7:0]));
							sr[3] <= ir[7];
							sr[2] <= (ir[7:0] == 8'd0);
							sr[1] <= 0; sr[0] <= 0;
							fetch_next;
						end
					end

					//------------------------------------- 0x8: OR/DIV/SBCD
					4'h8: begin
						if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111) begin
							// DIVU.W / DIVS.W
							exec_kind <= EK_MD_W;
							md_isdiv <= 1;
							md_sign <= d_op8_6[2];
							op_size <= `AP040_SZ_W;
							p_ssize <= `AP040_SZ_W;
							p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
							if (d_mode == 3'b001) go_illegal;
							else if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; pipe_go; end
							else if (ea_is_imm) begin p_src <= SK_IMM; immf(2'd1, S_PIPE_START); end
							else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
						end
						else if (ir[8] && d_mode[2:1] == 2'b00) begin
							case (d_op8_6[1:0])
								2'b00: begin
									// SBCD
									alu_op <= `AP040_ALU_SBCD;
									op_size <= `AP040_SZ_B;
									p_ssize <= `AP040_SZ_B; p_dsize <= `AP040_SZ_B;
									if (!d_mode[0]) begin
										p_src <= SK_REG; p_sreg <= {1'b0, d_rn};
										p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
									end
									else begin
										p_src <= SK_MEM; src_mode_r <= 3'b100; src_rn_r <= d_rn;
										p_dst <= DK_MEM; dst_mode_r <= 3'b100; dst_rn_r <= d_reg9;
										p_rmw <= 1;
									end
									pipe_go;
								end
								2'b01: begin
									// PACK
									exec_kind <= EK_PACK;
									p_flags <= 0;
									p_ssize <= `AP040_SZ_W; p_dsize <= `AP040_SZ_B;
									if (!d_mode[0]) begin
										p_src <= SK_REG; p_sreg <= {1'b0, d_rn};
										p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
									end
									else begin
										p_src <= SK_MEM; src_mode_r <= 3'b100; src_rn_r <= d_rn;
										p_dst <= DK_MEM; dst_mode_r <= 3'b100; dst_rn_r <= d_reg9;
									end
									immf(2'd1, S_PIPE_START);
								end
								2'b10: begin
									// UNPK
									exec_kind <= EK_UNPK;
									p_flags <= 0;
									p_ssize <= `AP040_SZ_B; p_dsize <= `AP040_SZ_W;
									if (!d_mode[0]) begin
										p_src <= SK_REG; p_sreg <= {1'b0, d_rn};
										p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
									end
									else begin
										p_src <= SK_MEM; src_mode_r <= 3'b100; src_rn_r <= d_rn;
										p_dst <= DK_MEM; dst_mode_r <= 3'b100; dst_rn_r <= d_reg9;
									end
									immf(2'd1, S_PIPE_START);
								end
								default: go_illegal;
							endcase
						end
						else begin
							// OR
							alu_op <= `AP040_ALU_OR;
							op_size <= std_size;
							p_ssize <= std_size; p_dsize <= std_size;
							if (!ir[8]) begin
								// <ea> OR Dn -> Dn
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
								if (d_mode == 3'b001) go_illegal;
								else if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; pipe_go; end
								else if (ea_is_imm) begin p_src <= SK_IMM; immf((std_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START); end
								else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
							end
							else begin
								// Dn OR <ea> -> <ea>
								p_src <= SK_REG; p_sreg <= {1'b0, d_reg9};
								p_dst <= DK_MEM; p_rmw <= 1;
								dst_mode_r <= d_mode; dst_rn_r <= d_rn;
								// The register-to-EA OR form is memory-only.
								// Mode 000/001 combinations are reserved for the
								// SBCD/PACK/UNPK subfamily above.
								if (d_mode < 3'b010 ||
								    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
								else pipe_go;
							end
						end
					end

					//------------------------------------ 0x9/0xD: SUB/ADD
					4'h9, 4'hD: begin : dec_addsub
						reg is_add;
						is_add = (ir_hi == 4'hD);
						if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111) begin
							// ADDA/SUBA
							alu_op <= is_add ? `AP040_ALU_ADD : `AP040_ALU_SUB;
							op_size <= `AP040_SZ_L;
							p_ssize <= d_op8_6[2] ? `AP040_SZ_L : `AP040_SZ_W;
							p_sextw <= !d_op8_6[2];
							p_flags <= 0;
							p_dst <= DK_REG; p_dreg <= {1'b1, d_reg9};
							if (d_mode == 3'b000 || d_mode == 3'b001) begin
								p_src <= SK_REG; p_sreg <= {d_mode[0], d_rn}; pipe_go;
							end
							else if (ea_is_imm) begin
								p_src <= SK_IMM;
								immf(d_op8_6[2] ? 2'd2 : 2'd1, S_PIPE_START);
							end
							else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
						end
						else if (ir[8] && d_mode[2:1] == 2'b00 && std_size != 2'b11) begin
							// ADDX/SUBX
							alu_op <= is_add ? `AP040_ALU_ADDX : `AP040_ALU_SUBX;
							op_size <= std_size;
							p_ssize <= std_size; p_dsize <= std_size;
							if (!d_mode[0]) begin
								p_src <= SK_REG; p_sreg <= {1'b0, d_rn};
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
							end
							else begin
								p_src <= SK_MEM; src_mode_r <= 3'b100; src_rn_r <= d_rn;
								p_dst <= DK_MEM; dst_mode_r <= 3'b100; dst_rn_r <= d_reg9;
								p_rmw <= 1;
							end
							pipe_go;
						end
						else begin
							alu_op <= is_add ? `AP040_ALU_ADD : `AP040_ALU_SUB;
							op_size <= std_size;
							p_ssize <= std_size; p_dsize <= std_size;
							if (!ir[8]) begin
								// <ea> op Dn -> Dn
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
								if (d_mode == 3'b001 && std_size == `AP040_SZ_B) go_illegal;
								else if (d_mode == 3'b000 || d_mode == 3'b001) begin
									p_src <= SK_REG; p_sreg <= {d_mode[0], d_rn}; pipe_go;
								end
								else if (ea_is_imm) begin p_src <= SK_IMM; immf((std_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START); end
								else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
							end
							else begin
								// Dn op <ea> -> <ea>
								p_src <= SK_REG; p_sreg <= {1'b0, d_reg9};
								p_dst <= DK_MEM; p_rmw <= 1;
								dst_mode_r <= d_mode; dst_rn_r <= d_rn;
								// The register-to-EA ADD/SUB form is memory-only;
								// register-direct encodings belong to ADDX/SUBX.
								if (d_mode < 3'b010 ||
								    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
								else pipe_go;
							end
						end
					end

					//---------------------------------------------- 0xA: A-line
					4'hA: exc(`AP040_VEC_ALINE, 4'd0, pc_i, 32'd0);

					//---------------------------------- 0xB: CMP/CMPA/EOR/CMPM
					4'hB: begin
						if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111) begin
							// CMPA
							alu_op <= `AP040_ALU_CMP;
							op_size <= `AP040_SZ_L;
							p_ssize <= d_op8_6[2] ? `AP040_SZ_L : `AP040_SZ_W;
							p_sextw <= !d_op8_6[2];
							p_wbsup <= 1;
							p_dst <= DK_REG; p_dreg <= {1'b1, d_reg9};
							if (d_mode == 3'b000 || d_mode == 3'b001) begin
								p_src <= SK_REG; p_sreg <= {d_mode[0], d_rn}; pipe_go;
							end
							else if (ea_is_imm) begin
								p_src <= SK_IMM;
								immf(d_op8_6[2] ? 2'd2 : 2'd1, S_PIPE_START);
							end
							else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
						end
						else if (!ir[8]) begin
							// CMP <ea>,Dn
							alu_op <= `AP040_ALU_CMP;
							op_size <= std_size;
							p_ssize <= std_size;
							p_wbsup <= 1;
							p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
							if (d_mode == 3'b001 && std_size == `AP040_SZ_B) go_illegal;
							else if (d_mode == 3'b000 || d_mode == 3'b001) begin
								p_src <= SK_REG; p_sreg <= {d_mode[0], d_rn}; pipe_go;
							end
							else if (ea_is_imm) begin p_src <= SK_IMM; immf((std_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START); end
							else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
						end
						else if (d_mode == 3'b001) begin
							// CMPM (Ay)+,(Ax)+
							alu_op <= `AP040_ALU_CMP;
							op_size <= std_size;
							p_ssize <= std_size; p_dsize <= std_size;
							p_wbsup <= 1;
							p_src <= SK_MEM; src_mode_r <= 3'b011; src_rn_r <= d_rn;
							p_dst <= DK_MEM; dst_mode_r <= 3'b011; dst_rn_r <= d_reg9;
							p_rmw <= 1;
							pipe_go;
						end
						else begin
							// EOR Dn,<ea>
							alu_op <= `AP040_ALU_EOR;
							op_size <= std_size;
							p_dsize <= std_size;
							p_src <= SK_REG; p_sreg <= {1'b0, d_reg9};
							if (d_mode == 3'b000) begin p_dst <= DK_REG; p_dreg <= {1'b0, d_rn}; pipe_go; end
							else if (dst_not_alt) go_illegal;
							else begin
								p_dst <= DK_MEM; p_rmw <= 1;
								dst_mode_r <= d_mode; dst_rn_r <= d_rn;
								pipe_go;
							end
						end
					end

					//------------------------------------ 0xC: AND/MUL/EXG
					4'hC: begin
						if (d_op8_6 == 3'b011 || d_op8_6 == 3'b111) begin
							// MULU.W / MULS.W
							exec_kind <= EK_MD_W;
							md_isdiv <= 0;
							md_sign <= d_op8_6[2];
							op_size <= `AP040_SZ_W;
							p_ssize <= `AP040_SZ_W;
							p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
							if (d_mode == 3'b001) go_illegal;
							else if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; pipe_go; end
							else if (ea_is_imm) begin p_src <= SK_IMM; immf(2'd1, S_PIPE_START); end
							else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
						end
						else if (ir[8] && d_op8_6[1:0] == 2'b00 && d_mode[2:1] == 2'b00) begin
							// ABCD
							alu_op <= `AP040_ALU_ABCD;
							op_size <= `AP040_SZ_B;
							p_ssize <= `AP040_SZ_B; p_dsize <= `AP040_SZ_B;
							if (!d_mode[0]) begin
								p_src <= SK_REG; p_sreg <= {1'b0, d_rn};
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
							end
							else begin
								p_src <= SK_MEM; src_mode_r <= 3'b100; src_rn_r <= d_rn;
								p_dst <= DK_MEM; dst_mode_r <= 3'b100; dst_rn_r <= d_reg9;
								p_rmw <= 1;
							end
							pipe_go;
						end
						else if (ir[8] && (d_op8_6[1:0] == 2'b01) && d_mode[2:1] == 2'b00) begin
							// EXG Dn,Dn (mode 000) / EXG An,An (mode 001)
							rr_a <= {d_mode[0], d_reg9};
							rr_b <= {d_mode[0], d_rn};
							state <= S_EXG1;
						end
						else if (ir[8] && d_op8_6[1:0] == 2'b10 && d_mode == 3'b001) begin
							// EXG Dn,An
							rr_a <= {1'b0, d_reg9};
							rr_b <= {1'b1, d_rn};
							state <= S_EXG1;
						end
						else begin
							// AND
							alu_op <= `AP040_ALU_AND;
							op_size <= std_size;
							p_ssize <= std_size; p_dsize <= std_size;
							if (!ir[8]) begin
								p_dst <= DK_REG; p_dreg <= {1'b0, d_reg9};
								if (d_mode == 3'b001) go_illegal;
								else if (d_mode == 3'b000) begin p_src <= SK_REG; p_sreg <= {1'b0, d_rn}; pipe_go; end
								else if (ea_is_imm) begin p_src <= SK_IMM; immf((std_size == `AP040_SZ_L) ? 2'd2 : 2'd1, S_PIPE_START); end
								else begin p_src <= SK_MEM; src_mode_r <= d_mode; src_rn_r <= d_rn; pipe_go; end
							end
							else begin
								p_src <= SK_REG; p_sreg <= {1'b0, d_reg9};
								p_dst <= DK_MEM; p_rmw <= 1;
								dst_mode_r <= d_mode; dst_rn_r <= d_rn;
								// The register-to-EA AND form is memory-only.
								// Mode 000 combinations not claimed by ABCD/EXG
								// are reserved, rather than AND Dn,Dn aliases.
								if (d_mode < 3'b010 ||
								    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
								else pipe_go;
							end
						end
					end

					//---------------------------------------- 0xE: shifts
					4'hE: begin
						if (ir[7:6] == 2'b11) begin
							if (ir[11]) begin
								// bitfield group; ext word first
								// modify ops need an alterable EA
								if (d_mode == 3'b001 || d_mode == 3'b011 ||
								    d_mode == 3'b100 || ea_is_imm) go_illegal;
								else if ((d_mode == 3'b111 && d_rn > 3'b001) &&
								         (ir[10:8] == 3'd2 || ir[10:8] == 3'd4 ||
								          ir[10:8] == 3'd6 || ir[10:8] == 3'd7)) go_illegal;
								else immf(2'd1, S_BF0);
							end
							else begin
								// memory shift by one, word
								exec_kind <= EK_SHIFT;
								sh_rox <= (ir[10:9] == 2'b10);
								case (ir[10:9])
									2'b00: alu_op <= ir[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1;
									2'b01: alu_op <= ir[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1;
									2'b10: alu_op <= ir[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1;
									default: alu_op <= ir[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1;
								endcase
								op_size <= `AP040_SZ_W;
								p_dsize <= `AP040_SZ_W;
								p_src <= SK_NONE;   // count of one
								p_rmw <= 1;
								// Memory shifts require a memory-alterable EA: Dn/An
								// direct and all program-space encodings are illegal.
								if (d_mode < 3'b010 ||
								    (d_mode == 3'b111 && d_rn > 3'b001)) go_illegal;
								else begin
									p_dst <= DK_MEM;
									dst_mode_r <= d_mode; dst_rn_r <= d_rn;
									pipe_go;
								end
							end
						end
						else begin
							// register shift
							exec_kind <= EK_SHIFT;
							sh_rox <= (ir[4:3] == 2'b10);
							case (ir[4:3])
								2'b00: alu_op <= ir[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1;
								2'b01: alu_op <= ir[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1;
								2'b10: alu_op <= ir[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1;
								default: alu_op <= ir[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1;
							endcase
							op_size <= std_size;
							p_dst <= DK_REG; p_dreg <= {1'b0, d_rn};
							if (ir[5]) begin
								p_src <= SK_REG; p_sreg <= {1'b0, d_reg9};
							end
							else begin
								p_src <= SK_IMPL;
								src_val <= {26'd0, (d_reg9 == 3'd0) ? 6'd8 : {3'd0, d_reg9}};
							end
							pipe_go;
						end
					end

					//------------------------------------------ 0xF: 040 group
					default: begin
						if (ir[11:8] == 4'h4) begin
							// CINV/CPUSH: write-through caches hold no dirty
							// data, so both invalidate the selected caches
							// (scope is widened to ALL, which is safe)
							// Scope bit patterns 000 and 100 are unassigned
							// F-line encodings.  Classify them before privilege.
							if (ir[4:3] == 2'b00) go_fp_fline;
							else if (!sr_s) go_priv;
							else begin
								cinv_ic <= ir[7];
								cinv_dc <= ir[6];
								epf_flush;
								cinv_req <= 1;
								state <= S_CINV2;
							end
						end
						else if (ir[11:8] == 4'h5) begin
							if (ir[7:5] == 3'b000) begin
								// PFLUSH group
								if (!sr_s) go_priv;
								else begin
									epf_flush;
									pf_mode <= ir[4:3];
									if (ir[4]) begin
										// PFLUSHAN / PFLUSHA
										state <= S_PFLUSH2;
									end
									else begin
										rr_a <= {1'b1, d_rn};
										state <= S_PFLUSH1;
									end
								end
							end
							else if (ir[7:6] == 2'b01) begin
								// PTEST
								// Only F548..F54F and F568..F56F are PTEST;
								// the rest of this quadrant is unassigned F-line.
								if (ir[4:3] != 2'b01) go_fp_fline;
								else if (!sr_s) go_priv;
								else begin
									rr_a <= {1'b1, d_rn};
									state <= S_PTEST1;
								end
							end
							else exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
						end
						else if (ir[11:8] == 4'h2) begin
							// FPU coprocessor space (cpid 1)
							if (AP040_HAS_FPU == 0)
								exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
							else case (ir[7:6])
								2'b00: begin                         // general
									// Mode-7 registers 5..7 are reserved for every
									// coprocessor command.  Reject them before fetching
									// an extension word; malformed primary opcodes take
									// the F-line vector, independent of the next word.
									if (d_mode == 3'b111 && d_rn > 3'b100)
										exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
									else immf(2'd1, S_FPU_DEC);
								end
								2'b01: begin                         // FScc/FDBcc/FTRAPcc
									// As in the general command space, mode-7
									// registers 5..7 are primary-word F-line errors.
									if (d_mode == 3'b111 && d_rn > 3'b100)
										exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
									else immf(2'd1, S_FSCC0);
								end
								2'b10:   immf(2'd1, S_FBCC);      // FBcc.W
								default: immf(2'd2, S_FBCC);      // FBcc.L
							endcase
						end
						else if (ir[11:8] == 4'h3) begin
							// FSAVE/FRESTORE state-frame model: NULL, IDLE and the
							// revision-$41 unimplemented-instruction frame are
							// implemented.  A true BUSY arithmetic-exception frame
							// remains outside this non-pipelined FPU's state model.
							if (ir[7:6] == 2'b00) begin
								// FSAVE: control alterable or -(An)
								// Malformed coprocessor EAs are F-line faults and
								// are classified before privilege.
								if (d_mode < 3'b010 || d_mode == 3'b011 ||
								    (d_mode == 3'b111 && d_rn > 3'b001)) go_fp_fline;
								else if (!sr_s) go_priv;
								else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FSAVE1);
							end
							else if (ir[7:6] == 2'b01) begin
								// FRESTORE: control, (An)+ or PC relative
								if (d_mode < 3'b010 || d_mode == 3'b100 ||
								    (d_mode == 3'b111 && d_rn >= 3'b100)) go_fp_fline;
								else if (!sr_s) go_priv;
								else ea_start(d_mode, d_rn, `AP040_SZ_L, S_FREST1);
							end
							else exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
						end
						else if (ir[11:8] == 4'h6 && ir[7:5] == 3'b000) begin
							// MOVE16 with absolute long operand
							m16_form <= {1'b0, ir[4:3]};
							immf(2'd2, S_M16_SRC);
						end
						else if (ir[11:8] == 4'h6 && ir[7:3] == 5'b00100) begin
							// MOVE16 (Ax)+,(Ay)+
							m16_form <= 3'd4;
							immf(2'd1, S_M16_SRC);
						end
						else exc(`AP040_VEC_FLINE, 4'd0, pc_i, 32'd0);
					end
				endcase
			end

			//------------------------------------------------------- stopped
			S_STOP_LD: begin
				epf_flush;
				sr <= imm[15:0] & `AP040_SR_MASK;
				// T1 traces STOP unconditionally.  T0 traces it only when
				// the written SR changes T1/T0/S/M or the interrupt mask:
				// WinUAE's MakeFromSR returns before its trace decision
				// when none of those bits change ("STOP SR-modification
				// does not generate T0"), and STOP has no check_t0_trace
				// like the MOVE/ORI/ANDI/EORI-to-SR family, so an
				// upper-identical STOP under T0 does not trace on the 040.
				if (tr_t1 || (tr_t0 && {imm[15:12], imm[10:8]} !=
				                       {sr[15:12], sr[10:8]})) begin
					tr_t1 <= 0;
					tr_t0 <= 0;
					exc(`AP040_VEC_TRACE, 4'd2, pc, pc_i);
				end
				else state <= S_STOPPED;
			end

			S_STOPPED: begin
				if (irq_pend) begin
					exc_vec <= `AP040_VEC_AUTOVEC + {5'd0, irq_take_lvl};
					exc_fmt <= 0; exc_spc <= pc; exc_addr <= 0;
					exc_is_irq <= 1; exc_pass2 <= 0;
					irq_lvl_l <= irq_take_lvl;
					epf_flush;
					state <= S_EXC0;
				end
			end

			//------------------------------------------------------- TRAPcc
			S_TRAPCC: exc(`AP040_VEC_TRAPCC, 4'd2, pc, pc_i);

			//--------------------------------------------------------- halted
			S_HALT: begin
				mem_req <= 0;
				m_issued <= 0;
				pt_req <= 0;
				pf_req <= 0;
				cinv_req <= 0;
				fpu_req <= 0;
				epf_flush;
				epf_pend <= 0;
				epf_kill <= 0;
			end

			default: fatal_halt;
		endcase

		//-------------------------------------------------- fetch queue engine
		// The queue fills itself: whenever the memory port is idle, the
		// stream is armed, and there is room for the whole request, the
		// next words of the instruction stream are fetched while the core
		// executes.  This runs after the case statement so that any state
		// which claimed the port this cycle keeps it; epf_issue/epf_flushed
		// carry that decision here combinationally.
		if (epf_pend && i_ack) begin
			// A longword request returns the word at the fetch address in
			// [31:16] and its successor in [15:0]; a word request returns
			// one word in [15:0].
			epf_pend <= 0;
			epf_kill <= 0;
			if (!epf_kill && !epf_flushed) begin
				if (epf_pend_lw) begin
					epf_data[epf_fill]        <= mem_rdata[31:16];
					epf_data[epf_fill + 3'd1] <= mem_rdata[15:0];
					epf_fillw = 2'd2;
				end
				else begin
					epf_data[epf_fill] <= mem_rdata[15:0];
					epf_fillw = 2'd1;
				end
			end
		end
		else if (epf_pend && i_err) begin
			// A fault on a queue fetch.  If the core is waiting for exactly
			// this word the access error is taken now, with the faulting
			// request still in the mem_* registers that build the frame.
			// A fault on a word fetched ahead of demand is only recorded:
			// the fetch is re-issued when execution actually reaches it, and
			// faults again there with the live context.
			mem_req  <= 0;
			epf_pend <= 0;
			epf_kill <= 0;
			if (epf_kill || epf_flushed) begin
				// abandoned before the fault: nothing to report
			end
			else if ((state == S_FETCH || state == S_IMMF) &&
			         epf_count == 4'd0 && epf_next == mem_addr) begin
				if (in_exc) fatal_halt;
				else aerr_start;
			end
			else epf_err <= 1;
		end
		// A recorded fault re-arms when execution reaches the faulting word.
		else if (epf_err && epf_armed && !epf_pend && epf_count == 4'd0 &&
		         epf_next == epf_ftail &&
		         (state == S_FETCH || state == S_IMMF))
			epf_err <= 0;
		// Self-fill.  The fetch stays inside the page the core is already
		// executing from: an aligned fetch within that page cannot translate
		// or fault differently than the fetch that got the core here, which
		// is what makes a speculative fetch safe.
		// epf_kill is not tested here: it only qualifies an outstanding
		// fetch, and the issue below clears it.
		// lk_cyc: a TAS/CAS/CAS2 operand sequence must stay indivisible at
		// the core/adapter boundary (plan section 8), so no SPECULATIVE
		// fetch may be interleaved between the locked read and write.  A
		// demand fetch (S_IMMF starving on an empty queue) must still be
		// served: lk_cyc is set at decode, BEFORE the extension words are
		// consumed, and those fetches precede the locked read.  This also
		// keeps a stale lk_cyc after a faulted CAS from starving the
		// handler's first instruction.
		else if (epf_armed && !epf_pend && !epf_err &&
		         !epf_issue && !epf_flushed &&
		         !mem_req && !mem_ack &&
		         (!lk_cyc || state == S_IMMF) &&
		         (epf_super == sr_s) &&
		         (epf_ftail[31:12] == pc[31:12]) &&
		         state != S_MRD && state != S_MWR &&
		         state != S_MRD_B && state != S_MWR_B &&
		         state != S_EPF_FILL && state != S_EPF_GAP &&
		         // Computing an effective address means a DATA access is
		         // imminent, and the port is shared: a speculative fetch
		         // started here is still in flight when S_MRD wants it, so
		         // the data access queues behind a whole fill.  Skipping
		         // these slots costs the queue a little run-ahead and is
		         // worth 12% on loop code (bench_loop).  Demand fetches are
		         // untouched -- S_FETCH and S_IMMF are not EA states.
		         !ea_state &&
		         (epf_ftail[1] ? (epf_count <= 4'd7) : (epf_count <= 4'd6)))
		begin
			mem_req <= 1; mem_write <= 0; mem_instr <= 1;
			mem_size <= epf_ftail[1] ? `AP040_SZ_W : `AP040_SZ_L;
			mem_addr <= epf_ftail;
			fc_r <= epf_super ? `AP040_FC_SUPER_PROG : `AP040_FC_USER_PROG;
			epf_pend <= 1;
			epf_pend_lw <= ~epf_ftail[1];
			epf_kill <= 0;
		end

		// Queue bookkeeping in one place, so that a pop and an append in the
		// same cycle cannot lose each other's update.  A flush has already
		// written the whole set and wins.
		if (!epf_flushed && (epf_pop != 2'd0 || epf_fillw != 2'd0)) begin
			epf_count <= epf_count + {2'd0, epf_fillw} - {2'd0, epf_pop};
			if (epf_pop != 2'd0) begin
				epf_head <= epf_head + {1'b0, epf_pop};
				epf_next <= epf_next + {29'd0, epf_pop, 1'b0};
			end
			if (epf_fillw != 2'd0) begin
				epf_fill  <= epf_fill + {1'b0, epf_fillw};
				epf_ftail <= epf_ftail + {29'd0, epf_fillw, 1'b0};
			end
		end
	end
end

//---------------------------------------------------------------------------
// debug/status
//---------------------------------------------------------------------------

assign debug_busy   = mem_req;
assign debug_fault  = fault_r;
assign debug_halted = (state == S_HALT);

assign debug_status2 = {
	aer_fa,                      // [127:96] address whose access faulted
	usp_q,                       // [95:64]
	isp_q,                       // [63:32]
	16'd0, exc_vec, 5'd0, in_exc, fault_r, 1'b0   // [31:0]
};

assign debug_status = {
	16'hA040,                    // [255:240] magic
	6'd0, fault_r, unused_in,    // [239:232]
	state,                       // [231:224]
	dbg_a0,                      // [223:192]
	dbg_d2,                      // [191:160]
	dbg_d1,                      // [159:128]
	dbg_d0,                      // [127:96]
	dbg_a7,                      // [95:64]
	ir,                          // [63:48]
	sr,                          // [47:32]
	pc                           // [31:0]
};

endmodule
