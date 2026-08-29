//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 15: supervisor      //
// state)                                                                   //
//                                                                          //
// tb_ap040_pipe_sup.v - VBR/SFC/DFC/CACR/USP/ISP/MSP, mode switching,      //
// privilege violation                                                      //
//                                                                          //
// Four phases, chained (each depends on the previous one's state, proving   //
// this is a coherent architectural sequence, not four isolated pokes):        //
//                                                                          //
// Phase 1 (supervisor, S=1 from reset) -- MOVEC's WRITE direction sets every    //
// one of VBR/SFC/DFC/CACR/USP/ISP/MSP to a distinct, checkable value (verified     //
// directly against ap040_pipe_core.v's own registers / ap040_pipe_regfile.v's        //
// usp/isp/msp -- hierarchical reads, the same style every push-verifying test          //
// already uses), then MOVEC's READ direction reads VBR back into D7, proving both       //
// directions work end-to-end through the real pipeline (regfile commit,                  //
// forwarding), not just the control-register storage itself.                              //
//                                                                          //
// Phase 2 -- MOVE to SR (register-direct, D0=0) drops S to 0: the ONLY way this      //
// pipeline can currently switch to user mode (RTE, the other real mechanism,           //
// is deferred -- see AP040_IMPLEMENTATION_PLAN.md).                                     //
//                                                                          //
// Phase 3 (now genuinely in user mode) -- an ORDINARY, unprivileged BSR proves            //
// A7 correctly banks to USP (set to $50 in phase 1), NOT ISP ($70, untouched): the         //
// push lands at USP-4, USP updates to USP-4, and ISP is bit-for-bit unperturbed.            //
// This is the pipeline's first real exercise of dynamic A7 banking -- every earlier          //
// BSR/JSR test ran with sr_s hardwired to 1 the whole time, so this is genuinely new           //
// coverage, not a re-run of an old one.                                                        //
//                                                                          //
// Phase 4 (still user mode) -- a privileged MOVEC now FAULTS: vector 8, format $0,           //
// verified against the SAME format-$0 frame-content checks tb_ap040_pipe_exc.v              //
// already established, but this time proving the CRITICAL, previously-untested                //
// detail that an exception taken WHILE ALREADY IN USER MODE still lands its frame              //
// on the SUPERVISOR stack (ISP, $70, decremented to $68) -- NOT on USP (which stays              //
// exactly where phase 3 left it, $4C, completely unperturbed by this second event).               //
// See ap040_ea_fetch.v's header for the real bug this test was specifically written               //
// to catch (a naive "read A7 via the currently active bank" implementation would                  //
// have silently pushed onto USP instead).                                                          //
//                                                                          //
// Word layout (PC_RESET-relative), with each phase's register reuse noted --            //
// D0-D6 all get distinct MOVEC source values in phase 1; phase 3/4's poison/target/          //
// handler markers deliberately reuse those SAME registers with FRESH, DISTINCT               //
// values rather than needing fresh ones -- "did NOT change from its phase-1 value"             //
// is exactly as valid a poison-never-ran proof as "stayed zero" is, and this program             //
// only has eight registers to work with across four phases:                                      //
//                                                                          //
//  0: NOP                                                                 //
//  1-3:   MOVEQ #$40,D0 ; MOVEC D0,VBR                                     //
//  4-6:   MOVEQ #$FF,D1 ; MOVEC D1,CACR   (D1=$FFFFFFFF, masked to $80008000) //
//  7-9:   MOVEQ #$03,D2 ; MOVEC D2,SFC                                     //
//  10-12: MOVEQ #$05,D3 ; MOVEC D3,DFC                                     //
//  13-15: MOVEQ #$70,D4 ; MOVEC D4,ISP                                     //
//  16-18: MOVEQ #$60,D5 ; MOVEC D5,MSP                                     //
//  19-21: MOVEQ #$50,D6 ; MOVEC D6,USP                                     //
//  22-23: MOVEC VBR,D7               (read-back check, D7 must become $40)  //
//  24-25: MOVEQ #$00,D0 ; MOVE D0,SR (drop to user mode)                    //
//  26:    BSR.B <+2>                 (user-mode push, target = idx 28)       //
//  27:    MOVEQ #$63,D2              (poison A: D2 must STAY $03)             //
//  28:    MOVEQ #$77,D3              (target A: D3 must become $77)            //
//  29-30: MOVEC D0,VBR (D0=$00)      (PRIVILEGED -- must fault, vector 8)        //
//  31:    MOVEQ #$88,D4              (poison B: D4 must STAY $70)                //
//                                                                          //
//  Privilege-violation handler @ byte $500 (word idx 128, well clear of both     //
//  the program above and the vector table's own word-index range):               //
//    MOVEQ #$99,D5      (marker: D5 must become $FFFFFF99 -- $99 sign-extends; was $60) //
//    NOP (drain)                                                                    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_sup;

localparam PROG_WORDS      = 40;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3;
wire [31:0] dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;
wire [15:0] dbg_sr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_ccr(dbg_ccr), .dbg_sr(dbg_sr)
);

integer errors = 0;

initial begin
	#1;
	// Phase 1: MOVEC's write direction, one control register at a time.
	dut.u_l1.mem[ 1] = 16'h7040;  dut.u_l1.mem[ 2] = 16'h4E7B;  dut.u_l1.mem[ 3] = 16'h0801; // D0 -> VBR
	dut.u_l1.mem[ 4] = 16'h72FF;  dut.u_l1.mem[ 5] = 16'h4E7B;  dut.u_l1.mem[ 6] = 16'h1002; // D1 -> CACR
	dut.u_l1.mem[ 7] = 16'h7403;  dut.u_l1.mem[ 8] = 16'h4E7B;  dut.u_l1.mem[ 9] = 16'h2000; // D2 -> SFC
	dut.u_l1.mem[10] = 16'h7605;  dut.u_l1.mem[11] = 16'h4E7B;  dut.u_l1.mem[12] = 16'h3001; // D3 -> DFC
	dut.u_l1.mem[13] = 16'h7870;  dut.u_l1.mem[14] = 16'h4E7B;  dut.u_l1.mem[15] = 16'h4804; // D4 -> ISP
	dut.u_l1.mem[16] = 16'h7A60;  dut.u_l1.mem[17] = 16'h4E7B;  dut.u_l1.mem[18] = 16'h5803; // D5 -> MSP
	dut.u_l1.mem[19] = 16'h7C50;  dut.u_l1.mem[20] = 16'h4E7B;  dut.u_l1.mem[21] = 16'h6800; // D6 -> USP
	// MOVEC's read direction: VBR -> D7.
	dut.u_l1.mem[22] = 16'h4E7A;  dut.u_l1.mem[23] = 16'h7801;

	// Phase 2: drop to user mode.
	dut.u_l1.mem[24] = 16'h7000;  // MOVEQ #$00,D0
	dut.u_l1.mem[25] = 16'h46C0;  // MOVE D0,SR

	// Phase 3: ordinary BSR in user mode -- must bank to USP.
	dut.u_l1.mem[26] = 16'h6102;  // BSR.B <+2> -> index 28
	dut.u_l1.mem[27] = 16'h7263;  // MOVEQ #$63,D2 (poison A: must not run)
	dut.u_l1.mem[28] = 16'h7677;  // MOVEQ #$77,D3 (target A: must run)

	// Phase 4: a privileged MOVEC in user mode -- must fault.
	dut.u_l1.mem[29] = 16'h4E7B;  dut.u_l1.mem[30] = 16'h0801;  // MOVEC D0,VBR (D0=0)
	dut.u_l1.mem[31] = 16'h7888;  // MOVEQ #$88,D4 (poison B: must not run)

	// Privilege-violation handler @ word idx 128 (byte $500).
	dut.u_l1.mem[128] = 16'h7A99; // MOVEQ #$99,D5
	dut.u_l1.mem[129] = 16'h4E71; // NOP (drain)

	// Vector table: vector 8 (privilege violation) -> $500. Word index
	// computed the same PC_RESET-relative-wraparound way every exception
	// test since milestone 14 has used -- see ap040_ea_fetch.v's header.
	dut.u_l1.mem[3600] = 16'h0000;
	dut.u_l1.mem[3601] = 16'h0500;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	repeat (PROG_WORDS + 100) @(posedge clk);

	// ---------------------------------------------- Phase 1: MOVEC writes
	if (dut.vbr !== 32'h0000_0040) begin
		errors = errors + 1;
		$display("FAIL: VBR = %h, expected 00000040", dut.vbr);
	end
	if (dut.cacr !== 32'h8000_8000) begin
		errors = errors + 1;
		$display("FAIL: CACR = %h, expected 80008000 (0xFFFFFFFF masked & 32'h8000_8000)", dut.cacr);
	end
	if (dut.sfc !== 3'd3) begin
		errors = errors + 1;
		$display("FAIL: SFC = %0d, expected 3", dut.sfc);
	end
	if (dut.dfc !== 3'd5) begin
		errors = errors + 1;
		$display("FAIL: DFC = %0d, expected 5", dut.dfc);
	end
	// ISP: written to $70 here, then decremented by phase 4's exception
	// frame (-8) at the very end -- checked at its FINAL value below, not
	// here, to avoid asserting a value this same test later legitimately
	// changes.
	if (dut.u_regfile.msp !== 32'h0000_0060) begin
		errors = errors + 1;
		$display("FAIL: MSP = %h, expected 00000060", dut.u_regfile.msp);
	end
	// USP: written to $50 here, then decremented by phase 3's BSR push
	// (-4) -- same "check the final value" reasoning as ISP above.

	if (dbg_d7 !== 32'h0000_0040) begin
		errors = errors + 1;
		$display("FAIL: D7 = %h, expected 00000040 (MOVEC's read direction: VBR -> D7)", dbg_d7);
	end

	// Phase 2 (mode switch) has no check of its own at this point in the
	// program: S is back to 1 by the time this final snapshot runs (phase
	// 4's privilege violation legitimately forces it back), so "S==0" can
	// only be checked TRANSIENTLY, not at end-of-simulation. It's proven
	// indirectly instead -- phase 3's BSR banking to USP (not ISP) below
	// could only pass if S genuinely was 0 while it executed.

	// ---------------------------------------------- Phase 3: user-mode BSR
	if (dbg_d2 !== 32'h0000_0003) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000003 (poison A ran -- BSR's redirect broken in user mode)", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0077) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000077 (target A did not run -- BSR broken in user mode)", dbg_d3);
	end
	if (dut.u_regfile.usp !== 32'h0000_004C) begin
		errors = errors + 1;
		$display("FAIL: USP = %h, expected 0000004c (BSR's push, in user mode, must decrement USP by 4 from $50)", dut.u_regfile.usp);
	end
	// The pushed return address, read directly out of the L1 array at
	// USP-4 = $4C, same style every push-verifying test already uses.
	if (dut.u_l1.mem[16'h0E26] !== 16'h0000 || dut.u_l1.mem[16'h0E27] !== 16'h0436) begin
		errors = errors + 1;
		$display("FAIL: BSR push (at USP-4=$4C) = %h%h, expected 00000436",
		          dut.u_l1.mem[16'h0E26], dut.u_l1.mem[16'h0E27]);
	end

	// ------------------------------------------- Phase 4: privilege fault
	if (dbg_d4 !== 32'h0000_0070) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000070 (poison B ran -- privileged MOVEC did not fault in user mode)", dbg_d4);
	end
	// MOVEQ #$99,D5: $99 as a SIGNED 8-bit immediate is -103, which MOVEQ
	// sign-extends to $FFFFFF99, not a zero-extended $99 -- real MOVEQ
	// semantics, not a bug (tb_ap040_pipe_bsr.v's own poison values rely on
	// the same sign-extension, e.g. #$63/#$58 staying small precisely
	// because their top bit is 0).
	if (dbg_d5 !== 32'hFFFF_FF99) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected ffffff99 (privilege-violation handler did not run)", dbg_d5);
	end
	// VBR must be UNCHANGED by the faulted MOVEC (it never executes its
	// real effect) -- still $40 from phase 1, not $00.
	if (dut.vbr !== 32'h0000_0040) begin
		errors = errors + 1;
		$display("FAIL: VBR = %h after the faulted MOVEC, expected 00000040 (unchanged -- the write must never have happened)", dut.vbr);
	end
	// The critical check this test exists for: the exception frame must
	// land on the SUPERVISOR stack (ISP, $70->$68), NOT on USP -- which
	// must stay EXACTLY where phase 3 left it, completely unperturbed.
	if (dut.u_regfile.isp !== 32'h0000_0068) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00000068 (privilege-violation frame must decrement ISP by 8 from $70, even though the fault occurred IN USER MODE)", dut.u_regfile.isp);
	end
	if (dut.u_regfile.usp !== 32'h0000_004C) begin
		errors = errors + 1;
		$display("FAIL: USP = %h after the privilege violation, expected 0000004c (must be COMPLETELY unperturbed by an exception frame that has nothing to do with the user stack)", dut.u_regfile.usp);
	end
	// Frame @ ISP-8=$68 (word idx $0F4/$0F6): SR=$0000 (the OLD, pre-fault
	// live SR -- S=0, everything else 0), PC=$0000043A (the faulting
	// MOVEC's OWN address, go_priv's convention, same as illegal's),
	// FmtVec=$0020 (format 0, vector 8 -> 8*4=$20).
	if (dut.u_l1.mem[16'h0E34] !== 16'h0000 || dut.u_l1.mem[16'h0E35] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: priv frame word0 (SR:PChi) = %h%h, expected 00000000",
		          dut.u_l1.mem[16'h0E34], dut.u_l1.mem[16'h0E35]);
	end
	if (dut.u_l1.mem[16'h0E36] !== 16'h043A || dut.u_l1.mem[16'h0E37] !== 16'h0020) begin
		errors = errors + 1;
		$display("FAIL: priv frame word1 (PClo:FmtVec) = %h%h, expected 043A0020",
		          dut.u_l1.mem[16'h0E36], dut.u_l1.mem[16'h0E37]);
	end
	// The live SR, after entering the handler, must be back in supervisor
	// mode -- S forced to 1 regardless of the pre-fault mode, T1/T0
	// cleared, M/IPL/CCR preserved (all 0 here either way).
	if (dbg_sr[13] !== 1'b1) begin
		errors = errors + 1;
		$display("FAIL: SR.S = %b after the privilege-violation handler entry, expected 1 (exceptions always enter supervisor mode)", dbg_sr[13]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
