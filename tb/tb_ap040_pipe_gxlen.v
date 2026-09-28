//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 113: how long a     //
// gathered instruction says it is)                                         //
//                                                                          //
// tb_ap040_pipe_gxlen.v - the next PC of an immediate-plus-EA form        //
//                                                                          //
// A gathered instruction's next PC was held_pc + 2 plus 2, 4 or 6 bytes,   //
// chosen by held_is_long and held_is_xlong. Those say how gather_disp      //
// combines the words it collected, not how many there were, and the forms  //
// milestones 112 and 113 added carry an operand AND an EA extension:      //
// ADDI.W #1,$10(A0) is three words and reported two; ADDI.L #1,$20(A0) is  //
// four and reported three. Nothing in the corpus reads that value -- the   //
// pipe driver does not check an end PC and skips traced rounds -- but it   //
// is the return address an interrupt or a trace after the instruction     //
// stacks, and RTE-ing to it lands inside the instruction.                  //
//                                                                          //
// So every form here runs under T1, and the trace handler files each       //
// frame's PC -- the next instruction's address -- into a table at $900:    //
//                                                                          //
//   $41C ADDI.W #1,$10(A0)          3 words -> $422                        //
//   $422 ADDI.L #1,$20(A0)          4 words -> $42A                        //
//   $42A ADDQ.W #1,$00000830.L      3 words -> $430                        //
//   $430 BSET   #3,$40(A0)          3 words -> $436                        //
//   $436 BTST   #2,$00000850.L      4 words -> $43E                        //
//   $43E CMPI.W #5,$12(A0,D6.W)     3 words -> $444                        //
//   $444 ANDI   #$7FFF,SR           2 words -> $448   (T1 off after this)  //
//                                                                          //
// The data each one leaves is checked too: a trace frame with the right   //
// PC on an instruction that did the wrong thing proves nothing.            //
//                                                                          //
// The trace frames alone do NOT pin the next-PC value: this core's trace   //
// stacks the address of the instruction the handler returns to, taken     //
// from the pipeline, and a mutation restoring the old formula still        //
// passed them. What reads eaf_next_pc today is CHK/TRAPcc/zero-divide      //
// frames, BSR/JSR return addresses and branch/STOP recovery -- none of     //
// them on these forms -- and the corpus driver's successor test for        //
// multi-instruction rounds; an interrupt entry will be the next. So the    //
// bench also checks eaf_next_pc itself as each listed instruction passes   //
// EX. That is a white-box check, and deliberately so.                      //
//                                                                          //
// Trace handler at $700 (vector 9): MOVE.L 2(A7),D7 ; MOVE.L D7,(A1)+ ;    //
// RTE.                                                                     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_gxlen;

localparam PROG_WORDS      = 600;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`endif

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),
	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;

task chk;
	input [255:0] what;
	input integer widx;
	input  [15:0] want;
	begin
		if (dut.u_l1.mem[widx] !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %04x, expected %04x", what, dut.u_l1.mem[widx], want);
		end
	end
endtask

reg [15:0] want_pc [0:6];
integer    npc_bad  = 0;
integer    npc_seen = 0;

// eaf_next_pc against pc + length, for every instruction on the list.
always @(posedge clk)
	if (nreset && ce && dut.u_cpu.u_ex.eaf_valid)
		for (i = 0; i < 7; i = i + 1)
			if (dut.u_cpu.u_ex.eaf_pc == {16'd0, (i == 0) ? 16'h041C : want_pc[i-1]}) begin
				npc_seen = npc_seen + 1;
				if (dut.u_cpu.u_ex.eaf_next_pc !== {16'd0, want_pc[i]}) begin
					npc_bad = npc_bad + 1;
					if (npc_bad <= 8)
						$display("FAIL: eaf_next_pc of the instruction at %08x = %08x, expected 0000%04x",
						         dut.u_cpu.u_ex.eaf_pc, dut.u_cpu.u_ex.eaf_next_pc, want_pc[i]);
				end
			end

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h203C;   // $400 MOVE.L #$00001000,D0
	dut.u_l1.mem[ 1] = 16'h0000;
	dut.u_l1.mem[ 2] = 16'h1000;
	dut.u_l1.mem[ 3] = 16'h4E7B;   // $406 MOVEC D0,ISP
	dut.u_l1.mem[ 4] = 16'h0804;
	dut.u_l1.mem[ 5] = 16'h207C;   // $40A MOVEA.L #$00000800,A0
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'h0800;
	dut.u_l1.mem[ 8] = 16'h227C;   // $410 MOVEA.L #$00000900,A1  -- the PC table
	dut.u_l1.mem[ 9] = 16'h0000;
	dut.u_l1.mem[10] = 16'h0900;
	dut.u_l1.mem[11] = 16'h7C02;   // $416 MOVEQ #2,D6
	dut.u_l1.mem[12] = 16'h007C;   // $418 ORI #$8000,SR  -- T1; not itself traced
	dut.u_l1.mem[13] = 16'h8000;

	dut.u_l1.mem[14] = 16'h0668;   // $41C ADDI.W #1,$10(A0)
	dut.u_l1.mem[15] = 16'h0001;
	dut.u_l1.mem[16] = 16'h0010;
	dut.u_l1.mem[17] = 16'h06A8;   // $422 ADDI.L #1,$20(A0)
	dut.u_l1.mem[18] = 16'h0000;
	dut.u_l1.mem[19] = 16'h0001;
	dut.u_l1.mem[20] = 16'h0020;
	dut.u_l1.mem[21] = 16'h5279;   // $42A ADDQ.W #1,$00000830.L
	dut.u_l1.mem[22] = 16'h0000;
	dut.u_l1.mem[23] = 16'h0830;
	dut.u_l1.mem[24] = 16'h08E8;   // $430 BSET #3,$40(A0)
	dut.u_l1.mem[25] = 16'h0003;
	dut.u_l1.mem[26] = 16'h0040;
	dut.u_l1.mem[27] = 16'h0839;   // $436 BTST #2,$00000850.L
	dut.u_l1.mem[28] = 16'h0002;
	dut.u_l1.mem[29] = 16'h0000;
	dut.u_l1.mem[30] = 16'h0850;
	dut.u_l1.mem[31] = 16'h0C70;   // $43E CMPI.W #5,$12(A0,D6.W)
	dut.u_l1.mem[32] = 16'h0005;
	dut.u_l1.mem[33] = 16'h6012;
	dut.u_l1.mem[34] = 16'h027C;   // $444 ANDI #$7FFF,SR -- T1 off
	dut.u_l1.mem[35] = 16'h7FFF;
	dut.u_l1.mem[36] = 16'h60FE;   // $448 BRA.B -2

	// $700: the trace handler.
	dut.u_l1.mem[384] = 16'h2E2F;  // MOVE.L 2(A7),D7   -- the frame's PC
	dut.u_l1.mem[385] = 16'h0002;
	dut.u_l1.mem[386] = 16'h22C7;  // MOVE.L D7,(A1)+
	dut.u_l1.mem[387] = 16'h4E73;  // RTE

	// Operands, and the table pre-filled so an entry never written shows.
	dut.u_l1.mem[520] = 16'h0010;                                  // $810
	dut.u_l1.mem[528] = 16'h0000;  dut.u_l1.mem[529] = 16'hFFFF;   // $820
	dut.u_l1.mem[536] = 16'h0005;                                  // $830
	dut.u_l1.mem[544] = 16'h0011;                                  // $840
	dut.u_l1.mem[552] = 16'h0400;                                  // $850 (bit 2 of $04 set)
	dut.u_l1.mem[522] = 16'h0005;                                  // $814 = $800+2+$12
	for (i = 640; i < 660; i = i + 1) dut.u_l1.mem[i] = 16'hDEAD;

	// Vector 9 -> $700
	dut.u_l1.mem[3602] = 16'h0000;
	dut.u_l1.mem[3603] = 16'h0700;

	want_pc[0] = 16'h0422;  want_pc[1] = 16'h042A;  want_pc[2] = 16'h0430;
	want_pc[3] = 16'h0436;  want_pc[4] = 16'h043E;  want_pc[5] = 16'h0444;
	want_pc[6] = 16'h0448;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $900 onward, one longword per trace: word 640 + 2k is the high half.
	for (i = 0; i < 7; i = i + 1) begin
		if (dut.u_l1.mem[640 + 2*i] !== 16'h0000 || dut.u_l1.mem[641 + 2*i] !== want_pc[i]) begin
			errors = errors + 1;
			$display("FAIL: trace %0d stacked PC %04x%04x, expected 0000%04x", i,
			         dut.u_l1.mem[640 + 2*i], dut.u_l1.mem[641 + 2*i], want_pc[i]);
		end
	end
	chk("no eighth trace ($91C)",       654, 16'hDEAD);
	if (npc_bad != 0) errors = errors + 1;
	if (npc_seen < 7) begin
		errors = errors + 1;
		$display("FAIL: only %0d of the 7 listed instructions were seen in EX", npc_seen);
	end

	chk("ADDI.W $810",                  520, 16'h0011);
	chk("ADDI.L $820 high",             528, 16'h0001);
	chk("ADDI.L $820 low",              529, 16'h0000);
	chk("ADDQ.W $830",                  536, 16'h0006);
	chk("BSET #3 $840",                 544, 16'h0811);
	chk("BTST/CMPI operands untouched", 552, 16'h0400);
	chk("CMPI.W operand $814",          522, 16'h0005);

	if (dbg_sr[15] !== 1'b0) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected T1 clear at the end", dbg_sr);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
