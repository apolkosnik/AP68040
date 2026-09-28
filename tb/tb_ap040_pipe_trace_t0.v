//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 79: T0, trace on change   //
// of flow)                                                                 //
//                                                                          //
// tb_ap040_pipe_trace_t0.v - T0 set, and only the flow changes traced      //
//                                                                          //
// With T1T0 = 01 a trace exception follows each instruction that changes   //
// the flow: a taken branch or DBcc, BSR/JMP/JSR, RTS/RTE, and the         //
// non-branch instructions the 68040 defines as flow changes because they   //
// resynchronise the pipeline -- MOVE to SR, ORI/ANDI/EORI to SR, MOVEC to  //
// a control register, NOP. A not-taken branch is not one, and neither is   //
// an exception entry: this bench used to expect a trace after the TRAP,    //
// the 68000/68020 rule; ap040_core.v's S_EXC0 arms none, and its notes     //
// record cputest on hardware reporting "Got unexpected trace exception"    //
// when T0 was carried through an ILLEGAL (review 14).                      //
// The frame is the same as T1's: vector 9, format $2, PC field = next      //
// instruction, address field = the instruction that changed the flow.     //
//                                                                          //
// The handler logs {address, PC, SR, fmt/vec} at (A5)+ with D2 as scratch. //
//                                                                          //
//   MOVE.L #$6700,D0 / MOVE D0,SR       T0 := 1 (not itself traced)        //
//   K1  $040A  MOVEQ #1,D1              -                                  //
//   K2  $040C  ADDQ.L #1,D1             -                                  //
//   K3  $040E  BEQ.B +2                 not taken (Z = 0): NOT traced       //
//   K4  $0410  BNE.B t1                 taken: traced, PC field = t1        //
//              $0412  MOVEQ #$7F,D1     skipped -- poison                  //
//   K5  $0414  NOP                      traced                             //
//   K7  $0416  JSR $0880                traced, PC field = the subroutine   //
//   K10 $041C  ORI #$0008,SR            traced                             //
//   K11 $0420  MOVEQ #1,D4              -                                  //
//   K12 $0422  DBF D4,K13               taken (D4 1 -> 0): traced          //
//              $0426  MOVEQ #$7F,D1     skipped -- poison                  //
//   K13 $0428  DBF D4,+4                not taken (D4 0 -> -1): NOT traced; //
//                                       taken would skip the TRAP          //
//   K14 $042C  TRAP #1                  NOT traced: no exception entry     //
//                                       leaves a trace behind it           //
//              (TRAP handler: MOVEQ #$42,D5 / RTE -- untraced)             //
//   K16 $042E  MOVE.L #$2700,D0         -                                  //
//   K17 $0434  MOVE D0,SR               traced, SR $2700: tracing off      //
//   K18 $0436  MOVEQ #$55,D6            -                                  //
//   sub $0880  MOVEQ #7,D3              -   (past $0800, out of            //
//   K9  $0882  RTS                      traced, PC field = $041C  fall-through reach) //
//                                                                          //
// Expected log, seven entries of {address, PC, SR} (fmt/vec $2024 each):   //
//   1 {$0410, $0414, $6700}    5 {$041C, $0420, $6708}                     //
//   2 {$0414, $0416, $6700}    6 {$0422, $0428, $6700}                     //
//   3 {$0416, $0880, $6700}    7 {$0434, $0436, $2700}                     //
//   4 {$0882, $041C, $6700}                                                //
//                                                                          //
// (A first draft of this listing was one word off from K12 on -- the DBF's //
// displacement word -- so the taken DBF landed in the poison slot, and it   //
// put the subroutine at $0440 where the end of the program fell into its    //
// RTS. Both were the bench's errors; the RTL traced exactly what ran.)      //
//                                                                          //
// D7 = 7. D1 = 2 says neither poison ran; D3 = 7 the subroutine ran;       //
// D4 = $0000FFFF the DBF pair counted down; D5 = $42; D6 = $55; ISP $0600; //
// SR $2700. Seven entries and not eight or more is the point: K3 and K13, //
// the not-taken pair, and the TRAP must leave nothing.                     //
//                                                                          //
// On milestone-78 RTL T0 is ignored: D7 = 0.                               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_trace_t0;

localparam PROG_WORDS      = 400;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
// A pseudo-random clock enable (milestone 94). Every bench in this suite
// tied ce high, and eight of the thirteen defects three rounds of external
// review found lived behind that: a cycle with ce low is a cycle that did
// not happen, and the core has to treat it that way. Driven on the falling
// edge so it is stable across every rising one, and left high until reset
// releases so the reset sequence itself is unchanged.
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
wire [31:0] dbg_d0, dbg_d1, dbg_d2;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer e, f;

task check32;
	input string  what;
	input [31:0]  got, want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", what, got, want);
		end
	end
endtask

localparam N_TRACE = 7;
reg [31:0] exp_addr [0:N_TRACE-1];
reg [31:0] exp_pc   [0:N_TRACE-1];
reg [15:0] exp_sr   [0:N_TRACE-1];
initial begin
	exp_addr[0] = 32'h0410; exp_pc[0] = 32'h0414; exp_sr[0] = 16'h6700;   // BNE taken
	exp_addr[1] = 32'h0414; exp_pc[1] = 32'h0416; exp_sr[1] = 16'h6700;   // NOP
	exp_addr[2] = 32'h0416; exp_pc[2] = 32'h0880; exp_sr[2] = 16'h6700;   // JSR
	exp_addr[3] = 32'h0882; exp_pc[3] = 32'h041C; exp_sr[3] = 16'h6700;   // RTS
	exp_addr[4] = 32'h041C; exp_pc[4] = 32'h0420; exp_sr[4] = 16'h6708;   // ORI to SR (sets N)
	exp_addr[5] = 32'h0422; exp_pc[5] = 32'h0428; exp_sr[5] = 16'h6700;   // DBF taken (MOVEQ #1 cleared N)
	exp_addr[6] = 32'h0434; exp_pc[6] = 32'h0436; exp_sr[6] = 16'h2700;   // MOVE to SR clearing T0 (the TRAP left nothing)
end

function [31:0] want_log;
	input integer e, f;
	begin
		case (f)
			0: want_log = exp_addr[e];
			1: want_log = exp_pc[e];
			2: want_log = {16'd0, exp_sr[e]};
			default: want_log = 32'h2024;
		endcase
	end
endfunction

wire [31:0] log_word [0:4*N_TRACE+3];   // one spare entry, to show an unexpected ninth
genvar g;
generate for (g = 0; g < 4*N_TRACE+4; g = g + 1) begin : lw
	assign log_word[g] = {dut.u_l1.mem[768 + 2*g], dut.u_l1.mem[769 + 2*g]};
end endgenerate

initial begin
	#1;
	dut.u_l1.mem[1 ] = 16'h203C;   // MOVE.L #$00006700,D0
	dut.u_l1.mem[2 ] = 16'h0000;
	dut.u_l1.mem[3 ] = 16'h6700;
	dut.u_l1.mem[4 ] = 16'h46C0;   // MOVE D0,SR           @ $0408   T0 := 1
	dut.u_l1.mem[5 ] = 16'h7201;   // K1  MOVEQ #1,D1      @ $040A
	dut.u_l1.mem[6 ] = 16'h5281;   // K2  ADDQ.L #1,D1     @ $040C   Z := 0
	dut.u_l1.mem[7 ] = 16'h6702;   // K3  BEQ.B +2         @ $040E   not taken
	dut.u_l1.mem[8 ] = 16'h6602;   // K4  BNE.B t1         @ $0410   taken -> $0414
	dut.u_l1.mem[9 ] = 16'h727F;   //     MOVEQ #$7F,D1    @ $0412   skipped
	dut.u_l1.mem[10] = 16'h4E71;   // K5  NOP              @ $0414   t1
	dut.u_l1.mem[11] = 16'h4EB9;   // K7  JSR $0880        @ $0416
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0880;
	dut.u_l1.mem[14] = 16'h007C;   // K10 ORI #$0008,SR    @ $041C
	dut.u_l1.mem[15] = 16'h0008;
	dut.u_l1.mem[16] = 16'h7801;   // K11 MOVEQ #1,D4      @ $0420
	dut.u_l1.mem[17] = 16'h51CC;   // K12 DBF D4,K13       @ $0422   taken -> $0428
	dut.u_l1.mem[18] = 16'h0004;
	dut.u_l1.mem[19] = 16'h727F;   //     MOVEQ #$7F,D1    @ $0426   skipped
	dut.u_l1.mem[20] = 16'h51CC;   // K13 DBF D4,+4        @ $0428   not taken (taken would skip the TRAP)
	dut.u_l1.mem[21] = 16'h0004;
	dut.u_l1.mem[22] = 16'h4E41;   // K14 TRAP #1          @ $042C
	dut.u_l1.mem[23] = 16'h203C;   // K16 MOVE.L #$2700,D0 @ $042E
	dut.u_l1.mem[24] = 16'h0000;
	dut.u_l1.mem[25] = 16'h2700;
	dut.u_l1.mem[26] = 16'h46C0;   // K17 MOVE D0,SR       @ $0434   T0 := 0
	dut.u_l1.mem[27] = 16'h7C55;   // K18 MOVEQ #$55,D6    @ $0436
	dut.u_l1.mem[28] = 16'h4E71;   // NOP
	dut.u_l1.mem[576] = 16'h7607;  // sub MOVEQ #7,D3      @ $0880
	dut.u_l1.mem[577] = 16'h4E75;  // K9  RTS              @ $0882

	// trace handler @ $0800, D2 scratch
	dut.u_l1.mem[512] = 16'h5287;  // ADDQ.L #1,D7
	dut.u_l1.mem[513] = 16'h242F;  // MOVE.L 8(A7),D2     address field
	dut.u_l1.mem[514] = 16'h0008;
	dut.u_l1.mem[515] = 16'h2AC2;  // MOVE.L D2,(A5)+
	dut.u_l1.mem[516] = 16'h242F;  // MOVE.L 2(A7),D2     PC field
	dut.u_l1.mem[517] = 16'h0002;
	dut.u_l1.mem[518] = 16'h2AC2;  // MOVE.L D2,(A5)+
	dut.u_l1.mem[519] = 16'h7400;  // MOVEQ #0,D2
	dut.u_l1.mem[520] = 16'h3417;  // MOVE.W (A7),D2      SR
	dut.u_l1.mem[521] = 16'h2AC2;  // MOVE.L D2,(A5)+
	dut.u_l1.mem[522] = 16'h7400;  // MOVEQ #0,D2
	dut.u_l1.mem[523] = 16'h342F;  // MOVE.W 6(A7),D2     format/vector word
	dut.u_l1.mem[524] = 16'h0006;
	dut.u_l1.mem[525] = 16'h2AC2;  // MOVE.L D2,(A5)+
	dut.u_l1.mem[526] = 16'h4E73;  // RTE

	dut.u_l1.mem[544] = 16'h7A42;  // TRAP #1 handler @ $0840: MOVEQ #$42,D5
	dut.u_l1.mem[545] = 16'h4E73;  // RTE

	dut.u_l1.mem[3602] = 16'h0000; dut.u_l1.mem[3603] = 16'h0800;   // vector 9
	dut.u_l1.mem[3650] = 16'h0000; dut.u_l1.mem[3651] = 16'h0840;   // vector 33
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp     = 32'h0000_0600;
	dut.u_cpu.u_regfile.areg[5] = 32'h0000_0A00;   // log pointer

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D7 (trace entries)", dut.u_cpu.u_regfile.dreg[7], N_TRACE);
	for (e = 0; e < N_TRACE; e = e + 1)
		for (f = 0; f < 4; f = f + 1)
			if (log_word[4*e + f] !== want_log(e, f)) begin
				errors = errors + 1;
				$display("FAIL: trace %0d field %0d (%0s) = %h, expected %h", e + 1, f,
				         f == 0 ? "address" : f == 1 ? "PC" : f == 2 ? "SR" : "fmt/vec",
				         log_word[4*e + f], want_log(e, f));
			end
	if (dut.u_cpu.u_regfile.dreg[7] > N_TRACE)
		$display("      unexpected entry %0d: address %h PC %h SR %h", N_TRACE + 1,
		         log_word[4*N_TRACE], log_word[4*N_TRACE+1], log_word[4*N_TRACE+2]);
	check32("D1 (neither poison ran)",     dbg_d1,                32'h0000_0002);
	check32("D3 (subroutine ran)",         dut.u_cpu.u_regfile.dreg[3], 32'h0000_0007);
	check32("D4 (DBF pair counted down)",  dut.u_cpu.u_regfile.dreg[4], 32'h0000_FFFF);
	check32("D5 (TRAP handler ran)",       dut.u_cpu.u_regfile.dreg[5], 32'h0000_0042);
	check32("D6 (end reached)",            dut.u_cpu.u_regfile.dreg[6], 32'h0000_0055);
	check32("ISP",                         dut.u_cpu.u_regfile.isp,     32'h0000_0600);
	check32("SR at the end",               {16'd0, dut.u_cpu.sr},       32'h0000_2700);

	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid ||
	    dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
		errors = errors + 1;
		$display("FAIL: a stage is still valid after the program should have drained");
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
