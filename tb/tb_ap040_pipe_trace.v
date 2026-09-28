//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 78: instruction trace)    //
//                                                                          //
// tb_ap040_pipe_trace.v - T1 set, and every trace frame logged             //
//                                                                          //
// With T1 set, each instruction is followed by a trace exception: vector   //
// 9, the six-word frame, PC field = the instruction that would have run    //
// next, address field = the instruction just traced, SR = the SR after it. //
// The handler here logs those four fields (address, PC, SR, format/vector  //
// word) as four longwords at (A5)+ and returns; nothing else in the        //
// handler is traced, because the exception entry cleared T1.              //
//                                                                          //
//   MOVE.L #$A700,D0 / MOVE D0,SR       T1 := 1 (not itself traced)        //
//   I1  $040A  MOVEQ #1,D1                                                 //
//   I2  $040C  ADDQ.L #2,D1                                                //
//   I3  $040E  BRA.B I5                 PC field must be the TARGET        //
//   I4  $0410  MOVEQ #$7F,D1            skipped -- poison                  //
//   I5  $0412  MOVE.L D1,(A6)           a store, traced like anything else //
//   I6  $0414  TRAP #1                  NOT traced: on the 68040 no        //
//                                       exception leaves a trace pending   //
//                                       (review 14; see below)             //
//              (TRAP handler: MOVEQ #$42,D2 / RTE -- untraced, T is clear) //
//   I7  $0416  MOVEQ #3,D3              traced again: RTE restored T1      //
//   J1  $0418  MOVEQ #0,D0              a format-$0 frame built by hand:   //
//   J2  $041A  MOVE.W D0,-(A7)            fmt/vec $0000                    //
//   J3  $041C  MOVE.L #$042E,D0                                            //
//   J4  $0422  MOVE.L D0,-(A7)            PC = I8a                         //
//   J5  $0424  MOVE.L #$A700,D0                                            //
//   J6  $042A  MOVE.W D0,-(A7)            SR = $A700                       //
//   J7  $042C  RTE                      a TRACED RTE: PC field = where it  //
//                                       went, SR = what it restored        //
//   I8a $042E  MOVE.L #$2700,D0                                            //
//   I8b $0434  MOVE D0,SR               clears T1 -- still traced, and the //
//                                       stacked SR is $2700, so the trace  //
//                                       handler's RTE returns with tracing //
//                                       off                                //
//   I9  $0436  MOVEQ #$55,D5            not traced                         //
//                                                                          //
// Expected log, fourteen entries of {address, PC, SR, fmt/vec}; the SR    //
// carries each instruction's own CCR result (MOVEQ #0 sets Z, MOVE.W of    //
// $A700 sets N):                                                           //
//    1 {$040A, $040C, $A700}     8 {$041C, $0422, $A700}                   //
//    2 {$040C, $040E, $A700}     9 {$0422, $0424, $A700}                   //
//    3 {$040E, $0412, $A700}    10 {$0424, $042A, $A700}                   //
//    4 {$0412, $0414, $A700}    11 {$042A, $042C, $A708}                   //
//    5 {$0416, $0418, $A700}    12 {$042C, $042E, $A700}   the RTE        //
//    6 {$0418, $041A, $A704}    13 {$042E, $0434, $A700}                   //
//    7 {$041A, $041C, $A704}    14 {$0434, $0436, $2700}   T1 cleared     //
// and fmt/vec $2024 in every one.                                          //
//                                                                          //
// D7 = 14 counts the entries. D1 = 3 (I4 skipped), D2 = $42 (TRAP handler //
// ran), D3 = 3, D5 = $55, [$0B00] = 3 (I5's store landed exactly once),    //
// ISP back at $0600, SR $2700 at the end.                                  //
//                                                                          //
// What the entries prove. 1: the instruction right after the MOVE to SR    //
// is the first traced (the SR write is forwarded from EX). 3: a taken      //
// branch's PC field is its target. Between 4 and 5 the TRAP leaves        //
// nothing. This bench used to expect an entry there -- the handler's      //
// first instruction, traced with the post-entry SR -- which is the 68000/  //
// 68020 rule. The 68040 keeps no trace pending across TRAP, TRAPV, CHK or  //
// divide by zero (WinUAE's exception_check_trace: "Except on 68040 or      //
// 68060"), and ap040_core.v's S_EXC0 arms none; review 14 found the pipe   //
// stacking a second frame after every traced exception. 5: T1 restored by //
// an RTE resumes tracing on the very next instruction. 14: the MOVE to SR  //
// that clears T1 is itself traced, with T1 clear in the frame. 12: a       //
// traced RTE is traced on its own start SR, and the frame carries the SR   //
// it restored and the PC it went to. Entry 4's store landing once says the //
// traced instruction ran exactly once and the held instruction behind it   //
// ran nothing.                                                             //
//                                                                          //
// On milestone-77 RTL there is no trace: D7 = 0, the log is empty.         //
//                                                                          //
// Vectors: n at word index 3584 + 2n. Log at $0A00 (word index 768).       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_trace;

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

// expected log: entry e (0-based), field f (0 address, 1 PC, 2 SR, 3 fmt/vec)
localparam N_TRACE = 14;
reg [31:0] exp_addr [0:N_TRACE-1];
reg [31:0] exp_pc   [0:N_TRACE-1];
reg [15:0] exp_sr   [0:N_TRACE-1];
initial begin
	exp_addr[0]  = 32'h040A; exp_pc[0]  = 32'h040C; exp_sr[0]  = 16'hA700;
	exp_addr[1]  = 32'h040C; exp_pc[1]  = 32'h040E; exp_sr[1]  = 16'hA700;
	exp_addr[2]  = 32'h040E; exp_pc[2]  = 32'h0412; exp_sr[2]  = 16'hA700;   // BRA: PC field is the target
	exp_addr[3]  = 32'h0412; exp_pc[3]  = 32'h0414; exp_sr[3]  = 16'hA700;
	exp_addr[4]  = 32'h0416; exp_pc[4]  = 32'h0418; exp_sr[4]  = 16'hA700;
	exp_addr[5]  = 32'h0418; exp_pc[5]  = 32'h041A; exp_sr[5]  = 16'hA704;   // MOVEQ #0: Z
	exp_addr[6]  = 32'h041A; exp_pc[6]  = 32'h041C; exp_sr[6]  = 16'hA704;   // MOVE.W of 0: Z
	exp_addr[7]  = 32'h041C; exp_pc[7]  = 32'h0422; exp_sr[7]  = 16'hA700;
	exp_addr[8]  = 32'h0422; exp_pc[8]  = 32'h0424; exp_sr[8]  = 16'hA700;
	exp_addr[9]  = 32'h0424; exp_pc[9]  = 32'h042A; exp_sr[9]  = 16'hA700;
	exp_addr[10] = 32'h042A; exp_pc[10] = 32'h042C; exp_sr[10] = 16'hA708;   // MOVE.W of $A700: N
	exp_addr[11] = 32'h042C; exp_pc[11] = 32'h042E; exp_sr[11] = 16'hA700;   // RTE: where it went, what it restored
	exp_addr[12] = 32'h042E; exp_pc[12] = 32'h0434; exp_sr[12] = 16'hA700;
	exp_addr[13] = 32'h0434; exp_pc[13] = 32'h0436; exp_sr[13] = 16'h2700;   // MOVE to SR clearing T1
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

wire [31:0] log_word [0:4*N_TRACE-1];
genvar g;
generate for (g = 0; g < 4*N_TRACE; g = g + 1) begin : lw
	assign log_word[g] = {dut.u_l1.mem[768 + 2*g], dut.u_l1.mem[769 + 2*g]};
end endgenerate

initial begin
	#1;
	dut.u_l1.mem[1 ] = 16'h203C;   // MOVE.L #$0000A700,D0
	dut.u_l1.mem[2 ] = 16'h0000;
	dut.u_l1.mem[3 ] = 16'hA700;
	dut.u_l1.mem[4 ] = 16'h46C0;   // MOVE D0,SR          @ $0408   T1 := 1
	dut.u_l1.mem[5 ] = 16'h7201;   // I1  MOVEQ #1,D1     @ $040A
	dut.u_l1.mem[6 ] = 16'h5481;   // I2  ADDQ.L #2,D1    @ $040C
	dut.u_l1.mem[7 ] = 16'h6002;   // I3  BRA.B I5        @ $040E
	dut.u_l1.mem[8 ] = 16'h727F;   // I4  MOVEQ #$7F,D1   @ $0410   skipped
	dut.u_l1.mem[9 ] = 16'h2C81;   // I5  MOVE.L D1,(A6)  @ $0412
	dut.u_l1.mem[10] = 16'h4E41;   // I6  TRAP #1         @ $0414
	dut.u_l1.mem[11] = 16'h7603;   // I7  MOVEQ #3,D3     @ $0416
	dut.u_l1.mem[12] = 16'h7000;   // J1  MOVEQ #0,D0     @ $0418
	dut.u_l1.mem[13] = 16'h3F00;   // J2  MOVE.W D0,-(A7) @ $041A   fmt/vec word $0000
	dut.u_l1.mem[14] = 16'h203C;   // J3  MOVE.L #$042E,D0 @ $041C
	dut.u_l1.mem[15] = 16'h0000;
	dut.u_l1.mem[16] = 16'h042E;
	dut.u_l1.mem[17] = 16'h2F00;   // J4  MOVE.L D0,-(A7) @ $0422   PC = I8a
	dut.u_l1.mem[18] = 16'h203C;   // J5  MOVE.L #$A700,D0 @ $0424
	dut.u_l1.mem[19] = 16'h0000;
	dut.u_l1.mem[20] = 16'hA700;
	dut.u_l1.mem[21] = 16'h3F00;   // J6  MOVE.W D0,-(A7) @ $042A   SR = $A700
	dut.u_l1.mem[22] = 16'h4E73;   // J7  RTE             @ $042C   traced
	dut.u_l1.mem[23] = 16'h203C;   // I8a MOVE.L #$2700,D0 @ $042E
	dut.u_l1.mem[24] = 16'h0000;
	dut.u_l1.mem[25] = 16'h2700;
	dut.u_l1.mem[26] = 16'h46C0;   // I8b MOVE D0,SR      @ $0434   T1 := 0
	dut.u_l1.mem[27] = 16'h7A55;   // I9  MOVEQ #$55,D5   @ $0436
	dut.u_l1.mem[28] = 16'h4E71;   // NOP

	// trace handler @ $0800: log {address field, PC field, SR, fmt/vec} at (A5)+.
	// D4 is its scratch: the traced program's MOVE D0,SR needs D0 intact
	// across the trace that lands between it and the MOVE.L that loaded D0.
	// (A first draft used D0 and wrote the fmt/vec word into SR.)
	dut.u_l1.mem[512] = 16'h5287;  // ADDQ.L #1,D7
	dut.u_l1.mem[513] = 16'h282F;  // MOVE.L 8(A7),D4     address field
	dut.u_l1.mem[514] = 16'h0008;
	dut.u_l1.mem[515] = 16'h2AC4;  // MOVE.L D4,(A5)+
	dut.u_l1.mem[516] = 16'h282F;  // MOVE.L 2(A7),D4     PC field
	dut.u_l1.mem[517] = 16'h0002;
	dut.u_l1.mem[518] = 16'h2AC4;  // MOVE.L D4,(A5)+
	dut.u_l1.mem[519] = 16'h7800;  // MOVEQ #0,D4
	dut.u_l1.mem[520] = 16'h3817;  // MOVE.W (A7),D4      SR
	dut.u_l1.mem[521] = 16'h2AC4;  // MOVE.L D4,(A5)+
	dut.u_l1.mem[522] = 16'h7800;  // MOVEQ #0,D4
	dut.u_l1.mem[523] = 16'h382F;  // MOVE.W 6(A7),D4     format/vector word
	dut.u_l1.mem[524] = 16'h0006;
	dut.u_l1.mem[525] = 16'h2AC4;  // MOVE.L D4,(A5)+
	dut.u_l1.mem[526] = 16'h4E73;  // RTE

	// TRAP #1 handler @ $0840
	dut.u_l1.mem[544] = 16'h7442;  // MOVEQ #$42,D2
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
	dut.u_cpu.u_regfile.areg[6] = 32'h0000_0B00;   // I5's store target

	repeat ((PROG_WORDS + 1500) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D7 (trace entries)", dut.u_cpu.u_regfile.dreg[7], N_TRACE);
	for (e = 0; e < N_TRACE; e = e + 1)
		for (f = 0; f < 4; f = f + 1)
			if (log_word[4*e + f] !== want_log(e, f)) begin
				errors = errors + 1;
				$display("FAIL: trace %0d field %0d (%0s) = %h, expected %h", e + 1, f,
				         f == 0 ? "address" : f == 1 ? "PC" : f == 2 ? "SR" : "fmt/vec",
				         log_word[4*e + f], want_log(e, f));
			end
	check32("D1", dbg_d1, 32'h0000_0003);
	check32("D2 (TRAP handler ran)", dbg_d2, 32'h0000_0042);
	check32("D3", dut.u_cpu.u_regfile.dreg[3], 32'h0000_0003);
	check32("D5 (untraced tail ran)", dut.u_cpu.u_regfile.dreg[5], 32'h0000_0055);
	check32("[$0B00] (I5's store)", {dut.u_l1.mem[896], dut.u_l1.mem[897]}, 32'h0000_0003);
	check32("ISP", dut.u_cpu.u_regfile.isp, 32'h0000_0600);
	check32("SR at the end", {16'd0, dut.u_cpu.sr}, 32'h0000_2700);

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
