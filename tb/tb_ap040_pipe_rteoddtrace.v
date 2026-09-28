//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 114: a traced RTE  //
// to an odd PC)                                                            //
//                                                                          //
// tb_ap040_pipe_rteoddtrace.v - the address error outranks the trace       //
//                                                                          //
// An RTE that restores an odd PC owes an address error, and an RTE under  //
// T1 or T0 owes a trace. The 68040 takes the address error:               //
// ap040_core.v's S_RTE_FIN2 checks the PC before its traced path. This    //
// core armed the trace on RTE completion regardless, the trace outranks   //
// the deferred address error in exc_vec_num, and vector 9 was taken with  //
// the odd PC in its frame (review 12, finding 2).                         //
// tb_ap040_pipe_oddvectrace.v is the same rule for an odd exception       //
// VECTOR, where review 11 found it; this is the RTE's own completion path. //
//                                                                          //
// Both trace modes, one program, ISP = $1000:                              //
//   push {format 0, PC $601, SR $2715}; ORI #$8000,SR (T1); RTE            //
//     -> address error: the frame at $FF4 carries SR $2715, the RTE's      //
//        address, $200C and $600                                          //
//   $700 (vector 3), first time: push {0, $641, $2715}; ORI #$4000,SR     //
//        (T0 -- an RTE is a change of flow); RTE                          //
//     -> address error: frame at $FE8 with $640                           //
//   $700, second time: stop.                                               //
//   $780 (vector 9): MOVEQ #$99,D5 -- the trace handler, never entered.   //
// The pushes happen BEFORE T is set, so nothing but the RTE is traced.     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rteoddtrace;

localparam PROG_WORDS      = 400;
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
wire [31:0] dbg_d3, dbg_d4, dbg_d5;
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
	.dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5),
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

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h203C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h1000;  // MOVE.L #$1000,D0
	dut.u_l1.mem[ 3] = 16'h4E7B;  dut.u_l1.mem[ 4] = 16'h0804;                               // MOVEC D0,ISP
	dut.u_l1.mem[ 5] = 16'h3F3C;  dut.u_l1.mem[ 6] = 16'h0000;                               // MOVE.W #$0000,-(A7)
	dut.u_l1.mem[ 7] = 16'h2F3C;  dut.u_l1.mem[ 8] = 16'h0000;  dut.u_l1.mem[ 9] = 16'h0601;  // MOVE.L #$601,-(A7)
	dut.u_l1.mem[10] = 16'h3F3C;  dut.u_l1.mem[11] = 16'h2715;                               // MOVE.W #$2715,-(A7)
	dut.u_l1.mem[12] = 16'h007C;  dut.u_l1.mem[13] = 16'h8000;                               // $418 ORI #$8000,SR (T1)
	dut.u_l1.mem[14] = 16'h4E73;                                                             // $41C RTE
	dut.u_l1.mem[15] = 16'h60FE;

	// $600 and $640: the two odd targets' aliases, poisoned.
	dut.u_l1.mem[256] = 16'h7677;  dut.u_l1.mem[257] = 16'h60FE;   // MOVEQ #$77,D3
	dut.u_l1.mem[288] = 16'h7677;  dut.u_l1.mem[289] = 16'h60FE;

	// $700: vector 3.
	dut.u_l1.mem[384] = 16'h5284;                                // $700 ADDQ.L #1,D4
	dut.u_l1.mem[385] = 16'h0C84;  dut.u_l1.mem[386] = 16'h0000;  // $702 CMPI.L #1,D4
	dut.u_l1.mem[387] = 16'h0001;
	dut.u_l1.mem[388] = 16'h6614;                                // $708 BNE.B -> $71E
	dut.u_l1.mem[389] = 16'h3F3C;  dut.u_l1.mem[390] = 16'h0000;  // $70A MOVE.W #$0000,-(A7)
	dut.u_l1.mem[391] = 16'h2F3C;  dut.u_l1.mem[392] = 16'h0000;  // $70E MOVE.L #$641,-(A7)
	dut.u_l1.mem[393] = 16'h0641;
	dut.u_l1.mem[394] = 16'h3F3C;  dut.u_l1.mem[395] = 16'h2715;  // $714 MOVE.W #$2715,-(A7)
	dut.u_l1.mem[396] = 16'h007C;  dut.u_l1.mem[397] = 16'h4000;  // $718 ORI #$4000,SR (T0)
	dut.u_l1.mem[398] = 16'h4E73;                                // $71C RTE
	dut.u_l1.mem[399] = 16'h60FE;                                // $71E BRA.B -2

	// $780: vector 9.
	dut.u_l1.mem[448] = 16'h7A99;  dut.u_l1.mem[449] = 16'h60FE;  // MOVEQ #$99,D5

	// Vector 3 -> $700, vector 9 -> $780.
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0700;
	dut.u_l1.mem[3602] = 16'h0000;  dut.u_l1.mem[3603] = 16'h0780;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 800) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0002) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000002 (both odd RTEs must reach the address-error handler)",
		         dbg_d4);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000 (the trace handler must never be entered)", dbg_d5);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (an odd target's alias must never run)", dbg_d3);
	end
	// Under T1: popped to $1000, the address error's frame at $FF4.
	chk("T1 AE frame $FF4 (SR)",      1530, 16'h2715);
	chk("T1 AE frame $FF8 (PC low)",  1532, 16'h041C);
	chk("T1 AE frame $FFA (fmt)",     1533, 16'h200C);
	chk("T1 AE frame $FFE (addr)",    1535, 16'h0600);
	// Under T0: popped back to $FF4, the frame at $FE8.
	chk("T0 AE frame $FE8 (SR)",      1524, 16'h2715);
	chk("T0 AE frame $FEC (PC low)",  1526, 16'h071C);
	chk("T0 AE frame $FEE (fmt)",     1527, 16'h200C);
	chk("T0 AE frame $FF2 (addr)",    1529, 16'h0640);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
