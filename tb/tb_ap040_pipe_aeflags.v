//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 114: the flags a    //
// deferred address error stacks)                                           //
//                                                                          //
// tb_ap040_pipe_aeflags.v - a held instruction's fault must not leak       //
//                                                                          //
// A deferred address error -- the one an odd exception vector owes, and    //
// the one an RTE to an odd PC owes -- is carried by the instruction the    //
// redirect reached, which must not execute. ae_susp stops its memory and   //
// register effects. Its FLAG effects got through: sr_faulted applied a     //
// CHK's N/C or a zero divide's C whenever those faults were true, and the  //
// address error that outranks them stacked the result (review 12, finding  //
// 1). A frame whose SR was $2715 said $2714.                               //
//                                                                          //
// Both sources, one program. SR = $2715 (X, Z, C set), ISP = $1000,        //
// D0 = $11223344, D1 = 0:                                                  //
//                                                                          //
//   TRAP #0 -> vector 32 = $601 (odd)                                      //
//   $600: DIVU.W D1,D0     the alias: a zero divide, which clears C        //
//   -> address error, frame at $FEC: SR must be $2715                      //
//   $700 (vector 3), first time: push {format 0, PC $641, SR $2715}, RTE  //
//   $640: CHK.W #-1,D0     the RTE's odd target: a CHK that traps and     //
//                          clears C                                        //
//   -> address error, frame at $FE0: SR must be the restored $2715         //
//   $700, second time: stop. D4 counts entries; D0 must be untouched.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_aeflags;

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
wire [31:0] dbg_d0, dbg_d4;
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
	.dbg_d0 (dbg_d0), .dbg_d4 (dbg_d4), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
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
	dut.u_l1.mem[ 5] = 16'h203C;  dut.u_l1.mem[ 6] = 16'h1122;  dut.u_l1.mem[ 7] = 16'h3344;  // MOVE.L #$11223344,D0
	dut.u_l1.mem[ 8] = 16'h7200;                                                             // MOVEQ #0,D1
	dut.u_l1.mem[ 9] = 16'h46FC;  dut.u_l1.mem[10] = 16'h2715;                               // MOVE #$2715,SR
	dut.u_l1.mem[11] = 16'h4E40;                                                             // TRAP #0
	dut.u_l1.mem[12] = 16'h60FE;                                                             // BRA.B -2

	// $600: the odd vector's alias. $640: the odd RTE's target.
	dut.u_l1.mem[256] = 16'h80C1;                                // DIVU.W D1,D0   (D1 = 0)
	dut.u_l1.mem[257] = 16'h60FE;
	dut.u_l1.mem[288] = 16'h41BC;  dut.u_l1.mem[289] = 16'hFFFF;  // CHK.W #-1,D0
	dut.u_l1.mem[290] = 16'h60FE;

	// $700: vector 3.
	dut.u_l1.mem[384] = 16'h5284;                                // $700 ADDQ.L #1,D4
	dut.u_l1.mem[385] = 16'h0C84;  dut.u_l1.mem[386] = 16'h0000;  // $702 CMPI.L #1,D4
	dut.u_l1.mem[387] = 16'h0001;
	dut.u_l1.mem[388] = 16'h6610;                                // $708 BNE.B -> $71A
	dut.u_l1.mem[389] = 16'h3F3C;  dut.u_l1.mem[390] = 16'h0000;  // $70A MOVE.W #$0000,-(A7)  format 0
	dut.u_l1.mem[391] = 16'h2F3C;  dut.u_l1.mem[392] = 16'h0000;  // $70E MOVE.L #$00000641,-(A7)
	dut.u_l1.mem[393] = 16'h0641;
	dut.u_l1.mem[394] = 16'h3F3C;  dut.u_l1.mem[395] = 16'h2715;  // $714 MOVE.W #$2715,-(A7)
	dut.u_l1.mem[396] = 16'h4E73;                                // $718 RTE
	dut.u_l1.mem[397] = 16'h60FE;                                // $71A BRA.B -2

	// Vector 3 -> $700, vector 32 -> $601.
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0700;
	dut.u_l1.mem[3648] = 16'h0000;  dut.u_l1.mem[3649] = 16'h0601;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 800) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0002) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000002 (two address-error entries)", dbg_d4);
	end
	if (dbg_d0 !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11223344 (neither held instruction executes)", dbg_d0);
	end
	// Odd vector: the TRAP's frame at $FF8, the address error's at $FEC.
	chk("vector AE frame $FEC (SR)",     1526, 16'h2715);
	chk("vector AE frame $FF2 (fmt)",    1529, 16'h200C);
	chk("vector AE frame $FF6 (addr)",   1531, 16'h0600);
	// Odd RTE: popped back to $FEC, the address error's frame at $FE0.
	chk("RTE AE frame $FE0 (SR)",        1520, 16'h2715);
	chk("RTE AE frame $FE4 (PC low)",    1522, 16'h0718);
	chk("RTE AE frame $FE6 (fmt)",       1523, 16'h200C);
	chk("RTE AE frame $FEA (addr)",      1525, 16'h0640);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
