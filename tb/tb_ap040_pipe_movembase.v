//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 96: MOVEM saving  //
// its own base register)                                                   //
//                                                                          //
// tb_ap040_pipe_movembase.v - the one register a predecrement MOVEM       //
// cannot store unchanged                                                   //
//                                                                          //
// MOVEM.L <list>,-(An) steps An down by one operation size before each     //
// transfer. When the LIST contains An itself, what reaches memory on that  //
// transfer is not the register's entry value: the 68020 through 68040      //
// store the initial value minus one operation size, and the 68000 and      //
// 68010 store it undecremented. rtl/ap040/ap040_core.v, which passes the   //
// cputest corpus, follows the later rule.                                  //
//                                                                          //
//   A7 = $1000 ; MOVEM.L A7,-(A7)                                          //
//                                                                          //
// $0FFC must contain $00000FFC and A7 must end there. $00001000 is the     //
// entry value, which is the 68000's answer and not this core's.            //
//                                                                          //
// A second MOVEM stores A6 and A7 together from A7, so the base is not     //
// the only register in the list and is not the first one transferred --    //
// the rule applies to the base wherever it sits, and a fix that keyed off  //
// the first beat rather than the register would pass the case above.       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movembase;

localparam PROG_WORDS      = 40;
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
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4;
wire [15:0] dbg_sr;
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

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_d4 (dbg_d4), .dbg_sr (dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (A7 = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h48E7;   // MOVEM.L <list>,-(A7)
	dut.u_l1.mem[ 7] = 16'h0001;   //   A7 alone (predecrement order: bit 0 is A7)
	dut.u_l1.mem[ 8] = 16'h2C3C;   // MOVE.L #$A6A6A6A6,D6
	dut.u_l1.mem[ 9] = 16'hA6A6;
	dut.u_l1.mem[10] = 16'hA6A6;
	dut.u_l1.mem[11] = 16'h2C46;   // MOVEA.L D6,A6
	dut.u_l1.mem[12] = 16'h48E7;   // MOVEM.L <list>,-(A7)
	dut.u_l1.mem[13] = 16'h0003;   //   A6 and A7: A7 transferred first, A6 second
	dut.u_l1.mem[14] = 16'h4E71;   // NOP (drain)

	// $0FFC, $0FF8 and $0FF4, where the three pushes land.
	dut.u_l1.mem[1534] = 16'h9999;  dut.u_l1.mem[1535] = 16'h9999;
	dut.u_l1.mem[1532] = 16'h9999;  dut.u_l1.mem[1533] = 16'h9999;
	dut.u_l1.mem[1530] = 16'h9999;  dut.u_l1.mem[1531] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[1534], dut.u_l1.mem[1535]} !== 32'h0000_0FFC) begin
		errors = errors + 1;
		$display("FAIL: $0FFC = %h%h, expected 00000ffc (a predecrement MOVEM storing its own base register stores the initial value MINUS one operation size; 00001000 is the entry value, which is the 68000's answer)",
		         dut.u_l1.mem[1534], dut.u_l1.mem[1535]);
	end
	if ({dut.u_l1.mem[1532], dut.u_l1.mem[1533]} !== 32'h0000_0FF8) begin
		errors = errors + 1;
		$display("FAIL: $0FF8 = %h%h, expected 00000ff8 (the second MOVEM's A7 transfer, with A6 also in the list)",
		         dut.u_l1.mem[1532], dut.u_l1.mem[1533]);
	end
	if ({dut.u_l1.mem[1530], dut.u_l1.mem[1531]} !== 32'hA6A6_A6A6) begin
		errors = errors + 1;
		$display("FAIL: $0FF4 = %h%h, expected a6a6a6a6 (A6 is NOT the base and must be stored unchanged)",
		         dut.u_l1.mem[1530], dut.u_l1.mem[1531]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0FF4) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000ff4 (three longwords pushed from $1000)", dut.u_cpu.u_regfile.isp);
	end

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
