//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: MOVEC to a     //
// stack pointer, read back at once)                                        //
//                                                                          //
// tb_ap040_pipe_movecsp.v - the auxiliary write port has no bypass         //
//                                                                          //
// USP, ISP and MSP are written from two places. An ordinary A7 result      //
// goes through the register file's main write port, which every read has   //
// a same-cycle bypass for. MOVEC's write goes through the auxiliary port   //
// instead -- a direct write to one of the three, bypassing the A7 index    //
// entirely -- and the reads have no bypass for that one.                   //
//                                                                          //
// So an instruction that reads A7 in the cycle a MOVEC to the ACTIVE stack //
// pointer commits sees the old value. Two instructions later it sees the   //
// new one, which is what makes this easy to miss: the value is not lost,   //
// it is late.                                                              //
//                                                                          //
//   MOVE.L #$1000,D0 ; MOVEC D0,ISP      the value to be superseded        //
//   MOVE.L #$1200,D0 ; MOVEC D0,ISP      the one that must be read back    //
//   MOVE.L A7,D1                         immediately, in supervisor mode   //
//                                                                          //
// D1 must be $1200. $1000 is the defect -- the previous ISP, still in the  //
// register file because MOVEC's write had not been made visible yet.       //
//                                                                          //
// The first MOVEC is not decoration. Reading back a stack pointer that was //
// never anything else cannot tell a working bypass from a lucky reset      //
// value, so the register is deliberately given a value to be wrong with.   //
//                                                                          //
// A7 is the active stack pointer, ISP here, so this is also the sequence a //
// supervisor uses to move its own stack and then touch it -- a push        //
// immediately after would go to the old address.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movecsp;

localparam PROG_WORDS      = 32;
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
wire [31:0] dbg_d1;
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

	.dbg_d1 (dbg_d1),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00001200,D0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h1200;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D0,ISP
	dut.u_l1.mem[10] = 16'h0804;
	dut.u_l1.mem[11] = 16'h220F;   // MOVE.L A7,D1   (immediately)
	dut.u_l1.mem[12] = 16'h4E71;   // NOP
	dut.u_l1.mem[13] = 16'h4E71;   // NOP (drain)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 100) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.isp !== 32'h0000_1200) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00001200 (the second MOVEC's value)", dut.u_cpu.u_regfile.isp);
	end
	if (dbg_d1 !== 32'h0000_1200) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00001200 (00001000 means MOVE.L A7,D1 read the register file before MOVEC's write to the ACTIVE stack pointer was made visible to it)",
		         dbg_d1);
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
