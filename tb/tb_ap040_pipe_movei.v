//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 28: MOVE #imm,Dn)   //
//                                                                          //
// tb_ap040_pipe_movei.v - the same gather, a different field layout        //
//                                                                          //
// Source mode 111 reg 100 is the immediate addressing mode, so this reuses  //
// milestone 26's gather wholesale: one extension word for byte and word,    //
// two for long. What differs is where the other fields live. MOVE's size    //
// is ir[13:12] with its own mapping (01=B, 11=W, 10=L), not ir[7:6] with    //
// std_size's, and its destination is ir[11:9], not ir[2:0]. Both of those   //
// are exactly the kind of thing that silently produces a plausible wrong    //
// answer, so both are checked here.                                         //
//                                                                          //
// Program:                                                                  //
//                                                                          //
//   1: MOVE.L #$DEADBEEF,D0   203C DEAD BEEF   D0 = DEADBEEF                //
//   4: MOVEQ #-1,D1           72FF             D1 = FFFFFFFF                //
//   5: MOVE.W #$1234,D1       323C 1234        D1 = FFFF1234                //
//   7: MOVEQ #-1,D2           74FF             D2 = FFFFFFFF                //
//   8: MOVE.B #$5A,D2         143C 005A        D2 = FFFFFF5A                //
//                                                                          //
// MOVE.L is the two-word gather and would be unreachable if either word     //
// were dropped. MOVE.W and MOVE.B are the size mapping: read with           //
// std_size's table instead of MOVE's, 11 means an invalid size and 01       //
// means Word rather than Byte, so the two would swap and D1 and D2 would    //
// each hold the other's shape. Both start all-ones so the preserved upper   //
// bits are visible -- FFFF1234 and FFFFFF5A differ from a Long-sized        //
// decode's 00001234 and 0000005A in the half a careless check would skip.   //
//                                                                          //
// The destination field is checked by construction: each instruction names  //
// a different register, so reading it from ir[2:0] would send all three     //
// writes to D4 and leave D0, D1 and D2 at their starting values.            //
//                                                                          //
// On milestone 27's RTL none of the three decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movei;

localparam PROG_WORDS      = 20;
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

initial begin
	#1;
	dut.u_l1.mem[1] = 16'h203C;   // MOVE.L #$DEADBEEF,D0
	dut.u_l1.mem[2] = 16'hDEAD;
	dut.u_l1.mem[3] = 16'hBEEF;
	dut.u_l1.mem[4] = 16'h72FF;   // MOVEQ #-1,D1
	dut.u_l1.mem[5] = 16'h323C;   // MOVE.W #$1234,D1
	dut.u_l1.mem[6] = 16'h1234;
	dut.u_l1.mem[7] = 16'h74FF;   // MOVEQ #-1,D2
	dut.u_l1.mem[8] = 16'h143C;   // MOVE.B #$5A,D2
	dut.u_l1.mem[9] = 16'h005A;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 24) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'hDEAD_BEEF) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected DEADBEEF (MOVE.L must gather both words, high first)", dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_1234) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected FFFF1234 (MOVE.W: ir[13:12]=11 is Word, and D1[31:16] survives)", dbg_d1);
	end
	if (dbg_d2 !== 32'hFFFF_FF5A) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected FFFFFF5A (MOVE.B: ir[13:12]=01 is Byte, and D2[31:8] survives)", dbg_d2);
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
