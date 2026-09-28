//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 41: ALU on (An)+/-) //
//                                                                          //
// tb_ap040_pipe_aluax.v - the binary ALU family with autoincrement modes   //
//                                                                          //
// Modes 011 and 100 finish the source side of the family milestones 39 and //
// 40 opened. They are the loop idiom: ADD.L (A0)+,D0 walking an array is    //
// what a 68k compiler emits for a summation, and it is one instruction      //
// where the FSM core needs several.                                        //
//                                                                          //
// They cost one shape term and two flags, because the address-register      //
// update already exists: ap040_ea_fetch.v drives it off eac_is_postinc/     //
// eac_is_predec through the second write port milestone 30 added for        //
// MOVE.L (An)+,Dn.                                                         //
//                                                                          //
// The interesting case is CMP. Its an_write is independent of writes_reg,   //
// which was true before this milestone but never mattered, because every    //
// earlier user of an_write also wrote a data register. CMP.L (A0)+,D0       //
// writes NO data register and must still advance A0 -- so this is the       //
// first instruction where the two can be told apart, and a plausible        //
// implementation that gated the address update on "does this instruction    //
// write a register" would pass every earlier bench and fail here.           //
//                                                                          //
// Memory: $0480 = 3, $0484 = 5, $0488 = 7.                                 //
//                                                                          //
//   MOVEA.L #$0480,A0 / MOVE.L #3,D0                                       //
//   CMP.L   (A0)+,D0    3 - 3, writes nothing, must still advance A0        //
//   ADD.L   (A0)+,D0    + 5 = 8     <- only if the CMP advanced A0          //
//   ADD.L   (A0)+,D0    + 7 = 0F                                           //
//   MOVEA.L #$0488,A1 / MOVE.L #$20,D1                                     //
//   SUB.L   -(A1),D1    - 5 = 1B                                           //
//   SUB.L   -(A1),D1    - 3 = 18                                           //
//   MOVE.L  #3,D2                                                          //
//   CMP.L   (A1)+,D2    3 - 3, sets Z and writes nothing                    //
//                                                                          //
// D0 = 0F is the chained check. Had the CMP not advanced A0, the two ADDs   //
// would read $0480 and $0484 and leave 0B; had the step been 2 rather than  //
// 4, the second read would straddle two longwords and leave something       //
// wildly different. Three instructions share one register precisely so      //
// that no single one of them can be right by accident.                      //
//                                                                          //
// D1 = 18 chains the predecrement the same way: the first SUB must land on  //
// $0484 (A1 - 4, not A1) and the second on $0480.                          //
//                                                                          //
// The final CMP reads through A1 rather than a fresh register, so D2 and Z  //
// together also confirm the SECOND predecrement left A1 at $0480.           //
//                                                                          //
// On milestone 40's RTL none of the six autoincrement forms decode.        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_aluax;

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
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0480;
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$00000003,D0
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0003;
	dut.u_l1.mem[7]  = 16'hB098;   // CMP.L (A0)+,D0
	dut.u_l1.mem[8]  = 16'hD098;   // ADD.L (A0)+,D0
	dut.u_l1.mem[9]  = 16'hD098;   // ADD.L (A0)+,D0
	dut.u_l1.mem[10] = 16'h227C;   // MOVEA.L #$00000488,A1
	dut.u_l1.mem[11] = 16'h0000;
	dut.u_l1.mem[12] = 16'h0488;
	dut.u_l1.mem[13] = 16'h223C;   // MOVE.L #$00000020,D1
	dut.u_l1.mem[14] = 16'h0000;
	dut.u_l1.mem[15] = 16'h0020;
	dut.u_l1.mem[16] = 16'h92A1;   // SUB.L -(A1),D1
	dut.u_l1.mem[17] = 16'h92A1;   // SUB.L -(A1),D1
	dut.u_l1.mem[18] = 16'h243C;   // MOVE.L #$00000003,D2
	dut.u_l1.mem[19] = 16'h0000;
	dut.u_l1.mem[20] = 16'h0003;
	dut.u_l1.mem[21] = 16'hB499;   // CMP.L (A1)+,D2

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000003
	dut.u_l1.mem[65] = 16'h0003;
	dut.u_l1.mem[66] = 16'h0000;   // $0484 = 00000005
	dut.u_l1.mem[67] = 16'h0005;
	dut.u_l1.mem[68] = 16'h0000;   // $0488 = 00000007
	dut.u_l1.mem[69] = 16'h0007;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_000F) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0000000F (CMP.L (A0)+ must advance A0 though it writes no register)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0018) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000018 (two chained SUB.L -(A1) must read $0484 then $0480)",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0003) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000003 (CMP must not write its result back)", dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. CMP.L of equal operands sets Z alone; the
	// reversed subtraction would give the same Z here, but N and C would
	// differ on unequal operands, so Z is checked together with the fact
	// that D2 survived.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMP.L of equal operands sets Z)", dbg_ccr[3:0]);
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
