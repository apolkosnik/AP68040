//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 38: sized stores)   //
//                                                                          //
// tb_ap040_pipe_szstore.v - writing part of a longword                     //
//                                                                          //
// Stores were Long only because the L1's write buffer drained a whole      //
// longword. Sized stores need byte enables, which is the first change to    //
// ap040_pipe_l1.v since milestone 12 -- and a small one, because a          //
// behavioural array writes a half-word as easily as a whole one, and a real //
// block RAM has byte enables anyway. The plan had recorded this as work to  //
// defer until the real cache arrives; that was over-cautious.               //
//                                                                          //
// The read-after-write forward had to change with it. It returned the       //
// buffered longword whole, which is right when all four lanes are written   //
// and fabricates three of them when one is. It now merges the buffer over   //
// memory lane by lane.                                                      //
//                                                                          //
// Memory at $0480 starts as AAAA BBBB.                                      //
//                                                                          //
//   MOVEA.L #$0480,A0 / #$0481,A1                                           //
//   MOVE.L  #$11223344,D0                                                   //
//   MOVE.B  D0,(A1)      byte 44 -> $0481, giving AA44 BBBB                 //
//   MOVE.W  D0,(A0)      word 3344 -> $0480, giving 3344 BBBB               //
//   MOVE.L  (A0),D2      read the whole longword back                       //
//                                                                          //
// The two stores overlap deliberately. The byte lands inside the word the   //
// next store overwrites, so the final memory proves the WORD store wrote    //
// exactly two bytes: had it written four, $0482 would no longer be BBBB.    //
// And $0482 staying BBBB through both is the check that a sized store does  //
// not disturb lanes it does not name -- the failure byte enables exist to   //
// prevent.                                                                  //
//                                                                          //
// The byte store uses an ODD address so it exercises the low lane of the    //
// high word rather than the aligned case.                                   //
//                                                                          //
// D2 reads back through the core, so a load and a store have to agree about //
// the layout, and it also exercises the merged forward if the store is      //
// still buffered when the load issues.                                      //
//                                                                          //
// On milestone 37's RTL neither sized store decodes.                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_szstore;

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
	dut.u_l1.mem[4]  = 16'h227C;   // MOVEA.L #$00000481,A1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0481;
	dut.u_l1.mem[7]  = 16'h203C;   // MOVE.L #$11223344,D0
	dut.u_l1.mem[8]  = 16'h1122;
	dut.u_l1.mem[9]  = 16'h3344;
	dut.u_l1.mem[10] = 16'h1280;   // MOVE.B D0,(A1)
	dut.u_l1.mem[11] = 16'h3080;   // MOVE.W D0,(A0)
	dut.u_l1.mem[12] = 16'h2410;   // MOVE.L (A0),D2

	dut.u_l1.mem[64] = 16'hAAAA;
	dut.u_l1.mem[65] = 16'hBBBB;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 44) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// The word store overwrote the byte store's lane; both wrote inside
	// mem[64] and neither may touch mem[65].
	if (dut.u_l1.mem[64] !== 16'h3344) begin
		errors = errors + 1;
		$display("FAIL: memory at $0480 = %h, expected 3344 (the Word store)", dut.u_l1.mem[64]);
	end
	if (dut.u_l1.mem[65] !== 16'hBBBB) begin
		errors = errors + 1;
		$display("FAIL: memory at $0482 = %h, expected BBBB (a sized store must not touch lanes it does not name)",
		         dut.u_l1.mem[65]);
	end
	if (dbg_d2 !== 32'h3344_BBBB) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 3344BBBB (read-back of the partially written longword)", dbg_d2);
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
