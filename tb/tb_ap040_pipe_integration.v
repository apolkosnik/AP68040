//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (integration, after milestone  //
// 33)                                                                      //
//                                                                          //
// tb_ap040_pipe_integration.v - the milestones running together            //
//                                                                          //
// Every other bench here proves ONE milestone in three to six instructions //
// and is deliberately narrow. None of them exercises what happens when the //
// features run back to back, which is where a pipeline actually fails: a   //
// forwarding path that works when the producer and consumer are the only   //
// two instructions in flight can still be wrong when a third sits between  //
// them, or when the same cycle carries both a primary result and an        //
// address update.                                                          //
//                                                                          //
// This program sums four longwords through (An)+ and stores the result. It //
// is a real dependency chain: every ADD depends on the load immediately    //
// before it, and every load depends on the A0 update from the load before  //
// THAT, so both forward paths are live on almost every cycle and the       //
// second one is driven by a different instruction each time.               //
//                                                                          //
//   MOVEA.L #$0480,A0       load the source pointer -- no poking           //
//   MOVEQ   #0,D0           accumulator                                    //
//   4x { MOVE.L (A0)+,D1 ; ADD.L D1,D0 }                                   //
//   MOVEA.L #$04A0,A1       load the destination pointer                   //
//   MOVE.L  D0,(A1)         store the sum                                  //
//   MOVE.L  (A1),D2         read it back                                   //
//                                                                          //
// Data: 1, 2, 3, 4 at $0480. The values are distinct and ascending so a    //
// load that repeats an address, skips one, or reads the wrong one gives a  //
// sum that is wrong rather than coincidentally right -- 1+2+3+4 = 10, but  //
// four reads of the same word give 4, 8, 12 or 16, and a dropped update    //
// gives 4. Only the correct sequence produces 10.                          //
//                                                                          //
// Milestones covered in one program: 19 (ADD register-register), 26/28     //
// (the gather and immediate source), 30 ((An)+ and the second write port), //
// 31 (the store), 33 (MOVEA both forms).                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_integration;

localparam PROG_WORDS      = 40;
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
	dut.u_l1.mem[4]  = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[5]  = 16'h2218;   // MOVE.L (A0)+,D1
	dut.u_l1.mem[6]  = 16'hD081;   // ADD.L  D1,D0
	dut.u_l1.mem[7]  = 16'h2218;   // MOVE.L (A0)+,D1
	dut.u_l1.mem[8]  = 16'hD081;   // ADD.L  D1,D0
	dut.u_l1.mem[9]  = 16'h2218;   // MOVE.L (A0)+,D1
	dut.u_l1.mem[10] = 16'hD081;   // ADD.L  D1,D0
	dut.u_l1.mem[11] = 16'h2218;   // MOVE.L (A0)+,D1
	dut.u_l1.mem[12] = 16'hD081;   // ADD.L  D1,D0
	dut.u_l1.mem[13] = 16'h227C;   // MOVEA.L #$000004A0,A1
	dut.u_l1.mem[14] = 16'h0000;
	dut.u_l1.mem[15] = 16'h04A0;
	dut.u_l1.mem[16] = 16'h2280;   // MOVE.L D0,(A1)
	dut.u_l1.mem[17] = 16'h2411;   // MOVE.L (A1),D2

	// Source data at $0480: word indices 64..71.
	dut.u_l1.mem[64] = 16'h0000; dut.u_l1.mem[65] = 16'h0001;
	dut.u_l1.mem[66] = 16'h0000; dut.u_l1.mem[67] = 16'h0002;
	dut.u_l1.mem[68] = 16'h0000; dut.u_l1.mem[69] = 16'h0003;
	dut.u_l1.mem[70] = 16'h0000; dut.u_l1.mem[71] = 16'h0004;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 50) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_000A) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 0000000A (1+2+3+4; a repeated or skipped load gives 4, 8, 12 or 16)", dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0004) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000004 (the last element loaded)", dbg_d1);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0490) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000490 (four postincrements from 0480)", dut.u_cpu.u_regfile.areg[0]);
	end
	// $04A0 is word index 80.
	if ({dut.u_l1.mem[80], dut.u_l1.mem[81]} !== 32'h0000_000A) begin
		errors = errors + 1;
		$display("FAIL: memory at $04A0 = %h%h, expected 0000000A",
		         dut.u_l1.mem[80], dut.u_l1.mem[81]);
	end
	if (dbg_d2 !== 32'h0000_000A) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 0000000A (read-back of the stored sum)", dbg_d2);
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
