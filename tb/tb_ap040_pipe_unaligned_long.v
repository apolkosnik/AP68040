//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 86)                 //
//                                                                          //
// tb_ap040_pipe_unaligned_long.v - Longwords at odd addresses              //
//                                                                          //
// The bytes of a Long at an odd address span THREE words, so no lane       //
// select out of one aligned longword can reach them -- which is why        //
// milestone 85 fixed Words and left this. The fix is the port, not a       //
// wider mux: port B carries a size now, the memory does all placement, and //
// ap040_pipe_l1.v assembles from however many words it takes.              //
//                                                                          //
// Memory, before:                                                          //
//   $0B00: 1234 5678 9ABC        $0C00: 0001 0002 0003                     //
//                                                                          //
//   MOVEA.L #$0B01,A6                                                      //
//   MOVE.L (A6),D4        bytes $0B01..$0B04 -> D4 = $3456789A             //
//   MOVE.L #$11223344,D5                                                   //
//   MOVE.L D5,(A6)        -> $0B00 = $1211, $0B02 = $2233, $0B04 = $44BC   //
//   MOVEA.L #$0C01,A4                                                      //
//   ADD.L D4,(A4)         read-modify-write across three words:            //
//                         $01000200 + $3456789A = $35567A9A                //
//                         -> $0C00 = $0035, $0C02 = $567A, $0C04 = $9A03   //
//   MOVE.L (A6)+,D3       reads back what the store wrote, A6 -> $0B05     //
//                                                                          //
// Every check is a byte in the middle word, which is the one no aligned    //
// access can touch: on milestone-85 RTL the loads return the aligned       //
// longword and the stores land a byte early.                               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_unaligned_long;

localparam PROG_WORDS      = 64;
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
	.clk (clk), .nreset (nreset), .ce  (ce),

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

// The array is indexed from PC_RESET; this bench thinks in byte addresses.
task poke;
	input [31:0] addr;
	input [15:0] word;
	begin dut.u_l1.mem[(addr - PC_RESET) >> 1] = word; end
endtask

function [15:0] peek;
	input [31:0] addr;
	peek = dut.u_l1.mem[(addr - PC_RESET) >> 1];
endfunction

initial begin
	#1;
	poke(32'h0400, 16'h2C7C); poke(32'h0402, 16'h0000); poke(32'h0404, 16'h0B01);
	poke(32'h0406, 16'h2816);                      // MOVE.L (A6),D4
	poke(32'h0408, 16'h2A3C); poke(32'h040A, 16'h1122); poke(32'h040C, 16'h3344);
	poke(32'h040E, 16'h2C85);                      // MOVE.L D5,(A6)
	poke(32'h0410, 16'h287C); poke(32'h0412, 16'h0000); poke(32'h0414, 16'h0C01);
	poke(32'h0416, 16'hD994);                      // ADD.L D4,(A4)
	poke(32'h0418, 16'h261E);                      // MOVE.L (A6)+,D3
	poke(32'h041A, 16'h4E71);                      // NOP

	poke(32'h0B00, 16'h1234); poke(32'h0B02, 16'h5678); poke(32'h0B04, 16'h9ABC);
	poke(32'h0C00, 16'h0001); poke(32'h0C02, 16'h0002); poke(32'h0C04, 16'h0003);
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D4 (odd Long load, across three words)",
	        dut.u_cpu.u_regfile.dreg[4], 32'h3456_789A);

	check32("[$0B00] (odd Long store, first byte)",  {16'd0, peek(32'h0B00)}, 32'h0000_1211);
	check32("[$0B02] (odd Long store, middle word)", {16'd0, peek(32'h0B02)}, 32'h0000_2233);
	check32("[$0B04] (odd Long store, last byte)",   {16'd0, peek(32'h0B04)}, 32'h0000_44BC);

	check32("[$0C00] (odd Long read-modify-write)",  {16'd0, peek(32'h0C00)}, 32'h0000_0035);
	check32("[$0C02] (odd Long read-modify-write)",  {16'd0, peek(32'h0C02)}, 32'h0000_567A);
	check32("[$0C04] (odd Long read-modify-write)",  {16'd0, peek(32'h0C04)}, 32'h0000_9A03);

	check32("D3 (reading back the odd Long store)",
	        dut.u_cpu.u_regfile.dreg[3], 32'h1122_3344);
	check32("A6 after the postincrement", dut.u_cpu.u_regfile.areg[6], 32'h0000_0B05);

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
