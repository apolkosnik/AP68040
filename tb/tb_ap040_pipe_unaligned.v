//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 85)                 //
//                                                                          //
// tb_ap040_pipe_unaligned.v - Word accesses at odd addresses               //
//                                                                          //
// A 68040 performs unaligned accesses in hardware; this core took the      //
// high half of the returned longword whatever the address said, so a Word  //
// at an odd address read the byte BEFORE the one asked for. The            //
// differential against the FSM core found it as a one-byte shift           //
// (milestone 84), which is what it looks like from the outside.            //
//                                                                          //
// A Word at an odd address is still inside the longword port B returns --  //
// bytes 1 and 2 of it -- so the load costs a lane select and the store a   //
// byte-enable pattern (0110), with no extra access either way. A Byte is   //
// never misaligned. A LONG at an odd address spans three words and is NOT  //
// covered here or by the RTL: it is the next milestone.                    //
//                                                                          //
// Memory, before:                                                          //
//   $0800: 1122 3344 5566        $0900: 5566 7788      $0A00: 0102 0304    //
//                                                                          //
//   MOVEA.L #$0801,A0                                                      //
//   MOVE.W (A0),D0        bytes $0801,$0802     -> D0 = $00002233          //
//   MOVE.W (A0)+,D1       the same, A0 -> $0803 -> D1 = $00002233          //
//   MOVE.W (A0),D2        bytes $0803,$0804     -> D2 = $00004455          //
//   MOVE.L #$0000ABCD,D3                                                   //
//   MOVEA.L #$0901,A4                                                      //
//   MOVE.W D3,(A4)        bytes $0901,$0902     -> $0900 = $55AB,          //
//                                                   $0902 = $CD88          //
//   MOVEA.L #$0A01,A5                                                      //
//   ADD.W D3,(A5)         read-modify-write at an odd address:             //
//                         $0203 + $ABCD = $ADD0 -> $0A00 = $01AD,          //
//                                                  $0A02 = $D004           //
//                                                                          //
// Each check is a byte the old code would have got wrong: the loads by     //
// one byte, the stores by landing a byte early, and the RMW by both.       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_unaligned;

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
	poke(32'h0400, 16'h207C); poke(32'h0402, 16'h0000); poke(32'h0404, 16'h0801);
	poke(32'h0406, 16'h3010);                      // MOVE.W (A0),D0
	poke(32'h0408, 16'h3218);                      // MOVE.W (A0)+,D1
	poke(32'h040A, 16'h3410);                      // MOVE.W (A0),D2
	poke(32'h040C, 16'h263C); poke(32'h040E, 16'h0000); poke(32'h0410, 16'hABCD);
	poke(32'h0412, 16'h287C); poke(32'h0414, 16'h0000); poke(32'h0416, 16'h0901);
	poke(32'h0418, 16'h3883);                      // MOVE.W D3,(A4)
	poke(32'h041A, 16'h2A7C); poke(32'h041C, 16'h0000); poke(32'h041E, 16'h0A01);
	poke(32'h0420, 16'hD755);                      // ADD.W D3,(A5)
	poke(32'h0422, 16'h4E71);                      // NOP

	poke(32'h0800, 16'h1122); poke(32'h0802, 16'h3344); poke(32'h0804, 16'h5566);
	poke(32'h0900, 16'h5566); poke(32'h0902, 16'h7788);
	poke(32'h0A00, 16'h0102); poke(32'h0A02, 16'h0304);
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D0 (odd Word load)",                 dbg_d0, 32'h0000_2233);
	check32("D1 (odd Word load, postincrement)",  dbg_d1, 32'h0000_2233);
	check32("A0 after the postincrement",         dut.u_cpu.u_regfile.areg[0], 32'h0000_0803);
	check32("D2 (odd Word load across the pair)", dbg_d2, 32'h0000_4455);

	check32("[$0900] (odd Word store, first byte)",  {16'd0, peek(32'h0900)}, 32'h0000_55AB);
	check32("[$0902] (odd Word store, second byte)", {16'd0, peek(32'h0902)}, 32'h0000_CD88);

	check32("[$0A00] (odd Word read-modify-write)",  {16'd0, peek(32'h0A00)}, 32'h0000_01AD);
	check32("[$0A02] (odd Word read-modify-write)",  {16'd0, peek(32'h0A02)}, 32'h0000_D004);

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
