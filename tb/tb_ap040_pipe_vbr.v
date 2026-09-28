//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 117: VBR)           //
//                                                                          //
// tb_ap040_pipe_vbr.v - exception vectors come from VBR + 4 * vector       //
//                                                                          //
// The vector fetch read 4 * vector and never consulted VBR, which MOVEC    //
// could set. No bench saw it: the ones that took an exception kept VBR at  //
// 0, and tb_ap040_pipe_sup.v set it to $40 but put its table at 0. The     //
// corpus driver's trace rounds are the first to run a handler, and theirs  //
// came from the wrong table.                                               //
//                                                                          //
// Three tables, at 0, $800 and $900; every entry this program must NOT use //
// points at a poison handler ($740) that logs $DEAD and stops.            //
//   MOVEC D1,VBR ($800); TRAP #0      $800 + $80 -> $700: logs $0080       //
//   ILLEGAL                            $800 + $10 -> $720: logs $0010,     //
//                                      steps over it                       //
//   MOVEC D2,VBR ($900); TRAP #1      the TRAP right behind the MOVEC must //
//                                      see $900: $984 -> $700, logs $0084  //
//   TRAP #2                            $988 holds $701, odd: an address    //
//                                      error through $90C -> $760, whose   //
//                                      frame's PC field is 4 * 34 = $88,   //
//                                      WITHOUT the base (ap040_core.v),    //
//                                      and whose address field is $700     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_vbr;

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

	dut.u_l1.mem[  0] = 16'h203C;  dut.u_l1.mem[  1] = 16'h0000;  dut.u_l1.mem[  2] = 16'h1000; // $400 MOVE.L #$1000,D0
	dut.u_l1.mem[  3] = 16'h4E7B;  dut.u_l1.mem[  4] = 16'h0804;                               // $406 MOVEC D0,ISP
	dut.u_l1.mem[  5] = 16'h2A7C;  dut.u_l1.mem[  6] = 16'h0000;  dut.u_l1.mem[  7] = 16'h0A00; // $40A MOVEA.L #$A00,A5          the log
	dut.u_l1.mem[  8] = 16'h223C;  dut.u_l1.mem[  9] = 16'h0000;  dut.u_l1.mem[ 10] = 16'h0800; // $410 MOVE.L #$800,D1
	dut.u_l1.mem[ 11] = 16'h4E7B;  dut.u_l1.mem[ 12] = 16'h1801;                               // $416 MOVEC D1,VBR
	dut.u_l1.mem[ 13] = 16'h4E40;                                                              // $41A TRAP #0                   $800 table: $700
	dut.u_l1.mem[ 14] = 16'h4AFC;                                                              // $41C ILLEGAL                   $800 table: $720
	dut.u_l1.mem[ 15] = 16'h243C;  dut.u_l1.mem[ 16] = 16'h0000;  dut.u_l1.mem[ 17] = 16'h0900; // $41E MOVE.L #$900,D2
	dut.u_l1.mem[ 18] = 16'h4E7B;  dut.u_l1.mem[ 19] = 16'h2801;                               // $424 MOVEC D2,VBR
	dut.u_l1.mem[ 20] = 16'h4E41;                                                              // $428 TRAP #1                   right behind: $900 table
	dut.u_l1.mem[ 21] = 16'h4E42;                                                              // $42A TRAP #2                   $900 table: odd, address error
	dut.u_l1.mem[ 22] = 16'h60FE;                                                              // $42C BRA.B -2

	// $700: log the format/vector word, return.
	dut.u_l1.mem[384] = 16'h3AEF;  dut.u_l1.mem[385] = 16'h0006;   // MOVE.W 6(A7),(A5)+
	dut.u_l1.mem[386] = 16'h4E73;                                  // RTE
	// $720: the same, stepping over the ILLEGAL.
	dut.u_l1.mem[400] = 16'h3AEF;  dut.u_l1.mem[401] = 16'h0006;   // MOVE.W 6(A7),(A5)+
	dut.u_l1.mem[402] = 16'h54AF;  dut.u_l1.mem[403] = 16'h0002;   // ADDQ.L #2,2(A7)
	dut.u_l1.mem[404] = 16'h4E73;                                  // RTE
	// $740: poison.
	dut.u_l1.mem[416] = 16'h3AFC;  dut.u_l1.mem[417] = 16'hDEAD;   // MOVE.W #$DEAD,(A5)+
	dut.u_l1.mem[418] = 16'h60FE;                                  // BRA.B -2
	// $760: the address error -- format/vector, PC field, address field.
	dut.u_l1.mem[432] = 16'h3AEF;  dut.u_l1.mem[433] = 16'h0006;   // MOVE.W 6(A7),(A5)+
	dut.u_l1.mem[434] = 16'h2AEF;  dut.u_l1.mem[435] = 16'h0002;   // MOVE.L 2(A7),(A5)+
	dut.u_l1.mem[436] = 16'h2AEF;  dut.u_l1.mem[437] = 16'h0008;   // MOVE.L 8(A7),(A5)+
	dut.u_l1.mem[438] = 16'h60FE;                                  // BRA.B -2

	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0740;   // $00C: vector 3 -> $740
	dut.u_l1.mem[3592] = 16'h0000;  dut.u_l1.mem[3593] = 16'h0740;   // $010: vector 4 -> $740
	dut.u_l1.mem[3648] = 16'h0000;  dut.u_l1.mem[3649] = 16'h0740;   // $080: vector 32 -> $740
	dut.u_l1.mem[3650] = 16'h0000;  dut.u_l1.mem[3651] = 16'h0740;   // $084: vector 33 -> $740
	dut.u_l1.mem[3652] = 16'h0000;  dut.u_l1.mem[3653] = 16'h0740;   // $088: vector 34 -> $740
	dut.u_l1.mem[518] = 16'h0000;  dut.u_l1.mem[519] = 16'h0740;   // $80C: vector 3 -> $740
	dut.u_l1.mem[520] = 16'h0000;  dut.u_l1.mem[521] = 16'h0720;   // $810: vector 4 -> $720
	dut.u_l1.mem[576] = 16'h0000;  dut.u_l1.mem[577] = 16'h0700;   // $880: vector 32 -> $700
	dut.u_l1.mem[578] = 16'h0000;  dut.u_l1.mem[579] = 16'h0740;   // $884: vector 33 -> $740
	dut.u_l1.mem[580] = 16'h0000;  dut.u_l1.mem[581] = 16'h0740;   // $888: vector 34 -> $740
	dut.u_l1.mem[646] = 16'h0000;  dut.u_l1.mem[647] = 16'h0760;   // $90C: vector 3 -> $760
	dut.u_l1.mem[648] = 16'h0000;  dut.u_l1.mem[649] = 16'h0740;   // $910: vector 4 -> $740
	dut.u_l1.mem[704] = 16'h0000;  dut.u_l1.mem[705] = 16'h0740;   // $980: vector 32 -> $740
	dut.u_l1.mem[706] = 16'h0000;  dut.u_l1.mem[707] = 16'h0700;   // $984: vector 33 -> $700
	dut.u_l1.mem[708] = 16'h0000;  dut.u_l1.mem[709] = 16'h0701;   // $988: vector 34 -> $701

	for (i = 768; i < 790; i = i + 1) dut.u_l1.mem[i] = 16'h0000;  // $A00: the log
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("TRAP #0 through $800",        768, 16'h0080);
	chk("ILLEGAL through $800",        769, 16'h0010);
	chk("TRAP #1 behind MOVEC to VBR", 770, 16'h0084);
	chk("odd vector: fmt/vec",         771, 16'h200C);
	chk("odd vector: PC field hi",     772, 16'h0000);
	chk("odd vector: PC field lo",     773, 16'h0088);
	chk("odd vector: address hi",      774, 16'h0000);
	chk("odd vector: address lo",      775, 16'h0700);
	chk("nothing more logged",         776, 16'h0000);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
