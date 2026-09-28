//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 99: an indexed    //
// JMP that faults)                                                         //
//                                                                          //
// tb_ap040_pipe_jmpidxodd.v - which address the frame identifies           //
//                                                                          //
// An odd JMP target has stacked the instruction's address plus two since   //
// milestone 17, and for the modes that existed then that is right. An      //
// INDEXED one is different: it has resolved its extension word against the //
// real program counter before it faults, so the counter has already moved  //
// past that word and the frame reads pc + 6.                               //
//                                                                          //
// rtl/ap040/ap040_core.v spells the rule out -- ea mode 110 and the        //
// PC-indexed form give pc_i + 6, everything else pc_i + 2 -- and says the  //
// values are what its own corpus group records. It is the core that passes //
// the cputest corpus, so it is the one to match.                           //
//                                                                          //
//   A0 = $0800 ; D0 = 0 ; JMP (1,A0,D0.W)   -> $0801, odd                  //
//                                                                          //
// The JMP sits at $0414, so its frame must carry $041A. $0416 is the       //
// answer for every non-indexed mode and the one this core gave.            //
//                                                                          //
// The format and vector word and the address field are checked too: a      //
// six-word frame, vector 3, and the target with bit 0 cleared. Getting the //
// PC field alone right while the rest drifted would not be worth much.     //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_jmpidxodd;

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
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000600,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0600;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (A7 = $0600)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h207C;   // MOVEA.L #$00000800,A0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h0800;
	dut.u_l1.mem[ 9] = 16'h7000;   // MOVEQ #0,D0
	dut.u_l1.mem[10] = 16'h4EF0;   // JMP (1,A0,D0.W)   @ $0414
	dut.u_l1.mem[11] = 16'h0001;   //   -> $0801, odd
	dut.u_l1.mem[12] = 16'h7466;   // MOVEQ #$66,D2 (poison: must not run)
	dut.u_l1.mem[13] = 16'h4E71;

	// Address-error handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2, to itself

	// Vector 3 -> $700.
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0700;

	// The frame lands at $05F4: twelve bytes below $0600.
	dut.u_l1.mem[250] = 16'h9999;  dut.u_l1.mem[251] = 16'h9999;
	dut.u_l1.mem[252] = 16'h9999;  dut.u_l1.mem[253] = 16'h9999;
	dut.u_l1.mem[254] = 16'h9999;  dut.u_l1.mem[255] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the address-error handler must run)", dbg_d3);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000 (the instruction after the JMP must not run)", dbg_d2);
	end
	if (dut.u_l1.mem[252] !== 16'h041A) begin
		errors = errors + 1;
		$display("FAIL: the frame's PC field = %h, expected 041a. An INDEXED JMP has already resolved its extension word against the program counter when it faults, so the frame reads pc + 6; 0416 is pc + 2, the answer for every other mode.",
		         dut.u_l1.mem[252]);
	end
	if (dut.u_l1.mem[253] !== 16'h200C) begin
		errors = errors + 1;
		$display("FAIL: the frame's format and vector word = %h, expected 200c (a six-word frame, vector 3)", dut.u_l1.mem[253]);
	end
	if ({dut.u_l1.mem[254], dut.u_l1.mem[255]} !== 32'h0000_0800) begin
		errors = errors + 1;
		$display("FAIL: the frame's address field = %h%h, expected 00000800 (the target with bit 0 cleared)",
		         dut.u_l1.mem[254], dut.u_l1.mem[255]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_05F4) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 000005f4 (one twelve-byte frame below $0600)", dut.u_cpu.u_regfile.isp);
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
