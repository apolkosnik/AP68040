//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 6: folder           //
// independence + Scc.B)                                                    //
//                                                                          //
// tb_ap040_pipe_scc.v - Scc.B byte-merge + writes_ccr split proof         //
//                                                                          //
// Program:                                                                //
//                                                                          //
//   1: MOVEQ #5,D3   (0x7605)  D3 = 5 (positive, nonzero, non-0xFF --      //
//                               distinguishable from both Scc byte fills)  //
//   2: MOVEQ #-1,D1  (0x72FF)  D1 = 0xFFFFFFFF; sets N=1,Z=0,V=0,C=0 --    //
//                               this is the flag state that must survive   //
//                               past the SEQ below                        //
//   3: SEQ D3        (0x57C3)  condition EQ against D1's Z=0 -> FALSE      //
//                                                                          //
// D3, not D1, is deliberately the Scc destination: this decoder's Scc      //
// dest field and its (unused, always-computed-anyway) "src" field are the  //
// SAME opcode bits, so Scc's incidental ALU_MOVE call always reads its own //
// destination register as operand_a. If Scc's destination were the same    //
// register the preceding MOVEQ set (D1), that incidental ALU_MOVE would    //
// coincidentally reproduce the exact same N/Z as the real MOVEQ, masking a //
// broken writes_ccr gate -- confirmed empirically: an early draft of this  //
// test used SEQ D1 and did not catch the mutation below at all. Using a    //
// different destination (D3, value 5: N=0,Z=0) makes a broken gate visible //
// as a genuinely different CCR value (N=0 from D3's incidental computation //
// instead of N=1 from D1's real one), not just an untested code path.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_scc;

localparam PROG_WORDS      = 10;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
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
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

// Override the IF stage's default all-NOP ROM. Scheduled at #1 so it runs
// after ap040_inst_fetch.v's own t=0 fill (standard Verilog idiom).
initial begin
	#1;
	dut.u_l1.mem[1] = 16'h7605;   // MOVEQ #5,D3
	dut.u_l1.mem[2] = 16'h72FF;   // MOVEQ #-1,D1
	dut.u_l1.mem[3] = 16'h57C3;   // SEQ D3
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	// Same margin as the other testbenches.
	repeat (PROG_WORDS + 20) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (upper bytes preserved -- they were already 0 -- low byte cleared -- condition evaluated wrong or byte-merge broken)", dbg_d3);
	end
	// XNZVC: SEQ must not have touched CCR -- still MOVEQ #-1,D1's N=1,Z=0,
	// V=0,C=0, not D3's own incidental N=0,Z=0 (which is what a broken
	// writes_reg/writes_ccr gate would leak -- see file header).
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (Scc incorrectly modified CCR -- writes_ccr split broken)", dbg_ccr[3:0]);
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
