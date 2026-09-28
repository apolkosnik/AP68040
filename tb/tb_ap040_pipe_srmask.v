//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 96: the bits the  //
// status register does not have)                                           //
//                                                                          //
// tb_ap040_pipe_srmask.v - writes to SR and CCR are masked                 //
//                                                                          //
// The status register is sixteen bits wide and only eleven of them exist:  //
// T1, T0, S, M, three interrupt-mask bits and five condition codes. The    //
// rest read as zero however they are written. RTE already applied that     //
// mask; MOVE to SR and the ORI/ANDI/EORI immediates did not.               //
//                                                                          //
//   MOVE.L #$2FFF,D0 ; MOVE D0,SR    SR := $271F, not $2FFF                //
//   ANDI #$0000,CCR                  the condition codes go to zero        //
//   ORI  #$00E0,CCR                  and stay there: those bits do not     //
//                                    exist                                 //
//                                                                          //
// The final status register must read $2700. Unmasked it reads $2FE0 --    //
// bit 11, which nothing defines, and three condition-code bits above the   //
// five that are real.                                                      //
//                                                                          //
// The three writes are chained so one check covers all of them, and the    //
// interrupt mask is left at 7 by the first of them: a fix that masked too  //
// hard would clear that too and fail here rather than pass.                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_srmask;

localparam PROG_WORDS      = 32;
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
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00002FFF,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h2FFF;
	dut.u_l1.mem[ 4] = 16'h46C0;   // MOVE D0,SR   -> $271F
	dut.u_l1.mem[ 5] = 16'h023C;   // ANDI #$0000,CCR
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'h003C;   // ORI #$00E0,CCR
	dut.u_l1.mem[ 8] = 16'h00E0;
	dut.u_l1.mem[ 9] = 16'h4E71;   // NOP (drain)
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_sr !== 16'h2700) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected 2700. $2fe0 means neither the MOVE nor the ORI was masked: bit 11 is not a status bit and neither are the three above the condition codes.",
		         dbg_sr);
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
