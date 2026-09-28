//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 114: an <ea>,An     //
// whose <ea> steps that An)                                                //
//                                                                          //
// tb_ap040_pipe_anself.v - the source's side effect comes first            //
//                                                                          //
// ADDA.W (A6)+,A6 and its relatives were the last 2,496 wrong rounds in    //
// the corpus. Port B reads the destination An before the source's (An)+    //
// or -(An) has stepped it, and the ALU operated on that stale value. The   //
// 68k evaluates the source effective address first, side effect included, //
// so the destination operand is the STEPPED An:                            //
//                                                                          //
//   ADDA.W (A0)+,A0    A0 = $800, [$800] = $0010:  A0 = $802 + $10 = $812  //
//   SUBA.L -(A1),A1    A1 = $820, [$81C] = 4:      A1 = $81C - 4 = $818    //
//   CMPA.L (A2)+,A2    A2 = $840, [$840] = $844:   $844 - $844, Z set      //
//                                                  (the old A2 gives Z=0)  //
//   MOVEA.W (A3)+,A3   A3 = $860, [$860] = $1234:  the load, not the step, //
//                                                  is what A3 ends up with //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_anself;

localparam PROG_WORDS      = 200;
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
wire [31:0] dbg_d7;
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
	.dbg_d7 (dbg_d7), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;

task chka;
	input [63:0] name;
	input [31:0] got;
	input [31:0] want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", name, got, want);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h207C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h0800;  // MOVEA.L #$800,A0
	dut.u_l1.mem[ 3] = 16'h227C;  dut.u_l1.mem[ 4] = 16'h0000;  dut.u_l1.mem[ 5] = 16'h0820;  // MOVEA.L #$820,A1
	dut.u_l1.mem[ 6] = 16'h247C;  dut.u_l1.mem[ 7] = 16'h0000;  dut.u_l1.mem[ 8] = 16'h0840;  // MOVEA.L #$840,A2
	dut.u_l1.mem[ 9] = 16'h267C;  dut.u_l1.mem[10] = 16'h0000;  dut.u_l1.mem[11] = 16'h0860;  // MOVEA.L #$860,A3
	dut.u_l1.mem[12] = 16'hD0D8;   // ADDA.W  (A0)+,A0
	dut.u_l1.mem[13] = 16'h93E1;   // SUBA.L  -(A1),A1
	dut.u_l1.mem[14] = 16'hB5DA;   // CMPA.L  (A2)+,A2
	dut.u_l1.mem[15] = 16'h57C7;   // SEQ     D7
	dut.u_l1.mem[16] = 16'h365B;   // MOVEA.W (A3)+,A3
	dut.u_l1.mem[17] = 16'h60FE;   // BRA.B -2

	dut.u_l1.mem[512] = 16'h0010;                                // $800
	dut.u_l1.mem[526] = 16'h0000;  dut.u_l1.mem[527] = 16'h0004; // $81C
	dut.u_l1.mem[544] = 16'h0000;  dut.u_l1.mem[545] = 16'h0844; // $840
	dut.u_l1.mem[560] = 16'h1234;                                // $860
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chka("A0", dut.u_cpu.u_regfile.areg[0], 32'h0000_0812);
	chka("A1", dut.u_cpu.u_regfile.areg[1], 32'h0000_0818);
	chka("A2", dut.u_cpu.u_regfile.areg[2], 32'h0000_0844);
	chka("A3", dut.u_cpu.u_regfile.areg[3], 32'h0000_1234);
	if (dbg_d7[7:0] !== 8'hFF) begin
		errors = errors + 1;
		$display("FAIL: SEQ after CMPA.L (A2)+,A2 = %h, expected FF (compared against the stepped A2)",
		         dbg_d7[7:0]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
