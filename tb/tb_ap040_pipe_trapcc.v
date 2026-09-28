//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 74: TRAPcc)               //
//                                                                          //
// tb_ap040_pipe_trapcc.v - conditional trap, vector 7                      //
//                                                                          //
// TRAPcc traps to vector 7 if its condition holds and does nothing         //
// otherwise. The .W and .L forms carry an immediate the hardware ignores;  //
// it is left on the stack for the handler.                                 //
//                                                                          //
// The program, with a handler that counts into D2 and returns:             //
//                                                                          //
//   Z := 1  ;  TRAPNE  (false)  ;  TRAPEQ  (true)                          //
//   Z := 0  ;  TRAPEQ.W #$1234 (false)  ;  TRAPNE.L #$12345678 (true)      //
//   MOVEQ #$2A,D1                                                          //
//   V := 1 by an overflowing ADD  ;  DIVU.W #7,D3 clears it (long stall)   //
//   TRAPVS (false)  ;  TRAPVC (true)  ;  TRAPT (true)                      //
//                                                                          //
// Checks: D2 = 4, D1 = $2A, D0 = 7, A7 back at $0600.                      //
//                                                                          //
// Each check's meaning, and which RTL changes it is sensitive to, is       //
// recorded in doc_AP040_PIPELINE_PLAN.md from mutation runs -- not here,   //
// and not before they were run.                                            //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_trapcc;

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
	dut.u_l1.mem[1   ] = 16'h7400;   // MOVEQ #0,D2   trap counter
	dut.u_l1.mem[2   ] = 16'h7005;   // MOVEQ #5,D0
	dut.u_l1.mem[3   ] = 16'h0C80;   // CMPI.L #5,D0   Z := 1
	dut.u_l1.mem[4   ] = 16'h0000;
	dut.u_l1.mem[5   ] = 16'h0005;
	dut.u_l1.mem[6   ] = 16'h56FC;   // TRAPNE        Z=1: condition false
	dut.u_l1.mem[7   ] = 16'h57FC;   // TRAPEQ        Z=1: condition true         (trap 1)
	dut.u_l1.mem[8   ] = 16'h7007;   // MOVEQ #7,D0
	dut.u_l1.mem[9   ] = 16'h0C80;   // CMPI.L #5,D0   Z := 0
	dut.u_l1.mem[10  ] = 16'h0000;
	dut.u_l1.mem[11  ] = 16'h0005;
	dut.u_l1.mem[12  ] = 16'h57FA;   // TRAPEQ.W #$1234   false; one extension word
	dut.u_l1.mem[13  ] = 16'h1234;
	dut.u_l1.mem[14  ] = 16'h56FB;   // TRAPNE.L #$12345678   true; two extension words   (trap 2)
	dut.u_l1.mem[15  ] = 16'h1234;
	dut.u_l1.mem[16  ] = 16'h5678;
	dut.u_l1.mem[17  ] = 16'h722A;   // MOVEQ #$2A,D1
	dut.u_l1.mem[18  ] = 16'h7664;   // MOVEQ #100,D3   dividend
	dut.u_l1.mem[19  ] = 16'h283C;   // MOVE.L #$7FFFFFFF,D4
	dut.u_l1.mem[20  ] = 16'h7FFF;
	dut.u_l1.mem[21  ] = 16'hFFFF;
	dut.u_l1.mem[22  ] = 16'hD884;   // ADD.L D4,D4    V := 1, the last flag-setter before the divide
	dut.u_l1.mem[23  ] = 16'h86FC;   // DIVU.W #7,D3   V := 0; EX stalls ~32 cycles
	dut.u_l1.mem[24  ] = 16'h0007;
	dut.u_l1.mem[25  ] = 16'h59FC;   // TRAPVS        V=0 after the divide: false
	dut.u_l1.mem[26  ] = 16'h58FC;   // TRAPVC        V=0: true                    (trap 3)
	dut.u_l1.mem[27  ] = 16'h50FC;   // TRAPT         always                       (trap 4)
	dut.u_l1.mem[28  ] = 16'h4E71;   // NOP
	dut.u_l1.mem[512 ] = 16'h5282;   // handler: ADDQ.L #1,D2
	dut.u_l1.mem[513 ] = 16'h4E73;   // RTE
	dut.u_l1.mem[3598] = 16'h0000;   // vector 7 -> $0800
	dut.u_l1.mem[3599] = 16'h0800;

end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// Three exception frames are pushed and never popped (the handler
	// returns with JMP, not RTE), so A7 must point somewhere real and
	// clear of the program. See tb_ap040_pipe_move_mem.v's header for why
	// the poke has to land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d2 !== 32'h0000_0004) begin
		errors = errors + 1;
		$display("FAIL: trap count D2 = %h, expected 00000004", dbg_d2);
	end
	if (dbg_d1 !== 32'h0000_002A) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000002a", dbg_d1);
	end
	if (dbg_d0 !== 32'h0000_0007) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000007", dbg_d0);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600", dut.u_cpu.u_regfile.isp);
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
