//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 108: STOP)          //
//                                                                          //
// tb_ap040_pipe_stop.v - privileged, and then actually stopped             //
//                                                                          //
// One opcode, $4E72, and 1,048,576 rounds of the cputest corpus -- the      //
// largest single item left after milestone 107 closed Scc, and 20.5% of    //
// everything this core could not decode.                                   //
//                                                                          //
// STOP is an immediate-to-SR with two differences: the SR is REPLACED       //
// rather than OR/AND/EOR'd, and afterwards the processor stops until an     //
// interrupt. It rides the immsr machinery for the first part, which is      //
// what puts it in eac_is_priv_capable for free.                            //
//                                                                          //
// Both halves are tested, because the corpus only ever exercises one of     //
// them. Every judged STOP round is a USER-mode one, where the whole         //
// instruction is a privilege violation and nothing stops at all -- so the   //
// supervisor half has no corpus coverage whatsoever and exists only here.   //
//                                                                          //
//   user mode:  STOP #$2700  -> vector 8, and SR must be UNCHANGED         //
//   supervisor: STOP #$2500  -> SR = $2500, and nothing runs afterwards    //
//                                                                          //
// The stacked SR is what proves the first one. ap040_core.v:6029 decides    //
// go_priv before it even fetches the extension word, so a user-mode STOP    //
// must not have written SR by the time it faults. D4 holds the frame's      //
// first longword, and its top half must be the USER SR of $0000 -- if the   //
// SR write happened first it reads $2700 instead, and the privilege check   //
// would be running after the damage.                                       //
//                                                                          //
// D5 is what proves the second, and the branch after it is why the proof    //
// holds. A single poison instruction is not enough: a mutation removing the //
// fetch stall left this bench passing, because the flush killed that one    //
// instruction and the core then ran on to a terminal BRA.B -2 and span      //
// there -- "flushed once" reading exactly like "stopped". The poison now    //
// ends in BRA.B -4, back to itself, so a machine that is still issuing      //
// anything at all executes it. Only a stopped one leaves D5 at zero.        //
//                                                                          //
// This core has no interrupt input, so "stops until an interrupt" is        //
// "stops until reset". That is the correct behaviour for a machine with     //
// nothing pending rather than a shortcut, and the bench asserts the stop    //
// rather than working around it.                                           //
//                                                                          //
// Vector 8 (privilege violation) is at word index 3600, four vectors past   //
// vector 4's 3592 in the L1 window's (address - PC_RESET) >> 1 map.         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stop;

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
wire [31:0] dbg_d2, dbg_d3, dbg_d4, dbg_d5;
wire [31:0] dbg_commits;
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

	.dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5),
	.dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr), .dbg_commits(dbg_commits)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000900,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0900;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP    (A7 = $900)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[ 7] = 16'h46C1;   // MOVE D1,SR      (S -> 0, user mode)
	dut.u_l1.mem[ 8] = 16'h4E72;   // STOP #$2700     -> privilege violation
	dut.u_l1.mem[ 9] = 16'h2700;
	dut.u_l1.mem[10] = 16'h7666;   // MOVEQ #$66,D3 (poison: the STOP must trap)
	dut.u_l1.mem[11] = 16'h60FE;   // BRA.B -2

	// Privilege-violation handler @ word idx 384 (byte $700). Supervisor
	// again here, so the second half of the test runs from inside it.
	dut.u_l1.mem[384] = 16'h7411;  // MOVEQ #$11,D2   the handler ran
	dut.u_l1.mem[385] = 16'h2817;  // MOVE.L (A7),D4  the stacked {SR, PC_hi}
	dut.u_l1.mem[386] = 16'h4E72;  // STOP #$2500     supervisor: load SR, stop
	dut.u_l1.mem[387] = 16'h2500;
	dut.u_l1.mem[388] = 16'h7A66;  // MOVEQ #$66,D5 (poison: must NOT run)
	dut.u_l1.mem[389] = 16'h60FC;  // BRA.B -4  -- back to the poison, forever

	// Vector 8 (privilege violation) -> $700.
	dut.u_l1.mem[3600] = 16'h0000;
	dut.u_l1.mem[3601] = 16'h0700;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d2 !== 32'h0000_0011) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000011 (a user-mode STOP must raise a privilege violation and run its handler)",
		         dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (the instruction after a trapping STOP must not run)",
		         dbg_d3);
	end
	if (dbg_d4[31:16] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: stacked SR = %h, expected 0000 (the privilege check must happen BEFORE the SR write; 2700 means the user-mode STOP loaded SR and faulted afterwards)",
		         dbg_d4[31:16]);
	end
	if (dbg_sr !== 16'h2500) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected 2500 (a supervisor STOP replaces SR with its immediate)",
		         dbg_sr);
	end
	if (dbg_d5 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000000 (the machine must STOP; 00000066 means it loaded SR and carried on)",
		         dbg_d5);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
