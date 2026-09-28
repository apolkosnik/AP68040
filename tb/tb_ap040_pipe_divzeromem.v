//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 52: divide by zero) //
//                                                                          //
// tb_ap040_pipe_divzeromem.v - the vector-5 exception                        //
//                                                                          //
// A zero divisor is a real exception, not a flag, and it is the first one  //
// in this core that comes from an operand VALUE in the ordinary            //
// instruction stream rather than from an opcode or a privilege state.      //
//                                                                          //
// It is detected in ap040_ea_fetch.v rather than in the divider, for the   //
// same reason an odd JMP target is: the operand is already in hand there,  //
// and that stage owns the frame push and the vector-table read. So the     //
// divider downstream never has to consider a zero divisor at all, and the  //
// whole feature is a trigger wire plus one entry in the vector mux.        //
//                                                                          //
//   MOVE.L #100,D0 / MOVE.L #0,D1                                          //
//   DIVU.W D1,D0     -> vector 5                                           //
//   MOVEQ  #$63,D0   -- poison, must never run                             //
//                                                                          //
//   handler @ $0800:  MOVEQ #$2A,D2                                        //
//                                                                          //
// Three things are checked, and the poison is why the first two are not    //
// enough on their own:                                                     //
//                                                                          //
//   D2 = 2A says the handler ran, so the exception was taken and vectored  //
//     through entry 5 specifically -- entry 4 (illegal) points elsewhere.  //
//   D0 = 64 says the divide wrote nothing. A divider that ran with a zero  //
//     divisor and then trapped would have left something else here.        //
//   D0 = 64 ALSO says the poison never ran, which is what distinguishes a  //
//     real exception from a decoder that quietly skipped the instruction.  //
//                                                                          //
// Word index N is byte address $400 + 2N and the vector table sits at      //
// VBR = $2000, so vector 5 lands at word index 3594. The handler is far    //
// past the mainline because PROG_WORDS is a fetch budget rather than an    //
// address bound -- see tb_ap040_pipe_ansrc_illegal.v.                      //
//                                                                          //
// On milestone 51's RTL the DIVU does not decode, so the ILLEGAL vector is //
// taken instead and D2 stays zero.                                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_divzeromem;

localparam PROG_WORDS      = 32;
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
	// Mainline
	dut.u_l1.mem[1]  = 16'h203C;   // MOVE.L #$00000064,D0  (100)
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0064;
	dut.u_l1.mem[4]  = 16'h227C;   // MOVEA.L #$00000480,A1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0480;
	dut.u_l1.mem[7]  = 16'h80D1;   // DIVU.W (A1),D0  -- divisor word at $0480 is ZERO
	dut.u_l1.mem[8]  = 16'h7063;   // MOVEQ #$63,D0  -- poison, must not run

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 0000FFFF: the WORD is zero
	dut.u_l1.mem[65] = 16'hFFFF;

	// Zero-divide handler @ word idx 512 (byte $800)
	dut.u_l1.mem[512] = 16'h742A;  // MOVEQ #$2A,D2

	// Vector table: vector 5 -> $800 (word idx 3594/3595)
	dut.u_l1.mem[3594] = 16'h0000;
	dut.u_l1.mem[3595] = 16'h0800;
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

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d2 !== 32'h0000_002A) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 0000002a (the vector-5 handler must run)", dbg_d2);
	end
	if (dbg_d0 !== 32'h0000_0064) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000064 (the divide must write nothing; 00000063 means the poison ran)",
		         dbg_d0);
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
