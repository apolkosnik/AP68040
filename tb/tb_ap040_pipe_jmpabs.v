//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 106: JMP to an      //
// absolute address writes no register)                                     //
//                                                                          //
// tb_ap040_pipe_jmpabs.v - the target is a destination, not a result       //
//                                                                          //
// The absolute EA forms all ride held_is_abs, and the decoder then sorts   //
// out what each one does with the address it gathered. Four sub-kinds are  //
// named: held_abs_lea delivers it to An, held_abs_push pushes it,          //
// held_abs_jsr pushes a return address, and held_abs_jmp only branches.    //
//                                                                          //
// id_writes_ccr excludes all four. id_writes_reg excluded three:           //
//                                                                          //
//   id_writes_reg <= ... || (held_is_abs && (!held_abs_alu ||              //
//                                            !held_alu_nowrite)) || ...    //
//                                                                          //
// A JMP is not an ALU operation, so held_abs_alu is 0, so the term is      //
// true and JMP committed a register write it has no result for. The        //
// register it wrote is not arbitrary: the absolute path takes its          //
// destination from the opcode's bits [11:9], and JMP's encoding fixes      //
// those at 111, so both forms write the TARGET ADDRESS into D7.            //
//                                                                          //
// The cputest corpus found it, 1,280 rounds of Basic/JMP across 4ef8 and   //
// 4ef9, each reading back a D7 holding the address it had just jumped to   //
// where the oracle expects the register untouched.                         //
//                                                                          //
//   MOVEQ #$55,D7           the value that must survive                    //
//   JMP $0500.W             -> MOVEQ #$33,D3                               //
//   JMP $00000600.L         -> MOVEQ #$44,D4                               //
//                                                                          //
// Both forms run, because they are separate decode paths that reach the    //
// same term, and a fix that only covered the one the corpus happened to    //
// print first would leave the other. D7 must still be $55 at the end; on   //
// the RTL this was written for it is $600, the second target, having been  //
// overwritten twice.                                                       //
//                                                                          //
// Each JMP is followed by a MOVEQ into D2 that must never execute, so a    //
// run where the branch did not happen at all cannot pass by leaving D7     //
// alone. D3 and D4 prove each jump LANDED -- without them a core that      //
// treated both JMPs as no-ops would satisfy the D7 check for free. Every   //
// path ends in BRA.B -2 rather than running off the end of the program.    //
//                                                                          //
// The bench then runs the two absolute sub-kinds that DO write a register, //
// PEA (xxx).L and JSR (xxx).L, because the fix is an exclusion from a term //
// those share and an exclusion is exactly the kind of fix that over-        //
// reaches. A mutation proved this was needed: moving the exclusion from    //
// held_abs_jmp to held_abs_push left tb_ap040_pipe_pea, _jsr and _lea all  //
// passing, so nothing in the suite covered the absolute PEA's A7 write.    //
// D5 is what PEA pushed and D6 is A7 after it; D1 proves the JSR landed    //
// and its RTS came back.                                                   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_jmpabs;

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
wire [31:0] dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
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

	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4),
	.dbg_d5 (dbg_d5), .dbg_d6 (dbg_d6), .dbg_d7 (dbg_d7),
	.dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h7E55;   // MOVEQ #$55,D7
	dut.u_l1.mem[ 2] = 16'h4EF8;   // JMP $0500.W
	dut.u_l1.mem[ 3] = 16'h0500;
	dut.u_l1.mem[ 4] = 16'h7466;   // MOVEQ #$66,D2 (poison: must not run)
	dut.u_l1.mem[ 5] = 16'h60FE;   // BRA.B -2

	// $0500: the word form landed here.
	dut.u_l1.mem[128] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[129] = 16'h4EF9;  // JMP $00000600.L
	dut.u_l1.mem[130] = 16'h0000;
	dut.u_l1.mem[131] = 16'h0600;
	dut.u_l1.mem[132] = 16'h7468;  // MOVEQ #$68,D2 (poison: must not run)
	dut.u_l1.mem[133] = 16'h60FE;  // BRA.B -2

	// $0600: the long form landed here. From here on the bench checks the
	// absolute sub-kinds that DO write a register.
	dut.u_l1.mem[256] = 16'h7844;  // MOVEQ #$44,D4
	dut.u_l1.mem[257] = 16'h203C;  // MOVE.L #$00000900,D0
	dut.u_l1.mem[258] = 16'h0000;
	dut.u_l1.mem[259] = 16'h0900;
	dut.u_l1.mem[260] = 16'h4E7B;  // MOVEC D0,ISP   (A7 = $900)
	dut.u_l1.mem[261] = 16'h0804;
	dut.u_l1.mem[262] = 16'h4879;  // PEA $00000700.L
	dut.u_l1.mem[263] = 16'h0000;
	dut.u_l1.mem[264] = 16'h0700;
	dut.u_l1.mem[265] = 16'h2A17;  // MOVE.L (A7),D5   what PEA pushed
	dut.u_l1.mem[266] = 16'h2C0F;  // MOVE.L A7,D6     A7 after the push
	dut.u_l1.mem[267] = 16'h4EB9;  // JSR $00000800.L
	dut.u_l1.mem[268] = 16'h0000;
	dut.u_l1.mem[269] = 16'h0800;
	dut.u_l1.mem[270] = 16'h60FE;  // BRA.B -2

	// $0800: the JSR's target.
	dut.u_l1.mem[512] = 16'h7222;  // MOVEQ #$22,D1
	dut.u_l1.mem[513] = 16'h4E75;  // RTS
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 120) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (JMP $0500.W must land)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0044) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000044 (JMP $00000600.L must land)", dbg_d4);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000 (the instruction after a JMP must not run)", dbg_d2);
	end
	if (dbg_d7 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D7 = %h, expected 00000055 (JMP has no result and must write no register; 00000600 is the long form's target committed as one, 00000500 the word form's)",
		         dbg_d7);
	end

	if (dbg_d5 !== 32'h0000_0700) begin
		errors = errors + 1;
		$display("FAIL: D5 = %h, expected 00000700 (PEA (xxx).L must push the address it gathered)", dbg_d5);
	end
	if (dbg_d6 !== 32'h0000_08FC) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 000008fc (PEA (xxx).L must also WRITE A7, four below $900; 00000900 means the exclusion that stops JMP writing a register reached PEA too)",
		         dbg_d6);
	end
	if (dbg_d1 !== 32'h0000_0022) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000022 (JSR (xxx).L must land and its RTS must return)", dbg_d1);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
