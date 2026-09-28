//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 93: a faulting     //
// (A7)+ in supervisor mode)                                                //
//                                                                          //
// tb_ap040_pipe_faultpisup.v - both A7 writes on the SAME stack            //
//                                                                          //
// Milestone 92 gave the address-register write port its own stack bank,    //
// which fixed the user-mode case: there the postincrement means USP and    //
// the exception's new stack pointer means ISP, and two banks is all it     //
// takes to keep them apart.                                                //
//                                                                          //
// In supervisor mode they are the SAME register, and separating the banks  //
// achieves nothing. Two writes to ISP in one cycle, and the later one in   //
// the register file's own order wins -- which is the postincrement, not    //
// the exception.                                                           //
//                                                                          //
//   ISP = $1000, the word there is zero                                    //
//   DIVU.W (A7)+,D0     divisor 0 -> vector 5, a twelve-byte frame         //
//                                                                          //
// Architecturally the increment happens first and the frame is pushed from //
// where it leaves the pointer: $1000 + 2 = $1002, then -12, so ISP must be //
// $0FF6. Two things have to be true for that, and they are separate        //
// defects: the frame's base has to be the INCREMENTED pointer, and the     //
// exception's write has to outrank the increment's when both name the same //
// register.                                                                //
//                                                                          //
// The WORD at $0FF4 is checked as well. It is where the frame's first      //
// beat lands if the base is taken before the increment. Only the word,     //
// not the longword: the frame that belongs here starts at $0FF6, so it     //
// covers $0FF6 onwards and the word below it must survive untouched.       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_faultpisup;

localparam PROG_WORDS      = 40;
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
wire [31:0] dbg_d1, dbg_d2, dbg_d3;
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

	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_sr (dbg_sr),
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
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00000064,D0  (the dividend)
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h0064;
	dut.u_l1.mem[ 9] = 16'h80DF;   // DIVU.W (A7)+,D0   divisor at $1000 = 0
	dut.u_l1.mem[10] = 16'h7466;   // MOVEQ #$66,D2 (poison)
	dut.u_l1.mem[11] = 16'h4E71;   // NOP

	// Divide-by-zero handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h7633;  // MOVEQ #$33,D3
	dut.u_l1.mem[385] = 16'h4E71;  // NOP

	// Vector 5 -> $700.
	dut.u_l1.mem[3594] = 16'h0000;
	dut.u_l1.mem[3595] = 16'h0700;

	// The divisor at $1000, and a sentinel where a pre-increment base
	// would put the frame.
	dut.u_l1.mem[1536] = 16'h0000;  dut.u_l1.mem[1537] = 16'h0000;
	dut.u_l1.mem[1530] = 16'h5555;  dut.u_l1.mem[1531] = 16'h5555;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0033) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000033 (the divide-by-zero handler must run)", dbg_d3);
	end
	if (dbg_d2 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000000 (the instruction after the faulting one must not run)", dbg_d2);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0FF6) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00000ff6 ((A7)+ leaves $1002, then a twelve-byte frame; 00001002 means the increment overwrote the exception's own result, 00000ff4 means the frame was based on the pre-increment pointer)",
		         dut.u_cpu.u_regfile.isp);
	end
	if (dut.u_l1.mem[1530] !== 16'h5555) begin
		errors = errors + 1;
		$display("FAIL: the word at $0FF4 = %h, expected 5555 (the frame was based on the pointer BEFORE the postincrement, so it started two bytes low)",
		         dut.u_l1.mem[1530]);
	end
	if (writes !== 3) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted, expected 3 (a format $2 frame is three beats)", writes);
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
