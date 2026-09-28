//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 100: an RTE that  //
// returns to an odd address)                                               //
//                                                                          //
// tb_ap040_pipe_rteodd.v - the fault comes AFTER the return, not instead   //
//                                                                          //
// An RTE whose frame carries an odd PC does not fault where it stands. It  //
// restores the status register, pops the frame, and THEN takes the         //
// address error -- so the error frame carries the RESTORED status          //
// register and sits below the popped frame, not twelve bytes below where   //
// the RTE started. rtl/ap040/ap040_core.v says so in as many words: "the   //
// odd restored PC is detected after the RTE has committed its SR".         //
//                                                                          //
// Two consequences, and this bench checks both:                            //
//                                                                          //
//   ISP = $1000, frame restores SR $0015 and PC $0601                       //
//     RTE pops eight -> ISP $1008 -> the error frame lands at $0FFC and    //
//     stacks $0015. Faulting first puts it at $0FF4 stacking $2700.         //
//                                                                          //
//   ISP = $1100, MSP = $2000, frame restores SR $3000 and PC $0601          //
//     The restored M bit is what chooses the stack: the error frame lands  //
//     on MSP at $1FF4 and ISP is left at $1108. Faulting first puts it on  //
//     ISP, twelve below $1100.                                             //
//                                                                          //
// The frame's PC field is the RTE's own address (pc_i in the reference)    //
// and its address field is the target with bit 0 cleared. The handler     //
// counts and returns through JMP (A2), so the second case runs after the   //
// first without an RTE of its own.                                         //
//                                                                          //
// The word at $0600 -- the one an odd fetch of $0601 brings back -- is a   //
// TRAP #0. That instruction is held behind the completed RTE while the     //
// debt is collected, and it must not be allowed to take ITS exception:     //
// the fault belongs to the RTE. A vector-32 handler poisons D1 if it ever  //
// runs. Without this word the held instruction is a harmless NOP and the   //
// exclusion has nothing to be right about.                                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rteodd;

localparam PROG_WORDS      = 48;
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
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h1000;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h203C;   // MOVE.L #$00002000,D0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h2000;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D0,MSP   (MSP = $2000)
	dut.u_l1.mem[10] = 16'h0803;
	dut.u_l1.mem[11] = 16'h247C;   // MOVEA.L #$00000420,A2   (resume 1)
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0420;
	dut.u_l1.mem[14] = 16'h4E73;   // RTE   @ $041C  -- restores $0015, PC $0601
	dut.u_l1.mem[15] = 16'h7466;   // MOVEQ #$66,D2 (poison)
	dut.u_l1.mem[16] = 16'h203C;   // MOVE.L #$00001100,D0   (resume 1)
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h1100;
	dut.u_l1.mem[19] = 16'h4E7B;   // MOVEC D0,ISP   (ISP = $1100)
	dut.u_l1.mem[20] = 16'h0804;
	dut.u_l1.mem[21] = 16'h247C;   // MOVEA.L #$00000434,A2   (resume 2)
	dut.u_l1.mem[22] = 16'h0000;
	dut.u_l1.mem[23] = 16'h0434;
	dut.u_l1.mem[24] = 16'h4E73;   // RTE   @ $0430  -- restores $3000, PC $0601
	dut.u_l1.mem[25] = 16'h7877;   // MOVEQ #$77,D4 (poison)
	dut.u_l1.mem[26] = 16'h60FE;   // resume 2: BRA.B -2, to itself

	// Address-error handler @ word idx 384 (byte $700).
	dut.u_l1.mem[384] = 16'h5283;  // ADDQ.L #1,D3
	dut.u_l1.mem[385] = 16'h4ED2;  // JMP (A2)

	// Vector 3 -> $700.
	dut.u_l1.mem[3590] = 16'h0000;  dut.u_l1.mem[3591] = 16'h0700;

	// What the odd fetch brings back: TRAP #0 at $0600, and its handler at
	// $0780 poisons D1. Vector 32 -> $780.
	dut.u_l1.mem[256]  = 16'h4E40;  // TRAP #0
	dut.u_l1.mem[448]  = 16'h7255;  // MOVEQ #$55,D1
	dut.u_l1.mem[449]  = 16'h4ED2;  // JMP (A2)
	dut.u_l1.mem[3648] = 16'h0000;  dut.u_l1.mem[3649] = 16'h0780;

	// The first frame, at $1000: SR $0015, PC $00000601, format $0.
	dut.u_l1.mem[1536] = 16'h0015;  dut.u_l1.mem[1537] = 16'h0000;
	dut.u_l1.mem[1538] = 16'h0601;  dut.u_l1.mem[1539] = 16'h0000;
	// Where a fault-first RTE would put its frame: $0FF4.
	dut.u_l1.mem[1530] = 16'h9999;  dut.u_l1.mem[1531] = 16'h9999;
	// $0FFC, where the error frame belongs.
	dut.u_l1.mem[1534] = 16'h9999;  dut.u_l1.mem[1535] = 16'h9999;

	// The second frame, at $1100: SR $3000 (S and M), PC $00000601.
	dut.u_l1.mem[1664] = 16'h3000;  dut.u_l1.mem[1665] = 16'h0000;
	dut.u_l1.mem[1666] = 16'h0601;  dut.u_l1.mem[1667] = 16'h0000;
	// $1FF4, where the second error frame belongs, on MSP.
	dut.u_l1.mem[3578] = 16'h9999;  dut.u_l1.mem[3579] = 16'h9999;
	dut.u_l1.mem[3580] = 16'h9999;  dut.u_l1.mem[3581] = 16'h9999;
	dut.u_l1.mem[3582] = 16'h9999;  dut.u_l1.mem[3583] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_0002) begin
		errors = errors + 1;
		$display("FAIL: the handler ran %0d times, expected 2 (one address error per RTE)", dbg_d3);
	end
	if (dbg_d1 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000000. The TRAP at the odd address was fetched and held while the RTE's address error was collected, and it took ITS exception instead of, or as well as, the RTE's.",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0000 || dbg_d4 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D2/D4 = %h/%h, expected 0/0 (the instruction after each RTE must not run)", dbg_d2, dbg_d4);
	end

	// ---- case 1: the frame sits below the POPPED frame and carries the
	// ---- RESTORED status register
	if (dut.u_l1.mem[1534] !== 16'h0015 || dut.u_l1.mem[1535] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: [$0FFC] = %h%h, expected 00150000. The RTE restores its SR and pops BEFORE it faults, so the error frame starts eight bytes higher than a fault-first RTE would put it and carries the RESTORED status register.",
		         dut.u_l1.mem[1534], dut.u_l1.mem[1535]);
	end
	if (dut.u_l1.mem[1536] !== 16'h041C || dut.u_l1.mem[1537] !== 16'h200C) begin
		errors = errors + 1;
		$display("FAIL: [$1000] = %h%h, expected 041c200c (the RTE's own address, then format $2 vector 3)",
		         dut.u_l1.mem[1536], dut.u_l1.mem[1537]);
	end
	if (dut.u_l1.mem[1538] !== 16'h0000 || dut.u_l1.mem[1539] !== 16'h0600) begin
		errors = errors + 1;
		$display("FAIL: [$1004] = %h%h, expected 00000600 (the odd target with bit 0 cleared)",
		         dut.u_l1.mem[1538], dut.u_l1.mem[1539]);
	end
	if (dut.u_l1.mem[1530] !== 16'h9999) begin
		errors = errors + 1;
		$display("FAIL: [$0FF4] = %h, expected 9999 untouched (a frame here means the RTE faulted BEFORE it popped)",
		         dut.u_l1.mem[1530]);
	end

	// ---- case 2: the restored M bit chooses the stack
	if (dut.u_l1.mem[3578] !== 16'h3000 || dut.u_l1.mem[3579] !== 16'h0000) begin
		errors = errors + 1;
		$display("FAIL: [$1FF4] = %h%h, expected 30000000. The RTE restored M, so the error frame belongs on the MASTER stack, twelve below $2000.",
		         dut.u_l1.mem[3578], dut.u_l1.mem[3579]);
	end
	if (dut.u_l1.mem[3580] !== 16'h0430 || dut.u_l1.mem[3581] !== 16'h200C) begin
		errors = errors + 1;
		$display("FAIL: [$1FF8] = %h%h, expected 0430200c", dut.u_l1.mem[3580], dut.u_l1.mem[3581]);
	end
	if (dut.u_cpu.u_regfile.msp !== 32'h0000_1FF4) begin
		errors = errors + 1;
		$display("FAIL: MSP = %h, expected 00001ff4", dut.u_cpu.u_regfile.msp);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_1108) begin
		errors = errors + 1;
		$display("FAIL: ISP = %h, expected 00001108 (popped by eight and then left alone: the second frame went on MSP)",
		         dut.u_cpu.u_regfile.isp);
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
