//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 109: the instruction //
// at an odd RTE target must not happen)                                     //
//                                                                          //
// tb_ap040_pipe_rteoddhold.v - a held instruction that held nothing back    //
//                                                                          //
// An RTE with an odd restored PC arms a deferred address error: the RTE     //
// finishes its pop and restores SR, and the fault is taken afterwards, with //
// the instruction at the odd target never executing. `ae_arm` is what       //
// carries that, and `own_exc` suppressed the held instruction's own         //
// exception triggers.                                                      //
//                                                                          //
// Nothing suppressed anything else. There was no hold BRANCH for it -- the  //
// trace has one, `trace_hold && !exc_active`, where the held instruction    //
// parks doing nothing until EX and WB drain -- so the instruction at the    //
// odd target simply retired. Its store went out, and because `eac_is_store` //
// outranks `exc_writing` in the port-B address mux, the exception's own     //
// frame beats followed it to the store's address:                          //
//                                                                          //
//   ISP $1000, format-0 frame there with SR $2715 and PC $601              //
//   A0 = $800, D0 = $11223344, and MOVE.L D0,(A0) sits at $600             //
//                                                                          //
// All three frame beats landed at $800 instead of $FFC/$1000/$1004, mixed   //
// in with the target's own stores, and the vector read used $800 too, so    //
// the handler was never reached. `ae_arm` is also too SHORT to fix this     //
// with: it clears on `ae_take`, which is the cycle the entry begins, while  //
// the frame writes, the vector read and the redirect all happen after that. //
//                                                                          //
// So the window is `ae_susp`, from the odd RTE to the vector read that ends //
// the entry, and the fix is both halves of what the trace does: the hold    //
// branch, and `!ae_busy` on the request gates.                             //
//                                                                          //
// The memory at $800 is the load-bearing check. $12345678 is neither the    //
// value the store would write nor anything a frame beat contains, so "the   //
// store did not happen" cannot be confused with "it happened somewhere      //
// else". The three frame words are checked at their proper addresses for    //
// the same reason in reverse.                                              //
//                                                                          //
// The target is (A0)+ rather than (A0) so that it has a REGISTER result as  //
// well as a memory one. With the request gates in place but no hold branch  //
// the store is suppressed and a plain (A0) form then looks entirely         //
// innocent -- a mutation proved it -- while the instruction has in fact     //
// still retired. A0 is what notices.                                       //
//                                                                          //
// D4 proves the vector-3 handler ran; D3 proves no OTHER vector's handler   //
// did, which is what the sibling defect in the vector mux would cause.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rteoddhold;

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
wire [31:0] dbg_d0, dbg_d3, dbg_d4;
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
	.dbg_d0 (dbg_d0), .dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;

task chk;
	input [255:0] what;
	input integer widx;
	input  [15:0] want;
	begin
		if (dut.u_l1.mem[widx] !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %04x, expected %04x", what, dut.u_l1.mem[widx], want);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h203C;   // MOVE.L #$00001000,D0
	dut.u_l1.mem[ 1] = 16'h0000;
	dut.u_l1.mem[ 2] = 16'h1000;
	dut.u_l1.mem[ 3] = 16'h4E7B;   // MOVEC D0,ISP    (A7 = $1000)
	dut.u_l1.mem[ 4] = 16'h0804;
	dut.u_l1.mem[ 5] = 16'h207C;   // MOVEA.L #$00000800,A0
	dut.u_l1.mem[ 6] = 16'h0000;
	dut.u_l1.mem[ 7] = 16'h0800;
	dut.u_l1.mem[ 8] = 16'h203C;   // MOVE.L #$11223344,D0
	dut.u_l1.mem[ 9] = 16'h1122;
	dut.u_l1.mem[10] = 16'h3344;
	dut.u_l1.mem[11] = 16'h4E73;   // RTE   -- restores PC $601, which is ODD

	// $600: the instruction at the odd target. It must never execute.
	dut.u_l1.mem[256] = 16'h20C0;  // MOVE.L D0,(A0)+
	dut.u_l1.mem[257] = 16'h60FE;  // BRA.B -2

	// $700: the address-error (vector 3) handler.
	dut.u_l1.mem[384] = 16'h7866;  // MOVEQ #$66,D4
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2

	// $780: every OTHER vector lands here, and must not run.
	dut.u_l1.mem[448] = 16'h7677;  // MOVEQ #$77,D3
	dut.u_l1.mem[449] = 16'h60FE;  // BRA.B -2

	// Vectors 2..47 -> $780, then vector 3 -> $700. Word index of vector n
	// is ((4*n - PC_RESET) & 13'h1FFF) >> 1.
	for (i = 2; i < 48; i = i + 1) begin
		dut.u_l1.mem[(3584 + 2*i) & 12'hFFF] = 16'h0000;
		dut.u_l1.mem[(3585 + 2*i) & 12'hFFF] = 16'h0780;
	end
	dut.u_l1.mem[3590] = 16'h0000;
	dut.u_l1.mem[3591] = 16'h0700;

	// The format-0 frame the RTE pops, at ISP $1000: SR, PC, format/vector.
	dut.u_l1.mem[1536] = 16'h2715;   // restored SR (supervisor)
	dut.u_l1.mem[1537] = 16'h0000;   // restored PC high
	dut.u_l1.mem[1538] = 16'h0601;   // restored PC low -- ODD
	dut.u_l1.mem[1539] = 16'h0000;   // format 0

	// The store's target, poisoned with a value no frame beat can produce.
	dut.u_l1.mem[512] = 16'h1234;
	dut.u_l1.mem[513] = 16'h5678;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0066) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000066 (the vector-3 handler must run)", dbg_d4);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (no other vector's handler may run)", dbg_d3);
	end

	// ...nor its postincrement, which is what proves the instruction did not
	// merely have its memory request suppressed but never ran at all.
	if (dbg_d0 !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11223344 (suppressing the store also swings an_wr_reg from the destination An to the SOURCE register, so a retiring target writes its address update into D0)",
		         dbg_d0);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0800) begin
		errors = errors + 1;
		$display("FAIL: A0 = %h, expected 00000800 (the (A0)+ at the odd target must not step; 00000804 means the instruction retired with its store gated)",
		         dut.u_cpu.u_regfile.areg[0]);
	end

	// The store must not have happened.
	chk("memory at $800", 512, 16'h1234);
	chk("memory at $802", 513, 16'h5678);

	// ...and the frame must be on the stack: the RTE popped eight bytes to
	// $1008, and a format $2 frame is twelve below that.
	chk("frame $FFC  (SR)",          1534, 16'h2715);
	chk("frame $FFE  (PC high)",     1535, 16'h0000);
	// The frame's PC field is the RTE's OWN address, and the RTE is the
	// twelfth word of this program, at $416 -- not $400, which is merely
	// where the program starts.
	chk("frame $1000 (PC low)",      1536, 16'h0416);
	chk("frame $1002 (format/vec)",  1537, 16'h200C);
	chk("frame $1004 (addr high)",   1538, 16'h0000);
	chk("frame $1006 (addr low)",    1539, 16'h0600);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
