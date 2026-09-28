//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 109: the suppressed  //
// instruction must not choose the vector)                                   //
//                                                                          //
// tb_ap040_pipe_rteoddvec.v - whose exception is this                       //
//                                                                          //
// The sibling of tb_ap040_pipe_rteoddhold.v. That one is about the held     //
// instruction's memory accesses; this one is about its OPCODE.             //
//                                                                          //
// exc_vec_num tested eac_is_illegal and eac_is_priv BEFORE eac_is_addrerr,  //
// and read them raw. So a deferred RTE address error started an entry and   //
// then took its vector from the very instruction it had just suppressed:    //
// an ILLEGAL at the odd target stacked vector 4, and a privileged opcode     //
// after a return to user mode stacked vector 8 -- each with the address     //
// error's own PC and address fields, so the frame combined one fault's      //
// context with another's vector.                                           //
//                                                                          //
// The tell was in the source: exc_pc_field, three lines above, has always   //
// tested eac_is_addrerr FIRST. The two muxes disagreed with each other, and //
// the fix is to make the vector agree with the PC field.                    //
//                                                                          //
//   ISP $1000, format-0 frame there with SR $2715 and PC $601              //
//   ILLEGAL ($4AFC) sits at $600 and must never execute                    //
//                                                                          //
// ILLEGAL is used rather than a privileged opcode because it fails in both  //
// supervisor and user mode, so this bench does not depend on which SR the   //
// frame restores. The privileged variant is the same mux line and is not    //
// separately reproduced here.                                              //
//                                                                          //
// D4 proves the vector-3 handler ran and D3 proves the vector-4 one did     //
// not; the stacked format/vector word is checked directly, because a core   //
// that reached the right handler by luck would still stack $2010.          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rteoddvec;

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
wire [31:0] dbg_d3, dbg_d4;
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
	.dbg_d3 (dbg_d3), .dbg_d4 (dbg_d4), .dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr)
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
	dut.u_l1.mem[ 5] = 16'h4E73;   // RTE   -- restores PC $601, which is ODD

	// $600: the instruction at the odd target. It must never execute.
	dut.u_l1.mem[256] = 16'h4AFC;  // ILLEGAL
	dut.u_l1.mem[257] = 16'h60FE;  // BRA.B -2

	// $700: the address-error (vector 3) handler.
	dut.u_l1.mem[384] = 16'h7866;  // MOVEQ #$66,D4
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2

	// $780: every OTHER vector lands here -- vector 4 above all.
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
		$display("FAIL: D3 = %h, expected 00000000 (vector 4 belongs to the suppressed ILLEGAL, not to this fault)", dbg_d3);
	end

	// ...and the frame must be on the stack: the RTE popped eight bytes to
	// $1008, and a format $2 frame is twelve below that.
	chk("frame $FFC  (SR)",          1534, 16'h2715);
	chk("frame $FFE  (PC high)",     1535, 16'h0000);
	// The frame's PC field is the RTE's OWN address: the sixth word here.
	chk("frame $1000 (PC low)",      1536, 16'h040A);
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
