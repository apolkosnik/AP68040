//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 110: an odd          //
// exception VECTOR)                                                         //
//                                                                          //
// tb_ap040_pipe_oddvec.v - the handler address has to be even too           //
//                                                                          //
// The last of the three exception defects the ninth review left open, and   //
// the oldest: it has been recorded in the plan since milestone 99 with its  //
// reference rule extracted, and it was blocked on this core having no halt. //
// Milestone 108 gave it one.                                                //
//                                                                          //
// The rule: an odd handler address for vector 2 or 3 is a DOUBLE FAULT and  //
// halts; any other odd handler becomes an address error whose frame's PC    //
// field is 4 * vector WITHOUT the vector base register -- "offset, not      //
// vbr + offset" -- and whose address field is the handler with bit 0        //
// cleared. Until now the vector was simply used: the core redirected to an  //
// odd address and executed the handler from it.                             //
//                                                                          //
//   A7 = $1000, TRAP #0 -- vector 32, whose table entry is $701             //
//                                                                          //
// So the TRAP's own format-0 frame lands at $FF8, and the address error it  //
// provokes pushes a format-2 frame below it at $FEC with PC field $80 --    //
// 4 * 32, and NOT $2000 + $80 -- and address field $700.                    //
//                                                                          //
// D3 is the poison at $700. It is what proves the odd handler was refused   //
// rather than entered: a core that redirects to $701 and runs from there    //
// reaches it. D4 is the address-error handler, deliberately at $780 so the  //
// two cannot be confused.                                                   //
//                                                                          //
// The re-entry could not ride exc_pend_addrerr, which is qualified by       //
// exc_go and so cannot raise anything after exc_vec_done has cleared it.    //
// vecodd_pend feeds addrerr_now instead -- the same door ae_take uses.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_oddvec;

localparam PROG_WORDS      = 400;
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
	dut.u_l1.mem[ 5] = 16'h4E40;   // TRAP #0   -- vector 32, handler $701
	dut.u_l1.mem[ 6] = 16'h60FE;   // BRA.B -2

	// $700: the ODD handler's own address, poisoned. Reaching it at all is
	// the failure this bench exists for.
	dut.u_l1.mem[384] = 16'h7677;  // MOVEQ #$77,D3
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2

	// $780: the address-error (vector 3) handler.
	dut.u_l1.mem[448] = 16'h7866;  // MOVEQ #$66,D4
	dut.u_l1.mem[449] = 16'h60FE;  // BRA.B -2

	// Vector 3 -> $780, and vector 32 (TRAP #0) -> $701, which is odd.
	dut.u_l1.mem[3590] = 16'h0000;
	dut.u_l1.mem[3591] = 16'h0780;
	dut.u_l1.mem[3648] = 16'h0000;
	dut.u_l1.mem[3649] = 16'h0701;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d4 !== 32'h0000_0066) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000066 (an odd handler must raise the address error whose handler this is)",
		         dbg_d4);
	end
	if (dbg_d3 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000000 (the odd handler at $700/$701 must never be entered)",
		         dbg_d3);
	end

	// The TRAP's format-0 frame sits at $FF8; the address error's format-2
	// frame is the twelve bytes below it.
	chk("frame $FF0 (PC low)",       1526 + 2, 16'h0080);
	chk("frame $FF2 (format/vec)",   1526 + 3, 16'h200C);
	chk("frame $FF4 (addr high)",    1526 + 4, 16'h0000);
	chk("frame $FF6 (addr low)",     1526 + 5, 16'h0700);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
