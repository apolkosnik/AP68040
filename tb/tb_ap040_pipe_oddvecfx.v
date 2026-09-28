//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 113: an odd vector  //
// whose handler alias has side effects)                                    //
//                                                                          //
// tb_ap040_pipe_oddvecfx.v - the alias instruction must do nothing         //
//                                                                          //
// tb_ap040_pipe_oddvec.v proves an odd handler address becomes an address  //
// error, and poisons the alias with a MOVEQ. A MOVEQ was the kind case:    //
// review 11 (finding 2) put a STORE there and it ran. The odd vector       //
// redirects the fetch to the even alias, and the first instruction found   //
// there is what carries the secondary address error -- exactly as the     //
// instruction behind an odd RTE carries its deferred one -- but only the   //
// RTE opened ae_susp, the window that gates a held instruction's loads,    //
// stores, pushes and MOVEM. So MOVE.L D0,(A0) at the alias wrote $800, and //
// worse, the secondary entry's three frame beats followed it to $800.      //
//                                                                          //
//   A7 = $1000, A0 = $800, D0 = $11223344, $800 holds $AAAAAAAA            //
//   TRAP #0 -- vector 32, whose table entry is $701                        //
//   $700: MOVE.L D0,(A0)     the alias: must not store                     //
//   $780: MOVEQ #$66,D4      the address-error handler                     //
//                                                                          //
// The TRAP's format-0 frame is at $FF8; the address error's format-2      //
// frame is the twelve bytes below it at $FEC, PC field $80 (4 * 32, no     //
// VBR), address field $700. $800 must still read $AAAAAAAA, and the         //
// handler must be reached, which it was not when the vector read followed  //
// the frame to $800.                                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_oddvecfx;

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
wire [31:0] dbg_d4;
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
	.dbg_d4 (dbg_d4), .dbg_sr(dbg_sr), .dbg_ccr(dbg_ccr)
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
	dut.u_l1.mem[11] = 16'h4E40;   // TRAP #0   -- vector 32, handler $701
	dut.u_l1.mem[12] = 16'h60FE;   // BRA.B -2

	// $700: the odd handler's even alias. A store: it must not happen.
	dut.u_l1.mem[384] = 16'h2080;  // MOVE.L D0,(A0)
	dut.u_l1.mem[385] = 16'h60FE;  // BRA.B -2

	// $780: the address-error (vector 3) handler.
	dut.u_l1.mem[448] = 16'h7866;  // MOVEQ #$66,D4
	dut.u_l1.mem[449] = 16'h60FE;  // BRA.B -2

	// $800, which the alias would write.
	dut.u_l1.mem[512] = 16'hAAAA;
	dut.u_l1.mem[513] = 16'hAAAA;

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
		$display("FAIL: D4 = %h, expected 00000066 (the address-error handler was not reached)", dbg_d4);
	end
	chk("$800 high (the alias store)",  512, 16'hAAAA);
	chk("$802 low",                     513, 16'hAAAA);

	// The TRAP's frame at $FF8, then the address error's at $FEC.
	chk("TRAP frame $FF8 (SR)",         1532, 16'h2700);
	chk("TRAP frame $FFE (format/vec)", 1535, 16'h0080);
	chk("AE frame $FEC (SR)",           1526, 16'h2700);
	chk("AE frame $FF0 (PC low)",       1528, 16'h0080);
	chk("AE frame $FF2 (format/vec)",   1529, 16'h200C);
	chk("AE frame $FF4 (addr high)",    1530, 16'h0000);
	chk("AE frame $FF6 (addr low)",     1531, 16'h0700);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
