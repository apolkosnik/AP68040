//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 115: MOVEP)         //
//                                                                          //
// tb_ap040_pipe_movep.v - alternate bytes                                  //
//                                                                          //
// MOVEP moves a register to or from every OTHER byte: (d16,Ay), +2, +4,   //
// +6, the high byte first. It is a four- or two-beat sequencer in          //
// ap040_eafetch.v. The base here is ODD, which MOVEP allows and which      //
// puts every byte in the low lane of its word; the high lanes are markers  //
// that must survive. No condition codes change (ORI #$1F,CCR before the    //
// block, CCR $1F after it), and a Word load keeps Dx's upper half.         //
//                                                                          //
//   A0 = $801, D0 = $11223344, D1 = $AAAAAAAA                              //
//   MOVEP.L D0,0(A0)     $801/$803/$805/$807 := $11 $22 $33 $44            //
//   MOVEP.W D0,$10(A0)   $811/$813 := $33 $44                              //
//   MOVEP.L 0(A0),D2     D2 = $11223344                                    //
//   MOVEP.W $10(A0),D1   D1 = $AAAA3344                                    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movep;

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
wire [31:0] dbg_d1, dbg_d2;
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
	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
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

	dut.u_l1.mem[ 0] = 16'h207C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h0801;  // MOVEA.L #$801,A0
	dut.u_l1.mem[ 3] = 16'h203C;  dut.u_l1.mem[ 4] = 16'h1122;  dut.u_l1.mem[ 5] = 16'h3344;  // MOVE.L #$11223344,D0
	dut.u_l1.mem[ 6] = 16'h223C;  dut.u_l1.mem[ 7] = 16'hAAAA;  dut.u_l1.mem[ 8] = 16'hAAAA;  // MOVE.L #$AAAAAAAA,D1
	dut.u_l1.mem[ 9] = 16'h003C;  dut.u_l1.mem[10] = 16'h001F;                               // ORI #$1F,CCR
	dut.u_l1.mem[11] = 16'h01C8;  dut.u_l1.mem[12] = 16'h0000;                               // MOVEP.L D0,0(A0)
	dut.u_l1.mem[13] = 16'h0188;  dut.u_l1.mem[14] = 16'h0010;                               // MOVEP.W D0,$10(A0)
	dut.u_l1.mem[15] = 16'h0548;  dut.u_l1.mem[16] = 16'h0000;                               // MOVEP.L 0(A0),D2
	dut.u_l1.mem[17] = 16'h0308;  dut.u_l1.mem[18] = 16'h0010;                               // MOVEP.W $10(A0),D1
	dut.u_l1.mem[19] = 16'h60FE;                                                             // BRA.B -2

	for (i = 512; i < 524; i = i + 1) dut.u_l1.mem[i] = 16'hEEEE;   // $800-$817
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("$800/$801", 512, 16'hEE11);
	chk("$802/$803", 513, 16'hEE22);
	chk("$804/$805", 514, 16'hEE33);
	chk("$806/$807", 515, 16'hEE44);
	chk("$808 untouched", 516, 16'hEEEE);
	chk("$810/$811", 520, 16'hEE33);
	chk("$812/$813", 521, 16'hEE44);
	chk("$814 untouched", 522, 16'hEEEE);
	if (dbg_d2 !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 11223344 (MOVEP.L load)", dbg_d2);
	end
	if (dbg_d1 !== 32'hAAAA_3344) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected AAAA3344 (MOVEP.W load keeps the upper half)", dbg_d1);
	end
	if (dbg_ccr !== 5'h1F) begin
		errors = errors + 1;
		$display("FAIL: CCR = %h, expected 1F (MOVEP changes no condition code)", dbg_ccr);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
