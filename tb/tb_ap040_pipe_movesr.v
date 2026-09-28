//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 114: MOVE to and    //
// from SR and CCR, every mode)                                             //
//                                                                          //
// tb_ap040_pipe_movesr.v - the status register as an ordinary operand      //
//                                                                          //
// Until milestone 114 the only form was MOVE Dn,SR. Now:                   //
//                                                                          //
//   MOVE 4(A0),CCR        $804 = $FF13: CCR := $13, the upper byte ignored //
//   MOVE CCR,(A1)+        $810 := $0013, A1 = $812                         //
//   MOVE (A0),SR          $800 = $2704: SR := $2704                        //
//   MOVE SR,$0820.W       $820 := $2704                                    //
//   MOVE #$0008,CCR       CCR := $08                                       //
//   MOVE SR,D3            D3 = $5555xxxx -> $55552708 (a word merge)       //
//   MOVE #$0000,SR        user mode                                        //
//   MOVE CCR,$0830.W      not privileged: $830 := $0000                    //
//   MOVE SR,-(A1)         privileged: vector 8, and NOTHING else -- $810   //
//                         keeps $0013 and A1 stays $812                    //
//                                                                          //
// MOVE to SR/CCR rides the ORI-to-SR path with ALU_MOVE; MOVE from SR/CCR  //
// rides Scc's read-modify-write carrier, the status word built in EX from  //
// the forwarded CCR. The last line is the privilege gate that now covers  //
// every EA-fetch access, not only MOVES's.                                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_movesr;

localparam PROG_WORDS      = 300;
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
wire [31:0] dbg_d3;
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
	.dbg_d3 (dbg_d3), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
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

	dut.u_l1.mem[ 0] = 16'h203C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h1000;  // MOVE.L #$1000,D0
	dut.u_l1.mem[ 3] = 16'h4E7B;  dut.u_l1.mem[ 4] = 16'h0804;                               // MOVEC D0,ISP
	dut.u_l1.mem[ 5] = 16'h207C;  dut.u_l1.mem[ 6] = 16'h0000;  dut.u_l1.mem[ 7] = 16'h0800;  // MOVEA.L #$800,A0
	dut.u_l1.mem[ 8] = 16'h227C;  dut.u_l1.mem[ 9] = 16'h0000;  dut.u_l1.mem[10] = 16'h0810;  // MOVEA.L #$810,A1
	dut.u_l1.mem[11] = 16'h263C;  dut.u_l1.mem[12] = 16'h5555;  dut.u_l1.mem[13] = 16'h0000;  // MOVE.L #$55550000,D3

	dut.u_l1.mem[14] = 16'h44E8;  dut.u_l1.mem[15] = 16'h0004;   // MOVE 4(A0),CCR
	dut.u_l1.mem[16] = 16'h42D9;                                 // MOVE CCR,(A1)+
	dut.u_l1.mem[17] = 16'h46D0;                                 // MOVE (A0),SR
	dut.u_l1.mem[18] = 16'h40F8;  dut.u_l1.mem[19] = 16'h0820;   // MOVE SR,$0820.W
	dut.u_l1.mem[20] = 16'h44FC;  dut.u_l1.mem[21] = 16'h0008;   // MOVE #$0008,CCR
	dut.u_l1.mem[22] = 16'h40C3;                                 // MOVE SR,D3
	dut.u_l1.mem[23] = 16'h46FC;  dut.u_l1.mem[24] = 16'h0000;   // MOVE #$0000,SR  -- user mode
	dut.u_l1.mem[25] = 16'h42F8;  dut.u_l1.mem[26] = 16'h0830;   // MOVE CCR,$0830.W
	dut.u_l1.mem[27] = 16'h40E1;                                 // MOVE SR,-(A1)  -- vector 8
	dut.u_l1.mem[28] = 16'h60FE;                                 // BRA.B -2

	// $780: vector 8.
	dut.u_l1.mem[448] = 16'h5278;  dut.u_l1.mem[449] = 16'h0600;  // ADDQ.W #1,$0600.W
	dut.u_l1.mem[450] = 16'h60FE;
	dut.u_l1.mem[3600] = 16'h0000; dut.u_l1.mem[3601] = 16'h0780;

	// Data. Word index = (address - $400) / 2.
	dut.u_l1.mem[256] = 16'h0000;   // $600
	dut.u_l1.mem[512] = 16'h2704;   // $800
	dut.u_l1.mem[514] = 16'hFF13;   // $804
	dut.u_l1.mem[520] = 16'hAAAA;   // $810
	dut.u_l1.mem[528] = 16'hBBBB;   // $820
	dut.u_l1.mem[536] = 16'hCCCC;   // $830
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("MOVE CCR,(A1)+ at $810, never overwritten", 520, 16'h0013);
	chk("MOVE SR,$0820.W",                            528, 16'h2704);
	chk("MOVE CCR,$0830.W in user mode",              536, 16'h0000);
	chk("vector 8 count at $600",                     256, 16'h0001);
	if (dbg_d3 !== 32'h5555_2708) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 55552708 (MOVE SR,D3 after MOVE #$0008,CCR)", dbg_d3);
	end
	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0812) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000812 (the user-mode MOVE SR,-(A1) must not step A1)",
		         dut.u_cpu.u_regfile.areg[1]);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
