//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 95: LINK A7)       //
//                                                                          //
// tb_ap040_pipe_linka7.v - the one register LINK cannot push unchanged     //
//                                                                          //
// LINK An,#d is three steps, in this order:                                //
//                                                                          //
//   SP := SP - 4 ;  (SP) := An ;  An := SP ;  SP := SP + d                 //
//                                                                          //
// When An IS A7 the first step has already changed the register the second //
// one pushes, so what reaches memory is the DECREMENTED pointer, not the   //
// value A7 held on entry. This core pushed the entry value.                //
//                                                                          //
//   ISP = $1000 ; LINK.W A7,#-8                                            //
//                                                                          //
// $0FFC must contain $00000FFC, and A7 must end at $0FF4. The final        //
// pointer was already right, which is why only the stored word says        //
// anything -- and rtl/ap040/ap040_core.v, which passes the cputest corpus, //
// stores $0FFC here.                                                       //
//                                                                          //
// LINK A6, the ordinary case, is checked in the same program so a fix that //
// pushed the decremented pointer for EVERY register would fail rather than //
// pass.                                                                    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_linka7;

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
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP    (A7 = $1000)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h2C3C;   // MOVE.L #$00005555,D6
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h5555;
	dut.u_l1.mem[ 9] = 16'h4E7B;   // MOVEC D6,USP  -- parks $5555 somewhere checkable
	dut.u_l1.mem[10] = 16'h0800;
	dut.u_l1.mem[11] = 16'h4E57;   // LINK.W A7,#-8
	dut.u_l1.mem[12] = 16'hFFF8;
	dut.u_l1.mem[13] = 16'h4E71;   // NOP (drain)

	// $0FFC, where LINK's push lands.
	dut.u_l1.mem[1534] = 16'h9999;  dut.u_l1.mem[1535] = 16'h9999;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[1534], dut.u_l1.mem[1535]} !== 32'h0000_0FFC) begin
		errors = errors + 1;
		$display("FAIL: $0FFC = %h%h, expected 00000ffc. LINK A7 decrements the stack pointer BEFORE it pushes it, so what reaches memory is the decremented value; 00001000 is the pointer as it stood on entry.",
		         dut.u_l1.mem[1534], dut.u_l1.mem[1535]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0FF4) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000ff4 ($1000 - 4, then the displacement -8)", dut.u_cpu.u_regfile.isp);
	end
	if (writes !== 1) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted, expected 1 (LINK pushes once)", writes);
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
