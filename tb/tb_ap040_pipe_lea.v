//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 42: LEA)            //
//                                                                          //
// tb_ap040_pipe_lea.v - compute an address, write it to An, touch nothing  //
//                                                                          //
// LEA is in every function prologue and every array index, and it is the    //
// cheapest useful instruction left in the ISA: it produces an address and   //
// writes it to An, reading no memory and setting no condition codes.        //
//                                                                          //
// The datapath already knew how to deliver a computed address as an         //
// operand -- ap040_ea_fetch.v routes ea_target into eaf_operand_a for JMP   //
// and JSR. LEA joins that one ternary, and with id_alu_op = MOVE the        //
// address lands in An through the ordinary writeback. Everything else is    //
// decode: mode 010 needs no gather, mode 101 is the ninth kind on the       //
// shared gather state machine.                                             //
//                                                                          //
// Memory is seeded with values that are NOTHING like the addresses that     //
// hold them, because the failure this instruction invites is reading        //
// memory it should only have addressed:                                    //
//                                                                          //
//   $047C = DEADBEEF   $0480 = CAFEBABE                                    //
//   $0488 = FEEDFACE   $048C = 12345678                                    //
//                                                                          //
//   MOVEA.L #$0480,A0                                                      //
//   LEA     (A0),A1      A1 = $0480, NOT CAFEBABE                          //
//   LEA     (8,A0),A2    A2 = $0488, NOT FEEDFACE                          //
//   LEA     (-4,A0),A3   A3 = $047C, NOT DEADBEEF                          //
//   MOVE.L  (A1),D0      must fetch CAFEBABE, i.e. A1 really addresses     //
//   MOVEQ   #-1,D1       sets N                                            //
//   LEA     (12,A0),A4   A4 = $048C -- and N must survive                  //
//                                                                          //
// The negative displacement is here for the same reason as milestone 40's:  //
// a zero-extended one is invisible for every forward reference.            //
//                                                                          //
// The final LEA is placed AFTER the MOVEQ specifically so the N flag can    //
// be checked afterwards. LEA setting condition codes is a plausible bug --  //
// it shares id_alu_op = MOVE with instructions that do set them -- and it   //
// would be invisible in any ordering where a flag-setting instruction ran   //
// last. The address registers are read from the register file directly      //
// rather than moved into data registers, because a MOVE used to observe    //
// one would itself overwrite the flags under test. (MOVE.L An,Dn is also   //
// not a form this core decodes -- source mode 001 has no entry.) Instead    //
// the LEA result is dereferenced once, with MOVE.L (A1),D0: if A1 holds     //
// $0480 then D0 must come back CAFEBABE, which checks the address is not    //
// merely stored but usable.                                                //
//                                                                          //
// On milestone 41's RTL neither LEA form decodes.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_lea;

localparam PROG_WORDS      = 32;
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
wire [31:0] dbg_d0, dbg_d1, dbg_d2;
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

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

initial begin
	#1;
	dut.u_l1.mem[1]  = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[2]  = 16'h0000;
	dut.u_l1.mem[3]  = 16'h0480;
	dut.u_l1.mem[4]  = 16'h43D0;   // LEA (A0),A1
	dut.u_l1.mem[5]  = 16'h45E8;   // LEA (8,A0),A2
	dut.u_l1.mem[6]  = 16'h0008;
	dut.u_l1.mem[7]  = 16'h47E8;   // LEA (-4,A0),A3
	dut.u_l1.mem[8]  = 16'hFFFC;
	dut.u_l1.mem[9]  = 16'h2011;   // MOVE.L (A1),D0
	dut.u_l1.mem[10] = 16'h72FF;   // MOVEQ #-1,D1
	dut.u_l1.mem[11] = 16'h49E8;   // LEA (12,A0),A4
	dut.u_l1.mem[12] = 16'h000C;

	dut.u_l1.mem[62] = 16'hDEAD;   // $047C
	dut.u_l1.mem[63] = 16'hBEEF;
	dut.u_l1.mem[64] = 16'hCAFE;   // $0480
	dut.u_l1.mem[65] = 16'hBABE;
	dut.u_l1.mem[68] = 16'hFEED;   // $0488
	dut.u_l1.mem[69] = 16'hFACE;
	dut.u_l1.mem[70] = 16'h1234;   // $048C
	dut.u_l1.mem[71] = 16'h5678;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0480) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000480 (LEA (A0) must address, not read)", dut.u_cpu.u_regfile.areg[1]);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_0488) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00000488 (LEA (8,A0))", dut.u_cpu.u_regfile.areg[2]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_047C) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 0000047c (LEA (-4,A0): the displacement must sign-extend)",
		         dut.u_cpu.u_regfile.areg[3]);
	end
	if (dut.u_cpu.u_regfile.areg[4] !== 32'h0000_048C) begin
		errors = errors + 1;
		$display("FAIL: A4 = %h, expected 0000048c (LEA (12,A0))", dut.u_cpu.u_regfile.areg[4]);
	end
	if (dbg_d0 !== 32'hCAFE_BABE) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected cafebabe (MOVE.L (A1),D0 -- the LEA result must work as an address)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'hFFFF_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected ffffffff (MOVEQ #-1)", dbg_d1);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. MOVEQ #-1 set N; the LEA after it must
	// leave the flags exactly as it found them.
	if (dbg_ccr[3:0] !== 4'b1000) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 1000 (LEA must not touch condition codes)", dbg_ccr[3:0]);
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
