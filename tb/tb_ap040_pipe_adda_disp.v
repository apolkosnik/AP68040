//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 46: ADDA (d16,An))  //
//                                                                          //
// tb_ap040_pipe_adda_disp.v - pointer arithmetic against a struct field    //
//                                                                          //
// This rides milestone 40's gather kind rather than adding a tenth: same   //
// one extension word, same held_reg base, same gather_disp. What differs   //
// is what the instruction DOES with the loaded value, so the kind grew     //
// three properties that used to be constants of it:                       //
//                                                                          //
//   held_alu_areg  the destination is An, and the ALU width is Long        //
//   held_alu_ccr   this form sets condition codes (CMPA yes, ADDA/SUBA no) //
//   held_alu_sxt   the Word form sign-extends its source                   //
//                                                                          //
// Each is checked by a case that a wrong constant would break, because a   //
// property left inferred is the failure this milestone invites:            //
//                                                                          //
//   areg  -- if the destination stayed a DATA register, all three address  //
//            registers below keep their old values and every check fails.  //
//   ccr   -- if it stayed the old constant 1, the final ADDA would clear   //
//            the Z the CMPA set, and only the CCR check would notice.      //
//   sxt   -- if it stayed 0, A2 comes back 00200EEF. The flag drives       //
//            eff_size as well as the extension, so losing it reads the     //
//            whole LONGWORD at $0484 rather than reading a word and        //
//            zero-filling it; that second failure, which milestone 44      //
//            checks by mutating the extension function itself, would give  //
//            FFFF2020 instead. Two different bugs, two different values.   //
//                                                                          //
// Memory: $047C = 00001030, $0480 = 00000030, $0484 = FFE01111.            //
//                                                                          //
//   MOVEA.L #$0480,A0 / #$1000,A1                                          //
//   ADDA.L  (0,A0),A1    A1 = 00001030                                     //
//   MOVEA.L #$2000,A2                                                      //
//   SUBA.W  (4,A0),A2    minus -32, so A2 = 00002020                       //
//   MOVEA.L #$1030,A3                                                      //
//   CMPA.L  (-4,A0),A3   sets Z, writes nothing                            //
//   ADDA.L  (0,A0),A3    A3 = 00001060 -- and Z must survive it            //
//                                                                          //
// The CMPA uses a NEGATIVE displacement, since a zero-extended one is       //
// invisible for every forward reference, and the first ADDA uses a         //
// displacement of ZERO, the case that would still pass if the extension    //
// word were dropped entirely. Neither stands alone.                        //
//                                                                          //
// On milestone 45's RTL none of the displacement forms decode.             //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_adda_disp;

localparam PROG_WORDS      = 24;
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
	dut.u_l1.mem[4]  = 16'h227C;   // MOVEA.L #$00001000,A1
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h1000;
	dut.u_l1.mem[7]  = 16'hD3E8;   // ADDA.L (0,A0),A1
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h247C;   // MOVEA.L #$00002000,A2
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h2000;
	dut.u_l1.mem[12] = 16'h94E8;   // SUBA.W (4,A0),A2
	dut.u_l1.mem[13] = 16'h0004;
	dut.u_l1.mem[14] = 16'h267C;   // MOVEA.L #$00001030,A3
	dut.u_l1.mem[15] = 16'h0000;
	dut.u_l1.mem[16] = 16'h1030;
	dut.u_l1.mem[17] = 16'hB7E8;   // CMPA.L (-4,A0),A3
	dut.u_l1.mem[18] = 16'hFFFC;
	dut.u_l1.mem[19] = 16'hD7E8;   // ADDA.L (0,A0),A3
	dut.u_l1.mem[20] = 16'h0000;

	dut.u_l1.mem[62] = 16'h0000;   // $047C = 00001030
	dut.u_l1.mem[63] = 16'h1030;
	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000030
	dut.u_l1.mem[65] = 16'h0030;
	dut.u_l1.mem[66] = 16'hFFE0;   // $0484 = FFE01111
	dut.u_l1.mem[67] = 16'h1111;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.areg[1] !== 32'h0000_1030) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00001030 (ADDA.L (0,A0),A1)", dut.u_cpu.u_regfile.areg[1]);
	end
	if (dut.u_cpu.u_regfile.areg[2] !== 32'h0000_2020) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00002020 (SUBA.W: 00200eef means the Word size was lost, ffff2020 the sign)",
		         dut.u_cpu.u_regfile.areg[2]);
	end
	if (dut.u_cpu.u_regfile.areg[3] !== 32'h0000_1060) begin
		errors = errors + 1;
		$display("FAIL: A3 = %h, expected 00001060 (CMPA writes nothing, then ADDA.L adds $30)",
		         dut.u_cpu.u_regfile.areg[3]);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. The CMPA set Z; the ADDA after it must not
	// touch the flags, which it would if held_alu_ccr had stayed constant.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMPA sets Z; ADDA after it must set nothing)",
		         dbg_ccr[3:0]);
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
