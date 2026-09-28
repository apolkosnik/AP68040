//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 55: RMW to (d16,An))      //
//                                                                          //
// tb_ap040_pipe_rmwdisp.v - the struct-field update                       //
//                                                                          //
// ADD.L D0,(8,A0) is what a compiler emits for `s->field += x`, and it is  //
// the most-used read-modify-write mode.                                    //
//                                                                          //
// The DATAPATH needed nothing. Milestone 48 already loads from ea_target   //
// and stores back to eaf_ea_target, and for mode 101 ea_target is already  //
// operand_a plus the displacement. This is decode alone, riding milestone  //
// 40's gather kind with three more carried properties -- the fifth time    //
// that pattern has come up, and the plan has predicted it since 46.        //
//                                                                          //
// One of those properties is the OP MAP, not just a flag, which is what    //
// makes this more than a copy of the previous four. In the ir[8]=1         //
// direction nibble 1011 is EOR, not CMP, so this gather kind can no longer //
// take held_alu_op from alu_nib_op unconditionally.                        //
//                                                                          //
// Memory: $047C = 11111111, $0480 = 00000010, $0484 = 00000020.            //
//                                                                          //
//   MOVEA.L #$0480,A0 / MOVE.L #5,D0                                       //
//   ADD.L  D0,(0,A0)    $0480 -> 00000015                                  //
//   SUB.L  D0,(4,A0)    $0484 -> 0000001B                                  //
//   EOR.L  D0,(-4,A0)   $047C -> 11111114                                  //
//                                                                          //
// The EOR is the op-map check and the reason it is in this bench at all:   //
// decoded through the ir[8]=0 map, nibble 1011 is CMP, which writes        //
// nothing -- so $047C would come back 11111111 unchanged, with no other    //
// symptom anywhere. The SUB is the operand-order check, invisible for ADD  //
// and EOR and giving FFFFFFE5 if reversed. The displacements are zero,     //
// positive and NEGATIVE, so a dropped extension word and a zero-extended   //
// one are both caught.                                                     //
//                                                                          //
// D0 = 5 at the end says an RMW still writes no register, which decode     //
// arranges by carrying held_alu_nowrite for this form -- the same bit CMP  //
// uses, for a different reason.                                            //
//                                                                          //
// On milestone 54's RTL none of the three decode.                          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_rmwdisp;

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
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$00000005,D0
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0005;
	dut.u_l1.mem[7]  = 16'hD1A8;   // ADD.L D0,(0,A0)
	dut.u_l1.mem[8]  = 16'h0000;
	dut.u_l1.mem[9]  = 16'h91A8;   // SUB.L D0,(4,A0)
	dut.u_l1.mem[10] = 16'h0004;
	dut.u_l1.mem[11] = 16'hB1A8;   // EOR.L D0,(-4,A0)
	dut.u_l1.mem[12] = 16'hFFFC;

	dut.u_l1.mem[62] = 16'h1111;   // $047C = 11111111
	dut.u_l1.mem[63] = 16'h1111;
	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000010
	dut.u_l1.mem[65] = 16'h0010;
	dut.u_l1.mem[66] = 16'h0000;   // $0484 = 00000020
	dut.u_l1.mem[67] = 16'h0020;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0000_0015) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 00000015 (ADD.L D0,(0,A0) -- a displacement of zero)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0000_001B) begin
		errors = errors + 1;
		$display("FAIL: $0484 = %h%h, expected 0000001b (SUB.L is memory MINUS D0; ffffffe5 is the reverse)",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if ({dut.u_l1.mem[62], dut.u_l1.mem[63]} !== 32'h1111_1114) begin
		errors = errors + 1;
		$display("FAIL: $047C = %h%h, expected 11111114 (nibble 1011 is EOR in this direction; 11111111 means it decoded as CMP)",
		         dut.u_l1.mem[62], dut.u_l1.mem[63]);
	end
	if (dbg_d0 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000005 (an RMW writes NO register)", dbg_d0);
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
