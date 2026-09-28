//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 40: ALU on (d16,An))//
//                                                                          //
// tb_ap040_pipe_aludisp.v - the binary ALU family with displacement mode   //
//                                                                          //
// Milestone 39 reached the ALU family with ea mode 010, (An). Mode 101,    //
// (d16,An), is the one compiled code leans on hardest -- every struct      //
// field and every stack-frame local is a displacement off a base register. //
//                                                                          //
// It cannot be a wire change like mode 010 was, because the displacement    //
// is an extension word, so this is a new kind on the shared gather state    //
// machine -- the eighth. It needs almost nothing new: held_reg already      //
// holds An, held_dest_reg holds Dn, held_mv_size holds the size, and        //
// gather_disp is already the sign-extended displacement that                //
// MOVE.L (d16,An),Dn has fed into id_imm since milestone 10. The one        //
// genuinely new piece of state is held_alu_op, because every earlier        //
// gather kind had a FIXED operation and could pick it from the kind flags.  //
//                                                                          //
// Memory: $047C = 00000003, $0480 = 00000005,                              //
//         $0484 = 00000007, $0488 = 00075555.                              //
//                                                                          //
//   MOVEA.L #$0480,A0                                                      //
//   MOVE.L  #$00000010,D0                                                  //
//   ADD.L   (4,A0),D0    10 + 7 = 17                                       //
//   SUB.L   (-4,A0),D0   17 - 3 = 14                                       //
//   MOVE.L  #$1111000A,D1                                                  //
//   AND.W   (8,A0),D1    000A & 0007 in the low word only                  //
//   MOVE.L  #$00000005,D2                                                  //
//   CMP.L   (0,A0),D2    5 - 5, sets Z and writes nothing                  //
//                                                                          //
// The NEGATIVE displacement is the point of the second operation. A         //
// displacement that is not sign-extended reads $0480 + $0000FFFC instead,   //
// far outside the program, and a zero-extension bug is invisible for every  //
// forward reference -- so a backward one has to be here. D0 is again        //
// checked only after both operations, because the composition is what       //
// pins them down: 14 needs both, the ADD alone leaves 17 and the SUB        //
// alone leaves 0D.                                                         //
//                                                                          //
// A displacement of ZERO is deliberately used for the CMP. It is the case   //
// that looks identical to mode 010 and would pass if the extension word     //
// were dropped entirely, so it is paired with the two non-zero ones rather  //
// than standing alone.                                                      //
//                                                                          //
// AND.W proves the gathered size comes from ir[7:6] and not from MOVE's     //
// ir[13:12] field, which held_mv_size carries for every other user of this  //
// gather: read as a MOVE size, 01 would mean Byte, and D1's low byte would  //
// survive as 0A rather than becoming 02.                                   //
//                                                                          //
// This gather must also NOT trigger IF's speculative redirect, the same     //
// gate MOVE.L (d16,An),Dn needed -- otherwise the program jumps to          //
// held_pc + 2 + the displacement and nothing after it runs. All four        //
// checks would fail at once, which is exactly what the milestone-39         //
// control shows for a different reason, so the drain check below is what    //
// separates the two.                                                       //
//                                                                          //
// On milestone 39's RTL none of the four displacement forms decode.        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_aludisp;

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
	dut.u_l1.mem[4]  = 16'h203C;   // MOVE.L #$00000010,D0
	dut.u_l1.mem[5]  = 16'h0000;
	dut.u_l1.mem[6]  = 16'h0010;
	dut.u_l1.mem[7]  = 16'hD0A8;   // ADD.L (4,A0),D0
	dut.u_l1.mem[8]  = 16'h0004;
	dut.u_l1.mem[9]  = 16'h90A8;   // SUB.L (-4,A0),D0
	dut.u_l1.mem[10] = 16'hFFFC;
	dut.u_l1.mem[11] = 16'h223C;   // MOVE.L #$1111000A,D1
	dut.u_l1.mem[12] = 16'h1111;
	dut.u_l1.mem[13] = 16'h000A;
	dut.u_l1.mem[14] = 16'hC268;   // AND.W (8,A0),D1
	dut.u_l1.mem[15] = 16'h0008;
	dut.u_l1.mem[16] = 16'h243C;   // MOVE.L #$00000005,D2
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h0005;
	dut.u_l1.mem[19] = 16'hB4A8;   // CMP.L (0,A0),D2
	dut.u_l1.mem[20] = 16'h0000;

	dut.u_l1.mem[62] = 16'h0000;   // $047C = 00000003
	dut.u_l1.mem[63] = 16'h0003;
	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000005
	dut.u_l1.mem[65] = 16'h0005;
	dut.u_l1.mem[66] = 16'h0000;   // $0484 = 00000007
	dut.u_l1.mem[67] = 16'h0007;
	dut.u_l1.mem[68] = 16'h0007;   // $0488 = 00075555
	dut.u_l1.mem[69] = 16'h5555;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h0000_0014) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000014 (ADD.L (4,A0) then SUB.L (-4,A0))", dbg_d0);
	end
	if (dbg_d1 !== 32'h1111_0002) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 11110002 (AND.W must size from ir[7:6], not MOVE's size field)",
		         dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000005 (CMP must not write its result back)", dbg_d2);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. CMP.L of equal operands sets Z alone; the
	// reversed subtraction would give the same Z here, but N and C would
	// differ on unequal operands, so Z is checked together with the fact
	// that D2 survived.
	if (dbg_ccr[3:0] !== 4'b0100) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0100 (CMP.L of equal operands sets Z)", dbg_ccr[3:0]);
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
