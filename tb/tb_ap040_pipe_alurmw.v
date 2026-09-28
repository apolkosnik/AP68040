//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 48: ALU to memory)  //
//                                                                          //
// tb_ap040_pipe_alurmw.v - read-modify-write, the ir[8]=1 direction        //
//                                                                          //
// The first instruction in this pipeline that both LOADS and STORES, and   //
// the first real datapath addition since milestone 30. Every other store   //
// issues from ap040_ea_fetch.v out of a register it already holds; an      //
// RMW's store data is the ALU RESULT, which does not exist until a stage   //
// later, so ap040_execute.v became a second producer on L1 port B.         //
//                                                                          //
// EX wins that port unconditionally -- it is the OLDER instruction, so     //
// making EA-fetch wait is both correct and deadlock-free, where the        //
// reverse could starve an RMW behind a run of loads. port_taken carries    //
// the decision backward and EA-fetch behaves as if the cycle had not       //
// happened: no read issued, no output register moved.                      //
//                                                                          //
// Operands cross over in EA-fetch. The ALU computes b op a, so the loaded  //
// value has to be b -- the opposite of every other memory-source form --   //
// or SUB.L D1,(A1) computes D1 minus memory instead of memory minus D1.    //
//                                                                          //
// Memory before:                                                           //
//   $0480 = 00000010   $0484 = 00000020                                    //
//   $0488 = FFFFAAAA   $048C = 11223344                                    //
//                                                                          //
//   MOVEA.L #$0480,A0 / MOVE.L #5,D0                                       //
//   ADD.L  D0,(A0)      $0480 -> 00000015                                  //
//   MOVEA.L #$0484,A1 / MOVE.L #3,D1                                       //
//   SUB.L  D1,(A1)      $0484 -> 0000001D                                  //
//   MOVEA.L #$0488,A2                                                      //
//   AND.W  D0,(A2)      $0488 -> 0005AAAA                                  //
//   MOVEA.L #$048D,A4                                                      //
//   ADD.B  D0,(A4)+     $048C -> 11273344, A4 -> 0000048E                  //
//   MOVE.L (A0),D2      D2 = 00000015                                      //
//                                                                          //
// Each case is aimed at a specific way this can go wrong:                  //
//                                                                          //
//   $0480 = 15 rather than 1A is the DOUBLE-STORE check: one instruction,  //
//     one store. It covers a request that fails to drop after the write    //
//     lands, which is a real hazard here, since ex_st_req is combinational //
//     off eaf_valid and EA-fetch bubbles rather than advancing when it     //
//     loses the port.                                                      //
//                                                                          //
//     It does NOT cover the other double-store route, and this bench       //
//     should not be read as if it did. ap040_execute.v gained a local      //
//     stall for a store the L1 cannot accept, and its output-register gate //
//     moved from stall_in to ex_stall to match -- without that move, a     //
//     stalled RMW would retire downstream and store again on the retry.    //
//     That path is UNREACHABLE against the current ap040_pipe_l1.v:        //
//     wr_busy is high for exactly one cycle after a write, and EA-fetch    //
//     bubbles whenever EX holds the port, so no sequence lets EX see a     //
//     busy port. Confirmed by instrumenting rmw_wait and running all 52    //
//     benches: it never fires. Reverting the gate to stall_in alone still  //
//     passes everything, so the guard is deliberate defence for the real   //
//     cache that replaces this model, not something tested here.           //
//   $0484 = 1D rather than FFFFFFE3 is the operand CROSSOVER check. It is  //
//     invisible for ADD and AND, which is why SUB is here.                 //
//   $0488 keeping AAAA is the sized-lane check, and $048C uses an ODD      //
//     address so the byte lands in the low half of the high word rather    //
//     than the aligned case.                                               //
//   A4 = $048E proves the address-register update still happens on the     //
//     read half, now that the same instruction also writes.                //
//   D0 = 5 and D1 = 3 prove an RMW writes NO register. Decode points       //
//     eac_dest_reg at the data register so its value can be read, which is //
//     exactly the arrangement that would silently write it back.           //
//                                                                          //
// The final MOVE.L reads $0480 back through the ordinary load path, so the //
// store is confirmed by the pipeline and not only by the bench peeking at  //
// the array.                                                               //
//                                                                          //
// On milestone 47's RTL none of the four memory-destination forms decode.  //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_alurmw;

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
	dut.u_l1.mem[7]  = 16'hD190;   // ADD.L D0,(A0)
	dut.u_l1.mem[8]  = 16'h227C;   // MOVEA.L #$00000484,A1
	dut.u_l1.mem[9]  = 16'h0000;
	dut.u_l1.mem[10] = 16'h0484;
	dut.u_l1.mem[11] = 16'h223C;   // MOVE.L #$00000003,D1
	dut.u_l1.mem[12] = 16'h0000;
	dut.u_l1.mem[13] = 16'h0003;
	dut.u_l1.mem[14] = 16'h9391;   // SUB.L D1,(A1)
	dut.u_l1.mem[15] = 16'h247C;   // MOVEA.L #$00000488,A2
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h0488;
	dut.u_l1.mem[18] = 16'hC152;   // AND.W D0,(A2)
	dut.u_l1.mem[19] = 16'h287C;   // MOVEA.L #$0000048D,A4
	dut.u_l1.mem[20] = 16'h0000;
	dut.u_l1.mem[21] = 16'h048D;
	dut.u_l1.mem[22] = 16'hD11C;   // ADD.B D0,(A4)+
	dut.u_l1.mem[23] = 16'h2410;   // MOVE.L (A0),D2

	dut.u_l1.mem[64] = 16'h0000;   // $0480 = 00000010
	dut.u_l1.mem[65] = 16'h0010;
	dut.u_l1.mem[66] = 16'h0000;   // $0484 = 00000020
	dut.u_l1.mem[67] = 16'h0020;
	dut.u_l1.mem[68] = 16'hFFFF;   // $0488 = FFFFAAAA
	dut.u_l1.mem[69] = 16'hAAAA;
	dut.u_l1.mem[70] = 16'h1122;   // $048C = 11223344
	dut.u_l1.mem[71] = 16'h3344;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 60) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h0000_0015) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 00000015 (ADD.L D0,(A0); 0000001a means the store fired twice)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if ({dut.u_l1.mem[66], dut.u_l1.mem[67]} !== 32'h0000_001D) begin
		errors = errors + 1;
		$display("FAIL: $0484 = %h%h, expected 0000001d (SUB.L D1,(A1) is memory MINUS D1; ffffffe3 is the reverse)",
		         dut.u_l1.mem[66], dut.u_l1.mem[67]);
	end
	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'h0005_AAAA) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected 0005aaaa (AND.W must leave the low half alone)",
		         dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if ({dut.u_l1.mem[70], dut.u_l1.mem[71]} !== 32'h1127_3344) begin
		errors = errors + 1;
		$display("FAIL: $048C = %h%h, expected 11273344 (ADD.B at the ODD address $048D)",
		         dut.u_l1.mem[70], dut.u_l1.mem[71]);
	end
	if (dut.u_cpu.u_regfile.areg[4] !== 32'h0000_048E) begin
		errors = errors + 1;
		$display("FAIL: A4 = %h, expected 0000048e (the read half must still autoincrement)",
		         dut.u_cpu.u_regfile.areg[4]);
	end
	if (dbg_d0 !== 32'h0000_0005) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000005 (an RMW writes NO register; 00000015 means it wrote one)",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h0000_0003) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 00000003 (an RMW writes NO register)", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_0015) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00000015 (MOVE.L (A0),D2 must read the stored value back)", dbg_d2);
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
