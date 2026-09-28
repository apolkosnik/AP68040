//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (milestone 62: ORI/ANDI/EORI to SR)  //
//                                                                          //
// tb_ap040_pipe_immsr.v - masking interrupts, and editing the flags        //
//                                                                          //
// ORI #$0700,SR is how interrupts get masked and ANDI #$F8FF,SR is how     //
// they come back, so these are not optional for code that runs with        //
// interrupts at all.                                                       //
//                                                                          //
// They share immop_shape's nibble but not its mode field -- 111/100 with   //
// size 00 addresses CCR and size 01 addresses SR, where immop_shape needs  //
// mode 000 -- so the two families are disjoint by construction. Only ORI,  //
// ANDI and EORI are legal here; SUBI, ADDI and CMPI have no CCR or SR      //
// form.                                                                    //
//                                                                          //
// The result does not go through the ALU: the operand is the status        //
// register, so ap040_execute.v computes it from eaf_sr_snapshot -- the     //
// live, forwarded SR already threaded there -- and commits it on the same  //
// path MOVE-to-SR and RTE use.                                             //
//                                                                          //
// SR resets to $2700: supervisor, interrupt mask 7.                        //
//                                                                          //
//   ANDI #$F8FF,SR    mask -> 0,  SR = $2000                               //
//   ORI  #$0500,SR    mask -> 5,  SR = $2500                               //
//   MOVEQ #-1,D0      sets N                                               //
//   ORI  #$0004,CCR   sets Z                                               //
//   ANDI #$0007,CCR   clears N and X, keeps Z, V, C                        //
//   EORI #$0001,CCR   toggles C                                            //
//                                                                          //
// ANDI #$0007,CCR is chosen so that the CCR form being computed on the     //
// WRONG WIDTH is unmistakable. Done on eight bits it clears N and X and    //
// leaves the upper byte alone. Done on sixteen -- the obvious              //
// implementation, masking afterwards -- it ANDs $0007 into the whole SR    //
// and clears the supervisor bit along with the interrupt mask, dropping    //
// the core out of supervisor mode entirely. The upper byte is therefore    //
// checked as carefully as the flags.                                       //
//                                                                          //
// The two SR operations are checked together through $25: the ANDI must    //
// clear the mask the reset value set, and the ORI must then put back 5 and //
// not 7. A single one of them could not tell those apart.                  //
//                                                                          //
// On milestone 61's RTL none of the five decode.                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_immsr;

// The fetch budget counts every word the fetch stage issues, and since
// bundle 10 an SR write refetches the words behind it (they were fetched
// under its privilege): the two SR operations each re-issue what was
// already fetched. The L1 is NOP beyond the program, so the slack is NOPs.
localparam PROG_WORDS      = 28;
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
	dut.u_l1.mem[1]  = 16'h027C;   // ANDI #$F8FF,SR   -- clear the interrupt mask
	dut.u_l1.mem[2]  = 16'hF8FF;
	dut.u_l1.mem[3]  = 16'h007C;   // ORI  #$0500,SR   -- mask := 5
	dut.u_l1.mem[4]  = 16'h0500;
	dut.u_l1.mem[5]  = 16'h70FF;   // MOVEQ #-1,D0     -- sets N
	dut.u_l1.mem[6]  = 16'h003C;   // ORI  #$0004,CCR  -- sets Z
	dut.u_l1.mem[7]  = 16'h0004;
	dut.u_l1.mem[8]  = 16'h023C;   // ANDI #$0007,CCR  -- clears N and X
	dut.u_l1.mem[9]  = 16'h0007;
	dut.u_l1.mem[10] = 16'h0A3C;   // EORI #$0001,CCR  -- toggles C
	dut.u_l1.mem[11] = 16'h0001;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// A7 is the address bank's register 7, which in supervisor mode is the
	// ISP. See tb_ap040_pipe_move_mem.v's header for why the poke has to
	// land past the reset edge's own NBA region.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// SR resets to $2700. ANDI #$F8FF clears the mask, ORI #$0500 sets it to
	// 5, and neither CCR operation may touch the upper byte.
	if (dut.u_cpu.sr[15:8] !== 8'h25) begin
		errors = errors + 1;
		$display("FAIL: SR[15:8] = %h, expected 25 (supervisor set, mask 5; 00 means a CCR op ran 16 bits wide)",
		         dut.u_cpu.sr[15:8]);
	end
	if (dbg_d0 !== 32'hFFFF_FFFF) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected ffffffff (MOVEQ #-1)", dbg_d0);
	end

	// dbg_ccr[3:0] is {N,Z,V,C}. MOVEQ set N; ORI set Z; ANDI #$0007 cleared
	// N; EORI #$0001 toggled C. V was never set.
	if (dbg_ccr[3:0] !== 4'b0101) begin
		errors = errors + 1;
		$display("FAIL: CCR[NZVC] = %b, expected 0101 (Z from the ORI, N cleared by the ANDI, C toggled by the EORI)",
		         dbg_ccr[3:0]);
	end
	// X is bit 4 and the ANDI #$0007 cleared it along with N.
	if (dbg_ccr[4] !== 1'b0) begin
		errors = errors + 1;
		$display("FAIL: CCR X = %b, expected 0 (ANDI #$0007 clears bit 4 too)", dbg_ccr[4]);
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
