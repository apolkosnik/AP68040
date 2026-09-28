//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (integration, after milestone 68)    //
//                                                                          //
// tb_ap040_pipe_integration2.v - a whole subroutine, called properly       //
//                                                                          //
// tb_ap040_pipe_integration.v dates from milestone 33 and uses MOVE, ADD   //
// and (An)+. Thirty milestones have gone in since, and every one of them   //
// was proved by a bench that runs three to six instructions in isolation.  //
// This one runs a subroutine the way compiled code would:                  //
//                                                                          //
//   main:     set up an array pointer and a count, put sentinels in D1/D2, //
//             BSR to the subroutine, store the result                      //
//                                                                          //
//   average:  LINK A6,#0                                                   //
//             MOVEM.L D1-D2,-(A7)     save what it clobbers                //
//             sum the array through (A0)+ in a SUBQ/BNE loop              //
//             DIVU.W by the count                                          //
//             MOVEM.L (A7)+,D1-D2     restore                              //
//             UNLK A6 ; RTS                                                //
//                                                                          //
// Array 10/20/30/40, so the sum is 100 and the average 25.                 //
//                                                                          //
// What this tests that no unit bench does is INTERACTION. The subroutine   //
// nests three things that each move A7 -- BSR's return address, LINK's     //
// frame, and MOVEM's register save -- and unwinds them in the opposite     //
// order. Any one of them being off by four still returns, to the wrong     //
// place; the check that catches it is that the RTS lands on the            //
// instruction after the BSR and the store then happens at all.             //
//                                                                          //
// D1 and D2 carry sentinels ACROSS the call. They are clobbered inside --  //
// D1 is the running sum and D2 the saved count -- so if MOVEM's save and   //
// restore disagree by even one register, they come back holding sums       //
// instead of AAAAAAAA and BBBBBBBB. That is the check the unit benches     //
// cannot make, because they never put a register's value at risk.          //
//                                                                          //
// The loop also keeps both forwarding paths live: every ADD depends on the //
// A0 update from the ADD before it, and the SUBQ/BNE pair makes the branch //
// condition depend on the instruction immediately preceding it.            //
//                                                                          //
// The subroutine sits at $0800 and the array at $0900, both far past main, //
// because PROG_WORDS is an issue budget and not an address bound. With the  //
// subroutine adjacent, main's trailing NOPs walked into a second LINK; with //
// the array adjacent, they EXECUTED it -- 0000 000A is ORI.B #$0A,D0, and   //
// the four array words OR'd $3F into the result. Data a program does not    //
// jump over must not be where a program can walk into it.                  //
//                                                                          //
// This bench is not tied to one milestone, so it has no control commit.    //
// Its value is that it fails if any of thirty interact badly -- and on its //
// first run it did, finding a lost redirect in IF (see the commit).        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_integration2;

localparam PROG_WORDS      = 200;
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
	dut.u_l1.mem[1  ] = 16'h207C;   // MOVEA.L #$0900,A0
	dut.u_l1.mem[2  ] = 16'h0000;
	dut.u_l1.mem[3  ] = 16'h0900;
	dut.u_l1.mem[4  ] = 16'h223C;   // MOVE.L #$AAAAAAAA,D1
	dut.u_l1.mem[5  ] = 16'hAAAA;
	dut.u_l1.mem[6  ] = 16'hAAAA;
	dut.u_l1.mem[7  ] = 16'h243C;   // MOVE.L #$BBBBBBBB,D2
	dut.u_l1.mem[8  ] = 16'hBBBB;
	dut.u_l1.mem[9  ] = 16'hBBBB;
	dut.u_l1.mem[10 ] = 16'h7004;   // MOVEQ #4,D0
	dut.u_l1.mem[11 ] = 16'h6100;   // BSR.W average  ($0800)
	dut.u_l1.mem[12 ] = 16'h03E8;
	dut.u_l1.mem[13 ] = 16'h227C;   // MOVEA.L #$04A0,A1
	dut.u_l1.mem[14 ] = 16'h0000;
	dut.u_l1.mem[15 ] = 16'h04A0;
	dut.u_l1.mem[16 ] = 16'h2280;   // MOVE.L D0,(A1)
	dut.u_l1.mem[17 ] = 16'h4E71;   // NOP
	dut.u_l1.mem[640] = 16'h0000;   // array: 10
	dut.u_l1.mem[641] = 16'h000A;
	dut.u_l1.mem[642] = 16'h0000;   // 20
	dut.u_l1.mem[643] = 16'h0014;
	dut.u_l1.mem[644] = 16'h0000;   // 30
	dut.u_l1.mem[645] = 16'h001E;
	dut.u_l1.mem[646] = 16'h0000;   // 40
	dut.u_l1.mem[647] = 16'h0028;
	dut.u_l1.mem[512] = 16'h4E56;   // average: LINK A6,#0
	dut.u_l1.mem[513] = 16'h0000;
	dut.u_l1.mem[514] = 16'h48E7;   // MOVEM.L D1-D2,-(A7)
	dut.u_l1.mem[515] = 16'h6000;
	dut.u_l1.mem[516] = 16'h2400;   // MOVE.L D0,D2
	dut.u_l1.mem[517] = 16'h7200;   // MOVEQ #0,D1
	dut.u_l1.mem[518] = 16'hD298;   // loop: ADD.L (A0)+,D1
	dut.u_l1.mem[519] = 16'h5380;   // SUBQ.L #1,D0
	dut.u_l1.mem[520] = 16'h66FA;   // BNE.B loop
	dut.u_l1.mem[521] = 16'h2001;   // MOVE.L D1,D0
	dut.u_l1.mem[522] = 16'h80C2;   // DIVU.W D2,D0
	dut.u_l1.mem[523] = 16'h4CDF;   // MOVEM.L (A7)+,D1-D2
	dut.u_l1.mem[524] = 16'h0006;
	dut.u_l1.mem[525] = 16'h4E5E;   // UNLK A6
	dut.u_l1.mem[526] = 16'h4E75;   // RTS

end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// BSR, LINK and MOVEM all push, so A7 must point somewhere real. See
	// tb_ap040_pipe_move_mem.v's header for why the poke lands here.
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	// $04A0 is word index 80.
	if ({dut.u_l1.mem[80], dut.u_l1.mem[81]} !== 32'h0000_0019) begin
		errors = errors + 1;
		$display("FAIL: [$04A0] = %h%h, expected 00000019 (100/4 = 25; the subroutine must return AND store)",
		         dut.u_l1.mem[80], dut.u_l1.mem[81]);
	end
	if (dbg_d1 !== 32'hAAAA_AAAA) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected aaaaaaaa (MOVEM must restore what the subroutine clobbered)", dbg_d1);
	end
	if (dbg_d2 !== 32'hBBBB_BBBB) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected bbbbbbbb (MOVEM must restore what the subroutine clobbered)", dbg_d2);
	end
	if (dbg_d0 !== 32'h0000_0019) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000019 (the returned average)", dbg_d0);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (BSR, LINK and MOVEM must all unwind)", dut.u_cpu.u_regfile.isp);
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
