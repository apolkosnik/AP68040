//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (restructuring plan, phase 1)  //
//                                                                          //
// tb_ap040_pipe_pushdep.v - a push straight behind the A7 it pushes onto   //
//                                                                          //
// The address hold (addr_hz) now names the ports whose address views are   //
// used. A push's is port B -- A7, through push_addr -- and BSR.S, JSR (An) //
// and PEA (An) are single words, so each can sit in EA-fetch while the     //
// instruction that set A7 is in EX, its result on the long forward. Every  //
// other form of those and every other push carries an extension word and  //
// decode's gather keeps it back a cycle; these do not. A push made from    //
// the stale A7 lands where the new A7 does not point, and RTS returns      //
// through whatever is there. Nothing else in the suite put them together,  //
// and the corpus runs one instruction per round.                          //
//                                                                          //
// The program, on the local array, A7 set by MOVEA.L, SUBQ, LEA and EXG:   //
//    org $400
//    lea ($2000).l,a3  ; the log
//    ; 1: MOVEA.L to A7, then BSR.S: the return address goes below the NEW A7
//    move.l #$1000,d0
//    movea.l d0,a7
//    bsr.s s1
//   r1: move.l a7,(a3)+  ; $1000 again
//    ; 2: SUBQ to A7, then PEA (A0)
//    lea ($1234).l,a0
//    subq.l #8,a7
//    pea (a0)   ; pushed at $1000-8-4 = $0FF4
//    move.l a7,(a3)+  ; $0FF4
//    addq.l #4,a7
//    addq.l #8,a7
//    ; 3: LEA (A1),A7, then JSR (A2)
//    lea ($1800).l,a1
//    lea (s3).l,a2
//    lea (a1),a7
//    jsr (a2)
//   r3: move.l a7,(a3)+  ; $1800
//    ; 4: EXG D1,A7, then BSR.S
//    move.l #$1400,d1
//    exg d1,a7
//    bsr.s s4
//   r4: move.l a7,(a3)+  ; $1400
//    ; 5: MOVEA.L to A7, then PEA (A7) -- the value pushed is the new A7 too
//    move.l #$1600,d0
//    movea.l d0,a7
//    pea (a7)
//    move.l (a7)+,(a3)+  ; $1600, and A7 back to $1600
//    move.l a7,(a3)+
//   halt: bra.s halt
//   s1: move.l (a7),(a3)+  ; the return address, read where RTS reads it
//    rts
//   s3: move.l (a7),(a3)+
//    rts
//   s4: move.l (a7),(a3)+
//    rts
//                                                                          //
// Checked: the log at $2000 -- each return address as the subroutine sees //
// it at (A7), and A7 after each case -- and each pushed longword where the //
// new A7 put it.                                                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_pushdep;

localparam PROG_WORDS      = 600;
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

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid (), .dbg_eac_pc (), .dbg_eaf_valid (), .dbg_eaf_pc (),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0 (), .dbg_d1 (), .dbg_d2 (), .dbg_ccr ()
);

integer errors = 0;

function [31:0] rdl;
	input [31:0] a;
	begin rdl = {dut.u_l1.mem[(a - 32'h400) >> 1], dut.u_l1.mem[((a - 32'h400) >> 1) + 1]}; end
endfunction
task wantl;
	input [31:0] a, v;
	input [8*12:1] what;
	begin
		if (rdl(a) !== v) begin
			errors = errors + 1;
			$display("FAIL: %0s: ($%h) = %h, want %h", what, a, rdl(a), v);
		end
	end
endtask

initial begin
	#1;
	dut.u_l1.mem[0] = 16'h47F9;
	dut.u_l1.mem[1] = 16'h0000;
	dut.u_l1.mem[2] = 16'h2000;
	dut.u_l1.mem[3] = 16'h203C;
	dut.u_l1.mem[4] = 16'h0000;
	dut.u_l1.mem[5] = 16'h1000;
	dut.u_l1.mem[6] = 16'h2E40;
	dut.u_l1.mem[7] = 16'h6140;
	dut.u_l1.mem[8] = 16'h26CF;
	dut.u_l1.mem[9] = 16'h41F9;
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h1234;
	dut.u_l1.mem[12] = 16'h518F;
	dut.u_l1.mem[13] = 16'h4850;
	dut.u_l1.mem[14] = 16'h26CF;
	dut.u_l1.mem[15] = 16'h588F;
	dut.u_l1.mem[16] = 16'h508F;
	dut.u_l1.mem[17] = 16'h43F9;
	dut.u_l1.mem[18] = 16'h0000;
	dut.u_l1.mem[19] = 16'h1800;
	dut.u_l1.mem[20] = 16'h45F9;
	dut.u_l1.mem[21] = 16'h0000;
	dut.u_l1.mem[22] = 16'h0454;
	dut.u_l1.mem[23] = 16'h4FD1;
	dut.u_l1.mem[24] = 16'h4E92;
	dut.u_l1.mem[25] = 16'h26CF;
	dut.u_l1.mem[26] = 16'h223C;
	dut.u_l1.mem[27] = 16'h0000;
	dut.u_l1.mem[28] = 16'h1400;
	dut.u_l1.mem[29] = 16'hC38F;
	dut.u_l1.mem[30] = 16'h611A;
	dut.u_l1.mem[31] = 16'h26CF;
	dut.u_l1.mem[32] = 16'h203C;
	dut.u_l1.mem[33] = 16'h0000;
	dut.u_l1.mem[34] = 16'h1600;
	dut.u_l1.mem[35] = 16'h2E40;
	dut.u_l1.mem[36] = 16'h4857;
	dut.u_l1.mem[37] = 16'h26DF;
	dut.u_l1.mem[38] = 16'h26CF;
	dut.u_l1.mem[39] = 16'h60FE;
	dut.u_l1.mem[40] = 16'h26D7;
	dut.u_l1.mem[41] = 16'h4E75;
	dut.u_l1.mem[42] = 16'h26D7;
	dut.u_l1.mem[43] = 16'h4E75;
	dut.u_l1.mem[44] = 16'h26D7;
	dut.u_l1.mem[45] = 16'h4E75;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);
	dut.u_cpu.sr = 16'h2700;
	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);
	wantl(32'h2000, 32'h00000410, "log 0");
	wantl(32'h2004, 32'h00001000, "log 1");
	wantl(32'h2008, 32'h00000FF4, "log 2");
	wantl(32'h200C, 32'h00000432, "log 3");
	wantl(32'h2010, 32'h00001800, "log 4");
	wantl(32'h2014, 32'h0000043E, "log 5");
	wantl(32'h2018, 32'h00001400, "log 6");
	wantl(32'h201C, 32'h00001600, "log 7");
	wantl(32'h2020, 32'h00001600, "log 8");
	wantl(32'h0FF4, 32'h00001234, "stack");
	wantl(32'h0FFC, 32'h00000410, "stack");
	wantl(32'h17FC, 32'h00000432, "stack");
	wantl(32'h13FC, 32'h0000043E, "stack");
	wantl(32'h15FC, 32'h00001600, "stack");
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_1600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, want 00001600", dut.u_cpu.u_regfile.isp);
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
