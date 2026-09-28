//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040 pipelined core (integration, after milestone 76)    //
//                                                                          //
// tb_ap040_pipe_integration4.v - a user-mode round trip through every      //
// exception this core can raise, with the handlers reading their frames    //
//                                                                          //
// The two earlier integration benches found four pipeline bugs that eighty //
// unit benches had missed, each at a seam between features. The seams here //
// are the ones that opened since: supervisor/user switching, the stack     //
// bank each side uses, and the frames the dynamic exceptions push.         //
//                                                                          //
//   supervisor:  MOVEQ #0,D0 / MOVE D0,SR           S := 0, A7 := USP      //
//   user:        MOVEM.L D1-D2,-(A7) ... (A7)+      on the USER stack      //
//                TRAP #1                             vector 33, format $0   //
//                CHK.W #10,D0 (D0 = 20)              vector 6,  format $2   //
//                CMPI / TRAPEQ                       vector 7,  format $2   //
//                DIVU.W #0,D0                        vector 5,  format $2   //
//                ORI #$0700,SR                       vector 8,  format $0:  //
//                                                    the handler skips it   //
//                MOVEQ #$55,D5                                              //
//                TRAP #0                             vector 32, and stays   //
//                                                    in supervisor          //
//                                                                          //
// Every handler bumps D7 and ORs a bit into D6 for each fact about its own  //
// frame that holds. An OS reads these frames, so they are checked the way  //
// one would: format/vector word, stacked PC, and for the six-word frames   //
// the instruction address field at 8(A7):                                   //
//                                                                          //
//   bit 0   TRAP #1: format/vector word $0084                              //
//   bit 1   TRAP #1: stacked SR has S clear (the trap came from user mode)  //
//   bit 2   CHK:     format/vector word $2018                              //
//   bit 3   CHK:     address field = the CHK instruction ($0426)           //
//   bit 4   CHK:     stacked PC = the instruction after it ($042A)         //
//   bit 5   TRAPEQ:  format/vector word $201C                              //
//   bit 6   TRAPEQ:  address field = the TRAPEQ ($0430)                    //
//   bit 7   DIVU #0: format/vector word $2014                              //
//   bit 8   DIVU #0: address field = the DIVU ($0432)                      //
//   bit 9   ORI,SR:  format/vector word $0020                              //
//   bit 10  ORI,SR:  stacked PC = the ORI itself ($0436), which the        //
//                    handler then advances by 4 so the RTE lands past it   //
//   bit 11  TRAP #0: format/vector word $0080                              //
//   bit 12  TRAP #0: stacked SR has S clear                                //
//                                                                          //
// The 68040 pushes the six-word format-$2 frame for CHK, TRAPcc and zero   //
// divide, with the faulting instruction's address in the extra longword;   //
// rtl/ap040/ap040_core.v does the same (exc(..., 4'd2, pc, pc_i)) and      //
// passes cputest with it. The RTE in each handler pops whatever the format  //
// word says, so a wrong format is self-consistent and only the frame        //
// contents show it -- which is why the handlers look.                       //
//                                                                          //
// D7 = 6 says every handler ran once. D5 = $55 says the user code reached   //
// its end, so each RTE returned to user mode at the right place. D4 = $33   //
// says the TRAP #0 handler finished. D1/D2 say the MOVEM round trip on the  //
// user stack returned the values. USP = $0500 and ISP = $0600 say both      //
// stacks balanced (the last handler discards its frame with LEA 8(A7),A7).  //
// S set and T clear at the end says the last transition stuck.             //
//                                                                          //
// On milestone-76 RTL, D6 = $1E13: bits 2, 3, 5, 6, 7 and 8. CHK, TRAPcc  //
// and zero divide pushed format $0 with no address field. Everything else  //
// in the round trip held -- 1 of 10 checks failed, and it named the seam.   //
//                                                                          //
// Vectors: n sits at word index 3584 + 2n (see tb_ap040_pipe_rte_fmt2.v).   //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_integration4;

localparam PROG_WORDS      = 400;
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
integer b;

task check32;
	input string  what;
	input [31:0]  got, want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", what, got, want);
		end
	end
endtask

// A handler: bump D7, then a series of (load, compare, BNE.B +4, ORI bit)
// checks, then the tail. Written out as words so the listing above is the
// truth; the tasks below only save repeating the encodings.
integer p;
task w;      input [15:0] v; begin dut.u_l1.mem[p] = v; p = p + 1; end endtask
// MOVE.W 6(A7),D0 / CMPI.W #fv,D0 / BNE.B skip / ORI.W #bit,D6
task chk_fmtvec; input [15:0] fv; input [15:0] flag;
	begin w(16'h302F); w(16'h0006); w(16'h0C40); w(fv); w(16'h6604); w(16'h0046); w(flag); end endtask
// MOVE.L d(A7),D0 / CMPI.L #addr,D0 / BNE.B skip / ORI.W #bit,D6
task chk_long; input [15:0] d; input [31:0] addr; input [15:0] flag;
	begin w(16'h202F); w(d); w(16'h0C80); w(addr[31:16]); w(addr[15:0]); w(16'h6604); w(16'h0046); w(flag); end endtask
// MOVE.W (A7),D0 / ANDI.W #$2000,D0 / BNE.B skip (S set: not from user) / ORI.W #bit,D6
task chk_user_sr; input [15:0] flag;
	begin w(16'h3017); w(16'h0240); w(16'h2000); w(16'h6604); w(16'h0046); w(flag); end endtask

initial begin
	#1;
	// ---- supervisor prologue, then user code ------------------------------
	p = 1;
	w(16'h7C00);                          // MOVEQ #0,D6      frame-fact bits
	w(16'h7E00);                          // MOVEQ #0,D7      handler entries
	w(16'h7000);                          // MOVEQ #0,D0
	w(16'h46C0);                          // MOVE D0,SR       S := 0 -> user mode, A7 := USP
	// $0408: user mode from here
	w(16'h223C); w(16'h1111); w(16'h2222); // MOVE.L #$11112222,D1
	w(16'h243C); w(16'h3333); w(16'h4444); // MOVE.L #$33334444,D2
	w(16'h48E7); w(16'h6000);             // MOVEM.L D1-D2,-(A7)   on the user stack
	w(16'h7200);                          // MOVEQ #0,D1
	w(16'h7400);                          // MOVEQ #0,D2
	w(16'h4CDF); w(16'h0006);             // MOVEM.L (A7)+,D1-D2
	w(16'h4E41);                          // TRAP #1          @ $0422 -> vector 33
	w(16'h7014);                          // MOVEQ #20,D0
	w(16'h41BC); w(16'h000A);             // CHK.W #10,D0     @ $0426 -> vector 6; next is $042A
	w(16'h7005);                          // MOVEQ #5,D0      @ $042A
	w(16'h0C40); w(16'h0005);             // CMPI.W #5,D0     Z := 1
	w(16'h57FC);                          // TRAPEQ           @ $0430 -> vector 7
	w(16'h80FC); w(16'h0000);             // DIVU.W #0,D0     @ $0432 -> vector 5
	w(16'h007C); w(16'h0700);             // ORI #$0700,SR    @ $0436 -> vector 8 (privileged); handler skips it
	w(16'h7A55);                          // MOVEQ #$55,D5    @ $043A  user code reached its end
	w(16'h4E40);                          // TRAP #0          -> vector 32, no return
	w(16'h4E71);                          // NOP (never reached)
	if (p != 32) $display("FAIL: user program assembled to %0d words, listing assumes 31", p - 1);

	// ---- handlers -----------------------------------------------------------
	p = 512;                              // $0800: TRAP #1
	w(16'h5287);                          // ADDQ.L #1,D7
	chk_fmtvec(16'h0084, 16'h0001);
	chk_user_sr(16'h0002);
	w(16'h4E73);                          // RTE

	p = 544;                              // $0840: CHK
	w(16'h5287);
	chk_fmtvec(16'h2018, 16'h0004);
	chk_long(16'h0008, 32'h0000_0426, 16'h0008);   // address field: the CHK
	chk_long(16'h0002, 32'h0000_042A, 16'h0010);   // stacked PC: after it
	w(16'h4E73);

	p = 576;                              // $0880: TRAPEQ
	w(16'h5287);
	chk_fmtvec(16'h201C, 16'h0020);
	chk_long(16'h0008, 32'h0000_0430, 16'h0040);
	w(16'h4E73);

	p = 608;                              // $08C0: DIVU #0
	w(16'h5287);
	chk_fmtvec(16'h2014, 16'h0080);
	chk_long(16'h0008, 32'h0000_0432, 16'h0100);
	w(16'h4E73);

	p = 640;                              // $0900: privilege violation
	w(16'h5287);
	chk_fmtvec(16'h0020, 16'h0200);
	chk_long(16'h0002, 32'h0000_0436, 16'h0400);   // stacked PC: the ORI itself
	w(16'h41EF); w(16'h0002);             // LEA 2(A7),A0
	w(16'h2010);                          // MOVE.L (A0),D0
	w(16'h5880);                          // ADDQ.L #4,D0
	w(16'h2080);                          // MOVE.L D0,(A0)   skip the four-byte ORI
	w(16'h4E73);

	p = 672;                              // $0940: TRAP #0 -- finish in supervisor
	w(16'h5287);
	chk_fmtvec(16'h0080, 16'h0800);
	chk_user_sr(16'h1000);
	w(16'h4FEF); w(16'h0008);             // LEA 8(A7),A7     discard the frame
	w(16'h7833);                          // MOVEQ #$33,D4
	w(16'h4E71);                          // NOP

	// ---- vectors ------------------------------------------------------------
	dut.u_l1.mem[3594] = 16'h0000; dut.u_l1.mem[3595] = 16'h08C0;   // 5  zero divide
	dut.u_l1.mem[3596] = 16'h0000; dut.u_l1.mem[3597] = 16'h0840;   // 6  CHK
	dut.u_l1.mem[3598] = 16'h0000; dut.u_l1.mem[3599] = 16'h0880;   // 7  TRAPcc
	dut.u_l1.mem[3600] = 16'h0000; dut.u_l1.mem[3601] = 16'h0900;   // 8  privilege violation
	dut.u_l1.mem[3648] = 16'h0000; dut.u_l1.mem[3649] = 16'h0940;   // 32 TRAP #0
	dut.u_l1.mem[3650] = 16'h0000; dut.u_l1.mem[3651] = 16'h0800;   // 33 TRAP #1
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;
	dut.u_cpu.u_regfile.usp = 32'h0000_0500;

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dut.u_cpu.u_regfile.dreg[6] !== 32'h0000_1FFF) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 00001fff -- frame facts that did not hold:", dut.u_cpu.u_regfile.dreg[6]);
		for (b = 0; b < 13; b = b + 1)
			if (!dut.u_cpu.u_regfile.dreg[6][b]) $display("      bit %0d", b);
	end
	check32("D7 (handler entries)",                       dut.u_cpu.u_regfile.dreg[7], 32'h0000_0006);
	check32("D5 (user code reached its end)",             dut.u_cpu.u_regfile.dreg[5], 32'h0000_0055);
	check32("D4 (TRAP #0 handler finished)",              dut.u_cpu.u_regfile.dreg[4], 32'h0000_0033);
	check32("D1 (MOVEM round trip on the user stack)",    dbg_d1,                32'h1111_2222);
	check32("D2 (MOVEM round trip on the user stack)",    dbg_d2,                32'h3333_4444);
	check32("USP",                                        dut.u_cpu.u_regfile.usp,     32'h0000_0500);
	check32("ISP",                                        dut.u_cpu.u_regfile.isp,     32'h0000_0600);
	check32("SR S bit at the end",                        {31'd0, dut.u_cpu.sr[13]},   32'h0000_0001);
	check32("SR T bits at the end",                       {30'd0, dut.u_cpu.sr[15:14]}, 32'h0000_0000);

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
