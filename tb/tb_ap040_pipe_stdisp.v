//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 91: a store with a  //
// displacement)                                                            //
//                                                                          //
// tb_ap040_pipe_stdisp.v - MOVE.sz Dn,(d16,An)                             //
//                                                                          //
// Destination mode 101 was the last one the MOVE family could not reach.   //
// Loads have had it since milestone 10 and the ALU family since milestone  //
// 40, and it is how every compiled function writes a local.                //
//                                                                          //
// No new operand plumbing. A store already takes its data from operand_a   //
// (the Dn at ir[2:0]) and its address from operand_b (the An at ir[11:9]), //
// read from opposite ends of the opcode. All that is new is an offset on   //
// the address, and the store branch was already adding one for the         //
// predecrement mode, so this selects a third value there.                  //
//                                                                          //
// Memory before:                                                           //
//   $0480 = 55555555   $0488 = 00000000   $0492 = 1234                     //
//   $04A0 = 00000000   $04B0 = FFFFFFFF   $04F8 = 11112222                 //
//   $0500 = 8888       $0508 = 7777       $0604 = 00000000                 //
//                                                                          //
//   MOVEA.L #$0480,A0 ; MOVE.L #$11223344,D0 ; MOVE.L D0,(8,A0)            //
//   MOVEA.L #$0500,A1 ; MOVE.W #$AABB,D1     ; MOVE.W D1,(-8,A1)           //
//   MOVEA.L #$0490,A2 ; MOVEQ #$CC,D2        ; MOVE.B D2,(3,A2)            //
//   MOVEA.L #$0600,A7 ; MOVEQ #$55,D3        ; MOVE.L D3,(4,A7)            //
//   MOVEA.L #$04A0,A4 ; MOVEQ #$66,D4        ; MOVE.L D4,(0,A4)            //
//   MOVEA.L #$04B0,A5 ; MOVEQ #0,D5 ; MOVEQ #$7F,D6                        //
//   MOVE.L D5,(0,A5) ; BNE.B +2 ; MOVEQ #$21,D6                            //
//                                                                          //
// Each case is aimed at a specific way this can go wrong:                   //
//                                                                          //
//   $0488 taking the data while $0480 keeps 55555555 is the whole point:   //
//     a decode that reaches mode 101 but drops the displacement writes at  //
//     A0 itself and both checks say so at once.                            //
//   The NEGATIVE displacement is the form that actually matters, since a   //
//     stack local is a negative offset off the frame pointer, and it has    //
//     three sentinels rather than one: $04F8 must take AABB, $0500 must     //
//     keep 8888 (the displacement dropped) and $0508 must keep 7777 (the    //
//     displacement added instead of subtracted).                           //
//                                                                          //
//   Memory contents cannot see every address error, and the sign is the    //
//     case where they cannot. ap040_pipe_l1.v indexes with address[12:0],  //
//     so the model wraps every 8 KB -- and a 16-bit displacement that is   //
//     zero-extended instead of sign-extended is off by exactly 65536,      //
//     which is a multiple of 8 KB. It lands on the SAME word, whatever     //
//     address is chosen, so no sentinel anywhere can catch it. The bench   //
//     therefore records the address the core DRIVES at each write post     //
//     and checks all six, which is the quantity in question and does not   //
//     depend on the memory model at all. The mutation that zero-extends    //
//     the store displacement is what showed this: it passed every value    //
//     check and both cores of the differential.                            //
//   $0492 keeping 12 in its high byte is the sized-lane check, with the    //
//     ODD address $0493 so the byte lands in the low half.                 //
//   $0604 with A7 as the base is the banking check: the address comes      //
//     through the ordinary destination port, so the supervisor stack       //
//     pointer is what must be read. A7 = $0600 afterwards is the other      //
//     half -- a store does not move its address register, and this is the   //
//     one instruction where a leaked register write would land on the      //
//     pointer rather than on a scratch data register.                       //
//   $04A0 with a ZERO displacement must match what the plain (An) form     //
//     would have done. It is the case a wrong offset select passes by      //
//     accident, which is why it is separate from the rest.                 //
//   D6 = $21 is the condition-code rule: MOVE to memory sets N and Z from   //
//     the DATA, and D5 is zero, so Z must be set. The poison is loaded      //
//     before the store and the marker is reached by NOT branching, the      //
//     shape milestone 89 had to correct.                                    //
//   Six write posts -- one per store, and the program has six -- is what   //
//     says none of them fired twice and no MOVEA reached memory. The       //
//     first draft of this bench expected five, which is the count being    //
//     off by one rather than the core storing twice: every one of the six  //
//     destinations above is separately checked and all six held.           //
//                                                                          //
// On milestone 90's RTL all five stores are illegal instructions.          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_stdisp;

localparam PROG_WORDS      = 60;
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
wire [31:0] dbg_d0, dbg_d6;
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

	.dbg_d0 (dbg_d0), .dbg_d6 (dbg_d6),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

// One count per store, whatever the request is held for -- see
// tb_ap040_pipe_immmem.v, where this check was established.
integer writes = 0;
reg [31:0] st_addr [0:7];
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy) begin
		if (writes < 8) st_addr[writes] = dut.u_cpu.l1_addr_b;
		writes = writes + 1;
	end

task check_addr;
	input integer n;
	input [31:0] want;
	input [1023:0] what;
	begin
		if (writes > n && st_addr[n] !== want) begin
			errors = errors + 1;
			$display("FAIL: store %0d drove address %h, expected %h (%0s)", n, st_addr[n], want, what);
		end
	end
endtask

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h207C;   // MOVEA.L #$00000480,A0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0480;
	dut.u_l1.mem[ 4] = 16'h203C;   // MOVE.L #$11223344,D0
	dut.u_l1.mem[ 5] = 16'h1122;
	dut.u_l1.mem[ 6] = 16'h3344;
	dut.u_l1.mem[ 7] = 16'h2140;   // MOVE.L D0,(8,A0)
	dut.u_l1.mem[ 8] = 16'h0008;
	dut.u_l1.mem[ 9] = 16'h227C;   // MOVEA.L #$00000500,A1
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h0500;
	dut.u_l1.mem[12] = 16'h323C;   // MOVE.W #$AABB,D1
	dut.u_l1.mem[13] = 16'hAABB;
	dut.u_l1.mem[14] = 16'h3341;   // MOVE.W D1,(-8,A1)
	dut.u_l1.mem[15] = 16'hFFF8;
	dut.u_l1.mem[16] = 16'h247C;   // MOVEA.L #$00000490,A2
	dut.u_l1.mem[17] = 16'h0000;
	dut.u_l1.mem[18] = 16'h0490;
	dut.u_l1.mem[19] = 16'h74CC;   // MOVEQ #$CC,D2
	dut.u_l1.mem[20] = 16'h1542;   // MOVE.B D2,(3,A2)
	dut.u_l1.mem[21] = 16'h0003;
	dut.u_l1.mem[22] = 16'h2E7C;   // MOVEA.L #$00000600,A7
	dut.u_l1.mem[23] = 16'h0000;
	dut.u_l1.mem[24] = 16'h0600;
	dut.u_l1.mem[25] = 16'h7655;   // MOVEQ #$55,D3
	dut.u_l1.mem[26] = 16'h2F43;   // MOVE.L D3,(4,A7)
	dut.u_l1.mem[27] = 16'h0004;
	dut.u_l1.mem[28] = 16'h287C;   // MOVEA.L #$000004A0,A4
	dut.u_l1.mem[29] = 16'h0000;
	dut.u_l1.mem[30] = 16'h04A0;
	dut.u_l1.mem[31] = 16'h7866;   // MOVEQ #$66,D4
	dut.u_l1.mem[32] = 16'h2944;   // MOVE.L D4,(0,A4)
	dut.u_l1.mem[33] = 16'h0000;
	dut.u_l1.mem[34] = 16'h2A7C;   // MOVEA.L #$000004B0,A5
	dut.u_l1.mem[35] = 16'h0000;
	dut.u_l1.mem[36] = 16'h04B0;
	dut.u_l1.mem[37] = 16'h7A00;   // MOVEQ #0,D5
	dut.u_l1.mem[38] = 16'h7C7F;   // MOVEQ #$7F,D6 (the "Z was clear" value)
	dut.u_l1.mem[39] = 16'h2B45;   // MOVE.L D5,(0,A5)
	dut.u_l1.mem[40] = 16'h0000;
	dut.u_l1.mem[41] = 16'h6602;   // BNE.B -> index 43, skipping the marker
	dut.u_l1.mem[42] = 16'h7C21;   // MOVEQ #$21,D6 (marker: only on Z)
	dut.u_l1.mem[43] = 16'h4E71;   // NOP (drain)

	dut.u_l1.mem[ 64] = 16'h5555;  // $0480 = 55555555 (must stay)
	dut.u_l1.mem[ 65] = 16'h5555;
	dut.u_l1.mem[ 68] = 16'h0000;  // $0488 = 00000000
	dut.u_l1.mem[ 69] = 16'h0000;
	dut.u_l1.mem[ 73] = 16'h1234;  // $0492 = 1234
	dut.u_l1.mem[ 80] = 16'h0000;  // $04A0 = 00000000
	dut.u_l1.mem[ 81] = 16'h0000;
	dut.u_l1.mem[ 88] = 16'hFFFF;  // $04B0 = FFFFFFFF
	dut.u_l1.mem[ 89] = 16'hFFFF;
	dut.u_l1.mem[124] = 16'h1111;  // $04F8 = 11112222
	dut.u_l1.mem[125] = 16'h2222;
	dut.u_l1.mem[128] = 16'h8888;  // $0500 (must stay)
	dut.u_l1.mem[132] = 16'h7777;  // $0508 (must stay)
	dut.u_l1.mem[258] = 16'h0000;  // $0604 = 00000000
	dut.u_l1.mem[259] = 16'h0000;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 80) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if ({dut.u_l1.mem[68], dut.u_l1.mem[69]} !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: $0488 = %h%h, expected 11223344 (MOVE.L D0,(8,A0))",
		         dut.u_l1.mem[68], dut.u_l1.mem[69]);
	end
	if ({dut.u_l1.mem[64], dut.u_l1.mem[65]} !== 32'h5555_5555) begin
		errors = errors + 1;
		$display("FAIL: $0480 = %h%h, expected 55555555 (the displacement was dropped and the store landed at A0 itself)",
		         dut.u_l1.mem[64], dut.u_l1.mem[65]);
	end
	if (dut.u_l1.mem[124] !== 16'hAABB || dut.u_l1.mem[125] !== 16'h2222) begin
		errors = errors + 1;
		$display("FAIL: $04F8 = %h%h, expected aabb2222 (MOVE.W D1,(-8,A1); a Word store must leave the next word alone)",
		         dut.u_l1.mem[124], dut.u_l1.mem[125]);
	end
	if (dut.u_l1.mem[128] !== 16'h8888) begin
		errors = errors + 1;
		$display("FAIL: $0500 = %h, expected 8888 (the negative displacement was dropped)", dut.u_l1.mem[128]);
	end
	if (dut.u_l1.mem[132] !== 16'h7777) begin
		errors = errors + 1;
		$display("FAIL: $0508 = %h, expected 7777 (the negative displacement was ADDED, not subtracted -- a missing sign extension)",
		         dut.u_l1.mem[132]);
	end
	if (dut.u_l1.mem[73] !== 16'h12CC) begin
		errors = errors + 1;
		$display("FAIL: $0492 = %h, expected 12cc (MOVE.B D2,(3,A2) writes the byte at the ODD address $0493 only)",
		         dut.u_l1.mem[73]);
	end
	if ({dut.u_l1.mem[258], dut.u_l1.mem[259]} !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: $0604 = %h%h, expected 00000055 (MOVE.L D3,(4,A7) must read the supervisor stack pointer as its base)",
		         dut.u_l1.mem[258], dut.u_l1.mem[259]);
	end
	if (dut.u_cpu.u_regfile.isp !== 32'h0000_0600) begin
		errors = errors + 1;
		$display("FAIL: A7 = %h, expected 00000600 (a store does not move its address register)",
		         dut.u_cpu.u_regfile.isp);
	end
	if ({dut.u_l1.mem[80], dut.u_l1.mem[81]} !== 32'h0000_0066) begin
		errors = errors + 1;
		$display("FAIL: $04A0 = %h%h, expected 00000066 (a ZERO displacement must match the plain (An) form)",
		         dut.u_l1.mem[80], dut.u_l1.mem[81]);
	end
	if ({dut.u_l1.mem[88], dut.u_l1.mem[89]} !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: $04B0 = %h%h, expected 00000000 (MOVE.L D5,(0,A5) with D5 zero)",
		         dut.u_l1.mem[88], dut.u_l1.mem[89]);
	end
	if (dbg_d6 !== 32'h0000_0021) begin
		errors = errors + 1;
		$display("FAIL: D6 = %h, expected 00000021 (0000007f means the store did not set Z from its DATA)", dbg_d6);
	end
	if (dut.u_cpu.u_regfile.areg[0] !== 32'h0000_0480 ||
	    dut.u_cpu.u_regfile.areg[1] !== 32'h0000_0500 ||
	    dut.u_cpu.u_regfile.areg[2] !== 32'h0000_0490) begin
		errors = errors + 1;
		$display("FAIL: A0/A1/A2 = %h/%h/%h, expected 00000480/00000500/00000490 (a store writes NO register, and its address register is where a leak would land)",
		         dut.u_cpu.u_regfile.areg[0], dut.u_cpu.u_regfile.areg[1], dut.u_cpu.u_regfile.areg[2]);
	end
	if (dbg_d0 !== 32'h1122_3344) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 11223344 (the source register must be untouched)", dbg_d0);
	end
	// Stores retire in program order, so this sequence is fixed. These are
	// the addresses the CORE drives, before the L1's 8 KB wrap.
	check_addr(0, 32'h0000_0488, "MOVE.L D0,(8,A0) with A0 = $0480");
	check_addr(1, 32'h0000_04F8, "MOVE.W D1,(-8,A1) with A1 = $0500; 000104f8 is a displacement zero-extended rather than sign-extended");
	check_addr(2, 32'h0000_0493, "MOVE.B D2,(3,A2) with A2 = $0490");
	check_addr(3, 32'h0000_0604, "MOVE.L D3,(4,A7) with A7 = $0600");
	check_addr(4, 32'h0000_04A0, "MOVE.L D4,(0,A4)");
	check_addr(5, 32'h0000_04B0, "MOVE.L D5,(0,A5)");

	if (writes !== 6) begin
		errors = errors + 1;
		$display("FAIL: %0d writes posted to the L1, expected 6 (one per store, and the program has six)", writes);
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
