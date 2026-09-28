//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 113: bit operations //
// on memory)                                                               //
//                                                                          //
// tb_ap040_pipe_bitmem.v - BTST/BCHG/BCLR/BSET beyond Dn                   //
//                                                                          //
// Until milestone 113 the bit operations named nothing but a data          //
// register. A memory operand is a BYTE, and its bit number is taken        //
// MODULO 8 -- the shared ALU uses a[4:0], as ap040_alu.v does, so          //
// ap040_execute.v masks it for a byte. Every bit number here is 8 or more, //
// so an unmasked one selects a bit outside the byte and changes nothing:   //
//                                                                          //
//   D1 = 11 (bit 3)  D2 = 13 (bit 5)  D3 = 16 (bit 0)  D6 = 2, the index   //
//                                                                          //
//   BSET  D1,(A0)            $800  $00 -> $08     dynamic, (An)            //
//   BCLR  D2,(A1)+           $810  $FF -> $DF     dynamic, (An)+ A1->$811  //
//   BCHG  D3,-(A2)           $821  $0F -> $0E     dynamic, -(An) A2->$821  //
//   BSET  #9,$4(A0)          $804  $00 -> $02     static, (d16,An)         //
//   BCLR  #15,$8(A0,D6.W)    $80A  $FF -> $7F     static, (d8,An,Xn)       //
//   BCHG  #12,$0830.W        $830  $00 -> $10     static, (xxx).W          //
//   BTST  D1,$C(A0)          $80C  $08: bit 3 set, Z := 0 -> SEQ D7 = $00  //
//   BSET  D2,$6(A0,D6.W)     $808  $00 -> $20     dynamic, (d8,An,Xn)      //
//   BCLR  D1,$0850.W         $850  $FF -> $F7     dynamic, (xxx).W         //
//   BTST  #10,$00000840.L    $840  $00: bit 2 clear, Z := 1               //
//                                                                          //
// The bit operations touch Z alone. ORI #$1B,CCR before them sets X, N, V  //
// and C, which must survive: the last BTST leaves CCR = $1F.               //
//                                                                          //
// BTST writes nothing back. It rides the read-modify-write carrier for its //
// operand crossover and ap040_ea_fetch.v drops the store -- which, since   //
// the byte would be written back unchanged, no VALUE check can see. So     //
// the bench counts L1 writes to the two BTST operands: there must be none. //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bitmem;

localparam PROG_WORDS      = 300;
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

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d4, dbg_d5, dbg_d7;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),
	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_d4 (dbg_d4), .dbg_d5 (dbg_d5), .dbg_d7 (dbg_d7),
	.dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;
integer btst_writes = 0;

// The L1 latches a write exactly when wren_b is high and its one-entry
// buffer is empty (tb_ap040_pipe_storeonce.v).
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy &&
	    (dut.u_l1.address_b == 32'h0000_080C || dut.u_l1.address_b == 32'h0000_0840))
		btst_writes = btst_writes + 1;

task chk;
	input [255:0] what;
	input integer widx;
	input  [15:0] want;
	begin
		if (dut.u_l1.mem[widx] !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %04x, expected %04x", what, dut.u_l1.mem[widx], want);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h207C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h0800;  // MOVEA.L #$800,A0
	dut.u_l1.mem[ 3] = 16'h227C;  dut.u_l1.mem[ 4] = 16'h0000;  dut.u_l1.mem[ 5] = 16'h0810;  // MOVEA.L #$810,A1
	dut.u_l1.mem[ 6] = 16'h247C;  dut.u_l1.mem[ 7] = 16'h0000;  dut.u_l1.mem[ 8] = 16'h0822;  // MOVEA.L #$822,A2
	dut.u_l1.mem[ 9] = 16'h7C02;   // MOVEQ #2,D6
	dut.u_l1.mem[10] = 16'h720B;   // MOVEQ #11,D1
	dut.u_l1.mem[11] = 16'h740D;   // MOVEQ #13,D2
	dut.u_l1.mem[12] = 16'h7610;   // MOVEQ #16,D3
	dut.u_l1.mem[13] = 16'h003C;  dut.u_l1.mem[14] = 16'h001B;   // ORI #$1B,CCR

	dut.u_l1.mem[15] = 16'h03D0;                                 // BSET  D1,(A0)
	dut.u_l1.mem[16] = 16'h0599;                                 // BCLR  D2,(A1)+
	dut.u_l1.mem[17] = 16'h0762;                                 // BCHG  D3,-(A2)
	dut.u_l1.mem[18] = 16'h08E8;  dut.u_l1.mem[19] = 16'h0009;   // BSET  #9,$4(A0)
	dut.u_l1.mem[20] = 16'h0004;
	dut.u_l1.mem[21] = 16'h08B0;  dut.u_l1.mem[22] = 16'h000F;   // BCLR  #15,$8(A0,D6.W)
	dut.u_l1.mem[23] = 16'h6008;
	dut.u_l1.mem[24] = 16'h0878;  dut.u_l1.mem[25] = 16'h000C;   // BCHG  #12,$0830.W
	dut.u_l1.mem[26] = 16'h0830;
	dut.u_l1.mem[27] = 16'h0328;  dut.u_l1.mem[28] = 16'h000C;   // BTST  D1,$C(A0)
	dut.u_l1.mem[29] = 16'h57C7;                                 // SEQ   D7
	dut.u_l1.mem[30] = 16'h05F0;  dut.u_l1.mem[31] = 16'h6006;   // BSET  D2,$6(A0,D6.W)
	dut.u_l1.mem[32] = 16'h03B8;  dut.u_l1.mem[33] = 16'h0850;   // BCLR  D1,$0850.W
	dut.u_l1.mem[34] = 16'h2809;                                 // MOVE.L A1,D4
	dut.u_l1.mem[35] = 16'h2A0A;                                 // MOVE.L A2,D5
	dut.u_l1.mem[36] = 16'h003C;  dut.u_l1.mem[37] = 16'h001B;   // ORI #$1B,CCR (the MOVEs set flags)
	dut.u_l1.mem[38] = 16'h0839;  dut.u_l1.mem[39] = 16'h000A;   // BTST  #10,$00000840.L
	dut.u_l1.mem[40] = 16'h0000;  dut.u_l1.mem[41] = 16'h0840;
	dut.u_l1.mem[42] = 16'h60FE;                                 // BRA.B -2

	// Operands. Word index = (address - $400) / 2; an even address is the
	// high byte. The neighbouring bytes are markers.
	dut.u_l1.mem[512] = 16'h00AA;   // $800
	dut.u_l1.mem[514] = 16'h0055;   // $804
	dut.u_l1.mem[516] = 16'h0033;   // $808
	dut.u_l1.mem[517] = 16'hFF44;   // $80A
	dut.u_l1.mem[518] = 16'h0822;   // $80C
	dut.u_l1.mem[520] = 16'hFF11;   // $810
	dut.u_l1.mem[528] = 16'h990F;   // $820/$821
	dut.u_l1.mem[536] = 16'h0066;   // $830
	dut.u_l1.mem[544] = 16'h0077;   // $840
	dut.u_l1.mem[552] = 16'hFF88;   // $850
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 600) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("BSET D1,(A0)         $800",  512, 16'h08AA);
	chk("BCLR D2,(A1)+        $810",  520, 16'hDF11);
	chk("BCHG D3,-(A2)        $821",  528, 16'h990E);
	chk("BSET #9,$4(A0)       $804",  514, 16'h0255);
	chk("BCLR #15,$8(A0,D6)   $80A",  517, 16'h7F44);
	chk("BCHG #12,$0830.W     $830",  536, 16'h1066);
	chk("BTST D1,$C(A0)       $80C",  518, 16'h0822);
	chk("BSET D2,$6(A0,D6)    $808",  516, 16'h2033);
	chk("BCLR D1,$0850.W      $850",  552, 16'hF788);
	chk("BTST #10,$840.L      $840",  544, 16'h0077);

	if (dbg_d4 !== 32'h0000_0811) begin
		errors = errors + 1;
		$display("FAIL: A1 = %h, expected 00000811 ((A1)+ steps a byte)", dbg_d4);
	end
	if (dbg_d5 !== 32'h0000_0821) begin
		errors = errors + 1;
		$display("FAIL: A2 = %h, expected 00000821 (-(A2) steps a byte)", dbg_d5);
	end
	if (dbg_d7[7:0] !== 8'h00) begin
		errors = errors + 1;
		$display("FAIL: SEQ after BTST D1,$C(A0) = %h, expected 00 (bit 3 of $08 is set, so Z is clear)",
		         dbg_d7[7:0]);
	end
	if (dbg_ccr !== 5'h1F) begin
		errors = errors + 1;
		$display("FAIL: CCR = %h, expected 1F (the last BTST sets Z; X, N, V and C are left alone)",
		         dbg_ccr);
	end
	if (btst_writes != 0) begin
		errors = errors + 1;
		$display("FAIL: %0d L1 write(s) to a BTST operand; BTST writes nothing back", btst_writes);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
