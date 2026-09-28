//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 116: bitfields)     //
//                                                                          //
// tb_ap040_pipe_bitfield.v - all eight, on a register and on memory        //
//                                                                          //
// Every expected value below comes from an independent Python model of the //
// 68k bitfield rules, not from the RTL: a register field is taken MSB-     //
// first from a bit offset and wraps within the 32 bits; a memory field     //
// starts at the byte ea + offset/8 (the offset signed) and spans up to     //
// five bytes.                                                              //
//                                                                          //
// Register, D0 = $89ABCDEF:                                                //
//   BFEXTU D0{4:8},D1         $0000009A                                    //
//   BFEXTS D0{D7:D6},D2       offset and width from registers (20, 12):    //
//                             $FFFFFDEF                                    //
//   BFFFO  D0{8:16},D3        $00000008 (the field's first bit is set)     //
//   BFCHG  D4{28:8}           wraps round bit 0: $79ABCDE0                 //
//   BFINS  D5,D6{24:8}        $0000005A                                    //
//   BFTST  D0{0:4}            N set ($600 SMI)                             //
// Memory, A0 = $800, bytes $7F0..$82F = address * $11 (low byte):          //
//   BFEXTU (A0){3:12},D1      $00000008                                    //
//   BFSET  (A0){30:12}        a three-byte span: $803..$805 = 33 FF D5     //
//   BFCLR  8(A0){7:32}        a five-byte span:  $808..$80C = 88 00 00 00 00 //
//   BFEXTS 16(A0){D7:8},D2    D7 = -12: the field starts two bytes BELOW   //
//                             the EA: $FFFFFFEF                            //
//   BFINS  D5,16(A0){D7:8}    $80E..$80F = EA 5F                           //
//   BFFFO  (0,A0,D4.W){0:16},D3   the EA is latched while port C is the    //
//                             index, before it reads offset and width: 2   //
//   BFTST  (d16,PC){0:8}      the PC base is the displacement word, which  //
//                             the bitfield word pushes to opcode + 4       //
//                             ($601 SMI)                                   //
// Register results are saved to $900-$913 before the memory half reuses   //
// the registers.                                                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_bitfield;

localparam PROG_WORDS      = 400;
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
wire [31:0] dbg_d1, dbg_d2, dbg_d3;
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
	.dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3), .dbg_sr (dbg_sr), .dbg_ccr(dbg_ccr)
);

integer errors = 0;
integer i;
integer a;

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

task chka;
	input [63:0] name;
	input [31:0] got;
	input [31:0] want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", name, got, want);
		end
	end
endtask

initial begin
	#1;
	for (i = 0; i < 4096; i = i + 1) dut.u_l1.mem[i] = 16'h4E71;   // NOP fill

	dut.u_l1.mem[ 0] = 16'h207C;  dut.u_l1.mem[ 1] = 16'h0000;  dut.u_l1.mem[ 2] = 16'h0800;  // MOVEA.L #$800,A0
	dut.u_l1.mem[ 3] = 16'h203C;  dut.u_l1.mem[ 4] = 16'h89AB;  dut.u_l1.mem[ 5] = 16'hCDEF;  // MOVE.L #$89ABCDEF,D0
	dut.u_l1.mem[ 6] = 16'h7E14;                                                             // MOVEQ #20,D7
	dut.u_l1.mem[ 7] = 16'h7C0C;                                                             // MOVEQ #12,D6
	dut.u_l1.mem[ 8] = 16'hE9C0;  dut.u_l1.mem[ 9] = 16'h1108;                               // BFEXTU D0{4:8},D1
	dut.u_l1.mem[10] = 16'hEBC0;  dut.u_l1.mem[11] = 16'h29E6;                               // BFEXTS D0{D7:D6},D2
	dut.u_l1.mem[12] = 16'hEDC0;  dut.u_l1.mem[13] = 16'h3210;                               // BFFFO D0{8:16},D3
	dut.u_l1.mem[14] = 16'h2800;                                                             // MOVE.L D0,D4
	dut.u_l1.mem[15] = 16'hEAC4;  dut.u_l1.mem[16] = 16'h0708;                               // BFCHG D4{28:8}
	dut.u_l1.mem[17] = 16'h7A5A;                                                             // MOVEQ #$5A,D5
	dut.u_l1.mem[18] = 16'hEFC6;  dut.u_l1.mem[19] = 16'h5608;                               // BFINS D5,D6{24:8}
	dut.u_l1.mem[20] = 16'hE8C0;  dut.u_l1.mem[21] = 16'h0004;                               // BFTST D0{0:4}
	dut.u_l1.mem[22] = 16'h5BF8;  dut.u_l1.mem[23] = 16'h0600;                               // SMI $0600.W
	dut.u_l1.mem[24] = 16'h21C1;  dut.u_l1.mem[25] = 16'h0900;                               // MOVE.L D1,$0900.W
	dut.u_l1.mem[26] = 16'h21C2;  dut.u_l1.mem[27] = 16'h0904;                               // MOVE.L D2,$0904.W
	dut.u_l1.mem[28] = 16'h21C3;  dut.u_l1.mem[29] = 16'h0908;                               // MOVE.L D3,$0908.W
	dut.u_l1.mem[30] = 16'h21C4;  dut.u_l1.mem[31] = 16'h090C;                               // MOVE.L D4,$090C.W
	dut.u_l1.mem[32] = 16'h21C6;  dut.u_l1.mem[33] = 16'h0910;                               // MOVE.L D6,$0910.W
	dut.u_l1.mem[34] = 16'hE9D0;  dut.u_l1.mem[35] = 16'h10CC;                               // BFEXTU (A0){3:12},D1
	dut.u_l1.mem[36] = 16'hEED0;  dut.u_l1.mem[37] = 16'h078C;                               // BFSET (A0){30:12}
	dut.u_l1.mem[38] = 16'hECE8;  dut.u_l1.mem[39] = 16'h01C0;  dut.u_l1.mem[40] = 16'h0008;  // BFCLR 8(A0){7:32}
	dut.u_l1.mem[41] = 16'h7EF4;                                                             // MOVEQ #-12,D7
	dut.u_l1.mem[42] = 16'hEBE8;  dut.u_l1.mem[43] = 16'h29C8;  dut.u_l1.mem[44] = 16'h0010;  // BFEXTS 16(A0){D7:8},D2
	dut.u_l1.mem[45] = 16'h7AA5;                                                             // MOVEQ #$A5,D5
	dut.u_l1.mem[46] = 16'hEFE8;  dut.u_l1.mem[47] = 16'h59C8;  dut.u_l1.mem[48] = 16'h0010;  // BFINS D5,16(A0){D7:8}
	dut.u_l1.mem[49] = 16'h7820;                                                             // MOVEQ #32,D4
	dut.u_l1.mem[50] = 16'hEDF0;  dut.u_l1.mem[51] = 16'h3010;  dut.u_l1.mem[52] = 16'h4000;  // BFFFO (0,A0,D4.W){0:16},D3
	dut.u_l1.mem[53] = 16'hE8FA;  dut.u_l1.mem[54] = 16'h0008;  dut.u_l1.mem[55] = 16'h0082;  // BFTST $4F0(PC){0:8}
	dut.u_l1.mem[56] = 16'h5BF8;  dut.u_l1.mem[57] = 16'h0601;                               // SMI $0601.W
	dut.u_l1.mem[58] = 16'h60FE;                                                             // BRA.B -2

	dut.u_l1.mem[120] = 16'h8000;                                  // $4F0: BFTST's byte, $80
	for (a = 32'h7F0; a < 32'h830; a = a + 2)
		dut.u_l1.mem[(a - 32'h400) / 2] = {a[7:0] * 8'h11, (a[7:0] + 8'h01) * 8'h11};
	dut.u_l1.mem[256] = 16'h0000;                                  // $600/$601
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 1200) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	chk("BFEXTU D0 ($900)",       640, 16'h0000);  chk("BFEXTU D0 ($902)", 641, 16'h009A);
	chk("BFEXTS D0 ($904)",       642, 16'hFFFF);  chk("BFEXTS D0 ($906)", 643, 16'hFDEF);
	chk("BFFFO  D0 ($908)",       644, 16'h0000);  chk("BFFFO  D0 ($90A)", 645, 16'h0008);
	chk("BFCHG  D4 ($90C)",       646, 16'h79AB);  chk("BFCHG  D4 ($90E)", 647, 16'hCDE0);
	chk("BFINS  D6 ($910)",       648, 16'h0000);  chk("BFINS  D6 ($912)", 649, 16'h005A);
	chk("BFTST N $600, pcrel $601", 256, 16'hFFFF);
	chk("BFSET $802/$803",        513, 16'h2233);
	chk("BFSET $804/$805",        514, 16'hFFD5);
	chk("BFSET $806 untouched",   515, 16'h6677);
	chk("BFCLR $808/$809",        516, 16'h8800);
	chk("BFCLR $80A/$80B",        517, 16'h0000);
	chk("BFCLR $80C/$80D",        518, 16'h00DD);
	chk("BFINS $80E/$80F",        519, 16'hEA5F);
	chka("D1 BFEXTU mem", dbg_d1, 32'h0000_0008);
	chka("D2 BFEXTS mem", dbg_d2, 32'hFFFF_FFEF);
	chka("D3 BFFFO mem",  dbg_d3, 32'h0000_0002);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
