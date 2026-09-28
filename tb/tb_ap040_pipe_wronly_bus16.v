//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (restructuring plan, phase 2)  //
//                                                                          //
// tb_ap040_pipe_wronly_bus16.v - CLR and Scc write memory without reading  //
//                                                                          //
// CLR and Scc were read-modify-writes: a whole read of the destination for //
// a value neither uses (the M68000PRM gives the preliminary read as the    //
// MC68000/MC68008's, CLR p. 4-74, Scc p. 4-173). They are stores now, made //
// by EX as a read-modify-write's store is, with no read first.             //
//                                                                          //
// ap040_pipe_bus16.v, MMU off. The destination window $2000-$21FF starts   //
// as $A5 in every byte and is READ-SENSITIVE: any read of it is an error,  //
// as a device register's would be. Checked, with ce on and pseudo-random:  //
//   - no read of the window at all;                                        //
//   - exactly the program's writes into it: 46 sub-cycles, a byte or word  //
//     one each, a longword two;                                            //
//   - every byte of the window: the targets written, every other byte --   //
//     the neighbours of each byte and word target above all -- still $A5;  //
//   - CLR's flags with X preset (X kept, Z set, N V C clear), logged by    //
//     MOVE CCR,(A3)+ after each of the eight forms;                        //
//   - the address registers' steps: (A1)+ by one, -(A2) by two, and        //
//     (A7)+ BYTE by two;                                                   //
//   - all sixteen conditions under two flag patterns, and two Scc straight //
//     behind the instruction producing their flags;                       //
//   - a CLR on the not-taken side of a forward branch -- fetched, decoded, //
//     never executed -- writes nothing.                                    //
//                                                                          //
// The program:                                                             //
//    org $400
//    lea ($2000).l,a0
//    lea ($2100).l,a1
//    lea ($2200).l,a2
//    lea ($3000).l,a3  ; the log
//    moveq #8,d4
//    move.w #$1F,ccr
//    clr.b (a0)
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.w 2(a0)
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.l 4(a0)
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.b (a1)+
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.w -(a2)
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.l 0(a0,d4.l)
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.l ($2010).w
//    move.w ccr,(a3)+
//    move.w #$1F,ccr
//    clr.b ($2014).l
//    move.w ccr,(a3)+
//    move.l a7,d6
//    lea ($2016).l,a7
//    clr.b (a7)+
//    move.l a7,(a3)+
//    move.l d6,a7
//    move.l a1,(a3)+
//    move.l a2,(a3)+
//    move.w #$04,ccr
//    st $20(a0)
//    sf $21(a0)
//    shi $22(a0)
//    sls $23(a0)
//    scc $24(a0)
//    scs $25(a0)
//    sne $26(a0)
//    seq $27(a0)
//    svc $28(a0)
//    svs $29(a0)
//    spl $2A(a0)
//    smi $2B(a0)
//    sge $2C(a0)
//    slt $2D(a0)
//    sgt $2E(a0)
//    sle $2F(a0)
//    move.w #$0B,ccr
//    st $30(a0)
//    sf $31(a0)
//    shi $32(a0)
//    sls $33(a0)
//    scc $34(a0)
//    scs $35(a0)
//    sne $36(a0)
//    seq $37(a0)
//    svc $38(a0)
//    svs $39(a0)
//    spl $3A(a0)
//    smi $3B(a0)
//    sge $3C(a0)
//    slt $3D(a0)
//    sgt $3E(a0)
//    sle $3F(a0)
//    moveq #5,d1
//    moveq #5,d2
//    cmp.l d1,d2
//    seq $40(a0)
//    moveq #0,d0
//    sne $41(a0)
//    moveq #1,d0
//    bne.s skip
//    clr.l $50(a0)
//    clr.l $54(a0)
//   skip:
//    move.w #$600D,($F102).l
//   halt: bra.s halt
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_wronly_bus16;

reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0;
reg ce = 1;
integer cemode = 0;
integer errors = 0;
integer ei;

reg [15:0] prog [0:255];
reg  [7:0] exp  [0:511];
// generated
localparam NPROG = 147;
localparam EXP_WRITES = 46;
initial begin
	prog[0] = 16'h41F9;
	prog[1] = 16'h0000;
	prog[2] = 16'h2000;
	prog[3] = 16'h43F9;
	prog[4] = 16'h0000;
	prog[5] = 16'h2100;
	prog[6] = 16'h45F9;
	prog[7] = 16'h0000;
	prog[8] = 16'h2200;
	prog[9] = 16'h47F9;
	prog[10] = 16'h0000;
	prog[11] = 16'h3000;
	prog[12] = 16'h7808;
	prog[13] = 16'h44FC;
	prog[14] = 16'h001F;
	prog[15] = 16'h4210;
	prog[16] = 16'h42DB;
	prog[17] = 16'h44FC;
	prog[18] = 16'h001F;
	prog[19] = 16'h4268;
	prog[20] = 16'h0002;
	prog[21] = 16'h42DB;
	prog[22] = 16'h44FC;
	prog[23] = 16'h001F;
	prog[24] = 16'h42A8;
	prog[25] = 16'h0004;
	prog[26] = 16'h42DB;
	prog[27] = 16'h44FC;
	prog[28] = 16'h001F;
	prog[29] = 16'h4219;
	prog[30] = 16'h42DB;
	prog[31] = 16'h44FC;
	prog[32] = 16'h001F;
	prog[33] = 16'h4262;
	prog[34] = 16'h42DB;
	prog[35] = 16'h44FC;
	prog[36] = 16'h001F;
	prog[37] = 16'h42B0;
	prog[38] = 16'h4800;
	prog[39] = 16'h42DB;
	prog[40] = 16'h44FC;
	prog[41] = 16'h001F;
	prog[42] = 16'h42B8;
	prog[43] = 16'h2010;
	prog[44] = 16'h42DB;
	prog[45] = 16'h44FC;
	prog[46] = 16'h001F;
	prog[47] = 16'h4239;
	prog[48] = 16'h0000;
	prog[49] = 16'h2014;
	prog[50] = 16'h42DB;
	prog[51] = 16'h2C0F;
	prog[52] = 16'h4FF9;
	prog[53] = 16'h0000;
	prog[54] = 16'h2016;
	prog[55] = 16'h421F;
	prog[56] = 16'h26CF;
	prog[57] = 16'h2E46;
	prog[58] = 16'h26C9;
	prog[59] = 16'h26CA;
	prog[60] = 16'h44FC;
	prog[61] = 16'h0004;
	prog[62] = 16'h50E8;
	prog[63] = 16'h0020;
	prog[64] = 16'h51E8;
	prog[65] = 16'h0021;
	prog[66] = 16'h52E8;
	prog[67] = 16'h0022;
	prog[68] = 16'h53E8;
	prog[69] = 16'h0023;
	prog[70] = 16'h54E8;
	prog[71] = 16'h0024;
	prog[72] = 16'h55E8;
	prog[73] = 16'h0025;
	prog[74] = 16'h56E8;
	prog[75] = 16'h0026;
	prog[76] = 16'h57E8;
	prog[77] = 16'h0027;
	prog[78] = 16'h58E8;
	prog[79] = 16'h0028;
	prog[80] = 16'h59E8;
	prog[81] = 16'h0029;
	prog[82] = 16'h5AE8;
	prog[83] = 16'h002A;
	prog[84] = 16'h5BE8;
	prog[85] = 16'h002B;
	prog[86] = 16'h5CE8;
	prog[87] = 16'h002C;
	prog[88] = 16'h5DE8;
	prog[89] = 16'h002D;
	prog[90] = 16'h5EE8;
	prog[91] = 16'h002E;
	prog[92] = 16'h5FE8;
	prog[93] = 16'h002F;
	prog[94] = 16'h44FC;
	prog[95] = 16'h000B;
	prog[96] = 16'h50E8;
	prog[97] = 16'h0030;
	prog[98] = 16'h51E8;
	prog[99] = 16'h0031;
	prog[100] = 16'h52E8;
	prog[101] = 16'h0032;
	prog[102] = 16'h53E8;
	prog[103] = 16'h0033;
	prog[104] = 16'h54E8;
	prog[105] = 16'h0034;
	prog[106] = 16'h55E8;
	prog[107] = 16'h0035;
	prog[108] = 16'h56E8;
	prog[109] = 16'h0036;
	prog[110] = 16'h57E8;
	prog[111] = 16'h0037;
	prog[112] = 16'h58E8;
	prog[113] = 16'h0038;
	prog[114] = 16'h59E8;
	prog[115] = 16'h0039;
	prog[116] = 16'h5AE8;
	prog[117] = 16'h003A;
	prog[118] = 16'h5BE8;
	prog[119] = 16'h003B;
	prog[120] = 16'h5CE8;
	prog[121] = 16'h003C;
	prog[122] = 16'h5DE8;
	prog[123] = 16'h003D;
	prog[124] = 16'h5EE8;
	prog[125] = 16'h003E;
	prog[126] = 16'h5FE8;
	prog[127] = 16'h003F;
	prog[128] = 16'h7205;
	prog[129] = 16'h7405;
	prog[130] = 16'hB481;
	prog[131] = 16'h57E8;
	prog[132] = 16'h0040;
	prog[133] = 16'h7000;
	prog[134] = 16'h56E8;
	prog[135] = 16'h0041;
	prog[136] = 16'h7001;
	prog[137] = 16'h6608;
	prog[138] = 16'h42A8;
	prog[139] = 16'h0050;
	prog[140] = 16'h42A8;
	prog[141] = 16'h0054;
	prog[142] = 16'h33FC;
	prog[143] = 16'h600D;
	prog[144] = 16'h0000;
	prog[145] = 16'hF102;
	prog[146] = 16'h60FE;
end
initial begin
	for (ei = 0; ei < 512; ei = ei + 1) exp[ei] = 8'hA5;
	exp[0] = 8'h00;
	exp[2] = 8'h00;
	exp[3] = 8'h00;
	exp[4] = 8'h00;
	exp[5] = 8'h00;
	exp[6] = 8'h00;
	exp[7] = 8'h00;
	exp[8] = 8'h00;
	exp[9] = 8'h00;
	exp[10] = 8'h00;
	exp[11] = 8'h00;
	exp[16] = 8'h00;
	exp[17] = 8'h00;
	exp[18] = 8'h00;
	exp[19] = 8'h00;
	exp[20] = 8'h00;
	exp[22] = 8'h00;
	exp[32] = 8'hFF;
	exp[33] = 8'h00;
	exp[34] = 8'h00;
	exp[35] = 8'hFF;
	exp[36] = 8'hFF;
	exp[37] = 8'h00;
	exp[38] = 8'h00;
	exp[39] = 8'hFF;
	exp[40] = 8'hFF;
	exp[41] = 8'h00;
	exp[42] = 8'hFF;
	exp[43] = 8'h00;
	exp[44] = 8'hFF;
	exp[45] = 8'h00;
	exp[46] = 8'h00;
	exp[47] = 8'hFF;
	exp[48] = 8'hFF;
	exp[49] = 8'h00;
	exp[50] = 8'h00;
	exp[51] = 8'hFF;
	exp[52] = 8'h00;
	exp[53] = 8'hFF;
	exp[54] = 8'hFF;
	exp[55] = 8'h00;
	exp[56] = 8'h00;
	exp[57] = 8'hFF;
	exp[58] = 8'h00;
	exp[59] = 8'hFF;
	exp[60] = 8'hFF;
	exp[61] = 8'h00;
	exp[62] = 8'hFF;
	exp[63] = 8'h00;
	exp[64] = 8'hFF;
	exp[65] = 8'h00;
	exp[256] = 8'h00;
	exp[510] = 8'h00;
	exp[511] = 8'h00;
end


reg [15:0] lfsr = 16'hACE1;
always @(negedge clk)
	if (nreset && cemode) begin
		lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
		ce   <= lfsr[0];
	end else ce <= 1'b1;

wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds, longword;
wire  [1:0] busstate;
wire  [2:0] fc;
reg         mem_ready;
wire        clkena_in = (busstate == `AP040_BUS_IDLE) | mem_ready;
reg  [15:0] mem [0:32767];
wire [15:0] data_in = mem[addr_out[15:1]];

ap040_pipe_bus16 #(.PC_RESET(32'h400), .PROG_WORDS(32'h7FFF_FFFF), .RESET_VECTORS(1)) dut
(
	.clk (clk), .nreset (nreset), .ce (ce), .irq_lvl (3'd0), .clkena_in (clkena_in),
	.berr (1'b0),
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // MMU off: no walks
	.data_in (data_in), .addr_out (addr_out), .data_write (data_write),
	.nwr (nwr), .nuds (nuds), .nlds (nlds), .busstate (busstate), .longword (longword), .fc (fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid (), .dbg_eac_pc (), .dbg_eaf_valid (), .dbg_eaf_pc (),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0 (), .dbg_d1 (), .dbg_d2 (), .dbg_d3 (), .dbg_d4 (), .dbg_d5 (), .dbg_d6 (), .dbg_d7 (),
	.dbg_ccr (), .dbg_sr (), .dbg_commits ()
);

wire in_window = (addr_out >= 32'h2000) && (addr_out < 32'h2200);
integer wwrites, wreads, done;
reg [1:0] dly;
always @(posedge clk) begin
	if (!nreset) begin
		mem_ready <= 1'b0; dly <= 2'd0;
	end else begin
		mem_ready <= 1'b0;
		if (busstate != `AP040_BUS_IDLE && !mem_ready) begin
			if (dly == 2'd0) begin
				mem_ready <= 1'b1;
				dly       <= lfsr[3:2];
				if (busstate == `AP040_BUS_READ && in_window) begin
					wreads = wreads + 1;
					$display("FAIL: ce %0d: a read of the read-sensitive window at %h", cemode, addr_out);
				end
				if (busstate == `AP040_BUS_WRITE) begin
					if (in_window) wwrites = wwrites + 1;
					if (addr_out == 32'hF102 && data_write == 16'h600D) done = 1;
					if (!nuds) mem[addr_out[15:1]][15:8] <= data_write[15:8];
					if (!nlds) mem[addr_out[15:1]][7:0]  <= data_write[7:0];
				end
			end else dly <= dly - 2'd1;
		end
	end
end

function [7:0] byte_at;
	input [31:0] a;
	begin byte_at = a[0] ? mem[a[15:1]][7:0] : mem[a[15:1]][15:8]; end
endfunction
function [31:0] long_at;
	input [31:0] a;
	begin long_at = {mem[a[15:1]], mem[a[15:1] + 15'd1]}; end
endfunction

task want;
	input [8*40:1] what;
	input [31:0] got, wanted;
	begin
		if (got !== wanted) begin
			errors = errors + 1;
			$display("FAIL: ce %0d: %0s = %h, want %h", cemode, what, got, wanted);
		end
	end
endtask

integer i, t;
initial begin
	for (cemode = 0; cemode < 2; cemode = cemode + 1) begin
		nreset = 0;
		for (i = 0; i < 32768; i = i + 1) mem[i] = 16'h4E71;
		for (i = 16'h1000; i < 16'h1100; i = i + 1) mem[i] = 16'hA5A5;    // the window, $2000-$21FF
		for (i = 16'h1800; i < 16'h1810; i = i + 1) mem[i] = 16'h0000;    // the log, $3000
		mem[0] = 16'h0000; mem[1] = 16'h1000; mem[2] = 16'h0000; mem[3] = 16'h0400;   // ISP $1000, PC $400
		for (i = 0; i < NPROG; i = i + 1) mem[16'h200 + i] = prog[i];
		wwrites = 0; wreads = 0; done = 0;
		repeat (5) @(posedge clk);
		nreset = 1;
		t = 0;
		while (t < 20000 && !done) begin @(posedge clk); t = t + 1; end
		repeat (50) @(posedge clk);
		if (!done) begin errors = errors + 1; $display("FAIL: ce %0d: the program did not finish", cemode); end
		errors = errors + wreads;
		if (wwrites != EXP_WRITES) begin
			errors = errors + 1;
			$display("FAIL: ce %0d: %0d write sub-cycles into the window, want %0d", cemode, wwrites, EXP_WRITES);
		end
		for (i = 0; i < 512; i = i + 1)
			if (byte_at(32'h2000 + i) !== exp[i]) begin
				errors = errors + 1;
				$display("FAIL: ce %0d: ($%h) = %h, want %h", cemode, 32'h2000 + i, byte_at(32'h2000 + i), exp[i]);
			end
		for (i = 0; i < 8; i = i + 1) want("CCR after a CLR (X preset)", mem[16'h1800 + i], 16'h0014);
		want("A7 after CLR.B (A7)+ from $2016", long_at(32'h3010), 32'h2018);
		want("A1 after CLR.B (A1)+ from $2100", long_at(32'h3014), 32'h2101);
		want("A2 after CLR.W -(A2) from $2200", long_at(32'h3018), 32'h21FE);
		$display("ce %0s: %0d window writes, %0d window reads", cemode ? "random" : "on", wwrites, wreads);
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
