//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: full-format EAs)  //
//                                                                          //
// tb_ap040_pipe_fxdual.v - the full-format extension on both cores        //
//                                                                          //
// The cputest groups FFEXT_SRC/FFEXT_DST put the full-format extension    //
// word on NOT, MOVE and ADD. This bench puts it on everything else that   //
// takes a memory EA -- loads and stores in every size, register and       //
// memory arithmetic, immediate-to-memory, LEA, PEA, MOVEM, the bitfields,  //
// CAS, MULU.L, CHK.L, the FPU, and MOVE memory-to-memory with the full     //
// format on either side and destination words after a full-format source //
// -- and runs the same generated program on the pipelined core            //
// and on rtl/ap040/ap040_core.v, comparing what each leaves in memory      //
// (tb_ap040_pipe_dual.v's method).                                         //
//                                                                          //
// Every shape the format has is generated: base suppressed or not, index  //
// suppressed or not, word or long index with each scale, null/word/long    //
// base displacement, no indirection, pre- and post-indexed indirection     //
// with null/word/long outer displacement, index-suppressed indirection    //
// (I/IS 001-011, and 101-111, which ap040_core.v executes as post-indexed  //
// with no index), and now and then a reserved encoding, which must be an   //
// illegal instruction on both. Indirect pointers come from a table at      //
// TABLE; every address lands in the compared scratch region.               //
//                                                                          //
// D0-D3 are the index registers, small and never written; D4-D7 take      //
// results; A0 is the table, A1 the scratch base, A2 a (A2)+ pointer, A5/A6 //
// take LEA results. Every instruction is preceded by                       //
//   MOVE.L #<the address after it>,RESUME.W                                //
// and the one handler logs each frame and resumes there, so an exception  //
// is compared like any other result.                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_fxdual;

localparam integer NINSN     = 90;
localparam integer NROUND    = 24;
localparam [31:0] PROG_BASE  = 32'h0000_0400;
localparam [31:0] HANDLER    = 32'h0000_0300;
localparam [31:0] RESUME     = 32'h0000_1200;
localparam [31:0] LOGPTR     = 32'h0000_1204;
localparam [31:0] LOG_BASE   = 32'h0000_1300;
localparam [31:0] LOG_END    = 32'h0000_1700;
localparam [31:0] DUMP_BASE  = 32'h0000_1000;
localparam [31:0] DONE_ADDR  = 32'h0000_1100;
localparam [31:0] STACK_TOP  = 32'h0000_3000;
localparam [31:0] TABLE      = 32'h0000_3800;   // A0: 64 pointers into the scratch region
localparam [31:0] SCRATCH    = 32'h0000_5000;   // A1
localparam [31:0] SCR_LO     = 32'h0000_4000;
localparam [31:0] SCR_HI     = 32'h0000_7FFE;
localparam integer MEM_WORDS = 32768;
localparam integer TIMEOUT   = 600000;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

reg [15:0] prog [0:4095];
integer    pw;
reg [31:0] rnd;

function [31:0] xorshift32;
	input [31:0] s;
	reg   [31:0] t;
	begin
		t = s; t = t ^ (t << 13); t = t ^ (t >> 17); t = t ^ (t << 5);
		xorshift32 = t;
	end
endfunction

function [31:0] rbits;
	input integer n;
	begin
		rnd = xorshift32(rnd);
		rbits = (n >= 32) ? rnd : (rnd & ((32'd1 << n) - 32'd1));
	end
endfunction

task emit;
	input [15:0] w;
	begin prog[pw] = w; pw = pw + 1; end
endtask

function [31:0] here;
	input integer dummy;
	begin here = PROG_BASE + 2 * pw; end
endfunction

reg [15:0] ib [0:15];
integer    in_;
task ib_put;
	input [15:0] w;
	begin ib[in_] = w; in_ = in_ + 1; end
endtask
task ib_flush;
	integer k;
	reg [31:0] ra;
	begin
		ra = here(0) + 8 + 2 * in_;   // before the first word moves here()
		emit(16'h21FC); emit(ra[31:16]); emit(ra[15:0]); emit(RESUME[15:0]);
		for (k = 0; k < in_; k = k + 1) emit(ib[k]);
		in_ = 0;
	end
endtask

// A full-format EA: fx_reg is the base register for the opcode's EA field
// (mode 110), fx_w[0..fx_n-1] the extension word and its displacements.
// r1..r4 carry rbits() results, which are 32 bits wide and would push the
// other fields out of any concatenation they were dropped into directly.
reg  [15:0] fx_w [0:4];
integer     fx_n, fx_reg;
integer     r1, r2, r3, r4, shape, bdsz, odsz, xi, sc, wl;
reg  [31:0] bd, od;

task put_disp;
	input integer sz;     // 1 null, 2 word, 3 long
	input [31:0] v;
	begin
		if (sz == 2) begin fx_w[fx_n] = v[15:0]; fx_n = fx_n + 1; end
		else if (sz == 3) begin fx_w[fx_n] = v[31:16]; fx_w[fx_n + 1] = v[15:0]; fx_n = fx_n + 2; end
	end
endtask

task gen_fx;
	reg bs, is;
	reg [2:0] iis;
	begin
		shape = rbits(4);
		xi = rbits(2);                 // D0-D3
		sc = rbits(2);
		wl = rbits(1);
		bs = 0; is = 0; iis = 3'b000;
		odsz = 1;
		case (shape)
		0, 1, 2: begin                 // A1 + Xn*s + bd, either sign
			fx_reg = 1; bdsz = 1 + (rbits(32) % 3); r1 = rbits(9); bd = {r1[8:1], 1'b0};
			if (rbits(1)) bd = -bd;
		end
		3: begin                       // A1 + bd, the index suppressed, either sign
			fx_reg = 1; is = 1; bdsz = 2 + rbits(1); r1 = rbits(10); bd = {r1[9:1], 1'b0};
			if (rbits(1)) bd = -bd;
		end
		4, 5: begin                    // bd.L + Xn*s, the base suppressed
			fx_reg = 1; bs = 1; bdsz = 3; r1 = rbits(10); bd = SCRATCH + {r1[9:1], 1'b0};
		end
		6, 7: begin                    // ([A0,Xn*4,bd],od): pre-indexed
			fx_reg = 0; sc = 2; bdsz = 1 + (rbits(32) % 3); r1 = rbits(5); bd = 4 * r1;
			odsz = 1 + (rbits(32) % 3); iis = {1'b0, odsz[1:0]};
		end
		8, 9: begin                    // ([A0,bd],Xn*s,od): post-indexed
			fx_reg = 0; bdsz = 1 + (rbits(32) % 3); r1 = rbits(5); bd = 4 * r1;
			odsz = 1 + (rbits(32) % 3); iis = {1'b1, odsz[1:0]};
		end
		10: begin                      // ([A0,bd],od), the index suppressed
			fx_reg = 0; is = 1; bdsz = 2; r1 = rbits(5); bd = 4 * r1;
			odsz = 1 + (rbits(32) % 3); iis = {1'b0, odsz[1:0]};
		end
		11: begin                      // index suppressed, I/IS 101-111
			fx_reg = 0; is = 1; bdsz = 2; r1 = rbits(5); bd = 4 * r1;
			odsz = 1 + (rbits(32) % 3); iis = {1'b1, odsz[1:0]};
		end
		12: begin                      // ([bd.L],od): base and index suppressed
			fx_reg = 0; bs = 1; is = 1; bdsz = 3; r1 = rbits(5); bd = TABLE + 4 * r1;
			odsz = 1 + (rbits(32) % 3); iis = {1'b0, odsz[1:0]};
		end
		13: begin                      // null bd, word index
			fx_reg = 1; bdsz = 1; bd = 0; wl = 0;
		end
		14: begin                      // a reserved encoding: I/IS 100, or BD SIZE 00
			fx_reg = 1; bdsz = rbits(1) ? 0 : 2; bd = 0; iis = (bdsz == 0) ? 3'b000 : 3'b100;
		end
		default: begin                 // ([A0,Xn*4],od) with a null bd
			fx_reg = 0; sc = 2; bdsz = 1; bd = 0; odsz = 2 + rbits(1); iis = {1'b0, odsz[1:0]};
		end
		endcase
		// Word displacements are sign-extended: both signs, and every address
		// still inside the scratch region (A1 and the table's pointers sit
		// well above its bottom).
		r2 = rbits(8); od = {r2[7:1], 1'b0};
		if (rbits(1)) od = -od;
		fx_w[0] = {1'b0, xi[2:0], wl[0], sc[1:0], 1'b1, bs, is, bdsz[1:0], 1'b0, iis};
		fx_n = 1;
		put_disp(bdsz, bd);
		if (iis != 3'b000 && iis != 3'b100) put_disp(odsz, od);
	end
endtask

task put_fx;
	integer k;
	begin for (k = 0; k < fx_n; k = k + 1) ib_put(fx_w[k]); end
endtask

integer kind, dn, an, n, bw;

task gen_program;
	input [31:0] seed;
	begin
		rnd = seed;
		pw  = 0;
		in_ = 0;
		emit(16'h207C); emit(TABLE[31:16]);   emit(TABLE[15:0]);     // MOVEA.L #TABLE,A0
		emit(16'h227C); emit(SCRATCH[31:16]); emit(SCRATCH[15:0]);   // MOVEA.L #SCRATCH,A1
		emit(16'h247C); emit(16'h0000);       emit(16'h6000);        // MOVEA.L #$6000,A2
		for (n = 0; n < 4; n = n + 1) begin
			r1 = rbits(3);
			emit({4'b0111, n[2:0], 1'b0, 5'd0, r1[2:0]});             // MOVEQ #0..7,Dn
		end
		for (n = 4; n < 8; n = n + 1) begin
			r1 = rbits(32);
			emit({4'b0010, n[2:0], 6'b000_111, 3'b100}); emit(r1[31:16]); emit(r1[15:0]);
		end

		for (n = 0; n < NINSN; n = n + 1) begin
			kind = rbits(32) % 27;
			dn = 4 + rbits(2);
			gen_fx;
			case (kind)
			0:  begin ib_put({4'b0010, dn[2:0], 3'b000, 3'b110, fx_reg[2:0]}); put_fx; end        // MOVE.L <fx>,Dn
			1:  begin ib_put({4'b0011, dn[2:0], 3'b000, 3'b110, fx_reg[2:0]}); put_fx; end        // MOVE.W <fx>,Dn
			2:  begin ib_put({4'b0001, dn[2:0], 3'b000, 3'b110, fx_reg[2:0]}); put_fx; end        // MOVE.B <fx>,Dn
			3:  begin ib_put({4'b0010, fx_reg[2:0], 3'b110, 3'b000, dn[2:0]}); put_fx; end        // MOVE.L Dn,<fx>
			4:  begin ib_put({4'b0011, fx_reg[2:0], 3'b110, 3'b000, dn[2:0]}); put_fx; end        // MOVE.W Dn,<fx>
			5:  begin ib_put({4'b1101, dn[2:0], 3'b010, 3'b110, fx_reg[2:0]}); put_fx; end        // ADD.L <fx>,Dn
			6:  begin ib_put({4'b1101, dn[2:0], 3'b110, 3'b110, fx_reg[2:0]}); put_fx; end        // ADD.L Dn,<fx>
			7:  begin ib_put({4'b1011, dn[2:0], 3'b010, 3'b110, fx_reg[2:0]}); put_fx; ib_flush;  // CMP.L <fx>,Dn
			          ib_put({4'b0101, 4'b0010, 2'b11, 3'b000, dn[2:0]}); end                      // SHI Dn
			8:  begin ib_put({10'b0100_0110_10, 3'b110, fx_reg[2:0]}); put_fx; end                // NOT.L <fx>
			9:  begin ib_put({10'b0100_0010_10, 3'b110, fx_reg[2:0]}); put_fx; end                // CLR.L <fx>
			10: begin r3 = rbits(32);                                                             // ADDI.L #imm,<fx>
			          ib_put({10'b0000_0110_10, 3'b110, fx_reg[2:0]}); ib_put(r3[31:16]); ib_put(r3[15:0]); put_fx; end
			11: begin r3 = rbits(16);                                                             // CMPI.W #imm,<fx>
			          ib_put({10'b0000_1100_01, 3'b110, fx_reg[2:0]}); ib_put(r3[15:0]); put_fx; ib_flush;
			          ib_put({4'b0101, 4'b0101, 2'b11, 3'b000, dn[2:0]}); end                      // SCS Dn
			12: begin an = 5 + rbits(1);                                                          // LEA <fx>,A5/A6
			          ib_put({4'b0100, an[2:0], 3'b111, 3'b110, fx_reg[2:0]}); put_fx; ib_flush;
			          ib_put({4'b0010, dn[2:0], 3'b000, 3'b001, an[2:0]}); end                    // MOVE.L An,Dn
			13: begin ib_put({10'b0100_1000_01, 3'b110, fx_reg[2:0]}); put_fx; ib_flush;          // PEA <fx>
			          ib_put({4'b0010, dn[2:0], 3'b000, 3'b011, 3'b111}); end                     // MOVE.L (A7)+,Dn
			14: begin ib_put({10'b0100_1100_11, 3'b110, fx_reg[2:0]}); ib_put(16'h00F0); put_fx; end   // MOVEM.L <fx>,D4-D7
			15: begin ib_put({10'b0100_1000_11, 3'b110, fx_reg[2:0]}); ib_put(16'h00F0); put_fx; end   // MOVEM.L D4-D7,<fx>
			16: begin r3 = rbits(5); r4 = rbits(5);                                               // BFEXTU <fx>{o:w},Dn
			          ib_put({10'b1110_1001_11, 3'b110, fx_reg[2:0]});
			          ib_put({1'b0, dn[2:0], 1'b0, r3[4:0], 1'b0, r4[4:0]}); put_fx; end
			17: begin r3 = rbits(5); r4 = rbits(5);                                               // BFINS Dn,<fx>{o:w}
			          ib_put({10'b1110_1111_11, 3'b110, fx_reg[2:0]});
			          ib_put({1'b0, dn[2:0], 1'b0, r3[4:0], 1'b0, r4[4:0]}); put_fx; end
			18: begin r3 = 4 + rbits(2);                                                          // CAS.L Dc,Du,<fx>
			          ib_put({10'b0000_1110_11, 3'b110, fx_reg[2:0]});
			          ib_put({7'd0, r3[2:0], 3'b000, dn[2:0]}); put_fx; end
			19: begin ib_put({10'b0100_1100_00, 3'b110, fx_reg[2:0]});                            // MULU.L <fx>,Dn
			          ib_put({1'b0, dn[2:0], 12'h000}); put_fx; end
			20: begin ib_put({10'b1111_0010_00, 3'b110, fx_reg[2:0]});                            // FMOVE.L <fx>,FP0
			          ib_put(16'h4000); put_fx; ib_flush;
			          ib_put(16'hF200 | dn[2:0]); ib_put(16'h6000); end                           // FMOVE.L FP0,Dn
			21: begin ib_put({10'b1111_0010_00, 3'b110, fx_reg[2:0]});                            // FMOVE.S FP1,<fx>
			          ib_put(16'h6480); put_fx; end
			22: begin ib_put({4'b0010, 3'b010, 3'b011, 3'b110, fx_reg[2:0]}); put_fx; end         // MOVE.L <fx>,(A2)+
			// CHK.L: traps about half the time, a format $2 frame that stacks the
			// NEXT instruction's address -- where a full-format EA's length shows.
			24: begin ib_put({4'b0100, dn[2:0], 3'b100, 3'b110, fx_reg[2:0]}); put_fx; end        // CHK.L <fx>,Dn
			// A full-format SOURCE with destination words after it: the gather
			// extends in the middle.
			25: begin r3 = rbits(6);                                                             // MOVE.L <fx>,(d16,A2)
			          ib_put({4'b0010, 3'b010, 3'b101, 3'b110, fx_reg[2:0]}); put_fx; ib_put({8'd0, r3[5:0], 2'b00}); end
			26: begin r3 = rbits(8);                                                             // MOVE.L <fx>,($7xxx).W
			          ib_put({4'b0010, 3'b000, 3'b111, 3'b110, fx_reg[2:0]}); put_fx; ib_put({4'h7, 2'b00, r3[7:0], 2'b00}); end
			default: begin                                                                        // MOVE.L (A2)+,<fx>
			          ib_put({4'b0010, fx_reg[2:0], 3'b110, 3'b011, 3'b010}); put_fx; end
			endcase
			ib_flush;
		end

		emit(16'h48F9); emit(16'h7FFF); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0]);   // MOVEM.L D0-A6,DUMP
		emit(16'h7001);
		emit(16'h23C0); emit(DONE_ADDR[31:16]); emit(DONE_ADDR[15:0]);
		emit(16'h60FE);
	end
endtask

reg [15:0] memp [0:MEM_WORDS-1];
reg [15:0] memf [0:MEM_WORDS-1];
integer i, e;

task put;
	input [31:0] addr;
	input [15:0] word;
	begin memp[addr >> 1] = word; memf[addr >> 1] = word; end
endtask

task put_handler;
	begin
		put(HANDLER + 32'h00, 16'h48E7); put(HANDLER + 32'h02, 16'h8080);
		put(HANDLER + 32'h04, 16'h2078); put(HANDLER + 32'h06, LOGPTR[15:0]);
		put(HANDLER + 32'h08, 16'h20EF); put(HANDLER + 32'h0A, 16'h0008);
		put(HANDLER + 32'h0C, 16'h20EF); put(HANDLER + 32'h0E, 16'h000C);
		put(HANDLER + 32'h10, 16'h302F); put(HANDLER + 32'h12, 16'h000E);
		put(HANDLER + 32'h14, 16'h0240); put(HANDLER + 32'h16, 16'hF000);
		put(HANDLER + 32'h18, 16'h6704);
		put(HANDLER + 32'h1A, 16'h20EF); put(HANDLER + 32'h1C, 16'h0010);
		put(HANDLER + 32'h1E, 16'h21C8); put(HANDLER + 32'h20, LOGPTR[15:0]);
		put(HANDLER + 32'h22, 16'h2F78); put(HANDLER + 32'h24, RESUME[15:0]); put(HANDLER + 32'h26, 16'h000A);
		put(HANDLER + 32'h28, 16'h4CDF); put(HANDLER + 32'h2A, 16'h0101);
		put(HANDLER + 32'h2C, 16'h4E73);
	end
endtask

task build_memory;
	input [31:0] seed;
	begin
		gen_program(seed);
		for (i = 0; i < MEM_WORDS; i = i + 1) begin
			memp[i] = `AP040_OP_NOP; memf[i] = `AP040_OP_NOP;
		end
		put(32'h0000, STACK_TOP[31:16]); put(32'h0002, STACK_TOP[15:0]);
		put(32'h0004, PROG_BASE[31:16]); put(32'h0006, PROG_BASE[15:0]);
		for (i = 2; i < 64; i = i + 1) begin
			put(i*4, HANDLER[31:16]); put(i*4 + 2, HANDLER[15:0]);
		end
		put_handler;
		put(RESUME, 16'h0000); put(RESUME + 2, 16'h0000);
		put(LOGPTR, LOG_BASE[31:16]); put(LOGPTR + 2, LOG_BASE[15:0]);
		for (i = LOG_BASE >> 1; i < LOG_END >> 1; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = DUMP_BASE >> 1; i < (DUMP_BASE >> 1) + 32; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = 0; i < pw; i = i + 1) put(PROG_BASE + 2*i, prog[i]);
		// The pointer table: 64 even addresses in the first half of the
		// scratch region, so pointer + od + index stays inside it.
		for (e = 0; e < 64; e = e + 1) begin
			put(TABLE + 4*e, 16'h0000);
			put(TABLE + 4*e + 2, SCRATCH[15:0] + 16'd32 * e[15:0]);
		end
		for (i = SCR_LO >> 1; i <= SCR_HI >> 1; i = i + 1) begin
			rnd = xorshift32(rnd);
			memp[i] = rnd[15:0]; memf[i] = rnd[15:0];
		end
		put(DONE_ADDR, 16'h0000); put(DONE_ADDR + 2, 16'h0000);
	end
endtask

wire [31:0] p_addr;
wire [15:0] p_dwrite;
wire        p_nwr, p_nuds, p_nlds, p_longword;
wire  [1:0] p_busstate;
wire  [2:0] p_fc;
reg         p_ready;
wire        p_clkena = (p_busstate == `AP040_BUS_IDLE) | p_ready;
wire [31:0] p_widx   = (p_addr >> 1) & (MEM_WORDS - 1);
wire [15:0] p_din    = memp[p_widx];

ap040_pipe_bus16 #(
	.PC_RESET  (PROG_BASE),
	.PROG_WORDS(1000000)
) dut_p
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.berr (1'b0),   // no bus errors in this bench
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // no MMU walks here
	.clk (clk), .nreset (nreset), .ce (1'b1), .clkena_in (p_clkena),
	.data_in (p_din), .addr_out(p_addr), .data_write(p_dwrite),
	.nwr (p_nwr), .nuds(p_nuds), .nlds(p_nlds),
	.busstate(p_busstate), .longword(p_longword), .fc(p_fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid(), .dbg_eac_pc(), .dbg_eaf_valid(), .dbg_eaf_pc(),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0(), .dbg_d1(), .dbg_d2(), .dbg_d3(),
	.dbg_d4(), .dbg_d5(), .dbg_d6(), .dbg_d7(),
	.dbg_ccr(), .dbg_sr(), .dbg_commits()
);

always @(posedge clk) begin
	if (!nreset) p_ready <= 1'b0;
	else begin
		p_ready <= 1'b0;
		if (p_busstate != `AP040_BUS_IDLE && !p_ready) begin
			p_ready <= 1'b1;
			if (p_busstate == `AP040_BUS_WRITE) begin
				if (!p_nuds) memp[p_widx][15:8] <= p_dwrite[15:8];
				if (!p_nlds) memp[p_widx][7:0]  <= p_dwrite[7:0];
			end
		end
	end
end

wire [31:0] f_addr;
wire [15:0] f_dwrite;
wire        f_nwr, f_nuds, f_nlds, f_longword, f_nresetout;
wire  [1:0] f_busstate;
wire  [2:0] f_fc;
reg         f_ready;
wire        f_clkena = (f_busstate == `AP040_BUS_IDLE) | f_ready;
wire [31:0] f_widx   = (f_addr >> 1) & (MEM_WORDS - 1);
wire [15:0] f_din    = memf[f_widx];

ap040_tg68k_compat #(
	.AP040_HAS_MMU(0), .AP040_HAS_FPU(1), .AP040_ENABLE_CACHE(0)
) dut_f
(
	.clk(clk), .nreset(nreset),
	.cache_allow_all(1'b1), .cache_snoop_stb(1'b0), .cache_snoop_addr(32'd0),
	.cache_z2_ena(1'b0), .cache_z3_base0(5'd0), .cache_z3_ena0(1'b0),
	.cache_z3_base1(4'd0), .cache_z3_ena1(1'b0),
	.clkena_in(f_clkena), .bus_clkena_in(f_clkena), .tick_in(1'b1),
	.data_in(f_din), .ipl(3'b111), .ipl_autovector(1'b1), .berr(1'b0),
	.addr_out(f_addr), .data_write(f_dwrite),
	.nwr(f_nwr), .nuds(f_nuds), .nlds(f_nlds),
	.busstate(f_busstate), .longword(f_longword), .post_drain(),
	.nresetout(f_nresetout), .fc(f_fc),
	.nmi_ack_toggle(), .cache_maint_req(), .cache_maint_ic(), .cache_maint_dc(),
	.mmu_addr_log(), .mmu_addr_phys(), .mmu_cache_inhibit(),
	.walker_req(), .walker_we(), .walker_addr(), .walker_wdat(),
	.walker_ack(1'b0), .walker_data(32'd0), .walker_berr(1'b0),
	.cache_req(), .cache_addr(), .cache_data(16'd0),
	.cache_ack(1'b0), .cache_burst(), .cache_burst_len(), .cache_ramaddr(),
	.cacr_out(), .vbr_out(),
	.debug_busy(), .debug_fault(), .debug_halted(),
	.debug_status(), .debug_status2(),
	.debug_exception_valid(), .debug_exception()
);

always @(posedge clk) begin
	if (!nreset) f_ready <= 1'b0;
	else begin
		f_ready <= 1'b0;
		if (f_busstate != `AP040_BUS_IDLE && !f_ready) begin
			f_ready <= 1'b1;
			if (f_busstate == `AP040_BUS_WRITE) begin
				if (!f_nuds) memf[f_widx][15:8] <= f_dwrite[15:8];
				if (!f_nlds) memf[f_widx][7:0]  <= f_dwrite[7:0];
			end
		end
	end
end

integer errors = 0;
integer cyc, round, mism, nlog_p, nlog_f, frames, show;
reg [31:0] seed, pdone, fdone;

function [31:0] rd32p; input [31:0] a; rd32p = {memp[a >> 1], memp[(a >> 1) + 1]}; endfunction
function [31:0] rd32f; input [31:0] a; rd32f = {memf[a >> 1], memf[(a >> 1) + 1]}; endfunction

task cmp_words;
	input [8*24-1:0] what;
	input     [31:0] lo;
	input     [31:0] hi;
	integer w, shown;
	begin
		shown = 0;
		for (w = lo >> 1; w < (hi >> 1); w = w + 1)
			if (memp[w] !== memf[w]) begin
				errors = errors + 1;
				mism = mism + 1;
				if (shown < 6)
					$display("FAIL: round %0d (seed %h): %0s word at %h = %h on the pipelined core, %h on the FSM core",
					         round, seed, what, 2*w, memp[w], memf[w]);
				shown = shown + 1;
			end
		if (shown > 6) $display("      ... %0d more in %0s", shown - 6, what);
	end
endtask

initial begin
	frames = 0;
	for (round = 0; round < NROUND; round = round + 1) begin
		seed = 32'h0BAD_F00D + round * 32'h9E37_79B9;
		nreset = 0;
		repeat (8) @(posedge clk);
		build_memory(seed);
		nreset = 1;
		@(posedge clk);
		@(posedge clk);
		dut_p.u_cpu.u_regfile.isp = STACK_TOP;

		cyc = 0; pdone = 0; fdone = 0;
		while (cyc < TIMEOUT && !(pdone != 0 && fdone != 0)) begin
			@(posedge clk);
			cyc = cyc + 1;
			pdone = rd32p(DONE_ADDR);
			fdone = rd32f(DONE_ADDR);
		end
		mism = 0;
		if (pdone != 32'd1) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the pipelined core never finished (%0d cycles)", round, seed, cyc);
		end
		if (fdone != 32'd1) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the FSM core never finished (%0d cycles)", round, seed, cyc);
		end
		if (pdone == 32'd1 && fdone == 32'd1) begin
			cmp_words("register dump", DUMP_BASE, DUMP_BASE + 60);
			cmp_words("scratch", SCR_LO, SCR_HI + 2);
			nlog_p = rd32p(LOGPTR) - LOG_BASE;
			nlog_f = rd32f(LOGPTR) - LOG_BASE;
			if (nlog_p != nlog_f) begin
				errors = errors + 1;
				$display("FAIL: round %0d (seed %h): %0d bytes of exception frames logged on the pipelined core, %0d on the FSM core",
				         round, seed, nlog_p, nlog_f);
			end
			cmp_words("exception log", LOG_BASE, LOG_END);
			frames = frames + nlog_f;
		end
		if ($value$plusargs("showround=%d", show) && show == round) begin
			for (i = 0; i < pw; i = i + 1)
				$display("  prog %h: %h", PROG_BASE + 2*i, prog[i]);
			for (i = LOG_BASE; i < LOG_BASE + nlog_f + 4; i = i + 4)
				$display("  log %h: pipe %h  fsm %h", i, rd32p(i), rd32f(i));
		end
		$display("round %0d: seed %h, %0d program words, %0d cycles, %0d log bytes, %0d mismatches",
		         round, seed, pw, cyc, rd32f(LOGPTR) - LOG_BASE, mism);
	end
	if (frames == 0) begin
		errors = errors + 1;
		$display("FAIL: no round took an exception: the reserved encodings went unexercised");
	end
	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);
	$finish;
end

endmodule
