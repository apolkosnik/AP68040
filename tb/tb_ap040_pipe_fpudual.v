//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: the FPU)          //
//                                                                          //
// tb_ap040_pipe_fpudual.v - FPU programs on both cores, compared           //
//                                                                          //
// tb_ap040_pipe_dual.v's method for the FPU: the same generated program    //
// runs on the pipelined core and on rtl/ap040/ap040_core.v, which carries  //
// the same FPU engine, and the architectural state each leaves in memory   //
// is compared. The engine being shared, what this measures is the part     //
// that is not: ap040_pipe_fpu.v's transliteration of the sequential core's //
// FPU states, decode's F-line gather, and their seams with the pipeline.   //
// The cputest corpus covers the general operations in every format and EA //
// (BasicFPU, PackedFPU); this archive's FINT group has no slices, so       //
// FMOVEM, the control registers, FBcc/FScc/FDBcc/FTRAPcc, FMOVECR and      //
// FSAVE/FRESTORE are measured here, and so is everything a single-         //
// instruction corpus round cannot hold: FPU instructions behind integer    //
// ones that produce their operands, released (background) operations      //
// followed by integer code and by the next F-line instruction that must   //
// wait for them or deliver their exception, branches on the FPU's         //
// condition codes, and FPCR rounding modes, precisions and exception       //
// enables set in the middle of a program.                                  //
//                                                                          //
// Programs are generated at time 0 from a fixed-seed xorshift, one per     //
// round, variable length. Every F-line instruction is preceded by          //
//   MOVE.L #<the address after it>,RESUME.W                                //
// and every exception goes to one handler that logs the frame -- SR, PC,   //
// format/vector word, and the address field of a format $2 or $3 frame --  //
// rewrites the stacked PC to RESUME and returns. So an exception, whether  //
// raised by its own instruction, pre-instruction for a released           //
// operation, or by FTRAPcc, is compared like any other result: both cores //
// must log the same frames in the same order.                              //
//                                                                          //
// The epilogue dumps D0-D7/A0-A6, FP0-FP7 (FMOVEM.X) and FPCR/FPSR/FPIAR,  //
// then flags done. Compared: the dump, the store area, the exception log.  //
// Memory operands come from typed tables of normalised values (so most     //
// operations are ordinary arithmetic) with some raw words among them.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_fpudual;

localparam integer NINSN     = 72;             // generated instructions per program
localparam integer NROUND    = 24;             // programs, one per seed
localparam [31:0] PROG_BASE  = 32'h0000_0400;
localparam [31:0] HANDLER    = 32'h0000_0300;
localparam [31:0] RESUME     = 32'h0000_1200;  // the handler's return address
localparam [31:0] LOGPTR     = 32'h0000_1204;  // where the next frame is logged
localparam [31:0] LOG_BASE   = 32'h0000_1300;  // frames: 8 or 12 bytes each
localparam [31:0] LOG_END    = 32'h0000_1700;
localparam [31:0] DUMP_BASE  = 32'h0000_1000;  // D0-A6 60, FP0-7 96, FPCR/FPSR/FPIAR 12
localparam [31:0] DONE_ADDR  = 32'h0000_1100;
localparam [31:0] STACK_TOP  = 32'h0000_3000;
localparam [31:0] TAB_X      = 32'h0000_4000;  // A0: 64 extended values, 12 bytes each
localparam [31:0] TAB_D      = 32'h0000_4400;  // A1: 64 doubles
localparam [31:0] TAB_S      = 32'h0000_4600;  // A2: 128 singles / longs / words / bytes
localparam [31:0] STORE_LO   = 32'h0000_5000;  // A3: stores, FMOVEM, FSAVE frames
localparam [31:0] STORE_HI   = 32'h0000_5FFE;
localparam [31:0] A3_BASE    = 32'h0000_5800;
localparam integer MEM_WORDS = 32768;
localparam integer TIMEOUT   = 600000;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

//--------------------------------------------------------------------------//
// The generator.                                                           //
//--------------------------------------------------------------------------//

reg [15:0] prog [0:4095];
integer    pw;           // words emitted

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

// The address the next word goes to.
function [31:0] here;
	input integer dummy;
	begin here = PROG_BASE + 2 * pw; end
endfunction

// MOVE.L #a,RESUME.W -- four words.
task emit_resume;
	input [31:0] a;
	begin
		emit(16'h21FC); emit(a[31:16]); emit(a[15:0]); emit(RESUME[15:0]);
	end
endtask

// The F-line instructions are built into a small buffer first, so their
// length -- and so the resume address ahead of them -- is known.
reg [15:0] ib [0:15];
integer    in_;
task ib_put;
	input [15:0] w;
	begin ib[in_] = w; in_ = in_ + 1; end
endtask
task ib_flush;
	integer k;
	begin
		emit_resume(here(0) + 8 + 2 * in_);
		for (k = 0; k < in_; k = k + 1) emit(ib[k]);
	end
endtask

// Operation modes the 68040 executes in hardware, and some it does not
// (FINT/FINTRZ/FSIN/FETOX: the unimplemented-instruction route, vector 11
// format $2 -- the handler logs it and moves on).
function [6:0] pick_op;
	input integer r;
	begin
		case (r % 30)
			0:  pick_op = 7'h00;  1:  pick_op = 7'h40;  2:  pick_op = 7'h44;  // FMOVE FSMOVE FDMOVE
			3:  pick_op = 7'h18;  4:  pick_op = 7'h58;  5:  pick_op = 7'h1A;  // FABS FSABS FNEG
			6:  pick_op = 7'h5E;  7:  pick_op = 7'h38;  8:  pick_op = 7'h3A;  // FDNEG FCMP FTST
			9:  pick_op = 7'h22;  10: pick_op = 7'h62;  11: pick_op = 7'h66;  // FADD FSADD FDADD
			12: pick_op = 7'h28;  13: pick_op = 7'h68;  14: pick_op = 7'h6C;  // FSUB FSSUB FDSUB
			15: pick_op = 7'h23;  16: pick_op = 7'h63;  17: pick_op = 7'h27;  // FMUL FSMUL FSGLMUL
			18: pick_op = 7'h20;  19: pick_op = 7'h64;  20: pick_op = 7'h24;  // FDIV FDDIV FSGLDIV
			21: pick_op = 7'h04;  22: pick_op = 7'h41;  23: pick_op = 7'h45;  // FSQRT FSSQRT FDSQRT
			24: pick_op = 7'h22;  25: pick_op = 7'h23;  26: pick_op = 7'h20;  // weight the common ones
			27: pick_op = 7'h01;  28: pick_op = 7'h03;                          // FINT FINTRZ: FPSP
			default: pick_op = 7'h0E;                                           // FSIN: FPSP
		endcase
	end
endfunction

// A source format and the table pointer that holds it.
integer fmt, an_t, entry, disp, fpm, fpn, dn, dm, q, cc, k, sel, list, opm, kind, skipw, imm;
// rbits() is 32 bits wide, so its results go through these before any
// concatenation: dropped straight in, one pushes the other fields out
// (tb_ap040_pipe_dual.v's header has the case that cost a false finding).
integer r1, r2, r3;
reg [15:0] eaw;

task gen_program;
	input [31:0] seed;
	integer n, save_pw, br_at;
	begin
		rnd = seed;
		pw  = 0;
		// Pointers: A0-A2 the typed tables, A3 the store area, A4 a second
		// store pointer for (An)+/-(An) pairs, A5/A6 scratch data.
		emit(16'h207C); emit(TAB_X[31:16]);   emit(TAB_X[15:0]);    // MOVEA.L #TAB_X,A0
		emit(16'h227C); emit(TAB_D[31:16]);   emit(TAB_D[15:0]);    // MOVEA.L #TAB_D,A1
		emit(16'h247C); emit(TAB_S[31:16]);   emit(TAB_S[15:0]);    // MOVEA.L #TAB_S,A2
		emit(16'h267C); emit(A3_BASE[31:16]); emit(A3_BASE[15:0]);  // MOVEA.L #A3_BASE,A3
		emit(16'h287C); emit(16'h0000);       emit(16'h5400);       // MOVEA.L #$5400,A4
		// D0-D7: a mix of small integers, words and raw bit patterns.
		for (n = 0; n < 8; n = n + 1) begin
			imm = rbits(32);
			emit({4'b0010, n[2:0], 6'b000_111, 3'b100});   // MOVE.L #imm,Dn
			emit(imm[31:16]); emit(imm[15:0]);
		end
		// FP0-FP7 from the tables, so they start as numbers.
		for (n = 0; n < 8; n = n + 1) begin
			emit_resume(here(0) + 8 + 6);
			emit(16'hF228);                                  // FMOVE.X (d16,A0),FPn
			emit({3'b010, 3'd2, n[2:0], 7'h00});
			emit(12 * n);
		end

		for (n = 0; n < NINSN; n = n + 1) begin
			kind = rbits(32) % 40;
			fpn  = rbits(3); fpm = rbits(3); dn = rbits(3); dm = rbits(3);
			in_  = 0;
			case (kind)
			// ---- integer glue: producers of the Dn the FPU reads next
			0: begin imm = rbits(8);
				emit({4'b0111, dn[2:0], 1'b0, imm[7:0]}); end                  // MOVEQ
			1: begin q = rbits(3);
				emit({4'b0101, q[2:0], 6'b010_000, dn[2:0]}); end              // ADDQ.L
			2: emit({4'b1101, dn[2:0], 6'b010_000, dm[2:0]});                  // ADD.L Dm,Dn
			3: begin imm = rbits(32);
				emit({4'b0010, dn[2:0], 6'b000_111, 3'b100});
				emit(imm[31:16]); emit(imm[15:0]); end                          // MOVE.L #imm,Dn
			// ---- register to register
			4, 5, 6: begin opm = pick_op(rbits(32));
				ib_put(16'hF200); ib_put({3'b000, fpm[2:0], fpn[2:0], opm[6:0]}); ib_flush; end
			// ---- from a data register: L, S, W, B
			7, 8: begin
				case (rbits(2)) 0: fmt = 0; 1: fmt = 1; 2: fmt = 4; default: fmt = 6; endcase
				opm = pick_op(rbits(32));
				ib_put({10'b1111_0010_00, 3'b000, dn[2:0]});
				ib_put({3'b010, fmt[2:0], fpn[2:0], opm[6:0]}); ib_flush; end
			// ---- from memory through a typed table: (d16,An) and (An)
			9, 10, 11: begin
				case (rbits(3))
					0, 1: begin fmt = 2; an_t = 0; entry = rbits(6); disp = 12 * entry; end   // X
					2, 3: begin fmt = 5; an_t = 1; entry = rbits(6); disp = 8 * entry; end    // D
					4:    begin fmt = 1; an_t = 2; entry = rbits(7); disp = 4 * entry; end    // S
					5:    begin fmt = 0; an_t = 2; entry = rbits(7); disp = 4 * entry; end    // L
					6:    begin fmt = 4; an_t = 2; entry = rbits(8); disp = 2 * entry; end    // W
					default: begin fmt = 6; an_t = 2; entry = rbits(9); disp = entry; end     // B
				endcase
				opm = pick_op(rbits(32));
				if (rbits(2) == 0) begin
					ib_put({10'b1111_0010_00, 3'b010, an_t[2:0]});                // (An)
					ib_put({3'b010, fmt[2:0], fpn[2:0], opm[6:0]});
				end else begin
					ib_put({10'b1111_0010_00, 3'b101, an_t[2:0]});                // (d16,An)
					ib_put({3'b010, fmt[2:0], fpn[2:0], opm[6:0]});
					ib_put(disp[15:0]);
				end
				ib_flush; end
			// ---- an immediate: L, W, B, S, D, X
			12: begin
				case (rbits(3))
					0: fmt = 0; 1: fmt = 4; 2: fmt = 6; 3: fmt = 1; 4, 5: fmt = 5; default: fmt = 2;
				endcase
				opm = pick_op(rbits(32));
				ib_put(16'hF23C);
				ib_put({3'b010, fmt[2:0], fpn[2:0], opm[6:0]});
				case (fmt)
					0, 1: begin imm = rbits(32); ib_put(imm[31:16]); ib_put(imm[15:0]); end
					4:    begin imm = rbits(16); ib_put(imm[15:0]); end
					6:    begin imm = rbits(8);  ib_put({8'h00, imm[7:0]}); end
					5:    begin r1 = rbits(10); ib_put({1'b0, 1'b1, 4'h0, r1[9:0]}); imm = rbits(32);   // a double
					            ib_put(imm[31:16]); ib_put(imm[15:0]); imm = rbits(16); ib_put(imm[15:0]); end
					default: begin r1 = rbits(4); ib_put({1'b0, 15'h3FFF + {11'd0, r1[3:0]}}); ib_put(16'h0000);  // an extended
					            imm = rbits(32); ib_put({1'b1, imm[30:16]}); ib_put(imm[15:0]);
					            imm = rbits(32); ib_put(imm[31:16]); ib_put(imm[15:0]); end
				endcase
				ib_flush; end
			// ---- stores: to Dn (L, S, W, B), to (d16,A3) in every format
			13, 14: begin
				case (rbits(2)) 0: fmt = 0; 1: fmt = 1; 2: fmt = 4; default: fmt = 6; endcase
				ib_put({10'b1111_0010_00, 3'b000, dn[2:0]});
				ib_put({3'b011, fmt[2:0], fpm[2:0], 7'd0}); ib_flush; end
			15, 16: begin
				case (rbits(3))
					0: fmt = 0; 1: fmt = 1; 2: fmt = 4; 3: fmt = 6; 4, 5: fmt = 5; 6: fmt = 2; default: fmt = 3;
				endcase
				disp = 16 * rbits(6);
				ib_put({10'b1111_0010_00, 3'b101, 3'd3});
				ib_put({3'b011, fmt[2:0], fpm[2:0], 7'd0});
				ib_put(disp[15:0]); ib_flush; end
			// ---- a store through -(A4) and a load back up through (A4)+, or a
			// store through (A4)+ and a load back down through -(A4): balanced
			// pairs, both steps both ways
			17: begin
				case (rbits(2)) 0: fmt = 0; 1: fmt = 5; default: fmt = 2; endcase
				r1 = rbits(1);
				ib_put({10'b1111_0010_00, r1[0] ? 3'b011 : 3'b100, 3'd4});          // FMOVE FPm,(A4)+ / -(A4)
				ib_put({3'b011, fmt[2:0], fpm[2:0], 7'd0}); ib_flush;
				in_ = 0;
				ib_put({10'b1111_0010_00, r1[0] ? 3'b100 : 3'b011, 3'd4});          // FMOVE -(A4) / (A4)+,FPn
				ib_put({3'b010, fmt[2:0], fpn[2:0], 7'h00}); ib_flush; end
			// ---- FMOVEM.X: a static list out through -(A4) and back through (A4)+
			// The store's list convention is either one: the predecrement
			// convention matches -(An), the other makes the 68040 write each
			// register's longwords in reverse (ap040_core.v's fp_rev).
			18, 19: begin list = rbits(8); r1 = rbits(1);
				ib_put(16'hF224);                                                   // FMOVEM.X list,-(A4)
				ib_put({3'b111, r1[0], 1'b0, 3'b000, list[7:0]}); ib_flush;
				in_ = 0;
				ib_put(16'hF21C);                                                   // FMOVEM.X (A4)+,list
				ib_put({3'b110, 2'b10, 3'b000, list[7:0]}); ib_flush; end
			// ---- FMOVEM.X to (d16,A3) and back, static or a dynamic list in Dn
			20: begin list = rbits(8); disp = 128 * rbits(3); r1 = rbits(1);
				if (rbits(1)) begin
					ib_put(16'hF22B); ib_put({3'b111, r1[0], 1'b0, 3'b000, list[7:0]}); ib_put(disp[15:0]); ib_flush;
					in_ = 0;
					ib_put(16'hF22B); ib_put({3'b110, 2'b10, 3'b000, list[7:0]}); ib_put(disp[15:0]); ib_flush;
				end else begin
					emit({4'b0111, dn[2:0], 1'b0, list[7:0]});                       // MOVEQ #list,Dn
					ib_put(16'hF22B); ib_put({3'b111, 2'b11, 3'b000, 1'b0, dn[2:0], 4'b0000}); ib_put(disp[15:0]); ib_flush;
					in_ = 0;
					ib_put(16'hF22B); ib_put({3'b110, 2'b11, 3'b000, 1'b0, dn[2:0], 4'b0000}); ib_put(disp[15:0]); ib_flush;
				end end
			// ---- control registers: to and from Dn, one at a time
			21: begin sel = 1 << rbits(2); if (sel == 8) sel = 1;
				ib_put({10'b1111_0010_00, 3'b000, dn[2:0]});                       // FMOVE.L FPcr,Dn
				ib_put({3'b101, sel[2:0], 10'd0}); ib_flush; end
			22: begin
				// FMOVE.L #fpcr,FPCR: a rounding mode and precision, and one
				// time in four some exception enables.
				r1 = rbits(2); r2 = rbits(2); r3 = rbits(8);
				imm = {24'd0, r1[1:0], r2[1:0], 4'b0000};
				if (rbits(2) == 0) imm = imm | {16'd0, r3[7:0], 8'd0};
				ib_put(16'hF23C); ib_put({3'b100, 3'b100, 10'd0});
				ib_put(16'h0000); ib_put(imm[15:0]); ib_flush; end
			23: begin
				// FMOVEM.L FPCR/FPSR/FPIAR to (d16,A3) and FPSR back from Dn.
				sel = rbits(3); if (sel == 0) sel = 7;
				disp = 4 * rbits(5);
				ib_put(16'hF22B); ib_put({3'b101, sel[2:0], 10'd0}); ib_put(disp[15:0]); ib_flush;
				in_ = 0;
				ib_put({10'b1111_0010_00, 3'b000, dn[2:0]});                       // FMOVE.L Dn,FPSR
				ib_put({3'b100, 3'b010, 10'd0}); ib_flush; end
			24: begin
				// FMOVEM.L #imm,FPCR/FPSR: two longwords; a clean FPCR first.
				r1 = rbits(2); r2 = rbits(2);
				imm = {24'd0, r1[1:0], r2[1:0], 4'b0000};
				ib_put(16'hF23C); ib_put({3'b100, 3'b110, 10'd0});
				ib_put(16'h0000); ib_put(imm[15:0]);
				imm = rbits(32) & 32'h0F00_FFF8;
				ib_put(imm[31:16]); ib_put(imm[15:0]); ib_flush; end
			// ---- branches on the FPU's condition codes, forward over the
			// next instruction; FNOP is F280 0000.
			25, 26: begin
				cc = rbits(5);                     // 0..31: the non-signalling half and some signalling
				save_pw = pw;
				emit_resume(here(0) + 8 + 4);
				br_at = pw;
				emit({10'b1111_0010_10, cc[5:0]}); emit(16'h0000);                 // FBcc.W, patched
				// the instruction it may skip: a register op
				opm = pick_op(rbits(32)); in_ = 0;
				ib_put(16'hF200); ib_put({3'b000, fpm[2:0], fpn[2:0], opm[6:0]}); ib_flush;
				prog[br_at + 1] = 2 * (pw - (br_at + 1));
				end
			27: begin
				emit_resume(here(0) + 8 + 4);
				emit(16'hF280); emit(16'h0000); end                               // FNOP
			// ---- FScc to Dn and to (d16,A3)
			28: begin cc = rbits(5);
				ib_put({10'b1111_0010_01, 3'b000, dn[2:0]}); ib_put({10'd0, cc[5:0]}); ib_flush; end
			29: begin cc = rbits(5); disp = rbits(8);
				ib_put({10'b1111_0010_01, 3'b101, 3'd3}); ib_put({10'd0, cc[5:0]}); ib_put(disp[15:0]); ib_flush; end
			// ---- FDBcc Dn,<over the next instruction>: counts down, falls
			// through at -1 or when the condition holds.
			30: begin cc = rbits(5);
				emit({4'b0111, dn[2:0], 1'b0, 8'd2});                               // MOVEQ #2,Dn
				emit_resume(here(0) + 8 + 6);
				br_at = pw;
				emit({10'b1111_0010_01, 3'b001, dn[2:0]}); emit({10'd0, cc[5:0]}); emit(16'h0000);
				opm = pick_op(rbits(32)); in_ = 0;
				ib_put(16'hF200); ib_put({3'b000, fpm[2:0], fpn[2:0], opm[6:0]}); ib_flush;
				prog[br_at + 2] = 2 * (pw - (br_at + 2));
				end
			// ---- FTRAPcc, no operand and with a word
			31: begin cc = rbits(5);
				if (rbits(1)) begin ib_put(16'hF27C); ib_put({10'd0, cc[5:0]}); end
				else begin imm = rbits(16); ib_put(16'hF27A); ib_put({10'd0, cc[5:0]}); ib_put(imm[15:0]); end
				ib_flush; end
			// ---- FMOVECR: not in the 68040's hardware
			32: begin imm = rbits(7);
				ib_put(16'hF200); ib_put({3'b010, 3'b111, fpn[2:0], imm[6:0]}); ib_flush; end
			// ---- FSAVE -(A4), FRESTORE (A4)+: the state goes out and back
			33: begin
				ib_put(16'hF324); ib_flush;                                          // FSAVE -(A4)
				in_ = 0;
				ib_put(16'hF35C); ib_flush; end                                      // FRESTORE (A4)+
			// ---- FSAVE to (d16,A3)
			34: begin disp = 128 * rbits(3);
				ib_put(16'hF32B); ib_put(disp[15:0]); ib_flush; end
			// ---- an integer use of an FPU result straight behind it
			35: begin
				ib_put({10'b1111_0010_00, 3'b000, dn[2:0]});
				ib_put({3'b011, 3'd0, fpm[2:0], 7'd0}); ib_flush;                    // FMOVE.L FPm,Dn
				emit({4'b1101, dm[2:0], 6'b010_000, dn[2:0]}); end                   // ADD.L Dn,Dm
			// ---- a malformed command: opclass 001, F-line
			36: begin
				ib_put(16'hF200); ib_put({3'b001, 13'd0}); ib_flush; end
			default: begin opm = pick_op(rbits(32));
				ib_put(16'hF200); ib_put({3'b000, fpm[2:0], fpn[2:0], opm[6:0]}); ib_flush; end
			endcase
		end

		// Epilogue: FPCR clean, then dump everything and flag done.
		emit_resume(here(0) + 8 + 8);
		emit(16'hF23C); emit({3'b100, 3'b100, 10'd0}); emit(16'h0000); emit(16'h0000);  // FMOVE.L #0,FPCR
		emit(16'h48F9); emit(16'h7FFF); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0]);  // MOVEM.L D0-A6,DUMP
		emit_resume(here(0) + 8 + 8);
		emit(16'hF239); emit({3'b111, 2'b10, 3'b000, 8'hFF});                            // FMOVEM.X FP0-FP7,DUMP+60
		emit(16'h0000); emit(DUMP_BASE[15:0] + 16'd60);
		emit_resume(here(0) + 8 + 8);
		emit(16'hF239); emit({3'b101, 3'b111, 10'd0});                                  // FMOVEM.L FPCR/FPSR/FPIAR,DUMP+156
		emit(16'h0000); emit(DUMP_BASE[15:0] + 16'd156);
		emit(16'h7001);                                                                  // MOVEQ #1,D0
		emit(16'h23C0); emit(DONE_ADDR[31:16]); emit(DONE_ADDR[15:0]);                   // MOVE.L D0,DONE
		emit(16'h60FE);                                                                  // BRA.B *
	end
endtask

//--------------------------------------------------------------------------//
// Two memories, identical at time 0.                                       //
//--------------------------------------------------------------------------//

reg [15:0] memp [0:MEM_WORDS-1];
reg [15:0] memf [0:MEM_WORDS-1];
integer i, e;

task put;
	input [31:0] addr;
	input [15:0] word;
	begin memp[addr >> 1] = word; memf[addr >> 1] = word; end
endtask

// The handler: log the frame, resume at RESUME.
//   MOVEM.L D0/A0,-(A7)       the frame is then at 8(A7)
//   MOVEA.L LOGPTR.W,A0
//   MOVE.L  8(A7),(A0)+       SR, PC high
//   MOVE.L  12(A7),(A0)+      PC low, format/vector
//   MOVE.W  14(A7),D0
//   ANDI.W  #$F000,D0
//   BEQ.S   +4                format $0: no address field
//   MOVE.L  16(A7),(A0)+      format $2/$3: the address
//   MOVE.L  A0,LOGPTR.W
//   MOVE.L  RESUME.W,10(A7)   the stacked PC
//   MOVEM.L (A7)+,D0/A0
//   RTE
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
		for (i = DUMP_BASE >> 1; i < (DUMP_BASE >> 1) + 96; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = 0; i < pw; i = i + 1) put(PROG_BASE + 2*i, prog[i]);

		// The tables. X: sign, a biased exponent near 1 (and one in eight
		// anywhere), the explicit integer bit set, random fraction. D and S
		// likewise. Some raw entries among them.
		for (e = 0; e < 64; e = e + 1) begin
			rnd = xorshift32(rnd);
			put(TAB_X + 12*e,     {rnd[31], (rnd[2:0] == 0) ? rnd[30:16] : (15'h3FF0 + rnd[20:16])});
			put(TAB_X + 12*e + 2, 16'h0000);
			rnd = xorshift32(rnd);
			put(TAB_X + 12*e + 4, {(rnd[3:0] != 0), rnd[30:16]});
			put(TAB_X + 12*e + 6, rnd[15:0]);
			rnd = xorshift32(rnd);
			put(TAB_X + 12*e + 8, rnd[31:16]); put(TAB_X + 12*e + 10, rnd[15:0]);
			rnd = xorshift32(rnd);
			put(TAB_D + 8*e,     {rnd[31], (rnd[2:0] == 0) ? rnd[30:20] : (11'h3F8 + rnd[23:20]), rnd[19:16]});
			put(TAB_D + 8*e + 2, rnd[15:0]);
			rnd = xorshift32(rnd);
			put(TAB_D + 8*e + 4, rnd[31:16]); put(TAB_D + 8*e + 6, rnd[15:0]);
		end
		for (e = 0; e < 128; e = e + 1) begin
			rnd = xorshift32(rnd);
			put(TAB_S + 4*e,     {rnd[31], (rnd[2:0] == 0) ? rnd[30:23] : (8'h78 + rnd[26:23]), rnd[22:16]});
			put(TAB_S + 4*e + 2, rnd[15:0]);
		end
		for (i = STORE_LO >> 1; i <= STORE_HI >> 1; i = i + 1) begin
			rnd = xorshift32(rnd);
			memp[i] = rnd[15:0]; memf[i] = rnd[15:0];
		end
		put(DONE_ADDR, 16'h0000); put(DONE_ADDR + 2, 16'h0000);
	end
endtask

//--------------------------------------------------------------------------//
// The pipelined core.                                                      //
//--------------------------------------------------------------------------//

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

//--------------------------------------------------------------------------//
// The FSM core, with its FPU.                                              //
//--------------------------------------------------------------------------//

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

//--------------------------------------------------------------------------//

integer errors = 0;
integer cyc, round, mism, nlog_p, nlog_f, frames, show;
reg [31:0] seed, pdone, fdone, pv, fv;

function [31:0] rd32p; input [31:0] a; rd32p = {memp[a >> 1], memp[(a >> 1) + 1]}; endfunction
function [31:0] rd32f; input [31:0] a; rd32f = {memf[a >> 1], memf[(a >> 1) + 1]}; endfunction

task cmp_words;
	input [8*24-1:0] what;
	input     [31:0] lo;
	input     [31:0] hi;    // exclusive
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
		seed = 32'h2468_ACE1 + round * 32'h9E37_79B9;

		nreset = 0;
		repeat (8) @(posedge clk);
		build_memory(seed);
		nreset = 1;
		@(posedge clk);
		@(posedge clk);
		dut_p.u_cpu.u_regfile.isp = STACK_TOP;

		cyc = 0;
		pdone = 0; fdone = 0;
		while (cyc < TIMEOUT && !(pdone != 0 && fdone != 0)) begin
			@(posedge clk);
			cyc = cyc + 1;
			pdone = rd32p(DONE_ADDR);
			fdone = rd32f(DONE_ADDR);
		end
		mism = 0;
		if (pdone != 32'd1) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the pipelined core never finished (%0d cycles, log at %h)",
			         round, seed, cyc, rd32p(LOGPTR));
		end
		if (fdone != 32'd1) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the FSM core never finished (%0d cycles, log at %h)",
			         round, seed, cyc, rd32f(LOGPTR));
		end
		if (pdone == 32'd1 && fdone == 32'd1) begin
			cmp_words("register/FPU dump", DUMP_BASE, DUMP_BASE + 168);
			cmp_words("store area", STORE_LO, STORE_HI + 2);
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
		// +showround=N prints round N's program and both cores' frame logs.
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
		$display("FAIL: no round took a single exception: the exception paths went unexercised");
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);
	$finish;
end

endmodule
