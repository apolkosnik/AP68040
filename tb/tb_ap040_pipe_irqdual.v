//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: interrupts)       //
//                                                                          //
// tb_ap040_pipe_irqdual.v - interrupts on both cores                       //
//                                                                          //
// The cputest IRQ groups raise a level against one instruction at a time. //
// This bench takes interrupts inside running programs, on the pipelined   //
// core and on rtl/ap040/ap040_core.v side by side, and compares what each //
// leaves in memory (tb_ap040_pipe_dual.v's method): the frames on all      //
// three stacks, a log the handler writes, the registers, and a store      //
// trail.                                                                   //
//                                                                          //
// Each core has its own level register, the word at IRQ_SET, which the     //
// bench drives onto that core's interrupt input; a write to IRQ_DLY sets   //
// it DELAY cycles later. The program makes every interrupt land on the    //
// same instruction boundary on both cores: it raises the level under mask //
// 7, polls the register until its own write has landed, and only then     //
// lowers the mask, so the interrupt belongs to the boundary after the     //
// lowering instruction:                                                   //
//   MOVE #sr,SR and ANDI #sr,SR,                                           //
//   RTE from a frame the program built (format $0 or $2),                  //
//   STOP, woken by a delayed level -- level 7, the edge-triggered one,     //
//   only here, since under mask 7 it would be taken at once,               //
//   RTE through a format-$1 THROWAWAY frame the program built: the real   //
//   frame behind it on the master stack (what a 68040 leaves), on the     //
//   same stack, or on the user stack.                                      //
// The new SR is drawn at random -- supervisor or user, M set or clear, any //
// mask below the level -- so interrupts are taken from both modes and on  //
// both supervisor stacks. One taken with M set pushes its frame on the    //
// master stack and a format-$1 throwaway on the interrupt stack, clearing //
// M; the handler's RTE returns through it.                                //
//                                                                          //
// The interrupt handler logs its own SR, the frame it finds on its stack,  //
// A7, MSP and USP, withdraws the level (and polls until the write has     //
// landed, so the RTE's lowered mask cannot re-take it), and returns. TRAP  //
// #1 returns a user-mode program to supervisor mode with its M bit.       //
//                                                                          //
// Traced events run T1 through the instruction that lowers the mask, so   //
// a trace and an interrupt fall due at the same boundary; a trace handler //
// logs every format-$2 frame and returns. A level raised as a core reads   //
// the first interrupt's autovector nests a second interrupt in its        //
// handler, behind a carried trace where there is one.                     //
//                                                                          //
// Not generated, because ap040_ea_fetch.v deliberately differs there (see  //
// ret_f1): a second throwaway behind the first, or a bad frame behind a    //
// throwaway -- both a format error raised from the RTE's own starting      //
// state, where ap040_core.v commits the throwaway's pop first. No 68040    //
// builds either.                                                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_irqdual;

localparam integer NEV       = 24;
localparam integer NROUND    = 24;
localparam integer DELAY     = 300;
localparam [31:0] PROG_BASE  = 32'h0000_0400;
localparam [31:0] HANDLER    = 32'h0000_0300;   // everything but the below: log, resume at RESUME
localparam [31:0] SUPER      = 32'h0000_0340;   // TRAP #1: back to supervisor mode
localparam [31:0] IRQ_HANDLER = 32'h0000_0360;  // vectors 25-31
localparam [31:0] TRC_HANDLER = 32'h0000_03A0;  // vector 9: log the frame, return
localparam [31:0] RESUME     = 32'h0000_1200;
localparam [31:0] LOGPTR     = 32'h0000_1204;
localparam [31:0] IRQ_SET    = 32'h0000_1210;
localparam [31:0] IRQ_DLY    = 32'h0000_1212;
localparam [31:0] IRQ_CHAIN  = 32'h0000_1214;   // raised when an autovector is read
localparam [31:0] LOG_BASE   = 32'h0000_1300;
localparam [31:0] LOG_END    = 32'h0000_1B00;
localparam [31:0] DUMP_BASE  = 32'h0000_1000;
localparam [31:0] DONE_ADDR  = 32'h0000_1100;
localparam [31:0] STK_LO     = 32'h0000_2000;
localparam [31:0] USP_TOP    = 32'h0000_2800;
localparam [31:0] STACK_TOP  = 32'h0000_3000;   // ISP
localparam [31:0] MSP_TOP    = 32'h0000_3800;
localparam [31:0] SCR_LO     = 32'h0000_4000;   // A4's store trail
localparam [31:0] SCR_HI     = 32'h0000_4800;
localparam integer MEM_WORDS = 32768;
localparam integer TIMEOUT   = 800000;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

reg [15:0] prog [0:8191];
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

// Counted as generated, so a run that never reached a case says so.
integer n_irq, n_irq_m1, n_irq_user, n_stop, n_nmi, n_rte, n_f1, n_f1_msp, n_f1_isp, n_f1_usp;
integer n_trace, n_trace_ev, fill_n, c1, t_new, n_chain, hl, n_trapfill, n_trap8, tf;

integer k, r1, r2, r3, ev, lvl, msk, s, m, m0, fmt, v, irq_on, tgt_at;
reg [15:0] sr;

// Arithmetic on D4-D7, none of it idempotent, and stores down A4's trail:
// an instruction run twice, or skipped, around an interrupt shows.
task filler;
	input integer nmin;
	input integer nmax;
	integer j, cnt, d1, d2;
	begin
		cnt = nmin + (rbits(32) % (nmax - nmin + 1));
		fill_n = cnt;
		for (j = 0; j < cnt; j = j + 1) begin
			d1 = 4 + rbits(2); d2 = 4 + rbits(2);
			case (rbits(3))
			0: emit({4'b1101, d1[2:0], 3'b010, 3'b000, d2[2:0]});        // ADD.L Dd2,Dd1
			1: emit({4'b1001, d1[2:0], 3'b010, 3'b000, d2[2:0]});        // SUB.L Dd2,Dd1
			2: emit({4'b1011, d2[2:0], 3'b110, 3'b000, d1[2:0]});        // EOR.L Dd2,Dd1
			3: emit(16'h4680 | d1[2:0]);                                  // NOT.L Dd1
			4: begin r3 = rbits(3); emit(16'hE198 | {r3[2:0], 9'd0} | d1[2:0]); end   // ROL.L #n,Dd1
			default: emit(16'h28C0 | d1[2:0]);                            // MOVE.L Dd1,(A4)+
			endcase
		end
	end
endtask

// Raise level l under mask 7 and wait until the write has landed, then a
// few instructions more for the input synchronizers.
task raise;
	input integer l;
	begin
		emit(16'h31FC); emit(l[15:0]); emit(IRQ_SET[15:0]);   // MOVE.W #l,IRQ_SET.W
		emit(16'h4A78); emit(IRQ_SET[15:0]);                   // TST.W IRQ_SET.W
		emit(16'h67FA);                                        // BEQ.S *-4
		emit(16'h4E71); emit(16'h4E71); emit(16'h4E71);
	end
endtask

// A higher level, raised by the bench the moment a core reads the first
// interrupt's autovector: it is pending by the time that handler's first
// instruction arrives, on both cores, so the second interrupt nests there
// -- behind the trace an interrupt carried into the handler, where there
// is one. Level 7 is the edge-triggered one.
task chain;
	input integer l;
	begin
		hl = l + 1 + (rbits(32) % (7 - l));
		emit(16'h31FC); emit(hl[15:0]); emit(IRQ_CHAIN[15:0]);   // MOVE.W #h,IRQ_CHAIN.W
		n_chain = n_chain + 1;
		count_irq(1, 0);   // taken in the first handler: supervisor, M clear
	end
endtask

// An instruction that always traps, as the one the interrupt holds: CHK.W
// D2,D2 (D2 = -1) or DIVU.W D3,D4 (D3 = 0), format $2, or ILLEGAL or TRAP
// #2, format $0 -- a verdict known from decode, which the hold cycle behind
// the SR write must not latch either. The interrupt goes first and
// must stack the SR as it is, not with the trap's own flag effects; the
// trap follows when the handler returns, and the generic handler resumes
// past it. trap_resume goes BEFORE the instruction that lowers the mask
// (4 words), which is 2 words, so the trapping one is 14 bytes on.
task trap_resume;
	reg [31:0] ra;
	begin
		ra = here(0) + 32'd14;
		emit(16'h21FC); emit(ra[31:16]); emit(ra[15:0]); emit(RESUME[15:0]);   // MOVE.L #ra,RESUME.W
	end
endtask
task trap_insn;
	begin
		case (rbits(2))
		0: begin emit(16'h4582); n_trapfill = n_trapfill + 1; end   // CHK.W D2,D2
		1: begin emit(16'h88C3); n_trapfill = n_trapfill + 1; end   // DIVU.W D3,D4
		2: begin emit(16'h4AFC); n_trap8 = n_trap8 + 1; end         // ILLEGAL
		default: begin emit(16'h4E42); n_trap8 = n_trap8 + 1; end   // TRAP #2
		endcase
	end
endtask

function [15:0] mk_sr;
	input integer fs, fm, fmask;
	input [4:0] ccr;
	begin mk_sr = {2'b00, fs[0], fm[0], 1'b0, fmask[2:0], 3'b000, ccr}; end
endfunction

// Back to the invariant every event starts from: supervisor, M clear, mask
// 7. A user-mode program first traps back to supervisor mode.
task settle;
	input integer fs;
	begin
		if (fs == 0) emit(16'h4E41);                // TRAP #1
		emit(16'h46FC); emit(16'h2700);             // MOVE #$2700,SR
	end
endtask

// Count an interrupt the event will take, from the SR it is taken under.
task count_irq;
	input integer fs, fm;
	begin
		n_irq = n_irq + 1;
		if (fm) n_irq_m1 = n_irq_m1 + 1;
		if (!fs) n_irq_user = n_irq_user + 1;
	end
endtask

// A frame pushed with MOVE #imm,-(An): 3F3C/2F3C for A7, 373C/273C for A3.
// Format $2 carries its extra longword first (highest address). The PC is
// patched once the target's address is known (tgt_at).
task push_frame;
	input integer a3;
	input integer ffmt;
	input [15:0] fsr;
	begin
		if (ffmt == 2) begin
			r3 = rbits(32);
			emit(a3 ? 16'h273C : 16'h2F3C); emit(r3[31:16]); emit({r3[15:1], 1'b0});
			emit(a3 ? 16'h373C : 16'h3F3C); emit(16'h2018);        // format $2, vector 6
		end else begin
			emit(a3 ? 16'h373C : 16'h3F3C); emit(16'h0080);        // format $0, vector 32
		end
		emit(a3 ? 16'h273C : 16'h2F3C); tgt_at = pw; emit(16'h0000); emit(16'h0000);
		emit(a3 ? 16'h373C : 16'h3F3C); emit(fsr);
	end
endtask

task patch_target;
	reg [31:0] t;
	begin
		t = here(0);
		prog[tgt_at] = t[31:16]; prog[tgt_at + 1] = t[15:0];
	end
endtask

task gen_program;
	input [31:0] seed;
	begin
		rnd = seed;
		pw  = 0;
		emit(16'h203C); emit(MSP_TOP[31:16]); emit(MSP_TOP[15:0]);   // MOVE.L #MSP_TOP,D0
		emit(16'h4E7B); emit(16'h0803);                               // MOVEC D0,MSP
		emit(16'h203C); emit(USP_TOP[31:16]); emit(USP_TOP[15:0]);   // MOVE.L #USP_TOP,D0
		emit(16'h4E7B); emit(16'h0800);                               // MOVEC D0,USP
		emit(16'h287C); emit(SCR_LO[31:16]); emit(SCR_LO[15:0]);     // MOVEA.L #SCR_LO,A4
		emit(16'h74FF);                                               // MOVEQ #-1,D2
		emit(16'h7600);                                               // MOVEQ #0,D3
		for (k = 4; k < 8; k = k + 1) begin
			r1 = rbits(32);
			emit({4'b0010, k[2:0], 6'b000_111, 3'b100}); emit(r1[31:16]); emit(r1[15:0]);
		end

		for (ev = 0; ev < NEV; ev = ev + 1) begin
			m0  = rbits(1);
			lvl = 1 + (rbits(32) % 6);
			msk = rbits(32) % lvl;               // below the level: taken
			s   = rbits(1);
			m   = rbits(1);
			case (rbits(32) % 7)
			0: begin                                                  // MOVE #sr,SR
				emit(16'h46FC); emit({3'b001, m0[0], 12'h700});
				raise(lvl);
				if (rbits(2) == 0) chain(lvl);
				filler(0, 2);
				tf = (rbits(2) == 0);
				if (tf) trap_resume;
				emit(16'h46FC); emit(mk_sr(s, m, msk, rbits(5)));
				if (tf) trap_insn;
				count_irq(s, m);
				filler(1, 3);
				settle(s);
			end
			1: begin                                                  // ANDI #sr,SR
				emit(16'h46FC); emit({3'b001, m0[0], 12'h700});
				raise(lvl);
				filler(0, 2);
				r1 = rbits(5);
				tf = (rbits(2) == 0);
				if (tf) trap_resume;
				emit(16'h027C); emit({2'b11, s[0], m[0], 1'b1, msk[2:0], 3'b111, r1[4:0]});
				if (tf) trap_insn;
				count_irq(s, m0 & m);
				filler(1, 3);
				settle(s);
			end
			2: begin                                                  // STOP, a delayed level
				if (rbits(2) == 0) lvl = 7;
				if (lvl == 7) begin msk = rbits(3); n_nmi = n_nmi + 1; end
				emit(16'h46FC); emit({3'b001, m0[0], 12'h700});
				emit(16'h31FC); emit(lvl[15:0]); emit(IRQ_DLY[15:0]);   // MOVE.W #l,IRQ_DLY.W
				emit(16'h4E72); emit(mk_sr(1, m, msk, rbits(5)));      // STOP #sr
				count_irq(1, m);
				n_stop = n_stop + 1;
				filler(1, 2);
				settle(1);
			end
			3: begin                                                  // RTE from a built frame
				irq_on = rbits(1);
				if (!irq_on) msk = rbits(3);
				emit(16'h46FC); emit({3'b001, m0[0], 12'h700});
				if (irq_on) raise(lvl);
				fmt = rbits(1) ? 2 : 0;
				push_frame(0, fmt, mk_sr(s, m, msk, rbits(5)));
				emit(16'h4E73);                                       // RTE
				patch_target;
				if (irq_on) count_irq(s, m);
				n_rte = n_rte + 1;
				filler(1, 3);
				settle(s);
			end
			4: begin                                                  // RTE through a throwaway
				irq_on = rbits(1);
				if (!irq_on) msk = rbits(3);
				v = rbits(32) % 4;                                    // 0,1: master stack
				emit(16'h46FC); emit(16'h2700);
				if (irq_on) raise(lvl);
				fmt = rbits(1) ? 2 : 0;
				sr  = mk_sr(s, m, msk, rbits(5));
				if (v <= 1) begin
					emit(16'h4E7A); emit(16'hB803);                   // MOVEC MSP,A3
					push_frame(1, fmt, sr);
					emit(16'h4E7B); emit(16'hB803);                   // MOVEC A3,MSP
					r1 = {3'b001, 1'b1, 12'h700} | rbits(5);          // S, M
					n_f1_msp = n_f1_msp + 1;
				end else if (v == 2) begin
					push_frame(0, fmt, sr);                           // the same (interrupt) stack
					r1 = {3'b001, 1'b0, 12'h700} | rbits(5);
					n_f1_isp = n_f1_isp + 1;
				end else begin
					emit(16'h4E6B);                                   // MOVE USP,A3
					push_frame(1, fmt, sr);
					emit(16'h4E63);                                   // MOVE A3,USP
					r2 = rbits(1);
					r1 = {3'b000, r2[0], 12'h700} | rbits(5);         // user mode
					n_f1_usp = n_f1_usp + 1;
				end
				emit(16'h3F3C); emit(16'h1064);                       // format $1, vector 25
				r3 = rbits(32);
				emit(16'h2F3C); emit(r3[31:16]); emit({r3[15:1], 1'b0});
				emit(16'h3F3C); emit(r1[15:0]);
				emit(16'h4E73);                                       // RTE
				patch_target;
				if (irq_on) count_irq(s, m);
				n_f1 = n_f1 + 1;
				filler(1, 3);
				settle(s);
			end
			5: begin                                                  // traced through the mask drop
				// T1 from the instruction after the first MOVE to SR, so the
				// one that lowers the mask is traced: its trace and the
				// interrupt are both owed at the same boundary. The trace is
				// taken first and the interrupt before its handler runs
				// (ap040_core.v's S_EXC_JMP), stacking the handler's address.
				// The level is raised before T1 is set: a traced poll loop
				// would log as many traces as each core happened to spin.
				emit(16'h46FC); emit(16'h2700);
				raise(lvl);
				if (rbits(1)) chain(lvl);
				r1 = rbits(5);
				emit(16'h46FC); emit({8'hA7, 3'b000, r1[4:0]});         // MOVE #$A7xx,SR: T1
				filler(1, 2); c1 = fill_n;
				t_new = rbits(1);
				emit(16'h46FC); emit(mk_sr(s, m, msk, rbits(5)) | {t_new[0], 15'd0});
				count_irq(s, m);
				n_trace = n_trace + c1 + 1;
				if (t_new) begin
					// Still traced after it: the fillers, and the MOVE #$2700,SR
					// that ends the event (TRAP #1 is not traced: no exception
					// entry leaves one behind it, and its handler returns T1).
					filler(1, 2);
					n_trace = n_trace + fill_n + 1;
				end
				n_trace_ev = n_trace_ev + 1;
				settle(s);
			end
			default: filler(2, 5);
			endcase
		end

		emit(16'h48F9); emit(16'h7FFF); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0]);   // MOVEM.L D0-A6,DUMP
		emit(16'h4E7A); emit(16'h0800);                                                  // MOVEC USP,D0
		emit(16'h23C0); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0] + 16'd60);
		emit(16'h4E7A); emit(16'h0804);                                                  // MOVEC ISP,D0
		emit(16'h23C0); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0] + 16'd64);
		emit(16'h4E7A); emit(16'h0803);                                                  // MOVEC MSP,D0
		emit(16'h23C0); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0] + 16'd68);
		emit(16'h40C0);                                                                  // MOVE SR,D0
		emit(16'h33C0); emit(DUMP_BASE[31:16]); emit(DUMP_BASE[15:0] + 16'd72);
		emit(16'h7001);
		emit(16'h23C0); emit(DONE_ADDR[31:16]); emit(DONE_ADDR[15:0]);
		emit(16'h60FE);
	end
endtask

reg [15:0] memp [0:MEM_WORDS-1];
reg [15:0] memf [0:MEM_WORDS-1];
integer i;

task put;
	input [31:0] addr;
	input [15:0] word;
	begin memp[addr >> 1] = word; memf[addr >> 1] = word; end
endtask

task put_handlers;
	begin
		// tb_ap040_pipe_fxdual.v's: log the frame, resume at RESUME.
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
		// TRAP #1: ORI.W #$2000,(A7); RTE.
		put(SUPER + 32'h00, 16'h0057); put(SUPER + 32'h02, 16'h2000);
		put(SUPER + 32'h04, 16'h4E73);
		// The interrupt handler.
		put(IRQ_HANDLER + 32'h00, 16'h48E7); put(IRQ_HANDLER + 32'h02, 16'h8080);   // MOVEM.L D0/A0,-(A7)
		put(IRQ_HANDLER + 32'h04, 16'h2078); put(IRQ_HANDLER + 32'h06, LOGPTR[15:0]); // MOVEA.L LOGPTR.W,A0
		put(IRQ_HANDLER + 32'h08, 16'h40C0);                                          // MOVE SR,D0
		put(IRQ_HANDLER + 32'h0A, 16'h30C0);                                          // MOVE.W D0,(A0)+
		put(IRQ_HANDLER + 32'h0C, 16'h20EF); put(IRQ_HANDLER + 32'h0E, 16'h0008);   // MOVE.L 8(A7),(A0)+
		put(IRQ_HANDLER + 32'h10, 16'h20EF); put(IRQ_HANDLER + 32'h12, 16'h000C);   // MOVE.L 12(A7),(A0)+
		put(IRQ_HANDLER + 32'h14, 16'h20CF);                                          // MOVE.L A7,(A0)+
		put(IRQ_HANDLER + 32'h16, 16'h4E7A); put(IRQ_HANDLER + 32'h18, 16'h0803);   // MOVEC MSP,D0
		put(IRQ_HANDLER + 32'h1A, 16'h20C0);                                          // MOVE.L D0,(A0)+
		put(IRQ_HANDLER + 32'h1C, 16'h4E7A); put(IRQ_HANDLER + 32'h1E, 16'h0800);   // MOVEC USP,D0
		put(IRQ_HANDLER + 32'h20, 16'h20C0);                                          // MOVE.L D0,(A0)+
		put(IRQ_HANDLER + 32'h22, 16'h21C8); put(IRQ_HANDLER + 32'h24, LOGPTR[15:0]); // MOVE.L A0,LOGPTR.W
		put(IRQ_HANDLER + 32'h26, 16'h4278); put(IRQ_HANDLER + 32'h28, IRQ_SET[15:0]); // CLR.W IRQ_SET.W
		put(IRQ_HANDLER + 32'h2A, 16'h4A78); put(IRQ_HANDLER + 32'h2C, IRQ_SET[15:0]); // TST.W IRQ_SET.W
		put(IRQ_HANDLER + 32'h2E, 16'h66FA);                                          // BNE.S *-4
		put(IRQ_HANDLER + 32'h30, 16'h4E71); put(IRQ_HANDLER + 32'h32, 16'h4E71);
		put(IRQ_HANDLER + 32'h34, 16'h4E71); put(IRQ_HANDLER + 32'h36, 16'h4E71);
		put(IRQ_HANDLER + 32'h38, 16'h4CDF); put(IRQ_HANDLER + 32'h3A, 16'h0101);   // MOVEM.L (A7)+,D0/A0
		put(IRQ_HANDLER + 32'h3C, 16'h4E73);                                          // RTE
		// The trace handler: the format-$2 frame's SR, PC, format word and
		// address field, twelve bytes.
		put(TRC_HANDLER + 32'h00, 16'h48E7); put(TRC_HANDLER + 32'h02, 16'h8080);   // MOVEM.L D0/A0,-(A7)
		put(TRC_HANDLER + 32'h04, 16'h2078); put(TRC_HANDLER + 32'h06, LOGPTR[15:0]); // MOVEA.L LOGPTR.W,A0
		put(TRC_HANDLER + 32'h08, 16'h20EF); put(TRC_HANDLER + 32'h0A, 16'h0008);   // MOVE.L 8(A7),(A0)+
		put(TRC_HANDLER + 32'h0C, 16'h20EF); put(TRC_HANDLER + 32'h0E, 16'h000C);   // MOVE.L 12(A7),(A0)+
		put(TRC_HANDLER + 32'h10, 16'h20EF); put(TRC_HANDLER + 32'h12, 16'h0010);   // MOVE.L 16(A7),(A0)+
		put(TRC_HANDLER + 32'h14, 16'h21C8); put(TRC_HANDLER + 32'h16, LOGPTR[15:0]); // MOVE.L A0,LOGPTR.W
		put(TRC_HANDLER + 32'h18, 16'h4CDF); put(TRC_HANDLER + 32'h1A, 16'h0101);   // MOVEM.L (A7)+,D0/A0
		put(TRC_HANDLER + 32'h1C, 16'h4E73);                                          // RTE
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
			if (i >= 25 && i <= 31) begin put(i*4, IRQ_HANDLER[31:16]); put(i*4 + 2, IRQ_HANDLER[15:0]); end
			else if (i == 33)       begin put(i*4, SUPER[31:16]);       put(i*4 + 2, SUPER[15:0]); end
			else if (i == 9)        begin put(i*4, TRC_HANDLER[31:16]); put(i*4 + 2, TRC_HANDLER[15:0]); end
			else                    begin put(i*4, HANDLER[31:16]);     put(i*4 + 2, HANDLER[15:0]); end
		end
		put_handlers;
		put(RESUME, 16'h0000); put(RESUME + 2, 16'h0000);
		put(LOGPTR, LOG_BASE[31:16]); put(LOGPTR + 2, LOG_BASE[15:0]);
		put(IRQ_SET, 16'h0000); put(IRQ_DLY, 16'h0000); put(IRQ_CHAIN, 16'h0000);
		for (i = LOG_BASE >> 1; i < LOG_END >> 1; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = DUMP_BASE >> 1; i < (DUMP_BASE >> 1) + 40; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = STK_LO >> 1; i < MSP_TOP >> 1; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = SCR_LO >> 1; i < SCR_HI >> 1; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = 0; i < pw; i = i + 1) put(PROG_BASE + 2*i, prog[i]);
		put(DONE_ADDR, 16'h0000); put(DONE_ADDR + 2, 16'h0000);
	end
endtask

// Each core's level register, and its delayed set.
integer    p_dly, f_dly;
reg  [2:0] p_dly_v, f_dly_v;
wire [2:0] p_lvl = memp[IRQ_SET >> 1][2:0];
wire [2:0] f_lvl = memf[IRQ_SET >> 1][2:0];

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
	.clk (clk), .nreset (nreset), .ce (1'b1), .clkena_in (p_clkena),
	.irq_lvl (p_lvl),
	.berr (1'b0),   // no bus errors in this bench
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // no MMU walks here
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
	if (!nreset) begin p_ready <= 1'b0; p_dly <= 0; end
	else begin
		p_ready <= 1'b0;
		if (p_dly > 1) p_dly <= p_dly - 1;
		else if (p_dly == 1) begin p_dly <= 0; memp[IRQ_SET >> 1] <= {13'd0, p_dly_v}; end
		if (p_busstate != `AP040_BUS_IDLE && !p_ready) begin
			p_ready <= 1'b1;
			if (p_busstate == `AP040_BUS_WRITE) begin
				if (!p_nuds) memp[p_widx][15:8] <= p_dwrite[15:8];
				if (!p_nlds) memp[p_widx][7:0]  <= p_dwrite[7:0];
				if (p_widx == (IRQ_DLY >> 1)) begin p_dly <= DELAY; p_dly_v <= p_dwrite[2:0]; end
			end else if (p_addr[31:0] >= 32'h64 && p_addr[31:0] <= 32'h7C && p_addr[1:0] == 2'b00 &&
			             memp[IRQ_CHAIN >> 1] != 16'd0) begin
				memp[IRQ_SET >> 1]   <= memp[IRQ_CHAIN >> 1];
				memp[IRQ_CHAIN >> 1] <= 16'd0;
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
	.data_in(f_din), .ipl(~f_lvl), .ipl_autovector(1'b1), .berr(1'b0),
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
	if (!nreset) begin f_ready <= 1'b0; f_dly <= 0; end
	else begin
		f_ready <= 1'b0;
		if (f_dly > 1) f_dly <= f_dly - 1;
		else if (f_dly == 1) begin f_dly <= 0; memf[IRQ_SET >> 1] <= {13'd0, f_dly_v}; end
		if (f_busstate != `AP040_BUS_IDLE && !f_ready) begin
			f_ready <= 1'b1;
			if (f_busstate == `AP040_BUS_WRITE) begin
				if (!f_nuds) memf[f_widx][15:8] <= f_dwrite[15:8];
				if (!f_nlds) memf[f_widx][7:0]  <= f_dwrite[7:0];
				if (f_widx == (IRQ_DLY >> 1)) begin f_dly <= DELAY; f_dly_v <= f_dwrite[2:0]; end
			end else if (f_addr[31:0] >= 32'h64 && f_addr[31:0] <= 32'h7C && f_addr[1:0] == 2'b00 &&
			             memf[IRQ_CHAIN >> 1] != 16'd0) begin
				memf[IRQ_SET >> 1]   <= memf[IRQ_CHAIN >> 1];
				memf[IRQ_CHAIN >> 1] <= 16'd0;
			end
		end
	end
end

// +irqtrace: the pipelined core's boundary, cycle by cycle.
reg irqtrace = 0;
initial irqtrace = $test$plusargs("irqtrace");
always @(posedge clk) if (irqtrace && nreset && dut_p.u_cpu.u_eaf.eac_valid)
	$display("%0t eac %h stall %b arm %b pend %b lvl %0d sr_ea %h sr %h hold %b take %b ex %b/%h",
	         $time, dut_p.u_cpu.u_eaf.eac_pc, dut_p.u_cpu.u_eaf.eaf_stall, dut_p.u_cpu.u_eaf.irq_arm,
	         dut_p.u_cpu.irq_pend, p_lvl, dut_p.u_cpu.sr_resolved_ea, dut_p.u_cpu.sr,
	         dut_p.u_cpu.u_eaf.trace_hold, dut_p.u_cpu.u_eaf.irq_take,
	         dut_p.u_cpu.u_eaf.eaf_valid, dut_p.u_cpu.u_eaf.eaf_pc);

// +fetchtrace +t0=<ps> +t1=<ps>: fetch and decode in a window.
reg fetchtrace = 0;
reg [63:0] ft0, ft1;
initial begin
	fetchtrace = $test$plusargs("fetchtrace");
	if (!$value$plusargs("t0=%d", ft0)) ft0 = 0;
	if (!$value$plusargs("t1=%d", ft1)) ft1 = 0;
end
always @(posedge clk) if (fetchtrace && $time >= ft0 && $time <= ft1)
	$display("%0t F stopped %b ifv %b ifpc %h op %h flush %b redir %b/%h idv %b idpc %h eac %b/%h",
	         $time, dut_p.u_cpu.stopped, dut_p.u_cpu.if_valid, dut_p.u_cpu.u_if.if_pc, dut_p.u_cpu.u_if.if_opcode,
	         dut_p.u_cpu.flush, dut_p.u_cpu.final_redirect_valid, dut_p.u_cpu.final_redirect_pc,
	         dut_p.u_cpu.u_id.id_valid, dut_p.u_cpu.u_id.id_pc, dut_p.u_cpu.u_eaf.eac_valid, dut_p.u_cpu.u_eaf.eac_pc);

integer errors = 0;
integer cyc, round, mism, nlog_p, nlog_f, show, logged;
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
	n_irq = 0; n_irq_m1 = 0; n_irq_user = 0; n_stop = 0; n_nmi = 0; n_rte = 0;
	n_f1 = 0; n_f1_msp = 0; n_f1_isp = 0; n_f1_usp = 0; logged = 0;
	n_trace = 0; n_trace_ev = 0; n_chain = 0; n_trapfill = 0; n_trap8 = 0;
	for (round = 0; round < NROUND; round = round + 1) begin
		seed = 32'h1A2B_3C4D + round * 32'h9E37_79B9;
		nreset = 0;
		repeat (8) @(posedge clk);
		build_memory(seed);
		if ($test$plusargs("showprog"))
			for (i = 0; i < pw; i = i + 1)
				$display("  round %0d prog %h: %h", round, PROG_BASE + 2*i, prog[i]);
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
			cmp_words("register dump", DUMP_BASE, DUMP_BASE + 76);
			cmp_words("stacks", STK_LO, MSP_TOP);
			cmp_words("store trail", SCR_LO, SCR_HI);
			nlog_p = rd32p(LOGPTR) - LOG_BASE;
			nlog_f = rd32f(LOGPTR) - LOG_BASE;
			if (nlog_p != nlog_f) begin
				errors = errors + 1;
				$display("FAIL: round %0d (seed %h): %0d bytes logged on the pipelined core, %0d on the FSM core",
				         round, seed, nlog_p, nlog_f);
			end
			cmp_words("handler log", LOG_BASE, LOG_END);
			logged = logged + nlog_f;
		end
		if ($value$plusargs("showround=%d", show) && show == round) begin
			for (i = 0; i < pw; i = i + 1)
				$display("  prog %h: %h", PROG_BASE + 2*i, prog[i]);
			for (i = LOG_BASE; i < LOG_BASE + nlog_f + 4; i = i + 2)
				$display("  log %h: pipe %h  fsm %h", i, memp[i >> 1], memf[i >> 1]);
		end
		$display("round %0d: seed %h, %0d program words, %0d cycles, %0d log bytes, %0d mismatches",
		         round, seed, pw, cyc, rd32f(LOGPTR) - LOG_BASE, mism);
	end
	$display("generated: %0d interrupts (%0d with M set, %0d from user mode, %0d woke a STOP, %0d level 7), %0d RTEs from built frames, %0d through a throwaway (%0d master, %0d same stack, %0d user), %0d traces in %0d traced events, %0d nested, %0d held a trapping instruction",
	         n_irq, n_irq_m1, n_irq_user, n_stop, n_nmi, n_rte, n_f1, n_f1_msp, n_f1_isp, n_f1_usp, n_trace, n_trace_ev, n_chain, n_trapfill + n_trap8);
	// Every interrupt logs 22 bytes, every trace 12 and every CHK/zero-divide
	// trap (format $2 through the generic handler) 12; nothing else writes
	// the log here.
	if (logged != 22 * n_irq + 12 * n_trace + 12 * n_trapfill + 8 * n_trap8) begin
		errors = errors + 1;
		$display("FAIL: %0d bytes logged for %0d generated interrupts (22 each), %0d traces and %0d format-$2 traps (12 each), %0d format-$0 traps (8)",
		         logged, n_irq, n_trace, n_trapfill, n_trap8);
	end
	if (n_irq_m1 == 0 || n_irq_user == 0 || n_nmi == 0 || n_f1_msp == 0 || n_f1_isp == 0 || n_f1_usp == 0 || n_trace_ev == 0 || n_chain == 0 || n_trapfill == 0 || n_trap8 == 0) begin
		errors = errors + 1;
		$display("FAIL: a case went ungenerated");
	end
	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);
	$finish;
end

endmodule
