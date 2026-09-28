//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 83)                 //
//                                                                          //
// tb_ap040_pipe_dual.v - the same program on both cores, compared          //
//                                                                          //
// Every bench before this one checks the pipelined core against values I   //
// worked out by hand. This one checks it against rtl/ap040/ap040_core.v,   //
// which passes 3,797 of 3,801 slices of the WinUAE cputest corpus -- real  //
// hardware behaviour, recorded from a real 68040. Where the two cores      //
// disagree, the pipelined one is wrong.                                    //
//                                                                          //
// Both cores run from the SAME generated program in their OWN copy of the  //
// same memory, each behind its own 16-bit bus (milestone 82 put the        //
// pipelined core on the FSM core's adapter, which is what makes this a     //
// fair comparison rather than a comparison of bus models). Neither core's  //
// internals are read: the PROGRAM dumps its own registers with one         //
// MOVEM.L to $1000 and then writes a done flag, so what is compared is     //
// architectural state in memory.                                           //
//                                                                          //
// The program is generated at time 0 from a fixed-seed xorshift, in        //
// four-byte slots: a one-word instruction is padded with a NOP, so every   //
// slot boundary is an instruction boundary and a Bcc displacement of       //
// 4k-2 always lands on one. Only register-to-register forms are            //
// generated -- no memory operands, so no address can wander -- and A7 is   //
// never a destination. Divides are left out: a divide by zero is a trap,   //
// and traps are the next milestone's business, not this one's.             //
//                                                                          //
// If either core takes ANY exception it lands on the shared handler at     //
// $0300, which writes -1 to the done flag, so a trap is reported rather    //
// than hung on. That is also how an instruction the pipelined core does    //
// not decode shows up: vector 4, and the bench says which core stopped.    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_dual;

localparam integer NSLOT     = 96;            // generated instruction slots
localparam integer NROUND    = 16;            // programs, one per seed
localparam [31:0] PROG_BASE  = 32'h0000_0400;
localparam [31:0] DUMP_BASE  = 32'h0000_1000; // MOVEM.L target: 15 longwords
localparam [31:0] DONE_ADDR  = 32'h0000_1100;
localparam [31:0] HANDLER    = 32'h0000_0300;
localparam [31:0] STACK_TOP  = 32'h0000_3000;
// Memory operands point here and nowhere else. A0-A6 start spread
// across the middle of it, and a program of NSLOT slots can move a
// pointer by at most 4*NSLOT bytes, so no access can reach the program,
// the vectors, the dump or the stack below $4000.
localparam [31:0] SCRATCH    = 32'h0000_5000;
localparam [31:0] SCRATCH_LO = 32'h0000_4000;
localparam [31:0] SCRATCH_HI = 32'h0000_7FFE;
localparam integer MEM_WORDS = 32768;
localparam integer TIMEOUT   = 400000;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

//--------------------------------------------------------------------------//
// The program, generated once into prog[] and copied into both memories.   //
//--------------------------------------------------------------------------//

reg [15:0] prog [0:4*NSLOT + 63];
integer    prog_words;
integer    slot_base;    // word index of the first generated slot

reg [31:0] rnd;
function [31:0] xorshift32;
	input [31:0] s;
	reg   [31:0] t;
	begin
		t = s; t = t ^ (t << 13); t = t ^ (t >> 17); t = t ^ (t << 5);
		xorshift32 = t;
	end
endfunction

function [31:0] rbits;            // n random bits, consuming the stream
	input integer n;
	begin
		rnd = xorshift32(rnd);
		rbits = rnd & ((32'd1 << n) - 32'd1);
	end
endfunction

// rbits() hands back a 32-bit value, so everything it feeds goes through a
// named integer and is indexed to width. Dropped straight into a
// concatenation it contributes 32 bits and pushes the opcode's own bits out
// of the low 16: the first run of this bench generated $0017 where $7217
// was meant, which is ORI.B #x,(A7) -- an instruction the FSM core has and
// the pipelined core does not, so it looked exactly like a real finding.
integer slot, kind, dn, dm, an, anw, q, cc, k, sh, imm, dir, want_scc;
integer want_idx, ix, ik, iv, n_idx;
reg     btgt [0:NSLOT+7];   // a forward Bcc lands on this slot
reg [7:0] d8;
integer pro;
reg [15:0] w0, w1;

task gen_program;
	input [31:0] seed;
	begin
		rnd = seed;
		// A0-A6 <- SCRATCH + n*$100, so every memory operand lands in the
		// scratch region. Branch displacements are relative, so it does not
		// matter that this sits ahead of the slots.
		prog_words = 0;
		for (pro = 0; pro < 7; pro = pro + 1) begin
			prog[prog_words + 0] = {4'b0010, pro[2:0], 6'b001_111, 3'b100};  // MOVEA.L #imm,An
			prog[prog_words + 1] = 16'h0000;
			// A4 and A5 start ODD on purpose. A5 is the pointer the Word and
			// Byte forms use (milestone 85) and A4 is one of the five the
			// Long forms use (milestone 86), so every run exercises
			// unaligned accesses of both sizes -- and through the bus,
			// where ap040_bus16_adapter.v splits them into byte/word/byte.
			// Long and Word steps both preserve parity, so they stay odd.
			prog[prog_words + 2] = SCRATCH[15:0] + pro[15:0] * 16'h0100
			                       + ((pro == 4 || pro == 5) ? 16'd1 : 16'd0);
			prog_words = prog_words + 3;
		end
		slot_base = prog_words;

		want_scc = 0; want_idx = 0;
		for (slot = 0; slot < NSLOT + 8; slot = slot + 1) btgt[slot] = 1'b0;
		for (slot = 0; slot < NSLOT; slot = slot + 1) begin
			dn = rbits(3); dm = rbits(3);
			an  = rbits(32) % 5;                         // A0-A4: even, Long-safe
			anw = 5 + (rbits(32) % 2);                   // A5 (odd) or A6: Word/Byte
			w1 = `AP040_OP_NOP;
			// A compare's only product is the condition codes, and this
			// bench compares REGISTERS and MEMORY -- so a compare whose
			// operands are the wrong way round is invisible here unless a
			// branch happens to land right behind it. The slot after every
			// compare therefore captures the flags into a register, where
			// the epilogue's MOVEM will dump them (milestone 89). The
			// mutation that crossed CMPI's operands back over is what
			// showed this was needed: it failed tb_ap040_pipe_immmem.v and
			// passed sixteen differential programs.
			// The consumer of a value EX is still producing (2026-09-24): the
			// slot before it ended in ADD.L Dx,Dx, so Dx arrives on the long
			// forward -- as the index of an access, or as a divisor. The
			// address views make such an instruction wait a cycle; without
			// the wait the index is the MOVEQ's k rather than 2k, and the
			// divisor likewise. No Bcc lands here (btgt), so Dx is always
			// the small, even 2k and the access stays in the scratch lane.
			if (want_idx) begin
				iv = rbits(2); imm = rbits(6);
				d8 = (imm - 32) * 2;
				w1 = {1'b0, ix[2:0], 1'b1, 2'b00, 1'b0, d8};
				case (iv)
				0: w0 = {4'b0010, dn[2:0], 6'b000_110, an[2:0]};        // MOVE.L (d8,An,Dx.L),Dn
				1: w0 = {4'b0010, an[2:0], 6'b110_000, dm[2:0]};        // MOVE.L Dm,(d8,An,Dx.L)
				2: w0 = {4'b1101, dn[2:0], 6'b010_110, an[2:0]};        // ADD.L (d8,An,Dx.L),Dn
				default: begin w0 = {4'b1000, dn[2:0], 6'b011_000, ix[2:0]}; w1 = `AP040_OP_NOP; end   // DIVU.W Dx,Dn
				endcase
				want_idx = 0; n_idx = n_idx + 1;
			end else if (want_scc) begin
				cc = 2 + (rbits(32) % 14);
				w0 = {4'b0101, cc[3:0], 2'b11, 3'b000, dn[2:0]};   // Scc Dn
				want_scc = 0;
			end else begin
			kind = rbits(32) % 46;
			if (kind == 45 && (slot + 1 >= NSLOT || btgt[slot + 1])) kind = 1;
			case (kind)
			45: begin ix = rbits(3); ik = 1 + (rbits(32) % 31);
			    w0 = {4'b0111, ix[2:0], 1'b0, ik[7:0]};               // MOVEQ #k,Dx
			    w1 = {4'b1101, ix[2:0], 6'b010_000, ix[2:0]};         // ADD.L Dx,Dx
			    want_idx = 1; end
			0:  begin imm = rbits(8);
			    w0 = {4'b0111, dn[2:0], 1'b0, imm[7:0]}; end          // MOVEQ
			1:  w0 = {4'b0010, dn[2:0], 6'b000_000, dm[2:0]};       // MOVE.L Dm,Dn
			2:  w0 = {4'b1101, dn[2:0], 6'b010_000, dm[2:0]};       // ADD.L
			3:  w0 = {4'b1001, dn[2:0], 6'b010_000, dm[2:0]};       // SUB.L
			4:  w0 = {4'b1100, dn[2:0], 6'b010_000, dm[2:0]};       // AND.L
			5:  w0 = {4'b1000, dn[2:0], 6'b010_000, dm[2:0]};       // OR.L
			6:  w0 = {4'b1011, dn[2:0], 6'b110_000, dm[2:0]};       // EOR.L Dn,Dm
			7:  begin w0 = {4'b1011, dn[2:0], 6'b010_000, dm[2:0]};  // CMP.L
			    want_scc = 1; end
			8:  begin q = rbits(3);
			    w0 = {4'b0101, q[2:0], 6'b010_000, dn[2:0]}; end     // ADDQ.L
			9:  begin q = rbits(3);
			    w0 = {4'b0101, q[2:0], 6'b110_000, dn[2:0]}; end     // SUBQ.L
			10: begin sh = rbits(3); q = rbits(3); dir = rbits(1);
			    // 1110 ccc d ss i tt rrr: bit 5 is the count source (0 =
			    // immediate) and bits 4:3 the type. Getting those the wrong
			    // way round generated ASR.L D1,D1 -- a REGISTER count, which
			    // milestone 23 did not implement, and the bench duly
			    // reported vector 4. The gap is real and noted; the
			    // generator meant immediate counts.
			    w0 = {4'b1110, q[2:0], dir[0], 2'b10, 1'b0, sh[1:0], dn[2:0]};
			    end                                                  // shift/rotate #q
			11: begin kind = rbits(32) % 5;
			    case (kind)
			    0: w0 = {10'b0100011000, 3'b000, dn[2:0]};           // NOT.L
			    1: w0 = {10'b0100010010, 3'b000, dn[2:0]};           // NEG.L
			    2: w0 = {10'b0100001010, 3'b000, dn[2:0]};           // CLR.L
			    3: w0 = {10'b0100100001, 3'b000, dn[2:0]};           // SWAP
			    default: w0 = {10'b0100100010, 3'b000, dn[2:0]};     // EXT.L
			    endcase end
			12: begin imm = rbits(16);
			    w0 = {10'b0000011001, 3'b000, dn[2:0]}; w1 = imm[15:0]; end   // ADDI.W
			13: begin imm = rbits(16);
			    w0 = {10'b0000001001, 3'b000, dn[2:0]}; w1 = imm[15:0]; end   // ANDI.W
			14: w0 = {4'b1100, dn[2:0], 6'b011_000, dm[2:0]};        // MULU.W
			15: begin                                                // Bcc.B forward
			    cc = 2 + (rbits(32) % 14);
			    k  = 1 + (rbits(32) % 3);
			    if (slot + k >= NSLOT) k = 1;
			    btgt[slot + k] = 1'b1;
			    imm = (4*k - 2) & 32'hFF;
			    w0 = {4'b0110, cc[3:0], imm[7:0]};
			    end
			// MOVEA.L Dm,An is deliberately NOT generated: it puts an
			// arbitrary value in a pointer, and the first run with memory
			// operands did exactly that. The two cores then diverged by one
			// byte -- pipe $6FE3FEF0 where the FSM core had $8D6FE3FE -- an
			// UNALIGNED longword access, which a 68040 performs and this
			// core silently rounds down to the aligned one. That gap is
			// recorded in the plan; keeping the pointers aligned is what
			// lets the rest of the differential run.
			16: w0 = {4'b0011, dn[2:0], 6'b000_000, dm[2:0]};        // MOVE.W Dm,Dn
			// ---- memory operands, always Long and always through an An
			// that the prologue pointed into the scratch region. Long keeps
			// (An)+ and -(An) even, so nothing here is ever misaligned:
			// unaligned data accesses are a separate question and this core
			// has not been asked it yet.
			17: w0 = {4'b0010, dn[2:0], 6'b000_010, an[2:0]};        // MOVE.L (An),Dn
			18: w0 = {4'b0010, an[2:0], 6'b010_000, dm[2:0]};        // MOVE.L Dm,(An)
			19: w0 = {4'b0010, dn[2:0], 6'b000_011, an[2:0]};        // MOVE.L (An)+,Dn
			20: w0 = {4'b0010, an[2:0], 6'b011_000, dm[2:0]};        // MOVE.L Dm,(An)+
			21: w0 = {4'b1101, dn[2:0], 6'b010_010, an[2:0]};        // ADD.L (An),Dn
			22: w0 = {4'b1101, dm[2:0], 6'b110_010, an[2:0]};        // ADD.L Dm,(An)  (RMW)
			23: w0 = {4'b1011, dn[2:0], 6'b010_010, an[2:0]};        // CMP.L (An),Dn
			// ---- Word and Byte through A5 (odd) or A6 (even)
			24: w0 = {4'b0011, dn[2:0], 6'b000_010, anw[2:0]};       // MOVE.W (Aw),Dn
			25: w0 = {4'b0011, anw[2:0], 6'b010_000, dm[2:0]};       // MOVE.W Dm,(Aw)
			26: w0 = {4'b0011, dn[2:0], 6'b000_011, anw[2:0]};       // MOVE.W (Aw)+,Dn
			27: w0 = {4'b1101, dm[2:0], 6'b101_010, anw[2:0]};       // ADD.W Dm,(Aw)
			28: w0 = {4'b0001, dn[2:0], 6'b000_010, anw[2:0]};       // MOVE.B (Aw),Dn
			// ---- an immediate straight into memory (milestone 89), the
			// last of the two decode gaps this bench found. One extension
			// word fits a slot, so these are the Word and Byte forms; the
			// Long ones need two and are covered by
			// tb_ap040_pipe_immmem.v instead. Word through the Long-safe
			// An keeps every access aligned, and the postincrement form
			// steps by two, so the pointer discipline the header describes
			// still holds.
			29: begin imm = rbits(16);
			    w0 = {10'b0000011001, 3'b010, an[2:0]}; w1 = imm[15:0]; end   // ADDI.W #x,(An)
			30: begin imm = rbits(16);
			    w0 = {10'b0000010001, 3'b010, an[2:0]}; w1 = imm[15:0]; end   // SUBI.W #x,(An)
			31: begin imm = rbits(16);
			    w0 = {10'b0000001001, 3'b010, an[2:0]}; w1 = imm[15:0]; end   // ANDI.W #x,(An)
			32: begin imm = rbits(16);
			    w0 = {10'b0000000001, 3'b010, an[2:0]}; w1 = imm[15:0]; end   // ORI.W  #x,(An)
			33: begin imm = rbits(16);
			    w0 = {10'b0000101001, 3'b010, an[2:0]}; w1 = imm[15:0]; end   // EORI.W #x,(An)
			34: begin imm = rbits(16);
			    w0 = {10'b0000110001, 3'b010, an[2:0]}; w1 = imm[15:0];
			    want_scc = 1; end                                            // CMPI.W #x,(An)
			35: begin imm = rbits(16);
			    w0 = {10'b0000011001, 3'b011, an[2:0]}; w1 = imm[15:0]; end   // ADDI.W #x,(An)+
			36: begin imm = rbits(8);
			    w0 = {10'b0000001000, 3'b010, anw[2:0]}; w1 = {8'h00, imm[7:0]}; end // ANDI.B #x,(Aw)
			// ---- the quick forms' other two destinations (milestone 90).
			// The memory ones are ordinary read-modify-writes and step An
			// exactly like the kinds above.
			37: begin q = rbits(3);
			    w0 = {4'b0101, q[2:0], 6'b010_010, an[2:0]}; end      // ADDQ.L #q,(An)
			38: begin q = rbits(3);
			    w0 = {4'b0101, q[2:0], 6'b101_010, an[2:0]}; end      // SUBQ.W #q,(An)
			39: begin q = rbits(3);
			    w0 = {4'b0101, q[2:0], 6'b010_011, an[2:0]}; end      // ADDQ.L #q,(An)+
			// The An destination has to come in a BALANCED PAIR, because
			// every address register here is a pointer the rest of the
			// program dereferences: left to drift, one would walk out of
			// its scratch lane and eventually into the program. A slot
			// holds two words, so the add and its matching subtract go in
			// together and the pointer ends where it started.
			//
			// What that can see: whether the form decodes at all (a core
			// that rejects it traps to vector 4), whether it writes the
			// register the opcode names, and -- through the Scc capture and
			// the Bcc kind -- whether it wrongly writes condition codes.
			// What it CANNOT see is a symmetric width bug, since a pair
			// that wraps one way wraps back the other. That is
			// tb_ap040_pipe_quickdst.v's A1 and A2.
			40: begin q = rbits(3);
			    w0 = {4'b0101, q[2:0], 6'b010_001, an[2:0]};          // ADDQ.L #q,An
			    w1 = {4'b0101, q[2:0], 6'b110_001, an[2:0]}; end      // SUBQ.L #q,An
			// ---- a store with a displacement (milestone 91). Signed and
			// bounded to plus or minus 128 bytes, which keeps every access
			// inside the compared window ($4000..$7FFE) while still going
			// both ways: a dropped sign extension lands the store on the
			// far side of the base, where the other core did not write.
			// The Long form keeps the base's parity by stepping in fours,
			// the Byte form steps in ones, and the Word form goes through
			// the odd A5 as well as the even A6.
			41: begin imm = rbits(6);
			    w0 = {4'b0010, an[2:0], 6'b101_000, dm[2:0]};
			    w1 = (imm - 32) * 4; end                             // MOVE.L Dm,(d,An)
			42: begin imm = rbits(6);
			    w0 = {4'b0011, anw[2:0], 6'b101_000, dm[2:0]};
			    w1 = (imm - 32) * 2; end                             // MOVE.W Dm,(d,Aw)
			43: begin imm = rbits(6);
			    w0 = {4'b0001, anw[2:0], 6'b101_000, dm[2:0]};
			    w1 = imm - 32; end                                   // MOVE.B Dm,(d,Aw)
			// A shift counted by a REGISTER (milestone 87), so the count is
			// whatever dm happens to hold: 0 to 63 after the modulo, which
			// covers both cases the immediate form cannot express -- more
			// than 32, and zero.
			default: begin sh = rbits(3); dir = rbits(1);
			    w0 = {4'b1110, dm[2:0], dir[0], 2'b10, 1'b1, sh[1:0], dn[2:0]};
			    end
			endcase
			end
			prog[slot_base + 2*slot]     = w0;
			prog[slot_base + 2*slot + 1] = w1;
		end

		// Epilogue: dump D0-D7/A0-A6, then flag done.
		prog_words = slot_base + 2*NSLOT;
		prog[prog_words + 0] = 16'h48F9;                  // MOVEM.L regs,$xxx.L
		prog[prog_words + 1] = 16'h7FFF;                  // D0-D7/A0-A6
		prog[prog_words + 2] = DUMP_BASE[31:16];
		prog[prog_words + 3] = DUMP_BASE[15:0];
		prog[prog_words + 4] = 16'h7001;                  // MOVEQ #1,D0
		prog[prog_words + 5] = 16'h23C0;                  // MOVE.L D0,$xxx.L
		prog[prog_words + 6] = DONE_ADDR[31:16];
		prog[prog_words + 7] = DONE_ADDR[15:0];
		prog[prog_words + 8] = 16'h60FE;                  // BRA.B *
		prog_words = prog_words + 9;
	end
endtask

//--------------------------------------------------------------------------//
// Two memories, identical at time 0.                                       //
//--------------------------------------------------------------------------//

reg [15:0] memp [0:MEM_WORDS-1];   // the pipelined core's
reg [15:0] memf [0:MEM_WORDS-1];   // the FSM core's
integer i;

task put;                          // into both
	input [31:0] addr;
	input [15:0] word;
	begin memp[addr >> 1] = word; memf[addr >> 1] = word; end
endtask

task build_memory;
	input [31:0] seed;
	begin
	gen_program(seed);
	for (i = 0; i < MEM_WORDS; i = i + 1) begin
		memp[i] = `AP040_OP_NOP; memf[i] = `AP040_OP_NOP;
	end

	// Reset vector: the FSM core reads these, the pipelined core is given
	// PC_RESET as a parameter and has its ISP poked to match.
	put(32'h0000, STACK_TOP[31:16]); put(32'h0002, STACK_TOP[15:0]);
	put(32'h0004, PROG_BASE[31:16]); put(32'h0006, PROG_BASE[15:0]);

	// Every vector 2..63 to the one handler.
	for (i = 2; i < 64; i = i + 1) begin
		put(i*4,     HANDLER[31:16]);
		put(i*4 + 2, HANDLER[15:0]);
	end

	// The handler records the frame before flagging: the format/vector word
	// at 6(A7) and the stacked PC at 2(A7), so a trap names the vector and
	// the instruction instead of just stopping.
	put(HANDLER + 32'h00, 16'h7000);             // MOVEQ #0,D0
	put(HANDLER + 32'h02, 16'h302F);             // MOVE.W (6,A7),D0
	put(HANDLER + 32'h04, 16'h0006);
	put(HANDLER + 32'h06, 16'h23C0);             // MOVE.L D0,$1108
	put(HANDLER + 32'h08, 16'h0000);
	put(HANDLER + 32'h0A, 16'h1108);
	put(HANDLER + 32'h0C, 16'h202F);             // MOVE.L (2,A7),D0
	put(HANDLER + 32'h0E, 16'h0002);
	put(HANDLER + 32'h10, 16'h23C0);             // MOVE.L D0,$1104
	put(HANDLER + 32'h12, 16'h0000);
	put(HANDLER + 32'h14, 16'h1104);
	put(HANDLER + 32'h16, 16'h70FF);             // MOVEQ #-1,D0
	put(HANDLER + 32'h18, 16'h23C0);             // MOVE.L D0,$1100
	put(HANDLER + 32'h1A, 16'h0000);
	put(HANDLER + 32'h1C, 16'h1100);
	put(HANDLER + 32'h1E, 16'h60FE);             // BRA.B *

	for (i = 0; i < prog_words; i = i + 1)
		put(PROG_BASE + 2*i, prog[i]);

	// Something for the loads to find. NOPs everywhere would make every
	// loaded value the same word.
	for (i = SCRATCH_LO >> 1; i <= SCRATCH_HI >> 1; i = i + 1) begin
		rnd = xorshift32(rnd);
		memp[i] = rnd[15:0]; memf[i] = rnd[15:0];
	end

	put(DONE_ADDR + 0, 16'h0000);
	put(DONE_ADDR + 2, 16'h0000);
	end
endtask

//--------------------------------------------------------------------------//
// The pipelined core, on the 16-bit bus.                                   //
//--------------------------------------------------------------------------//

wire [31:0] p_addr;
wire [15:0] p_dwrite;
wire        p_nwr, p_nuds, p_nlds, p_longword;
wire  [1:0] p_busstate;
wire  [2:0] p_fc;
reg         p_ready;
wire        p_clkena = (p_busstate == `AP040_BUS_IDLE) | p_ready;
wire [31:0] p_widx   = p_addr >> 1;
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
// The FSM core, through its own compatibility wrapper.                     //
//--------------------------------------------------------------------------//

wire [31:0] f_addr;
wire [15:0] f_dwrite;
wire        f_nwr, f_nuds, f_nlds, f_longword, f_nresetout;
wire  [1:0] f_busstate;
wire  [2:0] f_fc;
reg         f_ready;
wire        f_clkena = (f_busstate == `AP040_BUS_IDLE) | f_ready;
wire [31:0] f_widx   = f_addr >> 1;
wire [15:0] f_din    = memf[f_widx];

ap040_tg68k_compat #(
	.AP040_HAS_MMU(0), .AP040_HAS_FPU(0), .AP040_ENABLE_CACHE(0)
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
integer cyc;
reg [31:0] pdone, fdone, pv, fv;

function [31:0] rd32p; input [31:0] a; rd32p = {memp[a >> 1], memp[(a >> 1) + 1]}; endfunction
function [31:0] rd32f; input [31:0] a; rd32f = {memf[a >> 1], memf[(a >> 1) + 1]}; endfunction

task reg_name;
	input integer n;
	begin
		if (n < 8) $write("D%0d", n);
		else       $write("A%0d", n - 8);
	end
endtask

integer round, mism;
reg [31:0] seed;

initial begin
	n_idx = 0;
	for (round = 0; round < NROUND; round = round + 1) begin
		seed = 32'h1357_9BDF + round * 32'h9E37_79B9;   // one program per round

		nreset = 0;
		repeat (8) @(posedge clk);
		build_memory(seed);
		nreset = 1;
		@(posedge clk);
		@(posedge clk);

		// The pipelined core does not read the reset vector; give it the
		// same stack the FSM core just loaded from $0.
		dut_p.u_cpu.u_regfile.isp = STACK_TOP;

		cyc = 0;
		pdone = 0; fdone = 0;
		while (cyc < TIMEOUT && !(pdone != 0 && fdone != 0)) begin
			@(posedge clk);
			cyc = cyc + 1;
			pdone = rd32p(DONE_ADDR);
			fdone = rd32f(DONE_ADDR);
		end

		if (pdone == 0) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the pipelined core never finished (%0d cycles)",
			         round, seed, cyc);
		end
		if (fdone == 0) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the FSM core never finished (%0d cycles)",
			         round, seed, cyc);
		end
		if (pdone == 32'hFFFF_FFFF) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the pipelined core took vector %0d at pc=%h (opcode %h %h)",
			         round, seed, (rd32p(32'h0000_1108) >> 2) & 32'hFF, rd32p(32'h0000_1104),
			         memp[rd32p(32'h0000_1104) >> 1], memp[(rd32p(32'h0000_1104) >> 1) + 1]);
		end
		if (fdone == 32'hFFFF_FFFF) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the FSM core took vector %0d at pc=%h (opcode %h %h)",
			         round, seed, (rd32f(32'h0000_1108) >> 2) & 32'hFF, rd32f(32'h0000_1104),
			         memf[rd32f(32'h0000_1104) >> 1], memf[(rd32f(32'h0000_1104) >> 1) + 1]);
		end

		if (pdone == 32'd1 && fdone == 32'd1) begin
			mism = 0;
			for (i = 0; i < 15; i = i + 1) begin
				pv = rd32p(DUMP_BASE + 4*i);
				fv = rd32f(DUMP_BASE + 4*i);
				if (pv !== fv) begin
					errors = errors + 1;
					mism   = mism + 1;
					$write("FAIL: round %0d (seed %h): ", round, seed);
					reg_name(i);
					$display(" = %h on the pipelined core, %h on the FSM core", pv, fv);
				end
			end
			// ...and every word the program may have stored.
			for (i = SCRATCH_LO >> 1; i <= SCRATCH_HI >> 1; i = i + 1)
				if (memp[i] !== memf[i]) begin
					if (mism < 8) begin
						errors = errors + 1;
						$display("FAIL: round %0d (seed %h): [%h] = %h on the pipelined core, %h on the FSM core",
						         round, seed, i << 1, memp[i], memf[i]);
					end
					mism = mism + 1;
				end
			if (mism == 0)
				$display("  round %0d (seed %h): agreed on 15 registers and %0d scratch words after %0d slots, %0d cycles",
				         round, seed, ((SCRATCH_HI >> 1) - (SCRATCH_LO >> 1)) + 1, NSLOT, cyc);
			else if (mism >= 8)
				$display("      (%0d differing words in all)", mism);
		end
	end

	$display("%0d accesses and divides consumed a value still on EX's long forward", n_idx);
	if (n_idx == 0) begin
		errors = errors + 1;
		$display("FAIL: no consumer of a long forward was generated");
	end
	if (errors == 0)
		$display("ALL TESTS PASSED (%0d programs x %0d slots, both cores agree)", NROUND, NSLOT);
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
