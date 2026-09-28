//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 9a: unified L1)     //
//                                                                          //
// ap040_inst_fetch.v - IF stage                                           //
//                                                                          //
// No longer owns any storage of its own -- milestone 9a lifted the inline  //
// ROM out into ap040_pipe_l1.v, a unified dual-port memory shared with     //
// (eventually) the data path, so self-modifying code is coherent by        //
// construction rather than needing an explicit invalidation path (see      //
// ap040_pipe_l1.v's header for why, and what it still doesn't do). This    //
// stage now drives that memory's port A the same way it used to index its  //
// own `rom` array: l1_addr_a is (fetch_pc - PC_RESET) >> 1, computed        //
// COMBINATIONALLY (not registered here) so ap040_pipe_l1.v's own registered //
// read (`q_a <= mem[address_a]`) reproduces EXACTLY the one-cycle           //
// address-to-data latency the old inline `if_opcode <= rom[rom_idx]` had --  //
// this is a like-for-like timing swap, not a new latency stage. IF is       //
// read-only on this port (wren_a/data_a are tied off at the ap040_pipe_core //
// instantiation, not routed through here at all -- nothing downstream ever  //
// needs IF to write memory).                                               //
//                                                                          //
// Turns from a pure linear-index walker into a real PC register (unchanged //
// since milestone 4): redirect_valid/redirect_pc (driven combinationally   //
// by ap040_decode.v the same cycle a branch is recognized -- see its       //
// header comment) select the next PC instead of a plain +2 sequential       //
// advance. "Stop after PROG_WORDS instructions" -- which every existing     //
// testbench's drain-check relies on -- is tracked by a separate issued      //
// counter, decoupled from the PC value itself, so a program that branches   //
// still issues exactly PROG_WORDS instructions total rather than however    //
// many words a purely linear walk to PROG_WORDS would have covered.         //
//                                                                          //
// Two words at a time (restructuring plan, phase 8, 2026-09-25). Port A   //
// now answers with the word asked for AND the one after it when the       //
// address is the first half of a longword (l1_rdata_a2), and this stage   //
// keeps what decode has not taken in a four-word queue. Decode sees the    //
// first two words of the stream -- the queue, then whatever port A is      //
// returning this cycle, so a fetch after a redirect reaches it as soon as  //
// before -- and takes one, or two when it completes a two-word             //
// instruction in the opcode's cycle (take2). A fetch goes out when at     //
// most one word will be left once decode has taken its words, as the one- //
// word fetch went out when decode took its word; see `room` for why not    //
// sooner. Every word keeps its own fetch-fault flags.                      //
// PROG_WORDS now counts the words decode TAKES. Counting fetches charged  //
// a bench's budget with every word a redirect discards from the queue --   //
// up to six a time where it used to be one -- and cut programs short; a    //
// fetch goes out only while the words taken, queued and in flight leave   //
// room in the budget, and the words a redirect discards are returned to it.//
// if_end is the address after every word fetched and not yet taken, for   //
// the CPU's store snoop, which must cover the queue as it did the word     //
// being presented.                                                         //
//--------------------------------------------------------------------------//

module ap040_inst_fetch
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10
)
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,     // ID cannot accept a word this cycle
	input             take2,        // ...and when it takes one, it takes the second too
	// No fetching at all: the reset vectors are being read, or STOP holds
	// the machine. The stall alone no longer stops a fetch -- the queue
	// fills behind an ordinary stall -- and these two must not: the vector
	// read would find the words at PC_RESET already queued, and a stopped
	// machine keeps only the one word its STOP's redirect fetched.
	input             hold,
	// A mispredict recovery arrives with this asserted (milestone 69). It is
	// a ONE-cycle redirect, and it must land even if this stage is stalled
	// that cycle -- because the stall is coming from an instruction on the
	// path being flushed. Found by tb_ap040_pipe_integration2.v: a not-taken
	// loop-closing BNE whose speculatively fetched successor was a memory
	// load. The load's mem_issue stalled the front end for exactly the cycle
	// the recovery fired; l1_addr_a saw the recovery address but pc did not,
	// and the branch re-executed from stale state forever.
	input             flush,

	input             redirect_valid,
	input      [31:0] redirect_pc,

	// ap040_pipe_l1.v port A -- read-only from here (see header). Requested
	// with l1_req_a, returned with l1_rvalid_a (milestone 80), with the word
	// after it in l1_rdata_a2 when l1_addr_a[1] was 0.
	output     [31:0] l1_addr_a,
	output            l1_req_a,
	input      [15:0] l1_rdata_a,
	input      [15:0] l1_rdata_a2,
	input             l1_rvalid_a,
	input             l1_rflt_a,
	input             l1_rflt_a_bus,

	output            if_valid,
	output reg [31:0] if_pc,
	output     [15:0] if_opcode,
	output            if_flt,
	output            if_flt_bus,
	output            if_valid2,    // the word after it is here too...
	output     [15:0] if_word2,
	output            if_flt2,      // ...and whether its fetch faulted
	output     [31:0] if_end        // the address after every word fetched and not taken
);

reg [31:0] pc;                       // next word to fetch, absent a redirect
reg [31:0] issued;                   // words decode has taken, 0..PROG_WORDS
reg        if_pend;                  // a fetch has been requested and not yet returned
reg  [1:0] pend_n;                   // ...and the words it brings
reg [31:0] pend_pc;                  // ...from this address
reg        pend_dem;                 // ...and decode will want its first word next
reg        retry;                    // a speculative fetch faulted: fetch again on demand
// The queue: qn words from if_pc, oldest first, each with its fault flags.
reg  [2:0] qn;
reg [15:0] qw0, qw1, qw2, qw3;
reg  [3:0] qf, qb;

// This cycle's stream: the queue, then the fetch returning now. port A holds
// its answer as a level until the next request, so a return is counted in
// the one cycle if_pend meets it.
// A bus error on a speculative fetch never surfaces (X2.2, t_exceptions
// 138-139): the fetch that faults is dropped unseen if it was fetched ahead
// of need -- the stream had other words when it went out -- and fetched
// again once the stream is empty, as the one-word fetch always fetched:
// a lasting fault is then taken where decode needs the word, a one-shot one
// is gone. membus retries its own window's faulted prefetches the same way.
wire       ret  = if_pend && l1_rvalid_a;
wire       spec_flt = ret && l1_rflt_a && !pend_dem;
wire [2:0] nret = (ret && !spec_flt) ? {1'b0, pend_n} : 3'd0;
wire [2:0] ns   = qn + nret;             // words in the stream
// The stream's first six words, the queue's then the fetch's.
wire [15:0] s0 = (qn == 3'd0) ? l1_rdata_a  : qw0;
wire [15:0] s1 = (qn == 3'd0) ? l1_rdata_a2 : (qn == 3'd1) ? l1_rdata_a  : qw1;
wire [15:0] s2 = (qn == 3'd1) ? l1_rdata_a2 : (qn == 3'd2) ? l1_rdata_a  : qw2;
wire [15:0] s3 = (qn == 3'd2) ? l1_rdata_a2 : (qn == 3'd3) ? l1_rdata_a  : qw3;
wire [15:0] s4 = (qn == 3'd3) ? l1_rdata_a2 : l1_rdata_a;
wire [15:0] s5 = l1_rdata_a2;
// Fault flags, the same shape: the fetch's flag stands for both its words.
function [5:0] place;
	input [3:0] q;
	input       r;
	input [2:0] n;
	begin
		case (n)
		3'd0:    place = {4'b0000, r, r};
		3'd1:    place = {3'b000, r, r, q[0]};
		3'd2:    place = {2'b00, r, r, q[1:0]};
		3'd3:    place = {1'b0, r, r, q[2:0]};
		default: place = {r, r, q};
		endcase
	end
endfunction
wire  [5:0] sflt = place(qf, l1_rflt_a, qn);
wire  [5:0] sbus = place(qb, l1_rflt_a_bus, qn);

assign if_valid  = (ns != 3'd0);
assign if_valid2 = (ns >= 3'd2);
assign if_opcode = s0;
assign if_word2  = s1;
assign if_flt    = sflt[0];
assign if_flt_bus= sbus[0];
assign if_flt2   = sflt[1];
assign if_end    = pc;

// Decode takes a word whenever it is not stalled and one is here, and the
// second as well on take2.
wire       take  = ce && !flush && !stall_in && if_valid;
wire [2:0] nt    = !take ? 3'd0 : (take2 && if_valid2) ? 3'd2 : 3'd1;
// What is left of the stream once decode has taken its words.
wire [2:0] ns1 = ns - 3'd1;
wire [2:0] ns2 = ns - 3'd2;
wire [2:0] nq  = (nt == 3'd0) ? ns : (nt == 3'd1) ? ns1 : ns2;
// The budget left for new fetches: what decode has taken, and what is
// queued or in flight, is spent. A redirect discards the second and
// spends what decode takes with it.
// take is the pipeline's stall, as late a signal as the core has, and take2
// comes out of decode; the first fit of this stage summed issued + nt,
// compared it and added the result into pc, and every one of its 40 worst
// paths ran through that sum (-4.497 ns). Everything wide is formed here
// from registers, for no word taken, one and two, and the count only picks.
wire [31:0] spent_s = issued + {29'd0, qn} + {30'd0, if_pend ? pend_n : 2'd0};
wire        more_s  = (spent_s < PROG_WORDS);
wire        last_s  = (spent_s + 32'd1 == PROG_WORDS);
wire [31:0] iss1    = issued + 32'd1;
wire [31:0] iss2    = issued + 32'd2;
wire [31:0] iss3    = issued + 32'd3;
wire        more_r  = (nt == 3'd0) ? (issued < PROG_WORDS) : (nt == 3'd1) ? (iss1 < PROG_WORDS) : (iss2 < PROG_WORDS);
wire        last_r  = (nt == 3'd0) ? (iss1 == PROG_WORDS)  : (nt == 3'd1) ? (iss2 == PROG_WORDS) : (iss3 == PROG_WORDS);
wire [31:0] issued_n = (nt == 3'd0) ? issued : (nt == 3'd1) ? iss1 : iss2;
// A redirect lands on the fetch it arrives with. Decode redirects only on
// a word it takes (and while it is stalled it will redirect again); a flush
// lands whatever the stall, since the stall comes from the path it flushes.
// The reset vector's redirect arrives with nothing to take.
wire       redir = ce && (flush || (redirect_valid && !stall_in));
// A sequential fetch when at most one word will be left once decode has
// taken its words -- so a two-word instruction whose second word is in the
// next longword finds it there -- and not while a redirect waits to land,
// so the address below never depends on decode's stall. Asking whenever the
// queue had room for two more, judged before the take, kept the queue full
// and asked for the next longword just as decode took an instruction: on
// the bus tops that put the fetch in front of the instruction's own first
// data access in every iteration of a memory-bound loop (movem_load2 at two
// wait states, 17 -> 18 cycles; ten cases a cycle slower). Asking after the
// take matches the one-word fetch's timing and keeps the core top's two
// words a cycle.
wire       room  = (nq <= 3'd1);
wire       seq   = ce && !hold && !redirect_valid && !flush && more_s && (!if_pend || ret) && room &&
                   !(retry && (qn != 3'd0)) && !spec_flt;
wire       issue = (redir && more_r) || seq;
// The redirect must land on THIS fetch, not merely be scheduled for the
// following one -- otherwise the word at the old (sequential) pc still
// gets fetched first, one cycle late, which is exactly the "poison word
// executes anyway" bug this shape avoids.
wire [31:0] fetch_pc = redirect_valid ? redirect_pc : pc;
// Up to the end of the longword, and never past the budget.
wire [1:0] n_req = (fetch_pc[1] || (redir ? last_r : last_s)) ? 2'd1 : 2'd2;
// The next fetch address, for either length.
wire [31:0] pc_n1    = fetch_pc + 32'd2;
wire [31:0] pc_n2    = fetch_pc + 32'd4;
wire [31:0] pc_issue = (n_req == 2'd1) ? pc_n1 : pc_n2;
wire [31:0] ifpc_1   = if_pc + 32'd2;
wire [31:0] ifpc_2   = if_pc + 32'd4;
assign l1_addr_a = fetch_pc;
assign l1_req_a  = issue;

// What is left of the stream once decode has taken its words: the three
// possible queues are formed from the stream, and the count taken picks
// one, the last select on the path (see the budget above).
function [15:0] by_nt;
	input [2:0]  n;
	input [15:0] a0, a1, a2;
	by_nt = (n == 3'd0) ? a0 : (n == 3'd1) ? a1 : a2;
endfunction

always @(posedge clk) begin
	if (!nreset) begin
		pc      <= PC_RESET;
		issued  <= 32'd0;
		if_pend <= 1'b0;
		pend_n  <= 2'd0;
		pend_pc <= PC_RESET;
		pend_dem<= 1'b0;
		retry   <= 1'b0;
		if_pc   <= PC_RESET;
		qn      <= 3'd0;
		qf      <= 4'd0;
		qb      <= 4'd0;
	end else if (ce) begin
		if (redir) begin
			// Everything fetched along the old path goes; so does the
			// fetch in flight, which port A restarts on the new request.
			qn      <= 3'd0;
			if_pend <= more_r;
			if_pc   <= fetch_pc;
			issued  <= issued_n;
			retry   <= 1'b0;
			pend_dem<= 1'b1;            // decode wants the target next
			pend_pc <= fetch_pc;
			if (more_r) begin
				pend_n <= n_req;
				pc     <= pc_issue;
			end else
				pc     <= fetch_pc;
		end else begin
			qn  <= nq;
			qw0 <= by_nt(nt, s0, s1, s2);
			qw1 <= by_nt(nt, s1, s2, s3);
			qw2 <= by_nt(nt, s2, s3, s4);
			qw3 <= by_nt(nt, s3, s4, s5);
			qf  <= (nt == 3'd0) ? sflt[3:0] : (nt == 3'd1) ? sflt[4:1] : sflt[5:2];
			qb  <= (nt == 3'd0) ? sbus[3:0] : (nt == 3'd1) ? sbus[4:1] : sbus[5:2];
			// The stream's first word moves on by what decode took; an
			// empty stream starts at the fetch now going out.
			if (nq == 3'd0 && !(if_pend && !ret))
				if_pc <= pc;
			else
				if_pc <= (nt == 3'd0) ? if_pc : (nt == 3'd1) ? ifpc_1 : ifpc_2;
			issued <= issued_n;
			if (spec_flt) begin
				// Dropped: fetch it again, on demand.
				retry <= 1'b1;
				pc    <= pend_pc;
			end else if (seq)
				retry <= 1'b0;
			if (seq) begin
				if_pend <= 1'b1;
				pend_n  <= n_req;
				pend_pc <= pc;
				pend_dem<= (nq == 3'd0);
				pc      <= pc_issue;   // fetch_pc is pc: seq excludes a redirect
			end else if (ret)
				if_pend <= 1'b0;
		end
	end
end

`ifdef VERILATOR
// The queue never holds more than four words, nor decode take a word that
// is not there.
always @(posedge clk)
	if (nreset && ce) begin
		if (!redir && nq > 3'd4)
			$error("ap040_inst_fetch: %0d words left for a four-word queue", nq);
		if (take2 && take && !if_valid2)
			$error("ap040_inst_fetch: decode took a second word that is not here");
	end
`endif

endmodule
