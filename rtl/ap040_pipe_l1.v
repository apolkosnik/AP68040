//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 12: posted write     //
// buffer)                                                                  //
//                                                                          //
// ap040_pipe_l1.v - unified instruction/data L1, backing-store-less        //
//                                                                          //
// A true dual-port memory (same port shape/naming convention as            //
// rtl_old/primitives/dpram.v -- forked, not shared, per folder             //
// independence: see ap040_pipe_core.v's header), used as BOTH instruction  //
// and data storage. Port A is ap040_inst_fetch.v's read (unchanged since    //
// milestone 9a). Port B (milestone 9b, new) is ap040_ea_fetch.v's read for  //
// the first memory-referencing instruction, MOVE.L (An),Dn -- see its       //
// header for the stall/timing mechanism that actually consumes it.          //
//                                                                          //
// Port B is 32-bit, not 16-bit like port A: this core's internal memory     //
// interface is meant to be 32-bit-native (the original plan's section 1: a  //
// clean 32-bit request/response internally, with any 16-bit-bus splitting   //
// left to a future host adapter, not built into the core itself).           //
// PORT B IS SIZED (milestone 86): it carries a byte address and a size,     //
// and this module does every bit of placement -- right-aligned on the way   //
// out, placed from the address on the way in, at any alignment. Until then  //
// it returned "the 32 bits containing the address" and the CPU picked lanes //
// out of that, which cannot express a Long at an odd address: those bytes   //
// span THREE words, and no single 32-bit window holds them.                 //
//                                                                          //
// The old shape, for the record:                                            //
// address_b addressed the HIGH word of a longword; the low word was         //
// implicitly address_b+1, big-endian/high-word-first, the same word order   //
// ap040_decode.v's Bcc.L gather and rtl_old's bus16 adapter both already     //
// use. wren_b/data_b are wired for a future store instruction (MOVE.L       //
// Dn,(An) etc.) -- unused by milestone 9b's read-only MOVE.L (An),Dn, tied   //
// off at the ap040_pipe_core.v instantiation -- so the port shape doesn't    //
// need reshaping again when a store instruction lands.                      //
//                                                                          //
// Calling this a "cache" is aspirational today: there is no larger backing //
// store behind it yet for this repo to miss into (no SDRAM/DDR model the   //
// way the Minimig-AGA tree this core was extracted from has), so right now //
// this module simply IS the memory, sized generously (AW=12 -> 4096 words) //
// rather than to any real cache-line/set budget. What DOES carry over      //
// architecturally, and is the actual point of unifying it now rather than  //
// giving each future memory-referencing instruction its own private test   //
// stub: a write through port B is visible to a port-A read on any LATER    //
// cycle for free, just from both ports addressing the same `mem` array --  //
// no explicit invalidation path is needed for self-modifying code to work  //
// correctly, the way a split Harvard I/D cache would need one. Tags,       //
// per-line valid bits, replacement, and a real miss/fill path to an actual //
// backing store are deferred until this repo has a backing store worth     //
// missing into; the port shape below (one address/data/write-enable per    //
// side, registered read) is chosen so that adding those later doesn't      //
// require reshaping the ports IF/EA-fetch already depend on.               //
//                                                                          //
// One real semantic gap this shortcut creates, worth remembering rather    //
// than rediscovering: CINV/CPUSH exist on real hardware because a split    //
// I/D cache can go stale and needs explicit maintenance; on THIS substrate //
// there is no staleness to invalidate, so those instructions can only ever //
// be decoded/accepted as legal no-ops here, never exercised for their      //
// actual observable effect (stale-until-flush) the way rtl_old's t_cache   //
// suite does against the split cache in rtl_old/ap040_cache.v. That's an   //
// accepted trade for this pipeline (non-goal: architecturally-equivalent,  //
// not a transistor-level replica -- AP040_IMPLEMENTATION_PLAN.md section   //
// 2), not an oversight to fix later.                                      //
//                                                                          //
// Default fill is AP040_OP_NOP, not zero: every existing pipe testbench    //
// pokes a handful of program words via hierarchical reference             //
// (`dut.u_l1.mem[N] = ...`, moved here from `dut.u_if.rom[N]` when this    //
// milestone lifted the array out of ap040_inst_fetch.v) and relies on      //
// every OTHER word fetched within the program's PROG_WORDS budget          //
// draining as a harmless NOP. Nothing currently reads port B as data, so   //
// this default has no data-side consequence yet; revisit if/when a test    //
// cares about a genuine zero-fill default for data.                       //
//                                                                          //
// Timing note (why this is a drop-in for the UNSTALLED case): ap040_inst_    //
// fetch.v's own array read, before milestone 9a, was already a registered   //
// (1-cycle-latency) read -- functionally identical to this module's own      //
// `q_a <= mem[address_a]`. With nothing ever stalling (true through           //
// milestone 9a), having IF drive address_a combinationally reproduces the     //
// exact same address-to-data latency IF already had.                          //
//                                                                          //
// en_a (milestone 10, real bug fix, not a drop-in): a stall breaks the      //
// assumption above. ap040_inst_fetch.v's internal `pc` register is ALWAYS   //
// one step ahead of if_pc/if_opcode by design (pc = next fetch address,     //
// if_pc/if_opcode = the word currently being presented, one cycle younger). //
// When id_stall freezes if_pc (correctly holding the word decode still      //
// needs), pc freezes too -- but at its OWN, already-one-step-further value. //
// Before en_a existed, q_a had no idea any of this happened: it kept        //
// registering mem[address_a] EVERY edge regardless, and since address_a is  //
// combinationally address_a=f(pc), it kept reading the address pc had       //
// ALREADY moved to -- one step past if_pc -- silently overwriting the word  //
// if_opcode was supposed to keep presenting, one edge into the stall. A     //
// real bug, first caught by tb_ap040_pipe_move_disp.v (its case B gather    //
// needed the extension word held steady during a stall caused by an         //
// EARLIER instruction's own memory access); tb_ap040_pipe_move_mem.v's own  //
// stall never happened to land on anything but a harmless NOP, so this      //
// shipped undetected in milestone 9b. Fixed by gating port A's registers    //
// with en_a = ce && !id_stall (ap040_pipe_core.v wires it from the SAME     //
// condition already gating ap040_inst_fetch.v's own if_pc/pc registers) --  //
// when frozen, q_a now correctly HOLDS instead of free-running ahead of     //
// what if_pc represents.                                                   //
//                                                                          //
// Port B has no equivalent bug and gained no equivalent enable: its         //
// address (ap040_ea_fetch.v's l1_addr_b) is a direct combinational          //
// function of ap040_ea_calc.v's OWN registered output, which already        //
// freezes correctly on its own stall_in -- there is no separate,            //
// independently-advancing "one step ahead" register driving it the way      //
// IF's `pc` drives port A. If a future EA mode ever grows something like     //
// a real prefetch queue on the data side, revisit this asymmetry rather      //
// than assume it still holds.                                              //
//                                                                          //
// The other open question -- combinational vs. registered/stalled for       //
// port B once EA-fetch consumes it -- was decided (registered+stalled) in    //
// milestone 9b; see AP040_IMPLEMENTATION_PLAN.md section 5a.                 //
//                                                                            //
// Posted write buffer (milestone 12, new, ahead of BSR/JSR's stack push):    //
// port B is a SINGLE read/write port -- one address bus serving both q_b's   //
// read request and wren_b's write request -- so a store landing the SAME     //
// cycle port B is busy servicing a read (or an earlier still-undrained       //
// write) would otherwise have to stall the whole pipeline until the port     //
// frees up. A 1-entry buffer (wbuf_valid/wbuf_addr/wbuf_data -- one 32-bit    //
// longword, nothing wider is needed: this core never posts more than one     //
// store's worth at a time) decouples that: wren_b POSTS a write, which is     //
// accepted immediately whenever the buffer is empty (wr_busy low) -- the      //
// requester (ap040_ea_fetch.v) can then move on without waiting for the       //
// PHYSICAL mem[] write to land, the same "commit, don't wait for the          //
// backing store" contract a real store buffer gives a pipeline. If the        //
// buffer is still holding an undrained entry (wr_busy high), a NEW post        //
// must wait -- draining always takes priority over accepting a new post        //
// (see the always block), so the two are mutually exclusive per edge, not      //
// simultaneous. From the moment wr_busy is OBSERVED to drop, a held            //
// request is accepted on the very next edge (one cycle) -- but a request       //
// that starts waiting WHILE busy is already high needs the drain's OWN         //
// edge first, then the accept's, i.e. up to two edges from when it first       //
// started waiting (verified by tb_ap040_pipe_l1_wbuf.v's case B, which          //
// caught an early draft both of the RTL's own comment AND of the test's        //
// own edge-counting getting this wrong by one). Still bounded and small,        //
// not indefinite -- this behavioral model has no real port contention to        //
// make it longer. A REAL bus (arbitrating port A/port B against a real           //
// external memory with unknown latency) is explicitly future work for the        //
// BCU, built when the MMU is reimplemented -- not this module's job; see          //
// AP040_IMPLEMENTATION_PLAN.md section 6.                                          //
//                                                                            //
// Read-after-write forwarding: a read (q_b) whose address matches an           //
// undrained buffered write returns the BUFFERED value, not stale mem[]          //
// content -- otherwise a load immediately behind a store to the same            //
// address would see the wrong data for one cycle. Not reachable by any           //
// instruction implemented yet (nothing reads memory in the same window a         //
// store's write might still be buffered -- RTS, a stack pop, will be the          //
// first), built now anyway since it's the module genuinely responsible for         //
// this guarantee and it's cheap; don't let it go untested indefinitely once         //
// something DOES depend on it.                                                      //
//                                                                            //
// Contract on the requester: wren_b/address_b/data_b must be held STABLE       //
// across cycles where wr_busy reads high -- exactly the same "hold your        //
// inputs steady during a stall" discipline eaf_stall already imposes            //
// elsewhere in this pipeline (ap040_ea_fetch.v's mem_issue), not a new one.      //
//                                                                            //
// nreset (milestone 12, new): this module never needed one before -- q_a/q_b   //
// are pure data outputs with no meaningful "reset value" while nothing          //
// downstream trusts them yet (matching how a real BRAM's read port has no        //
// reset either). wbuf_valid is different: it's genuine CONTROL state, not         //
// data, and Verilog gives it X at time 0 with no reset -- which is not             //
// harmless here, unlike q_a/q_b's X: wbuf_hits_read's comparison against an          //
// X wbuf_addr, and the q_b ternary selecting on an X wbuf_valid, both propagate       //
// X into q_b even on a read that has nothing to do with any write, poisoning           //
// completely unrelated instructions. A real bug, caught immediately by the             //
// existing move_mem/move_disp tests going X the moment this milestone's code            //
// was added -- fixed by giving this module the SAME nreset every other stateful          //
// pipe module already has, gating wbuf_valid specifically (mem[]/q_a/q_b keep              //
// their original no-reset treatment; they're still pure data).                             //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

// Request/return handshake (milestone 80). Both ports are REQUESTED (en_a,
// rd_b) and RETURN with a valid (rvalid_a, rvalid_b); the data and its valid
// hold until the next accepted request on that port. Without
// AP040_PIPE_L1_SLOW every read returns the cycle after its request and the
// write buffer drains the cycle after a post -- the timing every stage was
// built against. With it, a deterministic xorshift adds 0-3 cycles to each
// read and 0-3 to each drain, so the same benches prove the requesters wait
// for the return rather than assume it. Port A accepts a new request while
// one is in flight (a redirect abandons the fetch); port B may not, and the
// slow model says ERROR if it happens.
module ap040_pipe_l1
#(
	parameter AW = 12,   // word address width -> 2**AW words of storage
	parameter DW = 16,   // word width
	// Both ports take a 32-bit BYTE address (milestone 81); this module maps
	// it to its own storage, which is a window of 2**AW words starting at
	// PC_RESET.
	parameter [31:0] PC_RESET = 32'h0000_0400
)
(
	input                clock,
	input                nreset,   // see header -- resets wbuf_valid only
	// port A: instruction fetch, 16-bit reads, always even
	input      [31:0]    address_a,
	input      [DW-1:0]  data_a,
	input                wren_a,
	input                en_a,      // request; the requester holds address_a until rvalid_a
	output reg [DW-1:0]  q_a,
	output reg [DW-1:0]  q_a2,      // the word after q_a (phase 8)
	output reg           rvalid_a,
	// port B: sized data accesses at any alignment (milestone 86)
	input      [31:0]    address_b,
	input      [31:0]    data_b,    // right-aligned by size_b
	input                wren_b,
	input       [1:0]    size_b,    // AP040_SZ_B/W/L
	input                rd_b,
	output               wr_busy,
	output reg  [31:0]   q_b,       // right-aligned by size_b
	output reg           rvalid_b
);

reg [DW-1:0] mem [0:(1<<AW)-1];
integer i;
initial for (i = 0; i < (1<<AW); i = i + 1) mem[i] = `AP040_OP_NOP;

// The window map. Only the low AW+1 bits of the difference survive the shift
// and the truncation, and a borrow only ever propagates upward, so this is a
// narrow subtract and is written as one -- spelled as a 32-bit subtract it
// sits after ap040_pipe_cpu.v's port-B mux and costs a nanosecond on the
// critical spine (the fit after milestone 81 measured it).
wire [AW:0] ia_full = address_a[AW:0] - PC_RESET[AW:0];
wire [AW:0] ib_full = address_b[AW:0] - PC_RESET[AW:0];
wire [AW-1:0] ia = ia_full[AW:1];
wire [AW-1:0] ib = ib_full[AW:1];
// PC_RESET is even, so the byte within the word is the address's own bit 0.
wire          ob = address_b[0];

// One-entry write buffer (see header), now holding a sized value rather than
// a lane mask.
reg              wbuf_valid;
reg [AW-1:0]     wbuf_addr;
reg              wbuf_odd;
reg  [1:0]       wbuf_size;
reg [31:0]       wbuf_data;
reg [1:0]        wbuf_hold;    // extra drain cycles left (slow model only)
// Busy only while the buffered write is still being held: in the cycle it
// drains, the next one is taken in its place (restructuring plan, phase 5),
// so a run of stores is one per cycle.
assign wr_busy = wbuf_valid && (wbuf_hold != 2'd0);

// Latency model. In the normal build every extra count is zero and the
// xorshift is optimised away.
`ifdef AP040_PIPE_L1_SLOW
reg [15:0] lfsr;
wire [15:0] lfsr_next = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
wire [1:0] extra_a = lfsr[1:0];
wire [1:0] extra_b = lfsr[5:4];
wire [1:0] extra_w = lfsr[9:8];
`else
wire [1:0] extra_a = 2'd0;
wire [1:0] extra_b = 2'd0;
wire [1:0] extra_w = 2'd0;
`endif

// Port A: one request in flight; a new request restarts it.
reg          a_busy;
reg [1:0]    a_cnt;
reg [AW-1:0] a_addr;

// Port B: one request in flight. A read never overtakes a buffered write --
// it waits for the drain instead of forwarding round it (milestone 86). The
// forward could only ever answer an exactly-matching access, and with sizes
// and alignment in play "matching" stops being a comparison; draining first
// gives the same answer for every overlap, and is what ap040_pipe_membus.v
// does on the bus side, so the two memories now order accesses alike.
reg          b_busy;
reg [1:0]    b_cnt;
reg [AW-1:0] b_addr;
reg          b_odd;
reg  [1:0]   b_size;

function [31:0] read_at;
	input [AW-1:0] a;
	input          odd;
	input   [1:0]  sz;
	begin
		case (sz)
		`AP040_SZ_B: read_at = {24'd0, odd ? mem[a][7:0] : mem[a][15:8]};
		`AP040_SZ_W: read_at = odd ? {16'd0, mem[a][7:0], mem[a + {{(AW-1){1'b0}}, 1'b1}][15:8]}
		                           : {16'd0, mem[a]};
		default:     read_at = odd ? {mem[a][7:0],
		                              mem[a + {{(AW-1){1'b0}}, 1'b1}],
		                              mem[a + {{(AW-2){1'b0}}, 2'b10}][15:8]}
		                           : {mem[a], mem[a + {{(AW-1){1'b0}}, 1'b1}]};
		endcase
	end
endfunction

wire [AW-1:0] wb1 = wbuf_addr + {{(AW-1){1'b0}}, 1'b1};
wire [AW-1:0] wb2 = wbuf_addr + {{(AW-2){1'b0}}, 2'b10};

// Port A never reads a word a write has not yet put in the array (2026-09-25,
// found by tb_ap040_pipe_program_local's t_integer 192). A store into the
// instruction stream raises a refetch in the cycle the write is accepted,
// and that refetch's read went to the array a cycle before the buffered write
// did: it fetched the old instruction again. ap040_pipe_membus.v sends writes
// before fetches, so the bus tops never could. A fetch whose words the
// buffered write, or the one being accepted, covers now waits for the drain,
// as port B's reads do; any other fetch goes ahead.
function covers;
	input [AW-1:0] wa;      // the write's first word
	input          odd;
	input    [1:0] sz;
	input [AW-1:0] fa;      // the fetch's word; the one after it is read too
	reg   [AW-1:0] d;
	begin
		d = fa - wa;
		covers = (d == {AW{1'b0}}) ||                                           // fa itself
		         (d == {AW{1'b1}}) ||                                           // fa+1 is wa
		         ((d == {{(AW-1){1'b0}}, 1'b1}) && (sz == `AP040_SZ_L || (sz == `AP040_SZ_W && odd))) ||
		         ((d == {{(AW-2){1'b0}}, 2'b10}) && (sz == `AP040_SZ_L) && odd);
	end
endfunction
wire          wr_acc = wren_b && !wr_busy;
wire          a_wait_new = (wbuf_valid && covers(wbuf_addr, wbuf_odd, wbuf_size, ia)) ||
                           (wr_acc && covers(ib, ob, size_b, ia));
wire          a_wait_old = (wbuf_valid && covers(wbuf_addr, wbuf_odd, wbuf_size, a_addr)) ||
                           (wr_acc && covers(ib, ob, size_b, a_addr));


always @(posedge clock) begin
`ifdef AP040_PIPE_L1_SLOW
	if (!nreset) lfsr <= 16'hACE1;
	else if (en_a || rd_b || wren_b) lfsr <= lfsr_next;
`endif

	// ---- port A ----
	if (en_a) begin
		if (wren_a) mem[ia] <= data_a;
		a_addr   <= ia;
		a_cnt    <= extra_a;
		a_busy   <= 1'b1;
		rvalid_a <= 1'b0;
	end else if (a_busy) begin
		if (a_cnt == 2'd0) begin
			if (!a_wait_old) begin
				q_a      <= mem[a_addr];
				q_a2     <= mem[a_addr + {{(AW-1){1'b0}}, 1'b1}];
				rvalid_a <= 1'b1;
				a_busy   <= 1'b0;
			end
		end else
			a_cnt <= a_cnt - 2'd1;
	end
	if (en_a && extra_a == 2'd0 && !a_wait_new) begin
		q_a      <= mem[ia];
		q_a2     <= mem[ia + {{(AW-1){1'b0}}, 1'b1}];
		rvalid_a <= 1'b1;
		a_busy   <= 1'b0;
	end

	// ---- port B read ----
	if (rd_b) begin
`ifdef AP040_PIPE_L1_SLOW
		if (b_busy) $display("ERROR: ap040_pipe_l1 port B read issued while one is in flight");
`endif
		b_addr   <= ib;
		b_odd    <= ob;
		b_size   <= size_b;
		b_cnt    <= extra_b;
		b_busy   <= 1'b1;
		rvalid_b <= 1'b0;
	end else if (b_busy && !wbuf_valid) begin
		if (b_cnt == 2'd0) begin
			q_b      <= read_at(b_addr, b_odd, b_size);
			rvalid_b <= 1'b1;
			b_busy   <= 1'b0;
		end else
			b_cnt <= b_cnt - 2'd1;
	end
	if (rd_b && extra_b == 2'd0 && !wbuf_valid) begin
		q_b      <= read_at(ib, ob, size_b);
		rvalid_b <= 1'b1;
		b_busy   <= 1'b0;
	end

	// ---- port B write buffer ----
	if (!nreset) begin
		wbuf_valid <= 1'b0;
		wbuf_hold  <= 2'd0;
	end else begin
		if (wbuf_valid) begin
			if (wbuf_hold == 2'd0) begin
				case (wbuf_size)
				`AP040_SZ_B: begin
					if (wbuf_odd) mem[wbuf_addr][7:0]  <= wbuf_data[7:0];
					else          mem[wbuf_addr][15:8] <= wbuf_data[7:0];
				end
				`AP040_SZ_W: begin
					if (wbuf_odd) begin
						mem[wbuf_addr][7:0] <= wbuf_data[15:8];
						mem[wb1][15:8]      <= wbuf_data[7:0];
					end else
						mem[wbuf_addr] <= wbuf_data[15:0];
				end
				default: begin
					if (wbuf_odd) begin
						mem[wbuf_addr][7:0] <= wbuf_data[31:24];
						mem[wb1]            <= wbuf_data[23:8];
						mem[wb2][15:8]      <= wbuf_data[7:0];
					end else begin
						mem[wbuf_addr] <= wbuf_data[31:16];
						mem[wb1]       <= wbuf_data[15:0];
					end
				end
				endcase
				wbuf_valid <= 1'b0;
			end else
				wbuf_hold <= wbuf_hold - 2'd1;
		end
		if (wren_b && !wr_busy) begin
			wbuf_valid <= 1'b1;
			wbuf_addr  <= ib;
			wbuf_odd   <= ob;
			wbuf_size  <= size_b;
			wbuf_data  <= data_b;
			wbuf_hold  <= extra_w;
		end
	end

	if (!nreset) begin
		a_busy <= 1'b0; rvalid_a <= 1'b0;
		b_busy <= 1'b0; rvalid_b <= 1'b0;
	end
end

endmodule
