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
// address_b addresses the HIGH word of a longword; the low word is          //
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

module ap040_pipe_l1
#(
	parameter AW = 12,   // word address width -> 2**AW words of storage
	parameter DW = 16    // word width
)
(
	input                clock,
	input                nreset,   // see header -- resets wbuf_valid only

	input      [AW-1:0]  address_a,
	input      [DW-1:0]  data_a,
	input                wren_a,
	input                en_a,     // see header -- must match the requester's own stall
	output reg [DW-1:0]  q_a,

	// port B: 32-bit, addresses the HIGH word; low word is address_b+1 -- see
	// header comment.
	input      [AW-1:0]  address_b,
	input       [31:0]   data_b,
	input                wren_b,
	output               wr_busy,   // see header -- hold wren_b/address_b/data_b while high
	output reg  [31:0]   q_b
);

reg [DW-1:0] mem [0:(1<<AW)-1];

integer i;
initial for (i = 0; i < (1<<AW); i = i + 1) mem[i] = `AP040_OP_NOP;

wire [AW-1:0] address_b_lo = address_b + {{(AW-1){1'b0}}, 1'b1};

// Posted write buffer -- see header.
reg              wbuf_valid;
reg [AW-1:0]     wbuf_addr;
reg [31:0]       wbuf_data;

wire [AW-1:0] wbuf_addr_lo = wbuf_addr + {{(AW-1){1'b0}}, 1'b1};

assign wr_busy = wbuf_valid;

// A read this cycle whose address matches the still-undrained buffered
// write must see the buffered value, not stale mem[] content -- see header.
wire wbuf_hits_read = wbuf_valid && (wbuf_addr == address_b);

always @(posedge clock) begin
	if (en_a) begin
		if (wren_a) mem[address_a] <= data_a;
		q_a <= mem[address_a];
	end

	if (!nreset) begin
		wbuf_valid <= 1'b0;
	end else begin
		// Drain takes priority over accepting a new post: a write already
		// posted must land before a new one can be buffered (only one
		// entry deep -- see header, including the exact worst-case wait
		// this ordering implies).
		if (wbuf_valid) begin
			mem[wbuf_addr]    <= wbuf_data[31:16];
			mem[wbuf_addr_lo] <= wbuf_data[15:0];
			wbuf_valid        <= 1'b0;
		end else if (wren_b) begin
			wbuf_valid <= 1'b1;
			wbuf_addr  <= address_b;
			wbuf_data  <= data_b;
		end
	end

	q_b <= wbuf_hits_read ? wbuf_data : {mem[address_b], mem[address_b_lo]};
end

endmodule
