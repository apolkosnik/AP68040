//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25)                   //
//                                                                          //
// ap040_pipe_dmu.v - the data memory unit                                  //
//                                                                          //
// Between EA-fetch's port B and the bus controller (ap040_pipe_membus.v),  //
// as the MC68040's DMU sits between its operand accesses and its bus       //
// controller (MC68040UM Figure 4-1): operand addresses are translated here //
// by ap040_pipe_mmu.v's data port, and only physical addresses go on.      //
// Stage A of doc_AP040_PIPELINE_CACHES.md put translation here; stage C    //
// the data cache, write-through (below).                                   //
//                                                                          //
// Toward the CPU this is the bus controller's port-B protocol exactly,     //
// which the bus controller implemented while the MMU sat below it: one     //
// read at a time, returned with a valid that holds as a level; writes      //
// posted; a write that translation can refuse (the CPU's l1_wr_sync: TC.E  //
// or a data TTR enabled) is TENTATIVE -- held busy until it passes,        //
// accepted in the cycle wr_busy drops, the refusal a level until the CPU   //
// has let go (c_wr_drop), and the acceptance kept across a cycle the CPU   //
// was not looking (review 16's receipt). Those rules move here with the    //
// translation; the bus controller sees only writes that have passed, and   //
// reports only physical bus errors.                                        //
//                                                                          //
// Translated from registers. The CPU's port-B address settles late in its  //
// cycle, from EA-fetch's stalls; an ATC compare and a physical address     //
// after it do not fit in 25 ns (the first draft, which translated in the   //
// request's own cycle, fitted at -5.3 ns). So with translation able to     //
// refuse (xlat), a request is latched as it arrives and translated from    //
// the latch, and every signal toward the CPU and the MMU comes from        //
// registers: a read goes to the bus controller the cycle after it is       //
// asked for, through its translated-read input (rx_*), which puts it on    //
// the bus in that cycle -- the bus controller's own timing, which took a   //
// read the cycle it came and put it on the bus the next; a write is        //
// accepted the cycle after its translation passes -- a cycle after it is   //
// taken when the MMU's most recent hit covers it, two when its lookup      //
// runs, as the bus controller accepted it. Untranslated (xlat low), reads  //
// and writes go straight through, as they did.                             //
//                                                                          //
// Ordering. A write accepted here goes to the bus controller before any    //
// read that arrives after it: a read waits while the write slot holds      //
// anything, and the bus controller sends the writes it holds before the    //
// reads.                                                                   //
//                                                                          //
// The MMU's data port is held by whoever raised it -- the slot or the read //
// -- at the same address until it passes or faults: a walk cannot be       //
// recalled, and its fault belongs to the access it was for. After a fault  //
// the port is left down for a cycle, which is what the walker waits for    //
// (ap040_pipe_mmu.v's W_DROP).                                             //
//                                                                          //
// Transfers that cross a page with translation on: the two pages are       //
// translated separately and the transfer goes out a byte at a time, each   //
// byte to its own page's physical address, a read's bytes assembled here;  //
// a fault on the second page reports MA. A crossing write is first CHECKED //
// on both pages (the MMU's access check: no M history), so a refused write //
// has written nothing and marked nothing -- as the bus controller's PTEST  //
// probes guaranteed -- then translated for real on each page, which sets   //
// M, and only then accepted and its bytes posted. Every table search a     //
// write needs is thereby over before it is accepted, and the walker never  //
// runs while an accepted write has still to reach memory (wr_pend).        //
//                                                                          //
// MOVES spaces (MC68040UM 3.2, Table 3-2; stage A2). $0, $3, $4 and $7 are //
// alternate address spaces: the address is physical, used without          //
// translation -- no ATC lookup, no search, no protection, no MMU fault --  //
// and not split at a page. $2 and $6 are converted to the data spaces $1   //
// and $5: translated as any data access, and on the bus as data (the bus   //
// controller converts what it is given; this unit its translated reads).   //
//                                                                          //
// The data cache (caches stages C and D; MC68040UM 4). 64 sets of four     //
// 16-byte lines, physically tagged, on stage 0's arrays (a longword a way  //
// a read, byte enables; a tag row a set; the snoop copy); valid bits, a    //
// dirty bit a longword and the replacement counter in flops. CACR DE turns //
// it on; with DE clear every path above is as it was.                      //
//   Reads. Every read is latched and translated (untranslated, the MMU     //
//   passes it at once, with the caching mode a TTR gives, else write-      //
//   through). Its set is read in the cycle its translation passes -- PA9-  //
//   PA2 are the untranslated bits, already in the latch -- and compared    //
//   the next: a hit is answered from the cache, right-aligned by size.     //
//   A miss reads the line, the longword asked for first and the rest       //
//   wrapping, each longword a bus controller read; the first answers the   //
//   read, the rest are handed to later reads of the line as they arrive.   //
//   The line is valid when all four are in; an error on any beat abandons  //
//   it, faulting only the read waiting for that longword.                  //
//   Writes. Written through (CM 00, and every write untranslated with no   //
//   data TTR): sent to the bus controller as they were, each updating a    //
//   line holding it through the byte enables, its dirty bits kept, in the  //
//   order the writes are sent (4.3.1.1; Table 4-4). Copyback (CM 01, the   //
//   translating slot's aligned writes): the write goes to the cache        //
//   instead -- a hit updates the line and marks its longword dirty, a miss //
//   reads the line and then does so, or, allocating nothing, goes to the   //
//   bus alone (4.3.1.2). A write's update waits for a line being read, and //
//   a read's lookup for the update of every write before it.               //
//   Not cached: an access a TTR or page marks inhibited (CM 1x), MOVES to  //
//   an alternate space, a locked access (TAS, CAS, CAS2), a MOVE16 write;  //
//   each pushes a dirty line holding its address -- a write merged into it //
//   first -- and invalidates it (4.3.2, 7.4.5), then goes to the bus. A    //
//   read spanning two longwords, or pages, goes to the bus too, and a      //
//   write that does is written through. Exception frames, the vector fetch //
//   and MOVE16 allocate nothing (4.3.3).                                   //
//   Replacement: the first invalid way, else the counter's, which counts   //
//   every read looked up and every write sent or taken (4.1). A dirty line //
//   replaced is read out through port A before the new one lands on it,    //
//   and its dirty longwords written once the new line is in, ahead of any  //
//   CPU write; no line is read and no read goes to the bus until they are. //
//   If the new line's read errs, the replaced line goes back to its place, //
//   valid and dirty, nothing written (4.6.2; stage R).                     //
//   CINV/CPUSH on the data cache (cm_*), whatever CACR says: all ways, a   //
//   line, or a 4 KB page (64 tag rows), at a physical address; CPUSH       //
//   pushes each dirty line first, CINV drops the dirty data.               //
//   Table searches go through the cache (wk_*): a read uses a hit, a write //
//   (U, M) updates a line holding the descriptor and goes to memory.       //
//   Snoops (sn_*): as the instruction cache's, through the tag copy; a     //
//   dirty line snooped loses its data (the platform's boundary).           //
// The store buffer (caches stage E). Every write for the bus -- a write    //
// sent, a push, a copyback write going alone -- waits in a four-entry      //
// buffer, in order, for the bus controller, which holds one at a time: a   //
// write is accepted as soon as there is room, not when the bus is free.    //
// The data cache takes a write's update as it enters, so a read that hits  //
// never waits for the buffer; a read that goes to the bus, a line read,    //
// and (on the bus16 top) a fetch from memory wait until it is empty; a     //
// read that misses meanwhile is looked up again once it is. The table      //
// walker waits for it (wr_pend). The prefetch window's snoop sees writes   //
// as the buffer takes them (sb_accept, sb_sla).                            //
// A write-back's bus error (caches stage R; MC68040UM 8.4.6). A write is   //
// accepted, and its instruction goes on, long before the bus answers it;   //
// a bus error then has no instruction to fault. The bus controller tells   //
// this unit (m_wberr), which records every write it hands over (hw_*),     //
// the whole write -- a crossing one's bytes are one write -- by its        //
// logical address, a push's by its physical one. The fault is held here    //
// (pw_*) with what its access error frame reports: the SSW; WB1S, WB1A     //
// and WB1D, the data in the byte lanes it was written on (Table 8-5), for  //
// the handler to complete; for a push, WB1S invalid and the line in PD0-   //
// PD3, so the push engine keeps its line until its last write is done;     //
// for MOVE16, TT 1, a line, and its four longwords. EA-fetch takes it at   //
// an instruction boundary (pae_hold) and lets it go (pw_ack). One is held  //
// at a time: another bus error meanwhile is dropped -- the writes behind   //
// the one that erred complete as they would have, where the 68040 would    //
// hand them to the handler in WB2/WB3 -- unless it is an exception         //
// frame's write, which makes the fault held a double fault (pw_exc).       //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_dmu
(
	input             clk,
	input             nreset,

	input             xlat,        // translation can refuse a data write (l1_wr_sync)
	input             tc_e,
	input             tc_p,

	// ---- the data cache's controls (stage C) ----
	input             dc_en,       // CACR DE
	input      [31:0] dtt0,        // the caching mode of an untranslated write
	input      [31:0] dtt1,
	// CINV/CPUSH (ap040_ea_fetch.v), held until cm_done; the top combines
	// this done with the instruction cache's
	input             cm_req,
	input             cm_dc,
	input             cm_push_in,  // CPUSH: dirty lines are pushed first
	input       [1:0] cm_scope,    // 01 line, 10 page, 11 all
	input      [31:0] cm_addr,     // physical
	output reg        cm_done,
	// another master wrote memory (caches stage F wires it)
	input             sn_req,
	input      [31:0] sn_addr,
	// the MMU's table walker, through the data cache (stage D) ...
	input             wk_req,
	input             wk_we,
	input      [31:0] wk_addr,
	input      [31:0] wk_wdat,
	output            wk_ack,
	output     [31:0] wk_data,
	output            wk_berr,
	// ...and its physical port, for what the cache does not answer
	output            walker_req,
	output            walker_we,
	output     [31:0] walker_addr,
	output     [31:0] walker_wdat,
	input             walker_ack,
	input      [31:0] walker_data,
	input             walker_berr,

	// ---- EA-fetch (port B) ----
	input      [31:0] c_addr,
	input             c_rd,
	input             c_wr,
	input       [1:0] c_size,
	input      [31:0] c_wdata,
	input             c_sup,
	input             c_fc_ovr,
	input       [2:0] c_fc_val,
	// what the access is (4.3.3, 7.4.5): allocates no line (an exception
	// frame, the vector fetch, MOVE16); MOVE16 (a write hit invalidates);
	// locked (TAS, CAS, CAS2: not cached)
	input             c_nalloc,
	input             c_m16,
	input             c_lock,
	input             c_wr_drop,
	output     [31:0] c_q,
	output            c_rvalid,
	output            c_wr_busy_w,
	output            c_rflt,
	output            c_wflt,
	output            c_flt_bus,
	output            c_flt_ma,
	output            c_idle,
	// A write accepted here has not reached memory yet: the table walker
	// must not read around it (ap040_pipe_mmu.v's walk_hold).
	output            wr_pend,

	// ---- ap040_pipe_mmu.v's data port ----
	output            d_req,
	output            d_write,
	output            d_acc,
	output     [31:0] d_addr,
	output            d_sup,
	input             d_pass,
	input             d_flt,
	input      [31:0] d_pa,
	input       [1:0] d_cm,        // with d_pass: the caching mode (00 WT, 01 CB, 1x inhibited)

	// ---- the bus controller's port B (physical) ----
	// writes, and reads untranslated
	output     [31:0] m_addr,
	output            m_rd,
	output            m_wr,
	output      [1:0] m_size,
	output     [31:0] m_wdata,
	output            m_sup,
	output            m_fc_ovr,
	output      [2:0] m_fc_val,
	// the logical address of the write on m_wr: the bus controller's
	// prefetch window is logical, and snoops writes by it
	output     [31:0] m_la,
	// translated reads, from registers
	output            m_rx,
	output     [31:0] m_rx_addr,
	output      [1:0] m_rx_size,
	output      [2:0] m_rx_fc,
	input      [31:0] m_q,
	input             m_rvalid,
	input             m_wr_busy_w,
	input             m_rflt,
	input             m_flt_bus,
	input             m_flt_ma,
	input             m_idle,
	// the write it was sending erred on the bus (caches stage R)
	input             m_wberr,

	// ---- a write-back's bus error, for EA-fetch (caches stage R) ----
	// A write has no instruction left when its bus error comes: the fault is
	// held here, with what the access error frame reports of it (MC68040UM
	// 8.4.6), until EA-fetch takes it at an instruction boundary.
	output            pw_pend,
	output     [15:0] pw_ssw,
	output     [31:0] pw_fa,
	output      [7:0] pw_wb1s,
	output    [127:0] pw_pd,       // WB1D/PD0 in 31:0, PD1, PD2, PD3
	output            pw_exc,      // an exception frame's write: a double fault
	input             pw_ack,      // its entry is done

	// ---- the store buffer (caches stage E) ----
	// Every write taken into it, for the prefetch window's snoop (the bus
	// controller's own w_accept comes too late now), with the logical
	// longword of the one taken last; and whether it is empty -- a fetch
	// from memory waits until it is.
	output            sb_accept,
	output reg [29:0] sb_sla,
	output            sb_empty
);

// Page geometry, and whether a transfer crosses out of its page
// (ap040_pipe_membus.v's).
wire [12:0] pg_mask   = tc_p ? 13'h1FFF : 13'h0FFF;
wire [31:0] pg_mask32 = {19'd0, pg_mask};
function crosses;
	input [31:0] a;
	input  [1:0] sz;
	input [12:0] mask;
	begin
		crosses = ({1'b0, a[12:0] & mask} + ((sz == `AP040_SZ_L) ? 14'd4 : (sz == `AP040_SZ_W) ? 14'd2 : 14'd1)) >
		          ({1'b0, mask} + 14'd1);
	end
endfunction
function [1:0] last_of;   // the transfer's last byte index
	input [1:0] sz;
	begin
		last_of = (sz == `AP040_SZ_L) ? 2'd3 : (sz == `AP040_SZ_W) ? 2'd1 : 2'd0;
	end
endfunction
// An access's privilege: its function code's, as the MMU has always taken it.
function sup_of;
	input       ovr;
	input [2:0] fcv;
	input       sup;
	begin
		sup_of = ovr ? fcv[2] : sup;
	end
endfunction
// MOVES to an alternate space: physical, untranslated.
function alt_of;
	input       ovr;
	input [2:0] fcv;
	begin
		alt_of = ovr && ((fcv[1:0] == 2'b00) || (fcv[1:0] == 2'b11));
	end
endfunction
// The function code on the bus: MOVES to program space is a data reference.
function [2:0] fc_bus;
	input [2:0] fcv;
	begin
		fc_bus = (fcv[1:0] == 2'b10) ? {fcv[2], 2'b01} : fcv;
	end
endfunction
wire c_alt = alt_of(c_fc_ovr, c_fc_val);
// An access error's SSW fields for a data access (ap040_ea_fetch.v's aer_*):
// MOVES to a space with no data meaning reports TT 10, to program space its
// function code with bit 0 set in place of bit 1; SIZE B 01, W 10, L 00.
function [1:0] tt_of;
	input       ovr;
	input [2:0] fcv;
	begin
		tt_of = (ovr && ((fcv == 3'd0) || (fcv == 3'd3) || (fcv == 3'd4) || (fcv == 3'd7))) ? 2'b10 : 2'b00;
	end
endfunction
function [2:0] tm_of;
	input       ovr;
	input [2:0] fcv;
	input       sup;
	begin
		tm_of = !ovr ? {sup, 2'b01} : (fcv[1:0] == 2'b10) ? {fcv[2], 2'b01} : fcv;
	end
endfunction
function [1:0] szf_of;
	input [1:0] sz;
	begin
		szf_of = (sz == `AP040_SZ_B) ? 2'b01 : (sz == `AP040_SZ_W) ? 2'b10 : 2'b00;
	end
endfunction
// WB1D: the data memory-aligned, in the byte lanes it was written on
// (Table 8-5) -- the operand's first byte in lane A1-A0, wrapping.
function [31:0] wb1d_of;
	input [31:0] d;
	input  [1:0] sz;
	input  [1:0] off;
	reg   [31:0] top;
	reg   [63:0] two;
	begin
		top = (sz == `AP040_SZ_B) ? {d[7:0], 24'd0} : (sz == `AP040_SZ_W) ? {d[15:0], 16'd0} : d;
		two = {top, top} >> {off, 3'b000};
		wb1d_of = two[31:0];
	end
endfunction
function ttr_match;   // ap040_pipe_mmu.v's
	input [31:0] ttr;
	input [31:0] la;
	input        s;
	begin
		ttr_match = ttr[15] &&
		            (&((la[31:24] ~^ ttr[31:24]) | ttr[23:16])) &&
		            (ttr[14] || (ttr[13] == s));
	end
endfunction
// A read's answer from a longword: right-aligned by its size, as the bus
// controller answers.
function [31:0] ext_of;
	input [31:0] lw;
	input  [1:0] off;
	input  [1:0] sz;
	begin
		case (sz)
		`AP040_SZ_B: ext_of = {24'd0, lw[(5'd24 - {off, 3'b000}) +: 8]};
		`AP040_SZ_W: ext_of = {16'd0, lw[(5'd16 - {off, 3'b000}) +: 16]};   // off 0-2
		default:     ext_of = lw;
		endcase
	end
endfunction
// Whether an access of sz bytes at off spans two longwords.
function spans;
	input [1:0] off;
	input [1:0] sz;
	begin
		spans = ({1'b0, off} + ((sz == `AP040_SZ_L) ? 3'd4 : (sz == `AP040_SZ_W) ? 3'd2 : 3'd1)) > 3'd4;
	end
endfunction

//---------------------------------------------------------------------------
// the data cache's state (stage C; its logic is below)
//---------------------------------------------------------------------------

// The line being read (a miss's), and its buffer.
reg        fl_act;
reg [27:0] fl_line;             // PA31-PA4
reg  [2:0] fl_fc;
reg  [1:0] fl_st;               // the longword asked for: read first
reg  [2:0] fl_iss;              // reads sent
reg  [1:0] fl_arr;              // longwords arrived
reg  [3:0] fl_have;
reg [31:0] fb [0:3];
reg  [1:0] fl_way;
reg [87:0] fl_tags;             // the set's tag row, written back with the new tag
reg        fl_out;              // a read of the line is with the bus controller
reg        fl_nov;              // snooped while being read: not to be made valid
reg        fl_su;               // read for a copyback write (su)
// The last write's update (see su_md): a line holding it takes its bytes,
// or is pushed and goes; or, copyback, it is the write itself.
localparam [1:0] SU_WT = 2'd0, SU_CI = 2'd1, SU_CB = 2'd2;
reg        su_v;
reg        su_cmp;              // its set's rows are on the outputs
reg        su_part;             // its second longword (it spans two)
reg [31:0] su_pa, su_la;
reg  [1:0] su_size;
reg [31:0] su_data;
reg  [1:0] su_md;
reg        su_na;               // CB: allocates nothing
reg        su_tt, su_s;         // untranslated: a data TTR may inhibit it; its privilege
reg        su_post;             // CB, missing, allocating nothing: to the bus alone
reg        su_wsb;              // CB, missing with writes buffered: looked up again when they are out
// Valid bits, dirty bits (a longword each: {way3 lw3-0, ..., way0 lw3-0}),
// and the replacement counter, in flops.
reg  [3:0] vld [0:63];
reg [15:0] dty [0:63];
reg  [1:0] rep;
// CINV/CPUSH
localparam [1:0] CM_IDLE = 2'd0, CM_RD = 2'd1, CM_CMP = 2'd2, CM_PUSH = 2'd3;
reg  [1:0] cm_st;
reg  [5:0] cm_set, cm_rset;
reg        cm_seen, cm_line, cm_allw, cm_push;
reg [87:0] cm_row;              // the row compared, for the pushes' addresses
reg  [3:0] cm_pend;             // its ways still to push
// the push engine: a line's longwords read, then its dirty ones written
reg        pb_act, pb_rd, pb_rv;
reg  [1:0] pb_i, pb_ri;
reg  [2:0] pb_n;
reg [27:0] pb_line;
reg  [1:0] pb_way;
reg  [3:0] pb_dty;
reg [31:0] pb_d [0:3];
reg        pb_vic;              // a replaced line's: written only once the new line is in
reg        pb_rs;               // ...whose new line's read erred: being put back
reg  [3:0] pb_rm;               // its longwords the fill overwrote, still to put back
reg        pb_sn;               // snooped while out of the cache: not put back
// the table walker's access
localparam [2:0] WK_IDLE = 3'd0, WK_CMP = 3'd1, WK_EXT = 3'd2, WK_ACK = 3'd3;
reg  [2:0] wk_st;
reg        wk_seen;             // answered: it drops its request before the next
reg [31:0] wk_q;
reg        wk_eber;
// snoops: captured (1), their row on the copy's outputs (2)
reg        sn_v1, sn_v2, sn_junk2;
reg [27:0] sn_l1, sn_l2;

//---------------------------------------------------------------------------
// the write slot (translation can refuse writes)
//---------------------------------------------------------------------------

localparam [2:0] WS_FREE = 3'd0,   // empty
                 WS_CHK1 = 3'd1,   // translating (crossing: checking page 1)
                 WS_CHK2 = 3'd2,   // crossing: checking page 2
                 WS_T1   = 3'd3,   // crossing: translating page 1 for real (sets M)
                 WS_T2   = 3'd4,   // crossing: translating page 2 for real (sets M)
                 WS_POST = 3'd5;   // translated: to the bus controller
reg  [2:0] ws;
reg [31:0] w_la, w_data;
reg  [1:0] w_size;
reg        w_sup, w_ovr;
reg  [2:0] w_fcv;
reg [31:0] w_pa1, w_pa2;        // physical: the address (w_x: page 1's base), page 2's base
reg  [1:0] w_bi, w_last;
reg  [1:0] w_cm;                // its caching mode, from its translation
reg        w_m16, w_lk;         // MOVE16; locked
reg        w_na;                // allocates nothing
reg        w_block;             // a tentative write was refused: take none until the CPU lets go
reg        w_receipt;           // accepted while the CPU was not looking
reg        w_acc;               // accepted this cycle: its translation passed last cycle
wire       w_slot  = (ws != WS_FREE);
// Crosses a page: from the latched address, not the CPU's (which settles late
// in its cycle). TC cannot change while the slot holds a write: a MOVEC to it
// waits for the memory side to be idle.
wire       w_x     = tc_e && !alt_of(w_ovr, w_fcv) && crosses(w_la, w_size, pg_mask);
wire [31:0] w_pg2  = (w_la | pg_mask32) + 32'd1;
wire       w_wants = (ws == WS_CHK1) || (ws == WS_CHK2) || (ws == WS_T1) || (ws == WS_T2);
// Taken into the slot as it arrives, and translated from there.
wire       w_take  = xlat && c_wr && !w_slot && !w_block && !w_receipt;
// The store buffer (caches stage E): every write for the bus waits here,
// in order, for the bus controller, which takes one at a time. A write is
// taken in as it is sent -- the data cache takes its update then, so a
// read that hits never waits for the buffer -- and a read that goes to
// the bus, a line read, or a fetch from memory waits until it is empty.
// An entry is the bus write, and the whole write a crossing write's byte
// belongs to (what a bus error reports).
reg  [31:0] sb_pa  [0:3];       // the bus address (a crossing write: its byte's)
reg  [31:0] sb_la  [0:3];       // the write's logical address
reg  [31:0] sb_d   [0:3];       // its data, right-aligned
reg   [1:0] sb_sz  [0:3];       // its size
reg   [1:0] sb_bi  [0:3];       // a crossing write: which byte
reg   [2:0] sb_fcv [0:3];
reg   [3:0] sb_x, sb_sup, sb_ovr, sb_push, sb_m16, sb_lk, sb_exc;   // a bit an entry
reg   [1:0] sb_hd, sb_tl;
reg   [2:0] sb_n;
assign      sb_empty = (sb_n == 3'd0);
wire        sb_room  = (sb_n != 3'd4);
// Untranslated, straight to the buffer -- unless the slot still holds a
// write from before translation was turned off, or (DE) the data cache has
// not yet taken the last write's update.
wire       w_thru  = !xlat && c_wr && !w_slot && !su_v && !pb_act && sb_room;

//---------------------------------------------------------------------------
// the read
//---------------------------------------------------------------------------

localparam [2:0] RS_IDLE = 3'd0,   // nothing outstanding here
                 RS_XL   = 3'd1,   // translating (crossing: page 1)
                 RS_XL2  = 3'd2,   // crossing: translating page 2
                 RS_RDY  = 3'd3,   // translated, waiting for the slot to empty
                 RS_BYTE = 3'd4,   // crossing: its bytes, one read each
                 RS_LK   = 3'd5,   // DE: waiting for the cache to look it up
                 RS_CMP  = 3'd6,   // DE: its set's rows are on the arrays' outputs
                 RS_FW   = 3'd7;   // DE: waiting for its line's first longword
reg  [2:0] rs;
reg [31:0] r_la;
reg  [1:0] r_size;
reg        r_sup, r_ovr;
reg  [2:0] r_fcv;
reg [31:0] r_pa1, r_pa2;
reg  [1:0] r_bi, r_last;
reg [23:0] r_acc;               // a crossing read's bytes so far, right-aligned
reg        r_wait;              // a byte's read is with the bus controller
reg        r_dc;                // DE was set when it came: the cache's
reg        r_na, r_lk;          // allocates nothing; locked
reg        r_ci;                // not cached: a line holding it is invalidated, then the bus
reg        r_wsb;               // missed with writes buffered: looked up again when they are out
// The answer: the bus controller's, or this unit's own (a fault, or a
// crossing read's assembled bytes).
reg        own;
reg        own_v, own_flt;
reg [31:0] own_q;
// The last fault's kind, from whichever unit raised it.
reg        fsrc;                // 1: this unit's registers, 0: the bus controller's
reg        flt_bus_r, flt_ma_r;

// Untranslated, straight to the bus controller; otherwise latched here --
// always with the data cache on, and while a line is being read.
wire        r_thru  = !xlat && !dc_en && c_rd && (rs == RS_IDLE) && !w_slot && !fl_act && sb_empty;
wire        r_new   = c_rd && (rs == RS_IDLE) && !r_thru;
// As w_x; an alternate-space read never consults it (it goes from the
// latch straight to RS_RDY, whole).
wire        r_x     = tc_e && crosses(r_la, r_size, pg_mask);
wire        r_wants = (rs == RS_XL) || (rs == RS_XL2);
wire [31:0] r_xaddr = (rs == RS_XL2) ? ((r_la | pg_mask32) + 32'd1) : r_la;

//---------------------------------------------------------------------------
// the MMU's data port: registers only
//---------------------------------------------------------------------------

reg  d_hw, d_hr;                // left raised for the slot / the read, unanswered
reg  d_gap;                     // the cycle after a fault: nothing raised
wire w_on = w_wants && !d_hr && !d_gap;
wire r_on = r_wants && !d_hw && !d_gap && !w_on;

assign d_req   = w_on || r_on;
assign d_write = w_on;
assign d_acc   = w_on && w_x && ((ws == WS_CHK1) || (ws == WS_CHK2));
assign d_addr  = w_on ? (((ws == WS_CHK2) || (ws == WS_T2)) ? w_pg2 : w_la) : r_xaddr;
assign d_sup   = w_on ? w_sup : r_sup;

wire w_pass = w_on && d_pass;
wire w_flt  = w_on && d_flt;
wire r_pass = r_on && d_pass;
wire r_flt  = r_on && d_flt;

// Accepted -- wr_busy low -- the cycle after its translation (crossing: its
// second page's) passes.
// An alternate-space write needs no translation: accepted as it is taken.
wire w_passed = (w_take && c_alt) || (w_pass && (((ws == WS_CHK1) && !w_x) || (ws == WS_T2)));

//---------------------------------------------------------------------------
// the bus controller's port B
//---------------------------------------------------------------------------

// The slot's write, or its bytes.
wire  [1:0] w_bk      = w_last - w_bi;              // byte k of the operand, from the top
wire  [7:0] w_byte    = w_data[{w_bk, 3'b000} +: 8];
wire [31:0] w_byte_la = w_la + {30'd0, w_bi};
wire        w_byte_p2 = (w_byte_la & ~pg_mask32) != (w_la & ~pg_mask32);
wire [31:0] w_byte_pa = (w_byte_p2 ? w_pa2 : w_pa1) | (w_byte_la & pg_mask32);
// Copyback (DE, CM 01): an aligned write that is neither MOVE16, locked
// nor to an alternate space goes to the cache instead (s_cb); a write
// spanning longwords, or crossing a page, is written through.
wire        w_cb      = dc_en && (w_cm == 2'b01) && !w_x && !spans(w_la[1:0], w_size) &&
                        !w_m16 && !w_lk && !alt_of(w_ovr, w_fcv);
wire        s_cb      = (ws == WS_POST) && w_cb && !su_v && !pb_act;
wire        s_post    = (ws == WS_POST) && !w_cb && sb_room && !su_v && !pb_act;

// A translated read goes only when nothing older is here: the slot empty.
// (A write the CPU presents in the same cycle is not older than a read
// already waiting here -- port B asks one thing at a time, and a read
// outstanding is answered before its instruction's own writes -- and it
// goes into the slot, or to the bus controller's write input: never onto
// rx's.)
wire        r_go      = !w_slot && !fl_act && !pb_act && !su_v;
wire        r_gob     = r_go && sb_empty;                          // ...and to the bus: nothing buffered
wire        r_send_x  = r_pass && (rs == RS_XL) && !r_x && r_gob && !r_dc;   // straight on its translation
wire        r_send_q  = (rs == RS_RDY) && r_gob;                   // translated earlier
wire [31:0] r_byte_la = r_la + {30'd0, r_bi};
wire        r_byte_p2 = (r_byte_la & ~pg_mask32) != (r_la & ~pg_mask32);
wire [31:0] r_byte_pa = (r_byte_p2 ? r_pa2 : r_pa1) | (r_byte_la & pg_mask32);
wire        r_issue   = (rs == RS_BYTE) && !r_wait && r_gob;

// Writes and untranslated reads: a push (the data cache's dirty line,
// ahead of everything), a copyback write that allocates nothing, the
// slot's, or the CPU's own.
wire        su_pw   = su_post && !pb_act && sb_room;
wire [31:0] pb_wa   = {pb_line, pb_wi, 2'b00};
// Into the buffer: at most one a cycle, the sources exclude each other.
wire        sb_in   = pb_wr || su_pw || s_post || w_thru;
wire [31:0] e_pa    = pb_wr ? pb_wa : su_pw ? su_pa : s_post ? (w_x ? w_byte_pa : w_pa1) : c_addr;
wire [31:0] e_la    = pb_wr ? pb_wa : su_pw ? su_la : s_post ? w_la : c_addr;
wire [31:0] e_bla   = s_post && w_x ? w_byte_la : e_la;                    // the byte's, for the snoop
wire [31:0] e_d     = pb_wr ? pb_d[pb_wi] : su_pw ? su_data : s_post ? w_data : c_wdata;
wire  [1:0] e_sz    = pb_wr ? `AP040_SZ_L : su_pw ? su_size : s_post ? w_size : c_size;
wire        e_sup   = pb_wr ? 1'b1 : su_pw ? su_s : s_post ? w_sup : c_sup;
wire        e_ovr   = !pb_wr && !su_pw && (s_post ? w_ovr : c_fc_ovr);
wire  [2:0] e_fcv   = s_post ? w_fcv : c_fc_val;
wire        e_x     = s_post && w_x;
wire        e_m16   = !pb_wr && !su_pw && (s_post ? w_m16 : c_m16);
wire        e_lk    = !pb_wr && !su_pw && (s_post ? w_lk : c_lock);
// allocating nothing and not MOVE16: an exception frame's
wire        e_exc   = !pb_wr && (su_pw ? su_na : s_post ? (w_na && !w_m16) : (c_nalloc && !c_m16));
assign sb_accept = sb_in;
// Out of it, to the bus controller: the oldest write; a read straight
// through only while it is empty.
wire  [1:0] sb_bk   = last_of(sb_sz[sb_hd]) - sb_bi[sb_hd];                  // a crossing write's byte, from the top
wire  [7:0] sb_byte = sb_d[sb_hd][{sb_bk, 3'b000} +: 8];
assign m_wr     = !sb_empty;
assign m_rd     = r_thru;
assign m_addr   = sb_empty ? c_addr : sb_pa[sb_hd];
assign m_la     = sb_x[sb_hd] ? (sb_la[sb_hd] + {30'd0, sb_bi[sb_hd]}) : sb_la[sb_hd];
assign m_size   = sb_empty ? c_size : sb_x[sb_hd] ? `AP040_SZ_B : sb_sz[sb_hd];
assign m_wdata  = sb_x[sb_hd] ? {24'd0, sb_byte} : sb_d[sb_hd];
assign m_sup    = sb_empty ? c_sup : sb_sup[sb_hd];
assign m_fc_ovr = sb_empty ? c_fc_ovr : sb_ovr[sb_hd];
assign m_fc_val = sb_empty ? c_fc_val : sb_fcv[sb_hd];
// Translated reads, and the data cache's line reads -- the first in the
// cycle the miss is seen (nothing else of this unit's is out then).
assign m_rx      = r_send_x || r_send_q || r_issue || fl_start || fl_rq;
assign m_rx_addr = fl_start ? {fl_spa, 2'b00} : fl_rq ? {fl_line, fl_st + fl_iss[1:0], 2'b00} :
                   r_send_x ? d_pa : r_send_q ? r_pa1 : r_byte_pa;
assign m_rx_size = (fl_start || fl_rq) ? `AP040_SZ_L : r_issue ? `AP040_SZ_B : r_size;
assign m_rx_fc   = fl_start ? (su_cmp ? {su_s, 2'b01} : r_fcb) : fl_rq ? fl_fc :
                   r_ovr ? fc_bus(r_fcv) : {r_sup, 2'b01};

//---------------------------------------------------------------------------
// the data cache (stages C, D)
//---------------------------------------------------------------------------

// The arrays' port A: lookups (a read's, an update's, the walker's), an
// update's or the walker's write, and the push engine's reads. Maintenance
// holds the tag port B; a line being read, the data port B.
wire        cm_go   = cm_req && !cm_seen && (cm_st == CM_IDLE) && !su_v && !fl_act && !pb_act &&
                      (rs != RS_CMP) && !su_cmp && (wk_st == WK_IDLE);
wire        c_free  = !fl_act && !pb_act && (cm_st == CM_IDLE) && !cm_go;

// ---- a write's update (su) ----
// SU_WT -- update a line holding it, its dirty bits kept (Table 4-4); SU_CI
// -- merge it into a line holding it, push the line's dirty longwords, then
// invalidate the line (4.3.2); SU_CB -- the write itself, not sent: update a
// line holding it and mark its longword dirty; missing, read the line and
// then do so, or send it alone if it allocates nothing (4.3.1.2, 4.3.3).
wire  [7:0] su_be8  = ((su_size == `AP040_SZ_L) ? 8'hF0 : (su_size == `AP040_SZ_W) ? 8'hC0 : 8'h80) >> su_pa[1:0];
wire [63:0] su_d64  = ((su_size == `AP040_SZ_L) ? {su_data, 32'd0} :
                       (su_size == `AP040_SZ_W) ? {su_data[15:0], 48'd0} : {su_data[7:0], 56'd0}) >> {su_pa[1:0], 3'b000};
wire [29:0] su_lw   = su_pa[31:2] + {29'd0, su_part};
wire  [3:0] su_be   = su_part ? su_be8[3:0] : su_be8[7:4];
wire [31:0] su_wd   = su_part ? su_d64[31:0] : su_d64[63:32];
wire        su_two  = (su_be8[3:0] != 4'd0);
wire        su_ta   = ttr_match(dtt0, su_pa, su_s);
wire        su_tb   = ttr_match(dtt1, su_pa, su_s);
// straight through (untranslated): a data TTR may still say not cached
wire  [1:0] su_mode = ((su_md == SU_WT) && su_tt && (su_ta ? dtt0[6] : su_tb ? dtt1[6] : 1'b0)) ? SU_CI : su_md;
wire        su_rd   = su_v && !su_cmp && !su_post && c_free && (!su_wsb || sb_empty);   // its set is read now
// The walker's lookup: its set read now.
wire        wk_new  = dc_en && wk_req && !wk_seen && (wk_st == WK_IDLE);
wire        wk_lk   = wk_new && c_free && !su_v;
wire        wk_cmp  = (wk_st == WK_CMP);
// A read's lookup: in the cycle its translation passes, or later from
// RS_LK; never while a write's update is waiting (the write is older).
wire        rd_lk   = c_free && !su_v && !wk_lk && !wk_cmp && r_go &&
                      (((rs == RS_XL) && r_pass && !r_x && r_dc && !spans(r_la[1:0], r_size)) ||
                       ((rs == RS_LK) && (!r_wsb || sb_empty)));
wire  [7:0] a_addr  = pb_rd ? {pb_line[5:0], pb_i} : (wk_lk || wk_cmp) ? wk_addr[9:2] :
                      su_v ? su_lw[7:0] : r_la[9:2];
wire [127:0] a_data;
wire  [87:0] a_tags, b_tags, s_tags;
// the compare: an update's, the walker's, or a read's (RS_CMP)
wire [21:0] ck_tag  = su_cmp ? su_lw[29:8] : wk_cmp ? wk_addr[31:10] : r_pa1[31:10];
wire  [5:0] ck_set  = su_cmp ? su_lw[7:2]  : wk_cmp ? wk_addr[9:4]   : r_pa1[9:4];
wire  [1:0] ck_lwi  = su_cmp ? su_lw[1:0]  : wk_addr[3:2];
wire  [3:0] ck_vrow = vld[ck_set];
wire [15:0] ck_drow = dty[ck_set];
wire  [3:0] ck_hw;
genvar gw;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : hw
	assign ck_hw[gw] = ck_vrow[gw] && (a_tags[gw*22 +: 22] == ck_tag);
end
endgenerate
wire        ck_hit  = |ck_hw;
wire  [1:0] ck_way  = ck_hw[0] ? 2'd0 : ck_hw[1] ? 2'd1 : ck_hw[2] ? 2'd2 : 2'd3;
wire  [3:0] ck_wdty = ck_drow[ck_way*4 +: 4];            // the hit line's dirty longwords
wire [31:0] ck_hlw  = a_data[ck_way*32 +: 32];
wire        rd_cmp  = (rs == RS_CMP) && !su_cmp && !wk_cmp;
wire        dc_hit  = rd_cmp && ck_hit;
wire        su_hit  = su_cmp && ck_hit;
wire        wk_hit  = wk_cmp && ck_hit;
wire        wk_whit = wk_hit && wk_we;
// the hit line is pushed if dirty -- a not-cached access's (the write merged
// into it first); the hit line is invalidated
wire        su_push = su_hit && (su_mode == SU_CI) && (ck_wdty != 4'd0);
wire        rd_push = dc_hit && r_ci && (ck_wdty != 4'd0);
wire        rd_inv  = dc_hit && r_ci;
wire        su_inv1 = su_hit && (su_mode == SU_CI);
wire        su_cbw  = su_hit && (su_mode == SU_CB);
// A CB write that misses reads its line, or goes to the bus alone.
wire        su_miss = su_cmp && !ck_hit && (su_mode == SU_CB);
// A miss reads the line (a read that allocates, or a CB write); the way it
// replaces: the first invalid, else the counter's -- a dirty one pushed.
wire        rd_fill = rd_cmp && !ck_hit && !r_ci && !r_na;
wire        su_fill = su_miss && !su_na;
// ...its first read in the miss cycle: only with nothing buffered, which
// could be older than the line (a miss meanwhile is looked up again, once
// the buffer is empty)
wire        fl_start = (rd_fill || su_fill) && sb_empty;
wire [29:0] fl_spa  = su_cmp ? su_lw : r_pa1[31:2];      // the longword asked for
wire  [1:0] fl_vict = !ck_vrow[0] ? 2'd0 : !ck_vrow[1] ? 2'd1 : !ck_vrow[2] ? 2'd2 :
                      !ck_vrow[3] ? 2'd3 : rep;
wire  [3:0] fl_vdty = ck_drow[fl_vict*4 +: 4];
wire        fl_vpush = fl_start && ck_vrow[fl_vict] && (fl_vdty != 4'd0);
wire  [1:0] fl_aidx  = fl_st + fl_arr;                    // the longword arriving next
wire        fl_ack   = fl_out && m_rvalid && !m_rflt;
wire        fl_flt   = fl_out && m_rvalid && m_rflt;
wire        fl_rq    = fl_act && !fl_out && (fl_iss != 3'd4);
wire        fl_done  = fl_ack && (fl_arr == 2'd3);
wire  [2:0] r_fcb    = r_ovr ? fc_bus(r_fcv) : {r_sup, 2'b01};
// A read in the line being read, its longword in (or arriving), and no
// write's update waiting that could change it: handed on.
wire        lk_infl  = (rs == RS_LK) && fl_act && !fl_nov && !su_v && !r_ci && (r_pa1[31:4] == fl_line);
wire        lk_fb    = lk_infl && fl_have[r_pa1[3:2]];
wire        lk_fa    = lk_infl && fl_ack && (fl_aidx == r_pa1[3:2]);
// ...or its longword's read errs: the read's fault; another longword's error
// sends it back to be looked up again
wire        lk_ff    = (rs == RS_LK) && fl_act && !r_ci && (r_pa1[31:4] == fl_line) && fl_flt &&
                       (fl_aidx == r_pa1[3:2]);
// the tag row the fill writes back
wire [87:0] fl_wtags;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : twr
	assign fl_wtags[gw*22 +: 22] = (fl_way == gw) ? fl_line[27:6] : fl_tags[gw*22 +: 22];
end
endgenerate

// ---- CINV/CPUSH ----
// A row a set: read through the tag port B, compared, its matching lines
// invalidated; CPUSH pushes each dirty one first, one at a time.
wire  [3:0] cm_hw;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : cmw
	assign cm_hw[gw] = cm_allw ? 1'b1 :
	                   cm_line ? (b_tags[gw*22 +: 22] == cm_addr[31:10])
	                           : (b_tags[gw*22 + 2 +: 20] == cm_addr[31:12]);
end
endgenerate
wire [15:0] cm_drow = dty[cm_rset];
wire  [3:0] cm_ddty = {|cm_drow[15:12], |cm_drow[11:8], |cm_drow[7:4], |cm_drow[3:0]};
wire        cm_clr  = (cm_st == CM_CMP);
wire  [3:0] cm_mv   = vld[cm_rset] & cm_hw;                  // the row's matching lines
wire  [3:0] cm_topush = cm_push ? (cm_mv & cm_ddty) : 4'd0;  // ...pushed before they go
wire  [1:0] cm_pway = cm_pend[0] ? 2'd0 : cm_pend[1] ? 2'd1 : cm_pend[2] ? 2'd2 : 2'd3;
wire        cm_push1 = (cm_st == CM_PUSH) && (cm_pend != 4'd0) && !pb_act;

// ---- the push engine ----
// A dirty line's four longwords are read through port A -- the one a
// replacing fill asks for first, so each is out before the fill's longword
// lands on it -- then its dirty longwords are written to the bus controller
// ahead of any CPU write. Until they are, no read goes to the bus and no new
// line is read: memory has the line first. The engine stays busy until the
// last of them is done on the bus, not only handed over: a push's bus error
// reports the whole line (pw_pd), from these registers.
// A replaced line (pb_vic) is written only once the new line is in, as the
// 68040 orders it (4.6.1). If the new line's read errs, the replaced line
// goes back to its place instead (4.6.2): its tag is still in the row (the
// fill writes the tag last), the longwords the fill overwrote are written
// back through port B, the fill's own -- the same way, the same set -- and
// its valid and dirty bits are set again; unless a snoop hit it while it
// was out, which it would have lost in the cache.
wire        pb_start = fl_vpush || su_push || rd_push || cm_push1;
wire  [1:0] pb_wi    = pb_dty[0] ? 2'd0 : pb_dty[1] ? 2'd1 : pb_dty[2] ? 2'd2 : 2'd3;
wire        pb_wr    = pb_act && !pb_rd && !pb_rv && (pb_dty != 4'd0) && sb_room && !pb_rs &&
                       !(pb_vic && fl_act);
wire        pb_rsw   = pb_rs && !pb_rd && !pb_rv && (pb_rm != 4'd0);    // a longword put back now
wire  [1:0] pb_rsi   = pb_rm[0] ? 2'd0 : pb_rm[1] ? 2'd1 : pb_rm[2] ? 2'd2 : 2'd3;
wire        pb_rdone = pb_rs && !pb_rd && !pb_rv && (pb_rm == 4'd0);    // back: valid and dirty again
// the line and way it starts with
wire [27:0] pb_sline = fl_vpush ? {a_tags[fl_vict*22 +: 22], ck_set} :
                       cm_push1 ? {cm_row[cm_pway*22 +: 22], cm_rset} :
                       su_push  ? su_lw[29:2] : r_pa1[31:4];
wire  [1:0] pb_sway  = fl_vpush ? fl_vict : cm_push1 ? cm_pway : ck_way;
wire  [3:0] pb_sdty  = fl_vpush ? fl_vdty : cm_push1 ? cm_drow[cm_pway*4 +: 4] : ck_wdty;
wire  [1:0] pb_si    = fl_vpush ? fl_spa[1:0] : 2'd0;

// ---- the table walker, through the cache ----
// Its reads use a hit; its writes (U, M) update a line holding the
// descriptor, whose longword is then clean -- memory takes the write too
// (4.3.3). DE clear, it goes straight to its port.
assign walker_req  = dc_en ? (wk_st == WK_EXT) : wk_req;
assign walker_we   = wk_we;
assign walker_addr = wk_addr;
assign walker_wdat = wk_wdat;
assign wk_ack      = dc_en ? (wk_st == WK_ACK) : walker_ack;
assign wk_data     = dc_en ? wk_q : walker_data;
assign wk_berr     = dc_en ? ((wk_st == WK_ACK) && wk_eber) : walker_berr;

// ---- snoops ----
wire  [3:0] sn_hw;
generate
for (gw = 0; gw < 4; gw = gw + 1) begin : snw
	assign sn_hw[gw] = (s_tags[gw*22 +: 22] == sn_l2[27:6]);
end
endgenerate
wire  [3:0] sn_clr  = sn_junk2 ? 4'hF : sn_hw;
wire        sn_fill = sn_v2 && fl_act && (sn_l2 == fl_line);

// ---- the valid and dirty bits ----
// A snoop's set and one other change a cycle, merged: a hit's (an
// invalidation, a CB update, the walker's write), a fill's start or end, a
// maintenance row's, a pushed line's.
function [15:0] wm16;   // the dirty bits of the ways in m
	input [3:0] m;
	begin
		wm16 = {{4{m[3]}}, {4{m[2]}}, {4{m[1]}}, {4{m[0]}}};
	end
endfunction
wire        cm_all  = cm_go && cm_dc && (cm_scope == 2'b11) && !cm_push_in;
wire [15:0] o_lwb   = 16'd1 << {ck_way, ck_lwi};             // a hit's longword
wire        o_v     = rd_inv || su_inv1 || su_cbw || wk_whit || fl_start || fl_done || cm_clr || cm_push1 ||
                      pb_rdone;
wire  [5:0] o_set   = pb_rdone ? pb_line[5:0] : fl_done ? fl_line[5:0] : (cm_clr || cm_push1) ? cm_rset : ck_set;
wire  [3:0] o_vand  = (rd_inv || su_inv1) ? ~(4'd1 << ck_way) :
                      fl_start ? ~(4'd1 << fl_vict) :
                      cm_clr   ? ~(cm_mv & ~cm_topush) :
                      cm_push1 ? ~(4'd1 << cm_pway) : 4'hF;
wire  [3:0] o_vor   = pb_rdone ? (4'd1 << pb_way) : (fl_done && !fl_nov && !sn_fill) ? (4'd1 << fl_way) : 4'd0;
wire [15:0] o_dand  = (rd_inv || su_inv1) ? ~wm16(4'd1 << ck_way) :
                      fl_start ? ~wm16(4'd1 << fl_vict) :
                      fl_done  ? ~wm16(4'd1 << fl_way) :
                      wk_whit  ? ~o_lwb :
                      cm_clr   ? ~wm16(cm_mv & ~cm_topush) :
                      cm_push1 ? ~wm16(4'd1 << cm_pway) : 16'hFFFF;
wire [15:0] o_dor   = pb_rdone ? ({12'd0, pb_dty} << {pb_way, 2'b00}) : su_cbw ? o_lwb : 16'd0;
wire [15:0] sn_dclr = wm16(sn_clr);

ap040_pipe_dcache_arr u_arr
(
	.clk     (clk),
	.a_addr  (a_addr), .a_data (a_data), .a_way (ck_way),
	.a_wdata (wk_whit ? wk_wdat : su_wd), .a_be (wk_whit ? 4'hF : su_be), .a_we (su_hit || wk_whit),
	// port B: the line being read lands here -- or, its read having erred,
	// the line it was to replace is put back, into the same way and set
	.b_addr  ({fl_line[5:0], pb_rsw ? pb_rsi : fl_aidx}), .b_data (), .b_way (fl_way),
	.b_wdata (pb_rsw ? pb_d[pb_rsi] : m_q), .b_we (fl_ack || pb_rsw),
	.ta_set  (a_addr[7:2]), .ta_tags (a_tags),
	.tb_set  ((cm_st != CM_IDLE) ? cm_set : fl_line[5:0]), .tb_tags (b_tags),
	.tb_wtags (fl_wtags), .tb_we (fl_done),
	.s_set   (sn_l1[5:0]), .s_tags (s_tags)
);

integer i;
always @(posedge clk) begin
	cm_done <= 1'b0;
	if (!nreset) begin
		fl_act <= 1'b0; fl_line <= 28'd0; fl_fc <= 3'd0; fl_st <= 2'd0; fl_iss <= 3'd0; fl_arr <= 2'd0;
		fl_have <= 4'd0; fl_way <= 2'd0; fl_tags <= 88'd0; fl_out <= 1'b0; fl_nov <= 1'b0; fl_su <= 1'b0;
		su_v <= 1'b0; su_cmp <= 1'b0; su_part <= 1'b0; su_pa <= 32'd0; su_la <= 32'd0; su_size <= `AP040_SZ_L;
		su_data <= 32'd0; su_md <= SU_WT; su_na <= 1'b0; su_tt <= 1'b0; su_s <= 1'b1; su_post <= 1'b0;
		su_wsb <= 1'b0;
		rep <= 2'd0;
		cm_st <= CM_IDLE; cm_set <= 6'd0; cm_rset <= 6'd0; cm_seen <= 1'b0; cm_pend <= 4'd0;
		cm_row <= 88'd0; cm_line <= 1'b0; cm_allw <= 1'b0; cm_push <= 1'b0;
		pb_act <= 1'b0; pb_rd <= 1'b0; pb_rv <= 1'b0; pb_i <= 2'd0; pb_ri <= 2'd0; pb_n <= 3'd0;
		pb_line <= 28'd0; pb_way <= 2'd0; pb_dty <= 4'd0;
		pb_vic <= 1'b0; pb_rs <= 1'b0; pb_rm <= 4'd0; pb_sn <= 1'b0;
		wk_st <= WK_IDLE; wk_seen <= 1'b0; wk_q <= 32'd0; wk_eber <= 1'b0;
		sn_v1 <= 1'b0; sn_v2 <= 1'b0; sn_junk2 <= 1'b0; sn_l1 <= 28'd0; sn_l2 <= 28'd0;
		for (i = 0; i < 64; i = i + 1) begin vld[i] <= 4'd0; dty[i] <= 16'd0; end
	end else begin
		// ---- a write: its update ----
		if (dc_en && (s_post || w_thru)) begin
			su_v    <= 1'b1;
			su_cmp  <= 1'b0;
			su_part <= 1'b0;
			su_pa   <= e_pa;
			su_la   <= e_la;
			su_size <= s_post && w_x ? `AP040_SZ_B : e_sz;
			su_data <= s_post && w_x ? {24'd0, w_byte} : e_d;
			// sent: not cached if its translation says so, or a MOVE16,
			// locked, or MOVES to an alternate space; straight through, a
			// data TTR may still say so (su_tt)
			su_md   <= (s_post ? (w_cm[1] || w_m16 || w_lk || alt_of(w_ovr, w_fcv))
			                   : (c_m16 || c_lock || c_alt)) ? SU_CI : SU_WT;
			su_na   <= 1'b0;
			su_tt   <= !s_post;
			su_s    <= s_post ? w_sup : sup_of(c_fc_ovr, c_fc_val, c_sup);
		end else if (s_cb) begin
			// copyback: the write goes to the cache instead of the bus
			su_v    <= 1'b1;
			su_cmp  <= 1'b0;
			su_part <= 1'b0;
			su_pa   <= w_pa1;
			su_la   <= w_la;
			su_size <= w_size;
			su_data <= w_data;
			su_md   <= SU_CB;
			su_na   <= w_na;
			su_tt   <= 1'b0;
			su_s    <= w_sup;
		end else if (su_rd) su_cmp <= 1'b1;
		else if (su_cmp) begin
			su_cmp <= 1'b0;
			su_wsb <= su_fill && !sb_empty;
			if (su_fill) ;                                   // looked up again once its line is in
			else if (su_miss) su_post <= 1'b1;               // allocates nothing: to the bus
			else if (!su_part && su_two) su_part <= 1'b1;
			else begin su_v <= 1'b0; su_part <= 1'b0; end
		end else if (su_post && su_pw) begin
			su_post <= 1'b0;
			su_v    <= 1'b0;
		end
		// the counter: every read looked up, every write sent or put in a
		// copyback line, and once more after naming a way to replace
		rep <= rep + {1'b0, rd_lk} + {1'b0, dc_en && (s_post || w_thru)} +
		       {1'b0, su_rd && (su_md == SU_CB)} + {1'b0, fl_start && (&ck_vrow)};

		// ---- a miss: its line ----
		if (fl_start) begin
			fl_act  <= 1'b1;
			fl_line <= fl_spa[29:2];
			fl_fc   <= su_cmp ? {su_s, 2'b01} : r_fcb;
			fl_st   <= fl_spa[1:0];
			fl_iss  <= 3'd1;                     // its first read goes now
			fl_arr  <= 2'd0;
			fl_have <= 4'd0;
			fl_way  <= fl_vict;
			fl_tags <= a_tags;
			fl_nov  <= 1'b0;
			fl_su   <= su_cmp;
			fl_out  <= 1'b1;
		end else if (fl_rq) begin fl_out <= 1'b1; fl_iss <= fl_iss + 3'd1; end
		if (fl_ack) begin
			fl_out           <= 1'b0;
			fl_arr           <= fl_arr + 2'd1;
			fl_have[fl_aidx] <= 1'b1;
			fb[fl_aidx]      <= m_q;
			if (fl_arr == 2'd3) fl_act <= 1'b0;       // valid (below), its tag written now
		end
		if (fl_flt) begin
			// the line is abandoned; its way stays invalid -- or holds again
			// the dirty line it was to replace (the push engine puts it
			// back). A copyback write it was read for goes to the bus alone:
			// looked up again it would miss, and read the line again, for as
			// long as the memory errs.
			fl_out <= 1'b0;
			fl_act <= 1'b0;
			if (fl_su) su_post <= 1'b1;
		end

		// ---- the push engine ----
		if (pb_start) begin
			pb_act  <= 1'b1;
			pb_rd   <= 1'b1;
			pb_i    <= pb_si;
			pb_n    <= 3'd0;
			pb_line <= pb_sline;
			pb_way  <= pb_sway;
			pb_dty  <= pb_sdty;
			pb_vic  <= fl_vpush;
			pb_rs   <= 1'b0;
			pb_sn   <= sn_v2 && (sn_l2 == pb_sline);
		end else begin
			if (pb_rd) begin
				pb_i <= pb_i + 2'd1;
				pb_n <= pb_n + 3'd1;
				if (pb_n == 3'd3) pb_rd <= 1'b0;
			end
			if (pb_wr) pb_dty[pb_wi] <= 1'b0;
			else if (pb_act && !pb_rd && !pb_rv && (pb_dty == 4'd0) && sb_empty && !m_wr_busy_w) pb_act <= 1'b0;
			if (sn_v2 && pb_act && (sn_l2 == pb_line)) pb_sn <= 1'b1;
			// the new line's read erred: the replaced line goes back, or, if a
			// snoop took it meanwhile, is dropped
			if (fl_flt && pb_act && pb_vic && !pb_rs) begin
				if (pb_sn) pb_act <= 1'b0;
				else begin pb_rs <= 1'b1; pb_rm <= fl_have; end
			end
			if (pb_rsw) pb_rm[pb_rsi] <= 1'b0;
			if (pb_rdone) begin pb_rs <= 1'b0; pb_act <= 1'b0; end
		end
		pb_rv <= pb_rd && !pb_start;
		pb_ri <= pb_i;
		if (pb_rv) pb_d[pb_ri] <= a_data[pb_way*32 +: 32];

		// ---- the walker ----
		if (!wk_req) wk_seen <= 1'b0;
		case (wk_st)
		WK_IDLE: if (wk_lk) wk_st <= WK_CMP;
		WK_CMP: begin
			// a read that hits is answered; a write that hits updates the line
			// (above: a_we), then goes to memory too; a miss goes to the port
			wk_q <= ck_hlw; wk_eber <= 1'b0;
			wk_st <= (ck_hit && !wk_we) ? WK_ACK : WK_EXT;
		end
		WK_EXT: if (walker_ack || walker_berr) begin
			wk_q <= walker_data; wk_eber <= walker_berr; wk_st <= WK_ACK;
		end
		default: begin wk_st <= WK_IDLE; wk_seen <= 1'b1; end   // WK_ACK
		endcase

		// ---- CINV/CPUSH ----
		if (!cm_req) cm_seen <= 1'b0;
		cm_rset <= cm_set;
		if (cm_go) begin
			if (!cm_dc || cm_all) begin cm_done <= 1'b1; cm_seen <= 1'b1; end   // all ways, no push: cm_all
			else begin
				cm_st   <= CM_RD;
				cm_line <= (cm_scope == 2'b01);
				cm_allw <= (cm_scope == 2'b11);
				cm_push <= cm_push_in;
				cm_set  <= (cm_scope == 2'b01) ? cm_addr[9:4] : 6'd0;
			end
		end
		case (cm_st)
		CM_RD:  cm_st <= CM_CMP;                        // cm_set's row read now
		CM_CMP: begin
			cm_row  <= b_tags;
			cm_pend <= cm_topush;
			cm_st   <= (cm_topush != 4'd0) ? CM_PUSH : CM_PUSH;
		end
		CM_PUSH: if (cm_push1) cm_pend[cm_pway] <= 1'b0;
		         else if ((cm_pend == 4'd0) && !pb_act) begin
			if (cm_line || (cm_rset == 6'd63)) begin cm_st <= CM_IDLE; cm_done <= 1'b1; cm_seen <= 1'b1; end
			else begin cm_set <= cm_set + 6'd1; cm_st <= CM_RD; end
		end
		default: ;
		endcase

		// ---- snoops ----
		sn_v1    <= sn_req;
		sn_l1    <= sn_addr[31:4];
		sn_v2    <= sn_v1;
		sn_l2    <= sn_l1;
		sn_junk2 <= sn_v1 && fl_done && (fl_line[5:0] == sn_l1[5:0]);
		if (sn_fill) fl_nov <= 1'b1;

		// ---- the valid and dirty bits ----
		if (cm_all) for (i = 0; i < 64; i = i + 1) begin vld[i] <= 4'd0; dty[i] <= 16'd0; end
		else if (o_v && sn_v2 && (o_set == sn_l2[5:0])) begin
			vld[o_set] <= ((vld[o_set] & o_vand) | o_vor) & ~sn_clr;
			dty[o_set] <= ((dty[o_set] & o_dand) | o_dor) & ~sn_dclr;
		end else begin
			if (o_v) begin
				vld[o_set] <= (vld[o_set] & o_vand) | o_vor;
				dty[o_set] <= (dty[o_set] & o_dand) | o_dor;
			end
			if (sn_v2) begin
				vld[sn_l2[5:0]] <= vld[sn_l2[5:0]] & ~sn_clr;
				dty[sn_l2[5:0]] <= dty[sn_l2[5:0]] & ~sn_dclr;
			end
		end
	end
end

//---------------------------------------------------------------------------
// a write-back's bus error (caches stage R)
//---------------------------------------------------------------------------

// The bus controller holds one write at a time, and takes the next only
// once the last is done: a bus error it reports is the write handed over
// last -- the store buffer's head -- recorded here as it goes: the whole
// write, a crossing one's bytes included, as the frame reports it
// (8.4.6.4: the first byte's address).
wire        m_take  = m_wr && !m_wr_busy_w;
reg         hw_push, hw_m16, hw_lk, hw_exc;
reg  [31:0] hw_la, hw_data;
reg   [1:0] hw_size, hw_tt;
reg   [2:0] hw_tm;
// The fault taken, until EA-fetch's entry for it is done. PD0-PD3 hold a
// push's line; a MOVE16's four longwords (captured as they go, while no
// fault is held, and after its own for the rest of its line); a write's
// memory-aligned data in WB1D. Any further bus error is dropped -- one on
// an exception frame's write marks the fault a double fault.
// The rest of a MOVE16's line: its fault comes before the bus controller
// takes another write, so the MOVE16 writes after it, up to its fourth
// longword, are that MOVE16's (pf_m16c).
reg         pf_v, pf_m16, pf_m16c, pf_exc;
reg  [15:0] pf_ssw;
reg  [31:0] pf_fa;
reg   [7:0] pf_wb1s;
reg  [31:0] pf_pd [0:3];
wire  [1:0] hw_szf  = hw_m16 ? 2'b11 : szf_of(hw_size);
wire  [1:0] hw_ttf  = hw_m16 ? 2'b01 : hw_tt;
wire        pf_new  = m_wberr && (!pf_v || pw_ack);
always @(posedge clk) begin
	if (!nreset) begin
		hw_push <= 1'b0; hw_m16 <= 1'b0; hw_lk <= 1'b0; hw_exc <= 1'b0;
		hw_la <= 32'd0; hw_data <= 32'd0; hw_size <= `AP040_SZ_L; hw_tt <= 2'b00; hw_tm <= 3'd0;
		pf_v <= 1'b0; pf_m16 <= 1'b0; pf_m16c <= 1'b0; pf_exc <= 1'b0;
		pf_ssw <= 16'd0; pf_fa <= 32'd0; pf_wb1s <= 8'd0;
		pf_pd[0] <= 32'd0; pf_pd[1] <= 32'd0; pf_pd[2] <= 32'd0; pf_pd[3] <= 32'd0;
	end else begin
		if (m_take) begin
			hw_push <= sb_push[sb_hd];
			hw_la   <= sb_la[sb_hd];
			hw_data <= sb_d[sb_hd];
			hw_size <= sb_sz[sb_hd];
			// a push: TT 0, TM 0
			hw_tt   <= sb_push[sb_hd] ? 2'b00 : tt_of(sb_ovr[sb_hd], sb_fcv[sb_hd]);
			hw_tm   <= sb_push[sb_hd] ? 3'd0 : tm_of(sb_ovr[sb_hd], sb_fcv[sb_hd], sb_sup[sb_hd]);
			hw_m16  <= sb_m16[sb_hd];
			hw_lk   <= sb_lk[sb_hd];
			hw_exc  <= sb_exc[sb_hd];
		end
		// a MOVE16's longwords, as they go
		if (m_take && sb_m16[sb_hd] && (!pf_v || pf_m16c)) begin
			pf_pd[sb_la[sb_hd][3:2]] <= sb_d[sb_hd];
			if (sb_la[sb_hd][3:2] == 2'd3) pf_m16c <= 1'b0;
		end
		if (pf_new) begin
			pf_v    <= 1'b1;
			pf_m16  <= hw_m16;
			pf_m16c <= hw_m16 && (hw_la[3:2] != 2'd3);
			pf_exc  <= hw_exc;
			pf_fa   <= hw_la;
			// ATC clear: a physical bus error; RW clear: a write. A push is TT 0,
			// TM 0, a longword (its record says so); MOVE16's write TT 1, a line.
			pf_ssw  <= {6'b000000, hw_lk, 1'b0, 1'b0, hw_szf, hw_ttf, hw_tm};
			pf_wb1s <= hw_push ? 8'h00 : {1'b1, hw_szf, hw_ttf, hw_tm};
			if (hw_push) begin
				pf_pd[0] <= pb_d[0]; pf_pd[1] <= pb_d[1]; pf_pd[2] <= pb_d[2]; pf_pd[3] <= pb_d[3];
			end else if (!hw_m16) begin
				pf_pd[0] <= wb1d_of(hw_data, hw_size, hw_la[1:0]);
				pf_pd[1] <= 32'd0; pf_pd[2] <= 32'd0; pf_pd[3] <= 32'd0;
			end
		end else begin
			if (pw_ack) pf_v <= 1'b0;
			if (m_wberr && hw_exc) pf_exc <= 1'b1;
		end
	end
end
// the buffer: in at the tail, out at the head
always @(posedge clk) begin
	if (!nreset) begin
		sb_hd <= 2'd0; sb_tl <= 2'd0; sb_n <= 3'd0; sb_sla <= 30'd0;
		sb_x <= 4'd0; sb_sup <= 4'd0; sb_ovr <= 4'd0; sb_push <= 4'd0; sb_m16 <= 4'd0; sb_lk <= 4'd0; sb_exc <= 4'd0;
	end else begin
		if (sb_in) begin
			sb_pa[sb_tl]   <= e_pa;
			sb_la[sb_tl]   <= e_la;
			sb_d[sb_tl]    <= e_d;
			sb_sz[sb_tl]   <= e_sz;
			sb_bi[sb_tl]   <= w_bi;
			sb_fcv[sb_tl]  <= e_fcv;
			sb_x[sb_tl]    <= e_x;
			sb_sup[sb_tl]  <= e_sup;
			sb_ovr[sb_tl]  <= e_ovr;
			sb_push[sb_tl] <= pb_wr;
			sb_m16[sb_tl]  <= e_m16;
			sb_lk[sb_tl]   <= e_lk;
			sb_exc[sb_tl]  <= e_exc;
			sb_tl          <= sb_tl + 2'd1;
			sb_sla         <= e_bla[31:2];
		end
		if (m_take) sb_hd <= sb_hd + 2'd1;
		sb_n <= sb_n + {2'd0, sb_in} - {2'd0, m_take};
	end
end

assign pw_pend = pf_v;
assign pw_ssw  = pf_ssw;
assign pw_fa   = pf_fa;
assign pw_wb1s = pf_wb1s;
assign pw_pd   = {pf_pd[3], pf_pd[2], pf_pd[1], pf_pd[0]};
assign pw_exc  = pf_exc;

//---------------------------------------------------------------------------
// toward the CPU: registers, and the bus controller's registered answers
//---------------------------------------------------------------------------

assign c_q         = own ? own_q   : m_q;
assign c_rvalid    = own ? own_v   : m_rvalid;
assign c_rflt      = own ? own_flt : m_rflt;
assign c_wflt      = w_block;
assign c_flt_bus   = fsrc ? flt_bus_r : m_flt_bus;
assign c_flt_ma    = fsrc ? flt_ma_r  : m_flt_ma;
// As the bus controller's: busy from the cycle a tentative write first
// appears until it is accepted, free that one cycle; untranslated, the bus
// controller's own.
assign c_wr_busy_w = xlat ? (w_block || (!w_receipt && !w_acc))
                          : (!sb_room || w_slot || su_v || pb_act);
assign c_idle      = m_idle && sb_empty && !w_slot && (rs == RS_IDLE) && !fl_act && !su_v && !pb_act;
// A write accepted has not reached memory: a push's too (its line is in
// neither the cache nor memory until it lands).
assign wr_pend     = (ws == WS_POST) || !sb_empty || m_wr_busy_w || pb_act;

always @(posedge clk) begin
	if (!nreset) begin
		ws <= WS_FREE; w_la <= 32'd0; w_data <= 32'd0; w_size <= `AP040_SZ_L;
		w_cm <= 2'b00; w_m16 <= 1'b0; w_lk <= 1'b0; w_na <= 1'b0;
		w_sup <= 1'b1; w_ovr <= 1'b0; w_fcv <= 3'd0;
		w_pa1 <= 32'd0; w_pa2 <= 32'd0; w_bi <= 2'd0; w_last <= 2'd0;
		w_block <= 1'b0; w_receipt <= 1'b0; w_acc <= 1'b0;
		rs <= RS_IDLE; r_la <= 32'd0; r_size <= `AP040_SZ_L; r_sup <= 1'b1; r_ovr <= 1'b0;
		r_dc <= 1'b0; r_na <= 1'b0; r_lk <= 1'b0; r_ci <= 1'b0; r_wsb <= 1'b0;
		r_fcv <= 3'd0; r_pa1 <= 32'd0; r_pa2 <= 32'd0; r_bi <= 2'd0; r_last <= 2'd0;
		r_acc <= 24'd0; r_wait <= 1'b0;
		own <= 1'b0; own_v <= 1'b0; own_flt <= 1'b0; own_q <= 32'd0;
		fsrc <= 1'b0; flt_bus_r <= 1'b0; flt_ma_r <= 1'b0;
		d_hw <= 1'b0; d_hr <= 1'b0; d_gap <= 1'b0;
	end else begin
		// ---- the MMU's data port ----
		d_hw  <= w_on && !d_pass && !d_flt;
		d_hr  <= r_on && !d_pass && !d_flt;
		d_gap <= d_flt;

		// ---- the write slot ----
		if (c_wr_drop) w_block <= 1'b0;
		w_acc <= w_passed;
		if (w_acc)      w_receipt <= !c_wr;
		else if (c_wr)  w_receipt <= 1'b0;   // consumed: wr_busy was low for it
		if (w_take) begin
			w_la   <= c_addr;
			w_data <= c_wdata;
			w_size <= c_size;
			w_sup  <= sup_of(c_fc_ovr, c_fc_val, c_sup);
			w_ovr  <= c_fc_ovr;
			w_fcv  <= c_fc_val;
			w_bi   <= 2'd0;
			w_last <= last_of(c_size);
			w_m16  <= c_m16;
			w_lk   <= c_lock;
			w_na   <= c_nalloc;
			if (c_alt) begin w_pa1 <= c_addr; w_cm <= 2'b10; ws <= WS_POST; end
			else ws <= WS_CHK1;
		end
		case (ws)
		WS_CHK1: if (w_flt) begin
			// refused: nothing written; MA clear, it is the first page
			ws <= WS_FREE; w_block <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b0; flt_ma_r <= 1'b0;
		end else if (w_pass) begin
			if (w_x) ws <= WS_CHK2;
			else begin w_pa1 <= d_pa; w_cm <= d_cm; ws <= WS_POST; end
		end
		WS_CHK2: if (w_flt) begin
			ws <= WS_FREE; w_block <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b0; flt_ma_r <= 1'b1;
		end else if (w_pass) ws <= WS_T1;
		// Both pages checked: now translated for real. A fault here can only
		// be a table search's bus error (the checks passed), and the write,
		// not yet accepted, is refused like any other.
		WS_T1: if (w_flt) begin
			ws <= WS_FREE; w_block <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b0; flt_ma_r <= 1'b0;
		end else if (w_pass) begin w_pa1 <= d_pa & ~pg_mask32; ws <= WS_T2; end
		WS_T2: if (w_flt) begin
			ws <= WS_FREE; w_block <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b0; flt_ma_r <= 1'b1;
		end else if (w_pass) begin w_pa2 <= d_pa & ~pg_mask32; ws <= WS_POST; end
		WS_POST: if (s_post) begin
			if (w_x && (w_bi != w_last)) w_bi <= w_bi + 2'd1;
			else ws <= WS_FREE;
		end else if (s_cb) ws <= WS_FREE;
		default: ;
		endcase

		// ---- the read ----
		if (r_thru) begin
			own  <= 1'b0;            // the bus controller answers; it drops its valid now
			fsrc <= 1'b0;
		end
		if (r_new) begin
			r_la    <= c_addr;
			r_size  <= c_size;
			r_sup   <= sup_of(c_fc_ovr, c_fc_val, c_sup);
			r_ovr   <= c_fc_ovr;
			r_fcv   <= c_fc_val;
			r_bi    <= 2'd0;
			r_last  <= last_of(c_size);
			r_acc   <= 24'd0;
			r_dc    <= dc_en;
			r_na    <= c_nalloc;
			r_lk    <= c_lock;
			r_ci    <= c_alt;
			r_wsb   <= 1'b0;
			own     <= 1'b1;         // answered here until it is sent
			own_v   <= 1'b0;
			own_flt <= 1'b0;
			// an alternate space needs no translation: it waits only for the
			// slot -- and with DE, for its line to be invalidated
			if (c_alt) begin r_pa1 <= c_addr; rs <= dc_en ? RS_LK : RS_RDY; end
			else rs <= RS_XL;
		end
		case (rs)
		RS_XL: if (r_flt) begin
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b0; flt_ma_r <= 1'b0;
		end else if (r_send_x) begin
			rs <= RS_IDLE; own <= 1'b0; fsrc <= 1'b0;
		end else if (r_pass) begin
			if (r_x) begin r_pa1 <= d_pa & ~pg_mask32; rs <= RS_XL2; end
			else if (r_dc) begin
				// its set read now, if the cache is free; a read spanning two
				// longwords goes to the bus
				r_pa1 <= d_pa;
				r_ci  <= r_lk || d_cm[1];
				rs    <= spans(r_la[1:0], r_size) ? RS_RDY : rd_lk ? RS_CMP : RS_LK;
			end
			else     begin r_pa1 <= d_pa;              rs <= RS_RDY; end
		end
		RS_XL2: if (r_flt) begin
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b0; flt_ma_r <= 1'b1;
		end else if (r_pass) begin
			r_pa2 <= d_pa & ~pg_mask32; rs <= RS_BYTE;
		end
		RS_RDY: if (r_send_q) begin
			rs <= RS_IDLE; own <= 1'b0; fsrc <= 1'b0;
		end
		RS_LK: if (lk_fb || lk_fa) begin
			// in the line being read, and in: handed on
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b0;
			own_q <= ext_of(lk_fb ? fb[r_pa1[3:2]] : m_q, r_pa1[1:0], r_size);
		end else if (lk_ff) begin
			// its longword's read erred: the read's bus error
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b1; flt_ma_r <= 1'b0;
		end else if (rd_lk) rs <= RS_CMP;
		RS_CMP: if (dc_hit && !r_ci) begin
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b0;
			own_q <= ext_of(ck_hlw, r_pa1[1:0], r_size);
		end else if (dc_hit || r_ci || r_na) rs <= RS_RDY;   // (a hit not cached: invalidated below)
		else if (!sb_empty) begin rs <= RS_LK; r_wsb <= 1'b1; end   // writes buffered: again later
		else rs <= RS_FW;                                     // the line is read (below)
		RS_FW: if (fl_ack && (fl_arr == 2'd0)) begin
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b0;
			own_q <= ext_of(m_q, r_pa1[1:0], r_size);
		end else if (fl_flt && (fl_arr == 2'd0)) begin
			rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b1;
			fsrc <= 1'b1; flt_bus_r <= 1'b1; flt_ma_r <= 1'b0;
		end
		RS_BYTE: begin
			if (r_issue) r_wait <= 1'b1;
			else if (r_wait && m_rvalid) begin
				r_wait <= 1'b0;
				if (m_rflt) begin
					// a byte's bus error: the read faults, MA past the boundary
					rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b1;
					fsrc <= 1'b1; flt_bus_r <= 1'b1; flt_ma_r <= r_byte_p2;
				end else if (r_bi != r_last) begin
					r_acc <= {r_acc[15:0], m_q[7:0]};
					r_bi  <= r_bi + 2'd1;
				end else begin
					rs <= RS_IDLE; own_v <= 1'b1; own_flt <= 1'b0;
					own_q <= {r_acc, m_q[7:0]};
				end
			end
		end
		default: ;
		endcase
	end
end

`ifdef VERILATOR
always @(posedge clk)
	if (nreset) begin
		// A put-back writes the way and set the fill wrote: a replaced line's
		// are the fill's by construction (pb_sway, pb_sline).
		if (pb_rsw && ((pb_way != fl_way) || (pb_line[5:0] != fl_line[5:0])))
			$error("ap040_pipe_dmu: a line put back where its fill was not");
		// The bus controller's port B takes one address a cycle.
		if (m_rd && m_wr)
			$error("ap040_pipe_dmu: a read and a write on the bus controller's port B in one cycle");
		// One read at a time: the CPU asks again only once it has its answer,
		// and by then this unit has sent or answered the last one.
		if (c_rd && (rs != RS_IDLE))
			$error("ap040_pipe_dmu: a read asked for while one is outstanding here (%0d)", rs);
	end
`endif

endmodule
