//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 81)                 //
//                                                                          //
// ap040_pipe_membus.v - the two CPU memory ports onto one bus transaction  //
//                                                                          //
// CPU side: port B is ap040_pipe_l1.v's protocol exactly -- byte           //
// addresses, a request and a return, one outstanding read, writes posted   //
// behind wr_busy. Port A's side, the prefetch window, is ap040_pipe_imu.v  //
// (2026-09-25, caches stage B), which asks this unit for its reads (f_*).  //
//                                                                          //
// Bus side: rtl/ap040/ap040_core.v's own external port, verbatim, so that  //
// ap040_bus16_adapter.v and the TG68K-shaped wrapper around it can be      //
// attached without reshaping anything: mem_req is held until mem_ack,      //
// mem_ack is a single-cycle pulse with mem_rdata valid, and the requester  //
// drops mem_req in the ack cycle. One transaction at a time; this module   //
// leaves one idle cycle between them rather than holding mem_req high      //
// across a new address, which is what that adapter's "one stable bus       //
// request at a time" contract asks for.                                    //
//                                                                          //
// Ordering: a posted write goes out BEFORE any read that is waiting. The   //
// L1 array answers a read from its write buffer when the two collide;      //
// draining first gets the same answer from memory and needs no forwarding  //
// path here. Reads then beat fetches, because a fetch can always be        //
// re-issued and a load cannot: the IMU's read goes out in the cycle it     //
// asks (f_req) only when the bus is free for it (f_free) -- nothing on it, //
// no write or read waiting -- and is answered with f_ack (mem_rdata) or    //
// f_flt. Every write taken is told to the IMU (w_accept), with the         //
// logical longword the program wrote (w_sla), for its window's snoop.      //
//                                                                          //
// Transaction sizes: data accesses carry the CPU's own size and address,  //
// at whatever alignment, and the adapter below splits them (milestone 86). //
// A fetch is an aligned longword.                                          //
//                                                                          //
// Translation (2026-09-25, caches stage A). On the 16-bit top the MMU is   //
// no longer below this unit. Port B's accesses arrive translated, from the //
// DMU (ap040_pipe_dmu.v); a translated read may take the bus in the cycle  //
// it arrives (rx_*). The IMU's reads arrive translated too.                //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_membus
(
	input             clk,
	input             nreset,

	// ---- the fetch port: ap040_pipe_imu.v's reads (see the header) ----
	input             f_req,        // a fetch, now, if the bus is free for it
	input      [31:0] f_addr,
	input             f_sup,
	output            f_free,       // the bus is free for a fetch this cycle
	output            f_ack,        // the fetch on the bus is answered (mem_rdata)...
	output            f_flt,        // ...or faulted
	output            f_flt_bus,    // ...with a physical bus error
	output            w_accept,     // a write is taken this cycle, for the IMU's snoop...
	output reg [29:0] w_sla,        // ...and the logical longword of the one taken last

	// ---- CPU side: ap040_pipe_l1.v's port-B protocol ----
	input      [31:0] address_b,
	// The address the program wrote to, for the IMU's window snoop: the
	// window is logical, and with the DMU above this unit address_b is
	// physical (2026-09-25; tied to address_b where nothing translates).
	input      [31:0] la_b,
	input      [31:0] data_b,      // right-aligned by size_b
	input             wren_b,
	input       [1:0] size_b,
	input             rd_b,
	output            wr_busy,
	// wr_busy for a write being presented: the same, with wren_b taken as
	// set. It is all the CPU ever asks -- it looks at wr_busy only while it
	// presents a write -- and it does not depend on wren_b, which the CPU
	// forms from its stalls, which EX forms from wr_busy (a read-modify-
	// write's store waits on it): the loop Quartus found at the bus16 top,
	// 22 nodes and 17.7 ns, and Verilator's UNOPTFLAT on stall_self.
	output            wr_busy_w,
	output reg [31:0] q_b,
	output reg        rvalid_b,

	// The supervisor bit, for the function code alone: the data access's
	// OWN, which an exception entry forces supervisor whatever the status
	// register still says (milestone 92). (The fetch's comes with f_req.)
	input             sup_b,
	// MOVES: the function code of this port-B access, in place of sup_b's.
	input             fc_ovr,
	input       [2:0] fc_ovr_val,

	// ---- bus side: ap040_core.v's external memory port ----
	output reg        mem_req,
	output reg        mem_write,
	output reg        mem_instr,
	output reg  [1:0] mem_size,
	output reg [31:0] mem_addr,
	output reg [31:0] mem_wdata,
	output reg  [2:0] mem_fc,
	input             mem_ack,
	input      [31:0] mem_rdata,

	// ---- access faults (2026-09-24) ----
	// The transaction on the bus faulted: the MMU refused it (a one-cycle
	// pulse; the request is consumed) or the bus answered with a physical
	// bus error (mem_flt_bus, which the adapter has already aborted on).
	input             mem_flt,
	input             mem_flt_bus,
	// The MMU forwarded the transaction on the bus: its translation passed.
	input             mem_pass,
	// Translation can refuse a data write (TC.E, or a data TTR enabled):
	// a write is then TENTATIVE until it passes, and wr_busy holds the
	// storing instruction until it does, so a refusal is precise.
	input             wr_sync,
	output reg        rflt_b,       // with rvalid_b: the read faulted
	output            wflt,         // the tentative write being presented faulted (held until withdrawn)
	output            idle,         // nothing on the bus, posted or waiting
	// A posted write -- accepted, its instruction gone -- erred on the bus: a
	// pulse, for the DMU, which knows what the write was (caches stage R).
	output            wr_berr,
	// The requester has withdrawn its write, in a cycle it ran: the refusal
	// has been seen. Not !wren_b, which the CPU gates with its ce -- a strobe
	// left up through a disabled cycle is a write accepted twice (milestone
	// 92) -- so a refusal cleared when wren_b dropped was gone after the
	// first disabled cycle, before the CPU had looked, and the write went
	// out again: re-refused by the MMU, and lost for good after a one-shot
	// bus error.
	input             wr_drop,
	// Transfers that cross a page (2026-09-24). The MMU translates one
	// address per transaction, so with translation on a transfer spanning
	// two pages goes out a byte at a time, each through its own page, and a
	// read's bytes are assembled; a fault past the boundary reports MA
	// (flt_ma). A write is first probed on both pages -- the MMU's PTEST
	// sideband as an access check (pb_*) -- and written only if both allow
	// it: ap040_core.v's check_write, so a refused one has written nothing
	// (an RMW restarted over a half-written operand would compute from it).
	input             xlat_e,       // TC.E
	input             xlat_p,       // TC.P: 8K pages
	output reg        pb_req,
	output reg [31:0] pb_addr,
	output     [2:0]  pb_fc,
	input             pb_done,
	input      [31:0] pb_mmusr,
	output reg        flt_ma,
	output reg        flt_bus,      // ...a physical bus error, not the MMU

	// ---- a translated read (the DMU's): on the bus in its own cycle when
	// the bus is free and no write is waiting, else held as rd_b holds one
	input             rx,
	input      [31:0] rx_addr,
	input       [1:0] rx_size,
	input       [2:0] rx_fc
);

localparam [1:0] WHO_A = 2'd0, WHO_BR = 2'd1, WHO_BW = 2'd2;

reg        busy;        // a transaction is on the bus
reg  [1:0] who;         // whose it is

reg        w_tent;      // the pending write has not passed translation yet
reg        w_block;     // a tentative write faulted: take no write until the strobe drops
// A tentative write passed while its requester was not presenting it (review
// 16). The pass frees wr_busy for one clock, and the CPU, under ce, may not
// be looking that clock: its instruction then went on waiting, the write
// drained, and the strobe it still held was taken as a new write -- one
// MOVE.L posted twice, which RAM forgives and a device register does not.
// The receipt keeps the acceptance until the CPU next presents its write,
// which is that same write: nothing can take the storing instruction away
// while it waits for this (it is the oldest there is, EX and WB having
// drained past it), so the strobe it shows next is the one that passed.
reg        w_receipt;
// A crossing transfer's bytes: which is next, and the last one's index.
reg        b_x, w_x;
reg  [1:0] b_bi, b_last, w_bi, w_last;
reg [23:0] b_acc;       // the bytes read so far, right-aligned
reg  [1:0] w_pb;        // the write's probe: page 1, page 2, or done (0)
reg        b_pend;      // a data read is wanted and has not been returned
reg [31:0] b_addr;
reg  [1:0] b_size;
reg        w_pend;      // a write has been accepted and has not been sent
reg [31:0] w_addr, w_data;
reg  [1:0] w_size;
// Captured WITH each request, not read when the transaction is finally
// sent: a posted write can sit here across the very commit that changes the
// privilege, and would then go out under the wrong one.
reg  [2:0] b_fc, w_fc;

// The requester must hold its write until this drops -- as with the array's
// one-entry buffer, the write is accepted the cycle wr_busy is low.
// A tentative write is not accepted when it is latched but when it passes:
// busy in the cycle it first appears, busy until then, and free for the one
// cycle the MMU forwards it -- the storing instruction's acceptance.
// The page geometry, and whether a transfer of n bytes at an address
// crosses out of its page.
wire [12:0] pg_mask   = xlat_p ? 13'h1FFF : 13'h0FFF;
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
wire        pb_fail    = !pb_mmusr[0] || pb_mmusr[11] || pb_mmusr[2] || (!w_fc[2] && pb_mmusr[7]);
wire        pb_pass2   = w_pend && w_x && (w_pb == 2'd2) && pb_req && pb_done && !pb_fail;
assign      pb_fc      = w_fc;
wire  [1:0] w_bk       = w_last - w_bi;              // byte k of the operand, from the top
wire  [7:0] w_byte     = w_data[{w_bk, 3'b000} +: 8];
// A crossing write is accepted when its second probe passes: from there on
// it is posted like any other, and its bytes cannot be refused.
wire   w_pass_now = w_pend && w_tent && ((busy && (who == WHO_BW) && mem_pass) || pb_pass2);
assign wr_busy = w_block || (!w_receipt && (w_pend ? !w_pass_now : (wr_sync && wren_b)));
assign wr_busy_w = w_block || (!w_receipt && (w_pend ? !w_pass_now : wr_sync));
// The refusal is a level, not the one-clock pulse the MMU gives: the CPU
// runs under ce and may not be looking that clock.
assign wflt    = w_block;
assign idle    = !busy && !w_pend && !b_pend;
assign wr_berr = busy && mem_flt && (who == WHO_BW) && !w_tent;

// Port B is sized (milestone 86), so address, size and data go out as they
// arrive: ap040_bus16_adapter.v splits whatever alignment they have.
function [2:0] fc_of;
	input is_instr;
	input priv;
	begin
		fc_of = priv ? (is_instr ? `AP040_FC_SUPER_PROG : `AP040_FC_SUPER_DATA)
		             : (is_instr ? `AP040_FC_USER_PROG  : `AP040_FC_USER_DATA);
	end
endfunction
// MOVES's function code on the bus: to program space ($2, $6) it is a data
// reference in $1 or $5 (MC68040UM 3.2, Table 3-2); the others as given.
function [2:0] fc_moves;
	input [2:0] fcv;
	begin
		fc_moves = (fcv[1:0] == 2'b10) ? {fcv[2], 2'b01} : fcv;
	end
endfunction

// Writes are taken as they are presented, when nothing is pending here.
assign w_accept  = wren_b && !w_pend && !w_block && !w_receipt;
// The fetch port: the bus is free for a fetch when nothing is on it and no
// write or read is waiting -- the chain below reaches the fetch.
assign f_free    = !busy && !w_pend && !b_pend && !rx;
assign f_ack     = busy && mem_ack && (who == WHO_A);
assign f_flt     = busy && mem_flt && (who == WHO_A);
assign f_flt_bus = mem_flt_bus;

always @(posedge clk) begin
	if (!nreset) begin
		busy <= 1'b0; who <= WHO_A;
		w_tent <= 1'b0; w_block <= 1'b0;
		w_receipt <= 1'b0;
		rflt_b <= 1'b0; flt_bus <= 1'b0; flt_ma <= 1'b0;
		b_x <= 1'b0; w_x <= 1'b0; b_bi <= 2'd0; b_last <= 2'd0; w_bi <= 2'd0; w_last <= 2'd0;
		b_acc <= 24'd0; w_pb <= 2'd0; pb_req <= 1'b0; pb_addr <= 32'd0;
		b_pend <= 1'b0; w_pend <= 1'b0;
		b_addr <= 32'd0; b_size <= `AP040_SZ_L;
		b_fc <= `AP040_FC_SUPER_DATA; w_fc <= `AP040_FC_SUPER_DATA;
		w_addr <= 32'd0; w_data <= 32'd0; w_size <= `AP040_SZ_L; w_sla <= 30'd0;
		rvalid_b <= 1'b0;
		q_b <= 32'd0;
		mem_req <= 1'b0; mem_write <= 1'b0; mem_instr <= 1'b0;
		mem_size <= `AP040_SZ_L; mem_addr <= 32'd0; mem_wdata <= 32'd0;
		mem_fc <= `AP040_FC_SUPER_PROG;
	end else begin
		if (wr_drop) w_block <= 1'b0;
		if (w_pass_now) begin
			w_tent    <= 1'b0;
			w_receipt <= !wren_b;
		end else if (wren_b) w_receipt <= 1'b0;   // consumed: wr_busy was low for it
		if (rd_b || rx) begin
			b_addr   <= rx ? rx_addr : address_b;
			b_size   <= rx ? rx_size : size_b;
			b_x      <= !rx && xlat_e && crosses(address_b, size_b, pg_mask);
			b_bi     <= 2'd0;
			b_last   <= last_of(rx ? rx_size : size_b);
			b_acc    <= 24'd0;
			b_fc     <= rx ? rx_fc : fc_ovr ? fc_moves(fc_ovr_val) : fc_of(1'b0, sup_b);
			b_pend   <= 1'b1;
			rvalid_b <= 1'b0;
		end
		if (w_accept) begin
			w_addr <= address_b;
			w_sla  <= la_b[31:2];
			w_data <= data_b;
			w_size <= size_b;
			w_x    <= xlat_e && crosses(address_b, size_b, pg_mask);
			w_pb   <= (xlat_e && crosses(address_b, size_b, pg_mask)) ? 2'd1 : 2'd0;
			w_bi   <= 2'd0;
			w_last <= last_of(size_b);
			w_fc   <= fc_ovr ? fc_moves(fc_ovr_val) : fc_of(1'b0, sup_b);
			w_pend <= 1'b1;
			w_tent <= wr_sync;
		end

		// ---- the bus ----
		if (busy && mem_flt) begin
			// Refused, or a physical bus error: the transaction is over, and
			// whoever it was for is told. A demand fetch gets the fault with its
			// valid; a speculative prefetch stops the stream -- the fetch unit
			// asking for that longword later restarts it, and faults then, so a
			// fault is only ever raised on a word the program needs. A request
			// that joined a prefetch already on the bus is not a demand yet:
			// it stays pending, and the read goes out again for it. A write
			// already accepted (posted) has no instruction left to take it: that
			// is only a physical bus error, and it is told (wr_berr) to whoever
			// can report it -- on the 16-bit top the DMU, which holds it for
			// EA-fetch; elsewhere nobody, and it is dropped.
			busy    <= 1'b0;
			mem_req <= 1'b0;
			flt_bus <= mem_flt_bus;
			flt_ma  <= 1'b0;
			case (who)
			WHO_A: ;   // the IMU's (f_flt)
			WHO_BR: begin
				rvalid_b <= 1'b1;
				rflt_b   <= 1'b1;
				b_pend   <= 1'b0;
				// a crossing read's byte past the boundary: MA
				flt_ma   <= b_x && ((mem_addr & ~pg_mask32) != (b_addr & ~pg_mask32));
			end
			default: begin   // WHO_BW
				w_pend <= 1'b0;
				w_tent <= 1'b0;
				if (w_tent) w_block <= 1'b1;
			end
			endcase
		end else if (busy) begin
			if (mem_ack) begin
				busy    <= 1'b0;
				mem_req <= 1'b0;   // dropped in the ack cycle, per the contract
				case (who)
				WHO_A: ;   // the IMU's (f_ack)
				WHO_BR: if (b_x && (b_bi != b_last)) begin
					// a crossing read's byte, and more to come
					b_acc    <= {b_acc[15:0], mem_rdata[7:0]};
					b_bi     <= b_bi + 2'd1;
				end else begin
					q_b      <= b_x ? {b_acc, mem_rdata[7:0]} : mem_rdata;
					rvalid_b <= 1'b1;
					rflt_b   <= 1'b0;
					b_pend   <= 1'b0;
				end
				default: if (w_x && (w_bi != w_last)) w_bi <= w_bi + 2'd1;   // WHO_BW
				         else w_pend <= 1'b0;
				endcase
			end
		end else if (w_pend && w_x && (w_pb != 2'd0)) begin
			// A crossing write's probes, page 1 then page 2, before any byte.
			// The bus stays free; the MMU holds every transaction off while
			// the probe is up. Refused: the write is withdrawn, as one the MMU
			// refused on the bus, with MA if it was the second page.
			if (!pb_req) begin
				pb_req  <= 1'b1;
				pb_addr <= (w_pb == 2'd1) ? w_addr : ((w_addr | pg_mask32) + 32'd1);
			end else if (pb_done) begin
				pb_req <= 1'b0;
				if (pb_fail) begin
					w_pend  <= 1'b0;
					w_tent  <= 1'b0;
					w_block <= 1'b1;
					w_pb    <= 2'd0;
					flt_bus <= 1'b0;
					flt_ma  <= (w_pb == 2'd2);
				end else w_pb <= (w_pb == 2'd1) ? 2'd2 : 2'd0;
			end
		end else if (w_pend && !(wren_b && !w_pend)) begin
			busy      <= 1'b1;  who <= WHO_BW;
			mem_req   <= 1'b1;  mem_write <= 1'b1;  mem_instr <= 1'b0;
			mem_size  <= w_x ? `AP040_SZ_B : w_size;
			mem_addr  <= w_x ? (w_addr + {30'd0, w_bi}) : w_addr;
			mem_wdata <= w_x ? {24'd0, w_byte} : w_data;
			mem_fc    <= w_fc;
		end else if (b_pend || rx) begin
			busy      <= 1'b1;  who <= WHO_BR;
			mem_req   <= 1'b1;  mem_write <= 1'b0;  mem_instr <= 1'b0;
			mem_size  <= !b_pend ? rx_size : b_x ? `AP040_SZ_B : b_size;
			mem_addr  <= !b_pend ? rx_addr : b_x ? (b_addr + {30'd0, b_bi}) : b_addr;
			mem_fc    <= !b_pend ? rx_fc : b_fc;
		end else if (f_req) begin
			// the IMU's fetch
			busy       <= 1'b1;  who <= WHO_A;
			mem_req    <= 1'b1;  mem_write <= 1'b0;  mem_instr <= 1'b1;
			mem_size   <= `AP040_SZ_L; mem_addr <= f_addr;
			mem_fc     <= fc_of(1'b1, f_sup);
		end
	end
end

endmodule
