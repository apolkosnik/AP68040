//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_biu.sv - bus interface unit (MC68040UM section 7)                 //
//                                                                          //
// Runs one internal transaction at a time on the 68040 bus:                //
//   * byte/word/long transfers (the requester has already split a          //
//     misaligned operand into aligned pieces, 7.3)                         //
//   * line transfers as a burst; TBI on the first beat falls back to       //
//     three long-word transfers (7.4.2, 7.4.4); A3:A2 of a burst are       //
//     incremented by the slave and wrap inside the line                   //
//   * TA+TEA retry of a byte/word/long transfer and of the first beat of  //
//     a line; a retry on a later beat is a bus error (7.6.2)              //
//   * TEA bus error, reported with the beat it hit (7.6.1)                //
//   * locked sequences: the arbiter stays with the locking requester       //
//     from its first LOCK transfer until the LOCKE transfer completes      //
//   * bus arbitration BR/BG/BB (7.8): ownership is taken when BG is        //
//     asserted and BB is free; a negated BG gives the bus up at the end   //
//     of the transfer in progress, unless a locked sequence is open        //
//   * RSTO for 512 bus clocks (RESET instruction, 7.10)                    //
//                                                                          //
// Every bus output is a flip-flop that changes only in a clk cycle with    //
// bclk_en high; inputs are sampled only in such cycles.                    //
//                                                                          //
// Requester interface (per client c):                                      //
//   req[c]      level; breq[c] and wdata[c] must be stable while req[c]    //
//               is high and gnt[c] has not pulsed                          //
//   gnt[c]      pulse: the transaction is latched, req may drop or change  //
//   rvalid      pulse per read beat for the client in rclient; rdata,      //
//               rbeat (A3:A2 of the long word), avec (AVEC with TA)        //
//   done[c]     pulse: transaction finished without error                  //
//   err[c]      pulse: transaction ended with a bus error; errbeat says    //
//               which long word of a line (0 = the first transferred)      //
//   tci         TCI sampled with the first beat, valid with the first      //
//               rvalid                                                     //
// Line write data: wdata[c][127:96] is the long word at A3:A2 = 0, ...,    //
// wdata[c][31:0] at A3:A2 = 3.  Single writes take wdata[c][31:0] already  //
// placed on the byte lanes D31-D0.                                         //
//--------------------------------------------------------------------------//

module ap68040_biu
	import ap68040_pkg::*;
#(
	parameter int NC = 3            // number of requesters, 0 = highest priority
)
(
	input  logic              clk,
	input  logic              nreset,
	input  logic              bclk_en,

	// requesters
	input  logic [NC-1:0]     req,
	input  busreq_t           breq   [NC],
	input  logic [127:0]      wdata  [NC],
	output logic [NC-1:0]     gnt,
	output logic [NC-1:0]     done,
	output logic [NC-1:0]     err,
	output logic              rvalid,
	output logic [$clog2(NC)-1:0] rclient,
	output logic [31:0]       rdata,
	output logic  [1:0]       rbeat,
	output logic              ravec,
	output logic              rtci,
	output logic  [1:0]       errbeat,
	output logic              idle,     // no transaction latched or running
	output logic              own,      // we are the bus master (BB ours)

	input  logic              unlock,   // abandon an open locked sequence

	// RESET instruction
	input  logic              rsto_req,  // pulse
	output logic              rsto_busy,

	// 68040 bus
	output logic [31:0]       a_o,
	output logic              a_oe,
	input  logic [31:0]       d_i,
	output logic [31:0]       d_o,
	output logic              d_oe,
	output logic              rw_n,
	output logic  [1:0]       siz,
	output logic  [1:0]       tt,
	output logic  [2:0]       tm,
	output logic  [1:0]       tln,
	output logic  [1:0]       upa,
	output logic              ciout_n,
	output logic              lock_n,
	output logic              locke_n,
	output logic              ts_n,
	output logic              tip_n,
	input  logic              ta_n,
	input  logic              tea_n,
	input  logic              tci_n,
	input  logic              tbi_n,
	input  logic              avec_n,
	output logic              br_n,
	input  logic              bg_n,
	input  logic              bb_n_i,
	output logic              bb_n_o,
	output logic              bb_oe,
	output logic              rsto_n
);

localparam int CW = (NC > 1) ? $clog2(NC) : 1;

typedef enum logic [2:0] {
	S_IDLE,     // owner (or not), no transfer
	S_C1,       // TS asserted this bus clock
	S_DATA,     // waiting for TA/TEA, burst beats
	S_GAP       // a burst-inhibited line restarts the next long word
} state_t;

state_t      st;
busreq_t     cur;
logic [CW-1:0] cur_c;
logic [127:0] cur_wd;
logic  [1:0] beat;        // beats completed so far in a line
logic        inhibited;   // line continued as long-word transfers
logic        owner;       // we hold BB
logic        locked;      // a locked sequence is open
logic [CW-1:0] lock_c;

//--------------------------------------------------------------------------
// arbitration among requesters: fixed priority, except that an open locked
// sequence belongs to its requester alone
//--------------------------------------------------------------------------
logic          pick_v;
logic [CW-1:0] pick_c;
always_comb begin
	pick_v = 1'b0;
	pick_c = '0;
	if (locked) begin
		pick_v = req[lock_c];
		pick_c = lock_c;
	end
	else begin
		for (int i = NC - 1; i >= 0; i--)
			if (req[i]) begin
				pick_v = 1'b1;
				pick_c = CW'(i);
			end
	end
end

// long-word data for beat number n of the latched line (address order)
function automatic logic [31:0] line_word(input logic [127:0] w, input logic [1:0] a32);
	case (a32)
		2'd0: line_word = w[127:96];
		2'd1: line_word = w[95:64];
		2'd2: line_word = w[63:32];
		default: line_word = w[31:0];
	endcase
endfunction

wire  [1:0] cur_a32  = cur.addr[3:2] + beat;   // long word now on the bus
wire        is_line  = (cur.siz == SIZ_LINE);
wire        ta       = !ta_n;
wire        tea      = !tea_n;

// ownership: BG asserted and the bus free, or already ours and BG still
// asserted (or a locked sequence open: it keeps the bus)
wire        can_own  = (owner && (!bg_n || locked)) || (!bg_n && bb_n_i);

logic [9:0] rsto_cnt;
logic       rsto_pend;   // a RESET request waiting for a bus clock
logic       unlock_pend;

always_ff @(posedge clk) begin
	gnt    <= '0;
	done   <= '0;
	err    <= '0;
	rvalid <= 1'b0;

	if (!nreset) begin
		st        <= S_IDLE;
		cur       <= '0;
		cur_c     <= '0;
		cur_wd    <= '0;
		beat      <= 2'd0;
		inhibited <= 1'b0;
		owner     <= 1'b0;
		locked    <= 1'b0;
		lock_c    <= '0;
		rclient   <= '0;
		rdata     <= '0;
		rbeat     <= 2'd0;
		ravec     <= 1'b0;
		rtci      <= 1'b0;
		errbeat   <= 2'd0;
		a_o       <= '0;
		a_oe      <= 1'b0;
		d_o       <= '0;
		d_oe      <= 1'b0;
		rw_n      <= 1'b1;
		siz       <= 2'd0;
		tt        <= 2'd0;
		tm        <= 3'd0;
		tln       <= 2'd0;
		upa       <= 2'd0;
		ciout_n   <= 1'b1;
		lock_n    <= 1'b1;
		locke_n   <= 1'b1;
		ts_n      <= 1'b1;
		tip_n     <= 1'b1;
		br_n      <= 1'b1;
		bb_n_o    <= 1'b1;
		bb_oe     <= 1'b0;
		rsto_n    <= 1'b1;
		rsto_cnt  <= '0;
		rsto_pend <= 1'b0;
		unlock_pend <= 1'b0;
	end
	else begin
		if (rsto_req) rsto_pend <= 1'b1;
		if (unlock) unlock_pend <= 1'b1;
		if ((unlock || unlock_pend) && st == S_IDLE) begin
			locked      <= 1'b0;
			unlock_pend <= 1'b0;
		end
		if (bclk_en) begin
			// RESET instruction: RSTO for 512 bus clocks
			if (rsto_cnt != 0) begin
				rsto_cnt <= rsto_cnt - 1'd1;
				if (rsto_cnt == 10'd1) rsto_n <= 1'b1;
			end

			// bus request: asserted while there is work and the bus is
			// not ours; negated once we own it
			br_n <= !((pick_v || st != S_IDLE) && !owner);

			case (st)
			S_IDLE: begin
				ts_n <= 1'b1;
				if (pick_v && can_own) begin
					// start a transfer
					owner   <= 1'b1;
					bb_n_o  <= 1'b0;
					bb_oe   <= 1'b1;
					cur     <= breq[pick_c];
					cur_c   <= pick_c;
					cur_wd  <= wdata[pick_c];
					gnt[pick_c] <= 1'b1;
					beat    <= 2'd0;
					inhibited <= 1'b0;
					if (breq[pick_c].lock) begin
						locked <= !breq[pick_c].locke;
						lock_c <= pick_c;
					end
					a_o     <= breq[pick_c].addr;
					a_oe    <= 1'b1;
					rw_n    <= breq[pick_c].rd;
					siz     <= breq[pick_c].siz;
					tt      <= breq[pick_c].tt;
					tm      <= breq[pick_c].tm;
					tln     <= breq[pick_c].tln;
					upa     <= breq[pick_c].upa;
					ciout_n <= !breq[pick_c].ci;
					lock_n  <= !breq[pick_c].lock;
					locke_n <= !(breq[pick_c].lock && breq[pick_c].locke);
					d_o     <= (breq[pick_c].siz == SIZ_LINE) ?
					           line_word(wdata[pick_c], breq[pick_c].addr[3:2]) :
					           wdata[pick_c][31:0];
					d_oe    <= !breq[pick_c].rd;
					ts_n    <= 1'b0;
					tip_n   <= 1'b0;
					st      <= S_C1;
				end
				else begin
					tip_n <= 1'b1;
					d_oe  <= 1'b0;
					// keep driving the bus only while we own it
					if (owner && bg_n && !locked) begin
						owner  <= 1'b0;
						bb_n_o <= 1'b1;   // negate, then release next clock
					end
					else if (!owner) begin
						bb_oe <= 1'b0;
						a_oe  <= 1'b0;
					end
				end
			end

			S_C1: begin
				ts_n <= 1'b1;
				st   <= S_DATA;
			end

			S_GAP: begin
				// next long word of a burst-inhibited line
				a_o   <= {cur.addr[31:4], cur_a32, cur.addr[1:0]};
				siz   <= SIZ_L;
				d_o   <= line_word(cur_wd, cur_a32);
				ts_n  <= 1'b0;
				st    <= S_C1;
			end

			S_DATA: begin
				if (ta && tea) begin
					// retry: legal on a single transfer and on the first
					// beat of a line, otherwise a bus error
					if (beat == 2'd0 && !inhibited) begin
						ts_n <= 1'b0;
						st   <= S_C1;
					end
					else begin
						err[cur_c] <= 1'b1;
						errbeat    <= beat;
						locked     <= locked && !cur.locke;
						d_oe       <= 1'b0;
						st         <= S_IDLE;
					end
				end
				else if (tea) begin
					err[cur_c] <= 1'b1;
					errbeat    <= beat;
					// a bus error ends a locked sequence: the requester
					// takes an access error and will not complete it
					locked     <= 1'b0;
					d_oe       <= 1'b0;
					st         <= S_IDLE;
				end
				else if (ta) begin
					if (cur.rd) begin
						rvalid  <= 1'b1;
						rclient <= cur_c;
						rdata   <= d_i;
						rbeat   <= cur_a32;
						ravec   <= !avec_n;
						if (beat == 2'd0) rtci <= !tci_n;
					end
					if (!is_line || beat == 2'd3) begin
						done[cur_c] <= 1'b1;
						if (cur.lock && cur.locke) locked <= 1'b0;
						d_oe <= 1'b0;
						st   <= S_IDLE;
					end
					else begin
						beat <= beat + 1'd1;
						if (beat == 2'd0 && !tbi_n) begin
							inhibited <= 1'b1;
							st        <= S_GAP;
						end
						else if (inhibited) begin
							st <= S_GAP;
						end
						else begin
							// burst: the slave advances A3:A2
							d_o <= line_word(cur_wd, cur_a32 + 2'd1);
						end
					end
				end
			end

			default: st <= S_IDLE;
			endcase

			if (rsto_req || rsto_pend) begin
				rsto_n    <= 1'b0;
				rsto_cnt  <= 10'd512;
				rsto_pend <= 1'b0;
			end
		end
	end
end

assign idle      = (st == S_IDLE);
assign own       = owner;
assign rsto_busy = !rsto_n || rsto_pend;

endmodule
