//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_fetch.sv - instruction fetch and the instruction queue            //
//                                                                          //
// The queue is a 16-word circular buffer.  The decoder sees the first      //
// eight words (win), their count, the address of win[0] (qpc) and a fault  //
// flag per word; it consumes 0..8 words per cycle.  Fetches run ahead      //
// sequentially until the queue is full; a redirect (decoder branch or a    //
// WB redirect) empties the queue and restarts at the new address.  A bus  //
// error on a fetch is not an exception here: the words are marked and the  //
// decoder raises the fault only if it uses them (MC68040UM 8.2.1).         //
//--------------------------------------------------------------------------//

module ap68040_fetch
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,

	input  logic        redir_v,      // highest priority
	input  logic [31:0] redir_pc,
	input  logic        d_redir_v,    // decoder (predicted branch)
	input  logic [31:0] d_redir_pc,
	input  logic        stop,         // do not start new fetches

	input  logic        smode,        // S bit (selects TM 6/2)

	// decoder side
	output logic [15:0] win [8],
	output logic  [7:0] win_flt,      // the word came from a faulted fetch
	output logic  [4:0] qcnt,
	output logic [31:0] qpc,
	input  logic  [3:0] consume,
	output logic        q_odd,        // the stream starts at an odd address

	// BIU client
	output logic        b_req,
	output busreq_t     b_breq,
	input  logic        b_gnt,
	input  logic        b_done,
	input  logic        b_err,
	input  logic        b_rvalid,
	input  logic [31:0] b_rdata
);

logic [15:0] qbuf [16];
logic [15:0] qflt;
logic  [3:0] h;
logic  [4:0] cnt;

logic [31:0] fpc;             // address of the next fetch (word aligned)
logic        busy;            // a fetch is on the bus
logic        gnt_seen;
logic        stale;           // the fetch on the bus belongs to a flushed stream
logic        odd;             // the stream starts at an odd address

assign qcnt = cnt;
assign q_odd = odd;
assign qpc  = qpc_r;
logic [31:0] qpc_r;

always_comb begin
	for (int i = 0; i < 8; i++) begin
		win[i]     = qbuf[4'(h + i)];
		win_flt[i] = qflt[4'(h + i)];
	end
end

// words this long word contributes: two, or one when fpc points at its
// second word (after a redirect)
wire  [1:0] nwords = fpc[1] ? 2'd1 : 2'd2;
wire  [4:0] cnt_after = cnt - {1'b0, consume};
wire        room   = (cnt_after + 5'd2) <= 5'd16;

always_comb begin
	b_req  = !busy && room && !stop && !redir_v && !d_redir_v && !odd;
	b_breq = '0;
	b_breq.addr = {fpc[31:2], 2'b00};
	b_breq.siz  = SIZ_L;
	b_breq.rd   = 1'b1;
	b_breq.tt   = TT_NORMAL;
	b_breq.tm   = smode ? TM_SCODE : TM_UCODE;
end

always_ff @(posedge clk) begin
	if (!nreset) begin
		h      <= '0;
		cnt    <= '0;
		qflt   <= '0;
		fpc    <= '0;
		qpc_r  <= '0;
		busy   <= 1'b0;
		stale  <= 1'b0;
		odd    <= 1'b0;
		for (int i = 0; i < 16; i++) qbuf[i] <= '0;
	end
	else begin
		logic [3:0] hn;
		logic [4:0] cn;
		hn = h + consume;
		cn = cnt - {1'b0, consume};

		if (b_gnt) busy <= 1'b1;

		if (busy && (b_done || b_err)) begin
			busy <= 1'b0;
			if (!stale) begin
				if (nwords == 2'd2) begin
					qbuf[4'(hn + cn)]     <= b_rdata[31:16];
					qbuf[4'(hn + cn + 1)] <= b_rdata[15:0];
					qflt[4'(hn + cn)]     <= b_err;
					qflt[4'(hn + cn + 1)] <= b_err;
					cn = cn + 5'd2;
				end
				else begin
					qbuf[4'(hn + cn)] <= b_rdata[15:0];
					qflt[4'(hn + cn)] <= b_err;
					cn = cn + 5'd1;
				end
				fpc <= {fpc[31:2], 2'b00} + 32'd4;
			end
			stale <= 1'b0;
		end

		h     <= hn;
		cnt   <= cn;
		qpc_r <= qpc_r + {27'd0, consume, 1'b0};

		if (redir_v || d_redir_v) begin
			logic [31:0] npc;
			npc = redir_v ? redir_pc : d_redir_pc;
			h     <= '0;
			cnt   <= '0;
			fpc   <= {npc[31:1], 1'b0};
			qpc_r <= npc;
			odd   <= npc[0];
			if (busy && !(b_done || b_err)) stale <= 1'b1;
			if (b_gnt) stale <= 1'b1;
		end
	end
end

endmodule
