//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_fetch.sv - instruction fetch and the instruction queue            //
//                                                                          //
// The queue is a 12-word shift register: slot 0 is always the next word   //
// D1 decodes, so D1 reads registers at fixed positions.  Every word is    //
// predecoded (ap68040_predec) as it enters, so D1 sizes an instruction    //
// from slot 0's predecode without decoding it.  D1 consumes 0..7 words a //
// cycle.  Fetches run ahead sequentially while there is room; a redirect  //
// (D1 branch or a WB redirect) empties the queue.  A bus error on a fetch //
// marks the words; D1 raises the fault only if it uses them (8.2.1).      //
// Only the first fetch of a redirected stream is a demand fetch; the     //
// sequential lookahead after it is speculative, and a fault there is     //
// retried by the back end as a demand fetch (refetch from the            //
// instruction) before it becomes an access error.                         //
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
	output logic  [7:0] win_fdem,     // ... the first of a redirected stream
	output pd_t         pd0,          // predecode of win[0]
	output logic  [3:0] qcnt,
	output logic [31:0] qpc,
	input  logic  [2:0] consume,
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

localparam int QN = 12;

logic [15:0] qw   [QN];
pd_t         qp   [QN];
logic [QN-1:0] qf;
logic [QN-1:0] qd;            // fault came from a demand fetch
logic        fdem;            // the fetch on the bus was issued on demand
logic        fnew;            // the next fetch starts a redirected stream
logic  [3:0] cnt;
logic [31:0] qpc_r;

logic [31:0] fpc;             // address of the next fetch (word aligned)
logic        busy;            // a fetch is on the bus
logic        stale;           // the fetch on the bus belongs to a flushed stream
logic        odd;             // the stream starts at an odd address

assign qcnt  = cnt;
assign qpc   = qpc_r;
assign q_odd = odd;
assign pd0   = qp[0];

always_comb begin
	for (int i = 0; i < 8; i++) begin
		win[i]     = qw[i];
		win_flt[i] = qf[i];
		win_fdem[i] = qd[i];
	end
end

// predecode of the two words of a fetched long word
pd_t pd_hi, pd_lo;
ap68040_predec pdh (.op(b_rdata[31:16]), .pd(pd_hi));
ap68040_predec pdl (.op(b_rdata[15:0]),  .pd(pd_lo));

// a request only when the queue has room for a whole long word even if D1
// consumes nothing (registered count: no dependency on D1 this cycle)
always_comb begin
	b_req  = !busy && (cnt <= 4'(QN - 2)) && !stop && !redir_v && !d_redir_v && !odd;
	b_breq = '0;
	b_breq.addr = {fpc[31:2], 2'b00};
	b_breq.siz  = SIZ_L;
	b_breq.rd   = 1'b1;
	b_breq.tt   = TT_NORMAL;
	b_breq.tm   = smode ? TM_SCODE : TM_UCODE;
end

wire        arrive = busy && (b_done || b_err) && !stale;
wire  [1:0] nin    = arrive ? (fpc[1] ? 2'd1 : 2'd2) : 2'd0;
wire  [3:0] keep   = cnt - {1'b0, consume};    // words left after D1

always_ff @(posedge clk) begin
	if (!nreset) begin
		cnt   <= '0;
		qf    <= '0;
		qd    <= '0;
		fdem  <= 1'b0;
		fnew  <= 1'b1;
		fpc   <= '0;
		qpc_r <= '0;
		busy  <= 1'b0;
		stale <= 1'b0;
		odd   <= 1'b0;
		for (int i = 0; i < QN; i++) begin qw[i] <= '0; qp[i] <= '0; end
	end
	else begin
		if (b_gnt) begin
			busy <= 1'b1;
			fdem <= fnew;
			fnew <= 1'b0;
		end
		if (busy && (b_done || b_err)) begin
			busy  <= 1'b0;
			stale <= 1'b0;
			if (!stale) fpc <= {fpc[31:2], 2'b00} + 32'd4;
		end

		// shift out what D1 consumed, append what arrived
		for (int i = 0; i < QN; i++) begin
			logic [4:0] src;
			src = 5'(i) + {2'b00, consume};
			if (4'(i) < keep) begin
				qw[i] <= qw[src[3:0]];
				qp[i] <= qp[src[3:0]];
				qf[i] <= qf[src[3:0]];
				qd[i] <= qd[src[3:0]];
			end
			else if (nin == 2'd2 && 4'(i) == keep) begin
				qw[i] <= b_rdata[31:16]; qp[i] <= pd_hi; qf[i] <= b_err; qd[i] <= fdem;
			end
			else if (nin == 2'd2 && 4'(i) == keep + 4'd1) begin
				qw[i] <= b_rdata[15:0];  qp[i] <= pd_lo; qf[i] <= b_err; qd[i] <= fdem;
			end
			else if (nin == 2'd1 && 4'(i) == keep) begin
				qw[i] <= b_rdata[15:0];  qp[i] <= pd_lo; qf[i] <= b_err; qd[i] <= fdem;
			end
		end
		cnt   <= keep + {2'b00, nin};
		qpc_r <= qpc_r + {28'd0, consume, 1'b0};

		if (redir_v || d_redir_v) begin
			logic [31:0] npc;
			npc = redir_v ? redir_pc : d_redir_pc;
			cnt   <= '0;
			fnew  <= 1'b1;
			fpc   <= {npc[31:1], 1'b0};
			qpc_r <= npc;
			odd   <= npc[0];
			if (busy && !(b_done || b_err)) stale <= 1'b1;
			if (b_gnt) stale <= 1'b1;
		end
	end
end

endmodule
