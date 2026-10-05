//--------------------------------------------------------------------------//
// AP68040-60 test bench                                                     //
//                                                                          //
// m68040_bus_slave.sv - memory on the 68040 bus (MC68040UM section 7)       //
//                                                                          //
// A synchronous slave: TS is sampled on a bus clock edge and TA/TEA/TBI/   //
// TCI are driven from flip-flops updated on bus clock edges, so the        //
// fastest transfer is the 68040's two-clock C1/C2 and a burst continues    //
// one beat per clock.  The slave increments A3:A2 of a line burst itself   //
// and wraps inside the line.                                               //
//                                                                          //
// Stress controls (set by the bench, may change between runs):             //
//   wait_mode  0: no wait states, 1: 0..3 random wait states per beat      //
//   tbi_mode   0: bursts allowed, 1: TBI on every line, 2: random TBI      //
//   retry_pct  percent chance of a TA+TEA retry on a retryable beat        //
//   tea_req    asserted by the bench while it wants the transfer that is   //
//              being answered to end in TEA (sampled when the beat would   //
//              be acknowledged)                                            //
//   hold       no answer while high: wait states (the benches tie the      //
//              68040's MI here, so memory waits for the snooper, 7.9)     //
//   oth_ta_n   TA from another responder (a snooping 68040 that inhibited //
//              memory and answers itself): counts the transfer's beats    //
//              so this slave follows it to its end                         //
//                                                                          //
// Every completed beat is published on the ev_* outputs for one clk so a   //
// bench can implement MMIO and protocol checks.                            //
//                                                                          //
// EXT = 1: the bench owns the memory.  Reads return ext_rdata for the      //
// beat's address (xfer_addr), an address with ext_inmem low ends in TEA,   //
// and writes are left to the bench (it applies the ev_* write events).     //
//--------------------------------------------------------------------------//

module m68040_bus_slave #(
	parameter int  AW   = 24,          // memory size 2**AW bytes at address 0
	parameter int  SEED = 1,
	parameter bit  EXT  = 0            // memory supplied by the bench
)(
	input  logic        clk,
	input  logic        nreset,
	input  logic        bclk_en,

	input  logic [31:0] a,
	input  logic [31:0] d_cpu,
	input  logic        rw_n,
	input  logic  [1:0] siz,
	input  logic  [1:0] tt,
	input  logic  [2:0] tm,
	input  logic        ts_n,
	output logic [31:0] d_mem,
	output logic        ta_n,
	output logic        tea_n,
	output logic        tbi_n,
	output logic        tci_n,
	output logic        avec_n,

	input  logic  [1:0] wait_mode,
	input  logic  [1:0] tbi_mode,
	input  int          retry_pct,
	input  logic        tea_req,
	input  logic        tci_req,
	input  logic        hold,          // insert wait states while high
	input  logic        oth_ta_n,      // another responder's TA
	input  logic  [7:0] iack_vector,   // 0 = answer with AVEC
	input  logic [31:0] ext_rdata,     // EXT: the long word at xfer_addr
	input  logic        ext_inmem,     // EXT: xfer_addr is backed by memory

	// the transfer being answered (valid while xfer_v)
	output logic        xfer_v,
	output logic [31:0] xfer_addr,     // address of the current beat
	output logic        xfer_rd,
	output logic  [1:0] xfer_siz,
	output logic  [1:0] xfer_tt,
	output logic  [2:0] xfer_tm,
	output logic  [1:0] xfer_beat,

	// completed-beat event (one clk)
	output logic        ev,
	output logic        ev_rd,
	output logic [31:0] ev_addr,       // long-word aligned for lines
	output logic [31:0] ev_data,
	output logic  [3:0] ev_be,         // byte lanes D31-24 .. D7-0
	output logic  [1:0] ev_siz,
	output logic  [1:0] ev_tt,
	output logic  [2:0] ev_tm,
	output logic        ev_err         // the beat ended in TEA
);

localparam int NW = 1 << (AW - 2);

logic [31:0] mem [0:NW-1];

logic        busy;
logic [31:0] base;          // address as driven with TS
logic        rd_q;
logic  [1:0] siz_q, tt_q;
logic  [2:0] tm_q;
logic  [1:0] beat;
logic  [2:0] waits;
logic        burst;          // still a burst (no TBI given)
logic  [1:0] inh_left;       // long-word transfers left of a burst-inhibited line

// byte lanes selected by SIZ/A1/A0 (Table 7-1)
function automatic logic [3:0] lanes(input logic [1:0] s, input logic [1:0] a10);
	case (s)
		2'b01: lanes = 4'b1000 >> a10;
		2'b10: lanes = a10[1] ? 4'b0011 : 4'b1100;
		default: lanes = 4'b1111;
	endcase
endfunction

function automatic logic [2:0] pick_waits();
	if (wait_mode == 2'd0) return 3'd0;
	return 3'($urandom % 4);
endfunction

wire  [1:0] a32   = base[3:2] + beat;
wire [31:0] baddr = (siz_q == 2'b11) ? {base[31:4], a32, 2'b00} : base;
wire        inmem = EXT ? ext_inmem : ((baddr >> AW) == 0);
wire [31:0] rword = EXT ? ext_rdata : mem[baddr[AW-1:2]];

assign xfer_v    = busy;
assign xfer_addr = baddr;
assign xfer_rd   = rd_q;
assign xfer_siz  = siz_q;
assign xfer_tt   = tt_q;
assign xfer_tm   = tm_q;
assign xfer_beat = beat;

initial begin
	void'($urandom(SEED));
end

always_ff @(posedge clk) begin
	ev <= 1'b0;
	if (!nreset) begin
		busy   <= 1'b0;
		ta_n   <= 1'b1;
		tea_n  <= 1'b1;
		tbi_n  <= 1'b1;
		tci_n  <= 1'b1;
		avec_n <= 1'b1;
		d_mem  <= '0;
		beat   <= 2'd0;
		waits  <= 3'd0;
		burst  <= 1'b0;
		inh_left <= 2'd0;
	end
	else if (bclk_en) begin
		ta_n   <= 1'b1;
		tea_n  <= 1'b1;
		tbi_n  <= 1'b1;
		tci_n  <= 1'b1;
		avec_n <= 1'b1;

		if (!busy) begin
			if (!ts_n) begin
				busy  <= 1'b1;
				base  <= a;
				rd_q  <= rw_n;
				siz_q <= siz;
				tt_q  <= tt;
				tm_q  <= tm;
				beat  <= 2'd0;
				burst <= (siz == 2'b11);
				waits <= pick_waits();
			end
		end
		else if (!oth_ta_n) begin
			// the transfer (this beat) was answered by another device
			if (burst && beat != 2'd3) beat <= beat + 1'd1;
			else busy <= 1'b0;
		end
		else if (ta_n == 1'b0 || tea_n == 1'b0) begin
			// the edge on which the CPU sampled our termination
			if (ta_n == 1'b0 && tea_n == 1'b1 && burst && beat != 2'd3) begin
				beat  <= beat + 1'd1;
				waits <= pick_waits();
			end
			else
				busy <= 1'b0;
			if (!ts_n) begin
				// back-to-back: TS for the next transfer came with our TA
				busy  <= 1'b1;
				base  <= a;
				rd_q  <= rw_n;
				siz_q <= siz;
				tt_q  <= tt;
				tm_q  <= tm;
				beat  <= 2'd0;
				burst <= (siz == 2'b11);
				waits <= pick_waits();
			end
		end
		else if (waits != 0) begin
			waits <= waits - 1'd1;
		end
		else if (hold) begin
			// the bench holds this beat
		end
		else begin
			// answer this beat
			logic retry;
			logic [3:0] be;
			// a burst-inhibited line's follow-up transfers may not be retried
			// (MC68040UM 7.6.2: that aborts the line)
			retry = (retry_pct > 0) && (beat == 2'd0) && (inh_left == 2'd0) &&
			        (($urandom % 100) < retry_pct) && (tt_q != 2'd3);
			be = lanes(siz_q, base[1:0]);
			if (tea_req || (!inmem && tt_q != 2'd3)) begin
				tea_n  <= 1'b0;
				ev     <= 1'b1;
				ev_err <= 1'b1;
			end
			else if (retry) begin
				ta_n  <= 1'b0;
				tea_n <= 1'b0;
			end
			else begin
				ta_n   <= 1'b0;
				ev     <= 1'b1;
				ev_err <= 1'b0;
				if (tt_q == 2'd3) begin
					// interrupt or breakpoint acknowledge
					if (iack_vector == 8'd0) avec_n <= 1'b0;
					d_mem <= {24'd0, iack_vector};
				end
				else if (rd_q) begin
					d_mem <= rword;
				end
				else if (!EXT) begin
					for (int i = 0; i < 4; i++)
						if (be[3 - i])
							mem[baddr[AW-1:2]][31 - 8*i -: 8] <= d_cpu[31 - 8*i -: 8];
				end
				if (inh_left != 2'd0) inh_left <= inh_left - 2'd1;
				if (beat == 2'd0) begin
					if (siz_q == 2'b11 && (tbi_mode == 2'd1 ||
					    (tbi_mode == 2'd2 && ($urandom % 2) == 0))) begin
						tbi_n <= 1'b0;
						burst <= 1'b0;
						inh_left <= 2'd3;
					end
					tci_n <= !tci_req;
				end
			end
			ev_rd   <= rd_q;
			ev_addr <= baddr;
			ev_data <= rd_q ? rword : d_cpu;
			ev_be   <= be;
			ev_siz  <= siz_q;
			ev_tt   <= tt_q;
			ev_tm   <= tm_q;
		end
	end
end

endmodule
