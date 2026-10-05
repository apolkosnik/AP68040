//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_snoop.sv - bus snooper (MC68040UM 4.4, 4.5, 4.7, 7.9)             //
//                                                                          //
// While another master owns the bus, every transfer it starts (TS) with   //
// TT = normal or MOVE16 and SC1/SC0 = 01 or 10 is snooped:                //
//                                                                          //
//   SC   read                               write                          //
//   01   dirty hit: inhibit memory, supply  byte/word/long dirty hit:      //
//        the data, line stays dirty         inhibit memory, sink the data //
//                                           into the line (stays dirty);  //
//                                           clean hit or line write:      //
//                                           invalidate, memory responds   //
//   10   dirty hit: inhibit memory, supply  any hit: invalidate, memory   //
//        the data, then invalidate; clean   responds                       //
//        hit: invalidate                                                   //
//                                                                          //
// The instruction cache drops a matching line for an SC = 10 read and     //
// for any snooped write (table 4-3).  SC = 00 and 11, and TT = 2 or 3,    //
// are not snooped.                                                        //
//                                                                          //
// MI (memory inhibit) is asserted whenever another master owns the bus,  //
// except while the memory is allowed to answer the current transfer: it  //
// is negated at once for a transfer that is not snooped, after the       //
// lookup when no intervention is needed, and never during an             //
// intervention, in which this unit is the slave (TA, and D31-D0 for a    //
// read; a line moves one long word per bus clock, A3:A2 wrapping).  MI   //
// is negated whenever we own the bus.  Terminations are counted only     //
// while MI is negated: four TAs for a line unless TBI ends the burst      //
// (the master then runs long-word transfers, each snooped), one TEA, or  //
// a retry.                                                                //
//                                                                          //
// Every output to the bus is a flip-flop that changes only on a bus clock //
// edge (bclk_en); inputs are sampled only then.  The cache lookup runs   //
// in processor clocks through the DMU's snoop port and the fetch unit's  //
// invalidation request.                                                   //
//--------------------------------------------------------------------------//

module ap68040_snoop
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        bclk_en,
	input  logic        own,            // BIU: we are the bus master

	// the bus, as the alternate master drives it
	input  logic [31:0] a_i,
	input  logic        ts_n_i,
	input  logic        rw_n_i,
	input  logic  [1:0] siz_i,
	input  logic  [1:0] tt_i,
	input  logic  [1:0] sc,
	input  logic [31:0] d_i,
	input  logic        ta_n_i,
	input  logic        tea_n_i,
	input  logic        tbi_n_i,
	output logic        mi_n,
	output logic        ta_n_o,
	output logic        ta_oe,
	output logic [31:0] d_o,
	output logic        d_oe,

	// data cache: the DMU's snoop port
	output logic        dc_req,
	output logic [31:0] dc_pa,
	input  logic        dc_look,
	input  logic        dc_hit,
	input  logic        dc_dirty,
	input  logic [127:0] dc_line,
	output logic        dc_inv,
	output logic        dc_wr,
	output logic [15:0] dc_wbe,
	output logic [31:0] dc_wword,       // on every long word, dc_wbe picks

	// instruction cache: line invalidation in the fetch unit
	output logic        ic_req,
	output logic [31:0] ic_pa,
	output logic        ic_all,         // ... or every line (queue overflow)
	input  logic        ic_done
);

typedef enum logic [2:0] {
	S_IDLE,     // between transfers: MI asserted unless we own the bus
	S_LOOK,     // the data cache is being searched (MI asserted)
	S_MEMW,     // memory may answer: negate MI on the next bus clock
	S_MEM,      // memory answers: count its terminations
	S_SRC,      // intervention: supply the data
	S_SINK,     // intervention: take the data
	S_SINKW,    // the data was taken: write it into the line
	S_END       // TA driven negated one bus clock before it is released
} st_t;

st_t         st;
logic        mi_q;            // MI as driven while another master owns the bus
logic [31:0] s_a;             // the snooped transfer
logic        s_rd, s_line;
logic  [1:0] s_siz, s_sc;
logic  [1:0] beat;
logic [127:0] s_line_d;       // the line supplied by an intervention

// I-cache invalidations wait in a two-entry queue (the fetch unit may be
// busy with a line fill): ic_req/ic_pa/ic_all is the head, iq_* the
// second entry.  A line that does not fit turns the rest of the queue
// into one invalidation of the whole cache (more than the 68040 drops,
// never less).
logic        iq_v1, iq_all;
logic [27:0] iq_l1;

assign mi_n = own ? 1'b1 : mi_q;

// byte lanes selected by SIZ/A1/A0 (MC68040UM table 7-1)
function automatic logic [3:0] lanes(input logic [1:0] s, input logic [1:0] a10);
	case (s)
		SIZ_B:   lanes = 4'b1000 >> a10;
		SIZ_W:   lanes = a10[1] ? 4'b0011 : 4'b1100;
		default: lanes = 4'b1111;
	endcase
endfunction

function automatic logic [31:0] line_word(input logic [127:0] l, input logic [1:0] k);
	line_word = l[127 - 32 * k -: 32];
endfunction

// a transfer starting on this bus clock edge
wire ts_alt  = !own && !ts_n_i;
wire snoopen = (tt_i == TT_NORMAL || tt_i == TT_MOVE16) && (sc == 2'b01 || sc == 2'b10);

always_ff @(posedge clk) begin
	dc_inv <= 1'b0;
	dc_wr  <= 1'b0;
	if (!nreset) begin
		st      <= S_IDLE;
		mi_q    <= 1'b0;          // asserted during and after reset (7.9.1)
		ta_n_o  <= 1'b1;
		ta_oe   <= 1'b0;
		d_o     <= '0;
		d_oe    <= 1'b0;
		s_a     <= '0;
		s_rd    <= 1'b0;
		s_line  <= 1'b0;
		s_siz   <= '0;
		s_sc    <= '0;
		beat    <= '0;
		s_line_d <= '0;
		dc_req  <= 1'b0;
		dc_pa   <= '0;
		dc_wbe  <= '0;
		dc_wword <= '0;
		ic_req  <= 1'b0;
		ic_pa   <= '0;
		ic_all  <= 1'b0;
		iq_v1   <= 1'b0;
		iq_all  <= 1'b0;
		iq_l1   <= '0;
	end
	else begin
		logic start;
		start = 1'b0;

		case (st)
		S_IDLE: begin
			if (bclk_en) begin
				mi_q  <= 1'b0;
				start = ts_alt;
			end
		end

		S_LOOK: begin
			if (dc_look) begin
				logic src, sink, inv;
				src  = s_rd && dc_hit && dc_dirty;
				sink = !s_rd && !s_line && s_sc == 2'b01 && dc_hit && dc_dirty;
				inv  = dc_hit && (s_rd ? (s_sc == 2'b10) : !sink);
				dc_inv   <= inv;
				s_line_d <= dc_line;
				if (sink) st <= S_SINK;          // keeps the cache until written
				else begin
					dc_req <= 1'b0;
					st     <= src ? S_SRC : S_MEMW;
				end
			end
		end

		S_MEMW: begin
			if (bclk_en) begin
				mi_q <= 1'b1;
				beat <= 2'd0;
				st   <= S_MEM;
			end
		end

		S_MEM: begin
			if (bclk_en) begin
				if (!tea_n_i || (!ta_n_i && (!s_line || beat == 2'd3 ||
				                            (beat == 2'd0 && !tbi_n_i)))) begin
					// TEA, retry, the last beat, or a burst-inhibited line
					// (its long words come as transfers of their own)
					mi_q  <= 1'b0;
					st    <= S_IDLE;
					start = ts_alt;
				end
				else if (!ta_n_i) beat <= beat + 2'd1;
			end
		end

		S_SRC: begin
			if (bclk_en) begin
				if (!ta_oe) begin
					// first beat: drive TA with the long word at A3:A2
					ta_oe  <= 1'b1;
					ta_n_o <= 1'b0;
					d_oe   <= 1'b1;
					d_o    <= line_word(s_line_d, s_a[3:2]);
					beat   <= 2'd0;
				end
				else if (!s_line || beat == 2'd3) begin
					ta_n_o <= 1'b1;
					d_oe   <= 1'b0;
					st     <= S_END;
				end
				else begin
					beat <= beat + 2'd1;
					d_o  <= line_word(s_line_d, s_a[3:2] + beat + 2'd1);
				end
			end
		end

		S_SINK: begin
			if (bclk_en) begin
				ta_oe  <= 1'b1;
				ta_n_o <= 1'b0;
				st     <= S_SINKW;
			end
		end

		S_SINKW: begin
			if (bclk_en) begin
				// the master sees TA on this edge, with its data on the bus
				logic [3:0] be;
				be = lanes(s_siz, s_a[1:0]);
				dc_wbe   <= {12'd0, be} << (4 * (2'd3 - s_a[3:2]));
				dc_wword <= d_i;
				dc_wr    <= 1'b1;
				dc_req   <= 1'b0;
				ta_n_o   <= 1'b1;
				st       <= S_END;
			end
		end

		S_END: begin
			if (bclk_en) begin
				ta_oe <= 1'b0;
				st    <= S_IDLE;
				start = ts_alt;
			end
		end

		default: st <= S_IDLE;
		endcase

		// a snooped transfer begins: latch it, ask the caches
		if (start) begin
			s_a    <= a_i;
			s_rd   <= rw_n_i;
			s_line <= (siz_i == SIZ_LINE);
			s_siz  <= siz_i;
			s_sc   <= sc;
			beat   <= 2'd0;
			if (snoopen) begin
				dc_req <= 1'b1;
				dc_pa  <= a_i;
				st     <= S_LOOK;
			end
			else begin
				mi_q <= 1'b1;            // not snooped: memory answers at once
				st   <= S_MEM;
			end
		end

		// the instruction cache: an invalidating read, or any write
		begin
			logic        v0, a0, v1, ov, push;
			logic [27:0] l0, l1, nl;
			v0 = ic_req; a0 = ic_all; l0 = ic_pa[31:4];
			v1 = iq_v1;  ov = iq_all; l1 = iq_l1;
			nl = a_i[31:4];
			push = start && snoopen && (!rw_n_i || sc == 2'b10);
			if (ic_done) begin
				// the head is done: the second entry moves up
				v0 = v1 || ov; a0 = ov; l0 = l1;
				v1 = 1'b0; ov = 1'b0;
			end
			if (push) begin
				if (!v0) begin v0 = 1'b1; a0 = 1'b0; l0 = nl; end
				else if (a0 || l0 == nl || ov || (v1 && l1 == nl)) ;   // covered
				else if (!v1) begin v1 = 1'b1; l1 = nl; end
				else ov = 1'b1;
			end
			ic_req <= v0; ic_all <= a0; ic_pa <= {l0, 4'd0};
			iq_v1  <= v1; iq_all <= ov; iq_l1 <= l1;
		end
	end
end

endmodule
