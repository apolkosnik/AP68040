//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_dmu.sv - data memory unit                                        //
//                                                                          //
// Mirrors the back end's DC1, DC2, EX and WB stages for memory uops:       //
//   DC1  address attributes (TTR match / translation)                      //
//   DC2  loads, and the load half of a read-modify-write                   //
//   WB   stores, and the store half of a read-modify-write                 //
//                                                                          //
// Accesses that are not cache hits run as bus transfers in program order:  //
// a load in DC2 waits while an older store is still in EX or WB.  A        //
// misaligned operand is split into aligned transfers by taking, at each   //
// address, the largest aligned piece that fits (MC68040UM table 7-3).      //
//--------------------------------------------------------------------------//

module ap68040_dmu
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,

	// back end
	input  logic        adv_ag,
	input  logic        dm_req,
	input  logic [31:0] dm_va,
	input  logic  [1:0] dm_mem,
	input  logic  [1:0] dm_msz,
	input  logic  [2:0] dm_fc,
	input  logic        dm_lock,
	input  logic        dm_locke,
	input  logic        dm_super,
	input  logic        dm_noalloc,
	input  logic        dm_iack,
	input  logic  [2:0] iack_lvl,
	input  logic        adv_dc1,
	input  logic        adv_dc2,
	input  logic        adv_ex,
	input  logic        adv_wb,
	input  logic        kill_now,
	output logic        dc2_rdy,
	output logic [31:0] ldata,
	output logic        fault,
	output logic  [7:0] fvec,
	output logic [31:0] faddr,
	output logic [15:0] fssw,
	input  logic        st_v,
	input  logic [31:0] st_data,
	output logic        st_rdy,
	output logic        st_fault,
	output logic [15:0] st_fssw,

	// control registers
	input  logic [31:0] cacr,
	input  logic [31:0] dtt0,
	input  logic [31:0] dtt1,

	// BIU client
	output logic        b_req,
	output busreq_t     b_breq,
	output logic [127:0] b_wdata,
	input  logic        b_gnt,
	input  logic        b_done,
	input  logic        b_err,
	input  logic        b_rvalid,
	input  logic [31:0] b_rdata,
	input  logic  [1:0] b_rbeat,
	input  logic        b_ravec
);

//--------------------------------------------------------------------------
// stage records
//--------------------------------------------------------------------------
typedef struct packed {
	logic        v;
	logic [31:0] a;       // logical address (physical while translation is off)
	logic  [1:0] mem;
	logic  [1:0] msz;
	logic  [2:0] fc;
	logic        lock;
	logic        locke;
	logic        smode;
	logic        noalloc;
	logic        iack;
	logic  [1:0] cm;      // cache mode: 0 WT, 1 CB, 2 CI serialized, 3 CI
	logic  [1:0] upa;
} mrec_t;

mrec_t m1, m2, m3, m4;

// transparent translation match (MC68040UM 3.1.3): base/mask on A31-A24,
// E, S field (00 user only, 01 supervisor only, 1x both)
function automatic logic ttr_hit(input logic [31:0] t, input logic [31:0] a,
                                 input logic s);
	logic [7:0] base, mask;
	base = t[31:24];
	mask = t[23:16];
	ttr_hit = t[15] &&
	          (((a[31:24] ^ base) & ~mask) == 8'd0) &&
	          (t[14] || (t[13] == s));
endfunction

// attributes of the access in DC1 (from the registered address)
wire        tt0 = ttr_hit(dtt0, m1.a, m1.smode);
wire        tt1 = ttr_hit(dtt1, m1.a, m1.smode);
wire  [1:0] dc1_cm  = tt0 ? dtt0[6:5] : tt1 ? dtt1[6:5] : 2'b00;
wire  [1:0] dc1_upa = tt0 ? dtt0[9:8] : tt1 ? dtt1[9:8] : 2'b00;

//--------------------------------------------------------------------------
// bus sequencer for one operand (load at DC2 or store at WB)
//--------------------------------------------------------------------------
// next aligned piece of an operand: returns the bus size and byte count
function automatic void piece(input logic [31:0] a, input logic [2:0] left,
                              output logic [1:0] bsiz, output logic [2:0] n);
	if (a[1:0] == 2'b00 && left >= 3'd4) begin bsiz = SIZ_L; n = 3'd4; end
	else if (a[0] == 1'b0 && left >= 3'd2) begin bsiz = SIZ_W; n = 3'd2; end
	else begin bsiz = SIZ_B; n = 3'd1; end
endfunction

function automatic logic [2:0] nbytes(input logic [1:0] msz);
	case (msz)
		SZ_B:    nbytes = 3'd1;
		SZ_W:    nbytes = 3'd2;
		default: nbytes = 3'd4;
	endcase
endfunction

typedef enum logic [1:0] { Q_IDLE, Q_REQ, Q_WAIT, Q_DONE } qst_t;

qst_t        q_st;
logic        q_wr;          // running the WB store
logic [31:0] q_a;           // address of the next piece
logic  [2:0] q_left;        // bytes still to transfer
logic  [2:0] q_n;           // bytes in the piece on the bus
logic [31:0] q_acc;         // operand assembled (load) / remaining (store)
logic        q_err;
logic        q_ld_done;     // DC2 load complete, data in ld_q
logic [31:0] ld_q;
logic        ld_fault;
logic        q_line;        // the transfer is a MOVE16 line
logic        q_iack;        // interrupt acknowledge
logic        q_avec;
logic [31:0] lbuf [4];      // MOVE16 line buffer, by long word of the line

logic  [1:0] p_siz;
logic  [2:0] p_n;
always_comb piece(q_a, q_left, p_siz, p_n);

// store operand bytes, left aligned as they go out: byte k of the operand
// (most significant first) is q_acc[31-8k -: 8]
wire  [1:0] lane0 = q_a[1:0];

// data placed on the lanes for a piece of n bytes at address q_a
function automatic logic [31:0] place(input logic [31:0] acc, input logic [2:0] n,
                                      input logic [1:0] a10);
	logic [31:0] v;
	case (n)
		3'd1:    v = {acc[31:24], acc[31:24], acc[31:24], acc[31:24]};
		3'd2:    v = {acc[31:16], acc[31:16]};
		default: v = acc;
	endcase
	place = v;
endfunction

// bytes of a read piece, from the lanes, right aligned
function automatic logic [31:0] grab(input logic [31:0] d, input logic [2:0] n,
                                     input logic [1:0] a10);
	case (n)
		3'd1:    grab = {24'd0, d[31 - 8*a10 -: 8]};
		3'd2:    grab = {16'd0, a10[1] ? d[15:0] : d[31:16]};
		default: grab = d;
	endcase
endfunction

// a store is older than the DC2 load when it is in EX or WB
wire older_store = (m3.v && (m3.mem == M_ST || m3.mem == M_RMW)) ||
                   (m4.v && (m4.mem == M_ST || m4.mem == M_RMW));
wire dc2_load    = m2.v && (m2.mem == M_LD || m2.mem == M_RMW);
wire start_ld    = dc2_load && !q_ld_done && q_st == Q_IDLE && !older_store && !kill_now;
wire start_st    = st_v && q_st == Q_IDLE && !st_done_q;
logic st_done_q;

always_comb begin
	b_req  = (q_st == Q_REQ);
	b_breq = '0;
	b_breq.addr  = q_iack ? 32'hFFFF_FFFF : q_line ? {q_a[31:4], 4'd0} : q_a;
	b_breq.siz   = q_iack ? SIZ_B : q_line ? SIZ_LINE : p_siz;
	b_breq.rd    = !q_wr;
	b_breq.tt    = q_iack ? TT_ACK : q_line ? TT_MOVE16 :
	               (q_fc == 3'd1 || q_fc == 3'd2 || q_fc == 3'd5 || q_fc == 3'd6) ?
	               TT_NORMAL : TT_ALT;
	b_breq.tm    = q_iack ? iack_lvl : q_fc;
	b_breq.upa   = q_upa;
	b_breq.ci    = q_cm[1];
	b_breq.lock  = q_lock;
	b_breq.locke = q_lock && q_locke && q_wr && (q_left == p_n);
	b_wdata      = q_line ? {lbuf[0], lbuf[1], lbuf[2], lbuf[3]} :
	               {96'd0, place(q_acc, p_n, q_a[1:0])};
end

logic  [2:0] q_fc;
logic  [1:0] q_upa, q_cm;
logic        q_lock, q_locke;

always_ff @(posedge clk) begin
	st_rdy <= 1'b0;
	if (!nreset) begin
		m1 <= '0; m2 <= '0; m3 <= '0; m4 <= '0;
		q_st <= Q_IDLE;
		q_wr <= 1'b0;
		q_a  <= '0;
		q_left <= '0;
		q_n  <= '0;
		q_acc <= '0;
		q_err <= 1'b0;
		q_ld_done <= 1'b0;
		ld_q <= '0;
		ld_fault <= 1'b0;
		st_done_q <= 1'b0;
		st_fault <= 1'b0;
		q_fc <= '0; q_upa <= '0; q_cm <= '0; q_lock <= 1'b0; q_locke <= 1'b0;
		q_line <= 1'b0;
		q_iack <= 1'b0;
		q_avec <= 1'b0;
	end
	else begin
		//--------------------------------------------------------------
		// stage records
		//--------------------------------------------------------------
		if (adv_ag) begin
			m1.v       <= dm_req;
			m1.a       <= dm_va;
			m1.mem     <= dm_mem;
			m1.msz     <= dm_msz;
			m1.fc      <= dm_fc;
			m1.lock    <= dm_lock;
			m1.locke   <= dm_locke;
			m1.smode   <= dm_super;
			m1.noalloc <= dm_noalloc;
			m1.iack    <= dm_iack;
		end
		else if (adv_dc1) m1.v <= 1'b0;

		if (adv_dc1) begin
			m2     <= m1;
			m2.cm  <= dc1_cm;
			m2.upa <= dc1_upa;
		end
		else if (adv_dc2) m2.v <= 1'b0;

		if (adv_dc2) begin
			m3 <= m2;
			q_ld_done <= 1'b0;
		end
		else if (adv_ex) m3.v <= 1'b0;

		if (adv_ex) m4 <= m3;
		else if (adv_wb) m4.v <= 1'b0;

		if (adv_wb) st_done_q <= 1'b0;

		if (kill_now) begin
			m1.v <= 1'b0;
			m2.v <= 1'b0;
			m3.v <= 1'b0;
			m4.v <= 1'b0;
			q_ld_done <= 1'b0;
		end

		//--------------------------------------------------------------
		// bus sequencer
		//--------------------------------------------------------------
		case (q_st)
		Q_IDLE: begin
			if (start_st) begin
				q_iack <= 1'b0;
				q_line <= (m4.msz == SZ_Q);
				q_wr   <= 1'b1;
				q_a    <= m4.a;
				q_left <= nbytes(m4.msz);
				// left-align the operand
				case (m4.msz)
					SZ_B:    q_acc <= {st_data[7:0], 24'd0};
					SZ_W:    q_acc <= {st_data[15:0], 16'd0};
					default: q_acc <= st_data;
				endcase
				q_fc   <= m4.fc;
				q_upa  <= m4.upa;
				q_cm   <= m4.cm;
				q_lock <= m4.lock;
				q_locke <= m4.locke;
				q_err  <= 1'b0;
				q_st   <= Q_REQ;
			end
			else if (start_ld) begin
				q_iack <= m2.iack;
				q_avec <= 1'b0;
				q_line <= (m2.msz == SZ_Q);
				q_wr   <= 1'b0;
				q_a    <= m2.a;
				q_left <= nbytes(m2.msz);
				q_acc  <= '0;
				q_fc   <= m2.fc;
				q_upa  <= m2.upa;
				q_cm   <= m2.cm;
				q_lock <= m2.lock;
				q_locke <= 1'b0;
				q_err  <= 1'b0;
				q_st   <= Q_REQ;
			end
		end
		Q_REQ: begin
			if (b_gnt) begin
				q_n  <= p_n;
				q_st <= Q_WAIT;
			end
		end
		Q_WAIT: begin
			if (b_rvalid && !q_wr && q_line)
				lbuf[b_rbeat] <= b_rdata;
			else if (b_rvalid && !q_wr && q_iack) begin
				q_acc  <= {24'd0, b_rdata[7:0]};
				q_avec <= b_ravec;
			end
			else if (b_rvalid && !q_wr)
				q_acc <= (q_acc << (8 * q_n)) | grab(b_rdata, q_n, q_a[1:0]);
			if (b_err) begin
				q_err <= 1'b1;
				q_st  <= Q_DONE;
			end
			else if (b_done && (q_line || q_iack)) begin
				q_st <= Q_DONE;
			end
			else if (b_done) begin
				if (q_wr) q_acc <= q_acc << (8 * q_n);
				q_a    <= q_a + {29'd0, q_n};
				q_left <= q_left - q_n;
				q_st   <= (q_left == q_n) ? Q_DONE : Q_REQ;
			end
		end
		Q_DONE: begin
			if (q_wr) begin
				st_rdy    <= 1'b1;
				st_fault  <= q_err;
				st_done_q <= 1'b1;
			end
			else if (m2.v && q_iack) begin
				// IACK: [7:0] vector, [8] AVEC, [9] TEA = spurious
				q_ld_done <= 1'b1;
				ld_q      <= {22'd0, q_err, q_avec, q_acc[7:0]};
				ld_fault  <= 1'b0;
			end
			else if (m2.v) begin
				q_ld_done <= 1'b1;
				ld_q      <= q_acc;
				ld_fault  <= q_err;
			end
			q_st <= Q_IDLE;
		end
		endcase
	end
end

// DC2 is ready: loads once their data is in, everything else at once
assign dc2_rdy  = !m2.v || (m2.mem == M_ST) || q_ld_done;
assign ldata    = ld_q;
assign fault    = q_ld_done && ld_fault;
assign fvec     = 8'd2;
assign faddr    = m2.a;
assign fssw     = {5'd0, 1'b0, 1'b0, m2.lock, 1'b1, m2.msz == SZ_L ? 2'b00 :
                   m2.msz == SZ_B ? 2'b01 : 2'b10, 2'b00, m2.fc};
assign st_fssw  = {5'd0, 1'b0, 1'b0, m4.lock, 1'b0, m4.msz == SZ_L ? 2'b00 :
                   m4.msz == SZ_B ? 2'b01 : 2'b10, 2'b00, m4.fc};

endmodule
