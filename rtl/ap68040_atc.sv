//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                            //
//                                                                          //
// ap68040_atc.sv - address translation cache (MC68040UM 3.3)                //
//                                                                          //
// 64 entries, four-way set associative, 16 sets indexed by LA15-12 (4K     //
// pages) or LA16-13 (8K pages).  The tag is FC2 and the logical page; the  //
// entry holds the physical page and the page attributes.  Valid and global //
// bits are flip-flops (PFLUSHA/PFLUSHAN in one cycle); tags and entries    //
// are MLAB (asynchronous read).                                            //
//                                                                          //
// Lookup: la/fc2 are the caller's registered address; the four ways are   //
// compared here and the hit way's entry returned.  Replacement takes an    //
// existing entry for the same page, else the first invalid way, else the  //
// way a 2-bit counter points to (the counter advances on every miss fill).//
// PFLUSH by page reads the set through the lookup port: the caller holds  //
// the lookup inputs on the page for one cycle while flush_page is high.   //
//--------------------------------------------------------------------------//

module ap68040_atc
	import ap68040_pkg::*;
#(
	// an install looks for its page among the set's entries (a data
	// access's M-bit update rewrites its own entry).  0: no copy for the
	// search -- the caller says whether the page is there (wsame) and in
	// which way (the hit vector of the lookup that started the search)
	parameter bit SAME_SEARCH = 1,
	// the lookup's set from the caller, registered (set_m), not formed from
	// la: no logic between the caller's flip-flops and the entries' read
	// address
	parameter bit EXT_SET = 0
)
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        p8k,           // TC.P: 8 Kbyte pages

	// lookup
	input  logic [31:0] la,
	input  logic        fc2,
	input  logic  [3:0] set_m,         // (EXT_SET) la's set
	output logic        hit,
	output atce_t       ent,
	output logic  [3:0] hv,            // the hit way (one-hot), and every way's
	output logic [19:0] wpa [4],       // physical page: a caller's tag compares
	                                   // run against all four in parallel
	// a second lookup port (the DMU's DC1: its own copy of the entries, so
	// no address multiplexer in front of it): its set is given registered
	input  logic [31:0] la_d,
	input  logic        fc2_d,
	input  logic  [3:0] set_d,
	output logic  [3:0] hv_d,
	output logic [19:0] wpa_d [4],

	// install (one cycle)
	input  logic        wr,
	input  logic [31:0] wla,
	input  logic        wfc2,
	input  atce_t       went,
	input  logic        wsame,         // (SAME_SEARCH 0) the page is in the set ...
	input  logic  [3:0] wsame_hv,      // ... in this way (one-hot)

	// flushes (one cycle each)
	input  logic        flush_all,      // PFLUSHA / PFLUSHAN
	input  logic        flush_page,     // PFLUSH / PFLUSHN (la/fc2 = the page)
	input  logic        flush_nonglobal // the N variants spare global entries
);

typedef struct packed {
	logic        fc2;
	logic [19:0] lpn;     // LA31-12 (bit 12 ignored with 8K pages)
	atce_t       e;
} slot_t;

localparam int SW = $bits(slot_t);

logic [3:0] v   [16];     // valid, per set and way
logic [3:0] g   [16];     // global
logic [1:0] rr;           // replacement counter

wire [3:0] lset = EXT_SET ? set_m : p8k ? la[16:13] : la[15:12];
wire [3:0] wset = p8k ? wla[16:13] : wla[15:12];

slot_t s [4];
slot_t ws [4];
// synthesis translate_off
logic [$bits(slot_t)-1:0] wqs [4];      // SAME_SEARCH 0: the simulation's copy
// synthesis translate_on
slot_t sd [4];

genvar w;
generate
	for (w = 0; w < 4; w++) begin : g_way
		logic [SW-1:0] q, wq;
		ap68040_lutram #(.AW(4), .DW(SW)) ram (
			.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
			.wdata({wfc2, wla[31:12], went}), .raddr(lset), .q(q)
		);
		if (SAME_SEARCH) begin : g_ws
			ap68040_lutram #(.AW(4), .DW(SW)) ram_w (
				.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
				.wdata({wfc2, wla[31:12], went}), .raddr(wset), .q(wq)
			);
		end
		else begin : g_nows
			assign wq = '0;
			// synthesis translate_off
			// (the copy, in simulation only: an install that finds its page
			// already there would duplicate it)
			ap68040_lutram #(.AW(4), .DW(SW)) ram_ws (
				.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
				.wdata({wfc2, wla[31:12], went}), .raddr(wset), .q(wqs[w])
			);
			// synthesis translate_on
		end
		logic [SW-1:0] dq;
		ap68040_lutram #(.AW(4), .DW(SW)) ram_d (
			.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
			.wdata({wfc2, wla[31:12], went}), .raddr(set_d), .q(dq)
		);
		assign s[w]  = q;
		assign ws[w] = wq;
		assign sd[w] = dq;
	end
endgenerate

function automatic logic tag_eq(input slot_t t, input logic [31:0] a, input logic f2,
                                input logic p8);
	tag_eq = (t.fc2 == f2) && (t.lpn[19:1] == a[31:13]) && (p8 || (t.lpn[0] == a[12]));
endfunction

always_comb for (int i = 0; i < 4; i++) wpa[i] = s[i].e.pa;
always_comb
	for (int i = 0; i < 4; i++) begin
		hv_d[i]  = v[set_d][i] && tag_eq(sd[i], la_d, fc2_d, p8k);
		wpa_d[i] = sd[i].e.pa;
	end
always_comb begin
	for (int i = 0; i < 4; i++) hv[i] = v[lset][i] && tag_eq(s[i], la, fc2, p8k);
	hit = |hv;
	// at most one way holds a page (install reuses its way): AND-OR, no
	// priority chain
	ent = '0;
	for (int i = 0; i < 4; i++) ent = ent | (hv[i] ? s[i].e : '0);
end
// synthesis translate_off
always_ff @(posedge clk)
	if (nreset && !$onehot0(hv)) begin
		$display("FAIL: ATC: more than one way hits (set %0d, ways %b)", lset, hv);
		$finish;
	end
// synthesis translate_on

// install way: the same page if present (an M-bit update), else invalid, else rr
logic [1:0] wway;
always_comb begin
	logic found;
	found = 1'b0;
	wway  = rr;
	if (SAME_SEARCH) begin
		for (int i = 3; i >= 0; i--)
			if (v[wset][i] && tag_eq(ws[i], wla, wfc2, p8k)) begin wway = 2'(i); found = 1'b1; end
	end
	else if (wsame) begin
		found = 1'b1;
		wway  = wsame_hv[3] ? 2'd3 : wsame_hv[2] ? 2'd2 : wsame_hv[1] ? 2'd1 : 2'd0;
	end
	if (!found) begin
		for (int i = 3; i >= 0; i--) if (!v[wset][i]) wway = 2'(i);
	end
end

// synthesis translate_off
always_ff @(posedge clk)
	if (!SAME_SEARCH && wr)
		for (int i = 0; i < 4; i++)
			if ((v[wset][i] && tag_eq(slot_t'(wqs[i]), wla, wfc2, p8k)) != (wsame && wsame_hv[i])) begin
				$display("FAIL: ATC: install of %h: the page %s way %0d, the caller says %b/%b",
				         wla, (v[wset][i] && tag_eq(slot_t'(wqs[i]), wla, wfc2, p8k)) ? "is in" : "is not in",
				         i, wsame, wsame_hv);
				$finish;
			end
// synthesis translate_on

// ATC entries survive RSTI (MC68040UM 3.6.1): only configuration clears them
initial begin
	for (int i = 0; i < 16; i++) begin v[i] = 4'd0; g[i] = 4'd0; end
	rr = 2'd0;
end

always_ff @(posedge clk) begin
	if (wr) begin
		v[wset][wway] <= 1'b1;
		g[wset][wway] <= went.g;
		if (v[wset] == 4'hF) rr <= rr + 2'd1;
	end
	if (flush_all)
		for (int i = 0; i < 16; i++)
			v[i] <= flush_nonglobal ? (v[i] & g[i]) : 4'd0;
	if (flush_page)
		for (int i = 0; i < 4; i++)
			if (v[lset][i] && tag_eq(s[i], la, fc2, p8k) &&
			    !(flush_nonglobal && g[lset][i]))
				v[lset][i] <= 1'b0;
end

endmodule
