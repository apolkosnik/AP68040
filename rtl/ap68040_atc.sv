//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_atc.sv - address translation cache (MC68040UM 3.3)                //
//                                                                          //
// 64 entries, four-way set associative, 16 sets indexed by LA15-12 (4K     //
// pages) or LA16-13 (8K pages).  The tag is FC2 and the logical page; the  //
// entry holds the physical page and the page attributes.  Valid and global //
// bits are flip-flops (PFLUSHA/PFLUSHAN in one cycle); tags and entries    //
// are in block RAM, one true-dual-port RAM per way.                         //
//                                                                          //
// Lookup: the RAM registers la_n, the address the caller will look up the  //
// next cycle; la/fc2 are that cycle's address, compared here with the four //
// ways and the hit way's entry returned.  A lookup whose set the RAM did   //
// not read (la_n was another address) misses: a wrong address can cost a  //
// lookup, never give a hit.  A way an install wrote at the edge its set    //
// was read gives the entry written (the RAM's output is undefined then).   //
//                                                                          //
// Install: two cycles.  wr registers the page and reads its set through   //
// port B; the next cycle picks the way -- an existing entry for the same  //
// page, else the first invalid way, else the way a 2-bit counter points   //
// to (the counter advances on every fill of a full set) -- and writes it.  //
// PFLUSH by page compares through the lookup port: the caller presents    //
// the page as la_n a cycle ahead and as la while flush_page is high.       //
//--------------------------------------------------------------------------//

module ap68040_atc
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        p8k,           // TC.P: 8 Kbyte pages

	// lookup
	input  logic [31:0] la_n,          // next cycle's lookup address (the RAM's)
	input  logic [31:0] la,
	input  logic        fc2,
	output logic        hit,
	output atce_t       ent,
	output logic        stale,         // the RAM did not read this lookup's set

	// install (one cycle; the entry is written the next)
	input  logic        wr,
	input  logic [31:0] wla,
	input  logic        wfc2,
	input  atce_t       went,

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

function automatic logic [3:0] aset(input logic [31:0] a, input logic p8);
	aset = p8 ? a[16:13] : a[15:12];
endfunction

wire [3:0] lset   = aset(la, p8k);
wire [3:0] lset_n = aset(la_n, p8k);

// the install's second cycle
logic        w1_v;
logic [31:0] w1_la;
logic        w1_fc2;
atce_t       w1_e;
wire  [3:0]  w1_set = aset(w1_la, p8k);
logic [1:0]  wway;

slot_t s [4];
slot_t ws [4];
slot_t sx [4];            // s, with a way written as it was read: the entry

// the lookup RAMs' read set, and the way an install wrote at the edge the
// lookup's set was read (with what it wrote)
logic [3:0]  ra;
logic [3:0]  cfw;
slot_t       cfs;
always_ff @(posedge clk) begin
	ra  <= lset_n;
	for (int i = 0; i < 4; i++) cfw[i] <= w1_v && (w1_set == lset_n) && (wway == 2'(i));
	cfs <= {w1_fc2, w1_la[31:12], w1_e};
end
always_comb for (int i = 0; i < 4; i++) sx[i] = cfw[i] ? cfs : s[i];

genvar w;
generate
	for (w = 0; w < 4; w++) begin : g_way
		logic [SW-1:0] q, wq;
		ap68040_tdp #(.AW(4), .DW(SW)) ram (
			.clk(clk),
			.addr_a(lset_n), .we_a(1'b0), .wd_a('0), .q_a(q),
			.addr_b(w1_v ? w1_set : aset(wla, p8k)),
			.we_b(w1_v && wway == 2'(w)),
			.wd_b({w1_fc2, w1_la[31:12], w1_e}), .q_b(wq)
		);
		assign s[w]  = q;
		assign ws[w] = wq;
	end
endgenerate

function automatic logic tag_eq(input slot_t t, input logic [31:0] a, input logic f2,
                                input logic p8);
	tag_eq = (t.fc2 == f2) && (t.lpn[19:1] == a[31:13]) && (p8 || (t.lpn[0] == a[12]));
endfunction

logic [3:0] hv;
always_comb begin
	for (int i = 0; i < 4; i++)
		hv[i] = (ra == lset) && v[lset][i] && tag_eq(sx[i], la, fc2, p8k);
	hit = |hv;
	stale = (ra != lset);
	// at most one way holds a page (install reuses its way): AND-OR, no
	// priority chain
	ent = '0;
	for (int i = 0; i < 4; i++) ent = ent | (hv[i] ? sx[i].e : '0);
end

// install way: the same page if present (an M-bit update), else invalid, else rr
always_comb begin
	logic found;
	found = 1'b0;
	wway  = rr;
	for (int i = 3; i >= 0; i--)
		if (v[w1_set][i] && tag_eq(ws[i], w1_la, w1_fc2, p8k)) begin wway = 2'(i); found = 1'b1; end
	if (!found) begin
		logic fi;
		fi = 1'b0;
		for (int i = 3; i >= 0; i--) if (!v[w1_set][i]) begin wway = 2'(i); fi = 1'b1; end
	end
end

// ATC entries survive RSTI (MC68040UM 3.6.1): only configuration clears them
initial begin
	for (int i = 0; i < 16; i++) begin v[i] = 4'd0; g[i] = 4'd0; end
	rr = 2'd0;
	w1_v = 1'b0;
end

always_ff @(posedge clk) begin
	w1_v <= wr;
	if (wr) begin
		w1_la  <= wla;
		w1_fc2 <= wfc2;
		w1_e   <= went;
	end
	if (w1_v) begin
		v[w1_set][wway] <= 1'b1;
		g[w1_set][wway] <= w1_e.g;
		if (v[w1_set] == 4'hF) rr <= rr + 2'd1;
	end
	if (flush_all)
		for (int i = 0; i < 16; i++)
			v[i] <= flush_nonglobal ? (v[i] & g[i]) : 4'd0;
	if (flush_page)
		for (int i = 0; i < 4; i++)
			if (v[lset][i] && tag_eq(sx[i], la, fc2, p8k) &&
			    !(flush_nonglobal && g[lset][i]))
				v[lset][i] <= 1'b0;
end

// synthesis translate_off
always_ff @(posedge clk) begin
	if (nreset && !$onehot0(hv)) begin
		$display("FAIL: ATC: more than one way hits (set %0d, ways %b)", lset, hv);
		$finish;
	end
	if (nreset && flush_page && ra != lset) begin
		$display("FAIL: ATC: a page flush without its set read (read %0d, set %0d)", ra, lset);
		$finish;
	end
	if (nreset && wr && w1_v) begin
		$display("FAIL: ATC: an install in the previous one's write cycle");
		$finish;
	end
end
// synthesis translate_on

endmodule
