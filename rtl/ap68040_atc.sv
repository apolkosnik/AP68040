//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
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
(
	input  logic        clk,
	input  logic        nreset,
	input  logic        p8k,           // TC.P: 8 Kbyte pages

	// lookup
	input  logic [31:0] la,
	input  logic        fc2,
	output logic        hit,
	output atce_t       ent,

	// install (one cycle)
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

wire [3:0] lset = p8k ? la[16:13]  : la[15:12];
wire [3:0] wset = p8k ? wla[16:13] : wla[15:12];

slot_t s [4];
slot_t ws [4];

genvar w;
generate
	for (w = 0; w < 4; w++) begin : g_way
		logic [SW-1:0] q, wq;
		ap68040_lutram #(.AW(4), .DW(SW)) ram (
			.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
			.wdata({wfc2, wla[31:12], went}), .raddr(lset), .q(q)
		);
		ap68040_lutram #(.AW(4), .DW(SW)) ram_w (
			.clk(clk), .we(wr && wway == 2'(w)), .waddr(wset),
			.wdata({wfc2, wla[31:12], went}), .raddr(wset), .q(wq)
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
	for (int i = 0; i < 4; i++) hv[i] = v[lset][i] && tag_eq(s[i], la, fc2, p8k);
	hit = |hv;
	ent = s[0].e;
	for (int i = 3; i >= 0; i--) if (hv[i]) ent = s[i].e;
end

// install way: the same page if present (an M-bit update), else invalid, else rr
logic [1:0] wway;
always_comb begin
	logic found;
	found = 1'b0;
	wway  = rr;
	for (int i = 3; i >= 0; i--)
		if (v[wset][i] && tag_eq(ws[i], wla, wfc2, p8k)) begin wway = 2'(i); found = 1'b1; end
	if (!found) begin
		logic fi;
		fi = 1'b0;
		for (int i = 3; i >= 0; i--) if (!v[wset][i]) begin wway = 2'(i); fi = 1'b1; end
	end
end

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
