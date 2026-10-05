//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040_ram.sv - memory primitives written for Quartus inference          //
//                                                                          //
//   ap68040_sdp_be   simple dual port block RAM (M10K): one write port     //
//                    with byte enables, one read port with a registered    //
//                    address and an optionally enabled output register     //
//   ap68040_tdp      true dual port block RAM (M10K), both ports read and  //
//                    write, registered addresses, unregistered outputs     //
//   ap68040_lutram   small RAM with an asynchronous read port (MLAB)       //
//--------------------------------------------------------------------------//

module ap68040_sdp_be #(
	parameter int AW = 6,
	parameter int NB = 16,           // bytes per word (1..4, or a multiple of 4)
	parameter bit OREG = 1           // registered output (one more cycle)
)(
	input  logic              clk,
	input  logic              we,
	input  logic [AW-1:0]     waddr,
	input  logic [NB-1:0]     wbe,
	input  logic [NB*8-1:0]   wdata,
	input  logic [AW-1:0]     raddr,    // sampled here: data the next cycle
	input  logic              oce,      // output register enable
	output logic [NB*8-1:0]   q
);
	// Intel's byte-enabled simple dual port template (Quartus 17 infers
	// byte enables for at most four bytes per memory, so a wider word is
	// built from 32-bit slices).  The read data is registered: a read of
	// the word written in the same cycle returns the old data (the users
	// track that case).
	localparam int SL = (NB > 4) ? 4 : NB;      // bytes per slice
	localparam int NS = (NB + SL - 1) / SL;     // slices
	logic [NB*8-1:0] qr;
	genvar gs, gb;
	generate
		for (gs = 0; gs < NS; gs++) begin : g_sl
			(* ramstyle = "M10K, no_rw_check" *) logic [SL-1:0][7:0] mem [0:(1<<AW)-1];
			for (gb = 0; gb < SL; gb++) begin : g_b
				always_ff @(posedge clk)
					if (we && wbe[gs*SL + gb]) mem[waddr][gb] <= wdata[(gs*SL + gb)*8 +: 8];
			end
			always_ff @(posedge clk) qr[gs*SL*8 +: SL*8] <= mem[raddr];
		end
	endgenerate
	generate
		if (OREG) begin : g_oreg
			always_ff @(posedge clk) if (oce) q <= qr;
		end
		else begin : g_noreg
			assign q = qr;
		end
	endgenerate
endmodule

module ap68040_tdp #(
	parameter int AW = 6,
	parameter int DW = 22
)(
	input  logic          clk,
	input  logic [AW-1:0] addr_a,
	input  logic          we_a,
	input  logic [DW-1:0] wd_a,
	output logic [DW-1:0] q_a,
	input  logic [AW-1:0] addr_b,
	input  logic          we_b,
	input  logic [DW-1:0] wd_b,
	output logic [DW-1:0] q_b
);
	(* ramstyle = "M10K, no_rw_check" *) logic [DW-1:0] mem [0:(1<<AW)-1];
	logic [AW-1:0] ra, rb;
	always_ff @(posedge clk) begin
		if (we_a) mem[addr_a] <= wd_a;
		if (we_b) mem[addr_b] <= wd_b;
		ra <= addr_a;
		rb <= addr_b;
	end
	assign q_a = mem[ra];
	assign q_b = mem[rb];
endmodule

module ap68040_lutram #(
	parameter int AW = 4,
	parameter int DW = 50
)(
	input  logic          clk,
	input  logic          we,
	input  logic [AW-1:0] waddr,
	input  logic [DW-1:0] wdata,
	input  logic [AW-1:0] raddr,
	output logic [DW-1:0] q
);
	(* ramstyle = "MLAB, no_rw_check" *) logic [DW-1:0] mem [0:(1<<AW)-1];
	always_ff @(posedge clk) if (we) mem[waddr] <= wdata;
	assign q = mem[raddr];
endmodule
