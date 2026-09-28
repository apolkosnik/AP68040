//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25)                   //
//                                                                          //
// ap040_pipe_cache_arr.v - the instruction and data caches' block RAMs     //
//                                                                          //
// Storage only, no policy: the arrays of doc_AP040_PIPELINE_CACHES.md,     //
// "Storage", as the IMU and DMU will drive them. Both caches: 64 sets x 4  //
// ways x 16-byte lines, a 22-bit physical tag (PA31-PA10) per line. Valid, //
// dirty and replacement state live in the units' flops, not here.          //
//                                                                          //
// Instruction cache (ap040_pipe_icache_arr)                                //
//   data   one simple dual-port RAM per way, 128 x 64: a half-line          //
//          (PA3 selects it) per read, so a hit gives the fetch four words; //
//          the write port is the line fill's alone.                        //
//   tags   one 88-bit row per set, all four ways' tags: simple dual-port,  //
//          read by lookups; a fill reads the row through that same port   //
//          (idle while the unit waits on its own miss) and writes it back  //
//          with the new way's tag.                                         //
//   snoop  a copy of the tag rows, written with them, read by snoops so   //
//          a snoop never takes the lookup port.                            //
//                                                                          //
// Data cache (ap040_pipe_dcache_arr)                                       //
//   data   one true dual-port RAM per way, 256 x 32 with byte enables:     //
//          port A the pipeline's lookups and store hits, port B fills and //
//          push reads.                                                     //
//   tags   one 88-bit row per set, true dual-port: port A lookups, port B  //
//          the fill's and maintenance's reads and writes.                  //
//   snoop  as the instruction cache's.                                     //
//                                                                          //
// Every read is registered: the word is there the cycle after its address. //
//--------------------------------------------------------------------------//

module ap040_pipe_icache_arr
(
	input             clk,

	// lookup: set and half-line, read every cycle
	input       [5:0] l_set,
	input             l_half,
	output    [255:0] l_data,     // {way3, way2, way1, way0}, 64 bits each
	output     [87:0] l_tags,     // {way3, way2, way1, way0}, 22 bits each

	// fill: a half-line into one way; the tag row back
	input       [5:0] f_set,
	input             f_half,
	input       [1:0] f_way,
	input      [63:0] f_data,
	input             f_data_we,
	input      [87:0] f_tags,
	input             f_tags_we,

	// snoop: the copy's row for a set
	input       [5:0] s_set,
	output     [87:0] s_tags
);

genvar w;
generate
for (w = 0; w < 4; w = w + 1) begin : way
	ap040_pipe_sdpram #(.addr_width(7), .data_width(64)) data
	(
		.clock     (clk),
		.wraddress ({f_set, f_half}),
		.data      (f_data),
		.wren      (f_data_we && (f_way == w)),
		.rdaddress ({l_set, l_half}),
		.q         (l_data[w*64 +: 64])
	);
end
endgenerate

ap040_pipe_sdpram #(.addr_width(6), .data_width(88)) tags
(
	.clock     (clk),
	.wraddress (f_set),
	.data      (f_tags),
	.wren      (f_tags_we),
	.rdaddress (l_set),
	.q         (l_tags)
);

ap040_pipe_sdpram #(.addr_width(6), .data_width(88)) snoop
(
	.clock     (clk),
	.wraddress (f_set),
	.data      (f_tags),
	.wren      (f_tags_we),
	.rdaddress (s_set),
	.q         (s_tags)
);

endmodule

module ap040_pipe_dcache_arr
(
	input             clk,

	// port A: lookups (all four ways at {set, longword}) and store hits
	input       [7:0] a_addr,     // {set, longword}
	output    [127:0] a_data,     // {way3, way2, way1, way0}
	input       [1:0] a_way,
	input      [31:0] a_wdata,
	input       [3:0] a_be,
	input             a_we,

	// port B: fills (one longword into one way) and push reads
	input       [7:0] b_addr,
	output    [127:0] b_data,
	input       [1:0] b_way,
	input      [31:0] b_wdata,
	input             b_we,

	// tag rows: port A lookups, port B the fill's and maintenance's
	input       [5:0] ta_set,
	output     [87:0] ta_tags,
	input       [5:0] tb_set,
	output     [87:0] tb_tags,
	input      [87:0] tb_wtags,
	input             tb_we,

	// snoop copy
	input       [5:0] s_set,
	output     [87:0] s_tags
);

genvar w;
generate
for (w = 0; w < 4; w = w + 1) begin : way
	ap040_pipe_tdpram_be #(.addr_width(8), .data_width(32)) data
	(
		.clock     (clk),
		.address_a (a_addr),
		.data_a    (a_wdata),
		.byteena_a (a_be),
		.wren_a    (a_we && (a_way == w)),
		.q_a       (a_data[w*32 +: 32]),
		.address_b (b_addr),
		.data_b    (b_wdata),
		.byteena_b (4'b1111),
		.wren_b    (b_we && (b_way == w)),
		.q_b       (b_data[w*32 +: 32])
	);
end
endgenerate

ap040_pipe_tdpram_be #(.addr_width(6), .data_width(88)) tags
(
	.clock     (clk),
	.address_a (ta_set),
	.data_a    (88'd0),
	.byteena_a (11'h7FF),
	.wren_a    (1'b0),
	.q_a       (ta_tags),
	.address_b (tb_set),
	.data_b    (tb_wtags),
	.byteena_b (11'h7FF),
	.wren_b    (tb_we),
	.q_b       (tb_tags)
);

ap040_pipe_sdpram #(.addr_width(6), .data_width(88)) snoop
(
	.clock     (clk),
	.wraddress (tb_set),
	.data      (tb_wtags),
	.wren      (tb_we),
	.rdaddress (s_set),
	.q         (s_tags)
);

endmodule
