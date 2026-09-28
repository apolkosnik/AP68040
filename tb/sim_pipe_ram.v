// Simulation models (for Verilator) of rtl/ap040_pipe/ap040_pipe_ram.vhd's two block RAMs.
// Parameter and port names match the VHDL entities so the same RTL
// elaborates against either. Read addresses are registered and the output
// is the word read at that edge. Where the hardware is DONT_CARE -- a port
// reading the address it writes, or reading an address the other port
// writes that cycle -- the model returns junk from an LFSR, so a unit that
// depended on either would fail here rather than on silicon.

module ap040_pipe_tdpram_be #(parameter addr_width = 8, parameter data_width = 32) (
	input                         clock,
	input      [addr_width-1:0]   address_a,
	input      [data_width-1:0]   data_a,
	input      [data_width/8-1:0] byteena_a,
	input                         wren_a,
	output reg [data_width-1:0]   q_a,
	input      [addr_width-1:0]   address_b,
	input      [data_width-1:0]   data_b,
	input      [data_width/8-1:0] byteena_b,
	input                         wren_b,
	output reg [data_width-1:0]   q_b
);
	reg [data_width-1:0] mem [0:(1<<addr_width)-1];
	reg [31:0] junk = 32'h1234_5678;
	integer i;
	function [data_width-1:0] spread;
		input [31:0] j;
		integer k;
		begin
			for (k = 0; k < data_width; k = k + 1) spread[k] = j[k % 32] ^ k[0];
		end
	endfunction
	always @(posedge clock) begin
		junk <= {junk[30:0], junk[31] ^ junk[21] ^ junk[1] ^ junk[0]};
		// reads first, from the array as it stands before this edge's writes
		if (wren_a || (wren_b && address_b == address_a)) q_a <= spread(junk);
		else                                             q_a <= mem[address_a];
		if (wren_b || (wren_a && address_a == address_b)) q_b <= spread(~junk);
		else                                             q_b <= mem[address_b];
		for (i = 0; i < data_width/8; i = i + 1) begin
			if (wren_a && byteena_a[i]) mem[address_a][i*8 +: 8] <= data_a[i*8 +: 8];
			if (wren_b && byteena_b[i]) mem[address_b][i*8 +: 8] <= data_b[i*8 +: 8];
		end
`ifdef VERILATOR
		if (wren_a && wren_b && address_a == address_b && |(byteena_a & byteena_b))
			$error("ap040_pipe_tdpram_be: both ports write the same bytes of %0d in one cycle", address_a);
`endif
	end
endmodule

module ap040_pipe_sdpram #(parameter addr_width = 8, parameter data_width = 32) (
	input                       clock,
	input      [addr_width-1:0] wraddress,
	input      [data_width-1:0] data,
	input                       wren,
	input      [addr_width-1:0] rdaddress,
	output reg [data_width-1:0] q
);
	reg [data_width-1:0] mem [0:(1<<addr_width)-1];
	reg [31:0] junk = 32'h8765_4321;
	function [data_width-1:0] spread;
		input [31:0] j;
		integer k;
		begin
			for (k = 0; k < data_width; k = k + 1) spread[k] = j[k % 32] ^ k[0];
		end
	endfunction
	always @(posedge clock) begin
		junk <= {junk[30:0], junk[31] ^ junk[21] ^ junk[1] ^ junk[0]};
		if (wren && wraddress == rdaddress) q <= spread(junk);
		else                                q <= mem[rdaddress];
		if (wren) mem[wraddress] <= data;
	end
endmodule
