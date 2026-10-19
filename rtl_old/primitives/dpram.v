// Parameter names match the hardware primitive's generics (rtl/bram.vhd)
// so cpu_cache_new can bind rdw_mode_a BY NAME and elaborate identically
// against Quartus and against this model.  CPU regressions do not access
// Akiko's battery-backed NVRAM; its MIF contents are not modeled.
module dpram #(parameter addr_width = 8, parameter data_width = 8,
               parameter mem_init_file = "",
               parameter rdw_mode_a = "NEW_DATA_NO_NBE_READ",
               // mixed-port read-during-write: what port A reads in the
               // cycle port B writes the same address.  Silicon is
               // DONT_CARE (the generated altsyncram wrapper says so:
               // READ_DURING_WRITE_MODE_MIXED_PORTS="DONT_CARE").  The
               // default keeps this model's historical old-data answer so
               // nothing already passing changes; a bench that needs the
               // silicon behaviour observable -- the snoop guard exists
               // for exactly this collision -- sets DONT_CARE.
               parameter rdw_mixed = "OLD_DATA") (
	input clock,
	input [addr_width-1:0] address_a,
	input [data_width-1:0] data_a,
	input wren_a,
	output reg [data_width-1:0] q_a,
	input [addr_width-1:0] address_b,
	input [data_width-1:0] data_b,
	input wren_b,
	output reg [data_width-1:0] q_b
);
	reg [data_width-1:0] mem [0:(1<<addr_width)-1];
	// A bench that knows what the lookup is asking for can supply the
	// don't-care word directly.  Left off, the model walks its LFSR, which
	// is honest but weak: a random word MISSES, and a miss is safe, so a
	// control built on it cannot fail.  A directed word that HITS the wrong
	// way is what makes an unsafe lookup observable.
	reg                  poison_en  = 1'b0;
	reg [data_width-1:0] poison_row = {data_width{1'b0}};
	reg [15:0] poison = 16'hACE1;
	function [data_width-1:0] rot;
		input [addr_width-1:0] a;
		integer k;
		begin
			for (k = 0; k < data_width; k = k + 1)
				rot[k] = poison[(k + a) % 16] ^ a[k % addr_width];
		end
	endfunction
	always @(posedge clock) begin
		if (wren_a) mem[address_a] <= data_a;
		if (wren_b) mem[address_b] <= data_b;
		// Read-during-write, same port and mixed port.  An instance
		// declared DONT_CARE has told synthesis it never reads what it
		// is writing, so a collision the RTL claims cannot happen must
		// not quietly return a plausible word.
		//
		// Two-state DONT_CARE.  X is not available here -- it reads back
		// as 0, a word a tag row can legitimately hold -- and no single
		// word is a substitute for "unspecified": the LFSR below SAMPLES
		// particular collision values, it does not demonstrate tolerance
		// of every row.  Deterministic from reset so a failing run
		// reproduces.  For a control that must FAIL, drive poison_row
		// instead; see tb_ap040_cache_snoop.v.
		poison <= {poison[14:0], poison[15] ^ poison[13] ^ poison[12] ^ poison[10]};
		if (wren_a && rdw_mode_a == "DONT_CARE")
			q_a <= poison_en ? poison_row : rot(address_a);
		else if (wren_b && address_b == address_a && rdw_mixed == "DONT_CARE")
			q_a <= poison_en ? poison_row : rot(address_a);
		else                                     q_a <= mem[address_a];
		if (wren_a && address_a == address_b && rdw_mixed == "DONT_CARE")
			q_b <= poison_en ? poison_row : rot(address_b);
		else                                     q_b <= mem[address_b];
	end
endmodule
