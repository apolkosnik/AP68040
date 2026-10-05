// Exhaustive decoder check: every operation word through the generated
// parallel decoder, against tools/ucode.py's model of the hardware rule
// (most specific pattern, then that entry's EA legality), which
// tools/check_isa.py checks against WinUAE.
module tb_decode;
	import ap68040_pkg::*, ap68040_upkg::*;
	`include "gen/ap68040_dec_pla.svh"
	logic [11:0] expect_t [0:65535];
	function automatic logic [3:0] ea_idx(input logic [2:0] m, input logic [2:0] r);
		if (m != 3'd7) ea_idx = {1'b0, m};
		else if (r <= 3'd4) ea_idx = 4'd7 + {1'b0, r};
		else ea_idx = 4'd15;
	endfunction
	initial begin
		int bad;
		bad = 0;
		$readmemh("build/dec_expect.hex", expect_t);
		for (int op = 0; op < 65536; op++) begin
			logic [NENT-1:0] s;
			pla_t p;
			logic [3:0] i0, i1;
			logic legal;
			logic [11:0] got;
			s  = dec_sel(16'(op));
			p  = ent_attr(sel_index(s), |s);
			i0 = ea_idx(op[5:3], op[2:0]);
			i1 = ea_idx(op[8:6], op[11:9]);
			legal = p.match && (!p.ea0v || (i0 != 4'd15 && p.ea0m[i0])) &&
			        (!p.ea1v || (i1 != 4'd15 && p.ea1m[i1]));
			if ($countones(s) > 1) begin
				bad++;
				if (bad < 10) $display("op %04x: %0d entries selected", op, $countones(s));
			end
			got = (|s) ? {1'b0, legal, 1'b0, 9'(sel_index(s) + 9'd1)} : 12'd0;
			if (got != expect_t[op]) begin
				bad++;
				if (bad < 20) $display("op %04x: got %03x expected %03x", op, got, expect_t[op]);
			end
		end
		if (bad == 0) $display("DECODE PASS: 65536 operation words");
		else $display("DECODE FAIL: %0d mismatches", bad);
		$finish;
	end
endmodule
