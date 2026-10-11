#!/bin/sh
# Verilate the pipelined-FPU differential bench (tb_fpu_pipe.sv).
set -e
cd "$(dirname "$0")"
WORK=${WORK:-$(cd .. && pwd)/obj}
# REF=old: compare against the committed FPU (git HEAD), renamed;
# REF=<revision>: against that revision's.  An FPU from before the area
# round (no okv_next port) is wrapped with the ports added since, which
# the bench drives idle, and the internals the bench compares.
EXTRA=""
if [ -n "$REF" ]; then
	REV=$REF; [ "$REF" = old ] && REV=HEAD
	if git show "$REV:rtl/fpu/ap040_fpu.v" | grep -q okv_next; then
		git show "$REV:rtl/fpu/ap040_fpu.v" | sed 's/^module ap040_fpu$/module ap040_fpu_ref/' > "$WORK/ap040_fpu_ref.v"
		EXTRA="+define+REF_OLD $WORK/ap040_fpu_ref.v"
	else
		git show "$REV:rtl/fpu/ap040_fpu.v" | sed 's/^module ap040_fpu$/module ap040_fpu_base/' > "$WORK/ap040_fpu_base.v"
		{
			sed -n '/^module ap040_fpu_base$/,/^);/p' "$WORK/ap040_fpu_base.v" | sed '$d' |
				sed 's/^module ap040_fpu_base$/module ap040_fpu_ref/; s/^\(\s*input\s*fp_reset\).*/\1,/'
			cat <<'EOF'
	// the ports added since
	input             din_nice,
	output      [7:0] okv_next,
	input             sel_x,
	input             aok_x,
	input             bok_x,
	input       [6:0] opmode_n,
	input       [2:0] op_class_n,
	input       [2:0] src_fmt_n,
	input             frestore_we,
	input       [3:0] frestore_wf,
	input      [31:0] frestore_wd
);
ap040_fpu_base #(.PIPE(PIPE)) u (.*);
// the internals the bench compares
wire  [7:0] fst      = u.fst;
wire [31:0] fpsr     = u.fpsr;
wire  [7:0] fr_valid = u.fr_valid;
wire        fr_s [0:7];
wire [14:0] fr_e [0:7];
wire [63:0] fr_m [0:7];
genvar gi;
generate for (gi = 0; gi < 8; gi++) begin : g
	assign fr_s[gi] = u.fr_s[gi];
	assign fr_e[gi] = u.fr_e[gi];
	assign fr_m[gi] = u.fr_m[gi];
end endgenerate
endmodule
EOF
		} > "$WORK/ap040_fpu_ref.v"
		EXTRA="+define+REF_OLD $WORK/ap040_fpu_ref.v $WORK/ap040_fpu_base.v"
	fi
fi
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	--top-module tb_fpu_pipe --Mdir "$WORK/obj_fpu_pipe${REF:+_$REF}" -o tb_fpu_pipe \
	-I../rtl/fpu -CFLAGS "-O2 -march=native" \
	../rtl/fpu/ap040_fpu.v $EXTRA tb_fpu_pipe.sv
echo "built $WORK/obj_fpu_pipe${REF:+_$REF}/tb_fpu_pipe"
