`timescale 1ns/1ps
`include "ap040_defs.svh"

module tb_ap040_alu_arithmetic;
reg [5:0] op = 0, shcnt = 0;
reg [1:0] size = 0;
reg [31:0] a = 0, b = 0;
reg [4:0] flags_in = 0;
wire [31:0] result;
// Fast compare flags also drive same-cycle branch and CAS decisions.
wire [4:0] fast_flags;
wire fast_ok;
wire [4:0] flags_out;
ap040_alu dut (.*);

// Optional differential leg: provide a renamed pre-change ALU module when
// compiling with -DALU_REFERENCE. No snapshot copy is kept in the tree.
`ifdef ALU_REFERENCE
wire [31:0] ref_result;
wire [4:0] ref_flags;
ap040_alu_reference reference_alu (.op(op), .size(size), .shcnt(shcnt),
    .a(a), .b(b), .flags_in(flags_in), .result(ref_result), .flags_out(ref_flags));
`endif

integer checks = 0;
task compare_reference;
    begin
`ifdef ALU_REFERENCE
        if ({result, flags_out} !== {ref_result, ref_flags}) begin
            $display("op=%0d size=%0d a=%h b=%h flags=%h count=%0d",
                     op, size, a, b, flags_in, shcnt);
            $fatal(1, "TEST FAILED: ALU differs from pre-sharing implementation");
        end
`endif
    end
endtask

task check_arithmetic;
    reg [63:0] mask, aa, bb, unsigned_sum;
    reg signed [63:0] sa, sb, signed_sum, limit;
    reg [31:0] expected, arithmetic_result;
    reg [4:0] expected_flags;
    reg extend_op, subtract_op, xin, carry, negative, zero, overflow;
    integer bits;
    begin
        bits = (size == `AP040_SZ_B) ? 8 : (size == `AP040_SZ_W) ? 16 : 32;
        limit = 64'sd1 << (bits - 1);
        mask = (64'd1 << bits) - 1;
        aa = {32'd0, a} & mask;
        bb = {32'd0, b} & mask;
        sa = aa; if (aa >= limit) sa = sa - (64'sd1 << bits);
        sb = bb; if (bb >= limit) sb = sb - (64'sd1 << bits);
        extend_op = (op == `AP040_ALU_ADDX || op == `AP040_ALU_SUBX);
        subtract_op = (op == `AP040_ALU_SUB || op == `AP040_ALU_SUBX ||
                       op == `AP040_ALU_CMP);
        xin = extend_op && flags_in[4];
        if (subtract_op) begin
            unsigned_sum = bb - aa - xin;
            signed_sum = sb - sa - xin;
            carry = (bb < aa + xin);
        end else begin
            unsigned_sum = bb + aa + xin;
            signed_sum = sb + sa + xin;
            carry = (unsigned_sum > mask);
        end
        arithmetic_result = unsigned_sum & mask;
        expected = (op == `AP040_ALU_CMP) ? bb[31:0] : arithmetic_result;
        negative = arithmetic_result[bits-1];
        zero = (arithmetic_result == 0) && (!extend_op || flags_in[2]);
        overflow = (signed_sum < -limit || signed_sum >= limit);
        expected_flags = {(op == `AP040_ALU_CMP) ? flags_in[4] : carry,
                          negative, zero, overflow, carry};
        #1;
        if ((op == `AP040_ALU_ADD || op == `AP040_ALU_SUB ||
             op == `AP040_ALU_CMP) &&
            (!fast_ok || fast_flags !== expected_flags)) begin
            $display("op=%0d size=%0d a=%h b=%h fast=%h expected=%h",
                     op, size, a, b, fast_flags, expected_flags);
            $fatal(1, "TEST FAILED: independent fast arithmetic flags oracle");
        end
        if ({result, flags_out} !== {expected, expected_flags}) begin
            $display("op=%0d size=%0d a=%h b=%h flags=%h got=%h/%h expected=%h/%h",
                     op, size, a, b, flags_in, result, flags_out,
                     expected, expected_flags);
            $fatal(1, "TEST FAILED: independent arithmetic oracle");
        end
        compare_reference;
        checks = checks + 1;
    end
endtask

reg [31:0] rng = 32'h68040a1d;
task randomize_inputs;
    begin
        rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
        a = rng;
        rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
        b = rng;
        rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
        flags_in = rng[4:0]; shcnt = rng[10:5];
    end
endtask

reg [31:0] edges [0:11];
integer ai, bi, f, o, s, i;
initial begin
    // Exhaustive byte operands, both X and sticky-Z states; upper operand
    // bits are deliberately nonzero to verify size masking.
    size = `AP040_SZ_B;
    for (ai = 0; ai < 256; ai = ai + 1)
        for (bi = 0; bi < 256; bi = bi + 1)
            for (f = 0; f < 4; f = f + 1)
                for (o = 1; o <= 5; o = o + 1) begin
                    a = 32'ha5a55a00 | ai; b = 32'h5a5aa500 | bi;
                    flags_in = {f[1], 1'b1, f[0], 2'b11}; op = o[5:0];
                    check_arithmetic;
                end
    edges[0] = 0; edges[1] = 1; edges[2] = 32'hffffffff;
    edges[3] = 32'h7fffffff; edges[4] = 32'h80000000; edges[5] = 32'h80000001;
    edges[6] = 32'h00007fff; edges[7] = 32'h00008000; edges[8] = 32'h0000ffff;
    edges[9] = 32'hffff0000; edges[10] = 32'haaaaaaaa; edges[11] = 32'h55555555;
    // Include reserved size=3: the existing ALU treats it as long.
    for (s = 1; s <= 3; s = s + 1)
        for (ai = 0; ai < 12; ai = ai + 1)
            for (bi = 0; bi < 12; bi = bi + 1)
                for (f = 0; f < 32; f = f + 1)
                    for (o = 1; o <= 5; o = o + 1) begin
                        size = s[1:0]; a = edges[ai]; b = edges[bi];
                        flags_in = f[4:0]; op = o[5:0]; check_arithmetic;
                    end
    for (i = 0; i < 100000; i = i + 1) begin
        randomize_inputs;
        size = i[1:0]; op = 1 + (i % 5); check_arithmetic;
    end
`ifdef ALU_REFERENCE
    // Exercise every other operation and shift count too, so sharing cannot
    // accidentally change otherwise-unrelated output selection or flags.
    for (s = 0; s < 4; s = s + 1)
        for (o = 0; o < 64; o = o + 1)
            for (i = 0; i < 4096; i = i + 1) begin
                randomize_inputs;
                size = s[1:0]; op = o[5:0]; shcnt = i[5:0];
                #1; compare_reference; checks = checks + 1;
            end
`endif
    $display("ALL TESTS PASSED: %0d ALU comparisons", checks);
    $finish;
end
endmodule
