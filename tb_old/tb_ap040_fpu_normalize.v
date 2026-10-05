`timescale 1ns/1ps

// White-box unit regression for the three mutually exclusive normalization
// states. The reference shifts one bit at a time, including GRS, and checks
// untouched operands, exponent wrap, next state and clock-enable holding.
module tb_ap040_fpu_normalize;
reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0, ce = 0;
ap040_fpu dut (
    .clk(clk), .nreset(nreset), .ce(ce), .req(1'b0),
    .op_class(3'd0), .opmode(7'd0), .src_fmt(3'd0),
    .src_r(3'd0), .dst_r(3'd0), .din(96'd0),
    .cr_sel(2'd0), .cr_we(1'b0), .cr_wdata(32'd0), .bsun_req(1'b0),
    .ia_we(1'b0), .ia_wdata(32'd0), .fm_sel(3'd0), .fm_we(1'b0),
    .fm_wdata(96'd0), .fsave_ack(1'b0), .frestore_idle(1'b0),
    .frestore_unimp(1'b0), .pend_capture(1'b0), .frestore_cusavepc(8'd0),
    .frestore_et15(1'b0), .frestore_fpt15(1'b0), .frestore_wbt(96'd0),
    .frestore_fpiar(32'd0), .frestore_busy(1'b0), .frestore_cmd1(16'd0),
    .frestore_cmd3(16'd0), .frestore_stag(3'd0), .frestore_dtag(3'd0),
    .frestore_flags(3'd0), .frestore_fpt(96'd0), .frestore_et(96'd0),
    .frestore_grs(3'd0), .frestore_wbte15(1'b0), .fp_reset(1'b0)
);
localparam [4:0] NORM = 2, EXEC = 3, WB = 4, NORM2 = 13,
                 ROUND = 14, RESTORE_N = 20;
integer checks = 0;

task check;
    input integer which;
    input [63:0] m;
    input [2:0] g;
    input [1:0] tag;
    input [17:0] exponent;
    reg [63:0] am, bm;
    reg [16:0] ae, be;
    reg [17:0] ew;
    reg [1:0] atag, btag;
    reg [2:0] rg;
    reg [4:0] state_in, state_out;
    reg [66:0] shifted;
    reg [255:0] held;
    integer lz;
    begin
        am = (which == 0) ? m : 64'h0123456789abcdef;
        bm = (which == 1) ? m : 64'hfedcba9876543210;
        ae = exponent[16:0]; be = exponent[16:0]; ew = exponent;
        atag = tag; btag = tag; rg = g;
        state_in = (which == 0) ? NORM : (which == 1) ? RESTORE_N : NORM2;
        state_out = (which == 0) ? EXEC : (which == 1) ? NORM : ROUND;
        @(negedge clk);
        ce = 0;
        dut.fst = state_in;
        dut.a_m = am; dut.b_m = bm;
        dut.a_e = ae; dut.b_e = be; dut.e_w = ew;
        dut.a_t = atag; dut.b_t = btag;
        // High accumulator bit is deliberately set; it is not normalized.
        dut.acc_hi = {1'b1, m}; dut.grs = rg;
        held = {dut.fst, dut.a_m, dut.b_m, dut.a_e, dut.b_e,
                dut.e_w, dut.a_t, dut.b_t, dut.grs};
        @(posedge clk); #1;
        if (held !== {dut.fst, dut.a_m, dut.b_m, dut.a_e, dut.b_e,
                      dut.e_w, dut.a_t, dut.b_t, dut.grs})
            $fatal(1, "TEST FAILED: normalization advanced with ce=0");

        shifted = {m, (which == 2) ? g : 3'd0};
        lz = 0;
        while (lz < 64 && !shifted[66]) begin
            shifted = shifted << 1;
            lz = lz + 1;
        end
        // The result's GRS-only case is deliberately special in the RTL:
        // the count is based on the mantissa alone, not the 67-bit value.
        if (m == 0) lz = 64;
        if (which == 0 && tag == 0) begin
            if (m == 0) begin atag = 1; ae = 0; end
            else begin am = shifted[66:3]; ae = ae - lz; end
        end else if (which == 1 && tag == 0) begin
            bm = shifted[66:3]; be = be - lz;
        end else if (which == 2) begin
            if (m == 0 && g == 0) begin atag = 1; state_out = WB; end
            else if (m == 0) begin am = {g, 61'd0}; rg = 0; ew = ew - 64; end
            else begin am = shifted[66:3]; rg = shifted[2:0]; ew = ew - lz; end
        end
        @(negedge clk); ce = 1;
        @(posedge clk); #1; ce = 0;
        if ({dut.fst, dut.a_m, dut.b_m, dut.a_e, dut.b_e,
             dut.e_w, dut.a_t, dut.b_t, dut.grs} !==
            {state_out, am, bm, ae, be, ew, atag, btag, rg}) begin
            $display("which=%0d m=%h grs=%h tag=%0d exp=%h lz=%0d",
                     which, m, g, tag, exponent, lz);
            $fatal(1, "TEST FAILED: normalization state/result mismatch");
        end
        checks = checks + 1;
    end
endtask

reg [63:0] rng = 64'h0471_6804_0123_4567;
reg [63:0] mant;
integer n, g, t, s, i;
initial begin
    repeat (2) @(negedge clk);
    nreset = 1;
    for (n = 0; n <= 64; n = n + 1)
        for (g = 0; g < 8; g = g + 1)
            for (t = 0; t < 4; t = t + 1)
                for (s = 0; s < 3; s = s + 1) begin
                    mant = (n == 64) ? 0 : (64'hffffffffffffffff >> n);
                    // Cross zero and both signed working-exponent limits.
                    check(s, mant, g[2:0], t[1:0],
                          (g[1:0] == 0) ? 18'd0 :
                          (g[1:0] == 1) ? 18'h1ffff :
                          (g[1:0] == 2) ? 18'h20000 : 18'h3ffff);
                end
    for (i = 0; i < 4096; i = i + 1) begin
        rng = rng ^ (rng << 13);
        rng = rng ^ (rng >> 7);
        rng = rng ^ (rng << 17);
        check(i % 3, rng >> (i % 65), rng[2:0], rng[4:3], rng[22:5]);
    end
    $display("ALL TESTS PASSED: %0d normalization cases and ce holds", checks);
    $finish;
end
endmodule
