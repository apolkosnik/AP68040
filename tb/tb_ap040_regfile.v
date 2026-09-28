`timescale 1ns/1ps

// Compare the public architectural view with independent flip-flop storage.
// +poison makes the pending RAM word unknown until its write commits: no
// read port may depend on that word while the bypass is responsible for it.
// This is an RTL collision-isolation test, not a vendor-primitive timing model.
module tb_ap040_regfile;
reg clk = 0, ce = 0, nreset = 0;
reg sr_s = 0, sr_m = 0, we = 0, aux_we = 0;
reg [3:0] waddr = 0, raddr_a = 0, raddr_b = 0, raddr_c = 0, raddr_d = 0, raddr_e = 0, raddr_f = 0;
reg [31:0] wdata = 0, aux_wdata = 0;
reg [1:0] aux_sel = 0;
wire [31:0] rdata_a, rdata_b, rdata_c, rdata_d, rdata_e, rdata_f, usp_q, isp_q, msp_q;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_a0, dbg_a7;
ap040_regfile #(.EXTRA_READS(1)) dut (.*);

reg [31:0] ref_regs [0:14];
reg [31:0] ref_sp [0:2];
wire [1:0] selected_sp = !sr_s ? 2'd0 : sr_m ? 2'd2 : 2'd1;
integer r;
always @(posedge clk) begin
    if (!nreset) begin
        for (r = 0; r < 15; r = r + 1) ref_regs[r] <= 0;
        for (r = 0; r < 3; r = r + 1) ref_sp[r] <= 0;
    end else if (ce) begin
        if (we) begin
            if (waddr == 15) ref_sp[selected_sp] <= wdata;
            else ref_regs[waddr] <= wdata;
        end
        if (aux_we) ref_sp[(aux_sel < 2) ? aux_sel : 2] <= aux_wdata;
    end
end

function [31:0] expected;
    input [3:0] addr;
    begin
        expected = (addr == 15) ? ref_sp[selected_sp] : ref_regs[addr];
    end
endfunction

integer ticks = 0, port_checks = 0, poisoned = 0;
reg poison;
task check_all;
    integer aidx;
    begin
        for (aidx = 0; aidx < 16; aidx = aidx + 1) begin
            raddr_a = aidx[3:0]; raddr_b = 15 - aidx;
            raddr_c = aidx[3:0] ^ 4'd5; raddr_d = aidx[3:0] ^ 4'd10; raddr_e = aidx[3:0] ^ 4'd3;
            #1;
            if (rdata_a !== expected(raddr_a) || rdata_b !== expected(raddr_b)) begin
                $display("tick=%0d ce=%b reset=%b pending=%b/%h raddr=%h/%h got=%h/%h expected=%h/%h",
                         ticks, ce, nreset, dut.pend_we, dut.pend_waddr,
                         raddr_a, raddr_b, rdata_a, rdata_b,
                         expected(raddr_a), expected(raddr_b));
                $fatal(1, "TEST FAILED: architectural register read");
            end
            if ({dbg_d0, dbg_d1, dbg_d2, dbg_a0, dbg_a7, usp_q, isp_q, msp_q} !==
                {ref_regs[0], ref_regs[1], ref_regs[2], ref_regs[8],
                 ref_sp[selected_sp], ref_sp[0], ref_sp[1], ref_sp[2]})
                $fatal(1, "TEST FAILED: debug/stack-pointer view at tick %0d", ticks);
            if (rdata_c !== expected(raddr_c) || rdata_d !== expected(raddr_d))
                $fatal(1, "TEST FAILED: extra read ports at tick %0d", ticks);
            if (rdata_e !== expected(raddr_e)) $fatal(1,"TEST FAILED: fifth read port");
            port_checks = port_checks + 5;
        end
    end
endtask

task tick;
    begin
        // Input writes must not leak into the old value before the edge.
        if (ticks != 0) check_all;
        #1; clk = 1;
        #1;
        if (poison && dut.pend_we) begin
            dut.bank_a[dut.pend_waddr] = 32'hxxxxxxxx;
            dut.bank_b[dut.pend_waddr] = 32'hxxxxxxxx;
            dut.extra_reads.bank_c[dut.pend_waddr] = 32'hxxxxxxxx;
            dut.extra_reads.bank_d[dut.pend_waddr] = 32'hxxxxxxxx;
            dut.extra_reads.bank_e[dut.pend_waddr] = 32'hxxxxxxxx;
            poisoned = poisoned + 1;
        end
        check_all;
        clk = 0;
        ticks = ticks + 1;
    end
endtask

reg [31:0] rng = 32'h68040bad;
integer i, mode;
initial begin
    poison = $test$plusargs("poison");
    // Deliberately blind both bypass matches as a negative test control.
    if ($test$plusargs("disable_bypass")) begin
        force dut.hit_a = 1'b0;
        force dut.hit_b = 1'b0;
    end
    if ($test$plusargs("disable_extra_bypass")) begin
        force dut.extra_reads.hit_c = 1'b0;
        force dut.extra_reads.hit_d = 1'b0;
    end
    if ($test$plusargs("disable_fifth_bypass")) force dut.extra_reads.hit_e = 0;
    tick; nreset = 1; ce = 1; we = 1;
    for (i = 0; i < 15; i = i + 1) begin
        waddr = i[3:0]; wdata = 32'h10203040 + i; tick;
    end
    // Successive replacements of the same pending address, then different
    // addresses; all debug taps must follow the architectural write edge.
    for (i = 0; i < 64; i = i + 1) begin
        waddr = (i < 32) ? 0 : (i % 15); wdata = 32'hfedcba98 ^ i; tick;
    end
    // A pending write must survive disabled clocks, and must not be mistaken
    // for the changing input write while ce=0. Reset must cancel it even then.
    ce = 0; waddr = 14; wdata = 32'hdeadbeef;
    repeat (4) tick;
    nreset = 0; tick; nreset = 1; tick; ce = 1; tick;
    // Drain a pending integer write while writing A7 or an auxiliary SP.
    for (mode = 0; mode < 4; mode = mode + 1) begin
        sr_s = mode[1]; sr_m = mode[0];
        waddr = 15; wdata = 32'h80000000 + mode; tick;
        we = 0; aux_we = 1; aux_sel = mode[1:0];
        aux_wdata = 32'h90000000 + mode; tick;
        aux_we = 0; we = 1; waddr = 1; wdata = mode; tick;
    end
    for (i = 0; i < 16384; i = i + 1) begin
        rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
        nreset = (i % 317 != 0); ce = rng[0] | rng[1];
        we = rng[2]; aux_we = rng[3] && !we;
        sr_s = rng[4]; sr_m = rng[5]; waddr = rng[9:6]; aux_sel = rng[11:10];
        wdata = rng; aux_wdata = ~rng; tick;
    end
    nreset = 1; ce = 1; we = 0; aux_we = 0;
    tick; tick;
    if (poison && poisoned == 0) $fatal(1, "TEST FAILED: no pending-write coverage");
    $display("ALL TESTS PASSED: %0d cycles, %0d port checks, %0d poisoned pending words",
             ticks, port_checks, poisoned);
    $finish;
end
endmodule
