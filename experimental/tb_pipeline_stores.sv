`timescale 1ns/1ps
module tb_pipeline_stores;
    reg clk = 0;
    always #5 clk = !clk;
    reg kill_younger = 0;
    wire idle;
    reg nreset = 0, ce = 1, flush = 0, in_valid = 0, retire_ready = 1;
    reg [31:0] in_pc = 0;
    reg [15:0] in_opcode = 0;
    wire in_ready, retire_valid, retire_we, fallback_valid;
    wire [31:0] retire_pc, retire_data, fallback_pc;
    wire [15:0] retire_opcode, fallback_opcode;
    wire [3:0] retire_dst;
    wire [4:0] retire_ccr;
    wire in_supported;
    ap040_pipeline_integer #(.ENABLE_STORES(1)) dut (.external_dst(32'd0), .read_old_dst(), .external_a(32'd0), .external_b(32'd0),
        .external_ccr(5'd0), .next_opcode(16'd0), .next_extension(16'd0), .next_valid(1'b0), .next_extension_valid(1'b0), .next_supported(), .external_sp(32'd0), .in_extension(16'd0), .in_extension_valid(1'b0), .in_words(), .retire_next_pc(), .read_src(), .read_dst(), .load_req(load_req), .load_write(load_write), .load_wdata(load_wdata), .load_ccr(), .load_addr(load_addr), .load_size(load_size), .load_pc(load_pc), .load_opcode(),
        .load_ack(load_ack), .load_fault(1'b0), .load_data(32'd0),
        .retire_fault(), .retire_fault_addr(), .empty_after_retire(), .retire_wb_valid(), .retire_branch_taken(),
        .external_alu_selected(1'b0), .external_alu_result(32'd0), .external_alu_flags(5'd0),
        .external_alu_fast_flags(5'd0), .external_alu_fast_ok(1'b0),
        .ex_op(), .ex_size(), .shcnt(), .src(), .b(), .flags_in(), .*);
    wire load_req,load_write;
    wire [31:0] load_wdata,load_addr,load_pc;
    wire [1:0] load_size;
    reg load_ack=0,pending=0;
    reg [99:0] expected_stores[0:2047], held;
    integer req_count=0, waitleft=0, delay=0, paused_ack=0, expected_requests=0;
    reg [1023:0] stores_file;
    initial begin
      if(!$value$plusargs("stores=%s",stores_file)) $fatal(1,"missing stores");
      if($value$plusargs("delay=%d",delay)) begin end
      if(!$value$plusargs("requests=%d",expected_requests)) $fatal(1,"missing request count");
      $readmemh(stores_file,expected_stores);
    end
    reg [15:0] code [0:32767];
    reg [31:0] regs [0:15];
    integer registers = 8;
    integer count, sent = 0, retired = 0, cycles = 0, i, fd;
    integer mode = 0, bubble_cycles = 0, stall_cycles = 0, disabled_cycles = 0;
    reg flushed = 0;
    reg held_input = 0;
    reg [0:0] supported [0:65535];
    reg [15:0] decode_probe;
    reg [1023:0] supported_file;
    reg [1023:0] program_file, trace_file;
    initial begin
        if (!$value$plusargs("program=%s", program_file) ||
            !$value$plusargs("trace=%s", trace_file) ||
            !$value$plusargs("count=%d", count)) $fatal(1, "missing arguments");
        if ($value$plusargs("mode=%d", mode)) begin end
        if ($value$plusargs("registers=%d", registers)) begin end
        $readmemh(program_file, code);
        if (!$value$plusargs("supported=%s", supported_file)) $fatal(1, "missing supported map");
        $readmemh(supported_file, supported);
        force dut.id_opcode = decode_probe;
        for (i = 0; i < 65536; i = i + 1) begin
            decode_probe = i[15:0]; in_opcode = i[15:0];
            #1;
            if (in_supported !== supported[i] || dut.legal !== supported[i]) $fatal(1, "decoder mismatch opcode=%04x", decode_probe);
        end
        release dut.id_opcode;
        fd = $fopen(trace_file, "w");
        if (!fd) $fatal(1, "trace open failed");
        for (i = 0; i < registers; i = i + 1) regs[i] = 0;
        repeat (4) @(negedge clk);
        nreset = 1;
        while (cycles < 100000) begin
            // A cancelled prefix fills all three stages but commits nothing.
            // Restart the stream to prove that no cancelled state leaks out.
            ce = mode != 1 || cycles % 11 != 4;
            retire_ready = (mode == 2 && !flushed) || (mode >= 3 && cycles >= 5000 && !flushed) ? 0 :
                           mode != 1 || (cycles % 13 < 8);
            flush = (mode == 2 && cycles == 5) || (mode == 3 && cycles == 5005);
            kill_younger = mode >= 4 && cycles == 5005;
            if (kill_younger) begin
                // Preserve the old WB, whether it commits now or is blocked.
                sent = retired + (dut.wb_v ? 1 : 0);
                flushed = 1; held_input = 0;
                if (mode == 4) retire_ready = 1;
            end
            if (flush) begin
                sent = retired; flushed = 1; held_input = 0;
            end
            in_valid = !flush && !kill_younger && (held_input || mode != 1 || cycles % 7 != 3);
            in_pc = 32'h400 + 2 * sent;
            in_opcode = sent < count ? code[sent] : 16'h4afc;
            if (!in_valid) bubble_cycles = bubble_cycles + 1;
            if (!retire_ready) stall_cycles = stall_cycles + 1;
            if (!ce) disabled_cycles = disabled_cycles + 1;
            #1; // allow CE/ready changes to settle before accepting an offer
            load_ack=0;
            if(load_req) begin
                if(!load_write) $fatal(1,"unexpected read");
                if(!pending) begin
                    held={load_pc,load_addr,load_wdata,2'b0,load_size};
                    if(held !== expected_stores[req_count]) $fatal(1,"store oracle mismatch index=%0d got=%h expected=%h",req_count,held,expected_stores[req_count]);
                    waitleft=delay; pending=1;
                    if(dut.wb_v && !(retire_ready && ce)) $fatal(1,"store passed older WB");
                end
                if(held !== {load_pc,load_addr,load_wdata,2'b0,load_size}) $fatal(1,"unstable store request");
                if(waitleft==0) begin
                    load_ack=1; pending=0; req_count=req_count+1;
                    if(!ce) paused_ack=paused_ack+1;
                end else waitleft=waitleft-1;
            end
            @(posedge clk);
            held_input = in_valid && !in_ready;
            if (in_valid && in_ready) sent = sent + 1;
            if (retire_valid && retire_ready) begin
                if (retired >= count) $fatal(1, "extra retirement");
                if (retire_we) regs[retire_dst] = retire_data;
                $fwrite(fd, "%08x %04x %02x", retire_pc, retire_opcode, retire_ccr);
                for (i = 0; i < registers; i = i + 1) $fwrite(fd, " %08x", regs[i]);
                $fwrite(fd, "\n");
                retired = retired + 1;
            end
            if (fallback_valid) begin
                if (retired != count || fallback_pc != 32'h400 + 2*count ||
                    fallback_opcode != 16'h4afc) $fatal(1, "fallback ordering");
                if(req_count != expected_requests) $fatal(1,"missing stores");
                $display("STORE ORACLE requests=%0d paused_ack=%0d",req_count,paused_ack);
                $fclose(fd);
                $display("PIPELINE PASS mode=%0d retired=%0d cycles=%0d bubbles=%0d stalls=%0d ce_off=%0d",
                         mode, retired, cycles, bubble_cycles, stall_cycles, disabled_cycles);
                $finish;
            end
            @(negedge clk);
            cycles = cycles + 1;
        end
        $fatal(1, "timeout");
    end
endmodule
