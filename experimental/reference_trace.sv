// Extra simulation root alongside the existing, unchanged CPU program bench.
// This is deliberately limited to the straight-line, register-only payload.
// fetch_next/retire_req is not a general retirement interface for branches,
// exceptions, or multi-write instructions.
`timescale 1ns/1ps
`define CORE tb_ap040_program.dut.core
module reference_trace;
    integer fd, count, i, seen = 0, registers = 8;
    reg [1023:0] path;
    reg [31:0] pc_before;
    reg [15:0] opcode_before;
    reg ce_before;
    reg retire_before;
    reg pipeline_before;
    reg drain_before;
    reg [31:0] value;
    initial begin
        if (!$value$plusargs("trace=%s", path) || !$value$plusargs("count=%d", count))
            $fatal(1, "missing reference arguments");
        if ($value$plusargs("registers=%d", registers)) begin end
        fd = $fopen(path, "w");
        if (!fd) $fatal(1, "cannot open reference trace");
    end
    always @(posedge tb_ap040_program.clk) begin
        pc_before = `CORE.pc_i;
        opcode_before = `CORE.ir;
        ce_before = `CORE.ce;
`ifdef AP040_EXPERIMENTAL_PIPELINE
        pipeline_before = `CORE.pipe_owner;
        drain_before = `CORE.state == `CORE.S_NEXT;
        if (pipeline_before) begin
            pc_before = `CORE.pipe_pc;
            opcode_before = `CORE.pipe_opcode;
        end
        retire_before = `CORE.pipe_retire;
`endif
        #1;
`ifdef AP040_EXPERIMENTAL_PIPELINE
        if (!pipeline_before) retire_before = `CORE.retire_req && !drain_before;
`else
        retire_before = `CORE.retire_req;
`endif
        if (tb_ap040_program.nreset && ce_before && retire_before &&
            pc_before >= 32'h400 && pc_before < 32'h400 + 2*count) begin
            $fwrite(fd, "%08x %04x %02x", pc_before, opcode_before, `CORE.sr[4:0]);
            for (i = 0; i < registers; i = i + 1) begin
                // Match the architectural pending-write order of the MLAB RF.
                value = i == 15 ? `CORE.dbg_a7 : (`CORE.regfile.rf_written[i] ? `CORE.regfile.bank_a[i] : 0);
                if (`CORE.regfile.pend_we && `CORE.regfile.pend_waddr == i)
                    value = `CORE.regfile.pend_wdata;
                if (`CORE.rf_we && `CORE.rf_waddr == i) value = `CORE.rf_wdata;
                $fwrite(fd, " %08x", value);
            end
            $fwrite(fd, "\n");
            seen = seen + 1;
            if (seen == count) begin $fclose(fd); $display("REFERENCE TRACE COMPLETE %0d", seen); end
            if (seen > count) $fatal(1, "duplicate reference retirement");
        end
    end
endmodule
`undef CORE
