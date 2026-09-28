// Additional simulation root for the real core's opt-in P1 integration.
`timescale 1ns/1ps
`define C tb_ap040_program.dut.core
module handoff_monitor;
    integer entries = 0, commits = 0, paused = 0, cancelled = 0, loads = 0, stores = 0, peas = 0;
    reg [31:0] expected_pc;
    always @(posedge tb_ap040_program.clk) begin
        if (tb_ap040_program.nreset) begin
            if (`C.pipe_rf_owner && (`C.rf_we || `C.aux_we))
                $fatal(1, "sequencer write during pipeline ownership");
            if (`C.pipe_retire && !`C.pipe_owner) begin
                if (!(`C.pipe_read_retire && `C.pipe_load_active && `C.state == `C.S_MRD &&
                      `C.m_issued && `C.d_ack && !`C.d_err && !`C.irq_pend &&
                      !`C.sr[15] && !`C.sr[14] && !`C.integer_pipeline.wb_v &&
                      `C.integer_pipeline.ex_v && `C.integer_pipeline.ex_load &&
                      !`C.integer_pipeline.ex_store))
                    $fatal(1, "retirement outside normal ownership or successful read completion");
            end
            if (`C.integer_pipeline.fallback_valid)
                $fatal(1, "unsupported opcode admitted to P1");
            if (`C.pipe_owner && !`C.ce) paused = paused + 1;
            if (`C.ce) begin
                if (`C.pipe_owner && `C.pipe_load_req && !`C.pipe_load_active) begin
                    if (`C.integer_pipeline.wb_v && !`C.pipe_retire)
                        $fatal(1, "memory operation passed blocked older retirement");
                    if ((`C.pipe_load_opcode & 16'hfff8) == 16'h4870) peas = peas + 1;
                    if (`C.pipe_load_write) stores = stores + 1;
                    else loads = loads + 1;
                end
                if (`C.pipe_input && `C.pipe_ready) begin
                    if (!`C.pipe_owner) expected_pc = `C.pc_i;
                    entries = entries + 1;
                end
                if (`C.pipe_load_abort)
                    cancelled = cancelled + int'(`C.integer_pipeline.id_v) + int'(`C.integer_pipeline.ex_v);
                if (`C.pipe_cancel)
                    cancelled = cancelled + int'(`C.integer_pipeline.id_v) + int'(`C.integer_pipeline.ex_v);
                if (`C.pipe_retire) begin
                    commits = commits + 1;
                    if (`C.pipe_pc !== expected_pc)
                        $fatal(1, "pipeline retirement PC order changed");
                    expected_pc = `C.pipe_next_pc;
                end
            end
        end
    end
    final $display("HANDOFF entries=%0d commits=%0d paused=%0d cancelled=%0d loads=%0d stores=%0d peas=%0d", entries, commits, paused, cancelled, loads, stores, peas);
endmodule
`undef C
