`timescale 1ns/1ps
`define C tb_ap040_program.dut.core
module irq_overlap_monitor;
    reg injected = 0;
    integer kills = 0, injections = 0;
    // Drive the public IPL stimulus before a full-pipeline retirement.
    // The core still performs its real synchronizer and mask qualification.
    always @(negedge tb_ap040_program.clk) begin
        if (!tb_ap040_program.nreset) injected = 0;
        else if (!injected && `C.pipe_owner && `C.pc >= 'h606 && `C.pc < 'h680 &&
                 `C.integer_pipeline.id_v && `C.integer_pipeline.ex_v &&
                 `C.integer_pipeline.wb_v) begin
            tb_ap040_program.ipl_lvl = 3'd2;
            injected = 1;
            injections = injections + 1;
        end
    end
    always @(posedge tb_ap040_program.clk) begin
        if (tb_ap040_program.nreset && `C.ce && `C.pipe_cancel)
            kills = kills + int'(`C.integer_pipeline.id_v) + int'(`C.integer_pipeline.ex_v);
    end
    final $display("IRQ OVERLAP injections=%0d killed=%0d", injections, kills);
endmodule
`undef C
