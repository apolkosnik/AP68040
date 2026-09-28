`timescale 1ns/1ps
`define C tb_ap040_program.dut.core
module irq_pea_monitor;
reg injected=0;integer injections=0,kills=0,stores=0;
always @(negedge tb_ap040_program.clk) begin
 if(!tb_ap040_program.nreset) injected=0;
 else if(!injected && `C.pipe_load_req && `C.pipe_load_pc=='h600) begin
  tb_ap040_program.ipl_lvl=2;injected=1;injections=injections+1;
 end
end
always @(posedge tb_ap040_program.clk) if(tb_ap040_program.nreset && `C.ce &&
 `C.pipe_load_launch && `C.pipe_load_write && `C.pipe_load_pc=='h600) stores=stores+1;
always @(posedge tb_ap040_program.clk) if(tb_ap040_program.nreset && `C.ce && `C.pipe_cancel) begin
 if(`C.pipe_pc!='h600)$fatal(1,"IRQ did not stop after PEA: pc=%h",`C.pipe_pc);
 kills=kills+int'(`C.integer_pipeline.id_v)+int'(`C.integer_pipeline.ex_v);
end
final begin
 if(injections!=3 || kills<3 || stores!=3)$fatal(1,"PEA IRQ coverage missing");
 $display("PEA IRQ injections=%0d killed=%0d stores=%0d",injections,kills,stores);
end
endmodule
`undef C
