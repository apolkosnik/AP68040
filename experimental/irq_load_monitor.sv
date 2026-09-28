`timescale 1ns/1ps
`define C tb_ap040_program.dut.core
module irq_load_monitor;
reg injected=0;integer injections=0,kills=0;
always @(negedge tb_ap040_program.clk) begin
 if(!tb_ap040_program.nreset) injected=0;
 else if(!injected && `C.pipe_load_req && `C.pipe_load_pc=='h600) begin
  tb_ap040_program.ipl_lvl=2;injected=1;injections=injections+1;
 end
end
always @(posedge tb_ap040_program.clk) if(tb_ap040_program.nreset && `C.ce && `C.pipe_cancel) begin
 if(`C.pipe_pc!='h600)$fatal(1,"IRQ did not stop after the load: pc=%h",`C.pipe_pc);
 kills=kills+int'(`C.integer_pipeline.id_v)+int'(`C.integer_pipeline.ex_v);
end
final begin
 if(injections!=3 || kills<3)$error("load IRQ coverage missing");
 $display("LOAD IRQ injections=%0d killed=%0d",injections,kills);
end
endmodule
`undef C
