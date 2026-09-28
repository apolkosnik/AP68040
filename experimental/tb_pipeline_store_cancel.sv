// A younger store must not escape when an interrupt cancels it at older WB.
`timescale 1ns/1ps
module tb_pipeline_store_cancel;
    reg clk=0; always #5 clk=~clk;
    reg nreset=0, ce=1, kill=0, valid=0, retire_ready=1;
    reg [15:0] opcode=0;
    wire ready,idle,req,write;
    wire [31:0] addr,data;
    integer requests=0, cycles=0;
    ap040_pipeline_integer #(.ENABLE_STORES(1)) dut(
        .clk(clk),.nreset(nreset),.ce(ce),.flush(1'b0),.kill_younger(kill),.idle(idle),
        .external_dst(32'd0), .read_old_dst(), .external_a(32'd0),.external_b(32'd0),.external_ccr(5'd0),.read_src(),.read_dst(),
        .in_supported(),.in_valid(valid),.in_ready(ready),.in_pc(32'd0),.in_opcode(opcode),
        .retire_ready(retire_ready),.retire_valid(),.retire_pc(),.retire_opcode(),
        .retire_we(),.retire_dst(),.retire_data(),.retire_ccr(),
        .load_req(req),.load_write(write),.load_addr(addr),.load_wdata(data),.load_ccr(),
        .load_size(),.load_pc(),.load_opcode(),.load_ack(req),.load_fault(1'b0),.load_data(32'd0),
        .retire_fault(),.retire_fault_addr(),.fallback_valid(),.fallback_pc(),.fallback_opcode());
    always @(posedge clk) if(nreset) begin
        cycles=cycles+1;
        if(cycles>200) $fatal(1,"timeout");
        if(req) begin
            requests=requests+1;
            $fatal(1,"cancelled younger store escaped addr=%h data=%h",addr,data);
        end
    end
    task send(input [15:0] op);
        begin
            @(negedge clk);opcode=op;valid=1;
            do @(posedge clk); while(!ready);
            @(negedge clk);valid=0;
        end
    endtask
    initial begin
        repeat(3) @(negedge clk);nreset=1;
        send(16'h7040); // MOVEQ #64,D0
        send(16'h2040); // MOVEA.L D0,A0
        send(16'h72ff); // MOVEQ #-1,D1
        send(16'h7600); // Initialize D3's physical RAM entry before checking it.
        wait(idle);@(negedge clk);retire_ready=0;
        send(16'h7405); // Older MOVEQ #5,D2 remains in WB.
        send(16'h20c1); // Younger MOVE.L D1,(A0)+ must not issue.
        send(16'h7607); // Younger MOVEQ #7,D3 must be discarded too.
        repeat(6) @(negedge clk);
        if(!dut.wb_v || !dut.ex_store || !dut.id_v) $fatal(1,"missing overlap");
        kill=1;retire_ready=1;
        #1;if(req) $fatal(1,"store offer escaped cancellation");
        @(posedge clk);@(negedge clk);kill=0;
        wait(idle);repeat(2) @(negedge clk);
        if(requests!=0 || dut.private_state.regfile.bank_a[8]!==32'h40 ||
           dut.private_state.regfile.bank_a[2]!==32'd5 ||
           dut.private_state.regfile.bank_a[3]!==32'd0)
            $fatal(1,"incorrect cancellation/older commit");
        $display("STORE CANCEL PASS: older WB commits; younger store and register are cancelled");
        $finish;
    end
endmodule
