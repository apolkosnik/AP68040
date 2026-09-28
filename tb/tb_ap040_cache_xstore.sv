// Byte-addressed oracle for posted cross-line cache-store merging.
// Includes tag/set wrap, all crossing sizes, partial residency, CE-frozen
// snoops during the second lookup, and recovery from a partial physical write.
`timescale 1ns/1ps
module tb_ap040_cache_xstore;
    reg clk=0; always #5 clk=~clk;
    reg reset_n=0, ce=1, req=0, wr=0, post=1, ack=0, err=0;
    reg [1:0] size=2;
    reg [31:0] addr=0, wdata=0, rdata=0;
    reg [31:0] hint=0;
    always @(posedge clk) hint<=addr;
    wire done, mreq, mwr;
    wire [31:0] data, ma, mw;
    wire [1:0] ms;
    reg snoop=0; reg [31:0] snoop_addr=0;
    reg error_arm=0;
    reg [7:0] memory[0:65535], expected[0:31];
    integer delay_cycles=0, remaining=0, reads=0, writes=0;
    integer i,j,k,n,b,base,offset,form,warm,mode,lat,cases=0,before_reads;
    reg [31:0] value, wanted;
    ap040_cache dut(
        .clk(clk),.nreset(reset_n),.ce(ce),.ie(1'b1),.de(1'b1),
        .cinv_req(1'b0),.cinv_ic(1'b0),.cinv_dc(1'b0),.cinv_done(),
        .c_req(req),.c_write(wr),.c_instr(1'b0),.c_size(size),
        .c_addr(addr),.c_wdata(wdata),.c_fc(3'd5),.c_nocache(1'b0),
        .c_hint_addr(addr),.c_hint_instr(1'b0),.c_hint_ptag(addr[31:10]),
        .c_ihint_addr(addr),.c_ihint_ptag(addr[31:10]),.c_ihint_match(1'b0),.c_ihold(1'b0),
        .c_hint_match(req && hint==addr),.c_hint_wmatch(1'b0),.c_hint_away(1'b0),
        .c_post_ok_hint(1'b0),.c_post_ok(post),.c_ack(done),.c_rdata(data),
        .m_req(mreq),.m_write(mwr),.m_instr(),.m_size(ms),.m_addr(ma),
        .m_wdata(mw),.m_fc(),.m_ack(ack),.m_rdata(rdata),.m_err(err),
        .m_line_valid(1'b0),.m_line_tag(28'd0),.m_line_data(128'd0),
        .s_stb(snoop),.s_addr(snoop_addr));

    // Completion is a pulse; no second acceptance in its acknowledge cycle.
    always @(posedge clk) begin
        if (ce) begin
            ack<=0; err<=0;
            if (mreq && !ack && !err) begin
                if (remaining < delay_cycles) remaining<=remaining+1;
                else begin
                    remaining<=0;
                    n=ms==0?1:(ms==1?2:4);
                    if (error_arm && mwr) begin
                        // Model a split platform transfer whose first byte
                        // reached RAM before a later beat reported an error.
                        memory[ma]=mw>>(8*(n-1));
                        err<=1;
                    end else begin
                        ack<=1; rdata=0;
                        if(mwr) writes=writes+1; else reads=reads+1;
                        for(b=0;b<n;b=b+1)
                            if(mwr) memory[ma+b]=mw>>(8*(n-b-1));
                            else rdata=(rdata<<8)|memory[ma+b];
                    end
                end
            end else remaining<=0;
        end
    end

    task idle;
        integer guard;
        begin
            guard=0;
            while(dut.cst!=0 || mreq || ack || err) begin
                @(negedge clk); guard=guard+1;
                if(guard>1000) $fatal(1,"cache did not drain state=%0d",dut.cst);
            end
        end
    endtask
    task access(input bit write_op,input [31:0] address,input [1:0] sz,
                input [31:0] payload,output [31:0] result);
        integer guard;
        begin
            @(negedge clk);req=1;wr=write_op;addr=address;size=sz;wdata=payload;
            guard=0;
            while(!(done && ce)) begin
                @(posedge clk);guard=guard+1;
                if(guard>1000) $fatal(1,"request timeout %h state=%0d",address,dut.cst);
            end
            result=data;
            @(negedge clk);req=0;wr=0;
            @(negedge clk);
        end
    endtask
    task setup(input integer address,input integer residency);
        begin
            @(negedge clk);reset_n=0;req=0;wr=0;ce=1;error_arm=0;snoop=0;
            repeat(3) @(negedge clk);
            reset_n=1;
            repeat(600) @(negedge clk);
            base=address;
            for(j=0;j<32;j=j+1) begin
                expected[j]=8'h40+j;memory[base+j]=expected[j];
            end
            if(residency&1) begin access(0,base,2,0,value);idle;end
            if(residency&2) begin access(0,base+16,2,0,value);idle;end
        end
    endtask
    task check_bytes;
        begin
            idle;
            for(k=0;k<32;k=k+4) begin
                wanted={expected[k],expected[k+1],expected[k+2],expected[k+3]};
                access(0,base+k,2,0,value);
                if(value!==wanted) $fatal(1,"cached bytes mismatch at %h got=%h expected=%h",base+k,value,wanted);
                idle;
            end
            for(k=0;k<32;k=k+1)
                if(memory[base+k]!==expected[k]) $fatal(1,"RAM byte mismatch at %h",base+k);
            cases=cases+1;
        end
    endtask
    initial begin
        for(i=0;i<65536;i=i+1) memory[i]=0;
        for(lat=0;lat<2;lat=lat+1)
        for(mode=0;mode<2;mode=mode+1)
        for(warm=0;warm<4;warm=warm+1)
        for(form=0;form<4;form=form+1) begin
            delay_cycles=lat?3:0;post=mode;
            setup(form[0]?'h1ff0:'h5000,warm);
            offset=form==3?15:13+form;
            for(j=0;j<(form==3?2:4);j=j+1)
                expected[offset+j]=32'ha1b2c3d4>>(8*((form==3?2:4)-j-1));
            before_reads=reads;
            access(1,base+offset,form==3?1:2,32'ha1b2c3d4,value);
            check_bytes;
            if(mode && warm==3 && reads!=before_reads)
                $fatal(1,"posted hit invalidated resident lines unnecessarily");
        end
        // Choose different hit ways for the two rows, and preserve every
        // unrelated resident way instead of invalidating either entire set.
        post=1;delay_cycles=3;setup('h5000,0);
        for(i=1;i<=3;i=i+1) begin
            access(0,base+i*4096,2,0,value);idle;
            if(i<=2) begin access(0,base+i*4096+16,2,0,value);idle;end
        end
        access(0,base,2,0,value);idle;
        access(0,base+16,2,0,value);idle;
        expected[14]='ha1;expected[15]='hb2;expected[16]='hc3;expected[17]='hd4;
        before_reads=reads;
        access(1,base+14,2,32'ha1b2c3d4,value);
        check_bytes;
        for(i=1;i<=3;i=i+1) begin
            access(0,base+i*4096,2,0,value);idle;
            wanted={memory[base+i*4096],memory[base+i*4096+1],memory[base+i*4096+2],memory[base+i*4096+3]};
            if(value!==wanted) $fatal(1,"unrelated first-row way corrupted");
            if(i<=2) begin
                access(0,base+i*4096+16,2,0,value);idle;
                wanted={memory[base+i*4096+16],memory[base+i*4096+17],memory[base+i*4096+18],memory[base+i*4096+19]};
                if(value!==wanted) $fatal(1,"unrelated second-row way corrupted");
            end
        end
        if(reads!=before_reads) $fatal(1,"unrelated ways were invalidated");
        // A snoop on either affected row during a CE-frozen second lookup.
        for(mode=0;mode<2;mode=mode+1) begin
            post=1;delay_cycles=0;setup('h1ff0,3);
            expected[14]='ha1;expected[15]='hb2;expected[16]='hc3;expected[17]='hd4;
            fork
                begin access(1,base+14,2,32'ha1b2c3d4,value);idle;end
                begin
                    wait(dut.cst==dut.C_XSTORE_LOOK);
                    @(negedge clk);ce=0;
                    memory[base+mode*16+7]='hee;expected[mode*16+7]='hee;
                    snoop_addr=base+mode*16;snoop=1;
                    repeat(2) @(negedge clk);
                    snoop=0;ce=1;
                end
            join
            check_bytes;
        end
        // Sweep invalidations across admission, the posted memory transfer,
        // both merge states and completion. DMA changes an untouched byte,
        // so the oracle is independent of the CPU/DMA ordering.
        for(mode=0;mode<2;mode=mode+1)
        for(offset=0;offset<16;offset=offset+1) begin
            post=1;delay_cycles=3;setup('h5000,3);
            expected[14]='ha1;expected[15]='hb2;expected[16]='hc3;expected[17]='hd4;
            fork
                begin access(1,base+14,2,32'ha1b2c3d4,value);idle;end
                begin
                    repeat(offset+1) @(negedge clk);
                    memory[base+mode*16+7]='hef;expected[mode*16+7]='hef;
                    snoop_addr=base+mode*16;snoop=1;
                    @(negedge clk);snoop=0;
                end
            join
            check_bytes;
        end
        // Posting normally promises non-faulting RAM. Still check that an
        // unexpected downstream error cannot leave either old line hittable.
        post=1;delay_cycles=3;setup('h5000,3);error_arm=1;
        expected[14]='ha1;
        access(1,base+14,2,32'ha1b2c3d4,value);idle;error_arm=0;
        check_bytes;
        $display("XSTORE PASS cases=%0d posted/unposted residency lanes wrap CE snoops partial-error",cases);
        $display("ALL TESTS PASSED");
        $finish;
    end
    initial begin #10000000; $fatal(1,"global timeout");end
endmodule
