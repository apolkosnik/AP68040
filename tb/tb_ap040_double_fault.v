// Directed MC68040 double-bus-fault tests.  Access faults during reset,
// exception processing, or RTE internal-state loading must halt the processor.
`timescale 1ns/1ps

module tb_ap040_double_fault;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

wire [15:0] data_in;
wire [31:0] addr_out;
wire [15:0] data_write;
wire nwr, nuds, nlds;
wire [1:0] busstate;
wire longword, nresetout;
wire [2:0] fc;
wire debug_busy, debug_fault, debug_halted;
wire [255:0] debug_status;

reg mem_ready;
integer phase;
wire active = busstate != 2'b01;
wire berr = nreset && active &&
            ((phase == 1 && addr_out[15:0] == 16'h0000) ||
             (phase == 2 && busstate == 2'b00 && addr_out[15:0] == 16'h0300) ||
             (phase == 3 && busstate == 2'b10 && addr_out[15:0] == 16'h1000) ||
             (phase == 4 && busstate == 2'b11 && addr_out[15:0] == 16'h0ff8) ||
             (phase == 5 && busstate == 2'b10 && addr_out[15:0] == 16'h0010) ||
             (phase == 6 && busstate == 2'b00 && addr_out[15:0] == 16'h030a) ||
             (phase == 7 && busstate == 2'b00 && addr_out[15:0] == 16'h020a));
wire clkena_in = !active || mem_ready || berr;

// stores are not posted here: the bench bus-errors the exception frame's
// stack write, which a posted (never-faulting) store could not carry
ap040_tg68k_compat #(.AP040_POST_STORES(0)) dut (
	.clk(clk), .nreset(nreset), .cache_allow_all(1'b1),
	.cache_snoop_stb(1'b0), .cache_snoop_addr(32'd0),
	.cache_z2_ena(1'b0),
	.cache_z3_base0(5'd0),
	.cache_z3_ena0(1'b0),
	.cache_z3_base1(4'd0),
	.cache_z3_ena1(1'b0),
	.clkena_in(clkena_in),
	.data_in(data_in), .ipl(3'b111), .ipl_autovector(1'b1), .berr(berr),
	.addr_out(addr_out), .data_write(data_write), .nwr(nwr),
	.nuds(nuds), .nlds(nlds), .busstate(busstate), .longword(longword),
	.nresetout(nresetout), .fc(fc),
	.walker_ack(1'b0), .walker_data(32'd0), .walker_berr(1'b0),
	.cache_data(16'd0), .cache_ack(1'b0),
	.debug_busy(debug_busy), .debug_fault(debug_fault),
	.debug_halted(debug_halted), .debug_status(debug_status)
);

reg [15:0] mem [0:32767];
assign data_in = mem[addr_out[15:1]];

integer i;
integer errors = 0;
integer cycles;
integer fetch_seen [0:7];

always @(posedge clk) begin
	mem_ready <= 0;
	if (nreset && active && !berr && !mem_ready) mem_ready <= 1;
	if (nreset && mem_ready && busstate == 2'b11) begin
		if (!nuds) mem[addr_out[15:1]][15:8] <= data_write[15:8];
		if (!nlds) mem[addr_out[15:1]][7:0]  <= data_write[7:0];
	end
	if (nreset && mem_ready && busstate == 2'b00 && phase == 0 &&
	    addr_out[15:0] >= 16'h0200 && addr_out[15:0] <= 16'h020e)
		fetch_seen[(addr_out[15:0] - 16'h0200) >> 1] =
			fetch_seen[(addr_out[15:0] - 16'h0200) >> 1] + 1;
end

task init_image;
	input integer ph;
	begin
		for (i = 0; i < 32768; i = i + 1) mem[i] = 0;
		// Reset ISP=$1000, PC=$0200.
		mem[0] = 16'h0000; mem[1] = 16'h1000;
		mem[2] = 16'h0000; mem[3] = 16'h0200;
		// Illegal vector -> $0300; bus-error vector -> $0400.
		mem[4] = 16'h0000; mem[5] = 16'h0400;
		mem[8] = 16'h0000; mem[9] = 16'h0300;
		for (i = 0; i < 8; i = i + 1) begin
			mem[(16'h0200 >> 1) + i] = 16'h4e71;
			fetch_seen[i] = 0;
		end
		mem[16'h0200 >> 1] = (ph == 3) ? 16'h4e73 : 16'h4afc;
		mem[16'h020e >> 1] = 16'h60fe;
		mem[16'h0300 >> 1] = 16'h4e71;
		mem[16'h0400 >> 1] = 16'h4e71;
		phase = ph;
	end
endtask

task expect_four_longword_prefetch;
	integer j;
	begin
		init_image(0);
		// A straight-line reset target consumes the prefetched FIFO.  Override
		// the phase's default ILLEGAL with NOP and stop at the last prefetched
		// word; no address in the window may be fetched a second time.
		mem[16'h0200 >> 1] = 16'h4e71;
		nreset = 0;
		repeat (8) @(posedge clk);
		nreset = 1;
		cycles = 0;
		while (!(dut.core.state == 8'd4 && dut.core.pc_i == 32'h0000020e) &&
		       !debug_halted && cycles < 3000) begin
			@(posedge clk);
			cycles = cycles + 1;
		end
		if (debug_halted || cycles >= 3000) begin
			errors = errors + 1;
			$display("FAIL: four-longword reset prefetch did not execute its FIFO");
		end
		for (j = 0; j < 8; j = j + 1) begin
			if (fetch_seen[j] != 1) begin
				errors = errors + 1;
				$display("FAIL: reset prefetch word %0d completed %0d bus reads",
				         j, fetch_seen[j]);
			end
		end
		if (!debug_halted && cycles < 3000)
			$display("PASS: four-longword reset prefetch was consumed without duplicate reads");
	end
endtask

task expect_halt;
	input integer ph;
	input [8*48-1:0] what;
	begin
		init_image(ph);
		nreset = 0;
		repeat (8) @(posedge clk);
		nreset = 1;
		cycles = 0;
		while (!debug_halted && cycles < 2000) begin
			@(posedge clk);
			cycles = cycles + 1;
		end
		if (!debug_halted || !debug_fault) begin
			errors = errors + 1;
			$display("FAIL: %0s did not enter double-fault halt (pc=%h state=%0d)",
			         what, debug_status[31:0], dut.core.state);
		end
		else begin
			repeat (8) @(posedge clk);
			if (debug_busy || active) begin
				errors = errors + 1;
				$display("FAIL: %0s left a bus request active after halt (busy=%b bus=%b)",
				         what, debug_busy, busstate);
			end
			else $display("PASS: %0s halted and bus is quiescent in %0d cycles",
			              what, cycles);
		end
	end
endtask

task expect_warm_reset_mmu_state;
	begin
		// Seed architectural MMU state and one ATC entry after the cold-reset
		// path has run.  RSTI clears only E; it preserves TC.P, the remaining
		// TTR fields, root/status registers, and both ATCs.
		dut.core.tc    = 32'h0000_c000;
		dut.core.itt0  = 32'h1122_e364;
		dut.core.itt1  = 32'h3344_a364;
		dut.core.dtt0  = 32'h5566_c364;
		dut.core.dtt1  = 32'h7788_8364;
		dut.core.urp   = 32'h1234_5600;
		dut.core.srp   = 32'h89ab_cc00;
		dut.core.mmusr = 32'h0000_5a5a;
		// entry 17 = row 4 (bank 0, set 4), way 1: the payload lives in
		// the ATC dpram row's way-1 bit slice, validity in the flop
		dut.mmu.atc_v[17] = 1'b1;
		dut.mmu.atc_ram.mem[4][89:45] = {17'h12345, 20'habcde, 8'hd3};

		nreset = 0;
		repeat (8) @(posedge clk);
		#1;
		if (dut.core.tc !== 32'h0000_4000 ||
		    dut.core.itt0 !== 32'h1122_6364 ||
		    dut.core.itt1 !== 32'h3344_2364 ||
		    dut.core.dtt0 !== 32'h5566_4364 ||
		    dut.core.dtt1 !== 32'h7788_0364 ||
		    dut.core.urp !== 32'h1234_5600 ||
		    dut.core.srp !== 32'h89ab_cc00 ||
		    dut.core.mmusr !== 32'h0000_5a5a) begin
			errors = errors + 1;
			$display("FAIL: warm reset altered retained MMU registers");
		end
		else if (dut.mmu.atc_v[17] !== 1'b1 ||
		         dut.mmu.atc_ram.mem[4][89:45] !==
		         {17'h12345, 20'habcde, 8'hd3}) begin
			errors = errors + 1;
			$display("FAIL: warm reset invalidated or altered an ATC entry");
		end
		else $display("PASS: warm reset preserved MMU state/ATC and cleared E bits");
	end
endtask

initial begin
	mem_ready = 0;
	phase = 0;
	expect_four_longword_prefetch;
	expect_halt(1, "reset-vector access fault");
	expect_halt(2, "first exception-handler fetch fault");
	expect_halt(3, "RTE frame-load access fault");
	expect_halt(4, "exception stack-write fault");
	expect_halt(5, "exception vector-fetch fault");
	expect_halt(6, "later exception-prefetch fault");
	expect_halt(7, "later reset-prefetch fault");
	expect_warm_reset_mmu_state;

	if (errors == 0) $display("ALL TESTS PASSED");
	else $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
