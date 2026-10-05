//--------------------------------------------------------------------------//
// AP040 16-bit bus-adapter request-boundary regression                     //
//                                                                          //
// The Minimig RAM/cache controllers return a level acknowledge and retain  //
// their accepted address/data until cpu_cs drops.  Consequently each word  //
// of a split longword must be a distinct external request with at least one //
// sampled idle cycle between them.  A pulse-only memory model cannot expose //
// this integration contract.                                               //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_defs.svh"

module tb_ap040_bus16_gap;
	reg clk = 0;
	reg nreset = 0;
	always #5 clk = ~clk;

	reg         mem_req = 0;
	reg         mem_write = 0;
	reg         mem_instr = 0;
	reg  [1:0]  mem_size = `AP040_SZ_L;
	reg  [31:0] mem_addr = 0;
	reg  [31:0] mem_wdata = 0;
	reg  [2:0]  mem_fc = `AP040_FC_SUPER_DATA;
	wire        mem_ack;
	wire [31:0] mem_rdata;

	reg  [15:0] data_in = 0;
	wire [31:0] addr_out;
	wire [15:0] data_write;
	wire        nwr, nuds, nlds;
	wire [1:0]  busstate;
	wire        longword;
	wire [2:0]  fc;

	// Model the real cache/controller completion contract: acknowledge is a
	// level, not a pulse, and the accepted request remains latched until the
	// master presents IDLE.  clkena therefore advances the adapter on either
	// an idle clock or an acknowledged external request.
	reg slave_ack = 0;
	reg slave_active = 0;
	reg [15:0] backing [0:15];
	integer accepted = 0;
	integer idle_boundaries = 0;
	integer errors = 0;
	wire clkena_in = (busstate == `AP040_BUS_IDLE) | slave_ack;

	ap040_bus16_adapter dut (
		.clk(clk), .nreset(nreset), .clkena_in(clkena_in),
		.mem_req(mem_req), .mem_berr(1'b0), .mem_write(mem_write),
		.mem_instr(mem_instr), .mem_size(mem_size), .mem_addr(mem_addr),
		.mem_wdata(mem_wdata), .mem_fc(mem_fc), .mem_ack(mem_ack),
		.mem_rdata(mem_rdata), .data_in(data_in), .addr_out(addr_out),
		.data_write(data_write), .nwr(nwr), .nuds(nuds), .nlds(nlds),
		.busstate(busstate), .longword(longword), .fc(fc)
	);

	// One-cycle response latency followed by a held acknowledge.  Writes are
	// committed exactly once when the request is accepted.
	always @(posedge clk) begin
		if (!nreset) begin
			slave_ack <= 0;
			slave_active <= 0;
			data_in <= 0;
		end
		else if (!slave_active && busstate != `AP040_BUS_IDLE) begin
			slave_active <= 1;
			slave_ack <= 1;
			data_in <= backing[addr_out[3:1]];
			accepted = accepted + 1;
			if (busstate == `AP040_BUS_WRITE) begin
				if (!nuds) backing[addr_out[3:1]][15:8] <= data_write[15:8];
				if (!nlds) backing[addr_out[3:1]][7:0]  <= data_write[7:0];
			end
		end
		else if (slave_active && busstate == `AP040_BUS_IDLE) begin
			slave_active <= 0;
			slave_ack <= 0;
			idle_boundaries = idle_boundaries + 1;
		end
	end

	task wait_ack;
		integer timeout;
		begin
			timeout = 0;
			while (!mem_ack && timeout < 100) begin
				@(posedge clk);
				timeout = timeout + 1;
			end
			if (!mem_ack) begin
				$display("FAIL: core transaction timed out");
				errors = errors + 1;
			end
			#1 mem_req = 0;
			repeat (3) @(posedge clk);
		end
	endtask

	initial begin
		backing[0] = 16'h0000;
		backing[1] = 16'h0001;
		backing[2] = 16'hCAFE;
		backing[3] = 16'hBABE;
		backing[4] = 16'hAA11;
		backing[5] = 16'h2233;
		backing[6] = 16'h44BB;
		repeat (4) @(posedge clk);
		nreset = 1;
		repeat (2) @(posedge clk);

		// This is the pointer fetch used by cputest's
		// fadd.p ([D0.w*8]),fp1 case.
		mem_addr = 0;
		mem_size = `AP040_SZ_L;
		mem_write = 0;
		mem_req = 1;
		wait_ack;
		if (mem_rdata !== 32'h0000_0001) begin
			$display("FAIL: split read returned %h, expected 00000001", mem_rdata);
			errors = errors + 1;
		end
		if (accepted != 2) begin
			$display("FAIL: split read produced %0d accepted requests, expected 2", accepted);
			errors = errors + 1;
		end

		// The failing FABS.X operand is odd-addressed.  Its constituent
		// longword reads split byte + word + byte, so all three target
		// requests need independent acknowledge boundaries too.
		accepted = 0;
		mem_addr = 9;
		mem_write = 0;
		mem_req = 1;
		wait_ack;
		if (mem_rdata !== 32'h1122_3344) begin
			$display("FAIL: odd split read returned %h, expected 11223344", mem_rdata);
			errors = errors + 1;
		end
		if (accepted != 3) begin
			$display("FAIL: odd split read produced %0d accepted requests, expected 3", accepted);
			errors = errors + 1;
		end

		accepted = 0;
		mem_addr = 4;
		mem_wdata = 32'h1234_5678;
		mem_write = 1;
		mem_req = 1;
		wait_ack;
		if ({backing[2], backing[3]} !== 32'h1234_5678) begin
			$display("FAIL: split write stored %h%h, expected 12345678",
			         backing[2], backing[3]);
			errors = errors + 1;
		end
		if (accepted != 2) begin
			$display("FAIL: split write produced %0d accepted requests, expected 2", accepted);
			errors = errors + 1;
		end

		accepted = 0;
		mem_addr = 9;
		mem_wdata = 32'hDEAD_BEEF;
		mem_write = 1;
		mem_req = 1;
		wait_ack;
		if ({backing[4], backing[5], backing[6]} !== 48'hAADE_ADBE_EFBB) begin
			$display("FAIL: odd split write stored %h%h%h, expected AADEADBEEFBB",
			         backing[4], backing[5], backing[6]);
			errors = errors + 1;
		end
		if (accepted != 3) begin
			$display("FAIL: odd split write produced %0d accepted requests, expected 3", accepted);
			errors = errors + 1;
		end

		if (errors == 0) $display("ALL TESTS PASSED");
		else             $display("TEST FAILED with %0d errors", errors);
		$finish;
	end
endmodule
