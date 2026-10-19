`timescale 1ns/1ps

module tb_ap040_bus_timeout;
	reg clk = 0;
	reg nreset = 0;
	reg req = 0;
	reg complete = 0;
	wire berr;
	integer errors = 0;
	integer i;

	always #5 clk = ~clk;

	ap040_bus_timeout #(.COUNTER_BITS(4)) dut (
		.clk(clk), .nreset(nreset), .req(req), .complete(complete), .berr(berr)
	);

	initial begin
		repeat (3) @(posedge clk);
		nreset = 1;
		req = 1;

		// A legitimate wait shorter than 2^4 clocks must not fault.
		for (i = 0; i < 15; i = i + 1) begin
			@(posedge clk);
			if (berr) begin
				$display("FAIL: premature berr at wait cycle %0d", i);
				errors = errors + 1;
			end
		end

		// A completion at the boundary resets the watchdog.
		@(negedge clk);
		complete = 1;
		@(posedge clk);
		@(negedge clk);
		if (berr) begin
			$display("FAIL: completion did not suppress berr");
			errors = errors + 1;
		end
		complete = 0;

		// With no completion, berr asserts and remains level-active until
		// the failed request is released by the bus adapter.
		for (i = 0; i < 18 && !berr; i = i + 1) @(posedge clk);
		if (!berr) begin
			$display("FAIL: missing timeout berr");
			errors = errors + 1;
		end
		repeat (3) begin
			@(posedge clk);
			if (!berr) begin
				$display("FAIL: berr did not remain asserted while req was held");
				errors = errors + 1;
			end
		end
		req = 0;
		@(posedge clk);
		@(negedge clk);
		if (berr) begin
			$display("FAIL: berr did not clear when req fell");
			errors = errors + 1;
		end

		if (errors == 0) $display("ALL TESTS PASSED");
		else             $display("TEST FAILED with %0d errors", errors);
		$finish;
	end
endmodule
