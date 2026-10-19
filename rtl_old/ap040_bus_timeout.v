//--------------------------------------------------------------------------//
// AP040 physical-bus watchdog                                              //
//--------------------------------------------------------------------------//

module ap040_bus_timeout
#(
	parameter COUNTER_BITS = 20
)
(
	input  clk,
	input  nreset,
	input  req,
	input  complete,
	output reg berr
);

reg [COUNTER_BITS-1:0] count;

always @(posedge clk) begin
	if (!nreset || !req || complete) begin
		count <= {COUNTER_BITS{1'b0}};
		berr  <= 1'b0;
	end
	else if (!berr) begin
		if (&count)
			berr <= 1'b1;
		else
			count <= count + {{(COUNTER_BITS-1){1'b0}}, 1'b1};
	end
end

endmodule
