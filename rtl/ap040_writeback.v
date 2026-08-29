//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 2: MOVEQ,           //
// MOVE.L Dn,Dm and register forwarding)                                    //
//                                                                          //
// ap040_writeback.v - WB stage                                            //
//                                                                          //
// This is the pipeline's commit point in the architectural sense -- but    //
// the actual regfile write and CCR update are driven directly off EX's     //
// registered output (exe_*) at the top level (ap040_pipe_core.v), not off  //
// this stage's own output, because the write must happen using the same    //
// signals ap040_ea_fetch.v's forwarding reads agree with (see its header   //
// comment). This stage is a one-cycle-later valid/pc pass-through purely   //
// for debug visibility (confirming an instruction finished committing) --  //
// a code-quality pass (2026-08-21) found the dest/data/writes_reg/flags    //
// fields carried in the original milestone-2 draft had no reader anywhere  //
// (the commit logic reads exe_*, one cycle earlier), so they were dropped. //
// Reintroduce them here if a future milestone needs post-commit visibility //
// of what was written (e.g. exception/trace logging at the commit point)   //
// rather than the pre-commit view exe_* already provides.                  //
//                                                                          //
// ap040_core.v's "pure restart" model (rolling back at most two address-   //
// register updates via u_rec, ap040_core.v:1330-1342) only works today     //
// because exactly one instruction is ever in flight. A pipeline has        //
// several in flight, so once an instruction can fault after touching       //
// architectural state, this stage will need either a hard no-side-effects- //
// before-commit rule or a generalized undo log -- not needed yet (MOVEQ/    //
// MOVE.L register-direct can't fault).                                     //
//--------------------------------------------------------------------------//

module ap040_writeback
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,   // tied 0 at top level: nothing follows WB

	input             exe_valid,
	input      [31:0] exe_pc,

	output            wb_stall,   // to EX: no local stall of its own yet

	output reg        wb_valid,
	output reg [31:0] wb_pc
);

assign wb_stall = stall_in;

always @(posedge clk) begin
	if (!nreset) begin
		wb_valid <= 1'b0;
		wb_pc    <= 32'h0;
	end else if (ce && !stall_in) begin
		wb_valid <= exe_valid;
		wb_pc    <= exe_pc;
	end
end

endmodule
