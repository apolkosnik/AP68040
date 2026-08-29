//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 9a: unified L1)     //
//                                                                          //
// ap040_inst_fetch.v - IF stage                                           //
//                                                                          //
// No longer owns any storage of its own -- milestone 9a lifted the inline  //
// ROM out into ap040_pipe_l1.v, a unified dual-port memory shared with     //
// (eventually) the data path, so self-modifying code is coherent by        //
// construction rather than needing an explicit invalidation path (see      //
// ap040_pipe_l1.v's header for why, and what it still doesn't do). This    //
// stage now drives that memory's port A the same way it used to index its  //
// own `rom` array: l1_addr_a is (fetch_pc - PC_RESET) >> 1, computed        //
// COMBINATIONALLY (not registered here) so ap040_pipe_l1.v's own registered //
// read (`q_a <= mem[address_a]`) reproduces EXACTLY the one-cycle           //
// address-to-data latency the old inline `if_opcode <= rom[rom_idx]` had --  //
// this is a like-for-like timing swap, not a new latency stage. IF is       //
// read-only on this port (wren_a/data_a are tied off at the ap040_pipe_core //
// instantiation, not routed through here at all -- nothing downstream ever  //
// needs IF to write memory).                                               //
//                                                                          //
// Turns from a pure linear-index walker into a real PC register (unchanged //
// since milestone 4): redirect_valid/redirect_pc (driven combinationally   //
// by ap040_decode.v the same cycle a branch is recognized -- see its       //
// header comment) select the next PC instead of a plain +2 sequential       //
// advance. "Stop after PROG_WORDS instructions" -- which every existing     //
// testbench's drain-check relies on -- is tracked by a separate issued      //
// counter, decoupled from the PC value itself, so a program that branches   //
// still issues exactly PROG_WORDS instructions total rather than however    //
// many words a purely linear walk to PROG_WORDS would have covered.         //
//--------------------------------------------------------------------------//

module ap040_inst_fetch
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10,
	parameter         L1_AW      = 12   // must match the ap040_pipe_l1.v instance's AW
)
(
	input             clk,
	input             nreset,
	input             ce,
	input             stall_in,     // ID cannot accept a new word this cycle

	input             redirect_valid,
	input      [31:0] redirect_pc,

	// ap040_pipe_l1.v port A -- read-only from here (see header)
	output [L1_AW-1:0] l1_addr_a,
	input       [15:0] l1_rdata_a,

	output reg        if_valid,
	output reg [31:0] if_pc,
	output     [15:0] if_opcode
);

reg [31:0] pc;                       // next word's address, absent a redirect
reg [31:0] issued;                   // instructions issued so far, 0..PROG_WORDS
wire       have_more = (issued < PROG_WORDS);

// The redirect must land on THIS fetch, not merely be scheduled for the
// following one -- otherwise the word at the old (sequential) pc still
// gets fetched first, one cycle late, which is exactly the "poison word
// executes anyway" bug this shape avoids. fetch_pc is what actually gets
// fetched and stored into if_pc/if_opcode this edge; pc (registered) only
// tracks the sequential fallback for cycles with no redirect.
wire [31:0] fetch_pc = redirect_valid ? redirect_pc : pc;

// Combinational -- see header. Assumes fetch_pc stays within the L1's
// address window (PC_RESET..PC_RESET+2*(2**AW-1)), same assumption the old
// inline rom_idx made about staying in-ROM.
assign l1_addr_a = (fetch_pc - PC_RESET) >> 1;

// if_opcode is L1's OWN registered output, not re-registered here -- see
// header for why that reproduces the old timing exactly rather than adding
// a second cycle of latency.
assign if_opcode = l1_rdata_a;

always @(posedge clk) begin
	if (!nreset) begin
		pc        <= PC_RESET;
		issued    <= 32'd0;
		if_valid  <= 1'b0;
		if_pc     <= PC_RESET;
	end else if (ce && !stall_in) begin
		if_valid <= have_more;
		if (have_more) begin
			if_pc     <= fetch_pc;
			pc        <= fetch_pc + 32'd2;
			issued    <= issued + 32'd1;
		end
	end
end

endmodule
