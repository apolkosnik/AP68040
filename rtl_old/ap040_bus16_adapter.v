//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_bus16_adapter.v - converts internal 32-bit AP040 transfers to the  //
// existing 16-bit Minimig bus contract (TG68K style).                      //
//                                                                          //
// Contract with cpu_wrapper.v:                                             //
//  - all outputs are registered and change only on clkena_in edges         //
//  - busstate: 00 fetch, 01 idle, 10 data read, 11 data write              //
//  - one stable bus request at a time; exactly one qualified completion    //
//    (clkena_in pulse) advances one 16-bit sub-cycle                       //
//  - split transfers insert a sampled IDLE cycle between sub-cycles.  The  //
//    Minimig RAM/cache controllers return a level acknowledge and do not   //
//    accept a new address until their chip-select drops.                   //
//  - nwr is 1 for read, 0 for write; nuds/nlds are active low              //
//                                                                          //
// Transaction splitting (big endian):                                      //
//  - byte: one cycle, UDS for even address, LDS for odd address            //
//  - word: one cycle when even; byte + byte when odd                       //
//  - long: two word cycles when even; byte + word + byte when odd          //
//                                                                          //
// Core handshake: mem_req is held by the core until mem_ack. mem_ack is a  //
// single-cycle pulse asserted in the same cycle busstate returns to idle,  //
// with mem_rdata valid. The core must drop mem_req in the ack cycle.       //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_bus16_adapter
(
	input             clk,
	input             nreset,
	input             clkena_in,

	// core side
	input             mem_req,
	input             mem_berr,     // abort the active external sub-cycle
	input             mem_write,
	input             mem_instr,
	input       [1:0] mem_size,     // AP040_SZ_B/W/L
	input      [31:0] mem_addr,
	input      [31:0] mem_wdata,    // right-aligned by size
	input       [2:0] mem_fc,
	output reg        mem_ack,
	output reg [31:0] mem_rdata,    // right-aligned by size, valid at mem_ack

	// external bus side (TG68K-like)
	input      [15:0] data_in,
	output reg [31:0] addr_out,
	output reg [15:0] data_write,
	output reg        nwr,
	output reg        nuds,
	output reg        nlds,
	output reg  [1:0] busstate,
	output reg        longword,
	output reg  [2:0] fc
);

reg        active;
reg        write_r;
reg        instr_r;
reg        subcycle_gap;
reg [31:0] cur_addr;
reg  [2:0] bytes_left;
reg [31:0] wshift;      // write data, left-aligned, consumed MSB first
reg [31:0] rshift;      // read data, assembled by shifting bytes in

// number of operand bytes for a request
function [2:0] size_bytes;
	input [1:0] size;
	begin
		case (size)
			`AP040_SZ_B: size_bytes = 3'd1;
			`AP040_SZ_W: size_bytes = 3'd2;
			default:     size_bytes = 3'd4;
		endcase
	end
endfunction

// current in-flight sub-cycle consumes this many bytes
wire [2:0] consumed  = cur_addr[0] ? 3'd1 : (bytes_left >= 3'd2) ? 3'd2 : 3'd1;
wire [31:0] next_addr = cur_addr + {29'd0, consumed};
wire  [2:0] next_left = bytes_left - consumed;
wire [31:0] next_wsh  = (consumed == 3'd2) ? {wshift[15:0], 16'd0} : {wshift[23:0], 8'd0};

// first sub-cycle shape of a new request
wire  [2:0] f_bytes = size_bytes(mem_size);
wire [31:0] f_wsh   = (mem_size == `AP040_SZ_B) ? {mem_wdata[7:0], 24'd0} :
                      (mem_size == `AP040_SZ_W) ? {mem_wdata[15:0], 16'd0} :
                                                  mem_wdata;
wire        f_odd  = mem_addr[0];
wire        f_word = !f_odd && (f_bytes >= 3'd2);

always @(posedge clk) begin
	if (!nreset) begin
		active     <= 0;
		mem_ack    <= 0;
		busstate   <= `AP040_BUS_IDLE;
		nwr        <= 1;
		nuds       <= 1;
		nlds       <= 1;
		longword   <= 0;
		write_r    <= 0;
		instr_r    <= 0;
		subcycle_gap <= 0;
		bytes_left <= 0;
		addr_out   <= 0;
		data_write <= 0;
		fc         <= 0;
		cur_addr   <= 0;
		wshift     <= 0;
		rshift     <= 0;
		mem_rdata  <= 0;
	end
	else if (clkena_in) begin
		mem_ack <= 0;

		if (active && mem_berr) begin
			// A physical bus error completes neither the current 16-bit
			// sub-cycle nor the enclosing core transaction.  The core samples
			// berr on this same qualified edge and builds the format-$7 frame;
			// release the TG68K request here so a subsequent exception-vector
			// fetch cannot inherit the failed cycle.
			active   <= 0;
			busstate <= `AP040_BUS_IDLE;
			nwr      <= 1;
			nuds     <= 1;
			nlds     <= 1;
			longword <= 0;
			subcycle_gap <= 0;
		end
		else if (!active) begin
			// idle: busstate is 01, so the wrapper keeps clkena high
			if (mem_req && !mem_ack) begin
				active     <= 1;
				write_r    <= mem_write;
				instr_r    <= mem_instr;
				subcycle_gap <= 0;
				cur_addr   <= mem_addr;
				bytes_left <= f_bytes;
				wshift     <= f_wsh;
				rshift     <= 0;
				fc         <= mem_fc;
				longword   <= (mem_size == `AP040_SZ_L);
				busstate   <= mem_instr ? `AP040_BUS_FETCH :
				              mem_write ? `AP040_BUS_WRITE : `AP040_BUS_READ;
				nwr        <= ~mem_write;
				addr_out   <= mem_addr;
				nuds       <= f_odd;
				nlds       <= !f_odd && !f_word;
				data_write <= (f_odd || !f_word) ? {f_wsh[31:24], f_wsh[31:24]}
				                                 : f_wsh[31:16];
			end
		end
		else if (subcycle_gap) begin
			// The external target sampled IDLE on this edge and can now
			// accept the next word of the same internal transfer.  Reassert
			// the saved request without consuming data on the boundary edge.
			subcycle_gap <= 0;
			busstate   <= instr_r ? `AP040_BUS_FETCH :
			              write_r ? `AP040_BUS_WRITE : `AP040_BUS_READ;
			nwr        <= ~write_r;
			nuds       <= cur_addr[0];
			nlds       <= !cur_addr[0] && !(bytes_left >= 3'd2);
			data_write <= (cur_addr[0] || bytes_left < 3'd2)
			              ? {wshift[31:24], wshift[31:24]}
			              : wshift[31:16];
		end
		else begin
			// one qualified completion for the in-flight 16-bit sub-cycle
			if (!write_r) begin
				if (cur_addr[0])              rshift <= {rshift[23:0], data_in[7:0]};
				else if (bytes_left >= 3'd2)  rshift <= {rshift[15:0], data_in[15:0]};
				else                          rshift <= {rshift[23:0], data_in[15:8]};
			end

			if (next_left == 3'd0) begin
				// transaction complete: return to idle and pulse ack
				active   <= 0;
				busstate <= `AP040_BUS_IDLE;
				nwr      <= 1;
				nuds     <= 1;
				nlds     <= 1;
				longword <= 0;
				subcycle_gap <= 0;
				mem_ack  <= 1;
				if (!write_r) begin
					if (cur_addr[0])             mem_rdata <= {rshift[23:0], data_in[7:0]};
					else if (bytes_left >= 3'd2) mem_rdata <= {rshift[15:0], data_in[15:0]};
					else                         mem_rdata <= {rshift[23:0], data_in[15:8]};
				end
			end
			else begin
				// Set up the following sub-cycle, but first lower the request
				// for one sampled clock.  cpu_cache_new and both RAM write
				// buffers retain ack/address state until cpu_cs drops; changing
				// addr_out under their held acknowledge loses the second word.
				cur_addr   <= next_addr;
				bytes_left <= next_left;
				wshift     <= next_wsh;
				addr_out   <= next_addr;
				busstate   <= `AP040_BUS_IDLE;
				nwr        <= 1;
				nuds       <= 1;
				nlds       <= 1;
				subcycle_gap <= 1;
			end
		end
	end
end

endmodule
