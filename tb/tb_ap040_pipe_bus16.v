//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 82)                 //
//                                                                          //
// tb_ap040_pipe_bus16.v - the pipelined core on the 16-bit Minimig bus     //
//                                                                          //
// ap040_pipe_bus16.v puts rtl/ap040/ap040_bus16_adapter.v -- the FSM       //
// core's own adapter, unmodified -- under the pipelined core, so this      //
// bench drives the interface cpu_wrapper.v drives: addr_out/data_in/       //
// data_write, nwr/nuds/nlds, busstate, longword, fc, and one qualified     //
// clkena_in pulse per 16-bit sub-cycle. The memory model is the one        //
// tests/ap040/tb_dat_replay.v uses for the FSM core, with a variable       //
// answer time: data_in is the whole word at addr_out and the lanes decide  //
// which half a write lands in.                                            //
//                                                                          //
// The program is tb_ap040_pipe_bus.v's, so the two benches differ only in  //
// what is under the CPU:                                                   //
//                                                                          //
//   $0400  MOVE.L #$12345678,D0                                            //
//   $0406  MOVEA.L #$0800,A0                                               //
//   $040C  MOVE.L D0,(A0)        ONE 32-bit store -> TWO bus sub-cycles    //
//   $040E  MOVE.L (A0),D1        one 32-bit load  -> two sub-cycles        //
//   $0410  ADDQ.L #1,D1                                                    //
//   $0412  TRAP #1                                                         //
//   $0414  MOVE.L D1,D2          <- the RTE returns here                   //
//   $0416  NOP                                                             //
//   $0900  MOVEQ #$2A,D3 / RTE                                             //
//                                                                          //
// What is checked beyond the program's result is the splitting itself,     //
// because that is all this layer does: the Long store must appear as two   //
// word sub-cycles, $1234 to $0800 then $5678 to $0802, both with longword  //
// asserted and both lanes enabled; every instruction fetch must be one     //
// sub-cycle with busstate = fetch; and no sub-cycle may select neither     //
// lane. A bridge that emitted the two halves in the wrong order, or the    //
// low half twice, leaves memory wrong and is caught by the program; one    //
// that emitted them as four byte cycles leaves memory RIGHT and is caught  //
// only here.                                                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
// The busstate encoding is the adapter's contract, not the pipeline's, so
// it comes from the FSM core's defs -- the same file ap040_bus16_adapter.v
// includes. run_pipe_verilator.py puts rtl/ap040 on this bench's include
// path for exactly this reason.
`include "ap040_defs.svh"

module tb_ap040_pipe_bus16;

localparam PROG_WORDS      = 64;
localparam [31:0] PC_RESET = 32'h0000_0400;
localparam integer MEM_WORDS = 32768;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
// A pseudo-random clock enable (milestone 94). Every bench in this suite
// tied ce high, and eight of the thirteen defects three rounds of external
// review found lived behind that: a cycle with ce low is a cycle that did
// not happen, and the core has to treat it that way. Driven on the falling
// edge so it is stable across every rising one, and left high until reset
// releases so the reset sequence itself is unchanged.
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`endif

wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds, longword;
wire  [1:0] busstate;
wire  [2:0] fc;
reg         mem_ready;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;
wire [15:0] dbg_sr;
wire [31:0] dbg_commits;

// The host's bus enable, exactly as tb_dat_replay.v forms it: free-running
// while the bus is idle, one pulse per completed sub-cycle otherwise.
wire clkena_in = (busstate == `AP040_BUS_IDLE) | mem_ready;

reg [15:0] mem [0:MEM_WORDS-1];
wire [31:0] widx = addr_out >> 1;
wire [15:0] data_in = mem[widx];

ap040_pipe_bus16 #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.berr (1'b0),   // no bus errors in this bench
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // no MMU walks here
	.clk (clk), .nreset (nreset), .ce (ce), .clkena_in (clkena_in),

	.data_in (data_in), .addr_out(addr_out), .data_write(data_write),
	.nwr (nwr), .nuds(nuds), .nlds(nlds),
	.busstate(busstate), .longword(longword), .fc(fc),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_d0(dbg_d0), .dbg_d1(dbg_d1), .dbg_d2(dbg_d2), .dbg_d3(dbg_d3),
	.dbg_d4(dbg_d4), .dbg_d5(dbg_d5), .dbg_d6(dbg_d6), .dbg_d7(dbg_d7),
	.dbg_ccr(dbg_ccr), .dbg_sr(dbg_sr), .dbg_commits(dbg_commits)
);

//--------------------------------------------------------------------------//
// The memory: answers a sub-cycle after 1 to 8 cycles, writes by lane.     //
//--------------------------------------------------------------------------//

reg [15:0] lfsr;
reg  [2:0] delay;
reg        counting;

integer subcycles, fetch_sub, read_sub, write_sub, lane_bad, fc_bad;
integer store_sub;
reg [15:0] w0800, w0802;
reg        lw0800, lw0802, lanes0800, lanes0802;

always @(posedge clk) begin
	if (!nreset) begin
		mem_ready <= 1'b0; counting <= 1'b0; delay <= 3'd0; lfsr <= 16'h1234;
	end else begin
		mem_ready <= 1'b0;
		if (busstate != `AP040_BUS_IDLE && !mem_ready) begin
			if (!counting) begin
				counting <= 1'b1;
				delay    <= lfsr[2:0];
				lfsr     <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
			end else if (delay == 3'd0) begin
				counting  <= 1'b0;
				mem_ready <= 1'b1;

				// The write lands on the completing edge, per the model in
				// tests/ap040/tb_dat_replay.v.
				if (busstate == `AP040_BUS_WRITE) begin
					if (!nuds) mem[widx][15:8] <= data_write[15:8];
					if (!nlds) mem[widx][7:0]  <= data_write[7:0];
				end

				// ...and the sub-cycle is counted as it completes.
				subcycles = subcycles + 1;
				if (nuds && nlds) lane_bad = lane_bad + 1;
				case (busstate)
				`AP040_BUS_FETCH: begin
					fetch_sub = fetch_sub + 1;
					if (fc !== `AP040_FC_SUPER_PROG) fc_bad = fc_bad + 1;
				end
				`AP040_BUS_READ: begin
					read_sub = read_sub + 1;
					if (fc !== `AP040_FC_SUPER_DATA) fc_bad = fc_bad + 1;
				end
				default: begin
					write_sub = write_sub + 1;
					if (fc !== `AP040_FC_SUPER_DATA) fc_bad = fc_bad + 1;
					if (addr_out == 32'h0000_0800) begin
						store_sub = store_sub + 1;
						w0800     = data_write; lw0800 = longword;
						lanes0800 = !nuds && !nlds;
					end
					if (addr_out == 32'h0000_0802) begin
						store_sub = store_sub + 1;
						w0802     = data_write; lw0802 = longword;
						lanes0802 = !nuds && !nlds;
					end
				end
				endcase
			end else
				delay <= delay - 3'd1;
		end
	end
end

integer errors = 0;
integer i;

task check32;
	input string  what;
	input [31:0]  got, want;
	begin
		if (got !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s = %h, expected %h", what, got, want);
		end
	end
endtask

task poke;
	input [31:0] addr;
	input [15:0] word;
	begin mem[addr >> 1] = word; end
endtask

initial begin
	for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = `AP040_OP_NOP;
	subcycles = 0; fetch_sub = 0; read_sub = 0; write_sub = 0;
	lane_bad = 0; fc_bad = 0; store_sub = 0;
	w0800 = 16'h0; w0802 = 16'h0;
	lw0800 = 1'b0; lw0802 = 1'b0; lanes0800 = 1'b0; lanes0802 = 1'b0;

	poke(32'h0400, 16'h203C); poke(32'h0402, 16'h1234); poke(32'h0404, 16'h5678);
	poke(32'h0406, 16'h207C); poke(32'h0408, 16'h0000); poke(32'h040A, 16'h0800);
	poke(32'h040C, 16'h2080);
	poke(32'h040E, 16'h2210);
	poke(32'h0410, 16'h5281);
	poke(32'h0412, 16'h4E41);
	poke(32'h0414, 16'h2401);
	poke(32'h0416, 16'h4E71);

	poke(32'h0900, 16'h762A);
	poke(32'h0902, 16'h4E73);

	poke(32'h0084, 16'h0000); poke(32'h0086, 16'h0900);

	poke(32'h0800, 16'hDEAD); poke(32'h0802, 16'hBEEF);
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 6000) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D0", dbg_d0, 32'h1234_5678);
	check32("D1 (the load saw the posted store)", dbg_d1, 32'h1234_5679);
	check32("D2 (the RTE returned to $0414)", dbg_d2, 32'h1234_5679);
	check32("D3 (the TRAP handler ran)", dbg_d3, 32'h0000_002A);
	check32("[$0800]", {mem[32'h0800 >> 1], mem[32'h0802 >> 1]}, 32'h1234_5678);
	check32("ISP", dut.u_cpu.u_regfile.isp, 32'h0000_0600);
	check32("SR", {16'd0, dbg_sr}, 32'h0000_2700);

	// The splitting itself.
	if (store_sub != 2) begin
		errors = errors + 1;
		$display("FAIL: the Long store took %0d word sub-cycles at $0800/$0802, expected 2", store_sub);
	end
	check32("first store sub-cycle, $0800", {16'd0, w0800}, 32'h0000_1234);
	check32("second store sub-cycle, $0802", {16'd0, w0802}, 32'h0000_5678);
	if (!lw0800 || !lw0802) begin
		errors = errors + 1;
		$display("FAIL: longword was not asserted for both halves of the Long store (%b, %b)", lw0800, lw0802);
	end
	if (!lanes0800 || !lanes0802) begin
		errors = errors + 1;
		$display("FAIL: a word sub-cycle of the Long store did not enable both lanes (%b, %b)", lanes0800, lanes0802);
	end
	if (fetch_sub < 8) begin
		errors = errors + 1;
		$display("FAIL: only %0d fetch sub-cycles", fetch_sub);
	end
	if (fetch_sub > PROG_WORDS + 16) begin
		errors = errors + 1;
		$display("FAIL: %0d fetch sub-cycles for a %0d-word budget -- a Word fetch must take one",
		         fetch_sub, PROG_WORDS);
	end
	if (read_sub < 8) begin
		errors = errors + 1;
		$display("FAIL: only %0d data read sub-cycles (four Long reads is eight)", read_sub);
	end
	if (write_sub < 6) begin
		errors = errors + 1;
		$display("FAIL: only %0d data write sub-cycles (three Long writes is six)", write_sub);
	end
	if (lane_bad != 0) begin
		errors = errors + 1;
		$display("FAIL: %0d sub-cycles selected neither lane", lane_bad);
	end
	if (fc_bad != 0) begin
		errors = errors + 1;
		$display("FAIL: %0d sub-cycles carried the wrong function code", fc_bad);
	end

	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid ||
	    dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
		errors = errors + 1;
		$display("FAIL: a stage is still valid after the program should have drained: IF%0b ID%0b EAC%0b EAF%0b EX%0b WB%0b",
		         dbg_if_valid, dbg_id_valid, dbg_eac_valid, dbg_eaf_valid,
		         dbg_ex_valid, dbg_wb_valid);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED (%0d sub-cycles: %0d fetch, %0d read, %0d write)",
		         subcycles, fetch_sub, read_sub, write_sub);
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
