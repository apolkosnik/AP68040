//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 81)                 //
//                                                                          //
// tb_ap040_pipe_bus.v - the pipeline running out of a real memory port     //
//                                                                          //
// Every other bench instantiates ap040_pipe_core.v, whose memory is an     //
// array inside the core. This one instantiates ap040_pipe_sys.v: the same  //
// CPU with ap040_pipe_membus.v underneath it, driving the external port    //
// rtl/ap040/ap040_core.v drives -- mem_req held until mem_ack, mem_ack a   //
// single-cycle pulse with mem_rdata valid. The memory here is a model in   //
// the testbench, not a module in the core, and it answers after 1 to 8     //
// cycles from a fixed-seed xorshift, so no transaction completes at a      //
// fixed distance from its request.                                         //
//                                                                          //
// The program is small on purpose; what is being tested is the bridge.     //
//                                                                          //
//   $0400  MOVE.L #$12345678,D0                                            //
//   $0406  MOVEA.L #$0800,A0                                               //
//   $040C  MOVE.L D0,(A0)        a Long write, posted behind wr_busy       //
//   $040E  MOVE.L (A0),D1        a Long read of the address just written    //
//   $0410  ADDQ.L #1,D1                                                    //
//   $0412  TRAP #1               frame push, vector fetch, redirect        //
//   $0414  MOVE.L D1,D2          <- the RTE returns here                   //
//   $0416  NOP                                                             //
//   $0900  MOVEQ #$2A,D3 / RTE   the handler                               //
//                                                                          //
// The load at $040E is the ordering case: the store ahead of it is still   //
// posted when it issues. The array answers such a read from its write      //
// buffer; the bridge instead drains the write first, and D1 says which     //
// value arrived. Vector 33 sits at byte $84 -- its architectural place,    //
// not the array benches' aliased word index, because the CPU now emits     //
// byte addresses.                                                          //
//                                                                          //
// Besides the program's result the bench watches the bus itself: every     //
// fetch must be an aligned Long -- the prefetch stream's unit since         //
// 2026-09-24; it was a Word -- with a supervisor-program function code,    //
// every data access a supervisor-data one, and the store must appear as   //
// one Long write to $0800 -- a byte-at-a-time or wrongly-sized bridge      //
// would still leave the right bytes in memory here.                        //
//                                                                          //
// There is no milestone-80 control for this bench: ap040_pipe_sys.v does   //
// not exist before milestone 81, and the CPU could not be instantiated     //
// without the array.                                                       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_bus;

// Enough fetch requests for the program and its speculation, and no more:
// the fetcher stops at PROG_WORDS, which is how every bench here drains.
// Through a memory that answers in 1 to 8 cycles that tail is long, hence
// the wait below.
localparam PROG_WORDS      = 64;
localparam [31:0] PC_RESET = 32'h0000_0400;
localparam integer MEM_WORDS = 32768;   // a 64 KB byte space

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

wire        mem_req, mem_write, mem_instr;
wire  [1:0] mem_size;
wire [31:0] mem_addr, mem_wdata;
wire  [2:0] mem_fc;
reg         mem_ack;
reg  [31:0] mem_rdata;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;
wire [15:0] dbg_sr;
wire [31:0] dbg_commits;

ap040_pipe_sys #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk), .nreset (nreset), .ce (ce),

	.mem_req (mem_req), .mem_write(mem_write), .mem_instr(mem_instr),
	.mem_size(mem_size), .mem_addr(mem_addr), .mem_wdata(mem_wdata),
	.mem_fc  (mem_fc),   .mem_ack (mem_ack),  .mem_rdata(mem_rdata),

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
// The memory model. Word storage, big endian, byte addressed from the bus. //
//--------------------------------------------------------------------------//

reg [15:0] mem [0:MEM_WORDS-1];
integer i;

reg [15:0] lfsr;
reg  [2:0] delay;
reg        active;

wire [31:0] widx = mem_addr >> 1;

integer fetches, data_reads, data_writes, fc_bad, size_bad, long_store_0800;

always @(posedge clk) begin
	if (!nreset) begin
		active <= 1'b0; mem_ack <= 1'b0; delay <= 3'd0; lfsr <= 16'hBEEF;
	end else begin
		mem_ack <= 1'b0;
		if (!active && mem_req && !mem_ack) begin
			active <= 1'b1;
			delay  <= lfsr[2:0];
			lfsr   <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
			// Watch the request itself, not just its effect.
			if (mem_instr) begin
				fetches = fetches + 1;
				if (mem_fc !== `AP040_FC_SUPER_PROG) fc_bad = fc_bad + 1;
				if (mem_size !== `AP040_SZ_L || mem_addr[1:0] != 2'b00) size_bad = size_bad + 1;   // prefetch: aligned Longs
			end else begin
				if (mem_fc !== `AP040_FC_SUPER_DATA) fc_bad = fc_bad + 1;
				if (mem_write) begin
					data_writes = data_writes + 1;
					if (mem_addr == 32'h0000_0800 && mem_size == `AP040_SZ_L)
						long_store_0800 = long_store_0800 + 1;
				end else begin
					data_reads = data_reads + 1;
					if (mem_size !== `AP040_SZ_L)    size_bad = size_bad + 1;
				end
			end
		end else if (active) begin
			if (delay == 3'd0) begin
				active  <= 1'b0;
				mem_ack <= 1'b1;
				if (mem_write) begin
					case (mem_size)
					`AP040_SZ_L: begin
						mem[widx]     <= mem_wdata[31:16];
						mem[widx + 1] <= mem_wdata[15:0];
					end
					`AP040_SZ_W: mem[widx] <= mem_wdata[15:0];
					default: begin
						if (mem_addr[0]) mem[widx][7:0]  <= mem_wdata[7:0];
						else             mem[widx][15:8] <= mem_wdata[7:0];
					end
					endcase
					mem_rdata <= 32'h0;
				end else begin
					case (mem_size)
					`AP040_SZ_L: mem_rdata <= {mem[widx], mem[widx + 1]};
					`AP040_SZ_W: mem_rdata <= {16'd0, mem[widx]};
					default:     mem_rdata <= {24'd0, mem_addr[0] ? mem[widx][7:0]
					                                              : mem[widx][15:8]};
					endcase
				end
			end else
				delay <= delay - 3'd1;
		end
	end
end

integer errors = 0;

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
	fetches = 0; data_reads = 0; data_writes = 0;
	fc_bad = 0; size_bad = 0; long_store_0800 = 0;

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

	// Vector 33 at its architectural address, 33 * 4 = $84.
	poke(32'h0084, 16'h0000); poke(32'h0086, 16'h0900);

	// The store's target, so a load that returned the OLD contents is
	// distinguishable from one that never ran.
	poke(32'h0800, 16'hDEAD); poke(32'h0802, 16'hBEEF);
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	dut.u_cpu.u_regfile.isp = 32'h0000_0600;

	repeat ((PROG_WORDS + 3000) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D0", dbg_d0, 32'h1234_5678);
	check32("D1 (the load saw the posted store, not $DEADBEEF)", dbg_d1, 32'h1234_5679);
	check32("D2 (the RTE returned to $0414)", dbg_d2, 32'h1234_5679);
	check32("D3 (the TRAP handler ran)", dbg_d3, 32'h0000_002A);
	check32("[$0800]", {mem[32'h0800 >> 1], mem[32'h0802 >> 1]}, 32'h1234_5678);
	check32("ISP", dut.u_cpu.u_regfile.isp, 32'h0000_0600);
	check32("SR", {16'd0, dbg_sr}, 32'h0000_2700);

	if (fetches < 8) begin
		errors = errors + 1;
		$display("FAIL: only %0d instruction fetches reached the bus", fetches);
	end
	if (data_reads < 3) begin
		errors = errors + 1;
		$display("FAIL: only %0d data reads reached the bus (load, vector, two RTE pops)", data_reads);
	end
	if (data_writes < 3) begin
		errors = errors + 1;
		$display("FAIL: only %0d data writes reached the bus (store, two frame beats)", data_writes);
	end
	// One bus transaction per fetch the fetcher actually issues, give or
	// take a discarded one a redirect left in flight. More than that means
	// it is asking memory for words it does not want -- which is what it
	// did until this milestone, once the program ran out: 347 fetches for
	// this program instead of 64.
	if (fetches > PROG_WORDS + 16) begin
		errors = errors + 1;
		$display("FAIL: %0d instruction fetches for a %0d-word budget -- the fetcher is re-asking for words it has",
		         fetches, PROG_WORDS);
	end
	if (long_store_0800 != 1) begin
		errors = errors + 1;
		$display("FAIL: %0d Long writes to $0800, expected exactly 1", long_store_0800);
	end
	if (fc_bad != 0) begin
		errors = errors + 1;
		$display("FAIL: %0d transactions carried the wrong function code", fc_bad);
	end
	if (size_bad != 0) begin
		errors = errors + 1;
		$display("FAIL: %0d transactions carried the wrong size", size_bad);
	end

	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid ||
	    dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
		errors = errors + 1;
		$display("FAIL: a stage is still valid after the program should have drained: IF%0b ID%0b EAC%0b EAF%0b EX%0b WB%0b, if_pc=%h",
		         dbg_if_valid, dbg_id_valid, dbg_eac_valid, dbg_eaf_valid,
		         dbg_ex_valid, dbg_wb_valid, dbg_if_pc);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED (%0d fetches, %0d reads, %0d writes)",
		         fetches, data_reads, data_writes);
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
