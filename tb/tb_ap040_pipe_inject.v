//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 102: starting the  //
// core where a corpus slice says to)                                       //
//                                                                          //
// tb_ap040_pipe_inject.v - arbitrary start address and injected state      //
//                                                                          //
// Groundwork for running the WinUAE cputest corpus against this core.      //
// tests/ap040/tb_dat_replay.v replays that corpus against the SEQUENTIAL   //
// core, and it works by holding the very first fetch, writing the slice's  //
// whole architectural state straight into the register file, and letting   //
// go. Everything it needs on the memory side this core already has --      //
// ap040_pipe_bus16.v drives the same sixteen-bit port, which is why the    //
// replay driver's own memory model is reused here almost verbatim.         //
//                                                                          //
// What it does NOT have is a reset vector. The sequential core fetches one //
// and can therefore be started anywhere; this one begins at a PARAMETER,   //
// PC_RESET, fixed at elaboration. A slice picks its own start address, so  //
// if that address cannot be set at run time the whole approach fails at    //
// the first slice. This bench answers that one question before any of the  //
// replay driver is ported.                                                 //
//                                                                          //
// The core is elaborated with PC_RESET at $1000 and started at $2000, with //
// D0 pre-loaded to a value no instruction here writes:                     //
//                                                                          //
//   $1000  the address the parameter would have used: MOVEQ #$7F,D1        //
//   $2000  the address injected instead:              MOVEQ #$2A,D1        //
//                                                                          //
// D1 = $2A says the injected address was honoured and the parameter's was  //
// not, and D0 = $55 says injected register state survives the release --   //
// the two halves of what a corpus slice needs. The fetch budget is raised  //
// out of the way: it is an instruction count for short milestone programs  //
// and a corpus slice must not run into it.                                 //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

`include "ap040_pipe_defs.svh"
// run_pipe_verilator.py puts rtl/ap040 on this bench's include path, for the
// adapter ap040_pipe_bus16.v brings with it and for the bus-state names.
`include "ap040_defs.svh"

module tb_ap040_pipe_inject;

localparam [31:0] PC_RESET   = 32'h0000_1000;
localparam integer MEM_WORDS = 8192;

reg clk = 0;
reg nreset = 0;
reg ce = 1;
always #5 clk = ~clk;

wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds, longword;
wire  [1:0] busstate;
wire  [2:0] fc;
reg  [15:0] data_in;
reg         mem_ready;

wire clkena_in = (busstate == `AP040_BUS_IDLE) | mem_ready;

wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;
wire [31:0] dbg_commits;
wire        dbg_if_valid, dbg_id_valid, dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid, dbg_wb_valid;
wire [31:0] dbg_if_pc, dbg_id_pc, dbg_eac_pc, dbg_eaf_pc, dbg_ex_pc, dbg_wb_pc;

ap040_pipe_bus16 #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(32'h0100_0000)   // out of the way: a slice must not meet it
) dut (
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.berr (1'b0),   // no bus errors in this bench
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // no MMU walks here
	.clk(clk), .nreset(nreset), .ce(ce), .clkena_in(clkena_in),
	.data_in(data_in), .addr_out(addr_out), .data_write(data_write),
	.nwr(nwr), .nuds(nuds), .nlds(nlds),
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
// The replay driver's memory model: one sixteen-bit word per access, ready //
// after a fixed delay, the write landing on the completing edge.           //
//--------------------------------------------------------------------------//

reg [15:0] mem [0:MEM_WORDS-1];
wire [31:0] widx = addr_out >> 1;
reg  [2:0] delay;
reg        counting;

always @(posedge clk) begin
	if (!nreset) begin
		mem_ready <= 1'b0; counting <= 1'b0; delay <= 3'd0;
	end else begin
		mem_ready <= 1'b0;
		if (busstate != `AP040_BUS_IDLE && !mem_ready) begin
			if (!counting) begin
				counting <= 1'b1; delay <= 3'd1;
			end else if (delay == 3'd0) begin
				counting  <= 1'b0;
				mem_ready <= 1'b1;
				if (busstate == `AP040_BUS_WRITE) begin
					if (!nuds) mem[widx[12:0]][15:8] <= data_write[15:8];
					if (!nlds) mem[widx[12:0]][7:0]  <= data_write[7:0];
				end
			end else
				delay <= delay - 3'd1;
		end
	end
end

always @(*) data_in = mem[widx[12:0]];

integer errors = 0;

initial begin
	for (errors = 0; errors < MEM_WORDS; errors = errors + 1) mem[errors] = 16'h4E71;
	errors = 0;
	// What the PARAMETER would run.
	mem[32'h1000 >> 1] = 16'h727F;   // MOVEQ #$7F,D1
	mem[32'h1002 >> 1] = 16'h60FE;   // BRA.B -2
	// What the INJECTED address runs.
	mem[32'h2000 >> 1] = 16'h722A;   // MOVEQ #$2A,D1
	mem[32'h2002 >> 1] = 16'h60FE;   // BRA.B -2
end

initial begin
	nreset = 0;
	ce     = 0;                      // held: nothing advances while state goes in
	repeat (4) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	// The injection a corpus slice needs: a start address the parameter does
	// not know about, and register state that must survive the release.
	dut.u_cpu.u_if.pc     = 32'h0000_2000;
	dut.u_cpu.u_if.issued = 32'd0;
	dut.u_cpu.u_regfile.dreg[0] = 32'h0000_0055;
	dut.u_cpu.sr          = 16'h2700;

	@(posedge clk);
	ce = 1;                          // released

	repeat (400) @(posedge clk);

	if (dbg_d1 !== 32'h0000_002A) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 0000002a. The core ran from the address injected into the fetch stage, not the one PC_RESET was elaborated with; 0000007f is the parameter's.",
		         dbg_d1);
	end
	if (dbg_d0 !== 32'h0000_0055) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 00000055 (injected register state must survive the release)", dbg_d0);
	end
	if (dbg_commits === 32'd0) begin
		errors = errors + 1;
		$display("FAIL: no instruction committed at all");
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
