//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 93: whose write is //
// it)                                                                      //
//                                                                          //
// tb_ap040_pipe_rmwfc.v - EX's store, and a younger exception's privilege  //
//                                                                          //
// Two stages can want the memory's write port in the same cycle. EX wins:  //
// it holds the older instruction, and a read-modify-write's store issues   //
// from there because the value does not exist until then. The address, the //
// size and the data all switch to EX when it takes the port.               //
//                                                                          //
// The privilege did not. It came straight from EA-fetch, where a YOUNGER   //
// instruction sits -- and if that younger instruction is taking an         //
// exception, EA-fetch is forcing supervisor for the frame it is about to   //
// push. The older store then goes out on the bus as a supervisor access    //
// although the instruction that made it was running in user mode.          //
//                                                                          //
//   ISP = $1000 ; A0 = $0800 ; A1 = $0810 ; D2 = 7 ; drop to user mode     //
//   MOVE.L D2,(A1)    fills the write buffer, so the next store must wait  //
//   ADDQ.L #1,(A0)    a user-mode read-modify-write, storing from EX       //
//   TRAP #1           immediately behind it, forcing supervisor            //
//                                                                          //
// The store ahead of it is not decoration. Without backpressure EX's store //
// is accepted in the cycle it is offered, before the exception behind it   //
// has decided anything, and the window never opens. With the buffer        //
// already full, EX waits -- and while it waits the TRAP reaches its own    //
// verdict and starts forcing supervisor, so the store goes out under it.   //
//                                                                          //
// The write to $0800 must carry the user-data code. The frame's two beats  //
// and the vector read must carry the supervisor-data one, which is         //
// milestone 92's rule and is checked here too so a fix that simply stopped //
// forcing supervisor at all could not pass.                                //
//                                                                          //
// On an MMU that separates the two spaces, a user store landing in         //
// supervisor space writes somewhere the program had no right to.           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_rmwfc;

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

integer fetches, data_reads, data_writes, size_bad;
integer fetch_user, data_user, data_super, rmw_writes, rmw_not_user;

always @(posedge clk) begin
	if (!nreset) begin
		active <= 1'b0; mem_ack <= 1'b0; delay <= 3'd0; lfsr <= 16'hBEEF;
	end else begin
		mem_ack <= 1'b0;
		if (!active && mem_req && !mem_ack) begin
			active <= 1'b1;
			// A FIXED long delay, not the other bus benches' random one. The
			// window under test needs the write buffer to be occupied when
			// EX offers its store, and a memory that sometimes answers in
			// one cycle sometimes closes the window before it opens. Seven
			// cycles for every access makes it deterministic.
			delay  <= 3'd7;
			// The function code carried by each request, by class. The
			// whole point of the bench: the code on the wire, not the
			// effect it had in memory.
			if (mem_instr) begin
				fetches = fetches + 1;
				if (mem_fc === `AP040_FC_USER_PROG)  fetch_user = fetch_user + 1;
				if (mem_size !== `AP040_SZ_W)        size_bad = size_bad + 1;
			end else begin
				if (mem_fc === `AP040_FC_USER_DATA)  data_user  = data_user + 1;
				if (mem_fc === `AP040_FC_SUPER_DATA) data_super = data_super + 1;
				if (mem_write) begin
					data_writes = data_writes + 1;
					// The read-modify-write's own store, by address.
					if (mem_addr == 32'h0000_0800) begin
						rmw_writes = rmw_writes + 1;
						if (mem_fc !== `AP040_FC_USER_DATA)
							rmw_not_user = rmw_not_user + 1;
					end
				end else data_reads = data_reads + 1;
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
	fetches = 0; data_reads = 0; data_writes = 0; size_bad = 0;
	fetch_user = 0; data_user = 0; data_super = 0; rmw_writes = 0; rmw_not_user = 0;

	poke(32'h0400, 16'h203C); poke(32'h0402, 16'h0000); poke(32'h0404, 16'h1000);
	poke(32'h0406, 16'h4E7B); poke(32'h0408, 16'h0804);   // MOVEC D0,ISP
	poke(32'h040A, 16'h207C); poke(32'h040C, 16'h0000); poke(32'h040E, 16'h0800);
	poke(32'h0410, 16'h227C); poke(32'h0412, 16'h0000); poke(32'h0414, 16'h0810);
	poke(32'h0416, 16'h7407);                              // MOVEQ #7,D2
	poke(32'h0418, 16'h7200);                              // MOVEQ #0,D1
	poke(32'h041A, 16'h46C1);                              // MOVE D1,SR  -> user
	poke(32'h041C, 16'h2282);                              // MOVE.L D2,(A1)
	poke(32'h041E, 16'h5290);                              // ADDQ.L #1,(A0)
	poke(32'h0420, 16'h4E41);                              // TRAP #1
	poke(32'h0422, 16'h7866);                              // MOVEQ #$66,D4 (poison)
	poke(32'h0424, 16'h4E71);

	// The read-modify-write's target, and the filler store's.
	poke(32'h0800, 16'h0000); poke(32'h0802, 16'h0041);
	poke(32'h0810, 16'h0000); poke(32'h0812, 16'h0000);

	poke(32'h0900, 16'h762A);                              // MOVEQ #$2A,D3
	poke(32'h0902, 16'h4E71);                              // NOP

	// Vector 33 at its architectural address, 33 * 4 = $84.
	poke(32'h0084, 16'h0000); poke(32'h0086, 16'h0900);
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);

	repeat ((PROG_WORDS + 3000) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D3 (the TRAP handler ran)", dbg_d3, 32'h0000_002A);
	check32("D4 (the instruction after the TRAP must not run)", dbg_d4, 32'h0000_0000);
	check32("[$0800] (ADDQ.L #1 on 00000041)", {mem[32'h0800 >> 1], mem[32'h0802 >> 1]}, 32'h0000_0042);

	if (fetch_user < 1) begin
		errors = errors + 1;
		$display("FAIL: no instruction fetch carried a user function code, so the program never left supervisor mode and the checks below are vacuous");
	end
	if (rmw_writes !== 1) begin
		errors = errors + 1;
		$display("FAIL: %0d writes to $0800, expected 1 (the read-modify-write's own store)", rmw_writes);
	end
	if (rmw_not_user !== 0) begin
		errors = errors + 1;
		$display("FAIL: the read-modify-write's store to $0800 went out with a SUPERVISOR function code. It belongs to a user-mode instruction; the exception behind it forces supervisor for its own frame, and EX took the port for address, size and data but not for privilege.");
	end
	if (data_super !== 3) begin
		errors = errors + 1;
		$display("FAIL: %0d data accesses carried the supervisor-data code, expected 3 (two frame beats and the vector read)", data_super);
	end
	check32("[$0810] (the filler store)", {mem[32'h0810 >> 1], mem[32'h0812 >> 1]}, 32'h0000_0007);
	if (data_user !== 3) begin
		errors = errors + 1;
		$display("FAIL: %0d data accesses carried the user-data code, expected 3 (the filler store, the read-modify-write's load and its store)", data_user);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
