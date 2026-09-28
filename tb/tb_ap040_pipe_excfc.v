//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 92: an exception's //
// function codes)                                                          //
//                                                                          //
// tb_ap040_pipe_excfc.v - a TRAP taken from user mode, watched on the bus  //
//                                                                          //
// The frame writes and the vector read of an exception are SUPERVISOR      //
// accesses, whatever mode the instruction that faulted was running in.     //
// That is not a detail: the function code is what an MMU and a decoder use //
// to tell supervisor space from user space, so a frame pushed with a user  //
// code lands in the wrong address space on any system that separates them, //
// and the vector is fetched from the wrong one.                            //
//                                                                          //
// ap040_pipe_sys.v takes the privilege for the whole bridge from the       //
// COMMITTED status register, which during exception entry still says user  //
// -- the switch to supervisor has not reached the commit point while the   //
// frame is being pushed. The code is also read when the transaction is     //
// SENT rather than when the request was accepted, so a posted write can go //
// out under whatever privilege happens to be current by then.              //
//                                                                          //
//   MOVE.L #$1000,D0 ; MOVEC D0,ISP    a supervisor stack to push onto     //
//   MOVEQ #0,D1 ; MOVE D1,SR           drop to user mode                   //
//   NOP                                fetched with a USER program code    //
//   TRAP #1                            frame on ISP, vector 33 at $84      //
//                                                                          //
// Every data access this program makes belongs to the exception: two frame //
// beats and the vector read. All three must carry the supervisor-data      //
// code and none may carry the user-data one.                               //
//                                                                          //
// The user-mode fetch is counted too. Without it a run that never left     //
// supervisor mode would pass the function-code check for the wrong reason. //
//                                                                          //
// The HANDLER's own first fetch is a separate case and a separate defect.  //
// It is a supervisor-program access: the instruction it brings back runs   //
// in supervisor mode, because that is what the exception just switched to. //
// The fetch goes out before that switch reaches the commit point, so a     //
// privilege read off the committed register calls it a user fetch -- the   //
// handler's first instruction arriving from user space.                    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_excfc;

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
integer fetch_user, data_user, data_super, handler_fetches, handler_fc_bad;

always @(posedge clk) begin
	if (!nreset) begin
		active <= 1'b0; mem_ack <= 1'b0; delay <= 3'd0; lfsr <= 16'hBEEF;
	end else begin
		mem_ack <= 1'b0;
		if (!active && mem_req && !mem_ack) begin
			active <= 1'b1;
			delay  <= lfsr[2:0];
			lfsr   <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
			// The function code carried by each request, by class. The
			// whole point of the bench: the code on the wire, not the
			// effect it had in memory.
			if (mem_instr) begin
				fetches = fetches + 1;
				if (mem_fc === `AP040_FC_USER_PROG)  fetch_user = fetch_user + 1;
				if (mem_addr == 32'h0000_0900) begin
					handler_fetches = handler_fetches + 1;
					if (mem_fc !== `AP040_FC_SUPER_PROG)
						handler_fc_bad = handler_fc_bad + 1;
				end
				if (mem_size !== `AP040_SZ_L || mem_addr[1:0] != 2'b00) size_bad = size_bad + 1;   // prefetch: aligned Longs
			end else begin
				if (mem_fc === `AP040_FC_USER_DATA)  data_user  = data_user + 1;
				if (mem_fc === `AP040_FC_SUPER_DATA) data_super = data_super + 1;
				if (mem_write) data_writes = data_writes + 1;
				else           data_reads  = data_reads + 1;
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
	fetch_user = 0; data_user = 0; data_super = 0; handler_fetches = 0; handler_fc_bad = 0;

	poke(32'h0400, 16'h203C); poke(32'h0402, 16'h0000); poke(32'h0404, 16'h1000);
	poke(32'h0406, 16'h4E7B); poke(32'h0408, 16'h0804);   // MOVEC D0,ISP
	poke(32'h040A, 16'h7200);                              // MOVEQ #0,D1
	poke(32'h040C, 16'h46C1);                              // MOVE D1,SR  -> user
	poke(32'h040E, 16'h4E71);                              // NOP, fetched as USER
	poke(32'h0410, 16'h4E41);                              // TRAP #1
	poke(32'h0412, 16'h7466);                              // MOVEQ #$66,D2 (poison)
	poke(32'h0414, 16'h4E71);

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
	check32("D2 (the instruction after the TRAP must not run)", dbg_d2, 32'h0000_0000);
	check32("ISP after a twelve-byte-free format $0 frame", dut.u_cpu.u_regfile.isp, 32'h0000_0FF8);

	// The core really was in user mode, or the rest proves nothing.
	if (fetch_user < 1) begin
		errors = errors + 1;
		$display("FAIL: no instruction fetch carried a user function code, so the program never left supervisor mode and the check below is vacuous");
	end
	// Every data access in this program belongs to the exception: two frame
	// writes and the vector read. All three are supervisor accesses however
	// the faulting instruction was running.
	if (data_user !== 0) begin
		errors = errors + 1;
		$display("FAIL: %0d data accesses went out with a USER function code. An exception's frame writes and vector read are SUPERVISOR accesses whatever mode the instruction that faulted was in -- the privilege has to travel with the request, not be read off the committed SR when the transaction is sent.",
		         data_user);
	end
	if (data_super !== 3) begin
		errors = errors + 1;
		$display("FAIL: %0d data accesses carried the supervisor-data code, expected 3 (two frame beats and the vector read)", data_super);
	end
	if (data_writes !== 2) begin
		errors = errors + 1;
		$display("FAIL: %0d data writes reached the bus, expected 2 (the format $0 frame's two beats)", data_writes);
	end
	if (handler_fetches < 1) begin
		errors = errors + 1;
		$display("FAIL: the handler at $0900 was never fetched");
	end
	if (handler_fc_bad !== 0) begin
		errors = errors + 1;
		$display("FAIL: %0d fetches of the handler at $0900 went out with a user program function code. The handler runs in SUPERVISOR mode -- the exception switched to it -- and the fetch goes out before that switch reaches the committed register, so the privilege cannot be read from there.",
		         handler_fc_bad);
	end
	if (data_reads !== 1) begin
		errors = errors + 1;
		$display("FAIL: %0d data reads reached the bus, expected 1 (the vector)", data_reads);
	end
	if (size_bad !== 0) begin
		errors = errors + 1;
		$display("FAIL: %0d requests went out with the wrong size", size_bad);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
