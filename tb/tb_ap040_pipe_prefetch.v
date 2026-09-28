//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: prefetch)         //
//                                                                          //
// tb_ap040_pipe_prefetch.v - the bus bridge's instruction stream buffer    //
//                                                                          //
// ap040_pipe_membus.v fetches aligned longwords into a four-entry window   //
// ahead of the fetch unit. On ap040_pipe_sys.v, through a memory that      //
// answers in 1 to 8 cycles:                                                //
//   eight DIVU.W hold EX while the fetch unit and the window run ahead;   //
//     the MOVE.W behind them patches a MOVEQ seven words on, which by     //
//     then is in the window but not in the pipe: the patched value must   //
//     run -- a store into the window empties it, and goes out before the  //
//     refetch. The bench checks the target WAS fetched before the store:  //
//     with one DIVU and the four-step divider it never was, and the case  //
//     went untested while the bench passed                                 //
//   a DBF loop with a load in it: data reads between prefetches           //
//   BRA.B +6 inside the window, over three poison MOVEQ #-1,D4            //
//   BRA.W to $4A2, past the window and into the middle of a longword      //
//   JMP to $FFFFFFD8 and the same patch across the top of the address     //
//     space: a MOVE.W to $0 while the window runs from $FFFFFFFx on       //
//     through it. The window's compares were ordered, and a write past    //
//     the wrap missed it (review 15); MOVEQ #$55,D0 must run at $0, and   //
//     is stored to $A00                                                   //
//   MOVE #$0000,SR, then BRA.B -2, whose longword must end up fetched     //
//     with the user-program function code: the window was filled as       //
//     supervisor, and a request under the other privilege misses it. The //
//     ADDQ between them is not refetched: it is in the pipe already, as   //
//     the reference core's queue would hold it -- neither core flushes on //
//     a write to SR                                                       //
// D5 $55, D6 $66, D7 $77; D3 4 (the loop ran four times), D2 $FFFF; D1    //
// the loaded $CAFEF00D; D4 0; D0 $23; SR $0000; $A00 $0055.               //
// ce is held low for three cycles out of reset, and the first transaction //
// must be the reset PC's longword: the stream once fetched $0 unasked     //
// there (review 15), and a $0 that never answers hung the core.           //
// The memory aliases the address space onto its 64 KB, so $FFFFFFD8 is    //
// $FFD8 and a longword at $FFFFFFFE is its last word and its first.      //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_prefetch;

localparam PROG_WORDS      = 200;
localparam [31:0] PC_RESET = 32'h0000_03F0;
localparam integer MEM_WORDS = 32768;

reg clk = 0;
reg nreset = 0;
reg ce = 0;
reg ce_run = 0;   // low for three cycles out of reset first

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset && ce_run) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`else
always @(negedge clk) ce <= ce_run;
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

// The memory: word storage, big endian, 1 to 8 cycles from a fixed seed.
reg [15:0] mem [0:MEM_WORDS-1];
integer i;
reg [15:0] lfsr;
reg  [2:0] delay;
reg        active;
wire [31:0] widx  = (mem_addr >> 1) & (MEM_WORDS - 1);
wire [31:0] widx1 = ((mem_addr >> 1) + 1) & (MEM_WORDS - 1);
reg  [31:0] first_addr;   // the first transaction out of reset
reg         first_instr;
reg         first_seen;
reg  [2:0] fc_last [0:MEM_WORDS/2-1];   // the function code each longword was last fetched with

always @(posedge clk) begin
	if (!nreset) begin
		active <= 1'b0; mem_ack <= 1'b0; delay <= 3'd0; lfsr <= 16'hBEEF;
		first_seen <= 1'b0; first_addr <= 32'hFFFF_FFFF; first_instr <= 1'b0;
	end else begin
		mem_ack <= 1'b0;
		if (!active && mem_req && !mem_ack) begin
			active <= 1'b1;
			delay  <= lfsr[2:0];
			lfsr   <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
			if (mem_instr) fc_last[(mem_addr >> 2) & (MEM_WORDS / 2 - 1)] = mem_fc;
			if (!first_seen) begin
				first_seen  <= 1'b1;
				first_addr  <= mem_addr;
				first_instr <= mem_instr;
			end
		end else if (active) begin
			if (delay == 3'd0) begin
				active  <= 1'b0;
				mem_ack <= 1'b1;
				if (mem_write) begin
					case (mem_size)
					`AP040_SZ_L: begin
						mem[widx]     <= mem_wdata[31:16];
						mem[widx1]    <= mem_wdata[15:0];
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
					`AP040_SZ_L: mem_rdata <= {mem[widx], mem[widx1]};
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

// When each patch target's longword was first fetched, and when its store
// went out: [0] $418, [1] $0.
time pt_fetch [0:1];
time pt_store [0:1];
initial begin pt_fetch[0] = 0; pt_fetch[1] = 0; pt_store[0] = 0; pt_store[1] = 0; end
always @(posedge clk) if (nreset && !active && mem_req && !mem_ack) begin
	if (mem_instr && mem_addr == 32'h0000_0418 && pt_fetch[0] == 0) pt_fetch[0] = $time;
	if (mem_instr && mem_addr == 32'h0000_0000 && pt_fetch[1] == 0) pt_fetch[1] = $time;
	if (mem_write && mem_addr == 32'h0000_0418 && pt_store[0] == 0) pt_store[0] = $time;
	if (mem_write && mem_addr == 32'h0000_0000 && pt_store[1] == 0) pt_store[1] = $time;
end

// +bustrace prints every transaction as the memory accepts it.
reg bustrace = 1'b0;
initial bustrace = $test$plusargs("bustrace");
always @(posedge clk) if (nreset && bustrace && !active && mem_req && !mem_ack)
	$display("%0t bus %s %h size %0d fc %0d  if_pc %h ex_pc %h sr %h", $time,
	         mem_write ? "W" : (mem_instr ? "I" : "R"), mem_addr, mem_size, mem_fc,
	         dbg_if_pc, dbg_ex_pc, dbg_sr);

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
	begin mem[(addr >> 1) & (MEM_WORDS - 1)] = word; end
endtask

initial begin
	for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = `AP040_OP_NOP;
	for (i = 0; i < MEM_WORDS / 2; i = i + 1) fc_last[i] = 3'd0;
	for (i = 32'h043A; i < 32'h04A2; i = i + 2) poke(i, 16'h78FF);   // poison up to the far target

	// EX held while the window runs ahead. The window gains on the fetch unit
	// only while that consumes less than the bus delivers -- a longword per
	// ten cycles or so here -- so the divides are one word each.
	poke(32'h03F0, 16'h7064);                                   // $3F0 MOVEQ #100,D0
	poke(32'h03F2, 16'h7201);                                   // $3F2 MOVEQ #1,D1
	for (i = 32'h03F4; i < 32'h0402; i = i + 2) poke(i, 16'h80C1); // $3F4-$400 DIVU.W D1,D0 seven times
	poke(32'h0402, 16'h80FC);  poke(32'h0404, 16'h0007);        // $402 DIVU.W #7,D0
	poke(32'h0406, 16'h31FC);  poke(32'h0408, 16'h7A55);  poke(32'h040A, 16'h0418); // $406 MOVE.W #$7A55,$0418.W  patch: MOVEQ #$55,D5
	poke(32'h040C, 16'h4E71);                                   // $40C NOP
	poke(32'h040E, 16'h4E71);                                   // $40E NOP
	poke(32'h0410, 16'h4E71);                                   // $410 NOP
	poke(32'h0412, 16'h4E71);                                   // $412 NOP
	poke(32'h0414, 16'h4E71);                                   // $414 NOP
	poke(32'h0416, 16'h4E71);                                   // $416 NOP
	poke(32'h0418, 16'h7A11);                                   // $418 MOVEQ #$11,D5  (patched): in the window, not yet in the pipe
	poke(32'h041A, 16'h7C66);                                   // $41A MOVEQ #$66,D6
	poke(32'h041C, 16'h7E77);                                   // $41C MOVEQ #$77,D7
	poke(32'h041E, 16'h4E71);                                   // $41E NOP
	poke(32'h0420, 16'h7403);                                   // $420 MOVEQ #3,D2                loop counter
	poke(32'h0422, 16'h7600);                                   // $422 MOVEQ #0,D3
	poke(32'h0424, 16'h5283);                                   // $424 ADDQ.L #1,D3
	poke(32'h0426, 16'h2238);  poke(32'h0428, 16'h0800);        // $426 MOVE.L $0800.W,D1          a load in the loop
	poke(32'h042A, 16'h51CA);  poke(32'h042C, 16'hFFF8);        // $42A DBF D2,loop
	poke(32'h042E, 16'h6006);                                   // $42E BRA.B +6                   inside the window
	poke(32'h0430, 16'h78FF);                                   // $430 MOVEQ #-1,D4               poison
	poke(32'h0432, 16'h78FF);                                   // $432 poison
	poke(32'h0434, 16'h78FF);                                   // $434 poison
	poke(32'h0436, 16'h6000);  poke(32'h0438, 16'h006A);        // $436 BRA.W far                  past it, mid-longword
	poke(32'h04A2, 16'h4EF9);  poke(32'h04A4, 16'hFFFF);  poke(32'h04A6, 16'hFFD8); // $4A2 JMP ($FFFFFFD8).L    the far target
	poke(32'h04A8, 16'h7022);                                        // $4A8 MOVEQ #$22,D0            back from the top
	poke(32'h04AA, 16'h46FC);                                        // $4AA MOVE #$0000,SR           user mode
	poke(32'h04AC, 16'h0000);
	poke(32'h04AE, 16'h5280);                                        // $4AE ADDQ.L #1,D0
	poke(32'h04B0, 16'h60FE);                                        // $4B0 BRA.B -2                 fetched as user program

	// Across the top: the $402 patch again, with the MOVEQ past the wrap.
	poke(32'hFFFF_FFD8, 16'h7064);                                   // $FFFFFFD8 MOVEQ #100,D0
	for (i = 32'hFFFF_FFDA; i < 32'hFFFF_FFEA; i = i + 2) poke(i, 16'h80C3); // $FFFFFFDA-E8 DIVU.W D3,D0 eight times (D3 = 4)
	poke(32'hFFFF_FFEA, 16'h80FC);  poke(32'hFFFF_FFEC, 16'h0007);   // $FFFFFFEA DIVU.W #7,D0
	poke(32'hFFFF_FFEE, 16'h31FC);  poke(32'hFFFF_FFF0, 16'h7055);  poke(32'hFFFF_FFF2, 16'h0000); // $FFFFFFEE MOVE.W #$7055,$0000.W
	poke(32'hFFFF_FFF4, 16'h4E71);                                   // $FFFFFFF4 NOP
	poke(32'hFFFF_FFF6, 16'h4E71);                                   // $FFFFFFF6 NOP
	poke(32'hFFFF_FFF8, 16'h4E71);                                   // $FFFFFFF8 NOP
	poke(32'hFFFF_FFFA, 16'h4E71);                                   // $FFFFFFFA NOP
	poke(32'hFFFF_FFFC, 16'h4E71);                                   // $FFFFFFFC NOP
	poke(32'hFFFF_FFFE, 16'h4E71);                                   // $FFFFFFFE NOP
	poke(32'h0000_0000, 16'h7011);                                   // $0 MOVEQ #$11,D0 (patched): in the window, not yet in the pipe
	poke(32'h0000_0002, 16'h31C0);  poke(32'h0000_0004, 16'h0A00);   // $2 MOVE.W D0,$0A00.W
	poke(32'h0000_0006, 16'h4EF9);  poke(32'h0000_0008, 16'h0000);  poke(32'h0000_000A, 16'h04A8); // $6 JMP ($04A8).L
	poke(32'h0000_0A00, 16'h0000);

	poke(32'h0800, 16'hCAFE); poke(32'h0802, 16'hF00D);
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;
	@(posedge clk);
	dut.u_cpu.u_regfile.isp = 32'h0000_0600;
	repeat (2) @(posedge clk);
	ce_run = 1;

	repeat ((PROG_WORDS + 4000) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	check32("D5 (patched MOVEQ)", dbg_d5, 32'h0000_0055);
	check32("D6", dbg_d6, 32'h0000_0066);
	check32("D7", dbg_d7, 32'h0000_0077);
	check32("D3 (loop iterations)", dbg_d3, 32'h0000_0004);
	check32("D2 (DBF counter)", dbg_d2, 32'h0000_FFFF);
	check32("D1 (the load)", dbg_d1, 32'hCAFE_F00D);
	check32("D4 (poison never ran)", dbg_d4, 32'h0000_0000);
	check32("D0 (far target, then user mode)", dbg_d0, 32'h0000_0023);
	check32("SR", {16'd0, dbg_sr}, 32'h0000_0000);
	check32("function code $4B0 was fetched with", {29'd0, fc_last[32'h04B0 >> 2]}, {29'd0, `AP040_FC_USER_PROG});
	// Both patches must have met their target IN the window: fetched before
	// the store went out. The divider going from one step a cycle to four
	// once left the store ahead of the window, and the bench passed with the
	// case it was written for never happening.
	if (!(pt_fetch[0] > 0 && pt_store[0] > pt_fetch[0])) begin
		errors = errors + 1;
		$display("FAIL: $418 was not in the window when its patch went out (fetched at %0t, stored at %0t)", pt_fetch[0], pt_store[0]);
	end
	if (!(pt_fetch[1] > 0 && pt_store[1] > pt_fetch[1])) begin
		errors = errors + 1;
		$display("FAIL: $0 was not in the window when its patch went out (fetched at %0t, stored at %0t)", pt_fetch[1], pt_store[1]);
	end
	check32("$A00 (patched MOVEQ past the wrap)", {16'd0, mem[32'h0A00 >> 1]}, 32'h0000_0055);
	check32("first transaction out of reset", first_addr, PC_RESET);
	check32("first transaction was a fetch", {31'd0, first_instr}, 32'd1);

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
