//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24)                   //
//                                                                          //
// tb_ap040_pipe_dblfault_bus16.v - a fault while taking an exception halts //
//                                                                          //
// ap040_core.v halts (fatal_halt) on any access fault during exception     //
// processing -- a frame write or the vector read -- and on an odd handler  //
// for vector 2 or 3 (milestone 110), and only reset ends it. The pipelined //
// core halted for the odd vector by retiring the entry as a STOP, and      //
// since bundle 9 an interrupt above the mask wakes a STOP: the double      //
// fault woke too. A refused frame write was withdrawn and presented again, //
// refused again, for ever; a faulted vector read was used as the handler.  //
//                                                                          //
// On ap040_pipe_bus16.v (the MMU, the 16-bit adapter), ISP $1000:          //
//   A  DTT0 = $C004 write-protects every data write, both modes; MOVE.L    //
//      D0,(A0) is refused, and so is its format $7 frame's first beat      //
//   B  TRAP #0, with a physical bus error on the read of vector 32         //
//   C  JMP to $701: an address error whose vector 3 holds $781 (odd)       //
//   D  TRAP #0, with a physical bus error on its frame's first write       //
//      sub-cycle: the write was posted, so the DMU holds the fault (caches //
//      stage R), and the entry departs as the halt                         //
// Vectors 2, 3 and 32 otherwise lead to $700, which sets D7 = 7: it must   //
// never run. Once the machine is quiet, IPL 7 is raised for 400 cycles and //
// nothing may happen: no bus cycle, no commit, D7 unchanged. Both with ce  //
// always on and pseudo-random.                                            //
//                                                                          //
// Checked per case: no write sub-cycle outside the frame (A: none at all;  //
// B: the four of TRAP's format $0 frame; C: the six of the format $2       //
// frame; D: three of the four, the erring longword's second word never     //
// sent), the machine halted, and an interrupt does not wake it.            //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_dblfault_bus16;

reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0;
reg ce = 1;
reg [2:0] ipl = 3'd0;
integer cemode = 0;
integer errors = 0;

reg [15:0] lfsr = 16'hACE1;
always @(negedge clk)
	if (nreset && cemode) begin
		lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
		ce   <= lfsr[0];
	end else ce <= 1'b1;

wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds, longword;
wire  [1:0] busstate;
wire  [2:0] fc;
wire [31:0] d7, commits;
reg         mem_ready;
wire        clkena_in = (busstate == `AP040_BUS_IDLE) | mem_ready;
reg  [15:0] mem [0:4095];
wire [15:0] data_in = mem[addr_out[12:1]];
reg         vec_berr;       // case B: a bus error on the read of vector 32
reg         frm_berr;       // case D: ...on the frame's first write sub-cycle
wire        berr = nreset && ((vec_berr && (busstate == `AP040_BUS_READ) && mem_ready &&
                               (addr_out == 32'h80 || addr_out == 32'h82)) ||
                              (frm_berr && (busstate == `AP040_BUS_WRITE) && mem_ready &&
                               (addr_out == 32'hFF8)));

ap040_pipe_bus16 #(.PC_RESET(32'h400), .PROG_WORDS(32'h7FFF_FFFF), .RESET_VECTORS(1)) dut
(
	.clk (clk), .nreset (nreset), .ce (ce), .irq_lvl (ipl), .clkena_in (clkena_in),
	.berr (berr),
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // TTRs only: no walks
	.data_in (data_in), .addr_out (addr_out), .data_write (data_write),
	.nwr (nwr), .nuds (nuds), .nlds (nlds), .busstate (busstate), .longword (longword), .fc (fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid (), .dbg_eac_pc (), .dbg_eaf_valid (), .dbg_eaf_pc (),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0 (), .dbg_d1 (), .dbg_d2 (), .dbg_d3 (), .dbg_d4 (), .dbg_d5 (), .dbg_d6 (), .dbg_d7 (d7),
	.dbg_ccr (), .dbg_sr (), .dbg_commits (commits)
);

integer writes, stray, cycles_bus;
reg [31:0] lo, hi;          // where the case's frame may be written
reg [1:0] dly;
always @(posedge clk) begin
	if (!nreset) begin
		mem_ready <= 1'b0; dly <= 2'd0;
	end else begin
		mem_ready <= 1'b0;
		if (busstate != `AP040_BUS_IDLE && !mem_ready) begin
			if (dly == 2'd0) begin
				mem_ready  <= 1'b1;
				dly        <= lfsr[3:2];
				cycles_bus = cycles_bus + 1;
				if (busstate == `AP040_BUS_WRITE) begin
					writes = writes + 1;
					if (addr_out < lo || addr_out >= hi) begin
						stray = stray + 1;
						$display("FAIL: stray write %h <- %h", addr_out, data_write);
					end
					if (!nuds) mem[addr_out[12:1]][15:8] <= data_write[15:8];
					if (!nlds) mem[addr_out[12:1]][7:0]  <= data_write[7:0];
				end
			end else dly <= dly - 2'd1;
		end
	end
end

integer i, t, n, bus0, com0, want_w;
task run_case;
	input integer c;
	begin
		nreset = 0; ipl = 3'd0;
		for (i = 0; i < 4096; i = i + 1) mem[i] = 16'h0000;
		mem[0] = 16'h0000; mem[1] = 16'h1000; mem[2] = 16'h0000; mem[3] = 16'h0400;   // ISP, PC
		for (i = 2; i < 64; i = i + 1) begin mem[2*i] = 16'h0000; mem[2*i + 1] = 16'h0700; end   // every vector: $700
		mem[16'h380] = 16'h7E07; mem[16'h381] = 16'h60FE;                              // $700: MOVEQ #7,D7 / BRA.S *
		vec_berr = 1'b0; frm_berr = 1'b0; lo = 32'h1000; hi = 32'h1000; want_w = 0;
		case (c)
		0: begin   // A: the frame's first write refused
			mem[16'h200] = 16'h41F9; mem[16'h201] = 16'h0000; mem[16'h202] = 16'h0800;
			mem[16'h203] = 16'h203C; mem[16'h204] = 16'h0000; mem[16'h205] = 16'hC004;
			mem[16'h206] = 16'h4E7B; mem[16'h207] = 16'h0006;
			mem[16'h208] = 16'h2080; mem[16'h209] = 16'h60FE;
		end
		1: begin   // B: TRAP #0's vector read faults
			mem[16'h200] = 16'h4E40; mem[16'h201] = 16'h60FE;
			vec_berr = 1'b1; lo = 32'hFF8; want_w = 4;
		end
		2: begin   // C: address error, vector 3 odd
			mem[16'h200] = 16'h41F9; mem[16'h201] = 16'h0000; mem[16'h202] = 16'h0701;
			mem[16'h203] = 16'h4ED0; mem[16'h204] = 16'h60FE;
			mem[6] = 16'h0000; mem[7] = 16'h0781;
			lo = 32'hFF4; want_w = 6;
		end
		default: begin   // D: TRAP #0's frame write errs on the bus
			mem[16'h200] = 16'h4E40; mem[16'h201] = 16'h60FE;
			frm_berr = 1'b1; lo = 32'hFF8; want_w = 3;
		end
		endcase
		writes = 0; stray = 0; cycles_bus = 0;
		repeat (5) @(posedge clk);
		nreset = 1;
		repeat (1500) @(posedge clk);
		n = errors;
		if (!dut.u_cpu.halted) begin errors = errors + 1; $display("FAIL: case %0d: not halted", c); end
		if (d7 === 32'd7) begin errors = errors + 1; $display("FAIL: case %0d: a handler ran", c); end
		if (writes != want_w) begin errors = errors + 1; $display("FAIL: case %0d: %0d write sub-cycles, want %0d", c, writes, want_w); end
		errors = errors + stray;
		// ...and an interrupt does not wake it
		bus0 = cycles_bus; com0 = commits;
		ipl = 3'd7;
		repeat (400) @(posedge clk);
		if (cycles_bus != bus0) begin errors = errors + 1; $display("FAIL: case %0d: %0d bus cycles after IPL 7", c, cycles_bus - bus0); end
		if (commits != com0)    begin errors = errors + 1; $display("FAIL: case %0d: %0d commits after IPL 7", c, commits - com0); end
		if (d7 === 32'd7)       begin errors = errors + 1; $display("FAIL: case %0d: IPL 7 reached a handler", c); end
		$display("ce %0s case %0s: %0s (%0d write sub-cycles)", cemode ? "random" : "on",
		         (c == 0) ? "A" : (c == 1) ? "B" : (c == 2) ? "C" : "D",
		         (errors == n) ? "halted" : "FAILED", writes);
	end
endtask

initial begin
	for (cemode = 0; cemode < 2; cemode = cemode + 1) begin
		run_case(0);
		run_case(1);
		run_case(2);
		run_case(3);
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
