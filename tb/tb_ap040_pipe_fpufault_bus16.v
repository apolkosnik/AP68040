//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24, review 16)        //
//                                                                          //
// tb_ap040_pipe_fpufault_bus16.v - an FPU operand's access fault           //
//                                                                          //
// The FPU's memory sequencer in ap040_ea_fetch.v was not stopped by an     //
// access fault on one of its own transfers: it took the faulted return as  //
// data and ran on, and because its address outranks the exception frame's //
// on port B, the format $7 frame's first beats went to the operand instead //
// of the stack -- the frame header missing, operand memory overwritten,   //
// and an RTE through that frame unsafe. The handler still ran, so a test   //
// that only watched for vector 2 missed it. This bench watches every      //
// write.                                                                   //
//                                                                          //
// ap040_pipe_bus16.v (the MMU and the 16-bit adapter), ISP $1000, A0 $800: //
//   kind 0  MOVE.L (A0),D0              read fault at $800 (the control)   //
//   kind 1  FMOVE.L (A0),FP0            read fault at $800                 //
//   kind 2  FMOVE.X (A0),FP0            at each of its three longwords     //
//   kind 3  FMOVEM.X (A0),FP0-FP1       at the first, second, third and    //
//                                       fourth longwords                   //
//   kind 4  FMOVE.X FP0,(A0)            refused: DTT0 write-protects user  //
//   kind 5  FMOVEM.X FP0-FP1,(A0)       data, and the store runs in user   //
//                                       mode (the frame, supervisor, is    //
//                                       not refused)                       //
//   kind 6  MOVE.L D0,(A0)              refused the same way: EA-fetch's   //
//   kind 7  MOVE.L #imm,(A0)            store and EX's, the integer        //
//                                       controls for the refusal           //
// Read faults are a one-shot physical bus error on the named sub-cycle.    //
// Every case runs with ce always on, with a pseudo-random ce, and with ce  //
// held low for three cycles from each refusal: the memory side keeps a     //
// refusal until the CPU withdraws its write in a cycle it runs, and one    //
// cleared by a disabled cycle had the write presented again, refused      //
// again -- a schedule that pauses at every refusal never got past it.      //
//                                                                          //
// Checked: the handler runs (it sets D7 = 7); exactly thirty write         //
// sub-cycles, all of them inside $FC4-$FFF, the frame; ISP is $FC4; the    //
// frame's format word is $7008, its PC the faulting instruction, its FA    //
// the faulted transfer, and its SSW's RW, ATC and TM what the access was. //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_fpufault_bus16;

reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0;
reg ce = 1;
integer cemode = 0;
integer errors = 0;

reg [15:0] lfsr = 16'hACE1;
reg        wflt_q = 0;
integer    hold = 0;
always @(negedge clk) begin
	wflt_q <= dut.l1_wflt;
	if (nreset && cemode == 1) begin
		lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
		ce   <= lfsr[0];
	end else if (nreset && cemode == 2) begin
		if (dut.l1_wflt && !wflt_q) hold = 3;
		ce <= (hold == 0);
		if (hold > 0) hold = hold - 1;
	end else ce <= 1'b1;
end

wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds, longword;
wire  [1:0] busstate;
wire  [2:0] fc;
wire [31:0] d7;
reg         mem_ready;
wire        clkena_in = (busstate == `AP040_BUS_IDLE) | mem_ready;
reg  [15:0] mem [0:4095];
wire [15:0] data_in = mem[addr_out[12:1]];
reg  [31:0] faddr;
reg         armed;
wire        berr = nreset && armed && (busstate == `AP040_BUS_READ) && (addr_out == faddr) && mem_ready;

ap040_pipe_bus16 #(.PC_RESET(32'h400), .PROG_WORDS(32'h7FFF_FFFF), .RESET_VECTORS(1)) dut
(
	.clk (clk), .nreset (nreset), .ce (ce), .irq_lvl (3'd0), .clkena_in (clkena_in),
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
	.dbg_ccr (), .dbg_sr (), .dbg_commits ()
);

integer writes, stray, faults;
reg [1:0] dly;
always @(posedge clk) begin
	if (!nreset) begin
		mem_ready <= 1'b0; dly <= 2'd0;
	end else begin
		mem_ready <= 1'b0;
		if (berr) begin
			armed  <= 1'b0;
			faults = faults + 1;
		end
		if (busstate != `AP040_BUS_IDLE && !mem_ready) begin
			if (dly == 2'd0) begin
				mem_ready <= 1'b1;
				dly       <= lfsr[3:2];
				if (busstate == `AP040_BUS_WRITE) begin
					writes = writes + 1;
					if (addr_out < 32'hFC4 || addr_out >= 32'h1000) begin
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

task fail;
	input [8*48:1] what;
	input [31:0] got, want;
	begin
		errors = errors + 1;
		$display("FAIL: %0s: got %h, want %h", what, got, want);
	end
endtask

integer i, t, k, n;
reg [31:0] fpc;
task run_case;
	input integer kind;
	input [31:0] fa;          // the faulted transfer's address
	begin
		nreset = 0;
		for (i = 0; i < 4096; i = i + 1) mem[i] = 16'h0000;
		mem[0] = 16'h0000; mem[1] = 16'h1000; mem[2] = 16'h0000; mem[3] = 16'h0400;   // ISP, PC
		mem[4] = 16'h0000; mem[5] = 16'h0700;                                           // vector 2
		mem[16'h380] = 16'h7E07; mem[16'h381] = 16'h60FE;                              // $700: MOVEQ #7,D7 / BRA.S *
		mem[16'h200] = 16'h41F9; mem[16'h201] = 16'h0000; mem[16'h202] = 16'h0800;     // LEA ($800).L,A0
		fpc = 32'h406;
		case (kind)
		0: begin mem[16'h203] = 16'h2010; mem[16'h204] = 16'h60FE; end                 // MOVE.L (A0),D0
		1: begin mem[16'h203] = 16'hF210; mem[16'h204] = 16'h4000; mem[16'h205] = 16'h60FE; end   // FMOVE.L (A0),FP0
		2: begin mem[16'h203] = 16'hF210; mem[16'h204] = 16'h4800; mem[16'h205] = 16'h60FE; end   // FMOVE.X (A0),FP0
		3: begin mem[16'h203] = 16'hF210; mem[16'h204] = 16'hD0C0; mem[16'h205] = 16'h60FE; end   // FMOVEM.X (A0),FP0-FP1
		default: begin
			// MOVE.L #$8004,D0 / MOVEC D0,DTT0 / ANDI.W #$DFFF,SR / the store
			mem[16'h203] = 16'h203C; mem[16'h204] = 16'h0000; mem[16'h205] = 16'h8004;
			mem[16'h206] = 16'h4E7B; mem[16'h207] = 16'h0006;
			mem[16'h208] = 16'h027C; mem[16'h209] = 16'hDFFF;
			fpc = 32'h414;
			case (kind)
			4: begin mem[16'h20A] = 16'hF210; mem[16'h20B] = 16'h6800; mem[16'h20C] = 16'h60FE; end   // FMOVE.X FP0,(A0)
			5: begin mem[16'h20A] = 16'hF210; mem[16'h20B] = 16'hF0C0; mem[16'h20C] = 16'h60FE; end   // FMOVEM.X FP0-FP1,(A0)
			6: begin mem[16'h20A] = 16'h2080; mem[16'h20B] = 16'h60FE; end                           // MOVE.L D0,(A0)
			default: begin   // MOVE.L #$12345678,(A0)
				mem[16'h20A] = 16'h20BC; mem[16'h20B] = 16'h1234; mem[16'h20C] = 16'h5678; mem[16'h20D] = 16'h60FE;
			end
			endcase
		end
		endcase
		// the operand: 1.0 and 2.0 in extended precision, 24 bytes
		mem[16'h400] = 16'h3FFF; mem[16'h401] = 16'h0000; mem[16'h402] = 16'h8000; mem[16'h403] = 16'h0000;
		mem[16'h404] = 16'h0000; mem[16'h405] = 16'h0000;
		mem[16'h406] = 16'h4000; mem[16'h407] = 16'h0000; mem[16'h408] = 16'h8000; mem[16'h409] = 16'h0000;
		mem[16'h40A] = 16'h0000; mem[16'h40B] = 16'h0000;
		faddr  = fa;
		armed  = (kind < 4);
		writes = 0; stray = 0; faults = 0;
		repeat (5) @(posedge clk);
		nreset = 1;
		t = 0;
		while (t < 6000 && d7 !== 32'd7) begin @(posedge clk); t = t + 1; end
		repeat (100) @(posedge clk);
		n = errors;
		if (d7 !== 32'd7) fail("handler reached (D7)", d7, 32'd7);
		if (kind < 4 && faults != 1) fail("bus errors injected", faults, 1);
		if (writes != 30) fail("write sub-cycles", writes, 30);
		errors = errors + stray;
		if (dut.u_cpu.u_regfile.isp !== 32'hFC4) fail("ISP", dut.u_cpu.u_regfile.isp, 32'hFC4);
		if (mem[12'h7E5] !== 16'h7008) fail("frame format word", mem[12'h7E5], 16'h7008);
		if ({mem[12'h7E3], mem[12'h7E4]} !== fpc) fail("frame PC", {mem[12'h7E3], mem[12'h7E4]}, fpc);
		if ({mem[12'h7EC], mem[12'h7ED]} !== fa) fail("frame FA", {mem[12'h7EC], mem[12'h7ED]}, fa);
		// SSW: RW/ATC/TM -- a supervisor data read with a physical bus error,
		// or a user data write the TTR refused
		if ((mem[12'h7E8] & 16'h0507) !== ((kind < 4) ? 16'h0105 : 16'h0401))
			fail("frame SSW & $0507", mem[12'h7E8] & 16'h0507, (kind < 4) ? 16'h0105 : 16'h0401);
		$display("ce %0s kind %0d fault %h: %0s (%0d write sub-cycles)",
		         (cemode == 0) ? "on" : (cemode == 1) ? "random" : "paused at each refusal", kind, fa,
		         (errors == n) ? "ok" : "FAILED", writes);
	end
endtask

initial begin
	for (cemode = 0; cemode < 3; cemode = cemode + 1) begin
		run_case(0, 32'h800);
		run_case(1, 32'h800);
		for (k = 0; k < 3; k = k + 1) run_case(2, 32'h800 + 4*k);
		for (k = 0; k < 4; k = k + 1) run_case(3, 32'h800 + 4*k);
		for (k = 4; k < 8; k = k + 1) run_case(k, 32'h800);
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
