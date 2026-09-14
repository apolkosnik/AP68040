//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// tb_ap040_reset.v - reset and bus smoke test (implementation plan         //
// milestone B exit gate)                                                   //
//                                                                          //
// Instantiates the real ap040_tg68k_compat top and drives it exactly the   //
// way cpu_wrapper.v does: clkena_in = idle | ready-pulse, one single       //
// cycle ready pulse per completed 16-bit bus sub-cycle.                    //
//                                                                          //
// Verifies:                                                                //
//  - reset vector fetch order and bus state (ISP from 0, PC from 4)        //
//  - first instruction fetch at the reset PC                               //
//  - byte/word/long and misaligned transfer splitting and byte lanes      //
//  - MOVEQ/MOVE/MOVEA/Bcc subset with CCR driven branching                 //
//  - (An)+ / -(An) address register updates                                //
//  - no write cycle before the first instruction fetch                    //
//  - two latency profiles: back-to-back and varied wait states            //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_reset;

reg clk = 0;
reg nreset = 0;

always #5 clk = ~clk;

//---------------------------------------------------------------------------
// DUT
//---------------------------------------------------------------------------

wire [15:0] data_in;
wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds;
wire  [1:0] busstate;
wire        longword;
wire        nresetout;
wire  [2:0] fc;
wire [31:0] cacr_out, vbr_out;
wire        debug_busy, debug_fault, debug_halted;
wire [255:0] debug_status;

reg         mem_ready;
wire        clkena_in = (busstate == 2'b01) | mem_ready;

ap040_tg68k_compat dut
(
	.clk(clk),
	.tick_in(1'b1),        // no P2 tick grid here: the core runs every clock
	.nreset(nreset),
	.cache_allow_all(1'b1),
	.cache_snoop_stb(1'b0), .cache_snoop_addr(32'd0),
	.cache_z2_ena(1'b0),
	.cache_z3_base0(5'd0),
	.cache_z3_ena0(1'b0),
	.cache_z3_base1(4'd0),
	.cache_z3_ena1(1'b0),
	.clkena_in(clkena_in),
	.data_in(data_in),
	.ipl(3'b111),
	.ipl_autovector(1'b1),
	.berr(1'b0),

	.addr_out(addr_out),
	.data_write(data_write),
	.nwr(nwr),
	.nuds(nuds),
	.nlds(nlds),
	.busstate(busstate),
	.longword(longword),
	.nresetout(nresetout),
	.fc(fc),

	.mmu_addr_log(),
	.mmu_addr_phys(),
	.mmu_cache_inhibit(),

	.walker_req(),
	.walker_we(),
	.walker_addr(),
	.walker_wdat(),
	.walker_ack(1'b0),
	.walker_data(32'd0),
	.walker_berr(1'b0),

	.cache_req(),
	.cache_addr(),
	.cache_data(16'd0),
	.cache_ack(1'b0),
	.cache_burst(),
	.cache_burst_len(),
	.cache_ramaddr(),

	.cacr_out(cacr_out),
	.vbr_out(vbr_out),
	.debug_busy(debug_busy),
	.debug_fault(debug_fault),
	.debug_halted(debug_halted),
	.debug_status(debug_status)
);

wire [31:0] dbg_pc = debug_status[31:0];
wire [15:0] dbg_sr = debug_status[47:32];
wire [31:0] dbg_a7 = debug_status[95:64];
wire [31:0] dbg_d0 = debug_status[127:96];
wire [31:0] dbg_d1 = debug_status[159:128];
wire [31:0] dbg_d2 = debug_status[191:160];
wire [31:0] dbg_a0 = debug_status[223:192];

//---------------------------------------------------------------------------
// 16-bit memory model (32 KB), big endian word array
//---------------------------------------------------------------------------

reg [15:0] mem [0:16383];

assign data_in = mem[addr_out[14:1]];

integer errors;
integer phase;          // 0 = back-to-back, 1 = varied wait states
integer subcycles;      // completed 16-bit sub-cycles this phase
integer rd_cnt;         // completed data-read sub-cycles this phase
reg [31:0] rd_log [0:3];
integer loop_hits;
reg seen_fetch;

localparam [31:0] LOOP_PC = 32'h0000_0454;

// deterministic wait state pattern
function [2:0] latency;
	input integer ph;
	input integer n;
	begin
		if (ph == 0) latency = 0;
		else         latency = (n * 7 + 3) % 6;
	end
endfunction

reg [2:0] lat_cnt;
integer lat_idx;

always @(posedge clk) begin
	mem_ready <= 0;
	if (!nreset) begin
		lat_cnt <= latency(phase, 0);
		lat_idx <= 1;
	end
	else if (busstate != 2'b01 && !mem_ready) begin
		if (lat_cnt == 0) begin
			mem_ready <= 1;
			lat_cnt   <= latency(phase, lat_idx);
			lat_idx   <= lat_idx + 1;
		end
		else lat_cnt <= lat_cnt - 1'd1;
	end
end

// loop pass detection: the end loop fits in the core's fetch buffer, so
// after the first pass its fetches never reach the bus, and the buffered
// dispatch never leaves pc parked on the loop head. Count decodes of the
// loop head instruction instead (state 4 = S_DECODE, pc_i = its address).
always @(posedge clk) begin
	if (nreset && dut.core.ce && dut.core.state == 8'd4 &&
	    dut.core.pc_i == LOOP_PC)
		loop_hits = loop_hits + 1;
end

// bus monitor and write commit, at the qualified completion of a sub-cycle
always @(posedge clk) begin
	if (nreset && mem_ready) begin
		subcycles = subcycles + 1;

		if (addr_out[31:15] != 0) begin
			errors = errors + 1;
			$display("FAIL: access outside memory model at %h", addr_out);
		end

		if (busstate == 2'b11) begin
			if (nwr !== 1'b0) begin
				errors = errors + 1;
				$display("FAIL: write busstate with nwr=%b at %h", nwr, addr_out);
			end
			if (!seen_fetch) begin
				errors = errors + 1;
				$display("FAIL: write to %h before first instruction fetch", addr_out);
			end
			if (!nuds) mem[addr_out[14:1]][15:8] = data_write[15:8];
			if (!nlds) mem[addr_out[14:1]][7:0]  = data_write[7:0];
		end
		else begin
			if (nwr !== 1'b1) begin
				errors = errors + 1;
				$display("FAIL: read busstate with nwr=%b at %h", nwr, addr_out);
			end
		end

		if (busstate == 2'b10 && rd_cnt < 4) begin
			rd_log[rd_cnt] = addr_out;
			rd_cnt = rd_cnt + 1;
		end

		if (busstate == 2'b00 && !seen_fetch) begin
			seen_fetch = 1;
			if (addr_out !== 32'h0000_0400) begin
				errors = errors + 1;
				$display("FAIL: first fetch at %h, expected 00000400", addr_out);
			end
		end


		// strobe checks for the byte write to $1007 (odd, LDS only)
		if (busstate == 2'b11 && addr_out == 32'h0000_1007) begin
			if (nuds !== 1'b1 || nlds !== 1'b0 || data_write[7:0] !== 8'h42) begin
				errors = errors + 1;
				$display("FAIL: byte write $1007 strobes/data uds=%b lds=%b data=%h",
				         nuds, nlds, data_write);
			end
		end
		// strobe checks for the word write to $1004
		if (busstate == 2'b11 && addr_out == 32'h0000_1004) begin
			if (nuds !== 1'b0 || nlds !== 1'b0 || data_write !== 16'h0042) begin
				errors = errors + 1;
				$display("FAIL: word write $1004 strobes/data uds=%b lds=%b data=%h",
				         nuds, nlds, data_write);
			end
		end
	end
end

//---------------------------------------------------------------------------
// program image
//---------------------------------------------------------------------------

task load_program;
	integer i;
	begin
		for (i = 0; i < 16384; i = i + 1) mem[i] = 16'hCAFE;

		// reset vectors: ISP = $00004000, PC = $00000400
		mem['h000 >> 1] = 16'h0000;
		mem['h002 >> 1] = 16'h4000;
		mem['h004 >> 1] = 16'h0000;
		mem['h006 >> 1] = 16'h0400;

		mem['h400 >> 1] = 16'h7042;   // MOVEQ  #$42,D0
		mem['h402 >> 1] = 16'h23C0;   // MOVE.L D0,$1000.L
		mem['h404 >> 1] = 16'h0000;
		mem['h406 >> 1] = 16'h1000;
		mem['h408 >> 1] = 16'h33C0;   // MOVE.W D0,$1004.L
		mem['h40A >> 1] = 16'h0000;
		mem['h40C >> 1] = 16'h1004;
		mem['h40E >> 1] = 16'h13C0;   // MOVE.B D0,$1007.L
		mem['h410 >> 1] = 16'h0000;
		mem['h412 >> 1] = 16'h1007;
		mem['h414 >> 1] = 16'h223C;   // MOVE.L #$DEADBEEF,D1
		mem['h416 >> 1] = 16'hDEAD;
		mem['h418 >> 1] = 16'hBEEF;
		mem['h41A >> 1] = 16'h23C1;   // MOVE.L D1,$1008.L
		mem['h41C >> 1] = 16'h0000;
		mem['h41E >> 1] = 16'h1008;
		mem['h420 >> 1] = 16'h23C1;   // MOVE.L D1,$1019.L (misaligned)
		mem['h422 >> 1] = 16'h0000;
		mem['h424 >> 1] = 16'h1019;
		mem['h426 >> 1] = 16'h7400;   // MOVEQ  #0,D2
		mem['h428 >> 1] = 16'h6600;   // BNE.W  +$7FF0 (not taken)
		mem['h42A >> 1] = 16'h7FF0;
		mem['h42C >> 1] = 16'h6706;   // BEQ.S  +6 -> $434
		mem['h42E >> 1] = 16'h23C1;   // MOVE.L D1,$1010.L (must be skipped)
		mem['h430 >> 1] = 16'h0000;
		mem['h432 >> 1] = 16'h1010;
		mem['h434 >> 1] = 16'h33C2;   // MOVE.W D2,$1014.L
		mem['h436 >> 1] = 16'h0000;
		mem['h438 >> 1] = 16'h1014;
		mem['h43A >> 1] = 16'h207C;   // MOVEA.L #$2000,A0
		mem['h43C >> 1] = 16'h0000;
		mem['h43E >> 1] = 16'h2000;
		mem['h440 >> 1] = 16'h30C0;   // MOVE.W D0,(A0)+
		mem['h442 >> 1] = 16'h3080;   // MOVE.W D0,(A0)
		mem['h444 >> 1] = 16'h2439;   // MOVE.L $1008.L,D2
		mem['h446 >> 1] = 16'h0000;
		mem['h448 >> 1] = 16'h1008;
		mem['h44A >> 1] = 16'h3420;   // MOVE.W -(A0),D2
		mem['h44C >> 1] = 16'h1439;   // MOVE.B $1019.L,D2
		mem['h44E >> 1] = 16'h0000;
		mem['h450 >> 1] = 16'h1019;
		mem['h452 >> 1] = 16'h4E71;   // NOP
		mem['h454 >> 1] = 16'h60FE;   // BRA.S * (loop)
	end
endtask

//---------------------------------------------------------------------------
// checks
//---------------------------------------------------------------------------

task check_word;
	input [31:0] addr;
	input [15:0] expval;
	begin
		if (mem[addr[14:1]] !== expval) begin
			errors = errors + 1;
			$display("FAIL: mem[%h] = %h, expected %h", addr, mem[addr[14:1]], expval);
		end
	end
endtask

task check_reg;
	input [8*4-1:0] name;
	input [31:0] got;
	input [31:0] expval;
	begin
		if (got !== expval) begin
			errors = errors + 1;
			$display("FAIL: %s = %h, expected %h", name, got, expval);
		end
	end
endtask

task check_results;
	begin
		// reset vector fetches: data reads at 0,2,4,6 in that order
		if (rd_cnt < 4) begin
			errors = errors + 1;
			$display("FAIL: fewer than 4 data-read sub-cycles logged");
		end
		else begin
			if (rd_log[0] !== 32'h0) begin errors = errors + 1; $display("FAIL: vec rd0 %h", rd_log[0]); end
			if (rd_log[1] !== 32'h2) begin errors = errors + 1; $display("FAIL: vec rd1 %h", rd_log[1]); end
			if (rd_log[2] !== 32'h4) begin errors = errors + 1; $display("FAIL: vec rd2 %h", rd_log[2]); end
			if (rd_log[3] !== 32'h6) begin errors = errors + 1; $display("FAIL: vec rd3 %h", rd_log[3]); end
		end

		check_word(32'h1000, 16'h0000);
		check_word(32'h1002, 16'h0042);
		check_word(32'h1004, 16'h0042);
		check_word(32'h1006, 16'hCA42);  // byte write to $1007, upper byte kept
		check_word(32'h1008, 16'hDEAD);
		check_word(32'h100A, 16'hBEEF);
		check_word(32'h1010, 16'hCAFE);  // skipped by taken BEQ
		check_word(32'h1012, 16'hCAFE);
		check_word(32'h1014, 16'h0000);
		check_word(32'h1018, 16'hCADE);  // misaligned long $1019: DE AD BE EF
		check_word(32'h101A, 16'hADBE);
		check_word(32'h101C, 16'hEFFE);
		check_word(32'h2000, 16'h0042);
		check_word(32'h2002, 16'h0042);

		check_reg("A7", dbg_a7, 32'h0000_4000);
		check_reg("D0", dbg_d0, 32'h0000_0042);
		check_reg("D1", dbg_d1, 32'hDEAD_BEEF);
		check_reg("D2", dbg_d2, 32'hDEAD_00DE);
		check_reg("A0", dbg_a0, 32'h0000_2000);

		if (dbg_pc !== LOOP_PC && dbg_pc !== LOOP_PC + 2) begin
			errors = errors + 1;
			$display("FAIL: PC = %h, expected loop at %h", dbg_pc, LOOP_PC);
		end

		// last flag update: MOVE.B $1019 (value $DE): N=1 Z=0 V=0 C=0
		if (dbg_sr[3:0] !== 4'b1000) begin
			errors = errors + 1;
			$display("FAIL: CCR NZVC = %b, expected 1000", dbg_sr[3:0]);
		end
		if (dbg_sr[13] !== 1'b1) begin
			errors = errors + 1;
			$display("FAIL: SR.S = %b, expected 1 (supervisor)", dbg_sr[13]);
		end

		if (debug_fault !== 1'b0 || debug_halted !== 1'b0) begin
			errors = errors + 1;
			$display("FAIL: fault=%b halted=%b at end of program", debug_fault, debug_halted);
		end

		if (vbr_out !== 32'd0) begin
			errors = errors + 1;
			$display("FAIL: VBR = %h, expected 0 after reset", vbr_out);
		end
	end
endtask

//---------------------------------------------------------------------------
// phase driver
//---------------------------------------------------------------------------

integer timeout;

task run_phase;
	input integer ph;
	begin
		phase      = ph;
		subcycles  = 0;
		rd_cnt     = 0;
		loop_hits  = 0;
		seen_fetch = 0;
		load_program;

		nreset = 0;
		repeat (10) @(posedge clk);
		nreset = 1;

		timeout = 0;
		while (loop_hits < 3 && timeout < 300000) begin
			@(posedge clk);
			timeout = timeout + 1;
		end

		if (loop_hits < 3) begin
			errors = errors + 1;
			$display("FAIL: phase %0d timeout, pc=%h state fault=%b halted=%b",
			         ph, dbg_pc, debug_fault, debug_halted);
		end
		else begin
			repeat (20) @(posedge clk);
			check_results;
		end
		$display("phase %0d done: %0d sub-cycles, %0d errors so far", ph, subcycles, errors);
	end
endtask

`ifdef AP040_TRACE
always @(posedge clk) if (nreset && dut.core.ce) begin
	if (dut.core.state == 4'd3)
		$display("TRACE decode pc=%h ir=%h sr=%h", dut.core.pc, dut.core.ir, dut.core.sr);
	if (dut.core.rf_we)
		$display("TRACE rfwrite waddr=%h wdata=%h", dut.core.rf_waddr, dut.core.rf_wdata);
end
always @(posedge clk) if (nreset && mem_ready)
	$display("TRACE bus %s addr=%h data=%h uds=%b lds=%b",
	         busstate == 2'b00 ? "iftc" : busstate == 2'b11 ? "wr  " : "rd  ",
	         addr_out, busstate == 2'b11 ? data_write : data_in, nuds, nlds);
`endif

initial begin
	errors = 0;
	$display("tb_ap040_reset: AP040 reset and bus smoke test");

	run_phase(0);   // back-to-back ready
	run_phase(1);   // varied wait states

	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
