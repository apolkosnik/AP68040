//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24)                   //
//                                                                          //
// tb_ap040_pipe_smcdual.v - stores onto instructions already fetched       //
//                                                                          //
// A store that rewrites an instruction the pipeline has already fetched   //
// behind it: ap040_pipe_membus.v snoops its prefetch stream, and           //
// ap040_pipe_cpu.v snoops EA-calc, decode's gather and the fetch word,    //
// and the storing instruction refetches what follows. rtl/ap040/          //
// ap040_core.v, which the programs were written against, executes one    //
// instruction at a time and snoops its queue, so it always runs the        //
// rewritten instruction; this bench runs the same generated programs on   //
// both cores and compares what they leave, and checks the log against the  //
// values the rewritten instructions must produce.                          //
//                                                                          //
// Each case points A0 at an instruction 0 to 3 instructions ahead and      //
// rewrites it through one of the store routes, with a divide just before   //
// so the fetch runs ahead:                                                 //
//   MOVE.W D0,(A0)        EA-fetch's store (judged a cycle late)            //
//   MOVE.W #imm,(A0)      EX's store beat                                   //
//   ADDQ.W #1,(A0)        EX's read-modify-write                            //
//   MOVEM.W D0/D1,(A0)    EA-fetch's sequencer, two words, two instructions //
// The target is MOVEQ #k,D5 (and for MOVEM, MOVEQ #m,D3 after it); the     //
// case then logs D5 and D3.                                                //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_smcdual;

localparam integer NCASE     = 40;
localparam integer NROUND    = 12;
localparam [31:0] PROG_BASE  = 32'h0000_0400;
localparam [31:0] HANDLER    = 32'h0000_0300;
localparam [31:0] DONE_ADDR  = 32'h0000_1100;
localparam [31:0] LOG_BASE   = 32'h0000_1300;
localparam [31:0] LOG_END    = 32'h0000_1B00;
localparam [31:0] STACK_TOP  = 32'h0000_3000;
localparam integer MEM_WORDS = 32768;
localparam integer TIMEOUT   = 600000;

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

reg [15:0] prog [0:8191];
integer    pw;
reg [31:0] rnd;

function [31:0] xorshift32;
	input [31:0] s;
	reg   [31:0] t;
	begin
		t = s; t = t ^ (t << 13); t = t ^ (t >> 17); t = t ^ (t << 5);
		xorshift32 = t;
	end
endfunction

function [31:0] rbits;
	input integer n;
	begin
		rnd = xorshift32(rnd);
		rbits = (n >= 32) ? rnd : (rnd & ((32'd1 << n) - 32'd1));
	end
endfunction

task emit;
	input [15:0] w;
	begin prog[pw] = w; pw = pw + 1; end
endtask

function [31:0] here;
	input integer dummy;
	begin here = PROG_BASE + 2 * pw; end
endfunction

// The values the log must hold, in order.
reg [31:0] expv [0:1023];
integer    nexp;
integer    n_route [0:3];
integer    c_route [0:511], c_ahead [0:511], c_div [0:511];
integer    c, route, ahead, j, lea_at, kk, k2, mm, m2, divp;
reg [31:0] t, d;

task gen_program;
	input [31:0] seed;
	begin
		rnd = seed; pw = 0; nexp = 0;
		emit(16'h287C); emit(16'h0000); emit(LOG_BASE[15:0]);   // MOVEA.L #LOG_BASE,A4
		emit(16'h7A00); emit(16'h7600);                          // MOVEQ #0,D5 / MOVEQ #0,D3
		for (c = 0; c < NCASE; c = c + 1) begin
			route = rbits(2);
			ahead  = rbits(2);
			divp  = rbits(2) != 0;
			kk = rbits(7); k2 = rbits(7); mm = rbits(7); m2 = rbits(7);
			if (route == 2) k2 = kk + 1;                          // ADDQ.W #1 steps the MOVEQ's immediate
			lea_at = pw;
			emit(16'h41FA); emit(16'h0000);                       // LEA target(PC),A0 (patched)
			emit(16'h303C); emit({8'h7A, k2[7:0]});               // MOVE.W #(MOVEQ #k2,D5),D0
			emit(16'h323C); emit({8'h76, m2[7:0]});               // MOVE.W #(MOVEQ #m2,D3),D1
			if (divp) begin
				emit(16'h7464);                                   // MOVEQ #100,D2
				emit(16'h84FC); emit(16'h0003);                   // DIVU.W #3,D2 -- the fetch runs ahead
			end
			case (route)
			0: emit(16'h3080);                                    // MOVE.W D0,(A0)
			1: begin emit(16'h30BC); emit({8'h7A, k2[7:0]}); end  // MOVE.W #imm,(A0)
			2: emit(16'h5250);                                    // ADDQ.W #1,(A0)
			default: begin emit(16'h4890); emit(16'h0003); end    // MOVEM.W D0/D1,(A0)
			endcase
			n_route[route] = n_route[route] + 1;
			c_route[c] = route; c_ahead[c] = ahead; c_div[c] = divp;
			for (j = 0; j < ahead; j = j + 1) emit(16'h5284);       // ADDQ.L #1,D4
			t = here(0);
			d = t - (PROG_BASE + 2 * lea_at + 2);
			if ($test$plusargs("showcases"))
				$display("case %0d: route %0d, %0d ahead, divide %0d, target %h", c, route, ahead, divp, t);
			prog[lea_at + 1] = d[15:0];
			emit({8'h7A, kk[7:0]});                               // target: MOVEQ #k,D5
			emit({8'h76, mm[7:0]});                               // MOVEQ #m,D3 (MOVEM's second word)
			emit(16'h28C5); emit(16'h28C3);                       // MOVE.L D5,(A4)+ / MOVE.L D3,(A4)+
			expv[nexp] = {{24{k2[7]}}, k2[7:0]};                 nexp = nexp + 1;
			expv[nexp] = (route == 3) ? {{24{m2[7]}}, m2[7:0]} : {{24{mm[7]}}, mm[7:0]}; nexp = nexp + 1;
		end
		emit(16'h7001);                                           // MOVEQ #1,D0
		emit(16'h23C0); emit(DONE_ADDR[31:16]); emit(DONE_ADDR[15:0]);
		emit(16'h60FE);
	end
endtask

reg [15:0] memp [0:MEM_WORDS-1];
reg [15:0] memf [0:MEM_WORDS-1];
integer i;

task put;
	input [31:0] addr;
	input [15:0] word;
	begin memp[addr >> 1] = word; memf[addr >> 1] = word; end
endtask

task build_memory;
	input [31:0] seed;
	begin
		gen_program(seed);
		for (i = 0; i < MEM_WORDS; i = i + 1) begin
			memp[i] = `AP040_OP_NOP; memf[i] = `AP040_OP_NOP;
		end
		put(32'h0000, STACK_TOP[31:16]); put(32'h0002, STACK_TOP[15:0]);
		put(32'h0004, PROG_BASE[31:16]); put(32'h0006, PROG_BASE[15:0]);
		// Any exception: write -1 to the done flag and stop.
		for (i = 2; i < 64; i = i + 1) begin put(i*4, HANDLER[31:16]); put(i*4 + 2, HANDLER[15:0]); end
		put(HANDLER + 0, 16'h70FF);                                    // MOVEQ #-1,D0
		put(HANDLER + 2, 16'h23C0); put(HANDLER + 4, DONE_ADDR[31:16]); put(HANDLER + 6, DONE_ADDR[15:0]);
		put(HANDLER + 8, 16'h60FE);
		for (i = LOG_BASE >> 1; i < LOG_END >> 1; i = i + 1) begin memp[i] = 16'h0000; memf[i] = 16'h0000; end
		for (i = 0; i < pw; i = i + 1) put(PROG_BASE + 2*i, prog[i]);
		put(DONE_ADDR, 16'h0000); put(DONE_ADDR + 2, 16'h0000);
	end
endtask

wire [31:0] p_addr;
wire [15:0] p_dwrite;
wire        p_nwr, p_nuds, p_nlds, p_longword;
wire  [1:0] p_busstate;
wire  [2:0] p_fc;
reg         p_ready;
wire        p_clkena = (p_busstate == `AP040_BUS_IDLE) | p_ready;
wire [31:0] p_widx   = (p_addr >> 1) & (MEM_WORDS - 1);
wire [15:0] p_din    = memp[p_widx];

ap040_pipe_bus16 #(
	.PC_RESET  (PROG_BASE),
	.PROG_WORDS(1000000)
) dut_p
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.berr (1'b0),   // no bus errors in this bench
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // no MMU walks here
	.clk (clk), .nreset (nreset), .ce (1'b1), .clkena_in (p_clkena),
	.data_in (p_din), .addr_out(p_addr), .data_write(p_dwrite),
	.nwr (p_nwr), .nuds(p_nuds), .nlds(p_nlds),
	.busstate(p_busstate), .longword(p_longword), .fc(p_fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid(), .dbg_eac_pc(), .dbg_eaf_valid(), .dbg_eaf_pc(),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0(), .dbg_d1(), .dbg_d2(), .dbg_d3(),
	.dbg_d4(), .dbg_d5(), .dbg_d6(), .dbg_d7(),
	.dbg_ccr(), .dbg_sr(), .dbg_commits()
);

always @(posedge clk) begin
	if (!nreset) p_ready <= 1'b0;
	else begin
		p_ready <= 1'b0;
		if (p_busstate != `AP040_BUS_IDLE && !p_ready) begin
			p_ready <= 1'b1;
			if (p_busstate == `AP040_BUS_WRITE) begin
				if (!p_nuds) memp[p_widx][15:8] <= p_dwrite[15:8];
				if (!p_nlds) memp[p_widx][7:0]  <= p_dwrite[7:0];
			end
		end
	end
end

wire [31:0] f_addr;
wire [15:0] f_dwrite;
wire        f_nwr, f_nuds, f_nlds, f_longword, f_nresetout;
wire  [1:0] f_busstate;
wire  [2:0] f_fc;
reg         f_ready;
wire        f_clkena = (f_busstate == `AP040_BUS_IDLE) | f_ready;
wire [31:0] f_widx   = (f_addr >> 1) & (MEM_WORDS - 1);
wire [15:0] f_din    = memf[f_widx];

ap040_tg68k_compat #(
	.AP040_HAS_MMU(0), .AP040_HAS_FPU(1), .AP040_ENABLE_CACHE(0)
) dut_f
(
	.clk(clk), .nreset(nreset),
	.cache_allow_all(1'b1), .cache_snoop_stb(1'b0), .cache_snoop_addr(32'd0),
	.cache_z2_ena(1'b0), .cache_z3_base0(5'd0), .cache_z3_ena0(1'b0),
	.cache_z3_base1(4'd0), .cache_z3_ena1(1'b0),
	.clkena_in(f_clkena), .bus_clkena_in(f_clkena), .tick_in(1'b1),
	.data_in(f_din), .ipl(3'b111), .ipl_autovector(1'b1), .berr(1'b0),
	.addr_out(f_addr), .data_write(f_dwrite),
	.nwr(f_nwr), .nuds(f_nuds), .nlds(f_nlds),
	.busstate(f_busstate), .longword(f_longword), .post_drain(),
	.nresetout(f_nresetout), .fc(f_fc),
	.nmi_ack_toggle(), .cache_maint_req(), .cache_maint_ic(), .cache_maint_dc(),
	.mmu_addr_log(), .mmu_addr_phys(), .mmu_cache_inhibit(),
	.walker_req(), .walker_we(), .walker_addr(), .walker_wdat(),
	.walker_ack(1'b0), .walker_data(32'd0), .walker_berr(1'b0),
	.cache_req(), .cache_addr(), .cache_data(16'd0),
	.cache_ack(1'b0), .cache_burst(), .cache_burst_len(), .cache_ramaddr(),
	.cacr_out(), .vbr_out(),
	.debug_busy(), .debug_fault(), .debug_halted(),
	.debug_status(), .debug_status2(),
	.debug_exception_valid(), .debug_exception()
);

always @(posedge clk) begin
	if (!nreset) f_ready <= 1'b0;
	else begin
		f_ready <= 1'b0;
		if (f_busstate != `AP040_BUS_IDLE && !f_ready) begin
			f_ready <= 1'b1;
			if (f_busstate == `AP040_BUS_WRITE) begin
				if (!f_nuds) memf[f_widx][15:8] <= f_dwrite[15:8];
				if (!f_nlds) memf[f_widx][7:0]  <= f_dwrite[7:0];
			end
		end
	end
end

// +storetrace: EX's store beats with what the snoop compared.
always @(posedge clk)
	if ($test$plusargs("storetrace") && nreset && dut_p.u_cpu.ex_st_req)
		$display("STX %0t a=%h busy=%b eac=%b %h-%h id=%b %h-%h dec=%b %h if=%b %h smc=%b stall=%b", $time,
		         dut_p.u_cpu.snx_a, dut_p.u_cpu.l1_wr_busy, dut_p.u_cpu.eac_valid, dut_p.u_cpu.eac_pc, dut_p.u_cpu.eac_next_pc,
		         dut_p.u_cpu.id_valid, dut_p.u_cpu.id_pc, dut_p.u_cpu.id_next_pc,
		         dut_p.u_cpu.dec_holding, dut_p.u_cpu.dec_hold_pc, dut_p.u_cpu.if_valid_id, dut_p.u_cpu.if_pc,
		         dut_p.u_cpu.st_smc, dut_p.u_cpu.ex_stall);

integer errors = 0;
integer cyc, round, mism, k;
reg [31:0] seed, pdone, fdone, gp;

function [31:0] rd32p; input [31:0] a; rd32p = {memp[a >> 1], memp[(a >> 1) + 1]}; endfunction
function [31:0] rd32f; input [31:0] a; rd32f = {memf[a >> 1], memf[(a >> 1) + 1]}; endfunction

initial begin
	n_route[0] = 0; n_route[1] = 0; n_route[2] = 0; n_route[3] = 0;
	for (round = 0; round < NROUND; round = round + 1) begin
		seed = 32'h5EED_0001 + round * 32'h9E37_79B9;
		nreset = 0;
		repeat (8) @(posedge clk);
		build_memory(seed);
		nreset = 1;
		@(posedge clk);
		@(posedge clk);
		dut_p.u_cpu.u_regfile.isp = STACK_TOP;
		cyc = 0; pdone = 0; fdone = 0;
		while (cyc < TIMEOUT && !(pdone != 0 && fdone != 0)) begin
			@(posedge clk);
			cyc = cyc + 1;
			pdone = rd32p(DONE_ADDR);
			fdone = rd32f(DONE_ADDR);
		end
		mism = 0;
		if (pdone != 32'd1) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the pipelined core did not finish cleanly (done = %h, %0d cycles)", round, seed, pdone, cyc);
		end
		if (fdone != 32'd1) begin
			errors = errors + 1;
			$display("FAIL: round %0d (seed %h): the FSM core did not finish cleanly (done = %h, %0d cycles)", round, seed, fdone, cyc);
		end
		for (k = 0; k < nexp; k = k + 1) begin
			gp = rd32p(LOG_BASE + 4*k);
			if (gp !== expv[k] || gp !== rd32f(LOG_BASE + 4*k)) begin
				errors = errors + 1; mism = mism + 1;
				if (mism <= 6)
					$display("FAIL: round %0d (seed %h): log[%0d] = %h on the pipelined core, %h on the FSM core, %h expected (route %0d, %0d ahead, divide %0d)",
					         round, seed, k, gp, rd32f(LOG_BASE + 4*k), expv[k], c_route[k/2], c_ahead[k/2], c_div[k/2]);
			end
		end
		$display("round %0d: seed %h, %0d program words, %0d cycles, %0d mismatches", round, seed, pw, cyc, mism);
	end
	$display("routes: %0d EA-fetch MOVE.W, %0d EX MOVE.W #imm, %0d EX ADDQ, %0d MOVEM", n_route[0], n_route[1], n_route[2], n_route[3]);
	if (n_route[0] == 0 || n_route[1] == 0 || n_route[2] == 0 || n_route[3] == 0) begin
		errors = errors + 1;
		$display("FAIL: a store route went ungenerated");
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("%0d CHECK(S) FAILED", errors);
	$finish;
end

endmodule
