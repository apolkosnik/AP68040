//--------------------------------------------------------------------------//
// AP68040-60 test bench                                                     //
//                                                                          //
// tb_cputest.sv - WinUAE cputest corpus replay on the 68040 bus             //
//                                                                          //
// tools/cputest/replay_gen.py expands one corpus slice into APR2 records   //
// (input state, oracle state, memory setup and checks, exception frames). //
// Each round runs on the core through its bus only: a monitor program     //
// (asm/cputest_mon.s, at $4211_0000) loads the round's registers from a   //
// mailbox and enters the tested instruction through a format $0 RTE, the  //
// way the native cputest runner does.  Every vector leads to a capture    //
// stub that saves the registers with absolute addressing (the corpus      //
// stack holds the frame under test) and reports through a bus write; the  //
// bench then tells it to resume through RTE (a trace stacked on a primary //
// exception) or to stop, and compares registers, SR, exception frame and  //
// memory writes with the oracle -- the same checks as the previous core's //
// tb_dat_replay.v.                                                        //
//                                                                          //
// Memory: the corpus' low memory ($0-$7FFF) and test memory (TBASE, from  //
// the job header), the synthetic vector table at CAPV (VBR), the monitor  //
// and its mailbox.  Other addresses read zero and ignore writes.          //
//                                                                          //
// Plusargs: +job= +lmem= +tmem= +mon= (the monitor binary), +limit=N,      //
// +waits, +trace_round=N, +report=N (mismatch lines printed, default 40), //
// +watch=<hex address> (report the bench's writes to that byte).          //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_cputest;

localparam logic [31:0] TMEM_MAX = 32'h0020_0000;
localparam logic [31:0] CAPV  = 32'h4210_0000;   // VBR: the vector table
localparam logic [31:0] MONB  = 32'h4211_0000;   // monitor image base
localparam int          MONN  = 32'h0003_0000;   // monitor, mailbox, stack
localparam logic [31:0] CAPT  = 32'h4211_1000;   // capture routine
localparam logic [31:0] IDLE  = 32'h4211_1800;   // monitor reset entry
localparam logic [31:0] MBOX  = 32'h4212_0000;
localparam logic [31:0] MSTK  = 32'h4213_0000;
localparam logic [31:0] RND2  = 32'h524E_4432;

localparam logic [31:0] F_FPU         = 32'h0000_0001;
localparam logic [31:0] F_IGNORE_EXC  = 32'h0000_0002;
localparam logic [31:0] F_CHECK_FPIAR = 32'h0000_0020;

//--------------------------------------------------------------------------
// clock, reset, core, bus slave
//--------------------------------------------------------------------------
logic clk = 1'b0;
always #5 clk = ~clk;
logic rsti_n;
logic bclk_en = 1'b1;

logic [31:0] a_o, d_i, d_o;
logic        a_oe, d_oe, rw_n, ciout_n, lock_n, locke_n, ts_n, tip_n;
logic  [1:0] siz, tt, tln, upa;
logic  [2:0] tm;
logic        ta_n, tea_n, tci_n, tbi_n, avec_n;
logic        br_n, bb_n_o, bb_oe, rsto_n;
logic  [2:0] ipl;
logic [31:0] dbg_pc;
logic        dbg_retire, dbg_halted;
logic        mi_n, cpu_ta_n, cpu_ta_oe;

// no other master on this bus: the snoop inputs stay idle, and memory
// still waits for MI as the 68040 bus requires
ap68040 dut (
	.clk(clk), .bclk_en(bclk_en), .rsti_n(rsti_n),
	.a_o(a_o), .a_oe(a_oe),
	.a_i(a_o), .ts_n_i(1'b1), .rw_n_i(1'b1), .siz_i(2'd0), .tt_i(2'd0), .sc(2'd0),
	.mi_n(mi_n), .ta_n_o(cpu_ta_n), .ta_oe(cpu_ta_oe),
	.d_i(d_i), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .tln(tln), .upa(upa),
	.ciout_n(ciout_n), .lock_n(lock_n), .locke_n(locke_n),
	.ts_n(ts_n), .tip_n(tip_n), .ta_n(ta_n), .tea_n(tea_n),
	.tci_n(tci_n), .tbi_n(tbi_n), .ipl_n(~ipl), .avec_n(avec_n),
	.br_n(br_n), .bg_n(1'b0), .bb_n_i(1'b1), .bb_n_o(bb_n_o), .bb_oe(bb_oe),
	.rsto_n(rsto_n),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .dbg_halted(dbg_halted)
);

logic        xfer_v, xfer_rd;
logic [31:0] xfer_addr;
logic  [1:0] xfer_siz, xfer_tt, xfer_beat;
logic  [2:0] xfer_tm;
logic        ev, ev_rd, ev_err;
logic [31:0] ev_addr, ev_data;
logic  [3:0] ev_be;
logic  [1:0] ev_siz, ev_tt;
logic  [2:0] ev_tm;
logic  [1:0] wait_mode;
logic [31:0] ext_rdata;

m68040_bus_slave #(.AW(24), .SEED(7), .EXT(1)) mem (
	.clk(clk), .nreset(rsti_n), .bclk_en(bclk_en),
	.a(a_o), .d_cpu(d_o), .rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .ts_n(ts_n),
	.d_mem(d_i), .ta_n(ta_n), .tea_n(tea_n), .tbi_n(tbi_n), .tci_n(tci_n), .avec_n(avec_n),
	.wait_mode(wait_mode), .tbi_mode(2'd0), .retry_pct(0), .tea_req(1'b0), .tci_req(1'b0),
	.hold(!mi_n), .oth_ta_n(1'b1), .iack_vector(8'd0), .ext_rdata(ext_rdata), .ext_inmem(1'b1),
	.xfer_v(xfer_v), .xfer_addr(xfer_addr), .xfer_rd(xfer_rd), .xfer_siz(xfer_siz),
	.xfer_tt(xfer_tt), .xfer_tm(xfer_tm), .xfer_beat(xfer_beat),
	.ev(ev), .ev_rd(ev_rd), .ev_addr(ev_addr), .ev_data(ev_data), .ev_be(ev_be),
	.ev_siz(ev_siz), .ev_tt(ev_tt), .ev_tm(ev_tm), .ev_err(ev_err)
);

//--------------------------------------------------------------------------
// memory
//--------------------------------------------------------------------------
logic [31:0] TBASE, TSIZE, odd_vector;
logic  [7:0] lmem [0:32767];
logic  [7:0] tmem [0:TMEM_MAX-1];
logic  [7:0] mon  [0:MONN-1];
logic        boot;            // reset: SSP/PC come from the monitor

function automatic logic [7:0] rd8(input logic [31:0] a);
	logic [31:0] v;
	logic  [7:0] vec;
	if (boot && a < 32'd8) begin
		v = (a < 32'd4) ? MSTK : IDLE;
		return v[31 - 8 * a[1:0] -: 8];
	end
	if (a[31:15] == 17'd0) return lmem[a[14:0]];
	if (a >= TBASE && a < TBASE + TSIZE) return tmem[a - TBASE];
	if (a >= CAPV && a < CAPV + 32'h400) begin
		vec = 8'((a - CAPV) >> 2);
		v = (odd_vector != 32'd0 && vec >= 8'd4) ? odd_vector
		                                        : MONB + {20'd0, vec, 4'd0};
		return v[31 - 8 * a[1:0] -: 8];
	end
	if (a >= MONB && a < MONB + MONN) return mon[a - MONB];
	return 8'h00;
endfunction

int          jr;                // the record (round) being run
logic [31:0] watch_addr;        // +watch=: report every write to this byte
task automatic wr8(input logic [31:0] a, input logic [7:0] v);
	if (a == watch_addr)
		$display("%0t WATCH j%0d %08x <= %02x (was %02x)", $time, jr, a, v, rd8(a));
	if (a[31:15] == 17'd0) lmem[a[14:0]] = v;
	else if (a >= TBASE && a < TBASE + TSIZE) tmem[a - TBASE] = v;
	else if (a >= MONB && a < MONB + MONN) mon[a - MONB] = v;
endtask

function automatic logic [31:0] rdv(input logic [31:0] a, input logic [7:0] sz);
	if (sz == 8'd0) return {24'd0, rd8(a)};
	if (sz == 8'd1) return {16'd0, rd8(a), rd8(a + 1)};
	return {rd8(a), rd8(a + 1), rd8(a + 2), rd8(a + 3)};
endfunction

task automatic wrv(input logic [31:0] a, input logic [7:0] sz, input logic [31:0] v);
	int nb;
	nb = (sz == 8'd0) ? 1 : (sz == 8'd1) ? 2 : 4;
	for (int i = 0; i < nb; i++) wr8(a + i, 8'(v >> (8 * (nb - 1 - i))));
endtask

assign ext_rdata = {rd8({xfer_addr[31:2], 2'b00}), rd8({xfer_addr[31:2], 2'b01}),
                    rd8({xfer_addr[31:2], 2'b10}), rd8({xfer_addr[31:2], 2'b11})};

// CPU writes, and counts of the events the round follows
int          evt_cnt;         // the capture stub reported (EVT written)
int          frm_cnt;         // the entry RTE read its frame's format word
int          iack_cnt;        // interrupt acknowledge cycles
int          rdy_cnt;         // the monitor reached its idle loop (mask 7)
int          rdy0;            // rdy_cnt already used by a round
logic [31:0] frm_addr;        // the entry RTE frame of this round
initial begin evt_cnt = 0; frm_cnt = 0; iack_cnt = 0; rdy_cnt = 0; end
always @(posedge clk) begin
	if (ev && !ev_err && !ev_rd && ev_tt != 2'd3) begin
		for (int i = 0; i < 4; i++)
			if (ev_be[3 - i]) wr8({ev_addr[31:2], 2'(i)}, ev_data[31 - 8 * i -: 8]);
		if ({ev_addr[31:2], 2'b00} == MBOX + 32'h100 && ev_be[3]) evt_cnt = evt_cnt + 1;
		if ({ev_addr[31:2], 2'b00} == MBOX + 32'h204 && ev_be[3]) rdy_cnt = rdy_cnt + 1;
	end
	// the read that covers the frame's last byte: the format word
	if (ev && !ev_err && ev_rd && ev_tt != 2'd3 &&
	    {ev_addr[31:2], 2'b00} == ((frm_addr + 32'd7) & ~32'd3) &&
	    ev_be[2'd3 - 2'(frm_addr + 32'd7)])
		frm_cnt = frm_cnt + 1;
	if (ev && ev_tt == 2'd3) iack_cnt = iack_cnt + 1;
end

// +trace_round: the core's retirement and exception entries of that round
always @(posedge clk) begin
	if (jr == trace_round && dut.be.adv_wb)
		$display("%0t WB pc=%08x op=%0d last=%b exc=%0d bound=%b tr=%b sr=%04x take_trace=%b",
		         $time, dut.be.wb_u.pc, dut.be.wb_u.op, dut.be.wb_u.last, dut.be.wb_exc,
		         dut.be.wb_bound, dut.be.tr_now, dut.be.sr_r, dut.be.take_trace);
	if (jr == trace_round && ev)
		$display("%0t BUS %s %08x data=%08x be=%b tt=%0d tm=%0d err=%b", $time, ev_rd ? "RD" : "WR",
		         ev_addr, ev_data, ev_be, ev_tt, ev_tm, ev_err);
	if (jr == trace_round && dut.be.x_go)
		$display("%0t EXC vec=%0d pc=%08x", $time, dut.be.x_vec, dut.be.x_pc);
end

//--------------------------------------------------------------------------
// APR2 input
//--------------------------------------------------------------------------
int jf, jn;
int errors, ran, mism, skipped, report_lim, trace_round;
logic [31:0] flags, test_idx, round_idx;

function automatic logic [7:0] jread8();
	int r;
	r = $fgetc(jf);
	if (r < 0) begin $display("FAIL: unexpected end of the job file"); $finish; end
	return r[7:0];
endfunction
function automatic logic [15:0] jread16();
	logic [7:0] h;
	h = jread8();
	return {h, jread8()};
endfunction
function automatic logic [31:0] jread32();
	logic [15:0] h;
	h = jread16();
	return {h, jread16()};
endfunction

logic [31:0] i_regs [16];
logic [31:0] i_sr, i_pc, i_ssp, i_msp, i_fpcr, i_fpsr, i_fpiar;
logic [31:0] i_fe [8];
logic [63:0] i_fm [8];
logic  [7:0] i_level;
logic [31:0] e_regs [16];
logic [31:0] e_sr, e_srmask, e_fpcr, e_fpsr, e_fpiar, e_pc;
logic [31:0] e_fe [8];
logic [63:0] e_fm [8];
logic  [7:0] e_exc, e_trace, e_group2;
logic [15:0] e_trace_sr, e_trace_srmask;
logic [31:0] e_trace_pc;
logic  [7:0] frame_b [256];
logic  [7:0] frame_m [256];
int          frame_len;
logic [31:0] em_a [256];
logic  [7:0] em_sz [256];
logic [31:0] em_v [256], em_old [256];
int          em_cnt;
logic [31:0] pp_a [512];
logic [15:0] pp_n [512];
int          pp_off [512];
logic  [7:0] pp_b [16384];
int          post_cnt, clean_cnt, pp_bytes;

task automatic read_apply_patches();
	int cnt, len;
	logic [31:0] a;
	cnt = jread16();
	for (int i = 0; i < cnt; i++) begin
		a = jread32(); len = jread16();
		for (int j = 0; j < len; j++) wr8(a + j, jread8());
	end
endtask

// post (deferred) and cleanup patches share one store: post first
task automatic read_deferred(output int cnt);
	cnt = jread16();
	for (int i = 0; i < cnt; i++) begin
		int k;
		k = post_cnt + clean_cnt + i;
		if (k >= 512) begin $display("FAIL: patch table overflow"); $finish; end
		pp_a[k] = jread32(); pp_n[k] = jread16(); pp_off[k] = pp_bytes;
		for (int j = 0; j < pp_n[k]; j++) begin
			if (pp_bytes >= 16384) begin $display("FAIL: patch store overflow"); $finish; end
			pp_b[pp_bytes] = jread8(); pp_bytes++;
		end
	end
endtask

task automatic apply_deferred();
	for (int k = 0; k < post_cnt + clean_cnt; k++)
		for (int j = 0; j < pp_n[k]; j++) wr8(pp_a[k] + j, pp_b[pp_off[k] + j]);
endtask

task automatic mismatch(input string what, input logic [31:0] exp, input logic [31:0] got);
	mism++;
	if (mism <= report_lim)
		$display("MISMATCH j%0d t%0d r%0d %s: expected %08x got %08x (op=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x sr=%04x)",
		         jr, test_idx, round_idx, what, exp, got,
		         rd8(i_pc), rd8(i_pc + 1), rd8(i_pc + 2), rd8(i_pc + 3),
		         rd8(i_pc + 4), rd8(i_pc + 5), rd8(i_pc + 6), rd8(i_pc + 7),
		         rd8(i_pc + 8), rd8(i_pc + 9), rd8(i_pc + 10), rd8(i_pc + 11),
		         rd8(i_pc + 12), rd8(i_pc + 13), rd8(i_pc + 14), rd8(i_pc + 15),
		         i_sr[15:0]);
endtask

//--------------------------------------------------------------------------
// one round
//--------------------------------------------------------------------------
logic [31:0] cap_regs [16];
logic [31:0] cap_sp, cap_msp;
logic [15:0] cap_fe [8];
logic [63:0] cap_fm [8];
logic [31:0] cap_fpcr, cap_fpsr, cap_fpiar;
logic  [7:0] cap_vec;
logic  [7:0] frm_save [8];

task automatic boot_core();
	rsti_n = 1'b0;
	boot   = 1'b1;
	repeat (8) @(posedge clk);
	rsti_n = 1'b1;
	// the first instruction fetch from the monitor ends the reset overlay
	while (!(xfer_v && xfer_addr >= MONB && xfer_addr < MONB + 32'h2000)) @(posedge clk);
	boot = 1'b0;
	rdy0 = rdy_cnt;
endtask

task automatic read_capture();
	for (int i = 0; i < 15; i++) cap_regs[i] = rdv(MBOX + 32'h104 + 4 * i, 8'd2);
	cap_regs[15] = rdv(MBOX + 32'h140, 8'd2);
	cap_sp  = rdv(MBOX + 32'h144, 8'd2);
	cap_msp = rdv(MBOX + 32'h148, 8'd2);
	cap_vec = rdv(MBOX + 32'h102, 8'd1);
	// FMOVEM.X layout: sign/exponent, a zero word, the 64-bit mantissa
	for (int i = 0; i < 8; i++) begin
		cap_fe[i] = 16'(rdv(MBOX + 32'h170 + 12 * i, 8'd1));
		cap_fm[i] = {rdv(MBOX + 32'h174 + 12 * i, 8'd2), rdv(MBOX + 32'h178 + 12 * i, 8'd2)};
	end
	cap_fpcr  = rdv(MBOX + 32'h1D0, 8'd2);
	cap_fpsr  = rdv(MBOX + 32'h1D4, 8'd2);
	cap_fpiar = rdv(MBOX + 32'h1D8, 8'd2);
endtask

task automatic command(input logic [15:0] c);
	wrv(MBOX + 32'h200, 8'd1, {16'd0, c});
endtask

task automatic check_final();
	logic [15:0] sr;
	logic [31:0] fsp;
	if (!(flags & F_IGNORE_EXC) && cap_vec !== e_exc) mismatch("exception", e_exc, cap_vec);
	for (int i = 0; i < 16; i++)
		if (cap_regs[i] !== e_regs[i])
			mismatch($sformatf("%s%0d", i < 8 ? "D" : "A", i % 8), e_regs[i], cap_regs[i]);
	// the SR the exception saw: the stacked SR, the format $0 frame's on
	// the master stack under a throwaway frame
	fsp = (rdv(cap_sp + 6, 8'd1) >> 12 == 1) ? cap_msp : cap_sp;
	sr  = rdv(fsp, 8'd1);
	if (((sr ^ e_sr[15:0]) & e_srmask[15:0]) != 16'd0) mismatch("SR", e_sr, {16'd0, sr});
	if (frame_len != 0) begin
		for (int i = 0; i < frame_len; i++)
			if (((rd8(cap_sp + i) ^ frame_b[i]) & frame_m[i]) != 8'd0) begin
				if (mism < report_lim)
					$display("  frame byte %0d at %08x mask=%02x", i, cap_sp + i, frame_m[i]);
				mismatch("exception frame", frame_b[i], rd8(cap_sp + i));
			end
	end
	else if (!(flags & F_IGNORE_EXC) && e_exc == 8'd4) begin
		if (rdv(cap_sp + 2, 8'd2) !== e_pc) mismatch("end PC", e_pc, rdv(cap_sp + 2, 8'd2));
	end
	if (flags & F_FPU) begin
		for (int i = 0; i < 8; i++) begin
			if (cap_fe[i] !== e_fe[i][15:0])
				mismatch($sformatf("FP%0d sign/exp", i), e_fe[i], {16'd0, cap_fe[i]});
			if (cap_fm[i][63:32] !== e_fm[i][63:32])
				mismatch($sformatf("FP%0d mantissa hi", i), e_fm[i][63:32], cap_fm[i][63:32]);
			if (cap_fm[i][31:0] !== e_fm[i][31:0])
				mismatch($sformatf("FP%0d mantissa lo", i), e_fm[i][31:0], cap_fm[i][31:0]);
		end
		if (cap_fpcr !== e_fpcr) mismatch("FPCR", e_fpcr, cap_fpcr);
		if (cap_fpsr !== e_fpsr) mismatch("FPSR", e_fpsr, cap_fpsr);
		// the native runner checks FPIAR when the result stream names it,
		// or when execution changed it from the input value
		if ((flags & F_CHECK_FPIAR) || cap_fpiar !== i_fpiar)
			if (cap_fpiar !== e_fpiar) mismatch("FPIAR", e_fpiar, cap_fpiar);
	end
	for (int i = 0; i < em_cnt; i++) begin
		if (rdv(em_a[i], em_sz[i]) !== em_v[i])
			mismatch("memory write", em_v[i], rdv(em_a[i], em_sz[i]));
		wrv(em_a[i], em_sz[i], em_old[i]);
	end
endtask

localparam int EXEC_TIMEOUT = 20000;

task automatic run_round();
	int t, evt0, frm0, iack0;
	logic saw_trace, done, trace_bits;
	logic [31:0] tpc;
	// supervisor rounds: the corpus' stack image at the ISP the entry RTE
	// leaves (as the native runner copies it)
	if (i_sr[13])
		for (int i = 0; i < 32; i++) wr8(i_ssp + i, rd8(i_regs[15] + i));
	// the entry RTE frame below that ISP; restored once RTE has read it
	frm_addr = i_ssp - 32'd8;
	for (int i = 0; i < 8; i++) frm_save[i] = rd8(frm_addr + i);
	wrv(frm_addr, 8'd1, i_sr);
	wrv(frm_addr + 2, 8'd2, i_pc);
	wrv(frm_addr + 6, 8'd1, 32'd0);
	for (int i = 0; i < 15; i++) wrv(MBOX + 32'h004 + 4 * i, 8'd2, i_regs[i]);
	wrv(MBOX + 32'h040, 8'd2, i_regs[15]);
	wrv(MBOX + 32'h044, 8'd2, frm_addr);
	wrv(MBOX + 32'h048, 8'd2, i_msp);
	wrv(MBOX + 32'h04C, 8'd2, CAPV);
	for (int i = 0; i < 8; i++) begin
		wrv(MBOX + 32'h060 + 12 * i, 8'd2, {i_fe[i][15:0], 16'd0});
		wrv(MBOX + 32'h064 + 12 * i, 8'd2, i_fm[i][63:32]);
		wrv(MBOX + 32'h068 + 12 * i, 8'd2, i_fm[i][31:0]);
	end
	wrv(MBOX + 32'h0C0, 8'd2, i_fpcr);
	wrv(MBOX + 32'h0C4, 8'd2, i_fpsr);
	wrv(MBOX + 32'h0C8, 8'd2, i_fpiar);
	if (jr == trace_round) begin
		$display("ROUND j%0d t%0d r%0d flags=%08x sr=%04x pc=%08x ssp=%08x msp=%08x",
		         jr, test_idx, round_idx, flags, i_sr[15:0], i_pc, i_ssp, i_msp);
		for (int i = 0; i < 16; i++)
			$display("  %s%0d=%08x", i < 8 ? "D" : "A", i % 8, i_regs[i]);
		for (int i = 0; i < 8; i++)
			$display("  FP%0d=%04x_%016x", i, i_fe[i][15:0], i_fm[i]);
		$display("  FPCR=%08x FPSR=%08x FPIAR=%08x", i_fpcr, i_fpsr, i_fpiar);
	end
	// the native runner raises the interrupt before its entry RTE, with
	// the interrupt mask at 7: wait for the monitor to be idle there (the
	// previous round's capture loop runs at that round's mask)
	t = 0;
	while (rdy_cnt == rdy0 && t < EXEC_TIMEOUT) begin @(posedge clk); t++; end
	if (rdy_cnt == rdy0) begin
		$display("FAIL j%0d t%0d r%0d: the monitor did not come back to its idle loop",
		         jr, test_idx, round_idx);
		errors++;
		boot_core();
		return;
	end
	rdy0 = rdy_cnt;
	ipl = i_level[2:0];
	evt0  = evt_cnt;
	frm0  = frm_cnt;
	iack0 = iack_cnt;
	wrv(MBOX, 8'd1, 32'd1);

	trace_bits = (i_sr[15:14] != 2'd0) || (e_sr[15:14] != 2'd0);
	saw_trace = 1'b0;
	done = 1'b0;
	t = 0;
	while (!done && t < EXEC_TIMEOUT) begin
		@(posedge clk);
		t++;
		if (frm_cnt != frm0) begin
			for (int i = 0; i < 8; i++) wr8(frm_addr + i, frm_save[i]);
			frm0 = frm_cnt;
			frm_addr = 32'hFFFF_FFF0;
		end
		// the request is released once the core has acknowledged it
		if (iack_cnt != iack0) ipl = 3'd0;
		if (evt_cnt != evt0) begin
			evt0 = evt_cnt;
			read_capture();
			if (jr == trace_round)
				$display("EVENT vec=%0d sp=%08x sr=%04x pc=%08x", cap_vec, cap_sp,
				         rdv(cap_sp, 8'd1), rdv(cap_sp + 2, 8'd2));
			if (cap_vec == 8'd9) begin
				tpc = rdv(cap_sp + 2, 8'd2);
				if (saw_trace && e_exc != 8'd9) begin
					// the trace bits survive the trace's RTE: the filler
					// instruction traces again on the way to the terminal
					// ILLEGAL.  Only the first trace is the oracle's (as in
					// the previous core's bench, whose vector 9 was RTE).
					command(16'd1);
				end
				else if (e_trace != 8'd0 || e_exc == 8'd9) begin
					if (e_trace == 8'd2) begin
						if (((rdv(cap_sp, 8'd1) ^ e_trace_sr) & e_trace_srmask) != 0)
							mismatch("trace SR", e_trace_sr, rdv(cap_sp, 8'd1));
						if (tpc !== e_trace_pc) mismatch("trace PC", e_trace_pc, tpc);
					end
					saw_trace = 1'b1;
					if (e_exc == 8'd9) done = 1'b1;
					else command(16'd1);
				end
				else if (trace_bits && e_exc != 8'd0 &&
				         tpc == MONB + {20'd0, e_exc, 4'd0}) begin
					// a trace at the recorded exception's handler entry,
					// beyond what the corpus records: run on to that handler
					command(16'd1);
				end
				else begin
					mismatch("unexpected trace", 32'd0, rdv(cap_sp, 8'd1));
					done = 1'b1;
				end
			end
			else done = 1'b1;
			t = 0;
		end
	end
	if (!done) begin
		$display("FAIL j%0d t%0d r%0d: execution timeout, pc=%08x", jr, test_idx, round_idx, dbg_pc);
		errors++;
		if (frm_addr != 32'hFFFF_FFF0)
			for (int i = 0; i < 8; i++) wr8(frm_addr + i, frm_save[i]);
		boot_core();
		ipl = 3'd0;
		return;
	end
	if (e_trace != 8'd0 && !saw_trace) mismatch("missing trace", 9, 0);
	check_final();
	ran++;
	ipl = 3'd0;
	command(16'd2);
endtask

//--------------------------------------------------------------------------
// main
//--------------------------------------------------------------------------
string job_file, lmem_file, tmem_file, mon_file;
int limit;

initial begin
	logic [31:0] magic, ver, tb, ts, ov;
	int fd, got;
	rsti_n = 1'b0; boot = 1'b1; ipl = 3'd0;
	frm_addr = 32'hFFFF_FFF0;
	errors = 0; ran = 0; mism = 0; skipped = 0;
	wait_mode = $test$plusargs("waits") ? 2'd1 : 2'd0;
	if (!$value$plusargs("report=%d", report_lim)) report_lim = 40;
	if (!$value$plusargs("trace_round=%d", trace_round)) trace_round = -1;
	if (!$value$plusargs("limit=%d", limit)) limit = 32'h7FFF_FFFF;
	if (!$value$plusargs("watch=%h", watch_addr)) watch_addr = 32'hFFFF_FFFF;
	if (!$value$plusargs("job=%s", job_file) || !$value$plusargs("lmem=%s", lmem_file) ||
	    !$value$plusargs("tmem=%s", tmem_file) || !$value$plusargs("mon=%s", mon_file)) begin
		$display("FAIL: require +job= +lmem= +tmem= +mon=");
		$finish;
	end
	for (int i = 0; i < 32768; i++) lmem[i] = 8'd0;
	for (int i = 0; i < MONN; i++) mon[i] = 8'd0;
	fd = $fopen(lmem_file, "rb"); got = $fread(lmem, fd); $fclose(fd);
	fd = $fopen(mon_file, "rb");  got = $fread(mon, fd);  $fclose(fd);
	jf = $fopen(job_file, "rb");
	if (jf == 0) begin $display("FAIL: cannot open the job"); $finish; end
	magic = jread32(); ver = jread32(); jn = jread32();
	tb = jread32(); ts = jread32(); ov = jread32();
	if (magic != "APR2" || ver != 32'd3 || ts > TMEM_MAX) begin
		$display("FAIL: unsupported APR2 job");
		$finish;
	end
	TBASE = tb; TSIZE = ts; odd_vector = ov;
	fd = $fopen(tmem_file, "rb"); got = $fread(tmem, fd); $fclose(fd);
	if (jn > limit) jn = limit;
	$display("tb_cputest: %0d APR2 records, test memory %08x/%0dK", jn, TBASE, TSIZE / 1024);

	boot_core();
	for (jr = 0; jr < jn; jr++) begin
		if (jread32() != RND2) begin $display("FAIL: record desync at %0d", jr); $finish; end
		test_idx = jread32(); round_idx = jread32(); flags = jread32();
		for (int k = 0; k < 16; k++) i_regs[k] = jread32();
		i_sr = jread32(); i_pc = jread32(); i_ssp = jread32(); i_msp = jread32();
		for (int k = 0; k < 8; k++) begin i_fe[k] = jread32(); i_fm[k] = {jread32(), jread32()}; end
		i_fpcr = jread32(); i_fpsr = jread32(); i_fpiar = jread32();
		i_level = jread8();
		read_apply_patches();
		begin
			int n;
			n = jread16();
			for (int k = 0; k < n; k++) begin
				logic [31:0] ta, tv;
				logic  [7:0] kind;
				ta = jread32(); kind = jread8();
				tv = rdv(ta, 8'd2);
				if (kind == 8'd1) wrv(ta, 8'd2, {tv[15:0], tv[31:16]});
				else if (kind == 8'd2) wrv(ta, 8'd1, (tv[31:16] == 16'h2048) ? 32'h4AFC : 32'h2048);
			end
		end
		for (int k = 0; k < 16; k++) e_regs[k] = jread32();
		e_sr = jread32(); e_srmask = jread32();
		for (int k = 0; k < 8; k++) begin e_fe[k] = jread32(); e_fm[k] = {jread32(), jread32()}; end
		e_fpcr = jread32(); e_fpsr = jread32(); e_fpiar = jread32();
		e_exc = jread8(); e_pc = jread32();
		e_trace = jread8(); e_group2 = jread8();
		e_trace_sr = jread16(); e_trace_srmask = jread16(); e_trace_pc = jread32();
		frame_len = jread16();
		for (int k = 0; k < frame_len; k++) frame_b[k] = jread8();
		for (int k = 0; k < frame_len; k++) frame_m[k] = jread8();
		em_cnt = jread16();
		for (int k = 0; k < em_cnt; k++) begin
			em_a[k] = jread32(); em_sz[k] = jread8(); em_v[k] = jread32(); em_old[k] = jread32();
		end
		post_cnt = 0; clean_cnt = 0; pp_bytes = 0;
		read_deferred(post_cnt);
		read_deferred(clean_cnt);
		// ignored rounds carry no oracle
		if (flags & F_IGNORE_EXC) skipped++;
		else run_round();
		apply_deferred();
	end
	$display("cputest replay: %0d rounds, %0d skipped (no oracle), %0d mismatches, %0d harness errors",
	         ran, skipped, mism, errors);
	if (mism == 0 && errors == 0) $display("ALL TESTS PASSED");
	else $display("TEST FAILED with %0d errors", mism + errors);
	$finish;
end

endmodule
