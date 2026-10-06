//--------------------------------------------------------------------------//
// AP68040-60 test bench                                                     //
//                                                                          //
// tb_ap68040.sv - runs an assembled self-checking program on the core      //
// through the 68040 bus                                                    //
//                                                                          //
// Program image: +prog=<hex> (one 16-bit word per line, from bin2hex.py),  //
// loaded at address 0.  The programs report through memory-mapped         //
// registers, the same protocol the previous core's bench used:             //
//   $F100 word  failing test number                                        //
//   $F102 word  $600D = all passed, anything else = failed                 //
//   $F108 word  cycle stamp (prints cycles since the previous stamp)       //
//   $F110 word  interrupt level for IPL (0 releases)                       //
//   $F120       writes must carry FC=1 (MOVES/DFC check)                   //
//   $F130 word  DMA-style poke: memory $3500 = data, $3502 = 0             //
//   $F142       arm a one-shot bus error on the next access to $F140       //
//               (armed at reset: the first access to $F140 is rejected)    //
//   $F146       arm a one-shot bus error on the next table search access  //
//   $F148 word  raise IPL 2 after the written number of clocks            //
//   $F14C word  IPL = bits 2:0, withdrawn after bits 15:8 clocks          //
//   $F150 word  IPL = bits 2:0, falls to bits 6:4 after bits 15:8 clocks  //
//   $F154 word  arm a one-shot bus error on an instruction fetch at the   //
//               written address (0 disarms)                               //
//   $F160 word  (read) bench capability word, +cap=<n> (default 7)        //
//   $F200 + 32k (k = 0..3): alternate-master transfer k: +0 address,     //
//               +4 word: bit 0 read, 2:1 SIZ, 4:3 SC1/SC0; +8..+$17 data //
//               (write data in, read data back: a line's four long words,//
//               or one long word as it is on D31-D0 at +8)                //
//   $F280 word  run transfers 0..n-1 as an alternate bus master: bits 2:0 //
//               n, bit 8 wait first for a 68040 line read of the line at //
//               $F284 (it takes the bus right after it)                   //
//   $F288 word  bus clocks the alternate master holds the bus before its  //
//               first transfer                                             //
//   $F28C word  (read) alternate master: 0 busy, 1 done, 2 bus error      //
//   $F294 long  copied to $F2A8 when written (a read of $F2A8, another   //
//               line, shows whether the write reached the bus first)      //
//   $F2B0 long  (read) counts its own bus reads into $F2B4              //
//   $F2C0 word  bit 0 asserts CDIS, bit 1 MDIS                            //
//   $F2C4 word  random snoop traffic from the alternate master: bit 15    //
//               on, bits 7:0 bus clocks between transfers (see t_snstress)//
//   $F164 word  (read) interrupts accepted on an IPEND claim alone: at   //
//               or below the boundary mask, after the request qualified   //
//               against an earlier, lower mask                            //
//                                                                          //
// Interrupt invariants checked on every run: no interrupt is accepted     //
// long after IPL went idle (phantom), none at or below the mask without   //
// an IPEND claim, and a claimed request is taken at the next instruction  //
// boundary.                                                               //
//                                                                          //
// Plusargs: +waits (random wait states), +tbi=<0|1|2>, +retry=<percent>,  //
// +bclk2 (bus at half the processor clock), +timeout=<cycles>, +trace.    //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap68040;

logic clk = 1'b0;
always #5 clk = ~clk;

logic rsti_n = 1'b0;
logic bclk_en;
logic bclk2;
logic bclk_ph = 1'b0;
always_ff @(posedge clk) bclk_ph <= !bclk_ph;
assign bclk_en = bclk2 ? bclk_ph : 1'b1;

// bus
logic [31:0] a_o, d_o, d_mem;
logic        a_oe, d_oe, rw_n, ciout_n, lock_n, locke_n, ts_n, tip_n;
logic  [1:0] siz, tt, tln, upa;
logic  [2:0] tm;
logic        ta_n, tea_n, tci_n, tbi_n, avec_n;
logic        br_n, bb_n_o, bb_oe, rsto_n;
logic  [2:0] ipl_lvl;
logic [31:0] dbg_pc;
logic        dbg_retire, dbg_halted;

// random snoop traffic ($F2C4, t_snstress): reads of the CPU's counters
// at $A000 (SC 01: they must never go backwards) and reads/writes of the
// shared words at $B000 (each value names its writer and address)
logic        rs_en;
logic  [7:0] rs_gap, rs_cnt;
logic  [2:0] rs_op;
logic  [5:0] rs_k;
logic [15:0] rs_seq;
logic        rs_busy;
int          rs_last [64];
int          rs_ops;
function automatic logic rs_valid(input logic [31:0] v, input logic [5:0] k);
	logic [15:0] a;
	a = {8'd0, k, 2'b00};
	rs_valid = (v == 32'd0) || (v[15:0] == (a ^ 16'h5A5A)) || (v[15:0] == (a ^ 16'hA5A5));
endfunction

// pins the program drives through $F2C0, and the status pins
logic  [1:0] pins_r;
logic        ipend_n;
logic  [3:0] pst;

// the bus: the 68040 or the alternate master drives it
logic        mi_n, cpu_ta_n, cpu_ta_oe, slv_ta_n;
logic        am_drive, am_rw_n, am_ts_n, am_bb_n, am_d_oe, am_bg_n;
logic [31:0] am_a, am_d;
logic  [1:0] am_siz, am_tt, am_sc;
logic  [2:0] am_tm;
wire  [31:0] bus_a    = am_drive ? am_a : a_o;
wire         bus_ts_n = am_drive ? am_ts_n : ts_n;
wire         bus_rw_n = am_drive ? am_rw_n : rw_n;
wire   [1:0] bus_siz  = am_drive ? am_siz : siz;
wire   [1:0] bus_tt   = am_drive ? am_tt : tt;
wire   [2:0] bus_tm   = am_drive ? am_tm : tm;
wire  [31:0] bus_dw   = am_d_oe ? am_d : d_o;            // write data
wire  [31:0] bus_dr   = d_oe ? d_o : d_mem;              // read data (snoop-supplied)
assign       ta_n     = slv_ta_n & (cpu_ta_oe ? cpu_ta_n : 1'b1);

ap68040 dut (
	.clk(clk), .bclk_en(bclk_en), .rsti_n(rsti_n),
	.a_o(a_o), .a_oe(a_oe),
	.a_i(bus_a), .ts_n_i(am_drive ? am_ts_n : 1'b1), .rw_n_i(bus_rw_n), .siz_i(bus_siz),
	.tt_i(bus_tt), .sc(am_drive ? am_sc : 2'd0),
	.mi_n(mi_n), .ta_n_o(cpu_ta_n), .ta_oe(cpu_ta_oe),
	.cdis_n(!pins_r[0]), .mdis_n(!pins_r[1]), .ipend_n(ipend_n), .pst(pst),
	.d_i(am_d_oe ? am_d : d_mem), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .tln(tln), .upa(upa),
	.ciout_n(ciout_n), .lock_n(lock_n), .locke_n(locke_n),
	.ts_n(ts_n), .tip_n(tip_n), .ta_n(ta_n), .tea_n(tea_n),
	.tci_n(tci_n), .tbi_n(tbi_n), .ipl_n(~ipl_lvl), .avec_n(avec_n),
	.br_n(br_n), .bg_n(am_bg_n), .bb_n_i((bb_oe ? bb_n_o : 1'b1) & am_bb_n),
	.bb_n_o(bb_n_o), .bb_oe(bb_oe), .rsto_n(rsto_n),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .dbg_halted(dbg_halted)
);

// slave
logic  [1:0] wait_mode, tbi_mode;
int          retry_pct;
logic        tea_req;
logic        xfer_v, xfer_rd;
logic [31:0] xfer_addr;
logic  [1:0] xfer_siz, xfer_tt, xfer_beat;
logic  [2:0] xfer_tm;
logic        ev, ev_rd, ev_err;
logic [31:0] ev_addr, ev_data;
logic  [3:0] ev_be;
logic  [1:0] ev_siz, ev_tt;
logic  [2:0] ev_tm;

m68040_bus_slave #(.AW(20)) mem (
	.clk(clk), .nreset(rsti_n), .bclk_en(bclk_en),
	.a(bus_a), .d_cpu(bus_dw), .rw_n(bus_rw_n), .siz(bus_siz), .tt(bus_tt), .tm(bus_tm), .ts_n(bus_ts_n),
	.d_mem(d_mem), .ta_n(slv_ta_n), .tea_n(tea_n), .tbi_n(tbi_n), .tci_n(tci_n),
	.avec_n(avec_n),
	.wait_mode(wait_mode), .tbi_mode(tbi_mode), .retry_pct(retry_pct),
	.tea_req(tea_req), .tci_req(1'b0), .hold(fetch_hold || !mi_n), .oth_ta_n(cpu_ta_oe ? cpu_ta_n : 1'b1),
	.iack_vector(8'd0), .ext_rdata(32'd0), .ext_inmem(1'b0),
	.xfer_v(xfer_v), .xfer_addr(xfer_addr), .xfer_rd(xfer_rd), .xfer_siz(xfer_siz),
	.xfer_tt(xfer_tt), .xfer_tm(xfer_tm), .xfer_beat(xfer_beat),
	.ev(ev), .ev_rd(ev_rd), .ev_addr(ev_addr), .ev_data(ev_data), .ev_be(ev_be),
	.ev_siz(ev_siz), .ev_tt(ev_tt), .ev_tm(ev_tm), .ev_err(ev_err)
);

//--------------------------------------------------------------------------
// program load
//--------------------------------------------------------------------------
logic [15:0] img [0:65535];
string       prog;
int          nwords;

task automatic load_prog();
	for (int i = 0; i < 65536; i++) img[i] = 16'h0000;
	$readmemh(prog, img);
	for (int i = 0; i < (1 << 18); i++) mem.mem[i] = 32'd0;
	for (int i = 0; i < 32768; i++) mem.mem[i] = {img[2 * i], img[2 * i + 1]};
endtask

// word written on a beat (address of the word, data on its lanes)
function automatic logic [15:0] ev_word(input logic [31:0] a, input logic [31:0] d);
	ev_word = a[1] ? d[15:0] : d[31:16];
endfunction

//--------------------------------------------------------------------------
// MMIO and error injection
//--------------------------------------------------------------------------
int          result;          // 0 running, 1 pass, 2 fail
int          errors;
int          cycles;
int          stamp_prev;
int          insns, insn_prev;   // instructions completed (last uops retired)
always_ff @(posedge clk) begin
	if (!rsti_n) insns <= 0;
	else if (dut.be.adv_wb && dut.be.wb_u.last) insns <= insns + 1;
end

// +prof: where the cycles go between stamps.  A cycle that retires a uop
// is "retire"; otherwise the oldest stage that holds (or the first empty
// one, going back from WB) takes the blame.
logic prof;
int   pf [12];
localparam int PF_RET = 0, PF_WB = 1, PF_EX = 2, PF_DC2 = 3, PF_DC1 = 4, PF_AG = 5,
               PF_FE = 6, PF_UOPS = 7, PF_REDIR = 8, PF_DREDIR = 9, PF_FQE = 10, PF_HALT = 11;
initial for (int i = 0; i < 12; i++) pf[i] = 0;
always_ff @(posedge clk) if (rsti_n) begin
	if (dut.be.adv_wb) begin pf[PF_RET]++; pf[PF_UOPS]++; end
	else if (dut.be.wb_v) pf[PF_WB]++;
	else if (dut.be.ex_v) pf[PF_EX]++;
	else if (dut.be.dc2_v) pf[PF_DC2]++;
	else if (dut.be.dc1_v) pf[PF_DC1]++;
	else if (dut.be.ag_v) pf[PF_AG]++;
	else if (dut.fetch.cnt == 0) pf[PF_FQE]++;
	else pf[PF_FE]++;
	if (dut.redir_v) pf[PF_REDIR]++;
	if (dut.d_redir_v) pf[PF_DREDIR]++;
	if (dut.be.adv_ex) begin
		if (dut.be.ex_mispred) begin
			case (dut.be.ex_u.br)
				3'd1:    rc[0]++;          // conditional (Bcc/DBcc/FBcc)
				3'd2:    rc[1]++;          // BR_IMM
				3'd3:    rc[2]++;          // BR_EA (JMP/JSR <ea>)
				3'd4:    rc[3]++;          // BR_A (RTS, RTD, RTR...)
				3'd5:    rc[4]++;          // BR_B
				default: rc[5]++;          // RTE and others
			endcase
		end
		else if (dut.be.ex_u.ser) rc[6]++;
		else if (dut.be.ex_sr_we) rc[7]++;
		else if (dut.be.ex_u.last && (dut.be.smc_pend || dut.be.ex_smc)) rc[8]++;
	end
end
int rc [9];
initial for (int i = 0; i < 9; i++) rc[i] = 0;
// loads that missed the DC2 fast path: miss, older store to the line,
// a same-set RAM write after the lookup, other (split, CI, ...)
int lsl [5];
initial for (int i = 0; i < 5; i++) lsl[i] = 0;
always_ff @(posedge clk) if (rsti_n && dut.dmu.adv_dc1 && dut.dmu.m1.v && dut.dmu.m1.mem == 2'd1) begin
	if (dut.dmu.m1_fast && !dut.dmu.m1_stale && !(dut.dmu.dw && dut.dmu.dw_set == dut.dmu.m1.a[9:4]) &&
	    dut.dmu.e_st == 0) lsl[0]++;
	else if (!dut.dmu.x_dc1.hit) lsl[1]++;
	else if (dut.dmu.m1_stale || (dut.dmu.dw && dut.dmu.dw_set == dut.dmu.m1.a[9:4])) lsl[2]++;
	else if (dut.dmu.e_st != 0) lsl[3]++;
	else lsl[4]++;
end
// DMU engine: jobs started (DC2 slow path, WB store, maintenance, walk)
// and the cycles it is busy; DC2 cycles blocked by an older store (hz)
int ej [6];
int ejk [16];   // DC2 jobs by {mem, hit, split}
initial begin for (int i = 0; i < 6; i++) ej[i] = 0; for (int i = 0; i < 16; i++) ejk[i] = 0; end
always_ff @(posedge clk) if (rsti_n) begin
	if (dut.dmu.e_st == dut.dmu.E_S_START) begin
		ej[0]++;
		ejk[{dut.dmu.m2.r.mem, dut.dmu.m2.x.hit, dut.dmu.m2.split}]++;
	end
	if (dut.dmu.e_st == dut.dmu.E_W_START) ej[1]++;
	if (dut.dmu.e_st == dut.dmu.E_M_START) ej[2]++;
	if (dut.dmu.e_st == dut.dmu.E_TW_START) ej[3]++;
	if (dut.dmu.e_st != dut.dmu.E_IDLE) ej[4]++;
	if (dut.dmu.m2.r.v && dut.dmu.m2.fast && dut.dmu.hz) ej[5]++;
end
// snoops: lookups by where the line was (cache, push buffer, queued push,
// miss), dirty supplies, sinks, invalidations, snoop-forced refetches
int snc [8];
initial for (int i = 0; i < 8; i++) snc[i] = 0;
always_ff @(posedge clk) if (rsti_n) begin
	if (dut.dmu.sn_look) begin
		if (!dut.dmu.sn_hit) snc[3]++;
		else snc[dut.dmu.sn_src]++;
	end
	if (dut.snoop.st == dut.snoop.S_SRC && dut.snoop.bclk_en && !dut.snoop.ta_oe) snc[4]++;
	if (dut.snoop.dc_wr) snc[5]++;
	if (dut.snoop.dc_inv) snc[6]++;
	if (dut.sn_ihit) snc[7]++;
end
// BTB: followed, overruled at a branch's end, restarted (flag inside)
int btc [3];
initial for (int i = 0; i < 3; i++) btc[i] = 0;
always_ff @(posedge clk) if (rsti_n) begin
	if (dut.dec.push && dut.dec.ph == 0 && dut.dec.bt_end) begin
		if (dut.dec.nrec.pred && dut.dec.ntarget == dut.dec.bt_tgt) btc[0]++;
		else btc[1]++;
	end
	if (dut.dec.bt_restart && !dut.dec.stall) btc[2]++;
end
// EX-blamed cycles by EX op; WB-blamed by WB op
int exop [64], wbop [64];
initial for (int i = 0; i < 64; i++) begin exop[i] = 0; wbop[i] = 0; end
int hold [8];
int us [5];
initial for (int i = 0; i < 5; i++) us[i] = 0;
initial for (int i = 0; i < 8; i++) hold[i] = 0;
always_ff @(posedge clk) if (rsti_n && prof) begin
	if (dut.be.wb_v && dut.be.wb_hold) begin hold[0]++; wbop[dut.be.wb_u.op]++; end
	if (dut.be.ex_v && dut.be.ex_hold) begin hold[1]++; exop[dut.be.ex_u.op]++; end
	if (dut.be.dc2_hold) hold[2]++;
	if (dut.be.dc1_v && dut.dm_hold1) hold[3]++;
	if (dut.be.ag_v && dut.be.ag_hold) hold[4]++;
	if (!dut.be.ag_v || !dut.be.stall_ag) begin
		// AG could take a uop: does the front end have one?
		if (dut.uq_n == 2'd0) begin
			if (dut.rq_n != 2'd0) begin
				hold[5]++;                            // useq busy / expanding
				if (!dut.useq.uw_v) us[0]++;          // first-word ROM read
				else if (dut.useq.step && dut.useq.n_jump) us[1]++;   // jump word
				else if (!dut.useq.step) us[2]++;     // not stepping
				else if (dut.useq.n_ptr) us[3]++;     // pointer load
				else us[4]++;
			end
			else if (dut.fetch.cnt != 0) hold[6]++;   // decode has words, no record
			else hold[7]++;                           // fetch queue empty
		end
	end
end
// +pcprof: the cycles since the previous instruction completed, charged
// to each completing instruction's address (dumped at the end)
logic pcprof;
int   pc_cyc [int unsigned];
int   pc_n   [int unsigned];
int   pc_last;
initial pcprof = $test$plusargs("pcprof");
always_ff @(posedge clk) if (rsti_n && pcprof && dut.be.adv_wb && dut.be.wb_u.last) begin
	pc_cyc[dut.be.wb_u.pc] += cycles - pc_last;
	pc_n[dut.be.wb_u.pc]   += 1;
	pc_last <= cycles;
end
int   pc_mp  [int unsigned];
int   pc_uo  [int unsigned];
always_ff @(posedge clk) if (rsti_n && pcprof && dut.be.adv_wb)
	pc_uo[dut.be.wb_u.pc] += 1;
always_ff @(posedge clk) if (rsti_n && pcprof && dut.be.adv_ex && dut.be.ex_mispred)
	pc_mp[dut.be.ex_u.pc] += 1;
// AG interlock cycles by the youngest producer: stage (1 DC1, 2 DC2, 3 EX),
// a load or not, and its op
int   il_k [int unsigned];
always_ff @(posedge clk) if (rsti_n && pcprof && dut.be.ag_v && dut.be.ag_interlock) begin
	logic [4:0] r [4];
	logic       rv [4];
	int         k;
	r[0] = dut.be.ag_u.base;     rv[0] = dut.be.ag_u.base_v;
	r[1] = dut.be.ag_u.idx;      rv[1] = dut.be.ag_u.idx_v;
	r[2] = dut.be.ag_u.upd_reg;  rv[2] = dut.be.ag_u.upd_v;
	r[3] = dut.be.ag_u.upd2_reg; rv[3] = dut.be.ag_u.upd2_v;
	k = 0;
	for (int i = 0; i < 4; i++) if (rv[i]) begin
		if (dut.be.dc1_v && dut.be.dc1_u.d_v && dut.be.dc1_u.d_reg == r[i])
			k = 1000 + (dut.be.dc1_u.mem != 0) * 100 + dut.be.dc1_u.op;
		else if (k == 0 && dut.be.dc2_v && dut.be.dc2_u.d_v && dut.be.dc2_u.d_reg == r[i])
			k = 2000 + (dut.be.dc2_u.mem != 0) * 100 + dut.be.dc2_u.op;
		else if (k == 0 && dut.be.ex_v && dut.be.ex_u.d_v && dut.be.ex_u.d_reg == r[i])
			k = 3000 + (dut.be.ex_u.mem != 0) * 100 + dut.be.ex_u.op;
	end
	il_k[k] += 1;
end
task automatic pcprof_dump();
	foreach (il_k[a]) $display("ILK %0d %0d", a, il_k[a]);
	foreach (pc_cyc[a]) $display("PCPROF %08x %0d %0d", a, pc_n[a], pc_cyc[a]);
	foreach (pc_mp[a]) $display("PCMISP %08x %0d", a, pc_mp[a]);
	foreach (pc_uo[a]) $display("PCUOPS %08x %0d", a, pc_uo[a]);
endtask
task automatic prof_report();
	int tot;
	tot = pf[PF_RET] + pf[PF_WB] + pf[PF_EX] + pf[PF_DC2] + pf[PF_DC1] + pf[PF_AG] + pf[PF_FE] + pf[PF_FQE];
	if (tot == 0) tot = 1;
	$display("PROF cycles=%0d uops=%0d | retire %0d%% | waiting on: WB %0d%% EX %0d%% DC2 %0d%% DC1 %0d%% AG %0d%% decode/useq %0d%% fetch-empty %0d%% | redirects WB=%0d D1=%0d",
	         tot, pf[PF_UOPS], 100 * pf[PF_RET] / tot, 100 * pf[PF_WB] / tot, 100 * pf[PF_EX] / tot,
	         100 * pf[PF_DC2] / tot, 100 * pf[PF_DC1] / tot, 100 * pf[PF_AG] / tot,
	         100 * pf[PF_FE] / tot, 100 * pf[PF_FQE] / tot, pf[PF_REDIR], pf[PF_DREDIR]);
	$display("PROF BTB: followed %0d, overruled at the end %0d, restarted %0d", btc[0], btc[1], btc[2]);
	$display("PROF snoops: hit cache %0d, push buffer %0d, queued push %0d, miss %0d; supplied %0d, sunk %0d, invalidated %0d, I-side refetches %0d",
	         snc[0], snc[1], snc[2], snc[3], snc[4], snc[5], snc[6], snc[7]);
	for (int i = 0; i < 8; i++) snc[i] = 0;
	$display("PROF loads at DC1: fast %0d, miss %0d, set written since lookup %0d, engine busy %0d, other %0d",
	         lsl[0], lsl[1], lsl[2], lsl[3], lsl[4]);
	for (int i = 0; i < 5; i++) lsl[i] = 0;
	$display("PROF engine: DC2 jobs %0d, WB store jobs %0d, maintenance %0d, walks %0d, busy cycles %0d; fast loads held by an older store %0d cycles",
	         ej[0], ej[1], ej[2], ej[3], ej[4], ej[5]);
	for (int i = 0; i < 6; i++) ej[i] = 0;
	for (int i = 0; i < 16; i++) begin
		if (ejk[i] != 0) $display("PROF   DC2 jobs mem=%0d hit=%0d split=%0d: %0d", i >> 2, (i >> 1) & 1, i & 1, ejk[i]);
		ejk[i] = 0;
	end
	for (int i = 0; i < 3; i++) btc[i] = 0;
	$display("PROF WB redirects: mispredict cond=%0d imm=%0d ea=%0d A(rts)=%0d B=%0d other=%0d | serialize=%0d sr-write=%0d smc=%0d",
	         rc[0], rc[1], rc[2], rc[3], rc[4], rc[5], rc[6], rc[7], rc[8]);
	for (int i = 0; i < 12; i++) pf[i] = 0;
	for (int i = 0; i < 9; i++) rc[i] = 0;
	$display("PROF holds (cycles): WB %0d%% EX %0d%% DC2 %0d%% DC1 %0d%% AG-interlock %0d%% | AG starved: useq %0d%% decode %0d%% fetch %0d%%",
	         100 * hold[0] / tot, 100 * hold[1] / tot, 100 * hold[2] / tot, 100 * hold[3] / tot,
	         100 * hold[4] / tot, 100 * hold[5] / tot, 100 * hold[6] / tot, 100 * hold[7] / tot);
	$display("PROF   useq: first-word %0d%% jump-word %0d%% not-stepping %0d%% pointer %0d%% other %0d%%",
	         100 * us[0] / tot, 100 * us[1] / tot, 100 * us[2] / tot, 100 * us[3] / tot, 100 * us[4] / tot);
	for (int i = 0; i < 5; i++) us[i] = 0;
	for (int i = 0; i < 8; i++) hold[i] = 0;
	for (int i = 0; i < 64; i++) begin
		if (exop[i] * 100 > tot) $display("PROF   EX op %0d: %0d%%", i, 100 * exop[i] / tot);
		if (wbop[i] * 100 > tot) $display("PROF   WB op %0d: %0d%%", i, 100 * wbop[i] / tot);
		exop[i] = 0; wbop[i] = 0;
	end
endtask
logic        berr_armed, wberr_arm, fberr_armed;
logic [15:0] fberr_addr;
logic [15:0] ipl_delay;
logic  [7:0] ipl_pulse, ipl_step;
logic  [2:0] ipl_next;
logic  [1:0] irq_exc_armed;   // $F144
logic  [2:0] fetch_stall;     // hold the next instruction fetch this many clocks
logic        fetch_hold;
assign fetch_hold = (fetch_stall != 0) && xfer_v && (xfer_tm == 3'd2 || xfer_tm == 3'd6);

// alternate bus master (and arbiter)
logic        am_go, am_trig_en, am_trig;
logic  [2:0] am_n;
logic [15:0] am_delay;
logic [31:0] am_trig_a;
logic [31:0] am_x_addr [4];
logic  [4:0] am_x_ctl  [4];
logic [127:0] am_x_wd  [4];
logic [127:0] am_x_rd  [4];
logic  [1:0] am_status, am_status_q;

m68040_alt_master am (
	.clk(clk), .nreset(rsti_n), .bclk_en(bclk_en),
	.go(am_go), .n(am_n), .trig_en(am_trig_en), .trig(am_trig), .delay(am_delay),
	.x_addr(am_x_addr), .x_ctl(am_x_ctl), .x_wd(am_x_wd), .x_rd(am_x_rd), .status(am_status),
	.cpu_bg_n(am_bg_n), .cpu_bb_n(bb_oe ? bb_n_o : 1'b1),
	.drive(am_drive), .a(am_a), .rw_n(am_rw_n), .siz(am_siz), .tt(am_tt), .tm(am_tm),
	.sc(am_sc), .ts_n(am_ts_n), .bb_n(am_bb_n), .d(am_d), .d_oe(am_d_oe),
	.d_bus(bus_dr), .ta_n(ta_n), .tea_n(tea_n), .tbi_n(tbi_n)
);

// the trigger: the 68040 starts a line read of the armed line
assign am_trig = bclk_en && !ts_n && a_oe && !am_drive && rw_n && siz == 2'b11 &&
                 a_o[31:4] == am_trig_a[31:4];

// PST never shows a reserved encoding (6 is the 68040V's, 7 reserved),
// and shows "stopped" (D) once STOP has held for a few clocks
int stop_clks;
always_ff @(posedge clk) begin
	stop_clks <= (rsti_n && dut.be.stopped) ? stop_clks + 1 : 0;
	if (rsti_n && (pst == 4'h6 || pst == 4'h7)) begin
		$display("FAIL: PST shows the reserved encoding %h", pst);
		errors <= errors + 1;
	end
	if (stop_clks > 8 && pst != 4'hD) begin
		$display("FAIL: PST is %h during STOP", pst);
		errors <= errors + 1;
	end
end

// +amtrace: every bus transfer beat, by either master
logic amtrace;
initial amtrace = $test$plusargs("amtrace");
always_ff @(posedge clk)
	if (amtrace && bclk_en) begin
		if (!bus_ts_n)
			$display("%8d %s TS %08x %s siz=%0d sc=%0d mi_n=%b", cycles, am_drive ? "ALT" : "CPU",
			         bus_a, bus_rw_n ? "RD" : "WR", bus_siz, am_sc, mi_n);
		if (!ta_n || !tea_n)
			$display("%8d %s %s%s d=%08x (cpu ta_oe=%b mi_n=%b)", cycles, am_drive ? "ALT" : "CPU",
			         !ta_n ? "TA" : "", !tea_n ? "TEA" : "", bus_rw_n ? bus_dr : bus_dw, cpu_ta_oe, mi_n);
	end

always_ff @(posedge clk)
	if (amtrace && dut.dmu.sn_look)
		$display("%8d SNOOP %08x hit=%b dirty=%b src=%0d way=%0d", cycles, dut.dmu.sn_pa,
		         dut.dmu.sn_hit, dut.dmu.sn_dirty, dut.dmu.sn_src, dut.dmu.sn_way);

// results back into the register block the program reads
always_ff @(posedge clk) begin
	am_status_q <= am_status;
	if (am_status != am_status_q && am_status != 2'd0) begin
		mem.mem[16'hF28C >> 2] <= {14'd0, am_status, 16'h0000};
		for (int k = 0; k < 4; k++)
			for (int i = 0; i < 4; i++)
				mem.mem[(16'hF208 + 32 * k + 4 * i) >> 2] <= am_x_rd[k][127 - 32 * i -: 32];
	end
	if (am_go) mem.mem[16'hF28C >> 2] <= 32'd0;
end
int          cap;

always_comb begin
	tea_req = 1'b0;
	if (xfer_v) begin
		if (berr_armed && xfer_addr[15:0] == 16'hF140 && xfer_tt != 2'd3 &&
		    xfer_tm != 3'd2 && xfer_tm != 3'd6)
			tea_req = 1'b1;
		if (fberr_armed && (xfer_tm == 3'd2 || xfer_tm == 3'd6) &&
		    xfer_addr[15:2] == fberr_addr[15:2])
			tea_req = 1'b1;
		if (wberr_arm && (xfer_tm == 3'd3 || xfer_tm == 3'd4))
			tea_req = 1'b1;
	end
end

always_ff @(posedge clk) begin
	cycles <= cycles + 1;
	if (!rsti_n) begin
		ipl_lvl     <= 3'd0;
		berr_armed  <= 1'b1;     // the first access to $F140 is rejected (old bench)
		wberr_arm   <= 1'b0;
		fberr_armed <= 1'b0;
		fberr_addr  <= '0;
		ipl_delay   <= '0;
		ipl_pulse   <= '0;
		ipl_step    <= '0;
		ipl_next    <= '0;
		irq_exc_armed <= '0;
		fetch_stall <= '0;
		am_go       <= 1'b0;
		pins_r      <= 2'b00;
		rs_en <= 1'b0; rs_gap <= '0; rs_cnt <= '0; rs_op <= '0; rs_k <= '0; rs_seq <= '0;
		rs_busy <= 1'b0; rs_ops <= 0;
		for (int i = 0; i < 64; i++) rs_last[i] <= 0;
		am_n        <= '0;
		am_trig_en  <= 1'b0;
		am_trig_a   <= '0;
		am_delay    <= '0;
		for (int k = 0; k < 4; k++) begin am_x_addr[k] <= '0; am_x_ctl[k] <= '0; am_x_wd[k] <= '0; end
	end
	else begin
		am_go <= 1'b0;
		// random snoop traffic: one transfer at a time
		if (rs_en && !rs_busy && !am_go && am.st == am.A_IDLE) begin
			if (rs_cnt != 0) rs_cnt <= rs_cnt - 1'd1;
			else begin
				int r;
				logic [5:0] k;
				r = $urandom % 100;
				k = 6'($urandom);
				rs_k   <= k;
				rs_seq <= rs_seq + 1'd1;
				am_n   <= 3'd1;
				am_trig_en <= 1'b0;
				am_delay <= '0;
				if (r < 30) begin       // read a counter, SC 01
					rs_op <= 3'd0;
					am_x_addr[0] <= 32'h0000_A000 + {24'd0, k, 2'b00};
					am_x_ctl[0]  <= 5'b01_00_1;
				end
				else if (r < 50) begin  // write a shared word, SC 01
					rs_op <= 3'd1;
					am_x_addr[0] <= 32'h0000_B000 + {24'd0, k, 2'b00};
					am_x_ctl[0]  <= 5'b01_00_0;
					am_x_wd[0]   <= {{rs_seq, {8'd0, k, 2'b00} ^ 16'h5A5A}, 96'd0};
				end
				else if (r < 60) begin  // write a shared line, SC 01
					logic [127:0] l;
					rs_op <= 3'd2;
					for (int i = 0; i < 4; i++)
						l[127 - 32 * i -: 32] = {rs_seq, {8'd0, k[5:2], 2'(i), 2'b00} ^ 16'h5A5A};
					am_x_addr[0] <= 32'h0000_B000 + {24'd0, k[5:2], 4'd0};
					am_x_ctl[0]  <= 5'b01_11_0;
					am_x_wd[0]   <= l;
				end
				else if (r < 68) begin  // write a shared word, SC 10
					rs_op <= 3'd3;
					am_x_addr[0] <= 32'h0000_B000 + {24'd0, k, 2'b00};
					am_x_ctl[0]  <= 5'b10_00_0;
					am_x_wd[0]   <= {{rs_seq, {8'd0, k, 2'b00} ^ 16'h5A5A}, 96'd0};
				end
				else if (r < 88) begin  // read a shared word, SC 01
					rs_op <= 3'd4;
					am_x_addr[0] <= 32'h0000_B000 + {24'd0, k, 2'b00};
					am_x_ctl[0]  <= 5'b01_00_1;
				end
				else begin              // read a counter line, SC 01
					rs_op <= 3'd5;
					am_x_addr[0] <= 32'h0000_A000 + {24'd0, k[5:2], 4'd0};
					am_x_ctl[0]  <= 5'b01_11_1;
				end
				am_go   <= 1'b1;
				rs_busy <= 1'b1;
				rs_cnt  <= rs_gap;
			end
		end
		if (rs_busy && !am_go && am_status != 2'd0 && am.st == am.A_IDLE) begin
			rs_busy <= 1'b0;
			rs_ops  <= rs_ops + 1;
			if (am_status == 2'd2) begin
				$display("FAIL: snoop traffic: bus error");
				errors <= errors + 1;
			end
			else case (rs_op)
				3'd0: begin
					int v;
					v = am_x_rd[0][127:96];
					if (v < rs_last[rs_k]) begin
						$display("FAIL: snoop read of counter %0d went back: %0d after %0d", rs_k, v, rs_last[rs_k]);
						errors <= errors + 1;
					end
					rs_last[rs_k] <= v;
				end
				3'd4: if (!rs_valid(am_x_rd[0][127:96], rs_k)) begin
					$display("FAIL: snoop read of shared word %0d: %08x", rs_k, am_x_rd[0][127:96]);
					errors <= errors + 1;
				end
				3'd5: for (int i = 0; i < 4; i++) begin
					int v;
					v = am_x_rd[0][127 - 32 * i -: 32];
					if (v < rs_last[{rs_k[5:2], 2'(i)}]) begin
						$display("FAIL: snoop line read of counter %0d went back: %0d after %0d",
						         {rs_k[5:2], 2'(i)}, v, rs_last[{rs_k[5:2], 2'(i)}]);
						errors <= errors + 1;
					end
					rs_last[{rs_k[5:2], 2'(i)}] <= v;
				end
				default: ;
			endcase
		end
		if (fetch_hold) fetch_stall <= fetch_stall - 1'd1;
		// $F144 mode 1: IPL2 while TRAP #0 starts stacking; mode 2: IPL2
		// once its vector has been read, the handler's first fetch held
		if (irq_exc_armed == 2'd1 && dut.be.exc_go && dut.be.xi_vecw[9:2] == 8'd32) begin
			ipl_lvl <= 3'd2; irq_exc_armed <= 2'd0;
		end
		if (irq_exc_armed == 2'd2 && ev && ev_rd && ev_tm == 3'd5 &&
		    ev_addr == dut.be.vbr_r + 32'h80) begin
			ipl_lvl <= 3'd2; irq_exc_armed <= 2'd0; fetch_stall <= 3'd5;
		end
		if (ipl_delay != 0) begin
			ipl_delay <= ipl_delay - 1'd1;
			if (ipl_delay == 16'd1) ipl_lvl <= 3'd2;
		end
		if (ipl_pulse != 0) begin
			ipl_pulse <= ipl_pulse - 1'd1;
			if (ipl_pulse == 8'd1) ipl_lvl <= 3'd0;
		end
		if (ipl_step != 0) begin
			ipl_step <= ipl_step - 1'd1;
			if (ipl_step == 8'd1) ipl_lvl <= ipl_next;
		end
		if (ev && ev_err) begin
			// one-shot injections clear themselves
			if (ev_addr[15:0] == 16'hF140) berr_armed <= 1'b0;
			if (ev_tm == 3'd2 || ev_tm == 3'd6) fberr_armed <= 1'b0;
			if (ev_tm == 3'd3 || ev_tm == 3'd4) wberr_arm <= 1'b0;
		end
		if (ev && ev_rd && !ev_err && ev_tt == 2'd0 && !am_drive &&
		    ev_addr[15:0] == 16'hF2B0)
			mem.mem[16'hF2B4 >> 2] <= mem.mem[16'hF2B4 >> 2] + 32'd1;
		if (ev && !ev_rd && !ev_err && ev_tt == 2'd0 && !am_drive &&
		    ev_addr[15:0] == 16'hF294 && ev_be == 4'b1111)
			mem.mem[16'hF2A8 >> 2] <= ev_data;
		if (ev && !ev_rd && !ev_err && ev_tt == 2'd0) begin
			logic [15:0] w;
			w = ev_word(ev_addr, ev_data);
			case (ev_addr[15:0] & 16'hFFFE)
				16'hF102: begin
					if (w == 16'h600D) result <= 1;
					else begin
						result <= 2;
						$display("FAIL: program reports failure, test %0d (pc=%h)",
						         mem.mem[16'hF100 >> 2][31:16], dbg_pc);
						if (mem.mem[16'hF100 >> 2][31:16] == 16'd98)
							$display("     hfail from handler id %0d",
							         mem.mem[16'h3670 >> 2][31:16]);
					end
				end
				16'hF108: begin
					$display("STAMP tag=%04x cycles=%0d instructions=%0d", w, cycles - stamp_prev,
					         insns - insn_prev);
					if (prof) prof_report();
					stamp_prev <= cycles;
					insn_prev  <= insns;
				end
				16'hF110: ipl_lvl <= w[2:0];
				16'hF130: begin
					mem.mem[16'h3500 >> 2] <= {w, 16'h0000};
				end
				16'hF142: berr_armed <= 1'b1;
				16'hF144: irq_exc_armed <= w[1:0];
				16'hF146: wberr_arm <= 1'b1;
				16'hF148: ipl_delay <= w;
				16'hF14C: begin ipl_lvl <= w[2:0]; ipl_pulse <= w[15:8]; end
				16'hF150: begin ipl_lvl <= w[2:0]; ipl_next <= w[6:4]; ipl_step <= w[15:8]; end
				16'hF154: begin fberr_armed <= (w != 16'd0); fberr_addr <= w; end
				16'hF2C0: pins_r <= w[1:0];
				16'hF2C4: begin
					rs_en  <= w[15];
					rs_gap <= w[7:0];
					if (!w[15]) $display("snoop traffic: %0d transfers", rs_ops);
				end
				16'hF280: begin
					// the command block was written to memory by the slave
					for (int k = 0; k < 4; k++) begin
						am_x_addr[k] <= mem.mem[(16'hF200 + 32 * k) >> 2];
						am_x_ctl[k]  <= mem.mem[(16'hF204 + 32 * k) >> 2][20:16];
						am_x_wd[k]   <= {mem.mem[(16'hF208 + 32 * k) >> 2], mem.mem[(16'hF20C + 32 * k) >> 2],
						                 mem.mem[(16'hF210 + 32 * k) >> 2], mem.mem[(16'hF214 + 32 * k) >> 2]};
					end
					am_n       <= w[2:0];
					am_trig_en <= w[8];
					am_trig_a  <= mem.mem[16'hF284 >> 2];
					am_delay   <= mem.mem[16'hF288 >> 2][31:16];
					am_go      <= 1'b1;
				end
				default: ;
			endcase
			if ((ev_addr[15:0] & 16'hFFFC) == 16'hF120 && ev_tm != 3'd1) begin
				errors <= errors + 1;
				$display("FAIL: write to F120 with TM=%0d, expected 1", ev_tm);
			end
		end
	end
end

// optional pipeline trace
logic ptrace;
always_ff @(posedge clk)
	if (ptrace)
		$display("%8d AG=%b/%08x mem=%0d DC1=%b DC2=%b EX=%b WB=%b | m1=%b m2=%b m3=%b m4=%b rdy=%b e=%0d done=%b req=%b gnt=%b | f=%b hz=%b x=%b/%0d/%h dw=%b/%0d st1=%b",
		         cycles, dut.be.ag_v, dut.be.ag_u.pc, dut.be.ag_u.mem, dut.be.dc1_v, dut.be.dc2_v,
		         dut.be.ex_v, dut.be.wb_v, dut.dmu.m1.v, dut.dmu.m2.r.v, dut.dmu.m3.r.v, dut.dmu.m4.r.v,
		         dut.dmu.dc2_rdy, dut.dmu.e_st, dut.dmu.e_dc2_done, dut.dmu.b_req, dut.dmu.b_gnt,
		         dut.dmu.m2.fast, dut.dmu.hz, dut.dmu.m2.x.hit, dut.dmu.m2.x.way, dut.dmu.m2.x.pa,
		         dut.dmu.dw, dut.dmu.dw_set, dut.dmu.m1_stale);

// +ftrace=<from>,+ftto=<to>: the whole pipeline by instruction address,
// one line a cycle (Q words in the fetch queue, records, uops; holds)
int ft_from, ft_to;
initial begin
	if (!$value$plusargs("ftrace=%d", ft_from)) ft_from = -1;
	if (!$value$plusargs("ftto=%d", ft_to)) ft_to = ft_from + 200;
end
always_ff @(posedge clk)
	if (ft_from >= 0 && cycles >= ft_from && cycles < ft_to)
		$display("%8d Q%0d R%0d:%04x U%0d:%04x | AG %s DC1 %s DC2 %s EX %s WB %s | h ag=%b d1=%b d2=%b ex=%b wb=%b | %s%s",
		         cycles, dut.fetch.cnt, dut.rq_n, dut.rq0.pc[15:0], dut.uq_n, dut.uq0.pc[15:0],
		         dut.be.ag_v ? $sformatf("%04x", dut.be.ag_u.pc[15:0]) : "----",
		         dut.be.dc1_v ? $sformatf("%04x", dut.be.dc1_u.pc[15:0]) : "----",
		         dut.be.dc2_v ? $sformatf("%04x", dut.be.dc2_u.pc[15:0]) : "----",
		         dut.be.ex_v ? $sformatf("%04x", dut.be.ex_u.pc[15:0]) : "----",
		         dut.be.wb_v ? $sformatf("%04x", dut.be.wb_u.pc[15:0]) : "----",
		         dut.be.ag_hold, dut.dm_hold1, dut.be.dc2_hold, dut.be.ex_hold, dut.be.wb_hold,
		         dut.redir_v ? "REDIR " : "", dut.d_redir_v ? "DREDIR" : "");

// exception entries
always_ff @(posedge clk)
	if (trace && dut.be.x_go)
		$display("%8d EXC vec=%0d kind=%0d pc=%08x addr=%08x osr=%04x nsr=%04x stopped=%b",
		         cycles, dut.be.x_vec, dut.be.x_kind, dut.be.x_pc, dut.be.x_addr,
		         dut.be.x_osr, dut.be.x_nsr, dut.be.stopped);

// optional bus trace
logic trace;
always_ff @(posedge clk)
	if (trace && ev)
		$display("%8d BUS %s a=%08x d=%08x be=%b siz=%0d tt=%0d tm=%0d%s", cycles,
		         ev_rd ? "RD" : "WR", ev_addr, ev_data, ev_be, ev_siz, ev_tt, ev_tm,
		         ev_err ? " TEA" : "");
always_ff @(posedge clk)
	if (trace && dut.be.adv_wb)
		$display("%8d WB pc=%08x op=%0d last=%b d=%b r%0d=%08x upd=%b r%0d=%08x st=%b exc=%0d redir=%b->%08x",
		         cycles, dut.be.wb_u.pc, dut.be.wb_u.op, dut.be.wb_u.last,
		         dut.be.wb_dwe, dut.be.wb_u.d_reg, dut.be.wb_res,
		         dut.be.wb_u.upd_v, dut.be.wb_u.upd_reg, dut.be.wb_upd_val,
		         dut.be.wb_st, dut.be.wb_exc, dut.be.wb_redir, dut.be.wb_redir_pc);

// Interrupt invariants.  tb_qual models the IPEND claims from the core's
// synchronized level and its SR alone: level L is claimed once it is the
// visible level and beats the mask, until the level falls below L (the
// device let go) or an acceptance at L consumes it.  A level hidden behind
// a higher request holds no claim of its own.
logic [6:1] tb_qual;
logic [15:0] ipl_idle_for;
logic [15:0] ipend_takes;
wire        tb_irq_acc = dut.be.take_irq && !(dut.be.adv_wb && dut.be.wb_is_exc);
always_ff @(posedge clk) begin
	if (!rsti_n) begin
		tb_qual      <= '0;
		ipl_idle_for <= '0;
		ipend_takes  <= '0;
	end
	else begin
		for (int l = 1; l <= 6; l++) begin
			if (dut.be.ipl_q < 3'(l))
				tb_qual[l] <= 1'b0;
			else if (dut.be.ipl_q == 3'(l) && 3'(l) > dut.be.sr_r[10:8])
				tb_qual[l] <= 1'b1;
		end
		if (ipl_lvl == 3'd0) begin
			if (ipl_idle_for != 16'hFFFF) ipl_idle_for <= ipl_idle_for + 1'd1;
		end
		else ipl_idle_for <= '0;
		if (tb_irq_acc && !dut.be.nmi_edge) begin
			if (dut.be.irq_lvl != 3'd0 && dut.be.irq_lvl != 3'd7)
				tb_qual[dut.be.irq_lvl] <= 1'b0;
			if (ipl_idle_for > 16'd12) begin
				errors <= errors + 1;
				$display("FAIL: interrupt accepted %0d cycles after IPL went idle (phantom)",
				         ipl_idle_for);
			end
			if (dut.be.irq_lvl <= dut.be.irq_mask) begin
				if (dut.be.irq_lvl == 3'd0 || !tb_qual[dut.be.irq_lvl]) begin
					errors <= errors + 1;
					$display("FAIL: level %0d interrupt accepted at or below mask %0d without a claim (pc=%h)",
					         dut.be.irq_lvl, dut.be.irq_mask, dbg_pc);
				end
				else begin
					ipend_takes <= ipend_takes + 1'd1;
					mem.mem[16'hF164 >> 2] <= {ipend_takes + 16'd1, 16'h0000};
				end
			end
		end
		// IPEND: a claimed request is processed at the next boundary,
		// except the RTE boundary that resumes a MOVEM (SSW CM): the
		// continued MOVEM completes first
		if (dut.be.wb_bound && !dut.be.take_irq && !dut.be.cm_block &&
		    dut.be.ipl_q != 3'd0 && dut.be.ipl_q != 3'd7 && tb_qual[dut.be.ipl_q]) begin
			errors <= errors + 1;
			$display("FAIL: claimed level %0d request not taken at the boundary of pc=%h",
			         dut.be.ipl_q, dut.be.wb_u.pc);
		end
	end
end

logic [2:0] ipl_seen;
always_ff @(posedge clk) begin
	ipl_seen <= ipl_lvl;
	if (trace && ipl_lvl != ipl_seen)
		$display("%8d IPL %0d -> %0d (sr=%04x ipend=%b)", cycles, ipl_seen, ipl_lvl,
		         dut.be.sr_r, dut.be.ipend_r);
end

//--------------------------------------------------------------------------
// run
//--------------------------------------------------------------------------
int timeout;
int tbiv;

initial begin
	if (!$value$plusargs("prog=%s", prog)) begin
		$display("usage: +prog=<image.hex>");
		$finish;
	end
	if (!$value$plusargs("timeout=%d", timeout)) timeout = 2000000;
	wait_mode = $test$plusargs("waits") ? 2'd1 : 2'd0;
	if (!$value$plusargs("tbi=%d", tbiv)) tbiv = 0;
	tbi_mode = tbiv[1:0];
	if (!$value$plusargs("retry=%d", retry_pct)) retry_pct = 0;
	bclk2 = $test$plusargs("bclk2");
	trace = $test$plusargs("trace");
	ptrace = $test$plusargs("ptrace");
	prof = $test$plusargs("prof");
	result = 0;
	errors = 0;
	cycles = 0;
	stamp_prev = 0;
	insn_prev = 0;
	load_prog();
	if (!$value$plusargs("cap=%d", cap)) cap = 7;
	mem.mem[16'hF160 >> 2] = {cap[15:0], 16'h0000};
	mem.mem[16'hF164 >> 2] = 32'd0;
	repeat (8) @(posedge clk);
	rsti_n = 1'b1;
	while (result == 0 && cycles < timeout && !dbg_halted) @(posedge clk);
	if (prof) prof_report();
	if (pcprof) pcprof_dump();
	if (result == 1 && errors == 0)
		$display("PASS %s cycles=%0d", prog, cycles);
	else if (result == 0)
		$display("FAIL %s: %s at pc=%h, last test %0d", prog,
		         dbg_halted ? "halted" : "timeout", dbg_pc,
		         mem.mem[16'hF100 >> 2][31:16]);
	else
		$display("FAIL %s", prog);
	$finish;
end

endmodule
