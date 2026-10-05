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

ap68040 dut (
	.clk(clk), .bclk_en(bclk_en), .rsti_n(rsti_n),
	.a_o(a_o), .a_oe(a_oe), .d_i(d_mem), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .tln(tln), .upa(upa),
	.ciout_n(ciout_n), .lock_n(lock_n), .locke_n(locke_n),
	.ts_n(ts_n), .tip_n(tip_n), .ta_n(ta_n), .tea_n(tea_n),
	.tci_n(tci_n), .tbi_n(tbi_n), .ipl_n(~ipl_lvl), .avec_n(avec_n),
	.br_n(br_n), .bg_n(1'b0), .bb_n_i(bb_oe ? bb_n_o : 1'b1),
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
	.a(a_o), .d_cpu(d_o), .rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .ts_n(ts_n),
	.d_mem(d_mem), .ta_n(ta_n), .tea_n(tea_n), .tbi_n(tbi_n), .tci_n(tci_n),
	.avec_n(avec_n),
	.wait_mode(wait_mode), .tbi_mode(tbi_mode), .retry_pct(retry_pct),
	.tea_req(tea_req), .tci_req(1'b0), .hold(fetch_hold), .iack_vector(8'd0),
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
logic        berr_armed, wberr_arm, fberr_armed;
logic [15:0] fberr_addr;
logic [15:0] ipl_delay;
logic  [7:0] ipl_pulse, ipl_step;
logic  [2:0] ipl_next;
logic  [1:0] irq_exc_armed;   // $F144
logic  [2:0] fetch_stall;     // hold the next instruction fetch this many clocks
logic        fetch_hold;
assign fetch_hold = (fetch_stall != 0) && xfer_v && (xfer_tm == 3'd2 || xfer_tm == 3'd6);
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
	end
	else begin
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
					$display("STAMP tag=%04x cycles=%0d", w, cycles - stamp_prev);
					stamp_prev <= cycles;
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
	result = 0;
	errors = 0;
	cycles = 0;
	stamp_prev = 0;
	load_prog();
	if (!$value$plusargs("cap=%d", cap)) cap = 7;
	mem.mem[16'hF160 >> 2] = {cap[15:0], 16'h0000};
	mem.mem[16'hF164 >> 2] = 32'd0;
	repeat (8) @(posedge clk);
	rsti_n = 1'b1;
	while (result == 0 && cycles < timeout && !dbg_halted) @(posedge clk);
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
