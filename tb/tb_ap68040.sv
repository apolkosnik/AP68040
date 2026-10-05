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
//   $F146       arm a one-shot bus error on the next table search access  //
//   $F148 word  raise IPL 2 after the written number of clocks            //
//   $F14C word  IPL = bits 2:0, withdrawn after bits 15:8 clocks          //
//   $F150 word  IPL = bits 2:0, falls to bits 6:4 after bits 15:8 clocks  //
//   $F154 word  arm a one-shot bus error on an instruction fetch at the   //
//               written address (0 disarms)                               //
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
	.tea_req(tea_req), .tci_req(1'b0), .iack_vector(8'd0),
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
		berr_armed  <= 1'b0;
		wberr_arm   <= 1'b0;
		fberr_armed <= 1'b0;
		fberr_addr  <= '0;
		ipl_delay   <= '0;
		ipl_pulse   <= '0;
		ipl_step    <= '0;
		ipl_next    <= '0;
	end
	else begin
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
		$display("%8d AG=%b/%08x mem=%0d DC1=%b DC2=%b EX=%b WB=%b | m1=%b m2=%b m3=%b m4=%b rdy=%b q=%0d ldd=%b req=%b gnt=%b",
		         cycles, dut.be.ag_v, dut.be.ag_u.pc, dut.be.ag_u.mem, dut.be.dc1_v, dut.be.dc2_v,
		         dut.be.ex_v, dut.be.wb_v, dut.dmu.m1.v, dut.dmu.m2.v, dut.dmu.m3.v, dut.dmu.m4.v,
		         dut.dmu.dc2_rdy, dut.dmu.q_st, dut.dmu.q_ld_done, dut.dmu.b_req, dut.dmu.b_gnt);

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
