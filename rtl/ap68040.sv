//--------------------------------------------------------------------------//
// AP68040 - pipelined MC68040                                               //
//                                                                          //
// ap68040.sv - top level: the 68040 bus (MC68040UM section 7)               //
//                                                                          //
// clk is the processor clock (PCLK); bclk_en marks the processor clocks    //
// that end a bus clock (BCLK rising edge).  Bidirectional pins are split   //
// into _i/_o/_oe: a_i, ts_n_i, rw_n_i, siz_i and tt_i are the bus as an    //
// alternate master drives it (snooped, with SC1/SC0), d_i the data bus    //
// in either direction, ta_n the bus TA; the 68040 drives TA (ta_n_o,      //
// ta_oe) and D31-D0 only as a snooping slave.                             //
//--------------------------------------------------------------------------//

module ap68040
	import ap68040_pkg::*;
#(
	parameter logic [7:0] FPU_REVISION = 8'h41,  // FSAVE frames: $41, or $40 (44-byte UNIMP)
	// area options (doc/STATUS.md has what each costs in ALMs and cycles)
	parameter bit         FPU_PIPE     = 0,      // the FPU's arithmetic pipeline (0: the state machine alone)
	parameter bit         LOOP_PRED    = 0,      // D1's loop exit predictor
	parameter bit         EARLY_RES    = 0,      // AG's early results for younger uops' base and index
	parameter bit         DC1_ATC      = 0,      // DC1's own ATC port and tag compare (timing)
	parameter bit         SKID         = 0,      // the skid register in front of AG (timing)
	parameter int         DIV_STEPS    = 2,      // divide steps per cycle (2 or 4)
	parameter bit         DUAL         = 1,      // dual issue: the second slot (ap68040_pkg s1_t)
	parameter bit         BTIC         = 1       // the fetch BTB's target words
)(

	input  logic        clk,
	input  logic        bclk_en,
	input  logic        rsti_n,

	output logic [31:0] a_o,
	output logic        a_oe,
	input  logic [31:0] a_i,
	input  logic        ts_n_i,
	input  logic        rw_n_i,
	input  logic  [1:0] siz_i,
	input  logic  [1:0] tt_i,
	input  logic  [1:0] sc,
	output logic        mi_n,
	output logic        ta_n_o,
	output logic        ta_oe,
	input  logic [31:0] d_i,
	output logic [31:0] d_o,
	output logic        d_oe,
	output logic        rw_n,
	output logic  [1:0] siz,
	output logic  [1:0] tt,
	output logic  [2:0] tm,
	output logic  [1:0] tln,
	output logic  [1:0] upa,
	output logic        ciout_n,
	output logic        lock_n,
	output logic        locke_n,
	output logic        ts_n,
	output logic        tip_n,
	input  logic        ta_n,
	input  logic        tea_n,
	input  logic        tci_n,
	input  logic        tbi_n,
	input  logic  [2:0] ipl_n,
	input  logic        avec_n,
	output logic        br_n,
	input  logic        bg_n,
	input  logic        bb_n_i,
	output logic        bb_n_o,
	output logic        bb_oe,
	output logic        rsto_n,
	input  logic        cdis_n,       // CDIS: disable the caches (contents kept)
	input  logic        mdis_n,       // MDIS: disable page translation
	output logic        ipend_n,      // IPEND
	output logic  [3:0] pst,          // PST3-PST0 (MC68040UM table 5-6)

	// debug / test
	output logic [31:0] dbg_pc,
	output logic        dbg_retire,
	output logic        dbg_halted,
	// for a system's external caches and monitors: VBR (the NMI vector's
	// address), CACR, and a pulse for every completed CINV/CPUSH
	output logic [31:0] dbg_vbr,
	output logic [31:0] dbg_cacr,
	output logic        dbg_cmaint,
	// observation only: the fetch unit, the decoder's queue, the redirects,
	// D1's BTB writes and WB's PC (384 bits, see the assignment)
	output logic [383:0] dbg_fe
);

// reset synchronizer: two bus clocks, so the reset and the release of the
// bus pins it begins start on a bus clock edge.  Once RSTI negates the
// processor stays in reset for another 128 bus clocks (MC68040UM 7.10.1).
logic [1:0] rst_q = 2'b00;
logic [7:0] rst_hold = 8'd128;
always_ff @(posedge clk) begin
	if (bclk_en) rst_q <= {rst_q[0], rsti_n};
	if (!rst_q[1])                         rst_hold <= 8'd128;
	else if (bclk_en && rst_hold != 8'd0)  rst_hold <= rst_hold - 8'd1;
end
wire nreset = rst_q[1] && (rst_hold == 8'd0);

// CDIS and MDIS act at the next internal boundary: two bus-clock samples
// (the multiplexed bus mode CDIS selects at reset is not provided)
logic [1:0] cdis_s, mdis_s;
always_ff @(posedge clk)
	if (bclk_en) begin
		cdis_s <= {cdis_s[0], !cdis_n};
		mdis_s <= {mdis_s[0], !mdis_n};
	end
wire cdis = cdis_s[1];
wire mdis = mdis_s[1];

// IPEND and PST change on bus clock edges.  PST shows the end of each
// instruction for one bus clock (the last one when several end within
// it), else the present status.
logic        irq_pending, pst_ev, pst_b;
logic  [3:0] pst_code, pst_st, pst_bc;
always_ff @(posedge clk) begin
	if (!nreset) begin
		ipend_n <= 1'b1;
		pst     <= 4'h8;
		pst_b   <= 1'b0;
		pst_bc  <= 4'h8;
	end
	else begin
		if (pst_ev) begin
			pst_b  <= 1'b1;
			pst_bc <= pst_code;
		end
		if (bclk_en) begin
			ipend_n <= !irq_pending;
			pst     <= pst_ev ? pst_code : pst_b ? pst_bc : pst_st;
			pst_b   <= 1'b0;
		end
	end
end

// interrupt level synchronizer: IPL2-IPL0 are sampled on consecutive bus
// clock edges, and a level is valid once two consecutive samples agree
// (MC68040UM 7.8.1: "held constant for two consecutive clock periods")
// -- the three pins settle at different times when the level changes, so
// a single sample can show a level no device raised
logic [2:0] ipl_s1 = 3'd0, ipl_s2 = 3'd0, ipl_v = 3'd0;
always_ff @(posedge clk)
	if (bclk_en) begin
		ipl_s1 <= ~ipl_n;
		ipl_s2 <= ipl_s1;
		if (ipl_s1 == ipl_s2) ipl_v <= ipl_s2;
	end

//--------------------------------------------------------------------------
// BIU: client 0 data (DMU), client 1 instruction fetch
//--------------------------------------------------------------------------
localparam int NC = 2;
logic [NC-1:0] b_req, b_gnt, b_done, b_err;
logic          b_own;
logic [31:0]   b_d_o, sn_d_o;
logic          b_d_oe, sn_d_oe;
busreq_t       b_breq [NC];
logic [127:0]  b_wdata [NC];
logic          b_rvalid;
logic          b_rclient;
logic [31:0]   b_rdata;
logic  [1:0]   b_rbeat;
logic          b_ravec, b_rtci;
logic  [1:0]   b_errbeat;
logic          b_idle;
logic          rsto_req, rsto_busy;
logic          be_fstop;
logic          b_drop;

ap68040_biu #(.NC(NC)) biu (
	.clk(clk), .nreset(nreset), .bclk_en(bclk_en),
	// (client 1, the fetch, is held while stopped, halted or resetting)
	.req(b_req & ~{be_fstop || rsto_busy, 1'b0}), .breq(b_breq), .wdata(b_wdata),
	.gnt(b_gnt), .done(b_done), .err(b_err),
	.rvalid(b_rvalid), .rclient(b_rclient), .rdata(b_rdata), .rbeat(b_rbeat),
	.ravec(b_ravec), .rtci(b_rtci), .errbeat(b_errbeat), .idle(b_idle), .own(b_own),
	.unlock(lk_abandon), .halt(dbg_halted), .more({1'b0, 1'b0}), .drop({1'b0, b_drop}),
	.rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.a_o(a_o), .a_oe(a_oe), .d_i(d_i), .d_o(b_d_o), .d_oe(b_d_oe),
	.rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .tln(tln), .upa(upa),
	.ciout_n(ciout_n), .lock_n(lock_n), .locke_n(locke_n),
	.ts_n(ts_n), .tip_n(tip_n), .ta_n(ta_n), .tea_n(tea_n),
	.tci_n(tci_n), .tbi_n(tbi_n), .avec_n(avec_n),
	.br_n(br_n), .bg_n(bg_n), .bb_n_i(bb_n_i), .bb_n_o(bb_n_o), .bb_oe(bb_oe),
	.rsto_n(rsto_n)
);

//--------------------------------------------------------------------------
// bus snooper
//--------------------------------------------------------------------------
logic        sn_req, sn_look, sn_hit, sn_dirty, sn_inv, sn_wr;
logic [31:0] sn_pa;
logic [127:0] sn_line;
logic [31:0] sn_wword;
logic [15:0] sn_wbe;
logic        sn_ic, sn_ic_all, sn_ic_done, sn_ihit;
logic [31:0] sn_ic_pa;

ap68040_snoop snoop (
	.clk(clk), .nreset(nreset), .bclk_en(bclk_en), .own(b_own),
	.a_i(a_i), .ts_n_i(ts_n_i), .rw_n_i(rw_n_i), .siz_i(siz_i), .tt_i(tt_i), .sc(sc),
	.d_i(d_i), .ta_n_i(ta_n), .tea_n_i(tea_n), .tbi_n_i(tbi_n),
	.mi_n(mi_n), .ta_n_o(ta_n_o), .ta_oe(ta_oe), .d_o(sn_d_o), .d_oe(sn_d_oe),
	.dc_req(sn_req), .dc_pa(sn_pa), .dc_look(sn_look), .dc_hit(sn_hit),
	.dc_dirty(sn_dirty), .dc_line(sn_line), .dc_inv(sn_inv), .dc_wr(sn_wr),
	.dc_wbe(sn_wbe), .dc_wword(sn_wword),
	.ic_req(sn_ic), .ic_pa(sn_ic_pa), .ic_all(sn_ic_all), .ic_done(sn_ic_done)
);

assign d_o  = sn_d_oe ? sn_d_o : b_d_o;
assign d_oe = sn_d_oe || b_d_oe;

//--------------------------------------------------------------------------
// back end and front end
//--------------------------------------------------------------------------
logic        redir_v, flush, fe_flush;
logic [31:0] redir_pc;
logic        exc_go, exc_pend;
logic  [3:0] exc_kind;
logic  [4:0] exc_ssp;
logic        ucond_v, ucond;
logic [15:0] sr;
logic [31:0] vbr, cacr;
logic  [2:0] sfc, dfc;
logic        kill_now, adv_ag, lk_abandon;
logic [31:0] dtt0, dtt1, itt0, itt1, tc;
logic        q_odd;
logic        dm_hold1, mt_v, mt_ng, mt_wr, mt_done, mt_fault;
logic  [2:0] mt_op, mt_fc;
logic  [1:0] mt_scope, mt_caches;
logic [31:0] mt_addr, mt_mmusr, mt_faddr, urp, srp, dm_st_faddr;
logic [15:0] mt_fssw;
logic  [2:0] iack_lvl;
logic  [1:0] dm_ack;

logic        dm_req, adv_dc1, adv_dc2, adv_ex, adv_wb;
logic [31:0] dm_va;
logic  [1:0] dm_mem, dm_msz;
logic  [2:0] dm_fc;
logic        dm_lock, dm_locke, dm_super, dm_noalloc, dm_pastc, dm_older, dm_s1;
logic        bht_we, tw_busy;
logic  [7:0] bht_wa;
logic        bht_dis;
logic        lp_we, lp_tk, lp_mp;
logic [11:0] lp_wa;
logic        dm_dc2_rdy, dm_fault, dm_st_v, dm_st_rdy, dm_st_fault;
logic [31:0] dm_ldata, dm_faddr, dm_st_data, dm_ldata_al;
logic        dm_ldata_alv;
logic  [7:0] dm_fvec;
logic [15:0] dm_fssw, dm_st_fssw;

logic  [1:0] uq_n;
uop_t        uq0;
s1_t         uq0s;
logic        uo_rdy;

ap68040_backend #(.FPU_REVISION(FPU_REVISION), .FPU_PIPE(FPU_PIPE), .EARLY_RES(EARLY_RES),
                  .SKID(SKID), .DIV_STEPS(DIV_STEPS), .DUAL(DUAL)) be (
	.clk(clk), .nreset(nreset),
	.in_v(uq_n != 2'd0), .in_u(uq0), .in_s(uq0s), .in_rdy(uo_rdy),
	.redir_v(redir_v), .redir_pc(redir_pc), .flush(flush), .fe_flush(fe_flush),
	.ras_rv(ras_rv), .ras_rtp(ras_rtp), .ras_rn(ras_rn),
	.exc_go(exc_go), .exc_pend(exc_pend), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.ucond_v(ucond_v), .ucond(ucond),
	.sr(sr), .vbr(vbr), .cacr(cacr), .sfc(sfc), .dfc(dfc),
	.dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem), .dm_msz(dm_msz),
	.dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke), .dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_pastc(dm_pastc), .dm_ack(dm_ack), .dm_older(dm_older), .dm_s1(dm_s1),
	.bht_we(bht_we), .bht_wa(bht_wa), .bht_dis(bht_dis),
	.lp_we(lp_we), .lp_wa(lp_wa), .lp_tk(lp_tk), .lp_mp(lp_mp), .sn_ihit(sn_ihit),
	.tw_busy(tw_busy), .pst_ev(pst_ev), .pst_code(pst_code), .pst_st(pst_st),
	.irq_pending(irq_pending),
	.adv_dc1(adv_dc1), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.dm_dc2_rdy(dm_dc2_rdy), .dm_ldata(dm_ldata), .dm_ldata_al(dm_ldata_al), .dm_ldata_alv(dm_ldata_alv), .dm_fault(dm_fault),
	.dm_fvec(dm_fvec), .dm_faddr(dm_faddr), .dm_fssw(dm_fssw),
	.dm_st_v(dm_st_v), .dm_st_data(dm_st_data), .dm_st_rdy(dm_st_rdy),
	.dm_st_fault(dm_st_fault), .dm_st_fssw(dm_st_fssw),
	.dm_hold1(dm_hold1), .mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope),
	.mt_caches(mt_caches), .mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng),
	.mt_wr(mt_wr), .mt_done(mt_done), .mt_mmusr(mt_mmusr), .mt_fault(mt_fault),
	.mt_fssw(mt_fssw), .mt_faddr(mt_faddr), .dm_st_faddr(dm_st_faddr),
	.urp(urp), .srp(srp),
	.ipl(ipl_v), .iack_lvl(iack_lvl), .rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.kill_now(kill_now), .lk_abandon(lk_abandon), .adv_ag(adv_ag),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1), .tc(tc),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .dbg_retire2(), .halted(dbg_halted), .fetch_stop(be_fstop)
);

ap68040_dmu #(.DC1_ATC(DC1_ATC)) dmu (
	.clk(clk), .nreset(nreset),
	.adv_ag(adv_ag), .dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem),
	.dm_msz(dm_msz), .dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke),
	.dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_pastc(dm_pastc), .dm_ack(dm_ack), .iack_lvl(iack_lvl),
	.dm_older(dm_older), .dm_s1(dm_s1),
	.adv_dc1(adv_dc1), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.kill_now(kill_now), .dm_hold1(dm_hold1),
	.dc2_rdy(dm_dc2_rdy), .ldata(dm_ldata), .ldata_al(dm_ldata_al), .ldata_alv(dm_ldata_alv), .fault(dm_fault), .fvec(dm_fvec),
	.faddr(dm_faddr), .fssw(dm_fssw),
	.st_v(dm_st_v), .st_data(dm_st_data), .st_rdy(dm_st_rdy),
	.st_fault(dm_st_fault), .st_fssw(dm_st_fssw), .st_faddr(dm_st_faddr),
	.mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope), .mt_caches(mt_caches),
	.mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng), .mt_wr(mt_wr),
	.mt_done(mt_done), .mt_mmusr(mt_mmusr), .mt_fault(mt_fault), .mt_fssw(mt_fssw),
	.mt_faddr(mt_faddr),
	.cacr(cacr), .tc(tc), .urp(urp), .srp(srp),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1),
	.cdis(cdis), .mdis(mdis), .tw_busy(tw_busy),
	.iw_req(iw_req), .iw_va(iw_va), .iw_fc2(iw_fc2), .iw_done(iw_done), .iw_ent(),
	.ic_inv(ic_inv), .ic_inv_scope(ic_inv_scope), .ic_inv_pa(ic_inv_pa),
	.ic_inv_done(ic_inv_done),
	.iatc_flush_all(iatc_flush_all), .iatc_flush_page(iatc_flush_page),
	.iatc_flush_ng(iatc_flush_ng), .iatc_flush_la(iatc_flush_la),
	.iatc_flush_fc2(iatc_flush_fc2), .iatc_wr(iatc_wr), .iatc_wla(iatc_wla),
	.iatc_wfc2(iatc_wfc2), .iatc_went(iatc_went),
	.b_req(b_req[0]), .b_breq(b_breq[0]), .b_wdata(b_wdata[0]),
	.b_gnt(b_gnt[0]), .b_done(b_done[0]), .b_err(b_err[0]), .b_drop(b_drop),
	.b_rvalid(b_rvalid && b_rclient == 1'b0), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.b_ravec(b_ravec), .b_rtci(b_rtci),
	.sn_req(sn_req), .sn_pa(sn_pa), .sn_look(sn_look), .sn_hit(sn_hit),
	.sn_dirty(sn_dirty), .sn_line(sn_line), .sn_inv(sn_inv), .sn_wr(sn_wr),
	.sn_wbe(sn_wbe), .sn_wword(sn_wword)
);

// instruction side <-> DMU
logic        iw_req, iw_fc2, iw_done;
logic [31:0] iw_va;
logic        ic_inv, ic_inv_done;
logic  [1:0] ic_inv_scope;
logic [31:0] ic_inv_pa;
logic        iatc_flush_all, iatc_flush_page, iatc_flush_ng, iatc_flush_fc2;
logic [31:0] iatc_flush_la;
logic        iatc_wr, iatc_wfc2;
logic [31:0] iatc_wla;
atce_t       iatc_went;

// front end
logic [15:0] win [8];
logic  [7:0] win_flt, win_fdem, win_fatc, win_bt, win_s1;
logic [31:0] bt_tgt;
logic        btb_we, btb_wv, btb_wslot;
logic [BTB_AW-1:0] btb_wi;
logic [BTB_TW-1:0] btb_wtag;
logic [30:0] btb_wtgt;
logic  [1:0] btb_wkind;
logic [30:0] d_ras;
logic        ras_rv;
logic  [2:0] ras_rtp;
logic  [3:0] ras_rn;
logic  [2:0] d_ras_tp;
logic  [3:0] d_ras_n;
pd_t         pd0, pd0_n;
logic  [3:0] qcnt;
logic [31:0] qpc;
logic  [2:0] consume;
logic        consume_t;
logic        d_redir_v;
logic [31:0] d_redir_pc;
logic  [1:0] rq_n;
dinst_t      rq0, rq1;
logic        rq_pop, rq_pop_e;

ap68040_fetch #(.BTIC(BTIC)) fetch (
	.clk(clk), .nreset(nreset),
	.redir_v(redir_v), .redir_pc(redir_pc),
	.d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	// no fetches while stopped or halted, nor while RSTO is asserted
	.stop(be_fstop || rsto_busy), .smode(sr[13]),
	.cacr(cacr), .tc(tc), .itt0(itt0), .itt1(itt1), .cdis(cdis), .mdis(mdis),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .win_fatc(win_fatc),
	.win_bt(win_bt), .win_s1(win_s1), .bt_tgt(bt_tgt),
	.pd0(pd0), .pd0_n(pd0_n), .qcnt(qcnt), .qpc(qpc), .consume(consume), .consume_t(consume_t),
	.q_odd(q_odd),
	.btb_we(btb_we), .btb_wi(btb_wi), .btb_wv(btb_wv), .btb_wtag(btb_wtag),
	.btb_wslot(btb_wslot), .btb_wkind(btb_wkind), .btb_wtgt(btb_wtgt),
	.d_ras(d_ras), .d_ras_tp(d_ras_tp), .d_ras_n(d_ras_n),
	.iw_req(iw_req), .iw_va(iw_va), .iw_fc2(iw_fc2), .iw_done(iw_done),
	.iatc_wr(iatc_wr), .iatc_wla(iatc_wla), .iatc_wfc2(iatc_wfc2), .iatc_went(iatc_went),
	.iatc_flush_all(iatc_flush_all), .iatc_flush_page(iatc_flush_page),
	.iatc_flush_ng(iatc_flush_ng), .iatc_flush_la(iatc_flush_la),
	.iatc_flush_fc2(iatc_flush_fc2),
	.ic_inv(ic_inv), .ic_inv_scope(ic_inv_scope), .ic_inv_pa(ic_inv_pa),
	.ic_inv_done(ic_inv_done),
	.sn_inv(sn_ic), .sn_inv_pa(sn_ic_pa), .sn_inv_all(sn_ic_all), .sn_inv_done(sn_ic_done),
	.sn_ihit(sn_ihit), .dbg_f(dbg_f),
	.b_req(b_req[1]), .b_breq(b_breq[1]),
	.b_gnt(b_gnt[1]), .b_done(b_done[1]), .b_err(b_err[1]),
	.b_rvalid(b_rvalid && b_rclient == 1'b1), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.b_rtci(b_rtci)
);
assign b_wdata[1] = '0;

ap68040_decode #(.LOOP_PRED(LOOP_PRED), .DUAL(DUAL)) dec (
	.clk(clk), .nreset(nreset), .flush(fe_flush),
	.ras_rv(ras_rv), .ras_rtp(ras_rtp), .ras_rn(ras_rn),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .win_fatc(win_fatc),
	.win_bt(win_bt), .win_s1(win_s1), .bt_tgt(bt_tgt),
	.pd0(pd0), .pd0_n(pd0_n), .qcnt(qcnt), .qpc(qpc), .q_odd(q_odd),
	.smode(sr[13]),
	.consume(consume), .consume_t(consume_t), .d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop), .rq_pop_e(rq_pop_e),
	.bht_we(bht_we), .bht_wa(bht_wa), .bht_dis(bht_dis),
	.lp_we(lp_we), .lp_wa(lp_wa), .lp_tk(lp_tk), .lp_mp(lp_mp),
	.btb_we(btb_we), .btb_wi(btb_wi), .btb_wv(btb_wv), .btb_wtag(btb_wtag),
	.btb_wslot(btb_wslot), .btb_wkind(btb_wkind), .btb_wtgt(btb_wtgt),
	.ras_o(d_ras), .ras_tp_o(d_ras_tp), .ras_n_o(d_ras_n)
);

ap68040_useq #(.DUAL(DUAL)) useq (
	.clk(clk), .nreset(nreset), .flush(fe_flush),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop), .rq_pop_e(rq_pop_e),
	.smode(sr[13]), .master(sr[12]), .trace(sr[15] || sr[14]),
	.exc_go(exc_go), .exc_pend(exc_pend), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.uq_n(uq_n), .uq0(uq0), .uq0s(uq0s), .uq_pop(uo_rdy && uq_n != 2'd0)
);

assign dbg_vbr  = vbr;
assign dbg_cacr = cacr;
// (mt_op 1 and 2: MT_CINV and MT_CPUSH of the DMU)
always_ff @(posedge clk)
	dbg_cmaint <= nreset && mt_done && (mt_op == 3'd1 || mt_op == 3'd2);

logic [191:0] dbg_f;
assign dbg_fe = {20'd0, consume, qcnt, d_redir_v, redir_v, btb_we, btb_wkind, btb_wslot, btb_wtgt,
                 d_redir_pc, redir_pc, qpc, dbg_pc, dbg_f};

endmodule
