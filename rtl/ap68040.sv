//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// ap68040.sv - top level: the 68040 bus (MC68040UM section 7)               //
//                                                                          //
// clk is the processor clock (PCLK); bclk_en marks the processor clocks    //
// that end a bus clock (BCLK rising edge).  Bidirectional pins are split   //
// into _i/_o/_oe.                                                          //
//--------------------------------------------------------------------------//

module ap68040
	import ap68040_pkg::*;
(
	input  logic        clk,
	input  logic        bclk_en,
	input  logic        rsti_n,

	output logic [31:0] a_o,
	output logic        a_oe,
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

	// debug / test
	output logic [31:0] dbg_pc,
	output logic        dbg_retire,
	output logic        dbg_halted
);

// reset synchronizer
logic [1:0] rst_q;
always_ff @(posedge clk) rst_q <= {rst_q[0], rsti_n};
wire nreset = rst_q[1];

// interrupt level synchronizer (two bus-clock samples)
logic [2:0] ipl_s1, ipl_s2;
always_ff @(posedge clk)
	if (bclk_en) begin
		ipl_s1 <= ~ipl_n;
		ipl_s2 <= ipl_s1;
	end

//--------------------------------------------------------------------------
// BIU: client 0 data (DMU), client 1 instruction fetch
//--------------------------------------------------------------------------
localparam int NC = 2;
logic [NC-1:0] b_req, b_gnt, b_done, b_err;
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

ap68040_biu #(.NC(NC)) biu (
	.clk(clk), .nreset(nreset), .bclk_en(bclk_en),
	.req(b_req), .breq(b_breq), .wdata(b_wdata),
	.gnt(b_gnt), .done(b_done), .err(b_err),
	.rvalid(b_rvalid), .rclient(b_rclient), .rdata(b_rdata), .rbeat(b_rbeat),
	.ravec(b_ravec), .rtci(b_rtci), .errbeat(b_errbeat), .idle(b_idle),
	.unlock(kill_now),
	.rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.a_o(a_o), .a_oe(a_oe), .d_i(d_i), .d_o(d_o), .d_oe(d_oe),
	.rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .tln(tln), .upa(upa),
	.ciout_n(ciout_n), .lock_n(lock_n), .locke_n(locke_n),
	.ts_n(ts_n), .tip_n(tip_n), .ta_n(ta_n), .tea_n(tea_n),
	.tci_n(tci_n), .tbi_n(tbi_n), .avec_n(avec_n),
	.br_n(br_n), .bg_n(bg_n), .bb_n_i(bb_n_i), .bb_n_o(bb_n_o), .bb_oe(bb_oe),
	.rsto_n(rsto_n)
);

//--------------------------------------------------------------------------
// back end and front end
//--------------------------------------------------------------------------
logic        redir_v, flush;
logic [31:0] redir_pc;
logic        exc_go;
logic  [3:0] exc_kind;
logic  [4:0] exc_ssp;
logic        ucond_v, ucond;
logic [15:0] sr;
logic [31:0] vbr, cacr;
logic  [2:0] sfc, dfc;
logic        kill_now, adv_ag;
logic [31:0] dtt0, dtt1, itt0, itt1, tc;
logic        q_odd;
logic        dm_hold1, mt_v, mt_ng, mt_wr, mt_done;
logic  [2:0] mt_op, mt_fc;
logic  [1:0] mt_scope, mt_caches;
logic [31:0] mt_addr, mt_mmusr, urp, srp, dm_st_faddr;
logic  [2:0] iack_lvl;
logic        dm_iack;

logic        dm_req, adv_dc1, adv_dc2, adv_ex, adv_wb;
logic [31:0] dm_va;
logic  [1:0] dm_mem, dm_msz;
logic  [2:0] dm_fc;
logic        dm_lock, dm_locke, dm_super, dm_noalloc;
logic        dm_dc2_rdy, dm_fault, dm_st_v, dm_st_rdy, dm_st_fault;
logic [31:0] dm_ldata, dm_faddr, dm_st_data;
logic  [7:0] dm_fvec;
logic [15:0] dm_fssw, dm_st_fssw;

logic  [1:0] uq_n;
uop_t        uq0;
logic        uo_rdy;

ap68040_backend be (
	.clk(clk), .nreset(nreset),
	.in_v(uq_n != 2'd0), .in_u(uq0), .in_rdy(uo_rdy),
	.redir_v(redir_v), .redir_pc(redir_pc), .flush(flush),
	.exc_go(exc_go), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.ucond_v(ucond_v), .ucond(ucond),
	.sr(sr), .vbr(vbr), .cacr(cacr), .sfc(sfc), .dfc(dfc),
	.dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem), .dm_msz(dm_msz),
	.dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke), .dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_iack(dm_iack),
	.adv_dc1(adv_dc1), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.dm_dc2_rdy(dm_dc2_rdy), .dm_ldata(dm_ldata), .dm_fault(dm_fault),
	.dm_fvec(dm_fvec), .dm_faddr(dm_faddr), .dm_fssw(dm_fssw),
	.dm_st_v(dm_st_v), .dm_st_data(dm_st_data), .dm_st_rdy(dm_st_rdy),
	.dm_st_fault(dm_st_fault), .dm_st_fssw(dm_st_fssw),
	.dm_hold1(dm_hold1), .mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope),
	.mt_caches(mt_caches), .mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng),
	.mt_wr(mt_wr), .mt_done(mt_done), .mt_mmusr(mt_mmusr), .dm_st_faddr(dm_st_faddr),
	.urp(urp), .srp(srp),
	.ipl(ipl_s2), .iack_lvl(iack_lvl), .rsto_req(rsto_req), .rsto_busy(rsto_busy),
	.kill_now(kill_now), .adv_ag(adv_ag),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1), .tc(tc),
	.dbg_pc(dbg_pc), .dbg_retire(dbg_retire), .halted(dbg_halted)
);

ap68040_dmu dmu (
	.clk(clk), .nreset(nreset),
	.adv_ag(adv_ag), .dm_req(dm_req), .dm_va(dm_va), .dm_mem(dm_mem),
	.dm_msz(dm_msz), .dm_fc(dm_fc), .dm_lock(dm_lock), .dm_locke(dm_locke),
	.dm_super(dm_super), .dm_noalloc(dm_noalloc), .dm_iack(dm_iack), .iack_lvl(iack_lvl),
	.adv_dc1(adv_dc1), .adv_dc2(adv_dc2), .adv_ex(adv_ex), .adv_wb(adv_wb),
	.kill_now(kill_now), .dm_hold1(dm_hold1),
	.dc2_rdy(dm_dc2_rdy), .ldata(dm_ldata), .fault(dm_fault), .fvec(dm_fvec),
	.faddr(dm_faddr), .fssw(dm_fssw),
	.st_v(dm_st_v), .st_data(dm_st_data), .st_rdy(dm_st_rdy),
	.st_fault(dm_st_fault), .st_fssw(dm_st_fssw), .st_faddr(dm_st_faddr),
	.mt_v(mt_v), .mt_op(mt_op), .mt_scope(mt_scope), .mt_caches(mt_caches),
	.mt_addr(mt_addr), .mt_fc(mt_fc), .mt_ng(mt_ng), .mt_wr(mt_wr),
	.mt_done(mt_done), .mt_mmusr(mt_mmusr),
	.cacr(cacr), .tc(tc), .urp(urp), .srp(srp),
	.dtt0(dtt0), .dtt1(dtt1), .itt0(itt0), .itt1(itt1),
	.iw_req(1'b0), .iw_va(32'd0), .iw_fc2(1'b0), .iw_done(), .iw_ent(),
	.ic_inv(), .ic_inv_scope(), .ic_inv_pa(), .ic_inv_done(1'b1),
	.iatc_flush_all(), .iatc_flush_page(), .iatc_flush_ng(), .iatc_flush_la(),
	.iatc_flush_fc2(), .iatc_wr(), .iatc_wla(), .iatc_wfc2(), .iatc_went(),
	.b_req(b_req[0]), .b_breq(b_breq[0]), .b_wdata(b_wdata[0]),
	.b_gnt(b_gnt[0]), .b_done(b_done[0]), .b_err(b_err[0]),
	.b_rvalid(b_rvalid && b_rclient == 1'b0), .b_rdata(b_rdata), .b_rbeat(b_rbeat),
	.b_ravec(b_ravec), .b_rtci(b_rtci)
);

// front end
logic [15:0] win [8];
logic  [7:0] win_flt, win_fdem;
pd_t         pd0;
logic  [3:0] qcnt;
logic [31:0] qpc;
logic  [2:0] consume;
logic        d_redir_v;
logic [31:0] d_redir_pc;
logic  [1:0] rq_n;
dinst_t      rq0, rq1;
logic        rq_pop;

ap68040_fetch fetch (
	.clk(clk), .nreset(nreset),
	.redir_v(redir_v), .redir_pc(redir_pc),
	.d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	.stop(1'b0), .smode(sr[13]),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .pd0(pd0), .qcnt(qcnt), .qpc(qpc), .consume(consume),
	.q_odd(q_odd),
	.b_req(b_req[1]), .b_breq(b_breq[1]),
	.b_gnt(b_gnt[1]), .b_done(b_done[1]), .b_err(b_err[1]),
	.b_rvalid(b_rvalid && b_rclient == 1'b1), .b_rdata(b_rdata)
);
assign b_wdata[1] = '0;

ap68040_decode dec (
	.clk(clk), .nreset(nreset), .flush(flush),
	.win(win), .win_flt(win_flt), .win_fdem(win_fdem), .pd0(pd0), .qcnt(qcnt), .qpc(qpc), .q_odd(q_odd),
	.smode(sr[13]),
	.consume(consume), .d_redir_v(d_redir_v), .d_redir_pc(d_redir_pc),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop)
);

ap68040_useq useq (
	.clk(clk), .nreset(nreset), .flush(flush),
	.rq_n(rq_n), .rq0(rq0), .rq1(rq1), .rq_pop(rq_pop),
	.smode(sr[13]), .master(sr[12]),
	.exc_go(exc_go), .exc_kind(exc_kind), .exc_ssp(exc_ssp),
	.uq_n(uq_n), .uq0(uq0), .uq_pop(uo_rdy && uq_n != 2'd0)
);

endmodule
