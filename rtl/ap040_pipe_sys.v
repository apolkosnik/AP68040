//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 81)                 //
//                                                                          //
// ap040_pipe_sys.v - the CPU on a bus instead of an array                  //
//                                                                          //
// The second of the two pairings ap040_pipe_cpu.v supports:                //
// ap040_pipe_core.v puts the array behind it for the milestone benches,    //
// this puts ap040_pipe_membus.v and a real memory port behind it. Nothing  //
// in the CPU differs between them.                                         //
//                                                                          //
// The port below is rtl/ap040/ap040_core.v's, so the next step is          //
// ap040_bus16_adapter.v underneath this without touching either side.      //
//                                                                          //
// One consequence of the CPU emitting byte addresses (milestone 81) shows  //
// up here and not in ap040_pipe_core.v: the exception vector table is at   //
// its architectural place, vector n at byte 4n, rather than aliased into   //
// the array's window the way every tb_ap040_pipe_*.v sees it (word index   //
// 3584 + 2n, which is 4n - PC_RESET wrapped). VBR is still not consulted   //
// for the fetch itself -- that is unchanged, and unrelated.                //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_sys
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10,
	parameter         RESET_VECTORS = 0   // see ap040_pipe_cpu.v
)
(
	input  clk,
	input  nreset,
	input  ce,
	input  [2:0] irq_lvl,   // the requested interrupt level, active high; 0 none

	output        mem_req,
	output        mem_write,
	output        mem_instr,
	output  [1:0] mem_size,
	output [31:0] mem_addr,
	output [31:0] mem_wdata,
	output  [2:0] mem_fc,
	input         mem_ack,
	input  [31:0] mem_rdata,

	output        dbg_if_valid,  output [31:0] dbg_if_pc,
	output        dbg_id_valid,  output [31:0] dbg_id_pc,
	output        dbg_eac_valid, output [31:0] dbg_eac_pc,
	output        dbg_eaf_valid, output [31:0] dbg_eaf_pc,
	output        dbg_ex_valid,  output [31:0] dbg_ex_pc,
	output        dbg_wb_valid,  output [31:0] dbg_wb_pc,

	output [31:0] dbg_d0, output [31:0] dbg_d1, output [31:0] dbg_d2, output [31:0] dbg_d3,
	output [31:0] dbg_d4, output [31:0] dbg_d5, output [31:0] dbg_d6, output [31:0] dbg_d7,
	output  [4:0] dbg_ccr,
	output [15:0] dbg_sr,
	output [31:0] dbg_commits
);

wire [31:0] l1_addr_a, l1_addr_b, l1_data_b, l1_q_b;
wire [15:0] l1_rdata_a, l1_rdata_a2;
wire  [1:0] l1_size_b;
wire        l1_req_a, l1_rvalid_a, l1_rd_b, l1_rvalid_b, l1_wren_b, l1_wr_busy;
wire        l1_wr_busy_w;   // the CPU's: see ap040_pipe_membus.v
wire        l1_sup_b, l1_sup_a;
wire        l1_inval_a;
wire        l1_rflt_a, l1_rflt_a_bus, l1_rflt_b, l1_wflt, l1_flt_bus, l1_flt_ma, l1_wr_sync;
wire        l1_idle, l1_quiet, l1_wr_drop, pt_req, pf_req;
wire        cm_req, cm_ic, cm_done, ic_en;
wire  [1:0] cm_scope;
wire [31:0] cm_addr, mmu_itt0, mmu_itt1;
wire        l1_fc_ovr;
wire  [2:0] l1_fc_val;

ap040_pipe_cpu #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS),
	.RESET_VECTORS(RESET_VECTORS)
) u_cpu
(
	.clk (clk), .nreset (nreset), .ce (ce), .irq_lvl (irq_lvl),

	.l1_addr_a (l1_addr_a), .l1_req_a (l1_req_a),
	.l1_rdata_a(l1_rdata_a), .l1_rdata_a2(l1_rdata_a2), .l1_rvalid_a(l1_rvalid_a),

	.l1_addr_b (l1_addr_b), .l1_rd_b (l1_rd_b), .l1_wren_b (l1_wren_b),
	.l1_sup_b  (l1_sup_b), .l1_sup_a (l1_sup_a),
	.l1_size_b (l1_size_b),   .l1_data_b(l1_data_b),
	.l1_wr_busy(l1_wr_busy_w), .l1_q_b (l1_q_b), .l1_rvalid_b(l1_rvalid_b),
	.l1_inval_a(l1_inval_a),
	.l1_rflt_a(l1_rflt_a), .l1_rflt_a_bus(l1_rflt_a_bus), .l1_rflt_b(l1_rflt_b), .l1_wflt(l1_wflt), .l1_flt_bus(l1_flt_bus), .l1_flt_ma(l1_flt_ma),
	.l1_wr_sync(l1_wr_sync),
	// no MMU on this top: PTEST answers MMUSR 0, PFLUSH at once
	.l1_idle (l1_idle), .l1_quiet (l1_quiet), .l1_wr_drop (l1_wr_drop), .pt_req (pt_req), .pt_write (), .pt_addr (), .pt_fc (),
	.pt_done (pt_req), .pt_mmusr (32'd0), .pf_req (pf_req), .pf_mode (), .pf_addr (), .pf_fc (),
	.pf_done (pf_req),
	// the instruction cache is the IMU's: CINV/CPUSH, IE, and the ITTs for
	// its caching modes
	.cm_req (cm_req), .cm_ic (cm_ic), .cm_dc (), .cm_push (), .cm_scope (cm_scope), .cm_addr (cm_addr),
	.cm_done (cm_done), .ic_en (ic_en), .pw_pend (1'b0), .pw_ssw (16'h0), .pw_fa (32'h0), .pw_wb1s (8'h0), .pw_pd (128'h0), .pw_exc (1'b0), .pw_ack (), .mmu_itt0 (mmu_itt0), .mmu_itt1 (mmu_itt1),
	// no data cache on this top: port B goes straight to the bus controller
	.dc_en (), .l1_nalloc_b (), .l1_m16_b (), .l1_lock_b (),
	.l1_fc_ovr (l1_fc_ovr), .l1_fc_val (l1_fc_val),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),
	.dbg_d0(dbg_d0), .dbg_d1(dbg_d1), .dbg_d2(dbg_d2), .dbg_d3(dbg_d3),
	.dbg_d4(dbg_d4), .dbg_d5(dbg_d5), .dbg_d6(dbg_d6), .dbg_d7(dbg_d7),
	.dbg_ccr(dbg_ccr), .dbg_sr(dbg_sr), .dbg_commits(dbg_commits)
);

// The instruction memory unit: port A's prefetch window and the instruction
// cache. Nothing to translate on this top.
wire        f_req, f_sup, f_free, f_ack, f_flt, f_flt_bus, f_w_accept;
wire [31:0] f_addr;
wire [29:0] f_w_sla;
ap040_pipe_imu u_imu
(
	.clk (clk), .nreset (nreset),
	.address_a(l1_addr_a), .en_a (l1_req_a),
	.q_a      (l1_rdata_a), .q_a2 (l1_rdata_a2), .rvalid_a(l1_rvalid_a),
	.rflt_a   (l1_rflt_a), .rflt_a_bus (l1_rflt_a_bus),
	.sup      (l1_sup_a), .pf_inval (l1_inval_a), .quiesce (l1_quiet),
	.ic_en    (ic_en), .itt0 (mmu_itt0), .itt1 (mmu_itt1),
	.cm_req   (cm_req), .cm_ic (cm_ic), .cm_scope (cm_scope), .cm_addr (cm_addr), .cm_done (cm_done),
	.sn_req   (1'b0), .sn_addr (32'd0),     // no other master
	.pf_xlat  (1'b0), .x_req (), .x_addr (), .x_sup (), .x_pass (1'b0), .x_flt (1'b0), .x_pa (32'd0),
	.x_cm     (2'b00),
	.pk_addr  (), .pk_sup (), .pk_hit (1'b0), .pk_pa (32'd0), .pk_cm (2'b00),
	.f_req    (f_req), .f_addr (f_addr), .f_sup (f_sup), .f_free (f_free),
	.f_ack    (f_ack), .f_rdata (mem_rdata), .f_flt (f_flt), .f_flt_bus (f_flt_bus),
	.w_accept (f_w_accept), .w_sla (f_w_sla)
);

ap040_pipe_membus u_bus
(
	.clk (clk), .nreset (nreset),

	.f_req    (f_req), .f_addr (f_addr), .f_sup (f_sup), .f_free (f_free),
	.f_ack    (f_ack), .f_flt (f_flt), .f_flt_bus (f_flt_bus),
	.w_accept (f_w_accept), .w_sla (f_w_sla),

	.address_b(l1_addr_b), .la_b(l1_addr_b), .data_b(l1_data_b), .wren_b(l1_wren_b),
	.size_b   (l1_size_b),   .rd_b  (l1_rd_b),
	.wr_busy  (l1_wr_busy), .wr_busy_w(l1_wr_busy_w), .q_b  (l1_q_b), .rvalid_b(l1_rvalid_b),

	.sup_b    (l1_sup_b),
	.fc_ovr   (l1_fc_ovr), .fc_ovr_val(l1_fc_val),

	.mem_req  (mem_req),  .mem_write(mem_write), .mem_instr(mem_instr),
	.mem_size (mem_size), .mem_addr (mem_addr),  .mem_wdata(mem_wdata),
	.mem_fc   (mem_fc),   .mem_ack  (mem_ack),   .mem_rdata(mem_rdata),
	// No MMU on this top: nothing is refused, and a request has passed as
	// soon as it is on the port.
	.mem_flt  (1'b0), .mem_flt_bus (1'b0), .mem_pass (mem_req), .wr_sync (l1_wr_sync),
	.rflt_b   (l1_rflt_b), .wflt (l1_wflt), .flt_bus (l1_flt_bus),
	.idle     (l1_idle), .wr_berr (), .wr_drop (l1_wr_drop),
	// no MMU behind this memory: nothing is translated, nothing crosses
	.xlat_e   (1'b0), .xlat_p (1'b0),
	.pb_req   (), .pb_addr (), .pb_fc (), .pb_done (1'b0), .pb_mmusr (32'd0), .flt_ma (l1_flt_ma),
	// the CPU's reads come straight in
	.rx (1'b0), .rx_addr (32'd0), .rx_size (2'd0), .rx_fc (3'd0)
);

endmodule
