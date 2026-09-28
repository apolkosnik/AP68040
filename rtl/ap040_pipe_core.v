//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core                                //
//                                                                          //
// ap040_pipe_core.v - the CPU paired with ap040_pipe_l1.v's array          //
//                                                                          //
// This was the whole core until milestone 81; the pipeline now lives in    //
// ap040_pipe_cpu.v and this file is the pairing every tb_ap040_pipe_*.v    //
// uses: one memory, no bus, preloaded through dut.u_l1.mem[]. The CPU's    //
// stages and register file moved one level down with it, so a bench        //
// reaching into them says dut.u_cpu.<...>.                                 //
//                                                                          //
// ap040_pipe_sys.v is the other pairing: the same CPU behind               //
// ap040_pipe_membus.v and a real memory port.                              //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_core
#(
	parameter [31:0] PC_RESET   = 32'h0000_0400,
	parameter         PROG_WORDS = 10,
	parameter         L1_AW      = 12,  // ap040_pipe_l1.v size: 2**L1_AW words
	parameter         RESET_VECTORS = 0 // see ap040_pipe_cpu.v: SSP and PC from $0/$4
)
(
	input  clk,
	input  nreset,
	input  ce,
	input  [2:0] irq_lvl,   // the requested interrupt level, active high; 0 none

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
wire        pt_req, pf_req, cm_req;

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
	.l1_sup_b  (), .l1_sup_a (),   // the array has no function codes
	.l1_size_b (l1_size_b),   .l1_data_b(l1_data_b),
	.l1_wr_busy(l1_wr_busy), .l1_q_b (l1_q_b), .l1_rvalid_b(l1_rvalid_b),
	// the L1 array never faults an access
	.l1_rflt_a (1'b0), .l1_rflt_a_bus (1'b0), .l1_rflt_b (1'b0), .l1_wflt (1'b0), .l1_flt_bus (1'b0), .l1_flt_ma (1'b0),
	// ...and has no MMU: PTEST answers MMUSR 0, PFLUSH at once
	.l1_idle (1'b1), .l1_quiet (), .l1_wr_drop (), .pt_req (pt_req), .pt_write (), .pt_addr (), .pt_fc (),
	.pt_done (pt_req), .pt_mmusr (32'd0), .pf_req (pf_req), .pf_mode (), .pf_addr (), .pf_fc (),
	.pf_done (pf_req),
	// ...nor caches: CINV/CPUSH at once
	.cm_req (cm_req), .cm_ic (), .cm_dc (), .cm_push (), .cm_scope (), .cm_addr (), .cm_done (cm_req),
	.pw_pend (1'b0), .pw_ssw (16'h0), .pw_fa (32'h0), .pw_wb1s (8'h0), .pw_pd (128'h0), .pw_exc (1'b0), .pw_ack (),
	.ic_en (), .dc_en (), .l1_nalloc_b (), .l1_m16_b (), .l1_lock_b (),
	.dbg_if_valid  (dbg_if_valid),
	.dbg_if_pc     (dbg_if_pc),
	.dbg_id_valid  (dbg_id_valid),
	.dbg_id_pc     (dbg_id_pc),
	.dbg_eac_valid (dbg_eac_valid),
	.dbg_eac_pc    (dbg_eac_pc),
	.dbg_eaf_valid (dbg_eaf_valid),
	.dbg_eaf_pc    (dbg_eaf_pc),
	.dbg_ex_valid  (dbg_ex_valid),
	.dbg_ex_pc     (dbg_ex_pc),
	.dbg_wb_valid  (dbg_wb_valid),
	.dbg_wb_pc     (dbg_wb_pc),
	.dbg_d0        (dbg_d0),
	.dbg_d1        (dbg_d1),
	.dbg_d2        (dbg_d2),
	.dbg_d3        (dbg_d3),
	.dbg_d4        (dbg_d4),
	.dbg_d5        (dbg_d5),
	.dbg_d6        (dbg_d6),
	.dbg_d7        (dbg_d7),
	.dbg_ccr       (dbg_ccr),
	.dbg_sr        (dbg_sr),
	.dbg_commits   (dbg_commits)
);

ap040_pipe_l1 #(
	.AW(L1_AW),
	.DW(16),
	.PC_RESET(PC_RESET)
) u_l1
(
	.clock     (clk),
	.nreset    (nreset),

	// Port A is read-only from the CPU; its enable must match the fetcher's
	// own advance exactly (see ap040_pipe_l1.v's header).
	.address_a (l1_addr_a),
	.data_a    (16'h0),
	.wren_a    (1'b0),
	.en_a      (l1_req_a),
	.q_a       (l1_rdata_a),
	.q_a2      (l1_rdata_a2),
	.rvalid_a  (l1_rvalid_a),

	.address_b (l1_addr_b),
	.data_b    (l1_data_b),
	.wren_b    (l1_wren_b),
	.size_b    (l1_size_b),
	.rd_b      (l1_rd_b),
	.wr_busy   (l1_wr_busy),
	.q_b       (l1_q_b),
	.rvalid_b  (l1_rvalid_b)
);

endmodule
