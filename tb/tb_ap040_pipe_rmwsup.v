//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 93: whose write is //
// it)                                                                      //
//                                                                          //
// tb_ap040_pipe_rmwsup.v - EX's store, and a younger exception's privilege //
//                                                                          //
// Two stages can want the memory's write port in the same cycle. EX wins:  //
// it holds the older instruction, and a read-modify-write's store issues   //
// from there because the value does not exist until then. The address, the //
// size and the data all switch to EX when it takes the port.               //
//                                                                          //
// The privilege did not. It came straight from EA-fetch, where a YOUNGER   //
// instruction sits -- and if that one is taking an exception, EA-fetch is  //
// forcing supervisor for the frame it is about to push. The older store    //
// then leaves the core as a supervisor access although the instruction     //
// that made it was running in user mode. On an MMU that separates the two  //
// spaces, that store lands somewhere the program had no right to.          //
//                                                                          //
// ap040_pipe_cpu.v is driven directly here, with the memory written in the //
// bench, for one reason: the window only opens when EX's store has to      //
// WAIT. Neither memory in this repository can make it wait. The array and  //
// the bus wrapper both drain the write buffer before answering the         //
// read-modify-write's own load, so by the time EX offers its store the     //
// port is always free and the store is accepted in the cycle it appears --  //
// one cycle before the exception behind it has decided anything. The same  //
// unreachability is recorded against milestone 48's rmw_wait. Here the     //
// memory holds wr_busy for three cycles after every write, which is what   //
// a real cache with a deeper queue does, and the window opens.             //
//                                                                          //
//   ISP = $1000 ; A0 = $0800 ; drop to user mode                           //
//   ADDQ.L #1,(A0)   a user-mode read-modify-write, storing from EX        //
//   TRAP #0          immediately behind it, forcing supervisor             //
//                                                                          //
// Every write the core emits is recorded with the privilege it carried.    //
// The one to $0800 must be a user access; the frame's beats must be        //
// supervisor ones, so a fix that stopped forcing supervisor at all would   //
// fail here rather than pass.                                              //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_rmwsup;

localparam PROG_WORDS      = 24;
localparam [31:0] PC_RESET = 32'h0000_0400;
localparam integer MEM_WORDS = 4096;      // 8 KB, byte addressed from the core

reg clk = 0;
reg nreset = 0;
always #5 clk = ~clk;

wire [31:0] l1_addr_a;
wire        l1_req_a;
reg  [15:0] l1_rdata_a = 16'h4E71;
reg  [15:0] l1_rdata_a2 = 16'h4E71;   // the word after it (phase 8)
reg         l1_rvalid_a = 1'b0;

wire [31:0] l1_addr_b, l1_data_b;
wire        l1_rd_b, l1_wren_b, l1_sup_b, l1_sup_a;
wire  [1:0] l1_size_b;
reg         l1_wr_busy = 1'b0;
reg  [31:0] l1_q_b = 32'd0;
reg         l1_rvalid_b = 1'b0;

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;
wire [15:0] dbg_sr;
wire [31:0] dbg_commits;

ap040_pipe_cpu #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut (
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk(clk), .nreset(nreset), .ce(1'b1),
	.l1_addr_a(l1_addr_a), .l1_req_a(l1_req_a),
	.l1_rdata_a(l1_rdata_a), .l1_rdata_a2(l1_rdata_a2), .l1_rvalid_a(l1_rvalid_a),
	.l1_addr_b(l1_addr_b), .l1_rd_b(l1_rd_b), .l1_wren_b(l1_wren_b),
	.l1_sup_b(l1_sup_b), .l1_sup_a(l1_sup_a),
	.l1_size_b(l1_size_b), .l1_data_b(l1_data_b),
	.l1_wr_busy(l1_wr_busy), .l1_q_b(l1_q_b), .l1_rvalid_b(l1_rvalid_b),
	.l1_inval_a(), .l1_fc_ovr(), .l1_fc_val(),
	// no MMU and no bus errors behind this memory: nothing faults, the
	// memory side is idle whenever PTEST would ask, and nothing answers one
	.l1_rflt_a(1'b0), .l1_rflt_a_bus(1'b0), .l1_rflt_b(1'b0), .l1_wflt(1'b0), .l1_flt_bus(1'b0), .l1_flt_ma(1'b0),
	.l1_wr_sync(), .l1_idle(1'b1), .l1_quiet(), .l1_wr_drop(),
	.mmu_tc(), .mmu_urp(), .mmu_srp(), .mmu_itt0(), .mmu_itt1(), .mmu_dtt0(), .mmu_dtt1(),
	.pt_req(), .pt_write(), .pt_addr(), .pt_fc(), .pt_done(1'b0), .pt_mmusr(32'd0),
	.pf_req(), .pf_mode(), .pf_addr(), .pf_fc(), .pf_done(1'b0),
	.cm_req(), .cm_ic(), .cm_dc(), .cm_push(), .cm_scope(), .cm_addr(), .cm_done(1'b0), .ic_en(),
	.pw_pend (1'b0), .pw_ssw (16'h0), .pw_fa (32'h0), .pw_wb1s (8'h0), .pw_pd (128'h0), .pw_exc (1'b0), .pw_ack (),
	.dc_en(), .l1_nalloc_b(), .l1_m16_b(), .l1_lock_b(),

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

//--------------------------------------------------------------------------//
// The memory. Word storage, big endian, byte addressed, and a write port   //
// that stays busy for three cycles after it accepts one.                   //
//--------------------------------------------------------------------------//

reg [15:0] mem [0:MEM_WORDS-1];
integer i;

wire [31:0] ia = (l1_addr_a - PC_RESET) >> 1;
wire [31:0] ib = (l1_addr_b - PC_RESET) >> 1;

reg  [2:0] wbusy_cnt;

integer wr_count, rmw_writes, rmw_not_user, frame_writes, frame_not_super;

always @(posedge clk) begin
	if (!nreset) begin
		l1_rvalid_a <= 1'b0; l1_rvalid_b <= 1'b0;
		l1_wr_busy  <= 1'b0; wbusy_cnt   <= 3'd0;
	end else begin
		// ---- port A: a fetch answers the cycle after it is asked for, and the
		// word and its valid HOLD until the next request -- ap040_pipe_l1.v's
		// contract (milestone 80), which the fetch stage relies on while decode
		// is stalled. This model pulsed the valid for one cycle, and passed
		// only while decode never happened to stall on a returning word; the
		// interrupt arm's bubble behind MOVE to SR (2026-09-24) moved a stall
		// onto exactly that cycle, and TRAP #0's opcode was dropped.
		// Since phase 8 the answer carries the word after it as well, which
		// the fetch uses when the address is a longword's first half.
		if (l1_req_a) begin
			l1_rvalid_a <= 1'b1; l1_rdata_a <= mem[ia[11:0]]; l1_rdata_a2 <= mem[ia[11:0] + 12'd1];
		end

		// ---- port B reads, Long only in this program; held the same way
		if (l1_rd_b) begin l1_rvalid_b <= 1'b1; l1_q_b <= {mem[ib[11:0]], mem[ib[11:0] + 1]}; end

		// ---- port B writes, and the backpressure that opens the window
		if (l1_wr_busy) begin
			if (wbusy_cnt == 3'd0) l1_wr_busy <= 1'b0;
			else                   wbusy_cnt  <= wbusy_cnt - 3'd1;
		end else if (l1_wren_b) begin
			l1_wr_busy <= 1'b1;
			wbusy_cnt  <= 3'd6;
			case (l1_size_b)
			`AP040_SZ_L: begin
				mem[ib[11:0]]     <= l1_data_b[31:16];
				mem[ib[11:0] + 1] <= l1_data_b[15:0];
			end
			`AP040_SZ_W: mem[ib[11:0]] <= l1_data_b[15:0];
			default:     mem[ib[11:0]][7:0] <= l1_data_b[7:0];
			endcase
			// Every write, with the privilege it left the core carrying.
			wr_count = wr_count + 1;
			if (l1_addr_b == 32'h0000_0800) begin
				rmw_writes = rmw_writes + 1;
				if (l1_sup_b) rmw_not_user = rmw_not_user + 1;
			end else if (l1_addr_b == 32'h0000_0810) begin
				// The filler. A user-mode store, and it must say so.
				rmw_writes = rmw_writes + 1;
				if (l1_sup_b) rmw_not_user = rmw_not_user + 1;
			end else begin
				frame_writes = frame_writes + 1;
				if (!l1_sup_b) frame_not_super = frame_not_super + 1;
			end
		end
	end
end

integer errors = 0;

initial begin
	for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = `AP040_OP_NOP;
	wr_count = 0; rmw_writes = 0; rmw_not_user = 0;
	frame_writes = 0; frame_not_super = 0;

	// $0400
	mem[  0] = 16'h203C;  mem[  1] = 16'h0000;  mem[  2] = 16'h1000;
	mem[  3] = 16'h4E7B;  mem[  4] = 16'h0804;   // MOVEC D0,ISP
	mem[  5] = 16'h207C;  mem[  6] = 16'h0000;  mem[  7] = 16'h0800;
	mem[  8] = 16'h227C;  mem[  9] = 16'h0000;  mem[ 10] = 16'h0810;
	mem[ 11] = 16'h7407;                         // MOVEQ #7,D2
	mem[ 12] = 16'h7200;                         // MOVEQ #0,D1
	mem[ 13] = 16'h46C1;                         // MOVE D1,SR -> user
	mem[ 14] = 16'h2282;                         // MOVE.L D2,(A1)  fills the port
	mem[ 15] = 16'h5290;                         // ADDQ.L #1,(A0)
	mem[ 16] = 16'h4E40;                         // TRAP #0
	mem[ 17] = 16'h7866;                         // MOVEQ #$66,D4 (poison)
	mem[ 18] = 16'h4E71;                         // NOP

	// The read-modify-write's target, $0800, and the filler store's, $0810.
	mem[512] = 16'h0000;  mem[513] = 16'h0041;
	mem[520] = 16'h0000;  mem[521] = 16'h0000;

	// TRAP #0 handler at $0900, and vector 32 at byte $80.
	mem[640] = 16'h762A;  mem[641] = 16'h4E71;   // MOVEQ #$2A,D3 ; NOP
	// Byte $80 is PC_RESET-relative index ($80 - $400) >> 1, wrapped.
	mem[(16'h0080 - 16'h0400) >> 1] = 16'h0000;
	mem[((16'h0080 - 16'h0400) >> 1) + 1] = 16'h0900;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 400) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d3 !== 32'h0000_002A) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 0000002a (the TRAP handler must run)", dbg_d3);
	end
	if (dbg_d4 !== 32'h0000_0000) begin
		errors = errors + 1;
		$display("FAIL: D4 = %h, expected 00000000 (the instruction after the TRAP must not run)", dbg_d4);
	end
	if ({mem[512], mem[513]} !== 32'h0000_0042) begin
		errors = errors + 1;
		$display("FAIL: [$0800] = %h%h, expected 00000042 (ADDQ.L #1 on 00000041)", mem[512], mem[513]);
	end
	if (rmw_writes !== 2) begin
		errors = errors + 1;
		$display("FAIL: %0d user-mode data writes, expected 2 (the filler and the read-modify-write's own store)", rmw_writes);
	end
	if (rmw_not_user !== 0) begin
		errors = errors + 1;
		$display("FAIL: the read-modify-write's store to $0800 left the core with l1_sup_b SET. It belongs to a user-mode instruction; the exception behind it forces supervisor for its own frame, and EX took the port for address, size and data but not for privilege.");
	end
	if (frame_writes !== 2) begin
		errors = errors + 1;
		$display("FAIL: %0d frame writes, expected 2 (a format $0 frame is two beats)", frame_writes);
	end
	if (frame_not_super !== 0) begin
		errors = errors + 1;
		$display("FAIL: %0d exception frame beats left the core WITHOUT supervisor privilege", frame_not_super);
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
