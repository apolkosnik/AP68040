//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25, caches stage A)   //
//                                                                          //
// tb_ap040_pipe_dmuport.v - the data memory unit and the MMU's ports       //
//                                                                          //
// ap040_pipe_dmu.v, ap040_pipe_mmu.v, ap040_pipe_imu.v and                 //
// ap040_pipe_membus.v as the bus16 top wires them, driven directly at the  //
// CPU's two ports -- the events                                            //
// under test are coincidences a program reaches only by luck: a write      //
// presented while a read's table search is under way, and a fetch refused //
// by its search. The memory behind the bus controller and the table        //
// walker's own port are one array here, as on the card, each with its own //
// latency. Translation on (TC.E, 4K pages); root $4000, pointer table      //
// $4200, page table $4400, pages identity-mapped except page $A, invalid.  //
//                                                                          //
//   1. A read of the invalid page $A000 searches the tables; while the     //
//      search runs, a tentative write to page 5 is presented. The MMU's    //
//      data port belongs to the read until its search ends: the read gets //
//      the fault (not MA, not a bus error), the write is not refused, is   //
//      accepted, and lands. Taken over by the write, the port gave the    //
//      search's fault to the write. And after the read's fault the port    //
//      must be left down a cycle for the walker to let go of it (W_DROP):  //
//      raised at once for the write, it never was, and nothing moved.      //
//   2. Two writes on a slow memory: the first on the bus, the second      //
//      accepted and waiting in the DMU for the bus controller to take it   //
//      (in its write slot, or since caches stage E its store buffer); then //
//      a read of the second's longword. The read must wait for that write: //
//      sent at once, it reached the bus controller first and read the     //
//      memory as it was.                                                   //
//   3. A fetch of the invalid page: $4AFC with the fault, not a bus error; //
//      then a fetch of page 5 returns its instruction words.               //
//   4. A write accepted on a slow memory, then a read of a page the ATC     //
//      does not hold: its search waits for the write to land. Throughout,  //
//      no table-walker access starts while a write the memory side has     //
//      committed to has not reached the memory.                           //
//   5. A read goes on the bus with its own function code: a user read,    //
//      and a supervisor's MOVES from user data, both FC 1; a MOVES from    //
//      user program space as user data, FC 1 (MC68040UM 3.2), translated   //
//      and -- translation off -- straight through, read and written.       //
//   6. A fetch held while another read is on the bus is sent later, and    //
//      may then go through the MMU's peek at its latest instruction        //
//      translation. When that translation is a page's nonresident entry    //
//      (two refused fetches of page $A), the peek must not cover it: the   //
//      held fetch of page $A is refused, not read from the entry's empty   //
//      frame. The read it waits behind is through an instruction TTR, so   //
//      the peek's translation is still page $A's.                          //
//   7. A write into the prefetch window, with the code mapped elsewhere:   //
//      page 8 is physical $5000. A fetch of $8100 lets the window read     //
//      ahead; a write to logical $8108 -- physical $5108 -- must empty it, //
//      and the fetch of $8108 return the written words. The window is     //
//      logical; the bus controller sees the write's physical address, and  //
//      matched that against it the window kept the old words.             //
//   8. A write whose snoop empties the window in a cycle with no read in   //
//      flight. The next longword was pf_base + pf_cnt before the snoop and //
//      is pf_base after it; a read for the first must not start in that    //
//      cycle, or it lands as the second. The bus controller's own read     //
//      cannot go then (the write is waiting there), but a translation      //
//      could, and did. The write's arrival is swept across the window's    //
//      refill on a slow memory, each time in a fresh 32 bytes of page 8,   //
//      and every longword then fetched must be memory's; the sweep must    //
//      reach the cycle it is after (snoop_idle). Twice: the window's reads //
//      to the bus controller (IE clear), then to the instruction cache,   //
//      which takes a read whatever the bus is doing (IE set; the written  //
//      longword itself is then the cache's to keep, MC68040UM 4.5).       //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_dmuport;

reg clk = 1'b0;
always #5 clk = ~clk;
reg nreset = 1'b0;

integer errors = 0;
task fail;
	input [8*120-1:0] msg;
	begin
		errors = errors + 1;
		$display("FAIL: %0s", msg);
	end
endtask
task step; begin @(posedge clk); #1; end endtask

//------------------------------------------------------------------- memory
// 64 KB of bytes, big-endian, shared by the bus controller and the walker.
reg [7:0] mem [0:65535];
function [31:0] rd32;
	input [15:0] a;
	begin rd32 = {mem[a], mem[a + 16'd1], mem[a + 16'd2], mem[a + 16'd3]}; end
endfunction
task wr32;
	input [15:0] a;
	input [31:0] d;
	begin
		mem[a] = d[31:24]; mem[a + 16'd1] = d[23:16]; mem[a + 16'd2] = d[15:8]; mem[a + 16'd3] = d[7:0];
	end
endtask

//----------------------------------------------------------- the MMU's state
reg  [31:0] tc = 32'd0;
wire [31:0] urp = 32'h4000, srp = 32'h4000, ttr0 = 32'd0;
reg  [31:0] itt0 = 32'd0;
reg         ic_en = 1'b0;

//------------------------------------------------------------- CPU, port B
reg  [31:0] c_addr = 32'd0, c_wdata = 32'd0;
reg         c_rd = 1'b0, c_wr = 1'b0, c_sup = 1'b1, c_fc_ovr = 1'b0;
reg   [2:0] c_fc_val = 3'd0;
reg   [1:0] c_size = `AP040_SZ_L;
wire [31:0] c_q;
wire        c_rvalid, c_wr_busy_w, c_rflt, c_wflt, c_flt_bus, c_flt_ma, c_idle, wr_pend;
// the CPU's l1_wr_drop: a cycle it runs presenting no write (ce is 1 here)
wire        c_wr_drop = !c_wr;

//------------------------------------------------------------- CPU, port A
reg  [31:0] a_addr = 32'd0;
reg         en_a = 1'b0;
wire [15:0] q_a, q_a2;
wire        rvalid_a, rflt_a, rflt_a_bus;

//--------------------------------------------------------------- the units
wire        d_req, d_write, d_acc, d_sup, d_pass, d_flt;
wire [31:0] d_addr, d_pa;
wire        i_req, i_sup, i_pass, i_flt, ip_sup, ip_hit;
wire [31:0] i_addr, i_pa, ip_addr, ip_pa;
wire  [1:0] i_cm, ip_cm, d_cm;
wire [31:0] bb_addr, bb_la, bb_wdata, bb_q, bb_rx_addr;
wire  [1:0] bb_size, bb_rx_size;
wire        bb_rd, bb_wr, bb_sup, bb_fc_ovr, bb_rvalid, bb_wr_busy_w, bb_rflt, bb_flt_bus, bb_flt_ma, bb_idle, bb_rx;
wire  [2:0] bb_fc_val, bb_rx_fc;
wire        mem_req, mem_write, mem_instr;
// a write-back's bus error (caches stage R)
wire        bb_wr_berr, pw_pend, pw_exc;
wire [15:0] pw_ssw;
wire [31:0] pw_fa;
wire  [7:0] pw_wb1s;
wire [127:0] pw_pd;
wire  [1:0] mem_size;
wire [31:0] mem_addr, mem_wdata;
wire  [2:0] mem_fc;
reg         mem_ack = 1'b0;
reg  [31:0] mem_rdata = 32'd0;
wire        walker_req, walker_we;
wire        mw_req, mw_we, mw_ack, mw_berr;
wire [31:0] mw_addr, mw_wdat, mw_data;
wire [31:0] walker_addr, walker_wdat;
reg         walker_ack = 1'b0;
reg  [31:0] walker_data = 32'd0;

ap040_pipe_dmu u_dmu
(
	.clk (clk), .nreset (nreset),
	.xlat (tc[15]), .tc_e (tc[15]), .tc_p (tc[14]),
	.c_addr (c_addr), .c_rd (c_rd), .c_wr (c_wr), .c_size (c_size), .c_wdata (c_wdata),
	.c_sup (c_sup), .c_fc_ovr (c_fc_ovr), .c_fc_val (c_fc_val), .c_wr_drop (c_wr_drop),
	.c_nalloc (1'b0), .c_m16 (1'b0), .c_lock (1'b0),
	.dc_en (1'b0), .dtt0 (ttr0), .dtt1 (ttr0),
	.cm_req (1'b0), .cm_dc (1'b0), .cm_push_in (1'b0), .cm_scope (2'd0), .cm_addr (32'd0), .cm_done (),
	.sn_req (1'b0), .sn_addr (32'd0),
	.wk_req (mw_req), .wk_we (mw_we), .wk_addr (mw_addr), .wk_wdat (mw_wdat),
	.wk_ack (mw_ack), .wk_data (mw_data), .wk_berr (mw_berr),
	.walker_req (walker_req), .walker_we (walker_we), .walker_addr (walker_addr),
	.walker_wdat (walker_wdat), .walker_ack (walker_ack), .walker_data (walker_data),
	.walker_berr (1'b0),
	.c_q (c_q), .c_rvalid (c_rvalid), .c_wr_busy_w (c_wr_busy_w), .c_rflt (c_rflt),
	.c_wflt (c_wflt), .c_flt_bus (c_flt_bus), .c_flt_ma (c_flt_ma), .c_idle (c_idle),
	.wr_pend (wr_pend),
	.d_req (d_req), .d_write (d_write), .d_acc (d_acc), .d_addr (d_addr), .d_sup (d_sup),
	.d_pass (d_pass), .d_flt (d_flt), .d_pa (d_pa), .d_cm (d_cm),
	.m_addr (bb_addr), .m_la (bb_la), .m_rd (bb_rd), .m_wr (bb_wr), .m_size (bb_size), .m_wdata (bb_wdata),
	.m_sup (bb_sup), .m_fc_ovr (bb_fc_ovr), .m_fc_val (bb_fc_val),
	.m_rx (bb_rx), .m_rx_addr (bb_rx_addr), .m_rx_size (bb_rx_size), .m_rx_fc (bb_rx_fc),
	.m_q (bb_q), .m_rvalid (bb_rvalid), .m_wr_busy_w (bb_wr_busy_w), .m_rflt (bb_rflt),
	.m_flt_bus (bb_flt_bus), .m_flt_ma (bb_flt_ma), .m_idle (bb_idle), .m_wberr (bb_wr_berr),
	.pw_pend (pw_pend), .pw_ssw (pw_ssw), .pw_fa (pw_fa), .pw_wb1s (pw_wb1s), .pw_pd (pw_pd),
	.pw_exc (pw_exc), .pw_ack (1'b0)
);

ap040_pipe_mmu u_mmu
(
	.clk (clk), .nreset (nreset),
	.tc (tc), .urp (urp), .srp (srp), .itt0 (itt0), .itt1 (ttr0), .dtt0 (ttr0), .dtt1 (ttr0),
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0),
	.i_req (i_req), .i_addr (i_addr), .i_sup (i_sup), .i_pass (i_pass), .i_flt (i_flt), .i_pa (i_pa), .i_cm (i_cm),
	.ip_addr (ip_addr), .ip_sup (ip_sup), .ip_hit (ip_hit), .ip_pa (ip_pa), .ip_cm (ip_cm),
	.d_req (d_req), .d_write (d_write), .d_acc (d_acc), .d_addr (d_addr), .d_sup (d_sup),
	.d_pass (d_pass), .d_flt (d_flt), .d_pa (d_pa), .d_cm (d_cm),
	.pt_req (1'b0), .pt_write (1'b0), .pt_access (1'b0), .pt_addr (32'd0), .pt_fc (3'd0),
	.pt_done (), .pt_mmusr (),
	.pf_req (1'b0), .pf_mode (2'd0), .pf_addr (32'd0), .pf_fc (3'd0), .pf_done (),
	.walk_hold (wr_pend),
	.walker_req (mw_req), .walker_we (mw_we), .walker_addr (mw_addr),
	.walker_wdat (mw_wdat), .walker_ack (mw_ack), .walker_data (mw_data),
	.walker_berr (mw_berr)
);

wire        ib_req, ib_sup, ib_free, ib_ack, ib_flt, ib_flt_bus, ib_w_accept;
wire [31:0] ib_addr;
wire [29:0] ib_w_sla;
ap040_pipe_imu u_imu
(
	.clk (clk), .nreset (nreset),
	.address_a (a_addr), .en_a (en_a), .q_a (q_a), .q_a2 (q_a2), .rvalid_a (rvalid_a),
	.rflt_a (rflt_a), .rflt_a_bus (rflt_a_bus),
	.sup (1'b1), .pf_inval (1'b0), .quiesce (1'b0),
	.ic_en (ic_en), .itt0 (itt0), .itt1 (ttr0),
	.cm_req (1'b0), .cm_ic (1'b0), .cm_scope (2'd0), .cm_addr (32'd0), .cm_done (),
	.sn_req (1'b0), .sn_addr (32'd0),
	.pf_xlat (tc[15]), .x_req (i_req), .x_addr (i_addr), .x_sup (i_sup), .x_pass (i_pass), .x_flt (i_flt), .x_pa (i_pa),
	.x_cm (i_cm),
	.pk_addr (ip_addr), .pk_sup (ip_sup), .pk_hit (ip_hit), .pk_pa (ip_pa), .pk_cm (ip_cm),
	.f_req (ib_req), .f_addr (ib_addr), .f_sup (ib_sup), .f_free (ib_free),
	.f_ack (ib_ack), .f_rdata (mem_rdata), .f_flt (ib_flt), .f_flt_bus (ib_flt_bus),
	.w_accept (ib_w_accept), .w_sla (ib_w_sla)
);

ap040_pipe_membus u_bus
(
	.clk (clk), .nreset (nreset),
	.f_req (ib_req), .f_addr (ib_addr), .f_sup (ib_sup), .f_free (ib_free),
	.f_ack (ib_ack), .f_flt (ib_flt), .f_flt_bus (ib_flt_bus),
	.w_accept (ib_w_accept), .w_sla (ib_w_sla),
	.address_b (bb_addr), .la_b (bb_la), .data_b (bb_wdata), .wren_b (bb_wr), .size_b (bb_size), .rd_b (bb_rd),
	.wr_busy (), .wr_busy_w (bb_wr_busy_w), .q_b (bb_q), .rvalid_b (bb_rvalid),
	.sup_b (bb_sup), .fc_ovr (bb_fc_ovr), .fc_ovr_val (bb_fc_val),
	.mem_req (mem_req), .mem_write (mem_write), .mem_instr (mem_instr), .mem_size (mem_size),
	.mem_addr (mem_addr), .mem_wdata (mem_wdata), .mem_fc (mem_fc), .mem_ack (mem_ack), .mem_rdata (mem_rdata),
	.mem_flt (1'b0), .mem_flt_bus (1'b0), .mem_pass (mem_req), .wr_sync (1'b0),
	.rflt_b (bb_rflt), .wflt (), .idle (bb_idle), .wr_berr (bb_wr_berr), .wr_drop (1'b0),
	.xlat_e (1'b0), .xlat_p (1'b0), .pb_req (), .pb_addr (), .pb_fc (), .pb_done (1'b0), .pb_mmusr (32'd0),
	.flt_ma (bb_flt_ma), .flt_bus (bb_flt_bus),
	.rx (bb_rx), .rx_addr (bb_rx_addr), .rx_size (bb_rx_size), .rx_fc (bb_rx_fc)
);

//------------------------------------------------ the bus controller's memory
// mem_req held until the one-cycle mem_ack; data right-aligned by size.
// Latency mem_lat cycles, set per test (a slow memory keeps a write waiting).
integer mem_lat = 1, mem_cnt = 0;
reg     mem_busy = 1'b0;
always @(posedge clk) begin
	mem_ack <= 1'b0;
	if (!nreset) begin
		mem_busy <= 1'b0;
	end else if (mem_req && !mem_ack) begin
		if (!mem_busy) begin
			mem_busy <= 1'b1; mem_cnt = mem_lat;
		end else if (mem_cnt > 0) mem_cnt = mem_cnt - 1;
		else begin
			mem_busy <= 1'b0;
			mem_ack  <= 1'b1;
			if (mem_write) begin
				case (mem_size)
				`AP040_SZ_B: mem[mem_addr[15:0]] = mem_wdata[7:0];
				`AP040_SZ_W: begin mem[mem_addr[15:0]] = mem_wdata[15:8]; mem[mem_addr[15:0] + 16'd1] = mem_wdata[7:0]; end
				default: wr32(mem_addr[15:0], mem_wdata);
				endcase
			end else
				mem_rdata <= (mem_size == `AP040_SZ_B) ? {24'd0, mem[mem_addr[15:0]]} :
				             (mem_size == `AP040_SZ_W) ? {16'd0, mem[mem_addr[15:0]], mem[mem_addr[15:0] + 16'd1]} :
				             rd32(mem_addr[15:0]);
		end
	end
end

//------------------------------------------------------ the walker's port
// Its own latency: four cycles from a request to its acknowledgement.
reg       wk_q = 1'b0;
integer   wk_cnt = 0;
reg       wk_on = 1'b0;
always @(posedge clk) begin
	walker_ack <= 1'b0;
	wk_q <= walker_req;
	if (!nreset) wk_on <= 1'b0;
	else if (walker_req && !wk_q && !wk_on) begin
		wk_on <= 1'b1; wk_cnt = 4;
	end else if (wk_on) begin
		if (wk_cnt > 0) wk_cnt = wk_cnt - 1;
		else begin
			wk_on      <= 1'b0;
			walker_ack <= 1'b1;
			if (walker_we) wr32(walker_addr[15:0], walker_wdat);
			else           walker_data <= rd32(walker_addr[15:0]);
		end
	end
end

//------------------------------------- 4. the walker never passes a write
// Bytes the memory side has committed to (the DMU's acceptance of a
// tentative write) against bytes the memory has taken.
integer committed = 0;
function integer sz_bytes;
	input [1:0] sz;
	begin sz_bytes = (sz == `AP040_SZ_L) ? 4 : (sz == `AP040_SZ_W) ? 2 : 1; end
endfunction
always @(posedge clk) if (nreset) begin
	if (walker_req && !wk_q && committed != 0)
		fail("a table-walker access started with a committed write not yet in memory");
	if (u_dmu.w_acc) committed = committed + sz_bytes(u_dmu.w_size);
	if (u_dmu.w_thru && !u_dmu.m_wr_busy_w) committed = committed + sz_bytes(c_size);
	if (mem_ack && mem_write) committed = committed - sz_bytes(mem_size);
end

// 8: the snoop cycles in which a stream read could start -- the snoop
// empties a window that was not empty, with nothing in flight and room.
integer snoop_idle = 0;
always @(posedge clk)
	if (nreset && u_imu.w_hits_pf && !u_imu.pf_out && (u_imu.pf_cnt != 3'd0) && (u_imu.pf_cnt < 3'd4))
		snoop_idle = snoop_idle + 1;

// The function code of each data read and write the bus controller starts.
reg       mreq_q = 1'b0;
reg [2:0] last_rd_fc = 3'd0, last_wr_fc = 3'd0;
always @(posedge clk) begin
	mreq_q <= mem_req;
	if (mem_req && !mreq_q && !mem_write && !mem_instr) last_rd_fc <= mem_fc;
	if (mem_req && !mreq_q &&  mem_write)               last_wr_fc <= mem_fc;
end

//----------------------------------------------------------------- driver
// Port B: a read is asked for in one cycle; a write is held until wr_busy
// is low in a cycle it is presented -- the CPU's own rules.
reg [31:0] rd_q;
reg        rd_flt, rd_bus, rd_ma, wr_refused;
task read_start;
	input [31:0] a;
	begin
		c_addr = a; c_size = `AP040_SZ_L; c_rd = 1'b1;
		step;
		c_rd = 1'b0;
	end
endtask
task read_wait;
	integer n;
	begin
		n = 0;
		while (!c_rvalid && n < 400) begin step; n = n + 1; end
		rd_q = c_q; rd_flt = c_rflt; rd_bus = c_flt_bus; rd_ma = c_flt_ma;
		if (n >= 400) fail("a read was never answered");
	end
endtask
task write_hold;   // present until accepted, or refused
	input [31:0] a;
	input [31:0] d;
	integer n;
	begin
		c_addr = a; c_wdata = d; c_size = `AP040_SZ_L; c_wr = 1'b1;
		n = 0; wr_refused = 1'b0;
		#0;
		while (c_wr_busy_w && !c_wflt && n < 400) begin step; n = n + 1; end
		if (c_wflt) wr_refused = 1'b1;
		if (n >= 400) fail("a write was never accepted or refused");
		step;               // the acceptance's cycle: the CPU is looking
		c_wr = 1'b0;
	end
endtask

// Port A: a fetch asked for in one cycle, answered with rvalid_a.
reg [15:0] f_q;
reg        f_flt, f_bus;
task fetch_start;
	input [31:0] a;
	begin
		a_addr = a; en_a = 1'b1;
		step;
		en_a = 1'b0;
	end
endtask
task fetch_wait;
	integer n;
	begin
		n = 0;
		while (!rvalid_a && n < 400) begin step; n = n + 1; end
		f_q = q_a; f_flt = rflt_a; f_bus = rflt_a_bus;
		if (n >= 400) fail("a fetch was never answered");
	end
endtask
task fetch;
	input [31:0] a;
	integer n;
	begin
		a_addr = a; en_a = 1'b1;
		step;
		en_a = 1'b0;
		n = 0;
		while (!rvalid_a && n < 400) begin step; n = n + 1; end
		f_q = q_a; f_flt = rflt_a; f_bus = rflt_a_bus;
		if (n >= 400) fail("a fetch was never answered");
	end
endtask

integer i, d, k, ie;
reg [15:0] lo16, rg;
initial begin
	// tables: every page identity-mapped and resident, page $A invalid
	for (i = 0; i < 65536; i = i + 1) mem[i] = 8'h00;
	wr32(16'h4000, 32'h0000_4203);
	wr32(16'h4200, 32'h0000_4403);
	for (i = 0; i < 16; i = i + 1) wr32(16'h4400 + i * 4, (i << 12) | 3);
	wr32(16'h4428, 32'h0000_0000);
	wr32(16'h4420, 32'h0000_5003);   // page 8 -> physical $5000 (test 7)
	wr32(16'h5000, 32'h0BAD_0BAD);
	wr32(16'h5100, 32'h4E71_4E75);   // NOP, RTS: instruction words on page 5
	repeat (4) step;
	nreset = 1'b1;
	tc = 32'h0000_8000;
	repeat (4) step;

	//------------------------------------------------------------- test 1
	mem_lat = 1;
	read_start(32'h0000_A000);
	repeat (2) step;                     // the read's search is under way
	fork
		read_wait;
		write_hold(32'h0000_5000, 32'h1234_5678);
	join
	if (!rd_flt) fail("1: the read of the invalid page was not refused");
	if (rd_bus)  fail("1: the read's refusal was reported as a bus error");
	if (rd_ma)   fail("1: the read's refusal was reported as MA");
	if (wr_refused) fail("1: the write was refused: the read's search fault went to it");
	repeat (20) step;
	if (rd32(16'h5000) !== 32'h1234_5678) fail("1: the write did not land");

	//------------------------------------------------------------- test 2
	mem_lat = 12;                        // the first write holds the bus
	write_hold(32'h0000_5008, 32'h5555_AAAA);
	write_hold(32'h0000_5004, 32'hAABB_CCDD);
	if ((u_dmu.ws != 3'd5) && u_dmu.sb_empty)
		fail("2: the second write is not waiting in the DMU (the test no longer tests)");
	read_start(32'h0000_5004);
	read_wait;
	if (rd_flt || rd_q !== 32'hAABB_CCDD) begin
		$display("    read %h flt %b", rd_q, rd_flt);
		fail("2: a read passed a write the DMU had accepted");
	end
	mem_lat = 1;
	repeat (40) step;

	//------------------------------------------------------------- test 3
	fetch(32'h0000_A100);
	if (!f_flt)             fail("3: a fetch of the invalid page was not refused");
	if (f_bus)              fail("3: the fetch's refusal was reported as a bus error");
	if (f_q !== 16'h4AFC)   fail("3: a refused fetch did not answer $4AFC");
	fetch(32'h0000_5100);
	if (f_flt || f_q !== 16'h4E71) fail("3: a fetch of page 5 after the refusal did not return its word");

	//------------------------------------------------------------- test 4
	mem_lat = 12;
	wr32(16'h6000, 32'h6666_0000);
	write_hold(32'h0000_5010, 32'h1010_1010);
	read_start(32'h0000_6000);           // page 6: a search
	read_wait;
	if (rd_flt || rd_q !== 32'h6666_0000) fail("4: the read of page 6 did not return its data");
	mem_lat = 1;

	//------------------------------------------------------------- test 5
	repeat (20) step;
	c_sup = 1'b0;
	read_start(32'h0000_5000);
	read_wait;
	if (rd_flt) fail("5: a user read of page 5 was refused");
	if (last_rd_fc !== 3'd1) fail("5: a user data read did not go on the bus as FC 1");
	c_sup = 1'b1; c_fc_ovr = 1'b1; c_fc_val = 3'd1;
	read_start(32'h0000_5004);
	read_wait;
	c_fc_ovr = 1'b0;
	if (rd_flt) fail("5: a MOVES read from user data was refused");
	if (last_rd_fc !== 3'd1) fail("5: a MOVES read from user data did not go on the bus as FC 1");
	c_fc_ovr = 1'b1; c_fc_val = 3'd2;
	read_start(32'h0000_5004);
	read_wait;
	if (rd_flt) fail("5: a MOVES read from user program space was refused");
	if (last_rd_fc !== 3'd1) fail("5: a translated MOVES read from program space did not go on the bus as FC 1");
	tc = 32'd0;                          // untranslated: straight to the bus controller
	repeat (4) step;
	read_start(32'h0000_5004);
	read_wait;
	if (last_rd_fc !== 3'd1) fail("5: an untranslated MOVES read from program space did not go on the bus as FC 1");
	write_hold(32'h0000_5014, 32'h2222_2222);
	repeat (20) step;
	if (last_wr_fc !== 3'd1) fail("5: an untranslated MOVES write to program space did not go on the bus as FC 1");
	c_fc_ovr = 1'b0;
	tc = 32'h0000_8000;
	repeat (4) step;

	//------------------------------------------------------------- test 6
	repeat (20) step;
	fetch(32'h0000_A100);                // refused again, now from the ATC:
	if (!f_flt) fail("6: a second fetch of the invalid page was not refused");
	itt0 = 32'h0100_C000;                // $01xxxxxx transparent, either mode
	mem_lat = 12;
	fetch_start(32'h0100_5100);          // on the bus a while, the ATC untouched
	repeat (4) step;
	fetch_start(32'h0000_A100);          // held behind it, sent after it
	fetch_wait;
	if (!f_flt) fail("6: a held fetch of the invalid page went out through the peek, unrefused");
	if (f_q !== 16'h4AFC) fail("6: the held fetch's refusal did not answer $4AFC");
	itt0 = 32'd0;
	mem_lat = 1;

	//------------------------------------------------------------- test 7
	repeat (20) step;
	wr32(16'h5100, 32'h4E71_4E71);
	wr32(16'h5104, 32'h4E71_4E71);
	wr32(16'h5108, 32'h4E71_4E71);
	wr32(16'h510C, 32'h4E71_4E71);
	fetch(32'h0000_8100);                // the window reads ahead from here
	if (f_flt || f_q !== 16'h4E71) fail("7: a fetch of page 8 did not return its word");
	repeat (30) step;                    // ...through $810C
	write_hold(32'h0000_8108, 32'h5247_5247);
	repeat (20) step;
	if (rd32(16'h5108) !== 32'h5247_5247) fail("7: the write to logical $8108 did not reach physical $5108");
	fetch(32'h0000_8108);
	if (f_flt || f_q !== 16'h5247) begin
		$display("    fetched %h", f_q);
		fail("7: a fetch returned the words a write had replaced (the window kept them)");
	end

	//------------------------------------------------------------- test 8
	repeat (20) step;
	mem_lat = 5;
	for (ie = 0; ie < 2; ie = ie + 1) begin
		ic_en = ie[0];
		snoop_idle = 0;
		for (d = 0; d < 40; d = d + 1) begin
			// 32 bytes at logical $8200 + 32d ($8800 + 32d with IE set),
			// physical $5000 on: the window reads at most 24 bytes past a
			// step's first longword, so the next step's are fresh
			rg = 16'h0200 + ie[0] * 16'h0600 + d * 32;
			for (k = 0; k < 5; k = k + 1) begin
				lo16 = 16'h1000 + d * 16 + k;
				wr32(16'h5000 + rg + k * 4, {4'hC, d[5:0], k[5:0], lo16});
			end
			fetch(32'h0000_8000 + rg);
			if (f_flt || f_q !== {4'hC, d[5:0], 6'd0}) begin
				$display("    ie=%0d d=%0d fetched %h", ie, d, f_q);
				fail("8: the first fetch of a sweep step did not return its word");
			end
			repeat (d) step;
			write_hold(32'h0000_800C + rg, 32'h5A5A_0000 + d);
			repeat (60) step;
			// the written longword is the cache's to keep (4.5): with IE set,
			// only the three the write left
			for (k = 0; k < 4 - ie; k = k + 1) begin
				fetch(32'h0000_8000 + rg + k * 4);
				if (f_flt || f_q !== rd32(16'h5000 + rg + k * 4) >> 16) begin
					$display("    ie=%0d d=%0d k=%0d fetched %h, memory %h", ie, d, k, f_q, rd32(16'h5000 + rg + k * 4));
					fail("8: a fetch after a write into the window returned another longword's word");
				end
			end
		end
		if (snoop_idle == 0) fail("8: no snoop emptied the window with nothing in flight (the test no longer tests)");
		$display("test 8, IE %0d: %0d snoop cycle(s) with the window idle", ie, snoop_idle);
	end
	ic_en = 1'b0;
	mem_lat = 1;

	repeat (20) step;
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("%0d CHECK(S) FAILED", errors);
	$finish;
end

initial begin
	#600000;
	$display("FAIL: timed out");
	$finish;
end

endmodule
