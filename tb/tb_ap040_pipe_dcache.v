//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25, caches stage C)   //
//                                                                          //
// tb_ap040_pipe_dcache.v - the data cache, at the DMU's ports              //
//                                                                          //
// ap040_pipe_dmu.v, ap040_pipe_mmu.v and ap040_pipe_membus.v as the bus16  //
// top wires them, driven at the CPU's port B against a memory whose        //
// latency each test sets and which can answer one read with a bus error,   //
// every transaction it takes logged. What a program cannot see, because    //
// it is timing or order: which longwords a miss reads and in what order,   //
// what a hit costs, which bytes a write puts in a line, how a write meets  //
// a line being read, which way a fill takes, what CINV leaves and how its  //
// done behaves, what an error on a fill's later beat does, and what a      //
// snoop meeting a fill does. Translation off except where a test says.     //
//                                                                          //
//   1. A miss at a line's third longword reads it first, then the fourth,  //
//      the first and the second (MC68040UM 4.1, 4.6.1); the line is valid  //
//      only after the fourth, and the read is answered from the first.     //
//   2. A hit: answered three cycles after the request's, no bus read; every//
//      size at every offset inside a longword, right-aligned.              //
//   3. A write hit puts exactly its bytes in the line (byte enables), and  //
//      memory takes the write; one spanning two longwords, and two lines,  //
//      updates both. A read right after a write sees it.                   //
//   4. A write meeting a line being read: swept from the cycle after the   //
//      read's (port B takes one address a cycle) to after the fill's last  //
//      read, the line ends valid holding the write's bytes.                //
//   5. Not cached: a data TTR's CM 10 (a read that hits invalidates the    //
//      line and goes to the bus), a locked access, MOVES to an alternate   //
//      space; allocating nothing: a miss is one bus read, a hit is served; //
//      a MOVE16 write that hits invalidates the line.                      //
//   6. CINV: a line, a page, everything; done one clock per request, even  //
//      when the request is held after it; the instruction cache's alone    //
//      -- all, a line, a page -- is done at once and leaves the data cache.//
//   7. An error on a fill's later beat: the answered read is not faulted,  //
//      the line is not valid; a read waiting in the line for that longword //
//      is faulted; one waiting for another longword is answered after the  //
//      line is read again from it. And the way the abandoned line took     //
//      still holds the tag of the line that was there before: that line,   //
//      invalidated earlier, must still miss.                               //
//   8. Snoops: a line invalidated, the rest of its set kept. Swept across  //
//      a fill, the snoop writing memory's first longword of the line as it //
//      is raised, including the cycle the fill writes its tag: the line    //
//      ends invalid, or valid holding what memory held after the write --  //
//      never the longword from before it.                                  //
//   9. Replacement: first invalid, else the counter's -- which counts every//
//      read looked up and every write sent, and once more after naming a   //
//      way (4.1).                                                          //
//  10. DE cleared under a fill: a read then waits for it, the fill         //
//      completes, the line valid and whole; the reads after it go to the   //
//      bus alone.                                                          //
//  11. Translated: a page whose descriptor says CM 10 is not cached.       //
// Copyback (caches stage D; a data TTR with CM 01 makes every access so):  //
//  12. A write that misses reads its line and stays in it; one that hits   //
//      stays in it; neither reaches memory; CPUSHL pushes exactly the      //
//      dirty longwords, CINVL drops them. A byte write that misses lands   //
//      in the longword the line read brought: its other bytes memory's.    //
//  13. A dirty line replaced: its longwords are read out before the new    //
//      line's land on them, and pushed after -- swept over memory latency  //
//      0-5 and the longword the fill asks for first; memory ends with the  //
//      dirty data and the cache with the new line.                         //
//  14. CPUSH over a set with three dirty ways and a clean one, two lines   //
//      in each of two pages: CPUSHL its line, CPUSHP its page's (the clean //
//      one invalidated, only the dirty longword written), CPUSHA the rest. //
//  15. The table walker through the cache: a read that hits reads nothing  //
//      from its port; a write that hits updates the line, goes to memory,  //
//      and leaves its longword clean (a CPUSH after it writes nothing).    //
//  16. A copyback write that allocates nothing and misses is one bus write;//
//      an inhibited read that hits a dirty line has its pushes on the bus  //
//      first; a copyback write meeting a line being read waits, then hits. //
//  17. A copyback write whose line read errs -- on the longword written,   //
//      or a later one -- goes to the bus alone, and nothing hangs.         //
//  18. A write-through write to a longword a replaced line is pushing: the //
//      push reaches memory first. Swept across the push.                   //
//  19. DE clear, CPUSHA pushing a line whose last longword is a page       //
//      descriptor, memory's copy stale: an instruction-side translation    //
//      started once the push has begun walks the tables (ap040_pipe_mmu.v's//
//      walker, held by the DMU's wr_pend) and must read the pushed         //
//      descriptor. Swept over the translation's start.                     //
// A write-back's bus error (caches stage R; MC68040UM 8.4.6):              //
//  20. A write the bus controller errs on is held for EA-fetch with its    //
//      frame's fields: SSW, FA, WB1S, WB1D memory-aligned (Table 8-5) for  //
//      each size and offset, MOVES's TT and TM; a crossing write whole; a  //
//      push's line in PD0-PD3, the push engine busy (CPUSH not done) until //
//      its last write is; a MOVE16's four longwords; an exception frame's  //
//      write a double fault, also when it errs behind a fault already held;//
//      a copyback write allocating nothing reported by its logical address;//
//      a second fault dropped while one is held, and taken when it comes   //
//      in the cycle the first is let go.                                   //
//  21. A replaced dirty line is written only after its new line is in      //
//      (4.6.1); if the new line's read errs it goes back to its place,     //
//      valid and dirty, nothing written (4.6.2) -- swept over the longword //
//      asked for and the beat that errs; a snoop on it, swept from before  //
//      the line read to after the replacement, leaves it invalid and       //
//      unwritten whenever it lands.                                        //
// The store buffer (caches stage E):                                       //
//  22. On a slow bus, write-through writes are taken into the buffer       //
//      without waiting for the bus: four behind the one the bus            //
//      controller holds, the next waiting; they reach the bus in order.    //
//      A read that hits is answered while they wait; one that misses, and  //
//      one with DE clear, go to the bus only after them -- a read of a     //
//      longword a buffered write holds returns the write. A bus error on   //
//      a buffered write reports that write; the writes around it land.     //
//      The window's snoop sees each write as the buffer takes it. A miss   //
//      behind buffered writes -- a read's, a copyback write's -- is looked //
//      up once more when they are out, not over and over: the way it       //
//      replaces is the counter's after exactly those lookups.              //
// The platform's cacheable windows (caches stage F, in the MMU):           //
//  23. With the windows on, chip RAM is cached and ROM/IO space is not --  //
//      even through a copyback TTR, whose write then goes to the bus; the  //
//      Zorro II and both Zorro III windows only while enabled, each its    //
//      whole extent and no more; and the instruction side's caching mode,  //
//      and its peek's, the same -- the peek, untranslated, never the mode  //
//      of the entry the ATC last held for the instruction side. A snoop    //
//      on a dirty copyback line invalidates it: its data is lost, memory   //
//      keeps the chipset's (the platform's boundary: nothing here can      //
//      take it).                                                           //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"

module tb_ap040_pipe_dcache;

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
reg  [31:0] dtt1 = 32'd0;
// the platform's cacheable windows (caches stage F; test 23)
reg         win_all = 1'b1, win_z2 = 1'b0, win_z3e0 = 1'b0, win_z3e1 = 1'b0;
reg   [4:0] win_z3b0 = 5'd0;
reg   [3:0] win_z3b1 = 4'd0;

//------------------------------------------------------------- CPU, port B
reg  [31:0] c_addr = 32'd0, c_wdata = 32'd0;
reg         c_rd = 1'b0, c_wr = 1'b0, c_sup = 1'b1, c_fc_ovr = 1'b0;
reg   [2:0] c_fc_val = 3'd0;
reg   [1:0] c_size = `AP040_SZ_L;
reg         c_nalloc = 1'b0, c_m16 = 1'b0, c_lock = 1'b0;
wire [31:0] c_q;
wire        c_rvalid, c_wr_busy_w, c_rflt, c_wflt, c_flt_bus, c_flt_ma, c_idle, wr_pend;
wire        c_wr_drop = !c_wr;
reg         dc_en = 1'b0;
reg         cm_req = 1'b0, cm_dc = 1'b1, cm_push = 1'b0;
reg   [1:0] cm_scope = 2'b11;
reg  [31:0] cm_addr = 32'd0;
wire        cm_done;
reg         sn_req = 1'b0;
reg  [31:0] sn_addr = 32'd0;

//--------------------------------------------------------------- the units
wire        d_req, d_write, d_acc, d_sup, d_pass, d_flt;
wire [31:0] d_addr, d_pa;
wire  [1:0] d_cm;
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
reg         pw_ack_b = 1'b0;
wire        sb_accept, sb_empty;
wire [29:0] sb_sla;
wire  [1:0] mem_size;
wire [31:0] mem_addr, mem_wdata;
wire  [2:0] mem_fc;
reg         mem_ack = 1'b0, mem_flt = 1'b0;
reg  [31:0] mem_rdata = 32'd0;
wire        walker_req, walker_we;
wire        mw_req, mw_we, mw_ack, mw_berr;
wire [31:0] mw_addr, mw_wdat, mw_data;
wire [31:0] walker_addr, walker_wdat;
reg         walker_ack = 1'b0;
reg  [31:0] walker_data = 32'd0;
// the MMU's instruction-side port (test 19)
reg         i_req_b = 1'b0;
reg  [31:0] i_addr_b = 32'd0;
wire        i_pass_b, i_flt_b;
wire [31:0] i_pa_b;
wire  [1:0] i_cm_b;
reg  [31:0] ip_addr_b = 32'd0;   // the instruction side's peek (test 23)
reg  [31:0] itt0_b = 32'd0;      // ITT0 (test 23)
wire [31:0] ip_pa_b;
wire  [1:0] ip_cm_b;
reg         pf_req_b = 1'b0;   // PFLUSHA
wire        pf_done_b;

ap040_pipe_dmu u_dmu
(
	.clk (clk), .nreset (nreset),
	.xlat (tc[15] || dtt1[15]), .tc_e (tc[15]), .tc_p (tc[14]),
	.c_addr (c_addr), .c_rd (c_rd), .c_wr (c_wr), .c_size (c_size), .c_wdata (c_wdata),
	.c_sup (c_sup), .c_fc_ovr (c_fc_ovr), .c_fc_val (c_fc_val), .c_wr_drop (c_wr_drop),
	.c_nalloc (c_nalloc), .c_m16 (c_m16), .c_lock (c_lock),
	.dc_en (dc_en), .dtt0 (ttr0), .dtt1 (dtt1),
	.cm_req (cm_req), .cm_dc (cm_dc), .cm_push_in (cm_push), .cm_scope (cm_scope), .cm_addr (cm_addr), .cm_done (cm_done),
	.sn_req (sn_req), .sn_addr (sn_addr),
	.wk_req (mw_req || wk_req_b), .wk_we (wk_req_b ? wk_we_b : mw_we),
	.wk_addr (wk_req_b ? wk_addr_b : mw_addr), .wk_wdat (wk_req_b ? wk_wdat_b : mw_wdat),
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
	.pw_exc (pw_exc), .pw_ack (pw_ack_b),
	.sb_accept (sb_accept), .sb_sla (sb_sla), .sb_empty (sb_empty)
);

ap040_pipe_mmu u_mmu
(
	.clk (clk), .nreset (nreset),
	.tc (tc), .urp (urp), .srp (srp), .itt0 (itt0_b), .itt1 (ttr0), .dtt0 (ttr0), .dtt1 (dtt1),
	.cache_allow_all (win_all), .cache_z2_ena (win_z2), .cache_z3_base0 (win_z3b0), .cache_z3_ena0 (win_z3e0),
	.cache_z3_base1 (win_z3b1), .cache_z3_ena1 (win_z3e1),
	.i_req (i_req_b), .i_addr (i_addr_b), .i_sup (1'b1), .i_pass (i_pass_b), .i_flt (i_flt_b), .i_pa (i_pa_b), .i_cm (i_cm_b),
	.ip_addr (ip_addr_b), .ip_sup (1'b1), .ip_hit (), .ip_pa (ip_pa_b), .ip_cm (ip_cm_b),
	.d_req (d_req), .d_write (d_write), .d_acc (d_acc), .d_addr (d_addr), .d_sup (d_sup),
	.d_pass (d_pass), .d_flt (d_flt), .d_pa (d_pa), .d_cm (d_cm),
	.pt_req (1'b0), .pt_write (1'b0), .pt_access (1'b0), .pt_addr (32'd0), .pt_fc (3'd0),
	.pt_done (), .pt_mmusr (),
	.pf_req (pf_req_b), .pf_mode (2'b11), .pf_addr (32'd0), .pf_fc (3'd0), .pf_done (pf_done_b),
	.walk_hold (wr_pend),
	.walker_req (mw_req), .walker_we (mw_we), .walker_addr (mw_addr),
	.walker_wdat (mw_wdat), .walker_ack (mw_ack), .walker_data (mw_data),
	.walker_berr (mw_berr)
);

ap040_pipe_membus u_bus
(
	.clk (clk), .nreset (nreset),
	.f_req (1'b0), .f_addr (32'd0), .f_sup (1'b1), .f_free (),
	.f_ack (), .f_flt (), .f_flt_bus (),
	.w_accept (), .w_sla (),
	.address_b (bb_addr), .la_b (bb_la), .data_b (bb_wdata), .wren_b (bb_wr), .size_b (bb_size), .rd_b (bb_rd),
	.wr_busy (), .wr_busy_w (bb_wr_busy_w), .q_b (bb_q), .rvalid_b (bb_rvalid),
	.sup_b (bb_sup), .fc_ovr (bb_fc_ovr), .fc_ovr_val (bb_fc_val),
	.mem_req (mem_req), .mem_write (mem_write), .mem_instr (mem_instr), .mem_size (mem_size),
	.mem_addr (mem_addr), .mem_wdata (mem_wdata), .mem_fc (mem_fc), .mem_ack (mem_ack), .mem_rdata (mem_rdata),
	.mem_flt (mem_flt), .mem_flt_bus (mem_flt), .mem_pass (mem_req), .wr_sync (1'b0),
	.rflt_b (bb_rflt), .wflt (), .idle (bb_idle), .wr_berr (bb_wr_berr), .wr_drop (1'b0),
	.xlat_e (1'b0), .xlat_p (1'b0), .pb_req (), .pb_addr (), .pb_fc (), .pb_done (1'b0), .pb_mmusr (32'd0),
	.flt_ma (bb_flt_ma), .flt_bus (bb_flt_bus),
	.rx (bb_rx), .rx_addr (bb_rx_addr), .rx_size (bb_rx_size), .rx_fc (bb_rx_fc)
);

//------------------------------------------------ the bus controller's memory
// mem_lat cycles to an answer; a read of berr_addr, while armed, is answered
// with a bus error instead (once). Every transaction taken is logged.
integer mem_lat = 1, mem_cnt = 0;
reg     mem_busy = 1'b0;
reg     berr_arm = 1'b0;
reg [31:0] berr_addr = 32'd0;
reg     wberr_arm = 1'b0;           // ...and a write to wberr_addr (test 20)
reg [31:0] wberr_addr = 32'd0;
reg  [31:0] log_addr  [0:4095];
reg         log_write [0:4095];
integer     log_n = 0;
always @(posedge clk) begin
	mem_ack <= 1'b0;
	mem_flt <= 1'b0;
	if (!nreset) begin
		mem_busy <= 1'b0;
	end else if (mem_req && !mem_ack && !mem_flt) begin
		if (!mem_busy) begin
			mem_busy <= 1'b1; mem_cnt = mem_lat;
			log_addr[log_n] = mem_addr; log_write[log_n] = mem_write; log_n = log_n + 1;
		end else if (mem_cnt > 0) mem_cnt = mem_cnt - 1;
		else begin
			mem_busy <= 1'b0;
			if (!mem_write && berr_arm && mem_addr == berr_addr) begin
				mem_flt  <= 1'b1;
				berr_arm <= 1'b0;
			end else if (mem_write && wberr_arm && mem_addr == wberr_addr) begin
				mem_flt   <= 1'b1;
				wberr_arm <= 1'b0;
			end else begin
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
end

//------------------------------------------------------ the walker's port
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

//------------------------------------------------------------------ probes
function lvalid;   // a line holding physical address a is valid
	input [31:0] a;
	integer w;
	begin
		lvalid = 1'b0;
		for (w = 0; w < 4; w = w + 1)
			if (u_dmu.vld[a[9:4]][w] && u_dmu.u_arr.tags.mem[a[9:4]][w*22 +: 22] == a[31:10])
				lvalid = 1'b1;
	end
endfunction
function [31:0] cword;   // the longword a valid line holds for a (x if none)
	input [31:0] a;
	integer w;
	begin
		cword = 32'hxxxxxxxx;
		for (w = 0; w < 4; w = w + 1)
			if (u_dmu.vld[a[9:4]][w] && u_dmu.u_arr.tags.mem[a[9:4]][w*22 +: 22] == a[31:10])
				case (w)
				0: cword = u_dmu.u_arr.way[0].data.mem[a[9:2]];
				1: cword = u_dmu.u_arr.way[1].data.mem[a[9:2]];
				2: cword = u_dmu.u_arr.way[2].data.mem[a[9:2]];
				default: cword = u_dmu.u_arr.way[3].data.mem[a[9:2]];
				endcase
	end
endfunction
// the counter's rule: reads looked up, writes sent, ways named
integer lookups = 0, sends = 0, replaced = 0;
always @(posedge clk) if (nreset) begin
	if (u_dmu.rd_lk) lookups = lookups + 1;
	if (dc_en && u_dmu.m_wr) sends = sends + 1;
end
integer cm_pulses = 0, sn_fills = 0, sn_junks = 0;
always @(posedge clk) if (nreset) begin
	if (cm_done) cm_pulses = cm_pulses + 1;
	if (u_dmu.sn_fill) sn_fills = sn_fills + 1;
	if (u_dmu.sn_v2 && u_dmu.sn_junk2) sn_junks = sn_junks + 1;
end

//----------------------------------------------------------------- driver
reg [31:0] rd_q;
reg        rd_flt, rd_bus;
integer    n;
task rd;   // a read, answered
	input [31:0] a;
	input  [1:0] sz;
	begin
		c_addr = a; c_size = sz; c_rd = 1'b1;
		step;
		c_rd = 1'b0;
		n = 1;
		while (!c_rvalid && n < 400) begin step; n = n + 1; end
		rd_q = c_q; rd_flt = c_rflt; rd_bus = c_flt_bus;
		if (n >= 400) fail("a read was never answered");
	end
endtask
task wr;   // a write, held until taken
	input [31:0] a;
	input  [1:0] sz;
	input [31:0] d;
	integer k;
	begin
		c_addr = a; c_size = sz; c_wdata = d; c_wr = 1'b1;
		#0;
		k = 0;
		while (c_wr_busy_w && k < 400) begin step; k = k + 1; end
		step;
		c_wr = 1'b0;
	end
endtask
task quiet;   // until nothing has moved on the bus for 20 cycles and the unit is idle
	integer k, last;
	begin
		k = 0; last = log_n;
		while (k < 20) begin
			step;
			if (log_n != last || mem_req || !c_idle) begin k = 0; last = log_n; end else k = k + 1;
		end
	end
endtask
task cinv;   // the DMU's CINV/CPUSH: held until done, then down a clock
	input [1:0] scope;
	input [31:0] a;
	integer k;
	begin
		cm_scope = scope; cm_addr = a; cm_req = 1'b1;
		k = 0;
		while (!cm_done && k < 400) begin step; k = k + 1; end
		n = k;
		if (!cm_done) fail("CINV never finished");
		step;
		cm_req = 1'b0;
		step;
	end
endtask
task cinv_p;   // CINV or CPUSH
	input [1:0] scope;
	input [31:0] a;
	input        push;
	begin
		cm_push = push;
		cinv(scope, a);
		cm_push = 1'b0;
	end
endtask
// The walker's port, driven as the MMU drives it: a request held until its
// acknowledgement, then down a cycle.
reg        wk_req_b = 1'b0, wk_we_b = 1'b0;
reg [31:0] wk_addr_b = 32'd0, wk_wdat_b = 32'd0;
reg [31:0] wk_got;
integer    wk_ext = 0;
always @(posedge clk) if (nreset && walker_req && !wk_q) wk_ext = wk_ext + 1;
// The write-back fault held (test 20): each field as the frame takes it.
task chk_pw;
	input  [15:0] ssw;
	input  [31:0] fa;
	input   [7:0] wb1s;
	input [127:0] pd;
	input         exc;
	input [8*64-1:0] what;
	begin
		if (!pw_pend || pw_ssw !== ssw || pw_fa !== fa || pw_wb1s !== wb1s || pw_pd !== pd || pw_exc !== exc) begin
			errors = errors + 1;
			$display("FAIL: %0s: pend %b ssw %h fa %h wb1s %h exc %b pd %h", what, pw_pend, pw_ssw, pw_fa, pw_wb1s,
			         pw_exc, pw_pd);
			$display("      want ssw %h fa %h wb1s %h exc %b pd %h", ssw, fa, wb1s, exc, pd);
		end
	end
endtask
task pw_let_go;   // EA-fetch's entry for it is done
	begin
		pw_ack_b = 1'b1; step; pw_ack_b = 1'b0; step;
		if (pw_pend) fail("20: a fault let go is still held");
	end
endtask
task wr_berr;   // a write whose bus error is armed; the fault then held
	input [31:0] a;
	input  [1:0] sz;
	input [31:0] d;
	input [31:0] ea;
	begin
		wberr_arm = 1'b1; wberr_addr = ea;
		wr(a, sz, d);
		quiet;
		if (wberr_arm) begin wberr_arm = 1'b0; fail("20: the write never erred (the test no longer tests)"); end
	end
endtask
task walk_access;
	input [31:0] a;
	input        we;
	input [31:0] d;
	integer k;
	begin
		wk_addr_b = a; wk_we_b = we; wk_wdat_b = d; wk_req_b = 1'b1;
		k = 0;
		while (!mw_ack && k < 400) begin step; k = k + 1; end
		wk_got = mw_data;
		if (k >= 400) fail("a walker access was never answered");
		step;
		wk_req_b = 1'b0;
		step; step;
	end
endtask
task want_rd;   // a read's answer
	input [31:0] a;
	input  [1:0] sz;
	input [31:0] want;
	input [8*56-1:0] what;
	begin
		rd(a, sz);
		if (rd_flt || rd_q !== want) begin
			errors = errors + 1;
			$display("FAIL: %0s: read %h size %0d gave %h flt %b, want %h", what, a, sz, rd_q, rd_flt, want);
		end
	end
endtask

integer i, k, d, w, lg, rep_m, t0;
reg [31:0] v;
initial begin
	for (i = 0; i < 65536; i = i + 1) mem[i] = i[7:0] ^ i[15:8];
	// tables for test 11: identity, page 6 CM 10
	wr32(16'h4000, 32'h0000_4203);
	wr32(16'h4200, 32'h0000_4403);
	for (i = 0; i < 16; i = i + 1) wr32(16'h4400 + i * 4, (i << 12) | 3);
	wr32(16'h4418, 32'h0000_6043);
	repeat (4) step;
	nreset = 1'b1;
	repeat (2) step;
	dc_en = 1'b1;
	cinv(2'b11, 32'd0);

	//------------------------------------------------------------- test 1
	mem_lat = 3;
	lg = log_n;
	want_rd(32'h0000_5008, `AP040_SZ_L, rd32(16'h5008), "1: a miss");
	quiet;
	if (log_n < lg + 4 || log_addr[lg] !== 32'h5008 || log_addr[lg + 1] !== 32'h500C ||
	    log_addr[lg + 2] !== 32'h5000 || log_addr[lg + 3] !== 32'h5004) begin
		$display("    reads %h %h %h %h", log_addr[lg], log_addr[lg + 1], log_addr[lg + 2], log_addr[lg + 3]);
		fail("1: a miss did not read its line from the longword asked for, wrapping");
	end
	if (!lvalid(32'h5000)) fail("1: the line is not valid after its fill");

	//------------------------------------------------------------- test 2
	lg = log_n;
	rd(32'h0000_5004, `AP040_SZ_L);
	if (n != 3) begin
		$display("    answered %0d cycle(s) after the request's", n);
		fail("2: a hit was not answered three cycles after its request");
	end
	if (rd_q !== rd32(16'h5004)) fail("2: a hit returned the wrong longword");
	for (k = 0; k < 4; k = k + 1) begin
		v = rd32(16'h5004);
		want_rd(32'h0000_5004 + k, `AP040_SZ_B, {24'd0, v[31 - k * 8 -: 8]}, "2: a byte hit");
		if (k < 3) want_rd(32'h0000_5004 + k, `AP040_SZ_W, {16'd0, v[31 - k * 8 -: 16]}, "2: a word hit");
	end
	quiet;
	if (log_n != lg) fail("2: a hit read the bus");

	//------------------------------------------------------------- test 3
	wr(32'h0000_5005, `AP040_SZ_B, 32'h0000_00A5);
	want_rd(32'h0000_5004, `AP040_SZ_L, {rd32(16'h5004) & 32'hFF00FFFF} | 32'h00A50000, "3: read right after a byte write");
	quiet;
	if (cword(32'h5004) !== rd32(16'h5004)) fail("3: a byte write left the line and memory different");
	wr(32'h0000_5001, `AP040_SZ_W, 32'h0000_B1B2);
	quiet;
	if (cword(32'h5000) !== rd32(16'h5000) || mem[16'h5001] !== 8'hB1 || mem[16'h5002] !== 8'hB2)
		fail("3: a word write at an odd address did not put its bytes in the line");
	wr(32'h0000_5006, `AP040_SZ_L, 32'hC1C2_C3C4);   // across two longwords of the line
	quiet;
	if (cword(32'h5004) !== rd32(16'h5004) || cword(32'h5008) !== rd32(16'h5008) ||
	    rd32(16'h5006) !== 32'hC1C2_C3C4)
		fail("3: a longword write across two longwords did not update both");
	want_rd(32'h0000_5010, `AP040_SZ_L, rd32(16'h5010), "3: the next line");
	wr(32'h0000_500E, `AP040_SZ_L, 32'hD1D2_D3D4);   // across two lines
	quiet;
	if (cword(32'h500C) !== rd32(16'h500C) || cword(32'h5010) !== rd32(16'h5010) ||
	    rd32(16'h500E) !== 32'hD1D2_D3D4)
		fail("3: a longword write across two lines did not update both");

	//------------------------------------------------------------- test 4
	mem_lat = 6;
	for (d = 1; d < 40; d = d + 1) begin
		cinv(2'b11, 32'd0);
		fork
			rd(32'h0000_5100 + d * 16, `AP040_SZ_L);
			begin
				repeat (d) step;
				wr(32'h0000_510C + d * 16, `AP040_SZ_L, 32'h7E00_0000 + d);
			end
		join
		quiet;
		if (!lvalid(32'h0000_5100 + d * 16)) begin
			$display("    d=%0d", d);
			fail("4: a line written into while being read was not left valid");
		end else if (cword(32'h0000_510C + d * 16) !== 32'h7E00_0000 + d) begin
			$display("    d=%0d: the line holds %h", d, cword(32'h0000_510C + d * 16));
			fail("4: a line written into while being read lost the write");
		end
	end
	mem_lat = 1;

	//------------------------------------------------------------- test 5
	cinv(2'b11, 32'd0);
	want_rd(32'h0000_5200, `AP040_SZ_L, rd32(16'h5200), "5: fill");
	quiet;
	dtt1 = 32'h0000_C040;                 // $00xxxxxx, CM 10
	repeat (2) step;
	lg = log_n;
	want_rd(32'h0000_5204, `AP040_SZ_L, rd32(16'h5204), "5: inhibited");
	quiet;
	if (log_n != lg + 1) fail("5: an inhibited read was not one bus read");
	if (lvalid(32'h5200)) fail("5: an inhibited read that hit left the line valid");
	dtt1 = 32'd0;
	repeat (2) step;
	// locked
	want_rd(32'h0000_5300, `AP040_SZ_L, rd32(16'h5300), "5: fill");
	quiet;
	c_lock = 1'b1;
	lg = log_n;
	want_rd(32'h0000_5300, `AP040_SZ_B, {24'd0, mem[16'h5300]}, "5: locked");
	c_lock = 1'b0;
	quiet;
	if (log_n != lg + 1 || lvalid(32'h5300)) fail("5: a locked read was cached, or left its line");
	// MOVES to FC 3
	want_rd(32'h0000_5310, `AP040_SZ_L, rd32(16'h5310), "5: fill");
	quiet;
	c_fc_ovr = 1'b1; c_fc_val = 3'd3;
	lg = log_n;
	want_rd(32'h0000_5310, `AP040_SZ_L, rd32(16'h5310), "5: MOVES FC 3");
	c_fc_ovr = 1'b0;
	quiet;
	if (log_n != lg + 1 || lvalid(32'h5310)) fail("5: a MOVES to an alternate space was cached, or left its line");
	// no allocation
	c_nalloc = 1'b1;
	lg = log_n;
	want_rd(32'h0000_5400, `AP040_SZ_L, rd32(16'h5400), "5: a miss allocating nothing");
	quiet;
	if (log_n != lg + 1 || lvalid(32'h5400)) fail("5: a read allocating nothing filled a line");
	c_nalloc = 1'b0;
	want_rd(32'h0000_5400, `AP040_SZ_L, rd32(16'h5400), "5: fill");
	quiet;
	c_nalloc = 1'b1;
	lg = log_n;
	want_rd(32'h0000_5404, `AP040_SZ_L, rd32(16'h5404), "5: a hit allocating nothing");
	c_nalloc = 1'b0;
	quiet;
	if (log_n != lg) fail("5: a hit of a read allocating nothing read the bus");
	// MOVE16 write hit
	c_m16 = 1'b1;
	wr(32'h0000_5408, `AP040_SZ_L, 32'h1616_1616);
	c_m16 = 1'b0;
	quiet;
	if (lvalid(32'h5400)) fail("5: a MOVE16 write that hit left the line valid");
	if (rd32(16'h5408) !== 32'h1616_1616) fail("5: a MOVE16 write did not reach memory");

	//------------------------------------------------------------- test 6
	cinv(2'b11, 32'd0);
	want_rd(32'h0000_0600, `AP040_SZ_L, rd32(16'h0600), "6: fill");
	want_rd(32'h0000_0A00, `AP040_SZ_L, rd32(16'h0A00), "6: fill");
	want_rd(32'h0000_1600, `AP040_SZ_L, rd32(16'h1600), "6: fill");
	want_rd(32'h0000_1A00, `AP040_SZ_L, rd32(16'h1A00), "6: fill");
	quiet;
	k = cm_pulses;
	cinv(2'b01, 32'h0000_0A08);
	if (n > 4) fail("6: CINVL took more than a row");
	if (lvalid(32'h0A00)) fail("6: CINVL left its line");
	if (!lvalid(32'h0600) || !lvalid(32'h1600) || !lvalid(32'h1A00)) fail("6: CINVL took another line");
	cinv(2'b10, 32'h0000_1FFC);
	if (n < 64) fail("6: CINVP did not read every set");
	if (lvalid(32'h1600) || lvalid(32'h1A00)) fail("6: CINVP left a line of its page");
	if (!lvalid(32'h0600)) fail("6: CINVP took a line of another page");
	if (cm_pulses != k + 2) fail("6: CINV's done was not one clock per request");
	// held after its done: not taken again
	k = cm_pulses;
	cm_scope = 2'b11; cm_req = 1'b1;
	repeat (12) step;
	cm_req = 1'b0; step;
	if (cm_pulses != k + 1) fail("6: a request held after its done was taken again");
	// the instruction cache's alone: done at once, the data cache left
	want_rd(32'h0000_0600, `AP040_SZ_L, rd32(16'h0600), "6: fill");
	quiet;
	cm_dc = 1'b0;
	cinv(2'b11, 32'd0);
	cinv(2'b01, 32'h0000_0600);
	cinv(2'b10, 32'h0000_0600);
	cm_dc = 1'b1;
	if (!lvalid(32'h0600)) fail("6: an instruction-cache CINV took a data line");

	//------------------------------------------------------------- test 7
	cinv(2'b11, 32'd0);
	mem_lat = 3;
	berr_arm = 1'b1; berr_addr = 32'h0000_5508;
	want_rd(32'h0000_5500, `AP040_SZ_L, rd32(16'h5500), "7: the line's first");
	quiet;
	if (berr_arm) fail("7: the error was never met (the test no longer tests)");
	if (lvalid(32'h5500)) fail("7: a line with an errored beat is valid");
	// waiting for the errored longword
	cinv(2'b11, 32'd0);
	berr_arm = 1'b1; berr_addr = 32'h0000_5608;
	mem_lat = 8;
	rd(32'h0000_5600, `AP040_SZ_L);
	c_addr = 32'h0000_5608; c_size = `AP040_SZ_L; c_rd = 1'b1; step; c_rd = 1'b0;
	n = 1;
	while (!c_rvalid && n < 400) begin step; n = n + 1; end
	if (!c_rflt || !c_flt_bus) fail("7: a read waiting for the longword whose read erred was not faulted");
	quiet;
	// waiting for another longword: answered after the line is read again
	cinv(2'b11, 32'd0);
	berr_arm = 1'b1; berr_addr = 32'h0000_5704;
	rd(32'h0000_5700, `AP040_SZ_L);
	lg = log_n;
	want_rd(32'h0000_570C, `AP040_SZ_L, rd32(16'h570C), "7: a read waiting for another longword");
	quiet;
	if (berr_arm) fail("7: $5704's error was never met (the test no longer tests)");
	// The way an abandoned fill wrote into still holds the tag of the line
	// that was there: $5B00 is read, then invalidated by CINVL, and $5F00 --
	// the same set, into the way $5B00 left -- errs on its third beat, after
	// two of its longwords are in that way. $5B00 must miss.
	cinv(2'b11, 32'd0);
	want_rd(32'h0000_5B00, `AP040_SZ_L, rd32(16'h5B00), "7: fill");
	quiet;
	cinv(2'b01, 32'h0000_5B00);
	berr_arm = 1'b1; berr_addr = 32'h0000_5F08;
	want_rd(32'h0000_5F00, `AP040_SZ_L, rd32(16'h5F00), "7: the fill that errs");
	quiet;
	if (berr_arm) fail("7: $5F08's error was never met (the test no longer tests)");
	lg = log_n;
	want_rd(32'h0000_5B04, `AP040_SZ_L, rd32(16'h5B04), "7: the invalidated line, after a fill into its way erred");
	quiet;
	if (log_n == lg) fail("7: the invalidated line hit after a fill into its way erred");
	mem_lat = 1;

	//------------------------------------------------------------- test 8
	cinv(2'b11, 32'd0);
	want_rd(32'h0000_5800, `AP040_SZ_L, rd32(16'h5800), "8: fill");
	want_rd(32'h0000_5C00, `AP040_SZ_L, rd32(16'h5C00), "8: fill");
	quiet;
	sn_addr = 32'h0000_5804; sn_req = 1'b1; step; sn_req = 1'b0;
	repeat (4) step;
	if (lvalid(32'h5800)) fail("8: a snooped line is still valid");
	if (!lvalid(32'h5C00)) fail("8: a snoop took another line of its set");
	mem_lat = 3;
	k = 0;
	for (d = 0; d < 40; d = d + 1) begin
		cinv(2'b11, 32'd0);
		fork
			rd(32'h0000_6000 + d * 16, `AP040_SZ_L);
			begin
				repeat (d) step;
				// another master writes the line's first longword
				wr32(16'h6000 + d * 16, 32'h5A00_0000 + d);
				sn_addr = 32'h0000_6000 + d * 16; sn_req = 1'b1; step; sn_req = 1'b0;
			end
		join
		quiet;
		if (lvalid(32'h0000_6000 + d * 16)) begin
			k = k + 1;
			if (cword(32'h0000_6000 + d * 16) !== 32'h5A00_0000 + d) begin
				$display("    snooped %0d cycle(s) after the read: the line holds %h", d, cword(32'h0000_6000 + d * 16));
				fail("8: a line snooped across its fill is valid with the longword from before the write");
			end
		end
	end
	$display("test 8: %0d of 40 lines valid after the sweep", k);
	mem_lat = 1;
	$display("test 8: %0d snoop(s) of a line being read, %0d at a tag write", sn_fills, sn_junks);
	if (sn_fills == 0) fail("8: no snoop met a fill (the test no longer tests)");
	if (sn_junks == 0) fail("8: no snoop's row read met a fill's tag write (the test no longer tests)");

	//------------------------------------------------------------- test 9
	cinv(2'b11, 32'd0);
	// set $12: $0120 $0520 $0920 $0D20, then $1120 and $1520
	for (k = 0; k < 6; k = k + 1) begin
		fork
			rd(32'h0000_0120 + k * 32'h400, `AP040_SZ_L);
			begin
				while (!u_dmu.fl_start) step;
				rep_m = (lookups + sends + replaced) % 4;
				w = u_dmu.fl_vict;
				if (k < 4 && w != k) begin
					$display("    line %0d took way %0d", k, w);
					fail("9: a fill with an invalid way free did not take the first");
				end
				if (k >= 4) begin
					if (w != rep_m) begin
						$display("    line %0d took way %0d, the counter says %0d", k, w, rep_m);
						fail("9: a full set's fill did not take the way the counter names");
					end
					replaced = replaced + 1;
				end
			end
		join
		quiet;
		// a write, which the counter counts too
		wr(32'h0000_3000, `AP040_SZ_L, 32'h0);
		quiet;
	end

	//------------------------------------------------------------- test 10
	cinv(2'b11, 32'd0);
	mem_lat = 8;
	fork
		rd(32'h0000_6400, `AP040_SZ_L);
		begin
			while (!u_dmu.fl_act) step;
			dc_en = 1'b0;
		end
	join
	// a read at once, while the line is still being read
	if (!u_dmu.fl_act) fail("10: the fill ended before the read (the test no longer tests)");
	want_rd(32'h0000_6A00, `AP040_SZ_L, rd32(16'h6A00), "10: a read with DE clear, under the fill");
	quiet;
	if (!lvalid(32'h6400)) fail("10: a fill DE was cleared under was not completed");
	if (cword(32'h6408) !== rd32(16'h6408)) fail("10: a fill a read went past holds the wrong longword");
	lg = log_n;
	want_rd(32'h0000_6404, `AP040_SZ_L, rd32(16'h6404), "10: after DE cleared");
	quiet;
	if (log_n != lg + 1) fail("10: a read with DE clear was not one bus read");
	dc_en = 1'b1;
	mem_lat = 1;

	//------------------------------------------------------------- test 11
	cinv(2'b11, 32'd0);
	tc = 32'h0000_8000;
	repeat (4) step;
	want_rd(32'h0000_6800, `AP040_SZ_L, rd32(16'h6800), "11: page 6");
	want_rd(32'h0000_5900, `AP040_SZ_L, rd32(16'h5900), "11: page 5");
	quiet;
	if (lvalid(32'h6800)) fail("11: a page whose descriptor says CM 10 was cached");
	if (!lvalid(32'h5900)) fail("11: a write-through page was not cached");
	tc = 32'd0;

	//------------------------------------------------------------- test 12
	cinv(2'b11, 32'd0);
	dtt1 = 32'h0000_C020;                // $00xxxxxx, either mode, CM 01
	repeat (2) step;
	v = rd32(16'h7000);
	lg = log_n;
	wr(32'h0000_7004, `AP040_SZ_L, 32'hC0DE_0001);
	quiet;
	if (rd32(16'h7004) === 32'hC0DE_0001) fail("12: a copyback write that missed reached memory");
	if (cword(32'h7004) !== 32'hC0DE_0001) fail("12: a copyback write that missed is not in its line");
	for (k = lg; k < log_n; k = k + 1) if (log_write[k]) fail("12: a copyback write missing wrote the bus");
	wr(32'h0000_7000, `AP040_SZ_B, 32'h0000_00AB);
	quiet;
	if (mem[16'h7000] === 8'hAB) fail("12: a copyback write that hit reached memory");
	want_rd(32'h0000_7000, `AP040_SZ_L, {8'hAB, v[23:0]}, "12: read back the copyback bytes");
	lg = log_n;
	cinv_p(2'b01, 32'h0000_7000, 1'b1);  // CPUSHL
	quiet;
	k = 0; for (i = lg; i < log_n; i = i + 1) if (log_write[i]) k = k + 1;
	if (k != 2) begin $display("    %0d push write(s)", k); fail("12: CPUSHL did not push exactly the two dirty longwords"); end
	if (rd32(16'h7000) !== {8'hAB, v[23:0]} || rd32(16'h7004) !== 32'hC0DE_0001) fail("12: CPUSHL did not put the dirty data in memory");
	if (lvalid(32'h7000)) fail("12: CPUSHL left the line");
	wr(32'h0000_7008, `AP040_SZ_L, 32'hDEAD_0008);
	quiet;
	cinv(2'b01, 32'h0000_7008);           // CINVL: the dirty longword is lost
	if (rd32(16'h7008) === 32'hDEAD_0008) fail("12: CINVL wrote the dirty longword");
	// a byte write that misses: the rest of its longword is memory's
	v = rd32(16'h7106);
	v = rd32(16'h7104);
	wr(32'h0000_7106, `AP040_SZ_B, 32'h0000_00EE);
	quiet;
	if (cword(32'h7104) !== {v[31:16], 8'hEE, v[7:0]}) begin
		$display("    the line holds %h, want %h", cword(32'h7104), {v[31:16], 8'hEE, v[7:0]});
		fail("12: a copyback byte write that missed did not land in memory's longword");
	end
	if (mem[16'h7106] === 8'hEE) fail("12: a copyback byte write that missed reached memory");
	// a write-through write to a dirty longword keeps it dirty (Table 4-4):
	// the CPUSH after it writes both dirty longwords
	cinv_p(2'b11, 32'd0, 1'b1);
	wr(32'h0000_7200, `AP040_SZ_L, 32'h1200_7200);
	wr(32'h0000_7204, `AP040_SZ_L, 32'h1200_7204);
	quiet;
	dtt1 = 32'h0000_C000;                // CM 00: write-through
	repeat (2) step;
	wr(32'h0000_7200, `AP040_SZ_L, 32'h1200_0000);
	quiet;
	dtt1 = 32'h0000_C020;
	repeat (2) step;
	if (rd32(16'h7200) !== 32'h1200_0000) fail("12: a write-through write to a dirty line did not reach memory");
	lg = log_n;
	cinv_p(2'b01, 32'h0000_7200, 1'b1);
	quiet;
	k = 0; for (i = lg; i < log_n; i = i + 1) if (log_write[i]) k = k + 1;
	if (k != 2) begin $display("    %0d push write(s)", k); fail("12: a write-through write cleared its longword's dirty bit"); end

	//------------------------------------------------------------- test 13
	for (d = 0; d < 6; d = d + 1) for (k = 0; k < 4; k = k + 1) begin
		mem_lat = d;
		cinv(2'b11, 32'd0);
		// four lines of set $30 made dirty, all four longwords each
		for (w = 0; w < 4; w = w + 1)
			for (i = 0; i < 4; i = i + 1)
				wr(32'h0000_0300 + w * 32'h400 + i * 4, `AP040_SZ_L, {8'h30 + w[7:0], 8'h00 + d[7:0], 8'h00 + k[7:0], i[7:0]});
		quiet;
		// a fifth line, read from its k'th longword: one dirty line replaced;
		// then every longword of the four read back at once -- the replaced
		// line's while its push drains
		want_rd(32'h0000_1300 + k * 4, `AP040_SZ_L, rd32(16'h1300 + k * 4), "13: the fifth line");
		for (w = 0; w < 4; w = w + 1) for (i = 0; i < 4; i = i + 1)
			want_rd(32'h0000_0300 + w * 32'h400 + i * 4, `AP040_SZ_L,
			        {8'h30 + w[7:0], 8'h00 + d[7:0], 8'h00 + k[7:0], i[7:0]}, "13: read back at once");
		quiet;
		for (w = 0; w < 4; w = w + 1) for (i = 0; i < 4; i = i + 1) begin
			v = {8'h30 + w[7:0], 8'h00 + d[7:0], 8'h00 + k[7:0], i[7:0]};
			if (lvalid(32'h0000_0300 + w * 32'h400)) begin
				if (cword(32'h0000_0300 + w * 32'h400 + i * 4) !== v) fail("13: a kept dirty line changed");
			end else if (rd32(16'h0300 + w * 32'h400 + i * 4) !== v) begin
				$display("    lat %0d first %0d: line %0d longword %0d in memory %h, want %h", d, k, w, i,
				         rd32(16'h0300 + w * 32'h400 + i * 4), v);
				fail("13: a replaced dirty line's longword was not pushed intact");
			end
		end
		// (the read back replaced lines again: the fifth line may have gone)
		for (i = 0; i < 4; i = i + 1)
			if (lvalid(32'h1300) && cword(32'h0000_1300 + i * 4) !== rd32(16'h1300 + i * 4))
				fail("13: the fifth line's longword is wrong");
	end
	mem_lat = 1;

	//------------------------------------------------------------- test 14
	// set $34: $0340 and $0740 in page 0, $1340 and $1740 in page 1; the
	// first three dirty, the fourth clean
	cinv(2'b11, 32'd0);
	want_rd(32'h0000_0340, `AP040_SZ_L, rd32(16'h0340), "14: fill");
	want_rd(32'h0000_0740, `AP040_SZ_L, rd32(16'h0740), "14: fill");
	want_rd(32'h0000_1340, `AP040_SZ_L, rd32(16'h1340), "14: fill");
	want_rd(32'h0000_1740, `AP040_SZ_L, rd32(16'h1740), "14: fill");
	wr(32'h0000_0348, `AP040_SZ_L, 32'h1400_0000);
	wr(32'h0000_0748, `AP040_SZ_L, 32'h1400_0001);
	wr(32'h0000_1348, `AP040_SZ_L, 32'h1400_0002);
	quiet;
	cinv_p(2'b01, 32'h0000_0348, 1'b1);   // CPUSHL: $0340's line only
	quiet;
	if (lvalid(32'h0340) || !lvalid(32'h0740) || !lvalid(32'h1340) || !lvalid(32'h1740))
		fail("14: CPUSHL took another line, or left its own");
	if (rd32(16'h0348) !== 32'h1400_0000) fail("14: CPUSHL did not push its line");
	lg = log_n;
	cinv_p(2'b10, 32'h0000_1ABC, 1'b1);   // CPUSHP page 1: $1340 (dirty) and $1740 (clean)
	quiet;
	if (lvalid(32'h1340) || lvalid(32'h1740) || !lvalid(32'h0740)) fail("14: CPUSHP took the wrong lines");
	if (rd32(16'h1348) !== 32'h1400_0002) fail("14: CPUSHP did not push its dirty line");
	k = 0; for (i = lg; i < log_n; i = i + 1) if (log_write[i]) k = k + 1;
	if (k != 1) begin $display("    %0d write(s)", k); fail("14: CPUSHP wrote other than the one dirty longword"); end
	if (rd32(16'h0748) === 32'h1400_0001) fail("14: page 0's dirty line reached memory before its CPUSH");
	cinv_p(2'b11, 32'h0000_0000, 1'b1);   // CPUSHA
	quiet;
	if (lvalid(32'h0740)) fail("14: CPUSHA left a line");
	if (rd32(16'h0748) !== 32'h1400_0001) fail("14: CPUSHA did not push a dirty line");

	//------------------------------------------------------------- test 15
	// the walker's port, driven as the MMU drives it
	cinv(2'b11, 32'd0);
	want_rd(32'h0000_7200, `AP040_SZ_L, rd32(16'h7200), "15: fill");
	wr(32'h0000_7204, `AP040_SZ_L, 32'h0000_7203);   // a descriptor, dirty in the line
	quiet;
	wk_ext = 0;
	walk_access(32'h0000_7204, 1'b0, 32'd0);
	if (wk_got !== 32'h0000_7203) fail("15: the walker's read of a dirty descriptor did not get the cached one");
	if (wk_ext != 0) fail("15: a walker read that hit went to the walker's port");
	walk_access(32'h0000_7204, 1'b1, 32'h0000_720B);  // its U write
	if (cword(32'h7204) !== 32'h0000_720B) fail("15: the walker's write did not update the line");
	if (rd32(16'h7204) !== 32'h0000_720B) fail("15: the walker's write did not reach memory");
	lg = log_n;
	cinv_p(2'b01, 32'h0000_7200, 1'b1);
	quiet;
	for (i = lg; i < log_n; i = i + 1)
		if (log_write[i] && log_addr[i] == 32'h7204) fail("15: a longword the walker wrote was pushed as dirty");
	wk_ext = 0;
	walk_access(32'h0000_7300, 1'b0, 32'd0);
	if (wk_ext != 1 || wk_got !== rd32(16'h7300)) fail("15: a walker read that missed was not the port's");

	//------------------------------------------------------------- test 16
	cinv(2'b11, 32'd0);
	c_nalloc = 1'b1;
	lg = log_n;
	wr(32'h0000_7400, `AP040_SZ_L, 32'h1600_0000);
	c_nalloc = 1'b0;
	quiet;
	if (log_n != lg + 1 || !log_write[lg] || rd32(16'h7400) !== 32'h1600_0000 || lvalid(32'h7400))
		fail("16: a copyback write allocating nothing that missed was not one bus write");
	// an inhibited read of a dirty line: its pushes first
	wr(32'h0000_7500, `AP040_SZ_L, 32'h1600_7500);
	quiet;
	c_lock = 1'b1;
	lg = log_n;
	want_rd(32'h0000_7500, `AP040_SZ_L, 32'h1600_7500, "16: a locked read of a dirty line");
	c_lock = 1'b0;
	quiet;
	if (log_n < lg + 2 || !log_write[lg] || log_write[lg + 1]) fail("16: the push was not on the bus before the read");
	// a copyback write meeting a line being read
	mem_lat = 6;
	fork
		rd(32'h0000_7600, `AP040_SZ_L);
		begin
			repeat (3) step;
			wr(32'h0000_760C, `AP040_SZ_L, 32'h1600_760C);
		end
	join
	quiet;
	if (cword(32'h760C) !== 32'h1600_760C) fail("16: a copyback write meeting its line's fill is not in the line");
	if (rd32(16'h760C) === 32'h1600_760C) fail("16: a copyback write meeting its line's fill reached memory");
	mem_lat = 1;

	//------------------------------------------------------------- test 17
	for (k = 0; k < 2; k = k + 1) begin
		cinv_p(2'b11, 32'd0, 1'b1);
		berr_arm = 1'b1; berr_addr = (k == 0) ? 32'h0000_7704 : 32'h0000_770C;
		lg = log_n;
		wr(32'h0000_7704, `AP040_SZ_L, 32'h1700_0000 + k);
		quiet;
		if (berr_arm) fail("17: the line read never erred (the test no longer tests)");
		if (rd32(16'h7704) !== 32'h1700_0000 + k) begin
			$display("    error on %h", berr_addr);
			fail("17: a copyback write whose line read erred did not reach memory");
		end
		if (lvalid(32'h7700)) fail("17: a line whose read erred is valid");
		if (!c_idle) fail("17: the unit did not go idle after a copyback write's line read erred");
	end
	//------------------------------------------------------------- test 18
	// A write-through write to a longword a replaced line is still pushing:
	// the push -- older -- reaches memory first, the write last. Swept.
	for (d = 0; d < 12; d = d + 1) begin
		mem_lat = 4;
		cinv_p(2'b11, 32'd0, 1'b1);
		for (w = 0; w < 4; w = w + 1)
			for (i = 0; i < 4; i = i + 1)
				wr(32'h0000_0380 + w * 32'h400 + i * 4, `AP040_SZ_L, 32'h1800_0000 + w * 16 + i);
		quiet;
		fork
			rd(32'h0000_1380, `AP040_SZ_L);                  // a fifth line: one replaced
			begin
				while (!u_dmu.pb_act) step;
				repeat (d) step;
				dtt1 = 32'h0000_C000;                        // write-through, now
				k = u_dmu.pb_line[7:6];                      // the line going out
				wr(32'h0000_0384 + k * 32'h400, `AP040_SZ_L, 32'h18FF_0000 + d);
				dtt1 = 32'h0000_C020;
			end
		join
		quiet;
		if (rd32(16'h0384 + k * 32'h400) !== 32'h18FF_0000 + d) begin
			$display("    d=%0d: memory %h", d, rd32(16'h0384 + k * 32'h400));
			fail("18: a push overtook a newer write to its longword");
		end
	end
	mem_lat = 1;

	cinv_p(2'b11, 32'd0, 1'b1);
	dtt1 = 32'd0;

	//------------------------------------------------------------- test 19
	// Pointer 1 (logical $00040000-$0007FFFF) -> a page table at $2000; page
	// 7's descriptor is longword 3 of line $2010. Memory maps it to $9000;
	// the cache -- the line written in copyback, all four longwords dirty --
	// to $8000. The translation starts d cycles into the line's push.
	wr32(16'h4204, 32'h0000_2003);
	mem_lat = 6;
	for (d = 0; d < 40; d = d + 1) begin
		dc_en = 1'b1;
		dtt1 = 32'h0000_C020;
		repeat (2) step;
		wr32(16'h201C, 32'h0000_9003);
		for (i = 0; i < 3; i = i + 1)
			wr(32'h0000_2010 + i * 4, `AP040_SZ_L, 32'h1900_0000 + d * 16 + i);
		wr(32'h0000_201C, `AP040_SZ_L, 32'h0000_8003);
		quiet;
		if (rd32(16'h201C) !== 32'h0000_9003) fail("19: the descriptor reached memory before the push");
		dc_en = 1'b0;                        // DE clear, then CPUSHA
		dtt1 = 32'd0;
		tc = 32'h0000_8000;
		pf_req_b = 1'b1;                     // PFLUSHA: the last iteration's translation
		k = 0;
		while (!pf_done_b && k < 400) begin step; k = k + 1; end
		step;
		pf_req_b = 1'b0;
		step;
		fork
			cinv_p(2'b11, 32'd0, 1'b1);
			begin
				k = 0;
				while (!u_dmu.pb_act && k < 400) begin step; k = k + 1; end
				if (k >= 400) fail("19: CPUSHA pushed nothing (the test no longer tests)");
				repeat (d) step;
				i_addr_b = 32'h0004_7124;
				i_req_b = 1'b1;
				#0;
				k = 0;
				while (!i_pass_b && !i_flt_b && k < 400) begin step; #0; k = k + 1; end
				if (i_flt_b || k >= 400) fail("19: the instruction-side translation did not pass");
				else if (i_pa_b !== 32'h0000_8124) begin
					$display("    d=%0d: translated to %h", d, i_pa_b);
					fail("19: the walk read the descriptor before its push landed");
				end
				step;
				i_req_b = 1'b0;
			end
		join
		quiet;
		if (rd32(16'h201C) !== 32'h0000_800B)
			fail("19: memory's descriptor is not the pushed one with its U bit");
		tc = 32'd0;
	end
	mem_lat = 1;
	dc_en = 1'b1;

	//------------------------------------------------------------- test 20
	cinv_p(2'b11, 32'd0, 1'b1);
	mem_lat = 2;
	// a longword, straight through: SSW RW 0, TT 0, TM 5; WB1 valid
	v = rd32(16'h7A04);
	wr_berr(32'h0000_7A04, `AP040_SZ_L, 32'h2001_A5A5, 32'h0000_7A04);
	chk_pw(16'h0005, 32'h0000_7A04, 8'h85, {96'd0, 32'h2001_A5A5}, 1'b0, "20: a longword");
	if (rd32(16'h7A04) !== v) fail("20: the write that erred reached memory");
	// a second while it is held: dropped
	wr_berr(32'h0000_7A08, `AP040_SZ_W, 32'h0000_1234, 32'h0000_7A08);
	chk_pw(16'h0005, 32'h0000_7A04, 8'h85, {96'd0, 32'h2001_A5A5}, 1'b0, "20: a second fault replaced the first");
	pw_let_go;
	// sizes and offsets: WB1D in the lanes written (Table 8-5)
	wr_berr(32'h0000_7A11, `AP040_SZ_W, 32'h0000_BEEF, 32'h0000_7A11);
	chk_pw(16'h0045, 32'h0000_7A11, 8'hC5, {96'd0, 32'h00BE_EF00}, 1'b0, "20: a word at offset 1");
	pw_let_go;
	wr_berr(32'h0000_7A13, `AP040_SZ_W, 32'h0000_CAFE, 32'h0000_7A13);
	chk_pw(16'h0045, 32'h0000_7A13, 8'hC5, {96'd0, 32'hFE00_00CA}, 1'b0, "20: a word at offset 3");
	pw_let_go;
	wr_berr(32'h0000_7A22, `AP040_SZ_L, 32'h1122_3344, 32'h0000_7A22);
	chk_pw(16'h0005, 32'h0000_7A22, 8'h85, {96'd0, 32'h3344_1122}, 1'b0, "20: a longword at offset 2");
	pw_let_go;
	wr_berr(32'h0000_7A31, `AP040_SZ_L, 32'h5566_7788, 32'h0000_7A31);
	chk_pw(16'h0005, 32'h0000_7A31, 8'h85, {96'd0, 32'h8855_6677}, 1'b0, "20: a longword at offset 1");
	pw_let_go;
	wr_berr(32'h0000_7A33, `AP040_SZ_B, 32'h0000_00C3, 32'h0000_7A33);
	chk_pw(16'h0025, 32'h0000_7A33, 8'hA5, {96'd0, 32'h0000_00C3}, 1'b0, "20: a byte at offset 3");
	pw_let_go;
	// MOVES: to user program space, TM 1 (a data reference); to FC 3, TT 2
	c_fc_ovr = 1'b1; c_fc_val = 3'd2;
	wr_berr(32'h0000_7A40, `AP040_SZ_L, 32'h2002_0002, 32'h0000_7A40);
	chk_pw(16'h0001, 32'h0000_7A40, 8'h81, {96'd0, 32'h2002_0002}, 1'b0, "20: MOVES to FC 2");
	pw_let_go;
	c_fc_val = 3'd3;
	wr_berr(32'h0000_7A44, `AP040_SZ_L, 32'h2002_0003, 32'h0000_7A44);
	chk_pw(16'h0013, 32'h0000_7A44, 8'h93, {96'd0, 32'h2002_0003}, 1'b0, "20: MOVES to FC 3");
	pw_let_go;
	c_fc_ovr = 1'b0;
	// translated (the slot), and a write crossing a page: its bytes one bus
	// write each; the fault reports the whole write, from its first byte
	tc = 32'h0000_8000;
	pf_req_b = 1'b1;
	k = 0;
	while (!pf_done_b && k < 400) begin step; k = k + 1; end
	step;
	pf_req_b = 1'b0;
	step;
	wr_berr(32'h0000_7A48, `AP040_SZ_L, 32'h2003_7A48, 32'h0000_7A48);
	chk_pw(16'h0005, 32'h0000_7A48, 8'h85, {96'd0, 32'h2003_7A48}, 1'b0, "20: a translated longword");
	pw_let_go;
	wr_berr(32'h0000_0FFE, `AP040_SZ_L, 32'hA1B2_C3D4, 32'h0000_1000);
	chk_pw(16'h0005, 32'h0000_0FFE, 8'h85, {96'd0, 32'hC3D4_A1B2}, 1'b0, "20: a longword crossing a page");
	if (mem[16'h0FFE] !== 8'hA1 || mem[16'h0FFF] !== 8'hB2 || mem[16'h1001] !== 8'hD4)
		fail("20: a crossing write's other bytes did not land");
	// the window's snoop names each byte's own longword (caches stage E):
	// the last, in the second page
	if (sb_sla !== 30'h0000_0400) fail("20: the snoop did not name a crossing write's last byte's longword");
	pw_let_go;
	// a copyback write allocating nothing (an exception frame's) that misses
	// goes to the bus alone: its fault is reported by its logical address, a
	// double fault. Logical page $00047 is physical $8000, copyback.
	wr32(16'h201C, 32'h0000_8023);
	pf_req_b = 1'b1;
	k = 0;
	while (!pf_done_b && k < 400) begin step; k = k + 1; end
	step;
	pf_req_b = 1'b0;
	step;
	c_nalloc = 1'b1;
	wr_berr(32'h0004_7010, `AP040_SZ_L, 32'h2004_7010, 32'h0000_8010);
	c_nalloc = 1'b0;
	chk_pw(16'h0005, 32'h0004_7010, 8'h85, {96'd0, 32'h2004_7010}, 1'b1, "20: a frame's copyback write alone");
	if (u_bus.w_sla !== 30'h0001_1C04) fail("20: a copyback write alone was snooped by its physical address");
	pw_let_go;
	tc = 32'd0;
	// an exception frame's write errs behind a fault held: now a double fault
	wr_berr(32'h0000_7A50, `AP040_SZ_L, 32'h2005_7A50, 32'h0000_7A50);
	c_nalloc = 1'b1;
	wr_berr(32'h0000_7A54, `AP040_SZ_L, 32'h2005_7A54, 32'h0000_7A54);
	c_nalloc = 1'b0;
	chk_pw(16'h0005, 32'h0000_7A50, 8'h85, {96'd0, 32'h2005_7A50}, 1'b1, "20: a frame's write erring behind a fault");
	pw_let_go;
	// a fault that comes in the cycle the one held is let go: taken
	wr_berr(32'h0000_7A58, `AP040_SZ_L, 32'h2006_7A58, 32'h0000_7A58);
	wberr_arm = 1'b1; wberr_addr = 32'h0000_7A5C;
	wr(32'h0000_7A5C, `AP040_SZ_L, 32'h2006_7A5C);
	k = 0;
	while (!bb_wr_berr && k < 400) begin step; k = k + 1; end
	if (k >= 400) fail("20: the second write never erred (the test no longer tests)");
	pw_ack_b = 1'b1;
	step;
	pw_ack_b = 1'b0;
	quiet;
	chk_pw(16'h0005, 32'h0000_7A5C, 8'h85, {96'd0, 32'h2006_7A5C}, 1'b0, "20: a fault in the cycle the last goes");
	pw_let_go;
	// a push's bus error: SSW 0 (TT 0, TM 0, a longword), FA its physical
	// address, WB1S invalid, PD0-PD3 the line; CPUSH done only after it
	dtt1 = 32'h0000_C020;
	repeat (2) step;
	wr(32'h0000_7B04, `AP040_SZ_L, 32'h2007_0004);
	wr(32'h0000_7B0C, `AP040_SZ_L, 32'h2007_000C);
	quiet;
	wberr_arm = 1'b1; wberr_addr = 32'h0000_7B0C;
	cinv_p(2'b01, 32'h0000_7B00, 1'b1);
	if (!pw_pend) fail("20: CPUSH was done before its push's last write was");
	quiet;
	if (wberr_arm) begin wberr_arm = 1'b0; fail("20: the push never erred (the test no longer tests)"); end
	chk_pw(16'h0000, 32'h0000_7B0C, 8'h00, {32'h2007_000C, rd32(16'h7B08), 32'h2007_0004, rd32(16'h7B00)}, 1'b0,
	       "20: a push");
	if (rd32(16'h7B04) !== 32'h2007_0004) fail("20: the push's other longword did not land");
	pw_let_go;
	dtt1 = 32'd0;
	repeat (2) step;
	// MOVE16's line: its second longword errs; SSW TT 1, SIZE line
	c_m16 = 1'b1; c_nalloc = 1'b1;
	wberr_arm = 1'b1; wberr_addr = 32'h0000_7C04;
	for (i = 0; i < 4; i = i + 1) wr(32'h0000_7C00 + i * 4, `AP040_SZ_L, 32'h2008_0000 + i);
	quiet;
	c_m16 = 1'b0; c_nalloc = 1'b0;
	if (wberr_arm) begin wberr_arm = 1'b0; fail("20: MOVE16's write never erred (the test no longer tests)"); end
	chk_pw(16'h006D, 32'h0000_7C04, 8'hED, {32'h2008_0003, 32'h2008_0002, 32'h2008_0001, 32'h2008_0000}, 1'b0,
	       "20: MOVE16's line");
	// ...and a second MOVE16 before the fault is taken: its line is not the
	// first's, and stays out of PD0-PD3
	c_m16 = 1'b1; c_nalloc = 1'b1;
	for (i = 0; i < 4; i = i + 1) wr(32'h0000_7D00 + i * 4, `AP040_SZ_L, 32'h2009_0000 + i);
	quiet;
	c_m16 = 1'b0; c_nalloc = 1'b0;
	chk_pw(16'h006D, 32'h0000_7C04, 8'hED, {32'h2008_0003, 32'h2008_0002, 32'h2008_0001, 32'h2008_0000}, 1'b0,
	       "20: a second MOVE16 reached the first's line");
	pw_let_go;
	// its last longword erring: nothing after it is captured
	c_m16 = 1'b1; c_nalloc = 1'b1;
	wberr_arm = 1'b1; wberr_addr = 32'h0000_7C1C;
	for (i = 0; i < 4; i = i + 1) wr(32'h0000_7C10 + i * 4, `AP040_SZ_L, 32'h200A_0000 + i);
	for (i = 0; i < 4; i = i + 1) wr(32'h0000_7D10 + i * 4, `AP040_SZ_L, 32'h200B_0000 + i);
	quiet;
	c_m16 = 1'b0; c_nalloc = 1'b0;
	if (wberr_arm) begin wberr_arm = 1'b0; fail("20: MOVE16's last write never erred (the test no longer tests)"); end
	chk_pw(16'h006D, 32'h0000_7C1C, 8'hED, {32'h200A_0003, 32'h200A_0002, 32'h200A_0001, 32'h200A_0000}, 1'b0,
	       "20: MOVE16's last longword");
	pw_let_go;
	mem_lat = 1;

	//------------------------------------------------------------- test 21
	dtt1 = 32'h0000_C020;
	repeat (2) step;
	mem_lat = 2;
	for (k = 0; k < 4; k = k + 1) for (d = 0; d < 4; d = d + 1) begin
		cinv_p(2'b11, 32'd0, 1'b1);
		// set $3C: four lines, every longword dirty
		for (w = 0; w < 4; w = w + 1)
			for (i = 0; i < 4; i = i + 1)
				wr(32'h0000_03C0 + w * 32'h400 + i * 4, `AP040_SZ_L, 32'h2100_0000 + k * 32'h1000 + d * 32'h100 + w * 16 + i);
		quiet;
		// a fifth line, read from longword k; its beat d errs (0: the one asked for)
		berr_arm = 1'b1; berr_addr = 32'h0000_13C0 + ((k + d) % 4) * 4;
		lg = log_n;
		rd(32'h0000_13C0 + k * 4, `AP040_SZ_L);
		quiet;
		if (berr_arm) begin berr_arm = 1'b0; fail("21: the line read never erred (the test no longer tests)"); end
		if ((d == 0) && !rd_flt) fail("21: the read whose longword erred did not fault");
		if ((d != 0) && (rd_flt || (rd_q !== rd32(16'h13C0 + k * 4)))) fail("21: the read was not answered");
		for (i = lg; i < log_n; i = i + 1)
			if (log_write[i]) fail("21: a replaced line was written though its new line's read erred");
		for (w = 0; w < 4; w = w + 1) begin
			if (!lvalid(32'h0000_03C0 + w * 32'h400)) begin
				$display("    first %0d beat %0d: line %0d", k, d, w);
				fail("21: a replaced line did not go back");
			end
			for (i = 0; i < 4; i = i + 1)
				if (cword(32'h0000_03C0 + w * 32'h400 + i * 4) !== 32'h2100_0000 + k * 32'h1000 + d * 32'h100 + w * 16 + i)
					fail("21: a replaced line went back with a longword changed");
		end
		if (lvalid(32'h0000_13C0)) fail("21: the line whose read erred is valid");
		// dirty again, every longword: CPUSHA writes all sixteen
		lg = log_n;
		cinv_p(2'b11, 32'd0, 1'b1);
		quiet;
		t0 = 0;
		for (i = lg; i < log_n; i = i + 1) if (log_write[i]) t0 = t0 + 1;
		if (t0 != 16) begin
			$display("    first %0d beat %0d: %0d writes", k, d, t0);
			fail("21: a replaced line went back without its dirty bits");
		end
	end
	// a new line read in full: the replaced line is written after its four reads
	cinv_p(2'b11, 32'd0, 1'b1);
	for (w = 0; w < 4; w = w + 1)
		for (i = 0; i < 4; i = i + 1)
			wr(32'h0000_03C0 + w * 32'h400 + i * 4, `AP040_SZ_L, 32'h2110_0000 + w * 16 + i);
	quiet;
	lg = log_n;
	want_rd(32'h0000_13C8, `AP040_SZ_L, rd32(16'h13C8), "21: a fifth line");
	quiet;
	t0 = 0;
	for (i = lg; i < log_n; i = i + 1) if (log_write[i]) t0 = t0 + 1;
	if ((log_n != lg + 8) || log_write[lg] || log_write[lg + 1] || log_write[lg + 2] || log_write[lg + 3] || (t0 != 4))
		fail("21: a replaced line was not written after its new line's four reads");
	// A snoop on the line to be replaced, swept from before the line read
	// starts to after it has replaced the line: whenever it lands -- on the
	// line still in the cache, or out with the push engine -- the line is
	// not valid after, its dirty data never written, the others intact.
	// The line to be replaced is the counter's: named before, checked after.
	for (d = 0; d < 12; d = d + 1) begin
		cinv_p(2'b11, 32'd0, 1'b1);
		for (w = 0; w < 4; w = w + 1)
			for (i = 0; i < 4; i = i + 1)
				wr(32'h0000_03C0 + w * 32'h400 + i * 4, `AP040_SZ_L, 32'h2120_0000 + d * 32'h100 + w * 16 + i);
		quiet;
		mem_lat = 5;
		k = (u_dmu.rep + 1) % 4;                          // the way the next miss replaces
		v = {u_dmu.u_arr.tags.mem[6'h3C][k*22 +: 22], 6'h3C, 4'h0};
		berr_arm = 1'b1; berr_addr = 32'h0000_13C4;
		lg = log_n;
		t0 = 0;
		fork
			rd(32'h0000_13C0, `AP040_SZ_L);
			begin
				repeat (d) step;
				sn_addr = v; sn_req = 1'b1;
				step;
				sn_req = 1'b0;
			end
			begin
				while (!u_dmu.pb_act && t0 < 60) begin step; t0 = t0 + 1; end
				if ((t0 < 60) && (u_dmu.pb_way !== k[1:0])) begin
					$display("    snoop %0d: way %0d replaced, %0d named", d, u_dmu.pb_way, k);
					fail("21: the counter did not name the way replaced (the test no longer tests)");
				end
			end
		join
		quiet;
		if (berr_arm) begin berr_arm = 1'b0; fail("21: the snooped case's line read never erred"); end
		if (lvalid(v)) begin
			$display("    snoop %0d cycles in: line %h", d, v);
			fail("21: a line a snoop hit is valid");
		end
		for (i = lg; i < log_n; i = i + 1) if (log_write[i]) fail("21: a line a snoop hit was written");
		for (w = 0; w < 4; w = w + 1)
			if ((32'h0000_03C0 + w * 32'h400) != v) begin
				if (!lvalid(32'h0000_03C0 + w * 32'h400)) fail("21: a line the snoop missed is not valid");
				for (i = 0; i < 4; i = i + 1)
					if (cword(32'h0000_03C0 + w * 32'h400 + i * 4) !== 32'h2120_0000 + d * 32'h100 + w * 16 + i)
						fail("21: a line the snoop missed changed");
			end
	end
	cinv_p(2'b11, 32'd0, 1'b1);
	dtt1 = 32'd0;
	mem_lat = 1;

	//------------------------------------------------------------- test 22
	// write-through (untranslated, DE set), a slow bus
	cinv_p(2'b11, 32'd0, 1'b1);
	want_rd(32'h0000_7E00, `AP040_SZ_L, rd32(16'h7E00), "22: a line cached");
	quiet;
	mem_lat = 10;
	lg = log_n;
	t0 = 0;
	for (i = 0; i < 6; i = i + 1) begin
		c_addr = 32'h0000_7F00 + i * 4; c_size = `AP040_SZ_L; c_wdata = 32'h2200_0000 + i; c_wr = 1'b1;
		#0;
		k = 0;
		while (c_wr_busy_w && k < 400) begin step; k = k + 1; end
		// the first five only wait out the data cache's update of the last
		if ((i < 5) && (k > 3)) begin
			$display("    write %0d waited %0d cycles", i, k);
			fail("22: a write waited for the bus with room in the store buffer");
		end
		if (i == 5) t0 = k;
		step;
		c_wr = 1'b0;
		if (sb_sla !== ((32'h0000_7F00 + i * 4) >> 2)) fail("22: the window's snoop did not see a write as it was taken");
	end
	if (t0 < 4) begin
		$display("    the sixth write waited %0d cycles", t0);
		fail("22: a write was taken into a full store buffer");
	end
	// a read that hits, answered while writes wait
	want_rd(32'h0000_7E04, `AP040_SZ_L, rd32(16'h7E04), "22: a hit behind buffered writes");
	if (sb_empty) fail("22: the hit waited for the store buffer (or the test no longer tests)");
	// a read that misses, of a longword a buffered write holds: after it
	want_rd(32'h0000_7F14, `AP040_SZ_L, 32'h2200_0005, "22: a miss behind a buffered write to its longword");
	quiet;
	t0 = 0;
	for (i = lg; i < log_n; i = i + 1) if (log_write[i]) t0 = t0 + 1;
	if (t0 != 6) fail("22: not six writes on the bus");
	for (i = 0; i < 6; i = i + 1)
		if (!log_write[lg + i] || log_addr[lg + i] != 32'h0000_7F00 + i * 4) begin
			$display("    bus %0d: %h %b", i, log_addr[lg + i], log_write[lg + i]);
			fail("22: the writes were not on the bus first, in order");
		end
	// DE clear: a read straight through waits for the buffer too
	dc_en = 1'b0;
	repeat (2) step;
	wr(32'h0000_7F40, `AP040_SZ_L, 32'h2201_0000);
	wr(32'h0000_7F44, `AP040_SZ_L, 32'h2201_0004);
	want_rd(32'h0000_7F44, `AP040_SZ_L, 32'h2201_0004, "22: DE clear, a read behind a buffered write");
	quiet;
	dc_en = 1'b1;
	// a bus error on the second of three buffered writes: that one reported
	wberr_arm = 1'b1; wberr_addr = 32'h0000_7F54;
	wr(32'h0000_7F50, `AP040_SZ_L, 32'h2202_0000);
	wr(32'h0000_7F54, `AP040_SZ_L, 32'h2202_0004);
	wr(32'h0000_7F58, `AP040_SZ_L, 32'h2202_0008);
	quiet;
	if (wberr_arm) begin wberr_arm = 1'b0; fail("22: the buffered write never erred (the test no longer tests)"); end
	chk_pw(16'h0005, 32'h0000_7F54, 8'h85, {96'd0, 32'h2202_0004}, 1'b0, "22: a buffered write's bus error");
	if (rd32(16'h7F50) !== 32'h2202_0000 || rd32(16'h7F58) !== 32'h2202_0008)
		fail("22: the writes around the one that erred did not land");
	pw_let_go;
	// A read missing behind buffered writes: its lookup, one more once they
	// are out, then the line read -- the way replaced is the counter's two
	// on (each lookup counts, 4.1). Set $3E full and clean first.
	cinv_p(2'b11, 32'd0, 1'b1);
	mem_lat = 1;
	for (w = 0; w < 4; w = w + 1) want_rd(32'h0000_03E0 + w * 32'h400, `AP040_SZ_L, rd32(16'h03E0 + w * 32'h400), "22: set $3E");
	quiet;
	// (the bus controller takes one write at once: two or more, so some wait)
	for (d = 1; d < 4; d = d + 1) begin
		mem_lat = 6 + d * 5;
		for (i = 0; i <= d; i = i + 1) wr(32'h0000_7F80 + i * 4, `AP040_SZ_L, 32'h2203_0000 + i);
		k = (u_dmu.rep + 2) % 4;
		v = {u_dmu.u_arr.tags.mem[6'h3E][k*22 +: 22], 6'h3E, 4'h0};
		t0 = 0;
		fork
			want_rd(32'h0000_13E0 + d * 32'h400, `AP040_SZ_L, rd32(16'h13E0 + d * 32'h400), "22: a miss behind buffered writes");
			begin repeat (80) begin if (u_dmu.r_wsb) t0 = 1; step; end end
		join
		if (t0 == 0) fail("22: the miss never waited for the buffer (the test no longer tests)");
		quiet;
		if (lvalid(v)) begin
			$display("    %0d writes: way %0d's line %h still valid", d + 1, k, v);
			fail("22: a miss behind buffered writes was looked up other than twice");
		end
	end
	// ...and a copyback write that misses behind a buffered one: the same
	dtt1 = 32'h0000_C020;
	repeat (2) step;
	cinv_p(2'b11, 32'd0, 1'b1);
	mem_lat = 1;
	for (w = 0; w < 4; w = w + 1) want_rd(32'h0000_03D0 + w * 32'h400, `AP040_SZ_L, rd32(16'h03D0 + w * 32'h400), "22: set $3D");
	quiet;
	mem_lat = 10;
	c_nalloc = 1'b1;                                      // allocating nothing: to the bus, buffered --
	wr(32'h0000_7F90, `AP040_SZ_L, 32'h2204_0000);        // one with the bus controller,
	wr(32'h0000_7F94, `AP040_SZ_L, 32'h2204_0004);        // one waiting
	c_nalloc = 1'b0;
	while (u_dmu.su_v || (u_dmu.ws != 3'd0)) step;        // both out of the slot, looked up
	if (sb_empty) fail("22: the copyback writes alone are not buffered (the test no longer tests)");
	k = (u_dmu.rep + 2) % 4;
	v = {u_dmu.u_arr.tags.mem[6'h3D][k*22 +: 22], 6'h3D, 4'h0};
	t0 = 0;
	fork
		wr(32'h0000_13D0, `AP040_SZ_L, 32'h2204_13D0);    // a copyback write that misses, allocating
		begin repeat (40) begin if (u_dmu.su_wsb) t0 = 1; step; end end
	join
	if (t0 == 0) fail("22: the copyback miss never waited for the buffer (the test no longer tests)");
	quiet;
	if (lvalid(v)) begin
		$display("    way %0d's line %h still valid", k, v);
		fail("22: a copyback miss behind a buffered write was looked up other than twice");
	end
	if (cword(32'h0000_13D0) !== 32'h2204_13D0) fail("22: the copyback write that missed is not in its line");
	cinv_p(2'b11, 32'd0, 1'b1);
	dtt1 = 32'd0;
	mem_lat = 1;

	//------------------------------------------------------------- test 23
	cinv_p(2'b11, 32'd0, 1'b1);
	win_all = 1'b0;
	repeat (2) step;
	// chip RAM: cached
	want_rd(32'h0000_7A00, `AP040_SZ_L, rd32(16'h7A00), "23: chip RAM");
	quiet;
	if (!lvalid(32'h0000_7A00)) fail("23: chip RAM was not cached with the windows on");
	// ROM/IO space: never ($F0xxxx; the bench's memory aliases it)
	want_rd(32'h00F0_7A10, `AP040_SZ_L, rd32(16'h7A10), "23: ROM space");
	quiet;
	if (lvalid(32'h00F0_7A10)) fail("23: ROM space was cached");
	// ...not through a copyback TTR either: the write goes to the bus
	dtt1 = 32'h0000_C020;
	repeat (2) step;
	lg = log_n;
	wr(32'h00F0_7A20, `AP040_SZ_L, 32'h2300_7A20);
	quiet;
	if (log_n != lg + 1 || !log_write[lg] || rd32(16'h7A20) !== 32'h2300_7A20)
		fail("23: a copyback write outside the windows did not go to the bus");
	if (lvalid(32'h00F0_7A20)) fail("23: a copyback write outside the windows was cached");
	dtt1 = 32'd0;
	repeat (2) step;
	// Zorro II ($200000-$9FFFFF), Zorro III (base0 $40000000, base1 $50000000):
	// only while enabled
	for (k = 0; k < 2; k = k + 1) begin
		win_z2 = k[0]; win_z3b0 = 5'd8; win_z3e0 = k[0]; win_z3b1 = 4'd5; win_z3e1 = k[0];
		repeat (2) step;
		want_rd(32'h0020_7A30, `AP040_SZ_L, rd32(16'h7A30), "23: Zorro II");
		want_rd(32'h009F_7A40, `AP040_SZ_L, rd32(16'h7A40), "23: Zorro II, its top");
		want_rd(32'h4000_7A50, `AP040_SZ_L, rd32(16'h7A50), "23: Zorro III, base 0");
		want_rd(32'h5000_7A60, `AP040_SZ_L, rd32(16'h7A60), "23: Zorro III, base 1");
		// base 0 is 128 MB (five bits), base 1 256 MB (four)
		want_rd(32'h5800_7A68, `AP040_SZ_L, rd32(16'h7A68), "23: Zorro III, base 1's upper half");
		want_rd(32'h4800_7A58, `AP040_SZ_L, rd32(16'h7A58), "23: past Zorro III base 0");
		want_rd(32'h00A0_7A70, `AP040_SZ_L, rd32(16'h7A70), "23: past Zorro II");
		quiet;
		if (lvalid(32'h0020_7A30) != k[0] || lvalid(32'h009F_7A40) != k[0] ||
		    lvalid(32'h4000_7A50) != k[0] || lvalid(32'h5000_7A60) != k[0] ||
		    lvalid(32'h5800_7A68) != k[0]) begin
			$display("    windows %0s: %b %b %b %b %b", k[0] ? "on" : "off", lvalid(32'h0020_7A30),
			         lvalid(32'h009F_7A40), lvalid(32'h4000_7A50), lvalid(32'h5000_7A60),
			         lvalid(32'h5800_7A68));
			fail("23: a Fast RAM window's caching did not follow its enable");
		end
		if (lvalid(32'h00A0_7A70)) fail("23: the space past Zorro II was cached");
		if (lvalid(32'h4800_7A58)) fail("23: the space past Zorro III base 0 was cached");
		cinv_p(2'b11, 32'd0, 1'b1);
	end
	// the instruction side: chip RAM write-through, ROM space inhibited
	for (k = 0; k < 2; k = k + 1) begin
		i_addr_b = k[0] ? 32'h00F0_7B00 : 32'h0000_7B00;
		i_req_b = 1'b1;
		#0;
		t0 = 0;
		while (!i_pass_b && !i_flt_b && t0 < 100) begin step; #0; t0 = t0 + 1; end
		if (!i_pass_b || i_cm_b !== (k[0] ? 2'b10 : 2'b00)) begin
			$display("    fetch %h: pass %b cm %b", i_addr_b, i_pass_b, i_cm_b);
			fail("23: the instruction side's caching mode did not follow the windows");
		end
		step;
		i_req_b = 1'b0;
		step;
	end
	// ...and its peek, whose caching mode the prefetch window sends every
	// untranslated read by: the address itself, write-through or the TTR's
	// mode, then the windows -- never the mode or the address of whatever
	// entry the ATC last held. An instruction translation of page 6 (CM 10
	// in its descriptor) leaves that entry inhibited; translation then off.
	tc = 32'h0000_8000;
	repeat (4) step;
	i_addr_b = 32'h0000_6B00;
	i_req_b = 1'b1;
	#0;
	t0 = 0;
	while (!i_pass_b && !i_flt_b && t0 < 400) begin step; #0; t0 = t0 + 1; end
	if (!i_pass_b) fail("23: an instruction translation of page 6 did not pass");
	step;
	// the port's entry register loads from the lookup the clock after
	if (u_mmu.iu_ent[3:2] !== 2'b10)
		fail("23: the instruction entry the peek must not use is not an inhibited one (the test no longer tests)");
	i_req_b = 1'b0;
	step;
	tc = 32'd0;
	repeat (2) step;
	ip_addr_b = 32'h00F0_7B20;
	#1;
	if (ip_pa_b !== 32'h00F0_7B20 || ip_cm_b !== 2'b10) begin
		$display("    peek %h: pa %h cm %b", ip_addr_b, ip_pa_b, ip_cm_b);
		fail("23: the instruction side's peek cached ROM space");
	end
	ip_addr_b = 32'h0000_7B20;
	#1;
	if (ip_pa_b !== 32'h0000_7B20 || ip_cm_b !== 2'b00) begin
		$display("    peek %h: pa %h cm %b", ip_addr_b, ip_pa_b, ip_cm_b);
		fail("23: the instruction side's peek did not cache chip RAM write-through");
	end
	itt0_b = 32'h0000_C040;   // chip RAM, inhibited, by ITT0
	repeat (2) step;
	#1;
	if (ip_cm_b !== 2'b10) fail("23: the instruction side's peek ignored a TTR's caching mode");
	itt0_b = 32'd0;
	repeat (2) step;
	ip_addr_b = 32'd0;
	win_all = 1'b1; win_z2 = 1'b0; win_z3e0 = 1'b0; win_z3e1 = 1'b0;
	// a dirty line snooped
	dtt1 = 32'h0000_C020;
	repeat (2) step;
	cinv_p(2'b11, 32'd0, 1'b1);
	wr(32'h0000_7C04, `AP040_SZ_L, 32'h2301_7C04);         // copyback: the line read, dirty
	quiet;
	if (!lvalid(32'h0000_7C00)) fail("23: the copyback line is not cached (the test no longer tests)");
	wr32(16'h7C04, 32'h2302_7C04);                          // the chipset writes memory...
	sn_addr = 32'h0000_7C04; sn_req = 1'b1;                 // ...and is snooped
	step;
	sn_req = 1'b0;
	quiet;
	if (lvalid(32'h0000_7C00)) fail("23: a snooped dirty line is still valid");
	want_rd(32'h0000_7C04, `AP040_SZ_L, 32'h2302_7C04, "23: a snooped dirty line's longword");
	cinv_p(2'b11, 32'd0, 1'b1);
	quiet;
	if (rd32(16'h7C04) !== 32'h2302_7C04) fail("23: a snooped dirty line's data reached memory");
	dtt1 = 32'd0;

	repeat (20) step;
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("%0d CHECK(S) FAILED", errors);
	$finish;
end

initial begin
	#4000000;
	$display("FAIL: timed out");
	$finish;
end

endmodule
