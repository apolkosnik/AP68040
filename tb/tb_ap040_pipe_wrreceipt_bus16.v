//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24, review 16)        //
//                                                                          //
// tb_ap040_pipe_wrreceipt_bus16.v - every store once, whatever ce does     //
//                                                                          //
// With translation able to refuse a data write (TC.E, or a data TTR        //
// enabled), the write is taken TENTATIVELY -- by ap040_pipe_membus.v until //
// 2026-09-25, by the DMU (ap040_pipe_dmu.v) on the 16-bit top since: the   //
// storing instruction waits on wr_busy until the write's translation       //
// passes, and wr_busy is low for the one clock it is accepted. Those units //
// run every clock; the CPU runs under ce. A forward in a cycle with ce low //
// was an acceptance nobody saw: the instruction went on waiting, the write //
// drained, and the strobe it still held was taken as a new write. One      //
// MOVE.L D0,(A0) became four bus sub-cycles on the 16-bit top, and two     //
// longword writes on the 32-bit one. RAM ends up right, so neither the     //
// registers nor memory show it; a device register would have seen the     //
// operation twice. This bench counts the writes.                           //
//                                                                          //
// Both tops run the same program side by side -- ap040_pipe_bus16.v, with  //
// the real MMU and the 16-bit adapter, whose forward is the DMU's          //
// acceptance, and ap040_pipe_sys.v's 32-bit port, whose forward is its own //
// request -- under four clock-enable schedules: always on; pseudo-random;  //
// held low for three cycles from each forward of a write; and held low for //
// one to five cycles from each one.                                        //
// DTT0 is made transparent first, by the program itself, so every data    //
// write is tentative. The program stores by every route the pipeline has:  //
//                                                                          //
//   $0400  MOVE.L #$0000C000,D0 / MOVEC D0,DTT0                            //
//   $040A  LEA ($800).L,A0 / MOVE.L #$12345678,D0 / MOVE.L #$9ABCDEF0,D1   //
//   $041C  MOVE.L D0,(A0)           EA-fetch store, long                   //
//   $041E  MOVE.W D0,4(A0)          EA-fetch store, word                   //
//   $0422  MOVE.L #$CAFEBABE,8(A0)  EX store                               //
//   $042A  ADDQ.L #1,(A0)           EX read-modify-write                   //
//   $042C  MOVEM.L D0-D1,16(A0)     MOVEM beats                            //
//   $0432  PEA (A0)                 a push                                 //
//   $0434  MOVE.B D0,24(A0)         byte                                   //
//   $0438  MOVEQ #7,D7 / BRA.S *                                           //
//                                                                          //
// Checked per top and schedule: D7 reaches 7; the number of write          //
// transfers is exactly the program's (sixteen-bit: 14 sub-cycles;         //
// thirty-two-bit: 8 transactions); every write lands inside a store's      //
// operand; and memory holds what the program wrote.                        //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps
`include "ap040_pipe_defs.svh"
`include "ap040_defs.svh"

module tb_ap040_pipe_wrreceipt_bus16;

reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0;
reg ce16 = 1, ce32 = 1;
integer mode = 0;
integer errors = 0;

//--------------------------------------------------------------- the program
reg [15:0] prog [0:29];
initial begin
	prog[ 0] = 16'h203C; prog[ 1] = 16'h0000; prog[ 2] = 16'hC000;   // MOVE.L #$C000,D0
	prog[ 3] = 16'h4E7B; prog[ 4] = 16'h0006;                        // MOVEC D0,DTT0
	prog[ 5] = 16'h41F9; prog[ 6] = 16'h0000; prog[ 7] = 16'h0800;   // LEA ($800).L,A0
	prog[ 8] = 16'h203C; prog[ 9] = 16'h1234; prog[10] = 16'h5678;   // MOVE.L #$12345678,D0
	prog[11] = 16'h223C; prog[12] = 16'h9ABC; prog[13] = 16'hDEF0;   // MOVE.L #$9ABCDEF0,D1
	prog[14] = 16'h2080;                                             // MOVE.L D0,(A0)
	prog[15] = 16'h3140; prog[16] = 16'h0004;                        // MOVE.W D0,4(A0)
	prog[17] = 16'h217C; prog[18] = 16'hCAFE; prog[19] = 16'hBABE; prog[20] = 16'h0008;  // MOVE.L #$CAFEBABE,8(A0)
	prog[21] = 16'h5290;                                             // ADDQ.L #1,(A0)
	prog[22] = 16'h48E8; prog[23] = 16'h0003; prog[24] = 16'h0010;   // MOVEM.L D0-D1,16(A0)
	prog[25] = 16'h4850;                                             // PEA (A0)
	prog[26] = 16'h1140; prog[27] = 16'h0018;                        // MOVE.B D0,24(A0)
	prog[28] = 16'h7E07;                                             // MOVEQ #7,D7
	prog[29] = 16'h60FE;                                             // BRA.S *
end

// A write is legitimate if it lies inside a store's operand.
function in_operand;
	input [31:0] a;
	begin
		in_operand = (a >= 32'h800 && a < 32'h81C) ||   // the stores through A0
		             (a >= 32'hFFC && a < 32'h1000);     // PEA's push below ISP $1000
	end
endfunction

//------------------------------------------------ the clock-enable schedules
reg [15:0] lfsr16 = 16'hACE1, lfsr32 = 16'hBEEF, plen = 16'h1D2B;
integer    hold16 = 0, hold32 = 0;
wire       fwd16, fwd32;           // the forward of a write, on each top
reg        fwd16_q = 0, fwd32_q = 0;
always @(negedge clk) begin
	fwd16_q <= fwd16;
	fwd32_q <= fwd32;
	plen    <= {plen[14:0], plen[15] ^ plen[13] ^ plen[12] ^ plen[10]};
	if (!nreset) begin
		ce16 <= 1; ce32 <= 1; hold16 = 0; hold32 = 0;
	end else case (mode)
	0: begin ce16 <= 1; ce32 <= 1; end
	1: begin
		lfsr16 <= {lfsr16[14:0], lfsr16[15] ^ lfsr16[13] ^ lfsr16[12] ^ lfsr16[10]};
		lfsr32 <= {lfsr32[14:0], lfsr32[15] ^ lfsr32[13] ^ lfsr32[12] ^ lfsr32[10]};
		ce16 <= lfsr16[0];
		ce32 <= lfsr32[0];
	end
	default: begin
		// low from each forward for 3 cycles (mode 2) or 1-5 (mode 3)
		if (fwd16 && !fwd16_q) hold16 = (mode == 2) ? 3 : 1 + (plen % 5);
		if (fwd32 && !fwd32_q) hold32 = (mode == 2) ? 3 : 1 + ((plen >> 4) % 5);
		ce16 <= (hold16 == 0);
		ce32 <= (hold32 == 0);
		if (hold16 > 0) hold16 = hold16 - 1;
		if (hold32 > 0) hold32 = hold32 - 1;
	end
	endcase
end

//------------------------------------------------------------ the 16-bit top
wire [31:0] addr_out;
wire [15:0] data_write;
wire        nwr, nuds, nlds, longword;
wire  [1:0] busstate;
wire  [2:0] fc;
wire [31:0] d7_16;
reg         mem_ready;
wire        clkena_in = (busstate == `AP040_BUS_IDLE) | mem_ready;
reg  [15:0] mem16 [0:4095];
wire [15:0] data_in = mem16[addr_out[12:1]];

ap040_pipe_bus16 #(.PC_RESET(32'h400), .PROG_WORDS(32'h7FFF_FFFF), .RESET_VECTORS(1)) u16
(
	.clk (clk), .nreset (nreset), .ce (ce16), .irq_lvl (3'd0), .clkena_in (clkena_in),
	.berr (1'b0),   // no bus errors: the forward is the MMU's, not a fault
	.cache_allow_all (1'b1), .cache_z2_ena (1'b0), .cache_z3_base0 (5'd0), .cache_z3_ena0 (1'b0),
	.cache_z3_base1 (4'd0), .cache_z3_ena1 (1'b0), .snoop_stb (1'b0), .snoop_addr (32'd0),
	.walker_req (), .walker_we (), .walker_addr (), .walker_wdat (),
	.walker_ack (1'b0), .walker_data (32'd0), .walker_berr (1'b0),   // DTT0 only: no walks
	.data_in (data_in), .addr_out (addr_out), .data_write (data_write),
	.nwr (nwr), .nuds (nuds), .nlds (nlds), .busstate (busstate), .longword (longword), .fc (fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid (), .dbg_eac_pc (), .dbg_eaf_valid (), .dbg_eaf_pc (),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0 (), .dbg_d1 (), .dbg_d2 (), .dbg_d3 (), .dbg_d4 (), .dbg_d5 (), .dbg_d6 (), .dbg_d7 (d7_16),
	.dbg_ccr (), .dbg_sr (), .dbg_commits ()
);
// The acceptance: wr_busy low for the tentative write, its translation passed.
assign fwd16 = u16.u_dmu.w_acc;

integer w16, stray16;
reg [1:0] dly16;
always @(posedge clk) begin
	if (!nreset) begin
		mem_ready <= 1'b0; dly16 <= 2'd0;
	end else begin
		mem_ready <= 1'b0;
		if (busstate != `AP040_BUS_IDLE && !mem_ready) begin
			// answer after 0-3 cycles
			if (dly16 == 2'd0) begin
				mem_ready <= 1'b1;
				dly16     <= lfsr16[3:2];
				if (busstate == `AP040_BUS_WRITE) begin
					w16 = w16 + 1;
					if (!in_operand(addr_out)) begin
						stray16 = stray16 + 1;
						$display("FAIL: 16-bit mode %0d: stray write %h <- %h", mode, addr_out, data_write);
					end
					if (!nuds) mem16[addr_out[12:1]][15:8] <= data_write[15:8];
					if (!nlds) mem16[addr_out[12:1]][7:0]  <= data_write[7:0];
				end
			end else dly16 <= dly16 - 2'd1;
		end
	end
end

//------------------------------------------------------------ the 32-bit top
wire        m_req, m_write, m_instr;
wire  [1:0] m_size;
wire [31:0] m_addr, m_wdata;
wire  [2:0] m_fc;
reg         m_ack;
reg  [31:0] m_rdata;
wire [31:0] d7_32;
reg   [7:0] mem32 [0:8191];

ap040_pipe_sys #(.PC_RESET(32'h400), .PROG_WORDS(32'h7FFF_FFFF), .RESET_VECTORS(1)) u32
(
	.clk (clk), .nreset (nreset), .ce (ce32), .irq_lvl (3'd0),
	.mem_req (m_req), .mem_write (m_write), .mem_instr (m_instr), .mem_size (m_size),
	.mem_addr (m_addr), .mem_wdata (m_wdata), .mem_fc (m_fc), .mem_ack (m_ack), .mem_rdata (m_rdata),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid (), .dbg_eac_pc (), .dbg_eaf_valid (), .dbg_eaf_pc (),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0 (), .dbg_d1 (), .dbg_d2 (), .dbg_d3 (), .dbg_d4 (), .dbg_d5 (), .dbg_d6 (), .dbg_d7 (d7_32),
	.dbg_ccr (), .dbg_sr (), .dbg_commits ()
);
assign fwd32 = m_req && m_write;

// Right-aligned by size, both ways (ap040_pipe_membus.v's contract).
function [31:0] rd32;
	input [31:0] a;
	input  [1:0] sz;
	begin
		rd32 = (sz == `AP040_SZ_B) ? {24'd0, mem32[a[12:0]]} :
		       (sz == `AP040_SZ_W) ? {16'd0, mem32[a[12:0]], mem32[a[12:0] + 13'd1]} :
		       {mem32[a[12:0]], mem32[a[12:0] + 13'd1], mem32[a[12:0] + 13'd2], mem32[a[12:0] + 13'd3]};
	end
endfunction
integer w32, stray32;
reg [1:0] dly32;
reg       busy32;
always @(posedge clk) begin
	if (!nreset) begin
		m_ack <= 1'b0; dly32 <= 2'd0; busy32 <= 1'b0;
	end else begin
		m_ack <= 1'b0;
		if (m_req && !m_ack) begin
			if (dly32 == 2'd0) begin
				m_ack   <= 1'b1;
				dly32   <= lfsr32[5:4];
				m_rdata <= rd32(m_addr, m_size);
				if (m_write) begin
					w32 = w32 + 1;
					if (!in_operand(m_addr)) begin
						stray32 = stray32 + 1;
						$display("FAIL: 32-bit mode %0d: stray write %h <- %h", mode, m_addr, m_wdata);
					end
					case (m_size)
					`AP040_SZ_B: mem32[m_addr[12:0]] <= m_wdata[7:0];
					`AP040_SZ_W: begin mem32[m_addr[12:0]] <= m_wdata[15:8]; mem32[m_addr[12:0] + 13'd1] <= m_wdata[7:0]; end
					default: begin
						mem32[m_addr[12:0]]         <= m_wdata[31:24];
						mem32[m_addr[12:0] + 13'd1] <= m_wdata[23:16];
						mem32[m_addr[12:0] + 13'd2] <= m_wdata[15:8];
						mem32[m_addr[12:0] + 13'd3] <= m_wdata[7:0];
					end
					endcase
				end
			end else dly32 <= dly32 - 2'd1;
		end
	end
end

//------------------------------------------------------------------- checks
function [31:0] l16;
	input [31:0] a;
	begin l16 = {mem16[a[12:1]], mem16[a[12:1] + 12'd1]}; end
endfunction
function [31:0] l32;
	input [31:0] a;
	begin l32 = {mem32[a[12:0]], mem32[a[12:0] + 13'd1], mem32[a[12:0] + 13'd2], mem32[a[12:0] + 13'd3]}; end
endfunction

task check_mem;
	input integer top;
	input [31:0] a, want, mask;
	reg   [31:0] got;
	begin
		got = (top == 16) ? l16(a) : l32(a);
		if ((got & mask) !== (want & mask)) begin
			errors = errors + 1;
			$display("FAIL: %0d-bit mode %0d: ($%h) = %h, want %h (mask %h)", top, mode, a, got, want, mask);
		end
	end
endtask

integer i, t;
initial begin
	for (mode = 0; mode < 4; mode = mode + 1) begin
		nreset = 0;
		for (i = 0; i < 4096; i = i + 1) mem16[i] = 16'h4E71;   // NOP everywhere else
		for (i = 0; i < 8192; i = i + 1) mem32[i] = (i & 1) ? 8'h71 : 8'h4E;
		for (i = 0; i < 16'h40; i = i + 1) mem16[16'h400 + i] = 16'h0000;   // the data area, $800-$87F
		for (i = 16'h800; i < 16'h880; i = i + 1) mem32[i] = 8'h00;
		// reset vectors: ISP $1000, PC $400
		mem16[0] = 16'h0000; mem16[1] = 16'h1000; mem16[2] = 16'h0000; mem16[3] = 16'h0400;
		mem32[0] = 8'h00; mem32[1] = 8'h00; mem32[2] = 8'h10; mem32[3] = 8'h00;
		mem32[4] = 8'h00; mem32[5] = 8'h00; mem32[6] = 8'h04; mem32[7] = 8'h00;
		for (i = 0; i < 30; i = i + 1) begin
			mem16[16'h200 + i] = prog[i];
			mem32[16'h400 + 2*i]     = prog[i][15:8];
			mem32[16'h400 + 2*i + 1] = prog[i][7:0];
		end
		w16 = 0; stray16 = 0; w32 = 0; stray32 = 0;
		repeat (5) @(posedge clk);
		nreset = 1;
		t = 0;
		while (t < 4000 && !(d7_16 === 32'd7 && d7_32 === 32'd7)) begin
			@(posedge clk);
			t = t + 1;
		end
		repeat (200) @(posedge clk);   // anything still posted drains
		if (d7_16 !== 32'd7) begin errors = errors + 1; $display("FAIL: 16-bit mode %0d: D7 = %h, the program did not finish", mode, d7_16); end
		if (d7_32 !== 32'd7) begin errors = errors + 1; $display("FAIL: 32-bit mode %0d: D7 = %h, the program did not finish", mode, d7_32); end
		if (w16 != 14) begin errors = errors + 1; $display("FAIL: 16-bit mode %0d: %0d write sub-cycles, want 14", mode, w16); end
		if (w32 != 8)  begin errors = errors + 1; $display("FAIL: 32-bit mode %0d: %0d write transactions, want 8", mode, w32); end
		errors = errors + stray16 + stray32;
		for (i = 16; i <= 32; i = i + 16) begin
			check_mem(i, 32'h800, 32'h12345679, 32'hFFFFFFFF);   // MOVE.L, then ADDQ
			check_mem(i, 32'h804, 32'h56780000, 32'hFFFF0000);   // MOVE.W
			check_mem(i, 32'h808, 32'hCAFEBABE, 32'hFFFFFFFF);   // the EX store
			check_mem(i, 32'h810, 32'h12345678, 32'hFFFFFFFF);   // MOVEM D0
			check_mem(i, 32'h814, 32'h9ABCDEF0, 32'hFFFFFFFF);   // MOVEM D1
			check_mem(i, 32'h818, 32'h78000000, 32'hFF000000);   // MOVE.B
			check_mem(i, 32'hFFC, 32'h00000800, 32'hFFFFFFFF);   // PEA
		end
		$display("mode %0d: 16-bit %0d write sub-cycles, 32-bit %0d write transactions", mode, w16, w32);
	end
	if (errors == 0) $display("ALL TESTS PASSED");
	else             $display("TEST FAILED with %0d errors", errors);
	$finish;
end

endmodule
