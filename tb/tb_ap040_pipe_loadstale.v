//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 101: what a load  //
// inherits)                                                                //
//                                                                          //
// tb_ap040_pipe_loadstale.v - a load behind each instruction whose flag    //
// the load path forgot                                                     //
//                                                                          //
// EA-fetch retires an instruction down two different paths: the general    //
// one, and mem_complete for anything that waited on a load. Each assigns   //
// the stage's output registers, and a field one of them leaves alone keeps //
// the PREVIOUS instruction's value -- which the next instruction then      //
// carries into EX as though it were its own.                               //
//                                                                          //
// Eight fields were stale on the load path. A review found three of them;  //
// the rest came out of diffing the two branches' assignments, which is the //
// only way to know there is not a ninth. Four are visible from a program:  //
//                                                                          //
//   LINK A6,#-4     ; MOVE.L (A0),D0   D0 must take $12345678, and the     //
//                                      load must not be treated as a LINK  //
//   PEA (A1)        ; MOVE.L (A0),D1   the same, for a PEA                 //
//   ORI #$0000,SR   ; MOVE.L (A2),D2   the load must not write the status  //
//                                      register; $4000 would set T1        //
//   ORI #$0000,CCR  ; MOVE.L (A3),D3   nor the condition codes; $0010      //
//                                      would set X                         //
//                                                                          //
// The status register is checked whole at the end, so tracing turned on by //
// the third pair would show up even though nothing branches on it, and X   //
// set by the fourth would too.                                             //
//                                                                          //
// The other four -- the CHK and TRAPcc flags and MOVEC's direction and     //
// selector -- are fixed alongside and have no program that reaches them:   //
// their consumers in EX are gated on flags this stage also clears. They    //
// are listed in the plan rather than left implied.                         //
//--------------------------------------------------------------------------//

`timescale 1ns/1ps

module tb_ap040_pipe_loadstale;

localparam PROG_WORDS      = 60;
localparam [31:0] PC_RESET = 32'h0000_0400;

reg clk = 0;
reg nreset = 0;
reg ce = 1;

always #5 clk = ~clk;

`ifdef AP040_PIPE_CE_RANDOM
reg [15:0] ce_lfsr = 16'hACE1;
always @(negedge clk) if (nreset) begin
	ce_lfsr <= {ce_lfsr[14:0], ce_lfsr[15] ^ ce_lfsr[13] ^ ce_lfsr[12] ^ ce_lfsr[10]};
	ce      <= ce_lfsr[0];
end
`endif

wire        dbg_if_valid,  dbg_id_valid,  dbg_eac_valid;
wire        dbg_eaf_valid, dbg_ex_valid,  dbg_wb_valid;
wire [31:0] dbg_if_pc,     dbg_id_pc,     dbg_eac_pc;
wire [31:0] dbg_eaf_pc,    dbg_ex_pc,     dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4;
wire [15:0] dbg_sr;
wire  [4:0] dbg_ccr;

ap040_pipe_core #(
	.PC_RESET  (PC_RESET),
	.PROG_WORDS(PROG_WORDS)
) dut
(
	.irq_lvl (3'd0),   // no interrupt source in this bench
	.clk (clk),
	.nreset (nreset),
	.ce  (ce),

	.dbg_if_valid (dbg_if_valid),  .dbg_if_pc (dbg_if_pc),
	.dbg_id_valid (dbg_id_valid),  .dbg_id_pc (dbg_id_pc),
	.dbg_eac_valid(dbg_eac_valid), .dbg_eac_pc(dbg_eac_pc),
	.dbg_eaf_valid(dbg_eaf_valid), .dbg_eaf_pc(dbg_eaf_pc),
	.dbg_ex_valid (dbg_ex_valid),  .dbg_ex_pc (dbg_ex_pc),
	.dbg_wb_valid (dbg_wb_valid),  .dbg_wb_pc (dbg_wb_pc),

	.dbg_d0 (dbg_d0), .dbg_d1 (dbg_d1), .dbg_d2 (dbg_d2), .dbg_d3 (dbg_d3),
	.dbg_d4 (dbg_d4), .dbg_sr (dbg_sr),
	.dbg_ccr(dbg_ccr)
);

integer errors = 0;

integer writes = 0;
always @(posedge clk)
	if (nreset && dut.u_l1.wren_b && !dut.u_l1.wr_busy)
		writes = writes + 1;

initial begin
	#1;
	dut.u_l1.mem[ 0] = 16'h4E71;   // NOP
	dut.u_l1.mem[ 1] = 16'h203C;   // MOVE.L #$00000600,D0
	dut.u_l1.mem[ 2] = 16'h0000;
	dut.u_l1.mem[ 3] = 16'h0600;
	dut.u_l1.mem[ 4] = 16'h4E7B;   // MOVEC D0,ISP   (A7 = $0600)
	dut.u_l1.mem[ 5] = 16'h0804;
	dut.u_l1.mem[ 6] = 16'h207C;   // MOVEA.L #$00000800,A0
	dut.u_l1.mem[ 7] = 16'h0000;
	dut.u_l1.mem[ 8] = 16'h0800;
	dut.u_l1.mem[ 9] = 16'h227C;   // MOVEA.L #$00000804,A1
	dut.u_l1.mem[10] = 16'h0000;
	dut.u_l1.mem[11] = 16'h0804;
	dut.u_l1.mem[12] = 16'h247C;   // MOVEA.L #$00000808,A2
	dut.u_l1.mem[13] = 16'h0000;
	dut.u_l1.mem[14] = 16'h0808;
	dut.u_l1.mem[15] = 16'h267C;   // MOVEA.L #$0000080C,A3
	dut.u_l1.mem[16] = 16'h0000;
	dut.u_l1.mem[17] = 16'h080C;
	dut.u_l1.mem[18] = 16'h2C7C;   // MOVEA.L #$00000900,A6
	dut.u_l1.mem[19] = 16'h0000;
	dut.u_l1.mem[20] = 16'h0900;

	// ---- a load behind a LINK
	dut.u_l1.mem[21] = 16'h4E56;   // LINK A6,#-4
	dut.u_l1.mem[22] = 16'hFFFC;
	dut.u_l1.mem[23] = 16'h2010;   // MOVE.L (A0),D0   -> $12345678

	// ---- a load behind a PEA
	dut.u_l1.mem[24] = 16'h4851;   // PEA (A1)
	dut.u_l1.mem[25] = 16'h2211;   // MOVE.L (A1),D1   -> $9ABCDEF0

	// ---- a load behind an ORI to SR
	dut.u_l1.mem[26] = 16'h007C;   // ORI #$0000,SR   (changes nothing)
	dut.u_l1.mem[27] = 16'h0000;
	dut.u_l1.mem[28] = 16'h2412;   // MOVE.L (A2),D2   -> $00004000

	// ---- a load behind an ORI to CCR
	dut.u_l1.mem[29] = 16'h003C;   // ORI #$0000,CCR  (changes nothing)
	dut.u_l1.mem[30] = 16'h0000;
	dut.u_l1.mem[31] = 16'h2613;   // MOVE.L (A3),D3   -> $00000010
	dut.u_l1.mem[32] = 16'h60FE;   // BRA.B -2, to itself

	dut.u_l1.mem[512] = 16'h1234;  // $0800
	dut.u_l1.mem[513] = 16'h5678;
	dut.u_l1.mem[514] = 16'h9ABC;  // $0804
	dut.u_l1.mem[515] = 16'hDEF0;
	dut.u_l1.mem[516] = 16'h0000;  // $0808 -- as an SR write this sets T1
	dut.u_l1.mem[517] = 16'h4000;
	dut.u_l1.mem[518] = 16'h0000;  // $080C -- as a CCR write this sets X
	dut.u_l1.mem[519] = 16'h0010;
end

initial begin
	nreset = 0;
	repeat (2) @(posedge clk);
	nreset = 1;

	repeat ((PROG_WORDS + 140) * `AP040_PIPE_WAIT_SCALE) @(posedge clk);

	if (dbg_d0 !== 32'h1234_5678) begin
		errors = errors + 1;
		$display("FAIL: D0 = %h, expected 12345678. The load behind a LINK inherited eaf_is_link and was retired as one, so it wrote no register.",
		         dbg_d0);
	end
	if (dbg_d1 !== 32'h9ABC_DEF0) begin
		errors = errors + 1;
		$display("FAIL: D1 = %h, expected 9abcdef0. The load behind a PEA inherited eaf_is_pea.", dbg_d1);
	end
	if (dbg_d2 !== 32'h0000_4000) begin
		errors = errors + 1;
		$display("FAIL: D2 = %h, expected 00004000", dbg_d2);
	end
	if (dbg_d3 !== 32'h0000_0010) begin
		errors = errors + 1;
		$display("FAIL: D3 = %h, expected 00000010", dbg_d3);
	end
	if (dbg_sr !== 16'h2700) begin
		errors = errors + 1;
		$display("FAIL: SR = %h, expected 2700. A load behind an ORI to SR or CCR inherited eaf_is_immsr and wrote the status register with what it had loaded: a7xx means the $4000 turned tracing on, and a low bit set means the $0010 set X.",
		         dbg_sr);
	end
	if (dut.u_cpu.u_regfile.areg[6] !== 32'h0000_05FC) begin
		errors = errors + 1;
		$display("FAIL: A6 = %h, expected 000005fc (the LINK itself must still work)", dut.u_cpu.u_regfile.areg[6]);
	end

	if (dbg_if_valid || dbg_id_valid || dbg_eac_valid ||
	    dbg_eaf_valid || dbg_ex_valid || dbg_wb_valid) begin
		errors = errors + 1;
		$display("FAIL: a stage is still valid after the program should have drained");
	end

	if (errors == 0)
		$display("ALL TESTS PASSED");
	else
		$display("%0d CHECK(S) FAILED", errors);

	$finish;
end

endmodule
