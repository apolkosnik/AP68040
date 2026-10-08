//--------------------------------------------------------------------------//
// AP68040-60 - pipelined MC68040                                            //
//                                                                          //
// tb_fpu_pipe.sv - the pipelined FPU against the sequential one            //
//                                                                          //
// Two ap040_fpu instances get the same random command stream: REF with    //
// the pipeline off (PIPE=0, the corpus-validated sequential FPU), DUT     //
// with it on.  Each instance takes its commands as fast as it accepts    //
// them (the pipelined one overlaps them); at random sync points both are  //
// drained and the eight registers and FPSR must be identical.  The       //
// stream: FMOVE/FABS/FNEG/FADD/FSUB/FMUL in all rounding precisions (the pipeline's //
// operations) from registers and from single/double/extended memory      //
// operands and long/word/byte integers, mixed with operations the pipeline leaves to the sequential  //
// FPU (FDIV FSQRT FCMP FTST FSGLMUL FSGLDIV), on values that are mostly //
// ordinary but include zeros, huge and tiny exponents, infinities, NaNs   //
// and denormals; FPCR rounding mode and precision change between         //
// segments, and some segments enable exceptions.                         //
//                                                                          //
//   +seed=<n>   random seed (default 1)   +cmds=<n>  commands (20000)     //
//   +pipeonly   only the pipeline's operations, on ordinary values, few  //
//               sync points: back-to-back issue, hazards and forwarding  //
//--------------------------------------------------------------------------//
`timescale 1ns/1ps

module tb_fpu_pipe;

logic clk = 0;
always #5 clk = ~clk;
logic nreset = 0;

// one command
typedef struct {
	int          kind;          // 0 op, 1 FPCR write, 2 register load
	logic  [2:0] cls;
	logic  [6:0] opm;
	logic  [2:0] fmt;
	logic  [2:0] sr, dr;
	logic [95:0] din;
	logic [31:0] cr;
} cmd_t;

cmd_t cmds [$];

//--------------------------------------------------------------------------
// two FPUs
//--------------------------------------------------------------------------
`define FPU_PORTS(P) \
	logic        P``_req = 0; \
	logic  [2:0] P``_cls = 0, P``_fmt = 0, P``_sr = 0, P``_dr = 0; \
	logic  [6:0] P``_opm = 0; \
	logic [95:0] P``_din = 0; \
	logic        P``_done, P``_pdone, P``_acc, P``_wbok, P``_unimp, P``_unsupp, P``_exc; \
	logic  [7:0] P``_vec; \
	logic [95:0] P``_dout, P``_fmrd; \
	logic  [3:0] P``_cc; \
	logic  [1:0] P``_crsel = 2'd1; \
	logic        P``_crwe = 0; \
	logic [31:0] P``_crwd = 0, P``_crrd; \
	logic  [2:0] P``_fmsel = 0; \
	logic        P``_fmwe = 0; \
	logic [95:0] P``_fmwd = 0; \
	logic        P``_pbusy;

`FPU_PORTS(r)
`FPU_PORTS(d)

`define FPU_INST(P, PIPEV) \
	ap040_fpu #(.PIPE(PIPEV)) P``_fpu ( \
		.clk(clk), .nreset(nreset), .ce(1'b1), \
		.req(P``_req), .op_class(P``_cls), .opmode(P``_opm), .wb_ok(P``_wbok), \
		.src_fmt(P``_fmt), .src_r(P``_fmwe ? P``_fmsel : P``_sr), .dst_r(P``_dr), \
		.din(P``_din), .done(P``_done), .accepted(P``_acc), \
		.unimp(P``_unimp), .unsupp(P``_unsupp), \
		.exc_req(P``_exc), .exc_vec(P``_vec), .dout(P``_dout), \
		.fpcc(P``_cc), \
		.cr_sel(P``_crsel), .cr_we(P``_crwe), .cr_wdata(P``_crwd), .cr_rdata(P``_crrd), \
		.bsun_req(1'b0), .bsun_enable(), \
		.ia_we(1'b0), .ia_wdata(32'd0), \
		.fm_sel(P``_fmsel), .fm_we(P``_fmwe), .fm_wdata(P``_fmwd), .fm_rdata(P``_fmrd), \
		.fpu_used(), .pbusy(P``_pbusy), .pdone(P``_pdone), \
		.fstate_unimp(), .fstate_cmd1(), .fstate_cmd3(), \
		.fstate_stag(), .fstate_dtag(), .fstate_flags(), \
		.fstate_fpt(), .fstate_et(), \
		.fsave_ack(1'b0), .frestore_idle(1'b0), .frestore_unimp(1'b0), \
		.pend_capture(P``_pcap), .cur_vec(), \
		.frestore_e1_pend(), .frestore_resume(), \
		.frestore_cusavepc(8'd0), .frestore_et15(1'b0), .frestore_fpt15(1'b0), \
		.fstate_grs(), .fstate_wbte15(), \
		.fstate_busy(), .fstate_wbt(), .fstate_fpiar_c(), \
		.frestore_wbt(96'd0), .frestore_fpiar(32'd0), .frestore_busy(1'b0), \
		.frestore_cmd1(16'd0), .frestore_cmd3(16'd0), \
		.frestore_stag(3'd0), .frestore_dtag(3'd0), .frestore_flags(3'd0), \
		.frestore_fpt(96'd0), .frestore_et(96'd0), \
		.frestore_grs(3'd0), .frestore_wbte15(1'b0), \
		.fp_reset(1'b0) \
	);

// an exception's pending frame is consumed as the core does (FSAVE or the
// next instruction's CHK delivers it): pend_capture right after it
logic r_pcap = 0, d_pcap = 0;
// completions (done, pdone, or an exception outcome), counted at the clock
// so a one-cycle pdone is not missed; lastx: the last one was an exception
int   r_nend = 0, d_nend = 0;
logic r_lastx = 0, d_lastx = 0;
always @(posedge clk) begin
	if (r_done || r_pdone || r_unimp || r_unsupp || r_exc) begin
		r_nend <= r_nend + 1; r_lastx <= r_unimp || r_unsupp || r_exc;
	end
	if (d_done || d_pdone || d_unimp || d_unsupp || d_exc) begin
		d_nend <= d_nend + 1; d_lastx <= d_unimp || d_unsupp || d_exc;
	end
end
`FPU_INST(r, 0)
`FPU_INST(d, 1)

//--------------------------------------------------------------------------
// values
//--------------------------------------------------------------------------
int unsigned seed;
int pipeonly = 0;

function automatic logic [79:0] rnd_x(int unsigned r);
	logic        s;
	logic [14:0] e;
	logic [63:0] m;
	int          k;
	s = $urandom() & 1;
	m = {$urandom(), $urandom()};
	k = pipeonly ? $urandom_range(0, 77) : $urandom_range(0, 99);
	if (k < 70) begin                       // ordinary: |exponent| <= 60
		e = 15'(16383 + $urandom_range(0, 120) - 60);
		m[63] = 1'b1;
		// short significands now and then (exact results, ties)
		if ($urandom_range(0, 3) == 0) m[39:0] = 40'd0;
		if ($urandom_range(0, 7) == 0) m[62:0] = 63'd0;
	end
	else if (k < 78) begin                  // zero
		e = 15'd0; m = 64'd0;
	end
	else if (k < 86) begin                  // larger exponents
		e = 15'(16383 + $urandom_range(0, 2000) - 1000);
		m[63] = 1'b1;
	end
	else if (k < 90) begin                  // near the extended range ends
		e = $urandom_range(0, 1) ? 15'($urandom_range(32700, 32766)) : 15'($urandom_range(1, 60));
		m[63] = 1'b1;
	end
	else if (k < 93) begin                  // infinity
		e = 15'h7FFF; m = 64'd0;
	end
	else if (k < 96) begin                  // NaN (quiet or signaling)
		e = 15'h7FFF; m[63] = 1'b1; if (m[62:0] == 0) m[0] = 1'b1;
	end
	else begin                              // denormal
		e = 15'd0; m[63] = 1'b0; if (m == 0) m[0] = 1'b1;
	end
	rnd_x = {s, e, m};
endfunction

// the 96-bit extended memory image of {s, e, m}
function automatic logic [95:0] xmem(logic [79:0] x);
	xmem = {x[79:64], 16'd0, x[63:0]};
endfunction

// a memory operand: single, double, extended or long, left aligned
function automatic logic [95:0] rnd_mem(logic [2:0] fmt);
	logic [79:0] x;
	logic [31:0] w;
	int          k;
	case (fmt)
		3'd1: begin
			k = $urandom_range(0, 99);
			if (k < 80) w = {1'($urandom() & 1), 8'(127 + $urandom_range(0, 120) - 60), 23'($urandom())};
			else if (k < 88) w = {1'($urandom() & 1), 31'd0};
			else w = $urandom();
			rnd_mem = {w, 64'd0};
		end
		3'd5: begin
			logic [63:0] q;
			k = $urandom_range(0, 99);
			if (k < 80) q = {1'($urandom() & 1), 11'(1023 + $urandom_range(0, 120) - 60), 52'({$urandom(), $urandom()})};
			else if (k < 88) q = {1'($urandom() & 1), 63'd0};
			else q = {$urandom(), $urandom()};
			rnd_mem = {q, 32'd0};
		end
		3'd2: begin
			x = rnd_x(0);
			rnd_mem = {x[79:64], 16'd0, x[63:0]};
		end
		3'd4: rnd_mem = {16'($urandom_range(0, 1) ? $urandom() : $urandom_range(0, 300)), 80'd0};
		3'd6: rnd_mem = {8'($urandom()), 88'd0};
		default: rnd_mem = {32'($urandom_range(0, 1) ? $urandom() : $urandom_range(0, 1000)), 64'd0};
	endcase
endfunction

//--------------------------------------------------------------------------
// the command stream
//--------------------------------------------------------------------------
logic [6:0] pipe_ops [18] = '{7'h00, 7'h40, 7'h44, 7'h22, 7'h62, 7'h66,
                              7'h28, 7'h68, 7'h6C, 7'h23, 7'h63, 7'h67,
                              7'h18, 7'h58, 7'h5C, 7'h1A, 7'h5A, 7'h5E};
logic [6:0] seq_ops  [6]  = '{7'h20, 7'h04, 7'h38, 7'h3A, 7'h27, 7'h24};
logic [2:0] fmts     [6]  = '{3'd1, 3'd5, 3'd2, 3'd0, 3'd4, 3'd6};

int sync_at [$];         // the command index after which both are compared

task automatic gen(int n);
	cmd_t c;
	int   left;
	left = 0;
	for (int i = 0; i < 8; i++) begin
		c.kind = 2; c.dr = 3'(i); c.din = xmem(rnd_x(0));
		cmds.push_back(c);
	end
	sync_at.push_back(cmds.size());
	while (cmds.size() < n) begin
		int k;
		k = $urandom_range(0, 99);
		if (k < 2 && !pipeonly) begin
			c.kind = 1;
			c.cr = {16'd0, ($urandom_range(0, 9) == 0) ? 8'($urandom()) : 8'd0,
			        2'($urandom_range(0, 2)), 2'($urandom()), 4'd0};
			cmds.push_back(c);
		end
		else if (k < 14) begin
			c.kind = 2; c.dr = 3'($urandom()); c.din = xmem(rnd_x(0));
			cmds.push_back(c);
		end
		else begin
			c.kind = 0;
			c.opm  = (pipeonly || $urandom_range(0, 9) < 8) ? pipe_ops[$urandom_range(0, 17)]
			                                     : seq_ops[$urandom_range(0, 5)];
			c.sr   = 3'($urandom());
			// dependent chains half the time: the destination of one is a
			// source of the next
			c.dr   = ($urandom_range(0, 1) && cmds.size() > 0 && cmds[$].kind == 0)
			         ? cmds[$].dr : 3'($urandom());
			if ($urandom_range(0, 3) == 0) begin
				c.cls = 3'b010;
				c.fmt = fmts[$urandom_range(0, 5)];
				c.din = rnd_mem(c.fmt);
			end
			else begin
				c.cls = 3'b000;
				c.fmt = 3'd0;
				c.din = 96'd0;
			end
			cmds.push_back(c);
		end
		if ($urandom_range(0, pipeonly ? 63 : 7) == 0) sync_at.push_back(cmds.size());
	end
	sync_at.push_back(cmds.size());
endtask

//--------------------------------------------------------------------------
// drivers
//--------------------------------------------------------------------------
// each runs commands [from, to) as fast as its FPU takes them
`define DRIVER(P) \
task automatic P``_run(int from, int to); \
	for (int i = from; i < to; i++) begin \
		cmd_t c; \
		c = cmds[i]; \
		if (c.kind == 0) begin \
			P``_cls <= c.cls; P``_opm <= c.opm; P``_fmt <= c.fmt; \
			P``_sr <= c.sr; P``_dr <= c.dr; P``_din <= c.din; \
			begin int n0; n0 = P``_nend; \
			P``_req <= 1'b1; \
			@(posedge clk); \
			P``_req <= 1'b0; \
			while (P``_nend == n0) @(posedge clk); end \
			if (P``_lastx) begin \
				P``_pcap <= 1'b1; @(posedge clk); P``_pcap <= 1'b0; \
			end \
		end \
		else begin \
			P``_drain(); \
			if (c.kind == 1) begin \
				P``_crsel <= 2'd2; P``_crwd <= c.cr; P``_crwe <= 1'b1; \
				@(posedge clk); \
				P``_crwe <= 1'b0; P``_crsel <= 2'd1; \
			end \
			else begin \
				P``_fmsel <= c.dr; P``_fmwd <= c.din; P``_fmwe <= 1'b1; \
				@(posedge clk); \
				P``_fmwe <= 1'b0; \
			end \
		end \
	end \
	P``_drain(); \
endtask \
task automatic P``_drain(); \
	@(posedge clk); \
	while (P``_fpu.fst != 0 || P``_pbusy) @(posedge clk); \
	@(posedge clk); \
endtask

`DRIVER(r)
`DRIVER(d)

//--------------------------------------------------------------------------
// run and compare
//--------------------------------------------------------------------------
int errors = 0;
int n_done = 0, n_unimp = 0, n_unsupp = 0, n_exc = 0, n_pipe = 0, n_fwd = 0;
always @(posedge clk) begin
	if (d_fpu.pipe_issue) n_pipe++;
	if (d_fpu.pipe_issue && (d_fpu.q_aw > 1 || (d_fpu.q_kind != 0 && d_fpu.q_bw > 1))) n_fwd++;
	if (r_done) n_done++;
	if (r_unimp) n_unimp++;
	if (r_unsupp) n_unsupp++;
	if (r_exc) n_exc++;
end
longint rcyc = 0, dcyc = 0;

task automatic compare(int at);
	for (int i = 0; i < 8; i++) begin
		if (r_fpu.fr_s[i] !== d_fpu.fr_s[i] || r_fpu.fr_e[i] !== d_fpu.fr_e[i] ||
		    r_fpu.fr_m[i] !== d_fpu.fr_m[i] || r_fpu.fr_valid[i] !== d_fpu.fr_valid[i]) begin
			if (errors < 20)
				$display("MISMATCH after command %0d: FP%0d ref %b %h %h v%b, pipe %b %h %h v%b",
				         at, i, r_fpu.fr_s[i], r_fpu.fr_e[i], r_fpu.fr_m[i], r_fpu.fr_valid[i],
				         d_fpu.fr_s[i], d_fpu.fr_e[i], d_fpu.fr_m[i], d_fpu.fr_valid[i]);
			errors++;
		end
	end
	if (r_fpu.fpsr !== d_fpu.fpsr) begin
		if (errors < 20)
			$display("MISMATCH after command %0d: FPSR ref %h, pipe %h", at, r_fpu.fpsr, d_fpu.fpsr);
		errors++;
	end
endtask

initial begin
	int n, from;
	if (!$value$plusargs("seed=%d", seed)) seed = 1;
	if (!$value$plusargs("cmds=%d", n)) n = 20000;
	pipeonly = $test$plusargs("pipeonly");
	void'($urandom(seed));
	gen(n);
	repeat (4) @(posedge clk);
	nreset <= 1;
	repeat (40) @(posedge clk);   // reset sweeps
	from = 0;
	foreach (sync_at[k]) begin
		longint t0;
		t0 = $time;
		fork
			begin longint t; t = $time; r_run(from, sync_at[k]); rcyc += ($time - t) / 10; end
			begin longint t; t = $time; d_run(from, sync_at[k]); dcyc += ($time - t) / 10; end
		join
		compare(sync_at[k] - 1);
		if (errors >= 20) break;
		from = sync_at[k];
	end
	for (int i = 0; i < 8; i++)
		$display("FP%0d = %b %h %h", i, r_fpu.fr_s[i], r_fpu.fr_e[i], r_fpu.fr_m[i]);
	$display("FPSR %h; operations: %0d done, %0d unimplemented, %0d unsupported, %0d exceptions; %0d through the pipeline (%0d forwarded)",
	         r_fpu.fpsr, n_done, n_unimp, n_unsupp, n_exc, n_pipe, n_fwd);
	$display("%0d commands, %0d sync points: reference %0d cycles, pipelined %0d cycles",
	         cmds.size(), sync_at.size(), rcyc, dcyc);
	if (errors == 0) $display("PASS: the pipelined FPU matches the sequential one");
	else $display("FAIL: %0d mismatches", errors);
	$finish;
end

endmodule
