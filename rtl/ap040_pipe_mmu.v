//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-25)                   //
//                                                                          //
// ap040_pipe_mmu.v - the ATCs and the table walker, at the memory ports    //
//                                                                          //
// The MC68040 translates in its instruction and data memory units, beside  //
// the pipeline, in parallel with indexing its caches (MC68040UM 4, Figure  //
// 4-1). This is rtl/ap040/ap040_mmu.v -- the sequential core's MMU, which  //
// sits on that core's one memory port -- turned to face the pipeline's two //
// ports: an instruction translation port (the bus controller's fetch      //
// stream in stage A, the IMU from stage B) and a data one for the DMU,     //
// each with its own ATC bank (16 sets x 4 ways -- the same row             //
// RAM as ap040_mmu.v, 32 rows {bank, set}, its port A now the instruction  //
// lookups and port B the data lookups and the walker's fill), its own      //
// lookup pipe and most-recent-hit copy, and its own transparent            //
// translation registers; one table walker behind both, with its own       //
// physical port, and the PTEST/PFLUSH sidebands.                           //
//                                                                          //
// Everything the sequential MMU decides it decides the same way here: TTR  //
// matching, the fault rules, the U/M history writes, nonresident entries   //
// installed by a failed search, the replacement pointers, the PTEST        //
// pre-flush of BOTH banks and its MMUSR, the PFLUSH variants. The banks     //
// are the same rows of the same RAM, so each keeps exactly the contents and //
// replacement order it had; only the two ports' requests can now be        //
// pending at once, and a walk for one holds the other off as a walk held   //
// off the one request there was. Port B holds the fill's row for the whole //
// walk, as it did there: no translation passes during a walk, so the data  //
// lookups it displaces are not wanted, and the data lookup pipe reads its  //
// row again once the walk is over.                                         //
//                                                                          //
// A port holds its request until it sees pass (the translation is on p_pa  //
// and p_cm this cycle) or a fault (a one-clock p_flt; the port must drop   //
// its request before the walker takes another -- W_DROP), as ap040_mmu.v's //
// core side did with c_req, m_req and c_flt.                               //
//--------------------------------------------------------------------------//

module ap040_pipe_mmu
(
	input             clk,
	input             nreset,

	input      [31:0] tc,          // bit15 E, bit14 P
	input      [31:0] urp,
	input      [31:0] srp,
	input      [31:0] itt0,
	input      [31:0] itt1,
	input      [31:0] dtt0,
	input      [31:0] dtt1,
	// The platform's cacheable windows (caches stage F; ap040_tg68k_compat's
	// own): outside them nothing is cached, whatever a TTR or a page says --
	// IO, ROM and unconfigured space must never be. Chip RAM and configured
	// Fast RAM (Zorro II, two Zorro III bases) may be; cache_allow_all lifts
	// the windows for benches with flat memory.
	input             cache_allow_all,
	input             cache_z2_ena,
	input       [4:0] cache_z3_base0,
	input             cache_z3_ena0,
	input       [3:0] cache_z3_base1,
	input             cache_z3_ena1,

	// instruction translation port (membus's stream; the IMU from stage B)
	input             i_req,
	input      [31:0] i_addr,
	input             i_sup,
	output            i_pass,
	output reg        i_flt,
	output     [31:0] i_pa,
	output      [1:0] i_cm,        // caching mode: 00 WT, 01 CB, 10/11 inhibited

	// A peek at the instruction port's most recent translation: whether an
	// address's page is the one it translated last (or a TTR's), where that
	// page is and its caching mode -- combinational from this unit's
	// registers, no search, no fault. The IMU's stream sends a prefetch in
	// its own cycle when this covers it.
	input      [31:0] ip_addr,
	input             ip_sup,
	output            ip_hit,
	output     [31:0] ip_pa,
	output      [1:0] ip_cm,

	// data translation port (the DMU)
	// Both are driven from registers: a translation's compare and its
	// physical address come after them in the cycle.
	input             d_req,
	input             d_write,
	// An access check: the write's permission judged without making the
	// write -- no M history. A page-crossing write checks both pages this
	// way before it is accepted, as ap040_pipe_membus.v's probes did.
	input             d_acc,
	input      [31:0] d_addr,
	input             d_sup,
	output            d_pass,
	output reg        d_flt,
	output     [31:0] d_pa,
	output      [1:0] d_cm,

	// PTEST/PFLUSH sideband
	input             pt_req,
	input             pt_write,
	input             pt_access,  // internal operand check, not a PTEST instruction
	input      [31:0] pt_addr,
	input       [2:0] pt_fc,
	output reg        pt_done,
	output reg [31:0] pt_mmusr,

	input             pf_req,
	input       [1:0] pf_mode,     // 00 (An) nonglobal, 01 (An), 10 all nonglobal, 11 all
	input      [31:0] pf_addr,
	input       [2:0] pf_fc,
	output reg        pf_done,

	// The table walker's own physical longword port. walk_hold keeps its
	// next access off that port: a write the DMU has accepted is still on
	// its way to memory, and a descriptor read now could miss it. The
	// sequential MMU's walks ran only for the transaction at the head of the
	// bus, after every older write had completed; this keeps that true now
	// that translation runs ahead of the bus controller.
	input             walk_hold,
	output            walker_req,
	output            walker_we,
	output     [31:0] walker_addr,
	output     [31:0] walker_wdat,
	input             walker_ack,
	input      [31:0] walker_data,
	input             walker_berr
);

wire tc_e = tc[15];
wire tc_p = tc[14];

function ttr_match;
	input [31:0] ttr;
	input [31:0] la;
	input        sup;
	begin
		ttr_match = ttr[15] &&
		            (&((la[31:24] ~^ ttr[31:24]) | ttr[23:16])) &&
		            (ttr[14] || (ttr[13] == sup));
	end
endfunction

function [31:0] pgtbl_addr;
	input [31:0] desc;
	begin
		pgtbl_addr = tc_p ? {desc[31:7], 7'd0} : {desc[31:8], 8'd0};
	end
endfunction

//---------------------------------------------------------------------------
// the ATC: index {bank, set, way}, bank 0 data, 1 instruction
//---------------------------------------------------------------------------

localparam EW   = 46;            // {resident, tag[16:0], pa[19:0], attr[7:0]}
localparam ROWW = 4*EW;

reg         atc_v  [0:127];      // {bank, set, way}
reg   [1:0] atc_rr [0:31];       // round robin per {bank, set}
// ATC entries survive RSTI; they are cleared only at configuration.
reg         atc_reset_seen = 1'b0;

wire  [3:0] i_set = tc_p ? i_addr[16:13] : i_addr[15:12];
wire [16:0] i_tag = tc_p ? {i_sup, i_addr[31:17], 1'b0} : {i_sup, i_addr[31:16]};
wire  [3:0] d_set = tc_p ? d_addr[16:13] : d_addr[15:12];
wire [16:0] d_tag = tc_p ? {d_sup, d_addr[31:17], 1'b0} : {d_sup, d_addr[31:16]};

// port A: instruction lookups, and the PFLUSH/PTEST sweep over all 32
// rows; port B: data lookups, and the walker's fill row for a whole walk.
reg         sweep_on;
reg   [5:0] sweep_cnt;
wire  [4:0] sweep_row = sweep_cnt[4:0];
wire        fill_we;             // below, with the walker state
reg         f_bank;              // 1 instruction, 0 data
wire        walk_holds_b;        // below: port B is the walker's
wire [ROWW-1:0] irow_q, drow_q;

// the free-running one-clock lookup pipes (see ap040_mmu.v)
reg   [3:0] il_set, dl_set;
reg  [16:0] il_tag, dl_tag;
reg         il_ld, dl_ld;
reg   [4:0] sweep_row_q;
reg         sweep_valid_q;
always @(posedge clk) begin
	il_set <= i_set;  il_tag <= i_tag;  il_ld <= i_req && !sweep_on && !fill_we;
	dl_set <= d_set;  dl_tag <= d_tag;  dl_ld <= d_req && !walk_holds_b && !fill_we;
	sweep_row_q   <= sweep_row;
	sweep_valid_q <= nreset && sweep_on;
end
wire ilk_fresh = il_ld && (il_set == i_set) && (il_tag == i_tag);
wire dlk_fresh = dl_ld && (dl_set == d_set) && (dl_tag == d_tag);

function [16:0] tag_of;
	input [EW-1:0] e;
	begin
		tag_of = e[44:28];
	end
endfunction

function [EW-1:0] way_of;
	input [ROWW-1:0] row;
	input      [1:0] w;
	begin
		way_of = row[w*EW +: EW];
	end
endfunction

wire ihit0 = ilk_fresh && atc_v[{1'b1, il_set, 2'd0}] && (tag_of(way_of(irow_q, 2'd0)) == il_tag);
wire ihit1 = ilk_fresh && atc_v[{1'b1, il_set, 2'd1}] && (tag_of(way_of(irow_q, 2'd1)) == il_tag);
wire ihit2 = ilk_fresh && atc_v[{1'b1, il_set, 2'd2}] && (tag_of(way_of(irow_q, 2'd2)) == il_tag);
wire ihit3 = ilk_fresh && atc_v[{1'b1, il_set, 2'd3}] && (tag_of(way_of(irow_q, 2'd3)) == il_tag);
wire ipipe_hit = ihit0 | ihit1 | ihit2 | ihit3;
wire [EW-1:0] ipipe_ent = ihit0 ? way_of(irow_q, 2'd0) : ihit1 ? way_of(irow_q, 2'd1) :
                          ihit2 ? way_of(irow_q, 2'd2) : way_of(irow_q, 2'd3);
wire dhit0 = dlk_fresh && atc_v[{1'b0, dl_set, 2'd0}] && (tag_of(way_of(drow_q, 2'd0)) == dl_tag);
wire dhit1 = dlk_fresh && atc_v[{1'b0, dl_set, 2'd1}] && (tag_of(way_of(drow_q, 2'd1)) == dl_tag);
wire dhit2 = dlk_fresh && atc_v[{1'b0, dl_set, 2'd2}] && (tag_of(way_of(drow_q, 2'd2)) == dl_tag);
wire dhit3 = dlk_fresh && atc_v[{1'b0, dl_set, 2'd3}] && (tag_of(way_of(drow_q, 2'd3)) == dl_tag);
wire dpipe_hit = dhit0 | dhit1 | dhit2 | dhit3;
wire [EW-1:0] dpipe_ent = dhit0 ? way_of(drow_q, 2'd0) : dhit1 ? way_of(drow_q, 2'd1) :
                          dhit2 ? way_of(drow_q, 2'd2) : way_of(drow_q, 2'd3);

// Each port retains its most recent hit (ap040_mmu.v's u_* copies, one per
// bank there): repeated accesses to a page need not wait for the RAM.
reg          iu_valid, du_valid;
reg    [3:0] iu_set, du_set;
reg   [16:0] iu_tag, du_tag;
reg [EW-1:0] iu_ent, du_ent;
reg   [31:0] u_tc;
wire u_clear = fill_we || sweep_on || pf_req || pt_req || (tc != u_tc);
always @(posedge clk) begin
	if (!nreset) begin
		u_tc <= 32'd0; iu_valid <= 1'b0; du_valid <= 1'b0;
	end else begin
		u_tc <= tc;
		if (u_clear) begin
			iu_valid <= 1'b0; du_valid <= 1'b0;
		end else begin
			if (ipipe_hit) begin iu_valid <= 1'b1; iu_set <= il_set; iu_tag <= il_tag; iu_ent <= ipipe_ent; end
			if (dpipe_hit) begin du_valid <= 1'b1; du_set <= dl_set; du_tag <= dl_tag; du_ent <= dpipe_ent; end
		end
	end
end
wire iu_hit = nreset && !u_clear && iu_valid && (iu_set == i_set) && (iu_tag == i_tag);
wire  [3:0] ip_set = tc_p ? ip_addr[16:13] : ip_addr[15:12];
wire [16:0] ip_tag = tc_p ? {ip_sup, ip_addr[31:17], 1'b0} : {ip_sup, ip_addr[31:16]};
wire ip_uhit = nreset && !u_clear && iu_valid && (iu_set == ip_set) && (iu_tag == ip_tag);
wire du_hit = nreset && !u_clear && du_valid && (du_set == d_set) && (du_tag == d_tag);
wire iatc_hit = iu_hit || ipipe_hit;
wire datc_hit = du_hit || dpipe_hit;
wire [EW-1:0] ih_ent = iu_hit ? iu_ent : ipipe_ent;
wire [EW-1:0] dh_ent = du_hit ? du_ent : dpipe_ent;
// entry fields: resident, pa, and attr {G?, U1:U0, S, CM, M, W}
wire        ih_r = ih_ent[45];
wire [19:0] ih_pa = ih_ent[27:8];
wire        ih_s = ih_ent[4];
wire  [1:0] ih_cm = ih_ent[3:2];
wire        dh_r = dh_ent[45];
wire [19:0] dh_pa = dh_ent[27:8];
wire        dh_s = dh_ent[4];
wire  [1:0] dh_cm = dh_ent[3:2];
wire        dh_m = dh_ent[1];
wire        dh_w = dh_ent[0];

//---------------------------------------------------------------------------
// transparent translation
//---------------------------------------------------------------------------

wire ip_ttr_a = ttr_match(itt0, ip_addr, ip_sup);
wire ip_ttr   = ip_ttr_a || ttr_match(itt1, ip_addr, ip_sup);
// resident and allowed at this privilege: a peek never reports a fault
assign ip_hit = tc_e && (ip_ttr || (ip_uhit && iu_ent[45] && !(!ip_sup && iu_ent[4])));
// Untranslated -- TC.E clear, or a TTR match -- the peek is the address
// itself, as the instruction port's own translation is: the IMU takes the
// peek's caching mode for every prefetch it sends untranslated
// (ap040_pipe_imu.v, rd_cm), not only for the ones a hit sends.
wire ip_xl    = tc_e && !ip_ttr;
assign ip_pa  = !ip_xl ? ip_addr : tc_p ? {iu_ent[27:9], ip_addr[12:0]} : {iu_ent[27:8], ip_addr[11:0]};
// A physical address in a cacheable window (see the ports).
function win_ok;
	input [31:0] pa;
	begin
		win_ok = cache_allow_all ||
		         (pa[31:21] == 11'd0) ||                                           // chip RAM
		         (!(|pa[31:24]) && (pa[23] ^ (|pa[22:21])) && cache_z2_ena) ||    // $200000-$9FFFFF
		         ((pa[31:27] == cache_z3_base0) && cache_z3_ena0) ||
		         ((pa[31:28] == cache_z3_base1) && cache_z3_ena1);
	end
endfunction
wire  [1:0] ip_cm_raw = ip_ttr ? (ip_ttr_a ? itt0[6:5] : itt1[6:5]) : ip_xl ? iu_ent[3:2] : 2'b00;
assign ip_cm  = win_ok(ip_pa) ? ip_cm_raw : 2'b10;
wire i_ttr_a = ttr_match(itt0, i_addr, i_sup);
wire i_ttr_b = ttr_match(itt1, i_addr, i_sup);
wire i_ttr   = i_ttr_a | i_ttr_b;
wire [1:0] i_ttr_cm = i_ttr_a ? itt0[6:5] : itt1[6:5];

wire d_ttr_a = ttr_match(dtt0, d_addr, d_sup);
wire d_ttr_b = ttr_match(dtt1, d_addr, d_sup);
wire d_ttr   = d_ttr_a | d_ttr_b;
wire       d_ttr_w  = d_ttr_a ? dtt0[2]   : dtt1[2];
wire [1:0] d_ttr_cm = d_ttr_a ? dtt0[6:5] : dtt1[6:5];

// PTEST: DFC selects supervisor/user and instruction/data space.
wire        pt_instr = (pt_fc[1:0] == 2'b10);
wire [31:0] pt_ttra  = pt_instr ? itt0 : dtt0;
wire [31:0] pt_ttrb  = pt_instr ? itt1 : dtt1;
wire        pt_ttr_a = ttr_match(pt_ttra, pt_addr, pt_fc[2]);
wire        pt_ttr_b = ttr_match(pt_ttrb, pt_addr, pt_fc[2]);
wire        pt_ttr_hit = pt_ttr_a | pt_ttr_b;
wire        pt_ttr_w = pt_ttr_a ? pt_ttra[2] : pt_ttrb[2];

//---------------------------------------------------------------------------
// translation decisions, per port
//---------------------------------------------------------------------------

// Instruction fetches are reads: no write protection and no M history.
wire i_atc_fault = tc_e && !i_ttr && iatc_hit && (!ih_r || (!i_sup && ih_s));
wire i_need_walk = tc_e && !i_ttr && (ilk_fresh || iu_hit) && !iatc_hit && !i_atc_fault;

wire d_ttr_fault = d_ttr && d_write && d_ttr_w;
wire d_atc_fault = tc_e && !d_ttr && datc_hit &&
                   (!dh_r || (d_write && dh_w) || (!d_sup && dh_s));
// a write to a clean page runs a table search to set the M bit
wire d_atc_mmiss = datc_hit && dh_r && d_write && !dh_m && !dh_w && !d_acc;
wire d_need_walk = tc_e && !d_ttr && (dlk_fresh || du_hit) && (!datc_hit || d_atc_mmiss) &&
                   !d_atc_fault;

// The walker idle and no maintenance under way: a translation may pass.
reg  [3:0] wst;
reg        w_active;
wire walker_free = (wst == 4'd0) && !w_active && !pf_req && !pt_req;

assign i_pass = i_req && !i_flt && !i_need_walk && !i_atc_fault &&
                (!tc_e || i_ttr || ilk_fresh || iu_hit) && walker_free;
assign d_pass = d_req && !d_flt && !d_need_walk && !d_ttr_fault && !d_atc_fault &&
                (!tc_e || d_ttr || dlk_fresh || du_hit) && walker_free;

assign i_pa = i_ttr ? i_addr :
              (tc_e && iatc_hit) ? (tc_p ? {ih_pa[19:1], i_addr[12:0]} : {ih_pa, i_addr[11:0]}) :
              i_addr;
assign d_pa = d_ttr ? d_addr :
              (tc_e && datc_hit) ? (tc_p ? {dh_pa[19:1], d_addr[12:0]} : {dh_pa, d_addr[11:0]}) :
              d_addr;
// Caching mode (MC68040UM 4.3): the matching TTR's, else the page's; with
// translation disabled and no TTR match, write-through.
// The caching mode, then the platform's windows: outside them, inhibited.
wire  [1:0] i_cm_raw = i_ttr ? i_ttr_cm : (tc_e && iatc_hit) ? ih_cm : 2'b00;
wire  [1:0] d_cm_raw = d_ttr ? d_ttr_cm : (tc_e && datc_hit) ? dh_cm : 2'b00;
assign i_cm = win_ok(i_pa) ? i_cm_raw : 2'b10;
assign d_cm = win_ok(d_pa) ? d_cm_raw : 2'b10;

//---------------------------------------------------------------------------
// the walker
//---------------------------------------------------------------------------

localparam W_IDLE = 4'd0;
localparam W_RA   = 4'd1;
localparam W_UA   = 4'd2;
localparam W_RB   = 4'd3;
localparam W_UB   = 4'd4;
localparam W_RC   = 4'd5;
localparam W_RI   = 4'd6;
localparam W_UC   = 4'd7;
localparam W_FILL = 4'd8;
localparam W_FLT  = 4'd9;
localparam W_DFLT = 4'd10;
localparam W_DROP = 4'd11;
localparam W_SWEEP = 4'd12;
localparam W_PTGO  = 4'd13;

reg        w_issued;
reg        w_pt;
reg        w_acc;
reg [31:0] w_la;
reg        w_super, w_write, w_user;
reg [31:0] w_desc_addr;
reg [31:0] w_desc;
reg        w_wp;
reg        w_buserr;
reg [31:0] w_req_addr, w_req_wdat;
reg        w_req_wr;
reg        sw_pt;

wire  [6:0] w_pi  = w_la[24:18];
wire  [5:0] w_pgi = tc_p ? {1'b0, w_la[17:13]} : w_la[17:12];

wire walk_ack = w_active && w_issued && walker_ack && !walker_berr;
wire walk_err = w_active && w_issued && walker_berr;

// the fill: overwrite an existing mapping of the same page, else the
// set's round-robin way
wire  [3:0] f_set = tc_p ? w_la[16:13] : w_la[15:12];
wire [16:0] f_tag = tc_p ? {w_super, w_la[31:17], 1'b0} : {w_super, w_la[31:16]};
wire [ROWW-1:0] frow_q = drow_q;   // port B holds the fill row for the walk
wire [EW-1:0] f_w0 = frow_q[0*EW +: EW];
wire [EW-1:0] f_w1 = frow_q[1*EW +: EW];
wire [EW-1:0] f_w2 = frow_q[2*EW +: EW];
wire [EW-1:0] f_w3 = frow_q[3*EW +: EW];
wire fv0 = atc_v[{f_bank, f_set, 2'd0}];
wire fv1 = atc_v[{f_bank, f_set, 2'd1}];
wire fv2 = atc_v[{f_bank, f_set, 2'd2}];
wire fv3 = atc_v[{f_bank, f_set, 2'd3}];
wire fhit0 = fv0 && (f_w0[44:28] == f_tag);
wire fhit1 = fv1 && (f_w1[44:28] == f_tag);
wire fhit2 = fv2 && (f_w2[44:28] == f_tag);
wire fhit3 = fv3 && (f_w3[44:28] == f_tag);
wire       f_way_hit = fhit0 | fhit1 | fhit2 | fhit3;
wire [1:0] f_rr  = atc_rr[{f_bank, f_set}];
wire [1:0] f_way = fhit0 ? 2'd0 : fhit1 ? 2'd1 : fhit2 ? 2'd2 : fhit3 ? 2'd3 : f_rr;

wire [19:0] f_pa_new   = tc_p ? {w_desc[31:13], 1'b0} : w_desc[31:12];
wire  [7:0] f_attr_new = {w_desc[10], w_desc[9:8], w_desc[7], w_desc[6:5],
                          w_desc[4], (w_wp | w_desc[2])};
// A failed search installs a VALID but nonresident entry.
wire [EW-1:0] f_ent_new = (wst == W_FLT) ? {1'b0, f_tag, 28'd0} :
                                          {1'b1, f_tag, f_pa_new, f_attr_new};
wire [ROWW-1:0] fill_wrow = {
	(f_way == 2'd3) ? f_ent_new : f_w3,
	(f_way == 2'd2) ? f_ent_new : f_w2,
	(f_way == 2'd1) ? f_ent_new : f_w1,
	(f_way == 2'd0) ? f_ent_new : f_w0 };
assign fill_we = (((wst == W_FILL || wst == W_DFLT) && !w_active) || wst == W_FLT) && nreset;

assign walk_holds_b = (wst != W_IDLE) || w_active;

dpram #(5, ROWW) atc_ram
(
	.clock     (clk),
	.address_a (sweep_on ? sweep_row : {1'b1, i_set}),
	.data_a    ({ROWW{1'b0}}),
	.wren_a    (1'b0),
	.q_a       (irow_q),
	.address_b (walk_holds_b ? {f_bank, f_set} : {1'b0, d_set}),
	.data_b    (fill_wrow),
	.wren_b    (fill_we),
	.q_b       (drow_q)
);

wire w_hist_m = w_write && !w_acc &&
                (!w_pt || (!(w_wp || w_desc[2]) && !(w_user && w_desc[7])));
wire w_denied = !w_pt && ((w_user && w_desc[7]) ||
                          (w_write && (w_wp || w_desc[2])));

assign walker_req  = w_active && w_issued;
assign walker_we   = w_req_wr;
assign walker_addr = w_req_addr;
assign walker_wdat = w_req_wdat;

task wrd;
	input [31:0] a;
	begin
		w_req_addr <= a;
		w_req_wr   <= 0;
		w_active   <= 1;
		w_issued   <= 0;
	end
endtask

task wwr;
	input [31:0] a;
	input [31:0] d;
	begin
		w_req_addr <= a;
		w_req_wdat <= d;
		w_req_wr   <= 1;
		w_active   <= 1;
		w_issued   <= 0;
	end
endtask

// The fault of the walk under way goes to the port that asked for it.
task walk_fault;
	begin
		if (f_bank) i_flt <= 1'b1;
		else        d_flt <= 1'b1;
	end
endtask

integer k;

always @(posedge clk) begin
	if (!nreset) begin
		wst <= W_IDLE;
		w_issued <= 0; w_pt <= 0; w_acc <= 0;
		w_la <= 0; w_super <= 0; w_write <= 0; w_user <= 0;
		w_desc_addr <= 0; w_desc <= 0; w_wp <= 0; w_buserr <= 0;
		w_req_addr <= 0; w_req_wdat <= 0; w_req_wr <= 0;
		w_active <= 0; f_bank <= 0;
		i_flt <= 0; d_flt <= 0;
		pt_done <= 0; pt_mmusr <= 0;
		pf_done <= 0;
		sweep_on <= 0; sweep_cnt <= 0; sw_pt <= 0;
		if (!atc_reset_seen) begin
			for (k = 0; k < 128; k = k + 1) atc_v[k] <= 0;
			for (k = 0; k < 32; k = k + 1) atc_rr[k] <= 0;
		end
		atc_reset_seen <= 1;
	end
	else begin
		i_flt <= 0;
		d_flt <= 0;
		pt_done <= 0;
		pf_done <= 0;
		if (w_active && !w_issued && !walk_hold) w_issued <= 1;
		if (fill_we) begin
			atc_v[{f_bank, f_set, f_way}] <= 1;
			if (!f_way_hit) atc_rr[{f_bank, f_set}] <= atc_rr[{f_bank, f_set}] + 2'd1;
		end

		if (walk_err) begin
			w_active <= 0;
			w_buserr <= 1;
			wst <= W_FLT;
		end
		else case (wst)
			W_IDLE: begin
				if (pf_req && !pf_done) begin
					if (pf_mode == 2'b11) begin
						for (k = 0; k < 128; k = k + 1) atc_v[k] <= 0;
						pf_done <= 1;
					end
					else begin
						sweep_on  <= 1;
						sweep_cnt <= 0;
						sw_pt     <= 0;
						wst       <= W_SWEEP;
					end
				end
				else if (pt_req && !pt_done) begin
					// PTEST first discards the matching entry in BOTH banks (see
					// ap040_mmu.v for why both); the search starts after.
					sweep_on  <= 1;
					sweep_cnt <= 0;
					sw_pt     <= 1;
					wst       <= W_SWEEP;
				end
				else if (d_req && !d_flt && (d_ttr_fault || d_atc_fault)) begin
					d_flt <= 1;
				end
				else if (i_req && !i_flt && i_atc_fault) begin
					i_flt <= 1;
				end
				else if (d_req && !d_flt && d_need_walk) begin
					w_pt    <= 0;
					w_acc   <= d_acc;
					w_la    <= d_addr;
					w_super <= d_sup;
					w_user  <= !d_sup;
					w_write <= d_write;
					w_wp    <= 0;
					w_buserr <= 0;
					f_bank  <= 1'b0;
					wrd({(d_sup ? srp[31:9] : urp[31:9]), 9'd0} + {23'd0, d_addr[31:25], 2'b00});
					wst <= W_RA;
				end
				else if (i_req && !i_flt && i_need_walk) begin
					w_pt    <= 0;
					w_acc   <= 0;
					w_la    <= i_addr;
					w_super <= i_sup;
					w_user  <= !i_sup;
					w_write <= 1'b0;
					w_wp    <= 0;
					w_buserr <= 0;
					f_bank  <= 1'b1;
					wrd({(i_sup ? srp[31:9] : urp[31:9]), 9'd0} + {23'd0, i_addr[31:25], 2'b00});
					wst <= W_RA;
				end
			end

			// Tag sweep over all 32 rows through port A. The RAM runs free:
			// pair its data with the registered read address.
			W_SWEEP: begin : sweep
				reg [16:0] sw_tag;
				reg  [3:0] sw_set;
				reg        sw_match;
				reg [EW-1:0] e;
				integer    w;
				sw_tag = sw_pt ? (tc_p ? {pt_fc[2], pt_addr[31:17], 1'b0}
				                       : {pt_fc[2], pt_addr[31:16]})
				               : (tc_p ? {pf_fc[2], pf_addr[31:17], 1'b0}
				                       : {pf_fc[2], pf_addr[31:16]});
				sw_set = sw_pt ? (tc_p ? pt_addr[16:13] : pt_addr[15:12])
				               : (tc_p ? pf_addr[16:13] : pf_addr[15:12]);
				if (sweep_valid_q && sweep_row_q == sweep_row) begin
					for (w = 0; w < 4; w = w + 1) begin
						e = irow_q[w*EW +: EW];
						if (sw_pt) begin
							if (sweep_row_q[3:0] == sw_set && e[44:28] == sw_tag)
								atc_v[{sweep_row_q, w[1:0]}] <= 0;
						end else begin
							sw_match = pf_mode[1] || (sweep_row_q[3:0] == sw_set && e[44:28] == sw_tag);
							if (sw_match && (pf_mode[0] || !e[7]))
								atc_v[{sweep_row_q, w[1:0]}] <= 0;
						end
					end
					if (sweep_cnt == 6'd31) begin
						sweep_on  <= 0;
						sweep_cnt <= 0;
						if (!sw_pt) begin
							pf_done <= 1;
							wst <= W_IDLE;
						end
						else wst <= W_PTGO;
					end
					else sweep_cnt <= sweep_cnt + 1'd1;
				end
			end

			W_PTGO: begin
				w_buserr <= 0;
				w_pt    <= 1;
				w_la    <= pt_addr;
				w_super <= pt_fc[2];
				w_user  <= !pt_fc[2];
				w_write <= pt_write;
				w_acc   <= pt_access;
				w_wp    <= 0;
				f_bank  <= pt_instr;
				if (pt_ttr_hit) begin
					// T and R only; a write probe against a write-protected TTR
					// reports B (ap040_mmu.v, t_mmu 38).
					pt_mmusr <= (pt_write && pt_ttr_w) ? 32'h0000_0800 : 32'h0000_0003;
					pt_done <= 1;
					w_pt <= 0;
					wst <= W_IDLE;
				end
				else if (pt_access && !tc_e) begin
					pt_mmusr <= 32'h0000_0001;
					pt_done <= 1;
					w_pt <= 0;
					wst <= W_IDLE;
				end
				else begin
					wrd({(pt_fc[2] ? srp[31:9] : urp[31:9]), 9'd0} + {23'd0, pt_addr[31:25], 2'b00});
					wst <= W_RA;
				end
			end

			W_RA: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				if (!walker_data[1]) wst <= W_FLT;
				else begin
					w_wp <= w_wp | walker_data[2];
					if (!walker_data[3]) begin
						wwr(w_req_addr, walker_data | 32'h8);
						wst <= W_UA;
					end
					else begin
						wrd({walker_data[31:9], 9'd0} + {23'd0, w_pi, 2'b00});
						wst <= W_RB;
					end
				end
			end

			W_UA: if (walk_ack) begin
				w_active <= 0;
				wrd({w_desc[31:9], 9'd0} + {23'd0, w_pi, 2'b00});
				wst <= W_RB;
			end

			W_RB: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				if (!walker_data[1]) wst <= W_FLT;
				else begin
					w_wp <= w_wp | walker_data[2];
					if (!walker_data[3]) begin
						wwr(w_req_addr, walker_data | 32'h8);
						wst <= W_UB;
					end
					else begin
						wrd(pgtbl_addr(walker_data) + {24'd0, w_pgi, 2'b00});
						wst <= W_RC;
					end
				end
			end

			W_UB: if (walk_ack) begin
				w_active <= 0;
				wrd(pgtbl_addr(w_desc) + {24'd0, w_pgi, 2'b00});
				wst <= W_RC;
			end

			W_RC: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				case (walker_data[1:0])
					2'b00: wst <= W_FLT;
					2'b10: begin
						wrd(walker_data & 32'hFFFF_FFFC);
						wst <= W_RI;
					end
					default: wst <= W_UC;
				endcase
			end

			W_RI: if (walk_ack) begin
				w_desc <= walker_data;
				w_desc_addr <= w_req_addr;
				w_active <= 0;
				if (walker_data[1:0] == 2'b00 || walker_data[1:0] == 2'b10) wst <= W_FLT;
				else wst <= W_UC;
			end

			W_UC: begin
				// U is set before an access error is reported; M never for a
				// denied write (ap040_mmu.v).
				if (w_denied) begin
					if (!w_desc[3]) begin
						wwr(w_desc_addr, w_desc | 32'h8);
						w_desc <= w_desc | 32'h8;
					end
					wst <= W_DFLT;
				end
				else if (!w_desc[3] || (w_hist_m && !w_desc[4])) begin
					wwr(w_desc_addr, w_desc | 32'h8 | (w_hist_m ? 32'h10 : 32'h0));
					w_desc <= w_desc | 32'h8 | (w_hist_m ? 32'h10 : 32'h0);
					wst <= W_FILL;
				end
				else wst <= W_FILL;
			end

			W_DFLT: begin
				if (w_active) begin
					if (walk_ack) w_active <= 0;
				end
				else begin
					walk_fault;
					wst <= W_DROP;
				end
			end

			// The faulted port sees its fault a clock after it is registered;
			// the walker takes nothing more until that port has let go.
			W_DROP: begin
				if (f_bank ? !i_req : !d_req) wst <= W_IDLE;
			end

			W_FILL: begin
				if (w_active) begin
					if (walk_ack) w_active <= 0;
				end
				else if (w_pt) begin
					// the PAGE FRAME (8K: bit 12 clear), with the attributes
					pt_mmusr <= (tc_p ? {w_desc[31:13], 13'd0} : {w_desc[31:12], 12'd0}) |
					            {21'd0, w_desc[10], w_desc[9:8], w_desc[7],
					             w_desc[6:5], w_desc[4], 1'b0,
					             (w_wp | w_desc[2]), 1'b0, 1'b1};
					pt_done <= 1;
					w_pt <= 0;
					wst <= W_IDLE;
				end
				else wst <= W_IDLE;   // the held request now hits and passes
			end

			W_FLT: begin
				if (w_pt) begin
					pt_mmusr <= w_buserr ? 32'h0000_0800 : 32'd0;
					pt_done <= 1;
					w_pt <= 0;
					wst <= W_IDLE;
				end
				else begin
					walk_fault;
					wst <= W_DROP;
				end
			end

			default: wst <= W_IDLE;
		endcase
	end
end

endmodule
