//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (2026-09-24: interrupts)       //
//                                                                          //
// ap040_pipe_irq.v - the interrupt request, as the core sees it            //
//                                                                          //
// ap040_core.v's input chain, transliterated: the level is sampled twice  //
// and believed only when two samples agree (a 3 -> 5 change passing       //
// through 7 must not look like an NMI); level 7 is edge-triggered, armed  //
// whenever the pins leave 7; a level 1-6 request that has once qualified  //
// against the mask is held until it is taken, so an instruction raising   //
// the mask afterwards does not make it vanish -- but it is tracked down   //
// when the source withdraws it. The chain runs every clock, not under ce, //
// as the reference's runs on every tick.                                  //
//                                                                          //
// The level comes in active high, 0 for none: an unconnected input is     //
// then no interrupt rather than a level-7 one. The pins' inversion is the  //
// board wrapper's business.                                                //
//                                                                          //
// pend/take_lvl are judged against mask_live, the SR the instruction at   //
// the boundary sees (EA-fetch's forwarded one); the hold is maintained    //
// against the committed mask, as the reference's is against its sr.       //
// pend_c, against the committed mask, is what wakes a STOP.                //
//                                                                          //
// The committed mask lags an entry: the reference raises it in the state   //
// that acknowledges, this pipeline when the entry retires, a few cycles    //
// on. Between the two the source is still asserting the level just taken, //
// and judged against the OLD mask it qualified again, was held, and was    //
// taken a second time at the handler's first instruction (found by        //
// tb_ap040_pipe_irqdual.v). From the acknowledge until the next SR commit  //
// -- the entry's own, since nothing older is left in flight when one is   //
// taken -- the level taken stands in for the mask.                         //
//--------------------------------------------------------------------------//

module ap040_pipe_irq
(
	input            clk,
	input            nreset,
	input      [2:0] irq_lvl_in,   // requested level, active high
	input      [2:0] mask,         // SR[10:8], committed
	input      [2:0] mask_live,    // SR[10:8] as the boundary instruction sees it
	input            ack,          // an interrupt entry was taken (one pulse)
	input            ack_nmi,      // ...and it was level 7
	input            sr_commit,    // an SR write lands this cycle
	output           pend,
	output     [2:0] take_lvl,
	output           pend_c
);

reg [2:0] s1, s2;         // the two samples
reg [2:0] lvl;            // the believed level
reg [2:0] hold;           // a mask-qualified level 1-6, until taken
reg       nmi_arm;
reg       ack_wait;       // an entry is taken and its SR not yet committed
reg [2:0] ack_mask;
wire [2:0] mask_c = ack_wait ? ack_mask : mask;

always @(posedge clk) begin
	if (!nreset) begin
		s1 <= 3'd0; s2 <= 3'd0; lvl <= 3'd0; hold <= 3'd0; nmi_arm <= 1'b0;
		ack_wait <= 1'b0; ack_mask <= 3'd0;
	end else begin
		s1 <= irq_lvl_in;
		s2 <= s1;
		if (s1 == s2) begin
			lvl <= s2;
			if (s2 != 3'd7) nmi_arm <= 1'b1;
		end
		if (ack && !ack_nmi)
			hold <= 3'd0;
		else if (hold > lvl)
			hold <= (lvl > mask_c) ? lvl : 3'd0;
		else if (lvl != 3'd0 && lvl != 3'd7 && lvl > mask_c && lvl > hold)
			hold <= lvl;
		if (ack) begin
			ack_wait <= 1'b1;
			ack_mask <= take_lvl;
		end else if (sr_commit)
			ack_wait <= 1'b0;
		// acceptance wins over re-arming in the same cycle
		if (ack && ack_nmi) nmi_arm <= 1'b0;
	end
end

// One cycle earlier than the registered level when the samples agree, as
// the reference recognises it (cputest times a request to land just ahead
// of an instruction boundary).
wire [2:0] lvl_live = (s1 == s2) ? s2 : lvl;
wire       nmi_pend = (lvl_live == 3'd7) && nmi_arm;
wire       live     = (lvl_live != 3'd0) && (lvl_live != 3'd7) && (lvl_live > mask_live);
wire       live_c   = (lvl_live != 3'd0) && (lvl_live != 3'd7) && (lvl_live > mask_c);

assign take_lvl = nmi_pend ? 3'd7 : (live && lvl_live > hold) ? lvl_live : hold;
assign pend     = nmi_pend || live || (hold != 3'd0);
assign pend_c   = nmi_pend || live_c || (hold != 3'd0);

endmodule
