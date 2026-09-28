//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (milestone 6: folder           //
// independence + Scc.B)                                                    //
//                                                                          //
// ap040_pipe_regfile.v - D0-D7, A0-A6 and the three stack pointers        //
//                                                                          //
// This is the pipe's own fork of rtl/ap040/ap040_regfile.v, not a shared   //
// instantiation of it: milestones 2 and 5 had added a WRITE_THROUGH        //
// parameter and a dbg_d3 tap to the shared file, on the reasoning that     //
// both were safe/inert additions for the working sequential core. The     //
// user decided that's not the structure wanted -- rtl/ap040/ stays        //
// completely untouched from here on, and rtl/ap040_pipe/ is fully self-    //
// contained, so deleting either directory can never affect the other.      //
// Both prior additions are folded in permanently here instead of carried   //
// as an opt-in: WRITE_THROUGH is unconditional (no parameter -- this core   //
// always needs the same-cycle write-then-read bypass, see                  //
// ap040_pipe_core.v's header comment), and the debug taps cover all eight   //
// data registers (dbg_d0-dbg_d7) rather than growing one at a time -- this  //
// exact file had already needed a one-more-register tap addition twice,    //
// and the taps are zero-risk pure wires.                                   //
//                                                                          //
// Named ap040_pipe_regfile, not ap040_regfile, for the same reason         //
// ap040_pipe_core is never ap040_core (see its header comment): it must    //
// never collide with, or be silently substitutable for, the shared         //
// sequential-core module of the same shape.                                //
//                                                                          //
// Register index encoding on both read ports and the write port:          //
//   0..7  = D0..D7                                                         //
//   8..14 = A0..A6                                                         //
//   15    = A7, banked to USP/ISP/MSP by the current SR.S/SR.M state       //
//                                                                          //
// Writes are full 32-bit; the core performs read-modify-write merging for  //
// byte and word register destinations.                                     //
//--------------------------------------------------------------------------//

module ap040_pipe_regfile
(
	input             clk,
	input             ce,
	input             nreset,

	// active stack pointer selection
	input             sr_s,
	input             sr_m,
	// The bank a WRITE lands in (milestone 92). Reads take the FORWARDED
	// SR, and must: a reader in EA-fetch has to see a mode switch that is
	// still in EX. A write arriving from EX belongs to an instruction that
	// is already past that point, and the bank it meant was fixed when it
	// executed -- so it takes the ARCHITECTURAL SR instead, and a younger
	// MOVE-to-SR cannot redirect it into the bank it is switching to.
	// Port 3 is EA-fetch's own (MOVEM), so it stays with the reads.
	input             sr_s_w,
	input             sr_m_w,
	// ...and port 2 carries its own, because one instruction can write A7
	// twice and mean two different stacks: a faulting (A7)+ updates the
	// USER stack pointer through this port while its exception's new
	// SUPERVISOR stack pointer goes through port 1 (milestone 92).
	input       [1:0] sp_sel2_w,

	// write port
	input             we,
	input       [3:0] waddr,
	input      [31:0] wdata,

	// read ports
	input       [3:0] raddr_a,
	output     [31:0] rdata_a,
	// Third read port (milestone 56). Indexed addressing needs An, the
	// index register Xn, and -- for anything but a plain load -- the
	// destination operand as well, which is one more than two ports allow.
	// Read-only and write-through-bypassed exactly like the other two.
	input       [3:0] raddr_c,
	output     [31:0] rdata_c,

	input       [3:0] raddr_b,
	output     [31:0] rdata_b,
	// EA-calculate's own read (restructuring plan, phase 4): the base of an
	// address it forms a stage early. The same bypasses as the others.
	input       [3:0] raddr_d,
	output     [31:0] rdata_d,
	// ...and its index register, for an indexed address (port E).
	input       [3:0] raddr_e,
	output     [31:0] rdata_e,

	// direct stack pointer access for MOVEC/MOVE USP, independent of the
	// currently active bank (never asserted together with the main write)
	// Second write port (milestone 30). (An)+ and -(An) write TWO registers
	// in one instruction -- the data to Dn and the updated address to An --
	// and one commit path cannot do that. The aux port below is not a way
	// out: it reaches only USP/ISP/MSP, never a GPR.
	//
	// Port 1 wins if both name the same register. No implemented instruction
	// can do that (MOVE.L (An)+,Dn has distinct banks), but the priority is
	// fixed rather than undefined so a future MOVEA.L (A0)+,A0 fails
	// predictably instead of racing.
	// Third write port (milestone 50). MOVEM's load direction writes up to
	// sixteen registers from ONE instruction, so it cannot use the normal
	// writeback path, which carries one result per instruction. This port is
	// driven straight from ap040_ea_fetch.v's MOVEM sequencer.
	//
	// It needs no arbitration against the other two: a MOVEM holds EA-fetch
	// and stalls everything behind it, and the pipeline ahead of it has
	// drained by the time any beat writes, so ports 1 and 2 are idle. It is
	// applied LAST for the same reason port 2 is applied after port 1 --
	// a defined order beats an undefined one even where it cannot occur.
	input             we3,
	input       [3:0] waddr3,
	input      [31:0] wdata3,

	input             we2,
	input       [3:0] waddr2,
	input      [31:0] wdata2,

	input             aux_we,
	input       [1:0] aux_sel,     // 0=USP 1=ISP 2=MSP
	input      [31:0] aux_wdata,
	output     [31:0] usp_q,
	output     [31:0] isp_q,
	output     [31:0] msp_q,

	// debug taps (registered values, no extra logic on the write path)
	output     [31:0] dbg_d0,
	output     [31:0] dbg_d1,
	output     [31:0] dbg_d2,
	output     [31:0] dbg_d3,
	output     [31:0] dbg_d4,
	output     [31:0] dbg_d5,
	output     [31:0] dbg_d6,
	output     [31:0] dbg_d7,
	output     [31:0] dbg_a0,
	output     [31:0] dbg_a7
);

reg [31:0] dreg [0:7];
reg [31:0] areg [0:6];
reg [31:0] usp;
reg [31:0] isp;
reg [31:0] msp;

// A7 resolves to the active stack pointer
wire [1:0] sp_sel = !sr_s ? 2'd0 : (sr_m ? 2'd2 : 2'd1); // 0=USP 1=ISP 2=MSP
wire [1:0] sp_sel_w = !sr_s_w ? 2'd0 : (sr_m_w ? 2'd2 : 2'd1);
wire [31:0] sp_active = (sp_sel == 2'd0) ? usp : (sp_sel == 2'd1) ? isp : msp;

// The write-through bypass has to agree about WHICH A7. The unified index
// is 15 whatever bank it names, so a write to ISP and a read of USP look
// like the same register here; when the two selects disagree, they are not
// the same register and the value must not be forwarded.
wire       a7_same  = (sp_sel_w   == sp_sel);
wire       a7_same2 = (sp_sel2_w  == sp_sel);
wire       we_fwd   = we  && (a7_same  || (waddr  != 4'd15));
wire       we2_fwd  = we2 && (a7_same2 || (waddr2 != 4'd15));
wire       w_collide = we && we2 && (waddr == waddr2) &&
                       ((waddr != 4'd15) || (sp_sel_w == sp_sel2_w));

// MOVEC writes a stack pointer through the auxiliary port, which bypasses
// the A7 index entirely -- so a reader of A7 in that cycle saw the old
// value and only caught up two instructions later (milestone 92).
wire [31:0] sp_read = (aux_we && (aux_sel == sp_sel)) ? aux_wdata : sp_active;

// direct expressions, not a function: a function referencing the register
// arrays breaks continuous-assign sensitivity on some simulators.
//
// Same-cycle write-then-read bypass, unconditional (this core can have a
// write and a same-address read of the same register committing in the
// same cycle -- the WB-forward case -- and needs the read to see it; see
// ap040_pipe_core.v's header comment for the full picture).
assign rdata_a = (we_fwd  && (waddr  == raddr_a)) ? wdata  :
                 (we3 && (waddr3 == raddr_a)) ? wdata3 :
                 (we2_fwd && (waddr2 == raddr_a)) ? wdata2 :
                 !raddr_a[3]            ? dreg[raddr_a[2:0]] :
                 (raddr_a[2:0] == 3'd7) ? sp_read : areg[raddr_a[2:0]];
assign rdata_c = (we_fwd  && (waddr  == raddr_c)) ? wdata  :
                 (we3 && (waddr3 == raddr_c)) ? wdata3 :
                 (we2_fwd && (waddr2 == raddr_c)) ? wdata2 :
                 !raddr_c[3]            ? dreg[raddr_c[2:0]] :
                 (raddr_c[2:0] == 3'd7) ? sp_read : areg[raddr_c[2:0]];
assign rdata_b = (we_fwd  && (waddr  == raddr_b)) ? wdata  :
                 (we3 && (waddr3 == raddr_b)) ? wdata3 :
                 (we2_fwd && (waddr2 == raddr_b)) ? wdata2 :
                 !raddr_b[3]            ? dreg[raddr_b[2:0]] :
                 (raddr_b[2:0] == 3'd7) ? sp_read : areg[raddr_b[2:0]];

assign rdata_d = (we_fwd  && (waddr  == raddr_d)) ? wdata  :
                 (we3 && (waddr3 == raddr_d)) ? wdata3 :
                 (we2_fwd && (waddr2 == raddr_d)) ? wdata2 :
                 !raddr_d[3]            ? dreg[raddr_d[2:0]] :
                 (raddr_d[2:0] == 3'd7) ? sp_read : areg[raddr_d[2:0]];

assign rdata_e = (we_fwd  && (waddr  == raddr_e)) ? wdata  :
                 (we3 && (waddr3 == raddr_e)) ? wdata3 :
                 (we2_fwd && (waddr2 == raddr_e)) ? wdata2 :
                 !raddr_e[3]            ? dreg[raddr_e[2:0]] :
                 (raddr_e[2:0] == 3'd7) ? sp_read : areg[raddr_e[2:0]];

integer i;
always @(posedge clk) begin
	if (!nreset) begin
		for (i = 0; i < 8; i = i + 1) dreg[i] <= 0;
		for (i = 0; i < 7; i = i + 1) areg[i] <= 0;
		usp <= 0;
		isp <= 0;
		msp <= 0;
	end
	else if (ce) begin
		if (we) begin
			if (!waddr[3])            dreg[waddr[2:0]] <= wdata;
			else if (waddr[2:0] != 7) areg[waddr[2:0]] <= wdata;
			else begin
				case (sp_sel_w)
					2'd0:    usp <= wdata;
					2'd1:    isp <= wdata;
					default: msp <= wdata;
				endcase
			end
		end
		// Second port, same shape as the first. The comment here used to
		// say port 1 wins a same-register collision; the code said the
		// opposite, because this block is applied SECOND and the later
		// assignment is the one that lands. It mattered the moment an
		// instruction could write A7 twice on the same stack: a faulting
		// (A7)+ in supervisor mode put the increment on top of the
		// exception's own stack pointer (milestone 93). Now the guard
		// makes the code match what was written -- and for A7 the two
		// only collide when they name the same BANK, which is exactly
		// the user-mode case that must keep BOTH writes.
		if (we2 && !w_collide) begin
			if (!waddr2[3])            dreg[waddr2[2:0]] <= wdata2;
			else if (waddr2[2:0] != 7) areg[waddr2[2:0]] <= wdata2;
			else begin
				case (sp_sel2_w)
					2'd0:    usp <= wdata2;
					2'd1:    isp <= wdata2;
					default: msp <= wdata2;
				endcase
			end
		end
		if (we3) begin
			if (!waddr3[3])            dreg[waddr3[2:0]] <= wdata3;
			else if (waddr3[2:0] != 7) areg[waddr3[2:0]] <= wdata3;
			else begin
				case (sp_sel)
					2'd0:    usp <= wdata3;
					2'd1:    isp <= wdata3;
					default: msp <= wdata3;
				endcase
			end
		end
		if (aux_we) begin
			case (aux_sel)
				2'd0:    usp <= aux_wdata;
				2'd1:    isp <= aux_wdata;
				default: msp <= aux_wdata;
			endcase
		end
	end
end

assign usp_q = usp;
assign isp_q = isp;
assign msp_q = msp;

assign dbg_d0 = dreg[0];
assign dbg_d1 = dreg[1];
assign dbg_d2 = dreg[2];
assign dbg_d3 = dreg[3];
assign dbg_d4 = dreg[4];
assign dbg_d5 = dreg[5];
assign dbg_d6 = dreg[6];
assign dbg_d7 = dreg[7];
assign dbg_a0 = areg[0];
assign dbg_a7 = sp_active;

endmodule
