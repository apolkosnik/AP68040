//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_regfile.v - D0-D7, A0-A6 and the three stack pointers              //
//                                                                          //
// Register index encoding on both read ports and the write port:          //
//   0..7  = D0..D7                                                         //
//   8..14 = A0..A6                                                         //
//   15    = A7, banked to USP/ISP/MSP by the current SR.S/SR.M state       //
//                                                                          //
// Writes are full 32-bit; the core performs read-modify-write merging for  //
// byte and word register destinations.                                     //
//--------------------------------------------------------------------------//

module ap040_regfile
(
	input             clk,
	input             ce,
	input             nreset,

	// active stack pointer selection
	input             sr_s,
	input             sr_m,

	// write port
	input             we,
	input       [3:0] waddr,
	input      [31:0] wdata,

	// read ports
	input       [3:0] raddr_a,
	output     [31:0] rdata_a,
	input       [3:0] raddr_b,
	output     [31:0] rdata_b,

	// direct stack pointer access for MOVEC/MOVE USP, independent of the
	// currently active bank (never asserted together with the main write)
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
	output     [31:0] dbg_a0,
	output     [31:0] dbg_a7
);

// D0-D7 and A0-A6 in two mirrored MLAB banks rather than flip-flops, the
// same trade the FP register file makes: an MLAB gives one write and one
// read port, so two asynchronous reads need two copies of the data.  A7 is
// not in the array -- it resolves to one of the three stack pointers below,
// which stay in flops because several consumers read them directly.
//
// MLAB contents cannot be reset, and this core resets the integer registers
// to zero, so a 15-bit "written" vector carries that instead: an entry reads
// as zero until it has been written once.
//
// READ DURING WRITE.  The first version of this carried "no_rw_check" from
// the FP file, which asserts the design never reads and writes one address in
// the same cycle.  That is true of FP0-FP7 and emphatically false here: an
// instruction writing Dn while the next reads Dn is ordinary, and counting it
// found 612 such cycles in t_integer alone.  With flip-flops that read
// returns the OLD value; an MLAB writes with an internal pulse, so an
// asynchronous read of the written address can return the NEW one part way
// through the cycle.  Simulation cannot show the difference -- it models the
// array exactly -- and the core did not boot (eb158fb71).
//
// So the write is held one cycle and the read bypasses it.  The RAM's output
// is never USED for an address whose write is in flight, which makes the
// result independent of what the primitive does with a simultaneous access:
//
//   cycle N    write issued, held in pend_*; RAM untouched; a read of that
//              address returns the RAM's old word, as flip-flops would
//   cycle N+1  pend_* is applied to the RAM, and a read of that address is
//              answered from pend_wdata, not the RAM being written
//   cycle N+2  the RAM holds it
//
// THE HOLD IS NOT OPTIONAL, AND NOT ONLY FOR THE BYPASS.  On 2026-09-17 this
// was "corrected" (06d90f6fb) to write the RAM on the issue edge from the
// core's wdata and keep pend_* only as the bypass copy -- identical in
// simulation, and the board went to a yellow screen before Workbench.  The
// timing report says why: TimeQuest lists the RAM cells (dpram_ilo1) as
// non-unate clock edges and "assumes pos-unate behavior" -- the MLAB inverts
// its clock internally for the write, so the data path INTO the RAM's write
// registers really has half a cycle, and the analyzer times it to the full
// one.  From pend_* that path is a register-to-register hop and the half
// cycle is trivial; from the ALU cone it is not, and the setup slack in the
// report (+0.149) said nothing about it.  Keep the RAM's write inputs
// registered here.  The 738 "reads in the commit cycle" that motivated the
// change were reads of a settled word: cycle N+2 is fine on hardware, cycle
// N+1 is what the bypass covers.
//
// no_rw_check STAYS, and with the bypass it is honest.  The attribute does
// not promise the accesses never coincide; it says the read data is
// undefined when they do, and asks the fitter not to spend logic defending
// against it.  The bypass discards exactly that datum, so the undefined value
// cannot reach the datapath.  Dropping the attribute instead was tried and is
// worse in both directions: without it Quartus will not infer an MLAB here at
// all -- the fit reported ALMs used for memory 0.0, both mirrored banks in
// flip-flops, 1,172 registers and 673 ALMs against the plain array's 576 and
// 459 -- so the file cost 214 ALMs and still had no memory in it.  The defect
// was never the attribute on its own; it was the attribute with nothing
// masking the datum it leaves undefined.
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] bank_a [0:15];
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] bank_b [0:15];
reg [14:0] rf_written;
reg        pend_we;
reg  [3:0] pend_waddr;
reg [31:0] pend_wdata;
reg [31:0] usp;
reg [31:0] isp;
reg [31:0] msp;
reg [31:0] dbg_shadow [0:3];

// A7 resolves to the active stack pointer
wire [1:0] sp_sel = !sr_s ? 2'd0 : (sr_m ? 2'd2 : 2'd1); // 0=USP 1=ISP 2=MSP
wire [31:0] sp_active = (sp_sel == 2'd0) ? usp : (sp_sel == 2'd1) ? isp : msp;

// direct expressions, not a function: a function referencing the register
// arrays breaks continuous-assign sensitivity on some simulators
wire hit_a = pend_we && (pend_waddr == raddr_a[3:0]);
wire hit_b = pend_we && (pend_waddr == raddr_b[3:0]);
wire [31:0] q_a = hit_a ? pend_wdata
                        : (rf_written[raddr_a[3:0]] ? bank_a[raddr_a[3:0]] : 32'd0);
wire [31:0] q_b = hit_b ? pend_wdata
                        : (rf_written[raddr_b[3:0]] ? bank_b[raddr_b[3:0]] : 32'd0);
assign rdata_a = (raddr_a == 4'd15) ? sp_active : q_a;
assign rdata_b = (raddr_b == 4'd15) ? sp_active : q_b;

integer i;
always @(posedge clk) begin
	if (!nreset) begin
		rf_written <= 0;
		pend_we <= 0; pend_waddr <= 0; pend_wdata <= 0;
		dbg_shadow[0] <= 0; dbg_shadow[1] <= 0;
		dbg_shadow[2] <= 0; dbg_shadow[3] <= 0;
		usp <= 0;
		isp <= 0;
		msp <= 0;
	end
	else if (ce) begin
		// apply the write held from the previous enabled cycle
		if (pend_we) begin
			bank_a[pend_waddr] <= pend_wdata;
			bank_b[pend_waddr] <= pend_wdata;
			rf_written[pend_waddr] <= 1'b1;
		end
		pend_we <= 0;
		if (we) begin
			if (waddr != 4'd15) begin
				pend_we <= 1;
				pend_waddr <= waddr;
				pend_wdata <= wdata;
				// the halt beacon's fixed taps would each need their own
				// mirrored bank; four shadow words are cheaper
				if (waddr == 4'd0) dbg_shadow[0] <= wdata;
				if (waddr == 4'd1) dbg_shadow[1] <= wdata;
				if (waddr == 4'd2) dbg_shadow[2] <= wdata;
				if (waddr == 4'd8) dbg_shadow[3] <= wdata;
			end
			else begin
				case (sp_sel)
					2'd0:    usp <= wdata;
					2'd1:    isp <= wdata;
					default: msp <= wdata;
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

assign dbg_d0 = dbg_shadow[0];
assign dbg_d1 = dbg_shadow[1];
assign dbg_d2 = dbg_shadow[2];
assign dbg_a0 = dbg_shadow[3];
assign dbg_a7 = sp_active;

endmodule
