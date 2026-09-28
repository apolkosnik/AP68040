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

module ap040_regfile #(parameter EXTRA_READS = 0)
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

    // Optional independent pipeline reads share the same pending write and
    // stack-bank selection. Extra MLAB mirrors keep ownership selection out
    // of the sequencer's operand-to-ALU timing path.
    input       [3:0] raddr_c,
    output     [31:0] rdata_c,
    input       [3:0] raddr_d,
    output     [31:0] rdata_d,
    input       [3:0] raddr_e,
    output     [31:0] rdata_e,
    // P196: the queue head's EA register, read every clock (retire-time
    // source reads); only with EXTRA_READS
    input       [3:0] raddr_f,
    output     [31:0] rdata_f,

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
// READ DURING WRITE.  "no_rw_check" allows unspecified RAM output during a
// read/write collision; it does not prevent collisions.  Integer-register
// collisions are ordinary (612 cycles in t_integer alone).  The first MLAB
// version used this attribute without isolating the collision.  With flops a read
// returns the OLD value; an MLAB writes with an internal pulse, so an
// asynchronous read of the written address can return the NEW one part way
// through the cycle.  Simulation cannot show the difference -- it models the
// array exactly -- and the core did not boot.
//
// So the write is held one cycle and the read bypasses it.  The RAM is never
// consulted for an address whose write is still pending, which makes the
// result independent of what the primitive does with a simultaneous access:
//
//   cycle N    write issued, held in pend_*; RAM untouched; a read of that
//              address returns the RAM's old word, as flip-flops would
//   cycle N+1  pend_* is applied to the RAM, and a read of that address is
//              answered from pend_wdata, not the RAM being written
//   cycle N+2  the RAM holds it
//
// Keep no_rw_check now that the pending-write bypass isolates both read
// ports.  With "MLAB" alone, Quartus 17 rejects these asynchronous RAMs for
// unsupported read-during-write behavior and implements both banks as flops.
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

generate if (EXTRA_READS) begin : extra_reads
    (* ramstyle = "MLAB, no_rw_check" *) reg [31:0] bank_c [0:15];
    (* ramstyle = "MLAB, no_rw_check" *) reg [31:0] bank_d [0:15];
    (* ramstyle = "MLAB, no_rw_check" *) reg [31:0] bank_e [0:15];
    wire hit_e = pend_we && pend_waddr == raddr_e;
    wire [31:0] q_e = hit_e ? pend_wdata : (rf_written[raddr_e] ? bank_e[raddr_e] : 32'd0);
    assign rdata_e = raddr_e == 15 ? sp_active : q_e;
    wire hit_c = pend_we && (pend_waddr == raddr_c);
    wire hit_d = pend_we && (pend_waddr == raddr_d);
    wire [31:0] q_c = hit_c ? pend_wdata
                           : (rf_written[raddr_c] ? bank_c[raddr_c] : 32'd0);
    wire [31:0] q_d = hit_d ? pend_wdata
                           : (rf_written[raddr_d] ? bank_d[raddr_d] : 32'd0);
    assign rdata_c = (raddr_c == 4'd15) ? sp_active : q_c;
    assign rdata_d = (raddr_d == 4'd15) ? sp_active : q_d;
    always @(posedge clk) begin
        if (nreset && ce && pend_we) begin
            bank_c[pend_waddr] <= pend_wdata;
            bank_e[pend_waddr] <= pend_wdata;
            bank_d[pend_waddr] <= pend_wdata;
        end
    end
end else begin : no_extra_reads
    assign rdata_e = 32'd0;
    assign rdata_c = 32'd0;
    assign rdata_d = 32'd0;
end endgenerate

// P196: port F, the core's lookahead of the queue head's EA register --
// present in every configuration (not an EXTRA_READS port)
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] bank_f [0:15];
wire hit_f = pend_we && pend_waddr == raddr_f;
wire [31:0] q_f = hit_f ? pend_wdata : (rf_written[raddr_f] ? bank_f[raddr_f] : 32'd0);
assign rdata_f = (raddr_f == 4'd15) ? sp_active : q_f;
always @(posedge clk)
    if (nreset && ce && pend_we) bank_f[pend_waddr] <= pend_wdata;

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
