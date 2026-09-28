//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core (caches stage F)               //
//                                                                          //
// ap040_pipe_tg68k_compat.v - the pipelined core behind the port set       //
// rtl/ap040/ap040_tg68k_compat.v presents to cpu_wrapper.v                 //
//                                                                          //
// The same ports, parameters and meaning, so cpu_wrapper.v can take either //
// core (its AP040_PIPE parameter). Inside is ap040_pipe_bus16.v: the       //
// pipelined CPU, its MMU, the instruction and data memory units with their //
// caches, and the FSM core's own 16-bit bus adapter.                       //
//                                                                          //
// What the ports carry here:                                               //
//   tick_in                    the core's enable: cpu_wrapper.v's core     //
//     tick, every clock unless FAST_CLOCK divides it. The memory side --   //
//     the DMU, the IMU, the MMU, the bus controller -- runs every clock.  //
//   clkena_in / bus_clkena_in  the FSM core's enable and the bus's. The   //
//     16-bit adapter advances only on the second, as the FSM core's does. //
//     The first is not used: cpu_wrapper.v holds it low while a bus cycle  //
//     runs, which the FSM core needs, and this core stalls on its own --   //
//     running through the bus's cycles is what its pipeline is for.       //
//   ipl                        active low, synchronized and taken once     //
//     stable over two clocks (ap040_core.v's rule); interrupts are always //
//     autovectored (ipl_autovector is ignored, as there).                 //
//   cache_*                    the platform's cacheable windows and the    //
//     chipset's writes, to the MMU and both caches (ap040_pipe_mmu.v,      //
//     ap040_pipe_bus16.v).                                                 //
//   walker_*                   the table walker's own physical port, as    //
//     the FSM core's: descriptors never cross the 16-bit bus.             //
//   nresetout                  low while RESET drives the reset line (128  //
//     cycles, as ap040_core.v's S_RESET_HOLD).                            //
//   nmi_ack_toggle             flips once for each level 7 interrupt taken.//
//   cache_maint_req            high while CINV/CPUSH runs, for a cache     //
//     below this core.                                                     //
//   mmu_cache_inhibit          high: this core's own caches are the 68040's //
//     two; one more below them would only add a coherency question, so     //
//     every access tells the RAM controller's cache to leave it alone.     //
//   cacr_out, vbr_out          the registers as they stand.                //
//   post_drain                 low: the store buffer is this core's own,  //
//     and nothing in cpu_wrapper.v needs to wait on it.                    //
//   debug_halted               a double fault; the other debug outputs,    //
//     the MMU's address taps and the external-cache port are tied off.    //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_tg68k_compat
#(
	parameter AP040_HAS_MMU      = 1,
	parameter AP040_HAS_FPU      = 1,
	parameter AP040_ENABLE_CACHE = 1,
	parameter AP040_FAST_SIM     = 0,
	parameter AP040_DEBUG_EXCEPTIONS = 0,
	parameter [7:0] AP040_FPU_REVISION = 8'h41,
	parameter       AP040_POST_STORES  = 0
)
(
	input         clk,
	input         nreset,
	input         clkena_in,
	input         bus_clkena_in,
	input         tick_in,

	input         cache_allow_all,
	input         cache_snoop_stb,
	input  [31:0] cache_snoop_addr,
	input         cache_z2_ena,
	input   [4:0] cache_z3_base0,
	input         cache_z3_ena0,
	input   [3:0] cache_z3_base1,
	input         cache_z3_ena1,
	input  [15:0] data_in,
	input  [2:0]  ipl,
	input         ipl_autovector,
	input         berr,

	output [31:0] addr_out,
	output [15:0] data_write,
	output        nwr,
	output        nuds,
	output        nlds,
	output [1:0]  busstate,
	output        longword,
	output        post_drain,
	output        nresetout,
	output [2:0]  fc,
	output        nmi_ack_toggle,
	output        cache_maint_req,
	output        cache_maint_ic,
	output        cache_maint_dc,

	output [31:0] mmu_addr_log,
	output [31:0] mmu_addr_phys,
	output        mmu_cache_inhibit,

	output        walker_req,
	output        walker_we,
	output [31:0] walker_addr,
	output [31:0] walker_wdat,
	input         walker_ack,
	input  [31:0] walker_data,
	input         walker_berr,

	output        cache_req,
	output [31:0] cache_addr,
	input  [15:0] cache_data,
	input         cache_ack,
	output        cache_burst,
	output [2:0]  cache_burst_len,
	output [28:1] cache_ramaddr,

	output [31:0] cacr_out,
	output [31:0] vbr_out,
	output        debug_busy,
	output        debug_fault,
	output        debug_halted,
	output [255:0] debug_status,
	output [127:0] debug_status2,
	output         debug_exception_valid,
	output [511:0] debug_exception
);

// The interrupt level: active low in, synchronized, taken when it has held
// for two clocks (a level changing between samples is not a request yet).
reg  [2:0] ipl_s1, ipl_s2, irq_lvl;
always @(posedge clk)
	if (!nreset) begin
		ipl_s1 <= 3'b111; ipl_s2 <= 3'b111; irq_lvl <= 3'd0;
	end else begin
		ipl_s1 <= ipl;
		ipl_s2 <= ipl_s1;
		if (ipl_s1 == ipl_s2) irq_lvl <= ~ipl_s2;
	end

wire reset_out, nmi_ack;
reg  nmi_t;
always @(posedge clk)
	if (!nreset)      nmi_t <= 1'b0;
	else if (nmi_ack) nmi_t <= ~nmi_t;
assign nmi_ack_toggle = nmi_t;
assign nresetout      = !reset_out;

ap040_pipe_bus16 #(
	.PC_RESET      (32'h0000_0000),
	.PROG_WORDS    (32'h7FFF_FFFF),   // no issue budget: that is the benches'
	.RESET_VECTORS (1)                // SSP and PC from $0 and $4
) u_pipe
(
	.clk (clk), .nreset (nreset), .ce (tick_in), .irq_lvl (irq_lvl),
	.clkena_in (bus_clkena_in), .berr (berr),
	.cache_allow_all (cache_allow_all), .cache_z2_ena (cache_z2_ena),
	.cache_z3_base0 (cache_z3_base0), .cache_z3_ena0 (cache_z3_ena0),
	.cache_z3_base1 (cache_z3_base1), .cache_z3_ena1 (cache_z3_ena1),
	.snoop_stb (cache_snoop_stb), .snoop_addr (cache_snoop_addr),
	.cacr_out (cacr_out), .vbr_out (vbr_out), .reset_out (reset_out), .nmi_ack (nmi_ack),
	.cache_maint (cache_maint_req), .halted (debug_halted),
	.walker_req (walker_req), .walker_we (walker_we), .walker_addr (walker_addr),
	.walker_wdat (walker_wdat), .walker_ack (walker_ack), .walker_data (walker_data),
	.walker_berr (walker_berr),
	.data_in (data_in), .addr_out (addr_out), .data_write (data_write),
	.nwr (nwr), .nuds (nuds), .nlds (nlds), .busstate (busstate), .longword (longword), .fc (fc),
	.dbg_if_valid (), .dbg_if_pc (), .dbg_id_valid (), .dbg_id_pc (),
	.dbg_eac_valid (), .dbg_eac_pc (), .dbg_eaf_valid (), .dbg_eaf_pc (),
	.dbg_ex_valid (), .dbg_ex_pc (), .dbg_wb_valid (), .dbg_wb_pc (),
	.dbg_d0 (), .dbg_d1 (), .dbg_d2 (), .dbg_d3 (), .dbg_d4 (), .dbg_d5 (), .dbg_d6 (), .dbg_d7 (),
	.dbg_ccr (), .dbg_sr (), .dbg_commits ()
);

assign post_drain        = 1'b0;
assign cache_maint_ic    = 1'b0;
assign cache_maint_dc    = 1'b0;
assign mmu_addr_log      = 32'd0;
assign mmu_addr_phys     = 32'd0;
assign mmu_cache_inhibit = 1'b1;
assign cache_req         = 1'b0;
assign cache_addr        = 32'd0;
assign cache_burst       = 1'b0;
assign cache_burst_len   = 3'd0;
assign cache_ramaddr     = 28'd0;
assign debug_busy        = 1'b0;
assign debug_fault       = 1'b0;
assign debug_status      = 256'd0;
assign debug_status2     = 128'd0;
assign debug_exception_valid = 1'b0;
assign debug_exception   = 512'd0;

endmodule
