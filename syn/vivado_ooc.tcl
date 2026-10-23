# Out-of-context Vivado fit of the core on the Artix-7 of the MinimigAGA_TC64
# board (xc7a100tfgg676-2), default directives.
#   vivado -mode batch -nojournal -source vivado_ooc.tcl -tclargs <rtl_dir> <out_dir> [period_ns]
# The default period is 17.630 ns (56.72 MHz, the Minimig CPU island).
lassign $argv R O PER
if {$PER eq ""} { set PER 17.630 }
file mkdir $O
set_param general.maxThreads 4
foreach f [list \
  $R/ap68040_pkg.sv $R/gen/ap68040_upkg.sv $R/ap68040_alu.sv $R/ap68040_alu_slow.sv \
  $R/ap68040_muldiv.sv $R/ap68040_biu.sv $R/ap68040_ram.sv $R/ap68040_atc.sv \
  $R/ap68040_dmu.sv $R/ap68040_predec.sv $R/ap68040_fetch.sv $R/ap68040_decode.sv \
  $R/ap68040_useq.sv $R/ap68040_fpif.sv $R/ap68040_backend.sv $R/ap68040_snoop.sv $R/ap68040.sv] {
  read_verilog -sv $f
}
read_verilog $R/fpu/ap040_fpu.v
synth_design -top ap68040 -part xc7a100tfgg676-2 -mode out_of_context -include_dirs [list $R $R/gen $R/fpu]
create_clock -name clk -period $PER [get_ports clk]
report_utilization -file $O/synth_util.rpt
opt_design
place_design
phys_opt_design
route_design
phys_opt_design
write_checkpoint -force $O/post_route.dcp
report_utilization -file $O/route_util.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $O/hier_util.rpt
report_timing_summary -max_paths 10 -file $O/route_timing.rpt
report_timing -max_paths 100 -nworst 1 -unique_pins -file $O/top100.rpt
puts "FIT DONE"
