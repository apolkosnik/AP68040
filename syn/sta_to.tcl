# quartus_sta -t sta_to.tcl <project> <to-pattern> [n]: full worst paths into matching registers
set top [lindex $argv 0]; set to [lindex $argv 1]; set n [lindex $argv 2]
if {$n eq ""} {set n 1}
project_open $top
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -to [get_registers $to] -npaths $n -detail full_path -file to_report.txt
delete_timing_netlist
project_close
