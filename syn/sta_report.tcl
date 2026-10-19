set top [lindex $argv 0]
project_open $top
create_timing_netlist -model slow
read_sdc
update_timing_netlist
set r [report_timing -setup -npaths 10 -detail summary -panel_name s -file paths.txt]
puts "WORST setup slack: [lindex $r 1]"
report_timing -setup -npaths 3 -detail full_path -file worst_full.txt
# every failing endpoint's worst path, for grouping by module (syn/paths_by_module.py)
report_timing -setup -npaths 20000 -pairs_only -less_than_slack 0 -detail summary -file failing.txt
foreach_in_collection c [get_clocks] { puts "clock [get_clock_info -name $c]" }
report_clock_fmax_summary -file fmax.txt
delete_timing_netlist
project_close
