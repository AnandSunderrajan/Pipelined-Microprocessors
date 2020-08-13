transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/LC_3B_2.sv}
vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/Library.sv}
vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/DatapathLib.sv}
vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/Cache.sv}
vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/Arbiter.sv}
vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/projtoplevel.sv}

vlog -sv -work work +incdir+C:/Users/Anand/Desktop/ECEFINALPROH/lc3b {C:/Users/Anand/Desktop/ECEFINALPROH/lc3b/testbench.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  testbench

add wave *
view structure
view signals
run 1000 ns
