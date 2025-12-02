# Vitis HLS Project Creation Script for EKF SLAM

# Open or create project
open_project ekf_slam_hls_project -reset

# Set the top-level function
set_top ekf_slam_update_hls

# Add source files
add_files ekf_slam_hls.cpp -cflags "-std=c++11 -I."
add_files ekf_slam_hls.h -cflags "-std=c++11 -I."

# Add testbench files (for co-simulation)
add_files -tb testbench_hls.cpp -cflags "-std=c++11 -I."
add_files -tb ekf_slam.cpp -cflags "-std=c++11 -I."
add_files -tb ekf_slam_dataloader.cpp -cflags "-std=c++11 -I."
add_files -tb ekf_slam_hls_wrapper.cpp -cflags "-std=c++11 -I."

# Add data files for testbench
add_files -tb ekf_slam_initial_conditions.txt
add_files -tb ekf_slam_data.csv

# Open solution
open_solution "solution1" -flow_target vivado -reset

# Set target FPGA part
# Common options:
# - xcvu9p-flga2104-2-i (Virtex UltraScale+ VU9P)
# - xcu250-figd2104-2L-e (Alveo U250)
# - xc7z020clg484-1 (Zynq-7000 ZC702)
# Adjust this to match your target FPGA
set_part {xcvu9p-flga2104-2-i}

# Create clock with 10ns period (100 MHz)
create_clock -period 10 -name default

# Configuration settings
config_compile -name_max_length 80
config_interface -m_axi_latency 64

# Run C simulation (optional - comment out if testbench not ready)
# csim_design -clean

# Run synthesis
csynth_design

# Run co-simulation (optional - takes time)
# cosim_design -rtl verilog -trace_level all

# Export design as IP (optional)
# export_design -format ip_catalog -description "EKF SLAM HLS Kernel" -vendor "user" -version "1.0"

# Generate implementation reports
# config_rtl -reset all
# config_rtl -reset_level low

exit
