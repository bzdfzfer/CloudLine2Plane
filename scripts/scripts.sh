#!/bin/bash

# lidar_type="hdl64"
# lidar_type="hdl32"
lidar_type="vlp16"

config_file="../config/${lidar_type}_sim.yaml"
data_path="../../Data/mrmb_lidar_simulation/${lidar_type}/data/"
prefix="${lidar_type}_"
output_path="../../Results/${lidar_type}/${lidar_type}_results/"

# for paper drawing
# start_idx=40
# end_idx=40
start_idx=1
end_idx=121

echo $config_file
echo $data_path
echo $prefix
echo $output_path

../build/cloudline2plane $config_file $data_path $prefix $output_path $start_idx $end_idx