#!/bin/bash

# lidar_type="hdl64"
# lidar_type="hdl32"
# lidar_type="vlp16"
lidar_type="segcomp"

config_file="../config/${lidar_type}.yaml"
data_path="../../Data/${lidar_type}/"
prefix="perc.test."
output_path="../../Results/${lidar_type}/${lidar_type}_results/"

# for paper drawing
# start_idx=40
# end_idx=40
start_idx=0
end_idx=29

echo $config_file
echo $data_path
echo $prefix
echo $output_path

../build/cloudline2plane $config_file $data_path $prefix $output_path