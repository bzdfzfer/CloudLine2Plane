#!/bin/bash

##### HDL-64 realistic data.
lidar_type="hdl64"
start_idx=0
end_idx=0

##### HDL-32 realistic data.
# lidar_type="hdl32"
# start_idx=1
# end_idx=1

##### VLP-16 realistic data.
# lidar_type="vlp16"
# start_idx=869
# end_idx=869

config_file="../config/${lidar_type}_real.yaml"
data_path="../../Data/realistic_data/${lidar_type}/"
prefix="${lidar_type}_"
output_path="../../Results//realistic/${lidar_type}/${lidar_type}_results/"

# for paper drawing
# start_idx=40
# end_idx=40


flip_flag=1;
repeat_num=0;

echo $config_file
echo $data_path
echo $prefix
echo $output_path

../build/cloudline2plane $config_file $data_path $prefix $output_path $start_idx $end_idx $flip_flag $repeat_num