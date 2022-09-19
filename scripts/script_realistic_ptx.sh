#!/bin/bash

##### HDL-64 realistic data.
lidar_type="hdl64"
start_idx=0
end_idx=0
data_path="../../Data/realistic_data/${lidar_type}_dataset/"
prefix="${lidar_type}_calib_"

##### HDL-32 realistic data.
# lidar_type="hdl32"
# start_idx=1
# end_idx=1
# data_path="../../Data/realistic_data/${lidar_type}e_501_dataset/"
# prefix="${lidar_type}e_"

##### VLP-16 realistic data.
# lidar_type="vlp16"
# start_idx=869
# end_idx=869
# data_path="../../Data/realistic_data/${lidar_type}_nsh_dataset/"
# prefix="${lidar_type}_"

config_file="../config/${lidar_type}_real.yaml"

output_path="../../Data/realistic_data/test/${lidar_type}/"

# for paper drawing
# start_idx=40
# end_idx=40


flip_flag=1;
repeat_num=100;

echo $config_file
echo $data_path
echo $prefix
echo $output_path

../build/cloudline2plane_proj $config_file $data_path $prefix $output_path $start_idx $end_idx $flip_flag $repeat_num