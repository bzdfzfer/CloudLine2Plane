# CloudLine2Plane
This is a repo for multi-plane segmentation algorithm on sparse multi-line point cloud.



## Dependencies

* Eigen3
* PCL
* OpenCV
* YAML-CPP

## Build This Program

```
mkdir -p circle_odom_ws/src/ && cd circle_odom_ws/src/
git clone https://github.com/bzdfzfer/circle_odometry
cd ~/circle_odom_ws/ && catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Run This Program

```
source devel/setup.bash
roslaunch circle_odometry circle_odometry.launch 
# Open another terminal.
cd ~/circle_odom_ws/src/circle_odometry/rviz_cfg/
rviz -d circle_odom.rviz
rosbag play scene1.bag --clock 
```



## Dataset Downloading

* TODO (add  links for three rosbags)

  

## Some Results

#### Comparison with RF2O

![](pics/Trajectories_comparison.png)

#### Degneration processing results

![](pics/Degeneration_processing.png)

## Paper Citation:

* Xiaoguo D, Yuchu L, Qijun Chen, A fast multi-plane segmentation algorithm for sparse 3D LiDAR point clouds by line segment grouping, IEEE Transactions on Instrumentation and Measurement. (**Under review**)

