cmake_minimum_required(VERSION 2.8.3)
project(cloudline2plane)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(YAML-CPP REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(cloudline2plane 
src/cloudline2plane.cpp
src/line_clustering.cpp
src/line_extraction.cpp
src/line_feature.cpp
src/plane_extraction.cpp
src/visualizer.cpp
)
target_link_libraries(cloudline2plane 
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libyaml-cpp.so)


add_executable(cloudline2plane_proj 
src/cloudline2plane_proj.cpp
src/line_clustering.cpp
src/line_extraction.cpp
src/line_feature.cpp
src/plane_extraction.cpp
src/visualizer.cpp
)
target_link_libraries(cloudline2plane_proj 
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libyaml-cpp.so)

add_executable(visPlaneCloud
src/visPlaneCloud.cpp
src/visualizer.cpp
)
target_link_libraries(visPlaneCloud
${PCL_LIBRARIES} 
${OpenCV_LIBRARIES}
)


