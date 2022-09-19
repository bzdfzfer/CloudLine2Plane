/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 4/7/18.
//

#include "visualizer.h"


void PlaneNormalVisualizer::UpdateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                        std::string cloud_name,
                                        std::vector<double> cloud_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";
//  DLOG(INFO) << cloud->size();

  if (cloud->size() == 0) {
    // DLOG(INFO) << ">>>>>>> no points <<<<<<<";
    return;
  }

  if (!viewer->updatePointCloud(cloud, cloud_name)) {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             cloud_color[0],
                                             cloud_color[1],
                                             cloud_color[2],
                                             cloud_name);
  }

}

void PlaneNormalVisualizer::UpdateCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                  pcl::PointCloud<pcl::Normal>::ConstPtr normals,
                                                  int ds_ratio,
                                                  std::string cloud_name,
                                                  std::string normals_name,
                                                  std::vector<double> cloud_color,
                                                  std::vector<double> normals_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";
//  DLOG(INFO) << cloud->size();
//  DLOG(INFO) << normals->size();

  if (cloud->size() == 0 || normals->size() == 0) {
    // DLOG(INFO) << ">>>>>>> no points <<<<<<<";
    return;
  }

  if (!viewer->updatePointCloud(cloud, cloud_name)) {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             cloud_color[0],
                                             cloud_color[1],
                                             cloud_color[2],
                                             cloud_name);
  }
  viewer->removePointCloud(normals_name, 0);
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, ds_ratio, 0.5, normals_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                           normals_color[0],
                                           normals_color[1],
                                           normals_color[2],
                                           normals_name);

}

void PlaneNormalVisualizer::UpdateLines(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                                        std::vector<double> line_color,
                                        std::string line_base_name, 
                                        bool clean_screen_flag ) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";

  int num_cloud1 = cloud1->size();
  int num_cloud2 = cloud2->size();
  if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2) {
    // DLOG(INFO) << ">>>>>>> no points or sizes are not the same <<<<<<<";
    // LOG_IF(INFO, num_cloud1 != num_cloud2) << num_cloud1 << " != " << num_cloud2;
    return;
  }
  if(clean_screen_flag) {
    for (const auto line_name : line_names) {
        viewer->removeShape(line_name);
    }    

    line_names.clear();
  }



  for (int i = 0; i < num_cloud1; ++i) {
    std::stringstream line_name_ss;
    line_name_ss << line_base_name << i;
    std::string line_name = line_name_ss.str();
    viewer->addLine(cloud1->at(i), cloud2->at(i), line_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        line_color[0],
                                        line_color[1],
                                        line_color[2],
                                        line_name);
    line_names.push_back(line_name);
  }

}

void PlaneNormalVisualizer::UpdateLines(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                                        std::vector<double> line_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";

  int num_cloud1 = cloud1->size();
  int num_cloud2 = cloud2->size();
  if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2) {
    // DLOG(INFO) << ">>>>>>> no points or sizes are not the same <<<<<<<";
    // LOG_IF(INFO, num_cloud1 != num_cloud2) << num_cloud1 << " != " << num_cloud2;
    return;
  }

  for (const auto line_name : line_names) {
    viewer->removeShape(line_name);
  }

  line_names.clear();

  for (int i = 0; i < num_cloud1; ++i) {
    std::stringstream line_name_ss;
    line_name_ss << "line" << i;
    std::string line_name = line_name_ss.str();
    viewer->addLine(cloud1->at(i), cloud2->at(i), line_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        line_color[0],
                                        line_color[1],
                                        line_color[2],
                                        line_name);
    line_names.push_back(line_name);
  }

}

void PlaneNormalVisualizer::UpdateLines(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                                        std::vector<std::vector<double>> v_line_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";

  int num_cloud1 = cloud1->size();
  int num_cloud2 = cloud2->size();
  int num_colors = v_line_color.size();

  if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2) {
    // DLOG(INFO) << ">>>>>>> no points or sizes are not the same <<<<<<<";
    // LOG_IF(INFO, num_cloud1 != num_cloud2) << num_cloud1 << " != " << num_cloud2;
    return;
  }

  if(num_colors != num_cloud1)
  {
    return;
  }

  for (const auto line_name : line_names) {
    viewer->removeShape(line_name);
  }

  line_names.clear();

  for (int i = 0; i < num_cloud1; ++i) {
    std::stringstream line_name_ss;
    line_name_ss << "line" << i;
    std::string line_name = line_name_ss.str();
    viewer->addLine(cloud1->at(i), cloud2->at(i), line_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        v_line_color[i][0],
                                        v_line_color[i][1],
                                        v_line_color[i][2],
                                        line_name);
    line_names.push_back(line_name);
  }

}

void PlaneNormalVisualizer::UpdatePlanes(const std::vector<Eigen::Vector4d,
                                                           Eigen::aligned_allocator<Eigen::Vector4d>> &plane_coeffs) {

  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";

  int size = plane_coeffs.size();
  if (size == 0) {
    // DLOG(INFO) << ">>>>>>> no planes <<<<<<<";
    return;
  }

  for (const auto plane_name : plane_names) {
    viewer->removeShape(plane_name);
  }

  plane_names.clear();

  for (int i = 0; i < size; ++i) {
    Eigen::Vector4d coeffs_eigen = plane_coeffs[i];
    std::stringstream plane_name_ss;
    plane_name_ss << "plane" << i;
    std::string plane_name = plane_name_ss.str();
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(coeffs_eigen.x());
    coeffs.values.push_back(coeffs_eigen.y());
    coeffs.values.push_back(coeffs_eigen.z());
    coeffs.values.push_back(coeffs_eigen.w());
    viewer->addPlane(coeffs, plane_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, plane_name);
    plane_names.push_back(plane_name);
  }
}

PlaneNormalVisualizer::PlaneNormalVisualizer() {
 // boost::mutex::scoped_lock lk(m);
 // viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
 // viewer->setBackgroundColor(0, 0, 0);
 // viewer->addCoordinateSystem(1.0);
 // viewer->addText("debugger by Kitkat7,  modified by bzdfzfer", 10, 10, "debugger text", 0);
 // viewer->initCameraParameters();
 init = true;
}

void PlaneNormalVisualizer::Spin() {

  {
    boost::mutex::scoped_lock lk(m);
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->addText("debugger by Kitkat7, modified by bzdfzfer", 10, 10, "debugger text", 0);
    viewer->initCameraParameters();
    // position x,y,z         view x,y,z      view up: x,y,z
    // viewer->setCameraPosition(-32.1585, 38.041, 40.6718,    0, 0, 0,   0.321541, -0.551288, 0.769866);
    // viewer->setCameraFieldOfView(1);
    // viewer->setCameraClipDistances( 0.244114, 244.114 );

    // simulation lidar.
    // viewer->loadCameraParameters("/media/bzdfzfer/Datasets/PlaneExtraction_Codes/CloudLine2Plane/scripts/screenshot-1654334072.cam");

    // realistic hdl-64,
    // viewer->loadCameraParameters("/media/bzdfzfer/Datasets/PlaneExtraction_Codes/CloudLine2Plane/scripts/screenshot-1655102915.cam");

    // realistic hdl-32,
    // viewer->loadCameraParameters("/media/bzdfzfer/Datasets/PlaneExtraction_Codes/CloudLine2Plane/scripts/screenshot-1655094079.cam");

    // realistic vlp-16,
    viewer->loadCameraParameters("/media/bzdfzfer/Datasets/PlaneExtraction_Codes/CloudLine2Plane/scripts/screenshot-1655089570.cam");


    init = true;
  }

  while (!viewer->wasStopped()) {
    {
      boost::mutex::scoped_lock lk(m);
//      DLOG(INFO) << ">>>>>>> spin <<<<<<<";
      viewer->spinOnce(100);
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}

// } // namespace lio