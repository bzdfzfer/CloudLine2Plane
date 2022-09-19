#include "segcomp_loader.h"
#include "visualizer.h"
#include "common.h"

#include <opencv2/opencv.hpp>

#include <map>


std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}



int main(int argc, char** argv) {

    if(argc < 5) {
        std::cerr << "usage: ./visPlaneCloud data_path result_path dataset_name alg_name frame_id rows cols" << std::endl;
        return -1;
    }

    std::string data_path;
    std::string result_path; 
    std::string dataset_name; // vlp16 hdl32 hdl64 segcomp
    std::string alg_name; // algorithm: ransac, rht, pe, peac, omps, ''
    int frame_id;

    data_path = std::string(argv[1]);
    result_path = std::string(argv[2]);
    dataset_name = std::string(argv[3]);
    alg_name = std::string(argv[4]);
    frame_id = std::atoi(argv[5]);
    ROWS = std::atoi(argv[6]);
    COLS = std::atoi(argv[7]);

    // load point cloud, load labels, load color images.
    std::string cloud_file;
    std::string label_img_file;
    std::string color_img_file;
    if(dataset_name == "segcomp")
        cloud_file = data_path + "/" + dataset_name + "/perc.test." +
            std::to_string(frame_id) + ".xyz";
    else 
        cloud_file = data_path + "/" + dataset_name + "/data/" + dataset_name +
            "_" + std::to_string(frame_id) + ".xyz";

    std::cout << "cloud file path: " << cloud_file << std::endl;

    if(alg_name == "") {
        label_img_file = result_path + "/" + dataset_name + "/" + dataset_name + 
            "_results/pngs/" + dataset_name + "_" + std::to_string(frame_id) + ".png";
        color_img_file = result_path + "/" + dataset_name + "/" + dataset_name + 
            "_results/pngs/" + dataset_name + "_" + std::to_string(frame_id) + "_color.png";
    } else {
        label_img_file = result_path + "/" + dataset_name + "/" + dataset_name + "_" +
            alg_name + "_results/pngs/" + dataset_name + "_" + std::to_string(frame_id) + ".png";        
        color_img_file = result_path + "/" + dataset_name + "/" + dataset_name + "_" +
            alg_name + "_results/pngs/" + dataset_name + "_" + std::to_string(frame_id) + "_color.png";
    }
    std::cout << "label image path: " << color_img_file << std::endl;

    // loading point cloud.
    float *coords[3];
    loadSegCompPerceptron3(cloud_file, ROWS, COLS, coords);
    CloudT segcomp_cloud;
    if(dataset_name == "segcomp")
        segcomp_cloud = CloudFromCoords(coords, ROWS, COLS);
    else 
        segcomp_cloud = CloudFromCoordsVLP(coords, ROWS, COLS);

    // loading label image.
    cv::Mat label_img = cv::imread(label_img_file, -1);
    std::cout << "label image loaded " << std::endl;
    std::cout << "label image type: " << type2str(label_img.type()) << std::endl;

    cv::Mat color_img = cv::imread(color_img_file);
    std::cout << "color label image loaded " << std::endl;



    std::cout << "visualizing label .... " << std::endl;
    cv::imshow("label image", label_img);
    cv::imshow("color label image", color_img);
    cv::waitKey(0);

    // prepare data
    std::map<int, CloudT> planes_map;
    std::map<int, std::vector<double>> colors_map;
    for(int row = 0; row < label_img.rows; row ++ ) {
        for(int col = 0; col < label_img.cols; col ++ ) {
            uint16_t label;
            if(alg_name == "gt")
                label = label_img.at<uchar>(row, col); // for groudth truth data.
            else 
                label = label_img.at<uint16_t>(row, col);
                
            if(label == 0 ) 
                continue;
            PointT pt;
            std::vector<double> pl_color;

            pt = segcomp_cloud.at(col, row);
            cv::Vec3b color_pix = color_img.at<cv::Vec3b>(row, col);
            pl_color.push_back(double(1.0/255.0*color_pix[0]));
            pl_color.push_back(double(1.0/255.0*color_pix[1]));
            pl_color.push_back(double(1.0/255.0*color_pix[2]));
            
            planes_map[label].points.push_back(pt);
            colors_map[label] = pl_color;
        }
    }

    int planes_size = planes_map.size();
    int colors_size = colors_map.size();
    std::cout << planes_size << " vs " << colors_size << std::endl;



    std::vector<CloudT::Ptr> planes_clouds;
    std::vector<std::string> planes_ids;
    std::vector<std::vector<double>> planes_colors;
    int plane_cnt = 0;
    for(auto m_ele:planes_map) {
        int plane_label = m_ele.first;
        std::string plane_id = "plane_" + std::to_string(plane_cnt++);
        std::vector<double> plane_color = colors_map[plane_label];
        planes_clouds.push_back(m_ele.second.makeShared());
        planes_ids.push_back(plane_id);
        planes_colors.push_back(plane_color);
    }
    

    // visualize color cloud.
    PlaneNormalVisualizer vis;
    boost::thread vis_thread(boost::bind(&PlaneNormalVisualizer::Spin, &vis));
    while( vis.init) {
        int key = cv::waitKey(10);
        if(key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        vis.UpdateCloud(planes_clouds, planes_ids, planes_colors);    
    }

    vis_thread.join();

    return 0;
}