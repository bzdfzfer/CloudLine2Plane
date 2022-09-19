#include "plane_extraction.h"

#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "tictoc.h"


template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

PlaneExtraction::PlaneExtraction() {
    m_frame_cnt = 0;
}

void PlaneExtraction::loadParams(std::string config_file) {

    std::ifstream fin(config_file);
    if (fin.fail()) {
        std::cout << ">>>>>  could not open " << config_file.c_str() << std::endl;
        exit(-1);
    }
    YAML::Node root = YAML::Load(fin);
    fin.close();

    std::cout << "LOADING parameters .... " << std::endl;

    //-------------------------------------------------------------------------------------------
    // set lidar intrinsic parameters.
    std::string lidar_name;


    root["lidar_name"] >> lidar_name;


    ProjectionParams proj_params;
	if(lidar_name == "VLP-16") {
		fprintf(stderr, "set VLP_16 params \n");
	 	proj_params = ProjectionParams::VLP_16();
	 }
	 else if(lidar_name == "HDL-32") {
		fprintf(stderr, "set HDL_32 params \n");	 	
	 	proj_params = ProjectionParams::HDL_32();

	 }	 
	 else if(lidar_name == "VLP-32C"){
		fprintf(stderr, "set VLP_32C params \n");	 		 	
	 	proj_params = ProjectionParams::VLP_32C();
	 }	
	 else if(lidar_name == "HDL-64") {
		fprintf(stderr, "set HDL_64 params \n");	 		 		 	
	 	proj_params = ProjectionParams::HDL_64();
	 } else if(lidar_name == "SegCompPerceptron") {
        fprintf(stderr, "set SegCompPerceptron params \n");
        proj_params = ProjectionParams::SegCompPerceptron();
     }

    bool reverse_flag;
    root["reverse_flag"] >> reverse_flag;
    if(reverse_flag) 
    {
        proj_params.reserseVerticalParams();

  	    float h_min_ang, v_min_ang;
  	    float h_max_ang, v_max_ang;
  	    int num_hbeam, num_vbeam;
        root["h_min_ang"] >> h_min_ang;
        root["h_max_ang"] >> h_max_ang;
        root["num_hbeam"] >> num_hbeam;

        // reset h angle parameters.
        proj_params.setScan(ScanParams(h_min_ang, h_max_ang, num_hbeam),
                    ScanParams::Direction::HORIZONTAL);
        proj_params.reserseHorizontalParams();
      	proj_params.FillCosSin();
    }    

    m_proj_params = proj_params;

    //-------------------------------------------------------------------------------------------
    // set line extraction parameters.

    // set projection params.
    m_le.setProjectionParams(proj_params);
    m_le.initLF(proj_params);

    int v_min_line_points, v_seed_line_points, v_pts_missing_tolerance;
    int h_min_line_points, h_seed_line_points, h_pts_missing_tolerance;
    double v_least_thresh, v_min_line_length, v_predict_distance, v_max_pts_gap;
    double h_least_thresh, h_min_line_length, h_predict_distance, h_max_pts_gap;

    root["v_min_line_points"] >> v_min_line_points;
    root["v_seed_line_points"] >> v_seed_line_points;
    root["v_least_thresh"] >> v_least_thresh;
    root["v_min_line_length"] >> v_min_line_length;
    root["v_predict_distance"] >> v_predict_distance;
    root["v_pts_missing_tolerance"] >> v_pts_missing_tolerance;
    root["v_max_pts_gap"] >> v_max_pts_gap;

    m_le.setVLFLeastThreshold(v_least_thresh);
    m_le.setVLFMinLineLength(v_min_line_length);
    m_le.setVLFPredictDistance(v_predict_distance);
    m_le.setVLFSeedLinePtsNum(v_seed_line_points);
    m_le.setVLFMinLinePtsNum(v_min_line_points);
    m_le.setVLFMissingPtsTolerance(v_pts_missing_tolerance);
    m_le.setVLFMaxPtsGap(v_max_pts_gap);

    root["h_min_line_points"] >> h_min_line_points;
    root["h_seed_line_points"] >> h_seed_line_points;
    root["h_least_thresh"] >> h_least_thresh;
    root["h_min_line_length"] >> h_min_line_length;
    root["h_predict_distance"] >> h_predict_distance;
    root["h_pts_missing_tolerance"] >> h_pts_missing_tolerance;
    root["h_max_pts_gap"] >> h_max_pts_gap;


    m_le.setHLFLeastThreshold(h_least_thresh);
    m_le.setHLFMinLineLength(h_min_line_length);
    m_le.setHLFPredictDistance(h_predict_distance);
    m_le.setHLFSeedLinePtsNum(h_seed_line_points);
    m_le.setHLFMinLinePtsNum(h_min_line_points);
    m_le.setHLFMissingPtsTolerance(h_pts_missing_tolerance);
    m_le.setHLFMaxPtsGap(h_max_pts_gap);

    //-------------------------------------------------------------------------------------------
    // set line clustering parameters.
    float line_plane_distThreshold;
    root["line_plane_distThreshold"] >> line_plane_distThreshold;

    m_lc.setProjectionParams(proj_params);
    m_lc.setLinePlaneDistThreshold(line_plane_distThreshold);    
}

void PlaneExtraction::segment(const CloudT::Ptr& cloud_in_ptr, bool colwise) {

	// fprintf(stderr, " received frame: %d\n", m_frame_cnt ++);

    // 1. extract line segments.
	m_le.extractOrganizedCloud(cloud_in_ptr, colwise);

    // output of line segments.
    auto vsweep_lines = m_le.getVSweepLines();
    auto hsweep_lines = m_le.getHSweepLines();
    // auto vsweep_lines_ptr = m_le.getVSweepLinesPtr();
    // auto hsweep_lines_ptr = m_le.getHSweepLinesPtr();    
    auto hl_image = m_le.getHLImage();
    auto vl_image = m_le.getVLImage();
    auto valid_image = m_le.getValidImage();

    // 2.  cluster line segments.
    // m_lc.run(hsweep_lines_ptr, vsweep_lines_ptr, hl_image, vl_image, valid_image);
    m_lc.run(hsweep_lines, vsweep_lines, hl_image, vl_image, valid_image);

    // results.
    m_plabel_image = m_lc.getPlaneLabelImage();
    m_pl_params_vec = m_lc.getPlaneParamsVec();

}

void PlaneExtraction::segment_unordered(const CloudT::Ptr& cloud_in_ptr, bool colwise) {

	// fprintf(stderr, " received frame: %d\n", m_frame_cnt ++);

    // 1. extract line segments.
	m_le.extract(cloud_in_ptr);

    // output of line segments.
    auto vsweep_lines = m_le.getVSweepLines();
    auto hsweep_lines = m_le.getHSweepLines();
    // auto vsweep_lines_ptr = m_le.getVSweepLinesPtr();
    // auto hsweep_lines_ptr = m_le.getHSweepLinesPtr();    
    auto hl_image = m_le.getHLImage();
    auto vl_image = m_le.getVLImage();
    auto valid_image = m_le.getValidImage();

    // 2.  cluster line segments.
    // m_lc.run(hsweep_lines_ptr, vsweep_lines_ptr, hl_image, vl_image, valid_image);

    TicToc timer;

    timer.Tic();
    m_lc.run(hsweep_lines, vsweep_lines, hl_image, vl_image, valid_image);
    double pe_time = timer.Toc();

    pe_times.push_back(pe_time);

    // results.
    m_plabel_image = m_lc.getPlaneLabelImage();
    m_pl_params_vec = m_lc.getPlaneParamsVec();

}