#include "line_extraction.h"

#include "tictoc.h"


LineExtraction::LineExtraction() {

}

LineExtraction::LineExtraction(const ProjectionParams& params ) {
    // setProjectionParams();
    m_proj_params = params;

    // initialize linefeature.		
    initLF(params);
}

void LineExtraction::setProjectionParams(const ProjectionParams& params) {
    m_proj_params = params;
}

ProjectionParams LineExtraction::getProjectionParams() {
    return m_proj_params;
}

void LineExtraction::initLF(const ProjectionParams& params) {
    // here we choose column-wise scan to extract lines.
    // 1. set angles, angle_cosines, angle_sines, indexes.
    std::vector<double> v_bearings(params.RowAngles().begin(), params.RowAngles().end());
    std::vector<double> cos_v_bearings(params.RowAngleCosines().begin(), params.RowAngleCosines().end()); 
    std::vector<double> sin_v_bearings(params.RowAngleSines().begin(), params.RowAngleSines().end());
    std::vector<unsigned int> v_index(params.rows());
    std::iota(v_index.begin(), v_index.end(), 0);	
    m_lf_vertical.setCosSinData(v_bearings, cos_v_bearings, sin_v_bearings, v_index);

    // 2. set angle start and step. 
    m_lf_vertical.set_angle_start(params.v_start_angle()*M_PI/180.0);
    m_lf_vertical.set_angle_increment(params.v_step()*M_PI/180.0);		


    // a row-wise scan is also added.
    // 1. set angles, angle_cosines, angle_sines, indexes.
    std::vector<double> h_bearings(params.ColAngles().begin(), params.ColAngles().end());
    std::vector<double> cos_h_bearings(params.ColAngleCosines().begin(), params.ColAngleCosines().end()); 
    std::vector<double> sin_h_bearings(params.ColAngleSines().begin(), params.ColAngleSines().end());
    std::vector<unsigned int> h_index(params.cols());
    std::iota(h_index.begin(), h_index.end(), 0);	
    m_lf_horizontal.setCosSinData(h_bearings, cos_h_bearings, sin_h_bearings, h_index);

    // 2. set angle start and step. 
    m_lf_horizontal.set_angle_start(params.h_start_angle()*M_PI/180.0);
    m_lf_horizontal.set_angle_increment(params.h_step()*M_PI/180.0);	    	
}

void LineExtraction::setVLFLeastThreshold(double val) {
    m_lf_vertical.set_least_threshold(val);
}

void LineExtraction::setVLFMinLineLength(double val) {
    m_lf_vertical.set_min_line_length(val);
}

void LineExtraction::setVLFPredictDistance(double val) {
    m_lf_vertical.set_predict_distance(val);
}

void LineExtraction::setVLFSeedLinePtsNum(int num) {
    m_lf_vertical.set_seed_line_points(num);
}

void LineExtraction::setVLFMinLinePtsNum(int num) {
    m_lf_vertical.set_min_line_points(num);
}

void LineExtraction::setVLFMissingPtsTolerance(int num) {
    m_lf_vertical.set_pts_missing_tolerance(num);
}

void LineExtraction::setVLFMaxPtsGap(double gap) {
    m_lf_vertical.set_max_pts_gap(gap);
}

void LineExtraction::setHLFLeastThreshold(double val) {
    m_lf_horizontal.set_least_threshold(val);
}

void LineExtraction::setHLFMinLineLength(double val) {
    m_lf_horizontal.set_min_line_length(val);
}

void LineExtraction::setHLFPredictDistance(double val) {
    m_lf_horizontal.set_predict_distance(val);
}

void LineExtraction::setHLFSeedLinePtsNum(int num) {
    m_lf_horizontal.set_seed_line_points(num);
}

void LineExtraction::setHLFMinLinePtsNum(int num) {
    m_lf_horizontal.set_min_line_points(num);
}

void LineExtraction::setHLFMissingPtsTolerance(int num) {
    m_lf_horizontal.set_pts_missing_tolerance(num);
}

void LineExtraction::setHLFMaxPtsGap(double gap) {
    m_lf_horizontal.set_max_pts_gap(gap);
}

void LineExtraction::extract(const CloudT::Ptr& cloud_in_ptr) {

    CloudProjection* proj_ptr = new CloudProjection(m_proj_params);	

	TicToc timer;
	timer.Tic();
    // m_proj_params.printLidarType();
    proj_ptr->InitFromPoints(*cloud_in_ptr);
	double proj_time = timer.Toc();

    // for debug.
    m_depth_image = proj_ptr->depth_image();
    m_x_image = proj_ptr->x_image();
    m_y_image = proj_ptr->y_image();
    m_z_image = proj_ptr->z_image();

    m_valid_image = proj_ptr->valid_image();

    // output: x_image, y_image, z_image, rxy_image, r_image.
	
    timer.Tic();
    
    processVerticalScans(*proj_ptr);
    processHorizontalScans(*proj_ptr);


    generateLineIdxImage(m_vsweep_lines, m_vl_image, true);
    generateLineIdxImage(m_hsweep_lines, m_hl_image, false);

	double lse_time = timer.Toc();

    cpr_times.push_back(proj_time);
    lse_times.push_back(lse_time);
}

void LineExtraction::extractOrganizedCloud(const CloudT::Ptr& cloud_in_ptr, bool colwise)
{
    CloudProjection* proj_ptr = new CloudProjection(m_proj_params);	

    // m_proj_params.printLidarType();
    proj_ptr->InitFromOrganizedPoints(*cloud_in_ptr, colwise);

    // for debug.
    m_depth_image = proj_ptr->depth_image();
    m_x_image = proj_ptr->x_image();
    m_y_image = proj_ptr->y_image();
    m_z_image = proj_ptr->z_image();

    m_valid_image = proj_ptr->valid_image();
    // output: x_image, y_image, z_image, rxy_image, r_image.

    processVerticalScans(*proj_ptr);
    processHorizontalScans(*proj_ptr);

    generateLineIdxImage(m_vsweep_lines, m_vl_image, true);
    generateLineIdxImage(m_hsweep_lines, m_hl_image, false);    
}


std::vector<std::vector<gline3d>> 
  LineExtraction::getVSweepGlines() { return m_vsweep_glines; }
std::vector<std::vector<line>> 
  LineExtraction::getVSweepLines() { return m_vsweep_lines; }
std::vector<std::vector<gline3d>> 
  LineExtraction::getHSweepGlines() { return m_hsweep_glines; }
std::vector<std::vector<line>> 
  LineExtraction::getHSweepLines() { return m_hsweep_lines; }

std::vector<std::vector<line>*> 
  LineExtraction::getVSweepLinesPtr() { return m_vsweep_lines_ptr; }
std::vector<std::vector<line>*> 
  LineExtraction::getHSweepLinesPtr() { return m_hsweep_lines_ptr; }


cv::Mat LineExtraction::getDepthImage() { return m_depth_image; }
cv::Mat LineExtraction::getXImage() { return m_x_image; }
cv::Mat LineExtraction::getYImage() { return m_y_image; }
cv::Mat LineExtraction::getZImage() { return m_z_image; }

cv::Mat LineExtraction::getVLImage() { return m_vl_image; }
cv::Mat LineExtraction::getHLImage() { return m_hl_image; }

cv::Mat LineExtraction::getValidImage() { return m_valid_image; }


std::vector<float> LineExtraction::getColumnVecFromMat(const cv::Mat& mat, int c) {
    cv::Mat col_vec = mat.col(c);
    return std::vector<float>(col_vec.begin<float>(), col_vec.end<float>());
}

std::vector<float> LineExtraction::getRowVecFromMat(const cv::Mat& mat, int r) {
    cv::Mat row_vec = mat.row(r);
    return std::vector<float>(row_vec.begin<float>(), row_vec.end<float>());
}

void LineExtraction::processVerticalScans(const CloudProjection& cloud_proj) {
    const cv::Mat& r_image = cloud_proj.depth_image();
    const cv::Mat& rxy_image = cloud_proj.rxy_image();
    const cv::Mat& x_image = cloud_proj.x_image();
    const cv::Mat& y_image = cloud_proj.y_image();
    const cv::Mat& z_image = cloud_proj.z_image();

    const int col_size = cloud_proj.cols();
    
    // resize output.
    if(m_vsweep_lines.size() != col_size)
        m_vsweep_lines.resize(col_size);
    if(m_vsweep_glines.size() != col_size)
        m_vsweep_glines.resize(col_size);
    if(m_vsweep_lines_ptr.size() != col_size)
        m_vsweep_lines_ptr.resize(col_size);

    // process (rxy, z)
    // #pragma omp parallel for
    for(int i=0; i < col_size; i++) {
        std::vector<float> col_xs, col_ys, col_zs, col_rs, col_rxys;
        col_rs = getColumnVecFromMat(r_image, i);
        col_rxys = getColumnVecFromMat(rxy_image, i);
        col_xs = getColumnVecFromMat(x_image, i);
        col_ys = getColumnVecFromMat(y_image, i);
        col_zs = getColumnVecFromMat(z_image, i);

        // std::vector<double> col_xsd(col_xs.begin(), col_xs.end());
        // std::vector<double> col_ysd(col_ys.begin(), col_ys.end());
        std::vector<double> col_zsd(col_zs.begin(), col_zs.end());
        std::vector<double> col_rsd(col_rs.begin(), col_rs.end());
        std::vector<double> col_rxysd(col_rxys.begin(), col_rxys.end());

        m_lf_vertical.setRangeDataNew(col_rxysd, col_zsd, col_rsd);

        std::vector<line> lines;
        std::vector<gline3d> glines;
        m_lf_vertical.extractLinesNew(lines, glines, i);

        // get 3d end point with col_xs, col_ys.
        for(int j=0; j < lines.size(); j++) {
            int l_idx = lines[j].left;
            int r_idx = lines[j].right;
            int m_idx = lines[j].mid;

            glines[j].x1 = static_cast<double>(col_xs[l_idx]);
            glines[j].y1 = static_cast<double>(col_ys[l_idx]);
            glines[j].z1 = static_cast<double>(col_zs[l_idx]);

            glines[j].x2 = static_cast<double>(col_xs[r_idx]);
            glines[j].y2 = static_cast<double>(col_ys[r_idx]);
            glines[j].z2 = static_cast<double>(col_zs[r_idx]);

            lines[j].pm.x = static_cast<double>(col_xs[m_idx]);
            lines[j].pm.y = static_cast<double>(col_ys[m_idx]);
            lines[j].zm = static_cast<double>(col_zs[m_idx]);

            lines[j].p1.x = glines[j].x1;
            lines[j].p1.y = glines[j].y1;
            lines[j].z1 = glines[j].z1;

            lines[j].p2.x = glines[j].x2;
            lines[j].p2.y = glines[j].y2;
            lines[j].z2 = glines[j].z2;

        }

        // collect output data.
        m_vsweep_lines[i] = lines;
        m_vsweep_glines[i] = glines;
        m_vsweep_lines_ptr[i] = &lines;
    }
}

void LineExtraction::processHorizontalScans(const CloudProjection& cloud_proj) {
    const cv::Mat& r_image = cloud_proj.depth_image();
    const cv::Mat& rxy_image = cloud_proj.rxy_image();
    const cv::Mat& x_image = cloud_proj.x_image();
    const cv::Mat& y_image = cloud_proj.y_image();
    const cv::Mat& z_image = cloud_proj.z_image();

    const int row_size = cloud_proj.rows();
    
    // resize output.
    if(m_hsweep_lines.size() != row_size)
        m_hsweep_lines.resize(row_size);
    if(m_hsweep_glines.size() != row_size)
        m_hsweep_glines.resize(row_size);
    if(m_hsweep_lines_ptr.size() != row_size)
        m_hsweep_lines_ptr.resize(row_size);
    // process (x, y)
    // #pragma omp parallel for
    for(int i=0; i < row_size; i++) {
        std::vector<float> row_xs, row_ys, row_zs, row_rxys;
        row_xs = getRowVecFromMat(x_image, i);
        row_ys = getRowVecFromMat(y_image, i);
        row_zs = getRowVecFromMat(z_image, i);
        row_rxys = getRowVecFromMat(rxy_image, i);

        std::vector<double> row_xsd(row_xs.begin(), row_xs.end());
        std::vector<double> row_ysd(row_ys.begin(), row_ys.end());
        std::vector<double> row_rxysd(row_rxys.begin(), row_rxys.end());

        m_lf_horizontal.setRangeDataNew(row_xsd, row_ysd, row_rxysd);

        std::vector<line> lines;
        std::vector<gline3d> glines;
        m_lf_horizontal.extractLinesNew(lines, glines, i);

        // get 3d end point with row_xs, row_ys.
        for(int j=0; j < lines.size(); j++) {
            // int l_idx = lines[j].left;
            // int r_idx = lines[j].right;

            // glines[j].x1 = static_cast<double>(row_xs[l_idx]);
            // glines[j].y1 = static_cast<double>(row_ys[l_idx]);
            // glines[j].z1 = static_cast<double>(row_zs[l_idx]);

            // glines[j].x2 = static_cast<double>(row_xs[r_idx]);
            // glines[j].y2 = static_cast<double>(row_ys[r_idx]);
            // glines[j].z2 = static_cast<double>(row_zs[r_idx]);

            int l_idx = lines[j].left;
            int r_idx = lines[j].right;
            int m_idx = lines[j].mid;

            glines[j].x1 = static_cast<double>(row_xs[l_idx]);
            glines[j].y1 = static_cast<double>(row_ys[l_idx]);
            glines[j].z1 = static_cast<double>(row_zs[l_idx]);

            glines[j].x2 = static_cast<double>(row_xs[r_idx]);
            glines[j].y2 = static_cast<double>(row_ys[r_idx]);
            glines[j].z2 = static_cast<double>(row_zs[r_idx]);

            lines[j].pm.x = static_cast<double>(row_xs[m_idx]);
            lines[j].pm.y = static_cast<double>(row_ys[m_idx]);
            lines[j].zm = static_cast<double>(row_zs[m_idx]);

            lines[j].p1.x = glines[j].x1;
            lines[j].p1.y = glines[j].y1;
            lines[j].z1 = glines[j].z1;

            lines[j].p2.x = glines[j].x2;
            lines[j].p2.y = glines[j].y2;
            lines[j].z2 = glines[j].z2;				
        }

        // collect output data.
        m_hsweep_lines[i] = lines;
        m_hsweep_glines[i] = glines;
        m_hsweep_lines_ptr[i] = &lines;
    }
}

void LineExtraction::generateLineIdxImage(const std::vector<std::vector<line>>& sweep_lines, 
                            cv::Mat& line_img, 
                            bool colwise_flag) {
    
    line_img = 5000*cv::Mat::ones(m_proj_params.rows(), 
                                    m_proj_params.cols(), 
                                    cv::DataType<uint16_t>::type);

    for(int i=0; i < sweep_lines.size(); i++) {
        std::vector<line> line_segs = sweep_lines[i];
        for(int j=0; j < line_segs.size(); j++) {
            int start_idx = line_segs[j].left;
            int stop_idx = line_segs[j].right;
            for(int k=start_idx; k<=stop_idx; k++) {
                if(colwise_flag) {
                    line_img.at<uint16_t>(k, i) = j;
                } else {
                    line_img.at<uint16_t>(i, k) = j;
                }					
            }
        }
    }
}



