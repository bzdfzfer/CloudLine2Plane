#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <array>
#include <vector>
#include <iostream>
#include <numeric>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <opencv2/opencv.hpp>

#include "cloud_projection.h"
#include "line_feature.h"
#include "common.h"


using line_feature::LineFeature;

class LineExtraction{
public:
    LineExtraction();
    LineExtraction(const ProjectionParams& params );

    void initLF(const ProjectionParams& params);
    void extract(const CloudT::Ptr& cloud_in_ptr);
    void extractOrganizedCloud(const CloudT::Ptr& cloud_in_ptr, bool colwise=true);

    void setProjectionParams(const ProjectionParams& params);    
    void setVLFLeastThreshold(double val);
    void setVLFMinLineLength(double val);
    void setVLFPredictDistance(double val);
    void setVLFSeedLinePtsNum(int num);
    void setVLFMinLinePtsNum(int num);
    void setVLFMissingPtsTolerance(int num);
    void setVLFMaxPtsGap(double gap);
	void setHLFLeastThreshold(double val);
	void setHLFMinLineLength(double val);
	void setHLFPredictDistance(double val);
	void setHLFSeedLinePtsNum(int num);
	void setHLFMinLinePtsNum(int num);
	void setHLFMissingPtsTolerance(int num);
	void setHLFMaxPtsGap(double gap);

	cv::Mat getXImage();
	cv::Mat getYImage();
	cv::Mat getZImage();
	cv::Mat getVLImage();
	cv::Mat getHLImage();
	cv::Mat getValidImage();
	cv::Mat getDepthImage();
    ProjectionParams getProjectionParams();
	std::vector<std::vector<gline3d>> getVSweepGlines();
	std::vector<std::vector<line>> getVSweepLines();
	std::vector<std::vector<gline3d>> getHSweepGlines();
	std::vector<std::vector<line>> getHSweepLines();

	std::vector<std::vector<line>*> getVSweepLinesPtr();
	std::vector<std::vector<line>*> getHSweepLinesPtr();


    std::vector<float> getColumnVecFromMat(const cv::Mat& mat, int c);
	std::vector<float> getRowVecFromMat(const cv::Mat& mat, int r);
	void processVerticalScans(const CloudProjection& cloud_proj);
	void processHorizontalScans(const CloudProjection& cloud_proj);
	void generateLineIdxImage(const std::vector<std::vector<line>>& sweep_lines, 
							  cv::Mat& line_img, 
							  bool colwise_flag = true);


	std::vector<double> getProjectionTimesVec() { return cpr_times; }
	std::vector<double> getLineExtractionTimeVec() { return lse_times; }

private:

	std::vector<std::vector<line>> m_vsweep_lines;
	std::vector<std::vector<gline3d>> m_vsweep_glines;
	std::vector<std::vector<line>> m_hsweep_lines;
	std::vector<std::vector<gline3d>> m_hsweep_glines;

	std::vector<std::vector<line>*> m_vsweep_lines_ptr;
	std::vector<std::vector<line>*> m_hsweep_lines_ptr;


	ProjectionParams m_proj_params;

	LineFeature m_lf_vertical;
	LineFeature m_lf_horizontal;

	cv::Mat m_depth_image;
	cv::Mat m_x_image;
	cv::Mat m_y_image;
	cv::Mat m_z_image;

	cv::Mat m_vl_image;
	cv::Mat m_hl_image;

	cv::Mat m_valid_image;


	std::vector<double> cpr_times;
	std::vector<double> lse_times;
		
};





#endif
