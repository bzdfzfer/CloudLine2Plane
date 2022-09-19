#ifndef LINE_CLUSTERING_H
#define LINE_CLUSTERING_H

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <queue>
#include <deque>


#include "plane_params.h"
#include "projection_params.h"

class LineClustering {
public:

	LineClustering();
	
	LineClustering(const ProjectionParams& proj_params);

//-------------------------------------------------------------------
	// initialize function.
	void setProjectionParams(const ProjectionParams& proj_params);

//-------------------------------------------------------------------
	// process function.
	void run(const std::vector<std::vector<line>>& hsweep_lines,
		     const std::vector<std::vector<line>>& vsweep_lines,
		     const cv::Mat& hl_image,
		     const cv::Mat& vl_image,
		     const cv::Mat& valid_image);

	void setInput(const std::vector<std::vector<line>>& hsweep_lines,
				  const std::vector<std::vector<line>>& vsweep_lines,
				  const cv::Mat& hl_image,
				  const cv::Mat& vl_image,
				  const cv::Mat& valid_image);


	void clearOutput();

	void labelPlanes();

	void detectSingleDirectionPlanes();

	// labelPlanes and detectsingledirectionplanes together.
	void labelPlanesTogether();


	void resetPlaneLabelOf(PlaneParams plane, uint16_t label);

	void clearLineLabelImageFromPlaneLabel();


    void setLinePlaneDistThreshold(float th) { m_lp_dist_threshold = th; }

//-------------------------------------------------------------------
	// output function.
	cv::Mat getPlaneLabelImage();
	int getPlaneNum();
	std::vector<PlaneParams> getPlaneParamsVec();
	std::vector<PixelCoord> getPlaneSeedsCoords();
	std::vector<PixelCoord> getPlaneCenterCoords();
	CloudT getPlaneCenterPoints();

	pcl::PointCloud<pcl::PointXYZ> castLineSegEndPoints(const line& la);
	std::vector<PixelCoord> castEndPointsPixCoords(const line& la, 
												   bool vertical_flag = true);	

protected:

//-------------------------------------------------------------------
	// label access and set function.
	uint16_t VLineLabelAt(PixelCoord coord) {
		return m_vl_image.at<uint16_t>(coord.row, coord.col);
	}

	uint16_t VLineLabelAt(int row, int col) {
		return m_vl_image.at<uint16_t>(row, col);
	}

	uint16_t HLineLabelAt(int row, int col) {
		return m_hl_image.at<uint16_t>(row, col);
	}

	uint16_t HLineLabelAt(PixelCoord coord) {
		return m_hl_image.at<uint16_t>(coord.row, coord.col);
	}

	uint16_t PlaneLabelAt(PixelCoord coord) {
		return m_plabel_image.at<uint16_t>(coord.row, coord.col);
	}

	uint16_t PlaneLabelAt(int row, int col) {
		return m_plabel_image.at<uint16_t>(row, col);
	}

	void setVLineLabel(PixelCoord coord, uint16_t label) {
		m_vl_image.at<uint16_t>(coord.row, coord.col) = label;
	}

	void setVLineLabel(int row, int col, uint16_t label) {
		m_vl_image.at<uint16_t>(row, col) = label;
	}

	void setHLineLabel(PixelCoord coord, uint16_t label) {
		m_hl_image.at<uint16_t>(coord.row, coord.col) = label;
	}

	void setHLineLabel(int row, int col, uint16_t label) {
		m_hl_image.at<uint16_t>(row, col) = label;
	}


	void SetPlaneLabel(PixelCoord coord, uint16_t label) {
		m_plabel_image.at<uint16_t>(coord.row, coord.col)  = label;
	}

	void SetPlaneLabel(int row, int col, uint16_t label) {
		m_plabel_image.at<uint16_t>(row, col) = label;
	}


private:
//-------------------------------------------------------------------
// implementation details.
	bool detectPlaneSeedAt(const int& row, 
						   const  int& col, 
						   PlaneParams& plane_param);
	void labelOnePlaneBFS(int row, 
						  int col, 
						  PlaneParams& plane_param, 
						  uint16_t label);

	void clearCheckedSweepLines(std::vector<std::vector<line>>& sweep_lines, 
								std::vector<PixelCoord> checked_idxs);
    void clearCheckedSweepLines(
							std::vector<std::vector<line>*>& sweep_lines_ptr, 
							std::vector<PixelCoord> checked_idxs);

	bool detectSDPlaneSeedAt(int row, 
							 int col, 
							 PlaneParams& plane_param);
	void labelOneSDPlaneBFS(int row, 
							int col, 
							PlaneParams& plane_param, 
							uint16_t label);

	int detectPlaneSeedTogetherAt(int row, 
								   int col,
								   PlaneParams& plane_param);
	void labelOnePlaneTogetherBFS(int row, 
								  int col, 
								  PlaneParams& plane_param, 
								  uint16_t label);


	// neighbor line search function.
	void findNeighorsOnPlane(const std::vector<PixelCoord>& Neighborhood, 
							 std::queue<PixelCoord>& label_queue,
							 std::vector<PixelCoord>& checked_lineseg_idxs,
							 PlaneParams& plane_param,
							 uint16_t l_label,
							 bool v_flag = true);

	bool findVNeighborsOfHLine(int nrow, 
							   line line_current, 
							   std::vector<line>& line_neighbors,
							   std::vector<PixelCoord>& ln_indexes);
	
	bool findHNeighborsOfVLine(int ncol, 
							   line line_current, 
							   std::vector<line>& line_neighbors,
							   std::vector<PixelCoord>& ln_indexes);

	// line check function.
	bool checkTwoLinesCoPlane(line la, line lb);
	bool checkThreeLinesCoPlane(line la, 
								line lb, 
								line lc, 
								PlaneParams& plane);
	std::vector<int> checkMaskedEndPoints(line la, 
										  line lb, 
										  line lc, 
									      bool vertical_flag = true);
	bool lineFallenOnPlane(line la, 
                           PlaneParams plane_param,
                           float dist_threshold = 0.15);
	bool pointFallOnPlane(PointT point, 
                        PlaneParams plane,
                        float dist_threshold = 0.15);
	PlaneParams fitPlaneFromTwoLines(line la, line lb);
	PlaneParams fitPlaneFromLines(std::vector<line> lines);
	PlaneParams fitPlanePCA(const CloudT::Ptr& cloud_ptr);

	// plane parameter function.
	void updatePlaneMidVariables(line la, 
							 	 PlaneParams& plane_param, 
							 	 bool vertical_flag = true);
	void updatePlaneMidVariables2(line la,
								 PlaneParams& plane_param);

	// find convex hull
	void findPlaneContourAndConvexHull(PlaneParams& plane_param);	
	PointT computeCenterPoint(PlaneParams plane, PixelCoord coord);
	PointT computeVerticalPoint(PlaneParams plane);
	std::vector<PixelCoord> calculateConvexHull(
		std::vector<std::pair<PixelCoord, PixelCoord>> lineseg_idxs);


private:
	// parameters.
	ProjectionParams m_proj_params;

	float m_lp_dist_threshold = 0.1;

	// input.
	std::vector<std::vector<line>> m_vsweep_lines;	
	std::vector<std::vector<line>> m_hsweep_lines;

	// middle variables.
	cv::Mat m_vl_image;
	cv::Mat m_hl_image;
	cv::Mat m_valid_image;

	Eigen::Matrix3f m_pts_cov33;
	Eigen::Vector3f m_pts_sum31;
	int m_lineOnPlane_cnt;

	uint16_t m_label = 0;

	// variables invovling merging and conflicts.
	cv::Mat m_conflicts_image;
	std::vector<std::vector<PlaneContainer>> m_plane_containers;

	// output.
	cv::Mat m_plabel_image;
	std::vector<PlaneParams> m_plane_params;
};


#endif