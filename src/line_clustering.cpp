#include "line_clustering.h"

// constructor.
//-------------------------------------------------------------------
LineClustering::LineClustering() {

}

LineClustering::LineClustering(const ProjectionParams& proj_params) {
	m_proj_params = proj_params;
}


// initializer.
//-------------------------------------------------------------------
void LineClustering::setProjectionParams(const ProjectionParams& proj_params) {
	m_proj_params = proj_params;
}

// main process.
//-------------------------------------------------------------------
void LineClustering::run(
	     const std::vector<std::vector<line>>& hsweep_lines,
	     const std::vector<std::vector<line>>& vsweep_lines,
	     const cv::Mat& hl_image,
	     const cv::Mat& vl_image,
	     const cv::Mat& valid_image) {
	setInput(hsweep_lines, vsweep_lines, hl_image, vl_image, valid_image);

	clearOutput();

	// labelPlanes();
	// detectSingleDirectionPlanes();

	labelPlanesTogether();

}

void LineClustering::setInput(
			  const std::vector<std::vector<line>>& hsweep_lines,
			  const std::vector<std::vector<line>>& vsweep_lines,
			  const cv::Mat& hl_image,
			  const cv::Mat& vl_image,
			  const cv::Mat& valid_image) {
	m_hsweep_lines = hsweep_lines;
	m_vsweep_lines = vsweep_lines;

	m_hl_image = hl_image;
	m_vl_image = vl_image;
	m_valid_image = valid_image;
}


void LineClustering::clearOutput() {
	m_plabel_image = cv::Mat::zeros(m_proj_params.rows(), 
									m_proj_params.cols(),
									cv::DataType<uint16_t>::type);
	m_plane_params.clear();
}

void LineClustering::labelPlanes() {
	m_label = 1;
	if(m_proj_params.lidar_type() == ProjectionParams::LIDARType::eSEGCOMPPERCEPTRON) {
		for(int row =0; row < m_proj_params.rows(); row ++) {
		// for(int row =m_proj_params.rows()-1; row >= 0; row --) {
			for(int col = 0; col < m_proj_params.cols(); col ++) {		
				if(PlaneLabelAt(row, col)>0)
					continue;

				PlaneParams plane_param;
				if(detectPlaneSeedAt(row, col, plane_param)) {
                    
					labelOnePlaneBFS(row, col, plane_param, m_label);
					plane_param.m_plabel = m_label;
					plane_param.m_vertical_pt = computeVerticalPoint(plane_param);

					if(m_lineOnPlane_cnt>3) {
						// findPlaneContourAndConvexHull(plane_param);

						m_plane_params.push_back(plane_param);
						m_label ++;
					}
					else { // clear plabel image with m_label.
						resetPlaneLabelOf(plane_param, m_label);
					}
				}
			}
		}
	}
	else {
		// for(int row =0; row < m_proj_params.rows(); row ++) {
		for(int row =m_proj_params.rows()-1; row >= 0; row --) {
			for(int col = 0; col < m_proj_params.cols(); col ++) {		
				if(PlaneLabelAt(row, col)>0)
					continue;

				PlaneParams plane_param;
				if(detectPlaneSeedAt(row, col, plane_param)) {
					labelOnePlaneBFS(row, col, plane_param, m_label);
					plane_param.m_plabel = m_label;
					plane_param.m_vertical_pt = computeVerticalPoint(plane_param);

					if(m_lineOnPlane_cnt>3) {
						// findPlaneContourAndConvexHull(plane_param);

						m_plane_params.push_back(plane_param);
						m_label ++;
					}
					else { // clear plabel image with m_label.
						resetPlaneLabelOf(plane_param, m_label);
					}
				}
			}
		}		
	}
}

void LineClustering::detectSingleDirectionPlanes() {

	clearLineLabelImageFromPlaneLabel();

	for(int row=0; row < m_proj_params.rows(); row++) {
	// for(int row=m_proj_params.rows()-1; row >= 0; row--) {
		for(int col=0; col < m_proj_params.cols(); col++) {
			if(PlaneLabelAt(row, col)>0)
				continue;

			PlaneParams plane_param;
			if(detectSDPlaneSeedAt(row, col, plane_param)) {
				labelOneSDPlaneBFS(row, col, plane_param, m_label);

				plane_param.m_vertical_pt = computeVerticalPoint(plane_param);

				if(m_lineOnPlane_cnt>2) {
					// findPlaneContourAndConvexHull(plane_param);
					m_plane_params.push_back(plane_param);
					m_label ++;						
				} 
				else { // clear plabel image with m_label.
					resetPlaneLabelOf(plane_param, m_label);
				}
			}
		}
	}

}

void LineClustering::labelPlanesTogether() {
	m_label = 1;

	// for(int row =m_proj_params.rows()-1; row >= 0; row --) {
	for(int row=0; row < m_proj_params.rows(); row++) {

		for(int col = 0; col < m_proj_params.cols(); col ++) {		
			if(PlaneLabelAt(row, col)>0)
				continue;

			PlaneParams plane_param;
			int seed_type = detectPlaneSeedTogetherAt(row, col, plane_param);
			if(seed_type > 0) {
				labelOnePlaneTogetherBFS(row, col, plane_param, m_label);
				plane_param.m_plabel = m_label;
				plane_param.m_vertical_pt = computeVerticalPoint(plane_param);

				if(m_lineOnPlane_cnt>3) {
					// findPlaneContourAndConvexHull(plane_param);

					m_plane_params.push_back(plane_param);
					m_label ++;
				}
				else { // clear plabel image with m_label.
					resetPlaneLabelOf(plane_param, m_label);
				}
			}
		}
	}	

}

void LineClustering::resetPlaneLabelOf(PlaneParams plane, uint16_t label) {
	for(int i=0; i < plane.m_line_seg_idxs.size(); i++) {
		PixelCoord left_coord = plane.m_line_seg_idxs[i].first;
		PixelCoord right_coord = plane.m_line_seg_idxs[i].second;

		if(left_coord.row == right_coord.row) {
			for(int j=left_coord.col; j <= right_coord.col; j++ ) {
				if(PlaneLabelAt(left_coord.row, j)==label)
					SetPlaneLabel(left_coord.row, j, 0);
			}
		} 
		else {
			for(int j=left_coord.row; j <= right_coord.row; j++) {
				if(PlaneLabelAt(j, left_coord.col)==label)
					SetPlaneLabel(j, left_coord.col, 0);
			}
		}
	}
}

void LineClustering::clearLineLabelImageFromPlaneLabel() {
	for(int r=0; r < m_proj_params.rows(); r++) {
		for(int c=0; c < m_proj_params.cols(); c++) {
			if(PlaneLabelAt(r,c) > 0) {
				setHLineLabel(r,c, 5000); 
				setVLineLabel(r,c, 5000);
			}
		}
	}
}

// output function.
//-------------------------------------------------------------------
cv::Mat LineClustering::getPlaneLabelImage() {
	return m_plabel_image;
}

int LineClustering::getPlaneNum() {
	return int(m_label-1);
}

std::vector<PlaneParams> LineClustering::getPlaneParamsVec() {
	return m_plane_params;
}

std::vector<PixelCoord> LineClustering::getPlaneSeedsCoords() {
	std::vector<PixelCoord> seed_coords;

	for(int i=0; i < m_plane_params.size(); i++) {
		seed_coords.push_back(
			PixelCoord(m_plane_params[i].m_seed_row, 
					   m_plane_params[i].m_seed_col)
			);
	}
	return seed_coords;
}

std::vector<PixelCoord> LineClustering::getPlaneCenterCoords() {
	std::vector<PixelCoord> center_pixs;
	for(int i=0; i < m_plane_params.size(); i++) {
		center_pixs.push_back(m_plane_params[i].m_center_pix);
	}
	return center_pixs;
}

pcl::PointCloud<pcl::PointXYZ> LineClustering::getPlaneCenterPoints() {
	pcl::PointCloud<pcl::PointXYZ> res;
	for(int i=0; i < m_plane_params.size(); i++) {
		res.push_back(m_plane_params[i].m_center_pt);
	}
	return res;
}

pcl::PointCloud<pcl::PointXYZ> LineClustering::castLineSegEndPoints(
	const line& la) {
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl::PointXYZ pt;
    // line a: two end points, one mid points.
    pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
    cloud_out.push_back(pt);
    pt.x = la.p2.x; pt.y = la.p2.y; pt.z = la.z2;
    cloud_out.push_back(pt);
    pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
    cloud_out.push_back(pt);
    return cloud_out;
}	

std::vector<PixelCoord> LineClustering::castEndPointsPixCoords(
											 const line& la, 
											 bool vertical_flag) {
	std::vector<PixelCoord> coords_out;
	if(vertical_flag) {
		coords_out.push_back(PixelCoord(la.left, la.laserIdx));
		coords_out.push_back(PixelCoord(la.right, la.laserIdx));
		coords_out.push_back(PixelCoord(la.mid, la.laserIdx));
	} else {
		coords_out.push_back(PixelCoord(la.laserIdx, la.left));
		coords_out.push_back(PixelCoord(la.laserIdx, la.right));
		coords_out.push_back(PixelCoord(la.laserIdx, la.mid));			
	}
	return coords_out;
}


// implementation details.
// //-------------------------------------------------------------------

bool LineClustering::detectPlaneSeedAt(const int& row, 
					   const  int& col, 
					   PlaneParams& plane_param) {
	int vllabel = VLineLabelAt(row, col);
	int hllabel = HLineLabelAt(row, col);
	
	if(PlaneLabelAt(row, col)>0)
		return false;

	if(m_valid_image.at<uint16_t>(row, col) == 0)
		return false;

	if(vllabel != 5000 && hllabel != 5000) {
		line lv, lh;

        lv = m_vsweep_lines[col][vllabel];
        lh = m_hsweep_lines[row][hllabel];

        int vl_left, vl_right, hl_left, hl_right;

        vl_left = m_vsweep_lines[col][vllabel].left;
        vl_right = m_vsweep_lines[col][vllabel].right;
        hl_left = m_hsweep_lines[row][hllabel].left;
        hl_right = m_hsweep_lines[row][hllabel].right;

		if( row <= vl_left || row >= vl_right) {
			return false;
		}

		if(col <= hl_left || col >= hl_right) {
			return false;
		}

		if(PlaneLabelAt(row, col-1) || PlaneLabelAt(row, col+1) )
		{
			return false;
		}


		if(PlaneLabelAt(row-1, col) || PlaneLabelAt(row+1, col) )
		{
			return false;
		}

		if(vl_right - vl_left <= 2)  
			return false;

		if(!checkTwoLinesCoPlane(lh, lv)) {
			return false;
		}

		if(PlaneLabelAt(lv.left, col)>0 && PlaneLabelAt(lv.right, col)==0) {
			// fit with 5 points.
			pcl::PointCloud<pcl::PointXYZ> cloud_fit_5pts;
			cloud_fit_5pts += castLineSegEndPoints(lv);
			cloud_fit_5pts += castLineSegEndPoints(lh);
			cloud_fit_5pts.erase(cloud_fit_5pts.begin());
			plane_param = fitPlanePCA(cloud_fit_5pts.makeShared());
		} 
		else if(PlaneLabelAt(lv.left, col)==0 && PlaneLabelAt(lv.right, col)>0) {
			// fit with 5 points.
			pcl::PointCloud<pcl::PointXYZ> cloud_fit_5pts;
			cloud_fit_5pts += castLineSegEndPoints(lv);
			cloud_fit_5pts += castLineSegEndPoints(lh);
			cloud_fit_5pts.erase(cloud_fit_5pts.begin()+1);
			plane_param = fitPlanePCA(cloud_fit_5pts.makeShared());				
		}
		else {
			plane_param = fitPlaneFromTwoLines(lv, lh);
		}


		plane_param.m_seed_row = row;
		plane_param.m_seed_col = col;

		// then, validity check. 
		// check if it is suitable for the third neighbor line.
		// if suitable, return true for good plane seed.
		// else, return false for bad plane seed.
		std::vector<line> lh_vneighbors;
		std::vector<PixelCoord> lh_vn_indexes;
		bool up_found_flag = findVNeighborsOfHLine(row+1, lh, lh_vneighbors, lh_vn_indexes);
		bool dn_found_flag = findVNeighborsOfHLine(row-1, lh, lh_vneighbors, lh_vn_indexes);

		if(up_found_flag || dn_found_flag) {
			for(auto lh_n : lh_vneighbors) {
				bool on_plane_flag = lineFallenOnPlane(lh_n, plane_param, m_lp_dist_threshold);
				if(!on_plane_flag) {
					return false;
				}
			}

			return true;
		}
	}

	return false;
}


// clustering by pixel.
void LineClustering::labelOnePlaneBFS(int row, 
										   int col, 
										   PlaneParams& plane_param, 
										   uint16_t label) {

	std::queue<PixelCoord> labeling_queue;
	labeling_queue.push(PixelCoord(row, col));

	// size_t max_queue_size = 0;

	// reset parameters.
	m_pts_cov33.setZero();
	m_pts_sum31.setZero();
	m_lineOnPlane_cnt = 0;

	std::vector<PixelCoord> vlinesegs_checked;
	std::vector<PixelCoord> hlinesegs_checked;

	while(!labeling_queue.empty()) {
      	// max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      	// fprintf(stderr, " ------ max queue size: %d \n",
      	//  max_queue_size);

		const PixelCoord current = labeling_queue.front();
		labeling_queue.pop();
		// labeling_queue.pop_front();
		if(PlaneLabelAt(current) > 0)
		{	// we have already labeled this point. No need to add it.
			continue;
		}

		uint16_t vllabel = VLineLabelAt(current);
		uint16_t hllabel = HLineLabelAt(current);

		if(vllabel == 5000 && hllabel == 5000)
			continue;

		SetPlaneLabel(current, label);

		// horizontal line up&down neighbors.
		if(hllabel < 5000) {
			line hline_current;
            hline_current = m_hsweep_lines[current.row][hllabel];


			std::vector<PixelCoord> HNeighborhood;
			// find Neighborhood.
			PixelCoord l_neighbor = current + PixelCoord(0, -1);
			if(l_neighbor.col < 0) 
				l_neighbor.col += m_proj_params.cols();
			if(l_neighbor.col >= hline_current.left &&
				l_neighbor.col <= hline_current.right)
				HNeighborhood.push_back(l_neighbor);

			PixelCoord r_neighbor = current + PixelCoord(0, 1);
			if(r_neighbor.col >= m_proj_params.cols())
				r_neighbor.col -= m_proj_params.cols();
			if(r_neighbor.col <= hline_current.right &&
				r_neighbor.col >= hline_current.left)
				HNeighborhood.push_back(r_neighbor);

			findNeighorsOnPlane(HNeighborhood, 
								labeling_queue, 
								hlinesegs_checked,
								plane_param, 
								hllabel, 
								false);		
		}			
		
		// vertical line left&right neighbors.
		if(vllabel < 5000) {
			line vline_current;
            vline_current = m_vsweep_lines[current.col][vllabel];			
	

			std::vector<PixelCoord> VNeighborhood;
			// find Neighborhood.
			PixelCoord u_neighbor = current + PixelCoord(1, 0);
			if(u_neighbor.row <= vline_current.right) {
				VNeighborhood.push_back(u_neighbor);
			}
			PixelCoord d_neighbor = current + PixelCoord(-1, 0);
			if(d_neighbor.row >= vline_current.left) {
				VNeighborhood.push_back(d_neighbor);
			}

			findNeighorsOnPlane(VNeighborhood, 
								labeling_queue, 
								vlinesegs_checked,
								plane_param, 
								vllabel, 
								true);		
		}
	}

	// clear checked lines.
    clearCheckedSweepLines(m_vsweep_lines, vlinesegs_checked);
    clearCheckedSweepLines(m_hsweep_lines, hlinesegs_checked);
}

void LineClustering::clearCheckedSweepLines(
							std::vector<std::vector<line>>& sweep_lines, 
							std::vector<PixelCoord> checked_idxs) {
	for(int i=0; i < checked_idxs.size(); i++) {
		auto& l_current = sweep_lines[checked_idxs[i].row][checked_idxs[i].col];
		// l_current.inte[0] = l_current.inte[1] = false;
		if(l_current.inte[1]==false)
			l_current.inte[0] = false;
	}
}

void LineClustering::clearCheckedSweepLines(
							std::vector<std::vector<line>*>& sweep_lines_ptr, 
							std::vector<PixelCoord> checked_idxs) {
	for(int i=0; i < checked_idxs.size(); i++) {
		auto& l_current = (*sweep_lines_ptr[checked_idxs[i].row])[checked_idxs[i].col];
		// l_current.inte[0] = l_current.inte[1] = false;
		if(l_current.inte[1]==false)
			l_current.inte[0] = false;
	}
}

bool LineClustering::detectSDPlaneSeedAt(int row, 
											  int col, 
											  PlaneParams& plane_param) {
	int vllabel = VLineLabelAt(row, col);
	int hllabel = HLineLabelAt(row, col);
	
	if(m_valid_image.at<uint16_t>(row, col) == 0)
		return false;

	if(vllabel != 5000) {
		// // find horizontal neighbors of veritcal lines.
		line lv;
		lv = m_vsweep_lines[col][vllabel];


		std::vector<line> lv_hneighbors_l;
		std::vector<line> lv_hneighbors_r;
		std::vector<PixelCoord> lv_hn_indexes;
		int lcol = col-1;
		// bool cross_flag = false;
		if(lcol < 0) {
			lcol += m_proj_params.cols();
			// cross_flag = true;
		}
		int rcol = col+1;
		if(rcol >= m_proj_params.cols()) {
			rcol -= m_proj_params.cols();
			// cross_flag = true;			
		}
		bool l_found_flag = findHNeighborsOfVLine(lcol, lv, lv_hneighbors_l, lv_hn_indexes);
		bool r_found_flag = findHNeighborsOfVLine(rcol, lv, lv_hneighbors_r, lv_hn_indexes);

		// if(cross_flag) {
		// 	fprintf(stderr, "col: %d, left right found flag: %d, %d \n", 
		// 		col, l_found_flag, r_found_flag);
		// 	for(int rev_i = m_proj_params.cols()-1; rev_i>0; rev_i--) {
		// 		if(m_vsweep_lines[rev_i].size()) {
		// 			fprintf(stderr, "last col %d vsweep_lines index: %d, %d\n", 
		// 				rev_i,
		// 				m_vsweep_lines[rev_i][0].left,			
		// 				m_vsweep_lines[rev_i][0].right);	
		// 			break;			
		// 		}				
		// 	}

			
		// }

		if(l_found_flag && r_found_flag) {
			if(PlaneLabelAt(row, lcol) || PlaneLabelAt(row, rcol) )
			{
				return false;
			}

			for(auto& lv_n_l : lv_hneighbors_l) {
				if(lv_n_l.inte[1])
					continue;

				for(auto& lv_n_r : lv_hneighbors_r) {
					if(lv_n_r.inte[1])
						continue;						
					bool three_lines_on_plane;
					three_lines_on_plane = checkThreeLinesCoPlane(lv_n_l, lv, lv_n_r, plane_param);
					if(three_lines_on_plane) {
						std::vector<int> masked_pt_idxs;
						masked_pt_idxs = checkMaskedEndPoints(lv_n_l, lv, lv_n_r, true);

						if(masked_pt_idxs.size()>3) {
							return false;
						} else if(masked_pt_idxs.size()>0) {
							pcl::PointCloud<pcl::PointXYZ> cloud_fit_raw;
							cloud_fit_raw += castLineSegEndPoints(lv_n_l);
							cloud_fit_raw += castLineSegEndPoints(lv);
							cloud_fit_raw += castLineSegEndPoints(lv_n_r);

							// delete masked points.
							int indexCorrection = 0;
							for(int i=0; i < masked_pt_idxs.size(); i++) {
								int remove_idx = masked_pt_idxs[i] + indexCorrection;
								cloud_fit_raw.erase(cloud_fit_raw.begin() + remove_idx);
								indexCorrection --;
							}
							plane_param = fitPlanePCA(cloud_fit_raw.makeShared());
						}

						// if(col < 10 || col > m_proj_params.cols()-10)
						// {
						// 	fprintf(stderr, "---------plane seed on the h boundary %d, %d \n",
						// 		row, col);
						// }
						plane_param.m_seed_row = row;
						plane_param.m_seed_col = col;


						return true;
					}
				}

			}
		}
	} else if(hllabel != 5000) {
		// find vertical neighbors of horizontal lines.
		line lh;
		lh = m_hsweep_lines[row][hllabel];


		std::vector<line> lh_vneighbors_dn;
		std::vector<line> lh_vneighbors_up;
		std::vector<PixelCoord> lh_vn_indexes;
		bool d_found_flag = findVNeighborsOfHLine(row-1, lh, lh_vneighbors_dn, lh_vn_indexes);
		bool u_found_flag = findVNeighborsOfHLine(row+1, lh, lh_vneighbors_up, lh_vn_indexes);

		if(d_found_flag && u_found_flag) {
			if(PlaneLabelAt(row-1, col) || PlaneLabelAt(row+1, col) )
			{
				return false;
			}				

			for(auto& lh_n_dn : lh_vneighbors_dn) {
				if(lh_n_dn.inte[1])
					continue;

				for(auto& lh_n_up: lh_vneighbors_up) {
					if(lh_n_up.inte[1])
						continue;

					bool three_lines_on_plane;
					three_lines_on_plane = checkThreeLinesCoPlane(lh_n_dn, lh, lh_n_up, plane_param);
					if(three_lines_on_plane) {

						std::vector<int> masked_pt_idxs;
						masked_pt_idxs = checkMaskedEndPoints(lh_n_dn, lh, lh_n_up, false);

						if(masked_pt_idxs.size()>3) {
							return false;
						} else if(masked_pt_idxs.size()>0) {
							pcl::PointCloud<pcl::PointXYZ> cloud_fit_raw;
							cloud_fit_raw += castLineSegEndPoints(lh_n_dn);
							cloud_fit_raw += castLineSegEndPoints(lh);
							cloud_fit_raw += castLineSegEndPoints(lh_n_up);

							// delete masked points.
							int indexCorrection = 0;
							for(int i=0; i < masked_pt_idxs.size(); i++) {
								int remove_idx = masked_pt_idxs[i] + indexCorrection;
								cloud_fit_raw.erase(cloud_fit_raw.begin() + remove_idx);
								indexCorrection --;
							}
							plane_param = fitPlanePCA(cloud_fit_raw.makeShared());
						}

						plane_param.m_seed_row = row;
						plane_param.m_seed_col = col;

						return true;
					}
				}					
			}
		}
	}

	return false;
}


void LineClustering::labelOneSDPlaneBFS(
						int row, 
						int col, 
						PlaneParams& plane_param, 
						uint16_t label) {

	std::queue<PixelCoord> labeling_queue;
	labeling_queue.push(PixelCoord(row, col));

	size_t max_queue_size = 0;

	// reset parameters.
	m_pts_cov33.setZero();
	m_pts_sum31.setZero();
	m_lineOnPlane_cnt = 0;

	std::vector<PixelCoord> vlinesegs_checked;
	std::vector<PixelCoord> hlinesegs_checked;

	while(!labeling_queue.empty()) {
      	// max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      	// fprintf(stderr, " ------ max queue size: %d \n",
      	//  max_queue_size);

		const PixelCoord current = labeling_queue.front();
		labeling_queue.pop();
		// labeling_queue.pop_front();
		if(PlaneLabelAt(current) > 0)
		{	// we have already labeled this point. No need to add it.
			continue;
		}

		uint16_t vllabel = VLineLabelAt(current);
		uint16_t hllabel = HLineLabelAt(current);

		if(vllabel == 5000 && hllabel == 5000)
			continue;

		SetPlaneLabel(current, label);

		
		if(hllabel < 5000) {
			line hline_current;
            hline_current = m_hsweep_lines[current.row][hllabel];


			std::vector<PixelCoord> HNeighborhood;
			// find Neighborhood.
			PixelCoord l_neighbor = current + PixelCoord(0, -1);
			if(l_neighbor.col < 0) 
				l_neighbor.col += m_proj_params.cols();
			if(l_neighbor.col >= hline_current.left &&
				l_neighbor.col <= hline_current.right)
				HNeighborhood.push_back(l_neighbor);

			PixelCoord r_neighbor = current + PixelCoord(0, 1);
			if(r_neighbor.col >= m_proj_params.cols())
				r_neighbor.col -= m_proj_params.cols();
			if(r_neighbor.col <= hline_current.right &&
				r_neighbor.col >= hline_current.left)
				HNeighborhood.push_back(r_neighbor);

			// horizontal line left&right neighbors.
			findNeighorsOnPlane(HNeighborhood, 
								labeling_queue, 
								hlinesegs_checked,
								plane_param, 
								hllabel, 
								false);		
			
			// jump from neighbor gaps.



			if(current.col == hline_current.left) {
				if(hllabel > 0) { // this is not the first line segment.
					auto& prev_hline = m_hsweep_lines[current.row][hllabel-1];
					if(!prev_hline.inte[0]) {
						prev_hline.inte[0] = true;
						prev_hline.inte[1] = lineFallenOnPlane(prev_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel-1));							
						if(prev_hline.inte[1]) {
							updatePlaneMidVariables(prev_hline, plane_param, false);
							labeling_queue.push(PixelCoord(current.row, prev_hline.right));
						}
					}  
				} else if(hllabel == 0 ) {
					uint16_t hllabel_left = m_hsweep_lines[current.row].size()-1;
					auto& prev_hline = m_hsweep_lines[current.row][hllabel_left];
					if(!prev_hline.inte[0]) {
						prev_hline.inte[0] = true;
						prev_hline.inte[1] = lineFallenOnPlane(prev_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel_left));							
						if(prev_hline.inte[1]) {
							updatePlaneMidVariables(prev_hline, plane_param, false);						
							labeling_queue.push(PixelCoord(current.row, prev_hline.right));
							plane_param.m_cross_hbound = true;
						}
					}						
				}
			} 
			else if( current.col == hline_current.right) {
				if(hllabel < m_hsweep_lines[current.row].size()-1)
				{
					auto& next_hline = m_hsweep_lines[current.row][hllabel+1];
					if(!next_hline.inte[0]) {
						next_hline.inte[0] = true;
						next_hline.inte[1] = lineFallenOnPlane(next_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel+1));
						if(next_hline.inte[1]) {
							updatePlaneMidVariables(next_hline, plane_param, false);								
							labeling_queue.push(PixelCoord(current.row, next_hline.left));
						}
					} 
				} else if(hllabel == m_hsweep_lines[current.row].size()-1) {
					uint16_t hllabel_right = 0;
					auto& next_hline = m_hsweep_lines[current.row][hllabel_right];
					if(!next_hline.inte[0]) {
						next_hline.inte[0] = true;
						next_hline.inte[1] = lineFallenOnPlane(next_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel_right));
						if(next_hline.inte[1]) {
							updatePlaneMidVariables(next_hline, plane_param, false);								
							labeling_queue.push(PixelCoord(current.row, next_hline.left));
							plane_param.m_cross_hbound = true;								
						}
					}						
				}
			}

			// find vertical neighbors of horizontal lines 
			HNeighborhood.clear();
			std::vector<line> hl_vneighbors;
			// left, right neighbors.
			findVNeighborsOfHLine(current.row-1, hline_current, hl_vneighbors, HNeighborhood);
			findVNeighborsOfHLine(current.row+1, hline_current, hl_vneighbors, HNeighborhood);
			for(const auto& hneighbor: HNeighborhood) {
				auto& hl_vn = m_hsweep_lines[hneighbor.row][hneighbor.col];
					if(!hl_vn.inte[0]) {
						hl_vn.inte[0] = true;
						hl_vn.inte[1] = lineFallenOnPlane(hl_vn, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(hneighbor);
						if(hl_vn.inte[1]) {
							updatePlaneMidVariables(hl_vn, plane_param, false);
							if(PlaneLabelAt(hl_vn.laserIdx, hl_vn.left)==0 ) {
								labeling_queue.push(PixelCoord(hl_vn.laserIdx, hl_vn.left));
							}
							else if(PlaneLabelAt(hl_vn.laserIdx, hl_vn.right)==0) {
								labeling_queue.push(PixelCoord(hl_vn.laserIdx, hl_vn.right));								
							}
						}
					}		
			}
		}			
		
		if(vllabel < 5000) {
			line vline_current;
            vline_current = m_vsweep_lines[current.col][vllabel];			
			

			
			// along vertical line.
			std::vector<PixelCoord> VNeighborhood;
			PixelCoord u_neighbor = current + PixelCoord(1, 0);
			if(u_neighbor.row >=vline_current.left && 
				u_neighbor.row <= vline_current.right) {
				VNeighborhood.push_back(u_neighbor);
			}
			PixelCoord d_neighbor = current + PixelCoord(-1, 0);
			if(d_neighbor.row >=vline_current.left && 
				d_neighbor.row <= vline_current.left) {
				VNeighborhood.push_back(d_neighbor);
			}

			// vertical line up&down neighbors.
			findNeighorsOnPlane(VNeighborhood, 
								labeling_queue, 
								vlinesegs_checked,
								plane_param, 
								vllabel, 
								true);		

			// jump over heighbor gaps.
			if(current.row == vline_current.left) {
				if(vllabel > 0) {
					auto& prev_vline = m_vsweep_lines[current.col][vllabel-1];
					if(!prev_vline.inte[0]) {
						prev_vline.inte[0] = true;
						prev_vline.inte[1] = lineFallenOnPlane(prev_vline, plane_param, m_lp_dist_threshold);
						vlinesegs_checked.push_back(PixelCoord(current.col, vllabel-1));
						if(prev_vline.inte[1]) {
							updatePlaneMidVariables(prev_vline, plane_param, true);
							labeling_queue.push(PixelCoord(prev_vline.right, current.col));								
						}
					} 
				}
			} 
			else if(current.row == vline_current.right) {
				if(vllabel < m_vsweep_lines[current.col].size()-1) {
					auto& next_vline = m_vsweep_lines[current.col][vllabel+1];
					if(!next_vline.inte[0]) {
						next_vline.inte[0] = true;
						next_vline.inte[1] = lineFallenOnPlane(next_vline, plane_param, m_lp_dist_threshold);
						vlinesegs_checked.push_back(PixelCoord(current.col, vllabel+1));
						if(next_vline.inte[1]) {
							updatePlaneMidVariables(next_vline, plane_param, true);
							labeling_queue.push(PixelCoord(next_vline.left, current.col));
						}
					}
				}
			}

			// find horizontal neighbors of vertical lines 
			VNeighborhood.clear();
			std::vector<line> vl_hneighbors;
			// left, right neighbors.
			int lcol = current.col-1;
			if(lcol < 0)
				lcol += m_proj_params.cols();
			int rcol = current.col+1;
			if(rcol>= m_proj_params.cols())
				rcol -= m_proj_params.cols();
			findHNeighborsOfVLine(lcol, vline_current, vl_hneighbors, VNeighborhood);
			findHNeighborsOfVLine(rcol, vline_current, vl_hneighbors, VNeighborhood);
			for(const auto& vneighbor: VNeighborhood) {
				auto& vl_hn = m_vsweep_lines[vneighbor.row][vneighbor.col];
				if(!vl_hn.inte[0]) {
					vl_hn.inte[0] = true;
					vl_hn.inte[1] = lineFallenOnPlane(vl_hn, plane_param, m_lp_dist_threshold);
					vlinesegs_checked.push_back(vneighbor);
					if(vl_hn.inte[1]) {
						updatePlaneMidVariables(vl_hn, plane_param, true);
						if(PlaneLabelAt(vl_hn.left, vneighbor.row)==0)
							labeling_queue.push(PixelCoord(vl_hn.left, vneighbor.row));
						else if(PlaneLabelAt(vl_hn.right, vneighbor.row)==0)
							labeling_queue.push(PixelCoord(vl_hn.right, vneighbor.row));
						if(current.col==0 || current.col==m_proj_params.cols()-1) {
							// fprintf(stderr, "vertical line segments plane cross cols bound \n");
							plane_param.m_cross_hbound = true;
						}						
					}
				}	
			}
		}								
	}

	// clear checked lines.
	clearCheckedSweepLines(m_vsweep_lines, vlinesegs_checked);
	clearCheckedSweepLines(m_hsweep_lines, hlinesegs_checked);

		
}

int LineClustering::detectPlaneSeedTogetherAt(int row, 
								   int col,
								   PlaneParams& plane_param)
{
	bool succeed_flag = detectPlaneSeedAt(row, col, plane_param);
	if(succeed_flag) 
		return 1;
	
	succeed_flag = detectSDPlaneSeedAt(row, col, plane_param);

	if(succeed_flag) {
		return 1;
		// int vllabel = VLineLabelAt(row, col);
		// // int hllabel = HLineLabelAt(row, col);		
		// if(vllabel != 5000) 
		// 	return 2;
		// else 
		// 	return 3;
	}
	else 
		return 0; 
}

void LineClustering::labelOnePlaneTogetherBFS(int row, 
											int col, 
											PlaneParams& plane_param, 
											uint16_t label) {

	std::queue<PixelCoord> labeling_queue;
	labeling_queue.push(PixelCoord(row, col));

	// size_t max_queue_size = 0;

	// reset parameters.
	m_pts_cov33.setZero();
	m_pts_sum31.setZero();
	m_lineOnPlane_cnt = 0;

	std::vector<PixelCoord> vlinesegs_checked;
	std::vector<PixelCoord> hlinesegs_checked;

	while(!labeling_queue.empty()) {
      	// max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      	// fprintf(stderr, " ------ max queue size: %d \n",
      	//  max_queue_size);

		const PixelCoord current = labeling_queue.front();
		labeling_queue.pop();
		// labeling_queue.pop_front();
		if(PlaneLabelAt(current) > 0)
		{	// we have already labeled this point. No need to add it.
			continue;
		}

		uint16_t vllabel = VLineLabelAt(current);
		uint16_t hllabel = HLineLabelAt(current);

		if(vllabel == 5000 && hllabel == 5000)
			continue;

		SetPlaneLabel(current, label);

		// horizontal line up&down neighbors.
		if(hllabel < 5000) {
			line hline_current;
            hline_current = m_hsweep_lines[current.row][hllabel];


			std::vector<PixelCoord> HNeighborhood;
			// find Neighborhood.
			PixelCoord l_neighbor = current + PixelCoord(0, -1);
			if(l_neighbor.col < 0) 
				l_neighbor.col += m_proj_params.cols();
			if(l_neighbor.col >= hline_current.left &&
				l_neighbor.col <= hline_current.right)
				HNeighborhood.push_back(l_neighbor);

			PixelCoord r_neighbor = current + PixelCoord(0, 1);
			if(r_neighbor.col >= m_proj_params.cols())
				r_neighbor.col -= m_proj_params.cols();
			if(r_neighbor.col <= hline_current.right &&
				r_neighbor.col >= hline_current.left)
				HNeighborhood.push_back(r_neighbor);

			findNeighorsOnPlane(HNeighborhood, 
								labeling_queue, 
								hlinesegs_checked,
								plane_param, 
								hllabel, 
								false);		


			// jump from neighbor gaps.

			if(current.col == hline_current.left) {
				if(hllabel > 0) { // this is not the first line segment.
					auto& prev_hline = m_hsweep_lines[current.row][hllabel-1];
					if(!prev_hline.inte[0]) {
						prev_hline.inte[0] = true;
						prev_hline.inte[1] = lineFallenOnPlane(prev_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel-1));							
						if(prev_hline.inte[1]) {
							updatePlaneMidVariables(prev_hline, plane_param, false);
							labeling_queue.push(PixelCoord(current.row, prev_hline.right));
						}
					}  
				} else if(hllabel == 0 ) {
					uint16_t hllabel_left = m_hsweep_lines[current.row].size()-1;
					auto& prev_hline = m_hsweep_lines[current.row][hllabel_left];
					if(!prev_hline.inte[0]) {
						prev_hline.inte[0] = true;
						prev_hline.inte[1] = lineFallenOnPlane(prev_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel_left));							
						if(prev_hline.inte[1]) {
							updatePlaneMidVariables(prev_hline, plane_param, false);						
							labeling_queue.push(PixelCoord(current.row, prev_hline.right));
							plane_param.m_cross_hbound = true;
						}
					}						
				}
			} 
			else if( current.col == hline_current.right) {
				if(hllabel < m_hsweep_lines[current.row].size()-1)
				{
					auto& next_hline = m_hsweep_lines[current.row][hllabel+1];
					if(!next_hline.inte[0]) {
						next_hline.inte[0] = true;
						next_hline.inte[1] = lineFallenOnPlane(next_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel+1));
						if(next_hline.inte[1]) {
							updatePlaneMidVariables(next_hline, plane_param, false);								
							labeling_queue.push(PixelCoord(current.row, next_hline.left));
						}
					} 
				} else if(hllabel == m_hsweep_lines[current.row].size()-1) {
					uint16_t hllabel_right = 0;
					auto& next_hline = m_hsweep_lines[current.row][hllabel_right];
					if(!next_hline.inte[0]) {
						next_hline.inte[0] = true;
						next_hline.inte[1] = lineFallenOnPlane(next_hline, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(PixelCoord(current.row, hllabel_right));
						if(next_hline.inte[1]) {
							updatePlaneMidVariables(next_hline, plane_param, false);								
							labeling_queue.push(PixelCoord(current.row, next_hline.left));
							plane_param.m_cross_hbound = true;								
						}
					}						
				}
			}

			// find vertical neighbors of horizontal lines 
			HNeighborhood.clear();
			std::vector<line> hl_vneighbors;
			// left, right neighbors.
			findVNeighborsOfHLine(current.row-1, hline_current, hl_vneighbors, HNeighborhood);
			findVNeighborsOfHLine(current.row+1, hline_current, hl_vneighbors, HNeighborhood);
			for(const auto& hneighbor: HNeighborhood) {
				auto& hl_vn = m_hsweep_lines[hneighbor.row][hneighbor.col];
					if(!hl_vn.inte[0]) {
						hl_vn.inte[0] = true;
						hl_vn.inte[1] = lineFallenOnPlane(hl_vn, plane_param, m_lp_dist_threshold);
						hlinesegs_checked.push_back(hneighbor);
						if(hl_vn.inte[1]) {
							updatePlaneMidVariables(hl_vn, plane_param, false);
							if(PlaneLabelAt(hl_vn.laserIdx, hl_vn.left)==0 ) {
								labeling_queue.push(PixelCoord(hl_vn.laserIdx, hl_vn.left));
							}
							else if(PlaneLabelAt(hl_vn.laserIdx, hl_vn.right)==0) {
								labeling_queue.push(PixelCoord(hl_vn.laserIdx, hl_vn.right));								
							}
						}
					}		
			}

		}			
		
		// vertical line left&right neighbors.
		if(vllabel < 5000) {
			line vline_current;
            vline_current = m_vsweep_lines[current.col][vllabel];			
	

			std::vector<PixelCoord> VNeighborhood;
			// find Neighborhood.
			PixelCoord u_neighbor = current + PixelCoord(1, 0);
			if(u_neighbor.row <= vline_current.right) {
				VNeighborhood.push_back(u_neighbor);
			}
			PixelCoord d_neighbor = current + PixelCoord(-1, 0);
			if(d_neighbor.row >= vline_current.left) {
				VNeighborhood.push_back(d_neighbor);
			}

			findNeighorsOnPlane(VNeighborhood, 
								labeling_queue, 
								vlinesegs_checked,
								plane_param, 
								vllabel, 
								true);	

			// jump over heighbor gaps.

			if(current.row == vline_current.left) {
				if(vllabel > 0) {
					auto& prev_vline = m_vsweep_lines[current.col][vllabel-1];
					if(!prev_vline.inte[0]) {
						prev_vline.inte[0] = true;
						prev_vline.inte[1] = lineFallenOnPlane(prev_vline, plane_param, m_lp_dist_threshold);
						vlinesegs_checked.push_back(PixelCoord(current.col, vllabel-1));
						if(prev_vline.inte[1]) {
							updatePlaneMidVariables(prev_vline, plane_param, true);
							labeling_queue.push(PixelCoord(prev_vline.right, current.col));								
						}
					} 
				}
			} 
			else if(current.row == vline_current.right) {
				if(vllabel < m_vsweep_lines[current.col].size()-1) {
					auto& next_vline = m_vsweep_lines[current.col][vllabel+1];
					if(!next_vline.inte[0]) {
						next_vline.inte[0] = true;
						next_vline.inte[1] = lineFallenOnPlane(next_vline, plane_param, m_lp_dist_threshold);
						vlinesegs_checked.push_back(PixelCoord(current.col, vllabel+1));
						if(next_vline.inte[1]) {
							updatePlaneMidVariables(next_vline, plane_param, true);
							labeling_queue.push(PixelCoord(next_vline.left, current.col));
						}
					}
				}
			}

			// find horizontal neighbors of vertical lines 
			VNeighborhood.clear();
			std::vector<line> vl_hneighbors;
			// left, right neighbors.
			int lcol = current.col-1;
			if(lcol < 0)
				lcol += m_proj_params.cols();
			int rcol = current.col+1;
			if(rcol>= m_proj_params.cols())
				rcol -= m_proj_params.cols();
			findHNeighborsOfVLine(lcol, vline_current, vl_hneighbors, VNeighborhood);
			findHNeighborsOfVLine(rcol, vline_current, vl_hneighbors, VNeighborhood);
			for(const auto& vneighbor: VNeighborhood) {
				auto& vl_hn = m_vsweep_lines[vneighbor.row][vneighbor.col];
				if(!vl_hn.inte[0]) {
					vl_hn.inte[0] = true;
					vl_hn.inte[1] = lineFallenOnPlane(vl_hn, plane_param, m_lp_dist_threshold);
					vlinesegs_checked.push_back(vneighbor);
					if(vl_hn.inte[1]) {
						updatePlaneMidVariables(vl_hn, plane_param, true);
						if(PlaneLabelAt(vl_hn.left, vneighbor.row)==0)
							labeling_queue.push(PixelCoord(vl_hn.left, vneighbor.row));
						else if(PlaneLabelAt(vl_hn.right, vneighbor.row)==0)
							labeling_queue.push(PixelCoord(vl_hn.right, vneighbor.row));
						if(current.col==0 || current.col==m_proj_params.cols()-1) {
							// fprintf(stderr, "vertical line segments plane cross cols bound \n");
							plane_param.m_cross_hbound = true;
						}						
					}
				}	
			}

		}
	}

	// clear checked lines.
    clearCheckedSweepLines(m_vsweep_lines, vlinesegs_checked);
    clearCheckedSweepLines(m_hsweep_lines, hlinesegs_checked);

}


void LineClustering::findNeighorsOnPlane(
						const std::vector<PixelCoord>& Neighborhood, 
						std::queue<PixelCoord>& label_queue,
						std::vector<PixelCoord>& checked_lineseg_idxs,
						PlaneParams& plane_param,
						uint16_t l_label,
						bool v_flag)
{
	for(int i=0; i < Neighborhood.size(); i++) {
		PixelCoord neighbor = Neighborhood[i];

		if(PlaneLabelAt(neighbor))
			continue;

		if(v_flag) {
			uint16_t vllabel = VLineLabelAt(neighbor);
			if(vllabel<5000) {
				auto& lv_n = m_vsweep_lines[neighbor.col][vllabel];
				if(!lv_n.inte[0]) { // not checked.	
					lv_n.inte[0] = true;
					lv_n.inte[1] = lineFallenOnPlane(lv_n, plane_param, m_lp_dist_threshold);
					checked_lineseg_idxs.push_back(PixelCoord(neighbor.col, vllabel));

					if(lv_n.inte[1]) {
						updatePlaneMidVariables(lv_n, plane_param, true);
						// m_lineOnPlane_cnt ++;
					}

					// for debug.
					// fprintf(stderr, "v lineseg fallen on plane \n");
				}
				if(vllabel == l_label ) {
					if(lv_n.inte[1])
						// label_queue.push_back(neighbor);					
						label_queue.push(neighbor);					
				} 
			}
		} else {
			uint16_t hllabel = HLineLabelAt(neighbor);
			if(hllabel < 5000) {
				auto& lh_n = m_hsweep_lines[neighbor.row][hllabel];
				if(!lh_n.inte[0]) { // not checked.	
					lh_n.inte[0] = true;
					lh_n.inte[1] = lineFallenOnPlane(lh_n, plane_param, m_lp_dist_threshold);
					checked_lineseg_idxs.push_back(PixelCoord(neighbor.row, hllabel));					

					if(lh_n.inte[1]) {
						updatePlaneMidVariables(lh_n, plane_param, false);					
						// m_lineOnPlane_cnt ++;
					}
					// for debug.
					// fprintf(stderr, "h lineseg fallen on plane \n");
				}						
				if(hllabel == l_label) {
					if(lh_n.inte[1])
						// label_queue.push_back(neighbor);
						label_queue.push(neighbor);
				}					
			}


		}		
	}
}

bool LineClustering::findVNeighborsOfHLine(int nrow, 
						   line line_current, 
						   std::vector<line>& line_neighbors,
						   std::vector<PixelCoord>& ln_indexes) {
	
	if(nrow < m_proj_params.rows() && nrow >= 0) {
		uint16_t n_lineseg_start_idx = 5000;
		int ncol_start = line_current.left;
		while(n_lineseg_start_idx==5000 && ncol_start <= line_current.right) {
			n_lineseg_start_idx = m_hl_image.at<uint16_t>(nrow, ncol_start);
			ncol_start ++;
		}

		// for debug.
		// fprintf(stderr, "n_lineseg_start_idx: %d \n", n_lineseg_start_idx);

		if(n_lineseg_start_idx == 5000) {
			return false;
		}

		uint16_t n_lineseg_stop_idx = 5000;
		int ncol_stop = line_current.right;
		while(n_lineseg_stop_idx == 5000 && ncol_stop >= line_current.left) {
			n_lineseg_stop_idx = m_hl_image.at<uint16_t>(nrow, ncol_stop);
			ncol_stop --;
		}

		// for debug.
		// fprintf(stderr, "n_lineseg_stop_idx: %d \n", n_lineseg_stop_idx);

		if(n_lineseg_stop_idx == 5000) {
			return false;
		}

		int ln_cnt = 0;
		for(uint16_t i=n_lineseg_start_idx; i<=n_lineseg_stop_idx; i++) {
			if(m_hsweep_lines[nrow][i].inte[0])
				continue;

			line_neighbors.push_back(m_hsweep_lines[nrow][i]);
			ln_indexes.push_back(PixelCoord(nrow, i));
			ln_cnt ++;
		}
		if(ln_cnt > 0)
			return true;
		else 
			return false;
	}
	return false;
}

bool LineClustering::findHNeighborsOfVLine(int ncol, 
						   line line_current, 
						   std::vector<line>& line_neighbors,
						   std::vector<PixelCoord>& ln_indexes) {
	
	if(ncol < m_proj_params.cols() && ncol >= 0) {
		uint16_t n_lineseg_start_idx = 5000;
		int nrow_start = line_current.left;
		while(n_lineseg_start_idx==5000 && nrow_start <= line_current.right) {
			n_lineseg_start_idx = m_vl_image.at<uint16_t>(nrow_start, ncol);
			nrow_start ++;
		}

		// for debug.
		// fprintf(stderr, "n_lineseg_start_idx: %d \n", n_lineseg_start_idx);

		if(n_lineseg_start_idx == 5000) {
			return false;
		}

		uint16_t n_lineseg_stop_idx = 5000;
		int nrow_stop = line_current.right;
		while(n_lineseg_stop_idx == 5000 && nrow_stop >= line_current.left) {
			n_lineseg_stop_idx = m_vl_image.at<uint16_t>(nrow_stop, ncol);
			nrow_stop --;
		}

		// for debug.
		// fprintf(stderr, "n_lineseg_stop_idx: %d \n", n_lineseg_stop_idx);

		if(n_lineseg_stop_idx == 5000) {
			return false;
		}

		int ln_cnt = 0;
		for(uint16_t i=n_lineseg_start_idx; i<=n_lineseg_stop_idx; i++) {
			if(m_vsweep_lines[ncol][i].inte[0])
				continue;

			line_neighbors.push_back(m_vsweep_lines[ncol][i]);
			ln_indexes.push_back(PixelCoord(ncol, i));
			ln_cnt ++;
		}
		if(ln_cnt > 0)
			return true;
		else 
			return false;
	}
	return false;
}


// line check function.
bool LineClustering::checkTwoLinesCoPlane(line la, line lb) {
	pcl::PointCloud<pcl::PointXYZ> cloud_fit;
	pcl::PointXYZ pt;

	// x-------------x----------------*
	//    x-------------*-----------*
	pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
	cloud_fit.push_back(pt);
	pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
	cloud_fit.push_back(pt);
	pt.x = lb.p1.x; pt.y = lb.p1.y; pt.z = lb.z1;
	cloud_fit.push_back(pt);
	PlaneParams plane = fitPlanePCA(cloud_fit.makeShared());

	bool la_on_plane = lineFallenOnPlane(la, plane, m_lp_dist_threshold);
	bool lb_on_plane = lineFallenOnPlane(lb, plane, m_lp_dist_threshold);

	if(la_on_plane && lb_on_plane) { 
		return true;
	}		

	return false;
}

bool LineClustering::checkThreeLinesCoPlane(line la, 
												 line lb, 
												 line lc, 
												 PlaneParams& plane) {
	pcl::PointCloud<pcl::PointXYZ> cloud_fit;
	pcl::PointXYZ pt;

	// x-------------x----------------*
	//       *--------------*---------------*
	//    x-------------*-----------*
	pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
	cloud_fit.push_back(pt);
	pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
	cloud_fit.push_back(pt);
	pt.x = lc.p1.x; pt.y = lc.p1.y; pt.z = lc.z1;
	cloud_fit.push_back(pt);
	PlaneParams plane1 = fitPlanePCA(cloud_fit.makeShared());

	pt.x = la.p2.x; pt.y = la.p2.y; pt.z = la.z2;
	bool la_p2_on_plane1 = pointFallOnPlane(pt, plane1);
	if(!la_p2_on_plane1)
		return false;

	pt.x = lc.p2.x; pt.y = lc.p2.y; pt.z = lc.z2;
	bool lc_p2_on_plane1 = pointFallOnPlane(pt, plane1);
	if(!lc_p2_on_plane1)
		return false;

	pt.x = lc.pm.x; pt.y = lc.pm.y; pt.z = lc.zm;
	bool lb_pm_on_plane1 = pointFallOnPlane(pt, plane1);		
	if(!lb_pm_on_plane1)
		return false;

	bool lb_on_plane1 = lineFallenOnPlane(lb, plane1, m_lp_dist_threshold);
	if(!lb_on_plane1)
		return false;		

	// x-------------x----------------*
	//       x--------------*---------------*
	//    *-------------*-----------*
	cloud_fit.clear();
	pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
	cloud_fit.push_back(pt);
	pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
	cloud_fit.push_back(pt);
	pt.x = lb.p1.x; pt.y = lb.p1.y; pt.z = lb.z1;
	cloud_fit.push_back(pt);
	PlaneParams plane2 = fitPlanePCA(cloud_fit.makeShared());

	pt.x = la.p2.x; pt.y = la.p2.y; pt.z = la.z2;
	bool la_p2_on_plane2 = pointFallOnPlane(pt, plane2);
	if(!la_p2_on_plane2)
		return false;

	pt.x = lb.p2.x; pt.y = lb.p2.y; pt.z = lb.z2;
	bool lb_p2_on_plane2 = pointFallOnPlane(pt, plane2);
	if(!lb_p2_on_plane2)
		return false;

	pt.x = lb.pm.x; pt.y = lb.pm.y; pt.z = lb.zm;
	bool lb_pm_on_plane2 = pointFallOnPlane(pt, plane2);		
	if(!lb_pm_on_plane2)
		return false;

	bool lc_on_plane2 = lineFallenOnPlane(lc, plane2, m_lp_dist_threshold);
	if(!lc_on_plane2)
		return false;	


	// // vertical distance difference of la, lb.
	float lab_dist = fabs(la.vdist - lb.vdist);
	// vertical distance difference of lb, lc.
	float lbc_dist = fabs(lb.vdist - lc.vdist);
	//  vertical distance difference of la, lc.
	float lac_dist = fabs(la.vdist - lc.vdist);

	float max_difference = std::max(lab_dist, lbc_dist);
	max_difference = std::max(max_difference, lac_dist);

	if(max_difference > 1.0) {
		if(lac_dist < lab_dist && lac_dist < lbc_dist) {
			return false;
		}

		if(lab_dist > 2.5*lbc_dist || lab_dist < 0.4*lbc_dist)
			return false;			
	}

	std::vector<line> lines_abc;
	lines_abc.push_back(la);
	lines_abc.push_back(lb);
	lines_abc.push_back(lc);
	plane = fitPlaneFromLines(lines_abc);
	return true;
}

std::vector<int> LineClustering::checkMaskedEndPoints(
							line la, 
							line lb, 
							line lc, 
							bool vertical_flag) {
	std::vector<int> plabeled_indices;
	std::vector<PixelCoord> pix_coords;
	auto la_endpts_pixs = castEndPointsPixCoords(la, vertical_flag);
	auto lb_endpts_pixs = castEndPointsPixCoords(lb, vertical_flag);
	auto lc_endpts_pixs = castEndPointsPixCoords(lc, vertical_flag);

	// insert three points.
	pix_coords.insert(pix_coords.end(), 
									  la_endpts_pixs.begin(), 
									  la_endpts_pixs.end());
	pix_coords.insert(pix_coords.end(), 
									  lb_endpts_pixs.begin(), 
									  lb_endpts_pixs.end());
	pix_coords.insert(pix_coords.end(), 
									  lc_endpts_pixs.begin(), 
									  lc_endpts_pixs.end());

	for(int i=0; i < pix_coords.size(); i++) {
		if(PlaneLabelAt(pix_coords[i]) > 0) {
			plabeled_indices.push_back(i);
		}
	}

	return plabeled_indices;
}

bool LineClustering::lineFallenOnPlane(line la, 
											PlaneParams plane_param,
											float dist_threshold) {
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointXYZ pt;
    // line a: two end points, one mid points.
    pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
    cloud_in.push_back(pt);
    pt.x = la.p2.x; pt.y = la.p2.y; pt.z = la.z2;
    cloud_in.push_back(pt);
    pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
    cloud_in.push_back(pt);

    bool planeValid = true;
    for (int j = 0; j < cloud_in.size(); j++)
    {
        // if OX * n > 0.2, then plane is not fit well
		if(!pointFallOnPlane(cloud_in[j], plane_param,dist_threshold))
        {
            planeValid = false;
            break;
        }
    }
    return planeValid; 		
}

bool LineClustering::pointFallOnPlane(pcl::PointXYZ point, 
										   PlaneParams plane,
										   float dist_threshold) {
	if (fabs(plane.m_a * point.x +
	         plane.m_b * point.y +
	         plane.m_c * point.z + plane.m_d) > dist_threshold)		{
		return false;
	} else {
		return true;
	}
}

PlaneParams LineClustering::fitPlaneFromTwoLines(line la, line lb) {
	std::vector<line> lines;
	lines.push_back(la);
	lines.push_back(lb);
	return fitPlaneFromLines(lines);
}

PlaneParams LineClustering::fitPlaneFromLines(std::vector<line> lines) {
    pcl::PointCloud<pcl::PointXYZ> cloud_fit;
    pcl::PointXYZ pt;
    for(int i=0; i < lines.size(); i++) {
		line la = lines[i];
	    // line a: two end points, one mid points.
	    pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
	    cloud_fit.push_back(pt);
	    pt.x = la.p2.x; pt.y = la.p2.y; pt.z = la.z2;
	    cloud_fit.push_back(pt);
	    pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
	    cloud_fit.push_back(pt);    	
    }		

    return fitPlanePCA(cloud_fit.makeShared());
}

PlaneParams LineClustering::fitPlanePCA(
					const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr) {
    int fit_num = cloud_ptr->size();
    Eigen::Matrix<float, Eigen::Dynamic, 3> matA0(fit_num, 3);
    Eigen::Matrix<float, Eigen::Dynamic, 1> matB0(fit_num, 1);
        
    for (int j = 0; j < cloud_ptr->size(); j++)
    {
        matA0(j, 0) = cloud_ptr->points[j].x;
        matA0(j, 1) = cloud_ptr->points[j].y;
        matA0(j, 2) = cloud_ptr->points[j].z;
        matB0(j, 0) = -1.0;
    }

    // find the norm of plane
    Eigen::Vector3f norm = matA0.colPivHouseholderQr().solve(matB0);
    float negative_OA_dot_norm = 1 / norm.norm();
    norm.normalize();

    PlaneParams plane;
    plane.m_a = norm(0);
    plane.m_b = norm(1);
    plane.m_c = norm(2);
    plane.m_d = negative_OA_dot_norm;  
    return plane;
}


void LineClustering::updatePlaneMidVariables(line la, 
								PlaneParams& plane_param, 
								bool vertical_flag) {
	pcl::PointCloud<pcl::PointXYZ> cloud_in;
	pcl::PointXYZ pt;
	// line a: two end points, one mid points.
	pt.x = la.p1.x; pt.y = la.p1.y; pt.z = la.z1;
	cloud_in.push_back(pt);
	pt.x = la.p2.x; pt.y = la.p2.y; pt.z = la.z2;
	cloud_in.push_back(pt);
	pt.x = la.pm.x; pt.y = la.pm.y; pt.z = la.zm;
	cloud_in.push_back(pt);

	for(int i=0; i < cloud_in.size(); i++) {
		Eigen::Vector3f pt_vec = cloud_in[i].getVector3fMap();
		m_pts_cov33 += pt_vec*pt_vec.transpose();
		m_pts_sum31 += pt_vec;
	}

	if(vertical_flag) {
		plane_param.m_line_seg_idxs.push_back(
			std::make_pair(PixelCoord(la.left, la.laserIdx),
										 PixelCoord(la.right, la.laserIdx))
			);
		// if(la.laserIdx==0 || la.laserIdx==m_proj_params.cols()-1) {
		// 	plane_param.m_cross_hbound = true;
		// }	
	} else {
		plane_param.m_line_seg_idxs.push_back(
			std::make_pair(PixelCoord(la.laserIdx, la.left),
										 PixelCoord(la.laserIdx, la.right))
			);
		uint16_t hllabel = HLineLabelAt(la.laserIdx, la.left);
		// if(hllabel==0 || hllabel == m_hsweep_lines[la.laserIdx].size()-1) {
		// 	if(hllabel ==0 && la.left < 20) {
		// 		plane_param.m_cross_hbound = true;				
		// 	} else if( hllabel == m_hsweep_lines[la.laserIdx].size()-1 &&
		// 		la.right >= m_proj_params.cols()-20) {
		// 		plane_param.m_cross_hbound = true;								
		// 	}
		// }	
	}

	m_lineOnPlane_cnt ++;
	if(m_lineOnPlane_cnt>=3) {
	    Eigen::Vector3f norm = m_pts_cov33.colPivHouseholderQr().solve(-m_pts_sum31);
	    float negative_OA_dot_norm = 1 / norm.norm();
	    norm.normalize();

	    plane_param.m_a = norm(0);
	    plane_param.m_b = norm(1);
	    plane_param.m_c = norm(2);
	    plane_param.m_d = negative_OA_dot_norm;  
	}
}

void LineClustering::updatePlaneMidVariables2(line la,
											  PlaneParams& plane_params) {
	// Eigen::Vector3d pt_s, pt_e, pt_m;
	// pt_s << la.p1.x, la.p1.y, la.z1;
	// pt_e << la.p2.x, la.p2.y, la.z2;
	// pt_m = (pt_s + pt_e)/2;
	// double line_len = (pt_s - pt_e).norm();

	// auto integral_T1=[](double phi_s, double phi_e, double psi_s, double psi_e) -> double {
	// 	return (phi_s*psi_s + phi_e*psi_e)/3 + (phi_s*psi_e + phi_e*psi_s)/6;
	// };

	// auto integral_T2=[](double phi_s, double phi_e) -> double {
	// 	return (phi_s + phi_e)/2;
	// };

	// double t1_xx = integral_T1(pt_s(0), pt_e(0), pt_s(0), pt_e(0));
	// double t1_xy = integral_T1(pt_s(0), pt_e(0), pt_s(1), pt_e(1));
	// double t1_xz = integral_T1(pt_s(0), pt_e(0), pt_s(2), pt_e(2));
	// double t1_yy = integral_T1(pt_s(1), pt_e(1), pt_s(1), pt_e(1));
	// double t1_yz = integral_T1(pt_s(1), pt_e(1), pt_s(2), pt_e(2));
	// double t1_zz = integral_T1(pt_s(2), pt_e(2), pt_s(2), pt_e(2));

	// double t2_x = integral_T2(pt_s(0), pt_e(0));
	// double t2_y = integral_T2(pt_s(1), pt_e(1));
	// double t2_z = integral_T2(pt_s(2), pt_e(2));

	// // update middle variables.
	// m_T5_x += line_len*pt_m(0);
	// m_T5_y += line_len*pt_m(1);
	// m_T5_z += line_len*pt_m(2);
	// m_T6 += line_len;	

	// // udpate parameters.
	// float mean_x = m_T5_x/m_T6;
	// float mean_y = m_T5_y/m_T6;
	// float mean_z = m_T5_z/m_T6;

	// m_pts_cov33(0,0) += (t1_xx - 2*mean_x*t2_x + mean_x*mean_x);
	// m_pts_cov33(1,1) += (t1_yy - 2*mean_y*t2_y + mean_y*mean_y);
	// m_pts_cov33(2,2) += (t1_zz - 2*mean_z*t2_z + mean_z*mean_z);
	// m_pts_cov33(0,1) += (t1_xy - mean_x*t2_y - mean_y*t2_x + mean_x*mean_y);
	// m_pts_cov33(0,2) += (t1_xz - mean_x*t2_z - mean_z*t2_x + mean_x*mean_z);
	// m_pts_cov33(1,2) += (t1_yz - mean_y*t2_z - mean_z*t2_y + mean_y*mean_z);
	// m_pts_cov33(1,0) = m_pts_cov33(0,1);
	// m_pts_cov33(2,0) = m_pts_cov33(0,2);
	// m_pts_cov33(2,1) = m_pts_cov33(1,2);

	// compute plane parameters.
	// normal vector is smallest eigen vector of covariance matrix.

}


void LineClustering::findPlaneContourAndConvexHull(
							PlaneParams& plane_param) {

	// new implementation: find convex hull directly.
	// 1. classify into two classes, 
	//		one cross the boundary, the other not on the boundary.
	// 2. for the former one, shift to the center of image, then shift back.
	//		for the later one, process as a single convex hull.

	if(!plane_param.m_cross_hbound) {
		plane_param.m_convexhulls = calculateConvexHull(plane_param.m_line_seg_idxs);

		// compute average pix.			
		PixelCoord average_pix(0,0);
		int min_hull_col = m_proj_params.cols();
		int max_hull_col = 0;
		for(int i=0; i < plane_param.m_convexhulls.size(); i++) {
			average_pix = average_pix + plane_param.m_convexhulls[i];
			if(plane_param.m_convexhulls[i].col < min_hull_col) {
				min_hull_col = plane_param.m_convexhulls[i].col;
			}
			if(plane_param.m_convexhulls[i].col > max_hull_col) {
				max_hull_col = plane_param.m_convexhulls[i].col;
			}
		}

		// if(min_hull_col < 20 || max_hull_col > m_proj_params.cols()-20) {
		// 	plane_param.m_cross_hbound = true;
		// }

		average_pix.row = std::floor(1.0*average_pix.row/plane_param.m_convexhulls.size());
		average_pix.col = std::floor(1.0*average_pix.col/plane_param.m_convexhulls.size());
		plane_param.m_center_pix = average_pix;

		//compute center point.
		plane_param.m_center_pt = computeCenterPoint(plane_param, average_pix);
	}
	else {
		// displace to the center of image.
		int col_shift = m_proj_params.cols()/2;
		std::vector<std::pair<PixelCoord, PixelCoord>> shifted_line_seg_idxs; 
		for(int i=0; i < plane_param.m_line_seg_idxs.size(); i++) {
			PixelCoord left_coord = plane_param.m_line_seg_idxs[i].first;
			PixelCoord right_coord = plane_param.m_line_seg_idxs[i].second;
			left_coord.col += col_shift;
			if(left_coord.col >= m_proj_params.cols()) {
				left_coord.col -= m_proj_params.cols();
			}
			right_coord.col += col_shift;
			if(right_coord.col >= m_proj_params.cols()) {
				right_coord.col -= m_proj_params.cols();
			}
			shifted_line_seg_idxs.push_back( {left_coord, right_coord} );
		}

		std::vector<PixelCoord> new_convex_hull = 
						calculateConvexHull(shifted_line_seg_idxs);

		// compute average pix.
		PixelCoord average_pix(0,0);
		for(int i=0; i < new_convex_hull.size(); i++) {
			average_pix = average_pix + new_convex_hull[i];
		}
		average_pix.row = std::floor(1.0*average_pix.row/new_convex_hull.size());
		average_pix.col = std::floor(1.0*average_pix.col/new_convex_hull.size());
		average_pix.col -= col_shift;
		if(average_pix.col < 0)
			average_pix.col += m_proj_params.cols();
		plane_param.m_center_pix = average_pix;

		//compute center point.
		plane_param.m_center_pt = computeCenterPoint(plane_param, average_pix);

		for(int i=0; i < new_convex_hull.size(); i++) {
			new_convex_hull[i].col -= col_shift;
			if(new_convex_hull[i].col < 0) {
				new_convex_hull[i].col += m_proj_params.cols();
			}
		}

		plane_param.m_convexhulls = new_convex_hull;
	}
}


pcl::PointXYZ LineClustering::computeCenterPoint(PlaneParams plane, 
													  PixelCoord coord) {
	float cos_theta = m_proj_params.AngleCosineFromRow(coord.row);
	float sin_theta = m_proj_params.AngleSineFromRow(coord.row);
	float cos_omega = m_proj_params.AngleCosineFromCol(coord.col);
	float sin_omega = m_proj_params.AngleSineFromCol(coord.col);

	float l_da, l_db, l_dc, l_r;
	l_da = cos_theta*cos_omega;
	l_db = cos_theta*sin_omega;
	l_dc = sin_theta;

	l_r = fabs(plane.m_d/(plane.m_a*l_da + plane.m_b*l_db + plane.m_c*l_dc));

	pcl::PointXYZ res;
	res.x = l_r*l_da;
	res.y = l_r*l_db;
	res.z = l_r*l_dc;
	return res;
}

pcl::PointXYZ LineClustering::computeVerticalPoint(PlaneParams plane) {
	float pl_a, pl_b, pl_c, pl_d, amp;
	pl_a = plane.m_a;
	pl_b = plane.m_b;
	pl_c = plane.m_c;
	pl_d = plane.m_d;

	amp = -pl_d/(pl_a*pl_a + pl_b*pl_b + pl_c*pl_c);

	pcl::PointXYZ res;
	res.x = amp * pl_a;
	res.y = amp * pl_b;
	res.z = amp * pl_c;
	return res;
}

std::vector<PixelCoord> LineClustering::calculateConvexHull(
	std::vector<std::pair<PixelCoord, PixelCoord>> lineseg_idxs) {
	int hline_cnt = 0;
	int vline_cnt = 0;		
	std::map<int, int> lsmap_row_leftmin;
	std::map<int, int> lsmap_row_rightmax;
	std::map<int, int> lsmap_col_leftmin;
	std::map<int, int> lsmap_col_rightmax;
	std::pair<std::map<int,int>::iterator, bool> ret;

	for(int i=0; i < lineseg_idxs.size(); i++) {
		auto left_coord = lineseg_idxs[i].first;
		auto right_coord = lineseg_idxs[i].second;
		if(left_coord.row == right_coord.row) {
			hline_cnt ++;
			ret = lsmap_row_leftmin.insert( {left_coord.row, left_coord.col} );
			if(!ret.second) {
				if(left_coord.col < lsmap_row_leftmin[left_coord.row])
					lsmap_row_leftmin[left_coord.row] = left_coord.col;	
			}
			// if(lsmap_row_leftmin.find(left_coord.row) == lsmap_row_leftmin.end()) {
			// 	lsmap_row_leftmin.insert( {left_coord.row, left_coord.col} );
			// } 
			// else {
			// 	if(left_coord.col < lsmap_row_leftmin[left_coord.row])
			// 		lsmap_row_leftmin[left_coord.row] = left_coord.col;
			// }

			ret = lsmap_row_rightmax.insert( {right_coord.row, right_coord.col} );
			if(!ret.second) {
				if(right_coord.col > lsmap_row_rightmax[right_coord.row])
					lsmap_row_rightmax[right_coord.row] = right_coord.col;	
			}
			// if(lsmap_row_rightmax.find(right_coord.row) == lsmap_row_rightmax.end()) {
			// 	lsmap_row_rightmax.insert( {right_coord.row, right_coord.col});
			// }
			// else {
			// 	if(right_coord.col > lsmap_row_rightmax[right_coord.row]) {
			// 		lsmap_row_rightmax[right_coord.row] = right_coord.col;
			// 	}
			// }
		} 
		else {
			vline_cnt ++;
			ret = lsmap_col_leftmin.insert( {left_coord.col, left_coord.row} );
			if(!ret.second) {
				if(left_coord.row < lsmap_col_leftmin[left_coord.col])
					lsmap_col_leftmin[left_coord.col] = left_coord.row;				
			}
			// if(lsmap_col_leftmin.find(left_coord.col) == lsmap_col_leftmin.end()) {
			// 	lsmap_col_leftmin.insert( {left_coord.col, left_coord.row} );
			// }
			// else {
			// 	if(left_coord.row < lsmap_col_leftmin[left_coord.col])
			// 		lsmap_col_leftmin[left_coord.col] = left_coord.row;
			// }

			ret = lsmap_col_rightmax.insert( {right_coord.col, right_coord.row} );
			if(!ret.second) {
				if(right_coord.row > lsmap_col_rightmax[right_coord.col])
					lsmap_col_rightmax[right_coord.col] = right_coord.row;				
			}
			// if(lsmap_col_rightmax.find(right_coord.col) == lsmap_col_rightmax.end()) {
			// 	lsmap_col_rightmax.insert( {right_coord.col, right_coord.row} );
			// }
			// else {
			// 	if(right_coord.row >lsmap_col_rightmax[right_coord.col]) 
			// 		lsmap_col_rightmax[right_coord.col] = right_coord.row;
			// }
		}
	}

	std::vector<PixelCoord> convex_hull;
	if(hline_cnt < 3 && vline_cnt==0) {
		return convex_hull;
	}		

	std::vector<cv::Point> end_points;
	for(auto iter_row_left=lsmap_row_leftmin.begin(),
		end_row_left = lsmap_row_leftmin.end(),
		iter_row_right=lsmap_row_rightmax.begin(),
		end_row_right = lsmap_row_rightmax.end(); 
		iter_row_left != end_row_left && iter_row_right != end_row_right; 
		iter_row_left++, iter_row_right++) {

		end_points.push_back(cv::Point(iter_row_left->first, iter_row_left->second));
		end_points.push_back(cv::Point(iter_row_right->first, iter_row_right->second));
	}

	for(auto iter_col_left=lsmap_col_leftmin.begin(),
		end_col_left = lsmap_col_leftmin.end(),
		iter_col_right=lsmap_col_rightmax.begin(),
		end_col_right = lsmap_col_rightmax.end(); 
		iter_col_left != end_col_left && iter_col_right != end_col_right; 
		iter_col_left++, iter_col_right++) {

		end_points.push_back(cv::Point(iter_col_left->second, iter_col_left->first));
		end_points.push_back(cv::Point(iter_col_right->second, iter_col_right->first));
	}		


	std::vector<cv::Point> hull;
	cv::convexHull(cv::Mat(end_points), hull, false);
	convex_hull.resize(hull.size());
	for(int i=0; i < hull.size(); i++) {
		convex_hull[i] = PixelCoord(hull[i].x, hull[i].y);
	}

	return convex_hull;
}








