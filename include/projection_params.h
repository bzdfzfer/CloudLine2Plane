#ifndef CLOUDLINE2PLANE_PROJECTION_PARAMS_H
#define CLOUDLINE2PLANE_PROJECTION_PARAMS_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <stdio.h>

using std::vector;
using std::upper_bound;
using std::floor;
using std::fabs;

class ScanParams{
 public:
  /**
   * Enum for the direction of the span.
   */
  enum class Direction { HORIZONTAL, VERTICAL };


  ScanParams() {}
  ScanParams(const float& start_angle, const float& end_angle,
             const float& step) {
    m_start_angle = start_angle;
    m_end_angle = end_angle;
    m_step = step;
    m_num_beams = floor((m_end_angle - m_start_angle) / m_step + 0.5);
    m_fov = fabs(end_angle - start_angle);
  }

  ScanParams(const float& start_angle, const float& end_angle,
             int num_beams) {
    m_start_angle = start_angle;
    m_end_angle = end_angle;
    m_num_beams = num_beams;
    m_step = (m_end_angle - m_start_angle) / (m_num_beams-1);
    m_fov = fabs(end_angle - start_angle);
  }

  const float& start_angle() const { return m_start_angle; }
  const float& end_angle() const { return m_end_angle; }
  const float& step() const { return m_step; }
  const float& fov() const { return m_fov; }
  const int& num_beams() const { return m_num_beams; }

  float& start_angle()  { return m_start_angle; }
  float& end_angle()  { return m_end_angle; }
  float& step()  { return m_step; }
  float& fov()  { return m_fov; }
  int& num_beams()  { return m_num_beams; }

  bool valid() const { return m_num_beams > 0 && m_fov > 0; }

 private:
  float m_start_angle = 0;
  float m_end_angle = 0;
  float m_step = 0;
  float m_fov = 0;
  int m_num_beams = 0;
};


class ProjectionParams {
public:

  enum class LIDARType {eVLP_16, eHDL_32, eVLP_32C, eHDL_64, eSEGCOMPPERCEPTRON};

	ProjectionParams() {}

	void setScan(const ScanParams& scan_params, const ScanParams::Direction& direction) {
		switch(direction) {
			case ScanParams::Direction::HORIZONTAL:
				m_hscan_params = scan_params;
				m_col_angles = FillVector(scan_params);
				break;
			case ScanParams::Direction::VERTICAL:
				m_vscan_params = scan_params;
				m_row_angles = FillVector(scan_params);
				break;			
		}
	} 

	void setLidarType(const LIDARType& type) {
		m_lidar_type = type;
	}

	void printLidarType() {
		switch(m_lidar_type) {
			case LIDARType::eVLP_16:
				fprintf(stderr, "Lidar type: VLP_16 \n");
				break;
			case LIDARType::eHDL_32:
				fprintf(stderr, "Lidar type: HDL_32 \n");
				break;
			case LIDARType::eVLP_32C:
				fprintf(stderr, "Lidar type: VLP_32C \n");
				break;
			case LIDARType::eHDL_64:
				fprintf(stderr, "Lidar type: HDL_64 \n");
				break;
			case LIDARType::eSEGCOMPPERCEPTRON:
				fprintf(stderr, "Lidar type: SEGCOMPPERCEPTRON \n");
				break;
			default:
				fprintf(stderr, "No Lidar type set, something wrong.\n");
				break;
		}
	}

	LIDARType lidar_type() {
		return m_lidar_type;
	}

  inline const float& v_start_angle() const {
    return m_vscan_params.start_angle();
  }
  inline const float& v_end_angle() const {
    return m_vscan_params.end_angle();
  }
  inline const float& v_step() const {
  	return m_vscan_params.step();
  }
  inline const float& v_fov() const { return m_vscan_params.fov(); }

  inline const float& h_start_angle() const {
    return m_hscan_params.start_angle();
  }
  inline const float& h_end_angle() const {
    return m_hscan_params.end_angle();
  }
  inline const float& h_step() const {
  	return m_hscan_params.step();
  }
  inline const float& h_fov() const { return m_hscan_params.fov(); }

  inline size_t rows() const { return m_row_angles.size(); }
  inline size_t cols() const { return m_col_angles.size(); }
  inline size_t size() const { return rows() * cols(); }


  const float AngleFromRow(int row) const {
  	return m_row_angles[row];
  }


  const float AngleFromCol(int col) const {
  	return m_col_angles[col];
  }

  const float AngleCosineFromRow(int row) const {
  	return m_row_angles_cosines[row];
  }

  const float AngleCosineFromCol(int col) const {
  	return m_col_angles_cosines[col];
  }

  const float AngleSineFromRow(int row) const {
  	return m_row_angles_sines[row];
  } 

  const float AngleSineFromCol(int col) const {
  	return m_col_angles_sines[col];
  }

  const float RowCosineDotColCosine(int row, int col) const {
  	return m_row_angles_cosines[row]*m_col_angles_cosines[col];
  }

  const float RowSineDotColCosine(int row, int col) const {
  	return m_row_angles_sines[row]*m_col_angles_cosines[col];
  }

  const float RowCosineDotColSine(int row, int col) const {
  	return m_row_angles_cosines[row]*m_col_angles_sines[col];
  }

  const float RowSineDotColSine(int row, int col) const {
  	return m_row_angles_sines[row]*m_col_angles_sines[col];
  }    

  size_t RowFromAngle(const float& angle) const {
  	// return FindClosest(m_row_angles, angle + 0.001f*sgn(angle));
  	return FindClosest(m_row_angles, angle);
  }


  size_t ColFromAngle(const float& angle) const {
  	return FindClosest(m_col_angles, angle);
  }

	/** utility for sign**/
  static inline float sgn(float value) {return (0.0f < value) - (value < 0.0f); }

	const std::vector<float>& RowAngleCosines() const {
	  return m_row_angles_cosines;
	}
	const std::vector<float>& ColAngleCosines() const {
	  return m_col_angles_cosines;
	}
	const std::vector<float>& RowAngleSines() const {
	  return m_row_angles_sines;
	}
	const std::vector<float>& ColAngleSines() const {
	  return m_col_angles_sines;
	}

	const std::vector<float>& ColAngles() const {
		return m_col_angles;
	}

	const std::vector<float>& RowAngles() const {
		return m_row_angles;
	}

  /**
   * @brief      Default parameters for 16 beam Velodyne
   *
   * @return     A pointer to parameters
   */
  static inline ProjectionParams VLP_16() {
  	auto params = ProjectionParams();
  	float h_min_ang = -180;
  	float h_max_ang = 180;
  	int num_hbeam = 870;
  	float h_angle_gap = (h_max_ang - h_min_ang) / (num_hbeam -1);
  	params.setScan(ScanParams(h_min_ang+h_angle_gap/2, h_max_ang-h_angle_gap/2, num_hbeam),
  				   ScanParams::Direction::HORIZONTAL);

  	fprintf(stderr, "horizontal scan param set ...\n");
  	fprintf(stderr, "printing hscan_params: \n");
  	params.printScanParams(params.m_hscan_params);

  	// fprintf(stderr, "\nprinting hscan_angles in rad: \n");
  	// params.printScanAngles(params.m_col_angles);

  	float v_min_ang = -15;
  	float v_max_ang = 15;
  	int num_vbeam = 16;
  	params.setScan(ScanParams(v_min_ang, v_max_ang, num_vbeam),
  				   ScanParams::Direction::VERTICAL);
  	fprintf(stderr, "vertical scan param set ...\n");
  	fprintf(stderr, "printing vscan_params: \n");
  	params.printScanParams(params.m_vscan_params);

  	// fprintf(stderr, "\nprinting vscan_angles in rad: \n");
  	// params.printScanAngles(params.m_row_angles);

  	params.FillCosSin();
  	params.setLidarType(ProjectionParams::LIDARType::eVLP_16);
  	return params;
  }

  /**
   * @brief      Default parameters for 32 beam Velodyne
   *
   * @return     A pointer to parameters
   */
  static inline ProjectionParams HDL_32() {
  	auto params = ProjectionParams();
  	float h_min_ang = -180;
  	float h_max_ang = 180;
  	int num_hbeam = 870;
  	float h_angle_gap = (h_max_ang - h_min_ang) / (num_hbeam -1);
  	params.setScan(ScanParams(h_min_ang+h_angle_gap/2-h_angle_gap/2, h_max_ang, num_hbeam),
  				   ScanParams::Direction::HORIZONTAL);

  	float v_min_ang = -30.67;
  	float v_max_ang = 10.67;
  	int num_vbeam = 32;
  	params.setScan(ScanParams(v_min_ang, v_max_ang, num_vbeam),
  				   ScanParams::Direction::VERTICAL);

  	params.FillCosSin();
  	params.setLidarType(ProjectionParams::LIDARType::eHDL_32);

  	return params;  	
  }

  /**
   * @brief      Parameters for 32 beam velodyne vlp-32c
   *             the lasers.
   *
   * @return     A pointer to parameters
   */
  static inline ProjectionParams VLP_32C() {
  	auto params = ProjectionParams();
  	float h_min_ang = -180;
  	float h_max_ang = 180;
  	int num_hbeam = 870;
  	float h_angle_gap = (h_max_ang - h_min_ang) / (num_hbeam -1);
  	params.setScan(ScanParams(h_min_ang+h_angle_gap/2, h_max_ang-h_angle_gap/2, num_hbeam),
  				   ScanParams::Direction::HORIZONTAL);

  	int num_vbeam = 32;
  	std::vector<float> vangles = {-25.0,  -15.639, -11.31, -8.843, 
  								  -7.254, -6.148,  -5.333, -4.667,
  								  -4.0,   -3.667,  -3.333, -3,
  								  -2.667, -2.333,  -2,     -1.667,
  								  -1.333, -1,      -0.667, -0.333,
  								  0.0,    0.333,   0.667,  1.0, 
  								  1.333,  1.667,   2.333,  3.333,
  								  4.667,  7.0,     10.333, 15.0};

		for(int r = 0; r < num_vbeam; r++) {
			params.m_row_angles.push_back(vangles[r] * M_PI/180.0);
		}  	
		params.m_vscan_params = ScanParams(vangles.front(), vangles.back(), num_vbeam);

		params.FillCosSin();
  	params.setLidarType(ProjectionParams::LIDARType::eVLP_32C);

  	return params;

  }


  /**
   * @brief      Default parameters for 64 beam Velodyne
   *
   * @return     A pointer to parameters
   */
  static inline ProjectionParams HDL_64() {
  	auto params = ProjectionParams();
  	float h_min_ang = -180;
  	float h_max_ang = 180;
  	int num_hbeam = 870;
  	float h_angle_gap = (h_max_ang - h_min_ang) / (num_hbeam -1);
  	params.setScan(ScanParams(h_min_ang+h_angle_gap/2, h_max_ang-h_angle_gap/2, num_hbeam),
  				   ScanParams::Direction::HORIZONTAL);

  	int num_vbeam = 64;
  	// up to down order.
  	// std::vector<float> vangles = {
  	// 	  1.9367, 1.57397, 1.30476, 0.871566, 0.57881, 0.180617, -0.088762, -0.451829, 
  	// 		-0.80315, -1.20124, -1.49388, -1.83325, -2.20757, -2.54663, -2.87384, -3.23588, 
  	// 		-3.53933, -3.93585, -4.21552, -4.5881, -4.91379, -5.25078, -5.6106, -5.9584, 
  	// 		-6.32889, -6.67575, -6.99904, -7.28731, -7.67877, -8.05803, -8.31047, -8.71141, 
  	// 		-9.02602, -9.57351, -10.0625, -10.4707, -10.9569, -11.599, -12.115, -12.5621, 
  	// 		-13.041, -13.4848, -14.0483, -14.5981, -15.1887, -15.6567, -16.1766, -16.554, 
  	// 		-17.1868, -17.7304, -18.3234, -18.7971, -19.3202, -19.7364, -20.2226, -20.7877, 
  	// 		-21.3181, -21.9355, -22.4376, -22.8566, -23.3224, -23.971, -24.5066, -24.9992};
  	std::vector<float> vangles = {
					-24.999200, -24.506600, -23.971000, -23.322400, -22.856600, -22.437600, -21.935500, -21.318100, 
					-20.787700, -20.222600, -19.736400, -19.320200, -18.797100, -18.323400, -17.730400, -17.186800, 
					-16.554000, -16.176600, -15.656700, -15.188700, -14.598100, -14.048300, -13.484800, -13.041000, 
					-12.562100, -12.115000, -11.599000, -10.956900, -10.470700, -10.062500, -9.573510, -9.026020, 
					-8.711410, -8.310470, -8.058030, -7.678770, -7.287310, -6.999040, -6.675750, -6.328890, 
					-5.958400, -5.610600, -5.250780, -4.913790, -4.588100, -4.215520, -3.935850, -3.539330, 
					-3.235880, -2.873840, -2.546630, -2.207570, -1.833250, -1.493880, -1.201240, -0.803150, 
					-0.451829, -0.088762, 0.180617, 0.578810, 0.871566, 1.304760, 1.573970, 1.936700};


		for(int r = 0; r < num_vbeam; r++) {
			params.m_row_angles.push_back(vangles[r] * M_PI/180.0);
		}  	
		params.m_vscan_params = ScanParams(vangles.front(), vangles.back(), num_vbeam);

		params.FillCosSin();
  	params.setLidarType(ProjectionParams::LIDARType::eHDL_64);


  	return params;

  }

  static inline ProjectionParams SegCompPerceptron() {
  	auto params = ProjectionParams();
  	int num_hbeam = 512;
  	float h_angle_gap = 51.65/512.0;
		float h_min_ang = h_angle_gap*255.5;
  	float h_max_ang = -h_angle_gap*255.5;
  	params.setScan(ScanParams(h_min_ang, h_max_ang, num_hbeam),
  				   ScanParams::Direction::HORIZONTAL);  	

  	int num_vbeam = 512;
  	float v_angle_gap = 36.73/512.0;
  	float v_min_ang = v_angle_gap*255.5;
  	float v_max_ang = -v_angle_gap*255.5;
  	params.setScan(ScanParams(v_min_ang, v_max_ang, num_vbeam),
  					ScanParams::Direction::VERTICAL);

  	params.FillCosSin();
  	params.setLidarType(ProjectionParams::LIDARType::eSEGCOMPPERCEPTRON);

  	return params;
  }
  
	 void FillCosSin() {
		m_row_angles_sines.clear();
		m_row_angles_cosines.clear();
		for (const auto& angle : m_row_angles) {
			m_row_angles_sines.push_back(sin(angle));
			m_row_angles_cosines.push_back(cos(angle));
		}
		m_col_angles_sines.clear();
		m_col_angles_cosines.clear();
		for (const auto& angle : m_col_angles) {
			m_col_angles_sines.push_back(sin(angle));
			m_col_angles_cosines.push_back(cos(angle));
		}	 	
	 }

	void reserseVerticalParams() {
		ScanParams new_vscan_params;
		new_vscan_params.start_angle() =  m_vscan_params.end_angle();
		new_vscan_params.end_angle() =  m_vscan_params.start_angle();
		new_vscan_params.step() = -m_vscan_params.step();
		new_vscan_params.num_beams() = m_vscan_params.num_beams();
		new_vscan_params.fov() =  m_vscan_params.fov();
		m_vscan_params = new_vscan_params;
		std::reverse(m_row_angles.begin(), m_row_angles.end());
	} 

	void reserseHorizontalParams() {
		ScanParams new_hscan_params;
		new_hscan_params.start_angle() =  m_hscan_params.end_angle();
		new_hscan_params.end_angle() =  m_hscan_params.start_angle();
		new_hscan_params.step() = -m_hscan_params.step();
		new_hscan_params.num_beams() = m_hscan_params.num_beams();
		new_hscan_params.fov() =  m_hscan_params.fov();
		m_hscan_params = new_hscan_params;
		std::reverse(m_col_angles.begin(), m_col_angles.end());
	} 	

protected:
	 std::vector<float> FillVector(const ScanParams& scan_params) {
	 	std::vector<float> res;
	 	float rad = scan_params.start_angle() * M_PI/180.0;
	 	float rad_step = scan_params.step() *M_PI/180.0;
	 	for(int i=0; i < scan_params.num_beams(); ++i) {
	 		res.push_back(rad);
	 		rad += rad_step;
	 	}
	 	return res;
	 }



	size_t FindClosest(const vector<float>& vec,
	                   const float& val) const {
	  size_t found = 0;
	  if (vec.front() < vec.back()) {
	    found = upper_bound(vec.begin(), vec.end(), val) - vec.begin();
	  } else {
	    found = vec.rend() - upper_bound(vec.rbegin(), vec.rend(), val);
	  }
	  if (found == 0) {
	    return found;
	  }
	  if (found == vec.size()) {
	    return found - 1;
	  }
	  auto diff_next = fabs(vec[found] - val);
	  auto diff_prev = fabs(val - vec[found - 1]);
	  return diff_next < diff_prev ? found : found - 1;
	}	 

	void printScanParams(const ScanParams& scan_params) {
		fprintf(stderr, " start_angle: %f \n", scan_params.start_angle());
		fprintf(stderr, " end_angle: %f \n", scan_params.end_angle());
		fprintf(stderr, " step: %f \n", scan_params.step());
		fprintf(stderr, " fov: %f \n", scan_params.fov());
		fprintf(stderr, " num_beams: %d \n", scan_params.num_beams());
	}

	void printScanAngles(const std::vector<float>& angles) {
		for(int i=0; i  < angles.size(); i++) {
			fprintf(stderr, "%f, ", angles[i]);
		}
		fprintf(stderr, "\n\n");
	}

private:

	ScanParams m_vscan_params;
	ScanParams m_hscan_params;

	std::vector<float> m_row_angles;
	std::vector<float> m_col_angles;

	std::vector<float> m_row_angles_cosines;
	std::vector<float> m_row_angles_sines;

	std::vector<float> m_col_angles_cosines;
	std::vector<float> m_col_angles_sines;

	LIDARType m_lidar_type;
};

#endif