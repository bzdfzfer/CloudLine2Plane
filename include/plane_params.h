#ifndef CLOUDLINE2PLANE_PLANE_PARAMS_H
#define CLOUDLINE2PLANE_PLANE_PARAMS_H

#include <vector>
#include <map>

#include "fstruct.h"
#include "common.h"

struct PixelCoord {
  PixelCoord() : row(0), col(0) { }
  PixelCoord(int16_t row_, int16_t col_) : row(row_), col(col_) { }
  PixelCoord operator+(const PixelCoord& other) const {
    return PixelCoord(row + other.row, col + other.col);
  }

  PixelCoord operator-(const PixelCoord& other) const {
    return PixelCoord(row - other.row, col - other.col);
  }

  int16_t row;
  int16_t col;
};

class PlaneParams {
public:
	PlaneParams() : m_a(0), m_b(0), m_c(0), m_d(0) {
		m_seed_row = -1;
		m_seed_col = -1;
		m_cross_hbound = false;
	}

	float m_a;
	float m_b;
	float m_c;
	float m_d;
	int m_seed_row;
	int m_seed_col;
	
	int m_plabel;
	bool m_cross_hbound;

	std::vector<std::pair<PixelCoord,PixelCoord>> m_line_seg_idxs;
	std::vector<PixelCoord> m_contours;
	std::vector<PixelCoord> m_convexhulls;
	PixelCoord m_center_pix;
	pcl::PointXYZ m_center_pt;
	pcl::PointXYZ m_vertical_pt;
};

// each bin may contain different plane indexes.
class PlaneContainer {
public:
	PlaneContainer() {}

	inline size_t Size() const { return m_planes_cnt.size(); }

	inline bool IsEmpty() const { return m_planes_cnt.empty(); }

	inline void Insert(int key) { 
		std::pair<std::map<int, int>::iterator, bool> ret;
		ret = m_planes_cnt.insert( {key, 1} );
		// insert operation does not overlap later identical key value.
		if(!ret.second) {
			++ m_planes_cnt[key];
		}
	}

	inline void Clear() { m_planes_cnt.clear(); }

	inline std::map<int, int>& planes() { return m_planes_cnt; }

	inline std::vector<int> plane_ids() {
		std::vector<int> key_vecs;
		for(auto iter=m_planes_cnt.begin(); iter != m_planes_cnt.end(); iter++) {
			key_vecs.push_back(iter->first);
		}
		return key_vecs;
	}
	
	inline std::vector<int> plane_nums() {
		std::vector<int> val_vecs;
		for(auto iter=m_planes_cnt.begin(); iter!=m_planes_cnt.end(); iter++) {
			val_vecs.push_back(iter->second);
		}
		return val_vecs;
	}

private:
	std::map<int, int> m_planes_cnt;
};

#endif