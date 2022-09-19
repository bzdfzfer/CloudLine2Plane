#ifndef CLOUDLINE2PLANE_PLANE_EXTRACTION_H
#define CLOUDLINE2PLANE_PLANE_EXTRACTION_H

#include "line_extraction.h"
#include "line_clustering.h"
#include "plane_params.h"



class PlaneExtraction {
public:
    PlaneExtraction();

    void loadParams(std::string config_file);

    void segment(const CloudT::Ptr& cloud_in_ptr, bool colwise=true);

    void segment_unordered(const CloudT::Ptr& cloud_in_ptr, bool colwise=true);

    ProjectionParams proj_params() { return m_proj_params; }

    // output.
    cv::Mat labelImage() { return m_plabel_image; }
    std::vector<PlaneParams> planeParamsVec() { return m_pl_params_vec; }

    std::vector<std::vector<line>> getVSweepLines() { return m_le.getVSweepLines(); }
    std::vector<std::vector<line>> getHSweepLines() { return m_le.getHSweepLines(); }

    std::vector<std::vector<gline3d>> getVSweepGLines() { return m_le.getVSweepGlines(); }
    std::vector<std::vector<gline3d>> getHSweepGLines() { return m_le.getHSweepGlines(); }

    cv::Mat getHLineImage() { return m_le.getHLImage(); }
    cv::Mat getVLineImage() { return m_le.getVLImage(); }


    std::vector<double> getPETimeVec() { return pe_times; }
    std::vector<double> getPRTimeVec() { return m_le.getProjectionTimesVec(); }
    std::vector<double> getLETimeVec() { return m_le.getLineExtractionTimeVec(); }
    
 private:
    // functions.
    LineExtraction m_le;
    LineClustering m_lc;

    // variables.
    int m_frame_cnt;

    // parameters.
    ProjectionParams m_proj_params;

    // output variables.
    cv::Mat m_plabel_image;
    std::vector<PlaneParams> m_pl_params_vec;


    std::vector<double> pe_times;

};

#endif