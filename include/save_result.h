#ifndef SAVE_RESULT_H
#define SAVE_RESULT_H


#include <string>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "projection_params.h"
#include "plane_params.h"


#define RAS_MAGIC   0x59a66a95

class SaveMultiPlanes {
public:

    SaveMultiPlanes() {}
    SaveMultiPlanes(std::string save_path, ProjectionParams proj_params):
        m_out_path(save_path), 
        m_proj_params(proj_params) {
        ROWS = m_proj_params.rows();
        COLS = m_proj_params.cols();
        m_SegmentImage=(unsigned char *)calloc(ROWS*COLS,sizeof(unsigned char));

    }

    ~SaveMultiPlanes() {
        free(m_SegmentImage);
    }

    void run(cv::Mat plabel_img, 
             std::vector<PlaneParams> plane_params,
             int idx,
             std::string seg_file_prefix,
             std::string seg_file_suffix,
             std::string params_file_prefix,
             std::string params_file_suffix) {
        m_plabel_image = plabel_img;
        m_plane_params = plane_params;

        genSegmentImageFromPLabelMat();

        save_raster_file(seg_file_prefix, idx, seg_file_suffix);
        save_plane_params(params_file_prefix, idx, params_file_suffix);
    }

    void run(cv::Mat plabel_img, 
             cv::Mat plabel_img_c,
             std::vector<PlaneParams> plane_params,
             int idx,
             std::string seg_file_prefix="perc.test.",
             std::string params_file_prefix="perc.test.",
             std::string seg_file_suffix=".ms-seg",
             std::string params_file_suffix=".ms-nor") {
        m_plabel_image = plabel_img;
        m_plane_params = plane_params;

        genSegmentImageFromPLabelMat();

        std::string pathLabelImg = m_out_path + "/pngs/" + seg_file_prefix + std::to_string(idx) + ".png";
        std::string pathLabelImgColor = m_out_path + "/pngs/" + seg_file_prefix + std::to_string(idx) + "_color.png";

        fprintf(stderr, "label image path : %s\n", pathLabelImg.c_str() );
        fprintf(stderr, "color label image path : %s\n", pathLabelImgColor.c_str() );

        cv::imwrite(pathLabelImg, plabel_img);
        cv::imwrite(pathLabelImgColor, plabel_img_c);

        save_raster_file(seg_file_prefix, idx, seg_file_suffix);
        save_plane_params(params_file_prefix, idx, params_file_suffix);
    }    

    void save_raster_file(std::string file_prefix, int file_idx, std::string file_suffix) {
        std::string file_name;
        file_name = m_out_path + file_prefix + std::to_string(file_idx)  + file_suffix;
        FILE *fptw;
        if ((fptw=fopen(file_name.c_str(),"w")) == NULL)
        {
            fprintf(stderr,"Unable to open file %s.\n",file_name.c_str());
            fprintf(stderr,"Check SegPathname and permissions?\n");
            return;
        }

        int RasterHeader[8];        
        RasterHeader[0]=RAS_MAGIC;
        RasterHeader[1]=COLS;
        RasterHeader[2]=ROWS;
        RasterHeader[3]=8;
        RasterHeader[4]=ROWS*COLS;
        RasterHeader[5]=0;
        RasterHeader[6]=0;
        RasterHeader[7]=0;
        fwrite(RasterHeader,4,8,fptw);

        fwrite(m_SegmentImage,1,ROWS*COLS,fptw);
        fclose(fptw);

        fprintf(stderr, "SegmentImage saved ...\n");
    }

    void save_plane_params(std::string file_prefix, int file_idx, std::string file_suffix) {
        std::string file_name;
        file_name = m_out_path + file_prefix + std::to_string(file_idx)  + file_suffix;
        FILE *fptw;
        if ((fptw=fopen(file_name.c_str(),"w")) == NULL)
        {
            fprintf(stderr,"Unable to open file %s.\n",file_name.c_str());
            fprintf(stderr,"Check SegPathname and permissions?\n");
            return;
        }

        int c = m_plane_params.size();
        fprintf(fptw,"%d\n",c);
        for (int i=10; i<10+c; i++)
            // fprintf(fptw,"%d\t%lf\t%lf\t%lf\t%lf\n",
            //     i,
            //     m_plane_params[i-10].m_a,
            //     m_plane_params[i-10].m_b,
            //     m_plane_params[i-10].m_c,
            //     m_plane_params[i-10].m_d);

            if(m_proj_params.lidar_type() == ProjectionParams::LIDARType::eSEGCOMPPERCEPTRON) {

            fprintf(fptw,"%d\t%lf\t%lf\t%lf\n",
                i,
                m_plane_params[i-10].m_b,
                m_plane_params[i-10].m_c,
                m_plane_params[i-10].m_a);

            }
            else {
            fprintf(fptw,"%d\t%lf\t%lf\t%lf\n",
                i,
                m_plane_params[i-10].m_a,
                m_plane_params[i-10].m_b,
                m_plane_params[i-10].m_c);                
            }
        fclose(fptw);
        fprintf(stderr, "plane params saved ...\n");
    }

    void genSegmentImageFromPLabelMat() {
        std::map<int, int> plabel_index_map;
        for(int i=0; i < m_plane_params.size(); i++) {
            auto pl_param = m_plane_params[i];
            fprintf(stderr, "plane[%d] -- label: %d \n", i, pl_param.m_plabel);
            plabel_index_map.insert( {pl_param.m_plabel, i} );
        }

        for(int i=0; i < ROWS; i ++ ) {
            for(int j=0; j < COLS; j ++ ) {
                uint16_t plabel = m_plabel_image.at<uint16_t>(i,j);
                if(plabel>0 && plabel < 255) {
                    m_SegmentImage[i*COLS + j] = 10 + plabel_index_map[plabel];
                }
                else {
                    m_SegmentImage[i*COLS + j] = 1;
                }
            }
        }
    }

private:

    // parameters.
    std::string m_out_path;
    ProjectionParams m_proj_params;

    // input.
    std::vector<PlaneParams> m_plane_params;
    cv::Mat m_plabel_image;

    // middle variables.
    int ROWS, COLS;
    unsigned char* m_SegmentImage;
};

#endif