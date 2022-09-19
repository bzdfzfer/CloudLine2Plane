#ifndef CLOUDLINE2PLANE_COMMON_H
#define CLOUDLINE2PLANE_COMMON_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv/cv.h>



using PointT=pcl::PointXYZ;
using CloudT=pcl::PointCloud<PointT>;

const int COLOR_TABLE_SIZE = 82;
static const unsigned char default_colors[COLOR_TABLE_SIZE][3] =
{
    {255, 0, 0}, 
    {255, 255, 0}, 
    {100, 20, 50}, 
    {0, 30, 255}, 
    {10, 255, 60}, 
    {80, 10, 100}, 
    {0, 255, 200}, 
    {10, 60, 60}, 
    {255, 0, 128}, 
    {60, 128, 128}, 
    {15, 99, 98}, 
    {102, 159, 227}, 
    {245, 180, 173}, 
    {37, 229, 49}, 
    {110, 81, 166}, 
    {238, 29, 94}, 
    {232, 57, 157}, 
    {17, 209, 187}, 
    {117, 165, 112}, 
    {38, 25, 29}, 
    {9, 201, 164}, 
    {149, 110, 55}, 
    {238, 108, 3}, 
    {113, 69, 235}, 
    {47, 52, 61},     
    {217, 129, 225}, 
    {19, 58, 205}, 
    {46, 234, 98}, 
    {206, 249, 79}, 
    {134, 45, 252}, 
    {96, 61, 59}, 
    {149, 203, 31}, 
    {119, 178, 190}, 
    {92, 168, 242}, 
    {137, 78, 217}, 
    {186, 167, 91}, 
    {28, 186, 21}, 
    {105, 233, 1}, 
    {203, 56, 122}, 
    {27, 62, 167}, 
    {25, 158, 100}, 
    {211, 180, 176}, 
    {114, 44, 99}, 
    {49, 8, 139}, 
    {164, 145, 217}, 
    {126, 76, 130}, 
    {89, 231, 188}, 
    {85, 162, 91}, 
    {180, 140, 20}, 
    {209, 157, 229}, 
    {141, 146, 11}, 
    {126, 57, 163}, 
    {27, 66, 110}, 
    {191, 87, 92}, 
    {246, 71, 73}, 
    {122, 56, 14}, 
    {102, 230, 66}, 
    {35, 61, 101}, 
    {126, 113, 241}, 
    {18, 67, 15}, 
    {247, 80, 161}, 
    {131, 206, 218}, 
    {39, 105, 157}, 
    {21, 42, 244}, 
    {114, 160, 187}, 
    {59, 27, 244}, 
    {73, 129, 219}, 
    {12, 37, 152}, 
    {113, 35, 10}, 
    {226, 54, 204}, 
    {113, 173, 157}, 
    {146, 49, 235}, 
    {237, 89, 86}, 
    {139, 110, 128}, 
    {0, 96, 160}, 
    {188, 155, 188}, 
    {177, 101, 189}, 
    {13, 240, 226}, 
    {166, 225, 7}, 
    {48, 68, 61}, 
    {253, 181, 106}, 
    {155, 72, 156}
};


inline CloudT CloudFromCoords(float *P[3], int rows, int cols) {
    CloudT segcomp_cloud;
    segcomp_cloud.width = cols;
    segcomp_cloud.height = rows;
    segcomp_cloud.is_dense = false;
    segcomp_cloud.points.resize(cols*rows);
    for (int r=0; r < rows; r++) {
        for (int c=0; c <  cols; c++) {
            PointT pt;
            // pt.x = 0.01*P[2][r*cols+c];
            // pt.y = 0.01*P[1][r*cols+c];
            // pt.z = 0.01*P[0][r*cols+c];
            pt.x = P[2][c*rows+r];
            pt.y = P[0][c*rows+r];
            pt.z = P[1][c*rows+r];
            segcomp_cloud.at(c, r) = pt;
        }
    }  
    return segcomp_cloud;  
}

inline CloudT loadPTXCloud(std::string file_name) {
    CloudT cloud_tmp;

    std::ifstream input(file_name.c_str(), std::ios::in);
    if(!input.good()){
        std::cerr << "Could not read file: " << file_name << std::endl;
    }

    int i=0;
    std::string line;
    while(getline(input, line)) {
        std::stringstream ss(line);
        // float px, py, pz;
        // ss >> px >> py >> pz;
        // if(i==0) {
        //     std::cout << px << ", " << py << ", " << pz << std::endl;
        // }
        std::string spx, spy, spz;
        std::getline(ss, spx, ' ');
        std::getline(ss, spy, ' ');
        std::getline(ss, spz, ' ');
        float px, py, pz;
        if(spx=="NaN" || spy=="NaN" || spz=="NaN") {
            px = py = pz = std::numeric_limits<float>::quiet_NaN();
        } else {
            px = std::stof(spx);
            py = std::stof(spy);
            pz = std::stof(spz);
        }

        PointT pt;   
        pt.x = px;
        pt.y = py;
        pt.z = pz;  
        cloud_tmp.points.push_back(pt);
    }

    return cloud_tmp;
}


inline CloudT CloudFromCoordsVLP(float *P[3], int rows, int cols) {
    CloudT segcomp_cloud;
    segcomp_cloud.width = cols;
    segcomp_cloud.height = rows;
    segcomp_cloud.is_dense = false;
    segcomp_cloud.points.resize(cols*rows);
    for (int r=0; r < rows; r++) {
        for (int c=0; c <  cols; c++) {
            PointT pt;
            // pt.x = 0.01*P[2][r*cols+c];
            // pt.y = 0.01*P[1][r*cols+c];
            // pt.z = 0.01*P[0][r*cols+c];
            // pt.x = P[0][c*rows+r];
            // pt.y = P[1][c*rows+r];
            // pt.z = P[2][c*rows+r];
            pt.x = P[0][r*cols+c];
            pt.y = P[1][r*cols+c];
            pt.z = P[2][r*cols+c];            
            segcomp_cloud.at(c, r) = pt;
            // if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
            //     std::cout << "invalid point at: (" << c << ", " << r << ")" << std::endl;
            // }
        }
    }  
    return segcomp_cloud;  
}

inline void saveTimesToFile(std::string out_file, std::vector<double> times_vec) {
    std::cout << "saving timings to file : " << out_file  << " ...." << std::endl;
    FILE* fptw = NULL;
    fptw = fopen(out_file.c_str(), "w");
    if(fptw == NULL) {
        std::cout << "cannot open " << out_file << std::endl;
        return;
    }

    for(int i=0; i < times_vec.size(); i++) {
        fprintf(fptw, "%lf\n", times_vec[i]);
    }
    fclose(fptw);
    std::cout << "timing file saved ..." << std::endl;    
}

#endif