//
// Created by bzdfzfer on 2021/9/7.
//
// from: 
// https://github.com/NKU-MobFly-Robotics/laser-line-segment

#ifndef CIRCLE_ODOMETRY_LINE_FEATURE_H
#define CIRCLE_ODOMETRY_LINE_FEATURE_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <limits>
#include "fstruct.h"


namespace line_feature
{

    class LineFeature
    {
    public:
        LineFeature();
        //
        ~LineFeature();
        //子函数声明
        //设置bearing，一次设置就行
        void setCosSinData(const std::vector<double>&, 
                           const std::vector<double>&,
                           const std::vector<double>&, 
                           const std::vector<unsigned int>&);

        void setCSDataIdx(const std::vector<unsigned int>&);
        
        //设置range，每次都需要传递range消息，在主入口函数的回调函数进行
        void setRangeData(const std::vector<double>&);

        void setRangeData(const std::vector<double>& xs, 
                          const std::vector<double>& ys);

        void setRangeData(const std::vector<double>& xs, 
                          const std::vector<double>& ys, 
                          const std::vector<double>& zs);

        void setRangeDataNew(const std::vector<double>& xs,
                             const std::vector<double>& ys,
                             const std::vector<double>& rs);

        void calcVarianceOfFittedLine(line& l_param, int start_idx, int stop_idx);

        //返回直线分割结果
        void extractLines(std::vector<line>&,std::vector<gline>&);

        void extractLinesNew(std::vector<line>&,std::vector<gline3d>&, int);

        void extractLines(std::vector<line>&,std::vector<gline3d>&, int laser_idx);

        //设置参数
        void set_angle_increment(double);
        void set_angle_start(double);
        void set_least_threshold(double);
        void set_min_line_length(double);
        void set_predict_distance(double);
        void set_min_line_points(unsigned int);
        void set_seed_line_points(unsigned int);
        void set_laser_points_num(unsigned int);
        void set_pts_missing_tolerance(unsigned int);
        void set_max_pts_gap(double);

        int getRangeDataSize() { return range_data_.ranges.size(); }
        int getBearingAngleSize() { return cs_data_.index.size();}
        void getXYData(std::vector<double> &xs, std::vector<double> &ys) { xs = range_data_.xs; ys = range_data_.ys; }
        double  get_angle_increment() { return params_.angle_increment; }
        double get_angle_start() { return params_.angle_start; } 
        double get_least_thresh() { return params_.least_thresh; } 
        double get_line_length() { return params_.min_line_length; } 
        double get_predict_distance() { return params_.predict_distance; } 
        int get_min_line_points() { return params_.min_line_points; } 
        int get_seed_line_points() { return params_.seed_line_points; } 

    private:
        //检测种子点，障碍物聚类的子函数
        //bool detectseed(PoinT );
        //障碍物检测，区域生长
        //int regiongrow(int);
        //通过激光数据的首末索引值进行直线方程的求解
        least leastsquare(int,int,int);
        //检测种子直线
        bool detectline(const int,const int);
        //通过种子直线，复原出整条直线，并进行最后的求定
        int detectfulline(const int);
        //整理整条直线
        void cleanline();
        //删除小于长度阈值的线段
        bool delete_short_line(const int,const int);
        //
        void generate(std::vector<gline>& temp_line2);

        void generate(std::vector<gline3d>& temp_line2, int laser_idx);

        void generateNew(std::vector<gline3d>& temp_line2, int laser_idx);

        bool delete_shadow_line(const line& l_t);
        

    private:
        CSdata cs_data_;
        Rangedata range_data_;
        Params params_;
        std::vector<unsigned int> point_num_;
        //线段结构体信息
        std::vector<line> m_line;
        //直线拟合中间传递变量，已设为全局变量
        least m_least;
        //拟合中间变量
        double mid1;
        double mid2;
        double mid3;
        double mid4;
        double mid5;

        std::vector<unsigned int> m_valid_index;
    };

}//namespace line_feature

#endif //CIRCLE_ODOMETRY_LINE_FEATURE_H
