//
// Created by bzdfzfer on 2021/9/7.
//
// from: 
// https://github.com/NKU-MobFly-Robotics/laser-line-segment

#ifndef CIRCLE_ODOMETRY_FSTRUCT_H
#define CIRCLE_ODOMETRY_FSTRUCT_H

#include <vector>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>


#define PI 3.1415926535898
#define distance_point(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))


//点信息

typedef struct _POINT
{
    double x;
    double y;
}POINT;

typedef struct _POINT3
{
    double x;
    double y;
    double z;
}POINT3;

typedef struct _CSData
{
    std::vector<unsigned int> index;//索引值
    std::vector<double> bearings;//角度
    std::vector<double> cos_value;//余弦
    std::vector<double> sin_value;//正弦
}CSdata;

typedef struct _RangeData
{
    std::vector<double> ranges;//role数值
    std::vector<double> xs;//x坐标
    std::vector<double> ys;//y坐标
    std::vector<double> zs;
}Rangedata;

//参数，从launch文件中读入
typedef struct _Params
{
    double angle_increment;//角度增量
    double angle_start;//初始角度
    double least_thresh;//正交拟合阈值
    double min_line_length;//拟合线段最短距离
    double predict_distance;//真实点与与预测点之间的距离阈值
    double max_pts_gap;
    unsigned int min_line_points;//一条线段包含的激光点个数
    unsigned int seed_line_points;//种子线段包含的激光点个数
    unsigned int laser_points_num;
    unsigned int pts_missing_tolerance=5;
}Params;


typedef struct _word_params
{
    double _role;
    double _theta_one;
    double _theta_two;
}word_params;

typedef struct _signal_params
{
    double distance_signal;
}signal_params;

//直线段信息结构体
typedef struct _line
{
    double a;//直线参数
    double b;
    double c;
    double var;
    double len;
    double theta;
    double vdist;
    double z1;
    double z2;
    double zm;
    int left;//直线范围
    int right;
    int mid;
    int laserIdx;
    POINT p1;
    POINT p2;
    POINT pm;
    POINT vp;
    POINT nvp;
    bool inte[2] = {false, false};
}line;

//直线方程式结构体
typedef struct _least
{
    double a;
    double b;
    double c;
}least;


typedef struct _generate_line
{
    //first point
    double x1;
    double y1;
    //end point
    double x2;
    double y2;
}gline;

typedef struct _generate_line3d
{
    //first point
    double x1;
    double y1;
    double z1;
    //end point
    double x2;
    double y2;
    double z2;
}gline3d;


#endif //CIRCLE_ODOMETRY_FSTRUCT_H
