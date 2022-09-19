//
// Created by bzdfzfer on 2021/9/7.
//

#include "line_feature.h"
#include "sys/time.h"
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace line_feature
{
//构造函数
    LineFeature::LineFeature()
    {

    }

//析构函数
    LineFeature::~LineFeature()
    {

    }

//set paramters
    void LineFeature::set_angle_increment(double angle_increment)
    {
        params_.angle_increment = angle_increment;
    }

    void LineFeature::set_angle_start(double angle_start)
    {
        params_.angle_start = angle_start;
    }

    void LineFeature::set_least_threshold(double least_thresh)
    {
        params_.least_thresh = least_thresh;
    }

    void LineFeature::set_min_line_length(double min_line_length)
    {
        params_.min_line_length = min_line_length;
    }

    void LineFeature::set_predict_distance(double predict_distance)
    {
        params_.predict_distance = predict_distance;
    }

    void LineFeature::set_min_line_points(unsigned int min_line_points)
    {
        params_.min_line_points = min_line_points;
    }

    void LineFeature::set_seed_line_points(unsigned int seed_line_points)
    {
        params_.seed_line_points = seed_line_points;
    }

    void LineFeature::set_laser_points_num(unsigned int laser_pts_num) {
        params_.laser_points_num = laser_pts_num;
    }

    void LineFeature::set_pts_missing_tolerance(unsigned int missing_num) {
        params_.pts_missing_tolerance = missing_num;
    }

    void LineFeature::set_max_pts_gap(double max_pts_gap) {
        params_.max_pts_gap = max_pts_gap;
    }
    
//设定正余弦等参数
    void LineFeature::setCosSinData(const std::vector<double>& bearings,
                                    const std::vector<double>& cos_value,
                                    const std::vector<double>& sin_value,
                                    const std::vector<unsigned int>& index)
    {
        cs_data_.index = index;
        cs_data_.cos_value = cos_value;
        cs_data_.sin_value = sin_value;
        cs_data_.bearings = bearings;
    }

    void LineFeature::setCSDataIdx(const std::vector<unsigned int>& index) {
        cs_data_.index = index;
    }



//设定激光点信息
    void LineFeature::setRangeData(const std::vector<double>& ranges)
    {
        range_data_.ranges = ranges;
        range_data_.xs.clear();
        range_data_.ys.clear();
        m_valid_index.clear();
        int i=0;
        for (std::vector<unsigned int>::const_iterator cit = cs_data_.index.begin();
             cit != cs_data_.index.end(); ++cit)
        {
            if(ranges[*cit] <0.001 || ranges[*cit] > 80.0) {
                range_data_.xs.push_back(0);
                range_data_.ys.push_back(0);
                i++;
                continue;
            }
            if(std::isnan(ranges[*cit])) {
                // fprintf(stderr, "[LF-91] `````` invalid laser points occured\n");
            }
            else {
                m_valid_index.push_back(i);
            }
            i++;
            range_data_.xs.push_back(cs_data_.cos_value[*cit] * ranges[*cit]);
            range_data_.ys.push_back(cs_data_.sin_value[*cit] * ranges[*cit]);
        }
    }

    void LineFeature::setRangeData(const std::vector<double>& xs, const std::vector<double>& ys) {

        range_data_.xs = xs;
        range_data_.ys = ys;
        range_data_.ranges.clear();
        m_valid_index.clear();
        for(int i =0; i < xs.size(); i++) {
            double rd = sqrt(xs[i]*xs[i] + ys[i]*ys[i]);
            if(rd < 0.001 || rd > 80.0) {
                range_data_.ranges.push_back(rd);
                continue;
            }

            if(std::isnan(xs[i]) || std::isnan(ys[i]))
            {
                // fprintf(stderr, "[LF-106]+++++++ invalid points occured at idx: %d\n", i);
            }
            else {
                m_valid_index.push_back(i);
            }
            // TODO: handle invalid points.
            range_data_.ranges.push_back(rd);
        }
    }

    void LineFeature::setRangeData(const std::vector<double>& xs, 
                                   const std::vector<double>& ys,
                                   const std::vector<double>& zs) {

        range_data_.xs = xs;
        range_data_.ys = ys;
        range_data_.zs = zs;
        range_data_.ranges.clear();
        m_valid_index.clear();
        for(int i =0; i < xs.size(); i++) {
            double rd = sqrt(xs[i]*xs[i] + ys[i]*ys[i]);
            if(rd < 0.001 || rd > 80.0) {
                range_data_.ranges.push_back(rd);
                continue;
            }

            if(std::isnan(xs[i]) || std::isnan(ys[i]) || std::isnan(zs[i]))
            {
                // fprintf(stderr, "[LF-106]+++++++ invalid points occured at idx: %d\n", i);
            }
            else {
                m_valid_index.push_back(i);
            }
            // TODO: handle invalid points.
            range_data_.ranges.push_back(rd);
        }
    }

    void LineFeature::setRangeDataNew(const std::vector<double>& xs,
                                      const std::vector<double>& ys,
                                      const std::vector<double>& rs) {
        range_data_.xs = xs;
        range_data_.ys = ys;
        range_data_.ranges = rs;
        m_valid_index.clear();

        for(int i=0; i < rs.size(); i++) {
            if(std::isnan(rs[i])==false && rs[i] > 0.001 && rs[i] < 80.0) {
                m_valid_index.push_back(i);
            }
        }
    }

//一次最小二乘求解直线参数
    least LineFeature::leastsquare(int start,int end,int firstfit)
    {
        double w1 = 0, w2 = 0, w3 = 0;
        least temp;
        double n = end - start + 1;

        
        //firstfit = true;
        int k_start = 0;
        int k_end = 0;

        if(firstfit == 1)
        {
            mid1 = 0;
            mid2 = 0;
            mid3 = 0;
            mid4 = 0;
            mid5 = 0;
            int i = 0;
            int k = 0;
            for(i = start;i <= end;i++)
            {
                k = m_valid_index[i];
                mid1+=range_data_.xs[k];
                mid2+=range_data_.ys[k];
                mid3+=range_data_.xs[k]*range_data_.xs[k];
                mid4+=range_data_.ys[k]*range_data_.ys[k];
                mid5+=range_data_.xs[k]*range_data_.ys[k];
            }                

        }
        else
        {
            if(firstfit == 2)
            {
                k_end = m_valid_index[end];
                mid1+=range_data_.xs[k_end];
                mid2+=range_data_.ys[k_end];
                mid3+=range_data_.xs[k_end]*range_data_.xs[k_end];
                mid4+=range_data_.ys[k_end]*range_data_.ys[k_end];
                mid5+=range_data_.xs[k_end]*range_data_.ys[k_end];                    
      
            }
            else
            {
                k_start = m_valid_index[start];
                mid1+=range_data_.xs[k_start];
                mid2+=range_data_.ys[k_start];
                mid3+=range_data_.xs[k_start]*range_data_.xs[k_start];
                mid4+=range_data_.ys[k_start]*range_data_.ys[k_start];
                mid5+=range_data_.xs[k_start]*range_data_.ys[k_start];              
            }
        }
        w1 = n*mid5-mid1*mid2;
        w2 = mid2*mid2-n*mid4-mid1*mid1+n*mid3;
        w3 = mid1*mid2-n*mid5;
        //ax+by+c = 0 等价于 y = kx + b;kx-y + b = 0 //a = k,c = b,b=-1
        if(w1==0)
        {
            temp.a = -1;
            temp.b = 0;
            temp.c = mid1/n;
            // std::cout << ">>>>>>>>zero w1 occured ...." << std::endl;
            // std::cout << "(w1, w2, w3): " << w1 << ", " << w2 << ", " << w3 << std::endl;
        }
        else
        {
            // double tmp_s1 = mid1*mid4 - mid2*mid5;
            // double tmp_s2 = mid2*mid3 - mid1*mid5;
            // double tmp_s3 = mid5*mid5 - mid3*mid4;
            // temp.a = -tmp_s1/tmp_s2;
            // temp.b = -1;
            // temp.c = -tmp_s3/tmp_s2;

            temp.a = (-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
            temp.b = -1;
            temp.c = (mid2-temp.a*mid1)/n;

            // temp.a = (-w2-sqrt(w2*w2-4*w1*w3))/2.0/w1;
            // temp.b = -1;            
            // temp.c = (mid2-temp.a*mid1)/n;

            // double delta_w = w2*w2 - 4*w1*w3;
            // double sqrt_dw = sqrt(delta_w);
            // double minus_a = (-w2 - sqrt_dw) / 2.0/w1;

            // std::cout << "larger a vas smaller a: " << temp.a << ", " << minus_a << std::endl;
            // std::cout << "delta_w:  " << delta_w << ", sqrt_dw: " << sqrt_dw << std::endl;
            // std::cout << "(w1, w2, w3): " << w1 << ", " << w2 << ", " << w3 << std::endl;

        }

        // double det_x = n*mid3 - mid1*mid1;
        // double det_y = n*mid4 - mid2*mid2;
        // if(det_x > det_y) {
        //     double tmp_lxk = (n*mid5 - mid1*mid2)/det_x;
        //     double tmp_lxb = (mid2 - tmp_lxk*mid1)/n;

        //     temp.a = tmp_lxk;
        //     temp.b = -1;
        //     temp.c = tmp_lxb;
        // } else {
        //     if(det_y>0) {
        //         double tmp_lyk = (n*mid5 - mid1*mid2)/det_y;
        //         double tmp_lyb = (mid1 - tmp_lyk*mid2)/n;
        //         if(tmp_lyk == 0) {
        //             temp.a = -1;
        //             temp.b = 0;
        //             temp.c = mid1/n;
        //         } else {
        //             temp.a = 1/tmp_lyk;
        //             temp.b = -1;
        //             temp.c = -tmp_lyb/tmp_lyk;                     
        //         }
               
        //     } else {
        //         // invalid line.
        //         // fprintf(stderr, "[[[[[[[[[[[[[[[[[[[invalid line occur..]]]]]] \n");
        //         temp.a = 0.001;
        //         temp.b = 0.001;
        //         temp.c = 10000;
        //     }
        // }
        return temp;
    }
//判断下一个点是否在直线上，是，返回true；否则，返回false。
    bool LineFeature::detectline(const int start,const int num)
    {

        bool flag = false;
        //定义点到直线的垂直误差
        double error1 = 0;
        //定义下一点到预测位置的误差
        double error2 = 0;
        int i = 0;
        int k = 0;
        //预测下一点位置
        POINT m_pn;
        m_pn.x = 0;
        m_pn.y = 0;
        //下一点，y = kp*x;
        double kp = 0;
        double theta = 0;
        // cout << "start:" << start << ", num:" << num << endl;
        // cout << "valid point num: " << m_valid_index.size() << endl;
        for(i = start;i < start+num;i++)
        {
            // cout << "i:" << i << "raw idx:" << m_valid_index[i] << endl;
            // if(i >= m_valid_index.size()) {
            //     cout << "i: " << i << " vs: " << m_valid_index.size() << endl;
            // }
            k = m_valid_index[i];
            // if( k >= 1800)
            // {
            //     cout << "k: " << k << endl;
            // }

            // 10 degrees. 2.5*10 = 25, 5 degs, 12.5 pts.
            double r1 = range_data_.ranges[k];
            if(i>=1 && 
                (k - m_valid_index[i-1]>params_.pts_missing_tolerance ||
                 std::fabs(r1 - range_data_.ranges[m_valid_index[i-1]]) > params_.max_pts_gap) )
            {
                flag = true;
                break;
            } 

            // cout << i << ": before perpendicular distance " << endl;
            //到直线的垂直距离
            // error1 = fabs(((m_least.a)*range_data_.xs[k]+(m_least.b)*range_data_.ys[k]+m_least.c))/sqrt((1+(m_least.a)*(m_least.a)));
            error1 = fabs(((m_least.a)*range_data_.xs[k]+(m_least.b)*range_data_.ys[k]+m_least.c))/sqrt(((m_least.b)*(m_least.b)+(m_least.a)*(m_least.a)));

            if(error1 > params_.least_thresh)
            {
                flag = true;
                break;
            }

            theta = params_.angle_increment*k + params_.angle_start;
            if(fabs((fabs(theta) - PI/2))<1e-05)
            {
                m_pn.x = 0;
                m_pn.y = m_least.c;
            }
            else
            {
                kp = tan(theta);
                m_pn.x = (m_least.c)/(kp - m_least.a);
                m_pn.y = kp*m_pn.x;
            }

            // cout << i << ": before distance_point " << endl;

            //计算到预测点之间的误差
            error2 = distance_point(range_data_.xs[k],range_data_.ys[k],m_pn.x,m_pn.y);
            
            // cout << i << ": after distance point" << error2 << endl;
            if(error2 > params_.predict_distance)
            {
                flag = true;
                break;
            }
            // cout << "for loop finished ..." << endl;
            // cout << "-----------------------" << endl;
        }
        if(flag)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

//检测完整的直线段
    int LineFeature::detectfulline(const int start)
    {
        line m_temp;

        bool flag1 = true;
        bool flag2 = true;
        int n1 = 0;
        int n2 = 0;
        double a = 0;
        double b = 0;
        double c = 0;

        a = m_least.a;
        b = m_least.b;
        c = m_least.c;

        n2 = start + params_.seed_line_points;

        least m_result;
        m_result.a = 0;
        m_result.b = 0;
        m_result.c = 0;
        //向前生长
        while(flag2)
        {
            unsigned int idx = m_valid_index[n2];
            if(n2 >0 && idx - m_valid_index[n2-1] > params_.pts_missing_tolerance) {
                break;
            }

            // if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(1+a*a)))<params_.least_thresh)
            if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(b*b+a*a)))<params_.least_thresh)
            {
                m_least = leastsquare(start,n2,2);
                if(n2 < (m_valid_index.size() - 1))
                {
                    n2 = n2 + 1;
                    a = m_least.a;
                    b = m_least.b;
                    c = m_least.c;
                }
                else
                {
                    flag2 = false;
                }
            }
            else
            {
                flag2 = false;
            }
        }
        if(n2 < m_valid_index.size()-1) {
            n2 = n2-1;
        }
        //向后回溯

        n1 = start - 1;
        if(n1 < 0)
        {
            flag1 = false;
        }
        while(flag1)
        {       
            unsigned int idx = m_valid_index[n1];
            if(n1 < range_data_.xs.size()-1 && m_valid_index[n1+1] - idx > params_.pts_missing_tolerance) {
                break;
            }

            // if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(1+a*a)))<params_.least_thresh)
            if((fabs(a*range_data_.xs[idx]+b*range_data_.ys[idx]+c)/(sqrt(b*b+a*a)))<params_.least_thresh)            
            {
                m_least = leastsquare(n1,n2,3);
                if(n1>0)
                {
                    n1 = n1 - 1;
                    a = m_least.a;
                    b = m_least.b;
                    c = m_least.c;
                }
                else
                {
                    flag1 = false;
                }
            }
            else
            {
                flag1 = false;
            }
        }
        n1 = n1+1;

        m_temp.left = n1;
        m_temp.right = n2;
        //此处是统一再做一次拟合，可能以一定步长进行拟合搜索的时候，需要这样完整的拟合过程，此时不需要
        m_result = leastsquare(n1,n2,1);
        m_temp.a = m_result.a;
        m_temp.b = m_result.b;
        m_temp.c = m_result.c;

        // if((n2-n1)>params_.min_line_points)
        if((m_valid_index[n2] - m_valid_index[n1])>params_.min_line_points)
        {
            if(delete_short_line(m_valid_index[n1],m_valid_index[n2]))
            {
                m_line.push_back(m_temp);
            }
            return n2;
        }
        else
        {
            return start;
        }
    }

    void LineFeature::cleanline()
    {
        if(m_line.size() < 2)
        {
            return;
        }

        int m = 0;
        int n = 0;
        int m_iter = 0;
        double error1 = 0;
        double error2 = 0;
        int line_temp = 0;
        least temp_least;
        temp_least.a = 0;
        temp_least.b = 0;
        temp_least.c = 0;

        double theta_one_ = 0;
        double theta_two_ = 0;
        double theta_d_ = 0;
        std::size_t q = 0,p = 0;

        for(p = 0; p < m_line.size() - 1; p++)
        {
            m = m_line[p].right;
            for(q = p+1;q < m_line.size();q++)
            {
                n = m_line[q].left;
                if(m >= n)
                {
                    theta_one_ = atan(m_line[p].a);
                    theta_two_ = atan(m_line[q].a);

                    theta_d_ = fabs(theta_one_ - theta_two_);

                    if((theta_d_<0.1)||(theta_d_>(PI - 0.1)))
                    {
                        // fprintf(stderr, "[LF-367] ###### merging two same lines into one\n");
                        int _left = std::min(m_line[p].left,m_line[q].left);

                        least m_temp = leastsquare(_left,m_line[q].right,1);

                        m_line[p].a = m_temp.a;
                        m_line[p].b = m_temp.b;
                        m_line[p].c = m_temp.c;

                        m_line[p].left = _left;
                        m_line[p].right = m_line[q].right;

                        m_line.erase(m_line.begin()+q);

                        m = m_line[p].right;
                        q = q - 1;
                        // fprintf(stderr, "[LF-480] ###### merged \n");

                    }


                }
            }
        }

        int k;
        //处理有相互链接关系的线段
        for(p = 0; p < (m_line.size() - 1); p++)
        {
            q = p+1;
            m = m_line[p].right;
            n = m_line[q].left;
            if(m >= n)
            {
                for(m_iter = n;m_iter <= m;m_iter++)
                {
                    line_temp = m_iter;
                    k = m_valid_index[m_iter];
                    // error1 = fabs(((m_line[p].a)*range_data_.xs[k]+(m_line[p].b)*range_data_.ys[k]+m_line[p].c))/sqrt((1+(m_line[p].a)*(m_line[p].a)));
                    // error2 = fabs(((m_line[q].a)*range_data_.xs[k]+(m_line[q].b)*range_data_.ys[k]+m_line[q].c))/sqrt((1+(m_line[q].a)*(m_line[q].a)));
                    error1 = fabs(((m_line[p].a)*range_data_.xs[k]+(m_line[p].b)*range_data_.ys[k]+m_line[p].c))/sqrt((m_line[p].b*m_line[p].b+(m_line[p].a)*(m_line[p].a)));
                    error2 = fabs(((m_line[q].a)*range_data_.xs[k]+(m_line[q].b)*range_data_.ys[k]+m_line[q].c))/sqrt((m_line[q].b*m_line[q].b+(m_line[q].a)*(m_line[q].a)));

                    if(error1 > error2)
                    {
                        break;
                    }
                }
                m_line[p].right = m_iter-1;
                temp_least = leastsquare(m_line[p].left,m_line[p].right,1);
                m_line[p].a = temp_least.a;
                m_line[p].b = temp_least.b;
                m_line[p].c = temp_least.c;

                m_line[q].left = m_iter;
                temp_least = leastsquare(m_line[q].left,m_line[q].right,1);
                m_line[q].a = temp_least.a;
                m_line[q].b = temp_least.b;
                m_line[q].c = temp_least.c;
            }
        }
    }

    bool LineFeature::delete_short_line(const int n1,const int n2)
    {

        if(distance_point(range_data_.xs[n1],range_data_.ys[n1],range_data_.xs[n2],range_data_.ys[n2])<params_.min_line_length)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool LineFeature::delete_shadow_line(const line& l_t) {
        // double gap_len = l_t.len/ (l_t.right - l_t.left + 1);
        double dist = std::fabs(l_t.c/std::sqrt(l_t.b*l_t.b + l_t.a*l_t.a));
        // if(gap_len < 0.04) // two point gap len at most 0.1 m. if larger, delete 
        if(dist < 0.2)
        {
            // fprintf(stderr, "[LF-505] --- gap_len: %f \n", gap_len);
            return true;
        }
        else
        {
            return false;
        }
    }


    void LineFeature::generate(std::vector<gline>& temp_line2)
    {
        gline line_temp;
        std::vector<gline> output;
        POINT endpoint1;
        POINT endpoint2;
        int m = 0,n = 0;
        double k1 = 0,k2 = 0;
        for(int i = 0;i < m_line.size();i++)
        {   
            // if(m_line[i].left >= m_valid_index.size() || m_line[i].right >= m_valid_index.size()) {
            //     cout << "line segment range i: " << i << ", left: " << m_line[i].left << ", right: " << m_line[i].right << endl;
            // }
            m = m_valid_index[m_line[i].left];
            n = m_valid_index[m_line[i].right];

            m_line[i].left = m;
            m_line[i].right = n;

            // cout << "line idx range [m,n]: " << m << ", " << n << endl;
            // if( m > n)
            //     cout << "descending line occured ....." << endl;
            calcVarianceOfFittedLine(m_line[i], m, n);

            // if(m_line[i].b!=0)
            // {
            //     endpoint1.x = (range_data_.xs[m]/m_line[i].a + range_data_.ys[m] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
            //     endpoint1.y = m_line[i].a*endpoint1.x + m_line[i].c;
            // }
            // else
            // {
            //     endpoint1.x = range_data_.ys[m];
            //     endpoint1.y = m_line[i].c/m_line[i].a;
            // }
            double l_a = m_line[i].a;
            double l_b = m_line[i].b;
            double l_c = m_line[i].c;
            double l_aa = l_a*l_a;
            double l_ab = l_a*l_b;
            double l_ac = l_a*l_c;
            double l_bb = l_b*l_b;
            double l_bc = l_b*l_c;
            double l_aa_p_bb = l_aa + l_bb;
            endpoint1.x = (l_bb*range_data_.xs[m] - l_ab*range_data_.ys[m] - l_ac)/l_aa_p_bb;
            endpoint1.y = (l_aa*range_data_.ys[m] - l_ab*range_data_.xs[m] - l_bc)/l_aa_p_bb;

            line_temp.x1 = endpoint1.x;
            line_temp.y1 = endpoint1.y;

            m_line[i].p1 = endpoint1;

            // if(m_line[i].b!=0)
            // {
            //     endpoint2.x = (range_data_.xs[n]/m_line[i].a + range_data_.ys[n] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
            //     endpoint2.y = m_line[i].a*endpoint2.x + m_line[i].c;
            // }
            // else
            // {
            //     endpoint2.x = range_data_.ys[n];
            //     endpoint2.y = m_line[i].c/m_line[i].a;
            // }


            endpoint2.x = (l_bb*range_data_.xs[n] - l_ab*range_data_.ys[n] - l_ac)/l_aa_p_bb;
            endpoint2.y = (l_aa*range_data_.ys[n] - l_ab*range_data_.xs[n] - l_bc)/l_aa_p_bb;

            line_temp.x2 = endpoint2.x;
            line_temp.y2 = endpoint2.y;

            m_line[i].p2 = endpoint2;

            m_line[i].len = distance_point(endpoint1.x, endpoint1.y, endpoint2.x, endpoint2.y);

            // calculate vertical point, theta, distance.
            double vp_x, vp_y, vp_theta, vp_dist;
            vp_x = -m_line[i].a*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            vp_y = -m_line[i].b*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            vp_theta = std::atan2(vp_y, vp_x);
            vp_dist = std::sqrt(vp_x*vp_x + vp_y*vp_y);

            m_line[i].theta = vp_theta;
            m_line[i].vdist = vp_dist;
            m_line[i].vp.x = vp_x;
            m_line[i].vp.y = vp_y;
            m_line[i].nvp.x = vp_x / vp_dist;
            m_line[i].nvp.y = vp_y / vp_dist;

            output.push_back(line_temp);
        }
        temp_line2 = output;
    }

//识别主函数
    void LineFeature::extractLines(std::vector<line>& temp_line1,std::vector<gline>& temp_line2)
    {
        int line_include = 0;
        m_line.clear();
        point_num_ = cs_data_.index;

        if(point_num_.size() < params_.min_line_points)
        {
            return;
        }
        //附近特征点数目

        for(unsigned int i = 0; i < (m_valid_index.size() - params_.min_line_points) ;i++)
        {
            // cout << "[LF 619] -- before least square" << endl; 
            m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
            // cout << "[LF 605] -- least square success " << endl;
            //std::cout<<m_least.a<<" "<<m_least.b<<" "<<m_least.c<<std::endl;
            if(detectline(i,params_.seed_line_points))
            {
                // cout << "[LF 605] -- before region grow " << endl;
                line_include = detectfulline(i);
                // cout << "[LF 605] -- after region grow " << endl;

                i = line_include;
            }

        }
        // cout << "[LF 613] ---before cleanline " << endl;

        cleanline();

        for(int p = 0; p < m_line.size();p++)
        {
            if(!delete_short_line(m_valid_index[m_line[p].left],m_valid_index[m_line[p].right]))
            {
                m_line.erase(m_line.begin()+p);
            }
        }

        // MergeHeadTailLines(m_line);
        
        generate(temp_line2);

        // delete radial lines.
        for(int p = 0; p < m_line.size();p++)
        {
            if(delete_shadow_line(m_line[p]))
            {
                m_line.erase(m_line.begin()+p);
                temp_line2.erase(temp_line2.begin()+p);
            }
        }        

        temp_line1 = m_line;
    }

    void LineFeature::generate(std::vector<gline3d>& temp_line2, int laser_idx)
    {
        gline3d line_temp;
        std::vector<gline3d> output;
        POINT endpoint1;
        POINT endpoint2;
        int m = 0,n = 0;
        double k1 = 0,k2 = 0;
        for(int i = 0;i < m_line.size();i++)
        {   
            // if(m_line[i].left >= m_valid_index.size() || m_line[i].right >= m_valid_index.size()) {
            //     cout << "line segment range i: " << i << ", left: " << m_line[i].left << ", right: " << m_line[i].right << endl;
            // }
            m = m_valid_index[m_line[i].left];
            n = m_valid_index[m_line[i].right];

            // find middle of valid points.
            int mid_idx = (m_line[i].left + m_line[i].right)/2;
            int valid_mid_pt_idx = m_valid_index[mid_idx];
            m_line[i].pm.x = range_data_.xs[valid_mid_pt_idx];
            m_line[i].pm.y = range_data_.ys[valid_mid_pt_idx];
            m_line[i].zm = range_data_.zs[valid_mid_pt_idx];


            // if(std::isnan(m_line[i].pm.x) || std::isnan(m_line[i].pm.y) || std::isnan(m_line[i].zm)) {
            //     cout << "invalid middle points at line [" << i << "]" << endl;
            //     cout <<m_line[i].pm.x << ", "
            //         << m_line[i].pm.y << ", "
            //         << m_line[i].zm << endl;
            // }

            m_line[i].left = m;
            m_line[i].right = n;
            m_line[i].laserIdx = laser_idx;

            // cout << "line idx range [m,n]: " << m << ", " << n << endl;
            // if( m > n)
            //     cout << "descending line occured ....." << endl;
            calcVarianceOfFittedLine(m_line[i], m, n);

            if(m_line[i].b!=0)
            {
                endpoint1.x = (range_data_.xs[m]/m_line[i].a + range_data_.ys[m] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
                endpoint1.y = m_line[i].a*endpoint1.x + m_line[i].c;
            }
            else
            {
                endpoint1.x = range_data_.ys[m];
                endpoint1.y = m_line[i].c/m_line[i].a;
            }

            line_temp.x1 = endpoint1.x;
            line_temp.y1 = endpoint1.y;
            line_temp.z1 = range_data_.zs[m];

            m_line[i].p1 = endpoint1;
            m_line[i].z1 = range_data_.zs[m];

            if(m_line[i].b!=0)
            {
                endpoint2.x = (range_data_.xs[n]/m_line[i].a + range_data_.ys[n] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
                endpoint2.y = m_line[i].a*endpoint2.x + m_line[i].c;
            }
            else
            {
                endpoint2.x = range_data_.ys[n];
                endpoint2.y = m_line[i].c/m_line[i].a;
            }

            line_temp.x2 = endpoint2.x;
            line_temp.y2 = endpoint2.y;
            line_temp.z2 = range_data_.zs[n];

            m_line[i].p2 = endpoint2;
            m_line[i].z2 = range_data_.zs[n];

            m_line[i].len = distance_point(endpoint1.x, endpoint1.y, endpoint2.x, endpoint2.y);

            // calculate vertical point, theta, distance.
            double vp_x, vp_y, vp_theta, vp_dist;
            vp_x = -m_line[i].a*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            vp_y = -m_line[i].b*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            vp_theta = std::atan2(vp_y, vp_x);
            vp_dist = std::sqrt(vp_x*vp_x + vp_y*vp_y);

            m_line[i].theta = vp_theta;
            m_line[i].vdist = vp_dist;
            m_line[i].vp.x = vp_x;
            m_line[i].vp.y = vp_y;
            m_line[i].nvp.x = vp_x / vp_dist;
            m_line[i].nvp.y = vp_y / vp_dist;

            output.push_back(line_temp);
        }
        temp_line2 = output;
    }

    void LineFeature::extractLines(std::vector<line>& temp_line1,std::vector<gline3d>& temp_line2, int laser_idx)
    {
        int line_include = 0;
        m_line.clear();
        point_num_ = cs_data_.index;

        if(m_valid_index.size() < params_.min_line_points)
        {
            temp_line1.clear();
            temp_line2.clear();
            return;
        }
        //附近特征点数目

        for(unsigned int i = 0; i < (m_valid_index.size() - params_.min_line_points) ;i++)
        {
            // cout << "[LF 619] -- before least square" << endl; 
            m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
            // cout << "[LF 605] -- least square success " << endl;
            //std::cout<<m_least.a<<" "<<m_least.b<<" "<<m_least.c<<std::endl;
            if(detectline(i,params_.seed_line_points))
            {
                // cout << "[LF 605] -- before region grow " << endl;
                line_include = detectfulline(i);
                // cout << "[LF 605] -- after region grow " << endl;

                i = line_include;
            }

        }
        // cout << "[LF 613] ---before cleanline " << endl;

        cleanline();

        for(int p = 0; p < m_line.size();p++)
        {
            if(!delete_short_line(m_valid_index[m_line[p].left],m_valid_index[m_line[p].right]))
            {
                m_line.erase(m_line.begin()+p);
            }
        }

        // MergeHeadTailLines(m_line);
        
        generate(temp_line2, laser_idx);

        // delete radial lines.
        for(int p = 0; p < m_line.size();p++)
        {
            if(delete_shadow_line(m_line[p]))
            {
                m_line.erase(m_line.begin()+p);
                temp_line2.erase(temp_line2.begin()+p);
            }
        }        

        temp_line1 = m_line;
    }


    void LineFeature::extractLinesNew(std::vector<line>& temp_line1,std::vector<gline3d>& temp_line2, int laser_idx)
    {
        int line_include = 0;
        m_line.clear();
        point_num_ = cs_data_.index;

        if(m_valid_index.size() < params_.seed_line_points)
        {
            temp_line1.clear();
            temp_line2.clear();
            return;
        }
        //附近特征点数目

        for(unsigned int i = 0; i < (m_valid_index.size() - params_.seed_line_points) ;i++)
        {
            // cout << "[LF 619] -- before least square" << endl; 
            m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
            // cout << "[LF 605] -- least square success " << endl;
            // std::cout<<m_least.a<<" "<<m_least.b<<" "<<m_least.c<<std::endl;
            if(detectline(i,params_.seed_line_points))
            {
                // cout << "[LF 605] -- before region grow " << endl;
                line_include = detectfulline(i);
                // cout << "[LF 605] -- after region grow " << endl;

                i = line_include;
            }

        }
        // cout << "[LF 613] ---before cleanline " << endl;

        cleanline();

        for(int p = 0; p < m_line.size();p++)
        {
            if(!delete_short_line(m_valid_index[m_line[p].left],m_valid_index[m_line[p].right]))
            {
                m_line.erase(m_line.begin()+p);
            }
        }

        // MergeHeadTailLines(m_line);
        
        generateNew(temp_line2, laser_idx);

        // delete radial lines.
        for(int p = 0; p < m_line.size();p++)
        {
            if(delete_shadow_line(m_line[p]))
            {
                m_line.erase(m_line.begin()+p);
                temp_line2.erase(temp_line2.begin()+p);
            }
        }        

        temp_line1 = m_line;
    }

    void LineFeature::generateNew(std::vector<gline3d>& temp_line2, int laser_idx)
    {
        gline3d line_temp;
        std::vector<gline3d> output;
        POINT endpoint1;
        POINT endpoint2;
        int m = 0,n = 0;
        double k1 = 0,k2 = 0;
        for(int i = 0;i < m_line.size();i++)
        {   

            m = m_valid_index[m_line[i].left];
            n = m_valid_index[m_line[i].right];

            // find middle of valid points.
            int mid_idx = (m_line[i].left + m_line[i].right)/2;
            int valid_mid_pt_idx = m_valid_index[mid_idx];
            m_line[i].mid = valid_mid_pt_idx;
            // m_line[i].pm.x = range_data_.xs[valid_mid_pt_idx];
            // m_line[i].pm.y = range_data_.ys[valid_mid_pt_idx];

            m_line[i].left = m;
            m_line[i].right = n;
            m_line[i].laserIdx = laser_idx;
            
            // cout << "line idx range [m,n]: " << m << ", " << n << endl;
            // if( m > n)
            //     cout << "descending line occured ....." << endl;
            calcVarianceOfFittedLine(m_line[i], m, n);

            if(m_line[i].b!=0)
            {
                endpoint1.x = (range_data_.xs[m]/m_line[i].a + range_data_.ys[m] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
                endpoint1.y = m_line[i].a*endpoint1.x + m_line[i].c;
            }
            else
            {
                endpoint1.x = range_data_.ys[m];
                endpoint1.y = m_line[i].c/m_line[i].a;
            }

            line_temp.x1 = endpoint1.x;
            line_temp.y1 = endpoint1.y;

            m_line[i].p1 = endpoint1;

            if(m_line[i].b!=0)
            {
                endpoint2.x = (range_data_.xs[n]/m_line[i].a + range_data_.ys[n] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
                endpoint2.y = m_line[i].a*endpoint2.x + m_line[i].c;
            }
            else
            {
                endpoint2.x = range_data_.ys[n];
                endpoint2.y = m_line[i].c/m_line[i].a;
            }

            line_temp.x2 = endpoint2.x;
            line_temp.y2 = endpoint2.y;

            m_line[i].p2 = endpoint2;

            m_line[i].len = distance_point(endpoint1.x, endpoint1.y, endpoint2.x, endpoint2.y);

            // calculate vertical point, theta, distance.
            double vp_x, vp_y, vp_theta, vp_dist;
            vp_x = -m_line[i].a*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            vp_y = -m_line[i].b*m_line[i].c/(m_line[i].a*m_line[i].a + m_line[i].b*m_line[i].b);
            vp_theta = std::atan2(vp_y, vp_x);
            vp_dist = std::sqrt(vp_x*vp_x + vp_y*vp_y);

            m_line[i].theta = vp_theta;
            m_line[i].vdist = vp_dist;
            m_line[i].vp.x = vp_x;
            m_line[i].vp.y = vp_y;
            m_line[i].nvp.x = vp_x / vp_dist;
            m_line[i].nvp.y = vp_y / vp_dist;

            output.push_back(line_temp);
        }
        temp_line2 = output;
    }    


    void LineFeature::calcVarianceOfFittedLine(line& l_param, int start_idx, int stop_idx) {
        double a, b, c;
        a = l_param.a; b = l_param.b; c = l_param.c;
        double sum = 0;
        double cnt = 0;
        for(int k=start_idx; k <= stop_idx; k++ ) {
            if(std::isnan(range_data_.xs[k]) || std::isnan(range_data_.ys[k]))
                continue;

            if(range_data_.ranges[k] <0.001 || range_data_.ranges[k] > 80.0)
                continue;

            double error1 = fabs((a*range_data_.xs[k]+ b*range_data_.ys[k]+c))/sqrt((a*a + b*b));
            sum += error1;
            cnt += 1.0;
        }
        sum /= cnt;
        l_param.var = sum;
    }

}
