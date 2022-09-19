#include "plane_extraction.h"
#include "segcomp_loader.h"
#include "tictoc.h"
#include "save_result.h"

#include <chrono>
#include <thread>
#include "visualizer.h"

#include <pcl/filters/filter.h>



PlaneNormalVisualizer vis;

int main(int argc, char * argv[]) {

    if(argc < 4) {
        std::cerr << "Usage: ./cloudline2plane config_file path_to_data prefix (output_path start_frame stop_frame)" << std::endl;
        return 1;
    }

    std::string config_file = argv[1];
    std::string segcomp_path = argv[2];
    std::string prefix = argv[3];

    boost::thread vis_thread(boost::bind(&PlaneNormalVisualizer::Spin, &vis));


    PlaneExtraction pe;
    pe.loadParams(config_file);

    SaveMultiPlanes* save_results_ptr;
    std::string output_path;
    if(argc == 5) {
        output_path = argv[4];
        ROWS = pe.proj_params().rows();
        COLS = pe.proj_params().cols();
        save_results_ptr = new SaveMultiPlanes(output_path, pe.proj_params());
    }

    int start_frame = 0;
    int stop_frame = 29;
    if(argc > 5) {
        output_path = argv[4];
        start_frame = atoi(argv[5]);
        stop_frame = atoi(argv[6]);
        ROWS = pe.proj_params().rows();
        COLS = pe.proj_params().cols();
        save_results_ptr = new SaveMultiPlanes(output_path, pe.proj_params());   
    }

    int reverse_flag = 0;
    int repeat_num = 0;
    if(argc > 7) {
        reverse_flag = atoi(argv[7]);
        repeat_num = atoi(argv[8]);
    }

    // initialize color table.
    std::vector<cv::Vec3b> colors;
    for(int i=0; i<COLOR_TABLE_SIZE; ++i) {
        colors.push_back(cv::Vec3b(default_colors[i]));
    }
    
    TicToc timer;
    std::vector<double> times_vec;
    int repeat_cnt = 0;
    for(int i=start_frame; i <= stop_frame; ) {
        


        std::string file_path = segcomp_path + prefix + std::to_string(i) + ".xyz";
        float *coords[3];

        loadSegCompPerceptron3(file_path, ROWS, COLS, coords);
        CloudT segcomp_cloud;
        if(argc > 5)
            segcomp_cloud = CloudFromCoordsVLP(coords, ROWS, COLS);
        else 
            segcomp_cloud = CloudFromCoords(coords, ROWS, COLS);

        timer.Tic();
        pe.segment(segcomp_cloud.makeShared(), false);
        double time_eclipsed = timer.Toc();
        std::cout << "segmentation cost: " << time_eclipsed << " ms" << std::endl;
        times_vec.push_back(time_eclipsed);        

        cv::Mat depth(ROWS, COLS, CV_16U);
        for(int r=0; r < ROWS; r++) {
            for(int c=0; c < COLS; c++) {
                uint16_t d = segcomp_cloud.at(c,r).getVector3fMap().norm();
                depth.at<uint16_t>(r,c) = d;
            }
        }



        cv::Mat depth_color;
        depth.convertTo(depth_color, CV_8UC1, 255.0/7.0);
        applyColorMap(depth_color, depth_color, cv::COLORMAP_JET);

        // visualize results.
        cv::Mat plabel_image = pe.labelImage();
        std::vector<PlaneParams> pl_params_vec = pe.planeParamsVec();
        int nPlanesNum = pl_params_vec.size();

        std::cout << "segmented plane num: " << nPlanesNum << std::endl;




        // publish plane label image.
        cv::Mat seg = cv::Mat::zeros(plabel_image.size(), CV_8UC3);
        std::map<int,int> labels_map;
        std::map<int, CloudT> planes_map;
        std::map<int, std::vector<double>> colors_map;
        for(int r=0; r < plabel_image.rows; r++) {
            for(int c=0; c < plabel_image.cols; c++) {
                auto plabel = plabel_image.at<uint16_t>(r,c);
                if(plabel==0)  {
                    // PointT pt_tmp = segcomp_cloud.at(c,r);
                    // if(fabs(pt_tmp.x) < 0.001 && fabs(pt_tmp.y) < 0.001 && fabs(pt_tmp.z) < 0.001)
                    //     continue;
                    
                    // labels_map[plabel]++;
                    // colors_map[plabel] = {0,0,0};
                    // planes_map[plabel].push_back(pt_tmp);
                    continue;
                }
                seg.at<cv::Vec3b>(r, c) = colors[(plabel+2) % colors.size()];
                labels_map[plabel]++;
                // PointT color;
                // color.x = RANDOM_COLORS[plabel % RANDOM_COLORS.size()][0];
                // color.y = RANDOM_COLORS[plabel % RANDOM_COLORS.size()][1];
                // color.z = RANDOM_COLORS[plabel % RANDOM_COLORS.size()][2];  
                // seg.at<cv::Vec3b>(r, c) = cv::Vec3b(color.x, color.y, color.z); 
                std::vector<double> plane_color;
                plane_color.push_back(1.0/255.0*colors[(plabel+2) % colors.size()][2]);
                plane_color.push_back(1.0/255.0*colors[(plabel+2) % colors.size()][1]);
                plane_color.push_back(1.0/255.0*colors[(plabel+2) % colors.size()][0]);
                colors_map[plabel] = plane_color;
                planes_map[plabel].push_back(segcomp_cloud.at(c,r));
            }       
        }


        // 
        if(reverse_flag) {
            cv::flip(depth_color, depth_color, -1);
            cv::flip(plabel_image, plabel_image, -1);
            cv::flip(seg, seg, -1);
        }        

        // save segcomp results.
        if(argc >= 5) {
            save_results_ptr->run(
                plabel_image, 
                seg,
                pl_params_vec,
                i,
                prefix,
                prefix);
        }

        std::cout << "labels :" << std::endl;
        for(auto ele:labels_map) {
            std::cout << ele.first <<": " << ele.second << std::endl;
        }

        // cv::imshow("depth",depth_color);        
        // cv::imshow("seg", seg);

        cv::Mat canvas;
        cv::vconcat(depth_color, seg, canvas);
        cv::imshow("canvas", canvas);
        // cv::waitKey(10);        
        cv::waitKey();  

        cv::Mat vline_img_color = cv::Mat::zeros(plabel_image.size(), CV_8UC3);
        cv::Mat hline_img_color = cv::Mat::zeros(plabel_image.size(), CV_8UC3);
        auto hsweep_lines = pe.getHSweepLines();
        auto vsweep_lines = pe.getVSweepLines();
        for(int li=0; li < hsweep_lines.size(); li++) {
            std::vector<line> line_segs = hsweep_lines[li]; 	
            for(int lj=0; lj < line_segs.size(); lj++) {
                int start_idx = line_segs[lj].left;
                int stop_idx = line_segs[lj].right;
                // pcl::PointXYZ color;
                // color.x = RANDOM_COLORS[j % 200][0];
                // color.y = RANDOM_COLORS[j % 200][1];
                // color.z = RANDOM_COLORS[j % 200][2];       		
                for(int lk=start_idx; lk<=stop_idx; lk++) {
                    hline_img_color.at<cv::Vec3b>(li, lk) = colors[(lj+15) % colors.size()];
                }
            }    	
        }
        for(int li=0; li < vsweep_lines.size(); li++ ) {
            std::vector<line> line_segs = vsweep_lines[li];
            for(int lj=0; lj < line_segs.size(); lj++) {
                // pcl::PointXYZ color;
                // color.x = RANDOM_COLORS[j % 200][0];
                // color.y = RANDOM_COLORS[j % 200][1];
                // color.z = RANDOM_COLORS[j % 200][2]; 
                int start_idx = line_segs[lj].left;
                int stop_idx = line_segs[lj].right;
                for(int lk=start_idx; lk<=stop_idx; lk++) {
                    vline_img_color.at<cv::Vec3b>(lk, li) = colors[(lj+15) % colors.size()];
                }
            }
        }

        cv::Mat line_canvas;
        cv::vconcat(vline_img_color, hline_img_color, line_canvas);
        // cv::vconcat(vline_img_color, depth_color, line_canvas);
        // cv::vconcat(line_canvas, hline_img_color, line_canvas);

        cv::imshow("line_canvas", line_canvas);        
        cv::waitKey(10);        
        // cv::imwrite("./line_mask_images.png", line_canvas);

        if( vis.init) {
            // ----------------------------------------------------------------------------
            // visualize plane cloud.

            // std::vector<CloudT::Ptr> planes_clouds;
            // std::vector<std::string> planes_ids;
            // std::vector<std::vector<double>> planes_colors;
            // int plane_cnt = 0;
            // for(auto m_ele:planes_map) {
            //     int plane_label = m_ele.first;
            //     std::string plane_id = "plane_" + std::to_string(plane_cnt++);
            //     std::vector<double> plane_color = colors_map[plane_label];
            //     planes_clouds.push_back(m_ele.second.makeShared());
            //     planes_ids.push_back(plane_id);
            //     planes_colors.push_back(plane_color);
            // }
            
            // vis.UpdateCloud(planes_clouds, planes_ids, planes_colors);

            // ----------------------------------------------------------------------------
            // visualize raw cloud.

            CloudT::Ptr frame(new CloudT);
            std::vector<int> ind;
            pcl::removeNaNFromPointCloud(segcomp_cloud, *frame, ind);            
            vis.UpdateCloud(frame, "cloud", {0.0, 0.0, 0.0});

            // ----------------------------------------------------------------------------
            // visualize line segments.
            // std::vector<std::vector<gline3d>> vsweep_glines = pe.getVSweepGLines();
            // std::vector<std::vector<gline3d>> hsweep_glines = pe.getHSweepGLines();
            // CloudT::Ptr cloud_edpts1(new CloudT);
            // CloudT::Ptr cloud_edpts2(new CloudT);
            // for(int l_i=0; l_i < vsweep_glines.size(); l_i+=3 ) {
            //     for(int l_j=0; l_j < vsweep_glines[l_i].size(); l_j ++ ) {
            //         PointT pt;
            //         pt.x = vsweep_glines[l_i][l_j].x1;
            //         pt.y = vsweep_glines[l_i][l_j].y1;
            //         pt.z = vsweep_glines[l_i][l_j].z1;
            //         cloud_edpts1->points.push_back(pt);
            //         pt.x = vsweep_glines[l_i][l_j].x2;
            //         pt.y = vsweep_glines[l_i][l_j].y2;
            //         pt.z = vsweep_glines[l_i][l_j].z2;
            //         cloud_edpts2->points.push_back(pt);
            //     }
            // }
            // vis.UpdateLines(cloud_edpts1, cloud_edpts2, {1.0, 0.0, 0.0}, "vline", true);
            // cloud_edpts1->clear();
            // cloud_edpts2->clear();
            // for(int l_i=0; l_i < hsweep_glines.size(); l_i+=1 ) {
            //     for(int l_j=0; l_j < hsweep_glines[l_i].size(); l_j ++ ) {
            //         PointT pt;
            //         pt.x = hsweep_glines[l_i][l_j].x1;
            //         pt.y = hsweep_glines[l_i][l_j].y1;
            //         pt.z = hsweep_glines[l_i][l_j].z1;
            //         cloud_edpts1->points.push_back(pt);
            //         pt.x = hsweep_glines[l_i][l_j].x2;
            //         pt.y = hsweep_glines[l_i][l_j].y2;
            //         pt.z = hsweep_glines[l_i][l_j].z2;
            //         cloud_edpts2->points.push_back(pt);
            //     }
            // }            
            // vis.UpdateLines(cloud_edpts1, cloud_edpts2, {0.0, 0.0, 1.0}, "hline", false);

            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }   


        if(repeat_cnt < repeat_num) {
            repeat_cnt ++;
        } else {
            repeat_cnt = 0;
            i++;            
        }                   
    }

    std::cout << output_path << std::endl;

    std::string pathTimesFile = output_path + "/timings/times.txt";
    saveTimesToFile(pathTimesFile, times_vec);

    vis_thread.join();

    return 0;
}