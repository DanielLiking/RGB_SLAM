#include "ProcessingClass.h"
#include <thread>


void ProcessingClass::init(std::string& file_path){
    rgb_param.loadParam(file_path);
    common_param.loadParam(file_path);
    num_of_plane=0;
    num_of_line=0;
    gap_line = rgb_param.gap_line;
    gap_plane = rgb_param.gap_plane; 
}

void ProcessingClass::lineFilter(cv::Mat& color_im, cv::Mat_<cv::Vec3f>& cloud_peac, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& pc_out_line)
{
    int length_threshold = 30;
    int distance_threshold = 2;
    int canny_th1 = 50;
    int canny_th2 = 50;
    int canny_aperture_size = 3;
    bool do_merge = ture;

    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
                                                                                       distance_threshold, canny_th1, canny_th2, canny_aperture_size,
                                                                                       do_merge);
    cv::Mat image_gray(color_im.rows, color_im.cols, CV_8U);
    cvtColor(color_im, image_gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Vec4f> lines_fld;
    fld->detect(image_gray, lines_fld);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_all_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr line_info_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    int line_cnt = 0;
    for(auto& l:lines_fld)
    {
        cv::LineIterator lit(color_im, cv::Point(l[0],l[1]), cv::Point(l[2],l[3]))
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        cloud->reserve(100);
        int gap_tmp_line = gap_line;
        if(lit.count< gap_tmp_line * 5)
            gap_tmp_line = gap_tmp_line / 2;
        for(int j=0; j<lit.count; ++j,++lit)
        {
            if(j%gap_tmp_line==0)
            {
                int col = lit.pos().x;
                int row = lit.pos().y;
                pcl::PointXYZRGBL pt;
                pt.x = cloud_peac
            }
        }
    }
}