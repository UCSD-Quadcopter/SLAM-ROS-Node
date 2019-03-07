#pragma once

#include <vector>
#include <exception>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include <sensor_template/mat.h>

// helper methods to download mats from gpu into vectors -
// copied from opencv sample code here:
// https://github.com/opencv/opencv/blob/master/samples/gpu/pyrlk_optical_flow.cpp
static void download(const cv::cuda::GpuMat& d_mat,
                     std::vector<cv::Point2f>& vec)
{
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
}

static void download(const cv::cuda::GpuMat& d_mat,
                     std::vector<uchar>& vec)
{
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
}

float angular_delta( cv::Point2f p1, cv::Point2f p2, cv::Point2f center )
{
    cv::Point2f d = p2 - p1;
    float s = (d.x + d.y);
    cv::Point2f add = p1 + p2;
    cv::Point2f avg = { add.x / 2, add.y / 2 };
    float r = cv::norm( avg - center );
    return (s/r) * (180/3.1415927);
}

class optflow_cam
{
public:
    optflow_cam(int cam_num)
    {
#ifdef DEBUG
        std::cout << "optflow_cam initializing" << std::endl;
        win_name_ = "uberr-advanced-algos";
        cv::namedWindow(win_name_);
#endif // DEBUG

        // open camera and check if opened
        cap_ = cv::VideoCapture(cam_num);
        if (!cap_.isOpened()) throw std::runtime_error("could not open cam");

        // read from camera
        cap_ >> frame_;
        cv::cvtColor(frame_, gray_, cv::COLOR_BGR2GRAY);
        center_ = {static_cast<float>(frame_.cols) / 2,
                   static_cast<float>(frame_.rows) / 2};

        // allocate space for new image and data points on gpu
        //cv::cuda::Stream opStream;
        gpu_gray_ = cv::cuda::GpuMat(gray_.rows, gray_.cols, gray_.type());
        gpu_old_gray_ = cv::cuda::GpuMat(gray_);

        // init detector and opt flow
        gpu_detector_ = cv::cuda::createGoodFeaturesToTrackDetector(gray_.type());
        gpu_pyr_LK_ = cv::cuda::SparsePyrLKOpticalFlow::create();
    }

    matf<3,1> calc_deltas()
    {
        cap_ >> frame_;
        cv::cvtColor(frame_, gray_, cv::COLOR_BGR2GRAY);
        gpu_gray_.upload(gray_);

        gpu_detector_->detect(gpu_old_gray_, gpu_old_track_pts_);
        gpu_pyr_LK_->calc( gpu_old_gray_, gpu_gray_, gpu_old_track_pts_,
            gpu_new_track_pts_, gpu_status_);

        download(gpu_old_track_pts_, old_track_pts_);
        download(gpu_new_track_pts_, new_track_pts_);
        download(gpu_status_, status_);

        gpu_gray_.swap(gpu_old_gray_);

        // average deltas
        int found_count = 0;
        cv::Point2f total_trans_delta = {0.0f, 0.0f};
        float total_ang_delta = 0;
        for( auto i = 0; i < new_track_pts_.size(); i++ )
        {
            if( status_[i] )
            {
                found_count++;
                total_trans_delta += (new_track_pts_[i] - old_track_pts_[i]);
                total_ang_delta += angular_delta(old_track_pts_[i],
                    new_track_pts_[i],
                    center_);
            }
        }

        float deltas[] = {total_trans_delta.x / found_count, 
            total_trans_delta.y / found_count, 
            total_ang_delta / found_count};
        return deltas;
    }



private:

#ifdef DEBUG
    std::string win_name_;
#endif // DEBUG
    cv::VideoCapture cap_;
    cv::Mat frame_;
    cv::Mat gray_;
    cv::Point2f center_;

    cv::cuda::GpuMat gpu_gray_;
    cv::cuda::GpuMat gpu_old_gray_;
    cv::cuda::GpuMat gpu_old_track_pts_;
    cv::cuda::GpuMat gpu_new_track_pts_;
    cv::cuda::GpuMat gpu_status_;

    std::vector<cv::Point2f> old_track_pts_;
    std::vector<cv::Point2f> new_track_pts_;
    std::vector<uchar> status_;

    cv::Ptr<cv::cuda::CornersDetector> gpu_detector_;
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> gpu_pyr_LK_;

};