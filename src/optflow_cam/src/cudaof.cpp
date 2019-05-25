#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <chrono>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

// lazy global
int itr = 0;

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

// lil helper function to store time for certain actions
template<typename Func>
void timeAndStoreAction(std::map<std::string, long>& map,
                std::string&& actionName, const Func& f )
{
   //if(itr <= 5) return;

   auto start = std::chrono::high_resolution_clock::now();
   f();
   auto stop = std::chrono::high_resolution_clock::now();
   auto duration =
       std::chrono::duration_cast<std::chrono::microseconds>( stop - start );
   if ( map.find( actionName ) == map.end() )
       map[actionName] = duration.count();
   else
       map[actionName] += duration.count();
}

float angularDelta( cv::Point2f p1, cv::Point2f p2, cv::Point2f center )
{
    cv::Point2f d = p2 - p1;
    float s = (d.x + d.y);
    cv::Point2f add = p1 + p2;
    cv::Point2f avg = { add.x / 2, add.y / 2 };
    float r = cv::norm( avg - center );
    return (s/r) * (180/3.1415927);
}

template<typename K, typename V>
std::string avgTimesStr( const std::map<K, V>& map, int count )
{
    std::stringstream ss;
    V avgTotalTime = 0;
    for( const auto& pair: map )
    {
        V avgTime = pair.second / count;
        ss << pair.first << ": " << avgTime << "us" << std::endl;
        avgTotalTime += avgTime;
    }
    ss << "\e[1mTotal time:\e[0m " << avgTotalTime << std::endl;
    return ss.str();
}

int main(int argc, const char* argv[])
{
    std::cout << "initializing" << std::endl;

    std::string winName = "uberr-advanced-algos";
    std::map<std::string, long> timingTotals;
    int count = 0;

    // open camera and check if opened
    cv::VideoCapture cap( 0 );
    if( !cap.isOpened() ) return -1;

    cv::namedWindow( winName );
    cv::Mat frame;
    cv::Mat gray;

    // read from camera
    cap >> frame;
    cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY );
    cv::Point2f center = { static_cast<float>( frame.cols ) / 2,
                           static_cast<float>( frame.rows ) / 2 };

    // allocate space for new image and data points on gpu
    cv::cuda::Stream opStream;
    cv::cuda::GpuMat gpuGray( gray.rows, gray.cols, gray.type() );
    cv::cuda::GpuMat gpuOldGray( gray );
    cv::cuda::GpuMat gpuOldTrackPts;
    cv::cuda::GpuMat gpuNewTrackPts;
    cv::cuda::GpuMat gpuStatus;

    // vectors to store points off device
    std::vector<cv::Point2f> oldTrackPts;
    std::vector<cv::Point2f> newTrackPts;
    std::vector<uchar> status;

    // init detector and opt flow
    cv::Ptr<cv::cuda::CornersDetector> gpuDetector =
        cv::cuda::createGoodFeaturesToTrackDetector( gray.type() );
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> gpuPyrLK =
        cv::cuda::SparsePyrLKOpticalFlow::create();

    while( true )
    {
        timeAndStoreAction( timingTotals, "cap_img", [&]() {
           cap >> frame;
        } );

        // grab new frame and upload to gpu
        timeAndStoreAction( timingTotals, "cvt_grayscale", [&]() {
            cv::cvtColor( frame, gray, cv::COLOR_BGR2GRAY );
        } );

        timeAndStoreAction( timingTotals, "gray_img_upl", [&]() {
            gpuGray.upload( gray );
        } );

        // find pts to track
        timeAndStoreAction( timingTotals, "find_track_pts", [&]() {
            gpuDetector->detect( gpuOldGray, gpuOldTrackPts );
        } );

        // use opt flow algos & stuff to find next pts
        timeAndStoreAction( timingTotals, "pyr_LK_opt_flow", [&]() {
            gpuPyrLK->calc( gpuOldGray, gpuGray, gpuOldTrackPts,
                gpuNewTrackPts, gpuStatus );
        } );

        // download point sets from gpu
        timeAndStoreAction( timingTotals, "trackPts_and_gpuStatus_dl", [&]() {
            download( gpuOldTrackPts, oldTrackPts );
            download( gpuNewTrackPts, newTrackPts );
            download( gpuStatus, status );
        } );

        // swap image and the usual grind
        timeAndStoreAction( timingTotals, "swap_gpu_imgs", [&]() {
            gpuGray.swap( gpuOldGray ); // shouldn't take too long,
                                        // i think the pointers are
                                        // just swapped
        } );

        count++;

        // calculate avg deltas
        cv::Point2f avgDelta;
        float avgAngDelta;
        timeAndStoreAction( timingTotals, "calc_avg_deltas", [&]() {
            int foundCount = 0;
            cv::Point2f totalDelta = { 0, 0 };
            float totalAngDelta = 0;

            for( auto i = 0; i < newTrackPts.size(); i++ )
            {
                if( status[i] )
                {
                    foundCount++;
                    totalDelta += newTrackPts[i] - oldTrackPts[i];
                    totalAngDelta += angularDelta( oldTrackPts[i],
                                                   newTrackPts[i],
                                                   center );
                }
            }
            avgDelta =
                { totalDelta.x / foundCount, totalDelta.y / foundCount };
            avgAngDelta = totalAngDelta / foundCount;
        } );

        std::cout << "---------- Avg Deltas ----------" << std::endl;
        std::cout << "dist traveled: (" << avgDelta.x << ", " <<
            avgDelta.y << ")" << std::endl;
        std::cout << "angular total delta: " << avgAngDelta << std::endl;
        std::cout << "---------- Avg Times ----------" << std::endl;
        std::cout << avgTimesStr( timingTotals, count ) << std::endl;

        cv::imshow( winName, frame );
        itr++;
        //if( cv::waitKey( 30 ) > 0 ) break;
    }
}
