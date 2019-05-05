// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <math.h>

// define a test constant for meters per pixel (want to get this somehow)
#define POW_2(x) x * x

// depth and output must have the same dimensions, output must be 3 dimensional
using namespace cv;
void get_avoidance_vectors(cv::Mat *depth, cv::Mat *output, cv::Mat *calibMat){
    int nRows = depth->rows;
    int nCols = depth->cols;

    int i,j;

    Mat calibInv = calibMat->inv();
    float * rowPtr;
    for(i = 0; i < nRows; i++) {
        rowPtr = depth->ptr<float>(i);
        for(j = 0; j < nCols; j++) {

            float depth = rowPtr[j]; 
            // handle case where depth is not available (place blank vector at current index)
            if (depth == 0) {
                output->at<Vec3f>(i,j)[0] = 0;
                output->at<Vec3f>(i,j)[1] = 0;
                output->at<Vec3f>(i,j)[2] = 0;
                continue;
            }
                
            // i is the pixel index in the y direction and j is the pixel index in the x direction
            // need calibration matrix here
            
            cv::Vec3f imgPointVec(j,i,1.);
            cv::Mat worldPointVec = (calibInv * Mat(imgPointVec)) * depth;
            float normSquared = POW_2(worldPointVec.ptr<float>(0)[0]) + POW_2(worldPointVec.ptr<float>(1)[0]) + POW_2(worldPointVec.ptr<float>(2)[0]);
            cv::Mat avoidanceVec = worldPointVec / -normSquared;

            output->at<Vec3f>(i,j)[0] = avoidanceVec.ptr<float>(0)[0];
            output->at<Vec3f>(i,j)[1] = avoidanceVec.ptr<float>(1)[0];
            output->at<Vec3f>(i,j)[2] = avoidanceVec.ptr<float>(2)[0];
        }
    }
}

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    const auto window_name = "Display Image";

    Mat calibMat;
    Mat outputDisplay;

    while (waitKey(1) < 0) 
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame();

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_32F , (void*)depth.get_data(), Mat::AUTO_STEP);
        
        Mat output = Mat::zeros(w,h,CV_32FC3);

        get_avoidance_vectors(&image, &output, &calibMat);
        
        output.convertTo(outputDisplay, CV_8UC3);

        imshow("Avoidance vectors",outputDisplay);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

