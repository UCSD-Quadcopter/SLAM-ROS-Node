// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.  
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <math.h>

// define a test constant for meters per pixel (want to get this somehow)
#define POW_2(x) ((x) * (x))
#define CAM_MAT {1736.04, 0.0, 356.59, 0.0, 1752.22, 291.99, 0.0, 0.0, 1.0}

void get_3d_pt(const rs2_intrinsics & intr, const rs2::depth_frame & frame, float in_pt[2], float out_pt[3]) {
    auto dist = frame.get_distance(in_pt[0], in_pt[1]);
    rs2_deproject_pixel_to_point(out_pt, &intr, in_pt, dist);
}

using namespace cv;
void get_avoidance_vectors(const rs2::depth_frame &frame, Mat *output, float net_avoidance_vec[3]) {

    net_avoidance_vec[0] = net_avoidance_vec[1] = net_avoidance_vec[2] = 0;
    int rows = frame.get_height();
    int cols = frame.get_width();

    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    

    for (int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            float pixel[2];
            pixel[0] = j;
            pixel[1] = i;

            float avoidance_vec[3] = { 0.0, 0.0, 0.0 };

            get_3d_pt(intr, frame, pixel, avoidance_vec);

            float norm_squared = POW_2(avoidance_vec[0]) + POW_2(avoidance_vec[1]) + POW_2(avoidance_vec[2]);
            if (norm_squared == 0) continue;
            for(int a = 0; a < 3; a++) {
                avoidance_vec[a] = -avoidance_vec[a]/norm_squared;

                net_avoidance_vec[a] += avoidance_vec[a];
            }

            output->at<Vec3f>(i,j)[0] = avoidance_vec[0];
            output->at<Vec3f>(i,j)[1] = avoidance_vec[1];
            output->at<Vec3f>(i,j)[2] = avoidance_vec[2];
        }
    }
    for(int a = 0; a < 3; a++) {
        net_avoidance_vec[a] = net_avoidance_vec[a]/(rows*cols);
    }
}

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start();

    //auto sensor = profile.get_device().first<rs2::depth_sensor>();
    //auto scale =  sensor.get_depth_scale(); 

    const auto window_name = "Display Image";

    float camMat[9] = CAM_MAT;
    Mat calibMat(3,3,CV_32F,camMat);
    Mat outputDisplay;

    while (waitKey(1) < 0) 
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame();

        int h = depth.as<rs2::video_frame>().get_height();
        int w = depth.as<rs2::video_frame>().get_width();
        
        float net_avoidance_vec[3] = {0, 0, 0};
        
        // TODO: maybe some post processing for the depth frame?
        
        Mat output = Mat::zeros(w,h,CV_32FC3);

        get_avoidance_vectors(depth, &output, net_avoidance_vec);

        std::cout << "\n( " << net_avoidance_vec[0] << " , " << net_avoidance_vec[1] << " , " << net_avoidance_vec[2] << " )" << std::endl;
        std::cout << "Norm: " << sqrt(POW_2(net_avoidance_vec[0]) + POW_2(net_avoidance_vec[1]) + POW_2(net_avoidance_vec[2])) << std::endl;
        
        //output.convertTo(output, CV_8UC3);

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

