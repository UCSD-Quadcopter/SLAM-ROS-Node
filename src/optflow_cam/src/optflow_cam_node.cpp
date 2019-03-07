#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sensor_template/mat.h>
#include <sensor_template/sensor_template.h>

#include "optflow_cam.h"

/*
 * sensor mat format:
 * [[dx    /dt]
 *  [dy    /dt]
 *  [dtheta/dt]]
 */

class optflow_cam_sensor : public isensor
{
public: 
    optflow_cam_sensor(std::string name) : isensor(name), ofcam_(0) {}

    void sensor_init() 
    {
        last_tick_ = std::chrono::high_resolution_clock::now();
    }

    matf<3,12> get_sensor_mat() const
    {
        // 6x12 arr
        float data[] = {0,0,0,0,0,0,1,0,0,0,0,0,
                        0,0,0,0,0,0,0,1,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,1,0,0};
        return data;
    }

    matf<3,3> get_sensor_cov() const 
    {
        float data[] = {1,0,0,
                        0,1,0,
                        0,0,1}; // I multiplied by some constant
        return data;
    }

    matf<3,1> get_sensor_data() 
    {
        auto deltas = ofcam_.calc_deltas();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> duration = end - last_tick_;
        last_tick_ = std::chrono::high_resolution_clock::now();
        float data[] = {deltas.data()[0] * duration.count(), 
            deltas.data()[1] * duration.count(), 
            deltas.data()[2] * duration.count()};

        return data;
    }

private:
    optflow_cam ofcam_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_tick_;
};

SENSOR_NODE_MAIN(optflow_cam_sensor, "optflow_cam")