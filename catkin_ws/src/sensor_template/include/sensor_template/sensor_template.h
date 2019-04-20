#pragma once
/**
 * sensor.cpp
 * @author Ronak Shah, Nicholas Nebel
 * 
 * This is a file intended to be copied for any sensor nodes. 
 * The sensor class (below) is intended to be used as a way to 
 * structure your code (can be kept in a seperate file), and
 * contains methods that are called below in the main function
 * for ros publishing
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "mat.h"

/**
 * Required Main Function for ROS Nodes to Work
 * 
 * See: h	
 */


 #define SENSOR_NODE_MAIN(Sensor, name) \
int main(int argc, char *argv[]) \
{ \
	Sensor sensor(name); \
    ros::init(argc, argv, name); \
	sensor.start(); \
    ros::spinOnce(); \
    \
    while(ros::ok()) \
    { \
        sensor.pulse(); \
        ros::spinOnce(); \
    } \
    return 0; \
}

/**
 * Sensor Class
 */
class isensor
{
public:
	/**
	 * Ctor
	 * 
	 * @param name The name of the sensor (used in ros init)
	 * @param topic The name of the topic to publish to (used in ros publish)
	 */
	isensor(std::string name) : 
        name_(name), 
        meta_pub_(nh_.advertise<std_msgs::Float32MultiArray>("sensors/" + name + "/meta", 1000)),
        cov_pub_(nh_.advertise<std_msgs::Float32MultiArray>("sensors/" + name + "/cov", 1000)),
        data_pub_(nh_.advertise<std_msgs::Float32MultiArray>("sensors/" + name + "/data", 1000)),
        loop_rate_(10) {}
	

	// MARK: interface funcions

	/**
	 * Start Initializion Function
	 * 
	 * This method will be called after ros::init, it should start
	 * connecting to sensors / getting initial data, etc
	 */
    //template<std::size_t r1_=0, std::size_t c1_=0, std::size_t r2_=0, std::size_t c2_=0>
	void start();

	/**
	 * Called every ros cycle.
	 */
    //template<std::size_t r_, std::size_t c_>
	void pulse();


    //suppress bullshit return type warnings
    #pragma GCC diagnostic ignored "-Wreturn-type"

	void sensor_init() {}

	/**
	 * implemented by subclass to return sensor-specific matrix
	 * which converts sensor reading data to object state
	 */
	template<std::size_t r_=0, std::size_t c_=0>
	matf<r_,c_> get_sensor_mat() const {}

    /**
	 * implemented by subclass to return sensor-specific covariance matrix
	 */
    template<std::size_t r_=0, std::size_t c_=0>
	matf<r_,c_> get_sensor_cov() const {}

	/**
	 * implemented by subclass to get sensor data from arduino, etc
	 */
	template<std::size_t r_=0, std::size_t c_=0>
	matf<r_,c_> get_sensor_data() const {}

private:
    // MARK: Variables
	/**
	 * The Name of the sensor node
	 */
	std::string name_;

    ros::NodeHandle nh_;

    ros::Publisher meta_pub_;
    ros::Publisher cov_pub_;
    ros::Publisher data_pub_;

    ros::Rate loop_rate_;
	
};

//SENSOR_NODE_MAIN(isensor, "test");