/**
 * sensor.cpp
 * @author Ronak Shah, Nicolas Nebel
 * 
 * This is a file intended to be copied for any sensor nodes. 
 * The sensor class (below) is intended to be used as a way to 
 * structure your code (can be kept in a seperate file), and
 * contains methods that are called below in the main function
 * for ros publishing
 */

#include "sensor_template/sensor_template.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_template/mat.h"

//template<std::size_t r1_, std::size_t c1_, std::size_t r2_, std::size_t c2_>
void isensor::start()
{
	meta_pub_.publish(static_cast<std_msgs::Float32MultiArray>(get_sensor_mat()));
	cov_pub_.publish(static_cast<std_msgs::Float32MultiArray>(get_sensor_cov()));
}

//template<std::size_t r_, std::size_t c_>
void isensor::pulse()
{
	data_pub_.publish(static_cast<std_msgs::Float32MultiArray>(get_sensor_data()));
	
	loop_rate_.sleep();
}
