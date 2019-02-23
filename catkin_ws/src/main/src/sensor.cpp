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
#include "std_msgs/String.h"
#include <std_msgs/Float32MultiArray.h"
#include <sstream>
#include "mat.h"

/**
 * Required Main Function for ROS Nodes to Work
 * 
 * See: https://answers.ros.org/question/188909/how-to-structure-a-node-to-publish-a-topic-using-classes/
 */
int main(int argc, char **argv)
{
	isensor sensor("sensor_name", "topic_name");
    ros::init(argc, argv, sensor->name);
	sensor->start();

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std_msgs::Float32MultiArray msg = sensor->pulse();
		chatter_pub.publish(msg);
		loop_rate.sleep();

        ros::spinOnce();
    }
    return 0;
}

/**
 * Sensor Class
 */
class isensor
{
public:
	/**
	 * Ctor: Implemented by default; added for explicitness
	 * 
	 * @param name The name of the sensor (used in ros init)
	 * @param topic The name of the topic to publish to (used in ros publish)
	 */
	isensor(std::string name, std::string topic) : name_(name), topic_(topic) {}

	// MARK: Variables
	/**
	 * The Name of the sensor node
	 */
	std::string name_;
	/**
	 * The Topic the sensor should be publishing to
	 */
	std::string topic_;
	

	// MARK: interface funcions

	/**
	 * Start Initializion Function
	 * 
	 * This method will be called after ros::init, it should start
	 * connecting to sensors / getting initial data, etc
	 */
	void start() {}

	/**
	 * Called every ros cycle, should return whatever needs to be published
	 * in the form of a Float32MultiArray. (See mat.h for a way to convert
	 * a mat to a Float32MultiArray)
	 */
	std_msgs::Float32MultiArray pulse() {}

private:
	/**
	 * implemented by subclass to return sensor-specific matrix
	 * which converts sensor reading data to object state
	 */
	template<std::size_t r_, std::size_t c_>
	virtual matf<r_, c_> get_sensor_mat() {}

	/**
	 * implemented by subclass to get sensor data from arduino, etc
	 */
	template<std::size_t r_, std::size_t c_>
	virtual matf<r_, c_> get_sensor_data() {}
};