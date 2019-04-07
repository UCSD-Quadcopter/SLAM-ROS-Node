#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <opencv2/opencv.hpp>

#include <sstream>
#include <iostream>
#include <string>

#define LOOP_RATE_HZ 20
#define CACHE_SIZ 10
#define STATE_VEC_DIM 12

// sensor struct
struct Sensor {
   std::string name;
   Float32MultiArray data;
   Float32MultiArray meta;
   Float32MultiArray noise;
   ros::Subscriber data_sub;
   ros::Subscriber meta_sub;
   ros::Subscriber noise_sub;
   cv::KalmanFilter filter;
};

// modified from 
// https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/
void split(const std::string& s, char delimiter, 
        std::vector<cv::String>& tokens)
{
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
}

// fills sensors and output with given arguments
bool parse_args(int argc, char* argv[], 
        std::vector<cv::String>& sensors, cv::String& output_topic)
{
    const cv::String keys = 
        "{help h usage ? |      | print this message   }"
        "{sensors        |      | sensors to read from }"
        "{output         |      | topic to output to   }";
    
    cv::CommandLineParser parser(argc, argv, keys);

    bool show_help = parser.has("help") | !parser.has("sensors") | 
        !parser.has("output");

    if (show_help)
    {
        parser.printMessage();
        return false;
    }

    std::string sensor_args = parser.get<cv::String>("sensors");
    output_topic = parser.get<cv::String>("output");

    if (!parser.check())
    {
        parser.printErrors();
        return false;
    }

    split(sensor_args, ',', sensors);

    return true;
}

// callback for data
void data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, Sensor & sensor) {
    sensor.data = msg->data;

    // call the kalman filter update function
}

// callback for meta
void meta_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, Sensor & sensor) {
    meta = msg->data;

    sensor.filter = cv::KalmanFilter(STATE_VEC_DIM,msg->layout.dim[0].size);
}

// callback for covariance
void noise_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, Float32MultiArray & noise) {
    noise = msg->data;
}

int main(int argc, char* argv[])
{
    std::vector<cv::String> sensors;
    cv::String output_topic;
    bool valid_input = parse_args(argc, argv, sensors, output_topic);

    if (!valid_input)
    {
        return 0;
    }

    ros::init(argc, argv, "system_node");
    ros::NodeHandle n;

    std::vector<cv::String> sensor_data_topics(sensors.size());
    std::vector<cv::String> sensor_meta_topics(sensors.size());
    std::vector<cv::String> sensor_noise_topics(sensors.size());

    std::vector<Sensor> sensor_structs;
    for (int i = 0; i < sensors.size(); i++)
    {
        sensor_data_topics[i] = "sensors/" + sensors[i] + "/data";
        sensor_meta_topics[i] = "sensors/" + sensors[i] + "/meta";
        sensor_noise_topics[i] = "sensors/" + sensors[i] + "/noise";


    	auto data_callback_bind = std::bind(data_callback,_1,sensor_structs[i]);
	auto meta_callback_bind = std::bind(meta_callback,_1,sensor_structs[i]);
	auto noise_callback_bind = std::bind(noise_callback,_1,sensor_structs[i].noise);

	sensor_structs[i].name = sensors[i];
	sensor_structs[i].data_sub = n.subscribe(sensor_data_topics[i], CACHE_SIZ, data_callback_bind);
	sensor_structs[i].meta_sub = n.subscribe(sensor_meta_topics[i], CACHE_SIZ, meta_callback_bind);
	sensor_structs[i].noise_sub = n.subscribe(sensor_noise_topics[i], CACHE_SIZ, noise_callback_bind);
    }

    

    ros::Publisher system_pub = 
        n.advertise<std_msgs::Float32MultiArray>(output_topic, 1000);
    ros::Rate loop_rate(LOOP_RATE_HZ);

    std_msgs::Float32MultiArray output_state;

    while (ros::ok())
    {
        
    }

    std::cout << "success" << std::endl;
}
