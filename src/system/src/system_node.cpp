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

int main(int argc, char* argv[])
{
    std::vector<cv::String> sensors;
    cv::String output_topic;
    bool valid_input = parse_args(argc, argv, sensors, output_topic);

    if (!valid_input)
    {
        return 0;
    }

    std::vector<cv::String> sensor_data_topics(sensors.size());
    std::vector<cv::String> sensor_meta_topics(sensors.size());
    std::vector<cv::String> sensor_noise_topics(sensors.size());
    for (int i = 0; i < sensors.size(); i++)
    {
        sensor_data_topics[i] = "sensors/" + sensors[i] + "/data";
        sensor_meta_topics[i] = "sensors/" + sensors[i] + "/meta";
        sensor_noise_topics[i] = "sensors/" + sensors[i] + "/noise";
    }

    ros::init(argc, argv, "system_node");
    ros::NodeHandle n;

    ros::Publisher system_pub = 
        n.advertise<std_msgs::Float32MultiArray>(output_topic, 1000);
    ros::Rate loop_rate(LOOP_RATE_HZ);

    std_msgs::Float32MultiArray output_state;

    while (ros::ok())
    {
        
    }

    std::cout << "success" << std::endl;
}
