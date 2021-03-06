#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <opencv2/opencv.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <sstream>
#include <iostream>
#include <string>
#include <mutex>

constexpr int LOOP_RATE_HZ = 20;
constexpr int CACHE_SIZ = 10;
constexpr int STATE_VEC_DIM = 12;
constexpr float LOOP_PERIOD = 1. / LOOP_RATE_HZ;

std::mutex data_mutex;

// sensor struct
struct Sensor {
   std::string name;
   std_msgs::Float32MultiArray data;
   std_msgs::Float32MultiArray meta;
   std_msgs::Float32MultiArray noise;
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

// converts ros Float32MultiArray to cv::Mat
inline cv::Mat F32MA2CVMat(const std_msgs::Float32MultiArray& f32ma)
{
    return cv::Mat(f32ma.layout.dim[0].size, 
        f32ma.layout.dim[1].size, CV_32F, const_cast<float*>(f32ma.data.data()));
}

// converts cv mat to f32 ma
std_msgs::Float32MultiArray CVMat2F32MA(cv::Mat& mat)
{
    std::vector<float> tempVec(mat.begin<float>(), mat.end<float>());
    std_msgs::Float32MultiArray fma;
	fma.data.clear();
	fma.data = tempVec;
    fma.layout.dim.resize(2);
    fma.layout.dim[0].label = "rows";
    fma.layout.dim[0].size = mat.rows;
    fma.layout.dim[0].stride = mat.rows * mat.cols;
    fma.layout.dim[1].label = "cols";
    fma.layout.dim[1].size = mat.rows;
    fma.layout.dim[1].stride = mat.cols;
	return fma;
}

// callback for data
void data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, Sensor& sensor, cv::KalmanFilter& main_filter) 
{
    std::lock_guard<std::mutex> lock(data_mutex); // ensures this method is only running once at a time
    
    //sensor.data = msg->data;

    sensor.filter.statePre = main_filter.statePre;
    sensor.filter.correct(F32MA2CVMat(*msg));
    main_filter.statePost = sensor.filter.statePost;
}

// callback for meta: called once to set the sensor matrix
void meta_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, Sensor& sensor, int& metas_set) 
{
    static std::mutex meta_cb_mutex;
    std::lock_guard<std::mutex> lock(meta_cb_mutex);
    // ok this isn't really needed

    sensor.meta = *msg;
    metas_set++;
}

// callback for covariance
void noise_callback(const std_msgs::Float32MultiArray::ConstPtr& msg, Sensor& sensor, int& noises_set) 
{
    static std::mutex noise_cb_mutex;
    std::lock_guard<std::mutex> lock(noise_cb_mutex);

    sensor.noise = *msg;
    noises_set++;
}

int main(int argc, char* argv[])
{
    std::vector<cv::String> sensors;
    cv::String output_topic;
    bool valid_input = parse_args(argc, argv, sensors, output_topic);
    cv::KalmanFilter main_filter(STATE_VEC_DIM,0);
    main_filter.transitionMatrix = (cv::Mat_<float>(STATE_VEC_DIM,STATE_VEC_DIM) <<
	    1,0,0,0,0,0,LOOP_PERIOD,0,0,0,0,0,
	    0,1,0,0,0,0,0,LOOP_PERIOD,0,0,0,0,
	    0,0,1,0,0,0,0,0,LOOP_PERIOD,0,0,0,
	    0,0,0,1,0,0,0,0,0,LOOP_PERIOD,0,0,
	    0,0,0,0,1,0,0,0,0,0,LOOP_PERIOD,0,
	    0,0,0,0,0,1,0,0,0,0,0,LOOP_PERIOD,
	    0,0,0,0,0,0,1,0,0,0,0,0,
	    0,0,0,0,0,0,0,1,0,0,0,0,
	    0,0,0,0,0,0,0,0,1,0,0,0,
	    0,0,0,0,0,0,0,0,0,1,0,0,
	    0,0,0,0,0,0,0,0,0,0,1,0,
	    0,0,0,0,0,0,0,0,0,0,0,1);

    if (!valid_input)
    {
        return 0;
    }

    ros::init(argc, argv, "system_node");
    ros::NodeHandle n;

    std::vector<cv::String> sensor_data_topics(sensors.size());
    std::vector<cv::String> sensor_meta_topics(sensors.size());
    std::vector<cv::String> sensor_noise_topics(sensors.size());


    std::vector<Sensor> sensor_structs(sensors.size());
    int metas_set = 0;
    int noises_set = 0;
    for (int i = 0; i < sensors.size(); i++)
    {
        sensor_data_topics[i] = "sensors/" + sensors[i] + "/data";
        sensor_meta_topics[i] = "sensors/" + sensors[i] + "/meta";
        sensor_noise_topics[i] = "sensors/" + sensors[i] + "/noise";


    	//auto data_callback_bind = std::bind(data_callback, _1, std::ref(sensor_structs[i], std::ref(main_filter));
	    //auto meta_callback_bind = std::bind(meta_callback, _1, std::ref(sensor_structs[i]), std::ref(metas_set));
	    //auto noise_callback_bind = std::bind(noise_callback, _1, std::ref(sensor_structs[i]), std::ref(noises_set));

        boost::function<void(const std_msgs::Float32MultiArray::ConstPtr& msg)> meta_callback_bind = 
            boost::bind(meta_callback, _1, std::ref(sensor_structs[i]), std::ref(metas_set));
        boost::function<void(const std_msgs::Float32MultiArray::ConstPtr& msg)> noise_callback_bind = 
            boost::bind(noise_callback, _1, std::ref(sensor_structs[i]), std::ref(noises_set));

	    sensor_structs[i].name = sensors[i];
	    //sensor_structs[i].data_sub = n.subscribe(sensor_data_topics[i], CACHE_SIZ, data_callback_bind);
        // dont subscribe to data yet
	    sensor_structs[i].meta_sub = n.subscribe(sensor_meta_topics[i], CACHE_SIZ, meta_callback_bind);
	    sensor_structs[i].noise_sub = n.subscribe(sensor_noise_topics[i], CACHE_SIZ, noise_callback_bind);
    }

    // guarantee that only meta and noise callbacks are called
    while(metas_set != sensors.size() && noises_set != sensors.size())
    {
        ros::spinOnce();
    }

    // initialise all kalman filters for each sensor
    for (int i = 0; i < sensors.size(); i++) {
        sensor_structs[i].filter = cv::KalmanFilter(STATE_VEC_DIM,sensor_structs[i].meta.layout.dim[0].size);

        //set noise and sensor mat for filter
        sensor_structs[i].filter.measurementMatrix = F32MA2CVMat(sensor_structs[i].meta);
        sensor_structs[i].filter.measurementNoiseCov = F32MA2CVMat(sensor_structs[i].noise);

        // register subscriber
        boost::function<void(const std_msgs::Float32MultiArray::ConstPtr& msg)> data_callback_bind = 
            boost::bind(data_callback, _1, std::ref(sensor_structs[i]), std::ref(main_filter));
        sensor_structs[i].data_sub = n.subscribe(sensor_data_topics[i], CACHE_SIZ, data_callback_bind);
    }

    

    ros::Publisher system_pub = 
        n.advertise<std_msgs::Float32MultiArray>(output_topic, 1000);
    ros::Rate loop_rate(LOOP_RATE_HZ);

    std_msgs::Float32MultiArray output_state;
    
    std::unique_lock<std::mutex> data_lock(data_mutex);
    data_lock.unlock();

    while (ros::ok())
    {
	    ros::spinOnce(); // calls all data callbacks

        data_lock.lock();
	    main_filter.predict();
	    output_state = CVMat2F32MA(main_filter.statePre); // TODO: make utility function to convert cv::Mat to Float32MultiArray
        data_lock.unlock();

	    system_pub.publish(output_state);
        loop_rate.sleep();     
    }

    std::cout << "success" << std::endl;
}
