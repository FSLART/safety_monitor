#ifndef MONITOR_H_
#define MONITOR_H_

#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <ctime>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

//Topic names
#define PARAM_TOPIC_LIMG "left_image"
#define PARAM_TOPIC_RIMG "right_image"
#define PARAM_TOPIC_DIMG "depth_image"
#define PARAM_TOPIC_LINFO "left_info"
#define PARAM_TOPIC_RINFO "right_info"
#define PARAM_TOPIC_DINFO "depth_info"
#define PARAM_TOPIC_MAPPER "cone_array_topic"
#define PARAM_TOPIC_PLANNER "path_topic"
#define PARAM_TOPIC_CONTROL "control_topic"
#define PARAM_TOPIC_STATE "state_topic"
#define PARAM_TOPIC_ACU "acu_topic"

//Frequency for each topic
#define PARAM_FREQ_LIMG "limg_freq"
#define PARAM_FREQ_RIMG "rimg_freq"
#define PARAM_FREQ_DIMG "dimg_freq"
#define PARAM_FREQ_LINFO "linf_freq"
#define PARAM_FREQ_RINFO "rinf_freq"
#define PARAM_FREQ_DINFO "dinf_freq"
#define PARAM_FREQ_MAPPER "mapr_freq"
#define PARAM_FREQ_PLANNER "plan_freq"
#define PARAM_FREQ_CONTROL "ctrl_freq"

//Padding for the frequencies
#define PARAM_PADDING "padding"

class SafetyMonitor : public rclcpp::Node
{

public:
    SafetyMonitor();

private:
    //unordered map
    std::unordered_map<std::string, std::chrono::time_point<std::chrono::system_clock>> last_times;

    //Subs for camera
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub;

    //Subs for the mapper|planner|control
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr mapping_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planning_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr control_sub;
    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_sub;

    //Pub to the ACU
    rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr ACU_pub;


    void acu_publisher();
    template <typename T>
    void check_freq_and_log(const typename T::SharedPtr msg, float frequency, const std::string &topic_name);
    void get_state(const lart_msgs::msg::State::SharedPtr msg);


    std::string left_image;
    std::string right_image;
    std::string depth_image;
    std::string left_info;
    std::string right_info;
    std::string depth_info;
    std::string cone_array_topic;
    std::string path_topic;
    std::string control_topic;
    std::string state_topic;
    std::string acu_topic;

    float limg_freq;
    float rimg_freq;
    float dimg_freq;
    float linf_freq;
    float rinf_freq;
    float dinf_freq;
    float mapr_freq;
    float plan_freq;
    float ctrl_freq;

    float padding;
    lart_msgs::msg::State state_msg;

    char time_str[100];

};


#endif