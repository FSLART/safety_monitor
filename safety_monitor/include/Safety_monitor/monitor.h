#ifndef MONITOR_H_
#define MONITOR_H_


#include <ctime>
#include <memory>
#include <string>
#include <fstream>
#include <functional>
#include <unordered_map>

#include "timeFreq.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "lart_msgs/msg/state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


//Topic names for the state_controller and ACU
#define PARAM_TOPIC_STATE "state_topic"
#define PARAM_TOPIC_ACU "acu_topic"

//Padding for the frequencies
#define PARAM_PADDING "padding"

class SafetyMonitor : public rclcpp::Node
{

public:
    SafetyMonitor();

private:
    //unordered map
    std::unordered_map<std::string, TimeFreq> last_times;

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
    rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr control_sub;
    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_sub;

    //Pub to the ACU
    rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr ACU_pub;


    void acu_publisher(const std::string &topic_name, const std::string &time);
    template <typename T>
    void update_time(const typename T::SharedPtr msg, const std::string &topic_name);
    void monitor_times();
    void get_state(const lart_msgs::msg::State::SharedPtr msg);

    std::string state_topic;
    std::string acu_topic;

    float padding;
    lart_msgs::msg::State state_msg;
    rclcpp::TimerBase::SharedPtr timer;

    char time_str[100];

};


#endif