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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/dynamics.hpp"
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

    //Subs for the mapper|planner|control
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr mapping_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_state_sub;
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr ekf_map_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_sub;
    rclcpp::Subscription<lart_msgs::msg::PathSpline>::SharedPtr planning_sub;
    rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr control_sub;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr acu_dynamics_sub;

    //Sub for the state
    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_sub;

    //Pub to the ACU
    rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr ACU_pub;

    void acu_publisher(const std::string &topic_name, const std::string &time);

    void update_time(const std::string &topic_name);

    void monitor_times();

    void get_state(const lart_msgs::msg::State::SharedPtr msg);

    void mapping_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void ekf_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void ekf_map_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void planning_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    void control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg);
    void acu_dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);


    std::string state_topic;
    std::string acu_topic;
    std::vector<std::string> topics;

    float padding;
    lart_msgs::msg::State state_msg;
    rclcpp::TimerBase::SharedPtr timer;

    char time_str[100];

};


#endif