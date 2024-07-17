#include "Safety_monitor/monitor.h"

using std::placeholders::_1;
using namespace std::chrono;

SafetyMonitor::SafetyMonitor() : Node("safety_monitor")
{
    //TODO check the topic names
    // Define the params 
    this->declare_parameter(PARAM_TOPIC_LIMG,"/left");
    this->get_parameter(PARAM_TOPIC_LIMG,left_image);
    this->declare_parameter(PARAM_TOPIC_RIMG,"/right");
    this->get_parameter(PARAM_TOPIC_RIMG,right_image);
    this->declare_parameter(PARAM_TOPIC_DIMG,"/depth");
    this->get_parameter(PARAM_TOPIC_DIMG,depth_image);
    this->declare_parameter(PARAM_TOPIC_LINFO,"/left/info");
    this->get_parameter(PARAM_TOPIC_LINFO,left_info);
    this->declare_parameter(PARAM_TOPIC_RINFO,"/right/info");
    this->get_parameter(PARAM_TOPIC_RINFO,right_info);
    this->declare_parameter(PARAM_TOPIC_DINFO,"/depth/info");
    this->get_parameter(PARAM_TOPIC_DINFO,depth_info);
    this->declare_parameter(PARAM_TOPIC_MAPPER,"/cones");
    this->get_parameter(PARAM_TOPIC_MAPPER,cone_array_topic);
    this->declare_parameter(PARAM_TOPIC_PLANNER,"/path");
    this->get_parameter(PARAM_TOPIC_PLANNER,path_topic);
    this->declare_parameter(PARAM_TOPIC_CONTROL,"/control");
    this->get_parameter(PARAM_TOPIC_CONTROL,control_topic);
    this->declare_parameter(PARAM_TOPIC_ACU,"/acu");
    this->get_parameter(PARAM_TOPIC_ACU,acu_topic);

    this->declare_parameter(PARAM_FREQ_LIMG,100);
    this->get_parameter(PARAM_FREQ_LIMG,limg_freq);
    this->declare_parameter(PARAM_FREQ_RIMG,100);
    this->get_parameter(PARAM_FREQ_RIMG,rimg_freq);
    this->declare_parameter(PARAM_FREQ_DIMG,100);
    this->get_parameter(PARAM_FREQ_DIMG,dimg_freq);
    this->declare_parameter(PARAM_FREQ_LINFO,100);
    this->get_parameter(PARAM_FREQ_LINFO,linf_freq);
    this->declare_parameter(PARAM_FREQ_RINFO,100);
    this->get_parameter(PARAM_FREQ_RINFO,rinf_freq);
    this->declare_parameter(PARAM_FREQ_DINFO,100);
    this->get_parameter(PARAM_FREQ_DINFO,dinf_freq);
    this->declare_parameter(PARAM_FREQ_MAPPER,200);
    this->get_parameter(PARAM_FREQ_MAPPER,mapr_freq);
    this->declare_parameter(PARAM_FREQ_PLANNER,200);
    this->get_parameter(PARAM_FREQ_PLANNER,plan_freq);
    this->declare_parameter(PARAM_FREQ_CONTROL,200);
    this->get_parameter(PARAM_FREQ_CONTROL,ctrl_freq);

    // Define the last time for each subscriber
    last_times["left_image"] = system_clock::now();
    last_times["right_image"] = system_clock::now();
    last_times["depth_image"] = system_clock::now();
    last_times["left_info"] = system_clock::now();
    last_times["right_info"] = system_clock::now();
    last_times["depth_info"] = system_clock::now();
    last_times["cone_array_topic"] = system_clock::now();
    last_times["path_topic"] = system_clock::now();
    last_times["control_topic"] = system_clock::now();

    // create the subscribers for the camera
    left_img_sub = this->create_subscription<sensor_msgs::msg::Image>(left_image, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, limg_freq, left_image));
    right_img_sub = this->create_subscription<sensor_msgs::msg::Image>(right_image, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, rimg_freq, right_image));
    depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>(depth_image, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, dimg_freq, depth_image));
    left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(left_info, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, linf_freq, left_info));
    right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(right_info, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, rinf_freq, right_info));
    depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(depth_info, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, dinf_freq, depth_info));

    // create the subscriber for the mapper
    mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>(cone_array_topic, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, mapr_freq, cone_array_topic));

    // create the subscriber for the planner
    planning_sub = this->create_subscription<nav_msgs::msg::Path>(path_topic, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, plan_freq, path_topic));

    // create the subscriber for the control
    control_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(control_topic, 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1, ctrl_freq, control_topic));

    // create the publisher for the ACU
    ACU_pub = this->create_publisher<std_msgs::msg::Int8>(acu_topic, 10);
}

void SafetyMonitor::acu_publisher()
{
    auto message = std::make_shared<std_msgs::msg::Int8>();
    message.data = 4;
    RCLCPP_INFO(this->get_logger(), "Publishing to the ACU: '%d'", message->data);

    for (int i = 0; i <= 1000; i++)
    {
        ACU_pub->publish(message);
    }
}

template <typename T>
void SafetyMonitor::check_freq_and_log(const typename T::SharedPtr msg, int frequency, const std::string &topic_name)
{
    auto current_time = system_clock::now();
    if (last_time.time_since_epoch().count() != 0)
    {
        auto duration = duration_cast<milliseconds>(current_time - last_times[topic_name]);
        std::time_t now_c = system_clock::to_time_t(current_time);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&now_c));

        if (duration > milliseconds(frequency))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] %s failed to send a message at the rate of 5Hz", time_str.c_str(), topic_name.c_str());
            acu_publisher();
        }
    }
    last_times[topic_name] = current_time;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<SafetyMonitor>();
    executor.add_node(node);

    rclcpp::shutdown();
    return 0;
}