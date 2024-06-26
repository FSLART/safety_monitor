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
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

using std::placeholders::_1;
using namespace std::chrono;

class SafetyMonitor : public rclcpp::Node
{
public:
    SafetyMonitor(): Node("safety_monitor")
    {
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
        left_img_sub = this->create_subscription<sensor_msgs::msg::Image>("left_image", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,10,"left_image"));
        right_img_sub = this->create_subscription<sensor_msgs::msg::Image>("right_image", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,10,"right_image"));
        depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>("depth_image", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,10,"depth_image"));
        left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("left_info", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,10,"left_info"));
        right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("right_info", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,10,"right_info"));
        depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("depth_info", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,10,"depth_info"));

        // create the subscriber for the mapper
        mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>("cone_array_topic", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,5,"cone_array_topic")); // Replace topic

        // create the subscriber for the planner
        planning_sub = this->create_subscription<nav_msgs::msg::Path>("path_topic", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,5,"path_topic")); // Replace topic

        // create the subscriber for the control
        control_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("control_topic", 10, std::bind(&SafetyMonitor::check_freq_and_log, this, _1,5,"control_topic")); // Replace topic

        // create the publisher for the ACU
        ACU_pub = this->create_publisher<std_msgs::msg::Int8>("acu_topic", 10); // Replace topic
    }

private:
   
    void acu_publisher()
    {
        auto message = std::make_shared<std_msgs::msg::Int8>();
        message.data = 4;
        RCLCPP_INFO(this->get_logger(), "Publishing to the ACU: '%d'", message->data);

        for (int i = 0; i <= 1000; i++)
        {
            ACU_pub->publish(message);
        }
    }

    template<typename T>
    void check_freq_and_log(const typename T::SharedPtr msg, int frequency, const std::string & topic_name)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_times[topic_name]);
            std::time_t now_c = system_clock::to_time_t(current_time);
            char time_str[100];
            std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&now_c));

            /*
            Still need to decide what is more efficient?

            int expected_duration_ms = 1000 / expected_frequency;
            if (duration > expected_duration_ms)
            {
                RCLCPP_ERROR(this->get_logger(), "[%s] %s failed to send a message at the rate of %dHz", time_str.c_str(), topic_name.c_str(), expected_frequency);
                acu_publisher();
            }
            */


            switch(frequency){
                case 5:
                    if (duration > milliseconds(200)) // 5Hz
                    {
                    RCLCPP_ERROR(this->get_logger(), "[%s] %s failed to send a message at the rate of 5Hz",time_str.c_str(),topic_name.c_str());
                    acu_publisher();
                    }
                    break;
                case 10:
                    if (duration > milliseconds(100)) // 10Hz
                    {
                    RCLCPP_ERROR(this->get_logger(), "[%s] %s failed to send a message at the rate of 10Hz",time_str.c_str(),topic_name.c_str());
                    acu_publisher();
                    }
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "[%s] Unexpected frequency value for the %s",time_str,topic_name.c_str());
            }
        }
        last_times[topic_name] = current_time;
    }

    std::chrono::time_point<std::chrono::system_clock> last_time;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub;

    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr mapping_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planning_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr control_sub;
    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<SafetyMonitor>();
    executor.add_node(node);

    //instead I could use multiple callbacks that use the same function maybe.

    rclcpp::shutdown();
    return 0;
}