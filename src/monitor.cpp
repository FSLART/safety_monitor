#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>

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
    SafetyMonitor()
        : Node("safety_monitor"),
          last_time(system_clock::now())
    {
        // create the subscribers for the camera
        left_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "left_image", 10, std::bind(&SafetyMonitor::check_img_left, this, _1));
        right_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "right_image", 10, std::bind(&SafetyMonitor::check_img_right, this, _1));
        depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "depth_image", 10, std::bind(&SafetyMonitor::check_img_depth, this, _1));
        left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "left_info", 10, std::bind(&SafetyMonitor::check_info_left, this, _1));
        right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "right_info", 10, std::bind(&SafetyMonitor::check_info_right, this, _1));
        depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "depth_info", 10, std::bind(&SafetyMonitor::check_info_depth, this, _1));

        // create the subscriber for the mapper
        mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>(
            "cone_array_topic", 10, std::bind(&SafetyMonitor::check_mapping, this, _1)); // Replace topic

        // create the subscriber for the planner
        planning_sub = this->create_subscription<nav_msgs::msg::Path>(
            "path_topic", 10, std::bind(&SafetyMonitor::check_planning, this, _1)); // Replace topic

        // create the subscriber for the control
        control_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "control_topic", 10, std::bind(&SafetyMonitor::check_control, this, _1)); // Replace topic

        // create the publisher for the ACU
        ACU_pub = this->create_publisher<std_msgs::msg::Int8>("acu_topic", 10); // Replace topic
    }

private:
    void LogError(int case_id)
    {
        std::ofstream log_file("LogError.txt", std::ios::app);
        if (log_file.is_open())
        {
            switch (case_id)
            {
            case 1:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The left image topic failed to communicate in time!");
                log_file << "[Camera] left image failed to communicate\n";
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The right image topic failed to communicate in time!");
                log_file << "[Camera] right image failed to communicate\n";
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The depth image topic failed to communicate in time!");
                log_file << "[Camera] depth image failed to communicate\n";
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The left image info topic failed to communicate in time!");
                log_file << "[Camera] left info failed to communicate\n";
                break;
            case 5:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The right image info topic failed to communicate in time!");
                log_file << "[Camera] right info failed to communicate\n";
                break;
            case 6:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The depth image info topic failed to communicate in time!");
                log_file << "[Camera] depth info failed to communicate\n";
                break;
            case 7:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The Mapper topic failed to communicate in time!");
                log_file << "[Mapper] mapping failed to communicate\n";
                break;
            case 8:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The Planner topic failed to communicate in time!");
                log_file << "[Planner] planning failed to communicate\n";
                break;
            case 9:
                RCLCPP_INFO(this->get_logger(), "[FAILURE] The Control topic failed to communicate in time!");
                log_file << "[Control] control failed to communicate\n";
                break;
            }
            log_file.close();
        }
        else
        {
            std::cout << "Failed to open the file\n";
        }
    }

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

    void check_img_left(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(100)) // 10Hz
            {
                LogError(1);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_img_right(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(100)) // 10Hz
            {
                LogError(2);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_img_depth(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(100)) // 10Hz
            {
                LogError(3);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_info_left(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(100)) // 10Hz
            {
                LogError(4);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_info_right(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(100)) // 10Hz
            {
                LogError(5);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_info_depth(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(100)) // 10Hz
            {
                LogError(6);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_mapping(const lart_msgs::msg::ConeArray::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(200)) // 5Hz
            {
                LogError(7);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_planning(const nav_msgs::msg::Path::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(200)) // 5Hz
            {
                LogError(8);
                acu_publisher();
            }
        }
        last_time = current_time;
    }

    void check_control(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
    {
        auto current_time = system_clock::now();
        if (last_time.time_since_epoch().count() != 0)
        {
            auto duration = duration_cast<milliseconds>(current_time - last_time);
            if (duration > milliseconds(200)) // 5Hz
            {
                LogError(9);
                acu_publisher();
            }
        }
        last_time = current_time;
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

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr ACU_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}