#include "Safety_monitor/monitor.h"

using std::placeholders::_1;
using namespace std::chrono;

SafetyMonitor::SafetyMonitor() : Node("safety_monitor")
{
    
    this->declare_parameter(PARAM_TOPIC_STATE,"/state");
    this->get_parameter(PARAM_TOPIC_STATE,state_topic);
    this->declare_parameter(PARAM_TOPIC_ACU,"/acu");
    this->get_parameter(PARAM_TOPIC_ACU,acu_topic);

    this->declare_parameter(PARAM_PADDING,1.0);
    this->get_parameter(PARAM_PADDING,padding);

    std::vector<std::string> topics = this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});

    std::vector<double> freqs = this->declare_parameter<std::vector<double>>("freqs", std::vector<double>{});

    int i=0;
    for (auto &&topic : topics)
    {
        freqs[i] = freqs[i] * padding;

        last_times[topic] = TimeFreq(freqs[i],system_clock::now());
        i++;
    }


    //initialize the state
    state_msg.data = lart_msgs::msg::State::OFF;

    // create the subscribers for the camera
    left_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        topics[0], 10, [this,&topics](const sensor_msgs::msg::Image::SharedPtr msg) {
            update_time<sensor_msgs::msg::Image>(msg, topics[0]);
        });

    right_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        topics[1], 10, [this,&topics](const sensor_msgs::msg::Image::SharedPtr msg) {
            update_time<sensor_msgs::msg::Image>(msg, topics[1]);
        });

    depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        topics[2], 10, [this,&topics](const sensor_msgs::msg::Image::SharedPtr msg) {
            update_time<sensor_msgs::msg::Image>(msg, topics[2]);
        });

    left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topics[3], 10, [this,&topics](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            update_time<sensor_msgs::msg::CameraInfo>(msg, topics[3]);
        });

    right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topics[4], 10, [this,&topics](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            update_time<sensor_msgs::msg::CameraInfo>(msg, topics[4]);
        });

     depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topics[5], 10, [this,&topics](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            update_time<sensor_msgs::msg::CameraInfo>(msg, topics[5]);
        });

    // create the subscriber for the mapper
    mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>(
        topics[6], 10, [this,&topics](const lart_msgs::msg::ConeArray::SharedPtr msg) {
            update_time<lart_msgs::msg::ConeArray>(msg, topics[6]);
        });


    // create the subscriber for the planner
    planning_sub = this->create_subscription<nav_msgs::msg::Path>(
        topics[7], 10, [this,&topics](const nav_msgs::msg::Path::SharedPtr msg) {
            update_time<nav_msgs::msg::Path>(msg, topics[7]);
        });

    // create the subscriber for the control
    control_sub = this->create_subscription<lart_msgs::msg::DynamicsCMD>(
        topics[8], 10, [this,&topics](const lart_msgs::msg::DynamicsCMD::SharedPtr msg) {
            update_time<lart_msgs::msg::DynamicsCMD>(msg, topics[8]);
        });

    // create the subscriber for the state controller
    state_sub = this->create_subscription<lart_msgs::msg::State>(state_topic, 10, std::bind(&SafetyMonitor::get_state,this, _1));

    // create the publisher for the ACU
    ACU_pub = this->create_publisher<lart_msgs::msg::State>(acu_topic, 10);

    //create wall timer to call the monitor function
    timer = this->create_wall_timer(100ms, std::bind(&SafetyMonitor::monitor_times, this));

}


//publishes to the ACU only if the car is in Driving
void SafetyMonitor::acu_publisher(const std::string &topic_name, const std::string &time)
{
    auto message = lart_msgs::msg::State();
    message.data = lart_msgs::msg::State::EMERGENCY;
    
    if(state_msg.data == lart_msgs::msg::State::DRIVING){
        RCLCPP_INFO(this->get_logger(), "Publishing to the ACU");

        try {
            std::ofstream outFile("SafetyMonitorLog.txt", std::ios::app);

            if (!outFile) {
                throw std::ios_base::failure("Failed to open the log file.");
            }

            outFile << "[" << time << "] " << topic_name << " failed to send a message in time" << std::endl;
            outFile.close();

        } catch (const std::ios_base::failure &e) {
            RCLCPP_ERROR(this->get_logger(), "File operation failed: %s", e.what());
        }

        for (int i = 0; i <= 1000; i++)
        {
            ACU_pub->publish(message);
        }
    }

}

// updates the time point of the last message received by a topic
template <typename T>
void SafetyMonitor::update_time(const typename T::SharedPtr msg, const std::string &topic_name){
    (void) msg;
    auto current_time = system_clock::now();
    last_times[topic_name].setTime(current_time);
}

// periodically checks if every topic is sending msgs in time
void SafetyMonitor::monitor_times(){
    for (auto &&pair : last_times)
    {
        auto current_time = system_clock::now();
        auto duration = duration_cast<milliseconds>(current_time - pair.second.getTime());

        std::time_t now_c = system_clock::to_time_t(current_time);
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&now_c));

        if (duration > std::chrono::milliseconds((int)(pair.second.getFrequency())))
        {
            RCLCPP_WARN(this->get_logger(), "[%s] %s failed to send a message in time", time_str, pair.first.c_str());
            acu_publisher(pair.first.c_str(),time_str);
        }

    }
    
}

// receive and update the state
void SafetyMonitor::get_state(const lart_msgs::msg::State::SharedPtr msg){
    state_msg.data = msg->data; 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}