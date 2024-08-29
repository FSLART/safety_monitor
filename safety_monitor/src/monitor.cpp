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

    this->topics = this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});

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
    left_img_sub = this->create_subscription<sensor_msgs::msg::Image>(topics[0], 10, std::bind(&SafetyMonitor::leftImage_callback,this, _1));
    right_img_sub = this->create_subscription<sensor_msgs::msg::Image>(topics[1], 10, std::bind(&SafetyMonitor::rightImage_callback,this, _1));
    depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>(topics[2], 10, std::bind(&SafetyMonitor::depthImage_callback,this, _1));

    left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(topics[3], 10, std::bind(&SafetyMonitor::leftInfo_callback,this, _1));
    right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(topics[4], 10, std::bind(&SafetyMonitor::rightInfo_callback,this, _1));
    depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(topics[5], 10, std::bind(&SafetyMonitor::depthInfo_callback,this, _1));

    mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>(topics[6], 10, std::bind(&SafetyMonitor::mapping_callback,this, _1));
    planning_sub = this->create_subscription<nav_msgs::msg::Path>(topics[7], 10, std::bind(&SafetyMonitor::planning_callback,this, _1));
    control_sub = this->create_subscription<lart_msgs::msg::DynamicsCMD>(topics[8], 10, std::bind(&SafetyMonitor::control_callback,this, _1));


    // create the subscriber for the state controller
    state_sub = this->create_subscription<lart_msgs::msg::State>(state_topic, 10, std::bind(&SafetyMonitor::get_state,this, _1));

    // create the publisher for the ACU
    ACU_pub = this->create_publisher<lart_msgs::msg::State>(acu_topic, 10);

    //create wall timer to call the monitor function
    timer = this->create_wall_timer(100ms, std::bind(&SafetyMonitor::monitor_times, this));

}


// TODO: Optimize later!!!
void SafetyMonitor::leftImage_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Left image msg has been received");
    update_time(topics[0]);
}

void SafetyMonitor::rightImage_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Right image msg has been received");
    update_time(topics[1]);
}

void SafetyMonitor::depthImage_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Depth image msg has been received");
    update_time(topics[2]);
}

void SafetyMonitor::leftInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Left info msg has been received");
    update_time(topics[3]);
}

void SafetyMonitor::rightInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Right info msg has been received");
    update_time(topics[4]);
}

void SafetyMonitor::depthInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Depth info msg has been received");
    update_time(topics[5]);
}

void SafetyMonitor::mapping_callback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Mapper msg has been received");
    update_time(topics[6]);
}

void SafetyMonitor::planning_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Planner msg has been received");
    update_time(topics[7]);
}

void SafetyMonitor::control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg)
{
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "Control msg has been received");
    update_time(topics[8]);
}



//publishes to the ACU and logs only if the car is in Driving
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
void SafetyMonitor::update_time(const std::string &topic_name){
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