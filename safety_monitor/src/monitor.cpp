#include "Safety_monitor/monitor.h"

using std::placeholders::_1;
using namespace std::chrono;

SafetyMonitor::SafetyMonitor() : Node("safety_monitor")
{
    // TODO check the topic names
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
    this->declare_parameter(PARAM_TOPIC_STATE,"/state");
    this->get_parameter(PARAM_TOPIC_STATE,state_topic);

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

    this->declare_parameter(PARAM_PADDING,0.7);
    this->get_parameter(PARAM_PADDING,padding);

    // Define the last time for each subscriber
    last_times[left_image] = system_clock::now();
    last_times[right_image] = system_clock::now();
    last_times[depth_image] = system_clock::now();
    last_times[left_info] = system_clock::now();
    last_times[right_info] = system_clock::now();
    last_times[depth_info] = system_clock::now();
    last_times[cone_array_topic] = system_clock::now();
    last_times[path_topic] = system_clock::now();
    last_times[control_topic] = system_clock::now();

    //initialize the state
    state_msg.data = lart_msgs::msg::State::OFF;

    // create the subscribers for the camera
    left_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        left_image, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            check_freq_and_log<sensor_msgs::msg::Image>(msg, limg_freq, left_image);
        });

    right_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        right_image, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            check_freq_and_log<sensor_msgs::msg::Image>(msg, rimg_freq, right_image);
        });

    depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depth_image, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            check_freq_and_log<sensor_msgs::msg::Image>(msg, dimg_freq, depth_image);
        });

    left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        left_info, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            check_freq_and_log<sensor_msgs::msg::CameraInfo>(msg, linf_freq, left_info);
        });

    right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        right_info, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            check_freq_and_log<sensor_msgs::msg::CameraInfo>(msg, rinf_freq, right_info);
        });

     depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            check_freq_and_log<sensor_msgs::msg::CameraInfo>(msg, dinf_freq, depth_info);
        });

    // create the subscriber for the mapper
    mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>(
        cone_array_topic, 10, [this](const lart_msgs::msg::ConeArray::SharedPtr msg) {
            check_freq_and_log<lart_msgs::msg::ConeArray>(msg, mapr_freq, cone_array_topic);
        });


    // create the subscriber for the planner
    planning_sub = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
            check_freq_and_log<nav_msgs::msg::Path>(msg, plan_freq, path_topic);
        });

    // create the subscriber for the control
    control_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        control_topic, 10, [this](const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
            check_freq_and_log<ackermann_msgs::msg::AckermannDrive>(msg, ctrl_freq, control_topic);
        });

    // create the subscriber for the state controller
    state_sub = this->create_subscription<lart_msgs::msg::State>(state_topic, 10, std::bind(&SafetyMonitor::get_state,this, _1));

    // create the publisher for the ACU
    ACU_pub = this->create_publisher<lart_msgs::msg::State>(acu_topic, 10);
}

void SafetyMonitor::acu_publisher()
{
    auto message = lart_msgs::msg::State();
    message.data = lart_msgs::msg::State::EMERGENCY;
    
    if(state_msg.data == lart_msgs::msg::State::DRIVING){
        RCLCPP_INFO(this->get_logger(), "Publishing to the ACU");

        for (int i = 0; i <= 1000; i++)
        {
            ACU_pub->publish(message);
        }
    }

}

template <typename T>
void SafetyMonitor::check_freq_and_log(const typename T::SharedPtr msg, float frequency, const std::string &topic_name)
{
    (void) msg;
    auto current_time = system_clock::now();
    if (last_times[topic_name].time_since_epoch().count() != 0)
    {
        auto duration = duration_cast<milliseconds>(current_time - last_times[topic_name]);
        std::time_t now_c = system_clock::to_time_t(current_time);
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&now_c));
        
        float time_expected = frequency * (padding+1);

        float time_expected = frequency * (padding+1);

        //add padding to the exepected frequency and compare the time intervals 
        if (duration > milliseconds((int)time_expected))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] %s failed to send a message", time_str, topic_name.c_str());
            acu_publisher();
        }
    }
    last_times[topic_name] = current_time;
}

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