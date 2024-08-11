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


    this->declare_parameter(PARAM_FREQ_LIMG,100.0);
    this->get_parameter(PARAM_FREQ_LIMG,limg_freq);
    this->declare_parameter(PARAM_FREQ_RIMG,100.0);
    this->get_parameter(PARAM_FREQ_RIMG,rimg_freq);
    this->declare_parameter(PARAM_FREQ_DIMG,100.0);
    this->get_parameter(PARAM_FREQ_DIMG,dimg_freq);
    this->declare_parameter(PARAM_FREQ_LINFO,100.0);
    this->get_parameter(PARAM_FREQ_LINFO,linf_freq);
    this->declare_parameter(PARAM_FREQ_RINFO,100.0);
    this->get_parameter(PARAM_FREQ_RINFO,rinf_freq);
    this->declare_parameter(PARAM_FREQ_DINFO,100.0);
    this->get_parameter(PARAM_FREQ_DINFO,dinf_freq);
    this->declare_parameter(PARAM_FREQ_MAPPER,200.0);
    this->get_parameter(PARAM_FREQ_MAPPER,mapr_freq);
    this->declare_parameter(PARAM_FREQ_PLANNER,200.0);
    this->get_parameter(PARAM_FREQ_PLANNER,plan_freq);
    this->declare_parameter(PARAM_FREQ_CONTROL,200.0);
    this->get_parameter(PARAM_FREQ_CONTROL,ctrl_freq);


    this->declare_parameter(PARAM_PADDING,0.7);
    this->get_parameter(PARAM_PADDING,padding);

    //add padding to the frequencys
    for (auto &&freq : freqs)
    {
        freq = freq * padding;
    }

    // Define the last time for each subscriber and his frequency
    last_times[left_image] = TimeFreq(limg_freq,system_clock::now());
    last_times[right_image] = TimeFreq(rimg_freq,system_clock::now());
    last_times[depth_image] = TimeFreq(dimg_freq,system_clock::now());
    last_times[left_info] = TimeFreq(linf_freq,system_clock::now());
    last_times[right_info] = TimeFreq(rinf_freq,system_clock::now());
    last_times[depth_info] = TimeFreq(dinf_freq,system_clock::now());
    last_times[cone_array_topic] = TimeFreq(mapr_freq,system_clock::now());
    last_times[path_topic] = TimeFreq(plan_freq,system_clock::now());
    last_times[control_topic] = TimeFreq(ctrl_freq,system_clock::now());


    //initialize the state
    state_msg.data = lart_msgs::msg::State::OFF;

    // create the subscribers for the camera
    left_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        left_image, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            update_time<sensor_msgs::msg::Image>(msg, left_image);
        });

    right_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        right_image, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            update_time<sensor_msgs::msg::Image>(msg, right_image);
        });

    depth_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depth_image, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            update_time<sensor_msgs::msg::Image>(msg, depth_image);
        });

    left_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        left_info, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            update_time<sensor_msgs::msg::CameraInfo>(msg, left_info);
        });

    right_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        right_info, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            update_time<sensor_msgs::msg::CameraInfo>(msg, right_info);
        });

     depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            update_time<sensor_msgs::msg::CameraInfo>(msg, depth_info);
        });

    // create the subscriber for the mapper
    mapping_sub = this->create_subscription<lart_msgs::msg::ConeArray>(
        cone_array_topic, 10, [this](const lart_msgs::msg::ConeArray::SharedPtr msg) {
            update_time<lart_msgs::msg::ConeArray>(msg, cone_array_topic);
        });


    // create the subscriber for the planner
    planning_sub = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
            update_time<nav_msgs::msg::Path>(msg, path_topic);
        });

    // create the subscriber for the control
    control_sub = this->create_subscription<lart_msgs::msg::DynamicsCMD>(
        control_topic, 10, [this](const lart_msgs::msg::DynamicsCMD::SharedPtr msg) {
            update_time<lart_msgs::msg::DynamicsCMD>(msg, control_topic);
        });

    // create the subscriber for the state controller
    state_sub = this->create_subscription<lart_msgs::msg::State>(state_topic, 10, std::bind(&SafetyMonitor::get_state,this, _1));

    // create the publisher for the ACU
    ACU_pub = this->create_publisher<lart_msgs::msg::State>(acu_topic, 10);

    //create wall timer to call the monitor function
    timer = this->create_wall_timer(100ms, std::bind(&SafetyMonitor::monitor_times, this));

}


//publishes to the ACU only if the car is in Driving
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
        auto duration = duration_cast<milliseconds>(current_time - last_times[pair.first].getTime());

        std::time_t now_c = system_clock::to_time_t(current_time);
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&now_c));

        if (duration > milliseconds((int)last_times[pair.first].getFrequency()))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] %s failed to send a message in time", time_str, pair.first.c_str());
            acu_publisher();
        }

    }
    
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