//#include "receiver/receiver_ros.hpp"
#include "include/receiver/receiver_ros.hpp"

namespace sgd_hardware_drivers
{

Receiver_Ros::Receiver_Ros():
    LifecycleNode("receiver_ros")
{
    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("max_vel", rclcpp::ParameterValue(0.5));
    declare_parameter("max_rot_vel", rclcpp::ParameterValue(0.5));

    declare_parameter("cmd_vel_master_topic", rclcpp::ParameterValue("cmd_vel/master"));
    declare_parameter("cmd_vel_frsky_topic", rclcpp::ParameterValue("cmd_vel/frsky"));
    declare_parameter("light_topic", rclcpp::ParameterValue("lights"));

    // initialize logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);
    std::string log_file(log_dir_ + "/" + sgd_util::create_log_file("motor"));

    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGD << "Created node " << this->get_name();
}

Receiver_Ros::~Receiver_Ros()
{
    // Destroy
}

CallbackReturn
Receiver_Ros::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Configuring";

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();

    frsky = std::make_unique<sgd_hardware_drivers::FrSky_RX8R>(max_vel_, max_rot_vel_);
    
    std::string port;
    get_parameter("port", port);
    try
    {
        PLOGD << "Open serial port " << port;
        serial.open_port(port, 115200);
    }
    catch(const std::exception& e)
    {
        PLOGE << e.what();
        RCLCPP_ERROR(get_logger(), e.what());
        return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Receiver_Ros::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Activating";
    init_pub_sub();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Receiver_Ros::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Deactivating";
    publish_cmd_vel_master_->on_deactivate();
    publish_cmd_vel_->on_deactivate();
    publish_light_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Receiver_Ros::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Cleanup";
    publish_cmd_vel_master_.reset();
    publish_cmd_vel_.reset();
    publish_light_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Receiver_Ros::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Shutdown";
    return CallbackReturn::SUCCESS;
}

void
Receiver_Ros::init_parameters()
{
    PLOGD << "Init parameters.";
    get_parameter("port", port_);
    get_parameter("max_vel", max_vel_);
    get_parameter("max_rot_vel", max_rot_vel_);

    get_parameter("cmd_vel_master_topic", cmd_vel_master_topic_);
    get_parameter("cmd_vel_frsky_topic", cmd_vel_topic_);
    get_parameter("light_topic", light_topic_);
}

void
Receiver_Ros::init_pub_sub()
{
    timer_ = this->create_wall_timer(10ms, std::bind(&Receiver_Ros::read_serial, this));

    PLOGD << "Create publisher on topic '" << cmd_vel_master_topic_ << "'";
    publish_cmd_vel_master_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_master_topic_, default_qos);
    PLOGD << "Create publisher on topic '" << cmd_vel_topic_ << "'";
    publish_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos);
    PLOGD << "Create publisher on topic '" << light_topic_ << "'";
    publish_light_ = this->create_publisher<sgd_msgs::msg::Light>(light_topic_, default_qos);

    publish_cmd_vel_master_->on_activate();
    publish_cmd_vel_->on_activate();
    publish_light_->on_activate();
}

void
Receiver_Ros::read_serial()
{
    if (serial.read_serial())
    {
        std::string msg = serial.get_msg();
        PLOGV << "Received message " << msg;

        frsky->parse_msg(msg);
        
        if (frsky->is_in_master_mode())
        {
            // publish cmd_vel
            geometry_msgs::msg::Twist cmd_vel_;
            auto vel_rot = frsky->get_vel_rot();
            cmd_vel_.linear.x = vel_rot.first;
            cmd_vel_.angular.z = vel_rot.second;

            publish_cmd_vel_master_->publish(cmd_vel_);
        }
        else if (true)  // if velocity cmd has changed
        {

        }



        if (msg.find("RX8R") == std::string::npos)
        {
            // message has to start with 'RX8R' to be sure it is from the remote control
            return;
        }

        // 8 channels in message, each channel ranges from 172 to 1811
        // channels 1 and 2 are for movement
        // channel 3 controls lights on left side, channel 6 lights on right side
        // channel 8 is master switch

        std::string parsed;
        std::stringstream input_stringstream(msg);
        int channels[8];
        int i = 0;

        // parse channels
        while (getline(input_stringstream, parsed, ',') && i < 8)
        {
            // message starts with 'RX8R' so skip this part
            if (parsed.find("RX8R") != std::string::npos)
            {
                continue;
            }

            try
            {
                // parse value and add it to array
                channels[i] = std::stoi(parsed);
                i++;
            }
            catch(const std::invalid_argument& e)
            {
                RCLCPP_WARN(get_logger(), "Could not parse value %s", parsed.c_str());
                std::cerr << e.what() << '\n';
            }
        }
        // send first two channels as movement
        pub_cmd_vel(channels[0], channels[1], channels[7]);
        pub_light(channels[2], channels[5]);
    }   
}

void
Receiver_Ros::pub_cmd_vel(int ch1, int ch2, int ch8)
{
    // ch1 = forward / backward; ch2 = left / right
    geometry_msgs::msg::Twist cmd_vel_;





    // subtract 992 so each channel ranges from -820 to +819
    ch1 -= 992;
    ch2 -= 992;
    ch8 -= 992;

    // if value is <20 set velocity to 0 to avoid shaking of the shared guide dog
    // calculate speed from channel input
    cmd_vel_.linear.x = abs(ch1) < 20 ? 0 : (double)ch1 / 820.0 * max_vel_;
    cmd_vel_.angular.z = abs(ch2) < 20 ? 0 : (double)-ch2 / 820.0 * max_rot_vel_;

    // publish velocity command
    if (ch8 < 0 && (abs(ch1) > 20 || abs(ch2) > 20))
    {
        last_msg_equ_zero_ = false;
        publish_cmd_vel_->publish(cmd_vel_);
    }
    else if (ch8 > 0)
    {
        // channel 8 is used as a master switch. So if it is pressed publish to velocity master topic
        last_msg_equ_zero_ = false;
        publish_cmd_vel_master_->publish(cmd_vel_);
    }
    else if (abs(ch1) < 20 && abs(ch2) < 20 && !last_msg_equ_zero_)
    {
        // last_msg_equ_zero is used to detect if the user is controlling the robot with the remote
        // control. If the user is not using the remote control don't publish the velocity
        last_msg_equ_zero_ = true;
        publish_cmd_vel_->publish(cmd_vel_);
    }
}

void
Receiver_Ros::pub_light(int ch3, int ch6)
{
    // subtract 992 so each channel ranges from -820 to +819
    ch3 -= 992;
    ch6 -= 992;

    // set state to 0, 1 or 2
    ch3 = abs(ch3) < 20 ? 2 : ch3 > 0;
    ch6 = abs(ch6) < 20 ? 2 : ch6 > 0;

    sgd_msgs::msg::Light light_msg_;
    std::vector<unsigned char> rgb = {0,0,0};

    int mode = -1;
    // if the light mode has changed publish new message
    if (ch3 != lights_l_)
    {
        mode = ch3;
        lights_l_ = ch3;
        light_msg_.strip = sgd_msgs::msg::Light::LEFT;
    }
    else if (ch6 != lights_r_)
    {
        mode = ch6;
        lights_r_ = ch6;
        light_msg_.strip = sgd_msgs::msg::Light::RIGHT;
    }
    else
    {
        // light mode has not changed so do nothing
        return;
    }
    light_msg_.mode = mode;

    // set color depending on mode
    switch (mode)
    {
    case sgd_msgs::msg::Light::FILL:
        rgb[0] = 255;
        break;
    
    case sgd_msgs::msg::Light::BLINK:
        rgb[0] = 255;
        rgb[1] = 165;
        break;

    case sgd_msgs::msg::Light::WAFT:
        rgb[2] = 255;
        break;

    case sgd_msgs::msg::Light::RUN:
        light_msg_.rgb[1] = 255;
        break;

    default:
        break;
    }

    light_msg_.rgb = rgb;

    // publish message
    publish_light_->publish(light_msg_);

}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Receiver_Ros>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
