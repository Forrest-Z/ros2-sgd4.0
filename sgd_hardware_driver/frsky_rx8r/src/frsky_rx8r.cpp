#include "frsky_rx8r/frsky_rx8r.hpp"

namespace sgd_hardware_drivers
{

FrSky_RX8R::FrSky_RX8R():
    nav2_util::LifecycleNode("capacitive_touch", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("max_vel", rclcpp::ParameterValue(0.5));
    declare_parameter("max_rot_vel", rclcpp::ParameterValue(0.5));
}

FrSky_RX8R::~FrSky_RX8R()
{
    // Destroy
}

nav2_util::CallbackReturn
FrSky_RX8R::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FrSky_RX8R::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    publish_cmd_vel_master_->on_activate();
    publish_cmd_vel_->on_activate();
    publish_light_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FrSky_RX8R::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publish_cmd_vel_master_->on_deactivate();
    publish_cmd_vel_->on_deactivate();
    publish_light_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FrSky_RX8R::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publish_cmd_vel_master_.reset();
    publish_cmd_vel_.reset();
    publish_light_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
FrSky_RX8R::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
FrSky_RX8R::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("max_vel", max_vel_);
    get_parameter("max_rot_vel", max_rot_vel_);
}

void
FrSky_RX8R::init_pub_sub()
{
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);

    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(serial_topic, default_qos,
            std::bind(&FrSky_RX8R::on_msg_received, this, std::placeholders::_1));
    publish_cmd_vel_master_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_master", default_qos);
    publish_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_frsky", default_qos);
    publish_light_ = this->create_publisher<sgd_msgs::msg::Light>("lights", default_qos);
}

void
FrSky_RX8R::on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    if (msg->msg.find("RX8R") == std::string::npos) {return;}     // message is not from touch sensor

    // 8 channels in message, each channel from 172 to 1811
    // channels 1 and 2 are for movement
    // channel 3 controls lights on left side, channel 6 lights on right side

    std::string parsed;
    std::stringstream input_stringstream(msg->msg);
    int channels[8];
    int i = 0;

    while (getline(input_stringstream, parsed, ',') && i < 8)
    {
        if (parsed.find("RX8R") != std::string::npos) continue;

        try
        {
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

void
FrSky_RX8R::pub_cmd_vel(int ch1, int ch2, int ch8)
{
    // ch1 = forward / backward; ch2 = left / right
    geometry_msgs::msg::Twist cmd_vel_;

    ch1 -= 992;
    ch2 -= 992;
    ch8 -= 992;

    cmd_vel_.linear.x = abs(ch1) < 20 ? 0 : (double)ch1 / 820.0 * max_vel_;
    cmd_vel_.angular.z = abs(ch2) < 20 ? 0 : (double)-ch2 / 820.0 * max_rot_vel_;

    if (ch8 < 0 && (abs(ch1) > 20 || abs(ch2) > 20))
    {
        last_msg_equ_zero_ = false;
        publish_cmd_vel_->publish(cmd_vel_);
    }
    else if (ch8 > 0)
    {
        last_msg_equ_zero_ = false;
        publish_cmd_vel_master_->publish(cmd_vel_);
    }
    else if (abs(ch1) < 20 && abs(ch2) < 20 && !last_msg_equ_zero_)
    {
        last_msg_equ_zero_ = true;
        publish_cmd_vel_->publish(cmd_vel_);
    }
}

void
FrSky_RX8R::pub_light(int ch3, int ch6)
{
    ch3 -= 992;
    ch6 -= 992;

    ch3 = abs(ch3) < 20 ? 2 : ch3 > 0;
    ch6 = abs(ch6) < 20 ? 2 : ch6 > 0;

    sgd_msgs::msg::Light light_msg_;
    std::vector<unsigned char> rgb = {0,0,0};

    int mode = -1;
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
        return;
    }
    light_msg_.mode = mode;

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
    publish_light_->publish(light_msg_);

}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::FrSky_RX8R>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
