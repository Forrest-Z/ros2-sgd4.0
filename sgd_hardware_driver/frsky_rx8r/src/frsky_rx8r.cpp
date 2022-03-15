#include "frsky_rx8r/frsky_rx8r.hpp"

namespace sgd_hardware_drivers
{

FrSky_RX8R::FrSky_RX8R():
    LifecycleNode("capacitive_touch")
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

CallbackReturn
FrSky_RX8R::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    std::string port;
    get_parameter("port", port);
    serial.open_port(port, 115200);
    init_pub_sub();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
FrSky_RX8R::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    publish_cmd_vel_master_->on_activate();
    publish_cmd_vel_->on_activate();
    publish_light_->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
FrSky_RX8R::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publish_cmd_vel_master_->on_deactivate();
    publish_cmd_vel_->on_deactivate();
    publish_light_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
FrSky_RX8R::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publish_cmd_vel_master_.reset();
    publish_cmd_vel_.reset();
    publish_light_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
FrSky_RX8R::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
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
    timer_ = this->create_wall_timer(10ms, std::bind(&FrSky_RX8R::read_serial, this));

    publish_cmd_vel_master_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_master", default_qos);
    publish_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_frsky", default_qos);
    publish_light_ = this->create_publisher<sgd_msgs::msg::Light>("lights", default_qos);
}

void
FrSky_RX8R::read_serial()
{
    if (serial.read_serial())
    {
        std::string msg = serial.get_msg();
        RCLCPP_INFO(get_logger(), "End of message reached: %c", msg.c_str());

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
FrSky_RX8R::pub_cmd_vel(int ch1, int ch2, int ch8)
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
FrSky_RX8R::pub_light(int ch3, int ch6)
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
    auto node = std::make_shared<sgd_hardware_drivers::FrSky_RX8R>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
