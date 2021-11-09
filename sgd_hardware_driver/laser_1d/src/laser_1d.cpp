#include "laser_1d/laser_1d.hpp"

namespace sgd_sensors
{

Laser_1D::Laser_1D():
    nav2_util::LifecycleNode("laser_1d", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("msg_regex", rclcpp::ParameterValue("Laser:(\\d*),R:(\\d*),S:(\\d*),P:(\\d*\\.\\d*),A:(\\d*\\.\\d*)"));
    declare_parameter("min_range", rclcpp::ParameterValue(0.1));
    declare_parameter("max_range", rclcpp::ParameterValue(1.5));
    declare_parameter("max_vel_percent", rclcpp::ParameterValue(150));
}
 
Laser_1D::~Laser_1D()
{
    // Destroy
}

nav2_util::CallbackReturn
Laser_1D::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    regex_ = std::regex(regex_);

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Laser_1D::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Laser_1D::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Laser_1D::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Laser_1D::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Laser_1D::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("msg_regex", msg_regex_);
    get_parameter("min_range", min_range_);
    get_parameter("max_range", max_range_);
    get_parameter("max_vel_percent", max_vel_percent_);

    m_ = (-max_vel_percent_) / (max_range_ - min_range_);
    b_ = -m_ * max_range_;
}

void
Laser_1D::init_pub_sub()
{
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);
    sub_serial_ = this->create_subscription<sgd_msgs::msg::Serial>(serial_topic, default_qos,
            std::bind(&Laser_1D::on_serial_received, this, std::placeholders::_1));
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", default_qos,
            std::bind(&Laser_1D::on_cmd_vel_received, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_contr", default_qos);

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s.",
            serial_topic.c_str(), "cmd_vel_contr");
}

void
Laser_1D::on_serial_received(const sgd_msgs::msg::Serial::SharedPtr msg_)
{
    // {Laser: <time>,R: <range>,S: <status>,P: <peak_count_rate>,A: <ambient_count_rate>}
    // Parse message and save values
    // publish values
    if (msg_->msg.find("Laser") == std::string::npos) {return;}     // message is not from laser sensor

    std::smatch matches;
    std::regex_search(msg_->msg, matches, regex_);
    
    int time, r;//, s, p, a;
    if (matches.size() > 3) // ist gut
    { 
        time = std::stoi(matches[1]);
        r = std::stoi(matches[2]);
        //s = std::stoi(matches[3]);    // currently unused
        //p = std::stoi(matches[4]);
        //a = std::stoi(matches[5]);
    } else { return; }

    vel_modifier_ = set_vel_modifier(r / 1000.0);
    RCLCPP_INFO(get_logger(), "Measured: %f, set vel modifier: %f", r/1000.0, vel_modifier_);
}

void
Laser_1D::on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg_)
{
    msg_->linear.x *= vel_modifier_;
    msg_->angular.z *= vel_modifier_;

    publisher_->publish(*msg_);
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_sensors::Laser_1D>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
