#include "../include/led_strips/led_node.hpp"

namespace sgd_hardware_drivers
{

LED_Strip::LED_Strip():
    nav2_util::LifecycleNode("led_node", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("input_topic", rclcpp::ParameterValue("led"));
}

LED_Strip::~LED_Strip()
{
    // Destroy
}

nav2_util::CallbackReturn
LED_Strip::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LED_Strip::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LED_Strip::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LED_Strip::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LED_Strip::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
LED_Strip::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("input_topic", led_);
}

void
LED_Strip::init_pub_sub()
{
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);

    subscriber_ = this->create_subscription<sgd_msgs::msg::Light>(led_, default_qos,
            std::bind(&LED_Strip::on_msg_received, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sgd_msgs::msg::Serial>(serial_topic, default_qos);
}

void
LED_Strip::on_msg_received(const sgd_msgs::msg::Light::SharedPtr msg)
{
    std::string light_status = "L";
    light_status.append(std::to_string(msg->mode)+",");
    light_status.append(std::to_string(msg->strip)+",");
    light_status.append(std::to_string(msg->rgb[0])+",");
    light_status.append(std::to_string(msg->rgb[1])+",");
    light_status.append(std::to_string(msg->rgb[2]));

    sgd_msgs::msg::Serial light_serial;
    light_serial.header = msg->header;
    light_serial.msg = light_status;
    
    publisher_->publish(light_serial);

}






}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::LED_Strip>(); 
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}