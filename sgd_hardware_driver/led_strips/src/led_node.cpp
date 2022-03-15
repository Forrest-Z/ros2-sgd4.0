// Copyright 2022 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "led_strips/led_node.hpp"

namespace sgd_hardware_drivers
{

LED_Strip::LED_Strip():
    rclcpp_lifecycle::LifecycleNode("led_node")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("input_topic", rclcpp::ParameterValue("led"));
}

LED_Strip::~LED_Strip()
{
    // Destroy
}

CallbackReturn
LED_Strip::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    std::string port;
    get_parameter("port", port);
    serial.open_port(port, 9600);
    init_pub_sub();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
LED_Strip::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    return CallbackReturn::SUCCESS;
}

CallbackReturn
LED_Strip::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    return CallbackReturn::SUCCESS;
}

CallbackReturn
LED_Strip::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    return CallbackReturn::SUCCESS;
}

CallbackReturn
LED_Strip::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
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
    //std::string serial_topic = "write_" + port_.substr(port_.find_last_of("/")+1);
    //timer_ = this->create_wall_timer(10ms, std::bind(&LED_Strip::read_serial, this));

    subscriber_ = this->create_subscription<sgd_msgs::msg::Light>(led_, default_qos,
            std::bind(&LED_Strip::on_msg_received, this, std::placeholders::_1));
    //publisher_ = this->create_publisher<sgd_msgs::msg::Serial>(serial_topic, default_qos);
}

void
LED_Strip::on_msg_received(const sgd_msgs::msg::Light::SharedPtr msg)
{
    serial.write_serial(compute_msg(msg));
}

std::string
LED_Strip::compute_msg(const sgd_msgs::msg::Light::SharedPtr msg)
{
    std::string light_status = "L";
    light_status.append(std::to_string(msg->mode)+",");
    light_status.append(std::to_string(msg->strip)+",");
    light_status.append(std::to_string(msg->rgb[0])+",");
    light_status.append(std::to_string(msg->rgb[1])+",");
    light_status.append(std::to_string(msg->rgb[2]));
    
    return light_status;
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