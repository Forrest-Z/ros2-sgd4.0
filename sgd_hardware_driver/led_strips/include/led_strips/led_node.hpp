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

#ifndef SGD_HARDWARE__LED_NODE_HPP_
#define SGD_HARDWARE__LED_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sgd_msgs/msg/light.hpp"

#include "sgd_io/serial.hpp"

namespace sgd_hardware_drivers
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LED_Strip : public rclcpp_lifecycle::LifecycleNode
{
public:
    LED_Strip();
    ~LED_Strip();

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    void init_parameters();
    std::string port_;
    std::string led_;

    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sgd_msgs::msg::Light>::SharedPtr subscriber_;

    sgd_io::Serial serial;

    void on_msg_received(const sgd_msgs::msg::Light::SharedPtr msg);

    std::string compute_msg(const sgd_msgs::msg::Light::SharedPtr msg);
    
};
}

#endif