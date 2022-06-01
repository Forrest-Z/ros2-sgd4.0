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

#ifndef SGD_CTRL__SUBSUM_CONTROLLER_HPP_
#define SGD_CTRL__SUBSUM_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sgd_msgs/msg/light.hpp"

namespace sgd_ctrl
{

#define NUM_LAYERS 10

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Subsum_Controller : public rclcpp_lifecycle::LifecycleNode
{

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    std::vector<std::string> in_topics_;
    std::string out_topic_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Light>::SharedPtr pub_light_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subscriber;

    // layer data
    int active_layer;
    std::vector<double> last_time_received_;

    void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg, int layer);

    void pub_lights(int layer);

public:
    Subsum_Controller();
    ~Subsum_Controller();
};

}   // namespace sgd_ctrl

#endif