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

#ifndef SGD_INTERACTION__ROS_AUDIO_HPP
#define SGD_INTERACTION__ROS_AUDIO_HPP

#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sgd_msgs/msg/light.hpp"

namespace sgd_interaction
{

class ROS_Audio : public rclcpp::Node
{
public:
    ROS_Audio();
    ~ROS_Audio();

    enum num_messages : uint8_t {achtung = 0, buergersteige, links, rechts, start, stop, ziel_erreicht};

protected:
    std::string messages[7] = {"achtung", "buergersteige", "links", "rechts", "start", "stop", "ziel_erreicht"};

    std::string voice_msgs_dir_;

    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sgd_msgs::msg::Light>::SharedPtr subscriber_;

    void on_msg_received(const sgd_msgs::msg::Light::SharedPtr msg);
};
}

#endif