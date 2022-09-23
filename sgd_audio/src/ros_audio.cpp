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

#include "sgd_audio/ros_audio.hpp"

namespace sgd_interaction
{

ROS_Audio::ROS_Audio(): rclcpp::Node("audio_node")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Init parameters
    declare_parameter("input_topic", rclcpp::ParameterValue("lights"));
    declare_parameter("voice_msgs_dir", rclcpp::ParameterValue("/"));

    std::string topic_;
    get_parameter("input_topic", topic_);
    get_parameter("voice_msgs_dir", voice_msgs_dir_);

    // Create subscription
    subscriber_ = this->create_subscription<sgd_msgs::msg::Light>(topic_, default_qos,
            std::bind(&ROS_Audio::on_msg_received, this, std::placeholders::_1));
    
    // check if audio files exist in voice_msgs_dir
    bool audio_files_exist = false;
    for (const auto & entry : std::filesystem::directory_iterator(voice_msgs_dir_))
    {
        if (entry.path().extension() == ".wav")
        {
            audio_files_exist = true;
            break;
        }
    }
    if (!audio_files_exist)
    {
        RCLCPP_WARN(get_logger(), "Could not find audio files in %s", voice_msgs_dir_.c_str());
        voice_msgs_dir_.clear();
    }
}

ROS_Audio::~ROS_Audio()
{
    // Destroy
}

void
ROS_Audio::on_msg_received(const sgd_msgs::msg::Light::SharedPtr msg)
{
    if (voice_msgs_dir_.empty())
    {
        RCLCPP_WARN(get_logger(), "No audio file found to play.");
        return;
    }
    
    // play sound
    uint8_t num_msg = 0xFF;
    if (msg->mode == sgd_msgs::msg::Light::BLINK && msg->strip == sgd_msgs::msg::Light::LEFT)
    {
        // say left
        num_msg = num_messages::links;
    }
    else if (msg->mode == sgd_msgs::msg::Light::BLINK && msg->strip == sgd_msgs::msg::Light::RIGHT)
    {
        num_msg = num_messages::rechts;
    }
    else if (msg->mode == sgd_msgs::msg::Light::FILL && msg->rgb[0] > 254)
    {
        // say Achtung
        num_msg = num_messages::achtung;
    }
    else if (msg->mode == sgd_msgs::msg::Light::FILL && msg->rgb[1] > 254)
    {
        // Ziel erreicht
        num_msg = num_messages::ziel_erreicht;
    }
    else
    {
        return;
    }

    std::filesystem::path p(voice_msgs_dir_ + "/" + messages[num_msg] + ".wav");
    system(("aplay " + p.string()).c_str());
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_interaction::ROS_Audio>(); 
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}