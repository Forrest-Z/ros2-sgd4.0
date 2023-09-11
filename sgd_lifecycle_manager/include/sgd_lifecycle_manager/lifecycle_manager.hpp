// Copyright 2021 HAW Hamburg
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

#ifndef SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
#define SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "tinyxml2.h"
#include "lf_node_factory.hpp"

// using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

using LF_State = lifecycle_msgs::msg::State;

namespace sgd_lifecycle
{

class Lifecycle_Manager : public rclcpp::Node
{

public:
    Lifecycle_Manager();
    ~Lifecycle_Manager();

protected:
    // parameters
    std::string launch_file;    /// @brief Name of the xml launch file
    bool is_sim_;               /// @brief Set to true if simulation is running

    uint8_t SGD_GOAL_STATE; /// @brief goal state of the Shared Guide Dog 4.0
    uint8_t SGD_STATE;      /// @brief state of the Shared Guide Dog 4.0
    
    rclcpp::TimerBase::SharedPtr timer_;

    /// @brief Map to store ChangeState service clients
    std::unordered_map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> service_clients_;

    /// @brief Vector containing all subscriber to ChangeState events
    std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> sub_node_transition;

    /// @brief Subscription to node_exit topic
    // rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr sub_node_exit;

    /// @brief Publisher for SGD_STATE change
    rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr pub_state_change;

    /**
     * @brief Calls the change_state service for the specified node.
     * 
     * @param transition_id goal state
     * @param node_name name of the node
     * @return true if state change was successful
     * @return false otherwise
     */
    bool change_state(uint8_t transition_id, std::string node_name);

    /**
     * @brief This function is called when a node sends a transition event and the
     * subscriber receives it
     * 
     * @param msg the message
     * @param node_name the node who sent the event
     */
    void on_node_transition_received(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg, std::string node_name);
};

}

#endif  // SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
