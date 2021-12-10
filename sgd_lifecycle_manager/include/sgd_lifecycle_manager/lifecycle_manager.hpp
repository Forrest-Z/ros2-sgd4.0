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

#include "tinyxml2.h"
#include "lf_node_factory.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace sgd_lifecycle
{

class Lifecycle_Manager : public rclcpp::Node
{

public:
    Lifecycle_Manager();
    ~Lifecycle_Manager();

protected:
    // parameters
    std::string launch_file;

    // map to store service clients
    std::unordered_map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> service_clients_;

    //void read_xml_file(tinyxml2::XMLElement * node, group *g = nullptr);
    //std::vector<lifecycle_node> lifecycle_nodes;

    //! \brief Init subscriber
    void init_sub();
    rclcpp::TimerBase::SharedPtr timer_;

    bool change_state(uint8_t transition_id, std::string node_name);
    //bool all_nodes_active();
    //bool is_node_active(std::string node_name);
};

}

#endif  // SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
