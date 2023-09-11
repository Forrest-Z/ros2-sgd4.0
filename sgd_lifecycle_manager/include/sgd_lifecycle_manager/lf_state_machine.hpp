// Copyright 2023 HAW Hamburg
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

#ifndef SGD_LIFECYCLE__LF_STATE_MACHINE_HPP_
#define SGD_LIFECYCLE__LF_STATE_MACHINE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace sgd_lifecycle
{

class LF_State_Machine
{
private:
    // Service sgd_get_state
    // Publisher sgd_state_changed
    // subscriber to crashed nodes
    rclcpp::Subscription<lifecycle_msgs::msg::Transition> sub_node_exit;
    // vector containing subscribers to transition events
    
    
public:
    LF_State_Machine(/* args */);
    ~LF_State_Machine();
};

} // namespace sgd_lifecycle

#endif