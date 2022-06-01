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

#ifndef SGD_CTRL__MASTER_CONTROL_UNIT_HPP_
#define SGD_CTRL__MASTER_CONTROL_UNIT_HPP_

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sgd_msgs/msg/route_info.hpp"
#include "sgd_msgs/msg/light.hpp"

namespace sgd_ctrl
{

/**
 * @brief The Master Control Unit controls and monitors all processes of the Shared Guide Dog.
 * The user communicates with this node and the requests are then forwarded.
 * 
 */
class Master_Control_Unit : public rclcpp::Node
{

protected:
    //! \brief Init parameters
    std::string goal_pose_topic_;
    std::string route_info_topic_;
    std::string light_topic_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<sgd_msgs::msg::RouteInfo>::SharedPtr sub_route_info_;
    rclcpp::Publisher<sgd_msgs::msg::Light>::SharedPtr pub_light_;

    void init_maneuvers();
    std::map<double, std::string> maneuvers_;

    // action
    std::chrono::milliseconds server_timeout_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_action_client_;
    nav2_msgs::action::NavigateToPose::Goal nav_to_pose_goal_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_goal_handle_;

    /**
     * @brief Handle received goal pose
     * 
     * @param msg 
     */
    void on_goalpose_received(const geometry_msgs::msg::Point::SharedPtr msg);

    /**
     * @brief Handle received route info
     * 
     * @param msg_ 
     */
    void on_route_info_received(const sgd_msgs::msg::RouteInfo::SharedPtr msg_);

public:
    Master_Control_Unit();
    ~Master_Control_Unit();
};

}   // namespace sgd_ctrl

#endif