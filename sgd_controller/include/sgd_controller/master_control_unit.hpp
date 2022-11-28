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
#include "nav_msgs/msg/path.hpp"
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

using Nav2Pose_GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

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
    geometry_msgs::msg::Point goalpose_;
    int next_wp_;   // number of next waypoint in global plan
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_action_client_;
    nav2_msgs::action::NavigateToPose::Goal nav_to_pose_goal_;
    Nav2Pose_GoalHandle::SharedPtr nav_to_pose_goal_handle_;

    nav_msgs::msg::Path global_plan;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_plan;
    void on_global_plan_received(const nav_msgs::msg::Path::SharedPtr path);

    /**
     * @brief Handle received goal pose
     * 
     * @param msg 
     */
    void on_goalpose_received(const geometry_msgs::msg::Point::SharedPtr msg);
    void send_goal_action(const geometry_msgs::msg::Point pnt);

    void goal_response_callback(std::shared_future<Nav2Pose_GoalHandle::SharedPtr> future);
    void feedback_callback(Nav2Pose_GoalHandle::SharedPtr,
            const std::shared_ptr<const nav2_msgs::action::NavigateToPose_Feedback> feedback);
    void result_callback(const Nav2Pose_GoalHandle::WrappedResult & result);

    /**
     * @brief Handle received route info
     * 
     * @param msg_ 
     */
    void on_route_info_received(const sgd_msgs::msg::RouteInfo::SharedPtr msg_);

    /**
     * @brief 
     * 
     * @param pos1 
     * @param pos2 
     * @return double 
     */
    inline double distance(const geometry_msgs::msg::Point pos1, const geometry_msgs::msg::Point pos2)
    {
        return std::hypot(pos1.x - pos2.x, pos1.y - pos2.y);
    }
    
public:
    Master_Control_Unit();
    ~Master_Control_Unit();
};

}   // namespace sgd_ctrl

#endif