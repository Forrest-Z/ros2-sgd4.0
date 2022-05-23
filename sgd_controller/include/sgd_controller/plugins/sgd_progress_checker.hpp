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

#ifndef SGD_CONTROLLER__PLUGINS__SGD_PROGRESS_CHECKER_HPP_
#define SGD_CONTROLLER__PLUGINS__SGD_PROGRESS_CHECKER_HPP_

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

// TODO delete includes
//#include "nav2_core/exceptions.hpp"
//#include "nav_2d_utils/conversions.hpp"
//#include "nav2_util/node_utils.hpp"

namespace sgd_ctrl
{
    /**
     * @class SgdProgressChecker
     * @brief This plugin is used to check the position of the robot to make sure
     * that it is actually progressing towards a goal.
     */

    class SgdProgressChecker : public nav2_core::ProgressChecker
    {
    public:
        void initialize(
            const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
            const std::string &plugin_name) override;
        bool check(geometry_msgs::msg::PoseStamped &current_pose) override;
        void reset() override;

    protected:
        /**
         * @brief Calculates robots movement from baseline pose
         * @param pose Current pose of the robot
         * @return true, if movement is greater than radius_, or false
         */
        bool is_robot_moved_enough(const geometry_msgs::msg::Pose &pose);
        /**
         * @brief Resets baseline pose with the current pose of the robot
         * @param pose Current pose of the robot
         */
        void set_baseline_pose(const geometry_msgs::msg::Pose &pose);

        double pose_distance(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &);

        rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;

        double radius_;
        rclcpp::Duration time_allowance_{0, 0};

        nav_msgs::msg::Path global_plan;

        geometry_msgs::msg::Pose baseline_pose_;
        rclcpp::Time baseline_time_;

        bool baseline_pose_set_{false};
    };
} // namespace nav2_controller

#endif // NAV2_CONTROLLER__PLUGINS__SIMPLE_PROGRESS_CHECKER_HPP_
