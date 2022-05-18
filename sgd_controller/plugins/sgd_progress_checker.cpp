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

#include "sgd_controller/plugins/sgd_progress_checker.hpp"

namespace sgd_ctrl
{
    void SgdProgressChecker::initialize(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
        const std::string &plugin_name)
    {
        nh_ = node;
        /*nav2_util::declare_parameter_if_not_declared(
            nh_, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(
            nh_, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
        // Scale is set to 0 by default, so if it was not set otherwise, set to 0
        nh_->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);*/

        // initialize parameters
        nh_->declare_parameter(plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
        nh_->declare_parameter(plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));

        nh_->get_parameter(plugin_name + ".required_movement_radius", radius_);
        double time_allowance_param = 0.0;
        nh_->get_parameter(plugin_name + ".movement_time_allowance", time_allowance_param);
        time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

        // create topic listener -> global_plan, sgd_state (not yet)
    }

    bool SgdProgressChecker::check(geometry_msgs::msg::PoseStamped &current_pose)
    {
        // relies on short circuit evaluation to not call is_robot_moved_enough if
        // baseline_pose is not set.
        // TODO check status of robo
        if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose.pose)))
        {
            set_baseline_pose(current_pose.pose);
            return true;
        }
        if ((nh_->now() - baseline_time_) > time_allowance_)
        {
            return false;
        }

        return true;
    }

    void SgdProgressChecker::reset()
    {
        baseline_pose_set_ = false;
    }

    void
    SgdProgressChecker::set_baseline_pose(const geometry_msgs::msg::Pose &pose)
    {
        baseline_pose_ = pose;
        baseline_time_ = nh_->now();
        baseline_pose_set_ = true;
    }

    bool
    SgdProgressChecker::is_robot_moved_enough(const geometry_msgs::msg::Pose &pose)
    {
        if (pose_distance(pose, baseline_pose_) > radius_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    double
    SgdProgressChecker::pose_distance(
        const geometry_msgs::msg::Pose &pose1,
        const geometry_msgs::msg::Pose &pose2)
    {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;

        return std::hypot(dx, dy);
    }

} // namespace sgd_ctrl

PLUGINLIB_EXPORT_CLASS(sgd_ctrl::SgdProgressChecker, nav2_core::ProgressChecker)
