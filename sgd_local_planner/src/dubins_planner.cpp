#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>

#include "sgd_local_planner/dubins.hpp"
#include "sgd_local_planner/dubins_planner.hpp"

namespace sgd_local_planner
{

  void DubinsCurve::configure(
      rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    node_->declare_parameter(name_ + ".interpolation_resolution",
                             rclcpp::ParameterValue(0.1));
    node_->declare_parameter(name_ + ".radius", rclcpp::ParameterValue(2.0));
    node_->declare_parameter(name_ + ".global_plan_topic", rclcpp::ParameterValue("global_plan"));

    node_->get_parameter(name_ + ".interpolation_resolution",
                         interpolation_resolution_);
    node_->get_parameter(name_ + ".radius", radius_);
    node_->get_parameter(name_ + ".global_plan_topic", global_plan_topic_);
  }

  void DubinsCurve::cleanup()
  {
    RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type Dubins Curve Planner",
                name_.c_str());
  }

  void DubinsCurve::activate()
  {
    sub_global_plan = node_->create_subscription<nav_msgs::msg::Path>(global_plan_topic_, rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
                std::bind(&DubinsCurve::on_glopbal_plan_received, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Activated plugin %s of type Dubins Curve Planner",
                name_.c_str());
  }

  void DubinsCurve::deactivate()
  {
    RCLCPP_INFO(node_->get_logger(),
                "Deactivating plugin %s of type Dubins Curve Planner", name_.c_str());
  }

  nav_msgs::msg::Path
  DubinsCurve::createPlan(const geometry_msgs::msg::PoseStamped &start,
                          const geometry_msgs::msg::PoseStamped &goal)
  {
    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Planner will only except start position from %s frame and not from %s",
          global_frame_.c_str(), start.header.frame_id);
      return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "Planner will only except goal position from %s frame and not from %s",
          global_frame_.c_str(), goal.header.frame_id);
      return global_path;
    }

    // Check if global plan is available and has the same goal position
    if (is_global_plan_available(goal))
    {
      // Wenn globoal plan available, dann immer die nächsten 2 Punkte mit einbeziehen
    }

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    DubinsPath path;
    double q0[] = {start.pose.position.x, start.pose.position.y,
                   tf2::getYaw(start.pose.orientation)};
    // Final position (x, y, theta)
    double q1[] = {goal.pose.position.x, goal.pose.position.y,
                   tf2::getYaw(goal.pose.orientation)};

    // Calculates and saves the path
    dubins_shortest_path(&path, q0, q1, radius_);
    // Samples the path and adds the pose at every step
    double q[3];
    double x = 0.0;
    double length = dubins_path_length(&path);
    geometry_msgs::msg::PoseStamped pose;
    tf2::Quaternion qpose;
    while (x < length)
    {
      dubins_path_sample(&path, x, q);
      x += interpolation_resolution_;
      pose.pose.position.x = q[0];
      pose.pose.position.y = q[1];
      pose.pose.position.z = 0.0;

      qpose.setRPY(0, 0, q[2]);
      pose.pose.orientation = tf2::toMsg(qpose);

      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }

    return global_path;
  }

  void
  DubinsCurve::on_glopbal_plan_received(const nav_msgs::msg::Path::SharedPtr path)
  {
    global_plan = *path;
    RCLCPP_INFO(node_->get_logger(), "Received global plan");
  }

  bool
  DubinsCurve::is_global_plan_available(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (global_plan.poses.size() > 0)
    {
      auto tol_x = abs(global_plan.poses.back().pose.position.x - goal.pose.position.x);
      auto tol_y = abs(global_plan.poses.back().pose.position.y - goal.pose.position.y);

      return (tol_x < 0.5 && tol_y < 0.5);
    }
    return false;
  }

} // namespace sgd_local_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sgd_local_planner::DubinsCurve, nav2_core::GlobalPlanner)
