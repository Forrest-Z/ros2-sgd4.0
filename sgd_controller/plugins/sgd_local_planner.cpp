#include <cmath>
#include <memory>
#include <string>

#include "sgd_controller/plugins/sgd_local_planner.hpp"

namespace sgd_ctrl
{

void LocalPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    tf_ = tf;
    name_ = name;
    // costmap_ = costmap_ros->getCostmap();
    // global_frame_ = costmap_ros->getGlobalFrameID();

    RCLCPP_INFO(
        node_->get_logger(), "Configuring plugin %s of type LocalPlanner",
        name_.c_str());

    // Parameter initialization
    node_->declare_parameter(name_ + ".interpolation_resolution",
                                rclcpp::ParameterValue(0.1));
    node_->declare_parameter(name_ + ".radius", rclcpp::ParameterValue(2.0));
    node_->declare_parameter(name_ + ".global_plan_topic", rclcpp::ParameterValue("global_plan_sgd"));
    // node_->declare_parameter(name_ + ".route_info_topic", rclcpp::ParameterValue("route_info"));
    // node_->declare_parameter(name_ + ".route_update_frequ", rclcpp::ParameterValue(1.0));

    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    node_->get_parameter(name_ + ".radius", radius_);
    node_->get_parameter(name_ + ".global_plan_topic", global_plan_topic_);

    sub_global_plan = node_->create_subscription<nav_msgs::msg::Path>(global_plan_topic_, default_qos,
                            std::bind(&LocalPlanner::on_global_plan_received, this, std::placeholders::_1));

    // initialize path smoothing
    // path_smoothing = std::make_unique<PathSmoothing>(0.1, 0.9, 1E-6, interpolation_resolution_);

    // create publisher 
    // RCLCPP_INFO(node_->get_logger(), "Creating publisher and subscriber");
    // std::string route_info_topic = node_->get_parameter(name_ + ".route_info_topic").as_string();
    // pub_route_info = node_->create_publisher<sgd_msgs::msg::RouteInfo>(route_info_topic, default_qos);
    // pub_route_info->on_activate();


    // double route_update_frequ_ = node_->get_parameter(name_ + ".route_update_frequ").as_double();
    // route_update_timeout_ = rclcpp::Duration::from_seconds(1/route_update_frequ_);
    // last_route_update_ = node_->now();
}

void LocalPlanner::cleanup()
{
    RCLCPP_INFO(node_->get_logger(), "Cleaning up...");
}

void LocalPlanner::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating...");
}

void LocalPlanner::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating...");
}

nav_msgs::msg::Path
LocalPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                        const geometry_msgs::msg::PoseStamped &goal)
{
    nav_msgs::msg::Path goal_poses;
    // Checking if the goal and start state is in the global frame
    // if (start.header.frame_id != global_frame_)
    // {
    //     RCLCPP_ERROR(
    //         node_->get_logger(),
    //         "Planner will only except start position from %s frame and not from %s",
    //         global_frame_.c_str(), start.header.frame_id);
    //     return goal_poses;
    // }

    // if (goal.header.frame_id != global_frame_)
    // {
    //     RCLCPP_INFO(
    //         node_->get_logger(),
    //         "Planner will only except goal position from %s frame and not from %s",
    //         global_frame_.c_str(), goal.header.frame_id);
    //     return goal_poses;
    // }

    // add start to poses
    goal_poses.poses.push_back(start);

    // check if global_plan is available and end point matches goal
    // if (!global_plan.poses.empty() && goal.pose.position.x == 10.234 && goal.pose.position.y == 1.123)
    // {
    //     // global_plan matches current goal
    //     // where are we now and which point from global_plan is next?
    //     // start is current pose
    //     double dist = distance(start.pose, global_plan.poses[0].pose)+0.2;
    //     while (dist > distance(start.pose, global_plan.poses[next_wp_].pose) && next_wp_ < global_plan.poses.size())
    //     {
    //         dist = distance(start.pose, global_plan.poses[next_wp_].pose);
    //         RCLCPP_INFO(node_->get_logger(), "distance is %.2f -> wp++", dist);
    //         next_wp_++;
    //     }
    //     RCLCPP_INFO(node_->get_logger(), "Add WP to path: %.3f, %.3f", 
    //                 global_plan.poses[next_wp_].pose.position.x,
    //                 global_plan.poses[next_wp_].pose.position.y);
    //     goal_poses.poses.push_back(global_plan.poses[next_wp_]);
        
    //     if (next_wp_+1 < global_plan.poses.size())
    //     {
    //         RCLCPP_INFO(node_->get_logger(), "Add WP to path: %.3f, %.3f", 
    //                 global_plan.poses[next_wp_+1].pose.position.x,
    //                 global_plan.poses[next_wp_+1].pose.position.y);
    //         goal_poses.poses.push_back(global_plan.poses[next_wp_+1]);
    //     }
    // }
    // else
    // {
        RCLCPP_INFO(node_->get_logger(), "Add goal WP to path: %.3f, %.3f", 
                    goal.pose.position.x,
                    goal.pose.position.y);
        goal_poses.poses.push_back(goal);
    // }
    
    goal_poses.header.frame_id = "map";
    return goal_poses;
}

void
LocalPlanner::on_global_plan_received(const nav_msgs::msg::Path::SharedPtr path)
{
    global_plan = *path;
    next_wp_ = 0;

    // add waypoints to route analyzer
    // std::vector<std::pair<double, double>> waypoints;
    // for (auto el : path->poses)
    // {
    //     waypoints.push_back({el.pose.position.x, el.pose.position.y});
    // }
    // route_analyzer->add_waypoints(waypoints);
    // last_route_update_ = node_->now();    // reset timer

    RCLCPP_INFO(node_->get_logger(), "Received global plan with %d waypoints", global_plan.poses.size());
}

} // namespace sgd_ctrl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sgd_ctrl::LocalPlanner, nav2_core::GlobalPlanner)
