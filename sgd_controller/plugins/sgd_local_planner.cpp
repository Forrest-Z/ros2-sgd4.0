#include <cmath>
#include <memory>
#include <string>

#include "sgd_controller/plugins/path_smoothing.hpp"
#include "sgd_controller/plugins/sgd_local_planner.hpp"

namespace sgd_ctrl
{

void LocalPlanner::configure(
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
    node_->declare_parameter(name_ + ".global_plan_topic", rclcpp::ParameterValue("global_plan_sgd"));
    node_->declare_parameter(name_ + ".route_info_topic", rclcpp::ParameterValue("route_info"));
    node_->declare_parameter(name_ + ".route_update_frequ", rclcpp::ParameterValue(1.0));

    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    node_->get_parameter(name_ + ".radius", radius_);
    node_->get_parameter(name_ + ".global_plan_topic", global_plan_topic_);

    // initialize path smoothing
    path_smoothing = std::make_unique<PathSmoothing>(0.1, 0.9, 1E-6, interpolation_resolution_);

    // create publisher 
    std::string route_info_topic = node_->get_parameter(name_ + ".route_info_topic").as_string();
    pub_route_info = node_->create_publisher<sgd_msgs::msg::RouteInfo>(route_info_topic, default_qos);
    pub_route_info->on_activate();


    double route_update_frequ_ = node_->get_parameter(name_ + ".route_update_frequ").as_double();
    route_update_timeout_ = rclcpp::Duration::from_seconds(1/route_update_frequ_);
    last_route_update_ = node_->now();

    RCLCPP_ERROR(node_->get_logger(), "Costmap size %.2f x %.2f -> %d x %d", costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY(),
            costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
}

void LocalPlanner::cleanup()
{
    RCLCPP_DEBUG(node_->get_logger(), "Cleaning up plugin %s of type SGD Local Planner",
                name_.c_str());
}

void LocalPlanner::activate()
{
    sub_global_plan = node_->create_subscription<nav_msgs::msg::Path>(global_plan_topic_, default_qos,
                            std::bind(&LocalPlanner::on_global_plan_received, this, std::placeholders::_1));

    route_analyzer = std::make_unique<RouteAnalyzer>();

    RCLCPP_INFO(node_->get_logger(), "Activated plugin %s of type SGD Local Planner",
                name_.c_str());
}

void LocalPlanner::deactivate()
{
    RCLCPP_DEBUG(node_->get_logger(),
                "Deactivating plugin %s of type SGD Local Planner", name_.c_str());
    pub_route_info->on_deactivate();
}

nav_msgs::msg::Path
LocalPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
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

    std::vector<geometry_msgs::msg::Pose> goal_poses;
    // add start to poses
    goal_poses.push_back(start.pose);
    // if (global_plan.poses.size() > 0 && distance(goal, global_plan.poses.back()) < 0.5)
    // {
    //     // Wenn global plan available, dann immer die nächsten 2 Punkte mit einbeziehen
    //     // get next waypoints
    //     for (std::size_t i = 0; i < global_plan.poses.size(); i++)
    //     {
    //         // add next two waypoints if available
    //         if (next_wp_+i >= global_plan.poses.size())      break;
    //         goal_poses.push_back(global_plan.poses.at(next_wp_ + i).pose);
    //     }

    //     if (distance(global_plan.poses.at(next_wp_), start) < 1.0)
    //     {
    //         RCLCPP_INFO(node_->get_logger(), "Reached waypoint %d", next_wp_);
    //         // distance to next waypoint is smaller than a threshold, so count up next_wp_
    //         next_wp_ = (next_wp_ >= global_plan.poses.size()-1) ? next_wp_ : next_wp_+1;
    //         route_analyzer->next_wp();
    //     }
    // }
    // else
    // {
    goal_poses.push_back(goal.pose);
    // }

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    //Smoothen path
    std::vector<xy_pnt> waypoints_;
    for (auto p : goal_poses)
    {
        waypoints_.push_back({p.position.x, p.position.y});
    }
    
    auto smoothened_path = path_smoothing->smoothen_path(waypoints_);
    for (std::size_t i = 0; i < smoothened_path.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = smoothened_path.at(i).first;
        pose.pose.position.y = smoothened_path.at(i).second;
        pose.pose.position.z = 0.0;

        tf2::Quaternion qpose;
        double ang = 0.0;
        if (i < 1)
        {
            // no last waypoint available -> take angle to next waypoint
            ang = atan2(smoothened_path.at(i+1).second - smoothened_path.at(i).second,
                        smoothened_path.at(i+1).first - smoothened_path.at(i).first);
            //RCLCPP_INFO(node_->get_logger(), "First waypoint");
        }
        else if (i >= (smoothened_path.size() - 1))
        {
            // no next waypoint available
            ang = atan2(smoothened_path.at(i).second - smoothened_path.at(i-1).second,
                        smoothened_path.at(i).first - smoothened_path.at(i-1).first);
            //RCLCPP_INFO(node_->get_logger(), "Last waypoint");
        }
        else
        {
            // last and next waypoint available
            ang = (atan2(smoothened_path.at(i+1).second - smoothened_path.at(i).second,
                            smoothened_path.at(i+1).first - smoothened_path.at(i).first)
                    + atan2(smoothened_path.at(i).second - smoothened_path.at(i-1).second,
                            smoothened_path.at(i).first - smoothened_path.at(i-1).first)) / 2;
        }
        // RCLCPP_INFO(node_->get_logger(), "WP %.2f, %.2f ==> Angle: %.2f",
        //         smoothened_path.at(i).first, smoothened_path.at(i).second, ang);
        qpose.setRPY(0.0, 0.0, ang);
        pose.pose.orientation = tf2::toMsg(qpose);

        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
    }

    // if current time - last pub time > publisher frequency
    if ((node_->now() - last_route_update_) > route_update_timeout_ && goal_poses.size() > 2)
    {
        auto next_maeuver = route_analyzer->info(start.pose.position.x, start.pose.position.y);
        last_route_update_ = node_->now();

        sgd_msgs::msg::RouteInfo info_;
        info_.header.stamp = node_->now();
        info_.header.frame_id = "map";

        info_.distance = next_maeuver.distance;
        info_.angle = next_maeuver.angle;
        pub_route_info->publish(info_);
    }
    return global_path;
}

void
LocalPlanner::on_global_plan_received(const nav_msgs::msg::Path::SharedPtr path)
{
    global_plan = *path;
    next_wp_ = 0;

    // add waypoints to route analyzer
    std::vector<std::pair<double, double>> waypoints;
    for (auto el : path->poses)
    {
        waypoints.push_back({el.pose.position.x, el.pose.position.y});
    }
    route_analyzer->add_waypoints(waypoints);
    last_route_update_ = node_->now();    // reset timer

    RCLCPP_INFO(node_->get_logger(), "Received global plan with %d waypoints", global_plan.poses.size());
}

} // namespace sgd_ctrl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sgd_ctrl::LocalPlanner, nav2_core::GlobalPlanner)
