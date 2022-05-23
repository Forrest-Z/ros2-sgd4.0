#include <cmath>
#include <memory>
#include <string>

#include "sgd_controller/plugins/dubins.hpp"
#include "sgd_controller/plugins/sgd_dubins_planner.hpp"

namespace sgd_ctrl
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
        node_->declare_parameter(name_ + ".route_info_topic", rclcpp::ParameterValue("route_info"));
        node_->declare_parameter(name_ + ".route_update_frequ", rclcpp::ParameterValue(1.0));

        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
        node_->get_parameter(name_ + ".radius", radius_);
        node_->get_parameter(name_ + ".global_plan_topic", global_plan_topic_);

        std::string route_info_topic;
        node_->get_parameter(name_ + ".route_info_topic", route_info_topic);
        pub_route_info = node_->create_publisher<sgd_msgs::msg::RouteInfo>(route_info_topic, default_qos);
        pub_route_info->on_activate();

        double route_update_frequ_ = 1.0;
        node_->get_parameter(name_ + ".route_update_frequ", route_update_frequ_);
        route_update_timeout_ = rclcpp::Duration::from_seconds(1/route_update_frequ_);
        last_route_update_ = node_->now();
    }

    void DubinsCurve::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type Dubins Curve Planner",
                    name_.c_str());
    }

    void DubinsCurve::activate()
    {
        sub_global_plan = node_->create_subscription<nav_msgs::msg::Path>(global_plan_topic_, default_qos,
                                std::bind(&DubinsCurve::on_global_plan_received, this, std::placeholders::_1));

        pub_route_info->on_activate();
        route_analyzer = std::make_unique<RouteAnalyzer>();

        RCLCPP_INFO(node_->get_logger(), "Activated plugin %s of type Dubins Curve Planner",
                    name_.c_str());
    }

    void DubinsCurve::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Deactivating plugin %s of type Dubins Curve Planner", name_.c_str());
        pub_route_info->on_deactivate();
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

        std::vector<geometry_msgs::msg::Pose> goal_poses;
        // add start to poses
        goal_poses.push_back(start.pose);
        // Check if global plan is available and has the same goal position
        if (global_plan.poses.size() > 0 && distance(goal, global_plan.poses.back()) < 0.5)
        {
            // Wenn global plan available, dann immer die nächsten 2 Punkte mit einbeziehen
            // get next waypoints
            for (std::size_t i = 0; i < 2; i++)
            {
                // add next two waypoints if available
                if (next_wp_+i >= global_plan.poses.size())      break;
                goal_poses.push_back(global_plan.poses.at(next_wp_ + i).pose);
            }

            if (distance(global_plan.poses.at(next_wp_), start) < 1.0)
            {
                RCLCPP_INFO(node_->get_logger(), "Reached waypoint %d", next_wp_);
                // distance to next waypoint is smaller than a threshold, so count up next_wp_
                next_wp_ = (next_wp_ >= global_plan.poses.size()-1) ? next_wp_ : next_wp_+1;
            }
        }
        else
        {
            goal_poses.push_back(goal.pose);
        }

        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // Minimum size for goal poses is 2 (start and end pose)
        for (std::size_t i = 1; i < goal_poses.size(); i++)
        {
            DubinsPath path;
            // set start pose
            double q0[] = {goal_poses.at(i-1).position.x, goal_poses.at(i-1).position.y,
                           tf2::getYaw(goal_poses.at(i-1).orientation)};
            // Final position (x, y, theta)
            double q1[] = {goal_poses.at(i).position.x, goal_poses.at(i).position.y,
                           tf2::getYaw(goal_poses.at(i).orientation)};

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
        }

        // if current time - last pub time > publisher frequency
        if ((node_->now() - last_route_update_) > route_update_timeout_)
        {
            auto next_maeuver = route_analyzer->next_step(start.pose.position.x, start.pose.position.y);
            last_route_update_ = node_->now();
            RCLCPP_INFO(node_->get_logger(), "Next maneuver is: %s", next_maeuver.text.c_str());
        }

        return global_path;
    }

    void
    DubinsCurve::on_global_plan_received(const nav_msgs::msg::Path::SharedPtr path)
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
PLUGINLIB_EXPORT_CLASS(sgd_ctrl::DubinsCurve, nav2_core::GlobalPlanner)