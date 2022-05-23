#ifndef SGD_CONTROLLER__DUBINS_PLANNER_HPP_
#define SGD_CONTROLLER__DUBINS_PLANNER_HPP_

// C++ includes
#include <string>
#include <memory>

// ROS2 base includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"

// other includes
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sgd_msgs/msg/route_info.hpp"
#include "route_analyzer.hpp"

namespace sgd_ctrl
{

    class DubinsCurve : public nav2_core::GlobalPlanner
    {
    public:
        DubinsCurve() = default;
        ~DubinsCurve() = default;

        // plugin configure
        void configure(
            rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        // plugin cleanup
        void cleanup() override;

        // plugin activate
        void activate() override;

        // plugin deactivate
        void deactivate() override;

        // This method creates path for given start and goal pose.
        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    private:
        // TF buffer
        std::shared_ptr<tf2_ros::Buffer> tf_;

        // node ptr
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        // Global Costmap
        nav2_costmap_2d::Costmap2D *costmap_;

        // The global frame of the costmap
        std::string global_frame_, name_;

        // parameters
        double interpolation_resolution_;
        double radius_;
        std::string global_plan_topic_;
        nav_msgs::msg::Path global_path;

        // global plan from osm planner
        ulong next_wp_; // next waypoint from global plan
        nav_msgs::msg::Path global_plan;

        rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_plan;
        void on_global_plan_received(const nav_msgs::msg::Path::SharedPtr path);

        rclcpp::Duration route_update_timeout_{1, 0};
        rclcpp::Time last_route_update_;
        rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::RouteInfo>::SharedPtr pub_route_info;
        std::unique_ptr<RouteAnalyzer> route_analyzer;

        /**
         * @brief 
         * 
         * @param pos1 
         * @param pos2 
         * @return double 
         */
        inline double distance(const geometry_msgs::msg::PoseStamped pos1, const geometry_msgs::msg::PoseStamped pos2)
        {
            return std::hypot(pos1.pose.position.x - pos2.pose.position.x, pos1.pose.position.y - pos2.pose.position.y);
        }
    };

} // namespace sgd_ctrl

#endif // SGD_CONTROLLER__DUBINS_PLANNER_HPP_
