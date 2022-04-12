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

#ifndef NAV_SGD__GLOBAL_PLANNER_OSM_HPP_
#define NAV_SGD__GLOBAL_PLANNER_OSM_HPP_

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "yaml-cpp/yaml.h"

//#include "nav2_util/lifecycle_node.hpp"

#include "sgd_msgs/srv/get_global_plan.hpp"
#include "sgd_msgs/srv/get_map_info.hpp"
#include "sgd_util/geotools.hpp"
#include "a_star.hpp"
#include "a_star_users.hpp"

namespace nav_sgd
{

using namespace std::placeholders;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
//using CallbackReturn = nav2_util::CallbackReturn;

class Global_Planner_OSM : public rclcpp_lifecycle::LifecycleNode // nav2_util::LifecycleNode // 
{

//! \brief Struct to hold position information latitude, longitude and angle around z in radians.
struct POSE
{
    double lat, lon, angle;
};

public:
    Global_Planner_OSM();
    ~Global_Planner_OSM();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    std::string waypoints_topic_;
    std::string clicked_point_topic_;
    std::string yaml_filename_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
    //rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
    

    //! \brief Init transforms
    void init_transforms();
    sgd_util::LatLon map_origin;
    CallbackReturn wait_for_transform();
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;

    /**
     * @brief Read the map yaml and parse parameters
     */
    CallbackReturn init_yaml();
    std::string map_filename;
    std::string users_filename;
    std::string adr_filename;

    // A*
    std::shared_ptr<A_Star_Users> a_star_users;
    std::unique_ptr<A_Star> a_star;

    // create services for global planner
    rclcpp::Service<sgd_msgs::srv::GetMapInfo>::SharedPtr map_info_srv;
    /**
     * @brief Get information about the active map
     * 
     * @param request 
     * @param response 
     */
    void getMapInfo(const std::shared_ptr<sgd_msgs::srv::GetMapInfo::Request> request,
                    std::shared_ptr<sgd_msgs::srv::GetMapInfo::Response> response);

    rclcpp::Service<sgd_msgs::srv::GetGlobalPlan>::SharedPtr compute_path_srv;
    /**
     * @brief Compute a path from current position to goal pose
     * 
     * @param request 
     * @param response 
     */
    void computePath(const std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Request> request,
                     std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Response> response);

    // action
    std::chrono::milliseconds server_timeout_;
    //rclcpp::Node::SharedPtr client_node_;
    //rclcpp::Service<sgd_msgs::srv::ComputePath>::SharedPtr compute_path_srv;
    //rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_action_client_;
    //nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
    //rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_goal_handle_;

    /**
     * @brief Publish the waypoints to show on the map
     * @param data the poses to publish
     */
    void publish_path(std::vector<geometry_msgs::msg::Pose> data);
    
    /**
     * @brief Calls the waypoint following service with the specified poses
     * @param data The poses to publish
     */
    void start_waypoint_following(std::vector<geometry_msgs::msg::Pose> data);

    /**
     * @brief Create a vector with ros2 poses from a vector of waypoints containing LatLon objects
     * 
     * @param waypoints the waypoints to create poses from
     * @return A vector containing the ros2 poses
     */
    std::vector<geometry_msgs::msg::Pose> create_poses_from_waypoints(std::vector<sgd_util::LatLon> waypoints);

    /**
     * @brief Get the value from yaml node
     * 
     * @tparam T type to return
     * @param node yaml node
     * @param key the key to get the value for
     * @return T 
     */
    template<typename T>
    T yaml_get_value(const YAML::Node & node, const std::string & key)
    {
        try {
            return node[key].as<T>();
        } catch (YAML::Exception & e) {
            std::stringstream ss;
            ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
            throw YAML::Exception(e.mark, ss.str());
        }
    }
};

}   // namespace nav_sgd

#endif  // NAV_SGD__GLOBAL_PLANNER_OSM_HPP_
