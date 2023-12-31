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

#ifndef SGD_GLOBAL_PLANNER__PLANNER_OSM_HPP_
#define SGD_GLOBAL_PLANNER__PLANNER_OSM_HPP_

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
#include "nav_msgs/msg/path.hpp"
#include "yaml-cpp/yaml.h"

#include "sgd_msgs/srv/get_global_plan.hpp"
#include "sgd_msgs/srv/get_map_info.hpp"
#include "sgd_util/geotools.hpp"
#include "visualization_msgs/msg/marker.hpp"
// #include "a_star_lib/a_star.hpp"
// #include "a_star.hpp"
// #include "a_star_users.hpp"
#include "../../a_star_lib/include/a_star.hpp"

#include <nlohmann/json.hpp>
#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_global_planner
{

using namespace std::placeholders;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Global_Planner_OSM : public rclcpp_lifecycle::LifecycleNode
{

//! \brief Struct to hold position information latitude, longitude and angle around z in radians.
struct POSE
{
    double lat, lon, angle;
};

public:
    /**
     * @brief Construct a new Global_Planner_OSM object
     * 
     */
    Global_Planner_OSM();

    ~Global_Planner_OSM();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_pnt_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr pub_map_visual_;

    //! \brief Init transforms
    sgd_util::LatLon map_origin;            /// @brief origin of the map frame
    CallbackReturn wait_for_transform();    /// @brief get the transform between earth and map frame from tf
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;    /// @brief tf buffer to get transforms
    geometry_msgs::msg::TransformStamped tf_map_odom_;

    std::string map_frame_;
    std::string robot_base_frame_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * @brief Read the map yaml and parse parameters
     */
    CallbackReturn init_yaml();
    std::string map_filename;
    std::string users_filename;
    std::string adr_filename;
    nlohmann::json js;

    // A*
    std::shared_ptr<A_Star_Users> a_star_users;
    std::unique_ptr<A_Star> a_star;

    rclcpp::Service<sgd_msgs::srv::GetMapInfo>::SharedPtr map_info_srv;
    rclcpp::Service<sgd_msgs::srv::GetGlobalPlan>::SharedPtr compute_path_srv;
    // rclcpp::Service<sgd_msgs::srv::GetMapInfo>::SharedPtr reload_map_srv;

    /**
     * @brief Handle subscription to clicked_point topic
     * 
     * @param msg 
     */
    void on_clicked_pnt(std::shared_ptr<geometry_msgs::msg::PointStamped> msg);

    /**
     * @brief Get information about the active map
     * 
     * @param request 
     * @param response 
     */
    void getMapInfo(const std::shared_ptr<sgd_msgs::srv::GetMapInfo::Request> request,
                    std::shared_ptr<sgd_msgs::srv::GetMapInfo::Response> response);

    /**
     * @brief Compute a path from current position to goal pose
     * 
     * @param request 
     * @param response 
     */
    void computePath(const std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Request> request,
                     std::shared_ptr<sgd_msgs::srv::GetGlobalPlan::Response> response);

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

    visualization_msgs::msg::Marker create_map_visual();
};

}   // namespace sgd_global_planner

#endif  // SGD_GLOBAL_PLANNER__PLANNER_OSM_HPP_
