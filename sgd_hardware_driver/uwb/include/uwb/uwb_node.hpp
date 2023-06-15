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

#ifndef SGD_HARDWARE__UWB_NODE_HPP_
#define SGD_HARDWARE__UWB_NODE_HPP_

// #include <regex>
// #include <memory>
// #include <fstream>
// #include <charconv>
// #include <string_view>
// #include <iostream>
#include <string>
// #include <sstream>
// #include <bitset>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/utils.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sgd_io/serial.hpp"
#include "sgd_util/geotools.hpp"
#include "sgd_msgs/msg/odom_improved.hpp"
#include "yaml-cpp/yaml.h"

#include "include/levmarq.hpp"
#include "nlohmann/json.hpp"
#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "sgd_uwb_pf.h"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UWB_Node : public rclcpp_lifecycle::LifecycleNode
{

/**
 * @brief Stores a point with x and y position 
 */
struct point
{
    double x;
    double y;
};


public:
    UWB_Node();
    ~UWB_Node();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Initialize parameters
     */
    void init_parameters();

    // std::regex regex_;                  /// @brief regular expression to get values from received message
    // std::string tag_defs_file_;     /// @brief file to read tag definitions from
    bool is_pub_wgs84_pose_;        /// @brief set to true to publish pose in wgs84 coordinate frame
    bool is_pub_visual_;            /// @brief set to true to publish visualization messages
    // int exp_frequ_;

    std::unordered_map<int, point> tags;    /// @brief map to hold all uwb tags with positions

    bool is_sim_;                   /// @brief is true if simulation is active
    
    /**
     * @brief Initialize publisher and subscriber.
     */
    void init_pub_sub();

    /**
     * @brief Read and parse the file containing tag definitions
     */
    CallbackReturn init_yaml();

    /**
     * @brief Publish the computed pose
     * 
     * @param x the poses x value
     * @param y the poses y value
     */
    void publish_pose(double x, double y);
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_marker_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_position_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_wgs84_pose_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr pub_tag_marker_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr pub_dist_marker_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_impr;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_ground_truth_;

    /**
     * @brief Initialize transforms if the local output is set.
     */
    void init_transforms();
    geometry_msgs::msg::TransformStamped get_transform(std::string target_frame, std::string source_frame);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    sgd_util::LatLon map_origin;
    // transform from base link to uwb frame
    geometry_msgs::msg::Transform tf_base_uwb_;

    /**
     * @brief Class to read from / write to serial port.
     */
    sgd_io::Serial serial;

    /**
     * @brief Read data from serial port. Bind this method to timer.
     */
    void read_serial();

    // std::unique_ptr<IMultilateration> optimizer;

    /**
     * @brief Publish the tag position to visualize in rviz
     */
    void publish_tag_position();

    /**
     * @brief Publish a circle around the tag position to visualize the
     * received distance in rviz.
     * 
     * @param tag_id the tag id
     * @param dist the received distance
     */
    void publish_tag_radius(int tag_id, double dist);

    // uint num_tags;   // number of tags
    //int num_ranges; // number of received measurements
    // bool is_last_estimate_valid;
    // double t_last_meas = 0.0;   // time of last received measurement

    SGD_UWB_ParticleFilter uwb_pf;

    nav_msgs::msg::Odometry last_odom_;
    void on_odom_impr_received(nav_msgs::msg::Odometry::SharedPtr msg);

    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

    /**
     * @brief Is called when a odometry message from gazebo p3d plugin is received 
     * @param msg 
     */
    void on_ground_truth_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

}

#endif
