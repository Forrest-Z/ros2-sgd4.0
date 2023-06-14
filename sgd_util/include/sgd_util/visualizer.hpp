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

#ifndef SGD_UTIL__VISUALIZER_HPP_
#define SGD_UTIL__VISUALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sgd_util
{

class Visualizer : public rclcpp::Node
{

public:
    Visualizer();
    ~Visualizer();

protected:
    int marker_lifetime_;
    std_msgs::msg::ColorRGBA status_colors[6];

    //! \brief Init publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_gnss_pose_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_uwb_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_odom_orientation_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_uwb_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    /**
     * @brief Is called when a new gnss message is received
     * @param msg the received message
     */
    void on_gnss_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    /**
     * @brief Is called when a new uwb message is received.
     * @param msg
     */
    void on_uwb_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    /**
     * @brief Is called when a new odom message is received
     * @param msg
     */
    void on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Create a marker message. The marker type is optional, standard value is a cylinder
     * 
     * @param marker_type the marker type
     * @return visualization_msgs::msg::Marker 
     */
    visualization_msgs::msg::Marker create_marker(int32_t marker_type = visualization_msgs::msg::Marker::CYLINDER);

    /**
     * @brief Create a std_msgs::msg::ColorRGBA object
     *
     * @param r red value
     * @param g green value
     * @param b blue value
     * @return std_msgs::msg::ColorRGBA
     */
    std_msgs::msg::ColorRGBA create_color(int r, int g, int b);
};

} // namespace sgd_util

#endif // SGD_UTIL__VISUALIZER_HPP_