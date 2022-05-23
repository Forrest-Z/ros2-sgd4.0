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

#ifndef SGD_LOCALIZATION__LOCALIZER_HPP_
#define SGD_LOCALIZATION__LOCALIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sgd_util/geotools.hpp"
#include "sgd_util/sensor_filter.hpp"


namespace sgd_localization
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Localizer : public rclcpp_lifecycle::LifecycleNode
{

public:
    Localizer();
    ~Localizer();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

    // Parameters
    CallbackReturn init_parameters();
    // basic settings for node
    double frequency_;
    bool is_publish_tf_;
    // frames for tf
    std::string earth_frame_, map_frame_, odom_frame_, base_link_frame_, tf_base_frame_;
    // topics
    std::string odom_, imu_, gps_, uwb_;

    /**
     * @brief Initialize publisher
     */
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_tf_;
    builtin_interfaces::msg::Time last_timestamp_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_uwb_;

    // functions handling subscriptions
    void on_odom_received(nav_msgs::msg::Odometry::SharedPtr msg_, std::string topic_name_);
    void on_imu_recevied(sensor_msgs::msg::Imu::SharedPtr msg_, std::string topic_name_);
    void on_navsatfix_received(sensor_msgs::msg::NavSatFix::SharedPtr msg_, std::string topic_name_);

    // filters for sensor data
    double odom_offset_ = 0.0;
    nav_msgs::msg::Odometry last_odom_;
    sgd_util::SensorFilter filter_imu_rot_;

    // state of sensors
    uint8_t odom_state_ = 0;
    uint8_t imu_state_ = 0;
    uint8_t gps_state_ = 0;
    uint8_t uwb_state_ = 0;

    void publish_tf();

    /**
     * @brief Initialize transforms
     */
    void init_transforms();
    tf2::Duration transform_tolerance_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    sgd_util::LatLon map_origin;

    /**
     * @brief Get the transform from source_frame to target_frame
     * 
     * @param target_frame
     * @param source_frame
     * @return geometry_msgs::msg::TransformStamped
     */
    geometry_msgs::msg::TransformStamped get_transform(const std::string target_frame, const std::string source_frame);
};

} // namespace sgd_localization

#endif