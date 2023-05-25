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

#ifndef SGD_HARDWARE_DRIVERS__GNSS_NODE_HPP_
#define SGD_HARDWARE_DRIVERS__GNSS_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sgd_util/geotools.hpp"
#include "sgd_io/serial.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "nmea_parser.hpp"
//#include "ubx_parser.hpp"
#include "ntrip_client.hpp"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Gnss_Node : public rclcpp_lifecycle::LifecycleNode
{

public: 
    Gnss_Node();
    ~Gnss_Node();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Init parameters
     */
    void init_parameters();
    bool is_sim_;
    std::string xml_file_, parser_type_;
    bool is_pub_local_pose_, is_publish_tf_;
    // topics
    std::string local_pose_topic_, gnss_sim_topic_, utc_clock_topic_, gnss_topic_;
    // transforms
    bool is_tf_to_base_link_;
    std::string base_link_frame_id_, odom_frame_id_;
    // ntrip options
    ntrip_opts ntrip_options_;

    /**
     * @brief Initialize publisher
     */
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_tf_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsatfix_;
    rclcpp_lifecycle::LifecyclePublisher<builtin_interfaces::msg::Time>::SharedPtr pub_utc_time_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_local_pose_;

    sensor_msgs::msg::NavSatFix last_msg_;
    double last_heading_ = 0.0;

    /**
     * @brief Initialize transforms
     */
    void init_transforms();
    tf2::Duration transform_tolerance_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    sgd_util::LatLon map_origin;

    // transform from base link to gps frame
    geometry_msgs::msg::Transform tf_base_gps_;
    
    sgd_io::Serial serial;
    void read_serial();
    double hdop;
    int status;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_sim;
    void on_gps_sim_received(sensor_msgs::msg::NavSatFix::SharedPtr msg);

    /**
     * @brief Publish transform
     * 
     */
    void publish_tf();
    double heading_ = 0.0;

    /**
     * @brief 
     * 
     * @param msg 
     * @return geometry_msgs::msg::PoseWithCovarianceStamped 
     */
    geometry_msgs::msg::PoseWithCovarianceStamped to_local(sensor_msgs::msg::NavSatFix msg);

    /**
     * @brief Get the transform from source_frame to target_frame
     * 
     * @param target_frame 
     * @param source_frame 
     * @return geometry_msgs::msg::TransformStamped 
     */
    geometry_msgs::msg::TransformStamped get_transform(const std::string target_frame, const std::string source_frame);

    std::unique_ptr<INMEA_Message> parser_;
    std::unique_ptr<Ntrip_Client> client;

    // logging
    // std::ofstream log_file;
};

} // namespace sgd_hardware_drivers

#endif