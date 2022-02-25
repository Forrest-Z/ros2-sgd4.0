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

#ifndef SGD_UTIL__WGS_TO_LOCAL_HPP_
#define SGD_UTIL__WGS_TO_LOCAL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sgd_util/geotools.hpp"

namespace sgd_util
{

class WGStoLocal : public rclcpp::Node
{
private:
    std::string gps_topic_;
    std::string gps_local_topic_;

    //! \brief Init publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gps_local_;

    //! \brief Init transforms
    void init_transforms();
    sgd_util::LatLon map_origin;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;

    void on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg_);

public:
    WGStoLocal();
    ~WGStoLocal();
};

} // namespace sgd_util

#endif