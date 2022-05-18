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

#ifndef SGD_TF__POSE_ESTIMATOR_HPP_
#define SGD_TF__POSE_ESTIMATOR_HPP_

#include <utility>
#include <math.h> 
#include <array>
#include <string>

//#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sgd_util/geotools.hpp"

namespace sgd_localization
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Pose_Estimator : public rclcpp_lifecycle::LifecycleNode
{

public:
    Pose_Estimator();
    ~Pose_Estimator();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
    
    //! \brief Init parameters
    void init_parameters();
    std::string source_frame_;
    std::string gps_topic_;
    std::string imu_topic_;
    std::string initial_pose_topic_;

    std::string earth_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    std::string tf_base_frame_;

    // init transforms
    CallbackReturn init_transforms();
    sgd_util::LatLon map_origin;
    //CallbackReturn wait_for_transform();
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    void on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg_);
    void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg_);

    void publish_initial_pose();
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    
    // indicator whether gps/imu signal is good enough to estimate pose
    bool gps_good, imu_good;
    bool initial_pose_set;
};

}   // namespace sgd_localization

#endif
