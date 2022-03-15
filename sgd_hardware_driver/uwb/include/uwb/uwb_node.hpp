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

#include <regex>
#include <memory>
#include <fstream>
#include <charconv>
#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sgd_io/serial.hpp"
#include "sgd_util/geotools.hpp"
#include "sgd_util/ieee_754_conv.hpp"

#include "ieee754.h"

#include "include/levmarq.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <bitset>

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UWB_Node : public rclcpp_lifecycle::LifecycleNode
{
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
    /**
     * @brief Regular expression to use to get values from received message.
     */
    std::regex regex_;
    std::string tag_definitions_;
    bool is_pub_wgs84_pose_;
    int exp_frequ_;
    
    /**
     * @brief Initialize publisher and subscriber.
     */
    void init_pub_sub();

    /**
     * @brief Publish the computed pose
     * 
     * @param x the poses x value
     * @param y the poses y value
     */
    void publish_pose(double x, double y);
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_position_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_wgs84_pose_;

    /**
     * @brief Initialize transforms if the local output is set.
     */
    void init_transforms();
    sgd_util::LatLon map_origin;

    /**
     * @brief Class to read from / write to serial port.
     */
    sgd_io::Serial serial;

    /**
     * @brief Read data from serial port. Bind this method to timer.
     */
    void read_serial();

    std::unique_ptr<IMultilateration> optimizer;

    uint num_tags;   // number of tags
    //int num_ranges; // number of received measurements
    bool is_last_estimate_valid;
    double t_last_meas = 0.0;   // time of last received measurement

    // robo position for debugging
    float real_x, real_y;
};

}

#endif
