// Copyright 2021 HAW Hamburg
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

#ifndef GPS__NAVILOCK_UBLOX6_GPS_HPP_
#define GPS__NAVILOCK_UBLOX6_GPS_HPP_

#include <list>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sgd_util/geotools.hpp"
#include "sgd_io/serial.hpp"

#include "include/nmea_parser.hpp"
#include "include/ubx_parser.hpp"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Navilock_UBlox6_GPS : public rclcpp_lifecycle::LifecycleNode
{

public: 
    Navilock_UBlox6_GPS();
    ~Navilock_UBlox6_GPS();

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
    std::string xml_file_;
    std::string parser_type_;
    bool is_pub_local_pose_;

    /**
     * @brief Initialize publisher
     */
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_navsatfix_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_local_pose_;

    /**
     * @brief Initialize transforms
     */
    void init_transforms();
    sgd_util::LatLon map_origin;
    
    sgd_io::Serial serial;
    void read_serial();

    std::unique_ptr<IGPS_Message> parser_;
    //void on_serial_received(const sgd_msgs::msg::Serial::SharedPtr msg);
};

}       // namespace sgd_hardware

#endif  // GPS__NAVILOCK_UBLOX6_GPS_HPP_"