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
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sgd_msgs/msg/serial.hpp"

#include "include/nmea_parser.hpp"
#include "include/ubx_parser.hpp"

namespace sgd_hardware
{

class Navilock_UBlox6_GPS : public nav2_util::LifecycleNode
{

public: 
    Navilock_UBlox6_GPS();
    ~Navilock_UBlox6_GPS();

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    
    //! \brief Init parameters
    void init_parameters();
    std::string port_;
    std::string xml_file_;
    std::string parser_type_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;
    
    std::shared_ptr<IGPS_Message> parser_;
    void on_serial_received(const sgd_msgs::msg::Serial::SharedPtr msg);
};

}       // namespace sgd_hardware

#endif  // GPS__NAVILOCK_UBLOX6_GPS_HPP_"