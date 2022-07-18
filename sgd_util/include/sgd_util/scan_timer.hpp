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

#ifndef SGD_UTILS__SCAN_TIMER_HPP_
#define SGD_UTILS__SCAN_TIMER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace sgd_utils
{

class Scan_Timer : public rclcpp::Node
{

public:
    Scan_Timer();
    ~Scan_Timer();

protected:
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    void on_scan_received(sensor_msgs::msg::LaserScan::SharedPtr msg);
};

}

#endif  // SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
