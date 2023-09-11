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

#include "sgd_util/scan_timer.hpp"

namespace sgd_utils
{

Scan_Timer::Scan_Timer():
    Node("scan_timer")
{
    pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", default_qos);
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan/hardware", default_qos,
        std::bind(&Scan_Timer::on_scan_received, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Scan_timer initialized...");
}

Scan_Timer::~Scan_Timer()
{
    // Destroy
}

void
Scan_Timer::on_scan_received(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "Publish scan");
    sensor_msgs::msg::LaserScan laser;
    laser = *msg;
    laser.header.stamp = now();
    for (int i = 0; i < laser.ranges.size(); i++)
    {
        if (laser.ranges.at(i) < 0.7)   laser.ranges.at(i) = laser.range_max;
    }

    pub_scan_->publish(laser);
}

}   // namespace sgd_utils

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_utils::Scan_Timer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
