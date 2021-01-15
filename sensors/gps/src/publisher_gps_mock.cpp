// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class GPS_Mock : public rclcpp::Node
{
public:
  GPS_Mock()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("topic", 10);
    timer_ = this->create_wall_timer(
      3000ms, std::bind(&GPS_Mock::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto navsatfix = sensor_msgs::msg::NavSatFix();
    auto navsatstatus = sensor_msgs::msg::NavSatStatus();
    navsatstatus.status = 0;
    navsatstatus.service = 1;

    navsatfix.status = navsatstatus;
    navsatfix.latitude = 53.555414;
    navsatfix.longitude = 10.021725;
    navsatfix.altitude = 3.201;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", navsatfix.latitude, navsatfix.longitude);
    publisher_->publish(navsatfix);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPS_Mock>());
  rclcpp::shutdown();
  return 0;
}
