

#ifndef NAVILOCK_UBLOX6_GPS_HPP_
#define NAVILOCK_UBLOX6_GPS_HPP_

#include <functional>
#include <memory>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/nav_sat_fix.hpp"
//#include "geometry_msgs/msg/point_stamped.hpp"
//#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sensor_gps
{

class Navilock_UBlox6_GPS : public rclcpp::Node
{
public:

  Navilock_UBlox6_GPS();
  ~Navilock_UBlox6_GPS();

protected:

  // Implement the lifecycle interface -> TODO
  //nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  //nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  //nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  //nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  //nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Publisher 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int const baud_rate_;
  char const port_[80];
  int serial_port_;
  bool isPortOpen_;
  struct termios tty_;

  int init();
  void readLine();
  void timerCallback();
};

}   // namespace sensor_gps

#endif // NAVILOCK_UBLOX6_GPS_HPP_