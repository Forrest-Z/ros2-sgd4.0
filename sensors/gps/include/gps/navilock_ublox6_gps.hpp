

#ifndef NAVILOCK_UBLOX6_GPS_HPP_
#define NAVILOCK_UBLOX6_GPS_HPP_

#include <functional>
#include <filesystem>
#include <memory>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "gps/nmea_parser.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sgd_sensors
{

namespace fs = std::filesystem;

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
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

  fs::path last_file_;

  std::shared_ptr<Nmea_Parser> nmea_parser_;
  

  void read_msg();
  void timerCallback();
};

}   // namespace sgd_sensors

#endif // NAVILOCK_UBLOX6_GPS_HPP_