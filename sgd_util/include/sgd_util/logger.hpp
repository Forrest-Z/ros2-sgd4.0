
#ifndef LOGGER_NODE_HPP_
#define LOGGER_NODE_HPP_

#include <chrono>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace sgd_util
{

class Logger : public rclcpp::Node
{
public:
  Logger();
  ~Logger();

protected:
  //! \brief Init parameters
  std::string imu_topic_;
  std::string odom_topic_;
  std::string gps_topic_;
  std::string output_folder_;

  std::fstream out_imu_;
  std::fstream out_gps_;
  std::fstream out_odom_;

  //! \brief Init publisher and subscriber
  void init_pub_sub();
  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  double time_at_start_;
  std::string scan_filename_;
  void imu_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg_);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_);

  std::string time_to_string();
  std::string vec3_to_string(geometry_msgs::msg::Vector3 vec3);
};

}   // namespace sgd_motor

#endif