#ifndef LOGGER_NODE_HPP_
#define LOGGER_NODE_HPP_

#include <chrono>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

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
  //rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_gps_;
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  //! \brief Init transforms
    void init_transforms();
    void wait_for_transform();
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;

  double time_at_start_;
  std::string scan_filename_;
  void imu_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_);
  //void gps_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg_);
  //void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_);

  std::string time_to_string();
  std::string stamp_to_string(std_msgs::msg::Header header);
  std::string to_string(double time_s, std::string format="%.3f");
  std::string vec3_to_string(geometry_msgs::msg::Vector3 vec3);
};

}   // namespace sgd_motor

#endif