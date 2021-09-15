

#ifndef SGD_DRIVERS_WH_F_CRUISER_HPP_
#define SGD_DRIVERS_WH_F_CRUISER_HPP_

#include <chrono>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "sgd_msgs/msg/serial.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sgd_hardware_drivers
{

class WH_Fcruiser : public nav2_util::LifecycleNode
{

#define RIGHT_WHEEL_FACTOR 57.93
#define LEFT_WHEEL_FACTOR 59.00

public:
  WH_Fcruiser();
  ~WH_Fcruiser();

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
  std::string msg_regex_;
  std::string odom_topic_;
  std::string battery_state_topic_;
  std::string vel_twist_topic_;
  double wheel_separation_;
  double wheel_circum_;

  //! \brief Init publisher and subscriber
  void init_pub_sub();
  std::regex regex_;
  double batt_volt_ = 0.0;
  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Serial>::SharedPtr pub_motor_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_motor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_data_;

  double pose_orie_z;
  nav_msgs::msg::Odometry last_odom_msg_;
  
  void publish_battery_state(double voltage);
  void on_motor_received(const sgd_msgs::msg::Serial::SharedPtr msg);
  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg);
};

}   // namespace sgd_hardware_drivers

#endif