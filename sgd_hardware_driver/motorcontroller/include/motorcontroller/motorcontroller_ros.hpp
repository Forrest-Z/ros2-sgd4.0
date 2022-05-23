#ifndef SGD_HARDWARE_DRIVERS__WH_F_CRUISER_HPP_
#define SGD_HARDWARE_DRIVERS__WH_F_CRUISER_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sgd_io/serial.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

#include "wh_fcruiser.hpp"

namespace sgd_hardware_drivers
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Motorcontroller : public rclcpp_lifecycle::LifecycleNode
{

public:
  Motorcontroller();
  ~Motorcontroller();

protected:
  // Implement the lifecycle interface
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  //! \brief Init parameters
  void init_parameters();
  std::string port_;
  std::string msg_regex_;
  std::string odom_topic_;
  std::string battery_state_topic_;
  std::string vel_twist_topic_;
  std::string imu_topic_;
  double wheel_separation_;
  double wheel_circum_;
  double initial_orientation_ = 0.0;
  int imu_msgs_rec_ = 0;
  double tmp_init_ori_ = 0.0;

  //! \brief Init publisher and subscriber
  void init_pub_sub();
  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_serial_;
  rclcpp::TimerBase::SharedPtr timer_vol_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  std::unique_ptr<WH_FCruiser> wh_fcruiser;
  
  void publish_battery_state();
  void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg);
  double cmd_vel_seconds_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  void publish_motor_cmd();

  /**
     * @brief Class to read from / write to serial port.
     */
    sgd_io::Serial serial;

  void read_serial();
};

}   // namespace sgd_hardware_drivers

#endif