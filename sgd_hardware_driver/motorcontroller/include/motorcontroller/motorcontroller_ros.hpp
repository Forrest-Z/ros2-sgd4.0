#ifndef SGD_HARDWARE_DRIVERS__WH_F_CRUISER_HPP_
#define SGD_HARDWARE_DRIVERS__WH_F_CRUISER_HPP_

#include <chrono>
#include <cstdarg>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sgd_io/serial.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "sgd_util/log_utils.hpp"
#include "sgd_msgs/msg/odom_improved.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "wh_fcruiser.hpp"
#include "sgd_odometry.h"

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
  std::string odom_topic_, odom_topic_improved_, odom_sim_topic_;
  std::string battery_state_topic_;
  std::string vel_twist_topic_, imu_topic_, gps_sim_topic_;
  bool is_sim_;
  bool is_relative_;
  double wheel_separation_;
  double wheel_circum_;
  float sim_battery_state_;

  double initial_orientation_ = 0.0;
  std::pair<double, double> initial_pos_ = std::pair<double, double>(0.0,0.0);
  int imu_msgs_rec_ = 0, gps_msgs_rec_ = 0;
  double tmp_init_ori_ = 0.0;

  //! \brief Init publisher and subscriber
  void init_pub_sub();
  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_serial_;
  rclcpp::TimerBase::SharedPtr timer_vol_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::OdomImproved>::SharedPtr pub_odom_improved_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_motor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_sim_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr sub_gps_;

  std::unique_ptr<WH_FCruiser> wh_fcruiser;
  SGD_Odometry odo_new;
  
  void publish_battery_state();
  void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_gps_received(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg);
  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_odom_sim_received(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_motor_received(const std_msgs::msg::UInt32::SharedPtr msg);
  double cmd_vel_seconds_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  void publish_motor_cmd();

  /**
     * @brief Class to read from / write to serial port.
     */
    sgd_io::Serial serial;

  void read_serial();

  std::string log_msg(double value...);
};

}   // namespace sgd_hardware_drivers

#endif