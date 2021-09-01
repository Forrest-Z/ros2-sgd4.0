

#ifndef WH_F_CRUISER_HPP_
#define WH_F_CRUISER_HPP_

#include <chrono>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "sgd_msgs/msg/serial.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rosgraph_msgs/msg/clock.hpp"

#include "motorcontroller/pid_controller.hpp"

namespace nav_sgd
{

class WH_Fcruiser : public nav2_util::LifecycleNode
{
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
  double motor_kp_, motor_ki_;
  double max_speed_, max_accel_;

  //! \brief Init publisher and subscriber
  void init_pub_sub();
  std::regex regex_;
  double batt_volt_ = 0.0;
  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Serial>::SharedPtr pub_motor_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;
  rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_motor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_data_;

  //! \brief Init transforms
  void init_transforms();
  double wheel_sep_, wheel_cir_;
  rclcpp::Time msg_delta_t_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  //! \brief Init controller for wheels
  void init_controller();
  double meas_r_, meas_l_;
  std::shared_ptr<PID_Controller> wheel_r_controller_;
  std::shared_ptr<PID_Controller> wheel_l_controller_;
  
  double pose_orie_z;
  nav_msgs::msg::Odometry last_odom_msg_;

  void publish_motordata();
  void publish_battery_state(double voltage);
  void on_motor_received(const sgd_msgs::msg::Serial::SharedPtr msg);
  void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg);

  void set_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  inline int sig(double x);
};

inline int
WH_Fcruiser::sig(double x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}

}   // namespace sgd_motor

#endif