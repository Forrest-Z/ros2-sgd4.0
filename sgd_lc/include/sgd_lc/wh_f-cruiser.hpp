

#ifndef WH_F_CRUISER_HPP_
#define WH_F_CRUISER_HPP_

#include <chrono>
#include <regex>

#include <rclcpp/rclcpp.hpp>
#include "sgd_msgs/msg/serial.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "sgd_lc/pid_controller.hpp"

namespace sgd_lc
{

class WH_Fcruiser : public rclcpp::Node
{
public:
  WH_Fcruiser();
  ~WH_Fcruiser();

protected:
  int message_count_;
  double meas_r_, meas_l_;
  double set_r_speed_, set_l_speed_;

  std::regex regex_;

  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::Publisher<sgd_msgs::msg::Serial>::SharedPtr pub_motor_;
  rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_motor_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_data_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_data_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<PID_Controller> wheel_r_controller_;
  std::shared_ptr<PID_Controller> wheel_l_controller_;

  void publish_motordata();
  void on_motor_received(const sgd_msgs::msg::Serial::SharedPtr msg);
  void on_data_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

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