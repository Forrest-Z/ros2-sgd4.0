

#ifndef WH_F_CRUISER_HPP_
#define WH_F_CRUISER_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "sgd_msgs/msg/serial.hpp"

namespace sgd_motor
{

class WH_Fcruiser : public rclcpp::Node
{
public:
  WH_Fcruiser();
  ~WH_Fcruiser();

protected:
  int message_count_;

  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::Publisher<sgd_msgs::msg::Serial>::SharedPtr pub_motor_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_motordata();
};

}   // namespace sgd_motor

#endif