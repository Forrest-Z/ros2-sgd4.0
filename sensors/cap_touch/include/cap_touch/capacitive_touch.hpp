#ifndef CAPACITIVE_TOUCH_HPP_
#define CAPACITIVE_TOUCH_HPP_

#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "sgd_msgs/msg/serial.hpp"
#include "sgd_msgs/msg/touch.hpp"

namespace sgd_sensors
{

class Capacitive_Touch : public rclcpp::Node
{
private:
    std::regex regex_;
    std::vector<int> fvalues;
    int filter_i_, thresh;

    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Publisher<sgd_msgs::msg::Touch>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;

    void on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg);
    int filter(int new_value);
public:
    Capacitive_Touch(/* args */);
    ~Capacitive_Touch();
};

}   // namespace sgd_sensors

#endif