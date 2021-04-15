#ifndef LASER_1D_HPP_
#define LASER_1D_HPP_

#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sgd_msgs/msg/serial.hpp"

namespace sgd_sensors
{

class Laser_1D : public rclcpp::Node
{
private:
    std::regex regex_;

    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;

    void on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg_);

public:
    Laser_1D(/* args */);
    ~Laser_1D();
};

}   // namespace sgd_sensors



#endif