
#ifndef SERIAL_TO_ROS_HPP_
#define SERIAL_TO_ROS_HPP_

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>

#include <iostream>

#include <memory>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <stack>

#include "rclcpp/rclcpp.hpp"
#include <termios.h>

#include <unistd.h>

#include "sgd_msgs/msg/serial.hpp"
#include "sgd_msgs/srv/serial_write.hpp"

namespace sgd_util
{

class Serial : public rclcpp::Node
{

private:
    int serial_port_;
    std::string read_buf_;

    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_serial;
    rclcpp::Publisher<sgd_msgs::msg::Serial>::SharedPtr pub_serial;
    rclcpp::TimerBase::SharedPtr timer_;

    //rclcpp::Service<sgd_msgs::srv::SerialWrite>::SharedPtr ser_write_srv_;

    //! \brief Open serial port and configure for further operations.
    //! \param port port specifier
    //! \param baud baud-rate
    int init_serial(const char *port, const int baud);

    //! \brief Read line
    char read_line();

bool isPortOpen_;
public:
    Serial();
    ~Serial();

    void read_serial();
    void write_serial(const sgd_msgs::msg::Serial::SharedPtr msg_);

};

}

#endif