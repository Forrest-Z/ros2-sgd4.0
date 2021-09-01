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

#include "nav2_util/lifecycle_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <stack>

#include <termios.h>

#include <unistd.h>

#include "sgd_msgs/msg/serial.hpp"

namespace sgd_util
{

class Serial : public nav2_util::LifecycleNode
{

enum RECEIVER_STATUS {
    INITIAL,
    ESTABLISH,
    AUTHENTIFICATE,
    WAIT_HEADER,
    IN_MSG,
    AFTER_ESC,
    ERROR,
    STOPPED = -1
};

public:
    Serial();
    ~Serial();

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
    int baud_rate_;
    std::string read_write_;
    bool raw_;  // set raw mode
    char sframe_, stframe_;   // start / stop frame
    bool log_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_serial;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Serial>::SharedPtr pub_serial;
    rclcpp::TimerBase::SharedPtr timer_;

    int serial_port_;
    std::string logfile_;
    std::string read_buf_;
    long time_at_start_;        // start time in millis
    std::fstream file_;

    //! \brief Open serial port and configure for further operations.
    //! \param port port specifier
    //! \param baud baud-rate
    RECEIVER_STATUS open_port(const char *port, const int baud);
    RECEIVER_STATUS set_config();
    RECEIVER_STATUS close_port();
    void read_serial();
    void write_serial(const sgd_msgs::msg::Serial::SharedPtr msg_);

    //! \brief 
    RECEIVER_STATUS rec_state_;
    const char ESC_CHAR = '#';      // escape character
};

}

#endif
