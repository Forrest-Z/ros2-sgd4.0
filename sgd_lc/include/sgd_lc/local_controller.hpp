
#ifndef LOCAL_CONTROLLER_HPP_
#define LOCAL_CONTROLLER_HPP_

#include <math.h>

#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sgd_msgs/msg/serial.hpp"
#include "sgd_msgs/msg/touch.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace sgd_lc
{

class Local_Controller : public rclcpp::Node
{

private:
    /* data */
    const int16_t MAX_SPEED = 160;
    double speed_;

    bool last_handle_ = false;
    double last_range_ = 100;

    // 1 x publisher, 1 x subscriber
    // 1x service
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Publisher<sgd_msgs::msg::Serial>::SharedPtr pub_motor_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_motor_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_laser_;
    rclcpp::Subscription<sgd_msgs::msg::Touch>::SharedPtr sub_touch_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_motordata();
    void print(const sgd_msgs::msg::Serial::SharedPtr msg_);
    void on_laser_received(const sensor_msgs::msg::Range::SharedPtr msg_);
    void on_touch_received(const sgd_msgs::msg::Touch::SharedPtr msg_);

public:
    Local_Controller();
    ~Local_Controller();
};



}   // namespace sgd_lc



#endif