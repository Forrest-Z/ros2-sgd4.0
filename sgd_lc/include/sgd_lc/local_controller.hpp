
#ifndef LOCAL_CONTROLLER_HPP_
#define LOCAL_CONTROLLER_HPP_

#include <math.h>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sgd_msgs/msg/touch.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "sgd_lc/pid_controller.hpp"

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

    bool ende = false;
    double last_sec, last_nsec;

    // Data import and filter
    int filter_i_;
    std::vector<double> filter_speed;
    std::vector<double> filter_steer;

    // regler data
    std::deque<float> poses_;
    float ref_x_, ref_y_, ref_w_;       // Sollposition
    float curr_x_, curr_y_, curr_w_;    // aktuelle Position


    // 1 x publisher, 1 x subscriber
    // 1x service
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_motor_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_motor_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_laser_;
    rclcpp::Subscription<sgd_msgs::msg::Touch>::SharedPtr sub_touch_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_motordata();
    void on_motor_received(const geometry_msgs::msg::TwistStamped::SharedPtr msg_);
    void on_laser_received(const sensor_msgs::msg::Range::SharedPtr msg_);
    void on_touch_received(const sgd_msgs::msg::Touch::SharedPtr msg_);

    double distance(float x1,float y1,float x2,float y2);

    double controller_speed();
    inline int sig(double x);

    std::shared_ptr<PID_Controller> speed_controller_;
    std::shared_ptr<PID_Controller> turn_controller_;

public:
    Local_Controller();
    ~Local_Controller();
};



}   // namespace sgd_lc



#endif