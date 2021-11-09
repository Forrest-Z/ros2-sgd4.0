#ifndef LASER_1D_HPP_
#define LASER_1D_HPP_

#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sgd_msgs/msg/serial.hpp"

namespace sgd_sensors
{
 
class Laser_1D : public nav2_util::LifecycleNode
{
public:
    Laser_1D();
    ~Laser_1D();

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
    double min_range_;
    double max_range_;
    int max_vel_percent_;
    double m_, b_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr sub_serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

    std::regex regex_;
    void on_serial_received(const sgd_msgs::msg::Serial::SharedPtr msg_);
    void on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg_);
    double vel_modifier_ = 1.0;

    inline double set_vel_modifier(double range)
    {
        return (range > max_range_ || range < min_range_) ? (range > max_range_) * max_vel_percent_ : range * m_ + b_;
    }
};

}

#endif
