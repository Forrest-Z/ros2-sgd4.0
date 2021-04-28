#ifndef LASER_1D_HPP_
#define LASER_1D_HPP_

#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/msg/range.hpp"
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

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;

    std::regex regex_;
    void on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg_);
};

}

#endif
