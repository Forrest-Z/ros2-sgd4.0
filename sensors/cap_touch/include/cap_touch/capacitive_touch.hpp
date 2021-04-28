#ifndef CAPACITIVE_TOUCH_HPP_
#define CAPACITIVE_TOUCH_HPP_

#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sgd_msgs/msg/serial.hpp"
#include "sgd_msgs/msg/touch.hpp" 

namespace sgd_sensors 
{

class Capacitive_Touch : public nav2_util::LifecycleNode
{
public:
    Capacitive_Touch();
    ~Capacitive_Touch();

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
    int thresh_;
    int filter_i_;
    std::string msg_regex_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Touch>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;

    std::regex regex_;
    std::vector<int> fvalues;
    void on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg);
    int filter(int new_value);
};

}

#endif
