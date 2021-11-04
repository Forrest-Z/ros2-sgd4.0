#ifndef SGD_HARDWARE__FRSKY_RX8R_HPP_
#define SGD_HARDWARE__FRSKY_RX8R_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sgd_msgs/msg/serial.hpp"
#include "sgd_msgs/msg/light.hpp"

namespace sgd_hardware_drivers 
{

class FrSky_RX8R : public nav2_util::LifecycleNode
{
public:
    FrSky_RX8R();
    ~FrSky_RX8R();

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
    double max_vel_;
    double max_rot_vel_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publish_cmd_vel_master_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publish_cmd_vel_;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Light>::SharedPtr publish_light_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;

    void on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg);
    // Publish movement 
    void pub_cmd_vel(int ch1, int ch2, int ch8);
    bool last_msg_equ_zero_;
    void pub_light(int ch3, int ch6);
    int lights_l_ = -1, lights_r_ = -1;
};

}

#endif
