#ifndef LED_NODE_HPP_
#define LED_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sgd_msgs/msg/serial.hpp"
#include "sgd_msgs/msg/light.hpp"


namespace sgd_hardware_drivers
{
    class LED_Strip : public nav2_util::LifecycleNode
    {
    public:
        LED_Strip();
        ~LED_Strip();

    protected:
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

        void init_parameters();
        std::string port_;
        std::string led_;

        void init_pub_sub();
        rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Serial>::SharedPtr publisher_;
        rclcpp::Subscription<sgd_msgs::msg::Light>::SharedPtr subscriber_;

        void on_msg_received(const sgd_msgs::msg::Light::SharedPtr msg);

        sgd_msgs::msg::Serial compute_msg(const sgd_msgs::msg::Light::SharedPtr msg);
        
    };
}

#endif