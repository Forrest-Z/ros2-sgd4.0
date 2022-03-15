#ifndef SGD_HARDWARE__FRSKY_RX8R_HPP_
#define SGD_HARDWARE__FRSKY_RX8R_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sgd_msgs/msg/light.hpp"
#include "sgd_io/serial.hpp"

namespace sgd_hardware_drivers 
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FrSky_RX8R : public rclcpp_lifecycle::LifecycleNode
{
public:
    FrSky_RX8R();
    ~FrSky_RX8R();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    void init_parameters();
    std::string port_;
    double max_vel_;
    double max_rot_vel_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publish_cmd_vel_master_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publish_cmd_vel_;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Light>::SharedPtr publish_light_;

    sgd_io::Serial serial;
    void read_serial();
    //void on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg);
    
    /**
     * @brief Publish a velocity command
     * 
     * @param ch1 velocity
     * @param ch2 rotation
     * @param ch8 master switch
     */
    void pub_cmd_vel(int ch1, int ch2, int ch8);
    bool last_msg_equ_zero_;

    /**
     * @brief Publish a command to control the led strips.
     * 
     * @param ch3 the command for left led strip
     * @param ch6 the command for right led strip
     */
    void pub_light(int ch3, int ch6);
    int lights_l_ = -1, lights_r_ = -1;
};

}   // namespace sgd_hardware_drivers 

#endif
