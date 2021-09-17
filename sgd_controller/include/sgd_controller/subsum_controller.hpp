
#ifndef NAV_SGD_SUBSUM_CONTROLLER_HPP_
#define NAV_SGD_SUBSUM_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace nav_sgd
{

#define NUM_LAYERS 10

class Subsum_Controller : public nav2_util::LifecycleNode
{

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    void init_parameters();
    std::vector<std::string> layer_topics;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subscriber;

    // layer data
    int active_layer;
    double last_time_received[NUM_LAYERS];

    void on_cm_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg, int layer);

public:
    Subsum_Controller();
    ~Subsum_Controller();
};



}   // namespace sgd_lc



#endif