
#ifndef NAV_SGD_OBSTACLE_CHECKER_HPP_
#define NAV_SGD_OBSTACLE_CHECKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav_sgd
{

class Lidar_Obstacle_Checker : public nav2_util::LifecycleNode
{

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    std::string scan_topic_;
    std::string cmd_vel_contr_topic_;
    std::string cmd_vel_topic_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_contr_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    void on_cmd_vel_contr_received(const geometry_msgs::msg::Twist::SharedPtr msg);
    void on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    geometry_msgs::msg::Twist last_cmd_vel_;
public:
    Lidar_Obstacle_Checker();
    ~Lidar_Obstacle_Checker();
};



}   // namespace sgd_lc



#endif