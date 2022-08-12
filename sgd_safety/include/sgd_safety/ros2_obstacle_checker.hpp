#ifndef SGD_SAFETY__ROS_OBSTACLE_CHECKER_HPP_
#define SGD_SAFETY__ROS_OBSTACLE_CHECKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "sgd_safety/obstacle_checker.hpp"

namespace sgd_safety
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Uses Lidar measurements to determine the speed of the robot according to the obstacles of the environment.
     * It also determines if there is enough space for the robot to continue or if it should stop to avoid collision. 
 */
class Ros2_Obstacle_Checker : public rclcpp_lifecycle::LifecycleNode
{

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    std::string scan_topic_;
    std::string cmd_vel_contr_topic_;
    std::string cmd_vel_topic_;
    double robot_width_;
    double distance_min_;
    double distance_max_;

    ObstacleChecker oc;
    geometry_msgs::msg::Twist last_cmd_vel_;
    double cmd_vel_time;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_contr_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    void on_cmd_vel_contr_received(const geometry_msgs::msg::Twist::SharedPtr msg);

   
    void on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg);
public:
    Ros2_Obstacle_Checker();
    ~Ros2_Obstacle_Checker();
};

}   // namespace sgd_safety

#endif
