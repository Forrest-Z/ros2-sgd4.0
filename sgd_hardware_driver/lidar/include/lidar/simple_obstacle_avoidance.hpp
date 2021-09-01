#ifndef SGD_SENSORS_SIMPLE_OBSTACLE_AVOIDANCE_HPP_
#define SGD_SENSORS_SIMPLE_OBSTACLE_AVOIDANCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace sgd_sensors
{

class Simple_Obstacle_Avoidance : public nav2_util::LifecycleNode
{
public:
    Simple_Obstacle_Avoidance();
    ~Simple_Obstacle_Avoidance();

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    void init_parameters();
    std::string example_param;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;

    void on_lidar_received(const sensor_msgs::msg::LaserScan::SharedPtr msg);

};

}

#endif
