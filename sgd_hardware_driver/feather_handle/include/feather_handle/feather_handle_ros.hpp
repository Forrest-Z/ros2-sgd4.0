#ifndef SGD_HARDWARE_DRIVERS__IMU_ROS_NODE_HPP_
#define SGD_HARDWARE_DRIVERS__IMU_ROS_NODE_HPP_

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "sgd_io/serial.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "bno055.hpp"
#include "vl53l1x.hpp"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Feather_Handle_ROS : public rclcpp_lifecycle::LifecycleNode
{

public:
    Feather_Handle_ROS();
    ~Feather_Handle_ROS();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    // classes for sensors
    std::unique_ptr<BNO055> bno055_;
    std::unique_ptr<VL53L1X> vl53l1x_;
    
    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr laser_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_scan_orientation_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_sim_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr vis_compass_pub_;
    // rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr vis_laser1d_pub_;

    bool is_sim_;
    double cmd_laser_time_;
    bool visualize_compass = false;
    bool visualize_laser1d = false;
    float scan_yaw_diff_ = 0.0;

    void publish_imu();
    void publish_imu_visual(sensor_msgs::msg::Imu::SharedPtr msg);
    void on_cmd_vel_received(geometry_msgs::msg::Twist::SharedPtr msg_);
    void on_scan_orient_received(geometry_msgs::msg::Quaternion::SharedPtr msg_);

    sgd_io::Serial serial;
    void read_serial();
};

}   // namespace sgd_hardware_drivers

#endif
