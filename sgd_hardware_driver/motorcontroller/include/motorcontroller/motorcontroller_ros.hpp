#ifndef SGD_HARDWARE_DRIVERS__WH_F_CRUISER_HPP_
#define SGD_HARDWARE_DRIVERS__WH_F_CRUISER_HPP_

#include <chrono>
#include <cstdarg>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "std_msgs/msg/u_int32.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include "wh_fcruiser.hpp"
#include "sgd_odometry.h"

namespace sgd_hardware_drivers
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Motorcontroller : public rclcpp_lifecycle::LifecycleNode
{

public:
    Motorcontroller();
    ~Motorcontroller();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    //! \brief Init parameters
    void init_parameters();
    bool is_sim_;
    bool is_relative_;
    float sim_battery_voltage_;

    std::pair<double, double> initial_pos_ = std::pair<double, double>(0.0, 0.0);

    //! \brief Init publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;                 /// @brief odometry publisher
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_;       /// @brief battery state publisher
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt32>::SharedPtr pub_motor_;                  /// @brief publisher for motorcontroller hardware

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_motor_;                                  /// @brief subscription to motorcontroller hardware
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_sim_;                             /// @brief subscription to simulated odometry data
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;                            /// @brief subscription to move commands from ros system
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;                                    /// @brief subscription to imu data
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr sub_gps_;                   /// @brief subscription to gps position information

    std::unique_ptr<WH_FCruiser> wh_fcruiser;
    SGD_Odometry odo_new;

    /**
     * @brief Publish the battery voltage
     *
     */
    void publish_battery_state();

    /**
     * @brief Called when a new imu message was received
     *
     * @param msg
     */
    void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Called when a new gps message is received
     *
     * @param msg
     */
    void on_gps_received(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg);

    /**
     * @brief Called when a new move command from the system is received
     *
     * @param msg
     */
    void on_sgd_move_received(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Called when a new odom message is received from the simulation.
     * Only active in simulation mode (use_sime_time=true)
     *
     * @param msg
     */
    void on_odom_sim_received(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Called when a new message from the motorcontroller hardware is received.
     *
     * @param msg
     */
    void on_motor_received(const std_msgs::msg::UInt32::SharedPtr msg);
    double cmd_vel_seconds_;
    geometry_msgs::msg::Twist last_cmd_vel_;

    /**
     * @brief Publish a command to the motorcontroller hardware
     *
     */
    void publish_motor_cmd();

    /**
     * @brief Class to read from / write to serial port.
     */
    // sgd_io::Serial serial;

    // void read_serial();
};

} // namespace sgd_hardware_drivers

#endif