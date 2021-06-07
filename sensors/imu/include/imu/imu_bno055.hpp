#ifndef EXAMPLE_NODE_HPP_
#define EXAMPLE_NODE_HPP_

#include <regex>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "sgd_msgs/msg/serial.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sgd_sensors
{

class IMU_BNO055 : public nav2_util::LifecycleNode
{

struct Vector3
{
    double x,y,z;
    Vector3() : x(0), y(0), z(0) { };
    Vector3(double x_, double y_, double z_)
        : x(x_), y(y_), z(z_) { };
};

struct Euler
{
    double x,y,z;
};

public:
    IMU_BNO055();
    ~IMU_BNO055();

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
    bool config_mode_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr serial_sub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;

    void on_serial_received(sgd_msgs::msg::Serial::SharedPtr msg);
    std::regex time_regex_;
    std::regex data_regex_;
    std::regex double_regex_;
    u_int8_t gyr_calibration;
    u_int8_t acc_calibration;
    u_int8_t hea_calibration;
    bool system_calibrated = false;
    rclcpp::Time last_calib_msg_;

    //! \brief Convert string to vector. String must contain comma separated values.
    //! \returns True if conversion was successful, false if sensor is not 
    geometry_msgs::msg::Vector3 data_to_vector(std::string data);
    geometry_msgs::msg::Quaternion data_to_quat(std::string data);

    std::array<double, 9UL> calc_cov_matrix(Vector3 meas[], Vector3 mean);
    Vector3 quat_to_euler(geometry_msgs::msg::Quaternion q);
    std::array<double, 9UL> cov_acc;   // acceleration
    std::array<double, 9UL> cov_hea;   // heading
    std::array<double, 9UL> cov_gyr;   // gyroscope
    int cov_count;
    Vector3 meas_acc[100];
    Vector3 meas_hea[100];
    Vector3 meas_gyr[100];
    Vector3 mean_acc, mean_hea, mean_gyr;
    const int max_cov_count = 100;
};

}

#endif
