#ifndef SGD_HARDWARE__TMCM1638_HPP_
#define SGD_HARDWARE__TMCM1638_HPP_

#include <iostream>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace sgd_hardware_drivers
{

class TMCM1638
{
private:
    ulong millis_ = 0;
    float volt_ = 37.0;
    float temp_ = 20.0;

    double vel_lin_x_ = 0.0, vel_ang_z_ = 0.0;
    double pos_x_ = 0.0, pos_y_ = 0.0, ori_z_ = 0.0;

    // parameters
    double wheel_circ_;     // wheel circumference
    double wheel_sep_;      // wheel separation

public:
    double meas_L = 0, meas_R = 0;

    /**
     * @brief Construct a new wh fcruiser object
     * 
     * @param wheel_circ 
     * @param wheel_sep 
     */
    TMCM1638(double wheel_circ, double wheel_sep);

    /**
     * @brief Destroy the wh fcruiser object
     * 
     */
    ~TMCM1638();

    /**
     * @brief Parse the json message
     * 
     * @param msg 
     */
    void parse_msg(geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Get the position relative to the start position
     * 
     * @param pos_x 
     * @param pos_y 
     */
    void get_position(double &pos_x, double &pos_y);

    /**
     * @brief Set the initial position object
     * 
     * @param x 
     * @param y 
     */
    void set_initial_position(double x, double y);

    /**
     * @brief Get the relative orientation
     * 
     * @return double 
     */
    double get_orientation();

    /**
     * @brief Set the initial orientation
     * 
     * @param ang 
     */
    void set_initial_orientation(double ang);

    /**
     * @brief Get the linear velocity
     * 
     * @return double 
     */
    double get_linear_vel();

    /**
     * @brief Get the angular velocity
     * 
     * @return double 
     */
    double get_angular_vel();

    /**
     * @brief Get the battery voltage
     * 
     * @return float 
     */
    float get_batt_voltage();

    /**
     * @brief Get the measured temperature of the controller board
     * 
     * @return float 
     */
    float get_temp();

    /**
     * @brief Translates the ROS cmd_vel message to the controller specific message
     * that can be send to the serial port
     * 
     * @param vel_lin_x linear velocity in x direction
     * @param vel_ang_z angular velocity around z axis
     * @return std::string the message
     */
    geometry_msgs::msg::Quaternion cmd_vel(double vel_lin_x, double vel_ang_z);

    /**
     * @brief Returns millis from last json message
     * 
     * @return ulong 
     */
    ulong millis();
};

} // namespace sgd_hardware_drivers

#endif  // SGD_HARDWARE__TMCM1638_HPP_