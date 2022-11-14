#ifndef SGD_HARDWARE__WH_FCRUISER_HPP_
#define SGD_HARDWARE__WH_FCRUISER_HPP_

#include <nlohmann/json.hpp>
#include <iostream>

namespace sgd_hardware_drivers
{

#define FACTOR_LIN_TO_WHEEL 74.00681

class WH_FCruiser
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
    int16_t meas_L = 0, meas_R = 0;

    /**
     * @brief Construct a new wh fcruiser object
     * 
     * @param wheel_circ 
     * @param wheel_sep 
     */
    WH_FCruiser(double wheel_circ, double wheel_sep);

    /**
     * @brief Destroy the wh fcruiser object
     * 
     */
    ~WH_FCruiser();

    /**
     * @brief Parse the json message
     * 
     * @param msg 
     */
    void parse_msg(uint32_t msg);

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
    std::string cmd_vel(double vel_lin_x, double vel_ang_z);

    /**
     * @brief Returns millis from last json message
     * 
     * @return ulong 
     */
    ulong millis();
};

} // namespace sgd_hardware_drivers

#endif