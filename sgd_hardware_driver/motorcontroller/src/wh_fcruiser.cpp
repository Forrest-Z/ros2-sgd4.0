#include "motorcontroller/wh_fcruiser.hpp"

namespace sgd_hardware_drivers
{

WH_FCruiser::WH_FCruiser(double wheel_circ, double wheel_sep)
    : wheel_circ_(wheel_circ), wheel_sep_(wheel_sep) {}

WH_FCruiser::~WH_FCruiser() {}

void
WH_FCruiser::parse_msg(uint32_t msg)
{
    meas_R = (int16_t)(msg & 0xFFFF)-1000;
    meas_L = (int16_t)(msg >> 16)-1000;

    // calculate current position
    float meas_R_ = (float)meas_R / FACTOR_LIN_TO_WHEEL;
    float meas_L_ = (float)meas_L / FACTOR_LIN_TO_WHEEL;

    vel_lin_x_ = (meas_R_ + meas_L_) / 2;
    vel_ang_z_ = (meas_R_ - meas_L_) / wheel_sep_;
    
    // set orientation
    // ori_z_ += vel_ang_z_ * (tmp_time_ - millis_) / 1000.0;
    // if (abs(ori_z_) > 2*M_PI)
    // {
    //     ori_z_ -= ori_z_ / abs(ori_z_) * 2*M_PI;
    // }

    // pos_x_ = pos_x_ + vel_lin_x_ * (tmp_time_ - millis_) / 1000.0 * cos(ori_z_);
    // pos_y_ = pos_y_ - vel_lin_x_ * (tmp_time_ - millis_) / 1000.0 * sin(ori_z_);

    // millis_ = tmp_time_;
}

void
WH_FCruiser::get_position(double &pos_x, double &pos_y)
{
    pos_x = pos_x_;
    pos_y = pos_y_;
}

void
WH_FCruiser::set_initial_position(double x, double y)
{
    pos_x_ = x;
    pos_y_ = y;
}

double
WH_FCruiser::get_orientation()
{
    return ori_z_;
}

void
WH_FCruiser::set_initial_orientation(double ang)
{
    ori_z_ = ang;
}

double
WH_FCruiser::get_linear_vel()
{
    return vel_lin_x_;
}

double
WH_FCruiser::get_angular_vel()
{
    return vel_ang_z_;
}

float
WH_FCruiser::get_batt_voltage()
{
    return volt_;
}

float
WH_FCruiser::get_temp()
{
    return temp_;
}

std::string
WH_FCruiser::cmd_vel(double vel_lin_x, double vel_ang_z)
{
    // max motor speed 300
    double r = (vel_ang_z*wheel_sep_/2+vel_lin_x);
    double l = (r-vel_ang_z*wheel_sep_);

    return std::to_string((int)round(r*FACTOR_LIN_TO_WHEEL)) + "," + std::to_string((int)round(l*FACTOR_LIN_TO_WHEEL));
}

ulong
WH_FCruiser::millis()
{
    return millis_;
}


} // namespace sgd_hardware_drivers
