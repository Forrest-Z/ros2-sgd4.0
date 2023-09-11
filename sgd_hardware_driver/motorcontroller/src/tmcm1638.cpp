#include "motorcontroller/tmcm1638.hpp"

namespace sgd_hardware_drivers
{

TMCM1638::TMCM1638(double wheel_circ, double wheel_sep)
    : wheel_circ_(wheel_circ), wheel_sep_(wheel_sep) {}

TMCM1638::~TMCM1638() {}

void
TMCM1638::parse_msg(geometry_msgs::msg::Twist::SharedPtr msg)
{

    // msg->linear.x = velocity (rpm) of left motor
    // msg->angular.x = velocity (rpm) of right motor
    // convert rpm to m/s
    meas_R = -msg->angular.x * wheel_circ_ / 60.0;
    meas_L = -msg->linear.x * wheel_circ_ / 60.0;

    vel_lin_x_ = (meas_R + meas_L) / 2;
    vel_ang_z_ = (meas_R - meas_L) / wheel_sep_;
    
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
TMCM1638::get_position(double &pos_x, double &pos_y)
{
    pos_x = pos_x_;
    pos_y = pos_y_;
}

void
TMCM1638::set_initial_position(double x, double y)
{
    pos_x_ = x;
    pos_y_ = y;
}

double
TMCM1638::get_orientation()
{
    return ori_z_;
}

void
TMCM1638::set_initial_orientation(double ang)
{
    ori_z_ = ang;
}

double
TMCM1638::get_linear_vel()
{
    return vel_lin_x_;
}

double
TMCM1638::get_angular_vel()
{
    return vel_ang_z_;
}

float
TMCM1638::get_batt_voltage()
{
    return volt_;
}

float
TMCM1638::get_temp()
{
    return temp_;
}

geometry_msgs::msg::Quaternion
TMCM1638::cmd_vel(double vel_lin_x, double vel_ang_z)
{
    // calculate left and right wheel m/s
    double r = (vel_ang_z*wheel_sep_/2+vel_lin_x);
    double l = (r-vel_ang_z*wheel_sep_);

    // convert m/s to rpm
    geometry_msgs::msg::Quaternion q;
    q.y = r / wheel_circ_ * 60;   // right motor
    q.x = l / wheel_circ_ * 60;     // left motor

    return q;
}

ulong
TMCM1638::millis()
{
    return millis_;
}


} // namespace sgd_hardware_drivers
