#include "motorcontroller/wh_fcruiser.hpp"

namespace sgd_hardware_drivers
{

WH_FCruiser::WH_FCruiser(double wheel_circ, double wheel_sep)
    : wheel_circ_(wheel_circ), wheel_sep_(wheel_sep) {}

WH_FCruiser::~WH_FCruiser() {}

void
WH_FCruiser::parse_msg(std::string msg)
{
    // msg: {"s_id": "hover", "time": 123, "Rm": 0, "Lm": 0, "volt": 0, "temp": 0}
    auto js = nlohmann::json::parse(msg);

    if (js["s_id"] == "hover")
    {
        // save values to temporary variable
        ulong tmp_time_;
        double tmp_rm_, tmp_lm_;

        if (js.count("time"))   tmp_time_ = js["time"].get<ulong>();
        if (js.count("Rm"))     tmp_rm_ = js["Rm"].get<int>();
        if (js.count("Lm"))     tmp_lm_ = js["Lm"].get<int>();
        if (js.count("volt"))   volt_ = (2 * volt_ + js["volt"].get<int>()/100.0) / 3.0;
        if (js.count("temp"))   temp_ = js["temp"].get<int>()/10.0;
        
        if (millis_ == 0)   millis_ = tmp_time_;

        // calculate current position
        tmp_rm_ /= FACTOR_LIN_TO_WHEEL;
        tmp_lm_ /= FACTOR_LIN_TO_WHEEL;

        vel_lin_x_ = (tmp_rm_ + tmp_lm_) / 2;
        vel_ang_z_ = (tmp_rm_ - tmp_lm_) / wheel_sep_;
        
        // set orientation
        ori_z_ += vel_ang_z_ * (tmp_time_ - millis_) / 1000.0;
        if (abs(ori_z_) > 2*M_PI)
        {
            ori_z_ -= ori_z_ / abs(ori_z_) * 2*M_PI;
        }

        pos_x_ = pos_x_ + vel_lin_x_ * (tmp_time_ - millis_) / 1000.0 * cos(ori_z_);
        pos_y_ = pos_y_ - vel_lin_x_ * (tmp_time_ - millis_) / 1000.0 * sin(ori_z_);

        millis_ = tmp_time_;
    }
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
