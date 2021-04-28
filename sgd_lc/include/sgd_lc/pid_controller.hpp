#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <limits>
#include "rclcpp/rclcpp.hpp"

namespace sgd_lc
{

class PID_Controller
{
private:
    const double kp_;
    const double ki_;
    const double kd_;
    double ref_value_;
    double max_, min_;
    double integral;
    double last_val_;

    rclcpp::Time last_t_;

    void init();
public:
    PID_Controller(double kp, double ki, double kd);
    PID_Controller(double kp);
    ~PID_Controller();

    void set_reference(double reference);
    void set_max(double max);
    void set_min(double min);
    double next(double actual_value);
    double next(double actual_value, rclcpp::Time time);
};

PID_Controller::PID_Controller(double kp)
    : kp_(kp), ki_(0), kd_(0)
{
    init();
}

PID_Controller::PID_Controller(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
    init();
}

PID_Controller::~PID_Controller()
{

}

void
PID_Controller::set_reference(double reference) {ref_value_ = reference;}

void
PID_Controller::set_max(double max) {max_ = max;}

void
PID_Controller::set_min(double min) {min_ = min;}

double
PID_Controller::next(double actual_value)
{
    return next(actual_value, rclcpp::Time(0));
}

double
PID_Controller::next(double actual_value, rclcpp::Time time)
{   
    if (ref_value_ == 0.0 && actual_value == 0.0)
    {
        return 0;
    }

    integral += (ref_value_ - actual_value) * rclcpp::Duration(time - last_t_).seconds();
    double r = last_val_ + (ref_value_ - actual_value) * kp_ + integral * ki_;
    last_t_ = time;

    if (r > max_) r = max_;
    if (r < min_) r = min_;
    
    last_val_ = r;
    return r;
}

void
PID_Controller::init()
{
    ref_value_ = 0.0;
    integral = 0.0;
    last_val_ = 0.0;
    max_ = std::numeric_limits<double>::max();
    min_ = -max_;
}

}       // namespace sgd_lc


#endif