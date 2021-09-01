#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <limits>

namespace nav_sgd
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

    void init();
public:
    PID_Controller(double kp, double ki, double kd);
    PID_Controller(double kp);
    ~PID_Controller();

    void set_reference(double reference);
    void set_max(double max);
    void set_min(double min);
    double next(double current_value);
    double next(double current_value, double interval);
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
PID_Controller::next(double current_value)
{
    return next(current_value, 0.0);
}

double
PID_Controller::next(double current_value, double interval)
{   
    if (ref_value_ == 0.0 && current_value == 0.0)
    {
        return 0;
    }

    integral += (ref_value_ - current_value) * interval;
    //double r = last_val_ + (ref_value_ - current_value) * kp_ + integral * ki_;
    double r = current_value + (ref_value_ - current_value) * kp_ + integral * ki_;
    
    if (r > max_) r = max_;
    if (r < min_) r = min_;
    
    //last_val_ = r;
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

}       // namespace nav_sgd


#endif