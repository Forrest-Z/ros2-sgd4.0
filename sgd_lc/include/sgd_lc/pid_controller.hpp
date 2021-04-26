#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <limits>

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

    void init();
public:
    PID_Controller(double kp, double ki, double kd);
    PID_Controller(double kp);
    ~PID_Controller();

    void set_reference(double reference);
    void set_max(double max);
    void set_min(double min);
    double next(double actual_value);
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
    //integral += (ref_value_ - actual_value) * 0.01;     // TODO Zeitschritt
    double r = (ref_value_ - actual_value) * kp_;
    
    if (r > max_) return max_;
    if (r < min_) return min_;
    return r;
}

void
PID_Controller::init()
{
    ref_value_ = 0.0;
    integral = 0.0;
    max_ = std::numeric_limits<double>::max();
    min_ = -max_;
}

}       // namespace sgd_lc


#endif