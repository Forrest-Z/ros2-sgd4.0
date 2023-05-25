#include "feather_handle/vl53l1x.hpp"

namespace sgd_hardware_drivers
{

VL53L1X::VL53L1X()
{
    init(350, 1.0, 0.0);
}

VL53L1X::~VL53L1X() {}

void
VL53L1X::init(int goal_dist, double max_vel, double min_vel)
{
    max_vel_ = max_vel;
    min_vel_ = min_vel;
    goal_dist_ = goal_dist;
}

int
VL53L1X::parse_msg(std::string msg)
{
    // check if temp or acc message
    auto js = nlohmann::json::parse(msg);
    auto& s_id = js["s_id"];

    if (s_id == "laser1d")
    {
        // message contains acceleration, gyro, heading, etc.
        if (js.count("range"))      range_mm_ = round((range_mm_*4+js["range"].get<int>())/5);
        if (js.count("status"))     status_ = js["status"].get<int>();
        if (js.count("peak"))       peak_ = js["peak"].get<double>();
        if (js.count("amb"))        ambient_ = js["amb"].get<double>();

        return 1;
    }
    return 0;
}

double
VL53L1X::get_vel_p()
{
    if (peak_ < 1.0)    return 0.0;

    int soll_ist = goal_dist_ - range_mm_;
    double vel_ = std::min(std::max(soll_ist*kp_+1.0, min_vel_), max_vel_);

    return vel_;
}
    
} // namespace sgd_hardware_drivers
