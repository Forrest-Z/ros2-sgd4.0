#ifndef SGD_HARDWARE__VL53L1X_HPP_
#define SGD_HARDWARE__VL53L1X_HPP_

#include <algorithm>
#include "nlohmann/json.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_hardware_drivers
{
 
class VL53L1X
{
public:
    VL53L1X();
    ~VL53L1X();

    /**
     * @brief Initialize sensor and set minimum and maximum velocity
     * 
     * @param goal_dist goal distance in mm
     * @param max_vel max velocity in percent
     * @param min_vel min velocity in percent
     */
    void init(int goal_dist, double max_vel, double min_vel);

    /**
     * @brief 
     * 
     * @param msg 
     * @return int 0 if message could not be parsed, otherwise 1
     */
    int parse_msg(std::string msg);

    /**
     * @brief Get the velocity in percent
     * 
     * @return velocity as double in the range [0.0, 1.0]
     */
    double get_vel_p();

protected:
    // variables to store sensor data
    int range_mm_ = 0;
    int status_ = 0;
    double peak_ = 0.0;
    double ambient_ = 0.0;

    double kp_ = 1.0/400.0;

    // parameters
    int goal_dist_;
    double b_, m_, max_vel_, min_vel_;
};

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

}

#endif
