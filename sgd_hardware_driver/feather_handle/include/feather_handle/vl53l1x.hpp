#ifndef SGD_HARDWARE__VL53L1X_HPP_
#define SGD_HARDWARE__VL53L1X_HPP_

#include "nlohmann/json.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_hardware_drivers
{
 
class VL53L1X
{
public:
    VL53L1X(const std::string log_dir);
    ~VL53L1X();

    void init(double max_range, double min_range);

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

    // parameters
    double b_, m_, max_range_, min_range_;
};

VL53L1X::VL53L1X(std::string log_dir)
{
    time_t now = time(0);
    tm *ltm = localtime(&now);

    char buf[32];
    std::sprintf(&buf[0], "vl53l1x_%4d-%2d-%2d_%2d-%2d-%2d.log", 1900+ltm->tm_year, 1+ltm->tm_mon, ltm->tm_mday,
                            ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    std::string log_file_(buf);
    log_file_ = log_dir + log_file_;
    plog::init(plog::info, log_file_.c_str());

    init(0.3, 0.8);
}

VL53L1X::~VL53L1X() {}

void
VL53L1X::init(double min_range, double max_range)
{
    max_range_ = max_range;
    min_range_ = min_range;

    m_ = -1.0 / (max_range - min_range);
    b_ = -m_ * max_range;
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
        if (js.count("range"))      range_mm_ = round((range_mm_*9+js["range"].get<int>())/10);
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

    double vel_ = range_mm_/1000.0 * m_ + b_;
    return (range_mm_/1000.0 > max_range_ ? 0.0 : (range_mm_/1000.0 < min_range_ ? 1.0 : vel_));
}

}

#endif
