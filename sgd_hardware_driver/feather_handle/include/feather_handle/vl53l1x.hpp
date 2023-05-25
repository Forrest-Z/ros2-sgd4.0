#ifndef SGD_HARDWARE__VL53L1X_HPP_
#define SGD_HARDWARE__VL53L1X_HPP_

#include <algorithm>
#include "nlohmann/json.hpp"

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

}

#endif
