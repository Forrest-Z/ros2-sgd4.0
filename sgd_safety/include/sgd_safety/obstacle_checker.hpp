#ifndef SGD_SAFETY__OBSTACLE_CHECKER_HPP_
#define SGD_SAFETY__OBSTACLE_CHECKER_HPP_

#include <vector>
#include <cmath>
#include <stdexcept>

namespace sgd_safety
{

class ObstacleChecker
{
private:
    // Parameters
    bool is_initialized = false;

    double robot_width_;         // Width of the robot in m including the wheels  //FIXME Approximate measurement. Verify!
    double distance_min_;        // Minimal distance the robot has to keep with respect to obstacles
    double distance_max_;        // At any distance smaller than this, the robot should start decreasing its speed.

    const float width_tolerance = 0.1;      // A tolerance on the robot width in m for an extra safety factor.
    const int min_ocurrences = 6;           // minimum number of ocurrences before it can be considered an obstacle
    const float workspace_lower_limit = 2;  // Variable to disregard the robot's body as an obstacle
    
    float half_width;       // Distance from the center of the robot to the wheel plus a tolerance.
    
public:
    ObstacleChecker();
    ~ObstacleChecker();

    /**
     * @brief Initialize variables used for speed computation.
     * 
     * @param robot_width 
     * @param distance_min 
     * @param distance_max 
     * @throw std::invalid_argument if the arguments can not be accepted
     * @return true if initialization was successful
     */
    bool initialize(double robot_width, double distance_min, double distance_max);

    /**
     * @brief Calculate speed based on laser scan and distances to nearest obstacles.
     * 
     * @param scan_ranges 
     * @param min_angle minimum angle for measurement
     * @param angle_increment angle difference between two measured points
     * @return speed in percent
     */
    double compute_speed(std::vector<float> scan_ranges, float min_angle, float angle_increment);
};

} // namespace sgd_safety

#endif