#include "sgd_safety/obstacle_checker.hpp"

namespace sgd_safety
{

ObstacleChecker::ObstacleChecker()
{
}

ObstacleChecker::~ObstacleChecker()
{
}

bool
ObstacleChecker::initialize(double robot_width, double distance_min, double distance_max)
{
    if (distance_min > distance_max)
    {
        throw std::invalid_argument("Maximum safety distance is smaller than minimum safety distance.");
    }
    robot_width_ = robot_width;
    distance_min_ = distance_min;
    distance_max_ = distance_max;
    half_width = (robot_width_ / 2) + width_tolerance;

    is_initialized = true;
    return is_initialized;
}

double
ObstacleChecker::compute_speed(std::vector<float> scan_ranges, float min_angle, float angle_increment)
{
    if (!is_initialized)
    {
        return 0.0;
    }

    double angle_ = min_angle;
    double speed = 0.0;

    double distance_nearest_obstacle = distance_max_ + 0.1;
    int obstacle_ocurrences = 0;  // a counter to verify obstacle occurrences

    /* Evaluates the distances the Lidar measures at every angle */
    for (double current_distance : scan_ranges)
    {
        /* If the angle is too big, it is detecting its own body. If the distance is too small, it is a false measurement regarding an issue with the lidar sensor.*/
        if ((abs(angle_) > workspace_lower_limit) || (current_distance < 0.009))
        {
        } // Discard this measurements because it's detecting itself.
        /* If there is an object outside the workspace angle, the distance of such objects must be verified to avoid collisions*/
        else if (abs(angle_) > atan2(half_width, distance_min_))
        {
            /* This equation determines if there is enough room for the robot to pass without colliding */
            if (current_distance < abs(half_width / sin(angle_)))
            {
                obstacle_ocurrences++;
                if (obstacle_ocurrences > min_ocurrences) // If certain number of measurements confirmed an obstacle.
                {
                    distance_nearest_obstacle = current_distance;
                    speed = 0.0;
                }
            }
            else
            {
                obstacle_ocurrences = 0; // Reset counter
            }
        }
        else /* If there are no obstacles on the sides that can lead to a collision, then the speed is determined by the nearest obstacle. */
        {
            if (current_distance < distance_nearest_obstacle)
            {
                distance_nearest_obstacle = current_distance;
            }
            /* This equation calculates the speed at which the robot should move as a function of the distance between the robot and the object. */
            speed = ((distance_nearest_obstacle - distance_min_) / (distance_max_ - distance_min_));
            if (speed < 0)
                speed = 0;
            else if (speed > 1) // maximum speed is 100%
                speed = 1;
        }
        angle_ += angle_increment;
    }

    return speed;
}

} // namespace sgd_safety